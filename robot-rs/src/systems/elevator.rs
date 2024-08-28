use std::{
  sync::{Arc, RwLock},
  time::Duration,
};

use num_traits::Zero;
use robot_rs_units::{
  electrical::{volt, Voltage},
  meter, millisecond,
  motion::{meters_per_second, meters_per_second2},
  traits::Angle as _,
  Angle, Length, Mass, QuantityBase, Time,
};

use crate::{
  actuators::VoltageActuator,
  physics::motor::SpooledMotorForwardDynamics,
  sensors::{StatefulBinarySensor, StatefulDisplacementSensor},
  time::now,
  transforms::{HasSetpoint, StatefulTransform},
};

pub trait GenericElevator {
  fn go_idle(&mut self);
  fn go_disabled(&mut self);
  fn go_to_height(&mut self, height: Length);

  fn is_stable(&self) -> bool;
}

#[async_trait::async_trait]
pub trait AwaitableElevator: GenericElevator {
  async fn go_to_height_wait(&mut self, height: Length);
  async fn wait_for_stable(&self);
}

#[derive(Debug, Clone)]
pub enum ElevatorDemand {
  Disabled,
  Idle,
  Height { height: Length },
  Manual { voltage: Voltage },
}

#[derive(Debug, Clone)]
pub enum ElevatorStateMode {
  Disabled,
  Idle,
  Manual,
  Limited,
  Stable,
  Moving,
}

#[derive(Debug, Clone)]
pub struct ElevatorState {
  pub height: Length,
  pub applied_voltage: Voltage,
  pub mode: ElevatorStateMode,
}

#[derive(Clone, Debug)]
pub struct ElevatorFrontend {
  demand: Arc<RwLock<(ElevatorDemand, /* is it new? */ bool)>>,
  state: Arc<RwLock<ElevatorState>>,
}

#[derive(Debug, Clone, Copy)]
pub struct ElevatorParams {
  pub angle_from_horizon: Angle,
  pub carriage_mass: Mass,
  pub limits: (Length, Length),
}

pub trait Controller:
  StatefulTransform<Length, Time, Output = Voltage> + HasSetpoint<Length>
{
}
impl<T: StatefulTransform<Length, Time, Output = Voltage> + HasSetpoint<Length>> Controller for T {}

pub struct Elevator {
  params: ElevatorParams,

  actuator: Box<dyn VoltageActuator + Send + Sync>,
  motor_model: Box<dyn SpooledMotorForwardDynamics + Send + Sync>,
  height_sensor: Box<dyn StatefulDisplacementSensor + Send + Sync>,
  limit_switches: (
    Option<Box<dyn StatefulBinarySensor + Send + Sync>>,
    Option<Box<dyn StatefulBinarySensor + Send + Sync>>,
  ),

  controller: Box<dyn Controller + Send + Sync>,
  stability_filter: Box<dyn StatefulTransform<Length, Time, Output = bool> + Send + Sync>,

  frontend: ElevatorFrontend,
}

impl Elevator {
  pub fn new(
    params: ElevatorParams,
    actuator: Box<dyn VoltageActuator + Send + Sync>,
    motor_model: Box<dyn SpooledMotorForwardDynamics + Send + Sync>,
    height_sensor: Box<dyn StatefulDisplacementSensor + Send + Sync>,
    limit_switches: (
      Option<Box<dyn StatefulBinarySensor + Send + Sync>>,
      Option<Box<dyn StatefulBinarySensor + Send + Sync>>,
    ),
    controller: Box<dyn Controller + Send + Sync>,
    stability_filter: Box<dyn StatefulTransform<Length, Time, Output = bool> + Send + Sync>,
  ) -> Self {
    Self {
      params,
      actuator,
      motor_model,
      height_sensor,
      limit_switches,
      controller,
      stability_filter,
      frontend: ElevatorFrontend {
        demand: Arc::new(RwLock::new((ElevatorDemand::Disabled, true))),
        state: Arc::new(RwLock::new(ElevatorState {
          height: 0.0 * meter,
          applied_voltage: 0.0 * volt,
          mode: ElevatorStateMode::Disabled,
        })),
      },
    }
  }

  pub fn frontend(&self) -> ElevatorFrontend {
    self.frontend.clone()
  }

  pub fn tick(&mut self, time: Time) {
    let current_height = self.height_sensor.get_displacement();
    let feedforward = self.motor_model.voltage(
      -9.81 * self.params.angle_from_horizon.sin() * meters_per_second2 * self.params.carriage_mass,
      0.0 * meters_per_second,
    );

    let limits_hit = (
      self
        .limit_switches
        .0
        .as_mut()
        .map(|x| x.get_sensor_value())
        .unwrap_or(false),
      self
        .limit_switches
        .1
        .as_mut()
        .map(|x| x.get_sensor_value())
        .unwrap_or(false),
    );

    let current_demand = self.frontend.demand.read().unwrap().clone();

    let (mut demand_voltage, mut state_mode) = match current_demand {
      (ElevatorDemand::Disabled, _) => (Zero::zero(), ElevatorStateMode::Disabled),
      (ElevatorDemand::Idle, _) => (Zero::zero(), ElevatorStateMode::Idle),
      (ElevatorDemand::Manual { voltage }, _) => (voltage, ElevatorStateMode::Manual),
      (ElevatorDemand::Height { height: target }, is_new_demand) => {
        self.controller.set_setpoint(target);
        if is_new_demand {
          self.stability_filter.reset();
          self.controller.reset();
          self.frontend.demand.write().unwrap().1 = false;
        }

        let state = if self
          .stability_filter
          .calculate(target - current_height, time)
        {
          ElevatorStateMode::Stable
        } else {
          ElevatorStateMode::Moving
        };

        let control_output = self.controller.calculate(current_height, time);
        (control_output, state)
      }
    };

    match (limits_hit, demand_voltage) {
      ((true, false), voltage) if voltage < Zero::zero() => {
        demand_voltage = Zero::zero();
        state_mode = ElevatorStateMode::Limited;
      }
      ((false, true), voltage) if voltage > feedforward => {
        demand_voltage = feedforward;
        state_mode = ElevatorStateMode::Limited;
      }
      _ => (),
    }

    self.actuator.set_actuator_value(demand_voltage, time);
    *self.frontend.state.write().unwrap() = ElevatorState {
      height: current_height,
      applied_voltage: demand_voltage,
      mode: state_mode,
    };
  }

  pub async fn run_async(mut self, period: Time) {
    loop {
      self.tick(now());
      tokio::time::sleep(Duration::from_millis(period.to::<millisecond>() as u64)).await;
    }
  }
}

impl GenericElevator for Elevator {
  fn go_idle(&mut self) {
    *self.frontend.demand.write().unwrap() = (ElevatorDemand::Idle, true);
    self.tick(now());
  }

  fn go_disabled(&mut self) {
    *self.frontend.demand.write().unwrap() = (ElevatorDemand::Disabled, true);
    self.tick(now());
  }

  fn go_to_height(&mut self, height: Length) {
    *self.frontend.demand.write().unwrap() = (ElevatorDemand::Height { height }, true);
    self.tick(now());
  }

  fn is_stable(&self) -> bool {
    matches!(
      self.frontend.state.read().unwrap().mode,
      ElevatorStateMode::Stable
    )
  }
}

impl GenericElevator for ElevatorFrontend {
  fn go_idle(&mut self) {
    *self.demand.write().unwrap() = (ElevatorDemand::Idle, true);
  }

  fn go_disabled(&mut self) {
    *self.demand.write().unwrap() = (ElevatorDemand::Disabled, true);
  }

  fn go_to_height(&mut self, height: Length) {
    *self.demand.write().unwrap() = (ElevatorDemand::Height { height }, true);
  }

  fn is_stable(&self) -> bool {
    matches!(self.state.read().unwrap().mode, ElevatorStateMode::Stable)
  }
}

#[async_trait::async_trait]
impl AwaitableElevator for ElevatorFrontend {
  async fn wait_for_stable(&self) {
    // We could make this more efficient by using channels, but busy waiting may actually be more efficient.
    while !self.is_stable() {
      tokio::time::sleep(Duration::from_millis(5)).await;
    }
  }

  async fn go_to_height_wait(&mut self, height: Length) {
    self.go_to_height(height);
    self.wait_for_stable().await;
  }
}

#[cfg(feature = "simulation")]
pub mod sim {
  use std::time::Duration;

  use ntcore_rs::GenericPublisher;
  use num_traits::Zero;
  use robot_rs_units::{
    ampere,
    electrical::{volt, Voltage},
    force::newton,
    meter, millisecond,
    motion::{meters_per_second, meters_per_second2, Velocity},
    traits::{Angle as _, MaybeUnitNumber},
    Length, QuantityBase, Time,
  };

  use crate::{
    actuators::sim::SimActuator, physics::motor::SpooledMotorDynamics, sensors::sim::SimSensor,
    time::now,
  };

  use super::ElevatorParams;

  pub struct ElevatorSim {
    params: ElevatorParams,

    actuator: Box<dyn SimActuator<Voltage, Time> + Send + Sync>,
    motor_model: Box<dyn SpooledMotorDynamics + Send + Sync>,
    height_sensor: Box<dyn SimSensor<Length> + Send + Sync>,
    limit_switches: (
      Option<Box<dyn SimSensor<bool> + Send + Sync>>,
      Option<Box<dyn SimSensor<bool> + Send + Sync>>,
    ),

    last_tick: Option<Time>,
    speed: Velocity,

    pub_demand: ntcore_rs::Publisher<f64>,
    pub_force: ntcore_rs::Publisher<f64>,
    pub_current: ntcore_rs::Publisher<f64>,
    pub_velocity: ntcore_rs::Publisher<f64>,
    pub_height: ntcore_rs::Publisher<f64>,
  }

  impl ElevatorSim {
    pub fn new(
      params: ElevatorParams,
      actuator: Box<dyn SimActuator<Voltage, Time> + Send + Sync>,
      motor_model: Box<dyn SpooledMotorDynamics + Send + Sync>,
      height_sensor: Box<dyn SimSensor<Length> + Send + Sync>,
      limit_switches: (
        Option<Box<dyn SimSensor<bool> + Send + Sync>>,
        Option<Box<dyn SimSensor<bool> + Send + Sync>>,
      ),
      topic: ntcore_rs::Topic,
    ) -> Self {
      Self {
        params,
        actuator,
        motor_model,
        height_sensor,
        limit_switches,
        last_tick: None,
        speed: Zero::zero(),
        pub_demand: topic.child("demand").publish(),
        pub_force: topic.child("force").publish(),
        pub_current: topic.child("current").publish(),
        pub_velocity: topic.child("velocity").publish(),
        pub_height: topic.child("height").publish(),
      }
    }

    pub fn tick(&mut self, time: Time) {
      if let Some(last_time) = self.last_tick {
        let dt = time - last_time;
        let (demand_volts, _demand_time) = self.actuator.get_actuator_value();

        let force = self.motor_model.force(demand_volts, self.speed);
        let current_draw = self.motor_model.current(force);

        let net_force = force
          - 9.81
            * meters_per_second2
            * self.params.carriage_mass
            * self.params.angle_from_horizon.sin();
        let accel = net_force / self.params.carriage_mass;
        let max_speed = self.motor_model.velocity(demand_volts, 0.0 * newton);
        self.speed += accel * dt;
        self.speed = self.speed.max(-max_speed).min(max_speed);

        let current_displacement = self.height_sensor.get_sensor_value();
        let mut new_displacement = current_displacement + self.speed * dt;
        let mut limit_triggered = (false, false);

        if new_displacement < self.params.limits.0 {
          let actual_speed = (self.params.limits.0 - current_displacement) / dt;
          self.speed = actual_speed;
          new_displacement = self.params.limits.0;
          limit_triggered.0 = true;
        } else if new_displacement > self.params.limits.1 {
          let actual_speed = (self.params.limits.1 - current_displacement) / dt;
          self.speed = actual_speed;
          new_displacement = self.params.limits.1;
          limit_triggered.1 = true;
        }

        if let Some(sw) = self.limit_switches.0.as_mut() {
          sw.set_sensor_value(limit_triggered.0, time)
        }
        if let Some(sw) = self.limit_switches.1.as_mut() {
          sw.set_sensor_value(limit_triggered.1, time)
        }

        self.pub_demand.set(demand_volts.to::<volt>()).ok();
        self.pub_force.set(force.to::<newton>()).ok();
        self.pub_current.set(current_draw.to::<ampere>()).ok();
        self
          .pub_velocity
          .set(self.speed.to::<meters_per_second>())
          .ok();
        self.pub_height.set(new_displacement.to::<meter>()).ok();

        self.height_sensor.set_sensor_value(new_displacement, time);
      }
      self.last_tick = Some(time);
    }

    pub async fn run(&mut self, period: Time) {
      loop {
        self.tick(now());
        tokio::time::sleep(Duration::from_millis(period.to::<millisecond>() as u64)).await;
      }
    }
  }
}
