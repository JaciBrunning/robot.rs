use std::{sync::{Arc, RwLock}, time::Duration};

use num_traits::Zero;
use robot_rs_units::{Angle, electrical::{Voltage, volt}, Mass, Length, Time, radian, millisecond, motion::{rads_per_second, meters_per_second2}, traits::Angle as _, QuantityBase as _};

use crate::{transforms::{HasSetpoint, StatefulTransform}, actuators::VoltageActuator, physics::motor::MotorForwardDynamics, sensors::{StatefulAngularSensor, StatefulBinarySensor}, time::now};

pub trait GenericArm {
  fn go_idle(&mut self);
  fn go_disabled(&mut self);
  fn go_to_angle(&mut self, angle: Angle);

  fn is_stable(&self) -> bool;
}

#[async_trait::async_trait]
pub trait AwaitableArm : GenericArm {
  async fn go_to_angle_wait(&mut self, angle: Angle);
  async fn wait_for_stable(&self);
}

#[derive(Debug, Clone)]
pub enum ArmDemand {
  Disabled,
  Idle,
  Angle { angle: Angle },
  Manual { voltage: Voltage }
}

#[derive(Debug, Clone)]
pub enum ArmStateMode {
  Disabled,
  Idle,
  Manual,
  Limited,
  Stable,
  Moving
}

#[derive(Debug, Clone)]
pub struct ArmState {
  pub angle: Angle,
  pub applied_voltage: Voltage,
  pub mode: ArmStateMode
}

#[derive(Clone, Debug)]
pub struct ArmFrontend {
  demand: Arc<RwLock<(ArmDemand, /* is it new? */ bool)>>,
  state: Arc<RwLock<ArmState>>,
}

#[derive(Debug, Clone, Copy)]
pub struct ArmParams {
  pub effective_mass: Mass,
  pub center_of_mass_length: Length,
  pub limits: (Angle, Angle)
}

pub trait Controller : StatefulTransform<Angle, Time, Output=Voltage> + HasSetpoint<Angle> {}
impl<T: StatefulTransform<Angle, Time, Output=Voltage> + HasSetpoint<Angle>> Controller for T {}

pub struct Arm {
  params: ArmParams,

  actuator: Box<dyn VoltageActuator + Send + Sync>,
  motor_model: Box<dyn MotorForwardDynamics + Send + Sync>,
  angle_sensor: Box<dyn StatefulAngularSensor + Send + Sync>,
  limit_switches: (Option<Box<dyn StatefulBinarySensor + Send + Sync>>, Option<Box<dyn StatefulBinarySensor + Send + Sync>>),
  
  controller: Box<dyn Controller + Send + Sync>,
  stability_filter: Box<dyn StatefulTransform<Angle, Time, Output=bool> + Send + Sync>,

  frontend: ArmFrontend
}

impl Arm {
  pub fn new(
    params: ArmParams,
    actuator: Box<dyn VoltageActuator + Send + Sync>,
    motor_model: Box<dyn MotorForwardDynamics + Send + Sync>,
    angle_sensor: Box<dyn StatefulAngularSensor + Send + Sync>,
    limit_switches: (Option<Box<dyn StatefulBinarySensor + Send + Sync>>, Option<Box<dyn StatefulBinarySensor + Send + Sync>>),
    controller: Box<dyn Controller + Send + Sync>,
    stability_filter: Box<dyn StatefulTransform<Angle, Time, Output=bool> + Send + Sync>
  ) -> Self {
    Self {
      params,
      actuator, motor_model, angle_sensor, 
      limit_switches,
      controller, stability_filter,
      frontend: ArmFrontend {
        demand: Arc::new(RwLock::new((ArmDemand::Disabled, true))),
        state: Arc::new(RwLock::new(ArmState { angle: 0.0 * radian, applied_voltage: 0.0 * volt, mode: ArmStateMode::Disabled }))
      }
    }
  }

  pub fn frontend(&self) -> ArmFrontend {
    self.frontend.clone()
  }

  pub fn tick(&mut self, time: Time) {
    let current_angle = self.angle_sensor.get_angle();
    let feedforward = self.motor_model.voltage(-9.81 * meters_per_second2 * current_angle.cos() * self.params.effective_mass * self.params.center_of_mass_length, 0.0 * rads_per_second);

    let limits_hit = (
      self.limit_switches.0.as_mut().map(|x| x.get_sensor_value()).unwrap_or(false),
      self.limit_switches.1.as_mut().map(|x| x.get_sensor_value()).unwrap_or(false)
    );

    let current_demand = self.frontend.demand.read().unwrap().clone();

    let (mut demand_voltage, mut state_mode) = match current_demand {
      (ArmDemand::Disabled, _) => (Zero::zero(), ArmStateMode::Disabled),
      (ArmDemand::Idle, _) => (Zero::zero(), ArmStateMode::Idle),
      (ArmDemand::Manual { voltage }, _) => (voltage, ArmStateMode::Manual),
      (ArmDemand::Angle { angle: target }, is_new_demand) => {
        self.controller.set_setpoint(target);
        if is_new_demand {
          self.stability_filter.reset();
          self.controller.reset();
          self.frontend.demand.write().unwrap().1 = false;
        }
        
        let state = if self.stability_filter.calculate(target - current_angle, time) {
          ArmStateMode::Stable
        } else {
          ArmStateMode::Moving
        };

        let control_output = self.controller.calculate(current_angle, time);
        (control_output, state)
      }
    };

    match (limits_hit, demand_voltage) {
      ((true, false), voltage) if voltage < Zero::zero() => {
        demand_voltage = Zero::zero();
        state_mode = ArmStateMode::Limited;
      },
      ((false, true), voltage) if voltage > feedforward => {
        demand_voltage = feedforward;
        state_mode = ArmStateMode::Limited;
      },
      _ => ()
    }

    self.actuator.set_actuator_value(demand_voltage, time);
    *self.frontend.state.write().unwrap() = ArmState {
      angle: current_angle,
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

impl GenericArm for Arm {
  fn go_idle(&mut self) {
    *self.frontend.demand.write().unwrap() = (ArmDemand::Idle, true);
    self.tick(now());
  }

  fn go_disabled(&mut self) {
    *self.frontend.demand.write().unwrap() = (ArmDemand::Disabled, true);
    self.tick(now());
  }

  fn go_to_angle(&mut self, angle: Angle) {
    *self.frontend.demand.write().unwrap() = (ArmDemand::Angle { angle }, true);
    self.tick(now());
  }

  fn is_stable(&self) -> bool {
    matches!(self.frontend.state.read().unwrap().mode, ArmStateMode::Stable)
  }
}

impl GenericArm for ArmFrontend {
  fn go_idle(&mut self) {
    *self.demand.write().unwrap() = (ArmDemand::Idle, true);
  }

  fn go_disabled(&mut self) {
    *self.demand.write().unwrap() = (ArmDemand::Disabled, true);
  }

  fn go_to_angle(&mut self, angle: Angle) {
    *self.demand.write().unwrap() = (ArmDemand::Angle { angle }, true);
  }

  fn is_stable(&self) -> bool {
    matches!(self.state.read().unwrap().mode, ArmStateMode::Stable)
  }
}

#[async_trait::async_trait]
impl AwaitableArm for ArmFrontend {
  async fn wait_for_stable(&self) {
    // We could make this more efficient by using channels, but busy waiting may actually be more efficient.
    while !self.is_stable() {
      tokio::time::sleep(Duration::from_millis(5)).await;
    }
  }

  async fn go_to_angle_wait(&mut self, angle: Angle) {
    self.go_to_angle(angle);
    self.wait_for_stable().await;
  }
}

#[cfg(feature = "simulation")]
pub mod sim {
  use std::time::Duration;

use ntcore_rs::GenericPublisher as _;
use num_traits::Zero;
use robot_rs_units::{Time, electrical::{Voltage, volt}, Angle, motion::{AngularVelocity, meters_per_second2, degrees_per_second}, millisecond, QuantityBase as _, traits::{Angle as _, MaybeUnitNumber as _}, radian, force::newton_meter, ampere, degree};

  use crate::{actuators::sim::SimActuator, physics::motor::MotorDynamics, sensors::sim::SimSensor, time::now};

  use super::ArmParams;

  pub struct ArmSim {
    params: ArmParams,

    actuator: Box<dyn SimActuator<Voltage, Time> + Send + Sync>,
    motor_model: Box<dyn MotorDynamics + Send + Sync>,
    angle_sensor: Box<dyn SimSensor<Angle> + Send + Sync>,
    limit_switches: (Option<Box<dyn SimSensor<bool> + Send + Sync>>, Option<Box<dyn SimSensor<bool> + Send + Sync>>),

    last_tick: Option<Time>,
    speed: AngularVelocity,

    pub_demand: ntcore_rs::Publisher<f64>,
    pub_torque: ntcore_rs::Publisher<f64>,
    pub_current: ntcore_rs::Publisher<f64>,
    pub_speed: ntcore_rs::Publisher<f64>,
    pub_angle: ntcore_rs::Publisher<f64>,
  }

  impl ArmSim {
    pub fn new(
      params: ArmParams,
      actuator: Box<dyn SimActuator<Voltage, Time> + Send + Sync>,
      motor_model: Box<dyn MotorDynamics + Send + Sync>,
      angle_sensor: Box<dyn SimSensor<Angle> + Send + Sync>,
      limit_switches: (Option<Box<dyn SimSensor<bool> + Send + Sync>>, Option<Box<dyn SimSensor<bool> + Send + Sync>>),
      topic: ntcore_rs::Topic,
    ) -> Self {
      Self {
        params,
        actuator,
        motor_model,
        angle_sensor,
        limit_switches,
        last_tick: None,
        speed: Zero::zero(),
        pub_demand: topic.child("demand").publish(),
        pub_torque: topic.child("torque").publish(),
        pub_current: topic.child("current").publish(),
        pub_speed: topic.child("speed").publish(),
        pub_angle: topic.child("angle").publish(),
      }
    }

    pub fn tick(&mut self, time: Time) {
      if let Some(last_time) = self.last_tick {
        let dt = time - last_time;
        let (demand_volts, _demand_time) = self.actuator.get_actuator_value();
        let current_angle = self.angle_sensor.get_sensor_value();

        let torque = self.motor_model.torque(demand_volts, self.speed);
        let current_draw = self.motor_model.current(torque);

        let torque_down = 9.81 * meters_per_second2 * current_angle.cos() * self.params.effective_mass * self.params.center_of_mass_length;
        let net_torque = torque - torque_down;
        // Assumes point load at radius
        let angular_accel = net_torque / (self.params.effective_mass * self.params.center_of_mass_length * self.params.center_of_mass_length) * (1.0 * radian);
        let max_speed = self.motor_model.speed(demand_volts, 0.0 * newton_meter);

        self.speed += angular_accel * dt;
        self.speed = self.speed.max(-max_speed).min(max_speed);

        let mut new_angle = current_angle + self.speed * dt;
        let mut limit_triggered = (false, false);

        if new_angle < self.params.limits.0 {
          let actual_speed = (self.params.limits.0 - current_angle) / dt;
          self.speed = actual_speed;
          new_angle = self.params.limits.0;
          limit_triggered.0 = true;
        } else if new_angle > self.params.limits.1 {
          let actual_speed = (self.params.limits.1 - current_angle) / dt;
          self.speed = actual_speed;
          new_angle = self.params.limits.1;
          limit_triggered.1 = true;
        }

        if let Some(sw) = self.limit_switches.0.as_mut() { sw.set_sensor_value(limit_triggered.0, time) }
        if let Some(sw) = self.limit_switches.1.as_mut() { sw.set_sensor_value(limit_triggered.1, time) }

        self.pub_demand.set(demand_volts.to::<volt>()).ok();
        self.pub_torque.set(torque.to::<newton_meter>()).ok();
        self.pub_current.set(current_draw.to::<ampere>()).ok();
        self.pub_speed.set(self.speed.to::<degrees_per_second>()).ok();
        self.pub_angle.set(new_angle.to::<degree>()).ok();

        self.angle_sensor.set_sensor_value(new_angle, time);
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