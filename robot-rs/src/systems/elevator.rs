use std::{sync::{RwLock, Arc}, time::Duration};

use num_traits::Zero;
use robot_rs_units::{Length, electrical::Voltage, Time, Angle, Mass, QuantityBase, millisecond, motion::{meters_per_second2, meters_per_second}, traits::{Angle as _, MaybeUnitNumber}};

use crate::{actuators::Actuator, physics::motor::SpooledMotorForwardDynamics, sensors::{StatefulBinarySensor, StatefulDisplacementSensor}, transforms::{StatefulTransform, HasSetpoint}, start::RobotResult, time::now};

pub trait Elevator {
  fn go_idle(&mut self);
  fn go_disabled(&mut self);
  fn go_to_height(&mut self, height: Length) -> RobotResult;

  fn is_stable(&self) -> bool;
}

#[async_trait::async_trait]
pub trait AwaitableElevator {
  async fn wait_for_stable(&self);
}

#[derive(Debug, Clone)]
pub enum ElevatorDemand {
  Disabled,
  Idle,
  Height { height: Length },
  Manual { voltage: Voltage }
}

#[derive(Debug, Clone)]
pub enum ElevatorState {
  Disabled,
  Idle {
    height: Length
  },
  Manual {
    voltage: Voltage
  },
  Limited,
  Stable {
    height: Length,
    demand: Length,
  },
  Moving {
    height: Length,
    demand: Length,
  },
}

#[derive(Clone, Debug)]
pub struct ElevatorFrontend {
  demand: Arc<RwLock<ElevatorDemand>>,
  state: Arc<RwLock<ElevatorState>>,
}

pub trait Controller : StatefulTransform<Length, Time, Output=Voltage> + HasSetpoint<Length> {}
impl<T: StatefulTransform<Length, Time, Output=Voltage> + HasSetpoint<Length>> Controller for T {}

pub struct ElevatorImpl {
  angle_from_horizon: Angle,
  carriage_mass: Mass,
  limits: (Length, Length),
  new_setpoint_thresh: Length,

  actuator: Box<dyn Actuator<Voltage, Time>>,
  motor_model: Box<dyn SpooledMotorForwardDynamics>,
  height_sensor: Box<dyn StatefulDisplacementSensor>,
  limit_switches: (Option<Box<dyn StatefulBinarySensor>>, Option<Box<dyn StatefulBinarySensor>>),

  controller: Box<dyn Controller>,
  stability_filter: Box<dyn StatefulTransform<Length, Time, Output=bool>>,

  frontend: ElevatorFrontend
}

impl ElevatorImpl {
  pub fn new(
    angle_from_horizon: Angle,
    carriage_mass: Mass,
    limits: (Length, Length),
    new_setpoint_thresh: Length,
    actuator: Box<dyn Actuator<Voltage, Time>>,
    motor_model: Box<dyn SpooledMotorForwardDynamics>,
    height_sensor: Box<dyn StatefulDisplacementSensor>,
    limit_switches: (Option<Box<dyn StatefulBinarySensor>>, Option<Box<dyn StatefulBinarySensor>>),
    controller: Box<dyn Controller>,
    stability_filter: Box<dyn StatefulTransform<Length, Time, Output=bool>>
  ) -> Self {
    Self {
      angle_from_horizon,
      carriage_mass,
      limits,
      new_setpoint_thresh,
      actuator, motor_model, height_sensor, 
      limit_switches,
      controller, stability_filter,
      frontend: ElevatorFrontend {
        demand: Arc::new(RwLock::new(ElevatorDemand::Disabled)),
        state: Arc::new(RwLock::new(ElevatorState::Disabled))
      }
    }
  }

  pub fn frontend(&self) -> ElevatorFrontend {
    self.frontend.clone()
  }

  pub fn tick(&mut self, time: Time) {
    let current_height = self.height_sensor.get_displacement();
    let measurement_time = self.height_sensor.get_last_measurement_time();
    let feedforward = self.motor_model.voltage(-9.81 * self.angle_from_horizon.sin() * meters_per_second2 * self.carriage_mass, 0.0 * meters_per_second);

    let limits_hit = (
      self.limit_switches.0.as_mut().map(|x| x.get_sensor_value()).unwrap_or(false),
      self.limit_switches.1.as_mut().map(|x| x.get_sensor_value()).unwrap_or(false),
    );

    let mut demand_voltage = Zero::zero();
    let mut state = self.frontend.state.read().unwrap().clone();
    let current_demand = self.frontend.demand.read().unwrap().clone();

    match current_demand {
      ElevatorDemand::Disabled => {
        state = ElevatorState::Disabled;
        demand_voltage = Zero::zero();
      },
      ElevatorDemand::Idle => {
        state = ElevatorState::Idle { height: current_height };
        demand_voltage = Zero::zero();
      },
      ElevatorDemand::Manual { voltage } => {
        state = ElevatorState::Manual { voltage };
        demand_voltage = voltage;
      },
      ElevatorDemand::Height { height: target } => {
        self.controller.set_setpoint(target);
        match state {
          ElevatorState::Disabled | ElevatorState::Idle { .. } | ElevatorState::Limited | ElevatorState::Manual { .. } => {
            self.controller.reset();
            self.stability_filter.reset();
          },
          ElevatorState::Stable { demand, .. } | ElevatorState::Moving { demand, .. } => {
            // Reset the stability filter so we can detect whether we're stable properly.
            if (demand - target).abs() >= self.new_setpoint_thresh {
              self.stability_filter.reset();
            }
          },
        }

        demand_voltage = self.controller.calculate(current_height, measurement_time);

        if self.stability_filter.calculate(target - current_height, measurement_time) {
          state = ElevatorState::Stable { height: current_height, demand: target }
        } else {
          state = ElevatorState::Moving { height: current_height, demand: target }
        }
      }
    }

    match (limits_hit, demand_voltage) {
      ((true, false), voltage) if voltage < Zero::zero() => {
        demand_voltage = Zero::zero();
        state = ElevatorState::Limited;
      },
      ((false, true), voltage) if voltage > feedforward => {
        demand_voltage = feedforward;
        state = ElevatorState::Limited;
      },
      _ => ()
    }

    self.actuator.set_actuator_value(demand_voltage, time);
    *self.frontend.state.write().unwrap() = state;
  }

  pub async fn run_async(&mut self, period: Time) -> RobotResult {
    loop {
      self.tick(now());
      tokio::time::sleep(Duration::from_millis(period.to::<millisecond>() as u64)).await;
    }
  }
}

impl Elevator for ElevatorImpl {
  fn go_idle(&mut self) {
    *self.frontend.demand.write().unwrap() = ElevatorDemand::Idle;
  }

  fn go_disabled(&mut self) {
    *self.frontend.demand.write().unwrap() = ElevatorDemand::Disabled;
  }

  fn go_to_height(&mut self, height:Length) -> RobotResult {
    if height > self.limits.1 || height < self.limits.0 {
      // TODO: Return error here
    }
    *self.frontend.demand.write().unwrap() = ElevatorDemand::Height { height };
    Ok(())
  }

  fn is_stable(&self) -> bool {
    matches!(*self.frontend.state.read().unwrap(), ElevatorState::Stable { .. })
  }
}

impl Elevator for ElevatorFrontend {
  fn go_idle(&mut self) {
    *self.demand.write().unwrap() = ElevatorDemand::Idle;
  }

  fn go_disabled(&mut self) {
    *self.demand.write().unwrap() = ElevatorDemand::Disabled;
  }

  fn go_to_height(&mut self, height:Length) -> RobotResult {
    //  TODO: Error here if necessary
    *self.demand.write().unwrap() = ElevatorDemand::Height { height };
    Ok(())
  }

  fn is_stable(&self) -> bool {
    matches!(*self.state.read().unwrap(), ElevatorState::Stable { .. })
  }
}

#[async_trait::async_trait]
impl AwaitableElevator for ElevatorFrontend {
  async fn wait_for_stable(&self) {
    // We could make this more efficient by using channels, but busy waiting may actually be more efficient.
    while !self.is_stable() {
      tokio::time::sleep(Duration::from_millis(2)).await;
    }
  }
}

#[cfg(feature = "simulation")]
mod sim {
  // TODO:
}