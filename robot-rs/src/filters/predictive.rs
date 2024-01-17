use robot_rs_units::{Current, electrical::Voltage, motion::AngularVelocity, traits::MaybeUnitNumber};

use crate::{physics::motor::{MotorCurrentDynamics, MotorForwardDynamics}, sensors::AngularVelocitySensor};

use super::Filter;

#[derive(Clone)]
pub struct CurrentLimitFilter<Motor, Sensor> {
  pub motor: Motor,
  pub sensor: Sensor,
  pub current_min: Current,
  pub current_max: Current,
}

impl<Motor, Sensor> CurrentLimitFilter<Motor, Sensor> {
  pub fn new(current_min: Current, current_max: Current, sensor: Sensor, motor: Motor) -> Self {
    Self { current_min, current_max, sensor, motor }
  }
}

impl<
  Motor: MotorCurrentDynamics + MotorForwardDynamics,
  Sensor: crate::sensors::Sensor<AngularVelocity>,
> Filter<Voltage> for CurrentLimitFilter<Motor, Sensor> {
  type Output = Voltage;

  fn calculate(&self, input: Voltage) -> Self::Output {
    let speed = self.sensor.get_angular_velocity();
    let v_min = self.motor.voltage(self.motor.torque_from_current(self.current_min), speed);
    let v_max = self.motor.voltage(self.motor.torque_from_current(self.current_max), speed);

    input.max(v_min).min(v_max)
  }
}
