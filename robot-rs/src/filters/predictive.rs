use robot_rs_units::{Current, electrical::Voltage, motion::{AngularVelocity, Velocity}, traits::MaybeUnitNumber};

use crate::physics::motor::{MotorCurrentDynamics, MotorForwardDynamics, SpooledMotorCurrentDynamics, SpooledMotorForwardDynamics};

use super::Filter;

pub struct CurrentLimitFilter<Motor> {
  pub motor: Motor,
  pub current_min: Current,
  pub current_max: Current,
}

impl<Motor> CurrentLimitFilter<Motor> {
  pub fn new(current_min: Current, current_max: Current, motor: Motor) -> Self {
    Self { current_min, current_max, motor }
  }
}

impl<
  Motor: MotorCurrentDynamics + MotorForwardDynamics
> Filter<(Voltage, AngularVelocity)> for CurrentLimitFilter<Motor> {
  type Output = Voltage;

  fn calculate(&mut self, input: (Voltage, AngularVelocity)) -> Self::Output {
    let v_min = self.motor.voltage(self.motor.torque_from_current(self.current_min), input.1);
    let v_max = self.motor.voltage(self.motor.torque_from_current(self.current_max), input.1);

    input.0.max(v_min).min(v_max)
  }

  fn reset(&mut self) { }
}

impl<
  Motor: SpooledMotorCurrentDynamics + SpooledMotorForwardDynamics
> Filter<(Voltage, Velocity)> for CurrentLimitFilter<Motor> {
  type Output = Voltage;

  fn calculate(&mut self, input: (Voltage, Velocity)) -> Self::Output {
    let v_min = self.motor.voltage(self.motor.force_from_current(self.current_min), input.1);
    let v_max = self.motor.voltage(self.motor.force_from_current(self.current_max), input.1);

    input.0.max(v_min).min(v_max)
  }

  fn reset(&mut self) { }
}
