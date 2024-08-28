use robot_rs_units::{
  electrical::Voltage, motion::AngularVelocity, traits::MaybeUnitNumber, Current, Time,
};

use crate::{
  physics::motor::{MotorCurrentDynamics, MotorForwardDynamics},
  sensors::{AngularVelocitySensor, StatefulAngularVelocitySensor},
};

use super::{StatefulTransform, Transform};

#[derive(Clone)]
pub struct CurrentLimitTransform<Motor, Sensor> {
  pub motor: Motor,
  pub sensor: Sensor,
  pub lim: Current,
}

impl<Motor, Sensor> CurrentLimitTransform<Motor, Sensor> {
  pub fn new(lim: Current, sensor: Sensor, motor: Motor) -> Self {
    Self { lim, sensor, motor }
  }
}

impl<
    Motor: MotorCurrentDynamics + MotorForwardDynamics,
    Sensor: crate::sensors::Sensor<AngularVelocity>,
  > Transform<Voltage> for CurrentLimitTransform<Motor, Sensor>
{
  type Output = Voltage;

  fn calculate(&self, input: Voltage) -> Self::Output {
    let speed = self.sensor.get_angular_velocity();
    let v_min = self
      .motor
      .voltage(self.motor.torque_from_current(-self.lim), speed);
    let v_max = self
      .motor
      .voltage(self.motor.torque_from_current(self.lim), speed);

    input.max(v_min).min(v_max)
  }
}

impl<
    Motor: MotorCurrentDynamics + MotorForwardDynamics,
    Sensor: crate::sensors::StatefulSensor<AngularVelocity>,
  > StatefulTransform<Voltage, Time> for CurrentLimitTransform<Motor, Sensor>
{
  type Output = Voltage;

  fn calculate(&mut self, input: Voltage, _time: Time) -> Self::Output {
    let speed = self.sensor.get_angular_velocity();
    let v_min = self
      .motor
      .voltage(self.motor.torque_from_current(-self.lim), speed);
    let v_max = self
      .motor
      .voltage(self.motor.torque_from_current(self.lim), speed);

    input.max(v_min).min(v_max)
  }
}
