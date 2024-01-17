use std::ops::Div;

use robot_rs_units::{Ticks, Angle, Length, motion::{TickVelocity, AngularVelocity, Velocity}, radian};

use super::{Transform, ReversibleTransform};

#[derive(Clone, Debug)]
pub struct EncoderToAngular {
  pub factor: <Angle as Div<Ticks>>::Output
}

impl EncoderToAngular {
  pub fn new(factor: <Angle as Div<Ticks>>::Output) -> Self {
    Self { factor }
  }
}

impl Transform<Ticks> for EncoderToAngular {
  type Output = Angle;

  fn calculate(&self, input: Ticks) -> Self::Output {
    input * self.factor
  }
}

impl Transform<TickVelocity> for EncoderToAngular {
  type Output = AngularVelocity;

  fn calculate(&self, input: TickVelocity) -> Self::Output {
    input * self.factor
  }
}

impl ReversibleTransform<Ticks> for EncoderToAngular {
  fn calculate_reverse(&self, output: <Self as Transform<Ticks>>::Output) -> Ticks {
    output / self.factor
  }
}

impl ReversibleTransform<TickVelocity> for EncoderToAngular {
  fn calculate_reverse(&self, output: <Self as Transform<TickVelocity>>::Output) -> TickVelocity {
    output / self.factor
  }
}

#[derive(Clone, Debug)]
pub struct AngularToLinear {
  pub radius: Length
}

impl AngularToLinear {
  pub fn new(radius: Length) -> Self {
    Self { radius }
  }
}

impl Transform<Angle> for AngularToLinear {
  type Output = Length;

  fn calculate(&self, input: Angle) -> Self::Output {
    input / (1.0 * radian) * self.radius
  }
}

impl Transform<AngularVelocity> for AngularToLinear {
  type Output = Velocity;

  fn calculate(&self, input: AngularVelocity) -> Self::Output {
    input / (1.0 * radian) * self.radius
  }
}

impl ReversibleTransform<Angle> for AngularToLinear {
  fn calculate_reverse(&self, output: <Self as Transform<Angle>>::Output) -> Angle {
    (output / self.radius) * (1.0 * radian)
  }
}

impl ReversibleTransform<AngularVelocity> for AngularToLinear {
  fn calculate_reverse(&self, output: <Self as Transform<AngularVelocity>>::Output) -> AngularVelocity {
    (output / self.radius) * (1.0 * radian)
  }
}