use std::{ops::Neg, f64::consts::PI};

use crate::{units::*, traits::Wrapper};

pub trait Sensor<U> {
  fn get_sensor_value(&self) -> U;
}

impl<'a, T: Sensor<U>, U> Sensor<U> for &'a T {
  fn get_sensor_value(&self) -> U {
    (**self).get_sensor_value()
  }
}

macro_rules! sensor_alias {
  ($ident:ident, $unit:ty, $fn_name:ident) => {
    pub trait $ident : Sensor<$unit> {
      fn $fn_name(&self) -> $unit { self.get_sensor_value() }
    }
    impl<T: Sensor<$unit>> $ident for T {}
  }
}

sensor_alias!(Encoder, EncoderTicks, get_ticks);
sensor_alias!(VelocityEncoder, EncoderTickVelocity, get_tick_velocity);
sensor_alias!(AngularSensor, Angle, get_angle);
sensor_alias!(AngularVelocitySensor, AngularVelocity, get_angular_velocity);
sensor_alias!(DisplacementSensor, Length, get_displacement);
sensor_alias!(VelocitySensor, Velocity, get_velocity);

#[derive(Clone, Debug)]
pub struct InvertedSensor<T>(pub T);

impl<T> From<T> for InvertedSensor<T> {
  fn from(value: T) -> Self {
    Self(value)
  }
}

impl<T> Wrapper<T> for InvertedSensor<T> {
  fn eject(self) -> T {
    self.0
  }
}

impl<U: Neg<Output = U>, T: Sensor<U>> Sensor<U> for InvertedSensor<T> {
  #[inline(always)]
  fn get_sensor_value(&self) -> U {
    -self.0.get_sensor_value()
  }
}

// CONVERSIONS

#[derive(Clone, Debug)]
pub struct EncoderToAngular<T> {
  pub encoder: T,
  pub ticks_per_rad: EncoderTicks
}

impl<T> EncoderToAngular<T> {
  pub fn new(encoder: T, ticks_per_revolution: EncoderTicks) -> Self {
    Self { encoder, ticks_per_rad: ticks_per_revolution / (2.0 * PI) }
  }
}

impl<T> Wrapper<T> for EncoderToAngular<T> {
  fn eject(self) -> T {
    self.encoder
  }
}

impl<T: Encoder> Sensor<Angle> for EncoderToAngular<T> {
  #[inline(always)]
  fn get_sensor_value(&self) -> Angle {
    let v = self.encoder.get_ticks();
    Angle::new::<angle::radian>((v / self.ticks_per_rad).value)
  }
}

impl<T: VelocityEncoder> Sensor<AngularVelocity> for EncoderToAngular<T> {
  #[inline(always)]
  fn get_sensor_value(&self) -> AngularVelocity {
    let v = self.encoder.get_tick_velocity();
    AngularVelocity::new::<angular_velocity::radian_per_second>((v / self.ticks_per_rad).value)
  }
}

#[derive(Clone, Debug)]
pub struct AngularToLinear<T> {
  pub angular: T,
  pub radius: Length
}

impl<T> AngularToLinear<T> {
  pub fn new(angular: T, radius: Length) -> Self {
    Self { angular, radius }
  }
}

impl<T> Wrapper<T> for AngularToLinear<T> {
  fn eject(self) -> T {
    self.angular
  }
}

impl<T: AngularSensor> Sensor<Length> for AngularToLinear<T> {
  #[inline(always)]
  fn get_sensor_value(&self) -> Length {
    self.angular.get_angle() * self.radius
  }
}

impl<T: AngularVelocitySensor> Sensor<Velocity> for AngularToLinear<T> {
  #[inline(always)]
  fn get_sensor_value(&self) -> Velocity {
    self.angular.get_angular_velocity() * self.radius
  }
}
