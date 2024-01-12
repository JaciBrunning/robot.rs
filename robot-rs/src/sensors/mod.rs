use std::{ops::{Neg, Sub, Div}, f64::consts::PI, cell::RefCell};

use crate::{units::*, traits::Wrapper};

pub trait Sensor<U> {
  fn get_sensor_value(&self) -> Option<U>;
}

impl<'a, T: Sensor<U>, U> Sensor<U> for &'a T {
  fn get_sensor_value(&self) -> Option<U> {
    (**self).get_sensor_value()
  }
}

macro_rules! sensor_alias {
  ($ident:ident, $unit:ty, $fn_name:ident) => {
    pub trait $ident : Sensor<$unit> {
      fn $fn_name(&self) -> Option<$unit> { self.get_sensor_value() }
    }
    impl<T: Sensor<$unit>> $ident for T {}
  }
}

sensor_alias!(BinarySensor, bool, get_state);
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
  fn get_sensor_value(&self) -> Option<U> {
    self.0.get_sensor_value().map(|x| -x)
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
  fn get_sensor_value(&self) -> Option<Angle> {
    self.encoder.get_ticks().map(|ticks| {
      Angle::new::<angle::radian>((ticks / self.ticks_per_rad).value)
    })
  }
}

impl<T: VelocityEncoder> Sensor<AngularVelocity> for EncoderToAngular<T> {
  #[inline(always)]
  fn get_sensor_value(&self) -> Option<AngularVelocity> {
    self.encoder.get_tick_velocity().map(|tick_vel| {
      AngularVelocity::new::<angular_velocity::radian_per_second>((tick_vel / self.ticks_per_rad).value)
    })
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
  fn get_sensor_value(&self) -> Option<Length> {
    self.angular.get_angle().map(|x| x * self.radius)
  }
}

impl<T: AngularVelocitySensor> Sensor<Velocity> for AngularToLinear<T> {
  #[inline(always)]
  fn get_sensor_value(&self) -> Option<Velocity> {
    self.angular.get_angular_velocity().map(|x| x * self.radius)
  }
}

#[derive(Debug, Clone)]
pub struct DifferentiableSensor<
  U: Clone,
  T: Sensor<U>
> {
  pub sensor: T,
  pub last_value: RefCell<Option<(Time, U)>>
}

impl<
  U: Clone,
  T: Sensor<U>
> DifferentiableSensor<U, T> {
  pub fn new(sensor: T) -> Self {
    Self { sensor, last_value: RefCell::new(None) }
  }
}

impl<
  U: Clone,
  Ur,
  DO,
  T: Sensor<U>,
> Sensor<Ur> for DifferentiableSensor<U, T>
where
  U: Sub<U, Output = DO>,
  DO: Div<Time, Output = Ur>
{
  fn get_sensor_value(&self) -> Option<Ur> {
    let now = crate::time::now();
    let measurement = self.sensor.get_sensor_value()?;

    let last = self.last_value.replace(Some((now, measurement.clone())));
    let ret = last.map(|(last_time, last_value)| {
      let dt = now - last_time;
      (measurement - last_value) / dt
    });

    ret
  }
}

#[cfg(test)]
mod tests {
  use std::{sync::{Arc, Mutex}, time::Duration};

  use approx::assert_relative_eq;
  use num_traits::Zero;

  use crate::units::*;
  use super::{Sensor, DifferentiableSensor};

  struct MockSensor {
    pub value: Arc<Mutex<Length>>
  }

  impl Sensor<Length> for MockSensor {
    fn get_sensor_value(&self) -> Option<Length> {
      Some(self.value.lock().unwrap().clone())
    }
  }

  #[test]
  fn test_diff() {
    let sensor = MockSensor { value: Arc::new(Mutex::new(Length::new::<length::meter>(0.0))) };

    let value = sensor.value.clone();
    let diff: DifferentiableSensor<Length, _> = DifferentiableSensor::new(sensor);
  
    assert_eq!(None, diff.get_sensor_value());
    std::thread::sleep(Duration::from_millis(100));
    assert_eq!(Some(Zero::zero()), diff.get_sensor_value());
    *(value.lock().unwrap()) = Length::new::<length::meter>(1.5);
    std::thread::sleep(Duration::from_millis(100));
    assert_relative_eq!(15.0, diff.get_sensor_value().unwrap().get::<velocity::meter_per_second>(), epsilon = 2.0);
  }
}