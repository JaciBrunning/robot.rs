use std::{ops::Neg, f64::consts::PI, marker::PhantomData};

#[cfg(feature = "ntcore")]
use ntcore_rs::GenericPublisher;
use robot_rs_units::{motion::{TickVelocity, AngularVelocity, Velocity, rads_per_second}, traits::ToFloat};

use crate::{units::*, traits::Wrapper, filters::{Filter, InvertingFilter, StatefulFilter, FilterExt, StatefulFilterAdapter}};

pub trait Sensor<U> {
  fn get_sensor_value(&self) -> U;
  fn get_last_measurement_time(&self) -> Time;
}

pub trait StatefulSensor<U> {
  fn get_sensor_value(&mut self) -> U;
  fn get_last_measurement_time(&self) -> Time;
}

impl<'a, T: Sensor<U>, U> Sensor<U> for &'a T {
  fn get_sensor_value(&self) -> U {
    (**self).get_sensor_value()
  }

  fn get_last_measurement_time(&self) -> Time {
    (**self).get_last_measurement_time()
  }
}

impl<'a, T: StatefulSensor<U>, U> StatefulSensor<U> for &'a mut T {
  fn get_sensor_value(&mut self) -> U {
    (**self).get_sensor_value()
  }

  fn get_last_measurement_time(&self) -> Time {
    (**self).get_last_measurement_time()
  }
}

macro_rules! sensor_alias {
  ($ident:ident, $stateful_ident:ident, $unit:ty, $fn_name:ident) => {
    pub trait $ident : Sensor<$unit> {
      fn $fn_name(&self) -> $unit { self.get_sensor_value() }
    }
    impl<T: Sensor<$unit>> $ident for T {}

    pub trait $stateful_ident : StatefulSensor<$unit> {
      fn $fn_name(&mut self) -> $unit { self.get_sensor_value() }
    }
    impl<T: StatefulSensor<$unit>> $stateful_ident for T {}
  }
}

sensor_alias!(BinarySensor, StatefulBinarySensor, bool, get_state);
sensor_alias!(Encoder, StatefulEncoder, Ticks, get_ticks);
sensor_alias!(VelocityEncoder, StatefulVelocityEncoder, TickVelocity, get_tick_velocity);
sensor_alias!(AngularSensor, StatefulAngularSensor, Angle, get_angle);
sensor_alias!(AngularVelocitySensor, StatefulAngularVelocitySensor, AngularVelocity, get_angular_velocity);
sensor_alias!(DisplacementSensor, StatefulDisplacementSensor, Length, get_displacement);
sensor_alias!(VelocitySensor, StatefulVelocitySensor, Velocity, get_velocity);
sensor_alias!(CurrentSensor, StatefulCurrentSensor, Current, get_current);

#[derive(Clone, Debug)]
pub struct StatefulSensorAdapter<T: Sensor<U>, U> {
  pub sensor: T,
  phantom: PhantomData<U>
}

impl<T: Sensor<U>, U> StatefulSensorAdapter<T, U> {
  pub fn new(sensor: T) -> Self {
    Self { sensor, phantom: PhantomData }
  }
}

impl<T: Sensor<U>, U> StatefulSensor<U> for StatefulSensorAdapter<T, U> {
  fn get_sensor_value(&mut self) -> U {
    self.sensor.get_sensor_value()
  }

  fn get_last_measurement_time(&self) -> Time {
    self.sensor.get_last_measurement_time()
  }
}

#[derive(Clone, Debug)]
pub struct FilteredSensor<T: Sensor<U>, U, F> {
  pub sensor: T,
  pub filter: F,
  phantom: PhantomData<U>
}

impl<T: Sensor<U>, U, F> FilteredSensor<T, U, F> {
  pub fn new(sensor: T, filter: F) -> Self {
    Self { sensor, filter, phantom: PhantomData }
  }
}

impl<T: Sensor<U>, U, F: Filter<U, Time>> Sensor<<F as Filter<U, Time>>::Output> for FilteredSensor<T, U, F> {
  fn get_sensor_value(&self) -> <F as Filter<U, Time>>::Output {
    self.filter.calculate(self.sensor.get_sensor_value(), self.get_last_measurement_time())
  }

  fn get_last_measurement_time(&self) -> Time {
    self.sensor.get_last_measurement_time()
  }
}

#[derive(Clone, Debug)]
pub struct FilteredStatefulSensor<T: StatefulSensor<U>, U, F> {
  pub sensor: T,
  pub filter: F,
  phantom: PhantomData<U>
}

impl<T: StatefulSensor<U>, U, F> FilteredStatefulSensor<T, U, F> {
  pub fn new(sensor: T, filter: F) -> Self {
    Self { sensor, filter, phantom: PhantomData }
  }
}

impl<T: StatefulSensor<U>, U, F: StatefulFilter<U, Time>> StatefulSensor<<F as StatefulFilter<U, Time>>::Output> for FilteredStatefulSensor<T, U, F> {
  fn get_sensor_value(&mut self) -> <F as StatefulFilter<U, Time>>::Output {
    self.filter.calculate(self.sensor.get_sensor_value(), self.get_last_measurement_time())
  }

  fn get_last_measurement_time(&self) -> Time {
    self.sensor.get_last_measurement_time()
  }
}

// CONVERSIONS
// TODO: Move these to filters?

#[derive(Clone, Debug)]
pub struct EncoderToAngular<T> {
  pub encoder: T,
  pub ticks_per_rad: Ticks
}

impl<T> EncoderToAngular<T> {
  pub fn new(encoder: T, ticks_per_revolution: Ticks) -> Self {
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
    Angle::new::<radian>((self.encoder.get_ticks() / self.ticks_per_rad).to_base())
  }

  fn get_last_measurement_time(&self) -> Time {
    self.encoder.get_last_measurement_time()
  }
}

impl<T: VelocityEncoder> Sensor<AngularVelocity> for EncoderToAngular<T> {
  #[inline(always)]
  fn get_sensor_value(&self) -> AngularVelocity {
    AngularVelocity::new::<rads_per_second>((self.encoder.get_tick_velocity() / self.ticks_per_rad).to_base())
  }

  fn get_last_measurement_time(&self) -> Time {
    self.encoder.get_last_measurement_time()
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
    self.angular.get_angle() / (1.0 * radian) * self.radius
  }

  fn get_last_measurement_time(&self) -> Time {
    self.angular.get_last_measurement_time()
  }
}

impl<T: AngularVelocitySensor> Sensor<Velocity> for AngularToLinear<T> {
  #[inline(always)]
  fn get_sensor_value(&self) -> Velocity {
    self.angular.get_angular_velocity() / (1.0 * radian) * self.radius
  }

  fn get_last_measurement_time(&self) -> Time {
    self.angular.get_last_measurement_time()
  }
}

#[cfg(feature = "ntcore")]
pub struct ObservableSensor<U, T: Sensor<U>> {
  sensor: T,
  #[allow(unused)]
  topic: crate::ntcore::Topic,
  publisher: crate::ntcore::Publisher<f64>,
  phantom: PhantomData<U>
}

#[cfg(feature = "ntcore")]
impl<U, T: Sensor<U>> ObservableSensor<U, T> {
  pub fn new(topic: crate::ntcore::Topic, sensor: T) -> Self {
    Self { sensor, publisher: topic.child("value").publish(), topic, phantom: PhantomData }
  }
}

#[cfg(feature = "ntcore")]
impl<U, T: Sensor<U>> Wrapper<T> for ObservableSensor<U, T> {
  fn eject(self) -> T {
    self.sensor
  }
}

#[cfg(feature = "ntcore")]
impl<U: ToFloat + Copy, T: Sensor<U>> Sensor<U> for ObservableSensor<U, T> {
  fn get_sensor_value(&self) -> U {
    let v = self.sensor.get_sensor_value();
    self.publisher.set(v.to_f64()).ok();
    v
  }

  fn get_last_measurement_time(&self) -> Time {
    self.sensor.get_last_measurement_time()
  }
}

#[cfg(feature = "ntcore")]
pub struct ObservableStatefulSensor<U, T: StatefulSensor<U>> {
  sensor: T,
  #[allow(unused)]
  topic: crate::ntcore::Topic,
  publisher: crate::ntcore::Publisher<f64>,
  phantom: PhantomData<U>
}

#[cfg(feature = "ntcore")]
impl<U, T: StatefulSensor<U>> ObservableStatefulSensor<U, T> {
  pub fn new(topic: crate::ntcore::Topic, sensor: T) -> Self {
    Self { sensor, publisher: topic.child("value").publish(), topic, phantom: PhantomData }
  }
}

#[cfg(feature = "ntcore")]
impl<U, T: StatefulSensor<U>> Wrapper<T> for ObservableStatefulSensor<U, T> {
  fn eject(self) -> T {
    self.sensor
  }
}

#[cfg(feature = "ntcore")]
impl<U: ToFloat + Copy, T: StatefulSensor<U>> StatefulSensor<U> for ObservableStatefulSensor<U, T> {
  fn get_sensor_value(&mut self) -> U {
    let v = self.sensor.get_sensor_value();
    self.publisher.set(v.to_f64()).ok();
    v
  }

  fn get_last_measurement_time(&self) -> Time {
    self.sensor.get_last_measurement_time()
  }
}

pub type InvertedSensor<T, U> = FilteredSensor<T, U, InvertingFilter<U>>;
pub type InvertedStatefulSensor<T, U> = FilteredStatefulSensor<T, U, StatefulFilterAdapter<InvertingFilter<U>, U, Time>>;

pub trait SensorExt<U> : Sized + Sensor<U> {
  fn invert(self) -> InvertedSensor<Self, U>;
  fn filter<F>(self, filter: F) -> FilteredSensor<Self, U, F>;

  fn to_stateful(self) -> StatefulSensorAdapter<Self, U>;

  #[cfg(feature = "ntcore")]
  fn observable(self, topic: crate::ntcore::Topic) -> ObservableSensor<U, Self>;
}

impl<U, T: Sensor<U>> SensorExt<U> for T {
  fn invert(self) -> InvertedSensor<Self, U> {
    FilteredSensor::new(self, InvertingFilter::new())
  }

  fn to_stateful(self) -> StatefulSensorAdapter<Self, U> {
    StatefulSensorAdapter::new(self)
  }

  fn filter<F>(self, filter: F) -> FilteredSensor<Self, U, F> {
    FilteredSensor::new(self, filter)
  }

  #[cfg(feature = "ntcore")]
  fn observable(self, topic: crate::ntcore::Topic) -> ObservableSensor<U, Self> {
    ObservableSensor::new(topic, self)
  }
}

pub trait StatefulSensorExt<U: Neg<Output = U>> : Sized + StatefulSensor<U> {
  fn invert(self) -> InvertedStatefulSensor<Self, U>;
  fn filter<F>(self, filter: F) -> FilteredStatefulSensor<Self, U, F>;

  #[cfg(feature = "ntcore")]
  fn observable(self, topic: crate::ntcore::Topic) -> ObservableStatefulSensor<U, Self>;
}

impl<U: Neg<Output = U>, T: StatefulSensor<U>> StatefulSensorExt<U> for T {
  fn invert(self) -> InvertedStatefulSensor<Self, U> {
    FilteredStatefulSensor::new(self, InvertingFilter::new().to_stateful())
  }

  fn filter<F>(self, filter: F) -> FilteredStatefulSensor<Self, U, F> {
    FilteredStatefulSensor::new(self, filter)
  }

  #[cfg(feature = "ntcore")]
  fn observable(self, topic: crate::ntcore::Topic) -> ObservableStatefulSensor<U, Self> {
    ObservableStatefulSensor::new(topic, self)
  }
}

#[cfg(feature = "simulation")]
pub mod sim {
  use std::sync::{RwLock, Arc};

  use robot_rs_units::Time;

  use super::Sensor;

  pub trait SettableSensor<U> : Sensor<U> {
    fn set_sensor_value(&self, value: U, time: Time);
  }

  #[derive(Debug, Clone)]
  pub struct SimulatedSensor<U> {
    value: Arc<RwLock<(U, Time)>>
  }

  impl<U> SimulatedSensor<U> {
    pub fn new(default: U) -> Self {
      Self { value: Arc::new(RwLock::new((default, crate::time::now()))) }
    }
  }

  impl<U: Clone> SettableSensor<U> for SimulatedSensor<U> {
    fn set_sensor_value(&self, value: U, time: Time) {
      *self.value.write().unwrap() = (value, time);
    }
  }

  impl<U: Clone> Sensor<U> for SimulatedSensor<U> {
    fn get_sensor_value(&self) -> U {
      self.value.read().unwrap().0.clone()
    }

    fn get_last_measurement_time(&self) -> robot_rs_units::Time {
      self.value.read().unwrap().1
    }
  }
}

#[cfg(test)]
mod tests {
  use std::{sync::{Arc, Mutex}, time::Duration};

  use approx::assert_relative_eq;
  use num_traits::Zero;
  use robot_rs_units::motion::meters_per_second;

  use crate::{units::*, sensors::{FilteredSensor, FilteredStatefulSensor, SensorExt, StatefulSensor}, filters::diff::DifferentiatingFilter, time};
  use super::Sensor;

  struct MockSensor {
    pub value: Arc<Mutex<Length>>
  }

  impl Sensor<Length> for MockSensor {
    fn get_sensor_value(&self) -> Length {
      self.value.lock().unwrap().clone()
    }

    fn get_last_measurement_time(&self) -> Time {
      crate::time::now()
    }
  }

  #[test]
  fn test_diff() {
    let sensor = MockSensor { value: Arc::new(Mutex::new(Length::new::<meter>(0.0))) };

    let value = sensor.value.clone();
    // let diff: DifferentiableSensor<Length, _> = DifferentiableSensor::new(sensor);
    let mut diff = FilteredStatefulSensor::new(sensor.to_stateful(), DifferentiatingFilter::new());
  
    assert_eq!(0.0 * meters_per_second, diff.get_sensor_value());
    std::thread::sleep(Duration::from_millis(100));
    assert_eq!(0.0 * meters_per_second, diff.get_sensor_value());
    *(value.lock().unwrap()) = Length::new::<meter>(1.5);
    std::thread::sleep(Duration::from_millis(100));
    assert_relative_eq!(15.0, diff.get_sensor_value().to::<meters_per_second>(), epsilon = 2.0);
  }
}