use std::{marker::PhantomData, ops::Neg};

#[cfg(feature = "ntcore")]
use ntcore_rs::GenericPublisher;
use robot_rs_units::{
  motion::{AngularVelocity, TickVelocity, Velocity},
  traits::ToFloat,
};

use crate::{
  traits::Wrapper,
  transforms::{
    InvertingTransform, StatefulTransform, StatefulTransformAdapter, Transform, TransformExt,
  },
  units::*,
};

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
    pub trait $ident: Sensor<$unit> {
      fn $fn_name(&self) -> $unit {
        self.get_sensor_value()
      }
    }
    impl<T: Sensor<$unit>> $ident for T {}

    pub trait $stateful_ident: StatefulSensor<$unit> {
      fn $fn_name(&mut self) -> $unit {
        self.get_sensor_value()
      }
    }
    impl<T: StatefulSensor<$unit>> $stateful_ident for T {}
  };
}

sensor_alias!(BinarySensor, StatefulBinarySensor, bool, get_state);
sensor_alias!(Encoder, StatefulEncoder, Ticks, get_ticks);
sensor_alias!(
  VelocityEncoder,
  StatefulVelocityEncoder,
  TickVelocity,
  get_tick_velocity
);
sensor_alias!(AngularSensor, StatefulAngularSensor, Angle, get_angle);
sensor_alias!(
  AngularVelocitySensor,
  StatefulAngularVelocitySensor,
  AngularVelocity,
  get_angular_velocity
);
sensor_alias!(
  DisplacementSensor,
  StatefulDisplacementSensor,
  Length,
  get_displacement
);
sensor_alias!(
  VelocitySensor,
  StatefulVelocitySensor,
  Velocity,
  get_velocity
);
sensor_alias!(CurrentSensor, StatefulCurrentSensor, Current, get_current);

#[derive(Clone, Debug)]
pub struct StatefulSensorAdapter<T: Sensor<U>, U> {
  pub sensor: T,
  phantom: PhantomData<U>,
}

impl<T: Sensor<U>, U> StatefulSensorAdapter<T, U> {
  pub fn new(sensor: T) -> Self {
    Self {
      sensor,
      phantom: PhantomData,
    }
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
pub struct TransformedSensor<T: Sensor<U>, U, F> {
  pub sensor: T,
  pub transform: F,
  phantom: PhantomData<U>,
}

impl<T: Sensor<U>, U, F> TransformedSensor<T, U, F> {
  pub fn new(sensor: T, transform: F) -> Self {
    Self {
      sensor,
      transform,
      phantom: PhantomData,
    }
  }
}

impl<T: Sensor<U>, U, F: Transform<U>> Sensor<<F as Transform<U>>::Output>
  for TransformedSensor<T, U, F>
{
  fn get_sensor_value(&self) -> <F as Transform<U>>::Output {
    self.transform.calculate(self.sensor.get_sensor_value())
  }

  fn get_last_measurement_time(&self) -> Time {
    self.sensor.get_last_measurement_time()
  }
}

#[derive(Clone, Debug)]
pub struct TransformedStatefulSensor<T: StatefulSensor<U>, U, F> {
  pub sensor: T,
  pub transform: F,
  phantom: PhantomData<U>,
}

impl<T: StatefulSensor<U>, U, F> TransformedStatefulSensor<T, U, F> {
  pub fn new(sensor: T, transform: F) -> Self {
    Self {
      sensor,
      transform,
      phantom: PhantomData,
    }
  }
}

impl<T: StatefulSensor<U>, U, F: StatefulTransform<U, Time>>
  StatefulSensor<<F as StatefulTransform<U, Time>>::Output> for TransformedStatefulSensor<T, U, F>
{
  fn get_sensor_value(&mut self) -> <F as StatefulTransform<U, Time>>::Output {
    self.transform.calculate(
      self.sensor.get_sensor_value(),
      self.get_last_measurement_time(),
    )
  }

  fn get_last_measurement_time(&self) -> Time {
    self.sensor.get_last_measurement_time()
  }
}

#[cfg(feature = "ntcore")]
pub struct ObservableSensor<U, T: Sensor<U>> {
  sensor: T,
  #[allow(unused)]
  topic: crate::ntcore::Topic,
  publisher: crate::ntcore::Publisher<f64>,
  phantom: PhantomData<U>,
}

#[cfg(feature = "ntcore")]
impl<U, T: Sensor<U>> ObservableSensor<U, T> {
  pub fn new(topic: crate::ntcore::Topic, sensor: T) -> Self {
    Self {
      sensor,
      publisher: topic.child("value").publish(),
      topic,
      phantom: PhantomData,
    }
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
  phantom: PhantomData<U>,
}

#[cfg(feature = "ntcore")]
impl<U, T: StatefulSensor<U>> ObservableStatefulSensor<U, T> {
  pub fn new(topic: crate::ntcore::Topic, sensor: T) -> Self {
    Self {
      sensor,
      publisher: topic.child("value").publish(),
      topic,
      phantom: PhantomData,
    }
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

pub type InvertedSensor<T, U> = TransformedSensor<T, U, InvertingTransform<U>>;
pub type InvertedStatefulSensor<T, U> =
  TransformedStatefulSensor<T, U, StatefulTransformAdapter<InvertingTransform<U>, U>>;

pub trait SensorExt<U>: Sized + Sensor<U> {
  fn invert(self) -> InvertedSensor<Self, U>;
  fn transform<F>(self, transform: F) -> TransformedSensor<Self, U, F>;

  fn to_stateful(self) -> StatefulSensorAdapter<Self, U>;

  #[cfg(feature = "ntcore")]
  fn observable(self, topic: crate::ntcore::Topic) -> ObservableSensor<U, Self>;
}

impl<U, T: Sensor<U>> SensorExt<U> for T {
  fn invert(self) -> InvertedSensor<Self, U> {
    TransformedSensor::new(self, InvertingTransform::new())
  }

  fn to_stateful(self) -> StatefulSensorAdapter<Self, U> {
    StatefulSensorAdapter::new(self)
  }

  fn transform<F>(self, transform: F) -> TransformedSensor<Self, U, F> {
    TransformedSensor::new(self, transform)
  }

  #[cfg(feature = "ntcore")]
  fn observable(self, topic: crate::ntcore::Topic) -> ObservableSensor<U, Self> {
    ObservableSensor::new(topic, self)
  }
}

pub trait StatefulSensorExt<U: Neg<Output = U>>: Sized + StatefulSensor<U> {
  fn invert(self) -> InvertedStatefulSensor<Self, U>;
  fn transform<F>(self, transform: F) -> TransformedStatefulSensor<Self, U, F>;

  #[cfg(feature = "ntcore")]
  fn observable(self, topic: crate::ntcore::Topic) -> ObservableStatefulSensor<U, Self>;
}

impl<U: Neg<Output = U>, T: StatefulSensor<U>> StatefulSensorExt<U> for T {
  fn invert(self) -> InvertedStatefulSensor<Self, U> {
    TransformedStatefulSensor::new(self, InvertingTransform::new().to_stateful())
  }

  fn transform<F>(self, transform: F) -> TransformedStatefulSensor<Self, U, F> {
    TransformedStatefulSensor::new(self, transform)
  }

  #[cfg(feature = "ntcore")]
  fn observable(self, topic: crate::ntcore::Topic) -> ObservableStatefulSensor<U, Self> {
    ObservableStatefulSensor::new(topic, self)
  }
}

#[cfg(feature = "simulation")]
pub mod sim {
  use std::sync::{Arc, RwLock};

  use robot_rs_units::Time;

  use super::Sensor;

  pub trait SimSensor<U>: Sensor<U> {
    fn set_sensor_value(&mut self, value: U, time: Time);
  }

  #[derive(Debug, Clone)]
  pub struct SimulatedSensor<U> {
    value: Arc<RwLock<(U, Time)>>,
  }

  impl<U> SimulatedSensor<U> {
    pub fn new(default: U) -> Self {
      Self {
        value: Arc::new(RwLock::new((default, crate::time::now()))),
      }
    }
  }

  impl<U: Clone> SimSensor<U> for SimulatedSensor<U> {
    fn set_sensor_value(&mut self, value: U, time: Time) {
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
  use std::{
    sync::{Arc, Mutex},
    time::Duration,
  };

  use approx::assert_relative_eq;
  use robot_rs_units::motion::meters_per_second;

  use super::Sensor;
  use crate::{
    sensors::{SensorExt, StatefulSensor, TransformedStatefulSensor},
    transforms::diff::DifferentiatingTransform,
    units::*,
  };

  struct MockSensor {
    pub value: Arc<Mutex<Length>>,
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
    let sensor = MockSensor {
      value: Arc::new(Mutex::new(Length::new::<meter>(0.0))),
    };

    let value = sensor.value.clone();
    // let diff: DifferentiableSensor<Length, _> = DifferentiableSensor::new(sensor);
    let mut diff =
      TransformedStatefulSensor::new(sensor.to_stateful(), DifferentiatingTransform::new());

    assert_eq!(0.0 * meters_per_second, diff.get_sensor_value());
    std::thread::sleep(Duration::from_millis(100));
    assert_eq!(0.0 * meters_per_second, diff.get_sensor_value());
    *(value.lock().unwrap()) = Length::new::<meter>(1.5);
    std::thread::sleep(Duration::from_millis(100));
    assert_relative_eq!(
      15.0,
      diff.get_sensor_value().to::<meters_per_second>(),
      epsilon = 2.0
    );
  }
}
