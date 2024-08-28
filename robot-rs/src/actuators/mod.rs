#[cfg(feature = "hal")]
pub mod pwm;

use std::marker::PhantomData;
use std::ops::Neg;

#[cfg(feature = "ntcore")]
use ntcore_rs::GenericPublisher;
use robot_rs_units::traits::ToFloat;

use crate::traits::Wrapper;
use crate::transforms::{
  ClampingTransform, InvertingTransform, StatefulTransform, StatefulTransformAdapter, TransformExt,
};
use crate::units::electrical::Voltage;

pub trait Actuator<U, Time = crate::units::Time> {
  fn set_actuator_value(&mut self, value: U, now: Time);
}

impl<'a, T: Actuator<U, Time>, U, Time> Actuator<U, Time> for &'a mut T {
  fn set_actuator_value(&mut self, value: U, now: Time) {
    (**self).set_actuator_value(value, now)
  }
}

macro_rules! actuator_alias {
  ($ident:ident, $unit:ty, $setter_name:ident) => {
    pub trait $ident<Time = crate::units::Time>: Actuator<$unit, Time> {
      fn $setter_name(&mut self, value: $unit, time: Time) {
        self.set_actuator_value(value, time)
      }
    }
    impl<T: Actuator<$unit, Time>, Time> $ident<Time> for T {}
  };
}

actuator_alias!(VoltageActuator, Voltage, set_voltage);

#[derive(Debug, Clone)]
pub struct TransformedActuator<T: Actuator<U, Time>, U, F, I, Time> {
  pub actuator: T,
  pub transform: F,
  phantom: PhantomData<(U, I, Time)>,
}

impl<T: Actuator<U, Time>, U, F, I, Time> TransformedActuator<T, U, F, I, Time> {
  pub fn new(actuator: T, transform: F) -> Self {
    Self {
      actuator,
      transform,
      phantom: PhantomData,
    }
  }
}

impl<T: Actuator<U, Time>, U, F: StatefulTransform<I, Time, Output = U>, I, Time: Copy>
  Actuator<I, Time> for TransformedActuator<T, U, F, I, Time>
{
  fn set_actuator_value(&mut self, value: I, now: Time) {
    self
      .actuator
      .set_actuator_value(self.transform.calculate(value, now), now)
  }
}

pub type InvertedActuator<T, U, Time> =
  TransformedActuator<T, U, StatefulTransformAdapter<InvertingTransform<U>, U>, U, Time>;
pub type ClampedActuator<T, U, Time> =
  TransformedActuator<T, U, StatefulTransformAdapter<ClampingTransform<U>, U>, U, Time>;

#[cfg(feature = "ntcore")]
pub struct ObservableActuator<T: Actuator<U, Time>, U, Time> {
  pub actuator: T,
  #[allow(unused)]
  topic: crate::ntcore::Topic,
  publisher: crate::ntcore::Publisher<f64>,
  value_type: PhantomData<(U, Time)>,
}

#[cfg(feature = "ntcore")]
impl<T: Actuator<U, Time>, U, Time> ObservableActuator<T, U, Time> {
  pub fn new(topic: crate::ntcore::Topic, act: T) -> Self {
    Self {
      actuator: act,
      publisher: topic.child("value").publish(),
      topic,
      value_type: PhantomData,
    }
  }
}

#[cfg(feature = "ntcore")]
impl<T: Actuator<U, Time>, U, Time> Wrapper<T> for ObservableActuator<T, U, Time> {
  fn eject(self) -> T {
    self.actuator
  }
}

#[cfg(feature = "ntcore")]
impl<T: Actuator<U, Time>, U: ToFloat + Copy, Time> Actuator<U, Time>
  for ObservableActuator<T, U, Time>
{
  #[inline(always)]
  fn set_actuator_value(&mut self, demand: U, now: Time) {
    self.actuator.set_actuator_value(demand, now);
    self.publisher.set(demand.to_f64()).ok();
  }
}

pub trait ActuatorExt<U: Neg<Output = U> + PartialOrd<U> + Copy, Time>:
  Sized + Actuator<U, Time>
{
  fn invert(self) -> InvertedActuator<Self, U, Time>;
  fn clamp(self, min: U, max: U) -> ClampedActuator<Self, U, Time>;
  fn transform<I, F>(self, transform: F) -> TransformedActuator<Self, U, F, I, Time>;

  #[cfg(feature = "ntcore")]
  fn observable(self, topic: crate::ntcore::Topic) -> ObservableActuator<Self, U, Time>;
}

impl<T: Actuator<U, Time>, U: Neg<Output = U> + PartialOrd<U> + Copy, Time> ActuatorExt<U, Time>
  for T
{
  fn invert(self) -> InvertedActuator<Self, U, Time> {
    TransformedActuator::new(self, InvertingTransform::new().to_stateful())
  }

  fn clamp(self, min: U, max: U) -> ClampedActuator<Self, U, Time> {
    TransformedActuator::new(self, ClampingTransform::new(min, max).to_stateful())
  }

  fn transform<I, F>(self, transform: F) -> TransformedActuator<Self, U, F, I, Time> {
    TransformedActuator::new(self, transform)
  }

  #[cfg(feature = "ntcore")]
  fn observable(self, topic: crate::ntcore::Topic) -> ObservableActuator<Self, U, Time> {
    ObservableActuator::new(topic, self)
  }
}

#[cfg(feature = "simulation")]
pub mod sim {
  use std::sync::{Arc, RwLock};

  use super::Actuator;

  pub trait SimActuator<U, Time>: Actuator<U, Time> {
    fn get_actuator_value(&self) -> (U, Time);
  }

  #[derive(Debug, Clone)]
  pub struct SimulatedActuator<U, Time> {
    demand: Arc<RwLock<(U, Time)>>,
  }

  impl<U, Time> SimulatedActuator<U, Time> {
    pub fn new(initial: U, now: Time) -> Self {
      Self {
        demand: Arc::new(RwLock::new((initial, now))),
      }
    }
  }

  impl<U: Clone, Time> Actuator<U, Time> for SimulatedActuator<U, Time> {
    fn set_actuator_value(&mut self, demand: U, time: Time) {
      *self.demand.write().unwrap() = (demand, time);
    }
  }

  impl<U: Clone, Time: Clone> SimActuator<U, Time> for SimulatedActuator<U, Time> {
    fn get_actuator_value(&self) -> (U, Time) {
      self.demand.read().unwrap().clone()
    }
  }
}
