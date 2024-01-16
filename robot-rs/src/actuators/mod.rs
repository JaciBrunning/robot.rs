#[cfg(feature = "hal")]
pub mod pwm;

use std::marker::PhantomData;

#[cfg(feature = "ntcore")]
use ntcore_rs::GenericPublisher;
use robot_rs_units::traits::ToFloat;

use crate::filters::{Filter, InvertingFilter, ClampingFilter};
use crate::traits::Wrapper;
use crate::units::electrical::Voltage;

pub trait Actuator<U> {
  fn set_actuator_value(&mut self, value: U);
}

impl<'a, T: Actuator<U>, U> Actuator<U> for &'a mut T {
  fn set_actuator_value(&mut self, value: U) {
    (**self).set_actuator_value(value)
  }
}

macro_rules! actuator_alias {
  ($ident:ident, $unit:ty, $setter_name:ident) => {
    pub trait $ident : Actuator<$unit> {
      fn $setter_name(&mut self, value: $unit) { self.set_actuator_value(value) }
    }
    impl<T: Actuator<$unit>> $ident for T {}
  }
}

actuator_alias!(VoltageActuator, Voltage, set_voltage);

#[derive(Debug, Clone)]
pub struct FilteredActuator<T: Actuator<U>, U, F, I> {
  pub actuator: T,
  pub filter: F,
  phantom: PhantomData<(U, I)>
}

impl<T: Actuator<U>, U, F, I> FilteredActuator<T, U, F, I> {
  pub fn new(actuator: T, filter: F) -> Self {
    Self {
      actuator, filter, phantom: PhantomData
    }
  }
}

impl<T: Actuator<U>, U, F: Filter<I, Output=U>, I> Actuator<I> for FilteredActuator<T, U, F, I> {
  fn set_actuator_value(&mut self, value: I) {
    self.actuator.set_actuator_value(self.filter.calculate(value))
  }
}

pub type InvertedActuator<T, U> = FilteredActuator<T, U, InvertingFilter<U>, U>;
pub type ClampedActuator<T, U> = FilteredActuator<T, U, ClampingFilter<U>, U>;

#[cfg(feature = "ntcore")]
pub struct ObservableActuator<T: Actuator<U>, U> {
  actuator: T,
  #[allow(unused)]
  topic: crate::ntcore::Topic,
  publisher: crate::ntcore::Publisher<f64>,
  value_type: PhantomData<U>
}

#[cfg(feature = "ntcore")]
impl<T: Actuator<U>, U> ObservableActuator<T, U> {
  pub fn new(topic: crate::ntcore::Topic, act: T) -> Self {
    Self { actuator: act, publisher: topic.child("value").publish(), topic, value_type: PhantomData }
  }
}

#[cfg(feature = "ntcore")]
impl<T: Actuator<U>, U> Wrapper<T> for ObservableActuator<T, U> {
  fn eject(self) -> T {
    self.actuator
  }
}

#[cfg(feature = "ntcore")]
impl<T: Actuator<U>, U: ToFloat + Copy> Actuator<U> for ObservableActuator<T, U> {
  #[inline(always)]
  fn set_actuator_value(&mut self, demand: U) {
    self.actuator.set_actuator_value(demand);
    self.publisher.set(demand.to_f64()).ok();
  }
}

pub trait ActuatorExt<U> : Sized + Actuator<U> {
  fn invert(self) -> InvertedActuator<Self, U>;
  fn clamp(self, min: U, max: U) -> ClampedActuator<Self, U>;

  #[cfg(feature = "ntcore")]
  fn observable(self, topic: crate::ntcore::Topic) -> ObservableActuator<Self, U>;
}

impl<T: Actuator<U>, U> ActuatorExt<U> for T {
  fn invert(self) -> InvertedActuator<Self, U> {
    FilteredActuator::new(self, InvertingFilter::new())
  }

  fn clamp(self, min: U, max: U) -> ClampedActuator<Self, U> {
    FilteredActuator::new(self, ClampingFilter::new(min, max))
  }

  #[cfg(feature = "ntcore")]
  fn observable(self, topic: crate::ntcore::Topic) -> ObservableActuator<Self, U> {
    ObservableActuator::new(topic, self)
  }
}

#[cfg(feature = "simulation")]
pub mod sim {
  use std::sync::{RwLock, Arc};

  use super::Actuator;

  #[derive(Debug, Clone)]
  pub struct SimulatedActuator<U> {
    demand: Arc<RwLock<U>>
  }

  impl<U> SimulatedActuator<U> {
    pub fn new(initial: U) -> Self {
      Self { demand: Arc::new(RwLock::new(initial)) }
    }
  }

  impl<U: Clone> Actuator<U> for SimulatedActuator<U> {
    fn set_actuator_value(&mut self, demand: U) {
      *self.demand.write().unwrap() = demand;
    }
  }
}