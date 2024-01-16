#[cfg(feature = "hal")]
pub mod pwm;

use std::marker::PhantomData;
use std::ops::Neg;

#[cfg(feature = "ntcore")]
use ntcore_rs::GenericPublisher;
use robot_rs_units::QuantityBase;
use robot_rs_units::traits::{MaybeUnitNumber, ToFloat};

use crate::traits::Wrapper;
use crate::units::electrical::Voltage;

pub trait Actuator<U> {
  fn set_actuator_value(&mut self, value: U);
  fn get_set_actuator_value(&self) -> U;
}

impl<'a, T: Actuator<U>, U> Actuator<U> for &'a mut T {
  fn set_actuator_value(&mut self, value: U) {
    (**self).set_actuator_value(value)
  }

  fn get_set_actuator_value(&self) -> U {
    (**self).get_set_actuator_value()
  }
}

macro_rules! actuator_alias {
  ($ident:ident, $unit:ty, $setter_name:ident, $getter_name:ident) => {
    pub trait $ident : Actuator<$unit> {
      fn $setter_name(&mut self, value: $unit) { self.set_actuator_value(value) }
      fn $getter_name(&self) -> $unit { self.get_set_actuator_value() }
    }
    impl<T: Actuator<$unit>> $ident for T {}
  }
}

actuator_alias!(VoltageActuator, Voltage, set_voltage, get_set_voltage);

#[derive(Debug, Clone)]
pub struct InvertedActuator<T: Actuator<U>, U> {
  actuator: T,
  value_type: PhantomData<U>
}

impl<T: Actuator<U>, U> From<T> for InvertedActuator<T, U> {
  fn from(value: T) -> Self {
    Self { actuator: value, value_type: PhantomData }
  }
}

impl<T: Actuator<U>, U> Wrapper<T> for InvertedActuator<T, U> {
  fn eject(self) -> T { self.actuator }
}

impl<T: Actuator<U>, U: Neg<Output = U>> Actuator<U> for InvertedActuator<T, U> {
  #[inline(always)]
  fn set_actuator_value(&mut self, demand: U) {
    self.actuator.set_actuator_value(-demand);
  }

  #[inline(always)]
  fn get_set_actuator_value(&self) -> U {
    -self.actuator.get_set_actuator_value()
  }
}

#[derive(Debug, Clone)]
pub struct ClampedActuator<T: Actuator<U>, U> {
  actuator: T,
  min: U,
  max: U,
}

impl<T: Actuator<U>, U> ClampedActuator<T, U> {
  pub fn new(min: U, max: U, act: T) -> Self {
    Self { actuator: act, min, max }
  }
}

impl<T: Actuator<U>, U> Wrapper<T> for ClampedActuator<T, U> {
  fn eject(self) -> T {
    self.actuator
  }
}

impl<T: Actuator<U>, U: PartialOrd<U> + Clone> Actuator<U> for ClampedActuator<T, U> {
  #[inline(always)]
  fn set_actuator_value(&mut self, demand: U) {
    self.actuator.set_actuator_value(match demand {
      d if d > self.max => self.max.clone(),
      d if d < self.min => self.min.clone(),
      d => d
    })
  }

  #[inline(always)]
  fn get_set_actuator_value(&self) -> U {
    self.actuator.get_set_actuator_value()
  }
}

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

  #[inline(always)]
  fn get_set_actuator_value(&self) -> U {
    self.actuator.get_set_actuator_value()
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
    InvertedActuator::from(self)
  }

  fn clamp(self, min: U, max: U) -> ClampedActuator<Self, U> {
    ClampedActuator::new(min, max, self)
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

    fn get_set_actuator_value(&self) -> U {
      self.demand.read().unwrap().clone()
    }
  }
}