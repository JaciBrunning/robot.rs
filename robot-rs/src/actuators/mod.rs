#[cfg(feature = "hal")]
pub mod pwm;

#[cfg(feature = "ntcore")]
use ntcore_rs::GenericPublisher;
use robot_rs_units::QuantityBase;
use robot_rs_units::traits::MaybeUnitNumber;

use crate::traits::Wrapper;
use crate::units::electrical::Voltage;

pub trait VoltageController {
  fn set_voltage(&mut self, voltage: Voltage);
  fn get_set_voltage(&self) -> Voltage;
}

impl<'a, T: VoltageController> VoltageController for &'a mut T {
  fn set_voltage(&mut self, voltage: Voltage) {
    (**self).set_voltage(voltage)
  }

  fn get_set_voltage(&self) -> Voltage {
    (**self).get_set_voltage()
  }
}

pub trait HasBusVoltage {
  fn get_bus_voltage(&self) -> Voltage;
}

impl<'a, T: HasBusVoltage> HasBusVoltage for &'a T {
  fn get_bus_voltage(&self) -> Voltage {
    (**self).get_bus_voltage()
  }
}

#[derive(Debug, Clone)]
pub struct InvertedVoltageController<T: VoltageController>(pub T);

impl<T: VoltageController> From<T> for InvertedVoltageController<T> {
  fn from(value: T) -> Self {
    Self(value)
  }
}

impl<T: VoltageController> Wrapper<T> for InvertedVoltageController<T> {
  fn eject(self) -> T { self.0 }
}

impl<T: VoltageController> VoltageController for InvertedVoltageController<T> {
  #[inline(always)]
  fn set_voltage(&mut self, voltage: Voltage) {
    self.0.set_voltage(-voltage);
  }

  #[inline(always)]
  fn get_set_voltage(&self) -> Voltage {
    -self.0.get_set_voltage()
  }
}

impl<T: VoltageController + HasBusVoltage> HasBusVoltage for InvertedVoltageController<T> {
  #[inline(always)]
  fn get_bus_voltage(&self) -> Voltage {
    self.0.get_bus_voltage()
  }
}

#[derive(Debug, Clone)]
pub struct ClampedVoltageController<T: VoltageController> {
  controller: T,
  min_voltage: Voltage,
  max_voltage: Voltage,
}

impl<T: VoltageController> ClampedVoltageController<T> {
  pub fn new(min_voltage: Voltage, max_voltage: Voltage, vc: T) -> Self {
    Self { controller: vc, min_voltage, max_voltage }
  }
}

impl<T: VoltageController> Wrapper<T> for ClampedVoltageController<T> {
  fn eject(self) -> T {
    self.controller
  }
}

impl<T: VoltageController> VoltageController for ClampedVoltageController<T> {
  #[inline(always)]
  fn set_voltage(&mut self, voltage: Voltage) {
    self.controller.set_voltage(voltage.max(self.min_voltage).min(self.max_voltage))
  }

  #[inline(always)]
  fn get_set_voltage(&self) -> Voltage {
    self.controller.get_set_voltage()
  }
}

impl<T: VoltageController + HasBusVoltage> HasBusVoltage for ClampedVoltageController<T> {
  #[inline(always)]
  fn get_bus_voltage(&self) -> Voltage {
    self.controller.get_bus_voltage()
  }
}

#[cfg(feature = "ntcore")]
pub struct ObservableVoltageController<T: VoltageController> {
  controller: T,
  #[allow(unused)]
  topic: crate::ntcore::Topic,
  publisher: crate::ntcore::Publisher<f64>
}

#[cfg(feature = "ntcore")]
impl<T: VoltageController> ObservableVoltageController<T> {
  pub fn new(topic: crate::ntcore::Topic, vc: T) -> Self {
    Self { controller: vc, publisher: topic.child("voltage").publish(), topic }
  }
}

#[cfg(feature = "ntcore")]
impl<T: VoltageController> Wrapper<T> for ObservableVoltageController<T> {
  fn eject(self) -> T {
    self.controller
  }
}

#[cfg(feature = "ntcore")]
impl<T: VoltageController> VoltageController for ObservableVoltageController<T> {
  #[inline(always)]
  fn set_voltage(&mut self, voltage: Voltage) {
    self.controller.set_voltage(voltage);
    self.publisher.set(voltage.to_base()).ok();
  }

  #[inline(always)]
  fn get_set_voltage(&self) -> Voltage {
    self.controller.get_set_voltage()
  }
}

#[cfg(feature = "ntcore")]
impl<T: VoltageController + HasBusVoltage> HasBusVoltage for ObservableVoltageController<T> {
  #[inline(always)]
  fn get_bus_voltage(&self) -> Voltage {
    self.controller.get_bus_voltage()
  }
}

pub trait VoltageControllerExt : Sized + VoltageController {
  fn invert(self) -> InvertedVoltageController<Self>;
  fn clamp(self, min_voltage: Voltage, max_voltage: Voltage) -> ClampedVoltageController<Self>;

  #[cfg(feature = "ntcore")]
  fn observable(self, topic: crate::ntcore::Topic) -> ObservableVoltageController<Self>;
}

impl<T: VoltageController> VoltageControllerExt for T {
  fn invert(self) -> InvertedVoltageController<Self> {
    InvertedVoltageController::from(self)
  }

  fn clamp(self, min_voltage: Voltage, max_voltage: Voltage) -> ClampedVoltageController<Self> {
    ClampedVoltageController::new(min_voltage, max_voltage, self)
  }

  #[cfg(feature = "ntcore")]
  fn observable(self, topic: crate::ntcore::Topic) -> ObservableVoltageController<Self> {
    ObservableVoltageController::new(topic, self)
  }
}

#[cfg(feature = "simulation")]
pub mod sim {
  use std::sync::{RwLock, Arc};

  use robot_rs_units::electrical::Voltage;

  use super::VoltageController;

  #[derive(Debug, Clone)]
  pub struct SimulatedVoltageController {
    voltage: Arc<RwLock<Voltage>>
  }

  impl SimulatedVoltageController {
    pub fn new(initial_voltage: Voltage) -> Self {
      Self { voltage: Arc::new(RwLock::new(initial_voltage)) }
    }
  }

  impl VoltageController for SimulatedVoltageController {
    fn set_voltage(&mut self, voltage: Voltage) {
      *self.voltage.write().unwrap() = voltage;
    }

    fn get_set_voltage(&self) -> Voltage {
      self.voltage.read().unwrap().clone()
    }
  }
}