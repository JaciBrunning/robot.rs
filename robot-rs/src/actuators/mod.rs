use crate::traits::Wrapper;

pub trait VoltageController {
  fn set_voltage(&mut self, voltage: f64);
  fn get_set_voltage(&mut self) -> f64;
}

pub trait VoltageControllerTraits : Sized + VoltageController {
  fn invert(self) -> InvertedVoltageController<Self>;
  fn clamp(self, min_voltage: f64, max_voltage: f64) -> ClampedVoltageController<Self>;
}

impl<T: VoltageController> VoltageControllerTraits for T {
  fn invert(self) -> InvertedVoltageController<Self> {
    InvertedVoltageController::from(self)
  }

  fn clamp(self, min_voltage: f64, max_voltage: f64) -> ClampedVoltageController<Self> {
    ClampedVoltageController::new(self, min_voltage, max_voltage)
  }
}

pub trait HasBusVoltage {
  fn get_bus_voltage(&self) -> f64;
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
  fn set_voltage(&mut self, voltage: f64) {
    self.0.set_voltage(-voltage);
  }

  #[inline(always)]
  fn get_set_voltage(&mut self) -> f64 {
    -self.0.get_set_voltage()
  }
}

impl<T: VoltageController + HasBusVoltage> HasBusVoltage for InvertedVoltageController<T> {
  #[inline(always)]
  fn get_bus_voltage(&self) -> f64 {
    self.0.get_bus_voltage()
  }
}

#[derive(Debug, Clone)]
pub struct ClampedVoltageController<T: VoltageController> {
  controller: T,
  min_voltage: f64,
  max_voltage: f64,
}

impl<T: VoltageController> ClampedVoltageController<T> {
  pub fn new(vc: T, min_voltage: f64, max_voltage: f64) -> Self {
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
  fn set_voltage(&mut self, voltage: f64) {
    self.controller.set_voltage(voltage.max(self.min_voltage).min(self.max_voltage))
  }

  #[inline(always)]
  fn get_set_voltage(&mut self) -> f64 {
    self.controller.get_set_voltage()
  }
}

impl<T: VoltageController + HasBusVoltage> HasBusVoltage for ClampedVoltageController<T> {
  #[inline(always)]
  fn get_bus_voltage(&self) -> f64 {
    self.controller.get_bus_voltage()
  }
}
