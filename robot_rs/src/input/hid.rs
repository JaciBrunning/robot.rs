use std::ffi::CStr;

use wpilib_hal::{HAL_GetJoystickButtons, HAL_JoystickButtons, HAL_JoystickAxes, HAL_GetJoystickAxes, HAL_JoystickPOVs, HAL_GetJoystickPOVs, HAL_JoystickDescriptor, HAL_GetJoystickDescriptor};

use crate::{sensors::{digital::DigitalInput, analog::AnalogInput}, control::edge_detect::{Edge, EdgeDetector, EdgeDetectorOwned}};

/* BUTTONS */

pub struct HIDButton {
  port: usize,
  index: usize
}

impl HIDButton {
  pub fn new(hid: &HID, index: usize) -> Self {
    HIDButton { port: hid.port, index }
  }

  pub fn edge(&self, edge: Edge) -> EdgeDetector {
    EdgeDetector::new(self, edge)
  }

  pub fn edge_take(self, edge: Edge) -> EdgeDetectorOwned<Self> {
    EdgeDetectorOwned::new(self, edge)
  }
}

impl DigitalInput for HIDButton {
  fn get(&self) -> bool {
    let mut buttons = HAL_JoystickButtons::default();
    unsafe { HAL_GetJoystickButtons(self.port as i32, &mut buttons) };
    if self.index > buttons.count as usize || self.index < 1 {
      false
    } else {
      buttons.buttons & (1 << (self.index - 1)) != 0
    }
  }
}

/* AXES */

pub struct HIDAxis {
  port: usize,
  index: usize
}

impl HIDAxis {
  pub fn new(hid: &HID, index: usize) -> Self {
    Self { port: hid.port, index }
  }
}

impl AnalogInput for HIDAxis {
  fn get(&self) -> f64 {
    let mut axes = HAL_JoystickAxes::default();
    unsafe { HAL_GetJoystickAxes(self.port as i32, &mut axes) };
    *axes.axes.get(self.index).unwrap_or(&0.0) as f64
  }
}

/* POVs */

pub struct HIDPOV {
  port: usize,
  index: usize
}

impl HIDPOV {
  pub fn new(hid: &HID, index: usize) -> Self {
    Self { port: hid.port, index }
  }

  pub fn get(&self) -> isize {
    let mut povs = HAL_JoystickPOVs::default();
    unsafe { HAL_GetJoystickPOVs(self.port as i32, &mut povs) };
    *povs.povs.get(self.index).unwrap_or(&-1) as isize
  }
}

/* HID */

pub struct HID {
  port: usize
}

impl HID {
  pub fn new(port: usize) -> Self {
    Self { port }
  }

  pub fn port(&self) -> usize { self.port }

  pub fn button(&self, index: usize) -> HIDButton {
    HIDButton::new(&self, index)
  }

  pub fn axis(&self, index: usize) -> HIDAxis {
    HIDAxis::new(&self, index)
  }

  pub fn pov(&self, index: usize) -> HIDPOV {
    HIDPOV::new(&self, index)
  }

  fn descriptor(&self) -> HAL_JoystickDescriptor {
    let mut descriptor = HAL_JoystickDescriptor::default();
    unsafe { HAL_GetJoystickDescriptor(self.port as i32, &mut descriptor) };
    descriptor
  }

  pub fn n_buttons(&self) -> usize {
    self.descriptor().buttonCount as usize
  }

  pub fn n_axes(&self) -> usize {
    self.descriptor().axisCount as usize
  }

  pub fn n_pov(&self) -> usize {
    self.descriptor().povCount as usize
  }

  pub fn name(&self) -> String {
    unsafe { CStr::from_ptr(self.descriptor().name.as_ptr()).to_string_lossy().to_string() }
  }
}