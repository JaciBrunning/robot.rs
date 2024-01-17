#[cfg(feature = "hal")]
use robot_rs_wpilib_sys::{HAL_JoystickDescriptor, HAL_GetJoystickDescriptor, HAL_JoystickButtons, HAL_GetJoystickButtons, HAL_JoystickAxes, HAL_GetJoystickAxes, HAL_GetJoystickPOVs, HAL_JoystickPOVs};

use crate::sensors::Sensor;

pub trait HIDDevice {
  type Button: Sensor<bool>;
  type Axis: Sensor<f64>;
  type POV: Sensor<isize>;

  // Note that these aren't Option<>s, primarily since the joystick descriptor might change
  // throughout the course of the user program as the DS connects and disconnects, hence 
  // we defer failures to Option<> in Sensor returning None
  fn button(&self, index: usize) -> Self::Button;
  fn axis(&self, index: usize) -> Self::Axis;
  fn pov(&self, index: usize) -> Self::POV;

  fn n_buttons(&self) -> usize;
  fn n_axes(&self) -> usize;
  fn n_pov(&self) -> usize;

  fn name(&self) -> Option<String>;
}

#[cfg(feature = "hal")]
#[derive(Debug, Clone, Copy)]
pub struct DriverStationHIDButton {
  port: usize, index: usize
}

#[cfg(feature = "hal")]
impl Sensor<bool> for DriverStationHIDButton {
  fn get_sensor_value(&self) -> bool {
    let mut buttons = HAL_JoystickButtons::default();
    unsafe { HAL_GetJoystickButtons(self.port as i32, &mut buttons) };
    if self.index > buttons.count as usize || self.index < 1 {
      false
    } else {
      buttons.buttons & (1 << (self.index - 1)) != 0
    }
  }

  fn get_last_measurement_time(&self) -> robot_rs_units::Time {
    crate::time::now()
  }
}

#[cfg(feature = "hal")]
#[derive(Debug, Clone, Copy)]
pub struct DriverStationHIDAxis {
  port: usize, index: usize
}

#[cfg(feature = "hal")]
impl Sensor<f64> for DriverStationHIDAxis {
  fn get_sensor_value(&self) -> f64 {
    let mut axes = HAL_JoystickAxes::default();
    unsafe { HAL_GetJoystickAxes(self.port as i32, &mut axes) };
    *axes.axes.get(self.index).unwrap_or(&0.0) as f64
  }

  fn get_last_measurement_time(&self) -> robot_rs_units::Time {
    crate::time::now()
  }
}

#[cfg(feature = "hal")]
#[derive(Debug, Clone, Copy)]
pub struct DriverStationHIDPOV {
  port: usize, index: usize
}

#[cfg(feature = "hal")]
impl Sensor<isize> for DriverStationHIDPOV {
  fn get_sensor_value(&self) -> isize {
    let mut povs = HAL_JoystickPOVs::default();
    unsafe { HAL_GetJoystickPOVs(self.port as i32, &mut povs) };
    *povs.povs.get(self.index).unwrap_or(&0) as isize
  }

  fn get_last_measurement_time(&self) -> robot_rs_units::Time {
    crate::time::now()
  }
}

#[cfg(feature = "hal")]
#[derive(Debug, Clone)]
pub struct DriverStationHID {
  port: usize
}

#[cfg(feature = "hal")]
impl DriverStationHID {
  pub fn new(port: usize) -> Self {
    Self { port }
  }

  fn get_descriptor(&self) -> Option<HAL_JoystickDescriptor> {
    let mut descriptor = HAL_JoystickDescriptor::default();
    if unsafe { HAL_GetJoystickDescriptor(self.port as i32, &mut descriptor) } == 0 {
      Some(descriptor)
    } else {
      None
    }
  }
}

#[cfg(feature = "hal")]
impl HIDDevice for DriverStationHID {
  type Button = DriverStationHIDButton;
  type Axis = DriverStationHIDAxis;
  type POV = DriverStationHIDPOV;

  fn button(&self, index: usize) -> Self::Button {
    DriverStationHIDButton { port: self.port, index }
  }

  fn axis(&self, index: usize) -> Self::Axis {
    DriverStationHIDAxis { port: self.port, index }
  }

  fn pov(&self, index: usize) -> Self::POV {
    DriverStationHIDPOV { port: self.port, index }
  }

  fn n_buttons(&self) -> usize {
    self.get_descriptor().map(|x| x.buttonCount as usize).unwrap_or(0)
  }

  fn n_axes(&self) -> usize {
    self.get_descriptor().map(|x| x.axisCount as usize).unwrap_or(0)
  }

  fn n_pov(&self) -> usize {
    self.get_descriptor().map(|x| x.povCount as usize).unwrap_or(0)
  }

  fn name(&self) -> Option<String> {
    unsafe { Some(std::ffi::CStr::from_ptr(self.get_descriptor()?.name.as_ptr()).to_string_lossy().to_string()) }
  }
}