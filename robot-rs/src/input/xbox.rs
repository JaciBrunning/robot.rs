use crate::traits::Wrapper;

#[cfg(feature = "hal")]
use super::hid::DriverStationHID;
use super::hid::HIDDevice;

pub trait XboxController {
  type HID: HIDDevice;

  fn inner(&self) -> &Self::HID;

  fn left_x(&self) -> <Self::HID as HIDDevice>::Axis;
  fn left_y(&self) -> <Self::HID as HIDDevice>::Axis;
  fn left_trigger(&self) -> <Self::HID as HIDDevice>::Axis;
  fn right_trigger(&self) -> <Self::HID as HIDDevice>::Axis;
  fn right_x(&self) -> <Self::HID as HIDDevice>::Axis;
  fn right_y(&self) -> <Self::HID as HIDDevice>::Axis;

  fn dpad(&self) -> <Self::HID as HIDDevice>::POV;

  fn a(&self) -> <Self::HID as HIDDevice>::Button;
  fn b(&self) -> <Self::HID as HIDDevice>::Button;
  fn x(&self) -> <Self::HID as HIDDevice>::Button;
  fn y(&self) -> <Self::HID as HIDDevice>::Button;
  fn left_bumper(&self) -> <Self::HID as HIDDevice>::Button;
  fn right_bumper(&self) -> <Self::HID as HIDDevice>::Button;
  fn back(&self) -> <Self::HID as HIDDevice>::Button;
  fn start(&self) -> <Self::HID as HIDDevice>::Button;
  fn left_stick(&self) -> <Self::HID as HIDDevice>::Button;
  fn right_stick(&self) -> <Self::HID as HIDDevice>::Button;
}

#[derive(Debug, Clone)]
pub struct Xbox<HID: HIDDevice>(HID);

impl<HID: HIDDevice> From<HID> for Xbox<HID> {
  fn from(value: HID) -> Self {
    Xbox(value)
  }
}

impl<HID: HIDDevice> Wrapper<HID> for Xbox<HID> {
  fn eject(self) -> HID {
    self.0
  }
}

#[cfg(feature = "hal")]
impl Xbox<DriverStationHID> {
  pub fn from_driver_station(port: usize) -> Self {
    Self::new(super::hid::DriverStationHID::new(port))
  }
}

impl<HID: HIDDevice> Xbox<HID> {
  pub fn new(hid: HID) -> Self {
    Xbox(hid)
  }
}

impl<HID: HIDDevice> XboxController for Xbox<HID> {
  type HID = HID;

  fn inner(&self) -> &HID {
    &self.0
  }

  fn left_x(&self) -> HID::Axis {
    self.0.axis(0)
  }
  fn left_y(&self) -> HID::Axis {
    self.0.axis(1)
  }
  fn left_trigger(&self) -> HID::Axis {
    self.0.axis(2)
  }
  fn right_trigger(&self) -> HID::Axis {
    self.0.axis(3)
  }
  fn right_x(&self) -> HID::Axis {
    self.0.axis(4)
  }
  fn right_y(&self) -> HID::Axis {
    self.0.axis(5)
  }

  fn dpad(&self) -> HID::POV {
    self.0.pov(0)
  }

  fn a(&self) -> HID::Button {
    self.0.button(1)
  }
  fn b(&self) -> HID::Button {
    self.0.button(2)
  }
  fn x(&self) -> HID::Button {
    self.0.button(3)
  }
  fn y(&self) -> HID::Button {
    self.0.button(4)
  }
  fn left_bumper(&self) -> HID::Button {
    self.0.button(5)
  }
  fn right_bumper(&self) -> HID::Button {
    self.0.button(6)
  }
  fn back(&self) -> HID::Button {
    self.0.button(7)
  }
  fn start(&self) -> HID::Button {
    self.0.button(8)
  }
  fn left_stick(&self) -> HID::Button {
    self.0.button(9)
  }
  fn right_stick(&self) -> HID::Button {
    self.0.button(10)
  }
}
