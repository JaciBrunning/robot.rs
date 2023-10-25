use crate::macros::wrapped_traits_nogen;

use super::hid::{HIDAxis, HIDButton, HID, HIDPOV};

pub struct Xbox(HID);

impl From<HID> for Xbox {
    fn from(value: HID) -> Self {
        Self(value)
    }
}

impl Xbox {
    pub fn new(port: usize) -> Self {
        Self(HID::new(port))
    }

    pub fn left_x(&self) -> HIDAxis {
        self.axis(0)
    }
    pub fn left_y(&self) -> HIDAxis {
        self.axis(1)
    }
    pub fn left_trigger(&self) -> HIDAxis {
        self.axis(2)
    }
    pub fn right_trigger(&self) -> HIDAxis {
        self.axis(3)
    }
    pub fn right_x(&self) -> HIDAxis {
        self.axis(4)
    }
    pub fn right_y(&self) -> HIDAxis {
        self.axis(5)
    }

    pub fn dpad(&self) -> HIDPOV {
        self.pov(0)
    }

    pub fn a(&self) -> HIDButton {
        self.button(1)
    }
    pub fn b(&self) -> HIDButton {
        self.button(2)
    }
    pub fn x(&self) -> HIDButton {
        self.button(3)
    }
    pub fn y(&self) -> HIDButton {
        self.button(4)
    }
    pub fn left_bumper(&self) -> HIDButton {
        self.button(5)
    }
    pub fn right_bumper(&self) -> HIDButton {
        self.button(6)
    }
    pub fn back(&self) -> HIDButton {
        self.button(7)
    }
    pub fn start(&self) -> HIDButton {
        self.button(8)
    }
    pub fn left_stick(&self) -> HIDButton {
        self.button(9)
    }
    pub fn right_stick(&self) -> HIDButton {
        self.button(10)
    }
}

wrapped_traits_nogen!(Xbox, HID);
