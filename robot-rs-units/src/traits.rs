use std::ops::{Add, Sub};

use num_traits::{Zero, Float, ToPrimitive};

use crate::{Dimension, Quantity, QuantityBase};

pub trait Angle {
  fn sin(self) -> f64;
  fn cos(self) -> f64;
  fn tan(self) -> f64;
}

impl Angle for crate::base::Angle {
  fn sin(self) -> f64 {
    self.base_unit_value.sin()
  }

  fn cos(self) -> f64 {
    self.base_unit_value.cos()
  }

  fn tan(self) -> f64 {
    self.base_unit_value.tan()
  }
}

impl Angle for f64 {
  fn sin(self) -> f64 {
    self.sin()
  }

  fn cos(self) -> f64 {
    self.cos()
  }

  fn tan(self) -> f64 {
    self.tan()
  }
}

pub fn sin<A: Angle>(angle: A) -> f64 {
  angle.sin()
}

pub fn cos<A: Angle>(angle: A) -> f64 {
  angle.cos()
}

pub fn tan<A: Angle>(angle: A) -> f64 {
  angle.tan()
}

pub trait MaybeUnitNumber : Zero + Add<Self, Output = Self> + Sub<Self, Output = Self> {
  fn one() -> Self;
  fn abs(self) -> Self;
  fn signum(self) -> Self;
  fn is_positive(self) -> bool;
  fn is_negative(self) -> bool;

  fn floor(self) -> Self;
  fn ceil(self) -> Self;
  fn round(self) -> Self;

  fn max(self, other: Self) -> Self;
  fn min(self, other: Self) -> Self;
}

impl<D: Dimension + ?Sized> MaybeUnitNumber for Quantity<D> {
  fn one() -> Self {
    Quantity::from_base(1.0)
  }

  fn abs(self) -> Self {
    Quantity::from_base(self.base_unit_value.abs())
  }

  fn signum(self) -> Self {
    Quantity::from_base(self.base_unit_value.signum())
  }

  fn is_positive(self) -> bool {
    self.base_unit_value > 0.0
  }

  fn is_negative(self) -> bool {
    self.base_unit_value < 0.0
  }

  fn floor(self) -> Self {
    Quantity::from_base(self.base_unit_value.floor())
  }

  fn ceil(self) -> Self {
    Quantity::from_base(self.base_unit_value.ceil())
  }

  fn round(self) -> Self {
    Quantity::from_base(self.base_unit_value.round())
  }

  fn max(self, other: Self) -> Self {
    Quantity::from_base(self.base_unit_value.max(other.base_unit_value))
  }

  fn min(self, other: Self) -> Self {
    Quantity::from_base(self.base_unit_value.min(other.base_unit_value))
  }
}

impl<T: Float> MaybeUnitNumber for T {
  fn one() -> Self {
    T::one()
  }

  fn abs(self) -> Self {
    T::abs(self)
  }

  fn signum(self) -> Self {
    T::signum(self)
  }

  fn is_positive(self) -> bool {
    T::is_sign_positive(self)
  }

  fn is_negative(self) -> bool {
    T::is_sign_negative(self)
  }

  fn floor(self) -> Self {
    T::floor(self)
  }

  fn ceil(self) -> Self {
    T::ceil(self)
  }

  fn round(self) -> Self {
    T::round(self)
  }

  fn max(self, other: Self) -> Self {
    T::max(self, other)
  }

  fn min(self, other: Self) -> Self {
    T::min(self, other)
  }
}

pub trait ToFloat {
  fn to_f64(self) -> f64;
}

impl ToFloat for f64 {
  fn to_f64(self) -> f64 {
    self
  }
}

impl ToFloat for f32 {
  fn to_f64(self) -> f64 {
    self as f64
  }
}

impl<T: QuantityBase> ToFloat for T {
  fn to_f64(self) -> f64 {
    self.to_base()
  }
}

pub trait FromFloat {
  fn from_f64(value: f64) -> Self;
}

impl FromFloat for f64 {
  fn from_f64(value: f64) -> Self {
    value
  }
}

impl FromFloat for f32 {
  fn from_f64(value: f64) -> Self {
    value as f32
  }
}

impl<T: QuantityBase> FromFloat for T {
  fn from_f64(value: f64) -> Self {
    Self::from_base(value)
  }
}