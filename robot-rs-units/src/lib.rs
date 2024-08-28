pub mod base;
pub mod electrical;
pub mod force;
pub mod motion;
pub mod traits;

pub use base::*;
pub use typenum;

use std::{
  fmt::Debug,
  marker::PhantomData,
  ops::{Add, AddAssign, Div, Mul, Neg, Sub, SubAssign},
};

use approx::{AbsDiffEq, RelativeEq};
use num_traits::Zero;

// Inspired by uom, but with some changes to better support our use-case.
pub trait Dimension: Send + Sync + Unpin {
  type Time: typenum::Integer;
  type Length: typenum::Integer;
  type Mass: typenum::Integer;
  type Current: typenum::Integer;
  type Temperature: typenum::Integer;
  type Molarity: typenum::Integer;
  type LuminousIntensity: typenum::Integer;
  type Angle: typenum::Integer;
  type Tick: typenum::Integer;
}

pub type ISQ<Time, Length, Mass, Current, Temperature, Molarity, LuminousIntensity, Angle, Tick> =
  dyn Dimension<
    Time = Time,
    Length = Length,
    Mass = Mass,
    Current = Current,
    Temperature = Temperature,
    Molarity = Molarity,
    LuminousIntensity = LuminousIntensity,
    Angle = Angle,
    Tick = Tick,
  >;

pub trait QuantityBase: Sized {
  type Dimension: ?Sized;
  fn new<U: Unit<Self>>(value: f64) -> Self;
  fn from_base(value: f64) -> Self;
  fn to<U: Unit<Self>>(&self) -> f64;
  fn to_base(&self) -> f64;
}

pub trait Unit<Q> {
  const FACTOR_TO_BASE: f64;
  const OFFSET_FROM_BASE: f64 = 0f64;
}

// Quantities
pub struct Quantity<Dim: ?Sized + Dimension> {
  dimension: PhantomData<Dim>,
  base_unit_value: f64,
}

impl<Dim: ?Sized + Dimension> Clone for Quantity<Dim> {
  fn clone(&self) -> Self {
    Self {
      dimension: PhantomData,
      base_unit_value: self.base_unit_value.clone(),
    }
  }
}

impl<Dim: ?Sized + Dimension> Copy for Quantity<Dim> {}

impl<Dim: ?Sized + Dimension> QuantityBase for Quantity<Dim> {
  type Dimension = Dim;

  fn new<U: Unit<Self>>(value: f64) -> Self {
    Self {
      base_unit_value: (value * U::FACTOR_TO_BASE) + U::OFFSET_FROM_BASE,
      dimension: PhantomData,
    }
  }

  fn from_base(value: f64) -> Self {
    Self {
      base_unit_value: value,
      dimension: PhantomData,
    }
  }

  fn to<U: Unit<Self>>(&self) -> f64 {
    (self.base_unit_value - U::OFFSET_FROM_BASE) / U::FACTOR_TO_BASE
  }

  fn to_base(&self) -> f64 {
    self.base_unit_value
  }
}

fn format_unit(f: &mut std::fmt::Formatter<'_>, value: isize, abbrev: &str) -> std::fmt::Result {
  if value == 1 {
    write!(f, " {}", abbrev)?;
  } else if value != 0 {
    write!(f, " {}^{{{}}}", abbrev, value)?;
  }
  Ok(())
}

impl<Dim: ?Sized + Dimension> Debug for Quantity<Dim> {
  fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
    write!(f, "{}", self.base_unit_value)?;
    format_unit(f, <Dim::Mass as typenum::Integer>::to_isize(), "kg")?;
    format_unit(f, <Dim::Length as typenum::Integer>::to_isize(), "m")?;
    format_unit(f, <Dim::Time as typenum::Integer>::to_isize(), "s")?;
    format_unit(f, <Dim::Current as typenum::Integer>::to_isize(), "A")?;
    format_unit(f, <Dim::Temperature as typenum::Integer>::to_isize(), "K")?;
    format_unit(f, <Dim::Molarity as typenum::Integer>::to_isize(), "mol")?;
    format_unit(
      f,
      <Dim::LuminousIntensity as typenum::Integer>::to_isize(),
      "cd",
    )?;
    format_unit(f, <Dim::Angle as typenum::Integer>::to_isize(), "rad")?;
    Ok(())
  }
}

impl<Dim: ?Sized + Dimension> std::fmt::Display for Quantity<Dim> {
  fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
    <Self as Debug>::fmt(&self, f)
  }
}

impl<D: Dimension + ?Sized> Neg for Quantity<D> {
  type Output = Quantity<D>;

  fn neg(self) -> Self::Output {
    Quantity {
      dimension: PhantomData,
      base_unit_value: -self.base_unit_value,
    }
  }
}

impl<D: Dimension + ?Sized> Mul<f64> for Quantity<D> {
  type Output = Quantity<D>;

  fn mul(self, rhs: f64) -> Self::Output {
    Quantity {
      dimension: PhantomData,
      base_unit_value: self.base_unit_value * rhs,
    }
  }
}

impl<D: Dimension + ?Sized> Mul<Quantity<D>> for f64 {
  type Output = Quantity<D>;

  fn mul(self, rhs: Quantity<D>) -> Self::Output {
    Quantity {
      dimension: PhantomData,
      base_unit_value: self * rhs.base_unit_value,
    }
  }
}

impl<D: Dimension + ?Sized> Div<f64> for Quantity<D> {
  type Output = Quantity<D>;

  fn div(self, rhs: f64) -> Self::Output {
    Quantity {
      dimension: PhantomData,
      base_unit_value: self.base_unit_value / rhs,
    }
  }
}

impl<D: Dimension + ?Sized> Div<Quantity<D>> for f64 {
  type Output = Quantity<D>;

  fn div(self, rhs: Quantity<D>) -> Self::Output {
    Quantity {
      dimension: PhantomData,
      base_unit_value: self / rhs.base_unit_value,
    }
  }
}

impl<D: Dimension + ?Sized> Add<Quantity<D>> for Quantity<D> {
  type Output = Quantity<D>;

  fn add(self, rhs: Quantity<D>) -> Self::Output {
    Quantity {
      dimension: PhantomData,
      base_unit_value: self.base_unit_value + rhs.base_unit_value,
    }
  }
}

impl<D: Dimension + ?Sized> AddAssign<Quantity<D>> for Quantity<D> {
  fn add_assign(&mut self, rhs: Quantity<D>) {
    self.base_unit_value += rhs.base_unit_value
  }
}

impl<D: Dimension + ?Sized> SubAssign<Quantity<D>> for Quantity<D> {
  fn sub_assign(&mut self, rhs: Quantity<D>) {
    self.base_unit_value += rhs.base_unit_value
  }
}

impl<D: Dimension + ?Sized> Sub<Quantity<D>> for Quantity<D> {
  type Output = Quantity<D>;

  fn sub(self, rhs: Quantity<D>) -> Self::Output {
    Quantity {
      dimension: PhantomData,
      base_unit_value: self.base_unit_value - rhs.base_unit_value,
    }
  }
}

impl<Dl: Dimension + ?Sized, Dr: Dimension + ?Sized> Mul<Quantity<Dr>> for Quantity<Dl>
where
  Dl::Time: Add<Dr::Time>,
  <Dl::Time as Add<Dr::Time>>::Output: typenum::Integer,
  Dl::Length: Add<Dr::Length>,
  <Dl::Length as Add<Dr::Length>>::Output: typenum::Integer,
  Dl::Mass: Add<Dr::Mass>,
  <Dl::Mass as Add<Dr::Mass>>::Output: typenum::Integer,
  Dl::Current: Add<Dr::Current>,
  <Dl::Current as Add<Dr::Current>>::Output: typenum::Integer,
  Dl::Temperature: Add<Dr::Temperature>,
  <Dl::Temperature as Add<Dr::Temperature>>::Output: typenum::Integer,
  Dl::Molarity: Add<Dr::Molarity>,
  <Dl::Molarity as Add<Dr::Molarity>>::Output: typenum::Integer,
  Dl::LuminousIntensity: Add<Dr::LuminousIntensity>,
  <Dl::LuminousIntensity as Add<Dr::LuminousIntensity>>::Output: typenum::Integer,
  Dl::Angle: Add<Dr::Angle>,
  <Dl::Angle as Add<Dr::Angle>>::Output: typenum::Integer,
  Dl::Tick: Add<Dr::Tick>,
  <Dl::Tick as Add<Dr::Tick>>::Output: typenum::Integer,
{
  type Output = Quantity<
    ISQ<
      <Dl::Time as Add<Dr::Time>>::Output,
      <Dl::Length as Add<Dr::Length>>::Output,
      <Dl::Mass as Add<Dr::Mass>>::Output,
      <Dl::Current as Add<Dr::Current>>::Output,
      <Dl::Temperature as Add<Dr::Temperature>>::Output,
      <Dl::Molarity as Add<Dr::Molarity>>::Output,
      <Dl::LuminousIntensity as Add<Dr::LuminousIntensity>>::Output,
      <Dl::Angle as Add<Dr::Angle>>::Output,
      <Dl::Tick as Add<Dr::Tick>>::Output,
    >,
  >;

  fn mul(self, rhs: Quantity<Dr>) -> Self::Output {
    Quantity {
      dimension: PhantomData,
      base_unit_value: self.base_unit_value * rhs.base_unit_value,
    }
  }
}

impl<Dl: Dimension + ?Sized, Dr: Dimension + ?Sized> Div<Quantity<Dr>> for Quantity<Dl>
where
  Dl::Time: Sub<Dr::Time>,
  <Dl::Time as Sub<Dr::Time>>::Output: typenum::Integer,
  Dl::Length: Sub<Dr::Length>,
  <Dl::Length as Sub<Dr::Length>>::Output: typenum::Integer,
  Dl::Mass: Sub<Dr::Mass>,
  <Dl::Mass as Sub<Dr::Mass>>::Output: typenum::Integer,
  Dl::Current: Sub<Dr::Current>,
  <Dl::Current as Sub<Dr::Current>>::Output: typenum::Integer,
  Dl::Temperature: Sub<Dr::Temperature>,
  <Dl::Temperature as Sub<Dr::Temperature>>::Output: typenum::Integer,
  Dl::Molarity: Sub<Dr::Molarity>,
  <Dl::Molarity as Sub<Dr::Molarity>>::Output: typenum::Integer,
  Dl::LuminousIntensity: Sub<Dr::LuminousIntensity>,
  <Dl::LuminousIntensity as Sub<Dr::LuminousIntensity>>::Output: typenum::Integer,
  Dl::Angle: Sub<Dr::Angle>,
  <Dl::Angle as Sub<Dr::Angle>>::Output: typenum::Integer,
  Dl::Tick: Sub<Dr::Tick>,
  <Dl::Tick as Sub<Dr::Tick>>::Output: typenum::Integer,
{
  type Output = Quantity<
    ISQ<
      <Dl::Time as Sub<Dr::Time>>::Output,
      <Dl::Length as Sub<Dr::Length>>::Output,
      <Dl::Mass as Sub<Dr::Mass>>::Output,
      <Dl::Current as Sub<Dr::Current>>::Output,
      <Dl::Temperature as Sub<Dr::Temperature>>::Output,
      <Dl::Molarity as Sub<Dr::Molarity>>::Output,
      <Dl::LuminousIntensity as Sub<Dr::LuminousIntensity>>::Output,
      <Dl::Angle as Sub<Dr::Angle>>::Output,
      <Dl::Tick as Sub<Dr::Tick>>::Output,
    >,
  >;

  fn div(self, rhs: Quantity<Dr>) -> Self::Output {
    Quantity {
      dimension: PhantomData,
      base_unit_value: self.base_unit_value / rhs.base_unit_value,
    }
  }
}

impl<D: Dimension + ?Sized> PartialEq<Quantity<D>> for Quantity<D> {
  fn eq(&self, other: &Quantity<D>) -> bool {
    self.base_unit_value.eq(&other.base_unit_value)
  }
}

impl<D: Dimension + ?Sized> Eq for Quantity<D> {}

impl<D: Dimension + ?Sized> PartialOrd<Quantity<D>> for Quantity<D> {
  fn partial_cmp(&self, other: &Quantity<D>) -> Option<std::cmp::Ordering> {
    self.base_unit_value.partial_cmp(&other.base_unit_value)
  }
}

impl<D: Dimension + ?Sized> AbsDiffEq<Quantity<D>> for Quantity<D> {
  type Epsilon = Quantity<D>;

  fn default_epsilon() -> Self::Epsilon {
    Quantity {
      base_unit_value: f64::default_epsilon(),
      dimension: PhantomData,
    }
  }

  fn abs_diff_eq(&self, other: &Quantity<D>, epsilon: Self::Epsilon) -> bool {
    other
      .base_unit_value
      .abs_diff_eq(&other.base_unit_value, epsilon.base_unit_value)
  }
}

impl<D: Dimension + ?Sized> RelativeEq<Quantity<D>> for Quantity<D> {
  fn default_max_relative() -> Self::Epsilon {
    Quantity {
      base_unit_value: f64::default_max_relative(),
      dimension: PhantomData,
    }
  }

  fn relative_eq(
    &self,
    other: &Quantity<D>,
    epsilon: Self::Epsilon,
    max_relative: Self::Epsilon,
  ) -> bool {
    other.base_unit_value.relative_eq(
      &other.base_unit_value,
      epsilon.base_unit_value,
      max_relative.base_unit_value,
    )
  }
}

impl<D: Dimension + ?Sized> Zero for Quantity<D> {
  fn zero() -> Self {
    Quantity {
      dimension: PhantomData,
      base_unit_value: f64::zero(),
    }
  }

  fn is_zero(&self) -> bool {
    self.base_unit_value.is_zero()
  }
}

#[macro_export]
macro_rules! unit {
  ($qty:path, $name:ident, $factor:expr, $offset:expr) => {
    #[allow(non_camel_case_types)]
    pub struct $name;
    impl Unit<$qty> for $name {
      const FACTOR_TO_BASE: f64 = $factor;
      const OFFSET_FROM_BASE: f64 = $offset;
    }

    impl std::ops::Mul<$name> for f64 {
      type Output = $qty;

      fn mul(self, _rhs: $name) -> Self::Output {
        <$qty>::new::<$name>(self)
      }
    }
  };
}
