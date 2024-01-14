use std::f64::consts::PI;

use typenum::{P1, Z0};

use crate::unit;
use super::{Unit, Quantity, ISQ, QuantityBase};

// Base Units
pub type Unitless           = Quantity<ISQ<Z0, Z0, Z0, Z0, Z0, Z0, Z0, Z0, Z0>>;
pub type Time               = Quantity<ISQ<P1, Z0, Z0, Z0, Z0, Z0, Z0, Z0, Z0>>;
pub type Length             = Quantity<ISQ<Z0, P1, Z0, Z0, Z0, Z0, Z0, Z0, Z0>>;
pub type Mass               = Quantity<ISQ<Z0, Z0, P1, Z0, Z0, Z0, Z0, Z0, Z0>>;
pub type Current            = Quantity<ISQ<Z0, Z0, Z0, P1, Z0, Z0, Z0, Z0, Z0>>;
pub type Temperature        = Quantity<ISQ<Z0, Z0, Z0, Z0, P1, Z0, Z0, Z0, Z0>>;
pub type Molarity           = Quantity<ISQ<Z0, Z0, Z0, Z0, Z0, P1, Z0, Z0, Z0>>;
pub type LuminousIntensity  = Quantity<ISQ<Z0, Z0, Z0, Z0, Z0, Z0, P1, Z0, Z0>>;
pub type Angle              = Quantity<ISQ<Z0, Z0, Z0, Z0, Z0, Z0, Z0, P1, Z0>>;
pub type Ticks              = Quantity<ISQ<Z0, Z0, Z0, Z0, Z0, Z0, Z0, Z0, P1>>;

impl From<f64> for Unitless {
  fn from(value: f64) -> Self {
    Unitless::new::<ratio>(value)
  }
}

impl From<Unitless> for f64 {
  fn from(value: Unitless) -> Self {
    value.base_unit_value
  }
}

unit!(Unitless, ratio, 1.0, 0.0);

unit!(Time, second, 1.0, 0.0);
unit!(Time, millisecond, 1.0e-3, 0.0);
unit!(Time, microsecond, 1.0e-6, 0.0);
unit!(Time, nanosecond, 1.0e-9, 0.0);

unit!(Length, meter, 1.0, 0.0);
unit!(Length, centimeter, 1e-2, 0.0);
unit!(Length, millimeter, 1e-3, 0.0);
unit!(Length, micrometer, 1e-6, 0.0);
unit!(Length, nanometer, 1e-9, 0.0);
unit!(Length, kilometer, 1e3, 0.0);
unit!(Length, inch, 0.0254, 0.0);
unit!(Length, foot, 0.3048, 0.0);

unit!(Mass, kilogram, 1.0, 0.0);
unit!(Mass, gram, 1e-3, 0.0);

unit!(Current, ampere, 1.0, 0.0);
unit!(Current, milliampere, 1e-3, 0.0);

unit!(Temperature, kelvin, 1.0, 0.0);
unit!(Temperature, celsius, 0.0, 273.15);

unit!(Molarity, mol, 1.0, 0.0);
unit!(LuminousIntensity, candela, 1.0, 0.0);

unit!(Angle, radian, 1.0, 0.0);
unit!(Angle, degree, PI / 180.0, 0.0);

unit!(Ticks, tick, 1.0, 0.0);

#[cfg(test)]
mod tests {
  use std::f64::consts::PI;

  use approx::assert_relative_eq;

  use crate::base::*;

  #[test]
  fn test_singles() {
    // Just test a few choice things
    assert_relative_eq!(1.0 * second, 1.0 * second);
    assert_relative_eq!(1e-3 * second, 1.0 * millisecond);
    assert_relative_eq!(1e-6 * second, 1.0 * microsecond);
    assert_relative_eq!(1e-9 * second, 1.0 * nanosecond);

    assert_relative_eq!(1.0 * inch, 25.4 * millimeter);
    assert_relative_eq!(12.0 * inch, 1.0 * foot);

    assert_eq!(0.0 * celsius, 273.15 * kelvin);

    assert_eq!(2.0 * PI * radian, 360.0 * degree);
  }
}