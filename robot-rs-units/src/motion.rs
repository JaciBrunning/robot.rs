use std::f64::consts::PI;

use typenum::{N1, P1, Z0, N2, N3};

use super::{Unit, Quantity, ISQ, QuantityBase};
use crate::unit;

pub type Velocity         = Quantity<ISQ<N1, P1, Z0, Z0, Z0, Z0, Z0, Z0, Z0>>;
pub type Acceleration     = Quantity<ISQ<N2, P1, Z0, Z0, Z0, Z0, Z0, Z0, Z0>>;
pub type Jerk             = Quantity<ISQ<N3, P1, Z0, Z0, Z0, Z0, Z0, Z0, Z0>>;

pub type AngularVelocity      = Quantity<ISQ<N1, Z0, Z0, Z0, Z0, Z0, Z0, P1, Z0>>;
pub type AngularAcceleration  = Quantity<ISQ<N2, Z0, Z0, Z0, Z0, Z0, Z0, P1, Z0>>;
pub type AngularJerk          = Quantity<ISQ<N3, Z0, Z0, Z0, Z0, Z0, Z0, P1, Z0>>;

pub type TickVelocity         = Quantity<ISQ<N1, Z0, Z0, Z0, Z0, Z0, Z0, Z0, P1>>;

unit!(Velocity, meters_per_second, 1.0, 0.0);
unit!(Velocity, inches_per_second, 0.0254, 0.0);
unit!(Velocity, feet_per_second, 0.3048, 0.0);
unit!(Velocity, kilometers_per_hour, 0.277778, 0.0);
unit!(Velocity, miles_per_hour, 0.44704, 0.0);

unit!(Acceleration, meters_per_second2, 1.0, 0.0);
unit!(Acceleration, inches_per_second2, 0.0254, 0.0);
unit!(Acceleration, feet_per_second2, 0.3048, 0.0);

unit!(Jerk, meters_per_second3, 1.0, 0.0);
unit!(Jerk, inches_per_second3, 0.0254, 0.0);
unit!(Jerk, feet_per_second3, 0.3048, 0.0);

unit!(AngularVelocity, rads_per_second, 1.0, 0.0);
unit!(AngularVelocity, degrees_per_second, PI / 180.0, 0.0);
unit!(AngularVelocity, revolutions_per_minute, 2.0*PI / 60.0, 0.0);

unit!(AngularAcceleration, rads_per_second2, 1.0, 0.0);
unit!(AngularAcceleration, degrees_per_second2, PI / 180.0, 0.0);

unit!(AngularJerk, rads_per_second3, 1.0, 0.0);
unit!(AngularJerk, degrees_per_second3, PI / 180.0, 0.0);

unit!(TickVelocity, ticks_per_second, 1.0, 0.0);

#[allow(non_camel_case_types)]
pub type rpm = revolutions_per_minute;

#[cfg(test)]
mod tests {
  use approx::assert_relative_eq;

  use crate::{base::{meter, second}, motion::feet_per_second};

  use super::Velocity;

  #[test]
  fn test_conversions() {
    let vel: Velocity = (12.0 * meter) / (1.0 * second);
    assert_relative_eq!(vel, 39.3701 * feet_per_second, epsilon = 0.001 * feet_per_second);
  }
}