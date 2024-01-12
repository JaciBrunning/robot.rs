#[cfg(feature = "hal")]
pub use robot_rs_wpilib_sys as hal;

pub use robot_rs_macros as macros;

pub mod actuators;
pub mod activity;
pub mod filters;
pub mod physics;
pub mod sensors;

pub mod ds;
pub mod start;
pub mod time;
pub mod traits;

pub mod units {
  use std::marker::PhantomData;

  use uom::{Kind, typenum::{Z0, N1}};
  pub use uom::si::*;
  pub use uom::si::f64::*;

  pub trait EncoderTickKind: Kind {}
  pub type EncoderTicks = Quantity<uom::si::ISQ<Z0, Z0, Z0, Z0, Z0, Z0, Z0, dyn EncoderTickKind>, uom::si::SI<f64>, f64>;
  pub const ENCODER_TICKS: EncoderTicks = EncoderTicks { dimension: PhantomData, units: PhantomData, value: 1.0 };

  pub type EncoderTickVelocity = Quantity<uom::si::ISQ<Z0, Z0, N1, Z0, Z0, Z0, Z0, dyn EncoderTickKind>, uom::si::SI<f64>, f64>;
  pub const ENCODER_TICKS_PER_SECOND: EncoderTickVelocity = EncoderTickVelocity { dimension: PhantomData, units: PhantomData, value: 1.0 };
}