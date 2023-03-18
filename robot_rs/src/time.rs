use wpilib_hal::hal_safe_call;
use wpilib_hal::HAL_GetFPGATime;

#[cfg(feature = "units")]
use uom::si::{f64::Time, time::second};

pub fn now() -> f64 {
  hal_safe_call!(HAL_GetFPGATime()).unwrap() as f64 * 1e-6
}

#[cfg(feature = "units")]
pub fn now_t() -> Time {
  Time::new::<second>(now())
}