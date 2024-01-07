use std::time::{SystemTime, UNIX_EPOCH};

#[cfg(feature = "hal")]
use robot_rs_wpilib_sys::{hal_safe_call, HAL_GetFPGATime};

pub fn now() -> crate::units::Time {
  #[cfg(feature = "hal")]
  let now = hal_safe_call!(HAL_GetFPGATime()).unwrap_or(SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_micros() as u64);
  #[cfg(not(feature = "hal"))]
  let now = SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_micros();

  crate::units::Time::new::<crate::units::time::microsecond>(now as f64)
}