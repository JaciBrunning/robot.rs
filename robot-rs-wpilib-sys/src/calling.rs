use std::{ffi::CStr, fmt::{Debug, Display}};
use crate::HAL_GetLastError;

// Parts borrowed from https://github.com/first-rust-competition/first-rust-competition/blob/master/wpilib-sys/src/hal_call.rs
#[derive(Debug)]
pub struct WpiHalError(String);

impl WpiHalError {
  pub fn new(mut status: i32) -> Self {
    let str = unsafe { CStr::from_ptr(HAL_GetLastError(&mut status)) };
    Self(str.to_string_lossy().to_string())
  }
}

impl Display for WpiHalError {
  fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
    write!(f, "HAL Error: \"{}\"", self.0)
  }
}

pub type WpiHalResult<T> = Result<T, WpiHalError>;

#[macro_export]
macro_rules! hal_safe_call {
  ($function:ident($($arg:expr),* $(,)?)) => {{
    use robot_rs_wpilib_sys::calling::WpiHalError;
    unsafe {
      let mut status = 0;
      let result = $function($($arg,)* &mut status as *mut i32);

      if status == 0 {
        Ok(result)
      } else {
        Err(WpiHalError::new(status))
      }
    }
  }};
}