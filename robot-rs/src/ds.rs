#[cfg(feature = "hal")]
use crate::hal::{HAL_ControlWord, HAL_GetControlWord, HAL_RefreshDSData};

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ControlMode {
  Autonomous,
  Teleop,
  Test,
  Disabled,
}

impl Default for ControlMode {
  fn default() -> Self {
    Self::Disabled
  }
}

#[derive(Debug, Clone, Default)]
pub struct RobotControlState {
  pub mode: ControlMode,
  pub enabled: bool,
  pub estopped: bool,
  pub ds_attached: bool,
  pub fms_attached: bool,
}

impl RobotControlState {
  #[cfg(feature = "hal")]
  pub fn current() -> Self {
    let mut control_word = HAL_ControlWord::default();
    let err = unsafe {
      HAL_RefreshDSData();
      HAL_GetControlWord(&mut control_word)
    };
    if err != 0 {
      panic!("Control Word could not be fetched")
    }
    Self {
      mode: match (
        control_word.enabled() != 0,
        control_word.autonomous(),
        control_word.test(),
      ) {
        (true, a, _t) if a != 0 => ControlMode::Autonomous,
        (true, _a, t) if t != 0 => ControlMode::Test,
        (false, _a, _t) => ControlMode::Disabled,
        _ => ControlMode::Teleop,
      },
      enabled: (control_word.enabled() != 0 && control_word.dsAttached() != 0),
      estopped: control_word.eStop() != 0,
      ds_attached: control_word.dsAttached() != 0,
      fms_attached: control_word.fmsAttached() != 0,
    }
  }
}

#[cfg(feature = "hal")]
pub mod observe {
  use robot_rs_wpilib_sys::{
    HAL_ObserveUserProgramAutonomous, HAL_ObserveUserProgramDisabled,
    HAL_ObserveUserProgramStarting, HAL_ObserveUserProgramTeleop, HAL_ObserveUserProgramTest,
  };

  pub fn start() {
    unsafe { HAL_ObserveUserProgramStarting() }
  }
  pub fn disabled() {
    unsafe { HAL_ObserveUserProgramDisabled() }
  }
  pub fn autonomous() {
    unsafe { HAL_ObserveUserProgramAutonomous() }
  }
  pub fn teleop() {
    unsafe { HAL_ObserveUserProgramTeleop() }
  }
  pub fn test() {
    unsafe { HAL_ObserveUserProgramTest() }
  }
}

#[cfg(not(feature = "hal"))]
pub mod observe {
  pub fn start() {}
  pub fn disabled() {}
  pub fn autonomous() {}
  pub fn teleop() {}
  pub fn test() {}
}

#[macro_export]
macro_rules! robot_init {
  (
    auto => $fut_auto:expr,
    teleop => $fut_teleop:expr,
    test => $fut_test:expr
  ) => {{
    let mut last_state = robot_rs::ds::RobotControlState::default();
    let mut future = None;
    loop {
      let state = robot_rs::ds::RobotControlState::current();
      match (&state.mode, &last_state.mode) {
        (robot_rs::ds::ControlMode::Autonomous, last)
          if *last != robot_rs::ds::ControlMode::Autonomous =>
        {
          future = $fut_auto;
        }
        (robot_rs::ds::ControlMode::Teleop, last) if *last != robot_rs::ds::ControlMode::Teleop => {
          future = $fut_teleop;
        }
        (robot_rs::ds::ControlMode::Test, last) if *last != robot_rs::ds::ControlMode::Test => {
          future = $fut_test;
        }
        _ => (),
      }

      last_state = state;
      if let Some(mut fut) = future.take() {
        tokio::select! {
          _ = fut.as_mut() => {},
          _ = tokio::time::sleep(tokio::time::Duration::from_millis(20)) => {
            future = Some(fut);
          }
        }
      } else {
        tokio::time::sleep(tokio::time::Duration::from_millis(20)).await;
      }
    }
  }};
}
