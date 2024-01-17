#[cfg(feature = "hal")]
pub use robot_rs_wpilib_sys as hal;

#[cfg(feature = "ntcore")]
pub use ntcore_rs as ntcore;

pub use robot_rs_macros as macros;

pub mod actuators;
pub mod activity;
pub mod input;
pub mod physics;
pub mod sensors;
pub mod systems;
pub mod transforms;

pub mod ds;
pub mod start;
pub mod time;
pub mod traits;

pub use robot_rs_units as units;

pub enum RuntimeType {
  Native,
  Simulation
}

#[cfg(native)]
#[macro_export]
macro_rules! runtime_type {
  () => { RuntimeType::Native };
}

#[cfg(native)]
#[macro_export]
macro_rules! with_runtime {
  (native $b:block) => { $b };
  (simulation $b:block) => {};
}

#[cfg(simulation)]
#[macro_export]
macro_rules! runtime_type {
    () => { RuntimeType::Simulation };
}

#[cfg(simulation)]
#[macro_export]
macro_rules! with_runtime {
  (native $b:block) => {};
  (simulation $b:block) => { $b };
}