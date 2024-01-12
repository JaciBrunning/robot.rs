pub use robot_rs_ntcore_sys as nt_internal;

pub mod instance;
pub mod topic;
pub mod types;
pub mod entry;

#[macro_export]
macro_rules! nt {
  ($path:expr, $value:expr) => { nt!(ntcore_rs::instance::NetworkTableInstance::default(), $path, $value) };
  ($inst:expr, $path:expr, $value:expr) => { $inst.entry($path).set($value) };
  (read $path:expr) => { nt!(read ntcore_rs::instance::NetworkTableInstance::default(), $path) };
  (read $inst:expr, $path:expr) => { $inst.entry($path).get() };
}