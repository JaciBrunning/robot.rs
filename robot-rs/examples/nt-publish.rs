use std::{sync::{atomic::AtomicBool, Arc}, time::Duration};

use ntcore_rs::instance::ServerConfig;
use robot_rs::{start::RobotResult, robot_main, ntcore::{nt, instance::NetworkTableInstance}};

fn my_robot(running: Arc<AtomicBool>) -> RobotResult {
  NetworkTableInstance::default().start_server(ServerConfig::default());
  while running.load(std::sync::atomic::Ordering::Relaxed) {
    nt!("/testtopic/test", 12.0)?;
    std::thread::sleep(Duration::from_millis(20));
  }

  Ok(())
}

robot_main!(my_robot);