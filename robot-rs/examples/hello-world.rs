use std::{sync::{atomic::AtomicBool, Arc}, time::Duration};

use log::info;
use robot_rs::{start::RobotResult, robot_main};

fn my_robot(running: Arc<AtomicBool>) -> RobotResult {
  while running.load(std::sync::atomic::Ordering::Relaxed) {
    info!("If you can read this, this robot is running Rust!");
    std::thread::sleep(Duration::from_millis(20));
  }

  Ok(())
}

robot_main!(my_robot);