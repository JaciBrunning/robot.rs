use std::time::Duration;

use log::info;
use robot_rs::{
  robot_main,
  start::{RobotResult, RobotState},
};

fn my_robot(state: RobotState) -> RobotResult {
  while state.running() {
    info!("If you can read this, this robot is running Rust!");
    std::thread::sleep(Duration::from_millis(20));
  }

  Ok(())
}

robot_main!(my_robot);
