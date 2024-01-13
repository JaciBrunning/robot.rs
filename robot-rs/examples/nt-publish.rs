use std::time::Duration;

use ntcore_rs::topic::GenericPublisher;
use robot_rs::{start::{RobotResult, RobotState}, robot_main, ntcore::instance::NetworkTableInstance};

fn my_robot(state: RobotState) -> RobotResult {  
  let topic = NetworkTableInstance::default().topic("/testtopic/some_value");
  let entry = topic.entry();
  while state.running() {
    entry.set(32.0)?;
    std::thread::sleep(Duration::from_millis(20));
  }

  Ok(())
}

robot_main!(my_robot);