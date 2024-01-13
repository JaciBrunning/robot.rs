use std::{sync::{atomic::AtomicBool, Arc}, time::Duration};

use ntcore_rs::{topic::GenericPublisher, instance::ServerConfig};
use robot_rs::{start::RobotResult, robot_main, ntcore::instance::NetworkTableInstance};

fn my_robot(running: Arc<AtomicBool>) -> RobotResult {
  // #[cfg(simulation)]
  NetworkTableInstance::default().start_server(ServerConfig::default());
  
  let topic = NetworkTableInstance::default().topic("/testtopic/some_value");
  let entry = topic.entry();
  while running.load(std::sync::atomic::Ordering::Relaxed) {
    entry.set(32.0)?;
    std::thread::sleep(Duration::from_millis(20));
  }

  Ok(())
}

robot_main!(my_robot);