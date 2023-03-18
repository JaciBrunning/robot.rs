use log::info;

// Fake Drivetrain, just used to demonstrate the async framework. See async-elevator
// for an actual subsystem implementation
pub struct Drivetrain { }

impl Drivetrain {
  pub fn new() -> Self {
    Self {}
  }

  pub async fn drive_distance(&self, distance: f64) {
    info!("[DRIVETRAIN] Driving Distance... {}", distance);
    tokio::time::sleep(tokio::time::Duration::from_millis((distance * 1000.0) as u64)).await;
    info!("[DRIVETRAIN] Driven {}", distance);
  }

  pub async fn turn_angle(&self, angle: f64) {
    info!("[DRIVETRAIN] Turning to Angle {}", angle);
    tokio::time::sleep(tokio::time::Duration::from_millis((angle / 180.0 * 750.0) as u64)).await;
    info!("[DRIVETRAIN] Turned {}", angle);
  }

  pub async fn run(&self) {
    loop {
      // Do loop stuff here
      tokio::time::sleep(tokio::time::Duration::from_millis(20)).await;
    }
  }
}