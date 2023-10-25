use log::info;

// Fake Elevator, just used to demonstrate the async framework. See async-elevator
// for an actual subsystem implementation
pub struct Elevator {}

impl Elevator {
    pub fn new() -> Self {
        Self {}
    }

    pub async fn go_to_height(&self, height: f64) {
        info!("[ELEVATOR] Going to height... {}", height);
        tokio::time::sleep(tokio::time::Duration::from_millis(500)).await;
        info!("[ELEVATOR] Gone to height... {}", height);
    }

    pub async fn run(&self) {
        loop {
            // Do loop stuff here
            tokio::time::sleep(tokio::time::Duration::from_millis(20)).await;
        }
    }
}
