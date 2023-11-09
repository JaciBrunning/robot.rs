use log::info;

// Fake Gripper, just used to demonstrate the async framework. See async-elevator
// for an actual subsystem implementation
pub struct Gripper {}

impl Gripper {
    pub fn new() -> Self {
        Self {}
    }

    pub async fn release(&self) {
        info!("[GRIPPER] Release");
        tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;
    }

    pub async fn grip(&self) {
        info!("[GRIPPER] Grip");
        tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;
    }

    pub async fn run(&self) {
        loop {
            // Do loop stuff here
            tokio::time::sleep(tokio::time::Duration::from_millis(20)).await;
        }
    }
}
