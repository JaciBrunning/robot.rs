use std::sync::{atomic::AtomicBool, Arc};

use robot_rs::{
    actuators::motors::PWMSparkMax, input::xbox::Xbox, robot_main, sensors::analog::AnalogInput,
    start::RobotResult,
};

pub fn my_robot(running: Arc<AtomicBool>) -> RobotResult {
    let mut motor = PWMSparkMax::new(0);
    let xbox = Xbox::new(0);

    while running.load(std::sync::atomic::Ordering::Relaxed) {
        let value = xbox.left_x().get();
        motor.set_speed(value);
        std::thread::sleep(std::time::Duration::from_millis(20));
    }
    Ok(())
}

robot_main!(my_robot);
