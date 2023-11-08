mod drivetrain;
mod elevator;
mod gripper;

use std::sync::Arc;

use drivetrain::Drivetrain;
use elevator::Elevator;
use futures::join;
use gripper::Gripper;
use log::info;
use robot_rs::{
    ds::{ControlMode, RobotControlState},
    robot_main,
    start::RobotResult,
};
use tokio::time::{sleep, Duration};

async fn two_piece_auto(
    elevator: Arc<Elevator>,
    drivetrain: Arc<Drivetrain>,
    gripper: Arc<Gripper>,
) {
    info!("Two Cube Auto Begin");
    // Drop the first gamepiece
    gripper.release().await;
    info!("That's Cube One!");

    // Turn 180
    drivetrain.drive_distance(-0.1).await;
    drivetrain.turn_angle(180.0).await;

    // Run at the same time - drop elevator to 0.2 and drive to the setpoint
    let elev = elevator.go_to_height(0.2);
    let drive = async {
        drivetrain.drive_distance(0.5).await;
        drivetrain.turn_angle(90.0).await;
        drivetrain.drive_distance(0.2).await;
    };
    join!(elev, drive);

    // Grab the gamepiece
    gripper.grip().await;
    info!("Grabbed Cube Two");

    // Raise the elevator and back off
    let elev = elevator.go_to_height(1.0);
    let drive = drivetrain.drive_distance(-0.5);
    join!(elev, drive);

    // Turn back towards the scoring area, drive over
    drivetrain.turn_angle(90.0).await;
    drivetrain.drive_distance(0.5).await;

    // Drop the 2nd gamepiece
    gripper.release().await;
    info!("That's Cube Two!");
    info!("Two Cube Auto Stop");
}

async fn my_robot() -> RobotResult {
    let elevator = Arc::new(Elevator::new());
    let drivetrain = Arc::new(Drivetrain::new());
    let gripper = Arc::new(Gripper::new());

    let mut last_enabled = false;

    // Main loop
    let main_loop = async {
        loop {
            let state = RobotControlState::current();
            match (state.enabled, last_enabled, state.mode) {
                (true, false, ControlMode::Autonomous) => {
                    println!("Start");
                    // Spawn the task
                    tokio::task::spawn(two_piece_auto(
                        elevator.clone(),
                        drivetrain.clone(),
                        gripper.clone(),
                    ));
                }
                _ => (),
            }
            last_enabled = state.enabled;
            sleep(Duration::from_millis(20)).await;
        }
    };

    join!(main_loop, elevator.run(), drivetrain.run(), gripper.run());
    Ok(())
}

robot_main!(async my_robot);
