pub mod elevator;

use std::sync::Arc;

use elevator::{Elevator, AbstractElevator, ElevatorResult};
use futures::{future::join_all, FutureExt};
use log::info;
use nt4_rs::instance::NetworkTableInstance;
use robot_rs::{start::RobotResult, robot_main, actuators::motors::{PWMSparkMax, ClampedMotor}, sensors::{analog::AnalogInput, distance::NaiveDistanceSource}, input::xbox::Xbox, control::{edge_detect::Edge, pid::PIDConfig}, types::MinMax, models::DcMotor, robot_init};
use tokio::sync::RwLock;

use crate::elevator::ElevatorConfig;

/// Manual Elevator Control. This async function will control the elevator based on the 
/// xbox controller input, until another function takes over control from the elevator's control
/// lock.
async fn manual_controls<'a, E: AbstractElevator<'a>>(elevator: &'a E, xbox: &Xbox) {
  let control = elevator.control().steal().await;
  let height_control = xbox.left_y();

  info!("Starting manual control");
  while elevator.set_voltage(height_control.get() * 12.0, &control).await.is_ok() {
    tokio::time::sleep(tokio::time::Duration::from_millis(20)).await;
  }
  info!("Manual control interrupted");
}

/// Move the elevator to a height. This function will demand the elevator to go to a height,
/// and will continue running until it either reaches the desired height or another function
/// steals the control lock.
async fn move_to_height<'a, E: AbstractElevator<'a>>(elevator: &'a E, height: f64) {
  let control = elevator.control().steal().await;
  info!("Going to height: {:?}", height);
  
  match elevator.go_to_height(height, &control).await {
    Ok(()) => info!("Go to height done"),
    Err(e) => info!("Go to height: {}", e)
  }

  info!("Go to height done");
}

/// Run a basic auto routine, setting the elevator to a series of different heights. This function
/// will continue until the control lock is stolen (i.e. the elevator fails to reach the height),
/// or until the auto routine is done.
async fn auto_routine<'a, E: AbstractElevator<'a>>(elevator: &'a E) -> ElevatorResult<()> {
  info!("Auto Start");
  let control = elevator.control().steal().await;

  elevator.go_to_height(0.5, &control).await?;
  elevator.go_to_height(1.0, &control).await?;
  elevator.go_to_height(0.75, &control).await?;
  elevator.go_to_height(0.0, &control).await?;
  elevator.go_to_height(1.0, &control).await?;

  Ok(())
}

/// Simple function to handle button presses scheduling new activities
async fn buttons<'a, E: AbstractElevator<'a> + Send + Sync>(elevator: &'a E, xbox: &Xbox) {
  let mut futs = vec![];
  futs.push(xbox.a().edge_take(Edge::Falling).run(|| { move_to_height(elevator, 1.0) }).boxed());
  futs.push(xbox.b().edge_take(Edge::Falling).run(|| { manual_controls(elevator, xbox) }).boxed());

  // Join all the futures, running them concurrently within this function until either of them complete (never)
  join_all(futs).await;
}

/// Init function - used for running start-of-mode behaviours (similar to AutonomousInit, TeleopInit, etc in wpilib)
async fn init<'a, E: AbstractElevator<'a> + Send + Sync>(elevator: &'a E, xbox: &Xbox) {
  // robot_init! takes a future for each branch - i.e, a non-awaited async function call
  robot_init!(
    auto => Some(auto_routine(elevator).map(|_| ()).boxed()),
    teleop => Some(manual_controls(elevator, &xbox).boxed()),
    test => None
  )
}

/// Main robot entry function.
async fn my_async_robot() -> RobotResult {
  NetworkTableInstance::default().start_server(Default::default());

  let xbox = Xbox::new(0);

  // Create the elevator
  let elevator_config = ElevatorConfig {
    motor: RwLock::new(ClampedMotor(PWMSparkMax::new(0), -10.0, 10.0)),
    height_sensor: RwLock::new(NaiveDistanceSource::new(0.5)),
    height_limit: MinMax::new(0.0, 1.3),
    mass: 10.0,
    motor_model: DcMotor::neo().reduce(20.0),
    spool_radius: 2.0 * 0.0254,
  };

  let elevator = Elevator::new(elevator_config, PIDConfig {
    kp: 6.0, ki: 0.0, kd: 0.5, izone: None
  });

  // Run the elevator and simulation. As with all async functions, they don't actually
  // run until they're awaited or spawned.
  let elevator_fut = elevator.run();
  let sim_fut = elevator.run_sim();

  // Spawn all futures (run concurrently) and wait until they're done (i.e. never)
  tokio_scoped::scope(|scope| {
    scope.spawn(init(&elevator, &xbox));
    scope.spawn(manual_controls(&elevator, &xbox));
    scope.spawn(buttons(&elevator, &xbox));
    scope.spawn(elevator_fut);
    scope.spawn(sim_fut);
  });

  Ok(())
}

robot_main!(async my_async_robot);