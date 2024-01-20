use std::time::Duration;

use futures::FutureExt;
use ntcore_rs::NetworkTableInstance;
use num_traits::Zero;
use robot_rs::{systems::elevator::{ElevatorParams, sim::ElevatorSim, AwaitableElevator, Elevator}, actuators::{sim::SimulatedActuator, ActuatorExt}, sensors::{sim::SimulatedSensor, SensorExt, StatefulSensorExt, StatefulSensor}, physics::motor::{from_dyno::KrakenTrap, MotorExtensionTrait}, transforms::{pid::PID, stability::RMSStabilityFilter, diff::DifferentiatingTransform, linear::LinearTransforms, profile::{Profiled1stOrderController, TrapezoidalProfile, ProfileState, ProfileFeedForward}, TransformExt, ChainedStatefulTransformsA}, start::{RobotResult, RobotState}, robot_main, system, perform, activity::Priority};
use robot_rs_units::{kilogram, degree, meter, electrical::{volt, Voltage}, Length, inch, motion::{meters_per_second, meters_per_second2}, millisecond, Time};

async fn my_robot(_state: RobotState) -> RobotResult {
  let nt = NetworkTableInstance::default();
  let sim_motor = SimulatedActuator::new(0.0 * volt, robot_rs::time::now());
  let sim_sensor = SimulatedSensor::<Length>::new(0.0 * meter);
  let motor_model = KrakenTrap().geared(15.0).multiply(2).to_linear(2.0 * inch);

  let motor = sim_motor.clone()
      .observable(nt.topic("/elevator/motor"))
      .clamp(-12.0 * volt, 12.0 * volt);

  let sensor = sim_sensor.clone()
      .observable(nt.topic("/elevator/height"));

  let mut velocity_sensor = sim_sensor.clone().to_stateful()
      .transform(DifferentiatingTransform::new())
      .transform(LinearTransforms::moving_average(5))
      .observable(nt.topic("/elevator/velocity"));

  let params = ElevatorParams {
    angle_from_horizon: 90.0 * degree,
    carriage_mass: 1.0 * kilogram,
    limits: (0.0 * meter, 1.2 * meter),
  };
  
  let controller = ChainedStatefulTransformsA::new(
    Profiled1stOrderController::new(
      TrapezoidalProfile::new(1.0 * meters_per_second, 0.5 * meters_per_second2, ProfileState::zero()),
      PID::<Length, Voltage, Time>::new(
        (12.0 * volt) / (0.25 * meter),
        Zero::zero(),
        Zero::zero(),
        0.5 * meter,
      ).tunable(nt.topic("/elevator/pid")),
      ProfileFeedForward::from_motor_linear(motor_model.clone(), 12.0 * volt, params.carriage_mass).to_stateful()
    ),
    LinearTransforms::moving_average(10)
  );

  let elevator = Elevator::new(
    params,
    Box::new(motor),
    Box::new(motor_model.clone()),
    Box::new(sensor.to_stateful()),
    (None, None),
    Box::new(controller),
    Box::new(RMSStabilityFilter::new(0.05 * meter, Some(0.1 * meters_per_second), 10))
  );

  let elevator_system = system!(elevator.frontend());

  let mut elevator_sim = ElevatorSim::new(
    params,
    Box::new(sim_motor),
    Box::new(motor_model),
    Box::new(sim_sensor),
    (None, None),
    nt.topic("/elevator/sim")
  );

  tokio_scoped::scope(|scope| {
    scope.spawn(async move {
      // This is where you'd put your robot control loop, or perhaps a new function 
      // that runs in a time-controlled loop
      perform!(elevator_system, Priority(1), |sys| async move { sys.go_to_height_wait(1.0 * meter).await; tokio::time::sleep(Duration::from_millis(500)).await; sys.go_to_height_wait(0.5 * meter).await; }.boxed());
    });
    scope.spawn(async move { loop { velocity_sensor.get_sensor_value(); tokio::time::sleep(Duration::from_millis(10)).await; } });
    scope.spawn(elevator.run_async(10.0 * millisecond));
    scope.spawn(elevator_sim.run(10.0 * millisecond));
  });

  Ok(())
}

robot_main!(async my_robot);