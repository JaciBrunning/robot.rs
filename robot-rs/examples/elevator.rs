use futures::FutureExt;
use ntcore_rs::NetworkTableInstance;
use num_traits::Zero;
use robot_rs::{systems::elevator::{ElevatorImpl, ElevatorParams, sim::ElevatorSim, AwaitableElevator}, actuators::{sim::SimulatedActuator, ActuatorExt}, sensors::{sim::SimulatedSensor, SensorExt}, physics::motor::{from_dyno::KrakenTrap, MotorExtensionTrait}, transforms::{pid::PID, stability::RMSStabilityFilter}, start::{RobotResult, RobotState}, robot_main, system, perform, activity::Priority};
use robot_rs_units::{kilogram, degree, meter, electrical::volt, Length, inch, motion::meters_per_second, second, millisecond};

async fn my_robot(_state: RobotState) -> RobotResult {
  let nt = NetworkTableInstance::default();
  let sim_motor = SimulatedActuator::new(0.0 * volt, robot_rs::time::now());
  let sim_sensor = SimulatedSensor::<Length>::new(0.0 * meter);
  let motor_model = KrakenTrap().geared(20.0).multiply(2).to_linear(2.0 * inch);

  let motor = sim_motor.clone()
      .observable(nt.topic("/elevator/motor"))
      .clamp(-12.0 * volt, 12.0 * volt);

  let sensor = sim_sensor.clone()
      .observable(nt.topic("/elevator/heightsensor"));

  let params = ElevatorParams {
    angle_from_horizon: 90.0 * degree,
    carriage_mass: 12.0 * kilogram,
    limits: (0.0 * meter, 1.2 * meter),
  };

  let elevator = ElevatorImpl::new(
    params,
    0.02 * meter,
    Box::new(motor),
    Box::new(motor_model.clone()),
    Box::new(sensor.to_stateful()),
    (None, None),
    Box::new(PID::new(
      (12.0 * volt) / (0.25 * meter),
      (1.0 * volt) / (0.5 * meter * (1.0 * second)),
      Zero::zero(),
      0.5 * meter,
      10
    ).tunable(nt.topic("/elevator/pid"))),
    Box::new(RMSStabilityFilter::new(0.05 * meter, Some(0.1 * meters_per_second), 10))
  );

  let elevator_system = system!(elevator.frontend());

  let mut elevator_sim = ElevatorSim::new(
    params,
    Box::new(sim_motor),
    Box::new(motor_model),
    Box::new(sim_sensor),
    (None, None)
  );

  tokio_scoped::scope(|scope| {
    scope.spawn(async move {
      // This is where you'd put your robot control loop, or perhaps a new function 
      // that runs in a time-controlled loop
      perform!(elevator_system, Priority(1), |sys| async move { sys.go_to_height_wait(1.0 * meter).await; }.boxed());
    });
    scope.spawn(elevator.run_async(10.0 * millisecond));
    scope.spawn(elevator_sim.run(5.0 * millisecond));
  });

  Ok(())
}

robot_main!(async my_robot);