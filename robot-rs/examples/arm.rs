use std::time::Duration;

use futures::FutureExt;
use ntcore_rs::NetworkTableInstance;
use num_traits::Zero;
use robot_rs::{
  activity::Priority,
  actuators::{sim::SimulatedActuator, ActuatorExt},
  perform,
  physics::motor::{from_dyno::KrakenTrap, MotorExtensionTrait},
  robot_main,
  sensors::{sim::SimulatedSensor, SensorExt, StatefulSensorExt},
  start::{RobotResult, RobotState},
  system,
  systems::arm::{sim::ArmSim, Arm, ArmParams, AwaitableArm},
  transforms::{
    diff::DifferentiatingTransform,
    linear::LinearTransforms,
    pid::PID,
    predictive::CurrentLimitTransform,
    profile::{ProfileFeedForward, ProfileState, Profiled1stOrderController, TrapezoidalProfile},
    stability::RMSStabilityFilter,
    ChainedStatefulTransformsA, TransformExt,
  },
};
use robot_rs_units::{
  ampere, degree,
  electrical::{volt, Voltage},
  kilogram, meter, millisecond,
  motion::{degrees_per_second, degrees_per_second2},
  Angle, Time,
};

async fn my_robot(_state: RobotState) -> RobotResult {
  let nt = NetworkTableInstance::default();
  let sim_motor = SimulatedActuator::new(0.0 * volt, robot_rs::time::now());
  let sim_sensor = SimulatedSensor::<Angle>::new(0.0 * degree);
  let motor_model = KrakenTrap().geared(120.0).multiply(2);

  let velocity_sensor = sim_sensor
    .clone()
    .to_stateful()
    .transform(DifferentiatingTransform::new())
    .transform(LinearTransforms::moving_average(5))
    .observable(nt.topic("/arm/velocity"));

  let motor = sim_motor
    .clone()
    .observable(nt.topic("/arm/motor"))
    .clamp(-12.0 * volt, 12.0 * volt)
    .transform(CurrentLimitTransform::new(
      90.0 * ampere,
      velocity_sensor,
      motor_model.clone(),
    ));

  let sensor = sim_sensor.clone().observable(nt.topic("/arm/angle"));

  let params = ArmParams {
    effective_mass: 15.0 * kilogram,
    center_of_mass_length: 1.0 * meter,
    limits: (0.0 * degree, 180.0 * degree),
  };

  let controller = ChainedStatefulTransformsA::new(
    Profiled1stOrderController::new(
      TrapezoidalProfile::new(
        180.0 * degrees_per_second,
        45.0 * degrees_per_second2,
        ProfileState::zero(),
      ),
      PID::<Angle, Voltage, Time>::new(
        (12.0 * volt) / (45.0 * degree),
        Zero::zero(),
        Zero::zero(),
        0.0 * degree,
      )
      .tunable(nt.topic("/arm/pid")),
      ProfileFeedForward::from_motor_angular(
        motor_model.clone(),
        12.0 * volt,
        params.effective_mass * params.center_of_mass_length * params.center_of_mass_length,
      )
      .to_stateful(),
    ),
    LinearTransforms::moving_average(10),
  );

  let arm = Arm::new(
    params,
    Box::new(motor),
    Box::new(motor_model.clone()),
    Box::new(sensor.to_stateful()),
    (None, None),
    Box::new(controller),
    Box::new(RMSStabilityFilter::new(
      3.0 * degree,
      Some(5.0 * degrees_per_second),
      10,
    )),
  );

  let arm_system = system!(arm.frontend());

  let mut arm_sim = ArmSim::new(
    params,
    Box::new(sim_motor),
    Box::new(motor_model),
    Box::new(sim_sensor),
    (None, None),
    nt.topic("/arm/sim"),
  );

  tokio_scoped::scope(|scope| {
    scope.spawn(async move {
      // This is where you'd put your robot control loop, or perhaps a new function
      // that runs in a time-controlled loop
      perform!(arm_system, Priority(1), |sys| async move {
        sys.go_to_angle_wait(145.0 * degree).await;
        tokio::time::sleep(Duration::from_millis(500)).await;
        sys.go_to_angle_wait(90.0 * degree).await;
      }
      .boxed());
    });
    scope.spawn(arm.run_async(10.0 * millisecond));
    scope.spawn(arm_sim.run(5.0 * millisecond));
  });

  Ok(())
}

robot_main!(async my_robot);
