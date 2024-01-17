use std::time::Duration;

use ntcore_rs::{NetworkTableInstance, GenericPublisher};
use num_traits::Zero;
use robot_rs::{start::{RobotState, RobotResult}, actuators::{ActuatorExt, sim::{SimulatedActuator, ReadableActuator}, VoltageActuator}, sensors::{sim::{SimulatedSensor, SettableSensor}, SensorExt, DisplacementSensor}, robot_main, filters::{pid::PID, Filter, ChainedFiltersA, feedforward::OffsetFeedforwardFilter, predictive::CurrentLimitFilter, diff::DifferentiatingFilter}, time::now, physics::motor::{from_dyno::KrakenTrap, MotorExtensionTrait, SpooledMotorInverseDynamics, SpooledMotorCurrentDynamics, SpooledMotorForwardDynamics}};
use robot_rs_units::{electrical::{volt, Voltage}, meter, Length, inch, kilogram, motion::{meters_per_second, meters_per_second2}, millisecond, traits::ToFloat, Time, second, ampere};

fn my_robot(state: RobotState) -> RobotResult {
  let nt = NetworkTableInstance::default();
  let sim_motor = SimulatedActuator::new(0.0 * volt, now());
  let mut sim_sensor = SimulatedSensor::<Length>::new(0.0 * meter);
  let motor_model = KrakenTrap().geared(30.0).multiply(2).to_linear(2.0 * inch);

  let mut motor = sim_motor.clone()
      .clamp(-12.0 * volt, 12.0 * volt)
      .observable(nt.topic("/motor"));

  let mut sensor = sim_sensor.clone();

  let carriage_mass = 12.0 * kilogram;

  let mut control_filter = ChainedFiltersA::new(
    PID::<Length, Voltage, Time>::new(
      (12.0 * volt) / (0.25 * meter),
      (1.0 * volt) / (0.5 * meter * (1.0 * second)),
      Zero::zero(),
      0.5 * meter,
      10
    ).tunable(nt.topic("/pid")),
    OffsetFeedforwardFilter::new(
      motor_model.voltage(carriage_mass * (9.81 * meters_per_second2), 0.0 * meters_per_second)
    )
  );

  let current_publisher = nt.topic("/sim/current").publish();
  let mut speed = 0.0 * meters_per_second;
  let mut last_time = robot_rs::time::now();

  while state.running() {
    let now = robot_rs::time::now();
    let dt = now - last_time;

    let demand = control_filter.calculate(sensor.get_displacement(), now);
    motor.set_voltage(demand, now);

    let force = motor_model.force(sim_motor.get_actuator_value().0, speed);
    let current = motor_model.current(force);
    let accel = force / carriage_mass - 9.81 * meters_per_second2;
    speed += accel * dt;

    let mut new_displacement = sim_sensor.get_displacement() + speed * dt;
    if new_displacement < 0.0 * meter {
      new_displacement = 0.0 * meter;
      speed = 0.0 * meters_per_second;
    }
    sim_sensor.set_sensor_value(new_displacement, now);

    current_publisher.set(current.to_f64()).ok();

    last_time = now;
    std::thread::sleep(Duration::from_millis(1));
  }

  Ok(())
}

robot_main!(my_robot);