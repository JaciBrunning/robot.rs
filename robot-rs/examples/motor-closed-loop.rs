use std::time::Duration;

use ntcore_rs::{NetworkTableInstance, GenericPublisher};
use num_traits::Zero;
use robot_rs::{start::{RobotState, RobotResult}, actuators::{ActuatorExt, sim::SimulatedActuator, VoltageActuator}, sensors::{sim::{SimulatedSensor, SettableSensor}, SensorExt, DisplacementSensor}, robot_main, filters::{pid::PID, Filter}, time::now, physics::motor::{from_dyno::KrakenTrap, MotorExtensionTrait, SpooledMotorInverseDynamics, SpooledMotorCurrentDynamics, SpooledMotorForwardDynamics}};
use robot_rs_units::{electrical::{volt, Voltage}, meter, Length, inch, kilogram, motion::{meters_per_second, meters_per_second2}, millisecond, traits::ToFloat, Time};

fn my_robot(state: RobotState) -> RobotResult {
  let nt = NetworkTableInstance::default();
  let mut motor = Box::new(SimulatedActuator::new(0.0 * volt).clamp(-12.0 * volt, 12.0 * volt).observable(nt.topic("/motor/a")));

  let sim_sensor = SimulatedSensor::<Length>::new();
  let sensor = Box::new(sim_sensor.clone()).observable(nt.topic("/sensor/a"), Zero::zero());

  let mut filter = PID::<Length, Voltage, Time>::new((12.0 * volt) / (1.0 * meter), Zero::zero(), Zero::zero(), 0.5 * meter, 10, now).tunable(nt.topic("/elevator/pid"));

  let motor_model = KrakenTrap().geared(10.0).multiply(2).to_linear(2.0 * inch);

  let mut speed = 0.0 * meters_per_second;

  let current_publisher = nt.topic("/current").publish();

  while state.running() {
    motor.set_voltage(filter.calculate(sensor.get_displacement().unwrap_or(0.0 * meter)) + motor_model.voltage(9.81 * meters_per_second2 * (12.0 * kilogram), 0.0 * meters_per_second));
    let force = SpooledMotorInverseDynamics::force(&motor_model, motor.get_set_voltage(), speed);
    let current = motor_model.current(force);
    let accel = force / (12.0 * kilogram) - 9.81 * meters_per_second2;
    speed += accel * (10.0 * millisecond);
    let mut new_displacement = sim_sensor.get_displacement().unwrap_or(0.0 * meter) + speed * (10.0 * millisecond);
    if new_displacement < 0.0 * meter {
      new_displacement = 0.0 * meter;
      speed = 0.0 * meters_per_second;
    }
    sim_sensor.set_sensor_value(Some(new_displacement)); 

    current_publisher.set(current.to_f64()).ok();

    std::thread::sleep(Duration::from_millis(10));
  }

  Ok(())
}

robot_main!(my_robot);