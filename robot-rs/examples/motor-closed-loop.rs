use std::time::Duration;

use ntcore_rs::NetworkTableInstance;
use num_traits::Zero;
use robot_rs::{start::{RobotState, RobotResult}, actuators::{ActuatorExt, sim::SimulatedActuator, VoltageActuator}, sensors::{sim::{SimulatedSensor, SettableSensor}, SensorExt, DisplacementSensor}, robot_main};
use robot_rs_units::{electrical::volt, meter, Length};

fn my_robot(state: RobotState) -> RobotResult {
  let nt = NetworkTableInstance::default();
  let mut motor = Box::new(SimulatedActuator::new(0.0 * volt).observable(nt.topic("/motor/a")));
  let sim_sensor = SimulatedSensor::<Length>::new();
  let sensor = Box::new(sim_sensor.clone()).observable(nt.topic("/sensor/a"), Zero::zero());

  while state.running() {
    // TODO: Do this  as a closed loop, with physics.
    motor.set_voltage(12.0 * volt);
    sim_sensor.set_sensor_value(Some(1.5 * meter)); 
    println!("{:?}", sensor.get_displacement());
    std::thread::sleep(Duration::from_millis(20));
  }

  Ok(())
}

robot_main!(my_robot);