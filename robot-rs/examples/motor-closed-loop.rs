use std::time::Duration;

use ntcore_rs::NetworkTableInstance;
use num_traits::Zero;
use robot_rs::{start::{RobotState, RobotResult}, actuators::{sim::SimulatedVoltageController, VoltageController, VoltageControllerExt}, units::{ElectricPotential, Length}, robot_main, sensors::{sim::SimulatedSensor, SensorExt, DisplacementSensor}};
use uom::si::{electric_potential, length};

fn my_robot(state: RobotState) -> RobotResult {
  // #[cfg(simulation)]  
  let nt = NetworkTableInstance::default();
  let mut motor = Box::new(SimulatedVoltageController::new(Zero::zero()).observable(nt.topic("/motor/a")));
  let sim_sensor = SimulatedSensor::<Length>::new();
  let sensor = Box::new(sim_sensor.clone()).observable(nt.topic("/sensor/a"), Zero::zero());

  while state.running() {
    // TODO: Do this  as a closed loop, with physics.
    motor.set_voltage(ElectricPotential::new::<electric_potential::volt>(12.0));
    sim_sensor.set_sensor_value(Some(Length::new::<length::meter>(1.5))); 
    println!("{:?}", sensor.get_displacement());
    std::thread::sleep(Duration::from_millis(20));
  }

  Ok(())
}

robot_main!(my_robot);