// pub mod elevator;

// use std::time::Duration;

// use elevator::{Elevator, ElevatorConfig, SimElevator};
// use ntcore_rs::NetworkTableInstance;
// use robot_rs::{start::{RobotState, RobotResult}, robot_main, system, actuators::sim::SimulatedVoltageController, sensors::sim::SimulatedSensorImpl, physics::motor::{AngularToLinearMotor, GearedMotor, MultiMotor, from_dyno::KrakenTrap, MotorExtensionTrait}, activity::make_shared};
// use robot_rs_units::{electrical::volt, meter, kilogram, inch};

// // TODO: Need to normalise traits for systems to avoid generics hell. Is there some way we can automate:
// // - Trait generation / generating a system from a trait?
// // - Handling the fast-spinning "backend", with shared state?

// async fn my_robot(state: RobotState) -> RobotResult {
//   let elevator_config = make_shared(ElevatorConfig {
//     voltage_controller: Box::new(SimulatedVoltageController::new(0.0 * volt)),
//     height_sensor: SimulatedSensorImpl::new(),
//     limit_switches: (SimulatedSensorImpl::new(), SimulatedSensorImpl::new()),
//     height_limit: (0.0 * meter, 1.0 * meter),
//     mass: 10.0*kilogram,
//     motor: AngularToLinearMotor::new(1.0 * inch, KrakenTrap().multiply(2).geared(60.0)),
//   });

//   let elevator = system!(Elevator::new(
//     NetworkTableInstance::default().topic("/elevator"),
//     elevator_config.clone()
//   ));

//   #[cfg(simulation)]
//   let sim_elevator = SimElevator::new(elevator_config);

//   loop {
//     tokio::time::sleep(Duration::from_millis(10)).await;
//   }
// }

// robot_main!(async my_robot);S
fn main() {}