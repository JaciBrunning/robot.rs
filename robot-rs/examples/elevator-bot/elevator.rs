// use std::{sync::RwLock, error::Error, time::Duration, fmt::Binary};

// use ntcore_rs::{Topic, Publisher, GenericPublisher};
// use num_traits::Zero;
// use robot_rs::{actuators::VoltageController, sensors::{BinarySensor, DisplacementSensor, VelocitySensor, sim::SimulatedSensor}, physics::motor::{Motor, MotorForwardDynamics, MotorInverseDynamics, SpooledMotorForwardDynamics, SpooledMotorInverseDynamics}, start::RobotResult, activity::Shared};
// use robot_rs_units::{Length, electrical::{Voltage, volt}, Mass, motion::{meters_per_second2, meters_per_second}, meter, millisecond, QuantityBase, second, traits::MaybeUnitNumber};

// pub enum ElevatorState {
//   Idle,
//   Manual { voltage: Voltage },
//   HeightControl { height: Length }
// }

// pub struct ElevatorConfig<M, HS: DisplacementSensor, LS1: BinarySensor, LS2: BinarySensor> {
//   pub voltage_controller: Box<dyn VoltageController>,
//   pub height_sensor: HS,
//   pub limit_switches: (LS1, LS2),
//   pub height_limit: (Length, Length),
//   pub mass: Mass,
//   pub motor: M,
// }

// pub struct ElevatorPublishers {
//   state: Publisher<String>,
//   height: Publisher<f64>,
//   demand_voltage: Publisher<f64>,
// }

// pub struct Elevator<M, HS: DisplacementSensor, LS1: BinarySensor, LS2: BinarySensor> {
//   topic: Topic,
//   publishers: ElevatorPublishers,
//   config: Shared<ElevatorConfig<M, HS, LS1, LS2>>,
//   state: RwLock<ElevatorState>,
// }

// impl<'a, M: SpooledMotorForwardDynamics, HS: DisplacementSensor, LS1: BinarySensor, LS2: BinarySensor>
//   Elevator<M, HS, LS1, LS2>
// {
//   pub fn new(topic: Topic, config: Shared<ElevatorConfig<M, HS, LS1, LS2>>) -> Self {
//     let pubs = ElevatorPublishers {
//       state: topic.child("state").publish(),
//       height: topic.child("height").publish(),
//       demand_voltage: topic.child("demand").publish(),
//     };
//     Self { topic, publishers: pubs, config, state: RwLock::new(ElevatorState::Idle) }
//   }

//   pub async fn run(&self) {
//     loop {
//       let mut demand_voltage = 0f64 * volt;
//       let config = self.config.read().unwrap();
//       let current_height = config.height_sensor.get_displacement().unwrap_or(0.0 * meter);
//       let feedforward = config.motor.voltage(config.mass * (9.81 * meters_per_second2), 0.0 * meters_per_second);

//       let mut state_msg = "".to_owned();

//       match &*self.state.read().unwrap() {
//         ElevatorState::Idle => {
//           demand_voltage = Zero::zero();
//           state_msg = "IDLE".to_owned();
//         },
//         ElevatorState::Manual { voltage } => {
//           demand_voltage = *voltage;
//           state_msg = "MANUAL".to_owned();
//         },
//         ElevatorState::HeightControl { height } => {
//           let error = *height - current_height;
//           demand_voltage = (12.0 * volt) / (1.0 * meter) * error + feedforward;
//           state_msg = "HEIGHT".to_owned();
//         },
//       }

//       if Some(true) == config.limit_switches.0.get_state() {
//         demand_voltage = demand_voltage.max(0.0 * volt);
//       }
//       if Some(true) == config.limit_switches.1.get_state() {
//         demand_voltage = demand_voltage.min(feedforward);
//       }

//       self.publishers.demand_voltage.set(demand_voltage.to::<volt>()).ok();
//       self.publishers.height.set(current_height.to::<meter>()).ok();
//       self.publishers.state.set(state_msg).ok();

//       drop(config);
//       self.config.write().unwrap().voltage_controller.set_voltage(demand_voltage);
//       tokio::time::sleep(Duration::from_millis(100)).await;
//     }
//   }
// }

// pub struct SimElevator<
//   M: SpooledMotorForwardDynamics + SpooledMotorInverseDynamics,
//   HS: DisplacementSensor + SimulatedSensor<Length>,
//   LS1: BinarySensor + SimulatedSensor<bool>,
//   LS2: BinarySensor + SimulatedSensor<bool>
// > {
//   config: Shared<ElevatorConfig<M, HS, LS1, LS2>>
// }

// impl<
//   M: SpooledMotorForwardDynamics + SpooledMotorInverseDynamics,
//   HS: DisplacementSensor + SimulatedSensor<Length>,
//   LS1: BinarySensor + SimulatedSensor<bool>,
//   LS2: BinarySensor + SimulatedSensor<bool>
// > SimElevator<M, HS, LS1, LS2> {
//   pub fn new(config: Shared<ElevatorConfig<M, HS, LS1, LS2>>) -> Self {
//     Self { config }
//   }

//   pub async fn sim(&self) {
//     let mut sim_speed = 0.0 * meters_per_second;
//     let dt = 5.0 * millisecond;
//     loop {
//       let config = self.config.write().unwrap();
//       let current_height = config.height_sensor.get_displacement().unwrap_or(0.0 * meter);

//       let demand_voltage = config.voltage_controller.get_set_voltage();

//       let f_down = config.mass * (-9.81 * meters_per_second2);
//       let f_up = config.motor.force(demand_voltage, sim_speed);
//       let f_net = f_up + f_down;

//       let acceleration = f_net / config.mass;
//       let mut new_velocity = sim_speed + acceleration * dt;

//       if new_velocity > Zero::zero() && (current_height + new_velocity * dt) > config.height_limit.1 {
//         new_velocity = Zero::zero();
//       } else if new_velocity < Zero::zero() && (current_height + new_velocity * dt) < config.height_limit.0 {
//         new_velocity = Zero::zero();
//       }

//       sim_speed = new_velocity;
//       config.height_sensor.set_sensor_value(Some(current_height + new_velocity * dt));

//       tokio::time::sleep(tokio::time::Duration::from_secs_f64(dt.to::<second>())).await;
//     }
//   }
// }