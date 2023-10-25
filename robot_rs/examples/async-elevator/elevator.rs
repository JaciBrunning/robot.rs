use std::error::Error;

use nt4_rs::nt;
use robot_rs::{
    actuators::motors::MotorController,
    control::{
        control_lock::{ControlLock, ControlLockResource},
        pid::{PIDConfig, PID},
    },
    models::DcMotor,
    sensors::distance::{DistanceSource, SimDistanceSource},
    time::now,
    types::MinMax,
};
use strum::Display;
use tokio::sync::RwLock;

#[derive(Clone, Debug, Display)]
pub enum ElevatorError {
    Interrupted,
}
impl Error for ElevatorError {}

pub type ElevatorResult<T> = Result<T, ElevatorError>;

/* Generic Elevator */

/// Trait for a generic elevator. We use a trait so we can pass it around without fully
/// qualifying the generic args for Elevator. Indeed, you can use this to have a different elevator
/// implementation for different robots with the same core behaviour
#[async_trait::async_trait]
pub trait AbstractElevator<'a>: Sized {
    fn control(&self) -> &ControlLockResource<'a, Self>;
    async fn set_voltage(&self, v: f64, lock: &ControlLock<'a, Self>) -> ElevatorResult<()>;
    async fn go_to_height(&self, height: f64, lock: &ControlLock<'a, Self>) -> ElevatorResult<()>;
}

/* Elevator Implementation */

#[derive(Debug, Clone)]
pub enum ElevatorState {
    Idle,
    Manual { voltage: f64 },
    HeightControl { height: f64 },
}

/// Configuration for an elevator. We use a separate configuration so we can pass it straight to the elevator
/// and leave the private, operating components to be initialised by the elevator itself.
pub struct ElevatorConfig<M, H> {
    pub motor: RwLock<M>,
    pub height_sensor: RwLock<H>,
    pub height_limit: MinMax<f64>,
    pub mass: f64,
    pub motor_model: DcMotor,
    pub spool_radius: f64,
}

/// Actual elevator class. This is what runs the elevator logic. In impl, you will see this implements
/// AbstractElevator
pub struct Elevator<'a, M, H> {
    config: ElevatorConfig<M, H>,
    state: RwLock<ElevatorState>,
    control_lock: ControlLockResource<'a, Self>,
    pid: RwLock<PID>,

    sim_speed: RwLock<f64>,
}

#[async_trait::async_trait]
impl<'a, M, H> AbstractElevator<'a> for Elevator<'a, M, H>
where
    M: MotorController + Send + Sync,
    H: DistanceSource + Send + Sync,
{
    fn control(&self) -> &ControlLockResource<'a, Self> {
        &self.control_lock
    }

    async fn set_voltage(&self, v: f64, lock: &ControlLock<'a, Self>) -> ElevatorResult<()> {
        if lock.is_mine().await {
            self.pid.write().await.reset(); // Reset the PID so IsStable() is reset
            *self.state.write().await = ElevatorState::Manual { voltage: v };
            Ok(())
        } else {
            Err(ElevatorError::Interrupted)
        }
    }

    async fn go_to_height(&self, height: f64, lock: &ControlLock<'a, Self>) -> ElevatorResult<()> {
        while let Some(_) = lock.get().await {
            *self.state.write().await = ElevatorState::HeightControl { height };
            if self.is_stable().await {
                return Ok(());
            }
            tokio::time::sleep(tokio::time::Duration::from_millis(10)).await;
        }
        Err(ElevatorError::Interrupted)
    }
}

impl<'a, M, H> Elevator<'a, M, H>
where
    M: MotorController,
    H: DistanceSource,
{
    pub fn new(config: ElevatorConfig<M, H>, pid_config: PIDConfig) -> Self {
        Self {
            config,
            state: RwLock::new(ElevatorState::Idle),
            control_lock: Default::default(),
            pid: RwLock::new(PID::new(pid_config, 0.0, 0.2)),

            sim_speed: RwLock::new(0.0),
        }
    }

    /// Run the elevator. This is an async function, so you can call it and let it loop
    /// using something like tokio::task::spawn(), instead of needing to spin up a new thread.
    /// This function will also update NetworkTables with information about the elevator and its PID loop.
    /// PID loop coefficients can also be updated from NetworkTables.
    pub async fn run(&self) {
        loop {
            let mut demand_voltage = 0.0;
            let current_height = self.config.height_sensor.read().await.get_distance();
            let mut pid = self.pid.write().await;

            match &*self.state.read().await {
                ElevatorState::Idle => pid.reset(),
                ElevatorState::Manual { voltage } => {
                    pid.reset();
                    demand_voltage = *voltage
                }
                ElevatorState::HeightControl { height } => {
                    pid.set_setpoint(*height);

                    let feedforward = self
                        .config
                        .motor_model
                        .voltage(self.config.mass * 9.81 * self.config.spool_radius, 0.0);

                    demand_voltage = pid.calculate(current_height, now()).output + feedforward;
                }
            }

            nt!("elevator/state", format!("{:?}", self.state.read().await)).unwrap();
            nt!("elevator/height", current_height).unwrap();
            nt!("elevator/voltage", demand_voltage).unwrap();
            pid.nt_update("elevator");

            self.config.motor.write().await.set_voltage(demand_voltage);
            tokio::time::sleep(tokio::time::Duration::from_millis(20)).await;
        }
    }

    /// Check if the loop is stable - i.e, has the elevator successfully reached where we want it
    /// to be.
    pub async fn is_stable(&self) -> bool {
        self.pid.read().await.is_stable(0.05, Some(0.1))
    }
}

/* Simulation */

impl<'a, M, H> Elevator<'a, M, H>
where
    M: MotorController,
    H: DistanceSource + SimDistanceSource,
{
    /// Simulation loop for the elevator. This runs the physics simulation of the elevator
    /// when not deployed on a RoboRIO.
    pub async fn run_sim(&self) {
        loop {
            let dt = 5.0 / 1000.0;
            let current_height = self.config.height_sensor.read().await.get_distance();

            let motor_speed = (*self.sim_speed.read().await / self.config.spool_radius).into();
            let voltage = self.config.motor.read().await.get_set_voltage();

            let f_down = self.config.mass * -9.81;
            let f_up = self
                .config
                .motor_model
                .torque(self.config.motor_model.current(motor_speed, voltage))
                / self.config.spool_radius;
            let f_net = f_up + f_down;

            let acceleration = f_net / self.config.mass;
            let mut new_velocity = *self.sim_speed.read().await + acceleration * dt;

            // Physical limits
            if new_velocity > 0.0
                && current_height + new_velocity * dt > self.config.height_limit.max
            {
                new_velocity = 0.0;
            } else if new_velocity < 0.0
                && current_height - new_velocity * dt < self.config.height_limit.min
            {
                new_velocity = 0.0;
            }

            *self.sim_speed.write().await = new_velocity;
            self.config
                .height_sensor
                .write()
                .await
                .set_distance(current_height + new_velocity * dt);

            tokio::time::sleep(tokio::time::Duration::from_secs_f64(dt)).await;
        }
    }
}
