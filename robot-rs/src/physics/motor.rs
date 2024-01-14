use robot_rs_units::{electrical::Voltage, motion::AngularVelocity, force::Torque, Quantity, ISQ, typenum::{N1, Z0, N2, P2, P1}, Current};

use crate::traits::Wrapper;

pub type Kt = Quantity<ISQ<N1, Z0, Z0, N1, Z0, Z0, Z0, Z0, Z0>>;
pub type Kw = Quantity<ISQ<N2, P2, P1, N1, Z0, Z0, Z0, N1, Z0>>;
pub type Ki = Quantity<ISQ<P2, N2, N1, P1, Z0, Z0, Z0, Z0, Z0>>;

pub trait MotorExtensionTrait : Sized {
  fn current_aware(self, ki: Ki) -> CurrentAwareMotor<Self> {
    CurrentAwareMotor::new(self, ki)
  }

  fn geared(self, ratio: f64) -> GearedMotor<Self> {
    GearedMotor::new(self, ratio)
  }

  fn multiply(self, n_motors: usize) -> MultiMotor<Self> {
    MultiMotor::new(self, n_motors)
  }
}

pub trait MotorForwardDynamics {
  fn voltage(&self, torque: Torque, speed: AngularVelocity) -> Voltage;
}

pub trait MotorInverseDynamics {
  fn speed(&self, voltage: Voltage, torque: Torque) -> AngularVelocity;
  fn torque(&self, voltage: Voltage, speed: AngularVelocity) -> Torque;
}

pub trait MotorCurrentDynamics {
  fn current(&self, torque: Torque) -> Current;
  fn torque(&self, current: Current) -> Torque;
}

#[derive(Clone, Debug)]
pub struct Motor {
  pub kt: Kt,
  pub kw: Kw,
}

impl Motor {
  pub fn from_coeffs(kt: Kt, kw: Kw) -> Self {
    Self { kt, kw }
  }

  pub fn from_params(v_nom: Voltage, free_speed: AngularVelocity, free_torque: Torque, stall_torque: Torque) -> Self {
    Self {
      kt: v_nom / stall_torque,
      kw: (v_nom - ((v_nom / stall_torque) * free_torque)) / (free_speed),
    }
  }
}

impl MotorExtensionTrait for Motor {}

impl MotorForwardDynamics for Motor {
  #[inline(always)]
  fn voltage(&self, torque: Torque, speed: AngularVelocity) -> Voltage {
    self.kt * torque + self.kw * speed
  }
}

impl MotorInverseDynamics for Motor {
  #[inline(always)]
  fn speed(&self, voltage: Voltage, torque: Torque) -> AngularVelocity {
    (voltage - self.kt * torque) / self.kw
  }

  #[inline(always)]
  fn torque(&self, voltage: Voltage, speed: AngularVelocity) -> Torque {
    (voltage - self.kw * speed) / self.kt
  }
}

#[derive(Clone, Debug)]
pub struct CurrentAwareMotor<T> {
  pub motor: T,
  pub ki: Ki,
}

impl<T> CurrentAwareMotor<T> {
  pub fn new(motor: T, ki: Ki) -> Self {
    Self { motor, ki }
  }
}

impl CurrentAwareMotor<Motor> {
  pub fn from_params(v_nom: Voltage, free_speed: AngularVelocity, free_current: Current, stall_torque: Torque, stall_current: Current) -> Self {
    let ki = stall_current / stall_torque;
    let motor = Motor::from_params(v_nom, free_speed, free_current / ki, stall_torque);
    CurrentAwareMotor::new(motor, ki)
  }
}

impl<T> MotorExtensionTrait for CurrentAwareMotor<T> {}

impl<T> Wrapper<T> for CurrentAwareMotor<T> {
  fn eject(self) -> T {
    self.motor
  }
}

impl<T: MotorForwardDynamics> MotorForwardDynamics for CurrentAwareMotor<T> {
  #[inline(always)]
  fn voltage(&self, torque: Torque, speed: AngularVelocity) -> Voltage {
    self.motor.voltage(torque, speed)
  }
}

impl<T: MotorInverseDynamics> MotorInverseDynamics for CurrentAwareMotor<T> {
  #[inline(always)]
  fn speed(&self, voltage: Voltage, torque: Torque) -> AngularVelocity {
    self.motor.speed(voltage, torque)
  }

  #[inline(always)]
  fn torque(&self, voltage: Voltage, speed: AngularVelocity) -> Torque {
    self.motor.torque(voltage, speed)
  }
}

impl<T> MotorCurrentDynamics for CurrentAwareMotor<T> {
  #[inline(always)]
  fn current(&self, torque: Torque) -> Current {
    self.ki * torque
  }

  #[inline(always)]
  fn torque(&self, current: Current) -> Torque {
    current / self.ki
  }
}

#[derive(Clone, Debug)]
pub struct GearedMotor<T> {
  pub motor: T,
  pub ratio: f64
}

impl<T> GearedMotor<T> {
  pub fn new(motor: T, ratio: f64) -> Self {
    Self { motor, ratio }
  }
}

impl<T> MotorExtensionTrait for GearedMotor<T> {}

impl<T> Wrapper<T> for GearedMotor<T> {
  fn eject(self) -> T {
    self.motor
  }
}

impl<T: MotorForwardDynamics> MotorForwardDynamics for GearedMotor<T> {
  #[inline(always)]
  fn voltage(&self, torque: Torque, speed: AngularVelocity) -> Voltage {
    self.motor.voltage(torque / self.ratio, speed * self.ratio)
  }
}

impl<T: MotorInverseDynamics> MotorInverseDynamics for GearedMotor<T> {
  #[inline(always)]
  fn speed(&self, voltage: Voltage, torque: Torque) -> AngularVelocity {
    self.motor.speed(voltage, torque / self.ratio) / self.ratio
  }

  #[inline(always)]
  fn torque(&self, voltage: Voltage, speed: AngularVelocity) -> Torque {
    self.motor.torque(voltage, speed * self.ratio) * self.ratio
  }
}

impl<T: MotorCurrentDynamics> MotorCurrentDynamics for GearedMotor<T> {
  #[inline(always)]
  fn current(&self, torque: Torque) -> Current {
    self.motor.current(torque / self.ratio)
  }

  #[inline(always)]
  fn torque(&self, current: Current) -> Torque {
    self.motor.torque(current) * self.ratio
  }
}

#[derive(Debug, Clone)]
pub struct MultiMotor<T> {
  pub motor: T,
  pub n_motors: usize
}

impl<T> MultiMotor<T> {
  pub fn new(motor: T, n_motors: usize) -> Self {
    Self { motor, n_motors }
  }
}

impl<T> MotorExtensionTrait for MultiMotor<T> {}

impl<T> Wrapper<T> for MultiMotor<T> {
  fn eject(self) -> T {
    self.motor
  }
}

impl<T: MotorForwardDynamics> MotorForwardDynamics for MultiMotor<T> {
  #[inline(always)]
  fn voltage(&self, torque: Torque, speed: AngularVelocity) -> Voltage {
    self.motor.voltage(torque / self.n_motors as f64, speed)
  }
}

impl<T: MotorInverseDynamics> MotorInverseDynamics for MultiMotor<T> {
  #[inline(always)]
  fn speed(&self, voltage: Voltage, torque: Torque) -> AngularVelocity {
    self.motor.speed(voltage, torque / self.n_motors as f64)
  }

  #[inline(always)]
  fn torque(&self, voltage: Voltage, speed: AngularVelocity) -> Torque {
    self.motor.torque(voltage, speed) * self.n_motors as f64
  }
}

impl<T: MotorCurrentDynamics> MotorCurrentDynamics for MultiMotor<T> {
  #[inline(always)]
  fn current(&self, torque: Torque) -> Current {
    self.motor.current(torque / self.n_motors as f64) * self.n_motors as f64
  }

  #[inline(always)]
  fn torque(&self, current: Current) -> Torque {
    self.motor.torque(current / self.n_motors as f64) * self.n_motors as f64
  }
}

pub mod from_dyno {
  use robot_rs_units::ampere;
  use robot_rs_units::electrical::volt;
  use robot_rs_units::motion::revolutions_per_minute;
  use robot_rs_units::force::newton_meter;

  macro_rules! define_motor {
    ($name:ident, $voltage:expr, $free_speed:expr, $free_current:expr, $stall_torque:expr, $stall_current:expr) => {
      #[allow(non_snake_case)]
      pub fn $name() -> super::CurrentAwareMotor<super::Motor> {
      super::CurrentAwareMotor::<super::Motor>::from_params($voltage as f64 * volt, $free_speed as f64 * revolutions_per_minute, $free_current as f64 * ampere, $stall_torque as f64 * newton_meter, $stall_current as f64 * ampere)
      }
    };
  }

  define_motor!(Falcon500,    12.0, 6380,   1.5,  4.69, 257);
  define_motor!(NEO,          12.0, 5880,   1.3,  3.36, 166);
  define_motor!(NEO550,       12.0, 11710,  1.1,  1.08, 111);
  define_motor!(CIM,          12.0, 5330,   2.7,  2.41, 131);
  define_motor!(MiniCIM,      12.0, 5840,   3,    1.41, 89);
  define_motor!(BAG,          12.0, 13180,  1.8,  0.43, 53);
  define_motor!(vex775pro,    12.0, 18730,  0.7,  0.71, 134);
  define_motor!(KrakenTrap,   12.0, 6000,   2,    7.09, 366);
  define_motor!(KrakenFOC,    12.0, 5800,   2,    9.37, 483);
}
