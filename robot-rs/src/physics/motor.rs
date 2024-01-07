pub trait MotorExtensionTrait : Sized {
  fn current_aware(self, ki: f64) -> CurrentAwareMotor<Self> {
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
  fn voltage(&self, torque: f64, speed: f64) -> f64;
}

pub trait MotorInverseDynamics {
  fn speed(&self, voltage: f64, torque: f64) -> f64;
  fn torque(&self, voltage: f64, speed: f64) -> f64;
}

pub trait MotorCurrentDynamics {
  fn current(&self, torque: f64) -> f64;
  fn torque(&self, current: f64) -> f64;
}

#[derive(Clone, Debug)]
pub struct Motor {
  pub kt: f64,
  pub kw: f64,
}

impl Motor {
  pub fn from_coeffs(kt: f64, kw: f64, k0: f64) -> Self {
    Self { kt, kw }
  }

  pub fn from_params(v_nom: f64, free_speed: f64, free_torque: f64, stall_torque: f64) -> Self {
    Self {
      kt: v_nom / stall_torque,
      kw: (v_nom - ((v_nom / stall_torque) * free_torque)) / (free_speed),
    }
  }
}

impl MotorExtensionTrait for Motor {}

impl MotorForwardDynamics for Motor {
  #[inline(always)]
  fn voltage(&self, torque: f64, speed: f64) -> f64 {
    self.kt * torque + self.kw * speed
  }
}

impl MotorInverseDynamics for Motor {
  #[inline(always)]
  fn speed(&self, voltage: f64, torque: f64) -> f64 {
    (voltage - self.kt * torque) / self.kw
  }

  #[inline(always)]
  fn torque(&self, voltage: f64, speed: f64) -> f64 {
    (voltage - self.kw * speed) / self.kt
  }
}

#[derive(Clone, Debug)]
pub struct CurrentAwareMotor<T> {
  pub motor: T,
  pub ki: f64,
}

impl<T> CurrentAwareMotor<T> {
  pub fn new(motor: T, ki: f64) -> Self {
    Self { motor, ki }
  }
}

impl CurrentAwareMotor<Motor> {
  pub fn from_params(v_nom: f64, free_speed: f64, free_current: f64, stall_torque: f64, stall_current: f64) -> Self {
    let ki = stall_current / stall_torque;
    let motor = Motor::from_params(v_nom, free_speed, free_current / ki, stall_torque);
    CurrentAwareMotor::new(motor, ki)
  }
}

impl<T> MotorExtensionTrait for CurrentAwareMotor<T> {}

impl<T: MotorForwardDynamics> MotorForwardDynamics for CurrentAwareMotor<T> {
  #[inline(always)]
  fn voltage(&self, torque: f64, speed: f64) -> f64 {
    self.motor.voltage(torque, speed)
  }
}

impl<T: MotorInverseDynamics> MotorInverseDynamics for CurrentAwareMotor<T> {
  #[inline(always)]
  fn speed(&self, voltage: f64, torque: f64) -> f64 {
    self.motor.speed(voltage, torque)
  }

  #[inline(always)]
  fn torque(&self, voltage: f64, speed: f64) -> f64 {
    self.motor.torque(voltage, speed)
  }
}

impl<T> MotorCurrentDynamics for CurrentAwareMotor<T> {
  #[inline(always)]
  fn current(&self, torque: f64) -> f64 {
    self.ki * torque
  }

  #[inline(always)]
  fn torque(&self, current: f64) -> f64 {
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

impl<T: MotorForwardDynamics> MotorForwardDynamics for GearedMotor<T> {
  #[inline(always)]
  fn voltage(&self, torque: f64, speed: f64) -> f64 {
    self.motor.voltage(torque / self.ratio, speed * self.ratio)
  }
}

impl<T: MotorInverseDynamics> MotorInverseDynamics for GearedMotor<T> {
  #[inline(always)]
  fn speed(&self, voltage: f64, torque: f64) -> f64 {
    self.motor.speed(voltage, torque / self.ratio) / self.ratio
  }

  #[inline(always)]
  fn torque(&self, voltage: f64, speed: f64) -> f64 {
    self.motor.torque(voltage, speed * self.ratio) * self.ratio
  }
}

impl<T: MotorCurrentDynamics> MotorCurrentDynamics for GearedMotor<T> {
  #[inline(always)]
  fn current(&self, torque: f64) -> f64 {
    self.motor.current(torque / self.ratio)
  }

  #[inline(always)]
  fn torque(&self, current: f64) -> f64 {
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

impl<T: MotorForwardDynamics> MotorForwardDynamics for MultiMotor<T> {
  #[inline(always)]
  fn voltage(&self, torque: f64, speed: f64) -> f64 {
    self.motor.voltage(torque / self.n_motors as f64, speed)
  }
}

impl<T: MotorInverseDynamics> MotorInverseDynamics for MultiMotor<T> {
  #[inline(always)]
  fn speed(&self, voltage: f64, torque: f64) -> f64 {
    self.motor.speed(voltage, torque / self.n_motors as f64)
  }

  #[inline(always)]
  fn torque(&self, voltage: f64, speed: f64) -> f64 {
    self.motor.torque(voltage, speed) * self.n_motors as f64
  }
}

impl<T: MotorCurrentDynamics> MotorCurrentDynamics for MultiMotor<T> {
  #[inline(always)]
  fn current(&self, torque: f64) -> f64 {
    self.motor.current(torque / self.n_motors as f64) * self.n_motors as f64
  }

  #[inline(always)]
  fn torque(&self, current: f64) -> f64 {
    self.motor.current(current / self.n_motors as f64) * self.n_motors as f64
  }
}

pub mod from_dyno {
  macro_rules! define_motor {
    ($name:ident, $($args:expr),*) => {
      #[allow(non_snake_case)]
      pub fn $name() -> super::CurrentAwareMotor<super::Motor> { super::CurrentAwareMotor::<super::Motor>::from_params($($args as f64),*) }
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

#[cfg(test)]
mod test {
  use approx::assert_relative_eq;
  use crate::physics::motor::{MotorForwardDynamics, MotorInverseDynamics, MultiMotor};

  use super::{Motor, GearedMotor};

  #[test]
  fn test_geared_motor() {
    let motor = GearedMotor::new(Motor::from_coeffs(1.23, 2.3, 4.7), 10.0);
    assert_relative_eq!(motor.voltage(motor.torque(12.0, 4.5), 4.5), 12.0, epsilon = 0.0001);
    assert_relative_eq!(motor.voltage(1.73, motor.speed(12.0, 1.73)), 12.0, epsilon = 0.0001);

    assert_relative_eq!(
      motor.torque(12.0, 4.5),
      motor.motor.torque(12.0, 4.5 * 10.0) * 10.0
    );

    assert_relative_eq!(
      motor.speed(12.0, 4.5),
      motor.motor.speed(12.0, 4.5 / 10.0) / 10.0
    );
  }

  #[test]
  fn test_multi_motor() {
    let motor = MultiMotor::new(Motor::from_coeffs(1.23, 2.3, 4.7), 2);
    assert_relative_eq!(motor.voltage(motor.torque(12.0, 4.5), 4.5), 12.0, epsilon = 0.0001);
    assert_relative_eq!(motor.voltage(1.73, motor.speed(12.0, 1.73)), 12.0, epsilon = 0.0001);

    assert_relative_eq!(
      motor.torque(12.0, 4.5),
      motor.motor.torque(12.0, 4.5) * 2.0
    );

    assert_relative_eq!(
      motor.speed(12.0, 4.5),
      motor.motor.speed(12.0, 4.5 / 2.0)
    );
  }
}