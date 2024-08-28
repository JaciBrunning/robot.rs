use std::ops::Div;

use robot_rs_units::{
  electrical::Voltage,
  force::{newton, newton_meter, Force, Torque, MOI},
  motion::{
    meters_per_second, rads_per_second, Acceleration, AngularAcceleration, AngularVelocity,
    Velocity,
  },
  radian,
  typenum::{N1, N2, P1, P2, Z0},
  Current, Length, Mass, Quantity, ISQ,
};

use crate::traits::Wrapper;

pub type Kt = Quantity<ISQ<N1, Z0, Z0, N1, Z0, Z0, Z0, Z0, Z0>>;
pub type Kw = Quantity<ISQ<N2, P2, P1, N1, Z0, Z0, Z0, N1, Z0>>;
pub type Ki = Quantity<ISQ<P2, N2, N1, P1, Z0, Z0, Z0, Z0, Z0>>;

pub trait MotorExtensionTrait: Sized {
  fn current_aware(self, ki: Ki) -> CurrentAwareMotor<Self> {
    CurrentAwareMotor::new(self, ki)
  }

  fn geared(self, ratio: f64) -> GearedMotor<Self> {
    GearedMotor::new(ratio, self)
  }

  fn multiply(self, n_motors: usize) -> MultiMotor<Self> {
    MultiMotor::new(n_motors, self)
  }

  fn to_linear(self, spool_radius: Length) -> AngularToLinearMotor<Self> {
    AngularToLinearMotor::new(spool_radius, self)
  }
}

pub trait MotorForwardDynamics {
  fn voltage(&self, torque: Torque, speed: AngularVelocity) -> Voltage;
}

pub trait MotorInverseDynamics {
  fn speed(&self, voltage: Voltage, torque: Torque) -> AngularVelocity;
  fn torque(&self, voltage: Voltage, speed: AngularVelocity) -> Torque;

  fn estimate_profile_kv(&self, v_nom: Voltage) -> <Voltage as Div<AngularVelocity>>::Output {
    v_nom / self.speed(v_nom, 0.0 * newton_meter)
  }

  fn estimate_profile_ka(
    &self,
    moi: MOI,
    v_nom: Voltage,
  ) -> <Voltage as Div<AngularAcceleration>>::Output {
    v_nom / (self.torque(v_nom, 0.0 * rads_per_second) / moi * (1.0 * radian))
  }
}

pub trait MotorCurrentDynamics {
  fn current(&self, torque: Torque) -> Current;
  fn torque_from_current(&self, current: Current) -> Torque;
}

pub trait MotorDynamics:
  MotorForwardDynamics + MotorInverseDynamics + MotorCurrentDynamics
{
}
impl<T: MotorForwardDynamics + MotorInverseDynamics + MotorCurrentDynamics> MotorDynamics for T {}

pub trait SpooledMotorForwardDynamics {
  fn voltage(&self, force: Force, velocity: Velocity) -> Voltage;
}

pub trait SpooledMotorInverseDynamics {
  fn velocity(&self, voltage: Voltage, force: Force) -> Velocity;
  fn force(&self, voltage: Voltage, velocity: Velocity) -> Force;

  fn estimate_profile_kv(&self, v_nom: Voltage) -> <Voltage as Div<Velocity>>::Output {
    v_nom / self.velocity(v_nom, 0.0 * newton)
  }

  fn estimate_profile_ka(
    &self,
    mass: Mass,
    v_nom: Voltage,
  ) -> <Voltage as Div<Acceleration>>::Output {
    v_nom / (self.force(v_nom, 0.0 * meters_per_second) / mass)
  }
}

pub trait SpooledMotorCurrentDynamics {
  fn current(&self, force: Force) -> Current;
  fn force_from_current(&self, current: Current) -> Force;
}

pub trait SpooledMotorDynamics:
  SpooledMotorForwardDynamics + SpooledMotorInverseDynamics + SpooledMotorCurrentDynamics
{
}
impl<T: SpooledMotorForwardDynamics + SpooledMotorInverseDynamics + SpooledMotorCurrentDynamics>
  SpooledMotorDynamics for T
{
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

  pub fn from_params(
    v_nom: Voltage,
    free_speed: AngularVelocity,
    free_torque: Torque,
    stall_torque: Torque,
  ) -> Self {
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
  pub fn from_params(
    v_nom: Voltage,
    free_speed: AngularVelocity,
    free_current: Current,
    stall_torque: Torque,
    stall_current: Current,
  ) -> Self {
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
  fn torque_from_current(&self, current: Current) -> Torque {
    current / self.ki
  }
}

#[derive(Clone, Debug)]
pub struct GearedMotor<T> {
  pub motor: T,
  pub ratio: f64,
}

impl<T> GearedMotor<T> {
  pub fn new(ratio: f64, motor: T) -> Self {
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
  fn torque_from_current(&self, current: Current) -> Torque {
    self.motor.torque_from_current(current) * self.ratio
  }
}

#[derive(Debug, Clone)]
pub struct MultiMotor<T> {
  pub motor: T,
  pub n_motors: usize,
}

impl<T> MultiMotor<T> {
  pub fn new(n_motors: usize, motor: T) -> Self {
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
  fn torque_from_current(&self, current: Current) -> Torque {
    self
      .motor
      .torque_from_current(current / self.n_motors as f64)
      * self.n_motors as f64
  }
}

#[derive(Debug, Clone)]
pub struct AngularToLinearMotor<T> {
  pub motor: T,
  pub spool_radius: Length,
}

impl<T> AngularToLinearMotor<T> {
  pub fn new(spool_radius: Length, motor: T) -> Self {
    Self {
      motor,
      spool_radius,
    }
  }
}

impl<T> Wrapper<T> for AngularToLinearMotor<T> {
  fn eject(self) -> T {
    self.motor
  }
}

impl<T: MotorForwardDynamics> SpooledMotorForwardDynamics for AngularToLinearMotor<T> {
  fn voltage(&self, force: Force, velocity: Velocity) -> Voltage {
    self.motor.voltage(
      force * self.spool_radius,
      velocity / self.spool_radius * (1.0 * radian),
    )
  }
}

impl<T: MotorInverseDynamics> SpooledMotorInverseDynamics for AngularToLinearMotor<T> {
  fn velocity(&self, voltage: Voltage, force: Force) -> Velocity {
    self.motor.speed(voltage, force * self.spool_radius) * self.spool_radius / (1.0 * radian)
  }

  fn force(&self, voltage: Voltage, velocity: Velocity) -> Force {
    self
      .motor
      .torque(voltage, velocity / self.spool_radius * (1.0 * radian))
      / self.spool_radius
  }
}

impl<T: MotorCurrentDynamics> SpooledMotorCurrentDynamics for AngularToLinearMotor<T> {
  fn current(&self, force: Force) -> Current {
    self.motor.current(force * self.spool_radius)
  }

  fn force_from_current(&self, current: Current) -> Force {
    self.motor.torque_from_current(current) / self.spool_radius
  }
}

pub mod from_dyno {
  use robot_rs_units::ampere;
  use robot_rs_units::electrical::volt;
  use robot_rs_units::force::newton_meter;
  use robot_rs_units::motion::revolutions_per_minute;

  macro_rules! define_motor {
    ($name:ident, $voltage:expr, $free_speed:expr, $free_current:expr, $stall_torque:expr, $stall_current:expr) => {
      #[allow(non_snake_case)]
      pub fn $name() -> super::CurrentAwareMotor<super::Motor> {
        super::CurrentAwareMotor::<super::Motor>::from_params(
          $voltage as f64 * volt,
          $free_speed as f64 * revolutions_per_minute,
          $free_current as f64 * ampere,
          $stall_torque as f64 * newton_meter,
          $stall_current as f64 * ampere,
        )
      }
    };
  }

  define_motor!(Falcon500, 12.0, 6380, 1.5, 4.69, 257);
  define_motor!(NEO, 12.0, 5880, 1.3, 3.36, 166);
  define_motor!(NEO550, 12.0, 11710, 1.1, 1.08, 111);
  define_motor!(CIM, 12.0, 5330, 2.7, 2.41, 131);
  define_motor!(MiniCIM, 12.0, 5840, 3, 1.41, 89);
  define_motor!(BAG, 12.0, 13180, 1.8, 0.43, 53);
  define_motor!(vex775pro, 12.0, 18730, 0.7, 0.71, 134);
  define_motor!(KrakenTrap, 12.0, 6000, 2, 7.09, 366);
  define_motor!(KrakenFOC, 12.0, 5800, 2, 9.37, 483);
}
