use std::{
  marker::PhantomData,
  ops::{Add, Div, Mul, Neg},
};

use num_traits::Zero;
use robot_rs_units::{
  electrical::Voltage,
  force::MOI,
  motion::{AngularVelocity, Velocity},
  traits::{FromFloat, MaybeUnitNumber, ToFloat},
  Mass, Time,
};

use crate::physics::motor::{MotorInverseDynamics, SpooledMotorInverseDynamics};

use super::{HasSetpoint, StatefulTransform, Transform};

pub type Displacement<Vel, Time> = <Vel as Mul<Time>>::Output;
pub type Acceleration<Vel, Time> = <Vel as Div<Time>>::Output;

pub type KV<Output, Vel> = <Output as Div<Vel>>::Output;
pub type KA<Output, Vel, Time> = <Output as Div<Acceleration<Vel, Time>>>::Output;

#[derive(Clone, Copy)]
pub struct ProfileState<Vel, Time>
where
  Vel: Div<Time> + Mul<Time> + Copy,
  Displacement<Vel, Time>: Copy,
  Acceleration<Vel, Time>: Copy,
{
  pub displacement: Displacement<Vel, Time>,
  pub velocity: Vel,
  pub acceleration: Acceleration<Vel, Time>, // Ignored in setpoint (for trapezoidal profiles)
}

impl<Vel, Time> ProfileState<Vel, Time>
where
  Vel: Div<Time> + Mul<Time> + Copy + Zero,
  Displacement<Vel, Time>: Copy + Zero,
  Acceleration<Vel, Time>: Copy + Zero,
  Time: Zero,
{
  pub fn zero() -> Self {
    Self {
      displacement: Zero::zero(),
      velocity: Zero::zero(),
      acceleration: Zero::zero(),
    }
  }
}

pub struct TrapezoidalProfile<Vel: Div<Time> + Mul<Time>, Time>
where
  Vel: Copy,
  Displacement<Vel, Time>: Copy,
  Acceleration<Vel, Time>: Copy,
{
  pub max_velocity: Vel,
  pub max_acceleration: Acceleration<Vel, Time>,
  pub setpoint: ProfileState<Vel, Time>,
  pub start_time: Option<Time>,
}

impl<Vel: Div<Time> + Mul<Time>, Time> TrapezoidalProfile<Vel, Time>
where
  Vel: Copy,
  Displacement<Vel, Time>: Copy,
  Acceleration<Vel, Time>: Copy,
{
  pub fn new(
    max_velocity: Vel,
    max_acceleration: Acceleration<Vel, Time>,
    setpoint: ProfileState<Vel, Time>,
  ) -> Self {
    Self {
      max_velocity,
      max_acceleration,
      setpoint,
      start_time: None,
    }
  }
}

impl<Vel: Div<Time> + Mul<Time>, Time> HasSetpoint<ProfileState<Vel, Time>>
  for TrapezoidalProfile<Vel, Time>
where
  Vel: Copy,
  Displacement<Vel, Time>: Copy,
  Acceleration<Vel, Time>: Copy,
{
  fn set_setpoint(&mut self, sp: ProfileState<Vel, Time>) {
    self.setpoint = sp;
  }
}

fn maybe_flip<Vel, Time>(flip: bool, mut input: ProfileState<Vel, Time>) -> ProfileState<Vel, Time>
where
  Vel: Neg<Output = Vel> + Div<Time> + Mul<Time> + Copy,
  Displacement<Vel, Time>: Neg<Output = Displacement<Vel, Time>> + Copy,
  Acceleration<Vel, Time>: Neg<Output = Acceleration<Vel, Time>> + Copy,
{
  if flip {
    input.acceleration = -input.acceleration;
    input.displacement = -input.displacement;
    input.velocity = -input.velocity;
  }
  input
}

impl<Vel, Time> StatefulTransform<ProfileState<Vel, Time>, Time> for TrapezoidalProfile<Vel, Time>
where
  Vel: MaybeUnitNumber
    + Div<Time>
    + Mul<Time>
    + Mul<Vel>
    + Div<Acceleration<Vel, Time>, Output = Time>
    + Copy,
  Time: MaybeUnitNumber + Mul<Time> + Copy + FromFloat,
  <Time as Mul<Time>>::Output: Mul<Acceleration<Vel, Time>, Output = Displacement<Vel, Time>>,
  <Vel as Mul<Vel>>::Output: MaybeUnitNumber
    + Neg<Output = <Vel as Mul<Vel>>::Output>
    + Div<Acceleration<Vel, Time>, Output = Displacement<Vel, Time>>
    + Copy,
  Displacement<Vel, Time>: MaybeUnitNumber + Copy + ToFloat + Div<Vel, Output = Time>,
  Acceleration<Vel, Time>: MaybeUnitNumber
    + Neg<Output = <Vel as Div<Time>>::Output>
    + Mul<Time, Output = Vel>
    + Copy
    + ToFloat,
  f64: Mul<Acceleration<Vel, Time>, Output = Acceleration<Vel, Time>> + Mul<Vel, Output = Vel>,
{
  type Output = ProfileState<Vel, Time>;

  // Copied from WPILib's TrapezoidProfile: https://github.com/wpilibsuite/allwpilib/blob/main/wpimath/src/main/native/include/frc/trajectory/TrapezoidProfile.inc
  fn calculate(&mut self, input: ProfileState<Vel, Time>, time: Time) -> Self::Output {
    let start_time = match self.start_time {
      None => {
        self.start_time = Some(time);
        time
      }
      Some(t) => t,
    };

    let t = time - start_time;

    let flip = input.displacement > self.setpoint.displacement;

    let mut cur = maybe_flip(flip, input);
    let goal = maybe_flip(flip, self.setpoint);

    if cur.velocity > self.max_velocity {
      cur.velocity = self.max_velocity;
    }

    // // Truncated - triangle profile
    let cutoff_starts = cur.velocity / self.max_acceleration;
    let cutoff_starts_distance = 0.5 * self.max_acceleration * cutoff_starts * cutoff_starts;

    let cutoff_ends = goal.velocity / self.max_acceleration;
    let cutoff_ends_distance = 0.5 * self.max_acceleration * cutoff_ends * cutoff_ends;

    let full_trap_dist =
      cutoff_starts_distance + (goal.displacement - cur.displacement) + cutoff_ends_distance;
    let mut acceleration_time = self.max_velocity / self.max_acceleration;

    let full_speed_dist =
      full_trap_dist - acceleration_time * acceleration_time * self.max_acceleration;

    if full_speed_dist < Zero::zero() {
      acceleration_time =
        Time::from_f64((full_trap_dist.to_f64() / self.max_acceleration.to_f64()).sqrt());
    }

    let end_accel = acceleration_time - cutoff_starts;
    let end_full_speed = end_accel + full_speed_dist / self.max_velocity;
    let end_decel = end_full_speed + acceleration_time - cutoff_ends;

    if t < end_accel {
      maybe_flip(
        flip,
        ProfileState {
          displacement: cur.displacement + (cur.velocity + 0.5 * self.max_acceleration * t) * t,
          velocity: cur.velocity + self.max_acceleration * t,
          acceleration: self.max_acceleration,
        },
      )
    } else if t < end_full_speed {
      maybe_flip(
        flip,
        ProfileState {
          displacement: cur.displacement
            + (cur.velocity + 0.5 * self.max_acceleration * end_accel) * end_accel
            + self.max_velocity * (t - end_accel),
          velocity: self.max_velocity,
          acceleration: Zero::zero(),
        },
      )
    } else if t <= end_decel {
      let remaining_time = end_decel - t;
      maybe_flip(
        flip,
        ProfileState {
          displacement: goal.displacement
            - (goal.velocity + 0.5 * self.max_acceleration * remaining_time) * remaining_time,
          velocity: goal.velocity + self.max_acceleration * remaining_time,
          acceleration: -self.max_acceleration,
        },
      )
    } else {
      self.setpoint
    }
  }

  fn reset(&mut self) {
    self.start_time = None;
  }
}

pub struct ProfileFeedForward<
  Vel: Div<Time> + Mul<Time>,
  Output: Div<Vel> + Div<Acceleration<Vel, Time>>,
  Time,
> {
  kv: KV<Output, Vel>,
  ka: KA<Output, Vel, Time>,
}

impl<Vel: Div<Time> + Mul<Time>, Output: Div<Vel> + Div<Acceleration<Vel, Time>>, Time>
  ProfileFeedForward<Vel, Output, Time>
{
  pub fn new(kv: KV<Output, Vel>, ka: KA<Output, Vel, Time>) -> Self {
    Self { kv, ka }
  }
}

impl ProfileFeedForward<Velocity, Voltage, Time> {
  pub fn from_motor_linear<M: SpooledMotorInverseDynamics>(
    model: M,
    v_nom: Voltage,
    mass: Mass,
  ) -> Self {
    Self {
      kv: model.estimate_profile_kv(v_nom),
      ka: model.estimate_profile_ka(mass, v_nom),
    }
  }
}

impl ProfileFeedForward<AngularVelocity, Voltage, Time> {
  pub fn from_motor_angular<M: MotorInverseDynamics>(model: M, v_nom: Voltage, moi: MOI) -> Self {
    Self {
      kv: model.estimate_profile_kv(v_nom),
      ka: model.estimate_profile_ka(moi, v_nom),
    }
  }
}

impl<Vel: Div<Time> + Mul<Time>, Output: Div<Vel> + Div<Acceleration<Vel, Time>>, Time>
  Transform<ProfileState<Vel, Time>> for ProfileFeedForward<Vel, Output, Time>
where
  Vel: Copy,
  Displacement<Vel, Time>: Copy,
  Acceleration<Vel, Time>: Copy,
  Output: MaybeUnitNumber,
  KV<Output, Vel>: Mul<Vel, Output = Output> + Copy,
  KA<Output, Vel, Time>: Mul<Acceleration<Vel, Time>, Output = Output> + Copy,
{
  type Output = Output;

  fn calculate(&self, input: ProfileState<Vel, Time>) -> Self::Output {
    self.kv * input.velocity + self.ka * input.acceleration
  }
}

pub struct Profiled1stOrderController<Vel, Output, Time, Profile, Controller, Feedforward>
where
  Profile: StatefulTransform<ProfileState<Vel, Time>, Time, Output = ProfileState<Vel, Time>>,
  Controller: StatefulTransform<Displacement<Vel, Time>, Time, Output = Output>,
  Feedforward: StatefulTransform<ProfileState<Vel, Time>, Time, Output = Output>,
  Vel: Div<Time> + Mul<Time> + Copy,
  Displacement<Vel, Time>: Copy,
  Acceleration<Vel, Time>: Copy,
{
  profile: Profile,
  controller: Controller,
  feedforward: Feedforward,
  last_state: Option<ProfileState<Vel, Time>>,
  phantom: PhantomData<Output>,
}

impl<Vel, Output, Time, Profile, Controller, Feedforward>
  Profiled1stOrderController<Vel, Output, Time, Profile, Controller, Feedforward>
where
  Profile: StatefulTransform<ProfileState<Vel, Time>, Time, Output = ProfileState<Vel, Time>>,
  Controller: StatefulTransform<Displacement<Vel, Time>, Time, Output = Output>,
  Feedforward: StatefulTransform<ProfileState<Vel, Time>, Time, Output = Output>,
  Vel: Div<Time> + Mul<Time> + Copy,
  Displacement<Vel, Time>: Copy,
  Acceleration<Vel, Time>: Copy,
{
  pub fn new(profile: Profile, controller: Controller, feedforward: Feedforward) -> Self {
    Self {
      profile,
      controller,
      feedforward,
      last_state: None,
      phantom: PhantomData,
    }
  }
}

impl<Vel, Output, Time, Profile, Controller, Feedforward> HasSetpoint<Displacement<Vel, Time>>
  for Profiled1stOrderController<Vel, Output, Time, Profile, Controller, Feedforward>
where
  Profile: StatefulTransform<ProfileState<Vel, Time>, Time, Output = ProfileState<Vel, Time>>
    + HasSetpoint<ProfileState<Vel, Time>>,
  Controller: StatefulTransform<Displacement<Vel, Time>, Time, Output = Output>,
  Feedforward: StatefulTransform<ProfileState<Vel, Time>, Time, Output = Output>,
  Vel: Div<Time> + Mul<Time> + Copy,
  Displacement<Vel, Time>: Copy,
  Acceleration<Vel, Time>: Copy,
  Time: Zero,
  Vel: Zero,
  Acceleration<Vel, Time>: Zero,
{
  fn set_setpoint(&mut self, sp: Displacement<Vel, Time>) {
    self.profile.set_setpoint(ProfileState {
      displacement: sp,
      velocity: Zero::zero(),
      acceleration: Zero::zero(),
    })
  }
}

impl<Vel, Output, Time, Profile, Controller, Feedforward>
  StatefulTransform<Displacement<Vel, Time>, Time>
  for Profiled1stOrderController<Vel, Output, Time, Profile, Controller, Feedforward>
where
  Profile: StatefulTransform<ProfileState<Vel, Time>, Time, Output = ProfileState<Vel, Time>>
    + HasSetpoint<ProfileState<Vel, Time>>,
  Controller: StatefulTransform<Displacement<Vel, Time>, Time, Output = Output>
    + HasSetpoint<Displacement<Vel, Time>>,
  Feedforward: StatefulTransform<ProfileState<Vel, Time>, Time, Output = Output>,
  Vel: Div<Time> + Mul<Time> + Copy + Zero,
  Displacement<Vel, Time>: Copy + Zero,
  Acceleration<Vel, Time>: Copy + Zero,
  Output: Zero + Add<Output, Output = Output>,
  Time: Copy,
  ProfileState<Vel, Time>: Clone,
{
  type Output = Output;

  fn calculate(&mut self, input: Displacement<Vel, Time>, time: Time) -> Self::Output {
    match self.last_state {
      Some(last) => {
        let next_state = self.profile.calculate(last, time);
        self.controller.set_setpoint(next_state.displacement);
        self.last_state = Some(next_state);
        self.controller.calculate(input, time) + self.feedforward.calculate(next_state, time)
      }
      None => {
        self.last_state = Some(ProfileState {
          displacement: input,
          velocity: Zero::zero(),
          acceleration: Zero::zero(),
        });
        Zero::zero()
      }
    }
  }

  fn reset(&mut self) {
    self.profile.reset();
    self.controller.reset();
    self.feedforward.reset();
    self.last_state = None;
  }
}
