use std::ops::{Div, Mul, Neg};

use num_traits::Zero;
use robot_rs_units::traits::MaybeUnitNumber;

use super::{StatefulTransform, HasSetpoint, Transform, pid::{PID, Ki, Kd, Derivative, Integral, Kp}};

pub type Displacement<Vel, Time> = <Vel as Mul<Time>>::Output;
pub type Acceleration<Vel, Time> = <Vel as Div<Time>>::Output;

pub type KV<Output, Vel> = <Output as Div<Vel>>::Output;
pub type KA<Output, Vel, Time> = <Output as Div<Acceleration<Vel, Time>>>::Output;

#[derive(Clone, Copy)]
pub struct ProfileState<Vel: Div<Time> + Mul<Time>, Time>
where
  Vel: Copy,
  Displacement<Vel, Time>: Copy,
  Acceleration<Vel, Time>: Copy,
{
  pub time: Time,   // Ignored in setpoint
  pub displacement: Displacement<Vel, Time>,
  pub velocity: Vel,
  pub acceleration: Acceleration<Vel, Time>,    // Ignored in setpoint (for trapezoidal profiles)
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
}

impl<Vel: Div<Time> + Mul<Time>, Time> TrapezoidalProfile<Vel, Time>
where
  Vel: Copy,
  Displacement<Vel, Time>: Copy,
  Acceleration<Vel, Time>: Copy,
{
  pub fn new(max_velocity: Vel, max_acceleration: Acceleration<Vel, Time>, setpoint: ProfileState<Vel, Time>) -> Self {
    Self { max_velocity, max_acceleration, setpoint }
  }
}

impl<Vel: Div<Time> + Mul<Time>, Time> HasSetpoint<ProfileState<Vel, Time>> for TrapezoidalProfile<Vel, Time>
where
  Vel: Copy,
  Displacement<Vel, Time>: Copy,
  Acceleration<Vel, Time>: Copy,
{
  fn set_setpoint(&mut self, sp: ProfileState<Vel, Time>) {
    self.setpoint = sp;
  }
}

impl<Vel, Time> StatefulTransform<ProfileState<Vel, Time>, Time> for TrapezoidalProfile<Vel, Time>
where
  Vel: MaybeUnitNumber + Div<Time> + Mul<Time> + Mul<Vel> + Copy,
  Time: MaybeUnitNumber + Copy,
  <Vel as Mul<Vel>>::Output: MaybeUnitNumber + Neg<Output = <Vel as Mul<Vel>>::Output> + Div<Acceleration<Vel, Time>, Output = Displacement<Vel, Time>> + Copy,
  Displacement<Vel, Time>: MaybeUnitNumber + Copy,
  Acceleration<Vel, Time>: MaybeUnitNumber + Neg<Output = <Vel as Div<Time>>::Output> + Mul<Time, Output = Vel> + Copy,
  f64: Mul<Acceleration<Vel, Time>, Output = Acceleration<Vel, Time>> + Mul<Vel, Output = Vel>
{
  type Output = ProfileState<Vel, Time>;

  fn calculate(&mut self, input: ProfileState<Vel, Time>, time: Time) -> Self::Output {
    let dt = time - input.time;
    let current_displacement = input.displacement;
    let current_velocity = input.velocity;

    let displacement_if_decelerate_now = ((self.setpoint.velocity * self.setpoint.velocity) - (current_velocity * current_velocity)) / (2.0 * -self.max_acceleration);

    if displacement_if_decelerate_now + current_displacement >= self.setpoint.displacement {
      let at = -self.max_acceleration * dt;
      let new_vel = current_velocity + at;
      ProfileState {
        time,
        velocity: new_vel,
        displacement: current_displacement + (current_velocity * dt) + 2.0 * at * dt,
        acceleration: -self.max_acceleration
      }
    } else {
      let mut at = self.max_acceleration * dt;
      let mut new_vel = current_velocity + at;

      if new_vel > self.max_velocity {
        new_vel = self.max_velocity;
        at = new_vel - input.velocity;
      }

      ProfileState {
        time, 
        velocity: new_vel,
        displacement: current_displacement + (current_velocity * dt) + 2.0 * at * dt,
        acceleration: if dt > Zero::zero() { at / dt } else { Zero::zero() }
      }
    }
  }

  fn reset(&mut self) { }
}

pub struct ProfileFeedForward<
  Vel: Div<Time> + Mul<Time>,
  Output: Div<Vel> + Div<Acceleration<Vel, Time>>,
  Time
> {
  kv: KV<Output, Vel>,
  ka: KA<Output, Vel, Time>
}

impl<
  Vel: Div<Time> + Mul<Time>,
  Output: Div<Vel> + Div<Acceleration<Vel, Time>>,
  Time
> ProfileFeedForward<Vel, Output, Time> {
  pub fn new(kv: KV<Output, Vel>, ka: KA<Output, Vel, Time>) -> Self {
    Self { kv, ka }
  }
}

impl<
  Vel: Div<Time> + Mul<Time>,
  Output: Div<Vel> + Div<Acceleration<Vel, Time>>,
  Time
> Transform<ProfileState<Vel, Time>> for ProfileFeedForward<Vel, Output, Time>
where
  Vel: Copy,
  Displacement<Vel, Time>: Copy,
  Acceleration<Vel, Time>: Copy,
  Output: MaybeUnitNumber,
  KV<Output, Vel>: Mul<Vel, Output=Output> + Copy,
  KA<Output, Vel, Time>: Mul<Acceleration<Vel, Time>, Output=Output> + Copy,
{
  type Output = Output;

  fn calculate(&self, input: ProfileState<Vel, Time>) -> Self::Output {
    self.kv * input.velocity + self.ka * input.acceleration
  }
}

// pub struct ProfiledPID<Vel, Output, Time, Profile>
// where
//   Vel: Div<Time> + Mul<Time> + Copy,
//   Output: Div<Vel> + Div<Displacement<Vel, Time>> + Div<<Displacement<Vel, Time> as Mul<Time>>::Output> + Div<Acceleration<Vel, Time>> + Copy,
//   Displacement<Vel, Time>: Mul<Time> + Div<Time, Output = Vel> + Copy,
//   <Displacement<Vel, Time> as Mul<Time>>::Output: Copy,
//   Acceleration<Vel, Time>: Copy,
//   Profile: StatefulTransform<ProfileState<Vel, Time>, Time, Output = ProfileState<Vel, Time>>
// {
//   pid: PID<Displacement<Vel, Time>, Output, Time>,
//   profile: Profile,
//   ff: ProfileFeedForward<Vel, Output, Time>,
// }

// impl<Vel, Output, Time, Profile> ProfiledPID<Vel, Output, Time, Profile>
// where
//   Vel: Div<Time> + Mul<Time> + Copy,
//   Output: Div<Vel> + Div<Displacement<Vel, Time>> + Div<<Displacement<Vel, Time> as Mul<Time>>::Output> + Div<Acceleration<Vel, Time>> + Copy,
//   Displacement<Vel, Time>: Mul<Time> + Div<Time, Output = Vel> + Copy,
//   <Displacement<Vel, Time> as Mul<Time>>::Output: Copy,
//   Acceleration<Vel, Time>: Copy,
//   Profile: StatefulTransform<ProfileState<Vel, Time>, Time, Output = ProfileState<Vel, Time>>
// {
//   pub fn new(pid: PID<Displacement<Vel, Time>, Output, Time>, profile: Profile, ff: ProfileFeedForward<Vel, Output, Time>) -> Self {
//     Self { pid, ff, profile }
//   }
// }

// impl<Vel, Output, Time, Profile> HasSetpoint<ProfileState<Vel, Time>> for ProfiledPID<Vel, Output, Time, Profile>
// where
//   Vel: Div<Time> + Mul<Time> + Copy,
//   Output: Div<Vel> + Div<Displacement<Vel, Time>> + Div<<Displacement<Vel, Time> as Mul<Time>>::Output> + Div<Acceleration<Vel, Time>> + Copy,
//   Displacement<Vel, Time>: Mul<Time> + Div<Time, Output = Vel> + Copy,
//   <Displacement<Vel, Time> as Mul<Time>>::Output: Copy,
//   Acceleration<Vel, Time>: Copy,
//   Profile: StatefulTransform<ProfileState<Vel, Time>, Time, Output = ProfileState<Vel, Time>> + HasSetpoint<ProfileState<Vel, Time>>
// {
//   fn set_setpoint(&mut self, sp: ProfileState<Vel, Time>) {
//     self.profile.set_setpoint(sp);
//   }
// }

// impl<Vel, Output, Time, Profile> StatefulTransform<ProfileState<Vel, Time>, Time> for ProfiledPID<Vel, Output, Time, Profile>
// where
//   Vel: Div<Time> + Mul<Time> + Copy,
//   Time: MaybeUnitNumber + Copy,
//   Output: MaybeUnitNumber + Div<Vel> + Div<Displacement<Vel, Time>> + Div<<Displacement<Vel, Time> as Mul<Time>>::Output> + Div<Acceleration<Vel, Time>> + Copy,
//   Displacement<Vel, Time>: MaybeUnitNumber + Mul<Time> + Div<Time, Output = Vel> + Mul<Kp<Displacement<Vel, Time>, Output>, Output = Output> + Copy,
//   <Displacement<Vel, Time> as Mul<Time>>::Output: Copy,
//   Acceleration<Vel, Time>: Copy,
//   Profile: StatefulTransform<ProfileState<Vel, Time>, Time, Output = ProfileState<Vel, Time>>,
//   KV<Output, Vel>: Mul<Vel, Output=Output> + Copy,
//   KA<Output, Vel, Time>: Mul<Acceleration<Vel, Time>, Output=Output> + Copy,
//   Integral<Displacement<Vel, Time>, Time>: Copy + Zero + Mul<Ki<Displacement<Vel, Time>, Output, Time>, Output = Output>,
//   Derivative<Displacement<Vel, Time>, Time>: Copy + Zero + Mul<Kd<Displacement<Vel, Time>, Output, Time>, Output = Output>,
//   Kp<Displacement<Vel, Time>, Output>: Zero + Copy,
//   Ki<Displacement<Vel, Time>, Output, Time>: Zero + Copy,
//   Kd<Displacement<Vel, Time>, Output, Time>: Zero + Copy,
// {
//   type Output = Output;

//   fn calculate(&mut self, input: ProfileState<Vel, Time>, time: Time) -> Self::Output {
//     let new_state = self.profile.calculate(input, time);
//     self.pid.set_setpoint(new_state.displacement);
//     let ff = self.ff.calculate(input);
//     self.pid.calculate(input.displacement, time) + ff
//   }

//   fn reset(&mut self) {
//     self.pid.reset();
//   }
// }
