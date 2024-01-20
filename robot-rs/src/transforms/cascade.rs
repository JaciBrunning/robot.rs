use std::marker::PhantomData;

use super::{HasSetpoint, StatefulTransform};

pub struct CascadeTransform<A, B, Time> {
  pub a: A,
  pub b: B,
  time: PhantomData<Time>
}

impl<A, B, Time> CascadeTransform<A, B, Time> {
  pub fn new(a: A, b: B) -> Self {
    Self { a, b, time: PhantomData }
  }
}

impl<A: HasSetpoint<SP>, B, SP, Time> HasSetpoint<SP> for CascadeTransform<A, B, Time> {
  fn set_setpoint(&mut self, sp: SP) {
    self.a.set_setpoint(sp)
  }
}

impl<A, B, I, J, Time> StatefulTransform<(I, J), Time> for CascadeTransform<A, B, Time>
where
  A: StatefulTransform<I, Time>,
  B: HasSetpoint<A::Output> + StatefulTransform<J, Time>,
  Time: Copy
{
  type Output = <B as StatefulTransform<J, Time>>::Output;

  fn calculate(&mut self, input: (I, J), time: Time) -> Self::Output {
    let new_sp = self.a.calculate(input.0, time);
    self.b.set_setpoint(new_sp);
    self.b.calculate(input.1, time)
  }

  fn reset(&mut self) {
    self.a.reset();
    self.b.reset();
  }
}