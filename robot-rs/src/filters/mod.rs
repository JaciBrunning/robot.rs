use std::{marker::PhantomData, ops::Neg};

pub mod binary;
pub mod diff;
pub mod feedforward;
pub mod linear;
pub mod pid;
pub mod predictive;

pub trait Filter<I, Time> {
  type Output;

  fn calculate(&self, input: I, time: Time) -> Self::Output;
}

pub trait StatefulFilter<I, Time> {
  type Output;

  fn calculate(&mut self, input: I, time: Time) -> Self::Output;
  fn reset(&mut self) { }
}

pub trait HasSetpoint<SP> {
  fn set_setpoint(&mut self, sp: SP);
}

#[derive(Clone, Debug)]
pub struct IdentityFilter<T> {
  phantom: PhantomData<T>
}

impl<T> IdentityFilter<T> {
  pub fn new() -> Self { Self { phantom: PhantomData } }
}

impl<T, Time> Filter<T, Time> for IdentityFilter<T> {
  type Output = T;
  fn calculate(&self, input: T, _time: Time) -> T {
    input
  }
}

pub struct StatefulFilterAdapter<T: Filter<I, Time>, I, Time> {
  pub filter: T,
  phantom: PhantomData<(I, Time)>
}

impl<T: Filter<I, Time>, I, Time> StatefulFilterAdapter<T, I, Time> {
  pub fn new(filter: T) -> Self { Self { filter, phantom: PhantomData } }
}

impl<T: Filter<I, Time>, I, Time> StatefulFilter<I, Time> for StatefulFilterAdapter<T, I, Time> {
  type Output = T::Output;

  fn calculate(&mut self, input: I, time: Time) -> Self::Output {
    self.filter.calculate(input, time)
  }
}

#[derive(Clone, Debug)]
pub struct InvertingFilter<T> {
  phantom: PhantomData<T>
}

impl<T> InvertingFilter<T> {
  pub fn new() -> Self { Self { phantom: PhantomData } }
}

impl<T: Neg, Time> Filter<T, Time> for InvertingFilter<T> {
  type Output = <T as Neg>::Output;

  fn calculate(&self, input: T, _time: Time) -> Self::Output {
    -input
  }
}

#[derive(Clone, Debug)]
pub struct ClampingFilter<T> {
  pub min: T,
  pub max: T
}

impl<T> ClampingFilter<T> {
  pub fn new(min: T, max: T) -> Self { Self { min, max } }
}

impl<T: PartialOrd<T> + Copy, Time> Filter<T, Time> for ClampingFilter<T> {
  type Output = T;

  fn calculate(&self, input: T, _time: Time) -> Self::Output {
    match input {
      input if input < self.min => self.min,
      input if input > self.max => self.max,
      input => input
    }
  }
}

// pub struct ChainedFiltersA<A, B, Time> {
//   pub a: A,
//   pub b: B,
//   phantom: PhantomData<Time>
// }

// impl<A, B, Time> ChainedFiltersA<A, B, Time> {
//   pub fn new(a: A, b: B) -> Self {
//     Self { a, b, phantom: PhantomData }
//   }
// }

// impl<A, B, I, Time: Copy> Filter<I, Time> for ChainedFiltersA<A, B, Time>
// where
//   A: Filter<I, Time>,
//   B: Filter<<A as Filter<I, Time>>::Output, Time>
// {
//   type Output = B::Output;

//   fn calculate(&mut self, input: I, time: Time) -> Self::Output {
//     self.b.calculate(self.a.calculate(input, time), time)
//   }

//   fn reset(&mut self) {
//     self.a.reset();
//     self.b.reset();
//   }
// }

// impl<A, B, I, Time> HasSetpoint<I> for ChainedFiltersA<A, B, Time>
// where
//   A: HasSetpoint<I>
// {
//   fn set_setpoint(&mut self, sp: I) {
//     self.a.set_setpoint(sp)
//   }
// }

// pub struct ChainedFiltersB<A, B, I, Time> {
//   pub a: A,
//   pub b: B,
//   input_type: PhantomData<(I, Time)>
// }

// impl<A, B, I, Time> ChainedFiltersB<A, B, I, Time> {
//   pub fn new(a: A, b: B) -> Self {
//     Self { a, b, input_type: PhantomData }
//   }
// }

// impl<A, B, I, Time: Copy> Filter<I, Time> for ChainedFiltersB<A, B, I, Time>
// where
//   A: Filter<I, Time>,
//   B: Filter<<A as Filter<I, Time>>::Output, Time>
// {
//   type Output = B::Output;

//   fn calculate(&mut self, input: I, time: Time) -> Self::Output {
//     self.b.calculate(self.a.calculate(input, time), time)
//   }

//   fn reset(&mut self) {
//     self.a.reset();
//     self.b.reset();
//   }
// }

// impl<A, B, I, Time> HasSetpoint<<A as Filter<I, Time>>::Output> for ChainedFiltersB<A, B, I, Time>
// where
//   A: Filter<I, Time>,
//   B: HasSetpoint<<A as Filter<I, Time>>::Output>
// {
//   fn set_setpoint(&mut self, sp: <A as Filter<I, Time>>::Output) {
//     self.b.set_setpoint(sp)
//   }
// }

pub trait FilterExt<I, Time> : Filter<I, Time> + Sized {
  fn to_stateful(self) -> StatefulFilterAdapter<Self, I, Time>;
}

impl<T: Filter<I, Time>, I, Time> FilterExt<I, Time> for T {
  fn to_stateful(self) -> StatefulFilterAdapter<Self, I, Time> {
    StatefulFilterAdapter::new(self)
  }
}