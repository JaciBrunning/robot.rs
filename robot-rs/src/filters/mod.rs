use std::{marker::PhantomData, ops::Neg};

pub mod binary;
pub mod diff;
pub mod feedforward;
pub mod linear;
pub mod pid;
pub mod predictive;

pub trait Filter<I> {
  type Output;

  fn calculate(&mut self, input: I) -> Self::Output;
  fn reset(&mut self) { }
}

pub trait HasSetpoint<SP> {
  fn set_setpoint(&mut self, sp: SP);
}

pub struct IdentityFilter<T> {
  phantom: PhantomData<T>
}

impl<T> IdentityFilter<T> {
  pub fn new() -> Self { Self { phantom: PhantomData } }
}

impl<T> Filter<T> for IdentityFilter<T> {
  type Output = T;
  fn calculate(&mut self, input: T) -> T {
    input
  }
}

pub struct InvertingFilter<T> {
  phantom: PhantomData<T>
}

impl<T> InvertingFilter<T> {
  pub fn new() -> Self { Self { phantom: PhantomData } }
}

impl<T: Neg> Filter<T> for InvertingFilter<T> {
  type Output = <T as Neg>::Output;

  fn calculate(&mut self, input: T) -> Self::Output {
    -input
  }
}

pub struct ClampingFilter<T> {
  pub min: T,
  pub max: T
}

impl<T> ClampingFilter<T> {
  pub fn new(min: T, max: T) -> Self { Self { min, max } }
}

impl<T: PartialOrd<T> + Copy> Filter<T> for ClampingFilter<T> {
  type Output = T;

  fn calculate(&mut self, input: T) -> Self::Output {
    match input {
      input if input < self.min => self.min,
      input if input > self.max => self.max,
      input => input
    }
  }
}

pub struct ChainedFiltersA<A, B, I> {
  pub a: A,
  pub b: B,
  input_type: PhantomData<I>
}

impl<A, B, I> ChainedFiltersA<A, B, I> {
  pub fn new(a: A, b: B) -> Self {
    Self { a, b, input_type: PhantomData }
  }
}

impl<A, B, I> Filter<I> for ChainedFiltersA<A, B, I>
where
  A: Filter<I>,
  B: Filter<<A as Filter<I>>::Output>
{
  type Output = B::Output;

  fn calculate(&mut self, input: I) -> Self::Output {
    self.b.calculate(self.a.calculate(input))
  }

  fn reset(&mut self) {
    self.a.reset();
    self.b.reset();
  }
}

impl<A, B, I> HasSetpoint<I> for ChainedFiltersA<A, B, I>
where
  A: HasSetpoint<I>
{
  fn set_setpoint(&mut self, sp: I) {
    self.a.set_setpoint(sp)
  }
}

pub struct ChainedFiltersB<A, B, I> {
  pub a: A,
  pub b: B,
  input_type: PhantomData<I>
}

impl<A, B, I> ChainedFiltersB<A, B, I> {
  pub fn new(a: A, b: B) -> Self {
    Self { a, b, input_type: PhantomData }
  }
}

impl<A, B, I> Filter<I> for ChainedFiltersB<A, B, I>
where
  A: Filter<I>,
  B: Filter<<A as Filter<I>>::Output>
{
  type Output = B::Output;

  fn calculate(&mut self, input: I) -> Self::Output {
    self.b.calculate(self.a.calculate(input))
  }

  fn reset(&mut self) {
    self.a.reset();
    self.b.reset();
  }
}

impl<A, B, I> HasSetpoint<<A as Filter<I>>::Output> for ChainedFiltersB<A, B, I>
where
  A: Filter<I>,
  B: HasSetpoint<<A as Filter<I>>::Output>
{
  fn set_setpoint(&mut self, sp: <A as Filter<I>>::Output) {
    self.b.set_setpoint(sp)
  }
}
