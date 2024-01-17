use std::{marker::PhantomData, ops::Neg};

pub mod binary;
pub mod diff;
pub mod offset;
pub mod linear;
pub mod pid;
pub mod predictive;
pub mod transform;

pub trait Filter<I> {
  type Output;

  fn calculate(&self, input: I) -> Self::Output;
}

pub trait ReversibleFilter<I> : Filter<I> {
  fn calculate_reverse(&self, output: <Self as Filter<I>>::Output) -> I;
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

impl<T> Filter<T> for IdentityFilter<T> {
  type Output = T;
  fn calculate(&self, input: T) -> T {
    input
  }
}

impl<T> ReversibleFilter<T> for IdentityFilter<T> {
  fn calculate_reverse(&self, output: <Self as Filter<T>>::Output) -> T {
    output
  }
}

pub struct StatefulFilterAdapter<T: Filter<I>, I> {
  pub filter: T,
  phantom: PhantomData<I>
}

impl<T: Filter<I>, I> StatefulFilterAdapter<T, I> {
  pub fn new(filter: T) -> Self { Self { filter, phantom: PhantomData } }
}

impl<T: Filter<I>, I, Time> StatefulFilter<I, Time> for StatefulFilterAdapter<T, I> {
  type Output = T::Output;

  fn calculate(&mut self, input: I, time: Time) -> Self::Output {
    self.filter.calculate(input)
  }
}

#[derive(Clone, Debug)]
pub struct InvertingFilter<T> {
  phantom: PhantomData<T>
}

impl<T> InvertingFilter<T> {
  pub fn new() -> Self { Self { phantom: PhantomData } }
}

impl<T: Neg> Filter<T> for InvertingFilter<T> {
  type Output = <T as Neg>::Output;

  fn calculate(&self, input: T) -> Self::Output {
    -input
  }
}

impl<T: Neg> ReversibleFilter<T> for InvertingFilter<T> 
  where <T as Neg>::Output: Neg<Output = T>
{
  fn calculate_reverse(&self, output: <Self as Filter<T>>::Output) -> T {
    -output
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

impl<T: PartialOrd<T> + Copy> Filter<T> for ClampingFilter<T> {
  type Output = T;

  fn calculate(&self, input: T) -> Self::Output {
    match input {
      input if input < self.min => self.min,
      input if input > self.max => self.max,
      input => input
    }
  }
}

pub struct ChainedFilters<A, B> {
  pub a: A, 
  pub b: B
}

impl<A, B> ChainedFilters<A, B> {
  pub fn new(a: A, b: B) -> Self {
    Self { a, b }
  }
}

impl<A, B, I> Filter<I> for ChainedFilters<A, B>
where
  A: Filter<I>,
  B: Filter<<A as Filter<I>>::Output>
{
  type Output = B::Output;

  fn calculate(&self, input: I) -> Self::Output {
    self.b.calculate(self.a.calculate(input))
  }
}

pub struct ChainedStatefulFiltersA<A, B, Time> {
  pub a: A,
  pub b: B,
  phantom: PhantomData<Time>
}

impl<A, B, Time> ChainedStatefulFiltersA<A, B, Time> {
  pub fn new(a: A, b: B) -> Self {
    Self { a, b, phantom: PhantomData }
  }
}

impl<A, B, I, Time: Copy> StatefulFilter<I, Time> for ChainedStatefulFiltersA<A, B, Time>
where
  A: StatefulFilter<I, Time>,
  B: StatefulFilter<<A as StatefulFilter<I, Time>>::Output, Time>
{
  type Output = B::Output;

  fn calculate(&mut self, input: I, time: Time) -> Self::Output {
    self.b.calculate(self.a.calculate(input, time), time)
  }

  fn reset(&mut self) {
    self.a.reset();
    self.b.reset();
  }
}

impl<A, B, I, Time> HasSetpoint<I> for ChainedStatefulFiltersA<A, B, Time>
where
  A: HasSetpoint<I>
{
  fn set_setpoint(&mut self, sp: I) {
    self.a.set_setpoint(sp)
  }
}

pub struct ChainedStatefulFiltersB<A, B, I, Time> {
  pub a: A,
  pub b: B,
  input_type: PhantomData<(I, Time)>
}

impl<A, B, I, Time> ChainedStatefulFiltersB<A, B, I, Time> {
  pub fn new(a: A, b: B) -> Self {
    Self { a, b, input_type: PhantomData }
  }
}

impl<A, B, I, Time: Copy> StatefulFilter<I, Time> for ChainedStatefulFiltersB<A, B, I, Time>
where
  A: StatefulFilter<I, Time>,
  B: StatefulFilter<<A as StatefulFilter<I, Time>>::Output, Time>
{
  type Output = B::Output;

  fn calculate(&mut self, input: I, time: Time) -> Self::Output {
    self.b.calculate(self.a.calculate(input, time), time)
  }

  fn reset(&mut self) {
    self.a.reset();
    self.b.reset();
  }
}

impl<A, B, I, Time> HasSetpoint<<A as StatefulFilter<I, Time>>::Output> for ChainedStatefulFiltersB<A, B, I, Time>
where
  A: StatefulFilter<I, Time>,
  B: HasSetpoint<<A as StatefulFilter<I, Time>>::Output>
{
  fn set_setpoint(&mut self, sp: <A as StatefulFilter<I, Time>>::Output) {
    self.b.set_setpoint(sp)
  }
}

pub trait FilterExt<I> : Filter<I> + Sized {
  fn to_stateful(self) -> StatefulFilterAdapter<Self, I>;
  fn then<Other: Filter<Self::Output>>(self, other: Other) -> ChainedFilters<Self, Other>;
}

impl<T: Filter<I>, I> FilterExt<I> for T {
  fn to_stateful(self) -> StatefulFilterAdapter<Self, I> {
    StatefulFilterAdapter::new(self)
  }

  fn then<Other: Filter<Self::Output>>(self, other: Other) -> ChainedFilters<Self, Other> {
    ChainedFilters::new(self, other)
  }
}