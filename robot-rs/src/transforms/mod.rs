use std::{marker::PhantomData, ops::Neg};

use self::{
  cascade::CascadeTransform,
  map::{MapStatefulTransform, MapTransform},
};

pub mod binary;
pub mod cascade;
pub mod diff;
pub mod linear;
pub mod map;
pub mod offset;
pub mod pid;
pub mod predictive;
pub mod profile;
pub mod stability;
pub mod transform;

pub trait Transform<I> {
  type Output;

  fn calculate(&self, input: I) -> Self::Output;
}

pub trait ReversibleTransform<I>: Transform<I> {
  fn calculate_reverse(&self, output: <Self as Transform<I>>::Output) -> I;
}

pub trait StatefulTransform<I, Time> {
  type Output;

  fn calculate(&mut self, input: I, time: Time) -> Self::Output;
  fn reset(&mut self) {}
}

pub trait HasSetpoint<SP> {
  fn set_setpoint(&mut self, sp: SP);
}

#[derive(Clone, Debug, Default)]
pub struct IdentityTransform<T> {
  phantom: PhantomData<T>,
}

impl<T> IdentityTransform<T> {
  pub fn new() -> Self {
    Self {
      phantom: PhantomData,
    }
  }
}

impl<T> Transform<T> for IdentityTransform<T> {
  type Output = T;
  fn calculate(&self, input: T) -> T {
    input
  }
}

impl<T> ReversibleTransform<T> for IdentityTransform<T> {
  fn calculate_reverse(&self, output: <Self as Transform<T>>::Output) -> T {
    output
  }
}

pub struct StatefulTransformAdapter<T: Transform<I>, I> {
  pub transform: T,
  phantom: PhantomData<I>,
}

impl<T: Transform<I>, I> StatefulTransformAdapter<T, I> {
  pub fn new(transform: T) -> Self {
    Self {
      transform,
      phantom: PhantomData,
    }
  }
}

impl<T: Transform<I>, I, Time> StatefulTransform<I, Time> for StatefulTransformAdapter<T, I> {
  type Output = T::Output;

  fn calculate(&mut self, input: I, _time: Time) -> Self::Output {
    self.transform.calculate(input)
  }
}

#[derive(Clone, Debug, Default)]
pub struct InvertingTransform<T> {
  phantom: PhantomData<T>,
}

impl<T> InvertingTransform<T> {
  pub fn new() -> Self {
    Self {
      phantom: PhantomData,
    }
  }
}

impl<T: Neg> Transform<T> for InvertingTransform<T> {
  type Output = <T as Neg>::Output;

  fn calculate(&self, input: T) -> Self::Output {
    -input
  }
}

impl<T: Neg> ReversibleTransform<T> for InvertingTransform<T>
where
  <T as Neg>::Output: Neg<Output = T>,
{
  fn calculate_reverse(&self, output: <Self as Transform<T>>::Output) -> T {
    -output
  }
}

#[derive(Clone, Debug)]
pub struct ClampingTransform<T> {
  pub min: T,
  pub max: T,
}

impl<T> ClampingTransform<T> {
  pub fn new(min: T, max: T) -> Self {
    Self { min, max }
  }
}

impl<T: PartialOrd<T> + Copy> Transform<T> for ClampingTransform<T> {
  type Output = T;

  fn calculate(&self, input: T) -> Self::Output {
    match input {
      input if input < self.min => self.min,
      input if input > self.max => self.max,
      input => input,
    }
  }
}

pub struct ChainedTransforms<A, B> {
  pub a: A,
  pub b: B,
}

impl<A, B> ChainedTransforms<A, B> {
  pub fn new(a: A, b: B) -> Self {
    Self { a, b }
  }
}

impl<A, B, I> Transform<I> for ChainedTransforms<A, B>
where
  A: Transform<I>,
  B: Transform<<A as Transform<I>>::Output>,
{
  type Output = B::Output;

  fn calculate(&self, input: I) -> Self::Output {
    self.b.calculate(self.a.calculate(input))
  }
}

pub struct ChainedStatefulTransformsA<A, B, Time> {
  pub a: A,
  pub b: B,
  phantom: PhantomData<Time>,
}

impl<A, B, Time> ChainedStatefulTransformsA<A, B, Time> {
  pub fn new(a: A, b: B) -> Self {
    Self {
      a,
      b,
      phantom: PhantomData,
    }
  }
}

impl<A, B, I, Time: Copy> StatefulTransform<I, Time> for ChainedStatefulTransformsA<A, B, Time>
where
  A: StatefulTransform<I, Time>,
  B: StatefulTransform<<A as StatefulTransform<I, Time>>::Output, Time>,
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

impl<A, B, I, Time> HasSetpoint<I> for ChainedStatefulTransformsA<A, B, Time>
where
  A: HasSetpoint<I>,
{
  fn set_setpoint(&mut self, sp: I) {
    self.a.set_setpoint(sp)
  }
}

pub struct ChainedStatefulTransformsB<A, B, I, Time> {
  pub a: A,
  pub b: B,
  input_type: PhantomData<(I, Time)>,
}

impl<A, B, I, Time> ChainedStatefulTransformsB<A, B, I, Time> {
  pub fn new(a: A, b: B) -> Self {
    Self {
      a,
      b,
      input_type: PhantomData,
    }
  }
}

impl<A, B, I, Time: Copy> StatefulTransform<I, Time> for ChainedStatefulTransformsB<A, B, I, Time>
where
  A: StatefulTransform<I, Time>,
  B: StatefulTransform<<A as StatefulTransform<I, Time>>::Output, Time>,
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

impl<A, B, I, Time> HasSetpoint<<A as StatefulTransform<I, Time>>::Output>
  for ChainedStatefulTransformsB<A, B, I, Time>
where
  A: StatefulTransform<I, Time>,
  B: HasSetpoint<<A as StatefulTransform<I, Time>>::Output>,
{
  fn set_setpoint(&mut self, sp: <A as StatefulTransform<I, Time>>::Output) {
    self.b.set_setpoint(sp)
  }
}

pub trait TransformExt<I>: Transform<I> + Sized {
  fn to_stateful(self) -> StatefulTransformAdapter<Self, I>;
  fn then<Other: Transform<Self::Output>>(self, other: Other) -> ChainedTransforms<Self, Other>;
  fn map<F: Fn(Self::Output) -> O, O>(
    self,
    map: F,
  ) -> ChainedTransforms<Self, MapTransform<F, Self::Output, O>>;
}

impl<T: Transform<I>, I> TransformExt<I> for T {
  fn to_stateful(self) -> StatefulTransformAdapter<Self, I> {
    StatefulTransformAdapter::new(self)
  }

  fn then<Other: Transform<Self::Output>>(self, other: Other) -> ChainedTransforms<Self, Other> {
    ChainedTransforms::new(self, other)
  }

  fn map<F: Fn(Self::Output) -> O, O>(
    self,
    map: F,
  ) -> ChainedTransforms<Self, MapTransform<F, Self::Output, O>> {
    ChainedTransforms::new(self, MapTransform::new(map))
  }
}

pub trait StatefulTransformExt<I, Time>: StatefulTransform<I, Time> + Sized {
  fn cascade<J, Other: HasSetpoint<Self::Output> + StatefulTransform<J, Time>>(
    self,
    other: Other,
  ) -> CascadeTransform<Self, Other, Time>;
  fn map<F: FnMut(Self::Output, Time) -> O, O>(
    self,
    map: F,
  ) -> ChainedStatefulTransformsA<Self, MapStatefulTransform<F, Self::Output, O, Time>, Time>;
}

impl<T: StatefulTransformExt<I, Time>, I, Time> StatefulTransformExt<I, Time> for T {
  fn cascade<J, Other: HasSetpoint<Self::Output> + StatefulTransform<J, Time>>(
    self,
    other: Other,
  ) -> CascadeTransform<Self, Other, Time> {
    CascadeTransform::new(self, other)
  }

  fn map<F: FnMut(Self::Output, Time) -> O, O>(
    self,
    map: F,
  ) -> ChainedStatefulTransformsA<Self, MapStatefulTransform<F, Self::Output, O, Time>, Time> {
    ChainedStatefulTransformsA::new(self, MapStatefulTransform::new(map))
  }
}
