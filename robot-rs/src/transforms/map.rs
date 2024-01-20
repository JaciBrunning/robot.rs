use std::marker::PhantomData;

use super::{Transform, StatefulTransform};

#[derive(Clone)]
pub struct MapTransform<F: Fn(I) -> O, I, O> {
  f: F,
  phantom: PhantomData<(I, O)>
}

impl<F: Fn(I) -> O, I, O> MapTransform<F, I, O> {
  pub fn new(f: F) -> Self { Self { f, phantom: PhantomData } }
}

impl<F: Fn(I) -> O, I, O> Transform<I> for MapTransform<F, I, O> {
  type Output = O;

  fn calculate(&self, input: I) -> Self::Output {
    (self.f)(input)
  }
}

pub struct MapStatefulTransform<F: FnMut(I, Time) -> O, I, O, Time> {
  f: F,
  phantom: PhantomData<(I, O, Time)>
}

impl<F: FnMut(I, Time) -> O, I, O, Time> MapStatefulTransform<F, I, O, Time> {
  pub fn new(f: F) -> Self { Self { f, phantom: PhantomData } }
}

impl<F: FnMut(I, Time) -> O, I, O, Time> StatefulTransform<I, Time> for MapStatefulTransform<F, I, O, Time> {
  type Output = O;

  fn calculate(&mut self, input: I, time: Time) -> Self::Output {
    (self.f)(input, time)
  }
}
