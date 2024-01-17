use std::ops::{Add, Sub};

use num_traits::Zero;

use super::Filter;

pub struct OffsetFeedforwardFilter<T> {
  pub offset: T
}

impl<T> OffsetFeedforwardFilter<T> {
  pub fn new(offset: T) -> Self { Self { offset } }
}

impl<T: Add<T, Output=T> + Copy, Time> Filter<T, Time> for OffsetFeedforwardFilter<T> {
  type Output = T;

  fn calculate(&mut self, input: T, _time: Time) -> T {
    input
  }
}

pub struct SymmetricFeedforwardFilter<T> {
  pub offset: T
}

impl<T> SymmetricFeedforwardFilter<T> {
  pub fn new(offset: T) -> Self { Self { offset } }
}

impl<T: Add<T, Output=T> + Sub<T, Output=T> + Copy + Zero + PartialOrd<T>, Time> Filter<T, Time> for SymmetricFeedforwardFilter<T> {
  type Output = T;

  fn calculate(&mut self, input: T, _time: Time) -> T {
    match input {
      input if input < Zero::zero() => input - self.offset,
      input if input > Zero::zero() => input + self.offset,
      _ => input
    }
  }
}
