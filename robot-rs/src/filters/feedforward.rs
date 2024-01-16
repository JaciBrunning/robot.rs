use std::ops::{Add, Sub};

use num_traits::Zero;

use super::Filter;

pub struct OffsetFeedforwardFilter<T> {
  pub offset: T
}

impl<T> OffsetFeedforwardFilter<T> {
  pub fn new(offset: T) -> Self { Self { offset } }
}

impl<T: Add<T, Output=T> + Copy> Filter<T> for OffsetFeedforwardFilter<T> {
  type Output = T;
  
  fn calculate(&mut self, input: T) -> T {
    input + self.offset
  }

  fn reset(&mut self) { }
}

pub struct SymmetricFeedforwardFilter<T> {
  pub offset: T
}

impl<T> SymmetricFeedforwardFilter<T> {
  pub fn new(offset: T) -> Self { Self { offset } }
}

impl<T: Add<T, Output=T> + Sub<T, Output=T> + Copy + Zero + PartialOrd<T>> Filter<T> for SymmetricFeedforwardFilter<T> {
  type Output = T;

  fn calculate(&mut self, input: T) -> T {
    match input {
      input if input < Zero::zero() => input - self.offset,
      input if input > Zero::zero() => input + self.offset,
      _ => input
    }
  }

  fn reset(&mut self) { }
}
