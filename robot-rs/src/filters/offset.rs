use std::ops::{Add, Sub, Neg};

use num_traits::Zero;

use super::{Filter, ReversibleFilter};

#[derive(Clone)]
pub struct OffsetFilter<T> {
  pub offset: T
}

impl<T> OffsetFilter<T> {
  pub fn new(offset: T) -> Self { Self { offset } }
}

impl<T: Add<T, Output=T> + Copy> Filter<T> for OffsetFilter<T> {
  type Output = T;

  fn calculate(&self, input: T) -> T {
    input + self.offset
  }
}

impl<T: Add<T, Output=T> + Sub<T, Output=T> + Copy> ReversibleFilter<T> for OffsetFilter<T> {
  fn calculate_reverse(&self, output: <Self as Filter<T>>::Output) -> T {
    output - self.offset
  }
}

#[derive(Clone)]
pub struct SymmetricOffsetFilter<T> {
  pub offset: T
}

impl<T> SymmetricOffsetFilter<T> {
  pub fn new(offset: T) -> Self { Self { offset } }
}

impl<T: Add<T, Output=T> + Sub<T, Output=T> + Copy + Zero + PartialOrd<T>> Filter<T> for SymmetricOffsetFilter<T> {
  type Output = T;

  fn calculate(&self, input: T) -> T {
    match input {
      input if input < Zero::zero() => input - self.offset,
      input if input > Zero::zero() => input + self.offset,
      _ => input
    }
  }
}

impl<T: Add<T, Output=T> + Sub<T, Output=T> + Neg<Output = T> + Copy + Zero + PartialOrd<T>> ReversibleFilter<T> for SymmetricOffsetFilter<T> {
  fn calculate_reverse(&self, output: <Self as Filter<T>>::Output) -> T {
    match output {
      output if output < -self.offset => output + self.offset,
      output if output > self.offset => output - self.offset,
      _ => output
    }
  }
}
