use std::ops::{Add, Sub, Neg};

use num_traits::Zero;

use super::{Transform, ReversibleTransform};

#[derive(Clone)]
pub struct OffsetTransform<T> {
  pub offset: T
}

impl<T> OffsetTransform<T> {
  pub fn new(offset: T) -> Self { Self { offset } }
}

impl<T: Add<T, Output=T> + Copy> Transform<T> for OffsetTransform<T> {
  type Output = T;

  fn calculate(&self, input: T) -> T {
    input + self.offset
  }
}

impl<T: Add<T, Output=T> + Sub<T, Output=T> + Copy> ReversibleTransform<T> for OffsetTransform<T> {
  fn calculate_reverse(&self, output: <Self as Transform<T>>::Output) -> T {
    output - self.offset
  }
}

#[derive(Clone)]
pub struct SymmetricOffsetTransform<T> {
  pub offset: T
}

impl<T> SymmetricOffsetTransform<T> {
  pub fn new(offset: T) -> Self { Self { offset } }
}

impl<T: Add<T, Output=T> + Sub<T, Output=T> + Copy + Zero + PartialOrd<T>> Transform<T> for SymmetricOffsetTransform<T> {
  type Output = T;

  fn calculate(&self, input: T) -> T {
    match input {
      input if input < Zero::zero() => input - self.offset,
      input if input > Zero::zero() => input + self.offset,
      _ => input
    }
  }
}

impl<T: Add<T, Output=T> + Sub<T, Output=T> + Neg<Output = T> + Copy + Zero + PartialOrd<T>> ReversibleTransform<T> for SymmetricOffsetTransform<T> {
  fn calculate_reverse(&self, output: <Self as Transform<T>>::Output) -> T {
    match output {
      output if output < -self.offset => output + self.offset,
      output if output > self.offset => output - self.offset,
      _ => output
    }
  }
}
