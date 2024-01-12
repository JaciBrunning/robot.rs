pub mod binary;
pub mod linear;

pub trait Filter<T, U> {
  fn calculate(&mut self, input: T) -> U;
  fn reset(&mut self);
}