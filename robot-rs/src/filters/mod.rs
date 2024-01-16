pub mod binary;
pub mod linear;
pub mod pid;

pub trait Filter<T, U> {
  fn calculate(&mut self, input: T) -> U;
  fn reset(&mut self);
}