pub struct MinMax<T> {
  pub min: T,
  pub max: T
}

impl<T> MinMax<T> {
  pub fn new(min: T, max: T) -> Self {
    Self { min, max }
  }
}