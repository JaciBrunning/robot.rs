pub trait DistanceSource {
  fn get_distance(&self) -> f64;
}

pub trait SimDistanceSource {
  fn set_distance(&mut self, distance: f64);
}

pub struct NaiveDistanceSource(f64);
impl DistanceSource for NaiveDistanceSource {
  fn get_distance(&self) -> f64 {
    self.0
  }
}
impl SimDistanceSource for NaiveDistanceSource {
  fn set_distance(&mut self, distance: f64) {
    self.0 = distance
  }
}
impl NaiveDistanceSource {
  pub fn new(distance: f64) -> Self {
    Self(distance)
  }
}
