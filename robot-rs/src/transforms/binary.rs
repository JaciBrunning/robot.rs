use super::StatefulTransform;

#[derive(Debug, Clone)]
pub enum Edge {
  Rising,
  Falling,
  Both
}

#[derive(Debug)]
pub struct EdgeTransform {
  edge: Edge,
  last: Option<bool>,
}

impl EdgeTransform {
  pub fn new(edge: Edge) -> Self {
    Self {
      edge,
      last: None
    }
  }
}

impl<Time> StatefulTransform<bool, Time> for EdgeTransform {
  type Output = bool;
  
  fn calculate(&mut self, input: bool, _time: Time) -> bool {
    let is_trigd = match (&self.edge, input, self.last) {
      (Edge::Rising, true, Some(false)) => true,
      (Edge::Falling, false, Some(true)) => true,
      (Edge::Both, a, Some(b)) if a != b => true,
      _ => false
    };
    self.last = Some(input);
    is_trigd
  }

  fn reset(&mut self) {
    self.last = None;
  }
}
