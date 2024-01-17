use super::StatefulFilter;

#[derive(Debug, Clone)]
pub enum Edge {
  Rising,
  Falling,
  Both
}

#[derive(Debug)]
pub struct EdgeFilter {
  edge: Edge,
  last: Option<bool>,
}

impl EdgeFilter {
  pub fn new(edge: Edge) -> Self {
    Self {
      edge,
      last: None
    }
  }
}

impl<Time> StatefulFilter<bool, Time> for EdgeFilter {
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
