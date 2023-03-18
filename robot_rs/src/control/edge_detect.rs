use futures::Future;

use crate::sensors::digital::DigitalInput;

pub enum Edge {
  Rising,
  Falling,
  Both
}

pub struct EdgeDetector<'a>{
  edge: Edge,
  input: &'a dyn DigitalInput,
  last: bool,
}

impl<'a> EdgeDetector<'a> {
  pub fn new(input: &'a dyn DigitalInput, edge: Edge) -> Self {
    Self {
      edge,
      input,
      last: input.get()
    }
  }

  pub fn get(&mut self) -> bool {
    let value = self.input.get();
    let is_trigd = match (&self.edge, value, self.last) {
      (Edge::Rising, true, false) => true,
      (Edge::Falling, false, true) => true,
      (Edge::Both, a, b) if a != b => true,
      _ => false
    };
    self.last = value;
    is_trigd
  }

  pub async fn run<F, R, Fut>(mut self, cb: F)
  where
    F: Fn() -> Fut,
    Fut: Future<Output = R>
  {
    loop {
      if self.get() {
        cb().await;
      }
      tokio::time::sleep(tokio::time::Duration::from_millis(20)).await;
    }
  }
}

pub struct EdgeDetectorOwned<I: DigitalInput> {
  edge: Edge,
  input: I,
  last: bool
}

impl<I: DigitalInput> EdgeDetectorOwned<I> {
  pub fn new(input: I, edge: Edge) -> Self {
    let last = input.get();
    Self {
      edge,
      input,
      last
    }
  }

  pub fn get(&mut self) -> bool {
    let value = self.input.get();
    let is_trigd = match (&self.edge, value, self.last) {
      (Edge::Rising, true, false) => true,
      (Edge::Falling, false, true) => true,
      (Edge::Both, a, b) if a != b => true,
      _ => false
    };
    self.last = value;
    is_trigd
  }

  pub async fn run<F, R, Fut>(mut self, cb: F)
  where
    F: Fn() -> Fut,
    Fut: Future<Output = R>,
  {
    loop {
      if self.get() {
        cb().await;
      }
      tokio::time::sleep(tokio::time::Duration::from_millis(20)).await;
    }
  }
}