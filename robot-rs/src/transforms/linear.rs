use std::{
  collections::VecDeque,
  ops::{Add, Div, Mul, Neg, Sub},
};

use num_traits::Zero;

use super::StatefulTransform;

pub struct LinearTransform<GI, GO, I, O> {
  ff_gains: Vec<GI>,
  fb_gains: Vec<GO>,
  inputs: VecDeque<I>,
  outputs: VecDeque<O>,
}

impl<GI: Clone, GO: Clone, I, O> LinearTransform<GI, GO, I, O> {
  pub fn new<FF: Into<GI> + Clone, FB: Into<GO> + Clone>(ff_gains: &[FF], fb_gains: &[FB]) -> Self {
    Self {
      ff_gains: ff_gains.iter().cloned().map(Into::into).collect(),
      fb_gains: fb_gains.iter().cloned().map(Into::into).collect(),
      inputs: VecDeque::with_capacity(ff_gains.len()),
      outputs: VecDeque::with_capacity(fb_gains.len()),
    }
  }
}

impl<GI, GO, I, O, Time> StatefulTransform<I, Time> for LinearTransform<GI, GO, I, O>
where
  O: Zero,
  GI: Copy,
  GO: Copy,
  I: Copy,
  O: Copy,
  GI: Mul<I, Output = O>,
  GO: Mul<O, Output = O>,
  O: Sub<O, Output = O>,
  O: Add<O, Output = O>,
{
  type Output = O;

  fn calculate(&mut self, input: I, _time: Time) -> O {
    if !self.ff_gains.is_empty() {
      if self.inputs.len() >= self.ff_gains.len() {
        self.inputs.pop_front();
      }
      self.inputs.push_back(input);
    }

    let ff = self
      .ff_gains
      .iter()
      .zip(self.inputs.iter())
      .map(|(&a, &b)| a * b)
      .fold(O::zero(), |a, b| a + b);
    let fb = self
      .fb_gains
      .iter()
      .zip(self.outputs.iter())
      .map(|(&a, &b)| a * b)
      .fold(O::zero(), |a, b| a + b);

    let output = ff - fb;

    if !self.fb_gains.is_empty() {
      if self.outputs.len() >= self.fb_gains.len() {
        self.outputs.pop_front();
      }
      self.outputs.push_back(output);
    }

    output
  }

  fn reset(&mut self) {
    self.inputs.clear();
    self.outputs.clear();
  }
}

pub struct LinearTransforms;

impl LinearTransforms {
  pub fn moving_average<T>(n: usize) -> LinearTransform<f64, f64, T, T> {
    let v = vec![1.0 / (n as f64); n];
    LinearTransform::new::<f64, f64>(&v[..], &[])
  }

  pub fn low_pass<Time: Div<Time, Output = O> + Neg<Output = Time>, T, O: Into<f64>>(
    time_constant: Time,
    period: Time,
  ) -> LinearTransform<f64, f64, T, T> {
    let gain = Into::<f64>::into(-period / time_constant).exp();
    LinearTransform::new(&[1.0 - gain], &[-gain])
  }

  pub fn high_pass<Time: Div<Time, Output = O> + Neg<Output = Time>, T, O: Into<f64>>(
    time_constant: Time,
    period: Time,
  ) -> LinearTransform<f64, f64, T, T> {
    let gain = Into::<f64>::into(-period / time_constant).exp();
    LinearTransform::new(&[gain, -gain], &[-gain])
  }
}
