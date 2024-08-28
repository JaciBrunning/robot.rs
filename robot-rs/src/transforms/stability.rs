use std::{
  collections::VecDeque,
  ops::{Div, Mul, Sub},
};

use robot_rs_units::traits::ToFloat;

use super::StatefulTransform;

// TODO: Make this actual RMS

pub struct RMSStabilityFilter<I: Mul<I> + Div<Time>, Time> {
  pub error_thresh: I,
  pub deriv_thresh: Option<<I as Div<Time>>::Output>,
  history: VecDeque<(I, Time)>,
  deriv_history: VecDeque<<I as Div<Time>>::Output>,
  history_len: usize,
}

impl<I: Mul<I> + Div<Time>, Time> RMSStabilityFilter<I, Time> {
  pub fn new(
    error_thresh: I,
    deriv_thresh: Option<<I as Div<Time>>::Output>,
    history_len: usize,
  ) -> Self {
    Self {
      deriv_thresh,
      error_thresh,
      history: VecDeque::with_capacity(history_len),
      deriv_history: VecDeque::with_capacity(history_len - 1),
      history_len,
    }
  }
}

impl<I: Mul<I> + Div<Time>, Time> StatefulTransform<I, Time> for RMSStabilityFilter<I, Time>
where
  I: ToFloat + Sub<I, Output = I> + Copy,
  <I as Div<Time>>::Output: ToFloat + Copy,
  Time: Copy + Sub<Time, Output = Time>,
{
  type Output = bool;

  fn calculate(&mut self, input: I, time: Time) -> Self::Output {
    if !self.history.is_empty() {
      let (last_value, last_time) = *self.history.back().unwrap();
      self
        .deriv_history
        .push_back((input - last_value) / (time - last_time));
    }
    self.history.push_back((input, time));

    while self.history.len() > self.history_len {
      self.history.pop_front();
    }
    while self.deriv_history.len() > self.history_len - 1 {
      self.deriv_history.pop_front();
    }

    let error_ok = if self.history.len() >= self.history_len {
      let rms_error = self
        .history
        .iter()
        .map(|x| x.0.to_f64() * x.0.to_f64())
        .fold(0.0, |a, b| a + b)
        / (self.history.len() as f64);
      rms_error.sqrt() <= self.error_thresh.to_f64()
    } else {
      false
    };

    let deriv_ok = match (self.deriv_thresh, self.deriv_history.len()) {
      (Some(thresh), i) if i >= (self.history_len - 1) => {
        let rms_deriv = self
          .deriv_history
          .iter()
          .map(|x| x.to_f64() * x.to_f64())
          .fold(0.0, |a, b| a + b)
          / (self.deriv_history.len() as f64);
        rms_deriv.sqrt() <= thresh.to_f64()
      }
      (None, _) => true,
      _ => false,
    };

    error_ok && deriv_ok
  }

  fn reset(&mut self) {
    self.history.clear()
  }
}
