use std::{ops::{Mul, Div, Sub, Add}, collections::VecDeque};

use num_traits::Zero;

use super::StatefulTransform;

// TODO: Make this actual RMS

pub struct RMSStabilityFilter<I: Mul<I> + Div<Time>, Time>
where
  <I as Div<Time>>::Output: Mul<<I as Div<Time>>::Output>
{
  pub error_thresh: <I as Mul<I>>::Output,
  pub deriv_thresh: Option<<<I as Div<Time>>::Output as Mul<<I as Div<Time>>::Output>>::Output>,
  history: VecDeque<(I, Time)>,
  deriv_history: VecDeque<<I as Div<Time>>::Output>,
  history_len: usize
}

impl<I: Mul<I> + Div<Time>, Time> RMSStabilityFilter<I, Time>
where
  <I as Div<Time>>::Output: Mul<<I as Div<Time>>::Output>
{
  pub fn new(
    error_thresh: <I as Mul<I>>::Output,
    deriv_thresh: Option<<<I as Div<Time>>::Output as Mul<<I as Div<Time>>::Output>>::Output>,
    history_len: usize
  ) -> Self {
    Self {
      deriv_thresh,
      error_thresh,
      history: VecDeque::with_capacity(history_len),
      deriv_history: VecDeque::with_capacity(history_len - 1),
      history_len
    }
  }
}

impl<I: Mul<I> + Div<Time>, Time> StatefulTransform<I, Time> for RMSStabilityFilter<I, Time>
where
  I: Copy + Sub<I, Output = I> + Zero,
  <I as Mul<I>>::Output: Add<<I as Mul<I>>::Output, Output = <I as Mul<I>>::Output> + Zero + PartialOrd<<I as Mul<I>>::Output>,
  <I as Div<Time>>::Output: Mul<<I as Div<Time>>::Output> + Copy,
  // Good god, I need to alias this
  <<I as Div<Time>>::Output as Mul<<I as Div<Time>>::Output>>::Output: Copy + Zero + Add<<<I as Div<Time>>::Output as Mul<<I as Div<Time>>::Output>>::Output, Output = <<I as Div<Time>>::Output as Mul<<I as Div<Time>>::Output>>::Output> + PartialOrd<<<I as Div<Time>>::Output as Mul<<I as Div<Time>>::Output>>::Output>,
  Time: Copy + Sub<Time, Output = Time>
{
  type Output = bool;

  fn calculate(&mut self, input: I, time: Time) -> Self::Output {
    if self.history.len() >= 1 {
      let (last_value, last_time) = *self.history.back().unwrap();
      self.deriv_history.push_back((input - last_value) / (time - last_time));
    }
    self.history.push_back((input, time));

    while self.history.len() > self.history_len {
      self.history.pop_front();
    }
    while self.deriv_history.len() > self.history_len - 1 {
      self.history.pop_front();
    }

    let error_ok = if self.history.len() >= self.history_len {
      let rms_error = self.history.iter().map(|x| x.0 * x.0).fold(Zero::zero(), |a: <I as Mul<I>>::Output, b| a + b);
      rms_error <= self.error_thresh
    } else {
      false
    };

    let deriv_ok = match (self.deriv_thresh, self.deriv_history.len()) {
      (Some(thresh), i) if i >= (self.history_len - 1) => {
        let rms_deriv = self.deriv_history.iter().map(|x| *x * *x).fold(Zero::zero(), |a: <<I as Div<Time>>::Output as Mul<<I as Div<Time>>::Output>>::Output, b| a + b);
        rms_deriv <= thresh
      },
      (None, _) => true,
      _ => false
    };

    error_ok && deriv_ok
  }

  fn reset(&mut self) {
    self.history.clear()
  }
}