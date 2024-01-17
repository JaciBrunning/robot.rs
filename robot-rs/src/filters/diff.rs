use std::ops::{Div, Sub};

use num_traits::Zero;

use super::Filter;

pub struct DifferentiatingFilter<U, Time> {
  pub last_value: Option<(Time, U)>,
}

impl<U, Time> DifferentiatingFilter<U, Time> {
  pub fn new() -> Self {
    Self { last_value: None }
  }
}

impl<U: Copy + Div<Time> + Sub<U, Output=U>, Time: Copy + Sub<Time, Output=Time>> Filter<U, Time> for DifferentiatingFilter<U, Time>
where
  <U as Div<Time>>::Output: Zero
{
  type Output = <U as Div<Time>>::Output;

  fn calculate(&mut self, input: U, time: Time) -> Self::Output {
    let last = self.last_value.replace((time, input));
    let ret = last.map(|(last_time, last_value)| {
      let dt = time - last_time;
      (input - last_value) / dt
    });

    ret.unwrap_or(Zero::zero())
  }
}