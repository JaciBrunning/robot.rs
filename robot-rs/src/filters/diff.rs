use std::ops::{Div, Sub};

use num_traits::Zero;

use super::Filter;

pub struct DifferentiatingFilter<U, Time> {
  pub last_value: Option<(Time, U)>,
  time_source: fn() -> Time
}

impl<U, Time> DifferentiatingFilter<U, Time> {
  pub fn new(time_source: fn() -> Time) -> Self {
    Self { last_value: None, time_source }
  }
}

impl<U: Copy + Div<Time> + Sub<U, Output=U>, Time: Copy + Sub<Time, Output=Time>> Filter<U> for DifferentiatingFilter<U, Time>
where
  <U as Div<Time>>::Output: Zero
{
  type Output = <U as Div<Time>>::Output;

  fn calculate(&mut self, input: U) -> Self::Output {
    let now = (self.time_source)();
    
    let last = self.last_value.replace((now, input));
    let ret = last.map(|(last_time, last_value)| {
      let dt = now - last_time;
      (input - last_value) / dt
    });

    ret.unwrap_or(Zero::zero())
  }
}