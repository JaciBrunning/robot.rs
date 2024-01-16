use std::{ops::{Div, Mul, Sub}, collections::VecDeque};

use num_traits::Zero;

use super::Filter;

pub type Derivative<PV, Time> = <PV as Div<Time>>::Output;
pub type Integral<PV, Time> = <PV as Mul<Time>>::Output;

pub type Kp<PV, Output> = <Output as Div<PV>>::Output;
pub type Ki<PV, Output, Time> = <Output as Div<Integral<PV, Time>>>::Output;
pub type Kd<PV, Output, Time> = <Output as Div<Derivative<PV, Time>>>::Output;

#[derive(Clone, Copy)]
pub struct PIDMeasurement<
  PV: Mul<Time> + Div<Time>,
  Output: Div<PV> + Div<<PV as Mul<Time>>::Output> + Div<<PV as Div<Time>>::Output>,
  Time
>
where
  Integral<PV, Time>: Copy
{
  pub time: Time,
  pub setpoint: PV,
  pub process_variable: PV,
  pub error: PV,
  pub integral_sum: Integral<PV, Time>,
  pub output: Output,
  pub output_parts: (Output, Output, Output),
}

pub struct PID<
  PV: Mul<Time> + Div<Time>,
  Output: Div<PV> + Div<<PV as Mul<Time>>::Output> + Div<<PV as Div<Time>>::Output>,
  Time
>
where
  Integral<PV, Time>: Copy
{
  kp: Kp<PV, Output>,
  ki: Ki<PV, Output, Time>,
  kd: Kd<PV, Output, Time>,
  setpoint: PV,
  history: VecDeque<PIDMeasurement<PV, Output, Time>>,
  history_len: usize,
  get_time: fn() -> Time,
}

impl<
  PV: Mul<Time> + Div<Time> + Copy,
  Output: Div<PV> + Div<<PV as Mul<Time>>::Output> + Div<<PV as Div<Time>>::Output> + Copy,
  Time: Copy
> PID<PV, Output, Time>
where
  Integral<PV, Time>: Copy
{
  pub fn new(kp: Kp<PV, Output>, ki: Ki<PV, Output, Time>, kd: Kd<PV, Output, Time>, setpoint: PV, history_len: usize, get_time: fn() -> Time) -> Self {
    if history_len == 0 {
      panic!("History must be greater than 0");
    }

    Self {
      kp, ki, kd,
      setpoint,
      history: VecDeque::with_capacity(history_len),
      history_len,
      get_time,
    }
  }

  pub fn last(&self) -> Option<PIDMeasurement<PV, Output, Time>> {
    self.history.back().cloned()
  }
}

impl<
  PV: Mul<Time> + Div<Time> + Copy,
  Output: Div<PV> + Div<<PV as Mul<Time>>::Output> + Div<<PV as Div<Time>>::Output> + Zero + Copy,
  Time: Sub<Time, Output = Time> + Copy
> Filter<PV, Output> for PID<PV, Output, Time>
where
  PV: Mul<Kp<PV, Output>, Output = Output> + Sub<PV, Output = PV>,
  Integral<PV, Time>: Copy + Zero + Mul<Ki<PV, Output, Time>, Output = Output>,
  Derivative<PV, Time>: Copy + Zero + Mul<Kd<PV, Output, Time>, Output = Output>,
  Kp<PV, Output>: Zero + Copy,
  Ki<PV, Output, Time>: Zero + Copy,
  Kd<PV, Output, Time>: Zero + Copy,
{
  fn calculate(&mut self, input: PV) -> Output {
    let now = (self.get_time)();
    let last = self.last();
    let error = self.setpoint - input;

    let measurement = match last {
      Some(last) => {
        let dt = now - last.time;
        let deriv = (error - last.error) / dt;
        let integral = error * dt;

        let integral_sum = last.integral_sum + integral;

        let parts = ( error * self.kp, integral_sum * self.ki, deriv * self.kd );
        let output = parts.0 + parts.1 + parts.2;

        PIDMeasurement {
          time: now,
          setpoint: self.setpoint,
          process_variable: input,
          error,
          integral_sum,
          output,
          output_parts: parts
        }
      },
      None => PIDMeasurement {
        time: now,
        setpoint: self.setpoint,
        process_variable: input,
        error,
        integral_sum: Zero::zero(),
        output: error * self.kp,
        output_parts: (error * self.kp, Zero::zero(), Zero::zero()),
      },
    };

    while self.history.len() >= self.history_len {
      self.history.pop_front();
    }

    let output = measurement.output;
    self.history.push_back(measurement);
    output
  }

  fn reset(&mut self) {
    self.history.clear();
  }
}
