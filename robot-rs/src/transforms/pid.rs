use std::ops::{Div, Mul, Sub};

use ntcore_rs::{Entry, Publisher, Topic, GenericPublisher, GenericSubscriber};
use num_traits::Zero;
use robot_rs_units::traits::{ToFloat, FromFloat};

use super::{HasSetpoint, StatefulTransform};

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
  last: Option<PIDMeasurement<PV, Output, Time>>
}

impl<
  PV: Mul<Time> + Div<Time> + Copy,
  Output: Div<PV> + Div<<PV as Mul<Time>>::Output> + Div<<PV as Div<Time>>::Output> + Copy,
  Time: Copy
> PID<PV, Output, Time>
where
  Integral<PV, Time>: Copy
{
  pub fn new(kp: Kp<PV, Output>, ki: Ki<PV, Output, Time>, kd: Kd<PV, Output, Time>, setpoint: PV) -> Self {
    Self {
      kp, ki, kd,
      setpoint,
      last: None
    }
  }

  pub fn last(&self) -> Option<PIDMeasurement<PV, Output, Time>> {
    self.last.clone()
  }

  pub fn tunable(self, topic: Topic) -> TunablePID<PV, Output, Time> {
    TunablePID::new(topic, self)
  }
}

impl<
  PV: Mul<Time> + Div<Time> + Copy,
  Output: Div<PV> + Div<<PV as Mul<Time>>::Output> + Div<<PV as Div<Time>>::Output> + Zero + Copy,
  Time: Sub<Time, Output = Time> + Copy
> HasSetpoint<PV> for PID<PV, Output, Time>
where
  Integral<PV, Time>: Copy
{
  fn set_setpoint(&mut self, sp: PV) {
    self.setpoint = sp;
  }
}

impl<
  PV: Mul<Time> + Div<Time> + Copy,
  Output: Div<PV> + Div<<PV as Mul<Time>>::Output> + Div<<PV as Div<Time>>::Output> + Zero + Copy,
  Time: Sub<Time, Output = Time> + Copy
> StatefulTransform<PV, Time> for PID<PV, Output, Time>
where
  PV: Mul<Kp<PV, Output>, Output = Output> + Sub<PV, Output = PV>,
  Integral<PV, Time>: Copy + Zero + Mul<Ki<PV, Output, Time>, Output = Output>,
  Derivative<PV, Time>: Copy + Zero + Mul<Kd<PV, Output, Time>, Output = Output>,
  Kp<PV, Output>: Zero + Copy,
  Ki<PV, Output, Time>: Zero + Copy,
  Kd<PV, Output, Time>: Zero + Copy,
{
  type Output = Output;

  fn calculate(&mut self, input: PV, time: Time) -> Output {
    let now = time;
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

    let output = measurement.output;
    self.last = Some(measurement);
    output
  }

  fn reset(&mut self) {
    self.last = None;
  }
}

// Tunable
pub struct TunablePID<
  PV: Mul<Time> + Div<Time>,
  Output: Div<PV> + Div<<PV as Mul<Time>>::Output> + Div<<PV as Div<Time>>::Output>,
  Time
> where Integral<PV, Time>: Copy {
  pub pid: PID<PV, Output, Time>,
  kp_entry: Entry<f64>,
  ki_entry: Entry<f64>,
  kd_entry: Entry<f64>,
  setpoint_pub: Publisher<f64>,
  error_pub: Publisher<f64>,
  pv_pub: Publisher<f64>,
  integral_sum_pub: Publisher<f64>,
  output_pub: Publisher<f64>
}

impl<
  PV: Mul<Time> + Div<Time> + Copy,
  Output: Div<PV> + Div<<PV as Mul<Time>>::Output> + Div<<PV as Div<Time>>::Output> + Copy,
  Time: Copy
> TunablePID<PV, Output, Time>
where
  Integral<PV, Time>: Copy
{
  pub fn new(topic: Topic, pid: PID<PV, Output, Time>) -> Self {
    Self {
      pid,
      kp_entry: topic.child("gains/Kp").entry(),
      ki_entry: topic.child("gains/Ki").entry(),
      kd_entry: topic.child("gains/Kd").entry(),
      setpoint_pub: topic.child("setpoint").publish(),
      error_pub: topic.child("error").publish(),
      pv_pub: topic.child("processvariable").publish(),
      integral_sum_pub: topic.child("integralsum").publish(),
      output_pub: topic.child("output").publish()
    }
  }
}

impl<
  PV: Mul<Time> + Div<Time> + Copy + ToFloat,
  Output: Div<PV> + Div<<PV as Mul<Time>>::Output> + Div<<PV as Div<Time>>::Output> + Zero + Copy,
  Time: Sub<Time, Output = Time> + Copy
> HasSetpoint<PV> for TunablePID<PV, Output, Time>
where
  Integral<PV, Time>: Copy
{
  fn set_setpoint(&mut self, sp: PV) {
    self.pid.set_setpoint(sp);
    self.setpoint_pub.set(sp.to_f64()).ok();
  }
}

impl<
  PV: Mul<Time> + Div<Time> + Copy + ToFloat,
  Output: Div<PV> + Div<<PV as Mul<Time>>::Output> + Div<<PV as Div<Time>>::Output> + Zero + Copy + ToFloat,
  Time: Sub<Time, Output = Time> + Copy
> StatefulTransform<PV, Time> for TunablePID<PV, Output, Time>
where
  PV: Mul<Kp<PV, Output>, Output = Output> + Sub<PV, Output = PV>,
  Integral<PV, Time>: Copy + Zero + Mul<Ki<PV, Output, Time>, Output = Output> + ToFloat,
  Derivative<PV, Time>: Copy + Zero + Mul<Kd<PV, Output, Time>, Output = Output>,
  Kp<PV, Output>: Zero + Copy + ToFloat + FromFloat,
  Ki<PV, Output, Time>: Zero + Copy + ToFloat + FromFloat,
  Kd<PV, Output, Time>: Zero + Copy + ToFloat + FromFloat,
{
  type Output = Output;

  fn calculate(&mut self, input: PV, time: Time) -> Output {
    match self.kp_entry.get() {
      Some(Ok(kp)) => { self.pid.kp = FromFloat::from_f64(kp); },
      _ => { self.kp_entry.set(self.pid.kp.to_f64()).ok(); },
    }

    match self.ki_entry.get() {
      Some(Ok(ki)) => { self.pid.ki = FromFloat::from_f64(ki); },
      _ => { self.ki_entry.set(self.pid.ki.to_f64()).ok(); },
    }

    match self.kd_entry.get() {
      Some(Ok(kd)) => { self.pid.kd = FromFloat::from_f64(kd); },
      _ => { self.kd_entry.set(self.pid.kd.to_f64()).ok(); },
    }

    let output = self.pid.calculate(input, time);

    if let Some(last) = self.pid.last() {
      self.setpoint_pub.set(last.setpoint.to_f64()).ok();
      self.error_pub.set(last.error.to_f64()).ok();
      self.pv_pub.set(last.process_variable.to_f64()).ok();
      self.integral_sum_pub.set(last.integral_sum.to_f64()).ok();
      self.output_pub.set(last.output.to_f64()).ok();
    }
    output
  }

  fn reset(&mut self) {
    self.pid.reset();
  }
}