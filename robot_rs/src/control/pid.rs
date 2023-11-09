use std::{collections::VecDeque, fmt::Debug};

use nt4_rs::{nt, types::Value};

#[derive(Clone, Debug)]
pub struct PIDConfig {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    pub izone: Option<f64>,
}

#[derive(Default, Debug, Clone)]
pub struct PIDMeasurement {
    pub time: f64,
    pub setpoint: f64,
    pub process_variable: f64,
    pub error: f64,
    pub derivative: f64,
    pub integral_sum: f64,
    pub output: f64,
    pub output_parts: [f64; 3],
}

macro_rules! pid_coeff_nt {
    ($configpath:ident, $name:literal, $value:expr) => {
        match nt!(read & format!("{}/{}", $configpath, $name)).data {
            Value::Unassigned => nt!(&format!("{}/{}", $configpath, $name), $value).unwrap(),
            Value::Double(val) => $value = val,
            Value::Float(val) => $value = val as f64,
            _ => (),
        }
    };
}

#[derive(Clone)]
pub struct PID {
    config: PIDConfig,
    history_time: f64,
    history: VecDeque<PIDMeasurement>,
    setpoint: f64,
}

impl PID {
    pub fn new(config: PIDConfig, setpoint: f64, history: f64) -> Self {
        Self {
            config,
            setpoint,
            history_time: history,
            history: VecDeque::new(),
        }
    }

    pub fn last(&self) -> Option<&PIDMeasurement> {
        self.history.back()
    }

    pub fn reset(&mut self) {
        self.history.clear();
    }

    pub fn set_setpoint(&mut self, setpoint: f64) {
        self.setpoint = setpoint;
    }

    pub fn get_setpoint(&self) -> f64 {
        self.setpoint
    }

    pub fn calculate(&mut self, pv: f64, time: f64) -> &PIDMeasurement {
        let last = self.last();
        let error = self.setpoint - pv;

        let measurement: PIDMeasurement = match last {
            Some(last) => {
                let dt = time - last.time;
                let derivative = (error - last.error) / dt;
                let integral = match self.config.izone {
                    Some(izone) if error < izone && error > -izone => 0.0,
                    Some(_) | None => error * dt,
                };

                let integral_sum = last.integral_sum + integral;

                let parts = [
                    self.config.kp * error,
                    self.config.ki * integral_sum,
                    self.config.kd * derivative,
                ];
                let output = parts.iter().sum();
                PIDMeasurement {
                    time,
                    setpoint: self.setpoint,
                    process_variable: pv,
                    error,
                    derivative,
                    integral_sum,
                    output,
                    output_parts: parts,
                }
            }
            None => PIDMeasurement {
                time,
                setpoint: self.setpoint,
                process_variable: pv,
                error,
                derivative: 0.0,
                integral_sum: 0.0,
                output: self.config.kp * error,
                output_parts: [self.config.kp * error, 0.0, 0.0],
            },
        };

        // Evict old history
        while self
            .history
            .front()
            .map(|x| (time - x.time) >= self.history_time)
            .unwrap_or(false)
        {
            self.history.pop_front();
        }

        // Add to the history and return
        self.history.push_back(measurement);
        self.history.back().unwrap()
    }

    pub fn is_stable(&self, error_threshold: f64, derivative_thresh: Option<f64>) -> bool {
        let n = self.history.len() as f64;

        if n < 1.0 {
            return false;
        }

        let mut setpoint_error = 0.0;
        let mut error_total = 0.0;
        let mut deriv_total = 0.0;

        for measurement in self.history.iter() {
            setpoint_error = self.setpoint - measurement.setpoint;
            error_total += measurement.error * measurement.error;
            deriv_total += measurement.derivative * measurement.derivative;
        }

        let setpoint_ok = setpoint_error < error_threshold && setpoint_error > -error_threshold;
        let error_ok = error_total.sqrt() < error_threshold;
        // let error_ok = (error_total / n) < error_threshold && (error_total / n) > -error_threshold;

        if let Some(d_thresh) = derivative_thresh {
            // let d_ok = (deriv_total / n) < d_thresh && (deriv_total / n) > -deriv_total;
            let d_ok = deriv_total.sqrt() < d_thresh;
            setpoint_ok && error_ok && d_ok
        } else {
            setpoint_ok && error_ok
        }
    }

    pub fn nt_update(&mut self, path: &str) {
        let basepath = path.to_owned() + "/pid";
        let configpath = format!("{}/config", basepath);

        pid_coeff_nt!(configpath, "kp", self.config.kp);
        pid_coeff_nt!(configpath, "ki", self.config.ki);
        pid_coeff_nt!(configpath, "kd", self.config.kd);

        let last = self.last().cloned().unwrap_or(PIDMeasurement::default());
        nt!(&format!("{}/{}", basepath, "time"), last.time).unwrap();
        nt!(&format!("{}/{}", basepath, "setpoint"), last.setpoint).unwrap();
        nt!(
            &format!("{}/{}", basepath, "process_variable"),
            last.process_variable
        )
        .unwrap();
        nt!(&format!("{}/{}", basepath, "error"), last.error).unwrap();
        nt!(&format!("{}/{}", basepath, "derivative"), last.derivative).unwrap();
        nt!(
            &format!("{}/{}", basepath, "integral_sum"),
            last.integral_sum
        )
        .unwrap();
        nt!(&format!("{}/{}", basepath, "output"), last.output).unwrap();
        nt!(
            &format!("{}/{}", basepath, "output_parts"),
            last.output_parts.to_vec()
        )
        .unwrap();
    }
}
