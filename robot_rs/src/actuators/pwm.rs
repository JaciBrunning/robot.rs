use wpilib_hal::{
    hal_safe_call, HAL_DigitalHandle, HAL_FreePWMPort, HAL_GetPWMPosition,
    HAL_GetPWMPulseTimeMicroseconds, HAL_GetPWMSpeed, HAL_GetPort, HAL_InitializePWMPort,
    HAL_LatchPWMZero, HAL_SetPWMConfigMicroseconds, HAL_SetPWMDisabled,
    HAL_SetPWMEliminateDeadband, HAL_SetPWMPeriodScale, HAL_SetPWMPosition,
    HAL_SetPWMPulseTimeMicroseconds, HAL_SetPWMSpeed,
};

use crate::macros::wrapped_traits_nogen;

pub struct PWM {
    port: usize,
    handle: HAL_DigitalHandle,
}

#[repr(i32)]
pub enum PWMPeriodMultiplier {
    Multiplier1X = 1,
    Multiplier2X = 2,
    Multiplier4X = 4,
}

impl PWM {
    pub fn new(port: usize) -> Self {
        let handle = hal_safe_call!(HAL_InitializePWMPort(
            HAL_GetPort(port as i32),
            "PWM::new".as_ptr() as *const i8
        ))
        .unwrap();

        Self { port, handle }
    }

    pub fn port(&self) -> usize {
        self.port
    }

    pub fn set_raw(&mut self, value: u16) {
        hal_safe_call!(HAL_SetPWMPulseTimeMicroseconds(self.handle, value as i32)).unwrap();
    }

    pub fn get_raw(&self) -> u16 {
        hal_safe_call!(HAL_GetPWMPulseTimeMicroseconds(self.handle)).unwrap() as u16
    }

    pub fn set_period_multiplier(&mut self, multiplier: PWMPeriodMultiplier) {
        hal_safe_call!(HAL_SetPWMPeriodScale(
            self.handle,
            match multiplier {
                PWMPeriodMultiplier::Multiplier1X => 3,
                PWMPeriodMultiplier::Multiplier2X => 1,
                PWMPeriodMultiplier::Multiplier4X => 0,
            }
        ))
        .unwrap();
    }

    pub fn set_zero_latch(&mut self) {
        hal_safe_call!(HAL_LatchPWMZero(self.handle)).unwrap()
    }

    pub fn set_deadband_elimination(&mut self, eliminate: bool) {
        hal_safe_call!(HAL_SetPWMEliminateDeadband(self.handle, eliminate as i32)).unwrap()
    }

    pub fn set_bounds(
        &mut self,
        max: i32,
        deadband_max: i32,
        center: i32,
        deadband_min: i32,
        min: i32,
    ) {
        hal_safe_call!(HAL_SetPWMConfigMicroseconds(
            self.handle,
            max,
            deadband_max,
            center,
            deadband_min,
            min
        ))
        .unwrap()
    }

    pub fn speed_controller(self) -> PWMSpeedController {
        PWMSpeedController(self)
    }

    pub fn servo_controller(self) -> PWMServoController {
        PWMServoController(self)
    }
}

impl Drop for PWM {
    fn drop(&mut self) {
        hal_safe_call!(HAL_SetPWMDisabled(self.handle)).unwrap();
        hal_safe_call!(HAL_FreePWMPort(self.handle)).unwrap();
    }
}

pub struct PWMSpeedController(PWM);

impl PWMSpeedController {
    pub fn set_speed(&mut self, speed: f64) {
        // Need to clamp for sim reasons
        hal_safe_call!(HAL_SetPWMSpeed(self.handle, speed.clamp(-1.0, 1.0))).unwrap();
    }

    pub fn get_speed(&self) -> f64 {
        hal_safe_call!(HAL_GetPWMSpeed(self.handle)).unwrap()
    }
}

wrapped_traits_nogen!(PWMSpeedController, PWM);

pub struct PWMServoController(PWM);

impl PWMServoController {
    pub fn set_position(&mut self, position: f64) {
        hal_safe_call!(HAL_SetPWMPosition(self.handle, position.clamp(0.0, 1.0))).unwrap();
    }

    pub fn get_position(&self) -> f64 {
        hal_safe_call!(HAL_GetPWMPosition(self.handle)).unwrap()
    }
}

wrapped_traits_nogen!(PWMServoController, PWM);
