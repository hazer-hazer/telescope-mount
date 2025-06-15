pub mod a4988;

use core::{f32::consts::SQRT_2, fmt::Debug};

use embassy_time::{Duration, Timer};
use embedded_hal::digital::OutputPin;
use esp_println::{dbg, println};
use micromath::F32Ext;

/// Min velocity given in steps per second
const MIN_VELOCITY: f32 = 1.0;

#[allow(unused)]
#[repr(u8)]
#[derive(Default, Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum StepDiv {
    #[default]
    Div1 = 1,
    Div2 = 2,
    Div4 = 4,
    Div8 = 8,
    Div16 = 16,
    Div32 = 32,
    Div64 = 64,
    Div128 = 128,
}

pub trait StepperDriver {
    type StepPin: OutputPin;
    type DirPin: OutputPin;
    type EnablePin: OutputPin;
    type Error: Debug;

    /// Set the direction pin state
    async fn set_direction(&mut self, clockwise: bool) -> Result<(), Self::Error>;

    /// Pulse the step pin
    async fn step(&mut self) -> Result<(), Self::Error>;

    /// Enable or disable the driver
    async fn set_enabled(&mut self, enabled: bool) -> Result<(), Self::Error>;

    // /// Set microstepping mode (if supported by driver)
    // async fn set_step_div(&mut self, division: StepDiv) -> Result<(), Self::Error>;
}

pub struct StepperConfig {
    pub steps_per_rev: u32,
    pub max_speed_rpm: f32, // steps per second
    pub acceleration: f32,  // steps per second squared
    pub step_div: StepDiv,
}

impl StepperConfig {
    fn min_interval(&self) -> u64 {
        (0.676 * (2.0 / self.acceleration).sqrt() * 1_000_000.0) as u64
    }

    fn max_interval(&self) -> u64 {
        (1_000_000.0 / (self.max_speed_rpm * self.steps_per_rev as f32 / 60.0)) as u64
    }
}

pub enum StepperError<D: StepperDriver> {
    Driver(D::Error),
}

impl<D: StepperDriver> Debug for StepperError<D> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Driver(arg0) => f.debug_tuple("Driver").field(arg0).finish(),
        }
    }
}

pub struct Stepper<D: StepperDriver> {
    position: i32,
    target: i32,
    speed: f32,
    // Current velocity in steps per second
    step_interval_us: u64,
    driver: D,
    dir_cw: bool,
    // Local step counter for current target
    local_step: i32,
    config: StepperConfig,
}

impl<D: StepperDriver> Stepper<D> {
    pub fn new(driver: D, config: StepperConfig) -> Self {
        let this = Self {
            position: 0,
            target: 0,
            step_interval_us: 0,
            dir_cw: false,
            local_step: 0,
            speed: 0.0,
            driver,
            config,
        };

        this
    }

    pub async fn move_to(&mut self, position: i32) -> Result<(), StepperError<D>> {
        self.target = position;
        self.run().await
    }

    pub async fn move_by(&mut self, delta: i32) -> Result<(), StepperError<D>> {
        self.target = self.position + delta;
        self.run().await
    }

    async fn run(&mut self) -> Result<(), StepperError<D>> {
        self.driver
            .set_enabled(true)
            .await
            .map_err(|err| StepperError::Driver(err))?;

        while self.step().await {
            self.driver
                .set_direction(self.dir_cw)
                .await
                .map_err(|err| StepperError::Driver(err))?;
            self.driver
                .step()
                .await
                .map_err(|err| StepperError::Driver(err))?;
        }

        self.driver
            .set_enabled(false)
            .await
            .map_err(|err| StepperError::Driver(err))?;

        Ok(())
    }

    async fn step(&mut self) -> bool {
        let dist = self.target - self.position;
        let steps_to_stop = (self.speed.powi(2) / (2.0 * self.config.acceleration)) as i32;

        if dist == 0 && steps_to_stop <= 1 {
            self.step_interval_us = 0;
            self.speed = 0.0;
            self.local_step = 0;
            return false;
        }

        if dist > 0 {
            if self.local_step > 0 && (steps_to_stop >= dist || !self.dir_cw) {
                self.local_step = -(steps_to_stop as i32);
            } else if self.local_step < 0 && steps_to_stop < dist && self.dir_cw {
                self.local_step = -self.local_step;
            }
        } else if dist < 0 {
            if self.local_step > 0 && (steps_to_stop >= -dist || self.dir_cw) {
                self.local_step = -steps_to_stop;
            } else if self.local_step < 0 && steps_to_stop < -dist && !self.dir_cw {
                self.local_step = -self.local_step;
            }
        }

        self.step_interval_us = if self.local_step == 0 {
            self.dir_cw = dist > 0;
            self.config.min_interval()
        } else {
            let step_interval = self.step_interval_us as f32;
            ((step_interval - 2.0 * step_interval / (4.0 * self.local_step as f32) + 1.0) as u64)
                .max(self.config.max_interval())
        };

        self.local_step += 1;
        self.speed =
            1_000_000.0 / self.step_interval_us as f32 * if self.dir_cw { 1.0 } else { -1.0 };
        self.position += if self.dir_cw { 1 } else { -1 };

        Timer::after(Duration::from_micros(self.step_interval_us)).await;

        true
    }

    /// Get current position
    pub fn position(&self) -> i32 {
        self.position
    }

    /// Set current position (for homing/reset)
    pub fn overwrite_position(&mut self, position: i32) {
        self.position = position;
    }
}
