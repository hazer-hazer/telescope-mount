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
    pub speed_rpm: f32,    // steps per second
    pub acceleration: f32, // steps per second squared
    pub step_div: StepDiv,
}

impl StepperConfig {
    fn min_interval(&self) -> f32 {
        0.676 * SQRT_2 / self.config.acceleration.sqrt()
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
    // Current velocity in steps per second
    current_interval: f32,
    driver: D,
    config: StepperConfig,
}

impl<D: StepperDriver> Stepper<D> {
    pub fn new(driver: D, config: StepperConfig) -> Self {
        Self {
            position: 0,
            target: 0,
            current_interval: MIN_VELOCITY,
            driver,
            config,
        }
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
        let steps_to_move = self.target - self.position;
        if steps_to_move == 0 {
            return Ok(());
        }

        let direction = steps_to_move > 0;
        let steps_to_move = steps_to_move.abs() as u32 * self.config.step_div as u32;
        let midpoint = steps_to_move as f32 / 2.0;

        self.driver
            .set_direction(direction)
            .await
            .map_err(|err| StepperError::Driver(err))?;
        self.driver
            .set_enabled(true)
            .await
            .map_err(|err| StepperError::Driver(err))?;

        // Calculate acceleration parameters
        let acceleration = self.config.acceleration;
        let max_velocity = self.config.speed_rpm
            * self.config.steps_per_rev as f32
            * self.config.step_div as u32 as f32
            / 60.0;
        let acceleration_steps = (max_velocity * max_velocity) / (2.0 * acceleration);
        let acceleration_steps = acceleration_steps.min(midpoint);
        let deceleration_steps = acceleration_steps;

        let coasting_steps = if steps_to_move > (acceleration_steps + deceleration_steps) as u32 {
            steps_to_move - (acceleration_steps + deceleration_steps) as u32
        } else {
            // Triangular profile (no coasting)
            0
        };

        assert!((acceleration_steps + deceleration_steps) as u32 <= steps_to_move);

        // Acceleration phase
        self.accelerate(1_000_000.0 / max_velocity, acceleration_steps as u32)
            .await?;

        // Coasting phase (constant speed)
        if coasting_steps > 0 {
            self.run_at_constant_speed(coasting_steps).await?;
        }

        // Deceleration phase
        self.decelerate(deceleration_steps as u32).await?;

        // Update current position
        self.position = self.target;
        self.current_interval = self.config.min_interval();
        self.driver
            .set_enabled(false)
            .await
            .map_err(|err| StepperError::Driver(err))?;

        Ok(())
    }

    /// Acceleration phase
    async fn accelerate(&mut self, max_interval: f32, steps: u32) -> Result<(), StepperError<D>> {
        let mut delay = 0.676 * SQRT_2 * 1_000_000.0 / self.config.acceleration;
        let delay0 = delay;

        for step in 0..steps {
            Timer::after(Duration::from_micros(delay as u64)).await;

            self.driver
                .step()
                .await
                .map_err(|err| StepperError::Driver(err))?;
            self.position += if self.target > self.position { 1 } else { -1 };

            delay = delay0 - 2.0 * delay0 / (4.0 * step as f32 + 1.0).min(max_interval);
            self.current_interval = delay;
        }

        Ok(())
    }

    /// Run at constant speed
    async fn run_at_constant_speed(&mut self, steps: u32) -> Result<(), StepperError<D>> {
        // TODO: Or use max speed?
        let delay = Duration::from_micros(self.current_interval as u64);

        for _ in 0..steps {
            Timer::after(delay).await;
            self.driver
                .step()
                .await
                .map_err(|err| StepperError::Driver(err))?;
            self.position += if self.target > self.position { 1 } else { -1 };
        }

        Ok(())
    }

    /// Deceleration phase
    async fn decelerate(&mut self, steps: u32) -> Result<(), StepperError<D>> {
        let mut delay = self.current_interval;
        let min_interval = self.config.min_interval();

        for step in 0..steps {
            Timer::after(Duration::from_micros(delay as u64)).await;

            self.driver
                .step()
                .await
                .map_err(|err| StepperError::Driver(err))?;
            self.position += if self.target > self.position { 1 } else { -1 };

            let decel_step =
                delay = delay0 - 2.0 * delay0 / (4.0 * step as f32 + 1.0).max(min_interval);
            self.current_interval = delay;
        }

        Ok(())
    }

    /// Calculate delay between steps based on current speed
    fn calculate_step_delay(&self, velocity: f32) -> Result<Duration, StepperError<D>> {
        assert!(velocity >= 0.0);

        if velocity == 0.0 {
            panic!("Velocity is zero")
            // Duration::from_micros(u64::MAX)
            // Ok(Duration::from_micros(0))
        } else {
            Ok(Duration::from_micros((1_000_000.0 / velocity) as u64))
        }
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
