use crate::stepper::StepperDriver;
use embassy_time::{Duration, Timer};
use embedded_hal::digital::OutputPin;

pub struct A4988<STEP, DIR, ENABLE> {
    step_pin: STEP,
    dir_pin: DIR,
    enable_pin: ENABLE,
}

impl<STEP, DIR, ENABLE> A4988<STEP, DIR, ENABLE>
where
    STEP: OutputPin,
    DIR: OutputPin,
    ENABLE: OutputPin,
{
    pub fn new(step_pin: STEP, dir_pin: DIR, enable_pin: ENABLE) -> Self {
        Self {
            step_pin,
            dir_pin,
            enable_pin,
        }
    }
}

#[derive(Debug)]
pub enum A4988Error {
    PinError,
}

impl<STEP, DIR, ENABLE> StepperDriver for A4988<STEP, DIR, ENABLE>
where
    STEP: OutputPin,
    DIR: OutputPin,
    ENABLE: OutputPin,
{
    type StepPin = STEP;
    type DirPin = DIR;
    type EnablePin = ENABLE;
    type Error = A4988Error;

    async fn set_direction(&mut self, clockwise: bool) -> Result<(), Self::Error> {
        self.dir_pin
            .set_state(clockwise.into())
            .map_err(|_| A4988Error::PinError)
    }


    async fn step(&mut self) -> Result<(), Self::Error> {
        // Create a step pulse (minimum 1µs high time according to A4988 datasheet)
        self.step_pin.set_high().map_err(|_| A4988Error::PinError)?;
        Timer::after(Duration::from_micros(200)).await;
        self.step_pin.set_low().map_err(|_| A4988Error::PinError)?;
        Ok(())
    }

    async fn set_enabled(&mut self, enabled: bool) -> Result<(), Self::Error> {
        // A4988 enable is active low
        self.enable_pin
            .set_state((!enabled).into())
            .map_err(|_| A4988Error::PinError)
    }
}
