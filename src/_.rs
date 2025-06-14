use embedded_hal::{
    delay::DelayNs,
    digital::{OutputPin, PinState},
};
use num_traits::Signed;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Direction {
    Cw,
    Ccw,
}

impl Direction {
    pub fn from_sign(signed: &impl Signed) -> Self {
        if signed.is_negative() {
            Self::Ccw
        } else {
            Self::Cw
        }
    }
}

impl Into<PinState> for Direction {
    fn into(self) -> PinState {
        match self {
            Direction::Cw => PinState::High,
            Direction::Ccw => PinState::Low,
        }
    }
}

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

const MINUTE_US: u32 = 60_000_000;

// TODO: Async
pub struct Stepper<D: DelayNs, DIR: OutputPin, STEP: OutputPin> {
    delay: D,
    dir: DIR,
    step: STEP,
    /// Steps per revolution
    spr: u32,
    step_interval: u32,
    step_div: StepDiv,
}

impl<D: DelayNs, DIR: OutputPin, STEP: OutputPin> Stepper<D, DIR, STEP> {
    pub fn new(
        delay: D,
        dir_pin: DIR,
        step_pin: STEP,
        steps_per_revolution: u32,
        step_div: StepDiv,
    ) -> Self {
        Self {
            delay,
            dir: dir_pin,
            step: step_pin,
            step_interval: MINUTE_US / steps_per_revolution,
            spr: steps_per_revolution,
            step_div,
        }
    }

    pub fn set_speed(&mut self, rpm: u32) -> &mut Self {
        self.step_interval = MINUTE_US / self.spr / rpm / self.step_div as u32;
        self
    }

    pub fn set_dir(&mut self, dir: Direction) -> Result<&mut Self, DIR::Error> {
        self.dir.set_state(dir.into())?;
        Ok(self)
    }

    pub fn move_instant(&mut self, full_steps: u64) -> Result<(), STEP::Error> {
        (0..full_steps * self.step_div as u64).try_for_each(|_| self.step(None))
    }

    pub fn move_smooth(
        &mut self,
        accel_steps: u64,
        steps: u64,
        deaccel_steps: u64,
    ) -> Result<(), STEP::Error> {
        (0..accel_steps * self.step_div as u64)
            .try_for_each(|point| self.step(Some((point, accel_steps))))?;
        (0..steps * self.step_div as u64).try_for_each(|_| self.step(None))?;
        (0..deaccel_steps * self.step_div as u64)
            .try_for_each(|point| self.step(Some((point, deaccel_steps))))
    }

    fn step(&mut self, ease: Option<(u64, u64)>) -> Result<(), STEP::Error> {
        self.step.set_high()?;

        self.delay.delay_us(1);
        self.step.set_low()?;

        let step_time = if let Some((point, len)) = ease {
            let r1 = point as f32 / len as f32;
            let r2 = 0.1 + 0.2 * r1 + 2.2 * r1 * r1 - 1.5 * r1 * r1 * r1;
            (self.step_interval as f32 / r2) as u32
        } else {
            self.step_interval
        };

        self.delay.delay_us(step_time);

        Ok(())
    }
}
