use core::ops::RangeInclusive;

use num_derive::FromPrimitive;

pub const DEFAULT_QMC5883P_ADDR: u8 = 0x2C;

#[derive(Debug, FromPrimitive)]
#[repr(u8)]
pub enum Range {
    Gauss30 = 0b00,
    Gauss12 = 0b01,
    Gauss8 = 0b10,
    Gauss2 = 0b11,
}

impl Range {
    pub fn lsb_per_gauss(&self) -> f32 {
        match self {
            Range::Gauss30 => 1_000.0,
            Range::Gauss12 => 2_500.0,
            Range::Gauss8 => 3_750.0,
            Range::Gauss2 => 15_000.0,
        }
    }
}

#[derive(Debug, FromPrimitive)]
#[repr(u8)]
pub enum SetResetMode {
    SetResetOn = 0b00,
    SetOnlyOn = 0b01,
    SetResetOff = 0b10,
}

#[derive(Debug, FromPrimitive)]
#[repr(u8)]
pub enum OverSampling {
    OverSampling1 = 0b11,
    OverSampling2 = 0b10,
    OverSampling4 = 0b01,
    OverSampling8 = 0b00,
}

#[repr(u8)]
#[derive(Debug, FromPrimitive)]
pub enum DownSampling {
    DownSampling1 = 0b00,
    DownSampling2 = 0b01,
    DownSampling4 = 0b10,
    DownSampling8 = 0b11,
}

#[repr(u8)]
#[derive(FromPrimitive, Debug)]
pub enum OutputDataRate {
    Rate10HZ = 0b00,
    Rate50HZ = 0b01,
    Rate100HZ = 0b10,
    Rate200HZ = 0b11,
}

#[derive(Debug, FromPrimitive)]
#[repr(u8)]
pub enum Mode {
    Suspend = 0b00,
    Normal = 0b01,
    Single = 0b10,
    Continuous = 0b11,
}

pub mod reg {
    use crate::gy87::bits::BitBlock;

    pub const CHIP_ID_REG: u8 = 0x00;
    pub const CHIP_ID_EXPECTED: u8 = 0x80;
    pub const DATA_REG: u8 = 0x01;

    pub struct Status;

    impl Status {
        pub const ADDR: u8 = 0x09;
        pub const OVERFLOW: u8 = 0x01;
        pub const DATA_READY: u8 = 0x00;
    }

    pub struct Control1;
    impl Control1 {
        pub const ADDR: u8 = 0x0A;

        pub const MODE: BitBlock = BitBlock { bit: 0, length: 2 };
        pub const OUTPUT_DATA_RATE: BitBlock = BitBlock { bit: 2, length: 2 };
        pub const OVER_SAMPLING: BitBlock = BitBlock { bit: 4, length: 2 };
        pub const DOWN_SAMPLING: BitBlock = BitBlock { bit: 6, length: 2 };
    }

    pub struct Control2;
    impl Control2 {
        pub const ADDR: u8 = 0x0B;

        pub const SOFT_RESET: u8 = 7;
        pub const SELF_TEST: u8 = 6;
        pub const RANGE: BitBlock = BitBlock { bit: 2, length: 2 };
        pub const SET_RESET_MODE: BitBlock = BitBlock { bit: 0, length: 2 };
    }
}
