use core::ops::RangeInclusive;

pub const DEFAULT_HMC5883L_ADDR: u8 = 0x1E;

pub const REG_CRA: u8 = 0x00;
pub const REG_CRB: u8 = 0x01;
pub const REG_MODE: u8 = 0x02;
pub const REG_OUTXM: u8 = 0x03;
pub const REG_OUTXL: u8 = 0x04;
pub const REG_OUTYM: u8 = 0x07;
pub const REG_OUTYL: u8 = 0x08;
pub const REG_OUTZM: u8 = 0x05;
pub const REG_OUTZL: u8 = 0x06;
pub const REG_STATUS: u8 = 0x09;
pub const REG_ID: RangeInclusive<u8> = 0x0A..=0x0C;

#[allow(non_camel_case_types)]
pub enum AvgSample {
    SAMPLE_1 = 0x00,
    SAMPLE_2 = 0x01,
    SAMPLE_4 = 0x02,
    SAMPLE_8 = 0x03,
}

#[allow(non_camel_case_types)]
pub enum DataOutputRate {
    RATE_0P75 = 0x00,
    RATE_1P5 = 0x01,
    RATE_3 = 0x02,
    RATE_7P5 = 0x03,
    RATE_15 = 0x04,
    RATE_30 = 0x05,
    RATE_75 = 0x06,
}

#[allow(non_camel_case_types)]
pub enum Mode {
    NORMAL = 0x00,
    POSITIVE_BIAS = 0x01,
    NEGATIVE_BIAS = 0x02,
}

#[allow(non_camel_case_types)]
pub enum Gain {
    GAIN_1370 = 0x00,
    GAIN_1090 = 0x01,
    GAIN_820 = 0x02,
    GAIN_660 = 0x03,
    GAIN_440 = 0x04,
    GAIN_390 = 0x05,
    GAIN_330 = 0x06,
    GAIN_230 = 0x07,
}
