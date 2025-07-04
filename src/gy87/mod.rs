use crate::{gy87::qmc5883p::*, list_i2c_devices};
use bits::*;
use core::ops::Neg;
use embassy_time::{with_timeout, Duration, Timer};
use embedded_hal_async::{delay::DelayNs, i2c::I2c};
use esp_println::println;
use micromath::F32Ext;
use mpu6050::*;
use nalgebra::Vector3;
use num_traits::FromPrimitive;

mod bits;
pub mod kalman;
pub mod madgwick;
pub mod mpu6050;
mod qmc5883p;

/// PI, f32
pub const PI: f32 = core::f32::consts::PI;

/// PI / 180, for conversion to radians
pub const PI_180: f32 = PI / 180.0;

const Q15: f32 = 32768.0;

/// All possible errors in this crate
#[derive(Debug)]
pub enum Error<E> {
    I2c(E),
    Mpu6050InvalidChipId(u8),
    MagWrongId(u8),
    Timeout,
    Conversion,
}

/// Handles all operations on/with Mpu6050
pub struct Gy87<I2C> {
    i2c: I2C,
    addr: u8,
    mag_addr: u8,
    accel_sens: f32,
    gyro_sens: f32,
}

impl<I2C, E> Gy87<I2C>
where
    I2C: I2c<Error = E>,
{
    pub fn i2c(&mut self) -> &mut I2C {
        &mut self.i2c
    }

    /// Side effect free constructor with default sensitivies, no calibration
    pub fn new(i2c: I2C) -> Self {
        Self {
            i2c,
            accel_sens: AccelRange::G2.sensitivity(),
            gyro_sens: GyroRange::D250.sensitivity(),
            addr: DEFAULT_MPU6050_ADDR,
            mag_addr: DEFAULT_QMC5883P_ADDR,
        }
    }

    pub fn with_mpu6050_addr(mut self, addr: u8) -> Self {
        self.addr = addr;
        self
    }

    pub fn with_hmc5883_addr(mut self, addr: u8) -> Self {
        self.mag_addr = addr;
        self
    }

    pub fn with_accel_range(mut self, accel_range: AccelRange) -> Self {
        self.accel_sens = accel_range.sensitivity();
        self
    }

    pub fn with_gyro_range(mut self, gyro_range: GyroRange) -> Self {
        self.gyro_sens = gyro_range.sensitivity();
        self
    }

    /// Wakes MPU6050 with all sensors enabled (default)
    async fn wake(&mut self) -> Result<(), Error<E>> {
        // MPU6050 has sleep enabled by default -> set bit 0 to wake
        // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001 (See Register Map )
        self.write_byte(PWR_MGMT_1::ADDR, 0x01).await?;
        Timer::after_millis(100).await;
        Ok(())
    }

    /// From Register map:
    /// "An  internal  8MHz  oscillator,  gyroscope based  clock,or  external  sources  can  be
    /// selected  as the MPU-60X0 clock source.
    /// When the internal 8 MHz oscillator or an external source is chosen as the clock source,
    /// the MPU-60X0 can operate in low power modes with the gyroscopes disabled. Upon power up,
    /// the MPU-60X0clock source defaults to the internal oscillator. However, it is highly
    /// recommended  that  the  device beconfigured  to  use  one  of  the  gyroscopes
    /// (or  an  external  clocksource) as the clock reference for improved stability.
    /// The clock source can be selected according to the following table...."
    pub async fn set_clock_source(&mut self, source: CLKSEL) -> Result<(), Error<E>> {
        Ok(self
            .write_bits(PWR_MGMT_1::ADDR, PWR_MGMT_1::CLKSEL, source as u8)
            .await?)
    }

    /// get current clock source
    pub async fn get_clock_source(&mut self) -> Result<CLKSEL, Error<E>> {
        let source = self.read_bits(PWR_MGMT_1::ADDR, PWR_MGMT_1::CLKSEL).await?;
        Ok(CLKSEL::from(source))
    }

    /// Init wakes MPU6050 and verifies register addr, e.g. in i2c
    pub async fn init(&mut self) -> Result<(), Error<E>> {
        self.wake().await?;
        self.verify().await?;
        self.set_accel_range(AccelRange::G2).await?;
        self.set_gyro_range(GyroRange::D250).await?;
        self.set_accel_hpf(ACCEL_HPF::_RESET).await?;

        self.init_mag().await
    }

    async fn init_mag(&mut self) -> Result<(), Error<E>> {
        // Set i2c master mode off mpu6050
        self.write_bit(INT_ENABLE::ADDR, INT_ENABLE::I2C_MST_INT_EN, false)
            .await?;
        // set i2c bypass mode on mpu6050
        self.write_bit(INT_PIN_CFG::ADDR, INT_PIN_CFG::I2C_BYPASS_EN, true)
            .await?;
        self.write_bit(0x6A, 5, false).await?;
        Timer::after_millis(500).await;
        // wake

        let id = self.read_byte_mag(qmc5883p::reg::CHIP_ID_REG).await?;

        if id != qmc5883p::reg::CHIP_ID_EXPECTED {
            return Err(Error::MagWrongId(id));
        }

        self.write_bit_mag(
            qmc5883p::reg::Control2::ADDR,
            qmc5883p::reg::Control2::SOFT_RESET,
            true,
        )
        .await?;
        Timer::after_millis(100).await;

        // self.write_byte_mag(qmc5883p::reg::Control1::ADDR, 0b1100_1111).await?;
        self.write_byte_mag(0x29, 0x06).await?;
        self.set_mag_mode(qmc5883p::Mode::Continuous).await?;
        self.write_bits_mag(
            qmc5883p::reg::Control1::ADDR,
            qmc5883p::reg::Control1::OUTPUT_DATA_RATE,
            qmc5883p::OutputDataRate::Rate200HZ as u8,
        )
        .await?;
        self.write_bits_mag(
            qmc5883p::reg::Control1::ADDR,
            qmc5883p::reg::Control1::OVER_SAMPLING,
            qmc5883p::OverSampling::OverSampling8 as u8,
        )
        .await?;
        self.write_bits_mag(
            qmc5883p::reg::Control1::ADDR,
            qmc5883p::reg::Control1::DOWN_SAMPLING,
            qmc5883p::DownSampling::DownSampling8 as u8,
        )
        .await?;
        self.write_bits_mag(
            qmc5883p::reg::Control2::ADDR,
            qmc5883p::reg::Control2::RANGE,
            qmc5883p::Range::Gauss30 as u8,
        )
        .await?;
        self.write_bits_mag(
            qmc5883p::reg::Control2::ADDR,
            qmc5883p::reg::Control2::SET_RESET_MODE,
            qmc5883p::SetResetMode::SetResetOn as u8,
        )
        .await?;
        self.write_bits_mag(
            qmc5883p::reg::Control2::ADDR,
            qmc5883p::reg::Control2::SET_RESET_MODE,
            qmc5883p::SetResetMode::SetResetOn as u8,
        )
        .await?;
        self.write_bit_mag(
            qmc5883p::reg::Control2::ADDR,
            qmc5883p::reg::Control2::SELF_TEST,
            false,
        )
        .await?;
        self.write_bit_mag(
            qmc5883p::reg::Control2::ADDR,
            qmc5883p::reg::Control2::SOFT_RESET,
            false,
        )
        .await?;

        Ok(())
    }

    async fn set_mag_mode(&mut self, mode: qmc5883p::Mode) -> Result<(), Error<E>> {
        self.write_bits_mag(
            qmc5883p::reg::Control1::ADDR,
            qmc5883p::reg::Control1::MODE,
            mode as u8,
        )
        .await
    }

    // async fn get_mag_mode(&mut self) -> Result<qmc5883p::Mode, Error<E>> {
    //     self.read_bits(qmc5883p::reg::Control1::ADDR, qmc5883p::reg::Control1::MODE)
    //         .await
    //         .map(|mode| mode.try_into().map_err(|_| Error::Conversion))
    // }

    async fn wait_mag_data_ready(&mut self) -> Result<(), Error<E>> {
        loop {
            if self
                .read_bit_mag(
                    qmc5883p::reg::Status::ADDR,
                    qmc5883p::reg::Status::DATA_READY as u8,
                )
                .await?
                == 1
            {
                return Ok(());
            }
        }
    }

    pub async fn mag_self_test(&mut self) -> Result<Vector3<i16>, Error<E>> {
        self.set_mag_mode(qmc5883p::Mode::Continuous).await?;

        with_timeout(Duration::from_millis(1_000), self.wait_mag_data_ready())
            .await
            .map_err(|_| Error::Timeout)??;

        let init = self.read_mag_raw().await?;

        let prev_mode = self
            .read_bits_mag(qmc5883p::reg::Control1::ADDR, qmc5883p::reg::Control1::MODE)
            .await?;

        self.write_bit_mag(
            qmc5883p::reg::Control2::ADDR,
            qmc5883p::reg::Control2::SELF_TEST,
            true,
        )
        .await?;

        Timer::after_millis(150).await;

        self.write_bit_mag(
            qmc5883p::reg::Control2::ADDR,
            qmc5883p::reg::Control2::SELF_TEST,
            false,
        )
        .await?;

        let self_test = self.read_mag_raw().await?;

        self.write_bits_mag(
            qmc5883p::reg::Control1::ADDR,
            qmc5883p::reg::Control1::MODE,
            prev_mode,
        )
        .await?;

        Ok(init - self_test)
    }

    /// Verifies device to address 0x68 with WHOAMI.addr() Register
    async fn verify(&mut self) -> Result<(), Error<E>> {
        let address = self.read_byte(WHOAMI_REG).await?;
        if address != DEFAULT_MPU6050_ADDR {
            return Err(Error::Mpu6050InvalidChipId(address));
        }
        Ok(())
    }

    /// setup motion detection
    /// sources:
    /// * https://github.com/kriswiner/MPU6050/blob/a7e0c8ba61a56c5326b2bcd64bc81ab72ee4616b/MPU6050IMU.ino#L486
    /// * https://arduino.stackexchange.com/a/48430
    pub async fn setup_motion_detection(&mut self) -> Result<(), Error<E>> {
        self.write_byte(0x6B, 0x00).await?;
        // optional? self.write_byte(0x68, 0x07).await?; // Reset all internal signal paths in the MPU-6050 by writing 0x07 to register 0x68;
        self.write_byte(INT_PIN_CFG::ADDR, 0x20).await?; //write register 0x37 to select how to use the interrupt pin. For an active high, push-pull signal that stays until register (decimal) 58 is read, write 0x20.
        self.write_byte(ACCEL_CONFIG::ADDR, 0x01).await?; //Write register 28 (==0x1C) to set the Digital High Pass Filter, bits 3:0. For example set it to 0x01 for 5Hz. (These 3 bits are grey in the data sheet, but they are used! Leaving them 0 means the filter always outputs 0.)
        self.write_byte(MOT_THR, 10).await?; //Write the desired Motion threshold to register 0x1F (For example, write decimal 20).
        self.write_byte(MOT_DUR, 40).await?; //Set motion detect duration to 1  ms; LSB is 1 ms @ 1 kHz rate
        self.write_byte(0x69, 0x15).await?; //to register 0x69, write the motion detection decrement and a few other settings (for example write 0x15 to set both free-fall and motion decrements to 1 and accelerometer start-up delay to 5ms total by adding 1ms. )
        self.write_byte(INT_ENABLE::ADDR, 0x40).await?; //write register 0x38, bit 6 (0x40), to enable motion detection interrupt.
        Ok(())
    }

    /// get whether or not motion has been detected (INT_STATUS, MOT_INT)
    pub async fn get_motion_detected(&mut self) -> Result<bool, Error<E>> {
        Ok(self.read_bit(INT_STATUS::ADDR, INT_STATUS::MOT_INT).await? != 0)
    }

    /// set accel high pass filter mode
    pub async fn set_accel_hpf(&mut self, mode: ACCEL_HPF) -> Result<(), Error<E>> {
        Ok(self
            .write_bits(ACCEL_CONFIG::ADDR, ACCEL_CONFIG::ACCEL_HPF, mode as u8)
            .await?)
    }

    /// get accel high pass filter mode
    pub async fn get_accel_hpf(&mut self) -> Result<ACCEL_HPF, Error<E>> {
        let mode: u8 = self
            .read_bits(ACCEL_CONFIG::ADDR, ACCEL_CONFIG::ACCEL_HPF)
            .await?;

        Ok(ACCEL_HPF::from(mode))
    }

    /// Set gyro range, and update sensitivity accordingly
    pub async fn set_gyro_range(&mut self, range: GyroRange) -> Result<(), Error<E>> {
        self.write_bits(GYRO_CONFIG::ADDR, GYRO_CONFIG::FS_SEL, range as u8)
            .await?;

        self.gyro_sens = range.sensitivity();
        Ok(())
    }

    /// get current gyro range
    pub async fn get_gyro_range(&mut self) -> Result<GyroRange, Error<E>> {
        let byte = self
            .read_bits(GYRO_CONFIG::ADDR, GYRO_CONFIG::FS_SEL)
            .await?;

        Ok(GyroRange::from(byte))
    }

    /// set accel range, and update sensitivy accordingly
    pub async fn set_accel_range(&mut self, range: AccelRange) -> Result<(), Error<E>> {
        self.write_bits(ACCEL_CONFIG::ADDR, ACCEL_CONFIG::FS_SEL, range as u8)
            .await?;

        self.accel_sens = range.sensitivity();
        Ok(())
    }

    /// get current accel_range
    pub async fn get_accel_range(&mut self) -> Result<AccelRange, Error<E>> {
        let byte = self
            .read_bits(ACCEL_CONFIG::ADDR, ACCEL_CONFIG::FS_SEL)
            .await?;

        Ok(AccelRange::from(byte))
    }

    /// reset device
    pub async fn reset_device<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Error<E>> {
        self.write_bit(PWR_MGMT_1::ADDR, PWR_MGMT_1::DEVICE_RESET, true)
            .await?;
        delay.delay_ms(100u32).await;
        // Note: Reset sets sleep to true! Section register map: resets PWR_MGMT to 0x40
        Ok(())
    }

    /// enable, disable i2c master interrupt
    pub async fn set_master_interrupt_enabled(&mut self, enable: bool) -> Result<(), Error<E>> {
        Ok(self
            .write_bit(INT_ENABLE::ADDR, INT_ENABLE::I2C_MST_INT_EN, enable)
            .await?)
    }

    /// get i2c master interrupt status
    pub async fn get_master_interrupt_enabled(&mut self) -> Result<bool, Error<E>> {
        Ok(self
            .read_bit(INT_ENABLE::ADDR, INT_ENABLE::I2C_MST_INT_EN)
            .await?
            != 0)
    }

    /// enable, disable bypass of sensor
    pub async fn set_bypass_enabled(&mut self, enable: bool) -> Result<(), Error<E>> {
        Ok(self
            .write_bit(INT_PIN_CFG::ADDR, INT_PIN_CFG::I2C_BYPASS_EN, enable)
            .await?)
    }

    /// get bypass status
    pub async fn get_bypass_enabled(&mut self) -> Result<bool, Error<E>> {
        Ok(self
            .read_bit(INT_PIN_CFG::ADDR, INT_PIN_CFG::I2C_BYPASS_EN)
            .await?
            != 0)
    }

    /// enable, disable sleep of sensor
    pub async fn set_sleep_enabled(&mut self, enable: bool) -> Result<(), Error<E>> {
        Ok(self
            .write_bit(PWR_MGMT_1::ADDR, PWR_MGMT_1::SLEEP, enable)
            .await?)
    }

    /// get sleep status
    pub async fn get_sleep_enabled(&mut self) -> Result<bool, Error<E>> {
        Ok(self.read_bit(PWR_MGMT_1::ADDR, PWR_MGMT_1::SLEEP).await? != 0)
    }

    /// enable, disable temperature measurement of sensor
    /// TEMP_DIS actually saves "disabled status"
    /// 1 is disabled! -> enable=true : bit=!enable
    pub async fn set_temp_enabled(&mut self, enable: bool) -> Result<(), Error<E>> {
        Ok(self
            .write_bit(PWR_MGMT_1::ADDR, PWR_MGMT_1::TEMP_DIS, !enable)
            .await?)
    }

    /// get temperature sensor status
    /// TEMP_DIS actually saves "disabled status"
    /// 1 is disabled! -> 1 == 0 : false, 0 == 0 : true
    pub async fn get_temp_enabled(&mut self) -> Result<bool, Error<E>> {
        Ok(self
            .read_bit(PWR_MGMT_1::ADDR, PWR_MGMT_1::TEMP_DIS)
            .await?
            == 0)
    }

    /// set accel x self test
    pub async fn set_accel_x_self_test(&mut self, enable: bool) -> Result<(), Error<E>> {
        Ok(self
            .write_bit(ACCEL_CONFIG::ADDR, ACCEL_CONFIG::XA_ST, enable)
            .await?)
    }

    /// get accel x self test
    pub async fn get_accel_x_self_test(&mut self) -> Result<bool, Error<E>> {
        Ok(self
            .read_bit(ACCEL_CONFIG::ADDR, ACCEL_CONFIG::XA_ST)
            .await?
            != 0)
    }

    /// set accel y self test
    pub async fn set_accel_y_self_test(&mut self, enable: bool) -> Result<(), Error<E>> {
        Ok(self
            .write_bit(ACCEL_CONFIG::ADDR, ACCEL_CONFIG::YA_ST, enable)
            .await?)
    }

    /// get accel y self test
    pub async fn get_accel_y_self_test(&mut self) -> Result<bool, Error<E>> {
        Ok(self
            .read_bit(ACCEL_CONFIG::ADDR, ACCEL_CONFIG::YA_ST)
            .await?
            != 0)
    }

    /// set accel z self test
    pub async fn set_accel_z_self_test(&mut self, enable: bool) -> Result<(), Error<E>> {
        Ok(self
            .write_bit(ACCEL_CONFIG::ADDR, ACCEL_CONFIG::ZA_ST, enable)
            .await?)
    }

    /// get accel z self test
    pub async fn get_accel_z_self_test(&mut self) -> Result<bool, Error<E>> {
        Ok(self
            .read_bit(ACCEL_CONFIG::ADDR, ACCEL_CONFIG::ZA_ST)
            .await?
            != 0)
    }

    /// Roll and pitch estimation from raw accelerometer readings
    /// NOTE: no yaw! no magnetometer present on MPU6050
    /// https://www.nxp.com/docs/en/application-note/AN3461.pdf equation 28, 29
    pub async fn get_acc_angles(&mut self) -> Result<(f32, f32), Error<E>> {
        let acc = self.get_acc().await?;

        Ok((
            acc.y.atan2((acc.x.powf(2.0) + acc.z.powf(2.0)).sqrt()),
            acc.x.neg().atan2((acc.y.powf(2.) + acc.z.powf(2.)).sqrt()),
        ))
    }

    /// Converts 2 bytes number in 2 compliment
    /// TODO i16?! whats 0x8000?!
    fn read_word_2c(&self, byte: &[u8]) -> i32 {
        let high: i32 = byte[0] as i32;
        let low: i32 = byte[1] as i32;
        let mut word: i32 = (high << 8) + low;

        if word >= 0x8000 {
            word = -((65535 - word) + 1);
        }

        word
    }

    /// Reads rotation (gyro/acc) from specified register
    async fn read_rot(&mut self, reg: u8) -> Result<Vector3<f32>, Error<E>> {
        let mut buf: [u8; 6] = [0; 6];
        self.read_bytes(reg, &mut buf).await?;

        Ok(Vector3::<f32>::new(
            self.read_word_2c(&buf[0..2]) as f32,
            self.read_word_2c(&buf[2..4]) as f32,
            self.read_word_2c(&buf[4..6]) as f32,
        ))
    }

    /// Accelerometer readings in g
    pub async fn get_acc(&mut self) -> Result<Vector3<f32>, Error<E>> {
        let mut acc = self.read_rot(ACC_REGX_H).await?;
        acc /= self.accel_sens;

        Ok(acc)
    }

    /// Gyro readings in rad/s
    pub async fn get_gyro(&mut self) -> Result<Vector3<f32>, Error<E>> {
        let mut gyro = self.read_rot(GYRO_REGX_H).await?;

        gyro *= PI_180 / self.gyro_sens;

        Ok(gyro)
    }

    /// Sensor Temp in degrees celcius
    pub async fn get_imu_temp(&mut self) -> Result<f32, Error<E>> {
        let mut buf: [u8; 2] = [0; 2];
        self.read_bytes(TEMP_OUT_H, &mut buf).await?;
        let raw_temp = self.read_word_2c(&buf[0..2]) as f32;

        // According to revision 4.2
        Ok((raw_temp / TEMP_SENSITIVITY) + TEMP_OFFSET)
    }

    pub async fn read_mag_raw(&mut self) -> Result<Vector3<i16>, Error<E>> {
        let mut data = [0u8; 6];
        self.read_bytes_mag(qmc5883p::reg::DATA_REG, &mut data)
            .await?;

        Ok(Vector3::new(
            i16::from_le_bytes([data[0], data[1]]), // X (little-endian)
            i16::from_le_bytes([data[2], data[3]]), // Y
            i16::from_le_bytes([data[4], data[5]]), // Z
        ))
    }

    pub async fn read_mag_gauss(&mut self) -> Result<Vector3<f32>, Error<E>> {
        let raw = self.read_mag_raw().await?;
        let range = self
            .read_bits(
                qmc5883p::reg::Control2::ADDR,
                qmc5883p::reg::Control2::RANGE,
            )
            .await?;
        let range = qmc5883p::Range::from_u8(range).ok_or(Error::Conversion)?;

        Ok(raw.map(|raw| raw as f32) / range.lsb_per_gauss())
    }

    /// Writes byte to register
    pub async fn write_byte(&mut self, reg: u8, byte: u8) -> Result<(), Error<E>> {
        self.i2c
            .write(self.addr, &[reg, byte])
            .await
            .map_err(Error::I2c)?;
        // delay disabled for dev build
        // TODO: check effects with physical unit
        // self.delay.delay_ms(10u8);
        Ok(())
    }

    pub async fn write_byte_mag(&mut self, reg: u8, byte: u8) -> Result<(), Error<E>> {
        self.i2c
            .write(self.mag_addr, &[reg, byte])
            .await
            .map_err(Error::I2c)?;

        Ok(())
    }

    /// Enables bit n at register address reg
    pub async fn write_bit(&mut self, reg: u8, bit_n: u8, enable: bool) -> Result<(), Error<E>> {
        let mut byte: [u8; 1] = [0; 1];
        self.read_bytes(reg, &mut byte).await?;
        bits::set_bit(&mut byte[0], bit_n, enable);
        Ok(self.write_byte(reg, byte[0]).await?)
    }

    pub async fn write_bit_mag(
        &mut self,
        reg: u8,
        bit_n: u8,
        enable: bool,
    ) -> Result<(), Error<E>> {
        let mut byte: [u8; 1] = [0; 1];
        self.read_bytes_mag(reg, &mut byte).await?;
        bits::set_bit(&mut byte[0], bit_n, enable);
        Ok(self.write_byte_mag(reg, byte[0]).await?)
    }

    /// Write bits data at reg from start_bit to start_bit+length
    pub async fn write_bits(&mut self, reg: u8, block: BitBlock, data: u8) -> Result<(), Error<E>> {
        let mut byte: [u8; 1] = [0; 1];
        self.read_bytes(reg, &mut byte).await?;
        bits::set_bits(&mut byte[0], block.bit, block.length, data);
        Ok(self.write_byte(reg, byte[0]).await?)
    }

    pub async fn write_bits_mag(
        &mut self,
        reg: u8,
        block: BitBlock,
        data: u8,
    ) -> Result<(), Error<E>> {
        let mut byte: [u8; 1] = [0; 1];
        self.read_bytes_mag(reg, &mut byte).await?;
        bits::set_bits(&mut byte[0], block.bit, block.length, data);
        Ok(self.write_byte_mag(reg, byte[0]).await?)
    }

    /// Read bit n from register
    async fn read_bit(&mut self, reg: u8, bit_n: u8) -> Result<u8, Error<E>> {
        let mut byte: [u8; 1] = [0; 1];
        self.read_bytes(reg, &mut byte).await?;
        Ok(bits::get_bit(byte[0], bit_n))
    }

    async fn read_bit_mag(&mut self, reg: u8, bit_n: u8) -> Result<u8, Error<E>> {
        let mut byte: [u8; 1] = [0; 1];
        self.read_bytes_mag(reg, &mut byte).await?;
        Ok(bits::get_bit(byte[0], bit_n))
    }

    /// Read bits at register reg, starting with bit start_bit, until start_bit+length
    pub async fn read_bits(&mut self, reg: u8, block: BitBlock) -> Result<u8, Error<E>> {
        let mut byte: [u8; 1] = [0; 1];
        self.read_bytes(reg, &mut byte).await?;
        Ok(bits::get_bits(byte[0], block.bit, block.length))
    }

    pub async fn read_bits_mag(&mut self, reg: u8, block: BitBlock) -> Result<u8, Error<E>> {
        let mut byte: [u8; 1] = [0; 1];
        self.read_bytes_mag(reg, &mut byte).await?;
        Ok(bits::get_bits(byte[0], block.bit, block.length))
    }

    /// Reads byte from register
    pub async fn read_byte(&mut self, reg: u8) -> Result<u8, Error<E>> {
        let mut byte: [u8; 1] = [0; 1];
        self.i2c
            .write_read(self.addr, &[reg], &mut byte)
            .await
            .map_err(Error::I2c)?;
        Ok(byte[0])
    }

    pub async fn read_byte_mag(&mut self, reg: u8) -> Result<u8, Error<E>> {
        let mut byte: [u8; 1] = [0; 1];
        self.i2c
            .write_read(self.mag_addr, &[reg], &mut byte)
            .await
            .map_err(Error::I2c)?;
        Ok(byte[0])
    }

    /// Reads series of bytes into buf from specified reg
    pub async fn read_bytes(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<E>> {
        self.i2c
            .write_read(self.addr, &[reg], buf)
            .await
            .map_err(Error::I2c)?;
        Ok(())
    }

    pub async fn read_bytes_mag(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<E>> {
        self.i2c
            .write_read(self.mag_addr, &[reg], buf)
            .await
            .map_err(Error::I2c)?;
        Ok(())
    }
}
