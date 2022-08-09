#![no_std]

use embedded_hal_async::i2c::Error as _;

pub const SHT25_ADDRESS: u8 = 0x40;

// These two use clock stretching
// const CMD_TRIGGER_T_MEASUREMENT_HOLD: u8 = 0b1110_0011;
// const CMD_TRIGGER_RH_MEASUREMENT_HOLD: u8 = 0b1110_0101;
const CMD_TRIGGER_T_MEASUREMENT_NOHOLD: u8 = 0b1111_0011;
const CMD_TRIGGER_RH_MEASUREMENT_NOHOLD: u8 = 0b1111_0101;
const CMD_WRITE_USER_REG: u8 = 0b1110_0110;
const CMD_READ_USER_REG: u8 = 0b1110_0111;
const CMD_SOFT_RESET: u8 = 0b1111_1110;

const SOFT_RESET_TIME_MS: u32 = 15;

#[derive(Debug)]
pub enum Error<I2CError, DelayError> {
    /// I2C bus error
    I2c(I2CError),
    /// Delay error
    Delay(DelayError),
    /// CRC Error,
    Crc,
}

#[derive(Debug)]
pub enum Resolution {
    /// 12 bit relative humidity and 14 bit temperature resolution
    Rh12T14,
    /// 8 bit relative humidity and 12 bit temperature resolution
    Rh8T12,
    /// 10 bit relative humidity and 13 bit temperature resolution
    Rh10T13,
    /// 11 bit relative humidity and 11 bit temperature resolution
    Rh11T11,
}

impl Resolution {
    /// Maximum relative humidity measurement time in ms
    pub fn rh_measure_time_ms(&self) -> u32 {
        match self {
            Resolution::Rh12T14 => 29,
            Resolution::Rh8T12 => 4,
            Resolution::Rh10T13 => 9,
            Resolution::Rh11T11 => 15,
        }
    }

    /// Maximum temperature measurement time in ms
    pub fn temp_measure_time_ms(&self) -> u32 {
        match self {
            Resolution::Rh12T14 => 85,
            Resolution::Rh8T12 => 22,
            Resolution::Rh10T13 => 43,
            Resolution::Rh11T11 => 11,
        }
    }

    /// Read resolution from user register value
    fn from_user_reg(reg: u8) -> Self {
        match reg & 0b1000_0001 {
            0b0000_0000 => Self::Rh12T14,
            0b0000_0001 => Self::Rh8T12,
            0b1000_0000 => Self::Rh10T13,
            0b1000_0001 => Self::Rh11T11,
            _ => unreachable!(),
        }
    }

    /// Write resolution into user register without changing other bits
    fn to_user_reg(&self, reg: &mut u8) {
        let val = match self {
            Resolution::Rh12T14 => 0b0000_0000,
            Resolution::Rh8T12 => 0b0000_0001,
            Resolution::Rh10T13 => 0b1000_0000,
            Resolution::Rh11T11 => 0b1000_0001,
        };
        *reg = (*reg & 0b0111_1110) | val;
    }
}

impl<I2CError, DelayError> From<I2CError> for Error<I2CError, DelayError> {
    fn from(i2c_err: I2CError) -> Self {
        Self::I2c(i2c_err)
    }
}

#[derive(Debug)]
pub struct Sht25<I2C, Delay> {
    i2c: I2C,
    delay: Delay,
    // Should be kept in sync with device register to avoid reading on each measurement
    resolution: Resolution,
}

impl<I2C, Delay> Sht25<I2C, Delay>
where
    I2C: embedded_hal_async::i2c::I2c,
    Delay: embedded_hal_async::delay::DelayUs,
{
    /// Constructs SHT25 sensor instance and performs soft-reset of the sensor
    pub async fn new(i2c: I2C, delay: Delay) -> Result<Self, Error<I2C::Error, Delay::Error>> {
        let mut sht = Self {
            i2c,
            delay,
            resolution: Resolution::Rh12T14,
        };
        sht.reset().await?;
        Ok(sht)
    }

    /// Performs a sensor soft reset. Function asynchronously waits 15ms until
    /// sensor reset is complete.
    pub async fn reset(&mut self) -> Result<(), Error<I2C::Error, Delay::Error>> {
        match self.i2c.write(SHT25_ADDRESS, &[CMD_SOFT_RESET]).await {
            Ok(_) => {}
            Err(e) => match e.kind() {
                embedded_hal_async::i2c::ErrorKind::NoAcknowledge(_) => {
                    // For some reason first reset might fail with NACK so retry a second time
                    self.i2c.write(SHT25_ADDRESS, &[CMD_SOFT_RESET]).await?
                }
                _ => return Err(e.into()),
            },
        };

        self.delay_ms(SOFT_RESET_TIME_MS).await?;
        self.resolution = Resolution::Rh12T14;
        Ok(())
    }

    /// Returns temperature in hundredths of a degree C. For example, to get whole degrees, divide by 100.
    ///
    /// This function works by triggering measurement, asynchronously waiting for the maximum measurement time acording to current resolution and then finally reading the measurement.
    pub async fn read_temp(&mut self) -> Result<i32, Error<I2C::Error, Delay::Error>> {
        self.i2c
            .write(SHT25_ADDRESS, &[CMD_TRIGGER_T_MEASUREMENT_NOHOLD])
            .await?;
        self.delay_ms(self.resolution.temp_measure_time_ms()).await?;

        let mut buf = [0u8; 3];
        self.i2c.read(SHT25_ADDRESS, &mut buf).await?;

        let res = Measurement::from_buf(buf).ok_or(Error::Crc)?;
        assert!(res.is_rh == false);

        Ok(Self::convert_temp(res.sample))
    }

    /// Returns relative humidity in hundredths of a percent. For example, to get whole percent, divide by 100.
    ///
    /// This function works by triggering measurement, asynchronously waiting for the maximum measurement time acording to current resolution and then finally reading the measurement.
    pub async fn read_rh(&mut self) -> Result<i32, Error<I2C::Error, Delay::Error>> {
        self.i2c
            .write(SHT25_ADDRESS, &[CMD_TRIGGER_RH_MEASUREMENT_NOHOLD])
            .await?;
        self.delay_ms(self.resolution.rh_measure_time_ms()).await?;

        let mut buf = [0u8; 3];
        self.i2c.read(SHT25_ADDRESS, &mut buf).await?;

        let res = Measurement::from_buf(buf).ok_or(Error::Crc)?;
        assert!(res.is_rh == true);

        Ok(Self::convert_rh(res.sample))
    }

    /// Returns current sensor resolution
    pub async fn get_resolution(&mut self) -> Result<Resolution, Error<I2C::Error, Delay::Error>> {
        let reg = self.read_user_reg().await?;
        Ok(Resolution::from_user_reg(reg))
    }

    /// Sets sensor resolution
    pub async fn set_resolution(&mut self, res: Resolution) -> Result<(), Error<I2C::Error, Delay::Error>> {
        let mut reg = self.read_user_reg().await?;
        res.to_user_reg(&mut reg);

        self.write_user_reg(reg).await?;
        self.resolution = res;
        Ok(())
    }

    async fn read_user_reg(&mut self) -> Result<u8, Error<I2C::Error, Delay::Error>> {
        self.i2c.write(SHT25_ADDRESS, &[CMD_READ_USER_REG]).await?;

        let mut val = [0; 1];
        self.i2c.read(SHT25_ADDRESS, &mut val).await?;

        Ok(val[0])
    }

    async fn write_user_reg(&mut self, val: u8) -> Result<(), Error<I2C::Error, Delay::Error>> {
        self.i2c.write(SHT25_ADDRESS, &[CMD_WRITE_USER_REG, val]).await.ok();
        Ok(())
    }

    /// Converts temperature to hundredths of degrees (divide by 100 to get C)
    fn convert_temp(sample: u16) -> i32 {
        -4685 + 17572 * (sample as i32) / 0xFFFF
    }

    /// Converts temperature to hundredths of percent (divide by 100 to get %)
    fn convert_rh(sample: u16) -> i32 {
        -600 + 12500 * (sample as i32) / 0xFFFF
    }

    async fn delay_ms(&mut self, ms: u32) -> Result<(), Error<I2C::Error, Delay::Error>> {
        self.delay.delay_ms(ms).await.map_err(|e| Error::Delay(e))
    }
}

#[derive(Debug)]
struct Measurement {
    sample: u16,
    is_rh: bool,
}

impl Measurement {
    pub fn from_buf(buf: [u8; 3]) -> Option<Self> {
        let is_rh = buf[1] & 0b10 != 0;
        let sample = ((buf[0] as u16) << 8) + (buf[1] & 0b11111100) as u16;

        if buf[2] == crc(&buf[..2]) {
            Some(Self { sample, is_rh })
        } else {
            None
        }
    }
}

fn crc(data: &[u8]) -> u8 {
    const POLY: u8 = 0x31;
    let mut crc = 0;

    for b in data {
        crc ^= b;

        for _ in 0..8 {
            if crc & 0x80 != 0 {
                crc = (crc << 1) ^ POLY;
            } else {
                crc <<= 1;
            }
        }
    }

    crc
}
