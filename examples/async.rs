#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::i2c::I2c;
use embassy_stm32::time::khz;
use embassy_time::{Delay, Duration, Timer};
use sht25::{Resolution, Sht25};
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    error!("Hello SHT25!");

    let p = embassy_stm32::init(Default::default());

    let i2c = I2c::new(p.I2C3, p.PA8, p.PC9, khz(100), Default::default());

    let mut sht25 = Sht25::new(i2c, Delay).unwrap();

    sht25.reset().unwrap();
    Timer::after(Duration::from_millis(sht25::SOFT_RESET_TIME_MS.into())).await;

    info!("Rresolution: {:?}", Debug2Format(&sht25.get_resolution().unwrap()));

    let resolution = Resolution::Rh11T11;
    sht25.set_resolution(resolution).unwrap();
    info!("New resolution: {:?}", Debug2Format(&sht25.get_resolution().unwrap()));

    loop {
        // Blocking temperture measurement
        let temp = sht25.read_temp_blocking().unwrap();
        info!("Blocking temp: {} C", HundredthsFormat(temp));

        // Blocking RH measurement
        let temp = sht25.read_rh_blocking().unwrap();
        info!("Blocking RH: {} C", HundredthsFormat(temp));

        // Async temperature measurement
        sht25.trigger_temp_measurement().unwrap();
        Timer::after(Duration::from_millis(resolution.temp_measure_time_ms().into())).await;
        let temp = sht25.read_temp().unwrap();
        info!("Async temp: {} C", HundredthsFormat(temp));

        // Async RH measurement
        sht25.trigger_rh_measurement().unwrap();
        Timer::after(Duration::from_millis(resolution.rh_measure_time_ms().into())).await;
        let rh = sht25.read_rh().unwrap();
        info!("Async RH: {} %", HundredthsFormat(rh));

        Timer::after(Duration::from_secs(1)).await;
    }
}

struct HundredthsFormat(i32);

impl Format for HundredthsFormat {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "{}.{}", self.0 / 100, self.0 % 100);
    }
}
