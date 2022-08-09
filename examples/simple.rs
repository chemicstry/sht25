#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::{info, Debug2Format, Format};
use embassy_embedded_hal::adapter::BlockingAsync;
use embassy_executor::executor::Spawner;
use embassy_executor::time::{Delay, Duration, Timer};
use embassy_stm32::i2c::I2c;
use embassy_stm32::time::khz;
use embassy_stm32::Peripherals;
use sht25_rs::{Resolution, Sht25};
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner, p: Peripherals) -> ! {
    info!("Hello SHT25!");

    let i2c = I2c::new(p.I2C2, p.PF1, p.PF0, khz(100));

    // I2Cv1 in embassy does not support async yet, so use adapter
    let async_i2c = BlockingAsync::new(i2c);

    let mut sht25 = Sht25::new(async_i2c, Delay).await.unwrap();

    info!(
        "Default resolution: {:?}",
        Debug2Format(&sht25.get_resolution().await.unwrap())
    );

    sht25.set_resolution(Resolution::Rh11T11).await.unwrap();

    info!(
        "New resolution: {:?}",
        Debug2Format(&sht25.get_resolution().await.unwrap())
    );

    Timer::after(Duration::from_millis(100)).await;

    loop {
        let temp = sht25.read_temp().await.unwrap();
        info!("Temp: {} C", HundredthsFormat(temp));
        let rh = sht25.read_rh().await.unwrap();
        info!("RH: {} %", HundredthsFormat(rh));
        Timer::after(Duration::from_secs(1)).await;
    }
}

struct HundredthsFormat(i32);

impl Format for HundredthsFormat {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "{}.{}", self.0 / 100, self.0 % 100);
    }
}