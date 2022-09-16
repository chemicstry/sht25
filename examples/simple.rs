#![no_main]
#![no_std]

use cortex_m_rt::entry;
use defmt::*;
use sht25::{Resolution, Sht25};
use stm32f4xx_hal::pac::Peripherals;
use stm32f4xx_hal::prelude::*;
use {defmt_rtt as _, panic_probe as _};

#[entry]
fn main() -> ! {
    info!("Hello world!");

    let dp = Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // Set up the system clock. We want to run at 48MHz for this one.
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(48.MHz()).freeze();

    let gpioa = dp.GPIOA.split();
    let gpioc = dp.GPIOC.split();
    let scl = gpioa.pa8.into_alternate().internal_pull_up(true).set_open_drain();
    let sda = gpioc.pc9.into_alternate().internal_pull_up(true).set_open_drain();

    let i2c = dp.I2C3.i2c((scl, sda), 400.kHz(), &clocks);

    let delay = cp.SYST.delay(&clocks);

    let mut sht25 = Sht25::new(i2c, delay).unwrap();

    info!("Resetting SHT25");
    sht25.reset_blocking().unwrap();

    info!(
        "Default resolution: {:?}",
        Debug2Format(&sht25.get_resolution().unwrap())
    );

    let res = Resolution::Rh11T11;
    sht25.set_resolution(res).unwrap();
    info!("New resolution: {:?}", Debug2Format(&sht25.get_resolution().unwrap()));

    // Blocking temp measurement
    let temp = sht25.read_temp_blocking().unwrap();
    info!("Temp: {} C", HundredthsFormat(temp));

    // Blocking RH measurement
    let rh = sht25.read_rh_blocking().unwrap();
    info!("RH: {} %", HundredthsFormat(rh));

    let mut delay = dp.TIM5.delay_ms(&clocks);

    // Async temp measurement
    sht25.trigger_temp_measurement().unwrap();
    delay.delay_ms(res.temp_measure_time_ms()); // Can do other stuff here
    let temp = sht25.read_temp().unwrap();
    info!("Async temp: {} C", HundredthsFormat(temp));

    // Async RH measurement
    sht25.trigger_rh_measurement().unwrap();
    delay.delay_ms(res.rh_measure_time_ms()); // Can do other stuff here
    let rh = sht25.read_rh().unwrap();
    info!("Async RH: {} %", HundredthsFormat(rh));

    loop {}
}

struct HundredthsFormat(i32);

impl Format for HundredthsFormat {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "{}.{}", self.0 / 100, self.0 % 100);
    }
}
