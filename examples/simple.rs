#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use hal::{i2c::I2c, pac, prelude::*};
use panic_probe as _;
use stm32f4xx_hal as hal;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    info!("Hello world!");

    // Set up the system clock. We want to run at 48MHz for this one.
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(48.MHz()).freeze();

    // Set up I2C - SCL is PB8 and SDA is PB9; they are set to Alternate Function 4
    // as per the STM32F446xC/E datasheet page 60. Pin assignment as per the Nucleo-F446 board.
    let gpiof = dp.GPIOF.split();
    let scl = gpiof
        .pf1
        .into_alternate()
        .internal_pull_up(true)
        .set_open_drain();
    let sda = gpiof
        .pf0
        .into_alternate()
        .internal_pull_up(true)
        .set_open_drain();

    let mut i2c = I2c::new(dp.I2C2, (scl, sda), 400.kHz(), &clocks);

    info!("Testing i2c");
    for address in 0..127 {
        let mut buf = [0u8; 1];
        match i2c.read(address, &mut buf) {
            Ok(_) => info!("i2c alive at: {}", address),
            Err(_) => {}
        }
    }
    info!("done");

    defmt::unreachable!()
}
