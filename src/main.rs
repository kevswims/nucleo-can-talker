#![no_std]
#![no_main]

// pick a panicking behavior
extern crate panic_halt; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// extern crate panic_abort; // requires nightly
// extern crate panic_itm; // logs messages over ITM; requires ITM support
// extern crate panic_semihosting; // logs messages to the host stderr; requires a debugger

use cortex_m::asm;
use cortex_m_rt::entry;
use stm32f4xx_hal as hal;

// use stm32f4::stm32f413;

use crate::hal::{
    time::Bps,
    prelude::*,
    stm32};

#[entry]
fn main() -> ! {
    if let (Some(dp), Some(cp)) = (
        stm32::Peripherals::take(),
        cortex_m::peripheral::Peripherals::take(),
    ) {
        let gpiob = dp.GPIOB.split();
        let mut led = gpiob.pb7.into_push_pull_output();

        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(48.mhz()).freeze();

        let mut delay = hal::delay::Delay::new(cp.SYST, clocks);


        let gpiod = dp.GPIOD.split();
        let bps = Bps(115200);

        logger::configure(dp.USART3, gpiod.pd8, gpiod.pd9, bps, clocks);

        loop {
            led.set_high().unwrap();
            delay.delay_ms(1000_u32);
            led.set_low().unwrap();
            delay.delay_ms(1000_u32);
        }
    }

    loop {
        // your code goes here
    }
}

use stm32f4xx_hal::{
    serial::{Serial, Tx},
    stm32::USART3,
    prelude::*,
};
use stm32f4xx_hal::gpio::gpiod::{PD8, PD9};
use stm32f4xx_hal::serial::config::Config;
use stm32f4xx_hal::rcc::Clocks;

pub mod logger {
    // USART3_TX is on PD8
    // USART3_RX is on PD9

    pub fn configure<X, Y>(
        uart: USART3, tx: PD8<X>, rx: PD9<Y>,
        baudrate: Bps, clocks: Clocks
    ) {
        let config = Config {
            baudrate,
            ..Config::default()
        };

        let tx = tx.into_alternate_af7();
        let rx = rx.into_alternate_af7();
    }
}