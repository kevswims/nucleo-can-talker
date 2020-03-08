#![no_std]
#![no_main]

// pick a panicking behavior
extern crate panic_halt; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// extern crate panic_abort; // requires nightly
// extern crate panic_itm; // logs messages over ITM; requires ITM support
// extern crate panic_semihosting; // logs messages to the host stderr; requires a debugger

use cortex_m_rt::entry;
use stm32f4xx_hal as hal;

use stm32f4xx_hal::gpio::gpiod::{PD8, PD9};
use crate::hal::serial::config::Config;
use crate::hal::rcc::Clocks;
use stm32f4xx_hal::stm32::USART3;
// use stm32f4::stm32f413::USART3;

use crate::hal::{
    serial::{Serial, Tx},
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

        // logger::configure(dp.USART3, gpiod.pd8, gpiod.pd9, bps, clocks);
        configure(dp.USART3, gpiod.pd8, gpiod.pd9, bps, clocks);

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


// pub fn configure<X, Y>(
//     uart: USART3, tx: PD8<X>, rx: PD9<Y>,
//     baudrate: Bps, clocks: Clocks
// ) {
pub fn configure<X, Y>(uart: USART3, tx: PD8<X>, rx: PD9<Y>, baudrate: Bps, clocks: Clocks) {
    let config = Config {
        baudrate,
        ..Config::default()
    };

    let tx = tx.into_alternate_af7();
    let rx = rx.into_alternate_af7();
    let serial = Serial::usart3(uart, (tx, rx), config, clocks).unwrap();
    let (mut tx, _) = serial.split();

    tx.write(b'a');
}
// pub mod logger {
//     // use stm32f4xx_hal::gpio::gpiod::{PD8, PD9};
//     // USART3_TX is on PD8
//     // USART3_RX is on PD9
// 
//     // pub fn configure<X, Y>(
//     //     uart: USART3, tx: PD8<X>, rx: PD9<Y>,
//     //     baudrate: Bps, clocks: Clocks
//     // ) {
//     pub fn configure(tx: PD8) {
//         // let config = Config {
//         //     baudrate,
//         //     ..Config::default()
//         // };
// 
//         let tx = tx.into_alternate_af7();
//         // let rx = rx.into_alternate_af7();
//     }
// }