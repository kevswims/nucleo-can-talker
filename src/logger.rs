use stm32f4xx_hal::{
    serial::{Serial, Tx},
    stm32::USART3,
    prelude::*,
    time::Bps
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