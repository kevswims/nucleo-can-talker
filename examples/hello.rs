//! Prints "Hello, world!" on the host console using semihosting

#![no_main]
#![no_std]

extern crate panic_halt;

use cortex_m_rt::entry;
use cortex_m_semihosting::{debug, hprintln};
use stm32f4::stm32f413;

#[entry]
fn main() -> ! {
    let peripherals = stm32f413::Peripherals::take().unwrap();
    
    hprintln!("Hello, world!").unwrap();

    // exit QEMU
    // NOTE do not run this on hardware; it can corrupt OpenOCD state
    // debug::exit(debug::EXIT_SUCCESS);

    loop {}
}
