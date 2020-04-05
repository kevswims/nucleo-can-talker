#![no_std]
#![no_main]

// pick a panicking behavior
// extern crate panic_halt; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// extern crate panic_abort; // requires nightly
// extern crate panic_itm; // logs messages over ITM; requires ITM support
extern crate panic_semihosting; // logs messages to the host stderr; requires a debugger

#[macro_use(block)]
extern crate nb;

use numtoa::NumToA;

use cortex_m_rt::entry;
use stm32f4xx_hal as hal;

use crate::hal::rcc::Clocks;
use crate::hal::serial::config::Config;
use stm32f4;
use stm32f4xx_hal::gpio::gpiod::{PD0, PD1, PD8, PD9};
use stm32f4xx_hal::stm32::USART3;
// use stm32f4::stm32f413::USART3;

use crate::hal::{prelude::*, serial::Serial, stm32, time::Bps};

#[entry]
fn main() -> ! {
    if let (Some(dp), Some(cp)) = (
        stm32::Peripherals::take(),
        cortex_m::peripheral::Peripherals::take(),
    ) {
        let gpiob = dp.GPIOB.split();
        let mut led = gpiob.pb7.into_push_pull_output();

        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(100.mhz()).freeze();

        let mut delay = hal::delay::Delay::new(cp.SYST, clocks);

        let gpiod = dp.GPIOD.split();
        let bps = Bps(115200);

        let mut tx = configure(dp.USART3, gpiod.pd8, gpiod.pd9, bps, clocks);

        let mut buffer = [0u8; 20];
        write_string_to_serial(&mut tx, "AHB1: ");
        (clocks.hclk().0 / 1000000).numtoa_str(10, &mut buffer);
        write_bytes_to_serial(&mut tx, &buffer);
        write_string_to_serial(&mut tx, "\n");

        let mut buffer = [0u8; 20];
        write_string_to_serial(&mut tx, "APB1: ");
        (clocks.pclk1().0 / 1000000).numtoa_str(10, &mut buffer);
        write_bytes_to_serial(&mut tx, &buffer);
        write_string_to_serial(&mut tx, "\n");

        let mut buffer = [0u8; 20];
        write_string_to_serial(&mut tx, "APB2: ");
        (clocks.pclk2().0 / 1000000).numtoa_str(10, &mut buffer);
        write_bytes_to_serial(&mut tx, &buffer);
        write_string_to_serial(&mut tx, "\n");

        let mut buffer = [0u8; 20];
        write_string_to_serial(&mut tx, "APB1 Prescaler: ");
        clocks.ppre1().numtoa_str(10, &mut buffer);
        write_bytes_to_serial(&mut tx, &buffer);
        write_string_to_serial(&mut tx, "\n");

        let mut buffer = [0u8; 20];
        write_string_to_serial(&mut tx, "APB2 Prescaler: ");
        clocks.ppre2().numtoa_str(10, &mut buffer);
        write_bytes_to_serial(&mut tx, &buffer);
        write_string_to_serial(&mut tx, "\n");

        let mut buffer = [0u8; 20];
        write_string_to_serial(&mut tx, "System Frequency: ");
        (clocks.sysclk().0 / 1000000).numtoa_str(10, &mut buffer);
        write_bytes_to_serial(&mut tx, &buffer);
        write_string_to_serial(&mut tx, "\n");

        // let clock_info = format!("AHB1: {:?}", clocks.hclk());
        // let clock_info = format!("AHB1: {}", 100);

        ///////////////////////////////////////////////////////////////////////
        // Pin Setup
        //////////////////////////////////////////////////////////////////////
        // Use PD0 RX, PD1 TX
        setup_can_gpio(gpiod.pd0, gpiod.pd1);

        let rcc = unsafe { &(*stm32::RCC::ptr()) };

        // Enable the clock for the can peripheral
        rcc.apb1enr.modify(|_, w| w.can3en().set_bit());

        // Need to figure out if there is a safe way to grab this peripheral
        let can3 = unsafe { &(*stm32::CAN3::ptr()) };

        // Switch hardware into initialization mode.
        can3.mcr
            .modify(|_, w| w.inrq().set_bit().sleep().clear_bit());

        // Wait for INAK bit in MSR to be set to indicate initialization is active
        loop {
            if can3.msr.read().inak().bit() {
                break;
            }
        }

        // Enable loopback mode so we can receive what we are sending.
        // Note: This will still send data out the TX pin unless silent mode is enabled.
        // Sets the timing to 125kbaud
        unsafe {
            can3.btr.modify(|_, w| {
                w.lbkm()
                    .enabled()
                    .sjw()
                    .bits(2)
                    .ts2()
                    .bits(5)
                    .ts1()
                    .bits(8)
                    .brp()
                    .bits(24)
            });
        }

        if !can3.msr.read().inak().bit() {
            write_string_to_serial(&mut tx, "INAK is cleared\n");
        } else {
            write_string_to_serial(&mut tx, "INAK is set\n");
        }

        // Switch hardware into normal mode.
        can3.mcr.modify(|_, w| w.inrq().clear_bit());

        // Wait for INAK bit in MSR to be cleared to indicate init has completed
        loop {
            if !can3.msr.read().inak().bit() {
                break;
            }
        }

        write_string_to_serial(&mut tx, "INAK cleared\n");

        // Set to standard identifier
        unsafe {
            can3.tx[0]
                .tir
                .modify(|_, w| w.ide().standard().stid().bits(12));
        }

        unsafe {
            can3.tx[0].tdtr.modify(|_, w| w.dlc().bits(2));
        }

        // Start transmission
        can3.tx[0].tir.modify(|_, w| w.txrq().set_bit());

        loop {
            if can3.tx[0].tir.read().txrq().bit_is_clear() {
                break;
            }
        }

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

pub fn setup_can_gpio<X, Y>(rx: PD0<X>, tx: PD1<Y>) {
    // CAN1 RX - PG0, TX - PG1
    // CAN1 RX - PA11, TX - PA12
    // CAN1 RX - PD0, TX - PD1 ---
    // CAN1 RX - PB8, TX - PB9
    // CAN2 RX - PB12, TX - PB13
    // CAN2 RX - PG11, TX - PG12
    // CAN2 RX - PB5, TX - PB6
    // CAN3 RX - PA8, TX - PA15
    // CAN3 RX - PB3, TX - PB4

    // Use PD0 RX, PD1 TX
    let _can_rx = rx.into_alternate_af9();
    let _can_tx = tx.into_alternate_af9();
}

pub fn write_string_to_serial(
    tx: &mut stm32f4xx_hal::serial::Tx<stm32f4::stm32f413::USART3>,
    string: &str,
) {
    write_bytes_to_serial(tx, string.as_bytes());
}

pub fn write_bytes_to_serial(
    tx: &mut stm32f4xx_hal::serial::Tx<stm32f4::stm32f413::USART3>,
    bytes: &[u8],
) {
    for byte in bytes.iter() {
        block!(tx.write(*byte)).unwrap();
    }
}

pub fn configure<X, Y>(
    uart: USART3,
    tx: PD8<X>,
    rx: PD9<Y>,
    baudrate: Bps,
    clocks: Clocks,
) -> hal::serial::Tx<stm32f4::stm32f413::USART3> {
    let config = Config {
        baudrate,
        ..Config::default()
    };

    let tx = tx.into_alternate_af7();
    let rx = rx.into_alternate_af7();
    let serial = Serial::usart3(uart, (tx, rx), config, clocks).unwrap();
    let (tx, _) = serial.split();
    tx
}

// Can FLOW:
// CAN clocks are enabled in RCC_APB1ENR
// CAN1 RX - PG0, TX - PG1
// CAN1 RX - PA11, TX - PA12
// CAN1 RX - PD0, TX - PD1 ---
// CAN1 RX - PB8, TX - PB9
// CAN2 RX - PB12, TX - PB13
// CAN2 RX - PG11, TX - PG12
// CAN2 RX - PB5, TX - PB6
// CAN3 RX - PA8, TX - PA15
// CAN3 RX - PB3, TX - PB4
//
// Can has 3 modes: Initialization, Normal, Sleep
// Set INRQ bit in CAN_MCR to enter initialization mode
// Wait for INAK bit to be set in CAN_MSR register
// Setup bit timing (CAN_BTR register) and CAN options (CAN_MCR registers)
// Clean INRQ bit to enter normal mode (Must wait for INAK to be cleared after 11 recessive cycles on the bus)

// Silent mode
// Entered by setting SILM bit in the CAN_BTR register
// Does not respond on the bus, perfect for logging

// Loop back mode
// Set LBKM in CAN_BTR register
// Stores transmitted messages in receive mailbox

// Transmit flow
// 1. Find an empty TX mailbox
// 2. Setup identifier, data length code, and data im empty TX mailbox
// 3. Set TXRQ bit in CAN_TIxR register to request the transmission start
// 4. Transmission success indicated by RWCP and TXOK bits set in CAN_TSR register
//    Failure indicated by ALST bit in CAN_Tsr for arbitration lost or TERR bit for transmission error
//
// Transmit priority
// Can be set to use identifier priority or FIFO by setting TXFP bit in CAN_MCR register

// Receive Flow
// Received after the message is completed and passed through the identifier filtering
// FMP[1:0] bits in CAN_RFR rigster indicates messages available in the FIFO
// Interrupts can be generated by setting FFIE bit in CAN_IER register
// Read from FIFO output mailbox, release the mailbox using the RFOM bit in CAN_RFR register

// Bit Timing
// Time is split into three segments
// Synchronization
// Segment 1
// Segment 2

// Baud rate = 1 / NominalBitTime
// NominalBitTime = 1 x t_q + t_bs1 + t_bs2
// t_bs1 = t_q x (TS1[3:0] + 1)
// t_bs1 = t_q x (TS2[3:0] + 1)
// t_q = (BRP[9:0] + 1) x t_pclk

// Need to find this
// t_pclk = time period of the APB clock

// CAN is on APB1 which is 50 MHz

// Baud Rate Prescaler = 24 (24 + 1)
// t_q = 0.5us
// t_bs1 = 9 (8 + 1)
// t_bs2 = 6 (5 + 1)

// Filter setup
// Can be setup while in initialization or normal mode
// Must set FINIT bit in CAN_FMR to modify the filters
// CAN reception is deactivated when FINIT = 1

// Notes:
// 1. Hard to tell if there are three or two can controllers
