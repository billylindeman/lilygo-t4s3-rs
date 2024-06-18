#![no_std]
#![no_main]

extern crate alloc;

use alloc::vec::Vec;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::{Io, Level, Output},
    peripherals::Peripherals,
    prelude::*,
    psram,
    spi::{
        master::{prelude::*, Spi},
        SpiMode,
    },
    system::SystemControl,
};

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_psram_heap() {
    unsafe {
        ALLOCATOR.init(psram::psram_vaddr_start() as *mut u8, psram::PSRAM_BYTES);
    }
}

pub mod rm690b0;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();

    psram::init_psram(peripherals.PSRAM);
    init_psram_heap();

    let system = SystemControl::new(peripherals.SYSTEM);

    let clocks = ClockControl::max(system.clock_control).freeze();
    let delay = Delay::new(&clocks);

    esp_println::logger::init_logger_from_env();

    {
        log::info!("========== Placing vector in PSRAM ==========");
        let mut large_vec: Vec<u32> = Vec::with_capacity(8000 * 1024 / 4);

        for i in 0..(8000 * 1024 / 4) {
            large_vec.push((i & 0xff) as u32);
        }

        log::info!("large vec size = {} bytes", large_vec.len() * 4);
        log::info!("large vec address = {:p}", large_vec.as_ptr());
        log::info!("large vec[..100] = {:?}", &large_vec[..100]);
    }

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let mut display = {
        let rst = Output::new(io.pins.gpio13, Level::High);

        let sclk = io.pins.gpio15;
        let cs = io.pins.gpio11;
        let data0 = io.pins.gpio14;
        let data1 = io.pins.gpio10;
        let data2 = io.pins.gpio16;
        let data3 = io.pins.gpio12;

        let spi = Spi::new_half_duplex(peripherals.SPI2, 30.MHz(), SpiMode::Mode0, &clocks)
            .with_pins(
                Some(sclk),
                Some(data0),
                Some(data1),
                Some(data2),
                Some(data3),
                Some(cs),
            );

        rm690b0::RM690B0::new(delay.clone(), spi, rst)
    };

    log::info!("initializing display");
    display.reset();
    display.init().expect("error initializing display");

    loop {
        log::info!("Hello world!");
        delay.delay(500.millis());
    }
}
