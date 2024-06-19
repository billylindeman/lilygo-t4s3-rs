#![no_std]
#![no_main]

extern crate alloc;

use alloc::vec::Vec;
use embedded_graphics::{
    geometry::Size,
    mono_font::iso_8859_14::FONT_6X10,
    pixelcolor::Rgb565,
    primitives::StyledDrawable,
    text::{Text, TextStyleBuilder},
};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::{Io, Level, Output},
    peripherals::{self, Peripherals},
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

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let mut pmic_en = Output::new(io.pins.gpio9, Level::High);

    let mut display = {
        let rst = Output::new(io.pins.gpio13, Level::High);

        let sclk = io.pins.gpio15;
        let cs = io.pins.gpio11;
        let data0 = io.pins.gpio14;
        let data1 = io.pins.gpio10;
        let data2 = io.pins.gpio16;
        let data3 = io.pins.gpio12;

        let spi = Spi::new_half_duplex(peripherals.SPI3, 36.MHz(), SpiMode::Mode0, &clocks)
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

    {
        use embedded_graphics::{
            mono_font::{ascii::FONT_10X20, MonoTextStyle},
            text::Text,
        };

        use embedded_graphics::{
            pixelcolor::Rgb565,
            prelude::*,
            primitives::{PrimitiveStyleBuilder, Rectangle},
        };

        let style = PrimitiveStyleBuilder::new().fill_color(Rgb565::RED).build();

        Rectangle::new(Point::new(0, 0), display.size())
            .into_styled(style)
            .draw(&mut display)
            .unwrap();

        let style = PrimitiveStyleBuilder::new()
            .fill_color(Rgb565::WHITE)
            .build();

        Rectangle::new(Point::new(64, 64), Size::new(450 - 64, 600 - 64))
            .into_styled(style)
            .draw(&mut display)
            .unwrap();

        //let style = MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE);
        //Text::new("Hello,\nRust!", Point::new(2, 28), style)
        //    .draw(&mut display)
        //    .expect("could not write text");
    }

    display.flush().expect("could not flush display");

    loop {
        delay.delay(5000.millis());
    }
}
