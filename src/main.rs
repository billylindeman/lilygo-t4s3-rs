#![no_std]
#![no_main]

extern crate alloc;

use embedded_graphics::{
    draw_target::{DrawTarget, DrawTargetExt},
    geometry::Point,
    pixelcolor::{Rgb565, Rgb888, RgbColor},
    prelude::*,
};
use embedded_hal::delay::DelayNs;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    dma::{Dma, DmaPriority},
    dma_buffers, dma_descriptors,
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

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let dma = Dma::new(peripherals.DMA);
    let dma_channel = dma.channel0;
    let (mut tx_descriptors, mut rx_descriptors) = dma_descriptors!(32000);

    let mut _pmic_en = Output::new(io.pins.gpio9, Level::High);

    let mut display = {
        let rst = Output::new(io.pins.gpio13, Level::High);

        let sclk = io.pins.gpio15;
        let cs = io.pins.gpio11;
        let data0 = io.pins.gpio14;
        let data1 = io.pins.gpio10;
        let data2 = io.pins.gpio16;
        let data3 = io.pins.gpio12;

        let spi = Spi::new_half_duplex(peripherals.SPI3, 100.MHz(), SpiMode::Mode0, &clocks)
            .with_pins(
                Some(sclk),
                Some(data0),
                Some(data1),
                Some(data2),
                Some(data3),
                Some(cs),
            )
            .with_dma(dma_channel.configure(
                false,
                &mut tx_descriptors,
                &mut rx_descriptors,
                DmaPriority::Priority0,
            ));

        rm690b0::RM690B0::new(delay.clone(), spi, rst)
    };

    log::info!("initializing display");
    display.reset();
    display.init().expect("error initializing display");

    //flip(delay, display);
    console(delay, display);
    unreachable!();
}

fn console<D: DrawTarget<Color = Rgb565> + OriginDimensions + rm690b0::Display, T: DelayNs>(
    mut delay: T,
    mut display: D,
) {
    use alloc::fmt::Write;
    use embedded_term::Console;

    let mut i = 0;

    let converted = display.color_converted::<Rgb888>();
    let mut console = Console::on_frame_buffer(converted);

    loop {
        console
            .write_str(&alloc::format!("HELLO WORLD RUST {} \n ", i))
            .unwrap();

        i += 1;
    }
}

fn flip<D: DrawTarget<Color = Rgb565> + OriginDimensions + rm690b0::Display, T: DelayNs>(
    mut delay: T,
    mut display: D,
) {
    let mut flip = false;
    loop {
        use embedded_graphics::{
            pixelcolor::Rgb565,
            prelude::*,
            primitives::{PrimitiveStyleBuilder, Rectangle},
        };

        // Rectangle with red 3 pixel wide stroke and green fill with the top left corner at (30, 20) and
        // a size of (10, 15)
        let style = PrimitiveStyleBuilder::new()
            .fill_color(match flip {
                true => Rgb565::GREEN,
                false => Rgb565::BLUE,
            })
            .build();

        Rectangle::new(Point::new(0, 0), display.size())
            .into_styled(style)
            .draw(&mut display)
            .ok();
        flip = !flip;

        display.flush().ok();
        delay.delay_ms(16);
    }
}
