#![no_std]
#![no_main]

extern crate alloc;

use embedded_graphics::{
    draw_target::{DrawTarget, DrawTargetExt},
    pixelcolor::{Rgb565, Rgb888, RgbColor},
    prelude::*,
    text::Text,
};
use embedded_hal::{delay::DelayNs, digital::InputPin};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    dma::{Dma, DmaPriority},
    dma_descriptors,
    gpio::{Input, Io, Level, Output},
    i2c::I2C,
    peripherals::Peripherals,
    prelude::*,
    psram,
    rtc_cntl::Rtc,
    spi::{
        master::{prelude::*, Spi},
        SpiMode,
    },
    system::SystemControl,
    timer,
};

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_psram_heap() {
    unsafe {
        ALLOCATOR.init(psram::psram_vaddr_start() as *mut u8, psram::PSRAM_BYTES);
    }
}

use lilygo_t4s3::{cst226, rm690b0};

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();

    psram::init_psram(peripherals.PSRAM);
    init_psram_heap();

    let system = SystemControl::new(peripherals.SYSTEM);

    let clocks = ClockControl::max(system.clock_control).freeze();
    let delay = Delay::new(&clocks);
    let rtc = Rtc::new(peripherals.LPWR, None);

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
                true,
                &mut tx_descriptors,
                &mut rx_descriptors,
                DmaPriority::Priority0,
            ));

        rm690b0::RM690B0::new(delay.clone(), spi, rst)
    };

    log::info!("initializing display");
    display.reset();
    display.init().expect("error initializing display");

    let mut touch = {
        //{6/*SDA*/, 7/*SCL*/, 8/*IRQ*/, 17/*RST*/};
        let rst = Output::new(io.pins.gpio17, Level::High);
        let irq = Input::new(io.pins.gpio8, esp_hal::gpio::Pull::Up);

        let sda = io.pins.gpio6;
        let scl = io.pins.gpio7;

        let i2c = I2C::new(peripherals.I2C1, sda, scl, 100.kHz(), &clocks, None);

        cst226::CST226::new(i2c, rst, delay.clone())
    };

    touch.reset().expect("could not reset touch controller");
    touch.init().expect("could not initialize touch controller");

    loop {
        //log::info!("reading touch");
    }
    //unreachable!();
}
