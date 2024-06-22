#![no_std]
#![no_main]

extern crate alloc;

use embedded_graphics::{
    draw_target::{DrawTarget, DrawTargetExt},
    pixelcolor::{Rgb565, Rgb888, RgbColor},
    prelude::*,
    text::Text,
};
use embedded_hal::delay::DelayNs;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    dma::{Dma, DmaPriority},
    dma_descriptors,
    gpio::{Io, Level, Output},
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

use lilygo_t4s3::rm690b0;

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

    render_3d_benchy(delay, display, rtc);
    unreachable!();
}

fn render_3d_benchy<
    D: DrawTarget<Color = Rgb565, Error = E> + OriginDimensions + rm690b0::Display,
    T: DelayNs,
    E: core::fmt::Debug,
>(
    mut delay: T,
    mut display: D,
    rtc: Rtc,
) {
    use core::f32::consts::PI;
    use embedded_gfx::draw;
    use embedded_gfx::mesh::Geometry;
    use embedded_gfx::mesh::K3dMesh;
    use embedded_gfx::K3dengine;
    use load_stl::embed_stl;
    use nalgebra::Point3;
    use num_traits::Float;
    log::info!("creating 3d scene");

    let mut benchy = K3dMesh::new(embed_stl!("src/bin/mesh/benchy.stl"));
    benchy.set_render_mode(embedded_gfx::mesh::RenderMode::Lines);
    benchy.set_position(0.0, 0.0, 0.0);
    benchy.set_color(Rgb565::CSS_WHITE);

    let size = display.size();
    let mut engine = K3dengine::new(size.width as u16, size.height as u16);
    engine.camera.set_position(Point3::new(0.0, 2.0, -2.0));
    engine.camera.set_target(Point3::new(0.0, 0.0, 0.0));
    engine.camera.set_fovy(PI / 4.0);

    let mut moving_parameter: f32 = 0.0;

    log::info!("starting main loop");
    let player_pos = Point3::new(-10.0, 2.0, 0.0);
    let player_dir = 0.0f32;
    let player_head = 0.0f32;

    let mut dt = 0.1;
    loop {
        let start_time = rtc.get_time_ms();

        engine.camera.set_position(player_pos);

        let lookat = player_pos
            + nalgebra::Vector3::new(player_dir.cos(), player_head.sin(), player_dir.sin());
        engine.camera.set_target(lookat);

        benchy.set_attitude(-PI / 2.0, moving_parameter * 1.0, 0.0);
        benchy.set_scale(0.1);

        display.clear(Rgb565::BLACK).unwrap(); // 2.2ms
        engine.render([&benchy], |p| draw::draw(p, &mut display));

        moving_parameter += 0.5 * dt;
        let render_time = rtc.get_time_ms();

        let fps = (1.0f32 / dt) as u32;

        display.flush().expect("could not flush display");

        let display_time = rtc.get_time_ms();
        dt = (display_time - start_time) as f32 / 1000f32;

        log::info!(
            "render_time = {}, display_time={}ms, dt={}, fps={} ",
            render_time - start_time,
            display_time - render_time,
            dt,
            fps,
        );
    }
}
