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
    //console(delay, display);
    three_dimension(delay, display);
    unreachable!();
}

fn three_dimension<
    D: DrawTarget<Color = Rgb565, Error = E> + OriginDimensions + rm690b0::Display,
    T: DelayNs,
    E: core::fmt::Debug,
>(
    mut delay: T,
    mut display: D,
) {
    use alloc::vec::Vec;
    use core::f32::consts::PI;
    use embedded_gfx::draw;
    use embedded_gfx::mesh::Geometry;
    use embedded_gfx::mesh::K3dMesh;
    use embedded_gfx::K3dengine;
    use load_stl::embed_stl;
    use nalgebra::Point3;
    use num_traits::Float;
    log::info!("creating 3d scene");

    //
    // ----------------- CUT HERE -----------------
    //
    let ground_vertices = {
        let step = 1.0;
        let nsteps = 10;

        let mut vertices = Vec::new();
        for i in 0..nsteps {
            for j in 0..nsteps {
                vertices.push([
                    (i as f32 - nsteps as f32 / 2.0) * step,
                    0.0,
                    (j as f32 - nsteps as f32 / 2.0) * step,
                ]);
            }
        }

        vertices
    };

    let mut ground = K3dMesh::new(Geometry {
        vertices: &ground_vertices,
        faces: &[],
        colors: &[],
        lines: &[],
        normals: &[],
    });
    ground.set_color(Rgb565::new(0, 255, 0));

    //let mut suzanne = K3dMesh::new(embed_stl!("src/models/Suzanne.stl"));
    //uzanne.set_render_mode(RenderMode::Lines);
    //suzanne.set_scale(2.0);
    //suzanne.set_color(Rgb565::CSS_RED);

    let mut teapot = K3dMesh::new(embed_stl!("src/models/Teapot_low.stl"));
    teapot.set_render_mode(embedded_gfx::mesh::RenderMode::Lines);
    teapot.set_position(0.0, 0.0, 0.0);
    teapot.set_color(Rgb565::CSS_WHITE);

    //let mut blahaj = K3dMesh::new(embed_stl!("src/models/blahaj.stl"));
    //blahaj.set_color(Rgb565::new(105 >> 3, 150 >> 2, 173 >> 3));
    //blahaj.set_render_mode(RenderMode::SolidLightDir(nalgebra::Vector3::new(
    //    -1.0, 0.0, 0.0,
    //)));

    let size = display.size();
    let mut engine = K3dengine::new(size.width as u16, size.height as u16);
    engine.camera.set_position(Point3::new(0.0, 2.0, -2.0));
    engine.camera.set_target(Point3::new(0.0, 0.0, 0.0));
    engine.camera.set_fovy(PI / 4.0);

    //let mut perf = PerformanceCounter::new();
    //perf.only_fps(true);

    let mut moving_parameter: f32 = 0.0;

    log::info!("starting main loop");
    let mut player_pos = Point3::new(-10.0, 2.0, 0.0);
    let mut player_dir = 0.0f32;
    let mut player_head = 0.0f32;
    loop {
        //let fbuf = buffers.swap_framebuffer();

        //let ft = perf.get_frametime();
        //let dt = ft as f32 / 1_000_000.0;
        let dt = 0.1f32;

        //perf.start_of_frame();

        let walking_speed = 5.0 * dt;
        let turning_speed = 0.6 * dt;

        //let keys = p.keyboard.read_keys();
        //for key in keys {
        //    match key {
        //        keyboard::Key::Semicolon => {
        //            player_pos.x += player_dir.cos() * walking_speed;
        //            player_pos.z += player_dir.sin() * walking_speed;
        //        }
        //        keyboard::Key::Period => {
        //            player_pos.x -= player_dir.cos() * walking_speed;
        //            player_pos.z -= player_dir.sin() * walking_speed;
        //        }
        //        keyboard::Key::Slash => {
        //            player_pos.x += (player_dir + PI / 2.0).cos() * walking_speed;
        //            player_pos.z += (player_dir + PI / 2.0).sin() * walking_speed;
        //        }
        //        keyboard::Key::Comma => {
        //            player_pos.x -= (player_dir + PI / 2.0).cos() * walking_speed;
        //            player_pos.z -= (player_dir + PI / 2.0).sin() * walking_speed;
        //        }

        //        keyboard::Key::D => {
        //            player_dir += turning_speed;
        //        }
        //        keyboard::Key::A => {
        //            player_dir -= turning_speed;
        //        }

        //        keyboard::Key::E => {
        //            player_head += turning_speed;
        //        }
        //        keyboard::Key::S => {
        //            player_head -= turning_speed;
        //        }
        //        _ => {}
        //    }
        //}

        engine.camera.set_position(player_pos);

        let lookat = player_pos
            + nalgebra::Vector3::new(player_dir.cos(), player_head.sin(), player_dir.sin());
        engine.camera.set_target(lookat);

        //suzanne.set_attitude(-PI / 2.0, moving_parameter * 2.0, 0.0);
        //suzanne.set_position(0.0, 0.7 + (moving_parameter * 3.4).sin() * 0.2, 10.0);

        //blahaj.set_attitude(-PI / 2.0, moving_parameter * 2.0, 0.0);
        //blahaj.set_position(0.0, 0.7 + (moving_parameter * 3.4).sin() * 0.2, 0.0);

        teapot.set_attitude(-PI / 2.0, moving_parameter * 1.0, 0.0);
        //teapot.set_scale(0.2 + 0.1 * (moving_parameter * 5.0).sin());
        teapot.set_scale(0.5);

        //perf.add_measurement("setup");

        display.clear(Rgb565::CSS_BLACK).unwrap(); // 2.2ms

        //perf.add_measurement("clear");
        //engine.render([&ground, &teapot, &suzanne, &blahaj], |p| draw(p, fbuf));
        engine.render([&ground, &teapot], |p| draw::draw(p, &mut display));

        //perf.add_measurement("render");

        //Text::new(perf.get_text(), Point::new(20, 20), text_style)
        //    .draw(fbuf)
        //    .unwrap();

        //perf.discard_measurement();

        moving_parameter += 0.3 * dt;

        //
        // ----------------- CUT HERE -----------------
        //

        //buffers.send_framebuffer();

        //perf.add_measurement("draw");

        //perf.print();

        //info!("-> {}", perf.get_text());

        display.flush().ok();
        //delay.delay_ms(8);
    }
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

        delay.delay_ms(500);
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
        delay.delay_ms(8);
    }
}
