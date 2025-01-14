//! RM690B0 Display Driver for esp-hal
//! Billy Lindeman <billylindeman@gmail.com>
//!
//! This is a display driver for the RM690B0 controller chip
//! on Lilygo T4S3 ESP32S3 w/ 2.41 AMOLED Display
//!
//! This controller utilizes SPI for Data and QSPI for Pixel Data
//! This driver supports both normal SPI mode as well as DMA
//! It provides an embedded_graphics::DrawTarget implementation for rendering

use alloc::vec;
use alloc::vec::Vec;
use anyhow::Result;
use embedded_graphics::{
    draw_target::DrawTarget, geometry::OriginDimensions, pixelcolor::Rgb565, prelude::*, Pixel,
};
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use esp_hal::spi::{
    master::{dma::SpiDma, Address, Command, HalfDuplexReadWrite, Spi},
    SpiDataMode, SpiMode,
};

pub enum MemoryAccessControlFlags {
    MY = 0x80,
    MX = 0x40,
    MV = 0x20,
    ML = 0x10,
    BGR = 0x08,
    MH = 0x04,
    RGB = 0x00,
}

#[derive(Debug, Clone, Copy)]
pub enum DisplayCommand {
    Nop = 0x0000,                  // No Operation
    SwReset = 0x0100,              // Software Reset
    SleepIn = 0x1000,              // Sleep In
    SleepOut = 0x1100,             // Sleep Out
    PartialDisplayModeOn = 0x1200, // Partial Display Mode On
    NormalDisplayModeOn = 0x1300,  // Normal Display Mode On
    DisplayInversionOff = 0x2000,
    DisplayInversionOn = 0x2100,
    AllPixelsOff = 0x2200, // Display Inversion Off
    AllPixelsOn = 0x2300,
    DisplayOff = 0x2800,       // Display Off
    DisplayOn = 0x2900,        // Display On
    ColumnAddressSet = 0x2A00, // Column Address Set
    RowAddressSet = 0x2B00,
    MemoryWrite = 0x2C00,
    PartialArea = 0x3000, // Partial Area
    PartialAreaVertical = 0x3100,
    TearingEffectLineOff = 0x3400, // Tearing Effect Line Off
    TearingEffectLineOn = 0x3500,  // Tearing Effect Line On
    MemoryAccessControl = 0x3600,  // Memory Access Control
    IdleModeOff = 0x3800,          // Idle Mode Off
    IdleModeOn = 0x3900,           // Idle Mode On
    InterfacePixelFormat = 0x3A00, // Interface Pixel Format
    MemoryWriteContinue = 0x3C00,  // Write Memory Continue
    SetTearScaline = 0x4400,
    GetScanline = 0x4500,
    DeepStandbyModeOn = 0x4F00,
    WriteDisplayBrightness = 0x5100, // Write Display Brightness
    ReadDisplayBrightness = 0x5200,  // Read Display Brightness
    WriteDisplayControl = 0x5300,    // Write Control Display
    ReadDisplayControl = 0x5400,     // Read Control Display
    SetDisplayMode = 0xC200,
    PageSet = 0xFE00,
}

impl From<DisplayCommand> for u16 {
    fn from(value: DisplayCommand) -> Self {
        value as u16
    }
}

pub trait SpiWrite {
    fn write_command<C: Copy + Into<u16>>(&mut self, cmd: C, param: &[u8]) -> Result<()>;
    fn write_pixels(&mut self, chunk: &[u8]) -> Result<()>;
}

pub trait Display {
    fn flush(&mut self) -> Result<()>;

    fn flush_dirty(&mut self) -> Result<()> {
        unimplemented!();
    }
}

#[cfg(not(feature = "dma"))]
impl<SPI> SpiWrite for SPI
where
    SPI: esp_hal::spi::master::HalfDuplexReadWrite,
{
    fn write_command<C: Copy + Into<u16>>(&mut self, cmd: C, mut param: &[u8]) -> Result<()> {
        self.write(
            SpiDataMode::Single,
            Command::Command8(0x02, SpiDataMode::Single),
            Address::Address24(cmd.into() as u32, SpiDataMode::Single),
            0,
            &mut param,
        )
        .map_err(|_| anyhow::format_err!("spi error"))?;

        Ok(())
    }

    fn write_pixels(&mut self, chunk: &[u8]) -> Result<()> {
        self.write(
            SpiDataMode::Quad,
            Command::Command8(0x12, SpiDataMode::Single),
            Address::Address24(0x3C00, SpiDataMode::Quad),
            0,
            &chunk,
        )
        .map_err(|_| anyhow::format_err!("spi error"))?;
        Ok(())
    }
}

#[cfg(feature = "dma")]
impl<'d, T, C, M, DmaMode> SpiWrite for SpiDma<'d, T, C, M, DmaMode>
where
    T: esp_hal::spi::master::InstanceDma<C::Tx<'d>, C::Rx<'d>>,
    C: esp_hal::dma::ChannelTypes,
    C::P: esp_hal::dma::SpiPeripheral,
    M: esp_hal::spi::IsHalfDuplex,
    DmaMode: esp_hal::Mode,
{
    fn write_command<CMD: Copy + Into<u16>>(&mut self, cmd: CMD, param: &[u8]) -> Result<()> {
        let dma_tx_buffer = dma_tx_buffer();
        dma_tx_buffer[..param.len()].copy_from_slice(param);

        let ptr = dma_tx_buffer as *const _;
        let slice: &[u8] = unsafe { alloc::slice::from_raw_parts::<u8>(ptr, param.len()) };

        self.write(
            SpiDataMode::Single,
            Command::Command8(0x02, SpiDataMode::Single),
            Address::Address24(cmd.into() as u32, SpiDataMode::Single),
            0,
            &slice,
        )
        .map_err(|_| anyhow::format_err!("spi dma error"))?
        .wait()
        .map_err(|_| anyhow::format_err!("spi dma error"))?;

        Ok(())
    }
    fn write_pixels(&mut self, framebuffer: &[u8]) -> Result<()> {
        for chunk in framebuffer.chunks(32000) {
            let dma_tx_buffer = dma_tx_buffer();
            dma_tx_buffer[..chunk.len()].copy_from_slice(chunk);

            let ptr = dma_tx_buffer as *const _;
            let slice: &[u8] = unsafe { alloc::slice::from_raw_parts::<u8>(ptr, chunk.len()) };

            self.write(
                SpiDataMode::Quad,
                Command::Command8(0x12, SpiDataMode::Single),
                Address::Address24(0x3C00, SpiDataMode::Quad),
                0,
                &slice,
            )
            .map_err(|_| anyhow::format_err!("spi dma error"))?
            .wait()
            .map_err(|_| anyhow::format_err!("spi dma error"))?;
        }

        Ok(())
    }
}

#[cfg(feature = "dma")]
#[allow(static_mut_refs)]
fn dma_tx_buffer() -> &'static mut [u8; 32000] {
    static mut BUFFER: [u8; 32000] = [0u8; 32000];
    unsafe { &mut BUFFER }
}

pub struct RM690B0<D, SPI, RST>
where
    D: DelayNs,
    SPI: SpiWrite,
    RST: OutputPin,
{
    delay: D,
    spi: SPI,
    rst: RST,
    buf: Vec<u8>,
}
impl<D, SPI, RST> RM690B0<D, SPI, RST>
where
    D: DelayNs,
    SPI: SpiWrite,
    RST: OutputPin,
{
    pub fn new(delay: D, spi: SPI, rst: RST) -> Self {
        let buf = vec![0; 540000];

        Self {
            delay,
            spi,
            rst,
            buf,
        }
    }

    // Reset Display
    pub fn reset(&mut self) {
        self.rst.set_high().expect("could not set reset pin");
        self.delay.delay_ms(200);

        self.rst.set_low().expect("could not set reset pin");
        self.delay.delay_ms(300);

        self.rst.set_high().expect("could not set reset pin");
        self.delay.delay_ms(200);
    }

    pub fn init(&mut self) -> Result<()> {
        //{0xFE, {0x20}, 0x01},           //SET PAGE
        //{0x26, {0x0A}, 0x01},           //MIPI OFF
        //{0x24, {0x80}, 0x01},           //SPI write RAM
        //{0x5A, {0x51}, 0x01},           //! 230918:SWIRE FOR BV6804
        //{0x5B, {0x2E}, 0x01},           //! 230918:SWIRE FOR BV6804
        //{0xFE, {0x00}, 0x01},           //SET PAGE
        //{0x3A, {0x55}, 0x01},           //Interface Pixel Format    16bit/pixel
        //{0xC2, {0x00}, 0x21},           //delay_ms(10);
        //{0x35, {0x00}, 0x01},           //TE ON
        //{0x51, {0x00}, 0x01},           //Write Display Brightness  MAX_VAL=0XFF
        //{0x11, {0x00}, 0x80},           //Sleep Out delay_ms(120);
        //{0x29, {0x00}, 0x20},           //Display on delay_ms(10);
        //{0x51, {0xFF}, 0x01},           //Write Display Brightness  MAX_VAL=0XFF

        self.spi.write_command(DisplayCommand::PageSet, &[0x20])?; //SET PAGE (MFG Command?)
        self.spi.write_command(0x2600u16, &[0x0A])?; //MIPI OFF
        self.spi.write_command(0x2400u16, &[0x80])?; //SPI write RAM
        self.spi.write_command(0x5A00u16, &[0x51])?; // 230918:SWIRE FOR BV6804
        self.spi.write_command(0x5B00u16, &[0x2E])?; // 230918:SWIRE FOR BV6804
        self.spi.write_command(DisplayCommand::PageSet, &[0x00])?; //SET PAGE (User Command)
        self.spi
            .write_command(DisplayCommand::InterfacePixelFormat, &[0x55])?; //Interface Pixel Format    16bit/pixel
        self.spi
            .write_command(DisplayCommand::SetDisplayMode, &[0x00])?;
        self.delay.delay_ms(10);
        self.spi
            .write_command(DisplayCommand::TearingEffectLineOn, &[])?; //TE ON
        self.spi
            .write_command(DisplayCommand::WriteDisplayBrightness, &[])?; //Write Display Brightness  MAX_VAL=0XFF
        self.spi.write_command(DisplayCommand::SleepOut, &[])?; //Sleep Out delay_ms(120);
        self.delay.delay_ms(120);
        self.spi.write_command(DisplayCommand::DisplayOn, &[])?; //Display on delay_ms(10);
        self.delay.delay_ms(10);
        self.spi
            .write_command(DisplayCommand::WriteDisplayBrightness, &[0xFF])?; //Write Display Brightness  MAX_VAL=0XFF
        self.spi
            .write_command(DisplayCommand::NormalDisplayModeOn, &[])?;

        self.spi.write_command(
            DisplayCommand::MemoryAccessControl,
            &[MemoryAccessControlFlags::RGB as u8
                | MemoryAccessControlFlags::MV as u8
                | MemoryAccessControlFlags::MX as u8],
        )?;

        Ok(())
    }

    pub fn set_address_window(
        &mut self,
        x_start: u16,
        y_start: u16,
        x_end: u16,
        y_end: u16,
    ) -> Result<()> {
        //log::info!(
        //    "set_address_window x=({},{}), y=({},{}) delta_x={}, delta_y={}",
        //    x_start,
        //    x_end,
        //    y_start,
        //    y_end,
        //    dx,
        //    dy
        //);
        let param_caset = vec![
            ((x_start >> 8) & 0xFF) as u8,
            (x_start & 0xFF) as u8,
            ((x_end >> 8) & 0xFF) as u8,
            (x_end & 0xFF) as u8,
        ];

        let param_raset = vec![
            ((y_start >> 8) & 0xFF) as u8,
            (y_start & 0xFF) as u8,
            ((y_end >> 8) & 0xFF) as u8,
            (y_end & 0xFF) as u8,
        ];

        self.spi
            .write_command(DisplayCommand::ColumnAddressSet, &param_caset)?;
        self.spi
            .write_command(DisplayCommand::RowAddressSet, &param_raset)?;
        self.spi.write_command(DisplayCommand::MemoryWrite, &[])?;

        Ok(())
    }

    //fn flush_clip(&mut self, start: Point, end: Point) -> Result<()> {
    //    self.set_address_window(
    //        16 + start.x as u16,
    //        16 + start.y as u16,
    //        16 + end.x as u16 - 1,
    //        16 + end.y as u16 - 1,
    //    )?;

    //    let mut chunk = Vec::with_capacity(32000);
    //    let size = self.size();

    //    for y in start.y..end.y {
    //        for x in start.x..end.x {
    //            let idx = ((x + (y * size.width as i32)) * 2) as usize;
    //            chunk.push(self.buf[idx]);
    //            chunk.push(self.buf[idx + 1]);

    //            if chunk.len() >= 32000 {
    //                self.spi.write_pixels(&chunk)?;
    //                chunk = Vec::with_capacity(32000);
    //            }
    //        }
    //    }

    //    if chunk.len() > 0 {
    //        //log::info!("last chunk {}", chunk.len());
    //        self.spi.write_pixels(&chunk)?;
    //    }

    //    //log::info!("flushed");

    //    Ok(())
    //}
}
impl<D, SPI, RST> Display for RM690B0<D, SPI, RST>
where
    D: DelayNs,
    SPI: SpiWrite,
    RST: OutputPin,
{
    fn flush(&mut self) -> Result<()> {
        //self.dirty.clear();
        let size = self.size();
        self.set_address_window(16, 0, 16 + size.width as u16 - 1, size.height as u16 - 1)?;
        self.spi.write_pixels(&self.buf)?;
        Ok(())
    }

    //fn flush_dirty(&mut self) -> Result<()> {
    //    while let Some((start, end)) = self.dirty.pop() {
    //        self.flush_clip(start, end)?;
    //    }

    //    Ok(())
    //}
}

impl<D, SPI, RST> OriginDimensions for RM690B0<D, SPI, RST>
where
    D: DelayNs,
    SPI: SpiWrite,
    RST: OutputPin,
{
    fn size(&self) -> embedded_graphics::prelude::Size {
        (600, 450).into()
    }
}

impl<D, SPI, RST> DrawTarget for RM690B0<D, SPI, RST>
where
    D: DelayNs,
    SPI: SpiWrite,
    RST: OutputPin,
{
    type Error = anyhow::Error;
    type Color = Rgb565;

    fn draw_iter<I>(&mut self, pixels: I) -> core::result::Result<(), Self::Error>
    where
        I: IntoIterator<Item = embedded_graphics::Pixel<Self::Color>>,
    {
        let size = self.size();

        //let mut start = Point::new(size.width as i32, size.height as i32);
        //let mut end = Point::zero();

        for pixel in pixels {
            let Pixel(point, color) = pixel;

            //if point.x <= start.x {
            //    start.x = point.x
            //}
            //if point.y <= start.y {
            //    start.y = point.y
            //}
            //if point.x >= end.x {
            //    end.x = point.x
            //}
            //if point.y >= end.y {
            //    end.y = point.y
            //}

            let idx = ((point.x + (point.y * size.width as i32)) * 2) as usize;

            if idx < self.buf.len() {
                let [a, b] = color.to_be_bytes();
                self.buf[idx] = a;
                self.buf[idx + 1] = b;
            }
        }

        // column addresses must be divisible 2
        //start.x -= start.x % 2;
        //start.y -= start.x % 2;
        //end.x += end.x % 2;
        //end.x += end.x % 2;

        //self.dirty.push((start, end));

        Ok(())
    }

    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        let color = u16::from_be(color.into_storage());
        let (_, body, _) = unsafe { self.buf.align_to_mut::<u16>() };
        body.fill(color);
        Ok(())
    }
}
