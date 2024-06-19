use alloc::vec;
use alloc::vec::Vec;
use anyhow::Result;
use embedded_graphics::{
    draw_target::DrawTarget, geometry::OriginDimensions, pixelcolor::Rgb565, prelude::*, Pixel,
};
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use esp_hal::spi::{
    master::{Address, Command, HalfDuplexReadWrite, Spi},
    SpiDataMode, SpiMode,
};

const RM690B0_MADCTL_MY: u8 = 0x80;
const RM690B0_MADCTL_MX: u8 = 0x40;
const RM690B0_MADCTL_MV: u8 = 0x20;
const RM690B0_MADCTL_ML: u8 = 0x10;
const RM690B0_MADCTL_RGB: u8 = 0x00;
const RM690B0_MADCTL_MH: u8 = 0x04;
const RM690B0_MADCTL_BGR: u8 = 0x08;

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

    PageSet = 0xFE00,
}

impl From<DisplayCommand> for u16 {
    fn from(value: DisplayCommand) -> Self {
        value as u16
    }
}

pub struct RM690B0<D, SPI, RST>
where
    D: DelayNs,
    SPI: esp_hal::spi::master::HalfDuplexReadWrite,
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
    SPI: esp_hal::spi::master::HalfDuplexReadWrite,
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

        self.write_command(DisplayCommand::PageSet, &[0x20])?; //SET PAGE
        self.write_command(0x2600u16, &[0x0A])?; //MIPI OFF
        self.write_command(0x2400u16, &[0x80])?; //SPI write RAM
        self.write_command(0x5A00u16, &[0x51])?; // 230918:SWIRE FOR BV6804
        self.write_command(0x5B00u16, &[0x2E])?; // 230918:SWIRE FOR BV6804
        self.write_command(DisplayCommand::PageSet, &[0x00])?; //SET PAGE
        self.write_command(DisplayCommand::InterfacePixelFormat, &[0x55])?; //Interface Pixel Format    16bit/pixel
        self.write_command(0xC200u16, &[])?; //delay_ms(10);
        self.delay.delay_ms(10);
        self.write_command(DisplayCommand::TearingEffectLineOn, &[])?; //TE ON
        self.write_command(DisplayCommand::WriteDisplayBrightness, &[])?; //Write Display Brightness  MAX_VAL=0XFF
        self.write_command(DisplayCommand::SleepOut, &[])?; //Sleep Out delay_ms(120);
        self.delay.delay_ms(120);
        self.write_command(DisplayCommand::DisplayOn, &[])?; //Display on delay_ms(10);
        self.delay.delay_ms(10);
        self.write_command(DisplayCommand::WriteDisplayBrightness, &[0xFF])?; //Write Display Brightness  MAX_VAL=0XFF

        self.write_command(DisplayCommand::NormalDisplayModeOn, &[])?;
        //self.write_command(DisplayCommand::AllPixelsOn, &[])?;

        self.write_command(
            DisplayCommand::MemoryAccessControl,
            &[RM690B0_MADCTL_RGB | RM690B0_MADCTL_MV | RM690B0_MADCTL_MX],
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
        log::info!(
            "set_address_window x=({},{}), y=({},{})",
            x_start,
            x_end,
            y_start,
            y_end
        );

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

        self.write_command(DisplayCommand::ColumnAddressSet, &param_caset)?;
        self.write_command(DisplayCommand::RowAddressSet, &param_raset)?;
        self.write_command(DisplayCommand::MemoryWrite, &[])?;

        Ok(())
    }

    fn write_command<C: Copy + Into<u16>>(&mut self, cmd: C, mut param: &[u8]) -> Result<()> {
        log::info!(
            "write_command addr=0x{:06x} param={:?}",
            (cmd.into() as u32),
            param,
        );
        self.spi
            .write(
                SpiDataMode::Single,
                Command::Command8(0x02, SpiDataMode::Single),
                Address::Address24(cmd.into() as u32, SpiDataMode::Single),
                0,
                &mut param,
            )
            .map_err(|_| anyhow::format_err!("spi error"))?;

        Ok(())
    }

    pub fn flush(&mut self) -> Result<()> {
        let size = self.size();
        self.set_address_window(16, 0, 16 + size.width as u16 - 1, size.height as u16 - 1)?;

        let chunks = self.buf.chunks(64);
        let mut first = true;

        for chunk in chunks {
            if first {
                self.spi
                    .write(
                        SpiDataMode::Quad,
                        Command::Command8(0x32, SpiDataMode::Single),
                        Address::Address24(0x2C00, SpiDataMode::Single),
                        0,
                        &chunk,
                    )
                    .map_err(|_| anyhow::format_err!("spi error"))?;
                first = false;
            } else {
                self.spi
                    .write(
                        SpiDataMode::Quad,
                        Command::Command8(0x32, SpiDataMode::Single),
                        Address::Address24(0x3C00, SpiDataMode::Single),
                        0,
                        &chunk,
                    )
                    .map_err(|_| anyhow::format_err!("spi error"))?;
            }
        }

        log::info!("flushed");

        Ok(())
    }
}

impl<D, SPI, RST> OriginDimensions for RM690B0<D, SPI, RST>
where
    D: DelayNs,
    SPI: esp_hal::spi::master::HalfDuplexReadWrite,
    RST: OutputPin,
{
    fn size(&self) -> embedded_graphics::prelude::Size {
        (600, 450).into()
    }
}

impl<D, SPI, RST> DrawTarget for RM690B0<D, SPI, RST>
where
    D: DelayNs,
    SPI: esp_hal::spi::master::HalfDuplexReadWrite,
    RST: OutputPin,
{
    type Error = anyhow::Error;
    type Color = Rgb565;

    fn draw_iter<I>(&mut self, pixels: I) -> core::result::Result<(), Self::Error>
    where
        I: IntoIterator<Item = embedded_graphics::Pixel<Self::Color>>,
    {
        for pixel in pixels {
            let size = self.size();
            let Pixel(point, color) = pixel;
            let idx = ((point.x + (point.y * size.width as i32)) * 2) as usize;

            if idx < self.buf.len() {
                let [a, b] = color.to_be_bytes();
                self.buf[idx] = a;
                self.buf[idx + 1] = b;
            }
        }

        Ok(())
    }
}
