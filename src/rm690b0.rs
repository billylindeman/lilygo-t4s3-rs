use alloc::vec;
use anyhow::Result;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use esp_hal::spi::{
    master::{Address, Command, HalfDuplexReadWrite, Spi},
    SpiDataMode, SpiMode,
};

#[derive(Debug, Clone, Copy)]
pub enum DisplayCommand {
    Nop = 0x00,                                     // No Operation
    SwReset = 0x0100,                               // Software Reset
    SleepIn = 0x1000,                               // Sleep In
    SleepOut = 0x1100,                              // Sleep Out
    PartialDisplayModeOn = 0x001200,                // Partial Display Mode On
    NormalDisplayModeOn = 0x001300,                 // Normal Display Mode On
    AllPixelsOn = 0x2200,                           // Display Inversion Off
    InversionOn = 0x2100,                           // Display Inversion On
    DisplayOff = 0x2800,                            // Display Off
    DisplayOn = 0x2900,                             // Display On
    ColumnAddressSet = 0x2A00,                      // Column Address Set
    PageAddressSet = 0xFE00,                        // Page Address Set
    MemoryWrite = 0x2C00,                           // Memory Write
    MemoryRead = 0x2E00,                            // Memory Read
    PartialArea = 0x3000,                           // Partial Area
    VerticalScrollingDefinition = 0x3300,           // Vertical Scrolling Definition
    TearingEffectLineOff = 0x3400,                  // Tearing Effect Line Off
    TearingEffectLineOn = 0x3500,                   // Tearing Effect Line On
    MemoryAccessControl = 0x3600,                   // Memory Access Control
    IdleModeOff = 0x3800,                           // Idle Mode Off
    IdleModeOn = 0x3900,                            // Idle Mode On
    InterfacePixelFormat = 0x3A00,                  // Interface Pixel Format
    WriteMemoryContinue = 0x3C00,                   // Write Memory Continue
    ReadMemoryContinue = 0x3E00,                    // Read Memory Continue
    WriteDisplayBrightness = 0x5100,                // Write Display Brightness
    ReadDisplayBrightness = 0x5200,                 // Read Display Brightness
    WriteControlDisplay = 0x5300,                   // Write Control Display
    ReadControlDisplay = 0x5400,                    // Read Control Display
    WriteContentAdaptiveBrightnessControl = 0x5500, // Write Content Adaptive Brightness Control
    ReadContentAdaptiveBrightnessControl = 0x5600,  // Read Content Adaptive Brightness Control
    WriteCABCMinimumBrightness = 0x5E00,            // Write CABC Minimum Brightness
    ReadCABCMinimumBrightness = 0x5F00,             // Read CABC Minimum Brightness
    ReadID1 = 0xDA00,                               // Read ID1
    ReadID2 = 0xDB00,                               // Read ID2
    ReadID3 = 0xDC00,                               // Read ID3
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
}
impl<D, SPI, RST> RM690B0<D, SPI, RST>
where
    D: DelayNs,
    SPI: esp_hal::spi::master::HalfDuplexReadWrite,
    RST: OutputPin,
{
    pub fn new(delay: D, spi: SPI, rst: RST) -> Self {
        Self { delay, spi, rst }
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

        self.write_command(0xFE, 0x20, 0x01)?;
        self.write_command(0x26, 0x0A, 0x01)?;
        self.write_command(0x24, 0x80, 0x01)?; //SPI write RAM
        self.write_command(0x5A, 0x51, 0x01)?; // 230918:SWIRE FOR BV6804
        self.write_command(0x5B, 0x2E, 0x01)?; // 230918:SWIRE FOR BV6804
        self.write_command(0xFE, 0x00, 0x01)?; //SET PAGE
        self.write_command(0x3A, 0x55, 0x01)?; //Interface Pixel Format    16bit/pixel
        self.write_command(0xC2, 0x00, 0x21)?; //delay_ms(10);
        self.write_command(0x35, 0x00, 0x01)?; //TE ON
        self.write_command(0x51, 0x00, 0x01)?; //Write Display Brightness  MAX_VAL=0XFF
        self.write_command(0x11, 0x00, 0x80)?; //Sleep Out delay_ms(120);
        self.write_command(0x29, 0x00, 0x20)?; //Display on delay_ms(10);
        self.write_command(0x51, 0xFF, 0x01)?; //Write Display Brightness  MAX_VAL=0XFF

        self.write_command(0x23, 0x0, 0x0)?; //All Pixels On

        Ok(())
    }

    fn write_command(&mut self, cmd: u8, param: u8, len: usize) -> Result<()> {
        let mut data = vec![param; len & 0x1F];

        log::info!(
            "write_command addr=0x{:06x} param=0x{:02x} len=0x{:02x}",
            (cmd as u32) << 8,
            param,
            len
        );
        self.spi
            .write(
                SpiDataMode::Single,
                Command::Command8(0x02, SpiDataMode::Single),
                Address::Address24((cmd as u32) << 8, SpiDataMode::Single),
                0,
                &mut data,
            )
            .map_err(|e| anyhow::format_err!("spi error"))?;

        if (len & 0x80) != 0 {
            //self.delay.delay_ms(120);
        } else if (len & 0x20) != 0 {
            //self.delay.delay_ms(10);
        }

        Ok(())
    }
}
