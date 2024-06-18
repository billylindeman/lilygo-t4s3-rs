use anyhow::Result;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use esp_hal::spi::{
    master::{Address, Command, HalfDuplexReadWrite, Spi},
    SpiDataMode, SpiMode,
};

#[derive(Debug, Clone, Copy)]
pub enum DisplayCommand {
    Nop = 0x00,                                   // No Operation
    SwReset = 0x01,                               // Software Reset
    SleepIn = 0x10,                               // Sleep In
    SleepOut = 0x11,                              // Sleep Out
    PartialDisplayModeOn = 0x12,                  // Partial Display Mode On
    NormalDisplayModeOn = 0x13,                   // Normal Display Mode On
    InversionOff = 0x20,                          // Display Inversion Off
    InversionOn = 0x21,                           // Display Inversion On
    DisplayOff = 0x28,                            // Display Off
    DisplayOn = 0x29,                             // Display On
    ColumnAddressSet = 0x2A,                      // Column Address Set
    PageAddressSet = 0xFE,                        // Page Address Set
    MemoryWrite = 0x2C,                           // Memory Write
    MemoryRead = 0x2E,                            // Memory Read
    PartialArea = 0x30,                           // Partial Area
    VerticalScrollingDefinition = 0x33,           // Vertical Scrolling Definition
    TearingEffectLineOff = 0x34,                  // Tearing Effect Line Off
    TearingEffectLineOn = 0x35,                   // Tearing Effect Line On
    MemoryAccessControl = 0x36,                   // Memory Access Control
    IdleModeOff = 0x38,                           // Idle Mode Off
    IdleModeOn = 0x39,                            // Idle Mode On
    InterfacePixelFormat = 0x3A,                  // Interface Pixel Format
    WriteMemoryContinue = 0x3C,                   // Write Memory Continue
    ReadMemoryContinue = 0x3E,                    // Read Memory Continue
    WriteDisplayBrightness = 0x51,                // Write Display Brightness
    ReadDisplayBrightness = 0x52,                 // Read Display Brightness
    WriteControlDisplay = 0x53,                   // Write Control Display
    ReadControlDisplay = 0x54,                    // Read Control Display
    WriteContentAdaptiveBrightnessControl = 0x55, // Write Content Adaptive Brightness Control
    ReadContentAdaptiveBrightnessControl = 0x56,  // Read Content Adaptive Brightness Control
    WriteCABCMinimumBrightness = 0x5E,            // Write CABC Minimum Brightness
    ReadCABCMinimumBrightness = 0x5F,             // Read CABC Minimum Brightness
    ReadID1 = 0xDA,                               // Read ID1
    ReadID2 = 0xDB,                               // Read ID2
    ReadID3 = 0xDC,                               // Read ID3
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
        self.write_command(DisplayCommand::NormalDisplayModeOn)?;
        self.delay.delay_ms(50);

        Ok(())
    }

    fn write_command(&mut self, cmd: DisplayCommand) -> Result<()> {
        let mut data = [0u8; 2];
        self.spi
            .write(
                SpiDataMode::Single,
                Command::Command8(cmd as u16, SpiDataMode::Single),
                Address::Address24(0x000000, SpiDataMode::Single),
                0,
                &mut data,
            )
            .map_err(|e| anyhow::format_err!("spi err"))
    }
}
