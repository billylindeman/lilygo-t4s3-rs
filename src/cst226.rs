use embedded_hal::{delay::DelayNs, digital::OutputPin, i2c::I2c};

pub enum TouchEvent {
    TouchDown(u16, u16),
}

pub trait TouchProvider {
    fn touch() -> Option<TouchEvent>;
}

enum Registers {
    Status = 0x00,
    Command = 0xD1,
}

pub enum Command {
    CommandModeEnter = 0x01,
    ReadChipCheckcode = 0xFC,
    ReadResolution = 0xF8,
    ReadChipTypeAndProjectId = 0x04,
    ReadFirmwareVersionAndChecksum = 0x08,
    CommandModeExit = 0x09,
}

const CST226SE_SLAVE_ADDRESS: u8 = 0x5A;

pub struct CST226<I2C: I2c, RST: OutputPin, D: DelayNs> {
    i2c: I2C,
    reset: RST,
    delay: D,
}

impl<I2C, RST, D> CST226<I2C, RST, D>
where
    I2C: I2c,
    RST: OutputPin,
    D: DelayNs,
{
    pub fn new(i2c: I2C, reset: RST, delay: D) -> Self {
        Self { i2c, reset, delay }
    }

    pub fn reset(&mut self) -> anyhow::Result<()> {
        self.reset
            .set_low()
            .map_err(|_| anyhow::format_err!("output pin error"))?;
        self.delay.delay_ms(100);

        self.reset
            .set_high()
            .map_err(|_| anyhow::format_err!("output pin error"))?;
        self.delay.delay_ms(100);

        Ok(())
    }

    pub fn init(&mut self) -> anyhow::Result<()> {
        // Enter Command Mode
        self.i2c
            .write(
                CST226SE_SLAVE_ADDRESS,
                &[Registers::Command as u8, Command::CommandModeEnter as u8],
            )
            .expect("cst226: could not enter command mode");
        self.delay.delay_ms(20);

        let mut buf: [u8; 4] = [0u8; 4];

        self.i2c
            .write_read(
                CST226SE_SLAVE_ADDRESS,
                &[Registers::Command as u8, Command::ReadChipCheckcode as u8],
                &mut buf,
            )
            .expect("cst226: could not read checkcode");

        let checkcode = u32::from_le_bytes(buf);
        log::info!("cst226: checkcode {:x?}", checkcode);

        let mut buf: [u8; 4] = [0u8; 4];
        self.i2c
            .write_read(
                CST226SE_SLAVE_ADDRESS,
                &[Registers::Command as u8, Command::ReadResolution as u8],
                &mut buf,
            )
            .expect("cst227: could not read resolution");

        let res_x: u16 = u16::from_le_bytes([buf[0], buf[1]]);
        let res_y: u16 = u16::from_le_bytes([buf[2], buf[3]]);

        log::info!("cst226: res_x={}, res_y={}", res_x, res_y);

        //todo
        // read chiptype

        let mut buf: [u8; 4] = [0u8; 4];
        self.i2c
            .write_read(
                CST226SE_SLAVE_ADDRESS,
                &[
                    Registers::Command as u8,
                    Command::ReadChipTypeAndProjectId as u8,
                ],
                &mut buf,
            )
            .expect("cst226: could not read chip_type and project_id");

        let project_id: u16 = u16::from_le_bytes([buf[0], buf[1]]);
        let chip_type: u16 = u16::from_le_bytes([buf[2], buf[3]]);

        log::info!(
            "cst226: project_id={:x}, chiptype={:x}",
            project_id,
            chip_type
        );

        let mut buf: [u8; 8] = [0u8; 8];
        self.i2c
            .write_read(
                CST226SE_SLAVE_ADDRESS,
                &[
                    Registers::Command as u8,
                    Command::ReadFirmwareVersionAndChecksum as u8,
                ],
                &mut buf,
            )
            .expect("cst226: could not read firmware_version and checksum");

        let firmware_version: u32 = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
        let checksum: u32 = u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]);

        log::info!(
            "cst226: firmware_version={:x}, checksum={:x}",
            firmware_version,
            checksum
        );

        if firmware_version == 0xA5A5A5A5 {
            anyhow::bail!("cst226: chip ic does not have firmware");
        }
        if (checkcode & 0xFFFF0000) != 0xCACA0000 {
            anyhow::bail!("cst226: chip ic does not have firmware");
        }

        self.i2c
            .write(
                CST226SE_SLAVE_ADDRESS,
                &[Registers::Command as u8, Command::CommandModeExit as u8],
            )
            .expect("cst226: could not exit command mode");

        Ok(())
    }
}