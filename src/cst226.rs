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
    ReadChecksum = 0xFC,
    ReadResolution = 0xF8,
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
            .expect("could not write register");
        self.delay.delay_ms(20);

        let mut buf: [u8; 4] = [0u8; 4];

        self.i2c
            .write_read(
                CST226SE_SLAVE_ADDRESS,
                &[Registers::Command as u8, Command::ReadChecksum as u8],
                &mut buf,
            )
            .expect("could not read checksum");
        log::info!("checksum {:x?}", u32::from_le_bytes(buf));

        let mut buf: [u8; 4] = [0u8; 4];
        self.i2c
            .write_read(
                CST226SE_SLAVE_ADDRESS,
                &[Registers::Command as u8, Command::ReadResolution as u8],
                &mut buf,
            )
            .expect("could not read checksum");

        let res_x: u16 = u16::from_le_bytes([buf[0], buf[1]]);
        let res_y: u16 = u16::from_le_bytes([buf[2], buf[3]]);

        log::info!("res_x={}, res_y={}", res_x, res_y);

        //todo
        // read chiptype
        // read fwversion
        // read checksum

        Ok(())
    }
}
