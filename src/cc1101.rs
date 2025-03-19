use defmt::*;
use embassy_rp::gpio::{Level, Output, Pin};
use embassy_rp::spi::{Error as SpiError, Instance, Mode as SpiMode, Spi};
use embassy_rp::Peripheral;
use libm::{floor, log2, round};
use thiserror::Error;

const FXOSC: u64 = 26_000_000;
const MAX_DEVIATION: u64 = 380_160;
const MAX_BITRATE: u64 = 1_621_826;

static CHANBW_VALUES: &[u64] = &[
    58_000, 68_000, 81_000, 102_000, 116_000, 135_000, 162_000, 203_000, 232_000, 270_000, 325_000,
    406_000, 464_000, 541_000, 650_000, 812_000,
];

#[derive(Error, Format, Debug)]
pub enum Error {
    #[error("SPI error")]
    Spi,
    #[error("Cannot write to write-only register.")]
    WriteOnly,
    #[error("Target value exceeds maxmimum of {0}.")]
    ValueToLarge(u64),
}

impl From<SpiError> for Error {
    fn from(_value: SpiError) -> Self {
        Self::Spi
    }
}

#[allow(non_camel_case_types)]
#[derive(Debug, PartialEq, PartialOrd)]
pub enum Register {
    // Burst capable read/write registers
    IOCFG2 = 0x00,
    IOCFG1 = 0x01,
    IOCFG0 = 0x02,
    FIFOTHR = 0x03,
    SYNC1 = 0x04,
    SYNC0 = 0x05,
    PKTLEN = 0x06,
    PKTCTRL1 = 0x07,
    PKTCTRL0 = 0x08,
    ADDR = 0x09,
    CHANNR = 0x0A,
    FSCTRL1 = 0x0B,
    FSCTRL0 = 0x0C,
    FREQ2 = 0x0D,
    FREQ1 = 0x0E,
    FREQ0 = 0x0F,
    MDMCFG4 = 0x10,
    MDMCFG3 = 0x11,
    MDMCFG2 = 0x12,
    MDMCFG1 = 0x13,
    MDMCFG0 = 0x14,
    DEVIATN = 0x15,
    MCSM2 = 0x16,
    MCSM1 = 0x17,
    MCSM0 = 0x18,
    FOCCFG = 0x19,
    BSCFG = 0x1A,
    AGCCTRL2 = 0x1B,
    AGCCTRL1 = 0x1C,
    AGCCTRL0 = 0x1D,
    WOREVT1 = 0x1E,
    WOREVT0 = 0x1F,
    WORCTRL = 0x20,
    FREND1 = 0x21,
    FREND0 = 0x22,
    FSCAL3 = 0x23,
    FSCAL2 = 0x24,
    FSCAL1 = 0x25,
    FSCAL0 = 0x26,
    RCCTRL1 = 0x27,
    RCCTRL0 = 0x28,
    FSTEST = 0x29,
    PTEST = 0x2A,
    AGCTEST = 0x2B,
    TEST2 = 0x2C,
    TEST1 = 0x2D,
    TEST0 = 0x2E,
    // Read only registers
    PARTNUM = 0x30,
    VERSION = 0x31,
    FREQEST = 0x32,
    LQI = 0x33,
    RSSI = 0x34,
    MARCSTATE = 0x35,
    WORTIME1 = 0x36,
    WORTIME0 = 0x37,
    PKTSTATUS = 0x38,
    VCO_VC_DAC = 0x39,
    TXBYTES = 0x3A,
    RXBYTES = 0x3B,
    RCCTRL1_STATUS = 0x3C,
    RCCTRL0_STATUS = 0x3D,
    // Multi byte registers
    PATABLE = 0x3E,
    TX_RX_FIFO = 0x3F,
}

#[derive(PartialEq, PartialOrd)]
pub enum StrobeRegister {
    SRES = 0x30,
    SFSTXON = 0x31,
    SXOFF = 0x32,
    SCAL = 0x33,
    SRX = 0x34,
    STX = 0x35,
    SIDLE = 0x36,
    SWOR = 0x38,
    SPWD = 0x39,
    SFRX = 0x3A,
    SFTX = 0x3B,
    SWORRST = 0x3C,
    SNOP = 0x3D,
}

#[derive(Format)]
pub enum Mode {
    Idle,
    Rx,
    Tx,
}

pub struct Cc1101<'d, SPI: Instance, M: SpiMode> {
    spi: Spi<'d, SPI, M>,
    cs: Output<'d>,
}

impl<'d, SPI: Instance, M: SpiMode> Cc1101<'d, SPI, M> {
    pub fn new(spi: Spi<'d, SPI, M>, cs_pin: impl Peripheral<P = impl Pin> + 'd) -> Self {
        let cs = Output::new(cs_pin, Level::High);

        Self { spi, cs }
    }

    fn with_cs<F, R>(&mut self, f: F) -> Result<R, Error>
    where
        F: FnOnce(&mut Spi<'d, SPI, M>) -> Result<R, Error>,
    {
        self.cs.set_low();
        let result = f(&mut self.spi);
        self.cs.set_high();

        result
    }

    pub fn strobe_register(&mut self, reg: StrobeRegister) -> Result<(), Error> {
        self.with_cs(|spi: &mut Spi<'d, SPI, M>| {
            let buf = [reg as u8];
            spi.blocking_write(&buf).map_err(Into::into)
        })
    }

    pub fn write_register(&mut self, reg: Register, value: u8) -> Result<(), Error> {
        if reg >= Register::PARTNUM && reg <= Register::RCCTRL0_STATUS {
            return Err(Error::WriteOnly);
        }

        self.with_cs(|spi: &mut Spi<'d, SPI, M>| {
            let buf = [reg as u8, value];
            spi.blocking_write(&buf).map_err(Into::into)
        })
    }

    pub fn read_register(&mut self, reg: Register) -> Result<u8, Error> {
        let address = if reg >= Register::PARTNUM {
            reg as u8 | 0xC0
        } else {
            reg as u8 | 0x80
        };

        self.with_cs(|spi: &mut Spi<'d, SPI, M>| {
            let mut buf = [address, 0x00];
            spi.blocking_transfer_in_place(&mut buf)
                .map(|_| buf[1])
                .map_err(Into::into)
        })
    }

    pub fn wait_for_chip_ready(&mut self) -> Result<(), Error> {
        while self.with_cs(|spi: &mut Spi<'d, SPI, M>| {
            let mut buf = [StrobeRegister::SNOP as u8];
            spi.blocking_transfer_in_place(&mut buf)
                .map(|_| (buf[0] & 0x80) != 0)
                .map_err(Into::into)
        })? {}

        Ok(())
    }

    pub fn reset(&mut self) -> Result<(), Error> {
        debug!("Resetting...");

        self.strobe_register(StrobeRegister::SRES)?;
        self.wait_for_chip_ready()?;

        debug!("Reset done.");

        Ok(())
    }

    pub fn set_mode(&mut self, mode: Mode) -> Result<(), Error> {
        debug!("Changing mode to {}...", mode);

        match mode {
            Mode::Idle => self.strobe_register(StrobeRegister::SIDLE),
            Mode::Rx => {
                self.set_mode(Mode::Idle)?;
                self.strobe_register(StrobeRegister::SRX)
            }
            Mode::Tx => {
                self.set_mode(Mode::Idle)?;
                self.strobe_register(StrobeRegister::STX)
            }
        }?;

        // Wait until state reached
        while (self.read_register(Register::MARCSTATE)? & 0x1F)
            != match mode {
                Mode::Idle => 0x01,
                Mode::Rx => 0x0D,
                Mode::Tx => 0x13,
            }
        {}

        debug!("Changed mode to {}.", mode);

        Ok(())
    }

    pub fn set_frequency(&mut self, hz: u64) -> Result<(), Error> {
        let freq = hz * 1u64.rotate_left(16) / FXOSC;
        let freq0 = (freq & 0xff) as u8;
        let freq1 = ((freq >> 8) & 0xff) as u8;
        let freq2 = ((freq >> 16) & 0xff) as u8;

        debug!("Setting frequency to {=u64}Hz", freq);

        self.write_register(Register::FREQ2, freq2)?;
        self.write_register(Register::FREQ1, freq1)?;
        self.write_register(Register::FREQ0, freq0)
    }

    /// Calculate exponent and mantissa register values accoring to datasheet p.35.
    ///
    /// This can be used for deviation, data rate and channel spacing.
    fn calculate_exponent_and_mantissa(
        target: u64,
        mantissa_offset_exponent: u8,
        divisor_exponent: u8,
    ) -> (u8, u8) {
        let e = floor(log2(
            (target * (1 << (divisor_exponent - mantissa_offset_exponent))) as f64 / FXOSC as f64,
        )) as u8;
        let m = round(
            (target * (1 << divisor_exponent) as u64) as f64 / ((FXOSC * (1 << e) as u64) as f64)
                - (1 << mantissa_offset_exponent) as f64,
        ) as u16;

        if m >= (1 << mantissa_offset_exponent) {
            (e + 1, 0)
        } else {
            (e, m as u8)
        }
    }

    pub fn set_deviation(&mut self, hz: u64) -> Result<(), Error> {
        if hz > MAX_DEVIATION {
            return Err(Error::ValueToLarge(MAX_DEVIATION));
        }

        let (e, m) = Self::calculate_exponent_and_mantissa(hz, 3, 17);

        debug!(
            "Setting deviation to {=u64}Hz (Eponnent: {=u8}, Mantissa: {=u8})",
            hz, e, m
        );

        self.write_register(Register::DEVIATN, ((e & 0x7) << 4) | (m & 0x7))
    }

    pub fn set_bitrate(&mut self, bitrate: u64) -> Result<(), Error> {
        if bitrate > MAX_BITRATE {
            return Err(Error::ValueToLarge(MAX_BITRATE));
        }

        let (e, m) = Self::calculate_exponent_and_mantissa(bitrate, 8, 28);

        debug!(
            "Setting bitrate to {=u64}baud (Eponnent: {=u8}, Mantissa: {=u8})",
            bitrate, e, m
        );

        let current = self.read_register(Register::MDMCFG4)?;
        self.write_register(Register::MDMCFG4, e | (current & 0xF0))?;
        self.write_register(Register::MDMCFG3, m)
    }

    pub fn set_channel_bandwidth(&mut self, hz: u64) -> Result<(), Error> {
        for (idx, &bandwidth) in CHANBW_VALUES.iter().enumerate() {
            if hz <= bandwidth {
                let e = (((CHANBW_VALUES.len() - 1) - idx) / 4) as u8;
                let m = (((CHANBW_VALUES.len() - 1) - idx) % 4) as u8;

                debug!(
                    "Setting channel bandwidth to {=u64}Hz (Eponnent: {=u8}, Mantissa: {=u8})",
                    hz, e, m
                );

                let current = self.read_register(Register::MDMCFG4)?;
                return self
                    .write_register(Register::MDMCFG4, ((e << 6) | (m << 4)) | (current & 0x0F));
            }
        }

        Err(Error::ValueToLarge(
            *CHANBW_VALUES
                .last()
                .expect("CHANBW_VALUES is static non-empty"),
        ))
    }

    pub fn set_frequency_offset(&mut self, hz: i64) -> Result<(), Error> {
        let offset = hz / (FXOSC / (1 << 14)) as i64;

        self.write_register(Register::FSCTRL0, offset as u8)
    }
}
