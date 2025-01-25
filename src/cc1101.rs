use defmt::{debug, Format};
use embassy_rp::gpio::{Level, Output, Pin};
use embassy_rp::spi::{Error as SpiError, Instance, Mode as SpiMode, Spi};
use embassy_rp::Peripheral;
use thiserror::Error;

const FXOSC: u64 = 26_000_000;
static RXBW_VALUES: &'static [u32] = &[
    58_000, 68_000, 81_000, 102_000, 116_000, 135_000, 162_000, 203_000, 232_000, 270_000, 325_000,
    406_000, 464_000, 541_000, 650_000, 812_000,
];

#[derive(Error, Format, Debug)]
pub enum Error {
    #[error("SPI error")]
    Spi,
    #[error("Cannot write to write-only register.")]
    WriteOnly,
}

impl From<SpiError> for Error {
    fn from(_value: SpiError) -> Self {
        Self::Spi
    }
}

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

pub enum Mode {
    Idle,
    Rx,
    Tx,
}

struct Cc1101<'d, SPI: Instance, M: SpiMode> {
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
        if reg >= Register::PARTNUM {
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
            Mode::Rx => self.strobe_register(StrobeRegister::SRX),
            Mode::Tx => self.strobe_register(StrobeRegister::STX),
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

    pub fn set_frequency(&mut self, hz: u64) {
        let freq = hz * 1u64.rotate_left(16) / FXOSC;
        let freq0 = (freq & 0xff) as u8;
        let freq1 = ((freq >> 8) & 0xff) as u8;
        let freq2 = ((freq >> 16) & 0xff) as u8;

        debug!("Changing frequency to {=u64}Hz", freq);

        self.write_register(Register::FREQ2, freq2);
        self.write_register(Register::FREQ1, freq1);
        self.write_register(Register::FREQ0, freq0);
    }

    // TODO
    //...

    pub fn set_deviation(&mut self, hz: u64) {
        let target = hz / (FXOSC / (1 << 17));

        let origin = (8 * FXOSC) / (1 << 17);

        let mut exponent = 7; // max
        let mut mantissa = 0;

        while exponent >= 0 {
            let interval_start = (1 << exponent) * origin;

            if hz >= interval_start {
                let step_size = interval_start / 8;

                mantissa = (hz - interval_start) / step_size;

                break;
            }

            exponent -= 1;
        }

        info!("Deviation Exp: {} Mant: {}", exponent, mantissa);

        self.write_register(
            0x15,
            (((exponent & 0x7) as u8) << 4) | ((mantissa & 0x7) as u8),
        );
    }

    pub fn calculate_rx_bandwidth(&self, freq: u64, bitrate: u64) -> u32 {
        // Uncertainty ~ +/- 40ppm for a cheap CC1101
        // Uncertainty * 2 for both transmitter and receiver
        let uncertainty = ((freq as f64 / 1_000_000.) * 40. * 2.) as u64;
        let minbw = bitrate + uncertainty;

        for &val in RXBW_VALUES {
            if val as u64 > minbw {
                return val;
            }
        }

        0
    }

    pub fn set_rx_bandwidth(&mut self, hz: u32) {
        info!("RxBw {}", hz);
        for e in (0..4).rev() {
            for m in (0..4).rev() {
                let point = FXOSC as i64 / (8 * (m + 4) * (1 << e));

                if (hz as i64 - point).abs() <= 1000 {
                    info!("RxBw {} Exp: {} Mant: {}", hz, e, m);
                    let current = self.read_register(0x10);
                    self.write_register(0x10, (((e << 6) | (m << 4)) as u8) | (current & 0x0F))
                }
            }
        }
    }

    pub fn set_bitrate(&mut self, bitrate: u64) {
        // TODO imprecise
        let origin = (256 * FXOSC) / (1 << 28);

        let mut exponent = 14; // max
        let mut mantissa = 0;

        while exponent >= 0 {
            let interval_start = (1 << exponent) * origin;

            if bitrate >= interval_start {
                let step_size = interval_start / 256;

                mantissa = (bitrate - interval_start) / step_size;

                break;
            }

            exponent -= 1;
        }

        info!("Bitrate Exp: {} Mant: {}", exponent, mantissa);

        let current = self.read_register(0x10);
        self.write_register(0x10, exponent as u8 | (current & 0xF0));
        self.write_register(0x11, mantissa as u8);
    }
}
