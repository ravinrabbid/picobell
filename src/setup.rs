use defmt::*;
use embassy_rp::dma;
use embassy_rp::gpio::Pin;
use embassy_rp::interrupt::typelevel::Binding;
use embassy_rp::pio;
use embassy_rp::pio::Pio;
use embassy_rp::spi;
use embassy_rp::spi::Spi;
use embassy_rp::Peripheral;

use crate::cc1101;
use crate::cc1101::Cc1101;
use crate::leds;
use crate::leds::Leds;
use crate::pio_honeywell::PioHoneywell;

pub fn setup_leds<'d, PIO: pio::Instance, const N: usize>(
    config: leds::Config,
    pio: impl Peripheral<P = PIO> + 'd,
    pio_interrupt: impl Binding<PIO::Interrupt, pio::InterruptHandler<PIO>>,
    dma: impl Peripheral<P = impl dma::Channel> + 'd,
    pin: impl pio::PioPin,
) -> Leds<'d, PIO, 0, N> {
    let Pio {
        mut common, sm0, ..
    } = Pio::new(pio, pio_interrupt);

    Leds::new(config, &mut common, sm0, dma, pin)
}

#[allow(clippy::too_many_arguments)]
pub fn setup_radio<'d, SPI: spi::Instance, PIO: pio::Instance>(
    frequency: u64,
    frequency_offest: i64,
    spi: impl Peripheral<P = SPI> + 'd,
    spi_clk: impl spi::ClkPin<SPI>,
    spi_mosi: impl spi::MosiPin<SPI>,
    spi_miso: impl spi::MisoPin<SPI>,
    spi_csn: impl Pin,
    pio: impl Peripheral<P = PIO> + 'd,
    pio_interrupt: impl Binding<PIO::Interrupt, pio::InterruptHandler<PIO>>,
    serial_clk: impl pio::PioPin,
    serial_data: impl pio::PioPin,
) -> (Cc1101<'d, SPI, spi::Blocking>, PioHoneywell<'d, PIO, 0>) {
    let Pio { common, sm0, .. } = Pio::new(pio, pio_interrupt);
    let honeywell = PioHoneywell::new(common, sm0, serial_data, serial_clk);

    let mut spi_config = spi::Config::default();
    spi_config.frequency = 8_000_000;
    let spi = Spi::new_blocking(spi, spi_clk, spi_mosi, spi_miso, spi_config);
    let mut radio = Cc1101::new(spi, spi_csn);

    unwrap!(radio.reset());
    unwrap!(radio.wait_for_chip_ready());
    unwrap!(radio.set_mode(cc1101::Mode::Idle));

    info!(
        "Found cc1101 Version {=u8}",
        unwrap!(radio.read_register(cc1101::Register::VERSION))
    );

    unwrap!(radio.set_frequency(frequency));
    unwrap!(radio.set_frequency_offset(frequency_offest));
    unwrap!(radio.set_deviation(50_000));
    unwrap!(radio.set_bitrate(6250));
    unwrap!(radio.set_channel_bandwidth(116_000));

    unwrap!(radio.write_register(cc1101::Register::IOCFG2, 0x0B)); // GDO2 Sync clock
    unwrap!(radio.write_register(cc1101::Register::IOCFG0, 0x0C)); // GDO0 Sync serial data
    unwrap!(radio.write_register(cc1101::Register::PKTCTRL0, 0b00010010)); // Sync, continious, no whitening
    unwrap!(radio.write_register(cc1101::Register::FSCTRL1, 0x06)); // IF Frequency, from SmartRF (maybe 0x08?)
    unwrap!(radio.write_register(cc1101::Register::MDMCFG2, 0x04)); // 2FSK, no preamble
    unwrap!(radio.write_register(cc1101::Register::MCSM0, 0x18)); // Calibrate on IDLE to RX/TX, PO_TIMEOUT 64
    unwrap!(radio.write_register(cc1101::Register::FOCCFG, 0x16)); // Frequency offset compensation, CS gate off , limit CHAN_BW/4
    unwrap!(radio.write_register(cc1101::Register::AGCCTRL2, 0x07)); // Max Gain, 42dB target
    unwrap!(radio.write_register(cc1101::Register::AGCCTRL1, 0x00)); // Decrease LNA2 first
    unwrap!(radio.write_register(cc1101::Register::AGCCTRL0, 0x91)); // Defaults
    unwrap!(radio.write_register(cc1101::Register::FREND1, 0x56)); // Defaults
    unwrap!(radio.write_register(cc1101::Register::FREND0, 0x10)); // Defaults
    unwrap!(radio.write_register(cc1101::Register::PATABLE, 0xC0)); // +12dBm (or 0xC5 +10dBm)

    (radio, honeywell)
}
