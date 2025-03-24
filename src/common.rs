use defmt::*;
use embassy_rp::spi;

use crate::cc1101;
use crate::cc1101::Cc1101;

pub fn setup_radio<SPI: spi::Instance, M: spi::Mode>(
    radio: &mut Cc1101<SPI, M>,
    frequency_offest: i64,
) {
    unwrap!(radio.reset());
    unwrap!(radio.wait_for_chip_ready());
    unwrap!(radio.set_mode(cc1101::Mode::Idle));

    info!(
        "Found cc1101 Version {=u8}",
        unwrap!(radio.read_register(cc1101::Register::VERSION))
    );

    unwrap!(radio.set_frequency(868_300_000));
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
}
