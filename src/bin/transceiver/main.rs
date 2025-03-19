#![no_std]
#![no_main]

use arbitrary_int::u20;
use bell::Bell;
use button::Button;
use cc1101::{Cc1101, Register};
use config::{
    AssignedResources, BellResources, ButtonResources, LedsResources, RfRessources, FREQENCY_OFFSET,
};
use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::select::{select3, Either3};
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{DMA_CH1, PIO0, PIO1, SPI0, USB};
use embassy_rp::pio;
use embassy_rp::pio::Pio;
use embassy_rp::spi;
use embassy_rp::spi::Spi;
use embassy_rp::usb;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::Timer;
use leds::{Leds, Mode as LedMode};
use pio_honeywell::{Frame, PioHoneywell};
use {defmt_rtt as _, panic_probe as _};

use picobell::{bell, button, cc1101, leds, pio_honeywell};

mod config;

static BUTTON_TRIGGER_WATCH: Watch<CriticalSectionRawMutex, Frame, 2> = Watch::new();
static RX_TRIGGER_WATCH: Watch<CriticalSectionRawMutex, Frame, 2> = Watch::new();

bind_interrupts!(struct IrqsPio0 {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
});

bind_interrupts!(struct IrqsPio1 {
    PIO1_IRQ_0 => pio::InterruptHandler<PIO1>;
});

bind_interrupts!(struct IrqsUsb {
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Picobell starting ...");

    let p = embassy_rp::init(Default::default());
    let r = split_resources!(p);

    unwrap!(spawner.spawn(rf_task(r.rf, p.PIO0, p.SPI0)));
    unwrap!(spawner.spawn(button_task(r.button)));
    unwrap!(spawner.spawn(bell_task(r.bell)));
    unwrap!(spawner.spawn(leds_task(r.leds, p.PIO1, p.DMA_CH1)));

    info!("Picobell ready");
}

#[embassy_executor::task]
async fn rf_task(r: RfRessources, mut pio: PIO0, spi: SPI0) {
    info!("PIO");

    let Pio { common, sm0, .. } = Pio::new(&mut pio, IrqsPio0);

    let mut honeywell = PioHoneywell::new(common, sm0, r.gdo0, r.gdo2);

    let tx_watch = RX_TRIGGER_WATCH.sender();
    let mut button = unwrap!(BUTTON_TRIGGER_WATCH.receiver());

    let mut spi_config = spi::Config::default();
    spi_config.frequency = 8_000_000;

    let spi = Spi::new_blocking(spi, r.clk, r.mosi, r.miso, spi_config);
    let mut radio = Cc1101::new(spi, r.csn);

    unwrap!(radio.reset());
    unwrap!(radio.wait_for_chip_ready());
    unwrap!(radio.set_mode(cc1101::Mode::Idle));

    info!(
        "Found cc1101 Version {=u8}",
        unwrap!(radio.read_register(Register::VERSION))
    );

    unwrap!(radio.set_frequency(868_300_000));
    unwrap!(radio.set_frequency_offset(FREQENCY_OFFSET));
    unwrap!(radio.set_deviation(50_000));
    unwrap!(radio.set_bitrate(6250));
    unwrap!(radio.set_channel_bandwidth(116_000));

    unwrap!(radio.write_register(Register::IOCFG2, 0x0B)); // GDO2 Sync clock
    unwrap!(radio.write_register(Register::IOCFG0, 0x0C)); // GDO0 Sync serial data
    unwrap!(radio.write_register(Register::PKTCTRL0, 0b00010010)); // Sync, continious, no whitening
    unwrap!(radio.write_register(Register::FSCTRL1, 0x06)); // IF Frequency, from SmartRF (maybe 0x08?)
    unwrap!(radio.write_register(Register::MDMCFG2, 0x04)); // 2FSK, no preamble
    unwrap!(radio.write_register(Register::MCSM0, 0x18)); // Calibrate on IDLE to RX/TX, PO_TIMEOUT 64
    unwrap!(radio.write_register(Register::FOCCFG, 0x16)); // Frequency offset compensation, CS gate off , limit CHAN_BW/4
    unwrap!(radio.write_register(Register::AGCCTRL2, 0x07)); // Max Gain, 42dB target
    unwrap!(radio.write_register(Register::AGCCTRL1, 0x00)); // Decrease LNA2 first
    unwrap!(radio.write_register(Register::AGCCTRL0, 0x91)); // Defaults
    unwrap!(radio.write_register(Register::FREND1, 0x56)); // Defaults
    unwrap!(radio.write_register(Register::FREND0, 0x10)); // Defaults
    unwrap!(radio.write_register(Register::PATABLE, 0xC0)); // +12dBm (or 0xC5 +10dBm)

    info!("RF task started");

    let mut repetitions = 0u8;
    let mut current_frame = None;

    loop {
        if let Err(e) = radio.set_mode(cc1101::Mode::Rx) {
            error!("Failed changing to Rx mode: {}", e);
            continue;
        }

        match select3(
            button.changed(),
            honeywell.read_frame(),
            Timer::after_secs(250),
        )
        .await
        {
            Either3::First(frame) => {
                if let Err(e) = radio.set_mode(cc1101::Mode::Tx) {
                    error!("Failed changing to Tx mode: {}", e);
                } else {
                    info!("Sending frame {}", frame);

                    // Send frame a few times for redundancy (Original sender does 50)
                    honeywell.write_frame(&frame, 20).await;
                }
            }
            Either3::Second(frame) => {
                debug!("Received frame {}", frame);

                if Some(frame) != current_frame {
                    debug!("Frame is new");

                    repetitions = 0;
                    current_frame = Some(frame);
                } else {
                    repetitions += 1;
                    debug!("Received frame repetition {}", repetitions);

                    if (repetitions >= 3)
                        && (frame.id == config::RECEIVE_ID
                            || frame.id == u20::new(0)
                            || config::RECEIVE_ID == u20::new(0))
                    {
                        info!("Received frame matches");
                        tx_watch.send(frame);

                        repetitions = 0;
                        current_frame = None;

                        Timer::after(config::TRIGGER_COOLDOWN).await;
                    }
                }
            }
            Either3::Third(_) => {
                debug!("Repetitions reset by timeout");

                repetitions = 0;
                current_frame = None;
            }
        }
    }
}

#[embassy_executor::task]
async fn button_task(r: ButtonResources) {
    let mut button = Button::new(r.pin, config::BUTTON_CONFIG);
    let sender = BUTTON_TRIGGER_WATCH.sender();

    info!("Button task started");

    let frame = Frame {
        id: config::SEND_ID,
        alert: pio_honeywell::AlertType::Normal,
        low_battery: false,
        ..Default::default()
    };

    loop {
        button.wait_for_press().await;
        info!("Button pressed");

        sender.send(frame);

        Timer::after(config::TRIGGER_COOLDOWN).await;
    }
}

#[embassy_executor::task]
async fn bell_task(r: BellResources) {
    let mut bell = Bell::new(r.pin, config::BELL_CONFIG);
    let mut receiver = unwrap!(RX_TRIGGER_WATCH.receiver());

    info!("Bell task started");

    loop {
        receiver.changed().await;

        bell.trigger().await;
        info!("Bell triggered");
    }
}

#[embassy_executor::task]
async fn leds_task(r: LedsResources, pio: PIO1, dma: DMA_CH1) {
    let Pio {
        mut common, sm0, ..
    } = Pio::new(pio, IrqsPio1);
    let mut leds: Leds<'_, PIO1, 0, { config::LED_COUNT }> =
        Leds::new(config::LED_CONFIG, &mut common, sm0, dma, r.pin);

    let mut rx_receiver = unwrap!(RX_TRIGGER_WATCH.receiver());
    let mut button_receiver = unwrap!(BUTTON_TRIGGER_WATCH.receiver());

    info!("Leds task started");

    let mut mode = LedMode::Idle;

    loop {
        match select3(
            rx_receiver.changed(),
            button_receiver.changed(),
            leds.show(mode),
        )
        .await
        {
            Either3::First(frame) => mode = LedMode::Frame(frame),
            Either3::Second(_) => mode = LedMode::ButtonPress,
            Either3::Third(_) => defmt::unreachable!("leds.show() future should never resolve!"),
        }
    }
}
