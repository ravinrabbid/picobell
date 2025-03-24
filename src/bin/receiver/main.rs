#![no_std]
#![no_main]

use arbitrary_int::u20;
use cc1101::Cc1101;
use config::{AssignedResources, BellResources, LedsResources, RfRessources};
use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{DMA_CH1, PIO0, PIO1, SPI0};
use embassy_rp::pio;
use embassy_rp::pio::Pio;
use embassy_rp::spi;
use embassy_rp::spi::Spi;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::Timer;
use leds::{Leds, Mode as LedMode};
use pio_honeywell::{Frame, PioHoneywell};
use {defmt_rtt as _, panic_probe as _};

use picobell::tasks::run_bell_task;
use picobell::{cc1101, common, leds, pio_honeywell};

mod config;

static RX_TRIGGER_WATCH: Watch<CriticalSectionRawMutex, Frame, 2> = Watch::new();

bind_interrupts!(struct IrqsPio0 {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
});

bind_interrupts!(struct IrqsPio1 {
    PIO1_IRQ_0 => pio::InterruptHandler<PIO1>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Picobell receiver starting ...");

    let p = embassy_rp::init(Default::default());
    let r = split_resources!(p);

    unwrap!(spawner.spawn(rf_task(r.rf, p.PIO0, p.SPI0)));
    unwrap!(spawner.spawn(bell_task(r.bell)));
    unwrap!(spawner.spawn(leds_task(r.leds, p.PIO1, p.DMA_CH1)));

    info!("Picobell receiver ready");
}

#[embassy_executor::task]
async fn rf_task(r: RfRessources, mut pio: PIO0, spi: SPI0) {
    let Pio { common, sm0, .. } = Pio::new(&mut pio, IrqsPio0);

    let mut honeywell = PioHoneywell::new(common, sm0, r.gdo0, r.gdo2);

    let mut spi_config = spi::Config::default();
    spi_config.frequency = 8_000_000;

    let spi = Spi::new_blocking(spi, r.clk, r.mosi, r.miso, spi_config);
    let mut radio = Cc1101::new(spi, r.csn);

    common::setup_radio(&mut radio, config::FREQENCY_OFFSET);

    info!("RF task started");

    let rx_trigger_sender = RX_TRIGGER_WATCH.sender();

    let mut repetitions = 0u8;
    let mut current_frame = None;

    loop {
        if let Err(e) = radio.set_mode(cc1101::Mode::Rx) {
            error!("Failed changing to Rx mode: {}", e);
            continue;
        }

        match select(honeywell.read_frame(), Timer::after_secs(250)).await {
            Either::First(frame) => {
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
                        rx_trigger_sender.send(frame);

                        repetitions = 0;
                        current_frame = None;

                        Timer::after(config::TRIGGER_COOLDOWN).await;
                    }
                }
            }
            Either::Second(_) => {
                debug!("Repetitions reset by timeout");

                repetitions = 0;
                current_frame = None;
            }
        }
    }
}

#[embassy_executor::task]
async fn bell_task(r: BellResources) {
    run_bell_task(
        r.pin,
        config::BELL_CONFIG,
        unwrap!(RX_TRIGGER_WATCH.receiver()),
    )
    .await;
}

#[embassy_executor::task]
async fn leds_task(r: LedsResources, pio: PIO1, dma: DMA_CH1) {
    let Pio {
        mut common, sm0, ..
    } = Pio::new(pio, IrqsPio1);
    let mut leds: Leds<'_, PIO1, 0, { config::LED_COUNT }> =
        Leds::new(config::LED_CONFIG, &mut common, sm0, dma, r.pin);

    let mut rx_receiver = unwrap!(RX_TRIGGER_WATCH.receiver());

    info!("Leds task started");

    let mut mode = LedMode::Idle;

    loop {
        match select(rx_receiver.changed(), leds.show(mode)).await {
            Either::First(frame) => mode = LedMode::Frame(frame),
            Either::Second(_) => defmt::unreachable!("leds.show() future should never resolve!"),
        }
    }
}
