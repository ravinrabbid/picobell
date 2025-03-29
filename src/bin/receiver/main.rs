#![no_std]
#![no_main]

use arbitrary_int::u20;
use config::{AssignedResources, BellResources, LedsResources, RfRessources};
use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{DMA_CH1, PIO0, PIO1, SPI0};
use embassy_rp::pio;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::Timer;
use pio_honeywell::Frame;
use {defmt_rtt as _, panic_probe as _};

use picobell::tasks::run_bell_task;
use picobell::{cc1101, leds, pio_honeywell, setup};

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
async fn rf_task(r: RfRessources, pio: PIO0, spi: SPI0) {
    let rx_trigger_sender = RX_TRIGGER_WATCH.sender();

    let (mut radio, mut honeywell) = setup::setup_radio(
        config::FREQENCY,
        config::FREQENCY_OFFSET,
        spi,
        r.clk,
        r.mosi,
        r.miso,
        r.csn,
        pio,
        IrqsPio0,
        r.gdo2,
        r.gdo0,
    );

    info!("RF task started");

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
    let mut rx_receiver = unwrap!(RX_TRIGGER_WATCH.receiver());

    let mut leds = setup::setup_leds::<'_, _, { config::LED_COUNT }>(
        config::LED_CONFIG,
        pio,
        IrqsPio1,
        dma,
        r.pin,
    );

    info!("Leds task started");

    let mut mode = leds::Mode::Idle;

    loop {
        match select(rx_receiver.changed(), leds.show(mode)).await {
            Either::First(frame) => mode = leds::Mode::Frame(frame),
            Either::Second(_) => defmt::unreachable!("leds.show() future should never resolve!"),
        }
    }
}
