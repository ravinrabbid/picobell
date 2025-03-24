#![no_std]
#![no_main]

use cc1101::Cc1101;
use config::{AssignedResources, ButtonResources, LedsResources, RfRessources};
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
use leds::{Leds, Mode as LedMode};
use pio_honeywell::{Frame, PioHoneywell};
use {defmt_rtt as _, panic_probe as _};

use picobell::tasks::run_button_task;
use picobell::{cc1101, common, leds, pio_honeywell};

mod config;

static BUTTON_TRIGGER_WATCH: Watch<CriticalSectionRawMutex, Frame, 2> = Watch::new();

bind_interrupts!(struct IrqsPio0 {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
});

bind_interrupts!(struct IrqsPio1 {
    PIO1_IRQ_0 => pio::InterruptHandler<PIO1>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Picobell transmitter starting ...");

    let p = embassy_rp::init(Default::default());
    let r = split_resources!(p);

    unwrap!(spawner.spawn(rf_task(r.rf, p.PIO0, p.SPI0)));
    unwrap!(spawner.spawn(button_task(r.button)));
    unwrap!(spawner.spawn(leds_task(r.leds, p.PIO1, p.DMA_CH1)));

    info!("Picobell transmitter ready");
}

#[embassy_executor::task]
async fn rf_task(r: RfRessources, mut pio: PIO0, spi: SPI0) {
    let Pio { common, sm0, .. } = Pio::new(&mut pio, IrqsPio0);

    let mut honeywell = PioHoneywell::new(common, sm0, r.gdo0, r.gdo2);

    let mut button_receiver = unwrap!(BUTTON_TRIGGER_WATCH.receiver());

    let mut spi_config = spi::Config::default();
    spi_config.frequency = 8_000_000;

    let spi = Spi::new_blocking(spi, r.clk, r.mosi, r.miso, spi_config);
    let mut radio = Cc1101::new(spi, r.csn);

    common::setup_radio(&mut radio, config::FREQENCY_OFFSET);

    info!("RF task started");

    loop {
        let frame = button_receiver.changed().await;

        if let Err(e) = radio.set_mode(cc1101::Mode::Tx) {
            error!("Failed changing to Tx mode: {}", e);
        } else {
            info!("Sending frame {}", frame);

            // Send frame a few times for redundancy (Original sender does 50)
            honeywell.write_frame(&frame, 20).await;
        }
    }
}

#[embassy_executor::task]
async fn button_task(r: ButtonResources) {
    let frame_to_send = Frame {
        id: config::SEND_ID,
        alert: pio_honeywell::AlertType::Normal,
        low_battery: false,
        ..Default::default()
    };

    run_button_task(
        r.pin,
        config::BUTTON_CONFIG,
        frame_to_send,
        config::TRIGGER_COOLDOWN,
        BUTTON_TRIGGER_WATCH.sender(),
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

    let mut button_receiver = unwrap!(BUTTON_TRIGGER_WATCH.receiver());

    info!("Leds task started");

    let mut mode = LedMode::Idle;

    loop {
        match select(button_receiver.changed(), leds.show(mode)).await {
            Either::First(_) => mode = LedMode::ButtonPress,
            Either::Second(_) => defmt::unreachable!("leds.show() future should never resolve!"),
        }
    }
}
