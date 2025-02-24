#![no_std]
#![no_main]

use arbitrary_int::u20;
use bell::Bell;
use button::Button;
use config::{AssignedResources, BellResources, ButtonResources, LedsResources, RfRessources};
use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::select::{select, select3, Either, Either3};
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{DMA_CH1, PIO0, PIO1};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::Timer;
use leds::Leds;
use picobell::leds::Mode as LedMode;
use picobell::pio_honeywell::PioHoneywell;
use pio_honeywell::Frame;
use {defmt_rtt as _, panic_probe as _};

use picobell::{bell, button, leds, pio_honeywell};

mod config;

static BUTTON_TRIGGER_WATCH: Watch<CriticalSectionRawMutex, Frame, 2> = Watch::new();
static RX_TRIGGER_WATCH: Watch<CriticalSectionRawMutex, Frame, 2> = Watch::new();

bind_interrupts!(struct Irqs0 {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

bind_interrupts!(struct Irqs1 {
    PIO1_IRQ_0 => InterruptHandler<PIO1>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Picobell starting ...");

    let p = embassy_rp::init(Default::default());
    let r = split_resources!(p);

    unwrap!(spawner.spawn(button_task(r.button)));
    unwrap!(spawner.spawn(bell_task(r.bell)));
    unwrap!(spawner.spawn(leds_task(r.leds, p.PIO1, p.DMA_CH1)));
    unwrap!(spawner.spawn(rf_task(r.rf, p.PIO0)));

    info!("Picobell ready");
}

#[embassy_executor::task]
async fn rf_task(r: RfRessources, mut pio: PIO0) {
    let Pio {
        mut common, sm0, ..
    } = Pio::new(&mut pio, Irqs0);
    let mut honeywell = PioHoneywell::new(&mut common, sm0, r.gdo0, r.gdo2);

    let tx_watch = RX_TRIGGER_WATCH.sender();
    let mut button = unwrap!(BUTTON_TRIGGER_WATCH.receiver());

    info!("RF task started");

    loop {
        // TODO time to restart phy from time to time
        match select(button.changed(), honeywell.read_frame()).await {
            Either::First(frame) => {
                honeywell.write_frame(&frame).await;
            }
            Either::Second(frame) => {
                if frame.id == config::RECEIVE_ID
                    || frame.id == u20::new(0)
                    || config::RECEIVE_ID == u20::new(0)
                {
                    tx_watch.send(frame);
                }
            }
        }
    }
}

#[embassy_executor::task]
async fn button_task(r: ButtonResources) {
    let mut button = Button::new(r.pin, config::BUTTON_CONFIG);
    let sender = BUTTON_TRIGGER_WATCH.sender();

    info!("Button task started");

    // TODO config
    let frame = Frame {
        id: config::SEND_ID,
        alert: pio_honeywell::AlertType::Normal,
        low_battery: false,
        ..Default::default()
    };

    loop {
        button.wait_for_press().await;

        sender.send(frame);

        info!("Button pressed");

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
    } = Pio::new(pio, Irqs1);
    let mut leds: Leds<'_, PIO1, 0, { config::LED_COUNT }> =
        Leds::new(config::LED_CONFIG, &mut common, sm0, dma, r.pin);

    let mut rx_receiver = unwrap!(RX_TRIGGER_WATCH.receiver());
    let mut button_receiver = unwrap!(BUTTON_TRIGGER_WATCH.receiver());

    info!("Leds task started");

    loop {
        match select3(
            rx_receiver.changed(),
            button_receiver.changed(),
            leds.show(),
        )
        .await
        {
            Either3::First(frame) => leds.update(LedMode::Frame(frame)),
            Either3::Second(_) => leds.update(LedMode::ButtonPress),
            Either3::Third(_) => leds.update(
                match select(rx_receiver.changed(), button_receiver.changed()).await {
                    Either::First(frame) => LedMode::Frame(frame),
                    Either::Second(_) => LedMode::ButtonPress,
                },
            ),
        }
    }
}
