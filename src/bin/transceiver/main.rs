#![no_std]
#![no_main]

use bell::Bell;
use button::Button;
use config::{AssignedResources, BellResources, ButtonResources, LedsResources};
use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{PIO0, PIO1};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::Timer;
use leds::Leds;
use pio_honeywell::Frame;
use {defmt_rtt as _, panic_probe as _};

use picobell::{bell, button, leds, pio_honeywell};

mod config;

static TRIGGER_WATCH: Watch<CriticalSectionRawMutex, Frame, 2> = Watch::new();

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
    unwrap!(spawner.spawn(leds_task(r.leds)));

    info!("Picobell ready");
}

#[embassy_executor::task]
async fn button_task(r: ButtonResources) {
    let mut button = Button::new(
        r.pin,
        config::BUTTON_DEBOUNCE_DELAY,
        config::BUTTON_ACTIVE_LEVEL,
    );
    let sender = TRIGGER_WATCH.sender();

    info!("Button task started");

    let frame = Frame {
        alert: pio_honeywell::AlertType::High1,
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
    let mut bell = Bell::new(
        r.pin,
        config::BELL_TRIGGER_DURATION,
        config::BELL_ACTIVE_LEVEL,
    );
    let mut receiver = unwrap!(TRIGGER_WATCH.receiver());

    info!("Bell task started");

    loop {
        receiver.changed().await;
        bell.trigger().await;
        info!("Bell triggered");
    }
}

#[embassy_executor::task]
async fn leds_task(r: LedsResources) {
    let Pio {
        mut common, sm0, ..
    } = Pio::new(r.pio, Irqs1);
    let mut leds: Leds<'_, PIO1, 0, { config::LED_COUNT }> =
        Leds::new(config::LED_CONFIG, &mut common, sm0, r.dma, r.pin);

    let mut receiver = unwrap!(TRIGGER_WATCH.receiver());

    info!("Leds task started");

    loop {
        match select(receiver.changed(), leds.show()).await {
            Either::First(frame) => leds.update(frame),
            Either::Second(_) => leds.update(receiver.changed().await),
        }
    }
}
