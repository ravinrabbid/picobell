use defmt::*;
use embassy_rp::gpio::Pin;
use embassy_rp::Peripheral;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::{Receiver, Sender};
use embassy_time::{Duration, Timer};

use crate::bell;
use crate::bell::Bell;
use crate::button;
use crate::button::Button;
use crate::pio_honeywell::Frame;

pub async fn run_button_task(
    pin: impl Peripheral<P = impl Pin>,
    config: button::Config,
    frame_to_send: Frame,
    cooldown: Duration,
    sender: Sender<'_, CriticalSectionRawMutex, Frame, 2>,
) {
    let mut button = Button::new(pin, config);

    info!("Button task started");

    loop {
        button.wait_for_press().await;
        info!("Button pressed");

        sender.send(frame_to_send);

        Timer::after(cooldown).await;
    }
}

pub async fn run_bell_task(
    pin: impl Peripheral<P = impl Pin>,
    config: bell::Config,
    mut receiver: Receiver<'_, CriticalSectionRawMutex, Frame, 2>,
) {
    let mut bell = Bell::new(pin, config);

    info!("Bell task started");

    loop {
        receiver.changed().await;

        bell.trigger().await;
        info!("Bell triggered");
    }
}
