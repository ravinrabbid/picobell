#![no_std]
#![no_main]

use arbitrary_int::u20;
use config::{AssignedResources, BellResources, ButtonResources, LedsResources, RfRessources};
use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::select::{select3, Either3};
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{DMA_CH1, PIO0, PIO1, SPI0};
use embassy_rp::pio;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::Timer;
use pio_honeywell::Frame;
use {defmt_rtt as _, panic_probe as _};

use picobell::tasks::{run_bell_task, run_button_task};
use picobell::{cc1101, leds, pio_honeywell, setup};

mod config;

static BUTTON_TRIGGER_WATCH: Watch<CriticalSectionRawMutex, Frame, 2> = Watch::new();
static RX_TRIGGER_WATCH: Watch<CriticalSectionRawMutex, Frame, 2> = Watch::new();

bind_interrupts!(struct IrqsPio0 {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
});

bind_interrupts!(struct IrqsPio1 {
    PIO1_IRQ_0 => pio::InterruptHandler<PIO1>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Picobell transceiver starting ...");

    let p = embassy_rp::init(Default::default());
    let r = split_resources!(p);

    unwrap!(spawner.spawn(rf_task(r.rf, p.PIO0, p.SPI0)));
    unwrap!(spawner.spawn(button_task(r.button)));
    unwrap!(spawner.spawn(bell_task(r.bell)));
    unwrap!(spawner.spawn(leds_task(r.leds, p.PIO1, p.DMA_CH1)));

    info!("Picobell transceiver ready");
}

#[embassy_executor::task]
async fn rf_task(r: RfRessources, pio: PIO0, spi: SPI0) {
    let rx_trigger_sender = RX_TRIGGER_WATCH.sender();
    let mut button_receiver = unwrap!(BUTTON_TRIGGER_WATCH.receiver());

    let (mut radio, mut honeywell) = setup::setup_radio(
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

        match select3(
            button_receiver.changed(),
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
                        rx_trigger_sender.send(frame);

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
    let frame_to_send = Frame {
        id: config::SEND_ID,
        alert: config::SEND_ALERT,
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
    let mut button_receiver = unwrap!(BUTTON_TRIGGER_WATCH.receiver());

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
        match select3(
            rx_receiver.changed(),
            button_receiver.changed(),
            leds.show(mode),
        )
        .await
        {
            Either3::First(frame) => mode = leds::Mode::Frame(frame),
            Either3::Second(_) => mode = leds::Mode::ButtonPress,
            Either3::Third(_) => defmt::unreachable!("leds.show() future should never resolve!"),
        }
    }
}
