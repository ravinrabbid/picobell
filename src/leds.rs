//! Module to control LED animations
//!
//! Targets WS2812 type leds using PIO for control.

use core::future;

use embassy_rp::dma::Channel;
use embassy_rp::pio::{Common, Instance, PioPin, StateMachine};
use embassy_rp::pio_programs::ws2812::{PioWs2812, PioWs2812Program};
use embassy_rp::Peripheral;
use embassy_time::{Duration, Timer};
use smart_leds::{RGB, RGB8};

use crate::pio_honeywell::{AlertType, Frame};

/// Configuration for the LEDs
pub struct Config {
    /// How often the LEDs will flash when a frame is received
    pub flash_count: u8,

    /// How long the countdown animation for a received frame will take
    pub countdown_duration: Duration,

    /// Flash color when the button is pressed
    pub color_button_press: RGB8,
    /// Flash color for normal alert level
    pub color_alert_normal: RGB8,
    /// Flash color for high1 alert level
    pub color_alert_high1: RGB8,
    /// Flash color for high2 alert level
    pub color_alert_high2: RGB8,
    /// Flash color for full alert level
    pub color_alert_full: RGB8,
    /// Secondary color if frame has low battery set
    pub color_low_battery: RGB8,
}

pub enum Mode {
    Idle,
    Frame(Frame),
    ButtonPress,
}

pub struct Leds<'d, PIO: Instance, const S: usize, const N: usize> {
    ws2812: PioWs2812<'d, PIO, S, N>,
    config: Config,
}

impl<'d, PIO: Instance, const S: usize, const N: usize> Leds<'d, PIO, S, N> {
    pub fn new(
        config: Config,
        common: &mut Common<'d, PIO>,
        sm: StateMachine<'d, PIO, S>,
        dma: impl Peripheral<P = impl Channel> + 'd,
        pin: impl PioPin,
    ) -> Self {
        let program = PioWs2812Program::new(common);
        let ws2812 = PioWs2812::new(common, sm, dma, pin, &program);

        Self { ws2812, config }
    }

    /// Show the animation for the given mode.
    ///
    /// This method will never return.
    pub async fn show(&mut self, mode: Mode) {
        match mode {
            Mode::Frame(Frame {
                id: _,
                device_type: _,
                alert,
                secret_knock: _,
                relay: _,
                low_battery,
            }) => {
                let color_on = match alert {
                    AlertType::Normal => self.config.color_alert_normal,
                    AlertType::High1 => self.config.color_alert_high1,
                    AlertType::High2 => self.config.color_alert_high2,
                    AlertType::Full => self.config.color_alert_full,
                };

                let color_off = if low_battery {
                    self.config.color_low_battery
                } else {
                    RGB8::default()
                };

                for _ in 0..self.config.flash_count {
                    self.ws2812.write(&[color_on; N]).await;
                    Timer::after(Duration::from_millis(100)).await;
                    self.ws2812.write(&[color_off; N]).await;
                    Timer::after(Duration::from_millis(500)).await;
                }

                let countdown_step = self.config.countdown_duration / N as u32;
                let mut colors = [color_on; N];

                self.ws2812.write(&colors).await;

                for step in (0..N).rev() {
                    Timer::after(countdown_step).await;
                    colors[step] = color_off;
                    self.ws2812.write(&colors).await;
                }
            }
            Mode::ButtonPress => {
                self.ws2812
                    .write(&[self.config.color_button_press; N])
                    .await;
                Timer::after(Duration::from_millis(500)).await;
            }
            Mode::Idle => {}
        };
        self.ws2812.write(&[RGB::default(); N]).await;

        let () = future::pending().await;
    }
}
