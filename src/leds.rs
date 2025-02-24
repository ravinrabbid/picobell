use embassy_rp::dma::Channel;
use embassy_rp::pio::{Common, Instance, PioPin, StateMachine};
use embassy_rp::pio_programs::ws2812::{PioWs2812, PioWs2812Program};
use embassy_rp::Peripheral;
use embassy_time::{Duration, Timer};
use smart_leds::{RGB, RGB8};

use crate::pio_honeywell::{AlertType, Frame};

pub struct Config {
    pub blink_count: u8,

    pub countdown_duration: Duration,

    pub color_button_press: RGB8,
    pub color_alert_normal: RGB8,
    pub color_alert_high1: RGB8,
    pub color_alert_high2: RGB8,
    pub color_alert_full: RGB8,
    pub color_low_battery: RGB8,
}

pub enum Mode {
    Frame(Frame),
    ButtonPress,
}

pub struct Leds<'d, PIO: Instance, const S: usize, const N: usize> {
    ws2812: PioWs2812<'d, PIO, S, N>,
    config: Config,
    current_mode: Option<Mode>,
}

impl<'d, PIO: Instance, const S: usize, const N: usize> Leds<'d, PIO, S, N> {
    pub fn new(
        config: Config,
        mut common: &mut Common<'d, PIO>,
        sm: StateMachine<'d, PIO, S>,
        dma: impl Peripheral<P = impl Channel> + 'd,
        pin: impl PioPin,
    ) -> Self {
        let program = PioWs2812Program::new(&mut common);
        let ws2812 = PioWs2812::new(&mut common, sm, dma, pin, &program);

        Self {
            ws2812,
            config,
            current_mode: None,
        }
    }

    pub fn update(&mut self, mode: Mode) {
        self.current_mode = Some(mode);
    }

    pub async fn show(&mut self) {
        let (color_on, color_off) = match self.current_mode {
            Some(Mode::Frame(Frame {
                id: _,
                device_type: _,
                alert,
                secret_knock: _,
                relay: _,
                low_battery,
            })) => (
                match alert {
                    AlertType::Normal => self.config.color_alert_normal,
                    AlertType::High1 => self.config.color_alert_high1,
                    AlertType::High2 => self.config.color_alert_high2,
                    AlertType::Full => self.config.color_alert_full,
                },
                if low_battery {
                    self.config.color_low_battery
                } else {
                    RGB8::default()
                },
            ),
            Some(Mode::ButtonPress) => {
                self.ws2812
                    .write(&[self.config.color_button_press; N])
                    .await;
                Timer::after(Duration::from_millis(500)).await;
                self.ws2812.write(&[RGB::default(); N]).await;
                return;
            }
            None => {
                self.ws2812.write(&[RGB8::default(); N]).await;
                return;
            }
        };

        for _ in 0..self.config.blink_count {
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

        self.ws2812.write(&[RGB8::default(); N]).await;
    }
}
