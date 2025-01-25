use embassy_rp::dma::Channel;
use embassy_rp::pio::{Common, Instance, PioPin, StateMachine};
use embassy_rp::pio_programs::ws2812::{PioWs2812, PioWs2812Program};
use embassy_rp::Peripheral;
use embassy_time::{Duration, Ticker};
use smart_leds::RGB8;

use crate::pio_honeywell::{AlertType, Frame};

const TICK_LENGTH: Duration = Duration::from_millis(10);

const BLINK_COUNT: u8 = 5;
const BLINK_ON_TICKS: u16 = 10;
const BLINK_OFF_TICKS: u16 = 50;

// const COUNTDOWN_TICKS: u16 = (LED_COUNTDOWN / LED_COUNT as u32).as_ticks() as u16;

pub struct Config {
    pub countdown_duration: Duration,

    pub color_alert_normal: RGB8,
    pub color_alert_high1: RGB8,
    pub color_alert_high2: RGB8,
    pub color_alert_full: RGB8,
    pub color_low_battery: RGB8,
}

enum Status {
    Idle,
    Blink,
    Countdown,
}

struct State {
    // Current overall status
    status: Status,
    // Remaining updates until the next status
    remaining_updates: u8,
    // Remaining ticks until the next update
    remaining_ticks: u16,
    // Color of active LEDs
    color_on: RGB8,
    // Color of inactive LEDs
    color_off: RGB8,
}

impl Default for State {
    fn default() -> Self {
        Self {
            status: Status::Idle,
            remaining_updates: 0,
            remaining_ticks: 0,
            color_on: RGB8::default(),
            color_off: RGB8::default(),
        }
    }
}

pub struct Leds<'d, PIO: Instance, const S: usize, const N: usize> {
    ws2812: PioWs2812<'d, PIO, S, N>,
    config: Config,
    state: State,
    ticker: Ticker,
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
            state: State::default(),
            ticker: Ticker::every(TICK_LENGTH),
        }
    }

    pub fn update(&mut self, frame: Frame) {
        self.state = State {
            status: Status::Blink,
            remaining_updates: BLINK_COUNT,
            remaining_ticks: BLINK_ON_TICKS,
            color_on: match frame.alert {
                AlertType::Normal => self.config.color_alert_normal,
                AlertType::High1 => self.config.color_alert_high1,
                AlertType::High2 => self.config.color_alert_high2,
                AlertType::Full => self.config.color_alert_full,
            },
            color_off: if frame.low_battery {
                self.config.color_low_battery
            } else {
                RGB8::default()
            },
        };
        self.ticker.reset();
    }

    pub async fn tick(&mut self) {
        self.ticker.next().await;

        self.ws2812.write(&[self.state.color_on; N]).await;

        // match self.state.status {
        //     Status::Idle => (),
        //     Status::Blink => {
        //         self.state.remaining_ticks -= 0;
        //         if self.state.remaining_ticks == 0 {

        //         }
        //     }
        //     Status::Countdown => self.state = State::Countdown(remain - 1),
        // }
    }
}
