use embassy_rp::gpio::{Input, Level, Pin, Pull};
use embassy_rp::Peripheral;
use embassy_time::{Duration, Timer};

pub struct Config {
    pub debounce: Duration,
    pub active: Level,
}

pub struct Button<'d> {
    input: Input<'d>,
    config: Config,
}

impl<'d> Button<'d> {
    pub fn new(pin: impl Peripheral<P = impl Pin> + 'd, config: Config) -> Self {
        let pull = match config.active {
            Level::High => Pull::Down,
            Level::Low => Pull::Up,
        };

        let input = Input::new(pin, pull);

        Self { input, config }
    }

    pub async fn wait_for_press(&mut self) {
        loop {
            match self.config.active {
                Level::High => self.input.wait_for_rising_edge().await,
                Level::Low => self.input.wait_for_falling_edge().await,
            }

            Timer::after(self.config.debounce).await;

            if self.input.get_level() == self.config.active {
                break;
            }
        }
    }
}
