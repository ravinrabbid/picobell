use embassy_rp::gpio::{Input, Level, Pin, Pull};
use embassy_rp::Peripheral;
use embassy_time::{Duration, Timer};

pub struct Button<'d> {
    input: Input<'d>,
    debounce: Duration,
    active: Level,
}

impl<'d> Button<'d> {
    pub fn new(pin: impl Peripheral<P = impl Pin> + 'd, debounce: Duration, active: Level) -> Self {
        let pull = match active {
            Level::High => Pull::Down,
            Level::Low => Pull::Up,
        };

        let input = Input::new(pin, pull);

        Self {
            input,
            debounce,
            active,
        }
    }

    pub async fn wait_for_press(&mut self) {
        loop {
            match self.active {
                Level::High => self.input.wait_for_rising_edge().await,
                Level::Low => self.input.wait_for_falling_edge().await,
            }

            Timer::after(self.debounce).await;

            if self.input.get_level() == self.active {
                break;
            }
        }
    }
}
