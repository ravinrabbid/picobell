use embassy_rp::gpio::{Level, Output, Pin};
use embassy_rp::Peripheral;
use embassy_time::{Duration, Timer};

pub struct Bell<'d> {
    output: Output<'d>,
    delay: Duration,
    active: Level,
}

impl<'d> Bell<'d> {
    pub fn new(pin: impl Peripheral<P = impl Pin> + 'd, delay: Duration, active: Level) -> Self {
        let output = match active {
            Level::High => Output::new(pin, Level::Low),
            Level::Low => Output::new(pin, Level::High),
        };

        Self {
            output,
            delay,
            active,
        }
    }

    pub async fn trigger(&mut self) {
        self.output.set_level(self.active);

        Timer::after(self.delay).await;

        match self.active {
            Level::High => self.output.set_low(),
            Level::Low => self.output.set_high(),
        };
    }
}
