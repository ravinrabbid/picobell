use embassy_rp::gpio::{Drive, Level, Output, Pin};
use embassy_rp::Peripheral;
use embassy_time::{Duration, Timer};

/// Configuration for the bell output
pub struct Config {
    /// How long the output will be set to active when triggered
    pub delay: Duration,
    /// Logic level to set to output to when triggered
    pub active: Level,
}

pub struct Bell<'d> {
    output: Output<'d>,
    config: Config,
}

impl<'d> Bell<'d> {
    pub fn new(pin: impl Peripheral<P = impl Pin> + 'd, config: Config) -> Self {
        let mut output = match config.active {
            Level::High => Output::new(pin, Level::Low),
            Level::Low => Output::new(pin, Level::High),
        };

        output.set_drive_strength(Drive::_12mA);

        Self { output, config }
    }

    pub async fn trigger(&mut self) {
        self.output.set_level(self.config.active);

        Timer::after(self.config.delay).await;

        match self.config.active {
            Level::High => self.output.set_low(),
            Level::Low => self.output.set_high(),
        };
    }
}
