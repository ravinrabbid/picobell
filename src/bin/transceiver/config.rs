use assign_resources::assign_resources;
use embassy_rp::gpio::Level;
use embassy_rp::peripherals;
use embassy_time::Duration;
use smart_leds::colors;

use crate::leds;

pub const TRIGGER_COOLDOWN: Duration = Duration::from_secs(5);

pub const BUTTON_DEBOUNCE_DELAY: Duration = Duration::from_millis(20);
pub const BUTTON_ACTIVE_LEVEL: Level = Level::Low;

pub const BELL_TRIGGER_DURATION: Duration = Duration::from_millis(100);
pub const BELL_ACTIVE_LEVEL: Level = Level::High;

pub const LED_COUNT: usize = 8;
pub const LED_CONFIG: leds::Config = leds::Config {
    countdown_duration: Duration::from_secs(30),
    color_alert_normal: colors::BLUE,
    color_alert_high1: colors::YELLOW,
    color_alert_high2: colors::ORANGE,
    color_alert_full: colors::WHITE,
    color_low_battery: colors::RED,
};

assign_resources! {
    button: ButtonResources {
        pin: PIN_17,
    },
    bell: BellResources {
        pin: PIN_27,
    },
    leds: LedsResources {
        pio: PIO1,
        dma: DMA_CH1,
        pin: PIN_26,
    },
}
