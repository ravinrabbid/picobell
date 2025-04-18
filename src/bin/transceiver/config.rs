use arbitrary_int::u20;
use assign_resources::assign_resources;
use embassy_rp::gpio::Level;
use embassy_rp::peripherals;
use embassy_time::Duration;
use picobell::{bell, button, leds, pio_honeywell::AlertType};
use smart_leds::colors;

/// Device ID to send packets with
///
/// Use 0x0 for broadcast
pub const SEND_ID: u20 = u20::new(0x0affe);

/// Alert Type to send
pub const SEND_ALERT: AlertType = AlertType::Normal;

/// Device ID to receive packets from
///
/// Use 0x0 to accept all IDs
pub const RECEIVE_ID: u20 = u20::new(0x0affe);

/// Frequency offset to compensate inaccuracies of the cc1101's crystal
///
/// Range is +/- 202_000Hz
pub const FREQENCY_OFFSET: i64 = 89_000;

/// Frequency to send/receive on
///
/// Must match the attached CC1101 module
pub const FREQENCY: u64 = 868_300_000;

/// Delay after which button can be pressed or packet can be received again
pub const TRIGGER_COOLDOWN: Duration = Duration::from_secs(2);

/// Configuration for button behaviour
pub const BUTTON_CONFIG: button::Config = button::Config {
    debounce: Duration::from_millis(20),
    active: Level::Low,
};

/// Configuration for bell behaviour
pub const BELL_CONFIG: bell::Config = bell::Config {
    delay: Duration::from_millis(100),
    active: Level::High,
};

/// Number of WS2812 leds attached
pub const LED_COUNT: usize = 12;
/// Configuration for led behaviour
pub const LED_CONFIG: leds::Config = leds::Config {
    flash_count: 7,
    countdown_duration: Duration::from_secs(30),
    color_button_press: colors::WHITE,
    color_alert_normal: colors::BLUE,
    color_alert_high1: colors::YELLOW,
    color_alert_high2: colors::ORANGE,
    color_alert_full: colors::WHITE,
    color_low_battery: colors::RED,
};

// Pins to be used by the external devices
assign_resources! {
    button: ButtonResources {
        pin: PIN_27,
    },
    bell: BellResources {
        pin: PIN_0,
    },
    leds: LedsResources {
        pin: PIN_26,
    },
    rf: RfRessources {
        mosi: PIN_3,
        miso: PIN_4,
        clk: PIN_2,
        csn: PIN_1,
        gdo0: PIN_6,
        gdo2: PIN_7,
    }
}
