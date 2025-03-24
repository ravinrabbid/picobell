use arbitrary_int::u20;
use assign_resources::assign_resources;
use embassy_rp::gpio::Level;
use embassy_rp::peripherals;
use embassy_time::Duration;
use picobell::{button, leds};
use smart_leds::colors;

/// Device ID to send packets with
pub const SEND_ID: u20 = u20::new(0x0affe);

/// Frequency offset to compensate inaccuracies of the cc1101's crystal
pub const FREQENCY_OFFSET: i64 = 89_000; // +/- 202_000Hz

/// Delay after which button can be pressed or packet can be received again
pub const TRIGGER_COOLDOWN: Duration = Duration::from_secs(2);

/// Configuration for button behaviour
pub const BUTTON_CONFIG: button::Config = button::Config {
    debounce: Duration::from_millis(20),
    active: Level::Low,
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
