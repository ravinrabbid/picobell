use assign_resources::assign_resources;
use embassy_rp::gpio::Level;
use embassy_rp::peripherals;
use embassy_time::Duration;
use picobell::{bell, button, leds};
use smart_leds::colors;

pub const TRIGGER_COOLDOWN: Duration = Duration::from_secs(5);

pub const BUTTON_CONFIG: button::Config = button::Config {
    debounce: Duration::from_millis(20),
    active: Level::Low,
};

pub const BELL_CONFIG: bell::Config = bell::Config {
    delay: Duration::from_millis(100),
    active: Level::High,
};

pub const LED_COUNT: usize = 12;
pub const LED_CONFIG: leds::Config = leds::Config {
    blink_count: 7,
    countdown_duration: Duration::from_secs(30),
    color_alert_normal: colors::BLUE,
    color_alert_high1: colors::YELLOW,
    color_alert_high2: colors::ORANGE,
    color_alert_full: colors::WHITE,
    color_low_battery: colors::RED,
};

assign_resources! {
    button: ButtonResources {
        pin: PIN_27,
    },
    bell: BellResources {
        pin: PIN_0,
    },
    leds: LedsResources {
        pio: PIO1,
        dma: DMA_CH1,
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
