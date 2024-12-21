#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::gpio::AnyPin;
use embassy_rp::peripherals::{PIO0, PIO1};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_rp::spi::{Instance, Mode, Spi};
use embassy_rp::{bind_interrupts, clocks};
use embassy_rp::{gpio, spi};
use embassy_time::{Duration, Timer};
use gpio::{Level, Output};
use pio_honeywell::PioHoneywellRx;
use {defmt_rtt as _, panic_probe as _};

mod pio_honeywell;

bind_interrupts!(struct Irqs0 {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

bind_interrupts!(struct Irqs1 {
    PIO1_IRQ_0 => InterruptHandler<PIO1>;
});

struct Rfm69<'a, SPI: Instance, M: Mode> {
    spi: Spi<'a, SPI, M>,
    cs_pin: Output<'a>,
}

impl<'a, SPI: Instance, M: Mode> Rfm69<'a, SPI, M> {
    pub fn write_register(&mut self, address: u8, value: u8) {
        self.cs_pin.set_low();

        let mut buf = [address, value];
        self.spi.blocking_transfer_in_place(&mut buf).unwrap();

        self.cs_pin.set_high();
    }

    pub fn read_register(&mut self, address: u8) -> u8 {
        self.cs_pin.set_low();

        let mut buf = [address, 0x00];
        self.spi.blocking_transfer_in_place(&mut buf).unwrap();

        self.cs_pin.set_high();

        buf[1]
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("Start!");

    // Clk gen
    let gpout = clocks::Gpout::new(p.PIN_21);
    gpout.set_src(clocks::GpoutSrc::Sys);
    gpout.set_div(1_000, 0);
    gpout.enable();
    info!(
        "Pin 21 is now outputting CLK_SYS/1000, should be toggling at {}Hz",
        gpout.get_freq()
    );

    let miso = p.PIN_4;
    let mosi = p.PIN_3;
    let clk = p.PIN_2;
    let cs = p.PIN_5;

    // create SPI
    let mut config = spi::Config::default();
    config.frequency = 10_000_000; // Up to 10mhz
    let spi = Spi::new_blocking(p.SPI0, clk, mosi, miso, config);

    // Configure CS
    let cs_pin = Output::new(cs, Level::Low);

    let mut rfm = Rfm69 { spi, cs_pin };

    info!("Version: {=u8:x}", rfm.read_register(0x10));

    let Pio {
        common: mut common0,
        sm0: sm0_0,
        ..
    } = Pio::new(p.PIO0, Irqs0);
    let prg0 = pio_honeywell::PioHoneywellTxProgram::new(&mut common0);
    let mut honeywelltx =
        pio_honeywell::PioHoneywellTx::new(&mut common0, sm0_0, p.PIN_16, p.PIN_17, &prg0);

    let Pio {
        common: mut common1,
        sm0: sm0_1,
        ..
    } = Pio::new(p.PIO1, Irqs1);
    let prg1 = pio_honeywell::PioHoneywellRxProgram::new(&mut common1);
    let honeywellrx =
        pio_honeywell::PioHoneywellRx::new(&mut common1, sm0_1, p.PIN_14, p.PIN_15, &prg1);

    unwrap!(spawner.spawn(loopback(honeywellrx)));

    let frame = [0xDEAD, 0xBEEF, 0xAFFE];

    loop {
        info!("Writing frame {:x} ...", frame);
        honeywelltx.write_frame(&frame).await;

        Timer::after(Duration::from_secs(5)).await;
    }
}

#[embassy_executor::task(pool_size = 1)]
async fn loopback(mut rx: PioHoneywellRx<'static, PIO1, 0>) {
    loop {
        let loopback_frame = rx.read_frame().await;

        info!("Read frame {:x}", loopback_frame);
    }
}
