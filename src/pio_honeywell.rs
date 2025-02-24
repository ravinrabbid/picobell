use arbitrary_int::u20;
use defmt::{debug, Format};
use fixed::traits::ToFixed;
use thiserror::Error;

use embassy_rp::gpio::Level;
use embassy_rp::pio::{
    Common, Config, Direction, FifoJoin, Instance, LoadedProgram, Pin, PioPin, ShiftConfig,
    ShiftDirection, StateMachine,
};

// From https://github.com/klohner/honeywell-wireless-doorbell
//
// # Frame bits used in Honeywell RCWL300A, RCWL330A, Series 3, 5, 9 and all Decor Series Wireless Chimes
// # 0000 0000 1111 1111 2222 2222 3333 3333 4444 4444 5555 5555
// # 7654 3210 7654 3210 7654 3210 7654 3210 7654 3210 7654 3210
// # XXXX XXXX XXXX XXXX XXXX XXXX XXXX XXXX XXXX XX.. XXX. .... KEY DATA (any change and receiver doesn't seem to recognize signal)
// # XXXX XXXX XXXX XXXX XXXX .... .... .... .... .... .... .... KEY ID (different for each transmitter)
// # .... .... .... .... .... 0000 00.. 0000 0000 00.. 000. .... KEY UNKNOWN 0 (always 0 in devices I've tested)
// # .... .... .... .... .... .... ..XX .... .... .... .... .... DEVICE TYPE (10 = doorbell, 01 = PIR Motion sensor)
// # .... .... .... .... .... .... .... .... .... ..XX ...X XXX. FLAG DATA (may be modified for possible effects on receiver)
// # .... .... .... .... .... .... .... .... .... ..XX .... .... ALERT (00 = normal, 01 or 10 = right-left halo light pattern, 11 = full volume alarm)
// # .... .... .... .... .... .... .... .... .... .... ...X .... SECRET KNOCK (0 = default, 1 if doorbell is pressed 3x rapidly)
// # .... .... .... .... .... .... .... .... .... .... .... X... RELAY (1 if signal is a retransmission of a received transmission, only some models)
// # .... .... .... .... .... .... .... .... .... .... .... .X.. FLAG UNKNOWN (0 = default, but 1 is accepted and I don't observe any effects)
// # .... .... .... .... .... .... .... .... .... .... .... ..X. LOWBAT (1 if battery is low, receiver gives low battery alert)
// # .... .... .... .... .... .... .... .... .... .... .... ...X PARITY (LSB of count of set bits in previous 47 bits)

#[derive(Error, Format, Debug)]
pub enum Error {
    #[error("Parity check failed.")]
    Parity,
    #[error("Could not convert DeviceType from '{0}'.")]
    DeviceType(u8),
    #[error("Could not convert AlertType from '{0}'.")]
    AlertType(u8),
}

pub trait Deserialize {
    fn deserialize(&self) -> Result<Frame, Error>;
}

#[derive(Format, Debug, Clone, Copy)]
pub enum DeviceType {
    PirMotionSensor = 1,
    Doorbell = 2,
}

impl TryFrom<u8> for DeviceType {
    type Error = Error;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            1 => Ok(DeviceType::PirMotionSensor),
            2 => Ok(DeviceType::Doorbell),
            _ => Err(Error::DeviceType(value)),
        }
    }
}

#[derive(Format, Debug, Clone, Copy)]
pub enum AlertType {
    Normal = 0,
    High1 = 1,
    High2 = 2,
    Full = 3,
}

impl TryFrom<u8> for AlertType {
    type Error = Error;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(AlertType::Normal),
            1 => Ok(AlertType::High1),
            2 => Ok(AlertType::High2),
            3 => Ok(AlertType::Full),
            _ => Err(Error::AlertType(value)),
        }
    }
}

#[derive(Format, Debug, Clone, Copy)]
pub struct Frame {
    pub id: u20,
    pub device_type: DeviceType,
    pub alert: AlertType,
    pub secret_knock: bool,
    pub relay: bool,
    pub low_battery: bool,
}

impl Default for Frame {
    fn default() -> Self {
        Self {
            id: u20::new(0), // Id 0 will trigger all receivers
            device_type: DeviceType::Doorbell,
            alert: AlertType::Normal,
            secret_knock: false,
            relay: false,
            low_battery: false,
        }
    }
}

impl Frame {
    pub fn new(id: u20) -> Self {
        Self {
            id,
            ..Default::default()
        }
    }

    pub fn serialize(self) -> [u8; 6] {
        let mut bytes = [0u8; 6];

        let id_bytes = &(self.id.value() << 4).to_be_bytes()[1..4];

        bytes[..3].copy_from_slice(id_bytes);
        bytes[3] |= (self.device_type as u8) << 4;
        bytes[4] |= self.alert as u8;
        bytes[5] |= (self.secret_knock as u8) << 4;
        bytes[5] |= (self.relay as u8) << 3;
        bytes[5] |= (self.low_battery as u8) << 1;

        let parity = bytes.iter().fold(false, |parity, byte| {
            if byte.count_ones() % 2 != 0 {
                !parity
            } else {
                parity
            }
        });
        bytes[5] |= parity as u8;

        bytes
    }
}

impl Deserialize for [u8; 6] {
    fn deserialize(&self) -> Result<Frame, Error> {
        if self.iter().fold(false, |parity, byte| {
            if byte.count_ones() % 2 != 0 {
                !parity
            } else {
                parity
            }
        }) {
            return Err(Error::Parity);
        };

        Ok(Frame {
            id: u20::extract_u32(u32::from_be_bytes(self[..4].try_into().unwrap()), 12),
            device_type: ((self[3] & 0x30) >> 4).try_into()?,
            alert: (self[4] & 0x03).try_into()?,
            secret_knock: (self[5] & 0x10) != 0,
            relay: (self[5] & 0x08) != 0,
            low_battery: (self[5] & 0x02) != 0,
        })
    }
}

pub struct PioHoneywellTxProgram<'a, PIO: Instance> {
    prg: LoadedProgram<'a, PIO>,
}

impl<'a, PIO: Instance> PioHoneywellTxProgram<'a, PIO> {
    pub fn new(common: &mut Common<'a, PIO>) -> Self {
        let prg = pio_proc::pio_asm!(
            r#"
                .side_set 1 opt            ; DOUT is written via side set

                .wrap_target
                reset:
                    set y, 5               ; 6 bytes per frame
                preamble:                  ; LOW-LOW-LOW
                    set x, 2
                preamble_loop:
                    wait 1 pin 0
                    wait 0 pin 0 side 0
                    jmp x-- preamble_loop
                next_word:
                    pull
                write_bit:                 ; All bits start with HIGH
                    out x, 1
                    wait 1 pin 0
                    wait 0 pin 0 side 1
                    jmp !x tx_0
                tx_1:                      ; '1' is HIGH next
                    wait 1 pin 0
                    wait 0 pin 0 side 1
                    jmp end_bit
                tx_0:                      ; '0' is LOW next
                    wait 1 pin 0
                    wait 0 pin 0 side 0
                end_bit:                   ; All bits end with LOW
                    wait 1 pin 0
                    wait 0 pin 0 side 0
                    jmp !osre write_bit
                    jmp y-- next_word
                postamble:                 ; HIGH-HIGH-HIGH
                    set x, 2
                postamble_loop:
                    wait 1 pin 0
                    wait 0 pin 0 side 1
                    jmp x-- postamble_loop
                .wrap
            "#
        );

        let prg = common.load_program(&prg.program);

        Self { prg }
    }
}

pub struct PioHoneywellRxProgram<'a, PIO: Instance> {
    prg: LoadedProgram<'a, PIO>,
}

impl<'a, PIO: Instance> PioHoneywellRxProgram<'a, PIO> {
    pub fn new(common: &mut Common<'a, PIO>) -> Self {
        let prg = pio_proc::pio_asm!(
            r#"
                .wrap_target
                reset:
                    set y, 2                    ; Preamble is at least 3 time LOW
                wait_preamble:
                    wait 0 pin 1
                    wait 1 pin 1
                    jmp pin reset               ; Reset at premature HIGH
                    jmp y-- wait_preamble
                    pull                        ; Pull bit count-1 from FIFO
                    mov y osr
                wait_first_symbol:
                    wait 0 pin 1
                    wait 1 pin 1
                    jmp pin read_second_symbol  ; First symbol must always be HIGH
                    jmp wait_first_symbol       ; Allow arbitrary number of LOW until before first bit
                read_first_symbol:
                    wait 0 pin 1
                    wait 1 pin 1
                    jmp pin read_second_symbol  ; First symbol must always be HIGH
                    jmp cancel_receive          ; Not a valid bit, cancel
                read_second_symbol:
                    wait 0 pin 1
                    wait 1 pin 1
                    jmp pin read_1              ; On a second HIGH we have a 1, on LOW we have a 0
                read_0:
                    set x 0                     ; Set 0 to be pushed to FIFO
                    jmp read_third_symbol
                read_1:
                    set x 1                     ; Set 1 to be pushed to FIFO
                read_third_symbol:
                    wait 0 pin 1
                    wait 1 pin 1
                    jmp pin cancel_receive      ; Third symbol must always be low, cancel otherwise
                push_bit:
                    in x, 1                     ; We have a valid bit, push to FIFO
                    jmp y-- read_first_symbol   ; Continue with next bit
                    jmp reset                   ; Ignore postamble and wait for next frame
                cancel_receive:
                    set x, 0                    ; Clock out remaining bits as 0
                    in x, 1                     ; to avoid stalling read_frame and
                    jmp y-- cancel_receive      ; the FIFO is empty for the next frame 
                .wrap
            "#
        );

        let prg = common.load_program(&prg.program);

        Self { prg }
    }
}

#[derive(Format, Debug, Clone, Copy, PartialEq)]
pub enum Mode {
    Rx,
    Tx,
}

pub struct PioHoneywell<'a, PIO: Instance, const S: usize> {
    data_pin: Pin<'a, PIO>,
    clk_pin: Pin<'a, PIO>,
    sm: StateMachine<'a, PIO, S>,
    rx_prg: PioHoneywellRxProgram<'a, PIO>,
    tx_prg: PioHoneywellTxProgram<'a, PIO>,
    mode: Option<Mode>,
}

impl<'a, PIO: Instance, const S: usize> PioHoneywell<'a, PIO, S> {
    pub fn new(
        common: &mut Common<'a, PIO>,
        sm: StateMachine<'a, PIO, S>,
        data_pin: impl PioPin,
        clk_pin: impl PioPin,
    ) -> Self {
        let data_pin = common.make_pio_pin(data_pin);
        let clk_pin = common.make_pio_pin(clk_pin);

        Self {
            data_pin,
            clk_pin,
            sm,
            rx_prg: PioHoneywellRxProgram::new(common),
            tx_prg: PioHoneywellTxProgram::new(common),
            mode: None,
        }
    }

    fn start(&mut self, mode: Mode) {
        if Some(mode) == self.mode {
            return;
        }

        self.stop_and_reset();

        let mut cfg = Config::default();

        match mode {
            Mode::Rx => {
                cfg.use_program(&self.rx_prg.prg, &[]);
                cfg.set_in_pins(&[&self.data_pin, &self.clk_pin]);
                cfg.set_jmp_pin(&self.data_pin);
                cfg.clock_divider = 1.to_fixed();
                cfg.shift_in = ShiftConfig {
                    threshold: 8,
                    direction: ShiftDirection::Left,
                    auto_fill: true,
                };
                cfg.shift_out = ShiftConfig {
                    threshold: 32,
                    direction: ShiftDirection::Left,
                    auto_fill: false,
                };
                cfg.fifo_join = FifoJoin::Duplex;

                self.sm
                    .set_pin_dirs(Direction::In, &[&self.data_pin, &self.clk_pin]);
            }
            Mode::Tx => {
                cfg.use_program(&self.tx_prg.prg, &[&self.data_pin]);
                cfg.set_in_pins(&[&self.clk_pin]);
                cfg.clock_divider = 1.to_fixed();
                cfg.shift_out = ShiftConfig {
                    threshold: 8,
                    direction: ShiftDirection::Left,
                    auto_fill: false,
                };
                cfg.fifo_join = FifoJoin::TxOnly;

                self.sm.set_pins(Level::Low, &[&self.data_pin]);
                self.sm.set_pin_dirs(Direction::Out, &[&self.data_pin]);
                self.sm.set_pin_dirs(Direction::In, &[&self.clk_pin]);
            }
        }

        self.sm.set_config(&cfg);
        self.sm.set_enable(true);

        self.mode = Some(mode);
    }

    pub fn stop_and_reset(&mut self) {
        self.sm.set_enable(false);
        self.sm.clear_fifos();
        self.sm.restart();

        self.mode = None;
    }

    pub async fn read_frame(&mut self) -> Result<Frame, Error> {
        self.start(Mode::Rx);

        let mut bytes = [0u8; 6];

        // Pre-load bit count - 1 to state machine
        self.sm
            .tx()
            .wait_push(((bytes.len() as u32 * u8::BITS) - 1) as u32)
            .await;

        for byte in bytes.iter_mut() {
            *byte = self.sm.rx().wait_pull().await as u8;
            debug!("Read word {=u8:#x}", byte);
        }

        bytes.deserialize()
    }

    pub async fn write_frame(&mut self, frame: &Frame) {
        self.start(Mode::Tx);

        for byte in frame.serialize().iter() {
            debug!("Writing byte {=u8:#x}", byte);
            self.sm.tx().wait_push((*byte as u32) << 24).await;
        }
    }
}
