//! Implementation of the Honeywell wireless doorbell protocol using PIOs.
//!
//! Data is read from and written to a data pin, both reading and writing need an external clock.
//!
//! Frame implementation according to <https://github.com/klohner/honeywell-wireless-doorbell>:
//!
//! ```text
//! Frame bits used in Honeywell RCWL300A, RCWL330A, Series 3, 5, 9 and all Decor Series Wireless Chimes
//! 0000 0000 1111 1111 2222 2222 3333 3333 4444 4444 5555 5555
//! 7654 3210 7654 3210 7654 3210 7654 3210 7654 3210 7654 3210
//! XXXX XXXX XXXX XXXX XXXX XXXX XXXX XXXX XXXX XX.. XXX. .... KEY DATA (any change and receiver doesn't seem to recognize signal)
//! XXXX XXXX XXXX XXXX XXXX .... .... .... .... .... .... .... KEY ID (different for each transmitter)
//! .... .... .... .... .... 0000 00.. 0000 0000 00.. 000. .... KEY UNKNOWN 0 (always 0 in devices I've tested)
//! .... .... .... .... .... .... ..XX .... .... .... .... .... DEVICE TYPE (10 = doorbell, 01 = PIR Motion sensor)
//! .... .... .... .... .... .... .... .... .... ..XX ...X XXX. FLAG DATA (may be modified for possible effects on receiver)
//! .... .... .... .... .... .... .... .... .... ..XX .... .... ALERT (00 = normal, 01 or 10 = right-left halo light pattern, 11 = full volume alarm)
//! .... .... .... .... .... .... .... .... .... .... ...X .... SECRET KNOCK (0 = default, 1 if doorbell is pressed 3x rapidly)
//! .... .... .... .... .... .... .... .... .... .... .... X... RELAY (1 if signal is a retransmission of a received transmission, only some models)
//! .... .... .... .... .... .... .... .... .... .... .... .X.. FLAG UNKNOWN (0 = default, but 1 is accepted and I don't observe any effects)
//! .... .... .... .... .... .... .... .... .... .... .... ..X. LOWBAT (1 if battery is low, receiver gives low battery alert)
//! .... .... .... .... .... .... .... .... .... .... .... ...X PARITY (LSB of count of set bits in previous 47 bits)
//! ```

use arbitrary_int::u20;
use defmt::{debug, trace, Format};
use fixed::traits::ToFixed;
use thiserror::Error;

use embassy_rp::gpio::Level;
use embassy_rp::pio::{
    Common, Config, Direction, FifoJoin, Instance, LoadedProgram, Pin, PioPin, ShiftConfig,
    ShiftDirection, StateMachine,
};

/// Errors than can occur during [`Frame`] deserialization.
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

#[derive(Format, Debug, Clone, Copy, PartialEq)]
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

#[derive(Format, Debug, Clone, Copy, PartialEq)]
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

/// Representation of a Honeywell wireless doorbell protocol frame.
#[derive(Format, Debug, Clone, Copy, PartialEq)]
pub struct Frame {
    /// Device id, can be 0 to trigger all receivers
    pub id: u20,

    /// Type of the sending device
    pub device_type: DeviceType,

    /// Type of the alert
    pub alert: AlertType,

    /// Secret knock flag of the alert
    pub secret_knock: bool,

    /// Frame shall be relayed
    pub relay: bool,

    /// Sending device has low battery
    pub low_battery: bool,
}

impl Default for Frame {
    fn default() -> Self {
        Self {
            id: u20::new(0),
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

    /// Serialize the frame to raw byte, attaching the checksum.
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
    /// Deserialize raw bytes into a frame struct.
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
            id: u20::extract_u32(
                u32::from_be_bytes(self[..4].try_into().expect("Has length 4.")),
                12,
            ),
            device_type: ((self[3] & 0x30) >> 4).try_into()?,
            alert: (self[4] & 0x03).try_into()?,
            secret_knock: (self[5] & 0x10) != 0,
            relay: (self[5] & 0x08) != 0,
            low_battery: (self[5] & 0x02) != 0,
        })
    }
}

pub enum Mode<'a, PIO: Instance> {
    Rx(LoadedProgram<'a, PIO>),
    Tx(LoadedProgram<'a, PIO>),
}

pub struct PioHoneywell<'a, PIO: Instance, const S: usize> {
    data_pin: Pin<'a, PIO>,
    clk_pin: Pin<'a, PIO>,
    common: Common<'a, PIO>,
    sm: StateMachine<'a, PIO, S>,
    mode: Option<Mode<'a, PIO>>,
}

impl<'a, PIO: Instance, const S: usize> PioHoneywell<'a, PIO, S> {
    pub fn new(
        mut common: Common<'a, PIO>,
        sm: StateMachine<'a, PIO, S>,
        data_pin: impl PioPin,
        clk_pin: impl PioPin,
    ) -> Self {
        let data_pin = common.make_pio_pin(data_pin);
        let clk_pin = common.make_pio_pin(clk_pin);

        Self {
            data_pin,
            clk_pin,
            common,
            sm,
            mode: None,
        }
    }

    /// Load and start RX PIO program.
    fn start_rx(&mut self) {
        self.stop_and_reset();

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

        let loaded_prg = self.common.load_program(&prg.program);

        let mut cfg = Config::default();
        cfg.use_program(&loaded_prg, &[]);
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

        self.sm.set_pin_dirs(Direction::Out, &[]);
        self.sm
            .set_pin_dirs(Direction::In, &[&self.data_pin, &self.clk_pin]);
        self.sm.set_config(&cfg);
        self.sm.set_enable(true);

        self.mode = Some(Mode::Rx(loaded_prg));

        debug!("Switched to RX mode.");
    }

    /// Load and start TX PIO program.
    fn start_tx(&mut self) {
        self.stop_and_reset();

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

        let loaded_prg = self.common.load_program(&prg.program);

        let mut cfg = Config::default();
        cfg.use_program(&loaded_prg, &[&self.data_pin]);
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
        self.sm.set_config(&cfg);
        self.sm.set_enable(true);

        self.mode = Some(Mode::Tx(loaded_prg));

        debug!("Switched to TX mode.");
    }

    /// Stop PIO and prepare for a new program to be loaded.
    pub fn stop_and_reset(&mut self) {
        self.sm.set_enable(false);
        self.sm.clear_fifos();
        self.sm.restart();

        match self.mode.take() {
            Some(Mode::Rx(prg)) => unsafe {
                self.common.free_instr(prg.used_memory);
            },
            Some(Mode::Tx(prg)) => unsafe {
                self.common.free_instr(prg.used_memory);
            },
            None => (),
        }

        debug!("Reset complete.");
    }

    /// Await a valid frame on the data pin clocked by the external clock on the clock pin.
    pub async fn read_frame(&mut self) -> Frame {
        self.start_rx();

        let mut bytes = [0u8; 6];

        loop {
            // Pre-load bit count - 1 to state machine
            self.sm
                .tx()
                .wait_push((bytes.len() as u32 * u8::BITS) - 1)
                .await;

            for byte in bytes.iter_mut() {
                *byte = self.sm.rx().wait_pull().await as u8;
                trace!("Read word {=u8:#x}", byte);
            }

            if let Ok(frame) = bytes.deserialize() {
                return frame;
            }

            debug!("Recieved invalid frame");
        }
    }

    /// Write the given frame to the data pin clocked by the external clock on the clock pin.
    pub async fn write_frame(&mut self, frame: &Frame, count: u8) {
        self.start_tx();

        for _ in 0..count {
            for byte in frame.serialize().iter() {
                trace!("Writing byte {=u8:#x}", byte);
                self.sm.tx().wait_push((*byte as u32) << 24).await;
            }
        }
    }
}
