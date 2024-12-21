use defmt::debug;
use fixed::traits::ToFixed;

use embassy_rp::gpio::Level;
use embassy_rp::pio::{
    Common, Config, Direction, FifoJoin, Instance, LoadedProgram, PioPin, ShiftConfig,
    ShiftDirection, StateMachine,
};

pub struct PioHoneywellTxProgram<'a, PIO: Instance> {
    prg: LoadedProgram<'a, PIO>,
}

impl<'a, PIO: Instance> PioHoneywellTxProgram<'a, PIO> {
    pub fn new(common: &mut Common<'a, PIO>) -> Self {
        let prg = pio_proc::pio_asm!(
            r#"
                .side_set 1 opt             ; DOUT is written via side set

                .wrap_target
                reset:
                    set y, 2                ; 3 words per frame
                preamble:                   ; LOW-LOW-LOW
                    wait 0 pin 0
                    wait 1 pin 0 side 0
                    wait 0 pin 0
                    wait 1 pin 0 side 0
                    wait 0 pin 0
                    wait 1 pin 0 side 0
                next_word:
                    pull
                write_bit:                  ; All bits start with HIGH
                    out x, 1
                    wait 0 pin 0
                    wait 1 pin 0 side 1
                    jmp !x tx_0
                tx_1:                       ; '1' is HIGH next
                    wait 0 pin 0
                    wait 1 pin 0 side 1
                    jmp end_bit
                tx_0:                       ; '0' is LOW next
                    wait 0 pin 0
                    wait 1 pin 0 side 0
                end_bit:                    ; All bits end with LOW
                    wait 0 pin 0
                    wait 1 pin 0 side 0
                    jmp !osre write_bit
                    jmp y-- next_word
                postamble:                  ; HIGH-HIGH-HIGH
                    wait 0 pin 0
                    wait 1 pin 0 side 1
                    wait 0 pin 0
                    wait 1 pin 0 side 1
                    wait 0 pin 0
                    wait 1 pin 0 side 1
                .wrap
            "#
        );

        let prg = common.load_program(&prg.program);

        Self { prg }
    }
}

pub struct PioHoneywellTx<'a, PIO: Instance, const S: usize> {
    sm: StateMachine<'a, PIO, S>,
}

impl<'a, PIO: Instance, const S: usize> PioHoneywellTx<'a, PIO, S> {
    pub fn new(
        common: &mut Common<'a, PIO>,
        mut sm: StateMachine<'a, PIO, S>,
        data_pin: impl PioPin,
        clk_pin: impl PioPin,
        program: &PioHoneywellTxProgram<'a, PIO>,
    ) -> Self {
        let data_pin = common.make_pio_pin(data_pin);
        let clk_pin = common.make_pio_pin(clk_pin);

        let mut cfg = Config::default();
        cfg.use_program(&program.prg, &[&data_pin]);
        cfg.set_in_pins(&[&clk_pin]);
        cfg.clock_divider = 1.to_fixed();
        cfg.shift_out = ShiftConfig {
            threshold: 16,
            direction: ShiftDirection::Left,
            auto_fill: false,
        };
        cfg.fifo_join = FifoJoin::TxOnly;

        sm.set_config(&cfg);

        sm.set_pins(Level::Low, &[&data_pin]);
        sm.set_pin_dirs(Direction::Out, &[&data_pin]);
        sm.set_pin_dirs(Direction::In, &[&clk_pin]);

        sm.set_enable(true);

        Self { sm }
    }

    pub async fn write_frame(&mut self, frame: &[u16; 3]) {
        for word in frame.iter() {
            debug!("Writing word {=u16:x}", word);
            self.sm.tx().wait_push((*word as u32) << 16).await;
        }
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

pub struct PioHoneywellRx<'a, PIO: Instance, const S: usize> {
    sm: StateMachine<'a, PIO, S>,
}

impl<'a, PIO: Instance, const S: usize> PioHoneywellRx<'a, PIO, S> {
    pub fn new(
        common: &mut Common<'a, PIO>,
        mut sm: StateMachine<'a, PIO, S>,
        data_pin: impl PioPin,
        clk_pin: impl PioPin,
        program: &PioHoneywellRxProgram<'a, PIO>,
    ) -> Self {
        let data_pin = common.make_pio_pin(data_pin);
        let clk_pin = common.make_pio_pin(clk_pin);

        let mut cfg = Config::default();
        cfg.use_program(&program.prg, &[]);
        cfg.set_in_pins(&[&data_pin, &clk_pin]);
        cfg.set_jmp_pin(&data_pin);
        cfg.clock_divider = 1.to_fixed();
        cfg.shift_in = ShiftConfig {
            threshold: 16,
            direction: ShiftDirection::Left,
            auto_fill: true,
        };
        cfg.shift_out = ShiftConfig {
            threshold: 32,
            direction: ShiftDirection::Left,
            auto_fill: false,
        };
        cfg.fifo_join = FifoJoin::Duplex;

        sm.set_config(&cfg);

        sm.set_pin_dirs(Direction::In, &[&data_pin, &clk_pin]);

        sm.set_enable(true);

        Self { sm }
    }

    pub async fn read_frame(&mut self) -> [u16; 3] {
        let mut frame = [0u16; 3];

        // Pre-load bit count - 1 to state machine
        self.sm
            .tx()
            .wait_push(((frame.len() * 16) - 1) as u32)
            .await;

        for word in frame.iter_mut() {
            *word = self.sm.rx().wait_pull().await as u16;
            debug!("Read word {=u16:x}", word);
        }

        frame
    }
}
