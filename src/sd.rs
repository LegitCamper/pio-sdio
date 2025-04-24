use embassy_rp::clocks::clk_sys_freq;
use embassy_rp::dma::{Channel, Transfer};
use embassy_rp::pio::program::pio_asm;
use embassy_rp::pio::{
    self, Common, Direction, FifoJoin, Instance, InterruptHandler, LoadedProgram, Pio, PioPin,
    ShiftConfig, ShiftDirection, StateMachine,
};
use embassy_rp::{PeripheralRef, into_ref};
use embassy_time::Timer;
use embedded_sdmmc::sdcard::proto::*;
use embedded_sdmmc::sdcard::{AcquireOpts, CardType, Error};
use embedded_sdmmc::{SdCard, TimeSource, Timestamp, VolumeIdx, VolumeManager};

pub struct PioSdPrograms<'d, PIO: Instance> {
    clk: LoadedProgram<'d, PIO>,
    cmd_tx: LoadedProgram<'d, PIO>,
    cmd_rx: LoadedProgram<'d, PIO>,
    // oneb_tx: LoadedProgram<'d, PIO>,
    // oneb_rx: LoadedProgram<'d, PIO>,
}

impl<'d, PIO: Instance> PioSdPrograms<'d, PIO> {
    pub fn new(common: &mut Common<'d, PIO>) -> Self {
        let clk = pio_asm!(".side_set 1", "irq 0 side 1", "irq clear 0 side 0",);

        // This is the default program for sm1
        // before starting the sm, ensure to push 47bit counter
        // irq1 raised when cmd write is done
        let cmd_tx = pio_asm!(
            ".side_set 1 opt pindirs",
            "set pins, 1     side 1" // idle high, set output
            "out y, 32", // stall and wait for cmd
            "irq clear 1", // write in progress
            "wait 0 irq 0", // sync clk
            "set pins, 0",
            "nop",
            "loop:",
            "out pins, 1",
            "jmp y-- loop",
            "irq set 1", // raise cmd write done
        );

        let cmd_rx = pio_asm!(
            "set pindirs 0",
            "wait 1 pin, 0"
            "wait 0 pin, 0", // wait until CMD goes low (start bit)
            "wait 0 irq 0",  // sync clk
            "loop:",
            "in pins, 1",
            "jmp loop",
        );

        // // before data can be written, length of data in bits, needs to be pushed
        // let oneb_tx = pio_asm!(
        //     "set pins, 1", // idle high
        //     "out x, 32",   // stall waiting for data len
        //     "set pindirs, 1",
        //     "wait 0 irq 0", // sync clk
        //     "loop:",
        //     "out pins, 1",
        //     "jmp x-- loop",
        // );

        // // requires sm to be stopped when not reading to avoid conflicts with tx
        // let oneb_rx = pio_asm!(
        //     "set pindirs, 0",
        //     "wait 0 pin, 0", // wait until DAT goes low (start bit)
        //     "wait 1 irq 0",  // sync clk
        //     "loop:",
        //     "in pins, 1",
        //     "jmp loop",
        // );

        Self {
            clk: common.load_program(&clk.program),
            cmd_tx: common.load_program(&cmd_tx.program),
            cmd_rx: common.load_program(&cmd_rx.program),
            // oneb_tx: common.load_program(&oneb_tx.program),
            // oneb_rx: common.load_program(&oneb_rx.program),
        }
    }
}

pub struct PioSd<
    'd,
    PIO: Instance,
    C: Channel,
    const SM0: usize,
    const SM1: usize,
    const SM2: usize,
> {
    pub dma: PeripheralRef<'d, C>,
    pub cfg: pio::Config<'d, PIO>,
    pub clk_irq: pio::Irq<'d, PIO, 0>,
    pub clk_sm: StateMachine<'d, PIO, SM0>,
    pub cmd_tx_irq: pio::Irq<'d, PIO, 1>,
    pub cmd_tx: pio::Config<'d, PIO>,
    pub cmd_rx: pio::Config<'d, PIO>,
    pub cmd_sm: StateMachine<'d, PIO, SM1>,
    // pub data_sm: StateMachine<'d, PIO, SM2>,
}

impl<'d, PIO: Instance, C: Channel, const SM0: usize, const SM1: usize, const SM2: usize>
    PioSd<'d, PIO, C, SM0, SM1, SM2>
{
    pub fn new_1_bit(
        pio: &mut Common<'d, PIO>,
        mut clk_sm: StateMachine<'d, PIO, SM0>,
        clk_irq: pio::Irq<'d, PIO, 0>,
        mut cmd_sm: StateMachine<'d, PIO, SM1>,
        cmd_tx_irq: pio::Irq<'d, PIO, 1>,
        mut data_sm: StateMachine<'d, PIO, SM2>,
        clk_pin: impl PioPin,
        cmd_pin: impl PioPin,
        d0_pin: impl PioPin,
        programs: PioSdPrograms<'d, PIO>,
        dma: C,
    ) -> Self {
        into_ref!(dma);
        let div = (clk_sys_freq() / 400_000) as u16;

        // Clk program config
        let clk = pio.make_pio_pin(clk_pin);
        let mut cfg = pio::Config::default();
        cfg.use_program(&programs.clk, &[&clk]);
        cfg.clock_divider = div.into();
        clk_sm.set_config(&cfg);
        clk_sm.set_pin_dirs(Direction::Out, &[&clk]);
        clk_sm.set_enable(true);

        // Cmd program configs
        let cmd = pio.make_pio_pin(cmd_pin);
        let cmd_tx = Self::cmd_config(&cmd, &programs, div, true);
        cmd_sm.clear_fifos();
        cmd_sm.set_config(&cmd_tx);
        cmd_sm.set_enable(true);
        let cmd_rx = Self::cmd_config(&pio.make_pio_pin(d0_pin), &programs, div, false);

        // let d0 = pio.make_pio_pin(d0_pin);

        // // Tx program config
        // let mut cfg = pio::Config::default();
        // cfg.use_program(&programs.oneb_tx, &[]);
        // cfg.set_out_pins(&[&d0]);
        // cfg.set_set_pins(&[&d0]);
        // cfg.clock_divider = div.into();
        // cfg.fifo_join = FifoJoin::TxOnly;
        // let mut shift_cfg = ShiftConfig::default();
        // shift_cfg.threshold = 32;
        // shift_cfg.direction = ShiftDirection::Left;
        // shift_cfg.auto_fill = true;
        // cfg.shift_in = shift_cfg;
        // tx_sm.set_config(&cfg);
        // tx_sm.clear_fifos();
        // tx_sm.set_enable(true);

        // // Rx program config
        // let mut cfg = pio::Config::default();
        // cfg.use_program(&programs.oneb_rx, &[]);
        // cfg.set_in_pins(&[&d0]);
        // cfg.set_set_pins(&[&d0]);
        // cfg.clock_divider = div.into();
        // cfg.fifo_join = FifoJoin::RxOnly;
        // let mut shift_cfg = ShiftConfig::default();
        // shift_cfg.threshold = 32;
        // shift_cfg.direction = ShiftDirection::Left;
        // shift_cfg.auto_fill = true;
        // cfg.shift_in = shift_cfg;
        // rx_sm.set_config(&cfg);
        // rx_sm.clear_fifos();
        // rx_sm.set_enable(false);

        Self {
            dma,
            cfg,
            clk_irq,
            clk_sm,
            cmd_tx_irq,
            cmd_tx,
            cmd_rx,
            cmd_sm,
            // tx_sm,
            // rx_sm,
        }
    }

    pub fn cmd_config(
        pin: &pio::Pin<'d, PIO>,
        programs: &PioSdPrograms<'d, PIO>,
        div: u16,
        tx: bool,
    ) -> pio::Config<'d, PIO> {
        let mut cfg = pio::Config::default();

        let mut shift_cfg = ShiftConfig::default();
        shift_cfg.threshold = 32;
        shift_cfg.direction = ShiftDirection::Left;
        shift_cfg.auto_fill = true;

        if tx {
            cfg.use_program(&programs.cmd_tx, &[&pin]);
            cfg.set_set_pins(&[&pin]);
            cfg.set_out_pins(&[&pin]);
            cfg.fifo_join = FifoJoin::TxOnly;
            cfg.shift_out = shift_cfg;
        } else {
            cfg.use_program(&programs.cmd_rx, &[]);
            cfg.set_set_pins(&[&pin]);
            cfg.set_in_pins(&[&pin]);
            cfg.fifo_join = FifoJoin::RxOnly;
            cfg.shift_in = shift_cfg;
        }
        cfg.clock_divider = div.into();
        cfg
    }

    // /// Will change the pio frequency and restart the state machine and clear any fifos.
    // /// DONT use while any communication is occurring
    // pub fn set_freq(&mut self, freq: u32) {
    //     self.cfg.clock_divider = ((clk_sys_freq() / freq) as u16).into();
    //     self.clk_sm.restart();
    //     self.cmd_sm.restart();
    //     self.tx_sm.restart();
    //     self.rx_sm.restart();
    //     self.cmd_sm.tx().push(47);
    // }
}
