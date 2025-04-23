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
    cmd: LoadedProgram<'d, PIO>,
    oneb_tx: LoadedProgram<'d, PIO>,
    oneb_rx: LoadedProgram<'d, PIO>,
}

impl<'d, PIO: Instance> PioSdPrograms<'d, PIO> {
    /// Loads 23 instructions into pio, and uses 4 state machines
    pub fn new(common: &mut Common<'d, PIO>) -> Self {
        let clk = pio_asm!(
            ".side_set 1",
            "irq 0 side 1",
            "irq clear 0 side 0",
            options(max_program_size = 2)
        );

        // the lower half of the second word sent needs to contain the length of the response
        let cmd = pio_asm!(
            ".side_set 1 opt pindirs",
            "out y, 32", // Store 47 in y (for write loop)
            ".wrap_target",
            // Write CMD
            "mov x, y          side 1", // (re)set counter, set cmd as output
            "wait 0 irq 0",
            "lp_write:",
            "out pins, 1",
            "jmp x-- lp_write",
            // Read CMD
            "out x, 16        side 0", // move counter to x (47 or 135), set cmd as input
            "wait 0 irq 0",
            "lp_read:",
            "in pins, 1",
            "jmp x-- lp_read",
            "push", // push remaining 16 bits from isr
            ".wrap",
            options(max_program_size = 10)
        );

        // before data can be written, length of data in bits, needs to be pushed
        let oneb_tx = pio_asm!(
            "set pins, 1", // idle high
            "out x, 32",   // stall waiting for data len
            "set pindirs, 1",
            "wait 0 irq 0", // sync clk
            "loop:",
            "out pins, 1",
            "jmp x-- loop",
            options(max_program_size = 7)
        );

        // requires sm to be stopped when not reading to avoid conflicts with tx
        let oneb_rx = pio_asm!(
            "set pindirs, 0",
            "wait 0 pin, 0", // wait until DAT goes low (start bit)
            "wait 1 irq 0",  // sync clk
            "loop:",
            "in pins, 1",
            "jmp loop",
            options(max_program_size = 5)
        );

        Self {
            clk: common.load_program(&clk.program),
            cmd: common.load_program(&cmd.program),
            oneb_tx: common.load_program(&oneb_tx.program),
            oneb_rx: common.load_program(&oneb_rx.program),
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
    const SM3: usize,
> {
    pub dma: PeripheralRef<'d, C>,
    pub cfg: pio::Config<'d, PIO>,
    pub clk_sm: StateMachine<'d, PIO, SM0>,
    pub cmd_sm: StateMachine<'d, PIO, SM1>,
    pub tx_sm: StateMachine<'d, PIO, SM2>,
    pub rx_sm: StateMachine<'d, PIO, SM3>,
}

impl<
    'd,
    PIO: Instance,
    C: Channel,
    const SM0: usize,
    const SM1: usize,
    const SM2: usize,
    const SM3: usize,
> PioSd<'d, PIO, C, SM0, SM1, SM2, SM3>
{
    pub fn new_1_bit(
        pio: &mut Common<'d, PIO>,
        mut clk_sm: StateMachine<'d, PIO, SM0>,
        mut cmd_sm: StateMachine<'d, PIO, SM1>,
        mut tx_sm: StateMachine<'d, PIO, SM2>,
        mut rx_sm: StateMachine<'d, PIO, SM3>,
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

        // Cmd program config
        let cmd = pio.make_pio_pin(cmd_pin);
        let mut cfg = pio::Config::default();
        cfg.use_program(&programs.cmd, &[&cmd]);
        cfg.set_out_pins(&[&cmd]);
        cfg.set_in_pins(&[&cmd]);
        cfg.clock_divider = div.into();

        let mut shift_cfg = ShiftConfig::default();
        shift_cfg.threshold = 32;
        shift_cfg.direction = ShiftDirection::Left;
        shift_cfg.auto_fill = true;
        cfg.shift_out = shift_cfg;
        cfg.shift_in = shift_cfg;

        cmd_sm.set_config(&cfg);
        cmd_sm.clear_fifos();
        cmd_sm.tx().push(47); // write loop counter
        cmd_sm.set_enable(true);

        let d0 = pio.make_pio_pin(d0_pin);

        // Tx program config
        let mut cfg = pio::Config::default();
        cfg.use_program(&programs.oneb_tx, &[]);
        cfg.set_out_pins(&[&d0]);
        cfg.set_set_pins(&[&d0]);
        cfg.clock_divider = div.into();
        cfg.fifo_join = FifoJoin::TxOnly;
        let mut shift_cfg = ShiftConfig::default();
        shift_cfg.threshold = 32;
        shift_cfg.direction = ShiftDirection::Left;
        shift_cfg.auto_fill = true;
        cfg.shift_in = shift_cfg;
        tx_sm.set_config(&cfg);
        tx_sm.clear_fifos();
        tx_sm.set_enable(true);

        // Rx program config
        let mut cfg = pio::Config::default();
        cfg.use_program(&programs.oneb_rx, &[]);
        cfg.set_in_pins(&[&d0]);
        cfg.set_set_pins(&[&d0]);
        cfg.clock_divider = div.into();
        cfg.fifo_join = FifoJoin::RxOnly;
        let mut shift_cfg = ShiftConfig::default();
        shift_cfg.threshold = 32;
        shift_cfg.direction = ShiftDirection::Left;
        shift_cfg.auto_fill = true;
        cfg.shift_in = shift_cfg;
        rx_sm.set_config(&cfg);
        rx_sm.clear_fifos();
        rx_sm.set_enable(false);

        Self {
            dma,
            cfg,
            clk_sm,
            cmd_sm,
            tx_sm,
            rx_sm,
        }
    }

    /// Will change the pio frequency and restart the state machine and clear any fifos.
    /// DONT use while any communication is occurring
    pub fn set_freq(&mut self, freq: u32) {
        self.cfg.clock_divider = ((clk_sys_freq() / freq) as u16).into();
        self.clk_sm.restart();
        self.cmd_sm.restart();
        self.tx_sm.restart();
        self.rx_sm.restart();
        self.cmd_sm.tx().push(47);
    }

    /// Spin until the card returns 0xFF, or we spin too many times and
    /// timeout.
    fn wait_not_busy(&mut self, mut delay: Delay) -> Result<(), Error> {
        loop {
            let s = self.read_cmd()?;
            if s == 0xFF {
                break;
            }
            delay.delay(&mut self.delayer, Error::TimeoutWaitNotBusy)?;
        }
        Ok(())
    }

    /// writes command
    fn write_cmd(&mut self, command: u8, arg: u32) -> Result<u8, Error> {
        // assert!(cmd.len() == 2);

        // // write 2 cmd words
        // self.cmd_sm.tx().push(cmd[0]); // first 32 bits of cmd
        // self.cmd_sm.tx().push(((cmd[1]) << 16) | response as u32); // last 16 bits of cmd + response len
        //
    }

    fn read_cmd(&mut self, buff: &mut [u32]) {
        for b in buff.iter_mut() {
            if let Some(word) = self.cmd_sm.rx().try_pull() {
                *b = word
            } else {
                return;
            }
        }
    }

    // write single byte to bus
    fn write_word<'b>(&'b mut self, word: u32) {
        self.tx_sm.tx().push(31);
        while self.tx_sm.tx().full() {}
        self.tx_sm.tx().push(word);
    }

    // read single byte from bus
    fn read_word<'b>(&'b mut self) -> u32 {
        self.rx_sm.clear_fifos();
        self.rx_sm.restart();
        self.rx_sm.set_enable(true);

        while self.rx_sm.rx().empty() {}
        self.rx_sm.rx().pull()
    }

    /// Return an in-prograss dma transfer future. Awaiting it will guarantee a complete transfer.
    fn write<'b>(&'b mut self, buff: &'b [u32]) -> Transfer<'b, C> {
        self.tx_sm.tx().push(buff.len() as u32 * 31);
        self.tx_sm.tx().dma_push(self.dma.reborrow(), buff, false)
    }

    /// Return an in-prograss dma transfer future. Awaiting it will guarantee a complete transfer.
    fn read<'b>(&'b mut self, buff: &'b mut [u32]) -> Transfer<'b, C> {
        self.rx_sm.clear_fifos();
        self.rx_sm.restart();
        self.rx_sm.set_enable(true);

        self.rx_sm.rx().dma_pull(self.dma.reborrow(), buff, false)
    }
}

/// This an object you can use to busy-wait with a timeout.
///
/// Will let you call `delay` up to `max_retries` times before `delay` returns
/// an error.
struct Delay {
    retries_left: u32,
}

impl Delay {
    /// The default number of retries for a read operation.
    ///
    /// At ~10us each this is ~100ms.
    ///
    /// See `Part1_Physical_Layer_Simplified_Specification_Ver9.00-1.pdf` Section 4.6.2.1
    pub const DEFAULT_READ_RETRIES: u32 = 10_000;

    /// The default number of retries for a write operation.
    ///
    /// At ~10us each this is ~500ms.
    ///
    /// See `Part1_Physical_Layer_Simplified_Specification_Ver9.00-1.pdf` Section 4.6.2.2
    pub const DEFAULT_WRITE_RETRIES: u32 = 50_000;

    /// The default number of retries for a control command.
    ///
    /// At ~10us each this is ~100ms.
    ///
    /// No value is given in the specification, so we pick the same as the read timeout.
    pub const DEFAULT_COMMAND_RETRIES: u32 = 10_000;

    /// Create a new Delay object with the given maximum number of retries.
    fn new(max_retries: u32) -> Delay {
        Delay {
            retries_left: max_retries,
        }
    }

    /// Create a new Delay object with the maximum number of retries for a read operation.
    fn new_read() -> Delay {
        Delay::new(Self::DEFAULT_READ_RETRIES)
    }

    /// Create a new Delay object with the maximum number of retries for a write operation.
    fn new_write() -> Delay {
        Delay::new(Self::DEFAULT_WRITE_RETRIES)
    }

    /// Create a new Delay object with the maximum number of retries for a command operation.
    fn new_command() -> Delay {
        Delay::new(Self::DEFAULT_COMMAND_RETRIES)
    }

    /// Wait for a while.
    ///
    /// Checks the retry counter first, and if we hit the max retry limit, the
    /// value `err` is returned. Otherwise we wait for 10us and then return
    /// `Ok(())`.
    fn delay<T>(&mut self, delayer: &mut T, err: Error) -> Result<(), Error>
    where
        T: embedded_hal::delay::DelayNs,
    {
        if self.retries_left == 0 {
            Err(err)
        } else {
            delayer.delay_us(10);
            self.retries_left -= 1;
            Ok(())
        }
    }
}
