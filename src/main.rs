#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use defmt::assert;
use defmt::{info, unwrap, warn};
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::clocks::clk_sys_freq;
use embassy_rp::dma::{Channel, Transfer};
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::program::pio_asm;
use embassy_rp::pio::{
    self, Common, Direction, FifoJoin, Instance, InterruptHandler, LoadedProgram, Pio, PioPin,
    ShiftConfig, ShiftDirection, StateMachine,
};
use embassy_rp::{PeripheralRef, into_ref};
use embassy_time::Timer;
use embedded_sdmmc::{SdCard, TimeSource, Timestamp, VolumeIdx, VolumeManager};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Running");
    let p = embassy_rp::init(Default::default());
    let Pio {
        mut common,
        sm0,
        sm1,
        sm2,
        sm3,
        ..
    } = Pio::new(p.PIO0, Irqs);

    let programs = PioSdioPrograms::new(&mut common);
    let mut sdio = PioSdio::new_1_bit(
        &mut common,
        sm0,
        sm1,
        sm2,
        sm3,
        p.PIN_2,
        p.PIN_3,
        p.PIN_4,
        programs,
        p.DMA_CH0,
    );

    info!("Done!!!");

    loop {}
}

pub enum CmdResponse {
    Long = 135,
    Short = 47,
}

pub struct PioSdioPrograms<'d, PIO: Instance> {
    clk: LoadedProgram<'d, PIO>,
    cmd: LoadedProgram<'d, PIO>,
    oneb_tx: LoadedProgram<'d, PIO>,
    oneb_rx: LoadedProgram<'d, PIO>,
}

impl<'d, PIO: Instance> PioSdioPrograms<'d, PIO> {
    pub fn new(pio: &mut Common<'d, PIO>) -> Self {
        let clk = pio_asm!(".side_set 1", "irq 0 side 1", "irq clear 0 side 0");

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
        );

        // the first word pushed before any data must be the length of the data
        // when not writing, pin direction is set as input and the write flag is cleared allowing reads to occur
        let oneb_tx = pio_asm!(
            ".side_set 1 opt pindirs",
            ".wrap_target",
            "pull block",            // wait for data
            "out y, 32",             // set y to counter
            "irq 1          side 1", // raise write, set d0 as output
            "wait 0 irq 0",
            "lp:", // write word from osr
            "out pins, 1",
            "jmp y-- lp",
            "irq clear 1    side 0", // clear write, set d0 back as input
            ".wrap",
        );

        // before data can be read, the length of data needs to be set
        // pin direction and read/write irqs are managed by oneb_tx
        let oneb_rx = pio_asm!(
            ".wrap_target",
            "wait 1 irq 0", // wait for not writing
            "wait 0 irq 0",
            "lp:", // read word into isr
            "in pins, 1",
            "jmp x-- lp",
            ".wrap",
        );

        Self {
            clk: pio.load_program(&clk.program),
            cmd: pio.load_program(&cmd.program),
            oneb_tx: pio.load_program(&oneb_tx.program),
            oneb_rx: pio.load_program(&oneb_rx.program),
        }
    }
}

pub struct PioSdio<
    'd,
    PIO: Instance,
    C: Channel,
    const SM0: usize,
    const SM1: usize,
    const SM2: usize,
    const SM3: usize,
> {
    dma: PeripheralRef<'d, C>,
    cfg: pio::Config<'d, PIO>,
    clk_sm: StateMachine<'d, PIO, SM0>,
    cmd_sm: StateMachine<'d, PIO, SM1>,
    tx_sm: StateMachine<'d, PIO, SM2>,
    rx_sm: StateMachine<'d, PIO, SM3>,
}

impl<
    'd,
    PIO: Instance,
    C: Channel,
    const SM0: usize,
    const SM1: usize,
    const SM2: usize,
    const SM3: usize,
> PioSdio<'d, PIO, C, SM0, SM1, SM2, SM3>
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
        programs: PioSdioPrograms<'d, PIO>,
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

        let mut shift_cfg = ShiftConfig::default();
        shift_cfg.threshold = 32;
        shift_cfg.direction = ShiftDirection::Left;
        shift_cfg.auto_fill = true;
        cfg.shift_in = shift_cfg;

        cmd_sm.set_config(&cfg);
        cmd_sm.clear_fifos();
        cmd_sm.tx().push(47); // write loop counter
        cmd_sm.set_enable(true);

        let d0 = pio.make_pio_pin(d0_pin);

        // Tx program config
        let mut cfg = pio::Config::default();
        cfg.use_program(&programs.oneb_tx, &[&d0]);
        cfg.set_out_pins(&[&d0]);
        cfg.clock_divider = div.into();
        cfg.fifo_join = FifoJoin::TxOnly;
        tx_sm.set_config(&cfg);
        tx_sm.set_enable(true);

        // Rx program config
        let mut cfg = pio::Config::default();
        cfg.use_program(&programs.oneb_rx, &[]);
        cfg.set_in_pins(&[&d0]);
        cfg.clock_divider = div.into();
        cfg.fifo_join = FifoJoin::RxOnly;
        rx_sm.set_config(&cfg);
        rx_sm.set_enable(true);

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

    /// writes command
    fn write_cmd(&mut self, cmd: &[u32], response: CmdResponse) {
        assert!(cmd.len() == 2);

        // write 2 cmd words
        self.cmd_sm.tx().push(cmd[0]); // first 32 bits of cmd
        self.cmd_sm.tx().push(((cmd[1]) << 16) | response as u32); // last 16 bits of cmd + response len
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

    /// Return an in-prograss dma transfer future. Awaiting it will guarantee a complete transfer.
    fn write<'b>(&'b mut self, buff: &'b [u32], len: u32) -> Transfer<'b, C> {
        self.tx_sm.tx().push(len);
        self.tx_sm.tx().dma_push(self.dma.reborrow(), buff, false)
    }

    /// Return an in-prograss dma transfer future. Awaiting it will guarantee a complete transfer.
    fn read<'b>(&'b mut self, buff: &'b mut [u32], len: u32) -> Transfer<'b, C> {
        // Because the fifos are tied, there is no other way to set counter
        unsafe {
            self.rx_sm.set_x(len);
        }
        self.rx_sm.rx().dma_pull(self.dma.reborrow(), buff, false)
    }
}
