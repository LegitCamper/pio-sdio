#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use defmt::assert;
use defmt::{info, unwrap, warn};
use embassy_executor::Spawner;
use embassy_rp::Peripheral;
use embassy_rp::bind_interrupts;
use embassy_rp::clocks::RoscRng;
use embassy_rp::clocks::clk_sys_freq;
use embassy_rp::dma::{AnyChannel, Channel};
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::program::{
    Assembler, InSource, MovDestination, MovOperation, MovSource, OutDestination, SideSet, pio_asm,
};
use embassy_rp::pio::{
    self, Common, Direction, Instance, InterruptHandler, LoadedProgram, Pio, PioPin, ShiftConfig,
    ShiftDirection, StateMachine,
};
use embassy_time::Timer;
use embedded_sdmmc::{SdCard, TimeSource, Timestamp, VolumeIdx, VolumeManager};
use rand::{Rng, RngCore};
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
        ..
    } = Pio::new(p.PIO0, Irqs);

    let programs = PioSdioPrograms::new(&mut common);
    let mut sdio = PioSdio::new(
        &mut common,
        sm0,
        sm1,
        p.PIN_2,
        p.PIN_3,
        p.PIN_4,
        programs,
        p.DMA_CH0,
    );

    info!("Done!!!");

    info!("Y: {}", unsafe { sdio.cmd_sm.get_y() },);
    loop {
        if sdio.cmd_sm.tx().empty() {
            sdio.write_cmd(&[100, 100], 135);
        }
        info!("Y: {}", unsafe { sdio.cmd_sm.get_y() },);
        info!("X: {:#010X}", unsafe { sdio.cmd_sm.get_x() });

        Timer::after_millis(10).await;
    }
}

pub struct PioSdioPrograms<'d, PIO: Instance> {
    clk: LoadedProgram<'d, PIO>,
    cmd: LoadedProgram<'d, PIO>,
}

impl<'d, PIO: Instance> PioSdioPrograms<'d, PIO> {
    pub fn new(pio: &mut Common<'d, PIO>) -> Self {
        let clk = pio_asm!(".side_set 1", "irq 0 side 0", "irq clear 0 side 1");

        // the lower half of the second word sent needs to contain the length of the response
        let cmd = pio_asm!(
            ".side_set 1 opt pindirs",
            "out y, 32", // Store 47 in y (for write loop)
            ".wrap_target",
            // Write CMD
            "mov x, y          side 1", // (re)set counter, set cmd as output
            "wait 0 irq 0",             // wait for clk low
            "lp_write:",
            "out pins, 1",
            "jmp x-- lp_write",
            // Read CMD
            "out x, 16        side 0", // move counter to x (47 or 135), set cmd as input
            "wait 0 irq 0",            // wait for clk low
            "lp_read:",
            "in pins, 1",
            "jmp x-- lp_read",
            "push", // push remaining 16 bits from isr
            ".wrap",
        );

        // let tx = pio_asm!();

        Self {
            clk: pio.load_program(&clk.program),
            cmd: pio.load_program(&cmd.program),
        }
    }
}

pub struct PioSdio<'d, PIO: Instance, const SM0: usize, const SM1: usize> {
    config: pio::Config<'d, PIO>,
    clk_sm: StateMachine<'d, PIO, SM0>,
    cmd_sm: StateMachine<'d, PIO, SM1>,
}

impl<'d, PIO: Instance, const SM0: usize, const SM1: usize> PioSdio<'d, PIO, SM0, SM1> {
    pub fn new(
        pio: &mut Common<'d, PIO>,
        mut clk_sm: StateMachine<'d, PIO, SM0>,
        mut cmd_sm: StateMachine<'d, PIO, SM1>,
        clk_pin: impl PioPin,
        cmd_pin: impl PioPin,
        d0_pin: impl PioPin,
        programs: PioSdioPrograms<'d, PIO>,
        dma: impl Channel,
    ) -> Self {
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
        cmd_sm.tx().push(47); // The counter for cmd write is 48
        cmd_sm.set_enable(true);

        Self {
            clk_sm,
            cmd_sm,
            config: cfg,
        }
    }

    /// Will change the pio frequency and restart the state machine and clear any fifos.
    /// Dont use while any communication is occurring
    pub fn set_freq(&mut self, freq: u32) {
        self.config.clock_divider = ((clk_sys_freq() / freq) as u16).into();
        self.clk_sm.restart();
        self.cmd_sm.restart();
        self.cmd_sm.tx().push(47);
    }

    fn write_cmd(&mut self, cmd: &[u32], response_len: u32) {
        assert!(cmd.len() == 2);

        // write 2 cmd words
        self.cmd_sm.tx().push(cmd[0]); // first 32 bits of cmd
        self.cmd_sm.tx().push(((cmd[1]) << 16) | response_len); // last 16 bits of cmd + response len
    }
}
