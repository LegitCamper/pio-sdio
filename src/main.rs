#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use core::borrow::BorrowMut;
use defmt::info;
use embassy_executor::Spawner;
use embassy_rp::clocks::clk_sys_freq;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::program::pio_asm;
use embassy_rp::pio::{
    self, Direction, FifoJoin, InterruptHandler, Pio, ShiftConfig, ShiftDirection, StateMachine,
};
use embassy_rp::{Peripheral, bind_interrupts};
use {defmt_rtt as _, panic_probe as _};

mod sdio;
use sdio::{PioSdio, PioSdioPrograms};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
    // PIO0_IRQ_0 => InterruptHandler<PIO0>;
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

    // let programs = PioSdioPrograms::new(&mut common);
    // let mut sdio = PioSdio::new_1_bit(
    //     &mut common,
    //     sm0,
    //     sm1,
    //     sm2,
    //     sm3,
    //     p.PIN_2,
    //     p.PIN_3,
    //     p.PIN_4,
    //     programs,
    //     p.DMA_CH0,
    // );

    // test tx/rx
    let mut clk_sm = sm0;
    let mut tx_sm = sm1;
    let mut rx_sm = sm2;
    let (mut clk, mut tx, mut rx) = {
        let clk = pio_asm!(
            ".side_set 1",
            "irq 0 side 1",
            "irq clear 0 side 0",
            options(max_program_size = 2)
        );
        let clk = common.load_program(&clk.program);

        // the first word pushed before any data must be the length of the data
        // when not writing, pin direction is set as input and the write flag is cleared allowing reads to occur
        let oneb_tx = pio_asm!(
            // ".side_set 1 opt pindirs",
            // "irq clear 1    side 0", // clear write, set d0 as input
            "out y, 32", // set y to counter
            "pull",
            // "irq 1          side 1", // raise write, set d0 as output
            "wait 0 irq 0",
            "lp:", // write word from osr
            "out pins, 1",
            "jmp y-- lp",
            options(max_program_size = 7)
        );
        let oneb_tx = common.load_program(&oneb_tx.program);

        // before data can be read, the length of data needs to be set
        // pin direction and read/write irqs are managed by oneb_tx
        let oneb_rx = pio_asm!(
            // "wait 0 irq 1", f/ wait for writing flag clear
            "wait 0 irq 0",
            "lp:", // read word into isr
            "in pins, 1",
            "jmp x-- lp",
            "push",
            options(max_program_size = 5)
        );
        let oneb_rx = common.load_program(&oneb_rx.program);

        let div = (clk_sys_freq() / 400_000) as u16;

        // Clk program config
        let clk_pin = common.make_pio_pin(p.PIN_1);
        let mut cfg = pio::Config::default();
        cfg.use_program(&clk, &[&clk_pin]);
        cfg.clock_divider = div.into();
        clk_sm.set_config(&cfg);
        clk_sm.set_pin_dirs(Direction::Out, &[&clk_pin]);
        clk_sm.set_enable(true);

        // Tx program config
        let tx_pin = common.make_pio_pin(p.PIN_2);
        let mut cfg = pio::Config::default();
        cfg.use_program(&oneb_tx, &[]);
        cfg.set_out_pins(&[&tx_pin]);
        cfg.clock_divider = div.into();
        cfg.fifo_join = FifoJoin::TxOnly;

        let mut shift_cfg = ShiftConfig::default();
        shift_cfg.threshold = 32;
        shift_cfg.direction = ShiftDirection::Left;
        shift_cfg.auto_fill = true;
        cfg.shift_out = shift_cfg;

        tx_sm.set_config(&cfg);
        tx_sm.clear_fifos();
        tx_sm.set_enable(true);

        // Rx program config
        let rx_pin = common.make_pio_pin(p.PIN_3);
        let mut cfg = pio::Config::default();
        cfg.use_program(&oneb_rx, &[]);
        cfg.set_in_pins(&[&rx_pin]);
        cfg.clock_divider = div.into();
        cfg.fifo_join = FifoJoin::RxOnly;
        rx_sm.set_config(&cfg);
        rx_sm.clear_fifos();
        rx_sm.set_enable(true);

        (clk_sm, tx_sm, rx_sm)
    };

    unsafe {
        rx.set_x(62);
    };
    info!("Rx X: {}", unsafe { rx.get_x() });

    tx.tx().push(64); // num bits
    tx.tx().push(1000);
    tx.tx().push(1005);
    info!("Pushed Commands, awaiting transfer");

    info!("Data read: {}", rx.rx().pull());
    info!("Data read: {}", rx.rx().pull());

    info!("Done!!!");

    loop {}
}
