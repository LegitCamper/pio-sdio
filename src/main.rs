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
use embassy_rp::{Peripheral, bind_interrupts, into_ref};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

mod sd;
use sd::{PioSd, PioSdPrograms};

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

    // let programs = PioSdPrograms::new(&mut common);
    // let mut sd = PioSd::new_1_bit(
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
    let (_clk, mut tx, mut rx) = {
        let clk = pio_asm!(
            ".side_set 1",
            "irq 0 side 1",       // rising edge
            "irq clear 0 side 0", // falling edge
            options(max_program_size = 2)
        );
        let clk = common.load_program(&clk.program);

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
        let oneb_tx = common.load_program(&oneb_tx.program);

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
        let oneb_rx = common.load_program(&oneb_rx.program);

        let div = (clk_sys_freq() / 400_000) as u16;

        // Clk program config
        let clk_pin = common.make_pio_pin(p.PIN_2);
        let mut cfg = pio::Config::default();
        cfg.use_program(&clk, &[&clk_pin]);
        cfg.clock_divider = div.into();
        clk_sm.set_config(&cfg);
        clk_sm.set_pin_dirs(Direction::Out, &[&clk_pin]);
        clk_sm.set_enable(true);

        // Tx program config
        let tx_pin = common.make_pio_pin(p.PIN_3);
        let mut cfg = pio::Config::default();
        cfg.use_program(&oneb_tx, &[]);
        cfg.set_out_pins(&[&tx_pin]);
        cfg.set_set_pins(&[&tx_pin]);
        cfg.clock_divider = div.into();
        cfg.fifo_join = FifoJoin::TxOnly;
        let mut shift_cfg = ShiftConfig::default();
        shift_cfg.threshold = 32;
        shift_cfg.direction = ShiftDirection::Left;
        shift_cfg.auto_fill = true;
        cfg.shift_out = shift_cfg;
        tx_sm.set_config(&cfg);
        tx_sm.set_pin_dirs(Direction::Out, &[&tx_pin]);
        tx_sm.clear_fifos();
        tx_sm.set_enable(true);

        // Rx program config
        let rx_pin = common.make_pio_pin(p.PIN_4);
        let mut cfg = pio::Config::default();
        cfg.use_program(&oneb_rx, &[]);
        cfg.set_in_pins(&[&rx_pin]);
        cfg.set_set_pins(&[&rx_pin]);
        cfg.clock_divider = div.into();
        cfg.fifo_join = FifoJoin::RxOnly;
        let mut shift_cfg = ShiftConfig::default();
        shift_cfg.threshold = 32;
        shift_cfg.direction = ShiftDirection::Left;
        shift_cfg.auto_fill = true;
        cfg.shift_in = shift_cfg;
        rx_sm.set_config(&cfg);
        rx_sm.set_pin_dirs(Direction::In, &[&rx_pin]);
        rx_sm.clear_fifos();
        rx_sm.set_enable(false);

        (clk_sm, tx_sm, rx_sm)
    };

    // let mut test_write: [u32; 7] = [10, 100, 1000, 10_000, 97, 88, 1001];
    // let mut test_read = [0_u32; 1];

    // tx.tx().push(test_write.len() as u32 * 31); // push len
    // let dma_tx = tx.tx().dma_push(p.DMA_CH0.into_ref(), &test_write, false);

    // let dma_rx = rx
    //     .rx()
    //     .dma_pull(p.DMA_CH1.into_ref(), &mut test_read, false);

    // info!("Awaiting Data transfer");
    // dma_tx.await;
    // // dma_rx.await;
    // info!("RX: {}", rx.rx().pull());

    // for w in test_write {
    //     tx.tx().push(w);
    // }

    // for w in test_read.iter_mut() {
    //     *w = rx.rx().pull();
    // }

    // info!("RX: {}", test_read);

    loop {
        rx.set_enable(true);
        rx.clear_fifos();
        rx.restart();

        tx.tx().push(0xAAAAAAAA);
        info!("Rx : {:#010X}", rx.rx().pull());

        rx.set_enable(false);
        rx.clear_fifos();

        Timer::after_millis(10).await;
        assert!(tx.tx().empty());
        assert!(rx.rx().empty());
    }

    // info!("Done!!!");

    loop {}
}
