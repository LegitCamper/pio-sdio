#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use core::borrow::BorrowMut;
use defmt::info;
use embassy_executor::Spawner;
use embassy_rp::clocks::clk_sys_freq;
use embassy_rp::dma::Channel;
use embassy_rp::gpio::Output;
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
        mut sm3,
        irq0,
        irq1,
        ..
    } = Pio::new(p.PIO0, Irqs);

    let programs = PioSdPrograms::new(&mut common);
    let mut sd = PioSd::new_1_bit(
        &mut common,
        sm0,
        irq0,
        sm1,
        irq1,
        sm2,
        p.PIN_2,
        p.PIN_3,
        p.PIN_4,
        programs,
        p.DMA_CH0,
    );

    sm3.clear_fifos();
    sm3.set_config(&sd.cmd_rx);
    sm3.set_enable(true);

    loop {
        sd.cmd_sm.tx().push(62);
        sd.cmd_sm.tx().push(0xACAAAAAA);
        sd.cmd_sm.tx().push(0xACAAACAA);

        sd.cmd_tx_irq.wait().await;

        while !sm3.rx().empty() {}
        info!("RX: {:#010X}", sm3.rx().pull());
        info!("RX: {:#010X}", sm3.rx().pull());

        Timer::after_millis(100).await;
    }

    info!("Done!!!");

    loop {}
}
