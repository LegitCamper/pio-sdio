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
use sd::{PioSd, PioSdClk, PioSdCmdData};

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
        irq0,
        ..
    } = Pio::new(p.PIO0, Irqs);

    let cmd_data_prg = PioSdCmdData::new(&mut common);
    let clk_prg = PioSdClk::new(&mut common);
    let mut sd = PioSd::new(
        p.PIN_2,
        p.PIN_3,
        p.PIN_4,
        clk_prg,
        cmd_data_prg,
        &mut common,
        irq0,
        sm0,
        sm1,
        sm2,
        p.DMA_CH0,
    );

    sd.acquire().await.unwrap();

    info!("Done!");

    loop {}
}
