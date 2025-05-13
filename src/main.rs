#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use defmt::{info, unwrap, warn};
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

mod sd;
use sd::{PioSd, PioSd1bit, PioSdClk};

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

    let clk_prg = PioSdClk::new(&mut common);
    let one_bit_prg = PioSd1bit::new(&mut common);
    let mut sd = PioSd::new(
        p.PIN_2,
        p.PIN_3,
        p.PIN_4,
        clk_prg,
        one_bit_prg,
        &mut common,
        irq0,
        sm0,
        sm1,
        sm2,
        p.DMA_CH0,
    );

    loop {
        loop {
            if sd.check_init().await.is_ok() {
                break;
            }
            warn!("Failed to acquire sd card, trying again...");
            Timer::after_millis(500).await;
        }
    }

    unwrap!(sd.set_frequency(16_000_000).await);
    // unwrap!(sd.read_csd().await);

    info!("Done!");
}
