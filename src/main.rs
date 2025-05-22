#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use defmt::{info, unwrap, warn};
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_time::Timer;
use embedded_sdmmc::Block;
use {defmt_rtt as _, panic_probe as _};

mod sd;
use sd::{Error, PioSd, PioSd1bit, PioSdClk};

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

    info!("Acquiring Card");
    'outer: loop {
        if sd.check_init().await.is_ok() {
            loop {
                match sd.enter_trans().await {
                    Ok(_) => break 'outer,
                    // Repeat init seq again
                    Err(_) => {
                        sd.reset();
                        warn!("Failed to enter transfer mode, Trying init again...");
                        break;
                    }
                }
            }
        } else {
            warn!("Failed to get card, trying again...");
            Timer::after_millis(200).await;
        }
    }

    let mut block = [Block::new()];
    unwrap!(sd.read_data(&mut block, embedded_sdmmc::BlockIdx(0)).await);
    info!("MBR: {:X}", block[0].contents);
    let mut block = [
        Block::new(),
        Block::new(),
        Block::new(),
        Block::new(),
        Block::new(),
    ];
    unwrap!(sd.read_data(&mut block, embedded_sdmmc::BlockIdx(1)).await);

    info!("Done!");
}
