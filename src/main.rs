#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::clocks::clk_sys_freq;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_time::Timer;
use embedded_sdmmc::Block;
use {defmt_rtt as _, panic_probe as _};

mod sd;
use sd::PioSd;
mod sdio;
use sdio::{PioSdio, PioSdio1bit, PioSdioClk};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Running");
    let p = embassy_rp::init(Default::default());
    info!("Sys CLK: {}Hz", clk_sys_freq());
    let Pio {
        mut common,
        sm0,
        sm1,
        sm2,
        irq0,
        ..
    } = Pio::new(p.PIO0, Irqs);

    let clk_prg = PioSdioClk::new(&mut common);
    let one_bit_prg = PioSdio1bit::new(&mut common);
    let sdio = PioSdio::new_1_bit(
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
    let mut sd = PioSd::new(sdio);

    info!("Acquiring Card");
    'outer: loop {
        match sd.check_init().await {
            Ok(_) => {
                loop {
                    if let Ok(_csd) = sd.read_csd().await {
                        break;
                    }
                }

                loop {
                    match sd.enter_trans().await {
                        Ok(_) => break 'outer,
                        // Repeat init seq again
                        Err(_) => {
                            warn!("Failed to enter transfer mode, Trying init again...");
                            break;
                        }
                    }
                }
            }
            Err(sd::Error::BadState) => panic!("Bad state"),
            Err(_) => {
                warn!("Failed to Get Card... Trying again");
                Timer::after_millis(500).await;
            }
        }
    }

    info!("Done!");
}
