#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use defmt::{info, unwrap, warn};
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::clocks::RoscRng;
use embassy_rp::clocks::clk_sys_freq;
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
        mut sm0,
        ..
    } = Pio::new(p.PIO0, Irqs);

    let clk = p.PIN_2;
    let cmd = p.PIN_3;
    let d0 = p.PIN_4;

    let programs = PioSdioPrograms::new(&mut common);
    let sdio = PioSdio::new(&mut common, sm0, clk, cmd, d0, programs);

    info!("Done!!!");

    loop {}
}

pub struct PioSdioPrograms<'d, PIO: Instance> {
    prg: LoadedProgram<'d, PIO>,
}

impl<'d, PIO: Instance> PioSdioPrograms<'d, PIO> {
    pub fn new(pio: &mut Common<'d, PIO>) -> Self {
        // side_set manages clk cycles
        // x is counter for 48 bit bit cmds
        let clk_cmd = pio_asm!(
            ".side_set 1 opt",
            "pull",
            "mov y, osr", // get 48 loop counter from tx
            ".wrap_target",
            "set pindirs 1     side 0", // make cmd output
            "mov x, y          side 1",
            "loop48:",
            "out pins, 1       side 0",
            "jmp x-- loop48    side 1",
            "set pindirs 0     side 0" // make cmd input
            "mov x, y          side 1",
            "loop48:",
            "in pins, 1        side 0",
            "jmp x-- loop48    side 1",
            ".wrap"
        );

        Self {
            prg: pio.load_program(&clk_cmd.program),
        }
    }
}

pub enum BusWidth {
    OneBit,
    FourBit,
}

pub struct PioSdio<'d, PIO: Instance, const SM: usize> {
    // config: pio::Config<'d, PIO>,
    sm: StateMachine<'d, PIO, SM>,
}

impl<'d, PIO: Instance, const SM: usize> PioSdio<'d, PIO, SM> {
    pub fn new(
        pio: &mut Common<'d, PIO>,
        mut sm: StateMachine<'d, PIO, SM>,
        clk_pin: impl PioPin,
        cmd_pin: impl PioPin,
        d0_pin: impl PioPin,
        programs: PioSdioPrograms<'d, PIO>,
    ) -> Self {
        let clk = pio.make_pio_pin(clk_pin);
        let cmd = pio.make_pio_pin(cmd_pin);
        let d0 = pio.make_pio_pin(d0_pin);

        let mut cfg = pio::Config::default();
        cfg.use_program(&programs.prg, &[&clk]);
        cfg.set_out_pins(&[&clk, &cmd, &d0]);
        cfg.set_in_pins(&[&cmd, &d0]);
        cfg.clock_divider = ((clk_sys_freq() / 400_000) as u16).into();

        // let mut shift_cfg = ShiftConfig::default();
        // shift_cfg.threshold = 32;
        // shift_cfg.direction = ShiftDirection::Left;
        // shift_cfg.auto_fill = true;
        // cfg.shift_in = shift_cfg;
        // cfg.shift_out = shift_cfg;

        sm.set_config(&cfg);
        sm.set_pin_dirs(Direction::Out, &[&clk]);
        sm.set_enable(true);
        sm.tx().push(48); // push 48 bit clk counter to tx

        Self { sm }
    }

    // /// Will change the pio frequency and restart the state machine and clear any fifos.
    // /// Dont use while any communication is occurring
    // pub fn set_freq(&mut self, freq: u32) {
    //     self.config.clock_divider = ((clk_sys_freq() / freq) as u16).into();
    //     self.sm.restart();
    // }

    fn write_byte(&mut self, byte: u8) {
        while self.sm.tx().full() {}
        self.sm.tx().push((byte as u32) << 24);
        // info!("Write {:x}", byte);
    }

    fn read_byte(&mut self) -> u8 {
        while self.sm.rx().empty() {}
        let read = self.sm.rx().pull();
        let byte = (read & 0xFF) as u8;
        // info!("Read byte: {:02X}", byte);
        byte
    }
}

// impl<'a, PIO: Instance, const SM: usize> SdioBus<u8> for PioSdio<'a, PIO, SM> {
//     fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
//         for word in words.iter_mut() {
//             self.write_byte(0xFF);
//             *word = self.read_byte() as u8;
//         }
//         Ok(())
//     }

//     fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
//         for &byte in words {
//             self.write_byte(byte);
//         }

//         self.flush()
//     }

//     fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
//         for (r, &w) in read.iter_mut().zip(write.iter()) {
//             self.write_byte(w);
//             *r = self.read_byte();
//         }

//         self.flush()
//     }

//     fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
//         for word in words.iter_mut() {
//             self.write_byte(*word);
//             *word = self.read_byte();
//         }

//         self.flush()
//     }

//     fn flush(&mut self) -> Result<(), Self::Error> {
//         while !self.sm.tx().empty() {}
//         Ok(())
//     }
// }
