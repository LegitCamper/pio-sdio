use defmt::{info, trace, warn};
use embassy_rp::clocks::clk_sys_freq;
use embassy_rp::dma::Channel;
use embassy_rp::gpio::Pull;
use embassy_rp::pio::program::{Instruction, SideSet, pio_asm};
use embassy_rp::pio::{
    self, Common, Direction, Instance, LoadedProgram, PioPin, ShiftConfig, ShiftDirection,
    StateMachine,
};
use embassy_rp::{PeripheralRef, into_ref};
use embassy_time::{Duration, Timer, with_timeout};
use embedded_sdmmc::sdcard::proto::*;
use embedded_sdmmc::sdcard::{CardType, Error};

const INIT_CLK: u32 = 100_000;

const SHORT_CMD_RES: u8 = 48;
const LONG_CMD_RES: u8 = 136;

pub struct PioSd<
    'd,
    PIO: Instance,
    C: Channel,
    const SM0: usize,
    const SM1: usize,
    const SM2: usize,
> {
    pub inner: PioSdInner<'d, PIO, C, SM0, SM1, SM2>,
    card_type: Option<CardType>,
}

impl<'d, PIO: Instance, C: Channel, const SM0: usize, const SM1: usize, const SM2: usize>
    PioSd<'d, PIO, C, SM0, SM1, SM2>
{
    pub fn new(
        clk_pin: impl PioPin,
        cmd_pin: impl PioPin,
        d0_pin: impl PioPin,
        clk_prg: PioSdClk<'d, PIO>,
        one_bit_prog: PioSd1bit<'d, PIO>,
        pio: &mut Common<'d, PIO>,
        clk_irq: pio::Irq<'d, PIO, 0>,
        clk_sm: StateMachine<'d, PIO, SM0>,
        cmd_sm: StateMachine<'d, PIO, SM1>,
        data_sm: StateMachine<'d, PIO, SM2>,
        dma: C,
    ) -> Self {
        Self {
            card_type: None,
            inner: PioSdInner::new_1_bit(
                clk_pin,
                cmd_pin,
                d0_pin,
                clk_prg,
                one_bit_prog,
                pio,
                clk_irq,
                clk_sm,
                cmd_sm,
                data_sm,
                dma,
            ),
        }
    }

    /// Change frequency - Initializes Card if not already
    pub async fn set_frequency(&mut self, freq: u32) -> Result<(), Error> {
        self.check_init().await?;

        // TODO: check if requested freq is compatible with card type

        self.inner.set_frequency(freq).map_err(|_| Error::Transport)
    }

    /// perform a command
    async fn card_command(
        &mut self,
        command: u8,
        arg: u32,
        read_buf: &mut [u8],
        read_len: u8,
    ) -> Result<(), Error> {
        let mut buf = [
            0x40 | command,
            (arg >> 24) as u8,
            (arg >> 16) as u8,
            (arg >> 8) as u8,
            arg as u8,
            0,
        ];
        buf[5] = crc7(&buf[0..5]);

        let cmd = self.inner.buf_to_cmd(&buf);
        self.inner.write_command(cmd.0, cmd.1).await;
        with_timeout(Duration::from_millis(2), async {
            while !self.inner.cmd_ready() {}
        })
        .await
        .map_err(|_| {
            self.inner.reset_command();
            warn!("Pio failed to finish write");
            Error::Transport
        })?;
        info!("Tx: {:#04X}", buf);

        if read_len == 0 || read_buf.is_empty() {
            return Ok(());
        }

        assert!(read_buf.len() as u8 >= read_len.div_ceil(8));
        read_buf.iter_mut().for_each(|i| *i = 0);

        // Ensure pio completed rx
        let timeout = Duration::from_millis(match command {
            // CMD8 | CMD55 => 50,
            // ACMD41 => 1000,
            // CMD9 => 10,
            // CMD0 => 1,
            _ => 25,
        });
        with_timeout(timeout, async {
            self.inner.read_command(read_buf, read_len).await;
            while !self.inner.cmd_ready() {}
        })
        .await
        .map_err(|_| {
            self.inner.reset_command();
            warn!("Pio failed to finish read");
            Error::TimeoutCommand(command)
        })?;
        info!("RX: {:#04X}", read_buf);

        self.inner.reset_command(); // take ownership of cmd

        Ok(())
    }

    /// Check the card is initialised.
    pub async fn check_init(&mut self) -> Result<(), Error> {
        if self.card_type.is_none() {
            // If we don't know what the card type is, try and initialise the
            // card. This will tell us what type of card it is.
            self.acquire().await
        } else {
            Ok(())
        }
    }

    async fn acquire(&mut self) -> Result<(), Error> {
        let mut buf = [0xFF; SHORT_CMD_RES as usize / 8];
        self.inner.reset_command();
        self.inner.reset_data();

        // Wait initial 74+ clocks high
        self.inner.cmd_sm.set_config(&self.inner.cmd_write_cfg);
        Timer::after(Duration::from_hz(INIT_CLK as u64) * 80).await;

        trace!("Reset card..");
        self.card_command(CMD0, 0, &mut [], 0).await?;

        let mut card_type = CardType::SD1;
        let mut arg = 0;

        for _ in 0..10 {
            if self
                .card_command(CMD8, 0x1AA, &mut buf, SHORT_CMD_RES)
                .await
                .is_ok()
            {
                if buf[0] == 0x08 && buf[3] == 0x01 && buf[4] == 0xAA {
                    card_type = CardType::SD2;
                    arg = 0x40FF_8000;
                    info!("READ: {:X}", buf);
                    break;
                }
            }
            Timer::after_millis(50).await;

            warn!("ERROR!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        }

        // with_timeout(Duration::from_secs(1), async {
        //     loop {
        //         info!("Sending app");
        //         if self
        //             .card_command(CMD55, 0, &mut buf, SHORT_CMD_RES)
        //             .await
        //             .is_err()
        //         {
        //             Timer::after_millis(5).await;
        //             continue;
        //         } else {
        //             info!("CMD55 RESP: {:X}", buf);
        //         }

        //         info!("Sending init");

        //         if self
        //             .card_command(ACMD41, arg, &mut buf, SHORT_CMD_RES)
        //             .await
        //             .is_ok()
        //             && buf[0] == 0x3F
        //             && buf[4] == 0x0
        //             && buf[5] == 0xFF
        //         {
        //             if buf[1] & 0x40 == 0x40 {
        //                 info!("Card is SDHC or SDXC!");
        //                 card_type = CardType::SDHC;
        //             }
        //             if buf[1] & 0x80 == 0x80 {
        //                 break;
        //             };
        //             info!("Card is still initializing");
        //         }

        //         Timer::after_millis(5).await
        //     }
        //     info!("Broke timeout");
        // })
        // .await
        // .map_err(|_| Error::CardNotFound)?;

        info!("Card Type: {}", card_type);
        self.card_type = Some(card_type);

        info!("Success! Card should be initialized");

        Ok(())
    }

    /// Read the CSD data
    pub async fn read_csd(&mut self) -> Result<(), Error> {
        let mut buf = [0; LONG_CMD_RES as usize / 8];
        self.card_command(CMD9, 0, &mut buf, LONG_CMD_RES).await?;
        Ok(())
    }
}

pub struct PioSdClk<'d, PIO: Instance> {
    clk: LoadedProgram<'d, PIO>,
}

impl<'d, PIO: Instance> PioSdClk<'d, PIO> {
    pub fn new(common: &mut Common<'d, PIO>) -> Self {
        let clk_prg = pio_asm!(".side_set 1", "irq 0 side 1", "irq clear 0 side 0",);
        let clk = common.load_program(&clk_prg.program);
        Self { clk }
    }
}

pub struct PioSd1bit<'d, PIO: Instance> {
    write: LoadedProgram<'d, PIO>,
    read: LoadedProgram<'d, PIO>,
}

impl<'d, PIO: Instance> PioSd1bit<'d, PIO> {
    /// Creates a new 1-bit Tx/Rx program
    pub fn new(common: &mut Common<'d, PIO>) -> Self {
        let write = pio_asm!(
            // make sure pins are hi when we set output dir ( avoid pin glitch/accidental start bit )
            "set pins, 1",
            "set pindirs, 1",
            "out x, 16",
            "wait 0 irq 0", // wait for clk
            "write_loop:",
            "out pins, 1",
            "jmp x-- write_loop",
        );

        let read = pio_asm!(
            "out x, 16",
            "in null, 1", // preload start bit that is skip on `wait 0 pin, 0`
            "set pindirs, 0",
            "wait 1 pin, 0", // wait until card takes ownership
            "wait 0 pin, 0",
            "wait 0 irq 0", // wait for clk
            "read_loop:",
            "in pins, 1",
            "jmp x-- read_loop",
            "out exec, 16", // flush remaining bits in isr
        );

        Self {
            write: common.load_program(&write.program),
            read: common.load_program(&read.program),
        }
    }
}

pub struct PioSdInner<
    'd,
    PIO: Instance,
    C: Channel,
    const SM0: usize,
    const SM1: usize,
    const SM2: usize,
> {
    dma: PeripheralRef<'d, C>,
    _clk_irq: pio::Irq<'d, PIO, 0>,
    clk_cfg: pio::Config<'d, PIO>,
    clk_sm: StateMachine<'d, PIO, SM0>,
    cmd_write_cfg: pio::Config<'d, PIO>,
    cmd_read_cfg: pio::Config<'d, PIO>,
    cmd_sm: StateMachine<'d, PIO, SM1>,
    data_write_cfg: pio::Config<'d, PIO>,
    data_read_cfg: pio::Config<'d, PIO>,
    data_sm: StateMachine<'d, PIO, SM2>,
}

impl<'d, PIO: Instance, C: Channel, const SM0: usize, const SM1: usize, const SM2: usize>
    PioSdInner<'d, PIO, C, SM0, SM1, SM2>
{
    pub fn new_1_bit(
        clk_pin: impl PioPin,
        cmd_pin: impl PioPin,
        data_pin: impl PioPin,
        clk_prg: PioSdClk<'d, PIO>,
        one_bit_prg: PioSd1bit<'d, PIO>,
        pio: &mut Common<'d, PIO>,
        clk_irq: pio::Irq<'d, PIO, 0>,
        mut clk_sm: StateMachine<'d, PIO, SM0>,
        mut cmd_sm: StateMachine<'d, PIO, SM1>,
        mut data_sm: StateMachine<'d, PIO, SM2>,
        dma: C,
    ) -> Self {
        into_ref!(dma);
        let div = (clk_sys_freq() / INIT_CLK) as u16;

        clk_sm.clear_fifos();
        cmd_sm.clear_fifos();
        data_sm.clear_fifos();

        let mut clk = pio.make_pio_pin(clk_pin);
        clk.set_pull(Pull::Down);
        let mut cmd = pio.make_pio_pin(cmd_pin);
        cmd.set_pull(Pull::Up);
        let mut data = pio.make_pio_pin(data_pin);
        data.set_pull(Pull::Up);

        // Clk program config
        let mut clk_cfg = pio::Config::default();
        clk_cfg.use_program(&clk_prg.clk, &[&clk]);
        clk_cfg.clock_divider = div.into();

        let shift_cfg = ShiftConfig {
            threshold: 32,
            direction: ShiftDirection::Left,
            auto_fill: true,
        };

        // Cmd config
        let mut cmd_write_cfg = pio::Config::default();
        cmd_write_cfg.use_program(&one_bit_prg.write, &[]);
        cmd_write_cfg.set_set_pins(&[&cmd]);
        cmd_write_cfg.set_out_pins(&[&cmd]);
        cmd_write_cfg.clock_divider = div.into();
        cmd_write_cfg.shift_out = shift_cfg;

        let mut cmd_read_cfg = pio::Config::default();
        cmd_read_cfg.use_program(&one_bit_prg.read, &[]);
        cmd_read_cfg.set_set_pins(&[&cmd]);
        cmd_read_cfg.set_in_pins(&[&cmd]);
        cmd_read_cfg.clock_divider = div.into();
        cmd_read_cfg.shift_out = shift_cfg;
        cmd_read_cfg.shift_in = shift_cfg;

        // data config
        let mut data_write_cfg = pio::Config::default();
        data_write_cfg.use_program(&one_bit_prg.write, &[]);
        data_write_cfg.set_set_pins(&[&data]);
        data_write_cfg.set_out_pins(&[&data]);
        data_write_cfg.clock_divider = div.into();
        data_write_cfg.shift_out = shift_cfg;

        let mut data_read_cfg = pio::Config::default();
        data_read_cfg.use_program(&one_bit_prg.read, &[]);
        data_read_cfg.set_set_pins(&[&data]);
        data_read_cfg.set_in_pins(&[&data]);
        data_read_cfg.clock_divider = div.into();
        data_read_cfg.shift_out = shift_cfg;
        data_read_cfg.shift_in = shift_cfg;

        clk_sm.set_config(&clk_cfg);
        clk_sm.set_pin_dirs(Direction::Out, &[&clk]);
        clk_sm.set_enable(true);

        // start write programs so pins are held high
        cmd_sm.set_config(&cmd_write_cfg);
        cmd_sm.set_enable(true);
        data_sm.set_config(&data_write_cfg);
        data_sm.set_enable(true);

        Self {
            dma,
            _clk_irq: clk_irq,
            clk_cfg,
            clk_sm,
            cmd_write_cfg,
            cmd_read_cfg,
            cmd_sm,
            data_write_cfg,
            data_read_cfg,
            data_sm,
        }
    }

    fn set_frequency(&mut self, freq: u32) -> Result<(), ()> {
        // ensure pio is stalled and not busy
        if self.cmd_ready() && self.data_ready() {
            let div = (clk_sys_freq() / freq) as u16;

            self.clk_sm.set_enable(false);
            self.cmd_sm.set_enable(false);
            self.data_sm.set_enable(false);

            self.clk_cfg.clock_divider = div.into();
            self.cmd_write_cfg.clock_divider = div.into();
            self.cmd_read_cfg.clock_divider = div.into();
            self.data_write_cfg.clock_divider = div.into();
            self.data_read_cfg.clock_divider = div.into();

            self.clk_sm.set_enable(true);
            self.cmd_sm.set_enable(true);
            self.data_sm.set_enable(true);

            return Ok(());
        }
        Err(())
    }

    fn buf_to_cmd(&self, buf: &[u8]) -> (u32, u16) {
        assert!(buf.len() == 6);
        (
            (buf[0] as u32) << 24 | (buf[1] as u32) << 16 | (buf[2] as u32) << 8 | buf[3] as u32,
            (buf[4] as u16) << 8 | buf[5] as u16,
        )
    }

    fn reset_command(&mut self) {
        self.cmd_sm.clear_fifos();
        self.cmd_sm.restart();
        self.cmd_sm.set_config(&self.cmd_write_cfg);
    }

    fn reset_data(&mut self) {
        self.data_sm.clear_fifos();
        self.data_sm.restart();
        self.data_sm.set_config(&self.cmd_write_cfg);
    }

    fn cmd_ready(&mut self) -> bool {
        self.cmd_sm.tx().stalled() && self.cmd_sm.tx().empty() && self.cmd_sm.rx().empty()
    }

    fn data_ready(&mut self) -> bool {
        self.data_sm.tx().stalled() && self.data_sm.tx().empty() && self.data_sm.rx().empty()
    }

    /// writes cmd to sm
    async fn write_command(&mut self, upper: u32, lower: u16) {
        self.cmd_sm.clear_fifos();
        self.cmd_sm.set_enable(false);

        // packs command into two words with loop counter
        // commands are always 48 bits minus loop
        self.cmd_sm.tx().push((48_u32 - 1) << 16 | upper >> 16);
        self.cmd_sm.tx().push(upper << 16 | lower as u32);

        self.cmd_sm.set_enable(true);
    }

    // uses dma to fill buf with read words
    pub async fn read_command(&mut self, buf: &mut [u8], bit_len: u8) {
        self.cmd_sm.clear_fifos();
        self.cmd_sm.set_config(&self.cmd_read_cfg);

        // creates an exec instruction to flush bits left in isr
        let null_instr = Instruction {
            operands: pio::program::InstructionOperands::IN {
                source: pio::program::InSource::NULL,
                bit_count: 32 - (bit_len % 32),
            },
            delay: 0,
            side_set: None,
        };

        self.cmd_sm
            .tx()
            .push((bit_len as u32 - 2) << 16 | null_instr.encode(SideSet::default()) as u32);

        for chunk in buf.chunks_mut(4) {
            let resp = self.cmd_sm.rx().wait_pull().await;

            let bytes = resp.to_be_bytes();
            for (i, b) in chunk.iter_mut().enumerate() {
                *b = bytes[i];
            }
        }
    }
}
