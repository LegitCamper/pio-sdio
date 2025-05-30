use defmt::{Format, info, trace, warn};
use embassy_rp::clocks::clk_sys_freq;
use embassy_rp::dma::Channel;
use embassy_rp::gpio::{Drive, Level, Pull, SlewRate};
use embassy_rp::pio::program::{Instruction, SideSet, pio_asm};
use embassy_rp::pio::{
    self, Common, Direction, Instance, LoadedProgram, PioPin, ShiftConfig, ShiftDirection,
    StateMachine,
};
use embassy_rp::{Peripheral, PeripheralRef, into_ref};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use embedded_hal::digital::InputPin;
use embedded_sdmmc::sdcard::CardType;
use embedded_sdmmc::sdcard::proto::*;
use embedded_sdmmc::{Block, BlockIdx};
// use embedded_sdmmc::{SdCard, TimeSource, Timestamp, VolumeIdx, VolumeManager};

const INIT_CLK: u32 = 100_000;

const SHORT_CMD_RESP: u8 = 48;
const LONG_CMD_RESP: u8 = 136;

/// The possible errors this crate can generate.
#[derive(Debug, Copy, Clone, Format)]
pub enum Error {
    /// We got an error from the Pio state machine
    Transport,
    /// We didn't get a response when reading data from the card
    TimeoutReadBuffer,
    /// We didn't get a response when waiting for the card to not be busy
    TimeoutWaitNotBusy,
    /// We didn't get a response when executing this command
    TimeoutCommand(u8),
    /// We got a bad response from Command 58
    Cmd58Error,
    /// We failed to read the Card Specific Data register
    RegisterReadError,
    /// We got a CRC mismatch (card gave us, we calculated)
    CrcError(u16, u16),
    /// Error reading from the card
    ReadError,
    /// Error writing to the card
    WriteError,
    /// Can't perform this operation with the card in this state
    BadState,
    /// Couldn't find the card
    CardNotFound,
}

pub struct PioSd<
    'd,
    PIO: Instance,
    C: Channel,
    const SM0: usize,
    const SM1: usize,
    const SM2: usize,
> {
    inner: PioSdInner<'d, PIO, C, SM0, SM1, SM2>,
    card_type: Option<CardType>,
    rca: Option<u16>,
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
            rca: None,
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

    /// perform a command
    async fn card_command(
        &mut self,
        command: u8,
        arg: u32,
        read_buf: &mut [u8],
    ) -> Result<(), Error> {
        assert!(
            read_buf.is_empty()
                || read_buf.len() == SHORT_CMD_RESP as usize / 8
                || read_buf.len() == LONG_CMD_RESP as usize / 8
        );
        self.inner.reset_command();

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
        self.inner.write_command(cmd.0, cmd.1)?;
        info!("TX 0x{:X}: {:#04X}", command, buf);

        if !read_buf.is_empty() {
            read_buf.iter_mut().for_each(|i| *i = 0);

            let timeout = Duration::from_millis(match command {
                ACMD41 => 1000,
                _ => 10,
            });
            self.inner
                .read_command(
                    read_buf,
                    match read_buf.len() {
                        6 => SHORT_CMD_RESP,
                        _ => LONG_CMD_RESP,
                    },
                    timeout,
                )
                .await
                .map_err(|_| Error::TimeoutCommand(command))?;

            info!("RX: {:#02X}", read_buf);
        }

        Ok(())
    }

    /// Reset the card but does not reinitialize
    pub fn reset(&mut self) {
        self.set_frequency(INIT_CLK);
        self.rca = None;
        self.card_type = None;
    }

    /// Check the card is initialised.
    pub async fn check_init(&mut self) -> Result<(), Error> {
        if self.card_type.is_none() {
            self.acquire().await
        } else {
            Ok(())
        }
    }

    async fn acquire(&mut self) -> Result<(), Error> {
        let mut long_buf = [0xFF; 17];
        let mut buf = [0xFF; 6];

        // Wait initial 74+ clocks high
        Timer::after(Duration::from_hz(INIT_CLK as u64) * 100).await;

        trace!("Reset card..");
        self.card_command(CMD0, 0, &mut []).await?;

        let mut card_type = None;
        let mut arg = 0;

        if self.card_command(CMD8, 0x1AA, &mut buf).await.is_ok() {
            // correct voltage echo
            if buf[3] == 0x01 && buf[4] == 0xAA {
                card_type = Some(CardType::SD2);
                arg = 1 << 30;
            // card says cmd8 is illegal
            } else if buf[0] == 0x05 {
                card_type = Some(CardType::SD1);
            }
        }

        if card_type.is_none() {
            return Err(Error::CardNotFound);
        }
        Timer::after_millis(5).await;

        const OCR: u32 = 0xFF0000; // support 2.7v - 3.6v
        const PERFORMANCE: u32 = 1 << 28;

        // // first send ocr inquiry
        // let mut got_inquiry = false;
        // for _ in 0..5 {
        //     self.acmd41(&mut buf, arg | PERFORMANCE | (OCR << 8))
        //         .await?;
        //     let busy = (buf[2] & 0x80) == 0;
        //     if !busy {
        //         if buf[1] & 0x40 == 0x40 {
        //             card_type = Some(CardType::SDHC)
        //         }
        //         got_inquiry = true;
        //     }
        //     Timer::after_millis(200).await
        // }

        // if !got_inquiry {
        //     return Err(Error::BadState);
        // }

        // then send init with ocr
        let mut init = false;
        for _ in 0..5 {
            self.acmd41(&mut buf, arg | PERFORMANCE | (OCR << 8) | (0x8 << 5))
                .await?;
            let busy = (buf[2] & 0x80) == 0;
            if !busy {
                init = true;
            }
            Timer::after_millis(200).await
        }

        if !init {
            return Err(Error::BadState);
        }

        Timer::after_millis(100).await;

        // Get CID
        for i in 0..5 {
            if self.card_command(0x02, 0, &mut long_buf).await.is_ok() {
                info!("CID: {:02X}", long_buf[1..]);
                break;
            };
            if i == 4 {
                return Err(Error::RegisterReadError);
            }
            Timer::after_millis(100).await
        }

        // Get RCA
        let mut rca = 0_u16;
        for i in 0..5 {
            if self.card_command(0x03, 0, &mut buf).await.is_ok() {
                rca = (buf[1] as u16) << 8 | buf[2] as u16;
                info!("RCA: {:X}", rca);
                break;
            }
            if i == 4 {
                return Err(Error::RegisterReadError);
            }
            Timer::after_millis(100).await
        }

        Timer::after_millis(1).await;

        info!("Card Type: {}", card_type);
        self.card_type = card_type;
        self.rca = Some(rca);

        Ok(())
    }

    async fn acmd41(&mut self, buf: &mut [u8], acmd_arg: u32) -> Result<(), Error> {
        // acd41 needs to be in idle status to be ran!
        for _ in 0..5 {
            if self.acmd(buf, acmd_arg, 0).await.is_ok() {
                return Ok(());
            }
        }
        Err(Error::BadState)
    }

    /// send an application specific command
    async fn acmd(
        &mut self,
        buf: &mut [u8],
        acmd_arg: u32,
        expected_status: u8,
    ) -> Result<(), Error> {
        for _ in 0..10 {
            if self.card_command(CMD55, 0, buf).await.is_ok() {
                let good_status = buf[3] & 0x1E == expected_status; // card status field
                let ready_for_app = (buf[4] & 0x20) != 0;
                info!("STATUS: {}, ready? {}", good_status, ready_for_app);
                if good_status && ready_for_app {
                    return self.card_command(ACMD41, acmd_arg, buf).await;
                }
            }
            Timer::after_millis(250).await;
        }
        Err(Error::ReadError)
    }

    /// Read the 'card specific data' block.
    pub async fn read_csd(&mut self) -> Result<Csd, Error> {
        let mut long_buf = [0xFF; 17];

        for _ in 0..5 {
            if self
                .card_command(CMD9, (self.rca.unwrap() as u32) << 16, &mut long_buf)
                .await
                .is_ok()
            {
                return match self.card_type {
                    Some(CardType::SD1) => {
                        let mut csd = CsdV1::new();
                        csd.data
                            .iter_mut()
                            .zip(long_buf[1..].iter())
                            .for_each(|(csd, resp)| *csd = *resp);
                        Ok(Csd::V1(csd))
                    }
                    Some(CardType::SD2 | CardType::SDHC) => {
                        let mut csd = CsdV2::new();
                        csd.data
                            .iter_mut()
                            .zip(long_buf[1..].iter())
                            .for_each(|(csd, resp)| *csd = *resp);
                        Ok(Csd::V2(csd))
                    }
                    None => Err(Error::CardNotFound),
                };
            }

            Timer::after_millis(100).await
        }

        Err(Error::RegisterReadError)
    }

    /// Change SD bus frequency
    pub fn set_frequency(&mut self, freq: u32) {
        self.inner.set_freq(freq);
    }

    pub async fn enter_trans(&mut self) -> Result<(), Error> {
        let mut buf = [0xFF; 6];

        // Enter TRANsfer mode
        self.card_command(0x07, (self.rca.unwrap() as u32) << 16, &mut buf)
            .await?;
        {
            if buf[3] == 0x07 {
                info!("Card is now in Trans state");
            }
        }
        Ok(())
    }

    pub async fn read_data(
        &mut self,
        blocks: &mut [Block],
        start_block_idx: BlockIdx,
    ) -> Result<(), Error> {
        let start_idx = match self.card_type {
            Some(CardType::SD1 | CardType::SD2) => start_block_idx.0 * 512,
            Some(CardType::SDHC) => start_block_idx.0,
            None => return Err(Error::CardNotFound),
        };

        if blocks.len() == 1 {
            // start single block trans
            self.card_command(CMD17, start_idx, &mut []).await?;
            // if cmd_resp[0]

            self.inner.read_data(&mut blocks[0]).await?;
        } else {
            // start multiblock trans
            self.card_command(CMD18, start_idx, &mut []).await?;
            // if cmd_resp[0]

            for block in blocks {
                self.inner.read_data(block).await?;
            }

            // stop multiblock trans
            self.card_command(CMD12, start_idx, &mut []).await?;
        }

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
            ".wrap_target",
            "out x, 16",
            "wait 0 irq 0", // wait for clk
            "write_loop:",
            "out pins, 1",
            "jmp x-- write_loop",
            ".wrap",
        );

        let read = pio_asm!(
            "out x, 16",
            "in null, 1", // preload start bit which is skipped below
            "set pindirs, 0",
            "wait 1 pin, 0", // wait until card takes ownership
            "wait 0 pin, 0", // start bit
            "wait 0 irq 0",  // wait for clk
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

struct PioSdInner<
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
    fn new_1_bit(
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
        let clkdiv = (clk_sys_freq() / INIT_CLK) as u16;
        info!("DIV: {}", clkdiv);

        clk_sm.clear_fifos();
        cmd_sm.clear_fifos();
        data_sm.clear_fifos();

        let mut clk = pio.make_pio_pin(clk_pin);
        clk.set_pull(Pull::Down);
        clk.set_slew_rate(SlewRate::Fast);
        clk.set_drive_strength(Drive::_8mA);
        let mut cmd = pio.make_pio_pin(cmd_pin);
        cmd.set_pull(Pull::Up);
        let mut data = pio.make_pio_pin(data_pin);
        data.set_pull(Pull::Up);

        // Clk program config
        let mut clk_cfg = pio::Config::default();
        clk_cfg.use_program(&clk_prg.clk, &[&clk]);
        clk_cfg.clock_divider = clkdiv.into();

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
        cmd_write_cfg.clock_divider = clkdiv.into();
        cmd_write_cfg.shift_out = shift_cfg;

        let mut cmd_read_cfg = pio::Config::default();
        cmd_read_cfg.use_program(&one_bit_prg.read, &[]);
        cmd_read_cfg.set_set_pins(&[&cmd]);
        cmd_read_cfg.set_in_pins(&[&cmd]);
        cmd_read_cfg.clock_divider = clkdiv.into();
        cmd_read_cfg.shift_out = shift_cfg;
        cmd_read_cfg.shift_in = shift_cfg;

        // data config
        let mut data_write_cfg = pio::Config::default();
        data_write_cfg.use_program(&one_bit_prg.write, &[]);
        data_write_cfg.set_set_pins(&[&data]);
        data_write_cfg.set_out_pins(&[&data]);
        data_write_cfg.clock_divider = clkdiv.into();
        data_write_cfg.shift_out = shift_cfg;

        let mut data_read_cfg = pio::Config::default();
        data_read_cfg.use_program(&one_bit_prg.read, &[]);
        data_read_cfg.set_set_pins(&[&data]);
        data_read_cfg.set_in_pins(&[&data]);
        data_read_cfg.clock_divider = clkdiv.into();
        data_read_cfg.shift_out = shift_cfg;
        data_read_cfg.shift_in = shift_cfg;

        clk_sm.set_pins(Level::Low, &[&clk]);
        clk_sm.set_pin_dirs(Direction::Out, &[&clk]);
        clk_sm.set_config(&clk_cfg);

        // the default program is the tx so the pins are held high
        cmd_sm.set_config(&cmd_write_cfg);
        data_sm.set_config(&data_write_cfg);

        // start write programs so pins are held high
        pio.apply_sm_batch(|pio| {
            pio.set_enable(&mut data_sm, true);
            pio.set_enable(&mut cmd_sm, true);
        });

        // wait for sm to set pins high before starting clk
        while !data_sm.tx().stalled() && !cmd_sm.tx().stalled() {}
        clk_sm.set_enable(true);

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

    fn set_freq(&mut self, freq: u32) {
        let clkdiv = (clk_sys_freq() / freq) as u16;

        self.clk_cfg.clock_divider = clkdiv.into();
        self.cmd_write_cfg.clock_divider = clkdiv.into();
        self.cmd_read_cfg.clock_divider = clkdiv.into();
        self.data_write_cfg.clock_divider = clkdiv.into();
        self.data_read_cfg.clock_divider = clkdiv.into();

        self.clk_sm.set_config(&self.clk_cfg);
        self.clk_sm.restart();
        self.reset_command();
        self.reset_data();
    }

    fn buf_to_cmd(&self, buf: &[u8]) -> (u32, u16) {
        assert!(buf.len() == 6);
        (
            (buf[0] as u32) << 24 | (buf[1] as u32) << 16 | (buf[2] as u32) << 8 | buf[3] as u32,
            (buf[4] as u16) << 8 | buf[5] as u16,
        )
    }

    fn reset_data(&mut self) {
        self.data_sm.clear_fifos();
        self.data_sm.restart();
        self.data_sm.set_config(&self.data_write_cfg);
    }

    /// writes data block to data pin(s)
    async fn write_data(&mut self, block: &Block) -> Result<(), Error> {
        self.data_sm.tx().push((Block::LEN_U32 - 1) << 16 | 0xFF); // keep line held high

        with_timeout(
            Duration::from_millis(100),
            self.data_sm
                .tx()
                .dma_push(self.dma.reborrow(), &block.contents, false),
        )
        .await
        .map_err(|_| {
            self.reset_command();
            Error::WriteError
        })
    }

    /// Read data block into provided block
    async fn read_data(&mut self, block: &mut Block) -> Result<(), Error> {
        self.data_sm.set_config(&self.data_read_cfg);

        let mut response = [0_u32; 16 + 2]; // 512 bytes
        let bits = Block::LEN_U32 + 16; // +16 for 2 byte crc

        // creates an exec instruction to flush bits left in isr
        let null_instr = Instruction {
            operands: pio::program::InstructionOperands::IN {
                source: pio::program::InSource::NULL,
                bit_count: 32 - (bits as u8 % 32),
            },
            delay: 0,
            side_set: None,
        };
        self.data_sm
            .tx()
            .push((bits - 2) << 16 | null_instr.encode(SideSet::default()) as u32);

        with_timeout(
            Duration::from_millis(100),
            self.data_sm
                .rx()
                .dma_pull(self.dma.reborrow(), &mut response, false),
        )
        .await
        .map_err(|_| {
            self.reset_command();
            Error::TimeoutReadBuffer
        })?;

        // take ownership of data again
        self.data_sm.set_config(&self.data_write_cfg);

        // break dma u32 resp into byte array
        for (chunk, word) in block.chunks_mut(4).zip(response.iter()) {
            let bytes = word.to_le().to_be_bytes();
            for (b, dst) in bytes.iter().zip(chunk.iter_mut()) {
                *dst = *b;
            }
        }

        // take ownership of dat again
        self.data_sm.set_config(&self.data_write_cfg);

        Ok(())
    }

    fn reset_command(&mut self) {
        self.cmd_sm.clear_fifos();
        self.cmd_sm.restart();
        self.cmd_sm.set_config(&self.cmd_write_cfg);
    }

    /// writes cmd to sm
    fn write_command(&mut self, upper: u32, lower: u16) -> Result<(), Error> {
        self.cmd_sm.set_enable(false);

        // packs command into two words with loop counter
        // commands are always 48 bits minus loop
        self.cmd_sm.tx().push((48_u32 - 1) << 16 | upper >> 16);
        self.cmd_sm.tx().push(upper << 16 | lower as u32);

        self.cmd_sm.set_enable(true);

        let timeout = Timeout::new(Duration::from_millis(2));
        while timeout.check().is_ok() {
            // ensure words were written and sm is stalled
            if self.cmd_sm.tx().empty() && self.cmd_sm.tx().stalled() {
                return Ok(());
            }
        }

        Err(Error::WriteError)
    }

    /// read response and fills buf
    ///
    /// Returns Error on timeout
    async fn read_command(
        &mut self,
        buf: &mut [u8],
        bit_len: u8,
        timeout: Duration,
    ) -> Result<(), Error> {
        self.cmd_sm.set_config(&self.cmd_read_cfg);

        let mut response = [0_u32; LONG_CMD_RESP.div_ceil(32) as usize];
        let response = &mut response[..buf.len().div_ceil(4)];

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
            .push(((bit_len - 2) as u32) << 16 | null_instr.encode(SideSet::default()) as u32);

        with_timeout(
            timeout,
            // use dma because long responses are 5 words
            self.cmd_sm
                .rx()
                .dma_pull(self.dma.reborrow(), response, false),
        )
        .await
        .map_err(|_| {
            self.reset_command();
            Error::Transport
        })?;

        // take ownership of cmd again
        self.cmd_sm.set_config(&self.cmd_write_cfg);

        // break dma u32 resp into byte array
        for (chunk, word) in buf.chunks_mut(4).zip(response.iter()) {
            let bytes = word.to_le().to_be_bytes();
            for (b, dst) in bytes.iter().zip(chunk.iter_mut()) {
                *dst = *b;
            }
        }

        Ok(())
    }
}

struct Timeout {
    deadline: Instant,
}

impl Timeout {
    /// Create a new Delay object with a custom timeout in microseconds.
    fn new(timeout: Duration) -> Self {
        Self {
            deadline: Instant::now().checked_add(timeout).unwrap(),
        }
    }

    /// Check if we are still within the allowed timeout period.
    ///
    /// Returns Ok(()) if still within deadline, or Err(()) if expired.
    fn check(&self) -> Result<(), ()> {
        if Instant::now() > self.deadline {
            Err(())
        } else {
            Ok(())
        }
    }
}
