use defmt::{debug, expect, info, trace, unwrap, warn};
use embassy_rp::clocks::clk_sys_freq;
use embassy_rp::dma::Channel;
use embassy_rp::gpio::Pull;
use embassy_rp::pio::program::{Instruction, Program, SideSet, pio_asm};
use embassy_rp::pio::{
    self, Common, Direction, FifoJoin, Instance, InterruptHandler, LoadedProgram, Pio, PioPin,
    ShiftConfig, ShiftDirection, StateMachine,
};
use embassy_rp::{PeripheralRef, into_ref};
use embassy_time::{Duration, Instant, Timer};
use embedded_sdmmc::sdcard::proto::*;
use embedded_sdmmc::sdcard::{AcquireOpts, CardType, Error};
use embedded_sdmmc::{SdCard, TimeSource, Timestamp, VolumeIdx, VolumeManager};

const INIT_CLK: u32 = 200_000;

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
    options: AcquireOpts,
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
        options: AcquireOpts,
    ) -> Self {
        Self {
            options,
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
    fn card_command(
        &mut self,
        command: u8,
        arg: u32,
        response: Option<CmdResponseLen>,
    ) -> Result<u8, Error> {
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
        info!("Tx: {:#04X}", buf);

        if let Some(response) = response {
            self.inner.reset_command();
            let mut read = [0; CmdResponseLen::Long as usize];
            self.inner
                .read_command(&mut read[..response as usize / 8])?;
            // let (cmd_idx, cmd_status) = self.parse_cmd_response(&buf)?;
            // info!("RX idx: {:X}, status: {:#04X}", cmd_idx, cmd_status);
            info!("RX: {:#04X}", read[..response as usize / 8]);

            return Ok(read[0]);
        }

        Ok(0)
    }

    fn parse_cmd_response(&self, buf: &[u8]) -> Result<(u8, u32), Error> {
        assert!(buf.len() == 6);

        let command_index = (buf[0] >> 2) & 0x3F; // 6 bits
        let status = ((buf[1] as u32) << 24)
            | ((buf[2] as u32) << 16)
            | ((buf[3] as u32) << 8)
            | (buf[4] as u32);

        Ok((command_index, status))
    }

    /// Check the card is initialised.
    pub fn check_init(&mut self) -> Result<(), Error> {
        if self.card_type.is_none() {
            // If we don't know what the card type is, try and initialise the
            // card. This will tell us what type of card it is.
            self.acquire()
        } else {
            Ok(())
        }
    }

    fn acquire(&mut self) -> Result<(), Error> {
        debug!("acquiring card with opts: {:?}", self.options);

        // Wait initial 74+ clocks high
        Delay::new(Duration::from_hz(INIT_CLK as u64) * 80).delay();

        trace!("Reset card..");
        let _ = self.card_command(CMD0, 0, None);

        let _ = self.card_command(CMD8, 0x1AA, Some(CmdResponseLen::Short));

        // Loop sending ACMD41 until card leaves idle
        for _ in 0..100 {
            if let Err(_) = self.card_command(CMD55, 0, Some(CmdResponseLen::Short)) {
                continue;
            }

            let resp = self.card_command(ACMD41, 0x400_000, Some(CmdResponseLen::Short))?;
            if (resp & R1_IDLE_STATE) == 0 {
                break;
            }

            Delay::new(Duration::from_millis(50)).delay();
        }

        // Optional: CMD58 to check OCR — useful, but can skip if you're only initializing SD
        // Optional: CMD2, CMD3, etc.

        // info!("Card initialized: {:?}", card_type);

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
            "wlp:",
            "out pins, 1",
            "jmp x-- wlp",
            ".wrap",
        );

        let read = pio_asm!(
            "out x, 16",
            "set pindirs, 0",
            "wait 1 pin, 0", // wait until card takes ownership
            "wait 0 pin, 0",
            "wait 0 irq 0", // wait for clk
            "rlp:",
            "in pins, 1",
            "jmp x-- rlp",
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
    clk_irq: pio::Irq<'d, PIO, 0>,
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
            clk_irq,
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

    fn buf_to_cmd(&self, buf: &[u8]) -> (u32, u16) {
        assert!(buf.len() == 6);
        (
            (buf[0] as u32) << 24 | (buf[1] as u32) << 16 | (buf[2] as u32) << 8 | buf[3] as u32,
            (buf[4] as u16) << 8 | buf[5] as u16,
        )
    }

    fn make_sdio_cmd(cmd: u8, arg: u32) -> u64 {
        assert!(cmd < 64);
        let mut bits = 0u64;

        bits |= 0b01 << 46; // Start bit (0) + Tx bit (1)
        bits |= (cmd as u64) << 40;
        bits |= (arg as u64) << 8;

        let mut buf = [0u8; 5];
        buf[0] = 0b01 << 6 | cmd; // First byte: start + tx + cmd
        buf[1] = (arg >> 24) as u8;
        buf[2] = (arg >> 16) as u8;
        buf[3] = (arg >> 8) as u8;
        buf[4] = arg as u8;

        let crc = crc7(&buf);
        bits |= (crc as u64) << 1;
        bits |= 1; // Stop bit

        bits
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
        self.cmd_sm.tx().push((48 - 1 as u32) << 16 | upper >> 16);
        self.cmd_sm.tx().push(upper << 16 | lower as u32);

        self.cmd_sm.set_enable(true);

        let timeout = Delay::new_command();
        while timeout.check().is_ok() {
            // ensure words were written and sm is stalled
            if self.cmd_sm.tx().empty() && self.cmd_sm.tx().stalled() {
                return Ok(());
            }
        }

        Err(Error::WriteError)
    }

    /// fills the entire buff with read bits
    pub fn read_command(&mut self, buff: &mut [u8]) -> Result<(), Error> {
        self.cmd_sm.set_config(&self.cmd_read_cfg);

        // creates an exec instruction to flush bits left in isr
        let null_instr = Instruction {
            operands: pio::program::InstructionOperands::IN {
                source: pio::program::InSource::NULL,
                bit_count: 32 - ((buff.len() as u8 * 8) % 31),
            },
            delay: 0,
            side_set: None,
        };

        // set loop counter to the length of buff
        // and pack flush instruction
        self.cmd_sm.tx().push(
            ((buff.len() as u32 * 8) - 1) << 16 | null_instr.encode(SideSet::default()) as u32,
        );

        let timeout = Delay::new_command();
        let mut i = 0;

        while i < buff.len() {
            if timeout.check().is_err() {
                self.reset_command();
                return Err(Error::ReadError);
            }

            if let Some(read) = self.cmd_sm.rx().try_pull() {
                info!("READ: {:032b}", read);

                let remaining = &mut buff[i..];
                remaining[0] = (read >> 24) as u8;
                if remaining.len() > 1 {
                    remaining[1] = (read >> 16) as u8;
                }
                if remaining.len() > 2 {
                    remaining[2] = (read >> 8) as u8;
                }
                if remaining.len() > 3 {
                    remaining[3] = read as u8;
                }

                i += 4;
            }
        }

        Ok(())
    }
}

// number of bits for cmd responses
#[derive(Copy, Clone)]
enum CmdResponseLen {
    Short = 48,
    Long = 136,
}

/// This is a timer-based object you can use to enforce a timeout
/// when waiting for SDIO card responses.
struct Delay {
    deadline: Instant,
}

impl Delay {
    /// Create a new Delay object with a custom timeout in microseconds.
    fn new(timeout: Duration) -> Self {
        Self {
            deadline: Instant::now().checked_add(timeout).unwrap(),
        }
    }

    /// Create a Delay for standard command waiting (~100ms).
    fn new_command() -> Self {
        Self::new(Duration::from_millis(100))
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

    /// Delay ultil timeout has been hit
    fn delay(&self) {
        while self.check().is_err() {}
    }
}
