use embassy_rp::{
    Peri,
    dma::{AnyChannel, Channel},
    gpio::{Drive, Level, Pull, SlewRate},
    pio::{
        Common, Config, Direction, Instance, Irq, LoadedProgram, PioPin, ShiftConfig,
        ShiftDirection, StateMachine,
        program::{InSource, Instruction, InstructionOperands, SideSet, pio_asm},
    },
    pio_programs::clock_divider::calculate_pio_clock_divider,
};
use embassy_time::{Duration, Instant, with_timeout};

/// Sdio Errors
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Copy, Clone)]
pub enum SdioError {
    /// State machine encountered an error writing command
    CmdWriteError,
    /// State machine encountered an error reading command response
    CmdReadError,
    /// State machine encountered an error writing data
    DataWriteError,
    /// State machine encountered an error reading data
    DataReadError,
    /// Commands must be 48bits or 6 bytes
    WrongCmdLen,
}

/// Pio Program for sdio clk
pub struct PioSdioClk<'d, PIO: Instance> {
    clk: LoadedProgram<'d, PIO>,
}

impl<'d, PIO: Instance> PioSdioClk<'d, PIO> {
    pub fn new(common: &mut Common<'d, PIO>) -> Self {
        let clk_prg = pio_asm!(".side_set 1", "irq 0 side 1", "irq clear 0 side 0",);
        let clk = common.load_program(&clk_prg.program);
        Self { clk }
    }
}

/// Pio Programs for sdio 1bit
pub struct PioSdio1bit<'d, PIO: Instance> {
    write: LoadedProgram<'d, PIO>,
    read: LoadedProgram<'d, PIO>,
}

impl<'d, PIO: Instance> PioSdio1bit<'d, PIO> {
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

pub struct PioSdio<'d, PIO: Instance, const SM0: usize, const SM1: usize, const SM2: usize> {
    dma: Peri<'d, AnyChannel>,
    _clk_irq: Irq<'d, PIO, 0>,
    clk_cfg: Config<'d, PIO>,
    clk_sm: StateMachine<'d, PIO, SM0>,
    cmd_write_cfg: Config<'d, PIO>,
    cmd_read_cfg: Config<'d, PIO>,
    cmd_sm: StateMachine<'d, PIO, SM1>,
    data_write_cfg: Config<'d, PIO>,
    data_read_cfg: Config<'d, PIO>,
    data_sm: StateMachine<'d, PIO, SM2>,
}

impl<'d, PIO: Instance, const SM0: usize, const SM1: usize, const SM2: usize>
    PioSdio<'d, PIO, SM0, SM1, SM2>
{
    /// Create a new 1bit sdio bus with 1 card
    pub fn new_1_bit(
        clk_pin: Peri<'d, impl PioPin>,
        cmd_pin: Peri<'d, impl PioPin>,
        data_pin: Peri<'d, impl PioPin>,
        clk_prg: PioSdioClk<'d, PIO>,
        one_bit_prg: PioSdio1bit<'d, PIO>,
        pio: &mut Common<'d, PIO>,
        clk_irq: Irq<'d, PIO, 0>,
        mut clk_sm: StateMachine<'d, PIO, SM0>,
        mut cmd_sm: StateMachine<'d, PIO, SM1>,
        mut data_sm: StateMachine<'d, PIO, SM2>,
        dma: Peri<'d, impl Channel>,
    ) -> Self {
        let clkdiv = calculate_pio_clock_divider(100_000);

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
        let mut clk_cfg = Config::default();
        clk_cfg.use_program(&clk_prg.clk, &[&clk]);
        clk_cfg.clock_divider = clkdiv;

        let shift_cfg = ShiftConfig {
            threshold: 32,
            direction: ShiftDirection::Left,
            auto_fill: true,
        };

        // Cmd config
        let mut cmd_write_cfg = Config::default();
        cmd_write_cfg.use_program(&one_bit_prg.write, &[]);
        cmd_write_cfg.set_set_pins(&[&cmd]);
        cmd_write_cfg.set_out_pins(&[&cmd]);
        cmd_write_cfg.clock_divider = clkdiv;
        cmd_write_cfg.shift_out = shift_cfg;

        let mut cmd_read_cfg = Config::default();
        cmd_read_cfg.use_program(&one_bit_prg.read, &[]);
        cmd_read_cfg.set_set_pins(&[&cmd]);
        cmd_read_cfg.set_in_pins(&[&cmd]);
        cmd_read_cfg.clock_divider = clkdiv;
        cmd_read_cfg.shift_out = shift_cfg;
        cmd_read_cfg.shift_in = shift_cfg;

        // data config
        let mut data_write_cfg = Config::default();
        data_write_cfg.use_program(&one_bit_prg.write, &[]);
        data_write_cfg.set_set_pins(&[&data]);
        data_write_cfg.set_out_pins(&[&data]);
        data_write_cfg.clock_divider = clkdiv;
        data_write_cfg.shift_out = shift_cfg;

        let mut data_read_cfg = Config::default();
        data_read_cfg.use_program(&one_bit_prg.read, &[]);
        data_read_cfg.set_set_pins(&[&data]);
        data_read_cfg.set_in_pins(&[&data]);
        data_read_cfg.clock_divider = clkdiv;
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
            dma: dma.into(),
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

    /// Change the sdio bus frequency after initilzing the card
    pub fn set_freq(&mut self, freq: u32) {
        let clkdiv = calculate_pio_clock_divider(freq);

        self.clk_cfg.clock_divider = clkdiv;
        self.cmd_write_cfg.clock_divider = clkdiv;
        self.cmd_read_cfg.clock_divider = clkdiv;
        self.data_write_cfg.clock_divider = clkdiv;
        self.data_read_cfg.clock_divider = clkdiv;

        self.clk_sm.set_config(&self.clk_cfg);
        self.clk_sm.restart();

        self.cmd_sm.clear_fifos();
        self.cmd_sm.restart();

        self.data_sm.clear_fifos();
        self.data_sm.restart();
    }

    /// writes data block to card
    ///
    /// returns `Sdio::DataWriteError` if write fails
    async fn write_data(&mut self, block: &[u8]) -> Result<(), SdioError> {
        // TODO: wont work
        // self.data_sm.tx().push((512 - 1) << 16 | 0xFF); // keep line held high

        with_timeout(
            Duration::from_millis(100),
            self.data_sm
                .tx()
                .dma_push(self.dma.reborrow(), block, false),
        )
        .await
        .map_err(|_| {
            self.data_sm.clear_fifos();
            self.data_sm.restart();
            SdioError::DataWriteError
        })
    }

    /// Read data block into provided block buffer
    ///
    /// returns `Sdio::DataReadError` if read fails
    pub async fn read_data(&mut self, block: &mut [u8]) -> Result<(), SdioError> {
        self.data_sm.set_config(&self.data_read_cfg);

        let mut response = [0_u32; 16 + 2]; // 512 bytes
        let bits = 512 + 16; // block +16 for 2 byte crc

        // creates an exec instruction to flush bits left in isr
        let null_instr = Instruction {
            operands: InstructionOperands::IN {
                source: InSource::NULL,
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
        .map_err(|_| -> SdioError {
            self.cmd_sm.clear_fifos();
            self.cmd_sm.restart();
            SdioError::DataReadError
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

    /// writes command to card
    ///
    /// returns `SdioError::CmdReadError` if cmd length is not 6
    pub fn write_command(&mut self, cmd: &[u8]) -> Result<(), SdioError> {
        if cmd.len() != 6 {
            return Err(SdioError::WrongCmdLen);
        }
        let upper =
            (cmd[0] as u32) << 24 | (cmd[1] as u32) << 16 | (cmd[2] as u32) << 8 | cmd[3] as u32;
        let lower = (cmd[4] as u16) << 8 | cmd[5] as u16;

        self.cmd_sm.set_enable(false);

        // packs command into two words with loop counter
        // commands are always 48 bits minus loop
        self.cmd_sm.tx().push((48_u32 - 1) << 16 | upper >> 16);
        self.cmd_sm.tx().push(upper << 16 | lower as u32);

        self.cmd_sm.set_enable(true);

        let timeout = Instant::now()
            .checked_add(Duration::from_millis(2))
            .ok_or(SdioError::CmdWriteError)?;
        while Instant::now() < timeout {
            // ensure words were written and sm is stalled
            if self.cmd_sm.tx().empty() && self.cmd_sm.tx().stalled() {
                return Ok(());
            }
        }

        Err(SdioError::CmdWriteError)
    }

    /// Attempts to read buf len from bus, until timeout and then return `SdioError::CmdReadError`
    pub async fn read_command(
        &mut self,
        buf: &mut [u8],
        bit_len: u8,
        timeout: Duration,
    ) -> Result<(), SdioError> {
        self.cmd_sm.set_config(&self.cmd_read_cfg);

        let mut response = [0_u32; 5];
        let response = &mut response[..buf.len().div_ceil(4)];

        // creates an exec instruction to flush bits left in isr
        let null_instr = Instruction {
            operands: InstructionOperands::IN {
                source: InSource::NULL,
                bit_count: 32 - (bit_len % 32),
            },
            delay: 0,
            side_set: None,
        };
        self.cmd_sm
            .tx()
            .push(((bit_len - 2) as u32) << 16 | null_instr.encode(SideSet::default()) as u32);

        let res = with_timeout(
            timeout,
            // use dma because long responses are 5 words
            self.cmd_sm
                .rx()
                .dma_pull(self.dma.reborrow(), response, false),
        )
        .await
        .map_err(|_| {
            self.cmd_sm.clear_fifos();
            self.cmd_sm.restart();
            SdioError::CmdReadError
        });

        // take ownership of cmd again
        self.cmd_sm.set_config(&self.cmd_write_cfg);
        res?;

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
