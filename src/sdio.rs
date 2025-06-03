use defmt::info;
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
pub struct PioSdioCmd<'d, PIO: Instance> {
    cmd: LoadedProgram<'d, PIO>,
}

impl<'d, PIO: Instance> PioSdioCmd<'d, PIO> {
    /// Creates a new 1-bit Tx/Rx program
    pub fn new(common: &mut Common<'d, PIO>) -> Self {
        let cmd = pio_asm!(
            // write
            "set pins, 1",
            "set pindirs, 1",
            "out x, 16",    // sets the write bit len, stalls waiting for write
            "wait 0 irq 0", // wait for clk
            "write_loop:",
            "out pins, 1",
            "jmp x-- write_loop",
            // read
            "out y, 16",  // configures the read bit length
            "in null, 1", // preload start bit which is skipped below
            "set pindirs, 0",
            "wait 1 pin, 0", // wait until card takes ownership
            "wait 0 pin, 0", // start bit
            "wait 0 irq 0",  // wait for clk
            "read_loop:",
            "in pins, 1",
            "jmp y-- read_loop",
            "push", // fetch remaining bits in the isr
        );
        Self {
            cmd: common.load_program(&cmd.program),
        }
    }
}

/// Pio Programs for sdio 1bit Data
pub struct PioSdio1bitData<'d, PIO: Instance> {
    write: LoadedProgram<'d, PIO>,
    read: LoadedProgram<'d, PIO>,
}

impl<'d, PIO: Instance> PioSdio1bitData<'d, PIO> {
    /// Creates a new 1-bit Data Tx/Rx program
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
    clk_cfg: Config<'d, PIO>,
    clk_sm: StateMachine<'d, PIO, SM0>,
    cmd_cfg: Config<'d, PIO>,
    cmd_sm: StateMachine<'d, PIO, SM1>,
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
        cmd_prg: PioSdioCmd<'d, PIO>,
        data_prg: PioSdio1bitData<'d, PIO>,
        pio: &mut Common<'d, PIO>,
        _clk_irq: Irq<'d, PIO, 0>,
        mut clk_sm: StateMachine<'d, PIO, SM0>,
        mut cmd_sm: StateMachine<'d, PIO, SM1>,
        mut data_sm: StateMachine<'d, PIO, SM2>,
        dma: Peri<'d, impl Channel>,
    ) -> Self {
        let clockdiv = calculate_pio_clock_divider(100_000);

        clk_sm.clear_fifos();
        let mut clk = pio.make_pio_pin(clk_pin);
        clk.set_pull(Pull::Down);
        clk.set_slew_rate(SlewRate::Fast);
        clk.set_drive_strength(Drive::_8mA);

        let mut clk_cfg = Config::default();
        clk_cfg.clock_divider = clockdiv;
        clk_cfg.use_program(&clk_prg.clk, &[&clk]);

        let shift_cfg = ShiftConfig {
            threshold: 32,
            direction: ShiftDirection::Left,
            auto_fill: true,
        };

        cmd_sm.clear_fifos();
        let mut cmd = pio.make_pio_pin(cmd_pin);
        cmd.set_pull(Pull::Up);

        let mut cmd_cfg = Config::default();
        cmd_cfg.clock_divider = clockdiv;
        cmd_cfg.use_program(&cmd_prg.cmd, &[]);
        cmd_cfg.set_set_pins(&[&cmd]);
        cmd_cfg.set_in_pins(&[&cmd]);
        cmd_cfg.set_out_pins(&[&cmd]);
        cmd_cfg.shift_in = shift_cfg;
        cmd_cfg.shift_out = ShiftConfig {
            threshold: 16,
            direction: ShiftDirection::Left,
            auto_fill: true,
        };

        data_sm.clear_fifos();
        let mut data = pio.make_pio_pin(data_pin);
        data.set_pull(Pull::Up);

        let mut data_write_cfg = Config::default();
        data_write_cfg.clock_divider = clockdiv;
        data_write_cfg.use_program(&data_prg.write, &[]);
        data_write_cfg.set_set_pins(&[&data]);
        data_write_cfg.set_out_pins(&[&data]);
        data_write_cfg.shift_out = shift_cfg;

        let mut data_read_cfg = Config::default();
        data_write_cfg.clock_divider = clockdiv;
        data_read_cfg.use_program(&data_prg.read, &[]);
        data_read_cfg.set_set_pins(&[&data]);
        data_read_cfg.set_in_pins(&[&data]);
        data_read_cfg.shift_out = shift_cfg;
        data_read_cfg.shift_in = shift_cfg;

        clk_sm.set_pins(Level::Low, &[&clk]);
        clk_sm.set_pin_dirs(Direction::Out, &[&clk]);
        clk_sm.set_config(&clk_cfg);

        cmd_sm.set_config(&cmd_cfg);
        // data_sm.set_config(&data_write_cfg); // the default program is the tx so the pins are held high

        // start write programs so pins are held high
        pio.apply_sm_batch(|pio| {
            pio.set_enable(&mut cmd_sm, true);
            pio.set_enable(&mut data_sm, false);
            pio.set_enable(&mut clk_sm, true);
        });

        Self {
            dma: dma.into(),
            clk_cfg,
            clk_sm,
            cmd_cfg,
            cmd_sm,
            data_sm,
        }
    }

    /// writes command to card and listens for a response if read_buf is not empty
    ///
    /// returns `SdioError::WrongCmdLen` if cmd length is not 6
    pub async fn command(
        &mut self,
        cmd: &[u8],
        read_buf: &mut [u8],
        read_timeout: Duration,
    ) -> Result<(), SdioError> {
        if cmd.len() != 6 {
            return Err(SdioError::WrongCmdLen);
        }

        self.cmd_sm.set_enable(false);

        self.cmd_sm.tx().push(48 - 1);

        // packs commands into the top 16 for 16bit shift
        for bytes in cmd.chunks(2) {
            self.cmd_sm
                .tx()
                .push((bytes[0] as u32) << 24 | (bytes[1] as u32) << 16);
        }

        // push the number of bits to read after write
        if !read_buf.is_empty() {
            self.cmd_sm.tx().push(read_buf.len() as u32 * 8);
        }

        self.cmd_sm.set_enable(true);

        if !read_buf.is_empty() {
            let mut response = [0_u32; 5];
            let response = &mut response[..read_buf.len().div_ceil(4) - 2];

            with_timeout(
                read_timeout,
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
            })?;

            info!("got an response");

            // break dma u32 resp into byte array
            for (chunk, word) in read_buf.chunks_mut(4).zip(response.iter()) {
                let bytes = word.to_be_bytes();
                for (b, dst) in bytes.iter().zip(chunk.iter_mut()) {
                    *dst = *b;
                }
            }
        } else {
            // counter underflows if not reading
            self.cmd_sm.restart();
        }

        Ok(())
    }
}
