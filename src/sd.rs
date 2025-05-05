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

const INIT_CLK: u32 = 100_000;

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
        cmd_or_data_prg: PioSdCmdData<'d, PIO>,
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
            inner: PioSdInner::new(
                clk_pin,
                cmd_pin,
                d0_pin,
                clk_prg,
                cmd_or_data_prg,
                pio,
                clk_irq,
                clk_sm,
                cmd_sm,
                data_sm,
                dma,
            ),
        }
    }

    /// ensure cmd pio is ready otherwise return timeout and reset
    fn cmd_delay_ready(&mut self, delay: &mut Delay) -> Result<(), Error> {
        let mut stable_ready_count = 0;

        while delay.check().is_ok() {
            if self.inner.cmd_ready() {
                stable_ready_count += 1;
                if stable_ready_count > 3 {
                    return Ok(());
                }
            } else {
                stable_ready_count = 0;
            }
        }

        self.inner.reset_cmd();
        Err(Error::Transport)
    }

    /// perform a command
    pub fn card_command(
        &mut self,
        command: u8,
        arg: u32,
        response: Option<CmdResponseLen>,
    ) -> Result<u8, Error> {
        let mut delay = Delay::new_command();

        let mut buf = [
            0x40 | command,
            (arg >> 24) as u8,
            (arg >> 16) as u8,
            (arg >> 8) as u8,
            arg as u8,
            0,
        ];
        buf[5] = crc7(&buf[0..5]);

        self.cmd_delay_ready(&mut delay)?;

        let cmd = self.inner.buf_to_cmd(&buf);
        self.inner.write_command(cmd.0, cmd.1);

        self.cmd_delay_ready(&mut delay)?;
        info!("Tx: {:#04X}", buf);

        if let Some(response) = response {
            let mut read = [0; CmdResponseLen::Long as usize];
            self.inner
                .read_command(&mut read[..response as usize / 8], response);
            // let (cmd_idx, cmd_status) = self.parse_cmd_response(&buf)?;
            // info!("RX idx: {:X}, status: {:#04X}", cmd_idx, cmd_status);

            self.cmd_delay_ready(&mut delay)?;
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

        // let mut card_type;
        trace!("Reset card..");
        let card_type;

        let _ = self.card_command(CMD0, 0, None);

        // Check card version
        let arg = loop {
            let status = unwrap!(self.card_command(CMD8, 0x1AA, Some(CmdResponseLen::Short)));
            if status == (R1_ILLEGAL_COMMAND | R1_IDLE_STATE) {
                card_type = CardType::SD1;
                break 0;
            } else if status == 0xAA || status == 0xA8 {
                card_type = CardType::SD2;
                break 0x4000_0000;
            }
            Delay::new(Duration::from_millis(10)).delay();
        };

        // App command
        unwrap!(self.card_command(CMD55, 0, Some(CmdResponseLen::Short)));

        // the card's initialization process
        unwrap!(self.card_command(ACMD41, 0x4000_0000, Some(CmdResponseLen::Short)));

        // unwrap!(self.card_command(0x02, 0, Some(CmdResponseLen::Long)));

        // let rca_resp = unwrap!(self.card_command(0x03, 0, Some(CmdResponseLen::Short)));
        // let rca = (rca_resp as u16) << 8 | (rca_resp as u16); // depends on how your response bytes are arranged
        // info!("RCA: {}", rca);

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

pub struct PioSdCmdData<'d, PIO: Instance> {
    cmd_or_data: LoadedProgram<'d, PIO>,
    no_arg_state_wait_high: u8,
    no_arg_state_waiting_for_cmd: u8,
    state_send_bits: u8,
    state_inline_instruction: u8,
    state_receive_bits: u8,
}

impl<'d, PIO: Instance> PioSdCmdData<'d, PIO> {
    pub fn new(common: &mut Common<'d, PIO>) -> Self {
        // This is a dual purpose program for both cmd and data
        // source: https://github.com/raspberrypi/pico-extras/blob/master/src/rp2_common/pico_sd_card/sd_card.pio
        // opted to exclude 4-bit mode for quick testing
        let cmd_or_data_prg = pio_asm!(
            r#"
                .origin 0 ; must load at zero (offsets are hardcoded in instruction stream)
                public no_arg_state_wait_high:      ; this is a no arg state which means it must always appear in the second half of a word
                    ; make sure pins are hi when we set output dir ( avoid pin glitch/accidental start bit )
                    set pins, 1
                    set pindirs, 1 
                
                public no_arg_state_waiting_for_cmd:
                    out exec, 16                    ; expected to be a jmp to a state
                
                public state_send_bits:
                    out x, 16
                    wait 0 irq 0
                slp:
                    out pins, 1
                    jmp x-- slp
                
                public state_inline_instruction:
                    out exec, 16                     ; may be any instruction
                .wrap_target
                    out exec, 16                     ; expected to be a jmp to a state
                
                public state_receive_bits:
                    out x, 16
                    set pindirs, 0
                    wait 1 pin, 0
                    wait 0 pin, 0
                    wait 0 irq 0
                    ; note we use wrap setup to configure receive bit/nibble transfers
                rlp:
                    in pins, 1
                    jmp x-- rlp
                .wrap
            "#,
        );
        let cmd_or_data = common.load_program(&cmd_or_data_prg.program);

        Self {
            cmd_or_data,
            no_arg_state_wait_high: cmd_or_data_prg.public_defines.no_arg_state_wait_high as u8,
            no_arg_state_waiting_for_cmd: cmd_or_data_prg
                .public_defines
                .no_arg_state_waiting_for_cmd as u8,
            state_send_bits: cmd_or_data_prg.public_defines.state_send_bits as u8,
            state_inline_instruction: cmd_or_data_prg.public_defines.state_inline_instruction as u8,
            state_receive_bits: cmd_or_data_prg.public_defines.state_receive_bits as u8,
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
    cmd_or_data_prg: PioSdCmdData<'d, PIO>,
    cmd_cfg: pio::Config<'d, PIO>,
    cmd_sm: StateMachine<'d, PIO, SM1>,
    data_cfg: pio::Config<'d, PIO>,
    data_sm: StateMachine<'d, PIO, SM2>,
}

impl<'d, PIO: Instance, C: Channel, const SM0: usize, const SM1: usize, const SM2: usize>
    PioSdInner<'d, PIO, C, SM0, SM1, SM2>
{
    pub fn new(
        clk_pin: impl PioPin,
        cmd_pin: impl PioPin,
        d0_pin: impl PioPin,
        clk_prg: PioSdClk<'d, PIO>,
        cmd_or_data_prg: PioSdCmdData<'d, PIO>,
        pio: &mut Common<'d, PIO>,
        clk_irq: pio::Irq<'d, PIO, 0>,
        mut clk_sm: StateMachine<'d, PIO, SM0>,
        mut cmd_sm: StateMachine<'d, PIO, SM1>,
        mut data_sm: StateMachine<'d, PIO, SM2>,
        dma: C,
    ) -> Self {
        into_ref!(dma);
        let div = (clk_sys_freq() / INIT_CLK) as u16;

        // Clk program config
        let mut clk = pio.make_pio_pin(clk_pin);
        clk.set_pull(Pull::Down);
        let mut clk_cfg = pio::Config::default();
        clk_cfg.use_program(&clk_prg.clk, &[&clk]);
        clk_cfg.clock_divider = div.into();
        clk_sm.set_config(&clk_cfg);
        clk_sm.set_pin_dirs(Direction::Out, &[&clk]);
        clk_sm.set_enable(true);

        let shift_cfg = ShiftConfig {
            threshold: 32,
            direction: ShiftDirection::Left,
            auto_fill: true,
        };

        // Cmd config
        let mut cmd = pio.make_pio_pin(cmd_pin);
        cmd.set_pull(Pull::Up);
        let mut cmd_cfg = pio::Config::default();
        cmd_cfg.use_program(&cmd_or_data_prg.cmd_or_data, &[]);
        cmd_cfg.set_set_pins(&[&cmd]);
        cmd_cfg.set_out_pins(&[&cmd]);
        cmd_cfg.set_in_pins(&[&cmd]);
        cmd_cfg.clock_divider = div.into();
        cmd_cfg.shift_in = shift_cfg;
        cmd_cfg.shift_out = shift_cfg;
        cmd_sm.set_config(&cmd_cfg);
        cmd_sm.clear_fifos();
        // cmd_sm.set_enable(true);
        unsafe { cmd_sm.exec_jmp(cmd_or_data_prg.no_arg_state_wait_high) };

        // Data config
        let mut d0 = pio.make_pio_pin(d0_pin);
        d0.set_pull(Pull::Up);
        let mut data_cfg = pio::Config::default();
        data_cfg.use_program(&cmd_or_data_prg.cmd_or_data, &[]);
        data_cfg.set_set_pins(&[&d0]);
        data_cfg.set_out_pins(&[&d0]);
        data_cfg.set_in_pins(&[&d0]);
        data_cfg.clock_divider = div.into();
        data_cfg.shift_in = shift_cfg;
        data_cfg.shift_out = shift_cfg;
        data_sm.set_config(&data_cfg);
        data_sm.clear_fifos();
        // data_sm.set_enable(true);
        unsafe { cmd_sm.exec_jmp(cmd_or_data_prg.no_arg_state_waiting_for_cmd) };

        let jmp = Self::get_jmp(cmd_or_data_prg.state_send_bits);
        cmd_sm.tx().push((jmp as u32) << 16 | 80 - 1);
        cmd_sm.tx().push(0xffffffff);
        cmd_sm.tx().push(0xffffffff);
        cmd_sm
            .tx()
            .push(0xffff0000 | Self::get_jmp(cmd_or_data_prg.no_arg_state_wait_high) as u32);
        pio.apply_sm_batch(|_| {
            cmd_sm.set_enable(true);
            data_sm.set_enable(true);
        });

        Self {
            dma,
            clk_irq,
            clk_cfg,
            clk_sm,
            cmd_or_data_prg,
            cmd_cfg,
            cmd_sm,
            data_cfg,
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

    /// writes cmd to sm
    /// requires sm to be at exec to jmp to write state
    fn write_command(&mut self, upper: u32, lower: u16) {
        assert!(self.cmd_ready());

        // disable sm to avoid sm stall after first word is read and losing clk
        self.cmd_sm.set_enable(false);

        // jmp to send_bits and set bit counter to 48 bits
        let jmp_send = Self::get_jmp(self.cmd_or_data_prg.state_send_bits);
        self.cmd_sm.tx().push((jmp_send as u32) << 16 | 48 - 1);

        self.cmd_sm.tx().push(upper);
        self.cmd_sm.tx().push((lower as u32) << 16);

        self.cmd_sm.set_enable(true);
    }

    /// reads cmd bits from sm into buff
    /// requires sm to be at exec to jmp to read state
    fn read_command(&mut self, buff: &mut [u8], response_len: CmdResponseLen) {
        assert!(self.cmd_ready());
        assert!(buff.len() >= response_len as usize / 8);

        // jmp to recv_bits and set bit counter to response_len
        let jmp_read = Self::get_jmp(self.cmd_or_data_prg.state_receive_bits);
        self.cmd_sm
            .tx()
            .push((jmp_read as u32) << 16 | response_len as u32 - 1);

        // iterate in chunks of 32bits for 48 or 148 bits
        for chunk in buff[0..(response_len as usize / 8)].chunks_mut(4) {
            let read = self.cmd_sm.rx().pull();

            chunk[0] = (read >> 24) as u8;
            if chunk.len() > 1 {
                chunk[1] = (read >> 16) as u8;
            }
            if chunk.len() > 2 {
                chunk[2] = (read >> 8) as u8;
            }
            if chunk.len() > 3 {
                chunk[3] = read as u8;
            }
        }
        self.cmd_sm.clear_fifos();

        // push remaining bits out of isr if bit len is not div 32
        if response_len as u32 & 32 != 0 {
            let jmp_instr = Self::get_jmp(self.cmd_or_data_prg.state_inline_instruction);
            let null_instr = Instruction {
                operands: pio::program::InstructionOperands::IN {
                    source: pio::program::InSource::NULL,
                    bit_count: 32 - (response_len as u8 & 31),
                },
                delay: 0,
                side_set: None,
            };
            self.cmd_sm
                .tx()
                .push((jmp_instr as u32) << 16 | null_instr.encode(SideSet::default()) as u32);
        }

        // set cmd pin back as output and wait for next state
        let jmp_instr = Self::get_jmp(self.cmd_or_data_prg.state_inline_instruction) as u32;
        let jmp_wait_high = Self::get_jmp(self.cmd_or_data_prg.no_arg_state_wait_high) as u32;
        self.cmd_sm.tx().push(jmp_instr << 16 | jmp_wait_high);
    }

    /// create a jmp exec instruction to an address
    fn get_jmp(addr: u8) -> u8 {
        let jmp = Instruction {
            operands: pio::program::InstructionOperands::JMP {
                condition: pio::program::JmpCondition::Always,
                address: addr,
            },
            delay: 0,
            side_set: None,
        };
        jmp.encode(SideSet::default()) as u8
    }

    /// checks if sm is at a exec instruction and ready to jmp to next state
    fn cmd_ready(&mut self) -> bool {
        let pc = self.cmd_sm.get_addr();
        // info!("CMD PC: {}", pc);
        pc == self.cmd_or_data_prg.no_arg_state_waiting_for_cmd
            || pc == self.cmd_or_data_prg.state_inline_instruction
            || pc == self.cmd_or_data_prg.state_inline_instruction + 1
                && self.cmd_sm.tx().empty()
                && self.cmd_sm.rx().empty()
    }

    /// resets cmd pio sm when in an unknown state or is stuck
    fn reset_cmd(&mut self) {
        self.cmd_sm.set_enable(false);
        self.cmd_sm.clear_fifos();
        self.cmd_sm.restart();
        unsafe {
            self.cmd_sm
                .exec_jmp(self.cmd_or_data_prg.no_arg_state_wait_high)
        };
        self.cmd_sm.set_enable(true);
        while !self.cmd_ready() {}
    }

    /// resets data pio sm when in an unknown state or is stuck
    fn reset_data(&mut self) {
        self.data_sm.set_enable(false);
        self.data_sm.clear_fifos();
        self.data_sm.restart();
        unsafe {
            self.data_sm
                .exec_jmp(self.cmd_or_data_prg.no_arg_state_wait_high)
        };
        self.data_sm.set_enable(true);
        // while !self.data_ready() {}
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
