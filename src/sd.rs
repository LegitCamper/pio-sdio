use defmt::info;
use embassy_rp::clocks::clk_sys_freq;
use embassy_rp::dma::{Channel, Transfer};
use embassy_rp::pio::program::{Instruction, Program, SideSet, pio_asm};
use embassy_rp::pio::{
    self, Common, Direction, FifoJoin, Instance, InterruptHandler, LoadedProgram, Pio, PioPin,
    ShiftConfig, ShiftDirection, StateMachine,
};
use embassy_rp::{PeripheralRef, into_ref};
use embassy_time::Timer;
use embedded_hal::delay::DelayNs;
use embedded_sdmmc::sdcard::proto::*;
use embedded_sdmmc::sdcard::{AcquireOpts, CardType, Error};
use embedded_sdmmc::{SdCard, TimeSource, Timestamp, VolumeIdx, VolumeManager};

pub struct PioSd<
    'd,
    PIO: Instance,
    C: Channel,
    const SM0: usize,
    const SM1: usize,
    const SM2: usize,
> {
    internal: PioSdInner<'d, PIO, C, SM0, SM1, SM2>,
    delayer: embassy_time::Delay,
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
    ) -> Self {
        Self {
            delayer: embassy_time::Delay,
            internal: PioSdInner::new(
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

    pub async fn command(&mut self, command: u8, arg: u32) -> Result<u8, Error> {
        let mut buf = [
            0x40 | command,
            (arg >> 24) as u8,
            (arg >> 16) as u8,
            (arg >> 8) as u8,
            arg as u8,
            0,
        ];
        buf[5] = crc7(&buf[0..5]);
        info!("Calculated Command: {:#04X}", buf);

        let cmd = self.internal.buf_to_cmd(&buf);
        self.internal.write_command(cmd.0, cmd.1).await?;

        // CMD0 sometimes has no response so just ignore
        if command != CMD0 {
            self.internal.read_command(&mut buf, CmdResponseLen::Short);
            info!("RX: {:#04X}", buf);
            let (cmd_idx, cmd_status) = self.parse_cmd_response(&buf)?;
            // info!("RX idx: {:X}, status: {:#04X}", cmd_idx, cmd_status);
        }

        Ok(ERROR_OK)
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

    pub async fn acquire(&mut self) -> Result<(), Error> {
        // Send initial clocks (74+)
        Timer::after_millis(1).await;

        // Go idle
        self.command(CMD0, 0).await?;
        // Check voltage
        self.command(CMD8, 0x1AA).await?;
        // App command
        self.command(CMD55, 0).await?;
        //
        self.command(ACMD41, 0x4000_0000).await?;

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
    no_arg_state_wait_high: u32,
    no_arg_state_waiting_for_cmd: u32,
    state_send_bits: u32,
    state_inline_instruction: u32,
    state_receive_bits: u32,
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
            no_arg_state_wait_high: cmd_or_data_prg.public_defines.no_arg_state_wait_high as u32,
            no_arg_state_waiting_for_cmd: cmd_or_data_prg
                .public_defines
                .no_arg_state_waiting_for_cmd as u32,
            state_send_bits: cmd_or_data_prg.public_defines.state_send_bits as u32,
            state_inline_instruction: cmd_or_data_prg.public_defines.state_inline_instruction
                as u32,
            state_receive_bits: cmd_or_data_prg.public_defines.state_receive_bits as u32,
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
        let div = (clk_sys_freq() / 400_000) as u16;

        // Clk program config
        let clk = pio.make_pio_pin(clk_pin);
        let mut clk_cfg = pio::Config::default();
        clk_cfg.use_program(&clk_prg.clk, &[&clk]);
        clk_cfg.clock_divider = div.into();
        clk_sm.set_config(&clk_cfg);
        clk_sm.set_pin_dirs(Direction::Out, &[&clk]);
        clk_sm.set_enable(true);

        let shift_cfg = pio::ShiftConfig {
            threshold: 32,
            direction: pio::ShiftDirection::Left,
            auto_fill: true,
        };

        // Cmd config
        let cmd = pio.make_pio_pin(cmd_pin);
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
        cmd_sm.set_enable(true);

        // Data config
        let d0 = pio.make_pio_pin(d0_pin);
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
        data_sm.set_enable(true);

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

    // writes command to cmd
    async fn write_command(&mut self, upper: u32, lower: u16) -> Result<(), Error> {
        while !self.cmd_sm.tx().empty() {}
        self.cmd_wait_ready().await?;

        // disable sm to avoid sm stall after first word is read and losing clk
        self.cmd_sm.set_enable(false);

        // jmp to send_bits and set bit counter to 48 bits
        let jmp_send = self.get_jmp(self.cmd_or_data_prg.state_send_bits as u8);
        self.cmd_sm.tx().push(jmp_send << 16 | 48 - 1);

        self.cmd_sm.tx().push(upper);
        self.cmd_sm.tx().push((lower as u32) << 16);

        self.cmd_sm.set_enable(true);
        Ok(())
    }

    // reads command response to cmd
    async fn read_command(
        &mut self,
        buff: &mut [u8],
        response_len: CmdResponseLen,
    ) -> Result<(), Error> {
        while !self.cmd_sm.tx().empty() {}
        self.cmd_wait_ready().await?;

        // jmp to recv_bits and set bit counter to response_len
        let jmp_read = self.get_jmp(self.cmd_or_data_prg.state_receive_bits as u8);
        self.cmd_sm
            .tx()
            .push(jmp_read << 16 | response_len as u32 - 1);

        match response_len {
            CmdResponseLen::Short => {
                assert!(buff.len() >= CmdResponseLen::Short as usize / 8);

                // iterate in chunks of 32bits
                for chunk in buff[0..6].chunks_mut(4) {
                    while !self.cmd_sm.rx().empty() {}
                    let read = self.cmd_sm.rx().pull();

                    chunk[0] = (read >> 24) as u8;
                    chunk[1] = (read >> 16) as u8;
                    if chunk.len() > 2 {
                        chunk[2] = (read >> 8) as u8;
                        chunk[3] = read as u8;
                    }
                }
            }
            CmdResponseLen::Long => todo!(),
        }

        // push remaining bits out of isr
        let jmp_instruction = self.get_jmp(self.cmd_or_data_prg.state_inline_instruction as u8);
        let push_inst = Instruction {
            operands: pio::program::InstructionOperands::PUSH {
                if_full: false,
                block: false,
            },
            delay: 0,
            side_set: None,
        };
        self.cmd_sm
            .tx()
            .push(jmp_instruction << 16 | push_inst.encode(SideSet::default()) as u32);

        // set cmd pin back as output and wait for next state
        let jmp_wait_high = self.get_jmp(self.cmd_or_data_prg.no_arg_state_wait_high as u8);
        let jmp_wait_cmd = self.get_jmp(self.cmd_or_data_prg.no_arg_state_waiting_for_cmd as u8);
        self.cmd_sm.tx().push(jmp_wait_high << 16 | jmp_wait_cmd);

        self.cmd_wait_ready().await?;
        Ok(())
    }

    // create a jmp exec instruction to an address
    fn get_jmp(&mut self, addr: u8) -> u32 {
        let jmp = Instruction {
            operands: pio::program::InstructionOperands::JMP {
                condition: pio::program::JmpCondition::Always,
                address: addr,
            },
            delay: 0,
            side_set: None,
        };
        jmp.encode(SideSet::default()).into()
    }

    // checks if sm is at a exec instruction and ready to jmp to next state
    async fn cmd_wait_ready(&self) -> Result<(), Error> {
        for _ in 0..1000 {
            if self.cmd_sm.get_addr() as u32 == self.cmd_or_data_prg.no_arg_state_waiting_for_cmd
                || self.cmd_sm.get_addr() as u32 == self.cmd_or_data_prg.state_inline_instruction
                || self.cmd_sm.get_addr() as u32
                    == self.cmd_or_data_prg.state_inline_instruction + 2
            {
                return Ok(());
            }
            Timer::after_micros(5).await;
        }
        Err(Error::TimeoutWaitNotBusy)
    }
}

// number of bits for cmd responses
#[derive(Copy, Clone)]
enum CmdResponseLen {
    Short = 48,
    Long = 148,
}
