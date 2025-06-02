use defmt::{Format, info, trace};
use embassy_rp::Peri;
use embassy_rp::dma::Channel;
use embassy_rp::pio::{Common, Instance, Irq, PioPin, StateMachine};
use embassy_time::{Duration, Timer};
use embedded_sdmmc::sdcard::CardType;
use embedded_sdmmc::sdcard::proto::*;
use embedded_sdmmc::{Block, BlockIdx};

use crate::sdio::{PioSdio, PioSdioClk, PioSdioCmd, SdioError};

const SHORT_CMD_RESP: u8 = 48;
const LONG_CMD_RESP: u8 = 136;

/// The possible errors this crate can generate.
#[derive(Debug, Copy, Clone, Format)]
pub enum Error {
    /// We got an error from the Pio state machine
    Transport(SdioError),
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
    /// Can't perform this operation with the card in this state
    BadState,
    /// Couldn't find the card
    CardNotFound,
}

pub struct PioSd<'d, PIO: Instance, const SM0: usize, const SM1: usize, const SM2: usize> {
    bus: PioSdio<'d, PIO, SM0, SM1, SM2>,
    card_type: Option<CardType>,
    rca: Option<u16>,
}

impl<'d, PIO: Instance, const SM0: usize, const SM1: usize, const SM2: usize>
    PioSd<'d, PIO, SM0, SM1, SM2>
{
    pub fn new(sdio: PioSdio<'d, PIO, SM0, SM1, SM2>) -> Self {
        Self {
            rca: None,
            card_type: None,
            bus: sdio,
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
        let mut cmd = [
            0x40 | command,
            (arg >> 24) as u8,
            (arg >> 16) as u8,
            (arg >> 8) as u8,
            arg as u8,
            0,
        ];
        cmd[5] = crc7(&cmd[0..5]);
        info!("TX 0x{:X}: {:#04X}", command, cmd);

        let timeout = Duration::from_millis(match command {
            CMD0 => 0,
            ACMD41 => 1000,
            _ => 100,
        });
        self.bus
            .command(&cmd, read_buf, timeout)
            .await
            .map_err(Error::Transport)?;

        if !read_buf.is_empty() {
            info!("RX: {:#02X}", read_buf);
        }

        Ok(())
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
        Timer::after(Duration::from_hz(100_000) * 100).await;

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
        Err(Error::BadState)
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
}
