//! Replaceable profile-v2 transport boundary.
//!
//! Production uses one variable-length LoRa batch. Laboratory training uses
//! the same protocol records over COBS-delimited USART2/PA2, so transport does
//! not change the decoder or sensor semantics.

use vesta_protocol_v2::v2::{EncodedFrame, EncodedProfile};

#[cfg(not(feature = "profile-v2-uart"))]
use crate::board::RadioResources;
#[cfg(feature = "profile-v2-uart")]
use crate::board::TrainingUartResources;
#[cfg(feature = "profile-v2-uart")]
use crate::framing::{self, MAX_COBS_FRAME_LEN};
#[cfg(not(feature = "profile-v2-uart"))]
use crate::radio;

// One config + four 3/3/3/1 profile fragments + one health record.
const MAX_RECORDS_PER_SCAN: usize = 6;
#[cfg(feature = "profile-v2-uart")]
pub const TRAINING_UART_BAUD: u32 = 115_200;
#[cfg(feature = "profile-v2-uart")]
const _: () = assert!(vesta_protocol_v2::v2::MAX_V2_FRAME_LEN <= framing::MAX_PROTOCOL_FRAME_LEN);

pub struct EmitReport {
    pub completed_records: u8,
    pub requested_records: u8,
}

pub struct EmitError {
    pub completed_records: u8,
    pub requested_records: u8,
    pub source: Source,
}

pub enum Source {
    TooManyRecords,
    #[cfg(not(feature = "profile-v2-uart"))]
    Radio(radio::TransmissionError),
    #[cfg(feature = "profile-v2-uart")]
    UartConfiguration(embassy_stm32::usart::ConfigError),
    #[cfg(feature = "profile-v2-uart")]
    UartWrite(embassy_stm32::usart::Error),
    #[cfg(feature = "profile-v2-uart")]
    Framing(framing::Error),
}

pub struct ProfileOutput {
    #[cfg(not(feature = "profile-v2-uart"))]
    radio: RadioResources,
    #[cfg(feature = "profile-v2-uart")]
    uart: TrainingUartResources,
}

impl ProfileOutput {
    #[cfg(not(feature = "profile-v2-uart"))]
    pub const fn new(radio: RadioResources) -> Self {
        Self { radio }
    }

    #[cfg(feature = "profile-v2-uart")]
    pub const fn new(uart: TrainingUartResources) -> Self {
        Self { uart }
    }

    /// Emit optional repeated config, every profile fragment, and optional
    /// health as one transport batch in deterministic record order.
    pub async fn emit(
        &mut self,
        config: Option<&EncodedFrame>,
        profile: &EncodedProfile,
        health: Option<&EncodedFrame>,
    ) -> Result<EmitReport, EmitError> {
        let mut payloads = [&[][..]; MAX_RECORDS_PER_SCAN];
        let mut count = 0_usize;

        if let Some(config) = config {
            push_payload(&mut payloads, &mut count, config.as_slice())?;
        }
        for fragment in profile.frames() {
            push_payload(&mut payloads, &mut count, fragment.as_slice())?;
        }
        if let Some(health) = health {
            push_payload(&mut payloads, &mut count, health.as_slice())?;
        }

        self.emit_payloads(&payloads[..count]).await
    }

    /// Emit one health record without inventing a profile or configuration.
    pub async fn emit_health(&mut self, health: &EncodedFrame) -> Result<EmitReport, EmitError> {
        self.emit_payloads(&[health.as_slice()]).await
    }

    async fn emit_payloads(&mut self, payloads: &[&[u8]]) -> Result<EmitReport, EmitError> {
        let count = payloads.len();

        let requested_records = u8::try_from(count).unwrap_or(u8::MAX);

        #[cfg(not(feature = "profile-v2-uart"))]
        {
            return match radio::transmit_payloads(&mut self.radio, payloads).await {
                Ok(completed_records) => Ok(EmitReport {
                    completed_records,
                    requested_records,
                }),
                Err(source) => Err(EmitError {
                    completed_records: source.completed_packets(),
                    requested_records,
                    source: Source::Radio(source),
                }),
            };
        }

        #[cfg(feature = "profile-v2-uart")]
        {
            use embassy_stm32::usart::{Config, DataBits, Parity, StopBits, UartTx};

            // `self.uart.rf_switch` deliberately remains owned and low/low
            // throughout this short UART session and the following STOP2
            // interval. Only USART2 is temporarily instantiated here.
            let _keep_rf_switch_outputs_low = (&self.uart.rf_switch.rx, &self.uart.rf_switch.tx);
            let mut uart_config = Config::default();
            uart_config.baudrate = TRAINING_UART_BAUD;
            uart_config.data_bits = DataBits::DataBits8;
            uart_config.parity = Parity::ParityNone;
            uart_config.stop_bits = StopBits::STOP1;
            let mut uart = UartTx::new_blocking(
                self.uart.peripheral.reborrow(),
                self.uart.tx.reborrow(),
                uart_config,
            )
            .map_err(|source| EmitError {
                completed_records: 0,
                requested_records,
                source: Source::UartConfiguration(source),
            })?;

            let mut completed_records = 0_u8;
            let mut encoded = [0_u8; MAX_COBS_FRAME_LEN];
            for payload in payloads {
                let encoded_len =
                    framing::encode_delimited(payload, &mut encoded).map_err(|source| {
                        EmitError {
                            completed_records,
                            requested_records,
                            source: Source::Framing(source),
                        }
                    })?;
                uart.blocking_write(&encoded[..encoded_len])
                    .map_err(|source| EmitError {
                        completed_records,
                        requested_records,
                        source: Source::UartWrite(source),
                    })?;
                uart.blocking_flush().map_err(|source| EmitError {
                    completed_records,
                    requested_records,
                    source: Source::UartWrite(source),
                })?;
                completed_records = completed_records.saturating_add(1);
            }

            // Dropping UartTx disables USART2; the retained ownership tokens do
            // not impose a STOP-mode RCC constraint between scans.
            drop(uart);
            Ok(EmitReport {
                completed_records,
                requested_records,
            })
        }
    }
}

fn push_payload<'a>(
    payloads: &mut [&'a [u8]; MAX_RECORDS_PER_SCAN],
    count: &mut usize,
    payload: &'a [u8],
) -> Result<(), EmitError> {
    let requested_records = u8::try_from(count.saturating_add(1)).unwrap_or(u8::MAX);
    let Some(slot) = payloads.get_mut(*count) else {
        return Err(EmitError {
            completed_records: 0,
            requested_records,
            source: Source::TooManyRecords,
        });
    };
    *slot = payload;
    *count += 1;
    Ok(())
}
