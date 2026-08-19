//! RAK3172-T internal STM32WL radio integration.
//!
//! The STM32WL interface glue is adapted from the MIT OR Apache-2.0 lora-rs
//! example at `lora-phy-v3.0.1/examples/stm32wl/src/iv.rs`:
//! <https://github.com/lora-rs/lora-rs/blob/lora-phy-v3.0.1/examples/stm32wl/src/iv.rs>
//!
//! RAK3172-T RF-switch, PA, TCXO, and DCDC facts come from RAK's official BSP:
//! <https://github.com/RAKWireless/RAK-STM32-RUI/tree/main/variants/WisDuo_RAK3172-T_Board>
//!
//! Session teardown mirrors ST's `HAL_SUBGHZ_DeInit` by reasserting the radio
//! reset after use:
//! <https://github.com/STMicroelectronics/stm32wlxx-hal-driver/blob/5c87cf00992c6e4ecf56c7c129b8dcfc6aa6f88e/Src/stm32wlxx_hal_subghz.c>
//!
//! SUBGHZSPI/DMA drivers are constructed for one measurement batch and dropped
//! afterward. RF-switch GPIO outputs persist low between batches; GPIO output
//! ownership has no RCC STOP2 constraint and prevents analog/high-Z idle.

use embassy_stm32::bind_interrupts;
use embassy_stm32::dma;
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::InterruptExt;
use embassy_stm32::pac;
use embassy_stm32::peripherals;
use embassy_stm32::spi::Spi;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Delay, Duration, Instant, Timer, with_timeout};
use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::{ErrorType, Operation, SpiBus, SpiDevice};
use lora_phy::DelayNs;
use lora_phy::LoRa;
use lora_phy::mod_params::{Bandwidth, CodingRate, RadioError, SpreadingFactor};
use lora_phy::mod_traits::InterfaceVariant;
use lora_phy::sx126x::{self, Stm32wl, Sx126x, TcxoCtrlVoltage};

use crate::board::RadioResources;
use crate::radio_config::{FREQUENCY_HZ, PREAMBLE_SYMBOLS, TX_POWER_DBM};
use crate::rf_switch::FailSafeRfOutput;

const RADIO_BUSY_TIMEOUT: Duration = Duration::from_millis(100);
const TX_COMPLETION_TIMEOUT: Duration = Duration::from_secs(2);

bind_interrupts!(struct Irqs {
    DMA1_CHANNEL1 => dma::InterruptHandler<peripherals::DMA1_CH1>;
    DMA1_CHANNEL2 => dma::InterruptHandler<peripherals::DMA1_CH2>;
    SUBGHZ_RADIO => RadioInterruptHandler;
});

static IRQ_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

pub struct RadioInterruptHandler;

impl interrupt::typelevel::Handler<interrupt::typelevel::SUBGHZ_RADIO> for RadioInterruptHandler {
    unsafe fn on_interrupt() {
        interrupt::SUBGHZ_RADIO.disable();
        IRQ_SIGNAL.signal(());
    }
}

/// Batch failure plus the exact number of packets whose TX-completion IRQ was
/// observed before the failure. Callers can account for unsent fragments
/// without assuming an all-or-nothing radio batch.
pub struct TransmissionError {
    source: RadioError,
    completed_packets: u8,
}

impl TransmissionError {
    #[cfg(feature = "profile-v2")]
    #[must_use]
    pub const fn source(&self) -> &RadioError {
        &self.source
    }

    #[must_use]
    pub const fn completed_packets(&self) -> u8 {
        self.completed_packets
    }
}

/// Transmit one batch of private-network raw LoRa packets, put the radio to
/// sleep, drop the concrete radio driver, and restore persistent switch outputs
/// low so the next timer wait can reach STOP2 safely.
#[cfg(feature = "telemetry-v1")]
pub async fn transmit<const N: usize>(
    resources: &mut RadioResources,
    payloads: &[[u8; N]],
) -> Result<(), RadioError> {
    let mut slices = [&[][..]; 3];
    if payloads.len() > slices.len() {
        return Err(RadioError::PayloadSizeUnexpected(payloads.len()));
    }
    for (slot, payload) in slices.iter_mut().zip(payloads) {
        *slot = payload;
    }
    transmit_payloads(resources, &slices[..payloads.len()])
        .await
        .map(|_| ())
        .map_err(|error| {
            let _completed_packets = error.completed_packets();
            error.source
        })
}

/// Transmit variable-length payloads in one cold radio session.
///
/// This is the profile-v2 hook. Each successful packet increments the returned
/// completion count only after the radio TX-done IRQ is observed.
pub async fn transmit_payloads(
    resources: &mut RadioResources,
    payloads: &[&[u8]],
) -> Result<u8, TransmissionError> {
    if payloads.is_empty() {
        return Ok(0);
    }

    // Locals drop in reverse declaration order. Declaring this first guarantees
    // it runs after `lora` has released SUBGHZSPI and both borrowed RF-switch
    // wrappers have restored their persistent outputs low.
    let _session_guard = RadioSessionGuard;

    let rf_switch_rx = FailSafeRfOutput::new(&mut resources.rf_switch.rx);
    let rf_switch_tx = FailSafeRfOutput::new(&mut resources.rf_switch.tx);

    let spi = Spi::new_subghz(
        resources.peripheral.reborrow(),
        resources.tx_dma.reborrow(),
        resources.rx_dma.reborrow(),
        Irqs,
    );
    let spi = SubghzSpiDevice(spi);
    let interface = Stm32wlInterfaceVariant::new(Irqs, Some(rf_switch_rx), Some(rf_switch_tx))
        .map_err(|source| TransmissionError {
            source,
            completed_packets: 0,
        })?;
    let config = sx126x::Config {
        // RAK3172 routes only the STM32WL high-power RFO internally.
        chip: Stm32wl {
            use_high_power_pa: true,
        },
        // RAK3172-T official board support specifies a 3.0 V TCXO control.
        tcxo_ctrl: Some(TcxoCtrlVoltage::Ctrl3V0),
        use_dcdc: true,
        rx_boost: false,
    };
    // `false` selects the private 0x1424 sync word used by raw P2P.
    let mut lora = LoRa::new(Sx126x::new(spi, interface, config), false, Delay)
        .await
        .map_err(|source| TransmissionError {
            source,
            completed_packets: 0,
        })?;

    let modulation = match lora.create_modulation_params(
        SpreadingFactor::_7,
        Bandwidth::_125KHz,
        CodingRate::_4_5,
        FREQUENCY_HZ,
    ) {
        Ok(value) => value,
        Err(error) => {
            let _ = lora.sleep(false).await;
            return Err(TransmissionError {
                source: error,
                completed_packets: 0,
            });
        }
    };
    let mut packet = match lora.create_tx_packet_params(
        PREAMBLE_SYMBOLS,
        false, // explicit header
        true,  // PHY CRC
        false, // normal/non-inverted IQ
        &modulation,
    ) {
        Ok(value) => value,
        Err(error) => {
            let _ = lora.sleep(false).await;
            return Err(TransmissionError {
                source: error,
                completed_packets: 0,
            });
        }
    };

    let mut operation_result = Ok(());
    let mut completed_packets = 0_u8;
    for payload in payloads {
        operation_result = match lora
            .prepare_for_tx(&modulation, &mut packet, TX_POWER_DBM, payload)
            .await
        {
            Ok(()) => lora.tx().await,
            Err(error) => Err(error),
        };

        if operation_result.is_err() {
            break;
        }
        completed_packets = completed_packets.saturating_add(1);
    }
    let sleep_result = lora.sleep(false).await;

    // On return, `lora` releases SUBGHZSPI and the borrowed fail-safe wrappers
    // drive both persistent outputs low. The earlier-declared session guard
    // then runs last and holds the radio reset.
    if let Err(source) = operation_result {
        return Err(TransmissionError {
            source,
            completed_packets,
        });
    }
    if let Err(source) = sleep_result {
        return Err(TransmissionError {
            source,
            completed_packets,
        });
    }
    Ok(completed_packets)
}

/// Last-drop cleanup for every transmit-session exit, including initialization,
/// configuration, transmit, and sleep errors.
///
/// The next session's [`InterfaceVariant::reset`] sequence releases `RFRST`.
struct RadioSessionGuard;

impl Drop for RadioSessionGuard {
    fn drop(&mut self) {
        interrupt::SUBGHZ_RADIO.disable();
        interrupt::SUBGHZ_RADIO.unpend();
        IRQ_SIGNAL.reset();
        pac::RCC.csr().modify(|register| register.set_rfrst(true));
    }
}

/// STM32WL's internal SUBGHZ NSS is controlled through PWR, not a GPIO pin.
struct SubghzSpiDevice<T>(T);

impl<T: SpiBus> ErrorType for SubghzSpiDevice<T> {
    type Error = T::Error;
}

impl<T: SpiBus> SpiDevice for SubghzSpiDevice<T> {
    async fn transaction(
        &mut self,
        operations: &mut [Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        pac::PWR.subghzspicr().modify(|w| w.set_nss(false));

        let operation_result = 'operations: {
            for operation in operations {
                let result = match operation {
                    Operation::Read(buffer) => self.0.read(buffer).await,
                    Operation::Write(buffer) => self.0.write(buffer).await,
                    Operation::Transfer(read, write) => self.0.transfer(read, write).await,
                    Operation::TransferInPlace(buffer) => self.0.transfer_in_place(buffer).await,
                    Operation::DelayNs(nanoseconds) => match self.0.flush().await {
                        Ok(()) => {
                            Timer::after_nanos(u64::from(*nanoseconds)).await;
                            Ok(())
                        }
                        Err(error) => Err(error),
                    },
                };
                if let Err(error) = result {
                    break 'operations Err(error);
                }
            }
            Ok(())
        };

        // Even failed transfers must flush and deassert the internal NSS.
        let flush_result = self.0.flush().await;
        pac::PWR.subghzspicr().modify(|w| w.set_nss(true));
        operation_result?;
        flush_result
    }
}

/// Glue between lora-phy and STM32WL's internal radio IRQ/RF-switch wiring.
struct Stm32wlInterfaceVariant<CTRL> {
    rf_switch_rx: Option<CTRL>,
    rf_switch_tx: Option<CTRL>,
}

impl<CTRL> Stm32wlInterfaceVariant<CTRL>
where
    CTRL: OutputPin,
{
    fn new(
        _irq: impl interrupt::typelevel::Binding<
            interrupt::typelevel::SUBGHZ_RADIO,
            RadioInterruptHandler,
        > + 'static,
        rf_switch_rx: Option<CTRL>,
        rf_switch_tx: Option<CTRL>,
    ) -> Result<Self, RadioError> {
        interrupt::SUBGHZ_RADIO.disable();
        interrupt::SUBGHZ_RADIO.unpend();
        IRQ_SIGNAL.reset();
        Ok(Self {
            rf_switch_rx,
            rf_switch_tx,
        })
    }
}

impl<CTRL> InterfaceVariant for Stm32wlInterfaceVariant<CTRL>
where
    CTRL: OutputPin,
{
    async fn reset(&mut self, _delay: &mut impl DelayNs) -> Result<(), RadioError> {
        pac::RCC.csr().modify(|w| w.set_rfrst(true));
        pac::RCC.csr().modify(|w| w.set_rfrst(false));
        Ok(())
    }

    async fn wait_on_busy(&mut self) -> Result<(), RadioError> {
        let deadline = Instant::now() + RADIO_BUSY_TIMEOUT;
        while pac::PWR.sr2().read().rfbusys() {
            if Instant::now() >= deadline {
                return Err(RadioError::Busy);
            }
            // Avoid a non-yielding spin if the radio or its clock is faulty.
            Timer::after_micros(20).await;
        }
        Ok(())
    }

    async fn await_irq(&mut self) -> Result<(), RadioError> {
        // Bound only the cancel-safe signal wait. Do not wrap `LoRa::tx()` as a
        // whole: cancelling while it processes an IRQ could interrupt an SPI
        // transaction and leave the radio state machine inconsistent.
        IRQ_SIGNAL.reset();
        unsafe { interrupt::SUBGHZ_RADIO.enable() };
        match with_timeout(TX_COMPLETION_TIMEOUT, IRQ_SIGNAL.wait()).await {
            Ok(()) => Ok(()),
            Err(_) => {
                interrupt::SUBGHZ_RADIO.disable();
                Err(RadioError::TransmitTimeout)
            }
        }
    }

    async fn enable_rf_switch_rx(&mut self) -> Result<(), RadioError> {
        if let Some(pin) = &mut self.rf_switch_tx {
            pin.set_low().map_err(|_| RadioError::RfSwitchTx)?;
        }
        if let Some(pin) = &mut self.rf_switch_rx {
            pin.set_high().map_err(|_| RadioError::RfSwitchRx)?;
        }
        Ok(())
    }

    async fn enable_rf_switch_tx(&mut self) -> Result<(), RadioError> {
        if let Some(pin) = &mut self.rf_switch_rx {
            pin.set_low().map_err(|_| RadioError::RfSwitchRx)?;
        }
        if let Some(pin) = &mut self.rf_switch_tx {
            pin.set_high().map_err(|_| RadioError::RfSwitchTx)?;
        }
        Ok(())
    }

    async fn disable_rf_switch(&mut self) -> Result<(), RadioError> {
        if let Some(pin) = &mut self.rf_switch_rx {
            pin.set_low().map_err(|_| RadioError::RfSwitchRx)?;
        }
        if let Some(pin) = &mut self.rf_switch_tx {
            pin.set_low().map_err(|_| RadioError::RfSwitchTx)?;
        }
        Ok(())
    }
}
