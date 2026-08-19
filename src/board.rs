//! RAK3172 board initialization and BME688 bus diagnostics.

use bme68x::interface::I2cAddress;
use defmt::{error, warn};
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::i2c::{self, I2c};
#[cfg(feature = "profile-v2")]
use embassy_stm32::pac;
use embassy_stm32::time::Hertz;
use embassy_stm32::{Peri, peripherals};
use embassy_time::{Duration, Timer};
use embedded_hal::i2c::{ErrorType, I2c as EmbeddedI2c, Operation};

const SENSOR_STARTUP_DELAY: Duration = Duration::from_millis(10);
#[cfg(feature = "telemetry-v1")]
const FAILURE_REPORT_INTERVAL: Duration = Duration::from_secs(5);

/// The board resources needed to create a short-lived BME688 I2C connection.
///
/// Each embedded-hal transaction temporarily configures and enables I2C2, then
/// drops it after the transaction attempt returns. Keeping only these ownership
/// tokens between transactions lets Embassy select STOP2 during the interval.
pub struct SensorBus {
    peripheral: Peri<'static, peripherals::I2C2>,
    scl: Peri<'static, peripherals::PA12>,
    sda: Peri<'static, peripherals::PA11>,
    config: i2c::Config,
}

impl SensorBus {
    /// Sample the physical bus levels without enabling MCU pull-ups.
    fn line_levels(&mut self) -> (bool, bool) {
        let scl_high = {
            let scl = Input::new(self.scl.reborrow(), Pull::None);
            scl.is_high()
        };
        let sda_high = {
            let sda = Input::new(self.sda.reborrow(), Pull::None);
            sda.is_high()
        };

        (scl_high, sda_high)
    }
}

impl ErrorType for SensorBus {
    type Error = i2c::Error;
}

impl EmbeddedI2c for SensorBus {
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        let Self {
            peripheral,
            scl,
            sda,
            config,
        } = self;
        let mut bus = I2c::new_blocking(
            peripheral.reborrow(),
            scl.reborrow(),
            sda.reborrow(),
            *config,
        );
        let result = bus.blocking_transaction(address, operations);

        // Dropping the concrete driver disables I2C2, releases PA12/PA11 back
        // to analog mode, and removes I2C2's STOP2 RCC constraint.
        drop(bus);
        result
    }
}

/// Initialized board resources owned by the application.
pub struct Board {
    sensor_bus: SensorBus,
    #[cfg(not(feature = "profile-v2-uart"))]
    radio: RadioResources,
    #[cfg(feature = "profile-v2")]
    rng: Peri<'static, peripherals::RNG>,
    #[cfg(feature = "profile-v2-uart")]
    training_uart: TrainingUartResources,
    #[cfg(feature = "profile-v2")]
    reset_cause: ResetCause,
}

impl Board {
    /// Split the independent sensor and radio resources between their owners.
    #[cfg(feature = "telemetry-v1")]
    pub fn into_parts(self) -> (SensorBus, RadioResources) {
        (self.sensor_bus, self.radio)
    }

    /// Split profile-v2 resources without constructing any concrete peripheral
    /// driver that would prevent STOP2 later.
    #[cfg(feature = "profile-v2")]
    pub fn into_profile_parts(self) -> ProfileBoardParts {
        ProfileBoardParts {
            sensor_bus: self.sensor_bus,
            #[cfg(not(feature = "profile-v2-uart"))]
            radio: self.radio,
            rng: self.rng,
            #[cfg(feature = "profile-v2-uart")]
            training_uart: self.training_uart,
            reset_cause: self.reset_cause,
        }
    }
}

/// Profile-v2 ownership tokens and reset metadata captured before RCC init.
#[cfg(feature = "profile-v2")]
pub struct ProfileBoardParts {
    pub sensor_bus: SensorBus,
    #[cfg(not(feature = "profile-v2-uart"))]
    pub radio: RadioResources,
    pub rng: Peri<'static, peripherals::RNG>,
    #[cfg(feature = "profile-v2-uart")]
    pub training_uart: TrainingUartResources,
    pub reset_cause: ResetCause,
}

/// Radio/DMA tokens plus persistent-low RF-switch outputs retained between
/// transmissions. Only concrete SUBGHZSPI/DMA drivers are session-scoped.
#[cfg(not(feature = "profile-v2-uart"))]
pub struct RadioResources {
    pub(crate) peripheral: Peri<'static, peripherals::SUBGHZSPI>,
    pub(crate) tx_dma: Peri<'static, peripherals::DMA1_CH1>,
    pub(crate) rx_dma: Peri<'static, peripherals::DMA1_CH2>,
    /// Persistent push-pull outputs keep the PE4259 controls at OFF=0/0 in
    /// STOP2. Merely retaining an Embassy GPIO `Output` adds no RCC stop-mode
    /// constraint; the pins keep their programmed mode and output latch.
    pub(crate) rf_switch: RfSwitchResources,
}

/// Persistent push-pull ownership of both RAK3172-T RF-switch controls.
///
/// These outputs are retained even by the UART-only training build. Leaving
/// PB8/PC13 in their reset analog/high-impedance state would not be a documented
/// PE4259 OFF state and could create an indeterminate RF/current condition.
pub struct RfSwitchResources {
    pub(crate) rx: Output<'static>,
    pub(crate) tx: Output<'static>,
}

/// USART2/PA2 ownership retained between short, blocking training writes.
///
/// PA2 is routed to the board's debug-header UART TX signal. No RX pin or DMA
/// channel is needed for the unidirectional laboratory stream.
#[cfg(feature = "profile-v2-uart")]
pub struct TrainingUartResources {
    pub(crate) peripheral: Peri<'static, peripherals::USART2>,
    pub(crate) tx: Peri<'static, peripherals::PA2>,
    /// No SUBGHZSPI/DMA tokens exist in this build, but the physical switch is
    /// still held at its documented OFF=0/0 state for the program lifetime.
    pub(crate) rf_switch: RfSwitchResources,
}

/// STM32 reset status captured before Embassy changes or clears RCC state.
#[cfg(feature = "profile-v2")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ResetCause {
    /// Exact 32-bit RCC CSR snapshot. Low bits include clock/radio state in
    /// addition to the reset flags in bits 24..31.
    pub raw: u32,
    /// Stable protocol flags: radio-illegal, option-byte, pin, BOR, software,
    /// independent-watchdog, window-watchdog, and low-power reset.
    pub flags: u16,
}

/// A responsive BME68x and the bus on which it was found.
pub struct SensorProbe {
    bus: SensorBus,
    address: I2cAddress,
    chip_id: u8,
}

/// Failed address probes together with ownership of the board's bus resources.
pub struct ProbeFailure {
    bus: SensorBus,
    low_error: i2c::Error,
    high_error: i2c::Error,
}

impl ProbeFailure {
    pub(crate) fn into_parts(self) -> (SensorBus, i2c::Error, i2c::Error) {
        (self.bus, self.low_error, self.high_error)
    }
}

impl SensorProbe {
    pub fn address_byte(&self) -> u8 {
        self.address.into()
    }

    pub const fn chip_id(&self) -> u8 {
        self.chip_id
    }

    pub(crate) fn into_parts(self) -> (SensorBus, I2cAddress) {
        (self.bus, self.address)
    }
}

/// Initialize the STM32WLE5 and the board's sensor bus.
pub fn init() -> Board {
    #[cfg(feature = "profile-v2")]
    let reset_cause = capture_and_clear_reset_cause();

    let mut config = embassy_stm32::Config::default();

    // Run from the internal MSI at the STM32WLE5's supported 48 MHz ceiling.
    // This avoids depending on the radio TCXO for the MCU system clock and lets
    // the wake/measure/transmit work complete quickly before returning to STOP2.
    config.rcc.msi = Some(embassy_stm32::rcc::MSIRange::Range48m);
    config.rcc.sys = embassy_stm32::rcc::Sysclk::Msi;
    #[cfg(feature = "profile-v2")]
    {
        // The boot nonce is generated once from hardware RNG, then the RNG
        // driver is dropped so its Stop1 RCC constraint cannot affect idle.
        config.rcc.mux.rngsel = embassy_stm32::rcc::mux::Rngsel::Msi;
    }

    // Real STOP mode is the production default. The opt-in debug feature keeps
    // SWD/RTT available by substituting shallow sleep for STOP.
    config.enable_debug_during_sleep = cfg!(feature = "debug-sleep");
    // Only pay the STOP entry/exit overhead when the next wake-up is at least
    // this far away. It remains far shorter than the sampling interval.
    config.min_stop_pause = Duration::from_millis(250);

    let peripherals = embassy_stm32::init(config);

    // RAK3172 board wiring: BME688 SCL=PA12 and SDA=PA11 on I2C2. The PCB has
    // external 10 kohm pull-ups, validated by the earlier C bring-up.
    let mut i2c_config = i2c::Config::default();
    i2c_config.frequency = Hertz::khz(100);
    i2c_config.scl_pullup = false;
    i2c_config.sda_pullup = false;

    let sensor_bus = SensorBus {
        peripheral: peripherals.I2C2,
        scl: peripherals.PA12,
        sda: peripherals.PA11,
        config: i2c_config,
    };

    // Always claim PB8/PC13 as persistent push-pull outputs. The selected
    // transport resource owns them for the rest of the program, including
    // STOP2 intervals and the UART-only training mode.
    let rf_switch = RfSwitchResources {
        // RAK3172-T internal RF switch: PB8=RX enable, PC13=TX enable.
        rx: Output::new(peripherals.PB8, Level::Low, Speed::High),
        tx: Output::new(peripherals.PC13, Level::Low, Speed::High),
    };

    #[cfg(not(feature = "profile-v2-uart"))]
    let radio = RadioResources {
        peripheral: peripherals.SUBGHZSPI,
        tx_dma: peripherals.DMA1_CH1,
        rx_dma: peripherals.DMA1_CH2,
        rf_switch,
    };

    Board {
        sensor_bus,
        #[cfg(not(feature = "profile-v2-uart"))]
        radio,
        #[cfg(feature = "profile-v2")]
        rng: peripherals.RNG,
        #[cfg(feature = "profile-v2-uart")]
        training_uart: TrainingUartResources {
            peripheral: peripherals.USART2,
            tx: peripherals.PA2,
            rf_switch,
        },
        #[cfg(feature = "profile-v2")]
        reset_cause,
    }
}

#[cfg(feature = "profile-v2")]
fn capture_and_clear_reset_cause() -> ResetCause {
    let status = pac::RCC.csr().read();
    let raw = status.0;
    let flags = u16::from(status.rfilarstf())
        | (u16::from(status.oblrstf()) << 1)
        | (u16::from(status.pinrstf()) << 2)
        | (u16::from(status.borrstf()) << 3)
        | (u16::from(status.sftrstf()) << 4)
        | (u16::from(status.iwdgrstf()) << 5)
        | (u16::from(status.wwdgrstf()) << 6)
        | (u16::from(status.lpwrrstf()) << 7);

    // Preserve the captured value, then clear sticky reset flags so a later
    // reset can be interpreted independently.
    pac::RCC.csr().modify(|register| register.set_rmvf(true));

    ResetCause { raw, flags }
}

/// Find the BME688 while preserving the bus on failure for diagnostics/recovery.
pub async fn probe_bme688(mut bus: SensorBus) -> Result<SensorProbe, ProbeFailure> {
    // Make a VOUT power cycle deterministic when reset and target power rise at
    // nearly the same time.
    Timer::after(SENSOR_STARTUP_DELAY).await;

    let mut chip_id = [0_u8];
    let address = match bus.write_read(0x76_u8, &[0xd0], &mut chip_id) {
        Ok(()) => I2cAddress::Low,
        Err(low_error) => {
            let mut high_address_chip_id = [0_u8];
            match bus.write_read(0x77_u8, &[0xd0], &mut high_address_chip_id) {
                Ok(()) => {
                    chip_id = high_address_chip_id;
                    warn!("BME68x answered at diagnostic address 0x77, not wired address 0x76");
                    I2cAddress::High
                }
                Err(high_error) => {
                    return Err(ProbeFailure {
                        bus,
                        low_error,
                        high_error,
                    });
                }
            }
        }
    };

    Ok(SensorProbe {
        bus,
        address,
        chip_id: chip_id[0],
    })
}

/// Report a failed probe and retain the bus resources without further traffic.
#[cfg(feature = "telemetry-v1")]
pub async fn halt_after_probe_failure(failure: ProbeFailure, radio: RadioResources) -> ! {
    let (mut bus, low_error, high_error) = failure.into_parts();
    // Allow the external pull-ups and any failed target transfer to settle
    // before temporarily enabling the GPIO input buffers for diagnosis.
    Timer::after(Duration::from_millis(1)).await;
    let (scl_high, sda_high) = bus.line_levels();
    error!(
        "BME688 I2C preflight failed: error_0x76={:?}, error_0x77={:?}, SCL(PA12)_high={}, SDA(PA11)_high={}",
        low_error, high_error, scl_high, sda_high
    );

    // Retain the peripheral and pin tokens. No concrete I2C driver exists, so
    // I2C2 stays disabled and no more traffic can be emitted.
    loop {
        let _keep_bus_and_pins_alive = &bus;
        let _keep_rf_switch_outputs_low = &radio;
        Timer::after(FAILURE_REPORT_INTERVAL).await;
        error!("BME688 unavailable; check the reported SCL/SDA levels and reset the board");
    }
}

/// Log both concrete address errors and electrical line levels, then return
/// the bus ownership tokens for the profile-v2 degraded-health loop.
#[cfg(feature = "profile-v2")]
pub async fn diagnose_probe_failure(failure: ProbeFailure) -> SensorBus {
    let (mut bus, low_error, high_error) = failure.into_parts();
    Timer::after(Duration::from_millis(1)).await;
    let (scl_high, sda_high) = bus.line_levels();
    error!(
        "BME688 I2C preflight failed: error_0x76={:?}, error_0x77={:?}, SCL(PA12)_high={}, SDA(PA11)_high={}",
        low_error, high_error, scl_high, sda_high
    );
    bus
}
