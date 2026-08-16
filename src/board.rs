//! RAK3172 board initialization and BME688 bus diagnostics.

use bme68x::interface::I2cAddress;
use defmt::{error, warn};
use embassy_stm32::gpio::{Input, Pull};
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::time::Hertz;
use embassy_stm32::{Peri, peripherals};
use embassy_time::{Duration, Timer};
use embedded_hal::i2c::{ErrorType, I2c as EmbeddedI2c, Operation};

const SENSOR_STARTUP_DELAY: Duration = Duration::from_millis(10);
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
}

impl Board {
    /// Transfer ownership of the BME688 bus to the sensor workflow.
    pub fn into_sensor_bus(self) -> SensorBus {
        self.sensor_bus
    }
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
    let mut config = embassy_stm32::Config::default();

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

    Board { sensor_bus }
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
pub async fn halt_after_probe_failure(failure: ProbeFailure) -> ! {
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
        Timer::after(FAILURE_REPORT_INTERVAL).await;
        error!("BME688 unavailable; check the reported SCL/SDA levels and reset the board");
    }
}
