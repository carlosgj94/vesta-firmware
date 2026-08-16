//! BME688 measurement policy for the Vesta application.

use bme68x::blocking::Bme68x;
use bme68x::interface::I2cInterface;
use bme68x::{
    Configuration, Error as Bme68xError, Filter, HeaterConfiguration, Measurements, OperationMode,
    Oversampling, StandbyTime, Variant,
};
use embassy_stm32::i2c;
use embassy_time::{Delay, Duration, Timer};

use crate::board::{SensorBus, SensorProbe};

pub const HEATER_TEMPERATURE_CELSIUS: u16 = 300;
pub const HEATER_DURATION_MS: u16 = 100;
const MEASUREMENT_MARGIN_US: u64 = 5_000;

type Driver = Bme68x<I2cInterface<SensorBus>, Delay>;
pub type DriverError = Bme68xError<i2c::Error>;

/// The driver operation that produced an error.
#[derive(Clone, Copy)]
pub enum Operation {
    Initialization,
    Configuration,
    HeaterConfiguration,
    ForcedModeTrigger,
    DataRead,
}

impl Operation {
    pub const fn label(self) -> &'static str {
        match self {
            Self::Initialization => "initialization",
            Self::Configuration => "configuration",
            Self::HeaterConfiguration => "heater configuration",
            Self::ForcedModeTrigger => "forced-mode trigger",
            Self::DataRead => "data read",
        }
    }
}

/// A BME688 failure paired with the operation that failed.
pub struct SensorError {
    operation: Operation,
    source: DriverError,
}

/// A configuration failure that retains the initialized sensor and its bus.
pub struct SetupFailure {
    sensor: Sensor,
    error: SensorError,
}

impl SetupFailure {
    pub(crate) fn into_parts(self) -> (Sensor, SensorError) {
        (self.sensor, self.error)
    }
}

impl SensorError {
    const fn new(operation: Operation, source: DriverError) -> Self {
        Self { operation, source }
    }

    pub const fn operation(&self) -> Operation {
        self.operation
    }

    pub const fn source(&self) -> &DriverError {
        &self.source
    }
}

/// Owned BME688 workflow used by this application.
pub struct Sensor {
    driver: Driver,
    address: u8,
    conversion_wait: Duration,
}

impl Sensor {
    /// Initialize the sensor discovered during board preflight.
    pub fn new(probe: SensorProbe) -> Result<Self, SensorError> {
        let address = probe.address_byte();
        let (bus, i2c_address) = probe.into_parts();
        let interface = I2cInterface::new(bus, i2c_address);
        let driver = Bme68x::new(interface, Delay)
            .map_err(|source| SensorError::new(Operation::Initialization, source))?;

        let configuration = measurement_configuration();
        let conversion_wait = Duration::from_micros(
            u64::from(bme68x::compensation::measurement_duration_us(
                OperationMode::Forced,
                &configuration,
            )) + u64::from(HEATER_DURATION_MS) * 1_000
                + MEASUREMENT_MARGIN_US,
        );

        Ok(Self {
            driver,
            address,
            conversion_wait,
        })
    }

    /// Apply the application's oversampling and heater policy.
    ///
    /// This consumes and returns `self` so an error can retain ownership of the
    /// initialized driver and bus resources instead of silently dropping them.
    pub fn configure(mut self) -> Result<Self, SetupFailure> {
        let configuration = measurement_configuration();
        if let Err(source) = self.driver.set_configuration(&configuration) {
            return Err(SetupFailure {
                sensor: self,
                error: SensorError::new(Operation::Configuration, source),
            });
        }

        let heater_configuration = HeaterConfiguration::Forced {
            enabled: true,
            temperature_celsius: HEATER_TEMPERATURE_CELSIUS,
            duration_ms: HEATER_DURATION_MS,
        };
        if let Err(source) = self.driver.set_heater_configuration(&heater_configuration) {
            return Err(SetupFailure {
                sensor: self,
                error: SensorError::new(Operation::HeaterConfiguration, source),
            });
        }

        Ok(self)
    }

    pub const fn chip_id(&self) -> u8 {
        self.driver.chip_id()
    }

    pub const fn variant(&self) -> Variant {
        self.driver.variant()
    }

    pub const fn address(&self) -> u8 {
        self.address
    }

    pub const fn conversion_wait(&self) -> Duration {
        self.conversion_wait
    }

    /// Trigger one forced conversion, wait asynchronously, and read its fields.
    pub async fn sample(&mut self) -> Result<Measurements, SensorError> {
        self.driver
            .set_operation_mode(OperationMode::Forced)
            .map_err(|source| SensorError::new(Operation::ForcedModeTrigger, source))?;

        Timer::after(self.conversion_wait).await;

        self.driver
            .measurements(OperationMode::Forced)
            .map_err(|source| SensorError::new(Operation::DataRead, source))
    }
}

const fn measurement_configuration() -> Configuration {
    // Match Bosch's official forced-mode example.
    Configuration {
        humidity_oversampling: Oversampling::X16,
        temperature_oversampling: Oversampling::X2,
        pressure_oversampling: Oversampling::X1,
        filter: Filter::Off,
        standby_time: StandbyTime::None,
    }
}
