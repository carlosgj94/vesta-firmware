//! Complete, bounded Bosch HP-354 parallel-profile acquisition.
//!
//! This is the neutral exploratory 10-step profile published as HP-354 in
//! Bosch's BME AI-Studio Manual (BST-BME688-AN001-00, v1.6.0). It is not a
//! fire classifier or a fire-specific profile. The firmware retains every
//! raw/compensated field and leaves interpretation to the receiver.

use bme68x::blocking::Bme68x;
use bme68x::interface::I2cInterface;
use bme68x::{
    CalibrationData, Configuration, Error as Bme68xError, Filter, HeaterConfiguration,
    OperationMode, Oversampling, ProfileCollector, ProfileCollectorError, ProfileFinishReason,
    SensorConfigurationReadback, StandbyTime, Variant,
};
use embassy_stm32::i2c;
use embassy_time::{Delay, Duration, Instant, Timer};

use crate::board::{SensorBus, SensorProbe};
use crate::profile_definition::{
    PROFILE_REPETITION_MULTIPLIERS, PROFILE_STEP_COUNT, PROFILE_TEMPERATURES_CELSIUS,
    PROGRAMMED_SHARED_DURATION_RAW, REQUESTED_SHARED_DURATION_MS, TPHG_DURATION_US,
};
use crate::profile_status::{
    EmergencySleepRetryProgress, carry_pending_reconfiguration,
    reconfiguration_marker_after_programming_attempt,
};

pub const POLL_INTERVAL: Duration = Duration::from_millis(100);
pub const COLLECTION_TIMEOUT: Duration = Duration::from_secs(15);
pub const MAX_POLL_COUNT: u16 = 150;
const EMERGENCY_SLEEP_RETRY_DELAY: Duration = Duration::from_millis(10);

type Driver = Bme68x<I2cInterface<SensorBus>, Delay>;
pub type DriverError = Bme68xError<i2c::Error>;

/// Driver operation that failed. This is retained in health accounting.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Operation {
    Initialization,
    Configuration,
    HeaterConfiguration,
    ConfigurationReadback,
    ParallelModeTrigger,
    DataRead,
    ReturnToSleep,
    PreTriggerFieldDrain,
}

impl Operation {
    #[must_use]
    pub const fn code(self) -> u16 {
        match self {
            Self::Initialization => 1,
            Self::Configuration => 2,
            Self::HeaterConfiguration => 3,
            Self::ConfigurationReadback => 4,
            Self::ParallelModeTrigger => 5,
            Self::DataRead => 6,
            Self::ReturnToSleep => 7,
            Self::PreTriggerFieldDrain => 8,
        }
    }
}

/// A BME688 failure paired with the exact operation that failed.
pub struct SensorError {
    operation: Operation,
    source: DriverError,
}

impl SensorError {
    const fn new(operation: Operation, source: DriverError) -> Self {
        Self { operation, source }
    }

    #[must_use]
    pub const fn operation(&self) -> Operation {
        self.operation
    }

    #[must_use]
    pub const fn source(&self) -> &DriverError {
        &self.source
    }

    #[must_use]
    pub const fn is_i2c_error(&self) -> bool {
        matches!(self.source, Bme68xError::Bus(_))
    }
}

/// Configuration failure that retains all sensor/bus ownership.
pub struct SetupFailure {
    pub(crate) sensor: Sensor,
    pub(crate) error: SensorError,
}

impl SetupFailure {
    pub(crate) fn into_parts(self) -> (Sensor, SensorError) {
        (self.sensor, self.error)
    }
}

/// Coherent readback and identity used by repeated DeviceConfig records.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SensorMetadata {
    pub chip_id: u8,
    pub variant: Variant,
    pub address: u8,
    pub calibration_fingerprint: u64,
    pub readback: SensorConfigurationReadback,
    pub exact_configuration_verified: bool,
}

impl SensorMetadata {
    /// Compare the stable sensor/configuration identity while ignoring only
    /// unprogrammed volatile IDAC readback bytes.
    #[must_use]
    pub fn same_configuration_identity(&self, other: &Self) -> bool {
        self.chip_id == other.chip_id
            && self.variant == other.variant
            && self.address == other.address
            && self.calibration_fingerprint == other.calibration_fingerprint
            && self.exact_configuration_verified == other.exact_configuration_verified
            && crate::profile_status::sensor_readback_identity_eq(&self.readback, &other.readback)
    }
}

/// A complete or explicitly partial scan. Runtime sensor failures do not erase
/// measurements collected before the failure.
pub struct ProfileScan {
    pub collector: ProfileCollector,
    pub started_at: Instant,
    pub duration: Duration,
    pub poll_count: u16,
    pub collection_error: Option<SensorError>,
    pub sleep_error: Option<SensorError>,
    /// Exact configuration snapshot captured after preflight and before the
    /// stale-slot drain/Parallel trigger. `None` means no heater scan began.
    pub pre_trigger_metadata: Option<SensorMetadata>,
    /// Result of the pre-trigger readback/recovery, never post-scan recovery.
    pub configuration_verified: bool,
    /// True only when programming plus readback restored the exact profile
    /// before this scan was triggered.
    pub sensor_reconfigured: bool,
    pub configuration_i2c_errors: u32,
    pub configuration_error: Option<SensorError>,
    pub configuration_error_code: u16,
    /// A trigger/read/sleep fault causes a separate recovery after the scan.
    /// These fields describe state for subsequent scans and must never relabel
    /// the current scan's configuration identity.
    pub post_scan_configuration_checked: bool,
    pub post_scan_configuration_verified: bool,
    pub post_scan_metadata: Option<SensorMetadata>,
    pub post_scan_sensor_reconfigured: bool,
    pub post_scan_configuration_i2c_errors: u32,
    pub post_scan_configuration_error: Option<SensorError>,
    pub post_scan_configuration_error_code: u16,
    /// Fixed-count stop-only recovery after both the normal Sleep request and
    /// post-scan configuration recovery failed to prove the sensor asleep.
    pub emergency_sleep_retry_attempts: u8,
    pub emergency_sleep_confirmed: bool,
    /// NEW_DATA fields drained while asleep immediately before this scan.
    pub pre_scan_discarded_fields: u16,
}

impl ProfileScan {
    #[must_use]
    pub const fn sensor_failed(&self) -> bool {
        !self.configuration_verified
            || self.collection_error.is_some()
            || self.sleep_error.is_some()
    }

    #[must_use]
    pub fn i2c_error_count(&self) -> u32 {
        self.configuration_i2c_errors
            .saturating_add(self.post_scan_configuration_i2c_errors)
            .saturating_add(u32::from(
                self.collection_error
                    .as_ref()
                    .is_some_and(SensorError::is_i2c_error),
            ))
            .saturating_add(u32::from(
                self.sleep_error
                    .as_ref()
                    .is_some_and(SensorError::is_i2c_error),
            ))
    }

    #[must_use]
    pub fn last_error_code(&self) -> u16 {
        if self.post_scan_configuration_error_code != 0 {
            self.post_scan_configuration_error_code
        } else {
            self.sleep_error
                .as_ref()
                .or(self.collection_error.as_ref())
                .map_or(self.configuration_error_code, |error| {
                    error.operation().code()
                })
        }
    }

    #[must_use]
    pub fn health_configuration_identity_available(&self) -> bool {
        crate::profile_status::health_configuration_identity_available(
            self.configuration_verified,
            self.post_scan_configuration_checked,
            self.post_scan_configuration_verified,
            self.post_scan_metadata
                .zip(self.pre_trigger_metadata)
                .is_some_and(|(post, pre)| post.same_configuration_identity(&pre)),
        )
    }
}

struct ConfigurationCheck {
    verified: bool,
    reconfigured: bool,
    programming_attempted: bool,
    i2c_errors: u32,
    error: Option<SensorError>,
    last_error_code: u16,
}

impl ConfigurationCheck {
    const fn assumed(verified: bool) -> Self {
        Self {
            verified,
            reconfigured: false,
            programming_attempted: false,
            i2c_errors: 0,
            error: None,
            last_error_code: 0,
        }
    }
}

/// Reusable sensor driver. The bus wrapper itself only instantiates I2C2 for
/// individual transactions, preserving STOP2 eligibility between scans.
pub struct Sensor {
    driver: Driver,
    address: u8,
    preflight_chip_id: u8,
    metadata: Option<SensorMetadata>,
    reconfiguration_marker_pending: bool,
}

impl Sensor {
    pub fn new(probe: SensorProbe) -> Result<Self, SensorError> {
        let address = probe.address_byte();
        let preflight_chip_id = probe.chip_id();
        let (bus, i2c_address) = probe.into_parts();
        let interface = I2cInterface::new(bus, i2c_address);
        let driver = Bme68x::new(interface, Delay)
            .map_err(|source| SensorError::new(Operation::Initialization, source))?;
        Ok(Self {
            driver,
            address,
            preflight_chip_id,
            metadata: None,
            reconfiguration_marker_pending: false,
        })
    }

    /// Program HP-354, read it back while asleep, and retain both requested
    /// and actual raw heater metadata for the configuration record.
    #[allow(clippy::result_large_err)]
    pub fn configure(mut self) -> Result<Self, SetupFailure> {
        let programmed = self.program_desired_configuration();
        // Preserve the programming/readback error if both operations fail, but
        // always make the explicit Sleep attempt before returning ownership.
        let sleep_result = self.driver.set_operation_mode(OperationMode::Sleep);
        let metadata = match programmed {
            Ok(metadata) => metadata,
            Err(error) => {
                return Err(SetupFailure {
                    sensor: self,
                    error,
                });
            }
        };
        if let Err(source) = sleep_result {
            return Err(SetupFailure {
                sensor: self,
                error: SensorError::new(Operation::ReturnToSleep, source),
            });
        }
        self.metadata = Some(metadata);
        Ok(self)
    }

    #[must_use]
    pub fn metadata(&self) -> Option<&SensorMetadata> {
        self.metadata.as_ref()
    }

    #[must_use]
    pub fn configuration_verified(&self) -> bool {
        self.metadata
            .as_ref()
            .is_some_and(|metadata| metadata.exact_configuration_verified)
    }

    /// Clear the receiver-history reset marker only after the application has
    /// confirmed a locally usable marked profile was delivered in full.
    pub fn acknowledge_reconfiguration_marker(&mut self) {
        self.reconfiguration_marker_pending = false;
    }

    /// Collect one complete profile or return a bounded partial result.
    ///
    /// Once the driver exists, every exit path attempts an explicit transition
    /// to Sleep because BME688 Parallel mode otherwise repeats indefinitely.
    pub async fn scan(
        &mut self,
        verify_configuration_before_scan: bool,
    ) -> Result<ProfileScan, ProfileCollectorError> {
        // Used only when verification fails before a heater scan can start.
        let attempt_started_at = Instant::now();
        let mut collector = match ProfileCollector::new(OperationMode::Parallel, PROFILE_STEP_COUNT)
        {
            Ok(collector) => collector,
            Err(error) => {
                // Compile-time constants make this unreachable in a valid
                // build, but even this path explicitly requests Sleep.
                let _ = self.driver.set_operation_mode(OperationMode::Sleep);
                return Err(error);
            }
        };
        let mut poll_count = 0_u16;
        let mut collection_error = None;
        let mut pre_scan_discarded_fields = 0_u16;
        let mut parallel_triggered = false;
        let mut configuration = ConfigurationCheck::assumed(self.configuration_verified());

        if verify_configuration_before_scan {
            configuration = self.validate_or_reconfigure();
        }
        self.reconfiguration_marker_pending = reconfiguration_marker_after_programming_attempt(
            self.reconfiguration_marker_pending,
            configuration.programming_attempted,
        );
        // Snapshot only metadata proven exact before this trigger. A later
        // recovery may update `self.metadata`, but cannot change the identity
        // or step descriptors attached to this logical scan.
        let pre_trigger_metadata = self
            .metadata
            .filter(|metadata| configuration.verified && metadata.exact_configuration_verified);
        if configuration.verified && pre_trigger_metadata.is_none() {
            configuration.verified = false;
            configuration.reconfigured = false;
            configuration.last_error_code = Operation::ConfigurationReadback.code();
        }
        // Reading all three field slots while verified asleep clears any
        // surviving NEW_DATA from a prior read/sleep failure. Without this
        // bounded drain a fresh collector could accept previous-cycle slots.
        if configuration.verified {
            match self.driver.measurements(OperationMode::Parallel) {
                Ok(stale_fields) => {
                    pre_scan_discarded_fields =
                        u16::try_from(stale_fields.len()).unwrap_or(u16::MAX);
                }
                Err(source) => {
                    collection_error =
                        Some(SensorError::new(Operation::PreTriggerFieldDrain, source));
                    collector.finish(error_finish_reason(collection_error.as_ref()));
                }
            }
        }

        // Protocol scan uptime and step offsets exclude preflight/recovery.
        // A failed preflight has no heater start, so retain the scheduled
        // attempt anchor for its explicit empty/partial record.
        let started_at = if configuration.verified && collection_error.is_none() {
            Instant::now()
        } else {
            attempt_started_at
        };

        if !configuration.verified {
            collector.finish(error_finish_reason(configuration.error.as_ref()));
        } else if collection_error.is_some() {
            // The failed stale-field drain was already recorded and finished.
        } else if let Err(source) = self.driver.set_operation_mode(OperationMode::Parallel) {
            collection_error = Some(SensorError::new(Operation::ParallelModeTrigger, source));
            collector.finish(error_finish_reason(collection_error.as_ref()));
        } else {
            parallel_triggered = true;
            loop {
                Timer::after(POLL_INTERVAL).await;
                poll_count = poll_count.saturating_add(1);
                let offset = elapsed_us_saturating(started_at);

                match self.driver.measurements(OperationMode::Parallel) {
                    Ok(fields) => collector.ingest_batch(&fields, offset),
                    Err(source) => {
                        collection_error = Some(SensorError::new(Operation::DataRead, source));
                        collector.finish(error_finish_reason(collection_error.as_ref()));
                        break;
                    }
                }

                if collector.all_steps_gas_valid() {
                    collector.finish(ProfileFinishReason::Complete);
                    break;
                }
                if collector.is_frozen() {
                    break;
                }
                if poll_count >= MAX_POLL_COUNT || started_at.elapsed() >= COLLECTION_TIMEOUT {
                    collector.finish(ProfileFinishReason::Timeout);
                    break;
                }
            }
        }

        // A preflight/post-scan restore belongs to the next heater profile
        // that actually starts. Drain/trigger failures retain the marker.
        let (mark_current_scan, marker_pending) = carry_pending_reconfiguration(
            self.reconfiguration_marker_pending,
            configuration.verified,
            configuration.reconfigured,
            parallel_triggered,
        );
        configuration.reconfigured = mark_current_scan;
        self.reconfiguration_marker_pending = marker_pending;

        // Always attempt Sleep, including trigger, read, timeout, rollover, and
        // successful completion paths. Preserve both failures if this also
        // fails; neither one discards the partial collector.
        let sleep_error = self
            .driver
            .set_operation_mode(OperationMode::Sleep)
            .err()
            .map(|source| SensorError::new(Operation::ReturnToSleep, source));
        let duration = started_at.elapsed();

        // A trigger/read/sleep fault may have reset or corrupted sensor
        // configuration. Re-read it immediately and make one bounded attempt
        // to restore HP-354 for later scans. Keep this outcome separate: an
        // after-the-fact recovery must never relabel the current partial scan.
        let mut post_scan_configuration = (collection_error.is_some() || sleep_error.is_some())
            .then(|| self.validate_or_reconfigure());

        // If the original post-Parallel Sleep failed and the bounded
        // readback/reconfiguration path still cannot prove exact Sleep, make
        // three additional stop-only attempts. A transient bus failure returns
        // immediately from the driver, so short gaps give it a chance to
        // recover without delaying health for an unbounded interval.
        let mut emergency_sleep = EmergencySleepRetryProgress::default();
        if sleep_error.is_some()
            && post_scan_configuration
                .as_ref()
                .is_some_and(|check| !check.verified)
        {
            while emergency_sleep.should_retry() {
                Timer::after(EMERGENCY_SLEEP_RETRY_DELAY).await;
                match self.driver.set_operation_mode(OperationMode::Sleep) {
                    Ok(()) => emergency_sleep.record_attempt(true, false),
                    Err(source) => {
                        let error = SensorError::new(Operation::ReturnToSleep, source);
                        emergency_sleep.record_attempt(false, error.is_i2c_error());
                        if let Some(check) = post_scan_configuration.as_mut() {
                            check.i2c_errors = check
                                .i2c_errors
                                .saturating_add(u32::from(error.is_i2c_error()));
                            check.last_error_code = error.operation().code();
                            check.error = Some(error);
                        }
                    }
                }
            }
        }
        if post_scan_configuration
            .as_ref()
            .is_some_and(|check| check.verified && check.reconfigured)
        {
            self.reconfiguration_marker_pending = true;
        }
        if let Some(check) = post_scan_configuration.as_ref() {
            self.reconfiguration_marker_pending = reconfiguration_marker_after_programming_attempt(
                self.reconfiguration_marker_pending,
                check.programming_attempted,
            );
        }
        let post_scan_metadata = post_scan_configuration
            .as_ref()
            .filter(|check| check.verified)
            .and(self.metadata)
            .filter(|metadata| metadata.exact_configuration_verified);

        Ok(ProfileScan {
            collector,
            started_at,
            duration,
            poll_count,
            collection_error,
            sleep_error,
            pre_trigger_metadata,
            configuration_verified: configuration.verified,
            sensor_reconfigured: configuration.reconfigured,
            configuration_i2c_errors: configuration.i2c_errors,
            configuration_error: configuration.error,
            configuration_error_code: configuration.last_error_code,
            post_scan_configuration_checked: post_scan_configuration.is_some(),
            post_scan_configuration_verified: post_scan_configuration
                .as_ref()
                .is_some_and(|check| check.verified),
            post_scan_metadata,
            post_scan_sensor_reconfigured: post_scan_configuration
                .as_ref()
                .is_some_and(|check| check.reconfigured),
            post_scan_configuration_i2c_errors: post_scan_configuration
                .as_ref()
                .map_or(0, |check| check.i2c_errors),
            post_scan_configuration_error: post_scan_configuration
                .as_mut()
                .and_then(|check| check.error.take()),
            post_scan_configuration_error_code: post_scan_configuration
                .as_ref()
                .map_or(0, |check| check.last_error_code),
            emergency_sleep_retry_attempts: emergency_sleep.attempts,
            emergency_sleep_confirmed: emergency_sleep.confirmed,
            pre_scan_discarded_fields,
        })
    }

    fn program_desired_configuration(&mut self) -> Result<SensorMetadata, SensorError> {
        self.driver
            .set_configuration(&measurement_configuration())
            .map_err(|source| SensorError::new(Operation::Configuration, source))?;
        self.driver
            .set_heater_configuration(&HeaterConfiguration::Parallel {
                enabled: true,
                temperatures_celsius: &PROFILE_TEMPERATURES_CELSIUS,
                repetition_multipliers: &PROFILE_REPETITION_MULTIPLIERS,
                shared_duration_ms: REQUESTED_SHARED_DURATION_MS,
            })
            .map_err(|source| SensorError::new(Operation::HeaterConfiguration, source))?;
        self.read_metadata()
    }

    fn read_metadata(&mut self) -> Result<SensorMetadata, SensorError> {
        let readback = self
            .driver
            .configuration_readback()
            .map_err(|source| SensorError::new(Operation::ConfigurationReadback, source))?;
        let variant = self.driver.variant();
        Ok(SensorMetadata {
            chip_id: self.driver.chip_id(),
            variant,
            address: self.address,
            calibration_fingerprint: self.driver.calibration_fingerprint(),
            exact_configuration_verified: self.preflight_chip_id == self.driver.chip_id()
                && verify_readback(
                    &readback,
                    variant,
                    self.driver.calibration(),
                    self.driver.ambient_temperature(),
                ),
            readback,
        })
    }

    /// Re-read current registers and, on mismatch or read failure, make one
    /// bounded reconfiguration/readback attempt. No concrete I2C driver lives
    /// beyond the individual blocking transactions in these calls.
    fn validate_or_reconfigure(&mut self) -> ConfigurationCheck {
        let mut check = ConfigurationCheck::assumed(false);
        match self.read_metadata() {
            Ok(metadata) if metadata.exact_configuration_verified => {
                self.metadata = Some(metadata);
                return ConfigurationCheck::assumed(true);
            }
            Ok(metadata) => {
                self.metadata = Some(metadata);
                check.last_error_code = Operation::ConfigurationReadback.code();
            }
            Err(error) => {
                check.i2c_errors = u32::from(error.is_i2c_error());
                check.last_error_code = error.operation().code();
                check.error = Some(error);
                if let Some(metadata) = self.metadata.as_mut() {
                    metadata.exact_configuration_verified = false;
                }
            }
        }

        check.programming_attempted = true;
        match self.program_desired_configuration() {
            Ok(metadata) => {
                check.verified = metadata.exact_configuration_verified;
                check.reconfigured = metadata.exact_configuration_verified;
                self.metadata = Some(metadata);
                if !check.verified {
                    check.last_error_code = Operation::ConfigurationReadback.code();
                }
            }
            Err(error) => {
                check.i2c_errors = check
                    .i2c_errors
                    .saturating_add(u32::from(error.is_i2c_error()));
                check.last_error_code = error.operation().code();
                check.error = Some(error);
                if let Some(metadata) = self.metadata.as_mut() {
                    metadata.exact_configuration_verified = false;
                }
            }
        }

        // Even a failed programming/readback sequence gets an explicit Sleep
        // request before returning ownership to the scheduled loop.
        if let Err(source) = self.driver.set_operation_mode(OperationMode::Sleep) {
            let error = SensorError::new(Operation::ReturnToSleep, source);
            check.i2c_errors = check
                .i2c_errors
                .saturating_add(u32::from(error.is_i2c_error()));
            check.last_error_code = error.operation().code();
            check.error = Some(error);
            check.verified = false;
            check.reconfigured = false;
            if let Some(metadata) = self.metadata.as_mut() {
                metadata.exact_configuration_verified = false;
            }
        }
        check
    }
}

const fn measurement_configuration() -> Configuration {
    Configuration {
        humidity_oversampling: Oversampling::X1,
        temperature_oversampling: Oversampling::X2,
        pressure_oversampling: Oversampling::X16,
        filter: Filter::Off,
        standby_time: StandbyTime::None,
    }
}

fn verify_readback(
    readback: &SensorConfigurationReadback,
    variant: Variant,
    calibration: &CalibrationData,
    ambient_temperature_celsius: i8,
) -> bool {
    let expected_run_gas = match variant {
        Variant::GasLow => 1,
        Variant::GasHigh => 2,
    };
    let expected_heater_resistance =
        PROFILE_TEMPERATURES_CELSIUS
            .iter()
            .enumerate()
            .all(|(index, temperature)| {
                readback.heater.registers.resistance[index]
                    == bme68x::compensation::calculate_heater_resistance(
                        *temperature,
                        ambient_temperature_celsius,
                        calibration,
                    )
            });
    readback.operation_mode == OperationMode::Sleep
        && readback.environmental == measurement_configuration()
        && Driver::measurement_duration(OperationMode::Parallel, &measurement_configuration())
            == TPHG_DURATION_US
        && readback.heater.heater_enabled()
        && readback.heater.run_gas() == expected_run_gas
        && readback.heater.profile_length() == PROFILE_STEP_COUNT
        && readback.heater.registers.shared_duration == PROGRAMMED_SHARED_DURATION_RAW
        && expected_heater_resistance
        && readback.heater.registers.gas_wait[..usize::from(PROFILE_STEP_COUNT)]
            == PROFILE_REPETITION_MULTIPLIERS
}

fn error_finish_reason(error: Option<&SensorError>) -> ProfileFinishReason {
    if error.is_some_and(SensorError::is_i2c_error) {
        ProfileFinishReason::BusError
    } else {
        ProfileFinishReason::SensorStopped
    }
}

fn elapsed_us_saturating(start: Instant) -> u32 {
    u32::try_from(start.elapsed().as_micros()).unwrap_or(u32::MAX)
}
