//! Health-only telemetry used when profile-v2 cannot finish sensor startup.
//!
//! This module is target-independent so its exact encoded records and retry
//! accounting can be exercised by the host test crate.

use vesta_protocol_v2::v2::{
    Common, DeviceHealth, EncodedFrame, Error as CodecError, HEALTH_FLAG_BOOT_ID_UNAVAILABLE,
    HEALTH_FLAG_CONFIG_MISMATCH, HEALTH_FLAG_COUNTERS_SATURATED, HEALTH_FLAG_RADIO_ERROR_SEEN,
    HEALTH_FLAG_SENSOR_ERROR_SEEN, encode_device_health,
};

/// No verified sensor/profile configuration exists on a fatal startup path.
pub const CONFIG_ID_UNAVAILABLE: u64 = 0;
/// Health-only startup failures do not claim a heater profile.
pub const PROFILE_ID_UNAVAILABLE: u16 = 0;
pub const PROFILE_VERSION_UNAVAILABLE: u16 = 0;

/// Both supported BME688 addresses failed their concrete I2C probes.
pub const STARTUP_ERROR_BME_PROBE: u16 = 0x0101;
/// A configured sensor unexpectedly had no retained read-back metadata.
pub const STARTUP_ERROR_SENSOR_METADATA: u16 = 0x0201;
/// The verified sensor metadata could not form an encodable `DeviceConfig`.
pub const STARTUP_ERROR_TELEMETRY_SETUP: u16 = 0x0202;

const TRANSMISSION_ERROR_PRESENT: u16 = 1;

/// Stable boot/reset metadata retained even when sensor setup fails.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Context {
    pub node_id: u64,
    pub boot_id: u64,
    pub boot_id_valid: bool,
    pub boot_id_from_hardware_rng: bool,
    pub reset_cause_raw: u32,
    pub reset_cause_flags: u16,
    pub sample_interval_ms: u32,
    pub firmware_version: [u8; 3],
}

impl Context {
    const fn boot_id_available(self) -> bool {
        self.boot_id_valid && self.boot_id_from_hardware_rng
    }
}

/// Concrete startup failure translated into protocol health accounting.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct StartupFailure {
    last_sensor_error: u16,
    i2c_errors: u32,
}

impl StartupFailure {
    #[must_use]
    pub const fn bme_probe() -> Self {
        Self {
            last_sensor_error: STARTUP_ERROR_BME_PROBE,
            // One failed transaction at each concrete address, 0x76 and 0x77.
            i2c_errors: 2,
        }
    }

    #[must_use]
    pub const fn sensor_operation(operation_code: u16, is_i2c_error: bool) -> Self {
        Self {
            last_sensor_error: operation_code,
            i2c_errors: is_i2c_error as u32,
        }
    }

    #[must_use]
    pub const fn sensor_metadata_missing() -> Self {
        Self {
            last_sensor_error: STARTUP_ERROR_SENSOR_METADATA,
            i2c_errors: 0,
        }
    }

    #[must_use]
    pub const fn telemetry_setup() -> Self {
        Self {
            last_sensor_error: STARTUP_ERROR_TELEMETRY_SETUP,
            i2c_errors: 0,
        }
    }
}

/// One encoded retry and the sequence placed in its common header.
pub struct EncodedHealth {
    pub sequence: u32,
    pub frame: EncodedFrame,
}

/// Saturating state retained between health-only output attempts.
pub struct Reporter {
    context: Context,
    failure: StartupFailure,
    i2c_errors: u32,
    next_sequence: u32,
    output_failures: u32,
    radio_tx_errors: u32,
    last_radio_error: u16,
}

impl Reporter {
    #[must_use]
    pub const fn new(context: Context, failure: StartupFailure) -> Self {
        Self {
            context,
            failure,
            i2c_errors: failure.i2c_errors,
            next_sequence: 0,
            output_failures: 0,
            radio_tx_errors: 0,
            last_radio_error: 0,
        }
    }

    /// Retain one more bounded startup attempt while preserving the reporter's
    /// sequence and output counters across recoverable-state retries.
    pub fn record_startup_failure(&mut self, failure: StartupFailure) {
        self.failure = failure;
        self.i2c_errors = self.i2c_errors.saturating_add(failure.i2c_errors);
    }

    /// Encode a health-only record. Sequence advances even if encoding fails,
    /// exposing a gap instead of silently reusing an attempted identity.
    pub fn encode_next(&mut self, uptime_ms: u64) -> Result<EncodedHealth, CodecError> {
        let sequence = self.next_sequence;
        self.next_sequence = self.next_sequence.wrapping_add(1);
        let common = if self.context.boot_id_available() {
            Common::production(
                self.context.node_id,
                self.context.boot_id,
                sequence,
                uptime_ms,
                CONFIG_ID_UNAVAILABLE,
                self.context.reset_cause_flags,
            )
        } else {
            Common::boot_id_unavailable(
                self.context.node_id,
                sequence,
                uptime_ms,
                CONFIG_ID_UNAVAILABLE,
                self.context.reset_cause_flags,
            )
        };
        let frame = encode_device_health(common, &self.snapshot())?;
        Ok(EncodedHealth { sequence, frame })
    }

    /// Count every output failure locally; radio failures are additionally
    /// represented by the protocol's cumulative radio-error fields.
    pub fn record_output_failure(&mut self, radio_output: bool) {
        self.output_failures = self.output_failures.saturating_add(1);
        if radio_output {
            self.radio_tx_errors = self.radio_tx_errors.saturating_add(1);
            self.last_radio_error = TRANSMISSION_ERROR_PRESENT;
        }
    }

    pub fn record_output_success(&mut self, radio_output: bool) {
        if radio_output {
            self.last_radio_error = 0;
        }
    }

    #[must_use]
    pub const fn output_failures(&self) -> u32 {
        self.output_failures
    }

    #[must_use]
    pub const fn i2c_errors(&self) -> u32 {
        self.i2c_errors
    }

    #[must_use]
    pub const fn radio_tx_errors(&self) -> u32 {
        self.radio_tx_errors
    }

    #[must_use]
    pub const fn last_radio_error(&self) -> u16 {
        self.last_radio_error
    }

    #[must_use]
    pub const fn last_sensor_error(&self) -> u16 {
        self.failure.last_sensor_error
    }

    fn snapshot(&self) -> DeviceHealth {
        let mut flags = HEALTH_FLAG_CONFIG_MISMATCH | HEALTH_FLAG_SENSOR_ERROR_SEEN;
        if !self.context.boot_id_available() {
            flags |= HEALTH_FLAG_BOOT_ID_UNAVAILABLE;
        }
        if self.i2c_errors == u32::MAX
            || self.radio_tx_errors == u32::MAX
            || self.output_failures == u32::MAX
        {
            flags |= HEALTH_FLAG_COUNTERS_SATURATED;
        }
        if self.radio_tx_errors != 0 {
            flags |= HEALTH_FLAG_RADIO_ERROR_SEEN;
        }

        DeviceHealth {
            flags,
            reset_cause_raw: self.context.reset_cause_raw,
            // No profile scan began, so scan/profile counters deliberately stay
            // zero. The startup failure is carried by flags and error fields.
            successful_sensor_scans: 0,
            failed_sensor_scans: 0,
            incomplete_profiles: 0,
            i2c_errors: self.i2c_errors,
            radio_tx_errors: self.radio_tx_errors,
            dropped_profiles: 0,
            dropped_fragments: 0,
            overwritten_fields: 0,
            current_sample_interval_ms: self.context.sample_interval_ms,
            firmware_version: self.context.firmware_version,
            profile_id: PROFILE_ID_UNAVAILABLE,
            profile_version: PROFILE_VERSION_UNAVAILABLE,
            last_sensor_error: self.failure.last_sensor_error,
            last_radio_error: self.last_radio_error,
            calibrated_mcu_temperature_centi_celsius: None,
            calibrated_vdd_millivolt: None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use vesta_protocol_v2::v2::{
        COMMON_FLAG_BOOT_ID_FROM_HW_RNG, COMMON_FLAG_BOOT_ID_VALID, DecodedFrame,
        HEALTH_FLAG_LAST_SCAN_INCOMPLETE, decode,
    };

    const CONTEXT: Context = Context {
        node_id: 0x4fe6_08a9_ee2f_303e,
        boot_id: 0x0123_4567_89ab_cdef,
        boot_id_valid: true,
        boot_id_from_hardware_rng: true,
        reset_cause_raw: 0x8400_0003,
        reset_cause_flags: 0x0084,
        sample_interval_ms: 300_000,
        firmware_version: [0, 2, 0],
    };

    #[test]
    fn probe_failure_encodes_decodable_health_without_a_config_or_scan() {
        let mut reporter = Reporter::new(CONTEXT, StartupFailure::bme_probe());
        let encoded = reporter.encode_next(12_345).unwrap();
        assert_eq!(encoded.frame.len(), 102);
        let DecodedFrame::DeviceHealth { header, health } =
            decode(encoded.frame.as_slice()).unwrap()
        else {
            panic!("expected DeviceHealth");
        };

        assert_eq!(encoded.sequence, 0);
        assert_eq!(header.common.node_id, CONTEXT.node_id);
        assert_eq!(header.common.boot_id, CONTEXT.boot_id);
        assert_eq!(header.common.uptime_ms, 12_345);
        assert_eq!(header.common.config_id, CONFIG_ID_UNAVAILABLE);
        assert_eq!(header.common.reset_cause_flags, CONTEXT.reset_cause_flags);
        assert_eq!(
            header.common.flags,
            COMMON_FLAG_BOOT_ID_VALID | COMMON_FLAG_BOOT_ID_FROM_HW_RNG
        );
        assert_eq!(health.reset_cause_raw, CONTEXT.reset_cause_raw);
        assert_eq!(health.failed_sensor_scans, 0);
        assert_eq!(health.incomplete_profiles, 0);
        assert_eq!(health.i2c_errors, 2);
        assert_eq!(health.profile_id, PROFILE_ID_UNAVAILABLE);
        assert_eq!(health.profile_version, PROFILE_VERSION_UNAVAILABLE);
        assert_eq!(health.last_sensor_error, STARTUP_ERROR_BME_PROBE);
        assert_ne!(health.flags & HEALTH_FLAG_CONFIG_MISMATCH, 0);
        assert_ne!(health.flags & HEALTH_FLAG_SENSOR_ERROR_SEEN, 0);
        assert_eq!(health.flags & HEALTH_FLAG_LAST_SCAN_INCOMPLETE, 0);
    }

    #[test]
    fn unavailable_rng_identity_is_exactly_flagged() {
        let mut context = CONTEXT;
        context.boot_id = 0;
        context.boot_id_valid = false;
        context.boot_id_from_hardware_rng = false;
        let mut reporter = Reporter::new(context, StartupFailure::sensor_metadata_missing());
        let encoded = reporter.encode_next(7).unwrap();
        let DecodedFrame::DeviceHealth { header, health } =
            decode(encoded.frame.as_slice()).unwrap()
        else {
            panic!("expected DeviceHealth");
        };

        assert_eq!(header.common.flags, 0);
        assert_eq!(header.common.boot_id, 0);
        assert_ne!(health.flags & HEALTH_FLAG_BOOT_ID_UNAVAILABLE, 0);
        assert_eq!(health.last_sensor_error, STARTUP_ERROR_SENSOR_METADATA);
    }

    #[test]
    fn recoverable_startup_retries_accumulate_i2c_errors_and_keep_sequence() {
        let mut reporter = Reporter::new(CONTEXT, StartupFailure::bme_probe());
        let first = reporter.encode_next(1).unwrap();
        reporter.record_startup_failure(StartupFailure::sensor_operation(4, true));
        let second = reporter.encode_next(2).unwrap();

        let DecodedFrame::DeviceHealth { health, .. } = decode(second.frame.as_slice()).unwrap()
        else {
            panic!("expected DeviceHealth");
        };
        assert_eq!(first.sequence, 0);
        assert_eq!(second.sequence, 1);
        assert_eq!(health.i2c_errors, 3);
        assert_eq!(health.last_sensor_error, 4);
    }

    #[test]
    fn radio_failure_is_visible_on_the_next_successful_retry() {
        let mut reporter = Reporter::new(CONTEXT, StartupFailure::sensor_operation(3, true));
        reporter.record_output_failure(true);
        assert_eq!(reporter.output_failures(), 1);
        let encoded = reporter.encode_next(20).unwrap();
        let DecodedFrame::DeviceHealth { health, .. } = decode(encoded.frame.as_slice()).unwrap()
        else {
            panic!("expected DeviceHealth");
        };
        assert_eq!(health.i2c_errors, 1);
        assert_eq!(health.radio_tx_errors, 1);
        assert_eq!(health.last_radio_error, TRANSMISSION_ERROR_PRESENT);
        assert_ne!(health.flags & HEALTH_FLAG_RADIO_ERROR_SEEN, 0);

        reporter.record_output_success(true);
        let encoded = reporter.encode_next(30).unwrap();
        let DecodedFrame::DeviceHealth { health, .. } = decode(encoded.frame.as_slice()).unwrap()
        else {
            panic!("expected DeviceHealth");
        };
        assert_eq!(health.radio_tx_errors, 1);
        assert_eq!(health.last_radio_error, 0);
        assert_ne!(health.flags & HEALTH_FLAG_RADIO_ERROR_SEEN, 0);
    }

    #[test]
    fn non_radio_output_failure_is_retried_without_fabricating_radio_errors() {
        let mut reporter = Reporter::new(CONTEXT, StartupFailure::telemetry_setup());
        reporter.record_output_failure(false);
        assert_eq!(reporter.output_failures(), 1);
        let encoded = reporter.encode_next(44).unwrap();
        let DecodedFrame::DeviceHealth { health, .. } = decode(encoded.frame.as_slice()).unwrap()
        else {
            panic!("expected DeviceHealth");
        };
        assert_eq!(health.radio_tx_errors, 0);
        assert_eq!(health.last_radio_error, 0);
        assert_eq!(health.last_sensor_error, STARTUP_ERROR_TELEMETRY_SETUP);
    }

    #[test]
    fn retry_sequence_wrap_is_explicit() {
        let mut reporter = Reporter::new(CONTEXT, StartupFailure::telemetry_setup());
        reporter.next_sequence = u32::MAX;
        assert_eq!(reporter.encode_next(1).unwrap().sequence, u32::MAX);
        assert_eq!(reporter.encode_next(2).unwrap().sequence, 0);
    }
}
