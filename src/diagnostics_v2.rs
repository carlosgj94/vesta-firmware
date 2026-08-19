//! Non-blocking RTT diagnostics for profile-v2 state and failures.

use bme68x::Error as Bme68xError;
use defmt::{Debug2Format, error, info, warn};

use crate::bme688_profile::{ProfileScan, Sensor, SensorError};
use crate::output_v2::{EmitError, EmitReport, Source};

pub fn log_ready(sensor: &Sensor) {
    if let Some(metadata) = sensor.metadata() {
        info!(
            "BME688 profile-v2 ready: chip=0x{:02x}, variant={:?}, address=0x{:02x}, config_readback_ok={}",
            metadata.chip_id,
            metadata.variant,
            metadata.address,
            metadata.exact_configuration_verified
        );
    }
}

pub fn log_scan(sequence: u32, scan: &ProfileScan) {
    info!(
        "HP-354 scan encoded: sequence={}, observed={}/10, missing=0x{:04x}, gas_valid=0x{:04x}, heater_stable=0x{:04x}, fields={}, stale_pre_scan={}, polls={}, duration_us={}, pre_config_verified={}, pre_reconfigured={}, post_config_checked={}, post_config_verified={}, post_reconfigured={}, emergency_sleep_attempts={}, emergency_sleep_confirmed={}",
        sequence,
        scan.collector.observed_steps(),
        scan.collector.missing_mask(),
        scan.collector.gas_valid_mask(),
        scan.collector.heater_stable_mask(),
        scan.collector.observed_field_count(),
        scan.pre_scan_discarded_fields,
        scan.poll_count,
        scan.duration.as_micros(),
        scan.configuration_verified,
        scan.sensor_reconfigured,
        scan.post_scan_configuration_checked,
        scan.post_scan_configuration_verified,
        scan.post_scan_sensor_reconfigured,
        scan.emergency_sleep_retry_attempts,
        scan.emergency_sleep_confirmed
    );
    if let Some(configuration_error) = &scan.configuration_error {
        log_sensor_error(configuration_error);
    }
    if let Some(configuration_error) = &scan.post_scan_configuration_error {
        log_sensor_error(configuration_error);
    }
    if scan.sensor_failed() || !scan.collector.is_structurally_complete() {
        warn!(
            "HP-354 scan quality issue: sensor_failed={}, finish={:?}",
            scan.sensor_failed(),
            Debug2Format(&scan.collector.finish_reason())
        );
    }
}

pub fn log_sensor_error(sensor_error: &SensorError) {
    let operation = sensor_error.operation().code();
    match sensor_error.source() {
        Bme68xError::Bus(bus_error) => {
            error!(
                "BME688 profile operation {} failed: I2C {:?}",
                operation, bus_error
            );
        }
        other => error!(
            "BME688 profile operation {} failed: {:?}",
            operation,
            Debug2Format(other)
        ),
    }
}

pub fn log_emit_error(error_value: &EmitError) {
    match &error_value.source {
        Source::TooManyRecords => error!(
            "profile-v2 output batch overflow: completed={}, requested={}",
            error_value.completed_records, error_value.requested_records
        ),
        #[cfg(not(feature = "profile-v2-uart"))]
        Source::Radio(source) => error!(
            "profile-v2 LoRa batch failed after {}/{} TX completions: {}",
            error_value.completed_records,
            error_value.requested_records,
            source.source()
        ),
        #[cfg(feature = "profile-v2-uart")]
        Source::UartConfiguration(source) => error!(
            "profile-v2 UART configuration failed after {}/{} records: {:?}",
            error_value.completed_records,
            error_value.requested_records,
            Debug2Format(source)
        ),
        #[cfg(feature = "profile-v2-uart")]
        Source::UartWrite(source) => error!(
            "profile-v2 UART write failed after {}/{} records: {:?}",
            error_value.completed_records,
            error_value.requested_records,
            Debug2Format(source)
        ),
        #[cfg(feature = "profile-v2-uart")]
        Source::Framing(source) => error!(
            "profile-v2 UART framing failed after {}/{} records: {:?}",
            error_value.completed_records,
            error_value.requested_records,
            Debug2Format(source)
        ),
    }
}

pub fn log_emit_success(report: &EmitReport) {
    info!(
        "profile-v2 output complete: records={}/{}",
        report.completed_records, report.requested_records
    );
}

pub fn log_collector_error(error_value: &bme68x::ProfileCollectorError) {
    error!(
        "profile collector construction failed: {:?}",
        Debug2Format(error_value)
    );
}

pub fn log_codec_error(error_value: &vesta_protocol_v2::v2::Error) {
    error!(
        "profile-v2 record encoding failed: {:?}",
        Debug2Format(error_value)
    );
}

pub fn log_internal_error(label: &'static str) {
    error!("profile-v2 internal setup failed: {}", label);
}

pub fn log_degraded_retry(output_failures: u32) {
    warn!(
        "profile-v2 degraded-health output will retry; cumulative output failures={}",
        output_failures
    );
}

pub fn log_degraded_emit_success(sequence: u32, prior_output_failures: u32, report: &EmitReport) {
    info!(
        "profile-v2 degraded health emitted: sequence={}, records={}/{}, prior_output_failures={}",
        sequence, report.completed_records, report.requested_records, prior_output_failures
    );
}
