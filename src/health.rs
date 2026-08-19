//! Saturating device-health accounting retained across scheduled scans.

use crate::bme688_profile::ProfileScan;
use bme68x::ProfileFinishReason;

use crate::profile_status::{ProfileQualityEvidence, logical_profile_success};

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct HealthCounters {
    pub sensor_errors_seen: bool,
    pub successful_sensor_scans: u32,
    pub failed_sensor_scans: u32,
    pub incomplete_profiles: u32,
    pub i2c_errors: u32,
    pub radio_tx_errors: u32,
    pub dropped_profiles: u32,
    pub dropped_fragments: u32,
    pub overwritten_fields: u32,
    pub last_sensor_error: u16,
    pub last_radio_error: u16,
}

impl HealthCounters {
    #[must_use]
    pub const fn any_counter_saturated(&self) -> bool {
        self.successful_sensor_scans == u32::MAX
            || self.failed_sensor_scans == u32::MAX
            || self.incomplete_profiles == u32::MAX
            || self.i2c_errors == u32::MAX
            || self.radio_tx_errors == u32::MAX
            || self.dropped_profiles == u32::MAX
            || self.dropped_fragments == u32::MAX
            || self.overwritten_fields == u32::MAX
    }

    pub fn record_scan(&mut self, scan: &ProfileScan) {
        let successful = scan_is_successful(scan);
        if successful {
            self.successful_sensor_scans = self.successful_sensor_scans.saturating_add(1);
        } else {
            self.sensor_errors_seen = true;
            self.failed_sensor_scans = self.failed_sensor_scans.saturating_add(1);
            self.last_sensor_error = scan.last_error_code();
        }

        // `incomplete_profiles` means any logical scan not proven complete,
        // error-free, and gas-valid. It includes timeouts with all slots and
        // structurally complete profiles containing invalid gas readings.
        if !successful {
            self.incomplete_profiles = self.incomplete_profiles.saturating_add(1);
        }
        self.i2c_errors = self.i2c_errors.saturating_add(scan.i2c_error_count());
        self.overwritten_fields = self
            .overwritten_fields
            .saturating_add(u32::from(scan.collector.counters().overwritten_fields));
    }

    /// Record one failed output batch. Fragment loss counts only profile
    /// fragments, not optional config/health records sharing the batch.
    pub fn record_output_failure(
        &mut self,
        config_included: bool,
        profile_fragment_count: u8,
        completed_records: u8,
        radio_failure: bool,
    ) {
        if radio_failure {
            self.radio_tx_errors = self.radio_tx_errors.saturating_add(1);
            self.last_radio_error = 1;
        }

        let profile_start = u8::from(config_included);
        let completed_profile_fragments = completed_records
            .saturating_sub(profile_start)
            .min(profile_fragment_count);
        let missing = profile_fragment_count.saturating_sub(completed_profile_fragments);
        if missing != 0 {
            self.dropped_profiles = self.dropped_profiles.saturating_add(1);
            self.dropped_fragments = self.dropped_fragments.saturating_add(u32::from(missing));
        }
    }

    pub fn record_output_success(&mut self, radio_output: bool) {
        if radio_output {
            self.last_radio_error = 0;
        }
    }

    pub fn record_codec_drop(&mut self, expected_fragments: u8) {
        self.dropped_profiles = self.dropped_profiles.saturating_add(1);
        self.dropped_fragments = self
            .dropped_fragments
            .saturating_add(u32::from(expected_fragments));
    }
}

#[must_use]
pub fn scan_is_successful(scan: &ProfileScan) -> bool {
    let collector = &scan.collector;
    let counters = collector.counters();
    logical_profile_success(ProfileQualityEvidence {
        sensor_failed: scan.sensor_failed(),
        i2c_error_count: scan.i2c_error_count(),
        structurally_complete: collector.is_structurally_complete(),
        finish_complete: collector.finish_reason() == Some(ProfileFinishReason::Complete),
        all_steps_gas_valid: collector.all_steps_gas_valid(),
        all_steps_heater_stable: collector.observed_mask() & !collector.heater_stable_mask() == 0,
        overwritten_field_count: counters.overwritten_fields,
        invalid_gas_index_count: counters.invalid_gas_indexes,
        measurement_discontinuity: counters.out_of_order_fields != 0
            || counters.ambiguous_index_jumps != 0
            || counters.profile_rollovers != 0
            || counters.fields_after_rollover != 0,
        observation_overflowed: collector.observed_field_count_overflowed()
            || crate::profile_status::discarded_field_count_overflowed(
                counters.intermediate_fields,
                scan.pre_scan_discarded_fields,
            ),
        stale_pre_scan_field_count: scan.pre_scan_discarded_fields,
    })
}
