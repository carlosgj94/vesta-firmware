//! Target-independent profile outcome and configuration-check policy.

use vesta_protocol_v2::v2::{COLLECTION_FLAG_CONFIG_MISMATCH, COLLECTION_FLAG_SENSOR_RECONFIGURED};

/// Silent BME resets must be detected before the next heater run, not merely
/// when the less-frequent `DeviceConfig` packet is due.
pub const VERIFY_CONFIGURATION_BEFORE_EVERY_SCAN: bool = true;

/// A failed post-Parallel Sleep/recovery gets only this many additional local
/// stop commands. The bound preserves prompt server-visible health instead of
/// allowing a faulty I2C bus to trap the node in an unbounded recovery loop.
pub const EMERGENCY_SLEEP_RETRY_LIMIT: u8 = 3;

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct EmergencySleepRetryProgress {
    pub attempts: u8,
    pub i2c_errors: u32,
    pub confirmed: bool,
}

impl EmergencySleepRetryProgress {
    #[must_use]
    pub const fn should_retry(self) -> bool {
        !self.confirmed && self.attempts < EMERGENCY_SLEEP_RETRY_LIMIT
    }

    pub const fn record_attempt(&mut self, confirmed: bool, i2c_error: bool) {
        self.attempts = self.attempts.saturating_add(1);
        self.confirmed |= confirmed;
        self.i2c_errors = self.i2c_errors.saturating_add(i2c_error as u32);
    }
}

/// Target-independent evidence used for firmware/server quality alignment.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ProfileQualityEvidence {
    pub sensor_failed: bool,
    pub i2c_error_count: u32,
    pub structurally_complete: bool,
    pub finish_complete: bool,
    pub all_steps_gas_valid: bool,
    pub all_steps_heater_stable: bool,
    pub overwritten_field_count: u16,
    pub invalid_gas_index_count: u16,
    pub measurement_discontinuity: bool,
    pub observation_overflowed: bool,
    pub stale_pre_scan_field_count: u16,
}

/// One logical profile is successful only when every condition required by
/// receiver-side profile analysis agrees. Expected repeated observations and
/// discarded Parallel-mode dummy fields are deliberately nonfatal.
#[must_use]
pub const fn logical_profile_success(evidence: ProfileQualityEvidence) -> bool {
    !evidence.sensor_failed
        && evidence.i2c_error_count == 0
        && evidence.structurally_complete
        && evidence.finish_complete
        && evidence.all_steps_gas_valid
        && evidence.all_steps_heater_stable
        && evidence.overwritten_field_count == 0
        && evidence.invalid_gas_index_count == 0
        && !evidence.measurement_discontinuity
        && !evidence.observation_overflowed
        && evidence.stale_pre_scan_field_count == 0
}

/// Decide the independent packet-level `DeviceConfig` repetition cadence.
#[must_use]
pub const fn configuration_check_due(sequence: u32, repeat_interval: u16) -> bool {
    repeat_interval != 0 && sequence.is_multiple_of(repeat_interval as u32)
}

/// Mark every due/forced config definition pending before encoding any later
/// record, so codec or transport failure retries it on the next verified scan.
/// Returns `(include_config_now, pending_after_encode_attempt)`.
#[must_use]
pub const fn device_config_encode_state(
    pending: bool,
    periodic_due: bool,
    configuration_verified: bool,
) -> (bool, bool) {
    let include = configuration_verified && (pending || periodic_due);
    (include, pending || include)
}

#[must_use]
pub const fn device_config_pending_after_delivery(
    pending: bool,
    config_record_completed: bool,
) -> bool {
    pending && !config_record_completed
}

/// Delivery state belongs to one exact configuration ID, not to boot/scan
/// sequence. A changed ID therefore starts again as a first definition even
/// when it appears late in the same boot.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct DeviceConfigDeliveryState {
    pending: bool,
    current_id_transmitted: bool,
}

impl DeviceConfigDeliveryState {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            pending: true,
            current_id_transmitted: false,
        }
    }

    pub const fn record_id_change(&mut self) {
        self.pending = true;
        self.current_id_transmitted = false;
    }

    /// Latch a due definition before any later encoder can fail. Returns
    /// `(include_now, repeated_for_this_exact_id)`.
    pub const fn prepare_encode(
        &mut self,
        periodic_due: bool,
        configuration_verified: bool,
    ) -> (bool, bool) {
        let (include, pending) =
            device_config_encode_state(self.pending, periodic_due, configuration_verified);
        self.pending = pending;
        (include, self.current_id_transmitted)
    }

    pub const fn record_completed(&mut self) {
        self.pending = device_config_pending_after_delivery(self.pending, true);
        self.current_id_transmitted = true;
    }
}

impl Default for DeviceConfigDeliveryState {
    fn default() -> Self {
        Self::new()
    }
}

/// Protocol flags for the result of current readback/recovery, never startup
/// assumptions.
#[must_use]
pub const fn configuration_collection_flags(verified: bool, reconfigured: bool) -> u32 {
    let mut flags = 0;
    if !verified {
        flags |= COLLECTION_FLAG_CONFIG_MISMATCH;
    }
    if verified && reconfigured {
        flags |= COLLECTION_FLAG_SENSOR_RECONFIGURED;
    }
    flags
}

/// Any attempted runtime programming may have partially reset the BME688's
/// temporal state. Preserve a receiver-history marker even when immediate
/// verification/Sleep fails; a later exactly verified, usable profile carries
/// and acknowledges it.
#[must_use]
pub const fn reconfiguration_marker_after_programming_attempt(
    pending: bool,
    programming_attempted: bool,
) -> bool {
    pending || programming_attempted
}

/// Carry a successful restore onto the next exactly verified profile whose
/// Parallel trigger actually succeeds, where the receiver can reset temporal
/// history without relabeling old/no-scan records.
/// Returns `(mark_current_scan, pending_after_current_scan)`. A successful
/// trigger marks the scan but deliberately leaves the marker pending until
/// transport delivery is acknowledged.
#[must_use]
pub const fn carry_pending_reconfiguration(
    pending: bool,
    current_verified: bool,
    current_reconfigured: bool,
    trigger_succeeded: bool,
) -> (bool, bool) {
    let marker = pending || (current_verified && current_reconfigured);
    if current_verified && trigger_succeeded {
        (marker, marker)
    } else {
        (false, marker)
    }
}

/// All profile fragments precede optional health in the deterministic output
/// batch. This detects delivery even when a later health record fails.
#[must_use]
pub const fn profile_delivery_complete(
    config_included: bool,
    profile_fragment_count: u8,
    completed_records: u8,
) -> bool {
    let config_count: u8 = if config_included { 1 } else { 0 };
    profile_fragment_count != 0
        && completed_records >= config_count.saturating_add(profile_fragment_count)
}

/// Keep a temporal-history reset marker until one locally usable marked
/// profile has been delivered in full.
#[must_use]
pub const fn reconfiguration_marker_after_delivery(
    pending: bool,
    scan_marked: bool,
    scan_locally_usable: bool,
    profile_fully_delivered: bool,
) -> bool {
    pending && !(scan_marked && scan_locally_usable && profile_fully_delivered)
}

/// Health describes the sensor state after the scan. It may reuse the scan's
/// pre-trigger config ID only when any post-scan verification produced the
/// exact same metadata snapshot.
#[must_use]
pub const fn health_configuration_identity_available(
    pre_trigger_verified: bool,
    post_scan_checked: bool,
    post_scan_verified: bool,
    post_scan_matches_pre_trigger: bool,
) -> bool {
    pre_trigger_verified
        && (!post_scan_checked || (post_scan_verified && post_scan_matches_pre_trigger))
}

#[must_use]
pub const fn wire_config_id(config_id: u64, identity_available: bool) -> u64 {
    if identity_available { config_id } else { 0 }
}

/// Wire `intermediate_field_count` totals collector-discarded dummy fields and
/// stale pre-trigger NEW_DATA fields, with explicit saturation.
#[must_use]
pub const fn total_discarded_field_count(
    intermediate_fields: u16,
    pre_scan_discarded_fields: u16,
) -> u16 {
    intermediate_fields.saturating_add(pre_scan_discarded_fields)
}

#[must_use]
pub const fn discarded_field_count_overflowed(
    intermediate_fields: u16,
    pre_scan_discarded_fields: u16,
) -> bool {
    intermediate_fields
        .checked_add(pre_scan_discarded_fields)
        .is_none()
}

#[cfg(test)]
mod tests {
    use super::*;

    const GOOD: ProfileQualityEvidence = ProfileQualityEvidence {
        sensor_failed: false,
        i2c_error_count: 0,
        structurally_complete: true,
        finish_complete: true,
        all_steps_gas_valid: true,
        all_steps_heater_stable: true,
        overwritten_field_count: 0,
        invalid_gas_index_count: 0,
        measurement_discontinuity: false,
        observation_overflowed: false,
        stale_pre_scan_field_count: 0,
    };

    #[test]
    fn timeout_with_all_slots_is_still_incomplete() {
        assert!(!logical_profile_success(ProfileQualityEvidence {
            finish_complete: false,
            ..GOOD
        }));
    }

    #[test]
    fn complete_profile_with_invalid_gas_is_still_incomplete() {
        assert!(!logical_profile_success(ProfileQualityEvidence {
            all_steps_gas_valid: false,
            ..GOOD
        }));
    }

    #[test]
    fn only_exact_complete_valid_profile_is_successful() {
        assert!(logical_profile_success(GOOD));
        assert!(!logical_profile_success(ProfileQualityEvidence {
            sensor_failed: true,
            ..GOOD
        }));
        assert!(!logical_profile_success(ProfileQualityEvidence {
            structurally_complete: false,
            ..GOOD
        }));
    }

    #[test]
    fn heater_unstable_profile_is_not_successful() {
        assert!(!logical_profile_success(ProfileQualityEvidence {
            all_steps_heater_stable: false,
            ..GOOD
        }));
    }

    #[test]
    fn overwrite_or_discontinuity_is_not_successful() {
        assert!(!logical_profile_success(ProfileQualityEvidence {
            overwritten_field_count: 1,
            ..GOOD
        }));
        assert!(!logical_profile_success(ProfileQualityEvidence {
            measurement_discontinuity: true,
            ..GOOD
        }));
        assert!(!logical_profile_success(ProfileQualityEvidence {
            invalid_gas_index_count: 1,
            ..GOOD
        }));
        assert!(!logical_profile_success(ProfileQualityEvidence {
            observation_overflowed: true,
            ..GOOD
        }));
    }

    #[test]
    fn recovered_preflight_i2c_error_is_not_successful() {
        assert!(!logical_profile_success(ProfileQualityEvidence {
            i2c_error_count: 1,
            ..GOOD
        }));
    }

    #[test]
    fn stale_pre_scan_field_is_not_successful() {
        assert!(!logical_profile_success(ProfileQualityEvidence {
            stale_pre_scan_field_count: 1,
            ..GOOD
        }));
    }

    #[test]
    fn config_record_is_due_on_each_repeat_boundary() {
        assert!(configuration_check_due(0, 6));
        assert!(!configuration_check_due(5, 6));
        assert!(configuration_check_due(6, 6));
        assert!(!configuration_check_due(6, 0));
    }

    #[test]
    fn changed_verified_config_is_forced_but_mismatch_is_never_advertised() {
        assert_eq!(
            device_config_encode_state(true, configuration_check_due(5, 6), true),
            (true, true)
        );
        assert_eq!(
            device_config_encode_state(false, configuration_check_due(5, 6), true),
            (false, false)
        );
        assert_eq!(
            device_config_encode_state(true, configuration_check_due(6, 6), false),
            (false, true)
        );
    }

    #[test]
    fn initial_config_failure_retries_until_first_record_completes() {
        let (include, pending) = device_config_encode_state(true, true, true);
        assert!(include);
        assert!(device_config_pending_after_delivery(pending, false));
        let (retry, pending) = device_config_encode_state(pending, false, true);
        assert!(retry);
        assert!(!device_config_pending_after_delivery(pending, true));
    }

    #[test]
    fn cadence_or_later_codec_failure_keeps_config_pending() {
        let (include, pending) = device_config_encode_state(false, true, true);
        assert!(include);
        // No transport occurred after a later profile/health codec failure.
        assert!(pending);
        let (retry, _) = device_config_encode_state(pending, false, true);
        assert!(retry);
    }

    #[test]
    fn pending_config_waits_through_config_mismatch() {
        let (include, pending) = device_config_encode_state(true, true, false);
        assert!(!include);
        assert!(pending);
    }

    #[test]
    fn config_repeat_metadata_tracks_exact_id_delivery_not_scan_sequence() {
        let mut state = DeviceConfigDeliveryState::new();

        // Startup attempt and retry are both the first definition until the
        // config-first record actually completes.
        assert_eq!(state.prepare_encode(true, true), (true, false));
        assert_eq!(state.prepare_encode(false, true), (true, false));
        state.record_completed();

        // A periodic record of the same delivered ID is a repeat.
        assert_eq!(state.prepare_encode(true, true), (true, true));
        state.record_completed();

        // A changed mid-boot ID is new again. Codec/output failure leaves it
        // first/non-repeated on the retry despite a nonzero scan sequence.
        state.record_id_change();
        assert_eq!(state.prepare_encode(false, true), (true, false));
        assert_eq!(state.prepare_encode(false, true), (true, false));
        state.record_completed();
        assert_eq!(state.prepare_encode(true, true), (true, true));
    }

    #[test]
    fn config_delivery_state_waits_through_unverified_scan() {
        let mut state = DeviceConfigDeliveryState::new();
        assert_eq!(state.prepare_encode(true, false), (false, false));
        assert_eq!(state.prepare_encode(false, true), (true, false));
    }

    #[test]
    fn sensor_configuration_is_verified_before_every_scan() {
        assert!(core::hint::black_box(
            VERIFY_CONFIGURATION_BEFORE_EVERY_SCAN
        ));
    }

    #[test]
    fn configuration_flags_do_not_claim_failed_recovery() {
        assert_eq!(
            configuration_collection_flags(false, true),
            COLLECTION_FLAG_CONFIG_MISMATCH
        );
        assert_eq!(
            configuration_collection_flags(true, true),
            COLLECTION_FLAG_SENSOR_RECONFIGURED
        );
        assert_eq!(configuration_collection_flags(true, false), 0);
    }

    #[test]
    fn post_scan_reconfiguration_is_carried_only_by_next_verified_scan() {
        assert_eq!(
            carry_pending_reconfiguration(true, false, false, false),
            (false, true)
        );
        assert_eq!(
            carry_pending_reconfiguration(true, true, false, true),
            (true, true)
        );
        assert_eq!(
            carry_pending_reconfiguration(false, true, true, true),
            (true, true)
        );
    }

    #[test]
    fn failed_programming_verification_preserves_future_history_marker() {
        assert!(reconfiguration_marker_after_programming_attempt(
            false, true
        ));
        assert!(reconfiguration_marker_after_programming_attempt(
            true, false
        ));
        assert!(!reconfiguration_marker_after_programming_attempt(
            false, false
        ));
    }

    #[test]
    fn drain_or_trigger_failure_does_not_consume_reconfiguration_marker() {
        assert_eq!(
            carry_pending_reconfiguration(true, true, false, false),
            (false, true)
        );
        assert_eq!(
            carry_pending_reconfiguration(false, true, true, false),
            (false, true)
        );
    }

    #[test]
    fn marker_survives_bad_or_partial_delivery_then_clears_after_full_good_profile() {
        let (bad_marked, pending) = carry_pending_reconfiguration(true, true, false, true);
        assert!(bad_marked);
        let pending = reconfiguration_marker_after_delivery(pending, bad_marked, false, true);
        assert!(pending);

        let (good_marked, pending) = carry_pending_reconfiguration(pending, true, false, true);
        assert!(good_marked);
        assert!(!reconfiguration_marker_after_delivery(
            pending,
            good_marked,
            true,
            true
        ));
    }

    #[test]
    fn profile_delivery_accounts_for_optional_config_and_ignores_later_health() {
        assert!(!profile_delivery_complete(true, 4, 4));
        assert!(profile_delivery_complete(true, 4, 5));
        assert!(profile_delivery_complete(false, 4, 4));
        assert!(!profile_delivery_complete(false, 4, 3));
    }

    #[test]
    fn health_config_identity_requires_matching_successful_postcheck() {
        assert!(health_configuration_identity_available(
            true, false, false, false
        ));
        assert!(!health_configuration_identity_available(
            true, true, false, false
        ));
        assert!(health_configuration_identity_available(
            true, true, true, true
        ));
        assert!(!health_configuration_identity_available(
            true, true, true, false
        ));

        let config_id = 0x9639_2f01_4bce_7745;
        assert_eq!(wire_config_id(config_id, true), config_id);
        assert_eq!(wire_config_id(config_id, false), 0);
    }

    #[test]
    fn stale_pre_scan_fields_are_added_exactly_and_saturate() {
        assert_eq!(total_discarded_field_count(7, 3), 10);
        assert_eq!(total_discarded_field_count(u16::MAX - 1, 3), u16::MAX);
        assert!(!discarded_field_count_overflowed(7, 3));
        assert!(discarded_field_count_overflowed(u16::MAX - 1, 3));
        assert_eq!(
            vesta_protocol_v2::v2::COLLECTION_FLAG_STALE_PRE_SCAN_FIELDS,
            1 << 13
        );
    }

    #[test]
    fn emergency_sleep_retry_is_bounded_and_counts_each_i2c_failure() {
        let mut progress = EmergencySleepRetryProgress::default();
        while progress.should_retry() {
            progress.record_attempt(false, true);
        }
        assert_eq!(progress.attempts, EMERGENCY_SLEEP_RETRY_LIMIT);
        assert_eq!(progress.i2c_errors, u32::from(EMERGENCY_SLEEP_RETRY_LIMIT));
        assert!(!progress.confirmed);
        assert!(!progress.should_retry());

        let mut recovered = EmergencySleepRetryProgress::default();
        recovered.record_attempt(false, true);
        assert!(recovered.should_retry());
        recovered.record_attempt(true, false);
        assert!(recovered.confirmed);
        assert!(!recovered.should_retry());

        let mut saturated = EmergencySleepRetryProgress {
            i2c_errors: u32::MAX,
            ..EmergencySleepRetryProgress::default()
        };
        saturated.record_attempt(false, true);
        assert_eq!(saturated.i2c_errors, u32::MAX);
    }
}
