//! Target-independent profile outcome and configuration-check policy.

use vesta_protocol_v2::v2::{
    COLLECTION_FLAG_CONFIG_MISMATCH, COLLECTION_FLAG_SENSOR_RECONFIGURED, DeviceConfig,
    Error as CodecError, HeaterStepConfig, MAX_PROFILE_STEPS, ProfileStep as WireProfileStep,
    device_config_id,
};

/// Stable IDAC metadata used only by `DeviceConfig` identity.
///
/// The BME688 IDAC registers are read-only in this firmware and can drift even
/// though every programmed heater/configuration register is unchanged. Capture
/// them once when telemetry is established, then reuse those exact bytes when
/// refreshing the configuration record. Per-scan measurements retain their
/// current live IDAC observation independently.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CanonicalIdacSnapshot {
    values: [u8; MAX_PROFILE_STEPS],
}

impl CanonicalIdacSnapshot {
    #[must_use]
    pub const fn capture(values: [u8; MAX_PROFILE_STEPS]) -> Self {
        Self { values }
    }

    /// Replace only active-step configuration IDAC metadata with the snapshot.
    pub fn apply_to(self, config: &mut DeviceConfig) {
        for (step, canonical) in config
            .steps
            .iter_mut()
            .zip(self.values)
            .take(usize::from(config.expected_step_count))
        {
            step.readback_heater_current = canonical;
        }
    }

    /// Canonicalize volatile IDAC metadata before calculating configuration ID.
    pub fn config_id(self, config: &mut DeviceConfig) -> Result<u64, CodecError> {
        self.apply_to(config);
        device_config_id(config)
    }
}

/// Translate one retained sensor field into its exact profile-wire record.
///
/// The heater current comes directly from this scan's measurement. It is
/// deliberately independent from the canonical IDAC bytes used by
/// `DeviceConfig` identity.
#[must_use]
pub const fn wire_profile_step(
    step_index: u8,
    step: bme68x::ProfileStep,
    config: HeaterStepConfig,
) -> WireProfileStep {
    let measurement = step.measurement;
    WireProfileStep {
        step_index,
        gas_index: measurement.gas_index,
        measurement_index: measurement.measurement_index,
        status: measurement.status.bits(),
        raw_measurement_status: measurement.raw_field_status,
        raw_gas_status: measurement.raw_gas_status,
        target_temperature_celsius: config.target_temperature_celsius,
        configured_duration_us: config.configured_duration_us,
        offset_us: step.observed_offset_us,
        temperature_centi_celsius: measurement.values.temperature,
        pressure_pascal: measurement.values.pressure,
        humidity_milli_percent_rh: measurement.values.humidity,
        gas_resistance_ohm: measurement.values.gas_resistance,
        temperature_adc: measurement.raw.temperature_adc,
        pressure_adc: measurement.raw.pressure_adc,
        humidity_adc: measurement.raw.humidity_adc,
        gas_resistance_adc: measurement.raw.gas_resistance_adc,
        gas_range: measurement.raw.gas_range,
        repetition_multiplier: config.repetition_multiplier,
        heater_resistance: measurement.heater_resistance,
        heater_current: measurement.heater_current,
        gas_wait: measurement.gas_wait,
    }
}

/// Compare every sensor configuration field that contributes real identity,
/// deliberately excluding only the unprogrammed volatile IDAC array.
#[must_use]
pub fn sensor_readback_identity_eq(
    left: &bme68x::SensorConfigurationReadback,
    right: &bme68x::SensorConfigurationReadback,
) -> bool {
    left.operation_mode == right.operation_mode
        && left.environmental == right.environmental
        && left.heater.control_gas_0 == right.heater.control_gas_0
        && left.heater.control_gas_1 == right.heater.control_gas_1
        && left.heater.registers.resistance == right.heater.registers.resistance
        && left.heater.registers.gas_wait == right.heater.registers.gas_wait
        && left.heater.registers.shared_duration == right.heater.registers.shared_duration
}

/// Silent BME resets must be detected before the next heater run, not merely
/// when the less-frequent `DeviceConfig` packet is due.
pub const VERIFY_CONFIGURATION_BEFORE_EVERY_SCAN: bool = true;

/// Production LoRa cadence for repeating an already delivered configuration.
#[allow(dead_code)] // The UART training build deliberately repeats every scan.
pub const LORA_CONFIG_REPEAT_INTERVAL_SCANS: u16 = 6;

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

    /// Mark a genuinely changed canonical configuration and report the result.
    pub const fn record_id_change_if_needed(&mut self, current_id: u64, next_id: u64) -> bool {
        if current_id == next_id {
            false
        } else {
            self.record_id_change();
            true
        }
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
    use vesta_protocol_v2::v2::{
        BUILD_FLAG_ID_VALID, CONFIG_FLAG_CALIBRATION_HASH_VALID,
        CONFIG_FLAG_SENSOR_CONFIG_READ_BACK, HeaterStepConfig, MAX_V2_FRAME_LEN_U8,
        OUTPUT_ROUTE_LORA_P2P, STEPS_PER_FRAGMENT_U8,
    };

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

    fn device_config_fixture() -> DeviceConfig {
        let mut steps = [HeaterStepConfig::default(); MAX_PROFILE_STEPS];
        for (index, step) in steps.iter_mut().enumerate() {
            let index = u8::try_from(index).unwrap();
            *step = HeaterStepConfig {
                target_temperature_celsius: 100 + u16::from(index) * 20,
                configured_duration_us: 138_898 * (u32::from(index) + 1),
                repetition_multiplier: index + 1,
                readback_heater_current: 95 + index,
                programmed_heater_resistance: 110 + index,
                programmed_gas_wait: 80 + index,
            };
        }
        DeviceConfig {
            flags: CONFIG_FLAG_CALIBRATION_HASH_VALID | CONFIG_FLAG_SENSOR_CONFIG_READ_BACK,
            firmware_version: [0, 2, 0],
            firmware_build_flags: BUILD_FLAG_ID_VALID,
            firmware_build_id: 0xee37_2943_3e6a_79f7,
            sensor_chip_id: 0x61,
            sensor_variant: 1,
            sensor_i2c_address: 0x76,
            temperature_oversampling: 2,
            humidity_oversampling: 1,
            pressure_oversampling: 5,
            iir_filter: 0,
            standby_time: 0,
            operation_mode: 2,
            heater_enabled: 1,
            parallel_requested_shared_wait_ms: 99,
            parallel_shared_wait_register: 0x73,
            parallel_quantized_shared_wait_us: 97_308,
            tphg_duration_us: 41_590,
            expected_profile_duration_us: 10_695_146,
            profile_id: 0x0354,
            profile_version: 1,
            expected_step_count: 10,
            heater_readback_valid_bitmap: 0x03ff,
            calibration_hash_algorithm: 1,
            calibration_hash: 0xb0b1_b2b3_b4b5_b6b7,
            scan_interval_ms: 180_000,
            config_repeat_interval_scans: LORA_CONFIG_REPEAT_INTERVAL_SCANS,
            output_routes: OUTPUT_ROUTE_LORA_P2P,
            radio_frequency_hz: 868_100_000,
            radio_tx_power_dbm: 5,
            radio_spreading_factor: 7,
            radio_bandwidth_hz: 125_000,
            radio_coding_rate_numerator: 4,
            radio_coding_rate_denominator: 5,
            radio_preamble_symbols: 8,
            radio_header_mode: 0,
            radio_phy_crc_enabled: 1,
            radio_iq_inverted: 0,
            radio_sync_word: 0x1424,
            max_frame_len: MAX_V2_FRAME_LEN_U8,
            profile_steps_per_fragment: STEPS_PER_FRAGMENT_U8,
            steps,
        }
    }

    fn idac_values(config: &DeviceConfig) -> [u8; MAX_PROFILE_STEPS] {
        core::array::from_fn(|index| config.steps[index].readback_heater_current)
    }

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
    fn idac_only_drift_keeps_config_id_and_delivery_idle() {
        let mut initial = device_config_fixture();
        let canonical = CanonicalIdacSnapshot::capture(idac_values(&initial));
        let initial_id = canonical.config_id(&mut initial).unwrap();

        let mut delivery = DeviceConfigDeliveryState::new();
        assert_eq!(delivery.prepare_encode(true, true), (true, false));
        delivery.record_completed();

        let mut later = initial;
        for (index, step) in later.steps.iter_mut().enumerate() {
            step.readback_heater_current = step
                .readback_heater_current
                .wrapping_add(u8::try_from(index % 3 + 1).unwrap());
        }
        let latest_raw_current = later.steps[1].readback_heater_current;
        let later_id = canonical.config_id(&mut later).unwrap();

        assert_eq!(later_id, initial_id);
        assert_eq!(idac_values(&later), idac_values(&initial));
        assert!(!delivery.record_id_change_if_needed(initial_id, later_id));
        assert_eq!(delivery.prepare_encode(false, true), (false, true));
        let emitted = wire_profile_step(
            1,
            bme68x::ProfileStep {
                measurement: bme68x::Measurement {
                    heater_current: latest_raw_current,
                    ..bme68x::Measurement::default()
                },
                observed_offset_us: 123_456,
            },
            later.steps[1],
        );
        assert_eq!(emitted.heater_current, latest_raw_current);
        assert_ne!(
            emitted.heater_current,
            later.steps[1].readback_heater_current
        );
    }

    #[test]
    fn res_heat_or_gas_wait_change_forces_new_non_repeated_config() {
        for mutate in [
            |config: &mut DeviceConfig| {
                config.steps[0].programmed_heater_resistance ^= 1;
            },
            |config: &mut DeviceConfig| {
                config.steps[0].programmed_gas_wait ^= 1;
            },
        ] {
            let mut initial = device_config_fixture();
            let canonical = CanonicalIdacSnapshot::capture(idac_values(&initial));
            let initial_id = canonical.config_id(&mut initial).unwrap();
            let mut changed = initial;
            mutate(&mut changed);
            let changed_id = canonical.config_id(&mut changed).unwrap();

            assert_ne!(changed_id, initial_id);
            let mut delivery = DeviceConfigDeliveryState::new();
            delivery.record_completed();
            assert!(delivery.record_id_change_if_needed(initial_id, changed_id));
            assert_eq!(delivery.prepare_encode(false, true), (true, false));
        }
    }

    #[test]
    fn unchanged_config_repeats_only_at_six_scan_boundary() {
        let mut delivery = DeviceConfigDeliveryState::new();
        assert_eq!(delivery.prepare_encode(true, true), (true, false));
        delivery.record_completed();

        for sequence in 1..LORA_CONFIG_REPEAT_INTERVAL_SCANS as u32 {
            assert_eq!(
                delivery.prepare_encode(
                    configuration_check_due(sequence, LORA_CONFIG_REPEAT_INTERVAL_SCANS),
                    true,
                ),
                (false, true)
            );
        }
        assert_eq!(
            delivery.prepare_encode(
                configuration_check_due(
                    u32::from(LORA_CONFIG_REPEAT_INTERVAL_SCANS),
                    LORA_CONFIG_REPEAT_INTERVAL_SCANS,
                ),
                true,
            ),
            (true, true)
        );
    }

    #[test]
    fn sensor_readback_identity_ignores_only_idac() {
        let mut initial = bme68x::SensorConfigurationReadback {
            operation_mode: bme68x::OperationMode::Sleep,
            environmental: bme68x::Configuration::default(),
            heater: bme68x::HeaterConfigurationReadback::default(),
        };
        initial.heater.registers.current = [95; MAX_PROFILE_STEPS];
        initial.heater.registers.resistance = [110; MAX_PROFILE_STEPS];
        initial.heater.registers.gas_wait = [80; MAX_PROFILE_STEPS];
        initial.heater.registers.shared_duration = 0x73;

        let mut changed = initial;
        changed.heater.registers.current = [172; MAX_PROFILE_STEPS];
        assert!(sensor_readback_identity_eq(&initial, &changed));

        changed.heater.registers.resistance[0] ^= 1;
        assert!(!sensor_readback_identity_eq(&initial, &changed));
        changed = initial;
        changed.heater.registers.gas_wait[0] ^= 1;
        assert!(!sensor_readback_identity_eq(&initial, &changed));
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
