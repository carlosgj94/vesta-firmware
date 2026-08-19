//! Deterministic translation from driver-owned scans to protocol-v2 records.

use bme68x::ProfileFinishReason;
use embassy_time::Instant;
use vesta_protocol_v2::v2::{
    BUILD_FLAG_DEBUG_SLEEP, BUILD_FLAG_DIRTY, BUILD_FLAG_ID_VALID, COLLECTION_FLAG_DUPLICATE,
    COLLECTION_FLAG_GAS_INDEX_OUT_OF_RANGE, COLLECTION_FLAG_HEATER_UNSTABLE,
    COLLECTION_FLAG_I2C_ERROR, COLLECTION_FLAG_INVALID_GAS,
    COLLECTION_FLAG_MEASUREMENT_DISCONTINUITY, COLLECTION_FLAG_NO_NEW_DATA,
    COLLECTION_FLAG_OBSERVATION_OVERFLOW, COLLECTION_FLAG_OVERWRITTEN,
    COLLECTION_FLAG_POLL_BUDGET_EXHAUSTED, COLLECTION_FLAG_STALE_PRE_SCAN_FIELDS,
    COLLECTION_FLAG_TIMEOUT, CONFIG_FLAG_CALIBRATION_HASH_VALID,
    CONFIG_FLAG_SENSOR_CONFIG_READ_BACK, Common, DeviceConfig, DeviceHealth, EncodedFrame,
    EncodedProfile, Error as CodecError, FINISH_REASON_COMPLETE, FINISH_REASON_POLL_BUDGET,
    FINISH_REASON_PROFILE_ROLLOVER, FINISH_REASON_SENSOR_ERROR, FINISH_REASON_TIMEOUT,
    HEALTH_FLAG_BOOT_ID_UNAVAILABLE, HEALTH_FLAG_CONFIG_MISMATCH, HEALTH_FLAG_COUNTERS_SATURATED,
    HEALTH_FLAG_LAST_SCAN_INCOMPLETE, HEALTH_FLAG_RADIO_ERROR_SEEN, HEALTH_FLAG_SENSOR_ERROR_SEEN,
    HeaterStepConfig, MAX_PROFILE_STEPS, MAX_V2_FRAME_LEN_U8, OUTPUT_ROUTE_LORA_P2P,
    OUTPUT_ROUTE_UART_COBS_CRC32, ProfileScan as WireProfileScan, STEPS_PER_FRAGMENT_U8,
    encode_device_config, encode_device_health, encode_profile,
};

use crate::bme688_profile::{MAX_POLL_COUNT, ProfileScan};
use crate::board::ResetCause;
use crate::health::{HealthCounters, scan_is_successful};
use crate::identity::{
    DeviceIdentity, FIRMWARE_BUILD_DIRTY, FIRMWARE_BUILD_ID, FIRMWARE_BUILD_ID_VALID,
    FIRMWARE_VERSION,
};
use crate::profile_definition::{
    EXPECTED_PROFILE_DURATION_US, PROFILE_ID, PROFILE_REPETITION_MULTIPLIERS, PROFILE_STEP_COUNT,
    PROFILE_TEMPERATURES_CELSIUS, PROFILE_VERSION, REQUESTED_SHARED_DURATION_MS, TPHG_DURATION_US,
    TimeoutBound,
};
use crate::profile_status::{
    CanonicalIdacSnapshot, DeviceConfigDeliveryState, configuration_check_due,
    configuration_collection_flags, discarded_field_count_overflowed, total_discarded_field_count,
    wire_config_id, wire_profile_step,
};
use crate::{bme688_profile, profile_policy, radio_config};

const CALIBRATION_HASH_FNV1A64_CANONICAL_REGISTER_BYTES: u8 = 1;

pub struct EncodedScan {
    pub sequence: u32,
    pub config: Option<EncodedFrame>,
    pub profile: EncodedProfile,
    pub health: Option<EncodedFrame>,
}

impl EncodedScan {
    #[must_use]
    pub fn profile_fragment_count(&self) -> u8 {
        u8::try_from(self.profile.frames().len()).unwrap_or(u8::MAX)
    }
}

pub struct Telemetry {
    identity: DeviceIdentity,
    reset_cause: ResetCause,
    config: DeviceConfig,
    config_id: u64,
    canonical_idac: CanonicalIdacSnapshot,
    next_scan_sequence: u32,
    config_delivery: DeviceConfigDeliveryState,
}

impl Telemetry {
    pub fn new(
        identity: DeviceIdentity,
        reset_cause: ResetCause,
        metadata: &bme688_profile::SensorMetadata,
    ) -> Result<Self, CodecError> {
        let canonical_idac =
            CanonicalIdacSnapshot::capture(metadata.readback.heater.registers.current);
        let mut config = device_config(metadata);
        let config_id = canonical_idac.config_id(&mut config)?;
        Ok(Self {
            identity,
            reset_cause,
            config,
            config_id,
            canonical_idac,
            next_scan_sequence: 0,
            config_delivery: DeviceConfigDeliveryState::new(),
        })
    }

    /// Refresh verified configuration metadata after mandatory per-scan
    /// validation. Volatile, unprogrammed IDAC bytes are restored from the
    /// canonical startup snapshot before hashing; a genuine remaining change
    /// forces its definition into the same output batch.
    pub fn refresh_configuration(
        &mut self,
        metadata: &bme688_profile::SensorMetadata,
    ) -> Result<(), CodecError> {
        let mut config = device_config(metadata);
        let config_id = self.canonical_idac.config_id(&mut config)?;
        if self
            .config_delivery
            .record_id_change_if_needed(self.config_id, config_id)
        {
            self.config = config;
            self.config_id = config_id;
        }
        Ok(())
    }

    /// Preserve a visible sequence gap when a scheduled acquisition cannot be
    /// represented before `encode_scan` gets ownership of it.
    pub fn record_unencoded_scan(&mut self) {
        self.next_scan_sequence = self.next_scan_sequence.wrapping_add(1);
    }

    /// A changed definition remains forced until transport confirms that the
    /// config-first batch completed its first record.
    pub const fn record_config_transmitted(&mut self) {
        self.config_delivery.record_completed();
    }

    /// Encode one logical scan. The sequence advances even if encoding fails,
    /// so a subsequent valid frame exposes the dropped logical scan as a gap.
    pub fn encode_scan(
        &mut self,
        scan: &ProfileScan,
        counters: &HealthCounters,
    ) -> Result<EncodedScan, CodecError> {
        let sequence = self.next_scan_sequence;
        self.next_scan_sequence = self.next_scan_sequence.wrapping_add(1);

        let (include_config, repeated_config) = self.config_delivery.prepare_encode(
            configuration_check_due(sequence, profile_policy::CONFIG_REPEAT_INTERVAL_SCANS),
            scan.configuration_verified,
        );
        let include_health = sequence.is_multiple_of(profile_policy::HEALTH_INTERVAL_SCANS);

        let config = if include_config {
            let uptime_ms = Instant::now().as_millis();
            Some(encode_device_config(
                self.common(sequence, uptime_ms, true),
                &self.config,
                repeated_config,
            )?)
        } else {
            None
        };

        let wire_scan = wire_profile_scan(scan, &self.config);
        let profile = encode_profile(
            self.common(
                sequence,
                scan.started_at.as_millis(),
                scan.configuration_verified,
            ),
            &wire_scan,
        )?;

        let health = if include_health {
            let snapshot = device_health(counters, self.reset_cause.raw, self.identity, scan);
            Some(encode_device_health(
                self.common(
                    sequence,
                    Instant::now().as_millis(),
                    scan.health_configuration_identity_available(),
                ),
                &snapshot,
            )?)
        } else {
            None
        };

        Ok(EncodedScan {
            sequence,
            config,
            profile,
            health,
        })
    }

    fn common(&self, sequence: u32, uptime_ms: u64, configuration_verified: bool) -> Common {
        let config_id = wire_config_id(self.config_id, configuration_verified);
        if self.identity.boot_id_valid && self.identity.boot_id_from_hardware_rng {
            Common::production(
                self.identity.node_id,
                self.identity.boot_id,
                sequence,
                uptime_ms,
                config_id,
                self.reset_cause.flags,
            )
        } else {
            Common::boot_id_unavailable(
                self.identity.node_id,
                sequence,
                uptime_ms,
                config_id,
                self.reset_cause.flags,
            )
        }
    }
}

fn device_config(metadata: &bme688_profile::SensorMetadata) -> DeviceConfig {
    let mut build_flags = 0_u8;
    if FIRMWARE_BUILD_ID_VALID {
        build_flags |= BUILD_FLAG_ID_VALID;
    }
    if FIRMWARE_BUILD_DIRTY {
        build_flags |= BUILD_FLAG_DIRTY;
    }
    if cfg!(feature = "debug-sleep") {
        build_flags |= BUILD_FLAG_DEBUG_SLEEP;
    }

    let readback = &metadata.readback;
    let heater = &readback.heater.registers;
    let mut steps = [HeaterStepConfig::default(); MAX_PROFILE_STEPS];
    for (index, step) in steps
        .iter_mut()
        .take(usize::from(PROFILE_STEP_COUNT))
        .enumerate()
    {
        *step = HeaterStepConfig {
            target_temperature_celsius: PROFILE_TEMPERATURES_CELSIUS[index],
            configured_duration_us: bme68x::compensation::parallel_step_duration_us(
                PROFILE_REPETITION_MULTIPLIERS[index],
                heater.shared_duration,
                TPHG_DURATION_US,
            ),
            repetition_multiplier: PROFILE_REPETITION_MULTIPLIERS[index],
            readback_heater_current: heater.current[index],
            programmed_heater_resistance: heater.resistance[index],
            programmed_gas_wait: heater.gas_wait[index],
        };
    }

    DeviceConfig {
        flags: CONFIG_FLAG_CALIBRATION_HASH_VALID | CONFIG_FLAG_SENSOR_CONFIG_READ_BACK,
        firmware_version: FIRMWARE_VERSION,
        firmware_build_flags: build_flags,
        firmware_build_id: if FIRMWARE_BUILD_ID_VALID {
            FIRMWARE_BUILD_ID
        } else {
            0
        },
        sensor_chip_id: metadata.chip_id,
        sensor_variant: metadata.variant.register_value(),
        sensor_i2c_address: metadata.address,
        temperature_oversampling: readback
            .environmental
            .temperature_oversampling
            .register_value(),
        humidity_oversampling: readback
            .environmental
            .humidity_oversampling
            .register_value(),
        pressure_oversampling: readback
            .environmental
            .pressure_oversampling
            .register_value(),
        iir_filter: readback.environmental.filter.register_value(),
        standby_time: readback.environmental.standby_time.register_value(),
        operation_mode: bme68x::OperationMode::Parallel.register_value(),
        heater_enabled: u8::from(readback.heater.heater_enabled()),
        parallel_requested_shared_wait_ms: REQUESTED_SHARED_DURATION_MS,
        parallel_shared_wait_register: heater.shared_duration,
        parallel_quantized_shared_wait_us: bme68x::compensation::decode_shared_heater_duration_us(
            heater.shared_duration,
        ),
        tphg_duration_us: TPHG_DURATION_US,
        expected_profile_duration_us: expected_profile_duration_us(heater.shared_duration),
        profile_id: PROFILE_ID,
        profile_version: PROFILE_VERSION,
        expected_step_count: PROFILE_STEP_COUNT,
        // Every descriptor's raw IDAC/RES_HEAT/GAS_WAIT bytes came from a
        // successful readback. This does not claim IDAC matched a programmed
        // expectation: the driver preserves IDAC but does not program it.
        heater_readback_valid_bitmap: (1_u16 << PROFILE_STEP_COUNT) - 1,
        calibration_hash_algorithm: CALIBRATION_HASH_FNV1A64_CANONICAL_REGISTER_BYTES,
        calibration_hash: metadata.calibration_fingerprint,
        scan_interval_ms: profile_policy::SCAN_INTERVAL_MS,
        config_repeat_interval_scans: profile_policy::CONFIG_REPEAT_INTERVAL_SCANS,
        output_routes: if cfg!(feature = "profile-v2-uart") {
            OUTPUT_ROUTE_UART_COBS_CRC32
        } else {
            OUTPUT_ROUTE_LORA_P2P
        },
        radio_frequency_hz: radio_config::FREQUENCY_HZ,
        radio_tx_power_dbm: i8::try_from(radio_config::TX_POWER_DBM).unwrap_or(i8::MAX),
        radio_spreading_factor: radio_config::SPREADING_FACTOR,
        radio_bandwidth_hz: radio_config::BANDWIDTH_HZ,
        radio_coding_rate_numerator: radio_config::CODING_RATE_NUMERATOR,
        radio_coding_rate_denominator: radio_config::CODING_RATE_DENOMINATOR,
        radio_preamble_symbols: radio_config::PREAMBLE_SYMBOLS,
        radio_header_mode: 0,
        radio_phy_crc_enabled: 1,
        radio_iq_inverted: 0,
        radio_sync_word: radio_config::SYNC_WORD,
        max_frame_len: MAX_V2_FRAME_LEN_U8,
        profile_steps_per_fragment: STEPS_PER_FRAGMENT_U8,
        steps,
    }
}

fn wire_profile_scan(scan: &ProfileScan, config: &DeviceConfig) -> WireProfileScan {
    let collector = &scan.collector;
    let counters = collector.counters();
    let mut steps = [None; MAX_PROFILE_STEPS];
    for (index, output) in steps
        .iter_mut()
        .take(usize::from(PROFILE_STEP_COUNT))
        .enumerate()
    {
        if let Some(step) = collector.step(u8::try_from(index).unwrap_or(u8::MAX)) {
            *output = Some(wire_profile_step(
                u8::try_from(index).unwrap_or(u8::MAX),
                *step,
                config.steps[index],
            ));
        }
    }

    WireProfileScan {
        profile_id: PROFILE_ID,
        profile_version: PROFILE_VERSION,
        expected_step_count: PROFILE_STEP_COUNT,
        observed_unique_step_count: u8::try_from(collector.observed_steps()).unwrap_or(u8::MAX),
        observed_field_count: collector.observed_field_count(),
        missing_steps_bitmap: collector.missing_mask(),
        duplicate_steps_bitmap: collector.duplicate_mask(),
        scan_duration_us: u32::try_from(scan.duration.as_micros()).unwrap_or(u32::MAX),
        collection_flags: collection_flags(scan),
        finish_reason: finish_reason(scan),
        duplicate_count: counters.duplicates,
        overwritten_field_count: counters.overwritten_fields,
        out_of_order_count: counters.out_of_order_fields,
        ambiguous_index_jump_count: counters.ambiguous_index_jumps,
        invalid_gas_index_count: counters.invalid_gas_indexes,
        intermediate_field_count: total_discarded_field_count(
            counters.intermediate_fields,
            scan.pre_scan_discarded_fields,
        ),
        profile_rollover_count: counters.profile_rollovers,
        fields_after_rollover_count: counters.fields_after_rollover,
        poll_count: scan.poll_count,
        steps,
    }
}

fn collection_flags(scan: &ProfileScan) -> u32 {
    let collector = &scan.collector;
    let counters = collector.counters();
    let mut flags = 0_u32;
    if collector.finish_reason() == Some(ProfileFinishReason::Timeout) {
        match crate::profile_definition::timeout_bound(scan.poll_count, MAX_POLL_COUNT) {
            TimeoutBound::Deadline => flags |= COLLECTION_FLAG_TIMEOUT,
            TimeoutBound::PollBudget => flags |= COLLECTION_FLAG_POLL_BUDGET_EXHAUSTED,
        }
    }
    if scan.i2c_error_count() != 0 {
        flags |= COLLECTION_FLAG_I2C_ERROR;
    }
    if counters.duplicates != 0 {
        flags |= COLLECTION_FLAG_DUPLICATE;
    }
    if counters.overwritten_fields != 0 {
        flags |= COLLECTION_FLAG_OVERWRITTEN;
    }
    if counters.invalid_gas_indexes != 0 {
        flags |= COLLECTION_FLAG_GAS_INDEX_OUT_OF_RANGE;
    }
    if counters.out_of_order_fields != 0
        || counters.ambiguous_index_jumps != 0
        || counters.profile_rollovers != 0
        || counters.fields_after_rollover != 0
    {
        flags |= COLLECTION_FLAG_MEASUREMENT_DISCONTINUITY;
    }
    if collector.observed_field_count() == 0 {
        flags |= COLLECTION_FLAG_NO_NEW_DATA;
    }
    if collector.gas_invalid_mask() != 0 {
        flags |= COLLECTION_FLAG_INVALID_GAS;
    }
    if collector.observed_mask() & !collector.heater_stable_mask() != 0 {
        flags |= COLLECTION_FLAG_HEATER_UNSTABLE;
    }
    if collector.observed_field_count_overflowed()
        || discarded_field_count_overflowed(
            counters.intermediate_fields,
            scan.pre_scan_discarded_fields,
        )
    {
        flags |= COLLECTION_FLAG_OBSERVATION_OVERFLOW;
    }
    if scan.pre_scan_discarded_fields != 0 {
        flags |= COLLECTION_FLAG_STALE_PRE_SCAN_FIELDS;
    }
    flags |= configuration_collection_flags(scan.configuration_verified, scan.sensor_reconfigured);
    flags
}

fn finish_reason(scan: &ProfileScan) -> u8 {
    if scan.sensor_failed() {
        return FINISH_REASON_SENSOR_ERROR;
    }
    match scan.collector.finish_reason() {
        Some(ProfileFinishReason::Complete) => FINISH_REASON_COMPLETE,
        Some(ProfileFinishReason::Timeout) => {
            match crate::profile_definition::timeout_bound(scan.poll_count, MAX_POLL_COUNT) {
                TimeoutBound::Deadline => FINISH_REASON_TIMEOUT,
                TimeoutBound::PollBudget => FINISH_REASON_POLL_BUDGET,
            }
        }
        Some(ProfileFinishReason::ProfileRollover) => FINISH_REASON_PROFILE_ROLLOVER,
        Some(ProfileFinishReason::BusError | ProfileFinishReason::SensorStopped) | None => {
            FINISH_REASON_SENSOR_ERROR
        }
    }
}

fn device_health(
    counters: &HealthCounters,
    reset_cause_raw: u32,
    identity: DeviceIdentity,
    scan: &ProfileScan,
) -> DeviceHealth {
    let mut flags = 0_u8;
    if counters.any_counter_saturated() {
        flags |= HEALTH_FLAG_COUNTERS_SATURATED;
    }
    if !identity.boot_id_valid {
        flags |= HEALTH_FLAG_BOOT_ID_UNAVAILABLE;
    }
    if !scan.health_configuration_identity_available() {
        flags |= HEALTH_FLAG_CONFIG_MISMATCH;
    }
    if !scan_is_successful(scan) {
        flags |= HEALTH_FLAG_LAST_SCAN_INCOMPLETE;
    }
    if counters.sensor_errors_seen {
        flags |= HEALTH_FLAG_SENSOR_ERROR_SEEN;
    }
    if counters.radio_tx_errors != 0 {
        flags |= HEALTH_FLAG_RADIO_ERROR_SEEN;
    }

    DeviceHealth {
        flags,
        reset_cause_raw,
        successful_sensor_scans: counters.successful_sensor_scans,
        failed_sensor_scans: counters.failed_sensor_scans,
        incomplete_profiles: counters.incomplete_profiles,
        i2c_errors: counters.i2c_errors,
        radio_tx_errors: counters.radio_tx_errors,
        dropped_profiles: counters.dropped_profiles,
        dropped_fragments: counters.dropped_fragments,
        overwritten_fields: counters.overwritten_fields,
        current_sample_interval_ms: profile_policy::SCAN_INTERVAL_MS,
        firmware_version: FIRMWARE_VERSION,
        profile_id: PROFILE_ID,
        profile_version: PROFILE_VERSION,
        last_sensor_error: counters.last_sensor_error,
        last_radio_error: counters.last_radio_error,
        // BAT_RAW is not wired to an ADC, and this firmware has not yet
        // implemented factory-calibrated temperature/VREFINT conversion.
        calibrated_mcu_temperature_centi_celsius: None,
        calibrated_vdd_millivolt: None,
    }
}

fn expected_profile_duration_us(shared_duration_register: u8) -> u32 {
    PROFILE_REPETITION_MULTIPLIERS
        .iter()
        .fold(0_u32, |total, multiplier| {
            total.saturating_add(bme68x::compensation::parallel_step_duration_us(
                *multiplier,
                shared_duration_register,
                TPHG_DURATION_US,
            ))
        })
}

const _: () = assert!(EXPECTED_PROFILE_DURATION_US == 10_695_146);
