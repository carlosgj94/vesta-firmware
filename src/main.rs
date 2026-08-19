#![no_std]
#![no_main]

#[cfg(all(feature = "telemetry-v1", feature = "profile-v2"))]
compile_error!("select exactly one telemetry path: telemetry-v1 or profile-v2");
#[cfg(not(any(feature = "telemetry-v1", feature = "profile-v2")))]
compile_error!("select exactly one telemetry path: telemetry-v1 or profile-v2");
#[cfg(all(feature = "profile-v2-uart", not(feature = "profile-v2")))]
compile_error!("profile-v2-uart requires profile-v2");

// Both driver releases intentionally appear to their feature-gated modules as
// `bme68x`; this prevents profile-v2 work from changing the deployed v1 build.
#[cfg(all(feature = "telemetry-v1", not(feature = "profile-v2")))]
extern crate bme68x_v1 as bme68x;
#[cfg(all(feature = "profile-v2", not(feature = "telemetry-v1")))]
extern crate bme68x_v2 as bme68x;

#[cfg(all(feature = "telemetry-v1", not(feature = "profile-v2")))]
mod bme688;
#[cfg(all(feature = "profile-v2", not(feature = "telemetry-v1")))]
mod bme688_profile;
#[cfg(any(
    all(feature = "telemetry-v1", not(feature = "profile-v2")),
    all(feature = "profile-v2", not(feature = "telemetry-v1"))
))]
mod board;
#[cfg(all(feature = "profile-v2", not(feature = "telemetry-v1")))]
mod degraded_health;
#[cfg(all(feature = "telemetry-v1", not(feature = "profile-v2")))]
mod diagnostics;
#[cfg(all(feature = "profile-v2", not(feature = "telemetry-v1")))]
mod diagnostics_v2;
#[cfg(all(feature = "profile-v2-uart", not(feature = "telemetry-v1")))]
mod framing;
#[cfg(all(feature = "profile-v2", not(feature = "telemetry-v1")))]
mod health;
#[cfg(all(feature = "profile-v2", not(feature = "telemetry-v1")))]
mod identity;
#[cfg(all(feature = "telemetry-v1", not(feature = "profile-v2")))]
mod output;
#[cfg(all(feature = "profile-v2", not(feature = "telemetry-v1")))]
mod output_v2;
#[cfg(all(feature = "telemetry-v1", not(feature = "profile-v2")))]
mod payload;
#[cfg(all(feature = "profile-v2", not(feature = "telemetry-v1")))]
mod profile_definition;
#[cfg(all(feature = "profile-v2", not(feature = "telemetry-v1")))]
mod profile_policy;
#[cfg(all(feature = "profile-v2", not(feature = "telemetry-v1")))]
mod profile_status;
#[cfg(any(
    all(feature = "telemetry-v1", not(feature = "profile-v2")),
    all(
        feature = "profile-v2",
        not(feature = "telemetry-v1"),
        not(feature = "profile-v2-uart")
    )
))]
mod radio;
#[cfg(any(
    all(feature = "telemetry-v1", not(feature = "profile-v2")),
    all(feature = "profile-v2", not(feature = "telemetry-v1"))
))]
mod radio_config;
#[cfg(any(
    all(feature = "telemetry-v1", not(feature = "profile-v2")),
    all(
        feature = "profile-v2",
        not(feature = "telemetry-v1"),
        not(feature = "profile-v2-uart")
    )
))]
mod rf_switch;
#[cfg(all(feature = "profile-v2", not(feature = "telemetry-v1")))]
mod telemetry_v2;

use embassy_executor::Spawner;
#[cfg(all(feature = "telemetry-v1", not(feature = "profile-v2")))]
use embassy_time::Duration;
#[cfg(any(
    all(feature = "telemetry-v1", not(feature = "profile-v2")),
    all(feature = "profile-v2", not(feature = "telemetry-v1"))
))]
use embassy_time::Ticker;
use {defmt_rtt as _, panic_probe as _};

// One-minute preserved bring-up cadence. Change this single value when the
// deployment cadence is selected.
#[cfg(all(feature = "telemetry-v1", not(feature = "profile-v2")))]
const SAMPLE_INTERVAL_MINUTES: u64 = 1;
#[cfg(all(feature = "telemetry-v1", not(feature = "profile-v2")))]
const SAMPLE_INTERVAL: Duration = Duration::from_secs(SAMPLE_INTERVAL_MINUTES * 60);
#[cfg(all(feature = "telemetry-v1", not(feature = "profile-v2")))]
const FAILURE_REPORT_INTERVAL: Duration = Duration::from_secs(5);

#[embassy_executor::main(
    executor = "embassy_stm32::executor::Executor",
    entry = "cortex_m_rt::entry"
)]
async fn main(_spawner: Spawner) {
    #[cfg(all(feature = "telemetry-v1", not(feature = "profile-v2")))]
    run_v1().await;

    #[cfg(all(feature = "profile-v2", not(feature = "telemetry-v1")))]
    run_profile_v2().await;
}

#[cfg(all(feature = "telemetry-v1", not(feature = "profile-v2")))]
async fn run_v1() -> ! {
    let board = board::init();
    let (sensor_bus, radio_resources) = board.into_parts();
    let probe = match board::probe_bme688(sensor_bus).await {
        Ok(probe) => probe,
        Err(probe_failure) => board::halt_after_probe_failure(probe_failure, radio_resources).await,
    };
    diagnostics::log_probe(&probe);

    let sensor = match bme688::Sensor::new(probe) {
        Ok(sensor) => sensor,
        Err(sensor_error) => {
            diagnostics::halt_after_sensor_error(
                &sensor_error,
                radio_resources,
                FAILURE_REPORT_INTERVAL,
            )
            .await;
        }
    };
    let mut sensor = match sensor.configure() {
        Ok(sensor) => sensor,
        Err(setup_failure) => {
            diagnostics::halt_after_setup_failure(
                setup_failure,
                radio_resources,
                FAILURE_REPORT_INTERVAL,
            )
            .await;
        }
    };
    diagnostics::log_ready(&sensor);
    let mut output = output::RadioOutput::new(radio_resources);

    // Ticker anchors each wake-up to a fixed deadline. Measurement and future
    // radio-transmission time therefore do not accumulate into schedule drift.
    let mut sample_schedule = Ticker::every(SAMPLE_INTERVAL);

    loop {
        match sensor.sample().await {
            Ok(measurements) => {
                if let Err(radio_error) = output.emit(&measurements).await {
                    diagnostics::log_radio_error(&radio_error);
                }
            }
            Err(sensor_error) => diagnostics::log_sensor_error(&sensor_error),
        }

        // Forced mode returns the BME688 to sleep automatically. The production
        // build can use STOP2 during this wait because the concrete I2C2 and
        // SUBGHZSPI drivers exist only during individual transactions.
        sample_schedule.next().await;
    }
}

#[cfg(all(feature = "profile-v2", not(feature = "telemetry-v1")))]
async fn run_profile_v2() -> ! {
    use crate::board::ProfileBoardParts;
    use crate::health::HealthCounters;
    use crate::output_v2::ProfileOutput;
    use vesta_protocol_v2::v2::MAX_PROFILE_FRAGMENTS;

    let expected_fragment_count = u8::try_from(MAX_PROFILE_FRAGMENTS).unwrap_or(u8::MAX);

    let parts = board::init().into_profile_parts();
    let ProfileBoardParts {
        sensor_bus,
        #[cfg(not(feature = "profile-v2-uart"))]
        radio,
        rng,
        #[cfg(feature = "profile-v2-uart")]
        training_uart,
        reset_cause,
    } = parts;

    // Generate the per-boot nonce once, then drop/disable RNG before any
    // scheduled idle so the RNG's Stop1 constraint cannot block STOP2.
    let identity = identity::establish(rng).await;

    // Own the output tokens before touching the sensor. Every startup failure
    // can therefore report a health-only record before one bounded retry.
    #[cfg(not(feature = "profile-v2-uart"))]
    let mut output = ProfileOutput::new(radio);
    #[cfg(feature = "profile-v2-uart")]
    let mut output = ProfileOutput::new(training_uart);

    let degraded_context = degraded_context(identity, reset_cause);
    let mut degraded_reporter = None;
    let mut startup_retry_schedule = Ticker::every(profile_policy::SCAN_INTERVAL);

    // A probe failure returns every I2C ownership token, so retrying it cannot
    // require unsafe peripheral recreation. Each failed pair of concrete
    // address transactions is emitted and followed by a STOP2-eligible wait.
    let mut available_bus = sensor_bus;
    let probe = loop {
        match board::probe_bme688(available_bus).await {
            Ok(probe) => break probe,
            Err(probe_failure) => {
                available_bus = board::diagnose_probe_failure(probe_failure).await;
                record_startup_failure(
                    &mut degraded_reporter,
                    degraded_context,
                    degraded_health::StartupFailure::bme_probe(),
                );
                emit_degraded_health_once(&mut degraded_reporter, &mut output).await;
                startup_retry_schedule.next().await;
            }
        }
    };

    let sensor = match bme688_profile::Sensor::new(probe) {
        Ok(sensor) => sensor,
        Err(sensor_error) => {
            diagnostics_v2::log_sensor_error(&sensor_error);
            let failure = degraded_health::StartupFailure::sensor_operation(
                sensor_error.operation().code(),
                sensor_error.is_i2c_error(),
            );
            record_startup_failure(&mut degraded_reporter, degraded_context, failure);
            // Bme68x::new consumes its interface and does not return it on
            // error. With no sound way to recreate Embassy ownership tokens,
            // this narrow initialization-failure state requires reset while
            // health records continue at the safe cadence.
            let reporter = degraded_reporter
                .take()
                .unwrap_or_else(|| degraded_health::Reporter::new(degraded_context, failure));
            report_unrecoverable_degraded_health((), output, reporter, startup_retry_schedule)
                .await;
        }
    };
    let mut sensor = sensor;
    let mut telemetry = loop {
        sensor = match sensor.configure() {
            Ok(sensor) => sensor,
            Err(setup_failure) => {
                let (retained_sensor, sensor_error) = setup_failure.into_parts();
                sensor = retained_sensor;
                diagnostics_v2::log_sensor_error(&sensor_error);
                let failure = degraded_health::StartupFailure::sensor_operation(
                    sensor_error.operation().code(),
                    sensor_error.is_i2c_error(),
                );
                record_startup_failure(&mut degraded_reporter, degraded_context, failure);
                emit_degraded_health_once(&mut degraded_reporter, &mut output).await;
                startup_retry_schedule.next().await;
                continue;
            }
        };

        let Some(metadata) = sensor.metadata().copied() else {
            diagnostics_v2::log_internal_error("configured sensor metadata missing");
            record_startup_failure(
                &mut degraded_reporter,
                degraded_context,
                degraded_health::StartupFailure::sensor_metadata_missing(),
            );
            emit_degraded_health_once(&mut degraded_reporter, &mut output).await;
            startup_retry_schedule.next().await;
            continue;
        };
        if !metadata.exact_configuration_verified {
            diagnostics_v2::log_internal_error("initial HP-354 configuration readback mismatch");
            record_startup_failure(
                &mut degraded_reporter,
                degraded_context,
                degraded_health::StartupFailure::sensor_operation(
                    bme688_profile::Operation::ConfigurationReadback.code(),
                    false,
                ),
            );
            emit_degraded_health_once(&mut degraded_reporter, &mut output).await;
            startup_retry_schedule.next().await;
            continue;
        }

        match telemetry_v2::Telemetry::new(identity, reset_cause, &metadata) {
            Ok(telemetry) => break telemetry,
            Err(codec_error) => {
                diagnostics_v2::log_internal_error("DeviceConfig is not encodable");
                diagnostics_v2::log_codec_error(&codec_error);
                record_startup_failure(
                    &mut degraded_reporter,
                    degraded_context,
                    degraded_health::StartupFailure::telemetry_setup(),
                );
                emit_degraded_health_once(&mut degraded_reporter, &mut output).await;
                startup_retry_schedule.next().await;
            }
        }
    };
    diagnostics_v2::log_ready(&sensor);

    let mut counters = HealthCounters::default();
    if let Some(reporter) = degraded_reporter {
        counters.sensor_errors_seen = true;
        counters.i2c_errors = reporter.i2c_errors();
        counters.radio_tx_errors = reporter.radio_tx_errors();
        counters.last_sensor_error = reporter.last_sensor_error();
        counters.last_radio_error = reporter.last_radio_error();
    }
    let mut scan_schedule = Ticker::every(profile_policy::SCAN_INTERVAL);

    loop {
        // A fresh readback precedes every 10-step heater run. A silent BME688
        // reset therefore cannot leave even one knowingly mislabeled profile.
        let scan = match sensor
            .scan(profile_status::VERIFY_CONFIGURATION_BEFORE_EVERY_SCAN)
            .await
        {
            Ok(scan) => scan,
            Err(collector_error) => {
                // The only constructor error would be an invalid compile-time
                // profile length/mode. Do not panic or touch the radio.
                diagnostics_v2::log_collector_error(&collector_error);
                counters.failed_sensor_scans = counters.failed_sensor_scans.saturating_add(1);
                counters.incomplete_profiles = counters.incomplete_profiles.saturating_add(1);
                counters.record_codec_drop(expected_fragment_count);
                telemetry.record_unencoded_scan();
                scan_schedule.next().await;
                continue;
            }
        };
        counters.record_scan(&scan);

        if scan.configuration_verified {
            let Some(pre_trigger_metadata) = scan.pre_trigger_metadata.as_ref() else {
                diagnostics_v2::log_internal_error(
                    "verified scan lost its pre-trigger configuration snapshot",
                );
                counters.record_codec_drop(expected_fragment_count);
                telemetry.record_unencoded_scan();
                scan_schedule.next().await;
                continue;
            };
            if let Err(codec_error) = telemetry.refresh_configuration(pre_trigger_metadata) {
                diagnostics_v2::log_codec_error(&codec_error);
                counters.record_codec_drop(expected_fragment_count);
                telemetry.record_unencoded_scan();
                scan_schedule.next().await;
                continue;
            }
        }

        let encoded = match telemetry.encode_scan(&scan, &counters) {
            Ok(encoded) => encoded,
            Err(codec_error) => {
                diagnostics_v2::log_codec_error(&codec_error);
                counters.record_codec_drop(expected_fragment_count);
                scan_schedule.next().await;
                continue;
            }
        };
        diagnostics_v2::log_scan(encoded.sequence, &scan);

        let config_included = encoded.config.is_some();
        let profile_fragment_count = encoded.profile_fragment_count();
        let scan_locally_usable = health::scan_is_successful(&scan);
        match output
            .emit(
                encoded.config.as_ref(),
                &encoded.profile,
                encoded.health.as_ref(),
            )
            .await
        {
            Ok(report) => {
                if scan.sensor_reconfigured
                    && !profile_status::reconfiguration_marker_after_delivery(
                        true,
                        true,
                        scan_locally_usable,
                        true,
                    )
                {
                    sensor.acknowledge_reconfiguration_marker();
                }
                if config_included {
                    telemetry.record_config_transmitted();
                }
                counters.record_output_success(!cfg!(feature = "profile-v2-uart"));
                diagnostics_v2::log_emit_success(&report);
            }
            Err(output_error) => {
                let profile_fully_delivered = profile_status::profile_delivery_complete(
                    config_included,
                    profile_fragment_count,
                    output_error.completed_records,
                );
                if scan.sensor_reconfigured
                    && !profile_status::reconfiguration_marker_after_delivery(
                        true,
                        true,
                        scan_locally_usable,
                        profile_fully_delivered,
                    )
                {
                    sensor.acknowledge_reconfiguration_marker();
                }
                if config_included && output_error.completed_records != 0 {
                    telemetry.record_config_transmitted();
                }
                #[cfg(not(feature = "profile-v2-uart"))]
                let radio_failure = matches!(output_error.source, output_v2::Source::Radio(_));
                #[cfg(feature = "profile-v2-uart")]
                let radio_failure = false;
                counters.record_output_failure(
                    config_included,
                    profile_fragment_count,
                    output_error.completed_records,
                    radio_failure,
                );
                diagnostics_v2::log_emit_error(&output_error);
            }
        }

        // Sensor, RNG, UART/radio, and their concrete RCC drivers are all
        // asleep or dropped here. The production build remains STOP2-eligible;
        // debug-sleep intentionally substitutes shallow debugger-safe idle.
        scan_schedule.next().await;
    }
}

#[cfg(all(feature = "profile-v2", not(feature = "telemetry-v1")))]
const fn degraded_context(
    identity: identity::DeviceIdentity,
    reset_cause: board::ResetCause,
) -> degraded_health::Context {
    degraded_health::Context {
        node_id: identity.node_id,
        boot_id: identity.boot_id,
        boot_id_valid: identity.boot_id_valid,
        boot_id_from_hardware_rng: identity.boot_id_from_hardware_rng,
        reset_cause_raw: reset_cause.raw,
        reset_cause_flags: reset_cause.flags,
        sample_interval_ms: profile_policy::SCAN_INTERVAL_MS,
        firmware_version: identity::FIRMWARE_VERSION,
    }
}

#[cfg(all(feature = "profile-v2", not(feature = "telemetry-v1")))]
fn record_startup_failure(
    reporter: &mut Option<degraded_health::Reporter>,
    context: degraded_health::Context,
    failure: degraded_health::StartupFailure,
) {
    if let Some(reporter) = reporter {
        reporter.record_startup_failure(failure);
    } else {
        *reporter = Some(degraded_health::Reporter::new(context, failure));
    }
}

/// Emit exactly one server-visible startup-health attempt. The caller retains
/// ownership and decides whether to retry setup or enter the reset-required
/// health loop. The transport's concrete driver is always short lived.
#[cfg(all(feature = "profile-v2", not(feature = "telemetry-v1")))]
async fn emit_degraded_health_once(
    reporter: &mut Option<degraded_health::Reporter>,
    output: &mut output_v2::ProfileOutput,
) {
    let Some(reporter) = reporter else {
        return;
    };
    let encoded = match reporter.encode_next(embassy_time::Instant::now().as_millis()) {
        Ok(encoded) => encoded,
        Err(codec_error) => {
            reporter.record_output_failure(false);
            diagnostics_v2::log_codec_error(&codec_error);
            diagnostics_v2::log_degraded_retry(reporter.output_failures());
            return;
        }
    };

    match output.emit_health(&encoded.frame).await {
        Ok(report) => {
            reporter.record_output_success(!cfg!(feature = "profile-v2-uart"));
            diagnostics_v2::log_degraded_emit_success(
                encoded.sequence,
                reporter.output_failures(),
                &report,
            );
        }
        Err(output_error) => {
            #[cfg(not(feature = "profile-v2-uart"))]
            let radio_failure = matches!(output_error.source, output_v2::Source::Radio(_));
            #[cfg(feature = "profile-v2-uart")]
            let radio_failure = false;
            reporter.record_output_failure(radio_failure);
            diagnostics_v2::log_emit_error(&output_error);
            diagnostics_v2::log_degraded_retry(reporter.output_failures());
        }
    }
}

/// Retry server-visible health after the sole startup failure that cannot
/// safely recover ownership (`Bme68x::new` consumes its interface on error).
/// `retained` keeps any remaining tokens alive without a concrete peripheral
/// driver, so production waits remain STOP2 eligible.
#[cfg(all(feature = "profile-v2", not(feature = "telemetry-v1")))]
async fn report_unrecoverable_degraded_health<T>(
    retained: T,
    mut output: output_v2::ProfileOutput,
    reporter: degraded_health::Reporter,
    mut retry_schedule: Ticker,
) -> ! {
    let mut reporter = Some(reporter);

    loop {
        emit_degraded_health_once(&mut reporter, &mut output).await;
        let _keep_owned_resources = &retained;
        retry_schedule.next().await;
    }
}
