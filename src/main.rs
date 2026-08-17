#![no_std]
#![no_main]

mod bme688;
mod board;
mod diagnostics;
mod output;
mod payload;
mod radio;

use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};
use {defmt_rtt as _, panic_probe as _};

// One-minute bring-up placeholder. Change this single value when the deployment
// cadence is selected.
const SAMPLE_INTERVAL_MINUTES: u64 = 1;
const SAMPLE_INTERVAL: Duration = Duration::from_secs(SAMPLE_INTERVAL_MINUTES * 60);
const FAILURE_REPORT_INTERVAL: Duration = Duration::from_secs(5);

#[embassy_executor::main(
    executor = "embassy_stm32::executor::Executor",
    entry = "cortex_m_rt::entry"
)]
async fn main(_spawner: Spawner) {
    let board = board::init();
    let (sensor_bus, radio_resources) = board.into_parts();
    let probe = match board::probe_bme688(sensor_bus).await {
        Ok(probe) => probe,
        Err(probe_failure) => board::halt_after_probe_failure(probe_failure).await,
    };
    diagnostics::log_probe(&probe);

    let sensor = match bme688::Sensor::new(probe) {
        Ok(sensor) => sensor,
        Err(sensor_error) => {
            diagnostics::halt_after_sensor_error(&sensor_error, FAILURE_REPORT_INTERVAL).await;
        }
    };
    let mut sensor = match sensor.configure() {
        Ok(sensor) => sensor,
        Err(setup_failure) => {
            diagnostics::halt_after_setup_failure(setup_failure, FAILURE_REPORT_INTERVAL).await;
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
