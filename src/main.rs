#![no_std]
#![no_main]

mod bme688;
mod board;
mod diagnostics;

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

const SAMPLE_INTERVAL: Duration = Duration::from_secs(5);

#[embassy_executor::main(
    executor = "embassy_stm32::executor::Executor",
    entry = "cortex_m_rt::entry"
)]
async fn main(_spawner: Spawner) {
    let board = board::init();
    let probe = match board::probe_bme688(board.into_sensor_bus()).await {
        Ok(probe) => probe,
        Err(probe_failure) => board::halt_after_probe_failure(probe_failure).await,
    };
    diagnostics::log_probe(&probe);

    let sensor = match bme688::Sensor::new(probe) {
        Ok(sensor) => sensor,
        Err(sensor_error) => {
            diagnostics::halt_after_sensor_error(&sensor_error, SAMPLE_INTERVAL).await;
        }
    };
    let mut sensor = match sensor.configure() {
        Ok(sensor) => sensor,
        Err(setup_failure) => {
            diagnostics::halt_after_setup_failure(setup_failure, SAMPLE_INTERVAL).await;
        }
    };
    diagnostics::log_ready(&sensor);

    loop {
        match sensor.sample().await {
            Ok(measurements) => diagnostics::log_measurements(&measurements),
            Err(sensor_error) => diagnostics::log_sensor_error(&sensor_error),
        }

        // Forced mode returns the BME688 to sleep automatically. The default
        // build keeps SWD usable; a no-default-features build can use STOP1 in
        // this interval. No radio peripheral is initialized in this firmware.
        Timer::after(SAMPLE_INTERVAL).await;
    }
}
