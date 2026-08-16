//! Human-readable RTT diagnostics kept separate from measurement logic.

use bme68x::Error as Bme68xError;
use defmt::{error, info};
use embassy_time::{Duration, Timer};

use crate::bme688::{
    HEATER_DURATION_MS, HEATER_TEMPERATURE_CELSIUS, Sensor, SensorError, SetupFailure,
};
use crate::board::SensorProbe;

pub fn log_probe(probe: &SensorProbe) {
    info!(
        "BME688 I2C preflight: address=0x{:02x}, register 0xd0 returned 0x{:02x}",
        probe.address_byte(),
        probe.chip_id()
    );
}

pub fn log_ready(sensor: &Sensor) {
    info!(
        "BME68x initialized: chip_id=0x{:02x}, variant={:?}, address=0x{:02x}, I2C2=100kHz",
        sensor.chip_id(),
        sensor.variant(),
        sensor.address()
    );
    info!("Vesta Rust Embassy firmware started");
    info!(
        "BME688 forced-mode loop: heater={} C for {} ms, conversion wait={} us",
        HEATER_TEMPERATURE_CELSIUS,
        HEATER_DURATION_MS,
        sensor.conversion_wait().as_micros()
    );
}

pub fn log_sensor_error(sensor_error: &SensorError) {
    let operation = sensor_error.operation().label();

    match sensor_error.source() {
        Bme68xError::Bus(bus_error) => {
            error!("BME688 {} failed: I2C error {:?}", operation, bus_error);
        }
        Bme68xError::UnexpectedChipId { found } => {
            error!(
                "BME688 {} failed: expected chip ID 0x61, read 0x{:02x}",
                operation, found
            );
        }
        Bme68xError::InvalidRegisterValue { register, value } => {
            error!(
                "BME688 {} failed: invalid value 0x{:02x} in register 0x{:02x}",
                operation, value, register
            );
        }
        Bme68xError::InvalidConfiguration(configuration_error) => {
            error!(
                "BME688 {} failed: invalid configuration {:?}",
                operation, configuration_error
            );
        }
        Bme68xError::SelfTestFailed(reason) => {
            error!("BME688 {} failed: self-test {:?}", operation, reason);
        }
        Bme68xError::Timeout => {
            error!("BME688 {} failed: sensor timeout", operation);
        }
    }
}

pub async fn halt_after_sensor_error(sensor_error: &SensorError, report_interval: Duration) -> ! {
    log_sensor_error(sensor_error);

    loop {
        Timer::after(report_interval).await;
        error!("BME688 unavailable; reset the board after checking power and I2C wiring");
    }
}

pub async fn halt_after_setup_failure(setup_failure: SetupFailure, report_interval: Duration) -> ! {
    let (sensor, sensor_error) = setup_failure.into_parts();
    log_sensor_error(&sensor_error);

    loop {
        // Retain the initialized sensor state and ownership of its board
        // resources. No concrete I2C driver exists between report intervals.
        let _keep_sensor_alive = &sensor;
        Timer::after(report_interval).await;
        error!("BME688 unavailable; reset the board after checking power and I2C wiring");
    }
}
