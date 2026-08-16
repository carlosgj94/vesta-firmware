//! Measurement output boundary.
//!
//! RTT is only a bring-up transport. Replace this function's implementation
//! with the radio transmitter without coupling transmission to sensor control.

use bme68x::Measurements;
use defmt::{info, warn};

pub fn emit(measurements: &Measurements) {
    if measurements.is_empty() {
        warn!("BME688 conversion completed but no new data field was available");
        return;
    }

    for measurement in measurements {
        let values = measurement.values;
        info!(
            "BME688: temperature_centi_c={}, pressure_pa={}, humidity_milli_percent={}, gas_ohms={}, status=0x{:02x}, new={}, gas_valid={}, heater_stable={}",
            values.temperature,
            values.pressure,
            values.humidity,
            values.gas_resistance,
            measurement.status.bits(),
            measurement.status.is_new(),
            measurement.status.gas_valid(),
            measurement.status.heater_stable()
        );
    }
}
