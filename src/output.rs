//! Measurement output boundary.

use bme68x::{Measurement, Measurements};
use defmt::{info, warn};
use lora_phy::mod_params::RadioError;

use crate::board::RadioResources;
use crate::payload::{self, SensorFrame};
use crate::radio;

// `bme68x::Measurements` owns exactly three FIFO-style field slots.
const MAX_MEASUREMENT_FIELDS: usize = 3;

/// Stateful radio output. The radio hardware driver itself is intentionally
/// short-lived; only ownership tokens and application metadata survive STOP2.
pub struct RadioOutput {
    resources: RadioResources,
    node_id: u64,
    sequence: u32,
}

impl RadioOutput {
    pub fn new(resources: RadioResources) -> Self {
        Self {
            resources,
            node_id: node_id(),
            sequence: 0,
        }
    }

    /// Send every new BME68x field as one fixed-width, versioned P2P frame.
    ///
    /// All fields from one sensor read share a single cold radio session. This
    /// avoids repeating reset, calibration, and TCXO startup if the sensor ever
    /// returns more than one of its three FIFO-style fields.
    pub async fn emit(&mut self, measurements: &Measurements) -> Result<(), RadioError> {
        if measurements.is_empty() {
            warn!("BME688 conversion completed but no new data field was available");
            return Ok(());
        }

        let first_sequence = self.sequence;
        let mut frames = [[0_u8; payload::FRAME_LEN]; MAX_MEASUREMENT_FIELDS];
        let mut frame_count = 0;

        for (measurement, frame) in measurements.iter().zip(frames.iter_mut()) {
            let sequence = self.sequence;
            self.sequence = self.sequence.wrapping_add(1);
            *frame = payload::encode(self.node_id, sequence, &sensor_frame(measurement));
            frame_count += 1;
        }

        // The driver currently guarantees this invariant. Return a bounded
        // error instead of silently truncating or panicking if its API changes.
        if frame_count != measurements.len() {
            return Err(RadioError::PayloadSizeUnexpected(measurements.len()));
        }

        radio::transmit(&mut self.resources, &frames[..frame_count]).await?;
        info!(
            "LoRa P2P frames sent: node_id=0x{:016x}, first_sequence={}, count={}, bytes_each={}",
            self.node_id,
            first_sequence,
            frame_count,
            payload::FRAME_LEN
        );

        Ok(())
    }
}

fn sensor_frame(measurement: &Measurement) -> SensorFrame {
    SensorFrame {
        status: measurement.status.bits(),
        temperature: measurement.values.temperature,
        pressure: measurement.values.pressure,
        humidity: measurement.values.humidity,
        gas_resistance: measurement.values.gas_resistance,
        temperature_adc: measurement.raw.temperature_adc,
        pressure_adc: measurement.raw.pressure_adc,
        humidity_adc: measurement.raw.humidity_adc,
        gas_resistance_adc: measurement.raw.gas_resistance_adc,
        gas_range: measurement.raw.gas_range,
        gas_index: measurement.gas_index,
        measurement_index: measurement.measurement_index,
        heater_resistance: measurement.heater_resistance,
        heater_current: measurement.heater_current,
        gas_wait: measurement.gas_wait,
    }
}

/// Hash all 96 hardware-UID bits into the stable 64-bit node identifier carried
/// on air. FNV-1a is used for identity compaction, not for security.
fn node_id() -> u64 {
    let mut hash = 0xcbf2_9ce4_8422_2325_u64;
    for byte in embassy_stm32::uid::uid() {
        hash ^= u64::from(byte);
        hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
    }
    hash
}
