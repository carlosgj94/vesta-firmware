//! Versioned Vesta sensor frame codec.
//!
//! This module is deliberately target-independent. It can be compiled and
//! tested directly on a host without Embassy or STM32 hardware.

pub const FRAME_LEN: usize = 48;
pub const FRAME_MAGIC: [u8; 2] = *b"VS";
pub const FRAME_VERSION: u8 = 1;

/// Every compensated, raw, and heater-metadata value exposed for one BME68x
/// field. Units match the fixed-point `bme68x` API.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SensorFrame {
    pub status: u8,
    pub temperature: i16,
    pub pressure: u32,
    pub humidity: u32,
    pub gas_resistance: u32,
    pub temperature_adc: u32,
    pub pressure_adc: u32,
    pub humidity_adc: u16,
    pub gas_resistance_adc: u16,
    pub gas_range: u8,
    pub gas_index: u8,
    pub measurement_index: u8,
    pub heater_resistance: u8,
    pub heater_current: u8,
    pub gas_wait: u8,
}

/// Encode `>2sBBQIhIIIIIHH6B`, with every multi-byte integer big-endian.
pub fn encode(node_id: u64, sequence: u32, measurement: &SensorFrame) -> [u8; FRAME_LEN] {
    let mut frame = [0_u8; FRAME_LEN];
    frame[0..2].copy_from_slice(&FRAME_MAGIC);
    frame[2] = FRAME_VERSION;
    frame[3] = measurement.status;
    frame[4..12].copy_from_slice(&node_id.to_be_bytes());
    frame[12..16].copy_from_slice(&sequence.to_be_bytes());
    frame[16..18].copy_from_slice(&measurement.temperature.to_be_bytes());
    frame[18..22].copy_from_slice(&measurement.pressure.to_be_bytes());
    frame[22..26].copy_from_slice(&measurement.humidity.to_be_bytes());
    frame[26..30].copy_from_slice(&measurement.gas_resistance.to_be_bytes());
    frame[30..34].copy_from_slice(&measurement.temperature_adc.to_be_bytes());
    frame[34..38].copy_from_slice(&measurement.pressure_adc.to_be_bytes());
    frame[38..40].copy_from_slice(&measurement.humidity_adc.to_be_bytes());
    frame[40..42].copy_from_slice(&measurement.gas_resistance_adc.to_be_bytes());
    frame[42] = measurement.gas_range;
    frame[43] = measurement.gas_index;
    frame[44] = measurement.measurement_index;
    frame[45] = measurement.heater_resistance;
    frame[46] = measurement.heater_current;
    frame[47] = measurement.gas_wait;
    frame
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matches_shared_receiver_fixture() {
        let measurement = SensorFrame {
            status: 0xb0,
            temperature: -1_234,
            pressure: 101_325,
            humidity: 45_678,
            gas_resistance: 987_654,
            temperature_adc: 519_888,
            pressure_adc: 364_576,
            humidity_adc: 30_000,
            gas_resistance_adc: 512,
            gas_range: 8,
            gas_index: 2,
            measurement_index: 3,
            heater_resistance: 4,
            heater_current: 5,
            gas_wait: 6,
        };

        assert_eq!(
            encode(0x0102_0304_0506_0708, 0x0a0b_0c0d, &measurement),
            [
                0x56, 0x53, 0x01, 0xb0, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x0a, 0x0b,
                0x0c, 0x0d, 0xfb, 0x2e, 0x00, 0x01, 0x8b, 0xcd, 0x00, 0x00, 0xb2, 0x6e, 0x00, 0x0f,
                0x12, 0x06, 0x00, 0x07, 0xee, 0xd0, 0x00, 0x05, 0x90, 0x20, 0x75, 0x30, 0x02, 0x00,
                0x08, 0x02, 0x03, 0x04, 0x05, 0x06,
            ]
        );
    }

    #[test]
    fn preserves_signed_and_unsigned_edge_values() {
        let minimum = SensorFrame {
            status: 0,
            temperature: i16::MIN,
            pressure: 0,
            humidity: 0,
            gas_resistance: 0,
            temperature_adc: 0,
            pressure_adc: 0,
            humidity_adc: 0,
            gas_resistance_adc: 0,
            gas_range: 0,
            gas_index: 0,
            measurement_index: 0,
            heater_resistance: 0,
            heater_current: 0,
            gas_wait: 0,
        };
        let frame = encode(0, 0, &minimum);
        assert_eq!(&frame[0..4], &[b'V', b'S', 1, 0]);
        assert_eq!(&frame[16..18], &[0x80, 0x00]);
        assert!(frame[4..16].iter().all(|byte| *byte == 0));
        assert!(frame[18..].iter().all(|byte| *byte == 0));

        let maximum = SensorFrame {
            status: u8::MAX,
            temperature: i16::MAX,
            pressure: u32::MAX,
            humidity: u32::MAX,
            gas_resistance: u32::MAX,
            temperature_adc: u32::MAX,
            pressure_adc: u32::MAX,
            humidity_adc: u16::MAX,
            gas_resistance_adc: u16::MAX,
            gas_range: u8::MAX,
            gas_index: u8::MAX,
            measurement_index: u8::MAX,
            heater_resistance: u8::MAX,
            heater_current: u8::MAX,
            gas_wait: u8::MAX,
        };
        let frame = encode(u64::MAX, u32::MAX, &maximum);
        assert_eq!(&frame[0..3], &[b'V', b'S', 1]);
        assert_eq!(frame[3], u8::MAX);
        assert!(frame[4..16].iter().all(|byte| *byte == u8::MAX));
        assert_eq!(&frame[16..18], &[0x7f, 0xff]);
        assert!(frame[18..].iter().all(|byte| *byte == u8::MAX));
    }
}
