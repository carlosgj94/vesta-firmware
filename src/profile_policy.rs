//! Compile-time profile cadence and periodic-record policy.

use embassy_time::Duration;

pub const SCAN_INTERVAL_SECONDS: u32 = parse_decimal_u32(env!("VESTA_SCAN_INTERVAL_SECS"));
pub const SCAN_INTERVAL_MS: u32 = SCAN_INTERVAL_SECONDS * 1_000;
pub const SCAN_INTERVAL: Duration = Duration::from_secs(SCAN_INTERVAL_SECONDS as u64);

/// A radio node repeats its configuration every six scans. The high-rate UART
/// training stream repeats it every scan so each laboratory capture is fully
/// self-describing even when recording begins late.
#[cfg(feature = "profile-v2-uart")]
pub const CONFIG_REPEAT_INTERVAL_SCANS: u16 = 1;
#[cfg(not(feature = "profile-v2-uart"))]
pub const CONFIG_REPEAT_INTERVAL_SCANS: u16 =
    crate::profile_status::LORA_CONFIG_REPEAT_INTERVAL_SCANS;

pub const HEALTH_INTERVAL_SCANS: u32 = 6;

const fn parse_decimal_u32(value: &str) -> u32 {
    let bytes = value.as_bytes();
    let mut parsed = 0_u32;
    let mut index = 0;
    while index < bytes.len() {
        parsed = parsed * 10 + (bytes[index] - b'0') as u32;
        index += 1;
    }
    parsed
}
