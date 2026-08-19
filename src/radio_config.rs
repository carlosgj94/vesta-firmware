//! Canonical raw-LoRa configuration metadata, independent of the radio driver.

pub const FREQUENCY_HZ: u32 = 868_100_000;
pub const TX_POWER_DBM: i32 = 5;
#[cfg(feature = "profile-v2")]
pub const SPREADING_FACTOR: u8 = 7;
#[cfg(feature = "profile-v2")]
pub const BANDWIDTH_HZ: u32 = 125_000;
#[cfg(feature = "profile-v2")]
pub const CODING_RATE_NUMERATOR: u8 = 4;
#[cfg(feature = "profile-v2")]
pub const CODING_RATE_DENOMINATOR: u8 = 5;
pub const PREAMBLE_SYMBOLS: u16 = 8;
#[cfg(feature = "profile-v2")]
pub const SYNC_WORD: u16 = 0x1424;
