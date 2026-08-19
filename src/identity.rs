//! Stable device identity plus a per-boot hardware-random nonce.

use embassy_stm32::bind_interrupts;
use embassy_stm32::peripherals;
use embassy_stm32::rng::{self, Rng};
use embassy_time::{Duration, with_timeout};

const RNG_TIMEOUT: Duration = Duration::from_millis(100);
const RNG_ATTEMPTS: usize = 3;

bind_interrupts!(struct RngIrqs {
    RNG => rng::InterruptHandler<peripherals::RNG>;
});

/// Identity metadata established once at boot.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct DeviceIdentity {
    pub node_id: u64,
    pub boot_id: u64,
    pub boot_id_valid: bool,
    pub boot_id_from_hardware_rng: bool,
}

/// Generate the boot nonce and drop/disable RNG before normal operation.
///
/// No deterministic value is presented as a nonce if all bounded attempts
/// fail. Such a boot remains observable with `boot_id_valid = false` and zero
/// on the wire rather than fabricating uniqueness.
pub async fn establish(
    rng_peripheral: embassy_stm32::Peri<'static, peripherals::RNG>,
) -> DeviceIdentity {
    let mut rng = Rng::new(rng_peripheral, RngIrqs);
    let mut bytes = [0_u8; 8];
    let mut valid = false;

    for _ in 0..RNG_ATTEMPTS {
        if let Ok(Ok(())) = with_timeout(RNG_TIMEOUT, rng.async_fill_bytes(&mut bytes)).await {
            valid = true;
            break;
        }
    }

    // `Drop` disables both RNG and its RCC clock, removing the peripheral's
    // Stop1 requirement before any scheduled sleep.
    drop(rng);

    DeviceIdentity {
        node_id: stable_node_id(),
        boot_id: if valid { u64::from_be_bytes(bytes) } else { 0 },
        boot_id_valid: valid,
        boot_id_from_hardware_rng: valid,
    }
}

/// Hash all 96 hardware-UID bits into the stable v1/v2 node identifier.
/// FNV-1a is identity compaction, not authentication or secrecy.
#[must_use]
pub fn stable_node_id() -> u64 {
    let mut hash = 0xcbf2_9ce4_8422_2325_u64;
    for byte in embassy_stm32::uid::uid() {
        hash ^= u64::from(byte);
        hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
    }
    hash
}

pub const FIRMWARE_VERSION: [u8; 3] = [
    parse_decimal_u8(env!("CARGO_PKG_VERSION_MAJOR")),
    parse_decimal_u8(env!("CARGO_PKG_VERSION_MINOR")),
    parse_decimal_u8(env!("CARGO_PKG_VERSION_PATCH")),
];

pub const FIRMWARE_BUILD_ID: u64 = parse_hex_u64(env!("VESTA_BUILD_ID_HEX"));
pub const FIRMWARE_BUILD_ID_VALID: bool = !env!("VESTA_BUILD_ID_HEX").is_empty();
pub const FIRMWARE_BUILD_DIRTY: bool = parse_decimal_u8(env!("VESTA_BUILD_DIRTY")) != 0;

const fn parse_decimal_u8(value: &str) -> u8 {
    let bytes = value.as_bytes();
    let mut parsed = 0_u8;
    let mut index = 0;
    while index < bytes.len() {
        let digit = bytes[index] - b'0';
        parsed = parsed * 10 + digit;
        index += 1;
    }
    parsed
}

const fn parse_hex_u64(value: &str) -> u64 {
    let bytes = value.as_bytes();
    let mut parsed = 0_u64;
    let mut index = 0;
    while index < bytes.len() {
        let digit = match bytes[index] {
            b'0'..=b'9' => bytes[index] - b'0',
            b'a'..=b'f' => bytes[index] - b'a' + 10,
            b'A'..=b'F' => bytes[index] - b'A' + 10,
            _ => 0,
        };
        parsed = (parsed << 4) | digit as u64;
        index += 1;
    }
    parsed
}
