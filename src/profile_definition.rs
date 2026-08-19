//! Target-independent definition of Bosch's neutral HP-354 exploratory profile.
//!
//! Documentary source: Bosch Sensortec, BME AI-Studio Manual,
//! BST-BME688-AN001-00 v1.6.0, HP-354 profile JSON (page 70):
//! <https://www.bosch-sensortec.com/media/boschsensortec/downloads/application_notes_1/bst-bme688-an001.pdf>

pub const PROFILE_ID: u16 = 0x0354;
pub const PROFILE_VERSION: u16 = 1;
pub const PROFILE_STEP_COUNT: u8 = 10;
pub const PROFILE_TEMPERATURES_CELSIUS: [u16; 10] =
    [320, 100, 100, 100, 200, 200, 200, 320, 320, 320];
pub const PROFILE_REPETITION_MULTIPLIERS: [u8; 10] = [5, 2, 10, 30, 5, 5, 5, 5, 5, 5];

/// Bosch's example requests 140 ms minus integer TPHG milliseconds. With this
/// configuration TPHG is 41,590 us, producing a requested shared wait of 99 ms.
pub const REQUESTED_SHARED_DURATION_MS: u16 = 99;
/// `99 ms` encodes to raw shared-wait register `0x73`, which decodes to
/// 97,308 us. Every nonzero multiplier repeats shared-wait + TPHG.
pub const PROGRAMMED_SHARED_DURATION_RAW: u8 = 0x73;
#[cfg(test)]
pub const PROGRAMMED_SHARED_DURATION_US: u32 = 97_308;
pub const TPHG_DURATION_US: u32 = 41_590;
#[cfg(test)]
pub const BASE_STEP_DURATION_US: u32 = PROGRAMMED_SHARED_DURATION_US + TPHG_DURATION_US;
#[cfg(test)]
pub const EXPECTED_STEP_DURATION_US: [u32; 10] = [
    694_490, 277_796, 1_388_980, 4_166_940, 694_490, 694_490, 694_490, 694_490, 694_490, 694_490,
];
pub const EXPECTED_PROFILE_DURATION_US: u32 = 10_695_146;

/// Which independent bound ended collection after the driver reported its
/// generic timeout finish state.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TimeoutBound {
    /// The monotonic 15-second deadline elapsed before the poll counter filled.
    Deadline,
    /// The configured number of 100-ms poll attempts was exhausted.
    PollBudget,
}

/// Classify the generic collector timeout without losing which firmware bound
/// actually fired. If both are reached together, the explicit poll budget is
/// reported because its threshold is directly observable in telemetry.
#[must_use]
pub const fn timeout_bound(poll_count: u16, maximum_poll_count: u16) -> TimeoutBound {
    if poll_count >= maximum_poll_count {
        TimeoutBound::PollBudget
    } else {
        TimeoutBound::Deadline
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hp354_has_ten_exact_ordered_steps() {
        assert_eq!(PROFILE_TEMPERATURES_CELSIUS.len(), 10);
        assert_eq!(PROFILE_REPETITION_MULTIPLIERS.len(), 10);
        assert_eq!(PROFILE_REPETITION_MULTIPLIERS.iter().sum::<u8>(), 77);
    }

    #[test]
    fn hp354_timing_uses_quantized_register_not_requested_milliseconds() {
        let mut total = 0_u32;
        for (index, multiplier) in PROFILE_REPETITION_MULTIPLIERS.iter().enumerate() {
            let duration = u32::from(*multiplier) * BASE_STEP_DURATION_US;
            assert_eq!(duration, EXPECTED_STEP_DURATION_US[index]);
            total += duration;
        }
        assert_eq!(total, EXPECTED_PROFILE_DURATION_US);
        assert_ne!(PROGRAMMED_SHARED_DURATION_US, 99_000);
    }

    #[test]
    fn timeout_reason_distinguishes_deadline_from_poll_budget() {
        assert_eq!(timeout_bound(149, 150), TimeoutBound::Deadline);
        assert_eq!(timeout_bound(150, 150), TimeoutBound::PollBudget);
        assert_eq!(timeout_bound(151, 150), TimeoutBound::PollBudget);
    }
}
