//! Borrowed fail-safe wrapper for persistent RF-switch GPIO outputs.

use embedded_hal::digital::{ErrorType, OutputPin};

/// Delegate switch control during a radio session, then always restore low on
/// drop without taking ownership of the persistent board-level GPIO output.
pub struct FailSafeRfOutput<'a, CTRL: OutputPin>(&'a mut CTRL);

impl<'a, CTRL: OutputPin> FailSafeRfOutput<'a, CTRL> {
    pub fn new(output: &'a mut CTRL) -> Self {
        Self(output)
    }
}

impl<CTRL: OutputPin> ErrorType for FailSafeRfOutput<'_, CTRL> {
    type Error = CTRL::Error;
}

impl<CTRL: OutputPin> OutputPin for FailSafeRfOutput<'_, CTRL> {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.0.set_low()
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.0.set_high()
    }
}

impl<CTRL: OutputPin> Drop for FailSafeRfOutput<'_, CTRL> {
    fn drop(&mut self) {
        let _ = self.0.set_low();
    }
}

#[cfg(test)]
mod tests {
    use core::convert::Infallible;

    use super::*;

    #[derive(Default)]
    struct MockOutput {
        high: bool,
        low_writes: u8,
    }

    impl ErrorType for MockOutput {
        type Error = Infallible;
    }

    impl OutputPin for MockOutput {
        fn set_low(&mut self) -> Result<(), Self::Error> {
            self.high = false;
            self.low_writes = self.low_writes.saturating_add(1);
            Ok(())
        }

        fn set_high(&mut self) -> Result<(), Self::Error> {
            self.high = true;
            Ok(())
        }
    }

    #[test]
    fn wrapper_drop_restores_low_without_consuming_persistent_output() {
        let mut output = MockOutput::default();
        {
            let mut session = FailSafeRfOutput::new(&mut output);
            session.set_high().unwrap();
        }
        assert!(!output.high);
        assert_eq!(output.low_writes, 1);

        // The board-level output remains owned and usable after the session.
        output.set_high().unwrap();
        assert!(output.high);
    }
}
