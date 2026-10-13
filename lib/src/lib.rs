//! spectractrl core logic: reusable async behavior, independent of
//! firmware wiring. Note: this crate is `no_std`, so it cannot use the
//! standard `#[test]` harness directly. Testing no_std logic generally
//! means either `defmt-test` on real hardware, or extracting pure,
//! `core`-only logic into a separate `std`-testable crate.
#![no_std]

use embassy_time::Duration;

/// Returns the on/off interval for a blink cycle.
pub fn blink_interval() -> Duration {
    Duration::from_millis(500)
}
