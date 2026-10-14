//! The clock every `mono_ns` field and every deadline of the contract is
//! read from.
//!
//! `CLOCK_MONOTONIC` is one clock for every process of a host, so the daemon
//! and a provider can compare each other's stamps: the age of a heartbeat,
//! whether a command's start deadline has passed. It is never stepped by NTP
//! or by an operator, which the wall clock is.

/// Nanoseconds on this host's `CLOCK_MONOTONIC`.
#[must_use]
pub fn monotonic_ns() -> u64 {
    let now = rustix::time::clock_gettime(rustix::time::ClockId::Monotonic);
    let secs = u64::try_from(now.tv_sec).unwrap_or(0);
    let nanos = u64::try_from(now.tv_nsec).unwrap_or(0);
    secs.saturating_mul(1_000_000_000).saturating_add(nanos)
}

/// Microseconds since the Unix epoch on this host's wall clock: the clock a
/// `joints` command's `timestamp_us` and a tick's `timestamp_us` are in.
#[must_use]
pub fn wall_clock_us() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |elapsed| {
            u64::try_from(elapsed.as_micros()).unwrap_or(u64::MAX)
        })
}

#[cfg(test)]
mod tests {
    use super::monotonic_ns;

    #[test]
    fn the_monotonic_clock_does_not_go_back() {
        let first = monotonic_ns();
        let second = monotonic_ns();
        assert!(first > 0);
        assert!(second >= first);
    }
}
