//! Portable capped-exponential-backoff reconnect state machine.
//!
//! Ports the reconnect *policy* of C# `Packet.Node.Core.Transports.ReconnectingKissModem`
//! — the part that decides *when* to re-dial after an established KISS link drops:
//! attempt immediately, then wait a backoff that doubles on each failure, capped at a
//! ceiling, until a fresh link is up.
//!
//! The C# type is a full transport decorator (it owns the socket, pumps the inner
//! stream, replays KISS params, and faults in-flight sends). All of that I/O lives in
//! the firmware crate, which owns the actual TCP/UART socket. What is portable — and
//! host-testable with a fake clock — is the **decision core**: fault → capped backoff
//! → reopen. This module is exactly that, and nothing else.
//!
//! `no_std`, allocation-free, FPU-free: time is an integer millisecond count the
//! caller supplies (a monotonic clock on-target, a fake counter in tests); backoff
//! durations are integer milliseconds doubled with a saturating clamp.
//!
//! ## How the firmware drives it
//!
//! ```ignore
//! let mut link = ReconnectingLink::default();
//! // …pump the socket; on end-of-stream:
//! link.on_fault();
//! loop {
//!     match link.poll(clock.now_ms()) {
//!         ReconnectAction::Idle => break,                 // link is up
//!         ReconnectAction::Wait { until_ms } => sleep_until(until_ms).await,
//!         ReconnectAction::Attempt => match reconnect().await {
//!             Ok(sock) => { link.on_connected(); /* resume pumping `sock` */ }
//!             Err(_)   => link.on_attempt_failed(clock.now_ms()),
//!         },
//!     }
//! }
//! ```

/// The default first / minimum backoff, matching the C# `ReconnectingKissModem`
/// default (`TimeSpan.FromSeconds(1)`).
pub const DEFAULT_MIN_BACKOFF_MS: u32 = 1_000;

/// The default backoff ceiling, matching the C# default (`TimeSpan.FromSeconds(30)`).
pub const DEFAULT_MAX_BACKOFF_MS: u32 = 30_000;

/// The min/max backoff bounds. Mirrors the C# `minBackoff` / `maxBackoff` ctor args.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ReconnectPolicy {
    min_backoff_ms: u32,
    max_backoff_ms: u32,
}

impl Default for ReconnectPolicy {
    fn default() -> Self {
        Self::new(DEFAULT_MIN_BACKOFF_MS, DEFAULT_MAX_BACKOFF_MS)
    }
}

impl ReconnectPolicy {
    /// A policy with the given min/max backoff in ms. `max` is raised to `min` if a
    /// caller passes `max < min` (the C# `Math.Clamp` would otherwise be ill-formed);
    /// a `min` of 0 means "attempt back-to-back with no delay", matching the C# tests
    /// that pass `TimeSpan.Zero`.
    pub const fn new(min_backoff_ms: u32, max_backoff_ms: u32) -> Self {
        let max_backoff_ms = if max_backoff_ms < min_backoff_ms {
            min_backoff_ms
        } else {
            max_backoff_ms
        };
        Self {
            min_backoff_ms,
            max_backoff_ms,
        }
    }

    /// The minimum / first-attempt backoff (ms).
    pub const fn min_backoff_ms(self) -> u32 {
        self.min_backoff_ms
    }

    /// The backoff ceiling (ms).
    pub const fn max_backoff_ms(self) -> u32 {
        self.max_backoff_ms
    }

    /// `clamp(current * 2, min, max)` — the C# doubling step
    /// (`Math.Clamp(backoff.Ticks * 2, min, max)`), in saturating integer ms.
    fn doubled(self, current_ms: u32) -> u32 {
        let doubled = (current_ms as u64) * 2;
        let clamped = doubled.clamp(self.min_backoff_ms as u64, self.max_backoff_ms as u64);
        clamped as u32
    }
}

/// What the caller should do right now, from [`ReconnectingLink::poll`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReconnectAction {
    /// The link is up — nothing to do.
    Idle,
    /// Attempt a reconnect now.
    Attempt,
    /// Still backing off. Wait until this absolute millisecond deadline (per the
    /// caller's clock) before the next attempt.
    Wait {
        /// Absolute-ms deadline at which the next [`ReconnectAction::Attempt`] is due.
        until_ms: u64,
    },
}

/// Internal link phase.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    /// The link is established.
    Up,
    /// A reconnect attempt is due now. `backoff_ms` is the delay to apply if it fails.
    Attempting { backoff_ms: u32 },
    /// Waiting out a backoff until `resume_at_ms`. `backoff_ms` is the (already
    /// doubled) delay to apply after the *next* failure.
    BackingOff { resume_at_ms: u64, backoff_ms: u32 },
}

/// The reconnect decision state machine: fault → capped backoff → reopen.
///
/// Ports the reconnect policy of C# `ReconnectingKissModem` (the `ReconnectAsync`
/// loop + the `IsReconnecting` flag). Drive it with [`on_fault`](Self::on_fault),
/// [`on_attempt_failed`](Self::on_attempt_failed), and
/// [`on_connected`](Self::on_connected), and ask [`poll`](Self::poll) what to do.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ReconnectingLink {
    policy: ReconnectPolicy,
    phase: Phase,
}

impl Default for ReconnectingLink {
    fn default() -> Self {
        Self::new(ReconnectPolicy::default())
    }
}

impl ReconnectingLink {
    /// A new link in the established (`Up`) state under `policy`.
    pub const fn new(policy: ReconnectPolicy) -> Self {
        Self {
            policy,
            phase: Phase::Up,
        }
    }

    /// The configured backoff bounds.
    pub const fn policy(&self) -> ReconnectPolicy {
        self.policy
    }

    /// True from the moment a drop is detected until a fresh link is live —
    /// the exact window of the C# `IsReconnecting` metric (#583).
    pub const fn is_reconnecting(&self) -> bool {
        !matches!(self.phase, Phase::Up)
    }

    /// The established link dropped. Begins a reconnect cycle: the first attempt is
    /// due immediately (no initial delay), exactly like the C# `ReconnectAsync`
    /// (`backoff = minBackoff`, then an eager first `reconnect()`). Ignored if a
    /// reconnect cycle is already in progress — faults only originate from a live
    /// pump, so a redundant fault must not reset the backoff.
    pub fn on_fault(&mut self) {
        if let Phase::Up = self.phase {
            self.phase = Phase::Attempting {
                backoff_ms: self.policy.min_backoff_ms,
            };
        }
    }

    /// The in-progress reconnect attempt failed at `now_ms`. Schedules a wait of the
    /// current backoff, then doubles it (capped) for next time — mirroring the C#
    /// `delay(backoff); backoff = clamp(backoff*2, min, max)`. A no-op unless an
    /// attempt was actually due.
    pub fn on_attempt_failed(&mut self, now_ms: u64) {
        if let Phase::Attempting { backoff_ms } = self.phase {
            self.phase = Phase::BackingOff {
                resume_at_ms: now_ms.saturating_add(backoff_ms as u64),
                backoff_ms: self.policy.doubled(backoff_ms),
            };
        }
    }

    /// A reconnect attempt succeeded — a fresh link is live. Returns to `Up` and
    /// resets the backoff, so a later drop starts again at the minimum. Mirrors the
    /// C# `inner = next; reconnecting = false`.
    pub fn on_connected(&mut self) {
        self.phase = Phase::Up;
    }

    /// The current backoff in ms: the delay that will be applied if the pending /
    /// next attempt fails. `0`-equivalent (the minimum) while `Up`. Useful for the
    /// "retrying in Ns" log line (C# event 5103).
    pub const fn current_backoff_ms(&self) -> u32 {
        match self.phase {
            Phase::Up => self.policy.min_backoff_ms,
            Phase::Attempting { backoff_ms } | Phase::BackingOff { backoff_ms, .. } => backoff_ms,
        }
    }

    /// Decide what to do at `now_ms`. Advances a matured backoff to an attempt, so a
    /// caller can poll on its own cadence. Pure decision — performs no I/O.
    pub fn poll(&mut self, now_ms: u64) -> ReconnectAction {
        match self.phase {
            Phase::Up => ReconnectAction::Idle,
            Phase::Attempting { .. } => ReconnectAction::Attempt,
            Phase::BackingOff {
                resume_at_ms,
                backoff_ms,
            } => {
                if now_ms >= resume_at_ms {
                    self.phase = Phase::Attempting { backoff_ms };
                    ReconnectAction::Attempt
                } else {
                    ReconnectAction::Wait {
                        until_ms: resume_at_ms,
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_match_the_csharp_one_and_thirty_second_bounds() {
        let p = ReconnectPolicy::default();
        assert_eq!(p.min_backoff_ms(), 1_000);
        assert_eq!(p.max_backoff_ms(), 30_000);
    }

    #[test]
    fn starts_up_and_idle() {
        let mut link = ReconnectingLink::default();
        assert!(!link.is_reconnecting());
        assert_eq!(link.poll(0), ReconnectAction::Idle);
    }

    #[test]
    fn fault_makes_the_first_attempt_due_immediately() {
        let mut link = ReconnectingLink::default();
        link.on_fault();
        assert!(link.is_reconnecting());
        // No initial wait — the first attempt is eager.
        assert_eq!(link.poll(0), ReconnectAction::Attempt);
        assert_eq!(link.poll(999_999), ReconnectAction::Attempt);
    }

    #[test]
    fn backoff_doubles_and_caps_at_the_ceiling() {
        // min 1 s, max 30 s → 1, 2, 4, 8, 16, 30, 30, …
        let expected = [1_000u64, 2_000, 4_000, 8_000, 16_000, 30_000, 30_000, 30_000];
        let mut link = ReconnectingLink::new(ReconnectPolicy::new(1_000, 30_000));
        let mut now = 0u64;
        link.on_fault();
        for &want in &expected {
            assert_eq!(link.poll(now), ReconnectAction::Attempt);
            link.on_attempt_failed(now);
            match link.poll(now) {
                ReconnectAction::Wait { until_ms } => {
                    assert_eq!(until_ms - now, want, "backoff delay at now={now}");
                    now = until_ms; // advance the fake clock to the resume instant
                }
                other => panic!("expected Wait, got {other:?}"),
            }
        }
    }

    #[test]
    fn waits_until_the_deadline_then_attempts() {
        let mut link = ReconnectingLink::new(ReconnectPolicy::new(1_000, 30_000));
        link.on_fault();
        assert_eq!(link.poll(0), ReconnectAction::Attempt);
        link.on_attempt_failed(0);
        // Before the 1 s deadline: wait.
        assert_eq!(link.poll(500), ReconnectAction::Wait { until_ms: 1_000 });
        assert_eq!(link.poll(999), ReconnectAction::Wait { until_ms: 1_000 });
        // At/after the deadline: attempt.
        assert_eq!(link.poll(1_000), ReconnectAction::Attempt);
    }

    #[test]
    fn is_reconnecting_is_true_exactly_while_the_link_is_down() {
        let mut link = ReconnectingLink::default();
        assert!(!link.is_reconnecting(), "initial link is up");
        link.on_fault();
        assert!(link.is_reconnecting(), "down after a drop");
        link.on_attempt_failed(0);
        assert!(link.is_reconnecting(), "still down while backing off");
        link.on_connected();
        assert!(!link.is_reconnecting(), "up again after a fresh connect");
        assert_eq!(link.poll(2_000), ReconnectAction::Idle);
    }

    #[test]
    fn on_connected_resets_the_backoff_to_the_minimum() {
        let mut link = ReconnectingLink::new(ReconnectPolicy::new(1_000, 30_000));
        link.on_fault();
        // Fail a few times to climb the backoff well past the minimum.
        let mut now = 0u64;
        for _ in 0..4 {
            assert_eq!(link.poll(now), ReconnectAction::Attempt);
            link.on_attempt_failed(now);
            if let ReconnectAction::Wait { until_ms } = link.poll(now) {
                now = until_ms;
            }
        }
        assert!(link.current_backoff_ms() > 1_000, "backoff has climbed");

        // Reconnect, then drop again: the next cycle restarts at the minimum.
        link.on_connected();
        link.on_fault();
        assert_eq!(link.poll(now), ReconnectAction::Attempt);
        link.on_attempt_failed(now);
        if let ReconnectAction::Wait { until_ms } = link.poll(now) {
            assert_eq!(until_ms - now, 1_000, "fresh cycle starts at min backoff");
        } else {
            panic!("expected a Wait");
        }
    }

    #[test]
    fn a_redundant_fault_does_not_reset_the_backoff() {
        let mut link = ReconnectingLink::new(ReconnectPolicy::new(1_000, 30_000));
        link.on_fault();
        link.on_attempt_failed(0); // backoff now doubled to 2 s
        let before = link.current_backoff_ms();
        link.on_fault(); // spurious — already reconnecting
        assert_eq!(link.current_backoff_ms(), before, "backoff unchanged by a redundant fault");
    }

    #[test]
    fn zero_backoff_policy_attempts_back_to_back() {
        // The C# tests drive minBackoff: Zero, maxBackoff: Zero — reconnect churns
        // with no delay. Here that means every Wait deadline equals `now`.
        let mut link = ReconnectingLink::new(ReconnectPolicy::new(0, 0));
        link.on_fault();
        assert_eq!(link.poll(0), ReconnectAction::Attempt);
        link.on_attempt_failed(0);
        // resume_at == now → poll immediately re-attempts.
        assert_eq!(link.poll(0), ReconnectAction::Attempt);
    }

    #[test]
    fn attempt_failed_is_ignored_when_no_attempt_is_due() {
        let mut link = ReconnectingLink::default();
        // Up: no attempt in flight → no-op.
        link.on_attempt_failed(100);
        assert!(!link.is_reconnecting());
    }

    #[test]
    fn max_below_min_is_clamped_up_to_min() {
        let p = ReconnectPolicy::new(5_000, 1_000);
        assert_eq!(p.max_backoff_ms(), 5_000);
        // Doubling can never fall below min nor exceed the (raised) max.
        let mut link = ReconnectingLink::new(p);
        link.on_fault();
        link.on_attempt_failed(0);
        assert_eq!(link.current_backoff_ms(), 5_000);
    }
}
