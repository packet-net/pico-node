//! The link timers (T1/T2/T3) as an abstract integer-millisecond service.
//!
//! Ports the `ITimerScheduler` contract the C# dispatcher arms/cancels/queries.
//! The runtime never reads a clock directly — it asks the [`TimerService`] to arm
//! a named timer for a duration in **milliseconds** (integerised per research §3;
//! the no-FPU M0+ has no business doing `f64` timer math), to cancel it, and to
//! report whether it is running + its remaining time (for the `Select_T1_Value`
//! round-trip sample). Expiries surface back to the session as an [`Event`] — the
//! firmware's timer task waits on the nearest armed timer and posts the matching
//! `T1Expiry` / `T2Expiry` / `T3Expiry`.
//!
//! [`super::session::Session`] also snapshots/restores the running-timer set
//! around a transition so a part-applied transition can't leave the link watchdog
//! cancelled (the packet.net#225 timer-rollback). [`TimerService`] therefore
//! exposes [`capture`](TimerService::capture) / [`restore`](TimerService::restore).

/// The three data-link timers. (TM201 — the MDL retry timer — is out of scope for
/// the data-link runtime; the management machine owns it.)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TimerId {
    /// T1 — acknowledgement timer (§6.7.1.3).
    T1,
    /// T2 — response-delay timer (§6.7.1.4).
    T2,
    /// T3 — inactive-link timer (§6.7.1.5).
    T3,
}

/// An opaque snapshot of which timers are running + their remaining ms, used to
/// roll back a thrown transition's timer side-effects. Three slots — one per
/// [`TimerId`] — each `Some(remaining_ms)` if running.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct TimerSnapshot {
    /// T1 remaining ms, or `None` if not running.
    pub t1: Option<u32>,
    /// T2 remaining ms, or `None` if not running.
    pub t2: Option<u32>,
    /// T3 remaining ms, or `None` if not running.
    pub t3: Option<u32>,
}

/// The abstract timer service the runtime drives. The firmware implements this
/// over `embassy-time`; [`MockTimerService`] implements it for host tests.
pub trait TimerService {
    /// Arm (or re-arm) `id` to expire after `duration_ms`.
    fn arm(&mut self, id: TimerId, duration_ms: u32);
    /// Cancel `id` if running (no-op if not).
    fn cancel(&mut self, id: TimerId);
    /// True if `id` is currently armed.
    fn is_running(&self, id: TimerId) -> bool;
    /// Remaining ms on `id`, or `0` if it is not running / already expired.
    fn time_remaining_ms(&self, id: TimerId) -> u32;
    /// Snapshot the running-timer set (for transition rollback).
    fn capture(&self) -> TimerSnapshot;
    /// Restore a previously [`capture`](Self::capture)d running-timer set.
    fn restore(&mut self, snap: TimerSnapshot);
}

/// SRT IIR update (`Select_T1_Value`), integerised. The spec draws
/// `SRT := 7·SRT/8 + (T1V − remaining_when_stopped)/8`. The new-sample term is the
/// elapsed portion of T1 from arm to stop = a round-trip estimate. Karn's
/// algorithm (the `karn_srt_sampling` quirk) skips the update unless the sample is
/// a clean measurement (T1 was stopped by an ack ⇒ remaining > 0), because on a
/// timeout the sample degenerates to the full T1V (=2·SRT) and the IIR
/// self-amplifies to overflow. Returns the new SRT in ms.
///
/// `t1v_ms` is the value T1 was armed for; `remaining_ms` is the time-remaining
/// captured by `stop_T1`. All integer math: `7*srt/8 + sample/8`.
pub fn srt_iir_update(srt_ms: u32, t1v_ms: u32, remaining_ms: u32, karn: bool) -> u32 {
    let sample = t1v_ms.saturating_sub(remaining_ms);
    let clean_measurement = remaining_ms > 0;
    if karn && !clean_measurement {
        return srt_ms; // skip the update — no clean round-trip sample
    }
    // 7/8 · srt + 1/8 · sample, integer arithmetic (matches the figure formula).
    (srt_ms.saturating_mul(7) / 8).saturating_add(sample / 8)
}

/// `Next T1 := (RC·0.25)+SRT·2`, integerised to `RC*250 + SRT*2` ms (research §3).
pub fn next_t1_rc_backoff_ms(rc: u32, srt_ms: u32) -> u32 {
    rc.saturating_mul(250)
        .saturating_add(srt_ms.saturating_mul(2))
}

/// A host-test timer service: tracks armed timers + their remaining ms in plain
/// fields, never advances time on its own (tests post expiry events explicitly).
/// `no_std`-clean — no clock, no alloc.
#[derive(Debug, Clone, Copy, Default)]
pub struct MockTimerService {
    t1: Option<u32>,
    t2: Option<u32>,
    t3: Option<u32>,
}

impl MockTimerService {
    /// A fresh service with no timers armed.
    pub fn new() -> Self {
        Self::default()
    }

    fn slot(&self, id: TimerId) -> Option<u32> {
        match id {
            TimerId::T1 => self.t1,
            TimerId::T2 => self.t2,
            TimerId::T3 => self.t3,
        }
    }

    fn slot_mut(&mut self, id: TimerId) -> &mut Option<u32> {
        match id {
            TimerId::T1 => &mut self.t1,
            TimerId::T2 => &mut self.t2,
            TimerId::T3 => &mut self.t3,
        }
    }

    /// Test helper: shrink a running timer's remaining time toward expiry (to
    /// simulate elapsed time before a `stop_T1` round-trip sample).
    pub fn set_remaining(&mut self, id: TimerId, remaining_ms: u32) {
        if let Some(slot) = self.slot_mut(id).as_mut() {
            *slot = remaining_ms;
        }
    }
}

impl TimerService for MockTimerService {
    fn arm(&mut self, id: TimerId, duration_ms: u32) {
        *self.slot_mut(id) = Some(duration_ms);
    }
    fn cancel(&mut self, id: TimerId) {
        *self.slot_mut(id) = None;
    }
    fn is_running(&self, id: TimerId) -> bool {
        self.slot(id).is_some()
    }
    fn time_remaining_ms(&self, id: TimerId) -> u32 {
        self.slot(id).unwrap_or(0)
    }
    fn capture(&self) -> TimerSnapshot {
        TimerSnapshot {
            t1: self.t1,
            t2: self.t2,
            t3: self.t3,
        }
    }
    fn restore(&mut self, snap: TimerSnapshot) {
        self.t1 = snap.t1;
        self.t2 = snap.t2;
        self.t3 = snap.t3;
    }
}
