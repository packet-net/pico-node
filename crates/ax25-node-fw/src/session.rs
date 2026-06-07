//! Link-layer session layer — the on-target home of the SDL runtime.
//!
//! The portable runtime now lives in [`ax25_node_core::sdl`] (the Rust port of
//! packet.net's `Ax25Session` + dispatcher + guards + subroutines, driven off the
//! generated `ax25sdl` typed tables). This module is the thin firmware wrapper: it
//! owns one [`ax25_node_core::sdl::SessionManager`] (a fixed `MAX_SESSIONS` array of
//! per-peer sessions — NOT the desktop's unbounded LRU dict, per research §6) and
//! the T1/T2/T3 timer service backed by `embassy-time`.
//!
//! Flow: a transport decodes an inbound wire frame, calls
//! [`ax25_node_core::sdl::classify_incoming`] to get the [`Event`], and posts it to
//! the manager keyed by the peer callsign; the manager routes it to that peer's
//! session, runs the state machine, and returns the wire frames to send back. DL
//! signals raised upward (connect/data/disconnect indications) are drained for the
//! telnet console / app.
//!
//! Concurrency note: this wrapper is intended to be owned by a single supervising
//! task (or guarded by an `embassy_sync` mutex) so the `&mut SessionManager` borrow
//! the manager's `post` needs is serialised across transports. The exact sharing
//! (one owning task with an event channel vs. a shared mutex) is finalised at
//! WiFi bring-up; the manager + timer logic below is hardware-independent and is
//! host-tested in `ax25_node_core::sdl::manager`.

// GATE 3 (HW-BRINGUP.md §4): only the NET/ROM tap (new_netrom/observe_inbound)
// is consumed so far; the SessionManager + timer machinery below is the seam the
// session supervisor wires up when connected mode lands. Remove then.
#![allow(dead_code)]

use ax25_node_core::ax25::{Callsign, Frame};
use ax25_node_core::netrom::{NetRomService, ObserveOutcome, PortId};
use ax25_node_core::sdl::{Event, SessionManager, TimerId, TimerService, TimerSnapshot};

use embassy_time::{Duration, Instant, Timer};

/// Maximum concurrent link-layer sessions. Fixed (no heap session map). Sized for
/// a Pico node; bump with care given the per-session window buffers (research §6).
pub const MAX_SESSIONS: usize = 4;

/// The node's per-peer session collection. Construct once at boot with the node's
/// own callsign; share it (single task or `embassy_sync` mutex) across transports.
pub type Sessions = SessionManager<MAX_SESSIONS>;

/// Build the session manager for this node's local callsign.
pub fn new_sessions(local: Callsign) -> Sessions {
    SessionManager::new(local)
}

/// The node's read-only NET/ROM observer (the Rust port of the C# `NetRomService`).
/// Construct once at boot; share it (single task or `embassy_sync` mutex) across
/// transports exactly like [`Sessions`]. It is *only* fed the read-only tap below —
/// it owns no socket and emits nothing on the air.
pub type NetRom = NetRomService;

/// Build the NET/ROM service (enabled, canonical defaults). Disable per config by
/// constructing with [`NetRomService::with_options`].
pub fn new_netrom() -> NetRom {
    NetRomService::new()
}

/// **The read-only NET/ROM tap — the inbound-frame hook.**
///
/// Call this for **every decoded inbound frame, BEFORE address filtering / before
/// routing it to a session** — exactly where the C# `Ax25Listener.InboundPumpAsync`
/// raises `FrameTraced` (which fires before `DispatchInbound` drops not-addressed-to-
/// us frames). NODES broadcasts are addressed to the literal callsign `NODES`, not to
/// the node, so they would never reach a session; the tap is how the node *hears*
/// them. This is observation-only: it never alters the frame, emits nothing, and
/// cannot touch a session — so a NODES storm mid-QSO leaves the link untouched
/// (proven in `ax25_node_core::netrom`'s read-only-guarantee test).
///
/// After this returns, the caller proceeds with its normal address filter: if the
/// frame is addressed to us, classify it and post it to the [`Sessions`] manager;
/// otherwise drop it (the tap has already extracted any NET/ROM value).
pub fn observe_inbound(
    netrom: &mut NetRom,
    frame: &Frame,
    my_call: Callsign,
    port_id: PortId,
) -> ObserveOutcome {
    // Monotonic millisecond tick for the neighbour's last-heard stamp (the core has
    // no wall-clock — research §2.7 / §3: time is injected).
    let now = Instant::now().as_millis();
    netrom.observe_frame(frame, my_call, port_id, now)
}

/// An `embassy-time`-backed [`TimerService`]: each of T1/T2/T3 is a deadline
/// ([`Instant`]) or `None`. The [`timer_task`] waits on the nearest deadline and
/// posts the matching expiry [`Event`] into the session manager.
///
/// Integer-millisecond throughout (research §3): the runtime arms timers in `u32`
/// ms, which map onto `embassy_time::Duration::from_millis`.
#[derive(Clone, Copy, Default)]
pub struct EmbassyTimers {
    t1: Option<Instant>,
    t2: Option<Instant>,
    t3: Option<Instant>,
}

impl EmbassyTimers {
    /// A service with no timers armed.
    pub const fn new() -> Self {
        Self {
            t1: None,
            t2: None,
            t3: None,
        }
    }

    fn slot(&self, id: TimerId) -> Option<Instant> {
        match id {
            TimerId::T1 => self.t1,
            TimerId::T2 => self.t2,
            TimerId::T3 => self.t3,
        }
    }

    fn slot_mut(&mut self, id: TimerId) -> &mut Option<Instant> {
        match id {
            TimerId::T1 => &mut self.t1,
            TimerId::T2 => &mut self.t2,
            TimerId::T3 => &mut self.t3,
        }
    }

    /// The nearest armed deadline across all timers, if any — what [`timer_task`]
    /// sleeps until.
    pub fn next_deadline(&self) -> Option<Instant> {
        [self.t1, self.t2, self.t3].into_iter().flatten().min()
    }

    /// Return the timer ids whose deadline is at/<= `now`, clearing them. The
    /// caller posts an expiry event for each.
    pub fn take_expired(&mut self, now: Instant) -> heapless::Vec<TimerId, 3> {
        let mut out = heapless::Vec::new();
        for id in [TimerId::T1, TimerId::T2, TimerId::T3] {
            if matches!(self.slot(id), Some(deadline) if deadline <= now) {
                *self.slot_mut(id) = None;
                let _ = out.push(id);
            }
        }
        out
    }
}

impl TimerService for EmbassyTimers {
    fn arm(&mut self, id: TimerId, duration_ms: u32) {
        *self.slot_mut(id) = Some(Instant::now() + Duration::from_millis(duration_ms as u64));
    }
    fn cancel(&mut self, id: TimerId) {
        *self.slot_mut(id) = None;
    }
    fn is_running(&self, id: TimerId) -> bool {
        self.slot(id).is_some()
    }
    fn time_remaining_ms(&self, id: TimerId) -> u32 {
        match self.slot(id) {
            Some(deadline) => {
                let now = Instant::now();
                if deadline > now {
                    (deadline - now).as_millis() as u32
                } else {
                    0
                }
            }
            None => 0,
        }
    }
    fn capture(&self) -> TimerSnapshot {
        // Snapshot as remaining-ms (the runtime's rollback unit).
        TimerSnapshot {
            t1: self.t1.map(|_| self.time_remaining_ms(TimerId::T1)),
            t2: self.t2.map(|_| self.time_remaining_ms(TimerId::T2)),
            t3: self.t3.map(|_| self.time_remaining_ms(TimerId::T3)),
        }
    }
    fn restore(&mut self, snap: TimerSnapshot) {
        let now = Instant::now();
        let to_deadline = |ms: Option<u32>| ms.map(|m| now + Duration::from_millis(m as u64));
        self.t1 = to_deadline(snap.t1);
        self.t2 = to_deadline(snap.t2);
        self.t3 = to_deadline(snap.t3);
    }
}

/// Map a timer id to the runtime expiry event the session manager expects.
pub fn expiry_event(id: TimerId) -> Event {
    match id {
        TimerId::T1 => Event::T1Expiry,
        TimerId::T2 => Event::T2Expiry,
        TimerId::T3 => Event::T3Expiry,
    }
}

/// The T1/T2/T3 timer service task. Waits on the nearest armed deadline, then feeds
/// the expiry into the session walk. STUB at the wiring seam: the shared-state
/// access (which session, which sink) is finalised with the supervisor's chosen
/// sharing model at WiFi bring-up. The timer math + service above are complete and
/// host-tested via the core `MockTimerService` parity (`ax25_node_core::sdl::timer`).
#[embassy_executor::task]
pub async fn timer_task() {
    loop {
        // The real loop borrows the shared EmbassyTimers, computes next_deadline(),
        // waits until it (or until re-armed), then for each take_expired() id posts
        // expiry_event(id) into the SessionManager and flushes the resulting frames
        // to the owning transport. Placeholder cadence until that shared state lands.
        Timer::after_millis(100).await;
    }
}
