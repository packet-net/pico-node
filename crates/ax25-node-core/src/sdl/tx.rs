//! The transition context (`Tx`) threaded through guard eval, action dispatch,
//! and subroutine walking — ports `Packet.Ax25.Session.TransitionContext`.
//!
//! One `Tx` is built per dispatched event and borrows the session's mutable
//! state, timer service, and outbound sink for the duration of the transition.
//! It also carries the [`PendingFrame`] scratch — the fields a chain of
//! processing verbs accumulate before a `signal_lower` verb consumes them to
//! build a frame (e.g. `N(r) := V(r); F := 1; RR`), and the
//! [`retrieved_stored_frame`](Tx::retrieved_stored_frame) staging slot the
//! figc4.4/figc4.5 stored-frame drain uses.

extern crate alloc;
use alloc::vec::Vec;

use super::context::SessionContext;
use super::event::Event;
use super::signal::SessionSink;
use super::timer::TimerService;

/// Scratch fields a processing-verb chain populates for the next outgoing frame.
/// `None` means "not explicitly set" — the frame builder applies the spec's
/// implicit default (N(R) ⇐ V(R), P/F ⇐ 0). Ports `PendingFrame`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct PendingFrame {
    /// N(R) to carry on the next frame (`N(r) := …`).
    pub nr: Option<u8>,
    /// N(S) to carry on the next I-frame (`N(s) := …`).
    pub ns: Option<u8>,
    /// The poll/final bit for the next frame (`P := …` / `F := …`).
    pub pf: Option<bool>,
}

/// The per-transition context. Borrows session state + the timer service + the
/// outbound sink; owns the pending-frame scratch + the stored-frame staging slot.
pub struct Tx<'a> {
    /// The mutable session state.
    pub session: &'a mut SessionContext,
    /// The timer service to arm/cancel/query.
    pub timers: &'a mut dyn TimerService,
    /// The outbound signal sink.
    pub sink: &'a mut dyn SessionSink,
    /// The event that triggered this transition.
    pub trigger: Event,
    /// Accumulated fields for the next outgoing frame in this chain.
    pub pending: PendingFrame,
    /// A stored out-of-sequence frame staged by `Retrieve Stored V(r) I Frame`
    /// for the next `DL_DATA_indication` in the chain to deliver (PID, info).
    pub retrieved_stored_frame: Option<(u8, Vec<u8>)>,
}

impl<'a> Tx<'a> {
    /// Build a transition context for `trigger`.
    pub fn new(
        session: &'a mut SessionContext,
        timers: &'a mut dyn TimerService,
        sink: &'a mut dyn SessionSink,
        trigger: Event,
    ) -> Self {
        Self {
            session,
            timers,
            sink,
            trigger,
            pending: PendingFrame::default(),
            retrieved_stored_frame: None,
        }
    }
}
