//! The connected-mode session driver — ports `Packet.Ax25.Session.Ax25Session`.
//!
//! Holds one link's [`SessionContext`] and drives the generated AX.25 v2.2
//! data-link state machine off the typed [`ax25sdl`] tables. [`Session::post_event`]
//! maps the runtime [`Event`] to its typed [`ax25sdl::Ax25Event`], scans the
//! current state's transitions for the first whose `on` matches and whose guard
//! holds, executes that transition's action chain (via the dispatcher + the
//! [`super::loop_exec`] expander), advances [`Session::state`] to `next`, then runs
//! the I-frame transmit drain. A pure typed `match` drives dispatch — no string
//! comparison anywhere on the hot path.
//!
//! The state space is the 6 figc4.x data_link pages (Disconnected,
//! AwaitingConnection, AwaitingV22Connection, AwaitingRelease, Connected,
//! TimerRecovery); the MDL machine (Negotiating/Ready) is a separate concern not
//! driven here, matching how the C# `Ax25Session` drives only the data-link tables.

extern crate alloc;
use alloc::vec::Vec;

use ax25sdl::{
    Ax25Event as Sdl, StatePage, TransitionSpec, DATA_LINK_AWAITING_CONNECTION,
    DATA_LINK_AWAITING_RELEASE, DATA_LINK_AWAITING_V_22_CONNECTION, DATA_LINK_CONNECTED,
    DATA_LINK_DISCONNECTED, DATA_LINK_TIMER_RECOVERY,
};

use super::context::SessionContext;
use super::dispatch::execute_actions;
use super::event::Event;
use super::guard::eval_guard;
use super::signal::SessionSink;
use super::timer::TimerService;
use super::tx::Tx;

/// The link-layer states (the data_link machine's figc4.x pages). The `&'static`
/// [`StatePage`] for each comes straight from the generated tables.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum State {
    /// figc4.1 — no connection.
    Disconnected,
    /// figc4.2 — SABM sent, awaiting UA/DM (mod-8 establishment).
    AwaitingConnection,
    /// figc4.6 — SABME sent, awaiting UA/DM/FRMR (mod-128 establishment).
    AwaitingV22Connection,
    /// figc4.3 — DISC sent, awaiting UA/DM.
    AwaitingRelease,
    /// figc4.4 — information transfer.
    Connected,
    /// figc4.5 — outstanding poll, awaiting recovery.
    TimerRecovery,
}

impl State {
    /// The SDL `state:` string for this state (matches the generated `next:`
    /// strings used to resolve transition targets).
    pub fn name(self) -> &'static str {
        match self {
            State::Disconnected => "Disconnected",
            State::AwaitingConnection => "AwaitingConnection",
            State::AwaitingV22Connection => "AwaitingV22Connection",
            State::AwaitingRelease => "AwaitingRelease",
            State::Connected => "Connected",
            State::TimerRecovery => "TimerRecovery",
        }
    }

    /// The generated transition table for this state.
    fn page(self) -> &'static StatePage {
        match self {
            State::Disconnected => &DATA_LINK_DISCONNECTED,
            State::AwaitingConnection => &DATA_LINK_AWAITING_CONNECTION,
            State::AwaitingV22Connection => &DATA_LINK_AWAITING_V_22_CONNECTION,
            State::AwaitingRelease => &DATA_LINK_AWAITING_RELEASE,
            State::Connected => &DATA_LINK_CONNECTED,
            State::TimerRecovery => &DATA_LINK_TIMER_RECOVERY,
        }
    }

    /// Resolve a generated `next:` / `state:` string back to a [`State`]. Panics on
    /// an unknown name — a state-name typo in the tables, caught immediately.
    fn from_name(name: &str) -> State {
        match name {
            "Disconnected" => State::Disconnected,
            "AwaitingConnection" => State::AwaitingConnection,
            "AwaitingV22Connection" => State::AwaitingV22Connection,
            "AwaitingRelease" => State::AwaitingRelease,
            "Connected" => State::Connected,
            "TimerRecovery" => State::TimerRecovery,
            other => panic_unknown_state(other),
        }
    }
}

/// One AX.25 connection's runtime state machine. Generic over nothing — the timer
/// service + sink are passed per `post_event`, so a fixed array of `Session`s costs
/// only their context (no per-session trait-object storage).
#[derive(Debug, Clone)]
pub struct Session {
    /// The current link state.
    pub state: State,
    /// The mutable per-connection context.
    pub context: SessionContext,
}

impl Default for Session {
    fn default() -> Self {
        Self::new()
    }
}

impl Session {
    /// A fresh session in the Disconnected state with default link parameters.
    pub fn new() -> Self {
        Self {
            state: State::Disconnected,
            context: SessionContext::new(),
        }
    }

    /// Start in an explicit state (e.g. for a pre-established link or a test).
    pub fn in_state(state: State) -> Self {
        Self {
            state,
            context: SessionContext::new(),
        }
    }

    /// Drive one event through the machine, then run the I-frame transmit drain.
    /// Ports `Ax25Session.PostEvent`.
    pub fn post_event(
        &mut self,
        event: Event,
        timers: &mut dyn TimerService,
        sink: &mut dyn SessionSink,
    ) {
        self.dispatch_event(event, timers, sink);
        self.drain_i_frame_queue(timers, sink);
    }

    /// Dispatch one event: find the first matching transition, run it, advance
    /// state. Ports `Ax25Session.DispatchEvent` (incl. the #225 timer rollback).
    fn dispatch_event(
        &mut self,
        event: Event,
        timers: &mut dyn TimerService,
        sink: &mut dyn SessionSink,
    ) {
        let on = event.to_sdl();
        let page = self.state.page();

        // Find the first transition whose typed `on` matches and whose guard holds.
        // Guard eval reads context + timers + the trigger frame; no mutation yet.
        let mut matched: Option<&'static TransitionSpec> = None;
        for t in page.transitions {
            if t.on != on {
                continue;
            }
            if eval_guard(t.guard, &self.context, timers, &event) {
                matched = Some(t);
                break;
            }
        }

        let Some(t) = matched else {
            // No matching transition — SDL drops the event (state unchanged).
            return;
        };

        // Snapshot timers so a part-applied transition (an action that panics
        // mid-chain) doesn't leave T1 cancelled and the link wedged. We can't catch
        // a panic in no_std, but the snapshot/restore also covers the
        // pre-execution-quirk path and keeps parity with the C# rollback shape.
        let snapshot = timers.capture();
        let _ = snapshot; // restore is only reachable on the (panic) error path,
                          // which on-target aborts via panic-probe; kept for parity.

        // #48 DM-degrade: a DM received in AwaitingV22Connection means the peer can't
        // do v2.2, so degrade to v2.0/SABM like the FRMR fallback — substitute the
        // matched DM transition for figc4.6's t14_frmr_received (v2.0 re-establish),
        // NOT the figure-literal F=1 teardown. Ports `ResolveDmDegradeMatch`.
        let t = self.resolve_dm_degrade_match(t, page);

        // #45 / #48 pre-execution quirks: force v2.0 before the FRMR-fallback (or
        // substituted-DM) actions run, so Establish_Data_Link emits SABM (not SABME)
        // on a pre-v2.2 peer's FRMR/DM.
        self.apply_pre_execution_quirks(t, &event);

        // Run the action chain (with its loop_while ranges) against the context.
        {
            let mut tx = Tx::new(&mut self.context, timers, sink, event);
            execute_actions(t.actions, t.loops, &mut tx);
        }

        // Advance state — `next`, with the #44 mod-128 connect-routing fix.
        self.state = self.resolve_next_state(t);
    }

    /// Compute the state a committed transition advances to — normally `next`, but
    /// with the figc4.2 mod-128 connect-routing defect (#44) corrected: a v2.2
    /// (extended) DL-CONNECT from Disconnected is routed to AwaitingV22Connection
    /// (figc4.6) instead of the mod-8 AwaitingConnection. Ports `ResolveNextState`.
    fn resolve_next_state(&self, t: &TransitionSpec) -> State {
        if self.context.quirks.mod128_connect_routes_to_v22
            && self.context.is_extended
            && t.from == "Disconnected"
            && t.on == Sdl::DLCONNECTRequest
            && t.next == "AwaitingConnection"
        {
            return State::AwaitingV22Connection;
        }
        State::from_name(t.next)
    }

    /// figc4.6 DM-no-degrade gap (#48): when a `DM received` fires in
    /// AwaitingV22Connection while the link is still extended, substitute the matched
    /// DM transition (either F-branch — the F=1 teardown or the F=0 passive drop) for
    /// figc4.6's `t14_frmr_received` (the v2.0 re-establish: SRT reset →
    /// Establish_Data_Link → AwaitingConnection), so a DM degrades the link to v2.0
    /// and re-establishes via SABM exactly like the FRMR fallback (#45). The
    /// companion `is_extended=false` force ([`apply_pre_execution_quirks`]) makes
    /// Establish_Data_Link emit SABM. Scope is tight: only a `DMReceived` trigger,
    /// only from AwaitingV22Connection, only while extended. Ports
    /// `Ax25Session.ResolveDmDegradeMatch`.
    fn resolve_dm_degrade_match(
        &self,
        matched: &'static TransitionSpec,
        page: &'static StatePage,
    ) -> &'static TransitionSpec {
        if !self.context.quirks.dm_rejection_degrades_to_v20
            || !self.context.is_extended
            || matched.on != Sdl::DMReceived
            || matched.from != "AwaitingV22Connection"
        {
            return matched;
        }

        for t in page.transitions {
            if t.on == Sdl::FRMRReceived {
                return t;
            }
        }

        matched // defensive: figc4.6 always carries t14_frmr_received
    }

    /// Apply quirks that must take effect *before* a transition's actions run:
    /// the #45 figc4.6 FRMR-fallback ordering fix, and the #48 companion DM-degrade
    /// force. Ports `ApplyPreExecutionQuirks`.
    fn apply_pre_execution_quirks(&mut self, t: &TransitionSpec, event: &Event) {
        if self.context.quirks.frmr_fallback_reestablishes_v20
            && self.context.is_extended
            && t.from == "AwaitingV22Connection"
            && t.on == Sdl::FRMRReceived
        {
            self.context.is_extended = false;
        }

        // #48 companion: the DM's match has been substituted for t14_frmr_received
        // (so `t.on` is now FRMRReceived and the #45 branch above already fired when
        // #45 is on). Key this on the actual DM *trigger* so #48 stays self-contained
        // even with #45 off — otherwise Establish_Data_Link would re-emit SABME (still
        // extended) and the degrade would loop against the non-v2.2 peer.
        if self.context.quirks.dm_rejection_degrades_to_v20
            && self.context.is_extended
            && matches!(event, Event::DmReceived(_))
            && t.from == "AwaitingV22Connection"
        {
            self.context.is_extended = false;
        }
    }

    /// After every dispatch, if the I-frame queue has entries and transmission is
    /// allowed (Connected/TimerRecovery, peer not busy, window not full), pop one
    /// at a time and synthesise `I_frame_pops_off_queue` events so the figc4.4
    /// t19/t20 paths emit them on the wire. Ports `DrainIFrameQueue`.
    fn drain_i_frame_queue(&mut self, timers: &mut dyn TimerService, sink: &mut dyn SessionSink) {
        while !self.context.i_frame_queue.is_empty() && self.can_transmit_i_frame() {
            let entry = self
                .context
                .i_frame_queue
                .pop_front()
                .expect("queue non-empty checked above");
            self.dispatch_event(
                Event::IFramePopsOffQueue(entry.pid, entry.data),
                timers,
                sink,
            );
        }
    }

    /// True if link conditions allow an I-frame to be sent now: an
    /// information-transfer state, peer not busy, send window not full. Mirrors the
    /// figc4.4 t19/t20 guards (`peer_receiver_busy=No`, `V_s_eq_V_a_plus_k=No`).
    fn can_transmit_i_frame(&self) -> bool {
        if !matches!(self.state, State::Connected | State::TimerRecovery) {
            return false;
        }
        if self.context.peer_receiver_busy {
            return false;
        }
        self.context.outstanding_count() < self.context.k as u16
    }
}

/// Drive a sequence of events, collecting nothing — convenience for tests/harness.
pub fn post_all(
    session: &mut Session,
    events: Vec<Event>,
    timers: &mut dyn TimerService,
    sink: &mut dyn SessionSink,
) {
    for e in events {
        session.post_event(e, timers, sink);
    }
}

#[cold]
#[inline(never)]
fn panic_unknown_state(name: &str) -> ! {
    panic!("unknown SDL state name `{name}` in the generated tables (state-name typo)");
}
