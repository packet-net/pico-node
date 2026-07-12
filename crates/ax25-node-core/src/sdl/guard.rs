//! Guard evaluation — ports `GuardEvaluator` + `Ax25SessionBindings.CreateDefault`.
//!
//! A transition's `guard` is a conjunction of optionally-negated typed
//! [`ax25sdl::Ax25Guard`] atoms ([`ax25sdl::GuardTerm`]). [`eval_guard`] holds
//! when every term holds; an empty slice means unguarded. [`eval_atom`] resolves a
//! single atom against the session [`SessionContext`] + the timer state + the
//! triggering [`Event`]'s frame — a typed `match` over the closed set, so a new or
//! renamed atom is a *compile error* here (the exhaustiveness the C# CS8509 buys),
//! never an unbound-identifier surprise at runtime.
//!
//! The two binding-side spec quirks (`discard_out_of_window_i_frames` #40 and
//! `dl_flow_off_enters_busy` #43) are applied inline where the C# overrides the
//! base binding after building the table.

use ax25sdl::{Ax25Guard, GuardTerm};

use super::context::SessionContext;
use super::event::Event;
use super::timer::{TimerId, TimerService};

/// Evaluate a guard conjunction. `true` when every term holds; an empty slice is
/// unguarded (always fires). Ports `GuardEvaluator.Evaluate`.
pub fn eval_guard(
    guard: &[GuardTerm],
    ctx: &SessionContext,
    timers: &dyn TimerService,
    trigger: &Event,
) -> bool {
    for term in guard {
        let value = eval_atom(term.atom, ctx, timers, trigger);
        let held = if term.negate { !value } else { value };
        if !held {
            return false;
        }
    }
    true
}

/// Evaluate one guard term (the shape [`ax25sdl::LoopRange::predicate`] carries).
pub fn eval_term(
    term: GuardTerm,
    ctx: &SessionContext,
    timers: &dyn TimerService,
    trigger: &Event,
) -> bool {
    let value = eval_atom(term.atom, ctx, timers, trigger);
    if term.negate {
        !value
    } else {
        value
    }
}

/// Resolve a single [`Ax25Guard`] atom to its boolean value. Exhaustive typed
/// `match` — mirrors `Ax25SessionBindings.CreateDefault`'s `BindAtom` switch.
pub fn eval_atom(
    atom: Ax25Guard,
    ctx: &SessionContext,
    timers: &dyn TimerService,
    trigger: &Event,
) -> bool {
    let frame = trigger.frame();
    let m = ctx.modulus();

    match atom {
        // ─── Session flags (§6.x) ───────────────────────────────────────
        Ax25Guard::OwnReceiverBusy => own_receiver_busy(ctx, trigger),
        Ax25Guard::PeerReceiverBusy => ctx.peer_receiver_busy,
        Ax25Guard::AckPending => ctx.acknowledge_pending,
        Ax25Guard::RejectException => reject_exception(ctx, trigger),
        Ax25Guard::Layer3Initiated => ctx.layer3_initiated,
        Ax25Guard::SREJEnabled => ctx.srej_enabled,
        Ax25Guard::SrejectExceptionGt0 => ctx.srej_exception_count > 0,
        Ax25Guard::OutOfSequenceFramesInReceiveBuffer => !ctx.stored_received_i_frames.is_empty(),
        Ax25Guard::VrIFrameStored => ctx.stored_received_i_frames.contains_key(&ctx.vr),

        // ─── Node policy (figc4.1 SABM_received decision) ───────────────
        Ax25Guard::AbleToEstablish => ctx.accept_incoming,

        // ─── Version / modulus (figc4.7 Mod 128? / Mod 8?) ──────────────
        Ax25Guard::Mod128 => ctx.is_extended,
        Ax25Guard::Mod8 => !ctx.is_extended,
        Ax25Guard::Version22 => ctx.is_extended,

        // ─── Sequence-variable comparisons (mod-aware) ──────────────────
        Ax25Guard::VsEqVa => ctx.vs == ctx.va,
        // #13: window bound is the effective (SREJ-clamped) window, not raw `k`.
        Ax25Guard::VsEqVaPlusK => ctx.outstanding_count() >= ctx.effective_window() as u16,
        Ax25Guard::VsEqX => ctx.x == Some(ctx.vs),

        // ─── Timer state ────────────────────────────────────────────────
        Ax25Guard::T1Running => timers.is_running(TimerId::T1),
        Ax25Guard::T1Expired => ctx.t1_had_expired,

        // ─── Retry-counter comparisons ──────────────────────────────────
        Ax25Guard::RCEq0 => ctx.rc == 0,
        Ax25Guard::RCEqN2 => ctx.rc == ctx.n2,
        // RC == NM201 — MDL only; carried in the data-link context's N2 (inert
        // for the data-link machine, which never reaches this atom).
        Ax25Guard::RCEqNM201 => ctx.rc == ctx.n2,

        // ─── Frame-aware: poll/final + command/response ─────────────────
        Ax25Guard::PEq1 => frame.map(|f| f.poll_final).unwrap_or(false),
        Ax25Guard::FEq1 => frame.map(|f| f.poll_final).unwrap_or(false),
        Ax25Guard::POrFEq1 => frame.map(|f| f.poll_final).unwrap_or(false),
        Ax25Guard::Command => frame.map(|f| f.is_command).unwrap_or(false),
        Ax25Guard::Response => frame.map(|f| f.is_response()).unwrap_or(false),
        Ax25Guard::CommandAndPEq1 => frame.map(|f| f.is_command && f.poll_final).unwrap_or(false),
        Ax25Guard::ResponseAndFEq1 => frame
            .map(|f| f.is_response() && f.poll_final)
            .unwrap_or(false),
        // Enquiry_Response's compound: F set AND the frame is RR/RNR/I. We only
        // hold a FrameInfo (already classified), so honour it for the
        // poll-able-shape frames the runtime actually feeds (I/RR/RNR all arrive
        // with poll_final set on a poll); REJ/SREJ never carry it into this path.
        Ax25Guard::FEq1AndFrameEqRROrFrameEqRNROrFrameEqI => {
            frame.map(|f| f.poll_final).unwrap_or(false)
        }

        // ─── Frame-aware: received N(s)/N(r) comparisons ────────────────
        Ax25Guard::NsEqVr => frame.map(|f| f.ns == ctx.vr).unwrap_or(false),
        Ax25Guard::NsGtVrPlus1 => frame
            .map(|f| {
                let diff = ((f.ns as u16 + m) - ctx.vr as u16) % m;
                diff > 1
            })
            .unwrap_or(false),
        // N(R) in the window [V(a), V(s)] (inclusive both ends, mod-N).
        Ax25Guard::VaLeNrLeVs => frame
            .map(|f| {
                let span = ((ctx.vs as u16 + m) - ctx.va as u16) % m;
                let nr_delta = ((f.nr as u16 + m) - ctx.va as u16) % m;
                nr_delta <= span
            })
            .unwrap_or(false),
        Ax25Guard::NrEqVa => frame.map(|f| f.nr == ctx.va).unwrap_or(false),
        Ax25Guard::NrEqVs => frame.map(|f| f.nr == ctx.vs).unwrap_or(false),
        // vs_eq_nr is the same comparison as n_r_eq_v_s.
        Ax25Guard::VsEqNr => frame.map(|f| f.nr == ctx.vs).unwrap_or(false),

        // ─── Frame-content validity ─────────────────────────────────────
        Ax25Guard::InfoFieldLengthLeN1AndContentIsOctetAligned => frame
            .map(|f| (f.info.len() as u32) <= ctx.n1)
            .unwrap_or(false),
    }
}

/// `own_receiver_busy` with the #43 DL-FLOW-OFF inversion applied: for a
/// `DlFlowOffRequest` trigger (when the quirk is on) a not-busy station takes the
/// action branch and an already-busy one no-ops.
fn own_receiver_busy(ctx: &SessionContext, trigger: &Event) -> bool {
    let base = ctx.own_receiver_busy;
    if ctx.quirks.dl_flow_off_enters_busy && matches!(trigger, Event::DlFlowOffRequest) {
        !base
    } else {
        base
    }
}

/// `reject_exception` with the #40 out-of-window discard ORed in: for an
/// `IReceived` trigger (when the quirk is on) a frame whose N(S) is outside the
/// receive window `[V(r), V(r)+k)` takes the figure's discard path.
fn reject_exception(ctx: &SessionContext, trigger: &Event) -> bool {
    let base = ctx.reject_exception;
    if !ctx.quirks.discard_out_of_window_i_frames {
        return base;
    }
    if let Event::IReceived(f) = trigger {
        let m = ctx.modulus();
        let offset = ((f.ns as u16 + m) - ctx.vr as u16) % m;
        // #13: the receive window bound is the effective (SREJ-clamped) window.
        if offset >= ctx.effective_window() as u16 {
            return true; // out of window ⇒ discard
        }
    }
    base
}
