//! Action dispatch — ports `Packet.Ax25.Session.ActionDispatcher`.
//!
//! [`execute_actions`] walks a transition/subroutine path's flat `actions` list,
//! expanding `loop_while` ranges (the [`super::loop_exec`] topology), and runs each
//! [`ax25sdl::ActionStep`] through [`execute_verb`] — a typed `match` over the
//! closed [`ax25sdl::Ax25ActionVerb`] set. Exhaustiveness is enforced by the
//! compiler (the Rust analogue of the C# CS8509 guarantee): a new or renamed verb
//! that lands without an arm is a build error, killing the verb-vs-dispatch bug
//! class (UI-reception, DL-DATA-while-connecting) at compile time.
//!
//! Three trigger-scoped verb rewrites (the SREJ/gap/drain spec-defect quirks #38,
//! #42, #47) are applied before dispatch, exactly as the C# does at the top of
//! `Execute(verb, tx)`.

extern crate alloc;

use ax25sdl::{ActionStep, Ax25ActionVerb};

use super::event::Event;
use super::guard::eval_term;
use super::signal::{
    DataLinkSignal, FrameSpec, InternalSignal, LinkMultiplexerSignal, SupervisoryKind,
    UnnumberedKind,
};
use super::subroutine;
use super::tx::Tx;

/// Execute an action chain (with its `loop_while` ranges) against `tx`. Ports
/// `SdlLoopExecutor.Execute(actions, loops, dispatcher, guards, tx)`: a flat walk
/// that, on reaching a loop body, repeats it while the loop predicate holds
/// (test-at-head / test-at-tail) up to [`super::loop_exec::MAX_ITERATIONS`].
///
/// Implemented as an inline walk (rather than the generic `run_loop` closure form)
/// so the per-action mutable borrow of `tx` and the per-iteration immutable borrow
/// for the predicate don't overlap — they're sequential.
pub fn execute_actions(actions: &[ActionStep], loops: &[ax25sdl::LoopRange], tx: &mut Tx<'_>) {
    let mut i = 0usize;
    while i < actions.len() {
        if let Some(range) = loops.iter().find(|r| r.start == i) {
            run_loop_range(range, actions, tx);
            i = range.start + range.length;
        } else {
            execute_verb(actions[i].verb, tx);
            i += 1;
        }
    }
}

fn run_loop_range(range: &ax25sdl::LoopRange, actions: &[ActionStep], tx: &mut Tx<'_>) {
    let body = range.start..range.start + range.length;
    let predicate_holds =
        |tx: &Tx<'_>| eval_term(range.predicate, tx.session, tx.timers, &tx.trigger);

    if range.test_at_end {
        let mut iters = 0;
        loop {
            for idx in body.clone() {
                execute_verb(actions[idx].verb, tx);
            }
            iters += 1;
            if iters >= super::loop_exec::MAX_ITERATIONS || !predicate_holds(tx) {
                break;
            }
        }
    } else {
        let mut iters = 0;
        while predicate_holds(tx) {
            for idx in body.clone() {
                execute_verb(actions[idx].verb, tx);
            }
            iters += 1;
            if iters >= super::loop_exec::MAX_ITERATIONS {
                break;
            }
        }
    }
}

/// Convenience: execute a slice of [`ActionStep`]s with no loops.
pub fn execute_steps(actions: &[ActionStep], tx: &mut Tx<'_>) {
    for step in actions {
        execute_verb(step.verb, tx);
    }
}

/// Execute a single action verb against `tx`. The exhaustive typed `match`.
pub fn execute_verb(verb: Ax25ActionVerb, tx: &mut Tx<'_>) {
    let verb = apply_verb_quirks(verb, tx);
    // The #38 quirk can suppress a verb entirely (skip Invoke_Retransmission on an
    // SREJ trigger); `apply_verb_quirks` signals that with `None` would complicate
    // the return type, so we re-check the one suppression case here.
    if tx.session.quirks.srej_selective_retransmit
        && matches!(tx.trigger, Event::SrejReceived(_))
        && verb == Ax25ActionVerb::InvokeRetransmission
    {
        return;
    }

    match verb {
        // ─── Flag mutations ─────────────────────────────────────────────
        Ax25ActionVerb::SetOwnReceiverBusy => tx.session.own_receiver_busy = true,
        Ax25ActionVerb::ClearOwnReceiverBusy => tx.session.own_receiver_busy = false,
        Ax25ActionVerb::SetPeerReceiverBusy => tx.session.peer_receiver_busy = true,
        Ax25ActionVerb::ClearPeerReceiverBusy => tx.session.peer_receiver_busy = false,
        Ax25ActionVerb::SetAcknowledgePending => tx.session.acknowledge_pending = true,
        Ax25ActionVerb::ClearAcknowledgePending => tx.session.acknowledge_pending = false,
        Ax25ActionVerb::SetLayer3Initiated => tx.session.layer3_initiated = true,
        Ax25ActionVerb::ClearLayer3Initiated => tx.session.layer3_initiated = false,

        // ─── Timer operations ───────────────────────────────────────────
        Ax25ActionVerb::StartT1 => {
            let d = tx.session.t1v_ms;
            tx.timers.arm(super::timer::TimerId::T1, d);
        }
        Ax25ActionVerb::StartT3 => {
            // T3 uses a fixed spec default (30 s); the figures never mutate it.
            tx.timers.arm(super::timer::TimerId::T3, T3_MS);
        }
        Ax25ActionVerb::StopT1 => {
            // Capture remaining BEFORE cancelling, for the Select_T1_Value sample.
            tx.session.t1_remaining_when_last_stopped_ms =
                tx.timers.time_remaining_ms(super::timer::TimerId::T1);
            tx.timers.cancel(super::timer::TimerId::T1);
        }
        Ax25ActionVerb::StopT3 => tx.timers.cancel(super::timer::TimerId::T3),

        // ─── Supervisory-frame transmissions ────────────────────────────
        Ax25ActionVerb::RRCommand | Ax25ActionVerb::RRCommandPEq0 => {
            send_s(tx, SupervisoryKind::Rr, true)
        }
        Ax25ActionVerb::RR | Ax25ActionVerb::RRResponse => send_s(tx, SupervisoryKind::Rr, false),
        Ax25ActionVerb::RNRCommand => send_s(tx, SupervisoryKind::Rnr, true),
        Ax25ActionVerb::RNR | Ax25ActionVerb::RNRResponse | Ax25ActionVerb::RNRResponseFEq0 => {
            send_s(tx, SupervisoryKind::Rnr, false)
        }
        Ax25ActionVerb::REJ => send_s(tx, SupervisoryKind::Rej, false),
        Ax25ActionVerb::SREJ => send_s(tx, SupervisoryKind::Srej, false),

        // ─── Unnumbered-frame transmissions ─────────────────────────────
        Ax25ActionVerb::UA => send_u(tx, UnnumberedKind::Ua, false, None, false),
        Ax25ActionVerb::DM | Ax25ActionVerb::DMResponseFEq0 => {
            send_u(tx, UnnumberedKind::Dm, false, None, false)
        }
        Ax25ActionVerb::DMFEq1 => send_u(tx, UnnumberedKind::Dm, false, Some(true), false),
        Ax25ActionVerb::ExpeditedUA => send_u(tx, UnnumberedKind::Ua, false, None, true),
        Ax25ActionVerb::ExpeditedDM => send_u(tx, UnnumberedKind::Dm, false, None, true),
        Ax25ActionVerb::SABM | Ax25ActionVerb::SABMPEqEq1 => {
            send_u(tx, UnnumberedKind::Sabm, true, Some(true), false)
        }
        Ax25ActionVerb::SABME | Ax25ActionVerb::SABMEPEq1 => {
            send_u(tx, UnnumberedKind::Sabme, true, Some(true), false)
        }
        Ax25ActionVerb::DISCPEq1 => send_u(tx, UnnumberedKind::Disc, true, Some(true), false),

        // ─── UI-frame transmission ──────────────────────────────────────
        Ax25ActionVerb::UICommand => send_ui(tx),

        // ─── I-frame transmission ───────────────────────────────────────
        Ax25ActionVerb::ICommand => emit_i_frame(tx),

        // ─── DL upper-layer signals (signal_upper) ──────────────────────
        Ax25ActionVerb::DLCONNECTIndication => {
            tx.sink.send_upward(DataLinkSignal::ConnectIndication)
        }
        Ax25ActionVerb::DLCONNECTConfirm => tx.sink.send_upward(DataLinkSignal::ConnectConfirm),
        Ax25ActionVerb::DLDISCONNECTIndication => {
            tx.sink.send_upward(DataLinkSignal::DisconnectIndication)
        }
        Ax25ActionVerb::DLDISCONNECTConfirm => {
            tx.sink.send_upward(DataLinkSignal::DisconnectConfirm)
        }
        Ax25ActionVerb::DLDATAIndication => build_data_indication(tx),
        Ax25ActionVerb::DLUNITDATAIndication => {
            let (pid, info) = incoming_info(tx);
            tx.sink
                .send_upward(DataLinkSignal::UnitDataIndication(pid, info));
        }

        // ─── DL_ERROR_indication_* (§C5 letter codes) ───────────────────
        Ax25ActionVerb::DLERRORIndicationCD => err(tx, "C_D"),
        Ax25ActionVerb::DLERRORIndicationD => err(tx, "D"),
        Ax25ActionVerb::DLERRORIndicationE => err(tx, "E"),
        Ax25ActionVerb::DLERRORIndicationF => err(tx, "F"),
        Ax25ActionVerb::DLERRORIndicationG => err(tx, "G"),
        Ax25ActionVerb::DLERRORIndicationI => err(tx, "I"),
        Ax25ActionVerb::DLERRORIndicationK => err(tx, "K"),
        Ax25ActionVerb::DLERRORIndicationL => err(tx, "L"),
        Ax25ActionVerb::DLERRORIndicationM => err(tx, "M"),
        Ax25ActionVerb::DLERRORIndicationN => err(tx, "N"),
        Ax25ActionVerb::DLERRORIndicationO => err(tx, "O"),
        Ax25ActionVerb::DLERRORIndicationT => err(tx, "T"),
        Ax25ActionVerb::DLERRORIndicationU => err(tx, "U"),
        Ax25ActionVerb::DLERRORIndicationA => err(tx, "A"),
        Ax25ActionVerb::DLERRORIndicationJ => err(tx, "J"),
        Ax25ActionVerb::DLERRORIndicationQ => err(tx, "Q"),
        Ax25ActionVerb::DLERRORIndicationAdd => err(tx, "add"),

        // ─── Link-multiplexer signals (signal_lower) ────────────────────
        Ax25ActionVerb::LMSeizeRequest => {
            tx.sink.send_link_mux(LinkMultiplexerSignal::SeizeRequest)
        }
        Ax25ActionVerb::LMReleaseRequest => {
            tx.sink.send_link_mux(LinkMultiplexerSignal::ReleaseRequest)
        }
        Ax25ActionVerb::LMDataRequest => tx.sink.send_link_mux(LinkMultiplexerSignal::DataRequest),

        // ─── Internal-out signals ───────────────────────────────────────
        Ax25ActionVerb::MDLNEGOTIATERequest => {
            tx.sink.send_internal(InternalSignal::MdlNegotiateRequest)
        }

        // ─── Management Data-Link verbs (figc5.x) — out of scope here ────
        //
        // These are emitted ONLY by the management_data_link machine
        // (Negotiating/Ready), which this data-link runtime does not drive (the
        // C# `Ax25Session` likewise drives only the data-link tables; the MDL has
        // its own driver). The data-link state pages never carry these verbs, so
        // they are unreachable in practice — but the typed `match` must stay
        // exhaustive (the SP-010 guarantee), so they are explicit inert arms. If
        // the MDL machine is ported later, these gain real bodies + sink methods.
        Ax25ActionVerb::XIDCommand
        | Ax25ActionVerb::StartTM201
        | Ax25ActionVerb::StopTM201
        | Ax25ActionVerb::ApplyNegotiatedParameters
        | Ax25ActionVerb::MDLNEGOTIATEConfirm
        | Ax25ActionVerb::MDLERRORIndicateB
        | Ax25ActionVerb::MDLERRORIndicateC
        | Ax25ActionVerb::MDLERRORIndicateD => { /* MDL-only; unreachable in data_link */ }

        // ─── I-frame queue pushes ───────────────────────────────────────
        Ax25ActionVerb::PushOnIFrameQueue
        | Ax25ActionVerb::PushOnIFrameQueueNoteWordOrder
        | Ax25ActionVerb::PushFrameOnQueue
        | Ax25ActionVerb::PushIFrameOnIQueue => push_on_i_frame_queue(tx),
        Ax25ActionVerb::PushOldIFrameNROnQueue => push_old_i_frame_nr(tx),

        // ─── Queue / storage clears ─────────────────────────────────────
        Ax25ActionVerb::DiscardFrameQueue
        | Ax25ActionVerb::DiscardQueue
        | Ax25ActionVerb::DiscardIFrameQueue
        | Ax25ActionVerb::DiscardIQueueEntries => tx.session.i_frame_queue.clear(),
        Ax25ActionVerb::DiscardIFrame
        | Ax25ActionVerb::DiscardContentsOfIFrame
        | Ax25ActionVerb::DiscardPrimitive => { /* no-op: incoming not stored */ }

        // ─── SREJ exception bookkeeping ─────────────────────────────────
        Ax25ActionVerb::SetRejectException => tx.session.reject_exception = true,
        Ax25ActionVerb::ClearRejectException | Ax25ActionVerb::ClearRejectCondition => {
            tx.session.reject_exception = false
        }
        Ax25ActionVerb::ClearSrejectCondition => {
            tx.session.selective_reject_exception = false;
            tx.session.srej_exception_count = 0;
        }
        Ax25ActionVerb::IncrementSrejectException | Ax25ActionVerb::SrejectAssignSrejectPlus1 => {
            tx.session.srej_exception_count += 1;
            tx.session.selective_reject_exception = true;
        }
        Ax25ActionVerb::DecrementSrejectExceptionIf0 => {
            if tx.session.srej_exception_count > 0 {
                tx.session.srej_exception_count -= 1;
                if tx.session.srej_exception_count == 0 {
                    tx.session.selective_reject_exception = false;
                }
            }
        }

        // ─── Out-of-sequence I-frame storage ────────────────────────────
        Ax25ActionVerb::SaveContentsOfIFrame => save_incoming_i_frame(tx),
        Ax25ActionVerb::RetrieveStoredVRIFrame => retrieve_stored_vr_i_frame(tx),

        // ─── Version / modulus / link params ────────────────────────────
        Ax25ActionVerb::SetVersion20 => set_version_20(tx),
        Ax25ActionVerb::SetVersion22 => tx.session.is_extended = true,
        Ax25ActionVerb::SetHalfDuplex => tx.session.half_duplex = true,
        Ax25ActionVerb::SetImplicitReject => {
            tx.session.implicit_reject = true;
            tx.session.srej_enabled = false;
        }
        Ax25ActionVerb::SetSelectiveReject => {
            tx.session.implicit_reject = false;
            tx.session.srej_enabled = true;
        }
        Ax25ActionVerb::ModuloAssign8 => tx.session.is_extended = false,
        Ax25ActionVerb::ModuloAssign128 => tx.session.is_extended = true,
        Ax25ActionVerb::N1Assign2048 => tx.session.n1 = 2048,
        Ax25ActionVerb::KAssign8 => tx.session.k = 8,
        Ax25ActionVerb::KAssign32 => tx.session.k = 32,
        Ax25ActionVerb::T2Assign3000 => tx.session.t2_ms = 3000,
        Ax25ActionVerb::N2Assign10 => tx.session.n2 = 10,

        // ─── Link-parameter assignments (SRT, T1V) ──────────────────────
        Ax25ActionVerb::SRTAssignInitialDefault => tx.session.srt_ms = INITIAL_SRT_MS,
        Ax25ActionVerb::T1VAssign2TimesSRT | Ax25ActionVerb::NextT1Assign2TimesSRT => {
            tx.session.t1v_ms = tx.session.srt_ms.saturating_mul(2)
        }
        Ax25ActionVerb::NextT1AssignRCTimes025PlusSRTTimes2 => {
            tx.session.t1v_ms =
                super::timer::next_t1_rc_backoff_ms(tx.session.rc, tx.session.srt_ms);
            tx.session.t1_had_expired = false;
        }
        Ax25ActionVerb::SRTAssign7SRT8PlusT18RemainingTimeOnT1WhenLastStopped8 => {
            let karn = tx.session.quirks.karn_srt_sampling;
            tx.session.srt_ms = super::timer::srt_iir_update(
                tx.session.srt_ms,
                tx.session.t1v_ms,
                tx.session.t1_remaining_when_last_stopped_ms,
                karn,
            );
            tx.session.t1_had_expired = false;
            tx.session.t1_remaining_when_last_stopped_ms = 0;
        }

        // ─── Subroutine calls ───────────────────────────────────────────
        Ax25ActionVerb::EstablishDataLink => subroutine::invoke("Establish_Data_Link", tx),
        Ax25ActionVerb::ClearExceptionConditions => {
            subroutine::invoke("Clear_Exception_Conditions", tx)
        }
        Ax25ActionVerb::UICheck => subroutine::invoke("UI_Check", tx),
        Ax25ActionVerb::SelectT1Value => subroutine::invoke("Select_T1", tx),
        Ax25ActionVerb::CheckIFrameAcknowledged => {
            subroutine::invoke("Check_I_Frame_Acknowledged", tx)
        }
        Ax25ActionVerb::CheckNeedForResponse => subroutine::invoke("Check_Need_for_Response", tx),
        Ax25ActionVerb::TransmitEnquiry | Ax25ActionVerb::TransmitEnquery => {
            subroutine::invoke("Transmit_Enquiry", tx)
        }
        Ax25ActionVerb::InvokeRetransmission => subroutine::invoke("Invoke_Retransmission", tx),
        Ax25ActionVerb::NRErrorRecovery | Ax25ActionVerb::NRRecovery => {
            subroutine::invoke("N_r_Error_Recovery", tx)
        }
        Ax25ActionVerb::EnquiryResponseFEq0 => subroutine::invoke_enquiry_response(tx, false),
        Ax25ActionVerb::EnquiryResponseF1 => subroutine::invoke_enquiry_response(tx, true),

        // ─── Sequence-variable assignments (pure context) ───────────────
        Ax25ActionVerb::VSAssign0 => tx.session.vs = 0,
        Ax25ActionVerb::VSAssignVSPlus1 => tx.session.vs = tx.session.increment_seq(tx.session.vs),
        Ax25ActionVerb::VRAssign0 => tx.session.vr = 0,
        Ax25ActionVerb::VRAssignVRPlus1 => tx.session.vr = tx.session.increment_seq(tx.session.vr),
        Ax25ActionVerb::VRAssignVR1 => tx.session.vr = tx.session.decrement_seq(tx.session.vr),
        Ax25ActionVerb::VAAssign0 => tx.session.va = 0,
        Ax25ActionVerb::XAssignVS => tx.session.x = Some(tx.session.vs),
        Ax25ActionVerb::VSAssignNR => tx.session.vs = extract_nr(tx),
        Ax25ActionVerb::RCAssign0 => tx.session.rc = 0,
        Ax25ActionVerb::RCAssign1 => tx.session.rc = 1,
        Ax25ActionVerb::RCAssignRCPlus1 => tx.session.rc += 1,

        // V(a) := N(r) — advance ack state; prune now-acked frames + clear the
        // once-per-cycle selective-retransmit set on genuine progress.
        Ax25ActionVerb::VAAssignNR => {
            let previous_va = tx.session.va;
            tx.session.va = extract_nr(tx);
            if tx.session.va != previous_va {
                tx.session.selectively_retransmitted_since_ack.clear();
                tx.session.prune_acknowledged_sent_i_frames();
            }
        }

        // ─── Pending-frame field assignments (write side) ───────────────
        Ax25ActionVerb::NRAssignVR => tx.pending.nr = Some(tx.session.vr),
        Ax25ActionVerb::NSAssignVS => tx.pending.ns = Some(tx.session.vs),
        Ax25ActionVerb::NRAssignNS => tx.pending.nr = Some(extract_ns(tx)),
        Ax25ActionVerb::FAssign0 | Ax25ActionVerb::PAssign0 => tx.pending.pf = Some(false),
        Ax25ActionVerb::FAssign1 | Ax25ActionVerb::PAssign1 => tx.pending.pf = Some(true),
        Ax25ActionVerb::FAssignP => tx.pending.pf = Some(extract_poll_final(tx)),

        // ─── Invoke_Retransmission body markers ─────────────────────────
        Ax25ActionVerb::Backtrack => { /* informational marker, no state effect */ }
        Ax25ActionVerb::PushOldIFrameOntoQueue => {
            let ns = tx.session.vs;
            emit_old_i_frame(tx, ns, false);
        }
    }
}

/// Apply the three trigger-scoped verb rewrites (the SREJ #38, gap #42, drain #47
/// quirks) before dispatch. Mirrors the top of the C# `Execute(verb, tx)`.
fn apply_verb_quirks(mut verb: Ax25ActionVerb, tx: &Tx<'_>) -> Ax25ActionVerb {
    let q = &tx.session.quirks;

    // #38 — SREJ selective retransmit: on an SREJ trigger, redirect a fresh-push
    // verb to the single-frame "Push Old I Frame N(r)" behaviour. (The
    // Invoke_Retransmission suppression is handled in execute_verb.)
    if q.srej_selective_retransmit && matches!(tx.trigger, Event::SrejReceived(_)) {
        verb = match verb {
            Ax25ActionVerb::PushOnIFrameQueue
            | Ax25ActionVerb::PushOnIFrameQueueNoteWordOrder
            | Ax25ActionVerb::PushFrameOnQueue
            | Ax25ActionVerb::PushIFrameOnIQueue => Ax25ActionVerb::PushOldIFrameNROnQueue,
            other => other,
        };
    }

    // #42 — SREJ targets gap: on an I_received trigger, retarget `N(r) := N(s)`
    // (request the just-arrived frame) to `N(r) := V(r)` (the next missing gap).
    if q.srej_targets_gap
        && matches!(tx.trigger, Event::IReceived(_))
        && verb == Ax25ActionVerb::NRAssignNS
    {
        verb = Ax25ActionVerb::NRAssignVR;
    }

    // #47 — Timer Recovery drain advances V(R): rewrite the figc4.5 drain's
    // `V(r) := V(r) - 1` to `+ 1`. Precisely scoped: the decrement appears only in
    // those drain loops, so no trigger gate is needed.
    if q.timer_recovery_drain_advances_vr && verb == Ax25ActionVerb::VRAssignVR1 {
        verb = Ax25ActionVerb::VRAssignVRPlus1;
    }

    verb
}

// ─── Spec-default integer constants (research §3) ───────────────────────────

/// T3 (inactive-link timer) default — 30 s.
const T3_MS: u32 = 30_000;
/// SRT "Initial Default" (§6.7.1.2) — 3 s ⇒ T1V 6 s.
const INITIAL_SRT_MS: u32 = 3000;

// ─── Frame-emission helpers (mirror BuildSFrame / EmitIFrame / …) ───────────

/// Raise a DL-ERROR indication with the §C5 letter code.
fn err(tx: &mut Tx<'_>, code: &'static str) {
    tx.sink.send_upward(DataLinkSignal::ErrorIndication(code));
}

fn send_s(tx: &mut Tx<'_>, kind: SupervisoryKind, is_command: bool) {
    let nr = tx.pending.nr.unwrap_or(tx.session.vr);
    let pf = tx.pending.pf.unwrap_or(false);
    tx.sink.send_frame(FrameSpec::Supervisory {
        kind,
        is_command,
        nr,
        pf,
    });
}

fn send_u(
    tx: &mut Tx<'_>,
    kind: UnnumberedKind,
    is_command: bool,
    pf_override: Option<bool>,
    expedited: bool,
) {
    let pf = pf_override.or(tx.pending.pf).unwrap_or(false);
    tx.sink.send_frame(FrameSpec::Unnumbered {
        kind,
        is_command,
        pf,
        expedited,
    });
}

fn send_ui(tx: &mut Tx<'_>) {
    let (pid, info) = match &tx.trigger {
        Event::DlUnitDataRequest(pid, info) => (*pid, info.clone()),
        _ => panic_trigger("UI_command", "DL_UNIT_DATA_request"),
    };
    let pf = tx.pending.pf.unwrap_or(false);
    tx.sink.send_frame(FrameSpec::Ui {
        is_command: true,
        pf,
        pid,
        info,
    });
}

fn emit_i_frame(tx: &mut Tx<'_>) {
    let (pid, info) = match &tx.trigger {
        Event::IFramePopsOffQueue(pid, info) => (*pid, info.clone()),
        _ => panic_trigger("I_command", "I_frame_pops_off_queue"),
    };
    let ns = tx.pending.ns.unwrap_or(tx.session.vs);
    let nr = tx.pending.nr.unwrap_or(tx.session.vr);
    let p = tx.pending.pf.unwrap_or(false);
    tx.sink.send_frame(FrameSpec::Information {
        p,
        nr,
        ns,
        pid,
        info: info.clone(),
    });
    tx.session
        .sent_i_frames
        .insert(ns, super::context::Payload::new(info, pid));
    // Freshly transmitted data at this N(S): a stale "already retransmitted" mark
    // from a prior ring cycle must not suppress a future SREJ-driven replay.
    tx.session.selectively_retransmitted_since_ack.remove(&ns);
}

/// `push_old_I_frame_N_r_on_queue` (figc4.4 REJ/SREJ retransmit): re-emit the
/// previously-sent I-frame at the incoming N(R), selective-replay guarded.
fn push_old_i_frame_nr(tx: &mut Tx<'_>) {
    let ns = extract_nr(tx);
    emit_old_i_frame(tx, ns, true);
}

/// Retransmit a stored I-frame at its ORIGINAL N(S). `selective_replay` applies
/// the #231 once-per-cycle + still-outstanding guard (REJ/SREJ path); the
/// go-back-N `Invoke_Retransmission` replay sets it false.
fn emit_old_i_frame(tx: &mut Tx<'_>, ns: u8, selective_replay: bool) {
    if selective_replay {
        if !tx.session.is_outstanding(ns) {
            return;
        }
        if !tx.session.selectively_retransmitted_since_ack.insert(ns) {
            return; // already replayed this cycle
        }
    }
    let entry = match tx.session.sent_i_frames.get(&ns) {
        Some(e) => e.clone(),
        None => return, // evicted — matches linbpq/direwolf
    };
    let nr = tx.session.vr;
    tx.sink.send_frame(FrameSpec::Information {
        p: false,
        nr,
        ns,
        pid: entry.pid,
        info: entry.data,
    });
}

fn push_on_i_frame_queue(tx: &mut Tx<'_>) {
    let (pid, data) = match &tx.trigger {
        Event::DlDataRequest(pid, data) => (*pid, data.clone()),
        _ => panic_trigger("push_*_queue", "DL_DATA_request"),
    };
    tx.session
        .i_frame_queue
        .push_back(super::context::Payload::new(data.clone(), pid));
    tx.sink.send_internal(InternalSignal::PushIFrameQueue(data));
}

fn save_incoming_i_frame(tx: &mut Tx<'_>) {
    let frame = require_frame(tx, "save_contents_of_I_frame");
    let ns = frame.ns;
    let pid = frame
        .pid
        .expect("save_contents_of_I_frame requires an I-frame trigger (carries PID)");
    let info = frame.info.clone();
    tx.session
        .stored_received_i_frames
        .insert(ns, super::context::Payload::new(info, pid));
}

fn retrieve_stored_vr_i_frame(tx: &mut Tx<'_>) {
    let vr = tx.session.vr;
    if let Some(stored) = tx.session.stored_received_i_frames.remove(&vr) {
        tx.retrieved_stored_frame = Some((stored.pid, stored.data));
    }
}

fn build_data_indication(tx: &mut Tx<'_>) {
    // A preceding Retrieve stages the stored frame to deliver here; consume it.
    if let Some((pid, info)) = tx.retrieved_stored_frame.take() {
        tx.sink
            .send_upward(DataLinkSignal::DataIndication(pid, info));
        return;
    }
    let frame = require_frame(tx, "DL_DATA_indication");
    let pid = frame
        .pid
        .expect("DL_DATA_indication requires the incoming frame to carry a PID");
    let info = frame.info.clone();
    tx.sink
        .send_upward(DataLinkSignal::DataIndication(pid, info));
}

/// `set_version_2_0` — the data-link figc4.6 semantics: clear `is_extended` (the
/// other v2.0 verbs in the chain run separately). The MDL driver would override
/// this with the full §1436 set; the data-link runtime uses the figure default.
fn set_version_20(tx: &mut Tx<'_>) {
    tx.session.is_extended = false;
}

// ─── Incoming-frame field extraction (mirror Extract* / Require*) ───────────

fn extract_nr(tx: &Tx<'_>) -> u8 {
    require_frame(tx, "V(a) := N(r)").nr
}
fn extract_ns(tx: &Tx<'_>) -> u8 {
    require_frame(tx, "N(r) := N(s)").ns
}
fn extract_poll_final(tx: &Tx<'_>) -> bool {
    require_frame(tx, "F := P").poll_final
}
fn incoming_info(tx: &Tx<'_>) -> (u8, alloc::vec::Vec<u8>) {
    let f = require_frame(tx, "DL-UNIT-DATA Indication");
    (f.pid.unwrap_or(crate::ax25::PID_NO_LAYER3), f.info.clone())
}

fn require_frame<'b>(tx: &'b Tx<'_>, verb: &str) -> &'b super::event::FrameInfo {
    tx.trigger.frame().unwrap_or_else(|| {
        // Mirrors the C# RequireIncomingFrame throw — a transcription / wiring bug,
        // not a wire condition. On-target this aborts via panic-probe with the verb
        // name; host tests see the message.
        panic_no_frame(verb)
    })
}

#[cold]
#[inline(never)]
fn panic_no_frame(verb: &str) -> ! {
    panic!(
        "action `{verb}` requires an incoming frame, but the trigger is not a frame-receipt event"
    );
}

#[cold]
#[inline(never)]
fn panic_trigger(verb: &str, expected: &str) -> ! {
    panic!("action `{verb}` requires the trigger to be {expected}");
}
