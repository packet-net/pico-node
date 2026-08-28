//! Management Data-Link (MDL) XID negotiation - ports
//! `Packet.Ax25.Session.XidNegotiator` + `Ax25ManagementDataLink` +
//! `Ax25Listener.HandleNoCachedSession`'s pre-session XID branch.
//!
//! The AX.25 v2.2 MDL (Appendix C5) is the XID parameter-negotiation FSM that
//! turns SREJ / segmentation / modulo / window / T1 / N2 from forced establishment
//! defaults into *negotiated* link parameters. Three pieces:
//!
//! - The **[`MdlMachine`]** - the runtime driver for the generated figc5.1/figc5.2
//!   state pages ([`ax25sdl::MANAGEMENT_DATA_LINK_READY`] /
//!   [`ax25sdl::MANAGEMENT_DATA_LINK_NEGOTIATING`]), consuming the
//!   MDL-NEGOTIATE Request poke figc4.6 raises after a v2.2 UA and fronting the
//!   initiator pre-connect probe. Mirrors `Ax25ManagementDataLink`.
//! - The **§6.3.2 reverts-to merge** ([`apply_negotiated`]) that turns our offer
//!   and the peer's XID into agreed link parameters, plus the §1436 version-2.0
//!   default set ([`apply_version_20_defaults`]) and the offer derivation
//!   ([`default_offer_for`]). These mirror `XidNegotiator` (the collapsed
//!   figc5.3-5.8 subroutine bodies).
//! - The **pre-session XID *command* responder** ([`respond_pre_session_xid`]) —
//!   the un-transcribed figc5.1 responder column that answers an inbound XID
//!   command *before* a session exists (a PDN NET/ROM mod-8 interlink initiator
//!   opening with XID before its SABM). Mirrors `RespondToXidCommand` +
//!   `HandleNoCachedSession`; hand-implemented in C# too, until upstream redraws
//!   figc5.1. The negotiated params stage on the cached context so the subsequent
//!   SABM's `Set Version 2.0` (which clears only `is_extended`) preserves the
//!   staged `srej_enabled` into the established link.
//!
//! `no_std` + `alloc`.

extern crate alloc;
use alloc::vec::Vec;

use crate::ax25::xid::{
    info_field, ClassesOfProcedures, HdlcOptionalFunctions, RejectMode, XidParameters,
};

use super::context::SessionContext;

/// Derive a sensible offered XID parameter set from a session context — our
/// current modulo / SREJ capability, window k, N1, T1, N2. We advertise our
/// capability (mod-128 + SREJ when the context is extended / SREJ-enabled) so the
/// §6.3.2 merge can revert to the lesser against the peer. Mirrors
/// `Ax25ManagementDataLink.DefaultOfferFor`.
pub fn default_offer_for(context: &SessionContext) -> XidParameters {
    XidParameters {
        classes_of_procedures: Some(if context.half_duplex {
            ClassesOfProcedures::HALF_DUPLEX_DEFAULT
        } else {
            ClassesOfProcedures::FULL_DUPLEX_CAPABLE
        }),
        hdlc_optional_functions: Some(HdlcOptionalFunctions {
            reject: if context.srej_enabled {
                RejectMode::SelectiveReject
            } else {
                RejectMode::ImplicitReject
            },
            modulo128: context.is_extended,
            // Advertise SREJ-multiframe alongside SREJ — LinBPQ's XID responder
            // REQUIRES the OPSREJMult bit or it rejects the whole XID and never
            // negotiates SREJ. Only meaningful when we are actually offering SREJ.
            srej_multiframe: context.srej_enabled,
            segmenter_reassembler: context.segmenter_reassembler_enabled,
        }),
        i_field_length_rx_bits: Some(XidParameters::octets_to_bits(context.n1)),
        window_size_rx: Some(context.k),
        ack_timer_millis: Some(context.t1v_ms),
        retries: Some(context.n2),
    }
}

/// Apply the §6.3.2 reverts-to merge of `offered` (what we sent / would send in an
/// XID command) and `response` (what the peer returned / offered) to `context`,
/// replacing the forced establishment defaults with the negotiated values. Each
/// parameter absent from *both* offers retains the context's current value
/// (§4.3.3.7 ¶1024). Mirrors `XidNegotiator.ApplyNegotiated`.
pub fn apply_negotiated(
    context: &mut SessionContext,
    offered: &XidParameters,
    response: &XidParameters,
) {
    // ─── HDLC Optional Functions (PI=3): reject scheme + modulo (§6.3.2 ¶1426) ──
    // The agreed value is the LOWER of the two on each axis: SREJ survives only if
    // BOTH offer it; mod-128 survives only if BOTH offer it. Absent from both →
    // the defaults (SREJ, mod-128) via HdlcOptionalFunctions::DEFAULT.
    let our_hdlc = offered
        .hdlc_optional_functions
        .unwrap_or(HdlcOptionalFunctions::DEFAULT);
    let their_hdlc = response
        .hdlc_optional_functions
        .unwrap_or(HdlcOptionalFunctions::DEFAULT);

    let agreed_selective_reject = our_hdlc.reject == RejectMode::SelectiveReject
        && their_hdlc.reject == RejectMode::SelectiveReject;
    let agreed_modulo128 = our_hdlc.modulo128 && their_hdlc.modulo128;
    // Segmenter/reassembler is a mutual-capability AND (§6.3.2 ¶1419).
    let agreed_segmenter = our_hdlc.segmenter_reassembler && their_hdlc.segmenter_reassembler;

    context.srej_enabled = agreed_selective_reject;
    context.implicit_reject = !agreed_selective_reject;
    context.is_extended = agreed_modulo128;
    context.segmenter_reassembler_enabled = agreed_segmenter;

    // ─── Classes of Procedures (PI=2): duplex (§6.3.2 ¶1424) ────────────────────
    // Reverts to half-duplex unless BOTH offer full-duplex.
    let our_cop = offered
        .classes_of_procedures
        .unwrap_or(ClassesOfProcedures::HALF_DUPLEX_DEFAULT);
    let their_cop = response
        .classes_of_procedures
        .unwrap_or(ClassesOfProcedures::HALF_DUPLEX_DEFAULT);
    let agreed_full_duplex = !our_cop.half_duplex && !their_cop.half_duplex;
    context.half_duplex = !agreed_full_duplex;

    // ─── Window k (PI=8) + N1 (PI=6): notification / min (§6.3.2 ¶1430 / ¶1428) ─
    // A notification of the receiver's capacity; our send is bounded by the peer's
    // advertised Rx, so take the min. Absent from both → retain current.
    if let Some(k) = min_present(offered.window_size_rx, response.window_size_rx) {
        context.k = k;
    }
    if let Some(n1) = min_present(
        offered.i_field_length_rx_octets(),
        response.i_field_length_rx_octets(),
    ) {
        context.n1 = n1;
    }

    // ─── T1 (PI=9) + N2 (PI=10): greater (§6.3.2 ¶1432 / ¶1434) ──────────────────
    // The more patient / safer choice on a slow/lossy link: both adopt the max.
    if let Some(t1ms) = max_present(offered.ack_timer_millis, response.ack_timer_millis) {
        context.t1v_ms = t1ms;
        context.srt_ms = t1ms / 2; // keep T1V ≈ 2·SRT (integer §3 port)
    }
    if let Some(n2) = max_present(offered.retries, response.retries) {
        context.n2 = n2;
    }
}

/// Install the complete AX.25 version-2.0 default parameter set per §6.3.2 ¶1 /
/// §1436 — used when a pre-v2.2 peer FRMRs our XID command. The FULL set, not
/// merely `is_extended = false`. Mirrors `XidNegotiator.ApplyVersion20Defaults`.
pub fn apply_version_20_defaults(context: &mut SessionContext) {
    context.half_duplex = true; // Set Half Duplex
    context.implicit_reject = true; // Set Implicit Reject
    context.srej_enabled = false; //   (REJ ⇒ no SREJ)
    context.is_extended = false; // Modulo = 8
    context.n1 = 256; // 2048 bits = 256 octets
    context.k = 7; // Window Size Receive = 7 (§1436, NOT the mod-8 XID default 4)
    context.t1v_ms = 3000; // Acknowledge Timer
    context.srt_ms = 1500; //   keep T1V == 2·SRT
    context.n2 = 10; // Retries
    context.segmenter_reassembler_enabled = false; // v2.2-only (§1621)
}

/// Handle an inbound XID *command* as the responder: merge the command's offered
/// parameters with our own offer per §6.3.2, apply the agreed values to `context`,
/// and return the *agreed* parameter set to echo back in the XID response. Placing
/// the agreed (post-merge) values guarantees both stations converge on the
/// identical reverts-to result. Mirrors `Ax25ManagementDataLink.RespondToXidCommand`.
pub fn respond_to_xid_command(
    context: &mut SessionContext,
    command: &XidParameters,
) -> XidParameters {
    let offered = default_offer_for(context);
    apply_negotiated(context, &offered, command);
    // Echo the agreed values so the initiator's merge (its offer vs our response)
    // lands on the identical result.
    default_offer_for(context)
}

/// The pre-session XID-command responder (mirrors `HandleNoCachedSession`'s XID
/// branch composed with `RespondToXidCommand`): seed `context` SREJ-capable so our
/// offer advertises SREJ, parse the command's offered parameters (strict; a
/// malformed / empty info ⇒ "no parameters offered", the merge falls through to the
/// §4.3.3.7 ¶1024 defaults), run the §6.3.2 merge into `context`, and return the
/// encoded XID *response* information field (an F=1 response carrying the agreed
/// values). The staged `srej_enabled` survives the subsequent SABM's `Set Version
/// 2.0` (which clears only `is_extended`), so the established link adopts SREJ when
/// both sides offered it.
pub fn respond_pre_session_xid(context: &mut SessionContext, command_info: &[u8]) -> Vec<u8> {
    // Seed SREJ-capable so default_offer_for advertises SREJ; the lesser-of merge
    // reverts this if the peer's offer lacked SREJ.
    context.srej_enabled = true;
    context.implicit_reject = false;

    let command = info_field::parse(command_info).unwrap_or_default();
    let agreed = respond_to_xid_command(context, &command);
    info_field::encode(&agreed)
}

/// Seed `context` SREJ-capable for an **initiator** pre-connect XID *probe* and
/// return the parameter set to advertise in the outbound XID command. This is the
/// offer step of the LinBPQ SREJ accommodation: on a mod-8 dial we send an XID
/// command *before* the SABM (BPQ only honours an XID that precedes the SABM), so
/// [`default_offer_for`] must read a SREJ-capable context to advertise SREJ + the
/// OPSREJMult bit BPQ requires. The caller encodes the returned parameters, puts them
/// on the wire as an XID *command*, and keeps the offer to merge against the peer's
/// response via [`apply_negotiated`]. Mirrors the offer step of
/// `Ax25Listener.NegotiateSrejBeforeConnectAsync` (`ctx.SrejEnabled = true;
/// ctx.ImplicitReject = false;` then `cached.Mdl.Negotiate()` advertising
/// `DefaultOfferFor(ctx)`).
pub fn begin_pre_connect_xid(context: &mut SessionContext) -> XidParameters {
    context.srej_enabled = true;
    context.implicit_reject = false;
    default_offer_for(context)
}

/// Revert `context` to go-back-N after a pre-connect XID probe went unanswered
/// (bounded-wait timeout / MDL give-up): we never put SREJ on the wire unilaterally,
/// so a silent peer degrades us cleanly to implicit reject before the plain SABM.
/// Mirrors the `if (!confirmed)` fallback of
/// `Ax25Listener.NegotiateSrejBeforeConnectAsync` (`ctx.SrejEnabled = false;
/// ctx.ImplicitReject = true;`). The merge on a *confirmed* response is
/// [`apply_negotiated`] instead — this is only the no-response leg.
pub fn revert_pre_connect_xid(context: &mut SessionContext) {
    context.srej_enabled = false;
    context.implicit_reject = true;
}

/// Lesser of two notification values, treating absence as "no constraint".
fn min_present(a: Option<u32>, b: Option<u32>) -> Option<u32> {
    match (a, b) {
        (None, other) | (other, None) => other,
        (Some(x), Some(y)) => Some(x.min(y)),
    }
}

/// Greater of two negotiated values, treating absence as "no preference".
fn max_present(a: Option<u32>, b: Option<u32>) -> Option<u32> {
    match (a, b) {
        (None, other) | (other, None) => other,
        (Some(x), Some(y)) => Some(x.max(y)),
    }
}

// ─── The management_data_link machine (figc5.1 Ready / figc5.2 Negotiating) ───
//
// Ports `Packet.Ax25.Session.Ax25ManagementDataLink`: the runtime driver for the
// generated MDL state pages [`ax25sdl::MANAGEMENT_DATA_LINK_READY`] /
// [`ax25sdl::MANAGEMENT_DATA_LINK_NEGOTIATING`] - the XID parameter-negotiation
// FSM of Appendix C5. It consumes the MDL-NEGOTIATE Request poke the data-link
// side emits after the UA on a v2.2 connect (figc4.6), and it also fronts the
// initiator pre-connect XID probe (the LinBPQ SREJ accommodation), which C#
// likewise runs through this same machine - "the same XID exchange the post-UA
// path runs, simply triggered before the SABM".
//
// Like the C#, the machine keeps its OWN retry bookkeeping (RC / NM201 / TM201)
// so it never disturbs the live data-link session's RC and T1/T2/T3; the
// negotiated parameters are applied to the REAL link [`SessionContext`] - that is
// the whole point of the exercise. The figc5.3-5.8 per-parameter "reverts-to"
// subroutines are collapsed in the SDL to a single `Apply Negotiated Parameters`
// placeholder; its runtime body is [`apply_negotiated`] (mirroring
// `XidNegotiator`), and the un-transcribed figc5.1 *responder* column stays the
// hand-implemented [`respond_pre_session_xid`] until upstream redraws figc5.1.

use ax25sdl::{
    Ax25ActionVerb, Ax25Event, Ax25Guard, StatePage, TransitionSpec,
    MANAGEMENT_DATA_LINK_NEGOTIATING, MANAGEMENT_DATA_LINK_READY,
};

use super::timer::{TimerId, TimerService};

/// TM201 duration - the management analogue of T1. §C5.3 gives no numeric
/// default; 3000 ms is the spec's T1 default, matching the C# dispatcher default
/// (deliberately NOT seeded from the link T1V, which establishment resets).
pub const TM201_MS: u32 = 3000;

/// The two MDL states (figc5.1 / figc5.2).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MdlState {
    /// figc5.1 - no XID command outstanding.
    Ready,
    /// figc5.2 - our XID command is on the wire, awaiting response / FRMR / TM201.
    Negotiating,
}

/// The runtime events posted into the MDL machine. Mirrors the C# event routing:
/// `Negotiate()` / `OnXidReceived` / `OnFrmrReceived` / the TM201 expiry.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MdlEvent {
    /// MDL-NEGOTIATE Request - open the XID exchange (from figc4.6's post-UA poke
    /// or a pre-connect probe).
    NegotiateRequest,
    /// The peer's XID *response* frame: its F bit + raw info field.
    XidResponseReceived {
        /// The response's F bit (the figc5.2 `F_eq_1` diamond).
        final_bit: bool,
        /// The raw XID info field (parsed by `Apply Negotiated Parameters`).
        info: Vec<u8>,
    },
    /// A FRMR answering our XID command - a pre-v2.2 peer; §6.3.2 ¶1 v2.0 fallback.
    FrmrReceived,
    /// TM201 expired with no response - retry the XID command or give up (error C).
    Tm201Expiry,
}

impl MdlEvent {
    /// The typed [`ax25sdl::Ax25Event`] this runtime event maps onto.
    fn to_sdl(&self) -> Ax25Event {
        match self {
            MdlEvent::NegotiateRequest => Ax25Event::MDLNEGOTIATERequest,
            MdlEvent::XidResponseReceived { .. } => Ax25Event::XIDResponseReceived,
            MdlEvent::FrmrReceived => Ax25Event::FRMRReceived,
            MdlEvent::Tm201Expiry => Ax25Event::TM201Expiry,
        }
    }
}

/// The Layer-3 signals the MDL machine raises (figc5.x, §5.1 / §C5.3).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MdlSignal {
    /// MDL-NEGOTIATE Confirm - negotiation complete (parameters applied, or the
    /// FRMR v2.0 fallback taken).
    NegotiateConfirm,
    /// MDL-ERROR Indicate - "B" unexpected XID response, "C" retry limit
    /// exceeded, "D" XID response without F=1.
    ErrorIndicate(&'static str),
}

/// What one [`MdlMachine::post_event`] drive produced: an XID command to put on
/// the wire (its encoded info field; the caller frames it as a U XID with P=1  -
/// error A, "XID command without P=1", implies P=1), and the Layer-3 signals.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct MdlOutcome {
    /// `Some(encoded info field)` when the drive ran the `XID_command` verb.
    pub xid_command: Option<Vec<u8>>,
    /// The MDL→L3 signals raised, in order.
    pub signals: Vec<MdlSignal>,
}

/// The table-driven MDL machine - see the section comment above.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MdlMachine {
    /// Current state (`Ready` / `Negotiating`).
    pub state: MdlState,
    /// RC - the XID retry count (§C5.3 Variables; distinct from the link RC).
    rc: u32,
    /// NM201 - maximum XID command retries. Defaults from the link N2 at
    /// construction, mirroring the C# `nm201 ?? linkContext.N2`.
    nm201: u32,
    /// The exact offer sent in our last XID command, kept so `Apply Negotiated
    /// Parameters` merges against precisely what we advertised.
    offered: Option<XidParameters>,
}

impl MdlMachine {
    /// A fresh machine in `Ready` with the given retry limit (typically the link
    /// context's `n2`).
    pub fn new(nm201: u32) -> Self {
        Self {
            state: MdlState::Ready,
            rc: 0,
            nm201,
            offered: None,
        }
    }

    /// True while a negotiation is in progress (awaiting the peer's XID response /
    /// FRMR / a TM201 retry). The manager uses this to decide whether an inbound
    /// XID/FRMR belongs to the MDL or the data-link session.
    pub fn is_negotiating(&self) -> bool {
        self.state == MdlState::Negotiating
    }

    /// The generated state page for the current state.
    fn page(&self) -> &'static StatePage {
        match self.state {
            MdlState::Ready => &MANAGEMENT_DATA_LINK_READY,
            MdlState::Negotiating => &MANAGEMENT_DATA_LINK_NEGOTIATING,
        }
    }

    /// Drive one event through the generated tables against the REAL data-link
    /// `link` context (negotiated parameters land there) and the shared timer
    /// service (TM201 is a distinct [`TimerId`], so it never collides with
    /// T1/T2/T3). An event with no matching transition is dropped - SDL semantics,
    /// same as the data-link driver (e.g. a FRMR arriving in `Ready`).
    pub fn post_event(
        &mut self,
        event: MdlEvent,
        link: &mut SessionContext,
        timers: &mut dyn TimerService,
    ) -> MdlOutcome {
        let mut outcome = MdlOutcome::default();
        let on = event.to_sdl();
        let Some(spec) = self
            .page()
            .transitions
            .iter()
            .find(|t| t.on == on && self.guards_hold(t, &event))
        else {
            return outcome;
        };
        // The MDL pages carry no loop_while ranges today; this walker does not
        // expand them, so fail loudly in tests if a regenerate ever adds one.
        debug_assert!(
            spec.loops.is_empty(),
            "MDL transition {} gained loops; teach MdlMachine to expand them",
            spec.id
        );
        for step in spec.actions {
            self.execute_verb(step.verb, &event, link, timers, &mut outcome);
        }
        self.state = match spec.next {
            "Ready" => MdlState::Ready,
            "Negotiating" => MdlState::Negotiating,
            other => unreachable!("unknown MDL next-state `{other}` in generated tables"),
        };
        outcome
    }

    /// Evaluate a transition's guard conjunction. The MDL pages use exactly two
    /// atoms; any other atom appearing here is a codegen/wiring surprise worth a
    /// loud stop (the same posture as the subroutine-name lookup).
    fn guards_hold(&self, spec: &TransitionSpec, event: &MdlEvent) -> bool {
        spec.guard.iter().all(|term| {
            let holds = match term.atom {
                // figc5.2's XID-response final-bit diamond.
                Ax25Guard::FEq1 => matches!(
                    event,
                    MdlEvent::XidResponseReceived {
                        final_bit: true,
                        ..
                    }
                ),
                // figc5.2's TM201-expiry retry-limit diamond.
                Ax25Guard::RCEqNM201 => self.rc == self.nm201,
                other => {
                    panic!("guard atom {other:?} is not part of the management_data_link machine")
                }
            };
            holds != term.negate
        })
    }

    /// Execute one MDL action verb. Exhaustive over the verbs the figc5.x pages
    /// carry; a data-link verb reaching here is a wiring bug (loud stop).
    fn execute_verb(
        &mut self,
        verb: Ax25ActionVerb,
        event: &MdlEvent,
        link: &mut SessionContext,
        timers: &mut dyn TimerService,
        outcome: &mut MdlOutcome,
    ) {
        match verb {
            Ax25ActionVerb::RCAssign0 => self.rc = 0,
            Ax25ActionVerb::RCAssignRCPlus1 => self.rc = self.rc.saturating_add(1),
            // Build + emit our XID command: derive the offer from the CURRENT
            // link context (so a context mutated since construction is
            // reflected - the C# `Offered` property), remember it verbatim for
            // the merge, and hand the encoded info field to the caller.
            Ax25ActionVerb::XIDCommand => {
                let offer = default_offer_for(link);
                outcome.xid_command = Some(info_field::encode(&offer));
                self.offered = Some(offer);
            }
            Ax25ActionVerb::StartTM201 => timers.arm(TimerId::Tm201, TM201_MS),
            Ax25ActionVerb::StopTM201 => timers.cancel(TimerId::Tm201),
            // The figc5.3-5.8 "reverts-to" placeholder: parse the peer's XID
            // response off the triggering event (malformed/empty ⇒ "no
            // parameters offered" → per-field spec defaults, §4.3.3.7 ¶1024)
            // and run the §6.3.2 merge into the REAL link context.
            Ax25ActionVerb::ApplyNegotiatedParameters => {
                let response = match event {
                    MdlEvent::XidResponseReceived { info, .. } => {
                        info_field::parse(info).unwrap_or_default()
                    }
                    _ => XidParameters::default(),
                };
                let offered = self.offered.unwrap_or_else(|| default_offer_for(link));
                apply_negotiated(link, &offered, &response);
            }
            // The figc5.2 FRMR path draws a single "Set Version 2.0" box meaning
            // the COMPLETE §1436 v2.0 default set on the real link context (not
            // merely is_extended = false).
            Ax25ActionVerb::SetVersion20 => apply_version_20_defaults(link),
            Ax25ActionVerb::MDLNEGOTIATEConfirm => {
                outcome.signals.push(MdlSignal::NegotiateConfirm)
            }
            Ax25ActionVerb::MDLERRORIndicateB => {
                outcome.signals.push(MdlSignal::ErrorIndicate("B"))
            }
            Ax25ActionVerb::MDLERRORIndicateC => {
                outcome.signals.push(MdlSignal::ErrorIndicate("C"))
            }
            Ax25ActionVerb::MDLERRORIndicateD => {
                outcome.signals.push(MdlSignal::ErrorIndicate("D"))
            }
            other => panic!("verb {other:?} is not part of the management_data_link machine"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdl::timer::MockTimerService;

    fn ctx() -> SessionContext {
        SessionContext::new()
    }

    /// figc5.1 t02: an XID *response* arriving in `Ready` (no command of ours
    /// outstanding) is error B - reported upward, state unchanged, link context
    /// untouched. (The manager's router only feeds responses to a Negotiating
    /// machine, so this is the machine-level contract test.)
    #[test]
    fn unexpected_xid_response_in_ready_is_error_b() {
        let mut m = MdlMachine::new(10);
        let mut c = ctx();
        let mut t = MockTimerService::new();

        let outcome = m.post_event(
            MdlEvent::XidResponseReceived {
                final_bit: true,
                info: Vec::new(),
            },
            &mut c,
            &mut t,
        );

        assert_eq!(outcome.signals, alloc::vec![MdlSignal::ErrorIndicate("B")]);
        assert!(outcome.xid_command.is_none());
        assert_eq!(m.state, MdlState::Ready);
        // An unexpected response never touches the link parameters.
        assert!(!c.srej_enabled);
        assert!(!c.is_extended);
        assert_eq!(c.n2, 10);
    }

    /// SDL semantics: an event with no transition in the current state is
    /// dropped - a FRMR in `Ready` (no XID command outstanding) does nothing.
    #[test]
    fn frmr_in_ready_is_dropped() {
        let mut m = MdlMachine::new(10);
        let mut c = ctx();
        let mut t = MockTimerService::new();

        let outcome = m.post_event(MdlEvent::FrmrReceived, &mut c, &mut t);
        assert_eq!(outcome, MdlOutcome::default());
        assert_eq!(m.state, MdlState::Ready);
    }

    fn hdlc(srej: bool, mod128: bool) -> HdlcOptionalFunctions {
        HdlcOptionalFunctions {
            reject: if srej {
                RejectMode::SelectiveReject
            } else {
                RejectMode::ImplicitReject
            },
            modulo128: mod128,
            srej_multiframe: false,
            segmenter_reassembler: false,
        }
    }

    // ─── §6.3.2 reverts-to merge (mirrors XidNegotiatorTests) ────────────────

    #[test]
    fn reject_scheme_is_the_lesser_of_the_two_offers() {
        for (ours, theirs, expect) in [
            (true, true, true),
            (true, false, false),
            (false, true, false),
            (false, false, false),
        ] {
            let mut c = ctx();
            let offered = XidParameters {
                hdlc_optional_functions: Some(hdlc(ours, true)),
                ..Default::default()
            };
            let response = XidParameters {
                hdlc_optional_functions: Some(hdlc(theirs, true)),
                ..Default::default()
            };
            apply_negotiated(&mut c, &offered, &response);
            assert_eq!(c.srej_enabled, expect);
            assert_eq!(c.implicit_reject, !expect);
        }
    }

    #[test]
    fn modulo_is_the_lesser_of_the_two_offers() {
        for (ours, theirs, expect) in [
            (true, true, true),
            (true, false, false),
            (false, true, false),
            (false, false, false),
        ] {
            let mut c = ctx();
            let offered = XidParameters {
                hdlc_optional_functions: Some(hdlc(true, ours)),
                ..Default::default()
            };
            let response = XidParameters {
                hdlc_optional_functions: Some(hdlc(true, theirs)),
                ..Default::default()
            };
            apply_negotiated(&mut c, &offered, &response);
            assert_eq!(c.is_extended, expect);
        }
    }

    #[test]
    fn segmenter_enabled_only_when_both_advertise_it() {
        let both_on = XidParameters {
            hdlc_optional_functions: Some(HdlcOptionalFunctions {
                segmenter_reassembler: true,
                ..hdlc(true, true)
            }),
            ..Default::default()
        };
        let one_off = XidParameters {
            hdlc_optional_functions: Some(hdlc(true, true)),
            ..Default::default()
        };
        let mut c = ctx();
        apply_negotiated(&mut c, &both_on, &both_on);
        assert!(c.segmenter_reassembler_enabled);
        let mut c2 = ctx();
        apply_negotiated(&mut c2, &both_on, &one_off);
        assert!(!c2.segmenter_reassembler_enabled);
    }

    #[test]
    fn window_k_is_the_min_and_n1_is_the_min() {
        let mut c = ctx();
        apply_negotiated(
            &mut c,
            &XidParameters {
                window_size_rx: Some(32),
                i_field_length_rx_bits: Some(XidParameters::octets_to_bits(256)),
                ..Default::default()
            },
            &XidParameters {
                window_size_rx: Some(10),
                i_field_length_rx_bits: Some(XidParameters::octets_to_bits(128)),
                ..Default::default()
            },
        );
        assert_eq!(c.k, 10);
        assert_eq!(c.n1, 128);
    }

    #[test]
    fn t1_and_n2_are_the_greater() {
        let mut c = ctx();
        apply_negotiated(
            &mut c,
            &XidParameters {
                ack_timer_millis: Some(1000),
                retries: Some(8),
                ..Default::default()
            },
            &XidParameters {
                ack_timer_millis: Some(4000),
                retries: Some(20),
                ..Default::default()
            },
        );
        assert_eq!(c.t1v_ms, 4000);
        assert_eq!(c.n2, 20);
    }

    #[test]
    fn absent_notification_fields_retain_current_values() {
        let mut c = ctx();
        c.k = 5;
        c.n1 = 200;
        c.n2 = 7;
        c.t1v_ms = 1234;
        let offered = XidParameters {
            hdlc_optional_functions: Some(HdlcOptionalFunctions::DEFAULT),
            ..Default::default()
        };
        apply_negotiated(&mut c, &offered, &offered);
        assert_eq!(c.k, 5);
        assert_eq!(c.n1, 200);
        assert_eq!(c.n2, 7);
        assert_eq!(c.t1v_ms, 1234);
    }

    #[test]
    fn absent_hdlc_selects_the_v22_defaults() {
        let mut c = ctx();
        let empty = XidParameters::default();
        apply_negotiated(&mut c, &empty, &empty);
        assert!(c.srej_enabled, "default selective reject");
        assert!(c.is_extended, "default modulo 128");
    }

    #[test]
    fn version20_defaults_install_the_complete_1436_set() {
        let mut c = ctx();
        c.is_extended = true;
        c.srej_enabled = true;
        c.segmenter_reassembler_enabled = true;
        c.k = 32;
        c.n1 = 512;
        c.n2 = 20;
        c.half_duplex = false;
        c.t1v_ms = 500;

        apply_version_20_defaults(&mut c);

        assert!(c.half_duplex);
        assert!(c.implicit_reject);
        assert!(!c.srej_enabled);
        assert!(!c.is_extended);
        assert_eq!(c.n1, 256);
        assert_eq!(c.k, 7);
        assert_eq!(c.t1v_ms, 3000);
        assert_eq!(c.n2, 10);
        assert!(!c.segmenter_reassembler_enabled);
    }

    // ─── Pre-session responder (mirrors Ax25ListenerPreSessionXidTests) ──────

    /// A mod-8 XID command offering SREJ (what a PDN interlink initiator sends
    /// before its SABM) is answered with an XID response that advertises SREJ, and
    /// the responder's context ends SREJ-enabled + mod-8.
    #[test]
    fn pre_session_xid_command_offering_srej_negotiates_srej() {
        let command = info_field::encode(&XidParameters {
            hdlc_optional_functions: Some(HdlcOptionalFunctions {
                reject: RejectMode::SelectiveReject,
                modulo128: false, // mod-8
                srej_multiframe: true,
                segmenter_reassembler: false,
            }),
            ..Default::default()
        });

        let mut c = ctx();
        let response_info = respond_pre_session_xid(&mut c, &command);

        // Context adopted SREJ (both offered it) and stayed mod-8 (peer offered mod-8).
        assert!(c.srej_enabled, "both sides offered SREJ ⇒ SREJ negotiated");
        assert!(!c.implicit_reject);
        assert!(!c.is_extended, "peer offered mod-8 ⇒ link is mod-8");

        // The response advertises SREJ.
        let p = info_field::parse(&response_info).expect("response info parses");
        assert_eq!(
            p.hdlc_optional_functions.unwrap().reject,
            RejectMode::SelectiveReject
        );
        assert!(!p.hdlc_optional_functions.unwrap().modulo128);
    }

    /// A peer that offers REJ (no SREJ) makes the lesser-of merge revert our seeded
    /// SREJ to go-back-N — we never end up SREJ-enabled unilaterally.
    #[test]
    fn pre_session_xid_command_offering_rej_reverts_to_go_back_n() {
        let command = info_field::encode(&XidParameters {
            hdlc_optional_functions: Some(hdlc(false, false)), // REJ, mod-8
            ..Default::default()
        });
        let mut c = ctx();
        let response_info = respond_pre_session_xid(&mut c, &command);
        assert!(!c.srej_enabled, "peer offered REJ ⇒ merge reverts SREJ off");
        assert!(c.implicit_reject);
        let p = info_field::parse(&response_info).unwrap();
        assert_eq!(
            p.hdlc_optional_functions.unwrap().reject,
            RejectMode::ImplicitReject
        );
    }

    /// An empty / malformed XID info field means "no parameters offered": the merge
    /// falls through to the §6.3.2 defaults (SREJ, mod-128) against our SREJ-capable
    /// seed — so we still answer with a well-formed XID response, and (our seeded
    /// SREJ meeting the SREJ default) end SREJ-enabled.
    #[test]
    fn pre_session_xid_command_with_empty_info_falls_to_defaults() {
        let mut c = ctx();
        let response_info = respond_pre_session_xid(&mut c, &[]);
        assert!(
            c.srej_enabled,
            "seeded SREJ meets the SREJ default ⇒ SREJ negotiated"
        );
        let p =
            info_field::parse(&response_info).expect("response is a well-formed XID info field");
        assert_eq!(
            p.hdlc_optional_functions.unwrap().reject,
            RejectMode::SelectiveReject
        );
    }

    // ─── Initiator pre-connect probe (mirrors NegotiateSrejBeforeConnectAsync) ─

    /// The offer step: `begin_pre_connect_xid` seeds the context SREJ-capable and
    /// returns a mod-8 offer advertising SREJ + the OPSREJMult bit BPQ requires.
    #[test]
    fn begin_pre_connect_xid_offers_srej_and_seeds_the_context() {
        let mut c = ctx();
        assert!(!c.srej_enabled, "starts go-back-N");
        let offer = begin_pre_connect_xid(&mut c);
        // Context is now SREJ-capable so the merge on a matching response keeps SREJ.
        assert!(c.srej_enabled);
        assert!(!c.implicit_reject);
        // The offer advertises SREJ, mod-8, and SREJ-multiframe (BPQ's OPSREJMult).
        let hdlc = offer
            .hdlc_optional_functions
            .expect("offer carries HDLC opts");
        assert_eq!(hdlc.reject, RejectMode::SelectiveReject);
        assert!(!hdlc.modulo128, "a mod-8 probe stays mod-8");
        assert!(hdlc.srej_multiframe, "OPSREJMult set — BPQ requires it");
    }

    /// The confirmed-response leg: our probe offer merged against a peer response
    /// that also offers SREJ lands SREJ-enabled + mod-8 (the mutual result).
    #[test]
    fn pre_connect_xid_response_offering_srej_negotiates_srej() {
        let mut c = ctx();
        let offer = begin_pre_connect_xid(&mut c);
        let response = XidParameters {
            hdlc_optional_functions: Some(HdlcOptionalFunctions {
                reject: RejectMode::SelectiveReject,
                modulo128: false,
                srej_multiframe: true,
                segmenter_reassembler: false,
            }),
            ..Default::default()
        };
        apply_negotiated(&mut c, &offer, &response);
        assert!(c.srej_enabled, "both offered SREJ ⇒ SREJ on the link");
        assert!(!c.implicit_reject);
        assert!(!c.is_extended, "a mod-8 probe never flips to mod-128");
    }

    /// The no-response leg: `revert_pre_connect_xid` undoes the seeded SREJ so a
    /// silent peer degrades to go-back-N — we never put SREJ on the wire alone.
    #[test]
    fn revert_pre_connect_xid_falls_back_to_go_back_n() {
        let mut c = ctx();
        let _ = begin_pre_connect_xid(&mut c);
        assert!(c.srej_enabled, "seeded on");
        revert_pre_connect_xid(&mut c);
        assert!(!c.srej_enabled, "reverted off for a silent peer");
        assert!(c.implicit_reject);
    }

    /// A peer that answers the probe but offers REJ makes the lesser-of merge revert
    /// our seeded SREJ — the confirmed-but-no-SREJ outcome (distinct from silence,
    /// but the resulting link parameters are the same go-back-N).
    #[test]
    fn pre_connect_xid_response_offering_rej_reverts_to_go_back_n() {
        let mut c = ctx();
        let offer = begin_pre_connect_xid(&mut c);
        let response = XidParameters {
            hdlc_optional_functions: Some(hdlc(false, false)), // REJ, mod-8
            ..Default::default()
        };
        apply_negotiated(&mut c, &offer, &response);
        assert!(!c.srej_enabled, "peer offered REJ ⇒ merge reverts SREJ off");
        assert!(c.implicit_reject);
    }
}
