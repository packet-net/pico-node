//! Management Data-Link (MDL) XID negotiation — ports the substantive logic of
//! `Packet.Ax25.Session.XidNegotiator` + `Ax25ManagementDataLink` +
//! `Ax25Listener.HandleNoCachedSession`'s pre-session XID branch.
//!
//! The AX.25 v2.2 MDL (Appendix C5) is the XID parameter-negotiation FSM that
//! turns SREJ / segmentation / modulo / window / T1 / N2 from forced establishment
//! defaults into *negotiated* link parameters. pico ports the two pieces that
//! matter on-air today:
//!
//! - The **§6.3.2 reverts-to merge** ([`apply_negotiated`]) that turns our offer
//!   and the peer's XID into agreed link parameters, plus the §1436 version-2.0
//!   default set ([`apply_version_20_defaults`]) and the offer derivation
//!   ([`default_offer_for`]). These mirror `XidNegotiator`.
//! - The **pre-session XID *command* responder** ([`respond_pre_session_xid`]) —
//!   the un-transcribed figc5.1 responder path that answers an inbound XID command
//!   *before* a session exists (a PDN NET/ROM mod-8 interlink initiator opening
//!   with XID before its SABM). Mirrors `RespondToXidCommand` +
//!   `HandleNoCachedSession`. The manager wires this in on the no-cached-session
//!   path; the negotiated params stage on the cached context so the subsequent
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
    let agreed_segmenter =
        our_hdlc.segmenter_reassembler && their_hdlc.segmenter_reassembler;

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

#[cfg(test)]
mod tests {
    use super::*;

    fn ctx() -> SessionContext {
        SessionContext::new()
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
        assert!(c.srej_enabled, "seeded SREJ meets the SREJ default ⇒ SREJ negotiated");
        let p = info_field::parse(&response_info).expect("response is a well-formed XID info field");
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
        let hdlc = offer.hdlc_optional_functions.expect("offer carries HDLC opts");
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
