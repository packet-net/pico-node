//! Named deviations from the SDL figures where a figure is a confirmed upstream
//! defect — ports `Packet.Ax25.Session.Ax25SessionQuirks`.
//!
//! Each flag gates one figc4.x errata fix that `m0lte/packet.net` ships **on by
//! default** because the figure as drawn is a confirmed spec defect (cross-checked
//! against direwolf/linbpq and raised as `packethacking/ax25spec` issues). The
//! ax25sdl tables stay faithful to the figure; the *runtime* applies the fix, so
//! the single-source-of-truth tables are unchanged. Set [`Quirks::strictly_faithful`]
//! to run the figures exactly as drawn (for conformance testing).
//!
//! See the linked memory notes / packet.net `Ax25SessionQuirks.cs` for each
//! errata's full rationale, the graphml citation, and the implementation
//! cross-reference. Defaults here mirror `Ax25SessionQuirks.Default`.

/// The set of spec-defect quirk toggles. `Default` = every fix on (spec-correct
/// behaviour); `strictly_faithful()` = every fix off (figures verbatim).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Quirks {
    /// figc4.5 draws the SREJ-received retransmit as the generic go-back-N push +
    /// `Invoke_Retransmission`, contradicting §4.3.2.4/figc4.4. On an SREJ trigger
    /// do single-frame selective retransmit instead (ax25spec#38, packet.net#234).
    pub srej_selective_retransmit: bool,
    /// figc4.4's out-of-sequence I_received path has no receive-window guard, so a
    /// duplicate behind V(R) provokes an endless out-of-window re-send (the SREJ
    /// livelock). OR the out-of-window condition into `reject_exception`'s
    /// discard-vs-reject switch (X.25 §2.4.6.4; ax25spec#40, packet.net#242).
    pub discard_out_of_window_i_frames: bool,
    /// `Select_T1_Value`'s SRT IIR self-amplifies on a timeout sample (no clean
    /// round-trip measurement), growing T1V unbounded. Karn's algorithm: skip the
    /// SRT update when T1 wasn't stopped by an ack (ax25spec#41, packet.net#241).
    pub karn_srt_sampling: bool,
    /// figc4.4's out-of-sequence I_received SREJ path does `N(r) := N(s)` before
    /// SREJ — requesting the frame that just arrived rather than the gap, so
    /// multi-frame SREJ recovery livelocks. Retarget the SREJ to V(R) (the next
    /// still-missing frame) (ax25spec#42, packet.net#246).
    pub srej_targets_gap: bool,
    /// figc4.4 gates DL-FLOW-OFF's set-own-receiver-busy/RNR on the
    /// own_receiver_busy=Yes branch, so a not-busy station can never enter busy.
    /// Invert the guard for the DL_FLOW_OFF_request trigger (§6.4.10, ax25spec#43).
    pub dl_flow_off_enters_busy: bool,
    /// figc4.2 routes the Disconnected DL-CONNECT request unconditionally to
    /// AwaitingConnection (mod-8 establishment) even when the link is mod-128, so a
    /// v2.2 connect parks in the mod-8 state and downgrades on T1 retry. When the
    /// link is extended at dispatch time, redirect to AwaitingV22Connection
    /// (figc4.6) (ax25spec#44, packet.net session ResolveNextState).
    pub mod128_connect_routes_to_v22: bool,
    /// figc4.6's FRMR handler draws `Establish Data Link` before `Set Version 2.0`,
    /// so the §975 v2.0 fallback re-establishes with SABME while still extended.
    /// Force version 2.0 up front for the AwaitingV22Connection FRMR_received
    /// transition (ax25spec#45, direwolf's pre-establish set_version_2_0).
    pub frmr_fallback_reestablishes_v20: bool,
    /// figc4.6's `DM received` handler tears the link down to Disconnected on the
    /// F=1 branch (§975 refusal) with no fallback, leaving `is_extended` stuck true.
    /// But a DM is precisely the signal that the peer can't do v2.2 (it doesn't
    /// recognise our SABME), so — like the FRMR fallback (#45) — it must degrade to
    /// v2.0/SABM, not fail. On a DM (either F-branch) in AwaitingV22Connection,
    /// substitute the `t14_frmr_received` v2.0 re-establish transition and force
    /// version 2.0 before the actions run (so Establish_Data_Link emits SABM). This
    /// is the XRouter-style DM-refusal degrade (ax25spec#48, packet.net Ax25Session
    /// ResolveDmDegradeMatch).
    pub dm_rejection_degrades_to_v20: bool,
    /// figc4.5's in-sequence I_received stored-frame drain loop draws
    /// `V(r) := V(r) - 1`, where the structurally-identical figc4.4 handler uses
    /// `+ 1`. The drain must ADVANCE V(R) past each delivered stored frame; rewrite
    /// the decrement to an increment (ax25spec#47, packet.net#247).
    pub timer_recovery_drain_advances_vr: bool,
}

impl Default for Quirks {
    /// Every fix on — spec-correct behaviour (mirrors `Ax25SessionQuirks.Default`).
    fn default() -> Self {
        Self {
            srej_selective_retransmit: true,
            discard_out_of_window_i_frames: true,
            karn_srt_sampling: true,
            srej_targets_gap: true,
            dl_flow_off_enters_busy: true,
            mod128_connect_routes_to_v22: true,
            frmr_fallback_reestablishes_v20: true,
            dm_rejection_degrades_to_v20: true,
            timer_recovery_drain_advances_vr: true,
        }
    }
}

impl Quirks {
    /// Every fix OFF — run the SDL figures exactly as drawn (conformance testing).
    pub fn strictly_faithful() -> Self {
        Self {
            srej_selective_retransmit: false,
            discard_out_of_window_i_frames: false,
            karn_srt_sampling: false,
            srej_targets_gap: false,
            dl_flow_off_enters_busy: false,
            mod128_connect_routes_to_v22: false,
            frmr_fallback_reestablishes_v20: false,
            dm_rejection_degrades_to_v20: false,
            timer_recovery_drain_advances_vr: false,
        }
    }
}
