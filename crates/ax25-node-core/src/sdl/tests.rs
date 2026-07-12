//! Behavioral tests for the SDL connected-mode runtime, driven off the real
//! generated `ax25sdl` tables. These mirror the key `packet.net`
//! `Packet.Ax25.Tests` scenarios (link establishment, release, I-frame transfer,
//! the typed-event dispatch, the integerised timers, and the spec-defect quirks),
//! asserting on the wire (the emitted `FrameSpec`s + `DataLinkSignal`s) and on the
//! session state / context — exactly the host `cargo test` loop the research note
//! identifies as where ~all correctness lives.

use super::*;
use alloc::vec;
use alloc::vec::Vec;

/// A recording sink: captures every outbound signal so tests assert on the wire.
#[derive(Debug, Default)]
struct Recorder {
    frames: Vec<FrameSpec>,
    upward: Vec<DataLinkSignal>,
    link_mux: Vec<LinkMultiplexerSignal>,
    internal: Vec<InternalSignal>,
}

impl SessionSink for Recorder {
    fn send_frame(&mut self, spec: FrameSpec) {
        self.frames.push(spec);
    }
    fn send_upward(&mut self, signal: DataLinkSignal) {
        self.upward.push(signal);
    }
    fn send_link_mux(&mut self, signal: LinkMultiplexerSignal) {
        self.link_mux.push(signal);
    }
    fn send_internal(&mut self, signal: InternalSignal) {
        self.internal.push(signal);
    }
}

impl Recorder {
    fn unnumbered(&self) -> Vec<UnnumberedKind> {
        self.frames
            .iter()
            .filter_map(|f| match f {
                FrameSpec::Unnumbered { kind, .. } => Some(*kind),
                _ => None,
            })
            .collect()
    }
    fn i_frames(&self) -> Vec<(u8, u8, Vec<u8>)> {
        self.frames
            .iter()
            .filter_map(|f| match f {
                FrameSpec::Information { ns, nr, info, .. } => Some((*ns, *nr, info.clone())),
                _ => None,
            })
            .collect()
    }
}

/// A frame-info for a received frame with the given P/F + command bit.
fn rx(poll_final: bool, is_command: bool) -> FrameInfo {
    FrameInfo {
        poll_final,
        is_command,
        ..Default::default()
    }
}

/// An I-frame info with N(S)/N(R).
fn rx_i(ns: u8, nr: u8, poll_final: bool) -> FrameInfo {
    FrameInfo {
        ns,
        nr,
        poll_final,
        is_command: true,
        info: vec![0xAA, 0xBB],
        pid: Some(crate::ax25::PID_NO_LAYER3),
    }
}

/// An S-frame info (RR/RNR/REJ/SREJ) carrying N(R).
fn rx_s(nr: u8, poll_final: bool, is_command: bool) -> FrameInfo {
    FrameInfo {
        nr,
        poll_final,
        is_command,
        ..Default::default()
    }
}

// ─── Typed-event mapping ────────────────────────────────────────────────────

#[test]
fn event_maps_to_typed_sdl_event() {
    use ax25sdl::Ax25Event as Sdl;
    assert_eq!(Event::DlConnectRequest.to_sdl(), Sdl::DLCONNECTRequest);
    assert_eq!(
        Event::SabmReceived(rx(true, true)).to_sdl(),
        Sdl::SABMReceived
    );
    assert_eq!(Event::T1Expiry.to_sdl(), Sdl::T1Expiry);
    assert_eq!(
        Event::IFramePopsOffQueue(0xF0, vec![]).to_sdl(),
        Sdl::IFramePopsOffQueue
    );
}

// ─── Link establishment (figc4.1 → figc4.2) ─────────────────────────────────

#[test]
fn outbound_connect_sends_sabm_and_enters_awaiting_connection() {
    let mut s = Session::new();
    let mut t = MockTimerService::new();
    let mut r = Recorder::default();

    s.post_event(Event::DlConnectRequest, &mut t, &mut r);

    // figc4.1 t03: SRT:=default; T1V:=2*SRT; Establish_Data_Link (→ SABM); set L3.
    assert_eq!(s.state, State::AwaitingConnection);
    assert_eq!(r.unnumbered(), vec![UnnumberedKind::Sabm]);
    assert!(s.context.layer3_initiated);
    // Establish_Data_Link arms T1.
    assert!(t.is_running(TimerId::T1));
    // T1V derived from the initial SRT: 2 * 3000 = 6000 ms.
    assert_eq!(s.context.t1v_ms, 6000);
}

#[test]
fn connect_then_ua_confirms_and_enters_connected() {
    let mut s = Session::new();
    let mut t = MockTimerService::new();
    let mut r = Recorder::default();

    s.post_event(Event::DlConnectRequest, &mut t, &mut r);
    r.upward.clear();
    // Peer accepts: UA (response, F=1).
    s.post_event(Event::UaReceived(rx(true, false)), &mut t, &mut r);

    assert_eq!(s.state, State::Connected);
    assert!(r.upward.contains(&DataLinkSignal::ConnectConfirm));
    // Sequence variables reset on entry to Connected.
    assert_eq!((s.context.vs, s.context.va, s.context.vr), (0, 0, 0));
}

#[test]
fn inbound_sabm_accepted_sends_ua_and_connects() {
    let mut s = Session::new();
    let mut t = MockTimerService::new();
    let mut r = Recorder::default();

    // Inbound SABM (command, P=1), node accepts by default (able_to_establish).
    s.post_event(Event::SabmReceived(rx(true, true)), &mut t, &mut r);

    assert_eq!(s.state, State::Connected);
    assert_eq!(r.unnumbered(), vec![UnnumberedKind::Ua]);
    assert!(r.upward.contains(&DataLinkSignal::ConnectIndication));
    assert!(!s.context.is_extended); // SABM ⇒ mod-8 (Set Version 2.0)
}

#[test]
fn inbound_sabm_rejected_sends_dm_and_stays_disconnected() {
    let mut s = Session::new();
    s.context.accept_incoming = false; // able_to_establish = No
    let mut t = MockTimerService::new();
    let mut r = Recorder::default();

    s.post_event(Event::SabmReceived(rx(true, true)), &mut t, &mut r);

    assert_eq!(s.state, State::Disconnected);
    assert_eq!(r.unnumbered(), vec![UnnumberedKind::Dm]);
    assert!(r.upward.is_empty());
}

#[test]
fn inbound_sabme_accepted_connects_extended() {
    let mut s = Session::new();
    let mut t = MockTimerService::new();
    let mut r = Recorder::default();

    s.post_event(Event::SabmeReceived(rx(true, true)), &mut t, &mut r);

    assert_eq!(s.state, State::Connected);
    assert_eq!(r.unnumbered(), vec![UnnumberedKind::Ua]);
    assert!(s.context.is_extended); // SABME ⇒ mod-128 (Set Version 2.2)
}

// ─── Link release (figc4.3) ─────────────────────────────────────────────────

#[test]
fn disconnect_request_from_connected_sends_disc() {
    let mut s = connected_session();
    let mut t = MockTimerService::new();
    let mut r = Recorder::default();

    s.post_event(Event::DlDisconnectRequest, &mut t, &mut r);

    assert_eq!(s.state, State::AwaitingRelease);
    assert_eq!(r.unnumbered(), vec![UnnumberedKind::Disc]);
}

#[test]
fn disconnect_then_ua_confirms_disconnect() {
    let mut s = connected_session();
    let mut t = MockTimerService::new();
    let mut r = Recorder::default();

    s.post_event(Event::DlDisconnectRequest, &mut t, &mut r);
    r.upward.clear();
    s.post_event(Event::UaReceived(rx(true, false)), &mut t, &mut r);

    assert_eq!(s.state, State::Disconnected);
    assert!(r.upward.contains(&DataLinkSignal::DisconnectConfirm));
}

#[test]
fn inbound_disc_when_connected_sends_ua_and_disconnects() {
    let mut s = connected_session();
    let mut t = MockTimerService::new();
    let mut r = Recorder::default();

    s.post_event(Event::DiscReceived(rx(true, true)), &mut t, &mut r);

    assert_eq!(s.state, State::Disconnected);
    assert_eq!(r.unnumbered(), vec![UnnumberedKind::Ua]);
    assert!(r.upward.contains(&DataLinkSignal::DisconnectIndication));
}

// ─── Disconnected-state catch-alls (figc4.1) ────────────────────────────────

#[test]
fn disconnected_ui_data_request_sends_ui() {
    let mut s = Session::new();
    let mut t = MockTimerService::new();
    let mut r = Recorder::default();

    s.post_event(
        Event::DlUnitDataRequest(crate::ax25::PID_NO_LAYER3, vec![1, 2, 3]),
        &mut t,
        &mut r,
    );

    assert_eq!(s.state, State::Disconnected);
    assert!(matches!(
        r.frames.first(),
        Some(FrameSpec::Ui { info, .. }) if info == &vec![1, 2, 3]
    ));
}

#[test]
fn disconnected_disc_received_sends_dm() {
    let mut s = Session::new();
    let mut t = MockTimerService::new();
    let mut r = Recorder::default();

    s.post_event(Event::DiscReceived(rx(true, true)), &mut t, &mut r);

    assert_eq!(s.state, State::Disconnected);
    assert_eq!(r.unnumbered(), vec![UnnumberedKind::Dm]);
}

// ─── I-frame transfer + the drain loop (figc4.4) ────────────────────────────

#[test]
fn dl_data_request_when_connected_sends_i_frame_and_advances_vs() {
    let mut s = connected_session();
    let mut t = MockTimerService::new();
    let mut r = Recorder::default();

    s.post_event(
        Event::DlDataRequest(crate::ax25::PID_NO_LAYER3, vec![0xDE, 0xAD]),
        &mut t,
        &mut r,
    );

    // The frame went on the wire (queue drained synthesised I_frame_pops_off_queue).
    let i = s_first_i(&r);
    assert_eq!(i.0, 0); // N(S) = 0 (was V(S))
    assert_eq!(i.2, vec![0xDE, 0xAD]);
    // V(S) advanced past the sent frame.
    assert_eq!(s.context.vs, 1);
    // The frame is retained for retransmission.
    assert!(s.context.sent_i_frames.contains_key(&0));
    assert_eq!(s.state, State::Connected);
}

#[test]
fn window_full_buffers_i_frames_until_acked() {
    let mut s = connected_session();
    s.context.k = 2; // tiny window
    let mut t = MockTimerService::new();
    let mut r = Recorder::default();

    // Three sends; only k=2 fit the window, the third stays queued.
    for n in 0..3u8 {
        s.post_event(
            Event::DlDataRequest(crate::ax25::PID_NO_LAYER3, vec![n]),
            &mut t,
            &mut r,
        );
    }
    assert_eq!(r.i_frames().len(), 2);
    assert_eq!(s.context.vs, 2);
    assert_eq!(s.context.i_frame_queue.len(), 1);

    // Peer acks both (RR, N(R)=2). The window reopens and the third drains.
    s.post_event(Event::RrReceived(rx_s(2, false, false)), &mut t, &mut r);
    assert_eq!(s.context.va, 2);
    assert_eq!(r.i_frames().len(), 3); // the buffered third went out
    assert_eq!(s.context.i_frame_queue.len(), 0);
}

#[test]
fn inbound_in_sequence_i_frame_delivers_data_and_advances_vr() {
    let mut s = connected_session();
    let mut t = MockTimerService::new();
    let mut r = Recorder::default();

    // In-sequence I-frame: N(S) = V(R) = 0.
    s.post_event(Event::IReceived(rx_i(0, 0, false)), &mut t, &mut r);

    assert!(r
        .upward
        .iter()
        .any(|u| matches!(u, DataLinkSignal::DataIndication(_, _))));
    assert_eq!(s.context.vr, 1); // V(R) advanced
}

#[test]
fn peer_rnr_sets_busy_and_blocks_drain() {
    let mut s = connected_session();
    let mut t = MockTimerService::new();
    let mut r = Recorder::default();

    // Peer RNR — sets peer_receiver_busy.
    s.post_event(Event::RnrReceived(rx_s(0, false, false)), &mut t, &mut r);
    assert!(s.context.peer_receiver_busy);

    // A data request now buffers (no I-frame on the wire while peer busy).
    s.post_event(
        Event::DlDataRequest(crate::ax25::PID_NO_LAYER3, vec![9]),
        &mut t,
        &mut r,
    );
    assert_eq!(r.i_frames().len(), 0);
    assert_eq!(s.context.i_frame_queue.len(), 1);
}

// ─── Integer timer math (research §3) ───────────────────────────────────────

#[test]
fn srt_iir_update_is_integer_and_karn_skips_timeout_sample() {
    // Clean measurement (remaining > 0): SRT' = 7*srt/8 + sample/8.
    // srt=4000, t1v=6000, remaining=2000 ⇒ sample=4000 ⇒ 3500 + 500 = 4000.
    assert_eq!(timer::srt_iir_update(4000, 6000, 2000, true), 4000);

    // Timeout sample (remaining = 0) with Karn ON ⇒ unchanged (skip the update).
    assert_eq!(timer::srt_iir_update(4000, 6000, 0, true), 4000);

    // Timeout sample with Karn OFF ⇒ degenerate update SRT' = 3500 + 6000/8 = 4250
    // (the self-amplifying behaviour Karn suppresses).
    assert_eq!(timer::srt_iir_update(4000, 6000, 0, false), 4250);
}

#[test]
fn next_t1_rc_backoff_is_integer() {
    // RC*250 + SRT*2: rc=3, srt=3000 ⇒ 750 + 6000 = 6750.
    assert_eq!(timer::next_t1_rc_backoff_ms(3, 3000), 6750);
}

#[test]
fn t1v_assign_2x_srt_uses_integer_doubling() {
    let mut s = Session::new();
    let mut t = MockTimerService::new();
    let mut r = Recorder::default();
    s.context.srt_ms = 2500;
    // Drive the connect path which runs SRT:=default; T1V:=2*SRT — but first force a
    // custom SRT via a direct dispatcher exercise: a DL_CONNECT resets SRT to the
    // initial default (3000), then T1V=6000. Assert the integer doubling.
    s.post_event(Event::DlConnectRequest, &mut t, &mut r);
    assert_eq!(s.context.srt_ms, 3000);
    assert_eq!(s.context.t1v_ms, 6000);
}

// ─── Quirk: #44 mod-128 connect routes to AwaitingV22Connection ─────────────

#[test]
fn extended_connect_routes_to_v22_with_quirk_on() {
    let mut s = Session::new();
    s.context.is_extended = true; // prefer mod-128
    let mut t = MockTimerService::new();
    let mut r = Recorder::default();

    s.post_event(Event::DlConnectRequest, &mut t, &mut r);

    // #44: instead of figc4.2's AwaitingConnection, the extended connect lands in
    // figc4.6's AwaitingV22Connection (and Establish_Data_Link sends SABME).
    assert_eq!(s.state, State::AwaitingV22Connection);
    assert_eq!(r.unnumbered(), vec![UnnumberedKind::Sabme]);
}

#[test]
fn extended_connect_stays_mod8_route_with_quirk_off() {
    let mut s = Session::new();
    s.context.is_extended = true;
    s.context.quirks = Quirks::strictly_faithful();
    let mut t = MockTimerService::new();
    let mut r = Recorder::default();

    s.post_event(Event::DlConnectRequest, &mut t, &mut r);

    // Strictly faithful to figc4.2: routes to AwaitingConnection regardless.
    assert_eq!(s.state, State::AwaitingConnection);
}

// ─── Quirk: #48 DM refusal degrades an extended connect to v2.0/SABM ─────────

#[test]
fn extended_dm_refusal_degrades_to_v20_with_quirk_on() {
    // Initiator prefers v2.2: an extended DL-CONNECT routes to AwaitingV22Connection
    // (#44) and Establish_Data_Link sends SABME.
    let mut s = Session::new();
    s.context.is_extended = true;
    let mut t = MockTimerService::new();
    let mut r = Recorder::default();

    s.post_event(Event::DlConnectRequest, &mut t, &mut r);
    assert_eq!(s.state, State::AwaitingV22Connection);
    assert_eq!(r.unnumbered(), vec![UnnumberedKind::Sabme]);
    r.frames.clear();

    // XRouter-style refusal: DM(F=1) answering our polled SABME. With #48 on this
    // degrades to v2.0 (re-establish via SABM) instead of tearing down.
    s.post_event(Event::DmReceived(rx(true, false)), &mut t, &mut r);

    // Substituted t14_frmr_received: → AwaitingConnection, mod-8, SABM on the wire.
    assert_eq!(s.state, State::AwaitingConnection);
    assert!(!s.context.is_extended); // forced to v2.0 so Establish emits SABM
    assert_eq!(r.unnumbered(), vec![UnnumberedKind::Sabm]);
}

#[test]
fn extended_dm_refusal_tears_down_with_quirk_off() {
    let mut s = Session::new();
    s.context.is_extended = true;
    s.context.layer3_initiated = true;
    s.context.quirks = Quirks::strictly_faithful();
    // Park directly in the extended-establishment state (with #44 off an extended
    // connect wouldn't route here; the DM handling is what's under test).
    s.state = State::AwaitingV22Connection;
    let mut t = MockTimerService::new();
    let mut r = Recorder::default();

    s.post_event(Event::DmReceived(rx(true, false)), &mut t, &mut r);

    // figc4.6 t11_dm_received_yes as drawn (F=1): §975 refusal → Disconnected, with
    // is_extended left stuck true and no SABM re-establish (the defect #48 fixes).
    assert_eq!(s.state, State::Disconnected);
    assert!(s.context.is_extended);
    assert!(r.unnumbered().is_empty());
}

// ─── Quirk: #9 ack progress resets RC (survives a working link) ─────────────

#[test]
fn ack_progress_resets_rc_survives_working_link_with_quirk_on() {
    // A bulk transfer living in Timer Recovery: RC has ratcheted to N2 but the peer
    // just acked NEW data (V(A) advanced), so the link is alive. With #9 on the next
    // T1 expiry clamps RC below N2 and keeps recovering instead of dying.
    let mut s = Session::in_state(State::TimerRecovery);
    s.context.reset_state();
    s.context.n2 = 2;
    s.context.k = 7;
    s.context.vs = 2; // two I-frames outstanding; V(A)=0, so V(S) != V(A)
    let mut t = MockTimerService::new();
    let mut r = Recorder::default();

    // Peer acks frame 0 (RR response, F=0, N(R)=1): V(A) advances 0→1 = progress.
    s.post_event(Event::RrReceived(rx_s(1, false, false)), &mut t, &mut r);
    assert_eq!(s.context.va, 1);
    assert_eq!(s.state, State::TimerRecovery);

    // Prior T1 hiccups pushed RC to the ceiling.
    s.context.rc = s.context.n2;
    r.upward.clear();

    // Next T1 expiry: progress was recorded, so RC is clamped below N2 → the
    // t21_t1_expiry_no retransmit branch fires, NOT the t21_t1_expiry_yes_no death.
    s.post_event(Event::T1Expiry, &mut t, &mut r);
    assert_eq!(s.state, State::TimerRecovery);
    assert!(!r.upward.contains(&DataLinkSignal::DisconnectIndication));
}

#[test]
fn rc_ratchets_and_kills_working_link_with_quirk_off() {
    let mut s = Session::in_state(State::TimerRecovery);
    s.context.reset_state();
    s.context.quirks = Quirks::strictly_faithful();
    s.context.n2 = 2;
    s.context.k = 7;
    s.context.vs = 2;
    let mut t = MockTimerService::new();
    let mut r = Recorder::default();

    // Same forward progress...
    s.post_event(Event::RrReceived(rx_s(1, false, false)), &mut t, &mut r);
    assert_eq!(s.context.va, 1);
    s.context.rc = s.context.n2;
    r.upward.clear();

    // ...but with #9 off RC stays at N2, so the T1 expiry declares the (still-alive)
    // link dead — the false-N2-death the quirk fixes.
    s.post_event(Event::T1Expiry, &mut t, &mut r);
    assert_eq!(s.state, State::Disconnected);
    assert!(r.upward.contains(&DataLinkSignal::DisconnectIndication));
}

// ─── Quirk: #13 clamp SREJ window to modulus/2 ──────────────────────────────

#[test]
fn srej_window_clamped_to_half_modulus_with_quirk_on() {
    let mut s = connected_session();
    s.context.k = 7; // above modulus/2 (=4 for mod-8) — unsafe for Selective Repeat
    s.context.srej_enabled = true;
    let mut t = MockTimerService::new();
    let mut r = Recorder::default();

    for n in 0..7u8 {
        s.post_event(
            Event::DlDataRequest(crate::ax25::PID_NO_LAYER3, vec![n]),
            &mut t,
            &mut r,
        );
    }

    // Effective window capped at modulus/2 = 4: only 4 frames go out, 3 stay queued,
    // so two in-flight frames can never share an N(S) (the ring-wrap corruption guard).
    assert_eq!(s.context.effective_window(), 4);
    assert_eq!(r.i_frames().len(), 4);
    assert_eq!(s.context.vs, 4);
    assert_eq!(s.context.i_frame_queue.len(), 3);
}

#[test]
fn srej_window_uncapped_with_quirk_off() {
    let mut s = connected_session();
    s.context.k = 7;
    s.context.srej_enabled = true;
    s.context.quirks = Quirks::strictly_faithful();
    let mut t = MockTimerService::new();
    let mut r = Recorder::default();

    for n in 0..7u8 {
        s.post_event(
            Event::DlDataRequest(crate::ax25::PID_NO_LAYER3, vec![n]),
            &mut t,
            &mut r,
        );
    }

    // Figure-literal: the full k=7 window is used — all 7 on the wire (the SREJ
    // ring-wrap corruption the quirk guards against, for conformance study).
    assert_eq!(s.context.effective_window(), 7);
    assert_eq!(r.i_frames().len(), 7);
    assert_eq!(s.context.vs, 7);
    assert_eq!(s.context.i_frame_queue.len(), 0);
}

#[test]
fn go_back_n_window_not_capped_even_with_quirk_on() {
    // The clamp gates on srej_enabled: a go-back-N link (SREJ off) buffers no
    // out-of-order frames and tolerates k up to modulus−1, so it is never capped —
    // even with the quirk on (default).
    let mut s = connected_session();
    s.context.k = 7;
    s.context.srej_enabled = false;
    assert!(s.context.quirks.clamp_srej_window_to_half_modulus);
    assert_eq!(s.context.effective_window(), 7);
}

// ─── Unhandled events are dropped (SDL semantics) ───────────────────────────

#[test]
fn unhandled_event_leaves_state_unchanged() {
    let mut s = Session::new();
    let mut t = MockTimerService::new();
    let mut r = Recorder::default();

    // A UA arriving in Disconnected has a transition (DL-ERROR C/D) — use instead a
    // timer expiry, which Disconnected does not handle at all.
    s.post_event(Event::T1Expiry, &mut t, &mut r);
    assert_eq!(s.state, State::Disconnected);
    assert!(r.frames.is_empty());
}

#[test]
fn disconnected_ua_received_raises_dl_error_cd() {
    let mut s = Session::new();
    let mut t = MockTimerService::new();
    let mut r = Recorder::default();

    s.post_event(Event::UaReceived(rx(true, false)), &mut t, &mut r);
    assert_eq!(s.state, State::Disconnected);
    assert_eq!(r.upward, vec![DataLinkSignal::ErrorIndication("C_D")]);
}

// ─── Helpers ────────────────────────────────────────────────────────────────

/// A session already in Connected (mod-8), as if a SABM/UA handshake completed.
fn connected_session() -> Session {
    let mut s = Session::in_state(State::Connected);
    s.context.reset_state();
    s
}

fn s_first_i(r: &Recorder) -> (u8, u8, Vec<u8>) {
    r.i_frames()
        .into_iter()
        .next()
        .expect("expected an I-frame")
}
