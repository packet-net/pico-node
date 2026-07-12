//! Two-session wire harness — the cross-stack parity artifact.
//!
//! Mirrors the deterministic two-session harness `packet.net` / `ax25-ts` use: two
//! independent [`Session`]s (A and B) linked by carrying each one's *emitted wire
//! octets* — built by [`WireSink`], decoded by the real [`crate::ax25::Frame`]
//! codec, classified by [`classify_incoming`] — into the other as a posted
//! [`Event`]. Nothing is faked at the byte level: a frame A sends is the exact
//! frame B receives, so this exercises the runtime, the spec→wire bridge, AND the
//! codec together — the assertion is on the wire and on each side's converged
//! state, not on an internal queue.
//!
//! These are the host-side end-to-end proofs that the connected-mode link works
//! off the generated `ax25sdl` tables, the loop the research note (§3.1/§4.1) puts
//! ~all correctness in.

use super::*;
use crate::ax25::{Callsign, Frame};
use alloc::vec;
use alloc::vec::Vec;

/// One station in the harness: its session + a wire sink addressed peer-ward.
struct Station {
    session: Session,
    timers: MockTimerService,
    sink: WireSink,
}

impl Station {
    fn new(local: &str, remote: &str) -> Self {
        let local = Callsign::parse(local).unwrap();
        let remote = Callsign::parse(remote).unwrap();
        Station {
            session: Session::new(),
            timers: MockTimerService::new(),
            sink: WireSink::new(local, remote, Vec::new()),
        }
    }

    /// Post an event into this station, returning the wire frames it emitted during
    /// the dispatch (drained from the sink so the next call sees only new frames).
    fn post(&mut self, event: Event) -> Vec<Vec<u8>> {
        self.sink.sent.clear();
        self.session
            .post_event(event, &mut self.timers, &mut self.sink);
        core::mem::take(&mut self.sink.sent)
    }

    /// Drain the DL signals raised upward since the last check.
    fn take_upward(&mut self) -> Vec<DataLinkSignal> {
        core::mem::take(&mut self.sink.upward)
    }
}

/// Deliver every wire frame in `frames` into `dst`, posting the classified event,
/// and return all frames `dst` emits in response (concatenated).
fn deliver(dst: &mut Station, frames: &[Vec<u8>]) -> Vec<Vec<u8>> {
    let mut out = Vec::new();
    for bytes in frames {
        let frame = Frame::decode(bytes).expect("harness frame decodes");
        let event = classify_incoming(&frame).expect("harness frame classifies");
        out.extend(dst.post(event));
    }
    out
}

#[test]
fn two_sessions_complete_sabm_ua_handshake_over_the_wire() {
    let mut a = Station::new("M0LTE-1", "M0LTE-2");
    let mut b = Station::new("M0LTE-2", "M0LTE-1");

    // A initiates: DL-CONNECT ⇒ emits SABM on the wire.
    let from_a = a.post(Event::DlConnectRequest);
    assert_eq!(a.session.state, State::AwaitingConnection);
    assert_eq!(from_a.len(), 1);

    // The exact SABM octets arrive at B ⇒ B accepts, emits UA, enters Connected.
    let from_b = deliver(&mut b, &from_a);
    assert_eq!(b.session.state, State::Connected);
    assert!(b.take_upward().contains(&DataLinkSignal::ConnectIndication));
    assert_eq!(from_b.len(), 1);

    // B's UA arrives at A ⇒ A confirms, enters Connected.
    let from_a2 = deliver(&mut a, &from_b);
    assert_eq!(a.session.state, State::Connected);
    assert!(a.take_upward().contains(&DataLinkSignal::ConnectConfirm));
    assert!(from_a2.is_empty());
}

#[test]
fn two_sessions_exchange_an_i_frame_and_ack_over_the_wire() {
    let (mut a, mut b) = connected_pair();

    // A sends data ⇒ emits an I-frame (N(S)=0) on the wire.
    let from_a = a.post(Event::DlDataRequest(
        crate::ax25::PID_NO_LAYER3,
        vec![0x01, 0x02, 0x03],
    ));
    assert_eq!(a.session.context.vs, 1);
    assert_eq!(from_a.len(), 1);

    // B receives the I-frame ⇒ delivers data upward, advances V(R) to 1, and (on the
    // poll / ack path) may emit an RR. Whatever B emits, deliver it back to A.
    let from_b = deliver(&mut b, &from_a);
    assert_eq!(b.session.context.vr, 1);
    assert!(b.take_upward().iter().any(
        |u| matches!(u, DataLinkSignal::DataIndication(_, info) if info == &vec![0x01, 0x02, 0x03])
    ));

    // Any acknowledgement B produced, delivered to A, must advance A's V(A) so the
    // frame is considered acknowledged (or at minimum not break A's state).
    let _ = deliver(&mut a, &from_b);
    assert_eq!(a.session.state, State::Connected);
}

#[test]
fn two_sessions_tear_down_with_disc_ua_over_the_wire() {
    let (mut a, mut b) = connected_pair();

    // A disconnects ⇒ DISC on the wire ⇒ AwaitingRelease.
    let from_a = a.post(Event::DlDisconnectRequest);
    assert_eq!(a.session.state, State::AwaitingRelease);
    assert_eq!(from_a.len(), 1);

    // B receives DISC ⇒ UA + DisconnectIndication ⇒ Disconnected.
    let from_b = deliver(&mut b, &from_a);
    assert_eq!(b.session.state, State::Disconnected);
    assert!(b
        .take_upward()
        .contains(&DataLinkSignal::DisconnectIndication));

    // B's UA arrives at A ⇒ A confirms ⇒ Disconnected.
    let _ = deliver(&mut a, &from_b);
    assert_eq!(a.session.state, State::Disconnected);
    assert!(a.take_upward().contains(&DataLinkSignal::DisconnectConfirm));
}

#[test]
fn round_trip_frame_octets_decode_to_the_same_event_kind() {
    // The bridge's encode (WireSink::build_frame) and decode (classify_incoming)
    // are mutual inverses for each frame shape — the property the harness relies on.
    let sink = WireSink::new(
        Callsign::parse("A").unwrap(),
        Callsign::parse("B").unwrap(),
        Vec::new(),
    );
    let cases = [
        FrameSpec::Unnumbered {
            kind: UnnumberedKind::Sabm,
            is_command: true,
            pf: true,
            expedited: false,
        },
        FrameSpec::Supervisory {
            kind: SupervisoryKind::Rr,
            is_command: false,
            nr: 5,
            pf: false,
        },
        FrameSpec::Information {
            p: false,
            nr: 2,
            ns: 3,
            pid: crate::ax25::PID_NO_LAYER3,
            info: vec![9, 9],
        },
    ];
    for spec in cases {
        let frame = sink.build_frame(&spec);
        let bytes = frame.encode();
        let decoded = Frame::decode(&bytes).expect("decodes");
        let event = classify_incoming(&decoded).expect("classifies");
        // Spot-check the decoded sequence fields survive the round trip.
        match (&spec, &event) {
            (FrameSpec::Supervisory { nr, .. }, Event::RrReceived(f)) => assert_eq!(*nr, f.nr),
            (FrameSpec::Information { nr, ns, .. }, Event::IReceived(f)) => {
                assert_eq!(*nr, f.nr);
                assert_eq!(*ns, f.ns);
            }
            (FrameSpec::Unnumbered { .. }, Event::SabmReceived(_)) => {}
            other => panic!("unexpected round-trip pairing: {other:?}"),
        }
    }
}

/// A modulo-128 sink: `local ↔ remote` with the extended flag set, so I/S specs
/// encode the 2-octet control field.
fn extended_sink() -> WireSink {
    let mut sink = WireSink::new(
        Callsign::parse("M0LTE-1").unwrap(),
        Callsign::parse("M0LTE-2").unwrap(),
        Vec::new(),
    );
    sink.extended = true;
    sink
}

#[test]
fn extended_i_frame_encodes_and_classifies_with_7bit_seqs() {
    // N(S)=120, N(R)=65 — both beyond mod-8's 3-bit range. The whole bridge round
    // trip (encode_spec → decode_with_modulo → classify_incoming_modulo) must
    // carry them intact, which only the 2-octet extended control field can do.
    let sink = extended_sink();
    let spec = FrameSpec::Information {
        p: true,
        nr: 65,
        ns: 120,
        pid: crate::ax25::PID_NO_LAYER3,
        info: vec![0xDE, 0xAD],
    };
    let bytes = sink.encode_spec(&spec);
    let (frame, ext) = Frame::decode_with_modulo(&bytes, true).expect("decodes extended");
    assert!(ext.is_some(), "extended I frame carries a second control octet");
    let event = classify_incoming_modulo(&frame, ext).expect("classifies");
    match event {
        Event::IReceived(f) => {
            assert_eq!(f.ns, 120);
            assert_eq!(f.nr, 65);
            assert!(f.poll_final);
            assert_eq!(f.info, vec![0xDE, 0xAD]);
        }
        other => panic!("expected IReceived, got {other:?}"),
    }
}

#[test]
fn extended_supervisory_frames_encode_and_classify() {
    // Each S type (RR/RNR/REJ/SREJ) at N(R)=100 (> 7) — mod-128 SREJ falls out of
    // the same path with no separate codec.
    let sink = extended_sink();
    for kind in [
        SupervisoryKind::Rr,
        SupervisoryKind::Rnr,
        SupervisoryKind::Rej,
        SupervisoryKind::Srej,
    ] {
        let spec = FrameSpec::Supervisory {
            kind,
            is_command: false,
            nr: 100,
            pf: true,
        };
        let bytes = sink.encode_spec(&spec);
        let (frame, ext) = Frame::decode_with_modulo(&bytes, true).expect("decodes extended");
        assert!(ext.is_some(), "extended S frame carries a second control octet");
        let event = classify_incoming_modulo(&frame, ext).expect("classifies");
        // The classified event kind must match the S type we asked for — mod-128
        // SREJ included (it rides the same extended S path, no separate codec).
        let matches_kind = match kind {
            SupervisoryKind::Rr => matches!(event, Event::RrReceived(_)),
            SupervisoryKind::Rnr => matches!(event, Event::RnrReceived(_)),
            SupervisoryKind::Rej => matches!(event, Event::RejReceived(_)),
            SupervisoryKind::Srej => matches!(event, Event::SrejReceived(_)),
        };
        assert!(matches_kind, "wrong S-type for {kind:?}: {event:?}");
        let f = event.frame().expect("S frame has FrameInfo");
        assert_eq!(f.nr, 100, "N(R) survives the 7-bit extended field for {kind:?}");
        assert!(f.poll_final);
    }
}

#[test]
fn extended_sequence_wrap_127_to_0_survives_the_bridge() {
    // The 7-bit field must keep 127 and 0 distinct through the bridge (a mod-8
    // encode/decode would alias them). Round-trip both boundaries on an I frame.
    let sink = extended_sink();
    for seq in [0u8, 127u8] {
        let spec = FrameSpec::Information {
            p: false,
            nr: seq,
            ns: seq,
            pid: crate::ax25::PID_NO_LAYER3,
            info: vec![seq],
        };
        let bytes = sink.encode_spec(&spec);
        let (frame, ext) = Frame::decode_with_modulo(&bytes, true).expect("decodes");
        let event = classify_incoming_modulo(&frame, ext).expect("classifies");
        match event {
            Event::IReceived(f) => {
                assert_eq!(f.ns, seq, "N(S)={seq} survives");
                assert_eq!(f.nr, seq, "N(R)={seq} survives");
            }
            other => panic!("expected IReceived, got {other:?}"),
        }
    }
}

#[test]
fn mod8_sink_is_unchanged_by_the_extended_field() {
    // Regression guard: a default (extended = false) sink still emits 1-octet
    // control — encode_spec must be byte-identical to build_frame().encode().
    let sink = WireSink::new(
        Callsign::parse("A").unwrap(),
        Callsign::parse("B").unwrap(),
        Vec::new(),
    );
    assert!(!sink.extended);
    let spec = FrameSpec::Information {
        p: false,
        nr: 2,
        ns: 3,
        pid: crate::ax25::PID_NO_LAYER3,
        info: vec![9, 9],
    };
    assert_eq!(sink.encode_spec(&spec), sink.build_frame(&spec).encode());
}

/// Build two sessions already Connected back-to-back (post-handshake).
fn connected_pair() -> (Station, Station) {
    let mut a = Station::new("M0LTE-1", "M0LTE-2");
    let mut b = Station::new("M0LTE-2", "M0LTE-1");
    let from_a = a.post(Event::DlConnectRequest);
    let from_b = deliver(&mut b, &from_a);
    let _ = deliver(&mut a, &from_b);
    assert_eq!(a.session.state, State::Connected);
    assert_eq!(b.session.state, State::Connected);
    a.take_upward();
    b.take_upward();
    (a, b)
}
