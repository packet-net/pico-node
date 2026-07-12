//! The NinoTNC CQBEEP remote air-test responder — frame builders.
//!
//! Ports `Packet.Kiss.NinoTnc.NinoTncCqBeep`. The firmware ships a remote test-tone
//! responder that is *disarmed* until the TNC transmits a TARPN status frame — a UI
//! frame whose info text starts `[TARPNstat`. [`build_arming_frame`] builds that
//! arming frame; send it through the TNC's own port to arm it (volatile — re-arm
//! after a reset).
//!
//! Once armed, receiving a UI frame addressed to `CQBEEP-N` makes the TNC key its
//! transmitter and send N seconds of 440 Hz tone (bench-verified 2026-07-02 on
//! firmware 3.41: N=7 measured 6.99 s). [`build_beep_request`] builds that request;
//! its `{N ` + stepping-ASCII info is the exact shape the front-panel TX-Test button
//! emits, so any receiver recognises it via
//! [`super::airtest::NinoTncAirTestFrame::try_recognise`] (and can count bit errors
//! against the known pattern). The tone is the remote half of a deviation/level
//! tuning loop: trigger a beep, meter the received audio (e.g. GETRSSI on the
//! listening TNC — see [`super::rssi`]), adjust, repeat.
//!
//! This is the **builder** the Rust node lacked (it had the [`super::airtest`]
//! *recogniser* already). The outbound-construction path stays strict: a request
//! with an out-of-range second count or counter, or arming text without the required
//! prefix, returns `None` rather than emitting a malformed frame.

use crate::ax25::frame::{CONTROL_UI, PID_NO_LAYER3};
use crate::ax25::{Address, Callsign, Frame};
use alloc::vec::Vec;

/// The destination callsign base that addresses the beep responder.
pub const RESPONDER_CALLSIGN_BASE: &[u8] = b"CQBEEP";

/// The info-text prefix the firmware's arming check looks for.
pub const TARPN_STATUS_PREFIX: &[u8] = b"[TARPNstat";

/// The number of pattern bytes in a TX-Test-shaped info field.
pub const PATTERN_LENGTH: usize = 50;

/// The default arming info text, `"[TARPNstat]"`.
pub const DEFAULT_ARMING_TEXT: &[u8] = b"[TARPNstat]";

/// The default destination address for an arming frame — `IDENT`, the fake
/// destination the firmware itself uses for status frames.
const DEFAULT_ARMING_DEST: &[u8] = b"IDENT";

/// Build a UI command frame: destination gets the command C-bit set, source clear;
/// no digipeaters; control `0x03` (UI), PID `0xF0` (no layer 3). Mirrors the wire
/// shape of C# `Ax25Frame.Ui`.
fn ui_frame(destination: Callsign, source: Callsign, info: Vec<u8>) -> Frame {
    Frame {
        destination: Address {
            callsign: destination,
            crh: true,
            extension: false,
        },
        source: Address {
            callsign: source,
            crh: false,
            extension: false,
        },
        digipeaters: Vec::new(),
        control: CONTROL_UI,
        pid: Some(PID_NO_LAYER3),
        info,
    }
}

/// Build the default arming frame: a UI frame to `IDENT` whose info text is
/// `[TARPNstat]`. Transmit it through the TNC to arm that TNC's CQBEEP responder
/// (volatile — re-arm after reset). Mirrors `NinoTncCqBeep.BuildArmingFrame` with
/// its defaults.
pub fn build_arming_frame(source: Callsign) -> Frame {
    let destination = Callsign::new(DEFAULT_ARMING_DEST, 0).expect("IDENT is a valid callsign");
    ui_frame(destination, source, DEFAULT_ARMING_TEXT.to_vec())
}

/// Build an arming frame with an explicit destination and info text. The firmware
/// keys on the info text, not the destination; `status_text` **must** start with
/// [`TARPN_STATUS_PREFIX`] or this returns `None` (the strict outbound contract —
/// we never emit an arming frame the firmware would ignore).
///
/// Mirrors `NinoTncCqBeep.BuildArmingFrame(source, destination, statusText)` (which
/// throws `ArgumentException` on a bad prefix; the Rust port returns `None`).
pub fn build_arming_frame_with(
    source: Callsign,
    destination: Callsign,
    status_text: &[u8],
) -> Option<Frame> {
    if !status_text.starts_with(TARPN_STATUS_PREFIX) {
        return None;
    }
    Some(ui_frame(destination, source, status_text.to_vec()))
}

/// Build a beep request: a UI frame addressed to `CQBEEP-N`, which makes any armed
/// NinoTNC that hears it transmit `seconds` seconds of 440 Hz tone. The info field
/// carries the deterministic `{N ` + 50-byte stepping-ASCII pattern the TX-Test
/// button uses, so receivers recognise it via
/// [`super::airtest::NinoTncAirTestFrame::try_recognise`].
///
/// Returns `None` if `seconds` is outside 1–15 (it is the destination SSID) or
/// `sequence_counter` is outside 0–9 (the per-request digit). Mirrors
/// `NinoTncCqBeep.BuildBeepRequest`.
pub fn build_beep_request(source: Callsign, seconds: u8, sequence_counter: u8) -> Option<Frame> {
    if !(1..=15).contains(&seconds) || sequence_counter > 9 {
        return None;
    }
    let destination = Callsign::new(RESPONDER_CALLSIGN_BASE, seconds)?;

    let mut info = Vec::with_capacity(3 + PATTERN_LENGTH);
    info.push(b'{');
    info.push(b'0' + sequence_counter);
    info.push(b' ');
    let mut next = 0x20u8.wrapping_add(sequence_counter);
    for _ in 0..PATTERN_LENGTH {
        info.push(next);
        next = next.wrapping_add(1);
    }

    Some(ui_frame(destination, source, info))
}

#[cfg(all(test, feature = "alloc"))]
mod tests {
    use super::*;
    use super::super::airtest::NinoTncAirTestFrame;

    fn call(s: &str) -> Callsign {
        Callsign::parse(s).unwrap()
    }

    #[test]
    fn arming_frame_has_the_tarpn_prefix_and_is_a_ui_frame() {
        let f = build_arming_frame(call("M0LTE"));
        assert!(f.is_ui());
        assert_eq!(f.destination.callsign.base(), b"IDENT");
        assert!(f.destination.crh, "command frame: dest C-bit set");
        assert!(!f.source.crh);
        assert_eq!(f.source.callsign.base(), b"M0LTE");
        assert!(f.info.starts_with(TARPN_STATUS_PREFIX));
        assert_eq!(f.info, DEFAULT_ARMING_TEXT);
    }

    #[test]
    fn arming_frame_with_rejects_text_without_the_prefix() {
        assert!(build_arming_frame_with(call("M0LTE"), call("IDENT"), b"hello").is_none());
        let ok = build_arming_frame_with(call("M0LTE"), call("TARPN"), b"[TARPNstat/gps]").unwrap();
        assert_eq!(ok.info, b"[TARPNstat/gps]");
        assert_eq!(ok.destination.callsign.base(), b"TARPN");
    }

    #[test]
    fn beep_request_builds_the_cqbeep_n_air_test_shape() {
        let f = build_beep_request(call("M0LTE"), 5, 1).unwrap();
        assert!(f.is_ui());
        assert_eq!(f.destination.callsign.base(), b"CQBEEP");
        assert_eq!(f.destination.callsign.ssid(), 5, "seconds = destination SSID");
        assert_eq!(f.info.len(), 3 + PATTERN_LENGTH);
        assert_eq!(&f.info[..3], b"{1 ");
        // Pattern begins at 0x20 + counter (0x21) and steps by 1.
        assert_eq!(f.info[3], 0x21);
        assert_eq!(f.info[4], 0x22);
        assert_eq!(*f.info.last().unwrap(), 0x21 + (PATTERN_LENGTH as u8 - 1));
    }

    #[test]
    fn beep_request_round_trips_through_the_air_test_recogniser() {
        // seconds = 5 → CQBEEP-5, which the airtest recogniser keys on.
        let built = build_beep_request(call("G7XYZ"), 5, 1).unwrap();
        // Encode → decode to prove it survives the wire, then recognise it.
        let wire = built.encode();
        let decoded = Frame::decode(&wire).unwrap();
        let air = NinoTncAirTestFrame::try_recognise(&decoded).unwrap();
        assert_eq!(air.learned_callsign.base(), b"G7XYZ");
        assert_eq!(air.sequence_counter, 1);
        assert_eq!(air.pattern.len(), PATTERN_LENGTH);
    }

    #[test]
    fn beep_request_counter_drives_the_pattern_start() {
        let f = build_beep_request(call("M0LTE"), 7, 3).unwrap();
        assert_eq!(&f.info[..3], b"{3 ");
        assert_eq!(f.info[3], 0x20 + 3);
    }

    #[test]
    fn out_of_range_seconds_and_counter_are_rejected() {
        assert!(build_beep_request(call("M0LTE"), 0, 1).is_none());
        assert!(build_beep_request(call("M0LTE"), 16, 1).is_none());
        assert!(build_beep_request(call("M0LTE"), 5, 10).is_none());
        // Boundaries are accepted.
        assert!(build_beep_request(call("M0LTE"), 1, 0).is_some());
        assert!(build_beep_request(call("M0LTE"), 15, 9).is_some());
    }
}
