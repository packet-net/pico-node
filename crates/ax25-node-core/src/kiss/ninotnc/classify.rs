//! NinoTNC-aware classification overlay.
//!
//! Ports `Packet.Kiss.NinoTnc.NinoTncFrameClassifier`. It runs the generic
//! [`crate::kiss::classify`] first, then *upgrades* the result when the frame
//! matches a NinoTNC-firmware-specific shape:
//!
//! 1. The synthetic host-side **TX-Test diagnostic** (`=FirmwareVr:` marker) — the
//!    KISS Data frame the firmware sends to its own host when the button is pressed.
//! 2. The numeric **status report** (`=II:` register markers) — the periodic
//!    diagnostic beacon, or a numeric GETALL reply.
//! 3. The **GETRSSI reply** (`RSSI:` on the 0xE0 reply command byte).
//! 4. The over-air **TX-Test UI frame** (`CQBEEP-5` + stepping-ASCII INFO) — the
//!    AX.25 frame *another* NinoTNC put on the air when its button was pressed.
//!
//! The dispatch order mirrors C# `NinoTncFrameClassifier.Classify` exactly.
//!
//! C# overlays via an open record hierarchy (a subclass before the event fires);
//! Rust enums are closed, so the overlay returns the dedicated
//! [`NinoTncInboundEvent`] enum: it carries the four NinoTNC variants plus a
//! [`NinoTncInboundEvent::Generic`] passthrough for everything the overlay doesn't
//! upgrade.

use super::airtest::NinoTncAirTestFrame;
use super::rssi::NinoTncRssiReading;
use super::status::NinoTncStatusFrame;
use super::txtest::NinoTncTxTestFrame;
use crate::kiss::classify::{self, InboundEvent};
use crate::kiss::frame::{Command, Frame};

/// A NinoTNC-aware inbound event: the two firmware-specific shapes the overlay
/// recognizes, plus a passthrough of the generic [`InboundEvent`] for everything
/// else.
///
/// Mirrors the union of `KissInboundEvent` + the two NinoTNC subclasses
/// (`NinoTncTxTestFrameReceivedEvent`, `NinoTncAirTestFrameReceivedEvent`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum NinoTncInboundEvent<'a> {
    /// The synthetic host-side TX-Test diagnostic frame (button pressed on *this*
    /// modem). Mirrors `NinoTncTxTestFrameReceivedEvent`.
    TxTestDiagnostic {
        /// The raw decoded KISS frame.
        raw: &'a Frame,
        /// The parsed diagnostic.
        diagnostic: NinoTncTxTestFrame,
    },
    /// The over-air TX-Test UI frame (button pressed on *another* modem, heard via
    /// ours). Mirrors `NinoTncAirTestFrameReceivedEvent`.
    AirTest {
        /// The raw decoded KISS frame.
        raw: &'a Frame,
        /// The recognized over-air test frame (pattern borrows the AX.25 info).
        air_test: NinoTncAirTestFrame<'a>,
    },
    /// The periodic numeric diagnostic-register report (or a numeric GETALL reply).
    /// Mirrors `NinoTncStatusFrameReceivedEvent`.
    StatusReport {
        /// The raw decoded KISS frame.
        raw: &'a Frame,
        /// The parsed status snapshot.
        status: NinoTncStatusFrame,
    },
    /// A GETRSSI reply — `RSSI:` ASCII on the 0xE0 reply command byte. Mirrors
    /// `NinoTncRssiReadingReceivedEvent`.
    RssiReading {
        /// The raw decoded KISS frame.
        raw: &'a Frame,
        /// The parsed RX-audio level reading.
        rssi: NinoTncRssiReading,
    },
    /// Anything the NinoTNC overlay does not upgrade — the generic classification.
    Generic(InboundEvent<'a>),
}

/// Classify `frame` with NinoTNC firmware awareness. Never fails.
///
/// Mirrors `NinoTncFrameClassifier.Classify`.
pub fn classify(frame: &Frame) -> NinoTncInboundEvent<'_> {
    let generic = classify::classify(frame);

    // 1) Synthetic host-side TX-Test diagnostic. The "=FirmwareVr:" marker is the
    //    authoritative signal; it can appear where the generic classifier produced
    //    Ax25 (incidental parse) or Unknown. Only Data frames carry it.
    if matches!(
        generic,
        InboundEvent::Ax25 { .. } | InboundEvent::Unknown { .. }
    ) && frame.command == Command::Data
    {
        if let Some(diag) = NinoTncTxTestFrame::try_parse_frame(frame) {
            return NinoTncInboundEvent::TxTestDiagnostic {
                raw: frame,
                diagnostic: diag,
            };
        }
    }

    // 2) Numeric =II: register report — the periodic status frame (fake UI header,
    //    KISS Data), or a numeric GETALL reply. Same generic-shape gate as (1).
    if matches!(
        generic,
        InboundEvent::Ax25 { .. } | InboundEvent::Unknown { .. }
    ) {
        if let Some(status) = NinoTncStatusFrame::try_parse_frame(frame) {
            return NinoTncInboundEvent::StatusReport { raw: frame, status };
        }
    }

    // 3) GETRSSI reply — "RSSI:" ASCII on the 0xE0 reply command byte. Only ever an
    //    Unknown to the generic classifier (the reply payload is not AX.25-shaped).
    if matches!(generic, InboundEvent::Unknown { .. }) {
        if let Some(rssi) = NinoTncRssiReading::try_parse_frame(frame) {
            return NinoTncInboundEvent::RssiReading { raw: frame, rssi };
        }
    }

    // 4) Over-air TX-Test UI frame — only when the generic classifier already gave
    //    us a parsed AX.25 frame. Re-parse from the raw payload so the recognized
    //    pattern can borrow a slice tied to `frame` (not the moved `generic.ax25`).
    if let InboundEvent::Ax25 { .. } = &generic {
        // SAFETY of borrow: re-decode from frame.payload to get a frame whose info
        // we can recognise; but to keep the returned pattern borrowing `frame` we
        // must recognise against a frame value that lives as long as `frame`. The
        // generic Ax25 already owns its parse; recognising against it and copying
        // the recognized scalar fields, while re-pointing the pattern into the raw
        // payload, keeps lifetimes sound — see `recognise_air_test`.
        if let Some(air) = recognise_air_test(frame) {
            return NinoTncInboundEvent::AirTest {
                raw: frame,
                air_test: air,
            };
        }
    }

    NinoTncInboundEvent::Generic(generic)
}

/// Recognise the over-air TX-Test shape and return a frame whose `pattern` borrows
/// the original KISS `frame.payload` (so the returned event borrows only `frame`).
///
/// The decoded AX.25 `info` is a copy, but the over-air pattern is a verbatim,
/// contiguous tail of the KISS Data payload (AX.25 info sits at the end of the frame
/// with no escaping at this layer), so we locate it by length and point into the raw
/// payload — avoiding returning a borrow of a temporary parse.
fn recognise_air_test(frame: &Frame) -> Option<NinoTncAirTestFrame<'_>> {
    use crate::ax25;
    let parsed = ax25::Frame::decode(&frame.payload).ok()?;
    let recognised = NinoTncAirTestFrame::try_recognise(&parsed)?;
    // The info field is the trailing `recognised.pattern.len()+3` bytes of the AX.25
    // body, which is the trailing slice of the KISS payload (Data command → payload
    // *is* the AX.25 body). Re-point the pattern into frame.payload.
    let pattern_len = recognised.pattern.len();
    let payload = &frame.payload;
    if payload.len() < pattern_len {
        return None;
    }
    let pattern = &payload[payload.len() - pattern_len..];
    Some(NinoTncAirTestFrame {
        learned_callsign: recognised.learned_callsign,
        sequence_counter: recognised.sequence_counter,
        pattern,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ax25::frame::{CONTROL_UI, PID_NO_LAYER3};
    use crate::ax25::{Address, Callsign};
    use crate::kiss::classify::InboundEvent;
    use alloc::vec;
    use alloc::vec::Vec;

    fn addr(call: &str, ssid: u8, crh: bool) -> Address {
        Address {
            callsign: Callsign::new(call.as_bytes(), ssid).unwrap(),
            crh,
            extension: false,
        }
    }

    fn ax25_bytes(dest: (&str, u8), src: (&str, u8), info: &[u8]) -> Vec<u8> {
        let frame = crate::ax25::Frame {
            destination: addr(dest.0, dest.1, true),
            source: addr(src.0, src.1, false),
            digipeaters: Vec::new(),
            control: CONTROL_UI,
            pid: Some(PID_NO_LAYER3),
            info: info.to_vec(),
        };
        frame.encode()
    }

    #[test]
    fn data_with_valid_ax25_body_classifies_as_generic_ax25() {
        let body = ax25_bytes(("CQ", 0), ("M0LTE", 1), b"hello");
        let raw = Frame::new(0, Command::Data, body);
        match classify(&raw) {
            NinoTncInboundEvent::Generic(InboundEvent::Ax25 { ax25, .. }) => {
                assert_eq!(ax25.source.callsign.base(), b"M0LTE");
            }
            other => panic!("expected Generic(Ax25), got {other:?}"),
        }
    }

    #[test]
    fn data_with_firmwarevr_marker_classifies_as_txtest_diagnostic() {
        // Marker after a non-AX.25 prefix; the classifier prefers the TX-Test shape.
        let mut payload = Vec::new();
        payload.push(b'x');
        payload.extend_from_slice(&[0x01, 0x02]);
        payload.extend_from_slice(b"prefix-garbage=FirmwareVr:3.44=BrdSwchMod:040F0002");
        let raw = Frame::new(0, Command::Data, payload);
        match classify(&raw) {
            NinoTncInboundEvent::TxTestDiagnostic { diagnostic, .. } => {
                assert_eq!(diagnostic.firmware_version_raw.as_str(), "3.44");
                assert_eq!(diagnostic.running_mode.unwrap().mode, 6);
            }
            other => panic!("expected TxTestDiagnostic, got {other:?}"),
        }
    }

    #[test]
    fn txtest_shape_wins_over_ax25_parse_when_both_match() {
        // A payload that decodes as AX.25 AND contains the marker → TX-Test wins.
        let body = ax25_bytes(
            ("CQ", 0),
            ("M0LTE", 1),
            b"=FirmwareVr:3.44=BrdSwchMod:040F0002",
        );
        let raw = Frame::new(0, Command::Data, body);
        assert!(matches!(
            classify(&raw),
            NinoTncInboundEvent::TxTestDiagnostic { .. }
        ));
    }

    #[test]
    fn ackmode_data_passes_through_as_generic() {
        let raw = Frame::new(0, Command::AckMode, vec![0xA5, 0xB6, 0x41, 0x42, 0x43]);
        match classify(&raw) {
            NinoTncInboundEvent::Generic(InboundEvent::AckModeData { sequence_tag, .. }) => {
                assert_eq!(sequence_tag, 0xA5B6);
            }
            other => panic!("expected Generic(AckModeData), got {other:?}"),
        }
    }

    #[test]
    fn ackmode_echo_passes_through_as_generic_unknown() {
        let raw = Frame::new(0, Command::AckMode, vec![0x12, 0x34]);
        assert!(matches!(
            classify(&raw),
            NinoTncInboundEvent::Generic(InboundEvent::Unknown { .. })
        ));
    }

    #[test]
    fn unrecognised_command_passes_through_as_generic_unknown() {
        let raw = Frame::new(0, Command::Poll, vec![]);
        assert!(matches!(
            classify(&raw),
            NinoTncInboundEvent::Generic(InboundEvent::Unknown { .. })
        ));
    }

    #[test]
    fn numeric_status_report_upgrades_to_status_report() {
        // A KISS Data frame carrying the =II: register report (fake UI header first).
        let mut payload: Vec<u8> = b"TNC>USB:".to_vec();
        payload.extend_from_slice(b"=00:3.44=02:0000000A=04:0000000F=06:00000002");
        let raw = Frame::new(0, Command::Data, payload);
        match classify(&raw) {
            NinoTncInboundEvent::StatusReport { status, .. } => {
                assert_eq!(status.firmware_version_raw.as_str(), "3.44");
                assert_eq!(status.uptime_ms, Some(0x0A));
                assert_eq!(status.running_mode.unwrap().mode, 6);
            }
            other => panic!("expected StatusReport, got {other:?}"),
        }
    }

    #[test]
    fn rssi_reply_upgrades_to_rssi_reading() {
        // Reply command byte 0xE0 = port 14 + Data nibble; payload "RSSI:-62.54".
        let raw = Frame::new(14, Command::Data, b"RSSI:-62.54".to_vec());
        match classify(&raw) {
            NinoTncInboundEvent::RssiReading { rssi, .. } => {
                assert_eq!(rssi.centi_db, -6254);
            }
            other => panic!("expected RssiReading, got {other:?}"),
        }
    }

    #[test]
    fn over_air_test_frame_upgrades_to_air_test() {
        // Build the over-air TX-Test INFO: "{1 " + 0x21.. stepping 50 bytes.
        let mut info = vec![b'{', b'1', b' '];
        info.extend((0x21u8..).take(50));
        let body = ax25_bytes(("CQBEEP", 5), ("M0LTE", 0), &info);
        let raw = Frame::new(0, Command::Data, body);
        match classify(&raw) {
            NinoTncInboundEvent::AirTest { air_test, .. } => {
                assert_eq!(air_test.learned_callsign.base(), b"M0LTE");
                assert_eq!(air_test.sequence_counter, 1);
                assert_eq!(air_test.pattern.len(), 50);
                assert!(air_test.pattern_as_ascii().starts_with("!\"#$%&'()*"));
            }
            other => panic!("expected AirTest, got {other:?}"),
        }
    }
}
