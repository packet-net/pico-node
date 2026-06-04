//! Generic (modem-agnostic) classification of a decoded KISS frame into a typed
//! inbound event.
//!
//! Ports `Packet.Kiss.KissFrameClassifier` + `Packet.Kiss.KissInboundEvents`. C#
//! models the inbound shapes as a `KissInboundEvent` record hierarchy
//! (`Ax25FrameReceivedEvent` / `AckModeDataReceivedEvent` / `UnknownInboundEvent`,
//! plus modem-specific subclasses); Rust models them as the closed [`InboundEvent`]
//! enum. The NinoTNC overlay adds its own variants — but because Rust enums are
//! closed, the NinoTNC-specific shapes live in the same enum (gated nothing) rather
//! than as open subclassing. See [`crate::kiss::ninotnc::classify`].
//!
//! Recognises the shapes KISS itself defines, with no modem-specific knowledge:
//! - `Data` whose body parses as AX.25 → [`InboundEvent::Ax25`].
//! - ACKMODE *data* (command `0x0C`, payload length > 2) → [`InboundEvent::AckModeData`].
//! - everything else (incl. the 2-byte ACKMODE TX-completion echo, which is
//!   correlated by tag inside the modem driver, not surfaced as an event) →
//!   [`InboundEvent::Unknown`].

use super::ackmode;
use super::frame::{Command, Frame};
use crate::ax25;

/// A typed inbound KISS event. The `'a` borrows the decoded [`Frame`] so the
/// payload slices (`raw`, `ax25_payload`) don't allocate. The parsed [`ax25::Frame`]
/// in [`InboundEvent::Ax25`] is owned (the AX.25 codec is `alloc`-backed).
///
/// Mirrors the `KissInboundEvent` hierarchy: [`Ax25`](Self::Ax25) ←
/// `Ax25FrameReceivedEvent`, [`AckModeData`](Self::AckModeData) ←
/// `AckModeDataReceivedEvent`, [`Unknown`](Self::Unknown) ← `UnknownInboundEvent`.
/// NinoTNC overlays ([`crate::kiss::ninotnc`]) add the TX-Test variants.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum InboundEvent<'a> {
    /// A KISS `Data` frame whose body parsed as an AX.25 frame.
    Ax25 {
        /// The raw decoded KISS frame.
        raw: &'a Frame,
        /// The parsed AX.25 frame.
        ax25: ax25::Frame,
    },
    /// An inbound ACKMODE-Data frame: command `0x0C`, a 2-byte sequence tag, then
    /// an AX.25 body. Not the same as our own outbound ACKMODE's TX-completion echo
    /// (which the driver correlates by tag and surfaces as a receipt, not an event).
    AckModeData {
        /// The raw decoded KISS frame.
        raw: &'a Frame,
        /// The 16-bit sequence tag.
        sequence_tag: u16,
        /// The AX.25 payload bytes after the tag.
        ax25_payload: &'a [u8],
    },
    /// A frame the generic rules did not recognise. Includes the 2-byte ACKMODE
    /// echo (by design — see the module docs) and any non-Data command.
    Unknown {
        /// The raw decoded KISS frame.
        raw: &'a Frame,
    },
}

/// Classify `frame` with no modem-specific knowledge. Never fails — frames the
/// rules don't recognise become [`InboundEvent::Unknown`].
///
/// Mirrors `KissFrameClassifier.Classify`.
pub fn classify(frame: &Frame) -> InboundEvent<'_> {
    // ACKMODE-Data: command 0x0C with a 2-byte seq tag + AX.25 payload.
    if let Some((tag, ax25_payload)) = ackmode::try_parse_data_frame(frame) {
        return InboundEvent::AckModeData {
            raw: frame,
            sequence_tag: tag,
            ax25_payload,
        };
    }

    // KISS Data with an AX.25-shaped body.
    if frame.command == Command::Data {
        if let Ok(ax25) = ax25::Frame::decode(&frame.payload) {
            return InboundEvent::Ax25 { raw: frame, ax25 };
        }
    }

    InboundEvent::Unknown { raw: frame }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ax25::frame::{CONTROL_UI, PID_NO_LAYER3};
    use crate::ax25::{Address, Callsign};
    use alloc::vec;
    use alloc::vec::Vec;

    fn addr(s: &str, crh: bool) -> Address {
        Address {
            callsign: Callsign::parse(s).unwrap(),
            crh,
            extension: false,
        }
    }

    /// Build the on-wire body of a minimal UI frame: dest/src + control/PID/info.
    fn ui_bytes(dest: &str, src: &str, info: &[u8]) -> Vec<u8> {
        let frame = ax25::Frame {
            destination: addr(dest, true),
            source: addr(src, false),
            digipeaters: Vec::new(),
            control: CONTROL_UI,
            pid: Some(PID_NO_LAYER3),
            info: info.to_vec(),
        };
        frame.encode()
    }

    #[test]
    fn data_frame_with_valid_ax25_body_classifies_as_ax25() {
        let body = ui_bytes("CQ", "M0LTE-1", b"hello");
        let raw = Frame::new(0, Command::Data, body);
        match classify(&raw) {
            InboundEvent::Ax25 { ax25, .. } => {
                assert_eq!(ax25.source.callsign.base(), b"M0LTE");
                assert_eq!(ax25.source.callsign.ssid(), 1);
                assert_eq!(ax25.info, b"hello");
            }
            other => panic!("expected Ax25, got {other:?}"),
        }
    }

    #[test]
    fn ackmode_data_frame_classifies_as_ackmode_data() {
        let payload = vec![0xA5, 0xB6, 0x41, 0x42, 0x43];
        let raw = Frame::new(0, Command::AckMode, payload);
        match classify(&raw) {
            InboundEvent::AckModeData {
                sequence_tag,
                ax25_payload,
                ..
            } => {
                assert_eq!(sequence_tag, 0xA5B6);
                assert_eq!(ax25_payload, &[0x41, 0x42, 0x43]);
            }
            other => panic!("expected AckModeData, got {other:?}"),
        }
    }

    #[test]
    fn ackmode_tx_completion_echo_classifies_as_unknown() {
        // 2-byte payload echo is correlated by the driver, not surfaced as an event.
        let raw = Frame::new(0, Command::AckMode, vec![0x12, 0x34]);
        assert!(matches!(classify(&raw), InboundEvent::Unknown { .. }));
    }

    #[test]
    fn unrecognised_command_classifies_as_unknown() {
        let raw = Frame::new(0, Command::Poll, vec![]);
        assert!(matches!(classify(&raw), InboundEvent::Unknown { .. }));
    }

    #[test]
    fn data_with_garbage_body_classifies_as_unknown() {
        // 8 bytes — too short for an AX.25 header.
        let raw = Frame::new(0, Command::Data, vec![1, 2, 3, 4, 5, 6, 7, 8]);
        assert!(matches!(classify(&raw), InboundEvent::Unknown { .. }));
    }
}
