//! The G8BPQ "ACKMODE" KISS extension (KISS command `0x0C`).
//!
//! Ports `Packet.Kiss.KissAckMode`. ACKMODE lets the host learn *when a frame is
//! actually keyed onto the air* (not merely accepted into the TNC's queue), which
//! matters for sizing T1 on slow modes where queue-acceptance is far from
//! transmit-completion.
//!
//! The host sends `FEND | (port<<4)|0xC | seqHi | seqLo | payload | FEND`; the TNC
//! echoes back `FEND | (port<<4)|0xC | seqHi | seqLo | FEND` (an exactly-2-byte
//! payload) when (and only when) the frame has been transmitted. The 2-byte tag is
//! an opaque token chosen by the host.
//!
//! This module is framing-neutral: it sits on top of [`Command::AckMode`] and a
//! decoded [`Frame`], and reuses the SLIP encoder/decoder for framing, the port
//! nibble, and FEND/FESC escapes. The build helper is `alloc`-gated (it returns a
//! `Vec`); the parse helpers are pure `core`.

use super::frame::{Command, Frame};

#[cfg(feature = "alloc")]
use super::encoder::encode;
#[cfg(feature = "alloc")]
use alloc::vec::Vec;

/// Build an ACKMODE outbound frame: command `0x0C` followed by the 2-byte
/// host-chosen sequence tag (big-endian) and the AX.25 payload bytes, SLIP-framed.
/// Returns `None` only if `port` is out of range (0–15). Requires `alloc`.
///
/// Mirrors `KissAckMode.BuildSendFrame`.
#[cfg(feature = "alloc")]
pub fn build_send_frame(port: u8, sequence_tag: u16, ax25_payload: &[u8]) -> Option<Vec<u8>> {
    let mut payload = Vec::with_capacity(ax25_payload.len() + 2);
    payload.push((sequence_tag >> 8) as u8);
    payload.push((sequence_tag & 0xFF) as u8);
    payload.extend_from_slice(ax25_payload);
    encode(port, Command::AckMode, &payload)
}

/// Build the ACKMODE payload (sequence tag + AX.25 bytes) into a caller-provided
/// buffer — the allocation-free path for the embedded transport. Returns the number
/// of payload bytes written (`ax25_payload.len() + 2`), or `None` if `dst` is too
/// small. The caller frames the result with [`super::encode_into`].
pub fn build_payload_into(dst: &mut [u8], sequence_tag: u16, ax25_payload: &[u8]) -> Option<usize> {
    let needed = ax25_payload.len() + 2;
    if dst.len() < needed {
        return None;
    }
    dst[0] = (sequence_tag >> 8) as u8;
    dst[1] = (sequence_tag & 0xFF) as u8;
    dst[2..needed].copy_from_slice(ax25_payload);
    Some(needed)
}

/// True if `frame` is the TNC's TX-completion echo for an ACKMODE send: command
/// `0x0C` with a payload of *exactly* 2 bytes (the sequence tag). Returns the
/// recovered 16-bit tag.
///
/// Mirrors `KissAckMode.TryParseAcknowledgement`.
pub fn try_parse_acknowledgement(frame: &Frame) -> Option<u16> {
    if frame.command != Command::AckMode || frame.payload.len() != 2 {
        return None;
    }
    Some(u16::from_be_bytes([frame.payload[0], frame.payload[1]]))
}

/// True if `frame` is an ACKMODE *data* frame — command `0x0C` with a payload of 2
/// sequence bytes followed by AX.25 bytes (length strictly greater than 2). Returns
/// `(sequence_tag, &ax25_payload)`. Single-port TNCs do not normally emit inbound
/// ACKMODE data, but multi-master / cross-link bridges can.
///
/// Mirrors `KissAckMode.TryParseDataFrame`.
pub fn try_parse_data_frame(frame: &Frame) -> Option<(u16, &[u8])> {
    if frame.command != Command::AckMode || frame.payload.len() <= 2 {
        return None;
    }
    let tag = u16::from_be_bytes([frame.payload[0], frame.payload[1]]);
    Some((tag, &frame.payload[2..]))
}

#[cfg(all(test, feature = "alloc"))]
mod tests {
    use super::*;
    use crate::kiss::frame::{FEND, FESC, TFEND, TFESC};
    use crate::kiss::Decoder;
    use alloc::vec;

    #[test]
    fn build_send_frame_has_ackmode_command_and_sequence_prefix() {
        // tag 0xA5B6, payload "AB" → FEND, 0x0C, 0xA5, 0xB6, 'A', 'B', FEND
        let wire = build_send_frame(0, 0xA5B6, &[0x41, 0x42]).unwrap();
        assert_eq!(wire, vec![FEND, 0x0C, 0xA5, 0xB6, 0x41, 0x42, FEND]);
    }

    #[test]
    fn build_send_frame_encodes_port_in_upper_nibble() {
        // port 5, ackmode → command byte (5<<4)|0x0C = 0x5C
        let wire = build_send_frame(5, 0x0001, &[]).unwrap();
        assert_eq!(wire, vec![FEND, 0x5C, 0x00, 0x01, FEND]);
    }

    #[test]
    fn build_send_frame_escapes_sequence_bytes_when_they_are_fend() {
        // seqHi = 0xC0 (FEND) → escape to FESC TFEND; seqLo = 0xDB (FESC) → FESC TFESC
        let wire = build_send_frame(0, 0xC0DB, &[]).unwrap();
        assert_eq!(wire, vec![FEND, 0x0C, FESC, TFEND, FESC, TFESC, FEND]);
    }

    #[test]
    fn try_parse_acknowledgement_recovers_the_tag_from_two_byte_payload() {
        let frame = Frame::new(0, Command::AckMode, vec![0x12, 0x34]);
        assert_eq!(try_parse_acknowledgement(&frame), Some(0x1234));
    }

    #[test]
    fn try_parse_acknowledgement_rejects_non_ackmode_commands() {
        let frame = Frame::new(0, Command::Data, vec![0x12, 0x34]);
        assert_eq!(try_parse_acknowledgement(&frame), None);
    }

    #[test]
    fn try_parse_acknowledgement_rejects_wrong_payload_length() {
        // 3-byte payload = a data frame (seq + 1 AX.25 byte), not an echo.
        let with_data = Frame::new(0, Command::AckMode, vec![0x12, 0x34, 0x99]);
        assert_eq!(try_parse_acknowledgement(&with_data), None);
        let empty = Frame::new(0, Command::AckMode, vec![]);
        assert_eq!(try_parse_acknowledgement(&empty), None);
    }

    #[test]
    fn try_parse_data_frame_splits_sequence_from_payload() {
        let frame = Frame::new(0, Command::AckMode, vec![0x12, 0x34, 0x41, 0x42, 0x43]);
        let (tag, data) = try_parse_data_frame(&frame).unwrap();
        assert_eq!(tag, 0x1234);
        assert_eq!(data, &[0x41, 0x42, 0x43]);
    }

    #[test]
    fn try_parse_data_frame_rejects_the_two_byte_echo() {
        let frame = Frame::new(0, Command::AckMode, vec![0x12, 0x34]);
        assert_eq!(try_parse_data_frame(&frame), None);
    }

    #[test]
    fn round_trip_send_frame_then_decode_recovers_tag_and_payload() {
        let payload = [0xA8, 0x8A, 0xA6, 0xC0, 0xDB, 0x03, 0xF0, 0x68, 0x69];
        let wire = build_send_frame(0, 0xBEEF, &payload).unwrap();
        let mut decoder = Decoder::new();
        let frames = decoder.push(&wire);
        assert_eq!(frames.len(), 1);
        let decoded = &frames[0];
        assert_eq!(decoded.command, Command::AckMode);
        let (tag, round_trip) = try_parse_data_frame(decoded).unwrap();
        assert_eq!(tag, 0xBEEF);
        assert_eq!(round_trip, &payload);
    }

    #[test]
    fn build_payload_into_matches_alloc_path() {
        let mut buf = [0u8; 16];
        let n = build_payload_into(&mut buf, 0xBEEF, &[0x41, 0x42, 0x43]).unwrap();
        assert_eq!(&buf[..n], &[0xBE, 0xEF, 0x41, 0x42, 0x43]);
    }

    #[test]
    fn build_payload_into_reports_too_small() {
        let mut buf = [0u8; 3];
        assert_eq!(build_payload_into(&mut buf, 0x1234, &[1, 2]), None);
    }
}
