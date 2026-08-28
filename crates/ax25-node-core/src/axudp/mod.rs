//! AXUDP framing helpers — AX.25-over-UDP for node↔node connectivity over WiFi.
//!
//! Ports the framing logic of `Packet.Axudp.AxudpSocket`. AXUDP is the simplest
//! BPQ-compatible encapsulation: **the UDP datagram payload is the AX.25 frame
//! body followed by the CRC-16/X.25 FCS (low byte first)** — no opening/closing
//! HDLC flag. The FCS is *not* optional: LinBPQ (the de-facto reference)
//! transmits it on every datagram and silently ignores datagrams without it
//! (verified on the wire against LinBPQ 6.0.25 during hardware bring-up,
//! 2026-06-07 — the earlier "FCS-less default" reading of the C# layer did not
//! survive contact with reality).
//!
//! This module is the *pure framing* half. The socket I/O (binding a UDP port,
//! `send`/`recv`, the peer `IpEndpoint`) lives in the firmware crate over
//! `embassy_net::udp::UdpSocket` — which maps 1:1 onto this, per the research
//! note. Here we provide encode (frame → datagram payload) and decode (datagram
//! payload → FCS-checked frame), both host-tested.

use crate::ax25::Frame;
use crate::crc;
use alloc::vec::Vec;

/// Build the AXUDP datagram payload for `frame`: the encoded AX.25 body with
/// the CRC-16/X.25 FCS appended, low byte first — matching `AxudpSocket.SendAsync`.
pub fn encode_datagram(frame: &Frame) -> Vec<u8> {
    append_fcs(frame.encode())
}

/// Append the AXUDP trailing FCS to already-encoded AX.25 wire octets (the path
/// for frames emitted by the session runtime, which produces raw wire bytes).
pub fn append_fcs(mut body: Vec<u8>) -> Vec<u8> {
    let fcs = crc::compute(&body);
    body.push((fcs & 0xFF) as u8); // low byte first on the wire
    body.push((fcs >> 8) as u8);
    body
}

/// Outcome of decoding a received AXUDP datagram.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReceivedDatagram {
    /// The decoded frame. `None` if the FCS was missing/invalid or the body
    /// didn't parse as AX.25 — either way the datagram must not be processed.
    pub frame: Option<Frame>,
    /// Whether a trailing CRC FCS was present and valid. When `false`, `frame`
    /// is always `None` (a failed integrity check poisons the whole datagram).
    pub fcs_valid: bool,
}

/// The smallest datagram worth checking: a minimum AX.25 frame (2 address
/// slots plus control = 15 octets) plus the 2-octet FCS. Mirrors the C#
/// `TryStripFcs` length gate, so a tiny datagram can never report a "valid" FCS.
pub const MIN_DATAGRAM_LEN: usize = (2 * 7) + 1 + 2;

/// Decode a received AXUDP datagram payload: split off the trailing 2-byte FCS,
/// verify it over the body, and parse the body as AX.25. Total — arbitrary
/// bytes yield `frame: None`, never a panic.
pub fn decode_datagram(payload: &[u8]) -> ReceivedDatagram {
    if payload.len() < MIN_DATAGRAM_LEN {
        return ReceivedDatagram {
            frame: None,
            fcs_valid: false,
        };
    }
    let (body, fcs_bytes) = payload.split_at(payload.len() - 2);
    let stored = (fcs_bytes[0] as u16) | ((fcs_bytes[1] as u16) << 8);
    if stored != crc::compute(body) {
        return ReceivedDatagram {
            frame: None,
            fcs_valid: false,
        };
    }
    ReceivedDatagram {
        frame: Frame::decode(body).ok(),
        fcs_valid: true,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ax25::{Address, Callsign, PID_NO_LAYER3};

    fn frame() -> Frame {
        Frame {
            destination: Address {
                callsign: Callsign::parse("APRS").unwrap(),
                crh: true,
                extension: false,
            },
            source: Address {
                callsign: Callsign::parse("M0LTE-1").unwrap(),
                crh: false,
                extension: false,
            },
            digipeaters: Vec::new(),
            control: crate::ax25::frame::CONTROL_UI,
            pid: Some(PID_NO_LAYER3),
            info: b"axudp test".to_vec(),
        }
    }

    #[test]
    fn round_trip_validates_fcs() {
        let f = frame();
        let dgram = encode_datagram(&f);
        assert_eq!(dgram.len(), f.encoded_len() + 2);
        let r = decode_datagram(&dgram);
        assert_eq!(r.frame, Some(f));
        assert!(r.fcs_valid);
    }

    #[test]
    fn fcs_low_byte_is_first() {
        let f = frame();
        let body = f.encode();
        let fcs = crc::compute(&body);
        let dgram = encode_datagram(&f);
        let n = dgram.len();
        assert_eq!(dgram[n - 2], (fcs & 0xFF) as u8);
        assert_eq!(dgram[n - 1], (fcs >> 8) as u8);
    }

    #[test]
    fn append_fcs_matches_encode_datagram() {
        let f = frame();
        assert_eq!(append_fcs(f.encode()), encode_datagram(&f));
    }

    #[test]
    fn corrupted_fcs_rejects_the_datagram() {
        let f = frame();
        let mut dgram = encode_datagram(&f);
        let n = dgram.len();
        dgram[n - 1] ^= 0xFF; // smash the FCS high byte
        let r = decode_datagram(&dgram);
        assert_eq!(r.frame, None);
        assert!(!r.fcs_valid);
    }

    #[test]
    fn corrupted_body_rejects_the_datagram() {
        let f = frame();
        let mut dgram = encode_datagram(&f);
        dgram[0] ^= 0x80; // flip a bit in the destination address
        let r = decode_datagram(&dgram);
        assert_eq!(r.frame, None);
        assert!(!r.fcs_valid);
    }

    #[test]
    fn fcsless_datagram_is_rejected() {
        // The body alone (no FCS appended) must not be accepted: the last two
        // body octets are interpreted as the FCS and won't verify.
        let f = frame();
        let r = decode_datagram(&f.encode());
        assert_eq!(r.frame, None);
        assert!(!r.fcs_valid);
    }

    #[test]
    fn short_datagrams_are_rejected() {
        for payload in [&[][..], &[0x01][..]] {
            let r = decode_datagram(payload);
            assert_eq!(r.frame, None);
            assert!(!r.fcs_valid);
        }
    }

    #[test]
    fn sub_minimum_datagram_with_a_matching_crc_is_rejected() {
        // 14 body bytes + a correct FCS = 16 bytes: below the C# TryStripFcs
        // 17-byte gate (min AX.25 frame + FCS), so it must be rejected outright,
        // not reported as fcs_valid with no frame.
        let body = [0xAAu8; 14];
        let mut dgram = body.to_vec();
        let fcs = crate::crc::compute(&body);
        dgram.push((fcs & 0xFF) as u8);
        dgram.push((fcs >> 8) as u8);
        assert_eq!(dgram.len(), MIN_DATAGRAM_LEN - 1);
        let r = decode_datagram(&dgram);
        assert_eq!(r.frame, None);
        assert!(!r.fcs_valid);
    }
}
