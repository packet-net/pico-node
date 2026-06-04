//! AXUDP framing helpers — AX.25-over-UDP for node↔node connectivity over WiFi.
//!
//! Ports the framing logic of `Packet.Axudp.AxudpSocket`. AXUDP is the simplest
//! BPQ-compatible encapsulation: **the UDP datagram payload *is* the AX.25 frame
//! body** — no opening/closing HDLC flag, and traditionally no FCS (UDP is
//! reliable enough that the receiver needn't verify one). The one variant is
//! XRouter's AXIP-with-CRC, which appends the CRC-16/X.25 FCS (low byte first);
//! `Packet.Axudp` exposes that as the `includeFcs` flag.
//!
//! This module is the *pure framing* half. The socket I/O (binding a UDP port,
//! `send`/`recv`, the peer `IpEndpoint`) lives in the firmware crate over
//! `embassy_net::udp::UdpSocket` — which maps 1:1 onto this, per the research
//! note. Here we provide encode (frame → datagram payload) and a best-effort
//! decode (datagram payload → frame), both host-tested.

use crate::ax25::Frame;
use crate::crc;
use alloc::vec::Vec;

/// Build the AXUDP datagram payload for `frame`.
///
/// With `include_fcs == false` (the LinBPQ-accepted default) the payload is just
/// the AX.25 frame body. With `include_fcs == true` (XRouter / AXIP-with-CRC) the
/// CRC-16/X.25 FCS is appended, low byte first — matching
/// `AxudpSocket.SendAsync(..., includeFcs: true)`.
pub fn encode_datagram(frame: &Frame, include_fcs: bool) -> Vec<u8> {
    let mut body = frame.encode();
    if include_fcs {
        let fcs = crc::compute(&body);
        body.push((fcs & 0xFF) as u8); // low byte first on the wire
        body.push((fcs >> 8) as u8);
    }
    body
}

/// Outcome of decoding a received AXUDP datagram.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReceivedDatagram {
    /// The decoded frame, or `None` if the bytes didn't parse as AX.25 (the raw
    /// bytes are still available for monitor/forwarding).
    pub frame: Option<Frame>,
    /// `Some(true)` if a trailing CRC FCS was present and valid, `Some(false)` if
    /// present but invalid, `None` if no FCS was checked (FCS-less form assumed).
    pub fcs_valid: Option<bool>,
}

/// Best-effort decode of a received AXUDP datagram payload.
///
/// AXUDP has no length/type header, so we cannot know for certain whether the
/// last two octets are an FCS or frame data. Strategy (matching the C# layer's
/// "best-effort decode, raw bytes retained" contract): first try to parse the
/// payload as-is; if `check_fcs` is set and the payload is long enough, also test
/// whether stripping a trailing 2-byte CRC-16/X.25 leaves a frame whose FCS
/// matches — if so, report it as the validated form.
pub fn decode_datagram(payload: &[u8], check_fcs: bool) -> ReceivedDatagram {
    // FCS-less interpretation: the whole payload is the frame.
    let plain = Frame::decode(payload).ok();

    if check_fcs && payload.len() >= 2 {
        let (body, fcs_bytes) = payload.split_at(payload.len() - 2);
        let stored = (fcs_bytes[0] as u16) | ((fcs_bytes[1] as u16) << 8);
        let computed = crc::compute(body);
        if stored == computed {
            if let Ok(frame) = Frame::decode(body) {
                return ReceivedDatagram {
                    frame: Some(frame),
                    fcs_valid: Some(true),
                };
            }
        }
    }

    ReceivedDatagram {
        frame: plain,
        fcs_valid: None,
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
    fn fcsless_round_trip() {
        let f = frame();
        let dgram = encode_datagram(&f, false);
        // No FCS appended: datagram length == frame length.
        assert_eq!(dgram.len(), f.encoded_len());
        let r = decode_datagram(&dgram, false);
        assert_eq!(r.frame, Some(f));
        assert_eq!(r.fcs_valid, None);
    }

    #[test]
    fn with_fcs_round_trip_validates() {
        let f = frame();
        let dgram = encode_datagram(&f, true);
        assert_eq!(dgram.len(), f.encoded_len() + 2);
        let r = decode_datagram(&dgram, true);
        assert_eq!(r.frame, Some(f));
        assert_eq!(r.fcs_valid, Some(true));
    }

    #[test]
    fn fcs_low_byte_is_first() {
        let f = frame();
        let body = f.encode();
        let fcs = crc::compute(&body);
        let dgram = encode_datagram(&f, true);
        let n = dgram.len();
        assert_eq!(dgram[n - 2], (fcs & 0xFF) as u8);
        assert_eq!(dgram[n - 1], (fcs >> 8) as u8);
    }

    #[test]
    fn corrupted_fcs_not_reported_valid() {
        let f = frame();
        let mut dgram = encode_datagram(&f, true);
        let n = dgram.len();
        dgram[n - 1] ^= 0xFF; // smash the FCS high byte
        let r = decode_datagram(&dgram, true);
        // It won't validate as the FCS form; falls back to the plain interpretation
        // (which decodes the whole payload incl. the bogus 2 bytes as frame data).
        assert_eq!(r.fcs_valid, None);
    }
}
