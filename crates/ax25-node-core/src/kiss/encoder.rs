//! KISS frame encoder. Ports `Packet.Kiss.KissEncoder`.
//!
//! Produces `FEND | (port<<4)|cmd | escaped-payload | FEND`. Like direwolf (and
//! unlike the literal KISS spec text), the command byte is escaped too, so e.g.
//! port 12 + a `Data` command can't accidentally emit a bare `FEND`.
//!
//! Two entry points mirror the C# API:
//! - [`encode_into`] — write into a caller-provided buffer (the embedded path:
//!   no allocation, the firmware owns a fixed scratch buffer).
//! - [`encode`] — allocate and return a `Vec` (host/convenience; needs `alloc`).

use super::frame::{Command, FEND, FESC, TFEND, TFESC};

#[cfg(feature = "alloc")]
use alloc::vec::Vec;

/// Maximum possible encoded length for a given payload length: two FENDs, a
/// worst-case-escaped command byte (2), and worst-case-escaped payload (2× each).
pub const fn max_encoded_len(payload_len: usize) -> usize {
    4 + payload_len * 2
}

/// Encode a KISS frame into `dst`. Returns the number of bytes written, or
/// `None` if `dst` is too small (call [`max_encoded_len`] to size it) or `port`
/// is out of range (0–15).
pub fn encode_into(dst: &mut [u8], port: u8, command: Command, payload: &[u8]) -> Option<usize> {
    if port > 15 {
        return None;
    }
    if dst.len() < max_encoded_len(payload.len()) {
        return None;
    }

    let mut i = 0;
    dst[i] = FEND;
    i += 1;

    let command_byte = ((port & 0x0F) << 4) | command.to_nibble();
    i += write_escaped(&mut dst[i..], command_byte);

    for &b in payload {
        i += write_escaped(&mut dst[i..], b);
    }

    dst[i] = FEND;
    i += 1;
    Some(i)
}

/// Encode a KISS frame and return the wire bytes. Requires `alloc`.
#[cfg(feature = "alloc")]
pub fn encode(port: u8, command: Command, payload: &[u8]) -> Option<Vec<u8>> {
    let mut buf = alloc::vec![0u8; max_encoded_len(payload.len())];
    let n = encode_into(&mut buf, port, command, payload)?;
    buf.truncate(n);
    Some(buf)
}

fn write_escaped(dst: &mut [u8], b: u8) -> usize {
    match b {
        FEND => {
            dst[0] = FESC;
            dst[1] = TFEND;
            2
        }
        FESC => {
            dst[0] = FESC;
            dst[1] = TFESC;
            2
        }
        _ => {
            dst[0] = b;
            1
        }
    }
}

#[cfg(all(test, feature = "alloc"))]
mod tests {
    use super::*;

    #[test]
    fn plain_payload_round_frames() {
        let out = encode(0, Command::Data, &[0x01, 0x02, 0x03]).unwrap();
        assert_eq!(out, vec![FEND, 0x00, 0x01, 0x02, 0x03, FEND]);
    }

    #[test]
    fn port_goes_in_high_nibble() {
        let out = encode(5, Command::Data, &[]).unwrap();
        // command byte = (5 << 4) | 0 = 0x50
        assert_eq!(out, vec![FEND, 0x50, FEND]);
    }

    #[test]
    fn payload_fend_is_escaped() {
        let out = encode(0, Command::Data, &[FEND]).unwrap();
        assert_eq!(out, vec![FEND, 0x00, FESC, TFEND, FEND]);
    }

    #[test]
    fn payload_fesc_is_escaped() {
        let out = encode(0, Command::Data, &[FESC]).unwrap();
        assert_eq!(out, vec![FEND, 0x00, FESC, TFESC, FEND]);
    }

    #[test]
    fn command_byte_is_escaped_when_it_collides_with_fend() {
        // port 12 (0xC) + Data (0x0) => command byte 0xC0 == FEND; must be escaped
        // or the stream is undecodable.
        let out = encode(12, Command::Data, &[0xAA]).unwrap();
        assert_eq!(out, vec![FEND, FESC, TFEND, 0xAA, FEND]);
    }

    #[test]
    fn rejects_port_out_of_range() {
        assert!(encode(16, Command::Data, &[]).is_none());
    }

    #[test]
    fn encode_into_reports_too_small() {
        let mut tiny = [0u8; 2];
        assert!(encode_into(&mut tiny, 0, Command::Data, &[1, 2, 3]).is_none());
    }
}
