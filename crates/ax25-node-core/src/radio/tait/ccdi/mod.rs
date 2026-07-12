//! CCDI wire codec — the Tait Computer-Controlled Data Interface (manual §1.8).
//!
//! This is the **shared serialization point** for the whole Tait integration: the
//! checksum, the frame framing, the typed message decode, and the CR-terminated
//! line assembler. It is a pure ASCII + integer codec — no I/O, no allocation on
//! the encode path — so it is fully host-testable against the manual's own worked
//! examples (see the golden vectors in each submodule's tests).
//!
//! Ports `Packet.Radio.Tait.Ccdi`:
//! [`CcdiChecksum`](checksum) · [`CcdiFrame`](frame) · [`CcdiMessage`](message) +
//! the read-pump line discipline as a stateful [`LineDecoder`](decoder).

pub mod checksum;
pub mod decoder;
pub mod frame;
pub mod message;

pub use checksum::{compute as compute_checksum, is_valid as checksum_is_valid};
pub use decoder::{CcdiEvent, LineDecoder};
pub use frame::{CcdiFrame, MAX_LINE, MAX_PARAMS};
pub use message::{CcdiMessage, CcdiProgressType};

// ───────────────────────── shared ASCII/number helpers ─────────────────────────
// CCDI is an all-ASCII protocol: idents, 2-hex sizes, 2-hex checksums, decimal
// query numbers and signed-decimal query values. These integer helpers keep the
// codec `no_std` + FPU-free and are shared across checksum/frame/message.

/// The upper-case ASCII hex digit for a nibble (`0..=15`). Bytes above 15 are
/// masked to the low nibble. Mirrors C#'s `"X"` formatting (upper-case).
pub(crate) const fn hex_upper(nibble: u8) -> u8 {
    match nibble & 0x0F {
        n @ 0..=9 => b'0' + n,
        n => b'A' + (n - 10),
    }
}

/// Value of one ASCII hex digit (`0-9`, `a-f`, `A-F`), or `None`. Case-insensitive,
/// matching `byte.TryParse(.., NumberStyles.HexNumber)`.
pub(crate) const fn from_hex_nibble(b: u8) -> Option<u8> {
    match b {
        b'0'..=b'9' => Some(b - b'0'),
        b'a'..=b'f' => Some(b - b'a' + 10),
        b'A'..=b'F' => Some(b - b'A' + 10),
        _ => None,
    }
}

/// Parse a 1- or 2-char ASCII hex slice into a byte, or `None` if any digit is not
/// hex (or the value overflows a byte). Mirrors `byte.TryParse(span, HexNumber)`.
pub(crate) fn parse_hex_u8(bytes: &[u8]) -> Option<u8> {
    if bytes.is_empty() || bytes.len() > 2 {
        return None;
    }
    let mut v: u16 = 0;
    for &b in bytes {
        v = v * 16 + from_hex_nibble(b)? as u16;
    }
    if v > 0xFF {
        None
    } else {
        Some(v as u8)
    }
}

/// Parse a digits-only ASCII slice into a `u16` (no sign, no whitespace) — the CCTM
/// query number in a `j` result. Mirrors `int.TryParse(span, NumberStyles.None)`.
pub(crate) fn parse_dec_u16(bytes: &[u8]) -> Option<u16> {
    if bytes.is_empty() {
        return None;
    }
    let mut v: u32 = 0;
    for &b in bytes {
        if !b.is_ascii_digit() {
            return None;
        }
        v = v * 10 + (b - b'0') as u32;
        if v > u16::MAX as u32 {
            return None;
        }
    }
    Some(v as u16)
}

/// Parse a signed decimal ASCII slice into an `i32` (optional leading `+`/`-`) — a
/// `j` result value (RSSI in 0.1 dB units, mV, temperature, …). Mirrors
/// `int.TryParse(.., NumberStyles.AllowLeadingSign)`.
pub(crate) fn parse_signed_i32(bytes: &[u8]) -> Option<i32> {
    let (neg, digits) = match bytes.first() {
        Some(b'-') => (true, &bytes[1..]),
        Some(b'+') => (false, &bytes[1..]),
        _ => (false, bytes),
    };
    if digits.is_empty() {
        return None;
    }
    let mut v: i64 = 0;
    for &b in digits {
        if !b.is_ascii_digit() {
            return None;
        }
        v = v * 10 + (b - b'0') as i64;
        if v > i32::MAX as i64 + 1 {
            return None;
        }
    }
    let v = if neg { -v } else { v };
    if v < i32::MIN as i64 || v > i32::MAX as i64 {
        None
    } else {
        Some(v as i32)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hex_upper_covers_all_nibbles() {
        assert_eq!(hex_upper(0x0), b'0');
        assert_eq!(hex_upper(0x9), b'9');
        assert_eq!(hex_upper(0xA), b'A');
        assert_eq!(hex_upper(0xF), b'F');
    }

    #[test]
    fn parse_hex_u8_accepts_both_cases_and_rejects_junk() {
        assert_eq!(parse_hex_u8(b"2F"), Some(0x2F));
        assert_eq!(parse_hex_u8(b"2f"), Some(0x2F));
        assert_eq!(parse_hex_u8(b"05"), Some(0x05));
        assert_eq!(parse_hex_u8(b"ZZ"), None);
        assert_eq!(parse_hex_u8(b""), None);
        assert_eq!(parse_hex_u8(b"123"), None);
    }

    #[test]
    fn parse_dec_u16_is_digits_only() {
        assert_eq!(parse_dec_u16(b"064"), Some(64));
        assert_eq!(parse_dec_u16(b"0"), Some(0));
        assert_eq!(parse_dec_u16(b"-5"), None);
        assert_eq!(parse_dec_u16(b"1a"), None);
    }

    #[test]
    fn parse_signed_i32_handles_sign() {
        assert_eq!(parse_signed_i32(b"-456"), Some(-456));
        assert_eq!(parse_signed_i32(b"1200"), Some(1200));
        assert_eq!(parse_signed_i32(b"+7"), Some(7));
        assert_eq!(parse_signed_i32(b""), None);
        assert_eq!(parse_signed_i32(b"-"), None);
        assert_eq!(parse_signed_i32(b"12x"), None);
    }
}
