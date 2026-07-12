//! The NinoTNC GETRSSI reply — an RX-audio level reading.
//!
//! Ports `Packet.Kiss.NinoTnc.NinoTncRssiReading`. The firmware answers a GETRSSI
//! query with an ASCII `"RSSI:-62.54"` payload on the reply command byte `0xE0`
//! (port 14 / KISS Data through a multi-drop decoder — see [`is_reply`]).
//!
//! Despite the name this is **not** an RF dBm figure: bench measurement
//! (2026-07-02, firmware 3.41 on the Tait rig) shows it is the RMS level of the
//! TNC's RX audio in dB — open-squelch flat-tap noise reads ≈ −33, a carrier
//! quieting the channel with a 440 Hz CQBEEP tone reads ≈ −62. It tracks what the
//! modem actually hears, which makes it a remote audio-level meter for
//! deviation/level tuning. **Firmware 3.41 only:** GETRSSI was removed in 3.44
//! (no reply at all), so no frame of this shape ever arrives from a 3.44 TNC.
//!
//! ## No-FPU representation (divergence from C#, documented)
//!
//! The C# stores `float LevelDb`. The embedded node core is FPU-free (integer maths
//! only — [`crate`] builds for a Cortex-M0+ with no hardware float), so the level is
//! kept as a **fixed-point signed integer in hundredths of a dB** ("centi-dB"),
//! which is exact for the two-decimal-place wire form. The *acceptance* semantics —
//! which bytes parse, which are rejected — mirror the C# `TryParse` byte-for-byte;
//! only the numeric container differs. Exponent notation (`1e3`), which
//! `NumberStyles.Float` technically allows but a NinoTNC never emits, is rejected.

use crate::kiss::frame::Frame;

/// The raw KISS command byte the firmware uses for direct query replies (GETVER /
/// GETALL / GETRSSI / GETSERNO). Decodes as port 14 + command 0x0 through a standard
/// multi-drop KISS decoder. Mirrors `NinoTncCommands.ReplyCommandByte`.
pub const REPLY_COMMAND_BYTE: u8 = 0xE0;

/// The `RSSI:` ASCII prefix a GETRSSI reply carries.
pub const PREFIX: &[u8] = b"RSSI:";

/// True when `frame` is a firmware query reply — its raw KISS command byte is
/// [`REPLY_COMMAND_BYTE`] (0xE0). Mirrors `NinoTncCommands.IsReply`.
pub fn is_reply(frame: &Frame) -> bool {
    frame.command_byte() == REPLY_COMMAND_BYTE
}

/// A decoded GETRSSI reply: the RX-audio RMS level in hundredths of a dB.
///
/// Mirrors `NinoTncRssiReading` (whose `LevelDb` is a `float`; see the module docs
/// for why the port uses fixed-point centi-dB instead).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NinoTncRssiReading {
    /// RX-audio RMS level in centi-dB (hundredths of a dB). E.g. `"-62.54"` → `-6254`.
    pub centi_db: i32,
}

impl NinoTncRssiReading {
    /// The level in tenths of a dB (the `i16` "tenths-of-dBm" convention the node's
    /// level plumbing uses elsewhere), truncated toward zero from the centi-dB value.
    pub fn tenths_db(self) -> i16 {
        (self.centi_db / 10) as i16
    }

    /// The whole-dB part of the level (truncated toward zero).
    pub fn whole_db(self) -> i32 {
        self.centi_db / 100
    }

    /// Try to parse a GETRSSI reply out of a decoded KISS [`Frame`]. Requires the
    /// firmware reply command byte ([`is_reply`]) and the `RSSI:` prefix.
    ///
    /// Mirrors `NinoTncRssiReading.TryParse(KissFrame, …)`.
    pub fn try_parse_frame(frame: &Frame) -> Option<Self> {
        if !is_reply(frame) {
            return None;
        }
        Self::try_parse(&frame.payload)
    }

    /// Try to parse a GETRSSI reply out of raw reply-frame payload bytes (`RSSI:` +
    /// a decimal level). Returns `None` if the prefix is absent or the level does not
    /// parse. Total — arbitrary bytes never panic.
    ///
    /// Mirrors `NinoTncRssiReading.TryParse(ReadOnlySpan<byte>, …)`.
    pub fn try_parse(payload: &[u8]) -> Option<Self> {
        if payload.len() <= PREFIX.len() {
            return None;
        }
        if &payload[..PREFIX.len()] != PREFIX {
            return None;
        }
        let centi_db = parse_signed_centi(&payload[PREFIX.len()..])?;
        Some(Self { centi_db })
    }
}

/// Parse an ASCII signed decimal with up to two fractional digits into centi-dB
/// (value × 100). Trims surrounding ASCII whitespace, accepts an optional leading
/// `+`/`-`, and truncates any fractional digits past the second. Rejects empty
/// input, any non-digit (including exponent markers), and out-of-`i32` magnitudes.
fn parse_signed_centi(bytes: &[u8]) -> Option<i32> {
    let text = trim_ascii(bytes);
    if text.is_empty() {
        return None;
    }
    let (negative, rest) = match text[0] {
        b'-' => (true, &text[1..]),
        b'+' => (false, &text[1..]),
        _ => (false, text),
    };
    if rest.is_empty() {
        return None;
    }

    let (int_part, frac_part) = match rest.iter().position(|&b| b == b'.') {
        Some(dot) => (&rest[..dot], &rest[dot + 1..]),
        None => (rest, &[][..]),
    };
    // Reject a bare "." (no digit on either side); ".5" / "5." are permitted.
    if int_part.is_empty() && frac_part.is_empty() {
        return None;
    }

    let mut whole: i64 = 0;
    for &b in int_part {
        let d = ascii_digit(b)?;
        whole = whole.checked_mul(10)?.checked_add(d as i64)?;
    }

    // Take the first two fractional digits (centi precision), validating the rest.
    let mut centi_frac: i64 = 0;
    let mut taken = 0;
    for &b in frac_part {
        let d = ascii_digit(b)?;
        if taken < 2 {
            centi_frac = centi_frac * 10 + d as i64;
            taken += 1;
        }
    }
    while taken < 2 {
        centi_frac *= 10;
        taken += 1;
    }

    let total = whole.checked_mul(100)?.checked_add(centi_frac)?;
    let signed = if negative { -total } else { total };
    i32::try_from(signed).ok()
}

fn ascii_digit(b: u8) -> Option<u8> {
    if b.is_ascii_digit() {
        Some(b - b'0')
    } else {
        None
    }
}

fn trim_ascii(bytes: &[u8]) -> &[u8] {
    let mut start = 0;
    let mut end = bytes.len();
    while start < end && bytes[start].is_ascii_whitespace() {
        start += 1;
    }
    while end > start && bytes[end - 1].is_ascii_whitespace() {
        end -= 1;
    }
    &bytes[start..end]
}

#[cfg(all(test, feature = "alloc"))]
mod tests {
    use super::*;
    use crate::kiss::frame::Command;
    use alloc::vec::Vec;

    fn reply_frame(payload: &[u8]) -> Frame {
        // Reply command byte 0xE0 = port 14 + Data nibble 0.
        Frame::new(14, Command::Data, payload.to_vec())
    }

    #[test]
    fn parses_the_bench_captured_negative_reading() {
        let r = NinoTncRssiReading::try_parse(b"RSSI:-62.54").unwrap();
        assert_eq!(r.centi_db, -6254);
        assert_eq!(r.whole_db(), -62);
        assert_eq!(r.tenths_db(), -625);
    }

    #[test]
    fn parses_an_integer_reading_without_a_decimal_point() {
        let r = NinoTncRssiReading::try_parse(b"RSSI:-33").unwrap();
        assert_eq!(r.centi_db, -3300);
    }

    #[test]
    fn parses_a_positive_open_squelch_style_reading() {
        // Firmware trims; a leading '+'/space must still parse.
        let r = NinoTncRssiReading::try_parse(b"RSSI: 32.86 ").unwrap();
        assert_eq!(r.centi_db, 3286);
    }

    #[test]
    fn parses_through_a_reply_frame() {
        let frame = reply_frame(b"RSSI:-32.86");
        let r = NinoTncRssiReading::try_parse_frame(&frame).unwrap();
        assert_eq!(r.centi_db, -3286);
    }

    #[test]
    fn a_non_reply_command_byte_is_rejected() {
        // Port 0 Data (command byte 0x00) is not the 0xE0 reply byte.
        let frame = Frame::new(0, Command::Data, b"RSSI:-32.86".to_vec());
        assert!(NinoTncRssiReading::try_parse_frame(&frame).is_none());
    }

    #[test]
    fn missing_prefix_is_rejected() {
        assert!(NinoTncRssiReading::try_parse(b"LEVL:-62.54").is_none());
        assert!(NinoTncRssiReading::try_parse(b"RSSI:").is_none(), "prefix only, no value");
    }

    #[test]
    fn non_numeric_level_is_rejected() {
        assert!(NinoTncRssiReading::try_parse(b"RSSI:banana").is_none());
        assert!(NinoTncRssiReading::try_parse(b"RSSI:-").is_none());
        assert!(NinoTncRssiReading::try_parse(b"RSSI:1e3").is_none(), "exponent not emitted by a NinoTNC");
        assert!(NinoTncRssiReading::try_parse(b"RSSI:.").is_none());
    }

    #[test]
    fn truncates_fractional_digits_past_the_second() {
        let r = NinoTncRssiReading::try_parse(b"RSSI:-62.549").unwrap();
        assert_eq!(r.centi_db, -6254, "third fractional digit is dropped, not rounded");
    }

    #[test]
    fn arbitrary_bytes_never_panic() {
        let mut payload: Vec<u8> = PREFIX.to_vec();
        payload.extend_from_slice(&[0x00, 0xFF, 0x80, b'-', b'.', b'e']);
        let _ = NinoTncRssiReading::try_parse(&payload);
    }
}
