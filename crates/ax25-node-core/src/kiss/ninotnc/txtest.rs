//! The NinoTNC "TX-Test" diagnostic frame — the synthetic, host-side-only KISS Data
//! frame the firmware emits when the operator presses the on-board TX-Test button.
//!
//! Ports `Packet.Kiss.NinoTnc.NinoTncTxTestFrame`. The frame is a regular KISS Data
//! frame (command `0x00`) whose payload is an ASCII run of `=Key:Value` pairs (no
//! separators between pairs), e.g.:
//!
//! ```text
//! =FirmwareVr:3.44=SerialNmbr:...=UptimeMilS:0001A2B3=BrdSwchMod:040F0023
//! =AX25RxPkts:0000007F=IL2PRxPkts:00000000=...=LostADCSmp:00000000
//! ```
//!
//! Numeric fields are hex-encoded. `BrdSwchMod` packs four bytes: `XX` (board
//! revision), `YY` (DIP switch position, low 4 bits), then `ZZZZ` (a 16-bit firmware
//! mode value — its low byte is the "running mode" lookup index in
//! [`super::catalog::try_get_by_firmware_byte`]).
//!
//! The parser is permissive: it scans for the `=FirmwareVr:` marker rather than
//! assuming an AX.25-shaped prefix (firmware emits this as a KISS Data frame and the
//! bytes before the marker are not a real address header). It is total — arbitrary
//! bytes never panic.
//!
//! `no_std`: where the C# splits into a `Dictionary<string,string>`, this scans the
//! ASCII region for each known key on demand (no allocation, no `String`/`HashMap`).
//! String-valued fields (firmware version, serial) are returned as fixed-capacity
//! `Copy` buffers rather than heap `String`s.

use super::catalog::NinoTncMode;
use super::firmware::{ChipVariant, FirmwareVersion};
use crate::kiss::frame::{Command, Frame};

/// The `=FirmwareVr:` marker that begins the diagnostic payload.
const MARKER: &[u8] = b"=FirmwareVr:";

/// Max bytes kept for a string-valued field (firmware version, serial number). The
/// real fields are short; over-long input is truncated to this.
pub const MAX_FIELD_LEN: usize = 32;

/// A short, fixed-capacity ASCII field value (firmware-version string, serial
/// number) — the no-heap stand-in for the C# `string?`. Empty when absent.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct FieldStr {
    buf: [u8; MAX_FIELD_LEN],
    len: u8,
}

impl FieldStr {
    fn from_bytes(b: &[u8]) -> Self {
        let mut buf = [0u8; MAX_FIELD_LEN];
        let n = b.len().min(MAX_FIELD_LEN);
        buf[..n].copy_from_slice(&b[..n]);
        Self { buf, len: n as u8 }
    }

    /// The significant bytes.
    pub fn as_bytes(&self) -> &[u8] {
        &self.buf[..self.len as usize]
    }

    /// The value as `&str` (best-effort; `""` if not valid UTF-8).
    pub fn as_str(&self) -> &str {
        core::str::from_utf8(self.as_bytes()).unwrap_or("")
    }

    /// True if the field carried no bytes.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
}

/// A decoded NinoTNC TX-Test diagnostic frame. Every field is optional, matching the
/// C# nullable layout — a permissive parse fills in what it can and leaves the rest
/// `None` / empty so callers can detect-and-report rather than crash.
///
/// Mirrors `NinoTncTxTestFrame`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct NinoTncTxTestFrame {
    /// Parsed firmware version (e.g. `3.44`). `None` if the field was missing or
    /// unparseable (the raw text is still kept in [`Self::firmware_version_raw`]).
    pub firmware_version: Option<FirmwareVersion>,
    /// The raw firmware-version string the firmware emitted (e.g. `"3.44"`). Empty
    /// if the field was missing.
    pub firmware_version_raw: FieldStr,
    /// Serial number. Empty when the TNC has none set (`\0`-padding stripped).
    pub serial_number: FieldStr,
    /// Uptime in milliseconds, decoded from `UptimeMilS` (hex).
    pub uptime_ms: Option<u64>,
    /// The `XX` byte from `BrdSwchMod` — the board revision number.
    pub board_revision: Option<u8>,
    /// The `YY` byte from `BrdSwchMod` — the DIP-switch position (0–15).
    pub dip_switch_position: Option<u8>,
    /// The low byte of the `ZZZZ` field from `BrdSwchMod` — the firmware-mode
    /// identifier that [`super::catalog::try_get_by_firmware_byte`] maps to the
    /// "actually running" mode (matters when DIP=15 = "Set from KISS").
    pub firmware_mode_byte: Option<u8>,
    /// The mode the TNC is currently running, resolved through the catalog. `None`
    /// if the firmware byte isn't in the catalog (firmware likely newer than ours).
    pub running_mode: Option<NinoTncMode>,
    /// Count of received AX.25 packets since boot.
    pub ax25_rx_packets: Option<u64>,
    /// Count of received IL2P packets since boot.
    pub il2p_rx_packets: Option<u64>,
    /// Count of IL2P packets received with uncorrectable errors.
    pub il2p_rx_uncorrectable: Option<u64>,
    /// Count of transmitted packets since boot.
    pub tx_packet_count: Option<u64>,
    /// Count of received preambles since boot.
    pub preamble_count: Option<u64>,
    /// Firmware main-loop cycle count since boot.
    pub loop_cycles: Option<u64>,
    /// Count of dropped ADC samples since boot.
    pub lost_adc_samples: Option<u64>,
}

impl NinoTncTxTestFrame {
    /// The dsPIC chip variant, derived from the firmware version's major component.
    pub fn chip_variant(&self) -> ChipVariant {
        self.firmware_version
            .map(|v| v.chip_variant())
            .unwrap_or(ChipVariant::Unknown)
    }

    /// Try to parse a TX-Test frame out of a decoded KISS frame. Only succeeds for
    /// [`Command::Data`] (mirrors the C# `TryParse(KissFrame, …)`).
    pub fn try_parse_frame(frame: &Frame) -> Option<Self> {
        if frame.command != Command::Data {
            return None;
        }
        Self::try_parse(&frame.payload)
    }

    /// Try to parse a TX-Test frame out of raw KISS-frame payload bytes (post-
    /// unescape). Returns `None` if the `=FirmwareVr:` marker is absent. Total.
    ///
    /// Mirrors `NinoTncTxTestFrame.TryParse(ReadOnlySpan<byte>, …)`.
    pub fn try_parse(payload: &[u8]) -> Option<Self> {
        let marker_index = index_of(payload, MARKER)?;
        let ascii = &payload[marker_index..];

        let mut out = NinoTncTxTestFrame::default();

        // FirmwareVr (string): kept raw, plus a strong parse attempt.
        if let Some(v) = field_value(ascii, b"FirmwareVr") {
            out.firmware_version_raw = FieldStr::from_bytes(v);
            out.firmware_version = core::str::from_utf8(v)
                .ok()
                .and_then(FirmwareVersion::parse);
        }

        // SerialNmbr (string): strip NULs + trim; empty → absent.
        if let Some(v) = field_value(ascii, b"SerialNmbr") {
            out.serial_number = normalise_serial(v);
        }

        out.uptime_ms = hex_field(ascii, b"UptimeMilS");

        // BrdSwchMod (hex string): XX YY ZZZZ (≥8 hex chars).
        if let Some(v) = field_value(ascii, b"BrdSwchMod") {
            if v.len() >= 8 {
                if let Some(xx) = parse_hex_byte(&v[0..2]) {
                    out.board_revision = Some(xx);
                }
                if let Some(yy) = parse_hex_byte(&v[2..4]) {
                    out.dip_switch_position = Some(yy & 0x0F);
                }
                // ZZZZ is 4 hex chars; the catalog keys on its low byte (chars 6..8).
                if let Some(low_z) = parse_hex_byte(&v[6..8]) {
                    out.firmware_mode_byte = Some(low_z);
                    out.running_mode = super::catalog::try_get_by_firmware_byte(low_z);
                }
            }
        }

        out.ax25_rx_packets = hex_field(ascii, b"AX25RxPkts");
        out.il2p_rx_packets = hex_field(ascii, b"IL2PRxPkts");
        out.il2p_rx_uncorrectable = hex_field(ascii, b"IL2PRxUnCr");
        out.tx_packet_count = hex_field(ascii, b"TxPktCount");
        out.preamble_count = hex_field(ascii, b"PreamblCnt");
        out.loop_cycles = hex_field(ascii, b"LoopCycles");
        out.lost_adc_samples = hex_field(ascii, b"LostADCSmp");

        Some(out)
    }
}

/// Find the value bytes for `=key:` within the `=Key:Value=Key:Value…` region. The
/// value runs from after the `:` up to (but not including) the next `=` or the end.
/// Returns `None` if the key is absent, or if its value is empty (matching the C#
/// `colon == pair.Length - 1` skip and `TryGetValue` miss). Mirrors the dictionary
/// build + `GetValueOrDefault`.
fn field_value<'a>(ascii: &'a [u8], key: &[u8]) -> Option<&'a [u8]> {
    // Build the needle "=KEY:" and search for it; the value is what follows.
    let mut i = 0;
    while i < ascii.len() {
        if ascii[i] == b'=' {
            let after_eq = &ascii[i + 1..];
            // Does this pair start with `key:`?
            if after_eq.len() > key.len()
                && after_eq[..key.len()] == *key
                && after_eq[key.len()] == b':'
            {
                let value_start = i + 1 + key.len() + 1;
                let mut value_end = value_start;
                while value_end < ascii.len() && ascii[value_end] != b'=' {
                    value_end += 1;
                }
                if value_end == value_start {
                    // Empty value — the C# parser skips `colon == last` pairs.
                    return None;
                }
                return Some(&ascii[value_start..value_end]);
            }
        }
        i += 1;
    }
    None
}

/// Parse a hex-valued field into `u64` (the C# `long.TryParse(NumberStyles.HexNumber)`
/// equivalent). Returns `None` if the field is absent or not valid hex.
fn hex_field(ascii: &[u8], key: &[u8]) -> Option<u64> {
    let v = field_value(ascii, key)?;
    parse_hex_u64(v)
}

/// Parse an exactly-2-char hex byte. `None` if not 2 hex digits.
fn parse_hex_byte(hex: &[u8]) -> Option<u8> {
    if hex.len() != 2 {
        return None;
    }
    let hi = hex_digit(hex[0])?;
    let lo = hex_digit(hex[1])?;
    Some((hi << 4) | lo)
}

/// Parse a hex string into `u64`. `None` on empty or any non-hex byte, or on
/// overflow (> 16 hex digits).
fn parse_hex_u64(hex: &[u8]) -> Option<u64> {
    if hex.is_empty() || hex.len() > 16 {
        return None;
    }
    let mut acc: u64 = 0;
    for &b in hex {
        acc = (acc << 4) | hex_digit(b)? as u64;
    }
    Some(acc)
}

fn hex_digit(b: u8) -> Option<u8> {
    match b {
        b'0'..=b'9' => Some(b - b'0'),
        b'a'..=b'f' => Some(b - b'a' + 10),
        b'A'..=b'F' => Some(b - b'A' + 10),
        _ => None,
    }
}

/// Strip NUL bytes + trim ASCII whitespace; empty result → empty field (the C#
/// `NormaliseSerial` returning `null`).
fn normalise_serial(raw: &[u8]) -> FieldStr {
    let mut tmp = [0u8; MAX_FIELD_LEN];
    let mut n = 0;
    for &b in raw {
        if b != 0 && n < MAX_FIELD_LEN {
            tmp[n] = b;
            n += 1;
        }
    }
    // Trim leading/trailing ASCII whitespace.
    let mut start = 0;
    let mut end = n;
    while start < end && tmp[start].is_ascii_whitespace() {
        start += 1;
    }
    while end > start && tmp[end - 1].is_ascii_whitespace() {
        end -= 1;
    }
    FieldStr::from_bytes(&tmp[start..end])
}

/// First index of `needle` in `haystack`, or `None`.
fn index_of(haystack: &[u8], needle: &[u8]) -> Option<usize> {
    if needle.is_empty() || haystack.len() < needle.len() {
        return None;
    }
    haystack.windows(needle.len()).position(|w| w == needle)
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec::Vec;

    /// Mirror the C# `PayloadFor` helper: a non-AX.25 prefix before the marker.
    fn payload_for(body: &str) -> Vec<u8> {
        let mut v = Vec::new();
        v.extend_from_slice(&[0x01, 0x02, 0x03]);
        v.extend_from_slice(b"prefix-garbage");
        v.extend_from_slice(body.as_bytes());
        v
    }

    #[test]
    fn parses_all_documented_fields_from_a_synthetic_frame() {
        // board rev 04, DIP=0F (15), ZZZZ=0023 → low byte 0x23 → mode 14.
        let body = "=FirmwareVr:3.44=SerialNmbr:ABC123=UptimeMilS:0001A2B3\
                    =BrdSwchMod:040F0023=AX25RxPkts:0000007F=IL2PRxPkts:00000005\
                    =IL2PRxUnCr:00000001=TxPktCount:0000003E=PreamblCnt:00000041\
                    =LoopCycles:000A28F2=LostADCSmp:00000002";
        let parsed = NinoTncTxTestFrame::try_parse(&payload_for(body)).unwrap();

        assert_eq!(parsed.firmware_version_raw.as_str(), "3.44");
        assert_eq!(
            parsed.firmware_version,
            Some(FirmwareVersion {
                major: 3,
                minor: 44
            })
        );
        assert_eq!(parsed.chip_variant(), ChipVariant::Dspic33Ep256);
        assert_eq!(parsed.serial_number.as_str(), "ABC123");
        assert_eq!(parsed.uptime_ms, Some(0x0001_A2B3));
        assert_eq!(parsed.board_revision, Some(0x04));
        assert_eq!(parsed.dip_switch_position, Some(0x0F));
        assert_eq!(parsed.firmware_mode_byte, Some(0x23));
        assert_eq!(parsed.running_mode.unwrap().mode, 14);
        assert_eq!(parsed.ax25_rx_packets, Some(0x7F));
        assert_eq!(parsed.il2p_rx_packets, Some(5));
        assert_eq!(parsed.il2p_rx_uncorrectable, Some(1));
        assert_eq!(parsed.tx_packet_count, Some(0x3E));
        assert_eq!(parsed.preamble_count, Some(0x41));
        assert_eq!(parsed.loop_cycles, Some(0x000A_28F2));
        assert_eq!(parsed.lost_adc_samples, Some(2));
    }

    #[test]
    fn returns_none_when_marker_missing() {
        let bytes = b"=SomethingElse:1234=FirmwareWrong:nope";
        assert_eq!(NinoTncTxTestFrame::try_parse(bytes), None);
    }

    #[test]
    fn try_parse_frame_only_succeeds_for_data_command() {
        let body = b"=FirmwareVr:3.44=BrdSwchMod:040F0002".to_vec();
        let data = Frame::new(0, Command::Data, body.clone());
        let parsed = NinoTncTxTestFrame::try_parse_frame(&data).unwrap();
        assert_eq!(parsed.firmware_version_raw.as_str(), "3.44");
        assert_eq!(parsed.running_mode.unwrap().mode, 6); // 0x02 → mode 6

        // Wrong command fails even though the payload would parse.
        let param = Frame::new(0, Command::SetHardware, body);
        assert_eq!(NinoTncTxTestFrame::try_parse_frame(&param), None);
    }

    #[test]
    fn tolerates_truncated_brdswchmod() {
        // Only board rev + DIP (4 chars total — too short to extract sub-bytes).
        let body = "=FirmwareVr:3.44=BrdSwchMod:0406";
        let parsed = NinoTncTxTestFrame::try_parse(&payload_for(body)).unwrap();
        assert_eq!(parsed.board_revision, None);
        assert_eq!(parsed.running_mode, None);
    }

    #[test]
    fn empty_serial_number_becomes_empty() {
        let body = "=FirmwareVr:3.44=SerialNmbr:\0\0\0\0";
        let parsed = NinoTncTxTestFrame::try_parse(&payload_for(body)).unwrap();
        assert!(parsed.serial_number.is_empty());
    }

    #[test]
    fn unparseable_firmwarevr_degrades_gracefully() {
        let body = "=FirmwareVr:banana=BrdSwchMod:040F0002";
        let parsed = NinoTncTxTestFrame::try_parse(&payload_for(body)).unwrap();
        assert_eq!(parsed.firmware_version_raw.as_str(), "banana");
        assert_eq!(parsed.firmware_version, None);
        assert_eq!(parsed.chip_variant(), ChipVariant::Unknown);
    }

    #[test]
    fn missing_firmwarevr_leaves_parse_failed() {
        // No marker at all → the parser fails outright (pins the C# behaviour).
        let body = "=BrdSwchMod:040F0002";
        assert_eq!(NinoTncTxTestFrame::try_parse(&payload_for(body)), None);
    }

    #[test]
    fn arbitrary_garbage_never_panics() {
        // Totality: feed adversarial byte runs; must return None, not panic.
        for n in 0..64usize {
            let junk: Vec<u8> = (0..n).map(|i| (i as u8).wrapping_mul(37) ^ 0xA5).collect();
            let _ = NinoTncTxTestFrame::try_parse(&junk);
        }
        // A marker followed immediately by EOF / a stray '=' must also be safe.
        let _ = NinoTncTxTestFrame::try_parse(b"=FirmwareVr:");
        let _ = NinoTncTxTestFrame::try_parse(b"=FirmwareVr:=");
        let _ = NinoTncTxTestFrame::try_parse(b"=FirmwareVr:3.44=BrdSwchMod:");
    }

    #[test]
    fn field_value_handles_value_at_end_of_buffer() {
        // A field whose value runs to the end (no trailing '=').
        let body = "=FirmwareVr:3.44=TxPktCount:000000FF";
        let parsed = NinoTncTxTestFrame::try_parse(&payload_for(body)).unwrap();
        assert_eq!(parsed.tx_packet_count, Some(0xFF));
    }
}
