//! The NinoTNC numeric diagnostic-register report — the periodic status frame.
//!
//! Ports `Packet.Kiss.NinoTnc.NinoTncStatusFrame`. This is the NUMERIC sibling of
//! the labelled `=FirmwareVr:` diagnostic that [`super::txtest`] parses: a fake UI
//! frame (KISS Data) whose info text is a run of `=II:VALUE` fields, where `II` is a
//! two-hex-digit register index.
//!
//! Bench-observed on firmware 3.41 (2026-07-02): emitted spontaneously once per
//! minute as a KISS Data frame carrying a fake AX.25 UI header `TNC>USB`. The parser
//! ignores the header entirely and scans for the `=00:` marker. Verbatim capture
//! from firmware 3.41:
//!
//! ```text
//! =00:3.41=01:········=02:00AC8F08=03:00000004=04:0000000F=06:00000002
//! =07:0000000D=08:00000004=09:00000000=0A:00000049=0B:00000016=0C:0483CA82
//! =0D:0000F4F6=0E:00014DBA=0F:000001E3=10:00000CE4=11:00000000
//! ```
//!
//! Field encodings differ per register: register 00 is the plain-ASCII firmware
//! version (e.g. `3.41`), register 01 is eight **raw** bytes (the KAUP8R identity
//! register — all-zero when unset, and may itself contain `=`/`:`), every other
//! register is uppercase hex digits.
//!
//! `no_std`: where the C# collects a `Dictionary<byte, byte[]>` of all registers,
//! this makes a single forward pass over the marker sequence (exactly the C# loop)
//! and fills the typed fields directly — no map, no `alloc`. The raw-register bag
//! (`RawRegisters`, kept in C# so newer-firmware additions are not lost) needs a
//! heap map and is intentionally omitted here, matching the map-free approach the
//! sibling [`super::txtest`] parser already takes.

use super::catalog::{self, NinoTncMode};
use super::firmware::FirmwareVersion;
use super::txtest::NinoTncTxTestFrame;
use crate::kiss::frame::{Command, Frame};

/// The register index carrying the plain-ASCII firmware version.
pub const FIRMWARE_VERSION_REGISTER: u8 = 0x00;

/// The register index carrying the raw 8-byte KAUP8R identity value.
pub const SERIAL_NUMBER_REGISTER: u8 = 0x01;

/// The KAUP8R identity register length, in bytes.
pub const SERIAL_NUMBER_LENGTH: usize = 8;

/// Max bytes kept for a status string field (firmware version, serial number).
pub const MAX_REG_STR: usize = 32;

/// A short, fixed-capacity ASCII field value — the no-heap stand-in for the C#
/// `string?` on the status registers. Empty when the register was absent/blank.
///
/// (The sibling [`super::txtest`] parser has its own equivalent `FieldStr`, but its
/// constructor is module-private, so the status parser carries this local twin
/// rather than reach across into that module.)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RegStr {
    buf: [u8; MAX_REG_STR],
    len: u8,
}

impl RegStr {
    /// Build from bytes, truncating to [`MAX_REG_STR`].
    pub fn from_bytes(b: &[u8]) -> Self {
        let mut buf = [0u8; MAX_REG_STR];
        let n = b.len().min(MAX_REG_STR);
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

/// A decoded NinoTNC numeric status report. Every field is optional, matching the
/// C# nullable layout — a permissive parse fills in what it can.
///
/// Mirrors `NinoTncStatusFrame` (minus the raw-register bag — see the module docs).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct NinoTncStatusFrame {
    /// Raw firmware-version string from register 00 (e.g. `"3.41"`). Empty if absent.
    pub firmware_version_raw: RegStr,
    /// Firmware version parsed into Nino's two-component form, or `None` when
    /// missing/unparseable.
    pub firmware_version: Option<FirmwareVersion>,
    /// Serial-number / identity string from register 01 (the KAUP8R register).
    /// Empty when unset (all zero bytes) — mirrors the C# `null`.
    pub serial_number: RegStr,
    /// Register 02 — uptime in milliseconds.
    pub uptime_ms: Option<u64>,
    /// Register 03 — the board id / revision number.
    pub board_id: Option<u64>,
    /// Register 04 — DIP switch positions, low four bits. `0b1111` (15) = all four
    /// switches up = "Set from KISS" = software control.
    pub dip_switches: Option<u8>,
    /// Register 06 — the configured-mode identifier byte (resolve through the
    /// catalog, or read [`Self::running_mode`]).
    pub firmware_mode_byte: Option<u8>,
    /// The mode the TNC is currently running, resolved from
    /// [`Self::firmware_mode_byte`] through the catalog. `None` if unknown to the
    /// catalog (firmware likely newer than ours).
    pub running_mode: Option<NinoTncMode>,
    /// Register 07 — AX.25 packets received since boot.
    pub ax25_rx_packets: Option<u64>,
    /// Register 08 — IL2P packets received and corrected (correctable RX) since boot.
    pub il2p_rx_correctable: Option<u64>,
    /// Register 09 — IL2P packets received with uncorrectable errors since boot.
    pub il2p_rx_uncorrectable: Option<u64>,
    /// Register 0A — packets transmitted since boot.
    pub tx_packets: Option<u64>,
    /// Register 0B — preamble word count.
    pub preamble_word_count: Option<u64>,
    /// Register 0C — firmware main-loop cycles since boot.
    pub loop_cycles: Option<u64>,
    /// Register 0D — cumulative PTT-asserted time in milliseconds.
    pub ptt_on_ms: Option<u64>,
    /// Register 0E — cumulative DCD-asserted time in milliseconds.
    pub dcd_on_ms: Option<u64>,
    /// Register 0F — bytes received since boot.
    pub rx_bytes: Option<u64>,
    /// Register 10 — bytes transmitted since boot.
    pub tx_bytes: Option<u64>,
    /// Register 11 — IL2P bytes repaired by FEC since boot.
    pub il2p_fec_corrected_bytes: Option<u64>,
    /// Dropped ADC samples since boot. No numeric register carries this; it is
    /// populated only when the snapshot was mapped from the labelled diagnostic via
    /// [`Self::from_diagnostic`].
    pub lost_adc_samples: Option<u64>,
}

impl NinoTncStatusFrame {
    /// True when the DIP switches read `1111` — the TNC's mode is under software
    /// (KISS SETHW) control rather than pinned by the DIPs. `None` when register 04
    /// was missing. Mirrors `IsSoftwareControlMode`.
    pub fn is_software_control_mode(&self) -> Option<bool> {
        self.dip_switches.map(|d| d == 0x0F)
    }

    /// Try to parse a numeric status report out of a decoded KISS [`Frame`]. Only
    /// succeeds for [`Command::Data`] (mirrors the C# `TryParse(KissFrame, …)`).
    pub fn try_parse_frame(frame: &Frame) -> Option<Self> {
        if frame.command != Command::Data {
            return None;
        }
        Self::try_parse(&frame.payload)
    }

    /// Try to parse a numeric status report out of raw KISS-frame payload bytes.
    /// Scans for the `=00:` marker (the bytes before it are a fake address header),
    /// then walks the `=II:VALUE` sequence. Returns `None` if no marker/register is
    /// found. Total — arbitrary bytes never panic.
    ///
    /// Mirrors `NinoTncStatusFrame.TryParse(ReadOnlySpan<byte>, …)`.
    pub fn try_parse(payload: &[u8]) -> Option<Self> {
        let start = index_of_first_marker(payload)?;
        let mut out = NinoTncStatusFrame::default();
        let mut found_any = false;

        let mut i = start;
        while is_marker_at(payload, i) {
            let register = (hex_value(payload[i + 1]) << 4) | hex_value(payload[i + 2]);
            i += 4;

            if register == SERIAL_NUMBER_REGISTER {
                // Register 01 is eight RAW bytes (which may include '=', ':', or
                // anything else) — take them positionally.
                let take = SERIAL_NUMBER_LENGTH.min(payload.len() - i);
                apply_register(&mut out, register, &payload[i..i + take]);
                i += take;
            } else {
                let mut end = i;
                while end < payload.len() && !is_marker_at(payload, end) {
                    end += 1;
                }
                apply_register(&mut out, register, &payload[i..end]);
                i = end;
            }
            found_any = true;
        }

        if !found_any {
            return None;
        }
        Some(out)
    }

    /// Map a labelled `=FirmwareVr:` diagnostic ([`NinoTncTxTestFrame`]) into this
    /// numeric-report shape. Firmware 3.41 and 3.44 answer GETALL with the labelled
    /// text, which carries a subset of the registers — the fields with no labelled
    /// counterpart (PTT-on, DCD-on, RX/TX bytes, FEC-corrected bytes) stay `None`;
    /// [`Self::lost_adc_samples`] conversely exists only via this labelled path.
    ///
    /// Mirrors `NinoTncStatusFrame.FromDiagnostic`.
    pub fn from_diagnostic(diagnostic: &NinoTncTxTestFrame) -> Self {
        NinoTncStatusFrame {
            firmware_version_raw: RegStr::from_bytes(diagnostic.firmware_version_raw.as_bytes()),
            firmware_version: diagnostic.firmware_version,
            serial_number: RegStr::from_bytes(diagnostic.serial_number.as_bytes()),
            uptime_ms: diagnostic.uptime_ms,
            board_id: diagnostic.board_revision.map(u64::from),
            dip_switches: diagnostic.dip_switch_position,
            firmware_mode_byte: diagnostic.firmware_mode_byte,
            running_mode: diagnostic.running_mode,
            ax25_rx_packets: diagnostic.ax25_rx_packets,
            il2p_rx_correctable: diagnostic.il2p_rx_packets,
            il2p_rx_uncorrectable: diagnostic.il2p_rx_uncorrectable,
            tx_packets: diagnostic.tx_packet_count,
            preamble_word_count: diagnostic.preamble_count,
            loop_cycles: diagnostic.loop_cycles,
            lost_adc_samples: diagnostic.lost_adc_samples,
            ..Default::default()
        }
    }
}

/// Dispatch one parsed register's value slice into the matching typed field.
fn apply_register(out: &mut NinoTncStatusFrame, register: u8, value: &[u8]) {
    match register {
        0x00 => {
            out.firmware_version_raw = RegStr::from_bytes(value);
            out.firmware_version = FirmwareVersion::parse(out.firmware_version_raw.as_str());
        }
        0x01 => out.serial_number = normalise_serial(value),
        0x02 => out.uptime_ms = hex_u64(value),
        0x03 => out.board_id = hex_u64(value),
        0x04 => out.dip_switches = hex_u64(value).map(|d| (d & 0x0F) as u8),
        0x06 => {
            let mode_byte = hex_u64(value).map(|m| (m & 0xFF) as u8);
            out.firmware_mode_byte = mode_byte;
            out.running_mode = mode_byte.and_then(catalog::try_get_by_firmware_byte);
        }
        0x07 => out.ax25_rx_packets = hex_u64(value),
        0x08 => out.il2p_rx_correctable = hex_u64(value),
        0x09 => out.il2p_rx_uncorrectable = hex_u64(value),
        0x0A => out.tx_packets = hex_u64(value),
        0x0B => out.preamble_word_count = hex_u64(value),
        0x0C => out.loop_cycles = hex_u64(value),
        0x0D => out.ptt_on_ms = hex_u64(value),
        0x0E => out.dcd_on_ms = hex_u64(value),
        0x0F => out.rx_bytes = hex_u64(value),
        0x10 => out.tx_bytes = hex_u64(value),
        0x11 => out.il2p_fec_corrected_bytes = hex_u64(value),
        _ => {} // unknown register — no typed field (RawRegisters bag omitted, see module docs)
    }
}

/// Strip the KAUP8R raw value to a printable identity: drop NULs, trim surrounding
/// ASCII whitespace. Empty (all-zero / blank) → an empty [`FieldStr`], mirroring the
/// C# `null`.
fn normalise_serial(raw: &[u8]) -> RegStr {
    let mut buf = [0u8; SERIAL_NUMBER_LENGTH];
    let mut n = 0;
    for &b in raw {
        if b != 0 {
            buf[n] = b;
            n += 1;
        }
    }
    // Trim surrounding ASCII whitespace.
    let mut start = 0;
    let mut end = n;
    while start < end && buf[start].is_ascii_whitespace() {
        start += 1;
    }
    while end > start && buf[end - 1].is_ascii_whitespace() {
        end -= 1;
    }
    RegStr::from_bytes(&buf[start..end])
}

/// Parse an ASCII hex-digit run into `u64`. `None` for an empty slice, any non-hex
/// byte, or an overflowing (> 16 hex-digit) value — matching the C#
/// `long.TryParse(HexNumber)` "returns null on failure" contract.
fn hex_u64(value: &[u8]) -> Option<u64> {
    if value.is_empty() {
        return None;
    }
    let mut acc: u64 = 0;
    for &b in value {
        let nibble = match b {
            b'0'..=b'9' => b - b'0',
            b'A'..=b'F' => b - b'A' + 10,
            b'a'..=b'f' => b - b'a' + 10,
            _ => return None,
        };
        acc = acc.checked_mul(16)?.checked_add(nibble as u64)?;
    }
    Some(acc)
}

/// The index of the first `=00:` marker (the report always starts at register 00;
/// requiring it keeps stray `=` bytes in ordinary traffic from matching), or `None`.
fn index_of_first_marker(payload: &[u8]) -> Option<usize> {
    payload
        .windows(4)
        .position(|w| w == b"=00:")
}

/// True if a `=II:` marker (`=` + two hex digits + `:`) begins at `index`.
fn is_marker_at(payload: &[u8], index: usize) -> bool {
    index + 4 <= payload.len()
        && payload[index] == b'='
        && is_hex(payload[index + 1])
        && is_hex(payload[index + 2])
        && payload[index + 3] == b':'
}

fn is_hex(b: u8) -> bool {
    b.is_ascii_digit() || (b'A'..=b'F').contains(&b) || (b'a'..=b'f').contains(&b)
}

/// Hex-nibble value of a single already-validated hex byte.
fn hex_value(b: u8) -> u8 {
    match b {
        b'0'..=b'9' => b - b'0',
        b'A'..=b'F' => b - b'A' + 10,
        _ => b - b'a' + 10,
    }
}

#[cfg(all(test, feature = "alloc"))]
mod tests {
    use super::*;
    use alloc::vec::Vec;

    /// The verbatim firmware-3.41 capture from the C# doc-comment, with register 01
    /// as eight raw 0xB7 bytes (`·` in the doc) — an unset-but-non-zero identity.
    fn firmware_341_capture() -> Vec<u8> {
        let mut v: Vec<u8> = Vec::new();
        // Fake UI header the parser must skip.
        v.extend_from_slice(b"TNC>USB:");
        v.extend_from_slice(b"=00:3.41");
        v.extend_from_slice(b"=01:");
        v.extend_from_slice(&[0xB7; 8]); // raw identity bytes (may contain '='/':')
        v.extend_from_slice(b"=02:00AC8F08=03:00000004=04:0000000F=06:00000002");
        v.extend_from_slice(b"=07:0000000D=08:00000004=09:00000000=0A:00000049");
        v.extend_from_slice(b"=0B:00000016=0C:0483CA82=0D:0000F4F6=0E:00014DBA");
        v.extend_from_slice(b"=0F:000001E3=10:00000CE4=11:00000000");
        v
    }

    #[test]
    fn parses_the_firmware_341_capture() {
        let s = NinoTncStatusFrame::try_parse(&firmware_341_capture()).unwrap();
        assert_eq!(s.firmware_version_raw.as_str(), "3.41");
        assert_eq!(s.firmware_version, FirmwareVersion::parse("3.41"));
        assert_eq!(s.uptime_ms, Some(0x00AC_8F08));
        assert_eq!(s.board_id, Some(4));
        assert_eq!(s.dip_switches, Some(0x0F));
        assert_eq!(s.is_software_control_mode(), Some(true));
        assert_eq!(s.firmware_mode_byte, Some(0x02));
        assert_eq!(s.running_mode.unwrap().mode, 6, "0x02 → mode 6 (1200 AFSK)");
        assert_eq!(s.ax25_rx_packets, Some(0x0D));
        assert_eq!(s.il2p_rx_correctable, Some(4));
        assert_eq!(s.il2p_rx_uncorrectable, Some(0));
        assert_eq!(s.tx_packets, Some(0x49));
        assert_eq!(s.preamble_word_count, Some(0x16));
        assert_eq!(s.loop_cycles, Some(0x0483_CA82));
        assert_eq!(s.ptt_on_ms, Some(0x0000_F4F6));
        assert_eq!(s.dcd_on_ms, Some(0x0001_4DBA));
        assert_eq!(s.rx_bytes, Some(0x0000_01E3));
        assert_eq!(s.tx_bytes, Some(0x0000_0CE4));
        assert_eq!(s.il2p_fec_corrected_bytes, Some(0));
        // No numeric register carries lost-ADC; only from_diagnostic supplies it.
        assert_eq!(s.lost_adc_samples, None);
    }

    #[test]
    fn register_01_is_taken_positionally_even_when_it_contains_markers() {
        // 8 raw bytes that spell "=05:AB=0" — must be swallowed whole, not re-parsed.
        let mut v: Vec<u8> = Vec::new();
        v.extend_from_slice(b"=00:3.44=01:");
        v.extend_from_slice(b"=05:AB=0");
        v.extend_from_slice(b"=02:00000001");
        let s = NinoTncStatusFrame::try_parse(&v).unwrap();
        assert_eq!(s.serial_number.as_str(), "=05:AB=0");
        assert_eq!(s.uptime_ms, Some(1), "the register after the raw 8 bytes still parses");
    }

    #[test]
    fn all_zero_identity_register_normalises_to_empty() {
        let mut v: Vec<u8> = Vec::new();
        v.extend_from_slice(b"=00:3.44=01:");
        v.extend_from_slice(&[0u8; 8]);
        v.extend_from_slice(b"=02:00000000");
        let s = NinoTncStatusFrame::try_parse(&v).unwrap();
        assert!(s.serial_number.is_empty());
    }

    #[test]
    fn no_marker_is_not_a_status_frame() {
        assert!(NinoTncStatusFrame::try_parse(b"hello world").is_none());
        // A stray '=' that is not the =00: start must not trigger a parse.
        assert!(NinoTncStatusFrame::try_parse(b"a=b=c").is_none());
    }

    #[test]
    fn non_data_command_is_rejected_at_the_frame_level() {
        let frame = Frame::new(0, Command::AckMode, firmware_341_capture());
        assert!(NinoTncStatusFrame::try_parse_frame(&frame).is_none());
        let data = Frame::new(0, Command::Data, firmware_341_capture());
        assert!(NinoTncStatusFrame::try_parse_frame(&data).is_some());
    }

    #[test]
    fn from_diagnostic_carries_labelled_fields_and_lost_adc() {
        // The labelled diagnostic path supplies lost-ADC but leaves byte counters null.
        let diag = NinoTncTxTestFrame::try_parse(
            b"=FirmwareVr:3.44=UptimeMilS:0001A2B3=BrdSwchMod:040F0002\
              =AX25RxPkts:0000007F=LostADCSmp:00000005",
        )
        .unwrap();
        let s = NinoTncStatusFrame::from_diagnostic(&diag);
        assert_eq!(s.firmware_version_raw.as_str(), "3.44");
        assert_eq!(s.uptime_ms, Some(0x0001_A2B3));
        assert_eq!(s.dip_switches, Some(0x0F));
        assert_eq!(s.running_mode.unwrap().mode, 6);
        assert_eq!(s.ax25_rx_packets, Some(0x7F));
        assert_eq!(s.lost_adc_samples, Some(5));
        assert_eq!(s.ptt_on_ms, None, "no labelled counterpart → stays None");
        assert_eq!(s.rx_bytes, None);
    }

    #[test]
    fn arbitrary_bytes_never_panic() {
        for seed in 0u16..512 {
            let bytes: Vec<u8> = (0..40u16)
                .map(|i| (seed.wrapping_mul(31).wrapping_add(i.wrapping_mul(7))) as u8)
                .collect();
            let _ = NinoTncStatusFrame::try_parse(&bytes);
        }
        // Truncated markers at the tail must not index past the end.
        let _ = NinoTncStatusFrame::try_parse(b"=00:");
        let _ = NinoTncStatusFrame::try_parse(b"=00:3=0");
    }
}
