//! Decoders for the two callsign/text encodings a NET/ROM NODES broadcast uses.
//!
//! Ports `Packet.NetRom.Wire.NetRomCallsign`. NET/ROM rides on AX.25, so a
//! *callsign* field is the familiar 7-octet AX.25 shifted form (6 chars
//! left-shifted by one, plus the SSID byte) — but a node's 6-character *alias /
//! mnemonic* is plain space-padded ASCII, not shifted, and has no SSID octet.
//!
//! The 7-octet decode delegates to [`crate::ax25::Address::decode`] — the same
//! shifted-callsign codec the frame layer uses — so there is one source of truth
//! for the shift/SSID/end-of-address semantics.
//!
//! `no_std`, allocation-free: the alias is returned as a fixed-capacity [`Alias`]
//! value, not a heap `String`.

use crate::ax25::{Address, Callsign};

/// Octets occupied by an AX.25 shifted callsign field (with SSID byte).
pub const SHIFTED_LENGTH: usize = crate::ax25::ADDRESS_LEN; // 7

/// Octets occupied by a NET/ROM alias / mnemonic field (plain ASCII, no SSID).
pub const ALIAS_LENGTH: usize = 6;

/// A NET/ROM alias / mnemonic — up to 6 printable-ASCII chars, trailing spaces
/// trimmed. A fixed-capacity, `Copy` value (no heap `String`) so it embeds in the
/// routing-table entries directly on the M0+.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Alias {
    buf: [u8; ALIAS_LENGTH],
    len: u8,
}

impl Alias {
    /// The empty alias (a blank / absent mnemonic field).
    pub const EMPTY: Self = Self {
        buf: [0; ALIAS_LENGTH],
        len: 0,
    };

    /// Build an alias from a `&str`, keeping only the leading printable-ASCII chars
    /// (up to 6) — used by tests and any caller building an alias from text. Mirrors
    /// the lossy display semantics of [`read_alias`].
    pub fn from_str_lossy(s: &str) -> Self {
        let mut buf = [0u8; ALIAS_LENGTH];
        let mut len = 0usize;
        for &b in s.as_bytes() {
            if len >= ALIAS_LENGTH {
                break;
            }
            if (b' '..=b'~').contains(&b) {
                buf[len] = b;
                len += 1;
            }
        }
        // Trim trailing spaces to match the wire decoder.
        while len > 0 && buf[len - 1] == b' ' {
            len -= 1;
        }
        // Canonicalise the unused tail (see `read_alias`) so equal aliases compare
        // equal regardless of construction path.
        for slot in &mut buf[len..] {
            *slot = 0;
        }
        Self {
            buf,
            len: len as u8,
        }
    }

    /// The significant alias bytes (no trailing padding). Empty for a blank alias.
    pub fn as_bytes(&self) -> &[u8] {
        &self.buf[..self.len as usize]
    }

    /// The alias as a `&str` (always valid UTF-8 — only printable ASCII is kept).
    pub fn as_str(&self) -> &str {
        // SAFETY-free: `read_alias`/`from_str_lossy` only ever store bytes in
        // 0x20..=0x7E, which is valid UTF-8. Fall back to "" if that ever changes.
        core::str::from_utf8(self.as_bytes()).unwrap_or("")
    }

    /// True if the alias is the empty string.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
}

/// Decode a 7-octet AX.25 shifted callsign field (callsign chars in the upper 7
/// bits, SSID + flags in the 7th octet). The end-of-address / command bits in the
/// SSID octet are read but not significant here — inside a NODES entry these
/// fields are payload, not an AX.25 address chain.
///
/// Returns `None` if the span is too short or the field is not a decodable
/// callsign. An all-space ("empty") base is permitted (some nodes pad an absent
/// best-neighbour slot) — it decodes to a zero-length-base callsign, matching the
/// C# lenient path; the routing-table builder decides what an empty callsign means.
pub fn try_read_shifted(source: &[u8]) -> Option<Callsign> {
    if source.len() < SHIFTED_LENGTH {
        return None;
    }
    // `Address::decode` already trims trailing-space padding and permits an
    // all-space base (Callsign::new accepts a zero-length base), exactly the C#
    // `Ax25ParseOptions.Lenient` behaviour `NetRomCallsign.TryReadShifted` relies on.
    Address::decode(&source[..SHIFTED_LENGTH]).map(|addr| addr.callsign)
}

/// Decode a 6-octet NET/ROM alias / mnemonic field: plain ASCII, space-padded on
/// the right, no shift and no SSID. Trailing spaces are stripped; an all-space
/// field yields the empty alias. Non-printable octets are dropped (a noisy link
/// can corrupt a byte) so the result is always a clean display string.
pub fn read_alias(source: &[u8]) -> Alias {
    if source.len() < ALIAS_LENGTH {
        return Alias::EMPTY;
    }
    let mut buf = [0u8; ALIAS_LENGTH];
    let mut len = 0usize;
    for &b in &source[..ALIAS_LENGTH] {
        // Printable ASCII only (0x20..=0x7E). Anything else (a corrupted or
        // high-bit octet) is skipped rather than rendered as mojibake.
        if (b' '..=b'~').contains(&b) {
            buf[len] = b;
            len += 1;
        }
    }
    // Trim trailing spaces (TrimEnd).
    while len > 0 && buf[len - 1] == b' ' {
        len -= 1;
    }
    // Zero the unused tail so the fixed buffer is canonical: an alias decoded from
    // a space-padded wire field must compare equal (under the derived `PartialEq`)
    // to the same alias built by `from_str_lossy` — they otherwise differ only in
    // the don't-care bytes past `len`.
    for slot in &mut buf[len..] {
        *slot = 0;
    }
    Alias {
        buf,
        len: len as u8,
    }
}

/// Encode a [`Callsign`] into a 7-octet AX.25 shifted callsign field — the inverse
/// of [`try_read_shifted`]. The command/response and end-of-address bits are
/// cleared: inside a NODES entry or a NET/ROM header these fields are *payload*,
/// not an AX.25 address-chain link. Delegates to [`Address::encode`] so the
/// shift/SSID encoding has one source of truth with the frame layer, matching C#
/// `NetRomCallsign.WriteShifted` (`CrhBit`/`ExtensionBit` false) byte-for-byte.
/// Returns `None` only if `dst` has fewer than [`SHIFTED_LENGTH`] octets.
pub fn write_shifted(callsign: &Callsign, dst: &mut [u8]) -> Option<()> {
    Address {
        callsign: *callsign,
        crh: false,
        extension: false,
    }
    .encode(dst)
}

/// Encode a NET/ROM alias / mnemonic into a 6-octet field — the inverse of
/// [`read_alias`]: plain ASCII, right-padded with spaces, no shift and no SSID.
/// (An [`Alias`] only ever holds printable, trailing-trimmed ASCII, so the field
/// is canonical by construction; mirrors C# `NetRomCallsign.WriteAlias`.) Returns
/// `None` only if `dst` has fewer than [`ALIAS_LENGTH`] octets.
pub fn write_alias(alias: &Alias, dst: &mut [u8]) -> Option<()> {
    if dst.len() < ALIAS_LENGTH {
        return None;
    }
    let bytes = alias.as_str().as_bytes();
    for (i, slot) in dst[..ALIAS_LENGTH].iter_mut().enumerate() {
        *slot = if i < bytes.len() { bytes[i] } else { b' ' };
    }
    Some(())
}
