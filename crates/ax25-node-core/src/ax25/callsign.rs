//! An amateur-radio callsign with an optional SSID (0–15).
//!
//! Ports the host-relevant parts of `Packet.Core.Callsign`. The base callsign is
//! 0–6 uppercase ASCII alphanumerics (the form AX.25 can encode). User-facing
//! text parsing ([`Callsign::parse`]) is strict (≥1 char), matching the C#
//! `TryParse` contract; the all-spaces / empty base is reachable only via the
//! address decoder for on-wire frames that legitimately carry a blank slot.
//!
//! `no_std`: a callsign is a fixed 6-byte buffer plus a length and an SSID — no
//! heap. Fits the embedded target directly.

/// Maximum base-callsign length AX.25 can encode.
pub const MAX_BASE_LEN: usize = 6;

/// A parsed callsign: up to 6 uppercase alphanumerics + an SSID (0–15).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Callsign {
    /// Base callsign bytes, uppercase A–Z / 0–9, left-justified.
    base: [u8; MAX_BASE_LEN],
    /// Number of significant bytes in `base` (0–6).
    len: u8,
    /// Secondary station identifier, 0–15.
    ssid: u8,
}

impl Callsign {
    /// Build from parts. Returns `None` if the base is too long, contains a
    /// non-`[A-Z0-9]` byte, or the SSID exceeds 15. An empty base is permitted
    /// here (the decoder uses it); text parsing is strict — see [`Callsign::parse`].
    pub fn new(base: &[u8], ssid: u8) -> Option<Self> {
        if base.len() > MAX_BASE_LEN || ssid > 15 {
            return None;
        }
        let mut buf = [b' '; MAX_BASE_LEN];
        for (i, &b) in base.iter().enumerate() {
            if !b.is_ascii_uppercase() && !b.is_ascii_digit() {
                return None;
            }
            buf[i] = b;
        }
        Some(Self {
            base: buf,
            len: base.len() as u8,
            ssid,
        })
    }

    /// Parse user-typed text like `"M0LTE-1"` or `"G7XYZ"`. Strict: the base must
    /// be 1–6 `[A-Za-z0-9]` chars (lower-cased input is upper-cased), the optional
    /// `-SSID` must be 0–15. Returns `None` on any violation. Mirrors the C#
    /// `Callsign.TryParse` (which is the path used for `Connect <call>`).
    pub fn parse(text: &str) -> Option<Self> {
        let text = text.trim();
        if text.is_empty() {
            return None;
        }
        let (base_str, ssid) = match text.split_once('-') {
            Some((b, s)) => {
                let n: u8 = s.parse().ok()?;
                if n > 15 {
                    return None;
                }
                (b, n)
            }
            None => (text, 0),
        };
        if base_str.is_empty() || base_str.len() > MAX_BASE_LEN {
            return None;
        }
        let mut buf = [b' '; MAX_BASE_LEN];
        for (i, c) in base_str.bytes().enumerate() {
            let up = c.to_ascii_uppercase();
            if !up.is_ascii_uppercase() && !up.is_ascii_digit() {
                return None;
            }
            buf[i] = up;
        }
        Some(Self {
            base: buf,
            len: base_str.len() as u8,
            ssid,
        })
    }

    /// The significant base bytes (no padding), e.g. `b"M0LTE"`.
    pub fn base(&self) -> &[u8] {
        &self.base[..self.len as usize]
    }

    /// The SSID (0–15).
    pub fn ssid(&self) -> u8 {
        self.ssid
    }

    /// Write the canonical text form into `dst` (e.g. `"M0LTE-1"`; SSID 0 is
    /// omitted). Returns the number of bytes written, or `None` if `dst` is too
    /// small. Allocation-free so it works on the embedded target.
    pub fn write_display(&self, dst: &mut [u8]) -> Option<usize> {
        let b = self.base();
        let mut n = 0;
        if dst.len() < b.len() {
            return None;
        }
        dst[..b.len()].copy_from_slice(b);
        n += b.len();
        if self.ssid != 0 {
            // "-" then 1–2 digits.
            let mut tmp = [0u8; 3];
            tmp[0] = b'-';
            let s = self.ssid;
            let written = if s >= 10 {
                tmp[1] = b'0' + s / 10;
                tmp[2] = b'0' + s % 10;
                3
            } else {
                tmp[1] = b'0' + s;
                2
            };
            if dst.len() < n + written {
                return None;
            }
            dst[n..n + written].copy_from_slice(&tmp[..written]);
            n += written;
        }
        Some(n)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_with_ssid() {
        let c = Callsign::parse("M0LTE-1").unwrap();
        assert_eq!(c.base(), b"M0LTE");
        assert_eq!(c.ssid(), 1);
    }

    #[test]
    fn parses_without_ssid_defaults_zero() {
        let c = Callsign::parse("G7XYZ").unwrap();
        assert_eq!(c.base(), b"G7XYZ");
        assert_eq!(c.ssid(), 0);
    }

    #[test]
    fn lowercases_are_upcased() {
        let c = Callsign::parse("m0lte").unwrap();
        assert_eq!(c.base(), b"M0LTE");
    }

    #[test]
    fn rejects_empty() {
        assert!(Callsign::parse("").is_none());
        assert!(Callsign::parse("   ").is_none());
    }

    #[test]
    fn rejects_too_long_base() {
        assert!(Callsign::parse("ABCDEFG").is_none());
    }

    #[test]
    fn rejects_ssid_over_15() {
        assert!(Callsign::parse("M0LTE-16").is_none());
    }

    #[test]
    fn rejects_bad_ssid_text() {
        assert!(Callsign::parse("M0LTE-x").is_none());
    }

    #[test]
    fn rejects_punctuation_in_base() {
        assert!(Callsign::parse("M0.LTE").is_none());
    }

    #[test]
    fn write_display_round_trips() {
        let c = Callsign::parse("M0LTE-15").unwrap();
        let mut buf = [0u8; 16];
        let n = c.write_display(&mut buf).unwrap();
        assert_eq!(&buf[..n], b"M0LTE-15");
    }

    #[test]
    fn write_display_omits_zero_ssid() {
        let c = Callsign::parse("G7XYZ").unwrap();
        let mut buf = [0u8; 16];
        let n = c.write_display(&mut buf).unwrap();
        assert_eq!(&buf[..n], b"G7XYZ");
    }
}
