//! AX.25 address slot — the 7-octet on-wire encoding of a callsign + flags.
//!
//! Ports `Packet.Core.Ax25Address`. Per AX.25 v2.2 §3.12.2 each of the 6
//! callsign characters is left-shifted by 1 in its octet; the 7th octet carries
//! the SSID (bits 4-1), the C/H bit (bit 7), and the HDLC extension/end-of-address
//! bit (bit 0). Trailing characters are space-padded.
//!
//! `no_std`, allocation-free: encode/decode operate on `&[u8]` / `&mut [u8]`.

use super::callsign::{Callsign, MAX_BASE_LEN};

/// The fixed encoded length of one address slot.
pub const ADDRESS_LEN: usize = 7;

/// One address slot: a callsign plus the two flag bits AX.25 packs into the SSID
/// octet. `crh` is the command/response (or has-been-repeated) bit; `extension`
/// is the address-field-continues bit (0 on the last slot).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Address {
    /// The callsign in this slot.
    pub callsign: Callsign,
    /// C/H bit (bit 7 of the SSID octet). Command/response on dest+source;
    /// has-been-repeated on a digipeater slot.
    pub crh: bool,
    /// HDLC extension bit (bit 0 of the SSID octet). `false` only on the final
    /// address slot.
    pub extension: bool,
}

impl Address {
    /// Decode a 7-octet address slot. Returns `None` if `src` is too short or a
    /// decoded base byte is not `[A-Z0-9]` after un-shifting (we keep the C#
    /// behaviour of trimming trailing spaces; an interior invalid char fails).
    pub fn decode(src: &[u8]) -> Option<Self> {
        if src.len() < ADDRESS_LEN {
            return None;
        }
        let mut base = [b' '; MAX_BASE_LEN];
        let mut significant = 0usize;
        for i in 0..MAX_BASE_LEN {
            let c = src[i] >> 1; // un-shift: the ASCII char is stored << 1
            base[i] = c;
            if c != b' ' {
                // Track the rightmost non-space so trailing padding is dropped.
                significant = i + 1;
            }
        }
        // Validate the significant prefix is alphanumeric (trailing spaces are pad).
        let trimmed = &base[..significant];
        for &c in trimmed {
            if c != b' ' && !c.is_ascii_uppercase() && !c.is_ascii_digit() {
                return None;
            }
        }
        let ssid_octet = src[6];
        let ssid = (ssid_octet >> 1) & 0x0F;
        let crh = (ssid_octet & 0x80) != 0;
        let extension = (ssid_octet & 0x01) != 0;

        let callsign = Callsign::new(trimmed, ssid)?;
        Some(Self {
            callsign,
            crh,
            extension,
        })
    }

    /// Encode this slot into 7 octets of `dst`. Returns `None` if `dst` is too
    /// small.
    pub fn encode(&self, dst: &mut [u8]) -> Option<()> {
        if dst.len() < ADDRESS_LEN {
            return None;
        }
        let base = self.callsign.base();
        for i in 0..MAX_BASE_LEN {
            let c = if i < base.len() { base[i] } else { b' ' };
            dst[i] = c << 1;
        }
        // SSID octet: bits 6-5 are the reserved "11" pattern (0x60), SSID in bits
        // 4-1, C/H in bit 7, extension in bit 0 — matches Packet.Core.Ax25Address.
        let mut ssid_octet = 0x60 | ((self.callsign.ssid() & 0x0F) << 1);
        if self.crh {
            ssid_octet |= 0x80;
        }
        if self.extension {
            ssid_octet |= 0x01;
        }
        dst[6] = ssid_octet;
        Some(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cs(s: &str) -> Callsign {
        Callsign::parse(s).unwrap()
    }

    #[test]
    fn encode_then_decode_round_trips() {
        let a = Address {
            callsign: cs("M0LTE-7"),
            crh: true,
            extension: false,
        };
        let mut buf = [0u8; ADDRESS_LEN];
        a.encode(&mut buf).unwrap();
        let b = Address::decode(&buf).unwrap();
        assert_eq!(a, b);
    }

    #[test]
    fn chars_are_left_shifted() {
        let a = Address {
            callsign: cs("A"),
            crh: false,
            extension: true,
        };
        let mut buf = [0u8; ADDRESS_LEN];
        a.encode(&mut buf).unwrap();
        assert_eq!(buf[0], b'A' << 1); // 0x82
        // Positions 1..6 are space-padded => 0x40.
        assert_eq!(buf[1], b' ' << 1);
    }

    #[test]
    fn ssid_octet_layout() {
        let a = Address {
            callsign: cs("G7XYZ-5"),
            crh: true,
            extension: true,
        };
        let mut buf = [0u8; ADDRESS_LEN];
        a.encode(&mut buf).unwrap();
        let ssid_octet = buf[6];
        assert_eq!((ssid_octet >> 1) & 0x0F, 5); // SSID
        assert_ne!(ssid_octet & 0x80, 0); // C/H set
        assert_ne!(ssid_octet & 0x01, 0); // extension set
        assert_eq!(ssid_octet & 0x60, 0x60); // reserved bits
    }

    #[test]
    fn decode_trims_trailing_space_padding() {
        // "M0LTE " encoded then decoded yields base "M0LTE" (5 chars).
        let a = Address {
            callsign: cs("M0LTE"),
            crh: false,
            extension: false,
        };
        let mut buf = [0u8; ADDRESS_LEN];
        a.encode(&mut buf).unwrap();
        let b = Address::decode(&buf).unwrap();
        assert_eq!(b.callsign.base(), b"M0LTE");
    }

    #[test]
    fn decode_rejects_short_input() {
        assert!(Address::decode(&[0u8; 6]).is_none());
    }

    #[test]
    fn known_vector_matches_spec_shape() {
        // "M0LTE" with SSID 0, last address, command bit set.
        let a = Address {
            callsign: cs("M0LTE"),
            crh: true,
            extension: false,
        };
        let mut buf = [0u8; ADDRESS_LEN];
        a.encode(&mut buf).unwrap();
        assert_eq!(
            &buf[..6],
            &[b'M' << 1, b'0' << 1, b'L' << 1, b'T' << 1, b'E' << 1, b' ' << 1]
        );
        // SSID octet: 0x60 | (0<<1) | 0x80 (crh) | 0 (ext clear) = 0xE0.
        assert_eq!(buf[6], 0xE0);
    }
}
