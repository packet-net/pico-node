//! CCDI wire framing (manual §1.8.3). Ports `Packet.Radio.Tait.Ccdi.CcdiFrame`.
//!
//! `[IDENT][SIZE][PARAMETERS][CHECKSUM]<CR>` where IDENT is one ASCII character,
//! SIZE is the PARAMETERS length as two ASCII hex digits, and CHECKSUM is the
//! [`super::checksum`] over everything before it.
//!
//! Unlike the C# `readonly record struct` (which owns a heap `string`), this port
//! keeps the parameters in a fixed `[u8; MAX_PARAMS]` field so the type is
//! allocation-free and `no_std`-friendly — the const-buffer idiom already used by
//! `SerialKissModem`. The two encode entry points mirror the codebase's dual
//! convention: [`CcdiFrame::encode_into`] writes into a caller buffer (the embedded
//! path), and the `alloc`-gated [`CcdiFrame::encode`] returns a `Vec`.

use super::{checksum, hex_upper, parse_hex_u8};

#[cfg(feature = "alloc")]
use alloc::vec::Vec;

/// The largest PARAMETERS a CCDI frame can carry: SIZE is two hex digits, so
/// `0x00..=0xFF` — up to 255 bytes.
pub const MAX_PARAMS: usize = 255;

/// The largest possible received line (CR stripped): IDENT(1) + SIZE(2) +
/// PARAMETERS(255) + CHECKSUM(2). The read-pump line buffer is sized to this.
pub const MAX_LINE: usize = 5 + MAX_PARAMS;

/// A CCDI frame: an IDENT character plus its PARAMETERS. Allocation-free — the
/// parameters live in an inline fixed buffer.
#[derive(Clone)]
pub struct CcdiFrame {
    ident: u8,
    params: [u8; MAX_PARAMS],
    params_len: u8,
}

impl CcdiFrame {
    /// Build a frame from an IDENT and PARAMETERS. Returns `None` if `params`
    /// exceeds [`MAX_PARAMS`] (SIZE can't encode more than 255).
    pub fn new(ident: u8, params: &[u8]) -> Option<Self> {
        if params.len() > MAX_PARAMS {
            return None;
        }
        let mut buf = [0u8; MAX_PARAMS];
        buf[..params.len()].copy_from_slice(params);
        Some(Self {
            ident,
            params: buf,
            params_len: params.len() as u8,
        })
    }

    /// The frame's IDENT character.
    pub fn ident(&self) -> u8 {
        self.ident
    }

    /// The frame's PARAMETERS bytes.
    pub fn params(&self) -> &[u8] {
        &self.params[..self.params_len as usize]
    }

    /// The on-wire byte length of [`Self::encode_into`] (no CR): IDENT + SIZE(2) +
    /// PARAMETERS + CHECKSUM(2).
    pub fn encoded_len(&self) -> usize {
        5 + self.params_len as usize
    }

    /// Render the frame as its on-wire ASCII form (no trailing CR) into `dst`.
    /// Returns the number of bytes written, or `None` if `dst` is too small (size
    /// it with [`Self::encoded_len`]). Mirrors `CcdiFrame.Encode`.
    pub fn encode_into(&self, dst: &mut [u8]) -> Option<usize> {
        let plen = self.params_len as usize;
        let total = 5 + plen;
        if dst.len() < total {
            return None;
        }
        dst[0] = self.ident;
        dst[1] = hex_upper(self.params_len >> 4);
        dst[2] = hex_upper(self.params_len & 0x0F);
        dst[3..3 + plen].copy_from_slice(self.params());
        let body_len = 3 + plen;
        let ck = checksum::compute(&dst[..body_len]);
        dst[body_len] = ck[0];
        dst[body_len + 1] = ck[1];
        Some(total)
    }

    /// Render the frame as transmit-ready bytes (including the trailing CR) into
    /// `dst`. Returns the number of bytes written, or `None` if `dst` is too small
    /// (needs [`Self::encoded_len`] + 1). Mirrors `CcdiFrame.EncodeToBytes`.
    pub fn encode_to_bytes_into(&self, dst: &mut [u8]) -> Option<usize> {
        let n = self.encode_into(dst)?;
        if dst.len() < n + 1 {
            return None;
        }
        dst[n] = b'\r';
        Some(n + 1)
    }

    /// Render the frame's on-wire ASCII form (no CR) as an owned `Vec`. Requires
    /// `alloc`; the convenience twin of [`Self::encode_into`].
    #[cfg(feature = "alloc")]
    pub fn encode(&self) -> Vec<u8> {
        let mut buf = alloc::vec![0u8; self.encoded_len()];
        let n = self.encode_into(&mut buf).expect("buffer sized by encoded_len");
        buf.truncate(n);
        buf
    }

    /// Render the frame as transmit-ready bytes (with the trailing CR) as an owned
    /// `Vec`. Requires `alloc`; mirrors `CcdiFrame.EncodeToBytes`.
    #[cfg(feature = "alloc")]
    pub fn encode_to_bytes(&self) -> Vec<u8> {
        let mut buf = alloc::vec![0u8; self.encoded_len() + 1];
        let n = self
            .encode_to_bytes_into(&mut buf)
            .expect("buffer sized by encoded_len + 1");
        buf.truncate(n);
        buf
    }

    /// Parse one received line (CR already stripped). Returns `None` for anything
    /// whose SIZE doesn't match the actual parameter length or whose checksum fails
    /// — CCDI runs over plain async serial, so line noise is a normal event, not an
    /// error. Mirrors `CcdiFrame.TryParse`.
    pub fn try_parse(line: &[u8]) -> Option<Self> {
        if line.len() < 5 {
            return None;
        }
        let size = parse_hex_u8(&line[1..3])? as usize;
        if line.len() != 5 + size {
            return None;
        }
        let body_len = line.len() - 2;
        if !checksum::is_valid(&line[..body_len], &line[body_len..]) {
            return None;
        }
        // size <= 255 by construction, so `new` cannot fail.
        Self::new(line[0], &line[3..3 + size])
    }
}

impl PartialEq for CcdiFrame {
    fn eq(&self, other: &Self) -> bool {
        self.ident == other.ident && self.params() == other.params()
    }
}

impl Eq for CcdiFrame {}

impl core::fmt::Debug for CcdiFrame {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("CcdiFrame")
            .field("ident", &(self.ident as char))
            .field("params", &core::str::from_utf8(self.params()))
            .finish()
    }
}

#[cfg(all(test, feature = "alloc"))]
mod tests {
    use super::*;

    fn encode_str(ident: u8, params: &[u8]) -> alloc::string::String {
        alloc::string::String::from_utf8(CcdiFrame::new(ident, params).unwrap().encode()).unwrap()
    }

    #[test]
    fn encodes_to_wire_form() {
        // Mirrors CcdiCodecTests.Frame_Encodes_To_Wire_Form.
        assert_eq!(encode_str(b'q', b""), "q002F");
        assert_eq!(encode_str(b'q', b"5064"), "q0450645C");
        assert_eq!(encode_str(b'f', b"91"), "f0291CE");
        assert_eq!(encode_str(b'f', b"041"), "f03041A2");
    }

    #[test]
    fn encode_to_bytes_appends_carriage_return() {
        let bytes = CcdiFrame::new(b'q', b"").unwrap().encode_to_bytes();
        assert_eq!(bytes, b"q002F\r");
    }

    #[test]
    fn parses_valid_lines() {
        // Mirrors CcdiCodecTests.Frame_Parses_Valid_Lines.
        let cases: &[(&[u8], u8, &[u8])] = &[
            (b"j07064-456C9", b'j', b"064-456"), // §1.10.1 raw RSSI -45.6 dBm
            (b"m0813203.02A2", b'm', b"13203.02"), // live TM8110 capture
            (b"p0205C9", b'p', b"05"),           // live capture: receiver busy
        ];
        for (line, ident, params) in cases {
            let frame = CcdiFrame::try_parse(line).expect("valid line");
            assert_eq!(frame.ident(), *ident);
            assert_eq!(frame.params(), *params);
        }
    }

    #[test]
    fn rejects_corrupt_lines() {
        // Mirrors CcdiCodecTests.Frame_Rejects_Corrupt_Lines.
        assert!(CcdiFrame::try_parse(b"").is_none());
        assert!(CcdiFrame::try_parse(b"q002").is_none()); // too short
        assert!(CcdiFrame::try_parse(b"j07064-456C8").is_none()); // checksum off by one
        assert!(CcdiFrame::try_parse(b"j08064-456C9").is_none()); // size ≠ param length
        assert!(CcdiFrame::try_parse(b"jZZ064-456C9").is_none()); // size not hex
    }

    #[test]
    fn round_trips_encode_then_parse() {
        // The load-bearing property: anything encode() emits, try_parse() recovers.
        for (ident, params) in [
            (b'q', &b""[..]),
            (b'q', &b"5064"[..]),
            (b'f', &b"91"[..]),
            (b'a', &b"130520312345678M01D0E"[..]),
            (b's', &b"0800TESTHi!"[..]),
        ] {
            let frame = CcdiFrame::new(ident, params).unwrap();
            let wire = frame.encode();
            let parsed = CcdiFrame::try_parse(&wire).expect("round-trip");
            assert_eq!(parsed, frame);
            assert_eq!(parsed.ident(), ident);
            assert_eq!(parsed.params(), params);
        }
    }

    #[test]
    fn rejects_over_long_params() {
        let too_big = [b'x'; MAX_PARAMS + 1];
        assert!(CcdiFrame::new(b'a', &too_big).is_none());
    }

    #[test]
    fn max_length_params_round_trip() {
        let params = [b'A'; MAX_PARAMS];
        let frame = CcdiFrame::new(b'a', &params).unwrap();
        assert_eq!(frame.encoded_len(), MAX_LINE);
        let wire = frame.encode();
        // SIZE is "FF" for 255 params.
        assert_eq!(&wire[1..3], b"FF");
        assert_eq!(CcdiFrame::try_parse(&wire).unwrap(), frame);
    }
}
