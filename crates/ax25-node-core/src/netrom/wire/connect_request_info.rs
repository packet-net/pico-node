//! Codec for the information field of a NET/ROM L4 Connect Request (opcode 0x01)
//! — the one transport message whose info field has a defined structure. It
//! conveys the proposed send-window and the originating user + node callsigns
//! end-to-end.
//!
//! Wire layout (the de-facto NET/ROM form), 15 octets:
//! ```text
//!   [1] proposed send-window size (1..127)
//!   [7] originating user callsign  (AX.25 shifted form)
//!   [7] originating node callsign  (AX.25 shifted form)
//!   (any trailing octets are an implementation extension — e.g. LinBPQ appends a
//!    timeout/flags pair — and are ignored on parse)
//! ```
//!
//! The window lives in the *info field*, not the transport header's TX-sequence
//! slot (verified on the wire against real LinBPQ — the packet.net #308/#309
//! lesson). Construction is strict (always the canonical 15-octet form); parsing
//! is total and tolerant of trailing extension octets. Ports
//! `Packet.NetRom.Wire.ConnectRequestInfo`. `no_std`, allocation-free.

use super::callsign::{try_read_shifted, write_shifted, SHIFTED_LENGTH};
use crate::ax25::Callsign;

/// Octets the canonical Connect Request info field occupies (window + two shifted
/// callsigns). A peer may append extension octets after these.
pub const CONNECT_REQUEST_INFO_LEN: usize = 1 + SHIFTED_LENGTH + SHIFTED_LENGTH; // 15

/// Octets in the LinBPQ "extended connect" form: the canonical 15 plus a 2-octet
/// trailer carrying the proposed session timer (T1, little-endian) — and, in the
/// high byte of that timer, the BPQ compression-supported bit. Gated behind
/// `netrom-compress`. Mirrors C# `ConnectRequestInfo.ExtendedLength`.
#[cfg(feature = "netrom-compress")]
pub const CONNECT_REQUEST_INFO_EXTENDED_LEN: usize = CONNECT_REQUEST_INFO_LEN + 2; // 17

/// The BPQ "compression supported" bit, OR-ed into the **high** byte of the
/// trailing T1 timer of an extended Connect Request (LinBPQ `L4Code.c`:
/// `MSG->L4DATA[16] |= 0x40`). The receiver masks it off (`BPQPARAMS[1] &= 0xf`)
/// before reading the timer. Mirrors C# `ConnectRequestInfo.CompressBit`.
#[cfg(feature = "netrom-compress")]
pub const CONNECT_REQUEST_COMPRESS_BIT: u8 = 0x40;

/// The parsed Connect Request info: the proposed window + the originating
/// user/node callsigns.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConnectRequestInfo {
    /// Proposed send-window size (1..127).
    pub proposed_window: u8,
    /// The end user the circuit is opened on behalf of.
    pub originating_user: Callsign,
    /// The originating node.
    pub originating_node: Callsign,
}

impl ConnectRequestInfo {
    /// Build the canonical 15-octet Connect Request info field into the front of
    /// `dst`: proposed window, then the originating user + node (both shifted).
    /// Returns `None` only if `dst` is too short.
    pub fn encode(&self, dst: &mut [u8]) -> Option<()> {
        if dst.len() < CONNECT_REQUEST_INFO_LEN {
            return None;
        }
        dst[0] = self.proposed_window;
        write_shifted(&self.originating_user, &mut dst[1..])?;
        write_shifted(&self.originating_node, &mut dst[1 + SHIFTED_LENGTH..])?;
        Some(())
    }

    /// Build the LinBPQ **extended** Connect Request info field into the front of
    /// `dst` (≥ [`CONNECT_REQUEST_INFO_EXTENDED_LEN`]): the canonical 15 octets
    /// followed by the 2-octet T1 timer trailer, with the compression-supported bit
    /// ([`CONNECT_REQUEST_COMPRESS_BIT`]) OR-ed into the timer's high byte when
    /// `offer_compression` is set. This is the exact shape LinBPQ both originates and
    /// parses. A peer that ignores the trailer (vanilla NET/ROM, or pico with
    /// compression off) simply sees a normal Connect Request (the trailer is beyond
    /// the 15 octets [`decode`](Self::decode) reads). Returns `None` only if `dst` is
    /// too short. Mirrors C# `ConnectRequestInfo.BuildExtended`.
    #[cfg(feature = "netrom-compress")]
    pub fn encode_extended(
        &self,
        dst: &mut [u8],
        timer_seconds: u16,
        offer_compression: bool,
    ) -> Option<()> {
        if dst.len() < CONNECT_REQUEST_INFO_EXTENDED_LEN {
            return None;
        }
        dst[0] = self.proposed_window;
        write_shifted(&self.originating_user, &mut dst[1..])?;
        write_shifted(&self.originating_node, &mut dst[1 + SHIFTED_LENGTH..])?;
        dst[CONNECT_REQUEST_INFO_LEN] = (timer_seconds & 0xFF) as u8; // T1 low
        let mut hi = ((timer_seconds >> 8) & 0x0F) as u8; // T1 high — only low nibble is the timer
        if offer_compression {
            hi |= CONNECT_REQUEST_COMPRESS_BIT;
        }
        dst[CONNECT_REQUEST_INFO_LEN + 1] = hi;
        Some(())
    }

    /// Read the BPQ compression-supported bit from a Connect Request info field, if
    /// the peer sent the extended (≥ 17-octet) form. Returns `false` for the
    /// canonical 15-octet form (no trailer ⇒ no offer). Mirrors C#
    /// `ConnectRequestInfo.OffersCompression`.
    #[cfg(feature = "netrom-compress")]
    pub fn offers_compression(info: &[u8]) -> bool {
        info.len() >= CONNECT_REQUEST_INFO_EXTENDED_LEN
            && (info[CONNECT_REQUEST_INFO_LEN + 1] & CONNECT_REQUEST_COMPRESS_BIT) != 0
    }

    /// Parse the proposed window + originating user/node from a Connect Request
    /// info field. Total: returns `None` if the field is shorter than the 15-octet
    /// canonical layout or a callsign is undecodable. Trailing octets beyond the
    /// 15 (a peer's extension) are ignored. Mirrors C# `TryParse`.
    pub fn decode(info: &[u8]) -> Option<Self> {
        if info.len() < CONNECT_REQUEST_INFO_LEN {
            return None;
        }
        let originating_user = try_read_shifted(&info[1..])?;
        let originating_node = try_read_shifted(&info[1 + SHIFTED_LENGTH..])?;
        Some(Self {
            proposed_window: info[0],
            originating_user,
            originating_node,
        })
    }
}

#[cfg(all(test, feature = "netrom-compress"))]
mod extended_tests {
    use super::*;
    use crate::ax25::Callsign;

    fn cs(b: &[u8]) -> Callsign {
        Callsign::new(b, 0).unwrap()
    }

    #[test]
    fn extended_form_is_17_octets_and_offers_when_asked() {
        let cri = ConnectRequestInfo {
            proposed_window: 4,
            originating_user: cs(b"M0LTE"),
            originating_node: cs(b"GB7RDG"),
        };
        let mut buf = [0u8; CONNECT_REQUEST_INFO_EXTENDED_LEN];
        cri.encode_extended(&mut buf, 60, true).unwrap();

        // Canonical 15-octet prefix round-trips through the (unchanged) decoder.
        let parsed = ConnectRequestInfo::decode(&buf).unwrap();
        assert_eq!(parsed, cri);

        // T1 trailer: low byte = 60, high byte's low nibble = 0, compress bit set.
        assert_eq!(buf[CONNECT_REQUEST_INFO_LEN], 60);
        assert_eq!(buf[CONNECT_REQUEST_INFO_LEN + 1] & 0x0F, 0);
        assert!(ConnectRequestInfo::offers_compression(&buf));
    }

    #[test]
    fn extended_without_offer_clears_the_bit() {
        let cri = ConnectRequestInfo {
            proposed_window: 4,
            originating_user: cs(b"M0LTE"),
            originating_node: cs(b"GB7RDG"),
        };
        let mut buf = [0u8; CONNECT_REQUEST_INFO_EXTENDED_LEN];
        cri.encode_extended(&mut buf, 0x123, false).unwrap();
        // Timer 0x123 → low nibble of high byte carries 0x1, no compress bit.
        assert_eq!(buf[CONNECT_REQUEST_INFO_LEN], 0x23);
        assert_eq!(buf[CONNECT_REQUEST_INFO_LEN + 1], 0x01);
        assert!(!ConnectRequestInfo::offers_compression(&buf));
    }

    #[test]
    fn canonical_15_octet_form_offers_nothing() {
        let cri = ConnectRequestInfo {
            proposed_window: 4,
            originating_user: cs(b"M0LTE"),
            originating_node: cs(b"GB7RDG"),
        };
        let mut buf = [0u8; CONNECT_REQUEST_INFO_LEN];
        cri.encode(&mut buf).unwrap();
        assert!(!ConnectRequestInfo::offers_compression(&buf));
    }
}
