//! The INP3 `L3RTT` link-time-measurement frame — an *ordinary* L3 info datagram,
//! not a new frame family. It is a [`NetRomPacket`] whose destination node
//! callsign is the literal `L3RTT-0`, whose transport opcode nibble is `0x02`, and
//! whose payload is space-padded ASCII carrying the INP3 capability flags (`$N` =
//! "I speak INP3", `$IX` = "I accept IP version X"). The neighbour reflects the
//! frame back verbatim; the originator times the round trip (RTT ÷ 2 → SNTT) —
//! that timing is a later slice. This type is the codec only: a thin
//! **builder + recogniser** over [`NetRomPacket`], reusing [`NetRomNetworkHeader`]
//! (15 B) and [`NetRomTransportHeader`] (5 B) unchanged.
//!
//! The opcode value `0x02` collides numerically with
//! [`NetRomOpcode::ConnectAcknowledge`]; an L3RTT frame is disambiguated by its
//! **destination = `L3RTT-0`**, never by opcode alone — see [`is_l3rtt`]. A frame
//! is recognised as *our own* reflection (vs. a peer's probe we must reflect) when
//! its origin equals our node callsign, because reflection is byte-for-byte echo
//! (origin stays the original prober) — see [`Inp3L3RttFrame::is_reflection_of`].
//!
//! Parsing is total: arbitrary, truncated, or adversarial bytes return `None`,
//! never panic (the §0 totality contract). The capability text is parsed by a
//! width-independent `$`-token scan, so the emitted pad width
//! ([`DEFAULT_CAPABILITY_TEXT_WIDTH`]) is a cosmetic choice (AMBIGUITY-L3RTT-3),
//! not something the recogniser depends on — unknown `$`-tokens are ignored
//! (forward-compat).
//!
//! Ports `Packet.NetRom.Wire.Inp3L3RttFrame`. WIRE PARITY: the byte layouts +
//! hex vectors are locked in `docs/netrom-inp3-i1-wire-spec.md` §1.5 (shared
//! cross-stack golden vectors; the C# reference is authoritative).
//!
//! ### Idiom divergence from the C# / TS reference
//!
//! - **Owned, lifetime-free frame.** The core's [`NetRomPacket`] *borrows* its
//!   payload slice; an `Inp3L3RttFrame` that owned a borrowed packet would carry a
//!   lifetime and could not survive past the input buffer. To match the C#
//!   record's "the frame *is* a packet you can hold and re-serialise" ergonomics
//!   on the M0+, this type stores the two decoded headers + an **owned**
//!   `alloc::vec::Vec<u8>` payload, and rebuilds a borrowing [`NetRomPacket`] on
//!   demand via [`Inp3L3RttFrame::packet`]. `to_bytes` returns an owned `Vec<u8>`
//!   like the C# `byte[]` (matching the `axudp` module's `Vec` idiom).
//! - **`build` returns `Option`, not a throwing constructor.** The C# `Build`
//!   throws `ArgumentOutOfRangeException` for an out-of-range `ipAccept`; the core
//!   never panics on a bad caller arg in a codec path, so [`Inp3L3RttFrame::build`]
//!   returns `None` for an `ip_accept` outside 0..=9 (the encoder stays strict —
//!   it just refuses rather than throws).

extern crate alloc;
use alloc::vec::Vec;

use super::callsign::SHIFTED_LENGTH;
use super::network_header::{NetRomNetworkHeader, DEFAULT_TIME_TO_LIVE};
use super::packet::{NetRomPacket, PACKET_HEADER_LEN};
use super::transport_header::{NetRomTransportHeader, OPCODE_MASK};
use crate::ax25::Callsign;

/// The literal base callsign every L3RTT datagram is destined to.
pub const L3RTT_BASE: &[u8] = b"L3RTT";

/// The canonical SSID of the L3RTT destination (always 0).
pub const L3RTT_SSID: u8 = 0;

/// The transport opcode nibble that marks an L3RTT datagram (0x02). Numerically
/// equal to [`NetRomOpcode::ConnectAcknowledge`](super::transport_header::NetRomOpcode::ConnectAcknowledge);
/// the destination callsign — not this value — is what disambiguates an L3RTT
/// frame from a Connect Acknowledge.
pub const L3RTT_OPCODE: u8 = 0x02;

/// The emitted capability-text field width: `$N` (+ optional `$IX`) right-padded
/// with ASCII spaces to this many octets. The INP3 PDF does not fix the width
/// (AMBIGUITY-L3RTT-3) — the recogniser is width-independent, so this is purely an
/// emit-side default to be calibrated against a live peer in a later slice.
pub const DEFAULT_CAPABILITY_TEXT_WIDTH: usize = 8;

/// A recognised / built L3RTT frame: the two decoded NET/ROM headers, the owned
/// capability-text payload, and the extracted capability flags.
///
/// Owns its payload (see the module-level idiom note) so it has no lifetime and
/// can outlive the input buffer; [`Inp3L3RttFrame::packet`] reconstructs a
/// borrowing [`NetRomPacket`] view for callers that want the packet shape.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Inp3L3RttFrame {
    /// The L3 network header (origin = prober; destination = `L3RTT-0`; TTL).
    pub network: NetRomNetworkHeader,
    /// The L4 transport header (opcode nibble 0x02, other fields zero).
    pub transport: NetRomTransportHeader,
    /// Whether the payload carried the `$N` token — i.e. the far end advertised
    /// INP3 capability.
    pub inp3_capable: bool,
    /// The IP version the far end accepts (the digit from a `$IX` token, e.g. 4
    /// for IPv4), or `None` if no `$IX` token was present.
    pub ip_accept: Option<u8>,
    /// The raw, untrimmed capability-text payload as it appeared on the wire (the
    /// bytes after the 20-octet L3+L4 header).
    pub capability_text: Vec<u8>,
}

impl Inp3L3RttFrame {
    /// Build an L3RTT probe datagram: a [`NetRomPacket`] to `L3RTT-0` with opcode
    /// nibble 0x02 and a space-padded capability text payload (`$N`, then an
    /// optional `$IX`, right-padded to [`DEFAULT_CAPABILITY_TEXT_WIDTH`]). Mirrors
    /// C# `Build` with default `time_to_live` = [`DEFAULT_TIME_TO_LIVE`] and the
    /// default width.
    pub fn build(origin: Callsign, ip_accept: Option<u8>) -> Option<Self> {
        Self::build_with(
            origin,
            ip_accept,
            DEFAULT_TIME_TO_LIVE,
            DEFAULT_CAPABILITY_TEXT_WIDTH,
        )
    }

    /// Build an L3RTT probe with an explicit TTL and capability-text width.
    /// Strict, like every encoder here: it never emits a malformed frame — it
    /// returns `None` (rather than the C# throwing `ArgumentOutOfRangeException`)
    /// when `ip_accept` is not a single decimal digit 0..=9. Any TTL ≥ 1 works for
    /// this single-hop neighbour probe; a width shorter than the tokens leaves them
    /// intact (the tokens are never truncated).
    pub fn build_with(
        origin: Callsign,
        ip_accept: Option<u8>,
        time_to_live: u8,
        capability_text_width: usize,
    ) -> Option<Self> {
        if let Some(v) = ip_accept {
            if v > 9 {
                return None;
            }
        }

        // Build the capability text: "$N", then any "$IX", then right-pad with
        // spaces to the requested width (no padding/truncation if already longer).
        let mut text: Vec<u8> = Vec::new();
        text.push(b'$');
        text.push(b'N');
        if let Some(v) = ip_accept {
            text.push(b'$');
            text.push(b'I');
            text.push(b'0' + v); // v in 0..=9, so a single ASCII digit
        }
        while text.len() < capability_text_width {
            text.push(b' ');
        }

        let destination = Callsign::new(L3RTT_BASE, L3RTT_SSID)?;
        let network = NetRomNetworkHeader {
            origin,
            destination,
            time_to_live,
        };
        let transport = NetRomTransportHeader {
            circuit_index: 0,
            circuit_id: 0,
            tx_sequence: 0,
            rx_sequence: 0,
            opcode: L3RTT_OPCODE,
            flags: 0,
        };

        Some(Self {
            network,
            transport,
            inp3_capable: true,
            ip_accept,
            capability_text: text,
        })
    }

    /// A borrowing [`NetRomPacket`] view over this frame (headers + the owned
    /// capability text as the payload). The C# `Frame.Packet`-equivalent.
    pub fn packet(&self) -> NetRomPacket<'_> {
        NetRomPacket {
            network: self.network,
            transport: self.transport,
            payload: &self.capability_text,
        }
    }

    /// Allocate and return the full L3RTT datagram bytes (the I-frame information
    /// field to send with PID 0xCF) — just the encoded [`NetRomPacket`]. Mirrors
    /// C# `ToBytes`.
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut buf = alloc::vec![0u8; PACKET_HEADER_LEN + self.capability_text.len()];
        // The buffer is sized exactly to the packet, so `encode` cannot fail.
        let n = self.packet().encode(&mut buf).unwrap_or(0);
        buf.truncate(n);
        buf
    }

    /// Try to recognise and decode an L3RTT frame from an interlink I-frame's
    /// information field. Returns `None` (never panics) if the bytes are not a
    /// well-formed [`NetRomPacket`], or are a packet that is not L3RTT (wrong
    /// destination or opcode). On success the capability flags (`$N` →
    /// [`Self::inp3_capable`], `$IX` → [`Self::ip_accept`]) are extracted by a
    /// width-independent token scan of the payload. Mirrors C# `TryParse`.
    pub fn try_parse(info: &[u8]) -> Option<Self> {
        let packet = NetRomPacket::decode(info)?;
        Self::try_from_packet(&packet)
    }

    /// Try to recognise an already-decoded [`NetRomPacket`] as an L3RTT frame and
    /// extract its capability flags. Returns `None` (never panics) if the packet is
    /// not L3RTT. Useful when the caller already decoded the datagram on a shared
    /// receive path and only wants to classify it. Mirrors C# `TryFrom`.
    pub fn try_from_packet(packet: &NetRomPacket<'_>) -> Option<Self> {
        if !is_l3rtt(packet) {
            return None;
        }
        let (inp3_capable, ip_accept) = scan_capabilities(packet.payload);
        Some(Self {
            network: packet.network,
            transport: packet.transport,
            inp3_capable,
            ip_accept,
            capability_text: packet.payload.to_vec(),
        })
    }

    /// Whether this frame is a reflection of *our own* probe (vs. a peer's probe we
    /// are expected to reflect): reflection is verbatim echo, so the origin of a
    /// returning frame is unchanged — it equals our node callsign (§1.4,
    /// AMBIGUITY-L3RTT-4 locks verbatim echo). Mirrors C# `IsReflectionOf`.
    pub fn is_reflection_of(&self, our_node_callsign: &Callsign) -> bool {
        self.network.origin == *our_node_callsign
    }
}

/// Whether an already-decoded [`NetRomPacket`] is an L3RTT frame: its destination
/// decodes to base `L3RTT` (SSID ignored for the match) **and** its transport
/// opcode nibble is `0x02`. The destination test comes first — opcode 0x02 alone is
/// also [`NetRomOpcode::ConnectAcknowledge`](super::transport_header::NetRomOpcode::ConnectAcknowledge).
/// Mirrors C# `IsL3Rtt`.
pub fn is_l3rtt(packet: &NetRomPacket<'_>) -> bool {
    packet.network.destination.base() == L3RTT_BASE
        && (packet.transport.opcode & OPCODE_MASK) == L3RTT_OPCODE
}

/// Scan a capability-text payload for the `$`-prefixed tokens. Width-independent
/// and total — it never panics. `$N` sets `inp3_capable`; a `$IX` with a single
/// decimal digit X sets `ip_accept` (the first such token wins). Unknown
/// `$`-tokens are ignored (forward-compat). Mirrors C# `ScanCapabilities`.
fn scan_capabilities(text: &[u8]) -> (bool, Option<u8>) {
    let mut inp3_capable = false;
    let mut ip_accept: Option<u8> = None;

    let mut i = 0;
    while i < text.len() {
        if text[i] != b'$' {
            i += 1;
            continue;
        }
        // Token = '$' + the following char. Classify by the first byte after '$'.
        let t = i + 1;
        if t >= text.len() {
            break;
        }
        let kind = text[t];
        if kind == b'N' {
            inp3_capable = true;
        } else if kind == b'I'
            && t + 1 < text.len()
            && text[t + 1].is_ascii_digit()
            && ip_accept.is_none()
        {
            ip_accept = Some(text[t + 1] - b'0');
        }
        // Any other '$'-token (unknown capability) is silently ignored.
        i += 1;
    }

    (inp3_capable, ip_accept)
}

// Keep `SHIFTED_LENGTH` referenced so the layout assumption (a callsign field is 7
// octets, so the 20-octet header is `2*7 + 1 + 5`) stays documented next to the
// codec; the wire-spec §1.5 vectors are length 28 = 20 header + 8 payload.
const _: () = assert!(PACKET_HEADER_LEN == 2 * SHIFTED_LENGTH + 1 + 5);

#[cfg(test)]
mod tests {
    //! Round-trip, spec hex-vector, recognition, and totality tests for the L3RTT
    //! codec. Vectors are taken verbatim from `docs/netrom-inp3-i1-wire-spec.md`
    //! §1.5 and are shared cross-stack golden vectors (the C# reference is
    //! authoritative; TS and Rust mirror it 1:1). Ports
    //! `tests/Packet.NetRom.Tests/Wire/Inp3L3RttTests.cs`.

    use super::*;
    use crate::ax25::Callsign;

    fn m0lte() -> Callsign {
        Callsign::parse("M0LTE").unwrap()
    }

    // From §1.5: origin M0LTE-0, dest L3RTT-0, TTL 0x19, transport 00 00 00 00 02.
    const HEADER_PREFIX: [u8; 20] = [
        0x9A, 0x60, 0x98, 0xA8, 0x8A, 0x40, 0x60, // origin M0LTE-0
        0x98, 0x66, 0xA4, 0xA8, 0xA8, 0x40, 0x60, // dest L3RTT-0
        0x19, // TTL = 25
        0x00, 0x00, 0x00, 0x00, 0x02, // transport: opcode 0x02, no flags
    ];

    /// Concatenate the header prefix with the given payload bytes.
    fn with_payload(payload: &[u8]) -> Vec<u8> {
        let mut v = Vec::new();
        v.extend_from_slice(&HEADER_PREFIX);
        v.extend_from_slice(payload);
        v
    }

    // Vector L3RTT-A — probe advertising plain INP3 ("$N      "), length 28.
    fn vector_a() -> Vec<u8> {
        with_payload(&[0x24, 0x4E, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20]) // "$N" + 6 spaces
    }

    // Vector L3RTT-B — probe advertising INP3 + IPv4 ("$N$I4   "), length 28.
    fn vector_b() -> Vec<u8> {
        with_payload(&[0x24, 0x4E, 0x24, 0x49, 0x34, 0x20, 0x20, 0x20]) // "$N$I4" + 3 spaces
    }

    // Vector L3RTT-C — reflection: byte-identical echo of Vector A.
    fn vector_c() -> Vec<u8> {
        vector_a()
    }

    #[test]
    fn build_plain_inp3_probe_matches_spec_vector_a() {
        let frame = Inp3L3RttFrame::build(m0lte(), None).unwrap();

        assert_eq!(frame.to_bytes(), vector_a());
        assert_eq!(frame.to_bytes().len(), 28);
        assert!(frame.inp3_capable);
        assert_eq!(frame.ip_accept, None);
        // default width 8: $N right-padded with six spaces
        assert_eq!(frame.capability_text, b"$N      ");

        // The frame IS a NetRomPacket with the canonical L3RTT shape.
        assert_eq!(frame.network.origin, m0lte());
        assert_eq!(
            frame.network.destination,
            Callsign::new(L3RTT_BASE, 0).unwrap()
        );
        assert_eq!(frame.network.time_to_live, 25);
        assert_eq!(frame.transport.opcode & OPCODE_MASK, 0x02);
        assert_eq!(frame.transport.flags, 0);
    }

    #[test]
    fn build_inp3_plus_ipv4_probe_matches_spec_vector_b() {
        let frame = Inp3L3RttFrame::build(m0lte(), Some(4)).unwrap();

        assert_eq!(frame.to_bytes(), vector_b());
        assert_eq!(frame.to_bytes().len(), 28);
        assert!(frame.inp3_capable);
        assert_eq!(frame.ip_accept, Some(4));
        assert_eq!(frame.capability_text, b"$N$I4   ");
    }

    #[test]
    fn parse_vector_a_extracts_plain_inp3_capability() {
        let frame = Inp3L3RttFrame::try_parse(&vector_a()).unwrap();
        assert!(frame.inp3_capable);
        assert_eq!(frame.ip_accept, None);
        assert_eq!(frame.network.origin, m0lte());
        assert_eq!(frame.network.destination.base(), L3RTT_BASE);
    }

    #[test]
    fn parse_vector_b_extracts_inp3_and_ipv4() {
        let frame = Inp3L3RttFrame::try_parse(&vector_b()).unwrap();
        assert!(frame.inp3_capable);
        // the $I4 token advertises IPv4 acceptance
        assert_eq!(frame.ip_accept, Some(4));
    }

    #[test]
    fn parse_vector_c_recognised_as_our_own_reflection_by_origin() {
        // Verbatim echo: a returning frame keeps the original prober's origin, so
        // the prober recognises its own probe by Origin == self (§1.4).
        let frame = Inp3L3RttFrame::try_parse(&vector_c()).unwrap();
        // origin came back unchanged as M0LTE-0
        assert!(frame.is_reflection_of(&m0lte()));
        // a different node's probe is not ours
        assert!(!frame.is_reflection_of(&Callsign::parse("GB7RDG").unwrap()));
    }

    #[test]
    fn build_then_parse_round_trips_through_bytes() {
        for ip in [None, Some(0u8), Some(4), Some(6), Some(9)] {
            let built = Inp3L3RttFrame::build(m0lte(), ip).unwrap();
            let parsed = Inp3L3RttFrame::try_parse(&built.to_bytes()).unwrap();
            assert!(parsed.inp3_capable);
            assert_eq!(parsed.ip_accept, ip);
            assert_eq!(parsed.network.origin, m0lte());
            assert_eq!(parsed.to_bytes(), built.to_bytes());
        }
    }

    #[test]
    fn capability_text_parse_is_width_independent() {
        // The recogniser scans $-tokens regardless of pad width / contiguity.
        let wide = Inp3L3RttFrame::build_with(m0lte(), Some(4), DEFAULT_TIME_TO_LIVE, 40).unwrap();
        // the payload was padded to the requested width
        assert_eq!(wide.to_bytes()[20..].len(), 40);
        let parsed = Inp3L3RttFrame::try_parse(&wide.to_bytes()).unwrap();
        assert!(parsed.inp3_capable);
        assert_eq!(parsed.ip_accept, Some(4));
    }

    #[test]
    fn capability_text_shorter_than_width_is_not_truncated() {
        // A width smaller than the tokens leaves them intact (no truncation, no pad).
        let frame = Inp3L3RttFrame::build_with(m0lte(), Some(4), DEFAULT_TIME_TO_LIVE, 0).unwrap();
        assert_eq!(frame.capability_text, b"$N$I4");
        let parsed = Inp3L3RttFrame::try_parse(&frame.to_bytes()).unwrap();
        assert!(parsed.inp3_capable);
        assert_eq!(parsed.ip_accept, Some(4));
    }

    #[test]
    fn unknown_dollar_tokens_are_ignored_but_known_ones_still_parse() {
        // Forward-compat: an unknown $-capability between $N and $I4 must not break
        // recognition of the tokens we do understand.
        let payload = b"$N$Z9$I4 "; // "$N$Z9$I4 "
        let info = with_payload(payload);
        let packet = NetRomPacket::decode(&info).unwrap();
        let frame = Inp3L3RttFrame::try_from_packet(&packet).unwrap();
        assert!(frame.inp3_capable);
        assert_eq!(frame.ip_accept, Some(4));
    }

    #[test]
    fn a_packet_without_dollar_n_is_l3rtt_but_not_inp3_capable() {
        // Absence of $N means fall back to vanilla NODES (§1.3) — still an L3RTT
        // frame by destination+opcode, just not advertising INP3.
        let info = with_payload(b"        "); // 8 spaces
        let packet = NetRomPacket::decode(&info).unwrap();
        assert!(is_l3rtt(&packet));
        let frame = Inp3L3RttFrame::try_from_packet(&packet).unwrap();
        assert!(!frame.inp3_capable);
        assert_eq!(frame.ip_accept, None);
    }

    #[test]
    fn non_l3rtt_destination_is_not_recognised() {
        // A real Connect Acknowledge (opcode 0x02) to a normal node must NOT be
        // mistaken for L3RTT — the destination is the discriminator, not the opcode.
        let packet = NetRomPacket {
            network: NetRomNetworkHeader {
                origin: m0lte(),
                destination: Callsign::parse("GB7RDG").unwrap(),
                time_to_live: 25,
            },
            transport: NetRomTransportHeader {
                circuit_index: 1,
                circuit_id: 1,
                tx_sequence: 0,
                rx_sequence: 0,
                opcode: 0x02, // ConnectAcknowledge
                flags: 0,
            },
            payload: &[],
        };

        // opcode 0x02 alone is not L3RTT
        assert!(!is_l3rtt(&packet));
        assert!(Inp3L3RttFrame::try_from_packet(&packet).is_none());
    }

    #[test]
    fn l3rtt_destination_with_wrong_opcode_is_not_recognised() {
        let packet = NetRomPacket {
            network: NetRomNetworkHeader {
                origin: m0lte(),
                destination: Callsign::new(L3RTT_BASE, 0).unwrap(),
                time_to_live: 25,
            },
            transport: NetRomTransportHeader {
                circuit_index: 0,
                circuit_id: 0,
                tx_sequence: 0,
                rx_sequence: 0,
                opcode: 0x05, // Information
                flags: 0,
            },
            payload: &[],
        };

        // opcode nibble must be 0x02
        assert!(!is_l3rtt(&packet));
        assert!(Inp3L3RttFrame::try_from_packet(&packet).is_none());
    }

    #[test]
    fn parse_is_total_on_empty_and_truncated_input() {
        assert!(Inp3L3RttFrame::try_parse(&[]).is_none());
        // a datagram needs the full 20-byte header
        assert!(Inp3L3RttFrame::try_parse(&[0u8; 19]).is_none());
        // an all-zero callsign slot is not a decodable callsign, so the packet
        // itself fails to parse
        assert!(Inp3L3RttFrame::try_parse(&[0u8; 20]).is_none());

        // Truncate Vector A at every length below full — none should panic or
        // succeed past the point the header decodes to a valid L3RTT packet.
        let va = vector_a();
        for len in 0..va.len() {
            // Must never panic.
            let _ = Inp3L3RttFrame::try_parse(&va[..len]);
        }

        // A header-only (payload-empty) L3RTT still parses: no $N → not capable.
        let header_only = Inp3L3RttFrame::try_parse(&va[..20]).unwrap();
        assert!(!header_only.inp3_capable);
    }

    #[test]
    fn parse_is_total_on_garbage() {
        // A tiny deterministic xorshift PRNG (the core is dep-free). The contract is
        // "never panics"; the exact byte stream is incidental — only totality matters.
        let mut state: u32 = 20260607;
        let mut next = || {
            state ^= state << 13;
            state ^= state >> 17;
            state ^= state << 5;
            state
        };
        for _ in 0..20_000 {
            let len = (next() % 64) as usize;
            let mut buf = alloc::vec![0u8; len];
            for b in buf.iter_mut() {
                *b = (next() & 0xFF) as u8;
            }
            // The contract: never panics. Whether it recognises is incidental.
            let _ = Inp3L3RttFrame::try_parse(&buf);
        }
    }

    #[test]
    fn build_rejects_out_of_range_ip_accept() {
        // IP version must be a single decimal digit; the core refuses (returns
        // None) where the C# throws ArgumentOutOfRangeException.
        assert!(Inp3L3RttFrame::build(m0lte(), Some(10)).is_none());
        // (Negative is unrepresentable in the u8-typed Rust API — the analogous
        // "below 0" case can't occur, so only the upper bound is tested.)
    }

    #[test]
    fn build_honours_custom_ttl() {
        let frame =
            Inp3L3RttFrame::build_with(m0lte(), None, 1, DEFAULT_CAPABILITY_TEXT_WIDTH).unwrap();
        // any TTL >= 1 works for the single-hop probe
        assert_eq!(frame.network.time_to_live, 1);
    }
}
