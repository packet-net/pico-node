//! Codec for the INP3 Routing Information Frame (RIF) and its constituents — the
//! `0xFF`-signed routing-information body carried in the information field of a
//! connected-mode interlink I-frame (PID 0xCF):
//!
//! - [`Inp3Tlv`] — one type/length/value record inside a RIP (alias / IP /
//!   unknown-retained-verbatim).
//! - [`Inp3Rip`] — one Routing Information Packet (a single routing entry).
//! - [`Inp3Rif`] — the whole `0xFF`-signed frame: a signature byte followed by
//!   one or more RIPs, each self-delimited by its `0x00` EOP.
//!
//! Ports `Packet.NetRom.Wire.Inp3Rif` / `Inp3Rip` / `Inp3Tlv` / `Inp3ParseOptions`
//! on the C# side. A RIF is the connected-mode analogue of a NODES broadcast —
//! both lead with the `0xFF` signature, both are a self-delimited sequence of
//! fixed-prefix entries — so this module mirrors [`super::broadcast`]'s shape:
//! total / never-panic parsing, a lenient-by-default [`Inp3ParseOptions`] surface
//! with the same preset names (`STRICT` / `LENIENT` / `BPQ` / `XROUTER`), and the
//! shifted-callsign codec reused from [`super::callsign`] so the shift/SSID
//! semantics have one home.
//!
//! Byte layouts and every hex vector here are LOCKED in
//! `docs/netrom-inp3-i1-wire-spec.md` §2 (packet.net). Wire parity against that
//! document is the correctness gate; the same vectors are asserted in the merged
//! `@packet-net/ax25` test `tests/netrom/inp3-rif.test.ts`.
//!
//! `no_std` + `alloc`: the per-RIP TLV list and the per-RIF RIP list are
//! [`alloc::vec::Vec`] (a RIF can in principle carry many variable-length RIPs, so
//! a fixed inline cap would either over-allocate or clip). Target-time maths is
//! integer-only (no FPU on the RP2040 M0+). Parsers borrow the source slice;
//! `to_bytes` allocates the wire encoding.

use alloc::vec::Vec;

use super::callsign::{try_read_shifted, write_shifted, SHIFTED_LENGTH};
use crate::ax25::Callsign;

// ─── Parse options (mirror NetRomParseOptions / Inp3ParseOptions) ───────────

/// Per-call configuration for the INP3 RIF wire-parse path
/// ([`Inp3Rif::try_parse_with`]). Mirrors `NetRomParseOptions` one-for-one: each
/// tolerance of a real-world peer's divergence from the canonical INP3 wire
/// format is a named, individually-toggleable flag with the same preset surface
/// (`STRICT` / `LENIENT` / `BPQ` / `XROUTER`).
///
/// A RIF is the connected-mode analogue of a NODES broadcast — both lead with the
/// `0xFF` signature, both are a self-delimited sequence of fixed-prefix entries —
/// so the strict-by-default / lenient-on-promiscuous-ingest discipline is
/// identical. The two currently-known divergences are about *tolerance of the
/// entry list* (an empty list, a clipped trailing RIP), not the field layout,
/// exactly as for NODES.
///
/// Mirrors `Packet.NetRom.Wire.Inp3ParseOptions` on the C# side.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Inp3ParseOptions {
    /// Accept a RIF body carrying *zero* RIPs (just the `0xFF` signature). The
    /// connected-mode analogue of `NetRomParseOptions::allow_empty_destination_list`.
    ///
    /// A neighbour with nothing new to advertise can in principle send a
    /// signature-only RIF. Default `true` (lenient); a strict caller can treat a
    /// contentless RIF as malformed.
    pub allow_empty_rip_list: bool,

    /// Accept a RIF whose final RIP is truncated (the body ends mid-RIP, or a
    /// TLV's claimed length runs off the end of the body): keep every whole RIP
    /// parsed so far and drop the clipped tail. The RIF analogue of
    /// `NetRomParseOptions::allow_trailing_partial_entry`.
    ///
    /// Driver: a noisy RF interlink can clip the tail of an I-frame. Dropping
    /// every learned route because the *last* RIP is short would be hostile; we
    /// keep the whole RIPs we did parse. Default `true` (lenient). Under
    /// [`STRICT`](Self::STRICT) any leftover byte that does not complete a RIP
    /// rejects the whole frame.
    pub allow_trailing_partial_rip: bool,
}

impl Inp3ParseOptions {
    /// Accept-everything mode. All currently-known accommodations enabled. The
    /// convenience [`Inp3Rif::try_parse`] path uses this — read-only promiscuous
    /// ingest wants to be forgiving.
    pub const LENIENT: Self = Self {
        allow_empty_rip_list: true,
        allow_trailing_partial_rip: true,
    };

    /// Strict canonical INP3 — every accommodation disabled. A RIF is accepted
    /// only if every byte after the signature forms a whole RIP and there is at
    /// least one RIP.
    pub const STRICT: Self = Self {
        allow_empty_rip_list: false,
        allow_trailing_partial_rip: false,
    };

    /// BPQ / LinBPQ-flavoured leniency. Today identical to [`LENIENT`](Self::LENIENT);
    /// kept named so a future BPQ-specific INP3 quirk lands here without churning
    /// call sites (the `NetRomParseOptions::BPQ` pattern).
    pub const BPQ: Self = Self::LENIENT;

    /// XRouter-flavoured leniency (Paula G8PZT). Today identical to
    /// [`LENIENT`](Self::LENIENT); kept named for symmetry with
    /// `NetRomParseOptions::XROUTER`.
    pub const XROUTER: Self = Self::LENIENT;
}

impl Default for Inp3ParseOptions {
    /// The promiscuous-ingest default — [`LENIENT`](Self::LENIENT), matching the
    /// convenience [`Inp3Rif::try_parse`] overload.
    fn default() -> Self {
        Self::LENIENT
    }
}

// ─── TLV (Inp3Tlv) ──────────────────────────────────────────────────────────

/// One INP3 type/length/value record carried inside a RIP (an [`Inp3Rip`]).
/// Encoded on the wire as `[type][len][value…]` where `len` is a single octet
/// equal to [`value`](Self::value)'s length (0..255).
///
/// Two types have defined meaning (INP3 spec / plan §4.2):
///
/// - [`ALIAS_TYPE`](Self::ALIAS_TYPE) (`0x00`) — the destination's ASCII alias /
///   mnemonic. Decode with [`as_alias`](Self::as_alias).
/// - [`IP_TYPE`](Self::IP_TYPE) (`0x01`) — an IP address; [`value`](Self::value)
///   length 4 = IPv4, 16 = IPv6. Decode with [`as_ipv4`](Self::as_ipv4) /
///   [`as_ipv6`](Self::as_ipv6).
///
/// **Unknown types are retained verbatim.** Any TLV whose type is neither of the
/// above is preserved exactly (type + value bytes) and re-emitted unchanged when
/// the RIP is forwarded — a RIP is never dropped for carrying a TLV we don't
/// understand (forward-compat, plan §4.2/§4.3). [`is_known`](Self::is_known)
/// reports whether the type is one we interpret.
///
/// Mirrors `Packet.NetRom.Wire.Inp3Tlv` on the C# side.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Inp3Tlv {
    /// The TLV type octet.
    pub r#type: u8,
    /// The TLV value bytes (0..255). Retained verbatim for unknown types.
    pub value: Vec<u8>,
}

impl Inp3Tlv {
    /// TLV type: the destination's ASCII alias / mnemonic.
    pub const ALIAS_TYPE: u8 = 0x00;

    /// TLV type: an IP address (value length 4 = IPv4, 16 = IPv6).
    pub const IP_TYPE: u8 = 0x01;

    /// Octets this TLV occupies on the wire: `1 (type) + 1 (len) + value.len()`.
    /// Mirrors `Inp3Tlv.EncodedLength`.
    pub fn encoded_length(&self) -> usize {
        2 + self.value.len()
    }

    /// `true` if [`type`](Self::type) is a type this codec interprets
    /// ([`ALIAS_TYPE`](Self::ALIAS_TYPE) or [`IP_TYPE`](Self::IP_TYPE)); `false`
    /// for an unknown type retained verbatim. Mirrors `Inp3Tlv.IsKnown`.
    pub fn is_known(&self) -> bool {
        self.r#type == Self::ALIAS_TYPE || self.r#type == Self::IP_TYPE
    }

    /// Build an alias TLV ([`ALIAS_TYPE`](Self::ALIAS_TYPE)) from a mnemonic
    /// string. The printable-ASCII characters of `alias` are written verbatim (no
    /// padding, no shift) — the alias is variable-length inside a TLV, unlike the
    /// fixed 6-byte NODES alias field. Any non-printable character is replaced
    /// with a space so a stray control / high-bit char can never reach the wire.
    ///
    /// Mirrors `Inp3Tlv.Alias`.
    pub fn alias(alias: &str) -> Self {
        let mut value = Vec::with_capacity(alias.len());
        for &b in alias.as_bytes() {
            value.push(if (b' '..=b'~').contains(&b) { b } else { b' ' });
        }
        Self {
            r#type: Self::ALIAS_TYPE,
            value,
        }
    }

    /// Build an IP TLV ([`IP_TYPE`](Self::IP_TYPE)) from raw network-order address
    /// bytes (4 octets = IPv4, 16 octets = IPv6). The bytes are taken verbatim —
    /// this is the analogue of the C# `Inp3Tlv.Ip(IPAddress)`, which calls
    /// `address.GetAddressBytes()`. The `no_std` core has no built-in IP-address
    /// type, so the codec works in the address bytes directly;
    /// [`as_ipv4`](Self::as_ipv4) / [`as_ipv6`](Self::as_ipv6) decode them back.
    ///
    /// Mirrors `Inp3Tlv.Ip`.
    pub fn ip(address_bytes: &[u8]) -> Self {
        Self {
            r#type: Self::IP_TYPE,
            value: address_bytes.to_vec(),
        }
    }

    /// Decode [`value`](Self::value) as a trimmed ASCII alias into `dst`,
    /// returning the number of bytes written. Keeps the printable characters only
    /// (a corrupted octet is dropped, never rendered as mojibake) with trailing
    /// spaces stripped — the same discipline as `read_alias`. Meaningful only when
    /// [`type`](Self::type) is [`ALIAS_TYPE`](Self::ALIAS_TYPE), but works on any
    /// value. Allocation-free (writes into a caller buffer) so it works on the
    /// embedded target; any byte that doesn't fit `dst` is dropped.
    ///
    /// Mirrors `Inp3Tlv.AsAlias` (which returns a `string`; the `no_std` analogue
    /// returns the printable-ASCII byte count written to `dst`, valid UTF-8 by
    /// construction — read it back with [`core::str::from_utf8`]).
    pub fn as_alias(&self, dst: &mut [u8]) -> usize {
        let mut len = 0usize;
        for &b in &self.value {
            if (b' '..=b'~').contains(&b) {
                if len >= dst.len() {
                    break;
                }
                dst[len] = b;
                len += 1;
            }
        }
        // Trim trailing spaces (TrimEnd).
        while len > 0 && dst[len - 1] == b' ' {
            len -= 1;
        }
        len
    }

    /// Decode [`value`](Self::value) as a 4-octet IPv4 address; returns `None`
    /// unless the value is exactly 4 octets. Meaningful only when
    /// [`type`](Self::type) is [`IP_TYPE`](Self::IP_TYPE), but works on any
    /// 4-octet value. Mirrors the IPv4 branch of `Inp3Tlv.AsIpAddress`.
    pub fn as_ipv4(&self) -> Option<[u8; 4]> {
        if self.value.len() == 4 {
            Some([self.value[0], self.value[1], self.value[2], self.value[3]])
        } else {
            None
        }
    }

    /// Decode [`value`](Self::value) as a 16-octet IPv6 address; returns `None`
    /// unless the value is exactly 16 octets. Mirrors the IPv6 branch of
    /// `Inp3Tlv.AsIpAddress`.
    pub fn as_ipv6(&self) -> Option<[u8; 16]> {
        if self.value.len() == 16 {
            let mut out = [0u8; 16];
            out.copy_from_slice(&self.value);
            Some(out)
        } else {
            None
        }
    }

    /// Encode this TLV (`[type][len][value…]`) into `dst`, returning the octets
    /// written. Returns `None` if the value is longer than 255 octets (cannot be
    /// length-prefixed by a single byte) or `dst` lacks room — both caller bugs;
    /// we never emit a malformed TLV. Mirrors `Inp3Tlv.Write` (the writer stays
    /// strict, per the §0 totality contract: only the *parser* is total).
    fn write(&self, dst: &mut [u8]) -> Option<usize> {
        if self.value.len() > 255 {
            return None;
        }
        let need = self.encoded_length();
        if dst.len() < need {
            return None;
        }
        dst[0] = self.r#type;
        dst[1] = self.value.len() as u8;
        dst[2..need].copy_from_slice(&self.value);
        Some(need)
    }
}

// ─── RIP (Inp3Rip) ──────────────────────────────────────────────────────────

/// One INP3 Routing Information Packet — a single routing entry inside a
/// [`Inp3Rif`]: "destination [`destination`](Self::destination) is reachable in
/// [`hop_count`](Self::hop_count) hops with a measured target time of
/// [`target_time_ms`](Self::target_time_ms) ms," plus zero or more [`Inp3Tlv`]
/// records.
///
/// Wire layout (plan §4.2):
///
/// ```text
///   [7] destination callsign  (AX.25 shifted form; reuse the callsign codec)
///   [1] hop count
///   [2] target time           MSB-first, 10 ms units (0..65535 → 0..655.35 s)
///   [*] TLV fields            zero or more [type][len][value] records (Inp3Tlv)
///   [1] 0x00                  EOP (end-of-packet) terminator
/// ```
///
/// **The horizon.** A target time at or above [`HORIZON_MS`](Self::HORIZON_MS)
/// (`0xEA60` units = 600.000 s) marks the destination unreachable; a RIP at the
/// horizon is a route *withdrawal* (plan §5.3). This codec decodes the value
/// faithfully and exposes [`is_horizon`](Self::is_horizon) so the routing layer
/// need not re-derive the constant; the act of withdrawing the route is out of
/// scope here (INP3 slice I-3).
///
/// **Alias TLV vs EOP.** An alias TLV has type `0x00`, identical to the EOP byte;
/// they are distinguished positionally (spec §2.3, AMBIGUITY-RIF-2, locked
/// reading (a)): a `0x00` followed by a length byte and that many value bytes
/// still inside the body is an alias TLV; a `0x00` that cannot be satisfied as a
/// TLV is the EOP. [`try_parse`](Self::try_parse) implements exactly that.
///
/// Mirrors `Packet.NetRom.Wire.Inp3Rip` on the C# side.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Inp3Rip {
    /// The destination node this RIP advertises a route to.
    pub destination: Callsign,
    /// Hop count to [`destination`](Self::destination).
    pub hop_count: u8,
    /// Target time to the destination, in milliseconds. On the wire this is a
    /// MSB-first 16-bit count of 10 ms units, so the stored value is always a
    /// multiple of 10 in the range 0..655350.
    pub target_time_ms: u32,
    /// The TLV records carried by this RIP (alias / IP / unknown), in wire order.
    /// May be empty.
    pub tlvs: Vec<Inp3Tlv>,
}

impl Inp3Rip {
    /// Octets of fixed prefix before the TLV region: 7 callsign + 1 hop + 2
    /// target-time (= 10).
    pub const PREFIX_LENGTH: usize = SHIFTED_LENGTH + 1 + 2;

    /// Target-time units (10 ms each) at the routing horizon — destination
    /// unreachable (`0xEA60` = 60000).
    pub const HORIZON_UNITS: u32 = 0xEA60;

    /// The routing horizon in milliseconds (600.000 s). A target time at or above
    /// this is a withdrawal.
    pub const HORIZON_MS: u32 = Self::HORIZON_UNITS * 10; // 600_000

    /// The EOP (end-of-packet) terminator byte that closes a RIP on the wire.
    pub const END_OF_PACKET: u8 = 0x00;

    /// `true` if [`target_time_ms`](Self::target_time_ms) is at or above the
    /// routing horizon ([`HORIZON_MS`](Self::HORIZON_MS)) — i.e. this RIP
    /// withdraws the route. Mirrors `Inp3Rip.IsHorizon`.
    pub fn is_horizon(&self) -> bool {
        self.target_time_ms >= Self::HORIZON_MS
    }

    /// The first alias TLV's decoded string written into `dst`, returning the
    /// byte count, or `None` if this RIP carries no alias TLV. Convenience over
    /// scanning [`tlvs`](Self::tlvs). Mirrors `Inp3Rip.Alias`.
    pub fn alias(&self, dst: &mut [u8]) -> Option<usize> {
        for tlv in &self.tlvs {
            if tlv.r#type == Inp3Tlv::ALIAS_TYPE {
                return Some(tlv.as_alias(dst));
            }
        }
        None
    }

    /// Octets this RIP occupies on the wire: prefix + every TLV + the EOP byte.
    /// Mirrors `Inp3Rip.EncodedLength`.
    pub fn encoded_length(&self) -> usize {
        let mut len = Self::PREFIX_LENGTH;
        for tlv in &self.tlvs {
            len += tlv.encoded_length();
        }
        len + 1 // EOP
    }

    /// Encode this RIP into `dst`, returning the octets written. Returns `None` if
    /// a field is out of encodable range (target time over 655350 ms, or a TLV
    /// value over 255 octets) or `dst` lacks room — all caller bugs; we never emit
    /// a malformed RIP. Mirrors `Inp3Rip.Write` (strict by the §0 contract).
    fn write(&self, dst: &mut [u8]) -> Option<usize> {
        let units = self.target_time_ms / 10;
        if units > 0xFFFF {
            return None;
        }
        let need = self.encoded_length();
        if dst.len() < need {
            return None;
        }

        write_shifted(&self.destination, dst)?;
        let mut offset = SHIFTED_LENGTH;

        dst[offset] = self.hop_count;
        offset += 1;
        dst[offset] = ((units >> 8) & 0xFF) as u8; // MSB first
        dst[offset + 1] = (units & 0xFF) as u8;
        offset += 2;

        for tlv in &self.tlvs {
            let n = tlv.write(&mut dst[offset..])?;
            offset += n;
        }

        dst[offset] = Self::END_OF_PACKET;
        Some(offset + 1)
    }

    /// Allocate and return this RIP's wire encoding, or `None` if a field is out
    /// of encodable range (a caller bug). Mirrors `Inp3Rip.ToBytes`.
    pub fn to_bytes(&self) -> Option<Vec<u8>> {
        let mut buf = alloc::vec![0u8; self.encoded_length()];
        self.write(&mut buf)?;
        Some(buf)
    }

    /// Try to decode one RIP from the front of `source`, returning the decoded RIP
    /// and how many octets it consumed (prefix + TLVs + EOP). Returns `None`
    /// (never panics) on any input that is too short or cannot be framed as a
    /// whole RIP — a truncated prefix, a callsign field that fails to decode, a
    /// TLV whose claimed length runs off the end of `source`, or a RIP with no
    /// terminating EOP.
    ///
    /// `source` is the RIF body at this RIP's start (it may contain further RIPs
    /// after this one — only the consumed prefix is parsed here).
    ///
    /// Mirrors `Packet.NetRom.Wire.Inp3Rip.TryParse`.
    pub fn try_parse(source: &[u8]) -> Option<(Self, usize)> {
        if source.len() < Self::PREFIX_LENGTH {
            return None;
        }

        let dest = try_read_shifted(source)?;
        let mut offset = SHIFTED_LENGTH;

        let hop = source[offset];
        offset += 1;
        let units = ((source[offset] as u32) << 8) | source[offset + 1] as u32; // MSB first
        offset += 2;

        // Walk the TLV region. The EOP is a 0x00 that cannot be satisfied as a
        // TLV; an alias TLV (type 0x00) is a 0x00 followed by [len][value] that
        // still fits inside the body (AMBIGUITY-RIF-2, locked reading (a)).
        let mut tlvs: Vec<Inp3Tlv> = Vec::new();
        loop {
            if offset >= source.len() {
                // Ran out of bytes before an EOP — the RIP is truncated.
                return None;
            }

            let r#type = source[offset];

            if r#type == Self::END_OF_PACKET {
                // Could be EOP, or the start of an alias TLV (type 0x00). It is a
                // TLV iff a length byte follows AND that many value bytes still fit
                // inside the source before its end. Otherwise it is the EOP.
                //
                // This "fits → alias, else → EOP" rule is forced by AMBIGUITY-RIF-2
                // (alias type == EOP == 0x00) and is exactly what lets a multi-RIP
                // RIF find its boundaries: a real EOP is followed by the next RIP's
                // shifted callsign, whose first octet (≈0x80+) frames as an alias
                // length that overruns the remaining body, so it reads as EOP. The
                // unavoidable consequence: a *truncated* trailing alias is
                // indistinguishable from EOP-plus-partial, so it degrades to a RIP
                // that keeps its route but drops the malformed alias (the residual
                // flagged for I-5 interop validation; alias *emission* stays gated
                // off until then). Never panics either way — the fuzz contract holds.
                let is_tlv = offset + 1 < source.len()                          // room for a len byte
                    && offset + 2 + source[offset + 1] as usize <= source.len(); // room for len value bytes

                if !is_tlv {
                    // EOP — RIP ends here.
                    offset += 1;
                    break;
                }
            } else {
                // Non-zero type must have a length byte.
                if offset + 1 >= source.len() {
                    return None;
                }
            }

            let len = source[offset + 1] as usize;
            let value_start = offset + 2;
            if value_start + len > source.len() {
                // TLV claims more value bytes than remain — truncated.
                return None;
            }

            tlvs.push(Inp3Tlv {
                r#type,
                value: source[value_start..value_start + len].to_vec(),
            });
            offset = value_start + len;
        }

        Some((
            Self {
                destination: dest,
                hop_count: hop,
                target_time_ms: units * 10,
                tlvs,
            },
            offset,
        ))
    }
}

// ─── RIF (Inp3Rif) ──────────────────────────────────────────────────────────

/// A parsed INP3 Routing Information Frame — the `0xFF`-signed body carried in the
/// information field of a connected-mode interlink I-frame (PID 0xCF). It is the
/// connected-mode analogue of a [`NodesBroadcast`](super::broadcast::NodesBroadcast):
/// a signature byte followed by a self-delimited sequence of routing entries
/// ([`Inp3Rip`]), each closed by its own EOP.
///
/// Body layout (plan §4.2):
///
/// ```text
///   [1]  0xFF  signature (gates the whole body; non-0xFF → not a RIF → None)
///   then 1..N RIPs, each self-delimited by its 0x00 EOP
/// ```
///
/// This type models the I-frame's *info-field body*, exactly as `NodesBroadcast`
/// models a UI info field — not the surrounding AX.25 frame. RIF and NODES are
/// **never confused** despite both leading with `0xFF`: they arrive on different
/// carriers (RIF on a connected I-frame, NODES on a UI frame to dest `NODES`), so
/// the caller selects the codec by carrier — there is no content-sniffing
/// (AMBIGUITY-RIF-1).
///
/// Parsing is read-only and total: arbitrary, truncated or adversarial bytes
/// never panic — they return `None`. Divergence tolerance (empty RIP list, a
/// clipped trailing RIP) is gated by [`Inp3ParseOptions`] — strict by default,
/// lenient on the convenience [`try_parse`](Self::try_parse) path used for
/// promiscuous ingest.
///
/// Mirrors `Packet.NetRom.Wire.Inp3Rif` on the C# side.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Inp3Rif {
    /// The RIPs carried in this RIF, in wire order. May be empty (lenient) but
    /// never absent.
    pub rips: Vec<Inp3Rip>,
}

impl Inp3Rif {
    /// The INP3 RIF signature byte that opens the info-field body (shared with
    /// NODES; disambiguated by carrier).
    pub const SIGNATURE: u8 = 0xFF;

    /// Octets this RIF occupies on the wire: the signature byte + every RIP.
    /// Mirrors `Inp3Rif.EncodedLength`.
    pub fn encoded_length(&self) -> usize {
        let mut len = 1; // signature
        for rip in &self.rips {
            len += rip.encoded_length();
        }
        len
    }

    /// Encode this RIF into `dst`, returning the octets written. Returns `None` if
    /// `dst` lacks room or a RIP field is out of encodable range — caller bugs; we
    /// never emit a malformed RIF. Mirrors `Inp3Rif.Write`.
    fn write(&self, dst: &mut [u8]) -> Option<usize> {
        let need = self.encoded_length();
        if dst.len() < need {
            return None;
        }

        dst[0] = Self::SIGNATURE;
        let mut offset = 1;
        for rip in &self.rips {
            let n = rip.write(&mut dst[offset..])?;
            offset += n;
        }
        Some(offset)
    }

    /// Allocate and return this RIF's wire encoding (the I-frame info field), or
    /// `None` if a RIP field is out of encodable range. Mirrors `Inp3Rif.ToBytes`.
    pub fn to_bytes(&self) -> Option<Vec<u8>> {
        let mut buf = alloc::vec![0u8; self.encoded_length()];
        self.write(&mut buf)?;
        Some(buf)
    }

    /// Try to parse a RIF body from an interlink I-frame's information field, using
    /// lenient options (the promiscuous-ingest default — see
    /// [`Inp3ParseOptions::LENIENT`]). Returns `None` (never panics) on any
    /// malformed input. Mirrors `Inp3Rif.TryParse(info, out _)`.
    pub fn try_parse(info: &[u8]) -> Option<Self> {
        Self::try_parse_with(info, Inp3ParseOptions::LENIENT)
    }

    /// Try to parse a RIF body, applying `options` for the strict-vs-lenient
    /// divergence choices. Returns `None` (never panics) on any malformed input —
    /// empty, wrong signature, truncated, or adversarial.
    ///
    /// Mirrors `Packet.NetRom.Wire.Inp3Rif.TryParse(info, options, out _)`.
    pub fn try_parse_with(info: &[u8], options: Inp3ParseOptions) -> Option<Self> {
        // Need at least the signature byte.
        if info.is_empty() {
            return None;
        }

        // Signature gates the whole body — a non-0xFF first octet means this is
        // not a RIF (the same "wrong signature → ignore" heuristic NODES uses).
        if info[0] != Self::SIGNATURE {
            return None;
        }

        let mut rips: Vec<Inp3Rip> = Vec::new();
        let mut offset = 1;
        while offset < info.len() {
            match Inp3Rip::try_parse(&info[offset..]) {
                Some((rip, consumed)) => {
                    rips.push(rip);
                    // Defensive: a zero-consumed RIP would loop forever. try_parse
                    // always consumes at least the prefix + EOP on success, but
                    // guard anyway.
                    if consumed == 0 {
                        break;
                    }
                    offset += consumed;
                }
                None => {
                    // A RIP that doesn't frame cleanly (truncated, bad callsign, a
                    // TLV running off the end). Under lenient, keep the whole RIPs
                    // already parsed and drop the clipped tail (RF-clip tolerance).
                    // Under strict, any leftover that doesn't complete a RIP rejects
                    // the whole frame.
                    if !options.allow_trailing_partial_rip {
                        return None;
                    }
                    break;
                }
            }
        }

        if rips.is_empty() && !options.allow_empty_rip_list {
            return None;
        }

        Some(Self { rips })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;

    // ─── Hex helpers (mirror the C#/TS `Hex` / `hex` test helpers) ───

    /// Parse a space-separated hex string into a byte vector.
    fn hex(s: &str) -> Vec<u8> {
        s.split_whitespace()
            .map(|t| u8::from_str_radix(t, 16).unwrap())
            .collect()
    }

    fn cs(base: &[u8], ssid: u8) -> Callsign {
        Callsign::new(base, ssid).unwrap()
    }

    /// Decode a RIP/alias-TLV alias to an owned `String` for ergonomic asserts.
    fn alias_str(rip: &Inp3Rip) -> Option<alloc::string::String> {
        let mut buf = [0u8; 64];
        let n = rip.alias(&mut buf)?;
        Some(core::str::from_utf8(&buf[..n]).unwrap().into())
    }

    // ─── Shifted-callsign sanity (the spec's stated shifted forms) ───

    #[test]
    fn shifted_callsign_matches_the_spec_vectors() {
        for (base, ssid, expected) in [
            (b"GB7RDG".as_slice(), 0u8, "8E 84 6E A4 88 8E 60"),
            (b"GB7RDG".as_slice(), 7, "8E 84 6E A4 88 8E 6E"),
            (b"M0LTE".as_slice(), 0, "9A 60 98 A8 8A 40 60"),
            (b"GB7XYZ".as_slice(), 0, "8E 84 6E B0 B2 B4 60"),
        ] {
            let mut buf = [0u8; SHIFTED_LENGTH];
            write_shifted(&cs(base, ssid), &mut buf).unwrap();
            assert_eq!(buf.as_slice(), hex(expected).as_slice());
        }
    }

    // ─── RIP single-entry vectors (§2.5) ───

    #[test]
    fn rip1_alias_tlv_parses_and_round_trips() {
        // 8E 84 6E A4 88 8E 60  02  00 2D  00 03 52 44 47  00
        let bytes = hex("8E 84 6E A4 88 8E 60 02 00 2D 00 03 52 44 47 00");
        assert_eq!(bytes.len(), 16);

        let (rip, consumed) = Inp3Rip::try_parse(&bytes).unwrap();
        assert_eq!(consumed, 16);
        assert_eq!(rip.destination, cs(b"GB7RDG", 0));
        assert_eq!(rip.hop_count, 2);
        assert_eq!(rip.target_time_ms, 450);
        assert!(!rip.is_horizon());
        assert_eq!(rip.tlvs.len(), 1);
        assert_eq!(rip.tlvs[0].r#type, Inp3Tlv::ALIAS_TYPE);
        assert_eq!(alias_str(&rip).as_deref(), Some("RDG"));

        assert_eq!(rip.to_bytes().unwrap(), bytes);
    }

    #[test]
    fn rip2_ip_tlv_parses_and_round_trips() {
        // 9A 60 98 A8 8A 40 60  01  00 0C  01 04 2C 83 5B 02  00   (44.131.91.2)
        let bytes = hex("9A 60 98 A8 8A 40 60 01 00 0C 01 04 2C 83 5B 02 00");
        assert_eq!(bytes.len(), 17);

        let (rip, consumed) = Inp3Rip::try_parse(&bytes).unwrap();
        assert_eq!(consumed, 17);
        assert_eq!(rip.destination, cs(b"M0LTE", 0));
        assert_eq!(rip.hop_count, 1);
        assert_eq!(rip.target_time_ms, 120);
        assert_eq!(rip.tlvs.len(), 1);
        assert_eq!(rip.tlvs[0].r#type, Inp3Tlv::IP_TYPE);
        assert_eq!(rip.tlvs[0].as_ipv4(), Some([44, 131, 91, 2]));

        assert_eq!(rip.to_bytes().unwrap(), bytes);
    }

    #[test]
    fn rip3_unknown_tlv_is_retained_verbatim_and_re_emitted() {
        // 8E 84 6E B0 B2 B4 60  04  00 FA  7F 02 AA BB  00 03 58 59 5A  00
        let bytes = hex("8E 84 6E B0 B2 B4 60 04 00 FA 7F 02 AA BB 00 03 58 59 5A 00");
        assert_eq!(bytes.len(), 20);

        let (rip, consumed) = Inp3Rip::try_parse(&bytes).unwrap();
        assert_eq!(consumed, 20);
        assert_eq!(rip.destination, cs(b"GB7XYZ", 0));
        assert_eq!(rip.hop_count, 4);
        assert_eq!(rip.target_time_ms, 2500);

        assert_eq!(rip.tlvs.len(), 2);

        // The 0x7F TLV is unknown → retained verbatim, flagged not-known.
        let unknown = &rip.tlvs[0];
        assert_eq!(unknown.r#type, 0x7F);
        assert!(!unknown.is_known());
        assert_eq!(unknown.value, vec![0xAA, 0xBB]);

        // The alias TLV after the unknown one still decodes.
        assert_eq!(rip.tlvs[1].r#type, Inp3Tlv::ALIAS_TYPE);
        assert!(rip.tlvs[1].is_known());
        assert_eq!(alias_str(&rip).as_deref(), Some("XYZ"));

        // Re-emission keeps the unknown TLV byte-for-byte.
        assert_eq!(rip.to_bytes().unwrap(), bytes);
    }

    #[test]
    fn rip4_horizon_withdrawal_has_no_tlv_and_flags_horizon() {
        // 8E 84 6E A4 88 8E 6E  FF  EA 60  00
        let bytes = hex("8E 84 6E A4 88 8E 6E FF EA 60 00");
        assert_eq!(bytes.len(), 11);

        let (rip, consumed) = Inp3Rip::try_parse(&bytes).unwrap();
        assert_eq!(consumed, 11);
        assert_eq!(rip.destination, cs(b"GB7RDG", 7));
        assert_eq!(rip.hop_count, 0xFF);
        assert_eq!(rip.target_time_ms, Inp3Rip::HORIZON_MS);
        assert_eq!(rip.target_time_ms, 600_000);
        assert!(rip.is_horizon());
        assert!(rip.tlvs.is_empty());
        assert_eq!(alias_str(&rip), None);

        assert_eq!(rip.to_bytes().unwrap(), bytes);
    }

    // ─── RIF body vectors (§2.5) ───

    #[test]
    fn rif_full_parses_all_four_rips_in_order() {
        let bytes = hex(concat!(
            "FF ",
            "8E 84 6E A4 88 8E 60 02 00 2D 00 03 52 44 47 00 ", // RIP-1
            "9A 60 98 A8 8A 40 60 01 00 0C 01 04 2C 83 5B 02 00 ", // RIP-2
            "8E 84 6E B0 B2 B4 60 04 00 FA 7F 02 AA BB 00 03 58 59 5A 00 ", // RIP-3
            "8E 84 6E A4 88 8E 6E FF EA 60 00", // RIP-4
        ));
        assert_eq!(bytes.len(), 65); // 1 + 16 + 17 + 20 + 11

        let rif = Inp3Rif::try_parse_with(&bytes, Inp3ParseOptions::STRICT).unwrap();
        assert_eq!(rif.rips.len(), 4);

        let dests: Vec<Callsign> = rif.rips.iter().map(|r| r.destination).collect();
        assert_eq!(
            dests,
            vec![
                cs(b"GB7RDG", 0),
                cs(b"M0LTE", 0),
                cs(b"GB7XYZ", 0),
                cs(b"GB7RDG", 7),
            ]
        );
        let hops: Vec<u8> = rif.rips.iter().map(|r| r.hop_count).collect();
        assert_eq!(hops, vec![2, 1, 4, 0xFF]);
        let times: Vec<u32> = rif.rips.iter().map(|r| r.target_time_ms).collect();
        assert_eq!(times, vec![450, 120, 2500, 600_000]);

        assert_eq!(alias_str(&rif.rips[0]).as_deref(), Some("RDG"));
        assert_eq!(rif.rips[1].tlvs[0].as_ipv4(), Some([44, 131, 91, 2]));
        assert_eq!(rif.rips[2].tlvs[0].r#type, 0x7F); // unknown retained
        assert_eq!(alias_str(&rif.rips[2]).as_deref(), Some("XYZ"));
        assert!(rif.rips[3].is_horizon());

        // Round-trip the whole frame.
        assert_eq!(rif.to_bytes().unwrap(), bytes);
    }

    #[test]
    fn rif_min_signature_plus_one_no_tlv_rip() {
        // FF  9A 60 98 A8 8A 40 60  01  00 7B  00
        let bytes = hex("FF 9A 60 98 A8 8A 40 60 01 00 7B 00");
        assert_eq!(bytes.len(), 12);

        let rif = Inp3Rif::try_parse_with(&bytes, Inp3ParseOptions::STRICT).unwrap();
        assert_eq!(rif.rips.len(), 1);
        let rip = &rif.rips[0];
        assert_eq!(rip.destination, cs(b"M0LTE", 0));
        assert_eq!(rip.hop_count, 1);
        assert_eq!(rip.target_time_ms, 1230); // 0x7B = 123 units × 10 ms
        assert!(rip.tlvs.is_empty());

        assert_eq!(rif.to_bytes().unwrap(), bytes);
    }

    // ─── Builder-side round-trip (parser is the oracle) ───

    #[test]
    fn built_rif_round_trips_through_the_parser() {
        let rif = Inp3Rif {
            rips: vec![
                Inp3Rip {
                    destination: cs(b"GB7RDG", 0),
                    hop_count: 2,
                    target_time_ms: 450,
                    tlvs: vec![Inp3Tlv::alias("RDG")],
                },
                Inp3Rip {
                    destination: cs(b"M0LTE", 0),
                    hop_count: 1,
                    target_time_ms: 120,
                    tlvs: vec![Inp3Tlv::ip(&[0x2C, 0x83, 0x5B, 0x02])],
                },
                Inp3Rip {
                    destination: cs(b"GB7RDG", 7),
                    hop_count: 0xFF,
                    target_time_ms: Inp3Rip::HORIZON_MS,
                    tlvs: vec![],
                },
            ],
        };

        let bytes = rif.to_bytes().unwrap();

        let parsed = Inp3Rif::try_parse_with(&bytes, Inp3ParseOptions::STRICT).unwrap();
        assert_eq!(parsed.rips.len(), 3);
        assert_eq!(alias_str(&parsed.rips[0]).as_deref(), Some("RDG"));
        assert_eq!(parsed.rips[1].tlvs[0].as_ipv4(), Some([44, 131, 91, 2]));
        assert!(parsed.rips[2].is_horizon());
        assert_eq!(parsed.to_bytes().unwrap(), bytes);
    }

    #[test]
    fn ipv6_tlv_round_trips() {
        // 2001:db8::1
        let v6: [u8; 16] = [
            0x20, 0x01, 0x0D, 0xB8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x01,
        ];
        let rip = Inp3Rip {
            destination: cs(b"M0LTE", 0),
            hop_count: 1,
            target_time_ms: 100,
            tlvs: vec![Inp3Tlv::ip(&v6)],
        };

        let bytes = rip.to_bytes().unwrap();
        let (parsed, _) = Inp3Rip::try_parse(&bytes).unwrap();
        assert_eq!(parsed.tlvs[0].value.len(), 16);
        assert_eq!(parsed.tlvs[0].as_ipv6(), Some(v6));
    }

    // ─── Empty-list preset gating (§2.6, mirrors NODES) ───

    #[test]
    fn signature_only_rif_is_rejected_by_strict_but_accepted_by_lenient() {
        let bytes = hex("FF"); // signature, zero RIPs

        assert!(Inp3Rif::try_parse_with(&bytes, Inp3ParseOptions::STRICT).is_none());

        let lenient = Inp3Rif::try_parse_with(&bytes, Inp3ParseOptions::LENIENT).unwrap();
        assert!(lenient.rips.is_empty());
    }

    #[test]
    fn bpq_and_xrouter_presets_accept_signature_only_like_lenient() {
        let bytes = hex("FF");
        let bpq = Inp3Rif::try_parse_with(&bytes, Inp3ParseOptions::BPQ).unwrap();
        assert!(bpq.rips.is_empty());
        let xr = Inp3Rif::try_parse_with(&bytes, Inp3ParseOptions::XROUTER).unwrap();
        assert!(xr.rips.is_empty());
    }

    // ─── Trailing-partial RIP gating (§2.6) ───

    #[test]
    fn rip_truncated_mid_target_time_is_rejected_by_strict_dropped_by_lenient() {
        // FF + a clean RIP-MIN body, then a second RIP clipped after 2 octets of its prefix.
        let mut clipped = hex("FF 9A 60 98 A8 8A 40 60 01 00 7B 00");
        clipped.extend_from_slice(&hex("8E 84 6E A4 88 8E 60 02 00")); // partial RIP-2

        // Strict: the leftover that doesn't complete a RIP rejects the whole frame.
        assert!(Inp3Rif::try_parse_with(&clipped, Inp3ParseOptions::STRICT).is_none());

        // Lenient: keep the whole RIP parsed, drop the clipped tail.
        let lenient = Inp3Rif::try_parse_with(&clipped, Inp3ParseOptions::LENIENT).unwrap();
        assert_eq!(lenient.rips.len(), 1);
        assert_eq!(lenient.rips[0].destination, cs(b"M0LTE", 0));
    }

    #[test]
    fn truncated_trailing_alias_tlv_degrades_to_eop_keeping_the_route() {
        // FF + a RIP whose trailing bytes look like an alias TLV (00 03 ...) but claim
        // more value bytes than remain (len=3, only "RD" present).
        let bytes = hex("FF 8E 84 6E A4 88 8E 60 02 00 2D 00 03 52 44");

        // The alias TLV type (0x00) is identical to the EOP byte (AMBIGUITY-RIF-2),
        // so a 0x00 that cannot be satisfied as a TLV is *necessarily* read as the
        // EOP — this is the same rule that lets a multi-RIP RIF find its boundaries.
        // The RIP therefore keeps its routing fields (450 ms) and simply drops the
        // malformed trailing alias; the leftover bytes are a trailing partial.

        // Strict: the leftover (03 52 44) is an un-frameable trailing partial → reject.
        assert!(Inp3Rif::try_parse_with(&bytes, Inp3ParseOptions::STRICT).is_none());

        // Lenient: the leftover partial is dropped; the one whole RIP survives, sans alias.
        let lenient = Inp3Rif::try_parse_with(&bytes, Inp3ParseOptions::LENIENT).unwrap();
        assert_eq!(lenient.rips.len(), 1);
        assert_eq!(lenient.rips[0].target_time_ms, 450);
        // The malformed trailing alias was read as EOP and dropped.
        assert_eq!(alias_str(&lenient.rips[0]), None);
    }

    #[test]
    fn a_target_time_above_the_horizon_is_flagged_unreachable() {
        // Max encodable target time 0xFFFF = 655350 ms — above the 600 000 ms horizon,
        // so still a withdrawal. (RIP-4 covers exactly-horizon; this covers above it.)
        let bytes = hex("FF 9A 60 98 A8 8A 40 60 01 FF FF 00");
        let rif = Inp3Rif::try_parse(&bytes).unwrap();
        assert_eq!(rif.rips.len(), 1);
        assert_eq!(rif.rips[0].target_time_ms, 655350);
        // Any target time at/above 600 s is unreachable.
        assert!(rif.rips[0].is_horizon());
    }

    // ─── Wrong / missing signature (§2.6) ───

    #[test]
    fn empty_input_returns_none() {
        assert!(Inp3Rif::try_parse(&[]).is_none());
    }

    #[test]
    fn wrong_signature_returns_none() {
        // Same bytes as RIF-MIN but signature 0x00 instead of 0xFF.
        let bytes = hex("00 9A 60 98 A8 8A 40 60 01 00 7B 00");
        assert!(Inp3Rif::try_parse(&bytes).is_none());
    }

    #[test]
    fn rip_with_bad_callsign_field_fails_to_parse() {
        // A 7-octet callsign slot of all-zero bytes does not decode
        // (try_read_shifted → None): 0x00 chars are not A-Z/0-9 once unshifted, so
        // the callsign decode fails first.
        let bytes = vec![0u8; Inp3Rip::PREFIX_LENGTH + 1]; // garbage prefix + a byte
        assert!(Inp3Rip::try_parse(&bytes).is_none());
    }

    // ─── Totality: arbitrary / truncated bytes never panic (§0 contract) ───

    #[test]
    fn short_or_truncated_input_never_panics() {
        for length in [0usize, 1, 2, 10, 11, 15, 64] {
            let mut bytes = vec![0u8; length];
            if length > 0 {
                bytes[0] = Inp3Rif::SIGNATURE;
            }
            // Must not panic.
            let _ = Inp3Rif::try_parse(&bytes);
            let _ = Inp3Rif::try_parse_with(&bytes, Inp3ParseOptions::STRICT);
        }
    }

    #[test]
    fn truncations_of_every_full_rif_prefix_never_panic_and_never_over_read() {
        let full = hex(concat!(
            "FF 8E 84 6E A4 88 8E 60 02 00 2D 00 03 52 44 47 00 ",
            "9A 60 98 A8 8A 40 60 01 00 0C 01 04 2C 83 5B 02 00 ",
            "8E 84 6E B0 B2 B4 60 04 00 FA 7F 02 AA BB 00 03 58 59 5A 00 ",
            "8E 84 6E A4 88 8E 6E FF EA 60 00",
        ));

        for n in 0..=full.len() {
            let prefix = &full[..n];
            // Must not panic / over-read for any prefix length.
            let _ = Inp3Rif::try_parse_with(prefix, Inp3ParseOptions::LENIENT);
            let _ = Inp3Rif::try_parse_with(prefix, Inp3ParseOptions::STRICT);
        }
    }

    /// A tiny deterministic PRNG (xorshift32) so the fuzz vectors are reproducible
    /// — the `no_std` analogue of the C# `new Random(seed)` / TS mulberry32 the
    /// ported tests use. Returns an integer in `[min, max)`.
    fn rng(state: &mut u32, min: u32, max: u32) -> u32 {
        let mut x = *state;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        *state = x;
        min + (x % (max - min))
    }

    #[test]
    fn random_garbage_never_panics() {
        let mut state: u32 = 20260607;
        for _ in 0..2000 {
            let len = rng(&mut state, 0, 400) as usize;
            let mut bytes = vec![0u8; len];
            for b in &mut bytes {
                *b = rng(&mut state, 0, 256) as u8;
            }
            // Must not panic on any of the three entry points.
            let _ = Inp3Rif::try_parse_with(&bytes, Inp3ParseOptions::LENIENT);
            let _ = Inp3Rif::try_parse_with(&bytes, Inp3ParseOptions::STRICT);
            let _ = Inp3Rip::try_parse(&bytes);
        }
    }

    #[test]
    fn random_signature_prefixed_garbage_never_panics() {
        // Bias toward 0xFF-signed bodies so the RIP walker is exercised on junk.
        let mut state: u32 = 424242;
        for _ in 0..2000 {
            let len = rng(&mut state, 1, 200) as usize;
            let mut bytes = vec![0u8; len];
            for b in &mut bytes {
                *b = rng(&mut state, 0, 256) as u8;
            }
            bytes[0] = Inp3Rif::SIGNATURE;
            let _ = Inp3Rif::try_parse_with(&bytes, Inp3ParseOptions::LENIENT);
            let _ = Inp3Rif::try_parse_with(&bytes, Inp3ParseOptions::STRICT);
        }
    }
}
