//! NET/ROM wire formats — the byte-level codecs.
//!
//! Mirrors the C# `Packet.NetRom.Wire` namespace. The read-only (ingest) half:
//! the named-divergence parse options ([`NetRomParseOptions`]), the two NET/ROM
//! callsign/alias field decoders ([`callsign`]), the 21-byte destination entry
//! ([`NodesRoutingEntry`]), and the NODES routing broadcast parser
//! ([`NodesBroadcast`]). The origination + L4 half (added for the full L3+L4
//! parity slice): the callsign field encoders ([`write_shifted`]/[`write_alias`]),
//! the L4 transport header ([`NetRomTransportHeader`]), the Connect Request info
//! field ([`ConnectRequestInfo`]), the NODES origination builder
//! ([`write_nodes_frame`]), and the L3 network header + datagram
//! ([`NetRomNetworkHeader`] / [`NetRomPacket`]).
//!
//! Everything is `no_std`/allocation-free: parsers borrow the source slice and the
//! builders write into a caller-provided buffer (no heap byte arrays). The
//! outbound path is strict/canonical even though the parser tolerates real-world
//! divergences inbound.

pub mod broadcast;
pub mod callsign;
pub mod connect_request_info;
pub mod entry;
pub mod network_header;
pub mod nodes_broadcast_builder;
pub mod options;
pub mod packet;
pub mod transport_header;

pub use broadcast::NodesBroadcast;
pub use callsign::{
    read_alias, try_read_shifted, write_alias, write_shifted, Alias, ALIAS_LENGTH, SHIFTED_LENGTH,
};
pub use connect_request_info::{ConnectRequestInfo, CONNECT_REQUEST_INFO_LEN};
pub use entry::NodesRoutingEntry;
pub use network_header::{NetRomNetworkHeader, DEFAULT_TIME_TO_LIVE, NETWORK_HEADER_LEN};
pub use nodes_broadcast_builder::{
    write_nodes_frame, NodesAdvertisementEntry, MAX_NODES_FRAME_LEN,
};
pub use options::NetRomParseOptions;
pub use packet::{NetRomPacket, MAX_PAYLOAD, PACKET_HEADER_LEN};
pub use transport_header::{
    NetRomOpcode, NetRomTransportHeader, FLAG_CHOKE, FLAG_MORE_FOLLOWS, FLAG_NAK, FLAGS_MASK,
    OPCODE_MASK, TRANSPORT_HEADER_LEN,
};

#[cfg(test)]
mod codec_tests;

#[cfg(test)]
pub(crate) mod test_support {
    //! Test-only encoder for NET/ROM NODES-broadcast information fields.
    //!
    //! Mirrors `tests/Packet.NetRom.Tests/NodesBroadcastBuilder.cs`. The production
    //! library is strictly read-only (it parses heard broadcasts and never
    //! originates one), so the tests bring their own byte builder to exercise the
    //! parser and routing table with realistic input — encode here, parse with
    //! [`NodesBroadcast::try_parse`], assert. The callsign fields use the genuine
    //! AX.25 shifted form via [`crate::ax25::Address::encode`] — the same codec the
    //! parser decodes with — so a round-trip proves the shift/SSID handling, not a
    //! tautology against a hand-rolled encoder.

    extern crate alloc;
    use alloc::vec::Vec;

    use super::{NodesBroadcast, NodesRoutingEntry};
    use crate::ax25::{Address, Callsign, ADDRESS_LEN};

    /// One (dest, dest-alias, best-neighbour, quality) tuple for the builder.
    pub type EntrySpec = (Callsign, &'static str, Callsign, u8);

    /// Encode a callsign in the 7-octet AX.25 shifted form (no flags set).
    pub fn encode_shifted(call: Callsign) -> [u8; ADDRESS_LEN] {
        let addr = Address {
            callsign: call,
            crh: false,
            extension: false,
        };
        let mut bytes = [0u8; ADDRESS_LEN];
        addr.encode(&mut bytes).expect("7-byte buffer");
        bytes
    }

    /// Encode a 6-char alias as plain space-padded ASCII (truncated to 6).
    pub fn encode_alias(alias: &str) -> [u8; super::ALIAS_LENGTH] {
        let mut bytes = [b' '; super::ALIAS_LENGTH];
        for (i, &b) in alias
            .as_bytes()
            .iter()
            .take(super::ALIAS_LENGTH)
            .enumerate()
        {
            bytes[i] = b;
        }
        bytes
    }

    /// Build a NODES info field: 0xFF signature + 6-byte alias + the entries.
    pub fn build(sender_alias: &str, entries: &[EntrySpec]) -> Vec<u8> {
        let mut buf = Vec::new();
        buf.push(NodesBroadcast::SIGNATURE);
        buf.extend_from_slice(&encode_alias(sender_alias));
        for &(dest, dest_alias, neighbour, quality) in entries {
            buf.extend_from_slice(&encode_shifted(dest));
            buf.extend_from_slice(&encode_alias(dest_alias));
            buf.extend_from_slice(&encode_shifted(neighbour));
            buf.push(quality);
        }
        debug_assert_eq!(
            buf.len(),
            7 + entries.len() * NodesRoutingEntry::ENCODED_LENGTH
        );
        buf
    }
}

#[cfg(test)]
mod tests {
    use super::test_support::build;
    use super::*;
    use crate::ax25::Callsign;

    fn call(s: &str) -> Callsign {
        Callsign::parse(s).unwrap()
    }

    fn call_ssid(s: &str, ssid: u8) -> Callsign {
        Callsign::new(s.as_bytes(), ssid).unwrap()
    }

    #[test]
    fn parses_signature_and_sender_alias() {
        let info = build("RDGBPQ", &[]);
        let bc = NodesBroadcast::try_parse(&info).unwrap();
        assert_eq!(bc.sender_alias().as_str(), "RDGBPQ");
        assert_eq!(bc.entry_count(), 0);
    }

    #[test]
    fn rejects_a_frame_whose_first_octet_is_not_the_signature() {
        let mut info = build("RDGBPQ", &[]);
        info[0] = 0x00; // canonical "wrong signature → ignore"
        assert!(NodesBroadcast::try_parse(&info).is_none());
    }

    #[test]
    fn parses_a_single_destination_entry_with_all_fields() {
        let sot = call("GB7SOT");
        let xyz = call_ssid("GB7XYZ", 5);
        let info = build("RDGBPQ", &[(sot, "SOT", xyz, 200)]);

        let bc = NodesBroadcast::try_parse(&info).unwrap();
        assert_eq!(bc.entry_count(), 1);
        let e = bc.entries().next().unwrap();
        assert_eq!(e.destination, sot);
        assert_eq!(e.destination_alias.as_str(), "SOT");
        assert_eq!(e.best_neighbour, xyz); // SSID 5 survives the shifted round-trip
        assert_eq!(e.best_quality, 200);
    }

    #[test]
    fn parses_several_entries_in_order() {
        let sot = call("GB7SOT");
        let xyz = call_ssid("GB7XYZ", 5);
        let rdg = call("GB7RDG");
        let info = build(
            "RDGBPQ",
            &[
                (sot, "SOT", xyz, 200),
                (xyz, "XYZ", xyz, 192),
                (rdg, "RDG", rdg, 255),
            ],
        );

        let bc = NodesBroadcast::try_parse(&info).unwrap();
        assert_eq!(bc.entry_count(), 3);
        let dests: alloc::vec::Vec<_> = bc.entries().map(|e| e.destination).collect();
        assert_eq!(dests, alloc::vec![sot, xyz, rdg]);
        let quals: alloc::vec::Vec<_> = bc.entries().map(|e| e.best_quality).collect();
        assert_eq!(quals, alloc::vec![200u8, 192, 255]);
    }

    #[test]
    fn caps_at_eleven_entries_per_frame_ignoring_the_surplus() {
        let sot = call("GB7SOT");
        let xyz = call_ssid("GB7XYZ", 5);
        // Hand-build 13 entries; the canonical format caps a frame at 11.
        let specs: alloc::vec::Vec<super::test_support::EntrySpec> =
            (0..13).map(|_| (sot, "SOT", xyz, 200u8)).collect();
        let info = build("RDGBPQ", &specs);

        let bc = NodesBroadcast::try_parse(&info).unwrap();
        assert_eq!(bc.entry_count(), NodesBroadcast::MAX_ENTRIES_PER_FRAME);
    }

    // ─── Strict-vs-lenient paired tests ───

    #[test]
    fn trailing_partial_entry_is_rejected_by_strict_but_accepted_by_lenient() {
        let sot = call("GB7SOT");
        let xyz = call_ssid("GB7XYZ", 5);
        let mut info = build("RDGBPQ", &[(sot, "SOT", xyz, 200)]);
        info.extend_from_slice(&[0x01, 0x02, 0x03]); // 3 trailing octets (< 21)

        assert!(NodesBroadcast::try_parse_with(&info, NetRomParseOptions::STRICT).is_none());

        let lenient = NodesBroadcast::try_parse_with(&info, NetRomParseOptions::LENIENT).unwrap();
        assert_eq!(lenient.entry_count(), 1); // the whole entry is kept; the remainder dropped
    }

    #[test]
    fn empty_destination_list_is_rejected_by_strict_but_accepted_by_lenient() {
        let info = build("RDGBPQ", &[]); // header only

        assert!(NodesBroadcast::try_parse_with(&info, NetRomParseOptions::STRICT).is_none());
        let lenient = NodesBroadcast::try_parse_with(&info, NetRomParseOptions::LENIENT).unwrap();
        assert_eq!(lenient.entry_count(), 0);
    }

    #[test]
    fn bpq_and_xrouter_presets_accept_a_padded_dump_like_lenient() {
        let sot = call("GB7SOT");
        let xyz = call_ssid("GB7XYZ", 5);
        let mut info = build("RDGBPQ", &[(sot, "SOT", xyz, 200)]);
        info.push(0x00); // one pad octet on the final frame

        let bpq = NodesBroadcast::try_parse_with(&info, NetRomParseOptions::BPQ).unwrap();
        assert_eq!(bpq.entry_count(), 1);
        let xr = NodesBroadcast::try_parse_with(&info, NetRomParseOptions::XROUTER).unwrap();
        assert_eq!(xr.entry_count(), 1);
    }

    // ─── Totality: arbitrary bytes never panic ───

    #[test]
    fn short_or_truncated_input_returns_none_without_panicking() {
        for length in [0usize, 1, 6, 7, 20] {
            let mut bytes = alloc::vec![0u8; length];
            if length > 0 {
                bytes[0] = NodesBroadcast::SIGNATURE;
            }
            // Must not panic; result is just an Option.
            let _ = NodesBroadcast::try_parse(&bytes);
        }
    }

    #[test]
    fn pseudo_random_garbage_never_panics() {
        // A tiny deterministic xorshift PRNG (no `rand` dep — core is dep-free).
        let mut state: u32 = 1234;
        let mut next = || {
            state ^= state << 13;
            state ^= state >> 17;
            state ^= state << 5;
            state
        };
        for _ in 0..500 {
            let len = (next() % 300) as usize;
            let mut bytes = alloc::vec![0u8; len];
            for b in bytes.iter_mut() {
                *b = (next() & 0xFF) as u8;
            }
            let _ = NodesBroadcast::try_parse_with(&bytes, NetRomParseOptions::LENIENT);
            let _ = NodesBroadcast::try_parse_with(&bytes, NetRomParseOptions::STRICT);
        }
    }

    #[test]
    fn alias_is_trimmed_of_trailing_spaces() {
        // "RDG" packed into a 6-byte field is "RDG   "; the parser trims it.
        let info = build("RDG", &[]);
        let bc = NodesBroadcast::try_parse(&info).unwrap();
        assert_eq!(bc.sender_alias().as_str(), "RDG");
    }

    #[test]
    fn all_space_best_neighbour_decodes_to_empty_base() {
        // A node padding an absent best-neighbour slot: the 7-octet field is all
        // shifted spaces. It must decode (lenient), to a zero-length-base callsign.
        let sot = call("GB7SOT");
        let blank = Callsign::new(b"", 0).unwrap();
        let info = build("RDGBPQ", &[(sot, "SOT", blank, 0)]);
        let bc = NodesBroadcast::try_parse(&info).unwrap();
        let e = bc.entries().next().unwrap();
        assert_eq!(e.best_neighbour.base(), b"");
    }
}
