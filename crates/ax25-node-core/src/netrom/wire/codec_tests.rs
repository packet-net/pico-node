//! Round-trip tests for the NET/ROM origination + L4 wire codecs (the TX half of
//! the wire layer). Each codec is exercised encode→decode, and the NODES builder
//! is round-tripped through the production [`NodesBroadcast`] parser as the oracle
//! (builder ↔ parser are inverse) — the same cross-check the C#/TS ports use.

use super::*;
use crate::ax25::Callsign;

fn cs(base: &[u8], ssid: u8) -> Callsign {
    Callsign::new(base, ssid).unwrap()
}

const OPCODES: [NetRomOpcode; 6] = [
    NetRomOpcode::ConnectRequest,
    NetRomOpcode::ConnectAcknowledge,
    NetRomOpcode::DisconnectRequest,
    NetRomOpcode::DisconnectAcknowledge,
    NetRomOpcode::Information,
    NetRomOpcode::InformationAcknowledge,
];

#[test]
fn transport_header_round_trips_each_opcode_and_flags() {
    for op in OPCODES {
        for flags in [0u8, FLAG_CHOKE, FLAG_NAK, FLAG_MORE_FOLLOWS, FLAG_CHOKE | FLAG_NAK] {
            let h = NetRomTransportHeader {
                circuit_index: 7,
                circuit_id: 9,
                tx_sequence: 42,
                rx_sequence: 17,
                opcode: op.as_u8(),
                flags,
            };
            let mut buf = [0u8; TRANSPORT_HEADER_LEN];
            h.encode(&mut buf).unwrap();
            assert_eq!(buf[4], op.as_u8() | flags, "opcode|flags byte");
            let d = NetRomTransportHeader::decode(&buf).unwrap();
            assert_eq!(d, h);
            assert_eq!(NetRomOpcode::from_nibble(d.opcode), Some(op));
            assert_eq!(d.choke(), flags & FLAG_CHOKE != 0);
            assert_eq!(d.nak(), flags & FLAG_NAK != 0);
            assert_eq!(d.more_follows(), flags & FLAG_MORE_FOLLOWS != 0);
        }
    }
}

#[test]
fn transport_header_unknown_opcode_parses_total_but_is_not_known() {
    let buf = [1u8, 2, 3, 4, 0x0F]; // nibble 0x0F is not a defined opcode
    let d = NetRomTransportHeader::decode(&buf).unwrap();
    assert_eq!(d.opcode, 0x0F);
    assert_eq!(NetRomOpcode::from_nibble(d.opcode), None);
}

#[test]
fn transport_header_too_short_is_none() {
    assert!(NetRomTransportHeader::decode(&[0u8; 4]).is_none());
    let h = NetRomTransportHeader {
        circuit_index: 0,
        circuit_id: 0,
        tx_sequence: 0,
        rx_sequence: 0,
        opcode: 1,
        flags: 0,
    };
    assert!(h.encode(&mut [0u8; 4]).is_none());
}

#[test]
fn connect_request_info_round_trips_and_tolerates_trailing_extension() {
    let info = ConnectRequestInfo {
        proposed_window: 4,
        originating_user: cs(b"M0LTE", 1),
        originating_node: cs(b"GB7RDG", 0),
    };
    let mut buf = [0u8; CONNECT_REQUEST_INFO_LEN + 5]; // 15 + a peer's extension octets
    info.encode(&mut buf).unwrap();
    assert_eq!(ConnectRequestInfo::decode(&buf), Some(info)); // trailing ignored
    assert_eq!(
        ConnectRequestInfo::decode(&buf[..CONNECT_REQUEST_INFO_LEN]),
        Some(info)
    );
    assert!(ConnectRequestInfo::decode(&buf[..CONNECT_REQUEST_INFO_LEN - 1]).is_none());
}

#[test]
fn network_header_round_trips_and_decrements_saturating() {
    let h = NetRomNetworkHeader {
        origin: cs(b"M0LTE", 0),
        destination: cs(b"GB7RDG", 0),
        time_to_live: DEFAULT_TIME_TO_LIVE,
    };
    let mut buf = [0u8; NETWORK_HEADER_LEN];
    h.encode(&mut buf).unwrap();
    assert_eq!(NetRomNetworkHeader::decode(&buf), Some(h));
    assert_eq!(h.decremented().time_to_live, DEFAULT_TIME_TO_LIVE - 1);
    let zero = NetRomNetworkHeader {
        time_to_live: 0,
        ..h
    };
    assert_eq!(zero.decremented().time_to_live, 0, "TTL saturates at 0");
    assert!(NetRomNetworkHeader::decode(&buf[..NETWORK_HEADER_LEN - 1]).is_none());
}

#[test]
fn packet_round_trips_with_and_without_payload() {
    let network = NetRomNetworkHeader {
        origin: cs(b"M0LTE", 0),
        destination: cs(b"GB7BBS", 0),
        time_to_live: 10,
    };
    let transport = NetRomTransportHeader {
        circuit_index: 3,
        circuit_id: 5,
        tx_sequence: 1,
        rx_sequence: 2,
        opcode: NetRomOpcode::Information.as_u8(),
        flags: 0,
    };
    let payload = b"hello world";
    let pkt = NetRomPacket {
        network,
        transport,
        payload,
    };
    let mut buf = [0u8; PACKET_HEADER_LEN + 64];
    let n = pkt.encode(&mut buf).unwrap();
    assert_eq!(n, PACKET_HEADER_LEN + payload.len());
    let d = NetRomPacket::decode(&buf[..n]).unwrap();
    assert_eq!(d.network, network);
    assert_eq!(d.transport, transport);
    assert_eq!(d.payload, payload);

    let empty = NetRomPacket {
        network,
        transport,
        payload: &[],
    };
    assert_eq!(empty.encode(&mut buf), Some(PACKET_HEADER_LEN));
    assert!(NetRomPacket::decode(&buf[..PACKET_HEADER_LEN - 1]).is_none());
}

#[test]
fn nodes_frame_round_trips_through_the_production_parser() {
    let sender = Alias::from_str_lossy("RDGND");
    let entries = [
        NodesAdvertisementEntry {
            destination: cs(b"GB7BBS", 0),
            destination_alias: Alias::from_str_lossy("RDGBBS"),
            best_neighbour: cs(b"GB7RDG", 0),
            quality: 200,
        },
        NodesAdvertisementEntry {
            destination: cs(b"GB7CHT", 0),
            destination_alias: Alias::from_str_lossy("CHAT"),
            best_neighbour: cs(b"GB7RDG", 0),
            quality: 150,
        },
    ];
    let mut buf = [0u8; MAX_NODES_FRAME_LEN];
    let n = write_nodes_frame(&sender, &entries, &mut buf).unwrap();
    assert_eq!(buf[0], NodesBroadcast::SIGNATURE);

    let parsed = NodesBroadcast::try_parse(&buf[..n]).unwrap();
    assert_eq!(parsed.sender_alias(), sender);
    assert_eq!(parsed.entry_count(), 2);
    for (i, e) in parsed.entries().enumerate() {
        assert_eq!(e.destination, entries[i].destination);
        assert_eq!(e.destination_alias, entries[i].destination_alias);
        assert_eq!(e.best_neighbour, entries[i].best_neighbour);
        assert_eq!(e.best_quality, entries[i].quality);
    }
}

#[test]
fn nodes_frame_header_only_when_empty() {
    let sender = Alias::from_str_lossy("RDGND");
    let mut buf = [0u8; MAX_NODES_FRAME_LEN];
    let n = write_nodes_frame(&sender, &[], &mut buf).unwrap();
    assert_eq!(n, 1 + ALIAS_LENGTH, "0xFF signature + 6-octet alias only");
    assert_eq!(buf[0], NodesBroadcast::SIGNATURE);
}

#[test]
fn nodes_frame_chunks_at_eleven_entries() {
    let sender = Alias::from_str_lossy("RDGND");
    let entry = NodesAdvertisementEntry {
        destination: cs(b"GB7BBS", 0),
        destination_alias: Alias::from_str_lossy("RDGBBS"),
        best_neighbour: cs(b"GB7RDG", 0),
        quality: 200,
    };
    let many = [entry; 13]; // more than one frame's worth
    let mut buf = [0u8; MAX_NODES_FRAME_LEN];
    let n = write_nodes_frame(&sender, &many, &mut buf).unwrap();
    let max = NodesBroadcast::MAX_ENTRIES_PER_FRAME;
    assert_eq!(n, 1 + ALIAS_LENGTH + max * NodesRoutingEntry::ENCODED_LENGTH);
    assert_eq!(NodesBroadcast::try_parse(&buf[..n]).unwrap().entry_count(), max);
}
