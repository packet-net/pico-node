//! Cross-stack golden-vector runner for the AX.25 v2.2 XID information-field codec.
//!
//! The XID leg of the three-stack parity contract (C# `Packet.Ax25.Xid`
//! authoritative <-> TS <-> Rust pico-node). It consumes `vectors/xid.json` — the
//! same *bytes* C#'s `XidInfoFieldTests` pins — and drives them through this
//! crate's `xid::info_field` codec, proving byte identity rather than re-deriving
//! expected values.
//!
//! Per vector: hex-decode `info_hex` -> `info_field::parse` -> assert the decoded
//! parameters -> when `roundtrip`, `info_field::encode` -> assert the bytes equal
//! `info_hex`. `serde_json` is a dev-dependency only; it never reaches the
//! `no_std` / firmware graph.

use ax25_node_core::ax25::xid::{info_field, RejectMode};
use serde::Deserialize;

#[derive(Deserialize)]
struct XidSet {
    set: String,
    vectors: Vec<XidVector>,
}

#[derive(Deserialize)]
struct XidVector {
    name: String,
    info_hex: String,
    #[serde(default)]
    roundtrip: bool,
    expect: XidExpect,
}

/// The expected decode of a vector. Every field is optional — only the present
/// ones are asserted (a vector need not pin parameters it doesn't carry).
#[derive(Deserialize, Default)]
struct XidExpect {
    half_duplex: Option<bool>,
    /// `"srej"` or `"rej"`.
    reject: Option<String>,
    modulo128: Option<bool>,
    srej_multiframe: Option<bool>,
    segmenter: Option<bool>,
    i_field_length_rx_bits: Option<u32>,
    window_size_rx: Option<u32>,
    ack_timer_millis: Option<u32>,
    retries: Option<u32>,
    /// When true, assert every decoded parameter is absent (None).
    #[serde(default)]
    all_none: bool,
}

fn hex_decode(s: &str) -> Vec<u8> {
    let nibbles: Vec<u8> = s
        .bytes()
        .filter(|b| !b.is_ascii_whitespace() && *b != b'_')
        .collect();
    assert!(
        nibbles.len().is_multiple_of(2),
        "hex string has an odd number of digits: {s:?}"
    );
    nibbles
        .chunks(2)
        .map(|pair| (hex_nibble(pair[0]) << 4) | hex_nibble(pair[1]))
        .collect()
}

fn hex_nibble(b: u8) -> u8 {
    match b {
        b'0'..=b'9' => b - b'0',
        b'a'..=b'f' => b - b'a' + 10,
        b'A'..=b'F' => b - b'A' + 10,
        other => panic!("not a hex digit: {:?}", other as char),
    }
}

const XID: &str = include_str!("../../../vectors/xid.json");

#[test]
fn xid_golden_vectors_round_trip() {
    let set: XidSet = serde_json::from_str(XID).expect("vectors/xid.json must parse");
    assert_eq!(set.set, "xid", "unexpected set name in corpus");
    assert!(!set.vectors.is_empty(), "corpus must contain at least one vector");

    for v in &set.vectors {
        let wire = hex_decode(&v.info_hex);
        let parsed = info_field::parse(&wire)
            .unwrap_or_else(|| panic!("vector `{}` failed to parse", v.name));

        if v.expect.all_none {
            assert!(parsed.classes_of_procedures.is_none(), "vector `{}` classes", v.name);
            assert!(parsed.hdlc_optional_functions.is_none(), "vector `{}` hdlc", v.name);
            assert!(parsed.i_field_length_rx_bits.is_none(), "vector `{}` n1", v.name);
            assert!(parsed.window_size_rx.is_none(), "vector `{}` k", v.name);
            assert!(parsed.ack_timer_millis.is_none(), "vector `{}` t1", v.name);
            assert!(parsed.retries.is_none(), "vector `{}` n2", v.name);
        }

        if let Some(want) = v.expect.half_duplex {
            assert_eq!(
                parsed.classes_of_procedures.expect("classes present").half_duplex,
                want,
                "vector `{}` half_duplex",
                v.name
            );
        }
        if let Some(want) = &v.expect.reject {
            let got = parsed.hdlc_optional_functions.expect("hdlc present").reject;
            let want = match want.as_str() {
                "srej" => RejectMode::SelectiveReject,
                "rej" => RejectMode::ImplicitReject,
                other => panic!("vector `{}` bad reject `{other}`", v.name),
            };
            assert_eq!(got, want, "vector `{}` reject", v.name);
        }
        if let Some(want) = v.expect.modulo128 {
            assert_eq!(
                parsed.hdlc_optional_functions.expect("hdlc present").modulo128,
                want,
                "vector `{}` modulo128",
                v.name
            );
        }
        if let Some(want) = v.expect.srej_multiframe {
            assert_eq!(
                parsed.hdlc_optional_functions.expect("hdlc present").srej_multiframe,
                want,
                "vector `{}` srej_multiframe",
                v.name
            );
        }
        if let Some(want) = v.expect.segmenter {
            assert_eq!(
                parsed.hdlc_optional_functions.expect("hdlc present").segmenter_reassembler,
                want,
                "vector `{}` segmenter",
                v.name
            );
        }
        if let Some(want) = v.expect.i_field_length_rx_bits {
            assert_eq!(parsed.i_field_length_rx_bits, Some(want), "vector `{}` n1 bits", v.name);
        }
        if let Some(want) = v.expect.window_size_rx {
            assert_eq!(parsed.window_size_rx, Some(want), "vector `{}` window", v.name);
        }
        if let Some(want) = v.expect.ack_timer_millis {
            assert_eq!(parsed.ack_timer_millis, Some(want), "vector `{}` t1", v.name);
        }
        if let Some(want) = v.expect.retries {
            assert_eq!(parsed.retries, Some(want), "vector `{}` n2", v.name);
        }

        if v.roundtrip {
            let reencoded = info_field::encode(&parsed);
            assert_eq!(
                reencoded, wire,
                "vector `{}` re-encode differs from info_hex (PARITY BREAK)",
                v.name
            );
        }
    }
}
