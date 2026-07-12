//! Cross-stack golden-vector runner for the AX.25 wire codecs.
//!
//! This is the pico-node leg of the three-stack parity contract (C# `Packet.*`
//! authoritative <-> TS `@packet-net/ax25` <-> Rust pico-node). It consumes the
//! shared golden-vector corpus under `vectors/` — the same *bytes* the C# and TS
//! suites assert — and drives them through this crate's codec, proving byte
//! identity rather than each stack re-deriving its own expected values.
//!
//! The corpus is embedded at compile time with `include_str!`, so the test needs
//! no runtime file IO and stays fully offline / hermetic — matching the crate's
//! "host tests run offline" rule. `serde_json` is a **dev-dependency only**; it
//! never reaches the `no_std` / firmware graph.
//!
//! Per vector: hex-decode `wire` -> `Frame::decode` -> assert the decoded fields
//! match -> `Frame::encode` -> assert the bytes equal `wire` (round-trip). A
//! re-encode that differs from `wire` is a genuine parity break, not a test bug.

use ax25_node_core::ax25::frame::Frame;
use serde::Deserialize;

/// One shared corpus file: a named set of vectors for a single codec.
#[derive(Deserialize)]
struct VectorSet {
    /// The set name — must match the `[vector_sets.<name>]` key in
    /// `parity-manifest.toml`.
    set: String,
    /// The vectors themselves.
    vectors: Vec<Vector>,
}

/// A single golden vector: input wire bytes plus the expected decoded fields.
#[derive(Deserialize)]
struct Vector {
    /// Stable identifier used in assertion messages.
    name: String,
    /// The frame body as hex (KISS-delivered form: no HDLC flags, no FCS).
    /// Whitespace and `_` in the string are ignored for readability.
    wire: String,
    /// The fields the codec must decode out of `wire`.
    fields: Fields,
}

/// The expected decode of a vector.
#[derive(Deserialize)]
struct Fields {
    /// Destination callsign + SSID.
    dest: Call,
    /// Source callsign + SSID.
    source: Call,
    /// Digipeater path in order (may be empty).
    #[serde(default)]
    digipeaters: Vec<Call>,
    /// True if the frame is a command (dest C-bit set, source C-bit clear).
    command: bool,
    /// The (first) control octet.
    control: u8,
    /// The PID octet, if present (I and UI frames carry one).
    pid: Option<u8>,
    /// The information field as hex (empty string == no info).
    info_hex: String,
}

/// A callsign + SSID expectation.
#[derive(Deserialize)]
struct Call {
    /// Base callsign, uppercase A-Z / 0-9 (no SSID suffix).
    call: String,
    /// Secondary station identifier, 0-15.
    ssid: u8,
}

/// Decode a hex string into bytes, ignoring ASCII whitespace and `_` separators.
fn hex_decode(s: &str) -> Vec<u8> {
    let nibbles: Vec<u8> = s
        .bytes()
        .filter(|b| !b.is_ascii_whitespace() && *b != b'_')
        .collect();
    assert!(
        nibbles.len() % 2 == 0,
        "hex string has an odd number of digits: {s:?}"
    );
    nibbles
        .chunks(2)
        .map(|pair| (hex_nibble(pair[0]) << 4) | hex_nibble(pair[1]))
        .collect()
}

/// Map one hex ASCII digit to its 0-15 value.
fn hex_nibble(b: u8) -> u8 {
    match b {
        b'0'..=b'9' => b - b'0',
        b'a'..=b'f' => b - b'a' + 10,
        b'A'..=b'F' => b - b'A' + 10,
        other => panic!("not a hex digit: {:?}", other as char),
    }
}

/// The modulo-8 AX.25 frame corpus, embedded at compile time.
const AX25_MOD8: &str = include_str!("../../../vectors/ax25-mod8.json");

#[test]
fn ax25_mod8_golden_vectors_round_trip() {
    let set: VectorSet =
        serde_json::from_str(AX25_MOD8).expect("vectors/ax25-mod8.json must parse");
    assert_eq!(set.set, "ax25_mod8", "unexpected set name in corpus");
    assert!(!set.vectors.is_empty(), "corpus must contain at least one vector");

    for v in &set.vectors {
        let wire = hex_decode(&v.wire);

        // Decode.
        let frame = Frame::decode(&wire)
            .unwrap_or_else(|e| panic!("vector `{}` failed to decode: {e:?}", v.name));

        // Destination.
        assert_eq!(
            frame.destination.callsign.base(),
            v.fields.dest.call.as_bytes(),
            "vector `{}` dest callsign",
            v.name
        );
        assert_eq!(
            frame.destination.callsign.ssid(),
            v.fields.dest.ssid,
            "vector `{}` dest ssid",
            v.name
        );

        // Source.
        assert_eq!(
            frame.source.callsign.base(),
            v.fields.source.call.as_bytes(),
            "vector `{}` source callsign",
            v.name
        );
        assert_eq!(
            frame.source.callsign.ssid(),
            v.fields.source.ssid,
            "vector `{}` source ssid",
            v.name
        );

        // Digipeaters.
        assert_eq!(
            frame.digipeaters.len(),
            v.fields.digipeaters.len(),
            "vector `{}` digipeater count",
            v.name
        );
        for (i, (got, want)) in frame
            .digipeaters
            .iter()
            .zip(&v.fields.digipeaters)
            .enumerate()
        {
            assert_eq!(
                got.callsign.base(),
                want.call.as_bytes(),
                "vector `{}` digipeater[{i}] callsign",
                v.name
            );
            assert_eq!(
                got.callsign.ssid(),
                want.ssid,
                "vector `{}` digipeater[{i}] ssid",
                v.name
            );
        }

        // Control / command-response / PID / info.
        assert_eq!(frame.control, v.fields.control, "vector `{}` control", v.name);
        assert_eq!(
            frame.is_command(),
            v.fields.command,
            "vector `{}` command/response bit",
            v.name
        );
        assert_eq!(frame.pid, v.fields.pid, "vector `{}` pid", v.name);
        assert_eq!(
            frame.info,
            hex_decode(&v.fields.info_hex),
            "vector `{}` info field",
            v.name
        );

        // Round-trip: re-encoding the decoded frame must reproduce the exact wire
        // bytes. A mismatch here is a real cross-stack parity break.
        let reencoded = frame.encode();
        assert_eq!(
            reencoded, wire,
            "vector `{}` re-encode differs from wire (PARITY BREAK)",
            v.name
        );
    }
}
