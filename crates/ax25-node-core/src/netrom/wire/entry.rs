//! One destination entry inside a NET/ROM NODES broadcast — a 21-octet record.
//!
//! Ports `Packet.NetRom.Wire.NodesRoutingEntry`. It advertises "I (the
//! broadcasting node) can reach [`NodesRoutingEntry::destination`] (alias
//! [`NodesRoutingEntry::destination_alias`]) via
//! [`NodesRoutingEntry::best_neighbour`] at quality
//! [`NodesRoutingEntry::best_quality`]."
//!
//! Layout (canonical NET/ROM appendix), 21 octets:
//!
//! ```text
//!   [7] destination callsign    (AX.25 shifted form)
//!   [6] destination alias       (plain ASCII, space-padded, no SSID)
//!   [7] best-neighbour callsign (AX.25 shifted form)
//!   [1] best quality            (0 worst … 255 best)
//! ```
//!
//! The quality is the *advertised* quality as the originator sees it; the
//! receiving node combines it multiplicatively with its own path quality to the
//! originator to derive the route quality it stores — see
//! [`crate::netrom::routing::quality`].
//!
//! `no_std`, allocation-free, `Copy`.

use crate::ax25::Callsign;

use super::callsign::{read_alias, try_read_shifted, Alias, ALIAS_LENGTH, SHIFTED_LENGTH};

/// One destination entry advertised in a NODES broadcast (21 octets on the wire).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NodesRoutingEntry {
    /// The destination node this entry advertises a route to.
    pub destination: Callsign,
    /// The destination node's alias / mnemonic (may be empty).
    pub destination_alias: Alias,
    /// The neighbour the originator forwards through to reach
    /// [`Self::destination`] — the originator's own chosen best next hop.
    pub best_neighbour: Callsign,
    /// The originator's quality for this route (0 worst … 255 best).
    pub best_quality: u8,
}

impl NodesRoutingEntry {
    /// Octets one entry occupies on the wire (= 21).
    pub const ENCODED_LENGTH: usize = SHIFTED_LENGTH  // 7  destination callsign
        + ALIAS_LENGTH                                // 6  destination alias
        + SHIFTED_LENGTH                              // 7  best-neighbour callsign
        + 1; // 1  best quality

    /// Decode one 21-octet entry. Returns `None` if the span is too short or either
    /// callsign field fails to decode.
    pub fn try_parse(source: &[u8]) -> Option<Self> {
        if source.len() < Self::ENCODED_LENGTH {
            return None;
        }

        let mut offset = 0;
        let destination = try_read_shifted(&source[offset..])?;
        offset += SHIFTED_LENGTH;

        let destination_alias = read_alias(&source[offset..]);
        offset += ALIAS_LENGTH;

        let best_neighbour = try_read_shifted(&source[offset..])?;
        offset += SHIFTED_LENGTH;

        let best_quality = source[offset];

        Some(Self {
            destination,
            destination_alias,
            best_neighbour,
            best_quality,
        })
    }
}
