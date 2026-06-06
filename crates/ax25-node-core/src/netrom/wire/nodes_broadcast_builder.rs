//! NODES routing-broadcast origination — the TX counterpart to [`super::broadcast`]'s
//! parser. The caller transmits each built info field as a UI frame (PID 0xCF,
//! AX.25 destination the literal text callsign `NODES`).
//!
//! Ports `Packet.NetRom.Wire.NodesBroadcastBuilder`. Where the desktop builder
//! returns a heap list of byte arrays, this `no_std`/allocation-free version
//! writes **one** frame's info field (the 0xFF signature, the sender's 6-octet
//! alias, then up to [`NodesBroadcast::MAX_ENTRIES_PER_FRAME`] entries) into a
//! caller-provided buffer. A routing table with more advertisable destinations
//! than fit in one frame is dumped by calling this once per 11-entry chunk — each
//! frame is a self-contained broadcast (the receiver merges by destination, so
//! frame boundaries don't matter). Construction stays strict/canonical.

use super::broadcast::NodesBroadcast;
use super::callsign::{write_alias, write_shifted, Alias, ALIAS_LENGTH, SHIFTED_LENGTH};
use super::entry::NodesRoutingEntry;
use crate::ax25::Callsign;

/// Octets the per-frame header occupies: the 0xFF signature + the 6-octet alias.
const HEADER_LEN: usize = 1 + ALIAS_LENGTH; // 7

/// One destination entry to advertise: the destination node + its alias, the
/// best-neighbour we forward through, and the quality we advertise for it.
/// Mirrors the C# `NodesBroadcastBuilder.Entry`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NodesAdvertisementEntry {
    /// The destination node's callsign.
    pub destination: Callsign,
    /// The destination node's alias / mnemonic (may be empty).
    pub destination_alias: Alias,
    /// The neighbour we forward through to reach it.
    pub best_neighbour: Callsign,
    /// The quality to advertise (0..=255).
    pub quality: u8,
}

/// The maximum info-field length one NODES frame can occupy — header + a full
/// 11-entry payload. A `[u8; MAX_NODES_FRAME_LEN]` buffer always holds one frame.
pub const MAX_NODES_FRAME_LEN: usize =
    HEADER_LEN + NodesBroadcast::MAX_ENTRIES_PER_FRAME * NodesRoutingEntry::ENCODED_LENGTH;

/// Write one NODES broadcast info field into the front of `dst`: the 0xFF
/// signature, `sender_alias`, then up to [`NodesBroadcast::MAX_ENTRIES_PER_FRAME`]
/// entries taken from the front of `entries` (any excess is ignored — chunk at the
/// call site). Returns the encoded length, or `None` if `dst` is too short. An
/// empty `entries` yields a header-only frame (a node announcing its presence with
/// nothing to advertise yet).
pub fn write_nodes_frame(
    sender_alias: &Alias,
    entries: &[NodesAdvertisementEntry],
    dst: &mut [u8],
) -> Option<usize> {
    let take = entries.len().min(NodesBroadcast::MAX_ENTRIES_PER_FRAME);
    let total = HEADER_LEN + take * NodesRoutingEntry::ENCODED_LENGTH;
    if dst.len() < total {
        return None;
    }

    dst[0] = NodesBroadcast::SIGNATURE;
    write_alias(sender_alias, &mut dst[1..])?;

    let mut off = HEADER_LEN;
    for e in &entries[..take] {
        write_shifted(&e.destination, &mut dst[off..])?;
        write_alias(&e.destination_alias, &mut dst[off + SHIFTED_LENGTH..])?;
        write_shifted(
            &e.best_neighbour,
            &mut dst[off + SHIFTED_LENGTH + ALIAS_LENGTH..],
        )?;
        dst[off + NodesRoutingEntry::ENCODED_LENGTH - 1] = e.quality;
        off += NodesRoutingEntry::ENCODED_LENGTH;
    }
    Some(total)
}
