//! A parsed NET/ROM NODES routing broadcast — the L3 content carried in the
//! information field of a UI frame (PID 0xCF, AX.25 destination the literal text
//! callsign `NODES`).
//!
//! Ports `Packet.NetRom.Wire.NodesBroadcast`. Information-field layout (canonical
//! NET/ROM appendix):
//!
//! ```text
//!   [1]  0xFF signature byte
//!   [6]  sender's alias / mnemonic (plain ASCII, space-padded)
//!   then up to 11 × 21-octet destination entries (NodesRoutingEntry)
//! ```
//!
//! A node's full routing table is dumped across as many UI frames as needed, each
//! frame carrying ≤ 11 entries. This type models *one* such frame's content; a
//! multi-frame dump produces several [`NodesBroadcast`] instances, all merged into
//! the routing table independently (the table keys on destination, so frame
//! boundaries don't matter to the merge).
//!
//! Parsing is read-only and total: arbitrary bytes never panic, they return
//! `None`. Divergence tolerance (trailing partial entry, empty list) is gated by
//! [`NetRomParseOptions`] — strict by default at the byte boundary, lenient on the
//! convenience [`NodesBroadcast::try_parse`] path used for promiscuous ingest.
//!
//! `no_std`, allocation-free: the entry list is a fixed-capacity inline array of
//! [`NodesBroadcast::MAX_ENTRIES_PER_FRAME`] entries (the canonical 11-per-frame
//! cap), not a heap `Vec`.

use super::callsign::{read_alias, Alias, ALIAS_LENGTH};
use super::entry::NodesRoutingEntry;
use super::options::NetRomParseOptions;

/// A parsed NODES broadcast — the sender's alias plus the destination entries
/// carried in one UI frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NodesBroadcast {
    sender_alias: Alias,
    entries: [Option<NodesRoutingEntry>; NodesBroadcast::MAX_ENTRIES_PER_FRAME],
    count: u8,
}

impl NodesBroadcast {
    /// The NET/ROM NODES-broadcast signature byte that opens the info field.
    pub const SIGNATURE: u8 = 0xFF;

    /// The literal AX.25 destination callsign a NODES broadcast is addressed to.
    pub const NODES_DESTINATION: &'static str = "NODES";

    /// Maximum destination entries the canonical format packs into one UI frame.
    pub const MAX_ENTRIES_PER_FRAME: usize = 11;

    /// signature + 6-byte alias = 7.
    const HEADER_LENGTH: usize = 1 + ALIAS_LENGTH;

    /// The broadcasting node's alias / mnemonic (may be empty).
    pub fn sender_alias(&self) -> Alias {
        self.sender_alias
    }

    /// The destination entries carried in this frame (0..=11).
    pub fn entries(&self) -> impl Iterator<Item = &NodesRoutingEntry> {
        self.entries[..self.count as usize]
            .iter()
            .filter_map(Option::as_ref)
    }

    /// Number of destination entries carried in this frame (0..=11).
    pub fn entry_count(&self) -> usize {
        self.count as usize
    }

    /// Try to parse a NODES broadcast from a UI frame's information field, using
    /// lenient options (the promiscuous-ingest default — see [`NetRomParseOptions::LENIENT`]).
    /// Returns `None` (never panics) on any malformed input.
    pub fn try_parse(info: &[u8]) -> Option<Self> {
        Self::try_parse_with(info, NetRomParseOptions::LENIENT)
    }

    /// Try to parse a NODES broadcast from a UI frame's information field, applying
    /// `options` for the strict-vs-lenient divergence choices. Returns `None`
    /// (never panics) on any malformed input.
    pub fn try_parse_with(info: &[u8], options: NetRomParseOptions) -> Option<Self> {
        // Need at least the signature + 6-byte alias.
        if info.len() < Self::HEADER_LENGTH {
            return None;
        }

        // Signature byte gates the whole frame — a non-0xFF first octet means this
        // is not a NODES broadcast (the canonical "wrong signature → ignore"
        // heuristic).
        if info[0] != Self::SIGNATURE {
            return None;
        }

        let sender_alias = read_alias(&info[1..]);

        let body = &info[Self::HEADER_LENGTH..];
        let entry_count = body.len() / NodesRoutingEntry::ENCODED_LENGTH;
        let remainder = body.len() - (entry_count * NodesRoutingEntry::ENCODED_LENGTH);

        // A non-zero remainder means the routing region isn't a whole number of
        // 21-byte entries — either trailing pad / a clipped frame, or a malformed
        // dump. Strict rejects; lenient keeps the whole entries it can read.
        if remainder != 0 && !options.allow_trailing_partial_entry {
            return None;
        }

        // Cap at the canonical 11-per-frame: a frame claiming more than that is out
        // of spec, so we ignore the surplus rather than trust it.
        let take = entry_count.min(Self::MAX_ENTRIES_PER_FRAME);

        if take == 0 && !options.allow_empty_destination_list {
            return None;
        }

        let mut entries: [Option<NodesRoutingEntry>; Self::MAX_ENTRIES_PER_FRAME] =
            [None; Self::MAX_ENTRIES_PER_FRAME];
        let mut count = 0usize;
        let mut offset = 0usize;
        for _ in 0..take {
            let slice = &body[offset..offset + NodesRoutingEntry::ENCODED_LENGTH];
            match NodesRoutingEntry::try_parse(slice) {
                Some(entry) => {
                    entries[count] = Some(entry);
                    count += 1;
                }
                None => {
                    // A single undecodable entry shouldn't sink the frame under
                    // lenient ingest — skip it and keep parsing the rest. Under
                    // strict, a bad entry is a malformed broadcast.
                    if !options.allow_trailing_partial_entry {
                        return None;
                    }
                }
            }
            offset += NodesRoutingEntry::ENCODED_LENGTH;
        }

        Some(Self {
            sender_alias,
            entries,
            count: count as u8,
        })
    }
}
