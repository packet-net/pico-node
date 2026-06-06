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
