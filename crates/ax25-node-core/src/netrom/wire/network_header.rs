//! The NET/ROM L3 network header — 15 octets prepended to every inter-node
//! datagram carried over a connected-mode AX.25 interlink (PID 0xCF). It names
//! the end-to-end origin and destination *nodes* (not the hop-by-hop AX.25
//! addresses, which are the interlink's own) and carries the hop-limit TTL a
//! forwarding node decrements.
//!
//! Layout (canonical NET/ROM appendix), 15 octets:
//! ```text
//!   [7] origin node callsign       (AX.25 shifted form)
//!   [7] destination node callsign  (AX.25 shifted form)
//!   [1] time-to-live               (hop limit; decremented per node; 0 → discard)
//! ```
//!
//! Ports `Packet.NetRom.Wire.NetRomNetworkHeader`. `no_std`, allocation-free.

use super::callsign::{try_read_shifted, write_shifted, SHIFTED_LENGTH};
use crate::ax25::Callsign;

/// Octets the network header occupies on the wire.
pub const NETWORK_HEADER_LEN: usize = SHIFTED_LENGTH + SHIFTED_LENGTH + 1; // 15

/// The canonical default initial time-to-live (BPQ's `L3TIMETOLIVE` default).
pub const DEFAULT_TIME_TO_LIVE: u8 = 25;

/// The NET/ROM L3 network header.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NetRomNetworkHeader {
    /// The end-to-end origin node.
    pub origin: Callsign,
    /// The end-to-end destination node.
    pub destination: Callsign,
    /// Hop-limit counter; a forwarding node decrements it and discards at 0.
    pub time_to_live: u8,
}

impl NetRomNetworkHeader {
    /// A copy of this header with the TTL decremented by one (saturating at 0, so
    /// it never underflows). The caller checks the result is `> 0` before
    /// forwarding (a header arriving at TTL 1 decrements to 0 and must not be
    /// forwarded).
    pub const fn decremented(&self) -> Self {
        Self {
            origin: self.origin,
            destination: self.destination,
            time_to_live: self.time_to_live.saturating_sub(1),
        }
    }

    /// Encode this header into the front of `dst` (≥ [`NETWORK_HEADER_LEN`]).
    /// Returns `None` only if `dst` is too short.
    pub fn encode(&self, dst: &mut [u8]) -> Option<()> {
        if dst.len() < NETWORK_HEADER_LEN {
            return None;
        }
        write_shifted(&self.origin, &mut dst[0..])?;
        write_shifted(&self.destination, &mut dst[SHIFTED_LENGTH..])?;
        dst[SHIFTED_LENGTH * 2] = self.time_to_live;
        Some(())
    }

    /// Decode a 15-octet network header from the front of `src`. Total: returns
    /// `None` if the slice is too short or either callsign fails to decode.
    pub fn decode(src: &[u8]) -> Option<Self> {
        if src.len() < NETWORK_HEADER_LEN {
            return None;
        }
        let origin = try_read_shifted(&src[0..])?;
        let destination = try_read_shifted(&src[SHIFTED_LENGTH..])?;
        Some(Self {
            origin,
            destination,
            time_to_live: src[SHIFTED_LENGTH * 2],
        })
    }
}
