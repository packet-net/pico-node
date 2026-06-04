//! The immutable read-side model of the learned NET/ROM routing table.
//!
//! Ports `Packet.NetRom.Routing.NetRomRoutingModel` (the `NetRomRoute`,
//! `NetRomDestination`, `NetRomNeighbour`, and `NetRomRoutingSnapshot` records).
//! This is the model the `Nodes` console command, a future MCP `network_topology`
//! tool, and the web monitor all consume.
//!
//! `no_std`, allocation-free. Where the C# snapshot hands out `IReadOnlyList`s, the
//! Rust [`crate::netrom::NetRomRoutingTable`] exposes its live state through
//! borrow-based accessors + visitor callbacks (no heap), so these are the *value*
//! shapes a consumer copies out — each is small and `Copy`.

use crate::ax25::Callsign;

use crate::netrom::wire::Alias;

/// One learned route to a NET/ROM destination: the next-hop neighbour to forward
/// through, the quality we derived for it, and its obsolescence count.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NetRomRoute {
    /// The neighbour we forward through for this route.
    pub neighbour: Callsign,
    /// Our derived quality for this route (0..=255), best first within a destination.
    pub quality: u8,
    /// Obsolescence count; decremented each sweep, purged at 0.
    pub obsolescence: u8,
}

/// A destination known to the table — its callsign + alias and a copy of its best
/// route. The full route set is iterated via the table's visitor accessors; this
/// value carries the active ([`NetRomDestination::best_route`]) route, which is
/// what every surfacing consumer needs first.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NetRomDestination {
    /// The destination node's callsign.
    pub destination: Callsign,
    /// The destination node's alias / mnemonic (may be empty).
    pub alias: Alias,
    /// The highest-quality (active) route to this destination, if any.
    pub best_route: Option<NetRomRoute>,
    /// How many routes are kept for this destination (≤ the per-destination cap).
    pub route_count: u8,
}

/// A directly-heard NET/ROM neighbour — a node whose NODES broadcast we received
/// firsthand, with the path quality we assume to it and the port we heard it on.
///
/// Mirrors the canonical neighbour list (the `ROUTES` command), restricted to what
/// read-only ingest can know (we don't probe links, so quality is the assumed
/// default-port quality, and there are no digipeaters or lock state). `LastHeard`
/// is an opaque caller-supplied `u64` tick (the embedding's monotonic time — no
/// wall-clock in the core, matching the `TimerService` injection pattern).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NetRomNeighbour {
    /// The neighbour's callsign.
    pub neighbour: Callsign,
    /// The neighbour's alias / mnemonic, as it announced (may be empty).
    pub alias: Alias,
    /// The node-host port id we heard it on (a small fixed string — see
    /// [`crate::netrom::PortId`]).
    pub port_id: crate::netrom::PortId,
    /// The path quality we assume to this neighbour (0..=255).
    pub path_quality: u8,
    /// The opaque caller tick at which we last heard a broadcast from it.
    pub last_heard: u64,
}
