//! # NET/ROM — read-only "NET/ROM aware" slice
//!
//! The Rust port of the C# `Packet.NetRom` library + `Packet.Node.Core.NetRom`
//! service (packet.net PR #303), brought to the embedded node: **hear NODES
//! broadcasts, build a routing table, surface it** — and **originate nothing on
//! the air** (no TX, no L4 circuits, no NODES origination).
//!
//! ## Layout (mirrors the C# split)
//!
//! - [`wire`] — the byte-level NODES-broadcast codec (`Packet.NetRom.Wire`):
//!   [`wire::NetRomParseOptions`] (the named-divergence flags), the two callsign /
//!   alias field decoders, the 21-byte [`wire::NodesRoutingEntry`], and
//!   [`wire::NodesBroadcast`].
//! - [`routing`] — the L3 routing model (`Packet.NetRom.Routing`): the
//!   multiplicative [`routing::quality`] decay, the [`routing::NetRomRoutingOptions`]
//!   knobs, the fixed-capacity [`routing::NetRomRoutingTable`], and the read-side
//!   [`routing::model`].
//! - [`NetRomService`] — the node-level observer (`Packet.Node.Core.NetRom.NetRomService`):
//!   a read-only frame tap that hears NODES on every port and maintains a routing
//!   table, plus the [`NetRomRoutingView`] read accessor (the
//!   `INetRomRoutingView` equivalent).
//!
//! ## The read-only guarantee
//!
//! The service's only interaction with a port is observing inbound frames via
//! [`NetRomService::observe_frame`] — a pure observation tap, called by the
//! firmware *before* address filtering (so it hears NODES broadcasts, which are
//! addressed to the literal text callsign `NODES`, not to us). It never gates,
//! delays, alters, or emits a frame; it cannot post into a session. A NODES storm
//! mid-QSO is consumed by the tap and leaves the connected session untouched —
//! [`NetRomService::observe_frame`] returns nothing and mutates only the routing
//! table.
//!
//! ## `no_std` / capacity
//!
//! Everything is `#![no_std]`-clean and allocation-free: integer quality maths
//! (no FPU on the M0+), and fixed-capacity (`[Option<…>; N]`) structures with
//! compile-time const-generic caps — not heap `Vec`s/maps. The default
//! [`NetRomService`] type alias sizes the caps for a node ([`MAX_DESTINATIONS`] /
//! [`MAX_ROUTES_PER_DEST`] / [`MAX_NEIGHBOURS`]).

pub mod connector;
pub mod forwarding;
pub mod originator;
pub mod routing;
pub mod transport;
pub mod wire;

use crate::ax25::{Callsign, Frame, PID_NETROM};

pub use connector::{
    InterlinkSend, NetRomConnection, NetRomConnector, NetRomConnectorOptions, NetRomNoRoute,
};
pub use forwarding::{decide_forward, ForwardDecision, ForwardMode, ForwardOutcome};
pub use originator::{NetRomOriginator, NetRomOriginatorOptions};
pub use routing::{
    NetRomDestination, NetRomNeighbour, NetRomRoute, NetRomRoutingOptions, NetRomRoutingTable,
    NetRomRoutingView,
};
pub use transport::{
    CircuitEvent, CircuitKey, CircuitManager, IncomingCircuit, NetRomCircuit,
    NetRomCircuitCloseReason, NetRomCircuitOptions, NetRomCircuitState, OutboundPacket,
};
pub use wire::{NetRomParseOptions, NodesBroadcast, NodesRoutingEntry};

/// Maximum length of a node-host port id we track (e.g. `"vhf"`, `"kiss-tcp"`).
pub const MAX_PORT_ID_LEN: usize = 16;

/// A node-host port id — a small fixed-capacity ASCII string (no heap `String`).
/// The C# service keys neighbours by a `string` port id; on the M0+ that is this
/// `Copy` value. Over-long input is truncated.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct PortId {
    buf: [u8; MAX_PORT_ID_LEN],
    len: u8,
}

impl PortId {
    /// Build a port id from text, truncating to [`MAX_PORT_ID_LEN`] bytes. Keeps any
    /// bytes (a port id is opaque) but the common case is short ASCII.
    pub fn from_str_lossy(s: &str) -> Self {
        let mut buf = [0u8; MAX_PORT_ID_LEN];
        let bytes = s.as_bytes();
        let n = bytes.len().min(MAX_PORT_ID_LEN);
        buf[..n].copy_from_slice(&bytes[..n]);
        Self { buf, len: n as u8 }
    }

    /// The significant id bytes.
    pub fn as_bytes(&self) -> &[u8] {
        &self.buf[..self.len as usize]
    }

    /// The id as a `&str` (best-effort UTF-8; `""` if the truncation split a char).
    pub fn as_str(&self) -> &str {
        core::str::from_utf8(self.as_bytes()).unwrap_or("")
    }
}

// ── Default capacity caps for the node-sized table (the firmware uses these) ──

/// Default maximum distinct destinations the node table holds. The C# default is
/// 1024; the embedded node is smaller, but this is generous for a Pico-class node
/// on a real network and is the memory-safety cap against an unbounded list.
pub const MAX_DESTINATIONS: usize = 128;

/// Maximum routes kept per destination — the canonical NET/ROM default of 3.
pub const MAX_ROUTES_PER_DEST: usize = 3;

/// Default maximum directly-heard neighbours tracked.
pub const MAX_NEIGHBOURS: usize = 32;

/// The node-sized routing table type (the caps the firmware uses).
pub type NodeRoutingTable =
    NetRomRoutingTable<MAX_DESTINATIONS, MAX_ROUTES_PER_DEST, MAX_NEIGHBOURS>;

/// Why a heard frame was not ingested as a NODES broadcast (returned by
/// [`NetRomService::observe_frame`] for the caller's diagnostics — it is *not* an
/// error, just an observation outcome).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ObserveOutcome {
    /// Service is disabled — nothing observed.
    Disabled,
    /// Not a UI frame (cheap first gate).
    NotUi,
    /// PID was not 0xCF (not NET/ROM).
    NotNetRom,
    /// Destination was not the literal text callsign `NODES`.
    NotNodesDestination,
    /// It was a NODES broadcast but its info field did not parse.
    Unparseable,
    /// A NODES broadcast was parsed and ingested into the routing table.
    Ingested {
        /// How many destination entries the broadcast carried.
        entries: usize,
    },
}

/// The node-level NET/ROM service — the read-only "NET/ROM aware" observer.
///
/// Ports `Packet.Node.Core.NetRom.NetRomService` to the embedded node. It owns a
/// [`NodeRoutingTable`] and exposes:
/// - [`Self::observe_frame`] — the read-only frame tap (call it for every inbound
///   frame, *before* address filtering),
/// - [`Self::sweep`] — the obsolescence sweep (call at the broadcast interval),
/// - and the [`NetRomRoutingView`] read accessor methods ([`Self::enabled`],
///   [`Self::for_each_neighbour`], [`Self::for_each_destination`], …).
///
/// It transmits nothing and cannot disturb a session — see the module docs' "read-
/// only guarantee". `no_std`, allocation-free.
#[derive(Debug)]
pub struct NetRomService {
    enabled: bool,
    table: NodeRoutingTable,
}

impl NetRomService {
    /// Construct an enabled service with the canonical default routing options.
    pub fn new() -> Self {
        Self::with_options(true, NetRomRoutingOptions::DEFAULT)
    }

    /// Construct a service with an explicit enabled flag + routing options. When
    /// disabled, the table stays empty and [`Self::observe_frame`] is a no-op
    /// returning [`ObserveOutcome::Disabled`] — matching the C# disabled-service
    /// behaviour.
    pub fn with_options(enabled: bool, options: NetRomRoutingOptions) -> Self {
        Self {
            enabled,
            table: NetRomRoutingTable::new(options),
        }
    }

    /// True if NET/ROM awareness is enabled on this node. When false, the table is
    /// always empty. (The `INetRomRoutingView.Enabled` equivalent.)
    pub fn enabled(&self) -> bool {
        self.enabled
    }

    /// The read-only frame tap. Call this for **every inbound frame**, *before*
    /// address filtering (so NODES broadcasts — addressed to the literal callsign
    /// `NODES`, not to us — are heard). `my_call` is the receiving port's local
    /// callsign (for the trivial-loop guard); `port_id` is the node-host port the
    /// frame arrived on; `now` is the embedding's monotonic time (for the
    /// neighbour's last-heard stamp).
    ///
    /// **Observation-only.** It reads the frame and may update the routing table; it
    /// never transmits, never alters the frame, and returns no frames to send.
    /// Mirrors C# `NetRomService.OnFrameTraced`. Total: never panics on any frame.
    pub fn observe_frame(
        &mut self,
        frame: &Frame,
        my_call: Callsign,
        port_id: PortId,
        now: u64,
    ) -> ObserveOutcome {
        if !self.enabled {
            return ObserveOutcome::Disabled;
        }

        // NODES broadcasts are UI frames, PID 0xCF, AX.25 destination the literal
        // text callsign "NODES". Cheap gates first (mirrors the C# order).
        if !frame.is_ui() {
            return ObserveOutcome::NotUi;
        }
        if frame.pid != Some(PID_NETROM) {
            return ObserveOutcome::NotNetRom;
        }
        if !is_nodes_destination(&frame.destination.callsign) {
            return ObserveOutcome::NotNodesDestination;
        }

        let originator = frame.source.callsign;
        let Some(broadcast) = NodesBroadcast::try_parse(&frame.info) else {
            return ObserveOutcome::Unparseable;
        };

        let entries = broadcast.entry_count();
        self.table
            .ingest(originator, my_call, port_id, &broadcast, now);
        ObserveOutcome::Ingested { entries }
    }

    /// Ingest a parsed broadcast directly (the wire-level path, for callers that
    /// already have a [`NodesBroadcast`] and the originator/port out of band — e.g.
    /// an AXUDP or KISS-TCP transport). Observation-only.
    pub fn ingest_broadcast(
        &mut self,
        originator: Callsign,
        my_call: Callsign,
        port_id: PortId,
        broadcast: &NodesBroadcast,
        now: u64,
    ) {
        if self.enabled {
            self.table
                .ingest(originator, my_call, port_id, broadcast, now);
        }
    }

    /// Age the routing table by one obsolescence tick (the
    /// `NetRomService.Sweep`/`OnSweep` equivalent — call at the broadcast interval).
    /// Returns the number of routes purged. No-op (returns 0) when disabled.
    pub fn sweep(&mut self) -> usize {
        if self.enabled {
            self.table.sweep()
        } else {
            0
        }
    }

    /// Borrow the underlying routing table (read access for richer queries).
    pub fn table(&self) -> &NodeRoutingTable {
        &self.table
    }

    // ── INetRomRoutingView equivalents (no_std: borrow/visitor, not an alloc snapshot) ──

    /// Number of destinations known (0 when disabled).
    pub fn destination_count(&self) -> usize {
        if self.enabled {
            self.table.destination_count()
        } else {
            0
        }
    }

    /// Number of directly-heard neighbours known (0 when disabled).
    pub fn neighbour_count(&self) -> usize {
        if self.enabled {
            self.table.neighbour_count()
        } else {
            0
        }
    }

    /// Visit each directly-heard neighbour in stable order (callsign ascending).
    /// No-op when disabled.
    pub fn for_each_neighbour(&self, f: impl FnMut(NetRomNeighbour)) {
        if self.enabled {
            self.table.for_each_neighbour(f);
        }
    }

    /// Visit each known destination in stable order (alias-or-callsign ascending).
    /// No-op when disabled.
    pub fn for_each_destination(&self, f: impl FnMut(NetRomDestination)) {
        if self.enabled {
            self.table.for_each_destination(f);
        }
    }

    /// Visit the kept routes of `dest` in best-first order. No-op when disabled or
    /// the destination is unknown.
    pub fn for_each_route(&self, dest: &Callsign, f: impl FnMut(NetRomRoute)) {
        if self.enabled {
            self.table.for_each_route(dest, f);
        }
    }
}

impl Default for NetRomService {
    fn default() -> Self {
        Self::new()
    }
}

/// True if `dest` is the literal NODES destination — base text `"NODES"`, SSID 0.
/// Mirrors C# `NetRomService.IsNodesDestination`.
fn is_nodes_destination(dest: &Callsign) -> bool {
    dest.ssid() == 0 && dest.base() == NodesBroadcast::NODES_DESTINATION.as_bytes()
}

#[cfg(test)]
mod tests {
    //! Service-level tests, mirroring `Packet.Node.Tests.Integration.NetRomAwareIntegrationTests`
    //! — but at the unit-of-logic level (no async / no radio bus): the read-only tap
    //! parsing a real NODES UI frame, the read-only guarantee around a connected
    //! session, and the disabled-service contract.

    use super::*;
    use crate::ax25::frame::{CONTROL_UI, PID_NO_LAYER3};
    use crate::ax25::{Address, Frame};
    use crate::netrom::wire::test_support::build;
    use crate::sdl::{Event, FrameInfo, MockTimerService, SessionManager, State};

    fn call(s: &str) -> Callsign {
        Callsign::parse(s).unwrap()
    }

    fn call_ssid(s: &str, ssid: u8) -> Callsign {
        Callsign::new(s.as_bytes(), ssid).unwrap()
    }

    fn addr(c: Callsign, crh: bool) -> Address {
        Address {
            callsign: c,
            crh,
            extension: false,
        }
    }

    // A genuine NODES broadcast frame: UI, source = broadcaster, dest = literal
    // "NODES", PID 0xCF, info = the built NODES table dump.
    fn nodes_frame(source: Callsign, info: alloc::vec::Vec<u8>) -> Frame {
        Frame {
            destination: addr(call("NODES"), true),
            source: addr(source, false),
            digipeaters: alloc::vec::Vec::new(),
            control: CONTROL_UI,
            pid: Some(PID_NETROM),
            info,
        }
    }

    #[test]
    fn node_hears_a_nodes_broadcast_and_learns_the_routes() {
        let mut svc = NetRomService::new();
        let node = call("M0NODE");
        let neighbour = call("GB7RDG");
        let dest_sot = call("GB7SOT");
        let via_xyz = call_ssid("GB7XYZ", 2);

        let info = build("RDGBPQ", &[(dest_sot, "SOT", via_xyz, 200)]);
        let frame = nodes_frame(neighbour, info);

        let outcome = svc.observe_frame(&frame, node, PortId::from_str_lossy("p1"), 12_000);
        assert_eq!(outcome, ObserveOutcome::Ingested { entries: 1 });

        assert_eq!(svc.neighbour_count(), 1);
        let mut nbrs = alloc::vec::Vec::new();
        svc.for_each_neighbour(|n| nbrs.push(n));
        assert_eq!(nbrs[0].neighbour, neighbour);
        assert_eq!(nbrs[0].alias.as_str(), "RDGBPQ");
        assert_eq!(nbrs[0].port_id.as_str(), "p1");

        // Two destinations: the assumed direct route to GB7RDG, and GB7SOT via it.
        assert!(svc.table().destination(&neighbour).is_some());
        let sot = svc.table().destination(&dest_sot).expect("SOT learned");
        assert_eq!(sot.alias.as_str(), "SOT");
        assert_eq!(sot.best_route.unwrap().neighbour, neighbour); // we forward to the broadcaster
    }

    #[test]
    fn a_non_nodes_frame_is_ignored_by_the_tap() {
        let mut svc = NetRomService::new();
        let node = call("M0NODE");
        let port = PortId::from_str_lossy("p1");

        // A plain UI frame to a normal destination, PID 0xF0 — not NET/ROM.
        let chat = Frame {
            destination: addr(call("CQ"), true),
            source: addr(call("G7XYZ"), false),
            digipeaters: alloc::vec::Vec::new(),
            control: CONTROL_UI,
            pid: Some(PID_NO_LAYER3),
            info: b"hello".to_vec(),
        };
        assert_eq!(
            svc.observe_frame(&chat, node, port, 0),
            ObserveOutcome::NotNetRom
        );

        // A 0xCF UI frame but to a non-NODES destination.
        let mut other = chat.clone();
        other.pid = Some(PID_NETROM);
        assert_eq!(
            svc.observe_frame(&other, node, port, 0),
            ObserveOutcome::NotNodesDestination
        );

        assert_eq!(svc.neighbour_count(), 0);
        assert_eq!(svc.destination_count(), 0);
    }

    #[test]
    fn a_disabled_service_hears_nothing() {
        let mut svc = NetRomService::with_options(false, NetRomRoutingOptions::DEFAULT);
        let info = build("RDGBPQ", &[(call("GB7SOT"), "SOT", call("GB7XYZ"), 200)]);
        let frame = nodes_frame(call("GB7RDG"), info);

        assert_eq!(
            svc.observe_frame(&frame, call("M0NODE"), PortId::from_str_lossy("p1"), 0),
            ObserveOutcome::Disabled
        );
        assert!(!svc.enabled());
        assert_eq!(svc.neighbour_count(), 0);
        assert_eq!(svc.destination_count(), 0);
    }

    #[test]
    fn the_read_only_guarantee_a_nodes_storm_does_not_disturb_a_live_session() {
        // The read-only guarantee, at the logic level: a connected session is held
        // in a SessionManager; a NODES storm is fed only to the NetRomService tap;
        // the session stays Connected and unperturbed throughout. The tap and the
        // session share no state — observe_frame mutates only the routing table.
        let node = call("M0NODE");
        let peer = call("M0RMOT");
        let mut sessions: SessionManager<2> = SessionManager::new(node);
        let mut timers = MockTimerService::new();

        // Bring up a connected session (inbound SABM → UA).
        let sabm = Event::SabmReceived(FrameInfo {
            poll_final: true,
            is_command: true,
            ..Default::default()
        });
        sessions.post(peer, sabm, &mut timers);
        assert_eq!(
            sessions.session_for(&peer).map(|s| s.state),
            Some(State::Connected)
        );

        // Storm the tap with NODES broadcasts while "connected".
        let mut svc = NetRomService::new();
        let info = build(
            "RDGBPQ",
            &[(call("GB7SOT"), "SOT", call_ssid("GB7XYZ", 2), 200)],
        );
        let frame = nodes_frame(call("GB7RDG"), info);
        for i in 0..5 {
            let outcome = svc.observe_frame(&frame, node, PortId::from_str_lossy("p1"), i);
            assert_eq!(outcome, ObserveOutcome::Ingested { entries: 1 });
        }
        // The node heard the NODES while "in a QSO"...
        assert!(svc.neighbour_count() > 0);

        // ...and the session is utterly unperturbed: still Connected, still able to
        // process a fresh event. The read-only tap touched nothing in the session.
        assert_eq!(
            sessions.session_for(&peer).map(|s| s.state),
            Some(State::Connected),
            "a read-only NODES tap must not disturb the live session"
        );
        // A further session event still works (proves the session machinery is intact).
        let rr = Event::RrReceived(FrameInfo {
            poll_final: false,
            is_command: true,
            nr: 0,
            ..Default::default()
        });
        sessions.post(peer, rr, &mut timers);
        assert_eq!(
            sessions.session_for(&peer).map(|s| s.state),
            Some(State::Connected)
        );
    }

    #[test]
    fn observe_frame_is_total_on_garbage_info() {
        // A NODES-shaped frame (UI/0xCF/NODES) whose info is junk must not panic;
        // it returns Unparseable or Ingested-with-0 depending on the bytes, never a
        // crash, and never disturbs anything.
        let mut svc = NetRomService::new();
        let node = call("M0NODE");
        let port = PortId::from_str_lossy("p1");

        // Empty info → too short to be a broadcast → Unparseable.
        let empty = nodes_frame(call("GB7RDG"), alloc::vec::Vec::new());
        assert_eq!(
            svc.observe_frame(&empty, node, port, 0),
            ObserveOutcome::Unparseable
        );

        // Random-ish info, signature present but garbage body.
        let mut info = alloc::vec![NodesBroadcast::SIGNATURE];
        info.extend_from_slice(&[0xAB; 40]);
        let junk = nodes_frame(call("GB7RDG"), info);
        // Must not panic; outcome is a valid variant.
        let _ = svc.observe_frame(&junk, node, port, 0);
    }
}
