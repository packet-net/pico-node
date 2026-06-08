//! NET/ROM L4 **outbound connector** — the integration seam that turns a
//! `connect <alias>` into an end-to-end network-routed circuit, plus the inbound
//! side (a remote opening a circuit to us). Given an **alias** (`SOT`) or
//! **callsign** (`GB7SOT`), it resolves the best route in a [`NetRomRoutingView`],
//! opens an L4 [`NetRomCircuit`] toward the destination node, and bridges the
//! circuit's datagrams onto an AX.25 **interlink** to the route's best neighbour —
//! reaching a node the operator has no direct RF path to, by name.
//!
//! Mirrors the C# `NetRomOutboundConnector`, `NetRomNodeConnection`, and the
//! L4/interlink portion of `NetRomService`, and the TS `NetRomConnector` /
//! `NetRomConnection`.
//!
//! **Sans-io interlink seam.** The TS/C# original owns an `Ax25Listener` (dials
//! CONNECTED-mode sessions, ships `sendData`, taps `onSessionAccepted`). This port
//! owns none of that: the interlink is a **neighbour-keyed byte seam**. Outbound, a
//! circuit's datagram is next-hop-resolved to a neighbour and queued as an
//! [`InterlinkSend`] the node host ships over that neighbour's AX.25 session as a
//! PID-`0xCF` I-frame (drain via [`take_interlink_sends`]). Inbound, the host feeds
//! every PID-`0xCF` DL-DATA indication back in via [`on_interlink_data`]. Circuit
//! lifecycle + reassembled data surface as drainable events
//! ([`take_events`]) / inbound connections ([`take_incoming_connections`]) — the
//! same drain-don't-callback shape as the circuit/manager/originator. The node host
//! wires this seam to the `sdl` AX.25 session manager; here it's driven directly,
//! so the L4 routing logic is unit-testable without a live AX.25 stack.
//!
//! [`take_interlink_sends`]: NetRomConnector::take_interlink_sends
//! [`on_interlink_data`]: NetRomConnector::on_interlink_data
//! [`take_events`]: NetRomConnector::take_events
//! [`take_incoming_connections`]: NetRomConnector::take_incoming_connections

use alloc::string::{String, ToString};
use alloc::vec::Vec;
use core::fmt;

use crate::ax25::Callsign;
use crate::netrom::forwarding::{decide_forward, ForwardMode};
use crate::netrom::routing::NetRomRoutingView;
use crate::netrom::transport::{
    CircuitEvent, CircuitKey, CircuitManager, NetRomCircuitOptions, NetRomCircuitState,
    OutboundPacket,
};
use crate::netrom::wire::{NetRomNetworkHeader, NetRomPacket, MAX_PAYLOAD, PACKET_HEADER_LEN};

/// Returned by [`NetRomConnector::connect`] when connect-routing is enabled but the
/// routing table has no route to the target — the signal for the host to fall back
/// to a direct same-port AX.25 dial. Mirrors the C# `NetRomOutboundConnector`
/// deferring to its `fallback` connector / the TS `NetRomNoRouteError`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NetRomNoRoute {
    target: String,
}

impl NetRomNoRoute {
    fn new(target: &str) -> Self {
        Self {
            target: target.to_string(),
        }
    }

    /// The alias/callsign the operator typed, that no route resolved.
    pub fn target(&self) -> &str {
        &self.target
    }
}

impl fmt::Display for NetRomNoRoute {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "no NET/ROM route to {}.", self.target)
    }
}

/// One datagram to ship over the interlink to `neighbour`, as a PID-`0xCF` I-frame.
/// The host maps `neighbour` to that neighbour's AX.25 session.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct InterlinkSend {
    /// The next-hop neighbour to ship over.
    pub neighbour: Callsign,
    /// The encoded NET/ROM datagram (the I-frame info field).
    pub datagram: Vec<u8>,
}

/// A duplex handle to a NET/ROM L4 circuit — the network-routed analogue of an AX.25
/// session, returned by [`connect`](NetRomConnector::connect) and raised for inbound
/// circuits. Sans-io: a passive `(key, peer)` reference; drive it via the connector
/// ([`write`](NetRomConnector::write) / [`disconnect`](NetRomConnector::disconnect),
/// data + close via [`take_events`](NetRomConnector::take_events)). Mirrors the C#
/// `NetRomNodeConnection` / the TS `NetRomConnection`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NetRomConnection {
    /// The circuit's local key, to drive it through the connector.
    pub key: CircuitKey,
    /// The far node's callsign (the destination we dialled, or the remote that
    /// dialled us) — the C# `PeerId`.
    pub peer: Callsign,
}

/// Construction options for [`NetRomConnector`].
#[derive(Debug, Clone, Copy)]
pub struct NetRomConnectorOptions {
    /// Whether NET/ROM L4 connect-routing is enabled (the C# `netRom.connect`
    /// opt-in). Default `false` — [`connect`](NetRomConnector::connect) then resolves
    /// no route, so the host falls straight to a direct AX.25 dial.
    pub enabled: bool,
    /// Whether this node **forwards transit datagrams** — relays a datagram whose
    /// destination node is not us onward toward its destination (the network-layer
    /// routing role; mirrors the C# `netRom.forward`). Default `true`, but effective
    /// only when [`enabled`](Self::enabled) is on (forwarding needs the connect-routing
    /// interlink machinery). So an endpoint node never relays; a connect-enabled node
    /// forwards by default; set `false` for an originate-only node.
    pub forward: bool,
    /// How a forwarding node picks among multiple kept routes to a destination
    /// ([`ForwardMode`]). Default [`ForwardMode::PerFlow`] — spread distinct L4
    /// circuits across the kept routes, quality-weighted, each circuit pinned to one
    /// path. Mirrors the C# `netRom.ForwardMode`.
    pub forward_mode: ForwardMode,
    /// The L4 circuit tunables handed to the owned circuit manager.
    pub circuit: NetRomCircuitOptions,
}

impl Default for NetRomConnectorOptions {
    fn default() -> Self {
        Self {
            enabled: false,
            forward: true,
            forward_mode: ForwardMode::PerFlow,
            circuit: NetRomCircuitOptions::default(),
        }
    }
}

/// The NET/ROM L4 connector over a routing view.
pub struct NetRomConnector {
    enabled: bool,
    forward_enabled: bool,
    forward_mode: ForwardMode,
    prefer_inp3_routes: bool,
    max_ttl: u8,
    node_call: Callsign,
    manager: CircuitManager,
    interlinks: Vec<Callsign>,
    outbound: Vec<InterlinkSend>,
    events: Vec<(CircuitKey, CircuitEvent)>,
    incoming: Vec<NetRomConnection>,
}

impl NetRomConnector {
    /// Construct the connector for `node_call` (the local node stamped into the L3
    /// origin of circuits we open). Off by default — set
    /// [`NetRomConnectorOptions::enabled`].
    pub fn new(node_call: Callsign, options: NetRomConnectorOptions) -> Self {
        Self {
            enabled: options.enabled,
            forward_enabled: options.enabled && options.forward,
            forward_mode: options.forward_mode,
            // Quality forwarding by default; the firmware host flips this via
            // set_prefer_inp3_routes once the INP3 overlay is feeding the table.
            prefer_inp3_routes: false,
            max_ttl: options.circuit.time_to_live,
            node_call,
            manager: CircuitManager::new(node_call, options.circuit),
            interlinks: Vec::new(),
            outbound: Vec::new(),
            events: Vec::new(),
            incoming: Vec::new(),
        }
    }

    /// True if connect-routing is enabled (the C# `ConnectEnabled`).
    pub fn enabled(&self) -> bool {
        self.enabled
    }

    /// Set the INP3 forwarding preference at runtime (BPQ's `PREFERINP3ROUTES`). When
    /// `true`, [`forward_datagram`](Self::on_interlink_data) and the connect/reply
    /// next-hop forward over a destination's lowest-target-time INP3 route (quality
    /// fallback when none is usable); when `false`, quality decides exactly as today.
    /// The firmware host flips this when the INP3 overlay is feeding the table
    /// time-routes — the live engine/scheduler/dispatch wiring is the firmware's job.
    pub fn set_prefer_inp3_routes(&mut self, prefer: bool) {
        self.prefer_inp3_routes = prefer;
    }

    /// True if this node forwards transit datagrams (the network-layer routing role).
    /// Rides on [`enabled`](Self::enabled) + the `forward` option. The C#
    /// `ForwardEnabled`.
    pub fn forward_enabled(&self) -> bool {
        self.forward_enabled
    }

    /// The local node callsign.
    pub fn node_call(&self) -> Callsign {
        self.node_call
    }

    /// The number of live circuits (the C# `NetRomService.Circuits.Count`).
    pub fn circuit_count(&self) -> usize {
        self.manager.circuit_count()
    }

    /// The neighbours we currently hold an interlink to.
    pub fn interlink_neighbours(&self) -> &[Callsign] {
        &self.interlinks
    }

    /// The lifecycle state of a circuit by key.
    pub fn circuit_state(&self, key: CircuitKey) -> Option<NetRomCircuitState> {
        self.manager.circuit_state(key)
    }

    /// True once a connection's circuit has closed (or was never live) — the C#
    /// `NetRomNodeConnection.Closed`.
    pub fn connection_closed(&self, connection: &NetRomConnection) -> bool {
        !matches!(
            self.manager.circuit_state(connection.key),
            Some(NetRomCircuitState::Connecting)
                | Some(NetRomCircuitState::Connected)
                | Some(NetRomCircuitState::Disconnecting)
        )
    }

    /// Resolve a `connect <target>` (alias or callsign) and open an L4 circuit to it
    /// across the network. On a route hit, opens the interlink + circuit, drives the
    /// connect, and returns the [`NetRomConnection`] (the circuit is `Connecting`
    /// until the host pumps the interlink and the Connect Acknowledge returns —
    /// observe via [`take_events`](Self::take_events)). On a miss (or when disabled),
    /// returns [`NetRomNoRoute`] so the host can fall back to a direct AX.25 dial.
    ///
    /// Mirrors the C# `NetRomOutboundConnector.ConnectAsync`.
    pub fn connect(
        &mut self,
        routing: &dyn NetRomRoutingView,
        target: &str,
        originating_user: Callsign,
        now_ms: u64,
    ) -> Result<NetRomConnection, NetRomNoRoute> {
        if !self.enabled {
            return Err(NetRomNoRoute::new(target));
        }
        let destination = match routing.resolve_destination(target) {
            Some(d) if d.best_route.is_some() => d,
            _ => return Err(NetRomNoRoute::new(target)),
        };
        let best = destination.best_route.expect("checked Some above");

        // Ensure the interlink to the best neighbour before originating.
        self.ensure_interlink(best.neighbour);

        let key = self.manager.open_circuit(destination.destination);
        self.manager
            .circuit_mut(key)
            .expect("just opened")
            .connect(originating_user, now_ms);
        self.after_manager(routing);

        Ok(NetRomConnection {
            key,
            peer: destination.destination,
        })
    }

    /// Send `data` to the peer over a connection's circuit (fragmented + windowed by
    /// the circuit). No-op if the circuit is gone. Mirrors `NetRomConnection.write`.
    pub fn write(
        &mut self,
        routing: &dyn NetRomRoutingView,
        connection: &NetRomConnection,
        data: &[u8],
        now_ms: u64,
    ) {
        if let Some(c) = self.manager.circuit_mut(connection.key) {
            c.send(data, now_ms);
        }
        self.after_manager(routing);
    }

    /// Tear a connection down (disconnect its circuit). Mirrors
    /// `NetRomConnection.dispose`.
    pub fn disconnect(
        &mut self,
        routing: &dyn NetRomRoutingView,
        connection: &NetRomConnection,
        now_ms: u64,
    ) {
        if let Some(c) = self.manager.circuit_mut(connection.key) {
            c.disconnect(now_ms);
        }
        self.after_manager(routing);
    }

    /// Feed an inbound interlink I-frame (PID-`0xCF` DL-DATA) heard from `from`. The
    /// datagram is parsed and demuxed to its circuit (or mints an inbound circuit,
    /// auto-accepted + raised via [`take_incoming_connections`](Self::take_incoming_connections));
    /// a malformed datagram is dropped. `from` is remembered as an interlink so our
    /// replies reuse it. Mirrors the C# `OnInterlinkData` + `OnSessionAccepted`.
    pub fn on_interlink_data(
        &mut self,
        routing: &dyn NetRomRoutingView,
        from: Callsign,
        info: &[u8],
        now_ms: u64,
    ) {
        self.ensure_interlink(from);
        if let Some(packet) = NetRomPacket::decode(info) {
            // L3 dispatch (mirrors the C# `NetRomService.OnInterlinkData`): a datagram
            // addressed to this node terminates here (up to the L4 circuit layer); one
            // addressed elsewhere is forwarded toward its destination (the
            // network-layer routing role); an endpoint-only node drops it.
            if packet.network.destination == self.node_call {
                self.manager.on_packet(&packet, now_ms);
            } else if self.forward_enabled {
                self.forward_datagram(routing, &packet, from);
            }
        }
        self.after_manager(routing);
    }

    /// Forward a transit datagram (one whose destination node is not us) one hop
    /// toward its destination. The decision (TTL decrement/cap, loop guard,
    /// no-bounce-back next-hop) is the pure [`decide_forward`]; this emits the
    /// rewritten datagram as an [`InterlinkSend`] to the next hop (the host ships it
    /// over that neighbour's AX.25 session). Mirrors the C#
    /// `NetRomService.ForwardDatagram`.
    fn forward_datagram(
        &mut self,
        routing: &dyn NetRomRoutingView,
        packet: &NetRomPacket,
        received_from: Callsign,
    ) {
        let decision = decide_forward(
            packet,
            &received_from,
            &self.node_call,
            routing,
            self.max_ttl,
            self.forward_mode,
            self.prefer_inp3_routes,
        );
        let Some(neighbour) = decision.next_hop else {
            return; // dropped — TTL expired, looped back, or no onward route
        };

        self.ensure_interlink(neighbour);
        let forwarded = OutboundPacket {
            network: NetRomNetworkHeader {
                origin: packet.network.origin,
                destination: packet.network.destination,
                time_to_live: decision.time_to_live,
            },
            transport: packet.transport,
            payload: packet.payload.to_vec(),
        };
        let mut buf = [0u8; PACKET_HEADER_LEN + MAX_PAYLOAD];
        if let Some(n) = forwarded.encode(&mut buf) {
            self.outbound.push(InterlinkSend {
                neighbour,
                datagram: buf[..n].to_vec(),
            });
        }
    }

    /// Advance every circuit's retransmit timers by one tick (the host drives this
    /// from its timer). Mirrors the C# manager's `TimeProvider` tick.
    pub fn tick(&mut self, routing: &dyn NetRomRoutingView, now_ms: u64) {
        self.manager.tick(now_ms);
        self.after_manager(routing);
    }

    /// Drain the interlink datagrams queued since the last call (ship each over its
    /// `neighbour`'s AX.25 session as a PID-`0xCF` I-frame).
    pub fn take_interlink_sends(&mut self) -> Vec<InterlinkSend> {
        core::mem::take(&mut self.outbound)
    }

    /// Drain the per-circuit lifecycle events since the last call (Connected /
    /// DataReceived / Closed, tagged with the circuit key).
    pub fn take_events(&mut self) -> Vec<(CircuitKey, CircuitEvent)> {
        core::mem::take(&mut self.events)
    }

    /// Drain the inbound connections raised (auto-accepted) since the last call.
    pub fn take_incoming_connections(&mut self) -> Vec<NetRomConnection> {
        core::mem::take(&mut self.incoming)
    }

    // ─── Internals ────────────────────────────────────────────────────

    fn ensure_interlink(&mut self, neighbour: Callsign) {
        if !self.interlinks.contains(&neighbour) {
            self.interlinks.push(neighbour);
        }
    }

    /// After driving the manager: auto-accept inbound circuits, surface events
    /// (delivery-draining each received frame to release choke, the synchronous
    /// bridge the C#/TS `NetRomConnection` does), and ship the resulting outbound
    /// datagrams over their interlinks. Pure draining — the time-advancing circuit
    /// ops (connect/send/disconnect/on_packet/tick) ran with `now_ms` before this.
    fn after_manager(&mut self, routing: &dyn NetRomRoutingView) {
        let incoming = self.manager.take_incoming();
        for inc in &incoming {
            self.manager.accept(inc);
            self.incoming.push(NetRomConnection {
                key: inc.key,
                peer: inc.remote_node,
            });
        }

        for (key, ev) in self.manager.take_events() {
            if matches!(ev, CircuitEvent::DataReceived(_)) {
                if let Some(c) = self.manager.circuit_mut(key) {
                    c.on_delivery_drained();
                }
            }
            self.events.push((key, ev));
        }

        self.pump_outbound(routing);
    }

    fn pump_outbound(&mut self, routing: &dyn NetRomRoutingView) {
        let mut buf = [0u8; PACKET_HEADER_LEN + MAX_PAYLOAD];
        for packet in self.manager.take_outbox() {
            let Some(neighbour) = self.next_hop(routing, &packet.network.destination) else {
                continue; // no resolvable next hop — drop (a transit edge case)
            };
            if let Some(n) = packet.encode(&mut buf) {
                self.outbound.push(InterlinkSend {
                    neighbour,
                    datagram: buf[..n].to_vec(),
                });
            }
        }
    }

    /// Next-hop for a datagram's L3 destination (the C# `SendNetRomPacket` order):
    /// (1) a direct interlink to the destination node itself (reply over the very
    /// session a peer reached us on); (2) the best route in the table; (3) the
    /// destination as a directly-heard neighbour.
    fn next_hop(&self, routing: &dyn NetRomRoutingView, dest: &Callsign) -> Option<Callsign> {
        if self.interlinks.contains(dest) {
            return Some(*dest);
        }
        if let Some(d) = routing.destination_for(dest) {
            if let Some(best) = d.best_route {
                return Some(best.neighbour);
            }
        }
        if routing.neighbour_for(dest).is_some() {
            return Some(*dest);
        }
        None
    }
}

#[cfg(test)]
mod tests {
    //! Ported from the TS `tests/netrom/connector.test.ts` + `connect-harness.ts`.
    //! The TS harness wires two real `Ax25Listener`s over a mock transport; the Rust
    //! sans-io equivalent wires two connectors over an in-process neighbour-keyed
    //! interlink (the AX.25 connected-mode reliability lives below this seam — the
    //! L4 routing/circuit logic is what's under test). Node B auto-accepts inbound
    //! circuits and bridges each to an echo console, so `connect <alias>` from A
    //! reaches a far prompt that talks back — the end-to-end L4 round-trip.

    use super::*;
    use crate::netrom::routing::{NetRomRoutingOptions, NetRomRoutingTable};
    use crate::netrom::wire::{
        write_nodes_frame, Alias, NodesAdvertisementEntry, NodesBroadcast, MAX_NODES_FRAME_LEN,
    };
    use crate::netrom::PortId;
    use alloc::collections::BTreeMap;

    type Table = NetRomRoutingTable<64, 3, 32>;

    const NOW: u64 = 1_000_000;

    fn call(text: &str) -> Callsign {
        Callsign::new(text.as_bytes(), 0).unwrap()
    }

    fn a_node() -> Callsign {
        call("GB7AAA")
    }
    fn b_node() -> Callsign {
        call("GB7BBB")
    }
    // The node `connect <alias>` reaches over the interlink IS node B (the real
    // interlink peer). Before L3 forwarding the harness used a fictional distinct END
    // node that B terminated on behalf of — which only worked because a node
    // terminated EVERY inbound circuit regardless of L3 destination. With forwarding a
    // node terminates only circuits addressed to itself, so the endpoint here is B;
    // genuine multi-hop transit is covered by the `decide_forward` tests + the C#
    // 3-node transit integration test.
    fn end_node() -> Callsign {
        b_node()
    }

    /// Seed a table the way real ingest would: `originator` advertised `entry`.
    fn seed(
        table: &mut Table,
        originator: Callsign,
        my_call: Callsign,
        entry: NodesAdvertisementEntry,
    ) {
        let mut buf = [0u8; MAX_NODES_FRAME_LEN];
        let n = write_nodes_frame(&Alias::from_str_lossy("BNODE"), &[entry], &mut buf).unwrap();
        let broadcast = NodesBroadcast::try_parse(&buf[..n]).unwrap();
        table.ingest(
            originator,
            my_call,
            PortId::from_str_lossy("p1"),
            &broadcast,
            NOW,
        );
    }

    /// Node A's table: a route to END via neighbour B (so `connect ENDND` resolves to
    /// a best-neighbour interlink to B).
    fn seeded_table_a() -> Table {
        let mut table: Table = NetRomRoutingTable::new(NetRomRoutingOptions::default());
        seed(
            &mut table,
            b_node(),
            a_node(),
            NodesAdvertisementEntry {
                destination: end_node(),
                destination_alias: Alias::from_str_lossy("ENDND"),
                best_neighbour: end_node(),
                quality: 200,
            },
        );
        table
    }

    #[derive(Default, Clone)]
    struct Captured {
        connected: bool,
        received: Vec<Vec<u8>>,
        closed: bool,
    }

    impl Captured {
        fn received_text(&self) -> String {
            let mut all = Vec::new();
            for r in &self.received {
                all.extend_from_slice(r);
            }
            String::from_utf8_lossy(&all).into_owned()
        }
    }

    /// Two connectors (A `GB7AAA`, B `GB7BBB`) wired over an in-process interlink, B
    /// running an echo console on each accepted inbound circuit.
    struct ConnectHarness {
        a: NetRomConnector,
        b: NetRomConnector,
        table_a: Table,
        table_b: Table,
        now: u64,
        echo_on_b: bool,
        banner: Vec<u8>,
        b_connections: Vec<NetRomConnection>,
        a_cap: BTreeMap<CircuitKey, Captured>,
        b_cap: BTreeMap<CircuitKey, Captured>,
    }

    impl ConnectHarness {
        fn create(options: NetRomConnectorOptions) -> Self {
            Self {
                a: NetRomConnector::new(a_node(), options),
                b: NetRomConnector::new(b_node(), options),
                table_a: seeded_table_a(),
                table_b: NetRomRoutingTable::new(NetRomRoutingOptions::default()),
                now: NOW,
                echo_on_b: false,
                banner: b"bnode-prompt\r".to_vec(),
                b_connections: Vec::new(),
                a_cap: BTreeMap::new(),
                b_cap: BTreeMap::new(),
            }
        }

        fn echo_console_on_b(&mut self) {
            self.echo_on_b = true;
        }

        /// Dial `connect <target>` from A and pump to quiescence.
        fn connect_a(
            &mut self,
            target: &str,
            user: Callsign,
        ) -> Result<NetRomConnection, NetRomNoRoute> {
            let now = self.now;
            let result = self.a.connect(&self.table_a, target, user, now);
            self.pump();
            result
        }

        fn write_a(&mut self, connection: &NetRomConnection, data: &[u8]) {
            let now = self.now;
            self.a.write(&self.table_a, connection, data, now);
            self.pump();
        }

        fn disconnect_a(&mut self, connection: &NetRomConnection) {
            let now = self.now;
            self.a.disconnect(&self.table_a, connection, now);
            self.pump();
        }

        fn disconnect_b(&mut self, connection: &NetRomConnection) {
            let now = self.now;
            self.b.disconnect(&self.table_b, connection, now);
            self.pump();
        }

        /// Deliver interlink traffic + drive B's echo console until both sides settle.
        fn pump(&mut self) {
            let mut guard = 0u32;
            loop {
                guard += 1;
                assert!(guard < 100_000, "connect harness livelock");
                if !self.step() {
                    break;
                }
            }
        }

        fn step(&mut self) -> bool {
            let now = self.now;
            let mut progressed = false;

            // B's newly-accepted inbound circuits → echo console banner.
            for conn in self.b.take_incoming_connections() {
                self.b_connections.push(conn);
                if self.echo_on_b {
                    let banner = self.banner.clone();
                    self.b.write(&self.table_b, &conn, &banner, now);
                }
                progressed = true;
            }
            // A is the dialler; it shouldn't receive inbound circuits, but drain to be safe.
            for _ in self.a.take_incoming_connections() {
                progressed = true;
            }

            // Events: record both sides; B's received lines → echo `ack:<line>`.
            for (key, ev) in self.a.take_events() {
                record(&mut self.a_cap, key, ev);
                progressed = true;
            }
            for (key, ev) in self.b.take_events() {
                if let CircuitEvent::DataReceived(data) = &ev {
                    if self.echo_on_b {
                        let mut ack = b"ack:".to_vec();
                        ack.extend_from_slice(data);
                        if let Some(conn) =
                            self.b_connections.iter().find(|c| c.key == key).copied()
                        {
                            self.b.write(&self.table_b, &conn, &ack, now);
                        }
                    }
                }
                record(&mut self.b_cap, key, ev);
                progressed = true;
            }

            // Deliver the interlink datagrams each way (from = the sender's node call).
            for send in self.a.take_interlink_sends() {
                self.b
                    .on_interlink_data(&self.table_b, a_node(), &send.datagram, now);
                progressed = true;
            }
            for send in self.b.take_interlink_sends() {
                self.a
                    .on_interlink_data(&self.table_a, b_node(), &send.datagram, now);
                progressed = true;
            }

            progressed
        }

        fn a_cap(&self, key: CircuitKey) -> Captured {
            self.a_cap.get(&key).cloned().unwrap_or_default()
        }
    }

    fn record(cap: &mut BTreeMap<CircuitKey, Captured>, key: CircuitKey, ev: CircuitEvent) {
        let c = cap.entry(key).or_default();
        match ev {
            CircuitEvent::Connected => c.connected = true,
            CircuitEvent::DataReceived(d) => c.received.push(d),
            CircuitEvent::Closed(_) => c.closed = true,
        }
    }

    // ─── The ported connector tests ─────────────────────────────────────

    #[test]
    fn routes_connect_alias_across_a_circuit_round_trips_data_and_tears_down() {
        let mut h = ConnectHarness::create(NetRomConnectorOptions {
            enabled: true,
            ..Default::default()
        });
        h.echo_console_on_b();

        let conn = h
            .connect_a("ENDND", Callsign::parse("M0LTE-7").unwrap())
            .expect("route to END");

        // 1. The interlink to B is up, the L4 circuit Connected, peer is END.
        assert!(h.a.interlink_neighbours().contains(&b_node()));
        assert_eq!(
            h.a.circuit_state(conn.key),
            Some(NetRomCircuitState::Connected)
        );
        assert_eq!(conn.peer, end_node());
        assert_eq!(h.a.circuit_count(), 1);

        // 2. B holds the accepted, Connected circuit.
        assert_eq!(h.b_connections.len(), 1);
        let b_conn = h.b_connections[0];
        assert_eq!(
            h.b.circuit_state(b_conn.key),
            Some(NetRomCircuitState::Connected)
        );
        assert_eq!(b_conn.peer, a_node(), "B's circuit remote is the dialler A");

        // 3. B's banner reached A.
        assert!(h.a_cap(conn.key).received_text().contains("bnode-prompt"));

        // 4. A line A sends reaches B's console and the ack relays back.
        h.write_a(&conn, b"hello-over-circuit\r");
        assert!(h
            .a_cap(conn.key)
            .received_text()
            .contains("ack:hello-over-circuit"));

        // 5. Disconnect tears down the circuit on both ends.
        h.disconnect_a(&conn);
        assert!(h.a.connection_closed(&conn));
        assert_eq!(h.a.circuit_count(), 0);
        assert_eq!(
            h.b.circuit_count(),
            0,
            "the manager deregisters a closed circuit"
        );
        assert_eq!(h.a.circuit_state(conn.key), None);
    }

    #[test]
    fn opens_the_interlink_once_and_reuses_it_for_a_second_circuit() {
        let mut h = ConnectHarness::create(NetRomConnectorOptions {
            enabled: true,
            ..Default::default()
        });
        h.echo_console_on_b();

        let c1 = h.connect_a("ENDND", a_node()).unwrap();
        let neighbours_after_first: Vec<Callsign> = h.a.interlink_neighbours().to_vec();
        let c2 = h.connect_a("ENDND", a_node()).unwrap();

        assert_eq!(
            h.a.circuit_state(c1.key),
            Some(NetRomCircuitState::Connected)
        );
        assert_eq!(
            h.a.circuit_state(c2.key),
            Some(NetRomCircuitState::Connected)
        );
        assert_eq!(neighbours_after_first, alloc::vec![b_node()]);
        assert_eq!(
            h.a.interlink_neighbours(),
            &[b_node()],
            "second connect reused the interlink"
        );
        assert_eq!(h.a.circuit_count(), 2);
        assert_ne!(c1.key, c2.key);
    }

    #[test]
    fn resolves_connect_by_the_destination_callsign_as_well_as_its_alias() {
        let mut h = ConnectHarness::create(NetRomConnectorOptions {
            enabled: true,
            ..Default::default()
        });
        h.echo_console_on_b();

        let conn = h
            .connect_a("GB7BBB", a_node())
            .expect("route to the endpoint by callsign");
        assert_eq!(
            h.a.circuit_state(conn.key),
            Some(NetRomCircuitState::Connected)
        );
        assert_eq!(conn.peer, end_node());
    }

    #[test]
    fn surfaces_a_no_route_connect_cleanly_so_the_host_can_fall_back() {
        let mut h = ConnectHarness::create(NetRomConnectorOptions {
            enabled: true,
            ..Default::default()
        });

        let result = h.connect_a("NOWHER", a_node());
        assert!(matches!(result, Err(ref e) if e.target() == "NOWHER"));
        assert_eq!(h.a.interlink_neighbours().len(), 0, "no interlink opened");
        assert_eq!(h.a.circuit_count(), 0, "no circuit minted");
    }

    #[test]
    fn treats_connect_as_a_no_route_miss_when_connect_routing_is_disabled() {
        let table = seeded_table_a();
        let mut connector = NetRomConnector::new(
            a_node(),
            NetRomConnectorOptions {
                enabled: false,
                ..Default::default()
            },
        );
        assert!(!connector.enabled());
        let result = connector.connect(&table, "ENDND", a_node(), NOW);
        assert!(matches!(result, Err(ref e) if e.target() == "ENDND"));
        assert_eq!(connector.interlink_neighbours().len(), 0);
    }

    #[test]
    fn the_no_route_error_names_the_target() {
        let err = NetRomNoRoute::new("GB7XYZ");
        assert_eq!(err.target(), "GB7XYZ");
        assert!(alloc::format!("{err}").contains("GB7XYZ"));
    }

    #[test]
    fn the_connection_wraps_the_circuit_as_a_duplex_stream() {
        let mut h = ConnectHarness::create(NetRomConnectorOptions {
            enabled: true,
            ..Default::default()
        });
        h.echo_console_on_b();

        let conn = h.connect_a("ENDND", a_node()).unwrap();
        assert!(!h.a.connection_closed(&conn));
        assert_eq!(conn.peer, end_node());

        h.disconnect_a(&conn);
        assert!(h.a.connection_closed(&conn));
        assert!(h.a_cap(conn.key).closed, "the close event settled");
    }

    // ─── connection.test.ts behaviours the thin handle still must honour ────

    #[test]
    fn a_peer_initiated_disconnect_closes_the_connection() {
        let mut h = ConnectHarness::create(NetRomConnectorOptions {
            enabled: true,
            ..Default::default()
        });
        h.echo_console_on_b();
        let conn = h.connect_a("ENDND", a_node()).unwrap();
        let b_conn = h.b_connections[0];

        // The far end (B) drops the circuit — A's connection settles closed.
        h.disconnect_b(&b_conn);
        assert!(h.a.connection_closed(&conn));
        assert!(h.a_cap(conn.key).closed, "A saw the peer-initiated close");
        assert_eq!(h.a.circuit_count(), 0);
    }

    #[test]
    fn write_after_dispose_is_a_no_op() {
        let mut h = ConnectHarness::create(NetRomConnectorOptions {
            enabled: true,
            ..Default::default()
        });
        h.echo_console_on_b();
        let conn = h.connect_a("ENDND", a_node()).unwrap();

        h.disconnect_a(&conn);
        assert_eq!(h.a.circuit_count(), 0);
        // Writing to the disposed connection neither panics nor revives the circuit.
        h.write_a(&conn, b"after dispose");
        assert_eq!(h.a.circuit_count(), 0);
    }

    // ─── 3-node transit: integration-level forwarding (mirrors the C# test) ──

    fn c_node() -> Callsign {
        call("GB7CCC")
    }

    /// Seed `table` so it learns `originator` as a directly-heard neighbour (and a
    /// destination via the assumed direct route) — a header-only NODES broadcast.
    fn seed_neighbour(table: &mut Table, originator: Callsign, my_call: Callsign) {
        let mut buf = [0u8; MAX_NODES_FRAME_LEN];
        let n = write_nodes_frame(&Alias::from_str_lossy("N"), &[], &mut buf).unwrap();
        let broadcast = NodesBroadcast::try_parse(&buf[..n]).unwrap();
        table.ingest(
            originator,
            my_call,
            PortId::from_str_lossy("p1"),
            &broadcast,
            NOW,
        );
    }

    /// Drive the in-process interlinks between three connectors (A, B, C) until
    /// quiescent, routing each [`InterlinkSend`] by its next-hop neighbour. C is the
    /// endpoint (echo console: banner on accept, `ack:<line>` per datagram); B is a
    /// pure transit node; A records what it receives.
    #[allow(clippy::too_many_arguments)]
    fn pump3(
        a: &mut NetRomConnector,
        table_a: &Table,
        b: &mut NetRomConnector,
        table_b: &Table,
        c: &mut NetRomConnector,
        table_c: &Table,
        c_conns: &mut Vec<NetRomConnection>,
        a_received: &mut Vec<Vec<u8>>,
        banner: &[u8],
    ) {
        let mut guard = 0u32;
        loop {
            guard += 1;
            assert!(guard < 100_000, "transit livelock");

            // Collect every pending interlink datagram: (sender, next-hop, bytes).
            let mut queue: Vec<(Callsign, Callsign, Vec<u8>)> = Vec::new();
            for s in a.take_interlink_sends() {
                queue.push((a_node(), s.neighbour, s.datagram));
            }
            for s in b.take_interlink_sends() {
                queue.push((b_node(), s.neighbour, s.datagram));
            }
            for s in c.take_interlink_sends() {
                queue.push((c_node(), s.neighbour, s.datagram));
            }

            let mut progressed = !queue.is_empty();

            // C echoes (the endpoint console).
            for conn in c.take_incoming_connections() {
                c_conns.push(conn);
                c.write(table_c, &conn, banner, NOW);
                progressed = true;
            }
            for (key, ev) in c.take_events() {
                if let CircuitEvent::DataReceived(data) = &ev {
                    if let Some(conn) = c_conns.iter().find(|x| x.key == key).copied() {
                        let mut ack = b"ack:".to_vec();
                        ack.extend_from_slice(data);
                        c.write(table_c, &conn, &ack, NOW);
                    }
                }
                progressed = true;
            }
            // A records its inbound data; B is pure transit (drain, no console).
            for (_key, ev) in a.take_events() {
                if let CircuitEvent::DataReceived(data) = ev {
                    a_received.push(data);
                }
                progressed = true;
            }
            for _ in b.take_incoming_connections() {
                progressed = true;
            }
            if !b.take_events().is_empty() {
                progressed = true;
            }

            if queue.is_empty() && !progressed {
                break;
            }

            for (from, to, datagram) in queue {
                if to == a_node() {
                    a.on_interlink_data(table_a, from, &datagram, NOW);
                } else if to == b_node() {
                    b.on_interlink_data(table_b, from, &datagram, NOW);
                } else if to == c_node() {
                    c.on_interlink_data(table_c, from, &datagram, NOW);
                }
            }
        }
    }

    #[test]
    fn forwards_an_l4_circuit_through_a_transit_node_without_terminating_it() {
        // A reaches C only via B; C reaches A only via B; B knows both directly. A
        // dials C by alias — the circuit's datagrams must transit B both ways.
        let opts = NetRomConnectorOptions {
            enabled: true,
            ..Default::default()
        };
        let mut a = NetRomConnector::new(a_node(), opts);
        let mut b = NetRomConnector::new(b_node(), opts);
        let mut c = NetRomConnector::new(c_node(), opts);

        let mut table_a: Table = NetRomRoutingTable::new(NetRomRoutingOptions::default());
        seed(
            &mut table_a,
            b_node(),
            a_node(),
            NodesAdvertisementEntry {
                destination: c_node(),
                destination_alias: Alias::from_str_lossy("CCC"),
                best_neighbour: c_node(),
                quality: 200,
            },
        );
        let mut table_c: Table = NetRomRoutingTable::new(NetRomRoutingOptions::default());
        seed(
            &mut table_c,
            b_node(),
            c_node(),
            NodesAdvertisementEntry {
                destination: a_node(),
                destination_alias: Alias::from_str_lossy("AAA"),
                best_neighbour: a_node(),
                quality: 200,
            },
        );
        let mut table_b: Table = NetRomRoutingTable::new(NetRomRoutingOptions::default());
        seed_neighbour(&mut table_b, a_node(), b_node());
        seed_neighbour(&mut table_b, c_node(), b_node());

        let conn = a
            .connect(&table_a, "CCC", a_node(), NOW)
            .expect("A routes to C via B");

        let mut c_conns: Vec<NetRomConnection> = Vec::new();
        let mut a_received: Vec<Vec<u8>> = Vec::new();
        let banner = b"c-prompt";
        pump3(
            &mut a,
            &table_a,
            &mut b,
            &table_b,
            &mut c,
            &table_c,
            &mut c_conns,
            &mut a_received,
            banner,
        );

        // The circuit established end-to-end, transiting B.
        assert_eq!(
            a.circuit_state(conn.key),
            Some(NetRomCircuitState::Connected)
        );
        assert_eq!(conn.peer, c_node());
        assert_eq!(c.circuit_count(), 1, "C terminated the circuit");
        assert_eq!(c_conns.len(), 1);
        assert_eq!(
            c_conns[0].peer,
            a_node(),
            "C's inbound circuit is from the dialler A"
        );
        assert_eq!(
            b.circuit_count(),
            0,
            "B forwarded the circuit's datagrams between its neighbours — it never terminated one"
        );

        // C's banner reached A through B (the C→B→A data-forwarding path).
        let a_text: Vec<u8> = a_received.iter().flatten().copied().collect();
        assert!(
            a_text.windows(banner.len()).any(|w| w == banner),
            "C's banner reached A via the transit node B"
        );

        // A→C→A round-trip: A sends a line, C echoes `ack:`, it returns through B.
        a.write(&table_a, &conn, b"hi-transit", NOW);
        pump3(
            &mut a,
            &table_a,
            &mut b,
            &table_b,
            &mut c,
            &table_c,
            &mut c_conns,
            &mut a_received,
            banner,
        );
        let a_text: Vec<u8> = a_received.iter().flatten().copied().collect();
        let want = b"ack:hi-transit";
        assert!(
            a_text.windows(want.len()).any(|w| w == want),
            "the line A sent transited to C and the ack relayed back through B"
        );
        assert_eq!(
            b.circuit_count(),
            0,
            "B is still pure transit after the data exchange"
        );
    }
}
