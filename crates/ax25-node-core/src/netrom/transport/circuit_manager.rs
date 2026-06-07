//! Owns this node's NET/ROM L4 circuit table: mints local circuits (allocating the
//! index/id pair), demultiplexes inbound datagrams to the right
//! [`NetRomCircuit`], raises inbound connect requests for the host to accept or
//! refuse, and drives every circuit's retransmit timer off one [`tick`]. It speaks
//! only [`NetRomPacket`] in and out — no AX.25 / host dependency.
//!
//! **Sans-io.** As with the circuit, this owns the circuits and aggregates their
//! outbound datagrams + lifecycle events. After driving the manager (`on_packet` /
//! `tick` / `accept` / `refuse` / a circuit `connect`/`send`/`disconnect` via
//! [`circuit_mut`]), the owner drains [`take_outbox`] (datagrams to ship, by
//! `network.destination`), [`take_events`] (per-circuit Connected/DataReceived/
//! Closed, tagged with the circuit's local key), and [`take_incoming`] (newly
//! minted inbound circuits awaiting accept/refuse). The C#/TS `sendPacket` sink +
//! `IncomingCircuit` event + per-circuit listeners become these drain points.
//!
//! Mirrors `Packet.NetRom.Transport.CircuitManager`.
//!
//! [`tick`]: CircuitManager::tick
//! [`circuit_mut`]: CircuitManager::circuit_mut
//! [`take_outbox`]: CircuitManager::take_outbox
//! [`take_events`]: CircuitManager::take_events
//! [`take_incoming`]: CircuitManager::take_incoming

use alloc::vec::Vec;

use super::circuit::{CircuitEvent, NetRomCircuit, OutboundPacket};
use super::circuit_options::NetRomCircuitOptions;
use crate::ax25::Callsign;
use crate::netrom::wire::{
    ConnectRequestInfo, NetRomNetworkHeader, NetRomOpcode, NetRomPacket, NetRomTransportHeader,
};

/// A circuit's local-table key: the `(index, id)` we allocated and the peer stamps
/// into datagrams addressed to us.
pub type CircuitKey = (u8, u8);

/// A freshly-minted inbound circuit awaiting the host's accept/refuse decision.
/// Mirrors `IncomingCircuitEventArgs`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct IncomingCircuit {
    /// The minted circuit's local key (route accept/refuse + later traffic by this).
    pub key: CircuitKey,
    /// The far node that originated the circuit.
    pub remote_node: Callsign,
    /// The end user the circuit is on behalf of (from the connect payload).
    pub originating_user: Callsign,
    /// The peer's circuit-table index (to address replies to it).
    pub peer_index: u8,
    /// The peer's circuit-table id.
    pub peer_id: u8,
    /// The window size the peer proposed in its Connect Request.
    pub proposed_window: u8,
}

struct Managed {
    circuit: NetRomCircuit,
    /// For an inbound circuit: the peer identity `(origin, peer_index, peer_id)`,
    /// so a retransmitted Connect Request (whose header names the peer's circuit,
    /// not ours) re-acks the existing circuit instead of minting a duplicate.
    peer_key: Option<(Callsign, u8, u8)>,
}

/// This node's NET/ROM L4 circuit table.
pub struct CircuitManager {
    local_node: Callsign,
    options: NetRomCircuitOptions,
    circuits: Vec<Managed>,
    next_index: u8,
    next_id: u8,
    outbox: Vec<OutboundPacket>,
    events: Vec<(CircuitKey, CircuitEvent)>,
    incoming: Vec<IncomingCircuit>,
}

impl CircuitManager {
    /// Construct the manager for a node.
    pub fn new(local_node: Callsign, options: NetRomCircuitOptions) -> Self {
        Self {
            local_node,
            options,
            circuits: Vec::new(),
            next_index: 0,
            next_id: 0,
            outbox: Vec::new(),
            events: Vec::new(),
            incoming: Vec::new(),
        }
    }

    /// Set the local node callsign stamped into the L3 origin of circuits minted
    /// *after* this call (existing circuits keep their origin).
    pub fn set_local_node(&mut self, node: Callsign) {
        self.local_node = node;
    }

    /// The number of live circuits.
    pub fn circuit_count(&self) -> usize {
        self.circuits.len()
    }

    /// A mutable handle to a circuit by key — the owner drives `connect` / `send` /
    /// `disconnect` on it, then drains the manager.
    pub fn circuit_mut(&mut self, key: CircuitKey) -> Option<&mut NetRomCircuit> {
        self.circuits
            .iter_mut()
            .find(|m| key_of(&m.circuit) == key)
            .map(|m| &mut m.circuit)
    }

    /// The lifecycle state of a circuit by key (immutable).
    pub fn circuit_state(&self, key: CircuitKey) -> Option<super::NetRomCircuitState> {
        self.circuits
            .iter()
            .find(|m| key_of(&m.circuit) == key)
            .map(|m| m.circuit.state())
    }

    /// The negotiated send-window of a circuit by key (immutable).
    pub fn circuit_window(&self, key: CircuitKey) -> Option<u8> {
        self.circuits
            .iter()
            .find(|m| key_of(&m.circuit) == key)
            .map(|m| m.circuit.window())
    }

    /// Whether the peer has choked this circuit (immutable) — the local sender is
    /// holding back until the peer drains and releases.
    pub fn circuit_peer_choked(&self, key: CircuitKey) -> Option<bool> {
        self.circuits
            .iter()
            .find(|m| key_of(&m.circuit) == key)
            .map(|m| m.circuit.peer_choked())
    }

    /// Mint a local circuit to `remote_node`, register it, and return its key. The
    /// owner then drives it via [`circuit_mut`](Self::circuit_mut).
    pub fn open_circuit(&mut self, remote_node: Callsign) -> CircuitKey {
        let (index, id) = self.allocate_key();
        let circuit = NetRomCircuit::new(index, id, self.local_node, remote_node, self.options);
        self.circuits.push(Managed {
            circuit,
            peer_key: None,
        });
        (index, id)
    }

    /// Feed an inbound datagram. Routes it to the addressed circuit, or, for a
    /// Connect Request with no matching circuit, mints an inbound circuit (raised
    /// via [`take_incoming`](Self::take_incoming)). Tolerant of stray datagrams.
    pub fn on_packet(&mut self, packet: &NetRomPacket, now_ms: u64) {
        let t = packet.transport;
        let key = (t.circuit_index, t.circuit_id);

        if let Some(m) = self.circuits.iter_mut().find(|m| key_of(&m.circuit) == key) {
            m.circuit.on_packet(packet, now_ms);
            return;
        }

        match NetRomOpcode::from_nibble(t.opcode) {
            Some(NetRomOpcode::ConnectRequest) => {
                // Dedup a retransmitted Connect Request by the peer's identity.
                let peer_key = (packet.network.origin, t.circuit_index, t.circuit_id);
                if let Some(m) = self
                    .circuits
                    .iter_mut()
                    .find(|m| m.peer_key == Some(peer_key))
                {
                    m.circuit.on_packet(packet, now_ms);
                    return;
                }
                self.mint_inbound(packet);
            }
            Some(NetRomOpcode::DisconnectRequest) => {
                // Courteously disconnect-ack so a half-open peer settles.
                let network = NetRomNetworkHeader {
                    origin: self.local_node,
                    destination: packet.network.origin,
                    time_to_live: self.options.time_to_live,
                };
                let transport = NetRomTransportHeader {
                    circuit_index: t.circuit_index,
                    circuit_id: t.circuit_id,
                    tx_sequence: 0,
                    rx_sequence: 0,
                    opcode: NetRomOpcode::DisconnectAcknowledge.as_u8(),
                    flags: 0,
                };
                self.outbox.push(OutboundPacket {
                    network,
                    transport,
                    payload: Vec::new(),
                });
            }
            _ => {} // stray datagram for an unknown circuit — drop.
        }
    }

    /// Accept an inbound circuit raised by [`take_incoming`](Self::take_incoming):
    /// adopt the peer's index/id + proposed window, move to Connected, send the
    /// Connect Acknowledge.
    pub fn accept(&mut self, incoming: &IncomingCircuit) {
        if let Some(c) = self.circuit_mut(incoming.key) {
            c.accept_inbound(
                incoming.peer_index,
                incoming.peer_id,
                incoming.proposed_window,
            );
        }
    }

    /// Refuse an inbound circuit: send a refusing Connect Acknowledge and drop it
    /// from the table (collecting its outbox first so the refusal still ships).
    pub fn refuse(&mut self, incoming: &IncomingCircuit) {
        let mut refusal = Vec::new();
        if let Some(m) = self
            .circuits
            .iter_mut()
            .find(|m| key_of(&m.circuit) == incoming.key)
        {
            m.circuit
                .refuse_inbound(incoming.peer_index, incoming.peer_id);
            refusal = m.circuit.take_outbox();
        }
        self.outbox.append(&mut refusal);
        self.circuits.retain(|m| key_of(&m.circuit) != incoming.key);
    }

    /// Advance every circuit's timers by one tick.
    pub fn tick(&mut self, now_ms: u64) {
        for m in &mut self.circuits {
            m.circuit.tick(now_ms);
        }
    }

    /// Drain the outbound datagrams queued across all circuits (and the manager's
    /// own reflections) since the last call.
    pub fn take_outbox(&mut self) -> Vec<OutboundPacket> {
        self.collect();
        core::mem::take(&mut self.outbox)
    }

    /// Drain the per-circuit lifecycle events since the last call, each tagged with
    /// its circuit's local key. A `Closed` event deregisters the circuit (after
    /// being surfaced here so the owner can notify the consumer).
    pub fn take_events(&mut self) -> Vec<(CircuitKey, CircuitEvent)> {
        self.collect();
        core::mem::take(&mut self.events)
    }

    /// Drain the inbound circuits minted since the last call, awaiting accept/refuse.
    pub fn take_incoming(&mut self) -> Vec<IncomingCircuit> {
        core::mem::take(&mut self.incoming)
    }

    // ─── Internals ──────────────────────────────────────────────────────

    fn mint_inbound(&mut self, request: &NetRomPacket) {
        let t = request.transport;
        let remote_node = request.network.origin;

        let mut originating_user = remote_node;
        let mut proposed_window = 0u8;
        if let Some(info) = ConnectRequestInfo::decode(request.payload) {
            proposed_window = info.proposed_window;
            originating_user = info.originating_user;
        }

        let peer_key = (remote_node, t.circuit_index, t.circuit_id);
        let (index, id) = self.allocate_key();
        let circuit = NetRomCircuit::new(index, id, self.local_node, remote_node, self.options);
        self.circuits.push(Managed {
            circuit,
            peer_key: Some(peer_key),
        });
        self.incoming.push(IncomingCircuit {
            key: (index, id),
            remote_node,
            originating_user,
            peer_index: t.circuit_index,
            peer_id: t.circuit_id,
            proposed_window,
        });
    }

    fn allocate_key(&mut self) -> (u8, u8) {
        for _ in 0..65536u32 {
            let index = self.next_index;
            let id = self.next_id;
            self.next_index = self.next_index.wrapping_add(1);
            if self.next_index == 0 {
                self.next_id = self.next_id.wrapping_add(1);
            }
            if !self
                .circuits
                .iter()
                .any(|m| key_of(&m.circuit) == (index, id))
            {
                return (index, id);
            }
        }
        // 65536 live circuits — practically unreachable; reuse the probe head.
        (self.next_index, self.next_id)
    }

    /// Sweep every circuit: aggregate its outbox + events into the manager's queues
    /// (tagging events with the circuit key), and deregister any that closed.
    fn collect(&mut self) {
        let mut new_out = Vec::new();
        let mut new_events = Vec::new();
        let mut closed: Vec<CircuitKey> = Vec::new();
        for m in &mut self.circuits {
            let key = key_of(&m.circuit);
            new_out.append(&mut m.circuit.take_outbox());
            for ev in m.circuit.take_events() {
                if matches!(ev, CircuitEvent::Closed(_)) {
                    closed.push(key);
                }
                new_events.push((key, ev));
            }
        }
        self.outbox.append(&mut new_out);
        self.events.append(&mut new_events);
        if !closed.is_empty() {
            self.circuits
                .retain(|m| !closed.contains(&key_of(&m.circuit)));
        }
    }
}

fn key_of(circuit: &NetRomCircuit) -> CircuitKey {
    (circuit.local_index(), circuit.local_id())
}

#[cfg(test)]
mod tests {
    //! The NET/ROM L4 behavioural suite — ported 1:1 from the TypeScript
    //! `tests/netrom/circuit*.test.ts` against the shared two-node harness. These
    //! are the faithfulness oracle: a divergence here means the Rust port drifted
    //! from the C#/TS reference, not that the test is wrong.

    use super::*;
    use crate::netrom::transport::circuit_state::{
        NetRomCircuitCloseReason as Reason, NetRomCircuitState as State,
    };
    use crate::netrom::wire::NetRomPacket;
    use alloc::collections::{BTreeMap, VecDeque};
    use alloc::vec::Vec;

    /// The harness seeds an arbitrary non-zero epoch; only the deltas matter.
    const SEED_MS: u64 = 1_750_000_000_000;

    fn call(text: &str) -> Callsign {
        Callsign::new(text.as_bytes(), 0).unwrap()
    }

    fn user() -> Callsign {
        call("M0LTE")
    }

    #[derive(Clone, Copy, PartialEq, Eq)]
    enum Side {
        A,
        B,
    }

    /// What a circuit's consumer observed — the sans-io analogue of the TS
    /// `CapturedCircuit` callbacks, filled by draining the manager's tagged events.
    #[derive(Default, Clone)]
    struct Captured {
        connected: bool,
        received: Vec<Vec<u8>>,
        closed: Vec<Reason>,
    }

    impl Captured {
        fn received_bytes(&self) -> Vec<u8> {
            let mut all = Vec::new();
            for r in &self.received {
                all.extend_from_slice(r);
            }
            all
        }
    }

    /// Two `CircuitManager`s (GB7AAA ↔ GB7BBB) wired through an in-process packet
    /// queue on a shared clock, with per-direction loss injection. Mirrors
    /// `tests/netrom/circuit-pair-harness.ts`.
    struct Harness {
        a_node: Callsign,
        b_node: Callsign,
        a: CircuitManager,
        b: CircuitManager,
        now_ms: u64,
        wire: VecDeque<(Side, OutboundPacket)>,
        drop_a_to_b: u32,
        drop_b_to_a: u32,
        auto_accept_b: bool,
        accepted_b: Vec<CircuitKey>,
        cap_a: BTreeMap<CircuitKey, Captured>,
        cap_b: BTreeMap<CircuitKey, Captured>,
    }

    impl Harness {
        fn with_options(a_opts: NetRomCircuitOptions, b_opts: NetRomCircuitOptions) -> Self {
            let a_node = call("GB7AAA");
            let b_node = call("GB7BBB");
            Self {
                a_node,
                b_node,
                a: CircuitManager::new(a_node, a_opts),
                b: CircuitManager::new(b_node, b_opts),
                now_ms: SEED_MS,
                wire: VecDeque::new(),
                drop_a_to_b: 0,
                drop_b_to_a: 0,
                auto_accept_b: false,
                accepted_b: Vec::new(),
                cap_a: BTreeMap::new(),
                cap_b: BTreeMap::new(),
            }
        }

        fn new() -> Self {
            Self::with_options(
                NetRomCircuitOptions::default(),
                NetRomCircuitOptions::default(),
            )
        }

        /// One options object applied to both ends — the TS single-arg form
        /// (`optsB = optionsB ?? options`).
        fn with_both(opts: NetRomCircuitOptions) -> Self {
            Self::with_options(opts, opts)
        }

        /// B auto-accepts every inbound circuit (records the accepted key in order).
        fn auto_accept_on_b(&mut self) {
            self.auto_accept_b = true;
        }

        /// A originates a circuit toward B; returns its key. Drive it with
        /// [`connect_a`](Self::connect_a) / [`send_a`](Self::send_a) etc.
        fn open_from_a(&mut self) -> CircuitKey {
            self.a.open_circuit(self.b_node)
        }

        fn connect_a(&mut self, key: CircuitKey, originating_user: Callsign) {
            let now = self.now_ms;
            self.a
                .circuit_mut(key)
                .unwrap()
                .connect(originating_user, now);
        }

        fn send_a(&mut self, key: CircuitKey, data: &[u8]) {
            let now = self.now_ms;
            self.a.circuit_mut(key).unwrap().send(data, now);
        }

        fn disconnect_a(&mut self, key: CircuitKey) {
            let now = self.now_ms;
            self.a.circuit_mut(key).unwrap().disconnect(now);
        }

        fn send_b(&mut self, key: CircuitKey, data: &[u8]) {
            let now = self.now_ms;
            self.b.circuit_mut(key).unwrap().send(data, now);
        }

        /// Tell B's circuit its consumer drained a delivered message (releases choke).
        fn drain_b(&mut self, key: CircuitKey) {
            self.b.circuit_mut(key).unwrap().on_delivery_drained();
        }

        fn drop_next_a_to_b(&mut self, n: u32) {
            self.drop_a_to_b += n;
        }

        fn drop_next_b_to_a(&mut self, n: u32) {
            self.drop_b_to_a += n;
        }

        /// Deliver every queued datagram, sweeping both managers after each, until
        /// the wire drains (guarded against livelock).
        fn pump(&mut self) {
            self.sweep();
            let mut guard = 0u32;
            while let Some((dest, packet)) = self.wire.pop_front() {
                guard += 1;
                assert!(guard < 100_000, "harness livelock");
                let now = self.now_ms;
                self.manager_mut(dest).on_packet(&as_packet(&packet), now);
                self.sweep();
            }
        }

        /// Advance the shared clock, tick both managers, then pump.
        fn advance(&mut self, delta_ms: u64) {
            self.now_ms += delta_ms;
            let now = self.now_ms;
            self.a.tick(now);
            self.b.tick(now);
            self.pump();
        }

        /// Drain both managers once: accept/refuse inbound, record events, ship
        /// outbound onto the wire. The within-side accept→ack cascade settles in
        /// this single pass; cross-side cascades are driven by [`pump`](Self::pump).
        fn sweep(&mut self) {
            for side in [Side::A, Side::B] {
                let incoming = self.manager_mut(side).take_incoming();
                for inc in &incoming {
                    if side == Side::B && self.auto_accept_b {
                        self.b.accept(inc);
                        self.accepted_b.push(inc.key);
                    } else {
                        self.manager_mut(side).refuse(inc);
                    }
                }
                let events = self.manager_mut(side).take_events();
                for (key, ev) in events {
                    self.record(side, key, ev);
                }
                let outbox = self.manager_mut(side).take_outbox();
                for p in outbox {
                    self.enqueue(side, p);
                }
            }
        }

        fn enqueue(&mut self, from: Side, packet: OutboundPacket) {
            let dropped = match from {
                Side::A if self.drop_a_to_b > 0 => {
                    self.drop_a_to_b -= 1;
                    true
                }
                Side::B if self.drop_b_to_a > 0 => {
                    self.drop_b_to_a -= 1;
                    true
                }
                _ => false,
            };
            if dropped {
                return;
            }
            let dest = match from {
                Side::A => Side::B,
                Side::B => Side::A,
            };
            self.wire.push_back((dest, packet));
        }

        fn record(&mut self, side: Side, key: CircuitKey, ev: CircuitEvent) {
            let map = match side {
                Side::A => &mut self.cap_a,
                Side::B => &mut self.cap_b,
            };
            let cap = map.entry(key).or_default();
            match ev {
                CircuitEvent::Connected => cap.connected = true,
                CircuitEvent::DataReceived(data) => cap.received.push(data),
                CircuitEvent::Closed(reason) => cap.closed.push(reason),
            }
        }

        fn manager_mut(&mut self, side: Side) -> &mut CircuitManager {
            match side {
                Side::A => &mut self.a,
                Side::B => &mut self.b,
            }
        }

        // ── Observations ──────────────────────────────────────────────
        fn cap_a(&self, key: CircuitKey) -> Captured {
            self.cap_a.get(&key).cloned().unwrap_or_default()
        }

        fn cap_b(&self, key: CircuitKey) -> Captured {
            self.cap_b.get(&key).cloned().unwrap_or_default()
        }

        fn accepted(&self, n: usize) -> CircuitKey {
            self.accepted_b[n]
        }
    }

    /// Borrow an `OutboundPacket` as a wire `NetRomPacket` for delivery.
    fn as_packet(p: &OutboundPacket) -> NetRomPacket<'_> {
        NetRomPacket {
            network: p.network,
            transport: p.transport,
            payload: &p.payload,
        }
    }

    fn first_bytes(received: &[Vec<u8>]) -> Vec<u8> {
        received.iter().map(|r| r[0]).collect()
    }

    // ─── circuit.test.ts — behavioural FSM ──────────────────────────────

    #[test]
    fn connect_then_acknowledge_brings_both_ends_up() {
        let mut h = Harness::new();
        h.auto_accept_on_b();
        let a = h.open_from_a();
        h.connect_a(a, user());
        h.pump();

        assert!(
            h.cap_a(a).connected,
            "the Connect Acknowledge reached the originator"
        );
        assert_eq!(h.a.circuit_state(a), Some(State::Connected));
        assert_eq!(h.accepted_b.len(), 1);
        let b = h.accepted(0);
        assert_eq!(h.b.circuit_state(b), Some(State::Connected));
        assert_eq!(h.b.circuit_mut(b).unwrap().remote_node(), h.a_node);
    }

    #[test]
    fn window_is_negotiated_down_to_the_responders_ceiling() {
        let mut h = Harness::with_options(
            NetRomCircuitOptions {
                window_size: 8,
                ..Default::default()
            },
            NetRomCircuitOptions {
                window_size: 2,
                ..Default::default()
            },
        );
        h.auto_accept_on_b();
        let a = h.open_from_a();
        h.connect_a(a, user());
        h.pump();

        assert_eq!(h.accepted_b.len(), 1);
        assert_eq!(h.b.circuit_window(h.accepted(0)), Some(2));
    }

    #[test]
    fn information_flows_with_piggybacked_acks() {
        let mut h = Harness::new();
        h.auto_accept_on_b();
        let a = h.open_from_a();
        h.connect_a(a, user());
        h.pump();

        let payload = b"hello netrom";
        h.send_a(a, payload);
        h.pump();
        let b = h.accepted(0);
        assert_eq!(h.cap_b(b).received_bytes(), payload);

        let reply = b"hi back";
        h.send_b(b, reply);
        h.pump();
        assert_eq!(h.cap_a(a).received_bytes(), reply);
    }

    #[test]
    fn a_multi_frame_burst_delivers_in_order_within_the_window() {
        let mut h = Harness::with_both(NetRomCircuitOptions {
            window_size: 4,
            ..Default::default()
        });
        h.auto_accept_on_b();
        let a = h.open_from_a();
        h.connect_a(a, user());
        h.pump();

        for i in 1u8..=6 {
            h.send_a(a, &[i]);
        }
        h.pump();

        let received = h.cap_b(h.accepted(0)).received;
        assert_eq!(received.len(), 6);
        assert_eq!(first_bytes(&received), alloc::vec![1, 2, 3, 4, 5, 6]);
    }

    #[test]
    fn a_large_payload_fragments_and_reassembles_at_236_bytes() {
        let mut h = Harness::with_both(NetRomCircuitOptions {
            window_size: 8,
            ..Default::default()
        });
        h.auto_accept_on_b();
        let a = h.open_from_a();
        h.connect_a(a, user());
        h.pump();

        let big: Vec<u8> = (0..600).map(|i| (i & 0xff) as u8).collect();
        h.send_a(a, &big);
        h.pump();

        let received = h.cap_b(h.accepted(0)).received;
        assert_eq!(received.len(), 1, "reassembled to a single logical message");
        assert_eq!(received[0], big);
    }

    #[test]
    fn disconnect_is_acknowledged_and_closes_both_ends() {
        let mut h = Harness::new();
        h.auto_accept_on_b();
        let a = h.open_from_a();
        h.connect_a(a, user());
        h.pump();
        let b = h.accepted(0);

        h.disconnect_a(a);
        h.pump();

        assert_eq!(
            h.a.circuit_state(a),
            None,
            "A's circuit was deregistered on close"
        );
        assert_eq!(h.cap_a(a).closed, alloc::vec![Reason::Normal]);
        assert_eq!(
            h.b.circuit_state(b),
            None,
            "B's circuit was deregistered on close"
        );
        assert!(h.cap_b(b).closed.contains(&Reason::Normal));
    }

    #[test]
    fn a_refused_connect_closes_the_originator_as_refused() {
        let mut h = Harness::new(); // no auto-accept ⇒ B refuses
        let a = h.open_from_a();
        h.connect_a(a, user());
        h.pump();

        assert!(!h.cap_a(a).connected);
        assert_eq!(h.cap_a(a).closed, alloc::vec![Reason::Refused]);
        assert_eq!(h.a.circuit_state(a), None);
    }

    #[test]
    fn a_lost_information_frame_is_retransmitted_after_the_timeout() {
        let mut h = Harness::with_both(NetRomCircuitOptions {
            window_size: 4,
            retransmit_timeout_ms: 5000,
            max_retries: 3,
            ..Default::default()
        });
        h.auto_accept_on_b();
        let a = h.open_from_a();
        h.connect_a(a, user());
        h.pump();

        h.drop_next_a_to_b(1);
        let payload = b"retransmit me";
        h.send_a(a, payload);
        h.pump();
        let b = h.accepted(0);
        assert_eq!(h.cap_b(b).received.len(), 0, "the only copy was dropped");

        h.advance(6000);
        assert_eq!(h.cap_b(b).received_bytes(), payload);
    }

    #[test]
    fn a_lost_connect_request_is_retransmitted_then_succeeds() {
        let mut h = Harness::with_both(NetRomCircuitOptions {
            retransmit_timeout_ms: 5000,
            max_retries: 3,
            ..Default::default()
        });
        h.auto_accept_on_b();
        let a = h.open_from_a();
        h.drop_next_a_to_b(1); // lose the first Connect Request
        h.connect_a(a, user());
        h.pump();
        assert!(!h.cap_a(a).connected);

        h.advance(6000); // retransmit the connect
        assert!(
            h.cap_a(a).connected,
            "the retransmitted Connect Request was acknowledged"
        );
        assert_eq!(h.accepted_b.len(), 1);
    }

    #[test]
    fn connect_fails_after_retries_are_exhausted() {
        let mut h = Harness::with_both(NetRomCircuitOptions {
            retransmit_timeout_ms: 5000,
            max_retries: 2,
            ..Default::default()
        });
        h.auto_accept_on_b();
        let a = h.open_from_a();
        h.drop_next_a_to_b(3);
        h.connect_a(a, user());
        h.pump();
        h.advance(6000); // retry 1 (dropped)
        h.advance(6000); // retry 2 (dropped) → exhausted
        h.advance(6000); // tick that trips the give-up

        assert!(!h.cap_a(a).connected);
        assert_eq!(h.cap_a(a).closed, alloc::vec![Reason::Timeout]);
    }

    // ─── circuit-recovery.test.ts — loss recovery + flow control ────────

    #[test]
    fn a_sequence_gap_triggers_a_nak_and_selective_retransmit() {
        let mut h = Harness::with_both(NetRomCircuitOptions {
            window_size: 4,
            retransmit_timeout_ms: 30000,
            ..Default::default()
        });
        h.auto_accept_on_b();
        let a = h.open_from_a();
        h.connect_a(a, user());
        h.pump();

        h.drop_next_a_to_b(1); // drop the first Information (seq 0)
        h.send_a(a, &[10]);
        h.send_a(a, &[20]);
        h.send_a(a, &[30]);
        h.pump(); // B sees seq 1,2 out of order → NAK seq 0 → A retransmits

        let received = h.cap_b(h.accepted(0)).received;
        assert_eq!(first_bytes(&received), alloc::vec![10, 20, 30]);
    }

    #[test]
    fn choke_stops_the_sender_until_released() {
        let mut h = Harness::with_both(NetRomCircuitOptions {
            window_size: 8,
            choke_threshold: 1,
            retransmit_timeout_ms: 30000,
            ..Default::default()
        });
        h.auto_accept_on_b();
        let a = h.open_from_a();
        h.connect_a(a, user());
        h.pump();
        let b = h.accepted(0);

        h.send_a(a, b"one");
        h.pump();
        assert_eq!(h.cap_b(b).received.len(), 1);
        assert_eq!(h.a.circuit_peer_choked(a), Some(true));

        h.send_a(a, b"two");
        h.pump();
        assert_eq!(
            h.cap_b(b).received.len(),
            1,
            "A is choked, so the second frame is held"
        );

        h.drain_b(b);
        h.pump();
        let received = h.cap_b(b).received;
        assert_eq!(
            received.len(),
            2,
            "the held frame went out once choke was released"
        );
        assert_eq!(received[1], b"two");

        h.drain_b(b); // drain "two" → release the re-choke
        h.pump();
        assert_eq!(h.a.circuit_peer_choked(a), Some(false));
    }

    // ─── circuit-manager.test.ts — demux, dedup, tolerance ──────────────

    #[test]
    fn two_concurrent_circuits_demultiplex_independently() {
        let mut h = Harness::new();
        h.auto_accept_on_b();
        let c1 = h.open_from_a();
        h.connect_a(c1, user());
        h.pump();
        let c2 = h.open_from_a();
        h.connect_a(c2, call("G0ABC"));
        h.pump();

        assert_eq!(h.accepted_b.len(), 2);
        h.send_a(c1, b"circuit one");
        h.send_a(c2, b"circuit two");
        h.pump();

        assert_eq!(h.cap_b(h.accepted(0)).received_bytes(), b"circuit one");
        assert_eq!(h.cap_b(h.accepted(1)).received_bytes(), b"circuit two");
    }

    #[test]
    fn closed_circuits_are_removed_from_the_table() {
        let mut h = Harness::new();
        h.auto_accept_on_b();
        let a = h.open_from_a();
        h.connect_a(a, user());
        h.pump();
        assert_eq!(h.a.circuit_count(), 1);

        h.disconnect_a(a);
        h.pump();
        assert_eq!(h.a.circuit_count(), 0);
        assert_eq!(h.b.circuit_count(), 0);
    }

    #[test]
    fn a_retransmitted_connect_request_does_not_mint_a_duplicate_inbound_circuit() {
        let mut h = Harness::with_both(NetRomCircuitOptions {
            retransmit_timeout_ms: 5000,
            max_retries: 3,
            ..Default::default()
        });
        h.auto_accept_on_b();
        let a = h.open_from_a();
        h.drop_next_b_to_a(1); // lose B's first Connect Acknowledge
        h.connect_a(a, user());
        h.pump();
        assert!(!h.cap_a(a).connected, "the connect-ack was dropped");
        assert_eq!(
            h.b.circuit_count(),
            1,
            "B minted exactly one inbound circuit"
        );

        h.advance(6000); // A retransmits the Connect Request
        assert!(
            h.cap_a(a).connected,
            "the re-ack from the deduped circuit completes the connect"
        );
        assert_eq!(
            h.b.circuit_count(),
            1,
            "the retransmit re-acked the existing circuit"
        );
        assert_eq!(h.accepted_b.len(), 1, "IncomingCircuit fired exactly once");
    }

    #[test]
    fn an_inbound_connect_with_no_listener_is_refused() {
        let mut h = Harness::new(); // no auto-accept ⇒ refuse
        let a = h.open_from_a();
        h.connect_a(a, user());
        h.pump();

        assert!(!h.cap_a(a).connected);
        assert_eq!(h.cap_a(a).closed, alloc::vec![Reason::Refused]);
        assert_eq!(
            h.b.circuit_count(),
            0,
            "the refused inbound circuit was deregistered"
        );
    }

    #[test]
    fn a_stray_datagram_for_an_unknown_circuit_is_dropped_without_throwing() {
        let mut manager = CircuitManager::new(call("GB7XXX"), NetRomCircuitOptions::default());
        let stray = OutboundPacket {
            network: NetRomNetworkHeader {
                origin: call("GB7YYY"),
                destination: call("GB7XXX"),
                time_to_live: 10,
            },
            transport: NetRomTransportHeader {
                circuit_index: 99,
                circuit_id: 99,
                tx_sequence: 0,
                rx_sequence: 0,
                opcode: NetRomOpcode::Information.as_u8(),
                flags: 0,
            },
            payload: alloc::vec![1, 2, 3],
        };
        manager.on_packet(&as_packet(&stray), SEED_MS);
        assert_eq!(
            manager.take_outbox().len(),
            0,
            "a stray Information datagram is silently dropped"
        );
    }

    #[test]
    fn a_disconnect_for_an_unknown_circuit_is_courteously_acknowledged() {
        let mut manager = CircuitManager::new(call("GB7XXX"), NetRomCircuitOptions::default());
        let disc = OutboundPacket {
            network: NetRomNetworkHeader {
                origin: call("GB7YYY"),
                destination: call("GB7XXX"),
                time_to_live: 10,
            },
            transport: NetRomTransportHeader {
                circuit_index: 5,
                circuit_id: 5,
                tx_sequence: 0,
                rx_sequence: 0,
                opcode: NetRomOpcode::DisconnectRequest.as_u8(),
                flags: 0,
            },
            payload: Vec::new(),
        };
        manager.on_packet(&as_packet(&disc), SEED_MS);
        let sent = manager.take_outbox();
        assert_eq!(sent.len(), 1);
        assert_eq!(
            NetRomOpcode::from_nibble(sent[0].transport.opcode),
            Some(NetRomOpcode::DisconnectAcknowledge)
        );
    }
}
