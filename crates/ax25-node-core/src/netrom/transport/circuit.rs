//! One end of a NET/ROM L4 virtual circuit: a hand-written, end-to-end
//! sliding-window transport (connect / info / disconnect with negotiated window,
//! 8-bit sequence numbers, choke flow control, selective-NAK retransmit, and L4
//! fragment/reassembly at 236 bytes). It runs *above* the AX.25 interlink and
//! knows nothing of AX.25 itself.
//!
//! **Sans-io.** Where the C#/TS port wires a `sendPacket` sink + `DataReceived` /
//! `Connected` / `Closed` listeners (closures), this `no_std` port collects
//! outbound datagrams into [`NetRomCircuit::take_outbox`] and lifecycle
//! notifications into [`NetRomCircuit::take_events`], which the owner drains after
//! each call. Closures-as-fields are awkward in `no_std`; the drain-the-queue
//! shape is the idiomatic equivalent and is also trivially testable.
//!
//! **Time.** All timing is in milliseconds via a `now_ms` the caller threads into
//! the entry points (`connect` / `send` / `disconnect` / `on_packet` / `tick`) —
//! the analogue of the C# `TimeProvider` / TS injected `now()`. The owner drives
//! retransmits by calling [`NetRomCircuit::tick`] at the clock cadence.
//!
//! **Allocation.** The send queue, in-flight list, and reassembly buffer are
//! `alloc::Vec` (the firmware provides a heap sized for a few links with a small
//! window); the desktop's unbounded collections port directly. Mirrors
//! `Packet.NetRom.Transport.NetRomCircuit`.

use alloc::collections::VecDeque;
use alloc::vec::Vec;

use super::circuit_options::NetRomCircuitOptions;
use super::circuit_state::{NetRomCircuitCloseReason, NetRomCircuitState};
use crate::ax25::Callsign;
use crate::netrom::wire::{
    ConnectRequestInfo, NetRomNetworkHeader, NetRomOpcode, NetRomPacket, NetRomTransportHeader,
    FLAG_CHOKE, FLAG_MORE_FOLLOWS, FLAG_NAK,
};

/// An outbound datagram the circuit wants shipped over its interlink. The owner
/// encodes it ([`OutboundPacket::encode`]) into a PID-0xCF I-frame, routing by
/// `network.destination`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct OutboundPacket {
    /// The L3 network header (origin = our node, destination = the far node).
    pub network: NetRomNetworkHeader,
    /// The L4 transport header.
    pub transport: NetRomTransportHeader,
    /// The transport payload.
    pub payload: Vec<u8>,
}

impl OutboundPacket {
    /// Encode this datagram (headers + payload) into `dst`, returning the length.
    pub fn encode(&self, dst: &mut [u8]) -> Option<usize> {
        NetRomPacket {
            network: self.network,
            transport: self.transport,
            payload: &self.payload,
        }
        .encode(dst)
    }
}

/// A lifecycle notification surfaced to the circuit's owner/consumer.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CircuitEvent {
    /// The circuit reached [`NetRomCircuitState::Connected`].
    Connected,
    /// A reassembled logical frame of user data was delivered upward.
    DataReceived(Vec<u8>),
    /// The circuit reached [`NetRomCircuitState::Disconnected`], with the reason.
    Closed(NetRomCircuitCloseReason),
}

/// An in-flight Information message awaiting ack.
struct Unacked {
    sequence: u8,
    payload: Vec<u8>,
    more_follows: bool,
    sent_at: u64,
    retries: u8,
}

/// A queued send fragment (waiting for window room).
struct Fragment {
    bytes: Vec<u8>,
    more_follows: bool,
}

/// One end of a NET/ROM L4 virtual circuit.
pub struct NetRomCircuit {
    options: NetRomCircuitOptions,

    local_index: u8,
    local_id: u8,
    remote_index: u8,
    remote_id: u8,

    local_node: Callsign,
    remote_node: Callsign,

    state: NetRomCircuitState,
    window: u8,

    // Send side — 8-bit sequence space (mod 256).
    vs: u8,
    va: u8,
    send_queue: VecDeque<Fragment>,
    unacked: Vec<Unacked>,

    // Receive side.
    vr: u8,
    reassembly: Vec<u8>,

    // Flow control.
    peer_choked: bool,
    local_choked: bool,
    pending_deliveries: usize,

    // Connect/disconnect retransmit bookkeeping.
    control_deadline: u64,
    control_timer_armed: bool,
    control_retries: u8,

    connect_user: Option<Callsign>,

    outbox: Vec<OutboundPacket>,
    events: Vec<CircuitEvent>,
}

impl NetRomCircuit {
    /// Construct a circuit end. The owner allocates the local index/id and supplies
    /// the node callsigns for the L3 header.
    pub fn new(
        local_index: u8,
        local_id: u8,
        local_node: Callsign,
        remote_node: Callsign,
        options: NetRomCircuitOptions,
    ) -> Self {
        Self {
            options,
            local_index,
            local_id,
            remote_index: 0,
            remote_id: 0,
            local_node,
            remote_node,
            state: NetRomCircuitState::Disconnected,
            window: options.window_size,
            vs: 0,
            va: 0,
            send_queue: VecDeque::new(),
            unacked: Vec::new(),
            vr: 0,
            reassembly: Vec::new(),
            peer_choked: false,
            local_choked: false,
            pending_deliveries: 0,
            control_deadline: 0,
            control_timer_armed: false,
            control_retries: 0,
            connect_user: None,
            outbox: Vec::new(),
            events: Vec::new(),
        }
    }

    // ─── Accessors ──────────────────────────────────────────────────────

    /// Our circuit-table index (the value the peer addresses replies to).
    pub fn local_index(&self) -> u8 {
        self.local_index
    }
    /// Our circuit-table id (qualifies [`Self::local_index`]).
    pub fn local_id(&self) -> u8 {
        self.local_id
    }
    /// The far node this circuit reaches.
    pub fn remote_node(&self) -> Callsign {
        self.remote_node
    }
    /// The current lifecycle state.
    pub fn state(&self) -> NetRomCircuitState {
        self.state
    }
    /// The negotiated send-window size.
    pub fn window(&self) -> u8 {
        self.window
    }
    /// True while the peer has us choked (we are holding Information back).
    pub fn peer_choked(&self) -> bool {
        self.peer_choked
    }
    /// Send-side V(s): the next send sequence to allocate (mod 256).
    pub fn send_state(&self) -> u8 {
        self.vs
    }
    /// Send-side V(a): the oldest unacknowledged sequence (mod 256).
    pub fn ack_state(&self) -> u8 {
        self.va
    }
    /// Receive-side V(r): the next send sequence expected from the peer (mod 256).
    pub fn receive_state(&self) -> u8 {
        self.vr
    }

    /// Drain the outbound datagrams queued since the last call — the owner ships
    /// each over the circuit's interlink. (Sans-io equivalent of the C# `SendPacket`.)
    pub fn take_outbox(&mut self) -> Vec<OutboundPacket> {
        core::mem::take(&mut self.outbox)
    }

    /// Drain the lifecycle events queued since the last call (Connected /
    /// DataReceived / Closed). The owner deregisters the circuit on `Closed` and
    /// forwards the rest to the consumer.
    pub fn take_events(&mut self) -> Vec<CircuitEvent> {
        core::mem::take(&mut self.events)
    }

    // ─── Origination ────────────────────────────────────────────────────

    /// Originate the circuit: send a Connect Request (proposing our window) and arm
    /// the connect retransmit timer. No-op unless Disconnected.
    pub fn connect(&mut self, originating_user: Callsign, now_ms: u64) {
        if self.state != NetRomCircuitState::Disconnected {
            return;
        }
        self.state = NetRomCircuitState::Connecting;
        self.control_retries = 0;
        self.connect_user = Some(originating_user);
        self.send_connect_request();
        self.arm_control_timer(now_ms);
    }

    // ─── Application send ───────────────────────────────────────────────

    /// Queue user data for transmission, fragmenting into ≤`fragment_size`
    /// Information messages and pumping as many as the window + peer-choke allow.
    /// No-op (data dropped) if not Connected.
    pub fn send(&mut self, data: &[u8], now_ms: u64) {
        if self.state != NetRomCircuitState::Connected || data.is_empty() {
            return;
        }
        let frag = self.options.fragment_size.max(1);
        let mut offset = 0;
        while offset < data.len() {
            let take = frag.min(data.len() - offset);
            let more = offset + take < data.len();
            self.send_queue.push_back(Fragment {
                bytes: data[offset..offset + take].to_vec(),
                more_follows: more,
            });
            offset += take;
        }
        self.pump_send_queue(now_ms);
    }

    // ─── Disconnect ─────────────────────────────────────────────────────

    /// Tear the circuit down: send a Disconnect Request and arm its timer. Closes
    /// locally if never established. Idempotent.
    pub fn disconnect(&mut self, now_ms: u64) {
        match self.state {
            NetRomCircuitState::Disconnected | NetRomCircuitState::Disconnecting => {}
            NetRomCircuitState::Connecting => {
                self.close(NetRomCircuitCloseReason::Normal);
            }
            NetRomCircuitState::Connected => {
                self.state = NetRomCircuitState::Disconnecting;
                self.control_retries = 0;
                self.send_disconnect_request();
                self.arm_control_timer(now_ms);
            }
        }
    }

    // ─── Inbound ────────────────────────────────────────────────────────

    /// Feed an inbound datagram addressed to this circuit. Tolerant of any opcode —
    /// an unexpected message for the current state is ignored, never panics.
    pub fn on_packet(&mut self, packet: &NetRomPacket, now_ms: u64) {
        let t = packet.transport;
        match NetRomOpcode::from_nibble(t.opcode) {
            Some(NetRomOpcode::ConnectRequest) => self.on_connect_request(),
            Some(NetRomOpcode::ConnectAcknowledge) => self.on_connect_acknowledge(&t, now_ms),
            Some(NetRomOpcode::DisconnectRequest) => self.on_disconnect_request(),
            Some(NetRomOpcode::DisconnectAcknowledge) => self.on_disconnect_acknowledge(),
            Some(NetRomOpcode::Information) => self.on_information(&t, packet.payload, now_ms),
            Some(NetRomOpcode::InformationAcknowledge) => {
                self.on_information_acknowledge(&t, now_ms)
            }
            None => {}
        }
    }

    /// Accept an inbound circuit: adopt the peer's index/id + proposed window, move
    /// to Connected, and send the Connect Acknowledge. (Owner-driven, for an
    /// incoming connect.)
    pub fn accept_inbound(&mut self, peer_index: u8, peer_id: u8, proposed_window: u8) {
        self.remote_index = peer_index;
        self.remote_id = peer_id;
        let proposed = if proposed_window == 0 {
            self.options.window_size
        } else {
            proposed_window
        };
        self.window = proposed.min(self.options.window_size).clamp(1, 127);
        self.state = NetRomCircuitState::Connected;
        self.send_connect_acknowledge(false);
        self.fire_connected();
    }

    /// Refuse an inbound circuit: Connect Acknowledge with the refuse bit, stay
    /// Disconnected. (Owner-driven, when it cannot accept.)
    pub fn refuse_inbound(&mut self, peer_index: u8, peer_id: u8) {
        self.remote_index = peer_index;
        self.remote_id = peer_id;
        self.send_connect_acknowledge(true);
        self.state = NetRomCircuitState::Disconnected;
    }

    // ─── Timer ──────────────────────────────────────────────────────────

    /// Drive time-based behaviour: retransmit the oldest unacknowledged Information
    /// (or the pending connect/disconnect control message) whose timeout elapsed,
    /// failing the circuit once retries are exhausted. Cheap when nothing is due.
    pub fn tick(&mut self, now_ms: u64) {
        // Control (connect/disconnect) retransmit.
        if self.control_timer_armed && now_ms >= self.control_deadline {
            if self.control_retries >= self.options.max_retries {
                self.control_timer_armed = false;
                self.close(NetRomCircuitCloseReason::Timeout);
                return;
            }
            self.control_retries += 1;
            match self.state {
                NetRomCircuitState::Connecting => self.send_connect_request(),
                NetRomCircuitState::Disconnecting => self.send_disconnect_request(),
                _ => {}
            }
            self.arm_control_timer(now_ms);
        }

        // Information retransmit — oldest unacked first.
        if self.state == NetRomCircuitState::Connected && !self.unacked.is_empty() {
            let oldest_due = self.unacked[0].sent_at + self.options.retransmit_timeout_ms;
            if now_ms >= oldest_due {
                if self.unacked[0].retries >= self.options.max_retries {
                    self.close(NetRomCircuitCloseReason::Timeout);
                    return;
                }
                // Go-back style: retransmit every in-flight frame, bumping timers.
                let frames: Vec<(u8, Vec<u8>, bool)> = self
                    .unacked
                    .iter()
                    .map(|u| (u.sequence, u.payload.clone(), u.more_follows))
                    .collect();
                for (seq, payload, more) in &frames {
                    self.send_information(*seq, payload, *more);
                }
                for u in &mut self.unacked {
                    u.sent_at = now_ms;
                    u.retries += 1;
                }
            }
        }
    }

    // ─── Receive-side flow control ──────────────────────────────────────

    /// Tell the circuit of the consumer's drain progress so it can release a
    /// previously-asserted choke once its receive backlog drains below threshold.
    pub fn on_delivery_drained(&mut self) {
        if self.pending_deliveries > 0 {
            self.pending_deliveries -= 1;
        }
        self.maybe_release_choke();
    }

    // ─── FSM handlers ───────────────────────────────────────────────────

    fn on_connect_acknowledge(&mut self, t: &NetRomTransportHeader, now_ms: u64) {
        if self.state != NetRomCircuitState::Connecting {
            return;
        }
        // The peer's own index/id ride in the TX/RX-sequence slots of a connect-ack.
        self.remote_index = t.tx_sequence;
        self.remote_id = t.rx_sequence;
        self.control_timer_armed = false;

        if t.choke() {
            self.close(NetRomCircuitCloseReason::Refused);
            return;
        }

        self.state = NetRomCircuitState::Connected;
        self.fire_connected();
        self.pump_send_queue(now_ms);
    }

    fn on_connect_request(&mut self) {
        // A retransmitted Connect Request after we're already up: just re-ack.
        if self.state == NetRomCircuitState::Connected {
            self.send_connect_acknowledge(false);
        }
        // Otherwise the manager owns inbound-connect minting; a bare circuit ignores it.
    }

    fn on_disconnect_request(&mut self) {
        self.send_disconnect_acknowledge();
        if self.state != NetRomCircuitState::Disconnected {
            self.close(NetRomCircuitCloseReason::Normal);
        }
    }

    fn on_disconnect_acknowledge(&mut self) {
        if self.state == NetRomCircuitState::Disconnecting {
            self.control_timer_armed = false;
            self.close(NetRomCircuitCloseReason::Normal);
        }
    }

    fn on_information(&mut self, t: &NetRomTransportHeader, payload: &[u8], now_ms: u64) {
        if self.state != NetRomCircuitState::Connected {
            return;
        }
        self.absorb_ack(t.rx_sequence);
        self.apply_peer_choke(t.choke());

        if t.tx_sequence == self.vr {
            self.vr = self.vr.wrapping_add(1);
            if !payload.is_empty() {
                self.reassembly.extend_from_slice(payload);
            }
            if !t.more_follows() && !self.reassembly.is_empty() {
                let whole = core::mem::take(&mut self.reassembly);
                if self.options.choke_threshold > 0 {
                    self.pending_deliveries += 1;
                }
                self.fire_data_received(whole);
                self.maybe_assert_choke();
            }
            self.send_information_acknowledge(false);
        } else {
            // Out-of-sequence: NAK a future frame (a gap); a stale duplicate just
            // gets a plain ack so the sender advances.
            let future = mod256_after(t.tx_sequence, self.vr);
            self.send_information_acknowledge(future);
        }

        // The piggybacked ack we absorbed (and any peer choke-release) may have
        // opened the window — pump queued fragments (the TS does this inside
        // absorbAck / applyPeerChoke).
        self.pump_send_queue(now_ms);
    }

    fn on_information_acknowledge(&mut self, t: &NetRomTransportHeader, now_ms: u64) {
        if self.state != NetRomCircuitState::Connected {
            return;
        }
        self.apply_peer_choke(t.choke());

        if t.nak() {
            self.absorb_ack(t.rx_sequence);
            self.retransmit_from(t.rx_sequence, now_ms);
        } else {
            self.absorb_ack(t.rx_sequence);
        }
        self.pump_send_queue(now_ms);
    }

    // ─── Send helpers ───────────────────────────────────────────────────

    fn send_connect_request(&mut self) {
        let t = NetRomTransportHeader {
            circuit_index: self.local_index,
            circuit_id: self.local_id,
            tx_sequence: 0,
            rx_sequence: 0,
            opcode: NetRomOpcode::ConnectRequest.as_u8(),
            flags: 0,
        };
        let user = match self.connect_user {
            Some(u) if !u.base().is_empty() => u,
            _ => self.local_node,
        };
        let mut info = [0u8; crate::netrom::wire::CONNECT_REQUEST_INFO_LEN];
        let cri = ConnectRequestInfo {
            proposed_window: self.options.window_size.clamp(1, 127),
            originating_user: user,
            originating_node: self.local_node,
        };
        cri.encode(&mut info).expect("15-byte buffer");
        self.emit(t, &info);
    }

    fn send_connect_acknowledge(&mut self, refused: bool) {
        let t = NetRomTransportHeader {
            circuit_index: self.remote_index,
            circuit_id: self.remote_id,
            tx_sequence: self.local_index,
            rx_sequence: self.local_id,
            opcode: NetRomOpcode::ConnectAcknowledge.as_u8(),
            flags: if refused { FLAG_CHOKE } else { 0 },
        };
        self.emit(t, &[]);
    }

    fn send_disconnect_request(&mut self) {
        let t = NetRomTransportHeader {
            circuit_index: self.remote_index,
            circuit_id: self.remote_id,
            tx_sequence: 0,
            rx_sequence: 0,
            opcode: NetRomOpcode::DisconnectRequest.as_u8(),
            flags: 0,
        };
        self.emit(t, &[]);
    }

    fn send_disconnect_acknowledge(&mut self) {
        let t = NetRomTransportHeader {
            circuit_index: self.remote_index,
            circuit_id: self.remote_id,
            tx_sequence: 0,
            rx_sequence: 0,
            opcode: NetRomOpcode::DisconnectAcknowledge.as_u8(),
            flags: 0,
        };
        self.emit(t, &[]);
    }

    fn send_information(&mut self, seq: u8, payload: &[u8], more_follows: bool) {
        let mut flags = 0u8;
        if more_follows {
            flags |= FLAG_MORE_FOLLOWS;
        }
        if self.local_choked {
            flags |= FLAG_CHOKE;
        }
        let t = NetRomTransportHeader {
            circuit_index: self.remote_index,
            circuit_id: self.remote_id,
            tx_sequence: seq,
            rx_sequence: self.vr, // piggyback our receive expectation
            opcode: NetRomOpcode::Information.as_u8(),
            flags,
        };
        self.emit(t, payload);
    }

    fn send_information_acknowledge(&mut self, nak: bool) {
        let mut flags = 0u8;
        if nak {
            flags |= FLAG_NAK;
        }
        if self.local_choked {
            flags |= FLAG_CHOKE;
        }
        let t = NetRomTransportHeader {
            circuit_index: self.remote_index,
            circuit_id: self.remote_id,
            tx_sequence: 0,
            rx_sequence: self.vr,
            opcode: NetRomOpcode::InformationAcknowledge.as_u8(),
            flags,
        };
        self.emit(t, &[]);
    }

    fn emit(&mut self, transport: NetRomTransportHeader, payload: &[u8]) {
        let network = NetRomNetworkHeader {
            origin: self.local_node,
            destination: self.remote_node,
            time_to_live: self.options.time_to_live,
        };
        self.outbox.push(OutboundPacket {
            network,
            transport,
            payload: payload.to_vec(),
        });
    }

    // ─── Window + ack mechanics ─────────────────────────────────────────

    fn pump_send_queue(&mut self, now_ms: u64) {
        if self.state != NetRomCircuitState::Connected || self.peer_choked {
            return;
        }
        while !self.send_queue.is_empty() && (self.unacked.len() as u8) < self.window {
            let fragment = self.send_queue.pop_front().unwrap();
            let seq = self.vs;
            self.vs = self.vs.wrapping_add(1);
            self.send_information(seq, &fragment.bytes, fragment.more_follows);
            self.unacked.push(Unacked {
                sequence: seq,
                payload: fragment.bytes,
                more_follows: fragment.more_follows,
                sent_at: now_ms,
                retries: 0,
            });
        }
    }

    /// Absorb a cumulative ack: every in-flight sequence strictly before `expected`
    /// (mod 256, within the window) is acked.
    fn absorb_ack(&mut self, expected: u8) {
        if self.unacked.is_empty() {
            self.va = expected;
            return;
        }
        self.unacked.retain(|u| !seq_acked(u.sequence, expected));
        self.va = expected;
        // Window may have opened — but the caller pumps (mirrors the TS, which
        // re-pumps here too); we defer the pump to the caller's `pump_send_queue`
        // to keep `now_ms` out of this helper. (The TS absorbAck calls pump with
        // its stored clock; here on_information_acknowledge pumps right after.)
    }

    fn retransmit_from(&mut self, seq: u8, now_ms: u64) {
        let to_send: Vec<(u8, Vec<u8>, bool)> = self
            .unacked
            .iter()
            .filter(|u| u.sequence == seq || mod256_after(u.sequence, seq))
            .map(|u| (u.sequence, u.payload.clone(), u.more_follows))
            .collect();
        for (s, payload, more) in &to_send {
            self.send_information(*s, payload, *more);
        }
        for u in &mut self.unacked {
            if u.sequence == seq || mod256_after(u.sequence, seq) {
                u.sent_at = now_ms;
                u.retries += 1;
            }
        }
    }

    // ─── Choke ──────────────────────────────────────────────────────────

    fn apply_peer_choke(&mut self, choke: bool) {
        if choke {
            self.peer_choked = true;
        } else if self.peer_choked {
            self.peer_choked = false;
            // Peer released choke — the caller re-pumps.
        }
    }

    fn maybe_assert_choke(&mut self) {
        if self.options.choke_threshold > 0
            && self.pending_deliveries >= self.options.choke_threshold
            && !self.local_choked
        {
            self.local_choked = true;
        }
    }

    fn maybe_release_choke(&mut self) {
        if self.local_choked && self.pending_deliveries < self.options.choke_threshold {
            self.local_choked = false;
            if self.state == NetRomCircuitState::Connected {
                self.send_information_acknowledge(false);
            }
        }
    }

    // ─── Lifecycle ──────────────────────────────────────────────────────

    fn arm_control_timer(&mut self, now_ms: u64) {
        self.control_deadline = now_ms + self.options.retransmit_timeout_ms;
        self.control_timer_armed = true;
    }

    fn close(&mut self, reason: NetRomCircuitCloseReason) {
        if self.state == NetRomCircuitState::Disconnected {
            return;
        }
        self.state = NetRomCircuitState::Disconnected;
        self.control_timer_armed = false;
        self.unacked.clear();
        self.send_queue.clear();
        self.reassembly.clear();
        self.fire_closed(reason);
    }

    // ─── Event fan-out (into the drainable queue) ───────────────────────

    fn fire_data_received(&mut self, data: Vec<u8>) {
        self.events.push(CircuitEvent::DataReceived(data));
    }
    fn fire_connected(&mut self) {
        self.events.push(CircuitEvent::Connected);
    }
    fn fire_closed(&mut self, reason: NetRomCircuitCloseReason) {
        self.events.push(CircuitEvent::Closed(reason));
    }
}

/// True if sequence `seq` is acknowledged by a peer that now expects `expected`:
/// seq is in [va, expected) walking forward mod 256, bounded by the window.
fn seq_acked(seq: u8, expected: u8) -> bool {
    let dist = expected.wrapping_sub(seq);
    (1..=128).contains(&dist)
}

/// True if `a` is strictly after `b` within a half-window horizon (mod 256).
fn mod256_after(a: u8, b: u8) -> bool {
    let dist = a.wrapping_sub(b);
    (1..=128).contains(&dist)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ax25::Callsign;
    use crate::netrom::wire::ConnectRequestInfo;

    fn cs(b: &[u8]) -> Callsign {
        Callsign::new(b, 0).unwrap()
    }

    /// Drain `from`'s outbox and feed each datagram into `to`. Returns the count.
    fn deliver(from: &mut NetRomCircuit, to: &mut NetRomCircuit, now: u64) -> usize {
        let out = from.take_outbox();
        for p in &out {
            let pkt = NetRomPacket {
                network: p.network,
                transport: p.transport,
                payload: &p.payload,
            };
            to.on_packet(&pkt, now);
        }
        out.len()
    }

    #[test]
    fn full_handshake_data_round_trip_and_disconnect() {
        let opts = NetRomCircuitOptions::default();
        let (node_a, node_b) = (cs(b"M0LTE"), cs(b"GB7RDG"));
        let now = 1000u64;

        // A originates.
        let mut a = NetRomCircuit::new(1, 7, node_a, node_b, opts);
        a.connect(cs(b"M0LTE"), now);
        let a_out = a.take_outbox();
        assert_eq!(a_out.len(), 1);
        let req = &a_out[0];
        assert_eq!(
            NetRomOpcode::from_nibble(req.transport.opcode),
            Some(NetRomOpcode::ConnectRequest)
        );
        let cri = ConnectRequestInfo::decode(&req.payload).unwrap();
        assert_eq!(cri.proposed_window, opts.window_size);

        // The manager mints B in response to the Connect Request and accepts.
        let mut b = NetRomCircuit::new(2, 9, node_b, node_a, opts);
        b.accept_inbound(
            req.transport.circuit_index,
            req.transport.circuit_id,
            cri.proposed_window,
        );
        assert_eq!(b.state(), NetRomCircuitState::Connected);
        assert!(b.take_events().contains(&CircuitEvent::Connected));

        // B's Connect Acknowledge → A reaches Connected.
        deliver(&mut b, &mut a, now);
        assert_eq!(a.state(), NetRomCircuitState::Connected);
        assert!(a.take_events().contains(&CircuitEvent::Connected));

        // A → B user data (one fragment), acked.
        a.send(b"hello world", now);
        deliver(&mut a, &mut b, now);
        assert_eq!(
            b.take_events(),
            alloc::vec![CircuitEvent::DataReceived(b"hello world".to_vec())]
        );
        deliver(&mut b, &mut a, now); // InfoAck
        assert_eq!(a.ack_state(), a.send_state(), "all in-flight acked");

        // Clean disconnect both ways.
        a.disconnect(now);
        deliver(&mut a, &mut b, now); // DisconnectRequest
        assert_eq!(b.state(), NetRomCircuitState::Disconnected);
        assert!(b
            .take_events()
            .contains(&CircuitEvent::Closed(NetRomCircuitCloseReason::Normal)));
        deliver(&mut b, &mut a, now); // DisconnectAcknowledge
        assert_eq!(a.state(), NetRomCircuitState::Disconnected);
        assert!(a
            .take_events()
            .contains(&CircuitEvent::Closed(NetRomCircuitCloseReason::Normal)));
    }
}
