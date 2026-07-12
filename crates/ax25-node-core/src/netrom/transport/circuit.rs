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
#[cfg(feature = "netrom-compress")]
use crate::netrom::wire::{ConnectAckInfo, CONNECT_REQUEST_INFO_EXTENDED_LEN, FLAG_COMPRESSED};
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
    /// This fragment's payload is part of a compressed logical frame — carries the
    /// [`FLAG_COMPRESSED`] flag on (re)transmit. Gated behind `netrom-compress`.
    #[cfg(feature = "netrom-compress")]
    compressed: bool,
    sent_at: u64,
    retries: u8,
}

/// A queued send fragment (waiting for window room).
struct Fragment {
    bytes: Vec<u8>,
    more_follows: bool,
    /// This fragment belongs to a compressed logical send — see [`Unacked::compressed`].
    #[cfg(feature = "netrom-compress")]
    compressed: bool,
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

    // Compression negotiation (BPQ L4Compress). `compression_enabled` is the settled
    // per-circuit result — true only when BOTH ends advertised compression at connect
    // time; until then it is false (send raw, the always-safe path).
    // `reassembly_compressed` tracks whether the more-follows fragments currently
    // being accumulated were flagged compressed, so the whole logical frame is
    // inflated exactly once at the end. Gated behind `netrom-compress`.
    #[cfg(feature = "netrom-compress")]
    compression_enabled: bool,
    #[cfg(feature = "netrom-compress")]
    reassembly_compressed: bool,

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
            #[cfg(feature = "netrom-compress")]
            compression_enabled: false,
            #[cfg(feature = "netrom-compress")]
            reassembly_compressed: false,
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
    /// True once the circuit is connected and *both* ends negotiated LinBPQ-style L4
    /// payload compression — i.e. outbound data is being zlib-compressed and flagged
    /// [`FLAG_COMPRESSED`]. False (the safe default) when either end declined, in
    /// which case data is sent raw. Gated behind `netrom-compress`. Mirrors C#
    /// `NetRomCircuit.CompressionNegotiated`.
    #[cfg(feature = "netrom-compress")]
    pub fn compression_negotiated(&self) -> bool {
        self.compression_enabled
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

        // When compression is negotiated on this circuit, compress the WHOLE logical
        // send into one zlib stream, then fragment that stream. Every fragment of a
        // compressed frame carries the Compressed flag; the receiver reassembles all
        // its more-follows fragments and inflates the concatenation once. Falls back
        // to raw when compression would not shrink the data (BPQ does the same:
        // "if complen >= dataLen … just send") — no point paying the zlib header for
        // an expansion, and raw is always decodable (the flag is per-frame).
        #[cfg(feature = "netrom-compress")]
        let compressed_buf: Option<Vec<u8>> = if self.compression_enabled {
            let z = super::compression::compress(data);
            if z.len() < data.len() {
                Some(z)
            } else {
                None
            }
        } else {
            None
        };
        #[cfg(feature = "netrom-compress")]
        let (body, compressed): (&[u8], bool) = match &compressed_buf {
            Some(z) => (z.as_slice(), true),
            None => (data, false),
        };
        #[cfg(not(feature = "netrom-compress"))]
        let body: &[u8] = data;

        let frag = self.options.fragment_size.max(1);
        let mut offset = 0;
        while offset < body.len() {
            let take = frag.min(body.len() - offset);
            let more = offset + take < body.len();
            self.send_queue.push_back(Fragment {
                bytes: body[offset..offset + take].to_vec(),
                more_follows: more,
                #[cfg(feature = "netrom-compress")]
                compressed,
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
            Some(NetRomOpcode::ConnectAcknowledge) => self.on_connect_acknowledge(
                &t,
                #[cfg(feature = "netrom-compress")]
                packet.payload,
                now_ms,
            ),
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
    pub fn accept_inbound(
        &mut self,
        peer_index: u8,
        peer_id: u8,
        proposed_window: u8,
        #[cfg(feature = "netrom-compress")] peer_offers_compression: bool,
    ) {
        self.remote_index = peer_index;
        self.remote_id = peer_id;
        let proposed = if proposed_window == 0 {
            self.options.window_size
        } else {
            proposed_window
        };
        self.window = proposed.min(self.options.window_size).clamp(1, 127);

        // Compression is enabled on this circuit only if BOTH ends advertised it:
        // the peer's Connect Request carried the offer AND our options enable it. The
        // Connect Acknowledge mirrors the agreement back so the originator knows.
        #[cfg(feature = "netrom-compress")]
        {
            self.compression_enabled = self.options.compression_enabled && peer_offers_compression;
        }

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
                // Take the list out so each frame can be borrowed while calling
                // `&mut self.send_information`, then put it back with bumped timers
                // (behaviour-identical to the prior clone-into-tuple, and it avoids
                // cloning every payload on each retransmit).
                let mut frames = core::mem::take(&mut self.unacked);
                for u in &frames {
                    self.send_information(
                        u.sequence,
                        &u.payload,
                        u.more_follows,
                        #[cfg(feature = "netrom-compress")]
                        u.compressed,
                    );
                }
                for u in &mut frames {
                    u.sent_at = now_ms;
                    u.retries += 1;
                }
                self.unacked = frames;
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

    fn on_connect_acknowledge(
        &mut self,
        t: &NetRomTransportHeader,
        #[cfg(feature = "netrom-compress")] info: &[u8],
        now_ms: u64,
    ) {
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

        // Compression negotiation: enable only if WE offered (options.compression_enabled)
        // AND the peer's Connect Acknowledge mirrored the agreement back. A peer that
        // ignored our offer (or that we never offered to) replies with the vanilla
        // empty/short ack ⇒ compression_enabled stays false ⇒ we send raw, always safe.
        #[cfg(feature = "netrom-compress")]
        {
            self.compression_enabled =
                self.options.compression_enabled && ConnectAckInfo::agrees_compression(info);
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
                // Track whether this logical frame is a compressed stream. BPQ sets
                // the Compressed flag on every fragment, so the FIRST fragment is
                // authoritative; read it at the start of accumulation and hold it
                // until the frame completes.
                #[cfg(feature = "netrom-compress")]
                if self.reassembly.is_empty() {
                    self.reassembly_compressed = t.compressed();
                }
                self.reassembly.extend_from_slice(payload);
            }
            if !t.more_follows() && !self.reassembly.is_empty() {
                let whole = core::mem::take(&mut self.reassembly);

                // Inflate first if the logical frame was sent compressed. A
                // corrupt/undecodable stream is dropped (fail closed) — but still
                // acked so the sender advances (a NAK can't recover a bad zlib
                // stream), never delivered as garbage and never panicking.
                #[cfg(feature = "netrom-compress")]
                let whole = if self.reassembly_compressed {
                    self.reassembly_compressed = false;
                    match super::compression::try_decompress(&whole) {
                        Some(w) => w,
                        None => {
                            self.send_information_acknowledge(false);
                            self.pump_send_queue(now_ms);
                            return;
                        }
                    }
                } else {
                    whole
                };

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
        let cri = ConnectRequestInfo {
            proposed_window: self.options.window_size.clamp(1, 127),
            originating_user: user,
            originating_node: self.local_node,
        };

        // When compression is enabled we OFFER it via the LinBPQ extended-connect form
        // (canonical 15 octets + a 2-octet timer trailer carrying the compress bit). A
        // peer that ignores the trailer just sees a normal Connect Request, so offering
        // is interop-safe; we only actually compress once the peer's Connect
        // Acknowledge confirms it agreed. Compression off ⇒ canonical 15-octet form.
        #[cfg(feature = "netrom-compress")]
        if self.options.compression_enabled {
            let mut info = [0u8; CONNECT_REQUEST_INFO_EXTENDED_LEN];
            cri.encode_extended(&mut info, self.options.proposed_timer_seconds, true)
                .expect("17-byte buffer");
            self.emit(t, &info);
            return;
        }

        let mut info = [0u8; crate::netrom::wire::CONNECT_REQUEST_INFO_LEN];
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

        // Mirror the compression agreement back to the originator (LinBPQ extended
        // Connect Acknowledge) only when compression was actually agreed; otherwise
        // the canonical empty-info Connect Acknowledge is sent, so a non-compressing
        // circuit is byte-for-byte vanilla NET/ROM.
        #[cfg(feature = "netrom-compress")]
        if !refused && self.compression_enabled {
            if let Some(info) =
                ConnectAckInfo::encode(self.window, self.options.time_to_live, true)
            {
                self.emit(t, &info);
                return;
            }
        }

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

    fn send_information(
        &mut self,
        seq: u8,
        payload: &[u8],
        more_follows: bool,
        #[cfg(feature = "netrom-compress")] compressed: bool,
    ) {
        let mut flags = 0u8;
        if more_follows {
            flags |= FLAG_MORE_FOLLOWS;
        }
        #[cfg(feature = "netrom-compress")]
        if compressed {
            flags |= FLAG_COMPRESSED;
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
            self.send_information(
                seq,
                &fragment.bytes,
                fragment.more_follows,
                #[cfg(feature = "netrom-compress")]
                fragment.compressed,
            );
            self.unacked.push(Unacked {
                sequence: seq,
                payload: fragment.bytes,
                more_follows: fragment.more_follows,
                #[cfg(feature = "netrom-compress")]
                compressed: fragment.compressed,
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
        // Take the in-flight list out so each matching frame can be borrowed while
        // calling `&mut self.send_information`, then put it back with bumped timers
        // (behaviour-identical to the prior clone-into-tuple).
        let mut frames = core::mem::take(&mut self.unacked);
        for u in &frames {
            if u.sequence == seq || mod256_after(u.sequence, seq) {
                self.send_information(
                    u.sequence,
                    &u.payload,
                    u.more_follows,
                    #[cfg(feature = "netrom-compress")]
                    u.compressed,
                );
            }
        }
        for u in &mut frames {
            if u.sequence == seq || mod256_after(u.sequence, seq) {
                u.sent_at = now_ms;
                u.retries += 1;
            }
        }
        self.unacked = frames;
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
        #[cfg(feature = "netrom-compress")]
        {
            self.reassembly_compressed = false;
            self.compression_enabled = false;
        }
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
            #[cfg(feature = "netrom-compress")]
            false,
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

// ─── L4 compression (BPQ L4Compress) — negotiation + send/recv, feature-gated ───
#[cfg(all(test, feature = "netrom-compress"))]
mod compression_tests {
    use super::*;
    use crate::ax25::Callsign;
    use crate::netrom::transport::compression;
    use crate::netrom::wire::{
        ConnectAckInfo, ConnectRequestInfo, CONNECT_REQUEST_INFO_EXTENDED_LEN,
        CONNECT_REQUEST_INFO_LEN, FLAG_COMPRESSED, MAX_PAYLOAD,
    };

    const NOW: u64 = 1_000;

    fn cs(b: &[u8]) -> Callsign {
        Callsign::new(b, 0).unwrap()
    }

    fn on() -> NetRomCircuitOptions {
        NetRomCircuitOptions {
            compression_enabled: true,
            ..Default::default()
        }
    }

    fn off() -> NetRomCircuitOptions {
        NetRomCircuitOptions::default()
    }

    /// Feed a batch of a peer's outbound datagrams into `to`.
    fn feed(pkts: &[OutboundPacket], to: &mut NetRomCircuit) {
        for p in pkts {
            let pkt = NetRomPacket {
                network: p.network,
                transport: p.transport,
                payload: &p.payload,
            };
            to.on_packet(&pkt, NOW);
        }
    }

    /// Bring up an A→B circuit under the given options, running the full connect
    /// handshake (offer/agree wired through as A's Connect Request advertised).
    /// Returns both connected ends. A = index 1/id 7, B = index 2/id 9.
    fn connected_pair(
        opts_a: NetRomCircuitOptions,
        opts_b: NetRomCircuitOptions,
    ) -> (NetRomCircuit, NetRomCircuit) {
        let (na, nb) = (cs(b"GB7AAA"), cs(b"GB7BBB"));
        let mut a = NetRomCircuit::new(1, 7, na, nb, opts_a);
        let mut b = NetRomCircuit::new(2, 9, nb, na, opts_b);

        a.connect(cs(b"M0LTE"), NOW);
        let creq = a.take_outbox().remove(0);
        let cri = ConnectRequestInfo::decode(&creq.payload).unwrap();
        let offered = ConnectRequestInfo::offers_compression(&creq.payload);
        b.accept_inbound(
            creq.transport.circuit_index,
            creq.transport.circuit_id,
            cri.proposed_window,
            offered,
        );
        // Deliver B's Connect Acknowledge back to A so A settles its side.
        let cack = b.take_outbox();
        feed(&cack, &mut a);
        (a, b)
    }

    // ── Negotiation ────────────────────────────────────────────────────

    #[test]
    fn offers_via_extended_connect_when_enabled() {
        let mut a = NetRomCircuit::new(1, 7, cs(b"GB7AAA"), cs(b"GB7BBB"), on());
        a.connect(cs(b"M0LTE"), NOW);
        let creq = a.take_outbox().remove(0);
        assert_eq!(creq.payload.len(), CONNECT_REQUEST_INFO_EXTENDED_LEN);
        assert!(ConnectRequestInfo::offers_compression(&creq.payload));
    }

    #[test]
    fn sends_canonical_connect_when_disabled() {
        // Feature compiled in, but the option off ⇒ byte-identical plain NET/ROM.
        let mut a = NetRomCircuit::new(1, 7, cs(b"GB7AAA"), cs(b"GB7BBB"), off());
        a.connect(cs(b"M0LTE"), NOW);
        let creq = a.take_outbox().remove(0);
        assert_eq!(creq.payload.len(), CONNECT_REQUEST_INFO_LEN);
        assert!(!ConnectRequestInfo::offers_compression(&creq.payload));
    }

    #[test]
    fn both_ends_offer_and_agree_enables_compression() {
        let (a, b) = connected_pair(on(), on());
        assert_eq!(a.state(), NetRomCircuitState::Connected);
        assert_eq!(b.state(), NetRomCircuitState::Connected);
        assert!(a.compression_negotiated(), "originator settled compression on");
        assert!(b.compression_negotiated(), "acceptor settled compression on");
    }

    #[test]
    fn responder_declining_leaves_both_ends_plain() {
        // A offers, B has compression disabled ⇒ B replies with the vanilla empty
        // Connect Acknowledge, and neither end compresses.
        let (na, nb) = (cs(b"GB7AAA"), cs(b"GB7BBB"));
        let mut a = NetRomCircuit::new(1, 7, na, nb, on());
        let mut b = NetRomCircuit::new(2, 9, nb, na, off());
        a.connect(cs(b"M0LTE"), NOW);
        let creq = a.take_outbox().remove(0);
        let cri = ConnectRequestInfo::decode(&creq.payload).unwrap();
        b.accept_inbound(
            creq.transport.circuit_index,
            creq.transport.circuit_id,
            cri.proposed_window,
            ConnectRequestInfo::offers_compression(&creq.payload),
        );
        let cack = b.take_outbox();
        assert_eq!(cack.len(), 1);
        assert!(
            cack[0].payload.is_empty(),
            "a declining Connect Acknowledge is the vanilla empty-info form"
        );
        assert!(!ConnectAckInfo::agrees_compression(&cack[0].payload));
        feed(&cack, &mut a);
        assert!(!a.compression_negotiated());
        assert!(!b.compression_negotiated());
    }

    #[test]
    fn initiator_declining_leaves_both_ends_plain() {
        // A never offers (canonical connect); B is willing but has nothing to agree
        // to ⇒ both stay plain.
        let (a, b) = connected_pair(off(), on());
        assert!(!a.compression_negotiated());
        assert!(!b.compression_negotiated());
    }

    // ── Send / receive ─────────────────────────────────────────────────

    /// A moderately compressible ~4 KiB payload (repeated text + a little entropy)
    /// whose zlib stream exceeds the 236-byte fragment size, forcing a multi-fragment
    /// compressed logical send. Stays under the 8 KiB decompress cap.
    fn multi_fragment_payload() -> Vec<u8> {
        let mut rng: u32 = 0x1234_5678;
        let mut data = Vec::new();
        for _ in 0..70 {
            data.extend_from_slice(b"GB7RDG NET/ROM node broadcast quality 192 via GB7RDG-7 ");
            for _ in 0..8 {
                rng = rng.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                data.push((rng >> 24) as u8);
            }
        }
        data
    }

    #[test]
    fn compressed_logical_send_round_trips_through_fragment_reassemble_inflate() {
        let original = multi_fragment_payload();
        let z = compression::compress(&original);
        assert!(z.len() < original.len(), "payload must actually compress");
        assert!(
            z.len() > MAX_PAYLOAD,
            "compressed stream must span >1 fragment (got {})",
            z.len()
        );

        // A window wide enough to emit every fragment in one burst (so the whole
        // logical frame is in the outbox to inspect + deliver at once).
        let opts = NetRomCircuitOptions {
            compression_enabled: true,
            window_size: 32,
            ..Default::default()
        };
        let (mut a, mut b) = connected_pair(opts, opts);
        a.send(&original, NOW);
        let frames = a.take_outbox();

        // Every fragment of a compressed logical send carries the Compressed flag;
        // all but the last carry more-follows.
        assert!(frames.len() >= 2, "expected multiple fragments");
        for (i, f) in frames.iter().enumerate() {
            assert_eq!(
                NetRomOpcode::from_nibble(f.transport.opcode),
                Some(NetRomOpcode::Information)
            );
            assert!(f.transport.compressed(), "fragment {i} lacks the Compressed flag");
            let last = i == frames.len() - 1;
            assert_eq!(f.transport.more_follows(), !last, "more-follows on fragment {i}");
        }

        feed(&frames, &mut b);
        let events = b.take_events();
        let received: Vec<Vec<u8>> = events
            .into_iter()
            .filter_map(|e| match e {
                CircuitEvent::DataReceived(d) => Some(d),
                _ => None,
            })
            .collect();
        assert_eq!(received.len(), 1, "reassembled to a single logical frame");
        assert_eq!(received[0], original, "inflated payload matches the original");
    }

    #[test]
    fn incompressible_payload_uses_the_raw_per_send_fallback() {
        // High-entropy data zlib cannot shrink ⇒ sent raw, Compressed flag clear,
        // even though the circuit negotiated compression.
        let mut rng: u32 = 0xDEAD_BEEF;
        let mut payload = Vec::new();
        for _ in 0..48 {
            rng = rng.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            payload.push((rng >> 24) as u8);
        }
        assert!(
            compression::compress(&payload).len() >= payload.len(),
            "test precondition: payload must not compress"
        );

        let (mut a, mut b) = connected_pair(on(), on());
        assert!(a.compression_negotiated());
        a.send(&payload, NOW);
        let frames = a.take_outbox();
        assert_eq!(frames.len(), 1);
        assert!(
            !frames[0].transport.compressed(),
            "raw fallback must leave the Compressed flag clear"
        );

        feed(&frames, &mut b);
        let received: Vec<Vec<u8>> = b
            .take_events()
            .into_iter()
            .filter_map(|e| match e {
                CircuitEvent::DataReceived(d) => Some(d),
                _ => None,
            })
            .collect();
        assert_eq!(received, alloc::vec![payload]);
    }

    #[test]
    fn disabled_circuit_never_sets_the_compressed_flag() {
        // Feature compiled in, both ends' option off ⇒ behaves like plain NET/ROM:
        // no negotiation, no Compressed flag, data flows raw.
        let (mut a, mut b) = connected_pair(off(), off());
        assert!(!a.compression_negotiated());
        let payload = b"plain netrom data, no compression on this circuit";
        a.send(payload, NOW);
        let frames = a.take_outbox();
        assert_eq!(frames.len(), 1);
        assert!(
            !frames[0].transport.compressed(),
            "a disabled circuit must not compress"
        );
        feed(&frames, &mut b);
        let received: Vec<Vec<u8>> = b
            .take_events()
            .into_iter()
            .filter_map(|e| match e {
                CircuitEvent::DataReceived(d) => Some(d),
                _ => None,
            })
            .collect();
        assert_eq!(received, alloc::vec![payload.to_vec()]);
    }

    #[test]
    fn a_corrupt_compressed_frame_is_dropped_but_still_acked() {
        // Fail-closed: an undecodable zlib payload flagged Compressed must not be
        // delivered as garbage nor panic — it is dropped, and still acked so the
        // sender advances (a NAK can't recover a bad zlib stream).
        let (_a, mut b) = connected_pair(on(), on());
        // Craft an Information frame addressed to B (its local key 2/9), sequence 0,
        // flagged Compressed, with a payload that is not a valid zlib stream.
        let garbage = OutboundPacket {
            network: NetRomNetworkHeader {
                origin: cs(b"GB7AAA"),
                destination: cs(b"GB7BBB"),
                time_to_live: 10,
            },
            transport: NetRomTransportHeader {
                circuit_index: 2,
                circuit_id: 9,
                tx_sequence: 0,
                rx_sequence: 0,
                opcode: NetRomOpcode::Information.as_u8(),
                flags: FLAG_COMPRESSED,
            },
            payload: alloc::vec![0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF],
        };
        feed(&[garbage], &mut b);

        // No data delivered upward…
        let delivered = b
            .take_events()
            .into_iter()
            .any(|e| matches!(e, CircuitEvent::DataReceived(_)));
        assert!(!delivered, "a corrupt compressed frame must not be delivered");
        // …but an Information Acknowledge was still emitted.
        let acked = b.take_outbox().into_iter().any(|p| {
            NetRomOpcode::from_nibble(p.transport.opcode)
                == Some(NetRomOpcode::InformationAcknowledge)
        });
        assert!(acked, "the dropped frame is still acked so the sender advances");
    }
}
