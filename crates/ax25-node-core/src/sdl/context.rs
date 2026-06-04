//! Per-session mutable state — ports `Packet.Ax25.Session.Ax25SessionContext`.
//!
//! Holds the sequence variables, flags, queues, and negotiated link parameters
//! that the generated SDL transitions read and mutate. Field names track the
//! spec's variable names (AX.25 v2.2 §4.2.2 sequence numbers, §C4.3 flags) so the
//! dispatcher's typed `match` reads cleanly against them.
//!
//! `no_std` + `alloc`: the growable per-session collections (the I-frame transmit
//! queue, the sent-frame retransmit store, the out-of-sequence receive store, the
//! once-per-cycle selective-retransmit set) use `alloc` containers. A Pico node
//! runs a handful of links with a small window (research §6), so these stay tiny;
//! a heapless follow-up can swap them for fixed-capacity maps without touching the
//! dispatcher.

extern crate alloc;
use alloc::collections::{BTreeMap, BTreeSet, VecDeque};
use alloc::vec::Vec;

use super::quirks::Quirks;

/// One payload + PID queued for / retained from transmission.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Payload {
    /// The Layer-3 information field.
    pub data: Vec<u8>,
    /// The PID octet that accompanies it.
    pub pid: u8,
}

impl Payload {
    /// Construct a payload.
    pub fn new(data: Vec<u8>, pid: u8) -> Self {
        Self { data, pid }
    }
}

/// Mutable per-connection AX.25 data-link state. One per `(local, remote, port)`.
///
/// Ports `Ax25SessionContext`. V(S)/V(A)/V(R) are stored as the underlying 0–127
/// value; the modulus (8 or 128, from [`is_extended`](Self::is_extended)) is
/// applied when comparing / incrementing.
#[derive(Debug, Clone)]
pub struct SessionContext {
    // ─── Sequence variables (§4.2.2) ────────────────────────────────────
    /// Send state variable — N(S) of the next I-frame to send.
    pub vs: u8,
    /// Acknowledge state variable — last acknowledged sent I-frame.
    pub va: u8,
    /// Receive state variable — N(S) of the next I-frame expected.
    pub vr: u8,
    /// Retry counter — retransmissions of the current outstanding poll.
    pub rc: u32,

    // ─── Flags (§C4.3) ──────────────────────────────────────────────────
    /// Layer 3 is busy and cannot receive I frames (own RNR sent).
    pub own_receiver_busy: bool,
    /// Remote station is busy and cannot receive I frames (peer RNR seen).
    pub peer_receiver_busy: bool,
    /// I frames received but not yet acknowledged.
    pub acknowledge_pending: bool,
    /// A REJ has been sent to the remote (mod-8 implicit reject).
    pub reject_exception: bool,
    /// An SREJ has been sent to the remote.
    pub selective_reject_exception: bool,
    /// Count of outstanding SREJ exceptions (§C4.3).
    pub srej_exception_count: u32,
    /// SABM(E) was sent by request of Layer 3 (DL-CONNECT request).
    pub layer3_initiated: bool,
    /// Node policy — accept inbound SABM/SABME (figc4.1 `able_to_establish`).
    pub accept_incoming: bool,

    /// Scratch register for figc4.7 `Invoke_Retransmission`: V(s) at routine
    /// entry, so the go-back-N loop knows when it has caught up. `None` when
    /// unset.
    pub x: Option<u8>,

    /// Set by the T1 expiry handler; consumed + cleared by `Select_T1_Value`.
    /// Records "T1 fired at least once since the last Select_T1_Value".
    pub t1_had_expired: bool,
    /// T1 time-remaining (ms) captured when `stop_T1` last ran. Consumed by the
    /// `Select_T1_Value` SRT IIR; `0` on a fresh session or after a timeout.
    pub t1_remaining_when_last_stopped_ms: u32,

    // ─── Negotiated link parameters (§6.7.2, XID defaults) ───────────────
    /// Maximum information field length in octets (N1). Default 256.
    pub n1: u32,
    /// Maximum number of retries (N2). Default 10.
    pub n2: u32,
    /// Maximum outstanding I frames (k). Default 4 (mod-8) / negotiated.
    pub k: u32,
    /// `true` for mod-128 (SABME/extended); `false` for mod-8 (SABM).
    pub is_extended: bool,
    /// `true` if SREJ has been negotiated via XID.
    pub srej_enabled: bool,
    /// `true` if the segmenter/reassembler has been negotiated via XID.
    pub segmenter_reassembler_enabled: bool,
    /// `true` for half-duplex operation.
    pub half_duplex: bool,
    /// `true` if implicit reject (v2.0) is selected; `false` = selective (v2.2).
    pub implicit_reject: bool,

    // ─── Integer timer parameters (ms) — research §3 integerisation ──────
    /// Acknowledgement-timer T2 duration (ms). Default 3000.
    pub t2_ms: u32,
    /// Smoothed Round-Trip Time (ms), §6.7.1.2. Default 3000.
    pub srt_ms: u32,
    /// T1 timeout value (ms), §6.7.1.3 — recomputed as 2×SRT. Default 6000.
    pub t1v_ms: u32,

    /// Named deviations from the SDL figures where a figure is a confirmed
    /// upstream defect. Defaults to spec-correct behaviour.
    pub quirks: Quirks,

    // ─── Queues / stores (alloc) ────────────────────────────────────────
    /// FIFO of I-frame payloads awaiting transmission. One entry pops per
    /// `I_frame_pops_off_queue` event when transmission conditions allow.
    pub i_frame_queue: VecDeque<Payload>,
    /// N(S) → payload for retransmission of previously-sent frames. Populated
    /// on emit; consumed by REJ/SREJ recovery + `Invoke_Retransmission`.
    pub sent_i_frames: BTreeMap<u8, Payload>,
    /// Out-of-sequence received I-frames keyed by N(S), awaiting their turn.
    pub stored_received_i_frames: BTreeMap<u8, Payload>,
    /// N(S) values already selectively retransmitted since V(a) last advanced —
    /// the mod-8 SREJ ring-wrap dedup (packet.net#231/#247). Cleared on V(a)
    /// advance and per-N(S) when a fresh I-frame is emitted at that N(S).
    pub selectively_retransmitted_since_ack: BTreeSet<u8>,
}

impl Default for SessionContext {
    fn default() -> Self {
        Self {
            vs: 0,
            va: 0,
            vr: 0,
            rc: 0,
            own_receiver_busy: false,
            peer_receiver_busy: false,
            acknowledge_pending: false,
            reject_exception: false,
            selective_reject_exception: false,
            srej_exception_count: 0,
            layer3_initiated: false,
            accept_incoming: true,
            x: None,
            t1_had_expired: false,
            t1_remaining_when_last_stopped_ms: 0,
            n1: 256,
            n2: 10,
            k: 4,
            is_extended: false,
            srej_enabled: false,
            segmenter_reassembler_enabled: false,
            half_duplex: true,
            implicit_reject: true,
            t2_ms: 3000,
            srt_ms: 3000,
            t1v_ms: 6000,
            quirks: Quirks::default(),
            i_frame_queue: VecDeque::new(),
            sent_i_frames: BTreeMap::new(),
            stored_received_i_frames: BTreeMap::new(),
            selectively_retransmitted_since_ack: BTreeSet::new(),
        }
    }
}

impl SessionContext {
    /// Construct a context with default link parameters.
    pub fn new() -> Self {
        Self::default()
    }

    /// Modulus used for sequence-variable arithmetic (8 or 128).
    pub fn modulus(&self) -> u16 {
        if self.is_extended {
            128
        } else {
            8
        }
    }

    /// Increment a sequence variable, wrapping at the modulus.
    pub fn increment_seq(&self, value: u8) -> u8 {
        let m = self.modulus();
        (((value as u16) + 1) % m) as u8
    }

    /// Decrement a sequence variable, wrapping at the modulus.
    pub fn decrement_seq(&self, value: u8) -> u8 {
        let m = self.modulus();
        (((value as u16) + m - 1) % m) as u8
    }

    /// True if `ns` is an *outstanding* (sent-but-unacknowledged) send sequence
    /// number — i.e. it lies in the half-open window `[V(a), V(s))` mod modulus.
    /// Replaying a frame outside this window would put a stale N(S) on the wire
    /// the peer can mis-deliver once its V(R) has wrapped (the #231-class bug).
    pub fn is_outstanding(&self, ns: u8) -> bool {
        let m = self.modulus();
        let span = ((self.vs as u16 + m) - self.va as u16) % m;
        let offset = ((ns as u16 + m) - self.va as u16) % m;
        offset < span
    }

    /// Number of outstanding (unacknowledged) I-frames, mod modulus.
    pub fn outstanding_count(&self) -> u16 {
        let m = self.modulus();
        ((self.vs as u16 + m) - self.va as u16) % m
    }

    /// Drop every `sent_i_frames` entry whose N(S) is no longer outstanding
    /// (has been acknowledged — now behind V(a)). Called when V(a) advances so a
    /// stale/duplicate REJ/SREJ cannot replay an already-acked frame. Mirrors
    /// direwolf's `cdata_delete(txdata_by_ns[...])` on acknowledgement.
    pub fn prune_acknowledged_sent_i_frames(&mut self) {
        if self.sent_i_frames.is_empty() {
            return;
        }
        // Collect non-outstanding keys, then remove (avoids borrow conflict).
        let mut to_remove: Vec<u8> = Vec::new();
        for &ns in self.sent_i_frames.keys() {
            if !self.is_outstanding(ns) {
                to_remove.push(ns);
            }
        }
        for ns in to_remove {
            self.sent_i_frames.remove(&ns);
        }
    }

    /// Reset all session state to "freshly connected" defaults (sequence vars,
    /// flags, queues). Leaves negotiated link parameters intact.
    pub fn reset_state(&mut self) {
        self.vs = 0;
        self.va = 0;
        self.vr = 0;
        self.rc = 0;
        self.own_receiver_busy = false;
        self.peer_receiver_busy = false;
        self.acknowledge_pending = false;
        self.reject_exception = false;
        self.selective_reject_exception = false;
        self.srej_exception_count = 0;
        self.layer3_initiated = false;
        self.x = None;
        self.t1_had_expired = false;
        self.t1_remaining_when_last_stopped_ms = 0;
        self.i_frame_queue.clear();
        self.sent_i_frames.clear();
        self.stored_received_i_frames.clear();
        self.selectively_retransmitted_since_ack.clear();
    }
}
