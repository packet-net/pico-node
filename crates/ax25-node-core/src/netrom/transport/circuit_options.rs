//! The tunable knobs of the NET/ROM L4 transport (the circuit layer).
//!
//! NET/ROM has no single normative standard for these — the timers and window
//! come from the de-facto reference (BPQ's `L4*` knobs / the Linux `transport_*`
//! tunables). Durations are in milliseconds, read against the caller-supplied
//! `now_ms` clock (no wall-clock in the circuit layer). Mirrors
//! `Packet.NetRom.Transport.NetRomCircuitOptions`; the TS port's
//! partial-override-interface-plus-resolver becomes the Rust `Default` + struct
//! update idiom (`NetRomCircuitOptions { window_size: 8, ..Default::default() }`).

use crate::netrom::wire::{DEFAULT_TIME_TO_LIVE, MAX_PAYLOAD};

/// The fully-specified circuit tunables.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NetRomCircuitOptions {
    /// The send-window size this node proposes in a Connect Request and the
    /// maximum it accepts in a Connect Acknowledge (BPQ `L4WINDOW`, default 4; the
    /// 8-bit sequence space allows up to 127).
    pub window_size: u8,
    /// The retransmit timeout in ms (BPQ `L4TIMEOUT` / Linux `transport_timeout`):
    /// how long to wait for an ack before retransmitting the oldest unacknowledged
    /// Information message. Default 5000.
    pub retransmit_timeout_ms: u64,
    /// Maximum retransmit attempts for a Connect / Disconnect / Information message
    /// before the circuit is declared failed (BPQ `L4RETRIES`). Default 3.
    pub max_retries: u8,
    /// The initial TTL stamped into the L3 network header of datagrams this circuit
    /// originates ([`DEFAULT_TIME_TO_LIVE`]).
    pub time_to_live: u8,
    /// Maximum bytes of user data per Information datagram — the fragment size. A
    /// larger logical send is split with the more-follows flag. Default
    /// [`MAX_PAYLOAD`] (236).
    pub fragment_size: usize,
    /// The queued-but-undelivered received-message count at which this node asserts
    /// *choke*. Default 0 — the receiver never self-chokes (it drains promptly); a
    /// host that can stall its reader sets this so backpressure reaches the wire.
    pub choke_threshold: usize,
    /// Offer (and accept) LinBPQ-style negotiated NET/ROM L4 payload compression on
    /// circuits this node originates or accepts (BPQ `L4Compress`). **Default
    /// `false`** (decline) — a circuit then runs uncompressed, which every NET/ROM
    /// peer can read. When `true`, the circuit advertises compression in its Connect
    /// Request / Acknowledge and only actually compresses outbound data when the
    /// *other end* also agreed. Gated behind `netrom-compress`. Mirrors C#
    /// `NetRomCircuitOptions.CompressionEnabled`.
    #[cfg(feature = "netrom-compress")]
    pub compression_enabled: bool,
    /// The proposed session timer (T1, whole seconds) carried in the trailing 2
    /// octets of a LinBPQ extended Connect Request — the carrier for the
    /// compression-supported bit. Only emitted when [`compression_enabled`] is set.
    /// Default 60 s; the high nibble is reserved for the compress flag so the value
    /// is masked to the low 12 bits on the wire. Gated behind `netrom-compress`.
    /// Mirrors C# `NetRomCircuitOptions.ProposedTimerSeconds`.
    ///
    /// [`compression_enabled`]: Self::compression_enabled
    #[cfg(feature = "netrom-compress")]
    pub proposed_timer_seconds: u16,
}

impl Default for NetRomCircuitOptions {
    fn default() -> Self {
        Self {
            window_size: 4,
            retransmit_timeout_ms: 5000,
            max_retries: 3,
            time_to_live: DEFAULT_TIME_TO_LIVE,
            fragment_size: MAX_PAYLOAD,
            choke_threshold: 0,
            #[cfg(feature = "netrom-compress")]
            compression_enabled: false,
            #[cfg(feature = "netrom-compress")]
            proposed_timer_seconds: 60,
        }
    }
}
