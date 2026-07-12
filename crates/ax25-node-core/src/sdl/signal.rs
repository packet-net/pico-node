//! Outbound signals the runtime emits, and the [`SessionSink`] that receives them.
//!
//! Ports the family of `send*` callbacks the C# `ActionDispatcher` is constructed
//! with (`sendSFrame`, `sendUFrame`, `sendUiFrame`, `sendIFrame`, `sendUpward`,
//! `sendLinkMux`, `sendInternal`) into one trait the embedding implements. The
//! firmware's `SessionSink` translates a [`FrameSpec`] into wire octets (via the
//! `ax25` codec) and hands it to the owning transport, and surfaces a
//! [`DataLinkSignal`] to the upper layer (the telnet console / app).
//!
//! Frame *specs* are intent ("send an RR response, N(R)=3, F=1"), not wire bytes —
//! exactly like the C# `SupervisoryFrameSpec` / `UFrameSpec` / `IFrameSpec` /
//! `UiFrameSpec`. Keeping the runtime at the spec level keeps it portable and
//! host-testable; the wire translation lives in the firmware. `no_std` + `alloc`.

extern crate alloc;
use alloc::vec::Vec;

/// A supervisory frame type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SupervisoryKind {
    /// Receive Ready.
    Rr,
    /// Receive Not Ready.
    Rnr,
    /// Reject.
    Rej,
    /// Selective Reject.
    Srej,
}

/// An unnumbered frame type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UnnumberedKind {
    /// Set Asynchronous Balanced Mode (mod-8 connect).
    Sabm,
    /// SABM Extended (mod-128 connect).
    Sabme,
    /// Disconnect.
    Disc,
    /// Unnumbered Acknowledge.
    Ua,
    /// Disconnected Mode.
    Dm,
}

/// An outgoing frame the runtime asks the sink to put on the wire. One spec per
/// `signal_lower` verb. Mirrors the four C# `*FrameSpec` records, unified.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FrameSpec {
    /// A supervisory frame (RR/RNR/REJ/SREJ) with N(R) and the P/F bit.
    Supervisory {
        /// Which S-frame.
        kind: SupervisoryKind,
        /// Command (true) vs response (false).
        is_command: bool,
        /// Receive sequence number to carry.
        nr: u8,
        /// Poll/final bit.
        pf: bool,
    },
    /// An unnumbered frame (SABM/SABME/DISC/UA/DM) with the P/F bit.
    Unnumbered {
        /// Which U-frame.
        kind: UnnumberedKind,
        /// Command (true) vs response (false).
        is_command: bool,
        /// Poll/final bit.
        pf: bool,
        /// Hint that the frame should jump the TX queue (figc4.3 Expedited UA/DM).
        expedited: bool,
    },
    /// A UI frame carrying connectionless data.
    Ui {
        /// Command (true) vs response (false).
        is_command: bool,
        /// Poll/final bit.
        pf: bool,
        /// PID octet.
        pid: u8,
        /// Information field.
        info: Vec<u8>,
    },
    /// An information (I) frame.
    Information {
        /// Poll bit.
        p: bool,
        /// Receive sequence number N(R).
        nr: u8,
        /// Send sequence number N(S).
        ns: u8,
        /// PID octet.
        pid: u8,
        /// Information field.
        info: Vec<u8>,
    },
    /// An XID (Exchange Identification) frame — the §4.3.3.7 parameter-negotiation
    /// U-frame. Carries no PID; the info field is the encoded XID parameters (see
    /// [`crate::ax25::xid`]). Emitted by the management data-link responder.
    Xid {
        /// Command (true) vs response (false).
        is_command: bool,
        /// Poll/final bit.
        pf: bool,
        /// Information field — the encoded XID parameter TLVs.
        info: Vec<u8>,
    },
}

/// A signal raised to Layer 3 (the upper-layer service-access point). Ports the
/// `DataLinkSignal` record hierarchy. The error indication carries the §C5
/// letter code as drawn on the figure.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DataLinkSignal {
    /// DL-CONNECT indication (inbound connection established).
    ConnectIndication,
    /// DL-CONNECT confirm (our connect request succeeded).
    ConnectConfirm,
    /// DL-DISCONNECT indication (peer/link tore the connection down).
    DisconnectIndication,
    /// DL-DISCONNECT confirm (our disconnect request completed).
    DisconnectConfirm,
    /// DL-DATA indication — delivered Layer-3 data (PID + info).
    DataIndication(u8, Vec<u8>),
    /// DL-UNIT-DATA indication — delivered connectionless (UI) data.
    UnitDataIndication(u8, Vec<u8>),
    /// DL-ERROR indication — the §C5 error-code letter.
    ErrorIndication(&'static str),
}

/// A signal to the link multiplexer (medium-access arbiter). Ports
/// `LinkMultiplexerSignal`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LinkMultiplexerSignal {
    /// LM-SEIZE request.
    SeizeRequest,
    /// LM-RELEASE request.
    ReleaseRequest,
    /// LM-DATA request.
    DataRequest,
}

/// An internal-out signal — to the management data-link or the internal I-frame
/// queue. Ports the `InternalSignal` family the data-link machine raises.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum InternalSignal {
    /// MDL-NEGOTIATE request (start XID negotiation).
    MdlNegotiateRequest,
    /// An I-frame payload was pushed onto the transmit queue.
    PushIFrameQueue(Vec<u8>),
}

/// The sink the runtime emits through. The embedding (firmware, or a host test
/// recorder) implements this to translate [`FrameSpec`]s into wire frames on the
/// owning transport and to surface [`DataLinkSignal`]s upward.
///
/// All methods default to no-ops so a sink only overrides what it cares about
/// (mirroring the C# dispatcher's no-op default callbacks).
pub trait SessionSink {
    /// Put a frame on the wire.
    fn send_frame(&mut self, _spec: FrameSpec) {}
    /// Raise a signal to Layer 3.
    fn send_upward(&mut self, _signal: DataLinkSignal) {}
    /// Raise a signal to the link multiplexer.
    fn send_link_mux(&mut self, _signal: LinkMultiplexerSignal) {}
    /// Raise an internal-out signal.
    fn send_internal(&mut self, _signal: InternalSignal) {}
}

/// A `SessionSink` that drops every signal. Useful for tests that only assert on
/// context / state transitions, and as the firmware's placeholder before the
/// transports are wired.
#[derive(Debug, Default, Clone, Copy)]
pub struct NullSink;
impl SessionSink for NullSink {}
