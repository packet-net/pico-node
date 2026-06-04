//! Runtime events + the typed mapping onto `ax25sdl::Ax25Event`.
//!
//! Ports the `Ax25Event` record hierarchy + `Ax25Session.ToSdlEvent` from
//! `Packet.Ax25.Session`. The runtime is fed [`Event`]s; each carries any
//! attached frame fields ([`FrameInfo`]) or payload the guards / dispatcher read.
//! [`Event::to_sdl`] maps each to the typed [`ax25sdl::Ax25Event`] the codegen's
//! `TransitionSpec::on` carries — a pure enum compare, no string dispatch.

extern crate alloc;
use alloc::vec::Vec;

use ax25sdl::Ax25Event as Sdl;

/// The mode-aware fields of a received frame the guards + dispatcher read.
///
/// Mirrors the subset of `Packet.Ax25.Ax25Frame` the C# session bindings touch:
/// N(S)/N(R) already resolved at the link's negotiated modulus (3-bit mod-8 /
/// 7-bit mod-128), the P/F bit, command/response classification, and the info +
/// PID. The transport layer parses the wire frame and fills this in before
/// posting the event (it knows the session's modulus; the raw octets don't).
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct FrameInfo {
    /// Receive sequence number N(R) (mode-aware). Meaningful on I and S frames.
    pub nr: u8,
    /// Send sequence number N(S) (mode-aware). Meaningful on I frames only.
    pub ns: u8,
    /// The poll/final bit.
    pub poll_final: bool,
    /// Command frame (dest C-bit set, source C-bit clear, §6.1.2).
    pub is_command: bool,
    /// Information field (empty if absent).
    pub info: Vec<u8>,
    /// PID octet — present on I and UI frames.
    pub pid: Option<u8>,
}

impl FrameInfo {
    /// Response per §6.1.2 — the logical complement of [`is_command`](Self::is_command)
    /// for a well-formed frame. (A frame with neither/both C-bits set is treated
    /// as not-a-response; the C# binding reads the source C-bit directly, which
    /// for our parsed `is_command` is its negation on valid frames.)
    pub fn is_response(&self) -> bool {
        !self.is_command
    }
}

/// A runtime event posted into a [`super::session::Session`].
///
/// The frame-receipt variants carry a [`FrameInfo`]; the upper-layer data
/// primitives carry a payload + PID; timer/internal/catch-all events carry
/// nothing. The discriminant maps 1:1 onto [`ax25sdl::Ax25Event`] via
/// [`Event::to_sdl`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Event {
    // ─── Upper-layer (Layer-3 → Data-Link) primitives ──────────────────
    /// DL-CONNECT request.
    DlConnectRequest,
    /// DL-DISCONNECT request.
    DlDisconnectRequest,
    /// DL-DATA request — carries the payload to enqueue + send.
    DlDataRequest(u8, Vec<u8>),
    /// DL-UNIT-DATA request — carries the UI payload.
    DlUnitDataRequest(u8, Vec<u8>),
    /// DL-FLOW-OFF request.
    DlFlowOffRequest,
    /// DL-FLOW-ON request.
    DlFlowOnRequest,

    // ─── Frame-received events ──────────────────────────────────────────
    /// Information (I) frame received.
    IReceived(FrameInfo),
    /// RR received.
    RrReceived(FrameInfo),
    /// RNR received.
    RnrReceived(FrameInfo),
    /// REJ received.
    RejReceived(FrameInfo),
    /// SREJ received.
    SrejReceived(FrameInfo),
    /// UI received.
    UiReceived(FrameInfo),
    /// SABM received.
    SabmReceived(FrameInfo),
    /// SABME received.
    SabmeReceived(FrameInfo),
    /// DISC received.
    DiscReceived(FrameInfo),
    /// UA received.
    UaReceived(FrameInfo),
    /// DM received.
    DmReceived(FrameInfo),
    /// FRMR received.
    FrmrReceived(FrameInfo),
    /// XID command received.
    XidReceived(FrameInfo),
    /// XID response received.
    XidResponseReceived(FrameInfo),
    /// TEST received.
    TestReceived(FrameInfo),

    // ─── Internal + catch-all events ────────────────────────────────────
    /// Synthetic event: an I-frame popped off the transmit queue (carries the
    /// payload to put on the wire).
    IFramePopsOffQueue(u8, Vec<u8>),
    /// Any other command frame (figc4.x catch-all column).
    AllOtherCommands(FrameInfo),
    /// Any other primitive from the lower layer.
    AllOtherPrimitivesFromLowerLayer,
    /// Any other primitive from the upper layer.
    AllOtherPrimitivesFromUpperLayer,
    /// Control-field error (unrecognised control octet).
    ControlFieldError,
    /// Information field not permitted in this frame type.
    InfoNotPermittedInFrame,
    /// U/S frame length error.
    UOrSFrameLengthError,

    // ─── Timer expiries ─────────────────────────────────────────────────
    /// T1 (acknowledgement timer) expiry.
    T1Expiry,
    /// T2 (response-delay timer) expiry.
    T2Expiry,
    /// T3 (inactive-link timer) expiry.
    T3Expiry,
}

impl Event {
    /// Map this runtime event onto the typed [`ax25sdl::Ax25Event`] the codegen's
    /// `TransitionSpec::on` carries. Exhaustive over the runtime vocabulary; a
    /// pure enum compare drives dispatch (mirrors `Ax25Session.ToSdlEvent`).
    pub fn to_sdl(&self) -> Sdl {
        match self {
            Event::DlConnectRequest => Sdl::DLCONNECTRequest,
            Event::DlDisconnectRequest => Sdl::DLDISCONNECTRequest,
            Event::DlDataRequest(..) => Sdl::DLDATARequest,
            Event::DlUnitDataRequest(..) => Sdl::DLUNITDATARequest,
            Event::DlFlowOffRequest => Sdl::DLFLOWOFFRequest,
            Event::DlFlowOnRequest => Sdl::DLFLOWONRequest,

            Event::IReceived(_) => Sdl::IReceived,
            Event::RrReceived(_) => Sdl::RRReceived,
            Event::RnrReceived(_) => Sdl::RNRReceived,
            Event::RejReceived(_) => Sdl::REJReceived,
            Event::SrejReceived(_) => Sdl::SREJReceived,
            Event::UiReceived(_) => Sdl::UIReceived,
            Event::SabmReceived(_) => Sdl::SABMReceived,
            Event::SabmeReceived(_) => Sdl::SABMEReceived,
            Event::DiscReceived(_) => Sdl::DISCReceived,
            Event::UaReceived(_) => Sdl::UAReceived,
            Event::DmReceived(_) => Sdl::DMReceived,
            Event::FrmrReceived(_) => Sdl::FRMRReceived,
            Event::XidReceived(_) => Sdl::XIDReceived,
            Event::XidResponseReceived(_) => Sdl::XIDResponseReceived,
            Event::TestReceived(_) => Sdl::TESTReceived,

            Event::IFramePopsOffQueue(..) => Sdl::IFramePopsOffQueue,
            Event::AllOtherCommands(_) => Sdl::AllOtherCommands,
            Event::AllOtherPrimitivesFromLowerLayer => Sdl::AllOtherPrimitivesFromLowerLayer,
            Event::AllOtherPrimitivesFromUpperLayer => Sdl::AllOtherPrimitivesFromUpperLayer,
            Event::ControlFieldError => Sdl::ControlFieldError,
            Event::InfoNotPermittedInFrame => Sdl::InfoNotPermittedInFrame,
            Event::UOrSFrameLengthError => Sdl::UOrSFrameLengthError,

            Event::T1Expiry => Sdl::T1Expiry,
            Event::T2Expiry => Sdl::T2Expiry,
            Event::T3Expiry => Sdl::T3Expiry,
        }
    }

    /// The attached [`FrameInfo`] for frame-receipt events, else `None`. Mirrors
    /// `Ax25SessionBindings.GetIncomingFrame`.
    pub fn frame(&self) -> Option<&FrameInfo> {
        match self {
            Event::IReceived(f)
            | Event::RrReceived(f)
            | Event::RnrReceived(f)
            | Event::RejReceived(f)
            | Event::SrejReceived(f)
            | Event::UiReceived(f)
            | Event::SabmReceived(f)
            | Event::SabmeReceived(f)
            | Event::DiscReceived(f)
            | Event::UaReceived(f)
            | Event::DmReceived(f)
            | Event::FrmrReceived(f)
            | Event::XidReceived(f)
            | Event::XidResponseReceived(f)
            | Event::TestReceived(f)
            | Event::AllOtherCommands(f) => Some(f),
            _ => None,
        }
    }
}
