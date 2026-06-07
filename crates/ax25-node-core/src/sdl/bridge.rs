//! The spec→wire adapter: turn the runtime's [`FrameSpec`] intents into encoded
//! AX.25 frames, and turn received wire frames into runtime [`Event`]s.
//!
//! This is the seam between the portable [`super::session::Session`] (which speaks
//! [`FrameSpec`] / [`Event`], not bytes) and the byte-level [`crate::ax25`] codec
//! the transports move on the wire. The firmware's `SessionSink` is a thin wrapper
//! around [`WireSink`] that, after [`WireSink`] builds the [`crate::ax25::Frame`],
//! hands its `encode()`d bytes to the owning transport; host tests use [`WireSink`]
//! directly to assert on the exact octets.
//!
//! ## Modulo
//!
//! Control-octet construction here is **modulo-8** (AX.25 v2.2 §4.3.3) — the one-
//! octet control field. The node is mod-128-*capable* at the session level (the
//! runtime tracks `is_extended`, the establishment paths negotiate it), but the
//! two-octet extended control framing is a documented follow-up in
//! [`crate::ax25::frame`] (the extended frame format the codec doesn't model yet).
//! [`WireSink::new`] therefore takes the link's local/remote callsigns + the
//! digipeater path and builds mod-8 frames; an extended-mode build asserts.
//!
//! `no_std` + `alloc`.

extern crate alloc;
use alloc::vec::Vec;

use crate::ax25::{Address, Callsign, Frame, PID_NO_LAYER3};

use super::event::{Event, FrameInfo};
use super::signal::{
    DataLinkSignal, FrameSpec, InternalSignal, LinkMultiplexerSignal, SessionSink, SupervisoryKind,
    UnnumberedKind,
};

// ─── AX.25 v2.2 §4.3.3 control-octet encodings (modulo-8) ───────────────────

/// U-frame control bits (P/F bit, 0x10, ORed in separately).
mod uctl {
    /// SABME — 0110_1111.
    pub const SABME: u8 = 0x6F;
    /// SABM — 0010_1111.
    pub const SABM: u8 = 0x2F;
    /// DISC — 0100_0011.
    pub const DISC: u8 = 0x43;
    /// DM — 0000_1111.
    pub const DM: u8 = 0x0F;
    /// UA — 0110_0011.
    pub const UA: u8 = 0x63;
}

/// S-frame control low nibble (the N(R) goes in the high 3 bits, mod-8; P/F is
/// 0x10).
mod sctl {
    /// RR — ..00_0001.
    pub const RR: u8 = 0x01;
    /// RNR — ..00_0101.
    pub const RNR: u8 = 0x05;
    /// REJ — ..00_1001.
    pub const REJ: u8 = 0x09;
    /// SREJ — ..00_1101.
    pub const SREJ: u8 = 0x0D;
}

/// The P/F bit position in a mod-8 control octet.
const PF_BIT: u8 = 0x10;

/// Builds wire frames from [`FrameSpec`]s for one link, and accumulates them.
/// The firmware wraps this; tests read [`WireSink::sent`] to assert on octets.
#[derive(Debug, Clone)]
pub struct WireSink {
    local: Callsign,
    remote: Callsign,
    digipeaters: Vec<Callsign>,
    /// Encoded frames produced, in emission order.
    pub sent: Vec<Vec<u8>>,
    /// DL signals raised upward, in order.
    pub upward: Vec<DataLinkSignal>,
    /// The SDL asked the link multiplexer for the channel (`LMSeizeRequest`)
    /// and awaits an [`super::Event::LmSeizeConfirm`]. Set by `send_link_mux`,
    /// cleared by the driver when it grants the seize.
    pub seize_pending: bool,
}

impl WireSink {
    /// A sink for the `local ↔ remote` link with an optional digipeater path.
    pub fn new(local: Callsign, remote: Callsign, digipeaters: Vec<Callsign>) -> Self {
        Self {
            local,
            remote,
            digipeaters,
            sent: Vec::new(),
            upward: Vec::new(),
            seize_pending: false,
        }
    }

    /// Build the [`Frame`] for a [`FrameSpec`] (modulo-8). Public so the firmware
    /// can encode without going through the accumulating sink.
    pub fn build_frame(&self, spec: &FrameSpec) -> Frame {
        match spec {
            FrameSpec::Supervisory {
                kind,
                is_command,
                nr,
                pf,
            } => {
                let base = match kind {
                    SupervisoryKind::Rr => sctl::RR,
                    SupervisoryKind::Rnr => sctl::RNR,
                    SupervisoryKind::Rej => sctl::REJ,
                    SupervisoryKind::Srej => sctl::SREJ,
                };
                let control = base | ((nr & 0x07) << 5) | if *pf { PF_BIT } else { 0 };
                self.frame(*is_command, control, None, Vec::new())
            }
            FrameSpec::Unnumbered {
                kind,
                is_command,
                pf,
                ..
            } => {
                let base = match kind {
                    UnnumberedKind::Sabm => uctl::SABM,
                    UnnumberedKind::Sabme => uctl::SABME,
                    UnnumberedKind::Disc => uctl::DISC,
                    UnnumberedKind::Ua => uctl::UA,
                    UnnumberedKind::Dm => uctl::DM,
                };
                let control = base | if *pf { PF_BIT } else { 0 };
                self.frame(*is_command, control, None, Vec::new())
            }
            FrameSpec::Ui {
                is_command,
                pf,
                pid,
                info,
            } => {
                let control = crate::ax25::frame::CONTROL_UI | if *pf { PF_BIT } else { 0 };
                self.frame(*is_command, control, Some(*pid), info.clone())
            }
            FrameSpec::Information {
                p,
                nr,
                ns,
                pid,
                info,
            } => {
                // I-frame (mod-8): bit0=0; N(S) in bits 3..1; P in bit4; N(R) in 7..5.
                let control = ((nr & 0x07) << 5) | if *p { PF_BIT } else { 0 } | ((ns & 0x07) << 1);
                self.frame(true, control, Some(*pid), info.clone())
            }
        }
    }

    /// Construct the addressed frame (we are the source; the peer the destination).
    fn frame(&self, is_command: bool, control: u8, pid: Option<u8>, info: Vec<u8>) -> Frame {
        // §6.1.2 command/response: command ⇒ dest C-bit set, source C-bit clear.
        let (dest_crh, src_crh) = if is_command {
            (true, false)
        } else {
            (false, true)
        };
        let digipeaters = self
            .digipeaters
            .iter()
            .map(|c| Address {
                callsign: *c,
                crh: false,
                extension: false,
            })
            .collect();
        Frame {
            destination: Address {
                callsign: self.remote,
                crh: dest_crh,
                extension: false,
            },
            source: Address {
                callsign: self.local,
                crh: src_crh,
                extension: false,
            },
            digipeaters,
            control,
            pid,
            info,
        }
    }
}

impl SessionSink for WireSink {
    fn send_frame(&mut self, spec: FrameSpec) {
        let frame = self.build_frame(&spec);
        self.sent.push(frame.encode());
    }
    fn send_upward(&mut self, signal: DataLinkSignal) {
        self.upward.push(signal);
    }
    fn send_link_mux(&mut self, signal: LinkMultiplexerSignal) {
        // Record seize requests so the driver can grant them (immediately, on a
        // full-duplex wire transport) by posting [`Event::LmSeizeConfirm`] —
        // the figc4 delayed-ack path depends on the confirm coming back.
        if signal == LinkMultiplexerSignal::SeizeRequest {
            self.seize_pending = true;
        }
    }
    fn send_internal(&mut self, _signal: InternalSignal) {}
}

/// Classify a received (modulo-8) [`Frame`] into the runtime [`Event`] that should
/// be posted for it, extracting the mode-aware [`FrameInfo`]. Returns `None` for a
/// frame whose control octet doesn't map to a known event (the caller can post
/// [`Event::ControlFieldError`]). This is the inbound counterpart of
/// [`WireSink::build_frame`] — the firmware transports call it on each decoded
/// wire frame before handing the event to the session.
pub fn classify_incoming(frame: &Frame) -> Option<Event> {
    let control = frame.control;
    let poll_final = (control & PF_BIT) != 0;
    let is_command = frame.is_command();
    let nr = (control >> 5) & 0x07;
    let ns = (control >> 1) & 0x07;
    let info = FrameInfo {
        nr,
        ns,
        poll_final,
        is_command,
        info: frame.info.clone(),
        pid: frame.pid,
    };

    if (control & 0x01) == 0 {
        // I frame (bit0 = 0).
        return Some(Event::IReceived(info));
    }
    if (control & 0x03) == 0x01 {
        // S frame — low nibble selects the type.
        return Some(match control & 0x0F {
            sctl::RR => Event::RrReceived(info),
            sctl::RNR => Event::RnrReceived(info),
            sctl::REJ => Event::RejReceived(info),
            sctl::SREJ => Event::SrejReceived(info),
            _ => return None,
        });
    }
    // U frame — mask off the P/F bit to compare the type bits.
    let u = control & !PF_BIT;
    Some(match u {
        uctl::SABM => Event::SabmReceived(info),
        uctl::SABME => Event::SabmeReceived(info),
        uctl::DISC => Event::DiscReceived(info),
        uctl::UA => Event::UaReceived(info),
        uctl::DM => Event::DmReceived(info),
        c if (c & 0xEF) == crate::ax25::frame::CONTROL_UI => Event::UiReceived(info),
        _ => return None,
    })
}

/// PID used when an outbound spec carries none (UI/I always carry one in practice,
/// but a defensive default keeps the encoder total).
pub const DEFAULT_PID: u8 = PID_NO_LAYER3;
