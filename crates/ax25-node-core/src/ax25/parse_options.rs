//! Per-call configuration for the AX.25 wire-parse path ([`crate::ax25::Frame`]
//! decode).
//!
//! Ports `Packet.Core.Ax25ParseOptions`. Each pragmatic accommodation beyond
//! strict AX.25 v2.2 compliance is a named, individually-toggleable flag — see
//! `docs/strict-vs-pragmatic-audit.md` in `packet.net` for the inventory.
//!
//! Spec philosophy (mirrors the C# side): the stack is spec-compliant by default,
//! but the parameterless decode entry points ([`Frame::decode`](crate::ax25::Frame::decode))
//! use [`Ax25ParseOptions::LENIENT`] (kitchen-sink accept-everything) to preserve
//! current behaviour. Callers who want strict spec adherence pass
//! [`Ax25ParseOptions::STRICT`]; callers who know their peer pass that peer's named
//! preset ([`Ax25ParseOptions::BPQ`], [`Ax25ParseOptions::XROUTER`],
//! [`Ax25ParseOptions::DIREWOLF`]).
//!
//! When a new real-world quirk is discovered, add a named flag here (defaulted to
//! keep current behaviour), surface it in the preset(s) it belongs to, and update
//! the audit doc. Don't silently widen an existing parser to accept new garbage.
//!
//! `no_std`, allocation-free: a `Copy` record of three flags.

/// Strict-vs-lenient parser choices for the AX.25 wire decode. Every field
/// defaults (via [`Ax25ParseOptions::LENIENT`] / [`Default`]) to preserving the
/// crate's historical accept-everything behaviour.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Ax25ParseOptions {
    /// Accept address slots with an empty callsign base (all six callsign bytes are
    /// `0x40`, i.e. ASCII space shifted left 1).
    ///
    /// Strict §3.12: "The call sign is made up of upper-case alpha and numeric ASCII
    /// characters only" — plural, implying ≥ 1. Driver: BPQ `>IS` ID beacons and
    /// PD4R-12 QRV broadcasts. Mirrors C# `AllowEmptyCallsignBase`.
    pub allow_empty_callsign_base: bool,

    /// Capture trailing bytes as the frame's info on S frames (and on U frames that
    /// §3.5 doesn't permit an info field on).
    ///
    /// Strict §3.5: only I, UI, FRMR, XID and TEST carry information fields; S frames
    /// and SABM/SABME/DISC/UA/DM do not. Pragmatic: sidesteps enumerating which
    /// U-frames legitimately carry info, and tolerates corrupted S frames with
    /// trailing bytes off a noisy RF link. Mirrors C# `AllowInfoOnSupervisoryFrames`.
    pub allow_info_on_supervisory_frames: bool,

    /// Accept a command-only unnumbered frame (SABM / SABME / DISC) whose address
    /// C-bits don't mark it a command.
    ///
    /// Strict §4.3.3.1 / §6.1.2: SABM, SABME and DISC are *always* commands.
    /// Pragmatic: a legacy AX.25 v1.x peer predates the v2.0 command/response C-bit
    /// encoding, so rejecting its connect/disconnect frames by default would break
    /// v1.x interop. Strict drops such a frame at decode (so a bogus-direction SABM
    /// can never open a session). Mirrors C# `AllowCommandFrameAsResponse`.
    pub allow_command_frame_as_response: bool,
}

impl Ax25ParseOptions {
    /// Accept-everything mode (the kitchen sink). All pragmatic flags enabled. Used
    /// by the parameterless decode entry points to preserve historical behaviour.
    /// Mirrors C# `Ax25ParseOptions.Lenient`.
    pub const LENIENT: Self = Self {
        allow_empty_callsign_base: true,
        allow_info_on_supervisory_frames: true,
        allow_command_frame_as_response: true,
    };

    /// Strict AX.25 v2.2 — all pragmatic accommodations disabled. Mirrors C#
    /// `Ax25ParseOptions.Strict`.
    pub const STRICT: Self = Self {
        allow_empty_callsign_base: false,
        allow_info_on_supervisory_frames: false,
        allow_command_frame_as_response: false,
    };

    /// BPQ-flavoured leniency (G8BPQ / LinBPQ). Today the same as [`Self::LENIENT`];
    /// may diverge as BPQ specifics surface. Mirrors C# `Ax25ParseOptions.Bpq`.
    pub const BPQ: Self = Self::LENIENT;

    /// Xrouter-flavoured leniency. Today identical to [`Self::STRICT`] — no
    /// Xrouter-specific quirks observed yet. Mirrors C# `Ax25ParseOptions.Xrouter`.
    pub const XROUTER: Self = Self::STRICT;

    /// Direwolf-as-AX.25-stack leniency. Today identical to [`Self::LENIENT`]; may
    /// diverge. Mirrors C# `Ax25ParseOptions.Direwolf`.
    pub const DIREWOLF: Self = Self::LENIENT;

    /// The lenient preset (accept-everything). See [`Self::LENIENT`].
    pub const fn lenient() -> Self {
        Self::LENIENT
    }

    /// The strict preset (spec-compliant). See [`Self::STRICT`].
    pub const fn strict() -> Self {
        Self::STRICT
    }
}

impl Default for Ax25ParseOptions {
    /// Lenient — matches the C# `TryParse` parameterless default.
    fn default() -> Self {
        Self::LENIENT
    }
}
