//! AX.25 address + frame codec essentials.
//!
//! Ports the bytes-only parts of `Packet.Core.Callsign`, `Packet.Core.Ax25Address`,
//! and `Packet.Ax25.Ax25Frame` from `m0lte/packet.net`. This is the layer the
//! AXUDP and KISS transports hand frames to, and the layer that will sit beneath
//! the SDL link-layer runtime (see [`crate::sdl`]).
//!
//! Scope note (mirrors the C# Phase-1 codec): destination + source + digipeater
//! address fields, the control octet (modulo-8; modulo-128 extension recognised),
//! PID, and the info field. Full mod-128 N(S)/N(R) extraction and the supervisory
//! sub-typing live with the runtime port and are stubbed here as the codec grows.

pub mod address;
pub mod callsign;
pub mod frame;

pub use address::{Address, ADDRESS_LEN};
pub use callsign::Callsign;
pub use frame::{Frame, ParseError, PID_NETROM, PID_NO_LAYER3, PID_SEGMENTED};
