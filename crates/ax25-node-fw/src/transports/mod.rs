//! The node's transports — one module per connectivity capability, mirroring the
//! C# `Packet.Node.Core.Transports` + `Packet.Axudp` / `Packet.Kiss`.
//!
//! Each is an Embassy task that owns its socket/UART, frames bytes with the
//! portable codecs in [`ax25_node_core`], and exchanges AX.25 frames with the
//! [`crate::session`] layer.

pub mod axudp;
pub mod telnet;
// GATE 5–6 (HW-BRINGUP.md §4): the KISS transports return gate by gate —
// kiss_tcp (Gate 5), kiss_serial (Gate 6; its UART generics also don't compile
// against embassy-rp 0.10 yet).
// pub mod kiss_serial;
// pub mod kiss_tcp;
