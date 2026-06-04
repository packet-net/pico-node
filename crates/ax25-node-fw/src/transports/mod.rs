//! The node's transports — one module per connectivity capability, mirroring the
//! C# `Packet.Node.Core.Transports` + `Packet.Axudp` / `Packet.Kiss`.
//!
//! Each is an Embassy task that owns its socket/UART, frames bytes with the
//! portable codecs in [`ax25_node_core`], and exchanges AX.25 frames with the
//! [`crate::session`] layer. STUBS — the I/O bodies are filled in once the
//! embassy-net/embassy-rp APIs are available against the pinned versions.

pub mod axudp;
pub mod kiss_serial;
pub mod kiss_tcp;
pub mod telnet;
