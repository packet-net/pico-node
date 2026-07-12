//! NET/ROM L4 transport — the virtual-circuit layer (the connected-mode,
//! sliding-window end-to-end transport that rides above the AX.25 interlink).
//!
//! Mirrors the C# `Packet.NetRom.Transport` namespace / the TypeScript
//! `circuit*.ts` modules. Hand-written (NET/ROM has no SDL figures; BPQ is the
//! de-facto reference). `no_std`; the dynamic per-circuit buffers use `alloc`
//! (the firmware provides a heap sized for a few links with a small window).

pub mod circuit;
pub mod circuit_manager;
pub mod circuit_options;
pub mod circuit_state;
/// The NET/ROM L4 payload (de)compression codec (zlib / RFC 1950 + DEFLATE /
/// RFC 1951), for BPQ `L4Compress` interop. Gated behind the `netrom-compress`
/// cargo feature so the default on-target build carries no compression code.
#[cfg(feature = "netrom-compress")]
pub mod deflate;
pub mod inp3_engine;
pub mod inp3_update_scheduler;

pub use circuit::{CircuitEvent, NetRomCircuit, OutboundPacket};
pub use circuit_manager::{CircuitKey, CircuitManager, IncomingCircuit};
pub use circuit_options::NetRomCircuitOptions;
pub use circuit_state::{NetRomCircuitCloseReason, NetRomCircuitState};
