//! # ax25-node-core
//!
//! The portable, hardware-independent core of the RP2040 / Pico W AX.25 packet
//! node. It mirrors, in Rust, the proven module structure of the C# node host in
//! `m0lte/packet.net` (`Packet.Kiss`, `Packet.Axudp`, `Packet.Ax25`,
//! `Packet.Node.Core.Console`) so the two stacks stay conceptually aligned.
//!
//! ## no_std posture
//!
//! This crate is `#![no_std]` unless the (default) `std` feature is on. Host unit
//! tests and the host simulator build with `std`; the on-target firmware builds
//! with `--no-default-features` (then only `core` + `alloc` are used). A handful
//! of streaming buffers want a growable container — those use `alloc::vec::Vec`
//! behind the `alloc` feature. See each module's docs for the heapless follow-up.
//!
//! ## What lives here (host-testable today, zero external deps)
//!
//! - [`crc`] — CRC-16/X.25 (the AX.25 FCS), ported from `Packet.Core.Crc16Ccitt`.
//! - [`kiss`] — KISS SLIP framing: encoder + streaming decoder (`Packet.Kiss`).
//! - [`axudp`] — AXUDP framing helpers (the UDP payload *is* the AX.25 body) with optional CRC FCS, ported from `Packet.Axudp.AxudpSocket`.
//! - [`ax25`] — AX.25 address + frame codec essentials (`Packet.Ax25.Ax25Frame`, `Packet.Core.Ax25Address`, `Packet.Core.Callsign`).
//! - [`console`] — the node command-prompt layer: line assembler, command parser, and the transport-agnostic prompt loop (`Packet.Node.Core.Console`).
//! - [`sdl`] — the connected-mode AX.25 link-layer runtime: the SDL state machine driven off the generated `m0lte/ax25sdl` typed tables (the Rust port of packet.net's `Ax25Session`).
//! - [`netrom`] — the read-only "NET/ROM aware" slice: parse NODES broadcasts, build a routing table, surface it (the Rust port of `Packet.NetRom` + `Packet.Node.Core.NetRom.NetRomService`). Hears NODES, originates nothing.
//!
//! The I/O-bound parts (sockets, the WiFi radio, the UART) live in the firmware
//! crate `ax25-node-fw`; this crate is pure logic over bytes so it can be tested
//! on the host with `cargo test`.

#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]
#![warn(missing_docs)]

#[cfg(feature = "alloc")]
extern crate alloc;

pub mod ax25;
pub mod axudp;
pub mod console;
pub mod crc;
pub mod kiss;
pub mod netrom;
pub mod radio;
pub mod sdl;
pub mod tune;

/// The node software version, surfaced by the `Info` console command (mirrors
/// `NodeCommandService.Version` in the C# host).
pub const VERSION: &str = env!("CARGO_PKG_VERSION");
