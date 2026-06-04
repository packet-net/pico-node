//! Node command-prompt layer — the telnet/AX.25 console.
//!
//! Ports `Packet.Node.Core.Console`. The command logic is deliberately split from
//! the I/O so it is pure and host-testable:
//!
//! - [`line::LineAssembler`] — reassembles a byte stream into bounded lines
//!   (`LineAssembler`).
//! - [`command::Command`] + [`command::parse`] — the closed typed command set and
//!   the total parser (`NodeCommand` + `NodeCommandParser`).
//! - [`service`] — the transport-agnostic prompt-loop *responses*: given a parsed
//!   command + node identity, produce the bytes to write and whether to
//!   disconnect (`NodeCommandService.DispatchAsync` text builders), with the
//!   per-transport newline convention (CR for AX.25, CR-LF for telnet).
//!
//! The async [`NodeConnection`] trait is the analogue of `INodeConnection`: the
//! firmware implements it over a telnet TCP socket and over an AX.25 session; the
//! prompt loop runs over the trait only, so it never depends on the transport.
//! The loop's `RunAsync` orchestration lives in the firmware crate (it needs the
//! async runtime); everything it decides is computed by the pure helpers here.

pub mod command;
pub mod connection;
pub mod line;
pub mod service;

pub use command::{parse, Command};
pub use connection::{NodeConnection, TransportKind};
pub use line::LineAssembler;
pub use service::{render_line, DispatchOutcome, Identity, Response};
