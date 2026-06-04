//! SDL link-layer runtime — the connected-mode AX.25 state machine, driven off the
//! generated [`ax25sdl`] typed tables.
//!
//! ## What this is
//!
//! The headline differentiator of this project (per
//! `docs/research/pico-packet-node.md`) is running the *same* generated AX.25 v2.2
//! SDL state machine that the C# (`packet.net`) and TypeScript (`ax25-ts`) stacks
//! run — from one spec source — proving link-layer parity across hardware classes.
//! `m0lte/ax25sdl` (`spec/rust`, the `ax25sdl` crate, 0.8.0) emits the ~243 v2.2
//! transitions + the figc4.7 subroutines as `&'static` [`ax25sdl::StatePage`] /
//! [`ax25sdl::SubroutinesPage`] data, plus SP-010's typed closed sets
//! ([`ax25sdl::Ax25Event`] / [`ax25sdl::Ax25Guard`] / [`ax25sdl::Ax25ActionVerb`]).
//! This module is the **runtime that walks them** — the Rust port of the C#
//! `Ax25Session` + `ActionDispatcher` + `GuardEvaluator` + `SubroutineRegistry` +
//! `SdlLoopExecutor`, consuming those tables via a clean exhaustive `match` (no
//! string dispatch).
//!
//! ## Layout (mirrors the C# `Packet.Ax25.Session` module boundaries)
//!
//! - [`context`] — [`SessionContext`]: sequence vars, flags, queues, params.
//! - [`event`] — the runtime [`Event`] vocabulary + the typed mapping onto [`ax25sdl::Ax25Event`].
//! - [`signal`] — the outbound [`signal::FrameSpec`] / [`signal::DataLinkSignal`] + the [`signal::SessionSink`] the embedding implements.
//! - [`timer`] — the [`timer::TimerService`] contract + the integerised SRT/T1V math (research §3: no FPU on the M0+).
//! - [`guard`] — typed-`match` guard evaluation over [`ax25sdl::Ax25Guard`].
//! - [`dispatch`] — typed-`match` action dispatch over [`ax25sdl::Ax25ActionVerb`].
//! - [`subroutine`] — the figc4.7 subroutine walker.
//! - [`loop_exec`] — the `loop_while` expander ([`ax25sdl::LoopRange`]).
//! - [`quirks`] — the named figc4.x spec-defect fixes (default-on, as in packet.net's `Ax25SessionQuirks`).
//! - [`session`] — the [`session::Session`] driver tying it together.
//!
//! ## Status
//!
//! The PLAN.md §6 blockers — "ax25sdl Rust crate not `no_std` / not typed" — are
//! **RESOLVED** upstream (ax25sdl 0.8.0: `#![no_std]` default-off-`std`, ADR-0003
//! typed closed sets). This crate consumes it as a local path dependency; the
//! runtime is host-tested with `cargo test` and is `no_std`-clean for the M0+.

pub mod bridge;
pub mod context;
pub mod dispatch;
pub mod event;
pub mod guard;
pub mod loop_exec;
pub mod quirks;
pub mod session;
pub mod signal;
pub mod subroutine;
pub mod timer;
pub mod tx;

pub use bridge::{classify_incoming, WireSink};
pub use context::{Payload, SessionContext};
pub use event::{Event, FrameInfo};
pub use loop_exec::{run_loop, MAX_ITERATIONS};
pub use quirks::Quirks;
pub use session::{Session, State};
pub use signal::{
    DataLinkSignal, FrameSpec, InternalSignal, LinkMultiplexerSignal, NullSink, SessionSink,
    SupervisoryKind, UnnumberedKind,
};
pub use timer::{MockTimerService, TimerId, TimerService, TimerSnapshot};
pub use tx::{PendingFrame, Tx};

#[cfg(test)]
mod harness_tests;
#[cfg(test)]
mod tests;
