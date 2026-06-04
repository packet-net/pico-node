//! SDL link-layer runtime — glue between the generated AX.25 state tables and
//! this node.
//!
//! ## Status: scaffolded, NOT yet wired to the real tables. See the blocker.
//!
//! The headline differentiator of this whole project (per
//! `docs/research/pico-packet-node.md`) is running the *same* generated AX.25 v2.2
//! SDL state machine that the C# (`packet.net`) and TypeScript (`ax25-ts`) stacks
//! run — from one spec source, proving link-layer parity across hardware classes.
//!
//! `m0lte/ax25sdl` already emits a **Rust** backend (`spec/rust`): the ~243 v2.2
//! transitions + figc4.7 subroutines as `pub static … : StatePage` tables of
//! `&'static` data (`TransitionSpec`, `ActionStep`, `SubroutinePath`, `LoopRange`).
//! Those tables are inert data — ideal for embedding (const, ROM-able, no heap).
//!
//! **The work — and what is NOT done here — is the *runtime that walks them*.**
//! In the C# stack that's ~6 k LOC (`ActionDispatcher`, `Ax25Session`,
//! `GuardEvaluator`, `SubroutineRegistry`, `SdlLoopExecutor`, the frame codec,
//! `Segmenter`, timers). This module is the place that port lands. The
//! [`loop_exec`] sub-module is the first piece (the `LoopRange` expander —
//! `SdlLoopExecutor`), ported and host-tested standalone because it depends only
//! on the table *shape*, not the table *data*.
//!
//! ## Blockers consuming the real `ax25sdl` Rust crate (tracked in docs/PLAN.md)
//!
//! 1. **Not published.** `spec/rust/Cargo.toml` is `publish = false`; the crate
//!    is a CI build/test target, not a crates.io artifact. To depend on it this
//!    workspace would use a path/git dependency, or ax25sdl would publish it.
//! 2. **Not `no_std`.** `spec/rust/src/lib.rs` / `types.rs` carry no `#![no_std]`.
//!    The *content* is `&'static str` / `&'static [...]` with no `Vec`/`String`/
//!    `Box` — so it is no_std-*compatible* — but the attribute + a `default-std`
//!    feature have to be added upstream (ax25sdl Phase-0 task in the research
//!    note) before it builds for `thumbv6m-none-eabi`.
//! 3. **Still stringly-typed (verbs AND guards/events).** The Rust emitter writes
//!    `verb: "V(s) := V(s) + 1"`, `guard: "peer_busy == false"`, `on: "..."` as
//!    raw `&'static str`. SP-010's typed closed sets (`Ax25ActionVerb`,
//!    `Ax25Guard`, `Ax25Event`) have shipped only in the **C# and TS** backends
//!    (ax25sdl ADR-0002, 2026-06-03); the **Rust backend has not been migrated**.
//!    Until it is, the Rust runtime must either ship a string-expression parser +
//!    string-keyed dispatch on the M0+ (works — it's what C#/TS do — but wasteful
//!    in flash and cycles) or hand-maintain an enum mapping that drifts from the
//!    codegen. The clean fix is upstream: extend the Rust emitter to emit the
//!    same typed enums, so this module's dispatcher is an exhaustive `match`.
//!
//! These are dependency/sequencing items, not blockers on *this* crate's
//! host-testable work — hence the scaffold here and the explicit plan entries.

pub mod loop_exec;

pub use loop_exec::{run_loop, LoopRange, MAX_ITERATIONS};
