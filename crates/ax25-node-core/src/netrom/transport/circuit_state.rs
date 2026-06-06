//! The lifecycle state of a NET/ROM L4 circuit and the reason it closed.
//!
//! A textbook connection FSM: Disconnected → (Connecting | accepting) → Connected
//! → Disconnecting → Disconnected. Hand-written (NET/ROM has no SDL figures, and
//! BPQ is the de-facto reference). Mirrors the C# `NetRomCircuitState` and
//! `NetRomCircuitCloseReason`. Rust gets real enums (the TS port uses an `as const`
//! value-union for the same compile-time-typed closed set).

/// The four lifecycle states of a circuit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NetRomCircuitState {
    /// No circuit — the initial and terminal state.
    Disconnected,
    /// We sent a Connect Request and are awaiting the Connect Acknowledge.
    Connecting,
    /// The circuit is up; Information may flow both ways.
    Connected,
    /// We sent a Disconnect Request and are awaiting the Disconnect Acknowledge.
    Disconnecting,
}

/// Why a circuit ended — surfaced to the consumer on close.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NetRomCircuitCloseReason {
    /// A clean disconnect (either end requested it and it was acknowledged).
    Normal,
    /// The far end refused our Connect Request (Connect Acknowledge with the
    /// refuse/choke bit).
    Refused,
    /// Retries were exhausted on a connect / disconnect / data message — the link
    /// is dead.
    Timeout,
}
