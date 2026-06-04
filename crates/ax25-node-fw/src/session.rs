//! Link-layer session layer — the on-target home of the SDL runtime port.
//!
//! Mirrors `Packet.Ax25.Session.Ax25Listener` + `Ax25Session`: a small fixed
//! array of per-peer sessions (NOT the desktop's unbounded LRU dictionary — a
//! Pico node serves a handful of links, see research note §6), each running the
//! generated AX.25 v2.2 state machine via the runtime in
//! [`ax25_node_core::sdl`]. Transports hand inbound frames here addressed by peer;
//! outbound frames flow back to whichever transport owns the link.
//!
//! STUB. The runtime that walks the generated tables (`ActionDispatcher` etc.) is
//! the major port still to be written, and it is gated on the `ax25sdl` Rust crate
//! becoming consumable (publish + no_std + typed) — see [`ax25_node_core::sdl`]
//! and docs/PLAN.md §"SDL integration".

use embassy_time::Timer;

/// Maximum concurrent link-layer sessions. Fixed (no heap session map). Sized for
/// a Pico node; bump with care given the per-session window buffers (research §6).
pub const MAX_SESSIONS: usize = 4;

/// The T1/T2/T3 timer service for all sessions. STUB: the real loop arms/services
/// per-session timers off `embassy_time` and feeds timer-expiry events into the
/// session walk (replacing the C# `SystemTimerScheduler`).
#[embassy_executor::task]
pub async fn timer_task() {
    loop {
        // Placeholder cadence; the real service waits on the nearest armed timer.
        Timer::after_millis(100).await;
    }
}
