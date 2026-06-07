//! The console→AX.25 outbound-connect relay — the `ConsoleRelay.PipeAsync`
//! analogue. A console user types `C <call>`; the console transport parks its
//! prompt loop and pipes raw bytes to/from an outbound AX.25 session that the
//! AXUDP task (the session owner) establishes on its behalf.
//!
//! Plumbing: a request channel (console → session owner), two byte pipes (one
//! per direction), a hangup signal (console user went away) and a status
//! signal (connect confirmed / link ended). **One relay at a time** — the
//! statics are a single relay slot, and `begin` bounces a second `C` with
//! `Busy` (a Pico node serving one operator; lift by generation-tagging the
//! pipes if it ever matters).

use ax25_node_core::ax25::Callsign;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::pipe::Pipe;
use embassy_sync::signal::Signal;

/// Relay lifecycle reported by the session owner to the console side.
#[derive(Clone, Copy, Debug)]
pub enum RelayStatus {
    /// The peer accepted (UA received) — the link is up.
    Connected,
    /// The connect could not even be attempted (no known endpoint / no slot).
    Failed(&'static str),
    /// The link ended: peer DISC, link failure (N2 exhausted), or our hangup
    /// completed.
    Disconnected,
}

/// Console → session-owner: "connect to this callsign and relay".
pub static CONNECT_REQ: Channel<CriticalSectionRawMutex, Callsign, 1> = Channel::new();
/// Bytes from the console user toward the AX.25 peer.
pub static USER_TO_AX: Pipe<CriticalSectionRawMutex, 1024> = Pipe::new();
/// Bytes from the AX.25 peer toward the console user.
pub static AX_TO_USER: Pipe<CriticalSectionRawMutex, 1024> = Pipe::new();
/// Console side hung up (socket EOF) — the session owner should DISC.
pub static USER_HANGUP: Signal<CriticalSectionRawMutex, ()> = Signal::new();
/// Lifecycle events for the console side.
pub static STATUS: Signal<CriticalSectionRawMutex, RelayStatus> = Signal::new();

/// Start a relay to `target`. Drains stale state from any previous relay and
/// enqueues the connect request. `Err(())` ⇒ a relay is already in progress.
pub fn begin(target: Callsign) -> Result<(), ()> {
    // A queued-but-unclaimed request means the owner hasn't even started; a
    // full channel means a relay is pending/active.
    let mut scratch = [0u8; 64];
    while USER_TO_AX.try_read(&mut scratch).is_ok() {}
    while AX_TO_USER.try_read(&mut scratch).is_ok() {}
    USER_HANGUP.reset();
    STATUS.reset();
    CONNECT_REQ.try_send(target).map_err(|_| ())
}
