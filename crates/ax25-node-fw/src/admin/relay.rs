//! The console→AX.25 outbound-connect relay — the `ConsoleRelay.PipeAsync`
//! analogue. A console user types `C <call>`; the console transport parks its
//! prompt loop and pipes raw bytes to/from an outbound AX.25 session that the
//! node task (the session owner) establishes on its behalf, over the air.
//!
//! Plumbing: a request channel (console → session owner), two byte pipes (one
//! per direction), a hangup signal (console user went away) and a status
//! signal (connect confirmed / link ended). **One relay at a time** — the
//! statics are a single relay slot, and `begin` bounces a second `C` with
//! `Busy` (a Pico node serving one operator; lift by generation-tagging the
//! pipes if it ever matters).

use core::cell::RefCell;

use alloc::collections::VecDeque;

use ax25_node_core::ax25::Callsign;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
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
/// Bytes from the AX.25 peer toward the console user. The node task writes
/// through [`to_user`], never directly, so nothing is dropped when the telnet
/// side falls behind.
pub static AX_TO_USER: Pipe<CriticalSectionRawMutex, 1024> = Pipe::new();

/// Peer bytes that did not fit in [`AX_TO_USER`] yet, in order. The node task
/// feeds them in with [`flush_some`] as the telnet task drains the pipe.
///
/// A burst of I-frames (a chat server's help text: seven 250-byte frames in 4 s)
/// used to overrun the 1 KB pipe, and `try_write`'s partial write silently cut
/// frames short mid-line (bench, 2026-09-25). Bounded at [`BACKLOG_MAX`]: beyond
/// that the user is told how much was lost rather than seeing corrupted text.
/// (AX.25 flow control, sending RNR while the backlog is high, would need a
/// local-busy event the session layer does not have yet.)
static BACKLOG: Mutex<CriticalSectionRawMutex, RefCell<VecDeque<u8>>> =
    Mutex::new(RefCell::new(VecDeque::new()));

/// The most peer bytes held back for a slow telnet user.
const BACKLOG_MAX: usize = 8 * 1024;

/// Hand bytes from the AX.25 peer to the console user, keeping order and
/// losing nothing short of [`BACKLOG_MAX`].
pub fn to_user(data: &[u8]) {
    BACKLOG.lock(|cell| {
        let mut backlog = cell.borrow_mut();
        let mut rest = data;
        // Straight into the pipe only when nothing is waiting ahead of it.
        if backlog.is_empty() {
            if let Ok(n) = AX_TO_USER.try_write(rest) {
                rest = &rest[n..];
            }
        }
        let room = BACKLOG_MAX.saturating_sub(backlog.len());
        let keep = rest.len().min(room);
        backlog.extend(&rest[..keep]);
        let lost = rest.len() - keep;
        if lost > 0 {
            defmt::warn!("relay: telnet user too slow, {=usize} bytes lost", lost);
            let mut note = heapless::String::<64>::new();
            let _ = core::fmt::Write::write_fmt(
                &mut note,
                format_args!("\r[pico-node: {lost} bytes lost, telnet too slow]\r"),
            );
            backlog.extend(note.as_bytes());
        }
    });
}

/// Bytes still waiting to reach the console user.
pub fn backlog_len() -> usize {
    BACKLOG.lock(|cell| cell.borrow().len())
}

/// Move some of the backlog into the pipe, waiting for the telnet task to make
/// room. Never completes while the backlog is empty. Cancel-safe: bytes leave
/// the backlog only once the pipe has taken them.
pub async fn flush_some() {
    let mut chunk = [0u8; 256];
    let n = BACKLOG.lock(|cell| {
        let backlog = cell.borrow();
        let n = backlog.len().min(chunk.len());
        for (dst, src) in chunk.iter_mut().zip(backlog.iter()) {
            *dst = *src;
        }
        n
    });
    if n == 0 {
        core::future::pending::<()>().await;
    }
    let written = AX_TO_USER.write(&chunk[..n]).await;
    BACKLOG.lock(|cell| {
        cell.borrow_mut().drain(..written);
    });
}
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
    BACKLOG.lock(|cell| cell.borrow_mut().clear());
    USER_HANGUP.reset();
    STATUS.reset();
    CONNECT_REQ.try_send(target).map_err(|_| ())
}
