//! Cross-task snapshot of the learned NET/ROM routes for the `Nodes` console
//! command.
//!
//! The NET/ROM service (and its routing table) lives in the AXUDP task; the
//! console-bearing transports (telnet, and the AX.25-session console) run in their
//! own Embassy tasks and only hold a clone of the boot-time [`Identity`]. This
//! static is the seam between them — the AXUDP task refreshes it on each beacon
//! tick (`set_routes(netrom.route_lines())`), and a console task reads it into the
//! `Identity.routes` it dispatches a `Nodes` command with. Same pattern as
//! [`crate::oled::STATUS`] / [`crate::mqtt::STATUS`], which already carry the
//! cross-task NET/ROM neighbour/destination counts.
//!
//! [`Identity`]: ax25_node_core::console::service::Identity

use alloc::string::String;
use alloc::vec::Vec;
use core::cell::RefCell;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;

/// The shared route lines, each already rendered by `NetRomService::route_lines`.
/// Empty until the first refresh (and whenever the table is empty / NET/ROM off).
static ROUTES: Mutex<CriticalSectionRawMutex, RefCell<Vec<String>>> =
    Mutex::new(RefCell::new(Vec::new()));

/// Replace the shared route snapshot. Called from the task that owns the routing
/// table (AXUDP) — cheap; the line count is small (≤ the destination cap).
pub fn set_routes(routes: Vec<String>) {
    ROUTES.lock(|c| *c.borrow_mut() = routes);
}

/// Clone the current route snapshot for a console `Nodes` render. Called from the
/// console tasks; returns an owned `Vec<String>` so the lock is held only for the
/// clone.
pub fn snapshot() -> Vec<String> {
    ROUTES.lock(|c| c.borrow().clone())
}
