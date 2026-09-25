//! The radio port: the hand-off between the node's session task
//! ([`super::axudp`], which runs connected-mode sessions, the console, NET/ROM
//! and NODES for every port) and the serial KISS task ([`super::kiss_serial`],
//! which owns the UART and the TNC).
//!
//! The serial task delivers each AX.25 frame it hears into [`RX`] and puts on
//! the air whatever arrives in [`TX`]. Both directions are gated on
//! [`usable`]: the serial task sets it only while the TNC is on supported
//! firmware and not being updated, so an unsupported or busy TNC carries no
//! traffic at all.
//!
//! Both channels are bounded and never block the session task: a full queue
//! drops the frame (AX.25 retransmits; a NODES broadcast comes round again).

use core::sync::atomic::{AtomicBool, Ordering};

use alloc::vec::Vec;

use ax25_node_core::ax25::Frame;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

/// Frames heard on the radio, for the session task.
pub static RX: Channel<CriticalSectionRawMutex, Frame, 4> = Channel::new();

/// Encoded AX.25 frames (no FCS) for the serial task to put on the air.
pub static TX: Channel<CriticalSectionRawMutex, Vec<u8>, 8> = Channel::new();

static USABLE: AtomicBool = AtomicBool::new(false);

/// Whether the radio port carries traffic now.
pub fn usable() -> bool {
    USABLE.load(Ordering::Relaxed)
}

/// Set by the serial task as the TNC becomes usable or not.
pub fn set_usable(on: bool) {
    USABLE.store(on, Ordering::Relaxed);
}

/// Queue a frame for the air (from the session task). Dropped when the port is
/// not usable or the queue is full.
pub async fn send(wire: Vec<u8>) {
    if !usable() {
        return;
    }
    if TX.try_send(wire).is_err() {
        defmt::warn!("rf: transmit queue full, frame dropped");
    }
}

/// Hand a heard frame to the session task (from the serial task). `false` if
/// the port is not usable or the session task is behind.
pub fn deliver(frame: Frame) -> bool {
    usable() && RX.try_send(frame).is_ok()
}
