//! The node's radio ports, and the hand-off between each port driver and the
//! node task ([`crate::node`]).
//!
//! A port is a KISS modem with a radio behind it:
//!
//! - [`Port::NINOTNC`]: the NinoTNC on the serial link ([`ninotnc`]), the
//!   node's radio. Its driver also owns the TNC's setup, monitor and firmware
//!   update.
//! - [`Port::KISS_TCP`]: an optional KISS-over-TCP modem ([`kiss_tcp`]), such
//!   as net-sim's emulated RF channel for testing without hardware. Enabled by
//!   the build-env `KISS_TCP_TARGET`.
//!
//! The node task runs everything above the modem for every port: sessions, the
//! console, NET/ROM (one routing table), NODES, interlinks. A driver delivers
//! each AX.25 frame it hears into [`RX`] (tagged with its port) and transmits
//! what the node queues for it ([`send`] / [`take_tx`]). Both directions are
//! gated on [`usable`]: a driver marks its port usable only when its modem can
//! carry traffic (the NinoTNC: supported firmware, not being updated; KISS-TCP:
//! connected).
//!
//! Queues are bounded and never block the node task: a full queue drops the
//! frame (AX.25 retransmits; a NODES broadcast comes round again).

pub mod kiss_tcp;
pub mod ninotnc;

use core::sync::atomic::{AtomicBool, Ordering};

use alloc::vec::Vec;

use ax25_node_core::ax25::frame::CONTROL_UI;
use ax25_node_core::ax25::{Address, Callsign, Frame};
use ax25_node_core::netrom::PortId;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

/// A radio port, by index.
#[derive(Clone, Copy, PartialEq, Eq, Debug, defmt::Format)]
pub struct Port(pub u8);

/// How many ports the node has.
pub const PORT_COUNT: usize = 2;

impl Port {
    /// The NinoTNC on the serial link.
    pub const NINOTNC: Port = Port(0);
    /// The optional KISS-over-TCP modem.
    pub const KISS_TCP: Port = Port(1);

    /// Every port, in preference order (the radio first).
    pub const ALL: [Port; PORT_COUNT] = [Port::NINOTNC, Port::KISS_TCP];

    /// The port's name, as the console and logs show it.
    pub fn name(self) -> &'static str {
        match self {
            Port::NINOTNC => "radio",
            _ => "kiss-tcp",
        }
    }

    /// The NET/ROM port id routes learned on this port carry.
    pub fn netrom_id(self) -> PortId {
        PortId::from_str_lossy(self.name())
    }

    fn index(self) -> usize {
        (self.0 as usize).min(PORT_COUNT - 1)
    }
}

/// Frames heard on any port, for the node task.
pub static RX: Channel<CriticalSectionRawMutex, (Port, Frame), 4> = Channel::new();

/// Encoded AX.25 frames (no FCS) for each port's driver to transmit.
static TX: [Channel<CriticalSectionRawMutex, Vec<u8>, 8>; PORT_COUNT] =
    [const { Channel::new() }; PORT_COUNT];

static USABLE: [AtomicBool; PORT_COUNT] = [const { AtomicBool::new(false) }; PORT_COUNT];

/// Whether `port` carries traffic now.
pub fn usable(port: Port) -> bool {
    USABLE[port.index()].load(Ordering::Relaxed)
}

/// Set by a port's driver as its modem becomes usable or not.
pub fn set_usable(port: Port, on: bool) {
    USABLE[port.index()].store(on, Ordering::Relaxed);
}

/// The first usable port in preference order, if any.
pub fn first_usable() -> Option<Port> {
    Port::ALL.into_iter().find(|p| usable(*p))
}

/// Queue a frame for `port` to transmit (from the node task). Dropped when the
/// port is not usable or its queue is full.
pub async fn send(port: Port, wire: Vec<u8>) {
    if !usable(port) {
        return;
    }
    if TX[port.index()].try_send(wire).is_err() {
        defmt::warn!("ports: {=str} transmit queue full, frame dropped", port.name());
    }
}

/// The next frame the node wants `port` to transmit (for the port's driver).
pub async fn take_tx(port: Port) -> Vec<u8> {
    TX[port.index()].receive().await
}

/// Hand a heard frame to the node task (from a port's driver). `false` if the
/// port is not usable or the node task is behind.
pub fn deliver(port: Port, frame: Frame) -> bool {
    usable(port) && RX.try_send((port, frame)).is_ok()
}

/// Render a callsign into a small stack buffer for defmt logging.
pub fn call_str<'b>(call: &Callsign, buf: &'b mut [u8; 16]) -> &'b str {
    let n = call.write_display(buf).unwrap_or(0);
    core::str::from_utf8(&buf[..n]).unwrap_or("?")
}

/// Build a UI frame `my_call -> dest` with the given PID + info (NODES
/// broadcasts use the NET/ROM 0xCF; test frames 0xF0 no-L3).
pub fn ui_frame(my_call: Callsign, dest: Callsign, pid: u8, info: &[u8]) -> Frame {
    Frame {
        destination: Address {
            callsign: dest,
            crh: true,
            extension: false,
        },
        source: Address {
            callsign: my_call,
            crh: false,
            extension: false,
        },
        digipeaters: Vec::new(),
        control: CONTROL_UI,
        pid: Some(pid),
        info: info.to_vec(),
    }
}
