//! KISS framing constants, command codes, and the decoded-frame type.
//!
//! Ports `Packet.Kiss.KissFraming`, `Packet.Kiss.KissCommand`, and
//! `Packet.Kiss.KissFrame`.

#[cfg(feature = "alloc")]
use alloc::vec::Vec;

/// Frame End delimiter.
pub const FEND: u8 = 0xC0;
/// Frame Escape — enter escape mode.
pub const FESC: u8 = 0xDB;
/// Transposed Frame End — escaped form of `FEND`.
pub const TFEND: u8 = 0xDC;
/// Transposed Frame Escape — escaped form of `FESC`.
pub const TFESC: u8 = 0xDD;
/// Exit-KISS-mode command, a single bare `0xFF` (no FEND framing required).
pub const EXIT_KISS_MODE: u8 = 0xFF;

/// KISS command codes — the low nibble of the KISS command byte. (The high nibble
/// is the multi-drop port number, 0–15.) Mirrors `Packet.Kiss.KissCommand`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Command {
    /// Data frame — HDLC payload to transmit / received from the radio.
    Data = 0x0,
    /// TXDELAY, units of 10 ms.
    TxDelay = 0x1,
    /// Persistence parameter (0–255).
    Persistence = 0x2,
    /// Slot time, units of 10 ms.
    SlotTime = 0x3,
    /// TX tail (obsolete on most modern TNCs).
    TxTail = 0x4,
    /// Full-duplex flag.
    FullDuplex = 0x5,
    /// TNC-specific set-hardware payload.
    SetHardware = 0x6,
    /// ACKMODE (G8BPQ extension) — data with a 2-byte ack-tag prefix.
    AckMode = 0xC,
    /// Poll request (polled-mode extension).
    Poll = 0xE,
    /// Any command code we don't model, preserved so the decoder is total and a
    /// round-trip is lossless. Holds the raw low nibble.
    Other(u8),
}

impl Command {
    /// Map a raw low-nibble command code to a [`Command`]. Total: unknown codes
    /// become [`Command::Other`].
    pub const fn from_nibble(nibble: u8) -> Self {
        match nibble & 0x0F {
            0x0 => Command::Data,
            0x1 => Command::TxDelay,
            0x2 => Command::Persistence,
            0x3 => Command::SlotTime,
            0x4 => Command::TxTail,
            0x5 => Command::FullDuplex,
            0x6 => Command::SetHardware,
            0xC => Command::AckMode,
            0xE => Command::Poll,
            other => Command::Other(other),
        }
    }

    /// The 4-bit command code for the wire.
    pub const fn to_nibble(self) -> u8 {
        match self {
            Command::Data => 0x0,
            Command::TxDelay => 0x1,
            Command::Persistence => 0x2,
            Command::SlotTime => 0x3,
            Command::TxTail => 0x4,
            Command::FullDuplex => 0x5,
            Command::SetHardware => 0x6,
            Command::AckMode => 0xC,
            Command::Poll => 0xE,
            Command::Other(n) => n & 0x0F,
        }
    }
}

/// One decoded KISS frame: a port (0–15), a command, and the raw payload.
/// Mirrors `Packet.Kiss.KissFrame`. For [`Command::Data`] the payload is the
/// AX.25 frame minus the FCS (the TNC strips/inserts the FCS).
#[cfg(feature = "alloc")]
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Frame {
    /// Multi-drop port number, 0–15.
    pub port: u8,
    /// KISS command code.
    pub command: Command,
    /// Raw bytes between the command byte and the closing FEND.
    pub payload: Vec<u8>,
}

#[cfg(feature = "alloc")]
impl Frame {
    /// Build a frame from parts. `port` is masked to 0–15.
    pub fn new(port: u8, command: Command, payload: Vec<u8>) -> Self {
        Self {
            port: port & 0x0F,
            command,
            payload,
        }
    }

    /// The command byte as it appears on the wire: `(port << 4) | command`.
    pub fn command_byte(&self) -> u8 {
        ((self.port & 0x0F) << 4) | self.command.to_nibble()
    }
}
