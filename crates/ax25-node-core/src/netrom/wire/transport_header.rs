//! The NET/ROM L4 transport header — the 5 octets immediately after the L3
//! network header in an inter-node datagram. It identifies the circuit, carries
//! the sliding-window sequence numbers, and names the message type + flow-control
//! flags.
//!
//! Layout (canonical NET/ROM appendix), 5 octets:
//! ```text
//!   [1] circuit index      (slot in the far end's circuit table — "your" index)
//!   [1] circuit ID         (serial qualifying the index — "your" id)
//!   [1] TX sequence number (this message's send-sequence; 8-bit, mod 256)
//!   [1] RX sequence number (the next send-sequence we expect; the piggybacked ack)
//!   [1] opcode & flags     (low nibble = opcode; high bits = flow-control flags)
//! ```
//!
//! Ports `Packet.NetRom.Wire.NetRomTransportHeader` + `NetRomOpcode`. `no_std`,
//! allocation-free. Unlike the TypeScript port (which has no `enum`), Rust models
//! the opcode as a real `#[repr(u8)]` enum; the header stores the raw masked
//! nibble so any value parses (an unknown opcode is surfaced for the circuit layer
//! to reject — parsing itself is total), with [`NetRomOpcode::from_nibble`] the
//! typed view of the known set.

/// Octets a transport header occupies on the wire.
pub const TRANSPORT_HEADER_LEN: usize = 5;

/// The low nibble of the opcode-and-flags byte (the message type).
pub const OPCODE_MASK: u8 = 0x0F;

/// The high bits of the opcode-and-flags byte (the flow-control flags).
pub const FLAGS_MASK: u8 = 0xF0;

/// More-follows (bit 5): this Information message is a non-final fragment of a
/// logical frame larger than one 236-byte payload.
pub const FLAG_MORE_FOLLOWS: u8 = 0x20;

/// NAK (bit 6): request selective retransmission of the frame named by the
/// RX-sequence field.
pub const FLAG_NAK: u8 = 0x40;

/// Choke (bit 7): tell the far end to stop sending Information. On a Connect
/// Acknowledge this same bit instead means the circuit was refused.
pub const FLAG_CHOKE: u8 = 0x80;

/// The six NET/ROM L4 (transport) message types — the low nibble of the
/// opcode-and-flags byte. The de-facto wire numbers every implementation (BPQ,
/// XRouter, the Linux `netrom` family) agrees on. Mirrors C# `NetRomOpcode`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum NetRomOpcode {
    /// Connect Request (0x01): open a circuit. Info carries the originating
    /// user + node callsigns; the header proposes a window size.
    ConnectRequest = 0x01,
    /// Connect Acknowledge (0x02): accept (or, with the choke/bit-7 flag, refuse)
    /// a circuit. Carries the accepted window size.
    ConnectAcknowledge = 0x02,
    /// Disconnect Request (0x03): tear a circuit down.
    DisconnectRequest = 0x03,
    /// Disconnect Acknowledge (0x04): confirm a disconnect.
    DisconnectAcknowledge = 0x04,
    /// Information (0x05): up to 236 bytes of user data, piggybacking an ack via
    /// the RX-sequence field; more-follows marks a fragment.
    Information = 0x05,
    /// Information Acknowledge (0x06): a standalone ack, and the carrier of the
    /// choke / NAK flow-control flags.
    InformationAcknowledge = 0x06,
}

impl NetRomOpcode {
    /// Map a raw opcode nibble (the low 4 bits of the opcode-and-flags byte) to a
    /// known opcode, or `None` for an unknown nibble (the circuit layer rejects
    /// unknowns; parsing the header itself is total).
    pub const fn from_nibble(raw: u8) -> Option<Self> {
        match raw & OPCODE_MASK {
            0x01 => Some(Self::ConnectRequest),
            0x02 => Some(Self::ConnectAcknowledge),
            0x03 => Some(Self::DisconnectRequest),
            0x04 => Some(Self::DisconnectAcknowledge),
            0x05 => Some(Self::Information),
            0x06 => Some(Self::InformationAcknowledge),
            _ => None,
        }
    }

    /// The opcode's wire nibble.
    pub const fn as_u8(self) -> u8 {
        self as u8
    }
}

/// The raw 5-octet NET/ROM transport header. The `opcode`/`flags` are the masked
/// low/high nibbles of the fifth octet (see [`NetRomOpcode::from_nibble`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NetRomTransportHeader {
    /// The far end's circuit-table slot ("your index").
    pub circuit_index: u8,
    /// The serial number qualifying [`Self::circuit_index`] ("your id").
    pub circuit_id: u8,
    /// This message's send sequence (8-bit, wraps mod 256).
    pub tx_sequence: u8,
    /// The next send sequence we expect from the peer — the piggybacked ack.
    pub rx_sequence: u8,
    /// The masked opcode nibble (any 0..=15 parses).
    pub opcode: u8,
    /// The masked flow-control flag bits (the high nibble).
    pub flags: u8,
}

impl NetRomTransportHeader {
    /// True if the choke flag (bit 7) is set. On a Connect Acknowledge this
    /// instead signals refusal.
    pub const fn choke(&self) -> bool {
        self.flags & FLAG_CHOKE != 0
    }

    /// True if the NAK flag (bit 6) is set (selective-retransmit request).
    pub const fn nak(&self) -> bool {
        self.flags & FLAG_NAK != 0
    }

    /// True if the more-follows flag (bit 5) is set (a non-final fragment).
    pub const fn more_follows(&self) -> bool {
        self.flags & FLAG_MORE_FOLLOWS != 0
    }

    /// The raw opcode-and-flags byte (opcode nibble OR-ed with the flag bits).
    pub const fn opcode_and_flags(&self) -> u8 {
        (self.opcode & OPCODE_MASK) | (self.flags & FLAGS_MASK)
    }

    /// Encode this header into the front of `dst` (≥ [`TRANSPORT_HEADER_LEN`]
    /// octets). Returns `None` only if `dst` is too short. Mirrors C# `Write`.
    pub fn encode(&self, dst: &mut [u8]) -> Option<()> {
        if dst.len() < TRANSPORT_HEADER_LEN {
            return None;
        }
        dst[0] = self.circuit_index;
        dst[1] = self.circuit_id;
        dst[2] = self.tx_sequence;
        dst[3] = self.rx_sequence;
        dst[4] = self.opcode_and_flags();
        Some(())
    }

    /// Decode a 5-octet transport header from the front of `src`. Total: returns
    /// `None` only if the slice is too short — any opcode nibble parses (surfaced
    /// raw for the circuit layer to interpret). Mirrors C# `TryParse`.
    pub fn decode(src: &[u8]) -> Option<Self> {
        if src.len() < TRANSPORT_HEADER_LEN {
            return None;
        }
        let op = src[4];
        Some(Self {
            circuit_index: src[0],
            circuit_id: src[1],
            tx_sequence: src[2],
            rx_sequence: src[3],
            opcode: op & OPCODE_MASK,
            flags: op & FLAGS_MASK,
        })
    }
}
