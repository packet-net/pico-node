//! A full NET/ROM L3 datagram — the payload of one inter-node interlink I-frame
//! (PID 0xCF): a 15-octet [`NetRomNetworkHeader`], a 5-octet
//! [`NetRomTransportHeader`], and the transport payload (0..236 octets).
//!
//! Ports `Packet.NetRom.Wire.NetRomPacket`. `no_std`, allocation-free: on decode
//! the payload **borrows** the source slice rather than copying into a heap
//! buffer; the circuit layer copies it into its own bounded buffers if it needs
//! to retain it.

use super::network_header::{NetRomNetworkHeader, NETWORK_HEADER_LEN};
use super::transport_header::{NetRomTransportHeader, TRANSPORT_HEADER_LEN};

/// Octets the two headers occupy (network 15 + transport 5).
pub const PACKET_HEADER_LEN: usize = NETWORK_HEADER_LEN + TRANSPORT_HEADER_LEN; // 20

/// Maximum transport payload per datagram (the §6.6 fragment size).
pub const MAX_PAYLOAD: usize = 236;

/// A NET/ROM datagram: the L3 + L4 headers and a borrowed payload slice.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NetRomPacket<'a> {
    /// The L3 network header (end-to-end origin/destination + TTL).
    pub network: NetRomNetworkHeader,
    /// The L4 transport header (circuit id, sequencing, opcode + flags).
    pub transport: NetRomTransportHeader,
    /// The transport payload (0..[`MAX_PAYLOAD`] octets), borrowed from the source.
    pub payload: &'a [u8],
}

impl<'a> NetRomPacket<'a> {
    /// Encode the headers + payload into the front of `dst`, returning the total
    /// length written. Returns `None` if `dst` cannot hold the 20-octet header
    /// plus the payload.
    pub fn encode(&self, dst: &mut [u8]) -> Option<usize> {
        let total = PACKET_HEADER_LEN + self.payload.len();
        if dst.len() < total {
            return None;
        }
        self.network.encode(&mut dst[0..])?;
        self.transport.encode(&mut dst[NETWORK_HEADER_LEN..])?;
        dst[PACKET_HEADER_LEN..total].copy_from_slice(self.payload);
        Some(total)
    }

    /// Decode a datagram from `src`; the returned packet's `payload` borrows
    /// `src`. Total: returns `None` only if `src` is shorter than the 20-octet
    /// header (a payload longer than [`MAX_PAYLOAD`] still parses — the circuit
    /// layer decides what to do with it).
    pub fn decode(src: &'a [u8]) -> Option<Self> {
        if src.len() < PACKET_HEADER_LEN {
            return None;
        }
        let network = NetRomNetworkHeader::decode(&src[0..])?;
        let transport = NetRomTransportHeader::decode(&src[NETWORK_HEADER_LEN..])?;
        Some(Self {
            network,
            transport,
            payload: &src[PACKET_HEADER_LEN..],
        })
    }
}
