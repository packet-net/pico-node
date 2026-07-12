//! Codec for the information field of a NET/ROM L4 Connect Acknowledge (opcode
//! 0x02) in the LinBPQ **extended** form. Vanilla NET/ROM sends a Connect
//! Acknowledge with an empty info field; LinBPQ, when the Connect Request came
//! from a BPQ node that offered compression, replies with two octets — the
//! accepted send-window and a time-to-live/flags octet — and folds its
//! compression-agreed bit into the latter.
//!
//! Wire layout (LinBPQ `L4Code.c` Connect Acknowledge build), 2 octets:
//! ```text
//!   [1] accepted send-window size
//!   [1] TTL byte; bit 0x80 = "compression agreed" (L4DATA[1] |= 0x80)
//! ```
//!
//! The bit is only ever set when *both* ends offered compression. On receipt the
//! originator masks it off before reading the TTL (`L4DATA[1] &= 0x7f`), so it is
//! harmless to a peer that ignores it. A **declining** acknowledge is the vanilla
//! empty-info form (byte-for-byte plain NET/ROM), so a non-compressing circuit
//! never emits this extension.
//!
//! Ports `Packet.NetRom.Wire.ConnectAckInfo`. `no_std`, allocation-free (the
//! 2-octet extension is returned by value). Gated behind the `netrom-compress`
//! feature so the default on-target build carries no compression surface.

/// Octets in the LinBPQ extended Connect Acknowledge info field.
pub const CONNECT_ACK_INFO_EXTENDED_LEN: usize = 2;

/// The "compression agreed" bit, OR-ed into the TTL octet of an extended Connect
/// Acknowledge (LinBPQ `L4Code.c`: `L3MSG->L4DATA[1] |= 0x80`). Mirrors C#
/// `ConnectAckInfo.CompressBit`.
pub const CONNECT_ACK_COMPRESS_BIT: u8 = 0x80;

/// Codec for the extended Connect Acknowledge info field. A unit type carrying the
/// two associated functions, mirroring the C# `static class ConnectAckInfo`.
pub struct ConnectAckInfo;

impl ConnectAckInfo {
    /// Build the extended Connect Acknowledge info field when
    /// `agree_compression` is set: `[accepted_window, time_to_live | 0x80]`.
    /// Returns `None` for the vanilla (declining) form so the caller emits an
    /// empty info field — a circuit that did not negotiate compression stays
    /// byte-for-byte the plain NET/ROM Connect Acknowledge. Mirrors C#
    /// `ConnectAckInfo.Build` (which returns `[]` when not agreeing).
    pub fn encode(
        accepted_window: u8,
        time_to_live: u8,
        agree_compression: bool,
    ) -> Option<[u8; CONNECT_ACK_INFO_EXTENDED_LEN]> {
        if !agree_compression {
            return None;
        }
        Some([accepted_window, time_to_live | CONNECT_ACK_COMPRESS_BIT])
    }

    /// Read the BPQ compression-agreed bit from a Connect Acknowledge info field.
    /// Returns `false` for the empty / short (vanilla) form. Mirrors C#
    /// `ConnectAckInfo.AgreesCompression`.
    pub fn agrees_compression(info: &[u8]) -> bool {
        info.len() >= CONNECT_ACK_INFO_EXTENDED_LEN && (info[1] & CONNECT_ACK_COMPRESS_BIT) != 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn agreeing_ack_carries_window_and_ttl_with_the_bit() {
        let info = ConnectAckInfo::encode(4, 10, true).unwrap();
        assert_eq!(info[0], 4);
        assert_eq!(info[1] & 0x7F, 10); // TTL survives under the masked-off bit
        assert!(ConnectAckInfo::agrees_compression(&info));
    }

    #[test]
    fn declining_ack_is_the_empty_vanilla_form() {
        assert!(ConnectAckInfo::encode(4, 10, false).is_none());
        // The vanilla empty info field agrees to nothing.
        assert!(!ConnectAckInfo::agrees_compression(&[]));
        assert!(!ConnectAckInfo::agrees_compression(&[4])); // too short
    }
}
