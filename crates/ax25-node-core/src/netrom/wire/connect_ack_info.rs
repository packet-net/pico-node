//! Codec for the information field of a NET/ROM L4 Connect Acknowledge (opcode
//! 0x02). The first octet, the **accepted send-window**, is base NET/ROM and
//! rides every acknowledgement; LinBPQ adds a second octet (a time-to-live/flags
//! byte) when the Connect Request came from a BPQ node, and folds its
//! compression-agreed bit into it.
//!
//! Wire layout (LinBPQ `L4Code.c` `SendConACK`), 1 or 2 octets:
//! ```text
//!   [1] accepted send-window size          (always - base NET/ROM)
//!   [1] TTL byte; bit 0x80 = "compression agreed"   (BPQ extension)
//! ```
//!
//! The window octet is *not* an extension: LinBPQ writes
//! `L3MSG->L4DATA[0] = L4->L4WINDOW` then sets `LENGTH = MSGHDDRLEN + 22`
//! (`L4Code.c:1768,1824`), a 21-byte vanilla Connect Acknowledge, and reads it
//! back unconditionally (`L4->L4WINDOW = L3MSG->L4DATA[0]`, `L4Code.c:2287`);
//! only the *second* octet is length-gated on `BPQNODE`. Linux `af_netrom` does
//! the same: `nr_write_internal` emits `nr->window` with `NR_CONNACK_LEN = 1`
//! and the receive path reads `skb->data[20]`. A peer that ignores the octet is
//! unharmed; it is trailing info to it.
//!
//! The compression bit is only ever set when *both* ends offered compression. On
//! receipt the originator masks it off before reading the TTL
//! (`L4DATA[1] &= 0x7f`), so it is harmless to a peer that ignores it. A
//! **refusing** acknowledge accepts nothing and carries no info field at all,
//! matching Linux's `nr_transmit_refusal` (a bare 20-byte frame).
//!
//! Ports `Packet.NetRom.Wire.ConnectAckInfo`. `no_std`, allocation-free (the
//! 1-or-2-octet field is returned by value). Not feature-gated: the window octet
//! is base NET/ROM, so the default build must be able to express it; only the
//! second (compress) octet is a BPQ extension.

/// Octets in a vanilla (base NET/ROM) Connect Acknowledge info field: the
/// accepted send-window alone. Mirrors C# `ConnectAckInfo.VanillaLength`.
pub const CONNECT_ACK_INFO_VANILLA_LEN: usize = 1;

/// Octets in the LinBPQ extended Connect Acknowledge info field.
pub const CONNECT_ACK_INFO_EXTENDED_LEN: usize = 2;

/// The largest window a peer may accept: the NET/ROM sequence space leaves bit 7
/// to the flags, so 127 is the ceiling, the same clamp the circuit applies to its
/// own proposal. Mirrors C# `ConnectAckInfo.MaxWindow`.
pub const CONNECT_ACK_MAX_WINDOW: u8 = 127;

/// The "compression agreed" bit, OR-ed into the TTL octet of an extended Connect
/// Acknowledge (LinBPQ `L4Code.c`: `L3MSG->L4DATA[1] |= 0x80`). Mirrors C#
/// `ConnectAckInfo.CompressBit`.
pub const CONNECT_ACK_COMPRESS_BIT: u8 = 0x80;

/// Codec for the Connect Acknowledge info field. A unit type carrying the
/// associated functions, mirroring the C# `static class ConnectAckInfo`.
pub struct ConnectAckInfo;

impl ConnectAckInfo {
    /// Build the Connect Acknowledge info field: the accepted window (always),
    /// plus the TTL octet carrying the compression-agreed bit when
    /// `agree_compression` is true. Returns the field by value with its length
    /// (1 or 2); the 1-octet form is the vanilla NET/ROM acknowledgement that
    /// LinBPQ and Linux both emit. Mirrors C# `ConnectAckInfo.Build`.
    pub fn encode(
        accepted_window: u8,
        time_to_live: u8,
        agree_compression: bool,
    ) -> ([u8; CONNECT_ACK_INFO_EXTENDED_LEN], usize) {
        if !agree_compression {
            return ([accepted_window, 0], CONNECT_ACK_INFO_VANILLA_LEN);
        }
        (
            [accepted_window, time_to_live | CONNECT_ACK_COMPRESS_BIT],
            CONNECT_ACK_INFO_EXTENDED_LEN,
        )
    }

    /// Read the accepted send-window an acknowledging peer reported. Returns
    /// `None` for an absent octet (a terse peer that sent no info field at all)
    /// or an out-of-range value (0, or > [`CONNECT_ACK_MAX_WINDOW`], a peer that
    /// put something else there); the originator then keeps the window it
    /// proposed. Mirrors LinBPQ's unconditional `L4WINDOW = L4DATA[0]`
    /// (`L4Code.c:2287`) and Linux's `skb->data[20]` read, with the sanity bound
    /// BPQ gets from its own `L4DEFAULTWINDOW` fallback (`L4Code.c:2010-2013`).
    /// Mirrors C# `ConnectAckInfo.TryReadAcceptedWindow`.
    pub fn try_read_accepted_window(info: &[u8]) -> Option<u8> {
        match info.first() {
            Some(&w) if w != 0 && w <= CONNECT_ACK_MAX_WINDOW => Some(w),
            _ => None,
        }
    }

    /// Read the BPQ compression-agreed bit from a Connect Acknowledge info field.
    /// Returns `false` for the short (vanilla, window-only) form. Mirrors C#
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
        let (info, len) = ConnectAckInfo::encode(4, 10, true);
        assert_eq!(len, CONNECT_ACK_INFO_EXTENDED_LEN);
        assert_eq!(info[0], 4);
        assert_eq!(info[1] & 0x7F, 10); // TTL survives under the masked-off bit
        assert!(ConnectAckInfo::agrees_compression(&info[..len]));
        assert_eq!(
            ConnectAckInfo::try_read_accepted_window(&info[..len]),
            Some(4)
        );
    }

    #[test]
    fn declining_ack_is_the_vanilla_window_only_form() {
        // The accepted-window octet is base NET/ROM, not a compression extension:
        // a declining ack still carries it, and nothing else.
        let (info, len) = ConnectAckInfo::encode(4, 10, false);
        assert_eq!(len, CONNECT_ACK_INFO_VANILLA_LEN);
        assert_eq!(&info[..len], &[4]);
        assert!(!ConnectAckInfo::agrees_compression(&info[..len]));
        assert_eq!(
            ConnectAckInfo::try_read_accepted_window(&info[..len]),
            Some(4)
        );
    }

    #[test]
    fn accepted_window_read_rejects_absent_or_out_of_range_octets() {
        // A terse peer (no info field) or a nonsense octet leaves the proposal
        // standing: the reader returns None and the caller keeps its window.
        assert_eq!(ConnectAckInfo::try_read_accepted_window(&[]), None);
        assert_eq!(ConnectAckInfo::try_read_accepted_window(&[0]), None);
        assert_eq!(ConnectAckInfo::try_read_accepted_window(&[128]), None);
        assert_eq!(ConnectAckInfo::try_read_accepted_window(&[127]), Some(127));
        assert_eq!(ConnectAckInfo::try_read_accepted_window(&[1]), Some(1));
    }
}
