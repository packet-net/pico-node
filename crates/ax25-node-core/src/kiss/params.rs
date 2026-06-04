//! KISS TNC parameter-command builders (TXDELAY / P / SLOTTIME / TXTAIL /
//! FULLDUPLEX).
//!
//! Ports the parameter-setting surface of `Packet.Kiss.Serial.KissSerialModem`
//! (`SetTxDelayAsync`, `SetPersistenceAsync`, `SetSlotTimeAsync`, `SetTxTailAsync`,
//! `SetFullDuplexAsync`). The C# side is an async serial write; in this byte-only
//! codec the equivalent is "produce the wire bytes for this single-byte parameter
//! command", which the firmware UART/TCP task then writes. Each command is a
//! one-byte-payload KISS frame on the given port.
//!
//! These are tiny and never need escaping in practice (the value byte is escaped by
//! the encoder anyway, so any value is safe).

use super::frame::Command;

#[cfg(feature = "alloc")]
use super::encoder::encode;
#[cfg(feature = "alloc")]
use alloc::vec::Vec;

/// Encode a single-byte parameter command into `dst`. Returns bytes written, or
/// `None` if `dst` is too small or `port` is out of range. The building block for
/// the named helpers below.
pub fn encode_param_into(dst: &mut [u8], port: u8, command: Command, value: u8) -> Option<usize> {
    super::encode_into(dst, port, command, &[value])
}

/// KISS TXDELAY (`0x01`), units of 10 ms. Mirrors `SetTxDelayAsync`.
#[cfg(feature = "alloc")]
pub fn tx_delay(port: u8, ten_ms_units: u8) -> Option<Vec<u8>> {
    encode(port, Command::TxDelay, &[ten_ms_units])
}

/// KISS PERSIST (`0x02`), 0..=255. Mirrors `SetPersistenceAsync`.
#[cfg(feature = "alloc")]
pub fn persistence(port: u8, value: u8) -> Option<Vec<u8>> {
    encode(port, Command::Persistence, &[value])
}

/// KISS SLOTTIME (`0x03`), units of 10 ms. Mirrors `SetSlotTimeAsync`.
#[cfg(feature = "alloc")]
pub fn slot_time(port: u8, ten_ms_units: u8) -> Option<Vec<u8>> {
    encode(port, Command::SlotTime, &[ten_ms_units])
}

/// KISS TXTAIL (`0x04`), units of 10 ms. Modern modems generally ignore this; the
/// KISS spec recommends 0. Mirrors `SetTxTailAsync`.
#[cfg(feature = "alloc")]
pub fn tx_tail(port: u8, ten_ms_units: u8) -> Option<Vec<u8>> {
    encode(port, Command::TxTail, &[ten_ms_units])
}

/// KISS FULLDUPLEX (`0x05`): non-zero = full duplex. Mirrors `SetFullDuplexAsync`.
#[cfg(feature = "alloc")]
pub fn full_duplex(port: u8, full: bool) -> Option<Vec<u8>> {
    encode(port, Command::FullDuplex, &[if full { 1 } else { 0 }])
}

#[cfg(all(test, feature = "alloc"))]
mod tests {
    use super::*;
    use crate::kiss::frame::FEND;
    use alloc::vec;

    #[test]
    fn tx_delay_frames_command_0x01() {
        // port 0, TXDELAY, value 50 (= 500 ms): FEND 0x01 0x32 FEND
        assert_eq!(tx_delay(0, 50).unwrap(), vec![FEND, 0x01, 0x32, FEND]);
    }

    #[test]
    fn persistence_frames_command_0x02() {
        assert_eq!(persistence(0, 63).unwrap(), vec![FEND, 0x02, 0x3F, FEND]);
    }

    #[test]
    fn slot_time_frames_command_0x03() {
        assert_eq!(slot_time(0, 10).unwrap(), vec![FEND, 0x03, 0x0A, FEND]);
    }

    #[test]
    fn tx_tail_frames_command_0x04() {
        assert_eq!(tx_tail(0, 0).unwrap(), vec![FEND, 0x04, 0x00, FEND]);
    }

    #[test]
    fn full_duplex_frames_command_0x05() {
        assert_eq!(full_duplex(0, true).unwrap(), vec![FEND, 0x05, 0x01, FEND]);
        assert_eq!(full_duplex(0, false).unwrap(), vec![FEND, 0x05, 0x00, FEND]);
    }

    #[test]
    fn param_uses_port_nibble() {
        // port 2, SLOTTIME: command byte (2<<4)|0x03 = 0x23
        assert_eq!(slot_time(2, 10).unwrap(), vec![FEND, 0x23, 0x0A, FEND]);
    }

    #[test]
    fn encode_param_into_writes_into_buffer() {
        let mut buf = [0u8; 8];
        let n = encode_param_into(&mut buf, 0, Command::TxDelay, 50).unwrap();
        assert_eq!(&buf[..n], &[FEND, 0x01, 0x32, FEND]);
    }
}
