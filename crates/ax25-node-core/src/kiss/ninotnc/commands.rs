//! NinoTNC-specific control-command KISS frames: the firmware's query channel
//! beyond standard KISS.
//!
//! Ports the query subset of `Packet.Kiss.NinoTnc.NinoTncCommands` that the node
//! needs: GETALL (the full diagnostic report, used to read the running mode back
//! after a SETHW) and GETVER. Replies come back on the reply command byte `0xE0`
//! ([`super::rssi::REPLY_COMMAND_BYTE`]), which decodes as port 14 / Data; GETALL
//! answers with the labelled `=FirmwareVr:` text that [`super::classify`] already
//! surfaces as a TX-Test diagnostic (bench-verified in packet.net on firmware 3.41
//! and 3.44).
//!
//! The bootloader, serial-number and STOPTX commands are deliberately not ported:
//! nothing on the node sends them, and the bootloader one reboots the modem.

use crate::kiss::frame::Command;

/// KISS command code for GETVER: request the firmware version string.
pub const GET_VERSION_COMMAND: u8 = 0x08;

/// KISS command code for GETALL: request a full diagnostic-register report.
pub const GET_ALL_COMMAND: u8 = 0x0B;

/// The GETALL / GETVER request payload (a single `0x00` byte).
pub const QUERY_PAYLOAD: [u8; 1] = [0x00];

/// Encode a GETALL request (`C0 0B 00 C0` on port 0) into `dst`. Returns the
/// encoded length, or `None` if `dst` is too small or `port > 15`.
pub fn build_get_all_into(dst: &mut [u8], port: u8) -> Option<usize> {
    crate::kiss::encode_into(dst, port, Command::Other(GET_ALL_COMMAND), &QUERY_PAYLOAD)
}

/// Encode a GETVER request (`C0 08 00 C0` on port 0) into `dst`.
pub fn build_get_version_into(dst: &mut [u8], port: u8) -> Option<usize> {
    crate::kiss::encode_into(
        dst,
        port,
        Command::Other(GET_VERSION_COMMAND),
        &QUERY_PAYLOAD,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn get_all_matches_the_packet_net_wire_bytes() {
        let mut buf = [0u8; 8];
        let n = build_get_all_into(&mut buf, 0).unwrap();
        assert_eq!(&buf[..n], &[0xC0, 0x0B, 0x00, 0xC0]);
    }

    #[test]
    fn get_version_matches_the_packet_net_wire_bytes() {
        let mut buf = [0u8; 8];
        let n = build_get_version_into(&mut buf, 0).unwrap();
        assert_eq!(&buf[..n], &[0xC0, 0x08, 0x00, 0xC0]);
    }

    #[test]
    fn port_nibble_is_carried() {
        let mut buf = [0u8; 8];
        let n = build_get_all_into(&mut buf, 3).unwrap();
        assert_eq!(&buf[..n], &[0xC0, 0x3B, 0x00, 0xC0]);
    }
}
