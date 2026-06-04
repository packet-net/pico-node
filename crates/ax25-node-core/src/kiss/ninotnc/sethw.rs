//! NinoTNC-specific `SETHW` (KISS command `0x06`) payload + frame construction.
//!
//! Ports `Packet.Kiss.NinoTnc.NinoTncSetHardware`. The NinoTNC's SETHW payload is a
//! single byte: the mode number 0–15, optionally `+16` to apply the mode for the
//! current power cycle only (leaving the flash-stored mode untouched, sparing flash
//! write cycles). This is the one piece of genuinely NinoTNC-flavoured KISS — other
//! modems use SETHW differently or ignore it (see `IKissModem` remarks in the C#).

use crate::kiss::frame::Command;

#[cfg(feature = "alloc")]
use crate::kiss::encoder::encode;
#[cfg(feature = "alloc")]
use alloc::vec::Vec;

/// Add this to a mode number to instruct the NinoTNC to honour the mode for the
/// current power cycle only, leaving the flash-stored mode unchanged.
pub const NON_PERSIST_OFFSET: u8 = 16;

/// Maximum valid mode number (DIP position 0–15).
pub const MAX_MODE: u8 = 15;

/// Compute the single-byte SETHW payload for a given mode and persist preference.
/// Returns `None` if `mode > 15`.
///
/// - `persist_to_flash == true` → the TNC writes the new mode to flash so it
///   survives a reboot (returns `mode`).
/// - `persist_to_flash == false` → RAM-only (`mode + 16`). This is the
///   commonly-preferred default in tooling because flash has limited write cycles.
///
/// Mirrors `NinoTncSetHardware.BuildPayloadByte`.
pub fn build_payload_byte(mode: u8, persist_to_flash: bool) -> Option<u8> {
    if mode > MAX_MODE {
        return None;
    }
    Some(if persist_to_flash {
        mode
    } else {
        mode + NON_PERSIST_OFFSET
    })
}

/// Build a fully-encoded KISS SETHW frame for the given mode on `port`. Returns
/// `None` if `mode > 15` or `port > 15`. Requires `alloc`.
///
/// Mirrors `NinoTncSetHardware.BuildKissFrame`.
#[cfg(feature = "alloc")]
pub fn build_kiss_frame(mode: u8, persist_to_flash: bool, port: u8) -> Option<Vec<u8>> {
    let payload = build_payload_byte(mode, persist_to_flash)?;
    encode(port, Command::SetHardware, &[payload])
}

/// Build a KISS SETHW frame into a caller-provided buffer — the allocation-free
/// path for the embedded transport. Returns bytes written, or `None` if `mode`/
/// `port` is out of range or `dst` is too small (size with
/// [`crate::kiss::max_encoded_len(1)`](crate::kiss::max_encoded_len)).
pub fn build_kiss_frame_into(
    dst: &mut [u8],
    mode: u8,
    persist_to_flash: bool,
    port: u8,
) -> Option<usize> {
    let payload = build_payload_byte(mode, persist_to_flash)?;
    crate::kiss::encode_into(dst, port, Command::SetHardware, &[payload])
}

#[cfg(test)]
mod tests {
    use super::*;
    #[cfg(feature = "alloc")]
    use crate::kiss::frame::FEND;
    #[cfg(feature = "alloc")]
    use alloc::vec;

    #[test]
    fn payload_byte_matches_kissproxy_arithmetic() {
        // (mode, persist, expected)
        let cases = [
            (0u8, true, 0u8),
            (0, false, 16),
            (6, true, 6),
            (6, false, 22),
            (15, true, 15),
            (15, false, 31),
        ];
        for (mode, persist, expected) in cases {
            assert_eq!(build_payload_byte(mode, persist), Some(expected));
        }
    }

    #[test]
    fn out_of_range_mode_returns_none() {
        assert_eq!(build_payload_byte(16, true), None);
        assert_eq!(build_payload_byte(255, false), None);
    }

    #[cfg(feature = "alloc")]
    #[test]
    fn kiss_frame_encodes_as_sethw_command_with_mode_payload() {
        // mode 6, persist=false → payload 22 (0x16); FEND, 0x06, 0x16, FEND.
        let frame = build_kiss_frame(6, false, 0).unwrap();
        assert_eq!(frame, vec![FEND, 0x06, 0x16, FEND]);
    }

    #[cfg(feature = "alloc")]
    #[test]
    fn kiss_frame_uses_port_nibble() {
        // port 2, mode 6, persist=true → command byte (2<<4)|0x06 = 0x26.
        let frame = build_kiss_frame(6, true, 2).unwrap();
        assert_eq!(frame, vec![FEND, 0x26, 0x06, FEND]);
    }

    #[cfg(feature = "alloc")]
    #[test]
    fn kiss_frame_rejects_out_of_range_mode() {
        assert_eq!(build_kiss_frame(16, true, 0), None);
    }

    #[test]
    fn build_into_matches_alloc_path() {
        let mut buf = [0u8; 8];
        let n = build_kiss_frame_into(&mut buf, 6, false, 0).unwrap();
        assert_eq!(&buf[..n], &[0xC0, 0x06, 0x16, 0xC0]);
    }
}
