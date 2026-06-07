//! CRC-16/X.25 — the AX.25 frame check sequence (FCS).
//!
//! A faithful port of `Packet.Core.Crc16Ccitt` from `m0lte/packet.net`. AX.25
//! v2.2 §3.7 names ISO 3309 CRC-CCITT (polynomial `0x1021`); the per-byte
//! processing is LSB-first, so we use the reflected polynomial `0x8408` and shift
//! right. In CRC-catalogue terms this is **CRC-16/X-25**:
//!
//! - Polynomial `0x1021` (`x^16 + x^12 + x^5 + 1`)
//! - Init `0xFFFF`, RefIn `true`, RefOut `true`, XorOut `0xFFFF`
//! - Standard check value: `"123456789"` → `0x906E`.
//!
//! This is `no_std`, allocation-free, and `const`-friendly.

/// `0x8408` is the bit-reverse of `0x1021`, used because RefIn/RefOut = true is
/// implemented by reflecting the polynomial and shifting right.
const REFLECTED_POLYNOMIAL: u16 = 0x8408;

/// Compute the AX.25 FCS over `data` (CRC-16/X.25).
pub fn compute(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        crc ^= byte as u16;
        for _ in 0..8 {
            if crc & 1 != 0 {
                crc = (crc >> 1) ^ REFLECTED_POLYNOMIAL;
            } else {
                crc >>= 1;
            }
        }
    }
    crc ^ 0xFFFF
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn check_vector_123456789() {
        // The canonical CRC-16/X.25 check value — matches the C# port's documented
        // test vector and the CRC catalogue.
        assert_eq!(compute(b"123456789"), 0x906E);
    }

    #[test]
    fn empty_input_is_init_xor_out() {
        // CRC over no bytes = Init XOR XorOut = 0xFFFF ^ 0xFFFF = 0x0000.
        assert_eq!(compute(b""), 0x0000);
    }

    #[test]
    fn appended_fcs_self_checks_to_constant_residue() {
        // X.25/HDLC property: for ANY message, running the CRC again over
        // (message || FCS-little-endian) yields a fixed residue independent of the
        // message — that's exactly how an AX.25 receiver validates a frame, and it
        // proves our FCS byte order (low byte first) matches the wire. We assert
        // the residue is the same across two unrelated messages (the invariant
        // that matters); for THIS implementation it is 0x0F47.
        let residue = |msg: &[u8]| {
            let fcs = compute(msg);
            let mut framed = msg.to_vec();
            framed.push((fcs & 0xFF) as u8); // low byte first on the wire
            framed.push((fcs >> 8) as u8); // then high byte
            compute(&framed)
        };
        let r1 = residue(b"M0LTE de G7XYZ test");
        let r2 = residue(b"a completely different payload of another length");
        assert_eq!(r1, r2, "residue must be message-independent");
        assert_eq!(
            r1, 0x0F47,
            "fixed CRC-16/X.25 residue for low-byte-first FCS"
        );
    }

    #[test]
    fn single_byte_differs_from_zero() {
        assert_ne!(compute(&[0x00]), 0x0000);
    }
}
