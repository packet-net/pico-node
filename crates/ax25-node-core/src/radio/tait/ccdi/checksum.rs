//! The CCDI checksum (manual §1.8.5). Ports `Packet.Radio.Tait.Ccdi.CcdiChecksum`.
//!
//! Modulo-256 sum of every message byte before the checksum field, two's-
//! complemented, rendered as two upper-case ASCII hex digits.

use super::hex_upper;

/// Compute the two-character checksum for `body` (the `[IDENT][SIZE][PARAMETERS]`
/// portion of a message). Returns the two upper-case ASCII hex digits.
///
/// Byte-for-byte identical to the C# `CcdiChecksum.Compute`: `(-Σbytes) & 0xFF`.
/// CCDI bodies are all-ASCII, so summing the raw bytes matches summing the chars.
pub fn compute(body: &[u8]) -> [u8; 2] {
    let mut sum: u32 = 0;
    for &b in body {
        sum = sum.wrapping_add(b as u32);
    }
    let checksum = (sum.wrapping_neg() & 0xFF) as u8;
    [hex_upper(checksum >> 4), hex_upper(checksum & 0x0F)]
}

/// Validate that `checksum` is the correct checksum of `body`. Case-sensitive, per
/// the spec's upper-case rule (mirrors `CcdiChecksum.IsValid`).
pub fn is_valid(body: &[u8], checksum: &[u8]) -> bool {
    checksum.len() == 2 && compute(body) == checksum
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The golden vector: the manual's §1.9.8 CCR-over-SDM worked example.
    /// `a130520312345678M01D0E` → `36`. This is the parity contract the whole
    /// Tait port is anchored on.
    #[test]
    fn golden_vector_ccr_over_sdm() {
        assert_eq!(&compute(b"a130520312345678M01D0E"), b"36");
        assert!(is_valid(b"a130520312345678M01D0E", b"36"));
    }

    /// Every worked example the manual gives (mirrors `CcdiCodecTests`).
    #[test]
    fn matches_manual_examples() {
        let cases: &[(&[u8], &[u8])] = &[
            (b"s0D050800TESTHi!", b"DA"),                 // §1.8.5
            (b"q00", b"2F"),                              // §1.8.4 minimum-length
            (b"f0291", b"CE"),                            // §1.9.3 activate transmitter
            (b"f03041", b"A2"),                           // §1.9.3 enable progress
            (b"q045063", b"5D"),                          // §1.10.1 averaged RSSI query
            (b"a130520312345678M01D0E", b"36"),           // §1.9.8 CCR-over-SDM
            (b"a120520612345678GPRMC", b"22"),            // §1.9.8 NMEA-request SDM
            (b"a1B0520612345678GPGGA,87654321", b"55"),   // §1.9.8 NMEA + return addr
            (b"s0A0512345678", b"13"),                    // §1.9.7 legacy SEND_SDM
            (b"s0CFF12345678Hi", b"39"),                  // §1.9.7 legacy SEND_SDM "Hi"
            (b"f03011", b"A5"),                           // §1.9.3 enable volume control
            (b"f040225", b"6D"),                          // §1.9.3 volume level 25
            (b"f03025", b"A0"),                           // §1.9.3 volume level 5
            (b"f03031", b"A3"),                           // §1.9.3 enable Selcall output
            (b"f03051", b"A1"),                           // §1.9.3 enable channel progress
            (b"f03101", b"A5"),                           // §1.9.3 SDM output on reception
            (b"f03111", b"A4"),                           // §1.9.3 SDM caller-ID encode
            (b"f03121", b"A3"),                           // §1.9.3 SDM caller-ID decode
            (b"f0241", b"D3"),                            // §1.9.3 disable user input
            (b"f0271", b"D0"),                            // §1.9.3 validate subaudible
        ];
        for (body, expected) in cases {
            assert_eq!(&compute(body), expected, "checksum of {:?}", core::str::from_utf8(body));
            assert!(is_valid(body, expected));
        }
    }

    #[test]
    fn rejects_wrong_or_malformed_checksum() {
        assert!(!is_valid(b"q00", b"2E")); // off by one
        assert!(!is_valid(b"q00", b"2f")); // wrong case (spec is upper-case)
        assert!(!is_valid(b"q00", b"2")); // wrong length
        assert!(!is_valid(b"q00", b"2F0")); // wrong length
    }

    #[test]
    fn empty_body_is_two_complement_of_zero() {
        assert_eq!(&compute(b""), b"00");
    }
}
