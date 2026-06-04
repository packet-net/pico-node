//! NinoTNC firmware version + dsPIC chip variant.
//!
//! Ports `Packet.Kiss.NinoTnc.Firmware.NinoTncFirmwareVersion` and
//! `NinoTncChipVariant` (the host-relevant value types). The OTA firmware
//! *catalogue/flasher* (GitHub release discovery, ICSP flashing) is host-only
//! tooling with no place on the embedded node and is intentionally out of scope —
//! see the parity notes in [`super`].

/// The dsPIC chip variant fitted to a NinoTNC. The two variants need *different*
/// firmware images — flashing the wrong one bricks the modem until an ICSP
/// programmer recovers it. The major component of the firmware version string
/// (Nino's convention from firmware 2.90 onward) tells you which is running.
///
/// Mirrors `NinoTncChipVariant`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ChipVariant {
    /// Unknown — firmware version not parseable / not recognised.
    #[default]
    Unknown,
    /// dsPIC33EP256GP. Firmware major version `3`.
    Dspic33Ep256,
    /// dsPIC33EP512GP. Firmware major version `4`.
    Dspic33Ep512,
}

/// A NinoTNC firmware version in Nino's two-component form, e.g. `3.44` or `4.44`.
/// The major component encodes the chip variant (`3` = dsPIC33EP256GP, `4` =
/// dsPIC33EP512GP) per the convention adopted from firmware 2.90 onward.
///
/// Mirrors `NinoTncFirmwareVersion`. Comparison is by `(major, minor)` — note that
/// ranking only makes sense *within* a chip variant (a `3.44` is not "less than" a
/// `4.44`; they target different chips), so callers should filter by
/// [`Self::chip_variant`] before ranking. `Ord` is still derived so the type sorts
/// in collections, with major as the primary key (matching the C# `CompareTo`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct FirmwareVersion {
    /// Chip-variant indicator. 3 or 4 for current hardware; older firmware
    /// (pre-2.90) used a single sequence (1.x, 2.x) without the chip overload.
    pub major: u16,
    /// Sequential release number within the major. E.g. 44 in `3.44`.
    pub minor: u16,
}

impl FirmwareVersion {
    /// The chip variant this firmware version implies.
    pub fn chip_variant(self) -> ChipVariant {
        match self.major {
            3 => ChipVariant::Dspic33Ep256,
            4 => ChipVariant::Dspic33Ep512,
            _ => ChipVariant::Unknown,
        }
    }

    /// Parse a firmware version string. Accepts `"3.44"`, `"4.44"`, and the legacy
    /// `"2.71"` shape. Whitespace is trimmed; otherwise strict (exactly one `.`,
    /// non-empty non-negative integer on each side). Returns `None` on any
    /// violation.
    ///
    /// Mirrors `NinoTncFirmwareVersion.TryParse`.
    pub fn parse(text: &str) -> Option<Self> {
        let trimmed = text.trim();
        if trimmed.is_empty() {
            return None;
        }
        let (major_s, minor_s) = trimmed.split_once('.')?;
        if major_s.is_empty() || minor_s.is_empty() {
            return None;
        }
        // u16::from_str rejects leading '-' / '+' and any non-digit, so the
        // "non-negative integer" contract is enforced for free.
        let major: u16 = major_s.parse().ok()?;
        let minor: u16 = minor_s.parse().ok()?;
        Some(Self { major, minor })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_three_dot_forty_four() {
        let v = FirmwareVersion::parse("3.44").unwrap();
        assert_eq!(
            v,
            FirmwareVersion {
                major: 3,
                minor: 44
            }
        );
        assert_eq!(v.chip_variant(), ChipVariant::Dspic33Ep256);
    }

    #[test]
    fn major_four_is_the_512_variant() {
        assert_eq!(
            FirmwareVersion::parse("4.10").unwrap().chip_variant(),
            ChipVariant::Dspic33Ep512
        );
    }

    #[test]
    fn legacy_two_x_is_unknown_variant() {
        assert_eq!(
            FirmwareVersion::parse("2.71").unwrap().chip_variant(),
            ChipVariant::Unknown
        );
    }

    #[test]
    fn trims_whitespace() {
        assert_eq!(
            FirmwareVersion::parse("  3.44 ").unwrap(),
            FirmwareVersion {
                major: 3,
                minor: 44
            }
        );
    }

    #[test]
    fn rejects_garbage() {
        assert_eq!(FirmwareVersion::parse("banana"), None);
        assert_eq!(FirmwareVersion::parse(""), None);
        assert_eq!(FirmwareVersion::parse("3."), None);
        assert_eq!(FirmwareVersion::parse(".44"), None);
        assert_eq!(FirmwareVersion::parse("3"), None);
        assert_eq!(FirmwareVersion::parse("-3.44"), None);
        assert_eq!(FirmwareVersion::parse("3.4.4"), None);
    }

    #[test]
    fn orders_by_major_then_minor() {
        let a = FirmwareVersion::parse("3.44").unwrap();
        let b = FirmwareVersion::parse("3.45").unwrap();
        let c = FirmwareVersion::parse("4.01").unwrap();
        assert!(a < b);
        assert!(b < c);
    }
}
