//! The NinoTNC mode catalog: the (mode number, human name, raw bit rate) table and
//! the firmware-byte → mode reverse map.
//!
//! Ports `Packet.Kiss.NinoTnc.NinoTncMode` + `Packet.Kiss.NinoTnc.NinoTncCatalog`.
//! Modes 0–14 are concrete operating modes selected by the front-panel DIP
//! switches; mode 15 is the "Set from KISS" escape that uses SETHW to choose the
//! effective mode at runtime.
//!
//! `no_std`: where C# uses a `FrozenDictionary`, this uses `const` tables with a
//! linear scan — there are only 16 entries, so the scan is trivially fast and
//! allocation-free. The tables are kept verbatim from the C# (and thus from
//! kissproxy `KissFrameBuilder.cs`), locked to NinoTNC firmware v3.44.

/// One NinoTNC operating mode — the (mode number, human name, raw bit rate) triple.
/// Mode numbers correspond to the DIP-switch position on the TNC's front panel (or
/// the value set via SETHW when DIP=15 "Set from KISS").
///
/// Mirrors `NinoTncMode`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NinoTncMode {
    /// DIP-switch position, 0–15.
    pub mode: u8,
    /// Human-readable description, e.g. `"1200 AFSK AX.25"`.
    pub name: &'static str,
    /// Raw data rate in bits per second (symbol rate × bits per symbol). `0` for
    /// the special *mode 15* entry, which is variable.
    pub bit_rate_hz: u32,
}

impl NinoTncMode {
    /// The time it takes to transmit a frame of `frame_bytes` bytes at this mode's
    /// raw bit rate, in **microseconds** — the no-FPU integer form (the M0+ has no
    /// hardware float). Returns `None` for variable-rate modes (`bit_rate_hz == 0`),
    /// mirroring the C# `double.PositiveInfinity` sentinel.
    ///
    /// Computed as `frame_bytes * 8 * 1_000_000 / bit_rate_hz`, in `u64` to avoid
    /// overflow for any realistic frame size.
    pub fn transmission_us(self, frame_bytes: u32) -> Option<u64> {
        if self.bit_rate_hz == 0 {
            return None;
        }
        Some((frame_bytes as u64) * 8 * 1_000_000 / (self.bit_rate_hz as u64))
    }

    /// The C# `TransmissionMs` formula in `f64`, for exact host-side parity with
    /// `Packet.Kiss.NinoTnc`. `std`-only: there is no FPU on the embedded target,
    /// so on-target code must use [`Self::transmission_us`]. Returns `None` for
    /// variable-rate modes (the C# `PositiveInfinity`).
    #[cfg(feature = "std")]
    pub fn transmission_ms(self, frame_bytes: u32) -> Option<f64> {
        if self.bit_rate_hz == 0 {
            return None;
        }
        Some(frame_bytes as f64 * 8.0 / self.bit_rate_hz as f64 * 1000.0)
    }
}

/// All 16 NinoTNC modes, indexed by DIP-switch position (the array index *is* the
/// mode number). Kept verbatim from the C# `NinoTncCatalog.ByMode` (firmware v3.44).
pub const BY_MODE: [NinoTncMode; 16] = [
    NinoTncMode {
        mode: 0,
        name: "9600 GFSK AX.25",
        bit_rate_hz: 9600,
    },
    NinoTncMode {
        mode: 1,
        name: "19200 4FSK",
        bit_rate_hz: 19200,
    },
    NinoTncMode {
        mode: 2,
        name: "9600 GFSK IL2P+CRC",
        bit_rate_hz: 9600,
    },
    NinoTncMode {
        mode: 3,
        name: "9600 4FSK",
        bit_rate_hz: 9600,
    },
    NinoTncMode {
        mode: 4,
        name: "4800 GFSK IL2P+CRC",
        bit_rate_hz: 4800,
    },
    NinoTncMode {
        mode: 5,
        name: "3600 QPSK IL2P+CRC",
        bit_rate_hz: 3600,
    },
    NinoTncMode {
        mode: 6,
        name: "1200 AFSK AX.25",
        bit_rate_hz: 1200,
    },
    NinoTncMode {
        mode: 7,
        name: "1200 AFSK IL2P+CRC",
        bit_rate_hz: 1200,
    },
    NinoTncMode {
        mode: 8,
        name: "300 BPSK IL2P+CRC",
        bit_rate_hz: 300,
    },
    NinoTncMode {
        mode: 9,
        name: "600 QPSK IL2P+CRC",
        bit_rate_hz: 600,
    },
    NinoTncMode {
        mode: 10,
        name: "1200 BPSK IL2P+CRC",
        bit_rate_hz: 1200,
    },
    NinoTncMode {
        mode: 11,
        name: "2400 QPSK IL2P+CRC",
        bit_rate_hz: 2400,
    },
    NinoTncMode {
        mode: 12,
        name: "300 AFSK AX.25",
        bit_rate_hz: 300,
    },
    NinoTncMode {
        mode: 13,
        name: "300 AFSKPLL IL2P",
        bit_rate_hz: 300,
    },
    NinoTncMode {
        mode: 14,
        name: "300 AFSKPLL IL2P+CRC",
        bit_rate_hz: 300,
    },
    NinoTncMode {
        mode: 15,
        name: "Set from KISS",
        bit_rate_hz: 0,
    },
];

/// Firmware-reported mode bytes (the low byte of the `BrdSwchMod` `ZZZZ` field in a
/// TX-Test frame) → DIP-switch-position mode. Used to decode the "actual operating
/// mode" when DIP=15 ("Set from KISS"). Firmware-version-specific (locked to v3.44).
///
/// Mirrors `NinoTncCatalog.FirmwareByteToMode`. `(firmware_byte, mode)` pairs.
pub const FIRMWARE_BYTE_TO_MODE: [(u8, u8); 16] = [
    (0x00, 0),
    (0x41, 1),
    (0xB0, 2),
    (0x40, 3),
    (0xA3, 4),
    (0xF1, 5),
    (0x02, 6),
    (0x93, 7),
    (0x91, 8),
    (0x92, 9),
    (0xA0, 10),
    (0xA2, 11),
    (0x31, 12),
    (0x22, 13),
    (0x23, 14),
    (0xF3, 15),
];

/// Look up a mode by its DIP-switch position. Returns `None` for out-of-range mode
/// numbers (> 15). Mirrors `NinoTncCatalog.TryGetByMode`.
pub fn try_get_by_mode(mode: u8) -> Option<NinoTncMode> {
    BY_MODE.get(mode as usize).copied()
}

/// Look up the mode the firmware is currently *running* given the byte it reports in
/// a TX-Test frame's `BrdSwchMod` field. Returns `None` for unrecognised firmware
/// values (a clue that the firmware is newer than this table).
///
/// Mirrors `NinoTncCatalog.TryGetByFirmwareByte`.
pub fn try_get_by_firmware_byte(firmware_byte: u8) -> Option<NinoTncMode> {
    FIRMWARE_BYTE_TO_MODE
        .iter()
        .find(|(b, _)| *b == firmware_byte)
        .and_then(|(_, mode)| try_get_by_mode(*mode))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn catalog_has_all_sixteen_modes() {
        assert_eq!(BY_MODE.len(), 16);
        for i in 0u8..=15 {
            assert!(
                try_get_by_mode(i).is_some(),
                "mode {i} should be in the catalog"
            );
            // The array index is the mode number.
            assert_eq!(try_get_by_mode(i).unwrap().mode, i);
        }
        assert!(try_get_by_mode(16).is_none());
    }

    #[test]
    fn catalog_matches_kissproxy_source() {
        let cases = [
            (0u8, "9600 GFSK AX.25", 9600u32),
            (1, "19200 4FSK", 19200),
            (2, "9600 GFSK IL2P+CRC", 9600),
            (6, "1200 AFSK AX.25", 1200),
            (8, "300 BPSK IL2P+CRC", 300),
            (14, "300 AFSKPLL IL2P+CRC", 300),
            (15, "Set from KISS", 0),
        ];
        for (mode, name, rate) in cases {
            let entry = try_get_by_mode(mode).unwrap();
            assert_eq!(entry.name, name);
            assert_eq!(entry.bit_rate_hz, rate);
        }
    }

    #[test]
    fn firmware_byte_lookup_resolves_to_correct_mode() {
        for (fw, mode) in [(0x00u8, 0u8), (0x41, 1), (0x02, 6), (0xF3, 15)] {
            let resolved = try_get_by_firmware_byte(fw).unwrap();
            assert_eq!(resolved.mode, mode);
        }
    }

    #[test]
    fn firmware_byte_lookup_returns_none_for_unknown_byte() {
        assert!(try_get_by_firmware_byte(0xFF).is_none());
    }

    #[test]
    fn transmission_us_matches_integer_formula() {
        // 256 bytes at 1200 baud: 256*8*1_000_000 / 1200 = 1_706_666 us.
        let mode = try_get_by_mode(6).unwrap();
        assert_eq!(mode.transmission_us(256), Some(1_706_666));
    }

    #[test]
    fn transmission_us_for_variable_rate_is_none() {
        assert_eq!(try_get_by_mode(15).unwrap().transmission_us(100), None);
    }

    #[cfg(feature = "std")]
    #[test]
    fn transmission_ms_matches_kissproxy_formula() {
        // kissproxy: (frameBytes * 8.0 / BitRateHz) * 1000.0
        // 256 bytes at 1200 baud → 1706.666… ms.
        let mode = try_get_by_mode(6).unwrap();
        let ms = mode.transmission_ms(256).unwrap();
        assert!((ms - 1_706.666_666_666_666).abs() < 0.001);
        assert!(try_get_by_mode(15).unwrap().transmission_ms(100).is_none());
    }
}
