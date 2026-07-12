//! The XID "Classes of Procedures" parameter (PI=2) — ports
//! `Packet.Ax25.Xid.ClassesOfProcedures`.
//!
//! A 16-bit (PL=2) field per AX.25 v2.2 §4.3.3.7 (Figure 4.5) and the
//! negotiation rules in §6.3.2. For AX.25 only the duplex selection is
//! negotiable; the remaining bits are fixed.
//!
//! Bit layout (LSB-first within each octet; octet 0 transmitted first):
//! bit 0 — Balanced ABM: always 1; bits 1–4 — always 0; bit 5 — Half Duplex;
//! bit 6 — Full Duplex (exactly one of bit 5 / bit 6 set); bits 7–15 — 0.
//!
//! Note the spec prose (§6.3.2 ¶1080) says "bit 0 always 1" but Figure 4.6
//! encodes PV `0x22 0x00` = ABM(bit1)+half-duplex(bit5). We follow the Figure 4.5
//! table + normative prose: ABM is the low bit (0x01), half-duplex is 0x20, so
//! half-duplex ABM encodes as `0x21 0x00`.

/// Balanced ABM — bit 0, always set for AX.25.
const BIT_ABM_BALANCED: u32 = 0;
/// Half-duplex operation — bit 5.
const BIT_HALF_DUPLEX: u32 = 5;
/// Full-duplex operation — bit 6.
const BIT_FULL_DUPLEX: u32 = 6;

/// The XID Classes of Procedures parameter — the duplex selection (§4.3.3.7,
/// Figure 4.5). Byte-for-byte with C# `ClassesOfProcedures`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ClassesOfProcedures {
    /// True for half-duplex, false for full-duplex. The default per §6.3.2 is
    /// half-duplex ("reverts to half-duplex if either TNC cannot support
    /// full-duplex").
    pub half_duplex: bool,
}

impl ClassesOfProcedures {
    /// Half-duplex Classes of Procedures (the AX.25 default). Mirrors C#
    /// `ClassesOfProcedures.HalfDuplexDefault`.
    pub const HALF_DUPLEX_DEFAULT: Self = Self { half_duplex: true };

    /// Full-duplex Classes of Procedures. Mirrors C#
    /// `ClassesOfProcedures.FullDuplexCapable`.
    pub const FULL_DUPLEX_CAPABLE: Self = Self { half_duplex: false };

    /// Encode to the 2-octet PV (octet 0 first). ABM (bit 0) is forced set;
    /// exactly one of half-duplex (bit 5) / full-duplex (bit 6) is set; all
    /// other bits are zero per the Figure 4.5 fixed values.
    pub fn to_octets(self) -> [u8; 2] {
        let field: u32 = (1 << BIT_ABM_BALANCED)
            | (1 << if self.half_duplex {
                BIT_HALF_DUPLEX
            } else {
                BIT_FULL_DUPLEX
            });
        // LSB-first per octet: octet0 = bits 0–7, octet1 = bits 8–15.
        [(field & 0xFF) as u8, ((field >> 8) & 0xFF) as u8]
    }

    /// Decode from the (up to) 2-octet PV. Duplex is read from bits 5/6; if
    /// neither is set we default to half-duplex (the spec default). All other
    /// bits are ignored on receive — only the duplex selection is meaningful.
    pub fn from_octets(octet0: u8, octet1: u8) -> Self {
        let field: u32 = octet0 as u32 | ((octet1 as u32) << 8);
        let full = (field & (1 << BIT_FULL_DUPLEX)) != 0;
        let half = (field & (1 << BIT_HALF_DUPLEX)) != 0;
        // Half-duplex unless only full-duplex is asserted.
        Self {
            half_duplex: !(full && !half),
        }
    }
}

impl Default for ClassesOfProcedures {
    /// Half-duplex — the AX.25 default (§6.3.2). Matches C# `new ClassesOfProcedures()`.
    fn default() -> Self {
        Self::HALF_DUPLEX_DEFAULT
    }
}
