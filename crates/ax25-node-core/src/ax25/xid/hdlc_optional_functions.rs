//! The XID "HDLC Optional Functions" parameter (PI=3) — ports
//! `Packet.Ax25.Xid.HdlcOptionalFunctions` + `RejectMode`.
//!
//! A 24-bit (PL=3) field per AX.25 v2.2 §4.3.3.7 (Figure 4.5) and §6.3.2
//! ¶1082–1090. For AX.25 this carries the two genuinely-negotiated selections —
//! the reject scheme (REJ vs SREJ) and the modulo (8 vs 128) — plus the
//! segmenter/reassembler bit; every other bit is fixed.
//!
//! Bit layout (logical bits 0–23; bit 0 = the low bit of the low-order octet):
//! bit 1 — REJ (set ⇒ implicit reject); bit 2 — SREJ (set ⇒ selective reject);
//! bit 7 — Extended address (always 1); bit 10 — Modulo 8; bit 11 — Modulo 128;
//! bit 13 — TEST (always 1); bit 15 — 16-bit FCS (always 1); bit 17 —
//! Synchronous transmit (always 1); bit 21 — SREJ multiframe; bit 22 —
//! Segmenter/reassembler; every other bit fixed 0.
//!
//! **Octet order.** The 3-octet PV goes on the wire most-significant octet first
//! per §3.8 ("high-order octet first"). Figure 4.6 prints the PV
//! least-significant-octet first (`82 A8 22`; §3.8-correct it is `22 A8 82`) — a
//! figure-rendering error that contradicts §3.8; we follow §3.8, matching
//! direwolf and LinBPQ on the wire (proven: BPQ accepts the MSB-first PV and
//! negotiates SREJ, silently drops the LSB-first one).

/// The reject scheme negotiated by the HDLC Optional Functions field.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RejectMode {
    /// Implicit reject (REJ) — bit 1 set, bit 2 reset (§6.3.2 ¶1086).
    ImplicitReject,
    /// Selective reject (SREJ) — bit 1 reset, bit 2 set (§6.3.2 ¶1087).
    SelectiveReject,
}

const BIT_REJ: u32 = 1;
const BIT_SREJ: u32 = 2;
const BIT_EXTENDED_ADDRESS: u32 = 7; // always 1
const BIT_MODULO8: u32 = 10;
const BIT_MODULO128: u32 = 11;
const BIT_TEST: u32 = 13; // always 1
const BIT_FCS16: u32 = 15; // always 1
const BIT_SYNC_TX: u32 = 17; // always 1
const BIT_SREJ_MULTIFRAME: u32 = 21;
const BIT_SEGMENTER: u32 = 22;

/// The XID HDLC Optional Functions parameter — reject scheme + modulo +
/// segmenter (§4.3.3.7, Figure 4.5). Byte-for-byte with C#
/// `HdlcOptionalFunctions`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HdlcOptionalFunctions {
    /// The reject scheme — implicit (REJ) or selective (SREJ).
    pub reject: RejectMode,
    /// True ⇒ modulo-128 selected; false ⇒ modulo-8.
    pub modulo128: bool,
    /// True ⇒ the SREJ-multiframe option (bit 21) is asserted.
    pub srej_multiframe: bool,
    /// True ⇒ the segmenter/reassembler option (bit 22) is asserted.
    pub segmenter_reassembler: bool,
}

impl HdlcOptionalFunctions {
    /// The AX.25 v2.2 default per §6.3.2 ¶1090: selective reject, modulo 128, no
    /// segmenter. Mirrors C# `HdlcOptionalFunctions.Default`.
    pub const DEFAULT: Self = Self {
        reject: RejectMode::SelectiveReject,
        modulo128: true,
        srej_multiframe: false,
        segmenter_reassembler: false,
    };

    /// Encode to the 3-octet PV, most-significant octet first (§3.8; direwolf /
    /// BPQ). Forces the always-1 bits (extended address, TEST, 16-bit FCS,
    /// synchronous Tx) and sets exactly one reject bit and exactly one modulo bit.
    pub fn to_octets(self) -> [u8; 3] {
        self.to_octets_ordered(false)
    }

    /// Encode to the 3-octet PV with the octet order selectable. When
    /// `lsb_octet_first` is `false` (the default, spec-correct) the value is
    /// transmitted most-significant octet first (§3.8). `true` reproduces the
    /// repo's historical (§3.8-violating) least-significant-octet-first layout,
    /// kept only for regression study. Mirrors C# `ToOctets(bool lsbOctetFirst)`.
    pub fn to_octets_ordered(self, lsb_octet_first: bool) -> [u8; 3] {
        let mut field: u32 =
            (1 << BIT_EXTENDED_ADDRESS) | (1 << BIT_TEST) | (1 << BIT_FCS16) | (1 << BIT_SYNC_TX);

        field |= match self.reject {
            RejectMode::ImplicitReject => 1 << BIT_REJ,
            RejectMode::SelectiveReject => 1 << BIT_SREJ,
        };
        field |= if self.modulo128 {
            1 << BIT_MODULO128
        } else {
            1 << BIT_MODULO8
        };

        if self.srej_multiframe {
            field |= 1 << BIT_SREJ_MULTIFRAME;
        }
        if self.segmenter_reassembler {
            field |= 1 << BIT_SEGMENTER;
        }

        if lsb_octet_first {
            // legacy (incorrect) least-significant octet first
            [
                (field & 0xFF) as u8,
                ((field >> 8) & 0xFF) as u8,
                ((field >> 16) & 0xFF) as u8,
            ]
        } else {
            // spec-correct most-significant octet first (§3.8; direwolf / BPQ)
            [
                ((field >> 16) & 0xFF) as u8,
                ((field >> 8) & 0xFF) as u8,
                (field & 0xFF) as u8,
            ]
        }
    }

    /// Decode from the (up to) 3-octet PV, most-significant octet first (§3.8).
    /// Octets beyond the first three are ignored. See [`Self::from_octets_ordered`].
    pub fn from_octets(pv: &[u8]) -> Self {
        Self::from_octets_ordered(pv, false)
    }

    /// Decode from the (up to) 3-octet PV with the octet order selectable. Reads
    /// the reject scheme from bits 1/2 and the modulo from bits 10/11; if a
    /// selection is ambiguous or absent it falls back to the spec defaults (SREJ,
    /// modulo 128). Mirrors C# `FromOctets(pv, bool lsbOctetFirst)`.
    pub fn from_octets_ordered(pv: &[u8], lsb_octet_first: bool) -> Self {
        let mut field: u32 = 0;
        let n = core::cmp::min(pv.len(), 3);
        for (i, &byte) in pv.iter().take(n).enumerate() {
            let shift = if lsb_octet_first {
                8 * i as u32
            } else {
                8 * (n - 1 - i) as u32
            };
            field |= (byte as u32) << shift;
        }

        let rej = (field & (1 << BIT_REJ)) != 0;
        let srej = (field & (1 << BIT_SREJ)) != 0;
        // SREJ takes precedence if both are (illegally) set; default SREJ if neither.
        let reject = if srej {
            RejectMode::SelectiveReject
        } else if rej {
            RejectMode::ImplicitReject
        } else {
            RejectMode::SelectiveReject
        };

        let mod128 = (field & (1 << BIT_MODULO128)) != 0;
        let mod8 = (field & (1 << BIT_MODULO8)) != 0;
        // Default modulo 128 if neither; mod-8 only if it alone is asserted.
        let is_mod128 = !(mod8 && !mod128);

        Self {
            reject,
            modulo128: is_mod128,
            srej_multiframe: (field & (1 << BIT_SREJ_MULTIFRAME)) != 0,
            segmenter_reassembler: (field & (1 << BIT_SEGMENTER)) != 0,
        }
    }
}

impl Default for HdlcOptionalFunctions {
    /// The AX.25 v2.2 default (SREJ + modulo 128). See [`Self::DEFAULT`].
    fn default() -> Self {
        Self::DEFAULT
    }
}
