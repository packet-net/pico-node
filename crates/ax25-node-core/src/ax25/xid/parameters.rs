//! The decoded, semantic view of an XID information field's parameter set —
//! ports `Packet.Ax25.Xid.XidParameters`.
//!
//! Each field is `None` when the corresponding PI/PL/PV triple is *absent* from
//! the frame — which, per §4.3.3.7 ¶1024, means "use the currently-negotiated
//! value" rather than any particular default. This type is just the wire payload,
//! decoded; the negotiation (the `sdl` MDL machine) turns a command + response
//! pair into the agreed link parameters.
//!
//! Unit conventions match the wire format and the session context:
//! [`XidParameters::i_field_length_rx_bits`] is in **bits** (Figure 4.5's N1×8);
//! [`XidParameters::i_field_length_rx_octets`] converts to the N1 octet count.
//! [`XidParameters::ack_timer_millis`] is in milliseconds.
//! [`XidParameters::window_size_rx`] and [`XidParameters::retries`] are counts.

use super::classes_of_procedures::ClassesOfProcedures;
use super::hdlc_optional_functions::HdlcOptionalFunctions;

/// A decoded XID parameter set (§4.3.3.7, Figure 4.5). Byte-for-byte with C#
/// `XidParameters`; every field `None` ⇒ absent ("use current value").
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct XidParameters {
    /// Classes of Procedures (PI=2) — duplex selection. `None` if absent.
    pub classes_of_procedures: Option<ClassesOfProcedures>,
    /// HDLC Optional Functions (PI=3) — reject scheme + modulo + segmenter.
    /// `None` if absent.
    pub hdlc_optional_functions: Option<HdlcOptionalFunctions>,
    /// I Field Length Receive (PI=6), in **bits** (the wire unit, N1×8). `None`
    /// if absent.
    pub i_field_length_rx_bits: Option<u32>,
    /// Window Size Receive k (PI=8), in frames. `None` if absent.
    pub window_size_rx: Option<u32>,
    /// Acknowledge Timer T1 (PI=9), in milliseconds. `None` if absent.
    pub ack_timer_millis: Option<u32>,
    /// Retries N2 (PI=10), the retry count. `None` if absent.
    pub retries: Option<u32>,
}

impl XidParameters {
    /// [`Self::i_field_length_rx_bits`] converted to octets (N1). `None` if the
    /// field is absent. The wire value is bits; N1 in the session is octets, so
    /// we divide by 8.
    pub fn i_field_length_rx_octets(self) -> Option<u32> {
        self.i_field_length_rx_bits.map(|bits| bits / 8)
    }

    /// Build an N1 (I-field length, octets) value in the wire's bit unit —
    /// convenience for callers that think in octets (as the session does).
    /// Mirrors C# `XidParameters.OctetsToBits`.
    pub fn octets_to_bits(octets: u32) -> u32 {
        octets * 8
    }
}
