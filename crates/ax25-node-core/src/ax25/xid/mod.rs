//! AX.25 v2.2 XID (Exchange Identification) information-field codec.
//!
//! Ports the `Packet.Ax25.Xid` namespace: the TLV parameter-negotiation payload
//! carried inside an XID U-frame (§4.3.3.7, Figure 4.5 / worked example Figure
//! 4.6) — the wire format the management data-link (MDL, App. C5) negotiates over.
//!
//! - [`info_field`] — the FI/GI/GL header + PI/PL/PV parameter codec
//!   ([`info_field::encode`] / [`info_field::parse`]).
//! - [`parameters::XidParameters`] — the decoded, semantic parameter set.
//! - [`classes_of_procedures::ClassesOfProcedures`] — PI=2 (duplex).
//! - [`hdlc_optional_functions`] — PI=3 (reject scheme + modulo + segmenter) and
//!   [`hdlc_optional_functions::RejectMode`].
//! - [`parse_options::XidParseOptions`] — spec-strict-by-default parse leniency.
//!
//! Byte-for-byte with the C# codec; see the per-module docs for spec citations.

pub mod classes_of_procedures;
pub mod hdlc_optional_functions;
pub mod info_field;
pub mod parameters;
pub mod parse_options;

pub use classes_of_procedures::ClassesOfProcedures;
pub use hdlc_optional_functions::{HdlcOptionalFunctions, RejectMode};
pub use parameters::XidParameters;
pub use parse_options::XidParseOptions;
