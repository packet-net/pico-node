//! Codec for the AX.25 v2.2 XID (Exchange Identification) *information field* —
//! ports `Packet.Ax25.Xid.XidInfoField`.
//!
//! The TLV parameter-negotiation payload carried inside an XID U-frame (§4.3.3.7,
//! parameter table Figure 4.5, worked example Figure 4.6). The resulting bytes go
//! into an XID frame's info field; bytes pulled off a received XID frame come back
//! here.
//!
//! ```text
//!   FI (1)  Format Identifier  = 0x82 (general-purpose XID information)
//!   GI (1)  Group Identifier   = 0x80 (parameter-negotiation identifier)
//!   GL (2)  Group Length       = length of the parameter field that follows,
//!                                big-endian, NOT counting FI/GI/GL themselves
//!   parameter field: a run of PI/PL/PV triples in ascending PI order
//!     PI (1)  Parameter Identifier
//!     PL (1)  Parameter Length  = length of PV in octets (excludes PI and PL)
//!     PV (PL) Parameter Value
//! ```
//!
//! A `PL` of zero means the PV is absent and the parameter takes its default; an
//! omitted PI/PL/PV triple means "use the currently-negotiated value"; an
//! unrecognised PI is ignored (§4.3.3.7 ¶1024). We model "absent" as a `None`
//! field on [`XidParameters`], distinct from a present-but-default value.
//!
//! **Strict by construction.** [`encode`] emits exactly the fields set on
//! [`XidParameters`], in ascending-PI order, with the fixed/reserved bits forced
//! to their spec-mandated constants. Parser leniency lives behind named flags on
//! [`XidParseOptions`]; the default is spec-strict.
//!
//! `no_std` + `alloc`: [`encode_into`] is the zero-alloc primary; [`encode`]
//! allocates the returned `Vec`.

extern crate alloc;
use alloc::vec::Vec;

use super::parameters::XidParameters;
use super::parse_options::XidParseOptions;
use super::{ClassesOfProcedures, HdlcOptionalFunctions};

/// Format Identifier for general-purpose XID information (§4.3.3.7 ¶1019).
pub const FORMAT_IDENTIFIER: u8 = 0x82;
/// Group Identifier for the parameter-negotiation group (§4.3.3.7 ¶1020).
pub const GROUP_IDENTIFIER: u8 = 0x80;
/// Minimum encoded length: FI + GI + GL with an empty parameter field.
pub const HEADER_LENGTH: usize = 4;

/// PI=2 — Classes of Procedures (half/full duplex, ABM). Figure 4.5.
pub const PI_CLASSES_OF_PROCEDURES: u8 = 0x02;
/// PI=3 — HDLC Optional Functions (REJ/SREJ, modulo, segmenter, …). Figure 4.5.
pub const PI_HDLC_OPTIONAL_FUNCTIONS: u8 = 0x03;
/// PI=5 — I Field Length Transmit (bits). ISO 8885; not negotiated by AX.25.
pub const PI_I_FIELD_LENGTH_TX: u8 = 0x05;
/// PI=6 — I Field Length Receive, in **bits** (N1×8). Figure 4.5.
pub const PI_I_FIELD_LENGTH_RX: u8 = 0x06;
/// PI=7 — Window Size Transmit. ISO 8885; not negotiated by AX.25.
pub const PI_WINDOW_SIZE_TX: u8 = 0x07;
/// PI=8 — Window Size Receive (k frames). Figure 4.5.
pub const PI_WINDOW_SIZE_RX: u8 = 0x08;
/// PI=9 — Acknowledge Timer T1, in milliseconds. Figure 4.5.
pub const PI_ACK_TIMER: u8 = 0x09;
/// PI=10 (0x0A) — Retries (N2). Figure 4.6 labels this "Retries (N2)".
pub const PI_RETRIES: u8 = 0x0A;

/// A fixed upper bound on an encoded XID info field: header (4) + Classes (4) +
/// HDLC (5) + N1 Rx (2+4) + window Rx (3) + T1 (2+4) + N2 (2+4) = 34, rounded up.
const MAX_ENCODED_LEN: usize = 64;

/// Encode a set of negotiation parameters into the XID information-field bytes
/// (FI + GI + GL + ordered PI/PL/PV). Only the non-`None` fields of `parameters`
/// are emitted, in ascending PI order per §4.3.3.7 ¶1024. Mirrors C#
/// `XidInfoField.Encode`.
pub fn encode(parameters: &XidParameters) -> Vec<u8> {
    let mut buf = [0u8; MAX_ENCODED_LEN];
    let n = encode_into(parameters, &mut buf).expect("XID info field fits the fixed buffer");
    buf[..n].to_vec()
}

/// Zero-alloc encode: write the XID information field into `dst`, returning the
/// number of octets written, or `None` if `dst` is too small. Same wire bytes as
/// [`encode`].
pub fn encode_into(parameters: &XidParameters, dst: &mut [u8]) -> Option<usize> {
    if dst.len() < HEADER_LENGTH {
        return None;
    }
    let mut off = HEADER_LENGTH;

    if let Some(cop) = parameters.classes_of_procedures {
        // PI=2, PL=2, PV = 16-bit field (LSB-first within each octet).
        push_parameter(dst, &mut off, PI_CLASSES_OF_PROCEDURES, &cop.to_octets())?;
    }
    if let Some(hof) = parameters.hdlc_optional_functions {
        // PI=3, PL=3, PV = 24-bit field, most-significant octet first (§3.8).
        push_parameter(dst, &mut off, PI_HDLC_OPTIONAL_FUNCTIONS, &hof.to_octets())?;
    }
    if let Some(bits) = parameters.i_field_length_rx_bits {
        let be = bits.to_be_bytes();
        push_parameter(dst, &mut off, PI_I_FIELD_LENGTH_RX, minimal_be(&be))?;
    }
    if let Some(k) = parameters.window_size_rx {
        // Window size is a single-octet count 0..127 (Figure 4.5: bits 0–6).
        push_parameter(dst, &mut off, PI_WINDOW_SIZE_RX, &[(k & 0x7F) as u8])?;
    }
    if let Some(t1) = parameters.ack_timer_millis {
        let be = t1.to_be_bytes();
        push_parameter(dst, &mut off, PI_ACK_TIMER, minimal_be(&be))?;
    }
    if let Some(n2) = parameters.retries {
        let be = n2.to_be_bytes();
        push_parameter(dst, &mut off, PI_RETRIES, minimal_be(&be))?;
    }

    let group_length = (off - HEADER_LENGTH) as u16;
    dst[0] = FORMAT_IDENTIFIER;
    dst[1] = GROUP_IDENTIFIER;
    dst[2] = (group_length >> 8) as u8;
    dst[3] = (group_length & 0xFF) as u8;
    Some(off)
}

/// Parse an XID information field into a [`XidParameters`], spec-strict. Returns
/// `None` on a malformed buffer. Mirrors C#
/// `XidInfoField.TryParse(info, out)` (the strict default).
pub fn parse(info: &[u8]) -> Option<XidParameters> {
    parse_with(info, &XidParseOptions::STRICT)
}

/// Parse an XID information field applying the supplied [`XidParseOptions`].
/// Returns `None` (without panicking) on a malformed buffer — a bad FI/GI, a
/// truncated header, a Group Length that overruns the buffer, or (under strict) a
/// PI/PL whose PV runs past the parameter field. Unrecognised PIs are skipped per
/// §4.3.3.7 ¶1024. Mirrors C# `XidInfoField.TryParse(info, options, out)`.
pub fn parse_with(info: &[u8], options: &XidParseOptions) -> Option<XidParameters> {
    if info.len() < HEADER_LENGTH {
        return None;
    }
    if info[0] != FORMAT_IDENTIFIER {
        return None;
    }
    if info[1] != GROUP_IDENTIFIER {
        return None;
    }

    let mut group_length = ((info[2] as usize) << 8) | info[3] as usize;
    let available = info.len() - HEADER_LENGTH;

    if group_length > available {
        // GL claims more parameter bytes than the buffer holds.
        if !options.allow_group_length_overrun {
            return None;
        }
        group_length = available; // lenient: clamp to what we actually have
    }

    let pf = &info[HEADER_LENGTH..HEADER_LENGTH + group_length];

    let mut params = XidParameters::default();

    let mut pos = 0usize;
    while pos < pf.len() {
        let pi = pf[pos];
        pos += 1;
        if pos >= pf.len() {
            // A trailing PI with no room for a PL octet.
            if !options.allow_truncated_parameter {
                return None;
            }
            break;
        }

        let mut pl = pf[pos] as usize;
        pos += 1;
        if pos + pl > pf.len() {
            // PV runs past the end of the parameter field.
            if !options.allow_truncated_parameter {
                return None;
            }
            pl = pf.len() - pos; // lenient: take what remains
        }

        let pv = &pf[pos..pos + pl];
        pos += pl;

        // A `PL=0` PV (guard `pl >= 1` fails) falls to the no-op arm ⇒ the field
        // stays `None` — "absent, take default" per §4.3.3.7 ¶1024, matching C#.
        match pi {
            PI_CLASSES_OF_PROCEDURES if pl >= 1 => {
                let octet0 = pv[0];
                let octet1 = if pl >= 2 { pv[1] } else { 0 };
                params.classes_of_procedures =
                    Some(ClassesOfProcedures::from_octets(octet0, octet1));
            }
            PI_HDLC_OPTIONAL_FUNCTIONS if pl >= 1 => {
                params.hdlc_optional_functions = Some(HdlcOptionalFunctions::from_octets(pv));
            }
            PI_I_FIELD_LENGTH_RX if pl >= 1 => {
                params.i_field_length_rx_bits = Some(decode_unsigned(pv));
            }
            PI_WINDOW_SIZE_RX if pl >= 1 => {
                params.window_size_rx = Some((pv[0] & 0x7F) as u32);
            }
            PI_ACK_TIMER if pl >= 1 => {
                params.ack_timer_millis = Some(decode_unsigned(pv));
            }
            PI_RETRIES if pl >= 1 => {
                params.retries = Some(decode_unsigned(pv));
            }
            // PI=5 / PI=7 (Tx variants), a PL=0 triple, and any unrecognised PI
            // are ignored per §4.3.3.7 ¶1024.
            _ => {}
        }
    }

    Some(params)
}

/// Write a `PI/PL/PV` triple to `dst` at `*off`, advancing it. Returns `None` if
/// the buffer overruns.
fn push_parameter(dst: &mut [u8], off: &mut usize, pi: u8, pv: &[u8]) -> Option<()> {
    let end = *off + 2 + pv.len();
    if end > dst.len() {
        return None;
    }
    dst[*off] = pi;
    dst[*off + 1] = pv.len() as u8;
    dst[*off + 2..end].copy_from_slice(pv);
    *off = end;
    Some(())
}

/// The minimal big-endian representation of a 4-octet big-endian value: strip
/// leading zero octets, keeping at least one octet (so `0` ⇒ `[0]`). Mirrors C#
/// `EncodeUnsigned` — Type-B numeric fields are variable-length big-endian.
fn minimal_be(be: &[u8; 4]) -> &[u8] {
    let first = be.iter().position(|&b| b != 0).unwrap_or(be.len() - 1);
    &be[first..]
}

/// Decode a big-endian Type-B numeric field of arbitrary octet width, saturating
/// at `i32::MAX` on pathological widths (matching C# `DecodeUnsigned`, which
/// returns an `int`).
fn decode_unsigned(pv: &[u8]) -> u32 {
    let mut acc: u64 = 0;
    for &b in pv {
        acc = (acc << 8) | b as u64;
        if acc > i32::MAX as u64 {
            acc = i32::MAX as u64;
        }
    }
    acc as u32
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::RejectMode;

    // The information field from Figure 4.6 (NJ7P → N7LEM), GL = 0x17 (23 octets).
    // NOTE on the HDLC PV (P3 = 03 03 22 A8 82): §3.8 sends multiple-octet fields
    // HIGH-ORDER OCTET FIRST, so the same logical selection Figure 4.6 prints as
    // `82 A8 22` (LSB-first, §3.8-violating) serialises here as `22 A8 82`.
    const FIGURE_46_INFO: [u8; 27] = [
        0x82, 0x80, 0x00, 0x17, //
        0x02, 0x02, 0x22, 0x00, //
        0x03, 0x03, 0x22, 0xA8, 0x82, //
        0x06, 0x02, 0x04, 0x00, //
        0x08, 0x01, 0x02, //
        0x09, 0x02, 0x10, 0x00, //
        0x0A, 0x01, 0x03,
    ];

    #[test]
    fn header_constants_match_spec() {
        assert_eq!(FORMAT_IDENTIFIER, 0x82);
        assert_eq!(GROUP_IDENTIFIER, 0x80);
        assert_eq!(PI_CLASSES_OF_PROCEDURES, 2);
        assert_eq!(PI_HDLC_OPTIONAL_FUNCTIONS, 3);
        assert_eq!(PI_I_FIELD_LENGTH_RX, 6);
        assert_eq!(PI_WINDOW_SIZE_RX, 8);
        assert_eq!(PI_ACK_TIMER, 9);
        assert_eq!(PI_RETRIES, 0x0A);
    }

    #[test]
    fn parses_figure_4_6_worked_example() {
        let p = parse(&FIGURE_46_INFO).expect("figure 4.6 parses");

        // Classes of Procedures: PV 0x22 0x00 ⇒ ABM + half-duplex.
        assert!(p.classes_of_procedures.unwrap().half_duplex);

        // HDLC: PV 22 A8 82 (MSB-first) ⇒ REJ (bit 1) + mod-128 (bit 11) +
        // SREJ-multiframe (bit 21) + the always-1 bits. (Fig 4.6's caption says
        // SREJ; the bytes select REJ — the caption is loose.)
        let hof = p.hdlc_optional_functions.unwrap();
        assert_eq!(hof.reject, RejectMode::ImplicitReject);
        assert!(hof.modulo128);
        assert!(hof.srej_multiframe);
        assert!(!hof.segmenter_reassembler);

        // N1 Rx: PV 0x04 0x00 = 1024 bits = 128 octets.
        assert_eq!(p.i_field_length_rx_bits, Some(1024));
        assert_eq!(p.i_field_length_rx_octets(), Some(128));
        // Window k Rx: PV 0x02 = 2 frames.
        assert_eq!(p.window_size_rx, Some(2));
        // T1: PV 0x10 0x00 = 4096 ms.
        assert_eq!(p.ack_timer_millis, Some(4096));
        // N2: PV 0x03 = 3 retries.
        assert_eq!(p.retries, Some(3));
    }

    #[test]
    fn encode_reproduces_figure_4_6_except_for_figure_abm_anomaly() {
        // The parameters the Figure 4.6 bytes encode.
        let params = XidParameters {
            classes_of_procedures: Some(ClassesOfProcedures::HALF_DUPLEX_DEFAULT),
            hdlc_optional_functions: Some(HdlcOptionalFunctions {
                reject: RejectMode::ImplicitReject,
                modulo128: true,
                srej_multiframe: true,
                segmenter_reassembler: false,
            }),
            i_field_length_rx_bits: Some(1024),
            window_size_rx: Some(2),
            ack_timer_millis: Some(4096),
            retries: Some(3),
        };

        let encoded = encode(&params);

        // KNOWN SPEC DEFECT: the table/prose put Balanced-ABM at bit 0 (0x21);
        // Fig 4.6's byte index 6 is 0x22 (its off-by-one). We follow the table.
        const ABM_ANOMALY_INDEX: usize = 6;
        assert_eq!(encoded[ABM_ANOMALY_INDEX], 0x21);
        assert_eq!(FIGURE_46_INFO[ABM_ANOMALY_INDEX], 0x22);

        // Splice the figure's anomalous byte in and the rest must match exactly.
        let mut with_figure_abm = encoded.clone();
        with_figure_abm[ABM_ANOMALY_INDEX] = 0x22;
        assert_eq!(with_figure_abm.as_slice(), &FIGURE_46_INFO[..]);
    }

    #[test]
    fn encode_empty_parameters_emits_bare_header_with_zero_group_length() {
        assert_eq!(encode(&XidParameters::default()), alloc::vec![0x82, 0x80, 0x00, 0x00]);
    }

    #[test]
    fn encode_sets_group_length_to_parameter_field_length_only() {
        let bytes = encode(&XidParameters {
            window_size_rx: Some(7),
            ..Default::default()
        });
        assert_eq!(bytes[0], 0x82);
        assert_eq!(bytes[1], 0x80);
        assert_eq!(bytes[2], 0x00);
        assert_eq!(bytes[3], 0x03); // GL counts only the 3 PI/PL/PV bytes
        assert_eq!(&bytes[4..], &[0x08, 0x01, 0x07]);
    }

    #[test]
    fn encode_orders_parameters_by_ascending_pi() {
        let bytes = encode(&XidParameters {
            retries: Some(5),                                            // PI 0x0A
            classes_of_procedures: Some(ClassesOfProcedures::HALF_DUPLEX_DEFAULT), // PI 0x02
            window_size_rx: Some(4),                                     // PI 0x08
            ack_timer_millis: Some(3000),                                // PI 0x09
            ..Default::default()
        });

        let mut pis = Vec::new();
        let mut pos = HEADER_LENGTH;
        while pos < bytes.len() {
            pis.push(bytes[pos]);
            let pl = bytes[pos + 1] as usize;
            pos += 2 + pl;
        }
        assert_eq!(pis, alloc::vec![0x02, 0x08, 0x09, 0x0A]);
    }

    #[test]
    fn roundtrip_classes_of_procedures_duplex() {
        for half_duplex in [true, false] {
            let p = XidParameters {
                classes_of_procedures: Some(ClassesOfProcedures { half_duplex }),
                ..Default::default()
            };
            let got = parse(&encode(&p)).unwrap();
            assert_eq!(got.classes_of_procedures.unwrap().half_duplex, half_duplex);
        }
    }

    #[test]
    fn classes_of_procedures_always_sets_abm_bit() {
        // bit 0 (ABM) always 1; half-duplex sets bit 5 ⇒ 0x21; full ⇒ 0x41.
        assert_eq!(ClassesOfProcedures::HALF_DUPLEX_DEFAULT.to_octets(), [0x21, 0x00]);
        assert_eq!(ClassesOfProcedures::FULL_DUPLEX_CAPABLE.to_octets(), [0x41, 0x00]);
    }

    #[test]
    fn roundtrip_hdlc_reject_and_modulo() {
        for reject in [RejectMode::ImplicitReject, RejectMode::SelectiveReject] {
            for mod128 in [true, false] {
                let p = XidParameters {
                    hdlc_optional_functions: Some(HdlcOptionalFunctions {
                        reject,
                        modulo128: mod128,
                        srej_multiframe: false,
                        segmenter_reassembler: false,
                    }),
                    ..Default::default()
                };
                let got = parse(&encode(&p)).unwrap();
                let hof = got.hdlc_optional_functions.unwrap();
                assert_eq!(hof.reject, reject);
                assert_eq!(hof.modulo128, mod128);
            }
        }
    }

    #[test]
    fn hdlc_forces_always_one_bits() {
        // ToOctets serialises MSB-octet first (octets[0] is bits 16-23); rebuild
        // the 24-bit field to check the (order-independent) bit positions.
        let octets = HdlcOptionalFunctions::DEFAULT.to_octets();
        let field: u32 = ((octets[0] as u32) << 16) | ((octets[1] as u32) << 8) | octets[2] as u32;
        assert_eq!((field >> 7) & 1, 1, "bit 7 extended address always 1");
        assert_eq!((field >> 13) & 1, 1, "bit 13 TEST always 1");
        assert_eq!((field >> 15) & 1, 1, "bit 15 16-bit FCS always 1");
        assert_eq!((field >> 17) & 1, 1, "bit 17 synchronous Tx always 1");
        assert_eq!((field >> 1) & 1, 0, "SREJ selected ⇒ bit 1 (REJ) reset");
        assert_eq!((field >> 2) & 1, 1, "SREJ selected ⇒ bit 2 set");
        assert_eq!((field >> 10) & 1, 0, "mod128 ⇒ bit 10 (mod8) reset");
        assert_eq!((field >> 11) & 1, 1, "mod128 ⇒ bit 11 set");
    }

    #[test]
    fn roundtrip_hdlc_segmenter_and_srej_multiframe() {
        let p = XidParameters {
            hdlc_optional_functions: Some(HdlcOptionalFunctions {
                reject: RejectMode::SelectiveReject,
                modulo128: true,
                srej_multiframe: true,
                segmenter_reassembler: true,
            }),
            ..Default::default()
        };
        let got = parse(&encode(&p)).unwrap().hdlc_optional_functions.unwrap();
        assert!(got.srej_multiframe);
        assert!(got.segmenter_reassembler);
    }

    #[test]
    fn roundtrip_i_field_length_rx_bits() {
        for bits in [2048u32, 1024, 8, 65535] {
            let p = XidParameters {
                i_field_length_rx_bits: Some(bits),
                ..Default::default()
            };
            let got = parse(&encode(&p)).unwrap();
            assert_eq!(got.i_field_length_rx_bits, Some(bits));
            assert_eq!(got.i_field_length_rx_octets(), Some(bits / 8));
        }
    }

    #[test]
    fn roundtrip_window_size_rx() {
        for k in [0u32, 4, 32, 127] {
            let p = XidParameters {
                window_size_rx: Some(k),
                ..Default::default()
            };
            assert_eq!(parse(&encode(&p)).unwrap().window_size_rx, Some(k));
        }
    }

    #[test]
    fn roundtrip_ack_timer() {
        for millis in [3000u32, 4096, 255, 60000] {
            let p = XidParameters {
                ack_timer_millis: Some(millis),
                ..Default::default()
            };
            assert_eq!(parse(&encode(&p)).unwrap().ack_timer_millis, Some(millis));
        }
    }

    #[test]
    fn roundtrip_retries() {
        for n2 in [1u32, 10, 255] {
            let p = XidParameters {
                retries: Some(n2),
                ..Default::default()
            };
            assert_eq!(parse(&encode(&p)).unwrap().retries, Some(n2));
        }
    }

    #[test]
    fn roundtrip_all_parameters_together() {
        let p = XidParameters {
            classes_of_procedures: Some(ClassesOfProcedures::FULL_DUPLEX_CAPABLE),
            hdlc_optional_functions: Some(HdlcOptionalFunctions {
                reject: RejectMode::SelectiveReject,
                modulo128: true,
                srej_multiframe: false,
                segmenter_reassembler: true,
            }),
            i_field_length_rx_bits: Some(XidParameters::octets_to_bits(256)),
            window_size_rx: Some(32),
            ack_timer_millis: Some(3000),
            retries: Some(10),
        };
        assert_eq!(parse(&encode(&p)), Some(p));
    }

    #[test]
    fn absent_fields_parse_as_none_not_default() {
        let got = parse(&encode(&XidParameters {
            window_size_rx: Some(4),
            ..Default::default()
        }))
        .unwrap();
        assert_eq!(got.window_size_rx, Some(4));
        assert!(got.classes_of_procedures.is_none());
        assert!(got.hdlc_optional_functions.is_none());
        assert!(got.i_field_length_rx_bits.is_none());
        assert!(got.ack_timer_millis.is_none());
        assert!(got.retries.is_none());
    }

    #[test]
    fn empty_parameter_field_parses_to_all_none() {
        let got = parse(&[0x82, 0x80, 0x00, 0x00]).unwrap();
        assert_eq!(got, XidParameters::default());
    }

    #[test]
    fn zero_length_pv_is_absent_parameter() {
        // PL=0 ⇒ PV absent ⇒ field stays None (¶1024).
        let info = [0x82, 0x80, 0x00, 0x02, PI_WINDOW_SIZE_RX, 0x00];
        assert!(parse(&info).unwrap().window_size_rx.is_none());
    }

    #[test]
    fn unrecognised_pi_is_skipped() {
        let info = [
            0x82, 0x80, 0x00, 0x07, //
            0x42, 0x02, 0xDE, 0xAD, // unknown PI, skipped
            0x08, 0x01, 0x05, // window k = 5
        ];
        assert_eq!(parse(&info).unwrap().window_size_rx, Some(5));
    }

    #[test]
    fn tx_variants_pi5_pi7_are_skipped() {
        let info = [
            0x82, 0x80, 0x00, 0x0A, //
            0x05, 0x02, 0x08, 0x00, // PI=5 Tx N1 — skipped
            0x07, 0x01, 0x10, // PI=7 Tx window — skipped
            0x08, 0x01, 0x05, // PI=8 Rx window = 5
        ];
        assert_eq!(parse(&info).unwrap().window_size_rx, Some(5));
    }

    #[test]
    fn parse_rejects_short_header() {
        assert!(parse(&[]).is_none());
        assert!(parse(&[0x82]).is_none());
        assert!(parse(&[0x82, 0x80, 0x00]).is_none());
    }

    #[test]
    fn parse_rejects_wrong_format_or_group_identifier() {
        assert!(parse(&[0x81, 0x80, 0x00, 0x00]).is_none());
        assert!(parse(&[0x82, 0x81, 0x00, 0x00]).is_none());
    }

    #[test]
    fn strict_rejects_group_length_overrun_but_lenient_clamps() {
        // GL claims 8 parameter bytes; only 3 follow.
        let info = [0x82, 0x80, 0x00, 0x08, 0x08, 0x01, 0x05];
        assert!(parse_with(&info, &XidParseOptions::STRICT).is_none());
        let got = parse_with(&info, &XidParseOptions::LENIENT).unwrap();
        assert_eq!(got.window_size_rx, Some(5));
    }

    #[test]
    fn strict_rejects_truncated_parameter_but_lenient_tolerates() {
        // GL=4: a window param (3 bytes) then a stray PI 0x09 with no PL octet.
        let info = [0x82, 0x80, 0x00, 0x04, 0x08, 0x01, 0x05, 0x09];
        assert!(parse_with(&info, &XidParseOptions::STRICT).is_none());
        let got = parse_with(&info, &XidParseOptions::LENIENT).unwrap();
        assert_eq!(got.window_size_rx, Some(5));
    }

    #[test]
    fn strict_rejects_pv_longer_than_remaining_but_lenient_truncates() {
        // GL=4: PI 0x09 (T1) PL=3 but only 1 PV byte before the field ends.
        let info = [0x82, 0x80, 0x00, 0x04, 0x09, 0x03, 0x10];
        assert!(parse_with(&info, &XidParseOptions::STRICT).is_none());
        let got = parse_with(&info, &XidParseOptions::LENIENT).unwrap();
        assert_eq!(got.ack_timer_millis, Some(0x10)); // 1 available octet ⇒ 16
    }

    #[test]
    fn encode_into_matches_encode_and_reports_too_small() {
        let p = XidParameters {
            hdlc_optional_functions: Some(HdlcOptionalFunctions::DEFAULT),
            window_size_rx: Some(7),
            ..Default::default()
        };
        let want = encode(&p);
        let mut buf = [0u8; MAX_ENCODED_LEN];
        let n = encode_into(&p, &mut buf).unwrap();
        assert_eq!(&buf[..n], want.as_slice());
        // A buffer that can't even hold the header fails.
        assert!(encode_into(&p, &mut [0u8; 3]).is_none());
    }
}
