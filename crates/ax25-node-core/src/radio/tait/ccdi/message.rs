//! Typed CCDI messages (manual §1.10). Ports `Packet.Radio.Tait.Ccdi.CcdiMessage`
//! and `CcdiProgressType`.
//!
//! [`CcdiMessage::decode`] turns a validated [`CcdiFrame`] into its typed form,
//! borrowing the frame's parameter bytes (zero-copy, like the KISS classifier's
//! borrowed `InboundEvent`). Unrecognised idents — or recognised idents whose
//! parameters don't fit the expected shape — surface as [`CcdiMessage::Unknown`]
//! rather than being dropped, exactly as the C# `switch`'s `_` arm does.

use super::{parse_dec_u16, parse_hex_u8, parse_signed_i32, CcdiFrame};

/// A decoded CCDI message, borrowing its parameter slices from the source
/// [`CcdiFrame`]. Mirrors the `CcdiMessage` record hierarchy as one enum.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CcdiMessage<'a> {
    /// MODEL (§1.10.4) — radio type/model/tier plus the CCDI protocol version.
    Model {
        /// RUTYPE character.
        ru_type: u8,
        /// RUMODEL character.
        ru_model: u8,
        /// RUTIER character.
        ru_tier: u8,
        /// The CCDI protocol version string (e.g. `03.02`).
        ccdi_version: &'a [u8],
    },
    /// RADIO_SERIAL (§1.10.7).
    Serial(&'a [u8]),
    /// RADIO_VERSIONS (§1.10.8) — one record of the version inventory. Record
    /// numbers: 00 model name, 01 software, 02 database, 03 FPGA.
    Version {
        /// The two-character record number.
        record_number: &'a [u8],
        /// The version string for that record.
        version: &'a [u8],
    },
    /// CCTM_QUERY_RESULTS (§1.10.1) — the answer to a QUERY type-5 CCTM command.
    QueryResult {
        /// The CCTM command number (e.g. `64` for raw RSSI).
        cctm_command: u16,
        /// The raw value string; see [`CcdiMessage::query_rssi_tenths`].
        value: &'a [u8],
    },
    /// PROGRESS (§1.10.5) — unsolicited radio state-change notification.
    Progress {
        /// The progress type.
        ptype: CcdiProgressType,
        /// Any trailing parameter bytes.
        para: &'a [u8],
    },
    /// ERROR (§1.10.2). Category `0` = transaction error, `1` = system error.
    Error {
        /// The category character (`'0'` / `'1'`).
        category: u8,
        /// The error number (from two hex digits).
        error_number: u8,
    },
    /// RING (§1.10.9) — an incoming call. `ring_type` is the four-character
    /// [TYPE1..TYPE4] string; `status` is `FF` when no status value was received.
    Ring {
        /// The category character.
        category: u8,
        /// The four-character ring-type field.
        ring_type: &'a [u8],
        /// The two-character status field.
        status: &'a [u8],
        /// The caller-ID tail (may be empty).
        caller_id: &'a [u8],
    },
    /// GET_SDM (§1.10.3) — the buffered short data message (empty = none buffered).
    Sdm(&'a [u8]),
    /// QUERY_DISPLAY_RESPONSE (§1.10.6) — one element of a display-dump burst.
    Display {
        /// Element kind: `0` start, `F` end, `1` text object, `2` icon object.
        kind: u8,
        /// The element payload.
        payload: &'a [u8],
    },
    /// TDMA_DATA (§1.10.10, TM8200 only) — a received TDMA packet's raw data.
    TdmaData(&'a [u8]),
    /// CCR positive acknowledgement (§2.6): the command ident was accepted.
    CcrAck {
        /// The echoed command ident.
        echoed_command: u8,
    },
    /// CCR negative acknowledgement (§2.7). `echoed_command` is present only when
    /// the command's checksum was valid.
    CcrNak {
        /// The NAK reason code.
        reason: u8,
        /// The echoed command ident, if the checksum was valid.
        echoed_command: Option<u8>,
    },
    /// CCR unsolicited Selcall decode (§2.9.3): tones decoded from the channel.
    CcrSelcallDecode(&'a [u8]),
    /// CCR unsolicited notification (§2.9): `R` = CCR initialised, `P` = PTT
    /// approaching the transmit-timer limit.
    CcrNotification {
        /// The notification kind character.
        kind: u8,
    },
    /// CCR pulse response (§2.8.15): `true` = the radio has its minimum CCR
    /// configuration; `false` = still on defaults (power-cycled, forgot everything).
    CcrPulseResult {
        /// Whether the radio holds its minimum CCR configuration.
        has_minimum_configuration: bool,
    },
    /// Any message whose ident (or parameter shape) we don't decode — kept raw so
    /// nothing the radio says is invisible to a consumer.
    Unknown {
        /// The undecoded ident.
        ident: u8,
        /// The raw parameter bytes.
        params: &'a [u8],
    },
}

impl<'a> CcdiMessage<'a> {
    /// Decode a validated [`CcdiFrame`] into its typed message form, borrowing the
    /// frame's parameter bytes. Total: anything unrecognised becomes
    /// [`CcdiMessage::Unknown`].
    pub fn decode(frame: &'a CcdiFrame) -> CcdiMessage<'a> {
        let ident = frame.ident();
        let p = frame.params();
        let unknown = CcdiMessage::Unknown { ident, params: p };
        match ident {
            b'm' if p.len() >= 3 => CcdiMessage::Model {
                ru_type: p[0],
                ru_model: p[1],
                ru_tier: p[2],
                ccdi_version: &p[3..],
            },
            b'n' => CcdiMessage::Serial(p),
            b'v' if p.len() >= 2 => CcdiMessage::Version {
                record_number: &p[..2],
                version: &p[2..],
            },
            b'j' if p.len() >= 3 => match parse_dec_u16(&p[..3]) {
                Some(cctm) => CcdiMessage::QueryResult {
                    cctm_command: cctm,
                    value: &p[3..],
                },
                None => unknown,
            },
            b'p' if p.len() >= 2 => match parse_hex_u8(&p[..2]) {
                Some(pt) => CcdiMessage::Progress {
                    ptype: CcdiProgressType::from_u8(pt),
                    para: &p[2..],
                },
                None => unknown,
            },
            b'e' if p.len() >= 3 => match parse_hex_u8(&p[1..3]) {
                Some(err) => CcdiMessage::Error {
                    category: p[0],
                    error_number: err,
                },
                None => unknown,
            },
            b'r' if p.len() >= 7 => CcdiMessage::Ring {
                category: p[0],
                ring_type: &p[1..5],
                status: &p[5..7],
                caller_id: &p[7..],
            },
            b's' => CcdiMessage::Sdm(p),
            b'd' if !p.is_empty() => CcdiMessage::Display {
                kind: p[0],
                payload: &p[1..],
            },
            b'z' => CcdiMessage::TdmaData(p),
            b'+' if !p.is_empty() => CcdiMessage::CcrAck { echoed_command: p[0] },
            b'-' if p.len() >= 2 => match parse_hex_u8(&p[..2]) {
                Some(reason) => CcdiMessage::CcrNak {
                    reason,
                    echoed_command: if p.len() >= 3 { Some(p[2]) } else { None },
                },
                None => unknown,
            },
            b'V' => CcdiMessage::CcrSelcallDecode(p),
            b'M' if !p.is_empty() => CcdiMessage::CcrNotification { kind: p[0] },
            b'Q' if !p.is_empty() => CcdiMessage::CcrPulseResult {
                has_minimum_configuration: p[0] == b'P',
            },
            _ => unknown,
        }
    }

    /// For a [`CcdiMessage::QueryResult`] whose value is a signed integer already in
    /// 0.1 dB units (the CCTM 063/064 RSSI queries), return it as **tenths of a
    /// dBm** (`-456` → `-456` == −45.6 dBm). `None` for any other message, or a
    /// value that isn't a plain signed integer, or one outside `i16` range.
    ///
    /// This is the integer port of the C# `AsDecibels()` (which returned
    /// `tenths / 10f`): the no-FPU node keeps the raw tenths as an `i16`.
    pub fn query_rssi_tenths(&self) -> Option<i16> {
        match self {
            CcdiMessage::QueryResult { value, .. } => {
                i16::try_from(parse_signed_i32(value)?).ok()
            }
            _ => None,
        }
    }

    /// The value of a [`CcdiMessage::QueryResult`] as a plain signed integer (mV,
    /// temperature, …), or `None`. Mirrors the C# `AsInteger()`.
    pub fn query_value_int(&self) -> Option<i32> {
        match self {
            CcdiMessage::QueryResult { value, .. } => parse_signed_i32(value),
            _ => None,
        }
    }
}

/// PROGRESS message types (§1.10.5, `[PTYPE]`). Mirrors the C# `CcdiProgressType`
/// `enum : byte`, with an [`Other`](CcdiProgressType::Other) catch-all so decoding
/// any byte is total (the C# cast `(CcdiProgressType)ptype` accepts any value).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CcdiProgressType {
    /// A Selcall/Type-99 call was answered (0x00).
    CallAnswered,
    /// Deferred calling in progress (0x01).
    DeferredCalling,
    /// Transmission requested but inhibited (0x02).
    TxInhibited,
    /// Emergency mode initiated (0x03).
    EmergencyModeInitiated,
    /// Emergency mode terminated (0x04).
    EmergencyModeTerminated,
    /// RF detected on the current channel — hardware DCD rising edge (0x05).
    ReceiverBusy,
    /// RF no longer detected — hardware DCD falling edge (0x06).
    ReceiverNotBusy,
    /// PTT asserted — radio began transmitting (0x07).
    PttActivated,
    /// PTT released — radio stopped transmitting (0x08).
    PttDeactivated,
    /// Selcall retry (0x16).
    SelcallRetry,
    /// Radio stunned (0x17).
    RadioStunned,
    /// Radio revived (0x18).
    RadioRevived,
    /// Valid FFSK data received while in command mode (0x19).
    FfskDataReceived,
    /// Selcall auto-acknowledge status (0x1C).
    SelcallAutoAcknowledge,
    /// SDM auto-acknowledge status — SDM over-air delivery receipt (0x1D).
    SdmAutoAcknowledge,
    /// SDM GPS data received (0x1E).
    SdmGpsDataReceived,
    /// Radio restarted (0x1F).
    RadioRestarted,
    /// Single in-band tone received (0x20).
    SingleInBandToneReceived,
    /// User-initiated channel change (0x21).
    UserInitiatedChannelChange,
    /// TDMA channel event, TM8200 (0x22).
    TdmaChannel,
    /// Key action report (0x23).
    KeyAction,
    /// Channel-name report (0x31).
    ChannelName,
    /// Any PTYPE value the manual doesn't name — kept raw.
    Other(u8),
}

impl CcdiProgressType {
    /// Map a raw PTYPE byte to its variant. Total: unknown values become
    /// [`Self::Other`].
    pub const fn from_u8(value: u8) -> Self {
        match value {
            0x00 => Self::CallAnswered,
            0x01 => Self::DeferredCalling,
            0x02 => Self::TxInhibited,
            0x03 => Self::EmergencyModeInitiated,
            0x04 => Self::EmergencyModeTerminated,
            0x05 => Self::ReceiverBusy,
            0x06 => Self::ReceiverNotBusy,
            0x07 => Self::PttActivated,
            0x08 => Self::PttDeactivated,
            0x16 => Self::SelcallRetry,
            0x17 => Self::RadioStunned,
            0x18 => Self::RadioRevived,
            0x19 => Self::FfskDataReceived,
            0x1C => Self::SelcallAutoAcknowledge,
            0x1D => Self::SdmAutoAcknowledge,
            0x1E => Self::SdmGpsDataReceived,
            0x1F => Self::RadioRestarted,
            0x20 => Self::SingleInBandToneReceived,
            0x21 => Self::UserInitiatedChannelChange,
            0x22 => Self::TdmaChannel,
            0x23 => Self::KeyAction,
            0x31 => Self::ChannelName,
            other => Self::Other(other),
        }
    }

    /// The raw PTYPE byte for this variant.
    pub const fn to_u8(self) -> u8 {
        match self {
            Self::CallAnswered => 0x00,
            Self::DeferredCalling => 0x01,
            Self::TxInhibited => 0x02,
            Self::EmergencyModeInitiated => 0x03,
            Self::EmergencyModeTerminated => 0x04,
            Self::ReceiverBusy => 0x05,
            Self::ReceiverNotBusy => 0x06,
            Self::PttActivated => 0x07,
            Self::PttDeactivated => 0x08,
            Self::SelcallRetry => 0x16,
            Self::RadioStunned => 0x17,
            Self::RadioRevived => 0x18,
            Self::FfskDataReceived => 0x19,
            Self::SelcallAutoAcknowledge => 0x1C,
            Self::SdmAutoAcknowledge => 0x1D,
            Self::SdmGpsDataReceived => 0x1E,
            Self::RadioRestarted => 0x1F,
            Self::SingleInBandToneReceived => 0x20,
            Self::UserInitiatedChannelChange => 0x21,
            Self::TdmaChannel => 0x22,
            Self::KeyAction => 0x23,
            Self::ChannelName => 0x31,
            Self::Other(v) => v,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Each test parses a line into a `CcdiFrame` it keeps alive, then decodes —
    // `CcdiMessage` borrows the frame's parameter bytes, so the frame must outlive
    // the message (which is why decode isn't wrapped in a one-liner helper).

    #[test]
    fn model_message_decodes_type_model_tier_and_version() {
        let frame = CcdiFrame::try_parse(b"m0813203.02A2").unwrap();
        match CcdiMessage::decode(&frame) {
            CcdiMessage::Model {
                ru_type,
                ru_model,
                ru_tier,
                ccdi_version,
            } => {
                assert_eq!(ru_type, b'1');
                assert_eq!(ru_model, b'3');
                assert_eq!(ru_tier, b'2');
                assert_eq!(ccdi_version, b"03.02");
            }
            other => panic!("expected Model, got {other:?}"),
        }
    }

    #[test]
    fn query_result_decodes_rssi_tenths() {
        let frame = CcdiFrame::try_parse(b"j07064-456C9").unwrap();
        let msg = CcdiMessage::decode(&frame);
        match msg {
            CcdiMessage::QueryResult {
                cctm_command,
                value,
            } => {
                assert_eq!(cctm_command, 64);
                assert_eq!(value, b"-456");
            }
            other => panic!("expected QueryResult, got {other:?}"),
        }
        // -45.6 dBm carried as integer tenths.
        assert_eq!(msg.query_rssi_tenths(), Some(-456));
    }

    #[test]
    fn progress_decodes_receiver_busy_and_not_busy() {
        let busy = CcdiFrame::try_parse(b"p0205C9").unwrap();
        assert!(matches!(
            CcdiMessage::decode(&busy),
            CcdiMessage::Progress {
                ptype: CcdiProgressType::ReceiverBusy,
                ..
            }
        ));
        let idle = CcdiFrame::try_parse(b"p0206C8").unwrap();
        assert!(matches!(
            CcdiMessage::decode(&idle),
            CcdiMessage::Progress {
                ptype: CcdiProgressType::ReceiverNotBusy,
                ..
            }
        ));
    }

    #[test]
    fn error_decodes_category_and_number() {
        let frame = CcdiFrame::try_parse(b"e03001A7").unwrap();
        match CcdiMessage::decode(&frame) {
            CcdiMessage::Error {
                category,
                error_number,
            } => {
                assert_eq!(category, b'0');
                assert_eq!(error_number, 0x01);
            }
            other => panic!("expected Error, got {other:?}"),
        }
    }

    #[test]
    fn ring_message_decodes_manual_example() {
        // §1.10.9 example: r0714000FFA6 — an SDM call.
        let frame = CcdiFrame::try_parse(b"r0714000FFA6").unwrap();
        match CcdiMessage::decode(&frame) {
            CcdiMessage::Ring {
                category,
                ring_type,
                status,
                caller_id,
            } => {
                assert_eq!(category, b'1');
                assert_eq!(ring_type, b"4000");
                assert_eq!(status, b"FF");
                assert_eq!(caller_id, b"");
            }
            other => panic!("expected Ring, got {other:?}"),
        }
    }

    #[test]
    fn sdm_message_decodes_manual_examples() {
        // §1.10.3: s002D (no data), s02Hi7A (data "Hi").
        let empty = CcdiFrame::try_parse(b"s002D").unwrap();
        assert!(matches!(CcdiMessage::decode(&empty), CcdiMessage::Sdm(b"")));
        let hi = CcdiFrame::try_parse(b"s02Hi7A").unwrap();
        assert!(matches!(CcdiMessage::decode(&hi), CcdiMessage::Sdm(b"Hi")));
    }

    #[test]
    fn ccr_ack_nak_and_pulse_decode() {
        let ack = CcdiFrame::try_parse(b"+01R22").unwrap();
        assert!(matches!(
            CcdiMessage::decode(&ack),
            CcdiMessage::CcrAck { echoed_command: b'R' }
        ));

        // NAK reason 05 (busy) echoing command 'T'.
        let nak_wire = CcdiFrame::new(b'-', b"05T").unwrap().encode();
        let nak = CcdiFrame::try_parse(&nak_wire).unwrap();
        match CcdiMessage::decode(&nak) {
            CcdiMessage::CcrNak {
                reason,
                echoed_command,
            } => {
                assert_eq!(reason, 0x05);
                assert_eq!(echoed_command, Some(b'T'));
            }
            other => panic!("expected CcrNak, got {other:?}"),
        }

        let pulse = CcdiFrame::try_parse(b"Q01PFE").unwrap();
        assert!(matches!(
            CcdiMessage::decode(&pulse),
            CcdiMessage::CcrPulseResult {
                has_minimum_configuration: true
            }
        ));
    }

    #[test]
    fn ccr_unsolicited_messages_decode() {
        let init = CcdiFrame::try_parse(b"M01R00").unwrap();
        assert!(matches!(
            CcdiMessage::decode(&init),
            CcdiMessage::CcrNotification { kind: b'R' }
        ));
        let selcall = CcdiFrame::try_parse(b"V0612345-18").unwrap();
        assert!(matches!(
            CcdiMessage::decode(&selcall),
            CcdiMessage::CcrSelcallDecode(b"12345-")
        ));
    }

    #[test]
    fn unknown_ident_surfaces_as_unknown() {
        let wire = CcdiFrame::new(b'y', b"Hi").unwrap().encode();
        let frame = CcdiFrame::try_parse(&wire).unwrap();
        assert!(matches!(
            CcdiMessage::decode(&frame),
            CcdiMessage::Unknown {
                ident: b'y',
                params: b"Hi"
            }
        ));
    }

    #[test]
    fn progress_type_round_trips_including_other() {
        for raw in 0u8..=0xFF {
            assert_eq!(CcdiProgressType::from_u8(raw).to_u8(), raw);
        }
    }
}
