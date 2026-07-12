//! The `V1|<seq>|<verb>|<args>` tuning-telegram line codec.
//!
//! Ports `Packet.Tune.Core.TuningTelegram` (`TuningTelegram.cs`) and its
//! `TuningVerb` enum. The canonical [`TuningTelegram::encode_into`] form is what
//! travels over a rich link; the [`TuningTelegram::encode_compact_into`] form
//! re-encodes `MS` (measurement) args with single-letter keys and, if still over
//! the [`SDM_CHARACTER_BUDGET`], drops the optional audio level so the telegram
//! fits one plain Tait SDM. [`TuningTelegram::try_parse`] accepts either form.
//!
//! `Args` is borrowed straight out of the input on decode (`&'a str`), so parsing
//! never allocates. The verbs whose argument bodies are themselves richer wire
//! forms (`MODE`, `HAIL`, `STAT`, `TXD`) are carried here only as their two/four
//! letter tokens; their argument codecs live in other `Packet.Tune.Core` types
//! and are out of scope for this codec.

use core::fmt::{self, Write};

use super::{CountWriter, MeterReport, SliceWriter};

/// The protocol version marker every telegram starts with.
pub const VERSION_PREFIX: &str = "V1";

/// The character budget of a plain Tait SDM — the compact wire form must fit
/// inside it.
pub const SDM_CHARACTER_BUDGET: usize = 32;

/// The verbs of the tuning-telegram protocol. The wire token of each is given by
/// [`TuningTelegram::verb_to_wire`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TuningVerb {
    /// `HI` — handshake/ready beacon (also the tuned end's "ready for the next
    /// burst" signal). Args = the sender's role.
    Hello,
    /// `RQ` — meter → tuned: transmit an n-frame burst now. Args = n.
    BurstRequest,
    /// `MS` — meter → tuned: the burst's measurement ([`MeterReport`] args).
    Measurement,
    /// `AD` — meter → tuned: advice for the human at the pot (`UP`/`DN`/`OK`).
    Advice,
    /// `BY` — end of session (no args).
    Bye,
    /// `MODE` — a mode-coordination message (args are the `ModeCoordMessage` wire
    /// form, opaque to this codec).
    ModeCoordination,
    /// `HAIL` — hailer → responder: "tell me your station status".
    Hail,
    /// `STAT` — responder → hailer: this station's status (may ride an extended
    /// SDM; args are the `StationStatus` wire form, opaque to this codec).
    Status,
    /// `TXD` — a TXDELAY-minimisation message (args are the `TxDelayMinMessage`
    /// wire form, opaque to this codec).
    TxDelay,
}

/// Every verb, in declaration order — for exhaustive round-trip tests and callers
/// that need to enumerate the protocol.
pub const ALL_VERBS: [TuningVerb; 9] = [
    TuningVerb::Hello,
    TuningVerb::BurstRequest,
    TuningVerb::Measurement,
    TuningVerb::Advice,
    TuningVerb::Bye,
    TuningVerb::ModeCoordination,
    TuningVerb::Hail,
    TuningVerb::Status,
    TuningVerb::TxDelay,
];

/// One tuning-protocol telegram: `V1|<seq>|<verb>|<args>` (the trailing `|args`
/// is omitted when `args` is empty). `Args` borrows from the parsed input.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TuningTelegram<'a> {
    /// Monotonic per-sender sequence number — the receiver dedupes on it
    /// (transport retries may deliver a telegram twice). Always text-encoded, and
    /// on the wire always non-negative.
    pub sequence: i32,
    /// The protocol verb.
    pub verb: TuningVerb,
    /// Verb-specific argument text (may be empty, e.g. `BY`).
    pub args: &'a str,
}

impl<'a> TuningTelegram<'a> {
    /// Construct a telegram from its fields.
    pub const fn new(sequence: i32, verb: TuningVerb, args: &'a str) -> Self {
        Self { sequence, verb, args }
    }

    /// The two/four-letter wire token for a verb. Mirrors `VerbToWire`.
    pub const fn verb_to_wire(verb: TuningVerb) -> &'static str {
        match verb {
            TuningVerb::Hello => "HI",
            TuningVerb::BurstRequest => "RQ",
            TuningVerb::Measurement => "MS",
            TuningVerb::Advice => "AD",
            TuningVerb::Bye => "BY",
            TuningVerb::ModeCoordination => "MODE",
            TuningVerb::Hail => "HAIL",
            TuningVerb::Status => "STAT",
            TuningVerb::TxDelay => "TXD",
        }
    }

    /// The verb for a wire token, or `None` if unknown. Mirrors `WireToVerb`.
    pub fn wire_to_verb(wire: &str) -> Option<TuningVerb> {
        Some(match wire {
            "HI" => TuningVerb::Hello,
            "RQ" => TuningVerb::BurstRequest,
            "MS" => TuningVerb::Measurement,
            "AD" => TuningVerb::Advice,
            "BY" => TuningVerb::Bye,
            "MODE" => TuningVerb::ModeCoordination,
            "HAIL" => TuningVerb::Hail,
            "STAT" => TuningVerb::Status,
            "TXD" => TuningVerb::TxDelay,
            _ => return None,
        })
    }

    /// Encode the canonical wire form `V1|seq|verb|args` (the trailing `|args` is
    /// omitted when `args` is empty) into a caller buffer. Returns bytes written,
    /// or `None` if the buffer is too small. Mirrors `TuningTelegram.Encode`.
    pub fn encode_into(&self, buf: &mut [u8]) -> Option<usize> {
        let mut w = SliceWriter::new(buf);
        self.write_encoded(&mut w).ok()?;
        Some(w.len())
    }

    /// Encode the compact SDM wire form into a caller buffer. Returns bytes
    /// written, or `None` if the buffer is too small. Mirrors
    /// `TuningTelegram.EncodeCompact`.
    pub fn encode_compact_into(&self, buf: &mut [u8]) -> Option<usize> {
        let mut w = SliceWriter::new(buf);
        self.write_compact(&mut w).ok()?;
        Some(w.len())
    }

    /// The canonical wire form as an owned `String`. Mirrors `TuningTelegram.Encode`.
    #[cfg(feature = "alloc")]
    pub fn encode(&self) -> alloc::string::String {
        let mut s = alloc::string::String::new();
        let _ = self.write_encoded(&mut s);
        s
    }

    /// The compact wire form as an owned `String`. Mirrors
    /// `TuningTelegram.EncodeCompact`.
    #[cfg(feature = "alloc")]
    pub fn encode_compact(&self) -> alloc::string::String {
        let mut s = alloc::string::String::new();
        let _ = self.write_compact(&mut s);
        s
    }

    /// Parse a telegram from its wire text (canonical or compact). Rejects
    /// anything not starting `V1|`, with fewer than three `|`-fields, a
    /// non-numeric sequence, or an unknown verb. `Args` borrows from `text`.
    /// Mirrors `TuningTelegram.TryParse`.
    pub fn try_parse(text: &'a str) -> Option<Self> {
        if text.is_empty() {
            return None;
        }

        // `splitn(4, '|')` mirrors the C# `Split('|', 4)`: at most four fields,
        // the fourth keeping any further '|' so args may itself contain pipes.
        let mut parts = text.splitn(4, '|');
        if parts.next()? != VERSION_PREFIX {
            return None;
        }
        let sequence = super::parse_nonneg_i32(parts.next()?)?;
        let verb = Self::wire_to_verb(parts.next()?)?;
        let args = parts.next().unwrap_or("");
        Some(Self { sequence, verb, args })
    }

    /// Render the canonical form into any writer.
    fn write_encoded<W: Write>(&self, w: &mut W) -> fmt::Result {
        write!(w, "{}|{}|{}", VERSION_PREFIX, self.sequence, Self::verb_to_wire(self.verb))?;
        if !self.args.is_empty() {
            write!(w, "|{}", self.args)?;
        }
        Ok(())
    }

    /// Render the compact form into any writer. Identical to the canonical form
    /// except that parseable `MS` args are re-encoded compactly, dropping the
    /// optional audio level if the result would otherwise bust the SDM budget.
    fn write_compact<W: Write>(&self, w: &mut W) -> fmt::Result {
        if self.verb == TuningVerb::Measurement {
            if let Some(report) = MeterReport::try_parse(self.args) {
                // Length of the with-level compact telegram: "V1|seq|MS|" + args.
                let head_len = self.compact_head_len();
                let full_len = head_len + report.compact_args_len();
                let report = if full_len > SDM_CHARACTER_BUDGET
                    && report.audio_level_db_tenths.is_some()
                {
                    report.without_audio_level()
                } else {
                    report
                };
                write!(w, "{}|{}|{}|", VERSION_PREFIX, self.sequence, Self::verb_to_wire(self.verb))?;
                return report.write_compact_args(w);
            }
        }
        self.write_encoded(w)
    }

    /// Rendered length of the compact `MS` head `V1|<seq>|MS|` (measured, not
    /// buffered).
    fn compact_head_len(&self) -> usize {
        let mut c = CountWriter::new();
        let _ = write!(c, "{}|{}|{}|", VERSION_PREFIX, self.sequence, Self::verb_to_wire(self.verb));
        c.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Encode the canonical form into a stack buffer as text (no `alloc` needed).
    fn enc<'a>(buf: &'a mut [u8], telegram: &TuningTelegram) -> &'a str {
        let n = telegram.encode_into(buf).unwrap();
        core::str::from_utf8(&buf[..n]).unwrap()
    }

    /// Encode the compact form into a stack buffer as text.
    fn enc_compact<'a>(buf: &'a mut [u8], telegram: &TuningTelegram) -> &'a str {
        let n = telegram.encode_compact_into(buf).unwrap();
        core::str::from_utf8(&buf[..n]).unwrap()
    }

    #[test]
    fn encodes_the_documented_wire_form() {
        let mut buf = [0u8; 64];
        assert_eq!(enc(&mut buf, &TuningTelegram::new(0, TuningVerb::Hello, "tuned")), "V1|0|HI|tuned");
        assert_eq!(enc(&mut buf, &TuningTelegram::new(3, TuningVerb::BurstRequest, "5")), "V1|3|RQ|5");
        assert_eq!(enc(&mut buf, &TuningTelegram::new(12, TuningVerb::Advice, "OK")), "V1|12|AD|OK");
        assert_eq!(enc(&mut buf, &TuningTelegram::new(99, TuningVerb::Bye, "")), "V1|99|BY");
    }

    #[test]
    fn parses_the_documented_wire_form() {
        let t = TuningTelegram::try_parse("V1|0|HI|tuned").unwrap();
        assert_eq!(t, TuningTelegram::new(0, TuningVerb::Hello, "tuned"));

        let t = TuningTelegram::try_parse("V1|3|RQ|5").unwrap();
        assert_eq!(t, TuningTelegram::new(3, TuningVerb::BurstRequest, "5"));

        let t = TuningTelegram::try_parse("V1|99|BY").unwrap();
        assert_eq!(t, TuningTelegram::new(99, TuningVerb::Bye, ""));

        // Args keep their embedded pipes (Split('|', 4)).
        let t = TuningTelegram::try_parse("V1|7|MS|4/5|fec:12|clip:0|rssi:-90.4").unwrap();
        assert_eq!(t.sequence, 7);
        assert_eq!(t.verb, TuningVerb::Measurement);
        assert_eq!(t.args, "4/5|fec:12|clip:0|rssi:-90.4");
    }

    #[test]
    fn rejects_junk() {
        assert!(TuningTelegram::try_parse("").is_none());
        assert!(TuningTelegram::try_parse("V2|0|HI|tuned").is_none());
        assert!(TuningTelegram::try_parse("V1|x|HI|tuned").is_none());
        assert!(TuningTelegram::try_parse("V1|0|ZZ|tuned").is_none());
        assert!(TuningTelegram::try_parse("hello world").is_none());
        assert!(TuningTelegram::try_parse("V1|0").is_none());
        // NumberStyles.None rejects a signed sequence.
        assert!(TuningTelegram::try_parse("V1|-1|HI|x").is_none());
    }

    #[test]
    fn every_verb_round_trips() {
        let mut buf = [0u8; 64];
        for verb in ALL_VERBS {
            let args = if verb == TuningVerb::Bye { "" } else { "x" };
            let original = TuningTelegram::new(42, verb, args);
            let wire = enc(&mut buf, &original);
            // Re-parse from an owned copy so the borrow of `buf` is released.
            let mut wire_copy = [0u8; 64];
            let len = wire.len();
            wire_copy[..len].copy_from_slice(wire.as_bytes());
            let text = core::str::from_utf8(&wire_copy[..len]).unwrap();
            assert_eq!(TuningTelegram::try_parse(text).unwrap(), original);
        }
    }

    #[test]
    fn compact_ms_fits_the_budget_and_round_trips() {
        // Canonical MS is too long for one SDM — the reason the compact form
        // exists (TuningTelegramTests.Compact_MS_fits_the_32_char_SDM_budget…).
        let report = MeterReport::new(10, 10, Some(480), Some(0), Some(-904), None);
        let mut args_buf = [0u8; 64];
        let args = {
            let n = report.to_args_into(&mut args_buf).unwrap();
            core::str::from_utf8(&args_buf[..n]).unwrap()
        };
        let telegram = TuningTelegram::new(12, TuningVerb::Measurement, args);

        let mut canon_buf = [0u8; 64];
        assert!(enc(&mut canon_buf, &telegram).len() > 32);

        let mut compact_buf = [0u8; 64];
        let compact = enc_compact(&mut compact_buf, &telegram);
        assert!(compact.len() <= SDM_CHARACTER_BUDGET);
        assert_eq!(compact, "V1|12|MS|10/10|f480|c0|r-90.4");

        let parsed = TuningTelegram::try_parse(compact).unwrap();
        assert_eq!(MeterReport::try_parse(parsed.args).unwrap(), report);
    }

    #[test]
    fn compact_leaves_non_ms_verbs_alone() {
        let telegram = TuningTelegram::new(1, TuningVerb::Hello, "meter");
        let mut a = [0u8; 64];
        let mut b = [0u8; 64];
        assert_eq!(enc_compact(&mut a, &telegram), enc(&mut b, &telegram));
    }

    #[test]
    fn compact_leaves_unparseable_ms_args_alone() {
        // MS verb but the args are not a MeterReport → falls through to canonical.
        let telegram = TuningTelegram::new(1, TuningVerb::Measurement, "not-a-report");
        let mut a = [0u8; 64];
        let mut b = [0u8; 64];
        assert_eq!(enc_compact(&mut a, &telegram), enc(&mut b, &telegram));
    }

    #[test]
    fn compact_ms_with_audio_level_fits_the_3_41_shape() {
        // fec absent, so level fits alongside clip + RSSI (3.41 meter shape).
        let report = MeterReport::new(5, 5, None, Some(0), Some(-901), Some(-625));
        let mut args_buf = [0u8; 64];
        let args = {
            let n = report.to_args_into(&mut args_buf).unwrap();
            core::str::from_utf8(&args_buf[..n]).unwrap()
        };
        let telegram = TuningTelegram::new(12, TuningVerb::Measurement, args);

        let mut compact_buf = [0u8; 64];
        let compact = enc_compact(&mut compact_buf, &telegram);
        assert!(compact.len() <= SDM_CHARACTER_BUDGET);
        assert_eq!(compact, "V1|12|MS|5/5|c0|r-90.1|l-62.5");
        let parsed = TuningTelegram::try_parse(compact).unwrap();
        assert_eq!(MeterReport::try_parse(parsed.args).unwrap(), report);
    }

    #[test]
    fn compact_drops_the_level_rather_than_busting_the_budget() {
        // All bracketing fields present AND a level: over 32 chars, so the level
        // (the enrichment) is dropped.
        let report = MeterReport::new(10, 10, Some(480), Some(0), Some(-904), Some(-625));
        let mut args_buf = [0u8; 96];
        let args = {
            let n = report.to_args_into(&mut args_buf).unwrap();
            core::str::from_utf8(&args_buf[..n]).unwrap()
        };
        let telegram = TuningTelegram::new(12, TuningVerb::Measurement, args);

        let mut compact_buf = [0u8; 64];
        let compact = enc_compact(&mut compact_buf, &telegram);
        assert!(compact.len() <= SDM_CHARACTER_BUDGET);

        let parsed = TuningTelegram::try_parse(compact).unwrap();
        let reparsed = MeterReport::try_parse(parsed.args).unwrap();
        assert_eq!(reparsed, report.without_audio_level());
    }

    #[test]
    fn encode_into_reports_none_on_undersized_buffer() {
        let telegram = TuningTelegram::new(0, TuningVerb::Hello, "tuned");
        let mut tiny = [0u8; 4];
        assert!(telegram.encode_into(&mut tiny).is_none());
    }

    #[cfg(feature = "alloc")]
    #[test]
    fn alloc_twins_match_the_into_forms() {
        let telegram = TuningTelegram::new(0, TuningVerb::Hello, "tuned");
        assert_eq!(telegram.encode(), "V1|0|HI|tuned");

        let report = MeterReport::new(10, 10, Some(480), Some(0), Some(-904), Some(-625));
        let args = report.to_args();
        let ms = TuningTelegram::new(12, TuningVerb::Measurement, &args);
        assert_eq!(ms.encode_compact(), "V1|12|MS|10/10|f480|c0|r-90.4");
    }
}
