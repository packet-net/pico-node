//! The meter end's per-burst measurement — the payload of an `MS` telegram.
//!
//! Ports `Packet.Tune.Core.MeterReport` (`MeterReport.cs`). Two wire forms:
//! the canonical `dec/n|fec:<Δ>|clip:<Δ>|rssi:<dBm>|lvl:<dB>` (with `na` for
//! unavailable values) and the single-letter compact form `dec/n|f<Δ>|c<Δ>|r<dBm>|l<dB>`
//! that omits unavailable fields to fit the 32-character SDM budget.
//!
//! The two `double?` dB fields of the C# record are carried as integer
//! tenths-of-dB (`Option<i16>`) — see the [module docs](crate::tune) for the
//! fixed-point rationale.

use core::fmt::{self, Write};

use super::{parse_nonneg_i32, CountWriter, SliceWriter};

/// The wire token for an unavailable field in the canonical form.
const UNAVAILABLE: &str = "na";

/// One burst measurement — the `MS`-telegram payload.
///
/// The bracketing signals (decoded-frame count vs sent, the IL2P FEC-corrected
/// byte delta, the lost-ADC-sample/clip delta, and the CCDI RSSI) are always
/// attempted; the RX-audio level is optional enrichment. Fields absent from the
/// wire parse as `None`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MeterReport {
    /// Burst frames the meter's TNC decoded.
    pub decoded_frames: i32,
    /// Burst frames the meter asked for (`RQ` n).
    pub requested_frames: i32,
    /// IL2P FEC-corrected-byte delta across the burst (GETALL register 11), or
    /// `None` when the firmware's GETALL reply doesn't carry it / the mode is not
    /// IL2P.
    pub fec_corrected_bytes_delta: Option<i64>,
    /// Lost-ADC-sample delta across the burst (`LostADCSmp`), or `None` when
    /// unavailable. Positive = the RX audio clipped (gross over-deviation).
    pub lost_adc_samples_delta: Option<i64>,
    /// Median Tait CCDI RSSI during the burst, in **tenths of a dBm**, or `None`
    /// when no CCDI radio is attached at the meter end. (`-904` == `-90.4 dBm`.)
    pub rssi_dbm_tenths: Option<i16>,
    /// RX-audio RMS level at the meter's TNC during the burst, in **tenths of a
    /// dB** (GETRSSI — firmware 3.41-era only, removed in 3.44), or `None`. A
    /// carrier *quiets* the demodulated audio, so lower = more quieting = signal
    /// present. Old peers that predate this field simply omit it (`None`).
    pub audio_level_db_tenths: Option<i16>,
}

impl MeterReport {
    /// Construct a report from its component fields (mirrors the C# record's
    /// primary constructor). dB fields are integer tenths — see the struct docs.
    pub const fn new(
        decoded_frames: i32,
        requested_frames: i32,
        fec_corrected_bytes_delta: Option<i64>,
        lost_adc_samples_delta: Option<i64>,
        rssi_dbm_tenths: Option<i16>,
        audio_level_db_tenths: Option<i16>,
    ) -> Self {
        Self {
            decoded_frames,
            requested_frames,
            fec_corrected_bytes_delta,
            lost_adc_samples_delta,
            rssi_dbm_tenths,
            audio_level_db_tenths,
        }
    }

    /// Decode success in parts-per-thousand (0 when nothing was requested). The
    /// integer twin of the C# `DecodeRate` `double` (`no_std`, no-FPU): `5/10`
    /// reports `500`.
    pub fn decode_rate_permille(&self) -> u32 {
        if self.requested_frames > 0 {
            let rate = self.decoded_frames as i64 * 1000 / self.requested_frames as i64;
            rate.max(0) as u32
        } else {
            0
        }
    }

    /// A copy with the optional audio level cleared — the "drop the enrichment to
    /// fit the SDM budget" move (`report with { AudioLevelDb = null }` in C#).
    pub(crate) fn without_audio_level(&self) -> Self {
        Self {
            audio_level_db_tenths: None,
            ..*self
        }
    }

    /// Write the canonical `MS` args
    /// (`dec/n|fec:<Δ>|clip:<Δ>|rssi:<dBm>|lvl:<dB>`, `na` for unavailable) into
    /// a caller buffer. Returns bytes written, or `None` if the buffer is too
    /// small. Mirrors `MeterReport.ToArgs`.
    pub fn to_args_into(&self, buf: &mut [u8]) -> Option<usize> {
        let mut w = SliceWriter::new(buf);
        self.write_args(&mut w).ok()?;
        Some(w.len())
    }

    /// Write the compact `MS` args (`dec/n|f<Δ>|c<Δ>|r<dBm>|l<dB>`, unavailable
    /// fields omitted) into a caller buffer. Returns bytes written, or `None` if
    /// the buffer is too small. Mirrors `MeterReport.ToCompactArgs`.
    pub fn to_compact_args_into(&self, buf: &mut [u8]) -> Option<usize> {
        let mut w = SliceWriter::new(buf);
        self.write_compact_args(&mut w).ok()?;
        Some(w.len())
    }

    /// The canonical `MS` args as an owned `String`. Mirrors `MeterReport.ToArgs`.
    #[cfg(feature = "alloc")]
    pub fn to_args(&self) -> alloc::string::String {
        let mut s = alloc::string::String::new();
        let _ = self.write_args(&mut s);
        s
    }

    /// The compact `MS` args as an owned `String`. Mirrors
    /// `MeterReport.ToCompactArgs`.
    #[cfg(feature = "alloc")]
    pub fn to_compact_args(&self) -> alloc::string::String {
        let mut s = alloc::string::String::new();
        let _ = self.write_compact_args(&mut s);
        s
    }

    /// Parse `MS` args in either the canonical or the compact form. Fields absent
    /// from the wire parse as `None`. Returns `None` for anything that is not a
    /// valid `MS` args string (bad counts, or an unrecognised field key). Mirrors
    /// `MeterReport.TryParse`.
    pub fn try_parse(args: &str) -> Option<Self> {
        if args.is_empty() {
            return None;
        }

        let mut fields = args.split('|');
        let counts = fields.next()?;
        let mut count_parts = counts.split('/');
        let decoded = parse_nonneg_i32(count_parts.next()?)?;
        let requested = parse_nonneg_i32(count_parts.next()?)?;
        if count_parts.next().is_some() {
            return None; // counts had more than two '/'-separated segments
        }

        let mut fec = None;
        let mut clip = None;
        let mut rssi = None;
        let mut level = None;
        for part in fields {
            if let Some(text) = try_field(part, "fec:", "f") {
                fec = parse_long_opt(text);
            } else if let Some(text) = try_field(part, "clip:", "c") {
                clip = parse_long_opt(text);
            } else if let Some(text) = try_field(part, "rssi:", "r") {
                rssi = parse_db_tenths_opt(text);
            } else if let Some(text) = try_field(part, "lvl:", "l") {
                level = parse_db_tenths_opt(text);
            } else {
                return None; // unknown field — not an MS args string
            }
        }

        Some(Self {
            decoded_frames: decoded,
            requested_frames: requested,
            fec_corrected_bytes_delta: fec,
            lost_adc_samples_delta: clip,
            rssi_dbm_tenths: rssi,
            audio_level_db_tenths: level,
        })
    }

    /// Render the compact form into any writer (shared by the `_into` and `alloc`
    /// twins, and by the telegram's compact encoder).
    pub(crate) fn write_compact_args<W: Write>(&self, w: &mut W) -> fmt::Result {
        write!(w, "{}/{}", self.decoded_frames, self.requested_frames)?;
        if let Some(fec) = self.fec_corrected_bytes_delta {
            write!(w, "|f{}", fec)?;
        }
        if let Some(clip) = self.lost_adc_samples_delta {
            write!(w, "|c{}", clip)?;
        }
        if let Some(rssi) = self.rssi_dbm_tenths {
            w.write_str("|r")?;
            write_db_tenths(w, rssi)?;
        }
        if let Some(level) = self.audio_level_db_tenths {
            w.write_str("|l")?;
            write_db_tenths(w, level)?;
        }
        Ok(())
    }

    /// Render the canonical form into any writer.
    fn write_args<W: Write>(&self, w: &mut W) -> fmt::Result {
        write!(w, "{}/{}|fec:", self.decoded_frames, self.requested_frames)?;
        write_long_opt(w, self.fec_corrected_bytes_delta)?;
        w.write_str("|clip:")?;
        write_long_opt(w, self.lost_adc_samples_delta)?;
        w.write_str("|rssi:")?;
        write_db_opt(w, self.rssi_dbm_tenths)?;
        w.write_str("|lvl:")?;
        write_db_opt(w, self.audio_level_db_tenths)?;
        Ok(())
    }

    /// Rendered length of the compact form without a buffer — lets the telegram
    /// encoder decide whether the SDM budget is busted before it commits bytes.
    pub(crate) fn compact_args_len(&self) -> usize {
        let mut c = CountWriter::new();
        let _ = self.write_compact_args(&mut c);
        c.len()
    }
}

/// Match a wire field against its canonical key (`fec:`) or its compact key
/// (`f`), returning the value text after whichever matched. Mirrors the C#
/// `TryField` (canonical checked first). The single-letter compact keys are
/// distinct, so the caller's ordered `else if` chain disambiguates them exactly
/// as the C# does.
fn try_field<'a>(part: &'a str, canonical_key: &str, compact_key: &str) -> Option<&'a str> {
    if let Some(rest) = part.strip_prefix(canonical_key) {
        return Some(rest);
    }
    part.strip_prefix(compact_key)
}

/// Parse an `fec`/`clip` value: `na` (or any unparseable text) → `None`,
/// otherwise a signed `i64`. Mirrors the C# `ParseLong` (`NumberStyles.AllowLeadingSign`).
fn parse_long_opt(text: &str) -> Option<i64> {
    if text == UNAVAILABLE {
        return None;
    }
    text.parse::<i64>().ok()
}

/// Parse an `rssi`/`lvl` value into tenths-of-dB: `na` (or unparseable) → `None`.
/// Mirrors the C# `ParseDouble` (`NumberStyles.Float`) reduced to fixed point.
fn parse_db_tenths_opt(text: &str) -> Option<i16> {
    if text == UNAVAILABLE {
        return None;
    }
    parse_db_tenths(text)
}

/// Parse a fixed-point decimal string into tenths (`"-90.4"` → `-904`). Accepts
/// an optional leading sign, an integer part, and an optional fractional part;
/// rounds half-up to the nearest tenth if more than one fractional digit is
/// present (a conforming peer only ever sends one). Rejects signs-only, empty
/// input, non-digits, and anything out of `i16` range.
fn parse_db_tenths(text: &str) -> Option<i16> {
    let bytes = text.as_bytes();
    let (negative, rest) = match bytes.first() {
        Some(b'-') => (true, &text[1..]),
        Some(b'+') => (false, &text[1..]),
        _ => (false, text),
    };
    if rest.is_empty() {
        return None;
    }

    let (int_part, frac_part) = match rest.find('.') {
        Some(dot) => (&rest[..dot], &rest[dot + 1..]),
        None => (rest, ""),
    };
    if int_part.is_empty() && frac_part.is_empty() {
        return None; // just a bare "."
    }

    let mut whole: i32 = 0;
    for &b in int_part.as_bytes() {
        if !b.is_ascii_digit() {
            return None;
        }
        whole = whole.checked_mul(10)?.checked_add((b - b'0') as i32)?;
    }

    let frac_bytes = frac_part.as_bytes();
    for &b in frac_bytes {
        if !b.is_ascii_digit() {
            return None;
        }
    }

    let tenth = frac_bytes.first().map_or(0, |&b| (b - b'0') as i32);
    let mut tenths = whole.checked_mul(10)?.checked_add(tenth)?;
    // Round half-up on the second fractional digit (never emitted by the codec).
    if frac_bytes.get(1).is_some_and(|&b| b >= b'5') {
        tenths = tenths.checked_add(1)?;
    }
    if negative {
        tenths = -tenths;
    }
    i16::try_from(tenths).ok()
}

/// Write an optional signed integer, or `na` when absent.
fn write_long_opt<W: Write>(w: &mut W, value: Option<i64>) -> fmt::Result {
    match value {
        Some(v) => write!(w, "{}", v),
        None => w.write_str(UNAVAILABLE),
    }
}

/// Write an optional dB value (tenths), or `na` when absent.
fn write_db_opt<W: Write>(w: &mut W, value: Option<i16>) -> fmt::Result {
    match value {
        Some(v) => write_db_tenths(w, v),
        None => w.write_str(UNAVAILABLE),
    }
}

/// Format a tenths-of-dB value as a one-decimal string (`-904` → `"-90.4"`),
/// matching the C# `ToString("0.0")` / `{:0.0}` output for a value that is
/// exactly a tenth.
fn write_db_tenths<W: Write>(w: &mut W, tenths: i16) -> fmt::Result {
    if tenths < 0 {
        w.write_str("-")?;
    }
    let magnitude = (tenths as i32).unsigned_abs();
    write!(w, "{}.{}", magnitude / 10, magnitude % 10)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Encode canonical args into a stack buffer and view them as text (works in
    /// every feature configuration — no `alloc` needed).
    fn args_str<'a>(buf: &'a mut [u8], report: &MeterReport) -> &'a str {
        let n = report.to_args_into(buf).unwrap();
        core::str::from_utf8(&buf[..n]).unwrap()
    }

    fn compact_str<'a>(buf: &'a mut [u8], report: &MeterReport) -> &'a str {
        let n = report.to_compact_args_into(buf).unwrap();
        core::str::from_utf8(&buf[..n]).unwrap()
    }

    #[test]
    fn canonical_args_render_the_documented_form() {
        // MeterReport.cs example: 5/5|fec:na|clip:0|rssi:-90.1|lvl:-62.5
        let report = MeterReport::new(5, 5, None, Some(0), Some(-901), Some(-625));
        let mut buf = [0u8; 64];
        assert_eq!(args_str(&mut buf, &report), "5/5|fec:na|clip:0|rssi:-90.1|lvl:-62.5");
    }

    #[test]
    fn compact_args_render_the_documented_form() {
        let report = MeterReport::new(5, 5, None, Some(0), Some(-901), Some(-625));
        let mut buf = [0u8; 64];
        assert_eq!(compact_str(&mut buf, &report), "5/5|c0|r-90.1|l-62.5");
    }

    #[test]
    fn unavailable_fields_are_na_canonically_and_omitted_compactly() {
        let report = MeterReport::new(3, 5, None, None, None, None);
        let mut buf = [0u8; 64];
        assert_eq!(args_str(&mut buf, &report), "3/5|fec:na|clip:na|rssi:na|lvl:na");
        assert_eq!(compact_str(&mut buf, &report), "3/5");
    }

    #[test]
    fn canonical_round_trip_with_all_fields() {
        let report = MeterReport::new(10, 10, Some(480), Some(0), Some(-904), Some(-625));
        let mut buf = [0u8; 64];
        let parsed = MeterReport::try_parse(args_str(&mut buf, &report)).unwrap();
        assert_eq!(parsed, report);
    }

    #[test]
    fn compact_round_trip_with_all_fields() {
        let report = MeterReport::new(10, 10, Some(480), Some(0), Some(-904), Some(-625));
        let mut buf = [0u8; 64];
        let parsed = MeterReport::try_parse(compact_str(&mut buf, &report)).unwrap();
        assert_eq!(parsed, report);
    }

    #[test]
    fn null_fields_round_trip_as_null_in_both_forms() {
        let report = MeterReport::new(3, 5, None, None, None, None);
        let mut buf = [0u8; 64];
        assert_eq!(MeterReport::try_parse(args_str(&mut buf, &report)).unwrap(), report);
        assert_eq!(MeterReport::try_parse(compact_str(&mut buf, &report)).unwrap(), report);
    }

    #[test]
    fn decodes_known_good_canonical_vector() {
        // From TuningTelegramTests: an old peer that predates the level field.
        let report = MeterReport::try_parse("4/5|fec:12|clip:0|rssi:-90.4").unwrap();
        assert_eq!(report.decoded_frames, 4);
        assert_eq!(report.requested_frames, 5);
        assert_eq!(report.fec_corrected_bytes_delta, Some(12));
        assert_eq!(report.lost_adc_samples_delta, Some(0));
        assert_eq!(report.rssi_dbm_tenths, Some(-904));
        assert_eq!(report.audio_level_db_tenths, None);
    }

    #[test]
    fn decodes_known_good_compact_vector() {
        let report = MeterReport::try_parse("5/5|c0|r-90.1").unwrap();
        assert_eq!(report.lost_adc_samples_delta, Some(0));
        assert_eq!(report.rssi_dbm_tenths, Some(-901));
        assert_eq!(report.audio_level_db_tenths, None);
    }

    #[test]
    fn decode_of_na_fields_yields_null() {
        let report = MeterReport::try_parse("3/5|fec:na|clip:na|rssi:na|lvl:na").unwrap();
        assert_eq!(report, MeterReport::new(3, 5, None, None, None, None));
    }

    #[test]
    fn rejects_junk() {
        assert!(MeterReport::try_parse("").is_none());
        assert!(MeterReport::try_parse("nope").is_none());
        assert!(MeterReport::try_parse("a/b").is_none());
        assert!(MeterReport::try_parse("3/5|bogus:1").is_none());
        // Signed / malformed counts (NumberStyles.None rejects the sign).
        assert!(MeterReport::try_parse("-3/5").is_none());
        assert!(MeterReport::try_parse("3/5/7").is_none());
        assert!(MeterReport::try_parse("3/5|").is_none());
    }

    #[test]
    fn positive_and_zero_tenths_format_correctly() {
        let report = MeterReport::new(1, 1, None, None, Some(0), Some(55));
        let mut buf = [0u8; 64];
        assert_eq!(compact_str(&mut buf, &report), "1/1|r0.0|l5.5");
    }

    #[test]
    fn negative_sub_unit_tenths_format_with_zero_whole() {
        let report = MeterReport::new(1, 1, None, None, Some(-5), None);
        let mut buf = [0u8; 64];
        assert_eq!(compact_str(&mut buf, &report), "1/1|r-0.5");
    }

    #[test]
    fn parse_rounds_sub_tenth_precision_half_up() {
        // A peer sending more precision than the codec emits — round to a tenth.
        assert_eq!(parse_db_tenths("-90.45"), Some(-905));
        assert_eq!(parse_db_tenths("-90.44"), Some(-904));
        assert_eq!(parse_db_tenths("90"), Some(900));
        assert_eq!(parse_db_tenths(".5"), Some(5));
    }

    #[test]
    fn parse_db_rejects_malformed() {
        assert!(parse_db_tenths("").is_none());
        assert!(parse_db_tenths("-").is_none());
        assert!(parse_db_tenths(".").is_none());
        assert!(parse_db_tenths("9x").is_none());
    }

    #[test]
    fn decode_rate_permille_is_integer() {
        assert_eq!(MeterReport::new(5, 10, None, None, None, None).decode_rate_permille(), 500);
        assert_eq!(MeterReport::new(10, 10, None, None, None, None).decode_rate_permille(), 1000);
        assert_eq!(MeterReport::new(0, 0, None, None, None, None).decode_rate_permille(), 0);
    }

    #[test]
    fn into_reports_none_on_undersized_buffer() {
        let report = MeterReport::new(10, 10, Some(480), Some(0), Some(-904), Some(-625));
        let mut tiny = [0u8; 4];
        assert!(report.to_args_into(&mut tiny).is_none());
    }

    #[cfg(feature = "alloc")]
    #[test]
    fn alloc_twins_match_the_into_forms() {
        let report = MeterReport::new(5, 5, None, Some(0), Some(-901), Some(-625));
        assert_eq!(report.to_args(), "5/5|fec:na|clip:0|rssi:-90.1|lvl:-62.5");
        assert_eq!(report.to_compact_args(), "5/5|c0|r-90.1|l-62.5");
    }
}
