//! # `tune` — the SDM tuning-telegram wire codec
//!
//! A pure, zero-dependency port of the parity-locked tuning-protocol codec from
//! the C# reference assembly `Packet.Tune.Core` (`TuningTelegram.cs`,
//! `MeterReport.cs`). It is deliberately isolated from the Tait CCDI driver: the
//! same telegram travels over *any* tuning link (a Tait SDM, a WebSocket frame),
//! so the codec knows nothing about how the bytes are carried. Only the two wire
//! forms live here — text in ↔ struct out, byte-for-byte with the C#.
//!
//! - [`TuningTelegram`] — the compact ASCII line `V1|<seq>|<verb>|<args>`
//!   (`TuningTelegram.cs`), with both the canonical [`TuningTelegram::encode_into`]
//!   and the SDM-budget [`TuningTelegram::encode_compact_into`] forms, plus
//!   [`TuningTelegram::try_parse`] that accepts either.
//! - [`MeterReport`] — the `MS`-verb measurement payload (`MeterReport.cs`), with
//!   the canonical `dec/n|fec:…|clip:…|rssi:…|lvl:…` args and the single-letter
//!   compact form.
//! - [`SdmTuningLink`] — the retry + sequence-dedupe reliability layer that carries
//!   telegrams over a Tait CCDI SDM side channel (`SdmTuningLink.cs`). Unlike the two
//!   codecs above, the link *does* ride the Tait driver — but it stays here (not under
//!   `radio/`) because it is a tuning-protocol concern. It is **receipt-tolerant by
//!   default** ([`SdmTuningLinkOptions::wait_for_delivery_receipt`] false): a send
//!   completes on the radio accepting the datagram, not on an over-air delivery
//!   receipt (which the TM8110 auto-ack refractory makes unreliable).
//!
//! ## Fixed-point / `no_std` divergences from the C# reference
//!
//! The M0+ has no FPU, so the two `double?` dB fields in the C# `MeterReport`
//! (`RssiDbm`, `AudioLevelDb`) are carried here as **integer tenths-of-dB**
//! (`Option<i16>`): `-90.4 dB` is `-904`. The wire form is unchanged — both the
//! C# `ToString("0.0")` / `{:0.0}` formats and this port emit exactly one
//! fractional digit — so the text round-trips byte-for-byte. On parse we accept
//! any decimal a conforming peer could send (the C# uses `NumberStyles.Float`)
//! and round to the nearest tenth (half-up) if a peer ever sent sub-tenth
//! precision, which the codec itself never emits. Scientific notation and
//! embedded whitespace are not accepted (the codec never produces them).
//!
//! Following the crate's fixed-capacity idiom, every growable C# `string` result
//! has a zero-alloc `..._into(&mut [u8]) -> Option<usize>` twin that writes into a
//! caller buffer, plus an `alloc`-gated `String`-returning twin mirroring the C#
//! signature. Decoding borrows straight out of the input `&str` (no allocation).

use core::fmt;

pub mod meter_report;
pub mod sdm_link;
pub mod telegram;

pub use meter_report::MeterReport;
pub use sdm_link::{LinkDelay, SdmLinkError, SdmTuningLink, SdmTuningLinkOptions};
pub use telegram::{TuningTelegram, TuningVerb};

/// A [`core::fmt::Write`] sink over a fixed caller buffer. `write_str` fails (and
/// aborts the whole `write!`) the moment the buffer would overflow, which the
/// `..._into` codec entry points map to `None`. Shared by both submodules.
pub(crate) struct SliceWriter<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl<'a> SliceWriter<'a> {
    pub(crate) fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, pos: 0 }
    }

    /// Bytes written so far — the return value of the `..._into` methods.
    pub(crate) fn len(&self) -> usize {
        self.pos
    }
}

impl fmt::Write for SliceWriter<'_> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let bytes = s.as_bytes();
        let end = self.pos.checked_add(bytes.len()).ok_or(fmt::Error)?;
        if end > self.buf.len() {
            return Err(fmt::Error);
        }
        self.buf[self.pos..end].copy_from_slice(bytes);
        self.pos = end;
        Ok(())
    }
}

/// A [`core::fmt::Write`] sink that measures the rendered length without a buffer.
/// Used to decide, before writing, whether the compact telegram busts the SDM
/// budget (the point at which the optional audio level is dropped).
pub(crate) struct CountWriter {
    len: usize,
}

impl CountWriter {
    pub(crate) fn new() -> Self {
        Self { len: 0 }
    }

    pub(crate) fn len(&self) -> usize {
        self.len
    }
}

impl fmt::Write for CountWriter {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        self.len += s.len();
        Ok(())
    }
}

/// Parse a non-negative decimal `i32` — the exact acceptance of the C#
/// `int.TryParse(_, NumberStyles.None, …)` used for the telegram sequence and the
/// meter frame counts: ASCII digits only (no sign, no whitespace, no separators),
/// non-empty, and within `i32` range. Leading zeros are accepted (`"007"` → `7`).
pub(crate) fn parse_nonneg_i32(s: &str) -> Option<i32> {
    if s.is_empty() {
        return None;
    }
    let mut value: i32 = 0;
    for &b in s.as_bytes() {
        if !b.is_ascii_digit() {
            return None;
        }
        value = value.checked_mul(10)?.checked_add((b - b'0') as i32)?;
    }
    Some(value)
}
