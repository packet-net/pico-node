//! Reflash a NinoTNC's firmware through its resident bootloader, over the same
//! serial line KISS uses.
//!
//! Ports `Packet.Kiss.NinoTnc.Firmware.BootloaderNinoTncFirmwareFlasher` +
//! `NinoTncFirmwareHexImage` + `NinoTncFlashTimings`, which in turn follow
//! upstream `flashtnc.py` (version f). The protocol, at the KISS baud (57 600):
//!
//! 1. Drain the line until it has been silent for a read timeout.
//! 2. Probe for a bootloader stranded by an earlier failed flash: send `R`; only
//!    the bootloader answers `K` (KISS firmware ignores a stray byte).
//! 3. Otherwise fill and flush: three bare GETALLs, discarding the replies, then
//!    drain again. Then send the entry command `C0 0D 37 C0` and wait for `K`.
//! 4. Send `V`; the reply letter names the bootloader and so the chip
//!    (lowercase = dsPIC33EP256GP, firmware 3.x; uppercase = dsPIC33EP512GP, 4.x).
//!    A mismatch with the image is refused before anything is written.
//! 5. Send the Intel-HEX file a line at a time, each answered `K` (next), `Z`
//!    (finished, the TNC reboots into the new firmware) or `F` / `N` / `X`
//!    (failed). The first line goes one character per 100 ms: the bootloader
//!    erases flash while it arrives.
//!
//! ## Pico specifics
//!
//! The node cannot hold a 700 KB text file in RAM, and it must validate the
//! whole image before touching the TNC (the chip fingerprint sits near the end
//! of the file). So [`HexStager`] checks the upload line by line as it streams
//! in and packs each record to binary for the caller to store in flash (about
//! 330 KB for firmware 3.44), and the flash later replays the records as text
//! with [`render_line`]. Validation is stricter than packet.net's: each
//! record's length byte and checksum are checked here, not left to the
//! bootloader's `N`.
//!
//! [`BootloaderFlasher`] is the protocol as a sans-I/O state machine: the caller
//! owns the UART and the clock and performs the [`FlashAction`]s it returns.

use core::fmt;

use super::firmware::ChipVariant;
use crate::crc::Crc16;

// ── The image ──

/// The longest Intel-HEX line: `:` + 2 hex digits for each of up to 255 data
/// bytes plus length, two address bytes, type and checksum.
pub const MAX_LINE: usize = 1 + 2 * (255 + 5);

/// The longest packed record: a 2-byte header plus up to 260 record bytes.
pub const MAX_PACKED_RECORD: usize = 2 + 255 + 5;

/// The Intel-HEX end-of-file record, which must be the image's last line.
pub const END_OF_FILE_RECORD: &str = ":00000001FF";

/// The first bootloader line of each known image, per `flashtnc.py` (version f,
/// 2022-05-01). The checksum suffix pins each line to its chip variant.
const KNOWN_BOOTLOADER_LINES: [(&str, ChipVariant); 7] = [
    (":108800007a00fa0000002200000f7800c3e8a900f7", ChipVariant::Dspic33Ep512),
    (":10427c007a00fa0000002200000f7800c3e8a900c1", ChipVariant::Dspic33Ep256),
    (":10427c007a00fa00403f9800ce389000010f780089", ChipVariant::Dspic33Ep256),
    (":10427c007c00fa00503f980000002200000f7800ec", ChipVariant::Dspic33Ep256),
    (":102800007c00fa00503f980000002200000f780082", ChipVariant::Dspic33Ep256),
    (":102800002f08b000889fbe008a9fbe008c9fbe002c", ChipVariant::Dspic33Ep256),
    (":108800002f08b000889fbe008a9fbe008c9fbe00cc", ChipVariant::Dspic33Ep512),
];

/// Why an uploaded image was refused. `line` numbers count from 1.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HexError {
    /// A byte that cannot appear in an Intel-HEX text file.
    NotText {
        /// The line.
        line: u32,
        /// The byte.
        byte: u8,
    },
    /// A line not starting with `:`.
    NoRecordMark {
        /// The line.
        line: u32,
    },
    /// A character that is not a hex digit.
    BadHexDigit {
        /// The line.
        line: u32,
    },
    /// An odd number of hex digits, or fewer than a record needs.
    Malformed {
        /// The line.
        line: u32,
    },
    /// The record's length byte disagrees with the line.
    LengthMismatch {
        /// The line.
        line: u32,
    },
    /// The record's checksum is wrong.
    BadChecksum {
        /// The line.
        line: u32,
    },
    /// Upper- and lower-case hex digits mixed in one line (never seen in a real
    /// image; refused because the node replays each line in one case).
    MixedCase {
        /// The line.
        line: u32,
    },
    /// Longer than any Intel-HEX record.
    LineTooLong {
        /// The line.
        line: u32,
    },
    /// A record after the end-of-file record.
    AfterEndOfFile {
        /// The line.
        line: u32,
    },
    /// No records at all.
    Empty,
    /// The last line is not the end-of-file record, so the bootloader would never
    /// say it had finished and the TNC would be left mid-flash.
    NoEndOfFile,
    /// None of the known bootloader fingerprints: the target chip cannot be told,
    /// and the wrong variant bricks the TNC.
    UnknownTarget,
    /// The node's storage for the image is full.
    TooLarge,
}

impl fmt::Display for HexError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match *self {
            HexError::NotText { line, byte } => write!(
                f,
                "Line {line} contains byte 0x{byte:02X}: this is not an Intel-HEX text file."
            ),
            HexError::NoRecordMark { line } => {
                write!(f, "Line {line} does not start with ':': not an Intel-HEX file.")
            }
            HexError::BadHexDigit { line } => write!(f, "Line {line} contains a non-hex character."),
            HexError::Malformed { line } => write!(f, "Line {line} is not a complete hex record."),
            HexError::LengthMismatch { line } => {
                write!(f, "Line {line}'s length byte does not match the line.")
            }
            HexError::BadChecksum { line } => write!(
                f,
                "Line {line} fails its checksum: the file is damaged. Download it again."
            ),
            HexError::MixedCase { line } => {
                write!(f, "Line {line} mixes upper- and lower-case hex digits.")
            }
            HexError::LineTooLong { line } => write!(f, "Line {line} is too long for a hex record."),
            HexError::AfterEndOfFile { line } => {
                write!(f, "Line {line} comes after the end-of-file record.")
            }
            HexError::Empty => write!(f, "The file contains no hex records."),
            HexError::NoEndOfFile => write!(
                f,
                "The file does not end with the end-of-file record ({END_OF_FILE_RECORD}); it \
may be cut short. Download it again."
            ),
            HexError::UnknownTarget => write!(
                f,
                "This is not a NinoTNC firmware file the node recognises (no known bootloader \
fingerprint), so it cannot tell which chip it is for. Refused: the wrong one bricks the TNC."
            ),
            HexError::TooLarge => write!(f, "The file is too large for the node to hold."),
        }
    }
}

/// What a successfully staged image holds.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HexSummary {
    /// Records (lines) in the image.
    pub lines: u32,
    /// Bytes of packed records handed to the sink.
    pub packed_len: u32,
    /// CRC-16/X.25 over the packed bytes, to check the stored copy before use.
    pub crc: u16,
    /// The chip the image is for.
    pub target: ChipVariant,
}

/// Validates an Intel-HEX upload as it streams in and packs each record for
/// storage. Feed with [`Self::push`], end with [`Self::finish`].
///
/// Packed record: a little-endian `u16` header (bits 0-14 the record's byte
/// count, bit 15 set when its hex digits were upper case) then the record bytes
/// (length, address high, address low, type, data, checksum).
pub struct HexStager {
    line: [u8; MAX_LINE],
    len: usize,
    too_long: bool,
    line_no: u32,
    lines: u32,
    target: ChipVariant,
    last_was_eof: bool,
    packed_len: u32,
    crc: Crc16,
}

impl Default for HexStager {
    fn default() -> Self {
        Self::new()
    }
}

impl HexStager {
    /// A fresh stager.
    pub const fn new() -> Self {
        Self {
            line: [0; MAX_LINE],
            len: 0,
            too_long: false,
            line_no: 0,
            lines: 0,
            target: ChipVariant::Unknown,
            last_was_eof: false,
            packed_len: 0,
            crc: Crc16::new(),
        }
    }

    /// Records accepted so far.
    pub fn lines(&self) -> u32 {
        self.lines
    }

    /// Feed the next chunk of the upload. `sink` receives each packed record and
    /// returns `false` when storage is full.
    pub fn push(
        &mut self,
        data: &[u8],
        sink: &mut impl FnMut(&[u8]) -> bool,
    ) -> Result<(), HexError> {
        for &b in data {
            match b {
                b'\n' | b'\r' => self.end_line(sink)?,
                0x20..=0x7E => {
                    if self.len < MAX_LINE {
                        self.line[self.len] = b;
                        self.len += 1;
                    } else {
                        self.too_long = true;
                    }
                }
                other => {
                    return Err(HexError::NotText {
                        line: self.line_no + 1,
                        byte: other,
                    })
                }
            }
        }
        Ok(())
    }

    /// End of upload: process any last line without a newline and check the
    /// image as a whole.
    pub fn finish(mut self, sink: &mut impl FnMut(&[u8]) -> bool) -> Result<HexSummary, HexError> {
        self.end_line(sink)?;
        if self.lines == 0 {
            return Err(HexError::Empty);
        }
        if !self.last_was_eof {
            return Err(HexError::NoEndOfFile);
        }
        if self.target == ChipVariant::Unknown {
            return Err(HexError::UnknownTarget);
        }
        Ok(HexSummary {
            lines: self.lines,
            packed_len: self.packed_len,
            crc: self.crc.finish(),
            target: self.target,
        })
    }

    fn end_line(&mut self, sink: &mut impl FnMut(&[u8]) -> bool) -> Result<(), HexError> {
        if self.len == 0 && !self.too_long {
            return Ok(()); // blank line, or the \n of a \r\n
        }
        self.line_no += 1;
        let line_no = self.line_no;
        let text_len = self.len;
        self.len = 0;
        if core::mem::take(&mut self.too_long) {
            return Err(HexError::LineTooLong { line: line_no });
        }
        if self.last_was_eof {
            return Err(HexError::AfterEndOfFile { line: line_no });
        }
        let text = &self.line[..text_len];
        let mut packed = [0u8; MAX_PACKED_RECORD];
        let (n, upper) = pack_line(text, &mut packed[2..], line_no)?;
        let header = (n as u16) | if upper { 0x8000 } else { 0 };
        packed[..2].copy_from_slice(&header.to_le_bytes());
        let record = &packed[..2 + n];

        if self.target == ChipVariant::Unknown {
            self.target = fingerprint(text);
        }
        self.last_was_eof = text.eq_ignore_ascii_case(END_OF_FILE_RECORD.as_bytes());
        if !sink(record) {
            return Err(HexError::TooLarge);
        }
        self.crc.update(record);
        self.packed_len += record.len() as u32;
        self.lines += 1;
        Ok(())
    }
}

/// The chip a line's fingerprint names, or `Unknown`.
fn fingerprint(line: &[u8]) -> ChipVariant {
    KNOWN_BOOTLOADER_LINES
        .iter()
        .find(|(known, _)| line.eq_ignore_ascii_case(known.as_bytes()))
        .map(|(_, chip)| *chip)
        .unwrap_or(ChipVariant::Unknown)
}

fn hex_value(c: u8) -> Option<u8> {
    match c {
        b'0'..=b'9' => Some(c - b'0'),
        b'a'..=b'f' => Some(c - b'a' + 10),
        b'A'..=b'F' => Some(c - b'A' + 10),
        _ => None,
    }
}

/// Validate one line and write its record bytes into `out`. Returns the byte
/// count and whether its hex letters were upper case.
fn pack_line(text: &[u8], out: &mut [u8], line: u32) -> Result<(usize, bool), HexError> {
    if text.first() != Some(&b':') {
        return Err(HexError::NoRecordMark { line });
    }
    let digits = &text[1..];
    if !digits.len().is_multiple_of(2) || digits.len() < 10 {
        return Err(HexError::Malformed { line });
    }
    let (mut lower, mut upper) = (false, false);
    let mut sum: u8 = 0;
    for (i, pair) in digits.chunks(2).enumerate() {
        let mut byte = 0u8;
        for &c in pair {
            let v = hex_value(c).ok_or(HexError::BadHexDigit { line })?;
            lower |= c.is_ascii_lowercase();
            upper |= c.is_ascii_uppercase();
            byte = (byte << 4) | v;
        }
        out[i] = byte;
        sum = sum.wrapping_add(byte);
    }
    if lower && upper {
        return Err(HexError::MixedCase { line });
    }
    let n = digits.len() / 2;
    if out[0] as usize + 5 != n {
        return Err(HexError::LengthMismatch { line });
    }
    if sum != 0 {
        return Err(HexError::BadChecksum { line });
    }
    Ok((n, upper))
}

/// Split a packed record header into (record byte count, upper case).
pub fn record_header(header: [u8; 2]) -> (usize, bool) {
    let h = u16::from_le_bytes(header);
    ((h & 0x7FFF) as usize, h & 0x8000 != 0)
}

/// Render a packed record's bytes back to its text line plus `\n`, into `out`
/// (at least `MAX_LINE + 1` bytes). Returns the length.
pub fn render_line(record: &[u8], upper: bool, out: &mut [u8]) -> usize {
    let digits: &[u8; 16] = if upper {
        b"0123456789ABCDEF"
    } else {
        b"0123456789abcdef"
    };
    out[0] = b':';
    let mut n = 1;
    for &b in record {
        out[n] = digits[(b >> 4) as usize];
        out[n + 1] = digits[(b & 0x0F) as usize];
        n += 2;
    }
    out[n] = b'\n';
    n + 1
}

// ── The bootloader conversation ──

/// Every timing of the protocol. The defaults are packet.net's
/// hardware-validated values (from `flashtnc.py`); tests shrink them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FlashTimings {
    /// Silence this long means the line is quiet (also the stranded-probe wait).
    pub read_timeout_ms: u64,
    /// Give up draining if the TNC is still talking after this long.
    pub drain_abort_ms: u64,
    /// Between the fill-and-flush GETALLs.
    pub probe_spacing_ms: u64,
    /// For the bootloader's `K` after the entry command.
    pub entry_timeout_ms: u64,
    /// For the reply to `V`.
    pub version_timeout_ms: u64,
    /// After each character of the first line (the bootloader erases meanwhile).
    pub first_line_char_delay_ms: u64,
    /// For each line's reply.
    pub line_reply_timeout_ms: u64,
    /// After telling an aborted bootloader to return to the old firmware.
    pub reset_settle_ms: u64,
}

impl Default for FlashTimings {
    fn default() -> Self {
        Self {
            read_timeout_ms: 5_000,
            drain_abort_ms: 15_000,
            probe_spacing_ms: 500,
            entry_timeout_ms: 15_000,
            version_timeout_ms: 5_000,
            first_line_char_delay_ms: 100,
            line_reply_timeout_ms: 15_000,
            reset_settle_ms: 1_000,
        }
    }
}

/// Plain bytes the flasher sends.
pub const RESET_PROBE: &[u8] = b"R";
/// The version query.
pub const VERSION_QUERY: &[u8] = b"V";
/// A bare GETALL, byte-identical to flashtnc's.
pub const BARE_GET_ALL: &[u8] = &[0xC0, 0x0B, 0xC0];
/// KISS command 0x0D with the magic 0x37: enter the bootloader.
pub const BOOTLOADER_ENTRY: &[u8] = &[0xC0, 0x0D, 0x37, 0xC0];

/// Something for the caller to do on the serial line.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlashAction {
    /// Write these bytes.
    Write(&'static [u8]),
    /// Throw away whatever has been received and not yet read.
    DiscardInput,
    /// Write image line `index` (0-based) as text plus `\n`, then call
    /// [`BootloaderFlasher::line_sent`]. When `paced`, write one character at a
    /// time with [`FlashTimings::first_line_char_delay_ms`] after each.
    SendLine {
        /// The line, from 0.
        index: u32,
        /// Write it a character at a time.
        paced: bool,
    },
}

/// Up to two actions from one event.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Actions([Option<FlashAction>; 2]);

impl Actions {
    fn none() -> Self {
        Self([None, None])
    }
    fn one(a: FlashAction) -> Self {
        Self([Some(a), None])
    }
    fn two(a: FlashAction, b: FlashAction) -> Self {
        Self([Some(a), Some(b)])
    }
}

impl IntoIterator for Actions {
    type Item = FlashAction;
    type IntoIter = core::iter::Flatten<core::array::IntoIter<Option<FlashAction>, 2>>;
    fn into_iter(self) -> Self::IntoIter {
        self.0.into_iter().flatten()
    }
}

/// Why a flash failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FailureKind {
    /// The TNC kept sending data; nothing was written.
    SerialNeverQuiet,
    /// No `K` after the entry command; nothing was written.
    BootloaderEntryTimeout,
    /// No reply to `V`; nothing was written.
    VersionUnreadable,
    /// The `V` reply was not a letter; nothing was written.
    VersionUnsupported,
    /// The bootloader is for the other chip; nothing was written.
    ChipMismatch {
        /// The chip the bootloader is for.
        bootloader: ChipVariant,
    },
    /// `F`: a flash write failed.
    FlashRejected,
    /// `N`: a line's checksum was rejected.
    ChecksumRejected,
    /// `X`: an invalid character.
    InvalidCharacter,
    /// No reply to a line.
    NoResponse,
    /// A reply other than K/Z/F/N/X.
    UnexpectedResponse,
    /// Every line accepted but no `Z`.
    EndedWithoutCompletion,
    /// The caller could not produce the next line from its stored image.
    ImageUnreadable,
}

/// How a flash ended.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlashOutcome {
    /// `Z`: the TNC is rebooting into the new firmware.
    Done {
        /// Lines written.
        lines: u32,
        /// The bootloader's version letter.
        bootloader: u8,
        /// Whether the bootloader was already running (left by an earlier failure).
        was_stranded: bool,
    },
    /// Failed; see [`write_flash_outcome`].
    Failed {
        /// What went wrong.
        kind: FailureKind,
        /// The line it went wrong on (1-based), once writing had started.
        line: Option<u32>,
        /// Lines accepted before it.
        written: u32,
        /// The unexpected byte, if any.
        byte: Option<u8>,
    },
}

impl FlashOutcome {
    /// Whether the TNC's old firmware is untouched (a failure before writing).
    pub fn nothing_written(&self) -> bool {
        matches!(self, FlashOutcome::Failed { line: None, .. })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    Drain { second: bool, started_ms: u64, last_byte_ms: u64 },
    StrandedProbe { deadline_ms: u64 },
    FillFlush { round: u8, until_ms: u64 },
    AwaitReady { deadline_ms: u64 },
    AwaitVersion { deadline_ms: u64, was_stranded: bool },
    AbortSettle { until_ms: u64, outcome: FlashOutcome },
    SendingLine { index: u32 },
    AwaitLine { index: u32, deadline_ms: u64 },
    Done(FlashOutcome),
}

/// The bootloader protocol as a state machine. See the module docs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BootloaderFlasher {
    timings: FlashTimings,
    target: ChipVariant,
    total_lines: u32,
    written: u32,
    bootloader: u8,
    was_stranded: bool,
    state: State,
}

impl BootloaderFlasher {
    /// Start flashing an image of `total_lines` lines for `target`. The first
    /// action discards stale input.
    pub fn start(target: ChipVariant, total_lines: u32, now_ms: u64, timings: FlashTimings) -> (Self, Actions) {
        let f = Self {
            timings,
            target,
            total_lines,
            written: 0,
            bootloader: 0,
            was_stranded: false,
            state: State::Drain {
                second: false,
                started_ms: now_ms,
                last_byte_ms: now_ms,
            },
        };
        (f, Actions::one(FlashAction::DiscardInput))
    }

    /// Lines accepted so far, and the total.
    pub fn progress(&self) -> (u32, u32) {
        (self.written, self.total_lines)
    }

    /// The outcome, once finished.
    pub fn outcome(&self) -> Option<FlashOutcome> {
        match self.state {
            State::Done(o) => Some(o),
            _ => None,
        }
    }

    /// Whether writing has started (from here a failure leaves the TNC in its
    /// bootloader until a flash completes).
    pub fn writing(&self) -> bool {
        matches!(self.state, State::SendingLine { .. } | State::AwaitLine { .. })
            || matches!(self.state, State::Done(FlashOutcome::Failed { line: Some(_), .. }))
    }

    /// When [`Self::on_time`] next needs calling. `None` while a line is being
    /// written (the caller reports [`Self::line_sent`]) or once finished.
    pub fn deadline_ms(&self) -> Option<u64> {
        let t = &self.timings;
        match self.state {
            State::Drain { last_byte_ms, .. } => Some(last_byte_ms + t.read_timeout_ms),
            State::StrandedProbe { deadline_ms }
            | State::AwaitReady { deadline_ms }
            | State::AwaitVersion { deadline_ms, .. }
            | State::AwaitLine { deadline_ms, .. } => Some(deadline_ms),
            State::FillFlush { until_ms, .. } | State::AbortSettle { until_ms, .. } => Some(until_ms),
            State::SendingLine { .. } | State::Done(_) => None,
        }
    }

    /// A byte arrived.
    pub fn on_byte(&mut self, b: u8, now_ms: u64) -> Actions {
        let t = self.timings;
        match self.state {
            State::Drain {
                second,
                started_ms,
                ..
            } => {
                if now_ms.saturating_sub(started_ms) > t.drain_abort_ms {
                    return self.fail_before_writing(FailureKind::SerialNeverQuiet, None, None, now_ms);
                }
                self.state = State::Drain {
                    second,
                    started_ms,
                    last_byte_ms: now_ms,
                };
                Actions::none()
            }
            State::StrandedProbe { .. } => {
                if b == b'K' {
                    self.was_stranded = true;
                    self.state = State::AwaitVersion {
                        deadline_ms: now_ms + t.version_timeout_ms,
                        was_stranded: true,
                    };
                    Actions::one(FlashAction::Write(VERSION_QUERY))
                } else {
                    self.begin_fill_flush(now_ms)
                }
            }
            State::FillFlush { .. } => Actions::none(), // discarded at the next round
            State::AwaitReady { .. } => {
                if b == b'K' {
                    self.state = State::AwaitVersion {
                        deadline_ms: now_ms + t.version_timeout_ms,
                        was_stranded: false,
                    };
                    Actions::one(FlashAction::Write(VERSION_QUERY))
                } else {
                    Actions::none()
                }
            }
            State::AwaitVersion { .. } if b == b'K' => Actions::none(),
            State::AwaitVersion { .. } => {
                self.bootloader = b;
                let chip = if b.is_ascii_lowercase() {
                    ChipVariant::Dspic33Ep256
                } else if b.is_ascii_uppercase() {
                    ChipVariant::Dspic33Ep512
                } else {
                    ChipVariant::Unknown
                };
                if chip == ChipVariant::Unknown {
                    return self.fail_before_writing(FailureKind::VersionUnsupported, Some(b), Some(t.reset_settle_ms), now_ms);
                }
                if chip != self.target {
                    return self.fail_before_writing(
                        FailureKind::ChipMismatch { bootloader: chip },
                        Some(b),
                        Some(t.reset_settle_ms),
                        now_ms,
                    );
                }
                // Point of no return: the first line starts the erase.
                self.state = State::SendingLine { index: 0 };
                Actions::one(FlashAction::SendLine { index: 0, paced: true })
            }
            State::SendingLine { index } | State::AwaitLine { index, .. } => self.on_line_reply(index, b),
            State::AbortSettle { .. } | State::Done(_) => Actions::none(),
        }
    }

    /// The caller could not read the line [`FlashAction::SendLine`] asked for
    /// from its stored image.
    pub fn line_unavailable(&mut self) {
        if let State::SendingLine { index } = self.state {
            self.fail_writing(FailureKind::ImageUnreadable, index, None);
        }
    }

    /// The caller finished writing the line [`FlashAction::SendLine`] asked for.
    pub fn line_sent(&mut self, now_ms: u64) {
        if let State::SendingLine { index } = self.state {
            self.state = State::AwaitLine {
                index,
                deadline_ms: now_ms + self.timings.line_reply_timeout_ms,
            };
        }
    }

    /// Advance on the clock. Call at (or after) [`Self::deadline_ms`].
    pub fn on_time(&mut self, now_ms: u64) -> Actions {
        let t = self.timings;
        match self.state {
            State::Drain {
                second,
                last_byte_ms,
                ..
            } if now_ms >= last_byte_ms + t.read_timeout_ms => {
                if second {
                    self.state = State::AwaitReady {
                        deadline_ms: now_ms + t.entry_timeout_ms,
                    };
                    Actions::one(FlashAction::Write(BOOTLOADER_ENTRY))
                } else {
                    self.state = State::StrandedProbe {
                        deadline_ms: now_ms + t.read_timeout_ms,
                    };
                    Actions::one(FlashAction::Write(RESET_PROBE))
                }
            }
            State::StrandedProbe { deadline_ms } if now_ms >= deadline_ms => self.begin_fill_flush(now_ms),
            State::FillFlush { round, until_ms } if now_ms >= until_ms => {
                if round + 1 < 3 {
                    self.state = State::FillFlush {
                        round: round + 1,
                        until_ms: now_ms + t.probe_spacing_ms,
                    };
                    Actions::two(FlashAction::DiscardInput, FlashAction::Write(BARE_GET_ALL))
                } else {
                    self.state = State::Drain {
                        second: true,
                        started_ms: now_ms,
                        last_byte_ms: now_ms,
                    };
                    Actions::one(FlashAction::DiscardInput)
                }
            }
            State::AwaitReady { deadline_ms } if now_ms >= deadline_ms => {
                self.fail_before_writing(FailureKind::BootloaderEntryTimeout, None, None, now_ms)
            }
            State::AwaitVersion { deadline_ms, .. } if now_ms >= deadline_ms => {
                self.fail_before_writing(FailureKind::VersionUnreadable, None, None, now_ms)
            }
            State::AwaitLine { index, deadline_ms } if now_ms >= deadline_ms => {
                self.fail_writing(FailureKind::NoResponse, index, None);
                Actions::none()
            }
            State::AbortSettle { until_ms, outcome } if now_ms >= until_ms => {
                self.state = State::Done(outcome);
                Actions::none()
            }
            _ => Actions::none(),
        }
    }

    fn begin_fill_flush(&mut self, now_ms: u64) -> Actions {
        self.state = State::FillFlush {
            round: 0,
            until_ms: now_ms + self.timings.probe_spacing_ms,
        };
        Actions::one(FlashAction::Write(BARE_GET_ALL))
    }

    /// Fail before anything was written. From the bootloader-entry stage on,
    /// tell the TNC to go back to its old firmware (`R`); `settle` waits a
    /// moment before reporting.
    fn fail_before_writing(&mut self, kind: FailureKind, byte: Option<u8>, settle: Option<u64>, now_ms: u64) -> Actions {
        let outcome = FlashOutcome::Failed {
            kind,
            line: None,
            written: 0,
            byte,
        };
        let send_reset = !matches!(kind, FailureKind::SerialNeverQuiet);
        self.state = match settle {
            Some(ms) => State::AbortSettle {
                until_ms: now_ms + ms,
                outcome,
            },
            None => State::Done(outcome),
        };
        if send_reset {
            Actions::one(FlashAction::Write(RESET_PROBE))
        } else {
            Actions::none()
        }
    }

    /// Fail after writing started. Never sends `R`: the old firmware is gone.
    fn fail_writing(&mut self, kind: FailureKind, index: u32, byte: Option<u8>) {
        self.state = State::Done(FlashOutcome::Failed {
            kind,
            line: Some(index + 1),
            written: self.written,
            byte,
        });
    }

    fn on_line_reply(&mut self, index: u32, b: u8) -> Actions {
        match b {
            b'K' => {
                self.written += 1;
                if index + 1 < self.total_lines {
                    self.state = State::SendingLine { index: index + 1 };
                    Actions::one(FlashAction::SendLine {
                        index: index + 1,
                        paced: false,
                    })
                } else {
                    self.fail_writing(FailureKind::EndedWithoutCompletion, index, None);
                    Actions::none()
                }
            }
            b'Z' => {
                self.written += 1;
                self.state = State::Done(FlashOutcome::Done {
                    lines: self.written,
                    bootloader: self.bootloader,
                    was_stranded: self.was_stranded,
                });
                Actions::none()
            }
            other => {
                let kind = match other {
                    b'F' => FailureKind::FlashRejected,
                    b'N' => FailureKind::ChecksumRejected,
                    b'X' => FailureKind::InvalidCharacter,
                    _ => FailureKind::UnexpectedResponse,
                };
                self.fail_writing(kind, index, Some(other));
                Actions::none()
            }
        }
    }

    /// Where the flash is up to, in a few words.
    pub fn write_progress<W: fmt::Write + ?Sized>(&self, w: &mut W) -> fmt::Result {
        match self.state {
            State::Drain { second: false, .. } | State::StrandedProbe { .. } | State::FillFlush { .. } => {
                w.write_str("Waiting for the serial line to go quiet...")
            }
            State::Drain { second: true, .. } | State::AwaitReady { .. } | State::AwaitVersion { .. } => {
                w.write_str("Starting the TNC's bootloader...")
            }
            State::AbortSettle { .. } => w.write_str("Returning the TNC to its old firmware..."),
            State::SendingLine { index } | State::AwaitLine { index, .. } => {
                let pct = (index as u64 * 100) / self.total_lines.max(1) as u64;
                write!(
                    w,
                    "Writing line {} of {} ({pct}%). Do not power off the TNC.",
                    index + 1,
                    self.total_lines
                )
            }
            State::Done(outcome) => write_flash_outcome(&outcome, w),
        }
    }
}

fn chip_name(chip: ChipVariant) -> &'static str {
    match chip {
        ChipVariant::Dspic33Ep256 => "dsPIC33EP256GP (firmware 3.x)",
        ChipVariant::Dspic33Ep512 => "dsPIC33EP512GP (firmware 4.x)",
        ChipVariant::Unknown => "an unknown chip",
    }
}

/// The chip, as the page shows it.
pub fn write_chip<W: fmt::Write + ?Sized>(chip: ChipVariant, w: &mut W) -> fmt::Result {
    w.write_str(chip_name(chip))
}

/// Describe a finished flash in plain words.
pub fn write_flash_outcome<W: fmt::Write + ?Sized>(outcome: &FlashOutcome, w: &mut W) -> fmt::Result {
    const UNTOUCHED: &str = " Nothing was written; the TNC keeps its old firmware.";
    const STRANDED: &str = " The TNC is left in its bootloader (LEDs dark, no KISS). Run the \
update again to finish it; the node picks up a waiting bootloader.";
    match *outcome {
        FlashOutcome::Done { lines, was_stranded, .. } => {
            write!(w, "TNC firmware updated ({lines} lines written")?;
            if was_stranded {
                w.write_str(", finishing an interrupted update")?;
            }
            w.write_str("). The TNC is restarting.")
        }
        FlashOutcome::Failed { kind, line, written, byte } => {
            match kind {
                FailureKind::SerialNeverQuiet => w.write_str(
                    "The TNC kept sending data for 15 seconds, so the update could not start. \
Quieten the channel (turn the radio down or off) and try again.",
                )?,
                FailureKind::BootloaderEntryTimeout => w.write_str(
                    "The TNC did not start its bootloader. Check it is a NinoTNC with firmware \
2.20 or later.",
                )?,
                FailureKind::VersionUnreadable => {
                    w.write_str("The TNC's bootloader did not say which version it is.")?
                }
                FailureKind::VersionUnsupported => write!(
                    w,
                    "The TNC's bootloader answered with an unexpected byte (0x{:02X}).",
                    byte.unwrap_or(0)
                )?,
                FailureKind::ChipMismatch { bootloader } => write!(
                    w,
                    "This firmware file is for the other chip: the TNC has {}. Use the {} file.",
                    chip_name(bootloader),
                    if bootloader == ChipVariant::Dspic33Ep512 { "v4" } else { "v3" }
                )?,
                FailureKind::FlashRejected => write!(
                    w,
                    "The TNC reported a flash write failure at line {}.",
                    line.unwrap_or(0)
                )?,
                FailureKind::ChecksumRejected => write!(
                    w,
                    "The TNC rejected line {} as corrupt.",
                    line.unwrap_or(0)
                )?,
                FailureKind::InvalidCharacter => write!(
                    w,
                    "The TNC rejected line {} (invalid character).",
                    line.unwrap_or(0)
                )?,
                FailureKind::NoResponse => write!(
                    w,
                    "The TNC stopped answering at line {} ({written} written).",
                    line.unwrap_or(0)
                )?,
                FailureKind::UnexpectedResponse => write!(
                    w,
                    "The TNC answered line {} with an unexpected byte (0x{:02X}).",
                    line.unwrap_or(0),
                    byte.unwrap_or(0)
                )?,
                FailureKind::ImageUnreadable => write!(
                    w,
                    "The node could not read line {} of its stored copy of the file.",
                    line.unwrap_or(0)
                )?,
                FailureKind::EndedWithoutCompletion => w.write_str(
                    "Every line was accepted but the TNC never said it had finished.",
                )?,
            }
            w.write_str(if line.is_some() { STRANDED } else { UNTOUCHED })
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::string::String;
    use alloc::vec::Vec;

    /// A tiny image: two data lines, the 256GP fingerprint, the EOF record.
    const IMAGE_256: &str = ":020000040000fa\n\
:080000000002040000000000f2\n\
:102800002f08b000889fbe008a9fbe008c9fbe002c\n\
:00000001FF\n";

    fn stage(text: &[u8], chunk: usize) -> Result<(HexSummary, Vec<u8>), HexError> {
        let mut store = Vec::new();
        let mut sink = |r: &[u8]| {
            store.extend_from_slice(r);
            true
        };
        let mut s = HexStager::new();
        for c in text.chunks(chunk.max(1)) {
            s.push(c, &mut sink)?;
        }
        let summary = s.finish(&mut sink)?;
        Ok((summary, store))
    }

    fn replay(packed: &[u8]) -> String {
        let mut out = String::new();
        let mut at = 0;
        while at < packed.len() {
            let (n, upper) = record_header([packed[at], packed[at + 1]]);
            let mut line = [0u8; MAX_LINE + 1];
            let len = render_line(&packed[at + 2..at + 2 + n], upper, &mut line);
            out.push_str(core::str::from_utf8(&line[..len]).unwrap());
            at += 2 + n;
        }
        out
    }

    #[test]
    fn stages_and_replays_byte_for_byte_whatever_the_chunking() {
        for chunk in [1, 7, 64, 4096] {
            let (sum, packed) = stage(IMAGE_256.as_bytes(), chunk).unwrap();
            assert_eq!(sum.lines, 4);
            assert_eq!(sum.target, ChipVariant::Dspic33Ep256);
            assert_eq!(sum.packed_len as usize, packed.len());
            assert_eq!(sum.crc, crate::crc::compute(&packed));
            assert_eq!(replay(&packed), IMAGE_256, "the EOF record keeps its upper case");
        }
    }

    #[test]
    fn crlf_and_blank_lines_are_tolerated() {
        let crlf = IMAGE_256.replace('\n', "\r\n") + "\r\n";
        let (sum, packed) = stage(crlf.as_bytes(), 5).unwrap();
        assert_eq!(sum.lines, 4);
        assert_eq!(replay(&packed), IMAGE_256);
    }

    #[test]
    fn refuses_what_would_brick_or_strand_the_tnc() {
        let no_fp = ":020000040000fa\n:00000001FF\n";
        assert_eq!(stage(no_fp.as_bytes(), 64).unwrap_err(), HexError::UnknownTarget);
        let no_eof = ":102800002f08b000889fbe008a9fbe008c9fbe002c\n";
        assert_eq!(stage(no_eof.as_bytes(), 64).unwrap_err(), HexError::NoEndOfFile);
        let after = String::from(IMAGE_256) + ":020000040000fa\n";
        assert_eq!(
            stage(after.as_bytes(), 64).unwrap_err(),
            HexError::AfterEndOfFile { line: 5 }
        );
        assert_eq!(stage(b"", 64).unwrap_err(), HexError::Empty);
    }

    #[test]
    fn refuses_damaged_lines_naming_the_line() {
        let bad_sum = IMAGE_256.replace(":080000000002040000000000f2", ":080000000002040000000000f3");
        assert_eq!(stage(bad_sum.as_bytes(), 64).unwrap_err(), HexError::BadChecksum { line: 2 });
        let bad_len = IMAGE_256.replace(":020000040000fa", ":030000040000f9");
        assert_eq!(stage(bad_len.as_bytes(), 64).unwrap_err(), HexError::LengthMismatch { line: 1 });
        assert_eq!(
            stage(b"hello\n", 64).unwrap_err(),
            HexError::NoRecordMark { line: 1 }
        );
        assert_eq!(
            stage(b":0200000g0000fa\n", 64).unwrap_err(),
            HexError::BadHexDigit { line: 1 }
        );
        assert_eq!(
            stage(b":020000040000Fa\n", 64).unwrap_err(),
            HexError::MixedCase { line: 1 }
        );
        assert_eq!(
            stage(&[b':', 0xC0, b'\n'], 64).unwrap_err(),
            HexError::NotText { line: 1, byte: 0xC0 }
        );
        let long = String::from(":") + &"0".repeat(MAX_LINE) + "\n";
        assert_eq!(stage(long.as_bytes(), 64).unwrap_err(), HexError::LineTooLong { line: 1 });
    }

    #[test]
    fn a_full_store_is_reported() {
        let mut s = HexStager::new();
        let mut room = 20usize;
        let mut sink = |r: &[u8]| {
            if r.len() > room {
                return false;
            }
            room -= r.len();
            true
        };
        let r = s.push(IMAGE_256.as_bytes(), &mut sink);
        assert_eq!(r.unwrap_err(), HexError::TooLarge);
    }

    /// The real flashtnc images, when present (set NINOTNC_HEX_DIR to a
    /// flashtnc checkout): both stage, fingerprint to the right chip, and
    /// replay to exactly the file's bytes.
    #[test]
    fn real_flashtnc_images_round_trip_when_available() {
        let Ok(dir) = std::env::var("NINOTNC_HEX_DIR") else {
            return;
        };
        for (name, chip) in [
            ("N9600A-v3-44.hex", ChipVariant::Dspic33Ep256),
            ("N9600A-v4-44.hex", ChipVariant::Dspic33Ep512),
        ] {
            let text = std::fs::read(format!("{dir}/{name}")).unwrap();
            let (sum, packed) = stage(&text, 1460).unwrap();
            assert_eq!(sum.target, chip, "{name}");
            assert_eq!(replay(&packed).as_bytes(), &text[..], "{name}");
            std::println!("{name}: {} lines, {} bytes packed", sum.lines, sum.packed_len);
        }
    }

    /// The whole path on a real image (NINOTNC_HEX_DIR, as above): stage, then
    /// flash into a simulated bootloader that answers like the real one (`K`
    /// per line, `Z` on the end-of-file record, `N` on a bad line checksum) and
    /// records what it receives. It must receive exactly the file.
    #[test]
    fn a_real_image_flashes_into_a_simulated_bootloader_byte_for_byte() {
        let Ok(dir) = std::env::var("NINOTNC_HEX_DIR") else {
            return;
        };
        let text = std::fs::read(format!("{dir}/N9600A-v3-44.hex")).unwrap();
        let (sum, packed) = stage(&text, 1460).unwrap();

        // The packed records, by line.
        let mut records = Vec::new();
        let mut at = 0;
        while at < packed.len() {
            let (n, upper) = record_header([packed[at], packed[at + 1]]);
            records.push((packed[at + 2..at + 2 + n].to_vec(), upper));
            at += 2 + n;
        }

        let mut received: Vec<u8> = Vec::new();
        let mut now = 0u64;
        let (mut f, first) = BootloaderFlasher::start(sum.target, sum.lines, now, T);
        let mut pending: Vec<FlashAction> = acts(first);
        let mut in_bootloader = false;
        let mut reply: Option<u8> = None;
        let mut guard = 0u32;
        while f.outcome().is_none() {
            guard += 1;
            assert!(guard < 200_000, "stalled");
            for a in core::mem::take(&mut pending) {
                match a {
                    FlashAction::DiscardInput => reply = None,
                    FlashAction::Write(b) if b == BOOTLOADER_ENTRY => {
                        in_bootloader = true;
                        reply = Some(b'K');
                    }
                    FlashAction::Write(b) if b == VERSION_QUERY && in_bootloader => reply = Some(b'c'),
                    FlashAction::Write(_) => {} // R / GETALL: KISS firmware stays quiet here
                    FlashAction::SendLine { index, .. } => {
                        let (rec, upper) = &records[index as usize];
                        let mut line = [0u8; MAX_LINE + 1];
                        let n = render_line(rec, *upper, &mut line);
                        received.extend_from_slice(&line[..n]);
                        now += 5;
                        f.line_sent(now);
                        let sum: u8 = rec.iter().fold(0u8, |a, b| a.wrapping_add(*b));
                        reply = Some(if sum != 0 {
                            b'N'
                        } else if rec[3] == 0x01 {
                            b'Z'
                        } else {
                            b'K'
                        });
                    }
                }
            }
            if let Some(b) = reply.take() {
                now += 1;
                pending = acts(f.on_byte(b, now));
            } else if let Some(d) = f.deadline_ms() {
                now = now.max(d);
                pending = acts(f.on_time(now));
            }
        }
        assert_eq!(
            f.outcome(),
            Some(FlashOutcome::Done {
                lines: sum.lines,
                bootloader: b'c',
                was_stranded: false
            })
        );
        assert_eq!(received, text, "the bootloader received exactly the file");
    }

    // ── the bootloader conversation ──

    const T: FlashTimings = FlashTimings {
        read_timeout_ms: 5_000,
        drain_abort_ms: 15_000,
        probe_spacing_ms: 500,
        entry_timeout_ms: 15_000,
        version_timeout_ms: 5_000,
        first_line_char_delay_ms: 100,
        line_reply_timeout_ms: 15_000,
        reset_settle_ms: 1_000,
    };

    fn acts(a: Actions) -> Vec<FlashAction> {
        a.into_iter().collect()
    }

    /// Drive a fresh flasher through quiet-line, fill-and-flush and bootloader
    /// entry to the version query. Returns it and the clock.
    fn to_version_query(target: ChipVariant, lines: u32) -> (BootloaderFlasher, u64) {
        let (mut f, a) = BootloaderFlasher::start(target, lines, 0, T);
        assert_eq!(acts(a), [FlashAction::DiscardInput]);
        assert_eq!(f.deadline_ms(), Some(5_000));
        assert_eq!(acts(f.on_time(5_000)), [FlashAction::Write(RESET_PROBE)]);
        // No stranded bootloader: the probe times out.
        assert_eq!(acts(f.on_time(10_000)), [FlashAction::Write(BARE_GET_ALL)]);
        assert_eq!(
            acts(f.on_time(10_500)),
            [FlashAction::DiscardInput, FlashAction::Write(BARE_GET_ALL)]
        );
        assert_eq!(
            acts(f.on_time(11_000)),
            [FlashAction::DiscardInput, FlashAction::Write(BARE_GET_ALL)]
        );
        assert_eq!(acts(f.on_time(11_500)), [FlashAction::DiscardInput]);
        assert_eq!(acts(f.on_time(16_500)), [FlashAction::Write(BOOTLOADER_ENTRY)]);
        assert_eq!(acts(f.on_byte(b'K', 17_000)), [FlashAction::Write(VERSION_QUERY)]);
        (f, 17_000)
    }

    #[test]
    fn a_whole_flash_follows_flashtnc_step_by_step() {
        let (mut f, now) = to_version_query(ChipVariant::Dspic33Ep256, 3);
        assert!(!f.writing());
        assert_eq!(acts(f.on_byte(b'K', now)), [], "a late ready K is not the version");
        assert_eq!(
            acts(f.on_byte(b'd', now)),
            [FlashAction::SendLine { index: 0, paced: true }]
        );
        assert!(f.writing());
        assert_eq!(f.deadline_ms(), None, "no clock while the line is written");
        f.line_sent(now + 4_300);
        assert_eq!(
            acts(f.on_byte(b'K', now + 4_310)),
            [FlashAction::SendLine { index: 1, paced: false }]
        );
        f.line_sent(now + 4_320);
        assert_eq!(
            acts(f.on_byte(b'K', now + 4_330)),
            [FlashAction::SendLine { index: 2, paced: false }]
        );
        f.line_sent(now + 4_340);
        assert_eq!(acts(f.on_byte(b'Z', now + 4_350)), []);
        assert_eq!(
            f.outcome(),
            Some(FlashOutcome::Done {
                lines: 3,
                bootloader: b'd',
                was_stranded: false
            })
        );
    }

    #[test]
    fn a_stranded_bootloader_is_picked_up_without_the_entry_command() {
        let (mut f, _) = BootloaderFlasher::start(ChipVariant::Dspic33Ep512, 2, 0, T);
        f.on_time(5_000);
        assert_eq!(acts(f.on_byte(b'K', 5_010)), [FlashAction::Write(VERSION_QUERY)]);
        assert_eq!(
            acts(f.on_byte(b'D', 5_020)),
            [FlashAction::SendLine { index: 0, paced: true }]
        );
        f.line_sent(9_000);
        f.on_byte(b'K', 9_010);
        f.line_sent(9_020);
        f.on_byte(b'Z', 9_030);
        let mut s = String::new();
        f.write_progress(&mut s).unwrap();
        assert_eq!(
            s,
            "TNC firmware updated (2 lines written, finishing an interrupted update). The TNC \
is restarting."
        );
    }

    #[test]
    fn the_wrong_chip_is_refused_before_writing_and_the_tnc_released() {
        let (mut f, now) = to_version_query(ChipVariant::Dspic33Ep256, 3);
        assert_eq!(acts(f.on_byte(b'D', now)), [FlashAction::Write(RESET_PROBE)]);
        assert_eq!(f.outcome(), None, "settling first");
        assert_eq!(acts(f.on_time(now + 1_000)), []);
        let o = f.outcome().unwrap();
        assert!(o.nothing_written());
        let mut s = String::new();
        write_flash_outcome(&o, &mut s).unwrap();
        assert_eq!(
            s,
            "This firmware file is for the other chip: the TNC has dsPIC33EP512GP (firmware \
4.x). Use the v4 file. Nothing was written; the TNC keeps its old firmware."
        );
    }

    #[test]
    fn a_chattering_line_aborts_without_touching_the_tnc() {
        let (mut f, _) = BootloaderFlasher::start(ChipVariant::Dspic33Ep256, 3, 0, T);
        let mut now = 0;
        while f.outcome().is_none() {
            now += 1_000;
            let a = f.on_byte(0xC0, now);
            assert!(acts(a).is_empty(), "never sends R: nothing was started");
        }
        assert_eq!(now, 16_000);
        assert!(matches!(
            f.outcome(),
            Some(FlashOutcome::Failed { kind: FailureKind::SerialNeverQuiet, line: None, .. })
        ));
    }

    #[test]
    fn no_bootloader_answer_sends_r_and_reports_nothing_written() {
        let (mut f, _) = BootloaderFlasher::start(ChipVariant::Dspic33Ep256, 3, 0, T);
        f.on_time(5_000);
        f.on_time(10_000);
        f.on_time(10_500);
        f.on_time(11_000);
        f.on_time(11_500);
        f.on_time(16_500);
        assert_eq!(acts(f.on_time(31_500)), [FlashAction::Write(RESET_PROBE)]);
        assert!(matches!(
            f.outcome(),
            Some(FlashOutcome::Failed { kind: FailureKind::BootloaderEntryTimeout, .. })
        ));
    }

    #[test]
    fn a_failure_mid_write_never_sends_r_and_says_how_to_recover() {
        let (mut f, now) = to_version_query(ChipVariant::Dspic33Ep256, 5);
        f.on_byte(b'd', now);
        f.line_sent(now + 10);
        f.on_byte(b'K', now + 20);
        f.line_sent(now + 30);
        assert_eq!(acts(f.on_byte(b'N', now + 40)), []);
        let o = f.outcome().unwrap();
        assert!(!o.nothing_written());
        let mut s = String::new();
        write_flash_outcome(&o, &mut s).unwrap();
        assert!(s.starts_with("The TNC rejected line 2 as corrupt. The TNC is left in its bootloader"), "{s}");
    }

    #[test]
    fn silence_mid_write_times_out() {
        let (mut f, now) = to_version_query(ChipVariant::Dspic33Ep256, 5);
        f.on_byte(b'd', now);
        f.line_sent(now + 100);
        assert_eq!(f.deadline_ms(), Some(now + 15_100));
        f.on_time(now + 15_100);
        assert!(matches!(
            f.outcome(),
            Some(FlashOutcome::Failed { kind: FailureKind::NoResponse, line: Some(1), written: 0, .. })
        ));
    }

    #[test]
    fn k_on_the_last_line_is_not_completion() {
        let (mut f, now) = to_version_query(ChipVariant::Dspic33Ep256, 1);
        f.on_byte(b'd', now);
        f.line_sent(now + 10);
        f.on_byte(b'K', now + 20);
        assert!(matches!(
            f.outcome(),
            Some(FlashOutcome::Failed { kind: FailureKind::EndedWithoutCompletion, .. })
        ));
    }
}
