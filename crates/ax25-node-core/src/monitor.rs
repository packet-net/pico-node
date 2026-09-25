//! A traffic monitor: a fixed-size log of recent frames and events, and the
//! one-line text form frames are shown in.
//!
//! The log is allocation-free and `const`-constructible, so the firmware can
//! keep it in a `static` rather than on its small heap. Every entry carries a
//! monotonically increasing sequence number so a polling client can ask for
//! "everything after N" and notice when it has fallen behind (the oldest
//! sequence still held is greater than the one it last saw).
//!
//! Frames render in the form most packet monitors use:
//!
//! ```text
//! M0ABC-1>IDENT,RELAY*: <UI C> hello
//! G4XYZ>M0ABC-1: <I C P R3 S2 pid=F0> data
//! ```

use core::fmt;

/// Which way an entry went, or whether it is a TNC event rather than a frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    /// Heard from the radio.
    Rx,
    /// Sent to the radio.
    Tx,
    /// A control command, TNC report or other event on the link.
    Info,
}

impl Direction {
    /// A short tag for display (`RX`, `TX`, `--`).
    pub fn tag(self) -> &'static str {
        match self {
            Direction::Rx => "RX",
            Direction::Tx => "TX",
            Direction::Info => "--",
        }
    }
}

/// One log entry: up to `W` bytes of printable ASCII text.
#[derive(Debug, Clone, Copy)]
pub struct Entry<const W: usize> {
    seq: u32,
    at_ms: u64,
    dir: Direction,
    len: u16,
    text: [u8; W],
}

impl<const W: usize> Entry<W> {
    const EMPTY: Self = Self {
        seq: 0,
        at_ms: 0,
        // Rx is the zero discriminant: an all-zero EMPTY keeps a static log in .bss.
        dir: Direction::Rx,
        len: 0,
        text: [0; W],
    };

    /// The entry's sequence number (starts at 1).
    pub fn seq(&self) -> u32 {
        self.seq
    }

    /// When it was logged, in the caller's millisecond clock.
    pub fn at_ms(&self) -> u64 {
        self.at_ms
    }

    /// Direction / kind.
    pub fn direction(&self) -> Direction {
        self.dir
    }

    /// The text (printable ASCII; always valid UTF-8).
    pub fn text(&self) -> &str {
        core::str::from_utf8(&self.text[..self.len as usize]).unwrap_or("")
    }
}

/// A ring of the last `N` entries, each up to `W` bytes.
pub struct MonitorLog<const N: usize, const W: usize> {
    entries: [Entry<W>; N],
    /// The newest sequence number held (0 = empty). Kept so a fresh log is all
    /// zero bytes, which lets a `static` of it live in zeroed RAM rather than
    /// being copied from flash at boot.
    newest: u32,
}

impl<const N: usize, const W: usize> Default for MonitorLog<N, W> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize, const W: usize> MonitorLog<N, W> {
    /// An empty log.
    pub const fn new() -> Self {
        Self {
            entries: [Entry::EMPTY; N],
            newest: 0,
        }
    }

    /// Append an entry whose text is produced by `write`. Text past `W` bytes
    /// is cut and marked with a trailing `~`; anything outside printable ASCII
    /// becomes `.`. Returns the new entry's sequence number.
    pub fn push_with(
        &mut self,
        dir: Direction,
        at_ms: u64,
        write: impl FnOnce(&mut dyn fmt::Write) -> fmt::Result,
    ) -> u32 {
        let seq = self.next_seq();
        self.newest = seq;
        let slot = &mut self.entries[(seq as usize) % N];
        let mut w = TruncatingWriter {
            buf: &mut slot.text,
            len: 0,
            truncated: false,
        };
        // A formatting error only means the text was cut short; keep what we have.
        let _ = write(&mut w);
        let (len, truncated) = (w.len, w.truncated);
        if truncated && len > 0 {
            slot.text[len - 1] = b'~';
        }
        slot.len = len as u16;
        slot.seq = seq;
        slot.at_ms = at_ms;
        slot.dir = dir;
        seq
    }

    /// Append a plain-text entry.
    pub fn push_str(&mut self, dir: Direction, at_ms: u64, text: &str) -> u32 {
        self.push_with(dir, at_ms, |w| w.write_str(text))
    }

    /// The sequence number the next entry will get (so the newest held is one
    /// less). A client that has seen everything polls with this minus one.
    pub fn next_seq(&self) -> u32 {
        self.newest.wrapping_add(1).max(1)
    }

    /// The oldest sequence number still held, or `None` if the log is empty.
    pub fn oldest_seq(&self) -> Option<u32> {
        let newest = self.newest;
        if newest == 0 {
            return None;
        }
        Some(if (newest as usize) > N {
            newest - N as u32 + 1
        } else {
            1
        })
    }

    /// Entries with a sequence number greater than `after`, oldest first.
    pub fn since(&self, after: u32) -> impl Iterator<Item = &Entry<W>> + '_ {
        let start = match self.oldest_seq() {
            Some(oldest) => oldest.max(after.saturating_add(1)),
            None => self.next_seq(),
        };
        (start..self.next_seq()).map(move |seq| &self.entries[(seq as usize) % N])
    }
}

/// A `fmt::Write` over a fixed buffer that keeps only printable ASCII and stops
/// quietly when full.
struct TruncatingWriter<'a> {
    buf: &'a mut [u8],
    len: usize,
    truncated: bool,
}

impl fmt::Write for TruncatingWriter<'_> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for &b in s.as_bytes() {
            if self.len == self.buf.len() {
                self.truncated = true;
                return Ok(());
            }
            self.buf[self.len] = if (0x20..0x7F).contains(&b) { b } else { b'.' };
            self.len += 1;
        }
        Ok(())
    }
}

/// Write `info` as monitor text: printable ASCII kept, CR/LF shown as a
/// space-separated break, everything else as `.`.
pub fn write_payload_text(info: &[u8], w: &mut dyn fmt::Write) -> fmt::Result {
    let mut last_was_break = false;
    for &b in info {
        let c = match b {
            b'\r' | b'\n' => {
                if last_was_break {
                    continue;
                }
                last_was_break = true;
                ' '
            }
            0x20..=0x7E => {
                last_was_break = false;
                b as char
            }
            _ => {
                last_was_break = false;
                '.'
            }
        };
        w.write_char(c)?;
    }
    Ok(())
}

#[cfg(feature = "alloc")]
mod frame_text {
    use core::fmt;

    use crate::ax25::frame::PID_NO_LAYER3;
    use crate::ax25::{Callsign, Frame};

    fn write_call(call: &Callsign, w: &mut dyn fmt::Write) -> fmt::Result {
        let mut buf = [0u8; 12];
        let n = call.write_display(&mut buf).unwrap_or(0);
        w.write_str(core::str::from_utf8(&buf[..n]).unwrap_or("?"))
    }

    /// Write a decoded AX.25 frame in the one-line monitor form:
    /// `SRC>DEST,DIGI*: <TYPE C/R flags> info`. The control field is read
    /// modulo-8. NET/ROM (PID `CF`) info is binary, so it is summarised by length
    /// rather than dumped.
    pub fn write_frame(frame: &Frame, w: &mut dyn fmt::Write) -> fmt::Result {
        write_call(&frame.source.callsign, w)?;
        w.write_char('>')?;
        write_call(&frame.destination.callsign, w)?;
        for digi in &frame.digipeaters {
            w.write_char(',')?;
            write_call(&digi.callsign, w)?;
            if digi.crh {
                w.write_char('*')?;
            }
        }
        w.write_str(": <")?;

        let c = frame.control;
        let pf = frame.poll_final();
        let cr = if frame.is_command() {
            "C"
        } else if frame.is_response() {
            "R"
        } else {
            "V1"
        };
        if frame.is_information() {
            write!(w, "I {cr}")?;
            if pf {
                w.write_str(" P")?;
            }
            write!(w, " R{} S{}", frame.nr_with(None), frame.ns_with(None))?;
        } else if frame.is_supervisory() {
            let name = match c & 0x0F {
                0x01 => "RR",
                0x05 => "RNR",
                0x09 => "REJ",
                0x0D => "SREJ",
                _ => "S?",
            };
            write!(w, "{name} {cr}")?;
            if pf {
                w.write_str(if frame.is_command() { " P" } else { " F" })?;
            }
            write!(w, " R{}", frame.nr_with(None))?;
        } else {
            let name = match c & 0xEF {
                0x03 => "UI",
                0x2F => "SABM",
                0x6F => "SABME",
                0x43 => "DISC",
                0x0F => "DM",
                0x63 => "UA",
                0x87 => "FRMR",
                0xAF => "XID",
                0xE3 => "TEST",
                _ => "U?",
            };
            write!(w, "{name} {cr}")?;
            if pf {
                w.write_str(if frame.is_command() { " P" } else { " F" })?;
            }
        }
        if let Some(pid) = frame.pid {
            if pid != PID_NO_LAYER3 {
                write!(w, " pid={pid:02X}")?;
            }
        }
        w.write_char('>')?;

        if frame.info.is_empty() {
            return Ok(());
        }
        if frame.pid == Some(crate::ax25::frame::PID_NETROM) {
            return write!(w, " NET/ROM, {} bytes", frame.info.len());
        }
        w.write_char(' ')?;
        super::write_payload_text(&frame.info, w)
    }
}

#[cfg(feature = "alloc")]
pub use frame_text::write_frame;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ax25::frame::{CONTROL_UI, PID_NETROM};
    use crate::ax25::{Address, Callsign, Frame};
    use alloc::string::String;
    use alloc::vec;
    use alloc::vec::Vec;

    fn addr(call: &str, crh: bool) -> Address {
        Address {
            callsign: Callsign::parse(call).unwrap(),
            crh,
            extension: false,
        }
    }

    fn ui(info: &[u8]) -> Frame {
        Frame {
            destination: addr("IDENT", true),
            source: addr("M0ABC-1", false),
            digipeaters: Vec::new(),
            control: CONTROL_UI,
            pid: Some(0xF0),
            info: info.to_vec(),
        }
    }

    fn render(f: &Frame) -> String {
        let mut s = String::new();
        write_frame(f, &mut s).unwrap();
        s
    }

    #[test]
    fn ui_frame_renders_in_monitor_form() {
        assert_eq!(render(&ui(b"hello\r")), "M0ABC-1>IDENT: <UI C> hello ");
    }

    #[test]
    fn digipeaters_show_the_repeated_mark() {
        let mut f = ui(b"x");
        f.digipeaters = vec![addr("RELAY", true), addr("WIDE2-1", false)];
        assert_eq!(render(&f), "M0ABC-1>IDENT,RELAY*,WIDE2-1: <UI C> x");
    }

    #[test]
    fn i_frame_shows_counters_poll_and_non_default_pid() {
        let f = Frame {
            destination: addr("M0ABC-1", true),
            source: addr("G4XYZ", false),
            digipeaters: Vec::new(),
            // N(R)=3, P=1, N(S)=2, I frame.
            control: (3 << 5) | 0x10 | (2 << 1),
            pid: Some(0xCC),
            info: b"data".to_vec(),
        };
        assert_eq!(render(&f), "G4XYZ>M0ABC-1: <I C P R3 S2 pid=CC> data");
    }

    #[test]
    fn supervisory_and_unnumbered_frames_are_named() {
        let mut rr = ui(b"");
        rr.destination.crh = false;
        rr.source.crh = true;
        rr.control = 0x01 | (5 << 5) | 0x10;
        rr.pid = None;
        assert_eq!(render(&rr), "M0ABC-1>IDENT: <RR R F R5>");

        let mut sabm = ui(b"");
        sabm.control = 0x3F;
        sabm.pid = None;
        assert_eq!(render(&sabm), "M0ABC-1>IDENT: <SABM C P>");
    }

    #[test]
    fn netrom_payload_is_summarised_not_dumped() {
        let mut f = ui(&[0xFF, 0x00, 0x01]);
        f.pid = Some(PID_NETROM);
        assert_eq!(render(&f), "M0ABC-1>IDENT: <UI C pid=CF> NET/ROM, 3 bytes");
    }

    #[test]
    fn log_keeps_the_last_n_and_reports_since() {
        let mut log: MonitorLog<3, 16> = MonitorLog::new();
        assert_eq!(log.oldest_seq(), None);
        assert_eq!(log.since(0).count(), 0);
        for i in 0..5u64 {
            log.push_with(Direction::Rx, i * 10, |w| write!(w, "frame {i}"));
        }
        assert_eq!(log.next_seq(), 6);
        assert_eq!(log.oldest_seq(), Some(3));
        let texts: Vec<_> = log.since(0).map(|e| (e.seq(), e.text())).collect();
        assert_eq!(texts, vec![(3, "frame 2"), (4, "frame 3"), (5, "frame 4")]);
        let after4: Vec<_> = log.since(4).map(|e| e.seq()).collect();
        assert_eq!(after4, vec![5]);
        assert_eq!(log.since(5).count(), 0);
        assert_eq!(log.since(99).count(), 0);
    }

    #[test]
    fn long_or_binary_text_is_cut_and_cleaned() {
        let mut log: MonitorLog<2, 8> = MonitorLog::new();
        log.push_str(Direction::Tx, 0, "0123456789");
        log.push_with(Direction::Info, 0, |w| write_payload_text(&[b'a', 0x00, 0xC0, b'b'], w));
        let e: Vec<_> = log.since(0).map(|e| e.text()).collect();
        assert_eq!(e, vec!["0123456~", "a..b"]);
    }
}
