//! Bounded line assembler. Ports `Packet.Node.Core.Console.LineAssembler`.
//!
//! Feed inbound bytes chunk-by-chunk; pull complete lines (terminator stripped).
//! Splits on CR, LF, or CR-LF (telnet sends CR-LF, raw TCP sends LF, AX.25 sends
//! a bare CR — all resolve to "one line"). A line that reaches the cap without a
//! terminator is flushed truncated and the overflow tail dropped, so a peer that
//! never sends a terminator can't drive unbounded buffering. Backspace/DEL erase
//! the last buffered byte, keeping the assembled line in step with server echo.

use alloc::vec::Vec;

/// Default cap on a single buffered line — mirrors
/// `NodeCommandParser.MaxLineLength` in the C# host (a callsign is ≤ 9 chars;
/// 512 is generous).
pub const DEFAULT_MAX_LINE_LEN: usize = 512;

/// Reassembles a byte stream into bounded lines.
#[derive(Debug)]
pub struct LineAssembler {
    max_line_len: usize,
    buffer: Vec<u8>,
    overflowing: bool,
    last_was_cr: bool,
}

impl Default for LineAssembler {
    fn default() -> Self {
        Self::new(DEFAULT_MAX_LINE_LEN)
    }
}

impl LineAssembler {
    /// Create an assembler with the given cap (0 falls back to the default).
    pub fn new(max_line_len: usize) -> Self {
        Self {
            max_line_len: if max_line_len > 0 {
                max_line_len
            } else {
                DEFAULT_MAX_LINE_LEN
            },
            buffer: Vec::new(),
            overflowing: false,
            last_was_cr: false,
        }
    }

    /// Feed a chunk; returns every complete line it completed (terminator
    /// stripped). Each line is the raw bytes — the parser decodes/validates them.
    pub fn push(&mut self, chunk: &[u8]) -> Vec<Vec<u8>> {
        let mut lines = Vec::new();
        for &b in chunk {
            // Coalesce CR-LF: an LF right after a CR is swallowed.
            if b == b'\n' && self.last_was_cr {
                self.last_was_cr = false;
                continue;
            }
            self.last_was_cr = b == b'\r';

            if b == b'\r' || b == b'\n' {
                lines.push(core::mem::take(&mut self.buffer));
                self.overflowing = false;
                continue;
            }

            // Backspace / DEL line editing.
            if b == 0x08 || b == 0x7f {
                if !self.overflowing {
                    self.buffer.pop();
                }
                continue;
            }

            if self.overflowing {
                continue; // dropping the overflow tail until the next terminator
            }

            if self.buffer.len() >= self.max_line_len {
                lines.push(core::mem::take(&mut self.buffer));
                self.overflowing = true;
                continue;
            }

            self.buffer.push(b);
        }
        lines
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn s(v: &[u8]) -> &str {
        core::str::from_utf8(v).unwrap()
    }

    #[test]
    fn splits_on_lf() {
        let mut a = LineAssembler::default();
        let lines = a.push(b"hello\nworld\n");
        assert_eq!(lines.len(), 2);
        assert_eq!(s(&lines[0]), "hello");
        assert_eq!(s(&lines[1]), "world");
    }

    #[test]
    fn splits_on_bare_cr() {
        let mut a = LineAssembler::default();
        let lines = a.push(b"C M0LTE\r");
        assert_eq!(lines.len(), 1);
        assert_eq!(s(&lines[0]), "C M0LTE");
    }

    #[test]
    fn coalesces_crlf() {
        let mut a = LineAssembler::default();
        let lines = a.push(b"one\r\ntwo\r\n");
        assert_eq!(lines.len(), 2);
        assert_eq!(s(&lines[0]), "one");
        assert_eq!(s(&lines[1]), "two");
    }

    #[test]
    fn crlf_split_across_chunks() {
        let mut a = LineAssembler::default();
        assert_eq!(a.push(b"line\r").len(), 1); // CR ends the line
        let lines = a.push(b"\nnext\r"); // the LF must be swallowed, not a new empty line
        assert_eq!(lines.len(), 1);
        assert_eq!(s(&lines[0]), "next");
    }

    #[test]
    fn empty_line_yielded() {
        let mut a = LineAssembler::default();
        let lines = a.push(b"\n");
        assert_eq!(lines.len(), 1);
        assert!(lines[0].is_empty());
    }

    #[test]
    fn backspace_erases() {
        let mut a = LineAssembler::default();
        let lines = a.push(b"CONXY\x08\x08NECT\r");
        assert_eq!(s(&lines[0]), "CONNECT");
    }

    #[test]
    fn over_long_line_is_truncated_and_tail_dropped() {
        let mut a = LineAssembler::new(4);
        // "abcdef\nok\n" with cap 4: 'a'..'d' buffer (4), 'e' flushes "abcd" and
        // enters overflow, 'f' dropped, '\n' clears overflow and yields the (now
        // empty) buffer, then "ok" then its '\n'. So: ["abcd", "", "ok"] — the
        // overflow tail "ef" never reaches the parser. This matches the C#
        // LineAssembler (its terminator branch also yields the cleared buffer).
        let lines = a.push(b"abcdef\nok\n");
        assert_eq!(lines.len(), 3);
        assert_eq!(s(&lines[0]), "abcd");
        assert!(lines[1].is_empty());
        assert_eq!(s(&lines[2]), "ok");
    }

    #[test]
    fn chunked_push_assembles_one_line() {
        let mut a = LineAssembler::default();
        assert!(a.push(b"C M").is_empty());
        assert!(a.push(b"0LT").is_empty());
        let lines = a.push(b"E\r");
        assert_eq!(s(&lines[0]), "C M0LTE");
    }
}
