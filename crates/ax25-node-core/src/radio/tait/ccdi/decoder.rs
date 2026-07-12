//! The CCDI read-pump line discipline as a stateful streaming decoder.
//!
//! Ports the byte loop of `TaitCcdiRadio.PumpReads` (TaitCcdiRadio.cs:1189-1212):
//! CCDI is **CR-delimited ASCII lines** with a bare `.` prompt at column 0 and
//! XON/XOFF (`0x11`/`0x13`) software-flow-control bytes to skip. This is a distinct
//! state machine from the KISS SLIP [`Decoder`](crate::kiss::Decoder) — no escape
//! bytes, no FEND framing — so it is its own type.
//!
//! Push received bytes as they arrive (any chunk size); pull out the completed
//! [`CcdiEvent`]s. Line-buffer state persists across calls, so the firmware can feed
//! it straight from a UART/TCP read of arbitrary length. `alloc`-gated returns,
//! mirroring the KISS decoder; the fixed line buffer is a const-sized struct field.

use super::frame::MAX_LINE;
use alloc::vec::Vec;

/// One event pulled from the CCDI byte stream.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CcdiEvent {
    /// A bare `.` prompt at the start of a line (the radio's ready/idle marker,
    /// used to complete prompt-terminated transactions).
    Prompt,
    /// A complete CR-terminated line (the CR is stripped). The bytes are the raw
    /// on-wire line — feed to [`CcdiFrame::try_parse`](super::CcdiFrame::try_parse);
    /// line noise is normal and simply fails the parse.
    Line(Vec<u8>),
}

/// A streaming CCDI line assembler. Construct with [`LineDecoder::new`], feed
/// [`LineDecoder::push`].
#[derive(Debug)]
pub struct LineDecoder {
    line: [u8; MAX_LINE],
    len: usize,
    /// A line that overran [`MAX_LINE`] before its CR: it can only be noise (no
    /// valid CCDI line exceeds the max), so it is dropped rather than emitted.
    overflow: bool,
}

impl Default for LineDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl LineDecoder {
    /// Create an empty line decoder.
    pub fn new() -> Self {
        Self {
            line: [0u8; MAX_LINE],
            len: 0,
            overflow: false,
        }
    }

    /// Push a chunk of received bytes. Returns every event the chunk completed
    /// (possibly none, possibly several). Mirrors the `PumpReads` byte switch.
    pub fn push(&mut self, bytes: &[u8]) -> Vec<CcdiEvent> {
        let mut events = Vec::new();
        for &c in bytes {
            match c {
                b'\r' => {
                    if self.len > 0 || self.overflow {
                        if !self.overflow {
                            events.push(CcdiEvent::Line(self.line[..self.len].to_vec()));
                        }
                        self.reset();
                    }
                }
                // LF, plus XON/XOFF: the link may use software flow control (§1.6.1).
                b'\n' | 0x11 | 0x13 => {}
                b'.' if self.len == 0 => events.push(CcdiEvent::Prompt),
                _ => {
                    if self.len < MAX_LINE {
                        self.line[self.len] = c;
                        self.len += 1;
                    } else {
                        // Over-long line: can only be noise; drop it (and the rest of
                        // the line) until the next CR.
                        self.overflow = true;
                    }
                }
            }
        }
        events
    }

    /// Discard any partially-assembled line state.
    pub fn reset(&mut self) {
        self.len = 0;
        self.overflow = false;
    }
}

#[cfg(test)]
mod tests {
    use super::super::{CcdiFrame, CcdiMessage, CcdiProgressType};
    use super::*;

    #[test]
    fn assembles_a_cr_terminated_line() {
        let mut d = LineDecoder::new();
        let events = d.push(b"j07064-456C9\r");
        assert_eq!(events, alloc::vec![CcdiEvent::Line(b"j07064-456C9".to_vec())]);
    }

    #[test]
    fn emits_prompt_for_bare_dot_at_column_zero() {
        let mut d = LineDecoder::new();
        assert_eq!(d.push(b"."), alloc::vec![CcdiEvent::Prompt]);
        // A dot mid-line is a normal character, not a prompt.
        let events = d.push(b"m0813203.02A2\r");
        assert_eq!(events, alloc::vec![CcdiEvent::Line(b"m0813203.02A2".to_vec())]);
    }

    #[test]
    fn skips_lf_and_xon_xoff() {
        let mut d = LineDecoder::new();
        // XOFF (0x13) then XON (0x11) and LF interleaved with the line bytes.
        let mut stream = alloc::vec![0x13u8];
        stream.extend_from_slice(b"p02");
        stream.push(0x11);
        stream.extend_from_slice(b"05C9\r\n");
        let events = d.push(&stream);
        assert_eq!(events, alloc::vec![CcdiEvent::Line(b"p0205C9".to_vec())]);
    }

    #[test]
    fn reassembles_across_chunked_reads() {
        // Mirror the serial.rs chunked-feed strategy: split a line (and a prompt)
        // across several reads.
        let mut d = LineDecoder::new();
        assert!(d.push(b"j070").is_empty());
        assert!(d.push(b"64-4").is_empty());
        let events = d.push(b"56C9\r.");
        assert_eq!(
            events,
            alloc::vec![CcdiEvent::Line(b"j07064-456C9".to_vec()), CcdiEvent::Prompt]
        );
    }

    #[test]
    fn multiple_lines_and_prompts_in_one_push() {
        let mut d = LineDecoder::new();
        // The radio's ".e03006A2\r." pattern: prompt, error line, prompt.
        let events = d.push(b".e03001A7\r.");
        assert_eq!(
            events,
            alloc::vec![
                CcdiEvent::Prompt,
                CcdiEvent::Line(b"e03001A7".to_vec()),
                CcdiEvent::Prompt
            ]
        );
    }

    #[test]
    fn over_long_noise_line_is_dropped_not_emitted() {
        let mut d = LineDecoder::new();
        let noise = alloc::vec![b'x'; MAX_LINE + 50];
        assert!(d.push(&noise).is_empty());
        // A clean line after the overflow line's CR decodes normally.
        let events = d.push(b"\rq002F\r");
        assert_eq!(events, alloc::vec![CcdiEvent::Line(b"q002F".to_vec())]);
    }

    #[test]
    fn empty_cr_produces_no_event() {
        let mut d = LineDecoder::new();
        assert!(d.push(b"\r\r\r").is_empty());
    }

    #[test]
    fn end_to_end_line_to_typed_message() {
        // The load-bearing seam: bytes off the wire → line → frame → typed message.
        let mut d = LineDecoder::new();
        let events = d.push(b"p0205C9\r");
        let line = match &events[0] {
            CcdiEvent::Line(b) => b,
            other => panic!("expected line, got {other:?}"),
        };
        let frame = CcdiFrame::try_parse(line).unwrap();
        assert!(matches!(
            CcdiMessage::decode(&frame),
            CcdiMessage::Progress {
                ptype: CcdiProgressType::ReceiverBusy,
                ..
            }
        ));
    }
}
