//! Stateful streaming KISS decoder. Ports `Packet.Kiss.KissDecoder`.
//!
//! Push received bytes as they arrive (any chunk size); pull completed frames out.
//! Escape state and the in-progress frame buffer persist across calls, so the
//! firmware can feed it straight from a UART/TCP read of arbitrary length.
//!
//! Behaviour matches the C# decoder: empty inter-frame FENDs (the common re-sync
//! prefix) are dropped, malformed escape sequences drop the offending byte
//! leniently, and a frame must carry at least the command byte.
//!
//! `alloc`-gated because the in-progress frame is a growable `Vec`. A heapless
//! follow-up (fixed `MAX_FRAME` cap, dropping over-long frames) is noted in the
//! module roadmap — the decode *logic* is unchanged, only the buffer type.

use super::frame::{Command, Frame, FEND, FESC, TFEND, TFESC};
use alloc::vec::Vec;

/// A streaming KISS decoder. Construct with [`Decoder::new`], feed [`Decoder::push`].
#[derive(Debug, Default)]
pub struct Decoder {
    current: Vec<u8>,
    in_escape: bool,
}

impl Decoder {
    /// Create an empty decoder.
    pub fn new() -> Self {
        Self {
            current: Vec::with_capacity(256),
            in_escape: false,
        }
    }

    /// Push a chunk of received bytes. Returns every frame the chunk completed
    /// (possibly none, possibly several).
    pub fn push(&mut self, bytes: &[u8]) -> Vec<Frame> {
        let mut frames = Vec::new();
        for &b in bytes {
            if self.in_escape {
                self.in_escape = false;
                match b {
                    TFEND => self.current.push(FEND),
                    TFESC => self.current.push(FESC),
                    // Lenient: drop a malformed escape byte and carry on.
                    _ => {}
                }
                continue;
            }

            match b {
                FEND => {
                    if !self.current.is_empty() {
                        if let Some(frame) = self.finish() {
                            frames.push(frame);
                        }
                        self.current.clear();
                    }
                    // else: empty inter-frame FEND, ignore.
                }
                FESC => self.in_escape = true,
                _ => self.current.push(b),
            }
        }
        frames
    }

    /// Discard any partially-decoded frame state.
    pub fn reset(&mut self) {
        self.current.clear();
        self.in_escape = false;
    }

    fn finish(&self) -> Option<Frame> {
        // Need at least a command byte.
        let command_byte = *self.current.first()?;
        let port = (command_byte >> 4) & 0x0F;
        let command = Command::from_nibble(command_byte & 0x0F);
        let payload = self.current[1..].to_vec();
        Some(Frame::new(port, command, payload))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kiss::encoder::encode;

    #[test]
    fn decodes_single_data_frame() {
        let mut d = Decoder::new();
        let frames = d.push(&[FEND, 0x00, 0xDE, 0xAD, FEND]);
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].port, 0);
        assert_eq!(frames[0].command, Command::Data);
        assert_eq!(frames[0].payload, vec![0xDE, 0xAD]);
    }

    #[test]
    fn unescapes_transposed_bytes() {
        let mut d = Decoder::new();
        let frames = d.push(&[FEND, 0x00, FESC, TFEND, FESC, TFESC, FEND]);
        assert_eq!(frames[0].payload, vec![FEND, FESC]);
    }

    #[test]
    fn handles_split_chunks() {
        let mut d = Decoder::new();
        assert!(d.push(&[FEND, 0x00, 0x11]).is_empty());
        assert!(d.push(&[0x22]).is_empty());
        let frames = d.push(&[0x33, FEND]);
        assert_eq!(frames[0].payload, vec![0x11, 0x22, 0x33]);
    }

    #[test]
    fn split_escape_across_chunks() {
        let mut d = Decoder::new();
        assert!(d.push(&[FEND, 0x00, FESC]).is_empty()); // escape pending across the boundary
        let frames = d.push(&[TFEND, FEND]);
        assert_eq!(frames[0].payload, vec![FEND]);
    }

    #[test]
    fn drops_empty_interframe_fends() {
        let mut d = Decoder::new();
        let frames = d.push(&[FEND, FEND, FEND, 0x00, 0x42, FEND]);
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].payload, vec![0x42]);
    }

    #[test]
    fn extracts_port_from_high_nibble() {
        let mut d = Decoder::new();
        let frames = d.push(&[FEND, 0x70, 0x01, FEND]); // port 7, Data
        assert_eq!(frames[0].port, 7);
        assert_eq!(frames[0].command, Command::Data);
    }

    #[test]
    fn malformed_escape_is_dropped_leniently() {
        let mut d = Decoder::new();
        // FESC followed by a non-transpose byte: drop the 0x99, keep going.
        let frames = d.push(&[FEND, 0x00, FESC, 0x99, 0x55, FEND]);
        assert_eq!(frames[0].payload, vec![0x55]);
    }

    #[test]
    fn two_frames_in_one_push() {
        let mut d = Decoder::new();
        let frames = d.push(&[FEND, 0x00, 0x01, FEND, FEND, 0x00, 0x02, FEND]);
        assert_eq!(frames.len(), 2);
        assert_eq!(frames[0].payload, vec![0x01]);
        assert_eq!(frames[1].payload, vec![0x02]);
    }

    #[test]
    fn encode_then_decode_round_trips_with_escapes() {
        // The load-bearing property: anything encode() emits, decode() recovers
        // exactly — including payloads full of FEND/FESC bytes.
        let payload: Vec<u8> = vec![FEND, 0x00, FESC, 0xC0, 0xDB, 0xDC, 0xDD, 0x42];
        let wire = encode(3, Command::Data, &payload).unwrap();
        let mut d = Decoder::new();
        let frames = d.push(&wire);
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].port, 3);
        assert_eq!(frames[0].command, Command::Data);
        assert_eq!(frames[0].payload, payload);
    }
}
