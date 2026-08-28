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
//! The in-progress buffer is bounded by [`DEFAULT_MAX_FRAME_LEN`] (configurable
//! via [`Decoder::with_max_frame_len`]). KISS has no length field; a frame ends
//! at the next FEND, so a peer (or a noise burst, or a mis-set baud rate) that
//! never sends one would otherwise grow the buffer without limit, and `clear`
//! keeps the grown capacity, so the memory stayed retained afterwards
//! (packet-net/packet.net#696). Over the bound the partial frame is dropped, the
//! buffer capacity released, and bytes are discarded until the next FEND
//! resynchronises the stream. Mirrors the C# `KissDecoder` bound.
//!
//! `alloc`-gated because the in-progress frame is a growable `Vec`.

use super::frame::{Command, Frame, FEND, FESC, TFEND, TFESC};
use alloc::vec::Vec;

/// Default bound on a single KISS frame's decoded length, in octets. Comfortably
/// above anything AX.25 produces - a maximum-size frame is 8 digipeaters (56) +
/// addresses/control/PID (18) + the §6.7.2 maximum N1 of 256, so ~330 - while
/// still small enough that a frameless stream cannot exhaust memory. Mirrors
/// `KissDecoder.DefaultMaxFrameLength`.
pub const DEFAULT_MAX_FRAME_LEN: usize = 4096;

/// Initial (and post-release) capacity of the in-progress frame buffer. Mirrors
/// `KissDecoder.InitialCapacity`.
const INITIAL_CAPACITY: usize = 256;

/// A streaming KISS decoder. Construct with [`Decoder::new`], feed [`Decoder::push`].
#[derive(Debug)]
pub struct Decoder {
    current: Vec<u8>,
    in_escape: bool,
    max_frame_len: usize,
    /// True while discarding a stream we have lost sync with (an oversize frame):
    /// every byte is dropped until the next FEND starts a fresh frame.
    resynchronising: bool,
    oversize_frames_dropped: u64,
}

impl Default for Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl Decoder {
    /// Create an empty decoder bounded by [`DEFAULT_MAX_FRAME_LEN`].
    pub fn new() -> Self {
        Self::with_max_frame_len(DEFAULT_MAX_FRAME_LEN)
    }

    /// Create an empty decoder with an explicit maximum frame length, in octets
    /// (0 falls back to the default). Mirrors `KissDecoder(int maxFrameLength)`.
    pub fn with_max_frame_len(max_frame_len: usize) -> Self {
        Self {
            current: Vec::with_capacity(INITIAL_CAPACITY),
            in_escape: false,
            max_frame_len: if max_frame_len > 0 {
                max_frame_len
            } else {
                DEFAULT_MAX_FRAME_LEN
            },
            resynchronising: false,
            oversize_frames_dropped: 0,
        }
    }

    /// How many partial frames have been discarded for exceeding the maximum
    /// frame length. A non-zero, growing count means the stream is not KISS
    /// (wrong baud rate, a raw-serial peer, line noise) - worth logging by a
    /// driver that wants to surface it. Mirrors `KissDecoder.OversizeFramesDropped`.
    pub fn oversize_frames_dropped(&self) -> u64 {
        self.oversize_frames_dropped
    }

    /// Push a chunk of received bytes. Returns every frame the chunk completed
    /// (possibly none, possibly several).
    pub fn push(&mut self, bytes: &[u8]) -> Vec<Frame> {
        let mut frames = Vec::new();
        for &b in bytes {
            if self.resynchronising {
                // Nothing between the overrun and the next FEND can be a frame.
                if b == FEND {
                    self.resynchronising = false;
                    self.in_escape = false;
                }
                continue;
            }

            if self.in_escape {
                self.in_escape = false;
                match b {
                    TFEND => self.current.push(FEND),
                    TFESC => self.current.push(FESC),
                    // Lenient: drop a malformed escape byte and carry on.
                    _ => {}
                }
                self.drop_if_oversize();
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
                _ => {
                    self.current.push(b);
                    self.drop_if_oversize();
                }
            }
        }
        frames
    }

    /// Discard any partially-decoded frame state (releasing grown capacity).
    pub fn reset(&mut self) {
        self.current.clear();
        self.release_buffer();
        self.in_escape = false;
        self.resynchronising = false;
    }

    // Over the bound the partial frame is unusable: drop it, hand the memory back,
    // count it, and skip everything up to the next FEND.
    fn drop_if_oversize(&mut self) {
        if self.current.len() <= self.max_frame_len {
            return;
        }

        self.oversize_frames_dropped += 1;
        self.current.clear();
        self.release_buffer();
        self.in_escape = false;
        self.resynchronising = true;
    }

    // `clear` keeps the grown capacity, so a single oversize burst would retain
    // its memory for the life of the decoder. Give it back.
    fn release_buffer(&mut self) {
        if self.current.capacity() > INITIAL_CAPACITY {
            self.current.shrink_to(INITIAL_CAPACITY);
        }
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
    fn oversize_partial_is_dropped_and_stream_resyncs_at_the_next_fend() {
        let mut d = Decoder::with_max_frame_len(4);
        // Command byte + 4 payload bytes = 5 buffered > 4: dropped mid-stream.
        // Everything up to the next FEND is discarded (including the 0x99 tail),
        // then the following frame decodes normally.
        let frames = d.push(&[
            FEND, 0x00, 0x01, 0x02, 0x03, 0x04, 0x99, 0x99, FEND, 0x00, 0x42, FEND,
        ]);
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].payload, vec![0x42]);
        assert_eq!(d.oversize_frames_dropped(), 1);
    }

    #[test]
    fn frame_exactly_at_the_bound_is_kept() {
        let mut d = Decoder::with_max_frame_len(4);
        // Command byte + 3 payload bytes = 4 buffered = the bound: accepted.
        let frames = d.push(&[FEND, 0x00, 0x01, 0x02, 0x03, FEND]);
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].payload, vec![0x01, 0x02, 0x03]);
        assert_eq!(d.oversize_frames_dropped(), 0);
    }

    #[test]
    fn oversize_drop_counts_an_escaped_byte_too() {
        // The bound applies to the decoded length, so an escaped byte that lands
        // the buffer over the cap triggers the same drop + resync.
        let mut d = Decoder::with_max_frame_len(2);
        let frames = d.push(&[FEND, 0x00, 0x01, FESC, TFEND, 0x77, FEND, 0x00, 0x55, FEND]);
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].payload, vec![0x55]);
        assert_eq!(d.oversize_frames_dropped(), 1);
    }

    #[test]
    fn frameless_byte_stream_is_bounded_and_capacity_released() {
        // A stream with no FEND at all (the packet.net#696 shape): the default
        // decoder must drop oversize partials rather than grow without limit,
        // and hand the grown capacity back.
        let mut d = Decoder::new();
        let noise = vec![0x55u8; DEFAULT_MAX_FRAME_LEN * 2 + 10];
        assert!(d.push(&noise).is_empty());
        assert!(d.oversize_frames_dropped() >= 1);
        assert!(
            d.current.capacity() < DEFAULT_MAX_FRAME_LEN,
            "grown buffer capacity must be released after an oversize drop"
        );
        // Resync + a clean frame still decodes.
        let frames = d.push(&[FEND, 0x00, 0x42, FEND]);
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].payload, vec![0x42]);
    }

    #[test]
    fn reset_clears_resync_state_and_releases_capacity() {
        let mut d = Decoder::with_max_frame_len(4);
        assert!(d.push(&[FEND, 0x00, 1, 2, 3, 4, 5]).is_empty()); // now resynchronising
        d.reset();
        // After reset the decoder accepts a frame without needing the resync FEND
        // first (the leading FEND here is the normal frame opener).
        let frames = d.push(&[FEND, 0x00, 0x42, FEND]);
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].payload, vec![0x42]);
        assert!(d.current.capacity() <= INITIAL_CAPACITY);
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
