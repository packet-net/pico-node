//! The G8BPQ "ACKMODE" KISS extension (KISS command `0x0C`).
//!
//! Ports `Packet.Kiss.KissAckMode`. ACKMODE lets the host learn *when a frame is
//! actually keyed onto the air* (not merely accepted into the TNC's queue), which
//! matters for sizing T1 on slow modes where queue-acceptance is far from
//! transmit-completion.
//!
//! The host sends `FEND | (port<<4)|0xC | seqHi | seqLo | payload | FEND`; the TNC
//! echoes back `FEND | (port<<4)|0xC | seqHi | seqLo | FEND` (an exactly-2-byte
//! payload) when (and only when) the frame has been transmitted. The 2-byte tag is
//! an opaque token chosen by the host.
//!
//! This module is framing-neutral: it sits on top of [`Command::AckMode`] and a
//! decoded [`Frame`], and reuses the SLIP encoder/decoder for framing, the port
//! nibble, and FEND/FESC escapes. The build helper is `alloc`-gated (it returns a
//! `Vec`); the parse helpers are pure `core`.

use super::frame::{Command, Frame};

#[cfg(feature = "alloc")]
use super::encoder::encode;
#[cfg(feature = "alloc")]
use alloc::vec::Vec;

/// Build an ACKMODE outbound frame: command `0x0C` followed by the 2-byte
/// host-chosen sequence tag (big-endian) and the AX.25 payload bytes, SLIP-framed.
/// Returns `None` only if `port` is out of range (0–15). Requires `alloc`.
///
/// Mirrors `KissAckMode.BuildSendFrame`.
#[cfg(feature = "alloc")]
pub fn build_send_frame(port: u8, sequence_tag: u16, ax25_payload: &[u8]) -> Option<Vec<u8>> {
    let mut payload = Vec::with_capacity(ax25_payload.len() + 2);
    payload.push((sequence_tag >> 8) as u8);
    payload.push((sequence_tag & 0xFF) as u8);
    payload.extend_from_slice(ax25_payload);
    encode(port, Command::AckMode, &payload)
}

/// Build the ACKMODE payload (sequence tag + AX.25 bytes) into a caller-provided
/// buffer — the allocation-free path for the embedded transport. Returns the number
/// of payload bytes written (`ax25_payload.len() + 2`), or `None` if `dst` is too
/// small. The caller frames the result with [`super::encode_into`].
pub fn build_payload_into(dst: &mut [u8], sequence_tag: u16, ax25_payload: &[u8]) -> Option<usize> {
    let needed = ax25_payload.len() + 2;
    if dst.len() < needed {
        return None;
    }
    dst[0] = (sequence_tag >> 8) as u8;
    dst[1] = (sequence_tag & 0xFF) as u8;
    dst[2..needed].copy_from_slice(ax25_payload);
    Some(needed)
}

/// True if `frame` is the TNC's TX-completion echo for an ACKMODE send: command
/// `0x0C` with a payload of *exactly* 2 bytes (the sequence tag). Returns the
/// recovered 16-bit tag.
///
/// Mirrors `KissAckMode.TryParseAcknowledgement`.
pub fn try_parse_acknowledgement(frame: &Frame) -> Option<u16> {
    if frame.command != Command::AckMode || frame.payload.len() != 2 {
        return None;
    }
    Some(u16::from_be_bytes([frame.payload[0], frame.payload[1]]))
}

/// True if `frame` is an ACKMODE *data* frame — command `0x0C` with a payload of 2
/// sequence bytes followed by AX.25 bytes (length strictly greater than 2). Returns
/// `(sequence_tag, &ax25_payload)`. Single-port TNCs do not normally emit inbound
/// ACKMODE data, but multi-master / cross-link bridges can.
///
/// Mirrors `KissAckMode.TryParseDataFrame`.
pub fn try_parse_data_frame(frame: &Frame) -> Option<(u16, &[u8])> {
    if frame.command != Command::AckMode || frame.payload.len() <= 2 {
        return None;
    }
    let tag = u16::from_be_bytes([frame.payload[0], frame.payload[1]]);
    Some((tag, &frame.payload[2..]))
}

// ---------------------------------------------------------------------------
// ACKMODE TX-completion echo-correlator.
// ---------------------------------------------------------------------------
//
// Mirrors the correlation half of C# `NinoTncSerialPort` (`pendingAcks` +
// `NextSequenceTag` + the timeout/dispose paths). The send side already frames
// the 2-byte tag ([`build_send_frame`] / [`build_payload_into`]); this closes the
// loop by matching the TNC's TX-completion echo back to the outstanding send.
//
// C# uses a `ConcurrentDictionary<ushort, TaskCompletionSource>` on an unbounded
// heap and the wall clock. The portable node core is `no_std`, allocation-free,
// and FPU-free, so this is a **fixed-capacity** table over a caller-supplied
// **integer monotonic clock** (embassy `Instant` on-target, a fake clock in
// tests). The behaviour is otherwise byte-for-byte the C#: tags auto-assign from a
// wrapping cursor that skips 0, a duplicate tag is rejected, a matched echo
// resolves the round trip, an unmatched echo is dropped, entries expire on a
// timeout, and a link fault fails every outstanding send.

/// A monotonic millisecond timestamp supplied by the caller. No wall-clock
/// semantics are assumed — only differences matter — so a fake counter drives the
/// host tests and the embassy monotonic clock drives the firmware.
pub type Millis = u64;

/// The default ACKMODE TX-completion timeout, matching the C#
/// `NinoTncSerialPort.SendFrameWithAckAsync` default of 30 s.
pub const DEFAULT_ACK_TIMEOUT_MS: u64 = 30_000;

/// A completed ACKMODE round trip: the tag, when the frame was handed to the modem
/// (`queued_at_ms`), and when its TX-completion echo arrived (`completed_at_ms`).
///
/// Mirrors C# `TxCompletion` (which the driver builds by pairing the queued instant
/// with the echo-arrival instant).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TxCompletion {
    /// The 16-bit sequence tag that identified the send.
    pub sequence_tag: u16,
    /// When the frame was handed to the modem.
    pub queued_at_ms: Millis,
    /// When the TX-completion echo arrived.
    pub completed_at_ms: Millis,
}

impl TxCompletion {
    /// Time from queue-to-modem to TX-completion echo, in ms. Saturates at 0 if a
    /// non-monotonic clock ever reports the echo before the queue.
    pub fn elapsed_ms(&self) -> u64 {
        self.completed_at_ms.saturating_sub(self.queued_at_ms)
    }
}

/// Why [`AckCorrelator::register`] could not accept a send.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AckRegisterError {
    /// The tag already has an outstanding send. Mirrors the C# `TryAdd`-fails path
    /// (which throws `InvalidOperationException`): pick a unique tag.
    DuplicateTag,
    /// The fixed-capacity table is full. This has no C# analogue (the dictionary is
    /// unbounded); it is the embedded back-pressure signal — the caller should stop
    /// issuing new ACKMODE sends until an echo or timeout frees a slot.
    Full,
}

/// One outstanding ACKMODE send awaiting its TX-completion echo.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct PendingAck {
    tag: u16,
    queued_at_ms: Millis,
    deadline_ms: Millis,
}

/// A fixed-capacity ACKMODE TX-completion correlator: registers up to `N`
/// outstanding sends by tag, matches inbound echoes back to them, and expires the
/// ones that never complete.
///
/// Ports the correlation state of C# `NinoTncSerialPort` (`pendingAcks`,
/// `NextSequenceTag`, the timeout registration, and `FailPendingAcks`). The socket
/// / async-await glue stays in the firmware transport; this is the portable,
/// host-testable decision core.
///
/// `N` bounds the number of concurrently in-flight ACKMODE sends. On a
/// half-duplex packet link this is small (a NinoTNC transmits one frame at a time),
/// so a handful of slots covers the pipeline.
#[derive(Debug)]
pub struct AckCorrelator<const N: usize> {
    pending: [Option<PendingAck>; N],
    cursor: u16,
}

impl<const N: usize> Default for AckCorrelator<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> AckCorrelator<N> {
    /// A new, empty correlator.
    pub const fn new() -> Self {
        Self {
            pending: [None; N],
            cursor: 0,
        }
    }

    /// The next auto-assigned sequence tag: a wrapping 16-bit counter that skips 0.
    ///
    /// Mirrors C# `NinoTncSerialPort.NextSequenceTag` — it does *not* consult the
    /// pending table (only skips 0); [`register`](Self::register) is what rejects a
    /// genuine collision. On a table of a few slots against the 65 535-value space a
    /// collision is effectively impossible.
    pub fn next_tag(&mut self) -> u16 {
        loop {
            self.cursor = self.cursor.wrapping_add(1);
            if self.cursor != 0 {
                return self.cursor;
            }
        }
    }

    /// Register an outstanding send under `tag`, timing out `timeout_ms` after
    /// `queued_at_ms`. Returns [`AckRegisterError::DuplicateTag`] if the tag is
    /// already pending, or [`AckRegisterError::Full`] if the table is full.
    ///
    /// Mirrors the C# `pendingAcks.TryAdd(tag, …)` step done *before* the frame is
    /// written to the wire. On a write failure the caller unwinds with
    /// [`cancel`](Self::cancel), matching the C# `catch { pendingAcks.TryRemove }`.
    pub fn register(
        &mut self,
        tag: u16,
        queued_at_ms: Millis,
        timeout_ms: u64,
    ) -> Result<(), AckRegisterError> {
        if self.contains(tag) {
            return Err(AckRegisterError::DuplicateTag);
        }
        let slot = self
            .pending
            .iter_mut()
            .find(|s| s.is_none())
            .ok_or(AckRegisterError::Full)?;
        *slot = Some(PendingAck {
            tag,
            queued_at_ms,
            deadline_ms: queued_at_ms.saturating_add(timeout_ms),
        });
        Ok(())
    }

    /// Auto-assign a tag ([`next_tag`](Self::next_tag)) and [`register`](Self::register)
    /// it, returning the tag to stamp into the outbound payload
    /// ([`build_payload_into`]). Mirrors the null-`sequenceTag` path of C#
    /// `SendFrameWithAckAsync` (which auto-assigns before sending).
    pub fn begin(&mut self, queued_at_ms: Millis, timeout_ms: u64) -> Result<u16, AckRegisterError> {
        let tag = self.next_tag();
        self.register(tag, queued_at_ms, timeout_ms)?;
        Ok(tag)
    }

    /// Correlate a decoded inbound [`Frame`] against the outstanding sends. Returns
    /// the [`TxCompletion`] if the frame is a TX-completion echo
    /// ([`try_parse_acknowledgement`]) whose tag is pending; otherwise `None` — a
    /// non-echo frame, or an echo for a tag we are not tracking (the
    /// dropped/mismatched-echo case), is left for the caller to route elsewhere.
    ///
    /// Mirrors the echo branch of C# `NinoTncSerialPort.DispatchFrame`.
    pub fn on_echo(&mut self, frame: &Frame, now_ms: Millis) -> Option<TxCompletion> {
        let tag = try_parse_acknowledgement(frame)?;
        self.on_echo_tag(tag, now_ms)
    }

    /// Correlate an already-parsed echo `tag`. Returns the [`TxCompletion`] and frees
    /// the slot if the tag was pending; `None` (dropped) if it was not. The
    /// alloc-free path for a caller that parsed the tag itself.
    pub fn on_echo_tag(&mut self, tag: u16, now_ms: Millis) -> Option<TxCompletion> {
        let slot = self
            .pending
            .iter_mut()
            .find(|s| matches!(s, Some(p) if p.tag == tag))?;
        let pending = slot.take()?;
        Some(TxCompletion {
            sequence_tag: tag,
            queued_at_ms: pending.queued_at_ms,
            completed_at_ms: now_ms,
        })
    }

    /// Remove one outstanding send whose deadline has passed (`deadline_ms <=
    /// now_ms`), returning its tag so the caller can fault the waiter. Call in a loop
    /// until it returns `None` to drain every expiry for this tick.
    ///
    /// Mirrors the per-tag timeout branch of C# `SendFrameWithAckAsync` (the linked
    /// `CancelAfter` registration that removes the tag and sets a `TimeoutException`).
    pub fn poll_timeout(&mut self, now_ms: Millis) -> Option<u16> {
        let slot = self
            .pending
            .iter_mut()
            .find(|s| matches!(s, Some(p) if p.deadline_ms <= now_ms))?;
        slot.take().map(|p| p.tag)
    }

    /// Cancel a single outstanding send by tag (e.g. the wire write failed after
    /// registration). Returns `true` if it was pending. Mirrors the C#
    /// `catch { pendingAcks.TryRemove(tag, out _) }` on a send failure.
    pub fn cancel(&mut self, tag: u16) -> bool {
        if let Some(slot) = self
            .pending
            .iter_mut()
            .find(|s| matches!(s, Some(p) if p.tag == tag))
        {
            *slot = None;
            true
        } else {
            false
        }
    }

    /// Drop every outstanding send — the link faulted or the port is closing.
    /// Returns how many were pending (the count of waiters the caller must fault).
    ///
    /// Mirrors C# `FailPendingAcks`, called from the dispatch loop's terminal branch
    /// and on dispose.
    pub fn reset(&mut self) -> usize {
        let mut n = 0;
        for slot in self.pending.iter_mut() {
            if slot.take().is_some() {
                n += 1;
            }
        }
        n
    }

    /// True if `tag` currently has an outstanding send.
    pub fn contains(&self, tag: u16) -> bool {
        self.pending
            .iter()
            .any(|s| matches!(s, Some(p) if p.tag == tag))
    }

    /// The number of outstanding sends.
    pub fn len(&self) -> usize {
        self.pending.iter().filter(|s| s.is_some()).count()
    }

    /// True if no sends are outstanding.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// True if the table is full — no further [`register`](Self::register) will
    /// succeed until a slot is freed.
    pub fn is_full(&self) -> bool {
        self.pending.iter().all(|s| s.is_some())
    }
}

#[cfg(all(test, feature = "alloc"))]
mod tests {
    use super::*;
    use crate::kiss::frame::{FEND, FESC, TFEND, TFESC};
    use crate::kiss::Decoder;
    use alloc::vec;

    #[test]
    fn build_send_frame_has_ackmode_command_and_sequence_prefix() {
        // tag 0xA5B6, payload "AB" → FEND, 0x0C, 0xA5, 0xB6, 'A', 'B', FEND
        let wire = build_send_frame(0, 0xA5B6, &[0x41, 0x42]).unwrap();
        assert_eq!(wire, vec![FEND, 0x0C, 0xA5, 0xB6, 0x41, 0x42, FEND]);
    }

    #[test]
    fn build_send_frame_encodes_port_in_upper_nibble() {
        // port 5, ackmode → command byte (5<<4)|0x0C = 0x5C
        let wire = build_send_frame(5, 0x0001, &[]).unwrap();
        assert_eq!(wire, vec![FEND, 0x5C, 0x00, 0x01, FEND]);
    }

    #[test]
    fn build_send_frame_escapes_sequence_bytes_when_they_are_fend() {
        // seqHi = 0xC0 (FEND) → escape to FESC TFEND; seqLo = 0xDB (FESC) → FESC TFESC
        let wire = build_send_frame(0, 0xC0DB, &[]).unwrap();
        assert_eq!(wire, vec![FEND, 0x0C, FESC, TFEND, FESC, TFESC, FEND]);
    }

    #[test]
    fn try_parse_acknowledgement_recovers_the_tag_from_two_byte_payload() {
        let frame = Frame::new(0, Command::AckMode, vec![0x12, 0x34]);
        assert_eq!(try_parse_acknowledgement(&frame), Some(0x1234));
    }

    #[test]
    fn try_parse_acknowledgement_rejects_non_ackmode_commands() {
        let frame = Frame::new(0, Command::Data, vec![0x12, 0x34]);
        assert_eq!(try_parse_acknowledgement(&frame), None);
    }

    #[test]
    fn try_parse_acknowledgement_rejects_wrong_payload_length() {
        // 3-byte payload = a data frame (seq + 1 AX.25 byte), not an echo.
        let with_data = Frame::new(0, Command::AckMode, vec![0x12, 0x34, 0x99]);
        assert_eq!(try_parse_acknowledgement(&with_data), None);
        let empty = Frame::new(0, Command::AckMode, vec![]);
        assert_eq!(try_parse_acknowledgement(&empty), None);
    }

    #[test]
    fn try_parse_data_frame_splits_sequence_from_payload() {
        let frame = Frame::new(0, Command::AckMode, vec![0x12, 0x34, 0x41, 0x42, 0x43]);
        let (tag, data) = try_parse_data_frame(&frame).unwrap();
        assert_eq!(tag, 0x1234);
        assert_eq!(data, &[0x41, 0x42, 0x43]);
    }

    #[test]
    fn try_parse_data_frame_rejects_the_two_byte_echo() {
        let frame = Frame::new(0, Command::AckMode, vec![0x12, 0x34]);
        assert_eq!(try_parse_data_frame(&frame), None);
    }

    #[test]
    fn round_trip_send_frame_then_decode_recovers_tag_and_payload() {
        let payload = [0xA8, 0x8A, 0xA6, 0xC0, 0xDB, 0x03, 0xF0, 0x68, 0x69];
        let wire = build_send_frame(0, 0xBEEF, &payload).unwrap();
        let mut decoder = Decoder::new();
        let frames = decoder.push(&wire);
        assert_eq!(frames.len(), 1);
        let decoded = &frames[0];
        assert_eq!(decoded.command, Command::AckMode);
        let (tag, round_trip) = try_parse_data_frame(decoded).unwrap();
        assert_eq!(tag, 0xBEEF);
        assert_eq!(round_trip, &payload);
    }

    #[test]
    fn build_payload_into_matches_alloc_path() {
        let mut buf = [0u8; 16];
        let n = build_payload_into(&mut buf, 0xBEEF, &[0x41, 0x42, 0x43]).unwrap();
        assert_eq!(&buf[..n], &[0xBE, 0xEF, 0x41, 0x42, 0x43]);
    }

    #[test]
    fn build_payload_into_reports_too_small() {
        let mut buf = [0u8; 3];
        assert_eq!(build_payload_into(&mut buf, 0x1234, &[1, 2]), None);
    }

    // ----- ACKMODE correlator -----

    /// The end-to-end happy path: assign a tag, register the send, then feed the
    /// TNC's 2-byte echo back in and get the timed completion.
    #[test]
    fn correlator_tag_emit_matching_echo_completes() {
        let mut c = AckCorrelator::<4>::new();
        let tag = c.begin(1_000, DEFAULT_ACK_TIMEOUT_MS).unwrap();
        assert_ne!(tag, 0, "auto-assigned tags skip 0");
        assert!(c.contains(tag));
        assert_eq!(c.len(), 1);

        // The TNC echoes the tag as a 2-byte ACKMODE frame once the frame is keyed.
        let echo = Frame::new(0, Command::AckMode, vec![(tag >> 8) as u8, (tag & 0xFF) as u8]);
        let done = c.on_echo(&echo, 1_150).expect("echo must correlate");
        assert_eq!(done.sequence_tag, tag);
        assert_eq!(done.queued_at_ms, 1_000);
        assert_eq!(done.completed_at_ms, 1_150);
        assert_eq!(done.elapsed_ms(), 150);
        // The slot is freed on completion.
        assert!(!c.contains(tag));
        assert!(c.is_empty());
    }

    /// A dropped / mismatched echo — one whose tag we are not tracking — correlates
    /// to nothing and leaves the real outstanding send untouched.
    #[test]
    fn correlator_mismatched_echo_is_dropped_and_leaves_pending_intact() {
        let mut c = AckCorrelator::<4>::new();
        c.register(0x0001, 0, DEFAULT_ACK_TIMEOUT_MS).unwrap();

        // Echo for a different tag (0xBEEF) → no completion, nothing removed.
        let stray = Frame::new(0, Command::AckMode, vec![0xBE, 0xEF]);
        assert_eq!(c.on_echo(&stray, 500), None);
        assert!(c.contains(0x0001));
        assert_eq!(c.len(), 1);

        // A non-echo frame (a data frame, >2 bytes) also correlates to nothing.
        let data = Frame::new(0, Command::AckMode, vec![0x00, 0x01, 0x41]);
        assert_eq!(c.on_echo(&data, 500), None);
        assert!(c.contains(0x0001));
    }

    #[test]
    fn correlator_rejects_a_duplicate_tag() {
        let mut c = AckCorrelator::<4>::new();
        c.register(0x1234, 0, DEFAULT_ACK_TIMEOUT_MS).unwrap();
        assert_eq!(
            c.register(0x1234, 10, DEFAULT_ACK_TIMEOUT_MS),
            Err(AckRegisterError::DuplicateTag)
        );
    }

    #[test]
    fn correlator_reports_full_when_capacity_is_exhausted() {
        let mut c = AckCorrelator::<2>::new();
        c.register(1, 0, DEFAULT_ACK_TIMEOUT_MS).unwrap();
        c.register(2, 0, DEFAULT_ACK_TIMEOUT_MS).unwrap();
        assert!(c.is_full());
        assert_eq!(c.register(3, 0, DEFAULT_ACK_TIMEOUT_MS), Err(AckRegisterError::Full));
        // Freeing one slot lets a new send register again.
        assert!(c.cancel(1));
        assert!(!c.is_full());
        assert!(c.register(3, 0, DEFAULT_ACK_TIMEOUT_MS).is_ok());
    }

    #[test]
    fn correlator_expires_sends_past_their_deadline() {
        let mut c = AckCorrelator::<4>::new();
        // 30 s timeout from t=1000 → deadline 31_000.
        let tag = c.begin(1_000, DEFAULT_ACK_TIMEOUT_MS).unwrap();
        // Just before the deadline: nothing expires.
        assert_eq!(c.poll_timeout(30_999), None);
        assert!(c.contains(tag));
        // At the deadline: the tag is surfaced and removed.
        assert_eq!(c.poll_timeout(31_000), Some(tag));
        assert!(!c.contains(tag));
        assert_eq!(c.poll_timeout(31_000), None);
    }

    #[test]
    fn correlator_reset_fails_all_pending() {
        let mut c = AckCorrelator::<4>::new();
        c.register(1, 0, DEFAULT_ACK_TIMEOUT_MS).unwrap();
        c.register(2, 0, DEFAULT_ACK_TIMEOUT_MS).unwrap();
        c.register(3, 0, DEFAULT_ACK_TIMEOUT_MS).unwrap();
        assert_eq!(c.reset(), 3);
        assert!(c.is_empty());
        assert_eq!(c.reset(), 0);
    }

    #[test]
    fn correlator_next_tag_wraps_past_zero() {
        let mut c = AckCorrelator::<1>::new();
        c.cursor = u16::MAX;
        // MAX + 1 wraps to 0, which is skipped → 1.
        assert_eq!(c.next_tag(), 1);
    }

    #[test]
    fn correlator_on_echo_tag_matches_without_a_frame() {
        let mut c = AckCorrelator::<4>::new();
        c.register(0xABCD, 2_000, DEFAULT_ACK_TIMEOUT_MS).unwrap();
        let done = c.on_echo_tag(0xABCD, 2_075).unwrap();
        assert_eq!(done.elapsed_ms(), 75);
        assert_eq!(c.on_echo_tag(0xABCD, 2_075), None, "second echo for the same tag is dropped");
    }
}
