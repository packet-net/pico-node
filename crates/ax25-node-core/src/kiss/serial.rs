//! The serial-KISS transport seam — a KISS modem over any async byte stream.
//!
//! Ports the *protocol behaviour* of `Packet.Kiss.Serial.KissSerialModem` (and the
//! `IKissModem` seam it implements) to a `no_std`, transport-agnostic shape. The C#
//! modem owns a `System.IO.Ports.SerialPort`, a read-pump task, and a write lock;
//! here the byte transport is abstracted behind the [`ByteStream`] trait so the same
//! framing/codec logic is:
//!
//! - **host-testable now** — driven over an in-memory loopback ([`MemStream`],
//!   `std`/`alloc`-gated, test-only) with `cargo test`;
//! - **embedded later** — driven over an `embassy_rp` UART in the firmware (the
//!   firmware implements [`ByteStream`] for the UART; this file stays unchanged).
//!
//! [`SerialKissModem`] is the seam the SDL runtime / node consume: `send_frame` to
//! transmit an AX.25 body, `read_frame` to pull the next inbound KISS frame, plus
//! the KISS parameter setters. It owns the outbound encode buffer and the streaming
//! inbound [`Decoder`], exactly mirroring `KissSerialModem`'s `KissEncoder` +
//! `KissDecoder` usage. The port nibble is fixed at 0 (single-port TNC), matching
//! `KissSerialModem`'s `KissPort = 0`.
//!
//! The async surface uses `async fn` in traits (stable since 1.75), the same pattern
//! as [`crate::console::connection::NodeConnection`], so it is Embassy-usable
//! without an `async-trait` crate.

use core::future::Future;

use super::frame::{Command, Frame, FEND};
use super::{encode_into, max_encoded_len, Decoder};

#[cfg(feature = "alloc")]
use alloc::vec::Vec;

/// The fixed KISS port the serial modem uses (single-port TNC). Mirrors
/// `KissSerialModem.KissPort = 0`.
pub const KISS_PORT: u8 = 0;

/// A bidirectional async byte stream — the raw transport under a KISS modem (a UART
/// on the target, an in-memory pipe in host tests).
///
/// `read` returns 0 on EOF / link-down (mirrors the C# pump's `read <= 0` → retry).
/// A normal close is not an error — it's a zero-length read.
pub trait ByteStream {
    /// Transport-specific error for read/write failures.
    type Error;

    /// Read the next chunk of inbound bytes into `buf`; returns the number read
    /// (0 == EOF / nothing yet). The embedded UART impl awaits bytes; the host
    /// loopback returns queued bytes.
    fn read<'a>(
        &'a mut self,
        buf: &'a mut [u8],
    ) -> impl Future<Output = Result<usize, Self::Error>> + 'a;

    /// Write all of `bytes` to the stream. Mirrors the C# write-under-lock.
    fn write<'a>(
        &'a mut self,
        bytes: &'a [u8],
    ) -> impl Future<Output = Result<(), Self::Error>> + 'a;
}

/// The maximum AX.25 body the fixed outbound encode buffer can frame. AX.25 v2.2 N1
/// tops out at 256 info octets + ~header; 512 covers the largest frame plus the
/// ACKMODE 2-byte tag with margin. (mod-128 large windows are bounded by config —
/// see PLAN §5 / research §6 — so a single frame stays small.)
pub const MAX_AX25_BODY: usize = 512;

/// Outbound scratch buffer sized for the worst-case KISS escaping of [`MAX_AX25_BODY`].
const OUT_BUF_LEN: usize = max_encoded_len(MAX_AX25_BODY);

/// A KISS modem over an arbitrary [`ByteStream`]. The serial-KISS transport, with
/// the byte source abstracted so the framing/codec is host-testable.
///
/// Generic over the stream `S`; the firmware instantiates `SerialKissModem<UartStream>`,
/// host tests instantiate `SerialKissModem<MemStream>`. Owns a fixed outbound buffer
/// (no per-send allocation — the embedded path) and the streaming inbound decoder.
pub struct SerialKissModem<S: ByteStream> {
    stream: S,
    decoder: Decoder,
    out: [u8; OUT_BUF_LEN],
    /// Inbound read scratch — bytes from the stream before decoding.
    rx: [u8; 256],
    /// Frames decoded from the last read but not yet returned by `read_frame`.
    #[cfg(feature = "alloc")]
    pending: Vec<Frame>,
}

impl<S: ByteStream> SerialKissModem<S> {
    /// Wrap a byte stream as a KISS modem.
    pub fn new(stream: S) -> Self {
        Self {
            stream,
            decoder: Decoder::new(),
            out: [0u8; OUT_BUF_LEN],
            rx: [0u8; 256],
            #[cfg(feature = "alloc")]
            pending: Vec::new(),
        }
    }

    /// Borrow the underlying stream (e.g. to inspect link state).
    pub fn stream(&self) -> &S {
        &self.stream
    }

    /// Frame and send a KISS `Data` frame carrying `ax25_bytes`. Mirrors
    /// `KissSerialModem.SendFrameAsync`. Returns the modem's `Error::TooLarge` if
    /// the body exceeds [`MAX_AX25_BODY`].
    pub async fn send_frame(&mut self, ax25_bytes: &[u8]) -> Result<(), ModemError<S::Error>> {
        self.send_kiss(Command::Data, ax25_bytes).await
    }

    /// Frame and send an ACKMODE (`0x0C`) frame: the 2-byte `sequence_tag` then the
    /// AX.25 body. Mirrors the *send half* of `NinoTncSerialPort.SendFrameWithAckAsync`.
    ///
    /// The echo-correlation (await the TNC's TX-completion echo carrying this tag) is
    /// the *caller's* concern — the same split the C# uses between the framing-neutral
    /// [`crate::kiss::ackmode`] helpers and the `NinoTncSerialPort` driver's
    /// `pendingAcks`/`TaskCompletionSource` plumbing. The caller pulls inbound frames
    /// via [`Self::read_frame`] and matches [`crate::kiss::ackmode::try_parse_acknowledgement`]
    /// against its outstanding tags. Allocation-free: the tag+body is staged in a
    /// fixed scratch buffer.
    pub async fn send_ackmode(
        &mut self,
        sequence_tag: u16,
        ax25_bytes: &[u8],
    ) -> Result<(), ModemError<S::Error>> {
        let mut staged = [0u8; MAX_AX25_BODY + 2];
        let n = super::ackmode::build_payload_into(&mut staged, sequence_tag, ax25_bytes)
            .ok_or(ModemError::TooLarge)?;
        self.send_kiss(Command::AckMode, &staged[..n]).await
    }

    /// Set the NinoTNC operating mode via KISS SETHW (`0x06`). Mirrors
    /// `NinoTncSerialPort.SetModeAsync`. `persist_to_flash == false` (the common
    /// default) applies the `+16` non-persist offset so the change is RAM-only.
    /// Returns `Error::TooLarge` if `mode > 15` (the SETHW byte can't encode it).
    pub async fn set_mode(
        &mut self,
        mode: u8,
        persist_to_flash: bool,
    ) -> Result<(), ModemError<S::Error>> {
        let payload = super::ninotnc::sethw::build_payload_byte(mode, persist_to_flash)
            .ok_or(ModemError::TooLarge)?;
        self.send_kiss(Command::SetHardware, &[payload]).await
    }

    /// Frame and send an arbitrary KISS command on [`KISS_PORT`]. Mirrors
    /// `KissSerialModem.SendKissAsync`.
    pub async fn send_kiss(
        &mut self,
        command: Command,
        payload: &[u8],
    ) -> Result<(), ModemError<S::Error>> {
        let n =
            encode_into(&mut self.out, KISS_PORT, command, payload).ok_or(ModemError::TooLarge)?;
        // Copy out the framed bytes so the borrow on `self.out` ends before the
        // `&mut self.stream` write borrow (both touch `self`).
        let len = n;
        // SAFETY-free: split the borrows by indexing distinct fields.
        let SerialKissModem { stream, out, .. } = self;
        stream.write(&out[..len]).await.map_err(ModemError::Io)
    }

    /// Send a KISS parameter command (a single value byte). The building block for
    /// the named setters; mirrors `KissSerialModem.SendParameterAsync`.
    pub async fn send_param(
        &mut self,
        command: Command,
        value: u8,
    ) -> Result<(), ModemError<S::Error>> {
        self.send_kiss(command, &[value]).await
    }

    /// KISS TXDELAY (`0x01`), units of 10 ms. Mirrors `SetTxDelayAsync`.
    pub async fn set_tx_delay(&mut self, ten_ms_units: u8) -> Result<(), ModemError<S::Error>> {
        self.send_param(Command::TxDelay, ten_ms_units).await
    }

    /// KISS PERSIST (`0x02`), 0..=255. Mirrors `SetPersistenceAsync`.
    pub async fn set_persistence(&mut self, value: u8) -> Result<(), ModemError<S::Error>> {
        self.send_param(Command::Persistence, value).await
    }

    /// KISS SLOTTIME (`0x03`), units of 10 ms. Mirrors `SetSlotTimeAsync`.
    pub async fn set_slot_time(&mut self, ten_ms_units: u8) -> Result<(), ModemError<S::Error>> {
        self.send_param(Command::SlotTime, ten_ms_units).await
    }

    /// KISS TXTAIL (`0x04`), units of 10 ms. Mirrors `SetTxTailAsync`.
    pub async fn set_tx_tail(&mut self, ten_ms_units: u8) -> Result<(), ModemError<S::Error>> {
        self.send_param(Command::TxTail, ten_ms_units).await
    }

    /// KISS FULLDUPLEX (`0x05`). Mirrors `SetFullDuplexAsync`.
    pub async fn set_full_duplex(&mut self, full: bool) -> Result<(), ModemError<S::Error>> {
        self.send_param(Command::FullDuplex, if full { 1 } else { 0 })
            .await
    }

    /// Send the bare Exit-KISS-mode command (`0xFF`, unframed). Mirrors the C#
    /// `KissFraming.ExitKissMode` convention (a single byte, no FEND framing).
    pub async fn exit_kiss_mode(&mut self) -> Result<(), ModemError<S::Error>> {
        self.stream
            .write(&[super::EXIT_KISS_MODE])
            .await
            .map_err(ModemError::Io)
    }

    /// Read the next inbound KISS frame, awaiting and decoding stream bytes as
    /// needed. Returns `Ok(None)` on EOF / link-down (a zero-length read with no
    /// buffered frames) — mirroring the C# pump treating `read <= 0` as "nothing,
    /// loop"; the caller decides whether to reconnect. The streaming decoder
    /// preserves partial-frame + escape state across reads, so any chunking works.
    ///
    /// `alloc`-gated: a completed read can yield several frames, which are buffered
    /// in a `Vec` and drained one per call. The heapless follow-up (a fixed
    /// frame-ring) is noted in the module roadmap.
    #[cfg(feature = "alloc")]
    pub async fn read_frame(&mut self) -> Result<Option<Frame>, ModemError<S::Error>> {
        loop {
            if !self.pending.is_empty() {
                return Ok(Some(self.pending.remove(0)));
            }
            let SerialKissModem {
                stream,
                decoder,
                rx,
                pending,
                ..
            } = self;
            let n = stream.read(rx).await.map_err(ModemError::Io)?;
            if n == 0 {
                return Ok(None);
            }
            let frames = decoder.push(&rx[..n]);
            if frames.is_empty() {
                // Partial frame — keep reading.
                continue;
            }
            *pending = frames;
        }
    }
}

/// A serial-KISS modem error: either the underlying byte stream failed, or an
/// outbound frame exceeded the fixed encode buffer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModemError<E> {
    /// The underlying [`ByteStream`] read/write failed.
    Io(E),
    /// The AX.25 body exceeded [`MAX_AX25_BODY`] (the fixed outbound buffer).
    TooLarge,
}

/// A frame-boundary helper for callers that own their own decode loop (e.g. a
/// firmware task that already has the UART). True if `byte` is the KISS frame
/// delimiter — handy for coarse logging without decoding.
pub const fn is_frame_delimiter(byte: u8) -> bool {
    byte == FEND
}

// ───────────────────────── host-test loopback stream ─────────────────────────

/// An in-memory, single-direction byte pipe for host tests — the loopback that
/// stands in for the UART so the serial-KISS framing/codec is exercised end-to-end
/// with `cargo test`. Writes append to a shared buffer; reads drain it.
///
/// `alloc`-gated and test-shaped (no real backpressure / async waiting — a read of
/// an empty buffer returns 0, i.e. "EOF for now"). The firmware's real UART
/// [`ByteStream`] awaits bytes instead.
#[cfg(feature = "alloc")]
#[derive(Debug, Default)]
pub struct MemStream {
    /// Bytes available to `read` (what the "wire" delivered to this end).
    inbox: Vec<u8>,
    /// Bytes this end `write`s (what it sent onto the "wire").
    outbox: Vec<u8>,
}

#[cfg(feature = "alloc")]
impl MemStream {
    /// A new empty stream.
    pub fn new() -> Self {
        Self::default()
    }

    /// Queue bytes as if they had arrived from the wire (the test feeds RX here).
    pub fn feed(&mut self, bytes: &[u8]) {
        self.inbox.extend_from_slice(bytes);
    }

    /// Take everything this end has written to the wire (the test reads TX here).
    pub fn take_written(&mut self) -> Vec<u8> {
        core::mem::take(&mut self.outbox)
    }
}

/// The never-failing error type for [`MemStream`].
#[cfg(feature = "alloc")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MemStreamError {}

#[cfg(feature = "alloc")]
impl ByteStream for MemStream {
    type Error = MemStreamError;

    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let n = self.inbox.len().min(buf.len());
        buf[..n].copy_from_slice(&self.inbox[..n]);
        self.inbox.drain(..n);
        Ok(n)
    }

    async fn write(&mut self, bytes: &[u8]) -> Result<(), Self::Error> {
        self.outbox.extend_from_slice(bytes);
        Ok(())
    }
}

#[cfg(all(test, feature = "alloc"))]
mod tests {
    use super::*;
    use crate::kiss::frame::{FESC, TFEND};
    use alloc::vec;

    /// A tiny synchronous executor for the async modem methods (host tests don't
    /// need a real runtime — the futures never truly suspend on `MemStream`, so a
    /// busy poll completes them immediately). `unsafe`-free: the future is heap-
    /// pinned with `Box::pin` (the crate forbids `unsafe`), and a no-op waker
    /// suffices because nothing ever schedules a wake.
    fn block_on<F: Future>(fut: F) -> F::Output {
        use alloc::boxed::Box;
        use core::task::{Context, Poll, Waker};
        let mut fut = Box::pin(fut);
        let waker = Waker::noop();
        let mut cx = Context::from_waker(waker);
        loop {
            match fut.as_mut().poll(&mut cx) {
                Poll::Ready(v) => return v,
                Poll::Pending => continue,
            }
        }
    }

    #[test]
    fn send_frame_emits_kiss_data_framing() {
        let mut modem = SerialKissModem::new(MemStream::new());
        block_on(modem.send_frame(&[0x01, 0x02, 0x03])).unwrap();
        // FEND, cmd 0x00 (port 0 + Data), payload, FEND.
        let wire = modem.stream.take_written();
        assert_eq!(wire, vec![FEND, 0x00, 0x01, 0x02, 0x03, FEND]);
    }

    #[test]
    fn send_ackmode_frames_command_0x0c_with_tag_prefix() {
        let mut modem = SerialKissModem::new(MemStream::new());
        block_on(modem.send_ackmode(0xBEEF, &[0x41, 0x42])).unwrap();
        let wire = modem.stream.take_written();
        // FEND, 0x0C (port 0 + AckMode), 0xBE, 0xEF, 'A', 'B', FEND.
        assert_eq!(wire, vec![FEND, 0x0C, 0xBE, 0xEF, 0x41, 0x42, FEND]);
        // And the framed tag round-trips through the ackmode echo parser.
        let mut d = Decoder::new();
        let frames = d.push(&wire);
        let (tag, body) = super::super::ackmode::try_parse_data_frame(&frames[0]).unwrap();
        assert_eq!(tag, 0xBEEF);
        assert_eq!(body, &[0x41, 0x42]);
    }

    #[test]
    fn set_mode_frames_ninotnc_sethw() {
        let mut modem = SerialKissModem::new(MemStream::new());
        // mode 6, persist=false → SETHW payload 6+16 = 22 (0x16).
        block_on(modem.set_mode(6, false)).unwrap();
        assert_eq!(modem.stream.take_written(), vec![FEND, 0x06, 0x16, FEND]);
        // persist=true → payload = mode unchanged.
        block_on(modem.set_mode(6, true)).unwrap();
        assert_eq!(modem.stream.take_written(), vec![FEND, 0x06, 0x06, FEND]);
        // out-of-range mode rejected.
        assert_eq!(
            block_on(modem.set_mode(16, false)),
            Err(ModemError::TooLarge)
        );
    }

    #[test]
    fn send_frame_escapes_payload() {
        let mut modem = SerialKissModem::new(MemStream::new());
        block_on(modem.send_frame(&[FEND, FESC])).unwrap();
        let wire = modem.stream.take_written();
        assert_eq!(
            wire,
            vec![
                FEND,
                0x00,
                FESC,
                TFEND,
                FESC,
                super::super::frame::TFESC,
                FEND
            ]
        );
    }

    #[test]
    fn parameter_setters_frame_the_right_commands() {
        let mut modem = SerialKissModem::new(MemStream::new());
        block_on(modem.set_tx_delay(50)).unwrap();
        block_on(modem.set_persistence(63)).unwrap();
        block_on(modem.set_slot_time(10)).unwrap();
        block_on(modem.set_full_duplex(true)).unwrap();
        let wire = modem.stream.take_written();
        assert_eq!(
            wire,
            vec![
                FEND, 0x01, 50, FEND, // TXDELAY
                FEND, 0x02, 63, FEND, // PERSIST
                FEND, 0x03, 10, FEND, // SLOTTIME
                FEND, 0x05, 1, FEND, // FULLDUPLEX
            ]
        );
    }

    #[test]
    fn exit_kiss_mode_writes_bare_ff() {
        let mut modem = SerialKissModem::new(MemStream::new());
        block_on(modem.exit_kiss_mode()).unwrap();
        assert_eq!(modem.stream.take_written(), vec![0xFF]);
    }

    #[test]
    fn read_frame_decodes_a_fed_frame() {
        let mut modem = SerialKissModem::new(MemStream::new());
        modem.stream.feed(&[FEND, 0x00, 0xDE, 0xAD, FEND]);
        let frame = block_on(modem.read_frame()).unwrap().unwrap();
        assert_eq!(frame.command, Command::Data);
        assert_eq!(frame.payload, vec![0xDE, 0xAD]);
    }

    #[test]
    fn read_frame_reassembles_across_reads() {
        // Feed the frame in three pieces (incl. a split escape) — the streaming
        // decoder must stitch it across reads, like the C# pump over a UART.
        let mut modem = SerialKissModem::new(MemStream::new());
        modem.stream.feed(&[FEND, 0x00, 0x11, FESC]);
        // No full frame yet (escape pending, no closing FEND).
        // The first read consumes everything queued; read_frame loops until a frame
        // or EOF. With the rest not yet fed it returns None (EOF-for-now).
        assert!(block_on(modem.read_frame()).unwrap().is_none());
        modem.stream.feed(&[TFEND, 0x22, FEND]);
        let frame = block_on(modem.read_frame()).unwrap().unwrap();
        assert_eq!(frame.payload, vec![0x11, FEND, 0x22]);
    }

    #[test]
    fn read_frame_returns_none_on_empty() {
        let mut modem = SerialKissModem::new(MemStream::new());
        assert!(block_on(modem.read_frame()).unwrap().is_none());
    }

    #[test]
    fn read_frame_drains_multiple_frames_one_per_call() {
        let mut modem = SerialKissModem::new(MemStream::new());
        modem
            .stream
            .feed(&[FEND, 0x00, 0xAA, FEND, FEND, 0x00, 0xBB, FEND]);
        let f1 = block_on(modem.read_frame()).unwrap().unwrap();
        let f2 = block_on(modem.read_frame()).unwrap().unwrap();
        assert_eq!(f1.payload, vec![0xAA]);
        assert_eq!(f2.payload, vec![0xBB]);
        assert!(block_on(modem.read_frame()).unwrap().is_none());
    }

    #[test]
    fn host_loopback_round_trips_an_ax25_body_through_two_modems() {
        // The load-bearing serial-KISS loopback: modem A sends an AX.25 body; the
        // framed wire bytes are carried to modem B's RX; B decodes the identical
        // body. This exercises encode → wire → streaming-decode end to end.
        let mut a = SerialKissModem::new(MemStream::new());
        let mut b = SerialKissModem::new(MemStream::new());
        let body = vec![0xA8, 0x8A, 0xA6, FEND, FESC, 0x03, 0xF0, b'h', b'i'];

        block_on(a.send_frame(&body)).unwrap();
        let wire = a.stream.take_written();
        // Deliver the wire in awkward 3-byte chunks to stress the streaming decoder.
        for chunk in wire.chunks(3) {
            b.stream.feed(chunk);
            // Drain whatever completed so far.
            while let Some(frame) = block_on(b.read_frame()).unwrap() {
                assert_eq!(frame.command, Command::Data);
                assert_eq!(frame.payload, body);
            }
        }
    }

    #[test]
    fn too_large_body_is_rejected() {
        let mut modem = SerialKissModem::new(MemStream::new());
        let huge = vec![0u8; MAX_AX25_BODY + 1];
        assert_eq!(block_on(modem.send_frame(&huge)), Err(ModemError::TooLarge));
    }
}
