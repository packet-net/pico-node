//! Tait FFSK transparent-mode transport — AX.25 over the radio's own modem, no TNC.
//! Ports `Packet.Radio.Tait.TaitTransparentTransport`.
//!
//! In Transparent mode the radio's serial port becomes an 8-bit-clean byte pipe
//! through its internal FFSK modem. This transport frames AX.25 with **KISS SLIP**
//! (FEND-delimited, port 0 `Data`) over that pipe — reusing the already-ported
//! [`kiss::encode_into`](crate::kiss::encode_into) encoder and streaming
//! [`kiss::Decoder`](crate::kiss::Decoder) verbatim, exactly as the C# reuses
//! `KissEncoder`/`KissDecoder` — and adds the CCDI mode control: the `t` command to
//! enter, and the `+++` guarded escape to leave.
//!
//! ## Signal-telemetry trade-off (inherent)
//!
//! In Transparent mode the serial port is a byte pipe: **CCDI is unavailable**, so
//! there is no RSSI, SNR, noise-floor, carrier-rise (DCD) or burst attribution — the
//! [`RadioMetadata`] on a received frame carries only [`RadioMetadata::estimated_airtime_us`].
//! This is the deliberate cost of a TNC-less link (one device, no audio wiring),
//! versus the NinoTNC-plus-CCDI arrangement ([`rssi_tagging`](super::super::rssi_tagging))
//! which has full per-frame signal telemetry but needs two devices. #2 (this) and #4
//! (RSSI tagging) are therefore mutually exclusive on one port.
//!
//! ## Escape / recovery WARNING
//!
//! [`TaitTransparentTransport::exit`] runs the §1.7.2 guard sequence (idle — escape
//! char ×3 — idle). The ~2.1 s idle guards **stall the owning task** — the core has
//! no clock, so the guard is injected by the caller (the firmware supplies an
//! `embassy_time::Timer` sleep; tests supply a no-op). **If the radio is programmed
//! with "Ignore Escape Sequence" ON, the `+++` escape does nothing and there is no
//! software way out — recovery is a power cycle.** Program the radio with the escape
//! sequence honoured before running this transport unattended.

use core::future::Future;

use super::super::RadioMetadata;
use super::ccdi::CcdiFrame;
use crate::kiss::serial::{ByteStream, ModemError, MAX_AX25_BODY};
use crate::kiss::{encode_into, max_encoded_len, Command, Decoder, FEND, FESC};

#[cfg(feature = "alloc")]
use crate::kiss::Frame;
#[cfg(feature = "alloc")]
use alloc::vec::Vec;

/// Outbound SLIP scratch, sized for the worst-case escaping of [`MAX_AX25_BODY`]
/// (shared with the serial-KISS modem's sizing).
const OUT_BUF_LEN: usize = max_encoded_len(MAX_AX25_BODY);

/// The §1.7.2 escape guard: the idle period either side of the `+++` burst. Default
/// 2.1 s (the protocol minimum), in micros — pass to the caller's guard timer.
pub const ESCAPE_GUARD_US: u64 = 2_100_000;

/// The FFSK over-air baud used to estimate frame airtime. Default 2400 — the
/// TM8110's internal FFSK modem raw rate (`TaitTransparentTransportOptions.FfskBaud`).
pub const DEFAULT_FFSK_BAUD: u32 = 2400;

/// The port used for the SLIP framing on the single-channel byte pipe. Port 0,
/// `Data` command — the 1-byte overhead is negligible and lets the tested KISS codec
/// be reused verbatim (mirrors the C# `SlipPort = 0`).
const SLIP_PORT: u8 = 0;

/// Options for [`TaitTransparentTransport`]: the escape character, the FFSK over-air
/// baud (airtime estimation), and whether to select the THSD modem on entry.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TransparentOptions {
    /// The Transparent-mode escape character (§1.7.2) — sent ×3 (guarded by idle
    /// time) to leave Transparent. Default `'+'` (the `+++` sequence).
    pub escape_char: u8,
    /// The FFSK over-air baud for airtime estimation. Default [`DEFAULT_FFSK_BAUD`].
    pub ffsk_baud: u32,
    /// Select the THSD modem (`H`) instead of FFSK (`0`) on entry. Default `false`.
    pub thsd: bool,
}

impl Default for TransparentOptions {
    fn default() -> Self {
        Self {
            escape_char: b'+',
            ffsk_baud: DEFAULT_FFSK_BAUD,
            thsd: false,
        }
    }
}

/// An AX.25 transport whose modem *is* a Tait radio in Transparent mode. Generic
/// over the byte transport `S`, owning the SLIP encode buffer and the streaming KISS
/// decoder.
pub struct TaitTransparentTransport<S: ByteStream> {
    stream: S,
    decoder: Decoder,
    out: [u8; OUT_BUF_LEN],
    /// Inbound read scratch.
    rx: [u8; 256],
    /// Frames decoded from the last read but not yet returned.
    #[cfg(feature = "alloc")]
    pending: Vec<Frame>,
    options: TransparentOptions,
}

impl<S: ByteStream> TaitTransparentTransport<S> {
    /// Wrap a byte stream. Does not enter Transparent mode — call [`Self::enter`].
    pub fn new(stream: S, options: TransparentOptions) -> Self {
        Self {
            stream,
            decoder: Decoder::new(),
            out: [0u8; OUT_BUF_LEN],
            rx: [0u8; 256],
            #[cfg(feature = "alloc")]
            pending: Vec::new(),
            options,
        }
    }

    /// Borrow the underlying stream.
    pub fn stream(&self) -> &S {
        &self.stream
    }

    /// Mutably borrow the underlying stream (host tests feed/drain the loopback here).
    pub fn stream_mut(&mut self) -> &mut S {
        &mut self.stream
    }

    /// The CCDI `t` frame that enters Transparent mode — `t` `"{escape}{modem}"`
    /// where modem is `H` (THSD) or `0` (FFSK). Mirrors
    /// `EnterTransparentModeAsync`'s `new CcdiFrame('t', $"{escapeChar}{...}")`.
    pub fn enter_frame(&self) -> CcdiFrame {
        let params = [self.options.escape_char, if self.options.thsd { b'H' } else { b'0' }];
        CcdiFrame::new(b't', &params).expect("2-byte params fit")
    }

    /// The `+++` (escape char ×3) escape burst.
    pub fn escape_bytes(&self) -> [u8; 3] {
        [self.options.escape_char; 3]
    }

    /// Enter Transparent mode by writing the CCDI `t` command (with its CR). After
    /// this the port is a byte pipe: [`Self::send`] transmits, [`Self::read_frame`]
    /// receives, and no CCDI/PROGRESS is available until [`Self::exit`].
    ///
    /// Unlike the C# `EnterTransparentModeAsync`, this does not await the radio's
    /// prompt (the transport owns no CCDI transact engine — that is the driver's job,
    /// and once `t` is accepted the port stops speaking CCDI). The caller is
    /// responsible for the stale-Transparent recovery dance if a prior session left
    /// the pipe open; the [`Self::exit`] escape primitive is provided for it.
    pub async fn enter(&mut self) -> Result<(), ModemError<S::Error>> {
        let frame = self.enter_frame();
        let n = frame
            .encode_to_bytes_into(&mut self.out)
            .ok_or(ModemError::TooLarge)?;
        let Self { stream, out, .. } = self;
        stream.write(&out[..n]).await.map_err(ModemError::Io)
    }

    /// SLIP-frame `ax25` and transmit it over the byte pipe. Returns the estimated
    /// **airtime in micros** (on-air SLIP bytes × 8 ÷ FFSK baud) — the TX-side twin
    /// of the inbound [`RadioMetadata::estimated_airtime_us`]. Mirrors
    /// `SendFramedAsync`. `ModemError::TooLarge` if the body exceeds [`MAX_AX25_BODY`].
    pub async fn send(&mut self, ax25: &[u8]) -> Result<u64, ModemError<S::Error>> {
        let n = encode_into(&mut self.out, SLIP_PORT, Command::Data, ax25)
            .ok_or(ModemError::TooLarge)?;
        let airtime = airtime_us(n, self.options.ffsk_baud);
        {
            let Self { stream, out, .. } = self;
            stream.write(&out[..n]).await.map_err(ModemError::Io)?;
        }
        Ok(airtime)
    }

    /// Read the next inbound AX.25 frame from the byte pipe, awaiting + SLIP-decoding
    /// stream bytes as needed. Returns the decoded [`Frame`] and its
    /// [`RadioMetadata`] (airtime only — no signal telemetry in Transparent mode), or
    /// `Ok(None)` on EOF / link-down. Mirrors `OnTransparentData` + the KISS pump.
    #[cfg(feature = "alloc")]
    pub async fn read_frame(
        &mut self,
    ) -> Result<Option<(Frame, RadioMetadata)>, ModemError<S::Error>> {
        loop {
            if !self.pending.is_empty() {
                let frame = self.pending.remove(0);
                // On-air size = the exact SLIP-framed byte count this frame was
                // carried as (deterministic from content) — symmetric with TX.
                let air = airtime_us(slip_on_air_len(&frame.payload), self.options.ffsk_baud);
                let meta = RadioMetadata {
                    estimated_airtime_us: Some(air),
                    ..RadioMetadata::default()
                };
                return Ok(Some((frame, meta)));
            }
            let n = {
                let Self { stream, rx, .. } = self;
                stream.read(rx).await.map_err(ModemError::Io)?
            };
            if n == 0 {
                return Ok(None);
            }
            let frames = self.decoder.push(&self.rx[..n]);
            if frames.is_empty() {
                continue;
            }
            // Keep only Data frames with a non-empty payload (the C# filter).
            self.pending = frames
                .into_iter()
                .filter(|f| f.command == Command::Data && !f.payload.is_empty())
                .collect();
        }
    }

    /// Escape Transparent mode: run the §1.7.2 guard sequence (idle — escape char ×3
    /// — idle), then Command mode is back. The idle guards are supplied by `guard`
    /// (called twice) so the core stays clock-free: the firmware passes a closure
    /// that sleeps [`ESCAPE_GUARD_US`]; tests pass a no-op. See the module WARNING
    /// about the "Ignore Escape Sequence" lockout. Mirrors `ExitTransparentModeAsync`.
    pub async fn exit<G, Fut>(&mut self, mut guard: G) -> Result<(), ModemError<S::Error>>
    where
        G: FnMut() -> Fut,
        Fut: Future<Output = ()>,
    {
        guard().await;
        let escape = self.escape_bytes();
        self.stream.write(&escape).await.map_err(ModemError::Io)?;
        guard().await;
        Ok(())
    }
}

/// Estimated airtime, micros: `on_air_bytes × 8 × 1_000_000 ÷ ffsk_baud`. Mirrors
/// the C# `EstimateAirtime` (`onAirByteCount × 8 ÷ FfskBaud` seconds).
pub fn airtime_us(on_air_bytes: usize, ffsk_baud: u32) -> u64 {
    if ffsk_baud == 0 {
        return 0;
    }
    on_air_bytes as u64 * 8 * 1_000_000 / ffsk_baud as u64
}

/// The exact SLIP-framed byte count for a port-0 `Data` payload, without encoding it:
/// two FENDs, the (never-escaped) 0x00 command byte, and each payload byte counted as
/// 1 (or 2 if it is FEND/FESC and must be escaped).
fn slip_on_air_len(payload: &[u8]) -> usize {
    let mut n = 3; // two FENDs + the 0x00 command byte
    for &b in payload {
        n += if b == FEND || b == FESC { 2 } else { 1 };
    }
    n
}

#[cfg(all(test, feature = "alloc"))]
mod tests {
    use super::*;
    use crate::kiss::serial::MemStream;
    use core::future::Future;

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

    fn transport() -> TaitTransparentTransport<MemStream> {
        TaitTransparentTransport::new(MemStream::new(), TransparentOptions::default())
    }

    #[test]
    fn enter_writes_ccdi_t_command_with_cr() {
        let mut t = transport();
        block_on(t.enter()).unwrap();
        // 't' + "+0" (FFSK) + checksum + CR.
        assert_eq!(t.stream_mut().take_written(), b"t02+0CF\r");
    }

    #[test]
    fn enter_selects_thsd_modem_when_requested() {
        let mut t = TaitTransparentTransport::new(
            MemStream::new(),
            TransparentOptions {
                thsd: true,
                ..TransparentOptions::default()
            },
        );
        block_on(t.enter()).unwrap();
        assert_eq!(t.stream_mut().take_written(), b"t02+HB7\r");
    }

    #[test]
    fn send_slip_frames_the_ax25_body_and_reports_airtime() {
        let mut t = transport();
        let air = block_on(t.send(&[0x01, 0x02, 0x03])).unwrap();
        // SLIP wire = FEND, 0x00, payload, FEND (reusing the KISS encoder).
        assert_eq!(t.stream_mut().take_written(), &[FEND, 0x00, 0x01, 0x02, 0x03, FEND]);
        // 6 on-air bytes × 8 × 1e6 ÷ 2400 = 20000 µs.
        assert_eq!(air, 20_000);
    }

    #[test]
    fn send_airtime_accounts_for_slip_escaping() {
        let mut t = transport();
        // Payload with a FEND and a FESC → each escaped to 2 bytes on air.
        let air = block_on(t.send(&[FEND, FESC])).unwrap();
        // FEND, 0x00, FESC,TFEND, FESC,TFESC, FEND = 7 on-air bytes.
        assert_eq!(t.stream_mut().take_written().len(), 7);
        assert_eq!(air, 7 * 8 * 1_000_000 / 2400);
    }

    #[test]
    fn read_frame_decodes_a_slip_frame_with_airtime_only_metadata() {
        let mut t = transport();
        t.stream_mut().feed(&[FEND, 0x00, 0xDE, 0xAD, FEND]);
        let (frame, meta) = block_on(t.read_frame()).unwrap().unwrap();
        assert_eq!(frame.command, Command::Data);
        assert_eq!(frame.payload, alloc::vec![0xDE, 0xAD]);
        // Airtime present; all signal telemetry absent (no CCDI in Transparent mode).
        assert_eq!(meta.estimated_airtime_us, Some(airtime_us(5, 2400)));
        assert_eq!(meta.rssi_dbm_tenths, None);
        assert_eq!(meta.noise_floor_dbm_tenths, None);
        assert_eq!(meta.carrier_rise_at_us, None);
    }

    #[test]
    fn send_then_receive_round_trips_through_two_transports_over_ffsk() {
        // A → wire → B: the data-plane loopback over the byte pipe, exercising the
        // reused KISS encode → SLIP wire → streaming decode end to end.
        let mut a = transport();
        let mut b = transport();
        let body = alloc::vec![0xA8, 0x8A, FEND, FESC, 0x03, 0xF0, b'h', b'i'];
        block_on(a.send(&body)).unwrap();
        let wire = a.stream_mut().take_written();
        // Deliver in awkward 3-byte chunks to stress the streaming decoder.
        for chunk in wire.chunks(3) {
            b.stream_mut().feed(chunk);
        }
        let (frame, _meta) = block_on(b.read_frame()).unwrap().unwrap();
        assert_eq!(frame.payload, body);
    }

    #[test]
    fn exit_runs_escape_burst_between_two_guard_delays() {
        let mut t = transport();
        let mut guard_calls = 0u32;
        block_on(t.exit(|| {
            guard_calls += 1;
            core::future::ready(())
        }))
        .unwrap();
        // The +++ burst went out, guarded by two idle periods.
        assert_eq!(t.stream_mut().take_written(), b"+++");
        assert_eq!(guard_calls, 2);
    }

    #[test]
    fn read_frame_returns_none_on_empty_pipe() {
        let mut t = transport();
        assert!(block_on(t.read_frame()).unwrap().is_none());
    }

    #[test]
    fn slip_on_air_len_matches_the_real_encoder() {
        // The airtime byte-count helper must agree with encode_into's output length.
        for payload in [
            &[0x01u8, 0x02][..],
            &[FEND, FESC, 0x00][..],
            &[FEND, FEND, FEND][..],
        ] {
            let mut buf = [0u8; 64];
            let n = encode_into(&mut buf, SLIP_PORT, Command::Data, payload).unwrap();
            assert_eq!(slip_on_air_len(payload), n, "payload {payload:?}");
        }
    }
}
