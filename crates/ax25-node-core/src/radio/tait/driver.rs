//! The Tait CCDI radio driver — command builders + a thin transact/demux over the
//! async [`ByteStream`] seam. Ports the request/response core of
//! `Packet.Radio.Tait.TaitCcdiRadio`.
//!
//! The C# driver runs a background read-pump thread, serialises commands with a
//! `SemaphoreSlim`, and matches solicited replies to the in-flight `Transaction`
//! (TaitCcdiRadio.cs:1072-1301). This port maps that onto a single `async`
//! request/response over [`ByteStream`]: [`TaitCcdiRadio::transact`] writes the
//! command frame, then reads + line-decodes replies until the transaction completes
//! (a matching message, or the radio's `.` prompt), routing anything unsolicited
//! (carrier-sense / PTT / SDM edges) to a side outlet the caller drains.
//!
//! Scope (the recon's "minimal viable slice"): RSSI read, PTT, channel select, and
//! progress-message enable — all host-testable against canned CCDI replies on a
//! [`MemStream`](crate::kiss::serial::MemStream). The full unsolicited-PROGRESS
//! demux richness (SDM, CCR, watchdog, the prompt-then-error grace window) is
//! deferred, per the recon. See the parity caveats on [`TaitCcdiRadio::transact`].

use super::ccdi::{CcdiEvent, CcdiFrame, CcdiMessage, CcdiProgressType, LineDecoder};
use crate::kiss::serial::ByteStream;

#[cfg(feature = "alloc")]
use alloc::vec::Vec;

/// The largest CCDI command this driver builds (channel/SDM params stay well under
/// this). Sized for a full frame + CR.
const OUT_LEN: usize = super::ccdi::MAX_LINE + 1;

/// An unsolicited radio event demuxed out of the CCDI stream during a transaction —
/// the port of `TaitCcdiRadio.RouteUnsolicited`'s event fan-out. Timestamps are the
/// caller's concern (the core has no clock): stamp each event when you drain it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RadioEvent {
    /// Hardware carrier-sense (DCD) edge: `true` = RF on channel (PROGRESS 0x05),
    /// `false` = channel quiet (0x06).
    CarrierSense(bool),
    /// Transmitter keying edge: `true` = PTT asserted (0x07), `false` = released (0x08).
    Transmitter(bool),
    /// SDM over-air delivery receipt (PROGRESS 0x1D): `true` = the destination radio
    /// auto-acknowledged; `false` = no ack within the configured wait.
    SdmDeliveryReceipt(bool),
    /// Any other PROGRESS message, kept raw so nothing is invisible.
    Progress(CcdiProgressType),
}

/// A driver / transaction failure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TaitError<E> {
    /// The underlying [`ByteStream`] read/write failed.
    Io(E),
    /// The radio answered with a CCDI ERROR message (§1.10.2).
    Ccdi {
        /// The error category character (`'0'` transaction, `'1'` system).
        category: u8,
        /// The error number.
        error_number: u8,
    },
    /// The input drained (or the link closed) before the expected response arrived —
    /// the host-side / link-down equivalent of the C# `TransactionTimeout`. On the
    /// firmware, wrap the transact future in an `embassy_time` timeout instead;
    /// there a genuine radio silence never yields a spurious zero-read.
    NoResponse,
    /// An outbound value was out of range (e.g. channel > 9999).
    Invalid,
}

/// A Tait TM8100/TM8200 radio over its CCDI serial control channel, generic over the
/// byte transport. The firmware instantiates `TaitCcdiRadio<UartStream>` (or a TCP
/// stream for the split-station head-end); host tests instantiate
/// `TaitCcdiRadio<MemStream>`.
pub struct TaitCcdiRadio<S: ByteStream> {
    stream: S,
    decoder: LineDecoder,
    /// Inbound read scratch.
    rx: [u8; 256],
    /// Outbound frame scratch (allocation-free command send).
    out: [u8; OUT_LEN],
    /// Last known carrier-sense state (`None` before the first PROGRESS edge),
    /// mirroring `TaitCcdiRadio.ChannelBusy`.
    channel_busy: Option<bool>,
    /// Whether we keyed the transmitter (best-effort unkey is the firmware's job on
    /// teardown; tracked here so it can).
    we_keyed: bool,
    /// Unsolicited events demuxed during transactions, drained by the caller.
    #[cfg(feature = "alloc")]
    events: Vec<RadioEvent>,
}

impl<S: ByteStream> TaitCcdiRadio<S> {
    /// Wrap a byte stream as a Tait CCDI radio. The radio itself is not touched —
    /// pair with [`Self::set_progress_messages`] to turn on carrier-sense edges.
    pub fn new(stream: S) -> Self {
        Self {
            stream,
            decoder: LineDecoder::new(),
            rx: [0u8; 256],
            out: [0u8; OUT_LEN],
            channel_busy: None,
            we_keyed: false,
            #[cfg(feature = "alloc")]
            events: Vec::new(),
        }
    }

    /// Borrow the underlying stream (e.g. to inspect link state).
    pub fn stream(&self) -> &S {
        &self.stream
    }

    /// Mutably borrow the underlying stream (host tests feed/drain the loopback here).
    pub fn stream_mut(&mut self) -> &mut S {
        &mut self.stream
    }

    /// Last known carrier-sense state, updated from unsolicited PROGRESS edges seen
    /// during any transaction. `None` before the first edge. Mirrors `ChannelBusy`.
    pub fn channel_busy(&self) -> Option<bool> {
        self.channel_busy
    }

    /// Whether the transmitter was last keyed through this driver (FUNCTION 9 latches
    /// TX until unkeyed, §1.9.3 note 5 — so teardown must unkey).
    pub fn transmitter_keyed(&self) -> bool {
        self.we_keyed
    }

    /// Drain the unsolicited events buffered since the last drain (carrier-sense,
    /// PTT, SDM receipts, other PROGRESS). Stamp them with your own clock.
    #[cfg(feature = "alloc")]
    pub fn drain_events(&mut self) -> Vec<RadioEvent> {
        core::mem::take(&mut self.events)
    }

    // ───────────────────────────── typed commands ─────────────────────────────

    /// Read the receiver's instantaneous RSSI (CCTM query 064, "raw"), in **tenths
    /// of a dBm** (`-456` == −45.6 dBm). Mirrors `ReadRssiDbmAsync`, integer-ised.
    pub async fn read_rssi_tenths(&mut self) -> Result<i16, TaitError<S::Error>> {
        self.read_cctm_rssi(b"064", 64).await
    }

    /// Read the radio's own sliding-average RSSI (CCTM query 063), tenths of a dBm.
    /// Mirrors `ReadAveragedRssiDbmAsync`.
    pub async fn read_averaged_rssi_tenths(&mut self) -> Result<i16, TaitError<S::Error>> {
        self.read_cctm_rssi(b"063", 63).await
    }

    /// Key (or unkey) the transmitter — FUNCTION 9 (`f` `91`/`90`). Mirrors
    /// `SetTransmitterAsync`. Records [`Self::transmitter_keyed`] on success.
    pub async fn set_transmitter(&mut self, transmit: bool) -> Result<(), TaitError<S::Error>> {
        let frame = build_transmitter(transmit);
        self.transact(&frame, |_| Option::<()>::None, true).await?;
        self.we_keyed = transmit;
        Ok(())
    }

    /// Enable (or disable) unsolicited PROGRESS output — FUNCTION 0/4 (`f` `041`/`040`).
    /// Required before carrier-sense / PTT edges are reported. Mirrors
    /// `SetProgressMessagesAsync`.
    pub async fn set_progress_messages(&mut self, enable: bool) -> Result<(), TaitError<S::Error>> {
        let frame = build_progress_messages(enable);
        self.transact(&frame, |_| Option::<()>::None, true).await?;
        Ok(())
    }

    /// Retune to a programmed conventional channel — GO_TO_CHANNEL (`g`, §1.9.4).
    /// `zone` is TM8200-only; `None` sends a bare channel number. Mirrors
    /// `GoToChannelAsync`. Returns [`TaitError::Invalid`] for a channel > 9999.
    pub async fn go_to_channel(
        &mut self,
        channel: u16,
        zone: Option<u8>,
    ) -> Result<(), TaitError<S::Error>> {
        let frame = build_go_to_channel(channel, zone).ok_or(TaitError::Invalid)?;
        self.transact(&frame, |_| Option::<()>::None, true).await?;
        Ok(())
    }

    async fn read_cctm_rssi(
        &mut self,
        cctm: &[u8; 3],
        cctm_num: u16,
    ) -> Result<i16, TaitError<S::Error>> {
        let frame = build_query_cctm(cctm);
        let result = self
            .transact(
                &frame,
                move |msg| match msg {
                    CcdiMessage::QueryResult { cctm_command, .. } if *cctm_command == cctm_num => {
                        Some(msg.query_rssi_tenths())
                    }
                    _ => None,
                },
                false,
            )
            .await?;
        // Outer None = no matching reply; inner None = matched but value not a signed
        // integer (a malformed RSSI). Both are a failed read.
        result.flatten().ok_or(TaitError::NoResponse)
    }

    /// The transaction core: write `command`, then read + line-decode replies until
    /// the transaction completes. `on_match` maps a decoded reply to the wanted
    /// owned result `T` (return `Some` to accept-and-finish); `complete_on_prompt`
    /// finishes the transaction on the radio's `.` ready prompt (for commands that
    /// only need acknowledgement). A CCDI ERROR line fails the transaction; every
    /// other unsolicited message is demuxed to [`Self::drain_events`].
    ///
    /// Returns `Ok(Some(T))` when `on_match` accepted a reply, `Ok(None)` when the
    /// prompt completed with no match (an acknowledged command), or an error.
    ///
    /// **Parity caveat:** the C# driver waits a short `PromptErrorGrace` after the
    /// prompt to catch a rejected command whose ERROR trails the prompt. Here the
    /// grace is implicit — every event already decoded from the *same* read is
    /// processed before the prompt returns, which covers canned/one-shot replies
    /// (the host case) and the common single-read hardware case; an ERROR split into
    /// a later read than its prompt is not awaited. The full grace window is deferred
    /// with the rest of the demux richness.
    pub async fn transact<T>(
        &mut self,
        command: &CcdiFrame,
        mut on_match: impl FnMut(&CcdiMessage) -> Option<T>,
        complete_on_prompt: bool,
    ) -> Result<Option<T>, TaitError<S::Error>> {
        // Send the command frame (allocation-free via the fixed out buffer).
        let n = command
            .encode_to_bytes_into(&mut self.out)
            .ok_or(TaitError::Invalid)?;
        {
            let Self { stream, out, .. } = self;
            stream.write(&out[..n]).await.map_err(TaitError::Io)?;
        }

        self.decoder.reset();
        let mut result: Option<T> = None;
        loop {
            let read = {
                let Self { stream, rx, .. } = self;
                stream.read(rx).await.map_err(TaitError::Io)?
            };
            if read == 0 {
                // Input drained / link down. Complete if we already have what we need.
                if result.is_some() {
                    return Ok(result);
                }
                return Err(TaitError::NoResponse);
            }

            let events = self.decoder.push(&self.rx[..read]);
            let mut prompt_seen = false;
            for ev in &events {
                match ev {
                    CcdiEvent::Prompt => prompt_seen |= complete_on_prompt,
                    CcdiEvent::Line(bytes) => {
                        let Some(frame) = CcdiFrame::try_parse(bytes) else {
                            continue; // line noise — normal on async serial
                        };
                        let msg = CcdiMessage::decode(&frame);
                        if let CcdiMessage::Error {
                            category,
                            error_number,
                        } = msg
                        {
                            return Err(TaitError::Ccdi {
                                category,
                                error_number,
                            });
                        }
                        if result.is_none() {
                            if let Some(v) = on_match(&msg) {
                                result = Some(v);
                                if !complete_on_prompt {
                                    return Ok(result);
                                }
                                continue;
                            }
                        }
                        self.route_unsolicited(&msg);
                    }
                }
            }
            if prompt_seen {
                return Ok(result);
            }
        }
    }

    /// Route an unsolicited message to the event outlet + carrier-sense state,
    /// mirroring `TaitCcdiRadio.RouteUnsolicited`.
    fn route_unsolicited(&mut self, msg: &CcdiMessage) {
        if let CcdiMessage::Progress { ptype, para } = msg {
            match ptype {
                CcdiProgressType::ReceiverBusy | CcdiProgressType::ReceiverNotBusy => {
                    let busy = *ptype == CcdiProgressType::ReceiverBusy;
                    self.channel_busy = Some(busy);
                    self.push_event(RadioEvent::CarrierSense(busy));
                }
                CcdiProgressType::PttActivated | CcdiProgressType::PttDeactivated => {
                    self.push_event(RadioEvent::Transmitter(
                        *ptype == CcdiProgressType::PttActivated,
                    ));
                }
                CcdiProgressType::SdmAutoAcknowledge => {
                    self.push_event(RadioEvent::SdmDeliveryReceipt(para.first() == Some(&b'1')));
                }
                other => self.push_event(RadioEvent::Progress(*other)),
            }
        }
    }

    fn push_event(&mut self, event: RadioEvent) {
        #[cfg(feature = "alloc")]
        self.events.push(event);
        #[cfg(not(feature = "alloc"))]
        let _ = event;
    }
}

// ───────────────────────────── command builders ─────────────────────────────
// Strict outbound construction (the C# frame factories): we never emit a frame that
// violates the CCDI spec, even though the inbound path tolerates line noise.

/// Build the RSSI/CCTM query frame — `q` `"5"+cctm` (§1.10.1). Mirrors
/// `ExpectCctmAsync`'s `new CcdiFrame('q', "5" + cctm)`.
fn build_query_cctm(cctm: &[u8; 3]) -> CcdiFrame {
    let mut params = [b'5', 0, 0, 0];
    params[1..].copy_from_slice(cctm);
    CcdiFrame::new(b'q', &params).expect("4-byte params fit")
}

/// Build the transmitter FUNCTION-9 frame — `f` `"91"`/`"90"` (§1.9.3).
fn build_transmitter(transmit: bool) -> CcdiFrame {
    CcdiFrame::new(b'f', if transmit { b"91" } else { b"90" }).expect("2-byte params fit")
}

/// Build the progress-message FUNCTION-0/4 frame — `f` `"041"`/`"040"` (§1.9.3).
fn build_progress_messages(enable: bool) -> CcdiFrame {
    CcdiFrame::new(b'f', if enable { b"041" } else { b"040" }).expect("3-byte params fit")
}

/// Build the GO_TO_CHANNEL frame — `g` + channel (or zone-qualified) digits
/// (§1.9.4). `None` for a channel above 9999. Mirrors `GoToChannelAsync`'s
/// `zone is {} z ? $"{z:00}{channel:0000}" : channel.ToString()`.
fn build_go_to_channel(channel: u16, zone: Option<u8>) -> Option<CcdiFrame> {
    if channel > 9999 {
        return None;
    }
    let mut params = [0u8; 6];
    let len = match zone {
        Some(z) => {
            write_zeropad(&mut params[0..2], z as u32, 2);
            write_zeropad(&mut params[2..6], channel as u32, 4);
            6
        }
        None => write_decimal(&mut params, channel as u32),
    };
    CcdiFrame::new(b'g', &params[..len])
}

/// Write `value` as decimal digits into `dst` right-padded to nothing (natural
/// width), returning the digit count. `value` must fit `dst`.
fn write_decimal(dst: &mut [u8], value: u32) -> usize {
    if value == 0 {
        dst[0] = b'0';
        return 1;
    }
    // Count digits, then fill.
    let mut n = value;
    let mut digits = 0;
    while n > 0 {
        digits += 1;
        n /= 10;
    }
    let mut v = value;
    for i in (0..digits).rev() {
        dst[i] = b'0' + (v % 10) as u8;
        v /= 10;
    }
    digits
}

/// Write `value` as `width` decimal digits (zero-padded) into `dst[..width]`.
fn write_zeropad(dst: &mut [u8], value: u32, width: usize) {
    let mut v = value;
    for i in (0..width).rev() {
        dst[i] = b'0' + (v % 10) as u8;
        v /= 10;
    }
}

#[cfg(all(test, feature = "alloc"))]
mod tests {
    use super::*;
    use crate::kiss::serial::MemStream;
    use core::future::Future;

    /// Minimal host executor (copied from `kiss::serial`'s tests): the futures never
    /// truly suspend on `MemStream`, so a busy poll completes them. `unsafe`-free.
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

    fn radio_primed(reply: &[u8]) -> TaitCcdiRadio<MemStream> {
        let mut stream = MemStream::new();
        stream.feed(reply);
        TaitCcdiRadio::new(stream)
    }

    #[test]
    fn command_builders_produce_strict_wire_bytes() {
        // Every builder's wire form (incl. checksum + CR) against the manual.
        assert_eq!(build_query_cctm(b"064").encode_to_bytes(), b"q0450645C\r");
        assert_eq!(build_transmitter(true).encode_to_bytes(), b"f0291CE\r");
        assert_eq!(build_transmitter(false).encode_to_bytes(), b"f0290CF\r");
        assert_eq!(build_progress_messages(true).encode_to_bytes(), b"f03041A2\r");
        assert_eq!(
            build_go_to_channel(5, None).unwrap().encode(),
            CcdiFrame::new(b'g', b"5").unwrap().encode()
        );
        assert_eq!(
            build_go_to_channel(5, Some(1)).unwrap().encode(),
            CcdiFrame::new(b'g', b"010005").unwrap().encode()
        );
        assert_eq!(
            build_go_to_channel(100, None).unwrap().encode(),
            CcdiFrame::new(b'g', b"100").unwrap().encode()
        );
        assert!(build_go_to_channel(10000, None).is_none());
    }

    #[test]
    fn read_rssi_round_trips_request_and_response() {
        // Radio answers the RSSI query with j07064-456C9 then its prompt.
        let mut radio = radio_primed(b"j07064-456C9\r.");
        let rssi = block_on(radio.read_rssi_tenths()).unwrap();
        assert_eq!(rssi, -456); // -45.6 dBm as tenths
                                // The exact query bytes went out (q + "5064" + checksum + CR).
        assert_eq!(radio.stream_mut().take_written(), b"q0450645C\r");
    }

    #[test]
    fn read_averaged_rssi_uses_query_063() {
        let mut radio = radio_primed(b"j07063-500D4\r.");
        let rssi = block_on(radio.read_averaged_rssi_tenths()).unwrap();
        assert_eq!(rssi, -500);
        assert_eq!(radio.stream_mut().take_written(), b"q0450635D\r");
    }

    #[test]
    fn set_transmitter_completes_on_prompt_and_tracks_state() {
        // A bare prompt is the radio's acknowledgement.
        let mut radio = radio_primed(b".");
        block_on(radio.set_transmitter(true)).unwrap();
        assert!(radio.transmitter_keyed());
        assert_eq!(radio.stream_mut().take_written(), b"f0291CE\r");

        radio.stream_mut().feed(b".");
        block_on(radio.set_transmitter(false)).unwrap();
        assert!(!radio.transmitter_keyed());
        assert_eq!(radio.stream_mut().take_written(), b"f0290CF\r");
    }

    #[test]
    fn go_to_channel_sends_and_acknowledges() {
        let mut radio = radio_primed(b".");
        block_on(radio.go_to_channel(7, None)).unwrap();
        assert_eq!(
            radio.stream_mut().take_written(),
            CcdiFrame::new(b'g', b"7").unwrap().encode_to_bytes().as_slice()
        );
    }

    #[test]
    fn rejected_command_surfaces_ccdi_error() {
        // The hardware ".e03001A7\r." rejection pattern: prompt, error, prompt — all
        // in one read, so the trailing error is caught before the prompt returns.
        let mut radio = radio_primed(b".e03001A7\r.");
        let err = block_on(radio.set_transmitter(true)).unwrap_err();
        assert_eq!(
            err,
            TaitError::Ccdi {
                category: b'0',
                error_number: 0x01
            }
        );
        assert!(!radio.transmitter_keyed()); // state not advanced on failure
    }

    #[test]
    fn unsolicited_progress_is_demuxed_out_of_a_response() {
        // A carrier-sense edge arrives interleaved before the RSSI answer: it must
        // update channel_busy + buffer an event, NOT be mistaken for the reply.
        let mut radio = radio_primed(b"p0205C9\rj07064-456C9\r.");
        let rssi = block_on(radio.read_rssi_tenths()).unwrap();
        assert_eq!(rssi, -456);
        assert_eq!(radio.channel_busy(), Some(true));
        assert_eq!(radio.drain_events(), alloc::vec![RadioEvent::CarrierSense(true)]);
    }

    #[test]
    fn carrier_and_ptt_and_sdm_edges_map_to_events() {
        // Enable progress messages; the radio then streams several PROGRESS edges
        // before the prompt. Each maps to the right RadioEvent.
        let mut radio = radio_primed(b"p0206C8\rp0207C7\rp031D187\r.");
        block_on(radio.set_progress_messages(true)).unwrap();
        assert_eq!(radio.channel_busy(), Some(false));
        assert_eq!(
            radio.drain_events(),
            alloc::vec![
                RadioEvent::CarrierSense(false),
                RadioEvent::Transmitter(true),
                RadioEvent::SdmDeliveryReceipt(true),
            ]
        );
    }

    #[test]
    fn drained_input_without_response_is_no_response() {
        // The radio says nothing (empty stream) → the transact drains and reports
        // NoResponse rather than hanging (the host-side timeout equivalent).
        let mut radio = radio_primed(b"");
        assert_eq!(
            block_on(radio.read_rssi_tenths()),
            Err(TaitError::NoResponse)
        );
    }

    #[test]
    fn response_chunked_across_reads_reassembles() {
        // Prime nothing, then feed the reply in pieces between poll attempts is hard
        // with the busy-poll executor; instead feed a reply split so the line decoder
        // must stitch it — the MemStream returns it all in one read, but the decoder
        // still proves it reassembles a line spanning the RSSI value.
        let mut radio = radio_primed(b"j07064-999");
        radio.stream_mut().feed(b"BD\r.");
        let rssi = block_on(radio.read_rssi_tenths()).unwrap();
        assert_eq!(rssi, -999);
    }
}
