//! The SDM tuning link — retry + dedupe over a Tait CCDI Short Data Message channel.
//!
//! Ports `Packet.Tune.Core.SdmTuningLink` (and the `SdmTuningLinkOptions` record):
//! the reliability layer that carries [`TuningTelegram`]s over the radio's own
//! small-datagram side channel (Tait SDM). A telegram is compact-encoded to fit one
//! plain SDM, sent as a CCDI `a` datagram over the [`TaitCcdiRadio`] driver, and
//! retried on a radio-level *reject*; the receive path parses buffered SDMs and
//! **dedupes on the telegram sequence number** so a transport retry never surfaces
//! twice.
//!
//! ## The critical default: receipt-tolerant (`WaitForDeliveryReceipt` = false)
//!
//! The TM8110's SDM auto-acknowledge has a refractory window that suppresses the
//! over-air delivery receipt (PROGRESS 0x1D) for close bidirectional traffic, while
//! the SDM *payload* is delivered every time. So by default a [`SdmTuningLink::send`]
//! completes **as soon as the radio accepts the datagram** — it does *not* wait for a
//! delivery receipt, and a delivered send with no receipt is **not** an error. Only a
//! radio command *reject* is retried (with a fresh attempt, same sequence — the
//! responder dedupes). End-to-end reliability is then the caller's application-level
//! reply (send-until-expected-reply). This mirrors the C# option record exactly,
//! including the `false` default; see `SdmTuningLinkOptions.WaitForDeliveryReceipt`
//! and `docs/research/tm8110-sdm-autoack-refractory.md` in the C# reference.
//!
//! ## `no_std` / layering divergences from the C# reference
//!
//! The C# link is event-driven with a background pump, `Task.Delay` timers, a
//! `DateTimeOffset` clock and a `HashSet`/`Channel`. This port keeps the same posture
//! the [`TaitCcdiRadio`] driver established — **the core is clock-free and
//! allocation-frugal, timing lives in the firmware** — so:
//!
//! - Dedupe is a fixed **`[i32; 64]`** ring (evict-oldest), not a `HashSet`.
//! - The inter-attempt backoff is awaited through the [`LinkDelay`] seam (the
//!   firmware supplies an `embassy_time` impl; host tests a no-op). The *other*
//!   duration knobs of [`SdmTuningLinkOptions`] (channel-clear wait, post-receive
//!   guard, receipt timeout, receive poll) are carried faithfully but consumed by the
//!   firmware timing layer, not this core — exactly as the driver deferred its
//!   background pump. Channel-clear etiquette is available via
//!   [`TaitCcdiRadio::channel_busy`] on the borrowed radio.
//! - Inbound telegrams are pulled one at a time by [`SdmTuningLink::poll_receive`]
//!   (borrowing the link's receive buffer) rather than pushed onto an unbounded
//!   channel — no queue, no allocation.
//! - `WaitForDeliveryReceipt` = true consults the receipts the driver demuxed during
//!   the send (via `drain_events`, `alloc`-gated); the *timed* wait for a late receipt
//!   is the firmware's job.

use core::future::Future;

use super::TuningTelegram;
use crate::kiss::serial::ByteStream;
use crate::radio::tait::driver::{TaitCcdiRadio, TaitError};

#[cfg(feature = "alloc")]
use crate::radio::tait::driver::RadioEvent;

/// Characters a plain Tait SDM can carry (CCDI §1.9.8) — the default payload budget.
/// Mirrors `TaitSdmSideChannel.PayloadBudget`.
pub const PAYLOAD_BUDGET: usize = 32;

/// Characters an extended (SFI 04) Tait SDM can carry (CCDI §1.9.8) — the budget when
/// extended SDM is enabled. Mirrors `TaitSdmSideChannel.ExtendedPayloadBudget`.
pub const EXTENDED_PAYLOAD_BUDGET: usize = 128;

/// Length of a Tait SDM data identity (CCDI §1.9.8). Mirrors
/// `TaitSdmSideChannel.IdentityLength`.
pub const IDENTITY_LEN: usize = 8;

/// The sequence-dedupe window — the most-recent-N unique sequences remembered.
/// Mirrors the C# `SdmTuningLink`'s 64-entry `seenOrder` cap.
const DEDUPE_WINDOW: usize = 64;

/// Receive scratch: sized for the largest SDM (an extended payload).
const RX_SDM_LEN: usize = EXTENDED_PAYLOAD_BUDGET;

/// Encode scratch: larger than any SDM budget so an over-budget compact telegram can
/// be measured (and rejected) rather than silently truncated.
const ENC_LEN: usize = 256;

/// Tunables for [`SdmTuningLink`]. Mirrors `Packet.Tune.Core.SdmTuningLinkOptions`
/// field-for-field (durations carried as integer milliseconds — no `TimeSpan` on the
/// no_std target).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SdmTuningLinkOptions {
    /// Delivery attempts per telegram (first try + retries). Default 3.
    pub max_attempts: u8,
    /// Wait between delivery attempts, milliseconds. Default 2000.
    pub retry_backoff_ms: u32,
    /// Maximum wait for the radio's over-air delivery receipt after a send,
    /// milliseconds (only used when [`Self::wait_for_delivery_receipt`]). The radio
    /// waits ~6 s before reporting "not acknowledged", so this exceeds that. Default
    /// 10000. Consumed by the firmware timing layer.
    pub receipt_timeout_ms: u32,
    /// Maximum wait for the channel to go quiet before transmitting, milliseconds (the
    /// link never keys over a busy channel). Default 30000. Consumed by the firmware
    /// timing layer.
    pub channel_clear_timeout_ms: u32,
    /// Poll interval while waiting for the channel to clear, milliseconds. Default 100.
    /// Consumed by the firmware timing layer.
    pub channel_clear_poll_interval_ms: u32,
    /// Fallback receive-buffer poll interval, milliseconds, for arrivals whose RING /
    /// FFSK-progress was missed. Default 1500. Consumed by the firmware timing layer.
    pub receive_poll_interval_ms: u32,
    /// Minimum gap between receiving a telegram and transmitting, milliseconds (the
    /// radio may still be transmitting its own auto-acknowledgement of the
    /// just-received SDM; half-duplex etiquette is not to key over it). Default 2000.
    /// Consumed by the firmware timing layer.
    pub post_receive_guard_ms: u32,
    /// When `true`, [`SdmTuningLink::send`] treats the radio's over-air delivery
    /// receipt (PROGRESS 0x1D) as the success signal and retries on its absence — the
    /// original behaviour, dependable only for unidirectional / well-spaced SDM. When
    /// `false` (**the default**), a send completes as soon as the radio accepts the
    /// datagram; the receipt is advisory only, because it is not reliable for close
    /// bidirectional SDM (the TM8110 auto-ack refractory). Default `false`.
    pub wait_for_delivery_receipt: bool,
}

impl Default for SdmTuningLinkOptions {
    fn default() -> Self {
        Self {
            max_attempts: 3,
            retry_backoff_ms: 2_000,
            receipt_timeout_ms: 10_000,
            channel_clear_timeout_ms: 30_000,
            channel_clear_poll_interval_ms: 100,
            receive_poll_interval_ms: 1_500,
            post_receive_guard_ms: 2_000,
            wait_for_delivery_receipt: false,
        }
    }
}

/// An async delay seam — the no_std stand-in for the C# `Task.Delay` the link uses for
/// its inter-attempt backoff. The firmware supplies an `embassy_time` impl; host tests
/// supply an immediate no-op. Modelled on [`ByteStream`]'s async-fn-in-trait shape, so
/// it is Embassy-usable without an `async-trait` crate.
pub trait LinkDelay {
    /// Sleep for approximately `ms` milliseconds.
    fn delay_ms(&mut self, ms: u32) -> impl Future<Output = ()>;
}

/// A tuning-link transport failure. Mirrors the situations the C# `TuningLinkException`
/// reports, but as a typed enum carrying the underlying [`TaitError`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SdmLinkError<E> {
    /// The compact telegram is larger than the side channel's payload budget — it can
    /// never fit one SDM, so it is rejected before any transmission.
    OverBudget {
        /// The compact telegram's length in characters (a floor if it also overran the
        /// encode scratch).
        len: usize,
        /// The side channel's character budget (32 plain, 128 extended).
        budget: usize,
    },
    /// The radio refused the datagram on every attempt (retries exhausted). Carries the
    /// last radio error.
    NotAccepted {
        /// How many attempts were made.
        attempts: u8,
        /// The last radio error seen.
        last: TaitError<E>,
    },
    /// [`SdmTuningLinkOptions::wait_for_delivery_receipt`] was set and no positive
    /// receipt arrived within the attempts (never returned by the default path).
    NotAcknowledged {
        /// How many attempts were made.
        attempts: u8,
    },
    /// A non-retryable radio error (e.g. an out-of-range argument).
    Radio(TaitError<E>),
}

/// A bounded FIFO set of the most-recent [`DEDUPE_WINDOW`] unique sequences — the
/// no_std stand-in for the C# `HashSet`/`Queue` pair (`MarkSeen`). Membership is an
/// integer scan; a new sequence evicts the oldest once the window is full.
struct SeenSequences {
    ring: [i32; DEDUPE_WINDOW],
    /// Number of populated slots (`0..=DEDUPE_WINDOW`).
    len: usize,
    /// Next write cursor (wraps — the oldest entry once full).
    next: usize,
}

impl SeenSequences {
    const fn new() -> Self {
        Self {
            ring: [0; DEDUPE_WINDOW],
            len: 0,
            next: 0,
        }
    }

    /// Record `seq`. Returns `true` if it was newly seen, `false` if it is a duplicate
    /// still inside the window. Mirrors `SdmTuningLink.MarkSeen`.
    fn mark(&mut self, seq: i32) -> bool {
        if self.ring[..self.len].contains(&seq) {
            return false;
        }
        self.ring[self.next] = seq;
        self.next = (self.next + 1) % DEDUPE_WINDOW;
        if self.len < DEDUPE_WINDOW {
            self.len += 1;
        }
        true
    }
}

/// A tuning link over a [`TaitCcdiRadio`]'s SDM side channel. Owns the radio and a
/// [`LinkDelay`]; carries the send/retry/dedupe policy of `SdmTuningLink`.
pub struct SdmTuningLink<S: ByteStream, D: LinkDelay> {
    radio: TaitCcdiRadio<S>,
    delay: D,
    peer_id: [u8; IDENTITY_LEN],
    options: SdmTuningLinkOptions,
    extended: bool,
    seen: SeenSequences,
    rx_sdm: [u8; RX_SDM_LEN],
    enc: [u8; ENC_LEN],
}

impl<S: ByteStream, D: LinkDelay> SdmTuningLink<S, D> {
    /// Wrap a radio (and a delay source) as a tuning link to the peer whose 8-character
    /// SDM data identity is `peer_id`. Plain SDM by default — see
    /// [`Self::with_extended_sdm`].
    pub fn new(
        radio: TaitCcdiRadio<S>,
        delay: D,
        peer_id: [u8; IDENTITY_LEN],
        options: SdmTuningLinkOptions,
    ) -> Self {
        Self {
            radio,
            delay,
            peer_id,
            options,
            extended: false,
            seen: SeenSequences::new(),
            rx_sdm: [0; RX_SDM_LEN],
            enc: [0; ENC_LEN],
        }
    }

    /// Allow telegrams over the 32-character plain-SDM budget (up to
    /// [`EXTENDED_PAYLOAD_BUDGET`]) to ride an extended SDM. Mirrors the C#
    /// `TaitSdmSideChannelOptions.EnableExtendedSdm`; default off.
    #[must_use]
    pub fn with_extended_sdm(mut self, extended: bool) -> Self {
        self.extended = extended;
        self
    }

    /// The side channel's character budget for one send (32 plain, 128 extended).
    /// Mirrors `IRadioSideChannel.MaxPayloadLength`.
    pub fn max_payload(&self) -> usize {
        if self.extended {
            EXTENDED_PAYLOAD_BUDGET
        } else {
            PAYLOAD_BUDGET
        }
    }

    /// Borrow the underlying radio (e.g. to consult
    /// [`TaitCcdiRadio::channel_busy`] for channel-clear etiquette).
    pub fn radio(&self) -> &TaitCcdiRadio<S> {
        &self.radio
    }

    /// Mutably borrow the underlying radio.
    pub fn radio_mut(&mut self) -> &mut TaitCcdiRadio<S> {
        &mut self.radio
    }

    /// Borrow the delay source.
    pub fn delay(&self) -> &D {
        &self.delay
    }

    /// The tunables in force.
    pub fn options(&self) -> &SdmTuningLinkOptions {
        &self.options
    }

    /// Send one telegram, with the retry + receipt policy of `SdmTuningLink.SendAsync`.
    ///
    /// The telegram is compact-encoded and rejected as [`SdmLinkError::OverBudget`] if
    /// it cannot fit one SDM. Otherwise it is sent (up to
    /// [`SdmTuningLinkOptions::max_attempts`] times, the same sequence each time — the
    /// responder dedupes), retrying only on a radio-level reject. By default
    /// ([`SdmTuningLinkOptions::wait_for_delivery_receipt`] false) the send returns as
    /// soon as the radio **accepts** the datagram — a missing over-air delivery receipt
    /// is **not** an error.
    pub async fn send(
        &mut self,
        telegram: TuningTelegram<'_>,
    ) -> Result<(), SdmLinkError<S::Error>> {
        let budget = self.max_payload();
        let wire_len = match telegram.encode_compact_into(&mut self.enc) {
            Some(n) if n <= budget => n,
            Some(n) => return Err(SdmLinkError::OverBudget { len: n, budget }),
            // Busted the whole scratch buffer → certainly over any SDM budget.
            None => {
                return Err(SdmLinkError::OverBudget {
                    len: ENC_LEN + 1,
                    budget,
                })
            }
        };

        let max = self.options.max_attempts.max(1);
        let extended = self.extended;
        let peer = self.peer_id;
        let mut last: Option<TaitError<S::Error>> = None;
        let mut receipt_missing = false;

        for attempt in 1..=max {
            let outcome = {
                let Self { radio, enc, .. } = self;
                let wire = &enc[..wire_len];
                if extended {
                    radio.send_extended_sdm(&peer, wire).await
                } else {
                    radio.send_sdm(&peer, wire).await
                }
            };

            match outcome {
                Ok(()) => {
                    if !self.options.wait_for_delivery_receipt {
                        // The radio accepted the datagram — on a working channel the
                        // payload is delivered. Don't gate on the (unreliable) receipt.
                        return Ok(());
                    }
                    if self.receipt_acknowledged() {
                        return Ok(());
                    }
                    receipt_missing = true;
                }
                // An out-of-range argument is not worth retrying.
                Err(TaitError::Invalid) => {
                    return Err(SdmLinkError::Radio(TaitError::Invalid))
                }
                // The radio refused the datagram (busy / not-ready / programming
                // rejection) — the one genuine transport failure worth retrying.
                Err(e) => last = Some(e),
            }

            if attempt < max {
                self.delay.delay_ms(self.options.retry_backoff_ms).await;
            }
        }

        Err(match last {
            Some(e) => SdmLinkError::NotAccepted { attempts: max, last: e },
            None if receipt_missing => SdmLinkError::NotAcknowledged { attempts: max },
            None => SdmLinkError::NotAccepted {
                attempts: max,
                last: TaitError::NoResponse,
            },
        })
    }

    /// Poll the radio's one-deep SDM buffer for the next inbound telegram, deduplicated
    /// by sequence number. Returns:
    ///
    /// - `Ok(Some(telegram))` — a fresh telegram (borrowed from the link's receive
    ///   buffer);
    /// - `Ok(None)` — nothing buffered, a non-telegram SDM, or a duplicate sequence
    ///   (dropped);
    /// - `Err(_)` — a radio error.
    ///
    /// Mirrors the read + `TryParse` + `MarkSeen` core of the C# link's inbound pump
    /// (the RING / FFSK-progress arrival events and the fallback poll cadence are the
    /// firmware's job).
    pub async fn poll_receive(
        &mut self,
    ) -> Result<Option<TuningTelegram<'_>>, SdmLinkError<S::Error>> {
        let len = {
            let Self { radio, rx_sdm, .. } = self;
            match radio.read_buffered_sdm_into(rx_sdm).await {
                Ok(Some(n)) => n,
                Ok(None) => return Ok(None),
                Err(e) => return Err(SdmLinkError::Radio(e)),
            }
        };

        // Parse for the sequence first (this borrow ends before the dedupe mutation).
        let sequence = match core::str::from_utf8(&self.rx_sdm[..len])
            .ok()
            .and_then(TuningTelegram::try_parse)
        {
            Some(t) => t.sequence,
            None => return Ok(None), // not a telegram — ignore
        };

        if !self.seen.mark(sequence) {
            return Ok(None); // duplicate (transport retry) — dropped
        }

        // Re-parse for the borrowed return; `rx_sdm` is unchanged since the check.
        let text = core::str::from_utf8(&self.rx_sdm[..len]).expect("validated above");
        Ok(TuningTelegram::try_parse(text))
    }

    /// Whether the radio demuxed a positive SDM delivery receipt (PROGRESS 0x1D) during
    /// the last send. Only meaningful under [`SdmTuningLinkOptions::wait_for_delivery_receipt`].
    /// Without `alloc` there is no event buffer, so this degrades to receipt-tolerant
    /// (treats acceptance as delivered).
    fn receipt_acknowledged(&mut self) -> bool {
        #[cfg(feature = "alloc")]
        {
            self.radio
                .drain_events()
                .into_iter()
                .any(|e| matches!(e, RadioEvent::SdmDeliveryReceipt(true)))
        }
        #[cfg(not(feature = "alloc"))]
        {
            true
        }
    }
}

#[cfg(all(test, feature = "alloc"))]
mod tests {
    use super::*;
    use crate::kiss::serial::{MemStream, MemStreamError};
    use crate::radio::tait::ccdi::CcdiFrame;
    use crate::tune::TuningVerb;
    use alloc::vec;
    use alloc::vec::Vec;
    use core::future::Future;

    /// Minimal host executor (as in the driver's own tests): `MemStream`/`ScriptStream`
    /// futures never truly suspend, so a busy poll completes them. `unsafe`-free.
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

    /// An immediate no-op delay.
    struct NullDelay;
    impl LinkDelay for NullDelay {
        async fn delay_ms(&mut self, _ms: u32) {}
    }

    /// A delay that counts its invocations (to prove backoff is awaited between attempts).
    #[derive(Default)]
    struct CountingDelay {
        calls: u32,
    }
    impl LinkDelay for CountingDelay {
        async fn delay_ms(&mut self, _ms: u32) {
            self.calls += 1;
        }
    }

    /// A `ByteStream` returning one scripted segment per `read` (so a multi-transaction
    /// `send` retry sees each attempt's reply in isolation — a single `MemStream` would
    /// hand the whole inbox to the first read). Each segment must be a complete CCDI
    /// line / prompt.
    struct ScriptStream {
        reads: Vec<Vec<u8>>,
        idx: usize,
        written: Vec<u8>,
    }
    impl ScriptStream {
        fn new(reads: Vec<Vec<u8>>) -> Self {
            Self {
                reads,
                idx: 0,
                written: Vec::new(),
            }
        }
        fn take_written(&mut self) -> Vec<u8> {
            core::mem::take(&mut self.written)
        }
    }
    impl ByteStream for ScriptStream {
        type Error = MemStreamError;
        async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            if self.idx >= self.reads.len() {
                return Ok(0);
            }
            let seg = &self.reads[self.idx];
            let n = seg.len().min(buf.len());
            buf[..n].copy_from_slice(&seg[..n]);
            self.idx += 1;
            Ok(n)
        }
        async fn write(&mut self, bytes: &[u8]) -> Result<(), Self::Error> {
            self.written.extend_from_slice(bytes);
            Ok(())
        }
    }

    fn link_over(
        stream: MemStream,
        options: SdmTuningLinkOptions,
    ) -> SdmTuningLink<MemStream, NullDelay> {
        SdmTuningLink::new(
            TaitCcdiRadio::new(stream),
            NullDelay,
            *b"12345678",
            options,
        )
    }

    /// Queue an inbound SDM (an `s`-frame) for the next `poll_receive`.
    fn feed_sdm<D: LinkDelay>(link: &mut SdmTuningLink<MemStream, D>, data: &[u8]) {
        let frame = CcdiFrame::new(b's', data).unwrap().encode_to_bytes();
        link.radio_mut().stream_mut().feed(&frame);
    }

    #[test]
    fn send_transmits_the_compact_telegram_and_returns_ok() {
        let mut stream = MemStream::new();
        stream.feed(b"."); // bare prompt = accepted
        let mut link = link_over(stream, SdmTuningLinkOptions::default());
        let telegram = TuningTelegram::new(7, TuningVerb::BurstRequest, "5");
        block_on(link.send(telegram)).unwrap();
        // `a` params = lead-in + GFI + SFI + id + the compact "V1|7|RQ|5".
        let expected = CcdiFrame::new(b'a', b"0520012345678V1|7|RQ|5")
            .unwrap()
            .encode_to_bytes();
        assert_eq!(link.radio_mut().stream_mut().take_written(), expected.as_slice());
    }

    #[test]
    fn receipt_tolerant_default_returns_ok_without_a_delivery_receipt() {
        // The critical default: WaitForDeliveryReceipt == false. The radio accepts the
        // datagram (bare prompt) and sends NO PROGRESS 1D receipt — the link must still
        // complete Ok (the TM8110 auto-ack refractory would otherwise falsely fail it).
        let options = SdmTuningLinkOptions::default();
        assert!(!options.wait_for_delivery_receipt);
        let mut stream = MemStream::new();
        stream.feed(b".");
        let mut link = link_over(stream, options);
        let telegram = TuningTelegram::new(1, TuningVerb::Hello, "meter");
        assert_eq!(block_on(link.send(telegram)), Ok(()));
    }

    #[test]
    fn retries_on_a_radio_reject_then_succeeds() {
        let reject = CcdiFrame::new(b'e', b"006").unwrap().encode_to_bytes();
        // attempt 1: reject; attempt 2: prompt (accepted).
        let stream = ScriptStream::new(vec![reject, vec![b'.']]);
        let mut link = SdmTuningLink::new(
            TaitCcdiRadio::new(stream),
            CountingDelay::default(),
            *b"12345678",
            SdmTuningLinkOptions::default(),
        );
        let telegram = TuningTelegram::new(3, TuningVerb::BurstRequest, "5");
        assert_eq!(block_on(link.send(telegram)), Ok(()));

        // The same telegram (same sequence) was transmitted twice.
        let one = CcdiFrame::new(b'a', b"0520012345678V1|3|RQ|5")
            .unwrap()
            .encode_to_bytes();
        let mut two = one.clone();
        two.extend_from_slice(&one);
        assert_eq!(link.radio_mut().stream_mut().take_written(), two.as_slice());
        // Backoff was awaited exactly once (between the two attempts).
        assert_eq!(link.delay().calls, 1);
    }

    #[test]
    fn exhausts_attempts_on_a_persistent_reject() {
        let reject = CcdiFrame::new(b'e', b"006").unwrap().encode_to_bytes();
        let stream = ScriptStream::new(vec![reject.clone(), reject.clone(), reject]);
        let mut link = SdmTuningLink::new(
            TaitCcdiRadio::new(stream),
            NullDelay,
            *b"12345678",
            SdmTuningLinkOptions::default(),
        );
        let telegram = TuningTelegram::new(3, TuningVerb::BurstRequest, "5");
        match block_on(link.send(telegram)) {
            Err(SdmLinkError::NotAccepted {
                attempts: 3,
                last: TaitError::Ccdi { .. },
            }) => {}
            other => panic!("expected NotAccepted after 3 rejects, got {other:?}"),
        }
    }

    #[test]
    fn poll_receive_delivers_and_dedupes_on_sequence() {
        let mut link = link_over(MemStream::new(), SdmTuningLinkOptions::default());

        // seq 5 arrives → delivered.
        feed_sdm(&mut link, b"V1|5|HI|x");
        assert_eq!(
            block_on(link.poll_receive()).unwrap().map(|t| t.sequence),
            Some(5)
        );
        // the same seq 5 again → deduplicated (dropped).
        feed_sdm(&mut link, b"V1|5|HI|x");
        assert_eq!(block_on(link.poll_receive()).unwrap(), None);
        // a fresh seq 6 → delivered.
        feed_sdm(&mut link, b"V1|6|AD|OK");
        let got = block_on(link.poll_receive()).unwrap().unwrap();
        assert_eq!(got.sequence, 6);
        assert_eq!(got.verb, TuningVerb::Advice);
        assert_eq!(got.args, "OK");
    }

    #[test]
    fn poll_receive_ignores_non_telegram_sdm_and_empty_buffer() {
        let mut link = link_over(MemStream::new(), SdmTuningLinkOptions::default());
        // Junk SDM that is not a telegram → None (not an error).
        feed_sdm(&mut link, b"not-a-telegram");
        assert_eq!(block_on(link.poll_receive()).unwrap(), None);
        // Empty buffer → None.
        feed_sdm(&mut link, b"");
        assert_eq!(block_on(link.poll_receive()).unwrap(), None);
    }

    #[test]
    fn wait_for_delivery_receipt_accepts_on_a_positive_receipt() {
        // Opt in to the strict mode: the radio emits PROGRESS 1D "acknowledged" then
        // the prompt. The link must complete Ok.
        let mut stream = MemStream::new();
        stream.feed(b"p031D187\r.");
        let options = SdmTuningLinkOptions {
            wait_for_delivery_receipt: true,
            ..Default::default()
        };
        let mut link = link_over(stream, options);
        let telegram = TuningTelegram::new(9, TuningVerb::Bye, "");
        assert_eq!(block_on(link.send(telegram)), Ok(()));
    }

    #[test]
    fn wait_for_delivery_receipt_errors_without_a_receipt() {
        // Opt in to the strict mode, but every attempt is accepted with NO receipt →
        // NotAcknowledged after the attempts (proving the strict mode really gates on
        // the receipt, unlike the default).
        let stream = ScriptStream::new(vec![vec![b'.'], vec![b'.'], vec![b'.']]);
        let options = SdmTuningLinkOptions {
            wait_for_delivery_receipt: true,
            ..Default::default()
        };
        let mut link = SdmTuningLink::new(
            TaitCcdiRadio::new(stream),
            NullDelay,
            *b"12345678",
            options,
        );
        let telegram = TuningTelegram::new(9, TuningVerb::Bye, "");
        assert_eq!(
            block_on(link.send(telegram)),
            Err(SdmLinkError::NotAcknowledged { attempts: 3 })
        );
    }

    #[test]
    fn over_budget_telegram_is_rejected_before_sending() {
        let mut link = link_over(MemStream::new(), SdmTuningLinkOptions::default());
        // A STAT telegram whose canonical/compact form exceeds the 32-char plain budget.
        let telegram = TuningTelegram::new(
            1,
            TuningVerb::Status,
            "STATUS-FIELD-THAT-IS-DEFINITELY-WAY-TOO-LONG",
        );
        match block_on(link.send(telegram)) {
            Err(SdmLinkError::OverBudget { budget: 32, .. }) => {}
            other => panic!("expected OverBudget, got {other:?}"),
        }
        // Nothing was transmitted.
        assert!(link.radio_mut().stream_mut().take_written().is_empty());
    }

    #[test]
    fn extended_mode_raises_the_budget_and_uses_sfi_04() {
        let mut stream = MemStream::new();
        stream.feed(b".");
        let mut link =
            link_over(stream, SdmTuningLinkOptions::default()).with_extended_sdm(true);
        assert_eq!(link.max_payload(), EXTENDED_PAYLOAD_BUDGET);
        // A STAT telegram over 32 chars now fits (extended budget) and rides SFI 04.
        let args = "STATUS-FIELD-OVER-THIRTY-TWO-CHARACTERS";
        let telegram = TuningTelegram::new(2, TuningVerb::Status, args);
        block_on(link.send(telegram)).unwrap();
        let written = link.radio_mut().stream_mut().take_written();
        // params start with lead-in "05", GFI '2', SFI "04".
        assert_eq!(&written[3..8], b"05204");
    }

    #[test]
    fn options_defaults_mirror_the_c_sharp_record() {
        let o = SdmTuningLinkOptions::default();
        assert_eq!(o.max_attempts, 3);
        assert_eq!(o.retry_backoff_ms, 2_000);
        assert_eq!(o.receipt_timeout_ms, 10_000);
        assert_eq!(o.channel_clear_timeout_ms, 30_000);
        assert_eq!(o.channel_clear_poll_interval_ms, 100);
        assert_eq!(o.receive_poll_interval_ms, 1_500);
        assert_eq!(o.post_receive_guard_ms, 2_000);
        // The critical receipt-tolerant default.
        assert!(!o.wait_for_delivery_receipt);
    }

    #[test]
    fn seen_sequences_ring_evicts_the_oldest_after_the_window_fills() {
        let mut seen = SeenSequences::new();
        for s in 0..DEDUPE_WINDOW as i32 {
            assert!(seen.mark(s), "first sight of {s} should be new");
        }
        // Everything in-window is a duplicate now.
        assert!(!seen.mark(10));
        // A fresh sequence evicts the oldest (0).
        assert!(seen.mark(100));
        // 0 was evicted → admissible again; a never-evicted one is still a duplicate.
        assert!(seen.mark(0));
        assert!(!seen.mark(63));
    }
}
