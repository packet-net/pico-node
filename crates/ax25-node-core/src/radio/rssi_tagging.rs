//! Per-frame RSSI/SNR attribution — the integer port of
//! `Packet.Radio.RssiTaggingTransport`.
//!
//! The C# transport decorates an AX.25 transport with a background RSSI sampler and
//! carrier-sense window tracking, then re-yields each inbound frame with
//! [`RadioMetadata`] populated. This port keeps the **attribution logic** — the part
//! where all the correctness lives — as a pure, allocation-free state machine that
//! is *fed* samples and carrier edges (the sampler task itself is the firmware's
//! Embassy loop, exactly as the recon recommends: I/O-free core, host-testable off a
//! synthetic timeline).
//!
//! ## Integer / no_std substitutions (documented divergences)
//!
//! - Levels are `i16` **tenths of a dB(m)** throughout (no FPU).
//! - The noise-floor EMA `nf + 0.2·(dbm − nf)` becomes a fixed-point integer form
//!   `nf + (dbm − nf)·num/den` with `num/den = 205/1024 ≈ 0.2003` by default — a
//!   documented approximation of the C# `f32` coefficient.
//! - The C# `Queue<>` sample/window buffers become fixed-capacity ring buffers
//!   ([`SAMPLE_CAPACITY`], [`WINDOW_CAPACITY`] — the latter matching the C# cap of
//!   8 closed windows). No `heapless`; plain const-sized arrays.
//! - Timestamps are `u64` micros of a monotonic clock (the firmware's
//!   `embassy_time::Instant` micros; the tests' synthetic timeline).

use super::RadioMetadata;

/// How many RSSI samples the ring retains. At the C# busy cadence (40 ms) over the
/// pruning horizon (2× the 5 s lookback) this is ~250 samples; 256 covers it.
pub const SAMPLE_CAPACITY: usize = 256;

/// How many *closed* carrier windows are retained — the C# `while (windows.Count >
/// 8) Dequeue()` cap.
pub const WINDOW_CAPACITY: usize = 8;

/// Tuning knobs for [`RssiTagger`], integer-ised from `RssiTaggingOptions`. The
/// sampler cadences live in the firmware sampler task, not here, so only the
/// attribution-relevant knobs are carried.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RssiTaggingConfig {
    /// How far back from a frame's arrival samples may be attributed to it in the
    /// no-carrier-sense fallback (micros). Default 5 s.
    pub attribution_lookback_us: u64,
    /// How long past carrier-fall a delivered frame may still be attributed to that
    /// window (micros) — the modem's end-of-frame decode + serial latency. Default
    /// 500 ms.
    pub window_attribution_slack_us: u64,
    /// How far above the noise floor a sample must sit to count as signal in the
    /// no-carrier-sense fallback, tenths of a dB. Default 60 (6 dB).
    pub signal_threshold_tenths: i16,
    /// Numerator of the fixed-point noise-floor EMA coefficient. Default 205.
    pub noise_floor_smoothing_num: i32,
    /// Denominator of the fixed-point noise-floor EMA coefficient. Default 1024
    /// (`205/1024 ≈ 0.2`, the C# coefficient).
    pub noise_floor_smoothing_den: i32,
}

impl Default for RssiTaggingConfig {
    fn default() -> Self {
        Self {
            attribution_lookback_us: 5_000_000,
            window_attribution_slack_us: 500_000,
            signal_threshold_tenths: 60,
            noise_floor_smoothing_num: 205,
            noise_floor_smoothing_den: 1024,
        }
    }
}

#[derive(Debug, Clone, Copy, Default)]
struct Sample {
    at_us: u64,
    dbm_tenths: i16,
}

#[derive(Debug, Clone, Copy, Default)]
struct CarrierWindow {
    rise_at_us: u64,
    fall_at_us: Option<u64>,
    frames_delivered: u16,
}

enum WindowRef {
    Current,
    Closed(usize),
}

/// The pure RSSI/SNR attribution state machine. Feed it [`Self::record_sample`] and
/// [`Self::carrier_edge`]; ask it to [`Self::attribute`] a received frame.
#[derive(Debug)]
pub struct RssiTagger {
    config: RssiTaggingConfig,
    carrier_sense: bool,
    samples: [Sample; SAMPLE_CAPACITY],
    sample_head: usize,
    sample_len: usize,
    windows: [CarrierWindow; WINDOW_CAPACITY],
    window_head: usize,
    window_len: usize,
    current_window: Option<CarrierWindow>,
    noise_floor_tenths: Option<i16>,
}

impl RssiTagger {
    /// Create a tagger. `carrier_sense` mirrors the radio advertising
    /// [`RadioCapabilities::CARRIER_SENSE`](super::RadioCapabilities::CARRIER_SENSE):
    /// when `false`, [`Self::attribute`] always uses the threshold-over-noise-floor
    /// fallback and window-derived fields stay `None`.
    pub fn new(config: RssiTaggingConfig, carrier_sense: bool) -> Self {
        Self {
            config,
            carrier_sense,
            samples: [Sample::default(); SAMPLE_CAPACITY],
            sample_head: 0,
            sample_len: 0,
            windows: [CarrierWindow::default(); WINDOW_CAPACITY],
            window_head: 0,
            window_len: 0,
            current_window: None,
            noise_floor_tenths: None,
        }
    }

    /// The current noise-floor estimate in tenths of a dBm (EMA over channel-idle
    /// samples), or `None` until the first idle sample lands.
    pub fn noise_floor_tenths(&self) -> Option<i16> {
        self.noise_floor_tenths
    }

    /// Record one RSSI sample. `channel_busy` is the radio's carrier-sense state at
    /// sample time (`None` when the radio has no carrier-sense channel). Mirrors
    /// `RssiTaggingTransport.Record`: samples judged *idle* feed the noise-floor EMA.
    pub fn record_sample(&mut self, at_us: u64, dbm_tenths: i16, channel_busy: Option<bool>) {
        self.push_sample(Sample { at_us, dbm_tenths });

        // Prune samples older than now − 2×lookback (the C# horizon).
        let horizon = at_us.saturating_sub(2 * self.config.attribution_lookback_us);
        while self.sample_len > 0 && self.sample_at(0).at_us < horizon {
            self.sample_head = (self.sample_head + 1) % SAMPLE_CAPACITY;
            self.sample_len -= 1;
        }

        // Idle samples feed the noise floor. With carrier-sense, "idle" == "not
        // busy"; without it, "quieter than floor + threshold" (seeded by the first
        // sample ever).
        let idle = match channel_busy {
            Some(busy) => !busy,
            None => match self.noise_floor_tenths {
                None => true,
                Some(nf) => dbm_tenths < nf.saturating_add(self.config.signal_threshold_tenths),
            },
        };
        if idle {
            self.noise_floor_tenths = Some(match self.noise_floor_tenths {
                None => dbm_tenths,
                Some(nf) => {
                    let delta = dbm_tenths as i32 - nf as i32;
                    let step = delta * self.config.noise_floor_smoothing_num
                        / self.config.noise_floor_smoothing_den;
                    (nf as i32 + step) as i16
                }
            });
        }
    }

    /// Feed a hardware carrier-sense (DCD) edge. Mirrors
    /// `RssiTaggingTransport.OnCarrierSenseChanged`: a rise opens the current window,
    /// a fall closes it into the (capped) window ring.
    pub fn carrier_edge(&mut self, busy: bool, at_us: u64) {
        if busy {
            self.current_window = Some(CarrierWindow {
                rise_at_us: at_us,
                fall_at_us: None,
                frames_delivered: 0,
            });
        } else if let Some(mut w) = self.current_window.take() {
            w.fall_at_us = Some(at_us);
            self.push_window(w);
        }
    }

    /// Attribute radio metadata to a frame received at `received_at_us`,
    /// `ax25_len` bytes long. `bit_rate_hz` (the modem's current over-air rate)
    /// enables the airtime + pre-data-carrier fields; pass `None` when unknown.
    ///
    /// Mirrors `RssiTaggingTransport.Attribute`: with carrier-sense, the frame is
    /// attributed to the transmission window containing its arrival (median/min/max
    /// RSSI over the in-window samples, SNR vs the noise floor, burst index, and —
    /// for the first frame of a burst — the pre-data carrier time); without it (or
    /// when no window matches), a threshold-over-noise-floor fallback is used.
    /// Returns `None` when nothing qualifies and no airtime is known.
    pub fn attribute(
        &mut self,
        received_at_us: u64,
        ax25_len: usize,
        bit_rate_hz: Option<u32>,
    ) -> Option<RadioMetadata> {
        let airtime_us = bit_rate_hz.and_then(|rate| airtime_us(ax25_len, rate));
        let floor = self.noise_floor_tenths;

        if self.carrier_sense {
            if let Some(chosen) = self.find_window(received_at_us) {
                let (rise, fall, burst_index) = self.claim_window(chosen);
                let to = match fall {
                    Some(f) if f < received_at_us => f,
                    _ => received_at_us,
                };
                let stats = self.stats_between(rise, to);
                let pre_data = if burst_index == 0 {
                    airtime_us.map(|air| {
                        received_at_us as i64 - air as i64 - rise as i64
                    })
                } else {
                    None
                };
                return Some(RadioMetadata {
                    rssi_dbm_tenths: stats.map(|s| s.median),
                    snr_db_tenths: match (stats, floor) {
                        (Some(s), Some(f)) => Some(s.median - f),
                        _ => None,
                    },
                    noise_floor_dbm_tenths: floor,
                    rssi_min_dbm_tenths: stats.map(|s| s.min),
                    rssi_max_dbm_tenths: stats.map(|s| s.max),
                    rssi_sample_count: stats.map(|s| s.count).unwrap_or(0),
                    carrier_rise_at_us: Some(rise),
                    burst_index: Some(burst_index),
                    estimated_airtime_us: airtime_us,
                    pre_data_carrier_us: pre_data,
                });
            }
        }

        self.threshold_attribution(received_at_us, floor, airtime_us)
    }

    /// The no-carrier-sense fallback: samples above the noise floor within the
    /// lookback are taken as signal. Mirrors `BuildThresholdAttribution`.
    fn threshold_attribution(
        &self,
        received_at_us: u64,
        floor: Option<i16>,
        airtime_us: Option<u64>,
    ) -> Option<RadioMetadata> {
        let from = received_at_us.saturating_sub(self.config.attribution_lookback_us);
        let mut scratch = [0i16; SAMPLE_CAPACITY];
        let mut n = 0;
        for i in 0..self.sample_len {
            let s = self.sample_at(i);
            if s.at_us < from || s.at_us > received_at_us {
                continue;
            }
            let is_signal = match floor {
                None => true,
                Some(f) => s.dbm_tenths >= f.saturating_add(self.config.signal_threshold_tenths),
            };
            if is_signal {
                scratch[n] = s.dbm_tenths;
                n += 1;
            }
        }

        if n == 0 {
            return airtime_us.map(|air| RadioMetadata {
                estimated_airtime_us: Some(air),
                ..RadioMetadata::default()
            });
        }

        let signal = &mut scratch[..n];
        signal.sort_unstable();
        let rssi = signal[n / 2];
        Some(RadioMetadata {
            rssi_dbm_tenths: Some(rssi),
            snr_db_tenths: floor.map(|f| rssi - f),
            noise_floor_dbm_tenths: floor,
            rssi_min_dbm_tenths: Some(signal[0]),
            rssi_max_dbm_tenths: Some(signal[n - 1]),
            rssi_sample_count: n as u16,
            estimated_airtime_us: airtime_us,
            ..RadioMetadata::default()
        })
    }

    /// The frame belongs to the window containing its arrival; delivery trails
    /// carrier-fall by the modem latency, so closed windows stay eligible for a
    /// slack. Mirrors `FindWindow` (open window preferred; else the *last* closed
    /// window that contains the arrival).
    fn find_window(&self, received_at_us: u64) -> Option<WindowRef> {
        if let Some(open) = &self.current_window {
            if open.rise_at_us <= received_at_us {
                return Some(WindowRef::Current);
            }
        }
        let mut best = None;
        for i in 0..self.window_len {
            let w = self.window_at(i);
            if let Some(fall) = w.fall_at_us {
                if w.rise_at_us <= received_at_us
                    && received_at_us <= fall + self.config.window_attribution_slack_us
                {
                    best = Some(i);
                }
            }
        }
        best.map(WindowRef::Closed)
    }

    /// Take the chosen window's rise/fall and burst index, incrementing its delivered
    /// counter (the C# `window.FramesDelivered++`).
    fn claim_window(&mut self, chosen: WindowRef) -> (u64, Option<u64>, u16) {
        match chosen {
            WindowRef::Current => {
                let w = self.current_window.as_mut().expect("current window present");
                let bi = w.frames_delivered;
                w.frames_delivered += 1;
                (w.rise_at_us, w.fall_at_us, bi)
            }
            WindowRef::Closed(i) => {
                let idx = (self.window_head + i) % WINDOW_CAPACITY;
                let w = &mut self.windows[idx];
                let bi = w.frames_delivered;
                w.frames_delivered += 1;
                (w.rise_at_us, w.fall_at_us, bi)
            }
        }
    }

    /// Median/min/max/count over the samples in `[from, to]` (inclusive). Mirrors
    /// `RssiStatistics`; `None` when the window holds no samples.
    fn stats_between(&self, from_us: u64, to_us: u64) -> Option<WindowStats> {
        let mut scratch = [0i16; SAMPLE_CAPACITY];
        let mut n = 0;
        for i in 0..self.sample_len {
            let s = self.sample_at(i);
            if s.at_us >= from_us && s.at_us <= to_us {
                scratch[n] = s.dbm_tenths;
                n += 1;
            }
        }
        if n == 0 {
            return None;
        }
        let slice = &mut scratch[..n];
        slice.sort_unstable();
        Some(WindowStats {
            median: slice[n / 2],
            min: slice[0],
            max: slice[n - 1],
            count: n as u16,
        })
    }

    // ── fixed-capacity ring helpers ──────────────────────────────────────────

    fn push_sample(&mut self, s: Sample) {
        if self.sample_len == SAMPLE_CAPACITY {
            self.sample_head = (self.sample_head + 1) % SAMPLE_CAPACITY;
            self.sample_len -= 1;
        }
        let idx = (self.sample_head + self.sample_len) % SAMPLE_CAPACITY;
        self.samples[idx] = s;
        self.sample_len += 1;
    }

    fn sample_at(&self, i: usize) -> Sample {
        self.samples[(self.sample_head + i) % SAMPLE_CAPACITY]
    }

    fn push_window(&mut self, w: CarrierWindow) {
        if self.window_len == WINDOW_CAPACITY {
            self.window_head = (self.window_head + 1) % WINDOW_CAPACITY;
            self.window_len -= 1;
        }
        let idx = (self.window_head + self.window_len) % WINDOW_CAPACITY;
        self.windows[idx] = w;
        self.window_len += 1;
    }

    fn window_at(&self, i: usize) -> CarrierWindow {
        self.windows[(self.window_head + i) % WINDOW_CAPACITY]
    }
}

#[derive(Debug, Clone, Copy)]
struct WindowStats {
    median: i16,
    min: i16,
    max: i16,
    count: u16,
}

/// Approximate on-air duration, micros: wire bytes (+FCS +flag) × 8 ÷ bit rate,
/// ignoring bit-stuffing. Mirrors the C# `(ax25Length + 3) * 8.0 / rate` seconds,
/// in integer micros (the `catalog::transmission_us` idiom). `None` for a zero rate.
pub fn airtime_us(ax25_len: usize, bit_rate_hz: u32) -> Option<u64> {
    if bit_rate_hz == 0 {
        return None;
    }
    Some((ax25_len as u64 + 3) * 8 * 1_000_000 / bit_rate_hz as u64)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn airtime_matches_integer_formula() {
        // 27 payload bytes + FCS(2) + flag(1) = 30 bytes × 8 ÷ 9600 = 25 ms.
        assert_eq!(airtime_us(27, 9600), Some(25_000));
        assert_eq!(airtime_us(10, 0), None);
    }

    #[test]
    fn noise_floor_seeds_then_emas_with_integer_coefficient() {
        let mut t = RssiTagger::new(RssiTaggingConfig::default(), true);
        // First idle sample seeds the floor.
        t.record_sample(100_000, -1000, Some(false));
        assert_eq!(t.noise_floor_tenths(), Some(-1000));
        // Second idle sample: nf += (−900 − (−1000)) × 205/1024 = +20 → −980.
        t.record_sample(200_000, -900, Some(false));
        assert_eq!(t.noise_floor_tenths(), Some(-980));
        // A busy sample does not move the floor.
        t.record_sample(300_000, -400, Some(true));
        assert_eq!(t.noise_floor_tenths(), Some(-980));
    }

    #[test]
    fn window_attribution_gives_median_snr_and_burst_index() {
        let mut t = RssiTagger::new(RssiTaggingConfig::default(), true);
        // Seed the noise floor from a pre-window idle sample.
        t.record_sample(100_000, -1000, Some(false));
        // Carrier up; three in-window samples; carrier down.
        t.carrier_edge(true, 1_000_000);
        t.record_sample(1_020_000, -500, Some(true));
        t.record_sample(1_050_000, -480, Some(true));
        t.record_sample(1_080_000, -520, Some(true));
        t.carrier_edge(false, 1_100_000);

        // First frame of the burst, delivered just after carrier-fall.
        let meta = t
            .attribute(1_130_000, 27, Some(9600))
            .expect("attributed");
        assert_eq!(meta.rssi_dbm_tenths, Some(-500)); // median of −520,−500,−480
        assert_eq!(meta.rssi_min_dbm_tenths, Some(-520));
        assert_eq!(meta.rssi_max_dbm_tenths, Some(-480));
        assert_eq!(meta.rssi_sample_count, 3);
        assert_eq!(meta.noise_floor_dbm_tenths, Some(-1000));
        assert_eq!(meta.snr_db_tenths, Some(500)); // −500 − (−1000)
        assert_eq!(meta.carrier_rise_at_us, Some(1_000_000));
        assert_eq!(meta.burst_index, Some(0));
        assert_eq!(meta.estimated_airtime_us, Some(25_000));
        // pre-data = received − airtime − rise = 1_130_000 − 25_000 − 1_000_000.
        assert_eq!(meta.pre_data_carrier_us, Some(105_000));

        // Second frame in the same (closed) window: burst index 1, no pre-data.
        let meta2 = t
            .attribute(1_140_000, 27, Some(9600))
            .expect("attributed");
        assert_eq!(meta2.burst_index, Some(1));
        assert_eq!(meta2.pre_data_carrier_us, None);
    }

    #[test]
    fn open_window_is_preferred_before_carrier_fall() {
        let mut t = RssiTagger::new(RssiTaggingConfig::default(), true);
        t.carrier_edge(true, 2_000_000);
        t.record_sample(2_010_000, -600, Some(true));
        // Frame delivered while carrier still up (window open, fall = None → to =
        // received).
        let meta = t.attribute(2_020_000, 20, None).expect("attributed");
        assert_eq!(meta.rssi_dbm_tenths, Some(-600));
        assert_eq!(meta.carrier_rise_at_us, Some(2_000_000));
        assert_eq!(meta.burst_index, Some(0));
        assert_eq!(meta.estimated_airtime_us, None); // no bit rate supplied
    }

    #[test]
    fn threshold_fallback_without_carrier_sense() {
        let mut t = RssiTagger::new(RssiTaggingConfig::default(), false);
        // Establish a floor from quiet samples (threshold-detected as idle).
        t.record_sample(1_000, -1000, None);
        t.record_sample(2_000, -1000, None);
        // A strong sample (>= floor + 6 dB) inside the lookback counts as signal.
        t.record_sample(3_000, -300, None);
        t.record_sample(4_000, -320, None);
        let meta = t.attribute(5_000, 15, None).expect("attributed");
        // Median of the two signal samples (index len/2 = 1): −300.
        assert_eq!(meta.rssi_dbm_tenths, Some(-300));
        assert_eq!(meta.rssi_min_dbm_tenths, Some(-320));
        assert_eq!(meta.rssi_max_dbm_tenths, Some(-300));
        assert_eq!(meta.rssi_sample_count, 2);
        assert_eq!(meta.burst_index, None); // no carrier-sense → no window fields
        assert_eq!(meta.carrier_rise_at_us, None);
    }

    #[test]
    fn no_qualifying_sample_yields_airtime_only_or_none() {
        let mut t = RssiTagger::new(RssiTaggingConfig::default(), true);
        // No samples, no window: nothing to attribute. With a bit rate, airtime only.
        let meta = t.attribute(10_000, 27, Some(9600)).expect("airtime-only");
        assert_eq!(meta.estimated_airtime_us, Some(25_000));
        assert_eq!(meta.rssi_dbm_tenths, None);
        assert_eq!(meta.burst_index, None);
        // Without a bit rate either, there is nothing at all.
        assert_eq!(t.attribute(10_000, 27, None), None);
    }

    #[test]
    fn closed_windows_are_capped_at_capacity() {
        let mut t = RssiTagger::new(RssiTaggingConfig::default(), true);
        // Open+close more windows than the ring holds; the oldest are dropped.
        for k in 0..(WINDOW_CAPACITY as u64 + 3) {
            let base = 1_000_000 + k * 10_000;
            t.carrier_edge(true, base);
            t.carrier_edge(false, base + 1_000);
        }
        assert_eq!(t.window_len, WINDOW_CAPACITY);
        // A frame in the very first (now-evicted) window no longer attributes to a
        // window; it falls through to the (sample-less) threshold path → None.
        assert_eq!(t.attribute(1_000_500, 20, None), None);
    }

    #[test]
    fn old_samples_are_pruned_beyond_the_horizon() {
        let mut t = RssiTagger::new(RssiTaggingConfig::default(), false);
        t.record_sample(0, -500, None);
        // A sample far in the future prunes the ancient one (horizon = now − 10 s).
        t.record_sample(20_000_000, -400, None);
        assert_eq!(t.sample_len, 1);
    }
}
