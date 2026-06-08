//! The INP3 **SNTT** (Smoothed Neighbour Transport Time) integer IIR smoother —
//! the link-timing metric the route layer sums. It is an integer EWMA over
//! `RTT/2` raw samples, in milliseconds, with the same round-to-nearest integer
//! discipline as [`super::quality::combine`] (no floating point anywhere, so the
//! pico-node M0+ has no FPU dependency and the three stacks agree bit-for-bit).
//!
//! The locked default filter is a **1/8-gain IIR** (the AX.25 SRT smoothed
//! round-trip-time convention, a shift-by-3):
//!
//! ```text
//!   SNTT' = (7 × SNTT + sample + 4) / 8        (integer division)
//! ```
//!
//! generalised to the configurable shift form so the gain stays a single integer
//! knob and the divide is a shift:
//!
//! ```text
//!   denom = 1 << gain_shift                       // gain_shift = 3 ⇒ denom 8 ⇒ gain 1/8
//!   SNTT' = ((denom - 1) × SNTT + sample + (denom >> 1)) >> gain_shift
//! ```
//!
//! The `+ (denom >> 1)` is round-to-nearest on the divide (exactly the `+ 128` in
//! `(a × b + 128) / 256`). This is a one-pole low-pass with gain `g = 1 / denom`:
//! `SNTT' = SNTT + (sample − SNTT) / denom`, rewritten to keep all intermediates
//! non-negative for the integer divide.
//!
//! **First-sample seeding (LOCKED).** A fresh neighbour has no history; seeding
//! SNTT = 0 would make the filter crawl up from zero and badly under-report the
//! link at first. The first valid sample therefore seeds the filter directly
//! (`SNTT := sample`, no smoothing on sample #1); every subsequent sample applies
//! the IIR. This is the canonical SRT/Karn seeding. The [`SNTT_UNSET`] sentinel
//! (`u32::MAX`) means "no measurement yet," distinct from a real `0 ms` (a
//! same-host loopback could legitimately measure ~0).
//!
//! **Overflow / range (LOCKED).** Samples are clamped to `[0, SNTT_SAMPLE_MAX_MS]`
//! (the INP3 600 s horizon — a transport time at/over the horizon is
//! "unreachable," and the 180 s link reset tears the link down long before a real
//! RTT reaches 600 s anyway). With both inputs ≤ 600 000, the worst-case
//! accumulator at the largest denom (256) is `255 × 600 000 + 600 000 + 128 =
//! 153 600 128` — far under `u32::MAX` (4.29e9), so a 32-bit intermediate is safe
//! with > 27× headroom. (The C# uses a signed `int` accumulator; every
//! intermediate here is non-negative, so an unsigned `u32` is the faithful and
//! tighter Rust mapping.) The IIR is a convex combination of two values each in
//! `[0, 600 000]`, so the result stays in `[0, 600 000]`.
//!
//! **Gain is interop-tuning, NOT wire-compat.** Two nodes never exchange their
//! smoothing gain — only the resulting SNTT-derived target times in RIPs, and even
//! those are advisory. The gain only affects how twitchy vs. sluggish our own link
//! metric is. The default 1/8 is exposed as [`SNTT_DEFAULT_GAIN_SHIFT`] and is
//! configurable per-call; cross-stack parity is "identical given identical config,"
//! so all three stacks must use the same configured value.
//!
//! This is a pure `Copy` value type — it carries only the smoothed value and the
//! seeded flag, holds no clock, and performs no I/O. The host-free `Inp3Engine`
//! owns the RTT measurement loop and feeds `RTT/2` samples here.
//!
//! Mirrors `Packet.NetRom.Routing.Inp3Sntt` on the C# side (design
//! `netrom-inp3-i2-design.md` §0; AMBIGUITY-I2-1), and the merged TS port
//! `ax25-ts/src/netrom/inp3-sntt.ts`.

/// The default SNTT IIR gain as a right-shift: `gain = 1 / (1 << shift)`. Default
/// `3` ⇒ gain `1/8` (the AX.25 SRT convention). Interop-tuning, not wire-compat
/// (design AMBIGUITY-I2-1). Mirrors `Inp3Sntt.DefaultGainShift`.
pub const SNTT_DEFAULT_GAIN_SHIFT: u32 = 3;

/// The minimum valid gain shift (gain `1/2`). A shift of `0` would be gain `1` =
/// no smoothing (pointless), so it is rejected. Mirrors `Inp3Sntt.MinGainShift`.
pub const SNTT_MIN_GAIN_SHIFT: u32 = 1;

/// The maximum valid gain shift (gain `1/256`). Past this the filter is sluggish
/// beyond usefulness. Mirrors `Inp3Sntt.MaxGainShift`.
pub const SNTT_MAX_GAIN_SHIFT: u32 = 8;

/// The upper clamp on a raw sample, in milliseconds — the INP3 600 s "unreachable"
/// horizon (i1-wire-spec §2.4). A sample at/over this is clamped; the smoothed
/// result therefore also stays within `[0, SNTT_SAMPLE_MAX_MS]`. Mirrors
/// `Inp3Sntt.SampleMaxMs`.
pub const SNTT_SAMPLE_MAX_MS: u32 = 600_000;

/// The "no measurement yet" sentinel for [`Inp3Sntt::ms`] — distinct from a real
/// `0 ms`. [`Inp3Sntt::initialised`] is the canonical test; this value is exposed
/// so callers that store the raw `u32` (the per-neighbour state field) can
/// recognise the un-seeded state. Equals C# `uint.MaxValue` (`0xFFFF_FFFF`).
/// Mirrors `Inp3Sntt.Unset`.
pub const SNTT_UNSET: u32 = u32::MAX;

/// Raw-`u32` alias for [`SNTT_UNSET`], for callers (e.g. the per-neighbour state
/// in `Inp3Engine`) that store the smoothed value as a bare `u32` rather than an
/// [`Inp3Sntt`]. Identical to [`SNTT_UNSET`]. Mirrors `Inp3Sntt.SnttUnset`.
pub const SNTT_UNSET_RAW: u32 = SNTT_UNSET;

/// Clamp a raw sample into `[0, SNTT_SAMPLE_MAX_MS]` (the horizon). The lower bound
/// is free: the input is unsigned.
#[inline]
const fn clamp_sample(sample_ms: u32) -> u32 {
    if sample_ms > SNTT_SAMPLE_MAX_MS {
        SNTT_SAMPLE_MAX_MS
    } else {
        sample_ms
    }
}

/// `true` if `gain_shift` is in `[SNTT_MIN_GAIN_SHIFT, SNTT_MAX_GAIN_SHIFT]`.
#[inline]
const fn gain_shift_in_range(gain_shift: u32) -> bool {
    gain_shift >= SNTT_MIN_GAIN_SHIFT && gain_shift <= SNTT_MAX_GAIN_SHIFT
}

/// Fold a `RTT/2` sample into a raw-`u32` smoothed value — the per-neighbour-state
/// form of [`Inp3Sntt::update`]. `current_ms` is [`SNTT_UNSET_RAW`] for an
/// un-seeded neighbour (the sample then seeds directly) or a prior smoothed value
/// (the IIR applies). Returns the new raw smoothed value (never [`SNTT_UNSET_RAW`]).
///
/// The pure-function bridge the engine uses; mirrors the static
/// `Inp3Sntt.Smooth(currentMs, sampleMs, gainShift)` and TS `smoothSntt`.
///
/// # Panics
/// Panics if `gain_shift` is outside `[SNTT_MIN_GAIN_SHIFT, SNTT_MAX_GAIN_SHIFT]`
/// (the C# `ArgumentOutOfRangeException` analogue — `gain_shift` is a config
/// constant, not wire input, so a bad value is a programmer error, not a parse
/// failure).
#[inline]
pub fn smooth(current_ms: u32, sample_ms: u32, gain_shift: u32) -> u32 {
    let state = if current_ms == SNTT_UNSET {
        Inp3Sntt::fresh()
    } else {
        Inp3Sntt::from_raw(current_ms)
    };
    state.update(sample_ms, gain_shift).ms()
}

/// The INP3 SNTT integer IIR smoother — a pure `Copy` value type carrying the
/// smoothed neighbour transport time (ms) and the seeded flag. [`Inp3Sntt::update`]
/// and [`Inp3Sntt::seed`] return new instances, never mutating the source (it is
/// `Copy`). Mirrors the readonly C# struct `Inp3Sntt`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Inp3Sntt {
    ms: u32,
}

impl Inp3Sntt {
    /// A fresh, un-seeded smoother — no measurement yet. The first
    /// [`Inp3Sntt::update`] seeds it directly from the sample. Mirrors
    /// `Inp3Sntt.Fresh`.
    #[inline]
    pub const fn fresh() -> Self {
        Self { ms: SNTT_UNSET }
    }

    /// Seed a smoother directly from a first sample (no smoothing), e.g. when
    /// reconstructing state. Equivalent to `Inp3Sntt::fresh().update(sample_ms, …)`.
    /// The sample is clamped to `[0, SNTT_SAMPLE_MAX_MS]`. Mirrors `Inp3Sntt.Seed`.
    #[inline]
    pub const fn seed(sample_ms: u32) -> Self {
        Self {
            ms: clamp_sample(sample_ms),
        }
    }

    /// Reconstruct a smoother from a raw smoothed value (a per-neighbour state
    /// field). Pass [`SNTT_UNSET`] for an un-seeded smoother. Internal-ish — the
    /// [`smooth`] bridge uses it; callers normally prefer [`Inp3Sntt::seed`] /
    /// [`Inp3Sntt::fresh`]. Mirrors TS `Inp3Sntt.fromRaw`.
    #[inline]
    pub const fn from_raw(ms: u32) -> Self {
        Self { ms }
    }

    /// `true` once at least one sample has been folded in. While false, [`ms`] is
    /// [`SNTT_UNSET`] and the route layer must treat the neighbour as contributing
    /// no time-route. Mirrors `Inp3Sntt.Initialised`.
    ///
    /// [`ms`]: Inp3Sntt::ms
    #[inline]
    pub const fn initialised(&self) -> bool {
        self.ms != SNTT_UNSET
    }

    /// The smoothed neighbour transport time in milliseconds, or [`SNTT_UNSET`]
    /// (`0xFFFF_FFFF`) if no sample has been folded in yet. Always in
    /// `[0, SNTT_SAMPLE_MAX_MS]` once [`initialised`]. Mirrors `Inp3Sntt.Ms`.
    ///
    /// [`initialised`]: Inp3Sntt::initialised
    #[inline]
    pub const fn ms(&self) -> u32 {
        self.ms
    }

    /// The smoothed value as an [`Option`], for the route layer: the millisecond
    /// value once [`initialised`], else `None`. Mirrors `Inp3Sntt.Value`.
    ///
    /// [`initialised`]: Inp3Sntt::initialised
    #[inline]
    pub const fn value(&self) -> Option<u32> {
        if self.initialised() {
            Some(self.ms)
        } else {
            None
        }
    }

    /// Fold a new `RTT/2` sample into the smoother using the [default gain]
    /// (`1/8`), returning the new smoothed value. Convenience for
    /// `update(sample_ms, SNTT_DEFAULT_GAIN_SHIFT)`. Mirrors `Inp3Sntt.Update(uint)`.
    ///
    /// [default gain]: SNTT_DEFAULT_GAIN_SHIFT
    #[inline]
    pub fn update_default(&self, sample_ms: u32) -> Self {
        self.update(sample_ms, SNTT_DEFAULT_GAIN_SHIFT)
    }

    /// Fold a new `RTT/2` sample into the smoother using the given gain shift
    /// (`gain = 1 / (1 << gain_shift)`), returning the new smoothed value. The
    /// first sample seeds directly; every subsequent sample applies the integer IIR
    /// `((denom-1)·SNTT + sample + denom/2) >> gain_shift`. The sample is clamped to
    /// `[0, SNTT_SAMPLE_MAX_MS]` before smoothing. Mirrors `Inp3Sntt.Update`.
    ///
    /// # Panics
    /// Panics if `gain_shift` is outside `[SNTT_MIN_GAIN_SHIFT,
    /// SNTT_MAX_GAIN_SHIFT]` (the C# `ArgumentOutOfRangeException` analogue —
    /// `gain_shift` is interop-tuning config, not wire input, so a bad value is a
    /// programmer error and panicking is the correct contract here).
    #[inline]
    pub fn update(&self, sample_ms: u32, gain_shift: u32) -> Self {
        assert!(
            gain_shift_in_range(gain_shift),
            "SNTT gain shift must be in [{SNTT_MIN_GAIN_SHIFT}, {SNTT_MAX_GAIN_SHIFT}] (gain 1/2 .. 1/256)"
        );

        let sample = clamp_sample(sample_ms);

        // First valid sample seeds the filter directly (canonical SRT/Karn seeding);
        // smoothing begins at the second sample.
        if !self.initialised() {
            return Self { ms: sample };
        }

        // Integer IIR, round-to-nearest:
        //   denom = 1 << gain_shift
        //   SNTT' = ((denom - 1) * SNTT + sample + denom/2) >> gain_shift
        //
        // Accumulator headroom: with SNTT ≤ 600_000 and sample ≤ 600_000 and the
        // largest denom (256), the worst case is 255*600_000 + 600_000 + 128 =
        // 153_600_128 — far under u32::MAX (4.29e9). Every term is non-negative, so
        // an unsigned accumulator is the faithful map of the C# signed-int math.
        let denom: u32 = 1 << gain_shift;
        let accumulator = (denom - 1) * self.ms + sample + (denom >> 1);
        let smoothed = accumulator >> gain_shift;

        // The IIR is a convex combination of two values each in [0, SNTT_SAMPLE_MAX_MS],
        // so the result is already in range; assert as a cheap invariant.
        debug_assert!(
            smoothed <= SNTT_SAMPLE_MAX_MS,
            "SNTT IIR result escaped [0, SNTT_SAMPLE_MAX_MS]"
        );

        Self { ms: smoothed }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── first-sample seeding (§0.2) ───────────────────────────────────────

    #[test]
    fn fresh_is_uninitialised_and_reads_unset() {
        let s = Inp3Sntt::fresh();
        assert!(!s.initialised());
        assert_eq!(s.ms(), SNTT_UNSET);
        assert_eq!(s.value(), None);
    }

    #[test]
    fn first_sample_seeds_the_filter_directly_with_no_smoothing() {
        // The first valid sample seeds SNTT := sample (canonical SRT/Karn). Were it
        // smoothed against a 0 seed it would read ~25 ((7*0+200+4)/8), not 200.
        let s = Inp3Sntt::fresh().update_default(200);
        assert!(s.initialised());
        assert_eq!(s.ms(), 200);
        assert_eq!(s.value(), Some(200));
    }

    #[test]
    fn first_sample_of_zero_seeds_a_real_zero_distinct_from_unset() {
        // A same-host loopback can legitimately measure ~0; the Unset sentinel must
        // be distinct from a genuine 0 ms measurement.
        let s = Inp3Sntt::fresh().update_default(0);
        assert!(s.initialised());
        assert_eq!(s.ms(), 0);
        assert_eq!(s.value(), Some(0));
        assert_ne!(s.ms(), SNTT_UNSET);
    }

    #[test]
    fn seed_factory_is_equivalent_to_fresh_then_update() {
        assert_eq!(Inp3Sntt::seed(200), Inp3Sntt::fresh().update_default(200));
        assert_eq!(Inp3Sntt::seed(0), Inp3Sntt::fresh().update_default(0));
    }

    // ── worked convergence example A — steady link (§0.5) ─────────────────

    #[test]
    fn example_a_steady_link_sits_exactly_on_its_fixed_point() {
        // RTT steady at 400 ms ⇒ sample 200. Seed = 200; every subsequent
        // (7·200 + 200 + 4)/8 = 1604/8 = 200 — the +4 round-to-nearest keeps a
        // steady input pinned on its fixed point, no drift.
        let mut s = Inp3Sntt::seed(200);
        assert_eq!(s.ms(), 200);
        for _ in 0..100 {
            s = s.update_default(200);
            assert_eq!(s.ms(), 200, "a steady sample reproduces itself exactly");
        }
    }

    #[test]
    fn example_b_step_up_then_settle_matches_the_design_trajectory() {
        // A link that got slower: RTT jumps 100 → 1000 ms (sample 50 → 500).
        // Design §0.5 table B: seed 50, then 50, 106, 155, 198, 236.
        let mut s = Inp3Sntt::seed(50); // step 1 (seed)
        assert_eq!(s.ms(), 50);

        s = s.update_default(50); // step 2: (7·50+50+4)/8 = 404/8 = 50
        assert_eq!(s.ms(), 50);

        s = s.update_default(500); // step 3: (7·50+500+4)/8 = 854/8 = 106
        assert_eq!(s.ms(), 106);

        s = s.update_default(500); // step 4: (7·106+500+4)/8 = 1246/8 = 155
        assert_eq!(s.ms(), 155);

        s = s.update_default(500); // step 5: (7·155+500+4)/8 = 1589/8 = 198
        assert_eq!(s.ms(), 198);

        s = s.update_default(500); // step 6: (7·198+500+4)/8 = 1890/8 = 236
        assert_eq!(s.ms(), 236);
    }

    #[test]
    fn example_b_converges_toward_the_new_sample_over_many_probes() {
        // A sustained slowdown is fully reflected within a few probe intervals: a
        // 1/8-gain EWMA reaches ~95% of a step in ~24 samples. With integer rounding
        // the fixed-point region for sample 500 is [497, 500]; assert it converges to
        // within 3 ms of the new sample and never overshoots it.
        let mut s = Inp3Sntt::seed(50).update_default(50); // steady at 50
        for _ in 0..60 {
            s = s.update_default(500);
            assert!(
                s.ms() <= 500,
                "the filter approaches a step from below, never overshooting"
            );
        }
        assert!(
            (497..=500).contains(&s.ms()),
            "a 1/8 IIR settles onto the integer fixed-point band of its input"
        );
    }

    // ── worked convergence example C — outlier rejection (§0.5) ───────────

    #[test]
    fn example_c_single_outlier_is_damped_then_walked_back() {
        // Steady 200 ms RTT ⇒ sample 100, with one 2000 ms spike ⇒ sample 1000.
        // Design §0.5 table C: seed 100, then 100, 213, 199, 187, 176, 167.
        let mut s = Inp3Sntt::seed(100); // step 1 (seed)
        assert_eq!(s.ms(), 100);

        s = s.update_default(100); // step 2: (7·100+100+4)/8 = 804/8 = 100
        assert_eq!(s.ms(), 100);

        // step 3 (spike): (7·100+1000+4)/8 = 1704/8 = 213 — a lone 10× spike moves
        // SNTT by only +113, not to 1000 (the outlier rejection the smoother exists for)
        s = s.update_default(1000);
        assert_eq!(s.ms(), 213);

        s = s.update_default(100); // step 4: (7·213+100+4)/8 = 1595/8 = 199
        assert_eq!(s.ms(), 199);

        s = s.update_default(100); // step 5: (7·199+100+4)/8 = 1497/8 = 187
        assert_eq!(s.ms(), 187);

        s = s.update_default(100); // step 6: (7·187+100+4)/8 = 1413/8 = 176
        assert_eq!(s.ms(), 176);

        s = s.update_default(100); // step 7: (7·176+100+4)/8 = 1336/8 = 167
        assert_eq!(s.ms(), 167);
    }

    #[test]
    fn example_c_walks_back_into_the_band_of_the_true_value_after_a_spike() {
        // After the spike, the filter walks back to the true 100 within a handful of
        // probes. It rests in the integer rounding band [100, 104] rather than exactly
        // 100: descending from the spike it settles on the upper fixed point (104),
        // because the round-to-nearest +denom/2 term gives the integer IIR a small DC
        // bias (the same artifact AX.25 SRT carries). The point is the 10x outlier
        // leaves only a few ms of residue, not 100+.
        let mut s = Inp3Sntt::seed(100).update_default(100).update_default(1000); // post-spike = 213
        for _ in 0..100 {
            s = s.update_default(100);
        }
        assert!(
            (100..=104).contains(&s.ms()),
            "the outlier residue decays into the rounding band of the true input"
        );
    }

    // ── monotonic-toward-sample (§0.1 one-pole low-pass) ──────────────────

    #[test]
    fn update_moves_strictly_toward_the_sample_when_above_current() {
        // A one-pole low-pass: SNTT' = SNTT + (sample - SNTT)/8. With sample > SNTT
        // the result is strictly between the old value and the sample (it moves
        // toward the sample, never past it).
        let mut s = Inp3Sntt::seed(100);
        for _ in 0..30 {
            let before = s.ms();
            s = s.update_default(1000);
            assert!(s.ms() > before, "SNTT rises toward a larger sample");
            assert!(s.ms() <= 1000, "but never past the sample");
        }
    }

    #[test]
    fn update_moves_strictly_toward_the_sample_when_below_current() {
        let mut s = Inp3Sntt::seed(1000);
        for _ in 0..30 {
            let before = s.ms();
            s = s.update_default(0);
            assert!(s.ms() < before, "SNTT falls monotonically toward a smaller sample");
            // floors at the sample band — never negative (it is unsigned).
        }
        // Run to the fixed point: a steady 0 sample settles in the integer rounding
        // band [0, denom/2] (~4 at the default 1/8 gain), not exactly 0 — the same
        // round-to-nearest +denom/2 DC bias as Example C. The invariant is convergence
        // into that band, not exact 0.
        for _ in 0..100 {
            s = s.update_default(0);
        }
        assert!(
            (0..=4).contains(&s.ms()),
            "a steady 0 sample converges into the rounding band, not exactly 0"
        );
    }

    #[test]
    fn a_smoothed_value_always_lies_between_its_previous_value_and_the_sample() {
        // (seed, sample): rise from a low seed; fall from a high seed; example-B
        // shape; example-C walk-back shape.
        for &(seed, sample) in &[(0u32, 1000u32), (1000, 0), (50, 500), (1000, 100)] {
            let s = Inp3Sntt::seed(seed);
            let before = s.ms();
            let after = s.update_default(sample);
            let lo = before.min(sample);
            let hi = before.max(sample);
            assert!(
                (lo..=hi).contains(&after.ms()),
                "the IIR is a convex combination of the previous value and the sample (seed {seed}, sample {sample})"
            );
        }
    }

    // ── overflow / range bounds (§0.3) ────────────────────────────────────

    #[test]
    fn sample_above_the_horizon_is_clamped_to_sample_max_ms_on_seed() {
        // C# uses uint.MaxValue - 1; the Rust analogue is any value past the horizon.
        assert_eq!(Inp3Sntt::seed(u32::MAX - 1).ms(), SNTT_SAMPLE_MAX_MS);
        assert_eq!(Inp3Sntt::fresh().update_default(700_000).ms(), SNTT_SAMPLE_MAX_MS);
    }

    #[test]
    fn sample_above_the_horizon_is_clamped_before_smoothing() {
        // A wild sample is clamped to 600_000 before the IIR sees it, so it cannot
        // drive SNTT past the horizon.
        let mut s = Inp3Sntt::seed(0);
        s = s.update_default(u32::MAX);
        // (7·0 + 600000 + 4)/8 = 600004/8 = 75000 — the clamped sample, smoothed.
        assert_eq!(s.ms(), 75_000);
    }

    #[test]
    fn smoothed_value_never_exceeds_sample_max_ms_even_under_max_input_storm() {
        // Pin SNTT at the top, then keep slamming max samples at the highest gain
        // (256): the convex-combination result can sit on the top but never above it,
        // and the u32 accumulator (worst case 255·600000 + 600000 + 128 ≈ 1.5e8)
        // never overflows.
        let mut s = Inp3Sntt::seed(SNTT_SAMPLE_MAX_MS);
        for _ in 0..100 {
            s = s.update(u32::MAX, SNTT_MAX_GAIN_SHIFT);
            assert!(s.ms() <= SNTT_SAMPLE_MAX_MS);
        }
        assert_eq!(s.ms(), SNTT_SAMPLE_MAX_MS, "max sample at the top stays at the top");
    }

    #[test]
    fn all_valid_gains_keep_a_max_x_max_update_within_range() {
        // gain 1/2 (denom 2): worst acc = 1·600000 + 600000 + 1
        // gain 1/8 (default): worst acc = 7·600000 + 600000 + 4
        // gain 1/256: worst acc = 255·600000 + 600000 + 128 ≈ 1.5e8
        for &gain_shift in &[1u32, 3, 8] {
            let s = Inp3Sntt::seed(SNTT_SAMPLE_MAX_MS).update(SNTT_SAMPLE_MAX_MS, gain_shift);
            assert!(s.ms() <= SNTT_SAMPLE_MAX_MS);
            assert_eq!(s.ms(), SNTT_SAMPLE_MAX_MS);
        }
    }

    // ── configurable gain (§0.4) ──────────────────────────────────────────

    #[test]
    fn default_update_uses_the_default_gain_shift() {
        assert_eq!(
            Inp3Sntt::seed(50).update_default(500),
            Inp3Sntt::seed(50).update(500, SNTT_DEFAULT_GAIN_SHIFT)
        );
    }

    #[test]
    fn a_smaller_gain_shift_is_twitchier_a_larger_one_is_more_sluggish() {
        // After one step from 100 toward 1000, a higher gain (1/2, shift 1) moves
        // further than the default (1/8, shift 3), which moves further than a low
        // gain (1/256, shift 8). gain = 1/(1<<shift): smaller shift ⇒ larger gain.
        let twitchy = Inp3Sntt::seed(100).update(1000, 1).ms(); // gain 1/2
        let mid = Inp3Sntt::seed(100).update(1000, 3).ms(); // gain 1/8 (default)
        let sluggish = Inp3Sntt::seed(100).update(1000, 8).ms(); // gain 1/256

        assert!(twitchy > mid);
        assert!(mid > sluggish);

        // Exact integer checks of the shift form, sample - seed = 900:
        //   shift 1: (1·100 + 1000 + 1) >> 1 = 1101 >> 1 = 550
        //   shift 3: (7·100 + 1000 + 4) >> 3 = 1704 >> 3 = 213
        //   shift 8: (255·100 + 1000 + 128) >> 8 = 26628 >> 8 = 104
        assert_eq!(twitchy, 550);
        assert_eq!(mid, 213);
        assert_eq!(sluggish, 104);
    }

    #[test]
    #[should_panic(expected = "SNTT gain shift must be in")]
    fn gain_shift_zero_is_rejected() {
        // gain 1 = no smoothing (pointless).
        let _ = Inp3Sntt::seed(100).update(200, 0);
    }

    #[test]
    #[should_panic(expected = "SNTT gain shift must be in")]
    fn gain_shift_nine_is_rejected() {
        // gain 1/512 = sluggish past usefulness.
        let _ = Inp3Sntt::seed(100).update(200, 9);
    }

    #[test]
    #[should_panic(expected = "SNTT gain shift must be in")]
    fn gain_shift_max_u32_is_rejected() {
        // The Rust analogue of the C# int.MaxValue case.
        let _ = Inp3Sntt::seed(100).update(200, u32::MAX);
    }

    #[test]
    fn every_in_range_gain_shift_is_accepted() {
        for &gain_shift in &[SNTT_MIN_GAIN_SHIFT, SNTT_DEFAULT_GAIN_SHIFT, SNTT_MAX_GAIN_SHIFT] {
            // Does not panic.
            let _ = Inp3Sntt::seed(100).update(200, gain_shift);
        }
    }

    #[test]
    fn gain_shift_only_applies_after_seeding_the_first_sample_still_seeds_directly() {
        // The first sample seeds regardless of gain (no smoothing on sample #1).
        assert_eq!(Inp3Sntt::fresh().update(321, SNTT_MAX_GAIN_SHIFT).ms(), 321);
        assert_eq!(Inp3Sntt::fresh().update(321, SNTT_MIN_GAIN_SHIFT).ms(), 321);
    }

    // ── value-type semantics ──────────────────────────────────────────────

    #[test]
    fn update_is_pure_it_does_not_mutate_the_source() {
        let original = Inp3Sntt::seed(100);
        let updated = original.update_default(1000);
        assert_eq!(original.ms(), 100, "the value is Copy; update returns a new value");
        assert_ne!(updated.ms(), 100);
    }

    #[test]
    fn equality_is_by_value() {
        assert_eq!(Inp3Sntt::seed(200), Inp3Sntt::seed(200));
        assert_ne!(Inp3Sntt::seed(200), Inp3Sntt::seed(201));
        assert_eq!(Inp3Sntt::fresh(), Inp3Sntt::fresh());
        assert_ne!(Inp3Sntt::fresh(), Inp3Sntt::seed(0));
    }

    // ── the static smooth() bridge (mirrors C# Inp3Sntt.Smooth / TS smoothSntt) ──

    #[test]
    fn smooth_bridge_seeds_from_unset_then_applies_the_iir() {
        // SNTT_UNSET_RAW ⇒ the sample seeds directly.
        assert_eq!(smooth(SNTT_UNSET_RAW, 200, SNTT_DEFAULT_GAIN_SHIFT), 200);
        // A prior value ⇒ the IIR applies: (7·50 + 500 + 4)/8 = 854/8 = 106.
        assert_eq!(smooth(50, 500, SNTT_DEFAULT_GAIN_SHIFT), 106);
        // Never returns the sentinel.
        assert_ne!(smooth(SNTT_UNSET_RAW, 0, SNTT_DEFAULT_GAIN_SHIFT), SNTT_UNSET_RAW);
    }
}
