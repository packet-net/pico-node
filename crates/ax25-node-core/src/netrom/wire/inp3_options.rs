//! The tunable knobs of the INP3 link-timing overlay — the probe cadence, the
//! reflection-timeout reset window, the SNTT smoother gain, optimistic-probe
//! policy, and the advertised capability text.
//!
//! Ports `Packet.NetRom.Wire.NetRomInp3Options`. Mirrors the other options
//! records ([`super::options::NetRomParseOptions`],
//! [`crate::netrom::routing::options::NetRomRoutingOptions`]): a [`Default`]
//! preset and validated ranges, every divergence a named knob defaulted to an
//! interoperable value (CLAUDE.md "spec-faithful core, pragmatism is a named
//! flag").
//!
//! All durations are plain milliseconds (the established netrom no_std idiom — the
//! C# `TimeSpan` timers become millisecond integers) and are driven by a
//! `now_ms` parameter against a *monotonic* source — no wall-clock anywhere in the
//! INP3 layer, and no stored clock (the engine/scheduler take `now_ms` per call,
//! like [`crate::netrom::transport`]).
//!
//! The SNTT gain ([`NetRomInp3Options::sntt_gain_shift`]) is interop-*tuning*, not
//! wire-compat: two nodes never exchange their smoothing constant, only the
//! resulting (advisory) SNTT-derived target times in RIPs. It does not have to
//! match a peer to interoperate — but cross-stack parity requires all three stacks
//! (C# / TS / Rust) use the same configured value (the "identical given identical
//! config" discipline, like the quality floor).
//!
//! `no_std`, allocation-free: a plain `Copy` record + a [`NetRomInp3Options::DEFAULT`]
//! const, with a [`NetRomInp3Options::validate`] returning a `Result<(), &'static str>`
//! (the no_std-friendly stand-in for the C# `ArgumentOutOfRangeException` /
//! TS `RangeError`).

/// The emit-side capability-text pad width default — mirrors the C#
/// `Inp3L3RttFrame.DefaultCapabilityTextWidth` (8) and the TS
/// `INP3_DEFAULT_CAPABILITY_TEXT_WIDTH`. Width-independent on the wire (the
/// recogniser ignores padding), so this is purely cosmetic.
pub const INP3_DEFAULT_CAPABILITY_TEXT_WIDTH: u32 = 8;

/// The tunable knobs of the INP3 link-timing overlay (mirrors C#
/// `NetRomInp3Options`). A `Copy` record of validated knobs; the canonical preset
/// is [`NetRomInp3Options::DEFAULT`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NetRomInp3Options {
    /// How often to probe each (capable / optimistically-probed) interlink
    /// neighbour with an L3RTT datagram. Plan §8 `l3RttIntervalSeconds` default
    /// **60 s** (60000 ms).
    pub l3_rtt_interval_ms: u32,

    /// Reflection-timeout → reset: how long a neighbour may go without reflecting a
    /// probe before its INP3 state is torn down (and, for an INP3-capable
    /// neighbour, `NeighbourDown` is raised). Plan §8 `l3RttResetSeconds` default
    /// **180 s** (180000 ms; the spec value). Must exceed
    /// [`Self::l3_rtt_interval_ms`] — a reset window shorter than one probe interval
    /// would tear down a live neighbour before it could answer.
    pub l3_rtt_reset_window_ms: u32,

    /// The SNTT IIR gain expressed as a right-shift: `gain = 1 / (1 << sntt_gain_shift)`.
    /// Default **3** ⇒ gain `1/8` (the AX.25 SRT convention; shift-by-3, no
    /// multiply — important on the FPU-less RP2040 M0+). Interop-tuning, **not**
    /// wire-compat (AMBIGUITY-I2-1). Valid range **1..=8** (gain 1/2 .. 1/256): 0
    /// means gain 1 = no smoothing (pointless) and > 8 is sluggish past usefulness.
    pub sntt_gain_shift: u8,

    /// Probe interlink neighbours whose INP3 capability is not yet known, to
    /// bootstrap discovery (we only learn a peer speaks INP3 by receiving its
    /// probe — AMBIGUITY-I2-2, so we must probe first). Default **true**. A
    /// never-capable neighbour that never reflects is dropped from probing silently
    /// after one reset window — it is *never* `MarkNeighbourDown`'d
    /// (AMBIGUITY-I2-3 guard); only an INP3-capable neighbour that goes silent
    /// raises `NeighbourDown`.
    pub probe_unknown_capability: bool,

    /// The IP version to advertise in our probes' `$IX` capability token (e.g.
    /// `Some(4)`), or `None` for none (`$N` only). Plan §8 `advertiseIp`; off unless
    /// we run IP-over-NET/ROM. Must be a single decimal digit 0–9 when set (the C#
    /// `int?` becomes `Option<u8>`).
    pub advertise_ip_accept: Option<u8>,

    /// The emit-side capability-text pad width for probes we build
    /// (AMBIGUITY-L3RTT-3). Default [`INP3_DEFAULT_CAPABILITY_TEXT_WIDTH`]. The
    /// recogniser is width-independent, so this is purely cosmetic on the wire.
    ///
    /// (Modelled as `u32` rather than the C# `int`, which is `>= 0` already — the
    /// "must be non-negative" guard is therefore structural here, but the
    /// [`Self::validate`] range check is retained for cross-stack symmetry; it can
    /// never fire.)
    pub capability_text_width: u32,

    /// Prefer INP3 (measured target-time) routes over NODES quality routes when
    /// selecting the active route for a destination — BPQ's `PREFERINP3ROUTES` knob
    /// (plan §8). Default **false**: even with the INP3 overlay enabled, the
    /// conservative default keeps quality primary, so a node "turns INP3 on"
    /// (ingesting + advertising time-routes) without changing where traffic flows;
    /// flip this once the measured times are trusted. When `true`, a destination
    /// that has *any* INP3 route forwards over its lowest-target-time INP3 route,
    /// falling back to the best-quality route only when no INP3 route exists (the
    /// selection truth table, plan risk #4 / `docs/netrom-inp3-i3-design.md` §3).
    /// When `false` the [`crate::netrom::routing::model::NetRomRoute::inp3`] metric
    /// is ignored by selection entirely (routes are still ingested + visible for
    /// monitoring and re-advertisement). Consumed by the INP3 route selector.
    pub prefer_inp3_routes: bool,

    /// Master switch for the whole INP3 overlay (plan §8 `inp3.enabled`). Default
    /// **false**: the node behaves exactly as it does today — no L3RTT probing, no
    /// RIF ingestion or emission, no INP3 routes — so enabling the feature is a
    /// deliberate opt-in. This is the host-layer gate that sits above the
    /// (always-correct, host-free) engine + selector; when `false` the host simply
    /// never drives them.
    pub enabled: bool,

    /// The INP3 routing horizon in hops (plan §8 `hopLimit`): a RIP whose local hop
    /// count would exceed this is not learned, bounding loop blast-radius. Default
    /// **30**. The host passes this into the routing table's RIF ingestion. Must be
    /// at least 1.
    pub hop_limit: u8,

    /// Periodic full-RIF cadence — the baseline refresh interval (plan §8
    /// `rifIntervalSeconds`, design I-4 §6.2 "a periodic full RIF on the INP3
    /// interval regardless"). Triggered updates fire regardless of this. Default
    /// **300 s** (300000 ms). Consumed by the INP3 update scheduler; separate from
    /// NODESINTERVAL and from the L3RTT cadence. Must be positive.
    pub rif_interval_ms: u32,

    /// Positive-update debounce — how long a NEW / BETTER (positive) route change is
    /// batched before a fan-out, coalescing a burst of positive changes into one RIF
    /// (design I-4 §3.3 rule 2). NEGATIVE changes (loss / worsen-past-threshold)
    /// ignore this and fan out immediately. Default **5 s** (5000 ms). Must be
    /// positive and strictly less than [`Self::rif_interval_ms`] (a debounce >= the
    /// periodic interval is pointless — the periodic emit would always drain the
    /// batch first).
    pub positive_debounce_ms: u32,

    /// The worsen-by amount (ms) at or above which a slowed selected route counts as
    /// NEGATIVE (immediate fan-out) rather than POSITIVE (batched) — design
    /// AMBIGUITY-I4-3. Sub-threshold worsenings are routine SNTT jitter and batched.
    /// A loss / withdrawal is *always* NEGATIVE regardless of this threshold.
    /// Default **1000 ms**. The table / ingestion path applies it when classifying a
    /// change for the update scheduler's `mark_dirty`; it is a knob here so it is
    /// tunable and cross-stack-pinned. Must be non-negative (structural — `u32`).
    pub worsen_threshold_ms: u32,
}

impl NetRomInp3Options {
    /// The canonical / widely-interoperable defaults (mirrors the C#
    /// `NetRomInp3Options.Default` and the TS `INP3_DEFAULTS`): probe every 60 s,
    /// 180 s reset window, SNTT gain 1/8 (shift 3), optimistic probing on, no `$IX`
    /// advertisement, the cosmetic-8 capability width, quality-primary selection,
    /// overlay off, 30-hop horizon, 300 s periodic RIF, 5 s positive debounce,
    /// 1000 ms worsen threshold.
    pub const DEFAULT: Self = Self {
        l3_rtt_interval_ms: 60_000,
        l3_rtt_reset_window_ms: 180_000,
        sntt_gain_shift: 3,
        probe_unknown_capability: true,
        advertise_ip_accept: None,
        capability_text_width: INP3_DEFAULT_CAPABILITY_TEXT_WIDTH,
        prefer_inp3_routes: false,
        enabled: false,
        hop_limit: 30,
        rif_interval_ms: 300_000,
        positive_debounce_ms: 5_000,
        worsen_threshold_ms: 1_000,
    };

    /// Validate the option ranges. Mirrors the C# `NetRomInp3Options.Validate`
    /// guards 1:1 (its `ArgumentOutOfRangeException` and the TS `RangeError` become
    /// an `Err(&'static str)` — the no_std-friendly error). The host's config
    /// validator surfaces an out-of-range value (plan §8).
    ///
    /// The `capability_text_width` and `worsen_threshold_ms` "non-negative" guards
    /// are structurally satisfied by the unsigned types, but are retained (as
    /// always-pass checks via the type) for 1:1 correspondence with the C#/TS
    /// validators.
    pub fn validate(&self) -> Result<(), &'static str> {
        if self.l3_rtt_interval_ms == 0 {
            return Err("L3RTT probe interval must be positive");
        }
        if self.l3_rtt_reset_window_ms <= self.l3_rtt_interval_ms {
            return Err(
                "L3RTT reset window must exceed the probe interval (a shorter window tears down a live neighbour before it can answer)",
            );
        }
        if self.sntt_gain_shift < 1 || self.sntt_gain_shift > 8 {
            return Err("SNTT gain shift must be in [1, 8] (gain 1/2 .. 1/256)");
        }
        if let Some(ip) = self.advertise_ip_accept {
            // The C#/TS guard is `0..=9`; the `Option<u8>` already excludes the
            // negative half, so only the upper bound can fail.
            if ip > 9 {
                return Err("advertised IP-accept version must be a single decimal digit 0\u{2013}9");
            }
        }
        // capability_text_width: the C#/TS "must be non-negative" guard is
        // structurally satisfied by `u32` — kept here as a no-op for parity.
        if self.hop_limit < 1 {
            return Err("INP3 hop limit must be at least 1");
        }
        if self.rif_interval_ms == 0 {
            return Err("periodic RIF interval must be positive");
        }
        if self.positive_debounce_ms == 0 {
            return Err("positive-update debounce must be positive");
        }
        if self.positive_debounce_ms >= self.rif_interval_ms {
            return Err(
                "positive-update debounce must be less than the periodic RIF interval (a debounce >= the interval is pointless — the periodic emit would always drain the batch first)",
            );
        }
        // worsen_threshold_ms: the C#/TS "must be non-negative" guard is
        // structurally satisfied by `u32` — kept here as a no-op for parity.
        Ok(())
    }
}

impl Default for NetRomInp3Options {
    fn default() -> Self {
        Self::DEFAULT
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_matches_the_canonical_preset() {
        // Mirrors the C# `NetRomInp3Options.Default` / TS `INP3_DEFAULTS` field-by-field.
        let d = NetRomInp3Options::DEFAULT;
        assert_eq!(d.l3_rtt_interval_ms, 60_000);
        assert_eq!(d.l3_rtt_reset_window_ms, 180_000);
        assert_eq!(d.sntt_gain_shift, 3);
        assert!(d.probe_unknown_capability);
        assert_eq!(d.advertise_ip_accept, None);
        assert_eq!(d.capability_text_width, 8);
        assert!(!d.prefer_inp3_routes);
        assert!(!d.enabled);
        assert_eq!(d.hop_limit, 30);
        assert_eq!(d.rif_interval_ms, 300_000);
        assert_eq!(d.positive_debounce_ms, 5_000);
        assert_eq!(d.worsen_threshold_ms, 1_000);
    }

    #[test]
    fn default_trait_equals_the_const_default() {
        assert_eq!(NetRomInp3Options::default(), NetRomInp3Options::DEFAULT);
    }

    #[test]
    fn capability_text_width_default_const_is_eight() {
        assert_eq!(INP3_DEFAULT_CAPABILITY_TEXT_WIDTH, 8);
    }

    #[test]
    fn default_validates_clean() {
        // The C# config-validator test: `Inp3 == Default` validates fine.
        assert_eq!(NetRomInp3Options::DEFAULT.validate(), Ok(()));
    }

    #[test]
    fn fully_specified_in_range_validates_clean() {
        // Mirrors the C# validator's "all fields set, in range" valid case
        // (L3RTT 60 s / reset 180 s / RIF 300 s / debounce 5 s / gain 3 / hop 30).
        let o = NetRomInp3Options {
            enabled: true,
            l3_rtt_interval_ms: 60_000,
            l3_rtt_reset_window_ms: 180_000,
            rif_interval_ms: 300_000,
            positive_debounce_ms: 5_000,
            sntt_gain_shift: 3,
            hop_limit: 30,
            ..NetRomInp3Options::DEFAULT
        };
        assert_eq!(o.validate(), Ok(()));
    }

    #[test]
    fn l3_rtt_interval_must_be_positive() {
        let o = NetRomInp3Options {
            l3_rtt_interval_ms: 0,
            ..NetRomInp3Options::DEFAULT
        };
        assert_eq!(o.validate(), Err("L3RTT probe interval must be positive"));
    }

    #[test]
    fn reset_window_must_exceed_the_probe_interval() {
        // Equal is rejected (must *exceed*).
        let equal = NetRomInp3Options {
            l3_rtt_interval_ms: 60_000,
            l3_rtt_reset_window_ms: 60_000,
            ..NetRomInp3Options::DEFAULT
        };
        assert!(equal.validate().is_err());

        // The C# validator case: interval 180 s, window 60 s (window < interval).
        let inverted = NetRomInp3Options {
            l3_rtt_interval_ms: 180_000,
            l3_rtt_reset_window_ms: 60_000,
            ..NetRomInp3Options::DEFAULT
        };
        assert_eq!(
            inverted.validate(),
            Err("L3RTT reset window must exceed the probe interval (a shorter window tears down a live neighbour before it can answer)")
        );

        // One ms over is accepted (the strict boundary).
        let just_over = NetRomInp3Options {
            l3_rtt_interval_ms: 60_000,
            l3_rtt_reset_window_ms: 60_001,
            ..NetRomInp3Options::DEFAULT
        };
        assert_eq!(just_over.validate(), Ok(()));
    }

    #[test]
    fn sntt_gain_shift_range_is_one_to_eight_inclusive() {
        // The C# config-validator case: SnttGainShift = 0 is out of range.
        let zero = NetRomInp3Options {
            sntt_gain_shift: 0,
            ..NetRomInp3Options::DEFAULT
        };
        assert_eq!(
            zero.validate(),
            Err("SNTT gain shift must be in [1, 8] (gain 1/2 .. 1/256)")
        );

        // 9 is past the top.
        let nine = NetRomInp3Options {
            sntt_gain_shift: 9,
            ..NetRomInp3Options::DEFAULT
        };
        assert!(nine.validate().is_err());

        // Both inclusive endpoints are accepted.
        for shift in 1u8..=8 {
            let o = NetRomInp3Options {
                sntt_gain_shift: shift,
                ..NetRomInp3Options::DEFAULT
            };
            assert_eq!(o.validate(), Ok(()), "shift {shift} must validate");
        }
    }

    #[test]
    fn advertise_ip_accept_must_be_a_single_digit_when_set() {
        // None is fine ($N only).
        assert_eq!(NetRomInp3Options::DEFAULT.validate(), Ok(()));

        // 0..=9 all accepted.
        for ip in 0u8..=9 {
            let o = NetRomInp3Options {
                advertise_ip_accept: Some(ip),
                ..NetRomInp3Options::DEFAULT
            };
            assert_eq!(o.validate(), Ok(()), "ip {ip} must validate");
        }

        // 10 is out of range.
        let ten = NetRomInp3Options {
            advertise_ip_accept: Some(10),
            ..NetRomInp3Options::DEFAULT
        };
        assert_eq!(
            ten.validate(),
            Err("advertised IP-accept version must be a single decimal digit 0\u{2013}9")
        );
    }

    #[test]
    fn hop_limit_floor_is_one() {
        // The C# config-validator case: HopLimit = 0 is out of range.
        let zero = NetRomInp3Options {
            hop_limit: 0,
            ..NetRomInp3Options::DEFAULT
        };
        assert_eq!(zero.validate(), Err("INP3 hop limit must be at least 1"));

        let one = NetRomInp3Options {
            hop_limit: 1,
            ..NetRomInp3Options::DEFAULT
        };
        assert_eq!(one.validate(), Ok(()));
    }

    #[test]
    fn rif_interval_must_be_positive() {
        let o = NetRomInp3Options {
            rif_interval_ms: 0,
            // keep debounce < interval invariant from firing first
            positive_debounce_ms: 0,
            ..NetRomInp3Options::DEFAULT
        };
        // positive_debounce check (==0) comes first in the C# order? No: the C#
        // order checks RifInterval before PositiveDebounce. With debounce 0 too,
        // RifInterval==0 is the first RIF/debounce guard reached.
        assert_eq!(o.validate(), Err("periodic RIF interval must be positive"));
    }

    #[test]
    fn positive_debounce_must_be_positive() {
        let o = NetRomInp3Options {
            positive_debounce_ms: 0,
            ..NetRomInp3Options::DEFAULT
        };
        assert_eq!(
            o.validate(),
            Err("positive-update debounce must be positive")
        );
    }

    #[test]
    fn positive_debounce_must_be_strictly_less_than_rif_interval() {
        // The C# config-validator case: RifInterval 5 s, PositiveDebounce 5 s (equal).
        let equal = NetRomInp3Options {
            rif_interval_ms: 5_000,
            positive_debounce_ms: 5_000,
            ..NetRomInp3Options::DEFAULT
        };
        assert_eq!(
            equal.validate(),
            Err("positive-update debounce must be less than the periodic RIF interval (a debounce >= the interval is pointless — the periodic emit would always drain the batch first)")
        );

        // debounce > interval also rejected.
        let over = NetRomInp3Options {
            rif_interval_ms: 5_000,
            positive_debounce_ms: 6_000,
            ..NetRomInp3Options::DEFAULT
        };
        assert!(over.validate().is_err());

        // One ms under is accepted (the strict boundary).
        let just_under = NetRomInp3Options {
            rif_interval_ms: 5_000,
            positive_debounce_ms: 4_999,
            ..NetRomInp3Options::DEFAULT
        };
        assert_eq!(just_under.validate(), Ok(()));
    }

    #[test]
    fn worsen_threshold_zero_is_valid() {
        // The C# guard is "non-negative"; 0 (the floor) validates.
        let o = NetRomInp3Options {
            worsen_threshold_ms: 0,
            ..NetRomInp3Options::DEFAULT
        };
        assert_eq!(o.validate(), Ok(()));
    }
}
