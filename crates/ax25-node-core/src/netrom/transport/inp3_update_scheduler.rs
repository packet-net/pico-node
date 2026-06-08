//! The host-free INP3 *triggered-update timing* state machine (slice I-4,
//! design §3): it answers **"when do we emit a RIF, and toward whom?"** — never
//! *what* the RIF contains (that is the routing table's `build_rif`, the content
//! half). It consumes per-destination *dirty signals* (the table / ingestion path
//! tells it a destination changed, and how — [`mark_dirty`] / [`mark_withdrawn`]),
//! consumes a host-driven [`tick`], and accumulates
//! [`Inp3AdvertiseIntent`]s ("advertise to neighbour X now") into an internal
//! outbox the host drains via [`take_advertise_intents`]. The host turns each
//! intent into `table.build_rif(my_call, X, prefer_inp3_routes)` + a send over X's
//! interlink.
//!
//! **Host-free + intent-emitting.** Like [`CircuitManager`] and the other core
//! engines, the scheduler owns no I/O, no routing table, and no AX.25 session — it
//! speaks only [`Callsign`] in and [`Inp3AdvertiseIntent`] out. It is a pure
//! function of (dirty signals, clock) → intents. The split keeps each piece pure:
//! `build_rif` is a pure read of table state; the scheduler is pure timing; the
//! host is the only stateful glue (design §3.1).
//!
//! **Monotonic clock — injected as a method parameter.** Unlike the C#/TS (which
//! capture a clock/closure and read elapsed time internally), the Rust core stores
//! no clock: `now_ms` is passed on each call ([`mark_dirty`] / [`mark_withdrawn`] /
//! [`tick`]), exactly as [`CircuitManager::on_packet`] / [`CircuitManager::tick`]
//! take it. The caller's `now_ms` is the embedding's *monotonic* millisecond tick
//! (never wall-clock — an NTP / DST step can never fire or suppress a debounce,
//! design §3.1). The periodic anchor is seeded to `0` at construction so the first
//! baseline refresh is due exactly one `rif_interval_ms` after the *first* tick's
//! clock reference, mirroring the C# monotonic-from-construction semantics.
//!
//! **Outbox / take — no closure sink.** The C#/TS expose a settable `Advertise`
//! action invoked per intent. The Rust core uses the OUTBOX/TAKE pattern instead
//! (the [`CircuitManager::take_outbox`] discipline): a [`tick`] accumulates intents
//! into an internal `Vec`, and the host drains them with
//! [`take_advertise_intents`]. There is no `FnMut` field — the firmware drains the
//! outbox and does the actual `build_rif` + interlink send, exactly as it drains the
//! circuit manager's outbox.
//!
//! **Per-destination dirty, per-neighbour fan-out.** Dirty state is tracked per
//! *destination* (a single change must reach every INP3-capable neighbour, each
//! with its own poison-reversed RIF at emit time — design §3.2); but the scheduler
//! only tracks *which destinations are dirty and at what priority* to decide
//! *whether / when / at what priority* to fan out — it never builds a partial RIF.
//! Every fan-out emits one intent per target neighbour, and the host rebuilds the
//! complete (full) poison-reversed RIF for each (design §3.3, "full RIF"): a
//! NEGATIVE fan-out therefore naturally carries the changed destination's
//! new/withdrawn state and subsumes any pending POSITIVE batch.
//!
//! **Totality.** Marking a destination dirty never panics; [`tick`] with no
//! neighbours and no dirty state is a no-op (and with no neighbours but dirty
//! state, the dirty is still consumed — a fan-out to an empty neighbour set is an
//! empty fan-out). The recently-withdrawn set is *not* held here — it is table
//! state (design AMBIGUITY-I4-5: `build_rif` consumes-and-clears it); a withdrawal
//! here only escalates the destination to NEGATIVE so the fan-out is immediate.
//!
//! Mirrors `Packet.NetRom.Transport.Inp3UpdateScheduler` on the C# side (and the
//! merged TS `inp3-update-scheduler.ts`).
//!
//! [`mark_dirty`]: Inp3UpdateScheduler::mark_dirty
//! [`mark_withdrawn`]: Inp3UpdateScheduler::mark_withdrawn
//! [`tick`]: Inp3UpdateScheduler::tick
//! [`take_advertise_intents`]: Inp3UpdateScheduler::take_advertise_intents
//! [`CircuitManager`]: super::circuit_manager::CircuitManager
//! [`CircuitManager::on_packet`]: super::circuit_manager::CircuitManager::on_packet
//! [`CircuitManager::tick`]: super::circuit_manager::CircuitManager::tick
//! [`CircuitManager::take_outbox`]: super::circuit_manager::CircuitManager::take_outbox

use alloc::vec::Vec;

use crate::ax25::Callsign;

/// The change class of a destination's selected INP3 route, set by whoever marks it
/// dirty (the table / ingestion path — design §3.2). NEGATIVE is immediate +
/// prioritised; POSITIVE is debounced + batched.
///
/// Mirrors `Packet.NetRom.Transport.Inp3UpdateClass`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Inp3UpdateClass {
    /// A new / improved / faster-next-hop route, or a sub-threshold worsening
    /// (routine SNTT jitter). Batched behind the positive-debounce window.
    Positive,
    /// A route lost (withdrawal / `mark_neighbour_down` / aged out) or a selected
    /// target time worsened by ≥ the worsen threshold. Fans out immediately on the
    /// next [`Inp3UpdateScheduler::tick`], ahead of any pending positive batch.
    Negative,
}

/// Why a fan-out fired, carried on each [`Inp3AdvertiseIntent`] for observability
/// (design §3.6).
///
/// Mirrors `Packet.NetRom.Transport.Inp3AdvertiseReason`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Inp3AdvertiseReason {
    /// A dirty-driven fan-out — a NEGATIVE change (immediate) or a debounced batch
    /// of POSITIVE changes.
    Triggered,
    /// The baseline periodic full-RIF refresh on the RIF interval, regardless of
    /// dirty state.
    Periodic,
}

/// One "advertise to neighbour X now" intent the scheduler emits into its outbox.
/// The host turns it into `table.build_rif(my_call, neighbour, prefer_inp3_routes)`
/// (the full, poison-reversed RIF) and a send over the neighbour's interlink
/// session.
///
/// Mirrors `Packet.NetRom.Transport.Inp3AdvertiseIntent` (a `readonly record
/// struct` there).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Inp3AdvertiseIntent {
    /// The INP3-capable neighbour to (re)advertise toward.
    pub neighbour: Callsign,
    /// Why this fan-out fired (triggered vs periodic).
    pub reason: Inp3AdvertiseReason,
}

/// An immutable snapshot of the scheduler's pending dirty state, for surfacing /
/// tests (the [`Inp3UpdateScheduler::status`] projection).
///
/// Mirrors `Packet.NetRom.Transport.Inp3SchedulerStatus`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Inp3SchedulerStatus {
    /// Destinations pending an immediate (NEGATIVE) fan-out.
    pub negative_dirty: usize,
    /// Destinations pending a debounced (POSITIVE) fan-out.
    pub positive_dirty: usize,
    /// The current INP3-capable fan-out target count.
    pub target_neighbours: usize,
}

/// Sentinel for [`Inp3UpdateScheduler::earliest_positive_mark_ms`]: "no POSITIVE
/// mark is pending", distinct from the monotonic clock's legitimate `0` at
/// construction (a positive marked at `t=0` must not read as "none pending"). The
/// C# uses `long.MinValue`; here the clock is an unsigned `u64` ms tick, so
/// `u64::MAX` is the faithful sentinel — strictly above any real monotonic ms, so
/// `now - earliest >= debounce` can never spuriously fire while it is set.
const NEVER_MARKED: u64 = u64::MAX;

/// The host-free INP3 triggered-update timing state machine. See the module docs.
pub struct Inp3UpdateScheduler {
    /// The periodic full-RIF cadence in ms (the C# `RifInterval`; TS `rifIntervalMs`).
    rif_interval_ms: u64,
    /// The positive-update coalescing window in ms (the C# `PositiveDebounce`).
    positive_debounce_ms: u64,

    /// Per-destination dirty class. A destination is in at most one class at a time
    /// (design §3.2). Absent ⇒ clean. The C#/TS key a hash map by callsign; the
    /// Rust core's [`Callsign`] is neither `Hash` nor `Ord`, and a node's INP3
    /// dirty set is tiny, so a linear-scan `Vec` of `(dest, class)` is the faithful
    /// `no_std` equivalent — same semantics, no map-key trait requirement.
    dirty: Vec<(Callsign, Inp3UpdateClass)>,

    /// The INP3-capable neighbour set to fan out to — host-supplied
    /// ([`set_target_neighbours`](Self::set_target_neighbours)); the scheduler never
    /// discovers neighbours (host-free, design §3.2/§3.6). Stored distinct +
    /// callsign-ordered so a fan-out emits intents in a deterministic order.
    target_neighbours: Vec<Callsign>,

    /// Monotonic ms of the *earliest still-pending* POSITIVE mark — the debounce
    /// anchor (design §3.3 rule 2: a steady positive drip drains within one
    /// `positive_debounce_ms` of the first, not perpetually deferred).
    /// [`NEVER_MARKED`] when no POSITIVE is pending.
    earliest_positive_mark_ms: u64,

    /// Monotonic ms of the last periodic fan-out (design §3.3 rule 3), anchored at
    /// construction (monotonic `0`) so the first baseline refresh fires exactly one
    /// `rif_interval_ms` after the scheduler is built — timing depends only on the
    /// caller's clock, not on when ticking begins.
    last_periodic_ms: u64,

    /// The accumulated advertise intents awaiting [`take_advertise_intents`]
    /// (the OUTBOX half of the OUTBOX/TAKE pattern — replaces the C#/TS `Advertise`
    /// closure sink).
    ///
    /// [`take_advertise_intents`]: Self::take_advertise_intents
    outbox: Vec<Inp3AdvertiseIntent>,
}

impl Inp3UpdateScheduler {
    /// Construct the scheduler with its two timing knobs.
    ///
    /// The C# constructor takes a whole `NetRomInp3Options` and reads only its
    /// `RifInterval` + `PositiveDebounce`; the Rust core takes those two ms values
    /// directly (the only fields the scheduler reads), matching the core's
    /// pass-the-primitives idiom and avoiding a dependency on the wire options
    /// module while it is still being landed by the fleet. `rif_interval_ms` is the
    /// periodic full-RIF cadence; `positive_debounce_ms` is the positive-update
    /// coalescing window (it should be > 0 and < `rif_interval_ms`, per the C#
    /// `NetRomInp3Options.Validate`; the scheduler does not re-validate — the host's
    /// options resolver owns that).
    pub fn new(rif_interval_ms: u64, positive_debounce_ms: u64) -> Self {
        Self {
            rif_interval_ms,
            positive_debounce_ms,
            dirty: Vec::new(),
            target_neighbours: Vec::new(),
            earliest_positive_mark_ms: NEVER_MARKED,
            last_periodic_ms: 0,
            outbox: Vec::new(),
        }
    }

    /// Set the INP3-capable neighbour set to fan out to. Host-supplied (e.g. from
    /// the engine's neighbour set filtered to INP3-capable); the scheduler never
    /// discovers neighbours. Replaces the previous set wholesale. Takes a
    /// defensive, distinct + callsign-ordered copy so a duplicate in the host set
    /// cannot double-advertise to one neighbour and the fan-out order is
    /// deterministic. Removing a neighbour here simply stops it receiving future
    /// fan-outs; it does not clear any dirty state (the next fan-out reaches
    /// whatever set is current at that [`tick`](Self::tick)).
    ///
    /// Mirrors the C# `SetTargetNeighbours`.
    pub fn set_target_neighbours(&mut self, capable_neighbours: &[Callsign]) {
        // Distinct + ordered: a duplicate in the host set must not double-advertise
        // to one neighbour, and a stable order keeps a fan-out's intents
        // deterministic (the C# `Distinct().OrderBy(ToString(), Ordinal)` /
        // the table.rs `callsign_lt` ordinal discipline).
        let mut snapshot: Vec<Callsign> = Vec::new();
        for &c in capable_neighbours {
            if !snapshot.contains(&c) {
                snapshot.push(c);
            }
        }
        snapshot.sort_by(cmp_callsign);
        self.target_neighbours = snapshot;
    }

    /// Mark a destination dirty with a change class (design §3.2). The table /
    /// ingestion path computes the class: NEGATIVE for a selected route worsened by
    /// ≥ the worsen threshold; POSITIVE for a new / improved / faster-next-hop route
    /// or a sub-threshold worsening. The class is **monotonic within the debounce
    /// window**: a POSITIVE destination re-marked NEGATIVE is *upgraded* to NEGATIVE
    /// (a loss must not be held back by a coincident positive); a NEGATIVE
    /// destination re-marked POSITIVE is **not** downgraded. Never panics; the actual
    /// fan-out happens on the next [`tick`](Self::tick) (NEGATIVE immediately,
    /// POSITIVE after the debounce). `now_ms` is the caller's monotonic ms tick.
    ///
    /// Mirrors the C# `MarkDirty`.
    pub fn mark_dirty(&mut self, destination: Callsign, cls: Inp3UpdateClass, now_ms: u64) {
        self.mark(destination, cls, now_ms);
    }

    /// Mark a destination's selected INP3 route *withdrawn* (fully lost — no selected
    /// INP3 route remains). A withdrawal is **always NEGATIVE** regardless of any
    /// threshold (design §3.2: it is a removal, not a worsening) so it fans out on
    /// the next [`tick`](Self::tick) immediately. The explicit one-shot horizon
    /// withdrawal RIP itself is emitted by `build_rif` from the *table's*
    /// recently-withdrawn set (design AMBIGUITY-I4-5) — the scheduler only escalates
    /// the timing here; it does not hold the withdrawn set. `now_ms` is the caller's
    /// monotonic ms tick.
    ///
    /// Mirrors the C# `MarkWithdrawn`.
    pub fn mark_withdrawn(&mut self, destination: Callsign, now_ms: u64) {
        self.mark(destination, Inp3UpdateClass::Negative, now_ms);
    }

    /// Advance the clock-driven state machine and fan out any updates now due,
    /// accumulating intents into the outbox (drained via
    /// [`take_advertise_intents`](Self::take_advertise_intents)). On each tick
    /// (design §3.3), in precedence:
    ///
    /// 1. **Any NEGATIVE dirty → immediate, prioritised.** Emit an
    ///    [`Inp3AdvertiseReason::Triggered`] intent for *every* target neighbour now
    ///    and clear **all** dirty (the full poison-reversed RIF the host rebuilds
    ///    subsumes pending positives too). No debounce.
    /// 2. **Else POSITIVE dirty and debounce elapsed → batched.** If any destination
    ///    is POSITIVE and `now - earliest_positive_mark >= positive_debounce`, emit a
    ///    [`Inp3AdvertiseReason::Triggered`] intent for every neighbour and clear the
    ///    POSITIVE dirty. The debounce coalesces a burst of positives into one
    ///    fan-out.
    /// 3. **Independently, periodic interval elapsed → full RIF regardless.** If
    ///    `now - last_periodic_emit >= rif_interval`, emit an
    ///    [`Inp3AdvertiseReason::Periodic`] intent for every neighbour, stamp the
    ///    periodic anchor, clear all dirty, and reset the debounce.
    ///
    /// At most one fan-out per tick — the rebuilt full RIF carries all current
    /// state, so there is never a reason to fan out twice. `now_ms` is the caller's
    /// monotonic ms tick (drive it from the host's interval).
    ///
    /// Mirrors the C# `Tick`.
    pub fn tick(&mut self, now_ms: u64) {
        // The periodic anchor was seeded to 0 (monotonic construction time), so the
        // first baseline refresh is due exactly one rif_interval after construction —
        // timing depends only on the caller's clock, not on when ticking began.
        let periodic_due = now_ms.saturating_sub(self.last_periodic_ms) >= self.rif_interval_ms;

        let mut negative_due = false;
        let mut positive_due = false;
        if !periodic_due {
            for (_, cls) in &self.dirty {
                if *cls == Inp3UpdateClass::Negative {
                    negative_due = true;
                    break; // NEGATIVE dominates — no need to scan further.
                }
            }
            if !negative_due
                && self.earliest_positive_mark_ms != NEVER_MARKED
                && now_ms.saturating_sub(self.earliest_positive_mark_ms) >= self.positive_debounce_ms
            {
                positive_due = true;
            }
        }

        // A periodic emit subsumes everything (full RIF) and takes the Periodic
        // reason; otherwise a NEGATIVE (immediate) or a debounced POSITIVE fans out
        // as Triggered. At most one fan-out per tick.
        let reason = if periodic_due {
            Some(Inp3AdvertiseReason::Periodic)
        } else if negative_due || positive_due {
            Some(Inp3AdvertiseReason::Triggered)
        } else {
            None
        };

        if let Some(reason) = reason {
            for &neighbour in &self.target_neighbours {
                self.outbox.push(Inp3AdvertiseIntent { neighbour, reason });
            }

            // Clearing semantics (design §3.3):
            //  - Periodic and NEGATIVE both clear ALL dirty (the full RIF subsumes
            //    every pending change) and reset the debounce anchor.
            //  - A pure debounced-POSITIVE fan-out clears only POSITIVE dirty (there
            //    are no NEGATIVEs by construction — rule 1 would have won) which, in
            //    practice, is also all dirty; either way the debounce anchor resets.
            self.dirty.clear();
            self.earliest_positive_mark_ms = NEVER_MARKED;

            if periodic_due {
                self.last_periodic_ms = now_ms;
            }
        }
    }

    /// Drain the advertise intents accumulated since the last call (the TAKE half of
    /// the OUTBOX/TAKE pattern — the host's replacement for the C#/TS `Advertise`
    /// closure). Returns them in fan-out order (callsign-ordered per fan-out, in the
    /// order the fan-outs were produced). Empty when nothing fired.
    ///
    /// Mirrors [`CircuitManager::take_outbox`](super::circuit_manager::CircuitManager::take_outbox).
    pub fn take_advertise_intents(&mut self) -> Vec<Inp3AdvertiseIntent> {
        core::mem::take(&mut self.outbox)
    }

    /// A point-in-time snapshot of pending dirty state, for surfacing / tests: how
    /// many destinations are dirty NEGATIVE vs POSITIVE, and the current neighbour
    /// fan-out count. A pure read.
    ///
    /// Mirrors the C# `Status` property.
    pub fn status(&self) -> Inp3SchedulerStatus {
        let mut negative = 0;
        let mut positive = 0;
        for (_, cls) in &self.dirty {
            match cls {
                Inp3UpdateClass::Negative => negative += 1,
                Inp3UpdateClass::Positive => positive += 1,
            }
        }
        Inp3SchedulerStatus {
            negative_dirty: negative,
            positive_dirty: positive,
            target_neighbours: self.target_neighbours.len(),
        }
    }

    // ─── Internals ──────────────────────────────────────────────────────

    /// Apply a dirty mark with the monotonic-within-window rule: POSITIVE→NEGATIVE
    /// upgrades; NEGATIVE→POSITIVE does not downgrade; a fresh POSITIVE anchors the
    /// debounce window if none is pending. Mirrors the C# `MarkLocked` (there is no
    /// lock in the single-threaded core).
    fn mark(&mut self, destination: Callsign, cls: Inp3UpdateClass, now_ms: u64) {
        if let Some(entry) = self.dirty.iter_mut().find(|(d, _)| *d == destination) {
            // Upgrade-only: NEGATIVE dominates, so only POSITIVE→NEGATIVE changes
            // the stored class. NEGATIVE→POSITIVE (and same-class) leave it
            // untouched.
            if entry.1 == Inp3UpdateClass::Positive && cls == Inp3UpdateClass::Negative {
                entry.1 = Inp3UpdateClass::Negative;
            }
            return;
        }

        self.dirty.push((destination, cls));
        if cls == Inp3UpdateClass::Positive && self.earliest_positive_mark_ms == NEVER_MARKED {
            // Anchor the debounce on the EARLIEST still-pending positive so a steady
            // drip drains within one window of the first mark, not perpetually
            // deferred.
            self.earliest_positive_mark_ms = now_ms;
        }
    }
}

/// Ordinal callsign comparison: base bytes then SSID. Mirrors the C# snapshot's
/// `StringComparer.Ordinal` over `callsign.ToString()` (and the routing table's
/// `callsign_lt`) for the alphanumeric base + SSID forms NET/ROM uses, giving a
/// deterministic neighbour fan-out order.
fn cmp_callsign(a: &Callsign, b: &Callsign) -> core::cmp::Ordering {
    a.base().cmp(b.base()).then_with(|| a.ssid().cmp(&b.ssid()))
}

#[cfg(test)]
mod tests {
    use super::*;

    // Timing knobs matching the C#/TS test defaults (300 s periodic, 5 s debounce).
    const RIF_INTERVAL_MS: u64 = 300_000;
    const POSITIVE_DEBOUNCE_MS: u64 = 5_000;

    fn call(text: &str) -> Callsign {
        Callsign::new(text.as_bytes(), 0).unwrap()
    }

    fn n1() -> Callsign {
        call("GB7AAA")
    }
    fn n2() -> Callsign {
        call("GB7BBB")
    }
    fn n3() -> Callsign {
        call("GB7CCC")
    }
    fn dest_a() -> Callsign {
        call("M0AAA")
    }
    fn dest_b() -> Callsign {
        call("M0BBB")
    }

    fn new_scheduler(neighbours: &[Callsign]) -> Inp3UpdateScheduler {
        new_scheduler_with(RIF_INTERVAL_MS, POSITIVE_DEBOUNCE_MS, neighbours)
    }

    fn new_scheduler_with(
        rif_interval_ms: u64,
        positive_debounce_ms: u64,
        neighbours: &[Callsign],
    ) -> Inp3UpdateScheduler {
        let mut s = Inp3UpdateScheduler::new(rif_interval_ms, positive_debounce_ms);
        s.set_target_neighbours(neighbours);
        s
    }

    /// The set of neighbours an intent list fanned out to, sorted for comparison.
    fn neighbours_of(intents: &[Inp3AdvertiseIntent]) -> Vec<Callsign> {
        let mut v: Vec<Callsign> = intents.iter().map(|i| i.neighbour).collect();
        v.sort_by(cmp_callsign);
        v
    }

    #[test]
    fn negative_fires_immediately_on_next_tick_for_every_neighbour() {
        let mut s = new_scheduler(&[n1(), n2(), n3()]);

        s.mark_withdrawn(dest_a(), 0); // a loss is always NEGATIVE

        // No debounce on NEGATIVE: the very next tick (no clock advance) fans out.
        s.tick(0);
        let intents = s.take_advertise_intents();

        // fans out to every INP3-capable neighbour at once
        assert_eq!(intents.len(), 3);
        assert_eq!(neighbours_of(&intents), {
            let mut e = [n1(), n2(), n3()];
            e.sort_by(cmp_callsign);
            e.to_vec()
        });
        assert!(intents
            .iter()
            .all(|i| i.reason == Inp3AdvertiseReason::Triggered));

        // The dirty flag cleared — a follow-up tick with no new change is silent.
        s.tick(0);
        // the NEGATIVE fan-out cleared the dirty flag
        assert!(s.take_advertise_intents().is_empty());
    }

    #[test]
    fn positive_is_debounced_then_fires() {
        let mut s = new_scheduler(&[n1(), n2()]);

        s.mark_dirty(dest_a(), Inp3UpdateClass::Positive, 0);

        // Before the 5 s debounce elapses, ticking does NOT fan out the positive.
        s.tick(0);
        assert!(s.take_advertise_intents().is_empty()); // held until the debounce elapses

        s.tick(4_000);
        assert!(s.take_advertise_intents().is_empty()); // still inside the 5 s window

        // Crossing the debounce boundary fans out once, to every neighbour.
        s.tick(5_000); // t = 5 s == positive_debounce
        let intents = s.take_advertise_intents();
        assert_eq!(intents.len(), 2); // drains to both neighbours once the window elapses
        assert_eq!(neighbours_of(&intents), {
            let mut e = [n1(), n2()];
            e.sort_by(cmp_callsign);
            e.to_vec()
        });
        assert!(intents
            .iter()
            .all(|i| i.reason == Inp3AdvertiseReason::Triggered));

        // Drained — no repeat.
        s.tick(15_000);
        assert!(s.take_advertise_intents().is_empty()); // the positive batch drained exactly once
    }

    #[test]
    fn positive_burst_within_the_window_coalesces_to_one_fan_out() {
        let mut s = new_scheduler(&[n1(), n2()]);

        // Two positives marked at different times inside one debounce window.
        s.mark_dirty(dest_a(), Inp3UpdateClass::Positive, 0);
        s.mark_dirty(dest_b(), Inp3UpdateClass::Positive, 2_000);

        // The debounce is anchored on the EARLIEST mark (dest_a at t=0), so it
        // drains at t=5 (one window after the first), not t=7.
        s.tick(5_000); // t = 5 s
        let intents = s.take_advertise_intents();

        // a burst coalesces into ONE fan-out per neighbour
        assert_eq!(intents.len(), 2);
        assert_eq!(neighbours_of(&intents), {
            let mut e = [n1(), n2()];
            e.sort_by(cmp_callsign);
            e.to_vec()
        });
    }

    #[test]
    fn periodic_fires_on_interval_regardless_of_dirty_state() {
        let mut s = new_scheduler(&[n1(), n2()]);

        // No dirty state at all. Nothing fires before the interval.
        s.tick(299_000);
        assert!(s.take_advertise_intents().is_empty()); // not yet at its interval

        // Crossing the 300 s interval fires a Periodic fan-out to every neighbour.
        s.tick(300_000); // t = 300 s
        let intents = s.take_advertise_intents();
        assert_eq!(intents.len(), 2); // the periodic full RIF fans out to every neighbour
        assert_eq!(neighbours_of(&intents), {
            let mut e = [n1(), n2()];
            e.sort_by(cmp_callsign);
            e.to_vec()
        });
        assert!(intents
            .iter()
            .all(|i| i.reason == Inp3AdvertiseReason::Periodic));

        // And again one interval later — it re-anchors each time.
        s.tick(600_000);
        let intents = s.take_advertise_intents();
        assert_eq!(intents.len(), 2); // the periodic refresh re-fires every interval
        assert!(intents
            .iter()
            .all(|i| i.reason == Inp3AdvertiseReason::Periodic));
    }

    #[test]
    fn periodic_subsumes_a_pending_positive_batch_and_resets_the_debounce() {
        let mut s = new_scheduler_with(10_000, 5_000, &[n1(), n2()]);

        // Mark a positive at t=6 s — it would drain at t=11 s. The periodic at
        // t=10 s pre-empts it as a single Periodic fan-out.
        s.mark_dirty(dest_a(), Inp3UpdateClass::Positive, 6_000);

        s.tick(10_000); // t = 10 s == rif_interval
        let intents = s.take_advertise_intents();

        assert_eq!(intents.len(), 2); // the periodic emit fans out once per neighbour
        // a periodic emit subsumes the pending positive batch (full RIF) — not a
        // second Triggered fan-out
        assert!(intents
            .iter()
            .all(|i| i.reason == Inp3AdvertiseReason::Periodic));

        // The pending positive was cleared by the periodic; it does NOT re-drain.
        s.tick(15_000); // would have been the old debounce boundary
        // the periodic cleared the pending positive and reset the debounce
        assert!(s.take_advertise_intents().is_empty());
    }

    #[test]
    fn negative_preempts_a_pending_positive_batch() {
        let mut s = new_scheduler(&[n1(), n2()]);

        // A positive is sitting in the debounce window...
        s.mark_dirty(dest_a(), Inp3UpdateClass::Positive, 0);
        // ...then a NEGATIVE arrives 2 s in for a different destination. The next
        // tick fans out IMMEDIATELY (no waiting out the positive's debounce) as
        // Triggered, and clears BOTH the negative and the still-pending positive
        // (full RIF subsumes).
        s.mark_withdrawn(dest_b(), 2_000);
        s.tick(2_000);
        let intents = s.take_advertise_intents();

        assert_eq!(intents.len(), 2); // the NEGATIVE pre-empts and fans out immediately
        assert!(intents
            .iter()
            .all(|i| i.reason == Inp3AdvertiseReason::Triggered));

        // The previously-pending positive was subsumed — nothing left to drain at
        // the old debounce boundary.
        s.tick(12_000); // well past the old positive's 5 s boundary
        // the NEGATIVE fan-out subsumed the pending positive batch (full RIF)
        assert!(s.take_advertise_intents().is_empty());
    }

    #[test]
    fn negative_upgrade_within_window_is_immediate_and_does_not_downgrade() {
        let mut s = new_scheduler(&[n1(), n2()]);

        // POSITIVE then upgraded to NEGATIVE for the SAME destination → NEGATIVE wins.
        s.mark_dirty(dest_a(), Inp3UpdateClass::Positive, 0);
        s.mark_dirty(dest_a(), Inp3UpdateClass::Negative, 0); // upgrade

        assert_eq!(s.status().negative_dirty, 1); // POSITIVE→NEGATIVE upgrades the class
        assert_eq!(s.status().positive_dirty, 0);

        s.tick(0);
        let intents = s.take_advertise_intents();
        assert_eq!(intents.len(), 2); // the upgraded-to-NEGATIVE dest fans out immediately
        assert!(intents
            .iter()
            .all(|i| i.reason == Inp3AdvertiseReason::Triggered));

        // The reverse: NEGATIVE then POSITIVE for the same dest must NOT downgrade.
        s.mark_dirty(dest_b(), Inp3UpdateClass::Negative, 0);
        s.mark_dirty(dest_b(), Inp3UpdateClass::Positive, 0); // must NOT downgrade
        // a loss cannot be demoted to a batched positive
        assert_eq!(s.status().negative_dirty, 1);
        s.tick(0);
        let intents = s.take_advertise_intents();
        // the still-NEGATIVE destination fans out immediately, not after a debounce
        assert_eq!(intents.len(), 2);
    }

    #[test]
    fn no_neighbours_means_no_intents_even_when_dirty() {
        let mut s = new_scheduler(&[]);

        s.mark_withdrawn(dest_a(), 0);
        s.tick(0);
        // with no target neighbours there is no one to advertise to
        assert!(s.take_advertise_intents().is_empty());

        // Adding neighbours later does not resurrect the already-cleared dirty flag —
        // the NEGATIVE was consumed by the (empty) fan-out on the previous tick.
        s.set_target_neighbours(&[n1()]);
        s.tick(0);
        // the NEGATIVE dirty was cleared by the prior (empty) fan-out
        assert!(s.take_advertise_intents().is_empty());
    }

    #[test]
    fn duplicate_neighbours_are_de_duplicated() {
        let mut s = new_scheduler(&[n1(), n1(), n2()]);

        s.mark_withdrawn(dest_a(), 0);
        s.tick(0);
        let intents = s.take_advertise_intents();

        // a duplicate in the host neighbour set must not double-advertise to one
        assert_eq!(neighbours_of(&intents), {
            let mut e = [n1(), n2()];
            e.sort_by(cmp_callsign);
            e.to_vec()
        });
    }

    #[test]
    fn first_tick_does_not_fire_a_periodic_immediately() {
        let mut s = new_scheduler(&[n1(), n2()]);

        // A brand-new scheduler ticked at t=0 must NOT fire a periodic — it waits one
        // full interval (the periodic anchor was seeded to 0 at construction).
        s.tick(0);
        // the periodic refresh waits one full interval after construction
        assert!(s.take_advertise_intents().is_empty());
    }
}
