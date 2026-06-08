//! The host-free INP3 **link-timing** engine (slice I-2): it owns the per-neighbour
//! INP3 state, probes each interlink neighbour with L3RTT datagrams on a cadence,
//! times the reflections (`RTT / 2` → the [`crate::netrom::routing::inp3_sntt`]
//! SNTT smoother), reflects a peer's probes back verbatim, learns INP3 capability
//! from the `$N` / `$IX` flags, and raises a *neighbour-down* signal when a
//! previously-capable neighbour stops reflecting for the reset window (default
//! 180 s). This is link timing only; it produces the SNTT value the route layer
//! (I-3) consumes but does not itself touch the routing table beyond signalling a
//! down neighbour.
//!
//! Ports `Packet.NetRom.Transport.Inp3Engine` (and the merged TS
//! `ax25-ts/src/netrom/inp3-engine.ts`).
//!
//! ### Rust core idiom — OUTBOX/TAKE + `now_ms`, no closures / no stored clock
//!
//! The C#/TS reference wires a `SendL3Rtt` action sink and a `NeighbourDown`
//! event. The Rust core uses the OUTBOX/TAKE pattern instead (the
//! [`CircuitManager`](super::circuit_manager::CircuitManager) discipline):
//!
//! - **No `send_l3rtt` closure.** Outbound L3RTT sends (probes the engine
//!   originates + verbatim reflections of a peer's probe) accumulate into an
//!   internal `Vec`; the host drains it via
//!   [`take_outbound_l3rtt`](Inp3Engine::take_outbound_l3rtt), wraps each frame in
//!   a PID-0xCF I-frame on the named neighbour's interlink session, and ships it.
//! - **No `neighbour_down` closure / event.** A 180 s no-reflection reset of a
//!   previously-INP3-capable neighbour pushes an [`Inp3NeighbourDownEvent`] into a
//!   second internal `Vec`, drained via
//!   [`take_neighbour_down`](Inp3Engine::take_neighbour_down). The host wires it to
//!   `NetRomRoutingTable::mark_neighbour_down` + a DISC / re-establish.
//! - **No stored clock.** Time is a `now_ms: u64` *method parameter* on every
//!   state-advancing call ([`tick`](Inp3Engine::tick),
//!   [`on_l3rtt`](Inp3Engine::on_l3rtt),
//!   [`observe_neighbour`](Inp3Engine::observe_neighbour)), exactly as
//!   [`CircuitManager::tick`](super::circuit_manager::CircuitManager::tick) takes
//!   it. `now_ms` is the embedding's *monotonic* millisecond tick — never
//!   wall-clock, so an NTP / DST step can never corrupt an RTT or fire / suppress
//!   the 180 s reset (design §2.1, AMBIGUITY-I2-5).
//!
//! ### The never-probed sentinel (load-bearing)
//!
//! The "no probe ever sent" marker for `last_l3rtt_sent_ms` is [`NEVER_PROBED`]
//! (`u64::MAX`), **not** `0` — the monotonic `now_ms` can legitimately be `0`, and
//! a probe genuinely sent at `t = 0` must not read as never-sent or the cadence
//! gate would re-fire it every tick. The C# uses `long.MinValue`; the core's clock
//! is an unsigned `u64`, so `u64::MAX` is the faithful sentinel — strictly above
//! any real monotonic ms, so the `now - last_sent >= cadence` arithmetic (guarded
//! by an explicit `== NEVER_PROBED` check first) never spuriously fires while it is
//! set.
//!
//! ### The AMBIGUITY-I2-3 graceful-degradation guard (load-bearing)
//!
//! On the reset window expiring, [`Inp3NeighbourDownEvent`] is enqueued **only** for
//! a neighbour that had proven INP3-capable. A never-capable vanilla neighbour that
//! never reflected our optimistic probes is dropped from probing *silently* — it is
//! reachable by vanilla NODES, it just does not speak L3RTT, so its silence must
//! never be fed into a routing teardown.
//!
//! ### Totality
//!
//! The engine never panics on any inbound frame: a negative / stale RTT, an
//! unsolicited reflection, a reflection from an unknown neighbour, or a non-L3RTT
//! packet are all handled without corrupting the metric (parsing returns `Option`,
//! never panics — the §0 totality contract).

use alloc::vec::Vec;

use crate::ax25::Callsign;
use crate::netrom::routing::inp3_sntt::{self, SNTT_SAMPLE_MAX_MS, SNTT_UNSET_RAW};
use crate::netrom::wire::inp3_l3rtt::Inp3L3RttFrame;
use crate::netrom::wire::inp3_options::NetRomInp3Options;
use crate::netrom::wire::packet::NetRomPacket;

/// Sentinel for [`Inp3NeighbourState::last_l3rtt_sent_ms`] meaning "no probe ever
/// sent" — distinct from the monotonic clock's legitimate `0` at engine start. The
/// C# uses `long.MinValue`; the core's clock is an unsigned `u64` ms tick, so
/// `u64::MAX` is the faithful sentinel (strictly above any real monotonic ms, and
/// the explicit `== NEVER_PROBED` guard means the cadence arithmetic never touches
/// it). See the module docs.
const NEVER_PROBED: u64 = u64::MAX;

/// Carries an INP3 link-down signal: a previously-INP3-capable neighbour went
/// silent past the reset window. The host drains it from
/// [`Inp3Engine::take_neighbour_down`] and wires it to
/// `NetRomRoutingTable::mark_neighbour_down` + a DISC / re-establish of the
/// interlink. The engine has already reset (removed) that neighbour's INP3 state by
/// the time this is enqueued.
///
/// Mirrors `Packet.NetRom.Transport.Inp3NeighbourDownEventArgs` / the TS
/// `Inp3NeighbourDownEvent`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Inp3NeighbourDownEvent {
    /// The neighbour to `mark_neighbour_down`.
    pub neighbour: Callsign,
    /// How long since its last reflection (≥ the reset window), in ms.
    pub silent_for_ms: u64,
}

/// An immutable snapshot of one neighbour's INP3 link-timing state, for surfacing /
/// tests (the [`Inp3Engine::neighbours`] projection). Mirrors
/// `Packet.NetRom.Transport.Inp3NeighbourTiming` / the TS `Inp3NeighbourTiming`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Inp3NeighbourTiming {
    /// The neighbour callsign.
    pub neighbour: Callsign,
    /// The smoothed neighbour transport time (ms), or `None` if no measurement yet.
    pub sntt_ms: Option<u32>,
    /// Whether the neighbour has advertised INP3 capability.
    pub inp3_capable: bool,
    /// The IP version the neighbour accepts (from `$IX`), or `None`.
    pub ip_accept: Option<u8>,
    /// Monotonic ms since the neighbour last reflected (or since it was registered,
    /// if it never has).
    pub last_reflection_age_ms: u64,
    /// Whether a probe is currently outstanding.
    pub awaiting_reflection: bool,
}

/// The per-neighbour INP3 link-timing state (design §1 / plan §5.1). All timestamps
/// are monotonic ms from the caller's `now_ms`. Mirrors the C# private
/// `Inp3NeighbourState` class / the TS `Inp3NeighbourState` record.
#[derive(Debug, Clone, Copy)]
struct Inp3NeighbourState {
    /// The neighbour callsign (kept so a snapshot/teardown can name it without
    /// re-deriving it from the table position).
    neighbour: Callsign,
    /// Smoothed neighbour transport time (the link metric); [`SNTT_UNSET_RAW`] until
    /// the first reflection.
    sntt_ms: u32,
    /// Monotonic ms when we last SENT a probe; [`NEVER_PROBED`] = never probed.
    last_l3rtt_sent_ms: u64,
    /// Monotonic ms when this neighbour last reflected our probe (drives the reset
    /// timer); seeded to "now" at add-time.
    last_reflection_ms: u64,
    /// Learned from the peer's `$N` flag.
    inp3_capable: bool,
    /// From `$IX`, if advertised; else `None`.
    ip_accept: Option<u8>,
    /// A probe is outstanding (sent, not yet reflected). At most one in flight per
    /// neighbour — bounds state and makes "is this reflection ours?" unambiguous.
    awaiting_reflection: bool,
}

impl Inp3NeighbourState {
    /// Whether a valid SNTT measurement exists yet.
    fn sntt_initialised(&self) -> bool {
        self.sntt_ms != SNTT_UNSET_RAW
    }
}

/// The host-free INP3 link-timing engine. See the module docs.
///
/// Mirrors `Packet.NetRom.Transport.Inp3Engine`.
pub struct Inp3Engine {
    /// Our own L3 callsign — the origin we stamp into probes and the
    /// [`Inp3L3RttFrame::is_reflection_of`] self-test target.
    local_node: Callsign,
    options: NetRomInp3Options,
    cadence_ms: u64,
    reset_window_ms: u64,

    /// Per-neighbour state. The core's [`Callsign`] is neither `Hash` nor `Ord`, and
    /// a node's interlink-neighbour set is tiny, so a linear-scan `Vec` is the
    /// faithful `no_std` equivalent of the C#/TS map — same semantics, no map-key
    /// trait requirement (the same choice the
    /// [`super::inp3_update_scheduler::Inp3UpdateScheduler`] makes).
    neighbours: Vec<Inp3NeighbourState>,

    /// Outbound L3RTT sends queued since the last drain (probes + verbatim
    /// reflections) — the OUTBOX half, replacing the C#/TS `SendL3Rtt` sink.
    outbound: Vec<(Callsign, Inp3L3RttFrame)>,

    /// Neighbour-down signals queued since the last drain — replacing the C#/TS
    /// `NeighbourDown` event.
    neighbour_down: Vec<Inp3NeighbourDownEvent>,
}

impl Inp3Engine {
    /// Construct the engine for a node. The C#/TS optional self-driving tick timer is
    /// dropped — the host drives [`tick`](Self::tick) from its own interval (the
    /// core has no ambient timers), exactly as it drives the circuit manager.
    ///
    /// `local_node` is our own L3 callsign (settable later via
    /// [`set_local_node`](Self::set_local_node)); `options` supplies the cadence,
    /// reset window, SNTT gain, and advertised capability. The caller is expected to
    /// have validated `options` ([`NetRomInp3Options::validate`]) before
    /// construction, as the C#/TS constructors do.
    pub fn new(local_node: Callsign, options: NetRomInp3Options) -> Self {
        Self {
            local_node,
            options,
            cadence_ms: options.l3_rtt_interval_ms as u64,
            reset_window_ms: options.l3_rtt_reset_window_ms as u64,
            neighbours: Vec::new(),
            outbound: Vec::new(),
            neighbour_down: Vec::new(),
        }
    }

    /// Set the local node callsign stamped into the L3 origin of probes built
    /// *after* this call, and the target of the reflection self-test. Mirrors the C#
    /// `SetLocalNode`.
    pub fn set_local_node(&mut self, node: Callsign) {
        self.local_node = node;
    }

    /// Register / refresh awareness of an interlink neighbour (e.g. when an
    /// interlink session is established, or a NODES neighbour is learned). Creates
    /// the per-neighbour state with a fresh reset window if new; a no-op refresh if
    /// already known. Probing begins on the next due [`tick`](Self::tick) (once the
    /// neighbour is known INP3-capable, or immediately if
    /// [`NetRomInp3Options::probe_unknown_capability`]). `now_ms` is the caller's
    /// monotonic ms tick. Mirrors the C# `ObserveNeighbour`.
    pub fn observe_neighbour(&mut self, neighbour: Callsign, now_ms: u64) {
        self.ensure_neighbour(neighbour, now_ms);
    }

    /// Drop a neighbour the host knows is gone (interlink torn down for non-INP3
    /// reasons). Removes its INP3 state; **no** neighbour-down event is enqueued (the
    /// host already knows). Idempotent — dropping an unknown neighbour is a no-op.
    /// Mirrors the C# `RemoveNeighbour`.
    pub fn remove_neighbour(&mut self, neighbour: &Callsign) {
        self.neighbours.retain(|n| n.neighbour != *neighbour);
    }

    /// Advance the engine by one tick: (a) for each neighbour due a probe
    /// (capability-permitted, not awaiting a reflection, and cadence elapsed since
    /// the last send) enqueue an outbound L3RTT probe and stamp the send; (b) for
    /// each neighbour silent past the reset window, reset it — enqueuing an
    /// [`Inp3NeighbourDownEvent`] **only** if it was INP3-capable (the
    /// AMBIGUITY-I2-3 guard). Drains via [`take_outbound_l3rtt`](Self::take_outbound_l3rtt)
    /// / [`take_neighbour_down`](Self::take_neighbour_down). `now_ms` is the caller's
    /// monotonic ms tick. Mirrors the C# `Tick`.
    pub fn tick(&mut self, now_ms: u64) {
        // Reset wins over probe for the same neighbour in the same tick: handle the
        // teardowns first (removing entries), then probe whatever survives. Collect
        // teardowns first so the borrow of `neighbours` is released before mutating
        // the outbound/neighbour_down vecs.
        let mut downs: Vec<Inp3NeighbourDownEvent> = Vec::new();
        self.neighbours.retain(|n| {
            // saturating_sub: with the monotonic-from-construction clock `now_ms`
            // never precedes `last_reflection_ms`, but a defensive saturating
            // subtraction keeps the comparison total even if a caller passes a clock
            // that stepped backwards (it would simply read as "0 ms silent").
            let silent_for = now_ms.saturating_sub(n.last_reflection_ms);
            if silent_for > self.reset_window_ms {
                if n.inp3_capable {
                    downs.push(Inp3NeighbourDownEvent {
                        neighbour: n.neighbour,
                        silent_for_ms: silent_for,
                    });
                }
                // else: a never-capable vanilla neighbour that never reflected our
                // optimistic probes — drop it silently, NO neighbour-down (the
                // AMBIGUITY-I2-3 guard).
                false // remove from the table
            } else {
                true // survives; eligible to be probed below
            }
        });
        self.neighbour_down.append(&mut downs);

        let probe_unknown = self.options.probe_unknown_capability;
        let mut probes: Vec<(Callsign, Inp3L3RttFrame)> = Vec::new();
        for n in &mut self.neighbours {
            let may_probe = n.inp3_capable || probe_unknown;
            let cadence_elapsed = n.last_l3rtt_sent_ms == NEVER_PROBED
                || now_ms.saturating_sub(n.last_l3rtt_sent_ms) >= self.cadence_ms;
            if may_probe && !n.awaiting_reflection && cadence_elapsed {
                if let Some(frame) = Inp3L3RttFrame::build(
                    self.local_node,
                    self.options.advertise_ip_accept,
                ) {
                    n.last_l3rtt_sent_ms = now_ms;
                    n.awaiting_reflection = true;
                    probes.push((n.neighbour, frame));
                }
                // If the frame fails to build (an out-of-range advertised IP, which
                // the validated options preclude), we simply do not probe this tick —
                // we never panic in a build path.
            }
        }
        self.outbound.append(&mut probes);
    }

    /// Feed an inbound L3RTT frame received from `neighbour` on the interlink (the
    /// caller already recognised it as L3RTT). Two cases:
    ///
    /// - If it is a reflection of *our* probe ([`Inp3L3RttFrame::is_reflection_of`]
    ///   with our local node, and we were awaiting one from this neighbour): compute
    ///   RTT, feed `RTT / 2` to the SNTT smoother, stamp the reflection, clear the
    ///   outstanding-probe flag, and learn the (echoed) capability.
    /// - Otherwise it is a peer's probe to us: reflect it verbatim (enqueue it for
    ///   [`take_outbound_l3rtt`](Self::take_outbound_l3rtt)), and learn the peer's
    ///   `$N` / `$IX` capability from it.
    ///
    /// Never panics. `now_ms` is the caller's monotonic ms tick. Mirrors the C#
    /// `OnL3Rtt(Callsign, Inp3L3RttFrame)`.
    pub fn on_l3rtt(&mut self, neighbour: Callsign, frame: Inp3L3RttFrame, now_ms: u64) {
        let gain_shift = self.options.sntt_gain_shift as u32;
        let local_node = self.local_node;

        self.ensure_neighbour(neighbour, now_ms);
        // ensure_neighbour guarantees the entry exists, so this find always succeeds.
        let n = match self.neighbours.iter_mut().find(|n| n.neighbour == neighbour) {
            Some(n) => n,
            None => return,
        };

        // Learn capability from whatever flags the frame carries (both directions
        // advertise capability; design §2.3).
        if frame.inp3_capable {
            n.inp3_capable = true;
        }
        if let Some(ip) = frame.ip_accept {
            n.ip_accept = Some(ip);
        }

        if frame.is_reflection_of(&local_node) && n.awaiting_reflection {
            // Our probe came back. The reflection itself proves liveness.
            let rtt = now_ms.saturating_sub(n.last_l3rtt_sent_ms);
            n.awaiting_reflection = false;
            n.last_reflection_ms = now_ms;

            // A non-negative sample (= RTT/2) is clamped to the INP3 horizon on the
            // wide `u64` before narrowing — a pathological RTT whose RTT/2 exceeds
            // `u32` range would otherwise wrap and present as a small sample
            // (under-reporting the link). Clamp to the horizon first so the narrowing
            // is always lossless. (The C#/TS "negative RTT contributes no sample"
            // branch is structurally impossible here: `now_ms` is an unsigned
            // monotonic clock and `saturating_sub` floors at 0, so `rtt >= 0` always
            // holds — the clock-went-backwards case maps to a 0-ms sample, which is a
            // legitimate same-host loopback measurement, not a corruption.)
            let half = rtt / 2;
            let sample = if half > SNTT_SAMPLE_MAX_MS as u64 {
                SNTT_SAMPLE_MAX_MS
            } else {
                half as u32
            };
            n.sntt_ms = inp3_sntt::smooth(n.sntt_ms, sample, gain_shift);
        } else {
            // A peer's probe to us (origin != us, or we weren't awaiting a reflection
            // — an unsolicited / duplicate reflection is treated as a peer probe,
            // never as a metric sample). Reflect it verbatim (i1-wire-spec §1.4
            // locked byte-for-byte echo) — enqueue the SAME frame back unchanged.
            self.outbound.push((neighbour, frame));
        }
    }

    /// Feed a raw [`NetRomPacket`] received from `neighbour`: if it is an L3RTT frame
    /// the engine recognises and processes it (as
    /// [`on_l3rtt`](Self::on_l3rtt)) and returns `true`; otherwise it returns `false`
    /// with no state change (the packet is something else the caller should route
    /// elsewhere). Never panics. `now_ms` is the caller's monotonic ms tick. Mirrors
    /// the C# `OnL3Rtt(Callsign, NetRomPacket)`.
    pub fn on_l3rtt_packet(
        &mut self,
        neighbour: Callsign,
        packet: &NetRomPacket<'_>,
        now_ms: u64,
    ) -> bool {
        match Inp3L3RttFrame::try_from_packet(packet) {
            Some(frame) => {
                self.on_l3rtt(neighbour, frame, now_ms);
                true
            }
            None => false,
        }
    }

    /// Drain the outbound L3RTT sends accumulated since the last call (probes the
    /// engine originated + verbatim reflections of a peer's probe) — the TAKE half of
    /// the OUTBOX/TAKE pattern, replacing the C#/TS `SendL3Rtt` sink. Each tuple is
    /// `(neighbour, frame)`: the host wraps `frame.to_bytes()` in a PID-0xCF I-frame
    /// on that neighbour's interlink session.
    ///
    /// Mirrors [`CircuitManager::take_outbox`](super::circuit_manager::CircuitManager::take_outbox).
    pub fn take_outbound_l3rtt(&mut self) -> Vec<(Callsign, Inp3L3RttFrame)> {
        core::mem::take(&mut self.outbound)
    }

    /// Drain the neighbour-down signals accumulated since the last call — replacing
    /// the C#/TS `NeighbourDown` event. Each is a previously-INP3-capable neighbour
    /// that went silent past the reset window; the host wires it to
    /// `NetRomRoutingTable::mark_neighbour_down` + a DISC / re-establish.
    pub fn take_neighbour_down(&mut self) -> Vec<Inp3NeighbourDownEvent> {
        core::mem::take(&mut self.neighbour_down)
    }

    /// An immutable snapshot of per-neighbour timing state, for surfacing / tests.
    /// Stable ordering by callsign (base then SSID, the ordinal discipline) so the
    /// surfaced output is deterministic. `now_ms` is the caller's monotonic ms tick
    /// (for the `last_reflection_age_ms` projection). Mirrors the C# `Neighbours`
    /// property.
    pub fn neighbours(&self, now_ms: u64) -> Vec<Inp3NeighbourTiming> {
        let mut out: Vec<Inp3NeighbourTiming> = self
            .neighbours
            .iter()
            .map(|n| Inp3NeighbourTiming {
                neighbour: n.neighbour,
                sntt_ms: if n.sntt_initialised() {
                    Some(n.sntt_ms)
                } else {
                    None
                },
                inp3_capable: n.inp3_capable,
                ip_accept: n.ip_accept,
                last_reflection_age_ms: now_ms.saturating_sub(n.last_reflection_ms),
                awaiting_reflection: n.awaiting_reflection,
            })
            .collect();
        out.sort_by(|a, b| cmp_callsign(&a.neighbour, &b.neighbour));
        out
    }

    /// The smoothed neighbour transport time (ms) the route layer (I-3) reads for a
    /// neighbour; `None` if the neighbour is unknown or has no measurement yet (still
    /// [`SNTT_UNSET_RAW`]). A pure read. Mirrors the C# `SnttMs`.
    pub fn sntt_ms(&self, neighbour: &Callsign) -> Option<u32> {
        self.neighbours
            .iter()
            .find(|n| n.neighbour == *neighbour)
            .filter(|n| n.sntt_initialised())
            .map(|n| n.sntt_ms)
    }

    /// The INP3-capable neighbour set, callsign-ordered — the set the
    /// [`super::inp3_update_scheduler::Inp3UpdateScheduler`] fans out to and the host
    /// surfaces for monitoring. A neighbour is "capable" once it has advertised `$N`
    /// (proven by receiving its probe / reflection). A pure read.
    pub fn inp3_capable_neighbours(&self) -> Vec<Callsign> {
        let mut out: Vec<Callsign> = self
            .neighbours
            .iter()
            .filter(|n| n.inp3_capable)
            .map(|n| n.neighbour)
            .collect();
        out.sort_by(cmp_callsign);
        out
    }

    // ─── Internals ──────────────────────────────────────────────────────

    /// Get-or-create a neighbour's state. A fresh entry seeds
    /// `last_reflection_ms = now` (a full reset window before it can be torn down)
    /// and `sntt_ms = SNTT_UNSET_RAW` (no measurement). Mirrors the C#
    /// `EnsureNeighbour`.
    fn ensure_neighbour(&mut self, neighbour: Callsign, now_ms: u64) {
        if self.neighbours.iter().any(|n| n.neighbour == neighbour) {
            return;
        }
        self.neighbours.push(Inp3NeighbourState {
            neighbour,
            sntt_ms: SNTT_UNSET_RAW,
            last_l3rtt_sent_ms: NEVER_PROBED,
            last_reflection_ms: now_ms,
            inp3_capable: false,
            ip_accept: None,
            awaiting_reflection: false,
        });
    }
}

/// Ordinal callsign comparison: base bytes then SSID — the analogue of the C#
/// snapshot's `StringComparer.Ordinal` over `callsign.ToString()` (and the
/// scheduler's / routing table's `cmp_callsign`), giving a deterministic snapshot /
/// fan-out order.
fn cmp_callsign(a: &Callsign, b: &Callsign) -> core::cmp::Ordering {
    a.base().cmp(b.base()).then_with(|| a.ssid().cmp(&b.ssid()))
}

#[cfg(test)]
mod tests {
    //! Deterministic tests for the INP3 link-timing engine (slice I-2), ported 1:1
    //! from `tests/Packet.NetRom.Tests/Transport/Inp3EngineTests.cs` and
    //! cross-checked against the merged TS `inp3-engine.ts`. The C# drives a
    //! `FakeTimeProvider`; the core takes `now_ms` per call, so each test advances an
    //! explicit monotonic clock instead. These are the faithfulness oracle.

    use super::*;
    use crate::netrom::wire::network_header::NetRomNetworkHeader;
    use crate::netrom::wire::transport_header::NetRomTransportHeader;

    fn local() -> Callsign {
        Callsign::new(b"GB7PDN", 0).unwrap()
    }

    fn peer() -> Callsign {
        Callsign::new(b"GB7RDG", 0).unwrap()
    }

    /// The C# test options: 60 s probe interval, 180 s reset window, otherwise
    /// defaults.
    fn opts_60_180() -> NetRomInp3Options {
        NetRomInp3Options {
            l3_rtt_interval_ms: 60_000,
            l3_rtt_reset_window_ms: 180_000,
            ..NetRomInp3Options::DEFAULT
        }
    }

    fn new_engine(options: NetRomInp3Options) -> Inp3Engine {
        Inp3Engine::new(local(), options)
    }

    #[test]
    fn probe_fires_on_cadence_and_not_before() {
        let mut e = new_engine(opts_60_180());
        e.observe_neighbour(peer(), 0);

        // First tick: never-probed neighbour is immediately due (last-sent sentinel).
        e.tick(0);
        let sent = e.take_outbound_l3rtt();
        assert_eq!(sent.len(), 1, "a freshly-observed neighbour is probed on the first tick");
        assert_eq!(sent[0].0, peer());
        // the probe carries our node as L3 origin
        assert_eq!(sent[0].1.network.origin, local());
        assert_eq!(sent[0].1.network.destination.base(), b"L3RTT");

        // A probe is outstanding (awaiting_reflection) — no re-probe even past cadence.
        e.tick(120_000);
        assert!(
            e.take_outbound_l3rtt().is_empty(),
            "a neighbour with a probe in flight is never re-probed"
        );
    }

    #[test]
    fn probe_does_not_re_fire_within_cadence_after_reflection() {
        let mut e = new_engine(opts_60_180());
        e.observe_neighbour(peer(), 0);
        e.tick(0); // probe #1 at t=0
        let sent = e.take_outbound_l3rtt();
        assert_eq!(sent.len(), 1);
        let our_probe = sent[0].1.clone();

        // Reflect it 1 s later so the outstanding-probe flag clears.
        e.on_l3rtt(peer(), our_probe, 1_000);
        let _ = e.take_outbound_l3rtt();

        // 30 s after probe #1 (< 60 s cadence) → no new probe.
        e.tick(30_000);
        assert!(
            e.take_outbound_l3rtt().is_empty(),
            "cadence has not elapsed since the last send"
        );

        // Past the 60 s mark since probe #1 → probe #2 fires.
        e.tick(61_000);
        assert_eq!(
            e.take_outbound_l3rtt().len(),
            1,
            "the next probe fires once the cadence has elapsed"
        );
    }

    #[test]
    fn reflection_of_our_probe_updates_sntt_with_half_the_round_trip() {
        let mut e = new_engine(opts_60_180());
        e.observe_neighbour(peer(), 0);
        e.tick(0); // probe at t=0
        let our_probe = e.take_outbound_l3rtt()[0].1.clone();

        assert_eq!(e.sntt_ms(&peer()), None, "no measurement before the first reflection");

        // Reflection arrives 400 ms later → RTT = 400, sample = RTT/2 = 200; the
        // first sample seeds the filter directly (SRT/Karn cold-start).
        e.on_l3rtt(peer(), our_probe, 400);

        assert_eq!(e.sntt_ms(&peer()), Some(200), "first reflection seeds SNTT = RTT/2");

        let timing = e.neighbours(400);
        assert_eq!(timing.len(), 1);
        assert_eq!(timing[0].neighbour, peer());
        assert_eq!(timing[0].sntt_ms, Some(200));
        assert!(
            !timing[0].awaiting_reflection,
            "the outstanding-probe flag cleared on reflection"
        );
    }

    #[test]
    fn a_peer_probe_is_reflected_verbatim() {
        let mut e = new_engine(NetRomInp3Options::DEFAULT);

        // A probe ORIGINATED BY THE PEER (its origin is the peer, not us) — we must
        // echo it back byte-for-byte, not treat it as a reflection / SNTT sample.
        let peer_probe = Inp3L3RttFrame::build(peer(), None).unwrap();
        e.on_l3rtt(peer(), peer_probe.clone(), 0);

        let sent = e.take_outbound_l3rtt();
        assert_eq!(sent.len(), 1, "a peer's probe is reflected back to it");
        assert_eq!(sent[0].0, peer());
        // reflection is verbatim — the same frame goes back unchanged
        assert_eq!(sent[0].1, peer_probe);
        // verbatim echo keeps the peer as the origin
        assert_eq!(sent[0].1.network.origin, peer());

        assert_eq!(
            e.sntt_ms(&peer()),
            None,
            "reflecting a peer's probe is not a measurement of our own RTT"
        );
    }

    #[test]
    fn capability_is_learned_from_a_peer_probe() {
        let mut e = new_engine(NetRomInp3Options::DEFAULT);

        // The peer probes us with $N and $I4 — we learn it speaks INP3 + accepts IPv4.
        let peer_probe = Inp3L3RttFrame::build(peer(), Some(4)).unwrap();
        assert!(peer_probe.inp3_capable);
        assert_eq!(peer_probe.ip_accept, Some(4));

        e.on_l3rtt(peer(), peer_probe, 0);

        let timing = e.neighbours(0);
        assert_eq!(timing.len(), 1);
        assert!(timing[0].inp3_capable, "a peer's $N probe proves it speaks INP3");
        assert_eq!(timing[0].ip_accept, Some(4), "its $I4 token advertises IPv4 acceptance");

        // It is now in the capable set.
        assert_eq!(e.inp3_capable_neighbours(), alloc::vec![peer()]);
    }

    #[test]
    fn reset_window_with_no_reflection_fires_neighbour_down_for_a_capable_neighbour_and_resets_it() {
        let mut e = new_engine(opts_60_180());

        // The peer proves it speaks INP3 (so the 180 s reset is allowed to raise
        // neighbour-down — the AMBIGUITY-I2-3 guard).
        e.on_l3rtt(peer(), Inp3L3RttFrame::build(peer(), None).unwrap(), 0);
        let timing = e.neighbours(0);
        assert_eq!(timing.len(), 1);
        assert!(timing[0].inp3_capable);
        let _ = e.take_outbound_l3rtt(); // discard the reflection we sent

        // It then goes silent. Just under the window → no reset yet.
        e.tick(179_000);
        assert!(
            e.take_neighbour_down().is_empty(),
            "179 s of silence is within the 180 s reset window"
        );
        assert_eq!(e.neighbours(179_000).len(), 1, "the neighbour is still tracked");
        let _ = e.take_outbound_l3rtt(); // discard probes fired meanwhile

        // Past the window → neighbour-down fires and the state is reset (removed).
        e.tick(181_000); // t = 181 s of silence
        let downs = e.take_neighbour_down();
        assert_eq!(
            downs.len(),
            1,
            "an INP3-capable neighbour that went silent raises neighbour-down"
        );
        assert_eq!(downs[0].neighbour, peer());
        assert!(
            downs[0].silent_for_ms >= 180_000,
            "it was silent at least the reset window"
        );

        assert!(
            e.neighbours(181_000).is_empty(),
            "the neighbour's INP3 state is reset (removed) on teardown"
        );
        assert_eq!(e.sntt_ms(&peer()), None, "a reset neighbour has no SNTT");
    }

    #[test]
    fn a_never_capable_vanilla_neighbour_is_dropped_silently_without_neighbour_down() {
        // The AMBIGUITY-I2-3 guard: a neighbour that never reflects our optimistic
        // probes (never proven INP3-capable) must NOT trigger a routing teardown — it
        // is reachable by vanilla NODES, it just doesn't speak L3RTT. After the reset
        // window it is dropped from probing silently, no event.
        let opts = NetRomInp3Options {
            probe_unknown_capability: true,
            ..opts_60_180()
        };
        let mut e = new_engine(opts);

        e.observe_neighbour(peer(), 0);
        e.tick(0); // optimistic probe fires (capability unknown)
        assert_eq!(
            e.take_outbound_l3rtt().len(),
            1,
            "probe_unknown_capability probes a not-yet-known neighbour"
        );
        let timing = e.neighbours(0);
        assert_eq!(timing.len(), 1);
        assert!(!timing[0].inp3_capable);

        // It never reflects. Past the reset window it is dropped — silently.
        e.tick(181_000);
        assert!(
            e.take_neighbour_down().is_empty(),
            "a never-capable vanilla neighbour is never mark_neighbour_down'd"
        );
        assert!(
            e.neighbours(181_000).is_empty(),
            "but it is dropped from probing so we don't probe a vanilla peer forever"
        );
    }

    #[test]
    fn conservative_policy_does_not_probe_an_unknown_capability_neighbour() {
        let opts = NetRomInp3Options {
            probe_unknown_capability: false,
            ..opts_60_180()
        };
        let mut e = new_engine(opts);

        e.observe_neighbour(peer(), 0);
        e.tick(0);
        assert!(
            e.take_outbound_l3rtt().is_empty(),
            "with probe_unknown_capability=false we wait to be probed first"
        );

        // Once the peer probes us (proving capability), we start probing it.
        e.on_l3rtt(peer(), Inp3L3RttFrame::build(peer(), None).unwrap(), 0);
        let _ = e.take_outbound_l3rtt(); // discard the reflection
        e.tick(0);
        assert_eq!(
            e.take_outbound_l3rtt().len(),
            1,
            "a now-known-capable neighbour is probed"
        );
    }

    #[test]
    fn reflection_smoothing_follows_the_one_eighth_gain_iir() {
        // Drive a sequence of reflections and assert the SNTT trajectory matches the
        // design §0.5 Example C (steady 200 ms RTT with one 2000 ms spike): the first
        // sample seeds, then SNTT' = (7*SNTT + sample + 4)/8. A wide reset window so
        // nothing tears down across the run.
        let opts = NetRomInp3Options {
            l3_rtt_interval_ms: 60_000,
            l3_rtt_reset_window_ms: 600_000,
            ..NetRomInp3Options::DEFAULT
        };
        let mut e = new_engine(opts);
        e.observe_neighbour(peer(), 0);

        // (RTT ms, expected SNTT after); sample = RTT/2.
        let steps: [(u64, u32); 5] = [
            (200, 100),  // seed = 100
            (200, 100),  // (7*100+100+4)/8 = 100
            (2000, 213), // (7*100+1000+4)/8 = 213 (the spike)
            (200, 199),  // (7*213+100+4)/8 = 199
            (200, 187),  // (7*199+100+4)/8 = 187 (walking the outlier back)
        ];

        // The clock advances past the 60 s cadence each loop so every tick probes.
        let mut now: u64 = 0;
        for (rtt_ms, expected) in steps {
            e.tick(now); // emit a probe (cadence has elapsed each loop)
            let probe = e.take_outbound_l3rtt();
            assert_eq!(probe.len(), 1, "a probe fires each loop");
            let frame = probe[0].1.clone();
            now += rtt_ms;
            e.on_l3rtt(peer(), frame, now);
            assert_eq!(
                e.sntt_ms(&peer()),
                Some(expected),
                "RTT {rtt_ms} ms => sample {} smoothed",
                rtt_ms / 2
            );
            // Advance past the cadence so the next loop's tick probes again.
            now += 60_000;
        }
    }

    #[test]
    fn on_l3rtt_with_a_non_l3rtt_packet_returns_false_and_does_nothing() {
        let mut e = new_engine(NetRomInp3Options::DEFAULT);

        // A real Information datagram to us — not L3RTT (wrong destination).
        let not_l3rtt = NetRomPacket {
            network: NetRomNetworkHeader {
                origin: peer(),
                destination: local(),
                time_to_live: 10,
            },
            transport: NetRomTransportHeader {
                circuit_index: 1,
                circuit_id: 1,
                tx_sequence: 0,
                rx_sequence: 0,
                opcode: 0x05, // Information
                flags: 0,
            },
            payload: &[1, 2, 3],
        };

        assert!(
            !e.on_l3rtt_packet(peer(), &not_l3rtt, 0),
            "a non-L3RTT packet is not ours to handle"
        );
        assert!(e.take_outbound_l3rtt().is_empty());
        assert!(
            e.neighbours(0).is_empty(),
            "a non-L3RTT packet creates no neighbour state"
        );
    }

    #[test]
    fn on_l3rtt_packet_recognises_and_processes_an_l3rtt_datagram() {
        // The raw-packet path: an L3RTT datagram from the peer is recognised, learns
        // capability, and is reflected verbatim — returns true.
        let mut e = new_engine(NetRomInp3Options::DEFAULT);
        let peer_probe = Inp3L3RttFrame::build(peer(), Some(4)).unwrap();
        let packet = peer_probe.packet();

        assert!(
            e.on_l3rtt_packet(peer(), &packet, 0),
            "an L3RTT datagram is recognised and handled"
        );
        let sent = e.take_outbound_l3rtt();
        assert_eq!(sent.len(), 1, "the peer's probe is reflected");
        assert_eq!(sent[0].0, peer());
        let timing = e.neighbours(0);
        assert_eq!(timing.len(), 1);
        assert!(timing[0].inp3_capable);
        assert_eq!(timing[0].ip_accept, Some(4));
    }

    #[test]
    fn remove_neighbour_after_a_down_event_is_safe_and_idempotent() {
        // The host drains take_neighbour_down() then calls remove_neighbour for each.
        // The engine already removed the neighbour on teardown, so this is a no-op
        // (idempotent) — it must not panic.
        let mut e = new_engine(opts_60_180());
        e.on_l3rtt(peer(), Inp3L3RttFrame::build(peer(), None).unwrap(), 0); // capable
        let _ = e.take_outbound_l3rtt();

        e.tick(181_000);
        let downs = e.take_neighbour_down();
        assert_eq!(downs.len(), 1);
        // Re-entrant-style host cleanup: drop the already-removed neighbour.
        e.remove_neighbour(&downs[0].neighbour);
        assert!(e.neighbours(181_000).is_empty());
    }

    #[test]
    fn an_unsolicited_reflection_when_not_awaiting_is_treated_as_a_peer_probe() {
        // A frame whose origin is OUR node but we are not awaiting a reflection (e.g.
        // a duplicate / late echo) is reflected verbatim, never folded as a sample.
        let mut e = new_engine(NetRomInp3Options::DEFAULT);
        // Build a frame with our own origin, but the engine has no outstanding probe.
        let stray = Inp3L3RttFrame::build(local(), None).unwrap();
        e.on_l3rtt(peer(), stray, 0);

        let sent = e.take_outbound_l3rtt();
        assert_eq!(
            sent.len(),
            1,
            "an unsolicited reflection (not awaiting) is treated as a peer probe and reflected"
        );
        assert_eq!(e.sntt_ms(&peer()), None, "and contributes no SNTT sample");
    }

    #[test]
    fn never_probed_sentinel_distinguishes_a_probe_genuinely_sent_at_t_zero() {
        // The load-bearing sentinel: a probe sent at now_ms=0 must NOT read as
        // never-sent. After probing at t=0, a tick still at t=0 must not re-probe
        // (cadence has not elapsed), and a probe is in flight.
        let mut e = new_engine(opts_60_180());
        e.observe_neighbour(peer(), 0);
        e.tick(0);
        assert_eq!(e.take_outbound_l3rtt().len(), 1, "probed once at t=0");
        // Same clock, fresh tick: not re-probed (a probe at t=0 is recorded, not
        // mistaken for never-probed), and one is still in flight.
        e.tick(0);
        assert!(
            e.take_outbound_l3rtt().is_empty(),
            "a probe genuinely sent at t=0 is not re-fired as if never-probed"
        );
    }
}
