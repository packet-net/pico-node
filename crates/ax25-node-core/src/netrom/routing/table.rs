//! The learned NET/ROM routing table — fixed-capacity, allocator-free.
//!
//! Ports `Packet.NetRom.Routing.NetRomRoutingTable`. It ingests NODES broadcasts
//! heard promiscuously, derives route qualities via the multiplicative per-hop
//! formula, keeps the best routes per destination with obsolescence decay, and
//! exposes the learned state for surfacing.
//!
//! **Read-only by construction.** The table is a pure consumer of heard broadcasts
//! — it transmits nothing, originates no NODES, opens no circuits. It implements
//! the canonical processing heuristics from the NET/ROM appendix:
//!
//! 1. A heard broadcast's originator becomes a directly-heard *neighbour*, created
//!    with the configured default-port path quality if not already known
//!    (heuristic 3 + 4).
//! 2. A **direct route to the originator** is assumed at the neighbour's path
//!    quality (heuristic 4).
//! 3. For each advertised destination, the route quality *via that neighbour* is
//!    the advertised quality combined with the path quality
//!    ([`crate::netrom::routing::quality::combine`], heuristic 5).
//! 4. **Trivial-loop guard**: if the advertised best-neighbour is our own
//!    callsign, the route is quality 0 — a last resort that is never kept
//!    (heuristic 6).
//! 5. Only the `MAX_ROUTES` best routes per destination are kept (heuristic 7).
//! 6. Routes at or below quality 0, or below
//!    [`NetRomRoutingOptions::min_quality`], are dropped (heuristic 8).
//! 7. Destinations stop being added once `MAX_DESTS` is reached (heuristic 9).
//!
//! **Obsolescence.** A route's count is (re)set to
//! [`NetRomRoutingOptions::obsolete_initial`] whenever a broadcast adds/refreshes
//! it. [`NetRomRoutingTable::sweep`] (called at the broadcast interval) decrements
//! every count and purges routes that reach 0; a destination with no remaining
//! routes is removed, and an orphaned neighbour is dropped.
//!
//! **`no_std` / capacity.** Unlike the desktop's unbounded `Dictionary`s, this is a
//! `[Option<…>; N]` of compile-time-sized arrays — no heap, no allocation. The caps
//! are const generics: `MAX_DESTS` destinations, `MAX_ROUTES` routes per
//! destination (canonical 3), `MAX_NBRS` neighbours. A `u64` tick is supplied by
//! the caller for last-heard stamps (the embedding's monotonic time — there is no
//! wall-clock here).

use crate::ax25::Callsign;

use super::model::{Inp3RouteMetric, NetRomDestination, NetRomNeighbour, NetRomRoute};
use super::options::NetRomRoutingOptions;
use super::quality;
use crate::netrom::wire::inp3_rif::{Inp3Rif, Inp3Rip};
use crate::netrom::wire::{Alias, NodesAdvertisementEntry, NodesBroadcast};
use crate::netrom::PortId;
use super::inp3_sntt::SNTT_UNSET_RAW;
use alloc::vec::Vec;

/// One kept route inside a destination's route set.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct RouteState {
    neighbour: Callsign,
    quality: u8,
    obsolescence: u8,
    /// The INP3 measured-time metric for this route, when one has been learned via a
    /// RIF ([`NetRomRoutingTable::ingest_rif`]). `None` on a pure NODES quality route.
    /// The second metric space, independent of `quality` (design AMBIGUITY-I3-2).
    inp3: Option<Inp3RouteMetric>,
}

/// A destination's state: its alias and a fixed array of up to `MAX_ROUTES` routes.
#[derive(Debug, Clone, Copy)]
struct DestinationState<const MAX_ROUTES: usize> {
    destination: Callsign,
    alias: Alias,
    routes: [Option<RouteState>; MAX_ROUTES],
}

impl<const MAX_ROUTES: usize> DestinationState<MAX_ROUTES> {
    fn route_index(&self, neighbour: &Callsign) -> Option<usize> {
        self.routes
            .iter()
            .position(|r| r.is_some_and(|rt| rt.neighbour == *neighbour))
    }

    fn route_count(&self) -> usize {
        self.routes.iter().filter(|r| r.is_some()).count()
    }

    // The index of the worst kept route by (quality asc, neighbour callsign desc) —
    // i.e. the eviction candidate. `None` only if there are no routes.
    fn worst_route_index(&self) -> Option<usize> {
        let mut worst: Option<(usize, RouteState)> = None;
        for (i, slot) in self.routes.iter().enumerate() {
            if let Some(rt) = slot {
                let replace = match worst {
                    None => true,
                    Some((_, w)) => is_worse(rt, &w),
                };
                if replace {
                    worst = Some((i, *rt));
                }
            }
        }
        worst.map(|(i, _)| i)
    }

    fn first_free(&self) -> Option<usize> {
        self.routes.iter().position(Option::is_none)
    }

    // The best route by the snapshot ordering (quality desc, neighbour callsign asc).
    fn best_route(&self) -> Option<RouteState> {
        let mut best: Option<RouteState> = None;
        for slot in self.routes.iter().flatten() {
            let replace = match best {
                None => true,
                Some(b) => is_better(slot, &b),
            };
            if replace {
                best = Some(*slot);
            }
        }
        best
    }
}

/// A directly-heard neighbour's state.
#[derive(Debug, Clone, Copy)]
struct NeighbourState {
    neighbour: Callsign,
    alias: Alias,
    port_id: PortId,
    path_quality: u8,
    last_heard: u64,
}

/// The fixed-capacity learned NET/ROM routing table.
///
/// `MAX_DESTS` distinct destinations, `MAX_ROUTES` routes per destination
/// (canonical 3), `MAX_NBRS` directly-heard neighbours. See the module docs.
#[derive(Debug)]
pub struct NetRomRoutingTable<
    const MAX_DESTS: usize,
    const MAX_ROUTES: usize,
    const MAX_NBRS: usize,
> {
    options: NetRomRoutingOptions,
    destinations: [Option<DestinationState<MAX_ROUTES>>; MAX_DESTS],
    neighbours: [Option<NeighbourState>; MAX_NBRS],

    /// INP3 invariant (W): destinations that have lost their LAST `inp3`-bearing
    /// route (withdrawn at horizon in [`ingest_rif`], dropped by
    /// [`mark_neighbour_down`], or aged out by [`sweep`]) since the host last
    /// [`drain_recently_withdrawn`]ed this set. The host drains it ONCE at the start
    /// of each fan-out round and hands the snapshot to every neighbour's
    /// [`build_rif`], so the one-shot horizon RIP reaches each neighbour exactly once.
    /// Populated ONLY when an `inp3`-bearing route fully leaves — so a vanilla
    /// (quality-only) `mark_neighbour_down` / `sweep`, the INP3-off path, never
    /// touches it (the load-bearing default-off guarantee, design §7.1). A heap `Vec`
    /// (kept distinct + emptied on drain) is the `no_std` equivalent of the C#
    /// `HashSet<Callsign>` — a node's withdrawn set is tiny, so a linear-scan dedup
    /// is faithful and allocation-light.
    ///
    /// [`ingest_rif`]: Self::ingest_rif
    /// [`mark_neighbour_down`]: Self::mark_neighbour_down
    /// [`sweep`]: Self::sweep
    /// [`drain_recently_withdrawn`]: Self::drain_recently_withdrawn
    /// [`build_rif`]: Self::build_rif
    recently_withdrawn: Vec<Callsign>,
}

impl<const MAX_DESTS: usize, const MAX_ROUTES: usize, const MAX_NBRS: usize>
    NetRomRoutingTable<MAX_DESTS, MAX_ROUTES, MAX_NBRS>
{
    /// Construct a table with the given options. All slots start free.
    pub fn new(options: NetRomRoutingOptions) -> Self {
        Self {
            options,
            destinations: [None; MAX_DESTS],
            neighbours: [None; MAX_NBRS],
            recently_withdrawn: Vec::new(),
        }
    }

    /// Construct a table with the canonical default options.
    pub fn with_defaults() -> Self {
        Self::new(NetRomRoutingOptions::DEFAULT)
    }

    /// The maximum routes retained per destination (the const-generic cap).
    pub const fn max_routes_per_destination(&self) -> usize {
        MAX_ROUTES
    }

    /// The maximum number of distinct destinations (the const-generic cap).
    pub const fn max_destinations(&self) -> usize {
        MAX_DESTS
    }

    /// Number of destinations currently known.
    pub fn destination_count(&self) -> usize {
        self.destinations.iter().filter(|d| d.is_some()).count()
    }

    /// Number of directly-heard neighbours currently known.
    pub fn neighbour_count(&self) -> usize {
        self.neighbours.iter().filter(|n| n.is_some()).count()
    }

    /// Ingest a NODES broadcast heard from `originator` on `port_id`, with this
    /// node's own callsign `my_call` (for the trivial-loop guard) and the caller's
    /// monotonic `now` tick (for the neighbour's last-heard stamp). Pure table
    /// maintenance — never transmits.
    pub fn ingest(
        &mut self,
        originator: Callsign,
        my_call: Callsign,
        port_id: PortId,
        broadcast: &NodesBroadcast,
        now: u64,
    ) {
        let path_quality = self.options.default_neighbour_quality;

        // Heuristic 3: ensure a neighbour-list entry for the originator, created
        // with the default-port path quality. Refresh its alias + last-heard each
        // time. (If the neighbour table is full and the originator is new, we still
        // proceed to maintain routes — the originator just isn't tracked as a
        // directly-heard neighbour; its assumed direct route below keeps it as a
        // destination.)
        let originator_path_quality = match self.neighbour_index(&originator) {
            Some(i) => {
                let nbr = self.neighbours[i].as_mut().expect("indexed slot present");
                nbr.alias = broadcast.sender_alias();
                nbr.port_id = port_id;
                nbr.last_heard = now;
                nbr.path_quality
            }
            None => {
                if let Some(free) = self.neighbours.iter().position(Option::is_none) {
                    self.neighbours[free] = Some(NeighbourState {
                        neighbour: originator,
                        alias: broadcast.sender_alias(),
                        port_id,
                        path_quality,
                        last_heard: now,
                    });
                }
                path_quality
            }
        };

        // Heuristic 4: assume a direct route to the originator at the neighbour path
        // quality.
        self.upsert_route(
            originator,
            broadcast.sender_alias(),
            originator,
            originator_path_quality,
        );

        // Heuristic 5/6/7/8: each advertised destination becomes a route via this
        // neighbour at the combined quality, loop-guarded against us.
        for entry in broadcast.entries() {
            let q = if entry.best_neighbour == my_call {
                quality::MIN // trivial-loop guard
            } else {
                quality::combine(entry.best_quality, originator_path_quality)
            };
            self.upsert_route(entry.destination, entry.destination_alias, originator, q);
        }
    }

    /// Decrement the obsolescence count of every route, purging routes that reach 0
    /// and destinations that lose all their routes. Call this at the NODES broadcast
    /// interval. Neighbours with no surviving route are also dropped. Returns the
    /// number of routes purged.
    pub fn sweep(&mut self) -> usize {
        let mut purged = 0usize;
        // Invariant (W): destinations whose LAST inp3-bearing route aged out this sweep.
        // Collected first, recorded after the borrow on `self.destinations` ends.
        let mut withdrawn: Vec<Callsign> = Vec::new();
        for slot in self.destinations.iter_mut() {
            if let Some(dest) = slot {
                // Did this destination hold an inp3-bearing route before the sweep?
                // (Pre-mutation predicate, guarding the default-off behaviour: a
                // quality-only sweep never records anything — design §7.1.)
                let had_inp3_before = dest.routes.iter().flatten().any(|rt| rt.inp3.is_some());

                for route in dest.routes.iter_mut() {
                    if let Some(rt) = route {
                        let next = rt.obsolescence.saturating_sub(1);
                        if next == 0 {
                            *route = None;
                            purged += 1;
                        } else {
                            rt.obsolescence = next;
                        }
                    }
                }

                let has_inp3_after = dest.routes.iter().flatten().any(|rt| rt.inp3.is_some());

                // An inp3-bearing destination whose last time-route aged out this
                // sweep leaves the INP3 space → record the one-shot horizon withdrawal.
                if had_inp3_before && !has_inp3_after {
                    withdrawn.push(dest.destination);
                }

                if dest.route_count() == 0 {
                    *slot = None;
                }
            }
        }
        for dc in withdrawn {
            self.record_recently_withdrawn(dc);
        }
        self.prune_orphan_neighbours();
        purged
    }

    /// React to a neighbour going down — its interlink could not be raised (it did
    /// not answer the connect) or its quality collapsed — by immediately dropping
    /// every route that forwards through it, and the neighbour entry itself. This is
    /// the explicit link-down failover signal: instead of waiting for the
    /// obsolescence [`sweep`](Self::sweep) to age the now-dead routes out over the
    /// broadcast interval (during which forwarding / connect-routing would keep
    /// choosing a route that can't carry traffic), the dead routes leave the table at
    /// once, so the very next forward or connect decision fails over to an alternate
    /// next hop. A destination that loses all its routes is removed; it and the
    /// neighbour re-learn naturally from the next NODES broadcast if the neighbour
    /// returns. Idempotent — marking an unknown / already-removed neighbour down is a
    /// no-op returning 0. Mirrors C# `NetRomRoutingTable.MarkNeighbourDown`.
    ///
    /// Returns the number of routes dropped (across all destinations).
    pub fn mark_neighbour_down(&mut self, neighbour: &Callsign) -> usize {
        let mut dropped = 0usize;
        // Invariant (W): destinations that lose their LAST inp3-bearing route by this
        // drop. Collected, then recorded after the borrow on `self.destinations` ends.
        let mut withdrawn: Vec<Callsign> = Vec::new();
        for slot in self.destinations.iter_mut() {
            if let Some(dest) = slot {
                // Only a removed route that CARRIED an inp3 metric can cost the
                // destination its last time-route — the default-off guard, so a vanilla
                // (quality-only) mark_neighbour_down never records anything (design §7.1).
                let removed_route_had_inp3 = dest
                    .route_index(neighbour)
                    .and_then(|ri| dest.routes[ri])
                    .is_some_and(|rt| rt.inp3.is_some());

                for route in dest.routes.iter_mut() {
                    if let Some(rt) = route {
                        if rt.neighbour == *neighbour {
                            *route = None;
                            dropped += 1;
                        }
                    }
                }

                let still_has_inp3 = dest.routes.iter().flatten().any(|rt| rt.inp3.is_some());
                if removed_route_had_inp3 && !still_has_inp3 {
                    withdrawn.push(dest.destination);
                }

                if dest.route_count() == 0 {
                    *slot = None;
                }
            }
        }
        for dc in withdrawn {
            self.record_recently_withdrawn(dc);
        }
        for slot in self.neighbours.iter_mut() {
            if let Some(nb) = slot {
                if nb.neighbour == *neighbour {
                    *slot = None;
                }
            }
        }
        self.prune_orphan_neighbours();
        dropped
    }

    // ─── INP3 (the measured target-time metric space) ────────────────────

    /// The fixed per-hop target-time increment (ms) added to every learned INP3
    /// time-route so target time is strictly increasing per hop even across a ~0 ms
    /// link — the loop-safety invariant "target time monotonic-nondecreasing per hop".
    /// Mirrors C# `NetRomRoutingTable.PerHopIncrementMs`.
    pub const PER_HOP_INCREMENT_MS: u32 = 10;

    /// The default INP3 hop horizon (canonical 30): a RIP whose learned hop count
    /// would exceed this is not learned — the hop-count analogue of the 600 s time
    /// horizon. Mirrors C# `NetRomRoutingTable.DefaultHopLimit`.
    pub const DEFAULT_HOP_LIMIT: u32 = 30;

    /// Ingest an INP3 [`Inp3Rif`] heard on a connected interlink from
    /// `received_from`, learning a measured *target-time* route (the second metric
    /// space) per RIP. The time-space analogue of [`ingest`](Self::ingest) for the
    /// quality space: it mirrors [`upsert_route`](Self::upsert_route)'s discipline
    /// (per-destination route cap, the trivial-loop guard) and is pure table
    /// maintenance — it never transmits.
    ///
    /// Host-free: the caller (the node host) supplies the smoothed neighbour transport
    /// time `neighbour_sntt_ms` it read from the INP3 engine, exactly as
    /// [`ingest`](Self::ingest) takes `my_call`/`port_id` rather than reaching for them.
    ///
    /// Per-RIP math (design §2.2/§5.2): for each RIP, the local INP3 metric for its
    /// destination *via `received_from`* is
    /// `local_target_time_ms = rip.target_time_ms + neighbour_sntt_ms + 10` (peer
    /// target + this link's measured cost + a fixed [`PER_HOP_INCREMENT_MS`] per-hop
    /// floor) and `local_hop_count = rip.hop_count + 1`.
    ///
    /// **Horizon = withdrawal** (design §2.3): if the RIP is at/over the 600 s horizon
    /// ([`Inp3Rip::is_horizon`]), or the computed `local_target_time_ms` reaches the
    /// horizon, the INP3 metric for `(destination via received_from)` is *withdrawn* —
    /// its [`Inp3RouteMetric`] is cleared, leaving any coexisting quality route intact;
    /// a route then left with neither a usable quality nor an INP3 metric is removed,
    /// and a destination left with no route is removed.
    ///
    /// **Skips** (no learn, no withdraw): a RIP is skipped when the link cost is not
    /// yet measured (`neighbour_sntt_ms == SNTT_UNSET_RAW` — an un-probed link must
    /// never *remove* a time-route it never learned), when `local_hop_count` exceeds
    /// `hop_limit`, or when the destination is `my_call` (the receive-side trivial-loop
    /// guard).
    ///
    /// **Coexistence**: an INP3 upsert only sets the metric on the `(dest via
    /// neighbour)` route, creating it as a pure time-route (quality 0) if none existed,
    /// or attaching the metric to an existing quality route without touching its
    /// quality/obsolescence. The per-destination cap evicts by quality (an INP3-only
    /// route counts as quality 0 for eviction ordering only — design AMBIGUITY-I3-2).
    ///
    /// `hop_limit` values `< 1` are treated as 1. Mirrors C#
    /// `NetRomRoutingTable.IngestRif`.
    pub fn ingest_rif(
        &mut self,
        received_from: Callsign,
        my_call: Callsign,
        neighbour_sntt_ms: u32,
        rif: &Inp3Rif,
        hop_limit: u32,
    ) {
        let effective_hop_limit = hop_limit.max(1);

        // An un-probed link has no measured cost — learn no time-route, and (crucially)
        // withdraw none either: an unset SNTT must never remove a route it never taught.
        let link_measured = neighbour_sntt_ms != SNTT_UNSET_RAW;

        for rip in &rif.rips {
            // local target time = peer target + this link's measured cost + per-hop
            // floor; computed in u64 so the horizon comparison is overflow-free even if
            // the peer advertised right up against the horizon (SNTT ≤ 600_000).
            let local_target_time: u64 =
                rip.target_time_ms as u64 + neighbour_sntt_ms as u64 + Self::PER_HOP_INCREMENT_MS as u64;

            // Horizon = withdrawal (clears the INP3 metric only), independent of the
            // SNTT measurement — a peer advertising the horizon withdraws regardless.
            // The computed-over-horizon case only applies once the link is measured (an
            // unset SNTT would trivially overflow the horizon, which we must NOT treat
            // as a withdrawal — hence the link_measured guard on the second clause).
            if rip.is_horizon()
                || (link_measured && local_target_time >= Inp3Rip::HORIZON_MS as u64)
            {
                self.withdraw_inp3(&rip.destination, &received_from);
                continue;
            }

            if !link_measured {
                continue; // link cost unknown — learn no time-route (and withdrew none)
            }

            let local_hop_count = rip.hop_count as u32 + 1;
            if local_hop_count > effective_hop_limit {
                continue; // hop horizon — path too long to learn
            }

            if rip.destination == my_call {
                continue; // trivial-loop guard: a route to ourselves is never learned
            }

            let metric = Inp3RouteMetric {
                target_time_ms: local_target_time as u32,
                hop_count: local_hop_count.min(u8::MAX as u32) as u8,
            };
            self.upsert_inp3_route(rip.destination, rip_alias(rip), received_from, metric);
        }
    }

    /// Build the poison-reversed, per-target-neighbour INP3 RIF this node advertises
    /// toward `to_target_neighbour` — the measured-target-time analogue of
    /// [`build_advertisement`](Self::build_advertisement) (the quality/NODES view). A
    /// pure read; the host calls [`Inp3Rif::to_bytes`] on the result and wraps it in a
    /// PID-0xCF I-frame on the neighbour's interlink session. Host-free: it takes
    /// `my_call` as a parameter (the same discipline as [`ingest_rif`](Self::ingest_rif)).
    ///
    /// The RIF emits, in order (AMBIGUITY-I4-4 — deterministic + cross-stack
    /// byte-identical):
    ///
    /// 1. **Our own node** — exactly one RIP for `my_call` at target-time 0 ms, hop 0,
    ///    no TLVs. The source identity, **always** present and **never** poisoned
    ///    (invariant (Source)).
    /// 2. **Every destination D (≠ `my_call`) holding an INP3 time-route** — at our
    ///    best (lowest) held target time, ordered by ascending local target time then
    ///    destination callsign (ordinal). One RIP each: hop = the best INP3 route's
    ///    hop count; target time = **poison-reverse**: if `to_target_neighbour` is ANY
    ///    of D's kept next hops the RIP is advertised at the horizon (unreachable —
    ///    breaks the would-be two-hop loop, invariant (P)); otherwise at the route's
    ///    real local target time, quantised to the 10 ms wire granule. No TLVs (alias
    ///    emission gated off, AMBIGUITY-I4-1).
    ///
    /// Quality-only destinations (no INP3 route) are **not** in the RIF — they are
    /// carried by NODES. Whether D is advertised is independent of any forwarding
    /// preference: a node that forwards by quality should still tell its neighbours the
    /// time it can reach D in.
    ///
    /// Finally one explicit horizon RIP is appended per `recently_withdrawn` entry
    /// (minus any re-learned-finite this round, and our own node) so the peer withdraws
    /// it immediately. Pass an empty slice for callers that don't drive the withdrawn
    /// set (e.g. pure poison-reverse). Passing the host-drained snapshot — not reading
    /// the live set — is what makes the fan-out race-free. Mirrors C#
    /// `NetRomRoutingTable.BuildRif`.
    pub fn build_rif(
        &self,
        my_call: Callsign,
        to_target_neighbour: Callsign,
        recently_withdrawn: &[Callsign],
    ) -> Inp3Rif {
        // The destination RIPs, ordered by ascending REAL local target time then
        // callsign (AMBIGUITY-I4-4). We sort by the real target time (stable across the
        // neighbour the RIF is built for), not the poison-overridden value, so the RIP
        // order is identical in every neighbour's RIF given identical state.
        let mut dest_rips: Vec<(Inp3Rip, u32)> = Vec::new();

        for slot in self.destinations.iter() {
            let Some(dest) = slot else { continue };
            if dest.destination == my_call {
                continue; // our own node is the 0/0 source RIP below, never a learned route.
            }

            // We ADVERTISE a destination iff we HOLD an INP3 time-route for it (design
            // §1) — at our best (lowest-target-time) INP3 route — and note whether the
            // neighbour we are building toward is ANY of D's kept next hops.
            let mut best_inp3: Option<Inp3RouteMetric> = None;
            let mut poison = false;
            for route in dest.routes.iter().flatten() {
                if route.neighbour == to_target_neighbour {
                    poison = true;
                }
                if let Some(m) = route.inp3 {
                    if best_inp3.is_none_or(|b| m.target_time_ms < b.target_time_ms) {
                        best_inp3 = Some(m);
                    }
                }
            }

            let Some(inp3) = best_inp3 else {
                continue; // no INP3 route held → carried by NODES (quality), not the RIF.
            };

            // POISON-REVERSE (design §2): advertise D back at the horizon (unreachable)
            // if the neighbour we are building this RIF for is ANY of D's kept
            // forwarding next hops — split-horizon over the full kept-route set so the
            // multi-route load-balancer can never seed a two-hop loop.
            let advertised_target_time_ms = if poison {
                Inp3Rip::HORIZON_MS
            } else {
                quantise10(inp3.target_time_ms)
            };

            dest_rips.push((
                Inp3Rip {
                    destination: dest.destination,
                    hop_count: inp3.hop_count,
                    target_time_ms: advertised_target_time_ms,
                    tlvs: Vec::new(), // alias TLV emission gated OFF (AMBIGUITY-I4-1)
                },
                inp3.target_time_ms,
            ));
        }

        dest_rips.sort_by(|a, b| {
            a.1.cmp(&b.1)
                .then_with(|| a.0.destination.base().cmp(b.0.destination.base()))
                .then_with(|| a.0.destination.ssid().cmp(&b.0.destination.ssid()))
        });

        // Own-node RIP first (the source seed: 0/0, no TLVs, never poisoned), then the
        // ordered destination RIPs.
        let mut rips: Vec<Inp3Rip> = Vec::with_capacity(dest_rips.len() + recently_withdrawn.len() + 1);
        rips.push(Inp3Rip {
            destination: my_call,
            hop_count: 0,
            target_time_ms: 0,
            tlvs: Vec::new(),
        });
        for (rip, _) in &dest_rips {
            rips.push(rip.clone());
        }

        // Invariant (W): append one explicit horizon RIP per recently-withdrawn
        // destination so the peer withdraws it immediately (rather than waiting for its
        // obsolescence sweep). A destination withdrawn-then-relearned in the same round
        // is carried by its FINITE RIP above (it's in `emitted`), not poisoned; and our
        // own node is never withdrawn (the Source invariant). Stable ordinal ordering.
        if !recently_withdrawn.is_empty() {
            let mut sorted: Vec<Callsign> = recently_withdrawn.to_vec();
            sorted.sort_by(cmp_callsign);
            for wd in sorted {
                if wd == my_call || dest_rips.iter().any(|(r, _)| r.destination == wd) {
                    continue;
                }
                rips.push(Inp3Rip {
                    destination: wd,
                    hop_count: 0,
                    target_time_ms: Inp3Rip::HORIZON_MS,
                    tlvs: Vec::new(),
                });
            }
        }

        Inp3Rif { rips }
    }

    /// A read-only **peek** at the recently-withdrawn destinations (INP3 invariant W)
    /// in stable ordinal order — for tests and monitoring only; does **not** clear. The
    /// host never reads this on the fan-out path; it
    /// [`drain_recently_withdrawn`](Self::drain_recently_withdrawn)s once at the start
    /// of a round and hands the snapshot to each neighbour's
    /// [`build_rif`](Self::build_rif). Mirrors C# `RecentlyWithdrawn`.
    pub fn recently_withdrawn(&self) -> Vec<Callsign> {
        let mut out = self.recently_withdrawn.clone();
        out.sort_by(cmp_callsign);
        out
    }

    /// Atomically snapshot **and clear** the recently-withdrawn set (INP3 invariant W).
    /// The host calls this **once** at the start of a fan-out round and hands the
    /// returned snapshot to every neighbour's [`build_rif`](Self::build_rif) — so the
    /// one-shot horizon RIP reaches each neighbour exactly once. Draining as one step
    /// closes the host race: a concurrent withdrawal mid-round lands in the live set
    /// AFTER this snapshot, captured by the NEXT round's drain. Stable ordinal ordering;
    /// an empty list when nothing is pending. Mirrors C# `DrainRecentlyWithdrawn`.
    pub fn drain_recently_withdrawn(&mut self) -> Vec<Callsign> {
        let mut snapshot = core::mem::take(&mut self.recently_withdrawn);
        snapshot.sort_by(cmp_callsign);
        snapshot
    }

    // ─── Read-side accessors (allocation-free) ───────────────────────────

    /// Visit every known destination in stable order (alias-or-callsign ascending,
    /// then callsign ascending), passing a value snapshot of each to `f`. No heap —
    /// the caller decides what to copy out.
    pub fn for_each_destination(&self, mut f: impl FnMut(NetRomDestination)) {
        // Collect indices, then selection-sort them into the snapshot order without
        // allocating (MAX_DESTS is a small const).
        let mut order = [usize::MAX; MAX_DESTS];
        let mut n = 0usize;
        for (i, d) in self.destinations.iter().enumerate() {
            if d.is_some() {
                order[n] = i;
                n += 1;
            }
        }
        selection_sort_by(&mut order[..n], |&a, &b| {
            self.destination_lt(
                self.destinations[a].as_ref().unwrap(),
                self.destinations[b].as_ref().unwrap(),
            )
        });
        for &idx in &order[..n] {
            let d = self.destinations[idx].as_ref().unwrap();
            f(self.destination_view(d));
        }
    }

    /// Visit every directly-heard neighbour in stable order (callsign ascending).
    pub fn for_each_neighbour(&self, mut f: impl FnMut(NetRomNeighbour)) {
        let mut order = [usize::MAX; MAX_NBRS];
        let mut n = 0usize;
        for (i, nb) in self.neighbours.iter().enumerate() {
            if nb.is_some() {
                order[n] = i;
                n += 1;
            }
        }
        selection_sort_by(&mut order[..n], |&a, &b| {
            callsign_lt(
                &self.neighbours[a].as_ref().unwrap().neighbour,
                &self.neighbours[b].as_ref().unwrap().neighbour,
            )
        });
        for &idx in &order[..n] {
            let nb = self.neighbours[idx].as_ref().unwrap();
            f(NetRomNeighbour {
                neighbour: nb.neighbour,
                alias: nb.alias,
                port_id: nb.port_id,
                path_quality: nb.path_quality,
                last_heard: nb.last_heard,
            });
        }
    }

    /// Look up a single destination by callsign, returning a value snapshot if known.
    pub fn destination(&self, dest: &Callsign) -> Option<NetRomDestination> {
        self.destination_index(dest)
            .map(|i| self.destination_view(self.destinations[i].as_ref().unwrap()))
    }

    /// Visit the kept routes of `dest` in best-first order. No-op if unknown.
    pub fn for_each_route(&self, dest: &Callsign, mut f: impl FnMut(NetRomRoute)) {
        let Some(i) = self.destination_index(dest) else {
            return;
        };
        let d = self.destinations[i].as_ref().unwrap();
        // Gather the occupied route-slot indices (compacted — empty slots skipped),
        // then sort best-first. MAX_ROUTES is a tiny const so this is allocation-free.
        let mut compact = [usize::MAX; MAX_ROUTES];
        let mut m = 0usize;
        for (j, r) in d.routes.iter().enumerate() {
            if r.is_some() {
                compact[m] = j;
                m += 1;
            }
        }
        selection_sort_by(&mut compact[..m], |&a, &b| {
            is_better(&d.routes[a].unwrap(), &d.routes[b].unwrap())
        });
        for &idx in &compact[..m] {
            let rt = d.routes[idx].unwrap();
            f(NetRomRoute {
                neighbour: rt.neighbour,
                quality: rt.quality,
                obsolescence: rt.obsolescence,
                inp3: rt.inp3,
            });
        }
    }

    /// Build the destination entries for *our own* outgoing NODES broadcast: one
    /// per known destination whose best route still clears the OBSMIN advertise
    /// gate, sorted best-quality-first (ties broken by destination callsign for a
    /// deterministic frame layout). The TX counterpart of [`ingest`](Self::ingest);
    /// the [`super::super::NetRomOriginator`] frames these.
    ///
    /// `obsolete_minimum` overrides the configured OBSMIN
    /// ([`NetRomRoutingOptions::obsolete_minimum`]) when `Some`; pass `0` to
    /// re-advertise every kept route. A destination is dropped when its best route
    /// is quality-0 (never advertise a loop-guarded route) or has decayed below the
    /// gate. Mirrors `NetRomRoutingTable.buildAdvertisement`.
    ///
    /// Note the "best" here is *advertise-best* — highest quality, then highest
    /// obsolescence (freshest) — deliberately distinct from the *routing-best*
    /// (quality, then neighbour callsign) the table uses for forwarding: the
    /// freshest route is what keys the OBSMIN decision, so advertisement tie-breaks
    /// on obsolescence, not neighbour.
    pub fn build_advertisement(
        &self,
        obsolete_minimum: Option<u8>,
    ) -> Vec<NodesAdvertisementEntry> {
        let obsmin = obsolete_minimum.unwrap_or(self.options.obsolete_minimum);
        let mut entries: Vec<NodesAdvertisementEntry> = Vec::new();

        // Snapshot (destination, alias) first so the per-destination route scan
        // below isn't a closure nested inside for_each_destination's borrow.
        let mut dests: Vec<(Callsign, Alias)> = Vec::new();
        self.for_each_destination(|d| dests.push((d.destination, d.alias)));

        for (destination, destination_alias) in dests {
            let mut best: Option<NetRomRoute> = None;
            self.for_each_route(&destination, |route| {
                let better = match best {
                    None => true,
                    Some(b) => {
                        route.quality > b.quality
                            || (route.quality == b.quality && route.obsolescence > b.obsolescence)
                    }
                };
                if better {
                    best = Some(route);
                }
            });

            let Some(best) = best else { continue };
            if best.quality == quality::MIN {
                continue; // never advertise a quality-0 / loop-guarded route (MIN == 0)
            }
            if best.obsolescence < obsmin {
                continue; // OBSMIN: decayed below the advertise threshold
            }
            entries.push(NodesAdvertisementEntry {
                destination,
                destination_alias,
                best_neighbour: best.neighbour,
                quality: best.quality,
            });
        }

        entries.sort_by(|a, b| {
            b.quality
                .cmp(&a.quality)
                .then_with(|| a.destination.base().cmp(b.destination.base()))
                .then_with(|| a.destination.ssid().cmp(&b.destination.ssid()))
        });
        entries
    }

    /// Resolve a `connect <target>` string — an **alias** (e.g. `SOT`) or a
    /// **callsign** (e.g. `GB7SOT`, with or without SSID) — to a known destination.
    /// Case-insensitive; an exact alias match is preferred over a callsign match
    /// (mirrors the TS `resolveDestination`). Returns `None` for an empty/whitespace
    /// target or a miss.
    pub fn resolve_destination(&self, target: &str) -> Option<NetRomDestination> {
        let needle = target.trim();
        if needle.is_empty() {
            return None;
        }
        let mut alias_hit = None;
        let mut call_hit = None;
        self.for_each_destination(|d| {
            if alias_hit.is_none() && !d.alias.is_empty() && eq_ascii_ci(d.alias.as_str(), needle) {
                alias_hit = Some(d);
            }
            if call_hit.is_none() {
                let mut buf = [0u8; 16];
                if let Some(n) = d.destination.write_display(&mut buf) {
                    if let Ok(text) = core::str::from_utf8(&buf[..n]) {
                        if eq_ascii_ci(text, needle) {
                            call_hit = Some(d);
                        }
                    }
                }
            }
        });
        alias_hit.or(call_hit)
    }

    /// The best next-hop neighbour to reach `dest`, excluding `exclude` — the
    /// transit-forwarding next hop that never bounces a datagram back the way it came.
    /// Routes are scanned best-first, so the first whose neighbour is not `exclude` is
    /// the best usable onward route; `None` if none remains.
    pub fn best_route_excluding(&self, dest: &Callsign, exclude: &Callsign) -> Option<Callsign> {
        let mut found: Option<Callsign> = None;
        self.for_each_route(dest, |route| {
            if found.is_none() && route.neighbour != *exclude {
                found = Some(route.neighbour);
            }
        });
        found
    }

    /// The lowest-target-time INP3 route to `dest` whose neighbour is not `exclude` (the
    /// way a transit datagram arrived) — the time-space mirror of
    /// [`Self::best_route_excluding`], used by `decide_forward` under `prefer_inp3_routes`.
    /// Delegates to the shared [`crate::netrom::forwarding::select_inp3_next_hop`] over the
    /// destination's kept routes (gathered via [`Self::for_each_route`]), so forward +
    /// connect agree on the active INP3 next hop. `None` when no usable INP3 route exists.
    pub fn inp3_next_hop_excluding(
        &self,
        dest: &Callsign,
        exclude: &Callsign,
    ) -> Option<Callsign> {
        let mut routes: alloc::vec::Vec<NetRomRoute> = alloc::vec::Vec::new();
        self.for_each_route(dest, |route| routes.push(route));
        crate::netrom::forwarding::select_inp3_next_hop(&routes, exclude)
    }

    /// A per-flow, quality-weighted next-hop among the eligible routes to `dest`
    /// (neighbour ≠ `exclude`, quality &gt; 0): the routes form weighted buckets sized
    /// by quality, and `flow_hash` picks one. A circuit's datagrams (constant flow
    /// hash) pin to one route while distinct circuits spread ∝ quality. `None` if no
    /// route is usable. The transit-forwarding load-balancing selector.
    pub fn select_route_excluding(
        &self,
        dest: &Callsign,
        exclude: &Callsign,
        flow_hash: u32,
    ) -> Option<Callsign> {
        let mut total: u32 = 0;
        self.for_each_route(dest, |route| {
            if route.neighbour != *exclude && route.quality > 0 {
                total += route.quality as u32;
            }
        });
        if total == 0 {
            return None;
        }

        let mut target = flow_hash % total;
        let mut chosen: Option<Callsign> = None;
        self.for_each_route(dest, |route| {
            if chosen.is_some() || route.neighbour == *exclude || route.quality == 0 {
                return;
            }
            let weight = route.quality as u32;
            if target < weight {
                chosen = Some(route.neighbour);
            } else {
                target -= weight;
            }
        });
        chosen
    }

    /// The directly-heard neighbour entry for `call`, if any (mirrors the TS
    /// `neighbourFor`). A connector uses this to decide whether a destination can be
    /// reached as its own neighbour.
    pub fn neighbour(&self, call: &Callsign) -> Option<NetRomNeighbour> {
        let i = self.neighbour_index(call)?;
        let nb = self.neighbours[i].as_ref().unwrap();
        Some(NetRomNeighbour {
            neighbour: nb.neighbour,
            alias: nb.alias,
            port_id: nb.port_id,
            path_quality: nb.path_quality,
            last_heard: nb.last_heard,
        })
    }

    // ─── Internals ───────────────────────────────────────────────────────

    fn neighbour_index(&self, call: &Callsign) -> Option<usize> {
        self.neighbours
            .iter()
            .position(|n| n.is_some_and(|nb| nb.neighbour == *call))
    }

    fn destination_index(&self, call: &Callsign) -> Option<usize> {
        self.destinations
            .iter()
            .position(|d| d.is_some_and(|dest| dest.destination == *call))
    }

    fn destination_view(&self, d: &DestinationState<MAX_ROUTES>) -> NetRomDestination {
        NetRomDestination {
            destination: d.destination,
            alias: d.alias,
            best_route: d.best_route().map(|rt| NetRomRoute {
                neighbour: rt.neighbour,
                quality: rt.quality,
                obsolescence: rt.obsolescence,
                inp3: rt.inp3,
            }),
            route_count: d.route_count() as u8,
        }
    }

    // Record a destination in the recently-withdrawn set (INP3 invariant W), kept
    // distinct (the no_std analogue of the C# HashSet add). Called ONLY when an
    // inp3-bearing route fully leaves — the default-off guard lives in the callers.
    fn record_recently_withdrawn(&mut self, destination: Callsign) {
        if !self.recently_withdrawn.contains(&destination) {
            self.recently_withdrawn.push(destination);
        }
    }

    // True iff some kept route to `destination` still carries an inp3 metric. A
    // destination gone from the table holds no route, so no inp3 route either. The
    // "lost its LAST INP3 route" predicate for invariant (W). Mirrors C#
    // `HasAnyInp3Route`.
    fn has_any_inp3_route(&self, destination: &Callsign) -> bool {
        match self.destination_index(destination) {
            Some(i) => self.destinations[i]
                .as_ref()
                .unwrap()
                .routes
                .iter()
                .flatten()
                .any(|rt| rt.inp3.is_some()),
            None => false,
        }
    }

    // Attach (or refresh) an INP3 time-route metric on the (destination via
    // via_neighbour) route — the time-space analogue of upsert_route. If the route
    // already exists (as a quality route, or a prior time-route) the metric is set in
    // place, resetting obsolescence to OBSINIT and preserving its quality; if it does
    // not exist the route is created as a pure time-route (quality 0, obsolescence
    // OBSINIT). The per-dest cap is enforced by the same quality-first eviction key as
    // the quality path (AMBIGUITY-I3-2). Honours the destination cap exactly as
    // upsert_route does. (Floor/horizon/hop/loop gating is done by ingest_rif before
    // here, so this only ever stores a live, finite, in-horizon metric.) Mirrors C#
    // `UpsertInp3Route`.
    fn upsert_inp3_route(
        &mut self,
        destination: Callsign,
        alias: Alias,
        via_neighbour: Callsign,
        metric: Inp3RouteMetric,
    ) {
        let dest_idx = match self.destination_index(&destination) {
            Some(i) => {
                if !alias.is_empty() {
                    self.destinations[i].as_mut().unwrap().alias = alias;
                }
                i
            }
            None => {
                let Some(free) = self.destinations.iter().position(Option::is_none) else {
                    return; // heuristic 9: destination list full — ignore new destinations
                };
                self.destinations[free] = Some(DestinationState {
                    destination,
                    alias,
                    routes: [None; MAX_ROUTES],
                });
                free
            }
        };

        let dest = self.destinations[dest_idx].as_mut().unwrap();

        if let Some(ri) = dest.route_index(&via_neighbour) {
            // Refresh the time-route in place: keep the route's quality (its other
            // metric space) and reset obsolescence so the time-route ages like a
            // quality route refreshed by a NODES broadcast.
            let mut rt = dest.routes[ri].unwrap();
            rt.obsolescence = self.options.obsolete_initial;
            rt.inp3 = Some(metric);
            dest.routes[ri] = Some(rt);
            return;
        }

        let new_route = RouteState {
            neighbour: via_neighbour,
            // A brand-new route known only via INP3: quality 0 (no NODES quality),
            // the time metric carrying its reachability. Quality 0 makes it invisible
            // to the quality path / never advertised, exactly as intended.
            quality: quality::MIN,
            obsolescence: self.options.obsolete_initial,
            inp3: Some(metric),
        };

        // A free slot takes it; otherwise enforce the per-destination cap by the same
        // quality-first eviction key the quality path uses (an INP3-only route sorts as
        // a quality-0 route for eviction ordering only — AMBIGUITY-I3-2).
        if let Some(free) = dest.first_free() {
            dest.routes[free] = Some(new_route);
            return;
        }
        if let Some(worst_idx) = dest.worst_route_index() {
            let worst = dest.routes[worst_idx].unwrap();
            if is_better(&new_route, &worst) {
                dest.routes[worst_idx] = Some(new_route);
            }
        }
    }

    // Withdraw the INP3 metric of the (destination via via_neighbour) route (a horizon
    // withdrawal). Clears inp3 only — a coexisting quality route stays. A route left
    // with neither a usable quality (≤ MINQUAL / 0) nor an inp3 metric is removed; a
    // destination left with no route is removed. A no-op if the route / destination is
    // unknown or the route had no inp3 metric. Records invariant (W) if the destination
    // is left with NO inp3 route. Mirrors C# `WithdrawInp3`.
    fn withdraw_inp3(&mut self, destination: &Callsign, via_neighbour: &Callsign) {
        let Some(dest_idx) = self.destination_index(destination) else {
            return;
        };
        let dest = self.destinations[dest_idx].as_mut().unwrap();
        let Some(ri) = dest.route_index(via_neighbour) else {
            return;
        };
        let route = dest.routes[ri].unwrap();
        if route.inp3.is_none() {
            return; // nothing INP3 to withdraw on this route
        }

        // A route whose only reason to exist was its (now-withdrawn) time metric — i.e.
        // it carries no usable quality — is removed outright; otherwise it survives as
        // a pure quality route with inp3 cleared.
        let has_usable_quality =
            route.quality > quality::MIN && route.quality >= self.options.min_quality;
        if has_usable_quality {
            let mut rt = route;
            rt.inp3 = None;
            dest.routes[ri] = Some(rt);
        } else {
            dest.routes[ri] = None;
            if dest.route_count() == 0 {
                self.destinations[dest_idx] = None;
            }
        }

        // Invariant (W): if the destination now holds NO inp3-bearing route at all, it
        // has left the INP3 time-space → record the one-shot horizon withdrawal. (We
        // had an inp3 metric on this route a moment ago, so this is only ever reached on
        // a genuine INP3 withdrawal.)
        if !self.has_any_inp3_route(destination) {
            self.record_recently_withdrawn(*destination);
        }
    }

    // Add or refresh a route to `destination` via `via_neighbour`. Applies the
    // quality-0 / MINQUAL floor (heuristic 8), resets obsolescence to OBSINIT,
    // enforces the per-destination route cap (heuristic 7) and the destination cap
    // (heuristic 9). Mirrors C# `NetRomRoutingTable.UpsertRoute`.
    fn upsert_route(
        &mut self,
        destination: Callsign,
        alias: Alias,
        via_neighbour: Callsign,
        q: u8,
    ) {
        // A quality-0 route is never usable / kept; likewise anything under the
        // configured floor. If such a route already existed (from a prior, better
        // advertisement), a now-too-low re-advertisement removes it.
        let acceptable = q > quality::MIN && q >= self.options.min_quality;

        let dest_idx = match self.destination_index(&destination) {
            Some(i) => {
                // Refresh a known destination's alias when the advertisement carries one.
                if !alias.is_empty() {
                    self.destinations[i].as_mut().unwrap().alias = alias;
                }
                i
            }
            None => {
                if !acceptable {
                    return; // nothing to add, nothing to update
                }
                let Some(free) = self.destinations.iter().position(Option::is_none) else {
                    return; // heuristic 9: destination list full — ignore new destinations
                };
                self.destinations[free] = Some(DestinationState {
                    destination,
                    alias,
                    routes: [None; MAX_ROUTES],
                });
                free
            }
        };

        let dest = self.destinations[dest_idx].as_mut().unwrap();

        if !acceptable {
            // Drop a route that has decayed below the floor.
            if let Some(ri) = dest.route_index(&via_neighbour) {
                dest.routes[ri] = None;
            }
            if dest.route_count() == 0 {
                self.destinations[dest_idx] = None;
            }
            return;
        }

        // Preserve any INP3 metric already learned for this (dest via neighbour)
        // route — a NODES quality refresh must not wipe a coexisting time-route (the
        // two metric spaces are independent; see ingest_rif). Mirrors C# `UpsertRoute`
        // (`Inp3 = existing?.Inp3`).
        let preserved_inp3 = dest
            .route_index(&via_neighbour)
            .and_then(|ri| dest.routes[ri].and_then(|rt| rt.inp3));

        let new_route = RouteState {
            neighbour: via_neighbour,
            quality: q,
            obsolescence: self.options.obsolete_initial,
            inp3: preserved_inp3,
        };

        // If this (dest, via) route already exists, update it in place.
        if let Some(ri) = dest.route_index(&via_neighbour) {
            dest.routes[ri] = Some(new_route);
            return;
        }

        // New route via a new neighbour. If there's a free slot, take it.
        if let Some(free) = dest.first_free() {
            dest.routes[free] = Some(new_route);
            return;
        }

        // Heuristic 7: route set is full. Evict the worst kept route iff the new
        // route is strictly better than it (keep only the N best). If the new route
        // isn't better than every kept route, it is simply not kept.
        if let Some(worst_idx) = dest.worst_route_index() {
            let worst = dest.routes[worst_idx].unwrap();
            if is_better(&new_route, &worst) {
                dest.routes[worst_idx] = Some(new_route);
            }
        }
    }

    // Drop neighbours that are no longer the next hop for any kept route. (A
    // neighbour we heard directly always has its own direct route, so it survives
    // until that route ages out — at which point it is a genuine orphan.) Mirrors
    // C# `PruneOrphanNeighbours`.
    fn prune_orphan_neighbours(&mut self) {
        for ni in 0..MAX_NBRS {
            let Some(nb) = self.neighbours[ni] else {
                continue;
            };
            let in_use = self.destinations.iter().flatten().any(|d| {
                d.routes
                    .iter()
                    .flatten()
                    .any(|r| r.neighbour == nb.neighbour)
            });
            if !in_use {
                self.neighbours[ni] = None;
            }
        }
    }

    // Snapshot ordering for destinations: alias-or-callsign (case-insensitive)
    // ascending, then callsign ascending. Returns true if `a` sorts before `b`.
    fn destination_lt(
        &self,
        a: &DestinationState<MAX_ROUTES>,
        b: &DestinationState<MAX_ROUTES>,
    ) -> bool {
        let ka = sort_key(a.alias, &a.destination);
        let kb = sort_key(b.alias, &b.destination);
        match cmp_ascii_ci(&ka, &kb) {
            core::cmp::Ordering::Less => true,
            core::cmp::Ordering::Greater => false,
            core::cmp::Ordering::Equal => callsign_lt(&a.destination, &b.destination),
        }
    }
}

// The case-insensitive sort key for a destination: its alias if present, else its
// callsign base. Returned as a small fixed buffer + length (no heap).
fn sort_key(alias: Alias, call: &Callsign) -> ([u8; 8], usize) {
    let mut buf = [0u8; 8];
    if alias.is_empty() {
        let b = call.base();
        let n = b.len().min(8);
        buf[..n].copy_from_slice(&b[..n]);
        (buf, n)
    } else {
        let b = alias.as_bytes();
        let n = b.len().min(8);
        buf[..n].copy_from_slice(&b[..n]);
        (buf, n)
    }
}

fn cmp_ascii_ci(a: &([u8; 8], usize), b: &([u8; 8], usize)) -> core::cmp::Ordering {
    let (ab, an) = a;
    let (bb, bn) = b;
    for i in 0..(*an).min(*bn) {
        let ca = ab[i].to_ascii_uppercase();
        let cb = bb[i].to_ascii_uppercase();
        match ca.cmp(&cb) {
            core::cmp::Ordering::Equal => {}
            other => return other,
        }
    }
    an.cmp(bn)
}

// Ordinal callsign comparison: base bytes then SSID. Matches the C# snapshot's
// `StringComparer.Ordinal` over `callsign.ToString()` for the alphanumeric base +
// SSID forms NET/ROM uses.
/// ASCII case-insensitive equality of two strings (no_std; callsign/alias text is
/// ASCII). Used by [`NetRomRoutingTable::resolve_destination`].
fn eq_ascii_ci(a: &str, b: &str) -> bool {
    a.len() == b.len()
        && a.bytes()
            .zip(b.bytes())
            .all(|(x, y)| x.eq_ignore_ascii_case(&y))
}

fn callsign_lt(a: &Callsign, b: &Callsign) -> bool {
    match a.base().cmp(b.base()) {
        core::cmp::Ordering::Less => true,
        core::cmp::Ordering::Greater => false,
        core::cmp::Ordering::Equal => a.ssid() < b.ssid(),
    }
}

// Ordinal callsign comparison (base bytes then SSID) as an `Ordering`, for the
// `sort_by` of the recently-withdrawn / RIF-destination orderings — the same total
// order `callsign_lt` induces, matching the C# `StringComparer.Ordinal`.
fn cmp_callsign(a: &Callsign, b: &Callsign) -> core::cmp::Ordering {
    a.base().cmp(b.base()).then_with(|| a.ssid().cmp(&b.ssid()))
}

// Quantise a full-ms local target time down to the 10 ms wire granule the RIP codec
// carries (the stored metric is full-ms — the granule is an emission-only concern,
// AMBIGUITY-I3-3). Floor, so the emitted finite time never exceeds the stored one;
// clamped to one granule below the horizon so a near-horizon finite metric can never
// round up to read as a withdrawal. Mirrors C# `Quantise10`.
fn quantise10(target_time_ms: u32) -> u32 {
    let quantised = (target_time_ms / 10) * 10;
    quantised.min(Inp3Rip::HORIZON_MS - 10)
}

// Decode a RIP's first alias TLV into an `Alias` (the destination mnemonic), or the
// empty alias when the RIP carries none. The fixed-capacity `no_std` analogue of the
// C# `rip.Alias ?? string.Empty`.
fn rip_alias(rip: &Inp3Rip) -> Alias {
    let mut buf = [0u8; 16];
    match rip.alias(&mut buf) {
        Some(n) => Alias::from_str_lossy(core::str::from_utf8(&buf[..n]).unwrap_or("")),
        None => Alias::EMPTY,
    }
}

// Route ordering used everywhere: "better" = higher quality, ties broken by
// neighbour callsign ascending (matches the C# `OrderByDescending(Quality)
// .ThenBy(Neighbour, Ordinal)`).
fn is_better(a: &RouteState, b: &RouteState) -> bool {
    match a.quality.cmp(&b.quality) {
        core::cmp::Ordering::Greater => true,
        core::cmp::Ordering::Less => false,
        core::cmp::Ordering::Equal => callsign_lt(&a.neighbour, &b.neighbour),
    }
}

// The exact inverse used for eviction: "worse" = lower quality, ties broken by
// neighbour callsign descending — so the kept set is the N best by `is_better`.
fn is_worse(a: &RouteState, b: &RouteState) -> bool {
    is_better(b, a)
}

// Allocation-free selection sort over a small index slice, ordering by `lt`
// (returns true when its first arg should come before its second). Stable enough
// for our fully-ordered keys; MAX_* are tiny consts so O(n²) is fine.
fn selection_sort_by<T: Copy>(items: &mut [T], mut lt: impl FnMut(&T, &T) -> bool) {
    for i in 0..items.len() {
        let mut min = i;
        for j in (i + 1)..items.len() {
            if lt(&items[j], &items[min]) {
                min = j;
            }
        }
        items.swap(i, min);
    }
}

#[cfg(test)]
mod inp3_tests {
    //! Ports of the C# INP3 routing-table suites onto the `no_std` core:
    //! `Inp3IngestTests`, `Inp3BuildRifTests`, `Inp3RecentlyWithdrawnTests`
    //! (`tests/Packet.NetRom.Tests/Routing/`), cross-checked against the merged TS
    //! `ax25-ts/src/netrom/routing-table.ts`. Same cases, same hop/horizon boundaries,
    //! same target-time math (`peer + sntt + 10`, `hop + 1`), same poison-reverse and
    //! recently-withdrawn semantics. A finite-capacity table type
    //! (`NetRomRoutingTable<MAX_DESTS, MAX_ROUTES, MAX_NBRS>`) replaces the C# unbounded
    //! `Dictionary`; the suites instantiate it with caps comfortably above each test's
    //! needs (the per-destination route cap tests pick their own MAX_ROUTES).

    use super::*;
    use crate::netrom::routing::inp3_sntt::SNTT_UNSET_RAW;
    use crate::netrom::wire::inp3_rif::{Inp3Rif, Inp3Rip, Inp3Tlv};
    use crate::netrom::wire::nodes_broadcast_builder::write_nodes_frame;
    use alloc::vec;
    use alloc::vec::Vec;

    // Generous caps for the general suites; the cap-specific tests use their own.
    type Table = NetRomRoutingTable<16, 3, 16>;

    fn cs(text: &str) -> Callsign {
        Callsign::parse(text).expect("test callsign parses")
    }

    fn me() -> Callsign {
        cs("M0LTE")
    }
    fn nbr_a() -> Callsign {
        cs("GB7RDG")
    }
    fn nbr_b() -> Callsign {
        cs("GB7XYZ")
    }
    fn dest_sot() -> Callsign {
        cs("GB7SOT")
    }
    fn dest_mnc() -> Callsign {
        cs("GB7MNC")
    }

    fn port() -> PortId {
        PortId::from_str_lossy("vhf")
    }

    fn rip(destination: Callsign, hop_count: u8, target_time_ms: u32) -> Inp3Rip {
        Inp3Rip {
            destination,
            hop_count,
            target_time_ms,
            tlvs: Vec::new(),
        }
    }

    fn rip_alias_of(destination: Callsign, hop_count: u8, target_time_ms: u32, alias: &str) -> Inp3Rip {
        Inp3Rip {
            destination,
            hop_count,
            target_time_ms,
            tlvs: vec![Inp3Tlv::alias(alias)],
        }
    }

    fn rif(rips: Vec<Inp3Rip>) -> Inp3Rif {
        Inp3Rif { rips }
    }

    // A NODES quality route: build a frame and parse it back so we exercise the real
    // codec, mirroring the C# `Nodes(...)` test helper.
    fn nodes(
        sender_alias: &str,
        entries: &[(Callsign, &str, Callsign, u8)],
    ) -> NodesBroadcast {
        let adv: Vec<NodesAdvertisementEntry> = entries
            .iter()
            .map(|(dest, alias, via, q)| NodesAdvertisementEntry {
                destination: *dest,
                destination_alias: Alias::from_str_lossy(alias),
                best_neighbour: *via,
                quality: *q,
            })
            .collect();
        let mut buf = [0u8; crate::netrom::wire::nodes_broadcast_builder::MAX_NODES_FRAME_LEN];
        let n = write_nodes_frame(&Alias::from_str_lossy(sender_alias), &adv, &mut buf).unwrap();
        NodesBroadcast::try_parse(&buf[..n]).unwrap()
    }

    // The kept route to `dest` via `via`, if any (the value snapshot), gathered via the
    // table's visitor accessor (the no_std analogue of the C# `RouteVia`).
    fn route_via(table: &Table, dest: Callsign, via: Callsign) -> Option<NetRomRoute> {
        let mut found = None;
        table.for_each_route(&dest, |r| {
            if r.neighbour == via {
                found = Some(r);
            }
        });
        found
    }

    fn routes_of(table: &Table, dest: Callsign) -> Vec<NetRomRoute> {
        let mut v = Vec::new();
        table.for_each_route(&dest, |r| v.push(r));
        v
    }

    fn dest_alias(table: &Table, dest: Callsign) -> Option<Alias> {
        table.destination(&dest).map(|d| d.alias)
    }

    fn has_dest(table: &Table, dest: Callsign) -> bool {
        table.destination(&dest).is_some()
    }

    const HOP_LIMIT: u32 = Table::DEFAULT_HOP_LIMIT;

    // ─────────────────────────── Inp3IngestTests ───────────────────────────

    #[test]
    fn ingesting_a_rif_learns_an_inp3_time_route_via_the_carrying_neighbour() {
        let mut table = Table::with_defaults();
        table.ingest_rif(
            nbr_a(),
            me(),
            50,
            &rif(vec![rip_alias_of(dest_sot(), 1, 100, "SOT")]),
            HOP_LIMIT,
        );

        let route = route_via(&table, dest_sot(), nbr_a()).expect("RIF teaches a route to SOT");
        assert_eq!(route.neighbour, nbr_a());
        assert!(route.inp3.is_some(), "the route carries an INP3 metric");
        assert_eq!(dest_alias(&table, dest_sot()).unwrap().as_str(), "SOT");
    }

    #[test]
    fn a_pure_inp3_route_has_quality_zero_so_it_is_invisible_to_the_quality_path() {
        let mut table = Table::with_defaults();
        table.ingest_rif(nbr_a(), me(), 50, &rif(vec![rip(dest_sot(), 1, 100)]), HOP_LIMIT);

        let route = route_via(&table, dest_sot(), nbr_a()).unwrap();
        assert_eq!(route.quality, 0, "a route known only via INP3 carries no NODES quality");
        assert!(route.inp3.is_some());
    }

    #[test]
    fn re_ingesting_the_same_dest_via_the_same_neighbour_refreshes_the_metric_in_place() {
        let mut table = Table::with_defaults();
        table.ingest_rif(nbr_a(), me(), 50, &rif(vec![rip(dest_sot(), 1, 100)]), HOP_LIMIT);
        table.ingest_rif(nbr_a(), me(), 50, &rif(vec![rip(dest_sot(), 1, 300)]), HOP_LIMIT);

        let routes = routes_of(&table, dest_sot());
        assert_eq!(routes.len(), 1, "the same (dest, via) is one route, refreshed not duplicated");
        assert_eq!(routes[0].inp3.unwrap().target_time_ms, 300 + 50 + 10);
    }

    #[test]
    fn local_target_time_is_peer_time_plus_link_sntt_plus_ten_ms_per_hop() {
        let mut table = Table::with_defaults();
        table.ingest_rif(nbr_a(), me(), 75, &rif(vec![rip(dest_sot(), 2, 100)]), HOP_LIMIT);

        let route = route_via(&table, dest_sot(), nbr_a()).unwrap();
        assert_eq!(route.inp3.unwrap().target_time_ms, 100 + 75 + 10);
        assert_eq!(route.inp3.unwrap().hop_count, 3, "one more hop — through us");
    }

    #[test]
    fn per_hop_increment_keeps_target_time_strictly_increasing_across_a_zero_ms_link() {
        let mut table = Table::with_defaults();
        table.ingest_rif(nbr_a(), me(), 0, &rif(vec![rip(dest_sot(), 0, 0)]), HOP_LIMIT);

        let route = route_via(&table, dest_sot(), nbr_a()).unwrap();
        assert_eq!(route.inp3.unwrap().target_time_ms, 10, "the +10 per-hop floor keeps it > 0");
        assert_eq!(route.inp3.unwrap().hop_count, 1);
    }

    #[test]
    fn full_millisecond_precision_is_kept_not_requantised_to_the_ten_ms_granule() {
        let mut table = Table::with_defaults();
        table.ingest_rif(nbr_a(), me(), 73, &rif(vec![rip(dest_sot(), 1, 100)]), HOP_LIMIT);

        let route = route_via(&table, dest_sot(), nbr_a()).unwrap();
        assert_eq!(route.inp3.unwrap().target_time_ms, 183, "100 + 73 + 10, full ms");
    }

    #[test]
    fn best_inp3_route_per_destination_is_the_lowest_target_time() {
        let mut table = Table::with_defaults();
        table.ingest_rif(nbr_a(), me(), 200, &rif(vec![rip(dest_sot(), 1, 100)]), HOP_LIMIT); // 310
        table.ingest_rif(nbr_b(), me(), 20, &rif(vec![rip(dest_sot(), 1, 100)]), HOP_LIMIT); // 130

        let best = routes_of(&table, dest_sot())
            .into_iter()
            .filter(|r| r.inp3.is_some())
            .min_by(|a, b| {
                let am = a.inp3.unwrap();
                let bm = b.inp3.unwrap();
                am.target_time_ms
                    .cmp(&bm.target_time_ms)
                    .then(am.hop_count.cmp(&bm.hop_count))
                    .then(cmp_callsign(&a.neighbour, &b.neighbour))
            })
            .unwrap();
        assert_eq!(best.neighbour, nbr_b());
        assert_eq!(best.inp3.unwrap().target_time_ms, 130);
    }

    // ─── Horizon withdraws ───

    #[test]
    fn a_rip_at_or_over_the_horizon_is_a_withdrawal_clearing_the_inp3_metric() {
        let mut table = Table::with_defaults();
        table.ingest_rif(nbr_a(), me(), 50, &rif(vec![rip(dest_sot(), 1, 100)]), HOP_LIMIT);
        assert!(route_via(&table, dest_sot(), nbr_a()).unwrap().inp3.is_some());

        table.ingest_rif(
            nbr_a(),
            me(),
            50,
            &rif(vec![rip(dest_sot(), 1, Inp3Rip::HORIZON_MS)]),
            HOP_LIMIT,
        );

        assert!(route_via(&table, dest_sot(), nbr_a()).is_none(), "withdrawing the only metric removes the route");
        assert!(!has_dest(&table, dest_sot()), "the destination left with no route is removed");
    }

    #[test]
    fn a_computed_target_time_reaching_the_horizon_also_withdraws() {
        let mut table = Table::with_defaults();
        table.ingest_rif(nbr_a(), me(), 50, &rif(vec![rip(dest_sot(), 1, 100)]), HOP_LIMIT);

        // peer just under horizon, link SNTT pushes the computed value over it.
        table.ingest_rif(
            nbr_a(),
            me(),
            100,
            &rif(vec![rip(dest_sot(), 1, Inp3Rip::HORIZON_MS - 10)]),
            HOP_LIMIT,
        );

        assert!(route_via(&table, dest_sot(), nbr_a()).is_none(), "100 + 599_990 + 10 ≥ 600_000 → withdrawn");
    }

    #[test]
    fn withdrawal_clears_only_the_inp3_metric_and_leaves_a_coexisting_quality_route() {
        let mut table = Table::with_defaults();
        table.ingest(nbr_a(), me(), port(), &nodes("RDG", &[(dest_sot(), "SOT", nbr_a(), 200)]), 0);
        table.ingest_rif(nbr_a(), me(), 50, &rif(vec![rip(dest_sot(), 1, 100)]), HOP_LIMIT);
        let both = route_via(&table, dest_sot(), nbr_a()).unwrap();
        assert!(both.quality > 0);
        assert!(both.inp3.is_some());

        table.ingest_rif(
            nbr_a(),
            me(),
            50,
            &rif(vec![rip(dest_sot(), 1, Inp3Rip::HORIZON_MS)]),
            HOP_LIMIT,
        );

        let after = route_via(&table, dest_sot(), nbr_a()).expect("the quality route survives");
        assert!(after.inp3.is_none(), "only the INP3 metric was withdrawn");
        assert_eq!(after.quality, quality::combine(200, 192), "the quality metric is untouched");
    }

    #[test]
    fn an_unset_sntt_never_withdraws_a_route_it_never_learned() {
        let mut table = Table::with_defaults();
        table.ingest(nbr_a(), me(), port(), &nodes("RDG", &[(dest_sot(), "SOT", nbr_a(), 200)]), 0);

        table.ingest_rif(nbr_a(), me(), SNTT_UNSET_RAW, &rif(vec![rip(dest_sot(), 1, 100)]), HOP_LIMIT);

        let route = route_via(&table, dest_sot(), nbr_a()).expect("the quality route is undisturbed");
        assert!(route.inp3.is_none(), "no time-route learned (link cost unknown)");
        assert!(route.quality > 0, "the quality route is intact");
    }

    #[test]
    fn a_horizon_rip_withdraws_even_when_the_link_is_unmeasured() {
        let mut table = Table::with_defaults();
        table.ingest_rif(nbr_a(), me(), 50, &rif(vec![rip(dest_sot(), 1, 100)]), HOP_LIMIT);
        assert!(route_via(&table, dest_sot(), nbr_a()).unwrap().inp3.is_some());

        table.ingest_rif(
            nbr_a(),
            me(),
            SNTT_UNSET_RAW,
            &rif(vec![rip(dest_sot(), 1, Inp3Rip::HORIZON_MS)]),
            HOP_LIMIT,
        );

        assert!(route_via(&table, dest_sot(), nbr_a()).is_none(), "explicit horizon RIP withdraws regardless of SNTT");
    }

    // ─── Hop limit ───

    #[test]
    fn a_rip_whose_local_hop_count_exceeds_the_hop_limit_is_not_learned() {
        let mut table = Table::with_defaults();
        table.ingest_rif(nbr_a(), me(), 50, &rif(vec![rip(dest_sot(), 5, 100)]), 5);
        assert!(route_via(&table, dest_sot(), nbr_a()).is_none(), "local hop 6 > hopLimit 5");
    }

    #[test]
    fn a_rip_at_exactly_the_hop_limit_is_learned() {
        let mut table = Table::with_defaults();
        table.ingest_rif(nbr_a(), me(), 50, &rif(vec![rip(dest_sot(), 4, 100)]), 5);
        let route = route_via(&table, dest_sot(), nbr_a()).expect("local hop 5 == hopLimit 5");
        assert_eq!(route.inp3.unwrap().hop_count, 5);
    }

    #[test]
    fn the_default_hop_limit_is_thirty() {
        assert_eq!(Table::DEFAULT_HOP_LIMIT, 30);
        let mut table = Table::with_defaults();
        table.ingest_rif(nbr_a(), me(), 50, &rif(vec![rip(dest_sot(), 29, 100)]), HOP_LIMIT);
        table.ingest_rif(nbr_b(), me(), 50, &rif(vec![rip(dest_mnc(), 30, 100)]), HOP_LIMIT);

        assert!(route_via(&table, dest_sot(), nbr_a()).is_some(), "30 hops within the default limit");
        assert!(route_via(&table, dest_mnc(), nbr_b()).is_none(), "31 hops exceeds the default limit");
    }

    // ─── Trivial-loop guard ───

    #[test]
    fn a_rip_whose_destination_is_us_is_skipped() {
        let mut table = Table::with_defaults();
        table.ingest_rif(nbr_a(), me(), 50, &rif(vec![rip(me(), 1, 100)]), HOP_LIMIT);
        assert!(!has_dest(&table, me()), "a route to ourselves is never learned");
    }

    // ─── Route cap ───

    #[test]
    fn the_per_destination_route_cap_is_respected_evicting_lowest_quality_first() {
        // Per-destination cap of 2 (the MAX_ROUTES const generic).
        let mut table: NetRomRoutingTable<16, 2, 16> = NetRomRoutingTable::with_defaults();
        let n1 = cs("GB7AAA");
        let n2 = cs("GB7BBB");
        let n3 = cs("GB7CCC");

        table.ingest_rif(n1, me(), 10, &rif(vec![rip(dest_sot(), 1, 100)]), HOP_LIMIT);
        table.ingest_rif(n2, me(), 20, &rif(vec![rip(dest_sot(), 1, 100)]), HOP_LIMIT);
        table.ingest_rif(n3, me(), 30, &rif(vec![rip(dest_sot(), 1, 100)]), HOP_LIMIT);

        let mut count = 0;
        table.for_each_route(&dest_sot(), |_| count += 1);
        assert_eq!(count, 2, "the per-destination route cap is 2");
    }

    #[test]
    fn an_inp3_only_route_is_evicted_in_favour_of_a_quality_route_when_capped() {
        // Per-destination cap of 1.
        let mut table: NetRomRoutingTable<16, 1, 16> = NetRomRoutingTable::with_defaults();
        table.ingest(nbr_a(), me(), port(), &nodes("RDG", &[(dest_sot(), "SOT", nbr_a(), 200)]), 0);
        table.ingest_rif(nbr_b(), me(), 10, &rif(vec![rip(dest_sot(), 1, 1)]), HOP_LIMIT);

        let mut routes = Vec::new();
        table.for_each_route(&dest_sot(), |r| routes.push(r));
        assert_eq!(routes.len(), 1);
        assert_eq!(routes[0].neighbour, nbr_a(), "the quality route is kept; the INP3-only (q0) route is evicted");
    }

    // ─── Coexistence with quality routes ───

    #[test]
    fn inp3_ingestion_attaches_a_time_metric_to_an_existing_quality_route_without_disturbing_it() {
        let mut table = Table::with_defaults();
        table.ingest(nbr_a(), me(), port(), &nodes("RDG", &[(dest_sot(), "SOT", nbr_a(), 200)]), 0);
        let quality_only = route_via(&table, dest_sot(), nbr_a()).unwrap();
        assert!(quality_only.inp3.is_none());
        let q = quality_only.quality;
        let obs = quality_only.obsolescence;

        table.ingest_rif(nbr_a(), me(), 50, &rif(vec![rip(dest_sot(), 1, 100)]), HOP_LIMIT);

        let both = route_via(&table, dest_sot(), nbr_a()).unwrap();
        assert_eq!(both.quality, q, "the quality metric is untouched");
        assert_eq!(both.obsolescence, obs, "the obsolescence of the quality route is untouched");
        assert!(both.inp3.is_some());
        assert_eq!(both.inp3.unwrap().target_time_ms, 160);
    }

    #[test]
    fn a_nodes_refresh_does_not_wipe_a_coexisting_inp3_metric() {
        let mut table = Table::with_defaults();
        table.ingest(nbr_a(), me(), port(), &nodes("RDG", &[(dest_sot(), "SOT", nbr_a(), 200)]), 0);
        table.ingest_rif(nbr_a(), me(), 50, &rif(vec![rip(dest_sot(), 1, 100)]), HOP_LIMIT);
        assert!(route_via(&table, dest_sot(), nbr_a()).unwrap().inp3.is_some());

        table.ingest(nbr_a(), me(), port(), &nodes("RDG", &[(dest_sot(), "SOT", nbr_a(), 100)]), 0);

        let route = route_via(&table, dest_sot(), nbr_a()).unwrap();
        assert!(route.inp3.is_some(), "a NODES quality refresh must not wipe the time-route");
        assert_eq!(route.inp3.unwrap().target_time_ms, 160);
        assert_eq!(route.quality, quality::combine(100, 192), "the quality is the refreshed value");
    }

    #[test]
    fn a_quality_route_and_a_distinct_time_route_coexist_under_one_destination() {
        let mut table = Table::with_defaults();
        table.ingest(nbr_a(), me(), port(), &nodes("RDG", &[(dest_sot(), "SOT", nbr_a(), 200)]), 0);
        table.ingest_rif(nbr_b(), me(), 20, &rif(vec![rip(dest_sot(), 1, 100)]), HOP_LIMIT);

        let routes = routes_of(&table, dest_sot());
        assert_eq!(routes.len(), 2, "one route per next-hop neighbour, two distinct metric carriers");
        let a = routes.iter().find(|r| r.neighbour == nbr_a()).unwrap();
        let b = routes.iter().find(|r| r.neighbour == nbr_b()).unwrap();
        assert!(a.inp3.is_none(), "the NbrA route is quality-only");
        assert!(a.quality > 0);
        assert!(b.inp3.is_some(), "the NbrB route is time-only");
        assert_eq!(b.quality, 0);
    }

    // ─────────────────────────── Inp3BuildRifTests ──────────────────────────

    fn rip_for(rif: &Inp3Rif, dest: Callsign) -> Inp3Rip {
        rif.rips.iter().find(|r| r.destination == dest).cloned().expect("RIP present for dest")
    }

    #[test]
    fn empty_table_emits_just_the_own_node_rip_at_zero_zero() {
        let table = Table::with_defaults();
        let rif = table.build_rif(me(), nbr_a(), &[]);

        assert_eq!(rif.rips.len(), 1, "an empty table advertises only our own-node source RIP");
        let own = &rif.rips[0];
        assert_eq!(own.destination, me());
        assert_eq!(own.target_time_ms, 0);
        assert_eq!(own.hop_count, 0);
        assert!(own.tlvs.is_empty(), "alias TLV emission is gated off");
        assert!(!own.is_horizon(), "the source is never poisoned");
    }

    #[test]
    fn own_node_rip_is_always_first_and_is_zero_zero_regardless_of_table_state() {
        let mut table = Table::with_defaults();
        table.ingest_rif(nbr_a(), me(), 50, &rif(vec![rip(dest_sot(), 1, 100)]), HOP_LIMIT);
        table.ingest_rif(nbr_b(), me(), 20, &rif(vec![rip(dest_mnc(), 1, 100)]), HOP_LIMIT);

        for toward in [nbr_a(), nbr_b()] {
            let rif = table.build_rif(me(), toward, &[]);
            assert_eq!(rif.rips[0].destination, me(), "the own-node RIP is always first");
            assert_eq!(rif.rips[0].target_time_ms, 0);
            assert_eq!(rif.rips[0].hop_count, 0);
            assert_eq!(rif.rips.iter().filter(|r| r.destination == me()).count(), 1, "exactly one own-node RIP");
        }
    }

    #[test]
    fn a_rif_built_toward_us_never_poisons_our_own_node() {
        let mut table = Table::with_defaults();
        table.ingest_rif(nbr_a(), me(), 50, &rif(vec![rip(dest_sot(), 1, 100)]), HOP_LIMIT);

        let rif = table.build_rif(me(), me(), &[]);
        let own = rif.rips.iter().find(|r| r.destination == me()).unwrap();
        assert_eq!(own.target_time_ms, 0, "our own node is exempt from poison-reverse — always 0/0");
        assert!(!own.is_horizon());
    }

    #[test]
    fn each_selected_inp3_route_becomes_one_destination_rip_at_its_quantised_target_time() {
        let mut table = Table::with_defaults();
        // 100 + 73 + 10 = 183 stored; emitted floored to the 10 ms granule → 180.
        table.ingest_rif(nbr_a(), me(), 73, &rif(vec![rip(dest_sot(), 1, 100)]), HOP_LIMIT);

        let rif = table.build_rif(me(), nbr_b(), &[]); // toward a DIFFERENT neighbour → no poison

        let r = rip_for(&rif, dest_sot());
        assert_eq!(r.target_time_ms, 180, "183 stored, quantised to the 10 ms granule");
        assert_eq!(r.hop_count, 2, "the selected route's local hop count (peer 1 + 1 through us)");
        assert!(r.tlvs.is_empty(), "no alias TLV (gated off)");
        assert!(!r.is_horizon());
    }

    #[test]
    fn a_quality_only_destination_is_not_in_the_rif() {
        let mut table = Table::with_defaults();
        table.ingest(nbr_a(), me(), port(), &nodes("RDG", &[(dest_sot(), "SOT", nbr_a(), 200)]), 0);

        let rif = table.build_rif(me(), nbr_b(), &[]);
        assert_eq!(rif.rips.len(), 1, "only the own-node RIP — the quality-only dest is carried by NODES");
        assert_eq!(rif.rips[0].destination, me());
    }

    #[test]
    fn destination_rips_are_ordered_by_ascending_target_time_then_callsign_after_the_own_node_rip() {
        let mut table = Table::with_defaults();
        table.ingest_rif(nbr_a(), me(), 200, &rif(vec![rip(dest_sot(), 1, 100)]), HOP_LIMIT); // 310
        table.ingest_rif(nbr_a(), me(), 20, &rif(vec![rip(dest_mnc(), 1, 100)]), HOP_LIMIT); // 130

        let rif = table.build_rif(me(), nbr_b(), &[]);
        assert_eq!(rif.rips[0].destination, me(), "own-node RIP first");
        assert_eq!(rif.rips[1].destination, dest_mnc(), "then the lowest target time (130)");
        assert_eq!(rif.rips[2].destination, dest_sot(), "then the slower one (310)");
    }

    #[test]
    fn a_dest_via_n_is_poisoned_at_the_horizon_in_the_rif_toward_n() {
        let mut table = Table::with_defaults();
        table.ingest_rif(nbr_a(), me(), 50, &rif(vec![rip(dest_sot(), 1, 100)]), HOP_LIMIT);

        let toward_a = table.build_rif(me(), nbr_a(), &[]);
        let r = rip_for(&toward_a, dest_sot());
        assert_eq!(r.target_time_ms, Inp3Rip::HORIZON_MS, "SOT is via NbrA — poison it back at the horizon");
        assert!(r.is_horizon());
    }

    #[test]
    fn the_same_dest_is_finite_in_the_rif_toward_a_different_neighbour() {
        let mut table = Table::with_defaults();
        table.ingest_rif(nbr_a(), me(), 50, &rif(vec![rip(dest_sot(), 1, 100)]), HOP_LIMIT);

        let toward_a = table.build_rif(me(), nbr_a(), &[]);
        let toward_b = table.build_rif(me(), nbr_b(), &[]);

        assert_eq!(rip_for(&toward_a, dest_sot()).target_time_ms, Inp3Rip::HORIZON_MS, "poisoned toward its own next hop");
        assert_eq!(rip_for(&toward_b, dest_sot()).target_time_ms, 160, "advertised at its real time (100+50+10)");
        assert!(!rip_for(&toward_b, dest_sot()).is_horizon());
    }

    #[test]
    fn poison_reverse_covers_every_kept_next_hop_not_just_the_best() {
        let mut table = Table::with_defaults();
        table.ingest_rif(nbr_a(), me(), 200, &rif(vec![rip(dest_sot(), 1, 100)]), HOP_LIMIT); // 310 via NbrA
        table.ingest_rif(nbr_b(), me(), 20, &rif(vec![rip(dest_sot(), 1, 100)]), HOP_LIMIT); // 130 via NbrB

        assert!(rip_for(&table.build_rif(me(), nbr_a(), &[]), dest_sot()).is_horizon(), "poison toward NbrA");
        assert!(rip_for(&table.build_rif(me(), nbr_b(), &[]), dest_sot()).is_horizon(), "poison toward NbrB too");

        let r = rip_for(&table.build_rif(me(), cs("GB7ZZZ"), &[]), dest_sot());
        assert!(!r.is_horizon(), "toward a non-next-hop neighbour, SOT is advertised finite");
        assert_eq!(r.target_time_ms, 130, "at the best (lowest) INP3 target time we hold");
    }

    #[test]
    fn emitter_never_advertises_a_finite_metric_back_to_a_routes_own_next_hop() {
        let mut table = Table::with_defaults();
        let n1 = cs("GB7AAA");
        let n2 = cs("GB7BBB");
        let d1 = cs("GB7DDD");
        let d2 = cs("GB7EEE");
        let d3 = cs("GB7FFF");

        table.ingest_rif(n1, me(), 10, &rif(vec![rip(d1, 1, 100)]), HOP_LIMIT); // d1 via n1
        table.ingest_rif(n2, me(), 10, &rif(vec![rip(d2, 1, 100)]), HOP_LIMIT); // d2 via n2
        table.ingest_rif(n1, me(), 10, &rif(vec![rip(d3, 1, 100)]), HOP_LIMIT); // d3 via n1

        for toward in [n1, n2] {
            let rif = table.build_rif(me(), toward, &[]);
            for r in &rif.rips {
                if r.destination == me() {
                    assert!(!r.is_horizon(), "the own-node RIP is never poisoned");
                    continue;
                }
                let via_toward = routes_of(&table, r.destination).iter().any(|x| x.neighbour == toward);
                if via_toward {
                    assert_eq!(r.target_time_ms, Inp3Rip::HORIZON_MS, "via the target — must be poisoned");
                } else {
                    assert!(!r.is_horizon(), "not via the target — must be finite");
                }
            }
        }
    }

    #[test]
    fn a_held_inp3_route_is_advertised_regardless_of_the_forwarding_preference() {
        let mut table = Table::with_defaults();
        table.ingest_rif(nbr_a(), me(), 50, &rif(vec![rip(dest_sot(), 1, 100)]), HOP_LIMIT); // 160 via NbrA

        let rif = table.build_rif(me(), nbr_b(), &[]); // toward a different neighbour → finite
        let r = rip_for(&rif, dest_sot());
        assert_eq!(r.destination, dest_sot());
        assert!(!r.is_horizon());
        assert_eq!(r.target_time_ms, 160, "the held INP3 route is advertised at its target time");
    }

    // ─────────────────────── Inp3RecentlyWithdrawnTests ─────────────────────

    #[test]
    fn ingesting_a_horizon_rip_withdraws_the_last_inp3_route_and_records_it() {
        let mut table = Table::with_defaults();
        table.ingest_rif(nbr_a(), me(), 50, &rif(vec![rip(dest_sot(), 1, 100)]), HOP_LIMIT);
        assert!(table.recently_withdrawn().is_empty(), "learning a route is not a withdrawal");

        table.ingest_rif(
            nbr_a(),
            me(),
            50,
            &rif(vec![rip(dest_sot(), 1, Inp3Rip::HORIZON_MS)]),
            HOP_LIMIT,
        );

        assert_eq!(table.recently_withdrawn(), vec![dest_sot()], "SOT lost its last INP3 route at the horizon");
    }

    #[test]
    fn mark_neighbour_down_records_a_destination_that_loses_its_last_inp3_route() {
        let mut table = Table::with_defaults();
        table.ingest_rif(nbr_a(), me(), 50, &rif(vec![rip(dest_sot(), 1, 100)]), HOP_LIMIT);

        table.mark_neighbour_down(&nbr_a());

        assert_eq!(table.recently_withdrawn(), vec![dest_sot()], "dropping NbrA removed SOT's only INP3 route");
    }

    #[test]
    fn sweep_records_a_destination_whose_last_inp3_route_ages_out() {
        let mut table = Table::with_defaults();
        table.ingest_rif(nbr_a(), me(), 50, &rif(vec![rip(dest_sot(), 1, 100)]), HOP_LIMIT);

        // OBSINIT default is 6 → sweep it down to 0 to purge the route.
        for _ in 0..6 {
            table.sweep();
        }

        assert_eq!(table.recently_withdrawn(), vec![dest_sot()], "SOT's only INP3 route aged out");
    }

    #[test]
    fn a_destination_that_keeps_another_inp3_route_is_not_withdrawn() {
        let mut table = Table::with_defaults();
        table.ingest_rif(nbr_a(), me(), 50, &rif(vec![rip(dest_sot(), 1, 100)]), HOP_LIMIT);
        table.ingest_rif(nbr_b(), me(), 20, &rif(vec![rip(dest_sot(), 1, 100)]), HOP_LIMIT);

        table.mark_neighbour_down(&nbr_a());

        assert!(table.recently_withdrawn().is_empty(), "SOT still has an INP3 route via NbrB");
    }

    #[test]
    fn withdrawing_one_route_when_another_inp3_route_survives_does_not_record() {
        let mut table = Table::with_defaults();
        table.ingest_rif(nbr_a(), me(), 50, &rif(vec![rip(dest_sot(), 1, 100)]), HOP_LIMIT);
        table.ingest_rif(nbr_b(), me(), 20, &rif(vec![rip(dest_sot(), 1, 100)]), HOP_LIMIT);

        table.ingest_rif(
            nbr_a(),
            me(),
            50,
            &rif(vec![rip(dest_sot(), 1, Inp3Rip::HORIZON_MS)]),
            HOP_LIMIT,
        );

        assert!(table.recently_withdrawn().is_empty(), "an INP3 route to SOT still exists via NbrB");
    }

    #[test]
    fn a_quality_only_mark_neighbour_down_never_populates_the_set() {
        let mut table = Table::with_defaults();
        table.ingest(nbr_a(), me(), port(), &nodes("RDG", &[(dest_sot(), "SOT", nbr_a(), 200)]), 0);

        table.mark_neighbour_down(&nbr_a());

        assert!(table.recently_withdrawn().is_empty(), "a quality-only neighbour-down must never touch the set");
    }

    #[test]
    fn a_quality_only_sweep_never_populates_the_set() {
        let mut table = Table::with_defaults();
        table.ingest(nbr_a(), me(), port(), &nodes("RDG", &[(dest_sot(), "SOT", nbr_a(), 200)]), 0);

        for _ in 0..10 {
            table.sweep();
        }

        assert!(table.recently_withdrawn().is_empty(), "a quality-only sweep must never touch the set");
    }

    #[test]
    fn a_route_that_keeps_its_quality_after_inp3_withdrawal_is_still_recorded() {
        let mut table = Table::with_defaults();
        table.ingest(nbr_a(), me(), port(), &nodes("RDG", &[(dest_sot(), "SOT", nbr_a(), 200)]), 0);
        table.ingest_rif(nbr_a(), me(), 50, &rif(vec![rip(dest_sot(), 1, 100)]), HOP_LIMIT);

        table.ingest_rif(
            nbr_a(),
            me(),
            50,
            &rif(vec![rip(dest_sot(), 1, Inp3Rip::HORIZON_MS)]),
            HOP_LIMIT,
        );

        assert_eq!(table.recently_withdrawn(), vec![dest_sot()]);
        assert!(has_dest(&table, dest_sot()), "the quality route survives — SOT still reachable by NODES");
    }

    #[test]
    fn build_rif_emits_one_horizon_rip_for_each_withdrawn_destination_in_the_snapshot() {
        let mut table = Table::with_defaults();
        table.ingest_rif(nbr_a(), me(), 50, &rif(vec![rip(dest_sot(), 1, 100)]), HOP_LIMIT);
        table.mark_neighbour_down(&nbr_a()); // SOT withdrawn

        let snapshot = table.recently_withdrawn();
        let rif = table.build_rif(me(), nbr_b(), &snapshot);

        let sot_rips: Vec<&Inp3Rip> = rif.rips.iter().filter(|r| r.destination == dest_sot()).collect();
        assert_eq!(sot_rips.len(), 1);
        assert_eq!(sot_rips[0].target_time_ms, Inp3Rip::HORIZON_MS, "an explicit one-shot horizon withdrawal");
        assert!(sot_rips[0].is_horizon());
    }

    #[test]
    fn build_rif_with_no_snapshot_omits_withdrawals() {
        let mut table = Table::with_defaults();
        table.ingest_rif(nbr_a(), me(), 50, &rif(vec![rip(dest_sot(), 1, 100)]), HOP_LIMIT);
        table.mark_neighbour_down(&nbr_a());

        let rif = table.build_rif(me(), nbr_b(), &[]);
        assert!(!rif.rips.iter().any(|r| r.destination == dest_sot()), "no snapshot ⇒ no horizon withdrawals");
        assert_eq!(table.recently_withdrawn().len(), 1, "build_rif does not consume the set — only the drain does");
    }

    #[test]
    fn the_drained_snapshot_carries_the_withdrawal_to_every_neighbour() {
        let mut table = Table::with_defaults();
        table.ingest_rif(nbr_a(), me(), 50, &rif(vec![rip(dest_sot(), 1, 100)]), HOP_LIMIT);
        table.mark_neighbour_down(&nbr_a());

        let snapshot = table.drain_recently_withdrawn();
        let toward_b = table.build_rif(me(), nbr_b(), &snapshot);
        let toward_c = table.build_rif(me(), cs("GB7ZZZ"), &snapshot);

        assert!(toward_b.rips.iter().find(|r| r.destination == dest_sot()).unwrap().is_horizon());
        assert!(toward_c.rips.iter().find(|r| r.destination == dest_sot()).unwrap().is_horizon());
        assert!(table.recently_withdrawn().is_empty(), "the drain cleared the live set atomically");
    }

    #[test]
    fn drain_recently_withdrawn_returns_then_empties_so_a_later_rif_omits_the_withdrawal() {
        let mut table = Table::with_defaults();
        table.ingest_rif(nbr_a(), me(), 50, &rif(vec![rip(dest_sot(), 1, 100)]), HOP_LIMIT);
        table.mark_neighbour_down(&nbr_a());

        let drained = table.drain_recently_withdrawn();
        assert_eq!(drained, vec![dest_sot()], "the drain returns the snapshot");
        assert!(table.build_rif(me(), nbr_b(), &drained).rips.iter().any(|r| r.destination == dest_sot()), "the round carries it once");

        assert!(table.recently_withdrawn().is_empty(), "the drain cleared the set");
        let empty = table.drain_recently_withdrawn();
        assert!(!table.build_rif(me(), nbr_b(), &empty).rips.iter().any(|r| r.destination == dest_sot()), "absent after the drain");
    }

    #[test]
    fn a_re_learned_destination_is_carried_finite_not_poisoned_in_the_same_round() {
        let mut table = Table::with_defaults();
        table.ingest_rif(nbr_a(), me(), 50, &rif(vec![rip(dest_sot(), 1, 100)]), HOP_LIMIT);
        table.mark_neighbour_down(&nbr_a());
        assert!(table.recently_withdrawn().contains(&dest_sot()));

        // Re-learned via NbrB in the SAME round (before the host drains).
        table.ingest_rif(nbr_b(), me(), 20, &rif(vec![rip(dest_sot(), 1, 100)]), HOP_LIMIT);

        // Toward NbrA: SOT is NOT via NbrA anymore → finite, not poisoned.
        let snapshot = table.recently_withdrawn();
        let rif = table.build_rif(me(), nbr_a(), &snapshot);
        let sot_rips: Vec<&Inp3Rip> = rif.rips.iter().filter(|r| r.destination == dest_sot()).collect();
        assert_eq!(sot_rips.len(), 1, "exactly one RIP for SOT — finite, not both finite + horizon");
        assert!(!sot_rips[0].is_horizon(), "re-learned finite — carried by its real metric");
        assert_eq!(sot_rips[0].target_time_ms, 130, "100 + 20 + 10, quantised");
    }

    #[test]
    fn the_own_node_is_never_emitted_as_a_withdrawal() {
        let mut table = Table::with_defaults();
        table.ingest_rif(nbr_a(), me(), 50, &rif(vec![rip(dest_sot(), 1, 100)]), HOP_LIMIT);
        table.mark_neighbour_down(&nbr_a());

        let snapshot = table.recently_withdrawn();
        let rif = table.build_rif(me(), nbr_b(), &snapshot);
        assert!(!rif.rips.iter().find(|r| r.destination == me()).unwrap().is_horizon(), "our own node is never withdrawn");
        assert_eq!(rif.rips[0].destination, me(), "own-node RIP first, at 0/0");
    }

    #[test]
    fn re_withdrawing_after_a_drain_re_populates_the_set() {
        let mut table = Table::with_defaults();
        table.ingest_rif(nbr_a(), me(), 50, &rif(vec![rip(dest_sot(), 1, 100)]), HOP_LIMIT);
        table.mark_neighbour_down(&nbr_a());
        table.drain_recently_withdrawn();
        assert!(table.recently_withdrawn().is_empty());

        table.ingest_rif(nbr_b(), me(), 20, &rif(vec![rip(dest_sot(), 1, 100)]), HOP_LIMIT);
        table.mark_neighbour_down(&nbr_b());

        assert_eq!(table.recently_withdrawn(), vec![dest_sot()]);
    }

    #[test]
    fn multiple_withdrawn_destinations_are_returned_in_stable_ordinal_order() {
        let mut table = Table::with_defaults();
        table.ingest_rif(
            nbr_a(),
            me(),
            50,
            &rif(vec![rip(dest_sot(), 1, 100), rip(dest_mnc(), 1, 100)]),
            HOP_LIMIT,
        );
        table.mark_neighbour_down(&nbr_a()); // both lose their last INP3 route

        assert_eq!(
            table.recently_withdrawn(),
            vec![dest_mnc(), dest_sot()],
            "GB7MNC < GB7SOT ordinally — stable, cross-stack-comparable order"
        );
    }
}
