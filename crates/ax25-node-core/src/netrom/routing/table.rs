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

use super::model::{NetRomDestination, NetRomNeighbour, NetRomRoute};
use super::options::NetRomRoutingOptions;
use super::quality;
use crate::netrom::wire::{Alias, NodesAdvertisementEntry, NodesBroadcast};
use crate::netrom::PortId;
use alloc::vec::Vec;

/// One kept route inside a destination's route set.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct RouteState {
    neighbour: Callsign,
    quality: u8,
    obsolescence: u8,
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
        for slot in self.destinations.iter_mut() {
            if let Some(dest) = slot {
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
                if dest.route_count() == 0 {
                    *slot = None;
                }
            }
        }
        self.prune_orphan_neighbours();
        purged
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
            }),
            route_count: d.route_count() as u8,
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

        let new_route = RouteState {
            neighbour: via_neighbour,
            quality: q,
            obsolescence: self.options.obsolete_initial,
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
