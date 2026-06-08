//! The NET/ROM L3 **forwarding decision** — what a transit node does with a datagram
//! whose destination is *not* itself: drop it, or forward it (with a decremented,
//! capped TTL) to a next-hop neighbour. Pure (no I/O): the connector feeds it the
//! datagram, the neighbour it arrived from, this node's callsign, the routing view,
//! and the TTL cap, then emits the interlink send for a [`ForwardOutcome::ForwardTo`].
//!
//! Mirrors the C# `Packet.NetRom.NetRomForwarding.Decide` (the runtime reference) and
//! the de-facto LinBPQ `L4Code.c` forward routine: decrement the hop limit and
//! discard at zero; cap the TTL on everything sent; drop a datagram that has looped
//! back to its own origin; resolve the destination's best route whose neighbour is
//! not the one it just arrived from (never bounce it straight back); otherwise
//! forward. The caller has already established the datagram is not addressed to this
//! node (the "for us" check terminates locally before forwarding is considered).
//!
//! INP3 forwarding-by-time (slice I-3): when the node's `prefer_inp3_routes` knob is
//! set (BPQ's `PREFERINP3ROUTES`) and the destination holds an INP3 time-route, the
//! datagram is forwarded over the lowest-measured-target-time route instead of the
//! quality next-hop (the way it came still excluded), falling back to quality when no
//! INP3 route is usable. The knob defaults off — degenerate-to-today, byte-for-byte.
//! Mirrors the C# `NetRomForwarding.Decide`'s `preferInp3Routes` / `SelectInp3NextHop`.

use crate::ax25::Callsign;
use crate::netrom::routing::model::NetRomRoute;
use crate::netrom::routing::NetRomRoutingView;
use crate::netrom::wire::{write_shifted, NetRomPacket, SHIFTED_LENGTH};

/// The route-selection policy a forwarding node uses when a destination has more than
/// one kept route. Mirrors the C# `NetRomForwardMode`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ForwardMode {
    /// Always the single best route (bounce-back excluded). Deterministic — every
    /// transit datagram for a destination takes the same path.
    BestRoute,
    /// Per-flow quality-weighted spread: every datagram of one L4 circuit hashes to
    /// the same route (so its ordering is preserved), while distinct circuits
    /// distribute across the kept routes in proportion to quality. Stateless. The
    /// default.
    #[default]
    PerFlow,
}

/// What [`decide_forward`] determined should happen to a datagram.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ForwardOutcome {
    /// Forward it (with the rewritten TTL) to [`ForwardDecision::next_hop`].
    ForwardTo,
    /// Drop: the hop limit reached zero.
    DropTtlExpired,
    /// Drop: the datagram's origin is this node — it has looped back.
    DropLooped,
    /// Drop: no onward route to the destination (excluding the way it came).
    DropNoRoute,
}

/// The outcome of a forwarding decision. When [`ForwardOutcome::ForwardTo`],
/// [`next_hop`](Self::next_hop) is `Some` and [`time_to_live`](Self::time_to_live) is
/// the rewritten (decremented + capped) hop limit to stamp into the forwarded header.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ForwardDecision {
    /// The verdict.
    pub outcome: ForwardOutcome,
    /// The neighbour to forward to (`Some` iff forwarding).
    pub next_hop: Option<Callsign>,
    /// The rewritten TTL to stamp into the forwarded datagram (meaningful iff forwarding).
    pub time_to_live: u8,
}

/// Decide what to do with a transit datagram. The caller has already confirmed
/// `packet`'s destination is not `node_call`.
///
/// * `received_from` — the neighbour the datagram arrived from (so it is not bounced
///   straight back to it).
/// * `node_call` — this node's callsign (for the loop guard).
/// * `routing` — the current routing view.
/// * `max_time_to_live` — the TTL cap applied to everything forwarded (the node's
///   configured initial TTL — BPQ's `L3LIVES`).
/// * `prefer_inp3_routes` — the resolved INP3 forwarding preference (BPQ's
///   `PREFERINP3ROUTES`; [`crate::netrom::wire::NetRomInp3Options::prefer_inp3_routes`]).
///   When `true` and the destination holds at least one INP3 time-route, the datagram
///   is forwarded over the **lowest-target-time** INP3 route (the way it came
///   excluded), falling back to the quality next-hop only when no INP3 route is usable.
///   When `false` (the default) the INP3 metric is ignored entirely and selection is
///   byte-for-byte today's quality path. Mirrors the C# `NetRomForwarding.Decide`
///   `preferInp3Routes` argument.
pub fn decide_forward(
    packet: &NetRomPacket,
    received_from: &Callsign,
    node_call: &Callsign,
    routing: &dyn NetRomRoutingView,
    max_time_to_live: u8,
    mode: ForwardMode,
    prefer_inp3_routes: bool,
) -> ForwardDecision {
    // 1. Decrement the hop limit; a datagram that arrives at TTL 1 (or 0) is at the
    //    end of its life and must not be forwarded.
    let decremented = packet.network.time_to_live.saturating_sub(1);
    if decremented == 0 {
        return drop(ForwardOutcome::DropTtlExpired);
    }

    // 2. Cap the TTL on everything sent, so a buggy/hostile peer can't make a frame
    //    circulate longer than this node's own initial TTL.
    let capped = decremented.min(max_time_to_live);

    // 3. Loop guard: a datagram whose origin is this node has come back to its start —
    //    forwarding it again just loops.
    if packet.network.origin == *node_call {
        return drop(ForwardOutcome::DropLooped);
    }

    // 4. Next hop. When INP3 is preferred and the destination holds a usable
    //    time-route, the lowest-target-time INP3 route wins (the way it came excluded);
    //    otherwise (knob off, or no usable INP3 route) the quality next-hop under the
    //    active mode, exactly as today. Mirrors the C# `Decide`: try `SelectInp3NextHop`
    //    first, then fall back to the quality `SelectNextHop`. Per-flow load-balancing
    //    stays a quality-space concept — the INP3 path always forwards the single
    //    fastest route (spreading flows across slower time-routes would defeat the
    //    measurement), so `mode` is moot once an INP3 route is chosen.
    let dest = &packet.network.destination;
    let mut next_hop = None;
    if prefer_inp3_routes {
        next_hop = routing.inp3_next_hop_excluding(dest, received_from);
    }
    if next_hop.is_none() {
        next_hop = match mode {
            ForwardMode::BestRoute => routing.best_route_excluding(dest, received_from),
            ForwardMode::PerFlow => {
                routing.select_route_excluding(dest, received_from, flow_hash(packet))
            }
        };
    }
    match next_hop {
        Some(next_hop) => ForwardDecision {
            outcome: ForwardOutcome::ForwardTo,
            next_hop: Some(next_hop),
            time_to_live: capped,
        },
        None => drop(ForwardOutcome::DropNoRoute),
    }
}

/// FNV-1a (32-bit) over the flow key — the L3 origin (AX.25-shifted, 7 octets) + the
/// L4 circuit index + id — so every datagram of a circuit hashes identically across
/// its lifetime. Defined byte-for-byte (mod-2^32 wrapping mul) to match the C#/TS
/// ports.
fn flow_hash(packet: &NetRomPacket) -> u32 {
    let mut key = [0u8; SHIFTED_LENGTH + 2];
    let _ = write_shifted(&packet.network.origin, &mut key);
    key[SHIFTED_LENGTH] = packet.transport.circuit_index;
    key[SHIFTED_LENGTH + 1] = packet.transport.circuit_id;

    let mut hash = 0x811c_9dc5u32; // FNV-1a offset basis
    for &b in &key {
        hash ^= b as u32;
        hash = hash.wrapping_mul(0x0100_0193); // FNV-1a prime, mod 2^32
    }
    hash
}

fn drop(outcome: ForwardOutcome) -> ForwardDecision {
    ForwardDecision {
        outcome,
        next_hop: None,
        time_to_live: 0,
    }
}

/// The single lowest-target-time INP3 route whose neighbour isn't `exclude` (the way
/// the datagram came) — the slice-level core of
/// [`NetRomRoutingView::inp3_next_hop_excluding`], for a view that gathers its kept
/// routes (e.g. via `for_each_route`). A single linear
/// scan keeps a running best by the time-space key: lowest [`Inp3RouteMetric`]
/// `target_time_ms`, then lowest `hop_count`, then neighbour callsign ordinal — the
/// time-space mirror of the quality "highest quality, then callsign" ordering, and
/// identical to [`crate::netrom::routing::inp3_route_selector`]'s `is_better_inp3`.
/// Routes with no INP3 metric (pure quality routes) are invisible to the search.
/// Returns `None` when no eligible INP3 route exists. Mirrors the C#
/// `NetRomForwarding.SelectInp3NextHop`.
///
/// [`Inp3RouteMetric`]: crate::netrom::routing::model::Inp3RouteMetric
pub fn select_inp3_next_hop(routes: &[NetRomRoute], exclude: &Callsign) -> Option<Callsign> {
    let mut best: Option<NetRomRoute> = None;
    for route in routes {
        // A pure quality-route, or the way it came — not an eligible INP3 next hop.
        let Some(m) = route.inp3 else { continue };
        if route.neighbour == *exclude {
            continue;
        }
        let better = match best.and_then(|b| b.inp3) {
            None => true,
            Some(b) => {
                m.target_time_ms < b.target_time_ms
                    || (m.target_time_ms == b.target_time_ms && m.hop_count < b.hop_count)
                    || (m.target_time_ms == b.target_time_ms
                        && m.hop_count == b.hop_count
                        && callsign_lt(&route.neighbour, &best.unwrap().neighbour))
            }
        };
        if better {
            best = Some(*route);
        }
    }
    best.map(|b| b.neighbour)
}

/// Ordinal callsign comparison: base bytes then SSID — the INP3 callsign tie-break.
/// Matches the C# `string.CompareOrdinal(callsign.ToString())` and the routing table /
/// `inp3_route_selector`'s own `callsign_lt`.
fn callsign_lt(a: &Callsign, b: &Callsign) -> bool {
    match a.base().cmp(b.base()) {
        core::cmp::Ordering::Less => true,
        core::cmp::Ordering::Greater => false,
        core::cmp::Ordering::Equal => a.ssid() < b.ssid(),
    }
}

#[cfg(test)]
mod tests {
    //! The forwarding-decision matrix, mirrored 1:1 from the C# `NetRomForwardingTests`
    //! / the TS `forwarding.test.ts`. Driven through a minimal [`NetRomRoutingView`]
    //! mock (best_route_excluding / select_route_excluding).

    use super::*;
    use crate::netrom::routing::model::Inp3RouteMetric;
    use crate::netrom::routing::{NetRomDestination, NetRomNeighbour};
    use crate::netrom::wire::{NetRomNetworkHeader, NetRomOpcode, NetRomTransportHeader};
    use alloc::vec::Vec;

    fn call(text: &str) -> Callsign {
        Callsign::new(text.as_bytes(), 0).unwrap()
    }

    /// A routing view that knows the best-first [`NetRomRoute`]s to one destination —
    /// each route carrying quality and (for the INP3 cases) an optional time metric.
    /// Implements [`NetRomRoutingView`] — both the quality-space next-hops and the
    /// INP3 time-space [`NetRomRoutingView::inp3_next_hop_excluding`], mirroring the C#
    /// `NetRomRoutingSnapshot`.
    struct MockRouting {
        dest: Callsign,
        routes: Vec<NetRomRoute>,
    }

    impl NetRomRoutingView for MockRouting {
        fn resolve_destination(&self, _: &str) -> Option<NetRomDestination> {
            None
        }
        fn destination_for(&self, _: &Callsign) -> Option<NetRomDestination> {
            None
        }
        fn neighbour_for(&self, _: &Callsign) -> Option<NetRomNeighbour> {
            None
        }
        fn best_route_excluding(&self, dest: &Callsign, exclude: &Callsign) -> Option<Callsign> {
            if *dest != self.dest {
                return None;
            }
            self.routes
                .iter()
                .find(|r| r.neighbour != *exclude)
                .map(|r| r.neighbour)
        }
        fn select_route_excluding(
            &self,
            dest: &Callsign,
            exclude: &Callsign,
            flow_hash: u32,
        ) -> Option<Callsign> {
            if *dest != self.dest {
                return None;
            }
            let total: u32 = self
                .routes
                .iter()
                .filter(|r| r.neighbour != *exclude && r.quality > 0)
                .map(|r| r.quality as u32)
                .sum();
            if total == 0 {
                return None;
            }
            let mut target = flow_hash % total;
            for r in &self.routes {
                if r.neighbour == *exclude || r.quality == 0 {
                    continue;
                }
                let weight = r.quality as u32;
                if target < weight {
                    return Some(r.neighbour);
                }
                target -= weight;
            }
            None
        }
        fn inp3_next_hop_excluding(&self, dest: &Callsign, exclude: &Callsign) -> Option<Callsign> {
            if *dest != self.dest {
                return None;
            }
            // The slice-level INP3 winner search — the same path the production table
            // takes over its kept routes (`for_each_route`).
            select_inp3_next_hop(&self.routes, exclude)
        }
    }

    fn datagram<'a>(
        origin: Callsign,
        dest: Callsign,
        ttl: u8,
        payload: &'a [u8],
    ) -> NetRomPacket<'a> {
        NetRomPacket {
            network: NetRomNetworkHeader {
                origin,
                destination: dest,
                time_to_live: ttl,
            },
            transport: NetRomTransportHeader {
                circuit_index: 7,
                circuit_id: 9,
                tx_sequence: 3,
                rx_sequence: 4,
                opcode: NetRomOpcode::Information.as_u8(),
                flags: 0,
            },
            payload,
        }
    }

    // A quality-only route (today's vanilla triple; no INP3 metric).
    fn q(neighbour: Callsign, quality: u8) -> NetRomRoute {
        NetRomRoute {
            neighbour,
            quality,
            obsolescence: 6,
            inp3: None,
        }
    }

    // A route carrying both a quality and an INP3 (target-time) metric.
    fn t(neighbour: Callsign, quality: u8, target_time_ms: u32, hop_count: u8) -> NetRomRoute {
        NetRomRoute {
            neighbour,
            quality,
            obsolescence: 6,
            inp3: Some(Inp3RouteMetric {
                target_time_ms,
                hop_count,
            }),
        }
    }

    fn routes_to(dest: Callsign, routes: &[Callsign]) -> MockRouting {
        MockRouting {
            dest,
            routes: routes.iter().map(|n| q(*n, 200)).collect(),
        }
    }

    fn routes_to_weighted(dest: Callsign, routes: &[(Callsign, u8)]) -> MockRouting {
        MockRouting {
            dest,
            routes: routes.iter().map(|(n, ql)| q(*n, *ql)).collect(),
        }
    }

    // Routes carrying BOTH a quality metric (NODES) and an INP3 time-route (RIF). Each
    // entry is (neighbour, quality, target_time_ms, hop_count). Best-quality-first
    // ordering as passed — mirrors the C# `Inp3RoutesTo`.
    fn inp3_routes_to(dest: Callsign, routes: &[(Callsign, u8, u32, u8)]) -> MockRouting {
        MockRouting {
            dest,
            routes: routes
                .iter()
                .map(|(n, ql, time, hop)| t(*n, *ql, *time, *hop))
                .collect(),
        }
    }

    /// A datagram with a chosen flow key (FlowHash keys on the L3 origin + L4 circuit
    /// index/id; vary the index to make distinct flows).
    fn flow<'a>(
        origin: Callsign,
        dest: Callsign,
        ttl: u8,
        circuit_index: u8,
        payload: &'a [u8],
    ) -> NetRomPacket<'a> {
        NetRomPacket {
            network: NetRomNetworkHeader {
                origin,
                destination: dest,
                time_to_live: ttl,
            },
            transport: NetRomTransportHeader {
                circuit_index,
                circuit_id: 0,
                tx_sequence: 0,
                rx_sequence: 0,
                opcode: NetRomOpcode::Information.as_u8(),
                flags: 0,
            },
            payload,
        }
    }

    #[test]
    fn forwards_a_transit_datagram_to_the_best_next_hop_with_the_ttl_decremented() {
        let payload = [0xDE, 0xAD, 0xBE, 0xEF];
        let packet = datagram(call("GB7AAA"), call("GB7CCC"), 10, &payload);
        let routing = routes_to(call("GB7CCC"), &[call("GB7CCC")]);

        let d = decide_forward(
            &packet,
            &call("GB7AAA"),
            &call("GB7BBB"),
            &routing,
            25,
            ForwardMode::BestRoute,
            false,
        );

        assert_eq!(d.outcome, ForwardOutcome::ForwardTo);
        assert_eq!(d.next_hop, Some(call("GB7CCC")));
        assert_eq!(d.time_to_live, 9);
    }

    #[test]
    fn drops_when_the_ttl_reaches_zero() {
        let payload = [1, 2, 3];
        let packet = datagram(call("GB7AAA"), call("GB7CCC"), 1, &payload);
        let d = decide_forward(
            &packet,
            &call("GB7AAA"),
            &call("GB7BBB"),
            &routes_to(call("GB7CCC"), &[call("GB7CCC")]),
            25,
            ForwardMode::BestRoute,
            false,
        );
        assert_eq!(d.outcome, ForwardOutcome::DropTtlExpired);
    }

    #[test]
    fn caps_the_ttl_at_the_configured_maximum() {
        let payload = [1];
        let packet = datagram(call("GB7AAA"), call("GB7CCC"), 200, &payload);
        let d = decide_forward(
            &packet,
            &call("GB7AAA"),
            &call("GB7BBB"),
            &routes_to(call("GB7CCC"), &[call("GB7CCC")]),
            25,
            ForwardMode::BestRoute,
            false,
        );
        assert_eq!(d.outcome, ForwardOutcome::ForwardTo);
        assert_eq!(d.time_to_live, 25);
    }

    #[test]
    fn drops_a_datagram_that_looped_back_to_its_origin() {
        let payload = [1];
        let packet = datagram(call("GB7BBB"), call("GB7CCC"), 10, &payload);
        let d = decide_forward(
            &packet,
            &call("GB7AAA"),
            &call("GB7BBB"),
            &routes_to(call("GB7CCC"), &[call("GB7CCC")]),
            25,
            ForwardMode::BestRoute,
            false,
        );
        assert_eq!(d.outcome, ForwardOutcome::DropLooped);
    }

    #[test]
    fn drops_when_there_is_no_route_to_the_destination() {
        let payload = [1];
        let packet = datagram(call("GB7AAA"), call("GB7CCC"), 10, &payload);
        let d = decide_forward(
            &packet,
            &call("GB7AAA"),
            &call("GB7BBB"),
            &routes_to(call("GB7ZZZ"), &[call("GB7ZZZ")]),
            25,
            ForwardMode::BestRoute,
            false,
        );
        assert_eq!(d.outcome, ForwardOutcome::DropNoRoute);
    }

    #[test]
    fn does_not_bounce_a_datagram_back_to_the_neighbour_it_arrived_from() {
        // the only route to the destination is back via the neighbour it came from
        let payload = [1];
        let packet = datagram(call("GB7AAA"), call("GB7CCC"), 10, &payload);
        let from = call("GB7AAA");
        let d = decide_forward(
            &packet,
            &from,
            &call("GB7BBB"),
            &routes_to(call("GB7CCC"), &[from]),
            25,
            ForwardMode::BestRoute,
            false,
        );
        assert_eq!(d.outcome, ForwardOutcome::DropNoRoute);
    }

    #[test]
    fn prefers_an_alternate_route_when_the_best_is_the_way_it_came() {
        let payload = [1];
        let packet = datagram(call("GB7AAA"), call("GB7CCC"), 10, &payload);
        let from = call("GB7AAA");
        // best route is back the way it came (from); a lower-quality alternate is used
        let d = decide_forward(
            &packet,
            &from,
            &call("GB7BBB"),
            &routes_to(call("GB7CCC"), &[from, call("GB7DDD")]),
            25,
            ForwardMode::BestRoute,
            false,
        );
        assert_eq!(d.outcome, ForwardOutcome::ForwardTo);
        assert_eq!(d.next_hop, Some(call("GB7DDD")));
    }

    // ─── multi-route load-balancing (per-flow, quality-weighted) ────────

    #[test]
    fn per_flow_pins_a_circuit_to_one_route_regardless_of_ttl() {
        // Two equal routes; the same flow (same origin + circuit index/id) takes the
        // same route across datagrams (so the circuit's L4 ordering is preserved).
        let payload = [1u8];
        let routing = routes_to(call("GB7CCC"), &[call("GB7CCC"), call("GB7DDD")]);
        let a = decide_forward(
            &flow(call("GB7AAA"), call("GB7CCC"), 20, 5, &payload),
            &call("GB7AAA"),
            &call("GB7BBB"),
            &routing,
            25,
            ForwardMode::PerFlow,
            false,
        );
        let b = decide_forward(
            &flow(call("GB7AAA"), call("GB7CCC"), 9, 5, &payload),
            &call("GB7AAA"),
            &call("GB7BBB"),
            &routing,
            25,
            ForwardMode::PerFlow,
            false,
        );
        assert_eq!(a.outcome, ForwardOutcome::ForwardTo);
        assert_eq!(
            a.next_hop, b.next_hop,
            "every datagram of one circuit hashes to the same route"
        );
    }

    #[test]
    fn per_flow_spreads_distinct_circuits_across_the_kept_routes() {
        let payload = [1u8];
        let routing = routes_to(call("GB7CCC"), &[call("GB7CCC"), call("GB7DDD")]);
        let (mut seen_c, mut seen_d) = (false, false);
        for i in 0..60u8 {
            let d = decide_forward(
                &flow(call("GB7AAA"), call("GB7CCC"), 20, i, &payload),
                &call("GB7AAA"),
                &call("GB7BBB"),
                &routing,
                25,
                ForwardMode::PerFlow,
                false,
            );
            seen_c |= d.next_hop == Some(call("GB7CCC"));
            seen_d |= d.next_hop == Some(call("GB7DDD"));
        }
        assert!(seen_c && seen_d, "distinct circuits should use both routes");
    }

    #[test]
    fn per_flow_weights_the_spread_by_route_quality() {
        // 2:1 quality -> the higher-quality route carries meaningfully more flows.
        let payload = [1u8];
        let routing = routes_to_weighted(
            call("GB7CCC"),
            &[(call("GB7CCC"), 200), (call("GB7DDD"), 100)],
        );
        let (mut c, mut d) = (0u32, 0u32);
        for i in 0..255u8 {
            let dec = decide_forward(
                &flow(call("GB7AAA"), call("GB7CCC"), 20, i, &payload),
                &call("GB7AAA"),
                &call("GB7BBB"),
                &routing,
                25,
                ForwardMode::PerFlow,
                false,
            );
            if dec.next_hop == Some(call("GB7CCC")) {
                c += 1;
            } else if dec.next_hop == Some(call("GB7DDD")) {
                d += 1;
            }
        }
        assert!(c > 0 && d > 0);
        assert!(
            c > d,
            "the higher-quality route carries proportionally more flows"
        );
    }

    #[test]
    fn best_route_mode_ignores_the_flow_and_always_takes_the_best() {
        let payload = [1u8];
        let routing = routes_to_weighted(
            call("GB7CCC"),
            &[(call("GB7CCC"), 200), (call("GB7DDD"), 100)],
        );
        for i in 0..20u8 {
            let d = decide_forward(
                &flow(call("GB7AAA"), call("GB7CCC"), 20, i, &payload),
                &call("GB7AAA"),
                &call("GB7BBB"),
                &routing,
                25,
                ForwardMode::BestRoute,
                false,
            );
            assert_eq!(
                d.next_hop,
                Some(call("GB7CCC")),
                "BestRoute always uses the best route"
            );
        }
    }

    // ─── INP3 forwarding-by-time (prefer_inp3_routes) ───────────────────
    //
    // Ported 1:1 from the C# `NetRomForwardingTests` INP3 section (and mirrored vs the
    // TS `forwarding.test.ts`). The fixture constants match the C# ones:
    //   Me = GB7BBB (the transit node), Source/FromNbr = GB7AAA (origin + the neighbour
    //   it arrived from), Dest/OnwardNbr = GB7CCC (the destination + the way onward),
    //   AltNbr = GB7DDD (an alternate next hop). Routes carry BOTH a NODES quality and a
    //   RIF-measured INP3 target time; prefer_inp3_routes selects by the lower time.

    #[test]
    fn prefers_the_lowest_target_time_inp3_route_overriding_quality_and_per_flow() {
        // GB7CCC is the best QUALITY route; GB7DDD is the fastest by measured TIME. With
        // prefer_inp3_routes on, every flow forwards over GB7DDD (the time winner) —
        // overriding both the quality ranking AND the per-flow spread.
        let payload = [1u8];
        let routing = inp3_routes_to(
            call("GB7CCC"),
            &[(call("GB7CCC"), 200, 300, 2), (call("GB7DDD"), 100, 100, 3)],
        );
        for i in 0..30u8 {
            let d = decide_forward(
                &flow(call("GB7AAA"), call("GB7CCC"), 20, i, &payload),
                &call("GB7AAA"),
                &call("GB7BBB"),
                &routing,
                25,
                ForwardMode::PerFlow,
                true,
            );
            assert_eq!(d.outcome, ForwardOutcome::ForwardTo);
            assert_eq!(
                d.next_hop,
                Some(call("GB7DDD")),
                "the lowest-target-time INP3 route wins for every flow when INP3 is preferred"
            );
        }

        // Knob off ⇒ quality wins, byte-for-byte today (BestRoute picks the best quality).
        let off = decide_forward(
            &datagram(call("GB7AAA"), call("GB7CCC"), 20, &payload),
            &call("GB7AAA"),
            &call("GB7BBB"),
            &routing,
            25,
            ForwardMode::BestRoute,
            false,
        );
        assert_eq!(
            off.next_hop,
            Some(call("GB7CCC")),
            "prefer_inp3_routes off — quality decides"
        );
    }

    #[test]
    fn prefer_inp3_routes_off_ignores_the_inp3_metric_entirely() {
        // The degenerate-to-today guard: routes carry INP3 metrics that would change the
        // pick, but with the knob off the metric is never read — quality is chosen,
        // identical to a node that never heard of INP3.
        let payload = [1u8];
        let routing = inp3_routes_to(
            call("GB7CCC"),
            &[(call("GB7CCC"), 200, 999, 9), (call("GB7DDD"), 100, 1, 1)],
        );

        let off = decide_forward(
            &datagram(call("GB7AAA"), call("GB7CCC"), 20, &payload),
            &call("GB7AAA"),
            &call("GB7BBB"),
            &routing,
            25,
            ForwardMode::BestRoute,
            false,
        );

        assert_eq!(
            off.next_hop,
            Some(call("GB7CCC")),
            "knob off ⇒ quality wins despite GB7DDD's far lower target time"
        );
    }

    #[test]
    fn falls_back_to_quality_when_preferred_but_no_inp3_route_exists() {
        // prefer_inp3_routes on, but the destination holds only quality routes (no
        // time-route) → fall back to the quality next-hop, exactly as today.
        let payload = [1u8];
        let routing = routes_to_weighted(
            call("GB7CCC"),
            &[(call("GB7CCC"), 200), (call("GB7DDD"), 100)],
        );

        let d = decide_forward(
            &datagram(call("GB7AAA"), call("GB7CCC"), 20, &payload),
            &call("GB7AAA"),
            &call("GB7BBB"),
            &routing,
            25,
            ForwardMode::BestRoute,
            true,
        );

        assert_eq!(
            d.next_hop,
            Some(call("GB7CCC")),
            "no INP3 route to prefer → quality fallback"
        );
    }

    #[test]
    fn excludes_the_inp3_route_that_arrived_from_and_takes_the_next_best_time() {
        // The fastest INP3 route is back the way it came (split-horizon) → excluded; the
        // next lowest-target-time INP3 route is used instead.
        let payload = [1u8];
        // arrived from GB7AAA; that route is the fastest (50 ms) but excluded.
        let routing = inp3_routes_to(
            call("GB7CCC"),
            &[(call("GB7AAA"), 100, 50, 1), (call("GB7CCC"), 200, 300, 2)],
        );

        let d = decide_forward(
            &datagram(call("GB7AAA"), call("GB7CCC"), 20, &payload),
            &call("GB7AAA"),
            &call("GB7BBB"),
            &routing,
            25,
            ForwardMode::PerFlow,
            true,
        );

        assert_eq!(
            d.next_hop,
            Some(call("GB7CCC")),
            "the time winner is the way it came → use the next-best INP3 route"
        );
    }

    #[test]
    fn falls_back_to_quality_when_the_only_inp3_route_is_the_way_it_came() {
        // The single INP3 route is back the way it came (excluded); a quality-only
        // alternate exists → fall back to it rather than dropping.
        let payload = [1u8];
        let routing = MockRouting {
            dest: call("GB7CCC"),
            routes: alloc::vec![
                t(call("GB7AAA"), 100, 50, 1), // INP3, but the way it came
                q(call("GB7DDD"), 200),        // quality-only alternate
            ],
        };

        let d = decide_forward(
            &datagram(call("GB7AAA"), call("GB7CCC"), 20, &payload),
            &call("GB7AAA"),
            &call("GB7BBB"),
            &routing,
            25,
            ForwardMode::BestRoute,
            true,
        );

        assert_eq!(
            d.next_hop,
            Some(call("GB7DDD")),
            "no usable INP3 route (the only one is the way it came) → quality fallback"
        );
    }

    #[test]
    fn inp3_tie_break_is_target_time_then_hop_then_callsign() {
        let payload = [1u8];

        // Two INP3 routes at the same target time: the lower hop count wins. GB7CCC has
        // 2 hops vs GB7DDD's 3 → GB7CCC.
        let by_hop = inp3_routes_to(
            call("GB7CCC"),
            &[(call("GB7DDD"), 200, 100, 3), (call("GB7CCC"), 100, 100, 2)],
        );
        let d = decide_forward(
            &datagram(call("GB7AAA"), call("GB7CCC"), 20, &payload),
            &call("GB7AAA"),
            &call("GB7BBB"),
            &by_hop,
            25,
            ForwardMode::PerFlow,
            true,
        );
        assert_eq!(
            d.next_hop,
            Some(call("GB7CCC")),
            "equal target time → fewer hops wins"
        );

        // Equal time AND hop → lower callsign ordinal wins. GB7CCC < GB7DDD.
        let by_call = inp3_routes_to(
            call("GB7CCC"),
            &[(call("GB7DDD"), 200, 100, 2), (call("GB7CCC"), 100, 100, 2)],
        );
        let d = decide_forward(
            &datagram(call("GB7AAA"), call("GB7CCC"), 20, &payload),
            &call("GB7AAA"),
            &call("GB7BBB"),
            &by_call,
            25,
            ForwardMode::PerFlow,
            true,
        );
        assert_eq!(
            d.next_hop,
            Some(call("GB7CCC")),
            "equal target time + hop → lower callsign ordinal wins"
        );
    }
}
