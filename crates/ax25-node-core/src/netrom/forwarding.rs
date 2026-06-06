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

use crate::ax25::Callsign;
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
pub fn decide_forward(
    packet: &NetRomPacket,
    received_from: &Callsign,
    node_call: &Callsign,
    routing: &dyn NetRomRoutingView,
    max_time_to_live: u8,
    mode: ForwardMode,
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

    // 4. Next hop, under the active mode, among the kept routes (excluding the way it
    //    came): the single best route, or a per-flow quality-weighted pick.
    let dest = &packet.network.destination;
    let next_hop = match mode {
        ForwardMode::BestRoute => routing.best_route_excluding(dest, received_from),
        ForwardMode::PerFlow => routing.select_route_excluding(dest, received_from, flow_hash(packet)),
    };
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

#[cfg(test)]
mod tests {
    //! The forwarding-decision matrix, mirrored 1:1 from the C# `NetRomForwardingTests`
    //! / the TS `forwarding.test.ts`. Driven through a minimal [`NetRomRoutingView`]
    //! mock (best_route_excluding / select_route_excluding).

    use super::*;
    use crate::netrom::routing::{NetRomDestination, NetRomNeighbour};
    use crate::netrom::wire::{NetRomNetworkHeader, NetRomOpcode, NetRomTransportHeader};
    use alloc::vec::Vec;

    fn call(text: &str) -> Callsign {
        Callsign::new(text.as_bytes(), 0).unwrap()
    }

    /// A routing view that knows the best-first routes (neighbour, quality) to one
    /// destination.
    struct MockRouting {
        dest: Callsign,
        routes: Vec<(Callsign, u8)>,
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
            self.routes.iter().find(|(n, _)| n != exclude).map(|(n, _)| *n)
        }
        fn select_route_excluding(&self, dest: &Callsign, exclude: &Callsign, flow_hash: u32) -> Option<Callsign> {
            if *dest != self.dest {
                return None;
            }
            let total: u32 = self
                .routes
                .iter()
                .filter(|(n, q)| n != exclude && *q > 0)
                .map(|(_, q)| *q as u32)
                .sum();
            if total == 0 {
                return None;
            }
            let mut target = flow_hash % total;
            for (n, q) in &self.routes {
                if n == exclude || *q == 0 {
                    continue;
                }
                let weight = *q as u32;
                if target < weight {
                    return Some(*n);
                }
                target -= weight;
            }
            None
        }
    }

    fn datagram<'a>(origin: Callsign, dest: Callsign, ttl: u8, payload: &'a [u8]) -> NetRomPacket<'a> {
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

    fn routes_to(dest: Callsign, routes: &[Callsign]) -> MockRouting {
        MockRouting {
            dest,
            routes: routes.iter().map(|n| (*n, 200u8)).collect(),
        }
    }

    fn routes_to_weighted(dest: Callsign, routes: &[(Callsign, u8)]) -> MockRouting {
        MockRouting {
            dest,
            routes: routes.to_vec(),
        }
    }

    /// A datagram with a chosen flow key (FlowHash keys on the L3 origin + L4 circuit
    /// index/id; vary the index to make distinct flows).
    fn flow<'a>(origin: Callsign, dest: Callsign, ttl: u8, circuit_index: u8, payload: &'a [u8]) -> NetRomPacket<'a> {
        NetRomPacket {
            network: NetRomNetworkHeader { origin, destination: dest, time_to_live: ttl },
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

        let d = decide_forward(&packet, &call("GB7AAA"), &call("GB7BBB"), &routing, 25, ForwardMode::BestRoute);

        assert_eq!(d.outcome, ForwardOutcome::ForwardTo);
        assert_eq!(d.next_hop, Some(call("GB7CCC")));
        assert_eq!(d.time_to_live, 9);
    }

    #[test]
    fn drops_when_the_ttl_reaches_zero() {
        let payload = [1, 2, 3];
        let packet = datagram(call("GB7AAA"), call("GB7CCC"), 1, &payload);
        let d = decide_forward(&packet, &call("GB7AAA"), &call("GB7BBB"), &routes_to(call("GB7CCC"), &[call("GB7CCC")]), 25, ForwardMode::BestRoute);
        assert_eq!(d.outcome, ForwardOutcome::DropTtlExpired);
    }

    #[test]
    fn caps_the_ttl_at_the_configured_maximum() {
        let payload = [1];
        let packet = datagram(call("GB7AAA"), call("GB7CCC"), 200, &payload);
        let d = decide_forward(&packet, &call("GB7AAA"), &call("GB7BBB"), &routes_to(call("GB7CCC"), &[call("GB7CCC")]), 25, ForwardMode::BestRoute);
        assert_eq!(d.outcome, ForwardOutcome::ForwardTo);
        assert_eq!(d.time_to_live, 25);
    }

    #[test]
    fn drops_a_datagram_that_looped_back_to_its_origin() {
        let payload = [1];
        let packet = datagram(call("GB7BBB"), call("GB7CCC"), 10, &payload);
        let d = decide_forward(&packet, &call("GB7AAA"), &call("GB7BBB"), &routes_to(call("GB7CCC"), &[call("GB7CCC")]), 25, ForwardMode::BestRoute);
        assert_eq!(d.outcome, ForwardOutcome::DropLooped);
    }

    #[test]
    fn drops_when_there_is_no_route_to_the_destination() {
        let payload = [1];
        let packet = datagram(call("GB7AAA"), call("GB7CCC"), 10, &payload);
        let d = decide_forward(&packet, &call("GB7AAA"), &call("GB7BBB"), &routes_to(call("GB7ZZZ"), &[call("GB7ZZZ")]), 25, ForwardMode::BestRoute);
        assert_eq!(d.outcome, ForwardOutcome::DropNoRoute);
    }

    #[test]
    fn does_not_bounce_a_datagram_back_to_the_neighbour_it_arrived_from() {
        // the only route to the destination is back via the neighbour it came from
        let payload = [1];
        let packet = datagram(call("GB7AAA"), call("GB7CCC"), 10, &payload);
        let from = call("GB7AAA");
        let d = decide_forward(&packet, &from, &call("GB7BBB"), &routes_to(call("GB7CCC"), &[from]), 25, ForwardMode::BestRoute);
        assert_eq!(d.outcome, ForwardOutcome::DropNoRoute);
    }

    #[test]
    fn prefers_an_alternate_route_when_the_best_is_the_way_it_came() {
        let payload = [1];
        let packet = datagram(call("GB7AAA"), call("GB7CCC"), 10, &payload);
        let from = call("GB7AAA");
        // best route is back the way it came (from); a lower-quality alternate is used
        let d = decide_forward(&packet, &from, &call("GB7BBB"), &routes_to(call("GB7CCC"), &[from, call("GB7DDD")]), 25, ForwardMode::BestRoute);
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
        let a = decide_forward(&flow(call("GB7AAA"), call("GB7CCC"), 20, 5, &payload), &call("GB7AAA"), &call("GB7BBB"), &routing, 25, ForwardMode::PerFlow);
        let b = decide_forward(&flow(call("GB7AAA"), call("GB7CCC"), 9, 5, &payload), &call("GB7AAA"), &call("GB7BBB"), &routing, 25, ForwardMode::PerFlow);
        assert_eq!(a.outcome, ForwardOutcome::ForwardTo);
        assert_eq!(a.next_hop, b.next_hop, "every datagram of one circuit hashes to the same route");
    }

    #[test]
    fn per_flow_spreads_distinct_circuits_across_the_kept_routes() {
        let payload = [1u8];
        let routing = routes_to(call("GB7CCC"), &[call("GB7CCC"), call("GB7DDD")]);
        let (mut seen_c, mut seen_d) = (false, false);
        for i in 0..60u8 {
            let d = decide_forward(&flow(call("GB7AAA"), call("GB7CCC"), 20, i, &payload), &call("GB7AAA"), &call("GB7BBB"), &routing, 25, ForwardMode::PerFlow);
            seen_c |= d.next_hop == Some(call("GB7CCC"));
            seen_d |= d.next_hop == Some(call("GB7DDD"));
        }
        assert!(seen_c && seen_d, "distinct circuits should use both routes");
    }

    #[test]
    fn per_flow_weights_the_spread_by_route_quality() {
        // 2:1 quality -> the higher-quality route carries meaningfully more flows.
        let payload = [1u8];
        let routing = routes_to_weighted(call("GB7CCC"), &[(call("GB7CCC"), 200), (call("GB7DDD"), 100)]);
        let (mut c, mut d) = (0u32, 0u32);
        for i in 0..255u8 {
            let dec = decide_forward(&flow(call("GB7AAA"), call("GB7CCC"), 20, i, &payload), &call("GB7AAA"), &call("GB7BBB"), &routing, 25, ForwardMode::PerFlow);
            if dec.next_hop == Some(call("GB7CCC")) {
                c += 1;
            } else if dec.next_hop == Some(call("GB7DDD")) {
                d += 1;
            }
        }
        assert!(c > 0 && d > 0);
        assert!(c > d, "the higher-quality route carries proportionally more flows");
    }

    #[test]
    fn best_route_mode_ignores_the_flow_and_always_takes_the_best() {
        let payload = [1u8];
        let routing = routes_to_weighted(call("GB7CCC"), &[(call("GB7CCC"), 200), (call("GB7DDD"), 100)]);
        for i in 0..20u8 {
            let d = decide_forward(&flow(call("GB7AAA"), call("GB7CCC"), 20, i, &payload), &call("GB7AAA"), &call("GB7BBB"), &routing, 25, ForwardMode::BestRoute);
            assert_eq!(d.next_hop, Some(call("GB7CCC")), "BestRoute always uses the best route");
        }
    }
}
