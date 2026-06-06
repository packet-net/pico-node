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
use crate::netrom::wire::NetRomPacket;

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

    // 4. Next hop: the destination's best route whose neighbour is not the one it
    //    arrived from.
    match routing.best_route_excluding(&packet.network.destination, received_from) {
        Some(next_hop) => ForwardDecision {
            outcome: ForwardOutcome::ForwardTo,
            next_hop: Some(next_hop),
            time_to_live: capped,
        },
        None => drop(ForwardOutcome::DropNoRoute),
    }
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
    //! mock (only `best_route_excluding` is consulted).

    use super::*;
    use crate::netrom::routing::{NetRomDestination, NetRomNeighbour};
    use crate::netrom::wire::{NetRomNetworkHeader, NetRomOpcode, NetRomTransportHeader};
    use alloc::vec::Vec;

    fn call(text: &str) -> Callsign {
        Callsign::new(text.as_bytes(), 0).unwrap()
    }

    /// A routing view that knows the best-first routes to one destination.
    struct MockRouting {
        dest: Callsign,
        routes: Vec<Callsign>,
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
            self.routes.iter().copied().find(|n| n != exclude)
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
            routes: routes.to_vec(),
        }
    }

    #[test]
    fn forwards_a_transit_datagram_to_the_best_next_hop_with_the_ttl_decremented() {
        let payload = [0xDE, 0xAD, 0xBE, 0xEF];
        let packet = datagram(call("GB7AAA"), call("GB7CCC"), 10, &payload);
        let routing = routes_to(call("GB7CCC"), &[call("GB7CCC")]);

        let d = decide_forward(&packet, &call("GB7AAA"), &call("GB7BBB"), &routing, 25);

        assert_eq!(d.outcome, ForwardOutcome::ForwardTo);
        assert_eq!(d.next_hop, Some(call("GB7CCC")));
        assert_eq!(d.time_to_live, 9);
    }

    #[test]
    fn drops_when_the_ttl_reaches_zero() {
        let payload = [1, 2, 3];
        let packet = datagram(call("GB7AAA"), call("GB7CCC"), 1, &payload);
        let d = decide_forward(&packet, &call("GB7AAA"), &call("GB7BBB"), &routes_to(call("GB7CCC"), &[call("GB7CCC")]), 25);
        assert_eq!(d.outcome, ForwardOutcome::DropTtlExpired);
    }

    #[test]
    fn caps_the_ttl_at_the_configured_maximum() {
        let payload = [1];
        let packet = datagram(call("GB7AAA"), call("GB7CCC"), 200, &payload);
        let d = decide_forward(&packet, &call("GB7AAA"), &call("GB7BBB"), &routes_to(call("GB7CCC"), &[call("GB7CCC")]), 25);
        assert_eq!(d.outcome, ForwardOutcome::ForwardTo);
        assert_eq!(d.time_to_live, 25);
    }

    #[test]
    fn drops_a_datagram_that_looped_back_to_its_origin() {
        let payload = [1];
        let packet = datagram(call("GB7BBB"), call("GB7CCC"), 10, &payload);
        let d = decide_forward(&packet, &call("GB7AAA"), &call("GB7BBB"), &routes_to(call("GB7CCC"), &[call("GB7CCC")]), 25);
        assert_eq!(d.outcome, ForwardOutcome::DropLooped);
    }

    #[test]
    fn drops_when_there_is_no_route_to_the_destination() {
        let payload = [1];
        let packet = datagram(call("GB7AAA"), call("GB7CCC"), 10, &payload);
        let d = decide_forward(&packet, &call("GB7AAA"), &call("GB7BBB"), &routes_to(call("GB7ZZZ"), &[call("GB7ZZZ")]), 25);
        assert_eq!(d.outcome, ForwardOutcome::DropNoRoute);
    }

    #[test]
    fn does_not_bounce_a_datagram_back_to_the_neighbour_it_arrived_from() {
        // the only route to the destination is back via the neighbour it came from
        let payload = [1];
        let packet = datagram(call("GB7AAA"), call("GB7CCC"), 10, &payload);
        let from = call("GB7AAA");
        let d = decide_forward(&packet, &from, &call("GB7BBB"), &routes_to(call("GB7CCC"), &[from]), 25);
        assert_eq!(d.outcome, ForwardOutcome::DropNoRoute);
    }

    #[test]
    fn prefers_an_alternate_route_when_the_best_is_the_way_it_came() {
        let payload = [1];
        let packet = datagram(call("GB7AAA"), call("GB7CCC"), 10, &payload);
        let from = call("GB7AAA");
        // best route is back the way it came (from); a lower-quality alternate is used
        let d = decide_forward(&packet, &from, &call("GB7BBB"), &routes_to(call("GB7CCC"), &[from, call("GB7DDD")]), 25);
        assert_eq!(d.outcome, ForwardOutcome::ForwardTo);
        assert_eq!(d.next_hop, Some(call("GB7DDD")));
    }
}

