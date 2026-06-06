//! NET/ROM L3 routing — the learned routing table + its model.
//!
//! Mirrors the C# `Packet.NetRom.Routing` namespace: the multiplicative quality
//! decay ([`quality`]), the route-maintenance knobs ([`NetRomRoutingOptions`]), the
//! fixed-capacity learned table ([`NetRomRoutingTable`]), and the immutable
//! read-side model ([`model`]).
//!
//! Read-only by construction and `no_std`/allocation-free (a `[Option<…>; N]` of
//! fixed-sized arrays, not heap maps). Nothing here transmits.

pub mod model;
pub mod options;
pub mod quality;
pub mod table;

pub use model::{NetRomDestination, NetRomNeighbour, NetRomRoute};
pub use options::NetRomRoutingOptions;
pub use table::NetRomRoutingTable;

use crate::ax25::Callsign;

/// The read-only routing view a connector resolves `connect <target>` and outbound
/// next-hops against — the alias/callsign lookup, the by-callsign destination, and
/// the directly-heard-neighbour check. Implemented by [`NetRomRoutingTable`]; taken
/// as `&dyn` so the connector needn't carry the table's capacity const generics.
/// The TS analogue is the `RoutingSnapshotSource` + the free `resolveDestination` /
/// `neighbourFor` functions.
pub trait NetRomRoutingView {
    /// Resolve a `connect <target>` alias or callsign to a destination (see
    /// [`NetRomRoutingTable::resolve_destination`]).
    fn resolve_destination(&self, target: &str) -> Option<NetRomDestination>;
    /// The destination known for a callsign, if any.
    fn destination_for(&self, call: &Callsign) -> Option<NetRomDestination>;
    /// The directly-heard neighbour for a callsign, if any.
    fn neighbour_for(&self, call: &Callsign) -> Option<NetRomNeighbour>;
}

impl<const MAX_DESTS: usize, const MAX_ROUTES: usize, const MAX_NBRS: usize> NetRomRoutingView
    for NetRomRoutingTable<MAX_DESTS, MAX_ROUTES, MAX_NBRS>
{
    fn resolve_destination(&self, target: &str) -> Option<NetRomDestination> {
        NetRomRoutingTable::resolve_destination(self, target)
    }
    fn destination_for(&self, call: &Callsign) -> Option<NetRomDestination> {
        self.destination(call)
    }
    fn neighbour_for(&self, call: &Callsign) -> Option<NetRomNeighbour> {
        self.neighbour(call)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ax25::Callsign;
    use crate::netrom::wire::test_support::{build, EntrySpec};
    use crate::netrom::wire::NodesBroadcast;
    use crate::netrom::PortId;

    fn call(s: &str) -> Callsign {
        Callsign::parse(s).unwrap()
    }

    // The canonical default table shape: the C# defaults are MAX_ROUTES 3,
    // MAX_DESTINATIONS 1024. We use a generous-but-finite test size.
    type Table = NetRomRoutingTable<64, 3, 32>;

    fn port() -> PortId {
        PortId::from_str_lossy("vhf")
    }

    fn broadcast(sender_alias: &str, entries: &[EntrySpec]) -> NodesBroadcast {
        let info = build(sender_alias, entries);
        NodesBroadcast::try_parse(&info).expect("builder produces a parseable broadcast")
    }

    // Collect destinations out of the table (test convenience).
    fn dests(t: &Table) -> alloc::vec::Vec<NetRomDestination> {
        let mut v = alloc::vec::Vec::new();
        t.for_each_destination(|d| v.push(d));
        v
    }

    fn neighbours(t: &Table) -> alloc::vec::Vec<NetRomNeighbour> {
        let mut v = alloc::vec::Vec::new();
        t.for_each_neighbour(|n| v.push(n));
        v
    }

    fn routes_of(t: &Table, dest: &Callsign) -> alloc::vec::Vec<NetRomRoute> {
        let mut v = alloc::vec::Vec::new();
        t.for_each_route(dest, |r| v.push(r));
        v
    }

    fn find_dest(t: &Table, c: &Callsign) -> Option<NetRomDestination> {
        t.destination(c)
    }

    // ─── Neighbour + direct route + combined quality (mirror C# tests) ───

    #[test]
    fn hearing_a_broadcast_records_the_originator_as_a_neighbour() {
        let mut t = Table::with_defaults();
        let nbr_a = call("GB7RDG");
        t.ingest(
            nbr_a,
            call("M0LTE"),
            port(),
            &broadcast("RDGBPQ", &[]),
            1000,
        );

        let n = neighbours(&t);
        assert_eq!(n.len(), 1);
        assert_eq!(n[0].neighbour, nbr_a);
        assert_eq!(n[0].alias.as_str(), "RDGBPQ");
        assert_eq!(n[0].port_id.as_str(), "vhf");
        assert_eq!(n[0].path_quality, 192); // default neighbour quality
        assert_eq!(n[0].last_heard, 1000);
    }

    #[test]
    fn a_direct_route_to_the_originator_is_assumed_at_path_quality() {
        let mut t = Table::with_defaults();
        let nbr_a = call("GB7RDG");
        t.ingest(nbr_a, call("M0LTE"), port(), &broadcast("RDGBPQ", &[]), 0);

        let dest = find_dest(&t, &nbr_a).expect("originator is a destination");
        let best = dest.best_route.expect("has a route");
        assert_eq!(best.neighbour, nbr_a);
        assert_eq!(best.quality, 192);
    }

    #[test]
    fn an_advertised_destination_is_learned_at_the_combined_quality() {
        let mut t = Table::with_defaults();
        let nbr_a = call("GB7RDG");
        let nbr_b = call("GB7XYZ");
        let sot = call("GB7SOT");
        // RDG advertises it can reach SOT via XYZ at quality 200. Our path to RDG is
        // the default 192. Derived = (200*192 + 128)/256 = 150.5 → 150.
        t.ingest(
            nbr_a,
            call("M0LTE"),
            port(),
            &broadcast("RDGBPQ", &[(sot, "SOT", nbr_b, 200)]),
            0,
        );

        let d = find_dest(&t, &sot).expect("SOT learned");
        assert_eq!(d.alias.as_str(), "SOT");
        let best = d.best_route.unwrap();
        assert_eq!(best.neighbour, nbr_a); // we forward to RDG (the originator)
        assert_eq!(best.quality, quality::combine(200, 192)); // 150
    }

    #[test]
    fn trivial_loop_guard_zeroes_a_route_whose_best_neighbour_is_us() {
        let mut t = Table::with_defaults();
        let nbr_a = call("GB7RDG");
        let me = call("M0LTE");
        let mnc = call("GB7MNC");
        // RDG advertises a destination reachable via US (M0LTE) — a loop. The route
        // becomes quality 0, which is never kept, so MNC gets no route.
        t.ingest(
            nbr_a,
            me,
            port(),
            &broadcast("RDGBPQ", &[(mnc, "MNC", me, 200)]),
            0,
        );
        assert!(find_dest(&t, &mnc).is_none());
    }

    #[test]
    fn keeps_only_the_three_best_routes_per_destination() {
        let mut t = Table::with_defaults();
        let me = call("M0LTE");
        let sot = call("GB7SOT");
        let n1 = call("GB7AAA");
        let n2 = call("GB7BBB");
        let n3 = call("GB7CCC");
        let n4 = call("GB7DDD");
        // Four different neighbours each advertise SOT at different qualities. Each
        // is a distinct originator, so we learn four routes — capped to 3.
        t.ingest(
            n1,
            me,
            port(),
            &broadcast("AAA", &[(sot, "SOT", n1, 250)]),
            0,
        );
        t.ingest(
            n2,
            me,
            port(),
            &broadcast("BBB", &[(sot, "SOT", n2, 200)]),
            0,
        );
        t.ingest(
            n3,
            me,
            port(),
            &broadcast("CCC", &[(sot, "SOT", n3, 150)]),
            0,
        );
        t.ingest(
            n4,
            me,
            port(),
            &broadcast("DDD", &[(sot, "SOT", n4, 100)]),
            0,
        );

        let r = routes_of(&t, &sot);
        assert_eq!(r.len(), 3, "the per-destination route cap is 3");
        // Best-first, and the weakest (via n4, derived from 100) is dropped.
        assert!(r.windows(2).all(|w| w[0].quality >= w[1].quality));
        assert!(!r.iter().any(|x| x.neighbour == n4));
    }

    #[test]
    fn re_advertising_updates_the_route_in_place_not_duplicates_it() {
        let mut t = Table::with_defaults();
        let me = call("M0LTE");
        let nbr_a = call("GB7RDG");
        let nbr_b = call("GB7XYZ");
        let sot = call("GB7SOT");
        t.ingest(
            nbr_a,
            me,
            port(),
            &broadcast("RDG", &[(sot, "SOT", nbr_b, 200)]),
            0,
        );
        t.ingest(
            nbr_a,
            me,
            port(),
            &broadcast("RDG", &[(sot, "SOT", nbr_b, 100)]),
            0,
        );

        let r = routes_of(&t, &sot);
        assert_eq!(
            r.len(),
            1,
            "the same (dest, via-neighbour) is one route, refreshed"
        );
        assert_eq!(r[0].quality, quality::combine(100, 192));
    }

    // ─── Obsolescence ───

    #[test]
    fn a_route_is_initialised_to_obsinit_and_decremented_each_sweep() {
        let mut t = Table::with_defaults();
        let me = call("M0LTE");
        let nbr_a = call("GB7RDG");
        let nbr_b = call("GB7XYZ");
        let sot = call("GB7SOT");
        t.ingest(
            nbr_a,
            me,
            port(),
            &broadcast("RDG", &[(sot, "SOT", nbr_b, 200)]),
            0,
        );

        assert_eq!(
            find_dest(&t, &sot)
                .unwrap()
                .best_route
                .unwrap()
                .obsolescence,
            6
        ); // OBSINIT
        t.sweep();
        assert_eq!(
            find_dest(&t, &sot)
                .unwrap()
                .best_route
                .unwrap()
                .obsolescence,
            5
        );
    }

    #[test]
    fn a_route_is_purged_when_its_obsolescence_reaches_zero() {
        let opts = NetRomRoutingOptions {
            obsolete_initial: 2,
            ..NetRomRoutingOptions::DEFAULT
        };
        let mut t: Table = NetRomRoutingTable::new(opts);
        let me = call("M0LTE");
        let nbr_a = call("GB7RDG");
        let nbr_b = call("GB7XYZ");
        let sot = call("GB7SOT");
        t.ingest(
            nbr_a,
            me,
            port(),
            &broadcast("RDG", &[(sot, "SOT", nbr_b, 200)]),
            0,
        );

        t.sweep(); // 2 -> 1
        assert!(find_dest(&t, &sot).is_some());
        let purged = t.sweep(); // 1 -> 0 → purge
        assert!(purged > 0);
        assert!(find_dest(&t, &sot).is_none());
    }

    #[test]
    fn a_fresh_broadcast_resets_obsolescence_back_to_obsinit() {
        let mut t = Table::with_defaults();
        let me = call("M0LTE");
        let nbr_a = call("GB7RDG");
        let nbr_b = call("GB7XYZ");
        let sot = call("GB7SOT");
        t.ingest(
            nbr_a,
            me,
            port(),
            &broadcast("RDG", &[(sot, "SOT", nbr_b, 200)]),
            0,
        );
        t.sweep(); // 6 -> 5
        t.sweep(); // 5 -> 4
        t.ingest(
            nbr_a,
            me,
            port(),
            &broadcast("RDG", &[(sot, "SOT", nbr_b, 200)]),
            0,
        ); // refresh
        assert_eq!(
            find_dest(&t, &sot)
                .unwrap()
                .best_route
                .unwrap()
                .obsolescence,
            6
        );
    }

    #[test]
    fn sweeping_a_purged_destinations_only_neighbour_drops_the_neighbour_too() {
        let opts = NetRomRoutingOptions {
            obsolete_initial: 1,
            ..NetRomRoutingOptions::DEFAULT
        };
        let mut t: Table = NetRomRoutingTable::new(opts);
        let me = call("M0LTE");
        let nbr_a = call("GB7RDG");
        let nbr_b = call("GB7XYZ");
        let sot = call("GB7SOT");
        t.ingest(
            nbr_a,
            me,
            port(),
            &broadcast("RDG", &[(sot, "SOT", nbr_b, 200)]),
            0,
        );

        assert_eq!(neighbours(&t).len(), 1);
        t.sweep(); // purges both the direct route to RDG and the SOT route
        assert_eq!(dests(&t).len(), 0);
        assert_eq!(
            neighbours(&t).len(),
            0,
            "a neighbour with no surviving route is an orphan"
        );
    }

    // ─── Quality floor (MINQUAL) ───

    #[test]
    fn a_route_below_the_minqual_floor_is_dropped_by_a_higher_floor_but_kept_by_the_default() {
        let me = call("M0LTE");
        let nbr_a = call("GB7RDG");
        let nbr_b = call("GB7XYZ");
        let sot = call("GB7SOT");
        // RDG advertises SOT via XYZ at quality 80 → derived (80*192+128)/256 = 60.
        let bc = broadcast("RDG", &[(sot, "SOT", nbr_b, 80)]);

        // Default floor (0): the route is learned.
        let mut lenient = Table::with_defaults();
        lenient.ingest(nbr_a, me, port(), &bc, 0);
        assert!(find_dest(&lenient, &sot).is_some());

        // Raised floor (MINQUAL 128): the derived 60 is below the floor → dropped.
        let mut strict: Table = NetRomRoutingTable::new(NetRomRoutingOptions {
            min_quality: 128,
            ..NetRomRoutingOptions::DEFAULT
        });
        strict.ingest(nbr_a, me, port(), &bc, 0);
        assert!(find_dest(&strict, &sot).is_none());
    }

    #[test]
    fn a_re_advertisement_that_falls_below_the_floor_removes_the_existing_route() {
        let mut t: Table = NetRomRoutingTable::new(NetRomRoutingOptions {
            min_quality: 128,
            ..NetRomRoutingOptions::DEFAULT
        });
        let me = call("M0LTE");
        let nbr_a = call("GB7RDG");
        let nbr_b = call("GB7XYZ");
        let sot = call("GB7SOT");
        t.ingest(
            nbr_a,
            me,
            port(),
            &broadcast("RDG", &[(sot, "SOT", nbr_b, 250)]),
            0,
        ); // derived 187 — kept
        assert!(find_dest(&t, &sot).is_some());

        t.ingest(
            nbr_a,
            me,
            port(),
            &broadcast("RDG", &[(sot, "SOT", nbr_b, 80)]),
            0,
        ); // derived 60 — below floor
        assert!(
            find_dest(&t, &sot).is_none(),
            "the route decayed below the floor and the destination has no other route"
        );
    }

    // ─── Destination cap ───

    #[test]
    fn the_destination_list_stops_growing_at_the_cap() {
        // A 2-destination cap. Originator NbrA itself counts as one destination (its
        // assumed direct route). Advertise two more distinct destinations; only one
        // fits.
        let mut t: NetRomRoutingTable<2, 3, 8> =
            NetRomRoutingTable::new(NetRomRoutingOptions::DEFAULT);
        let me = call("M0LTE");
        let nbr_a = call("GB7RDG");
        let nbr_b = call("GB7XYZ");
        let sot = call("GB7SOT");
        let mnc = call("GB7MNC");
        t.ingest(
            nbr_a,
            me,
            port(),
            &broadcast("RDG", &[(sot, "SOT", nbr_b, 200), (mnc, "MNC", nbr_b, 200)]),
            0,
        );
        assert_eq!(
            t.destination_count(),
            2,
            "the originator + one advertised destination fill the cap"
        );
    }

    // ─── Snapshot shape ───

    #[test]
    fn snapshot_orders_destinations_by_alias_then_callsign() {
        let mut t = Table::with_defaults();
        let me = call("M0LTE");
        let nbr_a = call("GB7RDG");
        let nbr_b = call("GB7XYZ");
        let sot = call("GB7SOT");
        let mnc = call("GB7MNC");
        t.ingest(
            nbr_a,
            me,
            port(),
            &broadcast("RDG", &[(sot, "SOT", nbr_b, 200), (mnc, "MNC", nbr_b, 200)]),
            0,
        );

        // `Alias` is Copy; collect the relevant aliases in table order and check the
        // alias-ascending snapshot ordering puts MNC before SOT.
        let order: alloc::vec::Vec<_> = dests(&t)
            .into_iter()
            .map(|d| d.alias)
            .filter(|a| a.as_str() == "MNC" || a.as_str() == "SOT")
            .collect();
        assert_eq!(order.len(), 2);
        assert_eq!(order[0].as_str(), "MNC"); // MNC before SOT
        assert_eq!(order[1].as_str(), "SOT");
    }

    #[test]
    fn empty_table_yields_an_empty_snapshot() {
        let t = Table::with_defaults();
        assert_eq!(dests(&t).len(), 0);
        assert_eq!(neighbours(&t).len(), 0);
        assert_eq!(t.destination_count(), 0);
        assert_eq!(t.neighbour_count(), 0);
    }
}
