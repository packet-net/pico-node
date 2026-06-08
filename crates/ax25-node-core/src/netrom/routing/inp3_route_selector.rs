//! The pure INP3 route-**selection** policy.
//!
//! Given a destination's kept routes and the `prefer_inp3_routes` knob, decide
//! which single [`NetRomRoute`] the node treats as *active* for that destination
//! (the route a `connect` or a best-route forward resolves to). This is the locked
//! truth table of plan risk #4 — the coexistence of the two metric spaces (NODES
//! quality vs INP3 measured target time) — realised as a side-effect-free function
//! (see `docs/netrom-inp3-i3-design.md` §3).
//!
//! Mirrors `Packet.NetRom.Routing.Inp3RouteSelector` on the C# side and
//! `inp3-route-selector.ts` on the TS side.
//!
//! **The truth table.**
//!
//! - `prefer_inp3_routes == true` *and* the destination has at least one INP3 route
//!   (a route whose [`NetRomRoute::inp3`] is `Some`): select the **lowest
//!   [`Inp3RouteMetric::target_time_ms`]** INP3 route, ties broken by lowest
//!   [`Inp3RouteMetric::hop_count`] then by neighbour callsign (ordinal) for
//!   determinism — the time-space mirror of the quality-space "highest quality,
//!   then callsign" ordering.
//! - Otherwise (the knob is off, *or* no INP3 route exists): fall back to the
//!   **best-quality** route — exactly today's behaviour, the destination's
//!   best-quality route (`best_quality`, the first of the best-quality-first
//!   route list the table maintains). The [`NetRomRoute::inp3`] metric is never
//!   read on this path.
//!
//! **Degenerate-to-today invariant (the acceptance bar, §3.3).** Selection
//! collapses to today's quality path — byte-for-byte — in every case where INP3
//! cannot win: (1) the knob off ⇒ quality; (2) a destination with no INP3 route ⇒
//! quality fallback; (3) a single-route destination ⇒ that one route regardless of
//! mode. INP3 only ever changes the result for a destination that *both* opted in
//! via the knob and actually holds a time-route. The `enabled` overlay switch sits
//! above this function: when the overlay is disabled no INP3 route is ever
//! ingested, so [`NetRomRoute::inp3`] is `None` on every route and the caller
//! passes `prefer_inp3_routes: false` (or it is moot) — either way this function
//! returns the quality route unchanged.
//!
//! **Purity.** No table, engine, options-record, or I/O dependency; no allocation
//! (the INP3 winner is found by a single linear scan with a running best, not a
//! sort). The single `bool` parameter is the already-resolved `prefer_inp3_routes`
//! knob (read by the host from `NetRomInp3Options::prefer_inp3_routes`), so the
//! selector itself stays free of the options type.
//!
//! **Signature divergence from C#/TS (judgement call).** The C#/TS take the whole
//! `NetRomDestination` (which carries both `BestRoute` *and* the full `Routes`
//! list) and read both off it. The Rust read-side [`NetRomDestination`] is a small
//! `Copy` value snapshot that carries only `best_route` + a `route_count`, not the
//! route list (the [`crate::netrom::routing::NetRomRoutingTable`] surfaces the kept
//! routes through `for_each_route`, not a stored `Vec`). So this is a free function
//! taking the routes as a borrowed slice plus the already-resolved best-quality
//! route — the host gathers the slice via `for_each_route` and passes
//! `dest.best_route` as `best_quality`. This is the project's "behaviour is
//! functions, state lives in the table" idiom, the same shape as the wire codecs
//! and `decide_forward`.

use crate::ax25::Callsign;

use super::model::NetRomRoute;

/// Select the active route under the INP3 selection policy.
///
/// Mirrors `Inp3RouteSelector.SelectActiveRoute` / TS `selectActiveRoute`.
///
/// - `routes` — the destination's kept routes (best-quality first, the ordering
///   the table maintains via `for_each_route`). Scanned only when
///   `prefer_inp3_routes` is set; ignored entirely otherwise.
/// - `best_quality` — the destination's best-quality (active) route, today's
///   behaviour — the C#/TS `dest.BestRoute` (= `routes[0]` when non-empty, `None`
///   for a destination with no routes). Returned whenever the knob is off, or no
///   INP3 route exists, or there are no routes at all.
/// - `prefer_inp3_routes` — the resolved `prefer_inp3_routes` knob (BPQ's
///   `PREFERINP3ROUTES`; `NetRomInp3Options::prefer_inp3_routes`). When `true` an
///   INP3 route, if any, beats quality; when `false` the [`NetRomRoute::inp3`]
///   metric is ignored entirely and quality wins.
///
/// Returns the lowest-target-time INP3 route when `prefer_inp3_routes` is set and
/// one exists; otherwise the best-quality route (`best_quality`); or `None` for a
/// destination with no routes.
pub fn select_active_route(
    routes: &[NetRomRoute],
    best_quality: Option<NetRomRoute>,
    prefer_inp3_routes: bool,
) -> Option<NetRomRoute> {
    // Quality fallback path == today's behaviour, byte-for-byte: the best-quality
    // route. Taken whenever the knob is off, or no INP3 route exists (handled
    // below), or there are no routes at all.
    if !prefer_inp3_routes {
        return best_quality;
    }

    // prefer_inp3_routes == true: prefer the best INP3 route if the destination
    // holds any time-route; else fall back to quality. A single linear scan keeps
    // a running best by the time-space key (lowest target_time_ms, then lowest
    // hop_count, then neighbour callsign ordinal) — no allocation, no sort.
    let mut best_inp3: Option<NetRomRoute> = None;
    for route in routes {
        if route.inp3.is_none() {
            continue; // a pure quality-route: invisible to the INP3 winner search.
        }
        let replace = match best_inp3 {
            None => true,
            Some(incumbent) => is_better_inp3(route, &incumbent),
        };
        if replace {
            best_inp3 = Some(*route);
        }
    }

    // Any INP3 route ⇒ it wins; otherwise the quality fallback (degenerates to
    // today for a destination known only via NODES).
    best_inp3.or(best_quality)
}

/// True if INP3 route `candidate` ranks strictly better than the current best
/// `incumbent` in the time metric space: lower [`Inp3RouteMetric::target_time_ms`]
/// wins; ties broken by lower [`Inp3RouteMetric::hop_count`], then by neighbour
/// callsign (ordinal) for a stable, deterministic choice. Both routes are assumed
/// INP3-bearing ([`NetRomRoute::inp3`] is `Some`) — the caller filters quality-only
/// routes out before comparing.
///
/// [`Inp3RouteMetric::target_time_ms`]: super::model::Inp3RouteMetric::target_time_ms
/// [`Inp3RouteMetric::hop_count`]: super::model::Inp3RouteMetric::hop_count
///
/// Mirrors `Inp3RouteSelector.IsBetterInp3` / TS `isBetterInp3`.
fn is_better_inp3(candidate: &NetRomRoute, incumbent: &NetRomRoute) -> bool {
    // Both are INP3-bearing by the caller's contract; `expect` documents that.
    let c = candidate.inp3.expect("candidate is INP3-bearing");
    let i = incumbent.inp3.expect("incumbent is INP3-bearing");

    if c.target_time_ms != i.target_time_ms {
        return c.target_time_ms < i.target_time_ms; // lowest target time = best.
    }
    if c.hop_count != i.hop_count {
        return c.hop_count < i.hop_count; // tie-break: fewest hops.
    }

    // Final tie-break: neighbour callsign ordinal, for a deterministic winner
    // across the C#/TS/Rust ports (mirrors the quality-space callsign tie-break).
    // The C# uses `string.CompareOrdinal(callsign.ToString())`; the established
    // Rust analogue (the routing table's `callsign_lt`) compares the base bytes
    // ordinally, then the SSID — identical to the ordinal text comparison for the
    // alphanumeric-base + SSID forms NET/ROM uses.
    callsign_lt(&candidate.neighbour, &incumbent.neighbour)
}

/// Ordinal callsign comparison: base bytes then SSID. Matches the C#
/// `string.CompareOrdinal(callsign.ToString())` tie-break for the alphanumeric
/// base + SSID forms NET/ROM uses; identical semantics to the routing table's own
/// `callsign_lt`.
fn callsign_lt(a: &Callsign, b: &Callsign) -> bool {
    match a.base().cmp(b.base()) {
        core::cmp::Ordering::Less => true,
        core::cmp::Ordering::Greater => false,
        core::cmp::Ordering::Equal => a.ssid() < b.ssid(),
    }
}

#[cfg(test)]
mod tests {
    //! The locked INP3 selection truth table (plan risk #4,
    //! `docs/netrom-inp3-i3-design.md` §3) realised as unit + property tests over
    //! [`select_active_route`]. Covers every row — disabled⇒quality;
    //! prefer+inp3⇒lowest-time; prefer+no-inp3⇒quality fallback; !prefer⇒quality —
    //! plus the three "degenerate to today" invariants (§3.3).
    //!
    //! Port of `tests/Packet.NetRom.Tests/Routing/Inp3RouteSelectorTests.cs` (and
    //! the TS `inp3-route-selector.test.ts`). Same cases, same boundary values.
    //!
    //! **Identity-assertion divergence.** The C# `BeSameAs(dest.BestRoute)` / TS
    //! `toBe(dest.bestRoute)` assert *reference identity* with the best-quality
    //! route object. Rust [`NetRomRoute`] is `Copy` (a small value, no reference
    //! identity), so the faithful Rust analogue is value equality — `best_quality`
    //! is the value we passed in, and we assert the returned route equals it and
    //! carries the expected neighbour. This matches how the routing table's own
    //! ported tests handle the same C# `BeSameAs` shape.

    use super::*;
    use crate::netrom::routing::model::Inp3RouteMetric;

    fn call(s: &str) -> Callsign {
        Callsign::parse(s).expect("test callsign parses")
    }

    fn nbr_a() -> Callsign {
        call("GB7AAA")
    }
    fn nbr_b() -> Callsign {
        call("GB7BBB")
    }
    fn nbr_c() -> Callsign {
        call("GB7CCC")
    }

    // A quality-only route (today's vanilla triple; no INP3 metric).
    fn q(nbr: Callsign, quality: u8) -> NetRomRoute {
        NetRomRoute {
            neighbour: nbr,
            quality,
            obsolescence: 6,
            inp3: None,
        }
    }

    // A route carrying both a quality and an INP3 (target-time) metric.
    fn t(nbr: Callsign, quality: u8, target_time_ms: u32, hop_count: u8) -> NetRomRoute {
        NetRomRoute {
            neighbour: nbr,
            quality,
            obsolescence: 6,
            inp3: Some(Inp3RouteMetric {
                target_time_ms,
                hop_count,
            }),
        }
    }

    // Build the (routes, best_quality) pair a destination would surface: the
    // best-quality-first route list (routes[0] is the quality-best = today's
    // best route) and the best-quality route taken from routes[0] (None when
    // empty) — mirroring the C#/TS `NetRomDestination.BestRoute => Routes[0]`.
    fn dest_of(routes: &[NetRomRoute]) -> (&[NetRomRoute], Option<NetRomRoute>) {
        (routes, routes.first().copied())
    }

    // ---- Row: !prefer (and the disabled-overlay default) -> quality, byte-for-byte ----

    #[test]
    fn not_prefer_returns_best_quality_route_ignoring_inp3() {
        // NbrA is quality-best (first); NbrB carries a far-better (lower) target
        // time. With prefer off, the INP3 metric is invisible — quality wins.
        let routes = [t(nbr_a(), 200, 9000, 3), t(nbr_b(), 100, 10, 1)];
        let (routes, best) = dest_of(&routes);

        let chosen = select_active_route(routes, best, false);

        assert_eq!(chosen, best);
        assert_eq!(chosen.unwrap().neighbour, nbr_a());
    }

    #[test]
    fn not_prefer_returns_best_quality_for_quality_only_destination() {
        let routes = [q(nbr_a(), 200), q(nbr_b(), 100)];
        let (routes, best) = dest_of(&routes);

        let chosen = select_active_route(routes, best, false);

        assert_eq!(chosen, best);
        assert_eq!(chosen.unwrap().neighbour, nbr_a());
    }

    // ---- Row: prefer + an INP3 route exists -> lowest-target_time_ms INP3 route ----

    #[test]
    fn prefer_with_inp3_routes_selects_lowest_target_time() {
        // Quality-best is NbrA; the lowest-target-time INP3 route is NbrC (5 ms).
        let routes = [
            t(nbr_a(), 250, 8000, 2),
            t(nbr_b(), 120, 500, 4),
            t(nbr_c(), 60, 5, 7),
        ];
        let (routes, best) = dest_of(&routes);

        let chosen = select_active_route(routes, best, true);

        assert_eq!(chosen.unwrap().neighbour, nbr_c());
        assert_eq!(chosen.unwrap().inp3.unwrap().target_time_ms, 5);
    }

    #[test]
    fn prefer_picks_inp3_route_even_when_a_higher_quality_quality_only_route_exists() {
        // NbrA is the quality-best route but carries NO INP3 metric; NbrB is INP3.
        // Prefer must pick the INP3 route, not the higher-quality quality-only one.
        let routes = [q(nbr_a(), 250), t(nbr_b(), 50, 1234, 3)];
        let (routes, best) = dest_of(&routes);

        let chosen = select_active_route(routes, best, true);

        assert_eq!(chosen.unwrap().neighbour, nbr_b());
        assert!(chosen.unwrap().inp3.is_some());
    }

    #[test]
    fn prefer_breaks_target_time_ties_by_lowest_hop_count() {
        let routes = [
            t(nbr_a(), 200, 400, 5),
            t(nbr_b(), 200, 400, 2), // same time, fewer hops
            t(nbr_c(), 200, 400, 9),
        ];
        let (routes, best) = dest_of(&routes);

        let chosen = select_active_route(routes, best, true);

        assert_eq!(chosen.unwrap().neighbour, nbr_b());
    }

    #[test]
    fn prefer_breaks_time_and_hop_ties_by_neighbour_callsign_ordinal() {
        // All three tie on time AND hop; deterministic winner is the lowest ordinal
        // callsign, regardless of the order they appear in the routes list.
        let routes = [
            t(nbr_c(), 200, 300, 4),
            t(nbr_a(), 200, 300, 4),
            t(nbr_b(), 200, 300, 4),
        ];
        let (routes, best) = dest_of(&routes);

        let chosen = select_active_route(routes, best, true);

        assert_eq!(chosen.unwrap().neighbour, nbr_a()); // GB7AAA < GB7BBB < GB7CCC
    }

    // ---- Row: prefer but NO INP3 route -> quality fallback (byte-for-byte today) ----

    #[test]
    fn prefer_with_no_inp3_route_falls_back_to_best_quality() {
        let routes = [q(nbr_a(), 200), q(nbr_b(), 100)];
        let (routes, best) = dest_of(&routes);

        let chosen = select_active_route(routes, best, true);

        assert_eq!(chosen, best);
        assert_eq!(chosen.unwrap().neighbour, nbr_a());
    }

    // ---- Degeneracy: single route -> same result regardless of mode ----

    #[test]
    fn single_quality_route_degenerates_to_that_route_in_any_mode() {
        for prefer in [false, true] {
            let routes = [q(nbr_a(), 180)];
            let (routes, best) = dest_of(&routes);

            let chosen = select_active_route(routes, best, prefer);

            assert_eq!(chosen, best);
            assert_eq!(chosen.unwrap().neighbour, nbr_a());
        }
    }

    #[test]
    fn single_inp3_route_is_selected_in_any_mode() {
        // One route that happens to carry an INP3 metric: prefer picks it as the
        // INP3 winner; !prefer picks it as the (only) quality route. Same route
        // either way — single-route degeneracy holds across the metric spaces.
        for prefer in [false, true] {
            let only = t(nbr_a(), 140, 250, 2);
            let routes = [only];
            let (routes, best) = dest_of(&routes);

            let chosen = select_active_route(routes, best, prefer);

            assert_eq!(chosen, Some(only));
        }
    }

    // ---- Degeneracy: empty destination -> None in any mode ----

    #[test]
    fn no_routes_returns_none() {
        for prefer in [false, true] {
            let routes: [NetRomRoute; 0] = [];
            let (routes, best) = dest_of(&routes);

            assert_eq!(select_active_route(routes, best, prefer), None);
        }
    }

    // ---- Property: !prefer ALWAYS returns today's best-quality route (full degeneracy) ----

    fn mixed_route_sets() -> alloc::vec::Vec<alloc::vec::Vec<NetRomRoute>> {
        alloc::vec![
            alloc::vec![q(nbr_a(), 200), q(nbr_b(), 100)],
            alloc::vec![t(nbr_a(), 200, 9000, 3), t(nbr_b(), 100, 10, 1)], // INP3 present but ignored
            alloc::vec![q(nbr_a(), 255), t(nbr_b(), 50, 5, 1)],
            alloc::vec![t(nbr_a(), 1, 1, 1)], // single INP3-bearing route
        ]
    }

    #[test]
    fn not_prefer_always_equals_today_best_quality() {
        for routes in mixed_route_sets() {
            let (routes_slice, best) = dest_of(&routes);

            let chosen = select_active_route(routes_slice, best, false);

            assert_eq!(chosen, best);
        }
    }

    // ---- Property: prefer + quality-only set ALWAYS falls back to today's best route ----

    fn quality_only_route_sets() -> alloc::vec::Vec<alloc::vec::Vec<NetRomRoute>> {
        alloc::vec![
            alloc::vec![q(nbr_a(), 200)],
            alloc::vec![q(nbr_a(), 200), q(nbr_b(), 100)],
            alloc::vec![q(nbr_a(), 200), q(nbr_b(), 199), q(nbr_c(), 1)],
        ]
    }

    #[test]
    fn prefer_over_quality_only_set_equals_today_best_quality() {
        for routes in quality_only_route_sets() {
            let (routes_slice, best) = dest_of(&routes);

            let chosen = select_active_route(routes_slice, best, true);

            assert_eq!(chosen, best);
        }
    }

    // ---- Property: prefer + ANY inp3 route -> a route with the minimum target_time_ms ----

    fn inp3_bearing_route_sets() -> alloc::vec::Vec<alloc::vec::Vec<NetRomRoute>> {
        alloc::vec![
            alloc::vec![t(nbr_a(), 200, 5, 2)],
            alloc::vec![
                t(nbr_a(), 200, 8000, 2),
                t(nbr_b(), 120, 500, 4),
                t(nbr_c(), 60, 5, 7)
            ],
            alloc::vec![q(nbr_a(), 255), t(nbr_b(), 50, 1234, 3)], // quality-best is quality-only
            alloc::vec![t(nbr_a(), 200, 400, 5), t(nbr_b(), 200, 400, 2)], // tie on time
        ]
    }

    #[test]
    fn prefer_selects_a_route_with_the_minimum_target_time() {
        for routes in inp3_bearing_route_sets() {
            let (routes_slice, best) = dest_of(&routes);

            let chosen = select_active_route(routes_slice, best, true);

            let min_time = routes
                .iter()
                .filter_map(|r| r.inp3.map(|m| m.target_time_ms))
                .min()
                .expect("set has at least one INP3 route");
            let chosen = chosen.expect("prefer with an INP3 route present must pick an INP3 route");
            assert!(
                chosen.inp3.is_some(),
                "prefer with an INP3 route present must pick an INP3 route"
            );
            assert_eq!(chosen.inp3.unwrap().target_time_ms, min_time);
        }
    }
}
