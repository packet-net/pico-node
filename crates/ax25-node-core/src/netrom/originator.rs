//! NET/ROM NODES **origination** (the TX half) — the counterpart to the read-only
//! ingest tap. It advertises *this* node and the learned destinations worth
//! re-advertising by framing a [`NetRomRoutingTable`]'s OBSMIN-gated advertisement
//! ([`NetRomRoutingTable::build_advertisement`]) into NODES broadcast frames
//! (signature `0xFF`, 6-octet header alias, then ≤ 11 routing entries per frame),
//! ready for the embedder to UI-send (dest the text callsign `NODES`, PID `0xCF`)
//! out every attached port.
//!
//! **Opt-in.** Origination is off until [`NetRomOriginatorOptions::enabled`] is set
//! — a library must not put traffic on the air unless asked (mirrors the C#
//! `netRom.broadcast` opt-in). When disabled, [`broadcast_nodes`] returns no
//! frames.
//!
//! **Sans-io, embedder-driven.** Unlike the TS/C# original this owns no ports, no
//! `sendUi` sink, and no NODESINTERVAL timer: [`broadcast_nodes`] is a pure
//! function of the routing table that *returns* the frames to send, and the node
//! host owns the port list + the re-broadcast interval (consistent with how the
//! circuit `tick()` and the ingest `sweep()` are embedder-driven — the library
//! starts no ambient timers, which keeps it trivially testable on-device). The
//! node announces *itself* simply by being the UI-frame source plus the header
//! alias, so a header-only frame (empty table) is still a useful "I'm here".
//!
//! Mirrors the origination portion of `Packet.Node.Core.NetRom.NetRomService`
//! (`BroadcastNodes`) / the TS `NetRomOriginator`.
//!
//! [`broadcast_nodes`]: NetRomOriginator::broadcast_nodes

use alloc::vec::Vec;

use crate::ax25::Callsign;
use crate::netrom::routing::NetRomRoutingTable;
use crate::netrom::wire::{write_nodes_frame, Alias, NodesBroadcast, MAX_NODES_FRAME_LEN};

/// Construction options for [`NetRomOriginator`]. Every field is optional
/// ([`Default`] is "disabled, empty alias, table-configured OBSMIN").
#[derive(Debug, Clone, Copy, Default)]
pub struct NetRomOriginatorOptions {
    /// Whether origination is enabled (the C# `netRom.broadcast` opt-in). Default
    /// `false` — a library must not transmit unless asked.
    pub enabled: bool,
    /// The broadcasting node's NET/ROM alias / mnemonic — the header alias every
    /// NODES frame carries (the receiver pairs it with the UI-frame source callsign
    /// to learn us). When `None`/empty and [`node_call`](Self::node_call) is given,
    /// the node callsign's base is used (mirroring the C# `ResolveAlias` fallback to
    /// `nodeCall.Base`); otherwise the alias is empty.
    pub alias: Option<Alias>,
    /// The node's own callsign — used *only* as the alias fallback (see
    /// [`alias`](Self::alias)). The per-frame source callsign is the embedder's own
    /// port callsign, not this.
    pub node_call: Option<Callsign>,
    /// OBSMIN override passed to [`NetRomRoutingTable::build_advertisement`]: a
    /// learned route whose obsolescence has decayed below this is kept + usable but
    /// no longer re-advertised. `None` uses the table's configured
    /// [`obsolete_minimum`](crate::netrom::routing::NetRomRoutingOptions::obsolete_minimum)
    /// (canonical 4); `Some(0)` re-advertises every kept route.
    pub obsolete_minimum: Option<u8>,
}

/// The NET/ROM NODES origination (TX) frame builder over a shared routing table.
#[derive(Debug, Clone)]
pub struct NetRomOriginator {
    enabled: bool,
    alias: Alias,
    obsolete_minimum: Option<u8>,
}

impl NetRomOriginator {
    /// The PID every originated NODES broadcast UI frame carries (NET/ROM, `0xCF`).
    pub const PID: u8 = crate::ax25::PID_NETROM;

    /// Construct the originator. Off by default — set
    /// [`NetRomOriginatorOptions::enabled`] to transmit.
    pub fn new(options: NetRomOriginatorOptions) -> Self {
        let alias = match options.alias {
            Some(a) if !a.is_empty() => a,
            _ => match options.node_call {
                Some(call) => alias_from_call_base(&call),
                None => Alias::default(),
            },
        };
        Self {
            enabled: options.enabled,
            alias,
            obsolete_minimum: options.obsolete_minimum,
        }
    }

    /// True if origination is enabled.
    pub fn enabled(&self) -> bool {
        self.enabled
    }

    /// The alias every originated NODES broadcast carries in its header.
    pub fn sender_alias(&self) -> &Alias {
        &self.alias
    }

    /// The AX.25 destination every originated NODES broadcast is addressed to (the
    /// text callsign `NODES`, SSID 0).
    pub fn nodes_destination() -> Callsign {
        Callsign::new(NodesBroadcast::NODES_DESTINATION.as_bytes(), 0)
            .expect("NODES is a valid base callsign")
    }

    /// Build our NODES broadcast frames *now* from `table`: the header alias plus a
    /// destination entry per advertisable route (OBSMIN-gated via
    /// [`NetRomRoutingTable::build_advertisement`]), chunked 11 entries per frame.
    /// Each returned `Vec<u8>` is one NODES info payload to UI-send (dest
    /// [`nodes_destination`](Self::nodes_destination), PID [`PID`](Self::PID)) on
    /// every attached port.
    ///
    /// Returns no frames when origination is disabled. When the table is empty a
    /// single header-only frame (7 octets: `0xFF` + 6-octet alias) is still
    /// produced — the node announcing itself. Mirrors the C#
    /// `NetRomService.BroadcastNodes`.
    pub fn broadcast_nodes<const MAX_DESTS: usize, const MAX_ROUTES: usize, const MAX_NBRS: usize>(
        &self,
        table: &NetRomRoutingTable<MAX_DESTS, MAX_ROUTES, MAX_NBRS>,
    ) -> Vec<Vec<u8>> {
        if !self.enabled {
            return Vec::new();
        }

        let entries = table.build_advertisement(self.obsolete_minimum);
        let mut frames = Vec::new();
        let mut buf = [0u8; MAX_NODES_FRAME_LEN];

        if entries.is_empty() {
            // Header-only "I'm here" frame — no entries.
            if let Some(len) = write_nodes_frame(&self.alias, &[], &mut buf) {
                frames.push(buf[..len].to_vec());
            }
            return frames;
        }

        for chunk in entries.chunks(NodesBroadcast::MAX_ENTRIES_PER_FRAME) {
            if let Some(len) = write_nodes_frame(&self.alias, chunk, &mut buf) {
                frames.push(buf[..len].to_vec());
            }
        }
        frames
    }
}

/// Derive a header alias from a callsign's significant base bytes (the C#
/// `ResolveAlias` fallback to `nodeCall.Base`).
fn alias_from_call_base(call: &Callsign) -> Alias {
    match core::str::from_utf8(call.base()) {
        Ok(base) => Alias::from_str_lossy(base),
        Err(_) => Alias::default(),
    }
}

#[cfg(test)]
mod tests {
    //! Ported from the TS `tests/netrom/originator.test.ts`. The cross-check oracle
    //! is the production parser ([`NodesBroadcast::try_parse`]) plus a fresh
    //! [`NetRomRoutingTable`]: an originated broadcast is asserted not against a
    //! hand-rolled byte expectation but by parsing it back (and, for the inverse
    //! oracle, re-ingesting it), so origination ↔ ingest is proven an inverse pair.
    //!
    //! The TS tests for port-fan-out, failing-port isolation, and the
    //! NODESINTERVAL timer are intentionally not ported: those exercise host
    //! plumbing this sans-io builder deliberately doesn't own (the node host owns
    //! the ports + the interval). The behavioural core — frame building, chunking,
    //! the OBSMIN gate, and the ingest round-trip — is ported in full.

    use super::*;
    use crate::netrom::routing::NetRomRoutingOptions;
    use crate::netrom::wire::NodesAdvertisementEntry;
    use crate::netrom::PortId;

    type Table = NetRomRoutingTable<64, 3, 32>;

    const NOW: u64 = 1_000_000;
    const ENTRY_LEN: usize = 21; // shifted dest (7) + alias (6) + shifted nbr (7) + quality (1)
    const HEADER_LEN: usize = 7; // 0xFF + 6-octet alias

    fn call(text: &str) -> Callsign {
        Callsign::new(text.as_bytes(), 0).unwrap()
    }

    fn port() -> PortId {
        PortId::from_str_lossy("p1")
    }

    fn entry(dest: &str, alias: &str, neighbour: &str, quality: u8) -> NodesAdvertisementEntry {
        NodesAdvertisementEntry {
            destination: call(dest),
            destination_alias: Alias::from_str_lossy(alias),
            best_neighbour: call(neighbour),
            quality,
        }
    }

    /// Build a parseable NODES broadcast to seed a table (the canonical encoder is
    /// fine here — seeding just needs valid input; the originator's own output is
    /// validated by parsing it back, and ingest transforms the qualities so the
    /// round-trip is no encode/decode tautology).
    fn broadcast(sender_alias: &str, entries: &[NodesAdvertisementEntry]) -> NodesBroadcast {
        let mut buf = [0u8; MAX_NODES_FRAME_LEN];
        let n = write_nodes_frame(&Alias::from_str_lossy(sender_alias), entries, &mut buf).unwrap();
        NodesBroadcast::try_parse(&buf[..n]).unwrap()
    }

    fn enabled_originator(alias: &str) -> NetRomOriginator {
        NetRomOriginator::new(NetRomOriginatorOptions {
            enabled: true,
            alias: Some(Alias::from_str_lossy(alias)),
            ..Default::default()
        })
    }

    /// True if any of `frames` advertises `dest`.
    fn advertises(frames: &[Vec<u8>], dest: &Callsign) -> bool {
        frames.iter().any(|f| {
            NodesBroadcast::try_parse(f)
                .map(|p| p.entries().any(|e| e.destination == *dest))
                .unwrap_or(false)
        })
    }

    #[test]
    fn origination_is_off_unless_enabled() {
        let table: Table = NetRomRoutingTable::new(NetRomRoutingOptions::default());
        let orig = NetRomOriginator::new(NetRomOriginatorOptions {
            enabled: false,
            alias: Some(Alias::from_str_lossy("AAANOD")),
            ..Default::default()
        });
        assert!(!orig.enabled());
        assert!(orig.broadcast_nodes(&table).is_empty(), "disabled ⇒ no frames");
    }

    #[test]
    fn emits_one_frame_advertising_the_tables_routes() {
        let mut table: Table = NetRomRoutingTable::new(NetRomRoutingOptions::default());
        table.ingest(
            call("GB7HUB"),
            call("GB7AAA"),
            port(),
            &broadcast("HUB", &[entry("GB7SOT", "SOT", "GB7HUB", 255)]),
            NOW,
        );

        let frames = enabled_originator("AAANOD").broadcast_nodes(&table);
        assert_eq!(frames.len(), 1);

        let parsed = NodesBroadcast::try_parse(&frames[0]).unwrap();
        assert_eq!(parsed.sender_alias(), Alias::from_str_lossy("AAANOD"));

        // A learned SOT (via HUB) and HUB itself (the heard neighbour's assumed
        // direct route) — both are advertised.
        assert!(advertises(&frames, &call("GB7SOT")));
        assert!(advertises(&frames, &call("GB7HUB")));

        let sot = parsed
            .entries()
            .find(|e| e.destination == call("GB7SOT"))
            .expect("SOT advertised");
        assert!(sot.best_quality < 255, "advertised at A's combined (decayed) quality");
        assert!(sot.best_quality > 0);
        assert_eq!(sot.best_neighbour, call("GB7HUB"));
    }

    #[test]
    fn emits_a_header_only_frame_when_the_table_is_empty() {
        let table: Table = NetRomRoutingTable::new(NetRomRoutingOptions::default());
        let frames = enabled_originator("AAANOD").broadcast_nodes(&table);

        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].len(), HEADER_LEN, "0xFF + 6-byte alias, no entries");
        let parsed = NodesBroadcast::try_parse(&frames[0]).unwrap();
        assert_eq!(parsed.sender_alias(), Alias::from_str_lossy("AAANOD"));
        assert_eq!(parsed.entry_count(), 0);
    }

    #[test]
    fn chunks_a_large_table_into_multiple_frames() {
        let mut table: Table = NetRomRoutingTable::new(NetRomRoutingOptions::default());
        // 24 distinct destinations heard via HUB (+ HUB itself ⇒ 25 to advertise).
        let mut specs = Vec::new();
        for i in 0..24u8 {
            let mut name = *b"GB7N00";
            name[4] = b'0' + i / 10;
            name[5] = b'0' + i % 10;
            let dest = core::str::from_utf8(&name).unwrap();
            specs.push(entry(dest, "N", "GB7HUB", 250 - i));
        }
        for chunk in specs.chunks(NodesBroadcast::MAX_ENTRIES_PER_FRAME) {
            table.ingest(call("GB7HUB"), call("GB7AAA"), port(), &broadcast("HUB", chunk), NOW);
        }

        let frames = enabled_originator("AAANOD").broadcast_nodes(&table);
        assert_eq!(frames.len(), 3, "25 entries ⇒ 11 + 11 + 3");

        let mut total = 0usize;
        for f in &frames {
            let parsed = NodesBroadcast::try_parse(f).unwrap();
            assert!(parsed.entry_count() <= NodesBroadcast::MAX_ENTRIES_PER_FRAME);
            total += parsed.entry_count();
        }
        assert_eq!(total, 25);
        assert_eq!(frames[0].len(), HEADER_LEN + 11 * ENTRY_LEN);
    }

    #[test]
    fn stops_advertising_a_faded_route_before_it_is_purged() {
        let mut table: Table = NetRomRoutingTable::new(NetRomRoutingOptions {
            obsolete_initial: 6,
            obsolete_minimum: 4,
            ..Default::default()
        });
        let orig = enabled_originator("AAANOD");
        table.ingest(
            call("GB7HUB"),
            call("GB7AAA"),
            port(),
            &broadcast("HUB", &[entry("GB7SOT", "SOT", "GB7HUB", 255)]),
            NOW,
        );

        assert!(advertises(&orig.broadcast_nodes(&table), &call("GB7SOT")), "fresh: obs 6 ≥ 4");
        table.sweep(); // 6 → 5
        table.sweep(); // 5 → 4 (still ≥ 4)
        assert!(advertises(&orig.broadcast_nodes(&table), &call("GB7SOT")));
        table.sweep(); // 4 → 3 (< OBSMIN 4)
        assert!(!advertises(&orig.broadcast_nodes(&table), &call("GB7SOT")));

        // Faded out of advertisement, but still kept in the table (purged only at 0).
        assert!(table.destination(&call("GB7SOT")).is_some());
    }

    #[test]
    fn an_obsolete_minimum_override_of_zero_advertises_every_kept_route() {
        let mut table: Table = NetRomRoutingTable::new(NetRomRoutingOptions {
            obsolete_initial: 6,
            obsolete_minimum: 4,
            ..Default::default()
        });
        let orig = NetRomOriginator::new(NetRomOriginatorOptions {
            enabled: true,
            alias: Some(Alias::from_str_lossy("AAANOD")),
            obsolete_minimum: Some(0),
            ..Default::default()
        });
        table.ingest(
            call("GB7HUB"),
            call("GB7AAA"),
            port(),
            &broadcast("HUB", &[entry("GB7SOT", "SOT", "GB7HUB", 255)]),
            NOW,
        );
        table.sweep(); // 6 → 5
        table.sweep(); // 5 → 4
        table.sweep(); // 4 → 3 — below the table's OBSMIN 4, but the originator gate is 0

        assert!(advertises(&orig.broadcast_nodes(&table), &call("GB7SOT")));
    }

    #[test]
    fn node_b_learns_node_a_and_its_routes_from_as_originated_broadcast() {
        // A learns SOT via HUB, then originates.
        let mut table_a: Table = NetRomRoutingTable::new(NetRomRoutingOptions::default());
        table_a.ingest(
            call("GB7HUB"),
            call("GB7AAA"),
            port(),
            &broadcast("HUB", &[entry("GB7SOT", "SOT", "GB7HUB", 255)]),
            NOW,
        );
        let frames = enabled_originator("AAANOD").broadcast_nodes(&table_a);
        assert!(!frames.is_empty());

        let advertised_by_a = NodesBroadcast::try_parse(&frames[0])
            .unwrap()
            .entries()
            .find(|e| e.destination == call("GB7SOT"))
            .expect("A advertises SOT")
            .best_quality;

        // B hears every frame from A and ingests it.
        let mut table_b: Table = NetRomRoutingTable::new(NetRomRoutingOptions::default());
        for f in &frames {
            let parsed = NodesBroadcast::try_parse(f).unwrap();
            table_b.ingest(call("GB7AAA"), call("GB7BBB"), port(), &parsed, NOW);
        }

        // B heard exactly one neighbour — A.
        assert_eq!(table_b.neighbour_count(), 1);
        let mut sole_neighbour = None;
        table_b.for_each_neighbour(|n| sole_neighbour = Some(n.neighbour));
        assert_eq!(sole_neighbour, Some(call("GB7AAA")));

        // B learned A as a destination, and SOT via A at a quality decayed over the
        // extra hop B→A.
        assert!(table_b.destination(&call("GB7AAA")).is_some());
        let sot = table_b.destination(&call("GB7SOT")).expect("B learned SOT");
        let best = sot.best_route.expect("SOT has a route");
        assert_eq!(best.neighbour, call("GB7AAA"), "B forwards to A");
        assert_eq!(sot.alias, Alias::from_str_lossy("SOT"));
        assert!(best.quality < advertised_by_a, "decayed over the extra hop");
    }
}
