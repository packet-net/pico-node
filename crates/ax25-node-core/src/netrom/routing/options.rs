//! The configurable knobs of NET/ROM route maintenance.
//!
//! Ports `Packet.NetRom.Routing.NetRomRoutingOptions`. These exist because
//! **NET/ROM has no single normative standard** — the canonical appendix names
//! defaults (OBSINIT 6, three routes per destination), but real nodes set the
//! quality floors and table caps differently (BPQ's per-port MINQUAL, XRouter's
//! deliberately-lower qualities), and quality-floor drift is the perennial NET/ROM
//! interop pain. We keep the canonical defaults and expose every divergence as a
//! named knob, rather than baking any one node's choices in.
//!
//! Most are read-only-ingest concerns: a higher floor simply means we *learn*
//! fewer routes. The one exception is [`NetRomRoutingOptions::obsolete_minimum`]
//! (OBSMIN), consulted only on the origination (TX) side by
//! [`super::NetRomRoutingTable::build_advertisement`] — ingest never reads it.
//!
//! `no_std`, allocation-free: a plain `Copy` record. The *capacity* caps
//! (per-destination route count, destination-list size, neighbour-list size) are
//! compile-time const generics on [`super::NetRomRoutingTable`] — the table is a
//! fixed array, not a heap map — so they are documented here but enforced by the
//! type. The runtime knobs (qualities, OBSINIT) are these fields.

/// The runtime-tunable knobs of NET/ROM route maintenance (mirrors C#
/// `NetRomRoutingOptions`). The structural caps are type-level const generics on
/// the table (see [`super::NetRomRoutingTable`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NetRomRoutingOptions {
    /// The path quality assumed for a directly-heard neighbour we have no
    /// configured link quality for — the quality of the assumed direct route to a
    /// broadcast's originator. Canonical default-port path quality is **192** (a
    /// common direct-link convention; the appendix's worked examples and BPQ both
    /// sit in the 192–203 band).
    pub default_neighbour_quality: u8,

    /// The worst quality a route may have and still be kept (MINQUAL). A derived
    /// route quality below this is dropped. Canonical floor is **0** (keep
    /// everything above zero); operators commonly raise it to 128/150/180 to reject
    /// mislabelled-neighbour qualities, so it is a knob.
    ///
    /// We default to **0** (canonical, maximally-receptive) so the read-only table
    /// learns the most it can from a mixed network; the node host can raise it. A
    /// quality-0 route is always dropped regardless (the trivial-loop guard and the
    /// "never usable" rule), independent of this floor.
    pub min_quality: u8,

    /// The obsolescence count a route is (re)initialised to when a broadcast
    /// adds/refreshes it (OBSINIT). The table is swept at the broadcast interval,
    /// decrementing every route's count; at 0 the route is purged. Canonical
    /// default **6**.
    pub obsolete_initial: u8,

    /// The advertise-gate (OBSMIN): a learned route whose obsolescence has decayed
    /// below this is kept + usable but no longer re-advertised in our own NODES
    /// broadcasts — so a fading route stops being advertised before it is finally
    /// purged at 0. Canonical / BPQ default **4**; a value ≤ 1 advertises every
    /// kept route. **Consulted only on the origination (TX) side** by
    /// [`super::NetRomRoutingTable::build_advertisement`]; ingest never reads it.
    pub obsolete_minimum: u8,
}

impl NetRomRoutingOptions {
    /// The canonical defaults: default-neighbour quality 192, MINQUAL 0 (keep all
    /// above zero), OBSINIT 6, OBSMIN 4.
    pub const DEFAULT: Self = Self {
        default_neighbour_quality: 192,
        min_quality: 0,
        obsolete_initial: 6,
        obsolete_minimum: 4,
    };
}

impl Default for NetRomRoutingOptions {
    fn default() -> Self {
        Self::DEFAULT
    }
}
