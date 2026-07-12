//! Per-neighbour capability cache — ports `Packet.Node.Core.Capabilities`
//! (`PeerCapabilityCache` + `PeerCapabilityRecord` + `PeerDialPlan` /
//! `PeerDialPolicy`) as a fixed-capacity, `no_std`, alloc-free structure.
//!
//! Remembers, per (port, neighbour), whether it supports v2.2/SABME
//! ([`PeerCapabilityRecord::supports_extended`]) and whether it answers a
//! pre-session XID ([`PeerCapabilityRecord::supports_srej_via_xid`]), so a dial
//! can skip probes a known non-answerer would only stall on, and re-probe a
//! learned negative after ~30 days. The decision is [`PeerCapabilityCache::plan_dial`];
//! the post-dial learning is [`PeerCapabilityCache::record_outcome`].
//!
//! **Deviations from the C# original (all embedded-shape, behaviour-preserving):**
//! - No SQLite / [`store`](https://www.nuget.org/): in-memory only. On a Pico the
//!   record set is tiny (research §6), so a fixed `[Option<_>; N]` replaces the
//!   `ConcurrentDictionary`; when full, the least-recently-probed record is evicted.
//! - Port identity is a `u8` port index (the fw assigns each transport one) rather
//!   than the desktop's `string PortId` — an alloc-free key for the fixed store.
//! - Time is an explicit `now_ms: u64` argument (a monotonic/RTC millisecond
//!   stamp the caller supplies) rather than a `TimeProvider` — core has no wall
//!   clock. The 30-day staleness window is [`STALE_AFTER_MS`].

use crate::ax25::Callsign;

/// A learned negative is re-probed after this long (30 days, in ms), in case the
/// peer (or its firmware) changed. Mirrors C# `PeerCapabilityCache.StaleAfter`.
pub const STALE_AFTER_MS: u64 = 30 * 24 * 60 * 60 * 1000;

/// How optimistic a dial should be before anything is learned about the peer.
/// Mirrors C# `PeerDialPolicy`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PeerDialPolicy {
    /// A node-to-node interlink: conservative — stay mod-8 until the peer is proven
    /// extended, and probe SREJ via a pre-connect XID.
    Interlink,
    /// A user-initiated connect: optimistic — offer SABME by default.
    UserConnect,
}

/// The dial decision produced by [`PeerCapabilityCache::plan_dial`]. Mirrors C#
/// `PeerDialPlan`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PeerDialPlan {
    /// Offer SABME (mod-128) rather than SABM (mod-8).
    pub extended: bool,
    /// Send a pre-connect XID to probe / negotiate SREJ (moot on the extended path).
    pub pre_connect_xid: bool,
}

/// One learned (port, peer) capability record. `None` on a capability dimension
/// means "never probed". Mirrors C# `PeerCapabilityRecord`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PeerCapabilityRecord {
    /// The port (transport) index this record is keyed on.
    pub port_id: u8,
    /// The neighbour this record describes.
    pub peer: Callsign,
    /// `Some(true)` = speaks v2.2/SABME; `Some(false)` = refused / degraded to
    /// mod-8; `None` = never probed.
    pub supports_extended: Option<bool>,
    /// `Some(true)` = answers a pre-session XID and negotiated SREJ; `Some(false)`
    /// = does not answer XID; `None` = never probed.
    pub supports_srej_via_xid: Option<bool>,
    /// When this record was last probed (ms stamp from the caller's clock).
    pub last_probed_ms: u64,
    /// When an extended dial last degraded to mod-8 (ms stamp), if ever.
    pub last_refused_ms: Option<u64>,
}

/// A fixed-capacity, (port, peer)-keyed capability cache. `N` is the maximum number
/// of records; a full cache evicts the least-recently-probed record on insert.
#[derive(Debug)]
pub struct PeerCapabilityCache<const N: usize> {
    records: [Option<PeerCapabilityRecord>; N],
}

impl<const N: usize> Default for PeerCapabilityCache<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> PeerCapabilityCache<N> {
    /// A fresh, empty cache.
    pub fn new() -> Self {
        Self {
            records: core::array::from_fn(|_| None),
        }
    }

    /// Decide how to dial `peer` on `port_id`, given the caller's clock `now_ms`.
    /// A miss or a stale record falls back to the optimistic `policy` default; a
    /// fresh learned positive is honoured (offer SABME); a fresh learned negative
    /// is skipped (mod-8, and skip the pre-connect XID for a known non-answerer).
    /// Mirrors C# `PeerCapabilityCache.PlanDial`.
    pub fn plan_dial(
        &self,
        port_id: u8,
        peer: &Callsign,
        policy: PeerDialPolicy,
        now_ms: u64,
    ) -> PeerDialPlan {
        let rec = self.lookup(port_id, peer);

        // Extended: a fresh learned answer wins; else the policy's optimistic
        // default (UserConnect offers SABME; Interlink stays mod-8).
        let extended = if fresh(rec, rec.and_then(|r| r.supports_extended), now_ms) {
            rec.unwrap().supports_extended.unwrap()
        } else {
            policy == PeerDialPolicy::UserConnect
        };

        // Pre-connect XID: moot on the extended path. Off it, send the XID unless we
        // have freshly learned this peer does NOT answer it.
        let known_non_answerer = fresh(rec, rec.and_then(|r| r.supports_srej_via_xid), now_ms)
            && rec.and_then(|r| r.supports_srej_via_xid) == Some(false);
        let pre_connect_xid = !extended && !known_non_answerer;

        PeerDialPlan {
            extended,
            pre_connect_xid,
        }
    }

    /// Record what a returned dial observed. **Plan-aware**: a dimension is learned
    /// only when the dial actually probed it — a mod-8 dial proves nothing about
    /// extended capability (leaves `supports_extended` untouched); a dial with no
    /// pre-connect XID leaves `supports_srej_via_xid` untouched. Mirrors C#
    /// `PeerCapabilityCache.RecordOutcome`.
    #[allow(clippy::too_many_arguments)]
    pub fn record_outcome(
        &mut self,
        port_id: u8,
        peer: Callsign,
        dialed_extended: bool,
        observed_is_extended: bool,
        dialed_pre_connect_xid: bool,
        observed_srej_enabled: bool,
        now_ms: u64,
    ) {
        let existing = self.lookup(port_id, &peer).copied();

        // Only learn a dimension we actually probed; else carry the prior value.
        let supports_extended = if dialed_extended {
            Some(observed_is_extended)
        } else {
            existing.and_then(|r| r.supports_extended)
        };
        let supports_srej_via_xid = if dialed_pre_connect_xid {
            Some(observed_srej_enabled)
        } else {
            existing.and_then(|r| r.supports_srej_via_xid)
        };

        // LastRefused stamps an extended degrade (offered SABME, came back mod-8);
        // else carry forward.
        let last_refused_ms = if dialed_extended && !observed_is_extended {
            Some(now_ms)
        } else {
            existing.and_then(|r| r.last_refused_ms)
        };

        self.upsert(PeerCapabilityRecord {
            port_id,
            peer,
            supports_extended,
            supports_srej_via_xid,
            last_probed_ms: now_ms,
            last_refused_ms,
        });
    }

    /// Forget one (port, peer). Returns whether an entry was present.
    pub fn forget(&mut self, port_id: u8, peer: &Callsign) -> bool {
        if let Some(i) = self.index_of(port_id, peer) {
            self.records[i] = None;
            true
        } else {
            false
        }
    }

    /// Every cached record, in slot order.
    pub fn iter(&self) -> impl Iterator<Item = &PeerCapabilityRecord> {
        self.records.iter().filter_map(|r| r.as_ref())
    }

    /// Number of records currently held.
    pub fn len(&self) -> usize {
        self.records.iter().filter(|r| r.is_some()).count()
    }

    /// Whether the cache holds no records.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// The record for (port, peer), if present.
    pub fn lookup(&self, port_id: u8, peer: &Callsign) -> Option<&PeerCapabilityRecord> {
        self.index_of(port_id, peer)
            .and_then(|i| self.records[i].as_ref())
    }

    fn index_of(&self, port_id: u8, peer: &Callsign) -> Option<usize> {
        self.records.iter().position(|r| {
            r.as_ref()
                .is_some_and(|rec| rec.port_id == port_id && rec.peer == *peer)
        })
    }

    /// Insert or replace `rec` by its (port, peer) key. When the cache is full and
    /// the key is new, evict the least-recently-probed record.
    fn upsert(&mut self, rec: PeerCapabilityRecord) {
        if let Some(i) = self.index_of(rec.port_id, &rec.peer) {
            self.records[i] = Some(rec);
            return;
        }
        if let Some(i) = self.records.iter().position(|r| r.is_none()) {
            self.records[i] = Some(rec);
            return;
        }
        // Full: evict the least-recently-probed record.
        let mut victim = 0usize;
        let mut oldest = u64::MAX;
        for (i, slot) in self.records.iter().enumerate() {
            if let Some(existing) = slot {
                if existing.last_probed_ms <= oldest {
                    oldest = existing.last_probed_ms;
                    victim = i;
                }
            }
        }
        self.records[victim] = Some(rec);
    }
}

/// A learned dimension is fresh when the record exists, that dimension has a value,
/// and the record was probed within the staleness window. A never-probed (`None`)
/// dimension is never fresh. Mirrors C# `PeerCapabilityCache.Fresh`.
fn fresh(rec: Option<&PeerCapabilityRecord>, dimension: Option<bool>, now_ms: u64) -> bool {
    match rec {
        Some(r) => dimension.is_some() && now_ms.saturating_sub(r.last_probed_ms) < STALE_AFTER_MS,
        None => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const PORT: u8 = 0; // "vhf0"
    const HF: u8 = 1; // "hf0"
    const T0: u64 = 1_000_000; // arbitrary base ms stamp

    fn peer() -> Callsign {
        Callsign::parse("GB7RDG-7").unwrap()
    }

    fn cache() -> PeerCapabilityCache<8> {
        PeerCapabilityCache::new()
    }

    // ─── plan_dial: miss ⇒ optimistic policy default ─────────────────────

    #[test]
    fn plan_dial_miss_user_connect_offers_sabme_and_no_xid() {
        let plan = cache().plan_dial(PORT, &peer(), PeerDialPolicy::UserConnect, T0);
        assert!(plan.extended);
        assert!(!plan.pre_connect_xid);
    }

    #[test]
    fn plan_dial_miss_interlink_stays_mod8_and_sends_xid() {
        let plan = cache().plan_dial(PORT, &peer(), PeerDialPolicy::Interlink, T0);
        assert!(!plan.extended);
        assert!(plan.pre_connect_xid);
    }

    // ─── plan_dial: fresh learned answers ────────────────────────────────

    #[test]
    fn plan_dial_fresh_positive_extended_honoured_even_for_interlink() {
        let mut c = cache();
        c.record_outcome(PORT, peer(), true, true, false, false, T0);
        let plan = c.plan_dial(PORT, &peer(), PeerDialPolicy::Interlink, T0);
        assert!(plan.extended);
        assert!(!plan.pre_connect_xid);
    }

    #[test]
    fn plan_dial_fresh_negative_extended_skips_sabme_even_for_user_connect() {
        let mut c = cache();
        c.record_outcome(PORT, peer(), true, false, false, false, T0);
        let plan = c.plan_dial(PORT, &peer(), PeerDialPolicy::UserConnect, T0);
        assert!(!plan.extended);
        assert!(plan.pre_connect_xid, "unknown XID answerer ⇒ still probe");
    }

    #[test]
    fn plan_dial_fresh_non_xid_answerer_skips_the_pre_connect_xid() {
        let mut c = cache();
        c.record_outcome(PORT, peer(), false, false, true, false, T0);
        let plan = c.plan_dial(PORT, &peer(), PeerDialPolicy::Interlink, T0);
        assert!(!plan.extended);
        assert!(!plan.pre_connect_xid, "known non-answerer ⇒ skip the stall");
    }

    #[test]
    fn plan_dial_fresh_xid_answerer_still_sends_the_pre_connect_xid() {
        let mut c = cache();
        c.record_outcome(PORT, peer(), false, false, true, true, T0);
        let plan = c.plan_dial(PORT, &peer(), PeerDialPolicy::Interlink, T0);
        assert!(!plan.extended);
        assert!(plan.pre_connect_xid);
    }

    // ─── plan_dial: staleness re-probe ───────────────────────────────────

    #[test]
    fn plan_dial_stale_negative_re_probes_with_the_policy_default() {
        let mut c = cache();
        c.record_outcome(PORT, peer(), true, false, false, false, T0);
        let later = T0 + STALE_AFTER_MS + 24 * 60 * 60 * 1000; // +1 day past window
        assert!(c.plan_dial(PORT, &peer(), PeerDialPolicy::UserConnect, later).extended);
    }

    #[test]
    fn plan_dial_just_inside_the_window_is_still_fresh() {
        let mut c = cache();
        c.record_outcome(PORT, peer(), true, false, false, false, T0);
        let later = T0 + STALE_AFTER_MS - 60 * 1000; // 1 min inside the window
        assert!(!c.plan_dial(PORT, &peer(), PeerDialPolicy::UserConnect, later).extended);
    }

    // ─── record_outcome: plan-aware learning ─────────────────────────────

    #[test]
    fn record_outcome_dialed_extended_sets_supports_extended() {
        let mut c = cache();
        c.record_outcome(PORT, peer(), true, true, false, false, T0);
        let rec = c.iter().next().unwrap();
        assert_eq!(rec.supports_extended, Some(true));
        assert_eq!(rec.supports_srej_via_xid, None, "never probed ⇒ null");
    }

    #[test]
    fn record_outcome_dialed_mod8_does_not_touch_supports_extended() {
        let mut c = cache();
        c.record_outcome(PORT, peer(), true, true, false, false, T0);
        c.record_outcome(PORT, peer(), false, false, false, false, T0 + 300_000);
        assert_eq!(c.iter().next().unwrap().supports_extended, Some(true));
    }

    #[test]
    fn record_outcome_dialed_mod8_from_unknown_leaves_extended_null() {
        let mut c = cache();
        c.record_outcome(PORT, peer(), false, false, false, false, T0);
        assert_eq!(c.iter().next().unwrap().supports_extended, None);
    }

    #[test]
    fn record_outcome_dialed_xid_sets_supports_srej_via_xid() {
        let mut c = cache();
        c.record_outcome(PORT, peer(), false, false, true, true, T0);
        let rec = c.iter().next().unwrap();
        assert_eq!(rec.supports_srej_via_xid, Some(true));
        assert_eq!(rec.supports_extended, None);
    }

    #[test]
    fn record_outcome_no_xid_does_not_touch_supports_srej_via_xid() {
        let mut c = cache();
        c.record_outcome(PORT, peer(), false, false, true, true, T0);
        c.record_outcome(PORT, peer(), true, true, false, false, T0 + 300_000);
        let rec = c.iter().next().unwrap();
        assert_eq!(rec.supports_srej_via_xid, Some(true), "preserved");
        assert_eq!(rec.supports_extended, Some(true), "the probed dimension");
    }

    #[test]
    fn record_outcome_sets_last_refused_on_an_extended_degrade() {
        let mut c = cache();
        c.record_outcome(PORT, peer(), true, false, false, false, T0);
        assert_eq!(c.iter().next().unwrap().last_refused_ms, Some(T0));
    }

    #[test]
    fn record_outcome_does_not_set_last_refused_on_a_clean_extended_dial() {
        let mut c = cache();
        c.record_outcome(PORT, peer(), true, true, false, false, T0);
        assert_eq!(c.iter().next().unwrap().last_refused_ms, None);
    }

    #[test]
    fn record_outcome_carries_last_refused_forward_on_a_non_degrade_dial() {
        let mut c = cache();
        c.record_outcome(PORT, peer(), true, false, false, false, T0);
        c.record_outcome(PORT, peer(), false, false, true, true, T0 + 300_000);
        assert_eq!(c.iter().next().unwrap().last_refused_ms, Some(T0), "carried forward");
    }

    #[test]
    fn record_outcome_stamps_last_probed_with_the_clock() {
        let mut c = cache();
        let t = T0 + 3 * 60 * 60 * 1000;
        c.record_outcome(PORT, peer(), true, true, false, false, t);
        assert_eq!(c.iter().next().unwrap().last_probed_ms, t);
    }

    // ─── per-link keying + forget + eviction ─────────────────────────────

    #[test]
    fn records_are_keyed_per_port_and_peer() {
        let mut c = cache();
        c.record_outcome(PORT, peer(), true, true, false, false, T0);
        c.record_outcome(HF, peer(), true, false, false, false, T0);
        assert_eq!(c.len(), 2);
        assert!(c.plan_dial(PORT, &peer(), PeerDialPolicy::Interlink, T0).extended);
        assert!(!c.plan_dial(HF, &peer(), PeerDialPolicy::UserConnect, T0).extended);
    }

    #[test]
    fn forget_removes_the_entry() {
        let mut c = cache();
        c.record_outcome(PORT, peer(), true, true, false, false, T0);
        assert!(c.forget(PORT, &peer()));
        assert!(c.is_empty());
        assert!(!c.forget(PORT, &peer()), "already gone");
    }

    #[test]
    fn full_cache_evicts_the_least_recently_probed() {
        let mut c: PeerCapabilityCache<2> = PeerCapabilityCache::new();
        let a = Callsign::parse("G7AAA").unwrap();
        let b = Callsign::parse("G7BBB").unwrap();
        let d = Callsign::parse("G7DDD").unwrap();
        c.record_outcome(PORT, a, true, true, false, false, T0); // oldest
        c.record_outcome(PORT, b, true, true, false, false, T0 + 1000);
        // Cache full; inserting a third evicts A (oldest last_probed).
        c.record_outcome(PORT, d, true, true, false, false, T0 + 2000);
        assert_eq!(c.len(), 2);
        assert!(c.lookup(PORT, &a).is_none(), "A evicted");
        assert!(c.lookup(PORT, &b).is_some());
        assert!(c.lookup(PORT, &d).is_some());
    }
}
