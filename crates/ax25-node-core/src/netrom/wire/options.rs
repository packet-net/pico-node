//! Per-call configuration for the NET/ROM wire-parse paths — the named-divergence
//! options record.
//!
//! Ports `Packet.NetRom.Wire.NetRomParseOptions`. Each pragmatic accommodation for
//! a real-world node's divergence from the canonical NET/ROM wire format is a
//! named, individually-toggleable flag — exactly the `Ax25ParseOptions` pattern
//! this project uses for AX.25.
//!
//! **There is no single normative NET/ROM standard.** The closest thing to
//! canonical is the original protocol appendix; in practice **G8BPQ / LinBPQ is
//! the de-facto reference**, with XRouter and the Linux kernel `netrom` family
//! diverging. We treat them all as interop targets, *not* reference truth — the
//! same discipline the repo mandates for AX.25. So the parser is faithful to the
//! canonical appendix by default, and every divergence we accommodate is a flag
//! here (defaulted to preserve the canonical reading), surfaced in the relevant
//! peer preset.
//!
//! The current divergences are about *tolerance of the table dump*, not the field
//! layout (the 0xFF signature, the 6-byte alias, and the 21-byte destination
//! entries are universal). A node that pads its final UI frame, or that runs an
//! entry count not landing exactly on a 21-byte boundary, should not make us drop
//! the whole frame — but accepting that is opt-in, so a strict caller can still
//! reject a malformed dump.
//!
//! `no_std`, allocation-free: a plain `Copy` record of flags.

/// Named, individually-toggleable pragmatic-accommodation flags for the NET/ROM
/// NODES-broadcast parser (mirrors C# `NetRomParseOptions`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NetRomParseOptions {
    /// Accept a routing-info region whose length is not an exact multiple of the
    /// 21-byte entry size: parse as many whole 21-byte entries as fit and ignore a
    /// short trailing remainder. Strict canonical NET/ROM emits only whole entries,
    /// so a remainder means either trailing pad or a truncated frame.
    ///
    /// Driver: real nodes (BPQ included) have been observed padding the final UI
    /// frame of a multi-frame NODES dump, and a noisy RF link can clip the tail of
    /// a frame. Dropping every learned route because the *last* entry is short
    /// would be hostile; we keep the whole entries we did parse. Default `true`
    /// (lenient) — this is read-only ingest of third-party broadcasts, where
    /// resilience matters more than rejecting a stray byte.
    pub allow_trailing_partial_entry: bool,

    /// Accept a NODES broadcast carrying *zero* destination entries (just the 0xFF
    /// signature + the 6-byte sender alias, info field exactly 7 bytes).
    ///
    /// A node with an empty routing table, or one announcing only its own presence,
    /// can emit a header-only broadcast. The canonical appendix frames the entry
    /// list as "repeated up to 11 times" — i.e. zero is in range. Still a flag
    /// (default `true`) so a caller that wants to treat a contentless broadcast as
    /// malformed can opt out.
    pub allow_empty_destination_list: bool,
}

impl NetRomParseOptions {
    /// Strict canonical NET/ROM — every accommodation disabled. A broadcast is
    /// accepted only if its routing-info region is an exact multiple of 21 bytes
    /// and contains at least one destination entry.
    pub const STRICT: Self = Self {
        allow_trailing_partial_entry: false,
        allow_empty_destination_list: false,
    };

    /// Accept-everything mode (the kitchen sink). All currently-known
    /// accommodations enabled. The convenience [`super::NodesBroadcast::try_parse`]
    /// path uses this — read-only promiscuous ingest wants to be forgiving.
    pub const LENIENT: Self = Self {
        allow_trailing_partial_entry: true,
        allow_empty_destination_list: true,
    };

    /// BPQ / LinBPQ-flavoured leniency (the de-facto reference node). Today the
    /// same value as [`Self::LENIENT`]; kept named so a future BPQ-specific quirk
    /// lands here without churning call sites.
    pub const BPQ: Self = Self::LENIENT;

    /// XRouter-flavoured leniency (Paula G8PZT). Today identical to
    /// [`Self::LENIENT`]. XRouter's notable divergence is the *quality* it
    /// advertises (its RTT→quality conversion is deliberately lower — the "British
    /// notion of quality"), which is a routing-table concern handled in
    /// [`crate::netrom::routing`], not a wire-parse concern — the bytes still parse
    /// identically.
    pub const XROUTER: Self = Self::LENIENT;
}

impl Default for NetRomParseOptions {
    /// The lenient default — promiscuous read-only ingest is forgiving.
    fn default() -> Self {
        Self::LENIENT
    }
}
