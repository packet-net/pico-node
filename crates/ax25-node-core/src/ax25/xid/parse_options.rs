//! Leniency knobs for the XID info-field parse path — ports
//! `Packet.Ax25.Xid.XidParseOptions`.
//!
//! Mirrors the repo's spec-compliant-by-default philosophy (see
//! `docs/strict-vs-pragmatic-audit.md` and `CLAUDE.md`): the [`Strict`] default
//! ([`XidParseOptions::STRICT`]) rejects any malformed XID information field; each
//! accommodation for a non-conformant real-world peer is a named flag, defaulted
//! off. The outbound construction path ([`super::info_field::encode`]) has no
//! equivalent — it is unconditionally strict and never emits a malformed field.
//!
//! [`Strict`]: XidParseOptions::STRICT
//!
//! `no_std`, allocation-free: a `Copy` record of two flags.

/// Strict-vs-lenient parser choices for the XID info-field decode. Both fields
/// default (via [`XidParseOptions::STRICT`] / [`Default`]) to spec-strict.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct XidParseOptions {
    /// Accept a Group Length that claims more parameter-field bytes than the
    /// buffer actually contains, by clamping to the available bytes. Strict spec
    /// (§4.3.3.7 ¶1021: GL is the exact parameter-field length) rejects this.
    /// Default `false`. Mirrors C# `AllowGroupLengthOverrun`.
    pub allow_group_length_overrun: bool,

    /// Accept a PI/PL whose PV runs past the end of the parameter field (a
    /// trailing PI with no PL octet, or a PL larger than the remaining bytes), by
    /// taking only the bytes that remain. Strict spec rejects this — a well-formed
    /// parameter field is an exact run of complete PI/PL/PV triples. Default
    /// `false`. Mirrors C# `AllowTruncatedParameter`.
    pub allow_truncated_parameter: bool,
}

impl XidParseOptions {
    /// Spec-strict: reject any malformed XID information field. The default.
    /// Mirrors C# `XidParseOptions.Strict`.
    pub const STRICT: Self = Self {
        allow_group_length_overrun: false,
        allow_truncated_parameter: false,
    };

    /// Lenient: tolerate a short/over-claimed Group Length and a truncated
    /// trailing parameter. Use for ingesting frames from peers that mis-size the
    /// XID info field; never for outbound construction. Mirrors C#
    /// `XidParseOptions.Lenient`.
    pub const LENIENT: Self = Self {
        allow_group_length_overrun: true,
        allow_truncated_parameter: true,
    };

    /// The strict preset (spec-compliant). See [`Self::STRICT`].
    pub const fn strict() -> Self {
        Self::STRICT
    }

    /// The lenient preset. See [`Self::LENIENT`].
    pub const fn lenient() -> Self {
        Self::LENIENT
    }
}

impl Default for XidParseOptions {
    /// Strict — matches the C# `TryParse` parameterless default (spec-strict).
    fn default() -> Self {
        Self::STRICT
    }
}
