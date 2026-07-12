//! Radio control / telemetry abstraction — the Rust port of `Packet.Radio`.
//!
//! A control/side-channel to the radio *behind* a modem: the layer that surfaces
//! what standard KISS cannot — receiver RSSI, hardware carrier-sense (DCD) edges,
//! transmitter keying — by talking the radio's own CAT-style serial protocol.
//! The first (and, today, only) concrete driver is Tait CCDI ([`tait`]).
//!
//! ## Parity divergences (no_std, no-FPU)
//!
//! The C# surface carries signal levels as `float` dBm and timestamps as
//! `DateTimeOffset`. On the M0+ there is no FPU, so this port is **integer-only**:
//!
//! - RSSI / SNR / noise-floor levels are `i16` **tenths of a dBm** (e.g. `-456`
//!   == −45.6 dBm), mirroring the CCDI wire unit (0.1 dB) directly rather than
//!   dividing to a float. See [`RadioMetadata`].
//! - Timestamps are `u64` **microseconds** of an opaque monotonic clock (the
//!   firmware supplies `embassy_time::Instant` micros; host tests supply a
//!   synthetic timeline), matching the `catalog::transmission_us` idiom.
//!
//! These are the only intentional departures from the C# behaviour; the wire
//! codecs ([`tait::ccdi`]) are byte-for-byte identical.

pub mod tait;

/// Feature flags for a radio control channel — which of the abstract operations
/// the concrete driver actually supports. Mirrors `Packet.Radio.RadioCapabilities`
/// (a `[Flags]` enum); implemented here as a small bit-set newtype to avoid an
/// external `bitflags` dependency (the core crate's one-dep rule).
///
/// Reserved flags with no operation yet ([`Self::CHANNEL_CHANGE`],
/// [`Self::FREQUENCY_CONTROL`], [`Self::TX_POWER_CONTROL`]) exist so a driver can
/// advertise them ahead of the abstraction growing those members, exactly as the
/// C# enum reserves them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RadioCapabilities(u8);

impl RadioCapabilities {
    /// No control-channel features available.
    pub const NONE: Self = Self(0);
    /// [`tait::driver::TaitCcdiRadio::read_rssi_tenths`] works.
    pub const RSSI_READ: Self = Self(1 << 0);
    /// Carrier-sense (DCD) edges are reported and channel-busy is maintained.
    pub const CARRIER_SENSE: Self = Self(1 << 1);
    /// The transmitter can be keyed/unkeyed under program control.
    pub const TRANSMITTER_CONTROL: Self = Self(1 << 2);
    /// Reserved: the radio can switch among programmed channels.
    pub const CHANNEL_CHANGE: Self = Self(1 << 3);
    /// Reserved: the radio accepts direct frequency programming.
    pub const FREQUENCY_CONTROL: Self = Self(1 << 4);
    /// Reserved: the radio accepts TX power-level changes.
    pub const TX_POWER_CONTROL: Self = Self(1 << 5);
    /// The driver can supply a radio-native small-datagram side channel (Tait SDM).
    pub const SIDE_CHANNEL: Self = Self(1 << 6);

    /// `true` when every flag set in `other` is also set in `self`. Mirrors
    /// `HasFlag`.
    pub const fn contains(self, other: Self) -> bool {
        (self.0 & other.0) == other.0
    }

    /// The raw bit pattern.
    pub const fn bits(self) -> u8 {
        self.0
    }

    /// The union of two capability sets (const-friendly `|`).
    pub const fn union(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }
}

impl core::ops::BitOr for RadioCapabilities {
    type Output = Self;
    fn bitor(self, rhs: Self) -> Self {
        Self(self.0 | rhs.0)
    }
}

/// One hardware carrier-sense (DCD) edge. Mirrors `Packet.Radio.CarrierSenseChange`.
///
/// This is a *hardware* data-carrier detect — it leads the modem's decoded frame
/// by the whole preamble + frame duration, which is what makes it valuable for
/// CSMA and for RSSI window attribution ([`rssi_tagging`](tait)).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CarrierSenseChange {
    /// `true` = RF appeared on channel; `false` = the channel went quiet.
    pub busy: bool,
    /// When the radio reported the edge (`u64` micros of the monotonic clock).
    pub at_us: u64,
}

/// Per-frame radio signal metadata — the integer port of
/// `Packet.Ax25.Transport.RadioMetadata`.
///
/// Every field the C# record carries is present, with the two documented no-FPU
/// substitutions: levels are `i16` tenths-of-a-dB(m) and instants/durations are
/// `u64`/`i64` microseconds. All fields are optional except the sample count
/// (which is `0` when unknown, matching the C# `stats?.Count ?? 0`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RadioMetadata {
    /// Attributed RSSI (median of in-window samples), tenths of a dBm.
    pub rssi_dbm_tenths: Option<i16>,
    /// [`Self::rssi_dbm_tenths`] − [`Self::noise_floor_dbm_tenths`], tenths of a dB.
    pub snr_db_tenths: Option<i16>,
    /// The tracked channel-idle noise floor, tenths of a dBm.
    pub noise_floor_dbm_tenths: Option<i16>,
    /// Weakest in-window RSSI sample, tenths of a dBm.
    pub rssi_min_dbm_tenths: Option<i16>,
    /// Strongest in-window RSSI sample, tenths of a dBm.
    pub rssi_max_dbm_tenths: Option<i16>,
    /// How many RSSI samples produced the statistics (`0` when none / unknown).
    pub rssi_sample_count: u16,
    /// When the carrier for this frame's window rose (`u64` micros), if known.
    pub carrier_rise_at_us: Option<u64>,
    /// Zero-based position of this frame within its carrier window; `None` when the
    /// source has no carrier-sense channel.
    pub burst_index: Option<u16>,
    /// Approximate on-air duration, micros: wire bytes (+FCS +flag) × 8 ÷ bit rate.
    pub estimated_airtime_us: Option<u64>,
    /// Measured carrier time before this frame's data (excess-TXDELAY input), micros.
    /// Only populated for the first frame of a burst; may be negative.
    pub pre_data_carrier_us: Option<i64>,
}
