//! The carrier-sense (CSMA) seam at the link multiplexer's transmit path.
//!
//! Ports `Packet.Ax25.Transport.ICarrierSense` + the fail-open decision of
//! `Packet.Ax25.Session.CarrierSenseGate`. Before the medium-access arbiter keys
//! the radio (grants an `LM-SEIZE`), it consults a [`CarrierSense`] source: while
//! the channel is busy the seize is *deferred* (a half-duplex radio port must not
//! key over received traffic), and it is granted once the channel clears.
//!
//! ## Fail-open, off by default
//!
//! Only a *definite* busy defers. An unknown state ([`None`] — no source, no DCD
//! edge yet, telemetry faulted) is treated as clear: traffic must never stop
//! because carrier-sense went dark (mirrors `CarrierSenseGate`'s `ChannelBusy !=
//! true` fast path). The [`super::manager::SessionManager`] holds *no* source by
//! default, which is the always-clear degenerate gate — so a node on a full-duplex
//! AXUDP / KISS-TCP wire keys up immediately, exactly as before. The seam touches
//! only the *physical* keyup grant; it never alters an SDL transition.
//!
//! ## `no_std` note
//!
//! The desktop gate polls a clock every slot-time and bounds the wait. The pico
//! runtime is synchronous and event-driven: the carrier state is sampled at each
//! seize-grant point, and a busy channel leaves the seize pending to be retried on
//! a later drive — the same fail-open "only a definite busy holds us" policy, minus
//! the async slot-time loop (there is no blocking wait on the M0+ hot path).

/// A source of hardware carrier-sense (DCD): "is the channel busy right now?".
///
/// Ports `ICarrierSense`. A transport or radio-control bridge that can genuinely
/// observe channel occupancy implements it; a consumer with none simply supplies no
/// source (the gate then treats the channel as always-clear). Requires [`core::fmt::Debug`]
/// so a holder can derive `Debug`.
pub trait CarrierSense: core::fmt::Debug {
    /// Last known carrier-sense state: `Some(true)` while the channel is busy (RF on
    /// channel / hardware DCD asserted), `Some(false)` when idle, and `None` when
    /// unknown (no report yet, or the source can't sense carrier). Mirrors
    /// `ICarrierSense.ChannelBusy` (`bool?`).
    fn channel_busy(&self) -> Option<bool>;

    /// Whether the channel is clear to key up. Fail-open: anything other than a
    /// definite `Some(true)` is clear (mirrors `CarrierSenseGate`'s `ChannelBusy !=
    /// true`). A busy channel is the only thing that defers a seize.
    fn is_clear(&self) -> bool {
        self.channel_busy() != Some(true)
    }
}

/// The always-clear source — reports the channel idle unconditionally. Equivalent
/// to supplying no source at all; provided for callers that want to wire an explicit
/// clear gate (and as the documented degenerate form of the C# null-source gate).
#[derive(Debug, Clone, Copy, Default)]
pub struct AlwaysClear;

impl CarrierSense for AlwaysClear {
    fn channel_busy(&self) -> Option<bool> {
        Some(false)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A carrier whose state is fixed at construction — the test double for the seam.
    #[derive(Debug, Clone, Copy)]
    struct FixedCarrier(Option<bool>);
    impl CarrierSense for FixedCarrier {
        fn channel_busy(&self) -> Option<bool> {
            self.0
        }
    }

    #[test]
    fn definite_busy_is_the_only_thing_that_defers() {
        assert!(!FixedCarrier(Some(true)).is_clear(), "busy defers");
        assert!(FixedCarrier(Some(false)).is_clear(), "idle is clear");
        assert!(FixedCarrier(None).is_clear(), "unknown fails open (clear)");
    }

    #[test]
    fn always_clear_is_clear() {
        assert_eq!(AlwaysClear.channel_busy(), Some(false));
        assert!(AlwaysClear.is_clear());
    }
}
