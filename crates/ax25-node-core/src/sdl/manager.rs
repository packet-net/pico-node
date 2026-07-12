//! A fixed-capacity, peer-keyed session manager — the on-target home of the
//! per-link [`Session`] array.
//!
//! Mirrors `Packet.Ax25.Session.Ax25Listener`'s session collection, but as a
//! **fixed array** rather than the desktop's unbounded LRU dictionary: a Pico node
//! serves a handful of links (research §6), so a small `[Option<Slot>; N]` with no
//! heap map is the right shape. The firmware owns one `SessionManager`; each
//! transport, on decoding an inbound frame, calls [`SessionManager::post`] keyed by
//! the peer callsign, and the manager routes the event to that peer's [`Session`]
//! (creating a slot on first contact), driving it against the shared timer service
//! and the peer's [`WireSink`]. Outbound frames accumulate on each slot's sink for
//! the transport to flush.
//!
//! This is host-testable logic (no I/O, no embassy) — the firmware adds only the
//! timer task + the socket/UART plumbing around it. `no_std` + `alloc`.

extern crate alloc;
use alloc::boxed::Box;
use alloc::vec::Vec;

use crate::ax25::xid::{info_field, XidParameters};
use crate::ax25::Callsign;

use super::bridge::WireSink;
use super::capability::PeerDialPlan;
use super::carrier::CarrierSense;
use super::event::Event;
use super::session::Session;
use super::timer::TimerService;

/// An in-flight initiator pre-connect XID probe (the LinBPQ SREJ accommodation).
/// Present on a slot between emitting our XID *command* and its resolution — the
/// bounded-wait window of C# `Ax25Listener.NegotiateSrejBeforeConnectAsync`. The
/// SABM is deferred until either the peer's XID *response* arrives (merge via
/// [`super::mdl::apply_negotiated`] → connect) or [`SessionManager::xid_probe_timeout`]
/// fires (revert → connect).
#[derive(Debug, Clone, Copy)]
pub struct XidProbe {
    /// The parameter set we advertised in the XID command, merged against the peer's
    /// response (§6.3.2) via [`super::mdl::apply_negotiated`]. Kept verbatim so the
    /// merge runs against the exact offer we sent (matching the C# MDL's stored offer).
    pub offered: XidParameters,
    /// The local callsign this dial uses, so the deferred SABM re-dials as the same
    /// station on the timeout path.
    pub local: Callsign,
}

/// One occupied link slot: the peer it serves, its session, and its outbound wire
/// sink (which also captures the DL signals raised upward for the app/console).
#[derive(Debug, Clone)]
pub struct Slot {
    /// The remote station this slot is connected to / establishing with.
    pub peer: Callsign,
    /// The link-layer session state machine.
    pub session: Session,
    /// The outbound wire sink (encoded frames + upward DL signals accumulate here).
    pub sink: WireSink,
    /// An in-flight initiator pre-connect XID probe, if one is awaiting its response.
    /// `None` on an ordinary link. See [`XidProbe`].
    pub xid_probe: Option<XidProbe>,
}

/// A fixed-capacity, peer-keyed collection of [`Session`]s. `N` is the maximum
/// concurrent links — sized for a node, not a desktop (no heap session map).
#[derive(Debug)]
pub struct SessionManager<const N: usize> {
    local: Callsign,
    /// Whether a plain [`Self::connect`] prefers a mod-128 (SABME) dial. See
    /// [`Self::with_prefer_extended_connect`].
    prefer_extended_connect: bool,
    /// Optional carrier-sense (CSMA) source gating the `LM-SEIZE` grant. `None` (the
    /// default) is the always-clear degenerate gate — the historical full-duplex
    /// behaviour. See [`Self::set_carrier_sense`].
    carrier: Option<Box<dyn CarrierSense>>,
    slots: [Option<Slot>; N],
}

impl<const N: usize> SessionManager<N> {
    /// Build a manager for the node's own `local` callsign with all slots free.
    /// A plain [`Self::connect`] dials mod-128 (SABME) by default — matching C#
    /// `Ax25ListenerOptions.PreferExtendedConnect = true` — with automatic degrade
    /// to mod-8 SABM if the peer refuses (FRMR #45 or DM #48); see
    /// [`Self::with_prefer_extended_connect`]. No carrier-sense source is wired
    /// (always-clear); see [`Self::set_carrier_sense`].
    pub fn new(local: Callsign) -> Self {
        Self {
            local,
            prefer_extended_connect: true,
            carrier: None,
            // `Option<Slot>` isn't `Copy`, so build the array element-by-element.
            slots: core::array::from_fn(|_| None),
        }
    }

    /// Wire a carrier-sense (CSMA) source that gates every `LM-SEIZE` grant: while it
    /// reports the channel busy the seize is deferred (the radio isn't keyed over
    /// received traffic), and it is granted once the channel clears. Fail-open — an
    /// unknown state keys up. Mirrors `Ax25ListenerOptions.CarrierSense` feeding
    /// `CarrierSenseGate` (Ax25Listener.cs:263). Off by default (always-clear).
    pub fn set_carrier_sense(&mut self, source: Box<dyn CarrierSense>) {
        self.carrier = Some(source);
    }

    /// [`Self::set_carrier_sense`] as a builder, returning `self` for chaining.
    pub fn with_carrier_sense(mut self, source: Box<dyn CarrierSense>) -> Self {
        self.carrier = Some(source);
        self
    }

    /// Remove any wired carrier-sense source, restoring the always-clear gate.
    pub fn clear_carrier_sense(&mut self) {
        self.carrier = None;
    }

    /// Whether the channel is currently clear to key up. Fail-open: `true` when no
    /// source is wired or the source reports anything other than a definite busy.
    fn carrier_is_clear(&self) -> bool {
        self.carrier.as_ref().is_none_or(|c| c.is_clear())
    }

    /// Set whether a plain [`Self::connect`] prefers a mod-128 (SABME) dial with
    /// SABM/mod-8 fallback on refusal, returning `self` for chaining. Mirrors the
    /// listener option `Ax25ListenerOptions.PreferExtendedConnect` (Ax25Listener.cs:1712),
    /// and — now that both refusal degrades are present on the session (FRMR
    /// fallback #45 and the DM-refusal degrade #48) — pico matches the C# default of
    /// **true**: a v2.2-preferred dial that a pre-v2.2 peer refuses with FRMR or DM
    /// degrades to a mod-8 SABM re-establishment instead of stranding. Pass `false`
    /// here (or dial mod-8 explicitly via [`Self::connect_extended`]) to force the
    /// historical mod-8 dial.
    pub fn with_prefer_extended_connect(mut self, prefer: bool) -> Self {
        self.prefer_extended_connect = prefer;
        self
    }

    /// Set the [`Self::with_prefer_extended_connect`] preference in place.
    pub fn set_prefer_extended_connect(&mut self, prefer: bool) {
        self.prefer_extended_connect = prefer;
    }

    /// Whether a plain [`Self::connect`] prefers a mod-128 (SABME) dial.
    pub fn prefer_extended_connect(&self) -> bool {
        self.prefer_extended_connect
    }

    /// The node's local callsign.
    pub fn local(&self) -> Callsign {
        self.local
    }

    /// Number of occupied slots.
    pub fn active(&self) -> usize {
        self.slots.iter().filter(|s| s.is_some()).count()
    }

    /// Find the slot index for `peer`, if one exists.
    fn index_of(&self, peer: &Callsign) -> Option<usize> {
        self.slots
            .iter()
            .position(|s| s.as_ref().is_some_and(|slot| slot.peer == *peer))
    }

    /// Get the session for `peer`, if a slot exists (read-only).
    pub fn session_for(&self, peer: &Callsign) -> Option<&Session> {
        self.index_of(peer)
            .and_then(|i| self.slots[i].as_ref())
            .map(|slot| &slot.session)
    }

    /// Whether `peer`'s session has an `LM-SEIZE` request still awaiting a grant —
    /// `true` when a seize has been requested but deferred (e.g. by a busy carrier).
    /// `false` if there is no slot or nothing is pending.
    pub fn seize_pending(&self, peer: &Callsign) -> bool {
        self.index_of(peer)
            .and_then(|i| self.slots[i].as_ref())
            .is_some_and(|slot| slot.sink.seize_pending)
    }

    /// Ensure a slot exists for `peer`, returning its index. Returns `None` if the
    /// manager is full and `peer` has no existing slot (the caller drops the frame /
    /// replies DM — a node at capacity refuses new links). Creates the slot's
    /// [`WireSink`] addressed for the `local ↔ peer` link.
    fn ensure_slot(&mut self, peer: Callsign, local: Callsign) -> Option<usize> {
        if let Some(i) = self.index_of(&peer) {
            return Some(i);
        }
        let free = self.slots.iter().position(|s| s.is_none())?;
        self.slots[free] = Some(Slot {
            peer,
            session: Session::new(),
            sink: WireSink::new(local, peer, Vec::new()),
            xid_probe: None,
        });
        Some(free)
    }

    /// Route `event` to `peer`'s session (creating a slot on first contact),
    /// driving it against the shared `timers` and the slot's own sink. Returns the
    /// wire frames the session emitted (drained from the slot's sink), or an empty
    /// vec if the manager is full and the peer is unknown.
    ///
    /// After dispatch, a slot that has returned to [`super::session::State::Disconnected`]
    /// is freed, so a torn-down link releases its capacity.
    pub fn post(
        &mut self,
        peer: Callsign,
        event: Event,
        timers: &mut dyn TimerService,
    ) -> Vec<Vec<u8>> {
        self.post_with_local(self.local, peer, event, timers)
    }

    /// [`Self::post`], but a slot created by this call uses `local` as its own
    /// station callsign instead of the manager default. The node convention for
    /// outgoing connects made on a console user's behalf: the *user's* callsign
    /// with complemented SSID (so the far node never sees its own downlink call
    /// coming back — two simultaneous links keyed on one callsign collide in
    /// real node stacks; observed live against LinBPQ). An existing slot keeps
    /// the local it was created with.
    pub fn post_with_local(
        &mut self,
        local: Callsign,
        peer: Callsign,
        mut event: Event,
        timers: &mut dyn TimerService,
    ) -> Vec<Vec<u8>> {
        let Some(i) = self.ensure_slot(peer, local) else {
            return Vec::new();
        };
        // Sample carrier-sense once for this drive (the synchronous runtime doesn't
        // advance time mid-post). A busy channel defers the LM-SEIZE grant below;
        // no source / unknown / idle is clear. Read before borrowing the slot.
        let carrier_clear = self.carrier_is_clear();
        let slot = self.slots[i]
            .as_mut()
            .expect("slot just ensured to be present");
        slot.sink.sent.clear();

        // Initiator pre-connect XID *response* handler (mirrors the inbound-router
        // leg of `Ax25Listener.NegotiateSrejBeforeConnectAsync`): the peer's XID
        // response to a probe we sent. Merge it into our offer via the §6.3.2
        // reverts-to (`apply_negotiated`) — settling SREJ/window/N1/T1/N2 to the
        // mutual result — then proceed to connect by converting this drive into the
        // deferred `DL-CONNECT-request` (the SABM the probe was holding back). The
        // negotiated `srej_enabled` is staged on the context and survives the SABM's
        // figc4.1 `Set Version 2.0` into the established link (proven by the responder
        // path). A mod-8 probe never merges to mod-128 (our offer is mod-8), so the
        // deferred connect is always a plain SABM.
        let mut proceed_to_connect = false;
        if let Event::XidReceived(fi) = &event {
            if fi.is_response() && slot.xid_probe.is_some() {
                let response_info = fi.info.clone();
                let probe = slot.xid_probe.take().expect("is_some checked above");
                let response = info_field::parse(&response_info).unwrap_or_default();
                super::mdl::apply_negotiated(
                    &mut slot.session.context,
                    &probe.offered,
                    &response,
                );
                proceed_to_connect = true;
            }
        }
        if proceed_to_connect {
            // Fall through to the normal dispatch as the deferred connect request.
            event = Event::DlConnectRequest;
        }

        // Pre-session XID *command* responder (mirrors
        // `Ax25Listener.HandleNoCachedSession`'s XID branch): a peer doing pre-SABM
        // XID negotiation to us before any link exists — the PDN NET/ROM mod-8
        // interlink initiator opening with XID. §4.3.3.7 makes answering an XID
        // command unconditional; the negotiated params stage on this cached slot's
        // context so the *subsequent* SABM's figc4.1 t14 `Set Version 2.0` (which
        // clears only `is_extended`) preserves the staged `srej_enabled` into the
        // established link. We answer directly (connectionless — no LM-SEIZE),
        // matching C# `RespondToXidCommand`; no ConnectIndication is raised (the
        // following SABM raises it). Gated on `accept_incoming`, like SABM-accept.
        if let Event::XidReceived(fi) = &event {
            if fi.is_command
                && slot.session.state == super::session::State::Disconnected
                && slot.session.context.accept_incoming
            {
                let command_info = fi.info.clone();
                let response_info = super::mdl::respond_pre_session_xid(
                    &mut slot.session.context,
                    &command_info,
                );
                // XID is a U-frame (1 octet in both modulos); modulo is immaterial.
                slot.sink.extended = slot.session.context.is_extended;
                let bytes = slot.sink.encode_spec(&super::signal::FrameSpec::Xid {
                    is_command: false,
                    pf: true, // F=1 so the initiator's figc5.2 F_eq_1 diamond fires
                    info: response_info,
                });
                slot.sink.sent.push(bytes);
                return core::mem::take(&mut slot.sink.sent);
            }
        }

        // Track the link's negotiated modulo so the sink emits 2-octet extended
        // control on an I/S frame once the session is mod-128 (SABME-established).
        // is_extended is settled before any I/S frame is emitted (it is set on the
        // connect request / adopted from an inbound SABM/SABME, all of which emit
        // only U frames), so reading it here — before dispatch — is correct.
        slot.sink.extended = slot.session.context.is_extended;
        slot.session.post_event(event, timers, &mut slot.sink);

        // Grant LM-SEIZE when the channel is clear. On a full-duplex wire (AXUDP,
        // KISS-TCP) — or with no carrier-sense source — `carrier_clear` is always
        // true, so the channel is treated as always free (the historical behaviour).
        // The confirm drives the figc4 `AckPending` path that emits the delayed RR
        // acknowledgement — without it, received I-frames with no reply data are
        // never acked and the peer eventually declares link failure (found live
        // against LinBPQ through the console relay). Bounded: the confirm path
        // releases, it never re-seizes. When a carrier-sense source reports the
        // channel busy the seize is *deferred* — `seize_pending` stays set and a
        // later drive (once the channel clears) grants it, so a half-duplex radio
        // port never keys over received traffic.
        let mut grants = 0;
        while carrier_clear && slot.sink.seize_pending && grants < 4 {
            slot.sink.seize_pending = false;
            slot.session
                .post_event(Event::LmSeizeConfirm, timers, &mut slot.sink);
            grants += 1;
        }

        core::mem::take(&mut slot.sink.sent)

        // NB: a slot whose session has returned to Disconnected is NOT freed
        // here — its upward signals (DisconnectIndication/-Confirm) haven't
        // been drained yet, and freeing now would lose them (found wiring the
        // firmware's link-failure path). Call [`Self::reap`] after draining.
    }

    /// Initiate an outbound connect to `peer` from the manager's local callsign,
    /// choosing the modulo from [`Self::prefer_extended_connect`]. Convenience over
    /// [`Self::connect_extended`]; mirrors `Ax25Listener.ConnectAsync(remote, local, ct)`
    /// (which uses the listener's `PreferExtendedConnect` default).
    pub fn connect(&mut self, peer: Callsign, timers: &mut dyn TimerService) -> Vec<Vec<u8>> {
        self.connect_extended(self.local, peer, self.prefer_extended_connect, timers)
    }

    /// Initiate an outbound connect to `peer` from `local`, explicitly choosing the
    /// modulo. `extended = true` dials mod-128 (SABME) with SABM/mod-8 fallback on a
    /// peer's refusal; `false` dials plain mod-8 (SABM). Sets the session's
    /// `is_extended` **before** posting `DL-CONNECT-request`, so — with the default
    /// quirks — an extended dial routes through `AwaitingV22Connection` (figc4.6, via
    /// #44) and `Establish_Data_Link` emits SABME, and a subsequent FRMR refusal
    /// degrades to a mod-8 SABM re-establishment (#45). A cached session re-dialled
    /// after a prior fallback dropped it to mod-8 is re-armed to the caller's
    /// preference here. Mirrors `Ax25Listener.ConnectAsync(remote, local, bool
    /// extended, …)` (Ax25Listener.cs:412 sets `Context.IsExtended = extended`).
    ///
    /// Returns the wire frames emitted (the SABM/SABME), or empty if the manager is
    /// full and `peer` has no slot.
    pub fn connect_extended(
        &mut self,
        local: Callsign,
        peer: Callsign,
        extended: bool,
        timers: &mut dyn TimerService,
    ) -> Vec<Vec<u8>> {
        let Some(i) = self.ensure_slot(peer, local) else {
            return Vec::new();
        };
        // Choose the version before posting DL-CONNECT-request (Ax25Listener.cs:412).
        self.slots[i]
            .as_mut()
            .expect("slot just ensured to be present")
            .session
            .context
            .is_extended = extended;
        self.post_with_local(local, peer, Event::DlConnectRequest, timers)
    }

    /// Dial `peer` from `local` per a capability-cache [`PeerDialPlan`] — the
    /// dial-time seam that supplies a peer's learned XID capabilities. Pair with
    /// [`PeerCapabilityCache::plan_dial`](super::capability::PeerCapabilityCache::plan_dial)
    /// upstream and
    /// [`PeerCapabilityCache::record_outcome`](super::capability::PeerCapabilityCache::record_outcome)
    /// once the dial resolves (extended-vs-degraded observable from the session's
    /// `is_extended`, SREJ from `srej_enabled`).
    ///
    /// Two dial shapes, mirroring `Ax25Listener.ConnectAsync(remote, local, extended,
    /// preConnectXidNegotiatesSrej, ct)`:
    ///
    /// - **`plan.pre_connect_xid` (a mod-8 probe)** — begin an *initiator* pre-connect
    ///   XID probe ([`Self::begin_xid_probe`]): emit our XID *command* and enter the
    ///   bounded-wait pre-connect state WITHOUT sending SABM yet. The SABM is deferred
    ///   until either the peer's XID *response* arrives — [`Self::post`] merges it via
    ///   [`super::mdl::apply_negotiated`] and proceeds to connect — or
    ///   [`Self::xid_probe_timeout`] fires — reverting to go-back-N and dialling a
    ///   plain SABM. Mirrors the `NegotiateSrejBeforeConnectAsync` fast-probe. (The
    ///   probe is meaningless on the extended path — SABME negotiates XID post-UA — so
    ///   it is only started when `plan.extended` is false; a plan is never both.)
    /// - **otherwise (a fresh cache hit, the extended path, or a known non-answerer)**
    ///   — dial straight via [`Self::connect_extended`], honouring
    ///   [`extended`](PeerDialPlan::extended) (SABME vs SABM). No probe: the whole
    ///   point of the cache is to skip the stall for a peer we already know.
    pub fn connect_planned(
        &mut self,
        local: Callsign,
        peer: Callsign,
        plan: PeerDialPlan,
        timers: &mut dyn TimerService,
    ) -> Vec<Vec<u8>> {
        if plan.pre_connect_xid && !plan.extended {
            self.begin_xid_probe(local, peer)
        } else {
            self.connect_extended(local, peer, plan.extended, timers)
        }
    }

    /// Begin an initiator pre-connect XID probe to `peer` from `local`: seed the
    /// session context SREJ-capable ([`super::mdl::begin_pre_connect_xid`]), emit our
    /// XID *command* advertising that offer, and arm the [`XidProbe`] pending state —
    /// but do NOT send SABM yet. The deferred SABM is driven later, by [`Self::post`]
    /// on the peer's XID response (merge + connect) or by [`Self::xid_probe_timeout`]
    /// (revert + connect). Mirrors the offer step of C#
    /// `Ax25Listener.NegotiateSrejBeforeConnectAsync`. Returns the XID command frame
    /// (or empty if the manager is full and `peer` has no slot). A no-op re-arm if a
    /// probe is already pending — the caller should not double-probe.
    pub fn begin_xid_probe(&mut self, local: Callsign, peer: Callsign) -> Vec<Vec<u8>> {
        let Some(i) = self.ensure_slot(peer, local) else {
            return Vec::new();
        };
        let slot = self.slots[i]
            .as_mut()
            .expect("slot just ensured to be present");
        slot.sink.sent.clear();
        // A mod-8 probe: the pre-SABM XID exchange only negotiates SREJ; the link
        // stays mod-8 (the SABME path negotiates XID post-UA instead). Force mod-8
        // before deriving the offer so it advertises modulo128 = false.
        slot.session.context.is_extended = false;
        let offered = super::mdl::begin_pre_connect_xid(&mut slot.session.context);
        let info = info_field::encode(&offered);
        // XID is a U-frame (1 octet in both modulos); keep the sink mod-8 regardless.
        slot.sink.extended = false;
        let bytes = slot.sink.encode_spec(&super::signal::FrameSpec::Xid {
            is_command: true, // an initiator XID *command* (our offer)
            pf: true,
            info,
        });
        slot.sink.sent.push(bytes);
        slot.xid_probe = Some(XidProbe { offered, local });
        core::mem::take(&mut slot.sink.sent)
    }

    /// Whether an initiator pre-connect XID probe to `peer` is still awaiting its
    /// response (the bounded-wait window is open). The firmware arms a timeout while
    /// this is `true`, and clears it once [`Self::post`] resolves the probe.
    pub fn xid_probe_pending(&self, peer: &Callsign) -> bool {
        self.index_of(peer)
            .and_then(|i| self.slots[i].as_ref())
            .is_some_and(|slot| slot.xid_probe.is_some())
    }

    /// The bounded-wait expiry for a pending pre-connect XID probe to `peer`: no XID
    /// response arrived in the budget, so revert the context to go-back-N
    /// ([`super::mdl::revert_pre_connect_xid`] — never SREJ unilaterally) and proceed
    /// to the deferred plain mod-8 SABM. Mirrors the `if (!confirmed)` fallback of C#
    /// `NegotiateSrejBeforeConnectAsync` composed with the subsequent
    /// `DL-CONNECT-request`. Returns the SABM frame(s); a no-op (empty) if no probe is
    /// pending for `peer`. After this resolves the caller records the no-response
    /// outcome (`dialed_pre_connect_xid = true`, `observed_srej_enabled = false`) into
    /// the capability cache, so the peer is learned a non-answerer.
    pub fn xid_probe_timeout(
        &mut self,
        peer: Callsign,
        timers: &mut dyn TimerService,
    ) -> Vec<Vec<u8>> {
        let Some(i) = self.index_of(&peer) else {
            return Vec::new();
        };
        let slot = self.slots[i]
            .as_mut()
            .expect("index_of returned Some");
        let Some(probe) = slot.xid_probe.take() else {
            return Vec::new(); // no probe pending — nothing to time out
        };
        super::mdl::revert_pre_connect_xid(&mut slot.session.context);
        let local = probe.local;
        // The slot borrow ends here; proceed to the deferred SABM.
        self.post_with_local(local, peer, Event::DlConnectRequest, timers)
    }

    /// Drain the DL signals a peer's session has raised upward since the last call
    /// (for the console / app to consume). Empty if the peer has no slot.
    pub fn take_upward(&mut self, peer: &Callsign) -> Vec<super::signal::DataLinkSignal> {
        match self.index_of(peer).and_then(|i| self.slots[i].as_mut()) {
            Some(slot) => core::mem::take(&mut slot.sink.upward),
            None => Vec::new(),
        }
    }

    /// Free `peer`'s slot if its session has fully disconnected (state back to
    /// `Disconnected`, nothing queued), reclaiming its capacity. Call after
    /// draining [`Self::take_upward`]; a no-op otherwise. Returns whether the
    /// slot was freed.
    pub fn reap(&mut self, peer: &Callsign) -> bool {
        if let Some(i) = self.index_of(peer) {
            if let Some(slot) = &self.slots[i] {
                if slot.session.state == super::session::State::Disconnected
                    && slot.session.context.i_frame_queue.is_empty()
                {
                    self.slots[i] = None;
                    return true;
                }
            }
        }
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sdl::{DataLinkSignal, Event, FrameInfo, MockTimerService, State};

    fn call(s: &str) -> Callsign {
        Callsign::parse(s).unwrap()
    }

    fn sabm() -> Event {
        Event::SabmReceived(FrameInfo {
            poll_final: true,
            is_command: true,
            ..Default::default()
        })
    }

    #[test]
    fn first_contact_creates_a_slot_and_replies() {
        let mut mgr: SessionManager<4> = SessionManager::new(call("M0LTE-1"));
        let mut t = MockTimerService::new();

        let out = mgr.post(call("G7XYZ"), sabm(), &mut t);
        assert_eq!(mgr.active(), 1);
        assert_eq!(out.len(), 1); // UA on the wire
        assert_eq!(
            mgr.session_for(&call("G7XYZ")).map(|s| s.state),
            Some(State::Connected)
        );
        assert!(mgr
            .take_upward(&call("G7XYZ"))
            .contains(&DataLinkSignal::ConnectIndication));
    }

    #[test]
    fn distinct_peers_get_distinct_sessions() {
        let mut mgr: SessionManager<4> = SessionManager::new(call("M0LTE-1"));
        let mut t = MockTimerService::new();

        mgr.post(call("G7AAA"), sabm(), &mut t);
        mgr.post(call("G7BBB"), sabm(), &mut t);
        assert_eq!(mgr.active(), 2);
        assert_eq!(
            mgr.session_for(&call("G7AAA")).map(|s| s.state),
            Some(State::Connected)
        );
        assert_eq!(
            mgr.session_for(&call("G7BBB")).map(|s| s.state),
            Some(State::Connected)
        );
    }

    #[test]
    fn full_manager_refuses_unknown_peer() {
        let mut mgr: SessionManager<1> = SessionManager::new(call("M0LTE-1"));
        let mut t = MockTimerService::new();

        mgr.post(call("G7AAA"), sabm(), &mut t); // fills the single slot
        let out = mgr.post(call("G7BBB"), sabm(), &mut t); // refused
        assert!(out.is_empty());
        assert_eq!(mgr.active(), 1);
        assert!(mgr.session_for(&call("G7BBB")).is_none());
    }

    #[test]
    fn disconnect_frees_the_slot() {
        let mut mgr: SessionManager<2> = SessionManager::new(call("M0LTE-1"));
        let mut t = MockTimerService::new();

        mgr.post(call("G7AAA"), sabm(), &mut t);
        assert_eq!(mgr.active(), 1);

        // Inbound DISC ⇒ session returns to Disconnected. The slot is kept
        // until the upward signals are drained + the caller reaps — freeing
        // inside post() would lose the DisconnectIndication.
        let disc = Event::DiscReceived(FrameInfo {
            poll_final: true,
            is_command: true,
            ..Default::default()
        });
        mgr.post(call("G7AAA"), disc, &mut t);
        assert_eq!(mgr.active(), 1);
        let ups = mgr.take_upward(&call("G7AAA"));
        assert!(ups.contains(&DataLinkSignal::DisconnectIndication));
        assert!(mgr.reap(&call("G7AAA")));
        assert_eq!(mgr.active(), 0);

        // Reaping a live session is a no-op.
        mgr.post(call("G7AAA"), sabm(), &mut t);
        assert!(!mgr.reap(&call("G7AAA")));
        assert_eq!(mgr.active(), 1);
    }

    /// Decode `bytes` and return the classified event kind (mod-8 — the connect
    /// handshake is all U-frames, 1 octet in both modulos).
    fn classify(bytes: &[u8]) -> Event {
        use crate::ax25::Frame;
        use crate::sdl::bridge::classify_incoming;
        classify_incoming(&Frame::decode(bytes).expect("emitted frame decodes"))
            .expect("classifies")
    }

    #[test]
    fn connect_extended_true_dials_sabme_and_routes_to_v22() {
        let mut mgr: SessionManager<2> = SessionManager::new(call("M0LTE-1"));
        let mut t = MockTimerService::new();
        let peer = call("G7XYZ");

        let out = mgr.connect_extended(call("M0LTE-1"), peer, true, &mut t);
        assert_eq!(out.len(), 1, "one SABME on the wire");
        assert!(matches!(classify(&out[0]), Event::SabmeReceived(_)));
        let s = mgr.session_for(&peer).unwrap();
        assert_eq!(s.state, State::AwaitingV22Connection);
        assert!(s.context.is_extended, "mod-128 preference set on the session");
    }

    #[test]
    fn connect_extended_false_dials_sabm_mod8() {
        let mut mgr: SessionManager<2> = SessionManager::new(call("M0LTE-1"));
        let mut t = MockTimerService::new();
        let peer = call("G7XYZ");

        let out = mgr.connect_extended(call("M0LTE-1"), peer, false, &mut t);
        assert_eq!(out.len(), 1, "one SABM on the wire");
        assert!(matches!(classify(&out[0]), Event::SabmReceived(_)));
        let s = mgr.session_for(&peer).unwrap();
        assert_eq!(s.state, State::AwaitingConnection);
        assert!(!s.context.is_extended);
    }

    #[test]
    fn plain_connect_honours_prefer_extended_default() {
        let mut t = MockTimerService::new();
        let peer = call("G7XYZ");

        // Default (true, matching C# PreferExtendedConnect) ⇒ mod-128 SABME.
        let mut mgr_default: SessionManager<2> = SessionManager::new(call("M0LTE-1"));
        assert!(mgr_default.prefer_extended_connect());
        let out = mgr_default.connect(peer, &mut t);
        assert!(matches!(classify(&out[0]), Event::SabmeReceived(_)));

        // Opt out ⇒ mod-8 SABM.
        let mut mgr_m8: SessionManager<2> =
            SessionManager::new(call("M0LTE-1")).with_prefer_extended_connect(false);
        assert!(!mgr_m8.prefer_extended_connect());
        let out = mgr_m8.connect(peer, &mut t);
        assert!(matches!(classify(&out[0]), Event::SabmReceived(_)));
    }

    /// The safety net that makes the SABME-first default safe: a plain
    /// (default-preference) connect to a peer that refuses SABME with **DM** must
    /// degrade to a mod-8 SABM re-establishment (#48 DM-degrade), not strand the
    /// connect in Disconnected. This is the DM analogue of the FRMR-degrade test,
    /// and the reason the default could be flipped to true.
    #[test]
    fn default_extended_connect_degrades_to_mod8_sabm_on_dm_refusal() {
        let mut mgr: SessionManager<2> = SessionManager::new(call("M0LTE-1"));
        let mut t = MockTimerService::new();
        let peer = call("G7XYZ");

        // Plain connect uses the new default (SABME-first).
        let out = mgr.connect(peer, &mut t);
        assert!(matches!(classify(&out[0]), Event::SabmeReceived(_)));
        assert!(mgr.session_for(&peer).unwrap().context.is_extended);
        assert_eq!(
            mgr.session_for(&peer).unwrap().state,
            State::AwaitingV22Connection
        );

        // Pre-v2.2 peer (XRouter-class) refuses SABME with DM (F=1).
        let dm = Event::DmReceived(FrameInfo {
            poll_final: true,
            is_command: false,
            ..Default::default()
        });
        let out = mgr.post(peer, dm, &mut t);

        // #48: degraded to mod-8 and a SABM re-establishment emitted — NOT stranded.
        let s = mgr.session_for(&peer).unwrap();
        assert!(!s.context.is_extended, "DM degraded the link to mod-8");
        assert_eq!(s.state, State::AwaitingConnection);
        assert!(
            out.iter().any(|b| matches!(classify(b), Event::SabmReceived(_))),
            "expected a mod-8 SABM re-establishment after the DM: {out:02x?}"
        );
    }

    #[test]
    fn extended_dial_accepted_reaches_connected_mod128_over_the_wire() {
        // Full accepted path: A dials SABME, B (answerer) adopts mod-128 and replies
        // UA, A confirms Connected with is_extended set. Two managers exchanging the
        // exact wire octets — the initiator preference yields a real mod-128 link.
        let mut a: SessionManager<2> = SessionManager::new(call("M0LTE-1"));
        let mut b: SessionManager<2> = SessionManager::new(call("M0LTE-2"));
        let mut t = MockTimerService::new();
        let (ca, cb) = (call("M0LTE-1"), call("M0LTE-2"));

        // A → SABME.
        let sabme = a.connect_extended(ca, cb, true, &mut t);
        assert!(matches!(classify(&sabme[0]), Event::SabmeReceived(_)));

        // B receives SABME ⇒ adopts v2.2, replies UA, enters Connected extended.
        let from_b = b.post(ca, classify(&sabme[0]), &mut t);
        let sb = b.session_for(&ca).unwrap();
        assert_eq!(sb.state, State::Connected);
        assert!(sb.context.is_extended, "answerer adopts mod-128 from the SABME");
        assert_eq!(from_b.len(), 1);
        assert!(matches!(classify(&from_b[0]), Event::UaReceived(_)));

        // B's UA arrives at A ⇒ A confirms Connected, still mod-128.
        let _ = a.post(cb, classify(&from_b[0]), &mut t);
        let sa = a.session_for(&cb).unwrap();
        assert_eq!(sa.state, State::Connected);
        assert!(sa.context.is_extended, "initiator link is mod-128");
    }

    #[test]
    fn extended_dial_degrades_to_mod8_sabm_on_frmr() {
        // The v2.2-preferred connect's fallback leg: an extended dial that a
        // pre-v2.2 peer refuses with FRMR degrades to a mod-8 SABM re-establishment
        // (#45 forces version 2.0 before Establish_Data_Link re-runs). This path is
        // only reachable because the initiator preference set is_extended = true.
        let mut mgr: SessionManager<2> = SessionManager::new(call("M0LTE-1"));
        let mut t = MockTimerService::new();
        let peer = call("G7XYZ");

        let out = mgr.connect_extended(call("M0LTE-1"), peer, true, &mut t);
        assert!(matches!(classify(&out[0]), Event::SabmeReceived(_)));
        assert!(mgr.session_for(&peer).unwrap().context.is_extended);

        // Peer refuses SABME with FRMR (final).
        let frmr = Event::FrmrReceived(FrameInfo {
            poll_final: true,
            is_command: false,
            ..Default::default()
        });
        let out = mgr.post(peer, frmr, &mut t);

        // Degraded: version forced to 2.0, and a mod-8 SABM re-establishment emitted.
        let s = mgr.session_for(&peer).unwrap();
        assert!(!s.context.is_extended, "FRMR degraded the link to mod-8");
        assert_eq!(s.state, State::AwaitingConnection);
        let re_sabm = out
            .iter()
            .any(|b| matches!(classify(b), Event::SabmReceived(_)));
        assert!(re_sabm, "expected a mod-8 SABM re-establishment: {out:02x?}");
    }

    /// The relay regression: an I-frame received while we have nothing to send
    /// back must still be acknowledged (RR) — via the immediate LM-SEIZE grant
    /// driving the figc4 AckPending path. Found live against LinBPQ: without
    /// the grant the ack never goes out and the peer declares link failure.
    #[test]
    fn idle_received_i_frame_is_still_acknowledged() {
        use crate::ax25::Frame;
        use crate::sdl::bridge::classify_incoming;

        let mut mgr: SessionManager<2> = SessionManager::new(call("M0LTE-1"));
        let mut t = MockTimerService::new();
        let peer = call("M0LTE-9");

        // Bring the session up (inbound SABM ⇒ UA out).
        let out = mgr.post(peer, sabm(), &mut t);
        assert_eq!(out.len(), 1);

        // Peer sends an I-frame (N(S)=0, no P); we have no reply data queued.
        let i_frame = Event::IReceived(FrameInfo {
            ns: 0,
            nr: 0,
            pid: Some(crate::ax25::PID_NO_LAYER3),
            info: alloc::vec![0x42],
            is_command: true,
            ..Default::default()
        });
        let out = mgr.post(peer, i_frame, &mut t);

        // Among the emitted frames there must be an RR acknowledging N(R)=1.
        let acked = out.iter().any(|bytes| {
            let frame = Frame::decode(bytes).expect("emitted frame decodes");
            matches!(
                classify_incoming(&frame),
                Some(Event::RrReceived(f)) if f.nr == 1
            )
        });
        assert!(acked, "received I-frame was not acknowledged: {out:02x?}");
    }

    // ─── Carrier-sense (CSMA) seam ──────────────────────────────────────────

    /// A carrier whose busy state is fixed at construction — the seam's test double.
    #[derive(Debug, Clone, Copy)]
    struct TestCarrier(Option<bool>);
    impl crate::sdl::carrier::CarrierSense for TestCarrier {
        fn channel_busy(&self) -> Option<bool> {
            self.0
        }
    }

    /// Bring `peer` up (SABM⇒UA) then feed it an in-sequence I-frame with no reply
    /// data queued — the scenario whose delayed RR ack rides the LM-SEIZE grant.
    fn connect_then_receive_i_frame(
        mgr: &mut SessionManager<2>,
        peer: Callsign,
        t: &mut MockTimerService,
    ) -> Vec<Vec<u8>> {
        mgr.post(peer, sabm(), t);
        let i_frame = Event::IReceived(FrameInfo {
            ns: 0,
            nr: 0,
            pid: Some(crate::ax25::PID_NO_LAYER3),
            info: alloc::vec![0x42],
            is_command: true,
            ..Default::default()
        });
        mgr.post(peer, i_frame, t)
    }

    fn emitted_rr(out: &[Vec<u8>]) -> bool {
        out.iter().any(|b| matches!(classify(b), Event::RrReceived(_)))
    }

    #[test]
    fn busy_carrier_defers_the_seize_and_the_ack() {
        let mut mgr: SessionManager<2> = SessionManager::new(call("M0LTE-1"));
        mgr.set_carrier_sense(Box::new(TestCarrier(Some(true)))); // channel busy
        let mut t = MockTimerService::new();
        let peer = call("M0LTE-9");

        let out = connect_then_receive_i_frame(&mut mgr, peer, &mut t);

        // Busy ⇒ the seize (and the RR ack it drives) is deferred, not granted.
        assert!(!emitted_rr(&out), "busy carrier must defer the ack: {out:02x?}");
        assert!(
            mgr.seize_pending(&peer),
            "the seize stays pending while the channel is busy"
        );
    }

    #[test]
    fn clear_carrier_grants_the_seize() {
        let mut mgr: SessionManager<2> = SessionManager::new(call("M0LTE-1"));
        mgr.set_carrier_sense(Box::new(TestCarrier(Some(false)))); // channel idle
        let mut t = MockTimerService::new();
        let peer = call("M0LTE-9");

        let out = connect_then_receive_i_frame(&mut mgr, peer, &mut t);

        // Clear ⇒ the seize is granted and the RR ack goes out; nothing left pending.
        assert!(emitted_rr(&out), "clear carrier must grant the ack: {out:02x?}");
        assert!(!mgr.seize_pending(&peer), "no seize left pending once granted");
    }

    #[test]
    fn unknown_carrier_fails_open_like_no_source() {
        // Unknown state (None) must fail open — behave like the default no-source
        // manager, granting the seize.
        let mut mgr: SessionManager<2> = SessionManager::new(call("M0LTE-1"));
        mgr.set_carrier_sense(Box::new(TestCarrier(None)));
        let mut t = MockTimerService::new();
        let peer = call("M0LTE-9");

        let out = connect_then_receive_i_frame(&mut mgr, peer, &mut t);
        assert!(emitted_rr(&out), "unknown carrier fails open (grants): {out:02x?}");
        assert!(!mgr.seize_pending(&peer));
    }

    #[test]
    fn deferred_seize_is_granted_once_the_channel_clears() {
        // A deferral must resume, not drop: after a busy defer, clearing the channel
        // and driving the session again grants the pending seize (the RR ack goes out).
        let mut mgr: SessionManager<2> = SessionManager::new(call("M0LTE-1"));
        mgr.set_carrier_sense(Box::new(TestCarrier(Some(true))));
        let mut t = MockTimerService::new();
        let peer = call("M0LTE-9");

        let out = connect_then_receive_i_frame(&mut mgr, peer, &mut t);
        assert!(!emitted_rr(&out));
        assert!(mgr.seize_pending(&peer));

        // Channel clears; a T2 expiry re-drives the session, and the still-pending
        // seize is now granted ⇒ the delayed RR is emitted.
        mgr.set_carrier_sense(Box::new(TestCarrier(Some(false))));
        let out = mgr.post(peer, Event::T2Expiry, &mut t);
        assert!(
            emitted_rr(&out),
            "the deferred seize resumes when the channel clears: {out:02x?}"
        );
        assert!(!mgr.seize_pending(&peer));
    }

    // ─── Pre-session XID responder (mirrors Ax25ListenerPreSessionXidTests) ──

    /// A mod-8 XID command offering SREJ (what a PDN interlink initiator sends
    /// before its SABM), as a classified inbound event.
    fn mod8_srej_xid_command() -> Event {
        use crate::ax25::xid::{info_field, HdlcOptionalFunctions, RejectMode, XidParameters};
        let info = info_field::encode(&XidParameters {
            hdlc_optional_functions: Some(HdlcOptionalFunctions {
                reject: RejectMode::SelectiveReject,
                modulo128: false,
                srej_multiframe: true,
                segmenter_reassembler: false,
            }),
            ..Default::default()
        });
        Event::XidReceived(FrameInfo {
            poll_final: true,
            is_command: true,
            info,
            ..Default::default()
        })
    }

    /// A pre-session XID command from an unknown peer is answered with an XID
    /// *response* (F=1) that advertises SREJ — NOT a DM, and NOT a connection.
    #[test]
    fn pre_session_xid_command_for_unknown_peer_is_answered_with_xid_response() {
        use crate::ax25::xid::info_field;
        use crate::ax25::Frame;
        use crate::sdl::bridge::classify_incoming;

        let mut mgr: SessionManager<2> = SessionManager::new(call("M0LTE"));
        let mut t = MockTimerService::new();
        let peer = call("G7XYZ");

        let out = mgr.post(peer, mod8_srej_xid_command(), &mut t);
        assert_eq!(out.len(), 1, "exactly one XID response on the wire");

        let reply = Frame::decode(&out[0]).expect("XID reply decodes");
        assert!(reply.is_response(), "the answer is an XID *response*");
        assert!(reply.poll_final(), "F=1 so the initiator's F_eq_1 diamond fires");
        match classify_incoming(&reply) {
            Some(Event::XidReceived(_)) => {}
            other => panic!("expected an XID reply, got {other:?} (must not be a DM)"),
        }
        // The response advertises SREJ (both sides offered it).
        let p = info_field::parse(&reply.info).expect("response info parses");
        assert_eq!(
            p.hdlc_optional_functions.unwrap().reject,
            crate::ax25::xid::RejectMode::SelectiveReject
        );

        // Answering an XID command is NOT a connection: the session stays
        // Disconnected and no ConnectIndication was raised.
        assert_eq!(
            mgr.session_for(&peer).map(|s| s.state),
            Some(State::Disconnected)
        );
        assert!(!mgr
            .take_upward(&peer)
            .contains(&DataLinkSignal::ConnectIndication));
    }

    /// The SABM that follows the pre-session XID brings the session to Connected
    /// with the XID-negotiated SREJ adopted (the staged SrejEnabled survives the
    /// SABM's Set Version 2.0, which clears only is_extended).
    #[test]
    fn sabm_after_pre_session_xid_reaches_connected_with_srej_adopted() {
        use crate::ax25::Frame;
        use crate::sdl::bridge::classify_incoming;

        let mut mgr: SessionManager<2> = SessionManager::new(call("M0LTE"));
        let mut t = MockTimerService::new();
        let peer = call("G7XYZ");

        // 1) Pre-session XID → XID response; still Disconnected.
        let xid_out = mgr.post(peer, mod8_srej_xid_command(), &mut t);
        assert_eq!(xid_out.len(), 1);
        assert!(matches!(
            classify_incoming(&Frame::decode(&xid_out[0]).unwrap()),
            Some(Event::XidReceived(_))
        ));
        assert_eq!(
            mgr.session_for(&peer).map(|s| s.state),
            Some(State::Disconnected)
        );

        // 2) The peer now sends SABM → the link establishes, adopting SREJ.
        let sabm = Event::SabmReceived(FrameInfo {
            poll_final: true,
            is_command: true,
            ..Default::default()
        });
        let ua_out = mgr.post(peer, sabm, &mut t);

        let s = mgr.session_for(&peer).expect("session exists");
        assert_eq!(s.state, State::Connected, "the SABM establishes the link");
        assert!(
            s.context.srej_enabled,
            "the XID-negotiated SREJ survives into the established session"
        );
        assert!(!s.context.implicit_reject);
        // The SABM is answered with a UA (not a DM).
        assert!(
            ua_out
                .iter()
                .any(|b| matches!(classify_incoming(&Frame::decode(b).unwrap()), Some(Event::UaReceived(_)))),
            "the SABM must be acknowledged with a UA: {ua_out:02x?}"
        );
        assert!(mgr
            .take_upward(&peer)
            .contains(&DataLinkSignal::ConnectIndication));
    }

    /// A `connect_planned` dial honours the capability plan's extended choice:
    /// an extended plan dials SABME, a mod-8 *no-probe* plan dials SABM straight.
    /// (The `pre_connect_xid` probe leg is covered by the initiator-probe tests below.)
    #[test]
    fn connect_planned_honours_the_dial_plan_extended_choice() {
        use crate::sdl::capability::PeerDialPlan;

        let peer = call("G7XYZ");
        let local = call("M0LTE-1");

        let mut ext: SessionManager<2> = SessionManager::new(local);
        let out = ext.connect_planned(
            local,
            peer,
            PeerDialPlan {
                extended: true,
                pre_connect_xid: false,
            },
            &mut MockTimerService::new(),
        );
        assert!(matches!(classify(&out[0]), Event::SabmeReceived(_)));

        let mut m8: SessionManager<2> = SessionManager::new(local);
        let out = m8.connect_planned(
            local,
            peer,
            PeerDialPlan {
                extended: false,
                pre_connect_xid: false, // fresh cache hit / known non-answerer: dial straight
            },
            &mut MockTimerService::new(),
        );
        assert!(matches!(classify(&out[0]), Event::SabmReceived(_)));
        assert!(!m8.xid_probe_pending(&peer), "no probe on a dial-straight plan");
    }

    // ─── Initiator pre-connect XID probe (mirrors NegotiateSrejBeforeConnectAsync) ─

    const PROBE_PORT: u8 = 0;
    const PROBE_T0: u64 = 1_000_000;

    /// An XID *response* event offering mod-8 SREJ — what a BPQ-class peer answers a
    /// pre-connect probe with (the mutual-SREJ path).
    fn xid_response_offering_srej() -> Event {
        use crate::ax25::xid::{info_field, HdlcOptionalFunctions, RejectMode, XidParameters};
        let info = info_field::encode(&XidParameters {
            hdlc_optional_functions: Some(HdlcOptionalFunctions {
                reject: RejectMode::SelectiveReject,
                modulo128: false,
                srej_multiframe: true,
                segmenter_reassembler: false,
            }),
            ..Default::default()
        });
        Event::XidReceived(FrameInfo {
            poll_final: true,
            is_command: false, // a *response*, not a command
            info,
            ..Default::default()
        })
    }

    fn ua() -> Event {
        Event::UaReceived(FrameInfo {
            poll_final: true,
            is_command: false,
            ..Default::default()
        })
    }

    /// (a) probe-out → inject XID response → merged params → the deferred connect
    /// uses the negotiated mod-8/SREJ; (d) the negotiated SREJ then takes effect on
    /// the established link (out-of-sequence I-frame ⇒ SREJ, not REJ). Full loop
    /// through the capability cache: plan_dial → connect_planned → response → connect
    /// → record_outcome learns the peer answers XID with SREJ.
    #[test]
    fn initiator_probe_response_negotiates_srej_connects_and_srej_activates() {
        use crate::ax25::Frame;
        use crate::sdl::capability::{PeerCapabilityCache, PeerDialPolicy};

        let local = call("M0LTE-1");
        let peer = call("G7XYZ");
        let mut mgr: SessionManager<2> = SessionManager::new(local);
        let mut t = MockTimerService::new();
        let mut cache: PeerCapabilityCache<4> = PeerCapabilityCache::new();

        // Cache miss + interlink ⇒ the plan probes (mod-8, pre_connect_xid).
        let plan = cache.plan_dial(PROBE_PORT, &peer, PeerDialPolicy::Interlink, PROBE_T0);
        assert!(!plan.extended);
        assert!(plan.pre_connect_xid);

        // connect_planned begins the probe: an XID *command* on the wire, NO SABM yet.
        let out = mgr.connect_planned(local, peer, plan, &mut t);
        assert_eq!(out.len(), 1, "one XID command on the wire");
        let cmd = Frame::decode(&out[0]).expect("XID command decodes");
        assert!(cmd.is_command(), "an initiator XID *command*");
        assert!(matches!(classify(&out[0]), Event::XidReceived(_)));
        assert!(mgr.xid_probe_pending(&peer), "probe pending until the response");
        assert_eq!(
            mgr.session_for(&peer).map(|s| s.state),
            Some(State::Disconnected),
            "no SABM yet — the connect is deferred behind the probe"
        );

        // The peer answers with an XID *response* offering SREJ ⇒ merge + deferred SABM.
        let out = mgr.post(peer, xid_response_offering_srej(), &mut t);
        assert!(
            out.iter().any(|b| matches!(classify(b), Event::SabmReceived(_))),
            "the deferred SABM fires on the XID response: {out:02x?}"
        );
        assert!(!mgr.xid_probe_pending(&peer), "probe resolved");
        let s = mgr.session_for(&peer).unwrap();
        assert_eq!(s.state, State::AwaitingConnection);
        assert!(!s.context.is_extended, "a mod-8 probe never flips to mod-128");
        assert!(s.context.srej_enabled, "both offered SREJ ⇒ negotiated on");

        // The peer accepts (UA) ⇒ Connected, the SREJ carried into the link.
        let _ = mgr.post(peer, ua(), &mut t);
        let s = mgr.session_for(&peer).unwrap();
        assert_eq!(s.state, State::Connected);
        assert!(s.context.srej_enabled, "negotiated SREJ survives establishment");
        let (obs_ext, obs_srej) = (s.context.is_extended, s.context.srej_enabled);

        // record_outcome ⇒ the cache learns the peer answers XID with SREJ.
        cache.record_outcome(
            PROBE_PORT, peer, plan.extended, obs_ext, plan.pre_connect_xid, obs_srej, PROBE_T0,
        );
        assert_eq!(
            cache.lookup(PROBE_PORT, &peer).unwrap().supports_srej_via_xid,
            Some(true)
        );

        // (d) the negotiated SREJ actually takes effect: an out-of-sequence I-frame
        // (expecting N(S)=0, receiving N(S)=1) provokes SREJ for the gap — not REJ.
        let oos = Event::IReceived(FrameInfo {
            ns: 1,
            nr: 0,
            pid: Some(crate::ax25::PID_NO_LAYER3),
            info: alloc::vec![0x42],
            is_command: true,
            ..Default::default()
        });
        let out = mgr.post(peer, oos, &mut t);
        assert!(
            out.iter().any(|b| matches!(classify(b), Event::SrejReceived(_))),
            "the negotiated SREJ must emit an SREJ on the wire: {out:02x?}"
        );
        assert!(
            !out.iter().any(|b| matches!(classify(b), Event::RejReceived(_))),
            "a negotiated-SREJ link must not fall back to REJ: {out:02x?}"
        );
    }

    /// (b) probe-out → timeout → correct fallback: revert to go-back-N + a plain
    /// mod-8 SABM, and the cache records the no-response outcome so the next dial
    /// skips the probe (a known non-answerer).
    #[test]
    fn initiator_probe_timeout_falls_back_to_go_back_n_and_cache_learns() {
        use crate::ax25::Frame;
        use crate::sdl::capability::{PeerCapabilityCache, PeerDialPolicy};

        let local = call("M0LTE-1");
        let peer = call("G7XYZ");
        let mut mgr: SessionManager<2> = SessionManager::new(local);
        let mut t = MockTimerService::new();
        let mut cache: PeerCapabilityCache<4> = PeerCapabilityCache::new();

        let plan = cache.plan_dial(PROBE_PORT, &peer, PeerDialPolicy::Interlink, PROBE_T0);
        let out = mgr.connect_planned(local, peer, plan, &mut t);
        assert!(Frame::decode(&out[0]).unwrap().is_command(), "XID command out");
        assert!(mgr.xid_probe_pending(&peer));
        // The context is optimistically SREJ-seeded while the probe is open.
        assert!(mgr.session_for(&peer).unwrap().context.srej_enabled);

        // No response in the budget ⇒ timeout: revert to go-back-N + the deferred SABM.
        let out = mgr.xid_probe_timeout(peer, &mut t);
        assert!(
            out.iter().any(|b| matches!(classify(b), Event::SabmReceived(_))),
            "the fallback SABM fires on timeout: {out:02x?}"
        );
        assert!(!mgr.xid_probe_pending(&peer), "probe cleared on timeout");
        let s = mgr.session_for(&peer).unwrap();
        assert_eq!(s.state, State::AwaitingConnection);
        assert!(!s.context.srej_enabled, "silent peer ⇒ reverted to go-back-N");
        assert!(s.context.implicit_reject);
        let (obs_ext, obs_srej) = (s.context.is_extended, s.context.srej_enabled);

        // record_outcome ⇒ the cache learns the peer is a non-answerer.
        cache.record_outcome(
            PROBE_PORT, peer, plan.extended, obs_ext, plan.pre_connect_xid, obs_srej, PROBE_T0,
        );
        assert_eq!(
            cache.lookup(PROBE_PORT, &peer).unwrap().supports_srej_via_xid,
            Some(false)
        );
        let next = cache.plan_dial(PROBE_PORT, &peer, PeerDialPolicy::Interlink, PROBE_T0);
        assert!(!next.pre_connect_xid, "known non-answerer ⇒ skip the probe next time");

        // Timing out with no probe pending is a no-op.
        assert!(mgr.xid_probe_timeout(peer, &mut t).is_empty());
    }

    /// (c) a fresh cache hit ⇒ no probe: dial straight with the cached capabilities.
    /// A learned non-answerer dials a plain SABM; a learned extended peer dials SABME.
    #[test]
    fn fresh_cache_hit_skips_the_probe_and_dials_straight() {
        use crate::sdl::capability::{PeerCapabilityCache, PeerDialPolicy};

        let local = call("M0LTE-1");
        let peer = call("G7XYZ");

        // Fresh negative (probed XID, no SREJ) ⇒ plan skips the probe.
        let mut cache: PeerCapabilityCache<4> = PeerCapabilityCache::new();
        cache.record_outcome(PROBE_PORT, peer, false, false, true, false, PROBE_T0);
        let plan = cache.plan_dial(PROBE_PORT, &peer, PeerDialPolicy::Interlink, PROBE_T0);
        assert!(!plan.pre_connect_xid, "fresh non-answerer ⇒ no probe");

        let mut mgr: SessionManager<2> = SessionManager::new(local);
        let out = mgr.connect_planned(local, peer, plan, &mut MockTimerService::new());
        assert!(
            matches!(classify(&out[0]), Event::SabmReceived(_)),
            "dials a plain SABM straight, no XID command"
        );
        assert!(!mgr.xid_probe_pending(&peer), "no probe on a fresh cache hit");

        // Fresh extended positive ⇒ dials SABME straight (still no probe).
        let mut cache2: PeerCapabilityCache<4> = PeerCapabilityCache::new();
        cache2.record_outcome(PROBE_PORT, peer, true, true, false, false, PROBE_T0);
        let plan2 = cache2.plan_dial(PROBE_PORT, &peer, PeerDialPolicy::UserConnect, PROBE_T0);
        assert!(plan2.extended);
        assert!(!plan2.pre_connect_xid);
        let mut mgr2: SessionManager<2> = SessionManager::new(local);
        let out = mgr2.connect_planned(local, peer, plan2, &mut MockTimerService::new());
        assert!(
            matches!(classify(&out[0]), Event::SabmeReceived(_)),
            "dials SABME straight from a learned-extended cache hit"
        );
        assert!(!mgr2.xid_probe_pending(&peer));
    }
}
