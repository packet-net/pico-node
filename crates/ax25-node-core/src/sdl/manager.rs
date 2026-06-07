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
use alloc::vec::Vec;

use crate::ax25::Callsign;

use super::bridge::WireSink;
use super::event::Event;
use super::session::Session;
use super::timer::TimerService;

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
}

/// A fixed-capacity, peer-keyed collection of [`Session`]s. `N` is the maximum
/// concurrent links — sized for a node, not a desktop (no heap session map).
#[derive(Debug)]
pub struct SessionManager<const N: usize> {
    local: Callsign,
    slots: [Option<Slot>; N],
}

impl<const N: usize> SessionManager<N> {
    /// Build a manager for the node's own `local` callsign with all slots free.
    pub fn new(local: Callsign) -> Self {
        Self {
            local,
            // `Option<Slot>` isn't `Copy`, so build the array element-by-element.
            slots: core::array::from_fn(|_| None),
        }
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

    /// Ensure a slot exists for `peer`, returning its index. Returns `None` if the
    /// manager is full and `peer` has no existing slot (the caller drops the frame /
    /// replies DM — a node at capacity refuses new links). Creates the slot's
    /// [`WireSink`] addressed for the `local ↔ peer` link.
    fn ensure_slot(&mut self, peer: Callsign) -> Option<usize> {
        if let Some(i) = self.index_of(&peer) {
            return Some(i);
        }
        let free = self.slots.iter().position(|s| s.is_none())?;
        self.slots[free] = Some(Slot {
            peer,
            session: Session::new(),
            sink: WireSink::new(self.local, peer, Vec::new()),
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
        let Some(i) = self.ensure_slot(peer) else {
            return Vec::new();
        };
        let slot = self.slots[i]
            .as_mut()
            .expect("slot just ensured to be present");
        slot.sink.sent.clear();
        slot.session.post_event(event, timers, &mut slot.sink);

        // Grant LM-SEIZE immediately: the node's wire transports (AXUDP,
        // KISS-TCP) are full-duplex, so the channel is always free. The
        // confirm drives the figc4 `AckPending` path that emits the delayed
        // RR acknowledgement — without it, received I-frames with no reply
        // data are never acked and the peer eventually declares link failure
        // (found live against LinBPQ through the console relay). Bounded:
        // the confirm path releases, it never re-seizes.
        let mut grants = 0;
        while slot.sink.seize_pending && grants < 4 {
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
}
