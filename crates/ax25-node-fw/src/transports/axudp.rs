//! Capability 1 — AXUDP: AX.25-over-UDP for node↔node connectivity over WiFi.
//!
//! Ports `Packet.Axudp.AxudpSocket` onto `embassy_net::udp::UdpSocket` (the 1:1
//! mapping the research note identifies). The UDP payload is the AX.25 frame
//! body + the mandatory trailing FCS; framing comes from
//! [`ax25_node_core::axudp`].
//!
//! Beyond the socket loop + read-only NET/ROM tap, this task owns **the
//! connected-mode session layer for the AXUDP port**. Each connected peer
//! carries a [`Role`] deciding where its DL signals go:
//!
//! - [`Role::Console`] — an inbound user at the node prompt
//!   (`TransportKind::Ax25`, CR line discipline).
//! - [`Role::Bridge`] — piped to *another AX.25 session*: a console user typed
//!   `C <call>` and this task connected onward, relaying I-frame data both
//!   ways (node-hopping *through* the Pico).
//! - [`Role::TelnetRelay`] — piped to the telnet console relay
//!   ([`super::relay`] statics).
//!
//! Cross-peer work (bridge data forwarding, bridge teardown notices) is queued
//! as [`FollowUp`]s and drained by [`drive`] — one borrow at a time, bounded.
//!
//! **Timers are live**: each peer carries its own [`session::EmbassyTimers`],
//! the main select loop wakes at the earliest armed deadline across all peers,
//! and expiries post the matching `Event::T?Expiry` into that peer's session —
//! retransmission, ack timing and dead-peer link failure (N2 exhausted →
//! teardown) run exactly as the SDL tables specify.
//!
//! Single-transport ownership (this task owns `Sessions` exclusively) keeps the
//! `&mut` story trivial; when a second connected-mode transport arrives the
//! manager moves behind the supervisor seam `session.rs` documents.

use ax25_node_core::ax25::{Callsign, PID_NO_LAYER3};
use ax25_node_core::axudp;
use ax25_node_core::console::command::parse_bytes;
use ax25_node_core::console::service::{banner_and_prompt, dispatch, Identity};
use ax25_node_core::console::{DispatchOutcome, LineAssembler, TransportKind};
use ax25_node_core::netrom::{ObserveOutcome, PortId};
use ax25_node_core::sdl::{classify_incoming, DataLinkSignal, Event};

use alloc::collections::VecDeque;
use alloc::string::String;
use alloc::vec::Vec;

use embassy_futures::select::{select, select4, Either, Either4};
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{IpEndpoint, Stack};
use embassy_time::{Duration, Instant, Ticker, Timer};

use crate::config::AxudpConfig;
use crate::session;
use crate::transports::relay::{self, RelayStatus};
use crate::transports::{call_str, parse_endpoint, ui_frame};

/// Seconds between beacon UI frames when a beacon target is configured.
const BEACON_INTERVAL_SECS: u64 = 10;

/// Where a connected peer's DL signals are routed.
enum Role {
    /// Connected, no upper attachment (e.g. an outbound link mid-handshake or
    /// one whose user has already gone away).
    None,
    /// An inbound user at the node console prompt.
    Console(LineAssembler),
    /// Piped to another AX.25 session (the other end's callsign). `initiator`
    /// marks the console-user side of the pair (the peer who typed `C`); the
    /// target side carries `initiator: false`. The distinction matters at
    /// teardown: a surviving initiator gets a notice + its console back, a
    /// surviving target just gets disconnected (its user is gone) — getting
    /// this wrong console-attaches to the REMOTE NODE, and two node consoles
    /// answering each other's prompts is a perfect I-frame echo loop (observed
    /// live: "Invalid command" ↔ "Unknown command" at 2.5 Hz until BPQ DISCed).
    Bridge { other: Callsign, initiator: bool },
    /// Piped to the telnet console relay statics.
    TelnetRelay,
}

/// Per-peer link state alongside the manager's session slot.
struct PeerState {
    peer: Callsign,
    /// Our station callsign on this link. The node call for inbound sessions
    /// and telnet-relay connects; for bridges, the console user's callsign
    /// with complemented SSID (the node cross-SSID convention — the far node
    /// must not see its own downlink callsign coming back; two simultaneous
    /// links keyed on one callsign collide in real node stacks, observed live
    /// against LinBPQ).
    local: Callsign,
    timers: session::EmbassyTimers,
    endpoint: IpEndpoint,
    role: Role,
}

/// Cross-peer work discovered while servicing one peer's signals — applied by
/// [`drive`] after that peer's borrow is released.
enum FollowUp {
    /// A console user asked to connect onward: bridge `console_peer ↔ target`.
    StartBridge {
        console_peer: Callsign,
        target: Callsign,
    },
    /// Relay I-frame data to the bridged other end.
    Forward { to: Callsign, data: Vec<u8> },
    /// A bridged link ended. The surviving side is handled by its own bridge
    /// direction: a console user gets a notice + prompt, a target gets DISC'd.
    BridgeEnded { survivor: Callsign },
}

#[embassy_executor::task]
pub async fn task(
    stack: Stack<'static>,
    cfg: AxudpConfig,
    my_call: Callsign,
    console_id: Identity,
    prompt: String,
) {
    defmt::info!("axudp: listen udp/{}", cfg.listen_port);

    let mut rx_meta = [PacketMetadata::EMPTY; 4];
    let mut tx_meta = [PacketMetadata::EMPTY; 4];
    let mut rx_buf = [0u8; 2048];
    let mut tx_buf = [0u8; 2048];
    let mut socket = UdpSocket::new(stack, &mut rx_meta, &mut rx_buf, &mut tx_meta, &mut tx_buf);
    defmt::unwrap!(socket.bind(cfg.listen_port));

    // §5: the harness endpoint comes from the build environment (LAN detail,
    // not committed). Without it the transport still listens + decodes.
    let beacon_ep: Option<IpEndpoint> = cfg.beacon_target.and_then(parse_endpoint);
    if beacon_ep.is_none() {
        defmt::info!("axudp: no AXUDP_BEACON_TARGET set — listen-only");
    }

    // The read-only NET/ROM tap (the C# FrameTraced-before-DispatchInbound
    // equivalent): fed every decoded inbound frame BEFORE address filtering.
    let mut netrom = session::new_netrom();
    let port_id = PortId::from_str_lossy("axudp");

    // The connected-mode session layer for this port + per-peer link state.
    let mut sessions = session::new_sessions(my_call);
    let mut peers: [Option<PeerState>; session::MAX_SESSIONS] =
        [const { None }; session::MAX_SESSIONS];

    // Callsign → last-heard UDP endpoint (the outbound-connect route table;
    // LinBPQ's periodic ID/NODES broadcasts keep it warm).
    let mut heard: [Option<(Callsign, IpEndpoint)>; 8] = [None; 8];

    let mut dgram_buf = [0u8; 2048];
    let mut ticker = Ticker::every(Duration::from_secs(BEACON_INTERVAL_SECS));
    let mut src_buf = [0u8; 16];
    let mut dst_buf = [0u8; 16];

    loop {
        // Wake at the earliest armed timer deadline across all peers (if any).
        let next_deadline: Option<Instant> = peers
            .iter()
            .flatten()
            .filter_map(|p| p.timers.next_deadline())
            .min();
        let timer_wait = async {
            match next_deadline {
                Some(at) => Timer::at(at).await,
                None => core::future::pending::<()>().await,
            }
        };

        // Telnet-relay arm: a pending connect request when idle; user bytes /
        // hangup while the telnet relay is active.
        let telnet_relay_active = peers
            .iter()
            .flatten()
            .any(|p| matches!(p.role, Role::TelnetRelay));
        let relay_fut = async {
            if telnet_relay_active {
                let mut buf = [0u8; 128];
                match select(relay::USER_TO_AX.read(&mut buf), relay::USER_HANGUP.wait()).await {
                    Either::First(n) => RelayEvent::UserData(buf, n),
                    Either::Second(()) => RelayEvent::Hangup,
                }
            } else {
                RelayEvent::Connect(relay::CONNECT_REQ.receive().await)
            }
        };

        match select4(
            ticker.next(),
            socket.recv_from(&mut dgram_buf),
            timer_wait,
            relay_fut,
        )
        .await
        {
            Either4::First(()) => {
                if let Some(ep) = beacon_ep {
                    let beacon = ui_frame(
                        my_call,
                        "IDENT",
                        b"pico-node AXUDP beacon (HW-BRINGUP Gate 3)",
                    );
                    let dgram = axudp::encode_datagram(&beacon);
                    match socket.send_to(&dgram, ep).await {
                        Ok(()) => defmt::info!("axudp: beacon sent ({=usize} bytes)", dgram.len()),
                        Err(e) => defmt::warn!("axudp: beacon send error {:?}", e),
                    }
                }
            }
            Either4::Second(Ok((n, meta))) => {
                let rx = axudp::decode_datagram(&dgram_buf[..n]);
                let Some(frame) = rx.frame else {
                    defmt::warn!(
                        "axudp: {=usize} bytes from {:?} rejected (fcs_valid={=bool})",
                        n,
                        meta.endpoint,
                        rx.fcs_valid
                    );
                    continue;
                };

                // READ-ONLY NET/ROM TAP — every frame, BEFORE the address filter,
                // so NODES broadcasts (addressed to "NODES", not us) are heard.
                let outcome = session::observe_inbound(&mut netrom, &frame, my_call, port_id);
                if let ObserveOutcome::Ingested { .. } = outcome {
                    defmt::info!(
                        "axudp: NODES broadcast ingested ({=u32} destinations known)",
                        netrom.destination_count() as u32
                    );
                }

                defmt::info!(
                    "axudp: rx {=str} -> {=str} ctl={=u8:#04x} info={=usize}B from {:?}",
                    call_str(&frame.source.callsign, &mut src_buf),
                    call_str(&frame.destination.callsign, &mut dst_buf),
                    frame.control,
                    frame.info.len(),
                    meta.endpoint
                );
                if frame.is_ui() && !frame.info.is_empty() {
                    if let Ok(text) = core::str::from_utf8(&frame.info) {
                        defmt::info!("axudp: rx UI text: {=str}", text);
                    }
                }

                heard_update(&mut heard, frame.source.callsign, meta.endpoint);

                // Address filter → the connected-mode session layer. A frame is
                // ours if addressed to the node call (new/inbound links) or to
                // the per-link local of an existing session (cross-SSID bridge
                // links don't use the node call).
                let dest = frame.destination.callsign;
                let for_us = dest == my_call
                    || peers
                        .iter()
                        .flatten()
                        .any(|ps| ps.peer == frame.source.callsign && ps.local == dest);
                if for_us && !frame.is_ui() {
                    let Some(event) = classify_incoming(&frame) else {
                        continue;
                    };
                    let peer = frame.source.callsign;
                    let Some(i) = peer_slot(&mut peers, peer, my_call, meta.endpoint) else {
                        defmt::warn!("axudp: peer table full, dropping session frame");
                        continue;
                    };
                    // NB: the link endpoint is PINNED at slot creation (inbound:
                    // the SABM's source; outbound: the heard-table/beacon entry)
                    // and deliberately NOT floated per-datagram — BPQ AXIP nodes
                    // can emit a link's frames from a different source socket
                    // than the one we address (observed live with two LinBPQ
                    // instances on one host: the bridge target's CTEXT arrived
                    // from its sibling's port, and floating the endpoint sent
                    // every reply to the wrong node).
                    drive(
                        &mut sessions,
                        &mut peers,
                        &socket,
                        &heard,
                        beacon_ep,
                        my_call,
                        i,
                        event,
                        &console_id,
                        &prompt,
                    )
                    .await;
                }
            }
            Either4::Second(Err(e)) => {
                defmt::warn!("axudp: recv error {:?}", e);
            }
            Either4::Third(()) => {
                // One or more peer timers hit their deadline: post the expiry
                // events into the owning sessions and flush what they emit.
                let now = Instant::now();
                for i in 0..peers.len() {
                    let Some(ps) = peers[i].as_mut() else {
                        continue;
                    };
                    let expired = ps.timers.take_expired(now);
                    for id in expired {
                        defmt::debug!("axudp: timer expiry ({=u8})", id as u8);
                        drive(
                            &mut sessions,
                            &mut peers,
                            &socket,
                            &heard,
                            beacon_ep,
                            my_call,
                            i,
                            session::expiry_event(id),
                            &console_id,
                            &prompt,
                        )
                        .await;
                        if peers[i].is_none() {
                            break; // expiry tore the session down
                        }
                    }
                }
            }
            Either4::Fourth(ev) => match ev {
                RelayEvent::Connect(target) => {
                    match start_outbound(
                        &mut peers,
                        &heard,
                        beacon_ep,
                        target,
                        my_call,
                        Role::TelnetRelay,
                    ) {
                        Ok(i) => {
                            drive(
                                &mut sessions,
                                &mut peers,
                                &socket,
                                &heard,
                                beacon_ep,
                                my_call,
                                i,
                                Event::DlConnectRequest,
                                &console_id,
                                &prompt,
                            )
                            .await;
                        }
                        Err(reason) => relay::STATUS.signal(RelayStatus::Failed(reason)),
                    }
                }
                RelayEvent::UserData(buf, n) => {
                    if let Some(i) = find_role(&peers, |r| matches!(r, Role::TelnetRelay)) {
                        drive(
                            &mut sessions,
                            &mut peers,
                            &socket,
                            &heard,
                            beacon_ep,
                            my_call,
                            i,
                            Event::DlDataRequest(PID_NO_LAYER3, buf[..n].to_vec()),
                            &console_id,
                            &prompt,
                        )
                        .await;
                    }
                }
                RelayEvent::Hangup => {
                    if let Some(i) = find_role(&peers, |r| matches!(r, Role::TelnetRelay)) {
                        peers[i].as_mut().expect("present").role = Role::None;
                        drive(
                            &mut sessions,
                            &mut peers,
                            &socket,
                            &heard,
                            beacon_ep,
                            my_call,
                            i,
                            Event::DlDisconnectRequest,
                            &console_id,
                            &prompt,
                        )
                        .await;
                    }
                }
            },
        }
    }
}

/// What the telnet-relay select-arm produced.
enum RelayEvent {
    /// The telnet console asked to connect to this callsign.
    Connect(Callsign),
    /// Telnet-user bytes for the relay peer.
    UserData([u8; 128], usize),
    /// The telnet user went away — disconnect the relay link.
    Hangup,
}

/// Drive one event into `peers[start]`'s session, then apply every cross-peer
/// [`FollowUp`] it (transitively) produces. Bounded by `guard`.
#[allow(clippy::too_many_arguments)]
async fn drive(
    sessions: &mut session::Sessions,
    peers: &mut [Option<PeerState>; session::MAX_SESSIONS],
    socket: &UdpSocket<'_>,
    heard: &[Option<(Callsign, IpEndpoint)>; 8],
    beacon_ep: Option<IpEndpoint>,
    my_call: Callsign,
    start: usize,
    event: Event,
    console_id: &Identity,
    prompt: &str,
) {
    let mut queue: VecDeque<(usize, Event)> = VecDeque::new();
    queue.push_back((start, event));
    let mut guard = 0u32;

    while let Some((i, ev)) = queue.pop_front() {
        guard += 1;
        if guard > 32 {
            defmt::warn!("axudp: drive guard tripped, dropping remaining work");
            break;
        }
        let Some(ps) = peers[i].as_mut() else {
            continue;
        };
        let (frames, followups) = post_one(sessions, ps, ev, console_id, prompt);
        let ep = ps.endpoint;
        send_all(socket, ep, frames).await;
        reap(sessions, peers, i);

        for f in followups {
            match f {
                FollowUp::StartBridge {
                    console_peer,
                    target,
                } => match start_outbound(
                    peers,
                    heard,
                    beacon_ep,
                    target,
                    // The node's own call — the convention real nodes use for
                    // outgoing links. NOT a per-user callsign: LinBPQ's AXIP
                    // misbehaves whenever a second callsign appears from one
                    // IP (a second MAP to the same address poisons its TX/RX
                    // resolution — CTEXT vanishes, streams never attach), and
                    // user-SSID variants trip its node-link heuristics. With
                    // the node call, each BPQ keeps exactly one map per peer
                    // IP and everything attaches cleanly. (Same-node loops —
                    // bridging back to the node the user came from — remain
                    // peer-limited: BPQ can't hold two L2 links under one
                    // callsign pair; a real network hops to a *different*
                    // node, which is the supported shape.)
                    my_call,
                    Role::Bridge {
                        other: console_peer,
                        initiator: false,
                    },
                ) {
                    Ok(ti) => {
                        if let Some(cp) = find_peer_mut(peers, &console_peer) {
                            cp.role = Role::Bridge {
                                other: target,
                                initiator: true,
                            };
                        }
                        queue.push_back((ti, Event::DlConnectRequest));
                    }
                    Err(reason) => {
                        if let Some(ci) = find_peer(peers, &console_peer) {
                            let mut msg = Vec::from(b"Failure: ".as_slice());
                            msg.extend_from_slice(reason.as_bytes());
                            msg.extend_from_slice(b"\r");
                            msg.extend_from_slice(prompt.as_bytes());
                            queue.push_back((ci, Event::DlDataRequest(PID_NO_LAYER3, msg)));
                        }
                    }
                },
                FollowUp::Forward { to, data } => {
                    if let Some(ti) = find_peer(peers, &to) {
                        queue.push_back((ti, Event::DlDataRequest(PID_NO_LAYER3, data)));
                    }
                }
                FollowUp::BridgeEnded { survivor } => {
                    if let Some(si) = find_peer(peers, &survivor) {
                        let sp = peers[si].as_mut().expect("present");
                        match sp.role {
                            Role::Bridge {
                                initiator: true, ..
                            } => {
                                // The console user survives: notice + prompt back.
                                sp.role = Role::Console(LineAssembler::default());
                                let mut msg = Vec::from(b"*** Disconnected\r".as_slice());
                                msg.extend_from_slice(prompt.as_bytes());
                                queue.push_back((si, Event::DlDataRequest(PID_NO_LAYER3, msg)));
                            }
                            _ => {
                                // The target survives but its user is gone (or
                                // the survivor is in an unexpected role): tear
                                // the link down — NEVER console-attach to it.
                                sp.role = Role::None;
                                queue.push_back((si, Event::DlDisconnectRequest));
                            }
                        }
                    }
                }
            }
        }
    }
}

/// Create the peer slot + role for an outbound connect to `target`, resolving
/// its endpoint from the heard-table (beacon target as fallback).
fn start_outbound(
    peers: &mut [Option<PeerState>; session::MAX_SESSIONS],
    heard: &[Option<(Callsign, IpEndpoint)>; 8],
    beacon_ep: Option<IpEndpoint>,
    target: Callsign,
    local: Callsign,
    role: Role,
) -> Result<usize, &'static str> {
    if find_peer(peers, &target).is_some() {
        return Err("target is busy (session already up)");
    }
    let Some(ep) = heard_lookup(heard, &target).or(beacon_ep) else {
        return Err("no known endpoint for target");
    };
    let Some(i) = peer_slot(peers, target, local, ep) else {
        return Err("no free session slot");
    };
    let mut name = [0u8; 16];
    let mut lname = [0u8; 16];
    defmt::info!(
        "axudp: outbound connect to {=str} (as {=str}) at {:?}",
        call_str(&target, &mut name),
        call_str(&local, &mut lname),
        ep
    );
    peers[i].as_mut().expect("slot just ensured").role = role;
    Ok(i)
}

fn find_peer(peers: &[Option<PeerState>], peer: &Callsign) -> Option<usize> {
    peers
        .iter()
        .position(|p| matches!(p, Some(ps) if ps.peer == *peer))
}

fn find_peer_mut<'a>(
    peers: &'a mut [Option<PeerState>],
    peer: &Callsign,
) -> Option<&'a mut PeerState> {
    peers.iter_mut().flatten().find(|ps| ps.peer == *peer)
}

fn find_role(peers: &[Option<PeerState>], pred: impl Fn(&Role) -> bool) -> Option<usize> {
    peers
        .iter()
        .position(|p| matches!(p, Some(ps) if pred(&ps.role)))
}

/// Find or create the [`PeerState`] slot for `peer`. Returns its index.
fn peer_slot(
    peers: &mut [Option<PeerState>],
    peer: Callsign,
    local: Callsign,
    endpoint: IpEndpoint,
) -> Option<usize> {
    if let Some(i) = find_peer(peers, &peer) {
        return Some(i);
    }
    let free = peers.iter().position(|p| p.is_none())?;
    peers[free] = Some(PeerState {
        peer,
        local,
        timers: session::EmbassyTimers::new(),
        endpoint,
        role: Role::None,
    });
    Some(free)
}

/// Reap a fully-disconnected session (after its upward signals were drained)
/// and the peer slot with it — capacity reclaimed, timers stopped.
fn reap(sessions: &mut session::Sessions, peers: &mut [Option<PeerState>], i: usize) {
    if let Some(ps) = &peers[i] {
        if sessions.reap(&ps.peer) {
            peers[i] = None;
        }
    }
}

/// Send each wire frame to `ep` with the AXUDP FCS appended.
async fn send_all(socket: &UdpSocket<'_>, ep: IpEndpoint, frames: Vec<Vec<u8>>) {
    for wire in frames {
        let dgram = axudp::append_fcs(wire);
        if let Err(e) = socket.send_to(&dgram, ep).await {
            defmt::warn!("axudp: session tx error {:?}", e);
        }
    }
}

/// Record `call → endpoint` in the heard table (update in place, else first
/// free slot, else overwrite the oldest by rotation).
fn heard_update(heard: &mut [Option<(Callsign, IpEndpoint)>; 8], call: Callsign, ep: IpEndpoint) {
    if let Some(e) = heard.iter_mut().flatten().find(|(c, _)| *c == call) {
        e.1 = ep;
        return;
    }
    if let Some(slot) = heard.iter_mut().find(|s| s.is_none()) {
        *slot = Some((call, ep));
        return;
    }
    heard.rotate_left(1);
    heard[7] = Some((call, ep));
}

/// Resolve a callsign to its last-heard endpoint.
fn heard_lookup(
    heard: &[Option<(Callsign, IpEndpoint)>; 8],
    call: &Callsign,
) -> Option<IpEndpoint> {
    heard
        .iter()
        .flatten()
        .find(|(c, _)| c == call)
        .map(|(_, ep)| *ep)
}

/// Post one event into `ps.peer`'s session and service every DL signal it
/// raises according to the peer's [`Role`]. Returns the wire frames to
/// transmit and any cross-peer follow-ups for [`drive`] to apply.
fn post_one(
    sessions: &mut session::Sessions,
    ps: &mut PeerState,
    event: Event,
    console_id: &Identity,
    prompt: &str,
) -> (Vec<Vec<u8>>, Vec<FollowUp>) {
    let peer = ps.peer;
    let local = ps.local;
    let mut to_send = sessions.post_with_local(local, peer, event, &mut ps.timers);
    let mut followups = Vec::new();

    // Service upward signals until quiescent (each console reply posts a
    // DlDataRequest, which can raise further signals; bounded in practice).
    loop {
        let ups = sessions.take_upward(&peer);
        if ups.is_empty() {
            break;
        }
        for sig in ups {
            match sig {
                DataLinkSignal::ConnectIndication => {
                    let mut name = [0u8; 16];
                    defmt::info!(
                        "axudp: AX.25 session up from {=str} — attaching console",
                        call_str(&peer, &mut name)
                    );
                    ps.role = Role::Console(LineAssembler::default());
                    let banner = banner_and_prompt(console_id, prompt, TransportKind::Ax25);
                    to_send.extend(sessions.post_with_local(
                        local,
                        peer,
                        Event::DlDataRequest(PID_NO_LAYER3, banner),
                        &mut ps.timers,
                    ));
                }
                DataLinkSignal::ConnectConfirm => match ps.role {
                    Role::TelnetRelay => {
                        defmt::info!("axudp: relay link up");
                        relay::STATUS.signal(RelayStatus::Connected);
                    }
                    Role::Bridge { .. } => {
                        let mut name = [0u8; 16];
                        defmt::info!(
                            "axudp: bridge link to {=str} is up",
                            call_str(&peer, &mut name)
                        );
                        // The target's own banner flows over the bridge next.
                    }
                    _ => {}
                },
                DataLinkSignal::DataIndication(_pid, info) => match &mut ps.role {
                    Role::Console(asm) => {
                        let lines = asm.push(&info);
                        for line in lines {
                            let cmd = parse_bytes(&line);
                            let resp = dispatch(&cmd, console_id, TransportKind::Ax25);
                            let mut reply = resp.body;
                            let mut disconnect = false;
                            let mut bridging = false;
                            match resp.outcome {
                                DispatchOutcome::Continue => {}
                                DispatchOutcome::Disconnect => disconnect = true,
                                DispatchOutcome::ConnectThenRelay(call) => {
                                    // "Connecting to X..." is already in reply;
                                    // the bridge proper is cross-peer work.
                                    bridging = true;
                                    followups.push(FollowUp::StartBridge {
                                        console_peer: peer,
                                        target: call,
                                    });
                                }
                            }
                            if !disconnect && !bridging {
                                reply.extend_from_slice(prompt.as_bytes());
                            }
                            if !reply.is_empty() {
                                to_send.extend(sessions.post_with_local(
                                    local,
                                    peer,
                                    Event::DlDataRequest(PID_NO_LAYER3, reply),
                                    &mut ps.timers,
                                ));
                            }
                            if disconnect {
                                to_send.extend(sessions.post_with_local(
                                    local,
                                    peer,
                                    Event::DlDisconnectRequest,
                                    &mut ps.timers,
                                ));
                            }
                        }
                    }
                    Role::Bridge { other, .. } => {
                        followups.push(FollowUp::Forward {
                            to: *other,
                            data: info,
                        });
                    }
                    Role::TelnetRelay => {
                        if relay::AX_TO_USER.try_write(&info).is_err() {
                            defmt::warn!("axudp: relay pipe full, dropping {=usize}B", info.len());
                        }
                    }
                    Role::None => {}
                },
                DataLinkSignal::DisconnectIndication | DataLinkSignal::DisconnectConfirm => {
                    let mut name = [0u8; 16];
                    defmt::info!(
                        "axudp: AX.25 session with {=str} closed",
                        call_str(&peer, &mut name)
                    );
                    match core::mem::replace(&mut ps.role, Role::None) {
                        Role::TelnetRelay => relay::STATUS.signal(RelayStatus::Disconnected),
                        Role::Bridge { other, .. } => {
                            followups.push(FollowUp::BridgeEnded { survivor: other })
                        }
                        _ => {}
                    }
                }
                DataLinkSignal::UnitDataIndication(..) => {}
                DataLinkSignal::ErrorIndication(code) => {
                    defmt::warn!("axudp: DL error indication {=str}", code);
                }
            }
        }
    }

    (to_send, followups)
}
