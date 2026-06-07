//! Capability 1 — AXUDP: AX.25-over-UDP for node↔node connectivity over WiFi.
//!
//! Ports `Packet.Axudp.AxudpSocket` onto `embassy_net::udp::UdpSocket` (the 1:1
//! mapping the research note identifies). The UDP payload is the AX.25 frame
//! body + the mandatory trailing FCS; framing comes from
//! [`ax25_node_core::axudp`].
//!
//! Beyond the socket loop + read-only NET/ROM tap, this task owns **the
//! connected-mode session layer for the AXUDP port**: inbound frames that pass
//! the address filter are classified ([`classify_incoming`]) and posted into a
//! [`session::Sessions`] manager (the host-tested SDL runtime); the wire frames
//! each session emits go back to the peer's UDP endpoint, and the DL signals
//! raised upward drive **the node console over AX.25** (`TransportKind::Ax25`,
//! CR line discipline) — connect from a real BPQ peer and you land at the same
//! prompt telnet users get.
//!
//! **Timers are live**: each peer carries its own [`session::EmbassyTimers`]
//! (T1 retransmit / T2 ack-delay / T3 idle), the main select loop wakes at the
//! earliest armed deadline across all peers, and expiries post the matching
//! `Event::T?Expiry` into that peer's session — so retransmission, ack timing
//! and dead-peer link failure (N2 retries exhausted → teardown) run exactly as
//! the SDL tables specify, against the peer's *last heard* UDP endpoint.
//!
//! **Outbound connects** (`C <call>` from the telnet console) arrive over the
//! [`super::relay`] statics: this task resolves the target's UDP endpoint from
//! its heard-table (every decoded frame records `source → endpoint`; the
//! beacon target is the fallback), posts `DlConnectRequest`, and while the
//! relay is up routes that peer's `DataIndication`s into the relay pipe
//! instead of the console dispatcher.
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

/// Per-peer link state alongside the manager's session slot: the peer's own
/// T1/T2/T3 timer service, the UDP endpoint it was last heard from (where
/// timer-generated frames go), and the console line assembler once attached.
struct PeerState {
    peer: Callsign,
    timers: session::EmbassyTimers,
    endpoint: IpEndpoint,
    console: Option<LineAssembler>,
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
    // The single active console relay, if any (see transports::relay).
    let mut relay_peer: Option<Callsign> = None;

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

        // Relay arm: a pending connect request when idle; user bytes / hangup
        // while a relay is active.
        let relay_fut = async {
            match relay_peer {
                None => RelayEvent::Connect(relay::CONNECT_REQ.receive().await),
                Some(_) => {
                    let mut buf = [0u8; 128];
                    match select(relay::USER_TO_AX.read(&mut buf), relay::USER_HANGUP.wait()).await
                    {
                        Either::First(n) => RelayEvent::UserData(buf, n),
                        Either::Second(()) => RelayEvent::Hangup,
                    }
                }
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

                // Address filter → the connected-mode session layer.
                if frame.destination.callsign == my_call && !frame.is_ui() {
                    let Some(event) = classify_incoming(&frame) else {
                        continue;
                    };
                    let peer = frame.source.callsign;
                    let Some(i) = peer_slot(&mut peers, peer, meta.endpoint) else {
                        defmt::warn!("axudp: peer table full, dropping session frame");
                        continue;
                    };
                    let ps = peers[i].as_mut().expect("slot just ensured");
                    ps.endpoint = meta.endpoint; // frames go to the last-heard endpoint
                    let to_send = run_session(
                        &mut sessions,
                        ps,
                        event,
                        &console_id,
                        &prompt,
                        &mut relay_peer,
                    );
                    send_all(&socket, ps.endpoint, to_send).await;
                    reap(&mut sessions, &mut peers, i);
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
                    if expired.is_empty() {
                        continue;
                    }
                    let mut to_send = Vec::new();
                    for id in expired {
                        defmt::debug!("axudp: timer expiry ({=u8})", id as u8);
                        to_send.extend(run_session(
                            &mut sessions,
                            ps,
                            session::expiry_event(id),
                            &console_id,
                            &prompt,
                            &mut relay_peer,
                        ));
                    }
                    let ep = ps.endpoint;
                    send_all(&socket, ep, to_send).await;
                    reap(&mut sessions, &mut peers, i);
                }
            }
            Either4::Fourth(ev) => match ev {
                RelayEvent::Connect(target) => {
                    let mut name = [0u8; 16];
                    let ep = heard_lookup(&heard, &target).or(beacon_ep);
                    let Some(ep) = ep else {
                        defmt::warn!(
                            "axudp: relay connect to {=str}: no known endpoint",
                            call_str(&target, &mut name)
                        );
                        relay::STATUS.signal(RelayStatus::Failed("no known endpoint"));
                        continue;
                    };
                    let Some(i) = peer_slot(&mut peers, target, ep) else {
                        relay::STATUS.signal(RelayStatus::Failed("no free session slot"));
                        continue;
                    };
                    defmt::info!(
                        "axudp: relay connecting to {=str} at {:?}",
                        call_str(&target, &mut name),
                        ep
                    );
                    relay_peer = Some(target);
                    let ps = peers[i].as_mut().expect("slot just ensured");
                    let to_send = run_session(
                        &mut sessions,
                        ps,
                        Event::DlConnectRequest,
                        &console_id,
                        &prompt,
                        &mut relay_peer,
                    );
                    send_all(&socket, ep, to_send).await;
                    reap(&mut sessions, &mut peers, i);
                }
                RelayEvent::UserData(buf, n) => {
                    if let Some(peer) = relay_peer {
                        if let Some(i) = peers
                            .iter()
                            .position(|p| matches!(p, Some(ps) if ps.peer == peer))
                        {
                            let ps = peers[i].as_mut().expect("present");
                            let to_send = run_session(
                                &mut sessions,
                                ps,
                                Event::DlDataRequest(PID_NO_LAYER3, buf[..n].to_vec()),
                                &console_id,
                                &prompt,
                                &mut relay_peer,
                            );
                            let ep = ps.endpoint;
                            send_all(&socket, ep, to_send).await;
                            reap(&mut sessions, &mut peers, i);
                        }
                    }
                }
                RelayEvent::Hangup => {
                    if let Some(peer) = relay_peer.take() {
                        if let Some(i) = peers
                            .iter()
                            .position(|p| matches!(p, Some(ps) if ps.peer == peer))
                        {
                            let ps = peers[i].as_mut().expect("present");
                            let mut rp = None; // relay already over from our side
                            let to_send = run_session(
                                &mut sessions,
                                ps,
                                Event::DlDisconnectRequest,
                                &console_id,
                                &prompt,
                                &mut rp,
                            );
                            let ep = ps.endpoint;
                            send_all(&socket, ep, to_send).await;
                            reap(&mut sessions, &mut peers, i);
                        }
                    }
                }
            },
        }
    }
}

/// What the relay select-arm produced.
enum RelayEvent {
    /// A console asked to connect to this callsign.
    Connect(Callsign),
    /// Console-user bytes for the relay peer.
    UserData([u8; 128], usize),
    /// The console user went away — disconnect the relay link.
    Hangup,
}

/// Record `call → endpoint` in the heard table (update in place, else first
/// free slot, else overwrite the oldest by rotation).
fn heard_update(heard: &mut [Option<(Callsign, IpEndpoint)>; 8], call: Callsign, ep: IpEndpoint) {
    if let Some(e) = heard
        .iter_mut()
        .flatten()
        .find(|(c, _)| *c == call)
    {
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
fn heard_lookup(heard: &[Option<(Callsign, IpEndpoint)>; 8], call: &Callsign) -> Option<IpEndpoint> {
    heard.iter().flatten().find(|(c, _)| c == call).map(|(_, ep)| *ep)
}

/// Find or create the [`PeerState`] slot for `peer`. Returns its index.
fn peer_slot(
    peers: &mut [Option<PeerState>],
    peer: Callsign,
    endpoint: IpEndpoint,
) -> Option<usize> {
    if let Some(i) = peers
        .iter()
        .position(|p| matches!(p, Some(ps) if ps.peer == peer))
    {
        return Some(i);
    }
    let free = peers.iter().position(|p| p.is_none())?;
    peers[free] = Some(PeerState {
        peer,
        timers: session::EmbassyTimers::new(),
        endpoint,
        console: None,
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

/// Post one event into `ps.peer`'s session and service every DL signal it
/// raises — the AX.25 console loop, or the relay pipes when `ps.peer` is the
/// active relay target. Returns all wire frames to transmit.
fn run_session(
    sessions: &mut session::Sessions,
    ps: &mut PeerState,
    event: Event,
    console_id: &Identity,
    prompt: &str,
    relay_peer: &mut Option<Callsign>,
) -> Vec<Vec<u8>> {
    let is_relay = *relay_peer == Some(ps.peer);
    let peer = ps.peer;
    let mut to_send = sessions.post(peer, event, &mut ps.timers);

    // Service upward signals until quiescent (each console reply posts a
    // DlDataRequest, which can raise further signals; bounded in practice).
    loop {
        let ups = sessions.take_upward(&peer);
        if ups.is_empty() {
            break;
        }
        for sig in ups {
            match sig {
                DataLinkSignal::ConnectConfirm if is_relay => {
                    let mut name = [0u8; 16];
                    defmt::info!(
                        "axudp: relay link to {=str} is up",
                        call_str(&peer, &mut name)
                    );
                    relay::STATUS.signal(RelayStatus::Connected);
                }
                DataLinkSignal::DataIndication(_pid, info) if is_relay => {
                    // Peer → console user. try_write: the console side drains
                    // promptly at human speeds; overflow is logged, not fatal.
                    if relay::AX_TO_USER.try_write(&info).is_err() {
                        defmt::warn!("axudp: relay pipe full, dropping {=usize}B", info.len());
                    }
                }
                DataLinkSignal::DisconnectIndication | DataLinkSignal::DisconnectConfirm
                    if is_relay =>
                {
                    let mut name = [0u8; 16];
                    defmt::info!(
                        "axudp: relay link to {=str} ended",
                        call_str(&peer, &mut name)
                    );
                    *relay_peer = None;
                    relay::STATUS.signal(RelayStatus::Disconnected);
                }
                DataLinkSignal::ConnectIndication => {
                    let mut name = [0u8; 16];
                    defmt::info!(
                        "axudp: AX.25 session up from {=str} — attaching console",
                        call_str(&peer, &mut name)
                    );
                    ps.console = Some(LineAssembler::default());
                    let banner = banner_and_prompt(console_id, prompt, TransportKind::Ax25);
                    to_send.extend(sessions.post(
                        peer,
                        Event::DlDataRequest(PID_NO_LAYER3, banner),
                        &mut ps.timers,
                    ));
                }
                DataLinkSignal::DataIndication(_pid, info) => {
                    let lines = match ps.console.as_mut() {
                        Some(asm) => asm.push(&info),
                        None => Vec::new(),
                    };
                    for line in lines {
                        let cmd = parse_bytes(&line);
                        let resp = dispatch(&cmd, console_id, TransportKind::Ax25);
                        let mut reply = resp.body;
                        let mut disconnect = false;
                        match resp.outcome {
                            DispatchOutcome::Continue => {}
                            DispatchOutcome::Disconnect => disconnect = true,
                            DispatchOutcome::ConnectThenRelay(_call) => {
                                reply.extend_from_slice(
                                    b"...node-to-node connects aren't wired yet (bring-up)\r",
                                );
                            }
                        }
                        if !disconnect {
                            reply.extend_from_slice(prompt.as_bytes());
                        }
                        if !reply.is_empty() {
                            to_send.extend(sessions.post(
                                peer,
                                Event::DlDataRequest(PID_NO_LAYER3, reply),
                                &mut ps.timers,
                            ));
                        }
                        if disconnect {
                            to_send.extend(sessions.post(
                                peer,
                                Event::DlDisconnectRequest,
                                &mut ps.timers,
                            ));
                        }
                    }
                }
                DataLinkSignal::DisconnectIndication | DataLinkSignal::DisconnectConfirm => {
                    let mut name = [0u8; 16];
                    defmt::info!(
                        "axudp: AX.25 session with {=str} closed",
                        call_str(&peer, &mut name)
                    );
                    ps.console = None;
                }
                DataLinkSignal::ConnectConfirm => {
                    // Outbound connects (node-to-node) arrive with the relay work.
                }
                DataLinkSignal::UnitDataIndication(..) => {}
                DataLinkSignal::ErrorIndication(code) => {
                    defmt::warn!("axudp: DL error indication {=str}", code);
                }
            }
        }
    }

    to_send
}
