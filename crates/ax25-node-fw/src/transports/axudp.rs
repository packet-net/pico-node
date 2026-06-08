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
//! **NET/ROM L4 circuits terminate here too**: inbound PID-0xCF I-frames are
//! interlink datagrams, fed to a [`NetRomConnector`] (the host-tested sans-io
//! L4 stack). Circuits addressed to this node are auto-accepted and get the
//! node console attached — `C PICO` from any NET/ROM neighbour lands at the
//! same prompt L2 users get; the connector's outbound datagrams ride back as
//! PID-0xCF I-frames over the neighbour's L2 session.
//!
//! Single-transport ownership (this task owns `Sessions` exclusively) keeps the
//! `&mut` story trivial; when a second connected-mode transport arrives the
//! manager moves behind the supervisor seam `session.rs` documents.

use ax25_node_core::ax25::{Callsign, PID_NETROM, PID_NO_LAYER3};
use ax25_node_core::axudp;
use ax25_node_core::console::command::parse_bytes;
use ax25_node_core::console::service::{banner_and_prompt, dispatch, Identity};
use ax25_node_core::console::{DispatchOutcome, LineAssembler, TransportKind};
use ax25_node_core::netrom::routing::inp3_sntt::SNTT_UNSET;
use ax25_node_core::netrom::transport::inp3_engine::{Inp3Engine, Inp3NeighbourDownEvent};
use ax25_node_core::netrom::transport::inp3_update_scheduler::Inp3UpdateScheduler;
use ax25_node_core::netrom::wire::inp3_l3rtt;
use ax25_node_core::netrom::wire::inp3_options::NetRomInp3Options;
use ax25_node_core::netrom::wire::inp3_rif::Inp3Rif;
use ax25_node_core::netrom::wire::{Alias, NetRomPacket};
use ax25_node_core::netrom::{
    CircuitEvent, NetRomConnection, NetRomConnector, NetRomConnectorOptions, NetRomOriginator,
    NetRomOriginatorOptions,
};
use ax25_node_core::netrom::{ObserveOutcome, PortId};
use ax25_node_core::sdl::{
    classify_incoming, DataLinkSignal, Event, FrameSpec, UnnumberedKind, WireSink,
};

use alloc::collections::VecDeque;
use alloc::string::String;
use alloc::vec::Vec;

use embassy_futures::select::{select, select4, Either, Either4};
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{IpEndpoint, Stack};
use embassy_time::{Duration, Instant, Ticker, Timer};

use crate::config::{AxudpConfig, NetRomConfig};
use crate::session;
use crate::transports::relay::{self, RelayStatus};
use crate::transports::{call_str, parse_endpoint, ui_frame};

/// Seconds between beacon UI frames when a beacon target is configured.
const BEACON_INTERVAL_SECS: u64 = 10;
/// Seconds between routing-table flash saves (bounds wear; only saves on change).
const NETROM_SAVE_SECS: u64 = 300;
/// Seconds between "ensure interlinks" passes — proactively (re)establish an
/// L2 link to every known NET/ROM neighbour we can reach, BPQ-style.
const INTERLINK_ENSURE_SECS: u64 = 30;

/// Master gate for the INP3 time-routing overlay on this firmware host (the
/// analogue of the C# `config.Inp3.Enabled` / the ax25-ts connector `inp3` opt-in).
/// Default ON for the demo node — flip later via a config knob. Kept a single const
/// here so the whole overlay (engine + scheduler construction, the inbound 0xCF tap,
/// the tick fan-out) is trivially gateable: when `false` the [`Inp3Host`] is never
/// constructed and every INP3 step is a no-op.
const INP3_ENABLED: bool = option_env!("INP3_DISABLE").is_none();

/// Demo-node INP3 cadences (SHORT, so a 2-node lab demo shows the time-route being
/// LEARNED in seconds, not minutes — production tunables are 60 s / 300 s / 180 s).
/// L3RTT probe every ~5 s; periodic full RIF every ~10 s; reflection-timeout reset
/// at ~30 s. Encoded as ms (the netrom no_std idiom).
const INP3_L3RTT_INTERVAL_MS: u32 = 5_000;
const INP3_RIF_INTERVAL_MS: u32 = 10_000;
const INP3_RESET_WINDOW_MS: u32 = 30_000;
/// The positive-update debounce — must be > 0 and < the RIF interval. Kept short for
/// the demo so a newly-learned route fans out within ~1 s of its debounce.
const INP3_POSITIVE_DEBOUNCE_MS: u32 = 1_000;

/// Set by a console REBOOT on the AX.25 path; honoured by [`drive`] after the
/// response frames have been transmitted (so the farewell reaches the user).
static REBOOT_PENDING: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

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
    /// A persistent L2 link to a NET/ROM neighbour — kept up (proactively
    /// established + auto-reconnected) so L4 circuits always have transport,
    /// like a BPQ interlink. Carries only PID-0xCF L4 traffic (handled before
    /// the role match); other DL signals are ignored.
    Interlink,
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
    /// An inbound PID-0xCF interlink datagram for the NET/ROM connector.
    NetRom {
        neighbour: Callsign,
        datagram: Vec<u8>,
    },
}

#[embassy_executor::task]
pub async fn task(
    stack: Stack<'static>,
    cfg: AxudpConfig,
    netrom_cfg: NetRomConfig,
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

    // Repopulate the routing table from flash (survives power failure — like
    // BPQ's BPQNODES.dat). Replays persisted routes through the live ingest
    // path so a rebooted node knows its routes immediately.
    let replayed = crate::config_store::netrom_load(&mut netrom, my_call);
    if replayed > 0 {
        defmt::info!("netrom: {=usize} route(s) restored from flash", replayed);
    }
    // Persist the table periodically (NOT per-broadcast — flash wear), and only
    // when it changed since the last save.
    let mut next_save_at = Instant::now() + Duration::from_secs(NETROM_SAVE_SECS);
    let mut next_interlink_at = Instant::now() + Duration::from_secs(INTERLINK_ENSURE_SECS);

    // NODES origination: our own broadcasts, built from the live routing table
    // (header alias + an entry per advertisable route, OBSMIN-gated) — the node
    // becomes *visible* in peers' nodes tables. The interval follows the BPQ
    // convention; the first broadcast goes out on the first beacon tick so a
    // fresh boot announces promptly.
    let originator = NetRomOriginator::new(NetRomOriginatorOptions {
        enabled: netrom_cfg.originate,
        alias: Some(Alias::from_str_lossy(&console_id.node_name)),
        node_call: Some(my_call),
        obsolete_minimum: None,
    });
    // The NET/ROM L4 connector: terminates circuits addressed to us (and can
    // forward transit datagrams). Sans-io: fed by FollowUp::NetRom, drained in
    // service_l4.
    let mut connector = NetRomConnector::new(
        my_call,
        NetRomConnectorOptions {
            enabled: true,
            ..Default::default()
        },
    );
    let mut circuits: [Option<CircuitConsole>; 4] = [const { None }; 4];

    // INP3 time-routing overlay (the embedded host wiring — the analogue of the C#
    // `NetRomService.Inp3Host` + the ax25-ts connector `inp3` glue). Constructed once
    // here, only when the overlay is on, with the node call already known (the C# sets
    // the engine's local node at AttachPort; we set it at construction, since `my_call`
    // is a parameter). Quality forwarding is left primary
    // (`set_prefer_inp3_routes(false)`) so the demo shows the time-route being LEARNED,
    // not yet routed by (AWARENESS ONLY, matching the C#/TS slice). `None` when off —
    // the engine/scheduler are then never even constructed.
    let mut inp3: Option<Inp3Host> = if INP3_ENABLED {
        connector.set_prefer_inp3_routes(false);
        let host = Inp3Host::new(my_call);
        defmt::info!(
            "axudp: INP3 overlay constructed (probe {=u32}ms, rif {=u32}ms, reset {=u32}ms) — awareness-only, quality forwarding unchanged",
            INP3_L3RTT_INTERVAL_MS,
            INP3_RIF_INTERVAL_MS,
            INP3_RESET_WINDOW_MS
        );
        Some(host)
    } else {
        defmt::info!("axudp: INP3 overlay disabled");
        None
    };

    let nodes_interval = Duration::from_secs(netrom_cfg.nodes_interval_secs as u64);
    let mut next_nodes_at = Instant::now(); // announce on the first tick
    if netrom_cfg.originate {
        defmt::info!(
            "axudp: NODES origination on, every {=u32}s",
            netrom_cfg.nodes_interval_secs
        );
    }

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
                        Callsign::parse("IDENT").expect("static"),
                        PID_NO_LAYER3,
                        b"pico-node AXUDP beacon (HW-BRINGUP Gate 3)",
                    );
                    let dgram = axudp::encode_datagram(&beacon);
                    match socket.send_to(&dgram, ep).await {
                        Ok(()) => defmt::info!("axudp: beacon sent ({=usize} bytes)", dgram.len()),
                        Err(e) => defmt::warn!("axudp: beacon send error {:?}", e),
                    }
                }

                // Reflect the live route counts on the OLED + MQTT status.
                crate::oled::set_counts(
                    netrom.neighbour_count() as u16,
                    netrom.destination_count() as u16,
                );
                crate::mqtt::set_status(
                    netrom.neighbour_count() as u16,
                    netrom.destination_count() as u16,
                );
                // Publish the rendered route lines (NET/ROM + INP3 metric) for the
                // `Nodes` console command — the console tasks read this snapshot
                // since they don't share this task's routing table.
                crate::netrom_view::set_routes(netrom.route_lines());

                // Persistent interlinks: keep an L2 link up to every reachable
                // NET/ROM neighbour, so the connector's L4 datagrams always
                // have transport (no "no L2 session" drops) — BPQ-style.
                if Instant::now() >= next_interlink_at {
                    next_interlink_at = Instant::now() + Duration::from_secs(INTERLINK_ENSURE_SECS);
                    ensure_interlinks(
                        &mut sessions,
                        &mut peers,
                        &socket,
                        &heard,
                        beacon_ep,
                        my_call,
                        &mut netrom,
                        &mut connector,
                        &mut circuits,
                        inp3.as_mut(),
                        &console_id,
                        &prompt,
                    )
                    .await;
                }

                // Routing-table persistence: the save self-gates on a content
                // CRC, so this only erases flash when the table actually changed
                // (a stable node writes nothing — flash wear tracks topology
                // churn, not the save cadence).
                if Instant::now() >= next_save_at {
                    next_save_at = Instant::now() + Duration::from_secs(NETROM_SAVE_SECS);
                    match crate::config_store::netrom_save(&netrom) {
                        Ok(n) if n > 0 => {
                            defmt::info!("netrom: {=usize} route(s) saved to flash (changed)", n)
                        }
                        Ok(_) => {} // unchanged — no write
                        Err(e) => defmt::warn!("netrom: save failed: {=str}", e),
                    }
                }

                // L4 circuit timers (ack/retransmit/idle) ride the beacon tick.
                connector.tick(netrom.table(), now_ms());
                {
                    let mut queue: VecDeque<(usize, Event)> = VecDeque::new();
                    service_l4(
                        &mut L4 {
                            connector: &mut connector,
                            netrom: &mut netrom,
                            circuits: &mut circuits,
                            inp3: inp3.as_mut(),
                        },
                        &peers,
                        &mut queue,
                        &console_id,
                        &prompt,
                    );
                    while let Some((i, ev)) = queue.pop_front() {
                        drive(
                            &mut sessions,
                            &mut peers,
                            &socket,
                            &heard,
                            beacon_ep,
                            my_call,
                            &mut L4 {
                                connector: &mut connector,
                                netrom: &mut netrom,
                                circuits: &mut circuits,
                                inp3: inp3.as_mut(),
                            },
                            i,
                            ev,
                            &console_id,
                            &prompt,
                        )
                        .await;
                    }
                }

                // INP3 time-routing overlay tick (the embedded host wiring). Rides the
                // beacon tick — the single driver, the core owns no ambient timer (as the
                // C# host's 1 s timer / the ax25-ts connector `tick`). Robust: a faulting
                // INP3 step must not kill the task, and the whole block is a no-op when
                // the overlay is off (`inp3` is `None`). Drive engine + scheduler in the
                // locked order, then SHIP each produced frame over the SAME PID-0xCF
                // interlink seam the connector's outbound datagrams use (find the
                // neighbour's L2 session; cold interlink → drop, don't dial).
                if let Some(round) = inp3
                    .as_mut()
                    // Advance the engine + scheduler in the locked order. The `host`
                    // borrow of `inp3` ends at the `tick_round` call (NLL: last use),
                    // so the ship loop below is free to reborrow `inp3` for L4.
                    .map(|host| host.tick_round(&mut netrom, my_call, now_ms()))
                {

                    // L3RTT probes + reflections, then advertised RIFs — both ride the
                    // neighbour's interlink as PID-0xCF I-frames, exactly as
                    // `service_l4` ships an InterlinkSend (find_peer → drive a
                    // DlDataRequest(PID_NETROM, …)); a cold interlink is dropped.
                    for (nbr, bytes) in round.l3rtt.into_iter().chain(round.rifs.into_iter()) {
                        if let Some(i) = find_peer(&peers, &nbr) {
                            drive(
                                &mut sessions,
                                &mut peers,
                                &socket,
                                &heard,
                                beacon_ep,
                                my_call,
                                &mut L4 {
                                    connector: &mut connector,
                                    netrom: &mut netrom,
                                    circuits: &mut circuits,
                                    inp3: inp3.as_mut(),
                                },
                                i,
                                Event::DlDataRequest(PID_NETROM, bytes),
                                &console_id,
                                &prompt,
                            )
                            .await;
                        } else {
                            let mut name = [0u8; 16];
                            defmt::debug!(
                                "axudp: INP3 frame dropped — no interlink up to {=str} (drop, don't dial)",
                                call_str(&nbr, &mut name)
                            );
                        }
                    }

                    // Neighbour-down events: no clean table-mut / mark-neighbour-down
                    // seam is reachable through the firmware's `netrom` handle from here
                    // (it lives on the routing table, not NetRomService), so — per the
                    // brief — log and skip rather than invent a teardown. The engine has
                    // already dropped the neighbour's INP3 state.
                    for down in &round.downs {
                        let mut name = [0u8; 16];
                        defmt::info!(
                            "axudp: INP3 neighbour {=str} down (silent {=u64}ms) — engine state reset (no table teardown wired)",
                            call_str(&down.neighbour, &mut name),
                            down.silent_for_ms
                        );
                    }
                }

                // NODES origination rides the beacon tick (10 s granularity is
                // plenty against minutes-scale intervals).
                if netrom_cfg.originate && Instant::now() >= next_nodes_at {
                    next_nodes_at = Instant::now() + nodes_interval;
                    let payloads = originator.broadcast_nodes(netrom.table());
                    // BPQ semantics: NODES go to every B-flagged map. Our
                    // analogue: the beacon target + every distinct endpoint in
                    // the heard table (the LAN peers we actually know).
                    let mut targets: [Option<IpEndpoint>; 9] = [None; 9];
                    let mut n_targets = 0usize;
                    for ep in beacon_ep
                        .iter()
                        .copied()
                        .chain(heard.iter().flatten().map(|(_, ep)| *ep))
                    {
                        if !targets[..n_targets].iter().flatten().any(|t| *t == ep) {
                            targets[n_targets] = Some(ep);
                            n_targets += 1;
                        }
                    }
                    let dest = NetRomOriginator::nodes_destination();
                    for payload in &payloads {
                        let frame = ui_frame(my_call, dest, NetRomOriginator::PID, payload);
                        let dgram = axudp::encode_datagram(&frame);
                        for ep in targets[..n_targets].iter().flatten() {
                            if let Err(e) = socket.send_to(&dgram, *ep).await {
                                defmt::warn!("axudp: NODES send error {:?}", e);
                            }
                        }
                    }
                    defmt::info!(
                        "axudp: NODES broadcast sent ({=usize} frame(s) to {=usize} endpoint(s))",
                        payloads.len(),
                        n_targets
                    );
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
                    crate::mqtt::log("NODES broadcast ingested");
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
                    let peer = frame.source.callsign;

                    // v2.2 XID negotiation arrives BEFORE SABM and isn't an SDL
                    // event (classify_incoming returns None — the tables carry
                    // only the initiator MDL). Detect it by control byte (0xAF
                    // + optional P/F) and answer like a v2.0 station: DM, so the
                    // peer (BPQ does) falls back to a plain SABM. Only when no
                    // session is up — a mid-session XID is ignored like any
                    // other unclassified frame.
                    const XID: u8 = 0xAF;
                    if frame.control & !0x10 == XID && sessions.session_for(&peer).is_none() {
                        defmt::info!("axudp: XID received — answering DM (v2.0 fallback)");
                        let sink = WireSink::new(my_call, peer, alloc::vec::Vec::new());
                        let dm = sink.build_frame(&FrameSpec::Unnumbered {
                            kind: UnnumberedKind::Dm,
                            is_command: false,
                            pf: (frame.control & 0x10) != 0,
                            expedited: false,
                        });
                        let dgram = axudp::encode_datagram(&dm);
                        if let Err(e) = socket.send_to(&dgram, meta.endpoint).await {
                            defmt::warn!("axudp: DM send error {:?}", e);
                        }
                        continue;
                    }

                    let Some(event) = classify_incoming(&frame) else {
                        continue;
                    };
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
                        &mut L4 {
                            connector: &mut connector,
                            netrom: &mut netrom,
                            circuits: &mut circuits,
                            inp3: inp3.as_mut(),
                        },
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
                            &mut L4 {
                                connector: &mut connector,
                                netrom: &mut netrom,
                                circuits: &mut circuits,
                                inp3: inp3.as_mut(),
                            },
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
                                &mut L4 {
                                    connector: &mut connector,
                                    netrom: &mut netrom,
                                    circuits: &mut circuits,
                                    inp3: inp3.as_mut(),
                                },
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
                            &mut L4 {
                                connector: &mut connector,
                                netrom: &mut netrom,
                                circuits: &mut circuits,
                                inp3: inp3.as_mut(),
                            },
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
                            &mut L4 {
                                connector: &mut connector,
                                netrom: &mut netrom,
                                circuits: &mut circuits,
                                inp3: inp3.as_mut(),
                            },
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

/// A node-console session attached to an inbound NET/ROM L4 circuit.
struct CircuitConsole {
    conn: NetRomConnection,
    asm: LineAssembler,
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
    l4: &mut L4<'_>,
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
        let peer_is_node = {
            let t = l4.netrom.table();
            t.neighbour(&ps.peer).is_some() || t.destination(&ps.peer).is_some()
        };
        let (frames, followups) = post_one(sessions, ps, ev, console_id, prompt, peer_is_node);
        let ep = ps.endpoint;
        send_all(socket, ep, frames).await;
        if REBOOT_PENDING.load(core::sync::atomic::Ordering::Relaxed) {
            // Console REBOOT: give the wire a beat to drain, then reset.
            Timer::after_millis(250).await;
            cortex_m::peripheral::SCB::sys_reset();
        }
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
                FollowUp::NetRom {
                    neighbour,
                    datagram,
                } => {
                    // INP3 peel BEFORE the connector (mirrors the C# `DispatchInp3` /
                    // the ax25-ts `dispatchInp3` precedence): a RIF (0xFF-led) or an
                    // L3RTT is consumed here so it can never reach L4 circuits /
                    // forwarding. `true` ⇒ consumed; fall through to the connector
                    // only on `false`. Disjoint-field borrows of `*l4` (inp3 + netrom).
                    let consumed = match l4.inp3.as_deref_mut() {
                        Some(host) => host.dispatch_inbound(
                            neighbour,
                            &datagram,
                            l4.netrom,
                            my_call,
                            now_ms(),
                        ),
                        None => false,
                    };
                    if !consumed {
                        l4.connector.on_interlink_data(
                            l4.netrom.table(),
                            neighbour,
                            &datagram,
                            now_ms(),
                        );
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

        // L4 housekeeping every round: attach consoles to fresh circuits,
        // service circuit events, and ship the connector's outbound interlink
        // datagrams over the right L2 sessions.
        service_l4(l4, peers, &mut queue, console_id, prompt);
    }
}

/// Millisecond monotonic tick for the sans-io NET/ROM layers.
fn now_ms() -> u64 {
    Instant::now().as_millis()
}

/// The L4 connector bundle threaded through [`drive`].
///
/// `netrom` is a `&mut` borrow (not the read-only `&` the pre-INP3 path used) so the
/// inbound 0xCF tap can ingest a RIF into the shared routing table
/// ([`NetRomService::ingest_rif`]) — the second metric space on the same table —
/// before the datagram would otherwise reach the connector. The connector + circuit
/// reads still go through `netrom.table()` (an immutable reborrow), unchanged.
struct L4<'a> {
    connector: &'a mut NetRomConnector,
    netrom: &'a mut session::NetRom,
    circuits: &'a mut [Option<CircuitConsole>; 4],
    /// The INP3 host (engine + scheduler + per-round withdrawn snapshot), or `None`
    /// when the overlay is off ([`INP3_ENABLED`] false). The inbound tap consults it
    /// in [`drive`]'s `FollowUp::NetRom` arm to peel RIF / L3RTT frames off the 0xCF
    /// stream ahead of the L4 path (mirrors the C# `DispatchInp3` precedence).
    inp3: Option<&'a mut Inp3Host>,
}

/// The embedded INP3 host: owns the host-free [`Inp3Engine`] + [`Inp3UpdateScheduler`]
/// and the per-round drained-withdrawn snapshot, and glues their OUTBOX/TAKE outputs
/// to the firmware's interlink send path + the shared routing table. The no_std
/// analogue of the C# `NetRomService.Inp3Host` nested type and the ax25-ts
/// `NetRomConnector` inp3 fields. Constructed once before the select loop, only when
/// [`INP3_ENABLED`]; when the overlay is off this type is never instantiated and every
/// INP3 step is skipped.
///
/// **Scope: AWARENESS ONLY** (as the C#/TS): the node learns + tells the time-space
/// (probe / ingest / advertise / reset); `set_prefer_inp3_routes(false)` keeps quality
/// forwarding unchanged, so the demo shows the time-route being LEARNED, not yet routed
/// by. Driven by the firmware's beacon tick (no ambient timer — the core has none),
/// exactly as the connector's circuit manager is.
struct Inp3Host {
    engine: Inp3Engine,
    scheduler: Inp3UpdateScheduler,
    /// The resolved overlay options — kept so RIF ingestion uses the configured
    /// `hop_limit` (the C# `options.HopLimit` / ax25-ts `inp3Options.hopLimit`).
    options: NetRomInp3Options,
    /// The recently-withdrawn snapshot DRAINED once at the top of the current fan-out
    /// round and handed to every `build_rif` this round (the atomic round boundary
    /// that mirrors the C# host's `currentRoundWithdrawn` / the ax25-ts
    /// `inp3RoundWithdrawn`). Empty outside a round.
    round_withdrawn: Vec<Callsign>,
}

/// What one [`Inp3Host::tick_round`] produced, for the caller (the ticker arm) to
/// SHIP over the interlinks (the engine/scheduler are host-free + own no I/O, so the
/// frames come back out as data to send). The caller maps each to a PID-0xCF I-frame
/// over the named neighbour's interlink, reusing the exact send seam the connector's
/// outbound datagrams use; a cold interlink is dropped (don't dial).
struct Inp3Round {
    /// Outbound L3RTT sends (probes the engine originated + verbatim reflections of a
    /// peer's probe) — each `(neighbour, frame_bytes)`.
    l3rtt: Vec<(Callsign, Vec<u8>)>,
    /// Built poison-reversed RIFs to advertise — each `(neighbour, rif_bytes)`.
    rifs: Vec<(Callsign, Vec<u8>)>,
    /// Neighbour-down events the engine raised this round (180 s reset of a
    /// previously-capable neighbour). The firmware has no clean table-mut seam from
    /// here (see [`Inp3Host::tick_round`]); these are logged, not torn down.
    downs: Vec<Inp3NeighbourDownEvent>,
}

impl Inp3Host {
    /// Construct the host with the demo cadences, the node call pinned as the engine's
    /// local node (the L3 origin stamped into probes + the reflection self-test
    /// identity — the C# pins it at AttachPort; we pin it here since `my_call` is
    /// known). Options are validated for parity with the C#/TS constructors; on the
    /// impossible event they don't validate we fall back to the canonical demo set
    /// rather than panic (a no_std host never unwraps on a config path).
    fn new(my_call: Callsign) -> Self {
        let options = NetRomInp3Options {
            enabled: true,
            l3_rtt_interval_ms: INP3_L3RTT_INTERVAL_MS,
            l3_rtt_reset_window_ms: INP3_RESET_WINDOW_MS,
            rif_interval_ms: INP3_RIF_INTERVAL_MS,
            positive_debounce_ms: INP3_POSITIVE_DEBOUNCE_MS,
            ..NetRomInp3Options::DEFAULT
        };
        // Validate for symmetry with the C#/TS resolver; if (impossibly) the demo
        // constants ever fall out of range, log and fall back rather than panic.
        let options = match options.validate() {
            Ok(()) => options,
            Err(reason) => {
                defmt::warn!(
                    "axudp: INP3 demo options invalid ({=str}) — using defaults",
                    reason
                );
                NetRomInp3Options {
                    enabled: true,
                    ..NetRomInp3Options::DEFAULT
                }
            }
        };
        Self {
            engine: Inp3Engine::new(my_call, options),
            scheduler: Inp3UpdateScheduler::new(
                options.rif_interval_ms as u64,
                options.positive_debounce_ms as u64,
            ),
            options,
            round_withdrawn: Vec::new(),
        }
    }

    /// The inbound 0xCF dispatch — mirrors the C# `DispatchInp3` / the ax25-ts
    /// `dispatchInp3` precedence EXACTLY, adapted to the Rust core API. Returns `true`
    /// when the frame was consumed as INP3 (the caller must NOT pass it to the
    /// connector); `false` when it is an ordinary L4 datagram to fall through.
    ///
    /// Any neighbour we hear ANYTHING 0xCF from becomes a probe target (optimistic
    /// probing is on by default — even a neighbour that only ever sent L4). Then:
    /// (A) a `0xFF`-led frame is a RIF — consumed regardless of whether it parses (a
    /// malformed RIF is dropped, NEVER retried as L4); a parsed RIF is ingested into
    /// the shared table with the engine's measured SNTT (or the unset sentinel when
    /// the link is un-probed). (B) else an L3RTT (a `NetRomPacket` to `L3RTT-0`) is
    /// timed / reflected by the engine and consumed. Anything else → `false`.
    ///
    /// Never panics: every parse returns `Option`, and a faulting step cannot occur (no
    /// unwraps). `now` is the monotonic ms tick.
    fn dispatch_inbound(
        &mut self,
        from: Callsign,
        info: &[u8],
        netrom: &mut session::NetRom,
        my_call: Callsign,
        now: u64,
    ) -> bool {
        // Optimistic neighbour observation (idempotent refresh) — every 0xCF speaker
        // becomes a probe target.
        self.engine.observe_neighbour(from, now);

        // (A) RIF? — the single-byte 0xFF signature is a total, unambiguous
        // discriminator (a 0xFF first byte can't be a valid AX.25-shifted callsign).
        if info.first() == Some(&Inp3Rif::SIGNATURE) {
            if let Some(rif) = Inp3Rif::try_parse(info) {
                // Supply the engine's measured SNTT for the carrying link, mapped to
                // the table's unset sentinel when the link is not yet probed (the C#
                // `engine.SnttMs(from) ?? Inp3Sntt.Unset` / the ax25-ts `?? SNTT_UNSET`).
                let sntt = self.engine.sntt_ms(&from).unwrap_or(SNTT_UNSET);
                netrom.ingest_rif(from, my_call, sntt, &rif, self.options.hop_limit as u32);
                let mut name = [0u8; 16];
                defmt::info!(
                    "axudp: INP3 RIF ingested from {=str} ({=usize} RIP(s), {=u32} destinations known)",
                    call_str(&from, &mut name),
                    rif.rips.len(),
                    netrom.destination_count() as u32
                );
            }
            // Consumed either way: a 0xFF-led-but-unparseable frame is a malformed RIF,
            // dropped — NEVER retried as an L4 datagram.
            return true;
        }

        // (B) L3RTT? — a well-formed NetRomPacket to L3RTT-0. Decode once, classify by
        // dest/opcode, and let the engine time our reflection or reflect a peer probe.
        if let Some(packet) = NetRomPacket::decode(info) {
            if inp3_l3rtt::is_l3rtt(&packet) {
                // on_l3rtt_packet recognises + processes the L3RTT (verbatim reflect or
                // SNTT fold) and returns true; it never panics on a non-L3RTT.
                self.engine.on_l3rtt_packet(from, &packet, now);
                return true;
            }
        }

        // Not INP3 — fall through to the existing connector (L4) path.
        false
    }

    /// One host tick in the LOCKED order (design §6.4 / the C# `TickOnce` / the
    /// ax25-ts `inp3Tick`): refresh the capable fan-out set from the engine → tick the
    /// engine (probes / resets) → DRAIN the table's recently-withdrawn set ONCE (the
    /// atomic round boundary) and mark each on the scheduler → set the round snapshot →
    /// tick the scheduler. Then build the per-neighbour poison-reversed RIFs from the
    /// SAME drained snapshot, and drain the engine's outbound L3RTT + neighbour-down
    /// outboxes. Returns everything to SHIP; the caller does the interlink I/O.
    ///
    /// Draining the withdrawn set ONCE at the round top (not per-neighbour) is the
    /// race fix: a withdrawal landing mid-round is captured by the NEXT drain, never
    /// cleared unadvertised. Never panics (every step is total).
    fn tick_round(&mut self, netrom: &mut session::NetRom, my_call: Callsign, now: u64) -> Inp3Round {
        // Keep the scheduler's fan-out set current before it reads it.
        let capable = self.engine.inp3_capable_neighbours();
        self.scheduler.set_target_neighbours(&capable);

        // Engine first — may raise neighbour-down (→ a future table mark) and queue
        // probes/reflections.
        self.engine.tick(now);

        // DRAIN the recently-withdrawn set ONCE, mark each NEGATIVE on the scheduler so
        // it fans out THIS round, and remember the snapshot for every build_rif below.
        let withdrawn = netrom.drain_recently_withdrawn();
        for dest in &withdrawn {
            self.scheduler.mark_withdrawn(*dest, now);
        }
        self.round_withdrawn = withdrawn;

        // Scheduler fans out due intents (NEGATIVE immediate / POSITIVE debounced /
        // periodic), one per target neighbour.
        self.scheduler.tick(now);

        // Build the full poison-reversed RIF for each advertise intent from the round's
        // drained snapshot (mirrors the C# Advertise sink's BuildRif(currentRoundWithdrawn)).
        let mut rifs: Vec<(Callsign, Vec<u8>)> = Vec::new();
        for intent in self.scheduler.take_advertise_intents() {
            let rif = netrom.build_rif(my_call, intent.neighbour, &self.round_withdrawn);
            if let Some(bytes) = rif.to_bytes() {
                rifs.push((intent.neighbour, bytes));
            }
        }
        // The snapshot belongs to exactly one round — clear it after the RIFs are built.
        self.round_withdrawn.clear();

        // Drain the engine's outbound L3RTT sends (probes + reflections) to ship.
        let l3rtt: Vec<(Callsign, Vec<u8>)> = self
            .engine
            .take_outbound_l3rtt()
            .into_iter()
            .map(|(nbr, frame)| (nbr, frame.to_bytes()))
            .collect();

        // Drain neighbour-down events. The C# wires these to table.MarkNeighbourDown +
        // a DISC/re-establish; the firmware's `netrom` handle exposes no public
        // mark-neighbour-down / table-mut seam reachable from here (it lives on the
        // routing table, not NetRomService, and "edit only axudp.rs" precludes adding
        // one), so — per the brief — we log them and skip the teardown rather than
        // invent one. The engine has already removed the neighbour's INP3 state.
        let downs = self.engine.take_neighbour_down();

        Inp3Round { l3rtt, rifs, downs }
    }
}

/// Drain the connector: new inbound circuits get the node console + banner;
/// circuit data runs the console dispatcher; closes detach; outbound interlink
/// datagrams are queued as PID-0xCF I-frames to the neighbour's L2 session.
fn service_l4(
    l4: &mut L4<'_>,
    peers: &[Option<PeerState>],
    queue: &mut VecDeque<(usize, Event)>,
    console_id: &Identity,
    prompt: &str,
) {
    let table = l4.netrom.table();

    for conn in l4.connector.take_incoming_connections() {
        let mut name = [0u8; 16];
        defmt::info!(
            "axudp: NET/ROM circuit up from {=str} — attaching console",
            call_str(&conn.peer, &mut name)
        );
        if let Some(slot) = l4.circuits.iter_mut().find(|c| c.is_none()) {
            *slot = Some(CircuitConsole {
                conn,
                asm: LineAssembler::default(),
            });
            let banner = banner_and_prompt(console_id, prompt, TransportKind::Ax25);
            l4.connector.write(table, &conn, &banner, now_ms());
        } else {
            defmt::warn!("axudp: circuit table full, disconnecting");
            l4.connector.disconnect(table, &conn, now_ms());
        }
    }

    for (key, event) in l4.connector.take_events() {
        match event {
            CircuitEvent::Connected => {} // outbound circuits only; none yet
            CircuitEvent::DataReceived(data) => {
                let Some(slot) = l4.circuits.iter_mut().flatten().find(|c| c.conn.key == key)
                else {
                    continue;
                };
                let conn = slot.conn;
                for line in slot.asm.push(&data) {
                    let cmd = parse_bytes(&line);
                    // Fill the live NET/ROM routes (incl. INP3 metric) for `Nodes`.
                    let id = console_id.with_routes(crate::netrom_view::snapshot());
                    let resp = dispatch(&cmd, &id, TransportKind::Ax25);
                    let mut reply = resp.body;
                    let mut disconnect = false;
                    match resp.outcome {
                        DispatchOutcome::Continue => {}
                        DispatchOutcome::Disconnect => disconnect = true,
                        DispatchOutcome::ConfigOp(op) => {
                            let (text, reboot) = crate::config_store::handle_op(&op);
                            reply.extend_from_slice(
                                &ax25_node_core::console::service::render_line(
                                    &text,
                                    TransportKind::Ax25,
                                ),
                            );
                            if reboot {
                                REBOOT_PENDING.store(true, core::sync::atomic::Ordering::Relaxed);
                            }
                        }
                        DispatchOutcome::ConnectThenRelay(_call) => {
                            reply.extend_from_slice(
                                b"...onward connects from a NET/ROM circuit aren't wired yet\r",
                            );
                        }
                    }
                    if !disconnect {
                        reply.extend_from_slice(prompt.as_bytes());
                    }
                    if !reply.is_empty() {
                        l4.connector.write(table, &conn, &reply, now_ms());
                    }
                    if disconnect {
                        l4.connector.disconnect(table, &conn, now_ms());
                    }
                }
            }
            CircuitEvent::Closed(_reason) => {
                defmt::info!("axudp: NET/ROM circuit closed");
                for slot in l4.circuits.iter_mut() {
                    if matches!(slot, Some(c) if c.conn.key == key) {
                        *slot = None;
                    }
                }
            }
        }
    }

    for send in l4.connector.take_interlink_sends() {
        if let Some(i) = find_peer(peers, &send.neighbour) {
            queue.push_back((i, Event::DlDataRequest(PID_NETROM, send.datagram)));
        } else {
            let mut name = [0u8; 16];
            defmt::warn!(
                "axudp: no L2 session to interlink neighbour {=str}, dropping datagram",
                call_str(&send.neighbour, &mut name)
            );
        }
    }
}

/// Proactively (re)establish an L2 link to every known NET/ROM neighbour we
/// have a heard endpoint for and no live session with. Each link is an
/// [`Role::Interlink`]; the connector ships its L4 datagrams over them. A
/// neighbour with no session was either never up or was torn down — either way
/// we re-SABM it here (the periodic cadence is the reconnect backoff).
#[allow(clippy::too_many_arguments)]
async fn ensure_interlinks(
    sessions: &mut session::Sessions,
    peers: &mut [Option<PeerState>; session::MAX_SESSIONS],
    socket: &UdpSocket<'_>,
    heard: &[Option<(Callsign, IpEndpoint)>; 8],
    beacon_ep: Option<IpEndpoint>,
    my_call: Callsign,
    netrom: &mut session::NetRom,
    connector: &mut NetRomConnector,
    circuits: &mut [Option<CircuitConsole>; 4],
    mut inp3: Option<&mut Inp3Host>,
    console_id: &Identity,
    prompt: &str,
) {
    // Collect neighbour callsigns (can't borrow the table across the connect).
    let mut neighbours = heapless::Vec::<Callsign, 16>::new();
    netrom.for_each_neighbour(|n| {
        let _ = neighbours.push(n.neighbour);
    });

    for nbr in neighbours {
        if nbr == my_call || find_peer(peers, &nbr).is_some() {
            continue; // ourselves, or already linked
        }
        if heard_lookup(heard, &nbr).is_none() {
            continue; // no endpoint to reach it — wait until we hear it
        }
        match start_outbound(peers, heard, beacon_ep, nbr, my_call, Role::Interlink) {
            Ok(i) => {
                let mut name = [0u8; 16];
                defmt::info!(
                    "axudp: bringing up interlink to {=str}",
                    call_str(&nbr, &mut name)
                );
                drive(
                    sessions,
                    peers,
                    socket,
                    heard,
                    beacon_ep,
                    my_call,
                    &mut L4 {
                        connector,
                        netrom,
                        circuits,
                        inp3: inp3.as_deref_mut(),
                    },
                    i,
                    Event::DlConnectRequest,
                    console_id,
                    prompt,
                )
                .await;
            }
            Err(_) => {} // busy/no-slot — fine, try again next pass
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
    peer_is_node: bool,
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
                    if peer_is_node {
                        // A known NET/ROM node connecting = an interlink (it
                        // will speak PID 0xCF). No console, no banner — a 0xF0
                        // banner at a node's interlink is garbage to it.
                        defmt::info!(
                            "axudp: interlink L2 up from node {=str}",
                            call_str(&peer, &mut name)
                        );
                        ps.role = Role::Interlink;
                    } else {
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
                    Role::Interlink => {
                        let mut name = [0u8; 16];
                        defmt::info!(
                            "axudp: interlink to {=str} established",
                            call_str(&peer, &mut name)
                        );
                    }
                    _ => {}
                },
                DataLinkSignal::DataIndication(pid, info) if pid == PID_NETROM => {
                    // An interlink datagram (NET/ROM L3/L4) — never console
                    // text. Routed to the connector by drive().
                    followups.push(FollowUp::NetRom {
                        neighbour: peer,
                        datagram: info,
                    });
                }
                DataLinkSignal::DataIndication(_pid, info) => match &mut ps.role {
                    Role::Console(asm) => {
                        let lines = asm.push(&info);
                        for line in lines {
                            let cmd = parse_bytes(&line);
                            // Fill the live NET/ROM routes (incl. INP3 metric) for `Nodes`.
                            let id = console_id.with_routes(crate::netrom_view::snapshot());
                            let resp = dispatch(&cmd, &id, TransportKind::Ax25);
                            let mut reply = resp.body;
                            let mut disconnect = false;
                            let mut bridging = false;
                            match resp.outcome {
                                DispatchOutcome::Continue => {}
                                DispatchOutcome::Disconnect => disconnect = true,
                                DispatchOutcome::ConfigOp(op) => {
                                    let (text, reboot) = crate::config_store::handle_op(&op);
                                    reply.extend_from_slice(
                                        &ax25_node_core::console::service::render_line(
                                            &text,
                                            TransportKind::Ax25,
                                        ),
                                    );
                                    if reboot {
                                        // The reset fires after this batch of
                                        // frames is sent (drive() checks).
                                        REBOOT_PENDING
                                            .store(true, core::sync::atomic::Ordering::Relaxed);
                                    }
                                }
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
                    Role::None | Role::Interlink => {}
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
