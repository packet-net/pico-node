//! The node: everything above the modem, for every radio port.
//!
//! One task owns the connected-mode session layer, the node console, NET/ROM
//! (one routing table for the whole node, NODES heard and originated, L4
//! circuits, interlinks, INP3) and the telnet connect relay. The port drivers
//! ([`crate::ports`]) own the modems: they deliver heard frames into
//! [`ports::RX`] and transmit what this task queues with [`ports::send`].
//!
//! Each connected peer is reached on a [`Port`] and carries a [`Role`] deciding
//! where its DL signals go:
//!
//! - [`Role::Console`]: an inbound user at the node prompt
//!   (`TransportKind::Ax25`, CR line discipline).
//! - [`Role::Bridge`]: piped to *another AX.25 session*: a console user typed
//!   `C <call>` and this task connected onward, relaying I-frame data both
//!   ways (node-hopping *through* the Pico).
//! - [`Role::TelnetRelay`]: piped to the telnet console relay
//!   ([`crate::admin::relay`] statics).
//! - [`Role::Interlink`]: a persistent link to a NET/ROM neighbour.
//!
//! Cross-peer work (bridge data forwarding, bridge teardown notices) is queued
//! as [`FollowUp`]s and drained by [`drive`], one borrow at a time, bounded.
//!
//! **Timers are live**: each peer carries its own [`session::EmbassyTimers`],
//! the main select loop wakes at the earliest armed deadline across all peers,
//! and expiries post the matching `Event::T?Expiry` into that peer's session:
//! retransmission, ack timing and dead-peer link failure (N2 exhausted, then
//! teardown) run exactly as the SDL tables specify.
//!
//! **NET/ROM L4 circuits terminate here too**: inbound PID-0xCF I-frames are
//! interlink datagrams, fed to a [`NetRomConnector`] (the host-tested sans-io
//! L4 stack). Circuits addressed to this node are auto-accepted and get the
//! node console attached, so `C <alias>` from any NET/ROM neighbour lands at
//! the same prompt L2 users get; the connector's outbound datagrams ride back
//! as PID-0xCF I-frames over the neighbour's L2 session.
//!
//! Until 2026-09-25 this task was the AXUDP transport (AX.25 over UDP, the
//! bring-up path before a TNC was attached); the radio-first restructure
//! removed AXUDP and made the radio ports its only links.

use ax25_node_core::ax25::{Callsign, Frame, PID_NETROM, PID_NO_LAYER3};
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
    CircuitEvent, CircuitKey, NetRomCircuitCloseReason, NetRomConnection, NetRomConnector,
    NetRomConnectorOptions, NetRomOriginator, NetRomOriginatorOptions,
};
use ax25_node_core::netrom::ObserveOutcome;
use ax25_node_core::sdl::{
    classify_incoming, DataLinkSignal, Event, FrameSpec, UnnumberedKind, WireSink,
};

use alloc::collections::VecDeque;
use alloc::string::String;
use alloc::vec::Vec;

use embassy_futures::select::{select, select3, select4, Either, Either3, Either4};
use embassy_time::{Duration, Instant, Ticker, Timer};

use crate::config::NetRomConfig;
use crate::session;
use crate::admin::relay::{self, RelayStatus};
use crate::ports::{self, call_str, ui_frame, Port};

/// Seconds between housekeeping ticks (route-count display, interlinks, flash
/// save, L4 timers, INP3, the obsolescence sweep and NODES each check their own
/// cadence on every tick).
const HOUSEKEEPING_TICK_SECS: u64 = 10;
/// Seconds between routing-table flash saves (bounds wear; only saves on change).
const NETROM_SAVE_SECS: u64 = 300;
/// Seconds between "ensure interlinks" passes — proactively (re)establish an
/// L2 link to every known NET/ROM neighbour we can reach, BPQ-style.
const INTERLINK_ENSURE_SECS: u64 = 30;

/// Master gate for the INP3 time-routing overlay on this firmware host (the
/// analogue of the C# `config.Inp3.Enabled` / the ax25-ts connector `inp3` opt-in).
/// Default ON; build with `INP3_DISABLE` set to leave it out. Kept a single const
/// here so the whole overlay (engine + scheduler construction, the inbound 0xCF tap,
/// the tick fan-out) is trivially gateable: when `false` the [`Inp3Host`] is never
/// constructed and every INP3 step is a no-op.
const INP3_ENABLED: bool = option_env!("INP3_DISABLE").is_none();

/// INP3 cadences: packet.net's production defaults (`NetRomInp3Options`). Every
/// probe and RIF goes on the air now, so these are the real-network values, not
/// the old seconds-scale lab-demo ones. L3RTT probe every 60 s; periodic full RIF
/// every 300 s; reflection-timeout reset at 180 s. Encoded as ms (the netrom
/// no_std idiom).
const INP3_L3RTT_INTERVAL_MS: u32 = 60_000;
const INP3_RIF_INTERVAL_MS: u32 = 300_000;
const INP3_RESET_WINDOW_MS: u32 = 180_000;
/// The positive-update debounce: must be > 0 and < the RIF interval.
const INP3_POSITIVE_DEBOUNCE_MS: u32 = 5_000;

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
    /// Cross-connected to the `other` leg (another AX.25 session, or a NET/ROM
    /// circuit when the target was reached through the network). `initiator`
    /// marks the console-user side of the pair (the peer who typed `C`); the
    /// target side carries `initiator: false`. The distinction matters at
    /// teardown: a surviving initiator gets a notice + its console back, a
    /// surviving target just gets disconnected (its user is gone) — getting
    /// this wrong console-attaches to the REMOTE NODE, and two node consoles
    /// answering each other's prompts is a perfect I-frame echo loop (observed
    /// live: "Invalid command" ↔ "Unknown command" at 2.5 Hz until BPQ DISCed).
    Bridge { other: Leg, initiator: bool },
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
    /// The port the peer is reached on.
    port: Port,
    role: Role,
}

/// One side of a cross-connect: an AX.25 session (by the peer's callsign) or a
/// NET/ROM L4 circuit (by its key). A user's leg and the far side can each be
/// either, so a user on the air, a user arriving by NET/ROM circuit, and the
/// telnet relay can all connect onward directly or through the network.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Leg {
    Peer(Callsign),
    Circuit(CircuitKey),
}

/// How many NET/ROM circuits the node keeps attached at once (inbound console
/// users, onward connects, telnet relays). The connector itself allows 16.
const MAX_CIRCUITS: usize = 8;

/// What a NET/ROM L4 circuit is attached to.
enum CircuitMode {
    /// An inbound circuit at the node console prompt.
    Console(LineAssembler),
    /// One side of a cross-connect (see [`Role::Bridge`]); `up` once the
    /// circuit has connected, so a failure before that reads as "no answer".
    Bridge {
        other: Leg,
        initiator: bool,
        up: bool,
    },
    /// The telnet user's onward connect, reached through the network.
    TelnetRelay { up: bool },
}

/// A NET/ROM circuit the node has attached to something.
struct CircuitSlot {
    conn: NetRomConnection,
    mode: CircuitMode,
}

/// Cross-leg work discovered while servicing one session or circuit, applied by
/// [`drive_all`] once that borrow is released.
enum FollowUp {
    /// A console user (on either kind of leg) asked to connect onward.
    StartBridge { console: Leg, target: Callsign },
    /// Relay data to the other end of a cross-connect.
    Forward { to: Leg, data: Vec<u8> },
    /// One side of a cross-connect ended, with a note for a surviving user. A
    /// console user gets the note and its prompt back; a target is torn down.
    BridgeEnded { survivor: Leg, note: &'static str },
    /// An inbound PID-0xCF interlink datagram for the NET/ROM connector.
    NetRom {
        neighbour: Callsign,
        datagram: Vec<u8>,
    },
    /// The telnet console asked to connect to `target`.
    StartRelay { target: Callsign },
    /// Telnet-user bytes for the relay's far end.
    RelayData(Vec<u8>),
    /// The telnet user went away: end the relay's far end.
    RelayHangup,
    /// A circuit needs an interlink to `neighbour` that is not up: dial it.
    DialInterlink { neighbour: Callsign },
}

#[embassy_executor::task]
pub async fn task(
    netrom_cfg: NetRomConfig,
    my_call: Callsign,
    console_id: Identity,
    prompt: String,
) {
    defmt::info!("node: up, serving {=usize} radio port(s)", ports::PORT_COUNT);

    // The read-only NET/ROM tap (the C# FrameTraced-before-DispatchInbound
    // equivalent): fed every decoded inbound frame BEFORE address filtering.
    let mut netrom = session::new_netrom();

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
    // convention; the first broadcast goes out on the first housekeeping tick so a
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
    let mut circuits: [Option<CircuitSlot>; MAX_CIRCUITS] = [const { None }; MAX_CIRCUITS];

    // INP3 time-routing overlay (the embedded host wiring — the analogue of the C#
    // `NetRomService.Inp3Host` + the ax25-ts connector `inp3` glue). Constructed once
    // here, only when the overlay is on, with the node call already known (the C# sets
    // the engine's local node at AttachPort; we set it at construction, since `my_call`
    // is a parameter). Quality forwarding is left primary
    // (`set_prefer_inp3_routes(false)`) so the node learns time-routes without yet
    // routing by them (AWARENESS ONLY, matching the C#/TS slice). `None` when off:
    // the engine/scheduler are then never even constructed.
    let mut inp3: Option<Inp3Host> = if INP3_ENABLED {
        connector.set_prefer_inp3_routes(false);
        let host = Inp3Host::new(my_call);
        defmt::info!(
            "node: INP3 overlay constructed (probe {=u32}ms, rif {=u32}ms, reset {=u32}ms), awareness-only, quality forwarding unchanged",
            INP3_L3RTT_INTERVAL_MS,
            INP3_RIF_INTERVAL_MS,
            INP3_RESET_WINDOW_MS
        );
        Some(host)
    } else {
        defmt::info!("node: INP3 overlay disabled");
        None
    };

    let nodes_interval = Duration::from_secs(netrom_cfg.nodes_interval_secs as u64);
    let mut next_nodes_at = Instant::now(); // announce on the first tick
    if netrom_cfg.originate {
        defmt::info!(
            "node: NODES origination on, every {=u32}s",
            netrom_cfg.nodes_interval_secs
        );
    }
    // Obsolescence sweep cadence: age/purge the routing table once per NODES
    // interval (the C# `NetRomService.OnInterval` sweep — see NetRomService.cs).
    // Runs whether or not WE originate: obsolescence aging is about the table, and
    // OBSINIT is calibrated to one broadcast period per decrement. First sweep after
    // one interval (never age a freshly booted/flash-restored table immediately).
    let mut next_sweep_at = Instant::now() + nodes_interval;

    // The connected-mode session layer for this port + per-peer link state.
    let mut sessions = session::new_sessions(my_call);
    let mut peers: [Option<PeerState>; session::MAX_SESSIONS] =
        [const { None }; session::MAX_SESSIONS];

    // Callsign -> the port it was last heard on (where to dial it; a station
    // not in here is dialled on the first usable port).
    let mut heard: [Option<(Callsign, Port)>; 8] = [None; 8];

    let mut ticker = Ticker::every(Duration::from_secs(HOUSEKEEPING_TICK_SECS));
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
            .any(|p| matches!(p.role, Role::TelnetRelay))
            || circuits
                .iter()
                .flatten()
                .any(|c| matches!(c.mode, CircuitMode::TelnetRelay { .. }));
        // Peer bytes waiting for a slow telnet user are fed on as it drains,
        // including after the link has ended (so the tail still arrives).
        let relay_fut = async {
            if telnet_relay_active {
                let mut buf = [0u8; 128];
                match select3(
                    relay::USER_TO_AX.read(&mut buf),
                    relay::USER_HANGUP.wait(),
                    relay::flush_some(),
                )
                .await
                {
                    Either3::First(n) => RelayEvent::UserData(buf, n),
                    Either3::Second(()) => RelayEvent::Hangup,
                    Either3::Third(()) => RelayEvent::Flushed,
                }
            } else {
                match select(relay::CONNECT_REQ.receive(), relay::flush_some()).await {
                    Either::First(target) => RelayEvent::Connect(target),
                    Either::Second(()) => RelayEvent::Flushed,
                }
            }
        };

        // Heard frames arrive from the port drivers (ports::RX), tagged with
        // their port, and are handled after the match.
        let woke = select4(ticker.next(), ports::RX.receive(), timer_wait, relay_fut).await;
        let mut received: Option<(Frame, Port)> = None;
        match woke {
            Either4::Second((port, frame)) => received = Some((frame, port)),
            Either4::First(()) => {
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
                        &heard,
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

                // L4 circuit timers (ack/retransmit/idle) ride the housekeeping tick.
                connector.tick(netrom.table(), now_ms());
                drive_all(
                    &mut sessions,
                    &mut peers,
                    &heard,
                    my_call,
                    &mut L4 {
                        connector: &mut connector,
                        netrom: &mut netrom,
                        circuits: &mut circuits,
                        inp3: inp3.as_mut(),
                    },
                    None,
                    Vec::new(),
                    &console_id,
                    &prompt,
                )
                .await;

                // INP3 time-routing overlay tick (the embedded host wiring). Rides the
                // housekeeping tick: the single driver, the core owns no ambient timer (as the
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
                                &heard,
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
                                "node: INP3 frame dropped, no interlink up to {=str} (drop, don't dial)",
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
                            "node: INP3 neighbour {=str} down (silent {=u64}ms), engine state reset (no table teardown wired)",
                            call_str(&down.neighbour, &mut name),
                            down.silent_for_ms
                        );
                    }
                }

                // Obsolescence sweep — age/purge routes once per NODES interval,
                // BEFORE origination so a broadcast advertises the freshly-aged
                // table (the C# `NetRomService.OnInterval` order: Sweep() then
                // BroadcastNodes()). Drives the correctness path the recon flagged:
                // without this, obsolescence never ages, dead routes never purge,
                // and the OBSMIN advertise-gate never engages.
                if Instant::now() >= next_sweep_at {
                    next_sweep_at = Instant::now() + nodes_interval;
                    let purged = netrom.sweep();
                    if purged > 0 {
                        defmt::info!(
                            "netrom: obsolescence sweep purged {=usize} stale route(s)",
                            purged
                        );
                    }
                }

                // NODES origination rides the housekeeping tick (10 s granularity is
                // plenty against minutes-scale intervals).
                if netrom_cfg.originate && Instant::now() >= next_nodes_at {
                    next_nodes_at = Instant::now() + nodes_interval;
                    let payloads = originator.broadcast_nodes(netrom.table());
                    // On every usable port (BPQ: NODES on each port set to
                    // broadcast them).
                    let dest = NetRomOriginator::nodes_destination();
                    for payload in &payloads {
                        let frame = ui_frame(my_call, dest, NetRomOriginator::PID, payload);
                        for port in Port::ALL {
                            ports::send(port, frame.encode()).await;
                        }
                    }
                    defmt::info!("node: NODES broadcast sent ({=usize} frame(s))", payloads.len());
                }
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
                        defmt::debug!("node: timer expiry ({=u8})", id as u8);
                        let Some(event) = session::expiry_event(id) else {
                            // TM201 (the MDL XID retry timer): not a data-link
                            // event - the manager drives the peer's MDL machine,
                            // which retries the XID command or, on give-up, fires
                            // a probe's deferred SABM.
                            let (frames, ep) = {
                                let Some(ps) = peers[i].as_mut() else {
                                    break;
                                };
                                let peer = ps.peer;
                                let ep = ps.port;
                                (sessions.tm201_expiry(peer, &mut ps.timers), ep)
                            };
                            send_all(ep, frames).await;
                            continue;
                        };
                        drive(
                            &mut sessions,
                            &mut peers,
                            &heard,
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
                        if peers[i].is_none() {
                            break; // expiry tore the session down
                        }
                    }
                }
            }
            Either4::Fourth(ev) => match ev {
                RelayEvent::Flushed => {}
                ev => {
                    let work = match ev {
                        RelayEvent::Connect(target) => FollowUp::StartRelay { target },
                        RelayEvent::UserData(buf, n) => FollowUp::RelayData(buf[..n].to_vec()),
                        _ => FollowUp::RelayHangup,
                    };
                    drive_all(
                        &mut sessions,
                        &mut peers,
                        &heard,
                        my_call,
                        &mut L4 {
                            connector: &mut connector,
                            netrom: &mut netrom,
                            circuits: &mut circuits,
                            inp3: inp3.as_mut(),
                        },
                        None,
                        alloc::vec![work],
                        &console_id,
                        &prompt,
                    )
                    .await;
                }
            },
        }

        let Some((frame, link)) = received else {
            continue;
        };

                // READ-ONLY NET/ROM TAP: every frame, BEFORE the address filter,
                // so NODES broadcasts (addressed to "NODES", not us) are heard.
                let outcome =
                    session::observe_inbound(&mut netrom, &frame, my_call, link.netrom_id());
                if let ObserveOutcome::Ingested { .. } = outcome {
                    defmt::info!(
                        "node: NODES broadcast ingested ({=u32} destinations known)",
                        netrom.destination_count() as u32
                    );
                    crate::mqtt::log("NODES broadcast ingested");
                }

                defmt::info!(
                    "node: rx {=str} -> {=str} ctl={=u8:#04x} info={=usize}B on {=str}",
                    call_str(&frame.source.callsign, &mut src_buf),
                    call_str(&frame.destination.callsign, &mut dst_buf),
                    frame.control,
                    frame.info.len(),
                    link.name()
                );
                if frame.is_ui() && !frame.info.is_empty() {
                    if let Ok(text) = core::str::from_utf8(&frame.info) {
                        defmt::info!("node: rx UI text: {=str}", text);
                    }
                }

                heard_update(&mut heard, frame.source.callsign, link);

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
                    // event (classify_incoming returns None; the tables carry
                    // only the initiator MDL). Detect it by control byte (0xAF
                    // + optional P/F) and answer like a v2.0 station: DM, so the
                    // peer (BPQ does) falls back to a plain SABM. Only when no
                    // session is up; a mid-session XID is ignored like any
                    // other unclassified frame.
                    const XID: u8 = 0xAF;
                    if frame.control & !0x10 == XID && sessions.session_for(&peer).is_none() {
                        defmt::info!("node: XID received, answering DM (v2.0 fallback)");
                        let sink = WireSink::new(my_call, peer, alloc::vec::Vec::new());
                        let dm = sink.build_frame(&FrameSpec::Unnumbered {
                            kind: UnnumberedKind::Dm,
                            is_command: false,
                            pf: (frame.control & 0x10) != 0,
                            expedited: false,
                        });
                        send_all(link, alloc::vec![dm.encode()]).await;
                        continue;
                    }

                    let Some(event) = classify_incoming(&frame) else {
                        continue;
                    };
                    let Some(i) = peer_slot(&mut peers, peer, my_call, link) else {
                        defmt::warn!("node: peer table full, dropping session frame");
                        continue;
                    };
                    // The link's port is pinned at slot creation (inbound: the
                    // port the SABM arrived on; outbound: where the target was
                    // heard, or the first usable port).
                    drive(
                        &mut sessions,
                        &mut peers,
                        &heard,
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

                    // If that inbound frame was an INP3 L3RTT *probe*, the engine has
                    // queued our reflection; ship it NOW (same event-loop turn), not on
                    // the next 10 s housekeeping tick: the peer times our reflection to measure
                    // its SNTT to us, so a tick-deferred reflection would inflate it by up
                    // to HOUSEKEEPING_TICK_SECS. (Originated probes + RIFs stay on the tick;
                    // only the reflection is latency-critical.) The inbound `drive` has
                    // returned, so `inp3`/`peers`/`sessions` are free to reborrow here.
                    let reflections = match inp3.as_mut() {
                        Some(host) => host.take_outbound_l3rtt(),
                        None => Vec::new(),
                    };
                    for (nbr, bytes) in reflections {
                        if let Some(j) = find_peer(&peers, &nbr) {
                            drive(
                                &mut sessions,
                                &mut peers,
                                &heard,
                                my_call,
                                &mut L4 {
                                    connector: &mut connector,
                                    netrom: &mut netrom,
                                    circuits: &mut circuits,
                                    inp3: inp3.as_mut(),
                                },
                                j,
                                Event::DlDataRequest(PID_NETROM, bytes),
                                &console_id,
                                &prompt,
                            )
                            .await;
                        }
                    }
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
    /// Some held-back peer bytes reached the telnet user; nothing else to do.
    Flushed,
}

/// Drive one event into `peers[start]`'s session, then everything it leads to.
#[allow(clippy::too_many_arguments)]
async fn drive(
    sessions: &mut session::Sessions,
    peers: &mut [Option<PeerState>; session::MAX_SESSIONS],
    heard: &[Option<(Callsign, Port)>; 8],
    my_call: Callsign,
    l4: &mut L4<'_>,
    start: usize,
    event: Event,
    console_id: &Identity,
    prompt: &str,
) {
    drive_all(
        sessions,
        peers,
        heard,
        my_call,
        l4,
        Some((start, event)),
        Vec::new(),
        console_id,
        prompt,
    )
    .await;
}

/// The node's work loop: post session events (starting with `start`, if any),
/// apply the cross-leg [`FollowUp`]s they and `initial` produce, and service
/// the NET/ROM circuits, until nothing is left. Bounded by `guard`.
#[allow(clippy::too_many_arguments)]
async fn drive_all(
    sessions: &mut session::Sessions,
    peers: &mut [Option<PeerState>; session::MAX_SESSIONS],
    heard: &[Option<(Callsign, Port)>; 8],
    my_call: Callsign,
    l4: &mut L4<'_>,
    start: Option<(usize, Event)>,
    initial: Vec<FollowUp>,
    console_id: &Identity,
    prompt: &str,
) {
    let mut queue: VecDeque<(usize, Event)> = VecDeque::new();
    queue.extend(start);
    let mut pending: VecDeque<FollowUp> = initial.into_iter().collect();
    let mut guard = 0u32;

    loop {
        guard += 1;
        if guard > 64 {
            defmt::warn!("node: drive guard tripped, dropping remaining work");
            break;
        }
        while let Some(f) = pending.pop_front() {
            apply_followup(f, peers, heard, my_call, l4, &mut queue, prompt);
        }
        // L4 housekeeping every round: attach consoles to fresh circuits,
        // service circuit events, and ship the connector's outbound interlink
        // datagrams over the right L2 sessions.
        service_l4(l4, peers, &mut queue, &mut pending, console_id, prompt);
        if !pending.is_empty() {
            continue;
        }
        let Some((i, ev)) = queue.pop_front() else {
            break;
        };
        let Some(ps) = peers[i].as_mut() else {
            continue;
        };
        let peer_is_node = {
            let t = l4.netrom.table();
            t.neighbour(&ps.peer).is_some() || t.destination(&ps.peer).is_some()
        };
        let (frames, followups) = post_one(sessions, ps, ev, console_id, prompt, peer_is_node);
        let ep = ps.port;
        send_all(ep, frames).await;
        if REBOOT_PENDING.load(core::sync::atomic::Ordering::Relaxed) {
            // Console REBOOT: give the wire a beat to drain, then reset.
            Timer::after_millis(250).await;
            cortex_m::peripheral::SCB::sys_reset();
        }
        reap(sessions, peers, i);
        pending.extend(followups);
    }
}

/// Send `bytes` to a leg: into an AX.25 session as stream data, or down a
/// NET/ROM circuit.
fn tell(
    leg: Leg,
    bytes: Vec<u8>,
    peers: &[Option<PeerState>],
    l4: &mut L4<'_>,
    queue: &mut VecDeque<(usize, Event)>,
) {
    match leg {
        Leg::Peer(call) => {
            if let Some(i) = find_peer(peers, &call) {
                queue.push_back((i, Event::DlDataRequest(PID_NO_LAYER3, bytes)));
            }
        }
        Leg::Circuit(key) => {
            if let Some(conn) = l4.circuits.iter().flatten().find(|c| c.conn.key == key).map(|c| c.conn) {
                l4.connector.write(l4.netrom.table(), &conn, &bytes, now_ms());
            }
        }
    }
}

/// Point a user leg at its far side once an onward connect has started.
fn attach_initiator(leg: Leg, far: Leg, peers: &mut [Option<PeerState>], l4: &mut L4<'_>) {
    match leg {
        Leg::Peer(call) => {
            if let Some(ps) = find_peer_mut(peers, &call) {
                ps.role = Role::Bridge {
                    other: far,
                    initiator: true,
                };
            }
        }
        Leg::Circuit(key) => {
            if let Some(slot) = l4.circuits.iter_mut().flatten().find(|c| c.conn.key == key) {
                slot.mode = CircuitMode::Bridge {
                    other: far,
                    initiator: true,
                    up: true,
                };
            }
        }
    }
}

/// Try to reach `target` through the network: a NET/ROM L4 circuit to it via the
/// best neighbour, attached as `mode`. `None` when the routing table has no
/// route (or no circuit slot is free): the caller falls back to a direct AX.25
/// connect, as packet.net's `NetRomOutboundConnector` does.
fn open_circuit(l4: &mut L4<'_>, target: Callsign, user: Callsign, mode: CircuitMode) -> Option<CircuitKey> {
    let free = l4.circuits.iter().position(|c| c.is_none())?;
    let mut text = [0u8; 16];
    let target_text = call_str(&target, &mut text);
    let conn = l4
        .connector
        .connect(l4.netrom.table(), target_text, user, now_ms())
        .ok()?;
    let mut name = [0u8; 16];
    defmt::info!(
        "node: onward connect to {=str} through NET/ROM",
        call_str(&target, &mut name)
    );
    l4.circuits[free] = Some(CircuitSlot { conn, mode });
    Some(conn.key)
}

/// Apply one cross-leg [`FollowUp`].
fn apply_followup(
    f: FollowUp,
    peers: &mut [Option<PeerState>; session::MAX_SESSIONS],
    heard: &[Option<(Callsign, Port)>; 8],
    my_call: Callsign,
    l4: &mut L4<'_>,
    queue: &mut VecDeque<(usize, Event)>,
    prompt: &str,
) {
    match f {
        FollowUp::StartBridge { console, target } => {
            // The originating user a circuit carries: the console user's call,
            // or for a user who arrived by circuit, the user that circuit was
            // opened for (not the node it came through), so the far end sees
            // who is really connecting.
            let user = match console {
                Leg::Peer(call) => call,
                Leg::Circuit(key) => l4
                    .circuits
                    .iter()
                    .flatten()
                    .find(|c| c.conn.key == key)
                    .map(|c| c.conn.user)
                    .unwrap_or(my_call),
            };
            let routed = open_circuit(
                l4,
                target,
                user,
                CircuitMode::Bridge {
                    other: console,
                    initiator: false,
                    up: false,
                },
            );
            if let Some(key) = routed {
                attach_initiator(console, Leg::Circuit(key), peers, l4);
                return;
            }
            // No NET/ROM route: a direct AX.25 connect, on the port the target
            // was heard on (else the first usable port).
            match start_outbound(
                peers,
                heard,
                target,
                // The node's own call, the convention real nodes use for
                // outgoing links (a per-user call trips BPQ's node-link
                // heuristics).
                my_call,
                Role::Bridge {
                    other: console,
                    initiator: false,
                },
            ) {
                Ok(ti) => {
                    attach_initiator(console, Leg::Peer(target), peers, l4);
                    queue.push_back((ti, Event::DlConnectRequest));
                }
                Err(reason) => {
                    let mut msg = Vec::from(b"Failure: ".as_slice());
                    msg.extend_from_slice(reason.as_bytes());
                    msg.extend_from_slice(b"\r");
                    msg.extend_from_slice(prompt.as_bytes());
                    tell(console, msg, peers, l4, queue);
                }
            }
        }
        FollowUp::Forward { to, data } => tell(to, data, peers, l4, queue),
        FollowUp::NetRom {
            neighbour,
            datagram,
        } => {
            // INP3 peel BEFORE the connector (mirrors the C# `DispatchInp3` /
            // the ax25-ts `dispatchInp3` precedence): a RIF (0xFF-led) or an
            // L3RTT is consumed here so it can never reach L4 circuits /
            // forwarding. `true` means consumed; fall through to the connector
            // only on `false`. Disjoint-field borrows of `*l4` (inp3 + netrom).
            let consumed = match l4.inp3.as_deref_mut() {
                Some(host) => host.dispatch_inbound(neighbour, &datagram, l4.netrom, my_call, now_ms()),
                None => false,
            };
            if !consumed {
                l4.connector
                    .on_interlink_data(l4.netrom.table(), neighbour, &datagram, now_ms());
            }
        }
        FollowUp::BridgeEnded { survivor, note } => match survivor {
            Leg::Peer(call) => {
                if let Some(si) = find_peer(peers, &call) {
                    let sp = peers[si].as_mut().expect("present");
                    match sp.role {
                        Role::Bridge {
                            initiator: true, ..
                        } => {
                            // The console user survives: the note + its prompt back.
                            sp.role = Role::Console(LineAssembler::default());
                            let mut msg = Vec::from(note.as_bytes());
                            msg.extend_from_slice(b"\r");
                            msg.extend_from_slice(prompt.as_bytes());
                            queue.push_back((si, Event::DlDataRequest(PID_NO_LAYER3, msg)));
                        }
                        _ => {
                            // The target survives but its user is gone (or the
                            // survivor is in an unexpected role): tear the link
                            // down, NEVER console-attach to it (two node consoles
                            // answering each other is an I-frame echo loop).
                            sp.role = Role::None;
                            queue.push_back((si, Event::DlDisconnectRequest));
                        }
                    }
                }
            }
            Leg::Circuit(key) => {
                let Some(slot) = l4.circuits.iter_mut().flatten().find(|c| c.conn.key == key) else {
                    return;
                };
                let conn = slot.conn;
                if matches!(slot.mode, CircuitMode::Bridge { initiator: true, .. }) {
                    // A user who arrived by circuit gets the note + its prompt back.
                    slot.mode = CircuitMode::Console(LineAssembler::default());
                    let mut msg = Vec::from(note.as_bytes());
                    msg.extend_from_slice(b"\r");
                    msg.extend_from_slice(prompt.as_bytes());
                    l4.connector.write(l4.netrom.table(), &conn, &msg, now_ms());
                } else {
                    // A circuit that was the far side: its user is gone.
                    l4.connector.disconnect(l4.netrom.table(), &conn, now_ms());
                }
            }
        },
        FollowUp::StartRelay { target } => {
            if open_circuit(l4, target, my_call, CircuitMode::TelnetRelay { up: false }).is_some() {
                return; // Connected / Failed is signalled from the circuit's events
            }
            match start_outbound(peers, heard, target, my_call, Role::TelnetRelay) {
                Ok(i) => queue.push_back((i, Event::DlConnectRequest)),
                Err(reason) => relay::STATUS.signal(RelayStatus::Failed(reason)),
            }
        }
        FollowUp::RelayData(data) => {
            let circuit = l4
                .circuits
                .iter()
                .flatten()
                .find(|c| matches!(c.mode, CircuitMode::TelnetRelay { .. }))
                .map(|c| c.conn);
            if let Some(conn) = circuit {
                l4.connector.write(l4.netrom.table(), &conn, &data, now_ms());
            } else if let Some(i) = find_role(peers, |r| matches!(r, Role::TelnetRelay)) {
                queue.push_back((i, Event::DlDataRequest(PID_NO_LAYER3, data)));
            }
        }
        FollowUp::RelayHangup => {
            let circuit = l4
                .circuits
                .iter()
                .flatten()
                .find(|c| matches!(c.mode, CircuitMode::TelnetRelay { .. }))
                .map(|c| c.conn);
            if let Some(conn) = circuit {
                l4.connector.disconnect(l4.netrom.table(), &conn, now_ms());
            } else if let Some(i) = find_role(peers, |r| matches!(r, Role::TelnetRelay)) {
                peers[i].as_mut().expect("present").role = Role::None;
                queue.push_back((i, Event::DlDisconnectRequest));
            }
        }
        FollowUp::DialInterlink { neighbour } => {
            if find_peer(peers, &neighbour).is_some() {
                return;
            }
            if let Ok(i) = start_outbound(peers, heard, neighbour, my_call, Role::Interlink) {
                let mut name = [0u8; 16];
                defmt::info!(
                    "node: dialling interlink to {=str} for a circuit",
                    call_str(&neighbour, &mut name)
                );
                queue.push_back((i, Event::DlConnectRequest));
            }
        }
    }
}

/// The note a user sees when the far side of its connect ended.
fn close_note(reason: NetRomCircuitCloseReason, was_up: bool) -> &'static str {
    match reason {
        NetRomCircuitCloseReason::Normal => "*** Disconnected",
        NetRomCircuitCloseReason::Refused => "*** Busy or refused",
        NetRomCircuitCloseReason::Timeout if was_up => "*** Link failure",
        NetRomCircuitCloseReason::Timeout => "*** No answer",
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
    circuits: &'a mut [Option<CircuitSlot>; MAX_CIRCUITS],
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
/// forwarding unchanged, so time-routes are learned, not yet routed
/// by. Driven by the firmware's housekeeping tick (no ambient timer; the core has none),
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
    /// Construct the host with the configured cadences, the node call pinned as the engine's
    /// local node (the L3 origin stamped into probes + the reflection self-test
    /// identity — the C# pins it at AttachPort; we pin it here since `my_call` is
    /// known). Options are validated for parity with the C#/TS constructors; on the
    /// impossible event they don't validate we fall back to the canonical defaults
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
        // Validate for symmetry with the C#/TS resolver; if (impossibly) the configured
        // constants ever fall out of range, log and fall back rather than panic.
        let options = match options.validate() {
            Ok(()) => options,
            Err(reason) => {
                defmt::warn!(
                    "node: INP3 options invalid ({=str}), using defaults",
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
                    "node: INP3 RIF ingested from {=str} ({=usize} RIP(s), {=u32} destinations known)",
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

        // Drain the engine's outbound L3RTT sends — any probes originated by tick()
        // above, plus any reflections not already shipped inline on receipt.
        let l3rtt = self.take_outbound_l3rtt();

        // Drain neighbour-down events. The C# wires these to table.MarkNeighbourDown +
        // a DISC/re-establish; the firmware's `netrom` handle exposes no public
        // mark-neighbour-down / table-mut seam reachable from here (it lives on the
        // routing table, not NetRomService), so we log them and skip the teardown
        // rather than invent one. The engine has already removed the neighbour's INP3 state.
        let downs = self.engine.take_neighbour_down();

        Inp3Round { l3rtt, rifs, downs }
    }

    /// Drain the engine's queued outbound L3RTT frames (probes the engine originated,
    /// plus reflections of peers' probes) as ready-to-ship `(neighbour, bytes)`. Called
    /// both INLINE right after an inbound L3RTT (so a reflection ships within the same
    /// event-loop turn — the peer times our reflection, so deferring it to the next
    /// `tick_round` would inflate its measured SNTT to us by up to one
    /// `HOUSEKEEPING_TICK_SECS`) and from `tick_round` itself (for originated probes).
    /// Idempotent: a no-op `Vec` when the outbox is empty.
    fn take_outbound_l3rtt(&mut self) -> Vec<(Callsign, Vec<u8>)> {
        self.engine
            .take_outbound_l3rtt()
            .into_iter()
            .map(|(nbr, frame)| (nbr, frame.to_bytes()))
            .collect()
    }
}

/// Drain the connector: new inbound circuits get the node console + banner;
/// circuit data runs the console dispatcher; closes detach; outbound interlink
/// datagrams are queued as PID-0xCF I-frames to the neighbour's L2 session.
fn service_l4(
    l4: &mut L4<'_>,
    peers: &[Option<PeerState>],
    queue: &mut VecDeque<(usize, Event)>,
    pending: &mut VecDeque<FollowUp>,
    console_id: &Identity,
    prompt: &str,
) {
    for conn in l4.connector.take_incoming_connections() {
        let mut name = [0u8; 16];
        defmt::info!(
            "node: NET/ROM circuit up from {=str}, attaching console",
            call_str(&conn.peer, &mut name)
        );
        if let Some(slot) = l4.circuits.iter_mut().find(|c| c.is_none()) {
            *slot = Some(CircuitSlot {
                conn,
                mode: CircuitMode::Console(LineAssembler::default()),
            });
            let banner = banner_and_prompt(console_id, prompt, TransportKind::Ax25);
            l4.connector.write(l4.netrom.table(), &conn, &banner, now_ms());
        } else {
            defmt::warn!("node: circuit table full, disconnecting");
            l4.connector.disconnect(l4.netrom.table(), &conn, now_ms());
        }
    }

    for (key, event) in l4.connector.take_events() {
        let Some(index) = l4
            .circuits
            .iter()
            .position(|c| matches!(c, Some(slot) if slot.conn.key == key))
        else {
            continue;
        };
        match event {
            CircuitEvent::Connected => {
                let slot = l4.circuits[index].as_mut().expect("found");
                match &mut slot.mode {
                    CircuitMode::Bridge { up, .. } => *up = true,
                    CircuitMode::TelnetRelay { up } => {
                        *up = true;
                        relay::STATUS.signal(RelayStatus::Connected);
                    }
                    CircuitMode::Console(_) => {}
                }
            }
            CircuitEvent::DataReceived(data) => {
                let slot = l4.circuits[index].as_mut().expect("found");
                let conn = slot.conn;
                match &mut slot.mode {
                    CircuitMode::Bridge { other, .. } => pending.push_back(FollowUp::Forward {
                        to: *other,
                        data,
                    }),
                    CircuitMode::TelnetRelay { .. } => relay::to_user(&data),
                    CircuitMode::Console(asm) => {
                        for line in asm.push(&data) {
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
                                        REBOOT_PENDING
                                            .store(true, core::sync::atomic::Ordering::Relaxed);
                                    }
                                }
                                DispatchOutcome::ConnectThenRelay(target) => {
                                    // "Connecting to X..." is already in reply; the
                                    // onward connect itself is cross-leg work.
                                    bridging = true;
                                    pending.push_back(FollowUp::StartBridge {
                                        console: Leg::Circuit(key),
                                        target,
                                    });
                                }
                            }
                            if !disconnect && !bridging {
                                reply.extend_from_slice(prompt.as_bytes());
                            }
                            if !reply.is_empty() {
                                l4.connector.write(l4.netrom.table(), &conn, &reply, now_ms());
                            }
                            if disconnect {
                                l4.connector.disconnect(l4.netrom.table(), &conn, now_ms());
                            }
                            if bridging {
                                // Lines typed after `C` belong to the far side.
                                break;
                            }
                        }
                    }
                }
            }
            CircuitEvent::Closed(reason) => {
                defmt::info!("node: NET/ROM circuit closed");
                let slot = l4.circuits[index].take().expect("found");
                match slot.mode {
                    CircuitMode::Console(_) => {}
                    CircuitMode::Bridge { other, up, .. } => {
                        pending.push_back(FollowUp::BridgeEnded {
                            survivor: other,
                            note: close_note(reason, up),
                        });
                    }
                    CircuitMode::TelnetRelay { up } => {
                        relay::STATUS.signal(if up {
                            RelayStatus::Disconnected
                        } else {
                            RelayStatus::Failed(close_note(reason, false).trim_start_matches("*** "))
                        });
                    }
                }
            }
        }
    }

    let mut dialled: heapless::Vec<Callsign, 4> = heapless::Vec::new();
    for send in l4.connector.take_interlink_sends() {
        if let Some(i) = find_peer(peers, &send.neighbour) {
            queue.push_back((i, Event::DlDataRequest(PID_NETROM, send.datagram)));
        } else if !dialled.contains(&send.neighbour) {
            // No L2 link to that neighbour yet: dial it now. This datagram is
            // dropped; the circuit layer retransmits once the link is up.
            let _ = dialled.push(send.neighbour);
            pending.push_back(FollowUp::DialInterlink {
                neighbour: send.neighbour,
            });
        }
    }
}

/// Proactively (re)establish an L2 link to every known NET/ROM neighbour we
/// have heard on a port and no live session with. Each link is an
/// [`Role::Interlink`]; the connector ships its L4 datagrams over them. A
/// neighbour with no session was either never up or was torn down — either way
/// we re-SABM it here (the periodic cadence is the reconnect backoff).
#[allow(clippy::too_many_arguments)]
async fn ensure_interlinks(
    sessions: &mut session::Sessions,
    peers: &mut [Option<PeerState>; session::MAX_SESSIONS],
    heard: &[Option<(Callsign, Port)>; 8],
    my_call: Callsign,
    netrom: &mut session::NetRom,
    connector: &mut NetRomConnector,
    circuits: &mut [Option<CircuitSlot>; MAX_CIRCUITS],
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
            continue; // not heard yet: wait until we hear it on a port
        }
        // Err from start_outbound (busy/no-slot) is fine; try again next pass.
        if let Ok(i) = start_outbound(peers, heard, nbr, my_call, Role::Interlink) {
            let mut name = [0u8; 16];
            defmt::info!(
                "node: bringing up interlink to {=str}",
                call_str(&nbr, &mut name)
            );
            drive(
                sessions,
                peers,
                heard,
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
    }
}

/// Create the peer slot + role for an outbound connect to `target`, resolving
/// its port: where it was last heard, else the first usable port.
fn start_outbound(
    peers: &mut [Option<PeerState>; session::MAX_SESSIONS],
    heard: &[Option<(Callsign, Port)>; 8],
    target: Callsign,
    local: Callsign,
    role: Role,
) -> Result<usize, &'static str> {
    if find_peer(peers, &target).is_some() {
        return Err("target is busy (session already up)");
    }
    // Where to dial: the port the target was last heard on; otherwise the first
    // usable port (a station need not have been heard to be called over the air).
    let Some(ep) = heard_lookup(heard, &target).or_else(ports::first_usable) else {
        return Err("no radio port is up");
    };
    let Some(i) = peer_slot(peers, target, local, ep) else {
        return Err("no free session slot");
    };
    let mut name = [0u8; 16];
    let mut lname = [0u8; 16];
    defmt::info!(
        "node: outbound connect to {=str} (as {=str}) at {:?}",
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
    port: Port,
) -> Option<usize> {
    if let Some(i) = find_peer(peers, &peer) {
        return Some(i);
    }
    let free = peers.iter().position(|p| p.is_none())?;
    peers[free] = Some(PeerState {
        peer,
        local,
        timers: session::EmbassyTimers::new(),
        port,
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

/// Queue each wire frame for transmission on `port`.
async fn send_all(port: Port, frames: Vec<Vec<u8>>) {
    for wire in frames {
        ports::send(port, wire).await;
    }
}

/// Record `call -> port` in the heard table (update in place, else first
/// free slot, else overwrite the oldest by rotation).
fn heard_update(heard: &mut [Option<(Callsign, Port)>; 8], call: Callsign, ep: Port) {
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

/// Resolve a callsign to the port it was last heard on.
fn heard_lookup(
    heard: &[Option<(Callsign, Port)>; 8],
    call: &Callsign,
) -> Option<Port> {
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
    // Stream data (console text, bridged or relayed bytes) is framed to the
    // link's N1; NET/ROM datagrams are atomic SDUs and go as they are.
    let mut to_send = match event {
        Event::DlDataRequest(pid, data) if pid != PID_NETROM => {
            sessions.post_stream(local, peer, pid, data, &mut ps.timers)
        }
        other => sessions.post_with_local(local, peer, other, &mut ps.timers),
    };
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
                            "node: interlink L2 up from node {=str}",
                            call_str(&peer, &mut name)
                        );
                        ps.role = Role::Interlink;
                    } else {
                        defmt::info!(
                            "node: AX.25 session up from {=str}, attaching console",
                            call_str(&peer, &mut name)
                        );
                        ps.role = Role::Console(LineAssembler::default());
                        let banner = banner_and_prompt(console_id, prompt, TransportKind::Ax25);
                        to_send.extend(sessions.post_stream(
                            local,
                            peer,
                            PID_NO_LAYER3,
                            banner,
                            &mut ps.timers,
                        ));
                    }
                }
                DataLinkSignal::ConnectConfirm => match ps.role {
                    Role::TelnetRelay => {
                        defmt::info!("node: relay link up");
                        relay::STATUS.signal(RelayStatus::Connected);
                    }
                    Role::Bridge { .. } => {
                        let mut name = [0u8; 16];
                        defmt::info!(
                            "node: bridge link to {=str} is up",
                            call_str(&peer, &mut name)
                        );
                        // The target's own banner flows over the bridge next.
                    }
                    Role::Interlink => {
                        let mut name = [0u8; 16];
                        defmt::info!(
                            "node: interlink to {=str} established",
                            call_str(&peer, &mut name)
                        );
                    }
                    _ => {}
                },
                DataLinkSignal::DataIndication(pid, info) if pid == PID_NETROM => {
                    // An interlink datagram (NET/ROM L3/L4), never console
                    // text. Routed to the connector by drive(). A console user
                    // never sends PID 0xCF: a node that connected before we
                    // knew it was one (we had not heard its NODES yet) got a
                    // console; it is an interlink.
                    if matches!(ps.role, Role::Console(_)) {
                        let mut name = [0u8; 16];
                        defmt::info!(
                            "node: {=str} speaks NET/ROM, treating its link as an interlink",
                            call_str(&peer, &mut name)
                        );
                        ps.role = Role::Interlink;
                    }
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
                                        console: Leg::Peer(peer),
                                        target: call,
                                    });
                                }
                            }
                            if !disconnect && !bridging {
                                reply.extend_from_slice(prompt.as_bytes());
                            }
                            if !reply.is_empty() {
                                to_send.extend(sessions.post_stream(
                                    local,
                                    peer,
                                    PID_NO_LAYER3,
                                    reply,
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
                    Role::TelnetRelay => relay::to_user(&info),
                    Role::None | Role::Interlink => {}
                },
                DataLinkSignal::DisconnectIndication | DataLinkSignal::DisconnectConfirm => {
                    let mut name = [0u8; 16];
                    defmt::info!(
                        "node: AX.25 session with {=str} closed",
                        call_str(&peer, &mut name)
                    );
                    match core::mem::replace(&mut ps.role, Role::None) {
                        Role::TelnetRelay => relay::STATUS.signal(RelayStatus::Disconnected),
                        Role::Bridge { other, .. } => {
                            followups.push(FollowUp::BridgeEnded {
                                survivor: other,
                                note: "*** Disconnected",
                            })
                        }
                        _ => {}
                    }
                }
                DataLinkSignal::UnitDataIndication(..) => {}
                DataLinkSignal::ErrorIndication(code) => {
                    defmt::warn!("node: DL error indication {=str}", code);
                }
            }
        }
    }

    (to_send, followups)
}
