//! Capability 1 — AXUDP: AX.25-over-UDP for node↔node connectivity over WiFi.
//!
//! Ports `Packet.Axudp.AxudpSocket` onto `embassy_net::udp::UdpSocket` (the 1:1
//! mapping the research note identifies). The UDP payload *is* the AX.25 frame
//! body; framing/encode/decode (incl. the optional XRouter CRC FCS) come from
//! [`ax25_node_core::axudp`].
//!
//! Beyond the Gate-3 socket loop + read-only NET/ROM tap, this task now owns
//! **the connected-mode session layer for the AXUDP port**: inbound frames that
//! pass the address filter are classified ([`classify_incoming`]) and posted
//! into a [`session::Sessions`] manager (the host-tested SDL runtime); the wire
//! frames each session emits go back to the peer's UDP endpoint, and the DL
//! signals raised upward drive **the node console over AX.25**
//! (`TransportKind::Ax25`, CR line discipline) — connect to the node from a real
//! BPQ peer and you land at the same prompt telnet users get.
//!
//! Single-transport ownership (this task owns `Sessions` exclusively) keeps the
//! `&mut` story trivial; when a second connected-mode transport arrives the
//! manager moves behind the supervisor seam `session.rs` documents. Timer note:
//! T1/T3 expiries aren't driven yet (no timer task) — responsive exchanges work
//! (every reply here is immediate); retransmit/keepalive behaviour is the
//! documented follow-up.

use ax25_node_core::ax25::{Callsign, PID_NO_LAYER3};
use ax25_node_core::axudp;
use ax25_node_core::console::command::parse_bytes;
use ax25_node_core::console::service::{banner_and_prompt, dispatch, Identity};
use ax25_node_core::console::{DispatchOutcome, LineAssembler, TransportKind};
use ax25_node_core::netrom::{ObserveOutcome, PortId};
use ax25_node_core::sdl::{classify_incoming, DataLinkSignal, Event};

use alloc::string::String;
use alloc::vec::Vec;

use embassy_futures::select::{select, Either};
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{IpEndpoint, Stack};
use embassy_time::{Duration, Ticker};

use crate::config::AxudpConfig;
use crate::session;
use crate::transports::{call_str, parse_endpoint, ui_frame};

/// Seconds between beacon UI frames when a beacon target is configured.
const BEACON_INTERVAL_SECS: u64 = 10;

/// Per-connected-peer console state: the CR/LF line assembler for the inbound
/// byte stream. One per session slot.
struct ConsolePeer {
    peer: Callsign,
    asm: LineAssembler,
}

#[embassy_executor::task]
pub async fn task(
    stack: Stack<'static>,
    cfg: AxudpConfig,
    my_call: Callsign,
    console_id: Identity,
    prompt: String,
) {
    defmt::info!(
        "axudp: listen udp/{} fcs={}",
        cfg.listen_port,
        cfg.include_fcs
    );

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

    // The connected-mode session layer for this port + the AX.25 console state.
    let mut sessions = session::new_sessions(my_call);
    let mut timers = session::EmbassyTimers::new();
    let mut consoles: [Option<ConsolePeer>; session::MAX_SESSIONS] =
        [const { None }; session::MAX_SESSIONS];

    let mut dgram_buf = [0u8; 2048];
    let mut ticker = Ticker::every(Duration::from_secs(BEACON_INTERVAL_SECS));
    let mut src_buf = [0u8; 16];
    let mut dst_buf = [0u8; 16];

    loop {
        match select(ticker.next(), socket.recv_from(&mut dgram_buf)).await {
            Either::First(()) => {
                if let Some(ep) = beacon_ep {
                    let beacon = ui_frame(
                        my_call,
                        "IDENT",
                        b"pico-node AXUDP beacon (HW-BRINGUP Gate 3)",
                    );
                    let dgram = axudp::encode_datagram(&beacon, cfg.include_fcs);
                    match socket.send_to(&dgram, ep).await {
                        Ok(()) => defmt::info!("axudp: beacon sent ({=usize} bytes)", dgram.len()),
                        Err(e) => defmt::warn!("axudp: beacon send error {:?}", e),
                    }
                }
            }
            Either::Second(Ok((n, meta))) => {
                let rx = axudp::decode_datagram(&dgram_buf[..n], cfg.include_fcs);
                let Some(frame) = rx.frame else {
                    defmt::warn!(
                        "axudp: {=usize} bytes from {:?} did not decode as AX.25",
                        n,
                        meta.endpoint
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
                    "axudp: rx {=str} -> {=str} ctl={=u8:#04x} info={=usize}B fcs_valid={:?} from {:?}",
                    call_str(&frame.source.callsign, &mut src_buf),
                    call_str(&frame.destination.callsign, &mut dst_buf),
                    frame.control,
                    frame.info.len(),
                    rx.fcs_valid,
                    meta.endpoint
                );
                if frame.is_ui() && !frame.info.is_empty() {
                    if let Ok(text) = core::str::from_utf8(&frame.info) {
                        defmt::info!("axudp: rx UI text: {=str}", text);
                    }
                }

                // Address filter → the connected-mode session layer.
                if frame.destination.callsign == my_call && !frame.is_ui() {
                    let Some(event) = classify_incoming(&frame) else {
                        continue;
                    };
                    let peer = frame.source.callsign;
                    let to_send = run_session(
                        &mut sessions,
                        &mut timers,
                        &mut consoles,
                        peer,
                        event,
                        &console_id,
                        &prompt,
                    );
                    for wire in to_send {
                        let dgram = with_fcs(wire, cfg.include_fcs);
                        if let Err(e) = socket.send_to(&dgram, meta.endpoint).await {
                            defmt::warn!("axudp: session tx error {:?}", e);
                        }
                    }
                }
            }
            Either::Second(Err(e)) => {
                defmt::warn!("axudp: recv error {:?}", e);
            }
        }
    }
}

/// Append the AXUDP trailing CRC (XRouter / BPQ AXIP-with-CRC form) to raw wire
/// octets when the port is configured for it.
fn with_fcs(mut wire: Vec<u8>, include_fcs: bool) -> Vec<u8> {
    if include_fcs {
        let fcs = ax25_node_core::crc::compute(&wire);
        wire.push((fcs & 0xFF) as u8);
        wire.push((fcs >> 8) as u8);
    }
    wire
}

/// Post one classified event into `peer`'s session and service every DL signal
/// it raises — the AX.25 console loop. Returns all wire frames to transmit.
fn run_session(
    sessions: &mut session::Sessions,
    timers: &mut session::EmbassyTimers,
    consoles: &mut [Option<ConsolePeer>],
    peer: Callsign,
    event: Event,
    console_id: &Identity,
    prompt: &str,
) -> Vec<Vec<u8>> {
    let mut to_send = sessions.post(peer, event, timers);

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
                    if let Some(slot) = consoles.iter_mut().find(|c| c.is_none()) {
                        *slot = Some(ConsolePeer {
                            peer,
                            asm: LineAssembler::default(),
                        });
                    }
                    let banner = banner_and_prompt(console_id, prompt, TransportKind::Ax25);
                    to_send.extend(sessions.post(
                        peer,
                        Event::DlDataRequest(PID_NO_LAYER3, banner),
                        timers,
                    ));
                }
                DataLinkSignal::DataIndication(_pid, info) => {
                    let lines = match consoles.iter_mut().flatten().find(|c| c.peer == peer) {
                        Some(c) => c.asm.push(&info),
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
                                timers,
                            ));
                        }
                        if disconnect {
                            to_send.extend(sessions.post(peer, Event::DlDisconnectRequest, timers));
                        }
                    }
                }
                DataLinkSignal::DisconnectIndication | DataLinkSignal::DisconnectConfirm => {
                    let mut name = [0u8; 16];
                    defmt::info!(
                        "axudp: AX.25 session with {=str} closed",
                        call_str(&peer, &mut name)
                    );
                    for slot in consoles.iter_mut() {
                        if matches!(slot, Some(c) if c.peer == peer) {
                            *slot = None;
                        }
                    }
                }
                DataLinkSignal::ConnectConfirm => {
                    // Outbound connects (node-to-node) arrive with the supervisor.
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
