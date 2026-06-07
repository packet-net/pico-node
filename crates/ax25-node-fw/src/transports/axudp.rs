//! Capability 1 — AXUDP: AX.25-over-UDP for node↔node connectivity over WiFi.
//!
//! Ports `Packet.Axudp.AxudpSocket` onto `embassy_net::udp::UdpSocket` (the 1:1
//! mapping the research note identifies). The UDP payload *is* the AX.25 frame
//! body; framing/encode/decode (incl. the optional XRouter CRC FCS) come from
//! [`ax25_node_core::axudp`].
//!
//! Gate 3 scope (HW-BRINGUP.md §4): the socket loop + frame codec path + the
//! read-only NET/ROM tap, exercised against a host harness — beacon a UI frame
//! at the configured target, decode + log whatever arrives. The session-layer
//! hand-off (connected mode) is the documented next seam: inbound frames that
//! pass the address filter currently stop at a log line; they will post into
//! [`crate::session::Sessions`] when the supervisor lands (Gate 3 stretch+).

use ax25_node_core::ax25::Callsign;
use ax25_node_core::axudp;
use ax25_node_core::netrom::{ObserveOutcome, PortId};

use embassy_futures::select::{select, Either};
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{IpEndpoint, Stack};
use embassy_time::{Duration, Ticker};

use crate::config::AxudpConfig;
use crate::session;
use crate::transports::{call_str, parse_endpoint, ui_frame};

/// Seconds between beacon UI frames when a beacon target is configured.
const BEACON_INTERVAL_SECS: u64 = 10;

#[embassy_executor::task]
pub async fn task(stack: Stack<'static>, cfg: AxudpConfig, my_call: Callsign) {
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
                // Address-filtered session routing lands at the session-supervisor
                // seam (frames to my_call → classify_incoming → Sessions::post).
            }
            Either::Second(Err(e)) => {
                defmt::warn!("axudp: recv error {:?}", e);
            }
        }
    }
}
