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

use ax25_node_core::ax25::frame::CONTROL_UI;
use ax25_node_core::ax25::{Address, Callsign, Frame, PID_NO_LAYER3};
use ax25_node_core::axudp;
use ax25_node_core::netrom::{ObserveOutcome, PortId};

use alloc::vec::Vec;

use embassy_futures::select::{select, Either};
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{IpEndpoint, Stack};
use embassy_time::{Duration, Ticker};

use crate::config::AxudpConfig;
use crate::session;

/// Seconds between beacon UI frames when a beacon target is configured.
const BEACON_INTERVAL_SECS: u64 = 10;

/// Render a callsign into a small stack buffer for defmt logging.
fn call_str<'b>(call: &Callsign, buf: &'b mut [u8; 16]) -> &'b str {
    let n = call.write_display(buf).unwrap_or(0);
    core::str::from_utf8(&buf[..n]).unwrap_or("?")
}

/// Build the Gate-3 beacon: a UI frame `my_call → IDENT`, standard 0xF0 PID.
fn beacon_frame(my_call: Callsign) -> Frame {
    Frame {
        destination: Address {
            callsign: Callsign::parse("IDENT").expect("static callsign"),
            crh: true,
            extension: false,
        },
        source: Address {
            callsign: my_call,
            crh: false,
            extension: false,
        },
        digipeaters: Vec::new(),
        control: CONTROL_UI,
        pid: Some(PID_NO_LAYER3),
        info: b"pico-node AXUDP beacon (HW-BRINGUP Gate 3)".to_vec(),
    }
}

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
                    let dgram = axudp::encode_datagram(&beacon_frame(my_call), cfg.include_fcs);
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

/// Parse `"a.b.c.d:port"` into an endpoint (build-env config — host-side LAN
/// detail, so kept out of committed defaults per HW-BRINGUP §5).
fn parse_endpoint(s: &str) -> Option<IpEndpoint> {
    let (ip, port) = s.split_once(':')?;
    let ip: core::net::Ipv4Addr = ip.parse().ok()?;
    let port: u16 = port.parse().ok()?;
    Some(IpEndpoint::new(ip.into(), port))
}
