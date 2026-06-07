//! Capability 2 — KISS-over-TCP to net-sim (the emulated RF channel) over WiFi.
//!
//! Ports `Packet.Kiss.KissTcpClient` onto `embassy_net::tcp::TcpSocket`: connect
//! to the configured KISS-TCP endpoint, then bytes in → the host-tested
//! [`ax25_node_core::kiss::Decoder`] → for each `Data` frame, decode the AX.25
//! body, run the read-only NET/ROM tap, and log; outbound AX.25 frame →
//! [`ax25_node_core::kiss::encode`] → write. A reconnect/backoff wrapper mirrors
//! the C# `ReconnectingKissModem`.
//!
//! Gate 5 scope (minimum-green): KISS framing round-trips against a KISS-TCP
//! endpoint on the LAN (the `tools/kiss-tcp-harness.py` listener) — a beacon UI
//! frame goes out KISS-framed, and inbound KISS `Data` frames are de-framed,
//! AX.25-decoded and logged. The net-sim attachment is the lab-coordinated
//! stretch (HW-BRINGUP §6); the session hand-off is the supervisor seam.

use ax25_node_core::kiss::{self, Decoder};
use ax25_node_core::netrom::{ObserveOutcome, PortId};

use embassy_futures::select::{select, Either};
use embassy_net::tcp::TcpSocket;
use embassy_net::Stack;
use embassy_time::{Duration, Ticker, Timer};

use ax25_node_core::ax25::Callsign;

use crate::config::KissTcpConfig;
use crate::session;
use crate::transports::{call_str, parse_endpoint, tcp_write_all, ui_frame};

/// Seconds between beacon UI frames while connected.
const BEACON_INTERVAL_SECS: u64 = 10;
/// KISS multi-drop port nibble (single-port endpoints use 0).
const KISS_PORT: u8 = 0;

#[embassy_executor::task]
pub async fn task(stack: Stack<'static>, cfg: KissTcpConfig, my_call: Callsign) {
    // §5: the endpoint is a LAN detail from the build env; absent ⇒ disabled.
    let Some(target) = cfg.target.and_then(parse_endpoint) else {
        defmt::info!("kiss-tcp: no KISS_TCP_TARGET set — disabled");
        return;
    };
    defmt::info!("kiss-tcp: connecting to {:?}", target);

    let mut rx_buf = [0u8; 2048];
    let mut tx_buf = [0u8; 2048];
    let mut backoff_secs = 1u64;

    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buf, &mut tx_buf);
        if let Err(e) = socket.connect(target).await {
            defmt::warn!(
                "kiss-tcp: connect {:?} failed {:?}, retrying in {=u64}s",
                target,
                e,
                backoff_secs
            );
            Timer::after_secs(backoff_secs).await;
            backoff_secs = (backoff_secs * 2).min(30);
            continue;
        }
        backoff_secs = 1;
        defmt::info!("kiss-tcp: connected to {:?}", target);

        serve(&mut socket, my_call).await;

        socket.close();
        let _ = socket.flush().await;
        socket.abort();
        defmt::warn!("kiss-tcp: connection lost, reconnecting");
    }
}

/// One connection: beacon ticker + read pump, until the peer goes away.
async fn serve(socket: &mut TcpSocket<'_>, my_call: Callsign) {
    // The read-only NET/ROM tap — same FrameTraced-equivalent point as axudp.
    let mut netrom = session::new_netrom();
    let port_id = PortId::from_str_lossy("kiss-tcp");

    let mut decoder = Decoder::new();
    let mut buf = [0u8; 512];
    let mut ticker = Ticker::every(Duration::from_secs(BEACON_INTERVAL_SECS));
    let mut src_buf = [0u8; 16];
    let mut dst_buf = [0u8; 16];

    loop {
        match select(ticker.next(), socket.read(&mut buf)).await {
            Either::First(()) => {
                let beacon = ui_frame(
                    my_call,
                    "IDENT",
                    b"pico-node KISS-TCP beacon (HW-BRINGUP Gate 5)",
                );
                let Some(kiss_bytes) =
                    kiss::encode(KISS_PORT, kiss::Command::Data, &beacon.encode())
                else {
                    defmt::warn!("kiss-tcp: beacon encode failed");
                    continue;
                };
                if !tcp_write_all(socket, &kiss_bytes).await {
                    return;
                }
                defmt::info!(
                    "kiss-tcp: beacon sent ({=usize} KISS bytes)",
                    kiss_bytes.len()
                );
            }
            Either::Second(Ok(0)) => return, // EOF
            Either::Second(Ok(n)) => {
                for kf in decoder.push(&buf[..n]) {
                    if kf.command != kiss::Command::Data {
                        defmt::info!(
                            "kiss-tcp: rx non-data KISS frame cmd={=u8}",
                            kf.command.to_nibble()
                        );
                        continue;
                    }
                    let Ok(frame) = ax25_node_core::ax25::Frame::decode(&kf.payload) else {
                        defmt::warn!(
                            "kiss-tcp: KISS data ({=usize}B) did not decode as AX.25",
                            kf.payload.len()
                        );
                        continue;
                    };

                    // READ-ONLY NET/ROM TAP — every frame, BEFORE address filtering.
                    let outcome = session::observe_inbound(&mut netrom, &frame, my_call, port_id);
                    if let ObserveOutcome::Ingested { .. } = outcome {
                        defmt::info!(
                            "kiss-tcp: NODES broadcast ingested ({=u32} destinations known)",
                            netrom.destination_count() as u32
                        );
                    }

                    defmt::info!(
                        "kiss-tcp: rx {=str} -> {=str} ctl={=u8:#04x} info={=usize}B",
                        call_str(&frame.source.callsign, &mut src_buf),
                        call_str(&frame.destination.callsign, &mut dst_buf),
                        frame.control,
                        frame.info.len(),
                    );
                    if frame.is_ui() && !frame.info.is_empty() {
                        if let Ok(text) = core::str::from_utf8(&frame.info) {
                            defmt::info!("kiss-tcp: rx UI text: {=str}", text);
                        }
                    }
                    // Address-filtered session routing: the session-supervisor seam.
                }
            }
            Either::Second(Err(e)) => {
                defmt::warn!("kiss-tcp: read error {:?}", e);
                return;
            }
        }
    }
}
