//! Port 1: an optional KISS-over-TCP modem, such as net-sim's emulated RF
//! channel (a way to run the node without radio hardware), or a TNC behind a
//! KISS-over-TCP server.
//!
//! Ports `Packet.Kiss.KissTcpClient` onto `embassy_net::tcp::TcpSocket`: connect
//! to the build-env `KISS_TCP_TARGET` (`a.b.c.d:port`; unset = no such port),
//! reconnecting with back-off (the C# `ReconnectingKissModem`). While connected
//! the port is usable: KISS `Data` frames are decoded and delivered to the node
//! task, and the node task's frames are KISS-framed and written. Everything
//! above the modem (sessions, NET/ROM, NODES) is the node task's, as for the
//! radio port; frames show in the monitor tagged `[tcp]`.

use ax25_node_core::kiss::{self, Decoder};
use ax25_node_core::monitor::{write_frame, Direction};

use embassy_futures::select::{select, Either};
use embassy_net::tcp::TcpSocket;
use embassy_net::Stack;
use embassy_time::Timer;

use crate::config::KissTcpConfig;
use crate::net::{parse_endpoint, tcp_write_all};
use crate::ports::{self, Port};
use crate::tnc;

/// KISS multi-drop port nibble (single-port endpoints use 0).
const KISS_PORT: u8 = 0;

/// This driver's port.
const PORT: Port = Port::KISS_TCP;

#[embassy_executor::task]
pub async fn task(stack: Stack<'static>, cfg: KissTcpConfig) {
    let Some(target) = cfg.target.and_then(parse_endpoint) else {
        defmt::info!("kiss-tcp: no KISS_TCP_TARGET set, port not in use");
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
        tnc::log(Direction::Info, "[tcp] KISS-over-TCP port connected");
        ports::set_usable(PORT, true);

        serve(&mut socket).await;

        ports::set_usable(PORT, false);
        tnc::log(Direction::Info, "[tcp] KISS-over-TCP port lost, reconnecting");
        socket.close();
        let _ = socket.flush().await;
        socket.abort();
        defmt::warn!("kiss-tcp: connection lost, reconnecting");
    }
}

/// One connection: frames in both directions until the peer goes away.
async fn serve(socket: &mut TcpSocket<'_>) {
    let mut decoder = Decoder::new();
    let mut buf = [0u8; 512];
    loop {
        // Both arms are cancel-safe: an unread socket byte stays in the socket,
        // an untaken frame stays in the port's queue.
        match select(socket.read(&mut buf), ports::take_tx(PORT)).await {
            Either::First(Ok(0)) => return, // EOF
            Either::First(Ok(n)) => {
                for kf in decoder.push(&buf[..n]) {
                    if kf.command != kiss::Command::Data {
                        continue;
                    }
                    let Ok(frame) = ax25_node_core::ax25::Frame::decode(&kf.payload) else {
                        defmt::warn!(
                            "kiss-tcp: KISS data ({=usize}B) did not decode as AX.25",
                            kf.payload.len()
                        );
                        continue;
                    };
                    tnc::log_with(Direction::Rx, |w| {
                        w.write_str("[tcp] ")?;
                        write_frame(&frame, w)
                    });
                    ports::deliver(PORT, frame);
                }
            }
            Either::First(Err(e)) => {
                defmt::warn!("kiss-tcp: read error {:?}", e);
                return;
            }
            Either::Second(wire) => {
                let Some(bytes) = kiss::encode(KISS_PORT, kiss::Command::Data, &wire) else {
                    defmt::warn!("kiss-tcp: frame too large to KISS-encode");
                    continue;
                };
                if !tcp_write_all(socket, &bytes).await {
                    return;
                }
                if let Ok(frame) = ax25_node_core::ax25::Frame::decode(&wire) {
                    tnc::log_with(Direction::Tx, |w| {
                        w.write_str("[tcp] ")?;
                        write_frame(&frame, w)
                    });
                }
            }
        }
    }
}
