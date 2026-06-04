//! Capability 2 — KISS-over-TCP to net-sim (the emulated RF channel) over WiFi.
//!
//! Ports `Packet.Kiss.KissTcpClient` onto `embassy_net::tcp::TcpSocket`. Connect
//! to net-sim's KISS-TCP listener, then: bytes in → [`ax25_node_core::kiss::Decoder`]
//! → for each `Data` frame, decode the AX.25 body and deliver to the session
//! layer; outbound AX.25 frame → [`ax25_node_core::kiss::encode_into`] → write.
//! A reconnect/backoff wrapper mirrors `ReconnectingKissModem`.
//!
//! STUB: socket I/O body to be written against embassy-net.

use ax25_node_core::kiss::{self, Decoder};
use embassy_net::Stack;

use crate::config::KissTcpConfig;

#[embassy_executor::task]
pub async fn task(_stack: Stack<'static>, cfg: KissTcpConfig) {
    defmt::info!("kiss-tcp: connect {}:{}", cfg.host, cfg.port);
    let mut _decoder = Decoder::new();
    // loop {
    //   connect TcpSocket to (cfg.host, cfg.port) with backoff;
    //   loop {
    //     let n = socket.read(&mut buf).await?; if n == 0 { break }   // reconnect
    //     for frame in _decoder.push(&buf[..n]) {
    //       if frame.command == kiss::Command::Data {
    //         if let Ok(ax) = ax25_node_core::ax25::Frame::decode(&frame.payload) {
    //           // READ-ONLY NET/ROM TAP — every frame, BEFORE the address filter,
    //           // so NODES broadcasts (dest "NODES", not us) are heard. Observation
    //           // only; cannot disturb a session.
    //           session::observe_inbound(&mut netrom, &ax, my_call, PortId::from_str_lossy("kiss-tcp"));
    //           // ...then the normal address-filtered routing:
    //           session::deliver_kiss(ax).await;
    //   } } } }
    //   // outbound: kiss::encode_into(&mut out, 0, Command::Data, &ax.encode()) → write
    // }
    let _ = kiss::encode_into;
    unimplemented!("KISS-over-TCP client loop")
}
