//! Capability 1 — AXUDP: AX.25-over-UDP for node↔node connectivity over WiFi.
//!
//! Ports `Packet.Axudp.AxudpSocket` onto `embassy_net::udp::UdpSocket` (the 1:1
//! mapping the research note identifies). The UDP payload *is* the AX.25 frame
//! body; framing/encode/decode (incl. the optional XRouter CRC FCS) come from
//! [`ax25_node_core::axudp`]. Receive datagram → decode → hand to the session
//! layer; session emits frame → encode → send datagram to the peer endpoint.
//!
//! STUB: socket I/O body to be written against embassy-net.

use ax25_node_core::axudp;
use embassy_net::Stack;

use crate::config::AxudpConfig;

#[embassy_executor::task]
pub async fn task(_stack: Stack<'static>, cfg: AxudpConfig) {
    defmt::info!("axudp: listen udp/{} fcs={}", cfg.listen_port, cfg.include_fcs);
    // 1. Bind a UdpSocket on cfg.listen_port (rx/tx packet metadata buffers in
    //    a static_cell, sized for the configured window — research §6).
    // 2. loop:
    //      let (n, ep) = socket.recv_from(&mut buf).await?;
    //      let rx = axudp::decode_datagram(&buf[..n], cfg.include_fcs);
    //      if let Some(frame) = rx.frame { session::deliver(ep, frame).await; }
    // 3. Outbound: session hands (ep, frame) → axudp::encode_datagram(&frame,
    //    cfg.include_fcs) → socket.send_to(&dgram, ep).await?;
    let _ = axudp::encode_datagram; // keep the import wired until the body lands
    unimplemented!("AXUDP UdpSocket loop")
}
