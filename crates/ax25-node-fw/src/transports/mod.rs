//! The node's transports — one module per connectivity capability, mirroring the
//! C# `Packet.Node.Core.Transports` + `Packet.Axudp` / `Packet.Kiss`.
//!
//! Each is an Embassy task that owns its socket/UART, frames bytes with the
//! portable codecs in [`ax25_node_core`], and exchanges AX.25 frames with the
//! [`crate::session`] layer.

pub mod axudp;
pub mod kiss_tcp;
pub mod telnet;
// GATE 6 (HW-BRINGUP.md §4): kiss_serial returns if/when a NinoTNC is present at
// this machine (its UART generics also don't compile against embassy-rp 0.10 yet).
// pub mod kiss_serial;

use ax25_node_core::ax25::frame::CONTROL_UI;
use ax25_node_core::ax25::{Address, Callsign, Frame, PID_NO_LAYER3};

use alloc::vec::Vec;

use embassy_net::tcp::TcpSocket;
use embassy_net::IpEndpoint;

/// Render a callsign into a small stack buffer for defmt logging.
pub fn call_str<'b>(call: &Callsign, buf: &'b mut [u8; 16]) -> &'b str {
    let n = call.write_display(buf).unwrap_or(0);
    core::str::from_utf8(&buf[..n]).unwrap_or("?")
}

/// Build a UI frame `my_call → dest` with the given info text (the bring-up
/// beacon shape; standard 0xF0 no-L3 PID).
pub fn ui_frame(my_call: Callsign, dest: &str, info: &[u8]) -> Frame {
    Frame {
        destination: Address {
            callsign: Callsign::parse(dest).expect("static callsign"),
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
        info: info.to_vec(),
    }
}

/// Parse `"a.b.c.d:port"` into an endpoint (build-env config — host-side LAN
/// details are kept out of committed defaults per HW-BRINGUP §5).
pub fn parse_endpoint(s: &str) -> Option<IpEndpoint> {
    let (ip, port) = s.split_once(':')?;
    let ip: core::net::Ipv4Addr = ip.parse().ok()?;
    let port: u16 = port.parse().ok()?;
    Some(IpEndpoint::new(ip.into(), port))
}

/// Write all of `bytes` to a TCP socket, returning `false` on any error or
/// closed peer (callers drop the connection).
pub async fn tcp_write_all(socket: &mut TcpSocket<'_>, mut bytes: &[u8]) -> bool {
    while !bytes.is_empty() {
        match socket.write(bytes).await {
            Ok(0) => return false,
            Ok(n) => bytes = &bytes[n..],
            Err(e) => {
                defmt::warn!("tcp write error {:?}", e);
                return false;
            }
        }
    }
    true
}
