//! The node's transports — one module per connectivity capability, mirroring the
//! C# `Packet.Node.Core.Transports` + `Packet.Axudp` / `Packet.Kiss`.
//!
//! Each is an Embassy task that owns its socket/UART, frames bytes with the
//! portable codecs in [`ax25_node_core`], and exchanges AX.25 frames with the
//! [`crate::session`] layer.

pub mod axudp;
pub mod kiss_tcp;
pub mod relay;
pub mod telnet;
// kiss_serial (NinoTNC over UART1 GP20/21 — NinoBLE Rev5; HARDWARE-NINOBLE.md).
// Spawned; the read pump + NODES origination run, but the live exchange is
// hardware-gated on a NinoTNC (HW-BRINGUP Gate 6). Compile-validated only.
pub mod kiss_serial;
// tait_ccdi (Tait TM8100/TM8200 over its CCDI control channel on UART0 GP0/GP1 —
// a SECOND UART). Spawned; RSSI/PTT/channel/carrier-sense drive loop runs, but the
// live exchange is hardware-gated on a Tait radio. Compile-validated only.
pub mod tait_ccdi;

use ax25_node_core::ax25::frame::CONTROL_UI;
use ax25_node_core::ax25::{Address, Callsign, Frame};

use alloc::vec::Vec;

use embassy_net::tcp::TcpSocket;
use embassy_net::IpEndpoint;

/// Render a callsign into a small stack buffer for defmt logging.
pub fn call_str<'b>(call: &Callsign, buf: &'b mut [u8; 16]) -> &'b str {
    let n = call.write_display(buf).unwrap_or(0);
    core::str::from_utf8(&buf[..n]).unwrap_or("?")
}

/// Build a UI frame `my_call → dest` with the given PID + info (beacons use
/// 0xF0 no-L3; NODES broadcasts use the NET/ROM 0xCF).
pub fn ui_frame(my_call: Callsign, dest: Callsign, pid: u8, info: &[u8]) -> Frame {
    Frame {
        destination: Address {
            callsign: dest,
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
        pid: Some(pid),
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
