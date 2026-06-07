//! Provisioning support — AP-mode services (PROVISIONING.md step 3+).
//!
//! Step 3 ships the **DHCP server** so a client associating to the node's
//! config AP gets an address on the 192.168.4.x subnet (the captive portal's
//! DNS catch-all + HTTP form are step 4). A minimal RFC 2131 server: it answers
//! DISCOVER with OFFER and REQUEST with ACK, leasing sequentially from a small
//! pool, with the node (192.168.4.1) as gateway + DNS. No persistence, no
//! relay, no decline handling — an AP that hands out a handful of leases for a
//! brief configuration session, nothing more.

use core::net::Ipv4Addr;

use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{IpEndpoint, IpListenEndpoint, Ipv4Address, Stack};

use crate::net::AP_ADDRESS;

const DHCP_SERVER_PORT: u16 = 67;
const DHCP_CLIENT_PORT: u16 = 68;
const MAGIC_COOKIE: [u8; 4] = [99, 130, 83, 99];

// DHCP message types (option 53).
const DHCPDISCOVER: u8 = 1;
const DHCPOFFER: u8 = 2;
const DHCPREQUEST: u8 = 3;
const DHCPACK: u8 = 5;

/// Lease pool: 192.168.4.10 .. .4.10+POOL-1.
const POOL_BASE: u8 = 10;
const POOL_SIZE: u8 = 16;
const LEASE_SECS: u32 = 3600;

/// One handed-out lease (client MAC → offered last octet).
#[derive(Clone, Copy)]
struct Lease {
    mac: [u8; 6],
    host: u8,
}

#[embassy_executor::task]
pub async fn dhcp_server(stack: Stack<'static>) {
    let mut rx_meta = [PacketMetadata::EMPTY; 4];
    let mut tx_meta = [PacketMetadata::EMPTY; 4];
    let mut rx_buf = [0u8; 1024];
    let mut tx_buf = [0u8; 1024];
    let mut socket = UdpSocket::new(stack, &mut rx_meta, &mut rx_buf, &mut tx_meta, &mut tx_buf);
    defmt::unwrap!(socket.bind(IpListenEndpoint {
        addr: None,
        port: DHCP_SERVER_PORT,
    }));
    defmt::info!("dhcp: server up on the AP subnet (192.168.4.0/24)");

    let mut leases: [Option<Lease>; POOL_SIZE as usize] = [None; POOL_SIZE as usize];
    let mut buf = [0u8; 1024];
    let mut out = [0u8; 1024];

    loop {
        let Ok((n, _)) = socket.recv_from(&mut buf).await else {
            continue;
        };
        let pkt = &buf[..n];
        let Some(req) = parse(pkt) else { continue };

        let host = assign(&mut leases, req.mac);
        let yiaddr = Ipv4Addr::new(192, 168, 4, host);
        let reply_type = match req.msg_type {
            DHCPDISCOVER => DHCPOFFER,
            DHCPREQUEST => DHCPACK,
            _ => continue,
        };

        let len = build_reply(&mut out, &req, yiaddr, reply_type);
        // Broadcast the reply (the client has no IP yet).
        let dst = IpEndpoint::new(Ipv4Address::new(255, 255, 255, 255).into(), DHCP_CLIENT_PORT);
        if socket.send_to(&out[..len], dst).await.is_ok() {
            let mt = if reply_type == DHCPOFFER { "OFFER" } else { "ACK" };
            defmt::info!("dhcp: {=str} 192.168.4.{=u8} to {=u8:02x}:..", mt, host, req.mac[5]);
        }
    }
}

struct Request {
    xid: [u8; 4],
    mac: [u8; 6],
    msg_type: u8,
}

fn parse(pkt: &[u8]) -> Option<Request> {
    if pkt.len() < 240 || pkt[0] != 1 {
        return None; // not a BOOTREQUEST
    }
    if pkt[236..240] != MAGIC_COOKIE {
        return None;
    }
    let mut xid = [0u8; 4];
    xid.copy_from_slice(&pkt[4..8]);
    let mut mac = [0u8; 6];
    mac.copy_from_slice(&pkt[28..34]);

    // Find option 53 (message type) in the options TLVs.
    let mut msg_type = 0u8;
    let mut i = 240;
    while i < pkt.len() {
        match pkt[i] {
            0 => i += 1,    // PAD
            255 => break,   // END
            opt => {
                let l = *pkt.get(i + 1)? as usize;
                if opt == 53 && l == 1 {
                    msg_type = *pkt.get(i + 2)?;
                }
                i += 2 + l;
            }
        }
    }
    Some(Request { xid, mac, msg_type })
}

/// Reuse a MAC's lease, else take the next free slot (wraps over the oldest).
fn assign(leases: &mut [Option<Lease>], mac: [u8; 6]) -> u8 {
    if let Some(l) = leases.iter().flatten().find(|l| l.mac == mac) {
        return l.host;
    }
    let idx = leases
        .iter()
        .position(|l| l.is_none())
        .unwrap_or(0);
    let host = POOL_BASE + idx as u8;
    leases[idx] = Some(Lease { mac, host });
    host
}

fn build_reply(out: &mut [u8], req: &Request, yiaddr: Ipv4Addr, msg_type: u8) -> usize {
    out[..240].fill(0);
    out[0] = 2; // BOOTREPLY
    out[1] = 1; // ethernet
    out[2] = 6; // hlen
    out[4..8].copy_from_slice(&req.xid);
    out[16..20].copy_from_slice(&yiaddr.octets()); // yiaddr
    out[20..24].copy_from_slice(&AP_ADDRESS.octets()); // siaddr (us)
    out[28..34].copy_from_slice(&req.mac);
    out[236..240].copy_from_slice(&MAGIC_COOKIE);

    let mut at = 240;
    let opt = |out: &mut [u8], at: &mut usize, code: u8, data: &[u8]| {
        out[*at] = code;
        out[*at + 1] = data.len() as u8;
        out[*at + 2..*at + 2 + data.len()].copy_from_slice(data);
        *at += 2 + data.len();
    };
    let us = AP_ADDRESS.octets();
    opt(out, &mut at, 53, &[msg_type]); // message type
    opt(out, &mut at, 54, &us); // server identifier
    opt(out, &mut at, 51, &LEASE_SECS.to_be_bytes()); // lease time
    opt(out, &mut at, 1, &[255, 255, 255, 0]); // subnet mask
    opt(out, &mut at, 3, &us); // router (us)
    opt(out, &mut at, 6, &us); // DNS (us — the captive-portal catch-all in step 4)
    out[at] = 255; // END
    at + 1
}
