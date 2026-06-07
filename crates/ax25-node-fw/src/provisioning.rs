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
        let dst = IpEndpoint::new(
            Ipv4Address::new(255, 255, 255, 255).into(),
            DHCP_CLIENT_PORT,
        );
        if socket.send_to(&out[..len], dst).await.is_ok() {
            let mt = if reply_type == DHCPOFFER {
                "OFFER"
            } else {
                "ACK"
            };
            defmt::info!(
                "dhcp: {=str} 192.168.4.{=u8} to {=u8:02x}:..",
                mt,
                host,
                req.mac[5]
            );
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
            0 => i += 1,  // PAD
            255 => break, // END
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
    let idx = leases.iter().position(|l| l.is_none()).unwrap_or(0);
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

// ─────────────────────────────────────────────────────────────────────────────
// Captive portal (PROVISIONING.md step 4): a DNS catch-all + an HTTP config
// form. The DNS server answers every A query with 192.168.4.1, so a client's
// OS connectivity probe resolves to us and — getting the config page instead of
// its expected response — pops the captive portal automatically.
// ─────────────────────────────────────────────────────────────────────────────

use ax25_node_core::console::service::ConfigOp;
use embassy_net::tcp::TcpSocket;
use embassy_time::Duration;

const DNS_PORT: u16 = 53;
const HTTP_PORT: u16 = 80;

/// DNS catch-all: every A query → the AP gateway. Minimal — copies the question
/// into the answer with a fixed A record; non-A / malformed queries are ignored.
#[embassy_executor::task]
pub async fn dns_catch_all(stack: Stack<'static>) {
    let mut rx_meta = [PacketMetadata::EMPTY; 4];
    let mut tx_meta = [PacketMetadata::EMPTY; 4];
    let mut rx_buf = [0u8; 512];
    let mut tx_buf = [0u8; 512];
    let mut socket = UdpSocket::new(stack, &mut rx_meta, &mut rx_buf, &mut tx_meta, &mut tx_buf);
    defmt::unwrap!(socket.bind(DNS_PORT));
    defmt::info!("dns: catch-all up (every name -> 192.168.4.1)");

    let mut buf = [0u8; 512];
    let mut out = [0u8; 512];
    loop {
        let Ok((n, from)) = socket.recv_from(&mut buf).await else {
            continue;
        };
        if n < 12 {
            continue;
        }
        // Header: copy ID; set response + recursion-available; QDCOUNT stays;
        // ANCOUNT = 1. Question section is echoed; one A answer appended.
        let qd = u16::from_be_bytes([buf[4], buf[5]]);
        if qd != 1 || (buf[2] & 0x80) != 0 {
            continue; // not a single-question query
        }
        // Find the end of the question (QNAME terminator + QTYPE/QCLASS).
        let mut i = 12;
        while i < n && buf[i] != 0 {
            i += buf[i] as usize + 1;
        }
        let qend = i + 5; // null label + qtype(2) + qclass(2)
        if qend > n || qend > out.len() {
            continue;
        }
        out[..qend].copy_from_slice(&buf[..qend]);
        out[2] = 0x81; // QR=1, RD copied via low bit below
        out[3] = 0x80; // RA=1
        out[6..8].copy_from_slice(&1u16.to_be_bytes()); // ANCOUNT=1
        out[10..12].copy_from_slice(&[0, 0]); // ARCOUNT=0
        let mut at = qend;
        out[at..at + 2].copy_from_slice(&[0xc0, 0x0c]); // name ptr to question
        out[at + 2..at + 4].copy_from_slice(&1u16.to_be_bytes()); // TYPE A
        out[at + 4..at + 6].copy_from_slice(&1u16.to_be_bytes()); // CLASS IN
        out[at + 6..at + 10].copy_from_slice(&60u32.to_be_bytes()); // TTL
        out[at + 10..at + 12].copy_from_slice(&4u16.to_be_bytes()); // RDLENGTH
        out[at + 12..at + 16].copy_from_slice(&AP_ADDRESS.octets());
        at += 16;
        let _ = socket.send_to(&out[..at], from).await;
    }
}

/// HTTP config form: GET serves the form (so any OS probe pops the portal),
/// POST /save applies the fields + reboots into STA mode.
#[embassy_executor::task]
pub async fn http_portal(stack: Stack<'static>) {
    let mut rx_buf = [0u8; 2048];
    let mut tx_buf = [0u8; 2048];
    let mut body_buf = [0u8; 1024];
    defmt::info!("http: captive portal up on :80");
    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buf, &mut tx_buf);
        socket.set_timeout(Some(Duration::from_secs(10)));
        if socket.accept(HTTP_PORT).await.is_err() {
            continue;
        }
        // Read the request (headers + any body) into one buffer.
        let mut req = [0u8; 2048];
        let mut len = 0;
        loop {
            match socket.read(&mut req[len..]).await {
                Ok(0) => break,
                Ok(n) => {
                    len += n;
                    // Stop once we have the headers (and a short body fits).
                    if len >= 4 && req[..len].windows(4).any(|w| w == b"\r\n\r\n") {
                        // Drain a little more for the form body if it's a POST.
                        if req[..len].starts_with(b"POST") && len < req.len() {
                            if let Ok(m) = socket.read(&mut req[len..]).await {
                                len += m;
                            }
                        }
                        break;
                    }
                    if len == req.len() {
                        break;
                    }
                }
                Err(_) => break,
            }
        }
        let request = &req[..len];

        let response = if request.starts_with(b"POST /save") {
            let saved = apply_form(request, &mut body_buf);
            if saved {
                // Reboot shortly after the page is sent (into STA mode).
                REBOOT_AFTER_HTTP.store(true, core::sync::atomic::Ordering::Relaxed);
                SAVED_PAGE
            } else {
                FORM_PAGE
            }
        } else {
            FORM_PAGE
        };

        let _ = http_write(&mut socket, response).await;
        let _ = socket.flush().await;
        socket.close();
        if REBOOT_AFTER_HTTP.load(core::sync::atomic::Ordering::Relaxed) {
            embassy_time::Timer::after_millis(500).await;
            cortex_m::peripheral::SCB::sys_reset();
        }
    }
}

static REBOOT_AFTER_HTTP: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(false);

async fn http_write(socket: &mut TcpSocket<'_>, body: &str) -> bool {
    use core::fmt::Write;
    let mut head = heapless::String::<128>::new();
    let _ = write!(
        head,
        "HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nContent-Length: {}\r\nConnection: close\r\n\r\n",
        body.len()
    );
    let mut bytes = head.as_bytes();
    while !bytes.is_empty() {
        match socket.write(bytes).await {
            Ok(0) | Err(_) => return false,
            Ok(n) => bytes = &bytes[n..],
        }
    }
    let mut b = body.as_bytes();
    while !b.is_empty() {
        match socket.write(b).await {
            Ok(0) | Err(_) => return false,
            Ok(n) => b = &b[n..],
        }
    }
    true
}

/// Parse the urlencoded POST body, apply each field to the pending config via
/// the console ConfigOp path, then SAVE. Returns whether anything was saved.
fn apply_form(request: &[u8], scratch: &mut [u8; 1024]) -> bool {
    let Some(pos) = request.windows(4).position(|w| w == b"\r\n\r\n") else {
        return false;
    };
    let body = &request[pos + 4..];
    let n = body.len().min(scratch.len());
    scratch[..n].copy_from_slice(&body[..n]);
    let body = &scratch[..n];

    let mut any = false;
    for pair in body.split(|&b| b == b'&') {
        let Some(eq) = pair.iter().position(|&b| b == b'=') else {
            continue;
        };
        let key = &pair[..eq];
        let val = url_decode(&pair[eq + 1..]);
        let (Ok(key), Some(val)) = (core::str::from_utf8(key), val) else {
            continue;
        };
        if val.is_empty() {
            continue;
        }
        // Map form names to config keys (uppercased) and stage them.
        let op = ConfigOp::Set {
            key: key.to_ascii_uppercase(),
            value: val,
        };
        let (_text, _reboot) = crate::config_store::handle_op(&op);
        any = true;
    }
    if any {
        let (_text, _) = crate::config_store::handle_op(&ConfigOp::Save);
    }
    any
}

/// Decode application/x-www-form-urlencoded (%XX + `+`). Returns an owned String.
fn url_decode(bytes: &[u8]) -> Option<alloc::string::String> {
    let mut out = alloc::vec::Vec::with_capacity(bytes.len());
    let mut i = 0;
    while i < bytes.len() {
        match bytes[i] {
            b'+' => {
                out.push(b' ');
                i += 1;
            }
            b'%' if i + 2 < bytes.len() => {
                let h = hex(bytes[i + 1])?;
                let l = hex(bytes[i + 2])?;
                out.push((h << 4) | l);
                i += 3;
            }
            c => {
                out.push(c);
                i += 1;
            }
        }
    }
    alloc::string::String::from_utf8(out).ok()
}

fn hex(b: u8) -> Option<u8> {
    match b {
        b'0'..=b'9' => Some(b - b'0'),
        b'a'..=b'f' => Some(b - b'a' + 10),
        b'A'..=b'F' => Some(b - b'A' + 10),
        _ => None,
    }
}

const FORM_PAGE: &str = "<!DOCTYPE html><html><head><meta name=viewport content=\"width=device-width,initial-scale=1\"><title>pico-node config</title><style>body{font-family:sans-serif;max-width:30em;margin:2em auto;padding:0 1em}label{display:block;margin:.8em 0 .2em}input{width:100%;padding:.4em;box-sizing:border-box}button{margin-top:1.2em;padding:.6em 1.2em;font-size:1em}</style></head><body><h2>pico-node configuration</h2><form method=post action=/save><label>Callsign</label><input name=callsign placeholder=M0ABC-1><label>Alias</label><input name=alias placeholder=NODE><label>Grid</label><input name=grid placeholder=IO91><label>WiFi network (SSID)</label><input name=wifi_ssid><label>WiFi password</label><input name=wifi_pass type=password><label>MQTT host (optional, host:port for logs)</label><input name=mqtt_host><button type=submit>Save &amp; reboot</button></form><p>Leave a field blank to keep its current value. Set a WiFi network to join it on the next boot; the node returns to this AP if it can't.</p></body></html>";

const SAVED_PAGE: &str = "<!DOCTYPE html><html><head><meta name=viewport content=\"width=device-width,initial-scale=1\"><title>saved</title></head><body style=\"font-family:sans-serif;max-width:30em;margin:2em auto\"><h2>Saved &mdash; rebooting</h2><p>The node is restarting. If you set a WiFi network it will join that now; otherwise it returns to this configuration AP.</p></body></html>";
