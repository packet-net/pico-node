//! A minimal mDNS responder (RFC 6762) — make the node discoverable on the WLAN.
//!
//! Serves, over UDP 5353 on the 224.0.0.251 multicast group:
//!
//! - **`<hostname>.local` A** — so `telnet pico-node.local 8023` just works on
//!   any mDNS-capable OS (macOS, Linux/Avahi, Windows 10+).
//! - **`_telnet._tcp.local` PTR / SRV / TXT** — so service browsers
//!   (`avahi-browse -a`, Discovery.app) list the node's console with its port.
//! - **`_services._dns-sd._udp.local` PTR** — the DNS-SD service-type
//!   enumerator, so "browse everything" finds the service type at all.
//!
//! Scope (deliberate, documented): a *responder*, not a full stack — it answers
//! queries and sends two startup announcements. No probing/conflict resolution
//! (RFC 6762 §8: a SHOULD; the node's name is operator-assigned and unique on
//! the LAN), no known-answer suppression, no compressed-name parsing in
//! questions (compression is rare in real query sections; such questions are
//! skipped, not crashed on). Responses use uncompressed names throughout —
//! bigger on the wire, simpler to verify.

use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{IpEndpoint, Ipv4Address, Stack};
use embassy_time::Timer;

const MDNS_PORT: u16 = 5353;
const MDNS_GROUP: Ipv4Address = Ipv4Address::new(224, 0, 0, 251);

const TYPE_A: u16 = 1;
const TYPE_PTR: u16 = 12;
const TYPE_TXT: u16 = 16;
const TYPE_SRV: u16 = 33;
const TYPE_ANY: u16 = 255;
const CLASS_IN: u16 = 1;
/// IN + the mDNS cache-flush bit (set on records we are authoritative for).
const CLASS_IN_FLUSH: u16 = 0x8001;

const TTL_HOST: u32 = 120; // A/SRV — follows the address/port
const TTL_SERVICE: u32 = 4500; // PTR/TXT — service existence, per RFC 6762 §10

/// What we advertise. Built once in `main` from the node config.
pub struct MdnsConfig {
    /// The bare hostname label (no `.local`), e.g. `"pico-node"`.
    pub hostname: &'static str,
    /// The TCP port the telnet console listens on.
    pub telnet_port: u16,
}

#[embassy_executor::task]
pub async fn task(stack: Stack<'static>, cfg: MdnsConfig) {
    if let Err(e) = stack.join_multicast_group(MDNS_GROUP) {
        defmt::warn!("mdns: join 224.0.0.251 failed {:?} — disabled", e);
        return;
    }

    let mut rx_meta = [PacketMetadata::EMPTY; 4];
    let mut tx_meta = [PacketMetadata::EMPTY; 4];
    let mut rx_buf = [0u8; 1024];
    let mut tx_buf = [0u8; 1024];
    let mut socket = UdpSocket::new(stack, &mut rx_meta, &mut rx_buf, &mut tx_meta, &mut tx_buf);
    defmt::unwrap!(socket.bind(MDNS_PORT));
    defmt::info!(
        "mdns: responder up — {=str}.local, _telnet._tcp port {=u16}",
        cfg.hostname,
        cfg.telnet_port
    );

    let group_ep = IpEndpoint::new(MDNS_GROUP.into(), MDNS_PORT);
    let mut out = [0u8; 512];

    // Startup announcement ×2, one second apart (RFC 6762 §8.3).
    for _ in 0..2 {
        Timer::after_secs(1).await;
        if let Some(n) = build_response(&mut out, 0, &cfg, &stack) {
            match socket.send_to(&out[..n], group_ep).await {
                Ok(()) => defmt::info!("mdns: announced ({=usize} bytes)", n),
                Err(e) => defmt::warn!("mdns: announce send error {:?}", e),
            }
        }
    }

    let mut inbuf = [0u8; 1024];
    loop {
        let Ok((n, meta)) = socket.recv_from(&mut inbuf).await else {
            continue;
        };
        let pkt = &inbuf[..n];
        defmt::debug!("mdns: rx {=usize}B from {:?}", n, meta.endpoint);
        let Some(query_id) = parse_query(pkt, &cfg) else {
            continue;
        };
        defmt::info!("mdns: matched query from {:?}", meta.endpoint);
        // RFC 6762 §6.7: legacy (one-shot) queriers use a source port ≠ 5353 —
        // answer them unicast, echoing the query ID. Standard queriers get the
        // multicast response with ID 0.
        let legacy = meta.endpoint.port != MDNS_PORT;
        let id = if legacy { query_id } else { 0 };
        if let Some(n) = build_response(&mut out, id, &cfg, &stack) {
            let dest = if legacy { meta.endpoint } else { group_ep };
            let _ = socket.send_to(&out[..n], dest).await;
        }
    }
}

/// Parse an mDNS query; `Some(query id)` if any question names a record we
/// serve (the response is always the full authoritative set — simpler than
/// per-question answer selection, and resolvers cache the extras).
fn parse_query(pkt: &[u8], cfg: &MdnsConfig) -> Option<u16> {
    if pkt.len() < 12 {
        return None;
    }
    let id = u16::from_be_bytes([pkt[0], pkt[1]]);
    let flags = u16::from_be_bytes([pkt[2], pkt[3]]);
    if flags & 0x8000 != 0 {
        return None; // a response, not a query
    }
    let qdcount = u16::from_be_bytes([pkt[4], pkt[5]]).min(8);

    let mut off = 12usize;
    let mut any_match = false;
    for _ in 0..qdcount {
        // Parse one (uncompressed) QNAME into label slices.
        let mut labels: [&[u8]; 8] = [&[]; 8];
        let mut nlabels = 0usize;
        loop {
            let len = *pkt.get(off)? as usize;
            if len == 0 {
                off += 1;
                break;
            }
            if len & 0xC0 != 0 {
                return None; // compressed question — skip the whole packet
            }
            let label = pkt.get(off + 1..off + 1 + len)?;
            if nlabels < labels.len() {
                labels[nlabels] = label;
                nlabels += 1;
            }
            off += 1 + len;
        }
        let qtype = u16::from_be_bytes([*pkt.get(off)?, *pkt.get(off + 1)?]);
        off += 4; // qtype + qclass

        let labels = &labels[..nlabels];
        let host_match =
            name_eq(labels, &[cfg.hostname, "local"]) && (qtype == TYPE_A || qtype == TYPE_ANY);
        let svc_match = name_eq(labels, &["_telnet", "_tcp", "local"]);
        let enum_match = name_eq(labels, &["_services", "_dns-sd", "_udp", "local"]);
        let inst_match = name_eq(labels, &[cfg.hostname, "_telnet", "_tcp", "local"]);
        any_match |= host_match || svc_match || enum_match || inst_match;
    }
    any_match.then_some(id)
}

/// Case-insensitive DNS-name compare against expected labels.
fn name_eq(labels: &[&[u8]], expect: &[&str]) -> bool {
    labels.len() == expect.len()
        && labels
            .iter()
            .zip(expect)
            .all(|(l, e)| l.eq_ignore_ascii_case(e.as_bytes()))
}

/// Build the full authoritative answer set (A + PTRs + SRV + TXT). Returns the
/// packet length, or `None` if the IP isn't up yet / the buffer is too small.
fn build_response(
    out: &mut [u8],
    id: u16,
    cfg: &MdnsConfig,
    stack: &Stack<'static>,
) -> Option<usize> {
    let ip = stack.config_v4()?.address.address();

    let mut w = W { buf: out, pos: 0 };
    w.u16(id)?;
    w.u16(0x8400)?; // QR=response, AA=authoritative
    w.u16(0)?; // QDCOUNT
    w.u16(5)?; // ANCOUNT (the fixed record set below)
    w.u16(0)?; // NSCOUNT
    w.u16(0)?; // ARCOUNT

    let host = [cfg.hostname, "local"];
    let svc = ["_telnet", "_tcp", "local"];
    let inst = [cfg.hostname, "_telnet", "_tcp", "local"];
    let enumr = ["_services", "_dns-sd", "_udp", "local"];

    // <hostname>.local A <ip>
    w.name(&host)?;
    w.rr_header(TYPE_A, CLASS_IN_FLUSH, TTL_HOST, 4)?;
    w.put(&ip.octets())?;

    // _services._dns-sd._udp.local PTR _telnet._tcp.local
    w.name(&enumr)?;
    w.rr_header(TYPE_PTR, CLASS_IN, TTL_SERVICE, name_len(&svc))?;
    w.name(&svc)?;

    // _telnet._tcp.local PTR <hostname>._telnet._tcp.local
    w.name(&svc)?;
    w.rr_header(TYPE_PTR, CLASS_IN, TTL_SERVICE, name_len(&inst))?;
    w.name(&inst)?;

    // <hostname>._telnet._tcp.local SRV 0 0 <port> <hostname>.local
    w.name(&inst)?;
    w.rr_header(TYPE_SRV, CLASS_IN_FLUSH, TTL_HOST, 6 + name_len(&host))?;
    w.u16(0)?; // priority
    w.u16(0)?; // weight
    w.u16(cfg.telnet_port)?;
    w.name(&host)?;

    // <hostname>._telnet._tcp.local TXT "" (a single empty string)
    w.name(&inst)?;
    w.rr_header(TYPE_TXT, CLASS_IN_FLUSH, TTL_SERVICE, 1)?;
    w.put(&[0])?;

    Some(w.pos)
}

/// Encoded length of an uncompressed DNS name.
fn name_len(labels: &[&str]) -> u16 {
    (labels.iter().map(|l| 1 + l.len()).sum::<usize>() + 1) as u16
}

/// Tiny bounds-checked big-endian packet writer.
struct W<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl W<'_> {
    fn put(&mut self, bytes: &[u8]) -> Option<()> {
        let end = self.pos + bytes.len();
        self.buf.get_mut(self.pos..end)?.copy_from_slice(bytes);
        self.pos = end;
        Some(())
    }
    fn u16(&mut self, v: u16) -> Option<()> {
        self.put(&v.to_be_bytes())
    }
    fn u32(&mut self, v: u32) -> Option<()> {
        self.put(&v.to_be_bytes())
    }
    fn name(&mut self, labels: &[&str]) -> Option<()> {
        for l in labels {
            self.put(&[l.len() as u8])?;
            self.put(l.as_bytes())?;
        }
        self.put(&[0])
    }
    fn rr_header(&mut self, rtype: u16, class: u16, ttl: u32, rdlen: u16) -> Option<()> {
        self.u16(rtype)?;
        self.u16(class)?;
        self.u32(ttl)?;
        self.u16(rdlen)
    }
}
