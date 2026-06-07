//! MQTT telemetry + log sink — push live node logs/status to an optionally
//! configured broker, so observability needs no debug probe (the SWD pins are
//! the NinoTNC connection on the carrier board).
//!
//! A minimal **publish-only MQTT 3.1.1 over TCP** client (same hand-rolled
//! philosophy as the DHCP/DNS/mDNS servers): CONNECT → CONNACK → PUBLISH (QoS 0)
//! + periodic PINGREQ, reconnecting with backoff. No subscribe, no QoS 1/2 — a
//! node shouting telemetry, nothing more. Configured via `MQTT_HOST`
//! (`host:port`, port defaults to 1883) in flash config / the captive portal.
//!
//! Topics (retained where noted), under `pico-node/<callsign>/`:
//! - `status`  — periodic JSON (callsign, ip, mode, neighbours, dests, uptime)
//! - `log`     — a line per event pushed via [`log`] / the `nlog!` macro
//!
//! `log()` is non-blocking (drops on a full queue), so the hot paths can call it
//! freely; the task drains the queue and publishes.

use embassy_net::tcp::TcpSocket;
use embassy_net::{IpEndpoint, Stack};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Timer};

use alloc::string::String;

use crate::transports::parse_endpoint;

/// A line of node log text destined for MQTT. Bounded length keeps it cheap.
pub type LogLine = heapless::String<96>;

/// The log queue: hot paths push, the MQTT task drains. Bounded — `log()` drops
/// on overflow rather than blocking (telemetry must never stall the node).
static LOG_QUEUE: Channel<CriticalSectionRawMutex, LogLine, 16> = Channel::new();

/// Push a log line toward MQTT (no-op if the queue is full or MQTT is disabled —
/// the line is still on defmt/RTT via the caller). Non-blocking.
pub fn log(line: &str) {
    if let Ok(s) = LogLine::try_from(line) {
        let _ = LOG_QUEUE.try_send(s);
    }
}

/// defmt-and-MQTT tee: logs to defmt at info level AND pushes to the MQTT queue.
/// Use at the high-value event points; takes a pre-formatted `&str`.
#[macro_export]
macro_rules! nlog {
    ($($arg:tt)*) => {{
        let s: alloc::string::String = alloc::format!($($arg)*);
        defmt::info!("{=str}", s.as_str());
        $crate::mqtt::log(&s);
    }};
}

/// Live status the MQTT task publishes. Mirrors the OLED status; the owner
/// updates it via [`set_status`].
#[derive(Clone, Copy, Default)]
pub struct Status {
    pub neighbours: u16,
    pub destinations: u16,
}

static STATUS: embassy_sync::blocking_mutex::Mutex<
    CriticalSectionRawMutex,
    core::cell::RefCell<Status>,
> = embassy_sync::blocking_mutex::Mutex::new(core::cell::RefCell::new(Status {
    neighbours: 0,
    destinations: 0,
}));

/// Update the published NET/ROM counts.
pub fn set_status(neighbours: u16, destinations: u16) {
    STATUS.lock(|c| {
        let mut s = c.borrow_mut();
        s.neighbours = neighbours;
        s.destinations = destinations;
    });
}

/// Config for the MQTT task.
pub struct MqttConfig {
    /// `host:port` (port optional, defaults 1883). Empty/None ⇒ task disabled.
    pub host: Option<&'static str>,
    /// The node callsign — the topic root + client id.
    pub callsign: heapless::String<12>,
}

const STATUS_INTERVAL: Duration = Duration::from_secs(30);
const KEEPALIVE_SECS: u16 = 60;

#[embassy_executor::task]
pub async fn task(stack: Stack<'static>, cfg: MqttConfig) {
    let Some(host) = cfg.host else {
        return; // MQTT not configured — nothing to do.
    };
    // host:port — reuse the transport endpoint parser; default port 1883.
    let endpoint = parse_endpoint(host).or_else(|| {
        let mut h = heapless::String::<32>::new();
        let _ = core::fmt::Write::write_fmt(&mut h, format_args!("{}:1883", host));
        parse_endpoint(&h)
    });
    let Some(endpoint) = endpoint else {
        defmt::warn!("mqtt: bad MQTT_HOST — disabled");
        return;
    };
    defmt::info!("mqtt: publishing to {:?} as {=str}", endpoint, cfg.callsign.as_str());

    let mut rx_buf = [0u8; 512];
    let mut tx_buf = [0u8; 1024];
    let mut backoff = 1u64;
    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buf, &mut tx_buf);
        socket.set_timeout(Some(Duration::from_secs(KEEPALIVE_SECS as u64 + 10)));
        if socket.connect(endpoint).await.is_err() {
            Timer::after_secs(backoff).await;
            backoff = (backoff * 2).min(60);
            continue;
        }
        if connect_session(&mut socket, &cfg.callsign).await.is_err() {
            socket.close();
            Timer::after_secs(backoff).await;
            backoff = (backoff * 2).min(60);
            continue;
        }
        backoff = 1;
        defmt::info!("mqtt: connected");
        log("node online");

        serve(&mut socket, &cfg.callsign).await;
        socket.close();
        defmt::warn!("mqtt: disconnected, reconnecting");
    }
}

/// The publish loop: drain log lines, publish status on a cadence, PINGREQ to
/// keep the session, until the socket errors.
async fn serve(socket: &mut TcpSocket<'_>, callsign: &str) {
    let mut next_status = Instant::now();
    let mut next_ping = Instant::now() + Duration::from_secs(KEEPALIVE_SECS as u64 / 2);
    let mut pkt = [0u8; 256];
    loop {
        // A pending log line (with a short wait so we also service timers).
        let line = embassy_time::with_timeout(Duration::from_secs(1), LOG_QUEUE.receive()).await;
        if let Ok(line) = line {
            let mut topic = heapless::String::<48>::new();
            let _ = core::fmt::Write::write_fmt(
                &mut topic,
                format_args!("pico-node/{}/log", callsign),
            );
            if publish(socket, &mut pkt, &topic, line.as_bytes(), false).await.is_err() {
                return;
            }
        }

        let now = Instant::now();
        if now >= next_status {
            next_status = now + STATUS_INTERVAL;
            let st = STATUS.lock(|c| *c.borrow());
            let ip = socket.local_endpoint().map(|e| e.addr);
            let mut payload = String::new();
            let _ = core::fmt::Write::write_fmt(
                &mut payload,
                format_args!(
                    "{{\"call\":\"{}\",\"ip\":\"{}\",\"neighbours\":{},\"destinations\":{},\"uptime\":{}}}",
                    callsign,
                    ip.map(DisplayAddr).unwrap_or(DisplayAddr(unspecified())),
                    st.neighbours,
                    st.destinations,
                    now.as_secs(),
                ),
            );
            let mut topic = heapless::String::<48>::new();
            let _ = core::fmt::Write::write_fmt(
                &mut topic,
                format_args!("pico-node/{}/status", callsign),
            );
            if publish(socket, &mut pkt, &topic, payload.as_bytes(), true).await.is_err() {
                return;
            }
        }
        if now >= next_ping {
            next_ping = now + Duration::from_secs(KEEPALIVE_SECS as u64 / 2);
            // PINGREQ = 0xC0 0x00.
            if write_all(socket, &[0xC0, 0x00]).await.is_err() {
                return;
            }
        }
    }
}

// ── minimal MQTT 3.1.1 packet construction (publish-only) ──

async fn connect_session(socket: &mut TcpSocket<'_>, client_id: &str) -> Result<(), ()> {
    // Variable header: protocol name "MQTT", level 4, flags (clean session),
    // keepalive; payload: client id.
    let mut vh = heapless::Vec::<u8, 64>::new();
    let _ = vh.extend_from_slice(&[0x00, 0x04, b'M', b'Q', b'T', b'T', 0x04, 0x02]);
    let _ = vh.extend_from_slice(&KEEPALIVE_SECS.to_be_bytes());
    let _ = vh.extend_from_slice(&(client_id.len() as u16).to_be_bytes());
    let _ = vh.extend_from_slice(client_id.as_bytes());

    let mut pkt = heapless::Vec::<u8, 80>::new();
    let _ = pkt.push(0x10); // CONNECT
    push_remaining_len(&mut pkt, vh.len());
    let _ = pkt.extend_from_slice(&vh);
    write_all(socket, &pkt).await.map_err(|_| ())?;

    // Expect CONNACK (0x20 0x02 flags rc), rc==0.
    let mut resp = [0u8; 4];
    let n = socket.read(&mut resp).await.map_err(|_| ())?;
    if n >= 4 && resp[0] == 0x20 && resp[3] == 0x00 {
        Ok(())
    } else {
        Err(())
    }
}

async fn publish(
    socket: &mut TcpSocket<'_>,
    buf: &mut [u8; 256],
    topic: &str,
    payload: &[u8],
    retain: bool,
) -> Result<(), ()> {
    // Fixed header byte: PUBLISH | retain bit. QoS 0 ⇒ no packet id.
    let header = 0x30 | (retain as u8);
    let mut body = heapless::Vec::<u8, 256>::new();
    let _ = body.extend_from_slice(&(topic.len() as u16).to_be_bytes());
    let _ = body.extend_from_slice(topic.as_bytes());
    if body.extend_from_slice(payload).is_err() {
        return Ok(()); // payload too large — skip, not fatal
    }

    let mut at = 0usize;
    buf[at] = header;
    at += 1;
    at += write_remaining_len(&mut buf[at..], body.len());
    if at + body.len() > buf.len() {
        return Ok(());
    }
    buf[at..at + body.len()].copy_from_slice(&body);
    write_all(socket, &buf[..at + body.len()]).await.map_err(|_| ())
}

fn push_remaining_len(v: &mut heapless::Vec<u8, 80>, mut len: usize) {
    loop {
        let mut byte = (len & 0x7F) as u8;
        len >>= 7;
        if len > 0 {
            byte |= 0x80;
        }
        let _ = v.push(byte);
        if len == 0 {
            break;
        }
    }
}

fn write_remaining_len(out: &mut [u8], mut len: usize) -> usize {
    let mut i = 0;
    loop {
        let mut byte = (len & 0x7F) as u8;
        len >>= 7;
        if len > 0 {
            byte |= 0x80;
        }
        out[i] = byte;
        i += 1;
        if len == 0 {
            break i;
        }
    }
}

async fn write_all(socket: &mut TcpSocket<'_>, mut bytes: &[u8]) -> Result<(), ()> {
    while !bytes.is_empty() {
        match socket.write(bytes).await {
            Ok(0) | Err(_) => return Err(()),
            Ok(n) => bytes = &bytes[n..],
        }
    }
    Ok(())
}

// Small helper so the status JSON can format an IpAddress without alloc churn.
struct DisplayAddr(embassy_net::IpAddress);
impl core::fmt::Display for DisplayAddr {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self.0 {
            embassy_net::IpAddress::Ipv4(a) => {
                let o = a.octets();
                write!(f, "{}.{}.{}.{}", o[0], o[1], o[2], o[3])
            }
        }
    }
}
fn unspecified() -> embassy_net::IpAddress {
    embassy_net::IpAddress::Ipv4(embassy_net::Ipv4Address::new(0, 0, 0, 0))
}

// Endpoint is `host:port`; this re-exports the parse for the doc above.
#[allow(unused_imports)]
use embassy_net::IpListenEndpoint as _IpListenEndpoint;
#[allow(dead_code)]
fn _ep_marker(_: IpEndpoint) {}
