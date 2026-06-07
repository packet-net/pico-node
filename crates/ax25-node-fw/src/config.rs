//! Firmware configuration — the on-target analogue of `Packet.Node.Core.Configuration`.
//!
//! The C# node loads YAML from disk behind an `IConfigProvider` seam. On the
//! Pico there is no filesystem by default; the equivalent options (a follow-up
//! decision, see docs/PLAN.md) are: compile-time `const` config, a TOML/JSON blob
//! in a reserved flash sector read at boot, or fetched over the network. For the
//! skeleton this is a struct mirroring `NodeConfig`'s fields with a compiled-in
//! default loader.
//!
//! STUB — fields + shape only; the persistent loader is a follow-up.

// GATE 1 (HW-BRINGUP.md §4): the wifi/axudp/kiss/telnet fields' only consumers
// (net.rs + transports) are gated out until Gates 2–6; keep the full config shape
// without dead-code noise meanwhile. Remove this allow when the transports return.
#![allow(dead_code)]

use ax25_node_core::ax25::Callsign;

/// Complete node config (mirrors `NodeConfig`).
#[derive(Clone)]
pub struct NodeConfig {
    pub identity: Identity,
    /// LAN hostname: the DHCP option-12 name and the mDNS `<hostname>.local`
    /// label — how the node is found on the WLAN without knowing its IP.
    pub hostname: &'static str,
    pub wifi: WifiConfig,
    pub axudp: AxudpConfig,
    pub kiss_tcp: KissTcpConfig,
    pub kiss_serial: KissSerialConfig,
    pub telnet: TelnetConfig,
    pub netrom: NetRomConfig,
}

#[derive(Clone)]
pub struct Identity {
    /// The node's on-air callsign. When [`callsign_configured`](Self::callsign_configured)
    /// is `false` this holds the inert placeholder `N0CALL` and the node runs
    /// **config-only** — it refuses to operate any on-air service (no beacons,
    /// NODES, AX.25 sessions, interlinks) because transmitting without a
    /// licensed callsign is illegal. Set a callsign via the captive portal or
    /// the console `SET CALLSIGN` to bring the node up.
    pub callsign: Callsign,
    /// `true` once a real callsign has been configured (flash or build env).
    /// `false` on a fresh node ⇒ provisioning-required mode.
    pub callsign_configured: bool,
    pub alias: &'static str,
    pub grid: &'static str,
}

#[derive(Clone)]
pub struct WifiConfig {
    pub ssid: &'static str,
    pub password: &'static str,
    /// WPA2 passphrase for the node's own config AP (provisioning fallback —
    /// docs/PROVISIONING.md). A well-known default for field usability; the AP
    /// guards configuration, not radio traffic (which is cleartext by law).
    pub ap_passphrase: &'static str,
}

/// AXUDP node↔node (capability 1).
#[derive(Clone)]
pub struct AxudpConfig {
    pub listen_port: u16,
    /// Optional `"a.b.c.d:port"` endpoint to beacon UI frames at (the Gate-3
    /// host harness). From the build env (`AXUDP_BEACON_TARGET`) — a LAN
    /// detail, never a committed default (HW-BRINGUP §5).
    pub beacon_target: Option<&'static str>,
}

/// KISS-over-TCP to net-sim (capability 2).
#[derive(Clone)]
pub struct KissTcpConfig {
    /// Optional `"a.b.c.d:port"` KISS-TCP endpoint to connect to. From the
    /// build env (`KISS_TCP_TARGET`) — a LAN detail, never a committed default
    /// (HW-BRINGUP §5). Absent ⇒ the transport is disabled.
    pub target: Option<&'static str>,
}

/// KISS-over-UART to a NinoTNC (capability 3).
#[derive(Clone)]
pub struct KissSerialConfig {
    pub baud: u32,
}

/// Telnet command console (capability 4).
#[derive(Clone)]
pub struct TelnetConfig {
    pub port: u16,
}

/// NET/ROM behaviour. The tap (hearing NODES) is always on; origination
/// (broadcasting our own NODES) is the node opt-in, per the library default.
#[derive(Clone)]
pub struct NetRomConfig {
    /// Originate NODES broadcasts (the C# `netRom.broadcast` opt-in).
    pub originate: bool,
    /// Seconds between NODES broadcasts. BPQ convention is minutes; the lab
    /// runs short. Overridable at build time via `NODES_INTERVAL_SECS`.
    pub nodes_interval_secs: u32,
}

/// Load the node config. STUB: returns a compiled-in default. A real loader
/// (flash sector / network) is the follow-up.
pub fn load() -> NodeConfig {
    NodeConfig {
        hostname: "pico-node",
        identity: {
            // NO compiled-in callsign by default: the node must be told its
            // callsign (captive portal / console SET) before it does anything on
            // the air. A build-env NODE_CALLSIGN is honoured for dev/CI rigs.
            let env_call = option_env!("NODE_CALLSIGN").and_then(Callsign::parse);
            Identity {
                callsign: env_call
                    .unwrap_or_else(|| Callsign::parse("N0CALL").expect("placeholder")),
                callsign_configured: env_call.is_some(),
                alias: option_env!("NODE_ALIAS").unwrap_or("PICO"),
                grid: option_env!("NODE_GRID").unwrap_or(""),
            }
        },
        // §5 secrets policy (HW-BRINGUP.md): WiFi credentials are read from the
        // BUILD environment, never committed. Missing creds still build (CI has
        // no secrets) — net::join fails loudly at boot instead.
        wifi: WifiConfig {
            ssid: option_env!("WIFI_SSID").unwrap_or(""),
            password: option_env!("WIFI_PASSWORD").unwrap_or(""),
            ap_passphrase: option_env!("AP_PASSPHRASE").unwrap_or("pico-node-config"),
        },
        axudp: AxudpConfig {
            listen_port: 10093,
            beacon_target: option_env!("AXUDP_BEACON_TARGET"),
        },
        kiss_tcp: KissTcpConfig {
            target: option_env!("KISS_TCP_TARGET"),
        },
        kiss_serial: KissSerialConfig { baud: 57600 },
        telnet: TelnetConfig { port: 8023 },
        netrom: NetRomConfig {
            originate: true,
            nodes_interval_secs: parse_u32(option_env!("NODES_INTERVAL_SECS"), 300),
        },
    }
}

/// Overlay a flash-stored config onto the compiled defaults. Strings are
/// leaked to `&'static str` — boot-once config, the whole node captures it at
/// startup (REBOOT applies changes), so the leak is the lifetime model.
pub fn apply_stored(cfg: &mut NodeConfig, st: &crate::config_store::StoredConfig) {
    fn leak<const N: usize>(s: &heapless::String<N>) -> &'static str {
        alloc::boxed::Box::leak(alloc::string::String::from(s.as_str()).into_boxed_str())
    }
    if let Some(v) = &st.callsign {
        if let Some(call) = Callsign::parse(v.as_str()) {
            cfg.identity.callsign = call;
            cfg.identity.callsign_configured = true;
        }
    }
    if let Some(v) = &st.alias {
        cfg.identity.alias = leak(v);
    }
    if let Some(v) = &st.grid {
        cfg.identity.grid = leak(v);
    }
    if let Some(v) = &st.hostname {
        cfg.hostname = leak(v);
    }
    if let Some(v) = &st.wifi_ssid {
        cfg.wifi.ssid = leak(v);
    }
    if let Some(v) = &st.wifi_pass {
        cfg.wifi.password = leak(v);
    }
    if let Some(v) = &st.beacon_target {
        cfg.axudp.beacon_target = Some(leak(v));
    }
    if let Some(v) = &st.kiss_tcp_target {
        cfg.kiss_tcp.target = Some(leak(v));
    }
    if let Some(v) = st.axudp_port {
        cfg.axudp.listen_port = v;
    }
    if let Some(v) = st.telnet_port {
        cfg.telnet.port = v;
    }
    if let Some(v) = st.nodes_interval_secs {
        cfg.netrom.nodes_interval_secs = v;
    }
    if let Some(v) = st.originate {
        cfg.netrom.originate = v;
    }
}

/// Parse an optional build-env decimal, falling back on absence or garbage.
fn parse_u32(s: Option<&str>, default: u32) -> u32 {
    match s {
        Some(v) => v.parse().unwrap_or(default),
        None => default,
    }
}
