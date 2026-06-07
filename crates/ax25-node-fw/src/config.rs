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
    pub callsign: Callsign,
    pub alias: &'static str,
    pub grid: &'static str,
}

#[derive(Clone)]
pub struct WifiConfig {
    pub ssid: &'static str,
    pub password: &'static str,
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
        identity: Identity {
            callsign: Callsign::parse("M0LTE-1").expect("valid default callsign"),
            alias: "PICO",
            grid: "IO91wm",
        },
        // §5 secrets policy (HW-BRINGUP.md): WiFi credentials are read from the
        // BUILD environment, never committed. Missing creds still build (CI has
        // no secrets) — net::join fails loudly at boot instead.
        wifi: WifiConfig {
            ssid: option_env!("WIFI_SSID").unwrap_or(""),
            password: option_env!("WIFI_PASSWORD").unwrap_or(""),
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

/// Parse an optional build-env decimal, falling back on absence or garbage.
fn parse_u32(s: Option<&str>, default: u32) -> u32 {
    match s {
        Some(v) => v.parse().unwrap_or(default),
        None => default,
    }
}
