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
    pub wifi: WifiConfig,
    pub axudp: AxudpConfig,
    pub kiss_tcp: KissTcpConfig,
    pub kiss_serial: KissSerialConfig,
    pub telnet: TelnetConfig,
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
    pub include_fcs: bool, // XRouter AXIP-with-CRC variant
    /// Optional `"a.b.c.d:port"` endpoint to beacon UI frames at (the Gate-3
    /// host harness). From the build env (`AXUDP_BEACON_TARGET`) — a LAN
    /// detail, never a committed default (HW-BRINGUP §5).
    pub beacon_target: Option<&'static str>,
}

/// KISS-over-TCP to net-sim (capability 2).
#[derive(Clone)]
pub struct KissTcpConfig {
    pub host: &'static str,
    pub port: u16,
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

/// Load the node config. STUB: returns a compiled-in default. A real loader
/// (flash sector / network) is the follow-up.
pub fn load() -> NodeConfig {
    NodeConfig {
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
            include_fcs: false,
            beacon_target: option_env!("AXUDP_BEACON_TARGET"),
        },
        kiss_tcp: KissTcpConfig {
            host: "192.168.1.10",
            port: 8001,
        },
        kiss_serial: KissSerialConfig { baud: 57600 },
        telnet: TelnetConfig { port: 8023 },
    }
}
