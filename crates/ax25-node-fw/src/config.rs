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
        wifi: WifiConfig {
            ssid: "set-me",
            password: "set-me",
        },
        axudp: AxudpConfig {
            listen_port: 10093,
            include_fcs: false,
        },
        kiss_tcp: KissTcpConfig {
            host: "192.168.1.10",
            port: 8001,
        },
        kiss_serial: KissSerialConfig { baud: 57600 },
        telnet: TelnetConfig { port: 8023 },
    }
}
