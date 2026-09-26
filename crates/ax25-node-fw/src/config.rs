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

// Some fields are consumed only by optional tasks (Tait, MQTT) or only on some
// builds; keep the whole shape without dead-code noise.
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
    pub ninotnc: NinoTncConfig,
    pub kiss_tcp: KissTcpConfig,
    pub tait: TaitConfig,
    pub telnet: TelnetConfig,
    pub netrom: NetRomConfig,
    pub power: PowerConfig,
    /// Optional `host[:port]` of an MQTT broker to publish logs/status to
    /// (observability without a debug probe). From `MQTT_HOST` / flash config;
    /// absent ⇒ no MQTT.
    pub mqtt_host: Option<&'static str>,
    /// When `true`, come up in the config AP instead of joining WiFi even though
    /// WiFi is configured (the web panel's "Switch to setup AP" maintenance
    /// action). Sticky in flash; cleared by the next config save.
    pub force_ap: bool,
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

/// Port 1: KISS-over-TCP (net-sim, or a TNC behind a KISS-TCP server).
#[derive(Clone)]
pub struct KissTcpConfig {
    /// Optional `"a.b.c.d:port"` KISS-TCP endpoint to connect to. From the
    /// build env (`KISS_TCP_TARGET`) — a LAN detail, never a committed default
    /// (HW-BRINGUP §5). Absent: the node has no such port.
    pub target: Option<&'static str>,
}

/// Port 0: the NinoTNC on the serial link.
#[derive(Clone)]
pub struct NinoTncConfig {
    pub baud: u32,
    /// Optional NinoTNC operating mode to set at boot via KISS SETHW (RAM-only —
    /// spares flash). `None` (the default) leaves the modem's own configured mode
    /// untouched. From the build env `NINOTNC_MODE`; a §policy knob so the node can
    /// force a known modem mode at startup. Values > 15 are rejected by SETHW.
    pub startup_mode: Option<u8>,
    /// The NinoTNC settings saved on the node (the `/tnc` page / console keys),
    /// re-applied every boot. A saved `mode` wins over `startup_mode`.
    pub tnc: crate::config_store::TncSettings,
}

/// CCDI-controlled Tait radio on a second UART (radio integration).
#[derive(Clone)]
pub struct TaitConfig {
    /// CCDI serial rate. The radio's programmed rate wins; default
    /// [`ax25_node_core::radio::tait::DEFAULT_BAUD`] (28 800). Overridable at build
    /// time via `TAIT_BAUD`.
    pub baud: u32,
    /// Optional programmed channel to select at boot (GO_TO_CHANNEL). `None` leaves
    /// the radio on its current channel. From the build env `TAIT_CHANNEL`.
    pub channel: Option<u16>,
    /// Seconds between RSSI polls — also the cadence at which interleaved
    /// carrier-sense / PTT PROGRESS edges are drained.
    pub rssi_poll_secs: u64,
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
    /// Seconds between NODES broadcasts, which is also the obsolescence sweep
    /// cadence (BPQ NODESINTERVAL). Default 3600 (hourly), packet.net's default:
    /// routes age one step per interval, so a short interval ages routes learned
    /// from slower-broadcasting neighbours below the advertise threshold between
    /// their broadcasts (seen on air at 300 s). Overridable at build time via
    /// `NODES_INTERVAL_SECS` and in config (`NODES_INTERVAL`).
    pub nodes_interval_secs: u32,
}

/// Station power monitoring: an INA226 on the I2C bus (GP4/GP5), reported as
/// APRS telemetry ([`crate::power`]).
#[derive(Clone)]
pub struct PowerConfig {
    /// Minutes between APRS telemetry reports; 0 = don't send. Default 10.
    /// From config (`TELEM_INTERVAL`).
    pub telemetry_interval_min: u16,
    /// The INA226's current shunt, in micro-ohms. Default 100 000 (the 0.1 ohm
    /// "R100" shunt on the common breakout boards). From config (`SHUNT_MOHM`,
    /// milliohms).
    pub shunt_micro_ohm: u32,
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
            ap_passphrase: option_env!("AP_PASSPHRASE").unwrap_or("packetradio"),
        },
        kiss_tcp: KissTcpConfig {
            // The release build sets these to "" (no LAN details baked in):
            // empty means unset.
            target: option_env!("KISS_TCP_TARGET").filter(|s| !s.is_empty()),
        },
        ninotnc: NinoTncConfig {
            baud: 57600,
            startup_mode: option_env!("NINOTNC_MODE").and_then(|s| s.parse::<u8>().ok()),
            tnc: crate::config_store::TncSettings::default(),
        },
        tait: TaitConfig {
            baud: parse_u32(
                option_env!("TAIT_BAUD"),
                ax25_node_core::radio::tait::DEFAULT_BAUD,
            ),
            channel: option_env!("TAIT_CHANNEL").and_then(|s| s.parse::<u16>().ok()),
            rssi_poll_secs: 5,
        },
        telnet: TelnetConfig { port: 8023 },
        netrom: NetRomConfig {
            originate: true,
            nodes_interval_secs: parse_u32(option_env!("NODES_INTERVAL_SECS"), 3600),
        },
        power: PowerConfig {
            telemetry_interval_min: 10,
            shunt_micro_ohm: 100_000,
        },
        mqtt_host: option_env!("MQTT_HOST").filter(|s| !s.is_empty()),
        force_ap: false,
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
    if let Some(v) = &st.kiss_tcp_target {
        cfg.kiss_tcp.target = Some(leak(v));
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
    if let Some(v) = &st.mqtt_host {
        cfg.mqtt_host = Some(leak(v));
    }
    if let Some(v) = st.force_ap {
        cfg.force_ap = v;
    }
    cfg.ninotnc.tnc = st.tnc;
    if let Some(v) = st.telemetry_interval_min {
        cfg.power.telemetry_interval_min = v;
    }
    if let Some(v) = st.shunt_micro_ohm {
        cfg.power.shunt_micro_ohm = v;
    }
}

/// Parse an optional build-env decimal, falling back on absence or garbage.
fn parse_u32(s: Option<&str>, default: u32) -> u32 {
    match s {
        Some(v) => v.parse().unwrap_or(default),
        None => default,
    }
}
