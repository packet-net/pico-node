//! Persisted node configuration — flash store + console-editable pending copy.
//!
//! Provisioning steps 1–2 (docs/PROVISIONING.md): the top 8 KiB of flash (two
//! 4 KiB sectors, kept out of the linker's FLASH region by `memory.x`) hold a
//! versioned config record. Writes alternate sectors with a monotonically
//! increasing generation and the old record is left intact until the new one
//! is fully written — a torn write can never lose the previous config. Boot
//! picks the highest-generation valid record.
//!
//! Record layout (little-endian):
//! `"PNC1" | version u8 | generation u32 | len u16 | payload | crc16`
//! where the CRC-16/X.25 covers everything before it, and the payload is a
//! sequence of `tag u8 | len u8 | bytes` fields (unknown tags are skipped on
//! read — forward compatible).
//!
//! Console semantics (the firmware-side executor for
//! [`ax25_node_core::console::ConfigOp`]): `SET` stages into the pending copy
//! in RAM, `SAVE` persists, `REBOOT` applies (boot-time config stays immutable
//! while running — every consumer captured it at startup).

use ax25_node_core::ax25::Callsign;
use ax25_node_core::crc;

use alloc::format;
use alloc::string::String;

use core::cell::RefCell;

use embassy_rp::flash::{Blocking, Flash, ERASE_SIZE};
use embassy_rp::peripherals::FLASH;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;

/// Total flash size the driver is instantiated over (stock Pico W).
pub const FLASH_SIZE: usize = 2 * 1024 * 1024;
/// Byte offsets (from flash base) of the two config sectors — the 8 KiB that
/// `memory.x` keeps out of the program region.
const SECTOR_A: u32 = (FLASH_SIZE - 2 * ERASE_SIZE) as u32;
const SECTOR_B: u32 = (FLASH_SIZE - ERASE_SIZE) as u32;

const MAGIC: &[u8; 4] = b"PNC1";
const VERSION: u8 = 1;
/// Header is magic(4) + version(1) + generation(4) + len(2).
const HEADER_LEN: usize = 11;
const MAX_PAYLOAD: usize = 512;

// Field tags (never reuse a retired tag).
const TAG_CALLSIGN: u8 = 1;
const TAG_ALIAS: u8 = 2;
const TAG_GRID: u8 = 3;
const TAG_HOSTNAME: u8 = 4;
const TAG_WIFI_SSID: u8 = 5;
const TAG_WIFI_PASS: u8 = 6;
const TAG_BEACON_TARGET: u8 = 7;
const TAG_KISS_TCP_TARGET: u8 = 8;
const TAG_AXUDP_PORT: u8 = 9;
const TAG_TELNET_PORT: u8 = 10;
const TAG_NODES_INTERVAL: u8 = 11;
const TAG_ORIGINATE: u8 = 12;
const TAG_MQTT_HOST: u8 = 13;

/// The persisted fields. Every field optional: only explicitly-set values are
/// stored, everything else falls back to the compiled factory defaults.
#[derive(Clone, Default)]
pub struct StoredConfig {
    pub callsign: Option<heapless::String<12>>,
    pub alias: Option<heapless::String<8>>,
    pub grid: Option<heapless::String<8>>,
    pub hostname: Option<heapless::String<24>>,
    pub wifi_ssid: Option<heapless::String<32>>,
    pub wifi_pass: Option<heapless::String<64>>,
    pub beacon_target: Option<heapless::String<24>>,
    pub kiss_tcp_target: Option<heapless::String<24>>,
    pub axudp_port: Option<u16>,
    pub telnet_port: Option<u16>,
    pub nodes_interval_secs: Option<u32>,
    pub originate: Option<bool>,
    pub mqtt_host: Option<heapless::String<32>>,
}

impl StoredConfig {
    /// Encode into `buf` (payload TLVs only); returns the payload length.
    pub fn encode(&self, buf: &mut [u8; MAX_PAYLOAD]) -> usize {
        let mut at = 0usize;
        let mut put = |tag: u8, data: &[u8]| {
            if at + 2 + data.len() <= MAX_PAYLOAD {
                buf[at] = tag;
                buf[at + 1] = data.len() as u8;
                buf[at + 2..at + 2 + data.len()].copy_from_slice(data);
                at += 2 + data.len();
            }
        };
        macro_rules! put_str_field {
            ($tag:expr, $field:expr) => {
                if let Some(s) = &$field {
                    put($tag, s.as_bytes());
                }
            };
        }
        put_str_field!(TAG_CALLSIGN, self.callsign);
        put_str_field!(TAG_ALIAS, self.alias);
        put_str_field!(TAG_GRID, self.grid);
        put_str_field!(TAG_HOSTNAME, self.hostname);
        put_str_field!(TAG_WIFI_SSID, self.wifi_ssid);
        put_str_field!(TAG_WIFI_PASS, self.wifi_pass);
        put_str_field!(TAG_BEACON_TARGET, self.beacon_target);
        put_str_field!(TAG_KISS_TCP_TARGET, self.kiss_tcp_target);
        if let Some(v) = self.axudp_port {
            put(TAG_AXUDP_PORT, &v.to_le_bytes());
        }
        if let Some(v) = self.telnet_port {
            put(TAG_TELNET_PORT, &v.to_le_bytes());
        }
        if let Some(v) = self.nodes_interval_secs {
            put(TAG_NODES_INTERVAL, &v.to_le_bytes());
        }
        if let Some(v) = self.originate {
            put(TAG_ORIGINATE, &[v as u8]);
        }
        if let Some(s) = &self.mqtt_host {
            put(TAG_MQTT_HOST, s.as_bytes());
        }
        at
    }

    /// Decode a payload (TLVs). Unknown tags are skipped; malformed TLV
    /// framing truncates the read (best effort, total).
    pub fn decode(payload: &[u8]) -> Self {
        let mut out = Self::default();
        let mut at = 0usize;
        while at + 2 <= payload.len() {
            let tag = payload[at];
            let len = payload[at + 1] as usize;
            if at + 2 + len > payload.len() {
                break;
            }
            let data = &payload[at + 2..at + 2 + len];
            at += 2 + len;

            fn s<const N: usize>(data: &[u8]) -> Option<heapless::String<N>> {
                core::str::from_utf8(data)
                    .ok()
                    .and_then(|t| heapless::String::try_from(t).ok())
            }
            match tag {
                TAG_CALLSIGN => out.callsign = s(data),
                TAG_ALIAS => out.alias = s(data),
                TAG_GRID => out.grid = s(data),
                TAG_HOSTNAME => out.hostname = s(data),
                TAG_WIFI_SSID => out.wifi_ssid = s(data),
                TAG_WIFI_PASS => out.wifi_pass = s(data),
                TAG_BEACON_TARGET => out.beacon_target = s(data),
                TAG_KISS_TCP_TARGET => out.kiss_tcp_target = s(data),
                TAG_AXUDP_PORT if len == 2 => {
                    out.axudp_port = Some(u16::from_le_bytes([data[0], data[1]]))
                }
                TAG_TELNET_PORT if len == 2 => {
                    out.telnet_port = Some(u16::from_le_bytes([data[0], data[1]]))
                }
                TAG_NODES_INTERVAL if len == 4 => {
                    out.nodes_interval_secs =
                        Some(u32::from_le_bytes([data[0], data[1], data[2], data[3]]))
                }
                TAG_ORIGINATE if len == 1 => out.originate = Some(data[0] != 0),
                TAG_MQTT_HOST => out.mqtt_host = s(data),
                _ => {} // unknown/short tag: skip (forward compatibility)
            }
        }
        out
    }
}

/// The blocking flash driver over the whole 2 MB chip. Public so the OTA path
/// (`crate::ota`) can take ownership of it to drive `embassy-boot`'s
/// `FirmwareUpdater` (see [`take_flash_for_ota`]).
pub type ConfigFlash = Flash<'static, FLASH, Blocking, FLASH_SIZE>;

/// The flash-backed config service: the driver, the pending (console-staged)
/// copy, and the generation counter for A/B writes.
pub struct ConfigService {
    flash: ConfigFlash,
    /// The console-staged copy (starts as the stored record, or empty).
    pub pending: StoredConfig,
    generation: u32,
    /// Generation counter for the NET/ROM routing store (separate sectors).
    netrom_generation: u32,
    /// Content CRC of the last persisted routing table — the wear gate (a save
    /// whose content matches this writes no flash).
    netrom_content_crc: u16,
}

/// Global handle for the console executors (telnet + AX.25 console both reach
/// it; flash ops run inside the critical section — they're rare, user-driven,
/// and the RP2040 pauses XIP during them anyway).
pub static CONFIG: Mutex<CriticalSectionRawMutex, RefCell<Option<ConfigService>>> =
    Mutex::new(RefCell::new(None));

/// Read one sector's record. Returns `(generation, config)` when valid.
fn read_sector(flash: &mut ConfigFlash, offset: u32) -> Option<(u32, StoredConfig)> {
    let mut header = [0u8; HEADER_LEN];
    flash.blocking_read(offset, &mut header).ok()?;
    if &header[0..4] != MAGIC || header[4] != VERSION {
        return None;
    }
    let generation = u32::from_le_bytes([header[5], header[6], header[7], header[8]]);
    let len = u16::from_le_bytes([header[9], header[10]]) as usize;
    if len > MAX_PAYLOAD {
        return None;
    }
    let mut rest = [0u8; MAX_PAYLOAD + 2];
    flash
        .blocking_read(offset + HEADER_LEN as u32, &mut rest[..len + 2])
        .ok()?;
    let stored_crc = u16::from_le_bytes([rest[len], rest[len + 1]]);
    // CRC covers header + payload.
    let mut crc_buf = [0u8; HEADER_LEN + MAX_PAYLOAD];
    crc_buf[..HEADER_LEN].copy_from_slice(&header);
    crc_buf[HEADER_LEN..HEADER_LEN + len].copy_from_slice(&rest[..len]);
    if crc::compute(&crc_buf[..HEADER_LEN + len]) != stored_crc {
        return None;
    }
    Some((generation, StoredConfig::decode(&rest[..len])))
}

/// Load the highest-generation valid record from either sector.
pub fn load(flash: &mut ConfigFlash) -> Option<(u32, StoredConfig)> {
    let a = read_sector(flash, SECTOR_A);
    let b = read_sector(flash, SECTOR_B);
    match (a, b) {
        (Some(a), Some(b)) => Some(if a.0 >= b.0 { a } else { b }),
        (a, b) => a.or(b),
    }
}

impl ConfigService {
    /// Initialise: read the store, seed pending with it.
    pub fn new(mut flash: ConfigFlash) -> (Self, Option<StoredConfig>) {
        let loaded = load(&mut flash);
        let (generation, stored) = match loaded {
            Some((g, c)) => (g, Some(c)),
            None => (0, None),
        };
        (
            ConfigService {
                flash,
                pending: stored.clone().unwrap_or_default(),
                generation,
                netrom_generation: 0,
                netrom_content_crc: 0,
            },
            stored,
        )
    }

    /// Tear down into the flash driver (test support: lets a fresh service
    /// re-read what this one wrote).
    #[allow(dead_code)] // bin build: consumed by tests/on_target.rs (+ the future portal)
    pub fn into_flash(self) -> ConfigFlash {
        self.flash
    }

    /// Erase both config sectors — back to factory defaults on next boot.
    /// (The provisioning flow's "factory reset"; also used by the on-target
    /// test so it never leaves its fixtures in the real store.)
    #[allow(dead_code)] // bin build: consumed by tests/on_target.rs (+ the future portal)
    pub fn factory_reset(&mut self) -> Result<(), &'static str> {
        self.flash
            .blocking_erase(SECTOR_A, SECTOR_A + ERASE_SIZE as u32)
            .map_err(|_| "erase A failed")?;
        self.flash
            .blocking_erase(SECTOR_B, SECTOR_B + ERASE_SIZE as u32)
            .map_err(|_| "erase B failed")?;
        self.pending = StoredConfig::default();
        self.generation = 0;
        Ok(())
    }

    /// Persist the pending config to the non-current sector with generation+1.
    pub fn save(&mut self) -> Result<(), &'static str> {
        let mut payload = [0u8; MAX_PAYLOAD];
        let len = self.pending.encode(&mut payload);
        let generation = self.generation + 1;

        let mut record = [0xFFu8; HEADER_LEN + MAX_PAYLOAD + 2];
        record[0..4].copy_from_slice(MAGIC);
        record[4] = VERSION;
        record[5..9].copy_from_slice(&generation.to_le_bytes());
        record[9..11].copy_from_slice(&(len as u16).to_le_bytes());
        record[HEADER_LEN..HEADER_LEN + len].copy_from_slice(&payload[..len]);
        let crc = crc::compute(&record[..HEADER_LEN + len]);
        record[HEADER_LEN + len..HEADER_LEN + len + 2].copy_from_slice(&crc.to_le_bytes());
        let total = HEADER_LEN + len + 2;
        // Pad to the flash write granularity (page-program friendly).
        let padded = total.div_ceil(256) * 256;

        // Alternate sectors by generation parity; the previous record survives
        // until this write completes.
        let offset = if generation % 2 == 0 {
            SECTOR_A
        } else {
            SECTOR_B
        };
        self.flash
            .blocking_erase(offset, offset + ERASE_SIZE as u32)
            .map_err(|_| "flash erase failed")?;
        self.flash
            .blocking_write(offset, &record[..padded])
            .map_err(|_| "flash write failed")?;

        // Read-back verify before adopting the new generation.
        match read_sector(&mut self.flash, offset) {
            Some((g, _)) if g == generation => {
                self.generation = generation;
                Ok(())
            }
            _ => Err("flash verify failed"),
        }
    }
}

/// Replay the persisted NET/ROM routing table into `netrom` at boot. Returns
/// the number of routes replayed (0 if none stored). Locks the config flash.
pub fn netrom_load(
    netrom: &mut crate::session::NetRom,
    my_call: ax25_node_core::ax25::Callsign,
) -> usize {
    CONFIG.lock(|cell| {
        let mut borrow = cell.borrow_mut();
        let Some(svc) = borrow.as_mut() else {
            return 0;
        };
        let (generation, replayed) = crate::netrom_store::load(&mut svc.flash, netrom, my_call);
        svc.netrom_generation = generation;
        replayed
    })
}

/// Persist the current routing table — but only if its content changed since
/// the last write (the flash-wear gate; a stable table writes nothing). Returns
/// the route count actually written (0 = unchanged or empty, no erase). Locks
/// the config flash.
pub fn netrom_save(netrom: &crate::session::NetRom) -> Result<usize, &'static str> {
    use crate::netrom_store::SaveOutcome;
    CONFIG.lock(|cell| {
        let mut borrow = cell.borrow_mut();
        let Some(svc) = borrow.as_mut() else {
            return Err("config store unavailable");
        };
        match crate::netrom_store::save(
            &mut svc.flash,
            netrom,
            svc.netrom_generation,
            svc.netrom_content_crc,
        )? {
            SaveOutcome::Wrote { count, content_crc } => {
                svc.netrom_generation += 1;
                svc.netrom_content_crc = content_crc;
                Ok(count)
            }
            SaveOutcome::Unchanged | SaveOutcome::Empty => Ok(0),
        }
    })
}

/// Take ownership of the flash driver for an OTA update, removing the
/// `ConfigService` from the global handle. Returns `None` if the store is
/// unavailable or already taken. The OTA path always ends in a reset, so the
/// service is never restored — config/routing saves no-op until the reboot.
pub fn take_flash_for_ota() -> Option<ConfigFlash> {
    CONFIG.lock(|cell| cell.borrow_mut().take().map(|svc| svc.into_flash()))
}

/// Execute one console [`ConfigOp`]: returns the response text (newline-
/// separated; the caller renders per transport) and whether a reboot was
/// requested (the caller flushes output first, then resets).
pub fn handle_op(op: &ax25_node_core::console::ConfigOp) -> (String, bool) {
    use ax25_node_core::console::ConfigOp;
    CONFIG.lock(|cell| {
        let mut borrow = cell.borrow_mut();
        let Some(svc) = borrow.as_mut() else {
            return (String::from("config store unavailable"), false);
        };
        match op {
            ConfigOp::Show => (render_show(&svc.pending), false),
            ConfigOp::Set { key, value } => (set_field(&mut svc.pending, key, value), false),
            ConfigOp::Save => match svc.save() {
                Ok(()) => (
                    String::from("Saved. REBOOT to apply (running config is boot-time)."),
                    false,
                ),
                Err(e) => (format!("Save failed: {e}"), false),
            },
            ConfigOp::Reboot => (String::from("Rebooting..."), true),
        }
    })
}

fn render_show(p: &StoredConfig) -> String {
    fn s_or(v: &Option<impl AsRef<str>>, def: &str) -> String {
        match v {
            Some(s) => String::from(s.as_ref()),
            None => format!("(default: {def})"),
        }
    }
    let mut out = String::from("Pending config (SET <key> <value>, SAVE, REBOOT):\n");
    out += &format!("  CALLSIGN       {}\n", s_or(&p.callsign, "factory"));
    out += &format!("  ALIAS          {}\n", s_or(&p.alias, "factory"));
    out += &format!("  GRID           {}\n", s_or(&p.grid, "factory"));
    out += &format!("  HOSTNAME       {}\n", s_or(&p.hostname, "factory"));
    out += &format!(
        "  WIFI_SSID      {}\n",
        s_or(&p.wifi_ssid, "factory/build-env")
    );
    out += &format!(
        "  WIFI_PASS      {}\n",
        match &p.wifi_pass {
            Some(_) => String::from("(set)"),
            None => String::from("(default: factory/build-env)"),
        }
    );
    out += &format!("  BEACON_TARGET  {}\n", s_or(&p.beacon_target, "build-env"));
    out += &format!(
        "  KISS_TCP       {}\n",
        s_or(&p.kiss_tcp_target, "build-env")
    );
    out += &match p.axudp_port {
        Some(v) => format!("  AXUDP_PORT     {v}\n"),
        None => String::from("  AXUDP_PORT     (default: 10093)\n"),
    };
    out += &match p.telnet_port {
        Some(v) => format!("  TELNET_PORT    {v}\n"),
        None => String::from("  TELNET_PORT    (default: 8023)\n"),
    };
    out += &match p.nodes_interval_secs {
        Some(v) => format!("  NODES_INTERVAL {v}\n"),
        None => String::from("  NODES_INTERVAL (default: 300)\n"),
    };
    out += &match p.originate {
        Some(v) => format!("  ORIGINATE      {v}\n"),
        None => String::from("  ORIGINATE      (default: true)\n"),
    };
    out += &match &p.mqtt_host {
        Some(v) => format!("  MQTT_HOST      {}", v.as_str()),
        None => String::from("  MQTT_HOST      (unset)"),
    };
    out
}

fn set_field(p: &mut StoredConfig, key: &str, value: &str) -> String {
    fn put<const N: usize>(
        slot: &mut Option<heapless::String<N>>,
        value: &str,
        what: &str,
    ) -> String {
        match heapless::String::try_from(value) {
            Ok(s) => {
                *slot = Some(s);
                format!("{what} staged. SAVE to persist.")
            }
            Err(_) => format!("{what} too long (max {N})"),
        }
    }
    match key {
        "CALLSIGN" => match Callsign::parse(value) {
            Some(_) => put(&mut p.callsign, value, "CALLSIGN"),
            None => String::from("not a valid callsign"),
        },
        "ALIAS" => put(&mut p.alias, value, "ALIAS"),
        "GRID" => put(&mut p.grid, value, "GRID"),
        "HOSTNAME" => put(&mut p.hostname, value, "HOSTNAME"),
        "WIFI_SSID" => put(&mut p.wifi_ssid, value, "WIFI_SSID"),
        "WIFI_PASS" => put(&mut p.wifi_pass, value, "WIFI_PASS"),
        "BEACON_TARGET" => put(&mut p.beacon_target, value, "BEACON_TARGET"),
        "KISS_TCP" => put(&mut p.kiss_tcp_target, value, "KISS_TCP"),
        "AXUDP_PORT" => match value.parse::<u16>() {
            Ok(v) => {
                p.axudp_port = Some(v);
                String::from("AXUDP_PORT staged. SAVE to persist.")
            }
            Err(_) => String::from("not a port number"),
        },
        "TELNET_PORT" => match value.parse::<u16>() {
            Ok(v) => {
                p.telnet_port = Some(v);
                String::from("TELNET_PORT staged. SAVE to persist.")
            }
            Err(_) => String::from("not a port number"),
        },
        "NODES_INTERVAL" => match value.parse::<u32>() {
            Ok(v) if v >= 10 => {
                p.nodes_interval_secs = Some(v);
                String::from("NODES_INTERVAL staged. SAVE to persist.")
            }
            _ => String::from("not a sane interval (>=10s)"),
        },
        "MQTT_HOST" => put(&mut p.mqtt_host, value, "MQTT_HOST"),
        "ORIGINATE" => match value {
            "true" | "TRUE" | "1" | "on" | "ON" => {
                p.originate = Some(true);
                String::from("ORIGINATE staged. SAVE to persist.")
            }
            "false" | "FALSE" | "0" | "off" | "OFF" => {
                p.originate = Some(false);
                String::from("ORIGINATE staged. SAVE to persist.")
            }
            _ => String::from("expected true/false"),
        },
        other => format!("unknown key {other} (SHOW lists keys)"),
    }
}
