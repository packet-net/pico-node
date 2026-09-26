//! Station power monitoring: an INA226 current/voltage monitor on the I2C bus
//! (GP4/GP5, shared with the OLED), reported on air as APRS telemetry.
//!
//! "Just works": the bus task ([`crate::i2c_bus`]) probes for an INA226 at
//! boot and again every minute while none answers, so a board plugged in later
//! is picked up. With one present the node sends, every `TELEM_INTERVAL`
//! minutes (default 10, 0 = off), a telemetry report of battery voltage and
//! load current; once an hour (and with the first report) it also sends the
//! label messages (PARM / UNIT / EQNS / BITS) and, when a grid locator is set, a
//! position report so the station shows on the map. Frames go to `APZ001` with
//! no digipeater path, on the first usable radio port.
//!
//! Channel scaling: voltage in 0.06 V steps (0 - 15.3 V, enough for a 4S
//! LiFePO4 pack at 14.6 V); current in the smallest of 0.01 / 0.02 / 0.05 /
//! 0.1 A steps that covers the shunt's full scale (81.92 mV across it), capped
//! at 0.1 A steps (25.5 A) so a big external shunt still shows an idling
//! station. Charging current (negative) reads as 0.

use core::cell::{Cell, RefCell};

use alloc::format;
use alloc::string::String;
use alloc::vec::Vec;

use ax25_node_core::aprs;
use ax25_node_core::ax25::{Callsign, PID_NO_LAYER3};
use embassy_rp::i2c::{Async, I2c};
use embassy_rp::peripherals::I2C0;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::blocking_mutex::Mutex;
use embassy_time::{Duration, Instant};

use crate::config::PowerConfig;
use crate::ports::{self, ui_frame};

/// The I2C0 bus, shared inside the bus task between the OLED driver and this
/// module. `NoopRawMutex`: only that one task touches it.
pub type Bus = Mutex<NoopRawMutex, RefCell<I2c<'static, I2C0, Async>>>;

/// One measurement.
#[derive(Clone, Copy)]
pub struct Reading {
    pub millivolts: u32,
    /// Positive = drawn by the load.
    pub milliamps: i32,
}

/// The latest reading, for the web panel. `None` without an INA226.
static LATEST: Mutex<CriticalSectionRawMutex, Cell<Option<Reading>>> =
    Mutex::new(Cell::new(None));

/// The latest reading, if an INA226 is fitted and answering.
pub fn latest() -> Option<Reading> {
    LATEST.lock(|c| c.get())
}

/// INA226 registers and identity (TI SBOS547).
const REG_CONFIG: u8 = 0x00;
const REG_SHUNT: u8 = 0x01;
const REG_BUS: u8 = 0x02;
const REG_MANUFACTURER: u8 = 0xFE;
const REG_DIE: u8 = 0xFF;
const MANUFACTURER_TI: u16 = 0x5449;
const DIE_INA226: u16 = 0x2260;
/// Average 16 samples, 1.1 ms bus and shunt conversions, continuous: a fresh,
/// settled value every ~35 ms.
const CONFIG_AVG16: u16 = 0x4527;

/// The voltage channel's step, millivolts.
const VOLT_STEP_MV: i32 = 60;
/// Candidate current steps, milliamps (smallest that covers full scale wins).
const AMP_STEPS_MA: [i32; 4] = [10, 20, 50, 100];
/// Wait after boot before the first report, so the radio port is up.
const FIRST_REPORT_DELAY: Duration = Duration::from_secs(60);
/// How often to look for an INA226 while none answers.
const PROBE_EVERY: Duration = Duration::from_secs(60);

/// The power monitor and telemetry schedule, driven by the bus task.
pub struct Monitor {
    cfg: PowerConfig,
    /// Our callsign when one is configured (never transmit without one).
    call: Option<Callsign>,
    /// Where the station is, in hundredths of a minute, from the grid locator.
    position: Option<(i32, i32)>,
    alias: &'static str,
    /// The INA226's address once found.
    addr: Option<u8>,
    next_probe: Instant,
    next_report: Instant,
    seq: u16,
    reports: u32,
}

impl Monitor {
    pub fn new(cfg: PowerConfig, call: Option<Callsign>, grid: &str, alias: &'static str) -> Self {
        Self {
            cfg,
            call,
            position: aprs::grid_centre(grid),
            alias,
            addr: None,
            next_probe: Instant::now(),
            next_report: Instant::now() + FIRST_REPORT_DELAY,
            seq: 0,
            reports: 0,
        }
    }

    /// Read the INA226 (finding it first if need be), publish the reading, and
    /// send telemetry when due. Called every few seconds by the bus task.
    pub async fn poll(&mut self, bus: &Bus) {
        let now = Instant::now();
        if self.addr.is_none() && now >= self.next_probe {
            self.next_probe = now + PROBE_EVERY;
            self.addr = probe(bus);
            if let Some(a) = self.addr {
                defmt::info!("power: INA226 at {=u8:#04x}", a);
                crate::nlog!("power: INA226 found at 0x{:02x}", a);
            }
        }
        let Some(addr) = self.addr else {
            return;
        };
        let Some(reading) = read(bus, addr, self.cfg.shunt_micro_ohm) else {
            defmt::warn!("power: INA226 stopped answering");
            crate::nlog!("power: INA226 stopped answering");
            self.addr = None;
            LATEST.lock(|c| c.set(None));
            return;
        };
        LATEST.lock(|c| c.set(Some(reading)));

        let interval = self.cfg.telemetry_interval_min;
        if interval == 0 || now < self.next_report {
            return;
        }
        self.next_report = now + Duration::from_secs(interval as u64 * 60);
        let Some(call) = self.call else {
            return;
        };
        let Some(port) = ports::first_usable() else {
            return; // no radio up: skip this one
        };
        let Some(dest) = Callsign::parse(aprs::TOCALL) else {
            return;
        };
        // Labels and position with the first report, then hourly.
        let labels_every = (60 / interval as u32).max(1);
        let with_labels = self.reports.is_multiple_of(labels_every);
        for info in self.frames(call, reading, with_labels) {
            ports::send(port, ui_frame(call, dest, PID_NO_LAYER3, info.as_bytes()).encode()).await;
        }
        self.seq = (self.seq + 1) % 1000;
        self.reports = self.reports.wrapping_add(1);
    }

    /// The information fields for one round: optionally the position and
    /// label messages, then the telemetry report.
    fn frames(&self, call: Callsign, r: Reading, with_labels: bool) -> Vec<String> {
        let amp_step = amp_step_ma(self.cfg.shunt_micro_ohm);
        let mut out = Vec::new();
        if with_labels {
            let mut buf = [0u8; 16];
            let me = ports::call_str(&call, &mut buf);
            if let Some((lat, lon)) = self.position {
                let comment = format!("{} pico-node", self.alias);
                out.push(aprs::position_report(lat, lon, '/', 'n', comment.trim()));
            }
            out.push(aprs::message(me, "PARM.Battery,Current"));
            out.push(aprs::message(me, "UNIT.V,A"));
            out.push(aprs::message(
                me,
                &format!(
                    "EQNS.0,{},0,0,{},0",
                    decimal(VOLT_STEP_MV),
                    decimal(amp_step)
                ),
            ));
            out.push(aprs::message(me, "BITS.11111111,Station power"));
        }
        let analog = [
            aprs::quantise(r.millivolts.min(i32::MAX as u32) as i32, VOLT_STEP_MV),
            aprs::quantise(r.milliamps, amp_step),
            0,
            0,
            0,
        ];
        out.push(aprs::telemetry_report(self.seq, &analog, 0));
        out
    }
}

/// Thousandths as a plain decimal: 60 -> "0.06", 100 -> "0.1", 1000 -> "1".
fn decimal(milli: i32) -> String {
    let mut s = format!("{}.{:03}", milli / 1000, milli % 1000);
    while s.ends_with('0') {
        s.pop();
    }
    if s.ends_with('.') {
        s.pop();
    }
    s
}

/// The current channel's step for this shunt: the smallest candidate whose
/// 255 steps cover the INA226's full scale (81.92 mV across the shunt), at
/// most 0.1 A (readings above 25.5 A then send as 255).
fn amp_step_ma(shunt_micro_ohm: u32) -> i32 {
    let full_scale_ma = 81_920_000u64 / shunt_micro_ohm.max(1) as u64;
    AMP_STEPS_MA
        .into_iter()
        .find(|&s| s as u64 * 255 >= full_scale_ma)
        .unwrap_or(100)
}

/// Look for an INA226 at 0x40-0x4F (its address straps), check its identity
/// registers, and set it averaging.
fn probe(bus: &Bus) -> Option<u8> {
    let addr = (0x40..=0x4F).find(|&a| {
        read_reg(bus, a, REG_MANUFACTURER) == Some(MANUFACTURER_TI)
            && read_reg(bus, a, REG_DIE) == Some(DIE_INA226)
    })?;
    let [hi, lo] = CONFIG_AVG16.to_be_bytes();
    bus.lock(|b| b.borrow_mut().blocking_write(addr, &[REG_CONFIG, hi, lo]))
        .ok()?;
    Some(addr)
}

/// Bus voltage (1.25 mV per bit) and current from the shunt voltage (2.5 uV
/// per bit, signed) across `shunt_micro_ohm`.
fn read(bus: &Bus, addr: u8, shunt_micro_ohm: u32) -> Option<Reading> {
    let bus_raw = read_reg(bus, addr, REG_BUS)?;
    let shunt_raw = read_reg(bus, addr, REG_SHUNT)? as i16;
    let millivolts = bus_raw as u32 * 5 / 4;
    // I = V / R: (raw * 2.5 uV) / (R uOhm) amps = raw * 2500 / R milliamps.
    let milliamps = (shunt_raw as i64 * 2500 / shunt_micro_ohm.max(1) as i64) as i32;
    Some(Reading {
        millivolts,
        milliamps,
    })
}

fn read_reg(bus: &Bus, addr: u8, reg: u8) -> Option<u16> {
    let mut buf = [0u8; 2];
    bus.lock(|b| b.borrow_mut().blocking_write_read(addr, &[reg], &mut buf))
        .ok()?;
    Some(u16::from_be_bytes(buf))
}
