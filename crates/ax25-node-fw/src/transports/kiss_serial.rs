#![allow(dead_code)] // spawned now; some core modem setters (ackmode) stay unused until a session supervisor drives outbound

//! Capability 3 — KISS-over-serial to a NinoTNC.
//!
//! Ports `Packet.Kiss.Serial.KissSerialModem` + the NinoTNC overlay onto an
//! `embassy_rp` UART. The KISS framing/codec, the [`SerialKissModem`] seam, the
//! ACKMODE/SETHW/parameter helpers, and the NinoTNC mode/TX-Test extensions are all
//! in [`ax25_node_core::kiss`] (host-tested) — this task only supplies the *byte
//! source*: a [`ByteStream`] over the UART, which [`SerialKissModem`] drives exactly
//! as the C# modem drives its `SerialPort`.
//!
//! ## Hardware note (research §5.3 / capability 3 caveat)
//!
//! The RP2040 cannot be a USB *host* and a USB-serial device simultaneously, so we
//! do NOT talk to the NinoTNC's USB chip. Instead we wire the Pico's UART directly
//! to the NinoTNC's UART pins (bypassing its USB-serial bridge) — TX→RX, RX→TX, GND,
//! at the NinoTNC's KISS baud ([`ax25_node_core::kiss::ninotnc::DEFAULT_BAUD`] =
//! 57 600 8N1). **UART1 on GP20 (TX) / GP21 (RX)** — the NinoTNC link pins on the
//! NinoBLE Rev5 carrier board (docs/HARDWARE-NINOBLE.md), our reference hardware.
//!
//! ## The setup surface
//!
//! This task is the only thing that talks to the TNC. At start it sends the
//! KISS parameters and operating mode saved on the node (the `/tnc` web page or
//! the console keys `TNC_MODE` / `TXDELAY` / `PERSIST` / `SLOTTIME` / `TXTAIL` /
//! `DUPLEX`), verifying the mode by GETALL readback. After that it takes
//! commands from the web page through [`crate::tnc::CMD`], and logs every frame
//! and TNC report to the monitor ([`crate::tnc::MONITOR`]).
//!
//! There is no periodic beacon on this port: a real radio is attached, and the
//! page's test-frame button covers the bring-up need the old 10 s beacon served.

use ax25_node_core::ax25::{Callsign, PID_NO_LAYER3};
use ax25_node_core::kiss::ninotnc::mode_set::{write_dip, write_mode, Step};
use ax25_node_core::kiss::ninotnc::{
    self, commands, ModeSetter, ModeVerifyPolicy, NinoTncInboundEvent, NinoTncStatusFrame,
};
use ax25_node_core::kiss::serial::ByteStream;
use ax25_node_core::kiss::{classify::InboundEvent, Command, SerialKissModem};
use ax25_node_core::monitor::{write_frame, Direction};
use ax25_node_core::netrom::wire::Alias;
use ax25_node_core::netrom::{
    NetRomOriginator, NetRomOriginatorOptions, ObserveOutcome, PortId,
};

use embassy_futures::select::{select4, Either4};
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{PIN_20, PIN_21, UART1};
use embassy_rp::uart::{
    BufferedInterruptHandler, BufferedUart, Config as UartConfig, Error as UartError,
};
use embassy_rp::Peri;
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_io_async::{Read, Write};
use static_cell::StaticCell;

use crate::config::{KissSerialConfig, NetRomConfig};
use crate::config_store::TncSettings;
use crate::session;
use crate::tnc::{self, TncCommand};
use crate::transports::{call_str, ui_frame};

/// Seconds between housekeeping ticks (NODES origination and the obsolescence
/// sweep each check their own interval on every tick).
const HOUSEKEEPING_TICK_SECS: u64 = 10;

bind_interrupts!(struct Irqs {
    UART1_IRQ => BufferedInterruptHandler<UART1>;
});

/// A [`ByteStream`] over an `embassy_rp` buffered UART — the embedded byte source the
/// portable [`SerialKissModem`] runs on. `read`/`write` are the only hardware seam;
/// everything above (framing, escaping, the modem, the NinoTNC extensions) is the
/// host-tested portable core.
pub struct UartByteStream {
    uart: BufferedUart,
}

impl UartByteStream {
    /// Wrap a configured buffered UART.
    pub fn new(uart: BufferedUart) -> Self {
        Self { uart }
    }
}

impl ByteStream for UartByteStream {
    type Error = UartError;

    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        // `embedded_io_async::Read::read` awaits at least one byte (0 only on EOF),
        // which is exactly the contract `SerialKissModem::read_frame` expects.
        Read::read(&mut self.uart, buf).await
    }

    async fn write(&mut self, bytes: &[u8]) -> Result<(), Self::Error> {
        Write::write_all(&mut self.uart, bytes).await
    }
}

type Modem = SerialKissModem<UartByteStream>;

#[embassy_executor::task]
pub async fn task(
    uart: Peri<'static, UART1>,
    tx_pin: Peri<'static, PIN_20>,
    rx_pin: Peri<'static, PIN_21>,
    cfg: KissSerialConfig,
    netrom_cfg: NetRomConfig,
    my_call: Callsign,
    node_alias: &'static str,
) {
    defmt::info!(
        "kiss-serial: UART1 GP20/21 @ {} baud (NinoTNC direct UART)",
        cfg.baud
    );

    let uart = configure_uart(uart, tx_pin, rx_pin, cfg.baud);
    let mut modem = SerialKissModem::new(UartByteStream::new(uart));
    tnc::with_state(|s| s.running = true);
    let baud = cfg.baud;
    tnc::log_with(Direction::Info, |w| {
        write!(w, "Serial link to the TNC started, {baud} baud")
    });

    // Bring the TNC to the settings saved on the node: KISS parameters first,
    // then the mode (verified by readback). With no saved mode, still ask for a
    // GETALL so the page can show what the TNC is running. The build-env
    // NINOTNC_MODE is the fallback when no mode is saved.
    let mut settings = cfg.tnc;
    if settings.mode.is_none() {
        settings.mode = cfg.startup_mode.filter(|m| *m <= 15);
    }
    send_params(&mut modem, &settings).await;
    let mut mode_job: Option<ModeSetter> = None;
    match settings.mode {
        Some(mode) => start_mode(&mut modem, &mut mode_job, mode, false).await,
        None => send_get_all(&mut modem).await,
    }

    // Read-only NET/ROM tap + NODES origination + obsolescence sweep — the same
    // wiring the KISS-TCP transport uses, now over real RF (Gap A + Gap B). Each
    // transport owns its own routing table (the single-transport-ownership model;
    // the shared session/routing supervisor seam is deferred).
    let mut netrom = session::new_netrom();
    let port_id = PortId::from_str_lossy("kiss-serial");
    let originator = NetRomOriginator::new(NetRomOriginatorOptions {
        enabled: netrom_cfg.originate,
        alias: Some(Alias::from_str_lossy(node_alias)),
        node_call: Some(my_call),
        obsolete_minimum: None,
    });
    let nodes_interval = Duration::from_secs(netrom_cfg.nodes_interval_secs as u64);
    let mut next_nodes_at = Instant::now(); // announce on the first tick
    let mut next_sweep_at = Instant::now() + nodes_interval;
    if netrom_cfg.originate {
        defmt::info!(
            "kiss-serial: NODES origination on, every {=u32}s",
            netrom_cfg.nodes_interval_secs
        );
    }

    let mut ticker = Ticker::every(Duration::from_secs(HOUSEKEEPING_TICK_SECS));

    // The pump: wake on an inbound KISS frame, the housekeeping tick, a command
    // from the web page, or the mode change's next deadline. `read_frame` is
    // cancel-safe (its only await is the UART read; the decode state lives in
    // the modem), so losing the race drops no bytes.
    loop {
        let deadline = mode_job.as_ref().and_then(|j| j.deadline_ms());
        let job_timer = async move {
            match deadline {
                Some(d) => Timer::at(Instant::from_millis(d)).await,
                None => core::future::pending::<()>().await,
            }
        };
        match select4(
            modem.read_frame(),
            ticker.next(),
            tnc::CMD.receive(),
            job_timer,
        )
        .await
        {
            Either4::First(read) => match read {
                Ok(Some(frame)) => {
                    let now = tnc::now_ms();
                    tnc::with_state(|s| s.last_heard_ms = Some(now));
                    let heard = handle_inbound(&frame, &mut netrom, my_call, port_id);
                    if let (Some(status), Some(job)) = (heard, mode_job.as_mut()) {
                        let step = job.on_status(&status, now);
                        advance_mode(&mut modem, &mut mode_job, step, &settings).await;
                    }
                }
                // EOF / link-down: a buffered UART doesn't really "close", but on a
                // read error or zero-read we yield and retry rather than spin.
                Ok(None) => Timer::after_millis(10).await,
                Err(e) => {
                    defmt::warn!("kiss-serial read error: {}", defmt::Debug2Format(&e));
                    Timer::after_millis(100).await;
                }
            },
            Either4::Second(()) => {
                // Obsolescence sweep — age/purge once per NODES interval, before
                // origination (the C# `NetRomService.OnInterval` order).
                if Instant::now() >= next_sweep_at {
                    next_sweep_at = Instant::now() + nodes_interval;
                    let purged = netrom.sweep();
                    if purged > 0 {
                        defmt::info!(
                            "kiss-serial: obsolescence sweep purged {=usize} stale route(s)",
                            purged
                        );
                    }
                }

                // NODES origination — build our broadcasts from the live table, wrap
                // each as a UI frame (dest NODES, PID 0xCF), and send it to the UART.
                if netrom_cfg.originate && Instant::now() >= next_nodes_at {
                    next_nodes_at = Instant::now() + nodes_interval;
                    let payloads = originator.broadcast_nodes(netrom.table());
                    let dest = NetRomOriginator::nodes_destination();
                    let mut sent = 0usize;
                    for payload in &payloads {
                        let frame = ui_frame(my_call, dest, NetRomOriginator::PID, payload);
                        if send_ax25(&mut modem, &frame).await {
                            sent += 1;
                        }
                    }
                    defmt::info!(
                        "kiss-serial: NODES broadcast sent ({=usize} frame(s))",
                        sent
                    );
                }
            }
            Either4::Third(cmd) => match cmd {
                TncCommand::SetMode {
                    mode,
                    persist_to_flash,
                } => {
                    settings.mode = Some(mode);
                    start_mode(&mut modem, &mut mode_job, mode, persist_to_flash).await;
                }
                TncCommand::SetParams(p) => {
                    settings = TncSettings {
                        mode: settings.mode,
                        ..p
                    };
                    send_params(&mut modem, &settings).await;
                }
                TncCommand::SendTest { dest, text } => {
                    let frame = ui_frame(my_call, dest, PID_NO_LAYER3, text.as_bytes());
                    send_ax25(&mut modem, &frame).await;
                }
                TncCommand::Refresh => send_get_all(&mut modem).await,
            },
            Either4::Fourth(()) => {
                if let Some(job) = mode_job.as_mut() {
                    let step = job.on_time(tnc::now_ms());
                    advance_mode(&mut modem, &mut mode_job, step, &settings).await;
                }
            }
        }
    }
}

/// Handle one inbound KISS frame: the NET/ROM tap, the monitor, and the shared
/// TNC state. Returns the TNC's status when the frame was a report (a GETALL
/// reply, the periodic beacon, or the TX-test diagnostic), for the mode-change
/// readback.
fn handle_inbound(
    frame: &ax25_node_core::kiss::Frame,
    netrom: &mut session::NetRom,
    my_call: Callsign,
    port_id: PortId,
) -> Option<NinoTncStatusFrame> {
    match ninotnc::classify(frame) {
        NinoTncInboundEvent::Generic(InboundEvent::Ax25 { ax25, .. }) => {
            // READ-ONLY NET/ROM TAP: every frame, BEFORE any address filter, so
            // NODES broadcasts (dest "NODES", not us) are heard. The same
            // FrameTraced-equivalent point as axudp / kiss_tcp.
            let outcome = session::observe_inbound(netrom, &ax25, my_call, port_id);
            if let ObserveOutcome::Ingested { .. } = outcome {
                defmt::info!(
                    "kiss-serial: NODES broadcast ingested ({=u32} destinations known)",
                    netrom.destination_count() as u32
                );
            }
            tnc::with_state(|s| s.rx_frames = s.rx_frames.wrapping_add(1));
            tnc::log_with(Direction::Rx, |w| write_frame(&ax25, w));
            // Address-filtered connected-mode session routing is the
            // session-supervisor seam (the same deferred point kiss_tcp leaves).
            None
        }
        NinoTncInboundEvent::TxTestDiagnostic { raw, diagnostic } => {
            let status = NinoTncStatusFrame::from_diagnostic(&diagnostic);
            // A GETALL reply arrives on the 0xE0 reply byte (port 14); the same
            // text on port 0 means someone pressed the TNC's TX-test button.
            let what = if raw.port == 14 {
                "TNC report"
            } else {
                "TNC test button pressed (it transmits its test signal)"
            };
            record_status(what, &status);
            Some(status)
        }
        NinoTncInboundEvent::StatusReport { status, .. } => {
            record_status("TNC status beacon", &status);
            Some(status)
        }
        NinoTncInboundEvent::AirTest { raw, air_test } => {
            // Another NinoTNC's TX-test transmission, heard over the air.
            tnc::with_state(|s| s.rx_frames = s.rx_frames.wrapping_add(1));
            let seq = air_test.sequence_counter;
            match ax25_node_core::ax25::Frame::decode(&raw.payload) {
                Ok(f) => tnc::log_with(Direction::Rx, |w| {
                    let mut buf = [0u8; 16];
                    let from = call_str(&f.source.callsign, &mut buf);
                    write!(w, "NinoTNC test transmission #{seq} from {from}")
                }),
                Err(_) => tnc::log(Direction::Rx, "NinoTNC test transmission"),
            }
            None
        }
        NinoTncInboundEvent::RssiReading { rssi, .. } => {
            let db = rssi.whole_db();
            tnc::log_with(Direction::Info, |w| write!(w, "TNC RX audio level {db} dB"));
            None
        }
        NinoTncInboundEvent::Generic(InboundEvent::AckModeData { .. }) => None,
        NinoTncInboundEvent::Generic(InboundEvent::Unknown { raw, .. }) => {
            let (cmd, len) = (raw.command_byte(), raw.payload.len());
            tnc::log_with(Direction::Info, |w| {
                write!(
                    w,
                    "Unrecognised KISS frame from the TNC: command 0x{cmd:02X}, {len} bytes"
                )
            });
            None
        }
    }
}

/// Store a TNC report for the page and log a one-line summary of it.
fn record_status(what: &str, status: &NinoTncStatusFrame) {
    tnc::with_state(|s| s.status = Some(*status));
    tnc::log_with(Direction::Info, |w| {
        write!(w, "{what}: firmware {}", status.firmware_version_raw.as_str())?;
        if let Some(d) = status.dip_switches {
            w.write_str(", MODE DIPs ")?;
            write_dip(d, w)?;
        }
        match (status.running_mode, status.firmware_mode_byte) {
            (Some(m), _) => {
                w.write_str(", running mode ")?;
                write_mode(m.mode, w)
            }
            (None, Some(b)) => write!(w, ", running unknown mode byte 0x{b:02X}"),
            (None, None) => Ok(()),
        }
    });
}

/// Send an AX.25 frame and log it. Returns whether it reached the UART.
async fn send_ax25(modem: &mut Modem, frame: &ax25_node_core::ax25::Frame) -> bool {
    match modem.send_frame(&frame.encode()).await {
        Ok(()) => {
            tnc::with_state(|s| s.tx_frames = s.tx_frames.wrapping_add(1));
            tnc::log_with(Direction::Tx, |w| write_frame(frame, w));
            true
        }
        Err(e) => {
            defmt::warn!("kiss-serial: send failed: {}", defmt::Debug2Format(&e));
            tnc::log(Direction::Info, "Send to the TNC failed (serial write error)");
            false
        }
    }
}

/// Send a KISS control command and log what it was for.
async fn send_control(
    modem: &mut Modem,
    command: Command,
    payload: &[u8],
    describe: impl FnOnce(&mut dyn core::fmt::Write) -> core::fmt::Result,
) {
    match modem.send_kiss(command, payload).await {
        Ok(()) => tnc::log_with(Direction::Info, describe),
        Err(e) => {
            defmt::warn!("kiss-serial: command failed: {}", defmt::Debug2Format(&e));
            tnc::log(Direction::Info, "Send to the TNC failed (serial write error)");
        }
    }
}

async fn send_get_all(modem: &mut Modem) {
    send_control(
        modem,
        Command::Other(commands::GET_ALL_COMMAND),
        &commands::QUERY_PAYLOAD,
        |w| w.write_str("Asked the TNC for its report (GETALL)"),
    )
    .await;
}

/// Send each KISS parameter that is set.
async fn send_params(modem: &mut Modem, t: &TncSettings) {
    for (command, value, name, is_ms) in [
        (Command::TxDelay, t.tx_delay, "TXDELAY", true),
        (Command::Persistence, t.persist, "PERSIST", false),
        (Command::SlotTime, t.slot_time, "SLOTTIME", true),
        (Command::TxTail, t.tx_tail, "TXTAIL", true),
        (
            Command::FullDuplex,
            t.full_duplex.map(u8::from),
            "FULLDUPLEX",
            false,
        ),
    ] {
        let Some(v) = value else { continue };
        send_control(modem, command, &[v], |w| {
            if is_ms {
                write!(w, "Set {name} {} ms", v as u32 * 10)
            } else {
                write!(w, "Set {name} {v}")
            }
        })
        .await;
    }
}

/// Start a mode change (replacing any in flight) and send its first SETHW.
async fn start_mode(modem: &mut Modem, job: &mut Option<ModeSetter>, mode: u8, persist: bool) {
    let Some((setter, step)) =
        ModeSetter::start(mode, persist, tnc::now_ms(), ModeVerifyPolicy::default())
    else {
        tnc::log(Direction::Info, "Mode must be 0-15");
        return;
    };
    tnc::with_state(|s| s.mode_job = Some(setter));
    do_step(modem, &setter, step).await;
    if setter.outcome().is_some() {
        // Mode 15 has no readback, so it is finished as soon as it is sent.
        report_outcome(&setter);
        *job = None;
    } else {
        *job = Some(setter);
    }
}

/// Carry out the step a mode change asked for and publish its progress. When it
/// finishes, log the outcome and re-send the KISS parameters, so nothing the TNC
/// might reset on a mode change is left behind.
async fn advance_mode(
    modem: &mut Modem,
    job: &mut Option<ModeSetter>,
    step: Option<Step>,
    settings: &TncSettings,
) {
    let Some(setter) = *job else { return };
    if let Some(step) = step {
        do_step(modem, &setter, step).await;
    }
    tnc::with_state(|s| s.mode_job = Some(setter));
    if setter.outcome().is_some() {
        report_outcome(&setter);
        *job = None;
        send_params(
            modem,
            &TncSettings {
                mode: None,
                ..*settings
            },
        )
        .await;
    }
}

async fn do_step(modem: &mut Modem, setter: &ModeSetter, step: Step) {
    match step {
        Step::SendSetHardware { payload } => {
            let (mode, persist, attempt) =
                (setter.mode(), setter.persist_to_flash(), setter.attempt());
            send_control(modem, Command::SetHardware, &[payload], |w| {
                write!(w, "SETHW {payload}: mode ")?;
                write_mode(mode, w)?;
                w.write_str(if persist {
                    ", saved in the TNC"
                } else {
                    ", not saved in the TNC"
                })?;
                if attempt > 1 {
                    write!(w, " (attempt {attempt})")?;
                }
                Ok(())
            })
            .await;
        }
        Step::SendGetAll => send_get_all(modem).await,
    }
}

fn report_outcome(setter: &ModeSetter) {
    tnc::log_with(Direction::Info, |w| setter.write_summary(w));
}

/// Configure UART1 as a buffered 8N1 UART at `baud` on GP20 (TX) / GP21 (RX) —
/// the NinoBLE Rev5 NinoTNC link. Static TX/RX ring buffers sized for a couple
/// of KISS frames.
fn configure_uart(
    uart: Peri<'static, UART1>,
    tx_pin: Peri<'static, PIN_20>,
    rx_pin: Peri<'static, PIN_21>,
    baud: u32,
) -> BufferedUart {
    let mut config = UartConfig::default();
    config.baudrate = baud;
    static TX_BUF: StaticCell<[u8; 256]> = StaticCell::new();
    static RX_BUF: StaticCell<[u8; 256]> = StaticCell::new();
    BufferedUart::new(
        uart,
        tx_pin,
        rx_pin,
        Irqs,
        TX_BUF.init([0; 256]),
        RX_BUF.init([0; 256]),
        config,
    )
}
