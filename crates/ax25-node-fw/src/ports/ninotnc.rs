#![allow(dead_code)] // some core modem setters (ackmode) are not used by this driver

//! Port 0: the NinoTNC on the serial link, the node's radio.
//!
//! Ports `Packet.Kiss.Serial.KissSerialModem` + the NinoTNC overlay onto an
//! `embassy_rp` UART. The KISS framing/codec, the [`SerialKissModem`] seam, the
//! ACKMODE/SETHW/parameter helpers, and the NinoTNC mode/TX-Test extensions are all
//! in [`ax25_node_core::kiss`] (host-tested); this task supplies the *byte
//! source*: a [`ByteStream`] over the UART, which [`SerialKissModem`] drives exactly
//! as the C# modem drives its `SerialPort`.
//!
//! ## Hardware
//!
//! The RP2040 cannot be a USB *host* and a USB-serial device simultaneously, so we
//! do NOT talk to the NinoTNC's USB chip. Instead the Pico's UART is wired directly
//! to the NinoTNC's UART pins (J5, bypassing its USB-serial bridge): TX to RX, RX
//! to TX, GND, at the NinoTNC's KISS baud
//! ([`ax25_node_core::kiss::ninotnc::DEFAULT_BAUD`] = 57 600 8N1). **UART1 on GP20
//! (TX) / GP21 (RX)**, the NinoTNC link pins on the NinoBLE Rev5 carrier board
//! (docs/HARDWARE-NINOBLE.md), our reference hardware.
//!
//! ## What this driver does
//!
//! It is the only thing that talks to the TNC. At start it asks for the TNC's
//! report (GETALL) and uses the TNC only once it reports supported firmware
//! (3.44 / 4.44 or later): then it sends the KISS parameters and operating mode
//! saved on the node (the `/tnc` web page or the console keys `TNC_MODE` /
//! `TXDELAY` / `PERSIST` / `SLOTTIME` / `TXTAIL` / `DUPLEX`; the mode verified by
//! GETALL readback) and opens the port to traffic ([`crate::ports`]): heard
//! AX.25 frames go to the node task, and the node task's frames go on the air.
//! It takes setup commands from the web page ([`crate::tnc::CMD`]), updates the
//! TNC's firmware through its bootloader, and logs every frame and TNC report to
//! the monitor ([`crate::tnc::MONITOR`]).

use ax25_node_core::ax25::{Callsign, PID_NO_LAYER3};
use ax25_node_core::kiss::ninotnc::flash::{
    write_flash_outcome, BootloaderFlasher, FlashAction, FlashOutcome, FlashTimings, MAX_LINE,
};
use ax25_node_core::kiss::ninotnc::mode_set::{refuse_before_sending, write_dip, write_mode, Step};
use ax25_node_core::kiss::ninotnc::{
    self, commands, ChipVariant, ModeSetter, ModeVerifyPolicy, NinoTncInboundEvent,
    NinoTncStatusFrame,
};
use ax25_node_core::kiss::serial::ByteStream;
use ax25_node_core::kiss::{classify::InboundEvent, Command, SerialKissModem};
use ax25_node_core::monitor::{write_frame, Direction};

use embassy_futures::select::{select, select4, Either, Either4};
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{PIN_20, PIN_21, UART1};
use embassy_rp::uart::{
    BufferedInterruptHandler, BufferedUart, Config as UartConfig, Error as UartError,
};
use embassy_rp::Peri;
use embassy_time::{Duration, Instant, Timer};
use embedded_io_async::{Read, Write};
use static_cell::StaticCell;

use crate::config::NinoTncConfig;
use crate::config_store::TncSettings;
use crate::tnc::{self, FlashStatus, TncCommand};
use crate::tnc_image::{self, LineReader};
use crate::ports::{self, call_str, ui_frame, Port};

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
    cfg: NinoTncConfig,
    my_call: Callsign,
) {
    defmt::info!(
        "ninotnc: UART1 GP20/21 @ {} baud (NinoTNC direct UART)",
        cfg.baud
    );

    let uart = configure_uart(uart, tx_pin, rx_pin, cfg.baud);
    let mut modem = SerialKissModem::new(UartByteStream::new(uart));
    tnc::with_state(|s| s.running = true);
    let baud = cfg.baud;
    tnc::log_with(Direction::Info, |w| {
        write!(w, "Serial link to the TNC started, {baud} baud")
    });

    // The node's saved settings (the /tnc page / console keys). The build-env
    // NINOTNC_MODE is the fallback when no mode is saved.
    let mut settings = cfg.tnc;
    if settings.mode.is_none() {
        settings.mode = cfg.startup_mode.filter(|m| *m <= 15);
    }
    let mut link = LinkState::default();
    let mut mode_job: Option<ModeSetter> = None;
    // When the next frame may be handed to the TNC (see `FRAME_GAP_MS`).
    let mut next_tx_at = Instant::now();

    // Nothing is sent to the TNC, and the radio port stays closed, until it has
    // said which firmware it runs: see `LinkState::on_report`.
    send_get_all(&mut modem).await;

    // The pump: wake on an inbound KISS frame, a command from the web page, the
    // mode change's next deadline, a frame from the node task to transmit, or
    // the retry timer for an unanswered GETALL. `read_frame` is cancel-safe (its
    // only await is the UART read; the decode state lives in the modem), so
    // losing the race drops no bytes.
    loop {
        let deadline = mode_job
            .as_ref()
            .and_then(|j| j.deadline_ms())
            .or(link.retry_at_ms);
        let timer = async move {
            match deadline {
                Some(d) => Timer::at(Instant::from_millis(d)).await,
                None => core::future::pending::<()>().await,
            }
        };
        // The next frame waits for the previous one to clear the wire. The
        // wait holds only this arm: inbound frames are still read meanwhile.
        let gap = next_tx_at;
        let next_frame = async move {
            Timer::at(gap).await;
            ports::take_tx(Port::NINOTNC).await
        };
        match select4(modem.read_frame(), tnc::CMD.receive(), timer, next_frame).await
        {
            Either4::First(read) => match read {
                Ok(Some(frame)) => {
                    let now = tnc::now_ms();
                    tnc::with_state(|s| s.last_heard_ms = Some(now));
                    let Some(status) = handle_inbound(&frame) else {
                        continue;
                    };
                    if let Some(job) = mode_job.as_mut() {
                        let step = job.on_status(&status, now);
                        advance_mode(&mut modem, &mut mode_job, step, &settings).await;
                    }
                    if link.on_report(&status) {
                        apply_saved(&mut modem, &settings, &mut mode_job).await;
                    }
                }
                // EOF / link-down: a buffered UART doesn't really "close", but on a
                // read error or zero-read we yield and retry rather than spin.
                Ok(None) => Timer::after_millis(10).await,
                // A line error (framing, overrun, break, parity). The UART has
                // already dropped the bad byte, but KISS has no checksum: the
                // frame it landed in must not be delivered, so skip to the next
                // frame boundary and let AX.25 retransmit. No pause: the FIFO
                // behind the error is only 32 bytes and would overrun.
                Err(e) => {
                    defmt::warn!("ninotnc: read error: {}", defmt::Debug2Format(&e));
                    modem.resynchronise();
                    tnc::with_state(|s| s.line_errors = s.line_errors.saturating_add(1));
                    tnc::log_with(Direction::Info, |w| {
                        write!(w, "Serial line error ({}); frame dropped", line_error_name(&e))
                    });
                }
            },
            Either4::Second(cmd) => match cmd {
                TncCommand::SetMode {
                    mode,
                    persist_to_flash,
                } => {
                    settings.mode = Some(mode);
                    if link.supported {
                        start_mode(&mut modem, &mut mode_job, mode, persist_to_flash).await;
                    }
                }
                TncCommand::SetParams(p) => {
                    settings = TncSettings {
                        mode: settings.mode,
                        ..p
                    };
                    if link.supported {
                        send_params(&mut modem, &settings).await;
                    }
                }
                TncCommand::SendTest { dest, text } => {
                    if link.supported {
                        let frame = ui_frame(my_call, dest, PID_NO_LAYER3, text.as_bytes());
                        send_wire(&mut modem, &frame.encode()).await;
                    }
                }
                TncCommand::Refresh => send_get_all(&mut modem).await,
                TncCommand::UpdateFirmware => {
                    ports::set_usable(Port::NINOTNC, false);
                    let updated = run_update(&mut modem, &mut mode_job).await;
                    if updated {
                        // Start again from the top: the next report says which
                        // firmware it runs, and the settings follow.
                        link = LinkState::default();
                        send_get_all(&mut modem).await;
                    }
                    ports::set_usable(Port::NINOTNC, link.supported);
                }
            },
            Either4::Third(()) => {
                let now = tnc::now_ms();
                if let Some(job) = mode_job.as_mut() {
                    if job.deadline_ms().is_some_and(|d| now >= d) {
                        let step = job.on_time(now);
                        advance_mode(&mut modem, &mut mode_job, step, &settings).await;
                    }
                }
                if link.retry_at_ms.is_some_and(|d| now >= d) {
                    link.retry_at_ms = None;
                    if !link.heard_report {
                        send_get_all(&mut modem).await;
                        link.retry_at_ms = Some(now + REPORT_RETRY_MS);
                    }
                }
            }
            Either4::Fourth(wire) => {
                // A frame from the node task. `ports::send` already checked the
                // port was usable when it queued; check again in case that
                // changed since.
                if ports::usable(Port::NINOTNC) {
                    send_wire(&mut modem, &wire).await;
                    next_tx_at = Instant::now()
                        + wire_time(wire.len(), baud)
                        + Duration::from_millis(FRAME_GAP_MS);
                }
            }
        }
    }
}

/// Quiet time on the serial link between frames sent to the TNC. Measured on
/// air: frames written back to back over J5 arrive at the NinoTNC damaged (the
/// second of a pair loses bytes and the rest are misread until it re-syncs),
/// seemingly because it is still busy starting the first transmission. The
/// damage passes its radio checksum, so nothing downstream catches it.
const FRAME_GAP_MS: u64 = 1200;

/// How long `len` bytes take on the serial wire (10 bits a byte at the link
/// rate, plus a few for the KISS framing). A write returns once the bytes are
/// buffered, not sent, so the gap is timed from here.
fn wire_time(len: usize, baud: u32) -> Duration {
    let bits = (len as u64 + 4) * 10;
    Duration::from_micros(bits * 1_000_000 / u64::from(baud.max(1)))
}

/// How often to repeat GETALL while the TNC has not reported (e.g. wiring not
/// yet right, or the TNC still booting).
const REPORT_RETRY_MS: u64 = 10_000;

/// Whether the TNC may be used, from its reports.
struct LinkState {
    /// Firmware 3.44 / 4.44 or later reported: settings applied, radio port open.
    supported: bool,
    /// Any report heard since start (or since a firmware update).
    heard_report: bool,
    /// The firmware last warned about, so an unsupported TNC's periodic beacon
    /// does not repeat the warning.
    warned: Option<ninotnc::FirmwareVersion>,
    /// When to ask again if no report has come.
    retry_at_ms: Option<u64>,
}

impl Default for LinkState {
    fn default() -> Self {
        Self {
            supported: false,
            heard_report: false,
            warned: None,
            retry_at_ms: Some(tnc::now_ms() + REPORT_RETRY_MS),
        }
    }
}

impl LinkState {
    /// Take in a report. Returns `true` when the TNC has just become usable, so
    /// the caller applies the saved settings.
    fn on_report(&mut self, status: &NinoTncStatusFrame) -> bool {
        self.heard_report = true;
        let Some(version) = status.firmware_version else {
            return false; // a report without a version changes nothing
        };
        let was = self.supported;
        self.supported = version.is_supported();
        ports::set_usable(Port::NINOTNC, self.supported);
        if !self.supported && self.warned != Some(version) {
            self.warned = Some(version);
            tnc::log_with(Direction::Info, |w| {
                write!(
                    w,
                    "The TNC runs firmware {}.{}; pico-node needs {}.{} or later, so it will not \
use it (no settings, no traffic) until it is updated on this page.",
                    version.major,
                    version.minor,
                    version.major,
                    ninotnc::firmware::MIN_SUPPORTED_MINOR
                )
            });
        }
        self.supported && !was
    }
}

/// Handle one inbound KISS frame: the monitor, the hand-off to the node task,
/// and the shared TNC state. Returns the TNC's status when the frame was a
/// report (a GETALL reply, the periodic beacon, or the TX-test diagnostic).
fn handle_inbound(frame: &ax25_node_core::kiss::Frame) -> Option<NinoTncStatusFrame> {
    match ninotnc::classify(frame) {
        NinoTncInboundEvent::Generic(InboundEvent::Ax25 { ax25, .. }) => {
            tnc::with_state(|s| s.rx_frames = s.rx_frames.wrapping_add(1));
            tnc::log_with(Direction::Rx, |w| write_frame(&ax25, w));
            // To the node task: sessions, the console, NET/ROM (including the
            // read-only NODES tap, which sees every frame before address
            // filtering). Dropped while the TNC is not usable.
            ports::deliver(Port::NINOTNC, ax25);
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
    tnc::with_state(|s| {
        s.status = Some(*status);
        s.status_at_ms = tnc::now_ms();
    });
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

/// Put an encoded AX.25 frame (no FCS) on the air and log it.
async fn send_wire(modem: &mut Modem, wire: &[u8]) {
    match modem.send_frame(wire).await {
        Ok(()) => {
            tnc::with_state(|s| s.tx_frames = s.tx_frames.wrapping_add(1));
            match ax25_node_core::ax25::Frame::decode(wire) {
                Ok(f) => tnc::log_with(Direction::Tx, |w| write_frame(&f, w)),
                Err(_) => tnc::log_with(Direction::Tx, |w| write!(w, "{} bytes", wire.len())),
            }
        }
        Err(e) => {
            defmt::warn!("ninotnc: send failed: {}", defmt::Debug2Format(&e));
            tnc::log(Direction::Info, "Send to the TNC failed (serial write error)");
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
            defmt::warn!("ninotnc: command failed: {}", defmt::Debug2Format(&e));
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

/// Send the node's saved settings to the TNC: KISS parameters, then the mode
/// (verified by readback). Called once the TNC has reported supported
/// firmware: at start, and again after a TNC firmware update.
async fn apply_saved(modem: &mut Modem, settings: &TncSettings, mode_job: &mut Option<ModeSetter>) {
    send_params(modem, settings).await;
    if let Some(mode) = settings.mode {
        start_mode(modem, mode_job, mode, false).await;
    }
}

/// Start a mode change (replacing any in flight) and send its first SETHW.
/// When the TNC has already reported firmware too old for SETHW, nothing is
/// sent and the page says why.
async fn start_mode(modem: &mut Modem, job: &mut Option<ModeSetter>, mode: u8, persist: bool) {
    let known = tnc::with_state(|s| s.status.and_then(|st| st.firmware_version));
    if let Some(outcome) = refuse_before_sending(mode, known) {
        let setter = ModeSetter::decided(mode, outcome);
        tnc::with_state(|s| s.mode_job = Some(setter));
        report_outcome(&setter);
        *job = None;
        return;
    }
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

/// Write the firmware image stored on the node to the TNC through its
/// bootloader (the flashtnc procedure, [`BootloaderFlasher`]). Takes over the
/// serial line for the few minutes it runs. Returns whether the TNC was
/// updated (the caller then starts over: report first, then settings).
async fn run_update(modem: &mut Modem, mode_job: &mut Option<ModeSetter>) -> bool {
    fn refuse(why: &'static str) {
        tnc::with_state(|s| s.flash = FlashStatus::Refused(why));
        tnc::log(Direction::Info, why);
    }
    let Some(image) = tnc::staged_image() else {
        refuse("No firmware file is stored on the node; upload one first.");
        return false;
    };
    if !tnc_image::verify(&image) {
        refuse("The stored firmware file is damaged; upload it again.");
        return false;
    }
    // The bootloader check is the authority, but a TNC that has told us its
    // chip lets us refuse the wrong file before touching it at all.
    let reported = tnc::with_state(|s| s.status.and_then(|st| st.firmware_version));
    if let Some(chip) = reported.map(|v| v.chip_variant()) {
        if chip != ChipVariant::Unknown && chip != image.target {
            refuse(if chip == ChipVariant::Dspic33Ep512 {
                "This TNC runs firmware 4.x (dsPIC33EP512GP): upload the v4 file, not v3."
            } else {
                "This TNC runs firmware 3.x (dsPIC33EP256GP): upload the v3 file, not v4."
            });
            return false;
        }
    }

    *mode_job = None;
    let (name, lines) = (image.name(), image.lines);
    tnc::log_with(Direction::Info, |w| {
        write!(w, "Updating the TNC firmware from {name} ({lines} lines). Do not power off the TNC.")
    });
    let timings = FlashTimings::default();
    let (mut flasher, first) = BootloaderFlasher::start(image.target, image.lines, tnc::now_ms(), timings);
    tnc::with_state(|s| s.flash = FlashStatus::Flashing(flasher));
    let mut reader = LineReader::new(&image);
    perform(modem, &mut flasher, first, &mut reader, &timings).await;

    let mut buf = [0u8; 32];
    while flasher.outcome().is_none() {
        // Every state but "writing a line" has a deadline, and lines are written
        // inside `perform`; the fallback only guards against a stall.
        let deadline = flasher.deadline_ms().unwrap_or(tnc::now_ms() + 1_000);
        let woke = select(
            modem.stream_mut().read(&mut buf),
            Timer::at(Instant::from_millis(deadline)),
        )
        .await;
        match woke {
            Either::First(Ok(n)) => {
                for &b in &buf[..n] {
                    let actions = flasher.on_byte(b, tnc::now_ms());
                    perform(modem, &mut flasher, actions, &mut reader, &timings).await;
                    if flasher.outcome().is_some() {
                        break;
                    }
                }
            }
            Either::First(Err(_)) => Timer::after_millis(10).await,
            Either::Second(()) => {
                let actions = flasher.on_time(tnc::now_ms());
                perform(modem, &mut flasher, actions, &mut reader, &timings).await;
            }
        }
        tnc::with_state(|s| s.flash = FlashStatus::Flashing(flasher));
    }
    tnc::with_state(|s| {
        s.flash = FlashStatus::Flashing(flasher);
        s.status = None; // whatever the TNC said before no longer holds
    });
    let outcome = flasher.outcome().unwrap_or(FlashOutcome::Failed {
        kind: ax25_node_core::kiss::ninotnc::flash::FailureKind::NoResponse,
        line: None,
        written: 0,
        byte: None,
    });
    tnc::log_with(Direction::Info, |w| write_flash_outcome(&outcome, w));
    modem.reset_decoder();

    match outcome {
        FlashOutcome::Done { .. } => {
            // First boot of new firmware: a bootloader self-update (~2 s), then KISS.
            Timer::after_secs(5).await;
            tnc::log(
                Direction::Info,
                "Asking the updated TNC for its report; the node's saved settings follow",
            );
            true
        }
        o if o.nothing_written() => {
            Timer::after_secs(2).await;
            send_get_all(modem).await;
            false
        }
        // Mid-write failure: the TNC sits in its bootloader until an update
        // completes, so there is nothing to talk KISS to.
        _ => false,
    }
}

/// Carry out the flasher's actions on the serial line.
async fn perform(
    modem: &mut Modem,
    flasher: &mut BootloaderFlasher,
    actions: ax25_node_core::kiss::ninotnc::flash::Actions,
    reader: &mut LineReader,
    timings: &FlashTimings,
) {
    for action in actions {
        match action {
            FlashAction::Write(bytes) => {
                let _ = modem.stream_mut().write(bytes).await;
            }
            FlashAction::DiscardInput => discard_input(modem).await,
            FlashAction::SendLine { index, paced } => {
                let mut line = [0u8; MAX_LINE + 1];
                let Some(n) = reader.next_line(index, &mut line) else {
                    flasher.line_unavailable();
                    return;
                };
                if paced {
                    for c in &line[..n] {
                        let _ = modem.stream_mut().write(core::slice::from_ref(c)).await;
                        Timer::after_millis(timings.first_line_char_delay_ms).await;
                    }
                } else {
                    let _ = modem.stream_mut().write(&line[..n]).await;
                }
                flasher.line_sent(tnc::now_ms());
            }
        }
    }
}

/// Read and drop whatever is waiting on the serial line (at most 2 s' worth).
async fn discard_input(modem: &mut Modem) {
    let mut buf = [0u8; 64];
    let until = Instant::now() + Duration::from_secs(2);
    while Instant::now() < until {
        match select(modem.stream_mut().read(&mut buf), Timer::after_millis(5)).await {
            Either::First(_) => continue,
            Either::Second(()) => break,
        }
    }
}

/// Configure UART1 as a buffered 8N1 UART at `baud` on GP20 (TX) / GP21 (RX) —
/// A plain name for a UART receive error, for the monitor.
fn line_error_name(e: &ax25_node_core::kiss::serial::ModemError<UartError>) -> &'static str {
    use ax25_node_core::kiss::serial::ModemError;
    match e {
        ModemError::Io(UartError::Overrun) => "overrun",
        ModemError::Io(UartError::Break) => "break",
        ModemError::Io(UartError::Parity) => "parity",
        ModemError::Io(UartError::Framing) => "framing",
        _ => "other",
    }
}

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
