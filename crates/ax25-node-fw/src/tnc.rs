//! The NinoTNC setup surface shared between the web server and the serial KISS
//! task: a command queue (web -> serial), the TNC status the page shows, and the
//! traffic monitor.
//!
//! The serial task ([`crate::ports::ninotnc`]) owns the UART and is the
//! only thing that talks to the TNC. The web server never touches the link; it
//! queues a [`TncCommand`] and the page then watches [`STATE`] and [`MONITOR`]
//! through `GET /tnc/poll`.
//!
//! Everything here is static (no heap): the node's heap is 16 KiB and shared
//! with the session layer, so the monitor ring and the poll response live in
//! fixed buffers.

use core::cell::RefCell;
use core::fmt::{self, Write};

use ax25_node_core::ax25::Callsign;
use ax25_node_core::kiss::ninotnc::mode_set::{write_dip, write_mode};
use ax25_node_core::kiss::ninotnc::flash::{write_chip, BootloaderFlasher, FlashOutcome};
use ax25_node_core::kiss::ninotnc::{ModeSetter, NinoTncStatusFrame};
use ax25_node_core::monitor::{Direction, MonitorLog};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::channel::Channel;
use embassy_time::Instant;

use crate::config_store::TncSettings;

/// Longest test-frame text accepted from the page.
pub const TEST_TEXT_MAX: usize = 128;

/// A request from the web page to the serial task.
pub enum TncCommand {
    /// SETHW to `mode`, read back and retry until it takes (or clearly won't).
    SetMode { mode: u8, persist_to_flash: bool },
    /// Send these KISS parameters (the ones that are `Some`).
    SetParams(TncSettings),
    /// Transmit one UI frame from the node's callsign.
    SendTest {
        dest: Callsign,
        text: heapless::String<TEST_TEXT_MAX>,
    },
    /// Ask the TNC for its diagnostic report (GETALL).
    Refresh,
    /// Write the stored firmware image to the TNC through its bootloader.
    UpdateFirmware,
}

/// Where a TNC firmware update is up to.
#[derive(Clone, Copy)]
pub enum FlashStatus {
    Idle,
    /// Asked for; the serial task has not picked it up yet.
    Starting,
    /// Refused before starting, with the reason.
    Refused(&'static str),
    /// Running, or finished (the flasher holds the outcome).
    Flashing(BootloaderFlasher),
}

/// Web -> serial task. Small: the page sends one command per click.
pub static CMD: Channel<CriticalSectionRawMutex, TncCommand, 4> = Channel::new();

/// What the page shows about the link and the TNC.
pub struct TncState {
    /// The serial task is running (it only starts once a callsign is set).
    pub running: bool,
    /// When anything last arrived from the TNC (ms since boot).
    pub last_heard_ms: Option<u64>,
    /// AX.25 frames heard / sent since boot.
    pub rx_frames: u32,
    pub tx_frames: u32,
    /// Serial receive errors on the TNC link since boot; each dropped a frame.
    pub line_errors: u32,
    /// The TNC's last diagnostic or status report, and when it arrived.
    pub status: Option<NinoTncStatusFrame>,
    pub status_at_ms: u64,
    /// The current or most recent mode change.
    pub mode_job: Option<ModeSetter>,
    /// The current or most recent TNC firmware update.
    pub flash: FlashStatus,
    /// The firmware image stored on the node, once looked up.
    pub staged: Option<Option<crate::tnc_image::StagedImage>>,
}

pub static STATE: Mutex<CriticalSectionRawMutex, RefCell<TncState>> =
    Mutex::new(RefCell::new(TncState {
        running: false,
        last_heard_ms: None,
        rx_frames: 0,
        tx_frames: 0,
        line_errors: 0,
        status: None,
        status_at_ms: 0,
        mode_job: None,
        flash: FlashStatus::Idle,
        staged: None,
    }));

/// Monitor lines kept, and the width of each.
const MONITOR_LINES: usize = 40;
const MONITOR_WIDTH: usize = 224;

pub static MONITOR: Mutex<CriticalSectionRawMutex, RefCell<MonitorLog<MONITOR_LINES, MONITOR_WIDTH>>> =
    Mutex::new(RefCell::new(MonitorLog::new()));

/// Milliseconds since boot: the clock for the log and the mode-set machine.
pub fn now_ms() -> u64 {
    Instant::now().as_millis()
}

/// Append a monitor line produced by `write`.
pub fn log_with(dir: Direction, write: impl FnOnce(&mut dyn fmt::Write) -> fmt::Result) {
    let at = now_ms();
    MONITOR.lock(|m| {
        m.borrow_mut().push_with(dir, at, write);
    });
}

/// Append a plain monitor line.
pub fn log(dir: Direction, text: &str) {
    log_with(dir, |w| w.write_str(text));
}

/// Update the shared state.
pub fn with_state<R>(f: impl FnOnce(&mut TncState) -> R) -> R {
    STATE.lock(|s| f(&mut s.borrow_mut()))
}

/// Queue a command for the serial task. `Err` carries the reason to show the user.
pub fn submit(cmd: TncCommand) -> Result<(), &'static str> {
    if !with_state(|s| s.running) {
        return Err("The TNC link is not running. Set the node's callsign first.");
    }
    if flashing() {
        return Err("The TNC firmware is being updated; wait for it to finish.");
    }
    CMD.try_send(cmd)
        .map_err(|_| "The TNC link is busy; try again in a moment.")
}

/// Refuse settings and transmissions until the TNC has reported firmware the
/// node supports (3.44 / 4.44 or later).
fn require_supported() -> Result<(), alloc::string::String> {
    let version = with_state(|s| s.status.and_then(|st| st.firmware_version));
    match version {
        Some(v) if v.is_supported() => Ok(()),
        Some(v) => Err(alloc::format!(
            "The TNC runs firmware {}.{}; pico-node needs {}.{} or later. Update the TNC \
firmware first.",
            v.major,
            v.minor,
            v.major,
            ax25_node_core::kiss::ninotnc::firmware::MIN_SUPPORTED_MINOR
        )),
        None => Err("The TNC has not reported its firmware yet. Check the serial wiring and \
that the TNC is powered."
            .into()),
    }
}

/// Whether a TNC firmware update is in progress.
pub fn flashing() -> bool {
    with_state(|s| match s.flash {
        FlashStatus::Starting => true,
        FlashStatus::Flashing(f) => f.outcome().is_none(),
        _ => false,
    })
}

/// The firmware image stored on the node (read from flash once, then cached).
pub fn staged_image() -> Option<crate::tnc_image::StagedImage> {
    if let Some(cached) = with_state(|s| s.staged) {
        return cached;
    }
    let found = crate::tnc_image::staged();
    with_state(|s| s.staged = Some(found));
    found
}

/// A `fmt::Write` into a fixed byte buffer that fails when full (so a caller can
/// stop cleanly at a boundary it chose).
pub(crate) struct BufWriter<'a> {
    buf: &'a mut [u8],
    len: usize,
}

impl<'a> BufWriter<'a> {
    pub(crate) fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, len: 0 }
    }

    pub(crate) fn len(&self) -> usize {
        self.len
    }

    pub(crate) fn truncate(&mut self, len: usize) {
        self.len = self.len.min(len);
    }

    pub(crate) fn remaining(&self) -> usize {
        self.buf.len() - self.len
    }

}

impl fmt::Write for BufWriter<'_> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let b = s.as_bytes();
        if b.len() > self.remaining() {
            return Err(fmt::Error);
        }
        self.buf[self.len..self.len + b.len()].copy_from_slice(b);
        self.len += b.len();
        Ok(())
    }
}

/// Writes a JSON string body (without the quotes), escaping as needed.
struct JsonEscape<'a, W: fmt::Write>(&'a mut W);

impl<W: fmt::Write> fmt::Write for JsonEscape<'_, W> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for c in s.chars() {
            match c {
                '"' => self.0.write_str("\\\"")?,
                '\\' => self.0.write_str("\\\\")?,
                c if (c as u32) < 0x20 => write!(self.0, "\\u{:04x}", c as u32)?,
                c => self.0.write_char(c)?,
            }
        }
        Ok(())
    }
}

fn json_str<W: fmt::Write>(
    w: &mut W,
    f: impl FnOnce(&mut JsonEscape<'_, W>) -> fmt::Result,
) -> fmt::Result {
    w.write_char('"')?;
    f(&mut JsonEscape(w))?;
    w.write_char('"')
}

/// Render the `GET /tnc/poll?since=N` response into `out`: the TNC status plus
/// monitor lines after `since`, as many as fit. The client carries on from
/// `next`. Returns the length written.
pub fn render_poll(out: &mut [u8], since: u32) -> usize {
    let mut w = BufWriter::new(out);
    let now = now_ms();
    // The status block is small and bounded; the log fills what is left.
    let _ = write_status_json(&mut w, now);

    // A client ahead of the log (the node rebooted under an open page) starts over.
    let (oldest, mut next) = MONITOR.lock(|m| {
        let m = m.borrow();
        let since = if since >= m.next_seq() { 0 } else { since };
        (m.oldest_seq().unwrap_or(0), since)
    });
    let _ = write!(w, ",\"oldest\":{oldest},\"log\":[");
    let mut first = true;
    let mut more = false;
    loop {
        // One entry per lock, so the lock is never held while formatting much.
        let entry = MONITOR.lock(|m| m.borrow().since(next).next().copied());
        let Some(e) = entry else { break };
        let mark = w.len();
        if write_log_entry(&mut w, &e, first).is_err() || w.remaining() < 48 {
            // Out of room (keeping enough to close the document): the client
            // picks up from `next` on its following poll.
            w.truncate(mark);
            more = true;
            break;
        }
        first = false;
        next = e.seq();
    }
    let _ = write!(w, "],\"next\":{next},\"more\":{more}}}");
    w.len()
}

fn write_log_entry(
    w: &mut BufWriter<'_>,
    e: &ax25_node_core::monitor::Entry<MONITOR_WIDTH>,
    first: bool,
) -> fmt::Result {
    if !first {
        w.write_char(',')?;
    }
    write!(w, "[{},{},\"{}\",", e.seq(), e.at_ms(), e.direction().tag())?;
    json_str(w, |j| j.write_str(e.text()))?;
    w.write_char(']')
}

fn write_status_json(w: &mut BufWriter<'_>, now: u64) -> fmt::Result {
    let (running, heard, rx, tx, errs, status, job) = with_state(|s| {
        (
            s.running,
            s.last_heard_ms,
            s.rx_frames,
            s.tx_frames,
            s.line_errors,
            s.status,
            s.mode_job,
        )
    });
    write!(
        w,
        "{{\"now\":{now},\"running\":{running},\"rx\":{rx},\"tx\":{tx},\"errs\":{errs},\"heard\":"
    )?;
    match heard {
        Some(h) => write!(w, "{h}")?,
        None => w.write_str("null")?,
    }
    w.write_str(",\"tnc\":")?;
    match status {
        None => w.write_str("null")?,
        Some(st) => {
            w.write_str("{\"fw\":")?;
            json_str(w, |j| j.write_str(st.firmware_version_raw.as_str()))?;
            w.write_str(",\"dip\":")?;
            match st.dip_switches {
                Some(d) => json_str(w, |j| write_dip(d, j))?,
                None => w.write_str("null")?,
            }
            w.write_str(",\"mode\":")?;
            match (st.running_mode, st.firmware_mode_byte) {
                (Some(m), _) => json_str(w, |j| write_mode(m.mode, j))?,
                (None, Some(b)) => json_str(w, |j| write!(j, "unknown (0x{b:02X})"))?,
                (None, None) => w.write_str("null")?,
            }
            w.write_str(",\"ok\":")?;
            match st.firmware_version {
                Some(v) => write!(w, "{},\"major\":{}", v.is_supported(), v.major)?,
                None => w.write_str("null,\"major\":null")?,
            }
            write!(w, ",\"at\":{}", with_state(|s| s.status_at_ms))?;
            w.write_str(",\"uptime\":")?;
            match st.uptime_ms {
                Some(u) => write!(w, "{}", u / 1000)?,
                None => w.write_str("null")?,
            }
            w.write_str("}")?;
        }
    }
    w.write_str(",\"img\":")?;
    match staged_image() {
        None => w.write_str("null")?,
        Some(img) => {
            w.write_str("{\"name\":")?;
            json_str(w, |j| j.write_str(img.name()))?;
            write!(w, ",\"lines\":{},\"chip\":", img.lines)?;
            json_str(w, |j| write_chip(img.target, j))?;
            w.write_str("}")?;
        }
    }
    w.write_str(",\"flash\":")?;
    match with_state(|s| s.flash) {
        FlashStatus::Idle => w.write_str("null")?,
        FlashStatus::Starting => {
            w.write_str("{\"text\":\"Starting the update...\",\"running\":true,\"ok\":null}")?
        }
        FlashStatus::Refused(why) => {
            w.write_str("{\"text\":")?;
            json_str(w, |j| j.write_str(why))?;
            w.write_str(",\"running\":false,\"ok\":false}")?;
        }
        FlashStatus::Flashing(f) => {
            w.write_str("{\"text\":")?;
            json_str(w, |j| f.write_progress(j))?;
            let ok = match f.outcome() {
                None => "null",
                Some(FlashOutcome::Done { .. }) => "true",
                Some(_) => "false",
            };
            write!(w, ",\"running\":{},\"ok\":{ok}}}", f.outcome().is_none())?;
        }
    }
    w.write_str(",\"job\":")?;
    match job {
        None => w.write_str("null")?,
        Some(j) => {
            w.write_str("{\"text\":")?;
            json_str(w, |e| j.write_summary(e))?;
            write!(
                w,
                ",\"done\":{},\"ok\":{}}}",
                j.outcome().is_some(),
                matches!(
                    j.outcome(),
                    Some(ax25_node_core::kiss::ninotnc::ModeSetOutcome::Applied { .. })
                        | Some(ax25_node_core::kiss::ninotnc::ModeSetOutcome::SentUnverified { .. })
                )
            )?;
        }
    }
    Ok(())
}

/// Find `key` in a urlencoded body and decode its value.
pub fn form_value(body: &[u8], key: &str) -> Option<alloc::string::String> {
    body.split(|&b| b == b'&').find_map(|pair| {
        let eq = pair.iter().position(|&b| b == b'=')?;
        if &pair[..eq] != key.as_bytes() {
            return None;
        }
        crate::provisioning::url_decode(&pair[eq + 1..])
    })
}

/// Handle `POST /tnc/mode` (`mode`, optional `flash=on`). Saves the mode on the
/// node (re-applied at every boot, RAM-only) and queues the change.
pub fn post_mode(body: &[u8]) -> Result<&'static str, alloc::string::String> {
    require_supported()?;
    let mode = form_value(body, "mode").unwrap_or_default();
    let flash = form_value(body, "flash").is_some_and(|v| v == "on");
    crate::config_store::set_tnc_and_save(&[("TNC_MODE", mode.as_str())])?;
    let mode: u8 = mode.trim().parse().map_err(|_| "bad mode")?;
    submit(TncCommand::SetMode {
        mode,
        persist_to_flash: flash,
    })?;
    Ok("Mode change sent; watch the status line for the TNC's answer.")
}

/// Handle `POST /tnc/params`. Blank fields keep their current value.
pub fn post_params(body: &[u8]) -> Result<&'static str, alloc::string::String> {
    require_supported()?;
    let mut owned: heapless::Vec<(&'static str, alloc::string::String), 5> = heapless::Vec::new();
    for (form, key) in [
        ("txdelay", "TXDELAY"),
        ("persist", "PERSIST"),
        ("slottime", "SLOTTIME"),
        ("txtail", "TXTAIL"),
        ("duplex", "DUPLEX"),
    ] {
        if let Some(v) = form_value(body, form) {
            if !v.trim().is_empty() {
                let _ = owned.push((key, v));
            }
        }
    }
    let fields: heapless::Vec<(&str, &str), 5> =
        owned.iter().map(|(k, v)| (*k, v.as_str())).collect();
    let saved = crate::config_store::set_tnc_and_save(&fields)?;
    submit(TncCommand::SetParams(saved))?;
    Ok("Parameters saved and sent to the TNC.")
}

/// Handle `POST /tnc/test` (`dest`, `text`).
pub fn post_test(body: &[u8]) -> Result<&'static str, alloc::string::String> {
    require_supported()?;
    let dest = form_value(body, "dest").unwrap_or_default();
    let dest = Callsign::parse(dest.trim()).ok_or("Destination is not a valid callsign.")?;
    let text = form_value(body, "text").unwrap_or_default();
    let mut t = heapless::String::<TEST_TEXT_MAX>::new();
    for c in text.chars() {
        let c = if (' '..='~').contains(&c) { c } else { ' ' };
        if t.push(c).is_err() {
            break;
        }
    }
    submit(TncCommand::SendTest { dest, text: t })?;
    Ok("Test frame queued.")
}

/// Handle `POST /tnc/refresh`.
pub fn post_refresh() -> Result<&'static str, alloc::string::String> {
    submit(TncCommand::Refresh)?;
    Ok("Asked the TNC for its report.")
}

/// Handle `POST /tnc/update`: write the stored image to the TNC.
pub fn post_update() -> Result<&'static str, alloc::string::String> {
    if staged_image().is_none() {
        return Err("Upload a firmware file first.".into());
    }
    submit(TncCommand::UpdateFirmware)?;
    // Replaces the previous attempt's outcome at once, and makes `submit`
    // refuse other commands until the serial task has finished.
    with_state(|s| s.flash = FlashStatus::Starting);
    Ok("Update started. Keep the TNC powered; progress shows below.")
}
