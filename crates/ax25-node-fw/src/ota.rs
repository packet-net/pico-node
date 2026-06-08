//! OTA firmware update (docs/OTA.md) — the application half of the A/B scheme.
//!
//! Two responsibilities:
//!
//! 1. [`mark_booted_early`] — called once at the very top of boot. It writes the
//!    embassy-boot "boot OK" magic so that, if this boot is a *trial* of a just-
//!    swapped image, the bootloader keeps it; if we never get here (the image
//!    hangs/panics before this point), the next reset reverts to the previous
//!    image. A normal boot (magic already set) writes nothing.
//!
//! 2. [`http_task`] — the node's single web server (port 80, **both** AP and STA
//!    modes; see [`WebCtx`]). It renders the mode-appropriate config panel and
//!    accepts a raw firmware image (`POST /firmware`), streaming it straight into
//!    the DFU partition via `FirmwareUpdater`, marking it for swap, and resetting.
//!    The bootloader then swaps DFU↔ACTIVE and boots the new image on trial.
//!
//! Flash sharing: there is one `FLASH` peripheral, owned by `config_store`. The
//! OTA path *takes* it ([`config_store::take_flash_for_ota`]) and never gives it
//! back — every OTA path ends in a reset, so that's fine.
//!
//! Security note: the upload is unauthenticated, like the captive portal —
//! anyone on the node's LAN can push firmware. Acceptable for a hobby node on a
//! trusted LAN; gate it (token / signed images via embassy-boot's `_verify`)
//! before exposing a node to an untrusted network.

use core::cell::RefCell;

use embassy_boot_rp::{
    AlignedBuffer, BlockingFirmwareState, BlockingFirmwareUpdater, FirmwareUpdaterConfig,
};
use embassy_net::tcp::TcpSocket;
use embassy_net::Stack;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::blocking_mutex::Mutex as BlockingMutex;
use embassy_time::{Duration, Timer};

use crate::config_store::{self, ConfigFlash};

/// A build identifier surfaced at `GET /version` (plain text) so a deployed
/// node self-reports which firmware it is running — useful for confirming an
/// OTA swap took effect remotely. Defaults to the crate version; set
/// `OTA_BUILD_TAG` in the build env (e.g. a release tag or commit) to override.
pub const BUILD_TAG: &str = match option_env!("OTA_BUILD_TAG") {
    Some(t) => t,
    None => env!("CARGO_PKG_VERSION"),
};

/// The web server port — :80 in both AP and STA mode (the same server renders
/// the captive-portal config form in AP and the node panel in STA).
const OTA_PORT: u16 = 80;

/// DFU partition capacity (must match memory.x DFU LENGTH). Uploads larger than
/// this are rejected up front.
const DFU_CAPACITY: usize = 516 * 1024;

type SharedFlash = BlockingMutex<NoopRawMutex, RefCell<ConfigFlash>>;

/// Mark the current firmware as good (confirm any pending trial). Idempotent and
/// wear-free on a normal boot (the magic is already set → no flash write).
/// Takes the flash by value and hands it straight back so the caller can pass it
/// on to `config_store`.
pub fn mark_booted_early(flash: ConfigFlash) -> ConfigFlash {
    let shared: SharedFlash = BlockingMutex::new(RefCell::new(flash));
    {
        let cfg = FirmwareUpdaterConfig::from_linkerfile_blocking(&shared, &shared);
        let mut aligned = AlignedBuffer([0u8; 1]);
        let mut state = BlockingFirmwareState::from_config(cfg, &mut aligned.0);
        match state.mark_booted() {
            Ok(()) => defmt::info!("ota: firmware marked good (trial, if any, confirmed)"),
            Err(e) => defmt::warn!("ota: mark_booted failed: {} (running un-chained?)", e),
        }
    }
    shared.into_inner().into_inner()
}

/// What the web server needs to render the mode-appropriate panel. Fixed at
/// boot, so cheap `Copy` static strings.
#[derive(Clone, Copy)]
pub struct WebCtx {
    /// `true` = STA (LAN) mode, `false` = AP mode.
    pub sta: bool,
    /// mDNS/DHCP name, shown as `<hostname>.local` in STA mode.
    pub hostname: &'static str,
    /// This node's own AP SSID (`pico-<callsign>`), shown in AP mode.
    pub ap_ssid: &'static str,
    /// The AP's WPA2 passphrase, shown in AP mode for reference.
    pub ap_pass: &'static str,
}

/// The node's single web server — config panel + firmware upload — on :80 in
/// **both** modes (the AP captive-portal form and the STA panel are the same
/// server, rendered per [`WebCtx::sta`]). The AP-only DHCP + DNS catch-all live
/// in `crate::provisioning`.
#[embassy_executor::task]
pub async fn http_task(stack: Stack<'static>, ctx: WebCtx) {
    let mut rx_buf = [0u8; 4096];
    let mut tx_buf = [0u8; 1024];
    defmt::info!(
        "web: server up on :{} ({=str} mode)",
        OTA_PORT,
        if ctx.sta { "STA" } else { "AP" }
    );
    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buf, &mut tx_buf);
        socket.set_timeout(Some(Duration::from_secs(30)));
        if socket.accept(OTA_PORT).await.is_err() {
            socket.close();
            continue;
        }
        serve_conn(&mut socket, stack, ctx).await;
        socket.close();
    }
}

/// Handle one connection. `GET /` → the mode-aware panel; `GET /version` → build
/// tag; `POST /save` → apply config + reboot; `POST /join` → join WiFi + reboot
/// to STA (AP mode); `POST /apmode` → reboot into AP (STA mode); `POST /firmware`
/// → stream into DFU, mark, reset.
async fn serve_conn(socket: &mut TcpSocket<'_>, stack: Stack<'static>, ctx: WebCtx) {
    // Read until the end of the request headers (carrying any body bytes that
    // arrive in the same segment). 2 KiB holds the headers plus a small config
    // POST body (firmware bodies are streamed separately, not buffered here).
    let mut hdr = [0u8; 2048];
    let mut hlen = 0usize;
    let header_end;
    loop {
        match socket.read(&mut hdr[hlen..]).await {
            Ok(0) => return,
            Ok(n) => {
                hlen += n;
                if let Some(p) = find(&hdr[..hlen], b"\r\n\r\n") {
                    header_end = p + 4;
                    break;
                }
                if hlen == hdr.len() {
                    return; // headers too large
                }
            }
            Err(_) => return,
        }
    }
    let headers = &hdr[..header_end];

    // A tiny machine-readable endpoint: which build is running right now. The
    // OTA bench test reads this before/after a swap to confirm the new image
    // took effect (and that a rollback restored the old one).
    if headers.starts_with(b"GET /version") {
        let _ = http_send(socket, "text/plain", BUILD_TAG.as_bytes()).await;
        let _ = socket.flush().await;
        return;
    }

    // `POST /save` — apply the identity/config form and reboot to take effect.
    if headers.starts_with(b"POST /save") {
        let content_len = parse_content_length(headers).unwrap_or(0);
        while hlen < header_end + content_len && hlen < hdr.len() {
            match socket.read(&mut hdr[hlen..]).await {
                Ok(0) => break,
                Ok(n) => hlen += n,
                Err(_) => break,
            }
        }
        if crate::provisioning::apply_config_form(&hdr[..hlen]) {
            // In STA mode the node rejoins the *same* WiFi at the same address, so
            // we can auto-return to the panel once it's back. In AP mode the SSID
            // may change (if the callsign changed) and the client must reconnect
            // to the AP, so a plain notice is correct there.
            if ctx.sta {
                let page = crate::webui::notice_reconnect(
                    "Saved — rebooting",
                    "Applying the new configuration and restarting; reconnect in ~20s.",
                );
                let _ = http_send(socket, "text/html", page.as_bytes()).await;
            } else {
                let page = crate::webui::notice(
                    "Saved — rebooting",
                    "Applying the new configuration and restarting in AP mode. If you \
changed the callsign the AP name changes too — reconnect to it in ~15s.",
                );
                let _ = http_send(socket, "text/html", page.as_bytes()).await;
            }
            let _ = socket.flush().await;
            defmt::info!("config saved via web; rebooting to apply");
            crate::nlog!("config saved (web), rebooting");
            Timer::after_millis(800).await;
            cortex_m::peripheral::SCB::sys_reset();
        } else {
            // Nothing set (all blank) — just re-show the panel.
            let _ = write_panel(socket, stack, ctx).await;
            let _ = socket.flush().await;
        }
        return;
    }

    // `POST /join` (AP mode) — the conscious path back to LAN: stage the supplied
    // WiFi, clear the sticky AP flag, reboot into STA.
    if headers.starts_with(b"POST /join") {
        let content_len = parse_content_length(headers).unwrap_or(0);
        while hlen < header_end + content_len && hlen < hdr.len() {
            match socket.read(&mut hdr[hlen..]).await {
                Ok(0) => break,
                Ok(n) => hlen += n,
                Err(_) => break,
            }
        }
        if crate::provisioning::apply_join_form(&hdr[..hlen]) {
            let page = crate::webui::notice(
                "Joining — rebooting",
                "Leaving AP mode and joining the Wi-Fi network. This access point \
will disappear; reconnect your device to your LAN to find the node again (it \
returns here only if the network can't be joined).",
            );
            let _ = http_send(socket, "text/html", page.as_bytes()).await;
            let _ = socket.flush().await;
            defmt::info!("join requested via web; rebooting into STA");
            crate::nlog!("join WiFi (web), rebooting to LAN");
            Timer::after_millis(800).await;
            cortex_m::peripheral::SCB::sys_reset();
        } else {
            let _ = write_panel(socket, stack, ctx).await;
            let _ = socket.flush().await;
        }
        return;
    }

    // `POST /apmode` (STA mode) — "Switch to AP mode": set the sticky FORCE_AP
    // flag and reboot. AP mode is a first-class operating mode (portable/hilltop
    // use), not just setup; it stays in AP until the conscious join flow.
    if headers.starts_with(b"POST /apmode") {
        use ax25_node_core::console::ConfigOp;
        let _ = config_store::handle_op(&ConfigOp::Set {
            key: alloc::string::String::from("FORCE_AP"),
            value: alloc::string::String::from("true"),
        });
        let (_t, _) = config_store::handle_op(&ConfigOp::Save);
        let page = crate::webui::notice(
            "Switching to AP mode",
            "Rebooting as a standalone access point. Join its Wi-Fi network (shown \
on the node's display) and browse to <code>192.168.4.1</code> to manage it. It \
stays in AP mode until you join a network from there.",
        );
        let _ = http_send(socket, "text/html", page.as_bytes()).await;
        let _ = socket.flush().await;
        defmt::info!("FORCE_AP set via web; rebooting into AP mode");
        crate::nlog!("switch to AP mode (web), rebooting");
        Timer::after_millis(800).await;
        cortex_m::peripheral::SCB::sys_reset();
    }

    if !headers.starts_with(b"POST /firmware") {
        // Any other GET → the mode-aware node panel.
        let _ = write_panel(socket, stack, ctx).await;
        let _ = socket.flush().await;
        return;
    }

    let content_len = parse_content_length(headers).unwrap_or(0);
    if content_len == 0 || content_len > DFU_CAPACITY {
        let _ = http_send(socket, "text/plain", b"bad firmware size").await;
        let _ = socket.flush().await;
        return;
    }

    let Some(flash) = config_store::take_flash_for_ota() else {
        let _ = http_send(socket, "text/plain", b"OTA unavailable (flash busy)").await;
        let _ = socket.flush().await;
        return;
    };

    defmt::info!("ota: receiving {} byte image into DFU", content_len);
    crate::nlog!("OTA: receiving {}-byte image", content_len);

    let ok = stream_to_dfu(socket, flash, &hdr[header_end..hlen], content_len).await;

    // Either way we reset: success swaps in the new image; failure boots the
    // (untouched, unmarked) current image cleanly — config_store is gone, so a
    // reset is the right way back to a working node.
    if ok {
        let page = crate::webui::notice(
            "Firmware staged — rebooting",
            "Rebooting to swap in the new image (on trial). Reconnect in ~30s. If \
it misbehaves the node rolls back automatically on the following reboot.",
        );
        let _ = http_send(socket, "text/html", page.as_bytes()).await;
        let _ = socket.flush().await;
        defmt::info!("ota: image staged + marked; rebooting to swap");
        crate::nlog!("OTA: staged OK, rebooting to swap");
    } else {
        let _ = http_send(socket, "text/plain", b"OTA write failed; rebooting unchanged").await;
        let _ = socket.flush().await;
        defmt::warn!("ota: write failed; not marked — rebooting current firmware");
        crate::nlog!("OTA: write FAILED, rebooting unchanged");
    }
    Timer::after_millis(800).await;
    cortex_m::peripheral::SCB::sys_reset();
}

/// Stream the request body into the DFU partition. `initial` is the body bytes
/// already read alongside the headers. Returns whether the full image was
/// written and marked for swap.
async fn stream_to_dfu(
    socket: &mut TcpSocket<'_>,
    flash: ConfigFlash,
    initial: &[u8],
    content_len: usize,
) -> bool {
    let shared: SharedFlash = BlockingMutex::new(RefCell::new(flash));
    let cfg = FirmwareUpdaterConfig::from_linkerfile_blocking(&shared, &shared);
    let mut aligned = AlignedBuffer([0u8; 1]);
    let mut updater = BlockingFirmwareUpdater::new(cfg, &mut aligned.0);

    let mut written = 0usize;
    if !initial.is_empty() {
        let take = initial.len().min(content_len);
        if updater.write_firmware(0, &initial[..take]).is_err() {
            return false;
        }
        written = take;
    }

    let mut chunk = [0u8; 4096];
    while written < content_len {
        match socket.read(&mut chunk).await {
            Ok(0) => break, // peer closed early — short upload
            Ok(n) => {
                let take = n.min(content_len - written);
                if updater.write_firmware(written, &chunk[..take]).is_err() {
                    defmt::warn!("ota: DFU write error at offset {}", written);
                    return false;
                }
                written += take;
            }
            Err(_) => return false,
        }
    }

    if written != content_len {
        defmt::warn!("ota: short upload {}/{} bytes", written, content_len);
        return false;
    }
    updater.mark_updated().is_ok()
}

// ── small HTTP helpers ──

async fn http_send(socket: &mut TcpSocket<'_>, ctype: &str, body: &[u8]) -> bool {
    use core::fmt::Write;
    let mut head = heapless::String::<160>::new();
    let _ = write!(
        head,
        "HTTP/1.0 200 OK\r\nContent-Type: {}\r\nContent-Length: {}\r\nConnection: close\r\n\r\n",
        ctype,
        body.len()
    );
    write_all(socket, head.as_bytes()).await && write_all(socket, body).await
}

async fn write_all(socket: &mut TcpSocket<'_>, mut bytes: &[u8]) -> bool {
    while !bytes.is_empty() {
        match socket.write(bytes).await {
            Ok(0) | Err(_) => return false,
            Ok(n) => bytes = &bytes[n..],
        }
    }
    true
}

fn find(hay: &[u8], needle: &[u8]) -> Option<usize> {
    hay.windows(needle.len()).position(|w| w == needle)
}

/// Parse the `Content-Length` header value (case-insensitive name).
fn parse_content_length(headers: &[u8]) -> Option<usize> {
    let mut i = 0;
    while i < headers.len() {
        // start of a line
        let line_end = find(&headers[i..], b"\r\n").map(|p| i + p).unwrap_or(headers.len());
        let line = &headers[i..line_end];
        if let Some(colon) = line.iter().position(|&b| b == b':') {
            if eq_ascii_ci(&line[..colon], b"content-length") {
                let val = &line[colon + 1..];
                let mut n = 0usize;
                let mut seen = false;
                for &b in val {
                    if b.is_ascii_digit() {
                        n = n.saturating_mul(10).saturating_add((b - b'0') as usize);
                        seen = true;
                    } else if seen {
                        break;
                    }
                }
                if seen {
                    return Some(n);
                }
            }
        }
        i = line_end + 2;
    }
    None
}

fn eq_ascii_ci(a: &[u8], b: &[u8]) -> bool {
    a.len() == b.len() && a.iter().zip(b).all(|(x, y)| x.eq_ignore_ascii_case(y))
}

// Static page fragments around the two small dynamic pieces (status header +
// config form). Split out so the panel can be sent with a correct
// `Content-Length` without ever holding the whole page on the heap.
const PANEL_HEAD: &str = "<!DOCTYPE html><html lang=en><head><meta charset=utf-8>\
<meta name=viewport content=\"width=device-width,initial-scale=1\">\
<title>pico-node</title><style>";
const PANEL_STYLE_MID: &str = "</style></head><body>";
const PANEL_TAIL: &str = "</body></html>";

/// Send the mode-aware node panel — a status header plus, in **STA** mode, the
/// pre-filled config form + firmware + "Switch to AP mode"; in **AP** mode, an
/// identity form + a conscious "Join a WiFi network" section + firmware.
///
/// **Heap discipline:** the heap is only 16 KiB and shared with the
/// session/codec allocations; building the whole page as one `String` peaked
/// well past that and the alloc-error handler halts the node. So only the small
/// dynamic pieces are allocated (header + the form section(s), ≲1.5 KiB total);
/// everything else is written from `&str` constants. The total is measured so
/// the response carries a real `Content-Length` (no close-delimited body → the
/// client and the single-socket server don't wait on the FIN).
async fn write_panel(socket: &mut TcpSocket<'_>, stack: Stack<'static>, ctx: WebCtx) -> bool {
    use crate::webui::{ap_identity_section, ap_join_section, esc, sta_config_form, CSS};
    use alloc::format;
    use alloc::string::String;
    use core::fmt::Write;

    let p = config_store::current_pending();

    // Dynamic pieces (mode-specific). `form_b` is empty/unused in STA.
    let (header, form_a, form_b): (String, String, String) = if ctx.sta {
        let call = p.callsign.as_deref().unwrap_or("(unconfigured)");
        let alias = p.alias.as_deref().unwrap_or("");
        let grid = p.grid.as_deref().unwrap_or("");
        let ip = stack
            .config_v4()
            .map(|c| format!("{}", c.address.address()))
            .unwrap_or_else(|| String::from("—"));
        let mut idline = String::new();
        if !alias.is_empty() {
            idline += &esc(alias);
        }
        if !grid.is_empty() {
            if !idline.is_empty() {
                idline += " · ";
            }
            idline += &esc(grid);
        }
        let header = format!(
            "<div class=spread><h1 class=mono>{}</h1>\
<span class=sub><span class=dot>●</span> online</span></div>{}\
<p class=\"sub mono\">{}.local · {} · {}</p>",
            esc(call),
            if idline.is_empty() {
                String::new()
            } else {
                format!("<p class=sub>{idline}</p>")
            },
            esc(ctx.hostname),
            esc(&ip),
            esc(BUILD_TAG),
        );
        (header, sta_config_form(&p), String::new())
    } else {
        let header = format!(
            "<div class=spread><h1 class=mono>{}</h1>\
<span class=sub><span class=dot>●</span> AP mode</span></div>\
<p class=\"sub mono\">Wi-Fi pass: {} · 192.168.4.1 · {}</p>",
            esc(ctx.ap_ssid),
            esc(ctx.ap_pass),
            esc(BUILD_TAG),
        );
        (header, ap_identity_section(&p), ap_join_section())
    };

    // Assemble the body as an ordered list of byte slices (static + dynamic);
    // measured for Content-Length, written in order. Nothing holds the whole page.
    let mut parts: heapless::Vec<&[u8], 9> = heapless::Vec::new();
    let _ = parts.push(PANEL_HEAD.as_bytes());
    let _ = parts.push(CSS.as_bytes());
    let _ = parts.push(PANEL_STYLE_MID.as_bytes());
    let _ = parts.push(header.as_bytes());
    let _ = parts.push(form_a.as_bytes());
    if !ctx.sta {
        let _ = parts.push(form_b.as_bytes());
    }
    let _ = parts.push(FIRMWARE_SECTION.as_bytes());
    if ctx.sta {
        let _ = parts.push(MAINTENANCE_SECTION.as_bytes());
    }
    let _ = parts.push(PANEL_TAIL.as_bytes());

    let body_len: usize = parts.iter().map(|s| s.len()).sum();
    let mut head = heapless::String::<96>::new();
    let _ = write!(
        head,
        "HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nContent-Length: {}\r\nConnection: close\r\n\r\n",
        body_len
    );
    if !write_all(socket, head.as_bytes()).await {
        return false;
    }
    for part in parts {
        if !write_all(socket, part).await {
            return false;
        }
    }
    true
}

/// The firmware-update section (static): file picker + a tiny uploader that
/// POSTs the raw image to `/firmware` and shows the server's reply.
const FIRMWARE_SECTION: &str = "<section><h2>Firmware</h2>\
<p class=hint>Upload the raw <code>pico-node-app.bin</code> (not the .uf2). The node \
writes it to the spare partition, reboots on trial, and rolls back automatically if it \
fails to come up.</p>\
<div class=row><input type=file id=f accept=\".bin,application/octet-stream\">\
<button class=ghost onclick=up()>Upload</button></div>\
<div id=log></div></section>\
<script>function log(m){document.getElementById('log').textContent=m}\
async function up(){var f=document.getElementById('f').files[0];\
if(!f){log('Pick a .bin file first.');return}\
log('Uploading '+f.size+' bytes… do not power off.');\
try{var r=await fetch('/firmware',{method:'POST',body:f});\
log(await r.text())}catch(e){log('Upload sent; if the node accepted it, it is now \
rebooting. Reconnect in ~30s.')}}</script>";

/// The maintenance section (static, STA only): switch to AP mode — a first-class
/// operating mode (portable/hilltop), not just setup.
const MAINTENANCE_SECTION: &str = "<section><h2>Maintenance</h2>\
<p class=hint>Switch to <strong>AP mode</strong> — run as a standalone access point for \
portable/hilltop use, or to manage the node without infrastructure Wi-Fi. It stays in AP \
mode until you join a network from there.</p>\
<form method=post action=/apmode><button type=submit class=ghost>Switch to AP mode</button></form>\
</section>";
