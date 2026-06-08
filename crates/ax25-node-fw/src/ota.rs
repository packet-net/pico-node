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
//! 2. [`http_task`] — an HTTP server (STA mode, port 80) that accepts a raw
//!    firmware image (`POST /firmware`), streams it straight into the DFU
//!    partition via `FirmwareUpdater`, marks it for swap, and resets. The
//!    bootloader then swaps DFU↔ACTIVE and boots the new image on trial.
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

/// The OTA HTTP server port (STA mode). In AP mode the captive portal owns :80,
/// so OTA isn't offered there (you're physically present → BOOTSEL).
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

/// The firmware-upload HTTP server. STA mode only (see [`OTA_PORT`]).
#[embassy_executor::task]
pub async fn http_task(stack: Stack<'static>) {
    let mut rx_buf = [0u8; 4096];
    let mut tx_buf = [0u8; 1024];
    defmt::info!("ota: firmware-upload server up on :{}", OTA_PORT);
    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buf, &mut tx_buf);
        socket.set_timeout(Some(Duration::from_secs(30)));
        if socket.accept(OTA_PORT).await.is_err() {
            socket.close();
            continue;
        }
        serve_conn(&mut socket).await;
        socket.close();
    }
}

/// Handle one connection: GET → the upload page; `POST /firmware` → stream into
/// DFU, mark, reset.
async fn serve_conn(socket: &mut TcpSocket<'_>) {
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

    // In-place configuration: the captive-portal form, served on the live node's
    // web server (STA mode) so a deployed node can be reconfigured WITHOUT
    // re-onboarding/BOOTSEL. `GET /config` shows the form; `POST /save` applies
    // it (same handler as the AP portal) and reboots to take effect.
    if headers.starts_with(b"GET /config") {
        let _ = http_send(socket, "text/html", crate::provisioning::FORM_PAGE.as_bytes()).await;
        let _ = socket.flush().await;
        return;
    }
    if headers.starts_with(b"POST /save") {
        let content_len = parse_content_length(headers).unwrap_or(0);
        // Pull the rest of the (small) urlencoded body into the buffer.
        while hlen < header_end + content_len && hlen < hdr.len() {
            match socket.read(&mut hdr[hlen..]).await {
                Ok(0) => break,
                Ok(n) => hlen += n,
                Err(_) => break,
            }
        }
        if crate::provisioning::apply_config_form(&hdr[..hlen]) {
            let _ = http_send(socket, "text/html", CONFIG_SAVED_PAGE.as_bytes()).await;
            let _ = socket.flush().await;
            defmt::info!("config saved via web; rebooting to apply");
            crate::nlog!("config saved (web), rebooting");
            Timer::after_millis(800).await;
            cortex_m::peripheral::SCB::sys_reset();
        } else {
            // Nothing set (all blank) — just re-show the form.
            let _ = http_send(socket, "text/html", crate::provisioning::FORM_PAGE.as_bytes()).await;
            let _ = socket.flush().await;
        }
        return;
    }

    if !headers.starts_with(b"POST /firmware") {
        let _ = http_send(socket, "text/html", UPLOAD_PAGE.as_bytes()).await;
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
        let _ = http_send(socket, "text/html", DONE_PAGE.as_bytes()).await;
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

const UPLOAD_PAGE: &str = "<!DOCTYPE html><html><head><meta name=viewport content=\"width=device-width,initial-scale=1\"><title>pico-node OTA</title><style>body{font-family:sans-serif;max-width:34em;margin:2em auto;padding:0 1em}input,button{font-size:1em;padding:.4em;margin:.3em 0}button{padding:.6em 1.2em}#log{white-space:pre-wrap;background:#f4f4f4;padding:.6em;margin-top:1em;min-height:2em}</style></head><body><h2>pico-node firmware update</h2><p>Select the raw firmware image (<code>pico-node-app.bin</code>, NOT the .uf2) and upload. The node writes it to the spare partition, then reboots into it on trial. If the new image fails to come up, the next reboot rolls back automatically.</p><input type=file id=f accept=\".bin,application/octet-stream\"><br><button onclick=up()>Upload &amp; reboot</button><div id=log></div><script>function log(m){document.getElementById('log').textContent=m}async function up(){var f=document.getElementById('f').files[0];if(!f){log('Pick a .bin file first.');return}log('Uploading '+f.size+' bytes... do not power off.');try{var r=await fetch('/firmware',{method:'POST',body:f});log('Server: '+r.status+' '+(await r.text())+'\\nThe node is rebooting; reconnect in ~30s.')}catch(e){log('Upload sent; if the node accepted it, it is now rebooting. Reconnect in ~30s.')}}</script><p style=\"margin-top:1.5em\"><a href=/config>&#9881; Configure this node &rarr;</a></p></body></html>";

const DONE_PAGE: &str = "firmware staged; rebooting to swap. Reconnect in ~30s. If the new image misbehaves it will roll back automatically on the following reboot.";
const CONFIG_SAVED_PAGE: &str = "<!DOCTYPE html><html><head><meta name=viewport content=\"width=device-width,initial-scale=1\"><title>saved</title></head><body style=\"font-family:sans-serif;max-width:30em;margin:2em auto;padding:0 1em\"><h2>Saved &mdash; rebooting</h2><p>The node is applying the new configuration and restarting. If you changed its WiFi it will join that now (and falls back to the <code>pico-setup</code> AP if it can't); reconnect in ~20s.</p></body></html>";
