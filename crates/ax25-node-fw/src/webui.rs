//! Shared on-board web UI — the look + the reused fragments for the node panel.
//!
//! One place owns the stylesheet and the pre-filled config form so the STA-mode
//! node panel (`crate::ota`) and the AP-mode captive portal
//! (`crate::provisioning`) render identically. Everything is built on the heap
//! with `format!` (alloc is available); pages are small (a few KiB) and sent as
//! one `text/html` body.
//!
//! Aesthetic: a restrained "lab-instrument" skin — warm off-white, ink text, a
//! single calm teal accent, hairline rules between sections, and a monospace
//! face for machine values (callsign, IP, version). Self-contained: no external
//! fonts or scripts (the node has no internet egress). Mobile rules are
//! deliberate — `box-sizing:border-box`, `overflow-x:hidden`, and **16px inputs**
//! (anything smaller triggers iOS auto-zoom-on-focus, which causes side-scroll).

use alloc::format;
use alloc::string::String;

use crate::config_store::StoredConfig;

/// The whole stylesheet, inlined into every page. Kept to one tight block — it
/// ships in flash and is sent on every request.
pub const CSS: &str = "\
:root{--bg:#faf8f4;--ink:#1a1a1a;--mut:#6b6b6b;--line:#e3ded5;--accent:#0f766e;--accentd:#0b5a54}\
*{box-sizing:border-box}html{overflow-x:hidden}\
body{background:var(--bg);color:var(--ink);font-family:system-ui,-apple-system,'Segoe UI',Roboto,sans-serif;max-width:30em;margin:0 auto;padding:1.3em 1em 3em;line-height:1.45}\
.mono{font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}\
h1{font-size:1.25em;margin:0;font-weight:600;letter-spacing:.01em}\
.sub{color:var(--mut);font-size:.85em;margin:.2em 0 0}\
.dot{color:var(--accent)}\
.spread{display:flex;justify-content:space-between;align-items:baseline;gap:1em}\
section{border-top:1px solid var(--line);margin-top:1.5em;padding-top:1.1em}\
h2{font-size:.72em;font-weight:700;text-transform:uppercase;letter-spacing:.13em;color:var(--mut);margin:0 0 .7em}\
label{display:block;font-size:.8em;color:var(--mut);margin:.7em 0 .25em}\
input{width:100%;padding:.55em .6em;font-size:16px;border:1px solid var(--line);border-radius:6px;background:#fff;color:var(--ink)}\
input:focus{outline:none;border-color:var(--accent);box-shadow:0 0 0 3px rgba(15,118,110,.15)}\
button{font-size:16px;border-radius:6px;cursor:pointer}\
.primary{width:100%;margin-top:1.3em;padding:.7em;background:var(--accent);color:#fff;border:1px solid var(--accentd);font-weight:600}\
.primary:active{background:var(--accentd)}\
.ghost{padding:.55em .95em;background:#fff;color:var(--accent);border:1px solid var(--accent)}\
.row{display:flex;gap:.5em;align-items:center;flex-wrap:wrap}\
.hint{color:var(--mut);font-size:.82em;margin:.6em 0 0}\
code{font-family:ui-monospace,Menlo,Consolas,monospace;font-size:.92em;background:#fff;border:1px solid var(--line);border-radius:4px;padding:0 .3em}\
#log{white-space:pre-wrap;background:#fff;border:1px solid var(--line);border-radius:6px;padding:.6em;margin-top:.8em;min-height:1.4em;font-size:.85em}";

/// HTML-escape a value for safe interpolation into text or a double-quoted
/// attribute. Conservative (covers `& < > "`); inputs here are short config
/// strings, not documents.
pub fn esc(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for c in s.chars() {
        match c {
            '&' => out.push_str("&amp;"),
            '<' => out.push_str("&lt;"),
            '>' => out.push_str("&gt;"),
            '"' => out.push_str("&quot;"),
            _ => out.push(c),
        }
    }
    out
}

/// Wrap a body fragment in the full page shell (doctype + head + inlined CSS).
pub fn page(title: &str, body: &str) -> String {
    format!(
        "<!DOCTYPE html><html lang=en><head><meta charset=utf-8>\
<meta name=viewport content=\"width=device-width,initial-scale=1\">\
<title>{}</title><style>{}</style></head><body>{}</body></html>",
        esc(title),
        CSS,
        body
    )
}

// Shared input-typing attrs: mobile rules (autocomplete/autocorrect/spellcheck
// off — callsign/SSID/MQTT aren't dictionary words) + auto-uppercase for the
// identity fields, which are conventionally upper case.
const GEN_ATTRS: &str = "autocomplete=off autocorrect=off spellcheck=false";
const CAPS_ATTR: &str = "autocapitalize=characters";
const LOWER_ATTR: &str = "autocapitalize=off";

/// One `<label> + <input>` row. `value` is pre-filled (HTML-escaped); pass `""`
/// to leave it blank.
fn field(name: &str, label: &str, value: &str, attrs: &str) -> String {
    format!(
        "<label>{}</label><input name={} value=\"{}\" {}>",
        label,
        name,
        esc(value),
        attrs
    )
}

/// The three identity inputs (callsign / alias / grid), pre-filled — shared by
/// the STA Configure form and the AP identity form. Validation + lengths are
/// enforced server-side in `config_store::set_field`; `maxlength` here is a UX
/// nicety only.
pub fn identity_inputs(p: &StoredConfig) -> String {
    let mut f = field("callsign", "Callsign", p.callsign.as_deref().unwrap_or(""),
        &format!("placeholder=M0ABC-1 maxlength=9 {GEN_ATTRS} {CAPS_ATTR}"));
    f += &field("alias", "Alias", p.alias.as_deref().unwrap_or(""),
        &format!("placeholder=NODE maxlength=6 {GEN_ATTRS} {CAPS_ATTR}"));
    f += &field("grid", "Grid", p.grid.as_deref().unwrap_or(""),
        &format!("placeholder=IO91 {GEN_ATTRS} {CAPS_ATTR}"));
    f
}

/// STA-mode CONFIGURE section: identity + WiFi (pre-filled) + MQTT, POSTing to
/// `/save`. The password is never echoed (empty, "leave blank = keep"; the
/// server treats blank as keep-current).
pub fn sta_config_form(p: &StoredConfig) -> String {
    let mut f = String::from("<section><h2>Configure</h2><form method=post action=/save autocomplete=off>");
    f += &identity_inputs(p);
    f += &field("wifi_ssid", "WiFi network (SSID)", p.wifi_ssid.as_deref().unwrap_or(""),
        &format!("{GEN_ATTRS} {LOWER_ATTR}"));
    f += &format!(
        "<label>WiFi password</label><input name=wifi_pass type=password \
placeholder=\"leave blank = keep\" {GEN_ATTRS} {LOWER_ATTR}>"
    );
    f += &field("mqtt_host", "MQTT host (optional, host:port for logs)",
        p.mqtt_host.as_deref().unwrap_or(""),
        &format!("placeholder=10.0.0.5:1883 inputmode=url {GEN_ATTRS} {LOWER_ATTR}"));
    f += "<button type=submit class=primary>Save &amp; reboot</button></form>";
    f += "<p class=hint>Blank fields keep their current value.</p></section>";
    f
}

/// AP-mode identity section: callsign/alias/grid only, POSTing to `/save`.
/// Saving here keeps the node in AP mode (no WiFi, no MQTT — those are LAN
/// concerns set from the STA panel).
pub fn ap_identity_section(p: &StoredConfig) -> String {
    let mut f = String::from("<section><h2>Node identity</h2><form method=post action=/save autocomplete=off>");
    f += &identity_inputs(p);
    f += "<button type=submit class=primary>Save &amp; reboot</button></form>";
    f += "<p class=hint>Saving keeps this node in AP mode. Changing the callsign \
changes the AP name (you'll reconnect to it).</p></section>";
    f
}

/// AP-mode "Join a WiFi network" section: a *conscious* path back to LAN mode.
/// Fields are deliberately **empty** (not pre-filled with any stored network) —
/// joining is an explicit action, not a side-effect. POSTs to `/join`, which
/// clears the sticky AP flag and reboots into STA.
pub fn ap_join_section() -> String {
    format!(
        "<section><h2>Join a Wi-Fi network</h2>\
<form method=post action=/join autocomplete=off>\
<label>Network (SSID)</label><input name=wifi_ssid placeholder=\"network name\" {GEN_ATTRS} {LOWER_ATTR}>\
<label>Password</label><input name=wifi_pass type=password {GEN_ATTRS} {LOWER_ATTR}>\
<button type=submit class=primary>Join network &amp; switch to LAN</button></form>\
<p class=hint>Connects this node to infrastructure Wi-Fi and leaves AP mode. This \
access point will disappear; the node returns here only if the network can't be \
joined.</p></section>"
    )
}

/// A standalone "saved, rebooting" confirmation page, themed to match.
pub fn notice(heading: &str, body_html: &str) -> String {
    page(
        heading,
        &format!("<h1>{}</h1><p class=hint>{}</p>", esc(heading), body_html),
    )
}

/// Like [`notice`], but auto-returns to the panel once the node is back: a
/// client-side poller waits for the reboot, polls `/version`, and redirects to
/// `/` on the first success (with a manual link as fallback). Runs entirely in
/// the browser — no device-side cost.
///
/// **Use only where the client stays on the same network as the node** — i.e.
/// the STA-mode panel's `POST /save`, where it returns at the same address. NOT
/// for the AP captive portal or the switch-to-AP action, where the node moves to
/// a different network and `/` would be unreachable. The poll starts after an
/// 8 s delay so it doesn't catch the node in the ~0.8 s window it's still up
/// before resetting (which would redirect prematurely).
pub fn notice_reconnect(heading: &str, body_html: &str) -> String {
    page(
        heading,
        &format!(
            "<h1>{}</h1><p class=hint>{}</p>\
<p class=hint id=w>Waiting for the node to come back…</p>\
<p class=hint><a href=/>Return to the panel &rarr;</a></p>\
<script>setTimeout(function p(){{fetch('/version',{{cache:'no-store'}})\
.then(function(r){{r.ok?location.assign('/'):setTimeout(p,2000)}})\
.catch(function(){{setTimeout(p,2000)}})}},8000)</script>",
            esc(heading),
            body_html
        ),
    )
}
