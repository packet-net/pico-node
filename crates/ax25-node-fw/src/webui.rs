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

/// The CONFIGURE section: a `<form>` POSTing to `/save`, with every field
/// **pre-filled** from the current pending config. The password is deliberately
/// never echoed — the field is empty with a "leave blank = keep" hint, and the
/// server-side `apply_form` treats an empty value as "keep current".
pub fn config_form(p: &StoredConfig) -> String {
    // name, label, current value, and the per-field input attributes (the mobile
    // + identity-typing rules: autocomplete/autocorrect/spellcheck off, and
    // callsign/alias/grid auto-uppercased since those are conventionally caps).
    fn field(name: &str, label: &str, value: &str, attrs: &str) -> String {
        format!(
            "<label>{}</label><input name={} value=\"{}\" {}>",
            label,
            name,
            esc(value),
            attrs
        )
    }
    let g = "autocomplete=off autocorrect=off spellcheck=false";
    let caps = "autocapitalize=characters";
    let lower = "autocapitalize=off";

    let mut f = String::from("<section><h2>Configure</h2><form method=post action=/save autocomplete=off>");
    f += &field("callsign", "Callsign", p.callsign.as_deref().unwrap_or(""),
        &format!("placeholder=M0ABC-1 {g} {caps}"));
    f += &field("alias", "Alias", p.alias.as_deref().unwrap_or(""),
        &format!("placeholder=NODE {g} {caps}"));
    f += &field("grid", "Grid", p.grid.as_deref().unwrap_or(""),
        &format!("placeholder=IO91 {g} {caps}"));
    f += &field("wifi_ssid", "WiFi network (SSID)", p.wifi_ssid.as_deref().unwrap_or(""),
        &format!("{g} {lower}"));
    // Password: never pre-filled.
    f += &format!(
        "<label>WiFi password</label><input name=wifi_pass type=password \
placeholder=\"leave blank = keep\" {g} {lower}>"
    );
    f += &field("mqtt_host", "MQTT host (optional, host:port for logs)",
        p.mqtt_host.as_deref().unwrap_or(""),
        &format!("placeholder=10.0.0.5:1883 inputmode=url {g} {lower}"));
    f += "<button type=submit class=primary>Save &amp; reboot</button></form>";
    f += "<p class=hint>Blank fields keep their current value. Set a WiFi network to join it on the next boot.</p></section>";
    f
}

/// A standalone "saved, rebooting" confirmation page, themed to match.
pub fn notice(heading: &str, body_html: &str) -> String {
    page(
        heading,
        &format!("<h1>{}</h1><p class=hint>{}</p>", esc(heading), body_html),
    )
}
