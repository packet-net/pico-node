//! Pure prompt-loop responses. Ports the decision + text-building half of
//! `Packet.Node.Core.Console.NodeCommandService` (the I/O orchestration —
//! `RunAsync` racing read vs. completion — lives in the firmware, over the
//! [`super::NodeConnection`] trait).
//!
//! Splitting it this way means the *behaviour* (what the node says to each
//! command, when it disconnects, the banner/help/info text, the CR-vs-CRLF
//! newline policy) is plain, deterministic, host-tested code; the firmware just
//! wires reads/writes around [`dispatch`].

use super::command::Command;
use super::connection::TransportKind;
use crate::VERSION;
use alloc::string::String;
use alloc::vec::Vec;

/// Node identity for banner/info/nodes text. Mirrors the bits of
/// `NodeConsoleEnvironment` the text builders read.
#[derive(Debug, Clone)]
pub struct Identity {
    /// Node name / alias (e.g. `"LONDON"`).
    pub node_name: String,
    /// Station callsign text (e.g. `"M0LTE-1"`).
    pub callsign: String,
    /// Optional Maidenhead grid.
    pub grid: Option<String>,
    /// Port descriptions for the `Nodes` command, e.g. `["axudp [up] udp/0.0.0.0:10093"]`.
    pub ports: Vec<String>,
}

/// What the loop should do after handling a command.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DispatchOutcome {
    /// Stay connected; write [`Response::body`] then re-prompt.
    Continue,
    /// Write [`Response::body`] then tear the connection down (no re-prompt).
    Disconnect,
    /// The `Connect` command — the firmware must perform the outbound connect +
    /// relay (it's I/O), then resume the loop. [`Response::body`] is the
    /// "Connecting to …" line to write first.
    ConnectThenRelay(crate::ax25::Callsign),
    /// A configuration operation (`SHOW`/`SET`/`SAVE`/`REBOOT`) — executed by
    /// the embedder, which owns the config schema, the flash store and the
    /// reset vector. [`Response::body`] is empty; the embedder writes its own
    /// rendered output (then re-prompts, except after a reboot).
    ConfigOp(ConfigOp),
}

/// The configuration operations the console can request of the embedder.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ConfigOp {
    /// Display effective + pending configuration.
    Show,
    /// Stage `key = value` into the pending configuration.
    Set {
        /// Upper-cased key.
        key: String,
        /// Raw value text.
        value: String,
    },
    /// Persist the pending configuration to flash.
    Save,
    /// Restart the node.
    Reboot,
}

/// The result of dispatching one command: bytes to write (already newline-
/// rendered for the transport) and the follow-up action.
#[derive(Debug, Clone)]
pub struct Response {
    /// Bytes to write to the connection (may be empty, e.g. for `Empty`).
    pub body: Vec<u8>,
    /// What to do next.
    pub outcome: DispatchOutcome,
}

/// Render free-form text (with `\n` separators) into wire bytes terminated by the
/// transport's newline, converting interior `\n` to that newline too. Mirrors
/// `NodeCommandService.WriteLineAsync`.
pub fn render_line(text: &str, kind: TransportKind) -> Vec<u8> {
    let nl = kind.newline();
    let mut out = Vec::with_capacity(text.len() + 2);
    let mut chars = text.as_bytes().iter().peekable();
    while let Some(&b) = chars.next() {
        if b == b'\r' {
            // Drop a CR that precedes an LF (normalise CRLF in source text), else
            // treat a lone CR as a newline.
            if chars.peek() == Some(&&b'\n') {
                continue;
            }
            out.extend_from_slice(nl);
        } else if b == b'\n' {
            out.extend_from_slice(nl);
        } else {
            out.push(b);
        }
    }
    out.extend_from_slice(nl);
    out
}

/// The banner + first prompt as one buffer (one I-frame on the air — see #292 in
/// the C# host). `prompt` is the already-expanded prompt string.
pub fn banner_and_prompt(id: &Identity, prompt: &str, kind: TransportKind) -> Vec<u8> {
    // "Welcome to {node} ({call})  [pico-node <ver>]" — the same banner shape pdn
    // uses (its configurable default is "Welcome to {node} ({call})"); the software
    // tag differs by design (pico-node vs Packet.NET). Keeps the node-prompt
    // experience aligned across the two node implementations.
    let mut banner = String::from("Welcome to ");
    banner.push_str(&id.node_name);
    banner.push_str(" (");
    banner.push_str(&id.callsign);
    banner.push(')');
    banner.push_str("  [pico-node ");
    banner.push_str(VERSION);
    banner.push(']');
    let nl = kind.newline();
    let mut out = render_line_no_trailing(&banner, nl);
    out.extend_from_slice(nl);
    out.extend_from_slice(prompt.as_bytes());
    out
}

fn render_line_no_trailing(text: &str, nl: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(text.len());
    for &b in text.as_bytes() {
        if b == b'\n' {
            out.extend_from_slice(nl);
        } else if b != b'\r' {
            out.push(b);
        }
    }
    out
}

/// Dispatch one parsed command to a [`Response`], given the node identity and the
/// transport (for newline rendering). Pure: mirrors
/// `NodeCommandService.DispatchAsync` + its text builders.
pub fn dispatch(cmd: &Command, id: &Identity, kind: TransportKind) -> Response {
    match cmd {
        Command::Empty => Response {
            body: Vec::new(),
            outcome: DispatchOutcome::Continue,
        },
        Command::Help => line(help_text(), kind),
        Command::Info => line(&info_text(id), kind),
        Command::Nodes => line(&nodes_text(id), kind),
        Command::Bye => Response {
            body: render_line("73", kind),
            outcome: DispatchOutcome::Disconnect,
        },
        Command::Connect(call) => {
            let mut dst = [0u8; 16];
            let n = call.write_display(&mut dst).unwrap_or(0);
            let target = core::str::from_utf8(&dst[..n]).unwrap_or("?");
            let mut s = String::from("Connecting to ");
            s.push_str(target);
            s.push_str("...");
            Response {
                body: render_line(&s, kind),
                outcome: DispatchOutcome::ConnectThenRelay(*call),
            }
        }
        Command::MalformedConnect { target } => match target {
            None => line("Connect needs a callsign, e.g. C M0LTE-1", kind),
            Some(t) => {
                let mut s = String::from("'");
                s.push_str(t);
                s.push_str("' is not a valid callsign (1-6 letters/digits, optional -SSID 0-15).");
                line(&s, kind)
            }
        },
        Command::ShowConfig => config_op(ConfigOp::Show),
        Command::Set { key, value } => config_op(ConfigOp::Set {
            key: key.clone(),
            value: value.clone(),
        }),
        Command::MalformedSet => line("Usage: SET <KEY> <VALUE>  (SHOW lists keys)", kind),
        Command::Save => config_op(ConfigOp::Save),
        Command::Reboot => config_op(ConfigOp::Reboot),
        Command::Unknown(raw) => {
            // Echo the offending line back (sanitised — strip control chars so a
            // hostile line can't inject terminal escapes / extra newlines), mirroring
            // pdn's `Unknown command: <raw>  (type H for help)`.
            let mut s = String::from("Unknown command: ");
            s.push_str(&sanitise(raw));
            s.push_str("  (type H for help)");
            line(&s, kind)
        }
    }
}

// Replace control characters with '.' so an echoed unknown command can't inject
// terminal escapes or extra newlines into our reply. Mirrors C# `Sanitise`.
fn sanitise(raw: &str) -> String {
    raw.chars()
        .map(|c| if c.is_control() { '.' } else { c })
        .collect()
}

fn config_op(op: ConfigOp) -> Response {
    Response {
        body: Vec::new(),
        outcome: DispatchOutcome::ConfigOp(op),
    }
}

fn line(text: &str, kind: TransportKind) -> Response {
    Response {
        body: render_line(text, kind),
        outcome: DispatchOutcome::Continue,
    }
}

fn help_text() -> &'static str {
    "Commands:\n\
     \x20 C[onnect] <call>   connect to a station\n\
     \x20 N[odes]            list this node and its ports\n\
     \x20 I[nfo]             node info and version\n\
     \x20 B[ye] / D          disconnect\n\
     \x20 H[elp] / ?         this help\n\
     \x20 SHOW               show configuration\n\
     \x20 SET <key> <value>  stage a config change\n\
     \x20 SAVE               persist staged config\n\
     \x20 REBOOT             restart (applies saved config)"
}

fn info_text(id: &Identity) -> String {
    let mut s = String::new();
    s.push_str("Node: ");
    s.push_str(&id.node_name);
    s.push_str(" (");
    s.push_str(&id.callsign);
    s.push(')');
    if let Some(grid) = &id.grid {
        if !grid.is_empty() {
            s.push_str("  Grid: ");
            s.push_str(grid);
        }
    }
    s.push('\n');
    s.push_str("Software: pico-node ");
    s.push_str(VERSION);
    s
}

fn nodes_text(id: &Identity) -> String {
    let mut s = String::new();
    s.push_str("Node ");
    s.push_str(&id.node_name);
    s.push_str(" (");
    s.push_str(&id.callsign);
    s.push(')');
    if id.ports.is_empty() {
        s.push('\n');
        s.push_str("Ports: (none configured)");
        return s;
    }
    s.push('\n');
    s.push_str("Ports:");
    for p in &id.ports {
        s.push('\n');
        s.push_str("  ");
        s.push_str(p);
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ax25::Callsign;

    fn id() -> Identity {
        Identity {
            node_name: String::from("LONDON"),
            callsign: String::from("M0LTE-1"),
            grid: Some(String::from("IO91wm")),
            ports: alloc::vec![String::from("axudp [up] udp/0.0.0.0:10093")],
        }
    }

    #[test]
    fn telnet_uses_crlf() {
        let out = render_line("hi", TransportKind::Telnet);
        assert_eq!(out, b"hi\r\n");
    }

    #[test]
    fn ax25_uses_bare_cr() {
        let out = render_line("hi", TransportKind::Ax25);
        assert_eq!(out, b"hi\r");
    }

    #[test]
    fn interior_newlines_become_transport_newline() {
        let out = render_line("a\nb", TransportKind::Telnet);
        assert_eq!(out, b"a\r\nb\r\n");
    }

    #[test]
    fn crlf_in_source_is_normalised() {
        let out = render_line("a\r\nb", TransportKind::Ax25);
        assert_eq!(out, b"a\rb\r");
    }

    #[test]
    fn bye_disconnects_with_73() {
        let r = dispatch(&Command::Bye, &id(), TransportKind::Telnet);
        assert_eq!(r.outcome, DispatchOutcome::Disconnect);
        assert_eq!(r.body, b"73\r\n");
    }

    #[test]
    fn empty_writes_nothing_and_continues() {
        let r = dispatch(&Command::Empty, &id(), TransportKind::Telnet);
        assert!(r.body.is_empty());
        assert_eq!(r.outcome, DispatchOutcome::Continue);
    }

    #[test]
    fn connect_signals_relay_and_announces() {
        let call = Callsign::parse("G7XYZ-2").unwrap();
        let r = dispatch(&Command::Connect(call), &id(), TransportKind::Ax25);
        assert_eq!(r.outcome, DispatchOutcome::ConnectThenRelay(call));
        assert_eq!(r.body, b"Connecting to G7XYZ-2...\r");
    }

    #[test]
    fn info_contains_call_grid_version() {
        let r = dispatch(&Command::Info, &id(), TransportKind::Telnet);
        let txt = String::from_utf8(r.body).unwrap();
        assert!(txt.contains("M0LTE-1"));
        assert!(txt.contains("IO91wm"));
        assert!(txt.contains("pico-node"));
        assert_eq!(r.outcome, DispatchOutcome::Continue);
    }

    #[test]
    fn nodes_lists_ports() {
        let r = dispatch(&Command::Nodes, &id(), TransportKind::Telnet);
        let txt = String::from_utf8(r.body).unwrap();
        assert!(txt.contains("LONDON"));
        assert!(txt.contains("axudp [up]"));
    }

    #[test]
    fn nodes_handles_no_ports() {
        let mut i = id();
        i.ports.clear();
        let r = dispatch(&Command::Nodes, &i, TransportKind::Telnet);
        let txt = String::from_utf8(r.body).unwrap();
        assert!(txt.contains("(none configured)"));
    }

    #[test]
    fn help_lists_commands() {
        let r = dispatch(&Command::Help, &id(), TransportKind::Telnet);
        let txt = String::from_utf8(r.body).unwrap();
        assert!(txt.contains("C[onnect]"));
        assert!(txt.contains("H[elp]"));
    }

    #[test]
    fn unknown_echoes_the_raw_line_and_continues() {
        let r = dispatch(&Command::Unknown(String::from("frobnicate")), &id(), TransportKind::Telnet);
        let txt = String::from_utf8(r.body).unwrap();
        assert!(txt.contains("Unknown command: frobnicate"), "echoes the offending line (pdn parity)");
        assert!(txt.contains("(type H for help)"));
        assert_eq!(r.outcome, DispatchOutcome::Continue);
    }

    #[test]
    fn unknown_echo_is_sanitised() {
        // Control chars in the echoed line become '.' so a hostile line can't inject
        // terminal escapes / newlines (pdn's Sanitise).
        let r = dispatch(&Command::Unknown(String::from("ev\u{1b}il")), &id(), TransportKind::Telnet);
        let txt = String::from_utf8(r.body).unwrap();
        assert!(txt.contains("ev.il"));
        assert!(!txt.contains('\u{1b}'));
    }

    #[test]
    fn malformed_connect_messages_match_pdn() {
        let none = dispatch(&Command::MalformedConnect { target: None }, &id(), TransportKind::Telnet);
        assert!(String::from_utf8(none.body).unwrap().contains("Connect needs a callsign"));

        let bad = dispatch(
            &Command::MalformedConnect { target: Some(String::from("not.a.call")) },
            &id(),
            TransportKind::Telnet,
        );
        let t = String::from_utf8(bad.body).unwrap();
        assert!(t.contains("not.a.call"), "echoes the offending token");
        assert!(t.contains("not a valid callsign"));
    }

    #[test]
    fn banner_is_welcome_node_call_version() {
        let out = banner_and_prompt(&id(), "M0LTE-1> ", TransportKind::Telnet);
        let txt = String::from_utf8(out).unwrap();
        assert!(txt.contains("Welcome to LONDON (M0LTE-1)"), "the pdn-aligned welcome banner");
        assert!(txt.contains("pico-node"));
        assert!(txt.ends_with("M0LTE-1> "));
    }
}
