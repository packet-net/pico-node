//! The node-console command set + total parser.
//!
//! Ports `Packet.Node.Core.Console.NodeCommand` and `NodeCommandParser`. The
//! parser is **total** (never panics) and allocation-bounded: any input maps to
//! exactly one [`Command`]; an unrecognised verb becomes [`Command::Unknown`] and
//! a recognised verb with a bad argument becomes a typed error variant
//! (e.g. [`Command::MalformedConnect`]). This is the fuzz contract.
//!
//! TNC2 conventions: verbs are case-insensitive and abbreviate to any unambiguous
//! prefix — `C`/`CONN`/`CONNECT`, `B`/`BYE`, `D`/`DISCONNECT`, `N`/`NODES`,
//! `I`/`INFO`, `H`/`HELP`/`?`.

use crate::ax25::Callsign;

use alloc::string::{String, ToString};

/// The longest input line the parser considers; longer input is truncated first
/// so a hostile peer can't drive unbounded work. Mirrors
/// `NodeCommandParser.MaxLineLength`.
pub const MAX_LINE_LEN: usize = 512;

/// A parsed console command — a closed, typed set.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Command {
    /// `C[onnect] <call>` — connect outbound and relay until either side drops.
    Connect(Callsign),
    /// `N[odes]` — list node identity + ports.
    Nodes,
    /// `I[nfo]` — node identity + version banner.
    Info,
    /// `B[ye]` / `D[isconnect]` — tear the console connection down.
    Bye,
    /// `H[elp]` / `?` — the command list.
    Help,
    /// An empty line — re-prompt without comment (not an error).
    Empty,
    /// A recognised `CONNECT` the parser could not complete. `target` is `None`
    /// when no callsign was given, or `Some(text)` carrying the offending token
    /// when it was present but not a valid callsign — so the service can render the
    /// same two distinct messages pdn does (mirrors C# `MalformedConnect`).
    MalformedConnect {
        /// `None` = no callsign given; `Some(text)` = the offending non-callsign token.
        target: Option<String>,
    },
    /// `SHOW` — display the node's effective + pending configuration. Executed
    /// by the embedder (the config lives firmware-side); the parser only
    /// recognises the verb.
    ShowConfig,
    /// `SET <KEY> <VALUE…>` — stage a configuration change (applied to the
    /// pending config; `SAVE` persists, reboot applies). Key/value are passed
    /// through verbatim; validation is the embedder's (it owns the schema).
    Set {
        /// The configuration key, upper-cased.
        key: String,
        /// The raw value text (may contain spaces, e.g. WiFi passphrases).
        value: String,
    },
    /// `SET` with no/incomplete arguments — the embedder prints usage.
    MalformedSet,
    /// `SAVE` — persist the pending configuration to flash.
    Save,
    /// `REBOOT` — restart the node (how pending config takes effect).
    Reboot,
    /// An input line that matched no known verb. Carries the trimmed raw line so
    /// the service can echo it back (`Unknown command: <raw>`), mirroring pdn's
    /// C# `UnknownCommand`.
    Unknown(String),
}

/// Parse raw line bytes (lenient UTF-8) into a [`Command`]. Used by the wire path
/// and the fuzz harness, which feed raw bytes. Total: never panics.
pub fn parse_bytes(line: &[u8]) -> Command {
    let bounded = if line.len() > MAX_LINE_LEN {
        &line[..MAX_LINE_LEN]
    } else {
        line
    };
    // Lossy decode: invalid UTF-8 becomes replacement chars, never a panic. On
    // no_std we have no String::from_utf8_lossy, so decode the validated prefix
    // and treat the rest as opaque (it can only be a verb/arg if it's ASCII text).
    match core::str::from_utf8(bounded) {
        Ok(s) => parse(s),
        Err(e) => {
            let valid = &bounded[..e.valid_up_to()];
            // Safe: valid_up_to() is a UTF-8 boundary by construction.
            parse(core::str::from_utf8(valid).unwrap_or(""))
        }
    }
}

/// Parse an already-decoded line into a typed command. Total.
pub fn parse(line: &str) -> Command {
    let line = if line.len() > MAX_LINE_LEN {
        &line[..MAX_LINE_LEN]
    } else {
        line
    };

    let trimmed = trim_control(line);
    if trimmed.is_empty() {
        return Command::Empty;
    }

    // "?" is help on its own.
    if trimmed == "?" {
        return Command::Help;
    }

    let (verb, rest) = match trimmed.find(char::is_whitespace) {
        Some(i) => (&trimmed[..i], trimmed[i + 1..].trim()),
        None => (trimmed, ""),
    };

    // Case-fold the verb to compare against canonical names. We only need ASCII
    // upper-casing (verbs are ASCII), so do it on a small stack buffer.
    let mut upper = [0u8; 16];
    let vlen = verb.len().min(upper.len());
    for (i, b) in verb.bytes().take(vlen).enumerate() {
        upper[i] = b.to_ascii_uppercase();
    }
    let upper = &upper[..vlen];

    if matches_prefix(upper, b"CONNECT") {
        return parse_connect(rest);
    }
    if matches_prefix(upper, b"BYE") || matches_prefix(upper, b"DISCONNECT") {
        return Command::Bye;
    }
    if matches_prefix(upper, b"NODES") {
        return Command::Nodes;
    }
    if matches_prefix(upper, b"INFO") {
        return Command::Info;
    }
    if matches_prefix(upper, b"HELP") {
        return Command::Help;
    }
    if matches_prefix(upper, b"SHOW") {
        return Command::ShowConfig;
    }
    // SET is exact (not a prefix): "S" alone is too easy to fat-finger for a
    // config mutation, and SAVE shares the S prefix.
    if upper == b"SET" {
        return parse_set(rest);
    }
    if upper == b"SAVE" {
        return Command::Save;
    }
    if upper == b"REBOOT" {
        return Command::Reboot;
    }

    Command::Unknown(trimmed.to_string())
}

fn parse_connect(rest: &str) -> Command {
    if rest.trim().is_empty() {
        return Command::MalformedConnect { target: None };
    }
    // First token is the target; trailing via-path/extras ignored (same-port only).
    let target = match rest.find(char::is_whitespace) {
        Some(i) => &rest[..i],
        None => rest,
    };
    match Callsign::parse(target) {
        Some(call) => Command::Connect(call),
        None => Command::MalformedConnect {
            target: Some(target.to_string()),
        },
    }
}

fn parse_set(rest: &str) -> Command {
    let rest = rest.trim();
    let (key, value) = match rest.find(char::is_whitespace) {
        Some(i) => (&rest[..i], rest[i + 1..].trim()),
        None => (rest, ""),
    };
    if key.is_empty() || value.is_empty() {
        return Command::MalformedSet;
    }
    Command::Set {
        key: key.to_ascii_uppercase(),
        value: value.to_string(),
    }
}

// An input verb matches a canonical verb if it's a non-empty case-folded prefix.
fn matches_prefix(upper_verb: &[u8], canonical: &[u8]) -> bool {
    !upper_verb.is_empty()
        && upper_verb.len() <= canonical.len()
        && canonical.starts_with(upper_verb)
}

// Trim leading/trailing control + whitespace; keep the interior intact.
fn trim_control(s: &str) -> &str {
    s.trim_matches(|c: char| c.is_control() || c.is_whitespace())
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::string::ToString;

    #[test]
    fn connect_full_and_abbrev() {
        let want = Command::Connect(Callsign::parse("M0LTE-1").unwrap());
        assert_eq!(parse("CONNECT M0LTE-1"), want);
        assert_eq!(parse("C M0LTE-1"), want);
        assert_eq!(parse("conn m0lte-1"), want);
    }

    #[test]
    fn connect_ignores_via_path() {
        assert_eq!(
            parse("C M0LTE-1 via WIDE2-2"),
            Command::Connect(Callsign::parse("M0LTE-1").unwrap())
        );
    }

    #[test]
    fn connect_missing_call_is_malformed() {
        assert_eq!(parse("C"), Command::MalformedConnect { target: None });
        assert_eq!(parse("C   "), Command::MalformedConnect { target: None });
    }

    #[test]
    fn connect_bad_call_is_malformed_and_keeps_the_offending_token() {
        assert_eq!(
            parse("C not.a.call"),
            Command::MalformedConnect { target: Some("not.a.call".to_string()) }
        );
        assert_eq!(
            parse("C TOOLONGG"),
            Command::MalformedConnect { target: Some("TOOLONGG".to_string()) }
        );
    }

    #[test]
    fn bye_and_disconnect() {
        assert_eq!(parse("B"), Command::Bye);
        assert_eq!(parse("BYE"), Command::Bye);
        assert_eq!(parse("D"), Command::Bye);
        assert_eq!(parse("DISCONNECT"), Command::Bye);
    }

    #[test]
    fn nodes_info_help() {
        assert_eq!(parse("N"), Command::Nodes);
        assert_eq!(parse("I"), Command::Info);
        assert_eq!(parse("H"), Command::Help);
        assert_eq!(parse("?"), Command::Help);
        assert_eq!(parse("HELP"), Command::Help);
    }

    #[test]
    fn empty_and_whitespace() {
        assert_eq!(parse(""), Command::Empty);
        assert_eq!(parse("   \t  "), Command::Empty);
        assert_eq!(parse("\r\n"), Command::Empty);
    }

    #[test]
    fn unknown_verb_keeps_the_raw_line_for_echo() {
        assert_eq!(parse("FROBNICATE"), Command::Unknown("FROBNICATE".to_string()));
        assert_eq!(parse("xyzzy foo"), Command::Unknown("xyzzy foo".to_string()));
    }

    #[test]
    fn parse_bytes_is_total_on_garbage() {
        // Invalid UTF-8 + control bytes: must not panic, must classify.
        assert_eq!(parse_bytes(&[0xFF, 0xFE, 0x00]), Command::Empty);
        assert_eq!(
            parse_bytes(b"C M0LTE\xFF\xFE"),
            Command::Connect(Callsign::parse("M0LTE").unwrap())
        );
    }

    #[test]
    fn parse_bytes_truncates_overlong() {
        let mut huge = alloc::vec![b'A'; MAX_LINE_LEN * 4];
        huge[0] = b'C';
        huge[1] = b' ';
        // Bytes 2.. are 'A's; the first token after "C " is a long run of 'A's,
        // which is not a valid callsign => malformed, not a panic or hang.
        assert!(matches!(parse_bytes(&huge), Command::MalformedConnect { .. }));
    }
}

#[cfg(test)]
mod config_command_tests {
    use super::*;
    use alloc::string::ToString;

    #[test]
    fn show_parses_with_prefix_matching() {
        assert_eq!(parse("SHOW"), Command::ShowConfig);
        assert_eq!(parse("show config"), Command::ShowConfig);
        assert_eq!(parse("sh"), Command::ShowConfig);
    }

    #[test]
    fn set_requires_exact_verb_key_and_value() {
        assert_eq!(
            parse("SET ALIAS HILL1"),
            Command::Set {
                key: "ALIAS".to_string(),
                value: "HILL1".to_string()
            }
        );
        // Keys upper-case; values keep their case and interior spaces.
        assert_eq!(
            parse("set wifi_pass correct horse battery"),
            Command::Set {
                key: "WIFI_PASS".to_string(),
                value: "correct horse battery".to_string()
            }
        );
        assert_eq!(parse("SET"), Command::MalformedSet);
        assert_eq!(parse("SET ALIAS"), Command::MalformedSet);
        // "SE" is NOT a SET prefix (mutations don't prefix-match)…
        assert_eq!(parse("SE ALIAS X"), Command::Unknown("SE ALIAS X".to_string()));
    }

    #[test]
    fn save_and_reboot_are_exact_words() {
        assert_eq!(parse("SAVE"), Command::Save);
        assert_eq!(parse("save"), Command::Save);
        assert_eq!(parse("REBOOT"), Command::Reboot);
    }
}
