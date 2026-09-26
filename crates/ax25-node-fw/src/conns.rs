//! Cross-task snapshot of the node's live connections, for the web panel's
//! "Connections" pane (`GET /conns`).
//!
//! The AX.25 sessions and NET/ROM circuits live in the node task
//! ([`crate::node`]); the web server runs in its own tasks. The node task
//! republishes this snapshot every time it wakes ([`set`]), and the web server
//! renders it as JSON on request ([`render_json`]). Same pattern as
//! [`crate::netrom_view`].

use core::cell::RefCell;
use core::fmt::Write;

use ax25_node_core::ax25::Callsign;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;

use crate::ports::call_str;
use crate::tnc::BufWriter;

/// Room for every AX.25 session plus every attached NET/ROM circuit.
pub const MAX_CONNS: usize = crate::session::MAX_SESSIONS + 8;

/// What a connection is for.
#[derive(Clone, Copy)]
pub enum Kind {
    /// A user on the air at the node prompt.
    Console,
    /// One side of an onward connect through the node.
    Onward,
    /// The telnet console's onward connect.
    Telnet,
    /// A persistent link to a NET/ROM neighbour.
    Interlink,
    /// An AX.25 link with nothing attached yet (connecting, or its user left).
    Plain,
}

impl Kind {
    fn label(self) -> &'static str {
        match self {
            Kind::Console => "user at node prompt",
            Kind::Onward => "onward connect",
            Kind::Telnet => "telnet onward connect",
            Kind::Interlink => "NET/ROM interlink",
            Kind::Plain => "link",
        }
    }
}

/// Where a connection is in its life.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Phase {
    Connecting,
    Up,
    /// Up, but waiting on the far end to answer a poll (AX.25 timer recovery).
    Retrying,
    Closing,
}

impl Phase {
    fn label(self) -> &'static str {
        match self {
            Phase::Connecting => "connecting",
            Phase::Up => "up",
            Phase::Retrying => "retrying",
            Phase::Closing => "closing",
        }
    }
}

/// One row of the pane.
#[derive(Clone, Copy)]
pub struct Conn {
    pub src: Callsign,
    pub dst: Callsign,
    /// For a NET/ROM circuit, the far node the user came in through or was
    /// connected on to (when it differs from `src`/`dst`).
    pub via: Option<Callsign>,
    /// `true` for a NET/ROM circuit, `false` for an AX.25 link.
    pub circuit: bool,
    pub kind: Kind,
    /// The radio port an AX.25 link runs on.
    pub port: &'static str,
    /// AX.25 v2.2 (modulo 128) rather than v2.0 (modulo 8).
    pub extended: bool,
    pub phase: Phase,
    /// When the link came up (node uptime ms), once it has.
    pub up_at_ms: Option<u64>,
    /// AX.25 only: frames sent on the link, and how many times the link had to
    /// retry (T1 ran out with no answer) since it was set up.
    pub frames: u32,
    pub retries: u32,
    /// AX.25 only: the current retry count and the give-up limit (N2).
    pub rc: u32,
    pub n2: u32,
}

static CONNS: Mutex<CriticalSectionRawMutex, RefCell<heapless::Vec<Conn, MAX_CONNS>>> =
    Mutex::new(RefCell::new(heapless::Vec::new()));

/// Replace the snapshot. Called by the node task.
pub fn set(rows: heapless::Vec<Conn, MAX_CONNS>) {
    CONNS.lock(|c| *c.borrow_mut() = rows);
}

/// Render the snapshot as JSON into `out`, returning the length:
/// `{"now":ms,"conns":[{...},...]}`. Times are node uptime in ms, so the page
/// counts "up for" itself between polls.
pub fn render_json(out: &mut [u8]) -> usize {
    let rows = CONNS.lock(|c| c.borrow().clone());
    let mut w = BufWriter::new(out);
    let now = crate::tnc::now_ms();
    let _ = write!(w, "{{\"now\":{now},\"conns\":[");
    for (i, c) in rows.iter().enumerate() {
        let mark = w.len();
        if write_row(&mut w, c, i == 0).is_err() || w.remaining() < 4 {
            w.truncate(mark);
            break;
        }
    }
    let _ = w.write_str("]}");
    w.len()
}

fn write_row(w: &mut BufWriter<'_>, c: &Conn, first: bool) -> core::fmt::Result {
    let mut a = [0u8; 16];
    let mut b = [0u8; 16];
    let mut v = [0u8; 16];
    if !first {
        w.write_char(',')?;
    }
    write!(
        w,
        "{{\"src\":\"{}\",\"dst\":\"{}\",\"via\":\"{}\",\"nr\":{},\"kind\":\"{}\",\"port\":\"{}\",\
\"v22\":{},\"phase\":\"{}\",\"up\":",
        call_str(&c.src, &mut a),
        call_str(&c.dst, &mut b),
        c.via.as_ref().map(|x| call_str(x, &mut v)).unwrap_or(""),
        c.circuit,
        c.kind.label(),
        c.port,
        c.extended,
        c.phase.label(),
    )?;
    match c.up_at_ms {
        Some(t) => write!(w, "{t}")?,
        None => w.write_str("null")?,
    }
    write!(
        w,
        ",\"frames\":{},\"retries\":{},\"rc\":{},\"n2\":{}}}",
        c.frames, c.retries, c.rc, c.n2
    )
}
