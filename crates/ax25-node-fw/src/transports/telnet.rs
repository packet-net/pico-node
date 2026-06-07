//! Capability 4 — the node command prompt over telnet over WiFi.
//!
//! Ports `Packet.Node.Core.Console.TelnetConsoleListener` + `NodeCommandService`
//! onto `embassy_net::tcp::TcpSocket`. Accept a TCP connection, write the banner
//! + prompt, then run the prompt loop: feed reads through
//! [`ax25_node_core::console::LineAssembler`] (the CR/LF/CR-NUL line discipline,
//! host-tested), parse each line, and act on it via
//! [`ax25_node_core::console::service::dispatch`] — writing the rendered
//! response bytes and re-prompting.
//!
//! All the *decisions* (what to say, when to disconnect, CR-vs-CRLF) are the
//! pure, host-tested helpers in `ax25_node_core::console`; this task is just the
//! socket I/O wrapped around them. One connection at a time (a packet node's
//! console is not a web server); further dial-ins queue at the TCP backlog.
//!
//! `Connect` is real: `C <call>` hands the target to the AXUDP session owner
//! over the [`super::relay`] statics, then this task parks its prompt loop and
//! relays raw bytes both ways (translating the AX.25 CR line convention to the
//! telnet CRLF one) until either side disconnects — the
//! `ConsoleRelay.PipeAsync` analogue. One relay at a time, per `relay`.

use ax25_node_core::console::command::parse_bytes;
use ax25_node_core::console::service::{banner_and_prompt, dispatch, Identity};
use ax25_node_core::console::{DispatchOutcome, LineAssembler, TransportKind};

use alloc::string::String;
use alloc::vec::Vec;

use embassy_futures::select::{select3, Either3};
use embassy_net::tcp::TcpSocket;
use embassy_net::Stack;
use embassy_time::Duration;

use crate::config::TelnetConfig;
use crate::transports::relay::{self, RelayStatus};
use crate::transports::tcp_write_all as write_all;

/// Idle timeout for a console connection; a dead peer frees the slot.
const IDLE_TIMEOUT_SECS: u64 = 300;

#[embassy_executor::task]
pub async fn task(stack: Stack<'static>, cfg: TelnetConfig, id: Identity, prompt: String) {
    defmt::info!("telnet: listen tcp/{}", cfg.port);

    let mut rx_buf = [0u8; 1024];
    let mut tx_buf = [0u8; 1024];

    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buf, &mut tx_buf);
        socket.set_timeout(Some(Duration::from_secs(IDLE_TIMEOUT_SECS)));

        if let Err(e) = socket.accept(cfg.port).await {
            defmt::warn!("telnet: accept error {:?}", e);
            continue;
        }
        defmt::info!("telnet: connection from {:?}", socket.remote_endpoint());

        serve(&mut socket, &id, &prompt).await;

        // Graceful FIN + drain, then hard-drop whatever's left.
        socket.close();
        let _ = socket.flush().await;
        socket.abort();
        defmt::info!("telnet: connection closed");
    }
}

/// One connection's banner + prompt loop. Returns when the peer goes away or a
/// command disconnects.
async fn serve(socket: &mut TcpSocket<'_>, id: &Identity, prompt: &str) {
    const KIND: TransportKind = TransportKind::Telnet;

    if !write_all(socket, &banner_and_prompt(id, prompt, KIND)).await {
        return;
    }

    let mut asm = LineAssembler::default();
    let mut buf = [0u8; 256];
    loop {
        let n = match socket.read(&mut buf).await {
            Ok(0) => return, // EOF — peer closed
            Ok(n) => n,
            Err(e) => {
                defmt::warn!("telnet: read error {:?}", e);
                return;
            }
        };

        for line in asm.push(&buf[..n]) {
            let cmd = parse_bytes(&line);
            let resp = dispatch(&cmd, id, KIND);
            if !write_all(socket, &resp.body).await {
                return;
            }
            match resp.outcome {
                DispatchOutcome::Continue => {}
                DispatchOutcome::Disconnect => return,
                DispatchOutcome::ConfigOp(op) => {
                    let (text, reboot) = crate::config_store::handle_op(&op);
                    let rendered = ax25_node_core::console::service::render_line(&text, KIND);
                    if !write_all(socket, &rendered).await {
                        return;
                    }
                    if reboot {
                        // Flush the farewell, then reset — never returns.
                        let _ = socket.flush().await;
                        cortex_m::peripheral::SCB::sys_reset();
                    }
                }
                DispatchOutcome::ConnectThenRelay(call) => {
                    match relay::begin(call) {
                        Ok(()) => {
                            if !relay_loop(socket).await {
                                return; // user side went away mid-relay
                            }
                            // Link over — fall through to a fresh prompt.
                        }
                        Err(()) => {
                            let msg = ax25_node_core::console::service::render_line(
                                "Busy: another connect relay is already in progress",
                                KIND,
                            );
                            if !write_all(socket, &msg).await {
                                return;
                            }
                        }
                    }
                }
            }
            if !write_all(socket, prompt.as_bytes()).await {
                return;
            }
        }
    }
}

/// Pipe bytes between the telnet socket and the active AX.25 relay until the
/// link ends. Returns `false` if the *telnet user* went away (caller drops the
/// connection); `true` when the AX.25 side ended and the prompt loop resumes.
async fn relay_loop(socket: &mut TcpSocket<'_>) -> bool {
    let mut sock_buf = [0u8; 256];
    let mut ax_buf = [0u8; 256];
    loop {
        match select3(
            relay::STATUS.wait(),
            relay::AX_TO_USER.read(&mut ax_buf),
            socket.read(&mut sock_buf),
        )
        .await
        {
            Either3::First(RelayStatus::Connected) => {
                // The peer's own banner follows over the relay; nothing to add.
                defmt::info!("telnet: relay connected");
            }
            Either3::First(RelayStatus::Failed(reason)) => {
                let mut msg = Vec::from(b"Failure: ".as_slice());
                msg.extend_from_slice(reason.as_bytes());
                msg.extend_from_slice(b"\r\n");
                return write_all(socket, &msg).await;
            }
            Either3::First(RelayStatus::Disconnected) => {
                return write_all(socket, b"*** Disconnected\r\n").await;
            }
            Either3::Second(n) => {
                // AX.25 → telnet: bare CR becomes CRLF.
                let mut out = Vec::with_capacity(n + 16);
                for &b in &ax_buf[..n] {
                    if b == b'\r' {
                        out.extend_from_slice(b"\r\n");
                    } else if b != b'\n' {
                        out.push(b);
                    }
                }
                if !write_all(socket, &out).await {
                    relay::USER_HANGUP.signal(());
                    return false;
                }
            }
            Either3::Third(Ok(0)) | Either3::Third(Err(_)) => {
                relay::USER_HANGUP.signal(());
                return false;
            }
            Either3::Third(Ok(n)) => {
                // Telnet → AX.25: CRLF (and bare LF) become the AX.25 bare CR.
                let mut out = Vec::with_capacity(n);
                for &b in &sock_buf[..n] {
                    if b == b'\n' {
                        if out.last() != Some(&b'\r') {
                            out.push(b'\r');
                        }
                    } else {
                        out.push(b);
                    }
                }
                if !out.is_empty() {
                    relay::USER_TO_AX.write_all(&out).await;
                }
            }
        }
    }
}
