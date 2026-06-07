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
//! Gate 4 scope: `Connect` answers its "Connecting to …" line and then explains
//! the session layer isn't wired yet — the outbound connect + relay is the
//! session-supervisor seam (HW-BRINGUP Gate 3 stretch+).

use ax25_node_core::console::command::parse_bytes;
use ax25_node_core::console::service::{banner_and_prompt, dispatch, Identity};
use ax25_node_core::console::{DispatchOutcome, LineAssembler, TransportKind};

use alloc::string::String;

use embassy_net::tcp::TcpSocket;
use embassy_net::Stack;
use embassy_time::Duration;

use crate::config::TelnetConfig;
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
                DispatchOutcome::ConnectThenRelay(_call) => {
                    // The outbound AX.25 connect + byte relay is the session-
                    // supervisor seam (Gate 3 stretch+). Be honest meanwhile.
                    let msg = ax25_node_core::console::service::render_line(
                        "...connected-mode sessions aren't wired to the console yet (bring-up)",
                        KIND,
                    );
                    if !write_all(socket, &msg).await {
                        return;
                    }
                }
            }
            if !write_all(socket, prompt.as_bytes()).await {
                return;
            }
        }
    }
}
