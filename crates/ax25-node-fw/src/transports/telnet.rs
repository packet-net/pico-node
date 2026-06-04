//! Capability 4 — the node command prompt over telnet over WiFi.
//!
//! Ports `Packet.Node.Core.Console.TelnetConsoleListener` + `NodeCommandService`
//! onto `embassy_net::tcp::TcpSocket`. Accept a TCP connection, wrap it as a
//! [`ax25_node_core::console::NodeConnection`] (telnet variant), then run the
//! prompt loop: feed reads through [`ax25_node_core::console::LineAssembler`],
//! parse each line with [`ax25_node_core::console::parse`], and act on it via
//! [`ax25_node_core::console::service::dispatch`] — writing the rendered response
//! bytes and re-prompting. `Connect` triggers an outbound AX.25 session + a
//! byte-relay (the `ConsoleRelay.PipeAsync` analogue).
//!
//! All the *decisions* (what to say, when to disconnect, CR-vs-CRLF) are the pure,
//! host-tested helpers in `ax25_node_core::console`; this task is just the socket
//! I/O wrapped around them. STUB: socket accept + loop body to be written.

use ax25_node_core::console::{self, LineAssembler, TransportKind};
use embassy_net::Stack;

use crate::config::TelnetConfig;

#[embassy_executor::task]
pub async fn task(_stack: Stack<'static>, cfg: TelnetConfig) {
    defmt::info!("telnet: listen tcp/{}", cfg.port);
    // loop {
    //   accept a TcpSocket on cfg.port;
    //   write console::service::banner_and_prompt(&id, &prompt, Telnet);
    //   let mut asm = LineAssembler::default();
    //   loop {
    //     let n = socket.read(&mut buf).await?; if n == 0 { break }
    //     for line in asm.push(&buf[..n]) {
    //       let cmd = console::parse(core::str::from_utf8(&line).unwrap_or(""));
    //       let resp = console::service::dispatch(&cmd, &id, Telnet);
    //       socket.write_all(&resp.body).await?;
    //       match resp.outcome {
    //         Disconnect => return,
    //         ConnectThenRelay(call) => { /* outbound connect + relay */ }
    //         Continue => socket.write_all(&prompt).await?,
    //   } } }
    // }
    let _ = (LineAssembler::default(), console::parse as fn(&str) -> _, TransportKind::Telnet);
    unimplemented!("telnet console accept + prompt loop")
}
