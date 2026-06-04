//! Capability 3 — KISS-over-serial to a NinoTNC.
//!
//! Ports `Packet.Kiss.Serial.KissSerialModem` onto an `embassy_rp` UART. Same KISS
//! codec as the TCP path ([`ax25_node_core::kiss`]) — only the byte source differs.
//!
//! ## Hardware note (research §5.3 / capability 3 caveat)
//!
//! The RP2040 cannot be a USB *host* and a USB-serial device simultaneously, so we
//! do NOT try to talk to the NinoTNC's USB chip. Instead we wire the Pico's UART
//! directly to the NinoTNC's UART pins (bypassing its USB-serial bridge) — TX→RX,
//! RX→TX, GND, at the NinoTNC's KISS baud. This is the planned, supported path.
//! UART0 on GP0 (TX) / GP1 (RX) by default.
//!
//! STUB: UART read/write body to be written against embassy-rp.

use ax25_node_core::kiss::Decoder;

use crate::config::KissSerialConfig;

#[embassy_executor::task]
pub async fn task(/* uart0, tx_pin, rx_pin */ cfg: KissSerialConfig) {
    defmt::info!("kiss-serial: UART @ {} baud (NinoTNC direct UART)", cfg.baud);
    let mut _decoder = Decoder::new();
    // Configure BufferedUart (or DMA UART) at cfg.baud, 8N1.
    // loop {
    //   let n = uart.read(&mut buf).await?;
    //   for frame in _decoder.push(&buf[..n]) { ... same as kiss_tcp ... }
    //   // outbound identical to kiss_tcp, writing to uart instead of socket
    // }
    unimplemented!("KISS-over-UART loop (NinoTNC direct UART)")
}
