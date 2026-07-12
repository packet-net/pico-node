#![allow(dead_code)] // built + type-checked; the task is spawned at Gate 6 (a NinoTNC on GP20/21)

//! Capability 3 — KISS-over-serial to a NinoTNC.
//!
//! Ports `Packet.Kiss.Serial.KissSerialModem` + the NinoTNC overlay onto an
//! `embassy_rp` UART. The KISS framing/codec, the [`SerialKissModem`] seam, the
//! ACKMODE/SETHW/parameter helpers, and the NinoTNC mode/TX-Test extensions are all
//! in [`ax25_node_core::kiss`] (host-tested) — this task only supplies the *byte
//! source*: a [`ByteStream`] over the UART, which [`SerialKissModem`] drives exactly
//! as the C# modem drives its `SerialPort`.
//!
//! ## Hardware note (research §5.3 / capability 3 caveat)
//!
//! The RP2040 cannot be a USB *host* and a USB-serial device simultaneously, so we
//! do NOT talk to the NinoTNC's USB chip. Instead we wire the Pico's UART directly
//! to the NinoTNC's UART pins (bypassing its USB-serial bridge) — TX→RX, RX→TX, GND,
//! at the NinoTNC's KISS baud ([`ax25_node_core::kiss::ninotnc::DEFAULT_BAUD`] =
//! 57 600 8N1). **UART1 on GP20 (TX) / GP21 (RX)** — the NinoTNC link pins on the
//! NinoBLE Rev5 carrier board (docs/HARDWARE-NINOBLE.md), our reference hardware.
//!
//! The UART layer below is real (embassy-rp 0.10 `BufferedUart`), so this module
//! compiles and is type-checked by CI. HARDWARE-GATED for *running*: the live
//! exchange needs a physical NinoTNC on GP20/21 — not present on the bare-Pico
//! bench rig, so the task is not spawned yet (HW-BRINGUP Gate 6).

use ax25_node_core::kiss::ninotnc::{self, NinoTncInboundEvent};
use ax25_node_core::kiss::serial::ByteStream;
use ax25_node_core::kiss::{classify::InboundEvent, SerialKissModem};

use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{PIN_20, PIN_21, UART1};
use embassy_rp::uart::{
    BufferedInterruptHandler, BufferedUart, Config as UartConfig, Error as UartError,
};
use embassy_rp::Peri;
use embedded_io_async::{Read, Write};
use static_cell::StaticCell;

use crate::config::KissSerialConfig;

bind_interrupts!(struct Irqs {
    UART1_IRQ => BufferedInterruptHandler<UART1>;
});

/// A [`ByteStream`] over an `embassy_rp` buffered UART — the embedded byte source the
/// portable [`SerialKissModem`] runs on. `read`/`write` are the only hardware seam;
/// everything above (framing, escaping, the modem, the NinoTNC extensions) is the
/// host-tested portable core.
pub struct UartByteStream {
    uart: BufferedUart,
}

impl UartByteStream {
    /// Wrap a configured buffered UART.
    pub fn new(uart: BufferedUart) -> Self {
        Self { uart }
    }
}

impl ByteStream for UartByteStream {
    type Error = UartError;

    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        // `embedded_io_async::Read::read` awaits at least one byte (0 only on EOF),
        // which is exactly the contract `SerialKissModem::read_frame` expects.
        Read::read(&mut self.uart, buf).await
    }

    async fn write(&mut self, bytes: &[u8]) -> Result<(), Self::Error> {
        Write::write_all(&mut self.uart, bytes).await
    }
}

#[embassy_executor::task]
pub async fn task(
    uart: Peri<'static, UART1>,
    tx_pin: Peri<'static, PIN_20>,
    rx_pin: Peri<'static, PIN_21>,
    cfg: KissSerialConfig,
) {
    defmt::info!(
        "kiss-serial: UART1 GP20/21 @ {} baud (NinoTNC direct UART)",
        cfg.baud
    );

    let uart = configure_uart(uart, tx_pin, rx_pin, cfg.baud);

    let mut modem = SerialKissModem::new(UartByteStream::new(uart));

    // Optionally drive the NinoTNC into a known mode at startup (RAM-only, sparing
    // flash). The C# node does this via NinoTncSerialPort.SetModeAsync. Example:
    //   let _ = ninotnc::sethw::build_kiss_frame_into(&mut buf, 6, false, 0)
    //               .map(|n| /* modem write */);
    // Left to config policy; the helper is wired below so the import is load-bearing.
    let _ = ninotnc::sethw::build_payload_byte;

    // The read pump: pull each inbound KISS frame and classify it with NinoTNC
    // awareness, then route. This mirrors NinoTncSerialPort.DispatchFramesAsync.
    loop {
        match modem.read_frame().await {
            Ok(Some(frame)) => match ninotnc::classify(&frame) {
                NinoTncInboundEvent::Generic(InboundEvent::Ax25 { ax25, .. }) => {
                    // READ-ONLY NET/ROM TAP — every frame, BEFORE the address filter,
                    // so NODES broadcasts (dest "NODES", not us) are heard. Then the
                    // normal address-filtered routing to a session (same seam as the
                    // kiss_tcp / axudp transports).
                    //   session::observe_inbound(&mut netrom, &ax25, my_call, PortId::from_str_lossy("kiss-serial"));
                    //   session::deliver_kiss(ax25).await;
                    let _ = ax25;
                }
                NinoTncInboundEvent::TxTestDiagnostic { diagnostic, .. } => {
                    // The on-demand modem diagnostic (button pressed on THIS NinoTNC):
                    // firmware version, running mode, counters. Surface to the console.
                    defmt::info!(
                        "ninotnc tx-test: fw={=str} running-mode={:?}",
                        diagnostic.firmware_version_raw.as_str(),
                        diagnostic.running_mode.map(|m| m.mode)
                    );
                }
                NinoTncInboundEvent::AirTest { air_test, .. } => {
                    // Over-air TX-Test from ANOTHER NinoTNC operator — a link-quality
                    // probe. Log the learned callsign + press counter.
                    defmt::info!("ninotnc air-test: seq={}", air_test.sequence_counter);
                }
                NinoTncInboundEvent::StatusReport { status, .. } => {
                    // Periodic numeric =II: diagnostic-register beacon (or a GETALL
                    // reply) — modem telemetry, not an inbound AX.25 frame.
                    defmt::info!(
                        "ninotnc status: fw={=str}",
                        status.firmware_version_raw.as_str()
                    );
                }
                NinoTncInboundEvent::RssiReading { rssi, .. } => {
                    // A GETRSSI reply — RX-audio level, not an inbound AX.25 frame.
                    defmt::info!("ninotnc rssi: {=i32} centi-dB", rssi.centi_db);
                }
                NinoTncInboundEvent::Generic(InboundEvent::AckModeData { .. })
                | NinoTncInboundEvent::Generic(InboundEvent::Unknown { .. }) => {
                    // ACKMODE data / unrecognised — not part of the inbound AX.25 path.
                }
            },
            // EOF / link-down: a buffered UART doesn't really "close", but on a read
            // error or zero-read we yield and retry rather than spin.
            Ok(None) => embassy_time::Timer::after_millis(10).await,
            Err(e) => {
                defmt::warn!("kiss-serial read error: {}", defmt::Debug2Format(&e));
                embassy_time::Timer::after_millis(100).await;
            }
        }
        // Outbound is symmetric: the session layer hands an AX.25 body to
        //   modem.send_frame(&ax25_bytes).await
        // (or modem.send_kiss(Command::AckMode, &payload) for ACKMODE), with the
        // SETHW / parameter setters available on the same `modem`.
    }
}

/// Configure UART1 as a buffered 8N1 UART at `baud` on GP20 (TX) / GP21 (RX) —
/// the NinoBLE Rev5 NinoTNC link. Static TX/RX ring buffers sized for a couple
/// of KISS frames.
fn configure_uart(
    uart: Peri<'static, UART1>,
    tx_pin: Peri<'static, PIN_20>,
    rx_pin: Peri<'static, PIN_21>,
    baud: u32,
) -> BufferedUart {
    let mut config = UartConfig::default();
    config.baudrate = baud;
    static TX_BUF: StaticCell<[u8; 256]> = StaticCell::new();
    static RX_BUF: StaticCell<[u8; 256]> = StaticCell::new();
    BufferedUart::new(
        uart,
        tx_pin,
        rx_pin,
        Irqs,
        TX_BUF.init([0; 256]),
        RX_BUF.init([0; 256]),
        config,
    )
}
