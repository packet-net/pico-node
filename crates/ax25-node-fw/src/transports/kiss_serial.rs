#![allow(dead_code)] // spawned now; some core modem setters (params/ackmode) stay unused until a session supervisor drives outbound

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
//! compiles and is type-checked by CI. The task is spawned (mirroring the KISS-TCP
//! transport: read-only NET/ROM tap + NODES origination + obsolescence sweep +
//! beacon), but is HARDWARE-GATED for *running*: the live exchange needs a physical
//! NinoTNC on GP20/21 — not present on the bare-Pico bench rig (HW-BRINGUP Gate 6).
//! Everything here is COMPILE-VALIDATED ONLY until that hardware is attached.

use ax25_node_core::ax25::{Callsign, PID_NO_LAYER3};
use ax25_node_core::kiss::ninotnc::{self, NinoTncInboundEvent};
use ax25_node_core::kiss::serial::ByteStream;
use ax25_node_core::kiss::{classify::InboundEvent, SerialKissModem};
use ax25_node_core::netrom::wire::Alias;
use ax25_node_core::netrom::{
    NetRomOriginator, NetRomOriginatorOptions, ObserveOutcome, PortId,
};

use embassy_futures::select::{select, Either};
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{PIN_20, PIN_21, UART1};
use embassy_rp::uart::{
    BufferedInterruptHandler, BufferedUart, Config as UartConfig, Error as UartError,
};
use embassy_rp::Peri;
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_io_async::{Read, Write};
use static_cell::StaticCell;

use crate::config::{KissSerialConfig, NetRomConfig};
use crate::session;
use crate::transports::{call_str, ui_frame};

/// Seconds between beacon UI frames (mirrors the KISS-TCP transport's beacon).
const BEACON_INTERVAL_SECS: u64 = 10;

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
    netrom_cfg: NetRomConfig,
    my_call: Callsign,
    node_alias: &'static str,
) {
    defmt::info!(
        "kiss-serial: UART1 GP20/21 @ {} baud (NinoTNC direct UART)",
        cfg.baud
    );

    let uart = configure_uart(uart, tx_pin, rx_pin, cfg.baud);
    let mut modem = SerialKissModem::new(UartByteStream::new(uart));

    // Optionally drive the NinoTNC into a known mode at startup (RAM-only, sparing
    // flash) — the C# `NinoTncSerialPort.SetModeAsync` equivalent, gated on config
    // policy. `None` (the default) leaves the modem's own mode untouched.
    if let Some(mode) = cfg.startup_mode {
        match modem.set_mode(mode, false).await {
            Ok(()) => defmt::info!("kiss-serial: NinoTNC mode set to {=u8} (RAM-only)", mode),
            Err(e) => {
                defmt::warn!("kiss-serial: set mode failed: {}", defmt::Debug2Format(&e))
            }
        }
    }

    // Read-only NET/ROM tap + NODES origination + obsolescence sweep — the same
    // wiring the KISS-TCP transport uses, now over real RF (Gap A + Gap B). Each
    // transport owns its own routing table (the single-transport-ownership model;
    // the shared session/routing supervisor seam is deferred).
    let mut netrom = session::new_netrom();
    let port_id = PortId::from_str_lossy("kiss-serial");
    let originator = NetRomOriginator::new(NetRomOriginatorOptions {
        enabled: netrom_cfg.originate,
        alias: Some(Alias::from_str_lossy(node_alias)),
        node_call: Some(my_call),
        obsolete_minimum: None,
    });
    let nodes_interval = Duration::from_secs(netrom_cfg.nodes_interval_secs as u64);
    let mut next_nodes_at = Instant::now(); // announce on the first tick
    let mut next_sweep_at = Instant::now() + nodes_interval;
    if netrom_cfg.originate {
        defmt::info!(
            "kiss-serial: NODES origination on, every {=u32}s",
            netrom_cfg.nodes_interval_secs
        );
    }

    let mut ticker = Ticker::every(Duration::from_secs(BEACON_INTERVAL_SECS));
    let mut src_buf = [0u8; 16];
    let mut dst_buf = [0u8; 16];

    // The pump: wake on either an inbound KISS frame or the periodic tick.
    // `SerialKissModem::read_frame` is cancel-safe — its only await is the UART
    // read, and the decode state lives in the modem — so dropping it when the
    // ticker wins loses no buffered bytes.
    loop {
        match select(modem.read_frame(), ticker.next()).await {
            Either::First(read) => match read {
                Ok(Some(frame)) => match ninotnc::classify(&frame) {
                    NinoTncInboundEvent::Generic(InboundEvent::Ax25 { ax25, .. }) => {
                        // READ-ONLY NET/ROM TAP — every frame, BEFORE any address
                        // filter, so NODES broadcasts (dest "NODES", not us) are heard.
                        // The same FrameTraced-equivalent point as axudp / kiss_tcp.
                        let outcome =
                            session::observe_inbound(&mut netrom, &ax25, my_call, port_id);
                        if let ObserveOutcome::Ingested { .. } = outcome {
                            defmt::info!(
                                "kiss-serial: NODES broadcast ingested ({=u32} destinations known)",
                                netrom.destination_count() as u32
                            );
                        }
                        defmt::info!(
                            "kiss-serial: rx {=str} -> {=str} ctl={=u8:#04x} info={=usize}B",
                            call_str(&ax25.source.callsign, &mut src_buf),
                            call_str(&ax25.destination.callsign, &mut dst_buf),
                            ax25.control,
                            ax25.info.len(),
                        );
                        // Address-filtered connected-mode session routing: the
                        // session-supervisor seam (the same deferred point kiss_tcp
                        // leaves — the SDL engine is host-tested in core; only the
                        // socket/UART wiring is hardware-gated).
                    }
                    NinoTncInboundEvent::TxTestDiagnostic { diagnostic, .. } => {
                        // The on-demand modem diagnostic (button pressed on THIS
                        // NinoTNC): firmware version, running mode, counters.
                        defmt::info!(
                            "ninotnc tx-test: fw={=str} running-mode={:?}",
                            diagnostic.firmware_version_raw.as_str(),
                            diagnostic.running_mode.map(|m| m.mode)
                        );
                    }
                    NinoTncInboundEvent::AirTest { air_test, .. } => {
                        // Over-air TX-Test from ANOTHER NinoTNC operator — a
                        // link-quality probe. Log the learned callsign + press counter.
                        defmt::info!("ninotnc air-test: seq={}", air_test.sequence_counter);
                    }
                    NinoTncInboundEvent::StatusReport { status, .. } => {
                        // Periodic numeric =II: diagnostic-register beacon (or a
                        // GETALL reply) — modem telemetry, not an inbound AX.25 frame.
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
                // EOF / link-down: a buffered UART doesn't really "close", but on a
                // read error or zero-read we yield and retry rather than spin.
                Ok(None) => Timer::after_millis(10).await,
                Err(e) => {
                    defmt::warn!("kiss-serial read error: {}", defmt::Debug2Format(&e));
                    Timer::after_millis(100).await;
                }
            },
            Either::Second(()) => {
                // The periodic tick drains outbound to the UART: beacon, then (on the
                // NODES interval) the obsolescence sweep and NODES origination. This
                // mirrors the KISS-TCP tick branch; a session supervisor will later
                // add connected-mode I-frame outbound through the same `modem`.
                let beacon = ui_frame(
                    my_call,
                    Callsign::parse("IDENT").expect("static"),
                    PID_NO_LAYER3,
                    b"pico-node KISS-serial beacon (HW-BRINGUP Gate 6)",
                );
                if let Err(e) = modem.send_frame(&beacon.encode()).await {
                    defmt::warn!("kiss-serial: beacon send failed: {}", defmt::Debug2Format(&e));
                }

                // Obsolescence sweep — age/purge once per NODES interval, before
                // origination (the C# `NetRomService.OnInterval` order).
                if Instant::now() >= next_sweep_at {
                    next_sweep_at = Instant::now() + nodes_interval;
                    let purged = netrom.sweep();
                    if purged > 0 {
                        defmt::info!(
                            "kiss-serial: obsolescence sweep purged {=usize} stale route(s)",
                            purged
                        );
                    }
                }

                // NODES origination — build our broadcasts from the live table, wrap
                // each as a UI frame (dest NODES, PID 0xCF), and send it to the UART.
                if netrom_cfg.originate && Instant::now() >= next_nodes_at {
                    next_nodes_at = Instant::now() + nodes_interval;
                    let payloads = originator.broadcast_nodes(netrom.table());
                    let dest = NetRomOriginator::nodes_destination();
                    let mut sent = 0usize;
                    for payload in &payloads {
                        let frame = ui_frame(my_call, dest, NetRomOriginator::PID, payload);
                        if let Err(e) = modem.send_frame(&frame.encode()).await {
                            defmt::warn!(
                                "kiss-serial: NODES send failed: {}",
                                defmt::Debug2Format(&e)
                            );
                            continue;
                        }
                        sent += 1;
                    }
                    defmt::info!(
                        "kiss-serial: NODES broadcast sent ({=usize} frame(s))",
                        sent
                    );
                }
            }
        }
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
