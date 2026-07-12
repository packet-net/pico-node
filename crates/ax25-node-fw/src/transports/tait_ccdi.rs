#![allow(dead_code)] // spawned now; the RSSI/PTT/channel surface is only partly driven until a session/tuning layer consumes it

//! Tait CCDI radio control — a SECOND UART transport driving the core
//! [`TaitCcdiRadio`] driver.
//!
//! Ports the firmware-wiring half of `Packet.Radio.Tait`: the CCDI codec, the
//! strict command builders (RSSI / PTT / channel / progress-enable) and the
//! transact/demux engine are all in [`ax25_node_core::radio::tait`] (host-tested) —
//! this task only supplies the *byte source* (a [`ByteStream`] over a second UART)
//! and the periodic drive loop, exactly as [`super::kiss_serial`] does for the
//! NinoTNC KISS link.
//!
//! ## What a Tait radio gives us that a bare TNC cannot
//!
//! Driven over its CCDI serial control channel, a Tait TM8100/TM8200 exposes
//! receiver RSSI (0.1 dB units → integer tenths-of-dBm here), hardware
//! carrier-sense (DCD) edges, transmitter keying and channel selection. This task:
//!
//! - enables unsolicited PROGRESS output at boot, so carrier-sense (DCD) and PTT
//!   edges are reported;
//! - optionally retunes to a configured channel at boot;
//! - polls RSSI on a ticker, draining any carrier-sense / PTT / SDM PROGRESS edges
//!   demuxed during each transaction (the driver maintains
//!   [`TaitCcdiRadio::channel_busy`] from them).
//!
//! ## Hardware note
//!
//! **UART0 on GP0 (TX) / GP1 (RX)** — the second UART, distinct from the NinoTNC
//! KISS link on UART1 (GP20/21). The CCDI serial rate defaults to
//! `ax25_node_core::radio::tait::DEFAULT_BAUD` (28 800 8N1), but the radio's
//! programmed rate wins — set it in [`TaitConfig`]. For the split-station head-end
//! the same driver runs over a
//! TCP [`ByteStream`] instead; here it is the local second UART.
//!
//! The UART layer is real (embassy-rp 0.10 `BufferedUart`), so this module compiles
//! and is type-checked by CI. It is HARDWARE-GATED for *running*: the live exchange
//! needs a Tait radio on GP0/GP1 — not present on the bare-Pico bench rig. Everything
//! here is COMPILE-VALIDATED ONLY until that hardware is attached.

use ax25_node_core::radio::tait::driver::{RadioEvent, TaitCcdiRadio, TaitError};

use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{PIN_0, PIN_1, UART0};
use embassy_rp::uart::{BufferedInterruptHandler, BufferedUart, Config as UartConfig};
use embassy_rp::Peri;
use embassy_time::{Duration, Ticker};
use static_cell::StaticCell;

use crate::config::TaitConfig;
use crate::transports::kiss_serial::UartByteStream;

bind_interrupts!(struct Irqs {
    UART0_IRQ => BufferedInterruptHandler<UART0>;
});

#[embassy_executor::task]
pub async fn task(
    uart: Peri<'static, UART0>,
    tx_pin: Peri<'static, PIN_0>,
    rx_pin: Peri<'static, PIN_1>,
    cfg: TaitConfig,
) {
    defmt::info!(
        "tait-ccdi: UART0 GP0/GP1 @ {} baud (Tait CCDI control channel)",
        cfg.baud
    );

    let uart = configure_uart(uart, tx_pin, rx_pin, cfg.baud);
    let mut radio = TaitCcdiRadio::new(UartByteStream::new(uart));

    // Enable unsolicited PROGRESS output — REQUIRED before carrier-sense (DCD) and
    // PTT edges are reported (FUNCTION 0/4). A radio that isn't answering yields
    // NoResponse; log once and carry on (the poll loop keeps retrying implicitly).
    match radio.set_progress_messages(true).await {
        Ok(()) => defmt::info!("tait-ccdi: PROGRESS output enabled (carrier-sense/PTT edges)"),
        Err(e) => defmt::warn!(
            "tait-ccdi: enable PROGRESS failed: {}",
            defmt::Debug2Format(&e)
        ),
    }

    // Optionally retune to a programmed conventional channel at boot (GO_TO_CHANNEL).
    if let Some(channel) = cfg.channel {
        match radio.go_to_channel(channel, None).await {
            Ok(()) => defmt::info!("tait-ccdi: tuned to channel {=u16}", channel),
            Err(e) => defmt::warn!(
                "tait-ccdi: go-to-channel {=u16} failed: {}",
                channel,
                defmt::Debug2Format(&e)
            ),
        }
    }

    let mut ticker = Ticker::every(Duration::from_secs(cfg.rssi_poll_secs));
    loop {
        ticker.next().await;

        // Poll instantaneous RSSI. Carrier-sense / PTT / SDM PROGRESS edges that
        // arrive interleaved are demuxed out of the same read into the driver's
        // event buffer (and update channel_busy) — drained just below.
        match radio.read_rssi_tenths().await {
            Ok(tenths) => {
                let busy = radio.channel_busy().unwrap_or(false);
                defmt::info!(
                    "tait-ccdi: RSSI {=i16} tenths-dBm, channel-busy={=bool}",
                    tenths,
                    busy
                );
            }
            // The radio said nothing this cycle (no radio attached, or genuinely
            // quiet) — stay silent rather than warn every poll.
            Err(TaitError::NoResponse) => {}
            Err(e) => defmt::warn!("tait-ccdi: RSSI read failed: {}", defmt::Debug2Format(&e)),
        }

        // Surface the unsolicited edges demuxed during the transaction above.
        for ev in radio.drain_events() {
            match ev {
                RadioEvent::CarrierSense(busy) => {
                    defmt::info!("tait-ccdi: carrier-sense {=bool} (DCD)", busy)
                }
                RadioEvent::Transmitter(keyed) => {
                    defmt::info!("tait-ccdi: transmitter {=bool} (PTT)", keyed)
                }
                RadioEvent::SdmDeliveryReceipt(ok) => {
                    defmt::info!("tait-ccdi: SDM delivery receipt {=bool}", ok)
                }
                RadioEvent::Progress(_) => {
                    defmt::info!("tait-ccdi: progress {}", defmt::Debug2Format(&ev))
                }
            }
        }
    }
}

/// Configure UART0 as a buffered 8N1 UART at `baud` on GP0 (TX) / GP1 (RX) — the
/// Tait CCDI control channel. Static TX/RX ring buffers sized for a couple of CCDI
/// lines (a CCDI line tops out at ~272 bytes).
fn configure_uart(
    uart: Peri<'static, UART0>,
    tx_pin: Peri<'static, PIN_0>,
    rx_pin: Peri<'static, PIN_1>,
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
