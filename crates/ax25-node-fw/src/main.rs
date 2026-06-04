//! RP2040 / Pico W AX.25 packet-node firmware — Embassy entry point.
//!
//! This is the thin "wiring" crate: it owns the silicon and the radios and hands
//! all protocol work to [`ax25_node_core`]. The structure mirrors the C# node
//! host (`Packet.Node.Core`): a set of transports feeding one AX.25 listener +
//! session layer, plus a telnet console, all coordinated by a small supervisor.
//!
//! ## Tasks (the multi-source event pump — research note §7 "async fits this")
//!
//! - `cyw43_task`          — drives the CYW43439 WiFi chip (PIO-SPI), always-on.
//! - `net_task`            — runs the `embassy-net` stack (DHCP, sockets).
//! - [`transports::axudp`] — AXUDP: AX.25-over-UDP to peer nodes (capability 1).
//! - [`transports::kiss_tcp`] — KISS-over-TCP to net-sim (capability 2).
//! - [`transports::kiss_serial`] — KISS-over-UART to a NinoTNC (capability 3).
//! - [`transports::telnet`]   — telnet command console (capability 4).
//! - [`session`]              — the SDL link-layer runtime per peer (the port).
//!
//! ## Build / flash / log
//!
//! `cargo run -p ax25-node-fw --release` → probe-rs flashes over SWD, resets, and
//! streams defmt/RTT logs. See `.cargo/config.toml` and docs/PLAN.md. This file
//! does NOT compile in the planning environment (no thumbv6m core, deps not
//! fetched) — it is the ready-to-build skeleton for when the toolchain + board
//! arrive. The `#![cfg]` gate keeps the crate from erroring if someone runs a
//! host `cargo check` against it before the toolchain is set up.

#![no_std]
#![no_main]
#![cfg_attr(not(target_os = "none"), allow(unused))]

// The firmware modules only exist for the bare-metal target. On a host build they
// are compiled out, so a stray `cargo check` on the host doesn't error before the
// embedded toolchain + deps are in place. The real firmware only builds for
// `target_os = "none"` (thumbv6m-none-eabi).
#[cfg(target_os = "none")]
mod config;
#[cfg(target_os = "none")]
mod net;
#[cfg(target_os = "none")]
mod session;
#[cfg(target_os = "none")]
mod transports;

#[cfg(target_os = "none")]
mod firmware {
    use defmt_rtt as _; // global defmt logger over RTT
    use panic_probe as _; // panic => defmt message + halt, seen over RTT

    use embassy_executor::Spawner;
    use embassy_rp::bind_interrupts;
    use embassy_rp::peripherals::{DMA_CH0, PIO0};
    use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};

    use crate::config;
    use crate::net as netmod;
    use crate::session;
    use crate::transports;

    bind_interrupts!(struct Irqs {
        PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    });

    #[embassy_executor::main]
    async fn main(spawner: Spawner) {
        defmt::info!("pico-node {} starting", ax25_node_core::VERSION);

        let p = embassy_rp::init(Default::default());
        let cfg = config::load();

        // --- Bring up the CYW43 WiFi chip over PIO-SPI ---
        // Firmware + CLM blobs are linked from flash (see docs/PLAN.md for how the
        // blobs are vendored). cyw43-pio bit-bangs the half-duplex SPI on PIO0.
        let Pio { common, sm0, .. } = Pio::new(p.PIO0, Irqs);
        let (net_device, mut control) =
            netmod::init_wifi(common, sm0, p.PIN_23, p.PIN_24, p.PIN_25, p.PIN_29, p.DMA_CH0, &spawner)
                .await;

        // --- Join the AP and start embassy-net (DHCP) ---
        netmod::join(&mut control, &cfg.wifi).await;
        let stack = netmod::start_stack(net_device, &spawner, &cfg).await;

        // --- Spawn the transports (each maps to one C# capability) ---
        // 1. AXUDP node↔node over WiFi.
        spawner.must_spawn(transports::axudp::task(stack, cfg.axudp.clone()));
        // 2. KISS-over-TCP to net-sim.
        spawner.must_spawn(transports::kiss_tcp::task(stack, cfg.kiss_tcp.clone()));
        // 3. KISS-over-UART to a NinoTNC (UART0 on GP0/GP1 by default).
        spawner.must_spawn(transports::kiss_serial::task(p.UART0, p.PIN_0, p.PIN_1, cfg.kiss_serial.clone()));
        // 4. Telnet command console.
        spawner.must_spawn(transports::telnet::task(stack, cfg.telnet.clone()));

        // The link-layer session layer (the SDL runtime) is driven by the
        // transports; its timer service runs as its own task.
        spawner.must_spawn(session::timer_task());

        defmt::info!("pico-node up");
    }
}

// A no-op host entry so the bin crate is structurally complete off-target. The
// firmware proper has no host role; this exists only so tooling that resolves the
// binary on the host doesn't fail for lack of a `main`. (On the real target the
// `#[embassy_executor::main]` above is the entry; `#![no_main]` suppresses this.)
#[cfg(not(target_os = "none"))]
fn main() {
    eprintln!(
        "ax25-node-fw is RP2040 firmware; build it for thumbv6m-none-eabi.\n\
         See docs/PLAN.md. Host-testable logic lives in the ax25-node-core crate."
    );
}
