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
// GATE 2+ (HW-BRINGUP.md §4): the CYW43/net bring-up and the four transport socket
// stubs don't compile against the real cyw43/embassy-net APIs yet — they were
// deliberately not written blind (no CYW43 emulator). They return module by module
// as Gates 2–6 land; Gate 1 is the minimal link + flash + defmt-heartbeat binary.
// #[cfg(target_os = "none")]
// mod net;
// #[cfg(target_os = "none")]
// mod session;
// #[cfg(target_os = "none")]
// mod transports;

// The global allocator. `ax25-node-core` uses `alloc` (the session queues, the
// streaming codecs), so the firmware must install one. `embedded-alloc`'s
// `LlffHeap` (linked-list first-fit) is a small, mature heap; its backing store is
// a static byte arena initialised once at boot (in `main`, before any allocation).
// Sized conservatively for a handful of sessions with a small window (research §6).
#[cfg(target_os = "none")]
#[global_allocator]
static HEAP: embedded_alloc::LlffHeap = embedded_alloc::LlffHeap::empty();

/// Heap arena size in bytes. A node serving a few links with a small (k≤8) window
/// needs little; this leaves the bulk of the 264 KB SRAM for stacks + statics.
#[cfg(target_os = "none")]
const HEAP_SIZE: usize = 16 * 1024;

#[cfg(target_os = "none")]
mod firmware {
    use defmt_rtt as _; // global defmt logger over RTT
    use panic_probe as _; // panic => defmt message + halt, seen over RTT

    use core::mem::MaybeUninit;

    use embassy_executor::Spawner;
    use embassy_time::{Duration, Instant, Ticker};

    use crate::config;
    use crate::{HEAP, HEAP_SIZE};

    #[embassy_executor::main]
    async fn main(_spawner: Spawner) {
        defmt::info!("pico-node {} starting", ax25_node_core::VERSION);

        // Initialise the global heap arena ONCE, before anything allocates. SAFETY:
        // called exactly once, at the very top of main, on a single static arena.
        {
            static mut ARENA: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
            #[allow(static_mut_refs)]
            unsafe {
                HEAP.init(ARENA.as_ptr() as usize, HEAP_SIZE)
            }
        }

        let _p = embassy_rp::init(Default::default());
        let cfg = config::load();

        // GATE 1 (HW-BRINGUP.md §4): minimal first-silicon binary — boot, init
        // embassy-rp, and heartbeat over defmt/RTT. Proves the whole hands-free
        // loop: memory.x + flip-link + probe-rs flash + reset + RTT streaming.
        // The CYW43/net/transport/session spawns return at Gates 2–6.
        //
        // The config load + callsign log below is deliberate, not decoration: it
        // exercises ax25-node-core (Callsign::parse + write_display) on the real M0+.
        let mut call_buf = [0u8; 16];
        let call_len = cfg
            .identity
            .callsign
            .write_display(&mut call_buf)
            .unwrap_or(0);
        defmt::info!(
            "node identity: {=str} (alias {=str}, grid {=str})",
            core::str::from_utf8(&call_buf[..call_len]).unwrap_or("?"),
            cfg.identity.alias,
            cfg.identity.grid,
        );

        // NOTE (HW-BRINGUP.md §4 Gate 1): no LED blinky here — the Pico W LED hangs
        // off the CYW43, which isn't up until Gate 2. defmt IS the heartbeat.
        let mut ticker = Ticker::every(Duration::from_secs(1));
        loop {
            ticker.next().await;
            defmt::info!("heartbeat: uptime {=u64} s", Instant::now().as_secs());
        }
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
