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

// The firmware uses `alloc` (ax25-node-core's frame buffers; the heap is the
// embedded-alloc arena installed below).
#[cfg(target_os = "none")]
extern crate alloc;

// The firmware modules only exist for the bare-metal target. On a host build they
// are compiled out, so a stray `cargo check` on the host doesn't error before the
// embedded toolchain + deps are in place. The real firmware only builds for
// `target_os = "none"` (thumbv6m-none-eabi).
#[cfg(target_os = "none")]
mod config;
#[cfg(target_os = "none")]
mod config_store;
#[cfg(target_os = "none")]
mod mdns;
#[cfg(target_os = "none")]
mod net;
#[cfg(target_os = "none")]
mod mqtt;
#[cfg(target_os = "none")]
mod netrom_store;
#[cfg(target_os = "none")]
mod netrom_view;
#[cfg(target_os = "none")]
mod oled;
#[cfg(target_os = "none")]
mod ota;
#[cfg(target_os = "none")]
mod provisioning;
#[cfg(target_os = "none")]
mod session;
#[cfg(target_os = "none")]
mod transports;

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

    use alloc::string::String;
    use core::mem::MaybeUninit;

    use embassy_executor::Spawner;
    use embassy_time::{Duration, Instant, Ticker};

    use crate::config;
    use crate::config_store;
    use crate::mdns;
    use crate::mqtt;
    use crate::net;
    use crate::oled;
    use crate::ota;
    use crate::provisioning;
    use crate::transports;
    use crate::{HEAP, HEAP_SIZE};

    #[embassy_executor::main]
    async fn main(spawner: Spawner) {
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

        let p = embassy_rp::init(Default::default());

        // OTA rollback bench-test hook: a build with OTA_FORCE_BRICK set resets
        // immediately, BEFORE marking the firmware good — simulating a trial
        // image that never confirms. The bootloader then reverts to the prior
        // image on the following boot. Off in every normal build (the env var is
        // unset), compiled to nothing.
        if option_env!("OTA_FORCE_BRICK").is_some() {
            defmt::error!("OTA_FORCE_BRICK: resetting without marking good (rollback test)");
            cortex_m::peripheral::SCB::sys_reset();
        }

        // Compiled factory defaults, then the flash store on top (provisioning
        // steps 1–2 — docs/PROVISIONING.md). The store also feeds the console
        // SET/SHOW/SAVE executor via the CONFIG static.
        let mut cfg = config::load();
        let flash =
            embassy_rp::flash::Flash::<_, _, { crate::config_store::FLASH_SIZE }>::new_blocking(
                p.FLASH,
            );
        // OTA: confirm this boot as good (no-op/wear-free unless we're a freshly
        // swapped trial image). Done first, before anything can panic — a hang
        // before this point makes the bootloader roll back on the next reset.
        let flash = ota::mark_booted_early(flash);
        let (service, stored) = config_store::ConfigService::new(flash);
        config_store::CONFIG.lock(|cell| cell.borrow_mut().replace(service));
        if let Some(stored) = stored {
            defmt::info!("config: stored record loaded (overrides factory defaults)");
            config::apply_stored(&mut cfg, &stored);
        } else {
            defmt::info!("config: no stored record — factory defaults");
        }

        // The config load + callsign log is deliberate, not decoration: it
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

        // --- GATE 2 (HW-BRINGUP.md §4): CYW43 + WiFi + DHCP ---
        // Pin map is the Pico W board wiring: 23=WL_ON, 24=DIO, 25=CS, 29=CLK.
        let (net_device, mut control) = net::init_wifi(
            &spawner, p.PIO0, p.PIN_23, p.PIN_24, p.PIN_25, p.PIN_29, p.DMA_CH0,
        )
        .await;
        // --- Mode machine (docs/PROVISIONING.md): STA if WiFi is configured and
        // joinable, else the config AP. STA join is bounded (not forever) so a
        // node with no reachable home WiFi falls back to offering its AP. ---
        let call_text = core::str::from_utf8(&call_buf[..call_len]).unwrap_or("?");

        // A node with no configured callsign MUST NOT operate on the air (illegal
        // to transmit without a licensed call). It comes up config-only: the AP +
        // captive portal, nothing else, until a callsign is set. So force AP mode
        // regardless of any WiFi config.
        let configured = cfg.identity.callsign_configured;
        if !configured {
            defmt::warn!("mode: NO CALLSIGN CONFIGURED — config-only AP mode (set a callsign to operate)");
        }

        let sta_ok = if !configured || cfg.wifi.ssid.is_empty() {
            if configured {
                defmt::info!("mode: no WiFi configured — AP mode");
            }
            false
        } else {
            net::try_join(&mut control, &cfg.wifi, 3).await
        };

        let stack;
        if sta_ok {
            // STA mode: DHCP client, re-associate forever if the link drops.
            stack = net::start_stack(net_device, &spawner, cfg.hostname).await;
            spawner.spawn(defmt::unwrap!(net::sta_keepalive(
                control,
                cfg.wifi.clone()
            )));
            defmt::info!("waiting for link + DHCPv4 lease...");
            stack.wait_link_up().await;
            stack.wait_config_up().await;
            if let Some(v4) = stack.config_v4() {
                defmt::info!("IP address: {} (STA mode)", v4.address);
            }
        } else {
            // AP mode: become the gateway, serve DHCP (captive portal = step 4).
            // SSID is "pico-<callsign>" once configured, "pico-setup" before.
            let mut ssid_buf = String::from("pico-");
            ssid_buf.push_str(if configured { call_text } else { "setup" });
            net::start_ap(&mut control, &ssid_buf, cfg.wifi.ap_passphrase).await;
            stack = net::start_stack_static(net_device, &spawner).await;
            spawner.spawn(defmt::unwrap!(provisioning::dhcp_server(stack)));
            // Captive portal: DNS catch-all (every name -> us, pops the portal)
            // + the HTTP config form (PROVISIONING.md step 4).
            spawner.spawn(defmt::unwrap!(provisioning::dns_catch_all(stack)));
            spawner.spawn(defmt::unwrap!(provisioning::http_portal(stack)));
            defmt::info!(
                "AP mode up: SSID {=str}, gateway 192.168.4.1 (connect to configure)",
                ssid_buf.as_str()
            );
            // Keep the control handle alive for the AP's lifetime.
            core::mem::forget(control);
        }

        // --- OLED status display (NinoBLE Rev5, I2C0 GP4/GP5; optional —
        // the task self-disables if no SSD1306 ACKs at 0x3C). ---
        {
            let mut st = oled::Status {
                sta: sta_ok,
                ..Default::default()
            };
            let disp = if configured { call_text } else { "NO CALL" };
            let n = disp.len().min(12);
            st.callsign[..n].copy_from_slice(&disp.as_bytes()[..n]);
            oled::set(st);
        }
        spawner.spawn(defmt::unwrap!(oled::task(p.I2C0, p.PIN_4, p.PIN_5, stack)));

        // CALLSIGN GATE: an unconfigured node stops here — the AP + captive
        // portal are up (so you can set a callsign), but NO on-air transport is
        // started. Park until reconfigured + rebooted.
        if !configured {
            defmt::info!("config-only: set a callsign via the portal (http://192.168.4.1/), then it reboots into service");
            let mut ticker = Ticker::every(Duration::from_secs(30));
            loop {
                ticker.next().await;
                defmt::info!("awaiting callsign configuration (uptime {=u64}s)", Instant::now().as_secs());
            }
        }

        // --- MQTT telemetry/log sink (optional; observability without a probe).
        // Self-disables if MQTT_HOST is unset. ---
        {
            let mut cs = heapless::String::<12>::new();
            let _ = core::fmt::Write::write_str(&mut cs, call_text);
            spawner.spawn(defmt::unwrap!(mqtt::task(
                stack,
                mqtt::MqttConfig {
                    host: cfg.mqtt_host,
                    callsign: cs,
                },
            )));
        }

        // The node console identity + prompt — shared by every console-bearing
        // transport (telnet now, AX.25 sessions on the AXUDP port too).
        let console_id = ax25_node_core::console::service::Identity {
            node_name: String::from(cfg.identity.alias),
            callsign: String::from(call_text),
            grid: Some(String::from(cfg.identity.grid)),
            ports: alloc::vec![alloc::format!(
                "axudp [up] udp/0.0.0.0:{}",
                cfg.axudp.listen_port
            )],
            // Filled live per-command by the console tasks from `netrom_view`
            // (the routing table lives in the axudp task, not the console tasks).
            routes: alloc::vec![],
        };
        // Prompt suffix "> " (the {call}> form), aligned with pdn's node prompt.
        let mut prompt = String::from(call_text);
        prompt.push_str("> ");

        // --- GATE 3 (HW-BRINGUP.md §4): AXUDP over WiFi (capability 1), now with
        // the connected-mode session layer + AX.25 console attached. ---
        spawner.spawn(defmt::unwrap!(transports::axudp::task(
            stack,
            cfg.axudp.clone(),
            cfg.netrom.clone(),
            cfg.identity.callsign,
            console_id.clone(),
            prompt.clone(),
        )));

        // --- GATE 4 (HW-BRINGUP.md §4): telnet console (capability 4) ---
        spawner.spawn(defmt::unwrap!(transports::telnet::task(
            stack,
            cfg.telnet.clone(),
            console_id,
            prompt,
        )));

        // --- OTA firmware update over HTTP (docs/OTA.md) — STA mode only: in
        // AP mode the captive portal owns :80 and you're physically present
        // (BOOTSEL). A configured, networked node serves the upload page +
        // POST /firmware at http://<ip>/. ---
        if sta_ok {
            spawner.spawn(defmt::unwrap!(ota::http_task(stack)));
        }

        // --- GATE 5 (HW-BRINGUP.md §4): KISS-over-TCP (capability 2) ---
        // Disabled unless KISS_TCP_TARGET is set in the build env (§5).
        spawner.spawn(defmt::unwrap!(transports::kiss_tcp::task(
            stack,
            cfg.kiss_tcp.clone(),
            cfg.identity.callsign,
        )));

        // --- mDNS: make the node discoverable as <hostname>.local + _telnet._tcp ---
        spawner.spawn(defmt::unwrap!(mdns::task(
            stack,
            mdns::MdnsConfig {
                hostname: cfg.hostname,
                telnet_port: cfg.telnet.port,
            },
        )));

        // GATE 6+ returns kiss_serial (needs a NinoTNC) + the session supervisor.

        let mut ticker = Ticker::every(Duration::from_secs(10));
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
