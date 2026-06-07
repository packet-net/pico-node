//! WiFi + TCP/IP bring-up over Embassy (HW-BRINGUP.md §4 Gate 2).
//!
//! Mirrors the embassy `examples/rp/src/bin/wifi_*` reference at the pinned
//! versions (cyw43 0.7 / cyw43-pio 0.10 / embassy-net 0.9 / embassy-rp 0.10):
//! load the CYW43 firmware + NVRAM + CLM blobs, bring the chip up over PIO-SPI,
//! spawn the `cyw43` runner task, init `embassy-net` with a DHCPv4 config, and
//! spawn the net runner task. The returned `Stack` (it is `Copy`) is shared by
//! every transport task (AXUDP/KISS-TCP/telnet).

use cyw43::{Control, JoinAuth, JoinOptions, NetDriver};
use cyw43_pio::{PioSpi, DEFAULT_CLOCK_DIVIDER};
use embassy_executor::Spawner;
use embassy_net::{Config, DhcpConfig, Ipv4Address, Ipv4Cidr, StaticConfigV4, Stack, StackResources};
use embassy_rp::bind_interrupts;
use embassy_rp::clocks::RoscRng;
use embassy_rp::dma;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIN_23, PIN_24, PIN_25, PIN_29, PIO0};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::Peri;
use embassy_time::{with_timeout, Duration, TimeoutError, Timer};
use static_cell::StaticCell;

use crate::config::WifiConfig;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>;
});

/// Socket-resource pool for the whole node: DHCPv4 + AXUDP (UDP) + KISS-TCP +
/// telnet, with headroom for a second console connection.
const SOCKET_COUNT: usize = 8;

/// The cyw43 chip driver task — services the PIO-SPI bus + chip events forever.
#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, cyw43::SpiBus<Output<'static>, PioSpi<'static, PIO0, 0>>>,
) -> ! {
    runner.run().await
}

/// The embassy-net stack task — runs DHCP + all socket I/O forever.
#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, NetDriver<'static>>) -> ! {
    runner.run().await
}

/// Bring up the CYW43439 over PIO-SPI and spawn its runner task; returns the
/// net device (for [`start_stack`]) and the control handle (join, LED GPIO).
///
/// Pin map is the Pico W board wiring (fixed by the board, not configurable):
/// PIN_23 = WL_ON (power), PIN_25 = chip select, PIN_24 = DIO, PIN_29 = CLK.
pub async fn init_wifi(
    spawner: &Spawner,
    pio0: Peri<'static, PIO0>,
    pwr_pin: Peri<'static, PIN_23>,
    dio_pin: Peri<'static, PIN_24>,
    cs_pin: Peri<'static, PIN_25>,
    clk_pin: Peri<'static, PIN_29>,
    dma_ch0: Peri<'static, DMA_CH0>,
) -> (NetDriver<'static>, Control<'static>) {
    // Firmware + NVRAM + CLM blobs, linked from flash (vendored under
    // ../cyw43-firmware/ with their licence — see the README there + PLAN §5).
    let fw = cyw43::aligned_bytes!("../cyw43-firmware/43439A0.bin");
    let nvram = cyw43::aligned_bytes!("../cyw43-firmware/nvram_rp2040.bin");
    let clm = include_bytes!("../cyw43-firmware/43439A0_clm.bin");

    let pwr = Output::new(pwr_pin, Level::Low);
    let cs = Output::new(cs_pin, Level::High);
    let mut pio = Pio::new(pio0, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        DEFAULT_CLOCK_DIVIDER,
        pio.irq0,
        cs,
        dio_pin,
        clk_pin,
        dma::Channel::new(dma_ch0, Irqs),
    );

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw, nvram).await;
    spawner.spawn(defmt::unwrap!(cyw43_task(runner)));

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;
    defmt::info!("cyw43 up (firmware + CLM loaded)");

    (net_device, control)
}

/// The AP-mode gateway/host address and subnet (the captive-portal subnet).
pub const AP_ADDRESS: Ipv4Address = Ipv4Address::new(192, 168, 4, 1);
pub const AP_PREFIX_LEN: u8 = 24;

/// Start the CYW43 as a WPA2 access point on `ssid`/`passphrase`, channel 6.
/// The node's config AP — clients associate to reach the captive portal.
pub async fn start_ap(control: &mut Control<'static>, ssid: &str, passphrase: &str) {
    defmt::info!("ap: starting WPA2 AP {=str} on channel 6", ssid);
    control.start_ap_wpa2(ssid, passphrase, 6).await;
    // LED on = AP is up (the only "radio alive" indicator in AP mode).
    control.gpio_set(0, true).await;
}

/// Start the embassy-net stack with a STATIC config (AP mode: we are the
/// gateway at [`AP_ADDRESS`]). The DHCP server (see `provisioning::dhcp`) hands
/// clients addresses on this subnet.
pub async fn start_stack_static(
    net_device: NetDriver<'static>,
    spawner: &Spawner,
) -> Stack<'static> {
    let config = Config::ipv4_static(StaticConfigV4 {
        address: Ipv4Cidr::new(AP_ADDRESS, AP_PREFIX_LEN),
        gateway: Some(AP_ADDRESS),
        dns_servers: Default::default(),
    });
    let seed = RoscRng.next_u64();
    static RESOURCES: StaticCell<StackResources<{ SOCKET_COUNT }>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(
        net_device,
        config,
        RESOURCES.init(StackResources::new()),
        seed,
    );
    spawner.spawn(defmt::unwrap!(net_task(runner)));
    stack
}

/// Try to join the configured AP up to `max_attempts` times (the mode machine
/// falls back to AP mode if this returns `false`). Explicit-count, not the
/// forever-retry of the original — a node with no reachable home WiFi must not
/// hang in STA limbo when it could be offering its config AP.
pub async fn try_join(
    control: &mut Control<'static>,
    wifi: &WifiConfig,
    max_attempts: u32,
) -> bool {
    if wifi.ssid.is_empty() {
        return false;
    }
    let mut backoff_secs = 1u64;
    for attempt in 1..=max_attempts {
        defmt::info!(
            "joining AP {=str} (attempt {=u32}/{=u32})...",
            wifi.ssid,
            attempt,
            max_attempts
        );
        let mut opts = JoinOptions::new(wifi.password.as_bytes());
        opts.auth = JoinAuth::Wpa2;
        match with_timeout(Duration::from_secs(20), control.join(wifi.ssid, opts)).await {
            Ok(Ok(())) => {
                defmt::info!("joined AP {=str}", wifi.ssid);
                return true;
            }
            Ok(Err(e)) => defmt::warn!("join failed ({:?})", e),
            Err(TimeoutError) => defmt::warn!("join timed out"),
        }
        if attempt < max_attempts {
            Timer::after_secs(backoff_secs).await;
            backoff_secs = (backoff_secs * 2).min(30);
        }
    }
    false
}

/// STA-mode keepalive task: owns the control handle after a successful join so
/// it (and the radio association) outlive `main`. cyw43 0.7's `Control` exposes
/// no link-down event, so this parks; a future bump turns it into active
/// re-association.
#[embassy_executor::task]
pub async fn sta_keepalive(mut control: Control<'static>, wifi: WifiConfig) {
    control.gpio_set(0, true).await; // LED on = associated
    let _ = &wifi;
    loop {
        embassy_time::Timer::after_secs(3600).await;
    }
}

/// Join the configured access point forever (kept for STA-mode operation once
/// joined — re-associates if the link drops).
#[allow(dead_code)]
pub async fn join(control: &mut Control<'static>, wifi: &WifiConfig) {
    if wifi.ssid.is_empty() {
        defmt::panic!("join() called with no SSID");
    }
    let mut backoff_secs = 1u64;
    loop {
        defmt::info!("joining AP {=str}...", wifi.ssid);
        // Explicit WPA2: the JoinOptions default (Wpa2Wpa3) sets the chip's auth
        // mode to SAE, which is known to hang the association against WPA2-only
        // APs (no join event ever fires). WPA2-PSK is what packet-node LANs run.
        let mut opts = JoinOptions::new(wifi.password.as_bytes());
        opts.auth = JoinAuth::Wpa2;
        // The chip can also swallow the join result entirely (observed during
        // bring-up: SetSsid issued, no SET_SSID/PSK_SUP event back) — so guard
        // the wait with a timeout and retry instead of hanging a headless node.
        match with_timeout(Duration::from_secs(20), control.join(wifi.ssid, opts)).await {
            Ok(Ok(())) => {
                defmt::info!("joined AP {=str}", wifi.ssid);
                return;
            }
            Ok(Err(e)) => {
                defmt::warn!("join failed ({:?}), retrying in {=u64}s", e, backoff_secs);
            }
            Err(TimeoutError) => {
                defmt::warn!(
                    "join timed out (no event from chip), retrying in {=u64}s",
                    backoff_secs
                );
            }
        }
        Timer::after_secs(backoff_secs).await;
        backoff_secs = (backoff_secs * 2).min(30);
    }
}

/// Start the embassy-net stack with DHCPv4 and spawn its runner task. Returns
/// the `Copy`able stack handle the transports share. Does not wait for a lease —
/// callers that need the network up await `stack.wait_config_up()`.
///
/// `hostname` rides in DHCP option 12, so the node shows up named in the
/// router's client list / local DNS even before mDNS is consulted.
pub async fn start_stack(
    net_device: NetDriver<'static>,
    spawner: &Spawner,
    hostname: &str,
) -> Stack<'static> {
    let mut dhcp = DhcpConfig::default();
    dhcp.hostname = hostname.try_into().ok();
    let config = Config::dhcpv4(dhcp);
    // Seed smoltcp's TCP sequence numbers from the ring-oscillator TRNG.
    let seed = RoscRng.next_u64();

    static RESOURCES: StaticCell<StackResources<SOCKET_COUNT>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(
        net_device,
        config,
        RESOURCES.init(StackResources::new()),
        seed,
    );
    spawner.spawn(defmt::unwrap!(net_task(runner)));
    stack
}
