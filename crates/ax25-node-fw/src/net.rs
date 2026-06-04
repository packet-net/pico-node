//! WiFi + TCP/IP bring-up over Embassy. STUB — the wiring shape only.
//!
//! Mirrors the embassy `examples/rp/src/bin/wifi_*` reference: load the CYW43
//! firmware + CLM blobs, spawn the `cyw43` runner task, init `embassy-net` with a
//! DHCP config, and spawn the net runner task. The returned `Stack` is shared by
//! every transport task (AXUDP/KISS-TCP/telnet).
//!
//! Not compiled yet (no cyw43/embassy-net deps in this environment). The exact
//! generics depend on the pinned crate versions; treat the signatures as the
//! intended seams, finalised against the real API when the toolchain is in place.

use embassy_executor::Spawner;
use embassy_net::Stack;

use crate::config::{NodeConfig, WifiConfig};

// The CYW43 firmware + country-locale-matrix blobs. These ship as byte arrays
// linked into flash. Vendoring approach is documented in docs/PLAN.md
// (§"When the hardware arrives") — they come from the embassy `cyw43-firmware`
// directory and must NOT be committed without checking their licence.
// static FW: &[u8] = include_bytes!("../cyw43-firmware/43439A0.bin");
// static CLM: &[u8] = include_bytes!("../cyw43-firmware/43439A0_clm.bin");

/// Initialise the CYW43 chip over PIO-SPI; returns the net device + control
/// handle. (Signature elided to a doc-stub: the concrete PIO/pin/DMA generics are
/// version-specific — see embassy `wifi_tcp_server.rs`.)
pub async fn init_wifi(/* common, sm0, pins, dma, spawner */) {
    unimplemented!("bring up cyw43 + spawn cyw43 runner task — see embassy examples/rp")
}

/// Join the configured access point (WPA2 station mode).
pub async fn join(_wifi: &WifiConfig) {
    unimplemented!("control.join_wpa2(ssid, password) with retry/backoff")
}

/// Start the embassy-net stack with DHCPv4 and spawn its runner task.
pub async fn start_stack(_spawner: &Spawner, _cfg: &NodeConfig) -> Stack<'static> {
    unimplemented!("embassy_net::new(device, Config::dhcpv4(...), resources, seed) + spawn net_task")
}
