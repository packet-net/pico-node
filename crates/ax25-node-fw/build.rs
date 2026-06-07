//! Build script: put `memory.x` (the RP2040 linker layout) on the linker search
//! path and re-run if it changes. Standard cortex-m-rt / embassy-rp scaffolding.
//!
//! Only runs when the firmware crate is actually built (for thumbv6m); it is a
//! no-op cost otherwise. See docs/PLAN.md.

use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    // Copy memory.x to the cargo OUT_DIR and add that dir to the linker search
    // path, so `-Tlink.x` (from cortex-m-rt) can find our MEMORY definition.
    let out = PathBuf::from(env::var("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");

    // Compile-time config knobs read via option_env! (config.rs, ota.rs,
    // main.rs). Declared here so changing them forces a recompile — without
    // this, cargo serves a cached build and silently bakes in STALE values.
    // This is security-relevant: WIFI_* / NODE_* live in this machine's
    // ~/.cargo/config.toml [env], so a release build MUST be able to clear them
    // and have that actually take effect (see scripts/package-ota.sh).
    for var in [
        "NODE_CALLSIGN",
        "NODE_ALIAS",
        "NODE_GRID",
        "WIFI_SSID",
        "WIFI_PASSWORD",
        "AP_PASSPHRASE",
        "AXUDP_BEACON_TARGET",
        "KISS_TCP_TARGET",
        "NODES_INTERVAL_SECS",
        "MQTT_HOST",
        "OTA_BUILD_TAG",
        "OTA_FORCE_BRICK",
    ] {
        println!("cargo:rerun-if-env-changed={var}");
    }

    // Gate 7 (HW-BRINGUP.md §4): the embedded-test harness's linker script. The
    // embedded-test crate sits in [dependencies] (not dev-) precisely so its
    // build script puts embedded-test.x on the search path for ALL targets —
    // in the normal firmware binary the script resolves to empty sections.
    println!("cargo::rustc-link-arg=-Tembedded-test.x");
}
