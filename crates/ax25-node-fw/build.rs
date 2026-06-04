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
}
