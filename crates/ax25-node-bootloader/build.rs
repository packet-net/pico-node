//! Put `memory.x` on the linker search path and add the bootloader link args.
//!
//! The bootloader links with BOTH `link.x` (cortex-m-rt) and `link-rp.x`
//! (embassy-rp's RP2040 boot2 glue) — it owns BOOT2 for the whole image. The
//! application, by contrast, must NOT use `link-rp.x`. `--nmagic` avoids
//! page-aligning sections (saves flash on the M0+). `defmt.x` only when the
//! defmt feature is on.

use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");

    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tlink-rp.x");
    if env::var("CARGO_FEATURE_DEFMT").is_ok() {
        println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
    }
}
