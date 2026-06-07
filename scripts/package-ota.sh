#!/usr/bin/env bash
# Build the OTA release artifacts (docs/OTA.md):
#   - pico-node-app.bin       raw application image (the OTA upload payload)
#   - pico-node-combined.uf2  bootloader + app, for the first BOOTSEL flash
#   - pico-node-app.elf       application ELF (debug/symbols)
#
# Usage: scripts/package-ota.sh [OUTDIR]   (default OUTDIR=dist)
# Requires: rustup thumbv6m target, picotool, rust-objcopy (cargo-binutils).
# Pass OTA_BUILD_TAG=<tag> in the env to stamp /version (defaults to the crate
# version).
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${1:-$ROOT/dist}"
mkdir -p "$OUT"

BL_DIR="$ROOT/crates/ax25-node-bootloader"
APP_DIR="$ROOT/crates/ax25-node-fw"
TGT=thumbv6m-none-eabi

echo "==> building bootloader"
( cd "$BL_DIR" && cargo build --release )
BL_ELF="$BL_DIR/target/$TGT/release/ax25-node-bootloader"

echo "==> building application"
( cd "$APP_DIR" && cargo build --release )
APP_ELF="$APP_DIR/target/$TGT/release/ax25-node-fw"

echo "==> objcopy app -> raw .bin (OTA upload payload)"
rust-objcopy -O binary "$APP_ELF" "$OUT/pico-node-app.bin"
cp "$APP_ELF" "$OUT/pico-node-app.elf"

echo "==> converting to UF2 + combining (bootloader + app)"
picotool uf2 convert "$BL_ELF"  -t elf "$OUT/.bootloader.uf2"
picotool uf2 convert "$APP_ELF" -t elf "$OUT/.app.uf2"
cat "$OUT/.bootloader.uf2" "$OUT/.app.uf2" > "$OUT/pico-node-combined.uf2"
rm -f "$OUT/.bootloader.uf2" "$OUT/.app.uf2"

echo "==> artifacts in $OUT:"
ls -la "$OUT"/pico-node-app.bin "$OUT"/pico-node-combined.uf2 "$OUT"/pico-node-app.elf
( cd "$OUT" && sha256sum pico-node-app.bin pico-node-combined.uf2 pico-node-app.elf > SHA256SUMS && cat SHA256SUMS )
