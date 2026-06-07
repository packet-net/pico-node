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

# RELEASE SAFETY: this builds a *same-for-everyone* image — NO machine-specific
# values baked in. A dev machine's ~/.cargo/config.toml [env] may set WIFI_* /
# NODE_* / *_TARGET; clear them to empty here so option_env! sees empty (cargo
# config [env] only fills a var when it's UNSET, so empty present values win) and
# the binary carries no credentials, callsign, or LAN endpoints. AP_PASSPHRASE is
# left untouched so it keeps its compiled default (`pico-node-config`); the WPA2
# config AP needs a valid passphrase. OTA_BUILD_TAG passes through (stamps
# /version). build.rs tracks all of these (rerun-if-env-changed), so this
# actually forces a clean recompile.
export NODE_CALLSIGN="" NODE_ALIAS="" NODE_GRID=""
export WIFI_SSID="" WIFI_PASSWORD=""
export AXUDP_BEACON_TARGET="" KISS_TCP_TARGET="" MQTT_HOST="" NODES_INTERVAL_SECS=""
unset OTA_FORCE_BRICK

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
