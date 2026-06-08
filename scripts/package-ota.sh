#!/usr/bin/env bash
# Build the OTA release artifacts (docs/OTA.md):
#   - pico-node-app.bin       raw application image, ~284 KB (the OTA upload
#                             payload — NO cyw43 blob; that lives in BLOBS)
#   - pico-node-combined.uf2  bootloader + STATE-clear + cyw43 BLOBS + app,
#                             for the first BOOTSEL flash
#   - pico-node-app.elf       application ELF (symbols)
#
# Usage: scripts/package-ota.sh [OUTDIR]   (default OUTDIR=dist)
# Requires: rustup thumbv6m target, picotool, rust-objcopy, python3.
# Pass OTA_BUILD_TAG=<tag> to stamp /version (defaults to the crate version).
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${1:-$ROOT/dist}"
mkdir -p "$OUT"

BL_DIR="$ROOT/crates/ax25-node-bootloader"
APP_DIR="$ROOT/crates/ax25-node-fw"
FW_DIR="$APP_DIR/cyw43-firmware"
TGT=thumbv6m-none-eabi

# Frozen-layout addresses (must match the memory.x files; check-layout.sh guards
# them). STATE is the embassy-boot state sector — we write a clean (0xFF) page
# there so a BOOTSEL flash always starts with no pending swap, even when
# UPGRADING a node whose previous firmware used a different layout (otherwise
# stale bytes in this sector corrupt the first swap — learned the hard way).
STATE_ADDR=0x10006000
BLOBS_ADDR=0x10108000

# RELEASE SAFETY: same-for-everyone image — clear machine-specific option_env!
# values so no credentials/callsign/LAN endpoints are baked in (a dev machine's
# ~/.cargo/config.toml [env] may set them; cargo config [env] only fills an UNSET
# var, so empty-present wins). AP_PASSPHRASE keeps its compiled default.
export NODE_CALLSIGN="" NODE_ALIAS="" NODE_GRID=""
export WIFI_SSID="" WIFI_PASSWORD=""
export AXUDP_BEACON_TARGET="" KISS_TCP_TARGET="" MQTT_HOST="" NODES_INTERVAL_SECS=""
unset OTA_FORCE_BRICK

echo "==> building bootloader"
( cd "$BL_DIR" && cargo build --release )
BL_ELF="$BL_DIR/target/$TGT/release/ax25-node-bootloader"

echo "==> building application (blobless)"
( cd "$APP_DIR" && cargo build --release )
APP_ELF="$APP_DIR/target/$TGT/release/ax25-node-fw"

echo "==> objcopy app -> raw .bin (OTA upload payload)"
rust-objcopy -O binary "$APP_ELF" "$OUT/pico-node-app.bin"
cp "$APP_ELF" "$OUT/pico-node-app.elf"

echo "==> building cyw43 BLOBS image"
python3 "$ROOT/scripts/build-blobs.py" \
  "$FW_DIR/43439A0.bin" "$FW_DIR/43439A0_clm.bin" "$FW_DIR/nvram_rp2040.bin" \
  "$OUT/.blobs.bin"

echo "==> STATE-clear page (clean embassy-boot state on flash)"
python3 -c "open('$OUT/.state.bin','wb').write(b'\xff'*4096)"

echo "==> converting to UF2 + combining (bootloader + STATE + BLOBS + app)"
picotool uf2 convert "$BL_ELF"        -t elf                  "$OUT/.bl.uf2"
picotool uf2 convert "$OUT/.state.bin" -t bin -o "$STATE_ADDR" "$OUT/.state.uf2"
picotool uf2 convert "$OUT/.blobs.bin" -t bin -o "$BLOBS_ADDR" "$OUT/.blobs.uf2"
picotool uf2 convert "$APP_ELF"       -t elf                  "$OUT/.app.uf2"
cat "$OUT/.bl.uf2" "$OUT/.state.uf2" "$OUT/.blobs.uf2" "$OUT/.app.uf2" > "$OUT/pico-node-combined.uf2"
rm -f "$OUT/.bl.uf2" "$OUT/.state.uf2" "$OUT/.blobs.uf2" "$OUT/.app.uf2" "$OUT/.blobs.bin" "$OUT/.state.bin"

echo "==> artifacts in $OUT:"
ls -la "$OUT"/pico-node-app.bin "$OUT"/pico-node-combined.uf2 "$OUT"/pico-node-app.elf
( cd "$OUT" && sha256sum pico-node-app.bin pico-node-combined.uf2 pico-node-app.elf > SHA256SUMS && cat SHA256SUMS )
