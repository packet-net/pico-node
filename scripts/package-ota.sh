#!/usr/bin/env bash
# Build the OTA release artifacts (docs/OTA.md):
#   - pico-node-app.bin       raw application image, ~284 KB (the OTA upload
#                             payload — NO cyw43 blob; that lives in BLOBS)
#   - pico-node-firmware.uf2  bootloader + STATE-clear + app, for first BOOTSEL
#   - pico-node-blobs.uf2     cyw43 firmware/CLM/NVRAM, for first BOOTSEL
#   - pico-node-app.elf       application ELF (symbols)
#
# Why TWO BOOTSEL files, not one combined? The de-dup puts the cyw43 BLOBS 1 MB
# above the app (the DFU region sits between them). The RP2040 BOOTSEL bootrom
# stops writing at that big address jump in a single multi-region UF2 (it flashes
# only up to the gap), and a single GAP-FILLED contiguous UF2 exceeds the
# RPI-RP2 drive's advertised size. So we ship two small CONTIGUOUS UF2s, each a
# single run (no jumps), flashed in sequence. (`sudo picotool load <uf2>` works
# too and is the more reliable path on Linux.) OTA over WiFi is unaffected — it
# only ever ships the single pico-node-app.bin.
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
# them). The firmware.uf2 is one contiguous run from the flash base; its 0xFF
# fill spans the bootloader-state sector (0x10006000), giving a clean embassy-boot
# state on every flash — so even UPGRADING from a different layout starts with no
# pending swap (stale bytes there would corrupt the first swap; learned the hard
# way). APP_OFFSET = ACTIVE (0x10007000) - flash base (0x10000000).
APP_OFFSET=0x7000
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

echo "==> objcopy app -> raw .bin (OTA upload payload) + bootloader -> .bin"
rust-objcopy -O binary "$APP_ELF" "$OUT/pico-node-app.bin"
cp "$APP_ELF" "$OUT/pico-node-app.elf"
rust-objcopy -O binary "$BL_ELF" "$OUT/.bl.bin"

echo "==> building cyw43 BLOBS image"
python3 "$ROOT/scripts/build-blobs.py" \
  "$FW_DIR/43439A0.bin" "$FW_DIR/43439A0_clm.bin" "$FW_DIR/nvram_rp2040.bin" \
  "$OUT/.blobs.bin"

echo "==> firmware.uf2 = contiguous bootloader + 0xFF state-clear + app"
python3 - "$OUT/.bl.bin" "$OUT/pico-node-app.bin" "$APP_OFFSET" "$OUT/.fw.bin" <<'PY'
import sys
bl = open(sys.argv[1], "rb").read()
app = open(sys.argv[2], "rb").read()
app_off = int(sys.argv[3], 16)
img = bytearray(b"\xff" * (app_off + len(app)))  # 0xFF fill clears the state sector
img[0:len(bl)] = bl
img[app_off:app_off + len(app)] = app
open(sys.argv[4], "wb").write(img)
print(f"  firmware image: {len(img)} bytes ({len(img)/1024:.0f} KiB)")
PY
picotool uf2 convert "$OUT/.fw.bin"    -t bin -o 0x10000000   "$OUT/pico-node-firmware.uf2"

echo "==> blobs.uf2 = contiguous cyw43 firmware"
picotool uf2 convert "$OUT/.blobs.bin" -t bin -o "$BLOBS_ADDR" "$OUT/pico-node-blobs.uf2"

rm -f "$OUT/.bl.bin" "$OUT/.fw.bin" "$OUT/.blobs.bin"

echo "==> artifacts in $OUT:"
ls -la "$OUT"/pico-node-app.bin "$OUT"/pico-node-firmware.uf2 "$OUT"/pico-node-blobs.uf2 "$OUT"/pico-node-app.elf
( cd "$OUT" && sha256sum pico-node-app.bin pico-node-firmware.uf2 pico-node-blobs.uf2 pico-node-app.elf > SHA256SUMS && cat SHA256SUMS )
