#!/usr/bin/env bash
# Flash-layout drift guard (docs/OTA.md, docs/OTA-RADIO.md).
#
# The A/B partition map is duplicated in two linker scripts:
#   crates/ax25-node-bootloader/memory.x  (ACTIVE / DFU / BOOTLOADER_STATE)
#   crates/ax25-node-fw/memory.x          (FLASH=ACTIVE / DFU / BOOTLOADER_STATE)
# They MUST agree. If they drift, the bootloader swaps to the wrong place (brick)
# and the FirmwareUpdater writes the wrong offsets — and any binary-delta OTA
# (docs/OTA-RADIO.md) silently balloons because the load address moved. The
# layout is FROZEN: changing it needs a coordinated bootloader reflash of every
# node, so this guard fails CI on any accidental edit to one side only.
#
# It also checks DFU stays clear of the persisted config/routing store (top 16 KiB).
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BL="$ROOT/crates/ax25-node-bootloader/memory.x"
APP="$ROOT/crates/ax25-node-fw/memory.x"

# Extract "ORIGIN|LENGTH" (whitespace-stripped) for a named MEMORY region.
field() {
  grep -E "^\s*$2\s*:" "$1" | head -1 \
    | sed -E 's/.*ORIGIN\s*=\s*([^,]+),\s*LENGTH\s*=\s*(.+)$/\1|\2/' | tr -d ' '
}

fail=0
check() { # label bootloader-value app-value
  if [ "$2" != "$3" ]; then
    echo "  MISMATCH $1: bootloader='$2'  app='$3'"; fail=1
  else
    echo "  ok  $1 = $2"
  fi
}

echo "flash-layout drift check (bootloader vs app):"
check "ACTIVE (app FLASH)"     "$(field "$BL" ACTIVE)"           "$(field "$APP" FLASH)"
check "DFU"                    "$(field "$BL" DFU)"              "$(field "$APP" DFU)"
check "BOOTLOADER_STATE"       "$(field "$BL" BOOTLOADER_STATE)" "$(field "$APP" BOOTLOADER_STATE)"

# DFU must end at or below 0x101FC000 — the persisted config + NET/ROM stores
# (top 16 KiB) live there and must never be overlapped by an OTA write.
dfu="$(field "$BL" DFU)"            # e.g. 0x100E9000|900K
dfu_org=$(( $(echo "$dfu" | cut -d'|' -f1) ))
dfu_len_raw=$(echo "$dfu" | cut -d'|' -f2 | sed -E 's/K$/*1024/; s/M$/*1024*1024/')
dfu_len=$(( dfu_len_raw ))
dfu_end=$(( dfu_org + dfu_len ))
store_top=$(( 0x101FC000 ))
if [ "$dfu_end" -gt "$store_top" ]; then
  printf "  MISMATCH DFU end 0x%X overruns the config/routing store at 0x%X\n" "$dfu_end" "$store_top"; fail=1
else
  printf "  ok  DFU end 0x%X <= config/routing store 0x%X\n" "$dfu_end" "$store_top"
fi

if [ "$fail" = 0 ]; then
  echo "layout OK"
else
  echo "FLASH LAYOUT DRIFT — bootloader and app disagree. This would brick the A/B"
  echo "swap and break OTA. Edit BOTH memory.x files together; the layout is frozen."
  exit 1
fi
