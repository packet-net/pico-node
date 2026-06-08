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
check "BLOBS"                  "$(field "$BL" BLOBS)"            "$(field "$APP" BLOBS)"
check "APPDATA"                "$(field "$BL" APPDATA)"          "$(field "$APP" APPDATA)"

# Resolve a "ORIGIN|LENGTH" pair into start/end byte addresses.
addr() { echo "$1" | cut -d'|' -f1; }
len()  { echo "$1" | cut -d'|' -f2 | sed -E 's/K$/*1024/; s/M$/*1024*1024/'; }
end_of() { local f; f="$(field "$BL" "$1")"; echo $(( $(addr "$f") + $(len "$f") )); }
org_of() { addr "$(field "$BL" "$1")"; }

store_top=$(( 0x101FC000 ))     # base of the persisted config + NET/ROM stores

# The map must be contiguous with no gaps/overlaps, in order, up to the store:
#   DFU end == BLOBS start, BLOBS end == APPDATA start, APPDATA end == store.
# Any drift means an OTA write (or the blob load) could land in the wrong place.
contig() { # label end-region start-region
  local e o; e="$(end_of "$2")"; o="$(( $(org_of "$3") ))"
  if [ "$e" -ne "$o" ]; then
    printf "  MISMATCH %s: 0x%X != 0x%X (gap/overlap)\n" "$1" "$e" "$o"; fail=1
  else
    printf "  ok  %s 0x%X\n" "$1" "$e"
  fi
}
contig "DFU end == BLOBS start"     DFU     BLOBS
contig "BLOBS end == APPDATA start" BLOBS   APPDATA
ad_end="$(end_of APPDATA)"
if [ "$ad_end" -ne "$store_top" ]; then
  printf "  MISMATCH APPDATA end 0x%X != config/routing store 0x%X\n" "$ad_end" "$store_top"; fail=1
else
  printf "  ok  APPDATA end 0x%X == config/routing store\n" "$ad_end"
fi

if [ "$fail" = 0 ]; then
  echo "layout OK"
else
  echo "FLASH LAYOUT DRIFT — bootloader and app disagree. This would brick the A/B"
  echo "swap and break OTA. Edit BOTH memory.x files together; the layout is frozen."
  exit 1
fi
