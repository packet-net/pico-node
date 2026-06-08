# OTA firmware update — design + implementation

*Implemented 2026-06-07 (PR follows the design originally written the same day).
Answers Tom's question: "is any OTA firmware upgrade path possible? I want no
requirement for users to have or use a debug probe."*

## Short answer

**Two separate things, both now true:**

1. **No debug probe is needed to flash.** The released `.uf2`s flash by
   **BOOTSEL + drag-and-drop over USB** — no probe, no soldering. (First flash is
   *two* small files in sequence, for a layout reason explained under "First
   flash" below; on Linux `sudo picotool load` is the easier route.)

2. **Over-the-air (network) update works.** A configured, networked node serves a
   firmware-upload page at **`http://<node-ip>/`**; drop in the raw app image and
   it writes it to a spare flash partition, reboots into it on **trial**, and
   **rolls back automatically** if the new image fails to confirm itself. The
   app image is **~284 KB** (the cyw43 blob is stored separately — see BLOBS
   below), so two copies (A/B) fit comfortably in the Pico W's 2 MB flash with a
   large reserve left over. Verified on hardware (see "Verification" below).

## Architecture

`embassy-boot` A/B scheme. A small **bootloader** (`crates/ax25-node-bootloader`,
`embassy-boot-rp` 0.10) runs first on every boot; it inspects a state partition
and, if the app staged + marked an update, swaps DFU↔ACTIVE before chaining the
active image; if a prior trial never confirmed itself, it reverts. The
**application** (`crates/ax25-node-fw`) is now **always bootloader-chained** — it
is not independently bootable — and does two OTA things (`src/ota.rs`):

- **Marks itself good** early in `main` (`mark_booted_early`), confirming any
  pending trial. Idempotent + wear-free on a normal boot (magic already set).
- **Serves the upload** (`http_task`, STA mode, port 80): `POST /firmware`
  streams the raw image straight into the DFU partition via `FirmwareUpdater`,
  marks it for swap, and resets.

**No watchdog.** A hung trial still self-heals: the next reset finds the
unconfirmed swap and reverts. (A trial that *resets itself* — e.g. a crash that
reboots — auto-reverts within one extra cycle, with no human present. A trial
that hangs hard needs one manual reset/power-cycle to trigger the revert.) We
deliberately omit `WatchdogFlash`: an 8 s watchdog would fight the app's slow
WiFi-join boot and the large OTA erases.

## Flash layout (2 MB Pico W)

Must match between `crates/ax25-node-bootloader/memory.x` and
`crates/ax25-node-fw/memory.x` (`scripts/check-layout.sh` enforces this in CI;
the layout is **frozen** — changing it needs a coordinated bootloader reflash of
every node, and it breaks binary-delta OTA):

| Region | Offset | Size |
|---|---|---|
| BOOT2 + bootloader | 0x10000000 | 24 KB (uses ~9.5 KB) |
| Bootloader state | 0x10006000 | 4 KB |
| **ACTIVE** (running app) | 0x10007000 | **512 KB** |
| **DFU** (staged update) | 0x10087000 | **516 KB** (= ACTIVE + 1 scratch page) |
| **BLOBS** (cyw43 firmware) | 0x10108000 | **256 KB** (flash-once) |
| **APPDATA** (reserved) | 0x10148000 | **720 KB** |
| Config + routing store | 0x101FC000 | 16 KB |

Every byte is allocated — no gaps. The ~284 KB app leaves ~228 KB of headroom in
ACTIVE. The config + NET/ROM routing sectors (top 16 KB, `src/config_store.rs` +
`src/netrom_store.rs`) and APPDATA are at absolute offsets in the full-chip Flash
driver — untouched by OTA, outside the app's FLASH (ACTIVE) region.

### The cyw43 blob lives once (BLOBS), not in A/B

The CYW43439 WiFi firmware (~226 KB) is **not** linked into the app image. If it
were, it would sit in **both** ACTIVE and DFU (a full image is staged into DFU on
every update) — ~452 KB of flash for a byte-identical, rarely-changing blob. So
it lives once in the **BLOBS** region: `scripts/build-blobs.py` lays out a `PBLB`
manifest + the firmware/CLM/NVRAM, flashed once via `pico-node-blobs.uf2`, and
`src/net.rs` reads them at the fixed XIP address `__blobs_start`. Result: the app
image is **~284 KB** (was ~510 KB), OTA payloads are smaller, and the upstream
firmware is rare to change (it changed ~twice in 3 years, and we pin it). A blob
update is therefore a deliberate **BOOTSEL** event (re-flash `pico-node-blobs.uf2`),
not a routine OTA — routine OTA updates code only.

### Clean state on flash (important for upgrades)

The `pico-node-firmware.uf2` includes a **STATE-clear** (a 0xFF page at the
bootloader-state sector). embassy-boot reads that sector to decide whether a swap is pending; a
chip carrying *stale* bytes there (e.g. upgrading from a previous layout where
that address held bootloader code) would corrupt the first swap. Writing a clean
page guarantees "no pending swap" on every BOOTSEL flash. On the bench, always
**full-erase** (`probe-rs download --chip-erase`) when changing the layout, for
the same reason.

**Flash sharing:** there is one `FLASH` peripheral, owned by `config_store`. The
OTA path *takes* it (`config_store::take_flash_for_ota`) and never returns it —
every OTA path ends in a reset, so that's fine.

## Using it

- **Check the running build:** `GET http://<node-ip>/version` → plain-text build
  tag (the crate version by default; override with `OTA_BUILD_TAG` at build time).
- **Update:** browse `http://<node-ip>/`, pick the raw **`pico-node-app.bin`**
  (NOT the `.uf2`), upload. The node writes it, reboots, and swaps. Reconnect in
  ~30 s; `/version` should show the new build.
- **AP mode:** the captive portal owns :80 there, so OTA isn't offered in AP mode
  — you're physically present, so use BOOTSEL.

**Security:** the upload is unauthenticated, like the captive portal — anyone on
the node's LAN can push firmware. Fine for a hobby node on a trusted LAN; gate it
(a token, or signed images via embassy-boot's `_verify` ed25519 support) before
exposing a node to an untrusted network.

## First flash (no probe): two UF2 files

The de-dup puts the cyw43 BLOBS 1 MB above the app (the DFU region sits between
them), and this defeats a *single* combined UF2 for BOOTSEL drag-drop:

- A **multi-region** UF2 (app + blobs far apart) **does not flash** — the RP2040
  BOOTSEL bootrom writes contiguously and **stops at the big address jump**,
  flashing only the part before the gap (the board then boots a bootloader with
  an un-flashed app → no WiFi, blank display). Verified by reading the flash back
  over a probe: everything after the jump was the *old* firmware.
- A **single gap-filled contiguous** UF2 *would* flash, but filling the ~750 KB
  app→blobs gap makes it ~2.6 MB, which **exceeds the RPI-RP2 drive's advertised
  size** — the OS refuses the copy.

So the first flash is **two small, single-run (contiguous) UF2s**, dragged in
sequence — each avoids any address jump and stays well under the size limit:

1. **`pico-node-firmware.uf2`** (~700 KB) — bootloader + a 0xFF STATE-clear +
   app. (The 0xFF fill spans the bootloader-state sector, so an upgrade from a
   different layout can't inherit a stale swap state.)
2. **`pico-node-blobs.uf2`** (~470 KB) — the cyw43 firmware/CLM/NVRAM.

BOOTSEL → drag #1, let `RPI-RP2` vanish → BOOTSEL again → drag #2. The board
won't boot fully until both are on (in between it sits dead — expected). Order
doesn't matter. On **Linux**, `sudo picotool load <uf2>` is the more reliable
path (it writes blocks directly, sidestepping both the drag-drop quirk and the
common "RPI-RP2 mounts read-only" file-manager issue); `picotool load` doesn't
reboot between files, so you can load both then `picotool reboot`.

OTA-over-WiFi is unaffected by all of this — it only ever ships the single
`pico-node-app.bin`.

## Building the artifacts

`scripts/package-ota.sh [outdir]` builds everything (credential-free,
reproducible): `pico-node-app.bin` (the blobless OTA payload),
`pico-node-firmware.uf2` + `pico-node-blobs.uf2` (the two first-flash files),
`pico-node-app.elf`, and `SHA256SUMS`. Internally each UF2 is a single
contiguous region built with `picotool uf2 convert` from a packed `.bin`:
`firmware` = bootloader at the flash base, 0xFF up through the state sector, then
the app at ACTIVE; `blobs` = the `PBLB` manifest + cyw43 firmware
(`scripts/build-blobs.py`) at `0x10108000`.

## Bench notes

Because the app is bootloader-chained AND reads the cyw43 blob from the BLOBS
region, the dev loop needs the **bootloader + BLOBS flashed once**:
`probe-rs download --chip RP2040 <bootloader-elf>` then
`probe-rs download --chip RP2040 --binary-format bin --base-address 0x10108000 blobs.bin`.
Thereafter `cargo run` flashes only the app to ACTIVE. **When changing the
layout, full-erase first** (`--chip-erase`) so the embassy-boot state sector is
clean — stale bytes there corrupt the first swap. (CI is unaffected — it only
link-checks.)

## Verification (on hardware, 2026-06-08)

Proven end-to-end on the bench Pico W — **full chip-erase, all segments flashed +
verified, clean state** (probe used only for the initial flash; updates went over
WiFi):

- **Chained boot:** bootloader → app at ACTIVE → `mark_booted` → full service
  (telnet `M9YYY-9}`, AXUDP, OTA server), **WiFi initialised from the BLOBS
  region** (the de-duplicated cyw43 firmware).
- **Swap:** `/version` = `0.1.0`; `POST /firmware` of a `v2`-tagged image →
  reboot → bootloader swap → `/version` = `v2`, WiFi rejoined, the swapped image
  read BLOBS correctly.
- **Rollback:** from `v2`, OTA'd a deliberately-broken image (resets before
  marking good). The bootloader swapped it in, it self-reset, the bootloader
  reverted — `/version` back to `v2`, no human intervention.
- **Combined UF2:** structurally validated (RP2040 family; segments at
  0x10000000 (bootloader), 0x10006000 (STATE-clear + app), 0x10108000 (BLOBS))
  and byte-identical to the proven probe-flashed images. The literal BOOTSEL
  drag-drop wasn't exercised on the remote bench (no physical button access).

> **Hard-won lesson:** a first attempt iterated three immutable layout changes on
> one chip without erasing between them; stale embassy-boot state + flaky-SWD
> flash corruption made the swap appear broken. Re-tested cleanly (full erase,
> verified flashes), it works. Hence the STATE-clear in the UF2 and the
> full-erase bench rule above.

## Risk + recovery

- Power loss mid-update is safe: ACTIVE is untouched until the bootloader's
  journaled swap; an interrupted DFU write is simply never marked, so nothing
  swaps.
- A bad-but-self-resetting image rolls back automatically; a hard-hung image
  rolls back on the next manual reset.
- A catastrophic image is always recoverable with BOOTSEL + the two UF2 files —
  the same no-probe path users already have.

## See also

`docs/OTA-RADIO.md` — feasibility + design for delivering updates **over packet
radio** (AX.25 / NET/ROM) instead of WiFi, reusing this same DFU/swap/rollback
machinery. Includes measured binary-delta sizes (a typical update is ~10 KB, not
the full image).
