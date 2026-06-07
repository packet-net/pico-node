# OTA firmware update — design + implementation

*Implemented 2026-06-07 (PR follows the design originally written the same day).
Answers Tom's question: "is any OTA firmware upgrade path possible? I want no
requirement for users to have or use a debug probe."*

## Short answer

**Two separate things, both now true:**

1. **No debug probe is needed to flash.** The released `.uf2` flashes by
   **BOOTSEL + drag-and-drop over USB** — hold BOOTSEL, plug the Pico's USB, drag
   the UF2 onto the `RPI-RP2` drive. No probe, no soldering.

2. **Over-the-air (network) update works.** A configured, networked node serves a
   firmware-upload page at **`http://<node-ip>/`**; drop in the raw app image and
   it writes it to a spare flash partition, reboots into it on **trial**, and
   **rolls back automatically** if the new image fails to confirm itself. The
   real image is **~510 KB**, so two copies (A/B) fit comfortably in the Pico W's
   2 MB flash. Verified on hardware (see "Verification" below).

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
`crates/ax25-node-fw/memory.x`:

| Region | Offset | Size |
|---|---|---|
| BOOT2 | 0x10000000 | 256 B |
| Bootloader | 0x10000100 | 32 KB (uses ~9.5 KB) |
| Bootloader state | 0x10008000 | 4 KB |
| **ACTIVE** (running app) | 0x10009000 | **896 KB** |
| **DFU** (staged update) | 0x100E9000 | **900 KB** (= ACTIVE + 1 scratch page) |
| *(free gap)* | 0x101CA000 | ~200 KB |
| Config + routing store | 0x101FC000 | 16 KB |

The ~510 KB image leaves ~386 KB of headroom in ACTIVE. The config + NET/ROM
routing sectors (top 16 KB, `src/config_store.rs` + `src/netrom_store.rs`) are at
absolute offsets in the full-chip Flash driver — untouched by OTA and by the
relink (they sit in the gap above DFU, outside the app's FLASH region).

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

## Building the artifacts

- **Bootloader:** `cd crates/ax25-node-bootloader && cargo build --release`
- **App:** `cd crates/ax25-node-fw && cargo build --release` → ELF; the raw
  OTA image is `rust-objcopy -O binary <elf> pico-node-app.bin`.
- **Combined first-flash UF2** (bootloader + app, for BOOTSEL):
  ```
  picotool uf2 convert <bootloader-elf> -t elf bootloader.uf2
  picotool uf2 convert <app-elf>        -t elf app.uf2
  cat bootloader.uf2 app.uf2 > pico-node-combined.uf2
  ```
  UF2 blocks are independent (each carries its own target address + the RP2040
  family id), so concatenation is a valid combined image. `scripts/package-ota.sh`
  automates this.

## Bench notes

Because the app is now bootloader-chained, the dev loop and on-target tests need
the **bootloader pre-flashed once**:
`probe-rs download --chip RP2040 <bootloader-elf>`. Thereafter `cargo run` /
`cargo test` flash only the app/test to ACTIVE and the resident bootloader chains
them. (CI is unaffected — it only link-checks.)

## Verification (on hardware, 2026-06-07)

Proven end-to-end on the bench Pico W (probe used only to flash the bootloader +
the initial app; the updates themselves went over WiFi):

- **Chained boot:** bootloader → relinked app at ACTIVE → `mark_booted` → full
  service (telnet `M9YYY-9}`, AXUDP, OTA server), with the stored flash config
  (callsign/WiFi) preserved through the relink.
- **Swap:** `/version` = `base`; `POST /firmware` of a `v2`-tagged image →
  reboot → bootloader swap → `/version` = `v2`. The new image booted and ran.
- **Rollback:** from `v2`, OTA'd a deliberately-broken image (resets before
  marking good). The bootloader swapped it in, it self-reset, the bootloader
  detected the unconfirmed trial and reverted — `/version` back to `v2`, with no
  human intervention.
- **Combined UF2:** structurally validated (2078 blocks, RP2040 family, segments
  at 0x10000000 and 0x10009000, state sector left erased) and byte-identical to
  the probe-flashed images that booted. The literal BOOTSEL drag-drop wasn't
  exercised on the remote bench (no physical button access).

## Risk + recovery

- Power loss mid-update is safe: ACTIVE is untouched until the bootloader's
  journaled swap; an interrupted DFU write is simply never marked, so nothing
  swaps.
- A bad-but-self-resetting image rolls back automatically; a hard-hung image
  rolls back on the next manual reset.
- A catastrophic image is always recoverable with BOOTSEL + the combined UF2 —
  the same no-probe path users already have.
