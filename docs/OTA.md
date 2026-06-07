# OTA firmware update — feasibility, design, and plan

*Written 2026-06-07 answering Tom's question: "is any OTA firmware upgrade path
possible? I want no requirement for users to have or use a debug probe."*

## Short answer

**Two separate things:**

1. **Users already need no debug probe today.** The released `.uf2` (v0.3.0+)
   flashes by **BOOTSEL + drag-and-drop over USB** — hold BOOTSEL, plug the
   Pico's USB, drag the UF2 onto the `RPI-RP2` drive. No probe, no soldering.
   That covers initial flash and any re-flash where someone can touch the board.

2. **Over-the-air (network) update is also feasible** — for genuinely remote /
   headless nodes you can't physically reach. It **fits**: the firmware image is
   **~503 KB** (the UF2 looks like ~1 MB only because UF2 stores 256 payload
   bytes per 512-byte block), so two copies (A/B) sit comfortably in the Pico
   W's 2 MB flash. This document is the validated design; it's the next focused
   piece of work (it changes the flash layout + the first-flash story, so it
   wants its own careful, bench-tested PR — a half-built bootloader bricks
   boards).

## Why A/B, and the space budget

The power-fail-safe approach is **`embassy-boot` + `embassy-boot-rp`** (0.10):
a small bootloader manages two equal partitions (ACTIVE + DFU) and a state
sector. The app writes a new image into DFU, marks it, and resets; the
bootloader swaps DFU↔ACTIVE and boots the new image on a **trial** — if the new
firmware doesn't "mark itself good" (we do, early in `main`), the next boot
**rolls back** to the previous image. A bad update self-heals; a truly dead
image is still recoverable via BOOTSEL.

Flash budget (2 MB = 0x10000000–0x10200000), measured against our 503 KB image:

| Region | Offset | Size |
|---|---|---|
| BOOT2 | 0x10000000 | 256 B |
| Bootloader | 0x10000100 | ~64 KB |
| Bootloader state | 0x10010000 | 4 KB |
| **ACTIVE** (running app) | 0x10011000 | **896 KB** |
| **DFU** (staged update) | 0x100F1000 | **896 KB (+scratch)** |
| *(free gap)* | | ~168 KB |
| Config + routing store (existing) | 0x101FC000 | 16 KB |

896 KB partitions for a 503 KB image = ~390 KB headroom for growth. The
existing config/routing sectors (top 16 KB) are untouched.

## Delivery path: the captive portal / HTTP upload

The least-friction UX (and the one that needs no extra tooling): the node's
existing HTTP server gains a **firmware-upload page**. Drag the new `.bin` in a
browser → it streams into the DFU partition via `embassy_boot`'s
`FirmwareUpdater`, marks it, and reboots into the trial. Works from a phone on
the node's AP, or from a browser on the LAN. A `POST` of the raw image (not UF2
— we want the plain binary, no 2× block overhead) is all it takes. An MQTT- or
HTTP-pull trigger ("fetch this URL and update") is a later convenience on top.

## Implementation steps (the next PR)

1. **Bootloader binary** — a tiny crate using `embassy-boot-rp`'s `BootLoader`,
   its own `memory.x` (BOOT2 + bootloader region + the active/dfu/state symbols).
2. **App relink** — `memory.x` FLASH origin moves to the ACTIVE offset; export
   the DFU/STATE symbols for `FirmwareUpdater`.
3. **Mark-good** — early in `main`, `FirmwareUpdater::mark_booted()` so a
   successful boot confirms the trial (else the bootloader rolls back).
4. **Flash sharing** — the single `FLASH` peripheral is already owned by
   `config_store`'s `ConfigService`; the OTA writer reuses it (a `config_store`
   helper hands the DFU/STATE partitions to a `BlockingFirmwareUpdater`), so
   there's one owner and no aliasing.
5. **HTTP upload** — a `POST /firmware` (+ a small upload page) streaming the
   image into DFU, then `mark_updated()` + `SCB::sys_reset()`.
6. **First-flash story** — ship a *combined* bootloader+app UF2 for the initial
   BOOTSEL flash (built by merging the two images), so the out-of-box experience
   is unchanged; OTA takes over after that.
7. **Bench verification** — flash bootloader+app, OTA a deliberately-changed
   build (bumped version string), confirm the swap boots the new image, then
   confirm rollback by OTA-ing a deliberately-broken image and watching the
   bootloader revert. Only claim OTA works once both are observed on hardware.

## Risk + recovery

- Power loss mid-update is safe: ACTIVE is untouched until the bootloader swaps,
  and the swap itself is journaled by `embassy-boot`.
- A bad image that boots-but-misbehaves rolls back automatically (trial/mark-good).
- A catastrophic image (won't boot at all) is recovered with BOOTSEL + the
  combined UF2 — the same no-probe path users already have.
