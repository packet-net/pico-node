# Firmware update over radio (AX.25 / NET/ROM) — feasibility + design note

*Written 2026-06-08, answering Tom's question: "could you see it being possible
to implement firmware update over radio (not WiFi/HTTP)?" Theoretical for now —
this is the design + the measurements that say it's worth doing, not an
implementation.*

## Short answer

**Yes — and the dangerous half is already built.** Everything from the WiFi OTA
work (`docs/OTA.md`) is transport-agnostic: the bootloader, the DFU partition,
the `FirmwareUpdater`, the trial-boot + automatic rollback. None of it cares
whether the image bytes arrived over TCP or over RF. "Firmware over radio" is
almost entirely a *transport* problem on top of the same DFU machinery — point
the firmware-writer at an AX.25 stream instead of a socket.

The make-or-break is **payload size on a slow, shared, half-duplex channel**, and
the measurements below show that **binary delta updates make it genuinely
practical**: a typical update is **single-to-low-tens of KB**, not 510 KB.

## The on-device half is mostly done

We already have, in firmware:

- **NET/ROM L4 circuits** — `C M9YYY-9` lands at the node console over a reliable,
  in-order, windowed connection (a byte stream).
- **`kiss_serial`** — the Pico↔NinoTNC↔radio path (built, type-checked, spawned
  when a NinoTNC is wired).
- **`ota::stream_to_dfu`** — takes a byte stream + length and writes it into DFU,
  then verifies + marks for swap.

Wire the L4 reassembled stream into the DFU writer and you have
firmware-over-radio. Swap + rollback safety is identical and free: a dropped or
corrupt transfer is simply never marked, so nothing swaps.

## The real problem: throughput on a shared half-duplex channel

The image is ~510 KB; packet radio is slow and the channel is shared:

| Link | Net throughput (after framing/ACK/CSMA) | Full 510 KB | Compressed (~285 KB, xz) |
|---|---|---|---|
| 1200 baud AFSK | ~50–80 B/s | ~2–3 hours | ~1.3 h |
| 9600 baud (G3RUH / NinoTNC IL2P) | ~400–700 B/s | ~12–20 min | ~7–10 min |

A multi-hour transfer that monopolises a shared channel is a non-starter; even
9600 + compression is a ~10-minute channel hog. Whole-image-over-radio is, at
best, a last resort.

## What makes it practical: delta updates (measured)

Most updates change a little code, not the whole image. We measured real binary
deltas between built images (same flash layout, credential-free release builds,
`bsdiff` — the algorithm embedded delta-OTA systems are built on):

| Change | raw bytes that differ | **bsdiff patch** | xdelta3 |
|---|---|---|---|
| one-line constant fix | 3 | **162 B** | 62 B |
| small feature (+448 B of code) | 454,915 (87% of image) | **10.3 KB** | 30.8 KB |
| larger feature (+1.9 KB) | 449,562 (86%) | **7.4 KB** | 26 KB |
| *(no delta) full image* | — | **510 KB** | xz: 285 KB |

The striking row is the small feature: inserting 448 bytes of code shifts **87%
of the binary** (every address after the insertion moves), so a naïve byte
compare sees almost the whole image as "changed" — yet **bsdiff collapses it to
~10 KB**, because it recognises the moved blocks and encodes the relocation
deltas instead of the bytes. Patch size tracks the *entropy of the genuinely new
code* plus the shift encoding, **not the image size** — so it stays in the
single-to-low-tens-of-KB range no matter how big the firmware is. (The larger
feature came out *smaller* than the small one because its added content was a
low-entropy repeated string — the patch's own compression ate it.)

What that does to airtime:

| Payload | @ 9600 baud (~500 B/s) | @ 1200 baud (~60 B/s) |
|---|---|---|
| full image, compressed (285 KB) | ~9–10 min | ~80 min |
| small-feature delta (10 KB) | **~20 s** | ~3 min |
| one-line-fix delta (162 B) | **sub-second** | ~3 s |

Delta turns an impractical transfer into seconds. **This is the enabling
technique** — without it, radio update isn't worth building; with it, it is.

### The caveat that matters: keep the layout stable

A delta is small only when most of the binary is *recognisably the same*. Two
things blow that up:

- **A flash-layout / load-address change** (like the OTA relink itself, which
  moved the app from 0x10000100 to 0x10009000) shifts *everything* by a constant
  and changes BOOT2 — the patch approaches full-image size. Lesson: the layout is
  now frozen; never churn it for a deliverable that wants delta updates.
- **A toolchain/opt change or a gratuitous global reorder** can reshuffle code
  enough to hurt. Reproducible builds (we have them — the release `.bin` is
  byte-identical across clean builds) keep deltas honest: the only differences
  are the ones you actually made.

For these cases, fall back to shipping the (compressed) full image, or do it over
WiFi/BOOTSEL. The delta path is the common case (a point fix), not the only one.

## Applying a delta on-device

`bsdiff`/`bspatch` (and the embedded-oriented [`detools`](https://github.com/eerimoq/detools),
which has a `no_std` applier and even in-place patching) need: the **old image**
(random-access readable — ours is in ACTIVE, directly XIP-addressable in the
0x10009000 region), the **patch** (received over radio), and a **sequential
write of the new image** (→ DFU). That maps cleanly onto what we have. `detools`
is essentially the proof that embedded delta-OTA is a solved problem; we'd port
or reimplement its applier.

## Two things radio forces that WiFi let us skip

- **Authentication is near-mandatory.** RF is open — anyone can transmit frames
  at your node. An unauthenticated firmware push over the air is a brick/hijack
  vector. embassy-boot already supports the fix: `verify_and_mark_updated()` with
  an **ed25519 signature** (its `_verify` feature). The image (or the patch
  result) travels in clear with a signature appended — which is also the *legal*
  design: ham bands prohibit encryption, but signing (authenticity, not secrecy)
  is fine, and the node still IDs with its callsign as it does today.
- **Resumability.** A 15-minute transfer *will* be interrupted (fade, collision).
  DFU is happy to be written out of order and resumed, so a block-indexed
  protocol (seq → DFU offset, per-block ACK/retry, whole-image hash+sig before
  `mark_updated`) handles drops gracefully. A half-applied DFU is safe — only the
  final verify triggers a swap.

## A concrete design (reusing what exists)

1. **Transport:** a NET/ROM L4 circuit (we already terminate `C <call>` at the
   console). A console command — say `FWLOAD <new-len> <patch-len> <sig>` — flips
   the circuit into binary block-receive mode.
2. **Block protocol over the circuit:** numbered blocks, resumable, with progress
   + selective retry. The L4 layer already gives reliable in-order delivery, so
   this is mostly framing for resume/progress, not a fresh ARQ. (IL2P's FEC on
   the NinoTNC reduces retransmit round-trips on the high-latency link.)
3. **Delta apply:** stream the patch through a `bspatch`/`detools` applier with
   ACTIVE as the source and DFU as the sink. (Or, for a full-image send, the
   existing `stream_to_dfu` unchanged.)
4. **Verify + swap:** `verify_and_mark_updated()` against a baked-in ed25519
   public key → reset → the bootloader swaps → trial → rollback if it misbehaves.

A sysop could then connect to a remote hilltop node **by callsign over RF**, with
no internet at the site, and reflash it with a ~10 KB patch in under a minute.

## Effort / verdict

Feasible, and lower-risk than it sounds because the flash-swap + rollback (the
part that can brick a board) is done and proven. The new work, roughly in order
of payoff:

1. **ed25519 signing/verify** (small; flips on embassy-boot's `_verify`, plus a
   signing step in `package-ota.sh`) — needed before *any* over-air push.
2. **Block transport over the L4 circuit** (a few hundred lines; reuses the
   connector + console).
3. **On-device delta applier** (the meatier piece; port/trim `detools`' `no_std`
   applier) — the thing that makes the payload KB-sized.

Items 1–2 give a working (if slow, full-image) radio update; item 3 is what makes
it something you'd actually use on a shared channel.
