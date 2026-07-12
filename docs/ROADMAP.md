# pico-node — feature roadmap / parity pick-list

Companion to [`PLAN.md`](PLAN.md) (architecture, module map, build/flash/test cycle, hardware gate). **PLAN.md is *how*; this is *what and whether*** — the menu of candidate capabilities, their parity class, their fit on an RP2040, and a running record of what's picked, deferred, or skipped.

pico-node is node **firmware people build a real node with** — so radios (a NinoTNC behind KISS, a Tait driven directly over CCDI, a radio's own FFSK modem, or a *remote* radio reached over a head-end) are core candidate scope, not out of scope.

## Where it stands today (2026-07-12, after the Fable wave)

`crates/ax25-node-core` builds `thumbv6m-none-eabi` `no_std`+`alloc`; the `ax25-node-fw` crate cross-compiles. Health on `main`: **717 core host tests** green, `clippy -D warnings` clean, the `no_std` build clean, both fw `--locked` gates green, and the parity drift-guard passing (0 gaps).

> **The old June "current state" in this doc was stale.** A fresh recon (2026-07-12) found the tree far more complete than recorded — full detail in **[Delivered — 2026-07-12 Fable wave](#delivered--2026-07-12-fable-wave)**. In brief: NET/ROM L4 + NODES origination + INP3 were *already* built (not "read-only"); mod-128 was genuinely missing *at the wire codec* (now shipped); the radio stack was greenfield (now shipped); no shared golden vectors existed (now built).

## Delivered — 2026-07-12 Fable wave

A parallel implementation wave — recon fan-out → 7 isolated git worktrees → merge, each track gated on `clippy -D warnings` + `cargo test` — landed most of tables A–E. Baseline 470 → **632 core tests**; the fw crate now cross-builds.

**Ground-truth correction (recon 2026-07-12)** — the tree had drifted *ahead* of this doc, not behind:
- **NET/ROM L4 transport, NODES origination, and INP3 were already fully built** and byte-identical to C#. What made it look "read-only": [`NetRomService`] is a pure observer; origination / L4 circuits / forwarding are separate components the firmware drives — and, pre-wave, only wired on the AXUDP/LAN path.
- **mod-128 was genuinely MISSING at the wire codec** — `sdl/bridge.rs` was hard-coded modulo-8, silently mis-encoding extended I/S frames past N(S)=7, even though the session tracked `is_extended` and did SABME establishment.
- **The radio stack (Tait CCDI, FFSK, SDM, RSSI) was greenfield** (0 files).
- **No shared golden vectors existed anywhere** — "vector-enforced parity" was aspirational; the corpus + runner + drift-guard had to be built.

### Shipped to pico-node `main` (PRs #53–#60)

| Area | What shipped | PR |
|---|---|---|
| A · mod-128 | extended-control wire codec (fixes the mod-8 mis-encode); mod-128 SREJ falls out of it; `Ax25ParseOptions` strict scaffold; v2.2-preferred connect; carrier-sense seam | #56 |
| A · quirks | session quirks #48 DM-degrade, #9 ack-progress-resets-RC, #13 clamp-SREJ-window → **11/12** of C#'s `Ax25SessionQuirks` | #55 |
| C · radio | Tait CCDI codec + driver (RSSI/PTT/channel + PROGRESS demux) + integer-EMA RSSI tagging + FFSK-transparent | #54 |
| D · SDM | SDM tuning-telegram + meter-report codec (parity-locked; codec only — the link is a follow-up) | #59 |
| C/E · KISS | ACKMODE echo-correlator, NinoTNC status/RSSI parsers + CQBEEP builder, portable reconnect backoff | #58 |
| parity | capability manifest + golden-vector runner (`tests/golden_vectors.rs` **round-trips C#'s golden UI bytes exactly**) | #57 |
| parity CI | cross-stack drift-guard: `scripts/parity-check.mjs` + vendored 50-item C# inventory + `parity-exceptions.json` + self-hosted `.github/workflows/parity.yml` | #53 |
| fw wiring | NODES obsolescence-sweep fix, origination + RX-observe on RF, `kiss_serial` pump+spawn, new Tait CCDI transport — **compile-validated only, no hardware run** (also repaired a fw-build break from #58's new enum variants) | #60 |

### Shipped — "land everything not explicitly deferred" pass (PRs #62–#63)

The In/Yes picks that had slipped to follow-up were then landed:

| Area | What shipped | PR |
|---|---|---|
| D · SDM | **SDM link** (`SdmTuningLink`) over the CCDI driver — receipt-tolerant default (returns on radio-accept; a delivered send with **no** over-air receipt is *not* an error — the TM8110 SDM auto-ack refractory finding), retry-on-reject, sequence dedupe | #62 |
| A · XID | **XID/MDL keystone** — XID info-field TLV codec (spec Fig 4.5–4.6 golden bytes), pre-session XID responder, peer-capability cache, **activates SREJ** (negotiated-SREJ link now emits SREJ — was dormant), `prefer_extended_connect` **default → on** + DM-degrade regression, XID golden vectors | #63 |

This retires the earlier "SREJ / v2.2 shipped but dormant" caveat: SREJ now activates on a negotiated link, and pico dials SABME-first by default. **717 core tests** on `main`; both fw `--locked` gates + drift-guard green.

### 3-way parity mirror (cross-repo — merged)
`interop.yml` (packet.net **#605**) and `parity-check.mjs` + `ci.yml` (ax25-ts **#73**) gained a `--rust` leg so the mirror is a true 3-way gate — C# `Packet.*` ⊆ TS `@packet-net/ax25` ⊆ Rust pico-node, drift failing on whichever side introduces it. Reuses the live C# extraction (single source of truth) against pico-node's manifest/exceptions. **Both merged** (validated locally: three legs green, backward-compatible, and the Rust leg bites on injected drift).

### Decision reconciliations
- **INP3 (decided "Later, leave a seam")** — it turned out *fully built and byte-identical*. Retroactively cargo-gating it would **diverge from the C# reference** (which also ships it always-compiled, runtime-gated). Resolution: **left as-is — present, runtime-gated *off* by default**; the wire form degrades exactly to plain NET/ROM when disabled, which honours "leave a seam" without divergence.
- **`prefer_extended_connect`** shipped default-off in #56 (pending #48), then **flipped to default-on in #63** with a DM-refusing-peer degrade-to-mod-8 regression — pico now dials SABME-first, matching C# `PreferExtendedConnect`.
- **XID / MDL responder + `preConnectXid` cache** (decided In) shipped in #63; **SDM link** (decided Yes) shipped in #62. The one deferred sub-piece is the initiator XID probe (below).

### Still open (follow-ups)
- **Initiator pre-connect XID *probe*** (`NegotiateSrejBeforeConnectAsync` — proactively send our XID *before* we dial, bounded-wait, merge). The XID *responder* + peer-capability cache landed (#63); the initiator half is an inherently async, fw-side multi-step flow left as a clean follow-on — the `mdl::apply_negotiated` merge + `capability.rs`/`connect_planned` dial seam are in place for it.
- **SDL version skew** — Rust `ax25sdl` 0.8.0 vs C# `Packet.Ax25.Sdl` 0.10.0, pico floats on `main`. Recorded in `parity-manifest.toml [sdl]`; pin to a matching `crate-v*` tag. Only bites SDL-*driven* session vectors, not pure codecs.
- **fw bench validation** — everything in #60 is compile-only; sweep timing, on-air NODES visibility, the `kiss_serial` pump under live NinoTNC traffic, and the Tait CCDI transact/demux all need the hardware-bringup session.
- **Optional NET/ROM parity** — L4 payload compression (needs a `no_std` deflate — a dependency decision) and NODESPACLEN per-port fragmentation; both default-off in C#, so interop is unaffected today.

## Parity discipline

- **Build order, every increment: C# reference (`packet-net/packet.net`) → TypeScript (`packet-net/ax25-ts`) → Rust (pico-node).** The C# codec/behaviour is authoritative; the other two mirror it 1:1. ax25-ts is the closest port model (TS→Rust is easier than C#→Rust).
- **SDL state-machine tables** come from the **`ax25sdl`** Rust crate in **`packet-net/ax25sdl`** (`spec/rust`), consumed as a local sibling path-dependency. (Note: `packethacking/ax25spec` is the *prose* spec — PDFs of AX.25/KISS/IL2P — and has **no** Rust crate; the SDL tables live in `packet-net/ax25sdl`. A version pin is a tracked follow-up — see above.)
- **Byte-identical codecs** across all three, enforced by **shared golden vectors** (FNV-1a flow hash, NET/ROM quality formula, NODES vectors, session/wire vectors). As of 2026-07-12 pico-node ships the first vector set + runner + drift-guard (below); the shared corpus grows from here.
- **Rust stays `no_std`-clean:** integer-only maths (the M0+ has no FPU), fixed-capacity const-generic state (no heap maps), `u64` monotonic ticks.
- **Named flags / quirks default to preserving prior behaviour** (mirrors `Ax25ParseOptions` / `Ax25SessionQuirks` / `NetRomParseOptions`).

## Legend

- **Status** — in pico-node today: `HAS` / `PARTIAL` / `NONE` (`check`/`audit` = confirm against the code). *(These are the pre-wave values; see [Delivered](#delivered--2026-07-12-fable-wave) for current.)*
- **Class** — **🔒** parity-locked shared codec/behaviour (golden-vector territory) · **🔧** node feature (parity only if it speaks a wire protocol shared with other pdn stacks).
- **Fit** — feasibility on the RP2040 (264 KB RAM, dual M0+, no FPU, WiFi via CYW43).
- **Rec** — `In` / `Decide` / `Later` / `Skip` (a starting recommendation); **decision** = the call made; a ✅ marks what shipped in the 2026-07-12 wave.

---

## A. Core AX.25 / link-layer parity — catch up the drift since June 4

| feature | status | class | rec | decision |
|---|---|---|---|---|
| **mod-128 (extended) framing** — SABME/extended control octets | NONE (its own flagged follow-up) | 🔒 | In | In · ✅ #56 |
| **v2.2-preferred CONNECT** (SABME-first, SABM fallback) | check | 🔒 | In | In · ✅ #56 codec; **#63 flips default on** (SABME-first) + DM-degrade regression |
| **SREJ-to-BPQ interop** tweaks (the working-SREJ leg) | PARTIAL (has SREJ) | 🔒 | In | In · ✅ codec #56/#55; **activated via XID negotiation #63** (was dormant) |
| **Pre-session XID responder** + mod-8 interlinks + fast-probe fallback | NONE | 🔒 | In | In · ✅ **#63** responder + TLV codec + activation (initiator *probe* = residual) |
| **Per-call preConnectXid / peer-capability cache** dial param | NONE | 🔒 | Decide | In · ✅ **#63** (peer-capability cache + dial seam) |
| **Carrier-sense (CSMA) seam** (`ICarrierSense` parity) | NONE | 🔒/🔧 | In (needed for any keyed modem) | In · ✅ #56 (default always-clear) |
| **figc4.x quirks added after #47** (if any landed) | HAS #38–47 | 🔒 | In (audit) | In (audit) · ✅ #55 adds #48/#9/#13 → 11/12 |

## B. NET/ROM — beyond read-only

| feature | status | class | rec | decision |
|---|---|---|---|---|
| **L4 transport** — CircuitManager, `connect <alias>`, interlink sessions | NONE (read-only only) | 🔒 | In (makes it a *usable* node) | In · ✅ **already built** (recon); RF-wired #60 |
| **NODES origination / broadcast scheduler** | NONE | 🔒 | In | In · ✅ **already built**; sweep-fix + RF origination #60 |
| **INP3** routing overlay | NONE (future, all-stack) | 🔒 | Later (not shipped anywhere yet) | Later, leave a seam · ✅ **already built, runtime-gated off** (not cargo-gated — would diverge from C#) |

## C. Radio integration

| feature | status | class | fit | rec | decision |
|---|---|---|---|---|---|
| **KISS-over-serial to a NinoTNC** (direct UART) | PARTIAL (codec done, transport stubbed) | 🔧 | good | In (planned cap #3) | In · ✅ pump+spawn #60 (compile-only) |
| **Direct Tait CCDI radio control** (RSSI, DCD/carrier-sense, PTT, channel, mode) over a UART | NONE | 🔧 (CCDI is a wire protocol) | good | In — the "radios are integral" core | In · ✅ codec+driver #54, fw transport #60 |
| **Tait FFSK transparent-mode modem** (AX.25 over the radio's own modem, no TNC) | NONE | 🔒 (SLIP-over-FFSK) | good | Decide | In · ✅ #54 |
| **Head-end *client*** (adopt a *remote* radio over TCP + inventory + line-control) | NONE | 🔧 | good (WiFi TCP) | Decide — lets a tiny Pico reach a shared radio | Out |
| **RSSI/SNR per-frame tagging** (RssiTaggingTransport) | NONE | 🔧 | good | In (pairs with CCDI) | In · ✅ #54 |
| **IL2P / FX.25 FEC framing** | NONE | 🔒 | RAM? | Decide (NinoTNC does IL2P today; only if pico drives a bare modem) | Out - done by modem |

## D. Radio coordination / tuning (the SDM stack)

| feature | status | class | rec | decision |
|---|---|---|---|---|
| **SDM side channel** (Tait CCDI short-data telegrams) | NONE | 🔒 (telegram wire form) | Decide | Yes · ✅ codec #59 + **link #62** (receipt-tolerant default) |
| **Mode coordination / TXDELAY-min / station-hail / deviation assist** | NONE | 🔒 (telegrams) 🔧 | Later — nice but heavy; depends on C + SDM | Later |

## E. Node-host services

| feature | status | class | fit | rec | decision |
|---|---|---|---|---|---|
| KISS-over-TCP (net-sim) · AXUDP · telnet console | PARTIAL (codecs done, transports stubbed) | 🔧 | good | In (existing caps 1/2/4) | Needs more breakdown · AXUDP/telnet already whole; KISS-TCP origination+sweep wired #60 |
| **AGWPE server** (TCP) | NONE | 🔧 (wire protocol) | ok | Decide | Defer |
| **RHPv2** (XRouter protocol) | NONE | 🔧 | RAM-heavy | Later | Defer |
| **MQTT frame emitter** (kissproxy-compatible tracing) | NONE | 🔧 | ok | Decide | Defer (telemetry MQTT already present; per-frame emitter deferred) |
| **Web config/monitor panel** | HAS (provisioning/AP panel) | 🔧 | ok | keep as-is | Extend as necessary |
| **OTA self-update** | HAS | 🔧 | ok | keep | keep |
| **APRS** (APRS101) | NONE | 🔒 | ok | Later | Defer |
| **Tailscale sidecar** | NONE | 🔧 | too heavy | Skip | Not relevant |

---

## How deferred / skipped 🔒 items affect the parity CI

**Status: implemented (2026-07-12).** pico-node now carries `parity-manifest.toml` (opted-in / declared-out sets + capabilities), a golden-vector runner (`crates/ax25-node-core/tests/golden_vectors.rs`, auto-run by the existing self-hosted `cargo test`), and a drift-guard (`scripts/parity-check.mjs` + a vendored C# inventory snapshot + `parity-exceptions.json` + `.github/workflows/parity.yml`). The 3-way mirror into packet.net `interop.yml` + ax25-ts is in review (PRs above). The mechanism below is what all of that enforces.

Short version: **deferring a 🔒 feature costs nothing in parity CI, provided the omission is *declared*.** The CI enforces *"what you implement is byte-correct, and what you skip is intentional"* — not *"you implement everything."* **Silence is the only failure mode.** Two complementary mechanisms:

1. **Shared golden vectors → per-stack capability manifest (opt-in).** Each stack declares which vector-sets it participates in (`ax25_mod8`, `ax25_mod128`, `kiss`, `axudp`, `netrom_quality`, `netrom_nodes`, `netrom_l4`, `srej`, `xid`, `sdm_telegram`, …). The CI runs **only the declared sets** against that stack. A deferred 🔒 item = a set pico-node doesn't opt into = **no test, no failure**. A failure fires only when a stack opts *in* and produces different bytes — i.e. a real drift/bug.

2. **Inventory comparison (the ax25-ts `parity-check.mjs` model) → documented exceptions.** It compares named-flag/quirk inventories and fails on **undocumented** gaps. A deliberate omission is a **reviewed exception** (`parity-exceptions.json`) — recorded with a reason, CI green. An *undocumented* gap fails — which is the whole point: it catches *silent* drift, not intentional scope.

Two caveats that keep a "skipped" 🔒 item honest:

- **Negotiation boundary — skipping a feature ≠ skipping the obligation to say so correctly.** mod-128 (via XID), v2.2 CONNECT (via SABME), capabilities (via XID) all *negotiate*. If pico-node doesn't do mod-128 it must still **advertise mod-8 only and negotiate a mod-128 peer *down* correctly**; if it doesn't do SREJ it must **not claim it in XID**. So a skipped feature leaves a small, real parity surface — the *degradation* behaviour — which pico-node **does** implement and **does** opt into. The manifest is therefore finer-grained than feature-on/off: e.g. `ax25_mod128` carries a `negotiate-down-from-mod128` capability. Golden-vector sets should include those degradation vectors.
- **Interop obligation is separate from parity.** Even a fully-declared skip must still *interop* — a BPQ/XRouter peer that offers mod-128 or SREJ must get a clean, spec-legal refusal/fallback, not a hang. That's covered by the shared AXUDP interop harness (LinBPQ/XRouter/direwolf), which pico-node runs once hardware is up, independently of the vector CI.

**Where this landed:** the capability manifest + documented exceptions are live as of the 2026-07-12 wave, and pico-node already shares AX.25 (mod-8 + mod-128), KISS, AXUDP, NET/ROM (quality/NODES/L4), SREJ, and XID vector sets. The manifest grows with each pick in tables A/B/C/D/E; you never have to implement a 🔒 item just to keep CI green.

---

*Living document. The **decision** columns are the calls made; a ✅ marks what shipped (pico-node PRs #53–#63; cross-repo mirror #605/#73, merged). Last updated 2026-07-12 against the fresh recon, the two merged waves, and packet.net's parity discipline.*
