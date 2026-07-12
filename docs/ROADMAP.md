# pico-node — feature roadmap / parity pick-list

Companion to [`PLAN.md`](PLAN.md) (architecture, module map, build/flash/test cycle, hardware gate). **PLAN.md is *how*; this is *what and whether*** — the menu of candidate capabilities, their parity class, their fit on an RP2040, and a running record of what's picked, deferred, or skipped.

pico-node is node **firmware people build a real node with** — so radios (a NinoTNC behind KISS, a Tait driven directly over CCDI, a radio's own FFSK modem, or a *remote* radio reached over a head-end) are core candidate scope, not out of scope.

## Where it stands today (protocol baseline ~2026-06-04)

Implemented + host-tested in `crates/ax25-node-core` (154 host tests; builds `thumbv6m-none-eabi` `no_std`+`alloc`):
- **AX.25 v2.2 connected-mode** — the full SDL link-layer runtime (SABM/UA/I/RR/REJ/**SREJ**/T1–T3), the figc4.x spec-defect quirks **#38, #40–#45, #47**, off the generated SDL typed tables.
- **KISS** codec, **AXUDP** framing, AX.25 frame/address/callsign codec, **CRC-16/X.25** FCS, telnet/command console.
- **NET/ROM — read-only:** hears NODES broadcasts, builds a fixed-capacity routing table, surfaces it. **No** origination, **no** L4 circuits/`connect`, **no** interlinks.
- Firmware crate (`ax25-node-fw`) cross-compiles; the **binary is at the hardware gate** — the WiFi/CYW43 bring-up + the transport sockets (AXUDP / KISS-TCP / KISS-serial) are stubs that need a physical Pico W + debug probe. UI (connectionless) frames can move end-to-end; connected-mode needs the (now-written) runtime + the transports filled.

## Parity discipline

- **Build order, every increment: C# reference (`m0lte/packet.net`) → TypeScript (`packet-net/ax25-ts`) → Rust (pico-node).** The C# codec/behaviour is authoritative; the other two mirror it 1:1. ax25-ts is the closest port model (TS→Rust is easier than C#→Rust) and is the *only* one of the three currently under CI-enforced parity.
- **SDL state-machine tables** come from **`packethacking/ax25spec`** (the canonical home; formerly `m0lte/ax25sdl`).
- **Byte-identical codecs** across all three, enforced by **shared golden vectors** (FNV-1a flow hash, NET/ROM quality formula, NODES vectors, session/wire vectors).
- **Rust stays `no_std`-clean:** integer-only maths (the M0+ has no FPU), fixed-capacity const-generic state (no heap maps), `u64` monotonic ticks.
- **Named flags / quirks default-off**, preserving prior behaviour (mirrors `Ax25ParseOptions` / `Ax25SessionQuirks` / `NetRomParseOptions`).

## Legend

- **Status** — in pico-node today: `HAS` / `PARTIAL` / `NONE` (`check`/`audit` = confirm against the code).
- **Class** — **🔒** parity-locked shared codec/behaviour (golden-vector territory) · **🔧** node feature (parity only if it speaks a wire protocol shared with other pdn stacks).
- **Fit** — feasibility on the RP2040 (264 KB RAM, dual M0+, no FPU, WiFi via CYW43).
- **Rec** — `In` / `Decide` / `Later` / `Skip` (a starting recommendation; the *decision* column is filled as we go).

---

## A. Core AX.25 / link-layer parity — catch up the drift since June 4

| feature | status | class | rec | decision |
|---|---|---|---|---|
| **mod-128 (extended) framing** — SABME/extended control octets | NONE (its own flagged follow-up) | 🔒 | In | In |
| **v2.2-preferred CONNECT** (SABME-first, SABM fallback) | check | 🔒 | In | In |
| **SREJ-to-BPQ interop** tweaks (the working-SREJ leg) | PARTIAL (has SREJ) | 🔒 | In | In |
| **Pre-session XID responder** + mod-8 interlinks + fast-probe fallback | NONE | 🔒 | In | In |
| **Per-call preConnectXid / peer-capability cache** dial param | NONE | 🔒 | Decide | In |
| **Carrier-sense (CSMA) seam** (`ICarrierSense` parity) | NONE | 🔒/🔧 | In (needed for any keyed modem) | In |
| **figc4.x quirks added after #47** (if any landed) | HAS #38–47 | 🔒 | In (audit) | In (audit) |

## B. NET/ROM — beyond read-only

| feature | status | class | rec | decision |
|---|---|---|---|---|
| **L4 transport** — CircuitManager, `connect <alias>`, interlink sessions | NONE (read-only only) | 🔒 | In (makes it a *usable* node) | In |
| **NODES origination / broadcast scheduler** | NONE | 🔒 | In | In |
| **INP3** routing overlay | NONE (future, all-stack) | 🔒 | Later (not shipped anywhere yet) | Later, leave a seam |

## C. Radio integration

| feature | status | class | fit | rec | decision |
|---|---|---|---|---|---|
| **KISS-over-serial to a NinoTNC** (direct UART) | PARTIAL (codec done, transport stubbed) | 🔧 | good | In (planned cap #3) | In |
| **Direct Tait CCDI radio control** (RSSI, DCD/carrier-sense, PTT, channel, mode) over a UART | NONE | 🔧 (CCDI is a wire protocol) | good | In — the "radios are integral" core | In |
| **Tait FFSK transparent-mode modem** (AX.25 over the radio's own modem, no TNC) | NONE | 🔒 (SLIP-over-FFSK) | good | Decide | In |
| **Head-end *client*** (adopt a *remote* radio over TCP + inventory + line-control) | NONE | 🔧 | good (WiFi TCP) | Decide — lets a tiny Pico reach a shared radio | Out |
| **RSSI/SNR per-frame tagging** (RssiTaggingTransport) | NONE | 🔧 | good | In (pairs with CCDI) | In |
| **IL2P / FX.25 FEC framing** | NONE | 🔒 | RAM? | Decide (NinoTNC does IL2P today; only if pico drives a bare modem) | Out - done by modem |

## D. Radio coordination / tuning (the SDM stack)

| feature | status | class | rec | decision |
|---|---|---|---|---|
| **SDM side channel** (Tait CCDI short-data telegrams) | NONE | 🔒 (telegram wire form) | Decide | Yes |
| **Mode coordination / TXDELAY-min / station-hail / deviation assist** | NONE | 🔒 (telegrams) 🔧 | Later — nice but heavy; depends on C + SDM | Later |

## E. Node-host services

| feature | status | class | fit | rec | decision |
|---|---|---|---|---|---|
| KISS-over-TCP (net-sim) · AXUDP · telnet console | PARTIAL (codecs done, transports stubbed) | 🔧 | good | In (existing caps 1/2/4) | Needs more breakdown |
| **AGWPE server** (TCP) | NONE | 🔧 (wire protocol) | ok | Decide | Defer |
| **RHPv2** (XRouter protocol) | NONE | 🔧 | RAM-heavy | Later | Defer |
| **MQTT frame emitter** (kissproxy-compatible tracing) | NONE | 🔧 | ok | Decide | Defer |
| **Web config/monitor panel** | HAS (provisioning/AP panel) | 🔧 | ok | keep as-is | Extend as necessary |
| **OTA self-update** | HAS | 🔧 | ok | keep | keep |
| **APRS** (APRS101) | NONE | 🔒 | ok | Later | Defer |
| **Tailscale sidecar** | NONE | 🔧 | too heavy | Skip | Not relevant |

---

## How deferred / skipped 🔒 items affect the parity CI

Short version: **deferring a 🔒 feature costs nothing in parity CI, provided the omission is *declared*.** The CI enforces *"what you implement is byte-correct, and what you skip is intentional"* — not *"you implement everything."* **Silence is the only failure mode.** Two complementary mechanisms:

1. **Shared golden vectors → per-stack capability manifest (opt-in).** Each stack declares which vector-sets it participates in (`ax25-mod8`, `kiss`, `axudp`, `netrom-read`, `netrom-l4`, `ax25-mod128`, `sdm-telegram`, …). The CI runs **only the declared sets** against that stack. A deferred 🔒 item = a set pico-node doesn't opt into = **no test, no failure**. A failure fires only when a stack opts *in* and produces different bytes — i.e. a real drift/bug.

2. **Inventory comparison (the ax25-ts `parity-check.mjs` model) → documented exceptions.** It compares named-flag/quirk inventories and fails on **undocumented** gaps. A deliberate omission is a **reviewed exception** (ax25-ts uses `scripts/parity-exceptions.json`) — recorded with a reason, CI green. An *undocumented* gap fails — which is the whole point: it catches *silent* drift, not intentional scope.

Two caveats that keep a "skipped" 🔒 item honest:

- **Negotiation boundary — skipping a feature ≠ skipping the obligation to say so correctly.** mod-128 (via XID), v2.2 CONNECT (via SABME), capabilities (via XID) all *negotiate*. If pico-node doesn't do mod-128 it must still **advertise mod-8 only and negotiate a mod-128 peer *down* correctly**; if it doesn't do SREJ it must **not claim it in XID**. So a skipped feature leaves a small, real parity surface — the *degradation* behaviour — which pico-node **does** implement and **does** opt into. The manifest is therefore finer-grained than feature-on/off: e.g. `"ax25: mod8 + negotiate-down-from-mod128"` is a distinct, testable capability. Golden-vector sets should include those degradation vectors.
- **Interop obligation is separate from parity.** Even a fully-declared skip must still *interop* — a BPQ/XRouter peer that offers mod-128 or SREJ must get a clean, spec-legal refusal/fallback, not a hang. That's covered by the shared AXUDP interop harness (LinBPQ/XRouter/direwolf), which pico-node runs once hardware is up, independently of the vector CI.

**Implication for wiring pico-node in:** the shared-vector job should be built around a **capability manifest + documented exceptions from day one** (the same discipline ax25-ts already uses). That's exactly what lets pico-node be a deliberate, evolving *subset* of the reference without false failures. It's worth wiring in as soon as pico-node shares *any* 🔒 codec — and it already shares AX.25 (mod-8), KISS, AXUDP, and read-only NET/ROM. The manifest then grows with each pick in tables A/B/C/D/F; you never have to implement a 🔒 item just to keep CI green.

---

*Living document. Recommendations are a starting point; the **decision** columns get filled as choices are made (some picked now, some deferred). Grounded 2026-07-09 from the packet.net side against pico-node's `PLAN.md`, the ax25-ts commit history, and packet.net's parity discipline.*
