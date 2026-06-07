# Pico-node — plan: a Rust RP2040 / Pico W AX.25 packet node at parity with the C# node host

*Workspace: `/home/tf/pico-node`. Status as of 2026-06-04. This is the living plan for a from-scratch Rust firmware that mirrors the `m0lte/packet.net` C# node host (`Packet.Node.Core`) on Pico W hardware, built on the AX.25 SDL state machine from `m0lte/ax25sdl`.*

This plan was produced **before the hardware arrives** (Pico W + official Raspberry Pi debug probe are on order). It is grounded on three prior research notes in `m0lte/packet.net/docs/research/`: [`pico-w-rust-dev-workflow.md`](../../packet.net/docs/research/pico-w-rust-dev-workflow.md) (the dev-loop/toolchain verdict), [`pico-packet-node.md`](../../packet.net/docs/research/pico-packet-node.md) (the node design — "the work is the runtime, not the tables"), and `codegen-reach.md` (conformance vectors as the drift-proof net). Where this plan and those notes diverge, the divergence is flagged (e.g. the SP-010-in-Rust status).

---

## 0. TL;DR

> **Status update 2026-06-04 (toolchain + SDL runtime landed).** The two §6 blockers and the §8.A environment blocker below are now **RESOLVED**; this TL;DR's original four bullets are kept (struck through inline) for history, followed by where things actually stand. See the §11 amendment log for the full entry.

- **Architecture is settled and proven**: a two-crate workspace — a portable, `no_std`-able logic crate (`ax25-node-core`, now with exactly one dependency: the local `ax25sdl` tables) that is `cargo test`ed on the host today, plus a thin RP2040/Embassy binary (`ax25-node-fw`) that wires it to the WiFi radio, the network stack, and the UART. This mirrors the research note's recommended split and the C# host's module boundaries.
- **Real, tested, hardware-independent code exists now**: the KISS codec, AXUDP framing, the AX.25 frame/address/callsign codec, the CRC-16/X.25 FCS, the telnet/command-prompt layer — **and now the full connected-mode SDL link-layer runtime** (the Rust port of packet.net's `Ax25Session` + dispatcher + guards + subroutines, driven off the generated `ax25sdl` typed tables) — pass **117 host unit tests** offline. The core crate **compiles cleanly for `thumbv6m-none-eabi` in `no_std` + `alloc`**, proving the embedded posture is real.
- **The two §6 SDL-backend blockers are RESOLVED upstream**: `ax25sdl` 0.8.0 is `no_std`-capable (default-on `std` feature) and carries SP-010's typed closed sets (`Ax25Event` / `Ax25Guard` / `Ax25ActionVerb` + `GuardTerm`). `ax25-node-core` consumes it as a **local path dependency** (kept local per Tom — no crates.io pin) and drives the state machine off a clean exhaustive `match`, no string dispatch.
- **The §8.A environment blocker is RESOLVED**: rustup + the `thumbv6m-none-eabi` target + `rust-src`/`llvm-tools` + `flip-link` + `cargo-binutils` are installed to this box. The core crate cross-compiles; the **entire embassy/cyw43/embassy-net/smoltcp firmware dependency stack compiles for thumbv6m** (367-crate tree resolves + builds). The firmware *binary* does not yet link — the only remaining errors are in its own CYW43-radio + transport-socket bring-up stubs, which is the **hardware gate** (no CYW43 emulator; needs a real Pico W + debug probe).

---

## 1. The parity goal (what we are mirroring)

A Rust firmware for the Pico W providing the four capabilities of the C# node host:

| # | Capability | C# source it mirrors | This workspace |
|---|---|---|---|
| 1 | **AXUDP** — AX.25-over-UDP, node↔node over WiFi (BPQ-compatible AXIP/AXUDP) | `Packet.Axudp.AxudpSocket` | `ax25-node-core::axudp` (framing, done+tested) + `ax25-node-fw::transports::axudp` (socket task, stub) |
| 2 | **KISS-over-TCP** — to net-sim (the emulated RF channel) over WiFi | `Packet.Kiss.KissTcpClient` | `ax25-node-core::kiss` (codec, done+tested) + `ax25-node-fw::transports::kiss_tcp` (stub) |
| 3 | **KISS-over-serial** — to a NinoTNC (direct UART, bypassing its USB chip) | `Packet.Kiss.Serial.KissSerialModem` | same `kiss` codec + `ax25-node-fw::transports::kiss_serial` (stub) |
| 4 | **Telnet command console** over WiFi | `Packet.Node.Core.Console.*` | `ax25-node-core::console` (parser/assembler/responses, done+tested) + `ax25-node-fw::transports::telnet` (stub) |
| — | **AX.25 link layer** underneath all of it | `Packet.Ax25` runtime + `Packet.Ax25.Sdl` tables | `ax25-node-core::ax25` (codec, done) + `ax25-node-core::sdl` (loop-exec done; the runtime port is the major remaining work, blocked on §6) |

**Capability 3 hardware note (carried from the brief):** the RP2040 cannot be a USB host *and* a USB-serial device at once, so we do **not** drive the NinoTNC over USB. We wire the Pico's UART directly to the NinoTNC's UART pins (bypassing its on-board USB-serial bridge). The KISS codec is identical to the TCP path — only the byte source differs. This is the planned, supported path and is reflected in `transports::kiss_serial`.

---

## 2. Architecture summary

```
                 ┌───────────────────────────────────────────────────────┐
                 │                    ax25-node-fw (binary)                │
                 │            thumbv6m-none-eabi · Embassy · no_std        │
                 │                                                         │
   2.4GHz WiFi ──┼─ cyw43 + cyw43-pio ─ embassy-net ─┬─ transports::axudp │ cap 1
                 │     (PIO-SPI, IRQ)    (UDP/TCP)    ├─ transports::kiss_tcp  cap 2
                 │                                    └─ transports::telnet   cap 4
        UART ────┼─ embassy-rp UART ───────────────── transports::kiss_serial cap 3
                 │                                            │            │
                 │   embassy-time (T1/T2/T3) ── session (SDL runtime port) │
                 │   defmt-rtt / panic-probe (diagnostics over SWD)        │
                 └──────────────────────────┬──────────────────────────────┘
                                            │ depends on (no_std + alloc)
                 ┌──────────────────────────┴──────────────────────────────┐
                 │              ax25-node-core (library)                     │
                 │        portable · no_std-able · ZERO external deps        │
                 │                                                           │
                 │  kiss/      encoder · decoder · frame (SLIP framing)      │
                 │  axudp/     datagram encode/decode (+ optional CRC FCS)   │
                 │  ax25/      callsign · address · frame codec              │
                 │  crc/       CRC-16/X.25 (the AX.25 FCS)                   │
                 │  console/   line · command · service · connection-trait   │
                 │  sdl/       loop_exec (done) + the runtime port (TODO)    │
                 │                                                           │
                 │  ← all of this is host-tested with `cargo test` today →   │
                 └───────────────────────────────────────────────────────────┘
                                            │ will depend on (once §6 clears)
                 ┌──────────────────────────┴──────────────────────────────┐
                 │   m0lte/ax25sdl  spec/rust  (generated SDL tables)        │
                 │   ~243 v2.2 transitions + figc4.7 subroutines as          │
                 │   &'static data — NOT YET no_std / published / typed (§6) │
                 └───────────────────────────────────────────────────────────┘
```

**Why this split** (from research note §3.1): the overwhelming majority of the work is portable logic over bytes and enums — it never needs a board. Keeping it in a `no_std`-able, dependency-free library lets the entire protocol be developed and regression-guarded with plain `cargo test` on x86-64, instantly, hands-free, with full `std` debugging — *before any hardware exists and before any embedded toolchain is installed*. The firmware crate is deliberately thin: it owns only the silicon and the radios.

**Why async / Embassy** (research §2.2, §7): the WiFi driver `cyw43` is an Embassy crate and `embassy-net` is the only mature no-alloc async TCP/IP stack for the Pico W; async tasks are the natural model for the multi-source event pump (WiFi packets + N session timers + a UART). `AxudpSocket` maps 1:1 onto `embassy_net::udp::UdpSocket`.

---

## 3. Workspace layout

```
pico-node/
├── Cargo.toml                       workspace (members = [ax25-node-core] only;
│                                    ax25-node-fw is EXCLUDED — see note in file)
├── docs/
│   └── PLAN.md                      this document
└── crates/
    ├── ax25-node-core/              portable, no_std-able, ZERO deps. Host-tested.
    │   ├── Cargo.toml               features: default=["std"], std, alloc
    │   └── src/
    │       ├── lib.rs               #![cfg_attr(not(feature="std"), no_std)]; forbid(unsafe)
    │       ├── crc.rs               CRC-16/X.25 (4 tests)
    │       ├── kiss/                encoder.rs · decoder.rs · frame.rs · mod.rs (22 tests)
    │       ├── axudp/mod.rs         datagram encode/decode + CRC FCS (4 tests)
    │       ├── ax25/                callsign.rs · address.rs · frame.rs · mod.rs (35 tests)
    │       ├── console/             line.rs · command.rs · service.rs · connection.rs · mod.rs (29 tests)
    │       └── sdl/                 mod.rs (blocker doc) · loop_exec.rs (7 tests)
    └── ax25-node-fw/                STANDALONE, workspace-excluded RP2040 binary.
        ├── Cargo.toml               embassy/cyw43/embassy-net/defmt/probe deps DECLARED (planning)
        ├── .cargo/config.toml       target = thumbv6m; runner = probe-rs run; flip-link
        ├── memory.x                 RP2040 BOOT2 + 2MB FLASH + 264KB RAM
        ├── build.rs                 emits memory.x to the linker search path
        ├── rust-toolchain.toml      stable + thumbv6m-none-eabi + rust-src + llvm-tools
        └── src/
            ├── main.rs              #[embassy_executor::main]; spawns the 4 transports + session timer
            ├── config.rs           NodeConfig shape (mirrors Packet.Node.Core.Configuration) — stub loader
            ├── net.rs              cyw43 + embassy-net bring-up — stub
            ├── session.rs         per-peer SDL session array + timer task — stub
            └── transports/        axudp.rs · kiss_tcp.rs · kiss_serial.rs · telnet.rs — stubs
```

The firmware modules are gated `#[cfg(target_os = "none")]` so the crate is structurally complete but a stray host `cargo check` against it won't error before the embedded deps exist.

---

## 4. What is implemented + host-tested now (zero hardware, zero installs)

All ported faithfully from `m0lte/packet.net`, all passing `cargo test` offline. **87 tests, 0 failures, 0 warnings.** The core crate also builds clean under `--no-default-features --features alloc` (the `no_std` posture).

| Module | Ports (C#) | Key behaviours covered by tests |
|---|---|---|
| `crc` | `Packet.Core.Crc16Ccitt` | `"123456789"` → `0x906E`; empty → `0x0000`; message-independent self-check residue; FCS byte order (low-first) |
| `kiss::encoder` | `KissEncoder` | FEND framing; port in high nibble; FEND/FESC escaping; **command-byte escaping** (port 12 + Data = 0xC0 collision); too-small buffer; port-range reject |
| `kiss::decoder` | `KissDecoder` | single/multi frame; unescape; **split chunks**; **split escape across chunks**; empty-interframe-FEND drop; lenient malformed escape; **encode↔decode round-trip with all-escape payload** |
| `axudp` | `AxudpSocket` framing | FCS-less round-trip (LinBPQ form); CRC-FCS round-trip + validation (XRouter form); FCS low-byte-first; corrupted-FCS not reported valid |
| `ax25::callsign` | `Packet.Core.Callsign` | parse with/without SSID; upcasing; strict rejects (empty, too-long, SSID>15, bad chars); display round-trip |
| `ax25::address` | `Packet.Core.Ax25Address` | encode/decode round-trip; char `<<1` shift; SSID-octet layout (`0x60` reserved, C/H, extension); trailing-space trim; known `0xE0` vector |
| `ax25::frame` | `Packet.Ax25.Ax25Frame` | UI/I/S frame round-trips; classification; command/response C-bits; **extension-bit address-field termination**; digipeater chains; PID presence rules; P/F bit; short-frame reject |
| `console::line` | `LineAssembler` | split on CR/LF/CR-LF; CR-LF coalescing across chunks; empty lines; backspace editing; over-long truncation + tail drop; chunked assembly |
| `console::command` | `NodeCommand` + `NodeCommandParser` | full+abbrev verbs; connect arg parsing; malformed/unknown classification; **totality on invalid UTF-8 + over-long input** (the fuzz contract) |
| `console::service` | `NodeCommandService` | per-command responses; **CR vs CR-LF newline policy**; banner; help/info/nodes text; disconnect-on-Bye; connect-then-relay signalling |
| `console::connection` | `INodeConnection` | the async transport-agnostic trait (Embassy-usable, async-fn-in-trait) |
| `sdl::loop_exec` | `SdlLoopExecutor` | test-at-head/test-at-tail `LoopRange` expansion; zero-iteration while; do-while-at-least-once; multi-action bodies; **1024-iteration safety cap** |

### Two real bugs the host loop caught immediately (the loop earning its keep)
1. **AX.25 address-field extension-bit walk was inverted** — decode looped while the E-bit was *set* instead of *clear*; no frame with a source/digi chain would have parsed. Fixed to mirror the C# `while (!lastAddress.ExtensionBit)`, plus normalising the positional E-bit out of the logical frame so a constructed frame round-trips equal to its decode.
2. **CRC self-check residue constant** was a copy-paste of the wrong variant's value; corrected to assert the (empirically verified, message-independent) residue for this low-byte-first FCS.

---

## 5. SDL integration — the link-layer runtime

**The differentiator** (research `pico-packet-node.md`): run the *same* generated AX.25 v2.2 SDL state machine the C# and TS stacks run, proving link-layer parity across hardware classes from one spec source. `m0lte/ax25sdl` already emits a **Rust** backend (`spec/rust/`): the ~243 transitions + figc4.7 subroutines as `pub static … : StatePage` tables of `&'static` data (`TransitionSpec`, `ActionStep`, `SubroutinePath`, `LoopRange`). The tables are inert data — ideal for embedding.

**The work is the *runtime that walks them*** — in C# ~6.2k LOC (`ActionDispatcher`, `Ax25Session`, `GuardEvaluator`, `SubroutineRegistry`, `SdlLoopExecutor`, frame codec, `Segmenter`, timers). This workspace has ported the standalone, table-shape-only pieces already (`sdl::loop_exec` ← `SdlLoopExecutor`; the frame codec ← `Ax25Frame`). The rest — the dispatcher `match`, guard evaluation, the session walk loop, the subroutine registry, the segmenter, the integerised timers — is the major remaining port, and it is gated on §6.

**Integerisation (research §3):** the only floating-point in the C# runtime is two timer formulas (`SRT` IIR smoothing; `T1V` linear backoff). On the no-FPU M0+ these must be integer math (`7*srt/8 + sample/8`, `rc*250 + srt*2` in ms). The tables carry these as opaque verb strings (no arithmetic), so no codegen change is needed — the port just implements them in integers. A one-page shared "runtime integerisation" note across C/Rust/C#/TS is recommended so conformance vectors involving T1V/SRT don't diverge.

**mod-128 buffering policy (research §6):** "fully v2.2 compliant" must be paired with a bounded N1/k policy (via XID + config), or the mod-128 defaults (k=32, N1=2048) imply ~256 KB/session and blow the 264 KB SRAM. The node should advertise a small window (recommend k≤8–16, N1≤256) and stay fully mod-128-*capable*. The fixed `session::MAX_SESSIONS` array (no LRU dict) reflects this.

---

## 6. BLOCKERS — consuming the real `ax25sdl` Rust tables

These are **dependency/sequencing blockers**, not blockers on the host work already done. They must clear (upstream in `m0lte/ax25sdl`) before `ax25-node-core::sdl` can wire in the real tables. **Do not work around them by hand-copying or hand-typing the tables — that defeats the single-source-of-truth parity claim.** Raise them against `m0lte/ax25sdl`.

1. **The generated Rust crate is not `no_std`.** `spec/rust/src/lib.rs` and `types.rs` carry no `#![no_std]`. The *content* is `&'static str` / `&'static [...]` with no `Vec`/`String`/`Box`/`HashMap` (verified) — so it is no_std-*compatible* — but the attribute + a `default = ["std"]` feature (for the existing test harness) have to be added upstream before it builds for `thumbv6m-none-eabi`. This is the research note's "ax25sdl Phase-0" task. **Small, mechanical upstream change.**

2. **The generated Rust crate is not published.** `spec/rust/Cargo.toml` is `publish = false` — it is a CI build/test target, not a crates.io artifact. To depend on it, either (a) consume it via a **path/git dependency** (works immediately once it's no_std — fine for this firmware), or (b) publish it (cleaner long-term; mirrors how `Packet.Ax25.Sdl` and the npm `ax25sdl` package are consumed). Recommend a `git` dependency to start, publish later.

3. **The Rust backend is still stringly-typed (verbs AND guards/events).** This is the **most important** finding and it **diverges from the memory note** that said "SP-010 has shipped in the ecosystem". It has shipped in the **C# and TS** backends only (ax25sdl ADR-0002, dated 2026-06-03: `Ax25ActionVerb`, `Ax25Guard`, `Ax25Event` typed closed sets). The **Rust emitter (`RustEmitter.cs`) was not migrated** — verified: it still writes `verb: "V(s) := V(s) + 1"`, `guard: "peer_busy == false"`, `on: "..."` as raw `&'static str`. Consequence for the runtime port: it must either ship a **string-expression parser + string-keyed dispatch** on the M0+ (works — it's what the C#/TS runtimes did pre-SP-010 — but wasteful in flash and cycles, and loses the compile-time exhaustiveness that caught real bugs), or hand-maintain an enum mapping that **drifts** from the codegen. **The clean fix is upstream: extend the Rust emitter to emit the same typed enums** SP-010 already produces for C#/TS, so the Rust dispatcher is an exhaustive `match`. This is shared work that strictly improves the Rust backend and is load-bearing for embedding.

**Recommended upstream sequencing in `m0lte/ax25sdl`** (Phase-0, before the runtime port): (a) add `#![no_std]` + `std` feature to `spec/rust`; (b) port SP-010 typed verb/guard/event closed sets to the Rust emitter; (c) add a build flag to strip transcription `notes` from the embedded build (citations are already absent from the Rust tables — they're empty `&[]`). Then this workspace adds a `git`/path dep on `ax25sdl` and the `sdl` runtime port proceeds against typed tables.

---

## 7. The hands-free dev cycle (when the board + probe arrive)

Three loops, in order of how often they run (research §4.5):

### Loop A — host `cargo test` (the dominant loop, zero hardware, runs now)
```
edit ax25-node-core → cargo test → green/red in <1s, full std backtraces
```
This is where ~all correctness lives. It needs nothing physical and is already working. An autonomous agent lives here. Add `proptest`/fuzz + the cross-language conformance vectors here once the SDL runtime lands.

### Loop B — cross-build + size gate (zero hardware; needs the toolchain from §8)
```
cargo build --manifest-path crates/ax25-node-fw/Cargo.toml --release
cargo size / cargo bloat        # catch std/soft-float/size regressions
```
Catches accidental `std` linkage, soft-float pulled in by a stray `as f64`, and flash bloat — without a board. Gate it in CI.

### Loop C — real board via probe-rs (the only loop that validates WiFi; needs the board)
```
cargo run --manifest-path crates/ax25-node-fw/Cargo.toml --release
  → probe-rs flashes ELF over SWD → resets → streams defmt/RTT logs → exit code
```
With `runner = "probe-rs run --chip RP2040"` (already in `.cargo/config.toml`), `cargo run` flashes and streams logs in one command — **no BOOTSEL, no drag-drop, no replug**. The board lives permanently wired to the probe; the loop is hands-free after the one-time wiring. On-target tests use **`embedded-test`** (libtest-compatible, device-reset between cases, async support) as the `cargo test` runner — already declared as a dev-dependency.

**The WiFi truth (research §3.4, load-bearing):** *no emulator emulates the CYW43 WiFi.* Wokwi's WiFi sim is ESP32-only; rp2040js and Renode have no CYW43 model. So the headline AXUDP-over-WiFi tier (capability 1) **cannot be emulated** — it must run on a real Pico W. Emulation (`wokwi-cli`) is still a useful hands-free CI pre-filter for the *non-WiFi* firmware paths (boot, GPIO, the UART/KISS path), but it is not a substitute for Loop C on the radio.

### Diagnostics (research §5)
`defmt` + `defmt-rtt`, decoded by `probe-rs`, is the workhorse: structured, level-filtered (`DEFMT_LOG`) log lines on stdout over SWD, with `panic-probe` printing panic location/message and halting. `flip-link` turns stack-overflow-into-silent-corruption into a clean fault. GDB-over-SWD is available for interactive stepping but is the human's tool; the agent's surface is defmt logs + exit codes.

---

## 8. CONSOLIDATED PACKAGE / TOOL / CRATE APPROVAL LIST

> **This is the gate.** Nothing below is installed. The host work in §4 needed none of it. Everything below is required to build/flash the firmware (Loops B + C). Grouped by what it unlocks.

### A. Rust toolchain manager + cross-compile target (REQUIRED for any firmware build — Loop B)
The environment has a Debian-packaged `rustc`/`cargo` 1.93.1 with **no `rustup`**, only the `x86_64` std, and **no `thumbv6m-none-eabi` core and no `rust-src`** — so cross-compiling is currently impossible (verified: `rustc --target thumbv6m-none-eabi` fails `E0463: can't find crate for core`; `-Z build-std` needs nightly *and* `rust-src`, neither present).

1. **`rustup`** (the toolchain manager) — to install targets/components. *Why:* the system rustc can't add targets. **Apt alternative:** Debian ships some `rust-*-thumbv6m` cross packages, but `rustup` is the standard and what `rust-toolchain.toml` expects.
2. **rustup target `thumbv6m-none-eabi`** (`rustup target add thumbv6m-none-eabi`) — the RP2040 core's precompiled `core`/`alloc`. *Why:* without it nothing links for the M0+.
3. *(pulled in by rust-toolchain.toml)* components **`rust-src`** + **`llvm-tools`** — `llvm-tools` is needed by `cargo size`/`cargo-binutils`; `rust-src` is belt-and-braces for any build-std fallback.

### B. Cargo subcommands + linker for the dev loop (REQUIRED for Loop C; B wants cargo-binutils)
4. **`probe-rs` / `probe-rs-tools`** (provides `probe-rs run`, `cargo-embed`, `cargo-flash`) — *the* flash+log tool; it is the `runner` in `.cargo/config.toml`. **Install:** `cargo install probe-rs-tools` (a non-trivial network build — flagged) *or* the prebuilt installer from probe.rs. **Without it there is no hands-free loop.**
5. **`flip-link`** (`cargo install flip-link`) — the linker referenced in `.cargo/config.toml` (zero-cost stack-overflow protection). *Either install it, or I drop the `-C linker=flip-link` line* (it's a hardening nicety, not strictly required to boot).
6. **`cargo-binutils`** (+ the `llvm-tools` component) — provides `cargo size` / `cargo nm` / `cargo objdump` for the size gate (Loop B). Optional-but-recommended.

### C. Embedded crates the firmware declares (fetched on first `cargo build` of ax25-node-fw)
7. The crate trees pinned in `crates/ax25-node-fw/Cargo.toml`: **`embassy-executor`, `embassy-rp`, `embassy-time`, `embassy-sync`, `embassy-futures`, `cyw43`, `cyw43-pio`, `embassy-net`, `cortex-m`, `cortex-m-rt`, `defmt`, `defmt-rtt`, `panic-probe`, `static_cell`, `embedded-alloc`** (+ dev-dep **`embedded-test`**). *These are declared (planning) but not fetched.* Building the firmware triggers a **large network crate build** — per the brief's hard constraint, flagging rather than doing it. **Approval = "yes, fetch/build the embassy stack."**

### D. System packages possibly needed by probe-rs on Linux (REQUIRED for Loop C hardware)
8. **`libudev`/`pkg-config`** (build-time for probe-rs) and a **udev rule** giving non-root access to the CMSIS-DAP probe (`/etc/udev/rules.d/`). Standard probe-rs setup. *Why:* probe-rs talks to the probe over USB.

### E. Spec-side upstream work (NOT a package — tracked separately, see §6)
9. `m0lte/ax25sdl` Rust backend: add `#![no_std]` + `std` feature; port SP-010 typed enums to `RustEmitter`; (optionally) publish the crate. **This is a code change in another repo, gated on Tom's say-so — raised here as a dependency, not an install.**

**Minimum to make the firmware *compile* (Loop B):** A1, A2, A3, then C7. **Minimum to *flash + log* (Loop C):** + B4, D8 (and the physical rig in §9). **To wire the real SDL tables:** + E9.

---

## 9. "When the hardware arrives" checklist

> **The hardware has arrived (2026-06-07).** The operational, self-contained bring-up runbook for the session driving the board is **[`docs/HW-BRINGUP.md`](HW-BRINGUP.md)** — it supersedes the sequence below as the working document (this section stays as the original planning context). See the §11 entry of the same date.

**Hardware to procure / assemble (research §6.2, ~£20):**
- [ ] 1× Raspberry Pi **Pico W** (the target — RP2040 + CYW43439).
- [ ] 1× **debug probe**: the official Raspberry Pi Debug Probe (recommended, packaged SWD+UART) *or* a 2nd Pico flashed with `debugprobe`.
- [ ] Jumper wires: 3 min (GND, SWCLK→probe, SWDIO→probe); +2 for the probe UART ↔ target UART if wanted.
- [ ] 2× USB cables to the host (or power the target from the probe's VSYS).
- [ ] For capability 3: a **NinoTNC** with its UART pins accessible, wired Pico-UART↔NinoTNC-UART (TX↔RX, GND), at the NinoTNC's KISS baud.
- [ ] A 2.4 GHz AP in range + the AXUDP/net-sim peers reachable on that LAN (the `net-sim` RF lab + a `packet.net`/LinBPQ endpoint).

**One-time setup:**
- [ ] Install the §8 toolchain (A) + dev tools (B) + system deps (D); add the udev rule.
- [ ] Flash `debugprobe` to the probe Pico (BOOTSEL + drag UF2), once.
- [ ] Wire the 3 SWD jumpers; plug both USBs in; leave assembled.
- [ ] `probe-rs list` confirms the probe is seen.

**Bring-up sequence (each step is a green light before the next):**
- [ ] **Loop B works:** `cargo build --manifest-path crates/ax25-node-fw/Cargo.toml --release` compiles for thumbv6m (resolve any embassy API drift vs the pinned versions; refresh pins from a live embassy checkout).
- [ ] **Loop C works (no radio):** a minimal `cargo run` blinky/`defmt::info!` flashes over SWD and streams a log line. Confirms the probe + memory.x + linker scripts + flip-link.
- [ ] **WiFi up:** fill `net.rs` from the embassy `wifi_*` examples (vendoring the cyw43 firmware/CLM blobs — check their licence before committing); join the AP; DHCP lease logged.
- [ ] **Capability 1 (AXUDP):** fill `transports::axudp`; exchange a UI frame with a peer over AXUDP; then a connected-mode SABM/UA round-trip. *This is the headline parity demo.*
- [ ] **Capability 4 (telnet):** fill `transports::telnet`; telnet in, see the banner/prompt, run `I`/`N`/`H`, `C <call>` relay, `B`.
- [ ] **Capability 2 (KISS-TCP):** fill `transports::kiss_tcp`; connect to net-sim; pass frames.
- [ ] **Capability 3 (KISS-UART):** fill `transports::kiss_serial`; KISS to the NinoTNC over direct UART.
- [ ] **On-target tests:** stand up `embedded-test`; run a SABM/UA + I-frame conformance scenario on the real M0+.
- [ ] **Parity proof:** run the same interop harness (vs LinBPQ/XRouter/direwolf over AXUDP) that `packet.net`/`ax25-ts` use, against the Pico.

**Prerequisite for the link layer to be real:** the §6 blockers cleared and `ax25-node-core::sdl` wired to the `ax25sdl` Rust tables with the runtime port written. Until then capabilities 1–4 can move *UI frames* (connectionless) end-to-end, but connected-mode (SABM/I/RR/REJ/SREJ/T1-T3) needs the runtime port.

---

## 10. Blockers summary (the gates, in one place)

| Blocker | Kind | Blocks | Owner / resolution |
|---|---|---|---|
| No `rustup` / no `thumbv6m` core / no `rust-src` | environment | building the firmware at all (Loops B, C) | install §8.A (Tom approval) |
| `probe-rs` + `flip-link` + system udev deps not installed | environment | flashing + logging (Loop C) | install §8.B, §8.D (Tom approval) |
| embassy/cyw43/embassy-net crate trees not fetched | environment (network build) | building the firmware | approve §8.C fetch (Tom) |
| `ax25sdl` Rust crate not `no_std`, not published | upstream code | wiring the real SDL tables | small change in `m0lte/ax25sdl` (§6.1/6.2) |
| `ax25sdl` Rust backend still stringly-typed (SP-010 not in Rust) | upstream code | a clean typed-`match` runtime port | port SP-010 to `RustEmitter` (§6.3) |
| Pico W + probe + NinoTNC not yet in hand | hardware | Loop C / on-air | arriving later (§9) |

None of these blocked the **87 passing host tests** or the `no_std` build — the host-testable parity work is done and green now.

---

## 11. Amendment log

- **2026-06-04 — initial plan + scaffold.** Stood up the `/home/tf/pico-node` workspace; ported and host-tested the KISS codec, AXUDP framing, AX.25 frame/address/callsign codec, CRC-16/X.25, the SDL loop-executor, and the full console/command layer from `m0lte/packet.net` (87 tests, 0 fail, 0 warn; core also builds `no_std`+`alloc`). Scaffolded the `ax25-node-fw` Embassy binary (deps declared, modules stubbed, `.cargo`/`memory.x`/`build.rs`/`rust-toolchain.toml` in place). Confirmed `m0lte/ax25sdl` emits a Rust backend but found it (a) not `no_std`/unpublished and (b) **still stringly-typed — SP-010 typed sets are C#/TS-only, not Rust** (diverges from the prior memory note; see §6). Documented the toolchain blocker (no rustup / no thumbv6m core) and the consolidated install approval list (§8).

- **2026-06-04 — toolchain installed + the §6/§8.A blockers cleared + the connected-mode SDL runtime ported.** Three things changed since the entry above, all of which invalidate the "two hard blockers + one environment blocker" framing it recorded.

  *Toolchain (§8.A, Phase 1).* Installed rustup (stable 1.96.0) to this box with the `thumbv6m-none-eabi` target + `rust-src` + `llvm-tools`, plus `cargo install flip-link cargo-binutils`. Deliberately did **not** install `probe-rs-tools` (hardware-flashing only; needs libudev) — deferred to the hardware tier. No system/apt packages were needed; non-interactive sudo was available but unused. The core crate now cross-compiles for thumbv6m (`--no-default-features --features alloc`), so the §8.A / §10 environment blocker is gone.

  *§6 blockers RESOLVED upstream.* The prior entry recorded the `ax25sdl` Rust backend as not-`no_std` and stringly-typed (SP-010 C#/TS-only). That is **no longer true**: `m0lte/ax25sdl` `main` is at crate 0.8.0 (ADR-0003) — `#![no_std]` behind a default-on `std` feature, with SP-010's typed closed sets (`Ax25Event`/`Ax25Guard`/`Ax25ActionVerb` + `GuardTerm`/`ActionStep`/`TransitionSpec`) emitted for Rust. `ax25-node-core` now depends on it via a **local path dependency** (`../../../ax25sdl/spec/rust`, `default-features = false`) — kept local per Tom, no crates.io publish dependence. The earlier prediction (in the prior memory note) that SP-010 had shipped ecosystem-wide turned out correct for Rust after all; this plan's §6, written before the 0.8.0 cut, was the stale view.

  *The SDL runtime (Phase 2, the major remaining port).* `ax25-node-core::sdl` is now the full Rust port of packet.net's `Ax25Session` + `ActionDispatcher` + `GuardEvaluator` + `SubroutineRegistry` + `SdlLoopExecutor`, consuming the generated `ax25sdl` data_link tables (6 figc4.x states + the figc4.7 subroutines) off a clean exhaustive `match` over the typed enums — no string dispatch anywhere. New modules: `context` (SessionContext), `event` (runtime Event + typed `to_sdl` mapping), `signal` (FrameSpec/DataLinkSignal + the `SessionSink` seam), `timer` (the `TimerService` contract + integerised SRT-IIR/Karn/RC-backoff math — research §3, no FPU on the M0+), `guard`, `dispatch`, `subroutine`, `quirks` (the named figc4.x spec-defect fixes packet.net ships on by default: #38/#40/#41/#42/#43/#44/#45/#47), `tx` (TransitionContext), `session` (the driver), `bridge` (WireSink + classify_incoming — the spec↔wire adapter, mod-8 control octets; mod-128 extended framing is the documented codec follow-up), and `manager` (a fixed-capacity peer-keyed SessionManager — the on-target session array, no heap map). **117 host tests pass** (was 87), including a two-session wire harness that runs a full SABM/UA connect → I-frame exchange → DISC/UA teardown by carrying each session's *emitted wire octets* into the other (encoded → decoded → classified) — the cross-stack parity artifact. Core is clippy-clean (`-D warnings`), the new files are rustfmt-clean, and it builds for thumbv6m `no_std`+`alloc` under `-D warnings`. The `sdl` module's blocker doc and §5/§6/§10 of this plan are superseded by this entry.

  *Firmware crate (`ax25-node-fw`) — refreshed to the hardware gate.* Refreshed the stale embassy pins to the current coherent crates.io set (embassy-rp 0.10, embassy-executor 0.10 with `platform-cortex-m` — the renamed `arch-cortex-m` — , embassy-time 0.5, embassy-sync 0.8, embassy-net 0.9, cyw43 0.7, cyw43-pio 0.10, embedded-alloc 0.7); added `portable-atomic` with the `critical-section` feature (the RP2040 M0+ has no native atomic CAS — required or static_cell/embassy fail to build for thumbv6m) and `heapless`. Installed a `#[global_allocator]` (embedded-alloc `LlffHeap`, 16 KB arena) in `main.rs`. Wired the firmware `session.rs` to the real core `SessionManager` + an `embassy-time`-backed `TimerService`. **The full 367-crate firmware dependency tree (embassy/cyw43/embassy-net/smoltcp + ax25sdl + ax25-node-core) now resolves and compiles for thumbv6m.** The firmware *binary* does NOT yet link: the only remaining errors (10) are in its own `net.rs` CYW43 bring-up + `transports::*` socket-wiring stubs (E0061 arg-count / E0308 / E0599 `must_spawn` against the real cyw43/embassy-net/embassy 0.10 API). **That is the hardware gate** — per the research note CYW43 has no emulator, so finishing the radio bring-up + the transports needs a physical Pico W + debug probe to verify; it is not written blind. The §8.C "approve the embassy fetch" gate is effectively satisfied (the stack is fetched + compiles); §8.B `probe-rs-tools` + §8.D udev remain for the hardware tier.

- **2026-06-04 — read-only NET/ROM-aware slice landed (`ax25-node-core::netrom`).** Ported the C# `Packet.NetRom` library + `Packet.Node.Core.NetRom.NetRomService` (packet.net PR #303, grounded on `/home/tf/netrom-research.md`) to the node as a new `netrom` module in the core crate (chosen over a separate crate so it reuses the local `ax25` Callsign/Address/Frame codec without a new crate-to-crate dep — the same way `sdl` lives in core). It is the read-only "NET/ROM aware" capability: **hear NODES broadcasts (UI, PID 0xCF, dest "NODES" — 0xFF sig + 6-byte sender alias + ≤11 × 21-byte entries), build a routing table, surface it** — originating nothing on the air (no TX, no L4 circuits, no NODES origination).

  *no_std-clean for the M0+.* The whole module is allocation-free: integer quality maths (no FPU), and **fixed-capacity const-generic structures throughout** (`NetRomRoutingTable<MAX_DESTS, MAX_ROUTES, MAX_NBRS>` is a `[Option<…>; N]` of fixed-sized arrays, NOT a heap `Vec`/map; `NodesBroadcast` holds an inline `[Option<entry>; 11]`; the alias + port-id are fixed `Copy` buffers, not `String`). Verified zero `alloc`/`Vec`/`String` uses in non-test code; builds for `thumbv6m-none-eabi` (`--no-default-features --features alloc`) under `-D warnings`, and the host `cargo test` (default `std`) is green. Clippy clean (`-D warnings`) on both host and thumbv6m; the new files are rustfmt-clean. Time is injected (a `u64` monotonic tick on `ingest`/`observe_frame`) — no wall-clock in the core, matching the `TimerService` pattern.

  *Layout mirrors the C# split.* `netrom::wire` ports `Packet.NetRom.Wire`: the named-divergence `NetRomParseOptions` (STRICT/LENIENT/BPQ/XROUTER presets — hand-written, NOT via ax25sdl, since NET/ROM has no SDL figures and BPQ is the de-facto reference, exactly the research-doc §3.6 recommendation), the two field decoders (the 7-octet shifted callsign delegates to `ax25::Address::decode` — one source of truth for the shift/SSID semantics; the 6-byte alias is printable-ASCII-only + trailing-trim), the 21-byte `NodesRoutingEntry`, and the total `NodesBroadcast` parser (arbitrary bytes return `None`, never panic). `netrom::routing` ports `Packet.NetRom.Routing`: the multiplicative decay `(bq*pq+128)/256` (`quality::combine`), the `NetRomRoutingOptions` knobs (OBSINIT 6, MINQUAL 0, default-neighbour-quality 192), and the `NetRomRoutingTable` implementing every canonical heuristic (assumed direct route to originator, combined per-hop quality, trivial-loop guard → quality 0, ≤3 routes/dest best-first, quality-0/MINQUAL floor, OBSINIT reset + obsolescence sweep/purge, destination + neighbour caps, orphan-neighbour prune). The top-level `netrom::NetRomService` ports `NetRomService`/`INetRomRoutingView`: `observe_frame` is the read-only tap (the UI + 0xCF + dest-"NODES" gate → parse → ingest), plus the read accessors (`enabled`, `destination_count`, `for_each_{neighbour,destination,route}` — borrow/visitor, not an alloc snapshot).

  *The inbound-frame hook + the read-only guarantee.* `ax25-node-fw::session::observe_inbound` is the firmware hook — the `Ax25Listener.FrameTraced`-fires-before-`DispatchInbound` equivalent: the transport pumps (axudp/kiss_tcp/kiss_serial stubs) call it for **every decoded inbound frame, BEFORE address filtering**, so NODES broadcasts (addressed to the literal callsign "NODES", not to the node) are heard — they would never reach a session otherwise. It is observation-only: it never alters the frame, emits nothing, and shares no state with the session layer, so a NODES storm mid-QSO leaves the connected link untouched (proven by a host test that storms the tap while a `SessionManager` session stays `Connected` and still processes a fresh event).

  *Tests + parity.* 37 new host tests (154 total, was 117), mirroring `Packet.NetRom.Tests` + the C# `NetRomAwareIntegrationTests` read-only slice: parser totality (short/truncated/pseudo-random-garbage never panic; the strict-vs-lenient paired divergence tests for trailing-partial-entry and empty-list; BPQ/XRouter presets accept a padded dump), the research doc's quality worked examples (200→156→78, plus the C# inline-data vectors), and the full routing-table suite (neighbour + assumed-direct-route + combined quality, trivial-loop guard, 3-best cap, in-place refresh, OBSINIT init/decrement/purge/reset, orphan-neighbour drop, MINQUAL floor + below-floor removal, destination cap, alias-then-callsign snapshot ordering). **Parity gaps vs the C# slice (all out of the brief's read-only scope):** no NODES *origination*/broadcast scheduler, no L4 `CircuitManager`, no interlink sessions, no `connect <alias>` routing — these are the Phase-9 body the research doc scopes as "after"; the read-only data model + interop calibration they depend on is exactly what this slice front-loads. Minor shape differences from the desktop (forced by no_std, not behavioural): the snapshot is a borrow/visitor API rather than an allocated `IReadOnlyList`, `LastHeard` is an injected `u64` tick rather than `DateTimeOffset`, and the structural caps are compile-time const generics rather than runtime options — the *behaviour* (heuristics, quality maths, divergence flags) is faithful.

- **2026-06-04 — serial-KISS + the NinoTNC-specific extensions landed (capability 3, `ax25-node-core::kiss::{serial,ninotnc}` + the rest of `Packet.Kiss`).** Built out the "KISS over serial to a NinoTNC" connectivity tier. The base KISS codec (encoder/decoder/frame, the `KissCommand` set incl. command-byte escape + port nibble) was already host-tested; this slice adds the rest of `Packet.Kiss`'s behaviour and the NinoTNC overlay, all `no_std`-clean and host-testable, with the live UART exchange the only hardware-gated piece.

  *KISS codec completion (`Packet.Kiss`).* New `kiss::ackmode` (the G8BPQ ACKMODE extension `Packet.Kiss.KissAckMode`: `build_send_frame`/`build_payload_into`, `try_parse_acknowledgement`, `try_parse_data_frame`), `kiss::params` (the TXDELAY/PERSIST/SLOTTIME/TXTAIL/FULLDUPLEX builders — the parameter surface of `KissSerialModem.Set*Async`, as wire-byte builders since the core is byte-only), and `kiss::classify` (`Packet.Kiss.KissFrameClassifier` + `KissInboundEvents` → the closed `InboundEvent<'a>` enum: `Ax25` / `AckModeData` / `Unknown`; the C# open record hierarchy becomes a Rust enum because Rust enums are closed).

  *The serial-KISS transport seam (`Packet.Kiss.Serial.KissSerialModem` + `IKissModem`).* `kiss::serial` defines the `ByteStream` async-byte-transport trait (the UART-vs-pipe seam, async-fn-in-trait like `console::connection::NodeConnection`) and the generic `SerialKissModem<S: ByteStream>` — the modem/frame-source seam the SDL runtime/node consume (`send_frame`/`send_ackmode`/`set_mode`/`send_kiss`/the parameter setters + `read_frame`). It owns a fixed outbound encode buffer (no per-send alloc — the embedded path) and the streaming inbound `Decoder`, mirroring `KissSerialModem`'s `KissEncoder`+`KissDecoder` usage; port nibble fixed at 0 (`KissPort = 0`). A test-only in-memory `MemStream` loopback proves the framing end-to-end on the host (encode → wire → streaming-decode, incl. awkward 3-byte chunking + a full AX.25-body round-trip through two modems). The firmware side (`ax25-node-fw::transports::kiss_serial`) now implements `ByteStream` over an `embassy_rp::uart::BufferedUart` (`UartByteStream`) and drives `SerialKissModem` + the NinoTNC classifier in a real read pump — the only seam left is the two `embedded_io_async` UART calls + `BufferedUart::new`, isolated for hardware bring-up (added `embedded-io-async` to the fw deps).

  *The NinoTNC-specific extensions (`Packet.Kiss.NinoTnc`).* `kiss::ninotnc` faithfully ports: `catalog` (the mode 0–15 table + the firmware-byte→mode reverse map — `NinoTncMode`/`NinoTncCatalog`, kept verbatim, `const` tables + linear scan instead of `FrozenDictionary`), `sethw` (the SETHW mode byte with the `+16` non-persist offset + `MAX_MODE 15` — `NinoTncSetHardware`), `txtest` (the synthetic host-side `=FirmwareVr:`-marker diagnostic parser `NinoTncTxTestFrame` — total, scans for each `=Key:Value` on demand with no `String`/`HashMap`, hex fields → `u64`, `BrdSwchMod` XX/YY/ZZZZ low-byte → catalog), `airtest` (the over-air `CQBEEP-5` + `{N }`-stepping-ASCII recognizer `NinoTncAirTestFrame`), `firmware` (the firmware-version + dsPIC chip-variant value types), and `classify` (the `NinoTncFrameClassifier` overlay → the `NinoTncInboundEvent<'a>` enum: `TxTestDiagnostic`/`AirTest`/`Generic(InboundEvent)`, upgrading the generic classification exactly as the C# does, marker-shape-wins-over-AX.25 included). `DEFAULT_BAUD = 57600` matches `NinoTncSerialPort.DefaultBaudRate`.

  *no_std / gates.* 77 new host tests (231 total, was 154), mirroring `Packet.Kiss.Tests` + `Packet.Kiss.NinoTnc.Tests` + `Packet.Kiss.Serial.Tests` (encode/decode totality incl. command-byte escape + arbitrary chunking, the ACKMODE worked vectors, the SETHW `+16` arithmetic, the catalog + firmware-byte lookups, the TX-Test field parse + graceful-degradation + totality-on-garbage, the over-air recognizer's captured-press vectors, the classifier upgrades, and the host-loopback serial framing). Host `cargo test` green; core builds for `thumbv6m-none-eabi` (`--no-default-features --features alloc`) under `-D warnings`; clippy clean (`-D warnings`) on host + thumbv6m; new files rustfmt-clean. The integer no-FPU posture is honoured: `NinoTncMode::transmission_us` is integer microseconds (the C# `double TransmissionMs` is kept `std`-only for exact host parity). **Parity divergences (all out of node scope, documented in `kiss::ninotnc`):** `NinoTncPortDiscovery` (host serial enumeration — the Pico UART is a fixed peripheral, no enumeration), the firmware OTA catalogue/flasher (host tooling — only the version/chip-variant value types are ported), and the `NinoTncSerialPort` async driver glue (the *protocol* it speaks is ported; its `System.IO.Ports` + `Channel`/`TaskCompletionSource` ACKMODE-echo correlation maps onto the firmware's embassy UART transport — `send_ackmode` frames the tag, the caller correlates the echo via `ackmode::try_parse_acknowledgement`, the same framing-neutral split the C# uses between `KissAckMode` and the driver). **Hardware-gated:** the real UART-to-NinoTNC run needs a physical Pico W wired to a NinoTNC (no UART/NinoTNC emulator) — the codec/transport/extension logic is the host-testable deliverable and is green now.

- **2026-06-07 — hardware arrived; `docs/HW-BRINGUP.md` written as the hand-off runbook.** Tom's Pico W + Raspberry Pi Debug Probe have arrived, but they cannot be attached to this dev box — the bring-up will be driven by a *different* Claude session on the machine the board is plugged into. `docs/HW-BRINGUP.md` is the self-contained runbook for that session: the prime directive (the protocol work is done and green — 287 host tests, the firmware compiles for thumbv6m but does not yet link; the whole job is the `net.rs` CYW43 bring-up + the 4 transport socket stubs), prerequisites (RP2040-not-RP2350, probe wiring, gh auth for the private repos, a 2.4 GHz AP), repo + toolchain setup (including the load-bearing `../../../ax25sdl` sibling-clone path dep), the rig smoke test, seven explicit gates with green criteria (defmt heartbeat → WiFi/DHCP → AXUDP UI+SABM/UA → telnet console → KISS-TCP/net-sim → NinoTNC UART (optional) → embedded-test on-target), the secrets/firmware-blob policy (WiFi creds via `option_env!`, never committed; cyw43 blob licence check before vendoring), a lab-coordination table for anything touching the `packetdotnet` lab box (flag, don't improvise — net-sim's 8102 is loopback-only today), the repo working conventions (branch→PR→merge-on-green, the CI sibling-clone + 1.93.1 pin, fmt only touched files, PLAN.md §11 per gate), and the definition of done (minimum Gates 1–4; out of scope: HIL CI, RF tiers, ax25sdl codegen changes). §9's bring-up sequence remains the planning-level view; HW-BRINGUP.md is the operational version with exact commands and the gotchas (the Pico W LED lives on the CYW43 — no blinky before the radio is up; probe-rs supports the RP2040 board only).

- **2026-06-07 — Gate 1 GREEN: first silicon contact (the bring-up session, on the rig machine).** The HW-BRINGUP.md session is live on the machine the Pico W + Debug Probe are plugged into (fresh Ubuntu, nothing preinstalled). *Environment:* rustup stable 1.96.0 + thumbv6m + rust-src/llvm-tools, flip-link, cargo-binutils, probe-rs 0.31.0 (prebuilt installer), udev rule, picotool 2.2.0-a4 (prebuilt), `ax25sdl` sibling cloned. Baseline gate green on this box: **287 core tests, 0 fail; clippy `-D warnings` clean**. *Hardware:* the board is confirmed RP2040 (BOOTSEL enumerates as `2e8a:0003` RP2 Boot, not the RP2350's `000f`). **Gotcha hit + fixed: the Debug Probe shipped with v1.x `debugprobe` firmware, which probe-rs ≥0.29 refuses outright (min 2.2.0, a USB data-corruption bug in older firmware).** Old firmware predates the picotool reset interface, so no software path to BOOTSEL existed — Tom replugged the probe with its button held, and `picotool load -x debugprobe-v2.3.1.uf2 --bus/--address` (selectors matter: the blank Pico W was *also* in BOOTSEL) flashed the current firmware. After that, `probe-rs info` reads the RP2040 DP at every SWD speed tried (100 kHz–4 MHz; one transient "did not respond" immediately after the probe's own reboot, gone on retry). *Gate 1 (HW-BRINGUP.md §4):* `main.rs` reduced to the minimal binary — heap init, `embassy_rp::init`, a 1 Hz defmt heartbeat, plus a deliberate `config::load()` + `Callsign::write_display` log line so `ax25-node-core` provably executes on the M0+; the `net`/`session`/`transports` mods are commented out with GATE 2+ markers (they don't compile against the real cyw43/embassy-net APIs yet — the documented hardware gate), `config.rs` carries a temporary crate-level `allow(dead_code)`. **`cargo run --release` flashes over SWD, resets, and streams `pico-node 0.1.0 starting` / `node identity: M0LTE-1 (alias PICO, grid IO91wm)` / `heartbeat: uptime N s` over defmt/RTT, repeatably across re-runs — the whole hands-free loop (memory.x + flip-link + probe-rs + RTT) is proven.** Build is warning-free. Gates 2+ next: cyw43 bring-up in `net.rs` from the embassy `wifi_*` examples.

- **2026-06-07 — Gate 2 GREEN: CYW43 + WiFi + DHCPv4 (the real hardware gate).** `net.rs` is real now, written against the pinned set (cyw43 0.7 / cyw43-pio 0.10 / embassy-net 0.9 / embassy-rp 0.10) from the embassy `wifi_*` reference at the `cyw43-v0.7.0` tag: PIO-SPI bring-up, cyw43 runner task, CLM init, `Config::dhcpv4` + `RoscRng`-seeded `embassy_net::new`, net runner task, and a retry/backoff `join`. *Blobs:* `43439A0.bin` + `43439A0_clm.bin` + `nvram_rp2040.bin` (cyw43 0.7 takes a separate NVRAM blob) vendored under `crates/ax25-node-fw/cyw43-firmware/` from the embassy `cyw43-v0.7.0` tag **with the Infineon Permissive Binary License alongside** (licence-check per HW-BRINGUP §5: redistribution in binary form is expressly permitted with the notice; provenance + sha256s in the README there). *Credentials:* `option_env!("WIFI_SSID"/"WIFI_PASSWORD")` per §5 — builds without secrets (CI-safe), fails loudly at boot if missing. **Two real bugs found on hardware:** (1) cyw43's `JoinOptions` default (`Wpa2Wpa3`) sets the chip auth to SAE, which wedges association against this WPA2-PSK AP — **no join event ever fires, and the wedge persists across warm resets** (probe-rs resets the RP2040, not the radio), which mimicked deeper failures all afternoon; fixed with explicit `JoinAuth::Wpa2` + a 20 s `with_timeout` watchdog around the join. (2) **Rig gotcha: the SWD link FAULTs whenever the CYW43 is RF-active** (scan/associate/TX) — reproduced ~8× across SWD speeds (100 kHz–default), probe-rs 0.29.1 + 0.31.0, hub/charger/direct-port power, re-dressed jumpers; radio-off control runs are 100% clean. Mitigations that landed: gSPI divider 8 (default ~33 MHz faulted SWD even during non-RF SPI bursts; TODO(rig) retest faster once the SWD wiring is hardened) and **the proven observability workflow: `DEFMT_RTT_BUFFER_SIZE=16384` (now in `.cargo/config.toml`) + `probe-rs download`/`reset` detached through the radio phase, then `probe-rs attach` once the radio idles and drain the RAM backlog**. `DEFMT_LOG` is now `info,ax25_node_fw=debug` (cyw43 hexdumps the CLM at debug). **Green evidence, twice over:** cold power-cycle → scan sees 28 BSS (unifi71 at −31 dBm), `joined AP` in ~3 s, `IP address: 10.45.0.95/24`, LED on, heartbeats; warm `probe-rs reset` → same join + lease (uptime restart in the log proves the cycle). Host answers ARP for the Pico (REACHABLE); ICMP echo is not answered (smoltcp config — irrelevant, Gates 3–5 are real UDP/TCP). Gate 3 next: `transports/axudp.rs` + the local UDP harness.

- **2026-06-07 — Gate 3 GREEN: AXUDP over WiFi against the host harness (capability 1, minimum-green) + Gate-2 post-mortem corrections.** *Corrections first (Tom called it):* the Gate-2 entry's "SWD faults whenever the radio is RF-active" physical theory is **retracted** — with the SAE wedge fixed, three consecutive *live* `probe-rs run` sessions streamed RTT straight through boot→join→DHCP with zero faults, and three more with the gSPI divider back at `DEFAULT_CLOCK_DIVIDER` (~33 MHz) were equally clean. Every earlier fault correlated with the *wedged driver state*, not RF; the divider-8 change and the cable/power shuffling were red herrings (divider reverted, `fixed` dep dropped). The `DEFMT_RTT_BUFFER_SIZE=16384` backlog trick stays — it's generally useful. The SWD-during-wedge fault mechanism remains unexplained but is moot in healthy operation; noted in case it resurfaces. *Gate 3:* `transports/axudp.rs` is real — `embassy_net::udp::UdpSocket` bound on the configured port, `select(beacon ticker, recv)` loop, `ax25-node-core::axudp` encode/decode both ways, **the read-only NET/ROM tap wired exactly at the C# `FrameTraced` point** (every decoded inbound frame, before address filtering, into a task-owned `NetRomService`; `Ingested` outcomes logged), and a 10 s UI beacon (`my_call → IDENT`) to the env-configured harness endpoint (`AXUDP_BEACON_TARGET`, §5 — LAN detail, never committed; absent ⇒ listen-only). `mod session` + `mod transports` re-enabled (only axudp spawns; telnet/kiss return Gates 4–6; session machinery awaits the supervisor). New `tools/axudp-harness.py` (stdlib-only): decodes AXUDP datagrams, prints `SRC > DEST` summaries, replies once per peer with a UI frame. **Green evidence (both directions, live RTT throughout):** harness logged `M0LTE-1 > IDENT UI pid=0xf0 info='pico-node AXUDP beacon (HW-BRINGUP Gate 3)'`; firmware logged `axudp: rx HARNES-1 -> M0LTE-1 ctl=0x03` + the reply text, repeatably across beacon cycles. Stretch (SABM/UA connected mode vs `pdn`) stays parked pending the lab-side `axudp` port (§6 coordination table). Gate 4 (telnet console) next.

- **2026-06-07 — Gate 4 GREEN: telnet console served from the Pico (capability 4). Minimum bring-up (Gates 1–4) COMPLETE.** `transports/telnet.rs` is real: an `embassy_net::tcp::TcpSocket` accept loop (one connection at a time, 300 s idle timeout, graceful close + abort between connections), the banner + prompt from `console::service::banner_and_prompt`, reads through the host-tested `LineAssembler` line discipline, `parse_bytes` → `dispatch` per line, CRLF rendering via `TransportKind::Telnet`. `Identity` (alias/callsign/grid + the live axudp port line) and the `CALL} ` prompt are built in `main` and passed in; `Connect` answers its "Connecting to …" line then says plainly that connected-mode isn't wired to the console yet (the session-supervisor seam). **Green evidence, first flash:** scripted `nc` session and a real `telnet` client both show the `PICO  [pico-node 0.1.0]` banner + `M0LTE-1} ` prompt; `I` (node info + version), `N` (node + ports incl. `axudp [up] udp/0.0.0.0:10093`), `H` (help) all answer; garbage gets "Unknown command"; `B` answers `73` and disconnects cleanly ("Connection closed by foreign host"); firmware logs both connections opening and closing, and the listener survives reconnection. **HW-BRINGUP.md §8 "minimum (call it a successful bring-up)" is hereby met: Gates 1–4 green** — hands-free flash/RTT loop, WiFi + DHCP, AXUDP frame exchange with a host harness, telnet console. Remaining stretch: Gate 5 (KISS-TCP minimum vs a local harness), Gate 7 (embedded-test on-target), and the lab-coordinated items (§6 table: pdn axudp port, net-sim LAN port). Gate 6 skipped per runbook (no NinoTNC at this machine).

- **2026-06-07 — Gate 5 GREEN (minimum): KISS-over-TCP round-trips against a local harness (capability 2).** `transports/kiss_tcp.rs` is real: a reconnect/backoff `TcpSocket` client (the `ReconnectingKissModem` shape) to the `KISS_TCP_TARGET` build-env endpoint (§5 — `KissTcpConfig` now carries `target: Option<&str>`; the old committed `host:"192.168.1.10"` placeholder is gone; absent ⇒ transport disabled), beacon ticker + read pump over `select`, outbound via `kiss::encode(port 0, Data, frame.encode())`, inbound through the host-tested streaming `kiss::Decoder` → `Frame::decode` → the read-only NET/ROM tap → log. Shared transport helpers extracted to `transports::mod` (`ui_frame`/`call_str`/`parse_endpoint`/`tcp_write_all`) — axudp/telnet de-duplicated onto them. New `tools/kiss-tcp-harness.py` (stdlib-only): KISS de-framer (FEND/FESC), AX.25 summary per Data frame, replies once per connection with a KISS-framed UI frame. **Green evidence (first flash, live RTT):** harness logged `KISS port=0 DATA 61B: M0LTE-1 > IDENT UI info='pico-node KISS-TCP beacon (HW-BRINGUP Gate 5)'` and replied; firmware logged `kiss-tcp: rx HARNES-2 -> M0LTE-1` + the reply text; beacons repeat each cycle. The net-sim attachment stays lab-coordinated (§6: net-sim's pdn KISS port is loopback-only today). Remaining stretch: Gate 7 (embedded-test on-target) and the §6 lab items. Gate 6 skipped (no NinoTNC here).

- **2026-06-07 — Gate 7 GREEN: `embedded-test` on-target suite — the connected-mode SDL lifecycle proven on the physical M0+.** `cargo test` in the fw crate now flashes the RP2040 over the probe and runs each case with a device reset in between (probe-rs autodetects the embedded-test binary). *Setup:* `embedded-test` bumped 0.6 → **0.7.1** to pair with probe-rs 0.31 (the coordinated bump Gate 1's runbook note anticipated), moved from dev- to regular `[dependencies]` so its build script puts `embedded-test.x` on the linker search path for every target (`build.rs` adds `-Tembedded-test.x`; the script resolves to empty sections in the normal firmware binary — verified the full node still boots + serves after the change); `[[test]] on_target` with `harness = false`. *Suite (`tests/on_target.rs`):* its own heap arena + `embassy_rp::init` per case in `#[init]`, then (1) codec fundamentals on-target (callsign parse/display, CRC-16/X.25 known-answer, I-frame encode/decode round trip), (2) **the on-target twin of the core's two-session wire harness: SABM/UA connect → I-frame exchange (V(S)/V(R) advance, DataIndication surfaces) → DISC/UA teardown, every frame carried as encoded wire octets through the real codec + `classify_incoming`** — the no-FPU/no-CAS/embedded-alloc proof of the whole SDL runtime on real silicon, and (3) a NET/ROM NODES ingest via the production `nodes_broadcast_builder` (fixed-capacity table + integer quality maths on-target). **`cargo test --release`: 3 passed, 0 failed, 3.48 s wall** (flash once, reset per case). This is the seed of the hands-free HIL CI rig (PLAN §7 Loop C); the CI workflow itself stays a follow-up per the runbook.
