# Pico-node — plan: a Rust RP2040 / Pico W AX.25 packet node at parity with the C# node host

*Workspace: `/home/tf/pico-node`. Status as of 2026-06-04. This is the living plan for a from-scratch Rust firmware that mirrors the `m0lte/packet.net` C# node host (`Packet.Node.Core`) on Pico W hardware, built on the AX.25 SDL state machine from `m0lte/ax25sdl`.*

This plan was produced **before the hardware arrives** (Pico W + official Raspberry Pi debug probe are on order). It is grounded on three prior research notes in `m0lte/packet.net/docs/research/`: [`pico-w-rust-dev-workflow.md`](../../packet.net/docs/research/pico-w-rust-dev-workflow.md) (the dev-loop/toolchain verdict), [`pico-packet-node.md`](../../packet.net/docs/research/pico-packet-node.md) (the node design — "the work is the runtime, not the tables"), and `codegen-reach.md` (conformance vectors as the drift-proof net). Where this plan and those notes diverge, the divergence is flagged (e.g. the SP-010-in-Rust status).

---

## 0. TL;DR

- **Architecture is settled and proven**: a two-crate workspace — a portable, `no_std`-able, **zero-dependency** logic crate (`ax25-node-core`) that is `cargo test`ed on the host today, plus a thin RP2040/Embassy binary (`ax25-node-fw`) that wires it to the WiFi radio, the network stack, and the UART. This mirrors the research note's recommended split and the C# host's module boundaries.
- **Real, tested, hardware-independent code exists now**: the KISS codec, AXUDP framing, the AX.25 frame/address/callsign codec, the CRC-16/X.25 FCS, the SDL loop-executor, and the whole telnet/command-prompt layer are ported from the C# host and pass **87 host unit tests**, offline, with zero external crates. The core crate also **compiles cleanly in `no_std` + `alloc` mode**, proving the embedded posture is real.
- **Two hard external blockers** gate going further, both in `m0lte/ax25sdl`'s **Rust** backend and both surfaced clearly below (§6): the generated SDL crate is (a) not `no_std` and not published, and (b) still **stringly-typed** — SP-010's typed verb/guard/event closed sets have shipped only in the C#/TS backends, *not* Rust. Neither blocks the host work done so far; both block wiring the real state tables.
- **One environment blocker** gates building the firmware at all: there is **no Rust cross-compiler for `thumbv6m-none-eabi`** here (system `rustc`, no `rustup`, no target `core`, no `rust-src`). The firmware crate is fully scaffolded and ready, but cannot be built until the toolchain in §8 is approved/installed.

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
