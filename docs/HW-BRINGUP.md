# Pico W hardware bring-up — runbook for the session driving the rig

*Written 2026-06-07, the day the hardware arrived. This is the hand-off document for a fresh Claude Code session running on the machine the Pico W + Raspberry Pi Debug Probe are physically connected to (the original dev box cannot host them). It operationalises [`PLAN.md`](PLAN.md) §9 ("when the hardware arrives") into concrete steps with verification gates. Read PLAN.md §0–§2 for the project context first; this document assumes it.*

---

## 0. Context — what you are walking into, and the prime directive

This repo is a Rust firmware for a Raspberry Pi **Pico W (RP2040 + CYW43439)** packet-radio node, at protocol parity with the C# node host in `m0lte/packet.net`. The split:

- **`crates/ax25-node-core`** — the portable `no_std` protocol stack: AX.25 frame/address codec, CRC, KISS (+ ACKMODE + NinoTNC extensions), AXUDP framing, the telnet console layer, the **full connected-mode AX.25 v2.2 SDL runtime** (driven off the generated `ax25sdl` typed tables), and **NET/ROM** (NODES ingest, routing table, L3 datagram forwarding with per-flow quality-weighted multi-route load-balancing). **This is DONE and host-tested: `cargo test -p ax25-node-core` = 287 tests green.** It also builds `no_std`+`alloc` for `thumbv6m-none-eabi` under `-D warnings`.
- **`crates/ax25-node-fw`** — the thin Embassy RP2040 binary. Its full 367-crate dependency tree (embassy-rp/cyw43/embassy-net/smoltcp + the core + ax25sdl) **resolves and compiles for thumbv6m**, but the binary **does not yet link**: the only remaining errors (~10) are in `src/net.rs` (three `unimplemented!()` doc-stubs whose signatures don't match the real cyw43/embassy-net 0.9/0.10 APIs) and the four `src/transports/*.rs` socket stubs. They were deliberately not written blind — **no emulator exists for the CYW43 radio**, so finishing them requires exactly the hardware you now have.

**The prime directive: the protocol work is finished and green — your job is wiring silicon, not protocol.** Do not modify `ax25-node-core`'s logic to make firmware integration easier without keeping its host tests green, and never hand-copy/edit the generated SDL tables (they come from the `ax25sdl` sibling; spec-side changes are raised there).

Grounding documents (read on demand, not all up front):

- [`PLAN.md`](PLAN.md) — the living plan. §9 is the checklist this runbook expands; §11 is the amendment log you must keep current.
- The research notes in `m0lte/packet.net` (`docs/research/pico-w-rust-dev-workflow.md`, `pico-packet-node.md`, `codegen-reach.md`) — the toolchain/workflow rationale. GitHub: <https://github.com/m0lte/packet.net/tree/main/docs/research>.
- The Embassy `examples/rp` tree — **the canonical reference for `net.rs`**: <https://github.com/embassy-rs/embassy/tree/main/examples/rp/src/bin> (`wifi_blinky.rs`, `wifi_tcp_server.rs`, `wifi_webrequest.rs`).

---

## 1. Prerequisites on this machine

**Hardware in hand (verify before starting):**

- [ ] 1× Raspberry Pi **Pico W** — confirm it is the original **RP2040** Pico W, **not** a Pico 2 W (RP2350): `probe-rs` does not support the RP2350, which breaks the entire `cargo run` flash/RTT loop this plan depends on. The chip marking should say RP2040.
- [ ] 1× **Raspberry Pi Debug Probe** (the "debug board") — ships pre-flashed with the `debugprobe` CMSIS-DAP firmware; no firmware step needed.
- [ ] 2× USB cables (probe → host, Pico W → host for power).
- [ ] If the Pico W is a **WH** variant (pre-fitted JST-SH debug connector): the probe's 3-pin JST-SH↔JST-SH cable plugs straight into the DEBUG connector. If it is a bare **W** (unpopulated DEBUG holes at the board edge): solder a 3-pin header or wires to the DEBUG pads and use the probe's JST-SH↔0.1" flying-lead cable — **SWCLK / GND / SWDIO**, matching the silkscreen on both ends. This is the only soldering/wiring in the whole plan.

**Access:**

- [ ] GitHub auth for **`m0lte/pico-node` (private)** — `gh auth status` or an SSH key Tom has authorised. `m0lte/ax25sdl` is public.
- [ ] A **2.4 GHz** WiFi AP in range, with the SSID/password available as environment variables at build time (§5 — never committed). The CYW43439 is 2.4 GHz-only.
- [ ] (Later gates) LAN reachability to a peer for AXUDP — see §6 Gate 3 for options.

---

## 2. Repo + toolchain setup

```sh
# 1. The two repos MUST be siblings — ax25-node-core's Cargo.toml has
#    `ax25sdl = { path = "../../../ax25sdl/spec/rust" }` (kept local by design, no crates.io pin).
git clone git@github.com:m0lte/pico-node.git
git clone https://github.com/m0lte/ax25sdl.git
# layout:  <dir>/pico-node  and  <dir>/ax25sdl

# 2. rustup (NOT a distro rust — a packaged cargo without rustup cannot add the thumbv6m target).
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
source "$HOME/.cargo/env"
rustup target add thumbv6m-none-eabi
rustup component add rust-src llvm-tools
# (crates/ax25-node-fw/rust-toolchain.toml pins these too and will auto-install on first build there.)

# 3. Linker + size tools + the flash/debug tool.
cargo install flip-link cargo-binutils
sudo apt install -y pkg-config libudev-dev     # probe-rs build deps on Debian/Ubuntu
cargo install probe-rs-tools                   # or the prebuilt installer from https://probe.rs

# 4. udev rule so probe-rs can open the probe without root (then REPLUG the probe):
#    https://probe.rs/docs/getting-started/probe-setup/  (69-probe-rs.rules)
sudo curl -o /etc/udev/rules.d/69-probe-rs.rules https://probe.rs/files/69-probe-rs.rules
sudo udevadm control --reload && sudo udevadm trigger
```

**Baseline gate (do this before touching anything):**

```sh
cd pico-node
cargo test -p ax25-node-core          # expect: 287 passed, 0 failed
cargo clippy -p ax25-node-core --all-targets -- -D warnings   # clean
```

If the baseline isn't green, stop and fix the environment (almost always: the ax25sdl sibling is missing/misplaced, or a distro cargo is shadowing rustup's — `which cargo` must be `~/.cargo/bin/cargo`).

---

## 3. Rig assembly + probe smoke test

1. Connect the Debug Probe's **"D" (debug) port** to the Pico W's **DEBUG** header (SWCLK/GND/SWDIO — §1). Optionally connect the probe's **"U" (UART) port** to the Pico's GP0/GP1 for a spare serial channel; the primary diagnostics are defmt-over-RTT via SWD, so this is not required.
2. USB: probe → host, Pico W → host (power). The Pico W needs nothing pre-flashed — probe-rs programs it over SWD regardless of flash state; **BOOTSEL is never used** in this workflow.
3. Verify, in order:

```sh
probe-rs list                          # expect: a "Debug Probe (CMSIS-DAP)" entry
probe-rs info --chip RP2040            # expect: target info read over SWD (two cores, RP2040 IDs)
```

If `probe-rs list` is empty: replug after the udev rule, check `lsusb` for 2e8a:000c (Debug Probe), and that you're not in a container without USB passthrough. If `list` sees the probe but `info` fails: re-check the three SWD wires (SWCLK↔SWCLK, SWDIO↔SWDIO, GND↔GND) and that the Pico has power.

Leave the rig permanently assembled — it later becomes the hands-free on-target CI rig (PLAN.md §7 Loop C / research note §4.3).

---

## 4. Bring-up gates

Work the gates **in order; each must be green before the next**. Commit per gate (small PRs to `main`; CI must stay green — see §7). The current firmware `main.rs` spawns transports that don't compile yet, so Gate 1 deliberately starts from a *minimal* binary rather than fixing all ~10 errors blind.

### Gate 1 — first silicon contact: a minimal binary flashes and logs

Reduce `ax25-node-fw` to the smallest thing that links: boot, init `embassy-rp`, and a periodic `defmt::info!` heartbeat. Temporarily `cfg`-gate or comment out the `net`/`transports`/`session` spawns (they return at Gates 2–6). **Known Pico-W gotcha: the onboard LED is wired to the CYW43, not an RP2040 GPIO — there is no "blinky" before the radio chip is up. Use defmt logs as the heartbeat, not the LED.**

```sh
cd crates/ax25-node-fw
cargo run --release        # probe-rs flashes over SWD, resets, streams defmt/RTT
```

**Green =** the heartbeat lines stream to your terminal and survive a re-run. This proves: linker scripts + `memory.x` + flip-link + the probe + RTT — the whole hands-free loop. If the link fails on embassy API drift, fix our code against the **pinned** versions first (embassy-rp 0.10 / embassy-executor 0.10 / embassy-time 0.5 / embassy-net 0.9 / cyw43 0.7 / cyw43-pio 0.10); bump pins only deliberately and coherently (the defmt family 0.3/0.4 and `embedded-test` 0.6 may need a coordinated bump to match your installed probe-rs — that's acceptable, note it in the PR).

### Gate 2 — CYW43 + WiFi up (`src/net.rs`, the real hardware gate)

Implement the three `unimplemented!()`s in `net.rs` from the Embassy `examples/rp` `wifi_*` examples: PIO-SPI bring-up of the CYW43 (`cyw43-pio`), spawn the cyw43 runner task, `join_wpa2` with retry/backoff, then `embassy_net::new` with DHCPv4 and spawn the net task.

- **Firmware blobs:** the CYW43439 needs `43439A0.bin` + `43439A0_clm.bin` (loaded via `include_bytes!`). They live in the embassy repo's `cyw43-firmware/` directory. **Check the licence file in that directory before vendoring them into this repo**; if it permits redistribution (it is Infineon-licensed for this purpose — verify), commit them under `crates/ax25-node-fw/cyw43-firmware/` *with the licence file alongside*; otherwise add a download script + `.gitignore` them.
- **Credentials:** per §5 — build-time env, never committed.
- Once the chip is up, set the CYW43 GPIO 0 high — **the LED turns on, which is itself the visible "radio alive" check.**

**Green =** defmt shows join success and a DHCP lease (`IP address: 192.168.x.y`), repeatably across power cycles.

### Gate 3 — Capability 1: AXUDP over WiFi (the headline parity demo)

Fill `transports/axudp.rs`: an `embassy_net::udp::UdpSocket` bound per config, feeding `ax25-node-core::axudp` framing (already written + tested) into the node's frame path.

First contact does **not** need the lab: run a tiny UDP listener on this very machine (a few lines of Python: receive datagram → it *is* an AX.25 frame, decode the addresses or just hexdump) and have the Pico beacon a UI frame at it. **Green (minimum) =** a well-formed AX.25 UI frame from the Pico arrives on the host listener, and a frame sent back is decoded and logged by the Pico.

**Stretch (coordination needed):** connected-mode SABM/UA + I-frame exchange against a real peer — `pdn` (the C# node on the lab box `packetdotnet`) speaks AXUDP but needs an `axudp` port added to its config, and LinBPQ speaks BPQ-style AXIP. Those are lab-side changes this session probably can't make — **flag to Tom rather than improvising**. The connected-mode state machine on the Pico is the already-tested SDL runtime; this gate is only about the socket plumbing.

### Gate 4 — Capability 4: telnet console

Fill `transports/telnet.rs`: an `embassy_net` TCP listener feeding `ax25-node-core::console` (parser/assembler/service — already written + tested, including the CR/LF/CR-NUL line discipline). **Green =** `telnet <pico-ip>` shows the banner + prompt; `I`, `N`, `H` answer; `B` disconnects cleanly.

### Gate 5 — Capability 2: KISS-over-TCP (net-sim)

Fill `transports/kiss_tcp.rs` (the `kiss` codec is done + tested; this is socket plumbing). **Lab note:** net-sim on `packetdotnet` currently publishes its `pdn` KISS port on `127.0.0.1:8102` (loopback-only) and `gb7rdg` on `0.0.0.0:8101` — attaching the Pico needs a net-sim node/port published on the LAN, which is a lab config change. **Green (minimum) =** KISS framing round-trips against any KISS-TCP endpoint reachable on your LAN (a local netcat-style harness is fine); the net-sim attachment is the stretch, coordinated with Tom.

### Gate 6 — Capability 3: KISS-over-serial to a NinoTNC (optional — needs the TNC)

Only if a NinoTNC is physically present at this machine (the known units are wired to the original dev box). Wire Pico UART ↔ NinoTNC UART pins directly (TX↔RX, GND; 57600 baud; bypassing the TNC's USB bridge), fill the two `embedded_io_async` calls in `transports/kiss_serial.rs` (`UartByteStream` over `embassy_rp::uart::BufferedUart` is already sketched). The codec, NinoTNC mode catalog, SETHW, and TX-Test parsers are all done + host-tested. Skip without guilt if no TNC is present.

### Gate 7 — on-target tests (`embedded-test`)

Stand up the declared `embedded-test` dev-dependency as the `cargo test` runner for the fw crate and run a small on-target suite on the real M0+: at minimum a SABM/UA connect + I-frame exchange scenario through the SDL runtime (mirror the core's two-session wire harness). **Green =** `cargo test` in the fw crate flashes, runs each case with device reset between, and reports pass/fail through probe-rs. This is the seed of the permanent hardware-in-the-loop CI job (a later, separate piece of work — don't build the CI workflow in this session unless asked).

---

## 5. Secrets and blobs policy

- **WiFi credentials:** read at **build time** from environment variables — `option_env!("WIFI_SSID")` / `option_env!("WIFI_PASSWORD")` (fail with a clear compile/boot error when absent). **Never** commit credentials, and never write them into tracked files; if you add a local config file convenience, `.gitignore` it in the same commit.
- **CYW43 firmware blobs:** licence-check before vendoring (Gate 2). Whatever you choose, the decision and the licence text travel in the same PR.
- This repo is **private**, but treat it as publishable: no SSIDs, passwords, or LAN details in committed code or docs (LAN details in PR descriptions are fine).

---

## 6. Lab / network coordination points (flag, don't improvise)

| Want | Needs | Owner |
|---|---|---|
| AXUDP vs a real node (`pdn` on `packetdotnet`) | an `axudp` port added to pdn's config (hot-reloadable) | Tom / a session with lab SSH |
| KISS-TCP into net-sim | a net-sim port published on the LAN (currently loopback-only for `pdn`) | Tom / a session with lab SSH |
| NinoTNC serial | a NinoTNC physically at this machine | Tom |
| RF on air | out of scope for bring-up | — |

The minimum-green path for every gate works **standalone on this machine** (local UDP/TCP harnesses); the lab targets are the stretch goals.

---

## 7. Working conventions (match the rest of the ecosystem)

- **Branch → PR → merge on green CI.** CI (`.github/workflows/ci.yml`, self-hosted runner) clones the **ax25sdl sibling** itself, pins Rust 1.93.1, and gates: `clippy -p ax25-node-core -- -D warnings`, `cargo test -p ax25-node-core`, and the core `no_std` build. It does **not** build the fw crate today — adding a thumbv6m fw build job once the binary links is a worthwhile follow-up PR.
- **Keep the 287 core tests green at every commit.** New core code (if any) needs host tests; firmware-only changes must not require core changes to pass clippy.
- **`cargo fmt` only the files you touch** — older core files carry known pre-existing rustfmt drift (ax25/console/crc/sdl); a wholesale reformat would bury your diff. (A dedicated fmt-only PR is owed someday; not this session.)
- **Update [`PLAN.md`](PLAN.md) §11 (amendment log) in the same PR as each gate** — same discipline as `packet.net`'s plan: if the log doesn't say it happened, it didn't happen.
- The fw crate is **workspace-excluded** (build it with `--manifest-path crates/ax25-node-fw/Cargo.toml` or from its directory); its `.cargo/config.toml` sets `target=thumbv6m-none-eabi` + the probe-rs runner, which is exactly why it must stay excluded — a workspace-level default target would break host `cargo test`.
- Firmware modules are `#[cfg(target_os = "none")]`-gated so stray host builds of the crate don't error.
- **When in doubt, ask Tom** — especially anything that transmits (even AXUDP beacons go onto his LAN) and anything touching the lab. The cost of a question is lower than a wrong assumption.

---

## 8. Definition of done

**Minimum (call it a successful bring-up):** Gates 1–4 — silicon contact with the hands-free `cargo run`/defmt loop proven, WiFi associated with DHCP, an AXUDP UI frame exchanged with a host harness, and a telnet session served from the Pico. PLAN.md §11 updated per gate; rig left assembled.

**Stretch:** Gate 5 minimum-green, Gate 7 on-target tests, and the connected-mode AXUDP exchange against a real peer once the lab side is coordinated.

**Out of scope for this session:** the hardware-in-the-loop CI workflow, NODES origination / L4 circuits on-target validation against the lab, RF/TNC tiers beyond Gate 6, and any `ax25sdl` codegen changes (raise upstream instead).

---

## 9. Dev loop + OTA bench (operational notes)

Hard-won rig knowledge. Credentials/IPs are placeholders — fill from your own
environment; never commit the real values (§5).

### Flashing since OTA (the app is bootloader-chained, blob de-duplicated)

The app is **not** standalone: it boots via the bootloader and reads the cyw43
firmware from the **BLOBS** flash region (`docs/OTA.md`). So the chip needs
**three** things present, flashed once:

```sh
# Build artifacts
( cd crates/ax25-node-bootloader && cargo build --release )   # -> bootloader ELF
( cd crates/ax25-node-fw         && cargo build --release )   # -> app ELF
python3 scripts/build-blobs.py crates/ax25-node-fw/cyw43-firmware/43439A0.bin \
  crates/ax25-node-fw/cyw43-firmware/43439A0_clm.bin \
  crates/ax25-node-fw/cyw43-firmware/nvram_rp2040.bin /tmp/blobs.bin

# Flash. FULL-ERASE whenever the flash layout changed — a stale embassy-boot
# state sector corrupts the first OTA swap (erased vector table -> lock-up).
probe-rs download --chip RP2040 --chip-erase <bootloader-elf>
probe-rs download --chip RP2040 --binary-format bin --base-address 0x10108000 /tmp/blobs.bin
probe-rs download --chip RP2040 <app-elf>
# Verify the boot-critical pages — the bench SWD is flaky and silently corrupts
# writes (see below). Compare the first 4 KiB of the app .bin and of blobs.bin:
probe-rs verify --chip RP2040 --binary-format bin --base-address 0x10007000 <app-vt-4k.bin>
probe-rs verify --chip RP2040 --binary-format bin --base-address 0x10108000 /tmp/blobs.bin
probe-rs reset --chip RP2040
```

Thereafter the hands-free loop (`cargo run` from the fw dir) flashes **only** the
app to ACTIVE; the resident bootloader + BLOBS chain it. For release-shaped,
credential-free artifacts and the BOOTSEL files (`pico-node-firmware.uf2` +
`pico-node-blobs.uf2` — two contiguous files, because a single multi-region
combined UF2 won't drag-drop; see docs/OTA.md), use `scripts/package-ota.sh`.

### probe-rs / SWD gotchas (these cost real time)

- **`pkill -9 -x probe-rs`, never `pkill -f probe-rs`.** `-f` matches the
  *full command line*, which includes the word "probe-rs" in your own shell
  wrapper — it SIGKILLs the very shell running the command (exit 1, no output,
  looks like the flash failed).
- **`DP Multidrop` / "communication with an access port" warnings on
  `reset`/`verify` are normal while the radio is running** — the CYW43 PIO +
  clocks make SWD flaky. It is NOT a boot failure; the chip still resets/runs.
  But that same flakiness *does* occasionally corrupt a flash write — hence
  always `verify` the boot-critical pages after a download.
- **`probe-rs run` (RTT streaming) produces no captured output in some headless
  shells.** Use `probe-rs download` + `probe-rs reset`, then verify behaviour
  over the network. To read the post-boot log, `probe-rs attach <elf>` (the
  16 KiB RTT buffer — `DEFMT_RTT_BUFFER_SIZE` in `.cargo/config.toml` — retains
  the boot history). To read the *bootloader's* log specifically, attach with the
  bootloader ELF (its own RTT block).
- **`probe-rs read` is RAM-only** — to inspect flash, use `probe-rs verify`
  against a known `.bin` (note: `objcopy -O binary` fills section gaps with
  `0x00` while erased flash is `0xFF`, so a *whole-image* verify can "fail" on
  the unused tail — verify the first N KiB, or a specific page, instead).

### Finding the node, and OTA

- The node answers **TCP/UDP services, not ICMP** (smoltcp default) — don't ping
  as a liveness check. After an OTA reboot it may take a **new DHCP lease**, so
  don't assume the old IP: `nmap -p 8023 --open -n <lan-cidr>` finds it by its
  telnet port, or use `pico-node.local` (mDNS).
- OTA: `GET http://<node-ip>/version` (running build tag); `POST` the raw
  `pico-node-app.bin` to `http://<node-ip>/firmware` (STA mode, :80) → swap →
  trial → auto-rollback. AP mode keeps the captive portal on :80 instead.

### Onboarding a same-for-everyone image (the real flow)

A credential-free release boots into **config-only AP mode**: SSID `pico-setup`,
WPA2 passphrase `packetradio`. Join it from a phone → captive portal pops
(or `http://192.168.4.1/`) → enter callsign + WiFi + alias → Save & reboot →
joins your WiFi from **flash-stored** config (not compiled). This is how the
bench should be set up to dogfood the actual release; compiled-in creds
(`option_env!` via a gitignored `~/.cargo/config.toml [env]`) are a dev
convenience only, and stored config overrides them at boot.

### CYW43 gotchas

- **SAE wedge:** cyw43's default join (`Wpa2Wpa3` → SAE) hangs forever against a
  WPA2-PSK AP and the wedge **persists across `probe-rs reset`** (the RP2040
  resets, the radio doesn't) — mimics hardware/SWD faults. Fixed in `net.rs`
  with explicit `JoinAuth::Wpa2` + a join timeout; recovery from a wedge is a
  **physical power-cycle** of the Pico.
- **Multicast RX filter:** `Stack::join_multicast_group` is not enough — the chip
  filters RX by MAC, so `Control::add_multicast_address` must add the group MAC
  (e.g. `01:00:5e:00:00:fb` for mDNS) or multicast never reaches smoltcp.
