# pico-node

A Rust firmware for the Raspberry Pi Pico W (RP2040) that makes a **radio-first packet node**: a Pico W wired to a NinoTNC, on the air, with the node logic of the C# node host in [`packet-net/packet.net`](https://github.com/packet-net/packet.net), built on the AX.25 v2.2 SDL state machine from [`packet-net/ax25sdl`](https://github.com/packet-net/ax25sdl).

On the air (the node's job):

- **The radio port**: a NinoTNC on a direct UART link (NinoBLE Rev5, J5; no USB chip needed). Connected-mode AX.25 sessions with the node console, `C` onward, NET/ROM (NODES, L4 circuits, interlinks, INP3). The node sets the TNC's mode and KISS parameters, verifies them, and can update the TNC's firmware.
- **KISS-over-TCP** (optional second port): net-sim's emulated RF channel, for running the node without radio hardware.
- **Tait CCDI** radio control on a second UART.

Over WiFi, administration only:

- **Web panel**: node config, the NinoTNC page (mode, KISS parameters, test frame, live monitor, TNC firmware update), node firmware update (OTA).
- **Telnet console**: the node console for the sysop, including `C` onward over the air.

AXUDP (AX.25 over UDP) was removed on 2026-09-25: it was the bring-up path before a TNC was attached. See [`docs/PLAN.md`](docs/PLAN.md) §11.

## Read first

**[`docs/PLAN.md`](docs/PLAN.md)** is the living plan: architecture, the module breakdown, the SDL integration story, the hands-free dev cycle (build → flash via probe-rs → defmt/RTT logs), the host-side test strategy, the package-approval gate, and the "when the hardware arrives" checklist + blockers.

## Layout

- `crates/ax25-node-core` - portable, `no_std`-able logic (KISS and the NinoTNC extensions, AX.25 codec, CRC, console, NET/ROM, the SDL runtime, the traffic monitor). Exactly one external dependency: the generated `ax25sdl` tables, as a local sibling path dependency. Host-tested with `cargo test` today.
- `crates/ax25-node-fw` - the thin RP2040 / Embassy firmware binary (standalone, workspace-excluded; builds for thumbv6m and is proven on hardware - see `docs/PLAN.md`'s amendment log and `docs/OTA.md`):
  - `node.rs` - the node task: sessions, console, NET/ROM for every radio port.
  - `ports/` - the radio ports: `ninotnc.rs` (port 0, the NinoTNC) and `kiss_tcp.rs` (port 1, optional).
  - `admin/` - the telnet console and its connect relay; `ota.rs` / `webui.rs` / `tnc.rs` - the web panel.

## Build + test the portable core (works now, offline, no hardware)

```sh
cargo test                                                   # host unit tests (default std feature)
cargo build -p ax25-node-core --no-default-features --features alloc   # prove the no_std posture
```

## Build the firmware (requires the toolchain in docs/PLAN.md §8)

```sh
cargo build --manifest-path crates/ax25-node-fw/Cargo.toml --release
cargo run   --manifest-path crates/ax25-node-fw/Cargo.toml --release   # flash + stream defmt over SWD
```
