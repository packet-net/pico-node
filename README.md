# pico-node

A Rust firmware for the Raspberry Pi Pico W (RP2040) that brings a packet-radio node to feature parity with the C# node host in [`m0lte/packet.net`](https://github.com/m0lte/packet.net), built on the AX.25 v2.2 SDL state machine from [`m0lte/ax25sdl`](https://github.com/m0lte/ax25sdl).

Four capabilities, mirroring the C# node:

1. **AXUDP** — AX.25-over-UDP, node↔node over WiFi (BPQ-compatible AXIP/AXUDP).
2. **KISS-over-TCP** — to net-sim (the emulated RF channel) over WiFi.
3. **KISS-over-serial** — to a NinoTNC, over a direct UART link.
4. **Telnet command console** over WiFi.

## Read first

**[`docs/PLAN.md`](docs/PLAN.md)** is the living plan: architecture, the module breakdown mapping to the four capabilities, the SDL integration story, the hands-free dev cycle (build → flash via probe-rs → defmt/RTT logs), the host-side test strategy, the package-approval gate, and the "when the hardware arrives" checklist + blockers.

## Layout

- `crates/ax25-node-core` — portable, `no_std`-able, **zero-dependency** logic (KISS, AXUDP, AX.25 codec, CRC, console, SDL glue). Host-tested with `cargo test` today.
- `crates/ax25-node-fw` — the thin RP2040 / Embassy firmware binary (standalone, workspace-excluded; built once the embedded toolchain is in place).

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
