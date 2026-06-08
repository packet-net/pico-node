# Provisioning & configurability — the out-of-box experience (design)

*Written 2026-06-07 at Tom's direction, while the firmware still takes its
config from compile-time `option_env!`. This is the target experience and the
staged plan toward it — groundwork to keep in mind on every PR that touches
config, net bring-up, or transports. Not scheduled as a single piece of work.*

## The experience we are building toward

One **same-for-everyone firmware image** (releases become directly flashable):

1. **Fresh flash, no config** → the node raises a **WiFi access point** with a
   well-known passphrase. AP SSID is discoverable and unique-ish, e.g.
   `pico-node-XXXX` (chip-id suffix until a callsign is set, callsign after).
2. Connecting to that AP triggers a **captive-portal flow** (phone/laptop pops
   the portal automatically): set callsign + SSID, alias, grid, modem/port
   options (AXUDP peers, KISS targets, NODES origination), and *optionally*
   join a WiFi network (scan + pick + passphrase).
3. Config is **persisted to flash**. Subsequent boots join the configured WiFi
   directly (STA mode).
4. **Joining WiFi is optional.** A node with no STA config runs in **remote
   mode**: fully operational (RF/serial transports, NET/ROM) with the config
   AP still available — the remote-hilltop case. The same firmware is equally
   at home on a LAN.
5. Reconfiguration is always possible: in remote mode the AP is always there;
   in LAN mode the portal is reachable over the LAN (`pico-node.local`), and a
   fallback returns the AP if the configured WiFi can't be joined.

## Constraints discovered so far (verified against the pinned stack)

- **AP mode exists**: `cyw43::Control::{start_ap_open, start_ap_wpa2,
  close_ap}` (cyw43 0.7). **AP+STA concurrently does NOT** — the driver
  exposes one interface. So "AP always available" on a LAN-joined node is not
  literal: LAN nodes get the portal *over the LAN* instead, plus an automatic
  AP fallback when STA join fails (see mode machine below).
- **Flash persistence**: `embassy_rp::flash::Flash` (4 KiB erase sectors).
  Reserve the top two sectors (8 KiB) of the 2 MiB flash via `memory.x` for a
  config store — two sectors so writes are atomic (write new, then invalidate
  old; a torn write can't lose the previous config). Candidate: the
  `sequential-storage` crate (tweedegolf; embedded-storage compatible) rather
  than hand-rolling wear/validity handling; decide at implementation time.
- **Captive portal needs three small servers** on the AP subnet, all feasible
  no_std (the mDNS responder set the pattern):
  - **DHCP server** (~150 lines or `edge-dhcp`): hand out 192.168.4.x.
  - **DNS catch-all** (~50 lines): every A query → 192.168.4.1, which is what
    makes phones pop the portal (their connectivity probes get redirected).
  - **HTTP server** on the existing `embassy-net` TCP stack: one form page,
    one POST handler, a 302 for the OS probe URLs. No TLS (portal is local).
- **smoltcp/embassy-net work unchanged in AP mode** — the stack doesn't care
  which side associates.

## Mode machine (boot decision)

```
            ┌────────────── no stored config ──────────────┐
boot ── read config ── STA configured? ── yes ── join WiFi ── up (LAN mode)
            │                │ no                  │ join fails N times
            │                ▼                     ▼
            └──────────► AP mode (config portal + full node services)
                          "remote mode" if callsign etc. already set,
                          "out-of-box" if not
```

- Remote mode = AP up + all transports that don't need STA (KISS serial, RF
  via TNC, NET/ROM over those) fully operational. AXUDP/KISS-TCP/telnet bind
  on the AP subnet too — a laptop joined to the node's AP can use everything.
- LAN mode = STA joined; portal served on the LAN (and config still editable
  via authenticated telnet/console commands later).
- Fallback: K consecutive STA join failures (AP gone, password rotated) drops
  back to AP mode so the node is never unreachable. The join loop in `net.rs`
  already has the retry/backoff scaffolding for this.

## Config schema (v1 sketch — versioned from day one)

`struct StoredConfig { version, crc }` over: callsign+ssid, alias, grid,
node hostname; wifi: Option<{ssid, psk}>; mode hints; axudp {port, peers[]},
kiss_tcp Option<target>, kiss_serial {baud}, telnet {port}; netrom
{enabled, origination params}; ap {passphrase (default well-known), channel}.
Serialization: `postcard` (no_std, tiny) + CRC32 + length, version-gated for
migrations.

## Security posture (deliberate, documented)

- Well-known AP passphrase is a conscious tradeoff for field usability
  (hilltop boxes have physical security, not secrecy); the portal allows
  changing it. Amateur radio traffic is cleartext by law anyway — the AP
  passphrase guards *configuration*, not traffic.
- The portal must rate-limit and only run plain HTTP on the AP/LAN — no
  internet exposure story at all.

## Staging (each lands as its own PR, in roughly this order)

1. **Flash config store** — reserve sectors, postcard+CRC codec, host-tested
   round-trip + torn-write tests; firmware reads it at boot with
   `option_env!` values demoted to factory-default fallbacks (dev rigs keep
   working). *This alone makes releases flashable-and-configurable via a
   temporary console command to write config.*
2. **Console `SET`/`SHOW` commands** (telnet + AX.25, host-tested in core) to
   edit stored config — full configurability before the portal exists.
3. **AP mode + mode machine** — boot decision, join-failure fallback, AP with
   derived SSID; node services bound on the AP subnet.
4. **Captive portal** — DHCP + DNS catch-all + HTTP form; the OS-probe 302s.
5. **Polish** — WiFi scan-and-pick in the portal, config export/import, AP
   passphrase change, factory-reset gesture (hold BOOTSEL at boot? GPIO
   strap? decide on hardware).

Steps 1–2 deliver most of the practical value (no more compile-time config)
and de-risk 3–5, which are UX.

## Status (2026-06-07)

- **Steps 1–2 DONE** (PR #21): flash config store + console SHOW/SET/SAVE/REBOOT.
- **Step 3 DONE** (this PR): the mode machine + AP mode + DHCP server. Boot tries
  STA when WiFi is configured (bounded, 3 attempts) and falls back to the config
  AP otherwise; AP SSID is `pico-<callsign>`, WPA2 with the `ap_passphrase`
  default, gateway 192.168.4.1, DHCP pool 192.168.4.10+. Verified on hardware:
  a laptop associated to `pico-M9YYY-9` and received `192.168.4.10` from the
  node's DHCP server.
- **Step 4 DONE** (PR for it): DNS catch-all (every name -> 192.168.4.1) + the
  HTTP config form. Verified on hardware: a laptop joined pico-M9YYY-9, got
  192.168.4.10, the DNS resolved captive.apple.com -> 192.168.4.1 (the iOS
  portal-pop probe), GET / served the config form, and POST /save wrote the
  submitted config to flash + rebooted — the change (alias) survived and applied.
- **In-place reconfiguration DONE** (web-panel PR): a deployed STA-mode node
  serves the same config form (pre-filled with its current values) on its web
  panel at `http://<node-ip>/` (`POST /save`), so you can change callsign / WiFi /
  alias / MQTT without re-onboarding or BOOTSEL. The node also offers a **"Switch
  to setup AP"** action (`POST /apmode`) that reboots it into the
  `pico-<callsign>` config AP to move it to a different WiFi — a **sticky** flag
  (`FORCE_AP`, `src/config_store.rs`) keeps it in setup mode across reboots until
  a config save clears it. This is the STA→AP return path that previously
  required a probe/erase.
- **Step 5 (polish) remaining**: WiFi scan-and-pick in the portal, AP passphrase
  change, factory-reset gesture, config export/import.

The same-for-everyone flashable image is now real: a fresh node with no stored
config raises its AP, a phone/laptop pops the portal, you fill in callsign +
WiFi, and it reboots onto your network — no toolchain, no build env.
