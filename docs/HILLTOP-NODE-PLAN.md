# Hilltop node: two radios, autonomous middle hop

*Plan written 2026-09-25, after the radio-first restructure (PR #84); updated the same day with Tom's decisions. Status: agreed, not started.*

## Objective

A pico-node on a hilltop, unattended and without WiFi, with two radios, that lets two stations who cannot hear each other talk through it without anyone touching the node:

- **A plain station** (a terminal user, no node software) heard on one port connects to the node and types `C <call>` for a station heard on the other. The node works out which port that station is on and connects onward.
- **A NET/ROM node** on one side reaches a NET/ROM node on the other side by alias (`C <alias>`), with the hilltop node as the invisible middle hop: it hears NODES on both ports, advertises each side to the other, keeps interlinks up on both, and forwards the L3/L4 traffic between them.
- **Unattended:** it recovers from a power cut or a hang on its own, relinks, and can be administered over the air by the sysop only. A hiking sysop can also walk up and use the node's WiFi access point.

## Design rules (decided 2026-09-25)

- **Ports are equal.** The node has no notion of bands. Each port has a **sysop-defined id** (a short name the sysop chooses) that appears in the console, the monitor and the web panel. Which band a radio is on, and which NinoTNC mode it runs, is the sysop's choice and nothing the node reasons about. There is no primary port and no port preference order.
- **One node callsign-SSID** on every port. The port a connection arrived on changes nothing about how it is handled.
- **No digipeating.** This is a session-oriented network: the node terminates links and relays at L3/L4, never by repeating frames.
- **No WiFi on the hill.** The node must run fully without infrastructure WiFi (it already falls back to its own access point when it cannot join a network). The access point stays, for a hiking sysop doing maintenance on site. Everything else is administered over the air.
- **Over-air authority by one-time pad.** Sysop commands over RF are unlocked with single-use codes from a pad; a code heard on air is worthless afterwards.
- **The Tait CCDI link stays** on UART0 (not yet exercised, but useful for SWR and RSSI monitoring). It is not part of this plan; it only fixes where the second TNC connects.

## Where we are (after PR #84)

Already in place:

- One node task (`node.rs`) runs sessions, the console, `C` onward, NET/ROM (one routing table, NODES, L4 circuits, interlinks, INP3) for every **port** (`ports/`). A link is pinned to the port it was made on; NODES go out on every usable port; routes carry the port they were learned on.
- `C <call>` picks the port the target was last heard on.
- The NinoTNC driver (port 0) sets the TNC's mode and KISS parameters, verifies them, gates traffic on supported firmware, and can update the TNC's firmware over its UART. Verified on air.
- KISS-over-TCP (port 1) gives a second, hardware-free port for lab testing against net-sim.
- Without WiFi the node comes up as its own access point with the web panel, and still runs every radio function.

The gaps:

| Gap | Why it matters here |
|---|---|
| Sessions keyed by callsign only | A station heard on both ports would share one session slot. |
| One NinoTNC driver, one set of TNC state | Status, monitor, command queue, saved settings, web page and TNC firmware update all assume a single TNC. |
| No second serial link | UART1 drives the NinoTNC and UART0 is the Tait link, so the second TNC needs a PIO UART. |
| Ports have fixed names and a preference order | `C <call>` for an unheard station falls back to "the first usable port", which breaks "ports are equal". Ports need sysop ids. |
| Link and NET/ROM parameters are node-wide | Two radios at different bit rates want different T1, window, PACLEN, NET/ROM port quality and MINQUAL. |
| Anyone on RF can use SET / SAVE / REBOOT | An unattended node can be reconfigured or rebooted by any caller. |
| No hardware watchdog | A firmware hang on a hilltop means a site visit. |
| 4 sessions, 8-entry heard table, 16 KB heap | A middle hop carries at least two links per conversation plus two interlinks. |
| Console has no port awareness | No `C <port> <call>`, no per-port heard list, no per-port status. |

## Phases

Each phase ends green on the existing gates (host tests, clippy, no_std build, firmware build and embedded-test link, layout and parity guards) and is deployable on its own.

### Phase 0: remote-node hygiene (independent of the second radio)

Do this first; it matters for a single-radio remote node too.

1. **Hardware watchdog.** Feed the RP2040 watchdog from the main heartbeat loop, with a timeout that survives the longest legitimate blocking operation (a flash erase while staging a TNC image; the TNC firmware update runs async and must keep it fed). Record the reset cause at boot and show it on the panel and in `I`nfo.
2. **Sysop authority over the air, by one-time pad.**
   - The node generates a pad of numbered single-use codes (for example 100 codes of 8 characters from the RP2040's hardware random source) when the sysop asks, from the web panel (AP mode on site, or the LAN before deployment) or telnet. It shows the pad once, for the sysop to print or save, and keeps only a hash of each code in flash.
   - On an AX.25 session, `SYSOP` makes the node issue a challenge naming a code number it has not used; the sysop replies with that code. A correct reply unlocks the config commands (`SHOW` / `SET` / `SAVE` / `REBOOT`, and later firmware update over RF) for the rest of that session only.
   - Every code is burned when it is challenged, whether the answer is right or wrong, so nothing heard on air can be replayed and guessing gains nothing. After a few failed attempts `SYSOP` is refused for a while (lockout), and failures are logged.
   - When the pad runs low the node says so after a successful login; the sysop generates a new pad on the next site visit or over an authenticated RF session.
   - Without authority, config commands are refused on AX.25 sessions. Telnet (LAN) and the web panel keep their current behaviour.
3. **Station identification.** A per-port ID beacon (UI to `ID`, the node call and alias, a short text) at a configurable interval (BPQ `IDINTERVAL`), so the node identifies on every port it transmits on even when NODES are hourly.
4. **Unattended recovery proven.** A test: power-cycle, confirm the node relinks to its neighbours from flash-restored routes without intervention (already seen on 2026-09-25 for one port), and that a node configured for WiFi it cannot reach comes up as its access point with everything else running.

### Phase 1: equal ports, port-aware sessions and console

1. **Sysop-defined port ids.** Each port gets an id set by the sysop (a short name, saved in config). The console, monitor, web panel and route display use it. No port preference order remains anywhere in the node.
2. **Session key = (port, peer)** in `SessionManager` (core, host-tested), so one station connected on both ports at once gets two independent sessions. The node's callsign is the same on every port, and a session is handled the same whichever port it is on.
3. **Heard table per port** in the node task (callsign, port, last heard, frames), sized for two busy ports (for example 32 entries), feeding both `C` resolution and a new `MH [port]` console command.
4. **Console:** `P`orts (id, TNC mode, usable or not, frames heard and sent), `C <port> <call>` to name a port, and `C <call>` using the port the station was last heard on. A station not heard on any port gets a clear "not heard; use C <port> <call>" rather than a guess.
5. **Session capacity:** raise `MAX_SESSIONS` from 4 to what RAM allows (target 12), after measuring the per-session heap cost on target. Raise the heap from 16 KB if needed; there is about 100 KB of free RAM.

Acceptance: host tests for per-port session keys (the same callsign on two ports, independent state) and for `C` resolution, and a two-port lab run on net-sim (Phase 5 harness) showing `C <port> <call>` and `MH`.

### Phase 2: two NinoTNCs

1. **Per-port driver instances.** `ports::ninotnc::task` becomes a pool of two, each owning its serial link, its `SerialKissModem`, its TNC state and its monitor tag (the port id). `tnc.rs` statics become arrays indexed by port.
2. **Per-port saved settings.** The TNC keys (`TNC_MODE`, `TXDELAY`, `PERSIST`, `SLOTTIME`, `TXTAIL`, `DUPLEX`) become per port in new flash tags; the existing single-port tags migrate to the first port on first boot.
3. **Web panel.** The NinoTNC page gets a port selector; the front-page monitor tags lines with the port id. TNC firmware update works per port (one staged image, flashed to whichever TNC is selected; the chip check still applies per TNC).

### Phase 3: the second serial link

1. **PIO UART on PIO1** (PIO0 drives the WiFi chip; UART0 stays with the Tait link): a TX and an RX state machine at 57600 8N1 implementing the same `ByteStream` as the hardware UART, with the RX FIFO drained into a ring buffer so KISS bytes are never lost while the executor is busy. On-target: a loopback test in the embedded-test suite, and a stress test during flash erases and WiFi bursts.
2. **Pins.** The NinoBLE board has one NinoTNC header, so the second TNC is wired by hand or through a new carrier board. Free GPIOs on NinoBLE Rev5 include GP9 to GP12 (the unused SD card pins).
3. **Configuration.** Which serial link each port uses is a build-time board profile, not a runtime setting.

### Phase 4: per-port link and NET/ROM parameters

1. **Link parameters per port:** N1 (PACLEN), window k, T1/T2/T3, N2, applied to each new session from its port (the C# per-port `Ax25PortParams` model). A slow port gets a longer T1 and a smaller window.
2. **NET/ROM per port:** port quality (given to a neighbour heard on that port), MINQUAL, NODESPACLEN, and whether NODES are broadcast on that port (BPQ `QUALITY`, `MINQUAL`, `NODESPACLEN` per port). These are what make routes through the hilltop sensible.
3. **Transit forwarding across ports.** Confirm (and test) that the L3 connector forwards a datagram arriving on an interlink on one port out of an interlink on the other, with TTL decrement, and that L4 circuits between the two outer nodes pass through without the hilltop terminating them.

### Phase 5: lab proof with net-sim (no radios needed)

A net-sim topology with **two channels** and three stations:

- station A on channel 1 only, station C on channel 2 only (they cannot hear each other);
- the pico-node on both channels (two KISS-TCP ports in the lab build, standing in for the two NinoTNCs);
- A and C run pdn (or LinBPQ), so both the plain-user and the NET/ROM paths can be exercised.

Tests:

1. **Plain user, autonomous port choice:** a user on A connects to the pico, types `C <C-call>`, and reaches C (the pico picks channel 2 from its heard table).
2. **NET/ROM middle hop:** A learns C's alias from the pico's NODES on channel 1; `C <C-alias>` from A's console reaches C's node, with the pico forwarding L3/L4 and nobody touching the pico.
3. **Link loss and recovery:** add loss on one channel; sessions recover by retransmission; an interlink torn down by N2 re-establishes.
4. **Power cycle:** restart the pico mid-soak; both interlinks and the routes come back from flash.
5. **Sysop over RF:** unlock with a pad code, change a setting, reboot; a replayed code and a wrong code are both refused and burned.
6. **24-hour soak** with periodic traffic both ways; no heap growth, no stuck sessions.

### Phase 6: on air, then the hill

1. **On air at home:** two real radios on two ports (GB7RDG has several radio ports, so it can play both outer stations). Repeat tests 1 to 5 over RF.
2. **Hilltop checklist:** power budget and brown-out behaviour, watchdog reset tested, the pad printed and tested over the air, ID beacons on both ports, the access point working for a site visit, a known-good firmware image and rollback proven, NODES interval and port qualities set for the local network.

## Out of scope for this plan

- **Digipeating** (design rule).
- **Firmware update over RF** (see `docs/OTA-RADIO.md`). With no WiFi on the hill, updates meanwhile mean a site visit (the access point takes a node firmware upload). With the one-time pad in place, RF firmware update is the natural next plan.
- **The Tait CCDI link** (kept on UART0; exercising it for SWR and RSSI monitoring is separate work).
- **More than two radio ports.** The design is N-port, but the RP2040's RAM and pins make two the practical target.
- **AX.25 flow control on the telnet relay** (RNR while the relay backlog is high): useful, but independent of this plan.

## Risks

- **RAM.** More sessions, a larger heard table, two TNC drivers and per-port state all cost RAM. Measure per-session heap on target in Phase 1 before choosing limits.
- **PIO UART reliability.** The KISS link must not drop bytes while the executor is busy (flash erase, WiFi bursts). The RX path needs a FIFO-fed ring buffer and an on-target stress test.
- **Access point power.** Keeping the access point up costs power on a battery or solar site. Measure it in the Phase 6 power budget; if it matters, consider bringing the AP up only for a while after boot or on a button press, without losing the maintenance path.
- **Co-located transmitters.** Two radios on one hilltop can desense each other. That is RF engineering (filtering, antenna separation, frequency choice), not firmware, but it will show up as lost frames and should be ruled out on site.
- **Unauthenticated admin over RF** (Phase 0) is a hard prerequisite for deployment.
