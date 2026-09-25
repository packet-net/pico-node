# Hilltop node: two radios, two bands, autonomous middle hop

*Plan written 2026-09-25, after the radio-first restructure (PR #84). Status: proposed, not started.*

## Objective

A pico-node on a hilltop, unattended, with two radios on different bands (for example 2 m and 70 cm), that lets two stations who cannot hear each other talk through it without anyone touching the node:

- **A plain station** (a terminal user, no node software) on band A connects to the node and types `C <call>` for a station on band B. The node works out which port that station is on and connects onward. No port number needed when the station has been heard.
- **A NET/ROM node** on band A reaches a NET/ROM node on band B by alias (`C <alias>`), with the hilltop node as the invisible middle hop: it hears NODES on both bands, advertises each side to the other, keeps interlinks up on both, and forwards the L3/L4 traffic between them.
- **Unattended:** it recovers from a power cut or a hang on its own, relinks, and can be administered over the air by the sysop only.

Digipeating is out of scope by design: this is a session-oriented network, so the node terminates links and relays at L3/L4, never by repeating frames.

## Where we are (after PR #84)

Already in place:

- One node task (`node.rs`) runs sessions, the console, `C` onward, NET/ROM (one routing table, NODES, L4 circuits, interlinks, INP3) for every **port** (`ports/`). A link is pinned to the port it was made on; NODES go out on every usable port; routes carry the port they were learned on.
- `C <call>` already picks the port the target was last heard on, else the first usable port.
- The NinoTNC driver (port 0) sets the TNC's mode and KISS parameters, verifies them, gates traffic on supported firmware, and can update the TNC's firmware over its UART. Verified on air.
- KISS-over-TCP (port 1) gives a second, hardware-free port for lab testing against net-sim.

The gaps, in order of how much they block the objective:

| Gap | Why it matters here |
|---|---|
| Sessions keyed by callsign only | The same station heard on both bands would share one session slot. Keys must include the port. |
| One NinoTNC driver, one set of TNC state | Status, monitor, command queue, saved settings, web page and TNC firmware update all assume a single TNC. |
| Only one free hardware UART | UART1 drives the NinoTNC, UART0 is reserved for Tait CCDI. The second TNC needs UART0 or a PIO UART. |
| Link and NET/ROM parameters are node-wide | Two bands at different bit rates want different T1, window, PACLEN, NET/ROM port quality and MINQUAL. |
| RF callers can use SET / SAVE / REBOOT | Anyone who connects over the air can reconfigure or reboot an unattended node. |
| No hardware watchdog | A firmware hang on a hilltop means a site visit. |
| 4 sessions, 8-entry heard table, 16 KB heap | A middle hop carries at least two links per conversation plus two interlinks. |
| Console has no port awareness | No `C <port> <call>`, no per-port heard list, no per-port status. |

## Phases

Each phase ends green on the existing gates (host tests, clippy, no_std build, firmware build and embedded-test link, layout and parity guards) and is deployable on its own.

### Phase 0: hilltop hygiene (independent of the second radio)

Do this first; it matters for a single-radio remote node too.

1. **Hardware watchdog.** Feed the RP2040 watchdog from the main heartbeat loop, with a timeout that survives the longest legitimate blocking operation (flash erase during TNC image staging; TNC firmware update runs async and must keep feeding). Record the reset cause at boot and show it on the panel and in `I`nfo.
2. **Sysop authority over the air.** Config commands (`SHOW` / `SET` / `SAVE` / `REBOOT`) refused on AX.25 sessions unless the session has authenticated. Use a challenge-response against a shared secret set from the web panel or telnet (never sent in clear on air, and never replayable), in the spirit of BPQ's `PASSWORD`. Telnet and the web panel stay as they are (LAN admin).
3. **Station identification.** A per-port ID beacon (UI to `ID`, the node call and alias, a short text) at a configurable interval (BPQ `IDINTERVAL`), so the node identifies on every band it transmits on even when NODES are hourly.
4. **Unattended recovery proven.** A test: power-cycle, confirm the node relinks to its neighbours from flash-restored routes without intervention (already seen on 2026-09-25 for one port).

### Phase 1: port-aware sessions and console

1. **Session key = (port, local, peer)** in `SessionManager` (core, host-tested): `index_of`, `session_for`, `post*`, `reap`, `take_upward` take the port. A station connected on both bands at once gets two sessions.
2. **Heard table per port** in the node task (callsign, port, last heard, frames), sized for two bands (for example 32 entries), feeding both `C` resolution and a new `MH [port]` console command.
3. **Console:** `P`orts (name, band, TNC mode, usable or not, frames heard/sent), `C <port> <call>` to force a port, `C <call>` unchanged (last heard port, else ask the user to name one when both ports are up rather than guessing).
4. **Session capacity:** raise `MAX_SESSIONS` from 4 to what RAM allows (target 12), after measuring the per-session heap cost on target. Raise the heap from 16 KB if needed; there is about 100 KB of free RAM.

Acceptance: host tests for per-port session keys (same callsign on two ports, independent state), and a two-port lab run on net-sim (Phase 5 harness) showing `C <port> <call>` and `MH`.

### Phase 2: two NinoTNCs

1. **Per-port driver instances.** `ports::ninotnc::task` becomes a pool of two, each owning its UART, its `SerialKissModem`, its TNC state and its monitor tag. `tnc.rs` statics become arrays indexed by port.
2. **Per-port saved settings.** The TNC keys (`TNC_MODE`, `TXDELAY`, `PERSIST`, `SLOTTIME`, `TXTAIL`, `DUPLEX`) become per port (`P1.TXDELAY`, `P2.TXDELAY`, ...) in new flash tags; the existing single-port tags migrate to port 1 on first boot.
3. **Web panel.** The NinoTNC page gets a port selector; the front-page monitor tags lines by port (`[2m]`, `[70cm]`). TNC firmware update works per port (one staged image, flashed to whichever TNC is selected; the chip check still applies per TNC).

### Phase 3: the second serial link

1. **PIO UART on PIO1** (PIO0 drives the WiFi chip): a TX and an RX state machine at 57600 8N1 implementing the same `ByteStream` as the hardware UART, with an RX FIFO drained into a ring buffer so KISS bytes are never lost while the executor is busy. Host-side: none; on-target: a loopback test in the embedded-test suite.
2. **Pin choice.** The NinoBLE board has one NinoTNC header, so the second TNC is wired by hand or through a new carrier board. Free GPIOs on NinoBLE Rev5 include GP9 to GP12 (the unused SD card pins) and GP0/GP1 (UART0) if the Tait CCDI link is not fitted.
3. **Configuration.** Which transport each port uses (UART1, UART0, PIO1) is a build-time board profile, not a runtime setting.

### Phase 4: per-port link and NET/ROM parameters

1. **Link parameters per port:** N1 (PACLEN), window k, T1/T2/T3, N2, applied to each new session from the port it is on (the C# per-port `Ax25PortParams` model). A slow band gets a longer T1 and a smaller window.
2. **NET/ROM per port:** port quality (the quality a neighbour heard on that port is given), MINQUAL, NODESPACLEN, and whether NODES are broadcast on that port (BPQ `QUALITY`, `MINQUAL`, `NODESPACLEN` per port). Different qualities per band are what make the routes through the hilltop sensible.
3. **Transit forwarding across ports.** Confirm (and test) that the L3 connector forwards a datagram arriving on an interlink on port 1 out of an interlink on port 2, with TTL decrement, and that L4 circuits between the two outer nodes pass through without the hilltop terminating them.

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
5. **24-hour soak** with periodic traffic both ways; no heap growth, no stuck sessions (reported through the panel and MQTT).

### Phase 6: on air, then the hill

1. **On air at home:** two real radios on two bands (GB7RDG has ports on both 2 m and 70 cm, so it can play both outer stations on separate ports). Repeat tests 1 to 4 over RF.
2. **Hilltop checklist:** power budget and brown-out behaviour, watchdog reset tested, sysop authentication tested over the air, ID beacons on both bands, WiFi AP mode for on-site access only, a known-good firmware image and rollback proven, NODES interval and qualities set for the local network.

## Out of scope for this plan

- **Digipeating** (design rule, see the objective).
- **Firmware update over RF** (see `docs/OTA-RADIO.md`). With sysop authentication in place it becomes practical, and it is the natural follow-on for a node nobody can reach by WiFi, but it is not needed for the middle-hop objective.
- **More than two radio ports.** The design is N-port, but the RP2040's RAM and pins make two the practical target.
- **AX.25 flow control on the telnet relay** (RNR while the relay backlog is high): useful, but independent of this plan.

## Risks

- **RAM.** More sessions, larger heard table, two TNC drivers and per-port state all cost RAM. Measure per-session heap on target in Phase 1 before choosing limits.
- **PIO UART reliability.** The KISS link must not drop bytes while the executor is busy (flash erase, WiFi bursts). The RX path needs a FIFO-fed ring buffer and an on-target stress test.
- **Co-located transmitters.** Two radios on one hilltop can desense each other. That is RF engineering (filtering, antenna separation, band choice), not firmware, but it will show up as "the node loses frames" and should be ruled out on site.
- **Unauthenticated admin over RF** (Phase 0) is a hard prerequisite for deployment, not a nice-to-have.

## Open questions for Tom

1. Which two bands and NinoTNC modes are intended for the hill?
2. Keep the Tait CCDI link (then the second TNC uses a PIO UART), or give UART0 to the second TNC?
3. Same callsign on both ports (typical BPQ), or a distinct SSID per port?
4. Sysop authentication: a BPQ-style password challenge, or something else?
5. Is there WiFi on the hill (for the web panel), or is admin over RF only after installation?
