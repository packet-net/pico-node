# LinBPQ interop container

A real G8BPQ node in Docker, used to validate pico-node's AXUDP interop
(HW-BRINGUP §6 "AXUDP vs a real node" — self-hosted instead of lab-coordinated).

## Run

```sh
# 1. Fetch the LinBPQ binary (not redistributable here — G8BPQ's licence):
curl -L -o linbpq64 https://www.cantab.net/users/john.wiseman/Downloads/Beta/linbpq64

# 2. Fill in the Pico's IP:
sed 's/PICO_IP/<pico-ip>/' bpq32.cfg.template > bpq32.cfg

# 3. Build + run (host network so AXUDP shares the LAN):
docker build -t linbpq-interop .
docker run -d --name linbpq --network host linbpq-interop
```

Note: if the host's root fs is overlay (live/overlayroot systems), Docker needs
`{"storage-driver": "vfs"}` in /etc/docker/daemon.json.

Sysop console: `nc 127.0.0.1 8011`, login `tom` / `p`, then e.g. `C 1 M0LTE-1`
to connect to the Pico's node console over AX.25, `MH 1` for the heard list.

## A second node (3-node topologies)

Copy the directory, then in the copy's `bpq32.cfg`: change `NODECALL`/
`NODEALIAS`, the port's `UDP 10093` to a free port (e.g. `UDP 10094`), and
`TCPPORT` (e.g. 8012) — but keep the `MAP ... UDP 10093` line pointing at the
Pico's real listen port. Build/run with different image/container names.

## Findings this validated (2026-06-07)

- LinBPQ 6.0.25 AXIP speaks **AXIP-with-CRC** (trailing CRC-16/X.25, low byte
  first) and ignores FCS-less datagrams — hence `include_fcs: true` in the
  firmware's AxudpConfig. (The "FCS-less is the LinBPQ-accepted default" note
  in core::axudp predates this wire check.)
- `QUALITY=192` on the port and `BROADCAST NODES` in the BPQAXIP CONFIG are
  both required before LinBPQ sends NODES broadcasts to B-flagged maps.
- Full session interop: SABM/UA, I-frames + RR both ways, DISC/UA — LinBPQ's
  `C 1 M0LTE-1` lands at the pico-node console prompt, and the reverse hop
  (pico-node console `C <node>`) lands at LinBPQ's prompt — including the full
  3-node chain BPQ1 → Pico → BPQ2.
- **`INFOMSG:` must be configured** — LinBPQ tears the stream down when a user
  sends `I` and no INFOMSG exists (cost a whole afternoon: every failing test
  typed `I`, every passing one typed `N`).
- **One MAP per peer IP.** Two maps to the same address (e.g. a static map
  plus an AUTOADDMAP entry for a second callsign from that IP) poison LinBPQ's
  AXIP layer: broadcasts duplicate, CTEXT transmissions vanish, streams never
  attach. Outgoing pico-node connects therefore use the node callsign (which
  matches the static map) — and a node also can't hold two L2 links under one
  callsign pair, so same-node loop connects (back to the node the user came
  from) are a peer-side limitation; hop to a *different* node instead.
- LinBPQ's first CTEXT transmission on links the Pico initiates never hits the
  wire (its T1 retry delivers it ~5–10 s later). Peer-side quirk; harmless.
