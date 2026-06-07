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

## Findings this validated (2026-06-07)

- LinBPQ 6.0.25 AXIP speaks **AXIP-with-CRC** (trailing CRC-16/X.25, low byte
  first) and ignores FCS-less datagrams — hence `include_fcs: true` in the
  firmware's AxudpConfig. (The "FCS-less is the LinBPQ-accepted default" note
  in core::axudp predates this wire check.)
- `QUALITY=192` on the port and `BROADCAST NODES` in the BPQAXIP CONFIG are
  both required before LinBPQ sends NODES broadcasts to B-flagged maps.
- Full session interop: SABM/UA, I-frames + RR both ways, DISC/UA — LinBPQ's
  `C 1 M0LTE-1` lands at the pico-node console prompt.
