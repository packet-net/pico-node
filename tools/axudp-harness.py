#!/usr/bin/env python3
"""Minimal AXUDP host harness for HW-BRINGUP.md §4 Gate 3.

Listens for AXUDP datagrams (UDP payload == AX.25 frame body), prints a decoded
summary of each, and replies once per peer with a UI frame addressed to the
sender's callsign — giving the firmware its "a frame sent back is decoded and
logged by the Pico" half of the gate.

Zero dependencies; standalone AX.25 address codec (shift-left-1 per spec).
AXUDP datagrams always carry the trailing CRC-16/X.25 FCS (low byte first) —
appended on send, verified + stripped on receive.

Usage:  python3 tools/axudp-harness.py [--port 10093] [--reply-text TEXT]
"""

import argparse
import socket
import sys
import time


def crc16_x25(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return crc ^ 0xFFFF


def append_fcs(body: bytes) -> bytes:
    fcs = crc16_x25(body)
    return body + bytes((fcs & 0xFF, fcs >> 8))


def strip_fcs(payload: bytes) -> bytes | None:
    """Verify + strip the trailing FCS; None if missing/invalid."""
    if len(payload) < 2:
        return None
    body, fcs = payload[:-2], payload[-2:]
    if crc16_x25(body) != (fcs[0] | (fcs[1] << 8)):
        return None
    return body


def decode_addr(b: bytes) -> tuple[str, bool]:
    """7 wire octets -> (CALL-SSID, extension-bit)."""
    call = "".join(chr(o >> 1) for o in b[:6]).rstrip()
    ssid = (b[6] >> 1) & 0x0F
    ext = bool(b[6] & 0x01)
    return (f"{call}-{ssid}" if ssid else call, ext)


def encode_addr(callsign: str, *, crh: bool, last: bool) -> bytes:
    base, _, ssid = callsign.partition("-")
    out = bytearray((ord(c) << 1) for c in f"{base:<6}"[:6])
    ssid_oct = ((int(ssid or 0) & 0x0F) << 1) | 0x60  # reserved bits set
    if crh:
        ssid_oct |= 0x80
    if last:
        ssid_oct |= 0x01
    out.append(ssid_oct)
    return bytes(out)


def decode_frame(payload: bytes) -> dict | None:
    """Best-effort AX.25 decode: dest, src, control, pid, info."""
    if len(payload) < 16:  # 2 addresses + control + pid minimum
        return None
    dest, ext = decode_addr(payload[0:7])
    if ext:
        return None  # destination can never be the last address
    src, last = decode_addr(payload[7:14])
    i = 14
    digis = []
    while not last:
        if len(payload) < i + 7:
            return None
        digi, last = decode_addr(payload[i : i + 7])
        digis.append(digi)
        i += 7
    if len(payload) < i + 1:
        return None
    control = payload[i]
    i += 1
    pid = None
    if (control & 0xEF) == 0x03 or (control & 0x01) == 0x00:  # UI or I frame
        if len(payload) < i + 1:
            return None
        pid = payload[i]
        i += 1
    return {
        "dest": dest,
        "src": src,
        "digis": digis,
        "control": control,
        "pid": pid,
        "info": payload[i:],
    }


def build_ui(dest: str, src: str, info: bytes) -> bytes:
    return (
        encode_addr(dest, crh=True, last=False)
        + encode_addr(src, crh=False, last=True)
        + b"\x03\xf0"
        + info
    )


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=10093)
    ap.add_argument("--reply-text", default="hello pico, from the host harness")
    ap.add_argument("--reply-every", action="store_true",
                    help="reply to every frame, not once per peer")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", args.port))
    print(f"axudp-harness: listening on udp/{args.port}", flush=True)

    replied: set[tuple] = set()
    while True:
        payload, peer = sock.recvfrom(65535)
        ts = time.strftime("%H:%M:%S")
        body = strip_fcs(payload)
        if body is None:
            print(f"{ts} {peer[0]}:{peer[1]} {len(payload)}B REJECTED (bad/missing "
                  f"FCS): {payload.hex()}", flush=True)
            continue
        f = decode_frame(body)
        if f is None:
            print(f"{ts} {peer[0]}:{peer[1]} {len(body)}B (not AX.25): "
                  f"{body.hex()}", flush=True)
            continue
        kind = "UI" if (f["control"] & 0xEF) == 0x03 else f"ctl=0x{f['control']:02x}"
        info = f["info"].decode("ascii", "replace")
        print(f"{ts} {peer[0]}:{peer[1]} {f['src']} > {f['dest']} {kind} "
              f"pid={f['pid']:#04x} info={info!r}", flush=True)

        if args.reply_every or peer not in replied:
            replied.add(peer)
            reply = append_fcs(build_ui(dest=f["src"], src="HARNES-1",
                                        info=args.reply_text.encode()))
            sock.sendto(reply, peer)
            print(f"{ts} -> replied to {f['src']} at {peer[0]}:{peer[1]} "
                  f"({len(reply)}B UI)", flush=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
