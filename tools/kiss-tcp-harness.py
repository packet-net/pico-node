#!/usr/bin/env python3
"""Minimal KISS-over-TCP host harness for HW-BRINGUP.md §4 Gate 5.

Listens for a KISS-TCP client (the Pico firmware), de-frames KISS (FEND/FESC
escaping), prints a decoded summary of each Data frame's AX.25 body, and
replies once per connection with a KISS-framed UI frame addressed to the
sender's callsign — proving the round trip.

Zero dependencies. Usage:
    python3 tools/kiss-tcp-harness.py [--port 8001] [--reply-text TEXT]
"""

import argparse
import socket
import sys
import time

FEND, FESC, TFEND, TFESC = 0xC0, 0xDB, 0xDC, 0xDD


def kiss_escape(payload: bytes) -> bytes:
    out = bytearray()
    for b in payload:
        if b == FEND:
            out += bytes((FESC, TFEND))
        elif b == FESC:
            out += bytes((FESC, TFESC))
        else:
            out.append(b)
    return bytes(out)


def kiss_frame(port: int, command: int, payload: bytes) -> bytes:
    return bytes((FEND, ((port & 0x0F) << 4) | (command & 0x0F))) + \
        kiss_escape(payload) + bytes((FEND,))


class KissDeframer:
    def __init__(self) -> None:
        self.buf = bytearray()
        self.in_frame = False
        self.escaped = False

    def push(self, data: bytes):
        """Yield (port, command, payload) per completed frame."""
        for b in data:
            if b == FEND:
                if self.in_frame and self.buf:
                    cmd_byte = self.buf[0]
                    yield (cmd_byte >> 4) & 0x0F, cmd_byte & 0x0F, bytes(self.buf[1:])
                self.buf.clear()
                self.in_frame = True
                self.escaped = False
            elif self.in_frame:
                if self.escaped:
                    self.buf.append(FEND if b == TFEND else FESC if b == TFESC else b)
                    self.escaped = False
                elif b == FESC:
                    self.escaped = True
                else:
                    self.buf.append(b)


def decode_addr(b: bytes) -> tuple[str, bool]:
    call = "".join(chr(o >> 1) for o in b[:6]).rstrip()
    ssid = (b[6] >> 1) & 0x0F
    return (f"{call}-{ssid}" if ssid else call, bool(b[6] & 0x01))


def encode_addr(callsign: str, *, crh: bool, last: bool) -> bytes:
    base, _, ssid = callsign.partition("-")
    out = bytearray((ord(c) << 1) for c in f"{base:<6}"[:6])
    oct7 = ((int(ssid or 0) & 0x0F) << 1) | 0x60 | (0x80 if crh else 0) | (1 if last else 0)
    out.append(oct7)
    return bytes(out)


def summarize_ax25(payload: bytes) -> str:
    if len(payload) < 16:
        return f"(short, {len(payload)}B): {payload.hex()}"
    dest, _ = decode_addr(payload[0:7])
    src, last = decode_addr(payload[7:14])
    i = 14
    while not last and len(payload) >= i + 7:
        _, last = decode_addr(payload[i : i + 7])
        i += 7
    ctl = payload[i] if i < len(payload) else None
    info = payload[i + 2 :] if ctl is not None and (ctl & 0xEF) == 0x03 else b""
    kind = "UI" if ctl is not None and (ctl & 0xEF) == 0x03 else f"ctl={ctl:#04x}"
    return f"{src} > {dest} {kind} info={info.decode('ascii', 'replace')!r}"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8001)
    ap.add_argument("--reply-text", default="hello pico, from the KISS-TCP harness")
    args = ap.parse_args()

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", args.port))
    srv.listen(1)
    print(f"kiss-tcp-harness: listening on tcp/{args.port}", flush=True)

    while True:
        conn, peer = srv.accept()
        print(f"connection from {peer[0]}:{peer[1]}", flush=True)
        deframer, replied = KissDeframer(), False
        try:
            while True:
                data = conn.recv(4096)
                if not data:
                    break
                for port, cmd, payload in deframer.push(data):
                    ts = time.strftime("%H:%M:%S")
                    if cmd == 0:
                        print(f"{ts} KISS port={port} DATA {len(payload)}B: "
                              f"{summarize_ax25(payload)}", flush=True)
                        if not replied:
                            replied = True
                            src, _ = decode_addr(payload[7:14])
                            reply_ax25 = (
                                encode_addr(src, crh=True, last=False)
                                + encode_addr("HARNES-2", crh=False, last=True)
                                + b"\x03\xf0" + args.reply_text.encode()
                            )
                            conn.sendall(kiss_frame(0, 0, reply_ax25))
                            print(f"{ts} -> replied to {src} (KISS-framed UI)",
                                  flush=True)
                    else:
                        print(f"{ts} KISS port={port} cmd={cmd:#x} "
                              f"{len(payload)}B", flush=True)
        finally:
            conn.close()
            print("connection closed", flush=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
