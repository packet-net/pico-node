#!/usr/bin/env python3
"""bin -> UF2 (RP2040): the picotool-free converter for package-ota.sh.

Writes the UF2 the RP2040 BOOTSEL bootrom consumes, matching the shape of
`picotool uf2 convert <in> -t bin -o <addr> <out>`: 512-byte blocks, 256-byte
payloads, the familyID-present flag, RP2040 family 0xe48bff56, one contiguous
run from the given base address. A short final chunk is padded with 0x00,
matching picotool, so the two converters produce byte-identical files (verified
field-for-field against the picotool-built, hardware-proven v0.4.2 blobs UF2).

Usage: bin2uf2.py <in.bin> <base_addr_hex> <out.uf2>
"""

import struct
import sys

UF2_MAGIC0 = 0x0A324655  # "UF2\n"
UF2_MAGIC1 = 0x9E5D5157
UF2_MAGIC_END = 0x0AB16F30
FLAG_FAMILY_ID_PRESENT = 0x00002000
RP2040_FAMILY = 0xE48BFF56
CHUNK = 256
BLOCK = 512


def main() -> None:
    if len(sys.argv) != 4:
        sys.exit("usage: bin2uf2.py <in.bin> <base_addr_hex> <out.uf2>")
    data = open(sys.argv[1], "rb").read()
    base = int(sys.argv[2], 16)
    if not data:
        sys.exit("bin2uf2: input is empty")
    if base % CHUNK:
        sys.exit(f"bin2uf2: base address {base:#x} is not {CHUNK}-byte aligned")

    chunks = [data[i : i + CHUNK] for i in range(0, len(data), CHUNK)]
    total = len(chunks)
    with open(sys.argv[3], "wb") as out:
        for seq, chunk in enumerate(chunks):
            payload = chunk.ljust(CHUNK, b"\x00")
            block = struct.pack(
                "<IIIIIIII",
                UF2_MAGIC0,
                UF2_MAGIC1,
                FLAG_FAMILY_ID_PRESENT,
                base + seq * CHUNK,
                CHUNK,
                seq,
                total,
                RP2040_FAMILY,
            )
            block += payload
            block += b"\x00" * (BLOCK - 4 - len(block))
            block += struct.pack("<I", UF2_MAGIC_END)
            assert len(block) == BLOCK
            out.write(block)
    print(f"  {sys.argv[3]}: {total} blocks, base {sys.argv[2]}, {len(data)} bytes")


if __name__ == "__main__":
    main()
