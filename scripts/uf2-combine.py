#!/usr/bin/env python3
# Combine several UF2 files into ONE valid UF2 (docs/OTA.md).
#
# Raw `cat` of UF2 files is NOT valid: each input keeps its own blockNo/numBlocks,
# so the RP2040 BOOTSEL bootrom counts blocks against the first segment's
# numBlocks and reboots early — flashing only that segment (e.g. just the
# bootloader → erased app → no boot). This concatenates the blocks and then
# RENUMBERS them: blockNo = 0..N-1, numBlocks = N for every block. Block order
# doesn't matter (each block self-describes its target address); only the
# numbering must be globally consistent.
#
# Usage: uf2-combine.py <out.uf2> <in1.uf2> <in2.uf2> ...
import struct, sys

out = sys.argv[1]
blocks = bytearray()
for f in sys.argv[2:]:
    blocks += open(f, "rb").read()

assert len(blocks) % 512 == 0, "not a whole number of 512-byte UF2 blocks"
n = len(blocks) // 512
for i in range(n):
    b = i * 512
    # UF2 header: offset 20 = blockNo, 24 = numBlocks (little-endian u32).
    struct.pack_into("<I", blocks, b + 20, i)
    struct.pack_into("<I", blocks, b + 24, n)

open(out, "wb").write(blocks)
print(f"combined {len(sys.argv)-2} UF2s -> {out} ({n} blocks, renumbered 0..{n-1})")
