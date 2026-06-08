#!/usr/bin/env python3
# Build the cyw43 BLOBS image (docs/OTA.md): a small manifest + the firmware,
# CLM and NVRAM blobs, laid out for the app to read at the fixed BLOBS XIP
# address (src/net.rs). Flashed once with the combined image; never travels over
# OTA. Keeping the ~226 KB firmware here (not in the app) keeps it out of BOTH
# A/B partitions.
#
# Manifest (little-endian, at BLOBS base):
#   0  magic "PBLB"        (4)
#   4  version u8 = 1
#   5  count   u8 = 3
#   6  reserved u16 = 0
#   8  fw_off u32, fw_len u32          (offsets are from BLOBS base)
#  16  clm_off u32, clm_len u32
#  24  nvram_off u32, nvram_len u32
#  -> header is 32 bytes; padded to 256; blobs follow, each 4-byte aligned.
#
# Usage: build-blobs.py <fw.bin> <clm.bin> <nvram.bin> <out.bin> [region_size]
import struct, sys

fw_p, clm_p, nvram_p, out_p = sys.argv[1:5]
REGION = int(sys.argv[5]) if len(sys.argv) > 5 else 256 * 1024
HDR = 256

def rd(p):
    with open(p, "rb") as f:
        return f.read()

fw, clm, nvram = rd(fw_p), rd(clm_p), rd(nvram_p)

def align4(n):
    return (n + 3) & ~3

img = bytearray()
off_fw = HDR
off_clm = align4(off_fw + len(fw))
off_nvram = align4(off_clm + len(clm))
end = off_nvram + len(nvram)
if end > REGION:
    sys.exit(f"BLOBS image {end} bytes exceeds region {REGION}")

hdr = bytearray(HDR)
struct.pack_into("<4sBBH", hdr, 0, b"PBLB", 1, 3, 0)
struct.pack_into("<6I", hdr, 8, off_fw, len(fw), off_clm, len(clm), off_nvram, len(nvram))

img += hdr
img += b"\xff" * (off_fw - len(img));  img += fw
img += b"\xff" * (off_clm - len(img)); img += clm
img += b"\xff" * (off_nvram - len(img)); img += nvram

with open(out_p, "wb") as f:
    f.write(img)
print(f"blobs image: {len(img)} bytes (fw@{off_fw}/{len(fw)} clm@{off_clm}/{len(clm)} "
      f"nvram@{off_nvram}/{len(nvram)}), region {REGION}, {REGION-end} bytes free")
