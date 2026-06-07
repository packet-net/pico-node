# CYW43439 firmware blobs (vendored)

WiFi firmware, CLM, and NVRAM blobs for the Pico W's CYW43439, loaded by
`src/net.rs` via `cyw43::aligned_bytes!`.

**Provenance:** copied verbatim from the embassy repo at tag `cyw43-v0.7.0`
(`embassy-rs/embassy/cyw43-firmware/`), which in turn obtains them from
<https://github.com/georgerobotics/cyw43-driver/tree/main/firmware>.

**Licence:** Infineon [Permissive Binary License 1.0](./LICENSE-permissive-binary-license-1.0.txt)
(redistribution in binary form permitted; the licence file must travel with the
blobs — hence this directory). Licence-check per `docs/HW-BRINGUP.md` §5 done
2026-06-07: embassy redistributes these blobs under the same licence-alongside
arrangement, and condition (1) (reproduce the notice with the distribution) is
satisfied by the LICENSE file here.

| file | sha256 |
|---|---|
| `43439A0.bin` | `5555e0261da2610a500d68c18d895cace0152bbefbf76f4aa683ebce77e3d7eb` |
| `43439A0_clm.bin` | `e712b3d218e8b1e2747b092e03b8b0afcb8c8c8e355d2a4a0d47b493800f3f89` |
| `nvram_rp2040.bin` | `4904bdbb0c937bd0ac2eb2a1d62f2da4dd90e32082384e02874e8d671b0f330d` |

When bumping the `cyw43` crate, re-sync these from the matching embassy tag
(the blob set is version-coupled: cyw43 0.7 added the separate NVRAM blob).
