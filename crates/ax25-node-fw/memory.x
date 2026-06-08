/* RP2040 memory layout for the pico-node application — the OTA/A-B variant.
 *
 * Since OTA (docs/OTA.md) the application is ALWAYS chained by the bootloader
 * (crates/ax25-node-bootloader): it is NOT independently bootable. Its FLASH
 * region IS the ACTIVE partition (0x10007000); the bootloader owns BOOT2 at the
 * flash base and jumps here after any pending A/B swap.
 *
 * No `.boot2` SECTIONS block here: embassy-rp still emits a `.boot2` static, but
 * for a chained app it is an unused orphan (the bootloader's boot2 at the flash
 * base is the one the BootROM runs). The linker keeps the vector table at ACTIVE
 * origin where the bootloader's `load()` points VTOR.
 *
 * The cyw43 firmware/CLM/NVRAM blobs are NOT linked into this image: they live
 * in the BLOBS region (flashed once with the combined image) and src/net.rs
 * reads them at the fixed XIP address __blobs_start. The persisted config +
 * routing stores (top 16 KiB) and the APPDATA reserve are likewise at absolute
 * offsets in the full-chip Flash driver — the linker FLASH region (ACTIVE) stays
 * clear of all of them.
 *
 * BLOB-DE-DUPED layout — must match crates/ax25-node-bootloader/memory.x exactly
 * (scripts/check-layout.sh enforces it in CI; the layout is FROZEN):
 *   BOOT2 + bootloader 0x10000000  24 KB    (bootloader-provided; not placed here)
 *   BOOTLOADER_STATE   0x10006000  4 KB
 *   ACTIVE (=FLASH)    0x10007000  512 KB    <- this application (max image size)
 *   DFU                0x10087000  516 KB    <- staged OTA image
 *   BLOBS              0x10108000  256 KB    <- cyw43 firmware (flash-once)
 *   APPDATA            0x10148000  720 KB    <- reserved store-and-forward data
 *   config + netrom    0x101FC000  16 KB
 */

MEMORY
{
    BOOT2            : ORIGIN = 0x10000000, LENGTH = 0x100
    BOOTLOADER_STATE : ORIGIN = 0x10006000, LENGTH = 4K
    FLASH            : ORIGIN = 0x10007000, LENGTH = 512K
    DFU              : ORIGIN = 0x10087000, LENGTH = 516K
    BLOBS            : ORIGIN = 0x10108000, LENGTH = 256K
    APPDATA          : ORIGIN = 0x10148000, LENGTH = 720K

    RAM              : ORIGIN = 0x20000000, LENGTH = 264K
}

/* embassy-boot partition symbols (offsets from flash base), consumed by
   FirmwareUpdaterConfig::from_linkerfile_blocking in src/ota.rs. */
__bootloader_state_start = ORIGIN(BOOTLOADER_STATE) - ORIGIN(BOOT2);
__bootloader_state_end   = ORIGIN(BOOTLOADER_STATE) + LENGTH(BOOTLOADER_STATE) - ORIGIN(BOOT2);

__bootloader_dfu_start = ORIGIN(DFU) - ORIGIN(BOOT2);
__bootloader_dfu_end   = ORIGIN(DFU) + LENGTH(DFU) - ORIGIN(BOOT2);

/* Absolute XIP address + length of the BLOBS region — src/net.rs reads the
   cyw43 firmware/CLM/NVRAM from here (they are flashed once, not in the image). */
__blobs_start = ORIGIN(BLOBS);
__blobs_len   = LENGTH(BLOBS);

/* Reserved store-and-forward data region (absolute flash offsets), for a future
   src/data_store.rs. Bulk data → SD card; this is for indices/spools/SD-less. */
__appdata_start = ORIGIN(APPDATA) - ORIGIN(BOOT2);
__appdata_end   = ORIGIN(APPDATA) + LENGTH(APPDATA) - ORIGIN(BOOT2);
