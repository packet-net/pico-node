/* RP2040 memory layout for the pico-node application — the OTA/A-B variant.
 *
 * Since OTA (docs/OTA.md) the application is ALWAYS chained by the bootloader
 * (crates/ax25-node-bootloader): it is NOT independently bootable. Its FLASH
 * region IS the ACTIVE partition (0x10009000); the bootloader owns BOOT2 at the
 * flash base and jumps here after any pending A/B swap. The combined
 * bootloader+app UF2 is the first-flash artifact; thereafter updates are staged
 * into DFU over the air.
 *
 * No `.boot2` SECTIONS block here: embassy-rp still emits a `.boot2` static, but
 * for a chained app it is an unused orphan (the bootloader's boot2 at the flash
 * base is the one the BootROM runs). We deliberately do NOT place it at
 * ORIGIN(BOOT2) — that belongs to the bootloader — and the linker keeps the
 * vector table at ACTIVE origin where the bootloader's `load()` points VTOR.
 *
 * The persisted config + NET/ROM routing stores live at absolute offsets in the
 * top 16 KiB (src/config_store.rs, src/netrom_store.rs); the Flash driver
 * addresses the whole 2 MB chip, so those are reachable regardless of this
 * (ACTIVE-bounded) FLASH region — they sit in the gap above DFU, outside it.
 *
 * Layout (must match crates/ax25-node-bootloader/memory.x):
 *   BOOT2            0x10000000  256 B    (bootloader-provided; not placed here)
 *   bootloader       0x10000100  ~32 KB
 *   BOOTLOADER_STATE 0x10008000  4 KB
 *   ACTIVE (=FLASH)  0x10009000  896 KB   <- this application
 *   DFU              0x100E9000  900 KB   <- staged OTA image
 *   config + netrom  0x101FC000  16 KB
 */

MEMORY
{
    BOOT2            : ORIGIN = 0x10000000, LENGTH = 0x100
    BOOTLOADER_STATE : ORIGIN = 0x10008000, LENGTH = 4K
    FLASH            : ORIGIN = 0x10009000, LENGTH = 896K
    DFU              : ORIGIN = 0x100E9000, LENGTH = 900K

    RAM              : ORIGIN = 0x20000000, LENGTH = 264K
}

/* embassy-boot partition symbols (offsets from flash base), consumed by
   FirmwareUpdaterConfig::from_linkerfile_blocking in src/ota.rs. The app needs
   only DFU + STATE; ACTIVE is the bootloader's concern. */
__bootloader_state_start = ORIGIN(BOOTLOADER_STATE) - ORIGIN(BOOT2);
__bootloader_state_end   = ORIGIN(BOOTLOADER_STATE) + LENGTH(BOOTLOADER_STATE) - ORIGIN(BOOT2);

__bootloader_dfu_start = ORIGIN(DFU) - ORIGIN(BOOT2);
__bootloader_dfu_end   = ORIGIN(DFU) + LENGTH(DFU) - ORIGIN(BOOT2);
