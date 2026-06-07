/* pico-node OTA bootloader — RP2040 flash layout (2 MB Pico W).
 *
 * This is the A/B partition map shared by the bootloader and the application.
 * The application's memory.x MUST place its FLASH region at ACTIVE and define
 * the same __bootloader_* symbols (it omits the ACTIVE region itself — its FLASH
 * *is* ACTIVE — and never reserves a .boot2 of its own: the bootloader provides
 * BOOT2 for the whole image).
 *
 * Layout (docs/OTA.md):
 *   BOOT2            0x10000000  256 B    second-stage bootloader (XIP setup)
 *   FLASH (bootldr)  0x10000100  ~32 KB   this bootloader binary
 *   BOOTLOADER_STATE 0x10008000  4 KB     embassy-boot swap/trial state + magic
 *   ACTIVE           0x10009000  896 KB   the running application
 *   DFU              0x100E9000  900 KB   staged update (= ACTIVE + 1 scratch page)
 *   (free gap)       0x101CA000  200 KB
 *   config + netrom  0x101FC000  16 KB    persisted node config + routing table
 *                                         (src/config_store.rs + netrom_store.rs;
 *                                          untouched by OTA — absolute offsets in
 *                                          the full-chip Flash driver)
 */

MEMORY
{
  BOOT2            : ORIGIN = 0x10000000, LENGTH = 0x100
  FLASH            : ORIGIN = 0x10000100, LENGTH = 32K - 0x100
  BOOTLOADER_STATE : ORIGIN = 0x10008000, LENGTH = 4K
  ACTIVE           : ORIGIN = 0x10009000, LENGTH = 896K
  DFU              : ORIGIN = 0x100E9000, LENGTH = 900K

  RAM              : ORIGIN = 0x20000000, LENGTH = 264K
}

__bootloader_state_start = ORIGIN(BOOTLOADER_STATE) - ORIGIN(BOOT2);
__bootloader_state_end   = ORIGIN(BOOTLOADER_STATE) + LENGTH(BOOTLOADER_STATE) - ORIGIN(BOOT2);

__bootloader_active_start = ORIGIN(ACTIVE) - ORIGIN(BOOT2);
__bootloader_active_end   = ORIGIN(ACTIVE) + LENGTH(ACTIVE) - ORIGIN(BOOT2);

__bootloader_dfu_start = ORIGIN(DFU) - ORIGIN(BOOT2);
__bootloader_dfu_end   = ORIGIN(DFU) + LENGTH(DFU) - ORIGIN(BOOT2);
