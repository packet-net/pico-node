/* pico-node OTA bootloader — RP2040 flash layout (2 MB Pico W).
 *
 * This is the A/B partition map shared by the bootloader and the application.
 * The application's memory.x MUST place its FLASH region at ACTIVE and define
 * the same __bootloader_* symbols. scripts/check-layout.sh (run in CI) asserts
 * the two files agree — the layout is FROZEN (changing it needs a coordinated
 * bootloader reflash of every node, and it breaks binary-delta OTA).
 *
 * The big cyw43 WiFi firmware (~226 KB) is NOT in the application image: it
 * lives ONCE in the dedicated BLOBS region, flashed with the combined image and
 * read by the app at a fixed XIP address (src/net.rs). Keeping it out of A/B
 * halves its flash cost (it would otherwise sit in BOTH ACTIVE and DFU) and
 * shrinks every OTA image. The blob is rarely updated (the upstream firmware
 * changed ~twice in 3 years and we pin it), so a blob refresh is a deliberate
 * BOOTSEL event, not a routine OTA. APPDATA is reserved for store-and-forward
 * app data (mail/chat spools + indices on SD-less nodes; bulk goes on the SD
 * card). The bootloader touches neither BLOBS nor APPDATA — they're declared
 * here only to document + freeze the map.
 *
 *   BOOT2 + bootloader 0x10000000  24 KB    BOOT2 (256 B) + this bootloader (~9.5 KB)
 *   BOOTLOADER_STATE   0x10006000  4 KB     embassy-boot swap/trial state + magic
 *   ACTIVE             0x10007000  512 KB   the running application (max image size)
 *   DFU                0x10087000  516 KB   staged update (= ACTIVE + 1 scratch page)
 *   BLOBS              0x10108000  256 KB   cyw43 firmware/CLM/NVRAM (flash-once)
 *   APPDATA            0x10148000  720 KB   reserved: store-and-forward app data
 *   config + netrom    0x101FC000  16 KB    persisted node config + routing table.
 *                                           No gaps: every byte is allocated.
 */

MEMORY
{
  BOOT2            : ORIGIN = 0x10000000, LENGTH = 0x100
  FLASH            : ORIGIN = 0x10000100, LENGTH = 24K - 0x100
  BOOTLOADER_STATE : ORIGIN = 0x10006000, LENGTH = 4K
  ACTIVE           : ORIGIN = 0x10007000, LENGTH = 512K
  DFU              : ORIGIN = 0x10087000, LENGTH = 516K
  BLOBS            : ORIGIN = 0x10108000, LENGTH = 256K
  APPDATA          : ORIGIN = 0x10148000, LENGTH = 720K

  RAM              : ORIGIN = 0x20000000, LENGTH = 264K
}

__bootloader_state_start = ORIGIN(BOOTLOADER_STATE) - ORIGIN(BOOT2);
__bootloader_state_end   = ORIGIN(BOOTLOADER_STATE) + LENGTH(BOOTLOADER_STATE) - ORIGIN(BOOT2);

__bootloader_active_start = ORIGIN(ACTIVE) - ORIGIN(BOOT2);
__bootloader_active_end   = ORIGIN(ACTIVE) + LENGTH(ACTIVE) - ORIGIN(BOOT2);

__bootloader_dfu_start = ORIGIN(DFU) - ORIGIN(BOOT2);
__bootloader_dfu_end   = ORIGIN(DFU) + LENGTH(DFU) - ORIGIN(BOOT2);
