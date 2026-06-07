/* RP2040 memory layout for cortex-m-rt + embassy-rp.
 *
 * The RP2040 boots from a 256-byte second-stage bootloader (BOOT2) at the start
 * of XIP flash, which configures the QSPI flash for execute-in-place; the rest of
 * flash holds the program, and SRAM is 264 KB. This is the canonical layout from
 * the rp2040-project-template / embassy examples. build.rs copies this file to the
 * linker search path. (Not used until the firmware is built — see docs/PLAN.md.)
 *
 * The Pico W's WiFi firmware + CLM blobs are large (~230 KB) and are linked as
 * byte arrays into the program FLASH region; 2 MB is ample.
 */

MEMORY {
    /* BOOT2: the 256-byte second-stage bootloader at the base of XIP flash. */
    BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100

    /* FLASH: the program, immediately after BOOT2. 2 MB on the stock Pico W,
       minus the 256 bytes of BOOT2 and minus the top 16 KiB reserved for persisted state: the node
       config (two 4 KiB sectors, src/config_store.rs) and the NET/ROM
       routing table (two more, src/netrom_store.rs) + docs/PROVISIONING.md; keeping them out of the
       linker's FLASH region guarantees the program can never overlap them). */
    FLASH : ORIGIN = 0x10000100, LENGTH = 2048K - 0x100 - 16K

    /* RAM: 264 KB of SRAM (the RP2040's 6 banks, contiguous for the linker). */
    RAM   : ORIGIN = 0x20000000, LENGTH = 264K
}

/* cortex-m-rt places the vector table at the start of FLASH; embassy-rp + the
   rp2040 boot2 crate populate BOOT2. The `_stack_start` default (top of RAM) is
   relocated by flip-link at link time. */
SECTIONS {
    /* The second-stage bootloader, CRC-checked by the BootROM. The boot2 helper
       (pulled in by embassy-rp / a boot2 crate) provides `__boot2_*` symbols. */
    .boot2 ORIGIN(BOOT2) :
    {
        KEEP(*(.boot2));
    } > BOOT2
} INSERT BEFORE .text;
