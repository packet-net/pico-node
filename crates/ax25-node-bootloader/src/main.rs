//! pico-node OTA bootloader (RP2040 / embassy-boot-rp).
//!
//! Runs first on every boot. It inspects the embassy-boot state partition and,
//! if the application staged + marked an update (SWAP_MAGIC), swaps DFU↔ACTIVE
//! before chaining the active image; if a previous trial never confirmed itself
//! (`mark_booted`), it reverts. Then it jumps to ACTIVE. See docs/OTA.md.
//!
//! Flash sharing: a single blocking `Flash` over the whole 2 MB chip, lent to
//! all three partitions (active/dfu/state) via a `Mutex<RefCell<…>>` — the
//! offsets come from `memory.x` (`__bootloader_*` symbols).
//!
//! No watchdog: a hung trial image still self-heals — the next reset (manual,
//! power-cycle, or BOOTSEL) finds the unconfirmed SWAP and reverts. We omit the
//! WatchdogFlash auto-reset on purpose: it would force the app to feed a ≤8 s
//! watchdog through its (slow) WiFi-join boot and large OTA erases.

#![no_std]
#![no_main]

use core::cell::RefCell;

use cortex_m_rt::{entry, exception};
#[cfg(feature = "defmt")]
use defmt_rtt as _;
use embassy_boot_rp::{BootLoader, BootLoaderConfig};
use embassy_rp::flash::{Blocking, Flash};
use embassy_sync::blocking_mutex::Mutex;

const FLASH_SIZE: usize = 2 * 1024 * 1024;

#[entry]
fn main() -> ! {
    let p = embassy_rp::init(Default::default());

    // A single blocking flash over the whole chip, shared by all partitions.
    let flash = Flash::<_, Blocking, FLASH_SIZE>::new_blocking(p.FLASH);
    let flash = Mutex::new(RefCell::new(flash));

    let config = BootLoaderConfig::from_linkerfile_blocking(&flash, &flash, &flash);
    let active_offset = config.active.offset();
    let bl: BootLoader = BootLoader::prepare(config);

    #[cfg(feature = "defmt")]
    defmt::info!("bootloader: state={}, chaining ACTIVE @ +0x{:x}", bl.state, active_offset);

    // SAFETY: sets VTOR + stack pointer and jumps to the active partition's
    // reset vector. `active_offset` is the ACTIVE region from the linker script.
    unsafe { bl.load(embassy_rp::flash::FLASH_BASE as u32 + active_offset) }
}

// A hard fault this early (e.g. touching flash before XIP is fully up) is best
// turned into a clean reset rather than a lockup.
#[unsafe(no_mangle)]
#[cfg_attr(target_os = "none", unsafe(link_section = ".HardFault.user"))]
unsafe extern "C" fn HardFault() {
    cortex_m::peripheral::SCB::sys_reset();
}

#[exception]
unsafe fn DefaultHandler(_: i16) -> ! {
    const SCB_ICSR: *const u32 = 0xE000_ED04 as *const u32;
    let irqn = unsafe { core::ptr::read_volatile(SCB_ICSR) } as u8 as i16 - 16;
    panic!("DefaultHandler #{:?}", irqn);
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    cortex_m::asm::udf();
}
