//! OLED status display — the SSD1306 on the NinoBLE Rev5 board (I2C0 GP4/GP5,
//! address 0x3C). docs/HARDWARE-NINOBLE.md. Optional: the panel is user-
//! installed, so the task initialises it and, if nothing ACKs at 0x3C, logs and
//! exits cleanly — the bare-Pico bench build boots unaffected.
//!
//! Shows a compact node status page, refreshed every few seconds: callsign +
//! mode (STA/AP), the IP or AP gateway, and the NET/ROM neighbour + route
//! counts. The render path is `embedded-graphics` text over the `ssd1306` crate
//! (the proven driver — its init sequence matches the NinoBLE firmware's known-
//! good one for this exact panel, so first-light risk is low).
//!
//! Verified-on-arrival: the NinoBLE board + panel are not on the current rig, so
//! this is build- and type-checked but not yet lit on hardware.

use core::fmt::Write as _;

use embassy_rp::i2c::{Config as I2cConfig, I2c};
use embassy_rp::peripherals::{I2C0, PIN_4, PIN_5};
use embassy_rp::Peri;
use embassy_time::{Duration, Ticker};

use embedded_graphics::mono_font::ascii::FONT_6X10;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::text::{Baseline, Text};

use ssd1306::mode::DisplayConfig;
use ssd1306::prelude::*;
use ssd1306::{I2CDisplayInterface, Ssd1306};

use embassy_net::Stack;

/// A snapshot of node state for the display, refreshed by the owner. Cheap
/// `Copy` so the OLED task can read it without locking the live structures.
#[derive(Clone, Copy, Default)]
pub struct Status {
    /// Callsign text (e.g. `"M9YYY-9"`), null-padded.
    pub callsign: [u8; 12],
    /// `true` = STA mode, `false` = AP mode.
    pub sta: bool,
    /// NET/ROM neighbours / destinations known.
    pub neighbours: u16,
    pub destinations: u16,
}

impl Status {
    fn callsign_str(&self) -> &str {
        let n = self.callsign.iter().position(|&b| b == 0).unwrap_or(12);
        core::str::from_utf8(&self.callsign[..n]).unwrap_or("?")
    }
}

/// Shared status the OLED renders. The owner (`main`/transports) updates it; the
/// OLED task reads it. `embassy_sync` blocking mutex — updates are tiny.
pub static STATUS: embassy_sync::blocking_mutex::Mutex<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    core::cell::RefCell<Status>,
> = embassy_sync::blocking_mutex::Mutex::new(core::cell::RefCell::new(Status {
    callsign: [0; 12],
    sta: true,
    neighbours: 0,
    destinations: 0,
}));

/// Update the shared status (call when identity/mode/route counts change).
pub fn set(status: Status) {
    STATUS.lock(|c| *c.borrow_mut() = status);
}

/// Update just the NET/ROM counts (the transports call this as the table
/// changes, without disturbing the identity/mode set once at boot).
pub fn set_counts(neighbours: u16, destinations: u16) {
    STATUS.lock(|c| {
        let mut s = c.borrow_mut();
        s.neighbours = neighbours;
        s.destinations = destinations;
    });
}

embassy_rp::bind_interrupts!(struct Irqs {
    I2C0_IRQ => embassy_rp::i2c::InterruptHandler<I2C0>;
});

#[embassy_executor::task]
pub async fn task(
    i2c0: Peri<'static, I2C0>,
    sda: Peri<'static, PIN_4>,
    scl: Peri<'static, PIN_5>,
    stack: Stack<'static>,
) {
    let i2c = I2c::new_async(i2c0, scl, sda, Irqs, I2cConfig::default());
    let interface = I2CDisplayInterface::new(i2c);
    // NinoBLE Rev5 panels are user-installed and vary (128x32 or 128x64); the
    // common one is the 0.91" 128x32, mounted such that it reads upside-down to
    // Rotate0. (TODO: make size+rotation a config option for 128x64 boards.)
    let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate180)
        .into_buffered_graphics_mode();

    if display.init().is_err() {
        defmt::info!("oled: no SSD1306 at 0x3C — display disabled (optional)");
        return;
    }
    defmt::info!("oled: SSD1306 up (I2C0 GP4/GP5)");

    let text = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
    let mut ticker = Ticker::every(Duration::from_secs(3));
    loop {
        let status = STATUS.lock(|c| *c.borrow());

        // Compose the page off the live structures + the net stack's current IP.
        let mut l1 = heapless::String::<24>::new();
        let _ = write!(l1, "{} pico-node", status.callsign_str());
        let mut l2 = heapless::String::<24>::new();
        match (status.sta, stack.config_v4()) {
            (true, Some(v4)) => {
                let a = v4.address.address().octets();
                let _ = write!(l2, "STA {}.{}.{}.{}", a[0], a[1], a[2], a[3]);
            }
            (true, None) => {
                let _ = write!(l2, "STA (no lease)");
            }
            (false, _) => {
                let _ = write!(l2, "AP 192.168.4.1");
            }
        }
        let mut l3 = heapless::String::<24>::new();
        let _ = write!(
            l3,
            "NET/ROM n{} d{}",
            status.neighbours, status.destinations
        );

        display.clear(BinaryColor::Off).ok();
        // Top-left aligned: three 10px lines filling the 128x32 panel (tops at
        // y=0/11/22, Baseline::Top so the first line sits flush against the top).
        for (i, line) in [l1.as_str(), l2.as_str(), l3.as_str()].iter().enumerate() {
            let y = i as i32 * 11;
            let _ = Text::with_baseline(line, Point::new(0, y), text, Baseline::Top)
                .draw(&mut display);
        }
        let _ = display.flush();

        ticker.next().await;
    }
}
