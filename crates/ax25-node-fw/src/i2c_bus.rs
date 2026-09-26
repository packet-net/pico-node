//! The I2C0 bus task (GP4/GP5): the one owner of the bus, serving the OLED
//! ([`crate::oled`]) and the INA226 power monitor ([`crate::power`]), both
//! optional. One task, so the two devices share the bus without a lock held
//! across tasks (a critical-section lock would stop interrupts, and so the
//! TNC's serial receive, for a whole display refresh).

use core::cell::RefCell;

use embassy_net::Stack;
use embassy_rp::i2c::{Config as I2cConfig, I2c};
use embassy_rp::peripherals::{I2C0, PIN_4, PIN_5};
use embassy_rp::Peri;
use embassy_sync::blocking_mutex::Mutex;
use embassy_time::{Duration, Ticker};

use crate::power::{Bus, Monitor};

embassy_rp::bind_interrupts!(struct Irqs {
    I2C0_IRQ => embassy_rp::i2c::InterruptHandler<I2C0>;
});

#[embassy_executor::task]
#[allow(clippy::too_many_arguments)]
pub async fn task(
    i2c0: Peri<'static, I2C0>,
    sda: Peri<'static, PIN_4>,
    scl: Peri<'static, PIN_5>,
    stack: Stack<'static>,
    hostname: &'static str,
    ap_ssid: &'static str,
    mut power: Monitor,
) {
    let bus: Bus = Mutex::new(RefCell::new(I2c::new_async(
        i2c0,
        scl,
        sda,
        Irqs,
        I2cConfig::default(),
    )));
    // DISABLE_OLED build env skips the panel: a diagnostic escape hatch for
    // boards where the (blocking-I2C) panel path misbehaves and starves the
    // executor. The power monitor still runs.
    let mut display = if option_env!("DISABLE_OLED").is_none() {
        crate::oled::init(&bus)
    } else {
        None
    };
    let mut ticker = Ticker::every(Duration::from_secs(3));
    loop {
        if let Some(d) = display.as_mut() {
            crate::oled::draw(d, stack, hostname, ap_ssid);
        }
        power.poll(&bus).await;
        ticker.next().await;
    }
}
