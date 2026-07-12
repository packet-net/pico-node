//! NinoTNC-specific KISS extensions.
//!
//! The Rust port of `Packet.Kiss.NinoTnc` (the NinoTNC-flavoured behaviour layered
//! over standard KISS): the mode catalog, the SETHW mode byte (with the `+16`
//! non-persist offset), the two TX-Test frame shapes, and the NinoTNC-aware
//! classifier. All `no_std`-clean and host-testable; the byte source (the UART) is
//! provided by the firmware transport ([`crate::kiss::serial`]).
//!
//! ## Layout (mirrors the C# `Packet.Kiss.NinoTnc` split)
//!
//! - [`catalog`] — [`catalog::NinoTncMode`] + the mode table + the firmware-byte
//!   reverse map (`NinoTncMode` + `NinoTncCatalog`).
//! - [`sethw`] — the SETHW mode byte + KISS frame builders, incl. the `+16`
//!   non-persist offset (`NinoTncSetHardware`).
//! - [`txtest`] — the synthetic host-side TX-Test *diagnostic* parser
//!   (`NinoTncTxTestFrame`).
//! - [`airtest`] — the over-air TX-Test UI-frame recognizer (`NinoTncAirTestFrame`).
//! - [`cqbeep`] — the CQBEEP arming / beep-request *builders* (`NinoTncCqBeep`) — the
//!   outbound counterpart to [`airtest`]'s recognizer.
//! - [`status`] — the periodic numeric diagnostic-register report parser
//!   (`NinoTncStatusFrame`).
//! - [`rssi`] — the GETRSSI RX-audio-level reply parser (`NinoTncRssiReading`).
//! - [`firmware`] — the firmware version + dsPIC chip variant value types.
//! - [`classify`] — the NinoTNC-aware classifier overlay (`NinoTncFrameClassifier`).
//!
//! ## Connectivity (device / baud)
//!
//! The NinoTNC's documented USB-CDC KISS baud is [`DEFAULT_BAUD`] (57 600 8N1), the
//! same value the C# `NinoTncSerialPort.DefaultBaudRate` / `KissSerialModem`
//! defaults to. On the Pico W we do **not** drive the NinoTNC over USB — the RP2040
//! cannot be a USB host and a USB-serial device at once — so we wire the Pico's UART
//! directly to the NinoTNC's UART pins at this baud (see [`crate::kiss::serial`] and
//! the firmware `transports::kiss_serial`).
//!
//! ## Parity scope (divergences from `Packet.Kiss.NinoTnc`, all out of node scope)
//!
//! - **Port discovery** (`NinoTncPortDiscovery`): host-OS serial enumeration
//!   (Linux `/dev/serial/by-id`, the Windows registry, USB VID/PID `04D8:00DD`). The
//!   Pico's UART is a fixed peripheral with no enumeration, so this has no embedded
//!   analogue and is intentionally omitted.
//! - **Firmware OTA** (`Firmware/GitHub*Catalogue`, `*Flasher`): GitHub release
//!   discovery + ICSP flashing — host tooling, no place on the node. Omitted; only
//!   the firmware *version* + *chip variant* value types are ported ([`firmware`]).
//! - **The async modem driver** (`NinoTncSerialPort`: the read pump, the event
//!   handlers): the *protocol* it speaks is ported (KISS codec +
//!   [`crate::kiss::ackmode`] — including the portable ACKMODE TX-completion
//!   correlator [`crate::kiss::ackmode::AckCorrelator`] — plus [`sethw`], [`status`],
//!   [`rssi`], [`cqbeep`], and [`classify`]); only its async I/O glue maps onto the
//!   firmware's embassy UART transport rather than C# `System.IO.Ports` +
//!   `System.Threading.Channels`.

pub mod airtest;
pub mod catalog;
pub mod classify;
pub mod cqbeep;
pub mod firmware;
pub mod rssi;
pub mod sethw;
pub mod status;
pub mod txtest;

pub use airtest::NinoTncAirTestFrame;
pub use catalog::NinoTncMode;
pub use classify::{classify, NinoTncInboundEvent};
pub use firmware::{ChipVariant, FirmwareVersion};
pub use rssi::NinoTncRssiReading;
pub use status::NinoTncStatusFrame;
pub use txtest::NinoTncTxTestFrame;

/// The NinoTNC's documented KISS baud rate (8N1). Matches the C#
/// `NinoTncSerialPort.DefaultBaudRate` / `KissSerialModem.DefaultBaudRate`.
pub const DEFAULT_BAUD: u32 = 57_600;
