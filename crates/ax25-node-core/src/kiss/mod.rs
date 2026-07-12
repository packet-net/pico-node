//! KISS host-attach codec — SLIP-style framing of AX.25 frames over a byte
//! stream (TCP to net-sim, or UART to a NinoTNC).
//!
//! Ports `Packet.Kiss` from `m0lte/packet.net`:
//! [`KissFraming`](frame) constants, [`Command`](frame::Command),
//! [`Frame`](frame::Frame), [`encode`](encoder)/[`encode_into`](encoder), and the
//! stateful streaming [`Decoder`](decoder).
//!
//! Covers both KISS transports the C# node exposes:
//! - **KISS-over-TCP** (`Packet.Kiss.KissTcpClient`) → net-sim over WiFi.
//! - **KISS-over-serial** (`Packet.Kiss.Serial.KissSerialModem`) → NinoTNC UART.
//!
//! The codec is transport-agnostic: the firmware's TCP and UART tasks both push
//! received bytes into a [`Decoder`] and frame outbound AX.25 bytes with
//! [`encode`]. All logic here is pure and host-tested.
//!
//! Beyond the base codec this module also carries the rest of `Packet.Kiss`'s
//! behaviour:
//! - [`ackmode`] — the G8BPQ ACKMODE extension (`Packet.Kiss.KissAckMode`).
//! - [`params`] — the KISS parameter-command builders (TXDELAY/P/SLOTTIME/…).
//! - [`classify`] — modem-agnostic inbound-frame classification
//!   (`Packet.Kiss.KissFrameClassifier`).
//! - [`serial`] — the serial-KISS transport seam over an async byte stream
//!   (`Packet.Kiss.Serial.KissSerialModem` + the `IKissModem` seam), host-testable.
//! - [`reconnect`] — the portable capped-backoff reconnect policy
//!   (`Packet.Node.Core.Transports.ReconnectingKissModem`'s decision core).
//! - [`ninotnc`] — the NinoTNC-specific extensions (`Packet.Kiss.NinoTnc`): the mode
//!   catalog, SETHW mode byte, the TX-Test frames, and the NinoTNC-aware classifier.

pub mod ackmode;
pub mod classify;
pub mod decoder;
pub mod encoder;
pub mod frame;
pub mod ninotnc;
pub mod params;
pub mod reconnect;
pub mod serial;

pub use ackmode::{AckCorrelator, AckRegisterError, TxCompletion, DEFAULT_ACK_TIMEOUT_MS};
pub use classify::{classify, InboundEvent};
pub use decoder::Decoder;
pub use encoder::{encode, encode_into, max_encoded_len};
pub use frame::{Command, Frame, EXIT_KISS_MODE, FEND, FESC, TFEND, TFESC};
pub use reconnect::{ReconnectAction, ReconnectPolicy, ReconnectingLink};
pub use serial::{ByteStream, ModemError, SerialKissModem};
