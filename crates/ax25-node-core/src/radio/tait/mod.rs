//! Tait TM8100/TM8200 radio integration — the Rust port of `Packet.Radio.Tait`.
//!
//! A Tait mobile driven over its **CCDI** serial control channel (Command mode)
//! exposes RSSI in 0.1 dB units, hardware carrier-sense (DCD) edges, transmitter
//! keying, channel selection, and a radio-native short-data-message side channel —
//! everything a bare KISS TNC cannot see.
//!
//! ## Module map
//!
//! - [`ccdi`] — the CCDI wire codec (checksum, frame, message, CR-line decoder).
//!   The shared serialization point; **frozen first**, everything else builds on it.
//! - [`driver`] — [`driver::TaitCcdiRadio`]: strict command builders + a thin
//!   transact/demux over the async [`crate::kiss::serial::ByteStream`] seam.
//! - [`transparent`] — AX.25 over the radio's own FFSK modem (KISS-SLIP over the
//!   Transparent-mode byte pipe), with the CCDI `t` / `+++` mode control.
//!
//! ## Parity notes
//!
//! CCDI is a prompt-disciplined request/response protocol carried over plain async
//! serial: line noise is normal, so the codec *rejects* rather than *throws* on a
//! bad line ([`ccdi::CcdiFrame::try_parse`] returns `None`). The wire codec is
//! byte-for-byte identical to the C# `Packet.Radio.Tait.Ccdi` types; the driver's
//! background-thread + `SemaphoreSlim` + `TaskCompletionSource` machinery maps onto
//! an `async` request/response over [`crate::kiss::serial::ByteStream`].

pub mod ccdi;
pub mod driver;

/// The CCDI serial rate these radios are commonly programmed for. The radio's
/// programmed rate wins — 1200 to 115200 are all possible (manual §1.8). Mirrors
/// `TaitCcdiRadio.DefaultBaudRate`.
pub const DEFAULT_BAUD: u32 = 28800;
