//! The over-air NinoTNC TX-Test UI frame.
//!
//! Ports `Packet.Kiss.NinoTnc.NinoTncAirTestFrame`. This is the on-air signal a
//! NinoTNC transmits when its operator presses the front-panel TX-Test button — the
//! *receiving* modem delivers it to its host as a normal KISS Data frame, so we see
//! it as a parsed AX.25 [`Frame`]. (The transmitting modem *also* emits the
//! synthetic host-side diagnostic — see [`super::txtest`].)
//!
//! Observed shape (firmware v3.44, modes 6 & 7 verified):
//! - UI frame, control `0x03`, PID `0xF0`
//! - Destination `CQBEEP-5`
//! - Source = the modem's *learned* callsign (the first callsign it saw transmitted
//!   through itself since power-on)
//! - INFO = `"{N "` then 50 bytes of printable ASCII starting at `0x20 + N` and
//!   stepping +1 — total INFO length always 53 bytes; `N` is a per-press counter
//!   (digit `1..9`).
//!
//! `no_std`: works on the borrowed parsed frame; the recognized window is returned
//! as a slice into the frame's `info` (no allocation).

use crate::ax25::{Callsign, Frame};

/// The fixed length of an over-air TX-Test INFO field: `{N ` (3) + 50 pattern bytes.
pub const INFO_LEN: usize = 53;
/// The length of the stepping-ASCII pattern window after the `{N ` prefix.
pub const PATTERN_LEN: usize = 50;

/// A recognized over-air NinoTNC TX-Test frame.
///
/// Mirrors `NinoTncAirTestFrame`. The pattern bytes borrow the source frame's info.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NinoTncAirTestFrame<'a> {
    /// The callsign the transmitting modem has learned (the frame's source).
    pub learned_callsign: Callsign,
    /// The per-press counter (the digit between `{` and the space).
    pub sequence_counter: u8,
    /// The 50-byte printable-ASCII window after the `{N ` prefix.
    pub pattern: &'a [u8],
}

impl<'a> NinoTncAirTestFrame<'a> {
    /// Try to recognize `frame` as a NinoTNC over-air TX-Test frame. Returns `None`
    /// if it is not a UI frame to `CQBEEP-5` with the exact `{N ` + 50-stepping-byte
    /// INFO shape.
    ///
    /// Mirrors `NinoTncAirTestFrame.TryRecognise`.
    pub fn try_recognise(frame: &'a Frame) -> Option<Self> {
        if !frame.is_ui() {
            return None;
        }
        let dest = &frame.destination.callsign;
        if dest.base() != b"CQBEEP" || dest.ssid() != 5 {
            return None;
        }

        let info = &frame.info;
        // "{N " + 50-byte pattern = exactly 53 bytes.
        if info.len() != INFO_LEN || info[0] != b'{' || info[2] != b' ' {
            return None;
        }
        if !info[1].is_ascii_digit() {
            return None;
        }
        let n = info[1] - b'0';

        // Pattern bytes start at 0x20 + N and increment by 1.
        let mut expected = 0x20u8.wrapping_add(n);
        for &b in &info[3..] {
            if b != expected {
                return None;
            }
            expected = expected.wrapping_add(1);
        }

        Some(Self {
            learned_callsign: frame.source.callsign,
            sequence_counter: n,
            pattern: &info[3..],
        })
    }

    /// The INFO pattern as the printable-ASCII string it is (`""` if not UTF-8,
    /// which can't happen for a recognized frame — the bytes are all printable).
    pub fn pattern_as_ascii(&self) -> &str {
        core::str::from_utf8(self.pattern).unwrap_or("")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ax25::frame::{CONTROL_UI, PID_NO_LAYER3};
    use crate::ax25::Address;
    use alloc::vec::Vec;

    fn addr(call: &str, ssid: u8, crh: bool) -> Address {
        Address {
            callsign: Callsign::new(call.as_bytes(), ssid).unwrap(),
            crh,
            extension: false,
        }
    }

    fn ui_to_cqbeep(info: Vec<u8>) -> Frame {
        Frame {
            destination: addr("CQBEEP", 5, true),
            source: addr("M0LTE", 0, false),
            digipeaters: Vec::new(),
            control: CONTROL_UI,
            pid: Some(PID_NO_LAYER3),
            info,
        }
    }

    // The exact INFO bytes captured on the 2026-05-14 dual-listener experiment
    // (mode 6, first press) — from the C# `NinoTncAirTestFrameTests.FirstPressInfo`.
    fn first_press_info() -> Vec<u8> {
        // "{1 " then 0x21.. stepping for 50 bytes.
        let mut v = alloc::vec![b'{', b'1', b' '];
        v.extend((0x21u8..).take(PATTERN_LEN));
        v
    }

    #[test]
    fn recognises_the_first_captured_press() {
        let frame = ui_to_cqbeep(first_press_info());
        let air = NinoTncAirTestFrame::try_recognise(&frame).unwrap();
        assert_eq!(air.learned_callsign.base(), b"M0LTE");
        assert_eq!(air.sequence_counter, 1);
        assert_eq!(air.pattern.len(), 50);
        assert!(air.pattern_as_ascii().starts_with("!\"#$%&'()*"));
        assert!(air.pattern_as_ascii().ends_with("OPQR"));
    }

    #[test]
    fn recognises_the_second_press_with_counter_two() {
        // Counter 2, window shifted +1 (starts at 0x22).
        let mut info = alloc::vec![b'{', b'2', b' '];
        info.extend((0x22u8..).take(PATTERN_LEN));
        let frame = ui_to_cqbeep(info);
        let air = NinoTncAirTestFrame::try_recognise(&frame).unwrap();
        assert_eq!(air.sequence_counter, 2);
        assert!(air.pattern_as_ascii().starts_with("\"#$%&'()*+"));
        assert!(air.pattern_as_ascii().ends_with("PQRS"));
    }

    #[test]
    fn wrong_destination_callsign_is_not_recognised() {
        let mut frame = ui_to_cqbeep(first_press_info());
        frame.destination = addr("CQ", 0, true);
        assert!(NinoTncAirTestFrame::try_recognise(&frame).is_none());
    }

    #[test]
    fn wrong_ssid_on_destination_is_not_recognised() {
        let mut frame = ui_to_cqbeep(first_press_info());
        frame.destination = addr("CQBEEP", 4, true);
        assert!(NinoTncAirTestFrame::try_recognise(&frame).is_none());
    }

    #[test]
    fn non_ui_frame_is_not_recognised() {
        let mut frame = ui_to_cqbeep(first_press_info());
        frame.control = 0x00; // I frame
        assert!(NinoTncAirTestFrame::try_recognise(&frame).is_none());
    }

    #[test]
    fn wrong_length_info_is_not_recognised() {
        let frame = ui_to_cqbeep(first_press_info()[..30].to_vec());
        assert!(NinoTncAirTestFrame::try_recognise(&frame).is_none());
    }

    #[test]
    fn wrong_pattern_step_is_not_recognised() {
        // Correct length but the window descends instead of ascending.
        let mut info = alloc::vec![b'{', b'1', b' '];
        let mut b = 0x52u8;
        for _ in 0..50 {
            info.push(b);
            b = b.wrapping_sub(1);
        }
        let frame = ui_to_cqbeep(info);
        assert!(NinoTncAirTestFrame::try_recognise(&frame).is_none());
    }
}
