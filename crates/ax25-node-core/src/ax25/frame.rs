//! AX.25 frame codec (KISS-delivered form: no HDLC flags, no FCS).
//!
//! Ports the essentials of `Packet.Ax25.Ax25Frame`. Layout per AX.25 v2.2 §3:
//!
//! ```text
//!   [destination 7B] [source 7B] [digipeaters 0..8 × 7B] [control 1..2B]
//!   [pid 0..1B] [info 0..N B]
//! ```
//!
//! Modulo-8 control is one octet; an extended (modulo-128) I/S frame has a second
//! control octet. The transport layer can't know the link's modulo from the bytes
//! alone (matching the C# `AxudpSocket.ReceiveAsync` remark), so [`Frame::decode`]
//! decodes the modulo-8 view and exposes the raw control octet; a session-aware
//! caller re-parses at the negotiated modulo. The frame-type discriminator bits
//! live in the first control octet in both modes, so classification is reliable.
//!
//! `alloc`-gated for the owned digipeater list + info buffer; a heapless variant
//! (fixed `MAX_DIGIPEATERS` array + an info slice into the caller's buffer) is the
//! noted embedded follow-up — the parsing math is identical.

use super::address::{Address, ADDRESS_LEN};
use alloc::vec::Vec;

/// Control byte for a UI frame, P bit clear.
pub const CONTROL_UI: u8 = 0x03;
/// Control byte for a UI frame, P/F bit set.
pub const CONTROL_UI_PF: u8 = 0x13;
/// PID 0xF0 — no Layer-3 protocol (§3.4).
pub const PID_NO_LAYER3: u8 = 0xF0;
/// PID 0xCF — NET/ROM.
pub const PID_NETROM: u8 = 0xCF;
/// PID 0x08 — segmented frame (§6.6).
pub const PID_SEGMENTED: u8 = 0x08;
/// Maximum number of Layer-2 repeater entries (§3.12.5).
pub const MAX_DIGIPEATERS: usize = 8;

/// Why a frame failed to decode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParseError {
    /// Fewer than the two mandatory address slots (14 octets).
    TooShort,
    /// A digipeater run that never terminated (no extension-bit-clear slot) or
    /// exceeded [`MAX_DIGIPEATERS`].
    BadAddressField,
    /// No control octet after the address field.
    MissingControl,
    /// An address slot's bytes were not a valid encoded callsign.
    BadAddress,
}

/// A decoded AX.25 frame (modulo-8 view of the control field).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Frame {
    /// Destination address slot.
    pub destination: Address,
    /// Source address slot.
    pub source: Address,
    /// Digipeater slots in path order (0–8).
    pub digipeaters: Vec<Address>,
    /// First (low-order) control octet.
    pub control: u8,
    /// PID octet — present on I and UI frames only.
    pub pid: Option<u8>,
    /// Information field (empty if absent).
    pub info: Vec<u8>,
}

impl Frame {
    /// True if the control octet identifies a UI frame (ignoring the P/F bit).
    pub fn is_ui(&self) -> bool {
        (self.control & 0xEF) == CONTROL_UI
    }

    /// True if this is an I frame (control bit 0 == 0). Modulo-8 view.
    pub fn is_information(&self) -> bool {
        (self.control & 0x01) == 0
    }

    /// True if this is a supervisory (S) frame (control low bits `0b01`).
    pub fn is_supervisory(&self) -> bool {
        (self.control & 0x03) == 0x01
    }

    /// True if this is an unnumbered (U) frame (control low bits `0b11`).
    pub fn is_unnumbered(&self) -> bool {
        (self.control & 0x03) == 0x03
    }

    /// The P/F bit (modulo-8 / U-frame position: bit 4).
    pub fn poll_final(&self) -> bool {
        (self.control & 0x10) != 0
    }

    /// Command/response per §6.1.2: command = dest C-bit set, source C-bit clear.
    pub fn is_command(&self) -> bool {
        self.destination.crh && !self.source.crh
    }

    /// Response per §6.1.2: dest C-bit clear, source C-bit set.
    pub fn is_response(&self) -> bool {
        !self.destination.crh && self.source.crh
    }

    /// Decode a KISS-delivered AX.25 frame (no flags, no FCS).
    pub fn decode(bytes: &[u8]) -> Result<Self, ParseError> {
        // Two mandatory address slots.
        if bytes.len() < ADDRESS_LEN * 2 {
            return Err(ParseError::TooShort);
        }
        let destination = Address::decode(&bytes[0..ADDRESS_LEN]).ok_or(ParseError::BadAddress)?;
        let source =
            Address::decode(&bytes[ADDRESS_LEN..ADDRESS_LEN * 2]).ok_or(ParseError::BadAddress)?;

        // The destination must NOT be the last address (its extension bit must be
        // clear — a source always follows). Matches the C# decode's guard.
        if destination.extension {
            return Err(ParseError::BadAddressField);
        }

        let mut offset = ADDRESS_LEN * 2;
        let mut digipeaters = Vec::new();
        // AX.25 HDLC extension bit: 0 on intermediate octets, 1 on the LAST
        // address octet (end of address). So we keep reading digipeaters while the
        // most recent slot's extension bit is CLEAR, and stop once it's set —
        // mirroring the C# `while (!lastAddress.ExtensionBit)` loop.
        let mut last_ext = source.extension;
        while !last_ext {
            if digipeaters.len() >= MAX_DIGIPEATERS {
                return Err(ParseError::BadAddressField);
            }
            if bytes.len() < offset + ADDRESS_LEN {
                return Err(ParseError::BadAddressField);
            }
            let digi = Address::decode(&bytes[offset..offset + ADDRESS_LEN])
                .ok_or(ParseError::BadAddress)?;
            last_ext = digi.extension;
            digipeaters.push(digi);
            offset += ADDRESS_LEN;
        }

        if bytes.len() < offset + 1 {
            return Err(ParseError::MissingControl);
        }
        let control = bytes[offset];
        offset += 1;

        // PID present on I and UI frames. I = bit0 clear; UI = (ctrl & 0xEF)==0x03.
        let is_i = (control & 0x01) == 0;
        let is_ui = (control & 0xEF) == CONTROL_UI;
        let (pid, info_start) = if (is_i || is_ui) && bytes.len() > offset {
            (Some(bytes[offset]), offset + 1)
        } else {
            (None, offset)
        };

        let info = bytes[info_start..].to_vec();

        // Normalise the per-slot HDLC extension bit out of the logical frame: it
        // is a wire-positional artifact (1 only on the last address octet) fully
        // determined by the digipeater count, and is re-derived on encode. Storing
        // it as a constant `false` means a constructed frame round-trips equal to
        // its decode, and downstream code never has to reason about a framing bit.
        let mut destination = destination;
        let mut source = source;
        destination.extension = false;
        source.extension = false;
        for d in &mut digipeaters {
            d.extension = false;
        }

        Ok(Self {
            destination,
            source,
            digipeaters,
            control,
            pid,
            info,
        })
    }

    /// Number of bytes [`Frame::encode_into`] will write.
    pub fn encoded_len(&self) -> usize {
        ADDRESS_LEN * (2 + self.digipeaters.len()) + 1 + self.pid.map_or(0, |_| 1) + self.info.len()
    }

    /// Encode this frame (KISS body form) into `dst`. Returns bytes written, or
    /// `None` if `dst` is too small. Sets the extension bits so the last address
    /// slot terminates the address field correctly.
    pub fn encode_into(&self, dst: &mut [u8]) -> Option<usize> {
        let n = self.encoded_len();
        if dst.len() < n {
            return None;
        }
        let mut off = 0;

        // Destination: extension clear iff there is no source after it — there
        // always is, so destination.extension is always true on the wire.
        write_addr(&mut dst[off..], &self.destination, true)?;
        off += ADDRESS_LEN;

        // Source: extension clear iff there are no digipeaters.
        let source_more = !self.digipeaters.is_empty();
        write_addr(&mut dst[off..], &self.source, source_more)?;
        off += ADDRESS_LEN;

        for (i, digi) in self.digipeaters.iter().enumerate() {
            let more = i + 1 < self.digipeaters.len();
            write_addr(&mut dst[off..], digi, more)?;
            off += ADDRESS_LEN;
        }

        dst[off] = self.control;
        off += 1;

        if let Some(pid) = self.pid {
            dst[off] = pid;
            off += 1;
        }

        dst[off..off + self.info.len()].copy_from_slice(&self.info);
        off += self.info.len();

        Some(off)
    }

    /// Encode and return the bytes.
    pub fn encode(&self) -> Vec<u8> {
        let mut buf = alloc::vec![0u8; self.encoded_len()];
        let n = self.encode_into(&mut buf).expect("buffer is exactly sized");
        buf.truncate(n);
        buf
    }
}

// Write an address slot with a forced extension bit (the address-field topology
// is the frame's concern, not the slot's, so we override it here).
fn write_addr(dst: &mut [u8], addr: &Address, more: bool) -> Option<()> {
    let forced = Address {
        extension: !more, // extension bit is 1 on the LAST slot in AX.25 (end-of-address)
        ..*addr
    };
    forced.encode(dst)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ax25::callsign::Callsign;

    fn addr(s: &str, crh: bool) -> Address {
        Address {
            callsign: Callsign::parse(s).unwrap(),
            crh,
            extension: false,
        }
    }

    fn ui_frame() -> Frame {
        Frame {
            destination: addr("APRS", true),
            source: addr("M0LTE-9", false),
            digipeaters: Vec::new(),
            control: CONTROL_UI,
            pid: Some(PID_NO_LAYER3),
            info: b"hello world".to_vec(),
        }
    }

    #[test]
    fn ui_frame_round_trips() {
        let f = ui_frame();
        let wire = f.encode();
        let g = Frame::decode(&wire).unwrap();
        assert_eq!(f, g);
    }

    #[test]
    fn classifies_ui() {
        let f = ui_frame();
        assert!(f.is_ui());
        assert!(f.is_unnumbered());
        assert!(!f.is_information());
        assert!(!f.is_supervisory());
    }

    #[test]
    fn command_response_bits() {
        let f = ui_frame(); // dest C set, source C clear => command
        assert!(f.is_command());
        assert!(!f.is_response());
    }

    #[test]
    fn decode_rejects_short() {
        assert_eq!(Frame::decode(&[0u8; 13]), Err(ParseError::TooShort));
    }

    #[test]
    fn frame_with_digipeaters_round_trips() {
        let mut f = ui_frame();
        f.digipeaters = alloc::vec![addr("WIDE1", false), addr("WIDE2-2", false)];
        let wire = f.encode();
        let g = Frame::decode(&wire).unwrap();
        assert_eq!(f.digipeaters.len(), 2);
        assert_eq!(g.digipeaters, f.digipeaters);
        assert_eq!(g, f);
    }

    #[test]
    fn extension_bit_terminates_address_field() {
        // With one digipeater, the address field is dest|source|digi; only the
        // digi slot should have the end-of-address (extension) bit set.
        let mut f = ui_frame();
        f.digipeaters = alloc::vec![addr("WIDE1", false)];
        let wire = f.encode();
        // dest ssid octet at index 6, source at 13, digi at 20.
        assert_eq!(wire[6] & 0x01, 0); // dest: more follows
        assert_eq!(wire[13] & 0x01, 0); // source: more follows
        assert_eq!(wire[20] & 0x01, 1); // digi: end of address
    }

    #[test]
    fn i_frame_carries_pid_and_info() {
        let f = Frame {
            destination: addr("M0LTE", true),
            source: addr("G7XYZ", false),
            digipeaters: Vec::new(),
            control: 0x00, // I frame, N(S)=0 N(R)=0 P=0
            pid: Some(PID_NO_LAYER3),
            info: b"data".to_vec(),
        };
        let wire = f.encode();
        let g = Frame::decode(&wire).unwrap();
        assert!(g.is_information());
        assert_eq!(g.pid, Some(PID_NO_LAYER3));
        assert_eq!(g.info, b"data");
    }

    #[test]
    fn s_frame_has_no_pid() {
        // RR, N(R)=0, response. Control low bits 01 => supervisory, no PID/info.
        let f = Frame {
            destination: addr("M0LTE", false),
            source: addr("G7XYZ", true),
            digipeaters: Vec::new(),
            control: 0x01,
            pid: None,
            info: Vec::new(),
        };
        let wire = f.encode();
        let g = Frame::decode(&wire).unwrap();
        assert!(g.is_supervisory());
        assert_eq!(g.pid, None);
        assert!(g.info.is_empty());
    }

    #[test]
    fn poll_final_bit_detected() {
        let mut f = ui_frame();
        f.control = CONTROL_UI_PF;
        let wire = f.encode();
        let g = Frame::decode(&wire).unwrap();
        assert!(g.poll_final());
    }
}
