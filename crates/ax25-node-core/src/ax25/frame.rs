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
//! control octet (Fig 4.1b: 7-bit N(S)/N(R), P/F at bit 0 of the second octet).
//! The transport layer can't know the link's modulo from the bytes alone (matching
//! the C# `AxudpSocket.ReceiveAsync` remark), so the caller picks the decode entry
//! point for the link's negotiated modulo: [`Frame::decode`] for modulo-8, and
//! [`Frame::decode_with_modulo`] for a link that may be extended (it consumes the
//! second octet on I/S frames and returns it). The frame-type discriminator bits
//! live in the first control octet in both modes, so classification is reliable.
//!
//! The second control octet is *not* stored on [`Frame`] — the struct stays the
//! modulo-8 envelope every transport in the crate constructs. It is threaded
//! explicitly: returned by [`Frame::decode_with_modulo`], read back through the
//! mode-aware [`Frame::nr_with`] / [`Frame::ns_with`] / [`Frame::poll_final_with`]
//! accessors, and supplied to [`Frame::encode_extended`]. The wire bytes are
//! byte-for-byte identical to the C# `Ax25Frame` extended codec.
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

    // ─── Mode-aware control accessors (mod-8 / mod-128) ─────────────────────
    //
    // The extended (mod-128) second control octet is NOT stored on this struct —
    // `Frame` stays the mod-8 envelope every transport in the crate constructs.
    // A modulo-aware caller threads the second octet (from [`Frame::decode_with_modulo`]
    // or the [`crate::sdl::bridge`] build path) into these accessors, which mirror
    // `Ax25Frame.Nr` / `Ax25Frame.Ns` / `Ax25Frame.PollFinal` (Ax25Frame.cs:96-119)
    // byte-for-byte. Pass `None` for a mod-8 frame (3-bit fields, P/F at bit 4);
    // `Some(ext)` for an extended I/S frame (7-bit fields, P/F at bit 0 of octet 2).

    /// N(R), mode-aware. `control_extension` is the second control octet of an
    /// extended (mod-128) I/S frame, or `None` for mod-8. 3-bit in mod-8 (control
    /// bits 7-5); 7-bit in mod-128 (second octet bits 7-1). Meaningless on U frames.
    pub fn nr_with(&self, control_extension: Option<u8>) -> u8 {
        match control_extension {
            Some(ext) => (ext >> 1) & 0x7F,
            None => (self.control >> 5) & 0x07,
        }
    }

    /// N(S), mode-aware. 3-bit in mod-8 (control bits 3-1); 7-bit in mod-128 (first
    /// control octet bits 7-1). Meaningful only on I frames — on an S frame the same
    /// bits encode the supervisory type, so the caller must check the frame type first.
    pub fn ns_with(&self, control_extension: Option<u8>) -> u8 {
        match control_extension {
            Some(_) => (self.control >> 1) & 0x7F,
            None => (self.control >> 1) & 0x07,
        }
    }

    /// The P/F bit, mode-aware. In mod-8 (and any U frame, 1 octet in both modes)
    /// it is bit 4 of the control octet; in an extended I/S frame it migrates to
    /// bit 0 of the second control octet (Fig 4.1b).
    pub fn poll_final_with(&self, control_extension: Option<u8>) -> bool {
        match control_extension {
            Some(ext) => (ext & 0x01) != 0,
            None => (self.control & 0x10) != 0,
        }
    }

    /// Command/response per §6.1.2: command = dest C-bit set, source C-bit clear.
    pub fn is_command(&self) -> bool {
        self.destination.crh && !self.source.crh
    }

    /// Response per §6.1.2: dest C-bit clear, source C-bit set.
    pub fn is_response(&self) -> bool {
        !self.destination.crh && self.source.crh
    }

    /// Decode a KISS-delivered AX.25 frame (no flags, no FCS), assuming modulo-8
    /// (a 1-octet control field). Use [`Frame::decode_with_modulo`] to decode a
    /// frame on a link operating at a known (possibly extended) modulo.
    pub fn decode(bytes: &[u8]) -> Result<Self, ParseError> {
        Self::decode_inner(bytes, false).map(|(frame, _)| frame)
    }

    /// Decode a KISS-delivered frame for a link at a known modulo. When `extended`,
    /// an I or S frame carries a 2-octet control field (Fig 4.1b); the second octet
    /// is consumed and returned alongside the frame. U frames are 1 octet in both
    /// modes, so the second octet is `None` for them and for every modulo-8 frame.
    /// The width is *not* derivable from the octets alone — the receive path, which
    /// knows the session's negotiated modulo, supplies `extended`. Mirrors
    /// `Ax25Frame.TryParse(..., bool extended, ...)` (Ax25Frame.cs:358-438).
    pub fn decode_with_modulo(
        bytes: &[u8],
        extended: bool,
    ) -> Result<(Self, Option<u8>), ParseError> {
        Self::decode_inner(bytes, extended)
    }

    /// Shared decode body. Returns the decoded frame plus the extended control
    /// octet (`Some` only for an extended I/S frame).
    fn decode_inner(bytes: &[u8], extended: bool) -> Result<(Self, Option<u8>), ParseError> {
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

        // Extended (modulo-128) I and S frames carry a 2-octet control field
        // (Fig 4.1b); U frames are 1 octet in both modes. The width can't be told
        // from the first octet alone, so the caller supplies the link's modulo via
        // `extended`. Frame-type discriminator: bits 1-0 = 11 → U. Mirrors
        // Ax25Frame.TryParse (Ax25Frame.cs:425-438).
        let is_u_frame = (control & 0x03) == 0x03;
        let mut control_extension = None;
        if extended && !is_u_frame {
            if bytes.len() < offset + 1 {
                return Err(ParseError::MissingControl);
            }
            control_extension = Some(bytes[offset]);
            offset += 1;
        }

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

        Ok((
            Self {
                destination,
                source,
                digipeaters,
                control,
                pid,
                info,
            },
            control_extension,
        ))
    }

    /// Number of bytes [`Frame::encode_into`] will write.
    pub fn encoded_len(&self) -> usize {
        ADDRESS_LEN * (2 + self.digipeaters.len()) + 1 + self.pid.map_or(0, |_| 1) + self.info.len()
    }

    /// Number of bytes [`Frame::encode_extended_into`] will write — as
    /// [`Frame::encoded_len`] but with the second (modulo-128) control octet.
    pub fn encoded_extended_len(&self) -> usize {
        ADDRESS_LEN * (2 + self.digipeaters.len()) + 2 + self.pid.map_or(0, |_| 1) + self.info.len()
    }

    /// Write the address field (destination + source + digipeaters) into `dst`,
    /// setting the extension bits so the last slot terminates the field. Returns
    /// the number of octets written. Shared by the modulo-8 and extended encoders.
    fn write_address_field(&self, dst: &mut [u8]) -> Option<usize> {
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
        Some(off)
    }

    /// Encode this frame (KISS body form, modulo-8: 1 control octet) into `dst`.
    /// Returns bytes written, or `None` if `dst` is too small. Sets the extension
    /// bits so the last address slot terminates the address field correctly.
    pub fn encode_into(&self, dst: &mut [u8]) -> Option<usize> {
        let n = self.encoded_len();
        if dst.len() < n {
            return None;
        }
        let mut off = self.write_address_field(dst)?;

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

    /// Encode this frame as an extended (modulo-128) I/S frame into `dst`:
    /// `self.control` is the first control octet (I: `(N(S) << 1)`; S: the base
    /// SS/"01" octet with high nibble zero), and `control_extension` the second
    /// (`(N(R) << 1) | P/F`, Fig 4.1b). Address / PID / info handling is identical
    /// to [`Frame::encode_into`]. The two control octets are transmitted first
    /// octet first (Ax25Frame.WriteTo, Ax25Frame.cs:303-312). Returns bytes written,
    /// or `None` if `dst` is too small.
    pub fn encode_extended_into(&self, control_extension: u8, dst: &mut [u8]) -> Option<usize> {
        let n = self.encoded_extended_len();
        if dst.len() < n {
            return None;
        }
        let mut off = self.write_address_field(dst)?;

        dst[off] = self.control;
        off += 1;
        dst[off] = control_extension;
        off += 1;

        if let Some(pid) = self.pid {
            dst[off] = pid;
            off += 1;
        }

        dst[off..off + self.info.len()].copy_from_slice(&self.info);
        off += self.info.len();

        Some(off)
    }

    /// Encode and return the bytes (modulo-8, 1 control octet).
    pub fn encode(&self) -> Vec<u8> {
        let mut buf = alloc::vec![0u8; self.encoded_len()];
        let n = self.encode_into(&mut buf).expect("buffer is exactly sized");
        buf.truncate(n);
        buf
    }

    /// Encode and return the bytes as an extended (modulo-128) I/S frame, with the
    /// supplied second control octet. See [`Frame::encode_extended_into`].
    pub fn encode_extended(&self, control_extension: u8) -> Vec<u8> {
        let mut buf = alloc::vec![0u8; self.encoded_extended_len()];
        let n = self
            .encode_extended_into(control_extension, &mut buf)
            .expect("buffer is exactly sized");
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

    // ─── Extended (modulo-128) control codec ────────────────────────────────

    /// Build the mod-128 first control octet for an I frame: `(N(S) << 1)`, bit0=0.
    fn ext_i_octet0(ns: u8) -> u8 {
        (ns & 0x7F) << 1
    }
    /// Build the mod-128 second control octet: `(N(R) << 1) | P/F` (Fig 4.1b).
    fn ext_octet1(nr: u8, pf: bool) -> u8 {
        ((nr & 0x7F) << 1) | if pf { 0x01 } else { 0 }
    }

    #[test]
    fn extended_i_frame_round_trips_with_7bit_seqs() {
        // N(S)=100, N(R)=50 — both beyond the mod-8 3-bit range, so this can only
        // round-trip through the 2-octet extended control field.
        let ns = 100u8;
        let nr = 50u8;
        let f = Frame {
            destination: addr("M0LTE", true),
            source: addr("G7XYZ", false),
            digipeaters: Vec::new(),
            control: ext_i_octet0(ns),
            pid: Some(PID_NO_LAYER3),
            info: b"extended payload".to_vec(),
        };
        let wire = f.encode_extended(ext_octet1(nr, true));

        let (g, ext) = Frame::decode_with_modulo(&wire, true).unwrap();
        assert!(ext.is_some(), "extended I frame must yield a second octet");
        assert!(g.is_information());
        assert_eq!(g.ns_with(ext), ns);
        assert_eq!(g.nr_with(ext), nr);
        assert!(g.poll_final_with(ext));
        assert_eq!(g.pid, Some(PID_NO_LAYER3));
        assert_eq!(g.info, b"extended payload");
    }

    #[test]
    fn extended_supervisory_frames_round_trip() {
        // Each S type carries only N(R) + P/F in the extended form. Base octet0 is
        // the SS/"01" nibble with the high nibble zero (§4.3.2 / Fig 4.3b).
        for (base, is_srej) in [(0x01u8, false), (0x05, false), (0x09, false), (0x0D, true)] {
            let nr = 99u8; // > 7: exercises the 7-bit field
            let f = Frame {
                destination: addr("M0LTE", false),
                source: addr("G7XYZ", true),
                digipeaters: Vec::new(),
                control: base,
                pid: None,
                info: Vec::new(),
            };
            let wire = f.encode_extended(ext_octet1(nr, false));

            let (g, ext) = Frame::decode_with_modulo(&wire, true).unwrap();
            assert!(ext.is_some());
            assert!(g.is_supervisory());
            assert_eq!(g.control & 0x0F, base, "S base octet survives");
            assert_eq!(g.nr_with(ext), nr);
            assert!(!g.poll_final_with(ext));
            assert_eq!(g.pid, None);
            assert!(g.info.is_empty());
            // mod-128 SREJ is just the extended form of the SREJ base — no separate path.
            assert_eq!(is_srej, (base == 0x0D));
        }
    }

    #[test]
    fn extended_sequence_wraps_at_127_to_0() {
        // The 7-bit field must carry 127 and 0 as distinct values (a mod-8 decode
        // would collapse them). Round-trip both boundaries for I (N(S)) and S (N(R)).
        for seq in [0u8, 127u8] {
            // I frame N(S)=seq, N(R)=seq.
            let f = Frame {
                destination: addr("A", true),
                source: addr("B", false),
                digipeaters: Vec::new(),
                control: ext_i_octet0(seq),
                pid: Some(PID_NO_LAYER3),
                info: b"x".to_vec(),
            };
            let wire = f.encode_extended(ext_octet1(seq, false));
            let (g, ext) = Frame::decode_with_modulo(&wire, true).unwrap();
            assert_eq!(g.ns_with(ext), seq);
            assert_eq!(g.nr_with(ext), seq);
        }
        // 127 and 0 must not alias: their encoded second octets differ.
        assert_ne!(ext_octet1(127, false), ext_octet1(0, false));
        assert_eq!(ext_octet1(127, false), 254); // (127<<1)
        assert_eq!(ext_octet1(0, false), 0);
    }

    #[test]
    fn mod8_decode_of_extended_i_frame_misreads_it() {
        // Demonstrates why decode is modulo-aware: a mod-8 decode of an extended
        // I frame swallows the second control octet as the PID (the latent bug the
        // extended path fixes). N(S)=10 (>7) also can't survive a 3-bit read.
        let f = Frame {
            destination: addr("A", true),
            source: addr("B", false),
            digipeaters: Vec::new(),
            control: ext_i_octet0(10),
            pid: Some(PID_NO_LAYER3),
            info: b"hi".to_vec(),
        };
        let wire = f.encode_extended(ext_octet1(3, false));

        // Correct (extended) decode.
        let (g_ext, ext) = Frame::decode_with_modulo(&wire, true).unwrap();
        assert_eq!(g_ext.ns_with(ext), 10);
        assert_eq!(g_ext.pid, Some(PID_NO_LAYER3));
        assert_eq!(g_ext.info, b"hi");

        // Wrong (mod-8) decode: the extended second octet is mistaken for the PID.
        let g_mod8 = Frame::decode(&wire).unwrap();
        assert_eq!(g_mod8.pid, Some(ext_octet1(3, false)));
        assert_ne!(g_mod8.info, b"hi");
    }

    #[test]
    fn extended_u_frame_has_no_second_octet() {
        // U frames are 1 octet in both modes — decode_with_modulo(extended) must
        // NOT consume a second octet for them.
        let f = ui_frame();
        let wire = f.encode();
        let (g, ext) = Frame::decode_with_modulo(&wire, true).unwrap();
        assert!(ext.is_none(), "U frame carries no extended control octet");
        assert_eq!(g, f);
    }
}
