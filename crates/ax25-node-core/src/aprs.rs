//! The APRS the node sends: telemetry reports, the telemetry label messages
//! (PARM / UNIT / EQNS / BITS) and a position report placed from a Maidenhead
//! locator. Build-only; the node parses no APRS.
//!
//! Each function returns the information field of a UI frame (PID 0xF0) to be
//! sent to [`TOCALL`]. The text matches what packet.net's `Packet.Aprs`
//! encoders write for the same values (APRS 1.2 / APRS12c chapters 8, 13, 14).

use alloc::format;
use alloc::string::String;

/// The destination address: packet.net's experimental tocall (`APZ001`).
pub const TOCALL: &str = "APZ001";

/// A telemetry report: `T#sss,aaa,aaa,aaa,aaa,aaa,bbbbbbbb` (APRS12c ch. 13).
/// `seq` is taken modulo 1000; the digital bits are sent bit 0 first.
pub fn telemetry_report(seq: u16, analog: &[u8; 5], digital: u8) -> String {
    let mut s = format!("T#{:03}", seq % 1000);
    for a in analog {
        s += &format!(",{a:03}");
    }
    s.push(',');
    for i in 0..8 {
        s.push(if digital & (1 << i) != 0 { '1' } else { '0' });
    }
    s
}

/// A message to `addressee` (padded to 9 characters) with no message id:
/// `:ADDRESSEE:text` (APRS12c ch. 14). The telemetry label messages are
/// addressed to the sending station itself.
pub fn message(addressee: &str, text: &str) -> String {
    format!(":{addressee:<9}:{text}")
}

/// Turn `value` into an 8-bit telemetry channel reading for the equation
/// `value = a * step`, rounding to nearest and clamping to 0-255. Both in the
/// same unit (millivolts, milliamps, ...).
pub fn quantise(value: i32, step: i32) -> u8 {
    if step <= 0 || value <= 0 {
        return 0;
    }
    ((value + step / 2) / step).min(255) as u8
}

/// The centre of a 4- or 6-character Maidenhead locator (e.g. `IO91` or
/// `IO91lk`), in hundredths of a minute of arc north and east (negative south
/// and west). `None` for anything else.
pub fn grid_centre(grid: &str) -> Option<(i32, i32)> {
    let g = grid.trim().as_bytes();
    if g.len() != 4 && g.len() != 6 {
        return None;
    }
    let field = |c: u8| {
        let c = c.to_ascii_uppercase();
        (b'A'..=b'R').contains(&c).then(|| (c - b'A') as i32)
    };
    let digit = |c: u8| c.is_ascii_digit().then(|| (c - b'0') as i32);
    let sub = |c: u8| {
        let c = c.to_ascii_uppercase();
        (b'A'..=b'X').contains(&c).then(|| (c - b'A') as i32)
    };
    // One degree is 6000 hundredths of a minute.
    let mut lon = field(g[0])? * 20 * 6000 - 180 * 6000 + digit(g[2])? * 2 * 6000;
    let mut lat = field(g[1])? * 10 * 6000 - 90 * 6000 + digit(g[3])? * 6000;
    if g.len() == 6 {
        // Subsquares are 5' of longitude by 2.5' of latitude.
        lon += sub(g[4])? * 500 + 250;
        lat += sub(g[5])? * 250 + 125;
    } else {
        lon += 6000;
        lat += 3000;
    }
    Some((lat, lon))
}

/// An uncompressed position report without timestamp, from a station without
/// messaging: `!DDMM.mmN/DDDMM.mmW` + symbol code + comment (APRS12c ch. 8).
/// Latitude and longitude in hundredths of a minute (see [`grid_centre`]).
pub fn position_report(lat: i32, lon: i32, table: char, code: char, comment: &str) -> String {
    let (ns, lat) = if lat < 0 { ('S', -lat) } else { ('N', lat) };
    let (ew, lon) = if lon < 0 { ('W', -lon) } else { ('E', lon) };
    format!(
        "!{:02}{:02}.{:02}{ns}{table}{:03}{:02}.{:02}{ew}{code}{comment}",
        lat / 6000,
        lat % 6000 / 100,
        lat % 100,
        lon / 6000,
        lon % 6000 / 100,
        lon % 100,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn telemetry_report_pads_and_orders_bits() {
        assert_eq!(
            telemetry_report(7, &[210, 12, 0, 0, 0], 0b0000_0101),
            "T#007,210,012,000,000,000,10100000"
        );
        assert_eq!(telemetry_report(1234, &[0; 5], 0), "T#234,000,000,000,000,000,00000000");
    }

    #[test]
    fn message_pads_the_addressee() {
        assert_eq!(message("M9YYY-9", "UNIT.V,A"), ":M9YYY-9  :UNIT.V,A");
        assert_eq!(message("GB7RDG-10", "x"), ":GB7RDG-10:x");
    }

    #[test]
    fn quantise_rounds_and_clamps() {
        assert_eq!(quantise(13_260, 60), 221);
        assert_eq!(quantise(13_289, 60), 221);
        assert_eq!(quantise(13_290, 60), 222);
        assert_eq!(quantise(-500, 100), 0);
        assert_eq!(quantise(40_000, 100), 255);
    }

    #[test]
    fn grid_centre_of_square_and_subsquare() {
        // IO91: 2W-0 lon, 51N-52N lat; centre 1W, 51.5N.
        assert_eq!(grid_centre("IO91"), Some((51 * 6000 + 3000, -6000)));
        // IO91lk: lon -2 deg + 11*5' + 2.5' = 1 deg 2.5' W, lat 51 deg + 10*2.5' + 1.25' = 51 deg 26.25' N.
        assert_eq!(grid_centre("io91LK"), Some((51 * 6000 + 2625, -2 * 6000 + 5750)));
        assert_eq!(grid_centre("IO9"), None);
        assert_eq!(grid_centre("ZZ99"), None);
    }

    #[test]
    fn position_report_formats_hemispheres() {
        let (lat, lon) = grid_centre("IO91lk").unwrap();
        assert_eq!(
            position_report(lat, lon, '/', 'n', "pico-node"),
            "!5126.25N/00102.50Wnpico-node"
        );
        assert_eq!(
            position_report(-(33 * 6000 + 5212), 151 * 6000 + 1234, '/', 'n', ""),
            "!3352.12S/15112.34En"
        );
    }

    /// The frames the node sends, as packet.net's `Packet.Aprs` encodes the
    /// same values (captured from its `ToInformationField`, 2026-09-26).
    #[test]
    fn matches_packet_net() {
        let me = "M9YYY-9";
        assert_eq!(message(me, "PARM.Battery,Current"), ":M9YYY-9  :PARM.Battery,Current");
        assert_eq!(message(me, "UNIT.V,A"), ":M9YYY-9  :UNIT.V,A");
        assert_eq!(message(me, "EQNS.0,0.06,0,0,0.1,0"), ":M9YYY-9  :EQNS.0,0.06,0,0,0.1,0");
        assert_eq!(
            message(me, "BITS.11111111,Station power"),
            ":M9YYY-9  :BITS.11111111,Station power"
        );
    }
}
