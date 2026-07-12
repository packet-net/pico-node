//! The payload (de)compressor for a compression-negotiated NET/ROM L4 circuit —
//! the thin, parity-named adapter the circuit calls, over the [`deflate`] zlib
//! codec. It mirrors the C# `Packet.NetRom.Transport.NetRomCompression` surface
//! (`Compress` / `TryDecompress`) and fixes the decompress cap at the BPQ-matching
//! 8 KiB so the circuit hook points read cleanly.
//!
//! The circuit performs the *framing* (compress the whole logical send as one
//! zlib stream, then fragment at 236 bytes with the [`FLAG_COMPRESSED`] flag on
//! every fragment; reassemble all more-follows fragments, then inflate the
//! concatenation once). This module is just the codec seam: `compress` deflates,
//! `try_decompress` inflates fail-closed (a corrupt / oversized stream returns
//! `None` so the circuit drops the frame rather than crashing).
//!
//! Gated behind the `netrom-compress` feature (via the parent module).
//!
//! [`deflate`]: super::deflate
//! [`FLAG_COMPRESSED`]: crate::netrom::wire::FLAG_COMPRESSED

use alloc::vec::Vec;

use super::deflate::{self, DEFAULT_MAX_INFLATE};

/// The cap on a single decompressed logical frame — generous headroom over the
/// 236-byte fragment size, matching LinBPQ's 8 KiB inflate buffer and the C#
/// `NetRomCircuit.MaxDecompressedFrame`. A compressed frame that expands past this
/// is treated as corrupt and dropped.
pub const MAX_DECOMPRESSED_FRAME: usize = DEFAULT_MAX_INFLATE; // 8192

/// Compress `data` into a zlib stream (RFC 1950) that LinBPQ's `doinflate`
/// accepts. Mirrors C# `NetRomCompression.Compress`.
pub fn compress(data: &[u8]) -> Vec<u8> {
    deflate::zlib_compress(data)
}

/// Decompress a zlib stream produced by LinBPQ (or by [`compress`]) back to the
/// original bytes, capped at [`MAX_DECOMPRESSED_FRAME`]. Returns `None` (never
/// panics) if `data` is not a valid zlib stream or expands past the cap — a
/// corrupt or truncated compressed frame must fail closed, not crash the circuit.
/// Mirrors C# `NetRomCompression.TryDecompress` (the `out`/`bool` shape becomes an
/// `Option`).
pub fn try_decompress(data: &[u8]) -> Option<Vec<u8>> {
    deflate::zlib_decompress(data, MAX_DECOMPRESSED_FRAME).ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trips_a_realistic_payload() {
        let data = b"GB7RDG NET/ROM node; connect from M0LTE-7; more follows. ".repeat(20);
        let z = compress(&data);
        assert!(z.len() < data.len(), "realistic text should shrink");
        assert_eq!(try_decompress(&z).unwrap(), data);
    }

    #[test]
    fn corrupt_stream_fails_closed() {
        let mut z = compress(b"the quick brown fox jumps over the lazy dog");
        let n = z.len();
        z[n - 1] ^= 0xFF; // clobber the Adler-32 trailer
        assert!(try_decompress(&z).is_none());
    }

    #[test]
    fn oversized_stream_fails_closed_at_the_cap() {
        // >8 KiB of highly compressible data inflates past MAX_DECOMPRESSED_FRAME.
        let big = alloc::vec![b'Z'; MAX_DECOMPRESSED_FRAME + 1];
        let z = compress(&big);
        assert!(try_decompress(&z).is_none());
    }
}
