//! Compact, correctness-critical zlib / DEFLATE codec for NET/ROM L4 payload
//! compression (BPQ `L4Compress` interop).
//!
//! This is the Rust port of the C# `Packet.NetRom.Transport.NetRomCompression`
//! reference (`src/Packet.NetRom/Transport/NetRomCompression.cs`). BPQ's L4
//! compression puts the user-data body on the wire as a **zlib stream (RFC 1950)**
//! — a 2-octet zlib header, a raw DEFLATE (RFC 1951) body, and an Adler-32
//! trailer — NOT raw deflate. LinBPQ inflates with a plain `inflateInit`/`inflate`
//! (default window bits, so it expects the zlib wrapper) and deflates with
//! `deflateInit(Z_BEST_COMPRESSION)`. So we must read/write exactly that framing.
//!
//! ## What lives here
//!
//! - [`zlib_decompress`] — full RFC-1951 inflate (stored + fixed-Huffman +
//!   dynamic-Huffman blocks), wrapped per RFC-1950 (parse + verify the 2-octet
//!   zlib header, verify the trailing Adler-32). A caller-supplied output cap
//!   ([`DEFAULT_MAX_INFLATE`] = 8 KiB, matching BPQ's inflate buffer). It is
//!   **fail-closed**: any malformed input, bad Adler-32, or cap-exceed returns
//!   `Err` — a corrupt compressed frame must never crash the circuit.
//! - [`zlib_compress`] — a compact greedy LZ77 (hash-chain match finder) emitting
//!   a single **fixed-Huffman** block (valid RFC-1951 that any zlib inflater —
//!   miniz, zlib, BPQ's `doinflate` — accepts), wrapped in the zlib header +
//!   Adler-32. The ratio need only be "useful", not optimal; a fixed-Huffman
//!   encoder is far smaller than a dynamic-Huffman one, which is the whole point
//!   of hand-rolling this rather than pulling in a full deflate crate.
//!
//! Both directions are proven against the `miniz_oxide` reference in the inline
//! test module (a `[dev-dependencies]` test oracle that never ships).
//!
//! `no_std`; the growable output buffers use `alloc::vec::Vec` (the crate's
//! existing streaming-buffer pattern). Integer-only, no `unsafe`, no panics on
//! any input.

use alloc::vec;
use alloc::vec::Vec;

/// Default cap on inflate output (octets). Matches BPQ's 8 KiB inflate buffer and
/// the C# `NetRomCircuit.MaxDecompressedFrame = 8192`. A stream that expands past
/// the caller's cap fails closed.
pub const DEFAULT_MAX_INFLATE: usize = 8192;

/// The failure modes of [`zlib_decompress`]. All are fail-closed: the caller drops
/// the frame. The variants are informational (useful in tests / logging); the
/// circuit treats any `Err` identically (drop, still ack so the sender advances).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ZlibError {
    /// The 2-octet zlib header is missing, not DEFLATE/CM=8, has an out-of-range
    /// window (CINFO > 7), fails the mod-31 check, or requests a preset dictionary
    /// (FDICT) we cannot supply.
    BadHeader,
    /// The stream ended before a complete DEFLATE structure could be decoded
    /// (truncated input, or a trailer shorter than the 4-octet Adler-32).
    Truncated,
    /// The DEFLATE body is structurally invalid (reserved block type, bad Huffman
    /// tables, a back-reference distance pointing before the output start, …).
    Malformed,
    /// Decoding would exceed the caller-supplied output cap.
    CapExceeded,
    /// The stream decoded, but its trailing Adler-32 does not match the checksum
    /// of the produced output — corrupt payload.
    BadChecksum,
}

// ---------------------------------------------------------------------------
// Adler-32 (RFC 1950 §9).
// ---------------------------------------------------------------------------

const ADLER_MOD: u32 = 65_521;

/// The Adler-32 checksum of `data` (the zlib trailer over the *uncompressed*
/// bytes). Integer-only; each step is reduced mod 65521 so no overflow occurs.
fn adler32(data: &[u8]) -> u32 {
    let mut a: u32 = 1;
    let mut b: u32 = 0;
    for &byte in data {
        a = (a + byte as u32) % ADLER_MOD;
        b = (b + a) % ADLER_MOD;
    }
    (b << 16) | a
}

// ---------------------------------------------------------------------------
// RFC 1951 constants (length / distance base + extra-bit tables).
// ---------------------------------------------------------------------------

/// Base length for length symbols 257..=285 (index = symbol - 257).
const LENGTH_BASE: [u16; 29] = [
    3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27, 31, 35, 43, 51, 59, 67, 83, 99, 115, 131,
    163, 195, 227, 258,
];
/// Extra bits for length symbols 257..=285.
const LENGTH_EXTRA: [u8; 29] = [
    0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0,
];
/// Base distance for distance symbols 0..=29.
const DIST_BASE: [u16; 30] = [
    1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129, 193, 257, 385, 513, 769, 1025, 1537,
    2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577,
];
/// Extra bits for distance symbols 0..=29.
const DIST_EXTRA: [u8; 30] = [
    0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13,
    13,
];

/// The order in which the 19 code-length-code lengths appear in a dynamic block
/// header (RFC 1951 §3.2.7).
const CODE_LENGTH_ORDER: [usize; 19] = [
    16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15,
];

const MAX_BITS: usize = 15;
const MAX_LCODES: usize = 286;
const MAX_DCODES: usize = 30;
const FIX_LCODES: usize = 288;

// ===========================================================================
// INFLATE
// ===========================================================================

/// A canonical Huffman decode table, built from a per-symbol code-length array
/// using the counts + sorted-symbols method (as in Mark Adler's `puff.c`). Small
/// and allocation-light; `symbols` is the only heap use.
struct Huffman {
    /// `count[len]` = number of codes of bit-length `len` (index 1..=MAX_BITS).
    count: [u16; MAX_BITS + 1],
    /// Symbols, ordered by (bit-length, symbol value) — the canonical order.
    symbols: Vec<u16>,
}

impl Huffman {
    /// Build a Huffman table from `lengths` (bit-length per symbol; 0 = unused).
    ///
    /// Returns `Ok(left)` where `left` is the "slack" in the code space:
    /// `left == 0` is a complete code, `left > 0` is an incomplete code (some code
    /// space unused), and an **over-subscribed** code (`left` would go negative)
    /// is rejected as [`ZlibError::Malformed`]. The caller decides whether an
    /// incomplete code is acceptable in its context.
    fn build(lengths: &[u16]) -> Result<(Self, i32), ZlibError> {
        let mut count = [0u16; MAX_BITS + 1];
        for &len in lengths {
            // A length > MAX_BITS is structurally impossible from a valid stream;
            // guard anyway so indexing can never panic.
            if len as usize > MAX_BITS {
                return Err(ZlibError::Malformed);
            }
            count[len as usize] += 1;
        }

        // Compute how much of the code space is left over (the completeness check).
        let mut left: i32 = 1;
        for &c in &count[1..=MAX_BITS] {
            left <<= 1;
            left -= c as i32;
            if left < 0 {
                // Over-subscribed: more codes of this length than the space allows.
                return Err(ZlibError::Malformed);
            }
        }

        // Offsets into the sorted symbol table for each length.
        let mut offsets = [0u16; MAX_BITS + 2];
        for len in 1..=MAX_BITS {
            offsets[len + 1] = offsets[len] + count[len];
        }

        // Place symbols into the table in canonical order.
        let total: usize = lengths.iter().filter(|&&l| l != 0).count();
        let mut symbols = vec![0u16; total];
        for (symbol, &len) in lengths.iter().enumerate() {
            if len != 0 {
                symbols[offsets[len as usize] as usize] = symbol as u16;
                offsets[len as usize] += 1;
            }
        }

        Ok((Huffman { count, symbols }, left))
    }
}

/// The streaming inflate state: an LSB-first bit reader over `data` plus the
/// growable output buffer and its cap.
struct Inflater<'a> {
    data: &'a [u8],
    /// Index of the next byte not yet pulled into `bit_buf`.
    pos: usize,
    /// Bit accumulator (LSB-first): the low `bit_cnt` bits are pending.
    bit_buf: u32,
    /// Number of valid bits currently in `bit_buf` (always < 8 after any `bits`).
    bit_cnt: u32,
    out: Vec<u8>,
    cap: usize,
}

impl<'a> Inflater<'a> {
    fn new(data: &'a [u8], cap: usize) -> Self {
        Inflater {
            data,
            pos: 0,
            bit_buf: 0,
            bit_cnt: 0,
            out: Vec::new(),
            cap,
        }
    }

    /// Pull `need` bits (0..=15) LSB-first from the stream. `Err(Truncated)` if the
    /// input runs out. Extra bits (length/distance) are read in this natural order;
    /// Huffman codes are read one bit at a time via this same path.
    fn bits(&mut self, need: u32) -> Result<u32, ZlibError> {
        while self.bit_cnt < need {
            if self.pos >= self.data.len() {
                return Err(ZlibError::Truncated);
            }
            self.bit_buf |= (self.data[self.pos] as u32) << self.bit_cnt;
            self.pos += 1;
            self.bit_cnt += 8;
        }
        let mask = if need == 0 { 0 } else { (1u32 << need) - 1 };
        let val = self.bit_buf & mask;
        self.bit_buf >>= need;
        self.bit_cnt -= need;
        Ok(val)
    }

    /// Decode one symbol using Huffman table `h`. Reads bits one at a time, MSB of
    /// the code first (the DEFLATE convention), and returns the symbol once the
    /// accumulated code falls inside a length's range. `Err(Malformed)` if the
    /// code runs past `MAX_BITS` without matching (an invalid / incomplete code).
    fn decode(&mut self, h: &Huffman) -> Result<u16, ZlibError> {
        let mut code: i32 = 0;
        let mut first: i32 = 0;
        let mut index: i32 = 0;
        for len in 1..=MAX_BITS {
            code |= self.bits(1)? as i32;
            let count = h.count[len] as i32;
            if code - count < first {
                let sym_index = (index + (code - first)) as usize;
                // sym_index is provably < symbols.len() for a valid table, but
                // guard so a malformed table can never panic.
                return h
                    .symbols
                    .get(sym_index)
                    .copied()
                    .ok_or(ZlibError::Malformed);
            }
            index += count;
            first += count;
            first <<= 1;
            code <<= 1;
        }
        Err(ZlibError::Malformed)
    }

    /// Append a byte to the output, enforcing the cap.
    fn push(&mut self, byte: u8) -> Result<(), ZlibError> {
        if self.out.len() >= self.cap {
            return Err(ZlibError::CapExceeded);
        }
        self.out.push(byte);
        Ok(())
    }

    /// A stored (uncompressed) block: byte-align, read LEN/NLEN, copy LEN octets.
    fn stored(&mut self) -> Result<(), ZlibError> {
        // Discard the partial byte in the accumulator to reach a byte boundary.
        // `bit_cnt` is always < 8 here, and those bits belong to the byte just
        // before `pos`, so restarting from `pos` is byte-aligned.
        self.bit_buf = 0;
        self.bit_cnt = 0;

        if self.pos + 4 > self.data.len() {
            return Err(ZlibError::Truncated);
        }
        let len = self.data[self.pos] as usize | ((self.data[self.pos + 1] as usize) << 8);
        let nlen = self.data[self.pos + 2] as usize | ((self.data[self.pos + 3] as usize) << 8);
        if nlen != (!len & 0xffff) {
            return Err(ZlibError::Malformed);
        }
        self.pos += 4;
        if self.pos + len > self.data.len() {
            return Err(ZlibError::Truncated);
        }
        if self.out.len() + len > self.cap {
            return Err(ZlibError::CapExceeded);
        }
        self.out
            .extend_from_slice(&self.data[self.pos..self.pos + len]);
        self.pos += len;
        Ok(())
    }

    /// Decode literal/length + distance codes until the end-of-block symbol (256).
    fn codes(&mut self, lencode: &Huffman, distcode: &Huffman) -> Result<(), ZlibError> {
        loop {
            let symbol = self.decode(lencode)?;
            if symbol < 256 {
                self.push(symbol as u8)?;
            } else if symbol == 256 {
                return Ok(());
            } else {
                // Length symbol (257..=285).
                let idx = (symbol - 257) as usize;
                if idx >= LENGTH_BASE.len() {
                    // 286/287 are invalid length codes (only reachable via the
                    // fixed table, which defines them but they never legally occur).
                    return Err(ZlibError::Malformed);
                }
                let extra = self.bits(LENGTH_EXTRA[idx] as u32)?;
                let length = LENGTH_BASE[idx] as usize + extra as usize;

                let dsym = self.decode(distcode)? as usize;
                if dsym >= DIST_BASE.len() {
                    return Err(ZlibError::Malformed);
                }
                let dextra = self.bits(DIST_EXTRA[dsym] as u32)?;
                let dist = DIST_BASE[dsym] as usize + dextra as usize;

                if dist > self.out.len() {
                    // Back-reference points before the start of the output.
                    return Err(ZlibError::Malformed);
                }
                if self.out.len() + length > self.cap {
                    return Err(ZlibError::CapExceeded);
                }
                // Copy byte-by-byte: overlapping copies (dist < length) are legal
                // and must read freshly-written bytes (RLE-style runs).
                let start = self.out.len() - dist;
                for k in 0..length {
                    let b = self.out[start + k];
                    self.out.push(b);
                }
            }
        }
    }

    /// A dynamic-Huffman block: read the code-length code, expand it into the
    /// literal/length + distance code lengths, build both tables, decode the body.
    fn dynamic(&mut self) -> Result<(), ZlibError> {
        let nlen = self.bits(5)? as usize + 257;
        let ndist = self.bits(5)? as usize + 1;
        let ncode = self.bits(4)? as usize + 4;
        if nlen > MAX_LCODES || ndist > MAX_DCODES {
            return Err(ZlibError::Malformed);
        }

        // Read the code-length code lengths (3 bits each) in the permuted order.
        let mut cl_lengths = [0u16; 19];
        for i in 0..ncode {
            cl_lengths[CODE_LENGTH_ORDER[i]] = self.bits(3)? as u16;
        }
        // Remaining entries stay 0 (already initialised).
        let (clcode, left) = Huffman::build(&cl_lengths)?;
        // The code-length code must be complete.
        if left != 0 {
            return Err(ZlibError::Malformed);
        }

        // Expand into nlen + ndist code lengths.
        let total = nlen + ndist;
        let mut lengths = [0u16; MAX_LCODES + MAX_DCODES];
        let mut index = 0usize;
        while index < total {
            let symbol = self.decode(&clcode)?;
            if symbol < 16 {
                lengths[index] = symbol;
                index += 1;
            } else {
                let (repeat, value) = match symbol {
                    16 => {
                        // Copy the previous code length 3..=6 times.
                        if index == 0 {
                            return Err(ZlibError::Malformed);
                        }
                        (3 + self.bits(2)? as usize, lengths[index - 1])
                    }
                    17 => (3 + self.bits(3)? as usize, 0), // repeat zero 3..=10
                    18 => (11 + self.bits(7)? as usize, 0), // repeat zero 11..=138
                    _ => return Err(ZlibError::Malformed),
                };
                if index + repeat > total {
                    return Err(ZlibError::Malformed);
                }
                for _ in 0..repeat {
                    lengths[index] = value;
                    index += 1;
                }
            }
        }

        // A block with no end-of-block code (256) is malformed.
        if lengths[256] == 0 {
            return Err(ZlibError::Malformed);
        }

        let (lencode, lleft) = Huffman::build(&lengths[..nlen])?;
        // The literal/length code must be complete (no legal incomplete case).
        if lleft != 0 {
            return Err(ZlibError::Malformed);
        }

        let (distcode, dleft) = Huffman::build(&lengths[nlen..total])?;
        // A distance code may legally be incomplete ONLY in the single-distance
        // special case: at most one code, of length 1 (all other symbols length 0).
        if dleft != 0 {
            let nonzero = distcode.symbols.len();
            let ones = distcode.count[1] as usize;
            if !(nonzero == ones && nonzero <= 1) {
                return Err(ZlibError::Malformed);
            }
        }

        self.codes(&lencode, &distcode)
    }

    /// Inflate the whole DEFLATE stream (a sequence of blocks) into `self.out`.
    fn inflate(&mut self) -> Result<(), ZlibError> {
        loop {
            let last = self.bits(1)?;
            let btype = self.bits(2)?;
            match btype {
                0 => self.stored()?,
                1 => {
                    let (lencode, distcode) = fixed_tables()?;
                    self.codes(&lencode, &distcode)?;
                }
                2 => self.dynamic()?,
                _ => return Err(ZlibError::Malformed), // reserved block type (3)
            }
            if last == 1 {
                return Ok(());
            }
        }
    }
}

/// Build the fixed-Huffman literal/length and distance tables (RFC 1951 §3.2.6).
/// Rebuilt per fixed block; fixed blocks are rare in real BPQ/zlib output, so the
/// simplicity is worth more than caching.
fn fixed_tables() -> Result<(Huffman, Huffman), ZlibError> {
    let mut ll = [0u16; FIX_LCODES];
    for (sym, slot) in ll.iter_mut().enumerate() {
        *slot = match sym {
            0..=143 => 8,
            144..=255 => 9,
            256..=279 => 7,
            _ => 8, // 280..=287
        };
    }
    let (lencode, _) = Huffman::build(&ll)?;

    // 30 distance codes, all length 5 (symbols 30/31 are absent — an incomplete
    // code, which is fine: they never legally occur).
    let dl = [5u16; MAX_DCODES];
    let (distcode, _) = Huffman::build(&dl)?;
    Ok((lencode, distcode))
}

/// Decompress a zlib stream (RFC 1950: 2-octet header + DEFLATE body + Adler-32)
/// back to the original bytes. Mirrors the C# `NetRomCompression.TryDecompress`.
///
/// **Fail-closed:** returns `Err` (never panics) on a bad zlib header, a malformed
/// or truncated DEFLATE body, output exceeding `max_output`, or a trailing
/// Adler-32 that does not match — a corrupt compressed frame must not crash the
/// circuit. `max_output` is the hard cap on produced octets (use
/// [`DEFAULT_MAX_INFLATE`] for the BPQ-matching 8 KiB).
pub fn zlib_decompress(data: &[u8], max_output: usize) -> Result<Vec<u8>, ZlibError> {
    // Smallest possible zlib stream: 2 header + >=2 body + 4 Adler.
    if data.len() < 6 {
        return Err(ZlibError::Truncated);
    }

    // ---- RFC 1950 header ----
    let cmf = data[0];
    let flg = data[1];
    let cm = cmf & 0x0f;
    let cinfo = cmf >> 4;
    if cm != 8 || cinfo > 7 {
        // Not DEFLATE, or a window larger than the 32 KiB we (and zlib) support.
        return Err(ZlibError::BadHeader);
    }
    let header = ((cmf as u16) << 8) | flg as u16;
    if !header.is_multiple_of(31) {
        return Err(ZlibError::BadHeader);
    }
    if (flg & 0x20) != 0 {
        // FDICT: a preset dictionary we cannot supply — refuse rather than
        // silently mis-decode. (BPQ/miniz never set it.)
        return Err(ZlibError::BadHeader);
    }

    // The DEFLATE body sits between the 2-octet header and the 4-octet Adler-32
    // trailer. This layout is exact for any well-formed zlib stream; trailing
    // garbage simply makes the Adler-32 check fail (still fail-closed).
    let body = &data[2..data.len() - 4];
    let trailer = &data[data.len() - 4..];

    let mut inflater = Inflater::new(body, max_output);
    inflater.inflate()?;
    let out = inflater.out;

    // ---- RFC 1950 Adler-32 trailer (big-endian, over the uncompressed data) ----
    let expected = ((trailer[0] as u32) << 24)
        | ((trailer[1] as u32) << 16)
        | ((trailer[2] as u32) << 8)
        | (trailer[3] as u32);
    if adler32(&out) != expected {
        return Err(ZlibError::BadChecksum);
    }

    Ok(out)
}

// ===========================================================================
// DEFLATE (compact greedy LZ77 + fixed Huffman)
// ===========================================================================

/// An LSB-first bit writer. DEFLATE packs the bit stream LSB-first within each
/// octet; Huffman codes are packed MSB-first, so [`Self::write_code`] reverses the
/// code's bits before emitting them LSB-first.
struct BitWriter {
    out: Vec<u8>,
    bit_buf: u32,
    bit_cnt: u32,
}

impl BitWriter {
    fn new() -> Self {
        BitWriter {
            out: Vec::new(),
            bit_buf: 0,
            bit_cnt: 0,
        }
    }

    /// Write the low `n` bits of `val` (0..=16 bits), LSB-first.
    fn write_bits(&mut self, val: u32, n: u32) {
        self.bit_buf |= (val & if n == 0 { 0 } else { (1u32 << n) - 1 }) << self.bit_cnt;
        self.bit_cnt += n;
        while self.bit_cnt >= 8 {
            self.out.push((self.bit_buf & 0xff) as u8);
            self.bit_buf >>= 8;
            self.bit_cnt -= 8;
        }
    }

    /// Write a canonical Huffman `code` of `len` bits. Canonical codes are defined
    /// MSB-first, but the stream is LSB-first, so reverse the `len` bits.
    fn write_code(&mut self, code: u32, len: u32) {
        self.write_bits(reverse_bits(code, len), len);
    }

    /// Flush any partial final octet (zero-padded) and return the buffer.
    fn finish(mut self) -> Vec<u8> {
        if self.bit_cnt > 0 {
            self.out.push((self.bit_buf & 0xff) as u8);
        }
        self.out
    }
}

/// Reverse the low `len` bits of `code`.
fn reverse_bits(mut code: u32, len: u32) -> u32 {
    let mut r = 0u32;
    for _ in 0..len {
        r = (r << 1) | (code & 1);
        code >>= 1;
    }
    r
}

/// The fixed-Huffman literal/length code for symbol `sym` (0..=287), as
/// `(code, bit_length)` with `code` in canonical MSB-first form. Computed directly
/// from the RFC 1951 §3.2.6 length assignment.
fn fixed_ll_code(sym: u16) -> (u32, u32) {
    match sym {
        // 7-bit codes, values 0b0000000..0b0010111.
        256..=279 => ((sym - 256) as u32, 7),
        // 8-bit codes, values 0b00110000..0b10111111.
        0..=143 => (0x30 + sym as u32, 8),
        // 8-bit codes, values 0b11000000..0b11000111.
        280..=287 => (0xc0 + (sym - 280) as u32, 8),
        // 9-bit codes, values 0b110010000..0b111111111.
        _ => (0x190 + (sym - 144) as u32, 9), // 144..=255
    }
}

/// Emit one literal byte using the fixed-Huffman literal/length code.
fn emit_literal(w: &mut BitWriter, byte: u8) {
    let (code, len) = fixed_ll_code(byte as u16);
    w.write_code(code, len);
}

/// Emit a length/distance back-reference: length symbol + extra bits, then
/// distance symbol (fixed 5-bit code) + extra bits.
fn emit_match(w: &mut BitWriter, length: usize, dist: usize) {
    // Length symbol: largest base <= length.
    let mut li = LENGTH_BASE.len() - 1;
    while LENGTH_BASE[li] as usize > length {
        li -= 1;
    }
    let (code, len) = fixed_ll_code(257 + li as u16);
    w.write_code(code, len);
    w.write_bits((length - LENGTH_BASE[li] as usize) as u32, LENGTH_EXTRA[li] as u32);

    // Distance symbol: largest base <= dist. Fixed distance codes are the 5-bit
    // canonical codes, i.e. code == symbol.
    let mut di = DIST_BASE.len() - 1;
    while DIST_BASE[di] as usize > dist {
        di -= 1;
    }
    w.write_code(di as u32, 5);
    w.write_bits((dist - DIST_BASE[di] as usize) as u32, DIST_EXTRA[di] as u32);
}

// ---- Greedy LZ77 hash-chain match finder ----

const MIN_MATCH: usize = 3;
const MAX_MATCH: usize = 258;
/// The DEFLATE 32 KiB sliding window.
const WINDOW: usize = 32_768;
/// Hash table size (2^13). A modest table: collisions only cost a little ratio,
/// and this keeps the transient encode allocation to ~32 KiB.
const HASH_BITS: u32 = 13;
const HASH_SIZE: usize = 1 << HASH_BITS;
/// Cap on hash-chain traversal per position (bounds worst-case encode time).
const MAX_CHAIN: usize = 128;
/// Sentinel for "no position" in the hash head/prev tables.
const NIL: u32 = u32::MAX;

/// Hash three bytes starting at `data[i]` into the hash table index.
#[inline]
fn hash3(data: &[u8], i: usize) -> usize {
    let h = ((data[i] as u32) << 10) ^ ((data[i + 1] as u32) << 5) ^ (data[i + 2] as u32);
    (h & (HASH_SIZE as u32 - 1)) as usize
}

/// Compress `data` into a zlib stream (RFC 1950) that any zlib inflater — miniz,
/// zlib, BPQ's `doinflate` — accepts. Mirrors the C# `NetRomCompression.Compress`.
///
/// Greedy LZ77 with a hash-chain match finder, emitting a single fixed-Huffman
/// block. The ratio is "useful", not optimal; the encoder is deliberately compact.
pub fn zlib_compress(data: &[u8]) -> Vec<u8> {
    // ---- RFC 1950 header: CM=8 (DEFLATE), CINFO=7 (32 KiB window), FDICT=0. ----
    // FLG = 0x01 makes (0x78<<8 | 0x01) % 31 == 0 with FLEVEL=0; the same header
    // shape the C# reference asserts (first byte 0x78, header % 31 == 0).
    let mut out: Vec<u8> = vec![0x78, 0x01];

    let deflate_body = deflate_fixed(data);
    out.extend_from_slice(&deflate_body);

    // ---- RFC 1950 trailer: Adler-32 of the uncompressed data, big-endian. ----
    let checksum = adler32(data);
    out.push((checksum >> 24) as u8);
    out.push((checksum >> 16) as u8);
    out.push((checksum >> 8) as u8);
    out.push(checksum as u8);

    out
}

/// Produce a single fixed-Huffman DEFLATE block (BFINAL=1) for `data`.
fn deflate_fixed(data: &[u8]) -> Vec<u8> {
    let mut w = BitWriter::new();
    // Block header: BFINAL=1, BTYPE=01 (fixed Huffman), LSB-first.
    w.write_bits(1, 1);
    w.write_bits(1, 2);

    let n = data.len();
    if n == 0 {
        // Just the end-of-block symbol.
        let (code, len) = fixed_ll_code(256);
        w.write_code(code, len);
        return w.finish();
    }

    // Hash chains: `head[h]` = most recent position with hash h; `prev[p]` = the
    // position with the same hash immediately before p.
    let mut head = vec![NIL; HASH_SIZE];
    let mut prev = vec![NIL; n];

    let mut i = 0usize;
    while i < n {
        let (mut best_len, mut best_dist) = (0usize, 0usize);

        if i + MIN_MATCH <= n {
            let h = hash3(data, i);
            let window_start = i.saturating_sub(WINDOW);
            let max_len = core::cmp::min(MAX_MATCH, n - i);
            let mut cur = head[h];
            let mut chain = MAX_CHAIN;
            while cur != NIL {
                let j = cur as usize;
                if j < window_start {
                    break; // older than the window; chain is strictly decreasing
                }
                // Extend the match at j vs i.
                let mut l = 0usize;
                while l < max_len && data[j + l] == data[i + l] {
                    l += 1;
                }
                if l > best_len {
                    best_len = l;
                    best_dist = i - j;
                    if l >= max_len {
                        break; // can't do better than the maximum
                    }
                }
                chain -= 1;
                if chain == 0 {
                    break;
                }
                cur = prev[j];
            }
        }

        if best_len >= MIN_MATCH {
            emit_match(&mut w, best_len, best_dist);
            // Insert every position the match covers so later matches can find them.
            let end = i + best_len;
            while i < end {
                if i + MIN_MATCH <= n {
                    let h = hash3(data, i);
                    prev[i] = head[h];
                    head[h] = i as u32;
                }
                i += 1;
            }
        } else {
            emit_literal(&mut w, data[i]);
            if i + MIN_MATCH <= n {
                let h = hash3(data, i);
                prev[i] = head[h];
                head[h] = i as u32;
            }
            i += 1;
        }
    }

    // End-of-block.
    let (code, len) = fixed_ll_code(256);
    w.write_code(code, len);
    w.finish()
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;
    use alloc::vec::Vec;

    // --- Corpus of inputs exercised by the round-trip / oracle tests. ---
    fn corpus() -> Vec<Vec<u8>> {
        // Realistic NET/ROM-ish text (repeats compress well).
        let text = b"GB7RDG:G8PZT-1} NET/ROM node broadcast: nodes RDGBBS via GB7RDG-7 \
                     quality 192, connect request from M0LTE-7 to G8PZT, more follows. ";
        let mut realistic = Vec::new();
        for _ in 0..40 {
            realistic.extend_from_slice(text);
        }
        // Pseudo-random / incompressible-ish (deterministic LCG).
        let mut rng: u32 = 0x1234_5678;
        let mut random = Vec::new();
        for _ in 0..2000 {
            rng = rng.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            random.push((rng >> 24) as u8);
        }
        // Mixed: text + run + text (forces multiple match/literal transitions).
        let mut mixed = text.to_vec();
        mixed.extend_from_slice(&vec![0x7Eu8; 500]);
        mixed.extend_from_slice(text);

        vec![
            Vec::new(),        // empty
            vec![0x42],        // one byte
            vec![0xAA; 1],     // one byte, different value
            vec![b'A'; 5000],  // highly repetitive (long RLE runs)
            vec![0u8; 300],    // zeros
            realistic,         // long repeated realistic text
            text.to_vec(),     // short realistic single copy
            random,            // incompressible-ish
            mixed,             // mixed match/literal transitions
        ]
    }

    // ---------------------------------------------------------------
    // 1. Self round-trip: inflate(deflate(x)) == x.
    // ---------------------------------------------------------------
    #[test]
    fn self_round_trip() {
        for input in corpus() {
            let compressed = zlib_compress(&input);
            let restored = zlib_decompress(&compressed, 1 << 20)
                .expect("our inflate must read our own deflate");
            assert_eq!(restored, input, "round-trip mismatch (len {})", input.len());
        }
    }

    #[test]
    fn our_output_has_zlib_framing() {
        // Matches the C# NetRomCompressionTests framing assertions.
        let c = zlib_compress(b"hello world hello world");
        assert_eq!(c[0], 0x78, "CMF must be 0x78 (CM=8, CINFO=7)");
        assert_eq!(
            (((c[0] as u16) << 8) | c[1] as u16) % 31,
            0,
            "zlib header must satisfy the mod-31 check"
        );
    }

    // ---------------------------------------------------------------
    // 2. Oracle A: miniz_oxide inflates OUR deflate output.
    // ---------------------------------------------------------------
    #[test]
    fn oracle_miniz_reads_our_output() {
        for input in corpus() {
            let ours = zlib_compress(&input);
            let via_miniz = miniz_oxide::inflate::decompress_to_vec_zlib(&ours)
                .expect("miniz must inflate our zlib output");
            assert_eq!(via_miniz, input, "miniz round-trip mismatch (len {})", input.len());
        }
    }

    // ---------------------------------------------------------------
    // 3. Oracle B: OUR inflate reads miniz_oxide's deflate output,
    //    including dynamic-Huffman blocks (levels 6..=10) and stored.
    // ---------------------------------------------------------------
    #[test]
    fn our_inflate_reads_miniz_output() {
        for input in corpus() {
            // Level 0 tends to stored blocks; 6/9/10 exercise dynamic Huffman.
            for level in [0u8, 1, 6, 9, 10] {
                let miniz = miniz_oxide::deflate::compress_to_vec_zlib(&input, level);
                let ours = zlib_decompress(&miniz, 1 << 20)
                    .unwrap_or_else(|e| panic!("our inflate failed on miniz L{level}: {e:?}"));
                assert_eq!(ours, input, "mismatch inflating miniz L{level} (len {})", input.len());
            }
        }
    }

    #[test]
    fn our_inflate_reads_dynamic_huffman() {
        // Explicitly assert the miniz stream we read back is a DYNAMIC block
        // (BTYPE=10), so this test genuinely covers the dynamic decoder.
        let text = b"the quick brown fox jumps over the lazy dog; \
                     the quick brown fox jumps over the lazy dog; pack my box.";
        let mut input = Vec::new();
        for _ in 0..30 {
            input.extend_from_slice(text);
        }
        let miniz = miniz_oxide::deflate::compress_to_vec_zlib(&input, 9);
        // Inspect first block type: header byte 2 is the first DEFLATE byte;
        // bit0 = BFINAL, bits1-2 = BTYPE (LSB-first).
        let first = miniz[2];
        let btype = (first >> 1) & 0b11;
        assert_eq!(btype, 0b10, "expected miniz L9 to emit a dynamic-Huffman block");
        let ours = zlib_decompress(&miniz, 1 << 20).expect("inflate dynamic block");
        assert_eq!(ours, input);
    }

    // ---------------------------------------------------------------
    // 4a. Adler-32 known vectors.
    // ---------------------------------------------------------------
    #[test]
    fn adler32_vectors() {
        assert_eq!(adler32(b""), 1);
        assert_eq!(adler32(b"a"), 0x0062_0062);
        assert_eq!(adler32(b"abc"), 0x024D_0127);
        assert_eq!(adler32(b"Wikipedia"), 0x11E6_0398);
    }

    // ---------------------------------------------------------------
    // 4b. Fail-closed: corrupt / bad-header / bad-checksum / cap.
    // ---------------------------------------------------------------
    #[test]
    fn rejects_too_short() {
        assert_eq!(zlib_decompress(&[], DEFAULT_MAX_INFLATE), Err(ZlibError::Truncated));
        assert_eq!(zlib_decompress(&[0x78], DEFAULT_MAX_INFLATE), Err(ZlibError::Truncated));
    }

    #[test]
    fn rejects_bad_header() {
        // CM != 8.
        let mut s = zlib_compress(b"data data data");
        s[0] = 0x77; // CM=7
        assert_eq!(zlib_decompress(&s, DEFAULT_MAX_INFLATE), Err(ZlibError::BadHeader));

        // Broken mod-31 check.
        let mut s2 = zlib_compress(b"data data data");
        s2[1] = s2[1].wrapping_add(1);
        assert_eq!(zlib_decompress(&s2, DEFAULT_MAX_INFLATE), Err(ZlibError::BadHeader));

        // CINFO > 7 (window too large).
        let mut s3 = zlib_compress(b"data data data");
        s3[0] = 0x88; // CINFO=8, CM=8
        // (may fail either the CINFO check or the mod-31 check — both are BadHeader)
        assert_eq!(zlib_decompress(&s3, DEFAULT_MAX_INFLATE), Err(ZlibError::BadHeader));

        // FDICT set.
        let mut s4 = zlib_compress(b"data data data");
        // Set FDICT (bit 5 of FLG) and fix the mod-31 check.
        s4[1] |= 0x20;
        // Recompute FCHECK so the header still passes mod-31, isolating FDICT.
        let base = ((s4[0] as u16) << 8) | (s4[1] as u16 & 0xE0); // keep CM/CINFO + FLEVEL+FDICT
        let rem = base % 31;
        let fcheck = if rem == 0 { 0 } else { 31 - rem };
        s4[1] = (s4[1] & 0xE0) | fcheck as u8;
        assert_eq!(zlib_decompress(&s4, DEFAULT_MAX_INFLATE), Err(ZlibError::BadHeader));
    }

    #[test]
    fn rejects_bad_checksum() {
        let mut s = zlib_compress(b"the quick brown fox");
        let n = s.len();
        s[n - 1] ^= 0xFF; // corrupt the Adler-32 trailer
        assert_eq!(zlib_decompress(&s, DEFAULT_MAX_INFLATE), Err(ZlibError::BadChecksum));
    }

    #[test]
    fn rejects_corrupt_body() {
        let s = zlib_compress(b"the quick brown fox jumps over the lazy dog");
        // Flip bits in the DEFLATE body (byte 4, past the header). This should
        // produce a malformed stream or a checksum failure — never a panic, never Ok.
        let mut corrupt = s.clone();
        corrupt[4] ^= 0xFF;
        let r = zlib_decompress(&corrupt, DEFAULT_MAX_INFLATE);
        assert!(r.is_err(), "corrupt body must fail closed, got {r:?}");
    }

    #[test]
    fn cap_exceeded_fails_closed() {
        // > 8 KiB of highly compressible data: compresses tiny, inflates past cap.
        let big = vec![b'Z'; 20_000];
        let compressed = zlib_compress(&big);
        assert!(compressed.len() < big.len());
        // With the 8 KiB cap it must fail closed...
        assert_eq!(
            zlib_decompress(&compressed, DEFAULT_MAX_INFLATE),
            Err(ZlibError::CapExceeded)
        );
        // ...but a generous cap decodes it fine (proves cap is the only reason).
        let ok = zlib_decompress(&compressed, 50_000).expect("decodes under a large cap");
        assert_eq!(ok, big);
    }

    #[test]
    fn cap_boundary_exact() {
        // Exactly-cap-sized output must succeed; one over must fail.
        let data = vec![b'q'; 1000];
        let c = zlib_compress(&data);
        assert!(zlib_decompress(&c, 1000).is_ok(), "exact cap must succeed");
        assert_eq!(zlib_decompress(&c, 999), Err(ZlibError::CapExceeded));
    }

    #[test]
    fn miniz_stored_incompressible_via_our_inflate() {
        // Random data at level 1 often lands in stored blocks; assert we read it.
        let mut rng: u32 = 0xDEAD_BEEF;
        let mut random = Vec::new();
        for _ in 0..4096 {
            rng = rng.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            random.push((rng >> 16) as u8);
        }
        let miniz = miniz_oxide::deflate::compress_to_vec_zlib(&random, 0);
        let ours = zlib_decompress(&miniz, 1 << 20).expect("read miniz stored blocks");
        assert_eq!(ours, random);
    }
}
