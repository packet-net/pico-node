//! A NinoTNC firmware image held in the APPDATA flash region, between the
//! upload (`POST /tnc/firmware`) and the flash (`ports::ninotnc`).
//!
//! The upload is validated and packed by
//! [`ax25_node_core::kiss::ninotnc::flash::HexStager`] (about 374 KB for
//! firmware 3.44, against 720 KB of APPDATA). Layout inside APPDATA:
//!
//! - sector 0: the header (magic, line count, packed length, CRC, target chip,
//!   file name), written **last**, so an interrupted or refused upload leaves no
//!   image; it is erased first thing on every new upload.
//! - from sector 1: the packed records, written a 256-byte page at a time.
//!
//! APPDATA's bounds come from the linker (`__appdata_start` / `__appdata_end`,
//! guarded by scripts/check-layout.sh), so nothing here can stray into the OTA
//! partitions or the config store.

use ax25_node_core::crc::Crc16;
use ax25_node_core::kiss::ninotnc::flash::{self, HexSummary, MAX_PACKED_RECORD};
use ax25_node_core::kiss::ninotnc::ChipVariant;
use embassy_rp::flash::ERASE_SIZE;

use crate::config_store;

const MAGIC: &[u8; 4] = b"NTFW";
const PAGE: usize = 256;
/// The longest file name kept for display.
pub const NAME_MAX: usize = 40;

/// What is stored.
#[derive(Clone, Copy)]
pub struct StagedImage {
    pub lines: u32,
    pub packed_len: u32,
    pub crc: u16,
    pub target: ChipVariant,
    name: [u8; NAME_MAX],
    name_len: u8,
}

impl StagedImage {
    /// The uploaded file's name.
    pub fn name(&self) -> &str {
        core::str::from_utf8(&self.name[..self.name_len as usize]).unwrap_or("")
    }
}

/// APPDATA's (start, end) as flash offsets.
fn region() -> (u32, u32) {
    extern "C" {
        static __appdata_start: u8;
        static __appdata_end: u8;
    }
    // SAFETY: linker-defined symbols; only their addresses (the values the
    // linker script assigns) are used, never the bytes behind them.
    unsafe {
        (
            &__appdata_start as *const u8 as u32,
            &__appdata_end as *const u8 as u32,
        )
    }
}

fn data_start() -> u32 {
    region().0 + ERASE_SIZE as u32
}

/// Streams packed records into APPDATA. Create with [`Writer::begin`].
pub struct Writer {
    next: u32,
    end: u32,
    page: [u8; PAGE],
    page_len: usize,
    failed: bool,
}

impl Writer {
    /// Invalidate any stored image and get ready to write a new one.
    pub fn begin() -> Result<Self, &'static str> {
        let (start, end) = region();
        match config_store::with_flash(|f| f.blocking_erase(start, start + ERASE_SIZE as u32)) {
            Some(Ok(())) => Ok(Self {
                next: data_start(),
                end,
                page: [0xFF; PAGE],
                page_len: 0,
                failed: false,
            }),
            Some(Err(_)) => Err("The node could not erase its image store."),
            None => Err("The node's flash is busy (a node firmware update?)."),
        }
    }

    /// Append bytes. `false` when the store is full or a write failed.
    pub fn write(&mut self, mut bytes: &[u8]) -> bool {
        while !bytes.is_empty() {
            if self.failed {
                return false;
            }
            let take = (PAGE - self.page_len).min(bytes.len());
            self.page[self.page_len..self.page_len + take].copy_from_slice(&bytes[..take]);
            self.page_len += take;
            bytes = &bytes[take..];
            if self.page_len == PAGE && !self.flush_page() {
                return false;
            }
        }
        true
    }

    fn flush_page(&mut self) -> bool {
        if self.next + PAGE as u32 > self.end {
            self.failed = true;
            return false;
        }
        let (at, page) = (self.next, self.page);
        let ok = config_store::with_flash(|f| {
            if at % ERASE_SIZE as u32 == 0 && f.blocking_erase(at, at + ERASE_SIZE as u32).is_err() {
                return false;
            }
            f.blocking_write(at, &page).is_ok()
        })
        .unwrap_or(false);
        self.next += PAGE as u32;
        self.page = [0xFF; PAGE];
        self.page_len = 0;
        self.failed = !ok;
        ok
    }

    /// Write the last page and then the header that makes the image valid.
    pub fn commit(mut self, summary: &HexSummary, name: &str) -> Result<StagedImage, &'static str> {
        if self.page_len > 0 && !self.flush_page() {
            return Err("The node could not write the image to flash.");
        }
        let mut image = StagedImage {
            lines: summary.lines,
            packed_len: summary.packed_len,
            crc: summary.crc,
            target: summary.target,
            name: [0; NAME_MAX],
            name_len: 0,
        };
        for (i, b) in name.bytes().filter(|b| (0x20..0x7F).contains(b)).take(NAME_MAX).enumerate() {
            image.name[i] = b;
            image.name_len = i as u8 + 1;
        }
        let mut hdr = [0xFFu8; PAGE];
        hdr[..4].copy_from_slice(MAGIC);
        hdr[4..8].copy_from_slice(&image.lines.to_le_bytes());
        hdr[8..12].copy_from_slice(&image.packed_len.to_le_bytes());
        hdr[12..14].copy_from_slice(&image.crc.to_le_bytes());
        hdr[14] = match image.target {
            ChipVariant::Dspic33Ep256 => 1,
            ChipVariant::Dspic33Ep512 => 2,
            ChipVariant::Unknown => 0,
        };
        hdr[15] = image.name_len;
        hdr[16..16 + NAME_MAX].copy_from_slice(&image.name);
        let check = ax25_node_core::crc::compute(&hdr[..16 + NAME_MAX]);
        hdr[16 + NAME_MAX..18 + NAME_MAX].copy_from_slice(&check.to_le_bytes());
        let (start, _) = region();
        match config_store::with_flash(|f| f.blocking_write(start, &hdr)) {
            Some(Ok(())) => Ok(image),
            _ => Err("The node could not write the image header."),
        }
    }
}

/// The stored image, if a complete one is there.
pub fn staged() -> Option<StagedImage> {
    let (start, _) = region();
    let mut hdr = [0u8; 18 + NAME_MAX];
    config_store::with_flash(|f| f.blocking_read(start, &mut hdr))?.ok()?;
    if &hdr[..4] != MAGIC {
        return None;
    }
    let check = u16::from_le_bytes([hdr[16 + NAME_MAX], hdr[17 + NAME_MAX]]);
    if ax25_node_core::crc::compute(&hdr[..16 + NAME_MAX]) != check {
        return None;
    }
    let mut name = [0u8; NAME_MAX];
    name.copy_from_slice(&hdr[16..16 + NAME_MAX]);
    Some(StagedImage {
        lines: u32::from_le_bytes([hdr[4], hdr[5], hdr[6], hdr[7]]),
        packed_len: u32::from_le_bytes([hdr[8], hdr[9], hdr[10], hdr[11]]),
        crc: u16::from_le_bytes([hdr[12], hdr[13]]),
        target: match hdr[14] {
            1 => ChipVariant::Dspic33Ep256,
            2 => ChipVariant::Dspic33Ep512,
            _ => ChipVariant::Unknown,
        },
        name,
        name_len: hdr[15].min(NAME_MAX as u8),
    })
}

/// Re-read the stored records and check them against the header's CRC, so a
/// damaged store is caught before the TNC is touched.
pub fn verify(image: &StagedImage) -> bool {
    let mut crc = Crc16::new();
    let mut buf = [0u8; 1024];
    let (mut at, end) = (data_start(), data_start() + image.packed_len);
    while at < end {
        let n = buf.len().min((end - at) as usize);
        match config_store::with_flash(|f| f.blocking_read(at, &mut buf[..n])) {
            Some(Ok(())) => crc.update(&buf[..n]),
            _ => return false,
        }
        at += n as u32;
    }
    crc.finish() == image.crc
}

/// Reads the stored lines back in order, as text ready for the bootloader.
pub struct LineReader {
    at: u32,
    end: u32,
    index: u32,
}

impl LineReader {
    pub fn new(image: &StagedImage) -> Self {
        Self {
            at: data_start(),
            end: data_start() + image.packed_len,
            index: 0,
        }
    }

    /// Render line `index` (which must be the next one) into `out` (at least
    /// `flash::MAX_LINE + 1` bytes). Returns its length with the `\n`.
    pub fn next_line(&mut self, index: u32, out: &mut [u8]) -> Option<usize> {
        if index != self.index || self.at + 2 > self.end {
            return None;
        }
        let mut header = [0u8; 2];
        config_store::with_flash(|f| f.blocking_read(self.at, &mut header))?.ok()?;
        let (n, upper) = flash::record_header(header);
        if n > MAX_PACKED_RECORD - 2 || self.at + 2 + n as u32 > self.end {
            return None;
        }
        let mut record = [0u8; MAX_PACKED_RECORD];
        let at = self.at + 2;
        config_store::with_flash(|f| f.blocking_read(at, &mut record[..n]))?.ok()?;
        self.at += 2 + n as u32;
        self.index += 1;
        Some(flash::render_line(&record[..n], upper, out))
    }
}
