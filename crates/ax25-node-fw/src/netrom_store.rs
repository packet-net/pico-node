//! Persisted NET/ROM routing table — survives power failure, like BPQ's
//! `BPQNODES.dat`. Snapshot the *learned routes* and, at boot, replay them
//! through the live ingest path so the table rebuilds via exactly the code
//! that learns from on-air NODES broadcasts (no parallel "restore" logic to
//! drift). A rebooted node knows its routes immediately instead of waiting to
//! relearn from the next broadcast cycle.
//!
//! Storage: two 4 KiB flash sectors below the config store (`memory.x` keeps
//! all four out of the program region). Same A/B-generation + CRC + read-back
//! discipline as `config_store`. Saved on a timer (not per-broadcast) and only
//! when the table changed, to bound flash wear.
//!
//! Record: `"PNR1" | ver u8 | generation u32 | count u16 | route* | crc16`,
//! each route `nbr_call[7] | nbr_alias[6] | dest_call[7] | dest_alias[6] |
//! quality u8` (33 bytes) — the best route per destination, keyed by its
//! next-hop neighbour, which is all `ingest` needs to reconstruct the table.

use ax25_node_core::ax25::{Address, Callsign, ADDRESS_LEN};
use ax25_node_core::crc;
use ax25_node_core::netrom::wire::nodes_broadcast_builder::{
    write_nodes_frame, NodesAdvertisementEntry, MAX_NODES_FRAME_LEN,
};
use ax25_node_core::netrom::wire::{Alias, NodesBroadcast};
use ax25_node_core::netrom::PortId;

use embassy_rp::flash::{Blocking, Flash, ERASE_SIZE};
use embassy_rp::peripherals::FLASH;

use crate::config_store::FLASH_SIZE;
use crate::session::NetRom;

/// Routing sectors sit just below the two config sectors.
const SECTOR_A: u32 = (FLASH_SIZE - 4 * ERASE_SIZE) as u32;
const SECTOR_B: u32 = (FLASH_SIZE - 3 * ERASE_SIZE) as u32;

const MAGIC: &[u8; 4] = b"PNR1";
const VERSION: u8 = 1;
const HEADER_LEN: usize = 4 + 1 + 4 + 2; // magic+ver+generation+count
const ALIAS_LEN: usize = 6;
const ROUTE_LEN: usize = ADDRESS_LEN + ALIAS_LEN + ADDRESS_LEN + ALIAS_LEN + 1; // 33
/// Cap persisted routes — sized above the table's destination capacity.
const MAX_ROUTES: usize = 64;
const MAX_BODY: usize = MAX_ROUTES * ROUTE_LEN;

type ConfigFlash = Flash<'static, FLASH, Blocking, FLASH_SIZE>;

/// One persisted route: a destination reachable via a next-hop neighbour.
#[derive(Clone, Copy)]
struct Route {
    neighbour: Callsign,
    neighbour_alias: Alias,
    destination: Callsign,
    dest_alias: Alias,
    quality: u8,
}

fn encode_call(call: &Callsign, out: &mut [u8]) {
    let addr = Address {
        callsign: *call,
        crh: false,
        extension: false,
    };
    let _ = addr.encode(&mut out[..ADDRESS_LEN]);
}

fn decode_call(bytes: &[u8]) -> Option<Callsign> {
    Address::decode(&bytes[..ADDRESS_LEN]).map(|a| a.callsign)
}

fn encode_alias(a: &Alias, out: &mut [u8]) {
    let b = a.as_bytes();
    out[..ALIAS_LEN].fill(b' ');
    let n = b.len().min(ALIAS_LEN);
    out[..n].copy_from_slice(&b[..n]);
}

/// Snapshot the service's best-route-per-destination into a flat route list.
fn snapshot(netrom: &NetRom, routes: &mut heapless::Vec<Route, MAX_ROUTES>) {
    // Neighbour → alias lookup (the dest's best route only carries the callsign).
    let mut nbr_alias = heapless::Vec::<(Callsign, Alias), 32>::new();
    netrom.for_each_neighbour(|n| {
        let _ = nbr_alias.push((n.neighbour, n.alias));
    });
    let alias_of = |call: &Callsign| -> Alias {
        nbr_alias
            .iter()
            .find(|(c, _)| c == call)
            .map(|(_, a)| *a)
            .unwrap_or_default()
    };

    netrom.for_each_destination(|d| {
        if let Some(best) = d.best_route {
            let _ = routes.push(Route {
                neighbour: best.neighbour,
                neighbour_alias: alias_of(&best.neighbour),
                destination: d.destination,
                dest_alias: d.alias,
                quality: best.quality,
            });
        }
    });
}

/// Outcome of a [`save`] attempt.
pub enum SaveOutcome {
    /// The table content was unchanged since `prev_content_crc` — NO flash
    /// write occurred (the wear-saving fast path; a stable node hits this
    /// every save tick).
    Unchanged,
    /// Wrote `count` routes; carries the new content CRC for the next gate.
    Wrote { count: usize, content_crc: u16 },
    /// The table is empty — nothing persisted, no write.
    Empty,
}

/// CRC over just the route payload (not the header/generation) — the change
/// detector. Same routes ⇒ same CRC regardless of save generation.
fn content_crc(routes: &[Route]) -> u16 {
    let mut buf = [0u8; MAX_BODY];
    let mut at = 0;
    for r in routes {
        encode_call(&r.neighbour, &mut buf[at..]);
        encode_alias(&r.neighbour_alias, &mut buf[at + ADDRESS_LEN..]);
        encode_call(&r.destination, &mut buf[at + ADDRESS_LEN + ALIAS_LEN..]);
        encode_alias(&r.dest_alias, &mut buf[at + 2 * ADDRESS_LEN + ALIAS_LEN..]);
        buf[at + ROUTE_LEN - 1] = r.quality;
        at += ROUTE_LEN;
    }
    crc::compute(&buf[..at])
}

/// Write the current routing table to flash (alternating sector, generation+1)
/// — but ONLY if its content changed since `prev_content_crc`. A stable node's
/// table CRC never changes, so it never erases a sector: flash wear is bounded
/// by *topology churn*, not by the save cadence. (Routing state is non-critical
/// — re-learned from on-air NODES within a cycle — so saving conservatively
/// costs nothing.)
pub fn save(
    flash: &mut ConfigFlash,
    netrom: &NetRom,
    generation: u32,
    prev_content_crc: u16,
) -> Result<SaveOutcome, &'static str> {
    let mut routes = heapless::Vec::<Route, MAX_ROUTES>::new();
    snapshot(netrom, &mut routes);
    if routes.is_empty() {
        return Ok(SaveOutcome::Empty);
    }
    let content = content_crc(&routes);
    if content == prev_content_crc {
        return Ok(SaveOutcome::Unchanged); // no erase/write — the wear win
    }

    let mut record = [0xFFu8; HEADER_LEN + MAX_BODY + 2];
    record[0..4].copy_from_slice(MAGIC);
    record[4] = VERSION;
    let generation = generation + 1;
    record[5..9].copy_from_slice(&generation.to_le_bytes());
    record[9..11].copy_from_slice(&(routes.len() as u16).to_le_bytes());

    let mut at = HEADER_LEN;
    for r in &routes {
        encode_call(&r.neighbour, &mut record[at..]);
        encode_alias(&r.neighbour_alias, &mut record[at + ADDRESS_LEN..]);
        encode_call(&r.destination, &mut record[at + ADDRESS_LEN + ALIAS_LEN..]);
        encode_alias(
            &r.dest_alias,
            &mut record[at + 2 * ADDRESS_LEN + ALIAS_LEN..],
        );
        record[at + ROUTE_LEN - 1] = r.quality;
        at += ROUTE_LEN;
    }
    let crc = crc::compute(&record[..at]);
    record[at..at + 2].copy_from_slice(&crc.to_le_bytes());
    let total = at + 2;
    let padded = total.div_ceil(256) * 256;

    let offset = if generation % 2 == 0 {
        SECTOR_A
    } else {
        SECTOR_B
    };
    flash
        .blocking_erase(offset, offset + ERASE_SIZE as u32)
        .map_err(|_| "erase failed")?;
    flash
        .blocking_write(offset, &record[..padded])
        .map_err(|_| "write failed")?;
    Ok(SaveOutcome::Wrote {
        count: routes.len(),
        content_crc: content,
    })
}

/// Read one sector's routes. Returns `(generation, routes)` when valid.
fn read_sector(
    flash: &mut ConfigFlash,
    offset: u32,
) -> Option<(u32, heapless::Vec<Route, MAX_ROUTES>)> {
    let mut header = [0u8; HEADER_LEN];
    flash.blocking_read(offset, &mut header).ok()?;
    if &header[0..4] != MAGIC || header[4] != VERSION {
        return None;
    }
    let generation = u32::from_le_bytes([header[5], header[6], header[7], header[8]]);
    let count = u16::from_le_bytes([header[9], header[10]]) as usize;
    if count > MAX_ROUTES {
        return None;
    }
    let body_len = count * ROUTE_LEN;
    let mut body = [0u8; MAX_BODY + 2];
    flash
        .blocking_read(offset + HEADER_LEN as u32, &mut body[..body_len + 2])
        .ok()?;
    let stored_crc = u16::from_le_bytes([body[body_len], body[body_len + 1]]);
    let mut crc_buf = [0u8; HEADER_LEN + MAX_BODY];
    crc_buf[..HEADER_LEN].copy_from_slice(&header);
    crc_buf[HEADER_LEN..HEADER_LEN + body_len].copy_from_slice(&body[..body_len]);
    if crc::compute(&crc_buf[..HEADER_LEN + body_len]) != stored_crc {
        return None;
    }

    let mut routes = heapless::Vec::new();
    for i in 0..count {
        let at = i * ROUTE_LEN;
        let neighbour = decode_call(&body[at..])?;
        let neighbour_alias = Alias::from_str_lossy(
            core::str::from_utf8(&body[at + ADDRESS_LEN..at + ADDRESS_LEN + ALIAS_LEN])
                .unwrap_or("")
                .trim_end(),
        );
        let destination = decode_call(&body[at + ADDRESS_LEN + ALIAS_LEN..])?;
        let dest_alias = Alias::from_str_lossy(
            core::str::from_utf8(
                &body[at + 2 * ADDRESS_LEN + ALIAS_LEN..at + 2 * ADDRESS_LEN + 2 * ALIAS_LEN],
            )
            .unwrap_or("")
            .trim_end(),
        );
        let quality = body[at + ROUTE_LEN - 1];
        let _ = routes.push(Route {
            neighbour,
            neighbour_alias,
            destination,
            dest_alias,
            quality,
        });
    }
    Some((generation, routes))
}

/// Load the persisted routes and replay them into `netrom` via the live ingest
/// path. Returns `(generation, replayed_count)`; `(0, 0)` if no valid record.
pub fn load(flash: &mut ConfigFlash, netrom: &mut NetRom, my_call: Callsign) -> (u32, usize) {
    let a = read_sector(flash, SECTOR_A);
    let b = read_sector(flash, SECTOR_B);
    let (generation, routes) = match (a, b) {
        (Some(a), Some(b)) => {
            if a.0 >= b.0 {
                a
            } else {
                b
            }
        }
        (Some(x), None) | (None, Some(x)) => x,
        (None, None) => return (0, 0),
    };

    // Group routes by next-hop neighbour, build one NODES broadcast per
    // neighbour (sender = neighbour + its alias, entries = its destinations),
    // and ingest — exactly the on-air learning path. now=0: the obsolescence
    // resets to OBSINIT and confirms on the next live broadcast.
    let port_id = PortId::from_str_lossy("axudp");
    let mut done = heapless::Vec::<Callsign, 32>::new();
    let mut replayed = 0usize;

    for r in &routes {
        if done.iter().any(|c| *c == r.neighbour) {
            continue;
        }
        let _ = done.push(r.neighbour);

        let mut entries = heapless::Vec::<NodesAdvertisementEntry, 11>::new();
        for e in &routes {
            if e.neighbour == r.neighbour && entries.len() < entries.capacity() {
                let _ = entries.push(NodesAdvertisementEntry {
                    destination: e.destination,
                    destination_alias: e.dest_alias,
                    best_neighbour: e.neighbour,
                    quality: e.quality,
                });
            }
        }

        let mut buf = [0u8; MAX_NODES_FRAME_LEN];
        if let Some(len) = write_nodes_frame(&r.neighbour_alias, &entries, &mut buf) {
            if let Some(bc) = NodesBroadcast::try_parse(&buf[..len]) {
                netrom.ingest_broadcast(r.neighbour, my_call, port_id, &bc, 0);
                replayed += entries.len();
            }
        }
    }
    (generation, replayed)
}

/// Erase both routing sectors (factory reset / parallel to the config store).
#[allow(dead_code)] // consumed by the on-target test + the future factory-reset gesture
pub fn erase(flash: &mut ConfigFlash) -> Result<(), &'static str> {
    flash
        .blocking_erase(SECTOR_A, SECTOR_A + ERASE_SIZE as u32)
        .map_err(|_| "erase A failed")?;
    flash
        .blocking_erase(SECTOR_B, SECTOR_B + ERASE_SIZE as u32)
        .map_err(|_| "erase B failed")?;
    Ok(())
}
