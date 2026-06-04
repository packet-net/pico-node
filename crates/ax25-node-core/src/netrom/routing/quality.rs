//! NET/ROM route-quality arithmetic — the multiplicative per-hop decay from the
//! canonical NET/ROM appendix. Quality is an integer 0 (worst) … 255 (best).
//!
//! Ports `Packet.NetRom.Routing.NetRomQuality`. When node A hears node B advertise
//! destination D at quality `bq`, A's quality for the route to D *via B* is the
//! advertised quality scaled by A's own path quality to B:
//!
//! ```text
//!   routequality = (broadcastquality × pathquality + 128) / 256   (integer, rounded)
//! ```
//!
//! The `+ 128` is round-to-nearest on the divide-by-256. Quality therefore decays
//! multiplicatively with each hop: a 200-quality direct link is ≈ 156 at two hops
//! (200 × 200 / 256) and ≈ 78 at three (last link 128). The practical per-hop /
//! floor conventions (direct link ~192–203, MINQUAL ~128–180) are *de-facto, not
//! normative* — they vary per implementation, so they live as configurable knobs
//! on [`super::NetRomRoutingOptions`], never hard-coded here.
//!
//! Pure `u32` integer math — no FPU touched (the M0+ has none), no allocation.

/// The maximum (best) quality value.
pub const MAX: u8 = 255;

/// The minimum (worst) quality value — a quality-0 route is never usable /
/// re-advertised.
pub const MIN: u8 = 0;

/// Combine an advertised broadcast quality with the path quality to the
/// advertising neighbour, per the canonical multiplicative formula
/// `(broadcastquality × pathquality + 128) / 256`, rounded and clamped to 0..=255.
///
/// `broadcast_quality` is the quality the neighbour advertised for the destination
/// (0..=255); `path_quality` is our path quality to that neighbour (0..=255). The
/// result is our derived route quality for the destination via that neighbour
/// (0..=255).
pub fn combine(broadcast_quality: u8, path_quality: u8) -> u8 {
    // (a × b + 128) / 256, integer. Max input 255 × 255 + 128 = 65153, well within
    // u32; the result is ≤ 254 so it always fits a byte, but clamp for total safety.
    let combined = ((broadcast_quality as u32 * path_quality as u32) + 128) / 256;
    combined.min(MAX as u32) as u8
}

#[cfg(test)]
mod tests {
    use super::*;

    // (broadcast_quality, path_quality) -> (a*b + 128) / 256, rounded
    #[test]
    fn combine_matches_the_canonical_formula() {
        assert_eq!(combine(255, 255), 254); // best × best
        assert_eq!(combine(0, 200), 0); // zero advertised → zero
        assert_eq!(combine(200, 0), 0); // zero path → zero
        assert_eq!(combine(192, 192), 144); // 36864 + 128 = 36992 / 256 = 144.5 → 144
        assert_eq!(combine(128, 128), 64); // 16384 + 128 = 16512 / 256 = 64.5 → 64
    }

    #[test]
    fn worked_example_two_hops_of_a_200_quality_link_is_about_156() {
        // Research doc: a 200-quality direct link, two hops = 200×200/256 ≈ 156.
        // (200*200 + 128) / 256 = 40128 / 256 = 156.75 → 156.
        assert_eq!(combine(200, 200), 156);
    }

    #[test]
    fn worked_example_three_hops_last_link_128_is_about_78() {
        // Research doc: three hops (last link 128) ≈ 78. The two-hop value (156)
        // combined with a 128 link: (156*128 + 128) / 256 = 19968 / 256 = 78.
        let two_hop = combine(200, 200); // 156
        assert_eq!(combine(two_hop, 128), 78);
    }

    #[test]
    fn combine_is_monotonic_quality_never_increases_with_an_extra_hop() {
        // A hop can only attenuate: for any path quality < 255 (i.e. not a perfect
        // link), combining reduces the advertised quality. This is the loop-safety
        // invariant — quality decreases per hop.
        for bq in 1u16..=255 {
            for pq in 1u16..255 {
                assert!(
                    combine(bq as u8, pq as u8) <= bq as u8,
                    "a hop at path quality {pq} must not raise advertised quality {bq}"
                );
            }
        }
    }

    #[test]
    fn combine_result_is_always_a_valid_byte() {
        for bq in 0u16..=255 {
            for pq in 0u16..=255 {
                let q = combine(bq as u8, pq as u8);
                assert!((MIN..=MAX).contains(&q));
            }
        }
    }
}
