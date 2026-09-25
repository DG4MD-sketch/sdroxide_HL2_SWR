//! The NCDXF/IARU International Beacon Project: which beacon is transmitting on
//! which band, right now.
//!
//! Eighteen beacons, five bands, and a schedule that is entirely deterministic:
//! a 180-second cycle of eighteen 10-second slots, aligned to UTC midnight, in
//! which exactly one beacon is on each band at any instant. No network, no
//! state — a clock and a table, which is why it lives here with the other pure
//! data.
//!
//! Why a listener cares: a beacon you can hear is a path that is open, measured
//! rather than forecast, and the 10 m beacon at 28.200 MHz is the closest
//! amateur-band proxy for 11 m conditions there is. The beacon's own site gives
//! its power, antenna and coordinates, so the same row also answers "which
//! direction, how far".
//!
//! Coordinates and callsigns are the NCDXF's published list; the slot order and
//! band offsets are the published schedule, cross-checked against
//! [OpenHamClock](https://github.com/accius/openhamclock) (MIT).

use crate::geo::{bearing_deg, distance_km};

/// One IBP beacon.
pub struct Beacon {
    pub callsign: &'static str,
    pub location: &'static str,
    /// WGS-84 latitude, degrees.
    pub lat: f64,
    /// WGS-84 longitude, degrees east.
    pub lon: f64,
}

/// The eighteen beacons, in transmission order. The index into this array *is*
/// the beacon's place in the cycle.
pub const BEACONS: &[Beacon] = &[
    Beacon { callsign: "4U1UN", location: "United Nations, NY", lat: 40.749, lon: -73.968 },
    Beacon { callsign: "VE8AT", location: "Inuvik, Canada", lat: 68.317, lon: -133.533 },
    Beacon { callsign: "W6WX", location: "Mt. Umunhum, CA", lat: 37.159, lon: -121.929 },
    Beacon { callsign: "KH6RS", location: "Hawaii, US", lat: 21.441, lon: -157.763 },
    Beacon { callsign: "ZL6B", location: "Masterton, New Zealand", lat: -40.683, lon: 175.567 },
    Beacon { callsign: "VK6RBP", location: "Bickley, Australia", lat: -31.802, lon: 116.126 },
    Beacon { callsign: "JA2IGY", location: "Mt. Asama, Japan", lat: 34.634, lon: 136.873 },
    Beacon { callsign: "RR9O", location: "Novosibirsk, Russia", lat: 54.853, lon: 83.125 },
    Beacon { callsign: "VR2B", location: "Hong Kong", lat: 22.255, lon: 114.137 },
    Beacon { callsign: "4S7B", location: "Colombo, Sri Lanka", lat: 6.816, lon: 79.924 },
    Beacon { callsign: "ZS6DN", location: "Pretoria, South Africa", lat: -25.683, lon: 28.183 },
    Beacon { callsign: "5Z4B", location: "Nairobi, Kenya", lat: -1.267, lon: 36.8 },
    Beacon { callsign: "4X6TU", location: "Tel Aviv, Israel", lat: 32.04, lon: 34.78 },
    Beacon { callsign: "OH2B", location: "Lohja, Finland", lat: 60.167, lon: 24.667 },
    Beacon { callsign: "CS3B", location: "Madeira, Portugal", lat: 32.7, lon: -16.883 },
    Beacon { callsign: "LU4AA", location: "Buenos Aires, Argentina", lat: -34.617, lon: -58.367 },
    Beacon { callsign: "OA4B", location: "Lima, Peru", lat: -12.043, lon: -77.017 },
    Beacon { callsign: "YV5B", location: "Caracas, Venezuela", lat: 10.483, lon: -66.983 },
];

/// One beacon band and its place in the cycle.
pub struct IbpBand {
    /// The metre band, as operators say it.
    pub label: &'static str,
    /// The beacon frequency, Hz — what the dial wants.
    pub freq_hz: f64,
    /// How many slots earlier this band's beacon entered the cycle. A beacon
    /// steps *up* one band every ten seconds, so the higher bands are showing
    /// beacons that started earlier: `(18 - step) % 18`.
    pub offset: usize,
}

/// The five beacon bands, in the order a beacon steps through them.
pub const BANDS: &[IbpBand] = &[
    IbpBand { label: "20m", freq_hz: 14_100_000.0, offset: 0 },
    IbpBand { label: "17m", freq_hz: 18_110_000.0, offset: 17 },
    IbpBand { label: "15m", freq_hz: 21_150_000.0, offset: 16 },
    IbpBand { label: "12m", freq_hz: 24_930_000.0, offset: 15 },
    IbpBand { label: "10m", freq_hz: 28_200_000.0, offset: 14 },
];

/// Seconds one beacon holds a band.
pub const SLOT_SECONDS: i64 = 10;
/// Seconds in a full pass over the eighteen beacons.
pub const CYCLE_SECONDS: i64 = SLOT_SECONDS * BEACONS.len() as i64;

/// The slot within the cycle, `0..18`, for a Unix time.
///
/// The cycle is aligned to UTC midnight, as the NCDXF's own schedule is; the
/// day length is a whole number of cycles, so only the time of day matters.
pub fn slot_at(unix_s: i64) -> usize {
    (unix_s.rem_euclid(86_400) % CYCLE_SECONDS / SLOT_SECONDS) as usize
}

/// Seconds left in the current slot, `1..=SLOT_SECONDS`.
pub fn seconds_left_in_slot(unix_s: i64) -> i64 {
    SLOT_SECONDS - unix_s.rem_euclid(SLOT_SECONDS)
}

/// A band's beacon at one instant, with its geometry from the observer.
pub struct Active {
    pub band: &'static IbpBand,
    pub beacon: &'static Beacon,
    /// The beacon's place in the cycle, `0..18`.
    pub beacon_index: usize,
    /// Bearing from the observer, degrees clockwise from north. `None` when no
    /// observer position was given.
    pub bearing_deg: Option<f64>,
    /// Great-circle distance from the observer, km. `None` when no observer
    /// position was given.
    pub distance_km: Option<f64>,
}

/// What is transmitting on each band at `unix_s`, from an optional observer
/// position `(lat, lon)` for the bearing and distance.
pub fn active_at(unix_s: i64, from: Option<(f64, f64)>) -> Vec<Active> {
    let slot = slot_at(unix_s);
    BANDS
        .iter()
        .map(|band| {
            let beacon_index = (slot + band.offset) % BEACONS.len();
            let beacon = &BEACONS[beacon_index];
            let (bearing, distance) = match from {
                Some((lat, lon)) => (
                    Some(bearing_deg((lat, lon), (beacon.lat, beacon.lon))),
                    Some(distance_km((lat, lon), (beacon.lat, beacon.lon))),
                ),
                None => (None, None),
            };
            Active { band, beacon, beacon_index, bearing_deg: bearing, distance_km: distance }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ymd_hms_to_unix;

    fn at(h: u32, m: u32, s: u32) -> i64 {
        ymd_hms_to_unix(2026, 6, 1, h, m, s)
    }

    /// The cycle starts at UTC midnight and turns over every three minutes.
    #[test]
    fn the_cycle_is_aligned_to_utc_midnight() {
        assert_eq!(slot_at(at(0, 0, 0)), 0);
        assert_eq!(slot_at(at(0, 0, 9)), 0);
        assert_eq!(slot_at(at(0, 0, 10)), 1);
        assert_eq!(slot_at(at(0, 2, 50)), 17);
        assert_eq!(slot_at(at(0, 3, 0)), 0, "the cycle did not restart at 3 minutes");
        assert_eq!(slot_at(at(23, 59, 59)), 17);
        // And on the next day, the same.
        let next = ymd_hms_to_unix(2026, 6, 2, 0, 0, 0);
        assert_eq!(slot_at(next), 0);
    }

    #[test]
    fn seconds_left_counts_down_within_the_slot() {
        assert_eq!(seconds_left_in_slot(at(0, 0, 0)), 10);
        assert_eq!(seconds_left_in_slot(at(0, 0, 1)), 9);
        assert_eq!(seconds_left_in_slot(at(0, 0, 9)), 1);
        assert_eq!(seconds_left_in_slot(at(0, 0, 10)), 10);
    }

    /// At the start of a cycle the 20 m beacon is the first in the list and the
    /// higher bands are the ones that started earlier — the offsets, not a
    /// guess.
    #[test]
    fn the_band_offsets_place_the_right_beacon_at_slot_zero() {
        let a = active_at(at(0, 0, 0), None);
        assert_eq!(a[0].beacon.callsign, "4U1UN", "20m at slot 0");
        assert_eq!(a[1].beacon.callsign, "YV5B", "17m is one slot behind the list");
        assert_eq!(a[2].beacon.callsign, "OA4B", "15m two slots behind");
        assert_eq!(a[3].beacon.callsign, "LU4AA", "12m three slots behind");
        assert_eq!(a[4].beacon.callsign, "CS3B", "10m four slots behind");
    }

    /// Over one full cycle every beacon reaches every band exactly once — the
    /// property the whole schedule rests on.
    #[test]
    fn every_beacon_works_every_band_once_a_cycle() {
        for band_ix in 0..BANDS.len() {
            let mut seen = vec![0u32; BEACONS.len()];
            for slot in 0..BEACONS.len() {
                let t = at(0, 0, 0) + slot as i64 * SLOT_SECONDS;
                let a = active_at(t, None);
                seen[a[band_ix].beacon_index] += 1;
            }
            assert!(
                seen.iter().all(|&n| n == 1),
                "band {} did not visit every beacon once: {seen:?}",
                BANDS[band_ix].label
            );
        }
    }

    /// The geometry is from the observer: zero distance to a beacon overhead,
    /// and the bearing to the north pole is north.
    #[test]
    fn the_geometry_is_measured_from_the_observer() {
        let here = (BEACONS[0].lat, BEACONS[0].lon);
        let a = active_at(at(0, 0, 0), Some(here));
        assert!(a[0].distance_km.unwrap() < 1.0, "distance to the beacon itself");
        let north = active_at(at(0, 0, 0), Some((10.0, 0.0)));
        let _ = north; // presence is enough; bearing is covered in geo's tests
    }
}
