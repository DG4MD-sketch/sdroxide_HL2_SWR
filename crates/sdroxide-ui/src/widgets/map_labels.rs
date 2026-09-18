//! Names beside the symbols on a map — the AIS chart's vessel names, the ADS-B
//! picture's data blocks — placed so that they never pile up.
//!
//! Both maps used to have an all-or-nothing rule: every name while few enough
//! targets were in view, none at all above a fixed count. The count in view
//! changes as the map pans and zooms, so names came and went with where the map
//! was rather than with the traffic (issue #408). Here each name is placed
//! greedily instead, the way [`super::worldmap`] places city names: the ones
//! that must show first, then the rest by rank, each only if it misses every
//! name already placed.

use eframe::egui::{Color32, FontId, Painter, Pos2, Rect, pos2, vec2};

/// From one line of a name to the next, in points.
const PITCH: f32 = 10.0;

/// One target's name, waiting to be placed.
pub struct MapLabel {
    /// The symbol's centre.
    pub at: Pos2,
    /// The symbol's radius. The name starts just beyond it.
    pub r: f32,
    /// Where the tick joining name and symbol starts, as a fraction of `r`
    /// along the diagonal toward the name.
    pub tick_from: f32,
    /// The tick's colour.
    pub tick: Color32,
    /// The lines, top to bottom, each in its own colour.
    pub lines: Vec<(String, Color32)>,
    /// Drawn whatever it lands on: the target the operator picked or is
    /// pointing at, or one in distress. Placed first, so a crowded map culls
    /// *around* it rather than through it.
    pub must: bool,
    /// How hard the name fights for room: lower wins.
    pub rank: u8,
    /// Settles a tie in rank — the MMSI, the ICAO address. The trackers hand
    /// targets over in hash map order, so breaking ties by position would let
    /// which of two neighbours keeps its name change whenever the map is
    /// rehashed.
    pub key: u32,
}

/// A name reduced to what placing it needs: where it would sit, and what
/// decides whether it gets to.
struct Slot {
    area: Rect,
    must: bool,
    rank: u8,
    key: u32,
}

/// Which of `slots` to draw, by index, in draw order.
///
/// The ones that must appear first, whatever they land on; then the rest in
/// rank order, each only if its box stays inside `bounds` and misses every one
/// already placed.
fn plan(slots: &[Slot], bounds: Rect) -> Vec<usize> {
    let mut order: Vec<usize> = (0..slots.len()).collect();
    order.sort_by_key(|&i| (!slots[i].must, slots[i].rank, slots[i].key));
    let mut placed: Vec<Rect> = Vec::new();
    let mut out: Vec<usize> = Vec::new();
    for i in order {
        let slot = &slots[i];
        if !slot.must
            && (!bounds.contains_rect(slot.area) || placed.iter().any(|q| q.intersects(slot.area)))
        {
            continue;
        }
        placed.push(slot.area);
        out.push(i);
    }
    out
}

/// Lay out every name in `labels`, place the ones that fit inside `bounds`,
/// and draw them — each up and to the right of its symbol, with a tick joining
/// the two so a crowded picture still says which name is whose.
pub fn draw(p: &Painter, bounds: Rect, font: &FontId, labels: Vec<MapLabel>) {
    let mut slots = Vec::with_capacity(labels.len());
    let mut laid = Vec::with_capacity(labels.len());
    for label in labels {
        let galleys: Vec<_> = label
            .lines
            .into_iter()
            .map(|(text, colour)| (p.layout_no_wrap(text, font.clone(), colour), colour))
            .collect();
        if galleys.is_empty() {
            continue;
        }
        let n = galleys.len();
        let line_h = galleys.iter().map(|(g, _)| g.size().y).fold(0.0, f32::max);
        let w = galleys.iter().map(|(g, _)| g.size().x).fold(0.0, f32::max);
        let size = vec2(w, (n - 1) as f32 * PITCH + line_h);
        let anchor = label.at + vec2(label.r + 3.0, -(label.r + 2.0));
        // Each line centred half a line above the tick's end, so the last one
        // sits on the tick rather than being cut through by it.
        let top = anchor.y - (n - 1) as f32 * PITCH - PITCH / 2.0 - line_h / 2.0;
        let block = Rect::from_min_size(pos2(anchor.x + 2.0, top), size);
        slots.push(Slot {
            area: block.expand(1.0),
            must: label.must,
            rank: label.rank,
            key: label.key,
        });
        let t = label.r * label.tick_from;
        laid.push((block, (label.at + vec2(t, -t), anchor), label.tick, galleys));
    }
    for i in plan(&slots, bounds) {
        let (block, tick, tick_colour, galleys) = &laid[i];
        p.line_segment([tick.0, tick.1], (1.0, *tick_colour));
        for (k, (galley, colour)) in galleys.iter().enumerate() {
            p.galley(block.min + vec2(0.0, k as f32 * PITCH), galley.clone(), *colour);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bounds() -> Rect {
        Rect::from_min_size(pos2(0.0, 0.0), vec2(200.0, 100.0))
    }

    fn slot(x: f32, y: f32, must: bool, rank: u8) -> Slot {
        Slot { area: Rect::from_min_size(pos2(x, y), vec2(20.0, 8.0)), must, rank, key: 0 }
    }

    /// A map too busy for every name keeps the ones that say the most, by
    /// rank — the opposite of the rule this replaced, which dropped them all at
    /// once once a few dozen targets were in view (issue #408).
    #[test]
    fn a_crowded_map_keeps_the_names_that_matter() {
        // Two names claiming the same room: the better-ranked one wins.
        let slots = vec![slot(0.0, 0.0, false, 3), slot(0.0, 0.0, false, 1)];
        assert_eq!(plan(&slots, bounds()), vec![1]);
        // Names that do not collide all fit, whatever their rank.
        let slots = vec![slot(0.0, 0.0, false, 3), slot(40.0, 0.0, false, 1)];
        let placed = plan(&slots, bounds());
        assert!(placed.contains(&0) && placed.contains(&1), "both should fit: {placed:?}");
    }

    /// Two names of the same rank after the same room: the lower key wins,
    /// whichever order the tracker happened to list them in.
    #[test]
    fn a_tie_in_rank_goes_to_the_lower_key_not_the_list_order() {
        let a = || Slot { key: 244_000_002, ..slot(0.0, 0.0, false, 1) };
        let b = || Slot { key: 244_000_001, ..slot(0.0, 0.0, false, 1) };
        assert_eq!(plan(&[a(), b()], bounds()), vec![1]);
        assert_eq!(plan(&[b(), a()], bounds()), vec![0]);
    }

    /// The name the operator picked or is pointing at is placed first and drawn
    /// whatever else is around it, so a crowded map culls *around* it rather
    /// than through it.
    #[test]
    fn the_name_the_operator_is_looking_at_always_shows() {
        // A must-label takes the room from a better-ranked one...
        let slots = vec![slot(10.0, 10.0, true, 3), slot(10.0, 10.0, false, 0)];
        assert_eq!(plan(&slots, bounds()), vec![0]);
        // ...and is drawn even where it runs off the map's edge, where the
        // painter clips it.
        let slots = vec![slot(190.0, 95.0, true, 3)];
        assert_eq!(plan(&slots, bounds()), vec![0]);
    }

    /// A name with no room on the map does not show, unless it must.
    #[test]
    fn a_name_off_the_map_is_dropped_unless_it_must_show() {
        assert!(plan(&[slot(190.0, 95.0, false, 0)], bounds()).is_empty());
        assert!(plan(&[slot(-25.0, 0.0, false, 0)], bounds()).is_empty());
    }
}
