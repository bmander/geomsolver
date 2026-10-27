//! **The kinds of thing the workspace draws** (#678): objects, planes, axes, sketches, points,
//! fixed marks and dimensions, each shown or not everywhere at once — where the outline's hidden
//! (#435) takes out what one *row* made, a kind takes out every entity drawn as it.
//!
//! View state, as hidden is: never saved, exported, solved or undone, and set whole by the front
//! end on every drawing it is handed (`Sketch::set_kinds`).  A kind turned off is not drawn, not
//! lit and not picked — `Sketch::shown` says so for each entity, and everything that asks it
//! (`seen`, `overview::shown_objects`, the workspace's picks and bounds, the callouts) follows.

use super::{EntKind, EntRef, Sketch};

/// A set of the kinds the workspace draws, as bits — the FFI's `gcs_sketch_set_shown_kinds`
/// takes the same number.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Kinds(u32);

impl Kinds {
    /// The objects, whole: what `overview::objects` lists.
    pub const OBJECTS: Kinds = Kinds(1);
    /// The planes' panes and their frames.
    pub const PLANES: Kinds = Kinds(1 << 1);
    /// The axes in space.
    pub const AXES: Kinds = Kinds(1 << 2);
    /// What is drawn on planes and in space: lines, circles, arcs, curves, regions and traces —
    /// and their points, which go with them.
    pub const SKETCHES: Kinds = Kinds(1 << 3);
    /// The points.
    pub const POINTS: Kinds = Kinds(1 << 4);
    /// The mark a fixed point is drawn with: off, it is drawn as any other point.
    pub const FIXED: Kinds = Kinds(1 << 5);
    /// The dimension callouts.
    pub const DIMENSIONS: Kinds = Kinds(1 << 6);
    pub const NONE: Kinds = Kinds(0);
    pub const ALL: Kinds = Kinds((1 << 7) - 1);

    /// Each kind with the word a front end names it by, in the order a menu lists them.
    pub const NAMED: [(Kinds, &'static str); 7] = [
        (Kinds::OBJECTS, "objects"),
        (Kinds::PLANES, "planes"),
        (Kinds::AXES, "axes"),
        (Kinds::SKETCHES, "sketches"),
        (Kinds::POINTS, "points"),
        (Kinds::FIXED, "fixed"),
        (Kinds::DIMENSIONS, "dimensions"),
    ];

    pub const fn bits(self) -> u32 {
        self.0
    }

    /// The kinds these bits name; a bit naming none is let go.
    pub const fn from_bits(bits: u32) -> Kinds {
        Kinds(bits & Kinds::ALL.0)
    }

    /// Whether every kind of `k` is in this set (`NONE` always is).
    pub const fn contains(self, k: Kinds) -> bool {
        self.0 & k.0 == k.0
    }

    pub const fn with(self, k: Kinds) -> Kinds {
        Kinds(self.0 | k.0)
    }

    pub const fn without(self, k: Kinds) -> Kinds {
        Kinds(self.0 & !k.0)
    }

    /// **The kinds an entity is drawn as**: shown only while all of them are.  A point is a
    /// sketch's and a point; a plane, an axis and an object their own kind; every figure a
    /// sketch's; a motion is not drawn, and so is of none.
    pub fn of(e: EntRef) -> Kinds {
        match e.kind {
            EntKind::Point => Kinds::SKETCHES.with(Kinds::POINTS),
            EntKind::Frame => Kinds::PLANES,
            EntKind::Axis => Kinds::AXES,
            EntKind::Solid => Kinds::OBJECTS,
            EntKind::Line | EntKind::Circle | EntKind::Arc | EntKind::Spline | EntKind::Curve
            | EntKind::Face | EntKind::Surface | EntKind::Envelope | EntKind::Meshing
            | EntKind::Traced => Kinds::SKETCHES,
            EntKind::Motion => Kinds::NONE,
        }
    }
}

impl Default for Kinds {
    /// Everything shown: a drawing opens as it is drawn.
    fn default() -> Kinds {
        Kinds::ALL
    }
}

impl Sketch {
    /// **Show these kinds and no others** (#678), and each of `anyway` whatever its kind — a
    /// plane being drawn on, a row chosen from the outline — the whole of the view state, set at
    /// once, as `set_hidden` sets the outline's.
    pub fn set_kinds(&mut self, kinds: Kinds, anyway: impl IntoIterator<Item = EntRef>) {
        self.kinds = kinds;
        self.shown_anyway = anyway.into_iter().collect();
    }

    /// Whether the workspace passes anything by: some row hidden or some kind off.
    pub fn hides_any(&self) -> bool {
        !self.hidden.is_empty() || self.kinds != Kinds::ALL
    }

    /// Whether an entity's kind is shown (`Kinds::of`), or it is shown whatever its kind.
    pub fn kind_shown(&self, e: EntRef) -> bool {
        self.kinds.contains(Kinds::of(e)) || self.shown_anyway.contains(&e)
    }

    /// What is drawn and not shown — the outline's hidden and the kinds turned off — for a front
    /// end that strokes some of the drawing itself and must pass the same things by.
    pub fn unseen(&self) -> Vec<EntRef> {
        if !self.hides_any() {
            return Vec::new();
        }
        self.drawn().into_iter().filter(|&e| !self.shown(e)).collect()
    }
}
