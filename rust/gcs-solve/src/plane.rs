//! A plane in space, and the fold line two planes share (`docs/planes-plan.md`): the kernel's
//! (`cad_kernel::plane`), and the glyph a plane is drawn as.  `Project`'s kernel is the one
//! equation the projector rule (`fold_line`) comes to.
pub use cad_kernel::plane::*;

/// The length of a plane glyph's arms, in screen pixels — a datum mark, sized like a callout's
/// arrowhead rather than like the drawing.
pub const TICK_PX: f64 = 8.0;
/// The length of a plane glyph's `u` arm, in screen pixels.
pub const AXIS_PX: f64 = 32.0;

/// **The glyph a plane is drawn as** in its own coordinates: an arm from its origin along `u`
/// and a tick along `v`, saying which way each coordinate grows.  Two segments, as
/// `[(from, to); 2]`.  Laid out here for the reason a dimension callout is laid out in
/// `callout.rs`: it is geometry, so the core says what the figure *is* and every front end only
/// strokes what it is handed.  `unit` is the world length of one screen pixel, so the glyph is
/// screen-constant the way a callout's arrowhead is.
pub fn glyph(unit: f64) -> [((f64, f64), (f64, f64)); 2] {
    let o = (0.0, 0.0);
    [(o, (AXIS_PX * unit, 0.0)), (o, (0.0, TICK_PX * unit))]
}
