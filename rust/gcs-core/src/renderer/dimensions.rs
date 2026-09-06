//! Dimensions inferred from projected solids. Callout layout supplies their label placement.
use crate::model::Sketch;
use crate::plane;
use crate::solid::ApproximationPolicy;
use crate::renderer::document::view_frame;

// -- the sheet as a report (§6.12) --------------------------------------------------------------

/// **The dimensions a machine can decide**, for one `dimensions(S) in P` statement.
///
/// Issue #48, item 10: half the edits to every part sheet were `at (t, r)` placements, moving
/// callouts off each other by trial and then rendering to see.  The human needs the picture; the
/// machine should produce it.
///
/// What this generates, and it is worth being exact about the boundary: **the part's overall
/// extents in the view, and the diameter of every round feature that view sees square on.**
/// Those are the dimensions that follow from the object — a machine can read them off the solid
/// and cannot get them wrong.  Which datum a stack is measured from, which fit is critical, what
/// is a reference and what controls: those are the *design*, and a machine that guessed would be
/// guessing.  A sheet says the rest as it always did, and this is what it no longer has to.
///
/// Nothing here is placed by hand: the figures come back with no placement, so
/// `callout::layout`'s own lane assignment stands them off each other — the engine that already
/// does this for every dimension a document states.
pub fn generated(sk: &Sketch, unit: f64) -> Vec<(usize, Dim)> {
    let mut out = Vec::new();
    for (i, d) in sk.derived.iter().enumerate() {
        if !d.dims {
            continue;
        }
        let (basis, pose) = view_frame(sk, d.plane.map(|p| p as usize));
        let Ok(solid) = sk.evaluated_solid(d.solid as usize, ApproximationPolicy::from_unit(unit)) else { continue };
        let local_basis = solid.local_basis(basis);
        // the box the part occupies, in the view's *own* axes, so an extent is measured the way
        // the view is turned and not the way the page is
        let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
        for p in solid.boundary().iter().flat_map(|p| &p.pts) {
            let v = local_basis.view_coords(*p);
            lo[0] = lo[0].min(v.0); lo[1] = lo[1].min(v.1);
            hi[0] = hi[0].max(v.0); hi[1] = hi[1].max(v.1);
        }
        if !lo[0].is_finite() {
            continue;
        }
        let page = |a: f64, b: f64| plane::on_page(pose.0, pose.1, pose.2, (a, b));
        // One across and one up, each **measured between the corners it bounds and stood off the
        // part** — a draughtsman does not draw an extent through the thing it measures, and the
        // engine's lane assignment then stacks the next one further out again.
        for (k, dir) in [(0usize, (1.0, 0.0)), (1, (0.0, 1.0))] {
            if hi[k] - lo[k] <= 0.0 {
                continue;
            }
            let (a, b) = if k == 0 {
                (page(lo[0], lo[1]), page(hi[0], lo[1]))
            } else {
                (page(lo[0], lo[1]), page(lo[0], hi[1]))
            };
            let along = plane::on_page(pose.0, pose.1, (0.0, 0.0), dir);
            out.push((i, Dim { a, b, dir: along, value: hi[k] - lo[k], round: false, clear: true }));
        }
        // and every round feature this view sees square on: a face that is one circle, whose
        // plane looks at the eye.  A hole is a size a printer needs and a machine can read
        let eye = basis.normal();
        for feature in solid.round_features() {
            if plane::dot(feature.normal, eye).abs() < 0.999_999 { continue; }
            let r = feature.radius;
            let (vu, vv) = local_basis.view_coords(feature.center.0);
            let (a, b) = (page(vu - r, vv), page(vu + r, vv));
            out.push((i, Dim { a, b, dir: plane::on_page(pose.0, pose.1, (0.0, 0.0), (1.0, 0.0)),
                               value: 2.0 * r, round: true, clear: false }));
        }
    }
    out
}

/// One generated dimension, in page coordinates: what it measures between, along which direction,
/// and what it comes to.
#[derive(Clone, Debug)]
pub struct Dim {
    pub a: (f64, f64),
    pub b: (f64, f64),
    pub dir: (f64, f64),
    pub value: f64,
    /// A diameter rather than a length — drawn with the `⌀` a draughtsman writes.
    pub round: bool,
    /// Stand it off the part rather than drawing it through: what an extent wants and a
    /// diameter, taken across its own circle, does not.
    pub clear: bool,
}
