//! Distances, orientation, picking and continuation waypoints.

#[allow(unused_imports)]
use crate::fmath::Det;
use super::*;

/// Signed perpendicular offset from the *infinite* line through `line`, positive to the left of
/// its direction.  A degenerate line has no side; it gives infinity rather than a silent zero.
pub fn signed_point_to_line(sk: &Sketch, px: f64, py: f64, line: usize) -> f64 {
    let l = &sk.lines[line];
    let (ax, ay) = sk.point_xy(l.p1 as usize);
    let (dx, dy) = sk.line_dir(line);
    let length = dx.dhypot(dy);
    if length == 0.0 {
        return f64::INFINITY;
    }
    (dx * (py - ay) - dy * (px - ax)) / length
}

/// How far (px, py) is from an entity — the one implementation, so the pair dispatch below and
/// the sweep along a curve cannot drift apart.  A point against a curve is exact: the projection
/// is a Newton solve the core does anyway, and this is the number a reader checks the drawing
/// against.
fn point_to(sk: &Sketch, px: f64, py: f64, e: EntRef) -> f64 {
    match e.kind {
        // never picked and never dimensioned: what a 2D statement may name is the drawing, and
        // a face or a solid is evaluated after the drawing is solved (§6.9)
        EntKind::Face | EntKind::Solid | EntKind::Surface | EntKind::Motion | EntKind::Envelope | EntKind::Patch | EntKind::Seam | EntKind::Vertex | EntKind::Edge
        // a cone or a cylinder is on no sheet: its axis is what is picked of it
        | EntKind::Cone | EntKind::Cylinder | EntKind::Axis => f64::MAX,
        // a curve has no idealised form a dimension could mean beyond the curve itself, so this
        // measurement and `point_to_drawn`'s are the same one
        EntKind::Curve => polyline_distance(&sk.curve_polyline(e.i()), px, py),
        EntKind::Point => {
            let (x, y) = sk.point_xy(e.i());
            (px - x).dhypot(py - y)
        }
        EntKind::Line => point_to_line(sk, px, py, e.i()),
        EntKind::Circle | EntKind::Arc => {
            let (cx, cy) = sk.point_xy(sk.round_center(e));
            ((px - cx).dhypot(py - cy) - sk.radius_value(e).abs()).abs()
        }
        EntKind::Spline => crate::curve::distance_to(sk, e.i(), px, py),
        // a datum is not a figure: the place it stands at is its origin
        EntKind::Plane => px.dhypot(py),
    }
}

fn point_to_line(sk: &Sketch, px: f64, py: f64, line: usize) -> f64 {
    let (dx, dy) = sk.line_dir(line);
    if dx == 0.0 && dy == 0.0 {
        let l = &sk.lines[line];
        let (ax, ay) = sk.point_xy(l.p1 as usize);
        return (px - ax).dhypot(py - ay);
    }
    signed_point_to_line(sk, px, py, line).abs()
}

/// How far (px, py) is from what is *drawn* of `e`: the segment a line is drawn as, the sweep an
/// arc is drawn as, the curve itself.  `point_to` measures the entity a *dimension* means — a
/// line is infinite, an arc is the whole circle it lies on — which is not what a pointer hits.
pub fn point_to_drawn(sk: &Sketch, px: f64, py: f64, e: EntRef) -> f64 {
    match e.kind {
        EntKind::Face | EntKind::Solid | EntKind::Surface | EntKind::Motion | EntKind::Envelope | EntKind::Patch | EntKind::Seam | EntKind::Vertex | EntKind::Edge
        // a cone or a cylinder is on no sheet: its axis is what is picked of it
        | EntKind::Cone | EntKind::Cylinder | EntKind::Axis => f64::MAX,
        EntKind::Curve => polyline_distance(&sk.curve_polyline(e.i()), px, py),
        EntKind::Line => {
            let l = &sk.lines[e.i()];
            let (a, b) = (sk.point_xy(l.p1 as usize), sk.point_xy(l.p2 as usize));
            seg_distance((px, py), a, b)
        }
        EntKind::Arc => {
            let (cx, cy) = sk.point_xy(sk.round_center(e));
            let r = sk.radius_value(e).abs();
            let (a0, a1) = sk.arc_angles(e.i());
            let mut th = (py - cy).datan2(px - cx);
            if th < a0 {
                th += 2.0 * std::f64::consts::PI;     // arc_angles keeps a1 within one turn of a0
            }
            if th <= a1 {
                return ((px - cx).dhypot(py - cy) - r).abs();
            }
            // off the ends of the sweep: the nearer end of what was drawn, not the phantom
            // remainder of the circle
            let at = |t: f64| (cx + r * t.dcos(), cy + r * t.dsin());
            let (sx, sy) = at(a0);
            let (ex, ey) = at(a1);
            (px - sx).dhypot(py - sy).min((px - ex).dhypot(py - ey))
        }
        // the kinds whose drawn figure *is* the entity: a point, a whole ring or rim, and a
        // curve that `curve::distance_to` already keeps between its own knots
        EntKind::Point | EntKind::Circle | EntKind::Spline => {
            point_to(sk, px, py, e)
        }
        // a plane is drawn as its glyph at its own origin, which is where it is taken hold of
        EntKind::Plane => px.dhypot(py),
    }
}

/// Distance from a point to a segment — the drawn figure of a line, and the flat side of every
/// callout box, so both ask here rather than each keeping the projection formula.
pub fn seg_distance(p: (f64, f64), a: (f64, f64), b: (f64, f64)) -> f64 {
    let (vx, vy) = (b.0 - a.0, b.1 - a.1);
    let l2 = vx * vx + vy * vy;
    if l2 <= 0.0 {
        return (p.0 - a.0).dhypot(p.1 - a.1);
    }
    let t = (((p.0 - a.0) * vx + (p.1 - a.1) * vy) / l2).clamp(0.0, 1.0);
    (p.0 - (a.0 + t * vx)).dhypot(p.1 - (a.1 + t * vy))
}

/// What a click at (x, y) picks: the nearest entity whose drawn figure comes within `tol`, or
/// nothing.  A point within reach wins outright, however much nearer an edge passes — a point is
/// what most of a sketcher's verbs are about, and it is the smaller target of the two.  The
/// tolerance is a world length, so a front end scales it by what one screen pixel is worth and
/// keeps no geometry of its own.
pub fn pick(sk: &Sketch, x: f64, y: f64, tol: f64) -> Option<EntRef> {
    pick_by(sk, sk.nearest_point(x, y), tol, |e| point_to_drawn(sk, x, y, e))
}

/// `pick`'s rule over any reading of where things are: the nearest point and how far it is, and
/// how far each figure is — on the page, or as an eye sees the workspace
/// (`overview::workspace::pick`).
pub fn pick_by(
    sk: &Sketch,
    nearest: (Option<usize>, f64),
    tol: f64,
    distance: impl Fn(EntRef) -> f64,
) -> Option<EntRef> {
    if let (Some(i), d) = nearest {
        if d <= tol {
            return Some(EntRef::point(i));
        }
    }
    let mut best: Option<(EntRef, f64)> = None;
    // what is drawn, curves included: a pick measures the figure on the sheet, and a curve
    // written in the language is one (`drawn`, where `primitives` stops short of it)
    for e in sk.drawn() {
        if e.kind == EntKind::Point {
            continue;
        }
        let d = distance(e);
        if d <= tol && best.map_or(true, |(_, bd)| d < bd) {
            best = Some((e, d));
        }
    }
    best.map(|(e, _)| e)
}

fn measure_order(k: EntKind) -> u8 {
    match k {
        EntKind::Point => 0,
        EntKind::Line => 1,
        EntKind::Circle | EntKind::Arc => 2,
        EntKind::Spline => 3,
        EntKind::Curve => 4,
        // never measured against anything: a face and a solid are not on the sheet
        EntKind::Face | EntKind::Solid | EntKind::Surface | EntKind::Motion | EntKind::Envelope | EntKind::Patch | EntKind::Seam | EntKind::Vertex | EntKind::Edge
        | EntKind::Cone | EntKind::Cylinder | EntKind::Axis => 5,
        // last, so any pair with a datum in it puts the datum second and one arm catches it
        EntKind::Plane => 6,
    }
}

/// The nearest a polyline comes to a point.
pub(crate) fn polyline_distance(pts: &[(f64, f64)], px: f64, py: f64) -> f64 {
    let mut best = f64::MAX;
    for w in pts.windows(2) {
        best = best.min(seg_distance((px, py), w[0], w[1]));
    }
    if pts.len() == 1 {
        best = best.min((px - pts[0].0).dhypot(py - pts[0].1));
    }
    best
}

/// Signed CCW angle from line `a` to line `b`, in radians — what an `Angle` constraint's value
/// means, and what a dimension dialog should offer as the current value.
pub fn angle_between(sk: &Sketch, a: EntRef, b: EntRef) -> f64 {
    let (d1x, d1y) = sk.line_dir(a.i());
    let (d2x, d2y) = sk.line_dir(b.i());
    (d1x * d2y - d1y * d2x).datan2(d1x * d2x + d1y * d2y)
}

/// The point at distance `r` from (cx, cy) in the direction of (tx, ty).  The centre–start–end
/// arc construction: the third click gives a direction, and the radius comes from the second.
/// `None` when the target is the centre, which names no direction.
pub fn on_radius(cx: f64, cy: f64, tx: f64, ty: f64, r: f64) -> Option<(f64, f64)> {
    let (dx, dy) = (tx - cx, ty - cy);
    let l = dx.dhypot(dy);
    if l <= 1e-12 {
        return None;
    }
    Some((cx + r * dx / l, cy + r * dy / l))
}

/// Points along whatever is drawn of a kind that has no closed form to measure against — a
/// curve's tessellation, an ellipse's rim.  One reader, so the arm in `distance_between` that
/// sweeps is one arm and does not have to know which family it is sweeping.
fn swept(sk: &Sketch, e: EntRef) -> Vec<(f64, f64)> {
    match e.kind {
        EntKind::Spline => crate::curve::sample(sk, e.i(), 64),
        // a datum is not a figure: the one place it stands at
        EntKind::Plane => vec![(0.0, 0.0)],
        _ => Vec::new(),
    }
}

/// Shortest distance between two entities, as a sketcher measures it.  Lines are treated as
/// infinite; arcs are measured as the whole circle they lie on.
pub fn distance_between(sk: &Sketch, first: EntRef, second: EntRef) -> f64 {
    let (a, b) = if measure_order(first.kind) > measure_order(second.kind) {
        (second, first)
    } else {
        (first, second)
    };
    match a.kind {
        EntKind::Point => {
            let (ax, ay) = sk.point_xy(a.i());
            point_to(sk, ax, ay, b)
        }
        // A curve and an ellipse have no closed form against any of the others, so both are
        // measured by sweeping what is drawn — close enough to measure by, and honestly the best
        // a sampled answer can be.  A frame joins them with a single sample, its origin: not for
        // want of a closed form but because a datum is measured where it stands.  All three sort
        // after everything they could be paired with, so the swept one is always `b`, and the
        // exact point case has already short-circuited above.
        //
        // This sits *above* the arms that reach for a centre and a radius, which is the whole
        // reason it is one arm and not one per family: a sampled kind that fell through to them
        // would ask a curve for a centre it does not have.
        _ if matches!(
            b.kind,
            EntKind::Spline | EntKind::Plane
        ) => swept(sk, b)
            .into_iter()
            .map(|(x, y)| point_to(sk, x, y, a))
            .fold(f64::INFINITY, f64::min),
        EntKind::Line => match b.kind {
            EntKind::Line => {
                let d1 = sk.line_dir(a.i());
                let d2 = sk.line_dir(b.i());
                let cross = d1.0 * d2.1 - d1.1 * d2.0;
                if cross.abs() > 1e-9 * d1.0.dhypot(d1.1) * d2.0.dhypot(d2.1) {
                    return 0.0; // they meet somewhere
                }
                let l = &sk.lines[b.i()];
                let (px, py) = sk.point_xy(l.p1 as usize);
                point_to_line(sk, px, py, a.i())
            }
            _ => {
                let (cx, cy) = sk.point_xy(sk.round_center(b));
                (point_to_line(sk, cx, cy, a.i()) - sk.radius_value(b).abs()).max(0.0)
            }
        },
        _ => {
            // outside each other, or one inside the other; overlapping rings give 0
            let (ax, ay) = sk.point_xy(sk.round_center(a));
            let (bx, by) = sk.point_xy(sk.round_center(b));
            let gap = (ax - bx).dhypot(ay - by);
            let (r1, r2) = (sk.radius_value(a).abs(), sk.radius_value(b).abs());
            (gap - r1 - r2).max((r1 - r2).abs() - gap).max(0.0)
        }
    }
}

/// Twice the signed area of (a, b, c) — the order-type invariant the drag guards.
pub fn orientation(sk: &Sketch, a: usize, b: usize, c: usize) -> f64 {
    let (ax, ay) = sk.point_xy(a);
    let (bx, by) = sk.point_xy(b);
    let (cx, cy) = sk.point_xy(c);
    orientation_xy(ax, ay, bx, by, cx, cy)
}

/// The same from bare coordinates — one formula, so this reading and a trace predicate's
/// (`locus::holds`) cannot fork on the sign convention.
pub fn orientation_xy(ax: f64, ay: f64, bx: f64, by: f64, cx: f64, cy: f64) -> f64 {
    (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
}

/// Continuation path from (x0, y0) to (x1, y1): waypoints no farther apart than `max_step`, so a
/// solution tracks its branch instead of teleporting across it.  Always at least one point.
pub fn increments(
    x0: f64,
    y0: f64,
    x1: f64,
    y1: f64,
    max_step: f64,
) -> Vec<(f64, f64)> {
    let n = (((x1 - x0).dhypot(y1 - y0) / max_step).ceil() as i64).max(1);
    (1..=n)
        .map(|i| {
            let t = i as f64 / n as f64;
            (x0 + (x1 - x0) * t, y0 + (y1 - y0) * t)
        })
        .collect()
}
