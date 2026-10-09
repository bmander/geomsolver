//! Solved planar profiles, boundary tessellation and provenance.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::*;

// -- building a primitive out of a face --------------------------------------------------------

/// A face as the classifier wants it: its loop in the plane's own view coordinates, the plane it
/// is on, and where each vertex came from — so a swept side can be named by the edge it was
/// swept from and a tessellated arc's facets can be told from a corner.
#[derive(Clone, Debug)]
pub struct FacePoly {
    /// The loop, closed implicitly, in the plane's 2D view coordinates.
    pub pts: Vec<(f64, f64)>,
    /// Per vertex, the index into `names` of the edge that *leaves* it, and whether the step to
    /// the next vertex is a tessellation chord rather than a drawn straight edge.
    pub of: Vec<(usize, bool)>,
    /// The drawn edges, in traversal order, by the name the document calls them.
    pub names: Vec<String>,
    pub basis: Basis,
    /// Per edge in `names`, a spline's or a curve's own parameter where the walk enters and
    /// leaves it — what a loft pairs such an edge by (`loft::pair`); `None` for the rest.
    pub curved: Vec<Option<(EntRef, [f64; 2])>>,
}

impl FacePoly {
    /// A closed walk must bound a simple, nonzero region before it can be swept.
    pub(super) fn valid(&self) -> bool {
        let pts = &self.pts;
        let n = pts.len();
        if n < 3 || pts.iter().any(|p| !p.0.is_finite() || !p.1.is_finite()) { return false; }
        let scale = pts.iter().fold(0.0f64, |m, p| m.max((p.0 - pts[0].0).abs()).max((p.1 - pts[0].1).abs()));
        let tol = scale * 1e-12;
        let area_tol = tol * scale;
        if self.area().abs() <= area_tol { return false; }
        let cross = |a: (f64, f64), b: (f64, f64), c: (f64, f64)|
            (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0);
        for i in 0..n {
            let (a, b, c) = (pts[i], pts[(i + 1) % n], pts[(i + 2) % n]);
            if (b.0 - a.0).dhypot(b.1 - a.1) <= tol { return false; }
            // Collinear forward corners are valid; a retraced spike is not.
            if cross(a, b, c).abs() <= area_tol
                && (b.0 - a.0) * (c.0 - b.0) + (b.1 - a.1) * (c.1 - b.1) < 0.0
            { return false; }
            for j in i + 2..n {
                if (j + 1) % n == i { continue; }
                let (c, d) = (pts[j], pts[(j + 1) % n]);
                if a.0.min(b.0) > c.0.max(d.0) + tol || c.0.min(d.0) > a.0.max(b.0) + tol
                    || a.1.min(b.1) > c.1.max(d.1) + tol || c.1.min(d.1) > a.1.max(b.1) + tol
                { continue; }
                let same_side = |x: f64, y: f64| (x > area_tol && y > area_tol) || (x < -area_tol && y < -area_tol);
                if !same_side(cross(a, b, c), cross(a, b, d))
                    && !same_side(cross(c, d, a), cross(c, d, b))
                { return false; }
            }
        }
        true
    }

    pub fn area(&self) -> f64 {
        let n = self.pts.len();
        let mut a = 0.0;
        let Some(&(ox, oy)) = self.pts.first() else { return 0.0 };
        for i in 0..n {
            let (x0, y0) = (self.pts[i].0 - ox, self.pts[i].1 - oy);
            let (x1, y1) = (self.pts[(i + 1) % n].0 - ox, self.pts[(i + 1) % n].1 - oy);
            a += x0 * y1 - x1 * y0;
        }
        a / 2.0
    }

    /// The loop, turned counter-clockwise in view coordinates — the winding every sweep below
    /// assumes, so that a prism's `near` cap faces the viewer.
    pub(super) fn ccw(&self) -> FacePoly {
        if self.area() >= 0.0 {
            return self.clone();
        }
        let n = self.pts.len();
        let mut pts = self.pts.clone();
        pts.reverse();
        // the edge that *leaves* vertex i, reversed, is the one that used to arrive at it
        let mut of = Vec::with_capacity(n);
        for i in 0..n {
            of.push(self.of[(n - 1 - i + n - 1) % n]);
        }
        let curved = self.curved.iter().map(|c| c.map(|(e, [a, b])| (e, [b, a]))).collect();
        FacePoly { pts, of, names: self.names.clone(), basis: self.basis, curved }
    }

    pub fn lift(&self, i: usize) -> [f64; 3] {
        let (a, b) = self.pts[i];
        self.basis.lift(a, b)
    }
}

/// Read a face’s outer boundary off the solved drawing, walking its edges, arcs and circles
/// tessellated by the sagitta rule the sheet itself is drawn by.
///
/// `None` when the loop does not close or an edge is degenerate — the elaborator has already
/// refused those (E080), so this is the runtime's own guard rather than a diagnosis.
pub fn face_poly(sk: &Sketch, fi: usize, unit: f64) -> Option<FacePoly> {
    let f = sk.faces.get(fi)?;
    loop_poly(sk, &f.edges, &f.edge_names, f.plane().ok()?, unit)
}

pub(super) fn loop_poly(sk: &Sketch, edges: &[EntRef], edge_names: &[String], plane: Option<u32>, unit: f64) -> Option<FacePoly> {
    if edges.is_empty() {
        return None;
    }
    let basis = match plane {
        Some(p) => { sk.planes.get(p as usize)?; sk.basis(p as usize) }
        None => Basis::page(),
    };

    let names = edge_names.to_vec();
    let mut pts: Vec<(f64, f64)> = Vec::new();
    let mut of: Vec<(usize, bool)> = Vec::new();

    // a circle standing alone is the whole loop
    if edges.len() == 1 && edges[0].kind == EntKind::Circle {
        let c = &sk.circles[edges[0].i()];
        let ctr = sk.point_xy(c.center as usize);
        let r = sk.params[c.radius as usize].value;
        if r.abs() <= 0.0 {
            return None;
        }
        let ring = tessellate_arc(ctr, r.abs(), 0.0, std::f64::consts::TAU, unit);
        for p in ring.iter().take(ring.len() - 1) {
            pts.push(*p);
            of.push((0, true));
        }
        let curved = vec![None; names.len()];
        let poly = tidy_poly(FacePoly { pts, of, names, basis, curved });
        return poly.valid().then(|| poly.ccw());
    }

    // a closed curve standing alone is the whole loop, walked within the sheet's sagitta
    if edges.len() == 1 && edges[0].kind == EntKind::Curve && sk.curves[edges[0].i()].trim.is_none() {
        let i = edges[0].i();
        if !sk.curve_closed(i) {
            return None;
        }
        let ring = sk.curve_polyline_within(i, unit);
        for &(_, p) in ring.iter().take(ring.len().saturating_sub(1)) {
            pts.push(p);
            of.push((0, true));
        }
        let (a, b) = sk.curve_domain(i);
        let curved = vec![Some((edges[0], [a, b]))];
        let poly = tidy_poly(FacePoly { pts, of, names, basis, curved });
        return poly.valid().then(|| poly.ccw());
    }

    // otherwise: every edge in traversal order, each starting where the last one ended
    let mut at: Option<u32> = None;
    let mut curved = vec![None; names.len()];
    for (i, e) in edges.iter().enumerate() {
        let (a, b) = crate::model::edge_ends(sk, *e)?;
        // which end this edge is entered by: the one the walk is standing on
        let (from, to) = match at {
            None => {
                // the first edge is entered by whichever end the *last* edge shares
                let (la, lb) = crate::model::edge_ends(sk, *edges.last()?)?;
                if a == la || a == lb {
                    (a, b)
                } else {
                    (b, a)
                }
            }
            Some(p) if p == a => (a, b),
            Some(p) if p == b => (b, a),
            Some(_) => return None,
        };
        walk_edge(sk, *e, from, to, i, unit, &mut pts, &mut of)?;
        if let Some(c) = curved.get_mut(i) { *c = curved_span(sk, *e, from); }
        at = Some(to);
    }
    if pts.len() < 3 {
        return None;
    }
    let poly = tidy_poly(FacePoly { pts, of, names, basis, curved });
    poly.valid().then(|| poly.ccw())
}

/// A spline's or a curve's parameter where a walk entering it at `from` enters and leaves it.
fn curved_span(sk: &Sketch, e: EntRef, from: u32) -> Option<(EntRef, [f64; 2])> {
    let (a, b) = match e.kind {
        EntKind::Spline => (crate::curve::domain(sk, e.i()), sk.splines[e.i()].ctrl[0] == from),
        EntKind::Curve => (sk.curve_domain(e.i()), sk.curves[e.i()].trim?.from == from),
        _ => return None,
    };
    let ((lo, hi), forward) = (a, b);
    Some((e, if forward { [lo, hi] } else { [hi, lo] }))
}

/// Where a spline or a curve is at its parameter `t`, on the page.
pub(super) fn curved_point(sk: &Sketch, e: EntRef, t: f64) -> (f64, f64) {
    match e.kind {
        EntKind::Spline => crate::curve::point_at(sk, e.i(), t),
        _ => sk.curve_point(e.i(), t),
    }
}

/// A zero-length side is no side: two coincident vertices would give a facet with no normal and
/// a seam with no direction.  Coordinates are left exactly as the solve found them.
fn tidy_poly(mut f: FacePoly) -> FacePoly {
    let origin = f.pts[0];
    let scale = f.pts.iter().fold(0.0f64, |m, p| m.max((p.0 - origin.0).abs()).max((p.1 - origin.1).abs()));
    let g = scale * SNAP;
    let mut pts = Vec::with_capacity(f.pts.len());
    let mut of = Vec::with_capacity(f.of.len());
    for i in 0..f.pts.len() {
        let j = (i + 1) % f.pts.len();
        let d = (f.pts[j].0 - f.pts[i].0).dhypot(f.pts[j].1 - f.pts[i].1);
        if d > g {
            pts.push(f.pts[i]);
            of.push(f.of[i]);
        }
    }
    if pts.len() >= 3 {
        f.pts = pts;
        f.of = of;
    }
    f
}

fn walk_edge(
    sk: &Sketch,
    e: EntRef,
    from: u32,
    _to: u32,
    idx: usize,
    unit: f64,
    pts: &mut Vec<(f64, f64)>,
    of: &mut Vec<(usize, bool)>,
) -> Option<()> {
    match e.kind {
        EntKind::Line => {
            pts.push(sk.point_xy(from as usize));
            of.push((idx, false));
            Some(())
        }
        EntKind::Arc => {
            let a = &sk.arcs[e.i()];
            let c = sk.point_xy(a.center as usize);
            let r = sk.params[a.radius as usize].value.abs();
            let ang = |p: u32| {
                let q = sk.point_xy(p as usize);
                (q.1 - c.1).datan2(q.0 - c.0)
            };
            let (a0, a1) = (ang(a.start), ang(a.end));
            // **How far an arc goes is the arc's own fact; which way is the walk's.**  An arc
            // runs CCW from start to end, and entered at `end` it is walked the other way — the
            // *same* stretch of the circle backwards, never the complement of it.  Normalising
            // `a0 - a1` instead gave `TAU - extent` there, so a face that happened to enter an
            // arc by its end came out as the rest of the circle: the V-twin plate's plenum, a
            // channel between two concentric arcs, closed as a bowtie of twelve times the area
            // and meshed with seventy-six unpaired edges.
            let ccw = from == a.start;
            let mut extent = a1 - a0;
            while extent <= 0.0 {
                extent += std::f64::consts::TAU;
            }
            let start = if ccw { a0 } else { a1 };
            let step = if ccw { extent } else { -extent };
            let ring = tessellate_arc(c, r, start, step, unit);
            for p in ring.iter().take(ring.len() - 1) {
                pts.push(*p);
                of.push((idx, true));
            }
            // the first vertex of an arc is a real corner, not a chord joint
            if let Some(last) = of.len().checked_sub(ring.len() - 1) {
                of[last].1 = ring.len() > 2;
            }
            Some(())
        }
        EntKind::Curve => {
            // a face's stretch of a curve, from `from` to `to` in its own sense, chords within
            // the sheet's flatness
            let t = sk.curves[e.i()].trim?;
            let mut ring: Vec<(f64, f64)> = sk.curve_polyline_within(e.i(), crate::curve::flatness(unit))
                .into_iter().map(|(_, p)| p).collect();
            if t.from != from { ring.reverse(); }
            for p in ring.iter().take(ring.len() - 1) {
                pts.push(*p);
                of.push((idx, true));
            }
            if let Some(last) = of.len().checked_sub(ring.len() - 1) {
                of[last].1 = ring.len() > 2;
            }
            Some(())
        }
        EntKind::Spline => {
            // walked from whichever end the loop arrives at; every chord joint a curve point
            let mut ring = crate::curve::tessellate(sk, e.i(), unit);
            if sk.splines[e.i()].ctrl[0] != from { ring.reverse(); }
            for p in ring.iter().take(ring.len() - 1) {
                pts.push(*p);
                of.push((idx, true));
            }
            if let Some(last) = of.len().checked_sub(ring.len() - 1) {
                of[last].1 = ring.len() > 2;
            }
            Some(())
        }
        _ => None,
    }
}

/// A circle or arc as chords no further from it than the sheet's own flatness — `overview::round`'s
/// rule, said here in page coordinates so the solid and the drawing round a corner alike.
pub(super) fn tessellate_arc(c: (f64, f64), r: f64, from: f64, sweep: f64, unit: f64) -> Vec<(f64, f64)> {
    let tol = crate::curve::flatness(unit);
    // Bound angular error too: below the absolute flatness a circle still needs a region,
    // with the same relative area accuracy as a larger circular profile.
    let step = (if r > tol { 2.0 * (1.0 - tol / r).dacos() } else { std::f64::consts::TAU })
        .min(std::f64::consts::TAU / 64.0);
    let n = ((sweep.abs() / step).ceil() as usize).clamp(2, 4096);
    (0..=n)
        .map(|k| {
            let a = from + sweep * k as f64 / n as f64;
            (c.0 + r * a.dcos(), c.1 + r * a.dsin())
        })
        .collect()
}
