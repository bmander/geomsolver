//! Solved planar and world geometry, drawing bounds and point access.

use super::*;

pub type Box2 = (f64, f64, f64, f64); // (xmin, ymin, xmax, ymax)

/// Grow a box to take in a point.
pub fn grow(b: &mut Box2, p: (f64, f64)) {
    b.0 = b.0.min(p.0);
    b.1 = b.1.min(p.1);
    b.2 = b.2.max(p.0);
    b.3 = b.3.max(p.1);
}

impl Sketch {
    /// Which plane a point is an image on, if it says.
    pub fn plane_of(&self, point: usize) -> Option<usize> {
        self.points[point].plane.map(|p| p as usize)
    }

    /// Put a point on a plane, or take it off (`None`).  A membership and not a constraint:
    /// nothing moves, and only `Project` reads it.
    pub fn set_plane(&mut self, point: usize, plane: Option<usize>) {
        debug_assert!(plane.map_or(true, |p| p < self.planes.len()));
        self.points[point].plane = plane.map(|p| p as u32);
    }

    /// The datum half of a plane — the origin, the toward point and the rotor.
    pub fn frame_of(&self, e: EntRef) -> &FrameE {
        match e.kind {
            EntKind::Plane => &self.planes[e.i()].frame,
            other => panic!("a {} has no rotor", other.as_str()),
        }
    }

    // -- accessors ----------------------------------------------------------

    pub fn point_xy(&self, i: usize) -> (f64, f64) {
        let p = &self.points[i];
        (self.params[p.x as usize].value, self.params[p.y as usize].value)
    }

    pub fn line_dir(&self, i: usize) -> (f64, f64) {
        let l = &self.lines[i];
        let (ax, ay) = self.point_xy(l.p1 as usize);
        let (bx, by) = self.point_xy(l.p2 as usize);
        (bx - ax, by - ay)
    }

    pub fn line_length(&self, i: usize) -> f64 {
        let (dx, dy) = self.line_dir(i);
        dx.hypot(dy)
    }

    /// Centre point index of a circle or an arc.
    pub fn round_center(&self, e: EntRef) -> usize {
        match e.kind {
            EntKind::Circle => self.circles[e.i()].center as usize,
            EntKind::Arc => self.arcs[e.i()].center as usize,
            EntKind::Sphere => self.spheres[e.i()].center as usize,
            _ => panic!("not a round entity"),
        }
    }

    pub fn radius_value(&self, e: EntRef) -> f64 {
        self.params[self.round_radius(e)].value
    }

    /// A solved point in world space, with its drawing-plane pose removed.
    pub fn world_point(&self, i: usize) -> [f64;3] {
        let p = self.point_xy(i);
        if let Some(i) = self.plane_of(i) {
            let f = &self.planes[i].frame;
            let q = crate::plane::in_view(self.params[f.c as usize].value,
                self.params[f.s as usize].value,self.point_xy(f.origin as usize),p);
            self.basis(i).lift(q.0,q.1)
        } else { crate::plane::Basis::page().lift(p.0,p.1) }
    }

    // -- geometry -----------------------------------------------------------

    pub fn arc_angles(&self, i: usize) -> (f64, f64) {
        let a = &self.arcs[i];
        let (cx, cy) = self.point_xy(a.center as usize);
        let (sx, sy) = self.point_xy(a.start as usize);
        let (ex, ey) = self.point_xy(a.end as usize);
        let a0 = (sy - cy).atan2(sx - cx);
        let mut a1 = (ey - cy).atan2(ex - cx);
        if a1 <= a0 {
            a1 += 2.0 * std::f64::consts::PI;
        }
        (a0, a1)
    }

    /// An arc's length along itself: its radius times that sweep.  What `length(L) a` states
    /// (the kernel reads the sweep the same way) and what `length(a)` measures.
    pub fn arc_length(&self, i: usize) -> f64 {
        let (a0, a1) = self.arc_angles(i);
        self.params[self.arcs[i].radius as usize].value.abs() * (a1 - a0)
    }

    /// The points that bound the drawn sweep: its two ends, plus every quarter-turn direction the
    /// sweep passes through.
    pub fn arc_extremes(&self, i: usize) -> Vec<(f64, f64)> {
        let a = &self.arcs[i];
        let (cx, cy) = self.point_xy(a.center as usize);
        let r = self.params[a.radius as usize].value.abs();
        let (a0, a1) = self.arc_angles(i);
        let at = |th: f64| (cx + r * th.cos(), cy + r * th.sin());
        let mut out = vec![at(a0), at(a1)];
        let quarter = std::f64::consts::FRAC_PI_2;
        let mut k = (a0 / quarter).ceil();
        while k * quarter < a1 {
            out.push(at(k * quarter));
            k += 1.0;
        }
        out
    }

    pub fn bounds(&self, e: EntRef) -> Box2 {
        match e.kind {
            // a face is where its edges are; a solid is not on the sheet at all, and what is
            // *drawn* of it is a derived view, which is geometry of its own
            EntKind::Face => {
                let mut b = (f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
                for c in self.faces[e.i()].edges.clone() {
                    let q = self.bounds(c);
                    b = (b.0.min(q.0), b.1.min(q.1), b.2.max(q.2), b.3.max(q.3));
                }
                b
            }
            EntKind::Solid | EntKind::Surface | EntKind::Motion | EntKind::Envelope | EntKind::Patch | EntKind::Seam | EntKind::Vertex | EntKind::Edge => {
                (f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY)
            }
            EntKind::Curve => {
                let mut b = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
                for (x, y) in self.curve_polyline(e.i()) {
                    b = (b.0.min(x), b.1.min(y), b.2.max(x), b.3.max(y));
                }
                b
            }
            EntKind::Point => {
                let (x, y) = self.point_xy(e.i());
                (x, y, x, y)
            }
            // on no sheet: its axis is drawn, in whichever view it is in
            EntKind::Cone | EntKind::Cylinder => {
                self.bounds(EntRef::line(self.axial(e).axis as usize))
            }
            // on no sheet: only its centre is drawn, in whichever view it is in
            EntKind::Sphere => {
                let (x, y) = self.point_xy(self.spheres[e.i()].center as usize);
                (x, y, x, y)
            }
            EntKind::Line => {
                let l = &self.lines[e.i()];
                let (ax, ay) = self.point_xy(l.p1 as usize);
                let (bx, by) = self.point_xy(l.p2 as usize);
                (ax.min(bx), ay.min(by), ax.max(bx), ay.max(by))
            }
            EntKind::Circle => {
                let c = &self.circles[e.i()];
                let (cx, cy) = self.point_xy(c.center as usize);
                let r = self.params[c.radius as usize].value.abs();
                (cx - r, cy - r, cx + r, cy + r)
            }
            EntKind::Plane => {
                let f = self.frame_of(e);
                let (ax, ay) = self.point_xy(f.origin as usize);
                let (bx, by) = self.point_xy(f.toward as usize);
                (ax.min(bx), ay.min(by), ax.max(bx), ay.max(by))
            }
            EntKind::Arc | EntKind::Spline => {
                let pts = if e.kind == EntKind::Arc {
                    self.arc_extremes(e.i())
                } else {
                    crate::curve::sample(self, e.i(), 16)
                };
                let mut b = (f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
                for (x, y) in pts {
                    b.0 = b.0.min(x);
                    b.1 = b.1.min(y);
                    b.2 = b.2.max(x);
                    b.3 = b.3.max(y);
                }
                b
            }
        }
    }

    /// (xmin, ymin, xmax, ymax) over all points.  Points only, deliberately: `extent()` is built
    /// on this, and `extent()` scales the solver's residual tolerances, the violated-constraint
    /// threshold, the witness perturbation and the drag continuation step.
    pub fn bbox(&self) -> Box2 {
        if self.points.is_empty() {
            return (0.0, 0.0, 1.0, 1.0);
        }
        let mut b = (f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
        for i in 0..self.points.len() {
            let (x, y) = self.point_xy(i);
            b.0 = b.0.min(x);
            b.1 = b.1.min(y);
            b.2 = b.2.max(x);
            b.3 = b.3.max(y);
        }
        b
    }

    /// Everything with something to draw: `primitives()` and the curves after it.
    ///
    /// A curve is written *over* the other kinds and so is built and grafted last, which is why
    /// `primitives()` stops short of it — but a consumer that means "draw the drawing" wants
    /// both, and this is where that is said.  Written once because it was already being patched
    /// up locally by everything that needed it.
    pub fn drawn(&self) -> Vec<EntRef> {
        let mut v = self.primitives();
        v.extend((0..self.curves.len()).map(|i| EntRef::new(EntKind::Curve, i)));
        v
    }

    /// Bounds of the drawn primitives — what a "fit the view" wants.
    ///
    /// **Curves are not in it, and that is a cost decision.**  A curve's `bounds` is its
    /// polyline, which for a traced family is a damped-Newton march per point; this runs inside
    /// `callout::layout`, so it is paid on every repaint.  Measured on `gear_trace` (24 traced
    /// curves) that is 11 ms a call against 1 µs — four orders of magnitude, per frame, to
    /// square up a box.  A caller that needs the curves in its box and is already sweeping them
    /// (the SVG export sizes a page from the polylines it is about to draw) grows this by what
    /// it swept; one that is not should not start.
    pub fn drawn_bounds(&self) -> Box2 {
        self.bounds_of(&self.primitives()).unwrap_or_else(|| self.bbox())
    }

    /// The box round a given set of entities, or `None` when the set draws nothing.  The fold
    /// `drawn_bounds` and the SVG export's page both want, so neither writes it out.
    pub fn bounds_of(&self, ents: &[EntRef]) -> Option<Box2> {
        let mut b = (f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
        for &e in ents {
            let x = self.bounds(e);
            b.0 = b.0.min(x.0);
            b.1 = b.1.min(x.1);
            b.2 = b.2.max(x.2);
            b.3 = b.3.max(x.3);
        }
        b.0.is_finite().then_some(b)
    }

    /// Characteristic length of the sketch (tolerances, drag weights).
    pub fn extent(&self) -> f64 {
        let (x0, y0, x1, y1) = self.bbox();
        (x1 - x0).max(y1 - y0).max(1.0)
    }

    pub fn nearest_point(&self, x: f64, y: f64) -> (Option<usize>, f64) {
        let mut best = None;
        let mut bd = f64::INFINITY;
        for i in 0..self.points.len() {
            let (px, py) = self.point_xy(i);
            let d = (px - x).hypot(py - y);
            if d < bd {
                best = Some(i);
                bd = d;
            }
        }
        (best, bd)
    }
}
