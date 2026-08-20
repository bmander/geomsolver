//! Parameters, primitives and the Sketch container.
//!
//! Every scalar degree of freedom is a `Param`.  Primitives are thin bundles of Param indices;
//! the Sketch owns the ordered list of Params (its parameter vector) and the ordered list of
//! Constraints.  Ordering is deterministic by construction — insertion order, never hashing — so
//! identical edits give bit-identical solves.
//!
//! Identity is an integer everywhere: a Param is its index, an entity is `(kind, index)`, and a
//! constraint is a monotonic `id`.  The bindings intern their proxies on those, so `is` / `===`
//! keep working across the FFI without any pointer ever leaving the core.

use crate::constraints::Constraint;
use crate::rng::Rng;
use std::collections::BTreeMap;

pub type Box2 = (f64, f64, f64, f64); // (xmin, ymin, xmax, ymax)

#[derive(Clone, Debug)]
pub struct Param {
    pub value: f64,
    pub fixed: bool,
    pub name: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum EntKind {
    Point,
    Line,
    Circle,
    Arc,
}

impl EntKind {
    pub fn as_str(self) -> &'static str {
        match self {
            EntKind::Point => "point",
            EntKind::Line => "line",
            EntKind::Circle => "circle",
            EntKind::Arc => "arc",
        }
    }

    pub fn parse(s: &str) -> Option<EntKind> {
        Some(match s {
            "point" => EntKind::Point,
            "line" => EntKind::Line,
            "circle" => EntKind::Circle,
            "arc" => EntKind::Arc,
            _ => return None,
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct EntRef {
    pub kind: EntKind,
    pub idx: u32,
}

impl EntRef {
    pub fn new(kind: EntKind, idx: usize) -> EntRef {
        EntRef { kind, idx: idx as u32 }
    }
    pub fn point(idx: usize) -> EntRef {
        EntRef::new(EntKind::Point, idx)
    }
    pub fn line(idx: usize) -> EntRef {
        EntRef::new(EntKind::Line, idx)
    }
    pub fn circle(idx: usize) -> EntRef {
        EntRef::new(EntKind::Circle, idx)
    }
    pub fn arc(idx: usize) -> EntRef {
        EntRef::new(EntKind::Arc, idx)
    }
    pub fn i(self) -> usize {
        self.idx as usize
    }
}

#[derive(Clone, Debug)]
pub struct PointE {
    pub x: u32,
    pub y: u32,
}

#[derive(Clone, Debug)]
pub struct LineE {
    pub p1: u32,
    pub p2: u32,
    /// Reference geometry: drawn dashed, constrains like any other.
    pub construction: bool,
}

#[derive(Clone, Debug)]
pub struct CircleE {
    pub center: u32,
    pub radius: u32,
    pub construction: bool,
}

/// CCW arc from `start` to `end` about `center`.  The radius is its own Param so Circle and Arc
/// share every radius-based constraint; the two intrinsic constraints |start-center|² = r² and
/// |end-center|² = r² are added by `Sketch::arc`.
#[derive(Clone, Debug)]
pub struct ArcE {
    pub center: u32,
    pub start: u32,
    pub end: u32,
    pub radius: u32,
    pub construction: bool,
}

/// The CCW arc through three points.
#[derive(Clone, Copy, Debug)]
pub struct ThreePointArc {
    pub cx: f64,
    pub cy: f64,
    pub r: f64,
    pub a0: f64,
    pub a1: f64,
    /// True when the sweep runs from the *second* given point to the first.
    pub swapped: bool,
}

/// Arc from (ax, ay) to (bx, by) passing through (cx, cy) — the circumcircle of the three, plus
/// the sweep direction that actually contains the third point.  `None` if they are collinear
/// (the test is on the sine of the angle, so it is scale-free).
pub fn three_point_arc(
    ax: f64,
    ay: f64,
    bx: f64,
    by: f64,
    cx: f64,
    cy: f64,
    tol: f64,
) -> Option<ThreePointArc> {
    let (ux, uy) = (bx - ax, by - ay);
    let (vx, vy) = (cx - ax, cy - ay);
    let cross = ux * vy - uy * vx;
    if cross.abs() <= tol * ux.hypot(uy) * vx.hypot(vy) {
        return None;
    }
    let d = 2.0 * cross;
    let (u2, v2) = (ux * ux + uy * uy, vx * vx + vy * vy);
    let ox = ax + (vy * u2 - uy * v2) / d;
    let oy = ay + (ux * v2 - vx * u2) / d;
    let r = (ax - ox).hypot(ay - oy);
    let ta = (ay - oy).atan2(ax - ox);
    let tb = (by - oy).atan2(bx - ox);
    let tau = 2.0 * std::f64::consts::PI;
    let sweep = |th: f64| ((th - ta) % tau + tau) % tau;
    let to_b = sweep(tb);
    let to_c = sweep((cy - oy).atan2(cx - ox));
    Some(if to_c < to_b {
        ThreePointArc { cx: ox, cy: oy, r, a0: ta, a1: ta + to_b, swapped: false }
    } else {
        ThreePointArc { cx: ox, cy: oy, r, a0: tb, a1: tb + (tau - to_b), swapped: true }
    })
}

#[derive(Default, Clone, Debug)]
pub struct Sketch {
    pub params: Vec<Param>,
    pub points: Vec<PointE>,
    pub lines: Vec<LineE>,
    pub circles: Vec<CircleE>,
    pub arcs: Vec<ArcE>,
    pub constraints: Vec<Constraint>,
    /// Recorded root choices (Stage 5), persisted with the document.
    pub branches: BTreeMap<String, i32>,
    next_cid: u32,
}

impl Sketch {
    pub fn new() -> Sketch {
        Sketch::default()
    }

    // -- construction -------------------------------------------------------

    pub fn param(&mut self, value: f64, fixed: bool, name: &str) -> usize {
        self.params.push(Param { value, fixed, name: name.to_string() });
        self.params.len() - 1
    }

    pub fn point(&mut self, x: f64, y: f64, fixed: bool, name: &str) -> usize {
        let px = self.param(x, fixed, &format!("{name}.x"));
        let py = self.param(y, fixed, &format!("{name}.y"));
        self.points.push(PointE { x: px as u32, y: py as u32 });
        self.points.len() - 1
    }

    pub fn line(&mut self, p1: usize, p2: usize) -> usize {
        self.lines.push(LineE { p1: p1 as u32, p2: p2 as u32, construction: false });
        self.lines.len() - 1
    }

    pub fn line_xy(&mut self, x1: f64, y1: f64, x2: f64, y2: f64, name: &str) -> usize {
        let a = self.point(x1, y1, false, &format!("{name}.p1"));
        let b = self.point(x2, y2, false, &format!("{name}.p2"));
        self.line(a, b)
    }

    pub fn circle(&mut self, center: usize, radius: f64, name: &str) -> usize {
        let r = self.param(radius, false, &format!("{name}.r"));
        self.circles.push(CircleE { center: center as u32, radius: r as u32, construction: false });
        self.circles.len() - 1
    }

    /// An arc plus its two intrinsic `PointOnCircle` constraints.
    pub fn arc(&mut self, center: usize, start: usize, end: usize, name: &str) -> usize {
        let (cx, cy) = self.point_xy(center);
        let (sx, sy) = self.point_xy(start);
        let r = (sx - cx).hypot(sy - cy);
        let rp = self.param(r, false, &format!("{name}.r"));
        self.arcs.push(ArcE {
            center: center as u32,
            start: start as u32,
            end: end as u32,
            radius: rp as u32,
            construction: false,
        });
        let ai = self.arcs.len() - 1;
        let aref = EntRef::arc(ai);
        let c1 = Constraint::point_on_circle(EntRef::point(start), aref, true);
        let c2 = Constraint::point_on_circle(EntRef::point(end), aref, true);
        self.add(c1);
        self.add(c2);
        ai
    }

    /// Arc from `start` to `end` bulging through `through` — the three-point construction.
    /// Creates the centre point; `None` if the three are collinear.
    pub fn arc_through(
        &mut self,
        start: usize,
        end: usize,
        through: (f64, f64),
        name: &str,
    ) -> Option<usize> {
        let (ax, ay) = self.point_xy(start);
        let (bx, by) = self.point_xy(end);
        let g = three_point_arc(ax, ay, bx, by, through.0, through.1, 1e-9)?;
        let centre = self.point(g.cx, g.cy, false, &format!("{name}.c"));
        let (a, b) = if g.swapped { (end, start) } else { (start, end) };
        Some(self.arc(centre, a, b, name))
    }

    /// Four lines round the corners `a` and (x1, y1), sharing corner points, with three
    /// perpendicular constraints.  Three, not four: the fourth follows, so adding it would make
    /// every rectangle over-constrained by one equation.  What is left is the 5 DOF a rectangle
    /// has — position, rotation, width, height.
    pub fn rectangle(&mut self, a: usize, x1: f64, y1: f64, name: &str) -> Vec<usize> {
        let (x0, y0) = self.point_xy(a);
        let corners = [
            a,
            self.point(x1, y0, false, &format!("{name}.b")),
            self.point(x1, y1, false, &format!("{name}.c")),
            self.point(x0, y1, false, &format!("{name}.d")),
        ];
        let lines: Vec<usize> =
            (0..4).map(|i| self.line(corners[i], corners[(i + 1) % 4])).collect();
        for i in 0..3 {
            let c = Constraint::two_line(
                crate::constraints::CKind::Perpendicular,
                EntRef::line(lines[i]),
                EntRef::line(lines[i + 1]),
            );
            self.add(c);
        }
        lines
    }

    pub fn rectangle_xy(&mut self, x0: f64, y0: f64, x1: f64, y1: f64, name: &str) -> Vec<usize> {
        let a = self.point(x0, y0, false, &format!("{name}.a"));
        self.rectangle(a, x1, y1, name)
    }

    /// Append a constraint, assigning it a fresh document-stable id.
    pub fn add(&mut self, mut c: Constraint) -> u32 {
        if c.id == 0 {
            self.next_cid += 1;
            c.id = self.next_cid;
        } else {
            self.next_cid = self.next_cid.max(c.id);
        }
        let id = c.id;
        self.constraints.push(c);
        id
    }

    pub fn remove(&mut self, id: u32) {
        self.constraints.retain(|c| c.id != id);
    }

    pub fn constraint(&self, id: u32) -> Option<&Constraint> {
        self.constraints.iter().find(|c| c.id == id)
    }

    pub fn constraint_mut(&mut self, id: u32) -> Option<&mut Constraint> {
        self.constraints.iter_mut().find(|c| c.id == id)
    }

    pub fn index_of(&self, id: u32) -> Option<usize> {
        self.constraints.iter().position(|c| c.id == id)
    }

    // -- accessors ----------------------------------------------------------

    pub fn point_xy(&self, i: usize) -> (f64, f64) {
        let p = &self.points[i];
        (self.params[p.x as usize].value, self.params[p.y as usize].value)
    }

    pub fn point_params(&self, i: usize) -> [u32; 2] {
        let p = &self.points[i];
        [p.x, p.y]
    }

    pub fn point_fixed(&self, i: usize) -> bool {
        let p = &self.points[i];
        self.params[p.x as usize].fixed && self.params[p.y as usize].fixed
    }

    pub fn fix_point(&mut self, i: usize, fixed: bool) {
        let (x, y) = (self.points[i].x as usize, self.points[i].y as usize);
        self.params[x].fixed = fixed;
        self.params[y].fixed = fixed;
    }

    pub fn line_params(&self, i: usize) -> [u32; 4] {
        let l = &self.lines[i];
        let (a, b) = (&self.points[l.p1 as usize], &self.points[l.p2 as usize]);
        [a.x, a.y, b.x, b.y]
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

    /// Centre point index of a circle or arc.
    pub fn round_center(&self, e: EntRef) -> usize {
        match e.kind {
            EntKind::Circle => self.circles[e.i()].center as usize,
            EntKind::Arc => self.arcs[e.i()].center as usize,
            _ => panic!("not a round entity"),
        }
    }

    /// Radius Param index of a circle or arc.
    pub fn round_radius(&self, e: EntRef) -> usize {
        match e.kind {
            EntKind::Circle => self.circles[e.i()].radius as usize,
            EntKind::Arc => self.arcs[e.i()].radius as usize,
            _ => panic!("not a round entity"),
        }
    }

    pub fn radius_value(&self, e: EntRef) -> f64 {
        self.params[self.round_radius(e)].value
    }

    /// Params of any primitive, in the model's canonical order.
    pub fn entity_params(&self, e: EntRef) -> Vec<u32> {
        match e.kind {
            EntKind::Point => self.point_params(e.i()).to_vec(),
            EntKind::Line => self.line_params(e.i()).to_vec(),
            EntKind::Circle => {
                let c = &self.circles[e.i()];
                let p = &self.points[c.center as usize];
                vec![p.x, p.y, c.radius]
            }
            EntKind::Arc => {
                let a = &self.arcs[e.i()];
                let mut v = Vec::with_capacity(7);
                for pi in [a.center, a.start, a.end] {
                    let p = &self.points[pi as usize];
                    v.push(p.x);
                    v.push(p.y);
                }
                v.push(a.radius);
                v
            }
        }
    }

    /// Sub-entities: a line's endpoints, an arc's centre and ends.
    pub fn children(&self, e: EntRef) -> Vec<EntRef> {
        match e.kind {
            EntKind::Point => Vec::new(),
            EntKind::Line => {
                let l = &self.lines[e.i()];
                vec![EntRef::point(l.p1 as usize), EntRef::point(l.p2 as usize)]
            }
            EntKind::Circle => vec![EntRef::point(self.circles[e.i()].center as usize)],
            EntKind::Arc => {
                let a = &self.arcs[e.i()];
                vec![
                    EntRef::point(a.center as usize),
                    EntRef::point(a.start as usize),
                    EntRef::point(a.end as usize),
                ]
            }
        }
    }

    /// What a compiled plan or `System` depends on: which entities exist, which constraints (by
    /// id, so swapping one Distance for another shows up — counts and type names alone do not)
    /// and which params are fixed.  A cache over compiled artefacts keys on this.
    pub fn topology_key(&self) -> String {
        use std::fmt::Write;
        let mut s = format!(
            "{}|{}|{}|{}|",
            self.points.len(),
            self.lines.len(),
            self.circles.len(),
            self.arcs.len()
        );
        for c in &self.constraints {
            let _ = write!(s, "{}:{},", c.id, c.type_name());
        }
        s.push('|');
        for p in &self.params {
            s.push(if p.fixed { '1' } else { '0' });
        }
        s
    }

    pub fn count(&self, kind: EntKind) -> usize {
        match kind {
            EntKind::Point => self.points.len(),
            EntKind::Line => self.lines.len(),
            EntKind::Circle => self.circles.len(),
            EntKind::Arc => self.arcs.len(),
        }
    }

    /// Every entity, in creation order per kind.
    pub fn primitives(&self) -> Vec<EntRef> {
        let mut out = Vec::new();
        for kind in [EntKind::Point, EntKind::Line, EntKind::Circle, EntKind::Arc] {
            for i in 0..self.count(kind) {
                out.push(EntRef::new(kind, i));
            }
        }
        out
    }

    // -- parameter vector ---------------------------------------------------

    pub fn get_x(&self) -> Vec<f64> {
        self.params.iter().map(|p| p.value).collect()
    }

    /// Write the parameter vector.  A vector of the wrong length is not this sketch's — writing
    /// the overlapping prefix would scatter one sketch's coordinates over another's — so it is
    /// refused; `false` says nothing was written.
    pub fn set_x(&mut self, x: &[f64]) -> bool {
        if x.len() != self.params.len() {
            return false;
        }
        for (i, p) in self.params.iter_mut().enumerate() {
            p.value = x[i];
        }
        true
    }

    pub fn free_indices(&self) -> Vec<i32> {
        self.params
            .iter()
            .enumerate()
            .filter(|(_, p)| !p.fixed)
            .map(|(i, _)| i as i32)
            .collect()
    }

    pub fn n_residuals(&self) -> usize {
        self.constraints.iter().map(|c| c.n_residuals()).sum()
    }

    /// Constraints the user added (excludes intrinsic and soft/transient ones).
    pub fn user_constraints(&self) -> Vec<&Constraint> {
        self.constraints.iter().filter(|c| !(c.intrinsic || c.soft)).collect()
    }

    /// Everything that must be satisfied (excludes soft ones such as drag targets).
    pub fn hard_constraints(&self) -> Vec<&Constraint> {
        self.constraints.iter().filter(|c| !c.soft).collect()
    }

    pub fn hard_ids(&self) -> Vec<u32> {
        self.constraints.iter().filter(|c| !c.soft).map(|c| c.id).collect()
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
            EntKind::Point => {
                let (x, y) = self.point_xy(e.i());
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
            EntKind::Arc => {
                let pts = self.arc_extremes(e.i());
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

    /// Bounds of everything drawn, curves included — what a "fit the view" wants.
    pub fn drawn_bounds(&self) -> Box2 {
        let mut b = (f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
        let mut any = false;
        for e in self.primitives() {
            let x = self.bounds(e);
            any = true;
            b.0 = b.0.min(x.0);
            b.1 = b.1.min(x.1);
            b.2 = b.2.max(x.2);
            b.3 = b.3.max(x.3);
        }
        if any {
            b
        } else {
            self.bbox()
        }
    }

    /// Characteristic length of the sketch (tolerances, drag weights).
    pub fn extent(&self) -> f64 {
        let (x0, y0, x1, y1) = self.bbox();
        (x1 - x0).max(y1 - y0).max(1.0)
    }

    /// Seeded Gaussian noise on every free parameter (warm starts, witness construction).
    pub fn perturb(&mut self, sigma: f64, seed: u32) {
        let mut rng = Rng::new(seed);
        for p in self.params.iter_mut() {
            if !p.fixed {
                p.value += rng.normal(0.0, sigma);
            }
        }
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

/// Signed perpendicular offset from the *infinite* line through `line`, positive to the left of
/// its direction.  A degenerate line has no side; it gives infinity rather than a silent zero.
pub fn signed_point_to_line(sk: &Sketch, px: f64, py: f64, line: usize) -> f64 {
    let l = &sk.lines[line];
    let (ax, ay) = sk.point_xy(l.p1 as usize);
    let (dx, dy) = sk.line_dir(line);
    let length = dx.hypot(dy);
    if length == 0.0 {
        return f64::INFINITY;
    }
    (dx * (py - ay) - dy * (px - ax)) / length
}

fn point_to_line(sk: &Sketch, px: f64, py: f64, line: usize) -> f64 {
    let (dx, dy) = sk.line_dir(line);
    if dx == 0.0 && dy == 0.0 {
        let l = &sk.lines[line];
        let (ax, ay) = sk.point_xy(l.p1 as usize);
        return (px - ax).hypot(py - ay);
    }
    signed_point_to_line(sk, px, py, line).abs()
}

fn measure_order(k: EntKind) -> u8 {
    match k {
        EntKind::Point => 0,
        EntKind::Line => 1,
        EntKind::Circle | EntKind::Arc => 2,
    }
}

/// Signed CCW angle from line `a` to line `b`, in radians — what an `Angle` constraint's value
/// means, and what a dimension dialog should offer as the current value.
pub fn angle_between(sk: &Sketch, a: EntRef, b: EntRef) -> f64 {
    let (d1x, d1y) = sk.line_dir(a.i());
    let (d2x, d2y) = sk.line_dir(b.i());
    (d1x * d2y - d1y * d2x).atan2(d1x * d2x + d1y * d2y)
}

/// The point at distance `r` from (cx, cy) in the direction of (tx, ty).  The centre–start–end
/// arc construction: the third click gives a direction, and the radius comes from the second.
/// `None` when the target is the centre, which names no direction.
pub fn on_radius(cx: f64, cy: f64, tx: f64, ty: f64, r: f64) -> Option<(f64, f64)> {
    let (dx, dy) = (tx - cx, ty - cy);
    let l = dx.hypot(dy);
    if l <= 1e-12 {
        return None;
    }
    Some((cx + r * dx / l, cy + r * dy / l))
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
            match b.kind {
                EntKind::Point => {
                    let (bx, by) = sk.point_xy(b.i());
                    (ax - bx).hypot(ay - by)
                }
                EntKind::Line => point_to_line(sk, ax, ay, b.i()),
                _ => {
                    let (cx, cy) = sk.point_xy(sk.round_center(b));
                    ((ax - cx).hypot(ay - cy) - sk.radius_value(b).abs()).abs()
                }
            }
        }
        EntKind::Line => match b.kind {
            EntKind::Line => {
                let d1 = sk.line_dir(a.i());
                let d2 = sk.line_dir(b.i());
                let cross = d1.0 * d2.1 - d1.1 * d2.0;
                if cross.abs() > 1e-9 * d1.0.hypot(d1.1) * d2.0.hypot(d2.1) {
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
            let gap = (ax - bx).hypot(ay - by);
            let (r1, r2) = (sk.radius_value(a).abs(), sk.radius_value(b).abs());
            (gap - r1 - r2).max((r1 - r2).abs() - gap).max(0.0)
        }
    }
}

/// Entities plus their sub-entities.
pub fn expand(sk: &Sketch, ents: &[EntRef]) -> Vec<EntRef> {
    let mut out = Vec::new();
    for &e in ents {
        out.push(e);
        out.extend(sk.children(e));
    }
    out
}

/// Twice the signed area of (a, b, c) — the order-type invariant the drag guards.
pub fn orientation(sk: &Sketch, a: usize, b: usize, c: usize) -> f64 {
    let (ax, ay) = sk.point_xy(a);
    let (bx, by) = sk.point_xy(b);
    let (cx, cy) = sk.point_xy(c);
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
    let n = (((x1 - x0).hypot(y1 - y0) / max_step).ceil() as i64).max(1);
    (1..=n)
        .map(|i| {
            let t = i as f64 / n as f64;
            (x0 + (x1 - x0) * t, y0 + (y1 - y0) * t)
        })
        .collect()
}
