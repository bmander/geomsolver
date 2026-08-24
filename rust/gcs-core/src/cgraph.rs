//! Constraint graph for decomposition (Stage 3).
//!
//! Fudos–Hoffmann work on geometric elements with 2 DOF each in the plane — points and (infinite)
//! lines — joined by valency-1 constraints: point-point distance, point-line signed distance
//! (0 = incidence), line-line angle.  This module maps a Sketch onto that:
//!
//! * points are contracted by `Coincident` (one element per equivalence class);
//! * a Line becomes a line element the first time a supported constraint refers to it (its
//!   endpoints are incident); lines nobody refers to are passive and get no element;
//! * a circle/arc whose radius is known (`Radius`, a fixed param, or an `EqualRadius` chain to a
//!   known one) contributes distance edges: point-on-circle → dist(centre, point) = r, line
//!   tangency → signed dist(centre, line) = ±r, arc-endpoint tangency → a virtual radius line
//!   through centre and endpoint, perpendicular to the tangent line;
//! * angle-type constraints (Horizontal/Vertical against a ground x-axis, Parallel, Perpendicular,
//!   Angle) are direction relations, not rigid pairs — they contribute one equation when clusters
//!   merge.  Fixed points and the x-axis form the ground elements;
//! * everything else is listed as `unsupported` and left to the numeric residual step.

use crate::constraints::CKind;
use crate::graph::UnionFind;
use crate::model::{EntKind, EntRef, Sketch};
use std::collections::BTreeMap;

/// Python's (kind, idx) tuple order is L < P < V; the derived `Ord` reproduces it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum ElKind {
    L,
    P,
    V,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct El {
    pub kind: ElKind,
    pub idx: i32,
}

impl El {
    pub fn new(kind: ElKind, idx: i32) -> El {
        El { kind, idx }
    }
    pub fn p(i: usize) -> El {
        El::new(ElKind::P, i as i32)
    }
    pub fn l(i: usize) -> El {
        El::new(ElKind::L, i as i32)
    }
    pub fn v(i: usize) -> El {
        El::new(ElKind::V, i as i32)
    }
    pub fn i(self) -> usize {
        self.idx as usize
    }
    pub fn is_point(self) -> bool {
        self.kind == ElKind::P
    }
    /// Pose length: a point has (x, y), a line (nx, ny, c).
    pub fn size(self) -> usize {
        if self.is_point() {
            2
        } else {
            3
        }
    }
}

/// Ground line y = 0 (normal (0,1), c = 0).
pub const X_AXIS: El = El { kind: ElKind::L, idx: -1 };

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EdgeKind {
    /// point–point distance
    Pp,
    /// point–line signed distance (0 = incidence)
    Pl,
}

/// Where an edge's value comes from.  Reading it on every execution is what lets edits and drags
/// replay without recompiling the plan.
#[derive(Clone, Copy, Debug)]
pub enum EdgeVal {
    Zero,
    /// A live dimension: argument `arg` of constraint `cid`.
    Dim { cid: u32, arg: usize },
    /// `sign` times the graph's known radius for that radius Param.
    Radius { param: u32, sign: f64 },
}

#[derive(Clone, Debug)]
pub struct Edge {
    pub kind: EdgeKind,
    pub a: El,
    pub b: El,
    pub value: EdgeVal,
    /// `None` for implicit edges (line endpoints, virtual radii).
    pub source: Option<u32>,
}

/// dir(b) = dir(a) + phi (normals: n_b = rot(phi) n_a).  `phi` is the branch (mod pi) nearest the
/// current geometry at build time — a chirality-like choice.
#[derive(Clone, Copy, Debug)]
pub struct DirRelation {
    pub a: El,
    pub b: El,
    pub phi: f64,
    pub source: u32,
}

#[derive(Clone, Debug, Default)]
pub struct ConstraintGraph {
    /// sketch point index → coincidence class
    pub point_of: Vec<usize>,
    /// class → member sketch point indices
    pub members: Vec<Vec<usize>>,
    /// line element index → sketch line index
    pub lines: Vec<usize>,
    line_of: BTreeMap<usize, usize>,
    pub virtuals: Vec<(El, El)>,
    pub edges: Vec<Edge>,
    pub dirs: Vec<DirRelation>,
    pub unsupported: Vec<u32>,
    pub ground_points: Vec<usize>,
    /// coincidence class → the member point that pinned it to the ground
    pub ground_member: BTreeMap<usize, usize>,
    /// radius Param index → its known value
    pub known_radius: BTreeMap<u32, f64>,
    pub passive: Vec<usize>,
}

impl ConstraintGraph {
    pub fn point_el(&self, p: usize) -> El {
        El::p(self.point_of[p])
    }

    /// The point of a coincidence class whose coordinates *are* the class's pose: the member that
    /// pinned the class to the ground if it has one — a fixed point, or the point a drag has
    /// pinned — otherwise the lowest-numbered member.  Every member agrees once the sketch is
    /// satisfied; while it is not, this is the one that has to win.
    pub fn class_pose_point(&self, class: usize) -> usize {
        self.ground_member.get(&class).copied().unwrap_or(self.members[class][0])
    }

    /// Re-read the known radius values from the sketch.  `known_radius` is a value cache over a
    /// structure fixed when the graph was built, and a plan is cached per topology and replayed
    /// after dimension edits — so an edited `Radius` has to reach the replay, or every such edit
    /// fails its residual check and falls back to the numeric solver.  Only radii already known
    /// are refreshed: a change in *which* radii are known is a topology change, and recompiles.
    pub fn refresh_radii(&mut self, sk: &Sketch) {
        if self.known_radius.is_empty() {
            return;
        }
        let fresh = known_radii(sk);
        for (p, v) in self.known_radius.iter_mut() {
            if let Some(&nv) = fresh.get(p) {
                *v = nv;
            }
        }
    }

    /// Line element for sketch line `ln`, registered on first use.
    fn line_el(&mut self, ln: usize) -> El {
        if let Some(&i) = self.line_of.get(&ln) {
            return El::l(i);
        }
        let i = self.lines.len();
        self.line_of.insert(ln, i);
        self.lines.push(ln);
        El::l(i)
    }

    pub fn has_line(&self, ln: usize) -> bool {
        self.line_of.contains_key(&ln)
    }

    /// Line element through two point elements (an arc's radius at an endpoint).
    fn virtual_line(&mut self, a: El, b: El) -> El {
        self.virtuals.push((a, b));
        El::v(self.virtuals.len() - 1)
    }

    /// Document-stable index of a point element: the smallest sketch index among the Points of its
    /// coincidence class (element numbering itself depends on how the graph was built).
    pub fn point_index(&self, e: El) -> usize {
        self.members[e.i()].iter().copied().min().unwrap_or(0)
    }

    pub fn n_points(&self) -> usize {
        self.members.len()
    }

    pub fn elements(&self) -> Vec<El> {
        let mut out: Vec<El> = (0..self.members.len()).map(El::p).collect();
        out.extend((0..self.lines.len()).map(El::l));
        out.extend((0..self.virtuals.len()).map(El::v));
        out.push(X_AXIS);
        out
    }

    pub fn edge_value(&self, sk: &Sketch, e: &Edge) -> f64 {
        match e.value {
            EdgeVal::Zero => 0.0,
            EdgeVal::Dim { cid, arg } => {
                sk.constraint(cid).map(|c| c.args[arg].num()).unwrap_or(0.0)
            }
            EdgeVal::Radius { param, sign } => {
                sign * self.known_radius.get(&param).copied().unwrap_or(0.0)
            }
        }
    }

    pub fn summary(&self) -> String {
        format!(
            "{} point elements, {} lines (+{} passive, {} virtual), {} edges, {} direction \
             relations ({} unsupported constraints), {} ground points",
            self.n_points(),
            self.lines.len(),
            self.passive.len(),
            self.virtuals.len(),
            self.edges.len(),
            self.dirs.len(),
            self.unsupported.len(),
            self.ground_points.len()
        )
    }
}

/// Points contracted by `Coincident`: (point → class index, class → member points).
pub fn coincident_classes(sk: &Sketch) -> (Vec<usize>, Vec<Vec<usize>>) {
    let n = sk.points.len();
    let mut uf = UnionFind::new(n);
    for c in &sk.constraints {
        if c.kind == CKind::Coincident {
            uf.union(c.args[0].ent().i(), c.args[1].ent().i());
        }
    }
    let (label, count) = uf.labels();
    let mut members: Vec<Vec<usize>> = vec![Vec::new(); count];
    for (i, &l) in label.iter().enumerate() {
        members[l].push(i);
    }
    (label, members)
}

/// Radius Param → value for every radius fixed, dimensioned by `Radius`, or `EqualRadius`-chained
/// to one.  An `AnnularDistance` carries a known radius across to the other circle; the pass
/// iterates so a chain of nested rings resolves from whichever one of them is dimensioned.
pub fn known_radii(sk: &Sketch) -> BTreeMap<u32, f64> {
    let mut radii: Vec<u32> = Vec::new();
    for i in 0..sk.circles.len() {
        radii.push(sk.circles[i].radius);
    }
    for i in 0..sk.arcs.len() {
        radii.push(sk.arcs[i].radius);
    }
    let ridx: BTreeMap<u32, usize> = radii.iter().enumerate().map(|(i, &r)| (r, i)).collect();
    let mut uf = UnionFind::new(radii.len());
    // a soft Radius (a live RadiusDrag) is not a dimension, and neither is one whose number is
    // a free variable — it states which radius this is the same as, not what it is
    let hard: Vec<&crate::constraints::Constraint> =
        sk.hard_constraints().into_iter().filter(|c| c.free.is_none()).collect();
    for c in &hard {
        if c.kind == CKind::EqualRadius {
            let a = ridx[&(sk.round_radius(c.args[0].ent()) as u32)];
            let b = ridx[&(sk.round_radius(c.args[1].ent()) as u32)];
            uf.union(a, b);
        }
    }
    let mut known: BTreeMap<usize, f64> = BTreeMap::new();
    for c in &hard {
        // after all unions, so class roots are final
        if c.kind == CKind::Radius {
            let root = uf.find(ridx[&(sk.round_radius(c.args[0].ent()) as u32)]);
            known.insert(root, c.args[1].num());
        }
    }
    for &r in &radii {
        let root = uf.find(ridx[&r]);
        if sk.params[r as usize].fixed {
            known.entry(root).or_insert(sk.params[r as usize].value);
        }
    }
    let offsets: Vec<&crate::constraints::Constraint> =
        hard.iter().copied().filter(|c| c.kind == CKind::AnnularDistance).collect();
    let mut changed = true;
    while changed {
        changed = false;
        for c in &offsets {
            let a = uf.find(ridx[&(sk.round_radius(c.args[0].ent()) as u32)]);
            let b = uf.find(ridx[&(sk.round_radius(c.args[1].ent()) as u32)]);
            let d = c.args[2].num();
            if known.contains_key(&a) && !known.contains_key(&b) {
                let v = known[&a] + d;
                known.insert(b, v);
            } else if known.contains_key(&b) && !known.contains_key(&a) {
                let v = known[&b] - d;
                known.insert(a, v);
            } else {
                continue;
            }
            changed = true;
        }
    }
    let mut out = BTreeMap::new();
    for &r in &radii {
        let root = uf.find(ridx[&r]);
        if let Some(&v) = known.get(&root) {
            out.insert(r, v);
        }
    }
    out
}

/// (nx, ny, c) for the line a→b: n is the unit left normal and n·X = c on the line.
pub fn normal_of(ax: f64, ay: f64, bx: f64, by: f64) -> [f64; 3] {
    let (dx, dy) = (bx - ax, by - ay);
    let l = {
        let h = dx.hypot(dy);
        if h == 0.0 {
            1.0
        } else {
            h
        }
    };
    let (nx, ny) = (-dy / l, dx / l);
    [nx, ny, nx * ax + ny * ay]
}

pub fn line_normal(sk: &Sketch, ln: usize) -> [f64; 3] {
    let l = &sk.lines[ln];
    let (ax, ay) = sk.point_xy(l.p1 as usize);
    let (bx, by) = sk.point_xy(l.p2 as usize);
    normal_of(ax, ay, bx, by)
}

/// Angle from normal n1 to normal n2 on the branch of `target` (mod pi) nearest the current
/// geometry.
pub fn branch(n1: &[f64], n2: &[f64], target: f64) -> f64 {
    let cur = (n1[0] * n2[1] - n1[1] * n2[0]).atan2(n1[0] * n2[0] + n1[1] * n2[1]);
    let k = ((cur - target) / std::f64::consts::PI).round();
    target + k * std::f64::consts::PI
}

/// IEEE remainder: the value r with |r| <= |y|/2 and x − r a multiple of y.
pub fn remainder(x: f64, y: f64) -> f64 {
    let q = (x / y).round();
    x - q * y
}

pub fn build(sk: &Sketch) -> ConstraintGraph {
    let mut g = ConstraintGraph::default();
    let (of, members) = coincident_classes(sk);
    g.point_of = of;
    for (k, ms) in members.iter().enumerate() {
        if let Some(&p) = ms.iter().find(|&&p| sk.point_fixed(p)) {
            g.ground_points.push(k);
            g.ground_member.insert(k, p);
        }
    }
    g.members = members;
    g.known_radius = known_radii(sk);

    let known = |g: &ConstraintGraph, e: EntRef| -> Option<f64> {
        g.known_radius.get(&(sk.round_radius(e) as u32)).copied()
    };

    for c in &sk.constraints {
        // A dimension written in terms of a free variable states no length: it states a
        // *relation* between dimensions, and the cluster vocabulary has no element for that —
        // an edge carrying it would be read as a rigid distance somebody had fixed.  So it goes
        // to the numeric residual, on the same grounds as the run and the rise.
        if c.free.is_some() {
            g.unsupported.push(c.id);
            continue;
        }
        // contracted / absorbed
        if c.soft || c.kind == CKind::Coincident || c.kind == CKind::Radius {
            continue;
        }
        match c.kind {
            CKind::Distance => {
                let (a, b) = (g.point_el(c.args[0].ent().i()), g.point_el(c.args[1].ent().i()));
                g.edges.push(Edge {
                    kind: EdgeKind::Pp,
                    a,
                    b,
                    value: EdgeVal::Dim { cid: c.id, arg: 2 },
                    source: Some(c.id),
                });
            }
            CKind::PointOnLine => {
                let a = g.point_el(c.args[0].ent().i());
                let b = g.line_el(c.args[1].ent().i());
                g.edges.push(Edge {
                    kind: EdgeKind::Pl,
                    a,
                    b,
                    value: EdgeVal::Zero,
                    source: Some(c.id),
                });
            }
            CKind::PointLineDistance => {
                let a = g.point_el(c.args[0].ent().i());
                let b = g.line_el(c.args[1].ent().i());
                g.edges.push(Edge {
                    kind: EdgeKind::Pl,
                    a,
                    b,
                    value: EdgeVal::Dim { cid: c.id, arg: 2 },
                    source: Some(c.id),
                });
            }
            CKind::ParallelDistance => {
                // one residual — l2's first endpoint offset from l1 — so it is the same PL element
                let l2 = c.args[1].ent().i();
                let p1 = sk.lines[l2].p1 as usize;
                let a = g.point_el(p1);
                let b = g.line_el(c.args[0].ent().i());
                g.edges.push(Edge {
                    kind: EdgeKind::Pl,
                    a,
                    b,
                    value: EdgeVal::Dim { cid: c.id, arg: 2 },
                    source: Some(c.id),
                });
            }
            CKind::Horizontal | CKind::Vertical => {
                let ln = c.args[0].ent().i();
                let target =
                    if c.kind == CKind::Vertical { std::f64::consts::FRAC_PI_2 } else { 0.0 };
                let b = g.line_el(ln);
                let phi = branch(&[0.0, 1.0, 0.0], &line_normal(sk, ln), target);
                g.dirs.push(DirRelation { a: X_AXIS, b, phi, source: c.id });
            }
            // A levelled pair of points says exactly what a levelled line does, about the
            // segment between them — so it decomposes the same way: a virtual line through the
            // two, in the ground x-axis's direction class.  Without this it would be an
            // unsupported constraint and every drag touching it would take the numeric path.
            CKind::HorizontalPoints | CKind::VerticalPoints => {
                let (i1, i2) = (c.args[0].ent().i(), c.args[1].ent().i());
                let (p, q) = (g.point_el(i1), g.point_el(i2));
                if p == q {
                    // already the same point: the constraint says nothing the graph can use
                    g.unsupported.push(c.id);
                    continue;
                }
                let v = g.virtual_line(p, q);
                for e in [p, q] {
                    g.edges.push(Edge {
                        kind: EdgeKind::Pl,
                        a: e,
                        b: v,
                        value: EdgeVal::Zero,
                        source: None,
                    });
                }
                let target = if c.kind == CKind::VerticalPoints {
                    std::f64::consts::FRAC_PI_2
                } else {
                    0.0
                };
                let (ax, ay) = sk.point_xy(i1);
                let (bx, by) = sk.point_xy(i2);
                let phi = branch(&[0.0, 1.0, 0.0], &normal_of(ax, ay, bx, by), target);
                g.dirs.push(DirRelation { a: X_AXIS, b: v, phi, source: c.id });
            }
            CKind::Parallel | CKind::Perpendicular | CKind::Angle => {
                let (i1, i2) = (c.args[0].ent().i(), c.args[1].ent().i());
                let target = match c.kind {
                    CKind::Parallel => 0.0,
                    CKind::Perpendicular => std::f64::consts::FRAC_PI_2,
                    _ => c.args[2].num(),
                };
                let a = g.line_el(i1);
                let b = g.line_el(i2);
                let phi = branch(&line_normal(sk, i1), &line_normal(sk, i2), target);
                g.dirs.push(DirRelation { a, b, phi, source: c.id });
            }
            CKind::PointOnCircle if known(&g, c.args[1].ent()).is_some() => {
                let circle = c.args[1].ent();
                let param = sk.round_radius(circle) as u32;
                let a = g.point_el(sk.round_center(circle));
                let b = g.point_el(c.args[0].ent().i());
                g.edges.push(Edge {
                    kind: EdgeKind::Pp,
                    a,
                    b,
                    value: EdgeVal::Radius { param, sign: 1.0 },
                    source: Some(c.id),
                });
            }
            CKind::TangentLineCircle if known(&g, c.args[1].ent()).is_some() => {
                // signed distance from centre to line = side*r (our line normal n = (-dy, dx)/|d|
                // gives n·(c − a) = cross(d, c − a)/|d|, which is the kernel's residual)
                let circle = c.args[1].ent();
                let param = sk.round_radius(circle) as u32;
                let side = c.args[2].num();
                let a = g.point_el(sk.round_center(circle));
                let b = g.line_el(c.args[0].ent().i());
                g.edges.push(Edge {
                    kind: EdgeKind::Pl,
                    a,
                    b,
                    value: EdgeVal::Radius { param, sign: side },
                    source: Some(c.id),
                });
            }
            CKind::TangentArcLine if known(&g, c.args[0].ent()).is_some() => {
                // tangent at endpoint p ⟺ radius c−p perpendicular to the line (p on the line and
                // |c − p| = r come from elsewhere)
                let arc = &sk.arcs[c.args[0].ent().i()];
                let at = match &c.args[2] {
                    crate::constraints::Arg::Str(s) if s == "start" => arc.start,
                    _ => arc.end,
                } as usize;
                let centre_pt = arc.center as usize;
                let cen = g.point_el(centre_pt);
                let pe = g.point_el(at);
                let vr = g.virtual_line(cen, pe);
                g.edges.push(Edge {
                    kind: EdgeKind::Pl,
                    a: cen,
                    b: vr,
                    value: EdgeVal::Zero,
                    source: None,
                });
                g.edges.push(Edge {
                    kind: EdgeKind::Pl,
                    a: pe,
                    b: vr,
                    value: EdgeVal::Zero,
                    source: None,
                });
                let (cx, cy) = sk.point_xy(centre_pt);
                let (px, py) = sk.point_xy(at);
                let n_r = normal_of(cx, cy, px, py);
                let ln = c.args[1].ent().i();
                let a = g.line_el(ln);
                let phi = branch(&line_normal(sk, ln), &n_r, std::f64::consts::FRAC_PI_2);
                g.dirs.push(DirRelation { a, b: vr, phi, source: c.id });
            }
            CKind::TangentLineCircleAt if known(&g, c.args[1].ent()).is_some() => {
                // the arc rule for a full circle: tangent at the line's own endpoint p ⟺ the
                // radius c−p is perpendicular to the line (p on the line is the endpoint itself,
                // |c − p| = r is the user's PointOnCircle)
                let l = &sk.lines[c.args[0].ent().i()];
                let at = match &c.args[2] {
                    crate::constraints::Arg::Str(s) if s == "p2" => l.p2,
                    _ => l.p1,
                } as usize;
                let centre_pt = sk.round_center(c.args[1].ent());
                let cen = g.point_el(centre_pt);
                let pe = g.point_el(at);
                let vr = g.virtual_line(cen, pe);
                g.edges.push(Edge {
                    kind: EdgeKind::Pl,
                    a: cen,
                    b: vr,
                    value: EdgeVal::Zero,
                    source: None,
                });
                g.edges.push(Edge {
                    kind: EdgeKind::Pl,
                    a: pe,
                    b: vr,
                    value: EdgeVal::Zero,
                    source: None,
                });
                let (cx, cy) = sk.point_xy(centre_pt);
                let (px, py) = sk.point_xy(at);
                let n_r = normal_of(cx, cy, px, py);
                let ln = c.args[0].ent().i();
                let a = g.line_el(ln);
                let phi = branch(&line_normal(sk, ln), &n_r, std::f64::consts::FRAC_PI_2);
                g.dirs.push(DirRelation { a, b: vr, phi, source: c.id });
            }
            CKind::EqualRadius | CKind::AnnularDistance
                if known(&g, c.args[0].ent()).is_some() =>
            {
                // absorbed into the known radii
            }
            // A run or a rise holds one *coordinate* of a pair against the page, and there is
            // no element here to name the line it is really measured from (one through the
            // first point, along a ground axis) — so it takes the numeric residue below on
            // purpose: an edge claiming "distance" would be a lie the merge ranks believe.
            _ => g.unsupported.push(c.id),
        }
    }
    // endpoints lie on their (registered) line — implicit incidences; the rest are passive
    let registered: Vec<usize> = g.lines.clone();
    for (li, &ln) in registered.iter().enumerate() {
        let l = &sk.lines[ln];
        for p in [l.p1 as usize, l.p2 as usize] {
            let a = g.point_el(p);
            g.edges.push(Edge {
                kind: EdgeKind::Pl,
                a,
                b: El::l(li),
                value: EdgeVal::Zero,
                source: None,
            });
        }
    }
    g.passive = (0..sk.lines.len()).filter(|&i| !g.has_line(i)).collect();
    g
}

/// Entity kinds that can carry a radius — the guard `build` uses for `circle_or_arc` slots.
pub fn is_round(e: EntRef) -> bool {
    matches!(e.kind, EntKind::Circle | EntKind::Arc)
}
