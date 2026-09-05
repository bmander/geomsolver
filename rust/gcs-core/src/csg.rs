//! **Boundary evaluation**: the surface of a term, found by classifying candidates against it.
//!
//! Requicha & Voelcker's route, and the reason there is no B-rep in this project.  The boundary
//! of `A ∪ B` or `A − B` is a *subset* of the boundaries of `A` and `B`, so nothing has to be
//! constructed: cut every primitive's facets by the planes of every other primitive that reaches
//! them, and ask of each piece whether the material is on one side of it.  Exactly one side
//! inside is a face of the solid; both or neither is interior or exterior, and is dropped.
//!
//! The same trick gives the *edges* a view draws, one dimension down: a candidate is a seam
//! between two facets or the meeting of two primitives' facets, and it is an edge of the solid
//! where the material around it forms a wedge rather than a slab or nothing.  A seam that is
//! merely tessellation — the flats of a bore's wall — is `smooth` and is drawn only where it is
//! a *silhouette*, which is what makes a cylinder two lines instead of sixty-four.
//!
//! Every walk here is culled by bounding box before it is paid for, and every container is
//! ordered, so the answer does not depend on how the drawing was written down.

use crate::plane;
use crate::solid::{Box3, Csg, Facet, Prim};

/// A face of the solid: a planar convex piece of some primitive's facet that survived
/// classification, with its outward normal and the path the document reaches it by.
#[derive(Clone, Debug)]
pub struct Piece {
    pub pts: Vec<[f64; 3]>,
    pub n: [f64; 3],
    /// `body.bore.wall` — the solid, the primitive, the drawn edge it was swept from.
    pub path: String,
    pub prim: usize,
    pub smooth: bool,
}

impl Piece {
    pub fn centroid(&self) -> [f64; 3] {
        let k = 1.0 / self.pts.len() as f64;
        let mut c = [0.0; 3];
        for p in &self.pts {
            for i in 0..3 {
                c[i] += p[i] * k;
            }
        }
        c
    }

    pub fn bbox(&self) -> Box3 {
        self.bbox_of()
    }

    pub fn bbox_of(&self) -> Box3 {
        let mut b = Box3::empty();
        for p in &self.pts {
            b.add(*p);
        }
        b
    }

    /// The unsigned area, evaluated in a local frame.
    pub fn area(&self) -> f64 {
        plane::norm(crate::solid::area_vector(&self.pts)) / 2.0
    }
}

/// An edge of the solid, as a view would draw it: the segment, the two surfaces meeting along
/// it, and whether that meeting is a corner of the design or a chord of a tessellation.
#[derive(Clone, Debug)]
pub struct Edge {
    pub a: [f64; 3],
    pub b: [f64; 3],
    pub na: [f64; 3],
    pub nb: [f64; 3],
    /// A tessellation seam: drawn only where it is a silhouette in the view being taken.
    pub smooth: bool,
    pub path: String,
}

/// The cutting planes of a primitive, deduped — many facets of one cap share one plane, and
/// splitting a piece by the same plane sixty times is sixty times the pieces.
fn planes_of(p: &Prim) -> Vec<([f64; 3], f64, Box3)> {
    let mut out: Vec<([f64; 3], f64, Box3)> = Vec::new();
    for f in &p.facets {
        let (n, d) = (f.n, f.offset());
        let b = f.bbox();
        if let Some(e) = out.iter_mut().find(|(m, o, _)| same_plane(*m, *o, n, d)) {
            e.2.add(b.lo);
            e.2.add(b.hi);
        } else {
            out.push((n, d, b));
        }
    }
    out
}

fn same_plane(n1: [f64; 3], d1: f64, n2: [f64; 3], d2: f64) -> bool {
    let dot = plane::dot(n1, n2);
    let par = (dot.abs() - 1.0).abs() < 1e-9;
    if !par {
        return false;
    }
    let d2 = if dot > 0.0 { d2 } else { -d2 };
    (d1 - d2).abs() < 1e-9 * (1.0 + d1.abs().max(d2.abs()))
}

/// `body.bore.wall`: the solid the primitive came from, and the drawn edge its facet was swept
/// from.  A path and never an index — a boolean never renames, which is why the naming problem
/// every history-based kernel has does not arise here.
fn path_of(prim: &Prim, f: &Facet) -> String {
    match prim.faces.get(f.face) {
        Some(n) => format!("{}.{}", prim.of, n),
        None => prim.of.clone(),
    }
}

// -- boundary evaluation, by binary space partition ---------------------------------------------
//
// The first version of this cut every facet by every plane of every primitive that reached it and
// classified the pieces.  That is Requicha & Voelcker exactly, and on a *square* hole it is exact
// — but the pieces go as the square of the facets: a block's cap cut by the six hundred wall
// planes of a faceted bore is the arrangement of six hundred lines, twenty-five thousand cells of
// which the drawing needs about six hundred.  Bounding boxes do not save it, because the piece
// left outside a chord still stretches across the whole cap.
//
// A BSP prunes exactly what that loop could not.  Descending the tree, a piece that lands wholly
// in front of a node's plane is *decided* and goes no further, so a cap against a cylinder costs
// pieces in proportion to the facets and not to their square.  It is the same set of planes and
// the same answer; what changes is that the walk stops asking once it knows.

/// A facet on its way through the tree, carrying where it came from.
#[derive(Clone, Debug)]
struct Poly {
    pts: Vec<[f64; 3]>,
    n: [f64; 3],
    w: f64,
    path: String,
    prim: usize,
    smooth: bool,
}

impl Poly {
    fn flipped(&self) -> Poly {
        let mut p = self.clone();
        p.pts.reverse();
        p.n = plane::scaled(p.n, -1.0);
        p.w = -p.w;
        p
    }
}

/// Where a vertex falls against a plane.  The tolerance is relative to the piece, so a facet far
/// from the origin is judged as sharply as one at it.
const FRONT: u8 = 1;
const BACK: u8 = 2;

fn cut(poly: &Poly, n: [f64; 3], w: f64, tol: f64) -> (Vec<Poly>, Vec<Poly>, Vec<Poly>, Vec<Poly>) {
    let (mut cf, mut cb, mut fr, mut bk) = (Vec::new(), Vec::new(), Vec::new(), Vec::new());
    let mut kind = 0u8;
    let types: Vec<u8> = poly
        .pts
        .iter()
        .map(|p| {
            let t = plane::dot(n, *p) - w;
            let k = if t < -tol {
                BACK
            } else if t > tol {
                FRONT
            } else {
                0
            };
            kind |= k;
            k
        })
        .collect();
    match kind {
        0 => {
            if plane::dot(n, poly.n) > 0.0 {
                cf.push(poly.clone())
            } else {
                cb.push(poly.clone())
            }
        }
        FRONT => fr.push(poly.clone()),
        BACK => bk.push(poly.clone()),
        _ => {
            let (mut f, mut b) = (Vec::new(), Vec::new());
            for i in 0..poly.pts.len() {
                let j = (i + 1) % poly.pts.len();
                let (ti, tj) = (types[i], types[j]);
                let (vi, vj) = (poly.pts[i], poly.pts[j]);
                if ti != BACK {
                    f.push(vi);
                }
                if ti != FRONT {
                    b.push(vi);
                }
                if (ti | tj) == (FRONT | BACK) {
                    let di = plane::dot(n, vi) - w;
                    let dj = plane::dot(n, vj) - w;
                    let t = di / (di - dj);
                    let x = [
                        vi[0] + t * (vj[0] - vi[0]),
                        vi[1] + t * (vj[1] - vi[1]),
                        vi[2] + t * (vj[2] - vi[2]),
                    ];
                    f.push(x);
                    b.push(x);
                }
            }
            if f.len() >= 3 {
                fr.push(Poly { pts: f, ..poly.clone() });
            }
            if b.len() >= 3 {
                bk.push(Poly { pts: b, ..poly.clone() });
            }
        }
    }
    (cf, cb, fr, bk)
}

#[derive(Debug, Default)]
struct Node {
    plane: Option<([f64; 3], f64)>,
    front: Option<Box<Node>>,
    back: Option<Box<Node>>,
    polys: Vec<Poly>,
}

impl Node {
    fn new(polys: Vec<Poly>, tol: f64) -> Node {
        let mut n = Node::default();
        n.build(polys, tol);
        n
    }

    fn build(&mut self, polys: Vec<Poly>, tol: f64) {
        if polys.is_empty() {
            return;
        }
        if self.plane.is_none() {
            self.plane = Some((polys[0].n, polys[0].w));
        }
        let (n, w) = self.plane.expect("just set");
        let (mut fr, mut bk) = (Vec::new(), Vec::new());
        for p in &polys {
            let (cf, cb, f, b) = cut(p, n, w, tol);
            self.polys.extend(cf);
            self.polys.extend(cb);
            fr.extend(f);
            bk.extend(b);
        }
        if !fr.is_empty() {
            self.front.get_or_insert_with(Default::default).build(fr, tol);
        }
        if !bk.is_empty() {
            self.back.get_or_insert_with(Default::default).build(bk, tol);
        }
    }

    /// The parts of `polys` that lie **outside** this solid.
    fn clip(&self, polys: Vec<Poly>, tol: f64) -> Vec<Poly> {
        let Some((n, w)) = self.plane else { return polys };
        let (mut fr, mut bk) = (Vec::new(), Vec::new());
        for p in &polys {
            let (cf, cb, f, b) = cut(p, n, w, tol);
            fr.extend(cf);
            fr.extend(f);
            bk.extend(cb);
            bk.extend(b);
        }
        let mut fr = match &self.front {
            Some(f) => f.clip(fr, tol),
            None => fr,
        };
        // nothing behind the deepest plane is outside the solid, so the walk stops there — the
        // pruning the flat loop could not do
        let bk = match &self.back {
            Some(b) => b.clip(bk, tol),
            None => Vec::new(),
        };
        fr.extend(bk);
        fr
    }

    fn clip_to(&mut self, other: &Node, tol: f64) {
        self.polys = other.clip(std::mem::take(&mut self.polys), tol);
        if let Some(f) = self.front.as_mut() {
            f.clip_to(other, tol);
        }
        if let Some(b) = self.back.as_mut() {
            b.clip_to(other, tol);
        }
    }

    fn invert(&mut self) {
        for p in self.polys.iter_mut() {
            *p = p.flipped();
        }
        if let Some((n, w)) = self.plane {
            self.plane = Some((plane::scaled(n, -1.0), -w));
        }
        if let Some(f) = self.front.as_mut() {
            f.invert();
        }
        if let Some(b) = self.back.as_mut() {
            b.invert();
        }
        std::mem::swap(&mut self.front, &mut self.back);
    }

    fn all(&self) -> Vec<Poly> {
        let mut v = self.polys.clone();
        if let Some(f) = &self.front {
            v.extend(f.all());
        }
        if let Some(b) = &self.back {
            v.extend(b.all());
        }
        v
    }
}

fn union(a: Vec<Poly>, b: Vec<Poly>, tol: f64) -> Vec<Poly> {
    if a.is_empty() {
        return b;
    }
    if b.is_empty() {
        return a;
    }
    let mut na = Node::new(a, tol);
    let mut nb = Node::new(b, tol);
    na.clip_to(&nb, tol);
    nb.clip_to(&na, tol);
    nb.invert();
    nb.clip_to(&na, tol);
    nb.invert();
    na.build(nb.all(), tol);
    na.all()
}

fn difference(a: Vec<Poly>, b: Vec<Poly>, tol: f64) -> Vec<Poly> {
    if a.is_empty() || b.is_empty() {
        return a;
    }
    let mut na = Node::new(a, tol);
    let mut nb = Node::new(b, tol);
    na.invert();
    na.clip_to(&nb, tol);
    nb.clip_to(&na, tol);
    nb.invert();
    nb.clip_to(&na, tol);
    nb.invert();
    na.build(nb.all(), tol);
    na.invert();
    na.all()
}

fn intersection(a: Vec<Poly>, b: Vec<Poly>, tol: f64) -> Vec<Poly> {
    if a.is_empty() || b.is_empty() {
        return Vec::new();
    }
    let mut na = Node::new(a, tol);
    let mut nb = Node::new(b, tol);
    na.invert();
    nb.clip_to(&na, tol);
    nb.invert();
    na.clip_to(&nb, tol);
    nb.clip_to(&na, tol);
    na.build(nb.all(), tol);
    na.invert();
    na.all()
}

/// Common material, evaluated in one local frame. Boundary sampling cannot find all crossings.
pub(crate) fn common_boundary(a: &Csg, b: &Csg, eps: f64) -> Vec<Piece> {
    if !a.bbox().overlaps(&b.bbox()) {
        return Vec::new();
    }
    let origin = a
        .prims
        .iter()
        .flat_map(|p| &p.facets)
        .find_map(|f| f.pts.first())
        .copied()
        .unwrap_or([0.0; 3]);
    let tol = eps * 1e-3;
    intersection(polys_of(a, &a.term, tol, origin), polys_of(b, &b.term, tol, origin), tol)
        .into_iter()
        .filter(|p| p.pts.len() >= 3)
        .map(|p| Piece {
            pts: p.pts.iter().map(|v| std::array::from_fn(|k| v[k] + origin[k])).collect(),
            n: p.n,
            path: p.path,
            prim: p.prim,
            smooth: p.smooth,
        })
        .collect()
}

fn polys_of(csg: &Csg, t: &crate::solid::Term, tol: f64, origin: [f64; 3]) -> Vec<Poly> {
    use crate::solid::Term;
    match t {
        Term::Empty => Vec::new(),
        Term::Prim(i) => {
            let prim = &csg.prims[*i];
            prim
                .facets
                .iter()
                .map(|f| {
                    let pts: Vec<_> = f
                        .pts
                        .iter()
                        .map(|p| [p[0] - origin[0], p[1] - origin[1], p[2] - origin[2]])
                        .collect();
                    Poly {
                        w: plane::dot(f.n, pts[0]),
                        pts,
                        n: f.n,
                        path: path_of(prim, f),
                        prim: *i,
                        smooth: f.smooth,
                    }
                })
                .collect()
        }
        Term::Union(a, b) => {
            union(polys_of(csg, a, tol, origin), polys_of(csg, b, tol, origin), tol)
        }
        Term::Diff(a, b) => {
            difference(polys_of(csg, a, tol, origin), polys_of(csg, b, tol, origin), tol)
        }
    }
}

/// **The boundary of a term**: the faces of the solid, each carrying the path the document
/// reaches it by.
pub fn boundary(csg: &Csg, eps: f64) -> Vec<Piece> {
    // Split in a local frame. At large world coordinates even a facet's own vertices can
    // fall off its rounded plane, so a BSP node repeatedly splits its first polygon forever.
    let origin = csg.prims.iter().flat_map(|p| &p.facets)
        .find_map(|f| f.pts.first()).copied().unwrap_or([0.0; 3]);
    let polys = polys_of(csg, &csg.term, eps * 1e-3, origin);
    let mut out: Vec<Piece> = polys
        .into_iter()
        .filter(|p| p.pts.len() >= 3)
        .map(|p| Piece {
            pts: p
                .pts
                .into_iter()
                .map(|v| [v[0] + origin[0], v[1] + origin[1], v[2] + origin[2]])
                .collect(),
            n: p.n,
            path: p.path,
            prim: p.prim,
            smooth: p.smooth,
        })
        .collect();
    // a sliver with no area is no face: the cut leaves them wherever a plane grazes a corner,
    // and they carry no volume, no ink and no name anyone would ask for
    let big = out.iter().map(|p| p.area()).fold(0.0f64, f64::max);
    out.retain(|p| p.area() > big * 1e-12);
    out
}

/// Containment is the emptiness of A − B, including portions of B's voids enclosed by A.
/// Sampling only A's exterior misses both enclosed cavities and unsampled boundary crossings.
pub(crate) fn contains_boundary(b: &Csg, a: &[Piece], eps: f64) -> bool {
    let origin = a.iter().find_map(|p| p.pts.first()).copied().unwrap_or([0.0; 3]);
    let ap = a
        .iter()
        .map(|p| {
            let pts: Vec<_> = p
                .pts
                .iter()
                .map(|v| [v[0] - origin[0], v[1] - origin[1], v[2] - origin[2]])
                .collect();
            Poly {
                w: plane::dot(p.n, pts[0]),
                pts,
                n: p.n,
                path: p.path.clone(),
                prim: p.prim,
                smooth: p.smooth,
            }
        })
        .collect();
    let tol = eps * 1e-3;
    let remainder = difference(ap, polys_of(b, &b.term, tol, origin), tol);
    !remainder.iter().any(|p| plane::norm(crate::solid::area_vector(&p.pts)) > tol * tol)
}

// -- the edges a view draws ---------------------------------------------------------------------

/// How many directions the material around a candidate edge is sampled in.  Twelve is enough to
/// tell a wedge from a slab and cheap enough to pay per candidate.
const RING: usize = 12;

/// **The edges of a term.**  Candidates are the seams inside each primitive and the meetings of
/// facets from different ones; each is cut where any other primitive's plane crosses it, and a
/// piece survives where the material around it forms a wedge.
pub fn edges(csg: &Csg, eps: f64) -> Vec<Edge> {
    let planes: Vec<Vec<([f64; 3], f64, Box3)>> = csg.prims.iter().map(planes_of).collect();
    let mut cand: Vec<Edge> = Vec::new();
    for (i, prim) in csg.prims.iter().enumerate() {
        seams(prim, &mut cand);
        for (j, other) in csg.prims.iter().enumerate() {
            if j <= i || !prim.bbox.overlaps(&other.bbox) {
                continue;
            }
            crossings(prim, other, &mut cand);
        }
    }
    let mut out = Vec::new();
    for e in cand {
        let mut cuts: Vec<f64> = vec![0.0, 1.0];
        let d = [e.b[0] - e.a[0], e.b[1] - e.a[1], e.b[2] - e.a[2]];
        let len = plane::norm(d);
        if len <= 0.0 {
            continue;
        }
        let eb = {
            let mut b = Box3::empty();
            b.add(e.a);
            b.add(e.b);
            b
        };
        for (j, other) in csg.prims.iter().enumerate() {
            let _ = j;
            if !other.bbox.grown(eps).overlaps(&eb) {
                continue;
            }
            for (n, dd, pb) in &planes[j] {
                if !pb.grown(eps).overlaps(&eb) {
                    continue;
                }
                let denom = plane::dot(*n, d);
                if denom.abs() < 1e-12 {
                    continue;
                }
                let t = (dd - plane::dot(*n, e.a)) / denom;
                if t > 1e-9 && t < 1.0 - 1e-9 {
                    cuts.push(t);
                }
            }
        }
        cuts.sort_by(|a, b| a.partial_cmp(b).unwrap());
        for w in cuts.windows(2) {
            if w[1] - w[0] < 1e-9 {
                continue;
            }
            let at = |t: f64| [e.a[0] + t * d[0], e.a[1] + t * d[1], e.a[2] + t * d[2]];
            let m = at((w[0] + w[1]) / 2.0);
            if !on_boundary(csg, m, d, eps) {
                continue;
            }
            out.push(Edge { a: at(w[0]), b: at(w[1]), ..e.clone() });
        }
    }
    out
}

/// Is the material around this point a *wedge* — a corner of the solid — rather than a slab, a
/// solid block or empty space?  Sampled on a ring perpendicular to the edge, counting the
/// changes: two or more transitions is a surface passing through, none is interior or exterior.
fn on_boundary(csg: &Csg, m: [f64; 3], along: [f64; 3], eps: f64) -> bool {
    let Some(w) = plane::unit(along) else { return false };
    let helper = if w[0].abs() < 0.9 { [1.0, 0.0, 0.0] } else { [0.0, 1.0, 0.0] };
    let Some(p) = plane::unit(plane::cross(w, helper)) else { return false };
    let q = plane::cross(w, p);
    let mut ins = [false; RING];
    for (k, slot) in ins.iter_mut().enumerate() {
        let a = std::f64::consts::TAU * k as f64 / RING as f64;
        let (s, c) = a.sin_cos();
        *slot = csg.inside([
            m[0] + eps * (c * p[0] + s * q[0]),
            m[1] + eps * (c * p[1] + s * q[1]),
            m[2] + eps * (c * p[2] + s * q[2]),
        ]);
    }
    let changes = (0..RING).filter(|k| ins[*k] != ins[(k + 1) % RING]).count();
    changes >= 2
}

/// The seams inside one primitive: every segment two of its facets share.
fn seams(prim: &Prim, out: &mut Vec<Edge>) {
    let mut map: std::collections::BTreeMap<([i64; 3], [i64; 3]), (usize, usize)> =
        std::collections::BTreeMap::new();
    let scale = {
        let b = &prim.bbox;
        (0..3).fold(1.0f64, |m, i| m.max((b.hi[i] - b.lo[i]).abs())).max(1.0)
    };
    let key = |p: [f64; 3]| {
        let g = scale * 1e-9;
        [(p[0] / g).round() as i64, (p[1] / g).round() as i64, (p[2] / g).round() as i64]
    };
    for (fi, f) in prim.facets.iter().enumerate() {
        for i in 0..f.pts.len() {
            let a = f.pts[i];
            let b = f.pts[(i + 1) % f.pts.len()];
            let (ka, kb) = (key(a), key(b));
            let k = if ka <= kb { (ka, kb) } else { (kb, ka) };
            match map.get_mut(&k) {
                Some(slot) => slot.1 = fi,
                None => {
                    map.insert(k, (fi, usize::MAX));
                }
            }
        }
    }
    for (k, (f1, f2)) in map {
        if f2 == usize::MAX {
            continue;
        }
        let g = scale * 1e-9;
        let a = [k.0[0] as f64 * g, k.0[1] as f64 * g, k.0[2] as f64 * g];
        let b = [k.1[0] as f64 * g, k.1[1] as f64 * g, k.1[2] as f64 * g];
        let (na, nb) = (prim.facets[f1].n, prim.facets[f2].n);
        // a seam whose two facets face the same way is a chord of a tessellation and not a
        // corner, and it is smooth if *either* facet says its sweep was one
        let flat = plane::dot(na, nb) > 0.999_999;
        let smooth = flat || (prim.facets[f1].smooth && prim.facets[f2].smooth);
        out.push(Edge { a, b, na, nb, smooth, path: path_of(prim, &prim.facets[f1]) });
    }
}

/// Where two primitives' facets meet: the intersection line of their planes, clipped to both.
fn crossings(p: &Prim, q: &Prim, out: &mut Vec<Edge>) {
    for f in &p.facets {
        let fb = f.bbox();
        if !q.bbox.overlaps(&fb) {
            continue;
        }
        for g in &q.facets {
            let gb = g.bbox();
            if !gb.overlaps(&fb) {
                continue;
            }
            let dir = plane::cross(f.n, g.n);
            let Some(dir) = plane::unit(dir) else { continue };
            // a point on both planes: solve the 3×3 with the direction as the third row
            let Some(x0) = meet(f.n, f.offset(), g.n, g.offset(), dir) else { continue };
            let Some((t0, t1)) = clip(&f.pts, f.n, x0, dir) else { continue };
            let Some((s0, s1)) = clip(&g.pts, g.n, x0, dir) else { continue };
            let (lo, hi) = (t0.max(s0), t1.min(s1));
            if hi - lo < 1e-9 {
                continue;
            }
            let at = |t: f64| [x0[0] + t * dir[0], x0[1] + t * dir[1], x0[2] + t * dir[2]];
            out.push(Edge {
                a: at(lo),
                b: at(hi),
                na: f.n,
                nb: g.n,
                smooth: false,
                path: path_of(p, f),
            });
        }
    }
}

/// A point on both planes, nearest the origin along the shared direction.
fn meet(n1: [f64; 3], d1: f64, n2: [f64; 3], d2: f64, dir: [f64; 3]) -> Option<[f64; 3]> {
    let m = [n1, n2, dir];
    let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    if det.abs() < 1e-12 {
        return None;
    }
    let rhs = [d1, d2, 0.0];
    let mut x = [0.0; 3];
    for c in 0..3 {
        let mut a = m;
        for r in 0..3 {
            a[r][c] = rhs[r];
        }
        let d = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
            - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
            + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
        x[c] = d / det;
    }
    Some(x)
}

/// The stretch of the line `x0 + t·dir` that lies inside a convex facet.
fn clip(pts: &[[f64; 3]], n: [f64; 3], x0: [f64; 3], dir: [f64; 3]) -> Option<(f64, f64)> {
    let (mut lo, mut hi) = (f64::NEG_INFINITY, f64::INFINITY);
    for i in 0..pts.len() {
        let a = pts[i];
        let b = pts[(i + 1) % pts.len()];
        let e = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        // inward normal of this side, in the facet's plane
        let ni = plane::cross(n, e);
        let denom = plane::dot(ni, dir);
        let num = plane::dot(ni, [x0[0] - a[0], x0[1] - a[1], x0[2] - a[2]]);
        if denom.abs() < 1e-15 {
            if num < -1e-12 * plane::norm(ni).max(1.0) {
                return None;
            }
            continue;
        }
        let t = -num / denom;
        if denom > 0.0 {
            lo = lo.max(t);
        } else {
            hi = hi.min(t);
        }
    }
    (hi > lo).then_some((lo, hi))
}
