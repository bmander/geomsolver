//! **A body's faces, read off its exact boundary** (#183 F1, F7): the stratum's B-rep located
//! (`BodyFaces`), each face named by its path, with the geometry the field's readings, the set
//! distances and the lowering table ask of it; and what a region term reads of it (`Reading`).
use crate::brep::geom::V;
use crate::brep::query::{Located, Place};
use crate::brep::topo::EdgeCurve;
use crate::model::{FaceRef, Sketch};
use crate::fmath::Det;
use crate::space::{add, distance, dot, lerp, norm, scale, sub};
use std::collections::BTreeSet;
use std::f64::consts::PI;
use super::{cached, readers_of, without, FacePath};

/// Two faces' outward normals nearer than this at a point meet tangent there: no edge.
const TANGENT: f64 = 1e-6;

/// A point just along a run from its end `v`, `d` the way it runs from there (`by` −1: just past
/// the end).
pub(crate) fn along_run(v: V, d: V, by: f64) -> V { add(v, scale(d, by * 1e-6 * (1.0 + norm(v)))) }

/// What a run of edges ends on (`BodyFaces::end_faces`): a face of the boundary, or a plane
/// square to an edge where its run ends on none, its unit normal `out` pointing past that end.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum EndBy {
    Face(usize),
    Square { at: V, out: V },
}

/// A body's exact boundary, located, in millimetres (`mm` of them a model unit).
#[derive(Debug)]
pub struct BodyFaces {
    loc: Located<'static>,
    mm: f64,
    /// Where the boundary is built about (model units): a point of space is `(p − origin)·mm`.
    origin: V,
    /// Each face's name as the body names it, beside `loc.b.faces`.
    paths: Vec<FacePath>,
    /// Each edge's material angle at its middle, where it bounds two faces: under a half turn
    /// where the material is the faces' sides' common part (convex).
    convex: Vec<Option<bool>>,
    /// Each face's edges, once each.
    edges: Vec<Vec<u32>>,
    /// The faces each edge bounds.
    bounding: Vec<Vec<usize>>,
    /// A box holding every face (millimetres, about `origin`): its low corner and its high.
    reach: (V, V),
    /// Each face's shell, and how many: the faces reached from it across their edges numbered
    /// alike, from 0 in the order the faces come — a body in several pieces is several shells.
    shells: (Vec<usize>, usize),
}

impl BodyFaces {
    /// Solid `body`'s faces, as made by the terms that do not read them.
    pub fn read(sk: &Sketch, body: usize) -> Result<BodyFaces, String> {
        let readers = readers_of(sk, body);
        if readers.is_empty() {
            return BodyFaces::built(sk, body, false);
        }
        BodyFaces::built(&without(sk, &readers)?, body, false)
    }

    /// Solid `body`'s faces as the drawing finishes it, the terms that read them made too (its
    /// rounds): what an analysis reads (#510 P7), where `read` is what those terms read.  A body
    /// nothing reads the faces of is the stratum `cached` already holds; any other is remembered
    /// in `Caches::whole`, its B-rep the drawing's own (`Sketch::exact_solid`).
    pub fn whole(sk: &Sketch, body: usize) -> Result<std::sync::Arc<BodyFaces>, String> {
        if readers_of(sk, body).is_empty() {
            return cached(sk, body);
        }
        let key = crate::solid::MemoKey::of(sk, body);
        let build = || BodyFaces::built(sk, body, true).map(std::sync::Arc::new);
        sk.caches.whole.get_or(body, key, build)
    }

    /// Solid `body`'s faces in `from`, every term of it made: its B-rep `from`'s remembered one
    /// where `remembered`, else built for this reading alone (a stratum's).
    fn built(from: &Sketch, body: usize, remembered: bool) -> Result<BodyFaces, String> {
        let name = from.solids.get(body).ok_or("no such solid")?.name.clone();
        // each face named by its document path (`pocket.near`), as `solid::exact` names them —
        // a swept body's by its blank's primitives and the cutter's faces its cuts left (#212)
        let swept = crate::solid::cad::contains_sweep(from, body);
        let ex = if swept {
            crate::solid::exact::swept(from, body)
        } else if remembered {
            from.exact_preimage(body).map(|e| (*e).clone())
        } else {
            crate::solid::exact::build(from, body)
        }.map_err(|e| format!("`{name}`'s faces: {e}"))?;
        let routes = crate::solid::document::operand_paths(from, body);
        let paths = ex.named.iter().map(|(of, face)| {
            FacePath::new(routes.get(of).map_or(of.as_str(), String::as_str), face)
        }).collect();
        Ok(BodyFaces::of(ex, paths))
    }

    /// An exact boundary's faces, each named by its path beside it (none: a boundary read whole,
    /// `field::BuiltField`).
    pub fn of(ex: crate::solid::Exact, paths: Vec<FacePath>) -> BodyFaces {
        let mut faces = BodyFaces::located(ex, paths);
        faces.convex = (0..faces.loc.b.edges.len()).map(|e| faces.edge_convex(e as u32)).collect();
        faces
    }

    /// A boundary's faces read with no names: what a construction builds from (a shell's body and
    /// its openings' prisms glued, `solid::shelled`), millimetres about `origin` as `of` reads one.
    pub fn unnamed(brep: crate::brep::topo::Brep, mm: f64, origin: V) -> BodyFaces {
        let named = vec![(String::new(), String::new()); brep.faces.len()];
        let paths = vec![FacePath::new("", ""); brep.faces.len()];
        BodyFaces::of(crate::solid::Exact { brep, mm, origin, leading: BTreeSet::new(), named }, paths)
    }

    /// `of` but each edge's convexity, left for the caller to read.
    fn located(ex: crate::solid::Exact, paths: Vec<FacePath>) -> BodyFaces {
        let tol = 1e-9 * ex.brep.size().max(1.0);
        let (mm, origin) = (ex.mm, ex.origin);
        let loc = Located::owned(ex.brep, tol);
        let edges: Vec<Vec<u32>> = loc.b.faces.iter().map(|f| {
            let set: BTreeSet<u32> = f.loops.iter().flatten().map(|c| c.edge).collect();
            set.into_iter().collect()
        }).collect();
        let mut bounding = vec![Vec::new(); loc.b.edges.len()];
        for (fi, es) in edges.iter().enumerate() {
            for &e in es {
                bounding[e as usize].push(fi);
            }
        }
        let mut reach = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
        for fi in 0..loc.b.faces.len() {
            let (lo, hi) = loc.face_box(fi);
            for k in 0..3 {
                reach.0[k] = reach.0[k].min(lo[k]);
                reach.1[k] = reach.1[k].max(hi[k]);
            }
        }
        let mut joined = crate::graph::UnionFind::new(edges.len());
        for by in &bounding {
            for w in by.windows(2) {
                joined.union(w[0], w[1]);
            }
        }
        let shells = joined.labels();
        BodyFaces { loc, mm, origin, paths, convex: Vec::new(), edges, bounding, reach, shells }
    }

    /// The same boundary bounding what lies outside it: every face's outward side turned, so each
    /// convex edge is concave and each concave one convex — what lies within reach of what is
    /// outside a body, its shell's walls, is what lies within reach of these (`solid::shelled`).
    pub fn inverted(&self) -> BodyFaces {
        let mut brep = self.loc.b.clone().into_owned();
        for f in &mut brep.faces {
            f.reversed = !f.reversed;
        }
        let named = vec![(String::new(), String::new()); brep.faces.len()];
        let ex = crate::solid::Exact { brep, mm: self.mm, origin: self.origin, leading: BTreeSet::new(), named };
        // each edge's convexity the other way round, never classified again
        let mut out = BodyFaces::located(ex, self.paths.clone());
        out.convex = self.convex.iter().map(|c| c.map(|x| !x)).collect();
        out
    }

    /// Each face's name as the body names it, beside its boundary's faces (`brep`).
    pub fn paths(&self) -> &[FacePath] { &self.paths }

    /// The boundary the faces are read off: in millimetres about `origin` (model units), `mm` of
    /// them a model unit.
    pub fn brep(&self) -> (&crate::brep::topo::Brep, f64, V) { (&self.loc.b, self.mm, self.origin) }

    /// A box about the boundary (model units): its low corner and its high.
    pub fn bounds(&self) -> (V, V) {
        let (lo, hi) = self.loc.b.bounds();
        (self.back(lo), self.back(hi))
    }

    /// A box holding every face, its bulges included (model units): its low corner and its high.
    pub fn reach(&self) -> (V, V) { (self.back(self.reach.0), self.back(self.reach.1)) }

    /// How near a face a point is read as on it (model units): the boundary's tolerance.
    pub fn tolerance(&self) -> f64 { self.loc.tol / self.mm }

    /// The signed distance from the boundary (model units, positive outside) and the face it is
    /// nearest.  The nearest point is a face's foot on its surface, where that lies on the face —
    /// signed by the face's side — else an edge's nearest point: outside a convex edge and inside
    /// a concave one (a ball within the material cannot touch a convex edge, nor one outside it a
    /// concave edge), and by the boundary's ray classification where the edge is no corner or the
    /// point is at its end (outside beyond every face's box).  Exact where each face's surface
    /// foot is (`Surface::inverse`'s global foot: a plane, a cylinder, a sphere, a torus; a cone
    /// along one generator).
    pub fn signed_distance(&self, p: V) -> (f64, usize) {
        let (v, fi, _) = self.nearest_point(p);
        (v, fi)
    }

    /// **Whether `signed_distance` reads the distance itself, and how far off it may be near
    /// which edges**: every face a plane, a cylinder, a sphere or a torus, whose foot is the
    /// nearest point (`Surface::inverse`), and every edge a line, a circle or a pole, whose nearest
    /// point is exact, or a traced intersection, found by its nearest chord and then Newton — off
    /// the curve's nearest by twice its chords' sag at most, taken twice over with its tolerance
    /// (model units).  Per traced edge that slack; `None` where any face or edge is read by a foot
    /// that need not be the nearest.
    pub fn distance_slack(&self) -> Option<Vec<(u32, f64)>> {
        use crate::brep::geom::{Curve, Surface};
        let b = &self.loc.b;
        let faces = b.faces.iter().all(|f| matches!(f.surface,
            Surface::Plane(..) | Surface::Cylinder(..) | Surface::Sphere(..) | Surface::Torus(..)));
        if !faces {
            return None;
        }
        let mut out = Vec::new();
        for (k, e) in b.edges.iter().enumerate() {
            match &e.curve {
                EdgeCurve::Degenerate => {}
                EdgeCurve::Curve(Curve::Line { .. } | Curve::Circle(..)) => {}
                EdgeCurve::Curve(c @ Curve::Traced(t)) => {
                    let sag = t.pts.windows(2).enumerate().map(|(i, w)| {
                        distance(c.point(i as f64 + 0.5), lerp(w[0], w[1], 0.5))
                    }).fold(0.0, f64::max);
                    out.push((k as u32, (4.0 * sag + 2.0 * e.tol) / self.mm));
                }
                EdgeCurve::Curve(_) => return None,
            }
        }
        Some(out)
    }

    /// How much `signed_distance` may read over the distance at `p`, having read `v` there: the
    /// slack of each traced edge (`distance_slack`) whose box is within reach of `p`, else none.
    pub fn slack_at(&self, slack: &[(u32, f64)], p: V, v: f64) -> f64 {
        let q = self.q(p);
        slack.iter().filter(|&&(e, s)| self.loc.edge_gap(e, q) <= (v.abs() + s) * self.mm)
            .map(|&(_, s)| s).fold(0.0, f64::max)
    }

    /// `signed_distance` and its gradient: the unit vector from the nearest point of the boundary
    /// toward `p`, turned to point the way the distance grows (the outward normal of the face
    /// nearest where `p` is on the boundary) — so a refinement places a crossing by Newton on
    /// readings rather than by bisection on signs.
    pub fn reading(&self, p: V) -> (f64, V) {
        let (v, fi, near) = self.nearest_point(p);
        let q = self.q(p);
        let d = sub(q, near);
        let n = norm(d);
        let g = if n > 0.0 && v != 0.0 {
            scale(d, v.signum() / n)
        } else {
            self.outward(fi, q)
        };
        (v, g)
    }

    /// `signed_distance`, and the nearest point of the boundary (millimetres, about `origin`).
    /// Only the faces whose boxes come within reach are read: the box nearest `q` gives a
    /// distance reached (its face's foot where that is on the face, else its nearest edge), and
    /// a face, or an edge of one, whose box is farther than that is no nearer.  What is chosen
    /// is what reading every face would choose: the nearest foot on its face, first by index
    /// among equals, and an edge only where strictly nearer than that, first by index.
    fn nearest_point(&self, p: V) -> (f64, usize, V) {
        use crate::brep::geom::Surface;
        let q = self.q(p);
        let b = &self.loc.b;
        let n = b.faces.len();
        if n == 0 {
            return (f64::INFINITY, 0, q);
        }
        // how far each face's box is: no point of the face or its edges is nearer
        let reach: Vec<f64> = (0..n).map(|fi| self.loc.face_gap(fi, q) - self.loc.tol).collect();
        let foot = |fi: usize| {
            let f = &b.faces[fi];
            let at = f.surface.point(f.surface.inverse(q));
            (distance(q, at), at)
        };
        // a distance reached: the nearest box's face, or where its foot is off it, its edges'
        // nearest points, kept for the edges' turn below
        let first = (0..n).min_by(|&a, &c| reach[a].total_cmp(&reach[c])).unwrap_or(0);
        let (d0, at0) = foot(first);
        let place0 = self.loc.face_place(first, at0);
        let read: Vec<(u32, (f64, f64))> = if place0 != Place::Out {
            Vec::new()
        } else {
            self.edges[first].iter().map(|&e| (e, self.loc.nearest(e, q))).collect()
        };
        let reached = if place0 != Place::Out {
            d0
        } else {
            read.iter().map(|r| r.1 .1).fold(f64::INFINITY, f64::min)
        };
        let nearest = |e: u32| -> (f64, f64) {
            read.iter().find(|r| r.0 == e).map_or_else(|| self.loc.nearest(e, q), |r| r.1)
        };
        // each face whose box is within reach, its surface foot, nearest first
        let mut feet: Vec<(f64, usize, V)> = (0..n).filter(|&fi| reach[fi] <= reached).map(|fi| {
            let (d, at) = if fi == first { (d0, at0) } else { foot(fi) };
            (d, fi, at)
        }).collect();
        feet.sort_by(|a, c| a.0.total_cmp(&c.0).then(a.1.cmp(&c.1)));
        let face = feet.iter().take_while(|f| f.0 <= reached).find(|&&(_, fi, at)| {
            if fi == first { place0 != Place::Out } else { self.loc.face_place(fi, at) != Place::Out }
        }).copied();
        // an edge lies on the surfaces of the faces it bounds, so is no nearer than their feet
        // (but for a cone's, whose foot is on one generator), nor than their boxes
        let mut least = reach;
        for &(d, fi, _) in &feet {
            if !matches!(b.faces[fi].surface, Surface::Cone(..)) {
                least[fi] = least[fi].max(d);
            }
        }
        let mut near: Vec<u32> = feet.iter().flat_map(|f| self.edges[f.1].iter().copied()).collect();
        near.sort_unstable();
        near.dedup();
        let mut edge: Option<(u32, f64, f64)> = None;
        for e in near {
            let below = self.bounding[e as usize].iter().map(|&fi| least[fi]).fold(0.0, f64::max);
            let best = edge.map_or(face.map_or(f64::INFINITY, |f| f.0), |(.., d)| d);
            if below >= best {
                continue;
            }
            let (t, d) = nearest(e);
            if d < best {
                edge = Some((e, t, d));
            }
        }
        let Some((e, t, d)) = edge else {
            let Some((d, fi, at)) = face else { return (f64::INFINITY, 0, q) };
            return (self.side(fi, p).signum() * d / self.mm, fi, at);
        };
        let span = b.edges[e as usize].t;
        let sign = match self.convex(e) {
            Some(convex) if t > span[0] && t < span[1] => if convex { 1.0 } else { -1.0 },
            _ if (0..3).any(|k| q[k] < self.reach.0[k] || q[k] > self.reach.1[k]) => 1.0,
            _ => match self.loc.solid_place(q) {
                Place::Out => 1.0,
                Place::In => -1.0,
                Place::On => 0.0,
            },
        };
        let v = sign * d / self.mm;
        // the face the edge bounds whose surface reads nearest the distance
        let off = |fi: usize| (self.side(fi, p) - v).abs();
        let fi = self.faces_of(e).iter().copied().min_by(|&a, &c| off(a).total_cmp(&off(c))).unwrap_or(0);
        (v, fi, b.edges[e as usize].point(t, &b.vertices))
    }

    /// The faces a reference names, by its path (`FacePath::matches`).
    pub fn faces(&self, f: &FaceRef) -> Vec<usize> {
        (0..self.paths.len()).filter(|&i| self.paths[i].matches(&f.path)).collect()
    }

    /// A point of the model's space in the boundary's (millimetres, about its origin).
    pub fn q(&self, p: V) -> V { scale(sub(p, self.origin), self.mm) }

    /// A point of the boundary's space back in the model's.
    pub fn back(&self, q: V) -> V { add(scale(q, 1.0 / self.mm), self.origin) }

    /// Edge `e`'s point a fraction `s` along its parameter (model units).
    pub fn along(&self, e: u32, s: f64) -> V {
        let edge = &self.loc.b.edges[e as usize];
        self.back(edge.point(edge.t[0] + s * (edge.t[1] - edge.t[0]), &self.loc.b.vertices))
    }

    /// Edge `e`'s middle, by its parameter (model units).
    pub fn middle(&self, e: u32) -> V {
        let edge = &self.loc.b.edges[e as usize];
        self.back(edge.point(0.5 * (edge.t[0] + edge.t[1]), &self.loc.b.vertices))
    }

    /// Face `fi`'s outward unit normal at `q` (millimetres), pointing away from the material.
    fn outward(&self, fi: usize, q: V) -> V {
        let f = &self.loc.b.faces[fi];
        let g = f.surface.gradient(q);
        if f.reversed { scale(g, -1.0) } else { g }
    }

    /// The signed distance from face `fi`'s surface, positive away from the material (model
    /// units).
    pub fn side(&self, fi: usize, p: V) -> f64 {
        let f = &self.loc.b.faces[fi];
        let s = f.surface.implicit(self.q(p)) / self.mm;
        if f.reversed { -s } else { s }
    }

    /// The distance from face `fi`'s patch (model units): its surface's foot where that is on
    /// the face, else the nearest point of its edges.
    pub fn patch_distance(&self, fi: usize, p: V) -> f64 {
        let q = self.q(p);
        let f = &self.loc.b.faces[fi];
        let foot = f.surface.point(f.surface.inverse(q));
        if self.loc.face_place(fi, foot) != Place::Out {
            return distance(q, foot) / self.mm;
        }
        self.edges[fi].iter().map(|&e| self.loc.nearest(e, q).1).fold(f64::INFINITY, f64::min)
            / self.mm
    }

    /// Where `p`'s foot on face `fi`'s surface lies off the face: the edge of the face nearest it
    /// (`None` where it lies on the face).
    pub fn off_face(&self, fi: usize, p: V) -> Option<u32> {
        let q = self.q(p);
        let f = &self.loc.b.faces[fi];
        let foot = f.surface.point(f.surface.inverse(q));
        if self.loc.face_place(fi, foot) != Place::Out {
            return None;
        }
        self.edges[fi].iter().map(|&e| (self.loc.nearest(e, foot).1, e))
            .min_by(|a, b| a.0.total_cmp(&b.0)).map(|(_, e)| e)
    }

    /// The edges two groups of faces share: where a face of `a` meets a face of `b`.
    pub fn edges_between(&self, a: &[usize], b: &[usize]) -> Vec<u32> {
        let of = |fs: &[usize]| -> BTreeSet<u32> {
            fs.iter().flat_map(|&fi| self.edges[fi].iter().copied()).collect()
        };
        let (ea, eb) = (of(a), of(b));
        ea.intersection(&eb).copied()
            .filter(|&e| matches!(self.loc.b.edges[e as usize].curve, EdgeCurve::Curve(_)))
            .collect()
    }

    /// The point of edges `es` nearest `p`: its edge, its place (millimetres), and its distance
    /// (model units).
    pub fn nearest_edge(&self, es: &[u32], p: V) -> Option<(u32, V, f64)> {
        let q = self.q(p);
        es.iter().map(|&e| {
            let (t, d) = self.loc.nearest(e, q);
            let at = self.loc.b.edges[e as usize].point(t, &self.loc.b.vertices);
            (e, at, d / self.mm)
        }).min_by(|a, b| a.2.total_cmp(&b.2))
    }

    /// Face `fi`'s edges, once each.
    pub fn face_edges(&self, fi: usize) -> &[u32] { &self.edges[fi] }

    /// The faces edge `e` bounds.
    pub fn faces_of(&self, e: u32) -> &[usize] { &self.bounding[e as usize] }

    /// The edges bounding face `fi`, once each.
    pub fn edges_of(&self, fi: usize) -> &[u32] { &self.edges[fi] }

    /// Each face's shell, and how many there are (`shells`).
    pub fn shells(&self) -> (&[usize], usize) { (&self.shells.0, self.shells.1) }

    /// Edge `e`'s vertices, first to last.
    pub fn ends(&self, e: u32) -> [u32; 2] { self.loc.b.edges[e as usize].v }

    /// Whether edge `e` is convex: a point beside its middle within the first face's side and
    /// outside the second's is outside the material where the material is both sides' common
    /// part.  `None` for a seam or an edge two faces meeting tangent bound.
    fn edge_convex(&self, e: u32) -> Option<bool> {
        let [fa, fb] = *self.faces_of(e) else { return None };
        let edge = &self.loc.b.edges[e as usize];
        let x = edge.point(0.5 * (edge.t[0] + edge.t[1]), &self.loc.b.vertices);
        let (na, nb) = (self.outward(fa, x), self.outward(fb, x));
        let across = norm(sub(nb, na));
        if across < TANGENT {
            return None;
        }
        let step = 1e-5 * self.loc.b.size().max(1.0);
        let y = add(x, scale(sub(nb, na), step / across));
        match self.loc.solid_place(y) {
            Place::In => Some(false),
            Place::Out => Some(true),
            Place::On => None,
        }
    }

    /// Whether edge `e` is convex, where it is a corner.
    pub fn convex(&self, e: u32) -> Option<bool> { self.convex.get(e as usize).copied().flatten() }

    /// The material's angle at the point of edges `es` nearest `p` (radians), or `None` where
    /// there is no edge or it is no corner.
    pub fn angle(&self, es: &[u32], p: V) -> Option<f64> {
        let (e, x, _) = self.nearest_edge(es, p)?;
        let convex = self.convex[e as usize]?;
        let [fa, fb] = *self.faces_of(e) else { return None };
        let turn = dot(self.outward(fa, x), self.outward(fb, x)).clamp(-1.0, 1.0).dacos();
        Some(if convex { PI - turn } else { PI + turn })
    }

    /// Face `fi`'s side and its gradient at `p` (model units; the gradient a unit vector).
    pub fn side_grad(&self, fi: usize, p: V) -> (f64, V) {
        let f = &self.loc.b.faces[fi];
        let q = self.q(p);
        let (s, g) = (f.surface.implicit(q) / self.mm, f.surface.gradient(q));
        if f.reversed { (-s, scale(g, -1.0)) } else { (s, g) }
    }

    /// The vertices three groups of faces share: where a face of each meets, with the faces of
    /// each group standing there (model units).
    pub fn vertices_among(&self, groups: &[&[usize]]) -> Vec<(V, Vec<Vec<usize>>)> {
        let at = |fi: usize| -> BTreeSet<u32> {
            self.edges[fi].iter().flat_map(|&e| self.loc.b.edges[e as usize].v).collect()
        };
        let of = |fs: &[usize]| -> BTreeSet<u32> { fs.iter().flat_map(|&fi| at(fi)).collect() };
        let mut common = of(groups[0]);
        for g in &groups[1..] {
            common = common.intersection(&of(g)).copied().collect();
        }
        common.into_iter().map(|v| {
            let faces = groups.iter().map(|g| g.iter().copied().filter(|&fi| at(fi).contains(&v)).collect()).collect();
            (self.back(self.loc.b.vertices[v as usize].p), faces)
        }).collect()
    }

    /// Edges `es` in runs: each run the edges joined end to end through the vertices they share.
    pub fn runs(&self, es: &[u32]) -> Vec<Vec<u32>> {
        let mut out: Vec<Vec<u32>> = Vec::new();
        for &e in es {
            let ends = self.loc.b.edges[e as usize].v;
            let touching: Vec<usize> = (0..out.len()).filter(|&k| out[k].iter()
                .any(|&f| self.loc.b.edges[f as usize].v.iter().any(|v| ends.contains(v)))).collect();
            let mut run = vec![e];
            for &k in touching.iter().rev() {
                run.extend(out.remove(k));
            }
            out.push(run);
        }
        out
    }

    /// Where a run of edges ends: each vertex only one of `es` reaches (model units), with the
    /// unit direction the edge runs from it.  A closed run has none.
    pub fn run_ends(&self, es: &[u32]) -> Vec<(V, V, u32)> {
        let mut count: std::collections::BTreeMap<u32, Vec<(u32, usize)>> = Default::default();
        for &e in es {
            if self.closed(e) {
                continue;
            }
            let edge = &self.loc.b.edges[e as usize];
            for (k, v) in edge.v.iter().enumerate() {
                count.entry(*v).or_default().push((e, k));
            }
        }
        count.into_iter().filter(|(_, at)| at.len() == 1).filter_map(|(v, at)| {
            let (e, k) = at[0];
            let edge = &self.loc.b.edges[e as usize];
            let EdgeCurve::Curve(c) = &edge.curve else { return None };
            let t = edge.t[k];
            let mut d = c.tangent(t);
            if k == 1 {
                d = scale(d, -1.0);
            }
            let d = crate::space::normalised(d)?;
            Some((self.back(self.loc.b.vertices[v as usize].p), d, v))
        }).collect()
    }

    /// Whether face `c` carries one of `faces` on across a seam at `v` (model units): their
    /// outward normals there one, so they meet tangent and the edge between them is none.
    fn continues(&self, c: usize, faces: &[usize], v: V) -> bool {
        let q = self.q(v);
        let n = self.outward(c, q);
        faces.iter().any(|&f| norm(sub(self.outward(f, q), n)) < TANGENT)
    }

    /// Face `fi`'s plane, where it lies in one: its unit normal away from the material and its
    /// frame's origin (model units).
    pub fn plane(&self, fi: usize) -> Option<(V, V)> {
        let f = &self.loc.b.faces[fi];
        let crate::brep::geom::Surface::Plane(frame) = &f.surface else { return None };
        Some((if f.reversed { scale(frame.z, -1.0) } else { frame.z }, self.back(frame.o)))
    }

    /// Face `fi`'s cylinder, where it lies on one: a point of its axis, the axis's unit direction
    /// and its radius (model units).
    pub fn cylinder(&self, fi: usize) -> Option<(V, V, f64)> {
        let crate::brep::geom::Surface::Cylinder(f, r) = &self.loc.b.faces[fi].surface else { return None };
        Some((self.back(f.o), f.z, r / self.mm))
    }

    /// Face `fi`'s cone, where it lies on one: a point of its axis, the axis's unit direction,
    /// its radius at that point (model units) and its half-angle, the radius growing along the
    /// axis by its tangent (a drafted circle's side, #245).
    pub fn cone(&self, fi: usize) -> Option<(V, V, f64, f64)> {
        let crate::brep::geom::Surface::Cone(f, r, a) = &self.loc.b.faces[fi].surface else { return None };
        Some((self.back(f.o), f.z, r / self.mm, *a))
    }

    /// Face `fi`'s sphere, where it lies on one: its centre and radius (model units).
    pub fn sphere(&self, fi: usize) -> Option<(V, f64)> {
        let crate::brep::geom::Surface::Sphere(f, r) = &self.loc.b.faces[fi].surface else { return None };
        Some((self.back(f.o), r / self.mm))
    }

    /// Face `fi`'s torus, where it lies on one: a point of its axis, the axis's unit direction, the
    /// radius of the circle its tube's centre runs round and the tube's (model units).
    pub fn torus(&self, fi: usize) -> Option<(V, V, f64, f64)> {
        let crate::brep::geom::Surface::Torus(f, big, r) = &self.loc.b.faces[fi].surface else { return None };
        Some((self.back(f.o), f.z, big / self.mm, r / self.mm))
    }

    /// Where face `fi`'s side reaches `v` (model units), its surface's level set read whole: the
    /// points whose side is at most `v` are within the solid `Bound` names, or outside it — the
    /// material's side of a plane, a cylinder, a sphere, a cone or a torus, grown or shrunk by `v`
    /// along its normal.  `None` for any other surface, or a level no such solid is (a cone at its
    /// apex, a torus's tube reaching its axis).
    pub fn level(&self, fi: usize, v: f64) -> Option<Level> {
        let reversed = self.loc.b.faces[fi].reversed;
        // the material within the surface where it is not reversed: its side is the surface's
        // own signed distance, positive outside; so the level is the surface grown by `v`, or the
        // surface reversed shrunk by it
        let grown = |r: f64| if reversed { r - v } else { r + v };
        let made = |b: Bound, r: f64| -> Level {
            if r > 0.0 { Level::Bound(b, !reversed) }
            else if reversed { Level::Everywhere } else { Level::Nowhere }
        };
        if let Some((n, o)) = self.plane(fi) {
            return Some(Level::Bound(Bound::Half { n, d: dot(n, o) + v }, true));
        }
        if let Some((a, d, r)) = self.cylinder(fi) {
            let r = grown(r);
            return Some(made(Bound::Cylinder { a, d, r }, r));
        }
        if let Some((c, r)) = self.sphere(fi) {
            let r = grown(r);
            return Some(made(Bound::Ball { c, r }, r));
        }
        if let Some((c, d, big, r)) = self.torus(fi) {
            let r = grown(r);
            return (r < big).then(|| made(Bound::Torus { c, d, big, r }, r));
        }
        if let Some((o, d, r, a)) = self.cone(fi) {
            // the radius at `o` grows along `d` by tan a; at the level, by `v / cos a` more
            let (sa, ca) = a.dsin_cos();
            if !(ca > 1e-9 && sa.abs() > 1e-9) {
                return None;
            }
            let r = if reversed { r - v / ca } else { r + v / ca };
            // the apex, where the radius is nothing, and the way the cone opens from it
            let apex = add(o, scale(d, -r * ca / sa));
            let (d, half) = if a > 0.0 { (d, a) } else { (scale(d, -1.0), -a) };
            return Some(Level::Bound(Bound::Cone { a: apex, d, half }, !reversed));
        }
        None
    }

    /// Edge `e`'s ends, first to last, where it is a straight line (model units).
    pub fn segment(&self, e: u32) -> Option<[V; 2]> {
        let edge = &self.loc.b.edges[e as usize];
        let EdgeCurve::Curve(crate::brep::geom::Curve::Line { .. }) = &edge.curve else { return None };
        Some(edge.v.map(|v| self.back(self.loc.b.vertices[v as usize].p)))
    }

    /// Edge `e`'s circle, where it is one (a whole turn or a stretch): its centre, unit normal and
    /// radius, and the unit direction from its centre its parameter starts at (model units).
    pub fn circle(&self, e: u32) -> Option<(V, V, f64, V)> {
        let edge = &self.loc.b.edges[e as usize];
        let EdgeCurve::Curve(crate::brep::geom::Curve::Circle(f, r)) = &edge.curve else { return None };
        Some((self.back(f.o), f.z, r / self.mm, f.x))
    }

    /// Edge `e`'s parameters, first to last: a circle's angles from the direction `circle` gives,
    /// turning about its normal.
    pub fn span(&self, e: u32) -> [f64; 2] { self.loc.b.edges[e as usize].t }

    /// Whether edge `e` is closed: one vertex at both ends.
    pub fn closed(&self, e: u32) -> bool {
        let v = self.loc.b.edges[e as usize].v;
        v[0] == v[1]
    }

    /// Face `fi`'s unit normal at `p` (on it), away from the material.
    pub fn normal(&self, fi: usize, p: V) -> V { self.side_grad(fi, p).1 }

    /// Whether face `fi`'s surface is turned about the line through `o` along the unit `d`: a plane
    /// square to it, a cylinder, a cone or a torus about it, a sphere centred on it (model units).
    pub fn coaxial(&self, fi: usize, o: V, d: V) -> bool {
        use crate::brep::geom::Surface;
        let along = |z: V| norm(crate::space::cross(z, d)) <= 1e-9;
        let on = |p: V| norm(crate::space::cross(sub(self.back(p), o), d)) <= 1e-9 * (1.0 + norm(o));
        match &self.loc.b.faces[fi].surface {
            Surface::Plane(f) => along(f.z),
            Surface::Cylinder(f, _) | Surface::Cone(f, _, _) | Surface::Torus(f, _, _) => along(f.z) && on(f.o),
            Surface::Sphere(f, _) => on(f.o),
            _ => false,
        }
    }

    /// Where a run of edges `es` (between `faces`) ends: each other face at each of its ends, and
    /// which side of that face's surface the run runs on — `1` where it runs within it (its
    /// material side, `side <= 0`), `-1` outside — so the run is where `σ·side <= 0` for all.
    /// A face going on from one of `faces` tangent at the end (a seam, no corner: an obround's
    /// half circle after its flat side) ends nothing: the round goes on round it, along the edge
    /// that face meets the other in.  Nor does a face whose surface the run itself goes across, nor
    /// a plane turning back over its section at a reflex corner (`ends_at`).  And a run carried
    /// past an end along the edge the face it ends on makes with one of `faces` (a run-out) goes no
    /// further than that edge: where that edge's run ends further on, on the side it runs on, so
    /// does the run.  Each with the end it was read at where it stands elsewhere: none for a face
    /// at an end of the run, that end's point (model units) for where the edge a run-out there
    /// runs along ends.
    pub fn end_faces(&self, es: &[u32], faces: &[usize]) -> Vec<(EndBy, f64, Option<V>)> {
        self.end_faces_as(es, faces, es.iter().all(|&e| self.carried(e)))
    }

    /// `end_faces`, the rules of a run's own ends (`goes_across`, a reflex corner's plane, where a
    /// run-out ends) read only where the lowering builds its runs and rings by them, every edge of
    /// the set `plain` (`carried`): a ball rolled along a traced run is cut where it ends by the
    /// kernel's own reading of the faces there (`fillet::end_at`).
    fn end_faces_as(&self, es: &[u32], faces: &[usize], plain: bool)
        -> Vec<(EndBy, f64, Option<V>)> {
        let mut sides: Vec<(EndBy, f64, Option<V>)> = Vec::new();
        let runs = self.runs(es);
        let touches = |run: &[u32], vid: u32| {
            run.iter().any(|&e| self.loc.b.edges[e as usize].v.contains(&vid))
        };
        for (v, d, vid) in self.run_ends(es) {
            let Some(run) = runs.iter().find(|run| touches(run, vid)) else { continue };
            let at = self.ends_at(run, faces, v, d, vid, plain);
            // a run that ends on no face — its faces going on across seams there, tangent into
            // another or into a piece of the same surface, or every face there one it goes across
            // — ends square there (#962): what a corner's edge reads as its end, where nothing
            // else bounds it, so a ball at one end of an edge rounds no further than its other
            // (a run's own common part ends there already).  A straight run: a plane square to an
            // arc is a half-space through its axis, which would cut a stretch of more than half a
            // turn short of its own other end
            if at.is_empty() && plain && run.iter().all(|&e| self.segment(e).is_some()) {
                sides.push((EndBy::Square { at: v, out: scale(d, -1.0) }, 1.0, None));
            }
            for (c, sign) in at {
                sides.push((EndBy::Face(c), sign, None));
                // (a plane: one curved is carried into only as far as its section takes)
                if !plain || self.plane(c).is_none() {
                    continue;
                }
                // the straight edges `c` makes with the run's faces going on from this end the way
                // the run is carried past it: where each one's run ends further on, so does the run
                // carried along it — within the faces it ends on there, or square to it where it
                // ends on none (a plate's side all but in line with the next, and the side past it
                // turning back in: carried no further than the plate's corner there)
                for &f in faces {
                    for other in self.runs(&self.edges_between(&[c], &[f])) {
                        if other.iter().any(|&e| self.segment(e).is_none()) {
                            continue;
                        }
                        let ends = self.run_ends(&other);
                        if !ends.iter().any(|&(_, way, w)| w == vid && dot(way, d) < -1e-6) {
                            continue;
                        }
                        for &(u, way, wid) in ends.iter().filter(|&&(_, _, w)| w != vid) {
                            let at = self.ends_at(&other, &[c, f], u, way, wid, true);
                            let by: Vec<(EndBy, f64)> = if at.is_empty() {
                                vec![(EndBy::Square { at: u, out: scale(way, -1.0) }, 1.0)]
                            } else {
                                at.into_iter().filter(|&(x, _)| !faces.contains(&x) && x != c)
                                    .map(|(x, sx)| (EndBy::Face(x), sx)).collect()
                            };
                            for (x, sx) in by {
                                let had = sides.iter().any(|&(y, sy, _)| y == x && sy == sx);
                                if had || self.end_across(&x, sx, run) {
                                    continue;
                                }
                                sides.push((x, sx, Some(v)));
                            }
                        }
                    }
                }
            }
        }
        sides
    }

    /// The signed distance from what a run ends on, positive away from the material (a square
    /// end: past it), and its gradient (model units).
    pub fn end_side(&self, by: &EndBy, p: V) -> (f64, V) {
        match *by {
            EndBy::Face(c) => self.side_grad(c, p),
            EndBy::Square { at, out } => (dot(out, sub(p, at)), out),
        }
    }

    /// Whether some point of edges `es` stands past what a run ends on, on the side `sign` keeps
    /// (`goes_across`; a square end's plane at the edges' ends).
    fn end_across(&self, by: &EndBy, sign: f64, es: &[u32]) -> bool {
        match *by {
            EndBy::Face(c) => self.goes_across(c, sign, es),
            EndBy::Square { .. } => {
                es.iter().flat_map(|&e| [self.along(e, 0.0), self.along(e, 1.0)])
                    .any(|p| sign * self.end_side(by, p).0 > self.past(p))
            }
        }
    }

    /// The faces run `run` (between `faces`) ends on at its end `v` (vertex `vid`, `d` the way it
    /// runs from there), each with the side of it the run runs on: every other face there, but
    /// one going on from one of `faces` tangent across a seam (a seam is no edge, and the run
    /// beyond it is the next one's), and one whose surface the run itself goes across away from
    /// this end — no side of it holds the run (a plane all but along a lug's rim where a plate's
    /// side runs into it, the arc dipping under the plane and back) — nor a plane the run stands
    /// outside of turning back over the run's faces (a reflex corner of less than a right angle),
    /// whose half-space reaches back over the section from the end, the more the shallower the
    /// turn (a whole side's length at a few degrees): past the end it cuts all of the run, and
    /// alongside nothing is past the end.  The run ends there square, as at a seam (each where
    /// `plain`, `end_faces_as`).
    fn ends_at(&self, run: &[u32], faces: &[usize], v: V, d: V, vid: u32, plain: bool)
        -> Vec<(usize, f64)> {
        let on = along_run(v, d, 1.0);
        let back = |c: usize| self.plane(c)
            .is_some_and(|(n, _)| faces.iter().any(|&f| dot(n, self.normal(f, v)) > 1e-9));
        let ends = self.faces_at(vid, faces).into_iter().filter(|&c| !self.continues(c, faces, v));
        ends.filter_map(|c| {
            let sign = if self.side(c, on) <= 0.0 { 1.0 } else { -1.0 };
            let across = plain && (self.goes_across(c, sign, run) || sign < 0.0 && back(c));
            (!across).then_some((c, sign))
        }).collect()
    }

    /// Whether edge `e` is one the lowering carries a section along (`lowering::piece::run`): a
    /// straight edge between planes and cylinders along it, or a circle every face of which is
    /// turned about its axis; any other a ball is rolled along, traced.
    pub fn carried(&self, e: u32) -> bool {
        match self.segment(e) {
            Some([p0, p1]) => {
                let d = crate::space::normalised(sub(p1, p0)).unwrap_or([0.0; 3]);
                let along = |axis: V| norm(crate::space::cross(axis, d)) <= 1e-9;
                self.faces_of(e).iter().all(|&f| self.plane(f).is_some()
                    || self.cylinder(f).is_some_and(|(_, axis, _)| along(axis)))
            }
            None => self.circle(e).is_some_and(|(centre, axis, ..)| {
                self.faces_of(e).iter().all(|&f| self.coaxial(f, centre, axis))
            }),
        }
    }

    /// How far past a face a point at `p` stands before it is past it: ten times the boundary's
    /// tolerance, or a billionth of its distance from the origin where that is more.
    fn past(&self, p: V) -> f64 {
        (10.0 * self.tolerance()).max(1e-9 * (1.0 + norm(p)))
    }

    /// Whether some point of edges `es` stands outside face `c`'s surface on the side `sign`
    /// keeps (`sign · side > 0`), by more than the boundary's tolerance: exactly for a plane
    /// against a line or a circle, at 64 points of an edge otherwise.
    fn goes_across(&self, c: usize, sign: f64, es: &[u32]) -> bool {
        let out = |p: V| sign * self.side(c, p) > self.past(p);
        es.iter().any(|&e| {
            let edge = &self.loc.b.edges[e as usize];
            match (self.plane(c), &edge.curve) {
                (Some(_), EdgeCurve::Curve(crate::brep::geom::Curve::Line { .. })) => {
                    out(self.along(e, 0.0)) || out(self.along(e, 1.0))
                }
                (Some((n, _)), EdgeCurve::Curve(crate::brep::geom::Curve::Circle(f, _))) => {
                    // `side` along a circle is `side(centre) + r (a cos t + b sin t)`: at its
                    // ends, and where it is greatest the way `sign` reads it, if within them
                    let (t0, t1) = (edge.t[0].min(edge.t[1]), edge.t[0].max(edge.t[1]));
                    let peak = (sign * dot(n, f.y)).datan2(sign * dot(n, f.x));
                    let turn = (peak - t0).rem_euclid(2.0 * PI);
                    let at = |t: f64| self.back(edge.point(t, &self.loc.b.vertices));
                    out(at(t0)) || out(at(t1)) || (t0 + turn <= t1 && out(at(t0 + turn)))
                }
                _ => (0..=64).any(|k| out(self.along(e, k as f64 / 64.0))),
            }
        })
    }

    /// Edges `es` (between `faces`) in runs (`runs`), each with where it ends (`end_faces`): a
    /// crease a slot cuts across ends on the slot's sides once for each run (#881), and the ends
    /// of runs read as one run's are within none of them.
    pub fn end_runs(&self, es: &[u32], faces: &[usize]) -> Vec<(Vec<u32>, Vec<(EndBy, f64)>)> {
        let plain = es.iter().all(|&e| self.carried(e));
        self.runs(es).into_iter().map(|run| {
            let sides = self.end_faces_as(&run, faces, plain).into_iter()
                .map(|(c, sign, _)| (c, sign)).collect();
            (run, sides)
        }).collect()
    }

    /// `faces` and every face going on from one of them tangent across a seam, and from those
    /// on (`F.smooth`, [0.149]): an edge two of the boundary's faces bound, where their outward
    /// normals are one at its middle, is no edge but a seam.  In the faces' order, then as met.
    pub fn smooth(&self, faces: &[usize]) -> Vec<usize> {
        let mut out = faces.to_vec();
        let mut k = 0;
        while k < out.len() {
            let fi = out[k];
            for &e in &self.edges[fi] {
                let [x, y] = *self.faces_of(e) else { continue };
                let other = if x == fi { y } else { x };
                if other != fi && !out.contains(&other) && self.seam(e) {
                    out.push(other);
                }
            }
            k += 1;
        }
        out
    }

    /// Whether edge `e` is a seam: two faces bound it whose outward normals are one at its middle,
    /// so they meet tangent there and it is no edge.
    pub fn seam(&self, e: u32) -> bool {
        let [x, y] = *self.faces_of(e) else { return false };
        let edge = &self.loc.b.edges[e as usize];
        let at = edge.point(0.5 * (edge.t[0] + edge.t[1]), &self.loc.b.vertices);
        norm(sub(self.outward(x, at), self.outward(y, at))) < TANGENT
    }

    /// **A vertex where edges of both turns meet** (#962): a vertex of the boundary within `tol`
    /// (model units) of one of `near`, where its edges (seams aside) are convex and concave both —
    /// where it stands, and each of those edges by the two faces it lies between and whether it is
    /// convex.
    pub fn mixed_vertex(&self, near: &[V], tol: f64) -> Option<(V, Vec<([usize; 2], bool)>)> {
        (0..self.loc.b.vertices.len() as u32).find_map(|v| {
            let at = self.back(self.loc.b.vertices[v as usize].p);
            near.iter().any(|&p| norm(sub(p, at)) <= tol).then_some(())?;
            let edges: Vec<([usize; 2], bool)> = (0..self.loc.b.edges.len() as u32)
                .filter(|&e| self.loc.b.edges[e as usize].v.contains(&v) && !self.seam(e))
                .filter_map(|e| match (self.faces_of(e), self.convex(e)) {
                    (&[a, b], Some(c)) => Some(([a, b], c)),
                    _ => None,
                }).collect();
            (edges.iter().any(|e| e.1) && edges.iter().any(|e| !e.1)).then_some((at, edges))
        })
    }

    /// The faces at vertex `v` other than `not`.
    pub fn faces_at(&self, v: u32, not: &[usize]) -> Vec<usize> {
        (0..self.edges.len()).filter(|fi| {
            !not.contains(fi) && self.edges[*fi].iter().any(|&e| self.loc.b.edges[e as usize].v.contains(&v))
        }).collect()
    }
}

/// What a level of a face's side bounds (`BodyFaces::level`): the solid the points at most that
/// far from its surface are within, or outside.
#[derive(Clone, Copy, Debug)]
pub enum Level {
    /// Within the solid (`true`) or outside it (`false`).
    Bound(Bound, bool),
    /// Every point: a reversed surface shrunk to nothing.
    Everywhere,
    /// No point: a surface shrunk to nothing.
    Nowhere,
}

/// A solid a face's surface bounds, read whole (model units).
#[derive(Clone, Copy, Debug)]
pub enum Bound {
    /// The half-space `n · x ≤ d`, `n` a unit normal.
    Half { n: V, d: f64 },
    /// The ball of radius `r` about `c`.
    Ball { c: V, r: f64 },
    /// The solid cylinder of radius `r` about the line through `a` along the unit `d`.
    Cylinder { a: V, d: V, r: f64 },
    /// The solid cone with its apex at `a`, opening along the unit `d`, `half` off it.
    Cone { a: V, d: V, half: f64 },
    /// The solid torus about the line through `c` along the unit `d`: its tube of radius `r` about
    /// the circle of radius `big` about that line through `c`.
    Torus { c: V, d: V, big: f64, r: f64 },
}

/// The bodies a reading has read, each once.
pub type Read = Vec<(u32, std::sync::Arc<BodyFaces>)>;

/// What a region term of a body's faces reads: the faces each reference names, resolved once —
/// for `F.smooth`, carried on across their tangent seams (`BodyFaces::smooth`), and a distance
/// from their surfaces read off the nearest piece's.
pub struct Reading {
    pub body: std::sync::Arc<BodyFaces>,
    pub faces: Vec<usize>,
    pub smooth: bool,
}

/// The faces `f` names, read off its solid (refused where it names none).
pub fn reading(sk: &Sketch, f: &FaceRef, read: &mut Read)
    -> Result<Reading, String> {
    let body = match read.iter().find(|(s, _)| *s == f.solid) {
        Some((_, b)) => b.clone(),
        None => {
            let b = cached(sk, f.solid as usize)?;
            read.push((f.solid, b.clone()));
            b
        }
    };
    // a face a boolean ate, or none of that name, is no point: what reads it holds nowhere
    let faces = if f.smooth { body.smooth(&body.faces(f)) } else { body.faces(f) };
    Ok(Reading { body, faces, smooth: f.smooth })
}

impl Reading {
    /// The signed distance from the surface of the face whose patch is nearest `p`.
    pub fn side(&self, p: V) -> f64 {
        self.nearest(p).map_or(f64::INFINITY, |fi| self.body.side(fi, p))
    }
    /// The face whose patch is nearest `p` (one face is, measuring nothing).
    fn nearest(&self, p: V) -> Option<usize> {
        if let [fi] = self.faces[..] {
            return Some(fi);
        }
        self.faces.iter().map(|&fi| (self.body.patch_distance(fi, p), fi))
            .min_by(|a, b| a.0.total_cmp(&b.0)).map(|(_, fi)| fi)
    }
    /// The distance from the faces' patches, or (`surface`) their surfaces — a smooth surface's
    /// the one of the piece whose patch is nearest, so it follows the faces across their seams
    /// and runs on past their edges as the nearest piece's surface does.
    pub fn distance(&self, p: V, surface: bool) -> f64 {
        if self.smooth && surface {
            return self.side(p).abs();
        }
        self.faces.iter().map(|&fi| {
            if surface { self.body.side(fi, p).abs() } else { self.body.patch_distance(fi, p) }
        }).fold(f64::INFINITY, f64::min)
    }

    /// Where the faces' side reaches `v`, read surface by surface (`BodyFaces::level`): the one
    /// surface they lie on, or, at the surfaces themselves, each of several their convex creases
    /// join (`convex_pieces`: the points within all of them are the faces' own; `within_only`
    /// says the level is wanted only as a bound kept within, as only then does that hold).
    /// `None` where a surface is none `level` reads.
    pub fn levels(&self, v: f64, within_only: bool) -> Option<Vec<Level>> {
        let (pieces, _) = self.surfaces();
        let pieces = match pieces.len() {
            0 => return None,
            1 => pieces,
            _ if within_only && v == 0.0 => self.convex_pieces()?,
            _ => return None,
        };
        pieces.iter().map(|&fi| self.body.level(fi, v)).collect()
    }

    /// Where the faces are one plane: its outward normal `n` and `d`, the side `n · x - d`
    /// (model units).
    pub fn half_space(&self) -> Option<(V, f64)> {
        let [fi] = self.faces[..] else { return None };
        self.body.plane(fi).map(|(n, o)| (n, dot(n, o)))
    }

    /// The side of the face whose patch is nearest `p`, and its gradient.
    pub fn side_grad(&self, p: V) -> (f64, V) {
        self.nearest(p).map_or((f64::INFINITY, [0.0, 0.0, 1.0]), |fi| self.body.side_grad(fi, p))
    }

    /// The surfaces the faces lie on, each by the first face on it (the same way out), and which of
    /// those each face is on, in the faces' order.
    pub fn surfaces(&self) -> (Vec<usize>, Vec<usize>) {
        let b = &self.body.loc.b;
        let mut pieces: Vec<usize> = Vec::new();
        let group = self.faces.iter().map(|&fi| {
            let (f, tol) = (&b.faces[fi], self.body.loc.tol);
            pieces.iter().position(|&q| b.faces[q].reversed == f.reversed
                && crate::brep::ssi::same(&b.faces[q].surface, &f.surface, tol))
                .unwrap_or_else(|| { pieces.push(fi); pieces.len() - 1 })
        }).collect();
        (pieces, group)
    }

    /// Where the faces lie on more than one surface and every crease between those is convex (a
    /// drafted side's pieces either side of its parting plane, #245): one face on each surface.
    /// Within such faces the material is within every surface's side, and a point within them at
    /// least `d` off them is at least `d` off each surface, so either condition holds surface by
    /// surface — where the nearest piece's surface would turn from one to the other across the
    /// crease, and a projection onto it with it. `None` where the faces are one surface's, meet
    /// otherwise, or lie on several shells (within pieces apart is within any one, never all).
    pub fn convex_pieces(&self) -> Option<Vec<usize>> {
        let (shell, _) = self.body.shells();
        if self.faces.first().is_some_and(|&f0| self.faces.iter().any(|&fi| shell[fi] != shell[f0])) {
            return None;
        }
        let (pieces, group) = self.surfaces();
        if pieces.len() < 2 {
            return None;
        }
        // every edge between two of the faces on different surfaces, convex
        let group_of = |fi: usize| self.faces.iter().position(|&g| g == fi).map(|k| group[k]);
        for (k, &fi) in self.faces.iter().enumerate() {
            for &e in &self.body.edges[fi] {
                let across = self.body.faces_of(e).iter().filter_map(|&o| group_of(o)).any(|g| g != group[k]);
                if across && self.body.convex(e) != Some(true) {
                    return None;
                }
            }
        }
        Some(pieces)
    }
}
