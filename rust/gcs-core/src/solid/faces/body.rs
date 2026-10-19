//! **A body's faces, read off its exact boundary** (#183 F1, F7): the stratum's B-rep located
//! (`BodyFaces`), each face named by its path, with the geometry the field's readings, the set
//! distances and the lowering table ask of it; and what a region term reads of it (`Reading`).
use crate::brep::geom::V;
use crate::brep::query::{Located, Place};
use crate::brep::topo::EdgeCurve;
use crate::model::{FaceRef, Sketch};
use crate::fmath::Det;
use crate::space::{add, distance, dot, norm, scale, sub};
use std::collections::BTreeSet;
use std::f64::consts::PI;
use super::{cached, readers_of, without, FacePath};

/// Two faces' outward normals nearer than this at a point meet tangent there: no edge.
const TANGENT: f64 = 1e-6;

/// A point just along a run from its end `v`, `d` the way it runs from there (`by` −1: just past
/// the end).
pub(crate) fn along_run(v: V, d: V, by: f64) -> V { add(v, scale(d, by * 1e-6 * (1.0 + norm(v)))) }

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
}

impl BodyFaces {
    /// Solid `body`'s faces, as made by the terms that do not read them.
    pub fn read(sk: &Sketch, body: usize) -> Result<BodyFaces, String> {
        let readers = readers_of(sk, body);
        let name = sk.solids.get(body).ok_or("no such solid")?.name.clone();
        let stratum;
        let from = if readers.is_empty() {
            sk
        } else {
            stratum = without(sk, &readers)?;
            &stratum
        };
        // each face named by its document path (`pocket.near`), as `solid::exact` names them —
        // a swept body's by its blank's primitives and the cutter's faces its cuts left (#212)
        let swept = crate::solid::cad::contains_sweep(from, body);
        let ex = if swept { crate::solid::exact::swept(from, body) } else { crate::solid::exact::build(from, body) }
            .map_err(|e| format!("`{name}`'s faces: {e}"))?;
        let routes = crate::solid::document::operand_paths(from, body);
        let paths = ex.named.iter().map(|(of, face)| {
            FacePath::new(routes.get(of).map_or(of.as_str(), String::as_str), face)
        }).collect();
        Ok(BodyFaces::of(ex, paths))
    }

    /// An exact boundary's faces, each named by its path beside it (none: a boundary read whole,
    /// `field::BuiltField`).
    pub fn of(ex: crate::solid::Exact, paths: Vec<FacePath>) -> BodyFaces {
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
        let mut faces = BodyFaces { loc, mm, origin, paths, convex: Vec::new(), edges, bounding, reach };
        faces.convex = (0..faces.loc.b.edges.len()).map(|e| faces.edge_convex(e as u32)).collect();
        faces
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
        use crate::brep::geom::Surface;
        let q = self.q(p);
        let b = &self.loc.b;
        // each face's surface foot, nearest first: the first on its face is the nearest foot
        let mut feet: Vec<(f64, usize, V)> = b.faces.iter().enumerate().map(|(fi, f)| {
            let foot = f.surface.point(f.surface.inverse(q));
            (distance(q, foot), fi, foot)
        }).collect();
        feet.sort_by(|a, c| a.0.total_cmp(&c.0));
        let face = feet.iter().find(|&&(_, fi, foot)| self.loc.face_place(fi, foot) != Place::Out)
            .map(|&(d, fi, _)| (d, fi));
        // an edge lies on the surfaces of the faces it bounds, so is no nearer than their feet
        // (but for a cone's, whose foot is on one generator)
        let mut least = vec![0.0; b.faces.len()];
        for &(d, fi, _) in &feet {
            if !matches!(b.faces[fi].surface, Surface::Cone(..)) {
                least[fi] = d;
            }
        }
        let mut edge: Option<(u32, f64, f64)> = None;
        for e in 0..b.edges.len() {
            let below = self.bounding[e].iter().map(|&fi| least[fi]).fold(0.0, f64::max);
            let best = edge.map_or(face.map_or(f64::INFINITY, |f| f.0), |(.., d)| d);
            if below >= best {
                continue;
            }
            let (t, d) = self.loc.nearest(e as u32, q);
            if d < best {
                edge = Some((e as u32, t, d));
            }
        }
        let Some((e, t, d)) = edge else {
            let Some((d, fi)) = face else { return (f64::INFINITY, 0) };
            return (self.side(fi, p).signum() * d / self.mm, fi);
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
        (v, fi)
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

    /// The faces edge `e` bounds.
    pub fn faces_of(&self, e: u32) -> &[usize] { &self.bounding[e as usize] }

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
    /// that face meets the other in.
    pub fn end_sides(&self, es: &[u32], faces: &[usize]) -> Vec<(usize, f64)> {
        let mut sides = Vec::new();
        for (v, d, vid) in self.run_ends(es) {
            // a face carrying one of the run's on across a seam is no face the run ends on: a seam
            // is no edge, and the run beyond it is the next one's
            let on = along_run(v, d, 1.0);
            for c in self.faces_at(vid, faces).into_iter().filter(|&c| !self.continues(c, faces, v)) {
                let s = self.side(c, on);
                sides.push((c, if s <= 0.0 { 1.0 } else { -1.0 }));
            }
        }
        sides
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
                if other == fi || out.contains(&other) {
                    continue;
                }
                let edge = &self.loc.b.edges[e as usize];
                let at = edge.point(0.5 * (edge.t[0] + edge.t[1]), &self.loc.b.vertices);
                if norm(sub(self.outward(fi, at), self.outward(other, at))) < TANGENT {
                    out.push(other);
                }
            }
            k += 1;
        }
        out
    }

    /// The faces at vertex `v` other than `not`.
    pub fn faces_at(&self, v: u32, not: &[usize]) -> Vec<usize> {
        (0..self.edges.len()).filter(|fi| {
            !not.contains(fi) && self.edges[*fi].iter().any(|&e| self.loc.b.edges[e as usize].v.contains(&v))
        }).collect()
    }
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

    /// Where the faces lie on more than one surface and every crease between those is convex (a
    /// drafted side's pieces either side of its parting plane, #245): one face on each surface.
    /// Within such faces the material is within every surface's side, and a point within them at
    /// least `d` off them is at least `d` off each surface, so either condition holds surface by
    /// surface — where the nearest piece's surface would turn from one to the other across the
    /// crease, and a projection onto it with it. `None` where the faces are one surface's, or
    /// meet otherwise.
    pub fn convex_pieces(&self) -> Option<Vec<usize>> {
        let b = &self.body.loc.b;
        // each face's surface, by the first face on it
        let mut pieces: Vec<usize> = Vec::new();
        let group: Vec<usize> = self.faces.iter().map(|&fi| {
            let (f, tol) = (&b.faces[fi], self.body.loc.tol);
            pieces.iter().position(|&q| b.faces[q].reversed == f.reversed
                && crate::brep::ssi::same(&b.faces[q].surface, &f.surface, tol))
                .unwrap_or_else(|| { pieces.push(fi); pieces.len() - 1 })
        }).collect();
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
