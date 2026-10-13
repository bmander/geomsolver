//! **The faces of a body, read as sets** (#183 F1, F7, F9): a region's body that names a face of
//! a solid (`p inside pad.plate.near`, `distance(p, block.near)`) reads it off the solid's exact
//! boundary as the faces stand in it.  A face is its patch (the trimmed face) and has a surface
//! (`F.surface`, the whole plane or cylinder it lies on); `inside F` is the surface's side the
//! material is on.
//!
//! **The stratum** (F6): a region that reads a body's faces and is a term of that body reads the
//! body as made by the terms that do not read it, so a round never reads its own faces or
//! another round's over the same body.
use crate::brep::geom::V;
use crate::brep::query::{Located, Place};
use crate::brep::topo::EdgeCurve;
use crate::model::{FaceRef, RegionShape, RegionTerm, Sketch, SolidDef, Target};
use crate::fmath::Det;
use crate::space::{add, distance, dot, norm, scale, sub};
use std::collections::BTreeSet;
use std::f64::consts::PI;

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
}

/// The region solids whose terms read the faces of `body`.
pub fn readers_of(sk: &Sketch, body: usize) -> BTreeSet<usize> {
    let reads = |t: &RegionTerm| t.shape.entities().iter().any(|e| {
        e.kind == crate::model::EntKind::Solid && e.i() == body
    });
    (0..sk.solids.len()).filter(|&i| match &sk.solids[i].def {
        SolidDef::Region { terms, .. } => terms.iter().any(reads),
        _ => false,
    }).collect()
}

/// **The stratum's order** (#183 F6): body `body`'s fills `on` and cuts `cut` as
/// `[on, cut, on_late, cut_late]`, the late ones those reading its faces (`readers_of`), which act
/// on the body the rest make, after them, fills then cuts — the order the field, the recipe and the
/// native host apply them in.
pub fn stratum(sk: &Sketch, body: usize, on: impl IntoIterator<Item = u32>, cut: impl IntoIterator<Item = u32>)
    -> [Vec<u32>; 4] {
    let readers = readers_of(sk, body);
    let late = |o: &u32| readers.contains(&(*o as usize));
    let (on_late, on): (Vec<u32>, Vec<u32>) = on.into_iter().partition(late);
    let (cut_late, cut): (Vec<u32>, Vec<u32>) = cut.into_iter().partition(late);
    [on, cut, on_late, cut_late]
}

/// The drawing with `readers` taken out of every body's terms: the stratum they read.
fn without(sk: &Sketch, readers: &BTreeSet<usize>) -> Result<Sketch, String> {
    let mut out = sk.clone();
    for s in &mut out.solids {
        if let SolidDef::Body { stock, on, through, bound } = &mut s.def {
            if readers.contains(&(*stock as usize)) {
                return Err(format!("`{}` stands on a solid that reads its own faces", s.name));
            }
            for v in [on, through, bound] {
                v.retain(|o| !readers.contains(&(*o as usize)));
            }
        }
    }
    Ok(out)
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
        let swept = super::cad::contains_sweep(from, body);
        let ex = if swept { super::exact::swept(from, body) } else { super::exact::build(from, body) }
            .map_err(|e| format!("`{name}`'s faces: {e}"))?;
        let routes = super::document::operand_paths(from, body);
        let paths = ex.named.iter().map(|(of, face)| {
            FacePath::new(routes.get(of).map_or(of.as_str(), String::as_str), face)
        }).collect();
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
        let mut faces = BodyFaces { loc, mm, origin, paths, convex: Vec::new(), edges, bounding };
        faces.convex = (0..faces.loc.b.edges.len()).map(|e| faces.edge_convex(e as u32)).collect();
        Ok(faces)
    }

    /// Each face's name as the body names it, beside its boundary's faces (`brep`).
    pub fn paths(&self) -> &[FacePath] { &self.paths }

    /// The boundary the faces are read off: in millimetres about `origin` (model units), `mm` of
    /// them a model unit.
    pub fn brep(&self) -> (&crate::brep::topo::Brep, f64, V) { (&self.loc.b, self.mm, self.origin) }

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
    pub fn continues(&self, c: usize, faces: &[usize], v: V) -> bool {
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

    /// The faces at vertex `v` other than `not`.
    pub fn faces_at(&self, v: u32, not: &[usize]) -> Vec<usize> {
        (0..self.edges.len()).filter(|fi| {
            !not.contains(fi) && self.edges[*fi].iter().any(|&e| self.loc.b.edges[e as usize].v.contains(&v))
        }).collect()
    }
}

/// The bodies a reading has read, each once.
pub type Read = Vec<(u32, std::sync::Arc<BodyFaces>)>;

/// What a region term of a body's faces reads: the faces each reference names, resolved once.
pub struct Reading {
    pub body: std::sync::Arc<BodyFaces>,
    pub faces: Vec<usize>,
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
    let faces = body.faces(f);
    Ok(Reading { body, faces })
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
    /// The distance from the faces' patches, or (`surface`) their surfaces.
    pub fn distance(&self, p: V, surface: bool) -> f64 {
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

/// Whether a term's shape is one of a face's (`Target::Face` inside a sum included).
pub fn reads_faces(shape: &RegionShape) -> bool {
    match shape {
        RegionShape::FaceSide { .. } | RegionShape::Angle { .. } | RegionShape::Ends { .. } => true,
        RegionShape::Measured { of } => of.iter().any(|(t, _)| !matches!(t, Target::Ent(_))),
        _ => false,
    }
}

/// Whether any of a region's terms reads a body's faces (`reads_faces`).
pub fn terms_read_faces(terms: &[RegionTerm]) -> bool {
    terms.iter().any(|t| reads_faces(&t.shape))
}

/// **A face's name, as a body names it** (#188): the route through the body's operands to the
/// primitive the face is a piece of (`plate`, a placed copy's `turned`, none for the body's own
/// sweep, `document::operand_primitives`) and the face's own name in that primitive (`near`, an
/// edge's `ab`).  Produced by the one walk the exact boundary's faces are named by, and matched
/// by the one rule, `matches`.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct FacePath {
    pub route: Vec<String>,
    pub face: String,
}

impl FacePath {
    /// The face `face` of the primitive at `route` (dotted, empty for the solid itself).
    pub fn new(route: &str, face: &str) -> FacePath {
        let route = route.split('.').filter(|s| !s.is_empty()).map(str::to_string).collect();
        FacePath { route, face: face.to_string() }
    }

    /// Its segments: the route's, then the face's own name.
    pub fn segments(&self) -> impl Iterator<Item = &str> {
        self.route.iter().map(String::as_str).chain(self.face.split('.').filter(|s| !s.is_empty()))
    }

    /// Whether a path written past the solid names this face: its segments, from the first, are
    /// this face's — `plate` every face `plate` gave the body, `plate.near` its near cap, the
    /// empty path every face.  Never by a suffix: `near` alone is the body's own sweep's cap,
    /// and an instance's operand is reached from the instance's solid by its route
    /// (`pad.plate.near` is `pad`'s `plate.near`, whatever `plate`'s absolute name).
    pub fn matches<S: AsRef<str>>(&self, path: &[S]) -> bool {
        let mine: Vec<&str> = self.segments().collect();
        path.len() <= mine.len() && path.iter().zip(&mine).all(|(a, b)| a.as_ref() == *b)
    }

    /// As written in a path (`plate.near`).
    pub fn text(&self) -> String { self.segments().collect::<Vec<_>>().join(".") }
}

/// **The faces of a solid by the paths they are named by from it** (#183 F4): each primitive's
/// own (the edges of its section, then `SolidE::caps`) under its route
/// (`document::operand_primitives`, the walk `solid::exact` names faces by, placed copies
/// included) — read off the definitions, so a face a boolean ate is still listed, and reads as
/// nothing.
pub fn face_paths(sk: &Sketch, i: usize) -> Vec<FacePath> {
    let own = |p: usize| -> Vec<String> {
        let s = &sk.solids[p];
        // a sweep's faces are its cutter's, each the face it made, and a region's its terms', each
        // by the set it bounds (#212)
        match &s.def {
            SolidDef::Swept { source, .. } => {
                return face_paths(sk, *source as usize).iter().map(FacePath::text).collect()
            }
            SolidDef::Region { terms, .. } => {
                let mut v: Vec<String> = terms.iter().map(|t| t.name.clone()).collect();
                v.dedup();
                return v;
            }
            _ => {}
        }
        let mut v: Vec<String> = s.face().and_then(|f| sk.faces.get(f as usize)).map_or(Vec::new(), |f| {
            f.boundaries().flat_map(|(_, n)| n.iter().cloned()).collect()
        });
        v.extend(s.caps().iter().map(|c| c.to_string()));
        v.dedup();
        v
    };
    if i >= sk.solids.len() {
        return Vec::new();
    }
    super::document::operand_primitives(sk, i).into_values().flat_map(|(route, p)| {
        own(p as usize).into_iter().map(move |f| FacePath::new(&route, &f))
    }).collect()
}

/// **What a face of a body is, analytically** (#212): read off the definitions along the face's
/// route, never off a B-rep, so a check holding a built body's faces to it is independent of the
/// kernel that built them.
#[derive(Clone, Debug)]
pub enum FaceSource {
    /// A face of revolution: of a revolution's profile, or of a region's meridian (a term's).
    Revolved { surface: super::RevolvedSurface, solid: usize },
    /// The envelope a cutter's face sweeps: the face (carried to where the cutter's own
    /// placements put it, `within`; `solid` the revolution it is a face of) under the sweep's
    /// motion over its roll, and where the cut that leaves it is placed (`pose`).
    Swept { surface: super::RevolvedSurface, solid: usize, within: crate::envelope::Motion,
        motion: crate::motion::Family, roll: [f64; 2], pose: crate::envelope::Motion },
}

/// The analytic face `path` names in solid `body` (`FaceSource`): the route walked through the
/// body's operands by their names as `document::operand_primitives` routes them (a placement's
/// pose carried, the first operand of a name taken where several share it, as the copies of one
/// cut do), then the face named in the primitive it reaches — a revolution's profile edge, a
/// region's term, or a sweep's cutter's face by its own path in the cutter.
pub fn face_source(sk: &Sketch, body: usize, path: &FacePath) -> Result<FaceSource, String> {
    let (p, pose) = face_reach(sk, body, &path.route)?;
    let s = &sk.solids[p];
    let name = format!("{}.{}", s.name, path.face);
    match &s.def {
        SolidDef::Swept { source, motion, .. } => {
            // the cutter's face, by its own path in the cutter (`outer_crown.inner`), where the
            // cutter's placements put it
            let (route, face) = path.face.rsplit_once('.').unwrap_or(("", path.face.as_str()));
            let (solid, within) = face_reach(sk, *source as usize, &FacePath::new(route, face).route)?;
            let surface = revolved(sk, solid, face, within, name).map_err(|why| match why {
                Some(why) => why,
                None => format!("`{}` sweeps a face that is no face of revolution", s.name),
            })?;
            let motion = crate::motion::Family::read(sk, *motion as usize)?;
            let roll = super::roll::interval(sk, p)?;
            Ok(FaceSource::Swept { surface, solid, within, motion, roll, pose })
        }
        _ => {
            let surface = revolved(sk, p, &path.face, pose, name)
                .map_err(|why| why.unwrap_or_else(|| format!("`{}`'s faces are not read analytically", s.name)))?;
            Ok(FaceSource::Revolved { surface, solid: p })
        }
    }
}

/// Face `face` of primitive `p` (a revolution's profile edge, a region's term) as the surface it
/// lies on, at `pose` and called `name`; `Err(None)` where `p` has no faces of revolution.
fn revolved(sk: &Sketch, p: usize, face: &str, pose: crate::envelope::Motion, name: String)
    -> Result<super::RevolvedSurface, Option<String>> {
    let s = &sk.solids[p];
    let none = || Some(format!("`{}` has no face `{face}`", s.name));
    let mut surface = match &s.def {
        SolidDef::Revolve { face: profile, .. } => {
            let f = sk.faces.get(*profile as usize).ok_or(Some("no profile".to_string()))?;
            let edge = f.boundaries().find_map(|(es, ns)| es.iter().zip(ns).find(|(_, n)| **n == face).map(|(e, _)| *e))
                .ok_or_else(none)?;
            super::RevolvedSurface::read(sk, p, edge).map_err(Some)?
        }
        SolidDef::Region { .. } => {
            // the term's face by its name in this region, whatever the face is called where it
            // is read (a sweep's face is named under the sweep)
            let m = super::region::meridian(sk, p).map_err(Some)?;
            let own = format!("{}.{face}", s.name);
            super::RevolvedSurface::of_meridian(&m, &s.name).into_iter().find(|f| f.name == own).ok_or_else(none)?
        }
        _ => return Err(None),
    }.placed(pose);
    surface.name = name;
    Ok(surface)
}

/// The primitive a route reaches from solid `i`, and the pose the placements along it put it at.
fn face_reach(sk: &Sketch, i: usize, route: &[String]) -> Result<(usize, crate::envelope::Motion), String> {
    let s = sk.solids.get(i).ok_or("no such solid")?;
    match &s.def {
        SolidDef::Body { .. } => {
            for o in s.operands().into_iter().map(|o| o as usize) {
                let operand = &sk.solids[o];
                // a swept solid's own sweep, split off as its stock, adds no step
                if operand.name == s.name {
                    if let Ok(found) = face_reach(sk, o, route) { return Ok(found) }
                    continue;
                }
                let short = operand.name.rsplit('.').next().unwrap_or(&operand.name);
                if route.first().is_some_and(|r| r == short) {
                    return face_reach(sk, o, &route[1..]);
                }
            }
            Err(format!("`{}` has no operand `{}`", s.name, route.join(".")))
        }
        SolidDef::Placed { source, motion, at } => {
            let (p, inner) = face_reach(sk, *source as usize, route)?;
            let outer = crate::motion::Family::read(sk, *motion as usize)?.at(at.value)?;
            Ok((p, inner.then(outer)))
        }
        _ if route.is_empty() => Ok((i, crate::envelope::Motion::identity())),
        _ => Err(format!("`{}` is a primitive, with no operand `{}`", s.name, route.join("."))),
    }
}

/// `BodyFaces::read`, remembered against what it reads (`Sketch::faces_cache`): the stratum, so
/// a round's own numbers do not build its body again.
pub fn cached(sk: &Sketch, body: usize) -> Result<std::sync::Arc<BodyFaces>, String> {
    let key = super::MemoKey::except(sk, body, &readers_of(sk, body));
    sk.faces_cache.get_or(body, key, || BodyFaces::read(sk, body).map(std::sync::Arc::new))
}

/// The strata of the bodies whose faces region terms `terms` read (`cached`'s, each body once, in
/// order): what a reading of them is keyed on beside its own numbers (`MemoKey::own`).
pub fn strata(sk: &Sketch, terms: &[RegionTerm]) -> Result<Vec<std::sync::Arc<BodyFaces>>, String> {
    let mut bodies: Vec<usize> = terms.iter().flat_map(|t| t.shape.entities())
        .filter(|e| e.kind == crate::model::EntKind::Solid).map(|e| e.i()).collect();
    bodies.sort_unstable();
    bodies.dedup();
    bodies.iter().map(|&b| cached(sk, b)).collect()
}
