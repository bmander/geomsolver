//! **The faces of a body, read as sets** (#183 F1, F7, F9): a region's body that names a face of
//! a solid (`p inside pad.plate.near`, `distance(p, block.near)`) reads it off the solid's exact
//! boundary as the faces stand in it.  A face is its patch (the trimmed face) and has a surface
//! (`F.surface`, the whole plane or cylinder it lies on); `inside F` is the surface's side the
//! material is on.
//!
//! **The stratum** (F6): a region that reads a body's faces and is a term of that body reads the
//! body as made by the terms that do not read it, so a round never reads its own faces or
//! another round's over the same body.
use super::cad;
use crate::brep::geom::V;
use crate::brep::query::{Located, Place};
use crate::brep::topo::EdgeCurve;
use crate::model::{FaceRef, RegionShape, RegionTerm, Sketch, SolidDef, Target};
use crate::space::{add, distance, dot, norm, scale, sub};
use std::collections::BTreeSet;
use std::f64::consts::PI;

/// A body's exact boundary, located, in millimetres (`mm` of them a model unit), its name.
#[derive(Debug)]
pub struct BodyFaces {
    loc: Located<'static>,
    mm: f64,
    name: String,
    /// Each edge's material angle at its middle, where it bounds two faces: under a half turn
    /// where the material is the faces' sides' common part (convex).
    convex: Vec<Option<bool>>,
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
        let b = crate::brep::recipe::build(&cad::recipe(from, body)?)
            .map_err(|e| format!("`{name}`'s faces: {e}"))?;
        let tol = 1e-9 * b.size().max(1.0);
        let loc = Located::owned(b, tol);
        let mm = cad::millimetres(sk)?;
        let mut faces = BodyFaces { loc, mm, name, convex: Vec::new() };
        faces.convex = (0..faces.loc.b.edges.len()).map(|e| faces.edge_convex(e as u32)).collect();
        Ok(faces)
    }

    /// The faces a reference names: every face where its path is empty, else those its operand
    /// path reaches (`plate.near`, `boss`) or the solid's own (`near`).
    pub fn faces(&self, f: &FaceRef) -> Vec<usize> {
        let path = f.path.as_str();
        let own = format!("{}.{path}", self.name);
        let last = path.rsplit('.').next().unwrap_or(path);
        self.loc.b.faces.iter().enumerate().filter(|(_, face)| {
            let n = face.name.as_str();
            path.is_empty()
                || n == path
                || n == own
                || n.starts_with(&format!("{path}."))
                || n.starts_with(&format!("{own}."))
                || path.ends_with(&format!(".{n}"))
                || (path.contains('.') && n.ends_with(&format!(".{path}")))
                || (path.contains('.') && n.starts_with(&format!("{last}.")))
        }).map(|(i, _)| i).collect()
    }

    fn q(&self, p: V) -> V { scale(p, self.mm) }

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
        let edges: BTreeSet<u32> = f.loops.iter().flatten().map(|c| c.edge).collect();
        edges.into_iter().map(|e| self.loc.nearest(e, q).1).fold(f64::INFINITY, f64::min) / self.mm
    }

    /// The edges two groups of faces share: where a face of `a` meets a face of `b`.
    pub fn edges_between(&self, a: &[usize], b: &[usize]) -> Vec<u32> {
        let of = |fs: &[usize]| -> BTreeSet<u32> {
            fs.iter().flat_map(|&fi| self.loc.b.faces[fi].loops.iter().flatten().map(|c| c.edge))
                .collect()
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
    fn faces_of(&self, e: u32) -> Vec<usize> {
        self.loc.b.faces.iter().enumerate()
            .filter(|(_, f)| f.loops.iter().flatten().any(|c| c.edge == e))
            .map(|(i, _)| i).collect()
    }

    /// Whether edge `e` is convex: a point beside its middle within the first face's side and
    /// outside the second's is outside the material where the material is both sides' common
    /// part.  `None` for a seam or an edge two faces meeting tangent bound.
    fn edge_convex(&self, e: u32) -> Option<bool> {
        let [fa, fb] = self.faces_of(e)[..] else { return None };
        let edge = &self.loc.b.edges[e as usize];
        let x = edge.point(0.5 * (edge.t[0] + edge.t[1]), &self.loc.b.vertices);
        let (na, nb) = (self.outward(fa, x), self.outward(fb, x));
        let across = norm(sub(nb, na));
        if across < 1e-6 {
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
        let [fa, fb] = self.faces_of(e)[..] else { return None };
        let turn = dot(self.outward(fa, x), self.outward(fb, x)).clamp(-1.0, 1.0).acos();
        Some(if convex { PI - turn } else { PI + turn })
    }
}

/// What a region term of a body's faces reads: the faces each reference names, resolved once.
pub struct Reading {
    pub body: std::sync::Arc<BodyFaces>,
    pub faces: Vec<usize>,
}

/// The faces `f` names, read off its solid (refused where it names none).
pub fn reading(sk: &Sketch, f: &FaceRef, read: &mut Vec<(u32, std::sync::Arc<BodyFaces>)>)
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
        let near = self.faces.iter().copied()
            .min_by(|&a, &b| self.body.patch_distance(a, p).total_cmp(&self.body.patch_distance(b, p)));
        near.map_or(f64::INFINITY, |fi| self.body.side(fi, p))
    }
    /// The distance from the faces' patches, or (`surface`) their surfaces.
    pub fn distance(&self, p: V, surface: bool) -> f64 {
        self.faces.iter().map(|&fi| {
            if surface { self.body.side(fi, p).abs() } else { self.body.patch_distance(fi, p) }
        }).fold(f64::INFINITY, f64::min)
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

impl Reading {
    /// Where the faces are one plane: its outward normal `n` and `d`, the side `n · x - d`
    /// (model units).
    pub fn half_space(&self) -> Option<(V, f64)> {
        let [fi] = self.faces[..] else { return None };
        let f = &self.body.loc.b.faces[fi];
        let crate::brep::geom::Surface::Plane(frame) = &f.surface else { return None };
        let sign = if f.reversed { -1.0 } else { 1.0 };
        let n = scale(frame.z, sign);
        Some((n, dot(n, frame.o) / self.body.mm))
    }
}

impl BodyFaces {
    /// Face `fi`'s side and its gradient at `p` (model units; the gradient a unit vector).
    pub fn side_grad(&self, fi: usize, p: V) -> (f64, V) {
        let f = &self.loc.b.faces[fi];
        let q = self.q(p);
        let (s, g) = (f.surface.implicit(q) / self.mm, f.surface.gradient(q));
        if f.reversed { (-s, scale(g, -1.0)) } else { (s, g) }
    }

    /// The vertices three groups of faces share: where a face of each meets.
    pub fn vertices_among(&self, groups: &[&[usize]]) -> Vec<V> {
        let of = |fs: &[usize]| -> BTreeSet<u32> {
            fs.iter().flat_map(|&fi| self.loc.b.faces[fi].loops.iter().flatten())
                .flat_map(|c| self.loc.b.edges[c.edge as usize].v).collect()
        };
        let mut common = of(groups[0]);
        for g in &groups[1..] {
            common = common.intersection(&of(g)).copied().collect();
        }
        common.into_iter().map(|v| scale(self.loc.b.vertices[v as usize].p, 1.0 / self.mm)).collect()
    }

    /// Where a run of edges ends: each vertex only one of `es` reaches (model units), with the
    /// unit direction the edge runs from it.  A closed run has none.
    pub fn run_ends(&self, es: &[u32]) -> Vec<(V, V, u32)> {
        let mut count: std::collections::BTreeMap<u32, Vec<(u32, usize)>> = Default::default();
        for &e in es {
            let edge = &self.loc.b.edges[e as usize];
            if edge.v[0] == edge.v[1] {
                continue;
            }
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
            Some((scale(self.loc.b.vertices[v as usize].p, 1.0 / self.mm), d, v))
        }).collect()
    }

    /// The faces at vertex `v` other than `not`.
    pub fn faces_at(&self, v: u32, not: &[usize]) -> Vec<usize> {
        self.loc.b.faces.iter().enumerate().filter(|(fi, f)| {
            !not.contains(fi) && f.loops.iter().flatten().any(|c| self.loc.b.edges[c.edge as usize].v.contains(&v))
        }).map(|(i, _)| i).collect()
    }
}

impl Reading {
    /// The side of the face whose patch is nearest `p`, and its gradient.
    pub fn side_grad(&self, p: V) -> (f64, V) {
        let near = self.faces.iter().copied()
            .min_by(|&a, &b| self.body.patch_distance(a, p).total_cmp(&self.body.patch_distance(b, p)));
        near.map_or((f64::INFINITY, [0.0, 0.0, 1.0]), |fi| self.body.side_grad(fi, p))
    }
}

/// **The faces of a solid by the paths they are named by from it** (#183 F4): a primitive's own
/// (`near`, `far`, one per edge of its section; `start` and `end` of a partial turn), a body's
/// every primitive's under that primitive's name (`plate.near`), as its boundary names them
/// (`solid::exact`) — read off the definitions, so a face a boolean ate is still listed, and
/// reads as nothing.
pub fn face_paths(sk: &Sketch, i: usize) -> Vec<String> {
    fn own(sk: &Sketch, i: usize) -> Vec<String> {
        let names = |face: u32| -> Vec<String> {
            sk.faces.get(face as usize).map_or(Vec::new(), |f| {
                f.boundaries().flat_map(|(_, n)| n.iter().cloned()).collect()
            })
        };
        let mut out = match &sk.solids[i].def {
            SolidDef::Prism { face, .. } | SolidDef::Through { face, .. } => {
                let mut v = names(*face);
                v.extend(["near".to_string(), "far".to_string()]);
                v
            }
            SolidDef::Revolve { face, sweep, .. } => {
                let mut v = names(*face);
                if sweep.value.abs() < 2.0 * PI - 1e-9 {
                    v.extend(["start".to_string(), "end".to_string()]);
                }
                v
            }
            SolidDef::Loft { face, .. } => {
                let mut v = names(*face);
                v.extend(["start".to_string(), "end".to_string()]);
                v
            }
            _ => Vec::new(),
        };
        out.dedup();
        out
    }
    fn primitives(sk: &Sketch, i: usize, depth: usize, out: &mut Vec<usize>) {
        if depth > 64 {
            return;
        }
        match &sk.solids[i].def {
            SolidDef::Body { .. } => {
                for o in sk.solids[i].operands() {
                    primitives(sk, o as usize, depth + 1, out);
                }
            }
            _ if !out.contains(&i) => out.push(i),
            _ => {}
        }
    }
    match &sk.solids.get(i).map(|s| &s.def) {
        Some(SolidDef::Body { .. }) => {
            let mut prims = Vec::new();
            primitives(sk, i, 0, &mut prims);
            prims.into_iter().flat_map(|p| {
                let name = sk.solids[p].name.clone();
                own(sk, p).into_iter().map(move |f| format!("{name}.{f}"))
            }).collect()
        }
        Some(_) => own(sk, i),
        None => Vec::new(),
    }
}

/// `BodyFaces::read`, remembered against what it reads (`Sketch::faces_cache`).
pub fn cached(sk: &Sketch, body: usize) -> Result<std::sync::Arc<BodyFaces>, String> {
    let key = super::reads(sk, body, 0.0);
    if let Some((old, b)) = sk.faces_cache.borrow().get(&body) {
        if *old == key {
            return b.clone();
        }
    }
    let b = BodyFaces::read(sk, body).map(std::sync::Arc::new);
    sk.faces_cache.borrow_mut().insert(body, (key, b.clone()));
    b
}
