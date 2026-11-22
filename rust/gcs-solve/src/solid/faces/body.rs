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

/// Two faces' outward normals nearer than this at a point meet tangent there: no edge.  (A rolled
/// run's end faces are the kernel's, by `fillet::goes_on`: `ends_at`'s `cutting`.)
pub(crate) const TANGENT: f64 = 1e-6;

/// A point just along a run from its end `v`, `d` the way it runs from there (`by` −1: just past
/// the end).
pub(crate) fn along_run(v: V, d: V, by: f64) -> V { add(v, scale(d, by * 1e-6 * (1.0 + norm(v)))) }

/// **Of `n` things, the nearest**: its index, what `read` gives of it and its distance — the least,
/// the first among equals, as reading every one in order chooses — each `read` only where `gap`,
/// a bound under its distance, comes within the best so far.  With no `gap`, or one thing, each is
/// read.
fn nearest_by_gap<T>(n: usize, gap: Option<&dyn Fn(usize) -> f64>, read: impl Fn(usize) -> (T, f64))
    -> Option<(usize, T, f64)> {
    let Some(gap) = gap.filter(|_| n > 1) else {
        return (0..n).map(|k| (k, read(k))).min_by(|a, b| a.1 .1.total_cmp(&b.1 .1)).map(|(k, (t, d))| (k, t, d));
    };
    let mut order: Vec<(f64, usize)> = (0..n).map(|k| (gap(k), k)).collect();
    order.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
    let mut best: Option<(usize, T, f64)> = None;
    for (gap, k) in order {
        if best.as_ref().is_some_and(|b| gap > b.2) {
            break;
        }
        let (t, d) = read(k);
        if best.as_ref().is_none_or(|b| d.total_cmp(&b.2).then(k.cmp(&b.0)).is_lt()) {
            best = Some((k, t, d));
        }
    }
    best
}

/// How many faces' feet a thread remembers (`BodyFaces::foot_of`), each in the slot its number
/// gives.
const FEET_KEPT: usize = 16;

thread_local! {
    /// The feet last read on this thread: of which body (`BodyFaces::id`) and face, from which
    /// point (its bits), and the foot with its place.
    static FEET: std::cell::RefCell<[Option<(u64, usize, [u64; 3], (V, Place))>; FEET_KEPT]> =
        const { std::cell::RefCell::new([None; FEET_KEPT]) };
}

/// What a run of edges ends on (`BodyFaces::end_faces`): a face of the boundary, or a plane
/// square to an edge where its run ends on none, its unit normal `out` pointing past that end.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum EndBy {
    Face(usize),
    Square { at: V, out: V },
}

/// A number no other `BodyFaces` has.
fn next_id() -> u64 {
    static NEXT: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    NEXT.fetch_add(1, std::sync::atomic::Ordering::Relaxed)
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
    /// Whether each face is a round's or a chamfer's cap (`lowering::CAPS`): where a piece the
    /// lowering table builds ends on a face of the body, its end on that face's surface.
    caps: Vec<bool>,
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
    /// Whether each edge is a seam (`seam`): two faces bound it whose outward normals are one at
    /// its middle.  A function of the boundary alone, read once (#994): every walk across seams
    /// (`smooth`, `pieces`, `chains_apart`) asks it of every edge it meets, and a body
    /// rounded all over is walked so by every region of every edge and corner.
    seams: Vec<bool>,
    /// Each face's smooth chain (`smooth`), numbered as `shells` numbers shells: the faces
    /// reached from it across seams.
    chains: Vec<usize>,
    /// Each face's surface (`same_surface`), as the first face on it: read once, when first asked.
    surfaces: std::sync::OnceLock<Vec<usize>>,
    /// The faces each path read so far names, caps taken in (`faces`): a face's reading is
    /// asked by every round over it, and a top with the caps of a plate's pockets is a walk over
    /// every one of their edges (#994).
    named: std::sync::Mutex<std::collections::BTreeMap<Vec<String>, Vec<usize>>>,
    /// The written terms of the body as its build made them, where it was built so (#996): what
    /// the body's own build goes on from to make its rounds (`solid::exact`), rather than make
    /// the stratum again.
    stratum: Option<crate::brep::recipe::Offered>,
    /// A number of its own, by which the feet read of it are remembered (`foot_of`).
    id: u64,
}

impl BodyFaces {
    /// Solid `body`'s faces, as made by the terms that do not read them.
    pub fn read(sk: &Sketch, body: usize) -> Result<BodyFaces, String> {
        let readers = readers_of(sk, body);
        if readers.is_empty() {
            return BodyFaces::built(sk, body, false, false);
        }
        BodyFaces::built(&without(sk, &readers)?, body, false, true)
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
        let build = || BodyFaces::built(sk, body, true, false).map(std::sync::Arc::new);
        sk.caches.whole.get_or(body, key, build)
    }

    /// Solid `body`'s faces in `from`, every term of it made: its B-rep `from`'s remembered one
    /// where `remembered`, else built for this reading alone (a stratum's) — its written terms
    /// offered to the body's own build where `offer` (a stratum its rounds read, #996).
    fn built(from: &Sketch, body: usize, remembered: bool, offer: bool) -> Result<BodyFaces, String> {
        let name = from.solids.get(body).ok_or("no such solid")?.name.clone();
        // each face named by its document path (`pocket.near`), as `solid::exact` names them —
        // a swept body's by its blank's primitives and the cutter's faces its cuts left (#212)
        let swept = crate::solid::cad::contains_sweep(from, body);
        let (ex, stratum) = if swept {
            crate::solid::exact::swept(from, body).map(|e| (e, None))
        } else if remembered {
            from.exact_preimage(body).map(|e| ((*e).clone(), None))
        } else {
            crate::solid::exact::build_stratum(from, body, offer)
        }.map_err(|e| format!("`{name}`'s faces: {e}"))?;
        let primitives = crate::solid::document::operand_primitives(from, body);
        let paths = ex.named.iter().map(|(of, face)| {
            FacePath::new(primitives.get(of).map_or(of.as_str(), |(route, _)| route.as_str()), face)
        }).collect();
        // a round's or a chamfer's cap: a face the table's construction names `far`, `near`,
        // `start` or `end`, of a region over the body's faces it recognises (`face_paths`' rule),
        // read once a primitive
        let mut recognised = std::collections::BTreeMap::new();
        let mut rounds = |p: usize| *recognised.entry(p).or_insert_with(|| {
            matches!(from.solids.get(p).map(|s| &s.def), Some(crate::model::SolidDef::Region { terms, .. })
                if super::terms_read_faces(terms) && crate::solid::lowering::face_name(from, terms).is_some())
        });
        let caps = ex.named.iter().map(|(of, face)| {
            crate::solid::lowering::CAPS.contains(&face.as_str())
                && primitives.get(of).is_some_and(|&(_, p)| rounds(p as usize))
        }).collect();
        let mut faces = BodyFaces::of(ex, paths);
        faces.caps = caps;
        Ok(BodyFaces { stratum, ..faces })
    }

    /// The written terms of the body as its build made them, offered (`stratum`), where it was
    /// built so.
    pub(crate) fn stratum(&self) -> Option<&crate::brep::recipe::Offered> {
        self.stratum.as_ref()
    }

    /// An exact boundary's faces, each named by its path beside it (none: a boundary read whole,
    /// `field::BuiltField`).
    pub fn of(ex: crate::solid::Exact, paths: Vec<FacePath>) -> BodyFaces {
        let mut faces = BodyFaces::located(ex, paths);
        faces.seamed();
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

    /// `of` but each edge's convexity and its seams, left for the caller to read.
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
        let caps = vec![false; loc.b.faces.len()];
        BodyFaces { loc, mm, origin, paths, caps, convex: Vec::new(), edges, bounding, reach,
            shells, seams: Vec::new(), chains: Vec::new(), surfaces: Default::default(),
            named: Default::default(), stratum: None, id: next_id() }
    }

    /// Each edge's seam read once, and the smooth chains they join.
    fn seamed(&mut self) {
        self.seams = (0..self.loc.b.edges.len() as u32).map(|e| self.seam_at(e)).collect();
        let mut smooth = crate::graph::UnionFind::new(self.edges.len());
        for (e, by) in self.bounding.iter().enumerate() {
            if self.seams[e] {
                for w in by.windows(2) {
                    smooth.union(w[0], w[1]);
                }
            }
        }
        self.chains = smooth.labels().0;
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
        // each edge's convexity the other way round, never classified again; its seams, chains
        // and surfaces the same (turning every face's outward side turns both of a seam's
        // normals, and both of two faces' on one surface)
        let mut out = BodyFaces::located(ex, self.paths.clone());
        out.caps = self.caps.clone();
        out.convex = self.convex.iter().map(|c| c.map(|x| !x)).collect();
        out.seams = self.seams.clone();
        out.chains = self.chains.clone();
        out.surfaces = self.surfaces.clone();
        out
    }

    /// Each face's name as the body names it, beside its boundary's faces (`brep`).
    pub fn paths(&self) -> &[FacePath] { &self.paths }

    /// Face `fi` as a refusal names it: its path as the document reads it, in backticks.
    pub fn said(&self, fi: usize) -> String {
        format!("`{}`", crate::vocab::public_path(&self.paths[fi].text()))
    }

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

    /// The faces a reference names, by its path (`FacePath::matches`).  The path alone is read:
    /// the caller holds `f` to be a face of the solid this body is (`reading`, `one_body`), since
    /// another solid's path may name some other face here (#961).
    pub fn faces(&self, f: &FaceRef) -> Vec<usize> {
        if let Some(fs) = self.named.lock().ok().and_then(|m| m.get(&f.path).cloned()) {
            return fs;
        }
        let fs = (0..self.paths.len()).filter(|&i| self.paths[i].matches(&f.path)).collect();
        let fs = self.with_caps(fs);
        if let Ok(mut m) = self.named.lock() {
            m.insert(f.path.clone(), fs.clone());
        }
        fs
    }

    /// **A face with the caps lying on it** (#994): faces `named` (each by its name, in the
    /// body's order), and each round's or chamfer's cap (`caps`) that goes on from one of them
    /// that is no cap across a seam on the same surface, or from such a cap on.  A fill ending
    /// flush on a face has grown that face: its cap lies on the face's own surface with no edge
    /// between them, one face of the body, so `whole.part.stock.far` of a block whose reflex
    /// uprights are filled is the top with the fills' caps, and a round of it goes round each
    /// fill's arc too.  A cap named is itself alone; a face of another operand on the same
    /// surface (a plate's top a post's foot is flush with) keeps its own name.  Where nothing of
    /// the body is a cap (a stratum beneath its rounds), `named` itself.
    pub fn with_caps(&self, named: Vec<usize>) -> Vec<usize> {
        if !self.caps.iter().any(|&c| c) {
            return named;
        }
        let given = named.len();
        let mut met = vec![false; self.edges.len()];
        for &fi in &named {
            met[fi] = true;
        }
        let mut out = named;
        // each cap going on across a seam on the same surface (`same_surface`): a piece of the
        // surface it is reached on (`pieces`, #994)
        let mut k = 0;
        while k < out.len() {
            let fi = out[k];
            k += 1;
            // (a cap named is itself alone, not the caps beside it)
            if self.caps[fi] && k <= given {
                continue;
            }
            for &e in self.edges[fi].iter().filter(|&&e| self.seams[e as usize]) {
                for &g in self.faces_of(e) {
                    if !self.caps[g] || met[g] || !self.same_surface(g, fi) {
                        continue;
                    }
                    met[g] = true;
                    out.push(g);
                }
            }
        }
        out
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
        self.loc.b.faces[fi].outward_gradient(q)
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
    pub fn patch_distance(&self, fi: usize, p: V) -> f64 { self.patch_reading(fi, p).0 }

    /// `patch_distance`, with the foot it was read from and where that lies on the face
    /// (`foot_of`), so a caller asking more of the face nearest reads its foot once.
    fn patch_reading(&self, fi: usize, p: V) -> (f64, V, Place) {
        let q = self.q(p);
        let (foot, place) = self.foot_of(fi, q);
        if place != Place::Out {
            return (distance(q, foot) / self.mm, foot, place);
        }
        // (a distance no number is read as none, as a fold of the least reads it)
        let d = self.nearest_of_edges(&self.edges[fi], q).map(|(_, (_, d))| d)
            .filter(|d| !d.is_nan()).map_or(f64::INFINITY, |d| d / self.mm);
        (d, foot, place)
    }

    /// The foot of `q` (millimetres) on face `fi`'s surface, and where it lies on the face: a
    /// step of a projection reads it of a face from one point several times (the chain's side,
    /// each face's own, `conditions::faces_apart`), so the last few on each thread are remembered
    /// against the body, the face and the point, bit for bit (`FEET`).
    fn foot_of(&self, fi: usize, q: V) -> (V, Place) {
        let at = q.map(f64::to_bits);
        let slot = fi % FEET_KEPT;
        if let Some((id, f, from, read)) = FEET.with_borrow(|feet| feet[slot]) {
            if id == self.id && f == fi && from == at {
                return read;
            }
        }
        let f = &self.loc.b.faces[fi];
        let foot = f.surface.point(f.surface.inverse(q));
        let read = (foot, self.loc.face_place(fi, foot));
        FEET.with_borrow_mut(|feet| feet[slot] = Some((self.id, fi, at, read)));
        read
    }

    /// The edge of face `fi` nearest a foot on its surface (millimetres), the first among equals,
    /// read where its box comes near.
    fn edge_nearest_foot(&self, fi: usize, foot: V) -> Option<u32> {
        self.nearest_of_edges(&self.edges[fi], foot).map(|(e, _)| e)
    }

    /// **The edge of `es` nearest `q` (millimetres), and its nearest point's parameter and
    /// distance** (#996): what reading every edge in order chooses — the least distance, the first
    /// edge of `es` among equals — read only where an edge's box comes within the best so far
    /// (`Located::edge_gap`, less the boundary's tolerance: no point of the edge is nearer).  A
    /// field read off a body's faces asks this at every step of every query, and an edge traced
    /// rather than drawn in closed form is a search to invert; read edge by edge, the cost of a
    /// query grew with the edges of the faces it reads, however far.  `None` where `es` is empty.
    fn nearest_of_edges(&self, es: &[u32], q: V) -> Option<(u32, (f64, f64))> {
        let gap = |k: usize| self.loc.edge_gap(es[k], q) - self.loc.tol;
        // (a point no box orders: every edge read, as the scan reads them)
        let boxed = q.iter().all(|x| x.is_finite());
        nearest_by_gap(es.len(), boxed.then_some(&gap), |k| self.loc.nearest(es[k], q))
            .map(|(k, t, d)| (es[k], (t, d)))
    }

    /// **Of `faces`, the one whose patch is nearest `p`, and that distance** (model units): what
    /// reading every face's `patch_distance` in order chooses — the least, the first of `faces`
    /// among equals — read only where a face's box comes within the best so far
    /// (`Located::face_gap`, less the boundary's tolerance: no point of the face or its edges is
    /// nearer).  A smooth chain's or a named group's nearest face is asked at every step of every
    /// query of a field that reads the body's faces (#996).  `None` where `faces` is empty.
    pub fn nearest_patch(&self, faces: &[usize], p: V) -> Option<(f64, usize)> {
        self.nearest_patch_read(faces, p).map(|(d, fi, _)| (d, fi))
    }

    /// `nearest_patch`, with the nearest face's foot and its place (`patch_reading`).
    fn nearest_patch_read(&self, faces: &[usize], p: V) -> Option<(f64, usize, (V, Place))> {
        let q = self.q(p);
        let gap = |k: usize| (self.loc.face_gap(faces[k], q) - self.loc.tol) / self.mm;
        let boxed = q.iter().all(|x| x.is_finite());
        let read = |k: usize| {
            let (d, foot, place) = self.patch_reading(faces[k], p);
            ((foot, place), d)
        };
        nearest_by_gap(faces.len(), boxed.then_some(&gap), read).map(|(k, at, d)| (d, faces[k], at))
    }

    /// The distance from the nearest of `faces`' patches (`nearest_patch`): none, read as a fold of
    /// the least reads it, where there is no face or no distance is a number.
    pub fn nearest_distance(&self, faces: &[usize], p: V) -> f64 {
        self.nearest_patch(faces, p).map(|(d, _)| d).filter(|d| !d.is_nan()).unwrap_or(f64::INFINITY)
    }

    /// How far `p` is from face `fi`'s box in space (`Located::face_box`, which holds its patch;
    /// model units): no farther than the patch, and nothing where `p` is within the box.
    pub fn patch_gap(&self, fi: usize, p: V) -> f64 { self.loc.face_gap(fi, self.q(p)) / self.mm }

    /// Where `p`'s foot on face `fi`'s surface lies off the face: the edge of the face nearest it
    /// (`None` where it lies on the face).
    pub fn off_face(&self, fi: usize, p: V) -> Option<u32> {
        let (foot, place) = self.foot_of(fi, self.q(p));
        if place != Place::Out {
            return None;
        }
        self.edge_nearest_foot(fi, foot)
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
        let (e, (t, d)) = self.nearest_of_edges(es, self.q(p))?;
        let at = self.loc.b.edges[e as usize].point(t, &self.loc.b.vertices);
        Some((e, at, d / self.mm))
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

    /// Whether edge `e` ends at vertex `vid`.
    fn touches(&self, e: u32, vid: u32) -> bool { self.ends(e).contains(&vid) }

    /// The first edge of `run` ending at vertex `vid`.
    fn run_edge_at(&self, run: &[u32], vid: u32) -> Option<u32> {
        run.iter().copied().find(|&e| self.touches(e, vid))
    }

    /// Whether face `g` is of the smooth chain of one of `faces` (`smooth`, read by `chains`).
    fn in_chains(&self, faces: &[usize], g: usize) -> bool {
        faces.iter().any(|&f| self.chains[f] == self.chains[g])
    }

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

    /// Face `fi`'s side carried on past its patch by its tangent planes, and its gradient at `p`
    /// (model units): its surface's signed distance where `p`'s foot on it lies on the face, else
    /// the signed distance from the plane tangent to the surface at the point of the face's edges
    /// nearest `p` where that edge is a seam, and from its own surface past any other edge (a
    /// crease, a face cutting across it: its side is its surface's, carried on only across seams,
    /// #994).  Where the face bulges toward a side (a plane, or a cylinder, a sphere or a cone
    /// with that side inside it), this is the signed distance from the convex solid its patch and
    /// those planes bound on that side — so past a seam where it goes on tangent into a face like
    /// it, the next face's own side, and over a smooth chain of them their sides together say the
    /// chain's (`Reading::bulging`).
    pub fn carried_side_grad(&self, fi: usize, p: V) -> (f64, V) {
        // a plane's tangent planes are itself
        if matches!(self.loc.b.faces[fi].surface, crate::brep::geom::Surface::Plane(_)) {
            return self.side_grad(fi, p);
        }
        self.carried_past(fi, p).1
    }

    /// `carried_side_grad` read past the patch whatever the surface (a plane's too), with the edge
    /// of the face it is carried on past, the distance from the patch (model units) and the point
    /// of the edge it is carried past (the boundary's millimetres): `None` and the surface's own
    /// side where `p`'s foot on it lies on the face.
    pub fn carried_past(&self, fi: usize, p: V) -> (Option<(u32, f64, V)>, (f64, V)) {
        let q = self.q(p);
        if self.foot_of(fi, q).1 != Place::Out {
            return (None, self.side_grad(fi, p));
        }
        let (nearest, seam) = self.nearest_and_seam(fi, q);
        let read = |(e, (t, d)): (u32, (f64, f64))| {
            (e, self.loc.b.edges[e as usize].point(t, &self.loc.b.vertices), d / self.mm)
        };
        let Some((e, at, d)) = nearest.map(read) else {
            return (None, self.side_grad(fi, p));
        };
        // past a seam, the plane tangent there, which the next face goes on from; past any other
        // edge — a crease, or where a face cuts across it (a hub's side across a cylinder) — the
        // face's own surface, as its side is (#994).  A seam as near as the nearest edge (a
        // corner of the patch where a seam and a crease meet, nearest a point beyond both) is
        // the one carried past
        if !self.seam(e) {
            return match seam.map(read) {
                Some((s, at, ds)) if ds <= d + 1e-9 * (1.0 + d) => (Some((s, ds, at)), self.tangent_side(fi, at, p)),
                _ => (Some((e, d, at)), self.side_grad(fi, p)),
            };
        }
        (Some((e, d, at)), self.tangent_side(fi, at, p))
    }

    /// **Of face `fi`'s edges, the nearest `q` (millimetres) and the nearest seam** (`seam`), each
    /// as `nearest_of_edges` reads it of those edges — the least, the first among equals, each
    /// edge read where its box comes within the best so far of either — in one pass (#994).
    #[allow(clippy::type_complexity)]
    fn nearest_and_seam(&self, fi: usize, q: V)
        -> (Option<(u32, (f64, f64))>, Option<(u32, (f64, f64))>) {
        let es = &self.edges[fi];
        let mut order: Vec<(f64, usize)> = (0..es.len()).map(|k| {
            // (a point no box orders: every edge read, as the scan reads them)
            let gap = if q.iter().all(|x| x.is_finite()) {
                self.loc.edge_gap(es[k], q) - self.loc.tol
            } else {
                f64::NEG_INFINITY
            };
            (gap, k)
        }).collect();
        order.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        let better = |d: f64, k: usize, b: &Option<(usize, (f64, f64))>| {
            b.is_none_or(|(j, (_, e))| d.total_cmp(&e).then(k.cmp(&j)).is_lt())
        };
        let (mut all, mut seam): (Option<(usize, (f64, f64))>, Option<(usize, (f64, f64))>) =
            (None, None);
        // the seams not read yet: past the nearest edge, only they are read on
        let mut seams = es.iter().filter(|&&e| self.seams[e as usize]).count();
        for (gap, k) in order {
            let past = |b: &Option<(usize, (f64, f64))>| b.is_some_and(|(_, (_, d))| gap > d);
            if past(&all) && (seams == 0 || past(&seam)) {
                break;
            }
            let is_seam = self.seams[es[k] as usize];
            if past(&all) && !is_seam {
                continue;
            }
            let (t, d) = self.loc.nearest(es[k], q);
            if better(d, k, &all) {
                all = Some((k, (t, d)));
            }
            if is_seam {
                seams -= 1;
                if better(d, k, &seam) {
                    seam = Some((k, (t, d)));
                }
            }
        }
        (all.map(|(k, r)| (es[k], r)), seam.map(|(k, r)| (es[k], r)))
    }

    /// The side of the plane tangent to face `fi`'s surface at `at` (the boundary's millimetres),
    /// and its gradient, at `p` (model units).
    pub fn tangent_side(&self, fi: usize, at: V, p: V) -> (f64, V) {
        let n = self.outward(fi, at);
        (dot(n, sub(self.q(p), at)) / self.mm, n)
    }

    /// **Faces whose side is convex toward a point** (#962), within them or (`beyond`) outside them:
    /// one face a plane, or a cylinder, a sphere or a cone with that side inside it — that solid,
    /// read by its surface's side, the face's own (a closed cylinder's pocket from inside, a boss's
    /// from its material); or a smooth chain (`smooth`, `F.smooth` gone on across a seam) every face
    /// of which is so and every edge between two of them a seam or a crease convex from that side —
    /// each read by its side carried on past its patch by its tangent planes
    /// (`carried_side_grad`, `Some(true)`; one face's, `Some(false)`).  That side is then the common part of
    /// each face's convex solid, and a point at least `d` inside it is at least `d` inside each, so
    /// either condition holds face by face, each convex.  Read through the nearest face alone, a
    /// corner of the chain's offset — where an arc's offset closes onto its axis, a ball as wide as
    /// a pocket's corner, or has passed it — is where the nearest face turns from one to another,
    /// and a projection onto it with it, never settling there; and an offset that has all but
    /// closed onto an axis or a centre is a surface curved tightly about a point far off, whose
    /// steps are steady only toward a convex set (`conditions::nearest`).  `None` where a face is
    /// none of those, the faces lie on several surfaces without a seam between them, meet
    /// otherwise, lie on several shells, or are no one sheet: a face's patch outside another's
    /// convex solid (a block's side flush with a deck's, the deck's corner arc above it, #994).
    pub fn bulging(&self, faces: &[usize], smooth: bool, beyond: bool) -> Option<bool> {
        let (shell, _) = self.shells();
        let &f0 = faces.first()?;
        if faces.iter().any(|&fi| shell[fi] != shell[f0]) {
            return None;
        }
        if !faces.iter().all(|&fi| self.bulges(fi, beyond, faces.len() > 1)) {
            return None;
        }
        if let [_] = faces {
            return Some(false);
        }
        // pieces of one surface (a face a crossing operand leaves in pieces, the lid an arm's
        // bottom is, either side of a hub): that surface's convex solid, its own side whichever
        // piece says it (`same_surface`), with no seam between them (#994) — a plane's
        // half-space, a cylinder's, a cone's or a sphere's solid (a torus's whole side is none)
        if faces.iter().all(|&fi| self.same_surface(fi, f0)) && self.bulges(f0, beyond, false) {
            return Some(false);
        }
        let mut seams = false;
        for &fi in faces {
            for &e in &self.edges[fi] {
                let [x, y] = *self.faces_of(e) else { continue };
                let other = if x == fi { y } else { x };
                if other == fi || !faces.contains(&other) {
                    continue;
                }
                if self.seam(e) {
                    seams = true;
                } else if self.convex(e) != Some(!beyond) {
                    return None;
                }
            }
        }
        if !(smooth && seams) {
            return None;
        }
        // convex across every seam and crease between its faces, a chain can still turn the
        // other way along a face's own outline: a top in two flush pieces, a rim of one rounded
        // and the other's run beside it past the reflex corner between them (#996).  The sides
        // carried on bound the chain's convex solid only where every face of it lies within each
        // other's side carried on past its patch; elsewhere the carried side of one cuts off
        // another face's patch, and the chain bulges no way
        // — read over each edge of the chain's faces once, against each face of the chain but
        // those of its own: a plane's side in closed form over a line or a circle (`plane_past`),
        // any other face's carried side at nine points of the edge, read once for every face; past
        // by more than `past`, the length the body reads a point past a face by
        let sigma = if beyond { -1.0 } else { 1.0 };
        let mut edges: BTreeSet<u32> = BTreeSet::new();
        for &fi in faces {
            edges.extend(self.edges[fi].iter().copied());
        }
        for e in edges {
            let own = self.faces_of(e);
            let mut points: Option<Vec<V>> = None;
            for &fi in faces.iter().filter(|&&fi| faces.iter().any(|&g| g != fi && own.contains(&g))) {
                let out = match self.plane_past(fi, sigma, e) {
                    Some(out) => out,
                    None => points.get_or_insert_with(|| (0..=8).map(|k| self.along(e, k as f64 / 8.0)).collect())
                        .iter().any(|&p| sigma * self.carried_side_grad(fi, p).0 > self.past(p)),
                };
                if out {
                    return None;
                }
            }
        }
        Some(true)
    }

    /// Whether face `fi` is convex toward a point within it or (`beyond`) outside it, as `bulging`
    /// reads each of its faces: a plane, or a cylinder, a sphere or a cone with that side inside
    /// it; in a chain (`chained`) also a torus's patch on the outer half of its tube, that side
    /// inside the tube.
    pub fn bulges(&self, fi: usize, beyond: bool, chained: bool) -> bool {
        // the side read within the face's surface's solid (a plane's half-space either way)
        match self.level(fi, 0.0) {
            Some(Level::Bound(Bound::Half { .. }, _)) => true,
            Some(Level::Bound(Bound::Cylinder { .. } | Bound::Ball { .. } | Bound::Cone { .. }, within)) =>
                within != beyond,
            // a torus's patch on the outer half of its tube, that side inside the tube: both its
            // curvatures' centres (the tube's middle circle, the axis) on that side, so convex
            // there — a fill round a pocket's arc, a boss's foot rounded — read in a chain alone,
            // by its side carried on past its patch (its whole surface's side winds round the
            // axis, no convex solid)
            Some(Level::Bound(Bound::Torus { .. }, within)) => within != beyond && chained && self.outer_half(fi),
            _ => false,
        }
    }

    /// Whether face `fi`, on a torus, lies on the outer half of its tube: every point of its
    /// edges at least the middle circle's radius from the axis.
    fn outer_half(&self, fi: usize) -> bool {
        let crate::brep::geom::Surface::Torus(frame, big, _) = self.loc.b.faces[fi].surface else { return false };
        let b = &self.loc.b;
        self.edges[fi].iter().all(|&e| {
            let ed = &b.edges[e as usize];
            (0..=8).all(|k| {
                let p = sub(ed.point(ed.t[0] + (ed.t[1] - ed.t[0]) * k as f64 / 8.0, &b.vertices), frame.o);
                let rho = norm(sub(p, scale(frame.z, dot(p, frame.z))));
                rho >= big * (1.0 - 1e-9)
            })
        })
    }

    /// Faces `named` grouped by the smooth chains they go on into (`smooth`): each group's faces and
    /// its chain — one group where they are one chain, several where a name holds pieces apart (two
    /// rounds' caps under one name, one on a step's side and one on its wall, #962).
    pub fn chains_apart(&self, named: &[usize]) -> Vec<(Vec<usize>, Vec<usize>)> {
        let mut out: Vec<(Vec<usize>, Vec<usize>)> = Vec::new();
        for &fi in named {
            // one chain is every face of one smooth chain (`chains`): a face of a chain met
            // already adds none to it
            match out.iter_mut().find(|(own, _)| self.chains[own[0]] == self.chains[fi]) {
                Some((own, _)) => own.push(fi),
                None => out.push((vec![fi], self.smooth(&[fi]))),
            }
        }
        out
    }

    /// Of `alts`, the one whose faces `named` gives hold the patch nearest `p` (`patch_distance`,
    /// each alternative measured once; the first of two as near): of a name in pieces whose chains
    /// lie apart (`chains_apart`), the chain of the piece nearest `p`.
    pub fn nearest_named<'a, T>(&self, p: V, alts: &'a [T], named: impl Fn(&T) -> &[usize])
        -> Option<&'a T> {
        let near = |fs: &[usize]| self.nearest_distance(fs, p);
        alts.iter().map(|a| (near(named(a)), a)).min_by(|x, y| x.0.total_cmp(&y.0)).map(|(_, a)| a)
    }

    /// Which of `fs` a point `p` reads a smooth surface's side by: the face whose patch is nearest
    /// it — but where that is a point of their rim, an edge to a face they do not hold, the surface
    /// ending there, no side is theirs: past a flush wall's top edge the wall says within, the deck
    /// it goes on into without (#994).  There the side is that of the surface going on from the
    /// rim's ends; or, where the face the rim is to stands between the point and the nearest patch
    /// (the point on one side of its surface, the patch going off the rim to the other), of one of
    /// `fs` that face bounds too — a band's side standing on a plate whose surface goes on into the
    /// band's top: beside the band the plate, within it the band's top, though the other's patch
    /// is the nearer (a casting's floor its drafted side leans over stands between nothing) — the
    /// nearest of those whose surface's foot lies on it, where one does (where none does, a hole a
    /// boss's foot cuts in a side, the nearest face's still).  A point in the plane of a face the
    /// rim is to that the set holds its point outside of or within (`held`, `true` outside) reads as
    /// one just past that face or just within it, as the set holds it: the gate's probes of a
    /// fill's leg, or of a round's on a band's side, lie on it.  `None` where `fs` is empty.
    pub fn nearest_face(&self, fs: &[usize], p: V, held: &[(usize, bool)]) -> Option<usize> {
        let (_, fi, (foot, place)) = self.nearest_patch_read(fs, p)?;
        let rim = |e: &u32| self.faces_of(*e).iter().any(|g| !fs.contains(g));
        // the point's foot on the face's surface as the nearest patch read it, and the edge
        // nearest it where it lies off the face (`off_face`) or on its boundary (#994)
        let nearest = (place != Place::In).then(|| self.edge_nearest_foot(fi, foot)).flatten();
        let off = nearest.filter(|_| place == Place::Out);
        let Some(e) = off.filter(rim) else {
            // (on the rim to a face the point is held outside of or within, in that face's plane:
            // read just past it, or just within it)
            let on = nearest.filter(rim).and_then(|e| self.faces_of(e).iter().find_map(|&c| {
                let &(_, beyond) = held.iter().find(|&&(h, _)| h == c)?;
                (self.side(c, p).abs() <= self.past(p)).then_some((c, if beyond { 1.0 } else { -1.0 }))
            }));
            return match on {
                Some((c, way)) => self.nearest_face(fs, add(p, scale(self.normal(c, p), way * 2.0 * self.past(p))), &[]),
                None => Some(fi),
            };
        };
        // (a face going on from the rim's ends: one bounding an edge at either)
        let ends = self.loc.b.edges[e as usize].v;
        let at_end = |g: usize| self.edges[g].iter().any(|&x| self.loc.b.edges[x as usize].v.iter().any(|w| ends.contains(w)));
        // (a face the rim is to standing between: the point past its surface one way, the patch
        // going off the rim — from the foot back to the rim's nearest point — the other)
        let edge = &self.loc.b.edges[e as usize];
        let on_rim = edge.point(self.loc.nearest(e, foot).0, &self.loc.b.vertices);
        let back = sub(self.back(on_rim), self.back(foot));
        let between: Vec<usize> = self.faces_of(e).iter().copied().filter(|&c| !fs.contains(&c))
            .filter(|&c| {
                let (s, n) = (self.side(c, p), self.normal(c, p));
                s.abs() > self.past(p) && dot(back, n) * s < 0.0
            }).collect();
        // (the chain goes on round it only where it bounds it nearer the point than any face not of
        // the chain does: a block's side bowed in under an eye bounds the eye at its far end, its
        // own end and top nearer, and past its foot the floor's own plane is read, #994)
        let off_chain = |c: usize| self.edges[c].iter().copied()
            .filter(|&x| self.faces_of(x).iter().any(|&h| h != c && !fs.contains(&h)))
            .filter_map(|x| self.nearest_edge(&[x], p)).map(|(_, _, d)| d)
            .fold(f64::INFINITY, f64::min);
        let reach: Vec<(usize, f64)> = between.iter().map(|&c| (c, off_chain(c))).collect();
        let near = |x: u32, d: f64| self.nearest_edge(&[x], p).is_some_and(|(_, _, e)| e <= d);
        let stepped = |g: usize| self.edges[g].iter().any(|&x| self.faces_of(x).iter()
            .any(|c| reach.iter().any(|&(b, d)| b == *c && near(x, d))));
        Some(fs.iter().copied().filter(|&g| g != fi && (at_end(g) || stepped(g)) && self.off_face(g, p).is_none())
            .map(|g| (self.side(g, p).abs(), g)).min_by(|a, b| a.0.total_cmp(&b.0)).map_or(fi, |(_, g)| g))
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

    /// Whether face `c` meets one of `faces` tangent at `v` (model units): their outward normals
    /// there one (`turn` 1: `c` carries that face on across a seam, and the edge between them is
    /// none) or opposite (`turn` -1: back to back, `c`'s material across that face's surface from
    /// its own — a flange's underside and the top of a block it stands on that runs out past its
    /// edge, the two meeting at the point where the block's side crosses the flange's rim).
    fn tangent_at(&self, c: usize, faces: &[usize], v: V, turn: f64) -> bool {
        let q = self.q(v);
        let n = scale(self.outward(c, q), turn);
        faces.iter().any(|&f| norm(sub(self.outward(f, q), n)) < TANGENT)
    }

    /// Whether face `c` at vertex `vid` (at `v`, model units) carries run `run` between `faces` on
    /// past it, of the run's turn (its first edge's, convex: `Some(true)`): going on tangent from
    /// one of them across a seam there (`tangent_at`) and meeting another of them there in an edge
    /// of that turn, along which the round goes on — an obround's half circle after its flat side
    /// (#994).  A face going on tangent that meets them there only in an edge of the other turn
    /// carries nothing on — unless it lies on the surface of a face of the run's edges there
    /// (`same_surface`), along which the run goes on: the run ends on it — a deck's corner arc
    /// running tangent into its side, flush with the block it stands on, whose side meets the
    /// block's top in a convex edge.  One meeting them there in no edge at all carries the run on,
    /// as `tangent_at` reads any face going on tangent (#996).
    fn carries_on(&self, c: usize, faces: &[usize], run: &[u32], v: V, vid: u32) -> bool {
        if !self.tangent_at(c, faces, v, 1.0) {
            return false;
        }
        // (the run's turn, and the faces of its edges at the vertex)
        let turn = run.first().and_then(|&e| self.convex(e));
        let own = run.iter().filter(|&&e| self.touches(e, vid))
            .flat_map(|&e| self.faces_of(e));
        // a piece of the surface of one of the edge's own faces (a floor two cuts leave in two
        // pieces, `same_surface`): the run goes on along it, whatever it meets
        if own.into_iter().any(|&f| self.same_surface(c, f)) {
            return true;
        }
        // (one of them, or a face it goes on into across a seam: a fill's cap flush with a side —
        // in an edge, not the seam it goes on across: a plate a band's foot arc runs tangent into
        // meets the arc in no turn, #994)
        // — and where it meets none of them there in any edge (a block's end flush under a
        // flange's side, tangent to the flange's corner arc at the vertex alone, #996), it carries
        // the run on as any face going on tangent does: it bounds nothing the run reaches
        let meets: Vec<u32> = self.edges[c].iter().copied().filter(|&e| {
            self.touches(e, vid) && !self.seam(e)
                && self.faces_of(e).iter().any(|&g| g != c && self.in_chains(faces, g))
        }).collect();
        turn.is_none() || meets.is_empty() || meets.iter().any(|&e| self.convex(e) == turn)
    }

    /// Whether face `c`'s surface holds the edge of `run` that ends at vertex `vid`
    /// (`holds_edge`).
    fn lies_along(&self, c: usize, run: &[u32], vid: u32) -> bool {
        run.iter().filter(|&&e| self.touches(e, vid)).any(|&e| self.holds_edge(c, e))
    }

    /// **Whether face `c`'s surface holds edge `e`**: no point of the edge past it either way by
    /// more than `past` (`goes_across`) — in closed form for a plane against a line (`plane_holds`)
    /// or a circle (a plane square to its axis through it), the edge sampled against any other.
    fn holds_edge(&self, c: usize, e: u32) -> bool {
        if let (Some((n, o)), Some(ends)) = (self.plane(c), self.segment(e)) {
            return self.plane_holds((n, dot(n, o)), ends);
        }
        !self.goes_across(c, 1.0, &[e]) && !self.goes_across(c, -1.0, &[e])
    }

    /// Whether the plane `n·x = d` (`n` a unit normal, model units) holds the straight edge
    /// between `ends`: each end within `past` of it, so every point between is.  `holds_edge`'s
    /// closed form for a plane against a line, read where only the plane is in hand (a run's
    /// piece, `lowering::piece::run_piece`): a plane along the run stands at no distance along it.
    pub(crate) fn plane_holds(&self, (n, d): (V, f64), ends: [V; 2]) -> bool {
        ends.iter().all(|&p| (dot(n, p) - d).abs() <= self.past(p))
    }

    /// Whether faces `c` and `f` meet in an edge at vertex `vid`.
    fn meet_at(&self, c: usize, f: usize, vid: u32) -> bool {
        self.edges_between(&[c], &[f]).iter()
            .any(|&e| self.touches(e, vid))
    }

    /// Face `fi`'s plane, where it lies in one: its unit normal away from the material and its
    /// frame's origin (model units).
    pub fn plane(&self, fi: usize) -> Option<(V, V)> {
        let f = &self.loc.b.faces[fi];
        let crate::brep::geom::Surface::Plane(frame) = &f.surface else { return None };
        Some((if f.reversed { scale(frame.z, -1.0) } else { frame.z }, self.back(frame.o)))
    }

    /// `faces`, but of those lying on one plane (`same_surface`) the first alone: what a plane
    /// says of a point — its side, carried on past its patch or not — it says whichever of its
    /// faces says it (#994).  A curved surface's side carried past a piece is that piece's own
    /// (`carried_past`: by its tangent planes past a seam), so its pieces each say theirs.
    pub fn one_per_plane(&self, faces: &[usize]) -> Vec<usize> {
        let mut planes: Vec<usize> = Vec::new();
        faces.iter().copied().filter(|&fi| {
            if self.plane(fi).is_none() {
                return true;
            }
            let s = self.surface_of(fi);
            let first = !planes.contains(&s);
            if first {
                planes.push(s);
            }
            first
        }).collect()
    }

    /// **Whether faces `f` and `g` lie on one surface the same way out** (#994): the kernel's
    /// `fillet::one_surface` at the boundary's tolerance — their surfaces one by the rule a
    /// Boolean joins faces by (`tolerant::one_surface`, over the faces' boxes) and their outward
    /// normals one — read once for the body (`surface_of`).  What the field reads as one surface
    /// (a plane's pieces one condition, `one_per_plane`; pieces of one surface its convex solid,
    /// `bulging`; a face going on along a piece of the run's own, `carries_on`; a surface's
    /// pieces, `pieces`), what the lowering does (`piece::held`) and what the kernel rolls a ball
    /// on (`fillet`'s sides) are this.
    pub fn same_surface(&self, f: usize, g: usize) -> bool {
        self.surface_of(f) == self.surface_of(g)
    }

    /// Face `fi`'s surface, as the first face of the body on it (`same_surface`).
    fn surface_of(&self, fi: usize) -> usize {
        self.surfaces.get_or_init(|| {
            let b = &self.loc.b;
            let mut firsts: Vec<usize> = Vec::new();
            (0..b.faces.len()).map(|fi| {
                let found = firsts.iter().copied()
                    .find(|&g| crate::brep::fillet::one_surface(b, g, fi, self.loc.tol));
                found.unwrap_or_else(|| { firsts.push(fi); fi })
            }).collect()
        })[fi]
    }

    /// **The pieces of face `fi`'s surface** (#952): `fi` and every face going on from it across a
    /// seam on the same surface (`same_surface`), and from those on — a plate's top in the pieces
    /// the union of its operands left it, each named by its own operand's.  `fi` first.
    pub fn pieces(&self, fi: usize) -> Vec<usize> {
        self.across_seams(&[fi], |g| self.same_surface(g, fi)).into_iter().map(|(g, _)| g).collect()
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
    /// that face meets the other in.  Nor does a face along the run or back to back with one of
    /// `faces` there (the top of a block a flange stands on, past the flange's edge), nor one whose
    /// surface the run itself goes across, nor a plane turning back over its section at a reflex
    /// corner (`ends_at`).  And a run carried
    /// past an end along the edge the face it ends on makes with one of `faces` (a run-out) goes no
    /// further than that edge: where that edge's run ends further on, on the side it runs on, so
    /// does the run.  Each with the end it was read at where it stands elsewhere: none for a face
    /// at an end of the run, that end's point (model units) for where the edge a run-out there
    /// runs along ends.  Past an end face shorter than the section, a face beyond it facing on
    /// past the end ends a run rolled along too, read at that end (`beyond_end`, #994): along an
    /// end face no longer than `reach`, the section's size, and nowhere where none is given.
    pub fn end_faces(&self, es: &[u32], faces: &[usize], reach: Option<f64>)
        -> Vec<(EndBy, f64, Option<V>)> {
        self.end_faces_as(es, faces, es.iter().all(|&e| self.carried(e)), reach)
    }

    /// `end_faces`, the rules of a run's own ends (`goes_across`, a reflex corner's plane, where a
    /// run-out ends) read only where the lowering builds its runs and rings by them, every edge of
    /// the set `plain` (`carried`): a ball rolled along a traced run ends on the faces the kernel
    /// cuts it by (`fillet::end_at`), read by the kernel's own rule (`ends_at`'s `cutting`).
    fn end_faces_as(&self, es: &[u32], faces: &[usize], plain: bool, reach: Option<f64>)
        -> Vec<(EndBy, f64, Option<V>)> {
        let mut sides: Vec<(EndBy, f64, Option<V>)> = Vec::new();
        let runs = self.runs(es);
        // (each end of a run that ends on no face there: where it may go on across a seam)
        let mut going_on: Vec<(&Vec<u32>, V, u32)> = Vec::new();
        for (v, d, vid) in self.run_ends(es) {
            let Some(run) = runs.iter().find(|run| self.run_edge_at(run, vid).is_some()) else {
                continue;
            };
            let at = self.ends_at(run, faces, v, d, vid, plain);
            if at.is_empty() && plain {
                going_on.push((run, v, vid));
            }
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
            // a ball rolled along: past an end face shorter than its section, each face beyond it
            // that faces on past the end cuts it off too, as the kernel reads it (#994)
            if let (false, Some(reach)) = (plain, reach) {
                for (x, sx) in self.beyond_end(run, &at, vid, d, reach) {
                    if !sides.iter().any(|&(y, _, _)| y == EndBy::Face(x)) {
                        sides.push((EndBy::Face(x), sx, Some(v)));
                    }
                }
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
                            // that edge ending only where `f` goes on tangent across a seam, `c`
                            // going on along the next face (a floor's edge along a drafted side
                            // running on round its corner's cone, #994): a seam is no edge, so
                            // nor is it the end of one
                            if at.is_empty() && self.seam_from(f, wid) {
                                continue;
                            }
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
        // a run going on across a seam into the next edge of its smooth chain ends where that
        // chain ends (`chain_ends`), read at the seam
        for (run, v, vid) in going_on {
            for (c, sign) in self.chain_ends(run, faces, vid) {
                if !sides.iter().any(|&(y, sy, _)| y == EndBy::Face(c) && sy == sign) {
                    sides.push((EndBy::Face(c), sign, Some(v)));
                }
            }
        }
        sides
    }

    /// **Where a run going on across a seam ends** (#994, #962's 127): run `run` (between `faces`)
    /// ending on no face at vertex `vid`, where a face going on tangent from one of `faces` across
    /// a seam carries it on (`carries_on`) — an obround's half circle into its flat side — goes on
    /// along the next edge of its smooth chain: the one edge at `vid` of the same turn, no seam,
    /// whose faces are each one of `faces` or one going on from one of them tangent there
    /// (`continues`); and on from that edge's far end in the same way, until the chain ends on
    /// faces there (`ends_at`, read as that edge's own end) or goes on no further (a closed
    /// chain, a corner, a branch).  Each face the chain ends on whose solid it is kept out of
    /// (`kept_out_of`: a boss's cylinder it stands outside, a hole's) is the run's too, on the
    /// same side: a round never goes into another body's material nor into a hole, so the boss
    /// standing on the flat side that the round along it ends on keeps the round round the half
    /// circle off it too, where its wall reaches back over the seam's section — and nowhere else,
    /// for the round round the half circle lies outside the boss everywhere else.  But not one
    /// whose surface some edge of the run or of the chain between goes across (`goes_across`): no
    /// side of it holds the chain; nor one of the chain's own faces (`smooth`), which a chain round
    /// a loop comes back to (a side leaving at a corner the lobe the run goes on into the other
    /// way).  The run is one region's and the next edge another's (the half
    /// circle's and the flat side's), so the field reads the one solid in each, and the pieces of
    /// the two are cut alike by it at the seam they are glued at (`lowering::assemble`).
    fn chain_ends(&self, run: &[u32], faces: &[usize], vid: u32) -> Vec<(usize, f64)> {
        let turn = run.first().and_then(|&e| self.convex(e));
        let (mut path, mut pair, mut at) = (run.to_vec(), faces.to_vec(), vid);
        while path.len() <= self.loc.b.edges.len() {
            let v = self.back(self.loc.b.vertices[at as usize].p);
            let carried = |c: usize| self.carries_on(c, &pair, &path, v, at);
            if !self.faces_at(at, &pair).into_iter().any(carried) {
                return Vec::new();
            }
            // the next edge: of the same turn, no seam, between faces each one of the pair or going
            // on from one of them tangent here
            let mut next: Vec<u32> = self.faces_at(at, &[]).into_iter()
                .flat_map(|f| self.edges[f].iter().copied())
                .filter(|&e| self.touches(e, at) && !path.contains(&e) && !self.closed(e)
                    && !self.seam(e) && self.convex(e) == turn && self.carried(e)
                    && self.faces_of(e).iter().all(|&g| pair.contains(&g) || self.tangent_at(g, &pair, v, 1.0)))
                .collect();
            next.sort_unstable();
            next.dedup();
            let [e] = next[..] else { return Vec::new() };
            path.push(e);
            pair = self.faces_of(e).to_vec();
            let Some((w, d, wid)) = self.run_ends(&[e]).into_iter().find(|&(_, _, w)| w != at) else {
                return Vec::new();
            };
            if wid == vid {
                return Vec::new();
            }
            let ends = self.ends_at(&[e], &pair, w, d, wid, true);
            if !ends.is_empty() {
                // (none of the chain's own faces, the run's and all they go on into across seams:
                // a chain round a loop ends on a face the run goes on into the other way, the lobe
                // a side leaves at a corner)
                return ends.into_iter().filter(|&(c, sign)| {
                    !self.in_chains(faces, c)
                        && (self.kept_out_of(c, sign) || self.another_body(c, faces))
                        && !self.goes_across(c, sign, &path)
                }).collect();
            }
            at = wid;
        }
        Vec::new()
    }

    /// **Whether face `c` is another body's than every one of `faces`** (#994): each a face of a
    /// primitive the document drew (`FacePath::route`, none a round made, `FacePath::of`), and
    /// `c`'s another primitive than each of theirs — a boss's wall unioned onto a flange, a cutter's
    /// wall cut from it, beside the flange's top and side.  A round never takes another body's
    /// material, so a run ends on such a face, kept on its own side of it, however the face stands:
    /// one turning back over the run at a reflex corner (`ends_at`) — where a side of the run's own
    /// body is no end, the run ending square — and a plane the run's smooth chain ends on beyond a
    /// seam (`chain_ends`).  A body read off a boundary names no primitive: none is another's.
    fn another_body(&self, c: usize, faces: &[usize]) -> bool {
        let p = &self.paths[c];
        let drawn = |q: &FacePath| q.of.is_none() && !q.route.is_empty();
        drawn(p) && faces.iter().all(|&f| drawn(&self.paths[f]) && self.paths[f].route != p.route)
    }

    /// Whether a run kept on the side `sign` of face `c` (`σ·side <= 0`) is kept out of the solid
    /// its surface bounds: a cylinder, a cone or a sphere, the side that holds its axis or its
    /// centre the far one — a boss's wall the run stands outside, a hole's wall it stands beside.
    /// A plane's half-space, or a curved face's solid a run is kept within, is no such solid: it
    /// reaches as far as the surface goes, round a chain and back across it (a side nearly tangent
    /// to the lobe it leaves, whose plane cuts the arc before the lobe), and bounds the run only at
    /// its own end (`chain_ends`).
    fn kept_out_of(&self, c: usize, sign: f64) -> bool {
        // (the solid `level` reads its surface as: the material within it kept out of it from
        // outside, `sign` −1; the material outside it, from within, `sign` 1)
        matches!(self.level(c, 0.0), Some(Level::Bound(Bound::Cylinder { .. } | Bound::Ball { .. }
            | Bound::Cone { .. }, within)) if within == (sign < 0.0))
    }

    /// **The faces beyond a run rolled along's end** (`run` ending at vertex `vid`, `d` the way it
    /// runs from there, the section `reach` long; #994): the kernel's own reading
    /// (`brep::fillet::beyond_end`, which `fillet::end_at` cuts the run by) — its sides every
    /// face on the surfaces of the faces of its edge there (`run_sides`), its end faces those
    /// there going on from neither — each holding the run on its material's side, behind it; none
    /// of the faces `at` it ends on already.  One walk, so the field and the construction cannot
    /// disagree on which faces end a run.
    fn beyond_end(&self, run: &[u32], at: &[(usize, f64)], vid: u32, d: V, reach: f64)
        -> Vec<(usize, f64)> {
        let Some((sides, m)) = self.run_sides(run, vid) else { return Vec::new() };
        let b = &self.loc.b;
        crate::brep::fillet::beyond_end(b, &self.bounding, &sides, vid, m, d, reach * self.mm)
            .into_iter().filter(|&(g, _)| !at.iter().any(|&(c, _)| c == g))
            .map(|(g, _)| (g, 1.0)).collect()
    }

    /// **Whether a face cuts across run `run`'s end at vertex `vid`**, as the kernel reads it
    /// (`cutting`; `brep::fillet::goes_on`, `dome`'s `dome_cut`).  A ball standing still is
    /// glued to the runs going on at such an end only where no face cuts across it; else its dome
    /// is carried past and cut off there (`lowering::piece::run`), the kernel's reading of the
    /// same end.
    pub fn cut_across(&self, run: &[u32], vid: u32) -> bool { !self.cutting(run, vid).is_empty() }

    /// **The faces cutting across run `run`'s end at vertex `vid`, as the kernel reads them**
    /// (`brep::fillet::end_at`'s end faces, `beyond_end`'s start): each face there besides the
    /// run's sides (`run_sides`, `fillet::others_at`) going on from neither — its normal one with
    /// neither side's either way, `1 − |n·m|` over a billionth (`fillet::goes_on`).  Where a rolled
    /// run ends (`ends_at`), and whether a standing ball's end is cut across (`cut_across`): one
    /// reading, the kernel's.  None where no edge of the run ends there between two faces.
    fn cutting(&self, run: &[u32], vid: u32) -> Vec<usize> {
        let Some((sides, m)) = self.run_sides(run, vid) else { return Vec::new() };
        let b = &self.loc.b;
        let p = b.vertices[vid as usize].p;
        crate::brep::fillet::others_at(b, &self.bounding, &sides, vid).into_iter()
            .filter(|&f| !crate::brep::fillet::goes_on(b, f, p, m)).collect()
    }

    /// A run's sides at its end `vid`, as the kernel reads them (`fillet`'s `meeting`): every face
    /// on the surface of either face of the run's edge there (`same_surface`), and those two
    /// surfaces' unit normals at the vertex (millimetres).  `None` where no edge of the run ends
    /// there between two faces.
    fn run_sides(&self, run: &[u32], vid: u32) -> Option<(Vec<usize>, [V; 2])> {
        let e = self.run_edge_at(run, vid)?;
        let &[fa, fb] = self.faces_of(e) else { return None };
        let b = &self.loc.b;
        let sides = (0..b.faces.len())
            .filter(|&g| self.same_surface(g, fa) || self.same_surface(g, fb)).collect();
        let p = b.vertices[vid as usize].p;
        Some((sides, [fa, fb].map(|f| crate::space::unit(b.faces[f].surface.gradient(p)))))
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
    /// beyond it is the next one's, `tangent_at`); nor one whose surface holds the run's edge there
    /// (a plane along it, `lies_along`), nor one back to back with one of `faces` there, their
    /// outward normals opposite (`tangent_at` turned).  Such a face stands at no distance along the run and
    /// meets its faces only at the vertex or along the edge's own line past it, so it lies beyond
    /// the face the run ends on: a flange's underside rounded where a block it stands on runs out
    /// past its edge, the block's top in the underside's plane past the block's side, which ends
    /// the run; a boss's underside where it overhangs the side of the part whose top it is filled
    /// to; a flange's underside where a rib's sloped edge, rounded against the flange's end, starts
    /// at the flange's edge (#996).  No side of such a face is the run's: kept within it a convex
    /// run was cut whole or along its length, the section of an edge whose faces it splits lying
    /// on both sides.  Nor one whose surface the run itself goes across away from this
    /// end — no side of it holds the run (a plane all but along a lug's rim where a plate's
    /// side runs into it, the arc dipping under the plane and back) — nor a plane the run stands
    /// outside of turning back over a face of the run it meets in an edge there (a reflex corner of
    /// less than a right angle; a plane meeting that face at the vertex alone makes no corner with
    /// it: the block's side, crossing the flange's rim below the rounded side of its corner),
    /// whose half-space reaches back over the section from the end, the more the shallower the
    /// turn (a whole side's length at a few degrees): past the end it cuts all of the run, and
    /// alongside nothing is past the end.  The run ends there square, as at a seam (each where
    /// `plain`, `end_faces_as`).  A plane turns back over the run's faces only where a patch of
    /// them reaches in behind it: a plate a band tapers onto, the band's faces wholly on its
    /// outside, bounds the run along its length, and the round of the band's top edge is kept
    /// over the plate, not cut on square into it (#994).  The side a face is run on is read a step
    /// along the run's tangent — or, where that lies in the face and the run's curved edge bends
    /// away outside it (a foot arc leaving the plate tangent), at the edge's middle.  A run that is
    /// not `plain` (a ball rolled along a traced run) ends on the faces the kernel's `end_at` cuts
    /// it by, by its rule (`cutting`: tangent where `1 − |n·m|` is within a billionth), not this
    /// one's (`TANGENT`), so the field and the construction read one set of end faces (#994).
    fn ends_at(&self, run: &[u32], faces: &[usize], v: V, d: V, vid: u32, plain: bool)
        -> Vec<(usize, f64)> {
        let on = along_run(v, d, 1.0);
        // (a plane turning back over the run's faces: one of them leaning toward its outside that
        // it meets in an edge at this vertex — one it meets at the vertex alone is no side of that
        // corner (#996) — and some patch of them reaching in behind it — not a plate a band tapers
        // onto, whose faces stand wholly on the plate's outside: that one bounds the run, #994)
        let behind = |c: usize, f: usize| self.goes_across(c, -1.0, &self.edges[f]);
        let back = |c: usize| self.plane(c).is_some_and(|(n, _)| {
            faces.iter().any(|&f| dot(n, self.normal(f, v)) > 1e-9 && self.meet_at(c, f, vid))
                && faces.iter().any(|&f| f != c && behind(c, f))
        });
        // (no end: a face carrying the run on across a seam (`carries_on`, #994), one back to back
        // with one of the run's faces there, or one whose surface holds the run's edge (#996); a
        // rolled run's ends the kernel's own, `cutting`)
        let ends: Vec<usize> = if plain {
            self.faces_at(vid, faces).into_iter().filter(|&c| {
                !(self.carries_on(c, faces, run, v, vid) || self.tangent_at(c, faces, v, -1.0)
                    || self.lies_along(c, run, vid))
            }).collect()
        } else {
            self.cutting(run, vid)
        };
        // (the side of `c` the run runs on: read a step along its tangent — or, where that lies in
        // `c` and the run's edge, curved, leaves it tangent and bends away outside it (a band's
        // foot arc off the plate it tapers onto, #994), outside: the edge's middle says so where
        // the step cannot.  An edge bending in, within `c`, as at a seam, is read by the step)
        let middle = self.run_edge_at(run, vid).filter(|&e| self.segment(e).is_none())
            .map(|e| self.along(e, 0.5));
        let runs_on = |c: usize| match middle {
            Some(m) if self.side(c, on).abs() <= self.past(on) && self.side(c, m) > self.past(m) => self.side(c, m),
            _ => self.side(c, on),
        };
        ends.into_iter().filter_map(|c| {
            let sign = if runs_on(c) <= 0.0 { 1.0 } else { -1.0 };
            // (another body's wall turning back over the run is still its end: square there, the
            // run went on into that body's material — #994, `another_body`)
            let across = plain && (self.goes_across(c, sign, run)
                || sign < 0.0 && back(c) && !self.another_body(c, faces));
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
    /// keeps (`sign · side > 0`), by more than `past`: exactly for a plane against a line or a
    /// circle (`plane_past`), at 64 points of an edge otherwise.
    fn goes_across(&self, c: usize, sign: f64, es: &[u32]) -> bool {
        es.iter().any(|&e| self.plane_past(c, sign, e)
            .unwrap_or_else(|| (0..=64).any(|k| self.beyond(c, sign, self.along(e, k as f64 / 64.0)))))
    }

    /// Whether `p` stands past face `c`'s surface on the side `sign` keeps, by more than `past`.
    fn beyond(&self, c: usize, sign: f64, p: V) -> bool { sign * self.side(c, p) > self.past(p) }

    /// Whether some point of edge `e` stands past face `c`'s surface on the side `sign` keeps
    /// (`beyond`), in closed form where `c` is a plane and `e` a line or a circle; `None` for any
    /// other, to be read at points of the edge.
    fn plane_past(&self, c: usize, sign: f64, e: u32) -> Option<bool> {
        let (n, _) = self.plane(c)?;
        let edge = &self.loc.b.edges[e as usize];
        let out = |p: V| self.beyond(c, sign, p);
        match &edge.curve {
            EdgeCurve::Curve(crate::brep::geom::Curve::Line { .. }) => {
                Some(out(self.along(e, 0.0)) || out(self.along(e, 1.0)))
            }
            EdgeCurve::Curve(crate::brep::geom::Curve::Circle(f, _)) => {
                // `side` along a circle is `side(centre) + r (a cos t + b sin t)`: at its
                // ends, and where it is greatest the way `sign` reads it, if within them
                let (t0, t1) = (edge.t[0].min(edge.t[1]), edge.t[0].max(edge.t[1]));
                let peak = (sign * dot(n, f.y)).datan2(sign * dot(n, f.x));
                let turn = (peak - t0).rem_euclid(2.0 * PI);
                let at = |t: f64| self.back(edge.point(t, &self.loc.b.vertices));
                Some(out(at(t0)) || out(at(t1)) || (t0 + turn <= t1 && out(at(t0 + turn))))
            }
            _ => None,
        }
    }

    /// Edges `es` (between `faces`) in runs (`runs`), each with where it ends (`end_faces`): a
    /// crease a slot cuts across ends on the slot's sides once for each run (#881), and the ends
    /// of runs read as one run's are within none of them.
    pub fn end_runs(&self, es: &[u32], faces: &[usize], reach: Option<f64>)
        -> Vec<(Vec<u32>, Vec<(EndBy, f64)>)> {
        let plain = es.iter().all(|&e| self.carried(e));
        self.runs(es).into_iter().map(|run| {
            let sides = self.end_faces_as(&run, faces, plain, reach).into_iter()
                .map(|(c, sign, _)| (c, sign)).collect();
            (run, sides)
        }).collect()
    }

    /// `faces` and every face going on from one of them tangent across a seam, and from those
    /// on (`F.smooth`, [0.149]): an edge two of the boundary's faces bound, where their outward
    /// normals are one at its middle, is no edge but a seam.  In the faces' order, then as met
    /// (`across_seams`).
    pub fn smooth(&self, faces: &[usize]) -> Vec<usize> {
        self.across_seams(faces, |_| true).into_iter().map(|(fi, _)| fi).collect()
    }

    /// **The walk across seams**: `from`, then every face `keep` holds going on from one of them
    /// tangent across a seam (`seam`), and from those on — breadth first, in `from`'s order and
    /// then as met, each edge of a face in its order and each face of an edge in its — each face
    /// once, with the place in the walk of the face it was reached from (`None` for one of
    /// `from`).  A smooth chain (`smooth`), the pieces of one surface (`pieces`) and the
    /// way through a chain to one of its faces (`lowering::piece::reached`) are this walk.
    pub fn across_seams(&self, from: &[usize], keep: impl Fn(usize) -> bool) -> Vec<(usize, Option<usize>)> {
        let mut out: Vec<(usize, Option<usize>)> = from.iter().map(|&fi| (fi, None)).collect();
        // each face met once, told at once: a chain round a plate's pockets is most of the body
        let mut met = vec![false; self.edges.len()];
        for &fi in from {
            met[fi] = true;
        }
        let mut k = 0;
        while k < out.len() {
            let fi = out[k].0;
            for &e in self.edges[fi].iter().filter(|&&e| self.seams[e as usize]) {
                for &g in self.faces_of(e) {
                    if !met[g] && keep(g) {
                        met[g] = true;
                        out.push((g, Some(k)));
                    }
                }
            }
            k += 1;
        }
        out
    }

    /// Whether face `fi` goes on tangent across a seam (`seam`) at vertex `vid`.
    fn seam_from(&self, fi: usize, vid: u32) -> bool {
        self.edges[fi].iter().any(|&e| self.touches(e, vid) && self.seam(e))
    }

    /// Whether edge `e` is a seam: two faces bound it whose outward normals are one at its middle,
    /// so they meet tangent there and it is no edge.  Read once for the body (`seams`).
    pub fn seam(&self, e: u32) -> bool { self.seams[e as usize] }

    /// `seam`, read off the boundary.
    fn seam_at(&self, e: u32) -> bool {
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
                .filter(|&e| self.touches(e, v) && !self.seam(e))
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
            !not.contains(fi) && self.edges[*fi].iter().any(|&e| self.touches(e, v))
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
    /// A smooth reference whose named faces go on into chains apart (#962): the named faces each
    /// chain holds, with the chain (`BodyFaces::chains_apart`).  Empty where they are one chain.
    pub(crate) chains: Vec<(Vec<usize>, Vec<usize>)>,
    /// Faces the point a set reads these by is held outside of (`true`) or within, none of them
    /// these (#994): a point in the plane of one the chain's rim is to reads the chain as one just
    /// past it or just within it (`BodyFaces::nearest_face`).
    pub(crate) held: Vec<(usize, bool)>,
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
    let named = body.faces(f);
    // a name in pieces that go on into chains apart (two rounds' caps under one name, one on a
    // step's side and one on its wall): each piece read with its own chain, not the others' (one
    // face is one chain)
    let chains = if f.smooth && named.len() > 1 {
        Some(body.chains_apart(&named)).filter(|c| c.len() > 1).unwrap_or_default()
    } else {
        Vec::new()
    };
    let faces = if f.smooth { body.smooth(&named) } else { named };
    Ok(Reading { body, faces, smooth: f.smooth, chains, held: Vec::new() })
}

/// **Faces read as one body's** (#961): the readings of `refs`, which name the faces an edge, a
/// corner or the material's angle there is read between — refused where they are faces of two
/// solids.  A reading's face numbers are its own body's, and an edge is two faces of one body
/// meeting: read in another body's lists, one body's numbers name some other face of it, or
/// none (one solid's face list indexed with another's face number).
pub fn one_body(sk: &Sketch, refs: &[&FaceRef], read: &mut Read) -> Result<Vec<Reading>, String> {
    if let Some(other) = refs.iter().find(|f| f.solid != refs[0].solid) {
        return Err(not_one_body(sk, refs[0].solid as usize, other.solid as usize));
    }
    refs.iter().map(|f| reading(sk, f, read)).collect()
}

/// Why faces of solids `a` and `b` have no edge, corner or angle between them: they are not one
/// body's.
pub fn not_one_body(sk: &Sketch, a: usize, b: usize) -> String {
    format!("`{}` and `{}` are not one body: faces meet in an edge of the one body both are faces \
             of", sk.solid_name(a), sk.solid_name(b))
}

impl Reading {
    /// The signed distance from the surface of the face whose patch is nearest `p`.
    pub fn side(&self, p: V) -> f64 {
        self.nearest(p).map_or(f64::INFINITY, |fi| self.body.side(fi, p))
    }
    /// The face whose patch is nearest `p` (one face is, measuring nothing).
    /// Of a name in pieces whose chains lie apart, in the chain of the named piece nearest `p`.
    fn nearest(&self, p: V) -> Option<usize> {
        if let [fi] = self.faces[..] {
            return Some(fi);
        }
        let near = |fs: &[usize]| self.body.nearest_face(fs, p, &self.held);
        if !self.chains.is_empty() {
            let (_, chain) = self.body.nearest_named(p, &self.chains, |c| &c.0[..])?;
            return near(chain);
        }
        near(&self.faces)
    }
    /// The distance from the faces' patches, or (`surface`) their surfaces — a smooth surface's
    /// the one of the piece whose patch is nearest, so it follows the faces across their seams
    /// and runs on past their edges as the nearest piece's surface does.
    pub fn distance(&self, p: V, surface: bool) -> f64 {
        if self.smooth && surface {
            return self.side(p).abs();
        }
        if !surface {
            return self.body.nearest_distance(&self.faces, p);
        }
        self.faces.iter().map(|&fi| self.body.side(fi, p).abs()).fold(f64::INFINITY, f64::min)
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
        let mut pieces: Vec<usize> = Vec::new();
        let group = self.faces.iter().map(|&fi| {
            pieces.iter().position(|&q| self.body.same_surface(q, fi))
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

    /// Where the faces' side is convex toward the side read — within them, or (`beyond`) outside
    /// them (`BodyFaces::bulging`), each of its faces a condition: whether each is read by its side
    /// carried on past its patch (a smooth chain) or by its whole surface's (one face).
    pub fn bulging(&self, beyond: bool) -> Option<bool> {
        self.body.bulging(&self.faces, self.smooth, beyond)
    }
}
