//! **A body shelled, built exactly** (`std.rounds`' `shell`): the word cuts from a body its
//! cavity, the points within the body at least `t` from each of its faces not left open — `{ c |
//! c inside B }` with each kept face's wall `{ p | p distance(<= t) w }` cut from it, or, every
//! face closed, the one region `{ p | p inside B; distance(p, beyond) >= t }`.  This module
//! recognises that cavity by what its sets say and builds it, where the body is a prism with
//! nothing put on it and solids cut from it.
//!
//! Within a prism the distance from its boundary is the lesser of the distance from its section's
//! edges, square to its axis, and from its caps along it: a point's foot on a side is level with
//! it, and on a cap stands over it.  So the points of a prism at least `t` from its kept faces are
//! a prism: its section's points at least `t` from its kept edges, between its kept caps moved in
//! by `t`.  That section is the section's edges each moved in by `t`, meeting where two meet at a
//! convex corner and joined by an arc of radius `t` about a reflex one; an open edge, or an open
//! cap, is moved out instead by half the thickness, so the cavity runs out through the opening and
//! crosses the body's faces there, never lying on them; where a kept edge meets an open one at
//! more than a right angle (a wall leaning in over the opening, or standing on it in a reflex
//! corner) the arc about the corner runs on to the open edge moved out.  A solid cut from the
//! prism takes from the cavity what lies within `t` of it (the cavity of `A` less `P` is `A`'s less
//! `P` grown by `t`), built as a body grown is (`grown::around`).  The cavity is the recipe's
//! `region`: the prism kept within, each cut grown kept outside; it lies on none of the body's
//! faces, so cutting it from the body meets them only across.  Held to the body's field before it stands (`gate`).
//!
//! Refused by name, and read by the field: a body whose stock is no prism of lines (a drafted one,
//! a revolution, a body of its own), with material put on it or bounding it, rounded as well; a
//! face left open that is none of the stock's or is only part of one; a section whose walls meet
//! or cross (a face narrower than twice the thickness); and a cut no body grown builds.
use crate::brep::build::{Profile, ProfileEdge};
use crate::brep::geom::Frame;
use crate::brep::recipe::Writer;
use crate::json::{object, Json};
use crate::model::{FaceRef, RegionShape, RegionTerm, Sketch, SolidDef, Target, TermNumber};
use crate::space::{add, dot, norm, scale, sub};
use crate::vocab::Cmp;
use super::faces::BodyFaces;
use std::f64::consts::{FRAC_PI_2, PI};
use std::rc::Rc;
use std::sync::Arc;

#[allow(unused_imports)]
use crate::fmath::Det;

type V = [f64; 3];
type P = [f64; 2];

/// A shell's cavity built: a prism over its section, less each solid cut from the body grown by
/// the thickness (model units).
#[derive(Clone, Debug)]
pub struct Shelled {
    /// The section, in the plane the prism starts from, and how far it runs along its normal.
    section: Profile,
    length: f64,
    /// Each solid cut from the body, grown by the thickness.
    cuts: Vec<super::grown::Grown>,
    /// The solids the cavity is made of, which its construction stands for.
    under: Vec<u32>,
}

impl Shelled {
    /// The recipe's node, written by `w`: the cavity's `prism`, or a `region` keeping within it and
    /// outside each cut grown.
    pub fn node(&self, w: &Writer) -> Result<Json, String> {
        let mut prism = object([("kind", "prism".into()), ("profile", self.section.to_json(w)?),
            ("from", 0.0.into()), ("to", (self.length * w.scale).into())]);
        if self.cuts.is_empty() {
            return Ok(prism);
        }
        prism.set("keep", Json::Bool(true));
        let mut pieces = vec![prism];
        for g in &self.cuts {
            let mut grown = g.node(w)?;
            grown.set("keep", Json::Bool(false));
            pieces.push(grown);
        }
        Ok(object([("kind", "region".into()), ("pieces", Json::Arr(pieces))]))
    }

    /// The solids whose recipe nodes its own reads: the cuts its grown pieces are glued to.
    pub fn cuts(&self) -> Vec<u32> { self.cuts.iter().map(|g| g.body()).collect() }

    /// The solids the cavity is made of — its stock and walls, or nothing but itself — which no
    /// construction of their own builds.
    pub fn under(&self) -> &[u32] { &self.under }
}

/// What a cavity's sets say: the body it is within, its thickness (each wall's, or the region's
/// bound), the faces kept (none named where every face is), and the solids it is made of.
struct Recognised {
    body: u32,
    thickness: Vec<TermNumber>,
    kept: Option<Vec<FaceRef>>,
    under: Vec<u32>,
}

/// The body whose material `t` bounds, the inside of every face of it (`Le`) or the outside
/// (`Ge`) at nothing: `{ c | c inside B }`'s term, or `{ c | c outside B }`'s.
fn whole(t: &RegionTerm, cmp: Cmp) -> Option<u32> {
    let RegionShape::FaceSide { face } = &t.shape else { return None };
    let at_nothing = t.lo.free.is_none() && t.lo.c == 0.0;
    (t.cmp == cmp && at_nothing && t.hi.is_none() && t.grow.is_none() && face.path.is_empty() && !face.surface
        && !face.smooth).then_some(face.solid)
}

/// Solid `solid` read as a shell's cavity, where it is one: open, the region within a body's
/// faces with each kept face's wall cut from it; closed, the region within them at least the
/// thickness from what is outside.
fn recognised(sk: &Sketch, solid: usize) -> Option<Recognised> {
    match &sk.solids.get(solid)?.def {
        SolidDef::Body { stock, on, through, bound } if on.is_empty() && bound.is_empty() && !through.is_empty() => {
            let SolidDef::Region { terms, .. } = &sk.solids.get(*stock as usize)?.def else { return None };
            let [inside] = &terms[..] else { return None };
            let body = whole(inside, Cmp::Le)?;
            let (mut thickness, mut kept) = (Vec::new(), Vec::new());
            for &w in through {
                let SolidDef::Region { terms, .. } = &sk.solids.get(w as usize)?.def else { return None };
                let [t] = &terms[..] else { return None };
                let RegionShape::Measured { of } = &t.shape else { return None };
                let [(Target::Face(f), c)] = &of[..] else { return None };
                if *c != 1.0 || f.solid != body || f.surface || f.smooth || t.cmp != Cmp::Le || t.hi.is_some()
                    || t.grow.is_some() {
                    return None;
                }
                thickness.push(t.lo.clone());
                kept.push(f.clone());
            }
            let under = std::iter::once(*stock).chain(through.iter().copied()).collect();
            Some(Recognised { body, thickness, kept: Some(kept), under })
        }
        SolidDef::Region { terms, .. } => {
            let [inside, reach] = &terms[..] else { return None };
            let body = whole(inside, Cmp::Le)?;
            let RegionShape::Measured { of } = &reach.shape else { return None };
            let [(Target::Set { terms: beyond, .. }, c)] = &of[..] else { return None };
            let [beyond] = &beyond[..] else { return None };
            (*c == 1.0 && whole(beyond, Cmp::Ge) == Some(body) && reach.cmp == Cmp::Ge && reach.hi.is_none()
                && reach.grow.is_none())
                .then(|| Recognised { body, thickness: vec![reach.lo.clone()], kept: None, under: Vec::new() })
        }
        _ => None,
    }
}

/// Solid `solid` built as a shell's cavity, where it is one (`None` otherwise): the construction,
/// or why it stands not.  Remembered against what it reads: its sets' numbers and the stratum of
/// the body it hollows (`Caches::shelled`).
pub fn built(sk: &Sketch, solid: usize) -> Option<Result<Rc<Shelled>, String>> {
    let r = recognised(sk, solid)?;
    let strata = super::faces::cached(sk, r.body as usize).map(|b| vec![b]).unwrap_or_default();
    let key = r.under.iter().fold(super::MemoKey::own(sk, solid, strata),
        |k, &u| k.and(super::MemoKey::own(sk, u as usize, Vec::new())));
    sk.caches.shelled.get_or(solid, key, || {
        Some(construction(sk, solid, &r).and_then(|(s, body)| {
            gate(sk, solid, r.body as usize, &s, &body)?;
            Ok(Rc::new(s))
        }))
    })
}

/// One piece of a section's loop in its plane: a stretch of a line, named for the face it is
/// moved in from, or an arc of radius `r` about `c` turning from angle `from` to `to`.
#[derive(Clone, Debug)]
enum Piece {
    Line { a: P, b: P, name: String },
    Arc { c: P, r: f64, from: f64, to: f64 },
}

/// An edge of the prism's section: its ends (the material on its left), whether the face it
/// sweeps is left open, and that face's name.
#[derive(Clone, Debug)]
struct Edge {
    a: P,
    b: P,
    open: bool,
    name: String,
}

fn cross2(a: P, b: P) -> f64 { a[0] * b[1] - a[1] * b[0] }
fn dot2(a: P, b: P) -> f64 { a[0] * b[0] + a[1] * b[1] }
fn sub2(a: P, b: P) -> P { [a[0] - b[0], a[1] - b[1]] }
fn add2(a: P, b: P) -> P { [a[0] + b[0], a[1] + b[1]] }
fn scale2(a: P, k: f64) -> P { [a[0] * k, a[1] * k] }
fn polar(c: P, r: f64, angle: f64) -> P { [c[0] + r * angle.dcos(), c[1] + r * angle.dsin()] }

/// The cavity of the body `r` names, and the faces of that body as its other terms make them.
fn construction(sk: &Sketch, solid: usize, r: &Recognised) -> Result<(Shelled, Arc<BodyFaces>), String> {
    let mut thickness = r.thickness.iter().map(|n| n.read(sk).ok_or("its thickness reads an unknown nothing determines"));
    let t = thickness.next().ok_or("a shell of no thickness")??;
    for other in thickness {
        if (other? - t).abs() > 1e-12 * t.abs() {
            return Err("walls of several thicknesses are read by the field".to_string());
        }
    }
    if !(t > 0.0) {
        return Err("a shell no thicker than nothing".to_string());
    }
    let body = r.body as usize;
    let SolidDef::Body { stock, on, through, bound } = &sk.solids[body].def else {
        return Err("a body shelled that is no body of terms".to_string());
    };
    let readers = super::faces::readers_of(sk, body);
    let reads = |o: &u32| readers.contains(&(*o as usize));
    if on.iter().chain(through).any(|o| reads(o) && *o as usize != solid) {
        return Err("a body shelled and rounded as well is read by the field".to_string());
    }
    if !on.is_empty() || !bound.is_empty() {
        return Err("a body shelled with material put on it or bounding it is read by the field".to_string());
    }
    let cut: Vec<u32> = through.iter().copied().filter(|o| !reads(o)).collect();
    if !matches!(sk.solids[*stock as usize].def, SolidDef::Prism { draft: None, .. }) {
        return Err("a body shelled whose stock is no prism is read by the field".to_string());
    }
    let prism = super::faces::cached(sk, *stock as usize)?;
    let faces = super::faces::cached(sk, body)?;
    let (section, length) = cavity(&prism, &faces, r.kept.as_deref(), t)?;
    let cuts = cut.iter().map(|&c| super::grown::around(sk, c as usize, t, "round")
        .map_err(|why| format!("the cut `{}` grown by the thickness: {why}", sk.solid_name(c as usize))))
        .collect::<Result<Vec<_>, String>>()?;
    Ok((Shelled { section, length, cuts, under: r.under.clone() }, faces))
}

/// The cavity's prism within the prism whose faces are `prism`, in the body whose faces are
/// `body` (the prism less what is cut from it), leaving open the body's faces `kept` does not name
/// (none where it is `None`), at thickness `t`: its section, in the plane it starts from, and how
/// far it runs.
fn cavity(prism: &BodyFaces, body: &BodyFaces, kept: Option<&[FaceRef]>, t: f64) -> Result<(Profile, f64), String> {
    let refused = |why: &str| format!("{why}: read by the field");
    let (b, _, _) = prism.brep();
    let named = |name: &str| (0..prism.paths().len()).find(|&i| prism.paths()[i].face == name && prism.paths()[i].route.is_empty());
    let (Some(far), Some(near)) = (named("far"), named("near")) else {
        return Err(refused("a prism whose caps are not named so"));
    };
    let planes = (0..b.faces.len()).map(|i| prism.plane(i)).collect::<Option<Vec<_>>>()
        .ok_or_else(|| refused("a prism whose sides are not all planes"))?;
    let (lo, hi) = prism.bounds();
    let tol = 1e-9 * norm(sub(hi, lo)).max(1.0);
    // which of the prism's faces are left open: those the body's open faces lie on
    let mut open = vec![false; b.faces.len()];
    let shut: Vec<bool> = match kept {
        None => vec![true; body.paths().len()],
        Some(refs) => {
            let mut k = vec![false; body.paths().len()];
            for f in refs {
                for i in body.faces(f) {
                    k[i] = true;
                }
            }
            k
        }
    };
    let on = |(n, o): (V, V), (m, q): (V, V)| dot(n, m) > 1.0 - 1e-9 && dot(sub(o, q), m).abs() <= tol;
    for fi in (0..shut.len()).filter(|&i| !shut[i]) {
        let p = body.plane(fi).ok_or_else(|| refused("a face left open that is no plane"))?;
        let s = (0..planes.len()).find(|&j| on(p, planes[j]))
            .ok_or_else(|| refused("a face left open that is none of the stock's (a cut's face is never left open)"))?;
        open[s] = true;
    }
    for fi in (0..shut.len()).filter(|&i| shut[i]) {
        if body.plane(fi).is_some_and(|p| (0..planes.len()).any(|j| open[j] && on(p, planes[j]))) {
            return Err(refused("a face of the stock left open in part only"));
        }
    }
    // the section, on the far cap, in a frame about the prism's axis
    let axis = scale(planes[far].0, -1.0);
    let frame = Frame::about(planes[far].1, axis);
    let flat = |p: V| { let d = sub(p, frame.o); [dot(d, frame.x), dot(d, frame.y)] };
    let mut loops = Vec::new();
    for l in &b.faces[far].loops {
        let mut edges = Vec::new();
        for c in l {
            let [p, q] = prism.segment(c.edge).ok_or_else(|| refused("a prism whose section has an edge no line carries"))?;
            let (a, z) = if c.reversed { (q, p) } else { (p, q) };
            let side = prism.faces_of(c.edge).iter().copied().find(|&g| g != far)
                .ok_or_else(|| refused("a prism's edge bounding one face"))?;
            edges.push(Edge { a: flat(a), b: flat(z), open: open[side], name: prism.paths()[side].face.clone() });
        }
        loops.push(edges);
    }
    // outer first and turning left, holes turning right, so the material is on each edge's left
    let area = |l: &[Edge]| l.iter().map(|e| cross2(e.a, e.b)).sum::<f64>() / 2.0;
    loops.sort_by(|x, y| area(y).abs().total_cmp(&area(x).abs()));
    for (k, l) in loops.iter_mut().enumerate() {
        if (area(l) > 0.0) != (k == 0) {
            l.reverse();
            for e in l.iter_mut() {
                std::mem::swap(&mut e.a, &mut e.b);
            }
        }
    }
    let out = t / 2.0;
    let eroded = loops.iter().map(|l| eroded(l, t, out, tol)).collect::<Result<Vec<_>, _>>().map_err(refused)?;
    simple(&eroded, tol).map_err(refused)?;
    // between the caps, each moved in by the thickness or out through an opening
    let height = dot(sub(planes[near].1, planes[far].1), axis);
    let from = if open[far] { -out } else { t };
    let to = if open[near] { height + out } else { height - t };
    if !(to - from > tol) {
        return Err(refused("a prism no longer than twice the thickness between its caps"));
    }
    let lift = |p: P| add(frame.o, add(add(scale(frame.x, p[0]), scale(frame.y, p[1])), scale(axis, from)));
    let mut names = Vec::new();
    let mut section = Vec::new();
    for l in &eroded {
        let (mut edges, mut called) = (Vec::new(), Vec::new());
        for piece in l {
            match piece {
                Piece::Line { a, b, name } => {
                    edges.push(ProfileEdge::Line { a: lift(*a), b: lift(*b) });
                    called.push(name.clone());
                }
                Piece::Arc { c, r, from, to } => {
                    let f = Frame::new(lift(*c), axis, frame.x);
                    edges.push(ProfileEdge::Arc { frame: f, r: *r, span: Some([from.min(*to), from.max(*to)]) });
                    called.push("round".to_string());
                }
            }
        }
        section.push(edges);
        names.push(called);
    }
    Ok((Profile { origin: lift([0.0, 0.0]), normal: axis, loops: section, names }, to - from))
}

/// A loop of the section moved in by `t` from each edge kept and out by `out` from each open one:
/// each edge's stretch of its line moved, and between two the corner where the lines meet, or an
/// arc of radius `t` about the corner where the points within `t` of it reach past both lines — a
/// reflex corner of two kept edges, or one where a kept edge meets an open one at more than a right
/// angle (the arc then run on to the open edge's line).  Refused where a stretch turns round: the
/// walls of a face meet.
fn eroded(l: &[Edge], t: f64, out: f64, tol: f64) -> Result<Vec<Piece>, &'static str> {
    const MEET: &str = "a face narrower than twice the thickness, its walls meeting";
    let m = l.len();
    if m < 3 {
        return Err("a section of fewer than three edges");
    }
    let along: Vec<P> = l.iter().map(|e| { let d = sub2(e.b, e.a); scale2(d, 1.0 / dot2(d, d).sqrt()) }).collect();
    let left = |k: usize| [-along[k][1], along[k][0]];
    let by = |k: usize| if l[k].open { -out } else { t };
    let base = |k: usize| add2(l[k].a, scale2(left(k), by(k)));
    let meet = |k: usize, j: usize| {
        let (p, q) = (base(k), base(j));
        add2(p, scale2(along[k], cross2(sub2(q, p), along[j]) / cross2(along[k], along[j])))
    };
    let angle = |v: P| v[1].datan2(v[0]);
    // at each corner, between edge `k` and the next: where the first stretch ends, and the next
    // begins, and the arc between them
    let mut corners: Vec<(P, P, Option<Piece>)> = Vec::new();
    for k in 0..m {
        let j = (k + 1) % m;
        let (cr, dt) = (cross2(along[k], along[j]), dot2(along[k], along[j]));
        if cr.abs() <= 1e-12 {
            return Err("a section two of whose edges run on in one line");
        }
        let turn = cr.datan2(dt);
        let interior = PI - turn;
        let v = l[k].b;
        // the arc turns clockwise about the corner: the cavity lies outside its circle
        let reach = (-out / t).dacos() - turn;
        let arc = match (l[k].open, l[j].open) {
            (false, false) if turn < 0.0 => Some((angle(left(k)), angle(left(k)) + turn)),
            (false, true) if interior > FRAC_PI_2 + 1e-9 => Some((angle(left(k)), angle(left(k)) - reach)),
            (true, false) if interior > FRAC_PI_2 + 1e-9 => Some((angle(left(j)) + reach, angle(left(j)))),
            _ => None,
        };
        corners.push(match arc {
            Some((from, to)) => (polar(v, t, from), polar(v, t, to), Some(Piece::Arc { c: v, r: t, from, to })),
            None => { let p = meet(k, j); (p, p, None) }
        });
    }
    let mut pieces = Vec::new();
    for k in 0..m {
        let (start, end) = (corners[(k + m - 1) % m].1, corners[k].0);
        if !(dot2(sub2(end, start), along[k]) > tol) {
            return Err(MEET);
        }
        pieces.push(Piece::Line { a: start, b: end, name: if l[k].open { "open".to_string() } else { l[k].name.clone() } });
        if let Some(arc) = corners[k].2.clone() {
            pieces.push(arc);
        }
    }
    Ok(pieces)
}

/// Whether the loops cross nowhere: no two stretches of them meet but where one goes on from the
/// next (each arc read as chords a sixteenth of a half turn long at most).
fn simple(loops: &[Vec<Piece>], tol: f64) -> Result<(), &'static str> {
    const CROSS: &str = "a section whose walls cross (a face narrower than twice the thickness)";
    let chords: Vec<Vec<[P; 2]>> = loops.iter().map(|l| l.iter().flat_map(|p| match p {
        Piece::Line { a, b, .. } => vec![[*a, *b]],
        Piece::Arc { c, r, from, to } => {
            let n = ((to - from).abs() / (PI / 16.0)).ceil().max(1.0) as usize;
            (0..n).map(|i| [polar(*c, *r, from + (to - from) * i as f64 / n as f64),
                            polar(*c, *r, from + (to - from) * (i + 1) as f64 / n as f64)]).collect()
        }
    }).collect()).collect();
    // two stretches meet where each straddles the other's line, or an end of one lies on the other
    let on = |p: P, [a, b]: [P; 2]| {
        let ab = sub2(b, a);
        let s = (dot2(sub2(p, a), ab) / dot2(ab, ab).max(f64::MIN_POSITIVE)).clamp(0.0, 1.0);
        let off = sub2(p, add2(a, scale2(ab, s)));
        dot2(off, off).sqrt() <= tol
    };
    let crosses = |x: [P; 2], y: [P; 2]| {
        let side = |[p, q]: [P; 2], r: P| cross2(sub2(q, p), sub2(r, p));
        side(x, y[0]) * side(x, y[1]) < 0.0 && side(y, x[0]) * side(y, x[1]) < 0.0
            || on(y[0], x) || on(y[1], x) || on(x[0], y) || on(x[1], y)
    };
    for (i, li) in chords.iter().enumerate() {
        for (j, lj) in chords.iter().enumerate().skip(i) {
            for (x, &p) in li.iter().enumerate() {
                for (y, &q) in lj.iter().enumerate() {
                    let next = i == j && (y == x + 1 || (x == 0 && y + 1 == lj.len()));
                    if (i == j && y <= x) || next {
                        continue;
                    }
                    if crosses(p, q) {
                        return Err(CROSS);
                    }
                }
            }
        }
    }
    Ok(())
}

/// **The gate**, as the other constructions': the cavity stands only where the body it is cut
/// from, built so, agrees with the body's own field — the body as its other terms make it, built
/// about the boundary they make, less the cavity.
fn gate(sk: &Sketch, solid: usize, body: usize, s: &Shelled, faces: &BodyFaces) -> Result<(), String> {
    let (stratum, mm, origin) = faces.brep();
    let w = Writer::about(mm, origin.map(|x| x * mm));
    let mut built = std::collections::BTreeMap::new();
    for c in s.cuts() {
        // each cut's boundary moved from its own origin to the body's
        let cut = super::faces::cached(sk, c as usize)?;
        let (b, _, at) = cut.brep();
        let shift = crate::brep::geom::Rigid { t: scale(sub(at, origin), mm), ..crate::brep::geom::Rigid::identity() };
        built.insert(c as i64, b.moved(&shift));
    }
    let cavity = crate::brep::recipe::node(&s.node(&w)?, &built)?;
    let shelled = crate::brep::boolean::combined(stratum, &cavity, crate::brep::boolean::Op::Cut, 0.0)
        .map_err(|e| format!("`{}`: {}", sk.solid_name(solid), String::from(e)))?;
    let field = super::MaterialField::read(sk, body, super::cad::AXIS_TOLERANCE)?;
    let back = |p: V| faces.back(p);
    super::agreement::gate(&field, &shelled, Some((&back, mm)), "its exact construction disagrees with the field")
}
