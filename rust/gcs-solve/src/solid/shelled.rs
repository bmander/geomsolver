//! **A body shelled, built exactly** (`std.rounds`' `shell`): the word cuts from a body its
//! cavity, the points within the body at least `t` from each of its faces not left open — `{ c |
//! c inside B }` with each kept face's wall `{ p | p distance(<= t) w }` cut from it, or, every
//! face closed, the one region `{ p | p inside B; distance(p, beyond) >= t }`.  This module
//! recognises that cavity by what its sets say and builds it two ways: where the body is a prism
//! with nothing put on it and solids cut from it, as a prism (below); any other body by its walls.
//!
//! Within a prism the distance from its boundary is the lesser of the distance from its section's
//! edges, square to its axis, and from its caps along it: a point's foot on a side is level with
//! it, and on a cap stands over it.  So the points of a prism at least `t` from its kept faces are
//! a prism: its section's points at least `t` from its kept edges, between its kept caps moved in
//! by `t`.  That section is the section's edges each moved in by `t` — a line along its normal, an
//! arc about its centre — meeting where two meet at a convex corner, going on tangent where they
//! do, and joined by an arc of radius `t` about a reflex one; an open edge, or an open
//! cap, is moved out instead by half the thickness, so the cavity runs out through the opening and
//! crosses the body's faces there, never lying on them; where a kept edge meets an open one at
//! more than a right angle (a wall leaning in over the opening, or standing on it in a reflex
//! corner) the arc about the corner runs on to the open edge moved out.  A solid cut from the
//! prism takes from the cavity what lies within `t` of it (the cavity of `A` less `P` is `A`'s less
//! `P` grown by `t`), built as a body grown is (`grown::around`).  The cavity is the recipe's
//! `region`: the prism kept within, each cut grown kept outside; it lies on none of the body's
//! faces, so cutting it from the body meets them only across.  Held to the body's field before it stands (`gate`).
//!
//! **By its walls**, any other body — a boss put on it, a turned one, a body of its own for a
//! stock, or a prism the construction above refuses: the walls are
//! what lies within `t` of what is outside the body, so they are a body grown by `t` — the body's
//! faces turned inside out (`BodyFaces::inverted`), each convex edge of it concave and each concave
//! one convex — built in the same pieces (`grown::pieces`): each face's prism in from it, cut back
//! to the bisector of each convex edge; a wedge about each concave edge (a boss's foot), cut back
//! at its corners.  An open face first has its prism `2t` out from it glued on, so the walls run
//! out through it.  In the recipe the walls are glued to a block less the body and those prisms
//! (each piece's foot one of the body's faces, nothing intersected), and the cavity is a smaller
//! block, holding the body clear, less all that: no face of either touches another.  Refused there
//! as a body grown is, and where a wall stands on a face left open (its foot's quarter round
//! hangs below the opening, which no block outside the body says).
//!
//! Refused by name, and read by the field: a body rounded as well; a face left open that is none
//! of the stock's, or only part of one; a section whose walls meet or cross (a face narrower than
//! twice the thickness); a cut no body grown builds; walls no body grown builds.
use crate::brep::build::{Profile, ProfileEdge};
use crate::brep::geom::Frame;
use crate::brep::plane2::{self, Touch};
use crate::brep::planar::Seg;
use crate::brep::recipe::Writer;
use crate::json::{object, Json};
use crate::model::{FaceRef, RegionShape, RegionTerm, Sketch, SolidDef, Target, TermNumber};
use crate::space::{dot, norm, scale, sub};
use crate::vocab::Cmp;
use super::faces::BodyFaces;
use std::f64::consts::{FRAC_PI_2, PI, TAU};
use std::rc::Rc;

#[allow(unused_imports)]
use crate::fmath::Det;

type V = [f64; 3];
type P = [f64; 2];

/// A shell's cavity built (model units).
#[derive(Clone, Debug)]
pub struct Shelled {
    built: Built,
}

/// The two constructions of a cavity.
#[derive(Clone, Debug)]
enum Built {
    /// Within a prism less what is cut from it: a prism over the section eroded, from the plane
    /// it starts in as far as `length` along its normal, less each cut grown by the thickness.
    Prism { section: Profile, length: f64, cuts: Vec<super::grown::Grown> },
    /// Within any other body: its walls, what lies within the thickness of what is outside it.
    Walls(Walls),
}

/// The walls of a body, glued to the outside of it (`walls`): the body as its other terms make it
/// (`terms`), each open face's prism `reach` out from it glued on, all that within a block
/// (`boxes[1]`) taken from the block, and the pieces of what lies within the thickness of that
/// glued to it; the cavity is a smaller block (`boxes[0]`) less all of it.
#[derive(Clone, Debug)]
struct Walls {
    terms: Terms,
    open: Vec<Profile>,
    reach: f64,
    walls: super::grown::Grown,
    boxes: [(V, V); 2],
}

/// A body's terms but those reading its faces: what its stratum is made of.
#[derive(Clone, Debug)]
struct Terms {
    stock: u32,
    on: Vec<u32>,
    cut: Vec<u32>,
    bound: Vec<u32>,
}

impl Shelled {
    /// The recipe's node, written by `w`: the walls glued to the body as its recipe node's other
    /// terms make it.
    pub fn node(&self, w: &Writer) -> Result<Json, String> {
        let stratum = match &self.built {
            Built::Prism { .. } => Json::Null,
            Built::Walls(Walls { terms: t, .. }) => {
                let mut node = super::cad::body_terms(t.stock, &t.on, &t.cut, &t.bound);
                node.set("kind", "body".into());
                node
            }
        };
        self.cavity(w, stratum)
    }

    /// The cavity's node, written by `w`: the prism's `prism`, or a `region` keeping within it
    /// and outside each cut grown; the walls' `region`, the smaller block less the larger one less
    /// the body (`stratum`, a node standing for it) and its openings' prisms with the walls glued
    /// to that.
    fn cavity(&self, w: &Writer, stratum: Json) -> Result<Json, String> {
        let kept = |mut n: Json, keep: bool| { n.set("keep", Json::Bool(keep)); n };
        let region = |pieces: Vec<Json>| object([("kind", "region".into()), ("pieces", Json::Arr(pieces))]);
        match &self.built {
            Built::Prism { section, length, cuts } => {
                let prism = swept(section, *length, w)?;
                if cuts.is_empty() {
                    return Ok(prism);
                }
                let mut pieces = vec![kept(prism, true)];
                for g in cuts {
                    pieces.push(kept(g.node(w)?, false));
                }
                Ok(region(pieces))
            }
            Built::Walls(walls) => {
                let outside = region(vec![kept(block(walls.boxes[1], w)?, true),
                                          kept(plus(stratum, &walls.open, walls.reach, w)?, false)]);
                let mut grown = vec![outside];
                grown.extend(walls.walls.pieces_json(w)?);
                Ok(region(vec![kept(block(walls.boxes[0], w)?, true), kept(glued(grown), false)]))
            }
        }
    }

    /// The solids whose recipe nodes its own reads: the cuts its grown pieces are glued to, or the
    /// body's other terms.
    pub fn reads(&self) -> Vec<u32> {
        match &self.built {
            Built::Prism { cuts, .. } => cuts.iter().filter_map(|g| g.body()).collect(),
            Built::Walls(Walls { terms: t, .. }) => std::iter::once(t.stock).chain(t.on.iter().copied())
                .chain(t.cut.iter().copied()).chain(t.bound.iter().copied()).collect(),
        }
    }
}

/// Section `p` swept along its normal by `length`, as the recipe's `prism` (written by `w`).
fn swept(p: &Profile, length: f64, w: &Writer) -> Result<Json, String> {
    Ok(object([("kind", "prism".into()), ("profile", p.to_json(w)?), ("from", 0.0.into()),
               ("to", (length * w.scale).into())]))
}

/// Pieces glued to one another, as the recipe's `glued`.
fn glued(pieces: Vec<Json>) -> Json {
    object([("kind", "glued".into()), ("pieces", Json::Arr(pieces))])
}

/// The body (`stratum`, a node standing for it) with each opening's prism `reach` out from it glued
/// on: what the walls are within reach of the outside of.
fn plus(stratum: Json, open: &[Profile], reach: f64, w: &Writer) -> Result<Json, String> {
    let mut pieces = vec![stratum];
    for p in open {
        pieces.push(swept(p, reach, w)?);
    }
    Ok(glued(pieces))
}

/// The block between the corners `lo` and `hi` (model units), as the recipe's `prism`.
fn block((lo, hi): (V, V), w: &Writer) -> Result<Json, String> {
    let at = |x: f64, y: f64| [x, y, lo[2]];
    let c = [at(lo[0], lo[1]), at(hi[0], lo[1]), at(hi[0], hi[1]), at(lo[0], hi[1])];
    let p = Profile { origin: c[0], normal: [0.0, 0.0, 1.0], loops: vec![super::grown::polygon(&c)],
                      names: vec![vec!["block".to_string(); 4]] };
    swept(&p, hi[2] - lo[2], w)
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

/// The solids shell's cavity `solid` is made of — its stock and walls, or none but itself — where
/// it is one (by what its sets say, nothing built): what its construction stands for, where it is
/// built.
pub fn stands_for(sk: &Sketch, solid: usize) -> Option<Vec<u32>> {
    recognised(sk, solid).map(|r| r.under)
}

/// Solid `solid` built as a shell's cavity, where it is one (`None` otherwise): the construction,
/// or why it stands not.  Remembered against what it reads: its sets' numbers and the stratum of
/// the body it hollows (`Caches::shelled`).
pub fn built(sk: &Sketch, solid: usize) -> Option<Result<Rc<Shelled>, String>> {
    let r = recognised(sk, solid)?;
    let faces = super::faces::cached(sk, r.body as usize);
    let key = r.under.iter().fold(super::MemoKey::own(sk, solid, faces.iter().cloned().collect()),
        |k, &u| k.and(super::MemoKey::own(sk, u as usize, Vec::new())));
    sk.caches.shelled.get_or(solid, key, || {
        Some(faces.clone().and_then(|faces| {
            let s = construction(sk, solid, &r, &faces)?;
            gate(sk, solid, r.body as usize, &s, &faces)?;
            Ok(Rc::new(s))
        }))
    })
}

/// One piece of a section's loop in its plane, named for the face it is moved in from (`round`
/// about a corner): a stretch of a line, or an arc of radius `r` about `c` turning from angle
/// `from` to `to`.
#[derive(Clone, Debug)]
enum Piece {
    Line { a: P, b: P, name: String },
    Arc { c: P, r: f64, from: f64, to: f64, name: String },
}

/// An edge of the prism's section: its ends (the material on its left), what carries it, whether
/// the face it sweeps is left open, and that face's name.
#[derive(Clone, Debug)]
struct Edge {
    a: P,
    b: P,
    curve: Carried,
    open: bool,
    name: String,
}

/// What carries an edge of a section: a line, or a circle about `c` of radius `r` turning
/// counter-clockwise (`ccw`) or clockwise from the edge's start, through `sweep` (a whole turn
/// where the edge is the whole circle).
#[derive(Clone, Copy, Debug)]
enum Carried {
    Line,
    Arc { c: P, r: f64, ccw: bool, sweep: f64 },
}

impl Edge {
    /// The unit direction it runs in at `p`, a point of it.
    fn along(&self, p: P) -> P {
        match self.curve {
            Carried::Line => plane2::unit(plane2::sub(self.b, self.a)),
            Carried::Arc { c, ccw, .. } => {
                let d = plane2::left(plane2::unit(plane2::sub(p, c)));
                if ccw { d } else { plane2::scale(d, -1.0) }
            }
        }
    }

    /// The same edge run the other way.
    fn reversed(&self) -> Edge {
        let curve = match self.curve {
            Carried::Line => Carried::Line,
            Carried::Arc { c, r, ccw, sweep } => Carried::Arc { c, r, ccw: !ccw, sweep },
        };
        Edge { a: self.b, b: self.a, curve, ..self.clone() }
    }

    /// The area between its chord and itself, signed as the loop turns about it: a line's none.
    fn bulge(&self) -> f64 {
        match self.curve {
            Carried::Line => 0.0,
            Carried::Arc { r, ccw, sweep, .. } => (if ccw { 1.0 } else { -1.0 }) * r * r * (sweep - sweep.dsin()) / 2.0,
        }
    }
}

/// The point at `angle` on the circle of radius `r` about `c`.
fn polar(c: P, r: f64, angle: f64) -> P { [c[0] + r * angle.dcos(), c[1] + r * angle.dsin()] }

/// Which of the faces `body` stay shut: those `kept` names, or every one where it is `None`.
fn shut(body: &BodyFaces, kept: Option<&[FaceRef]>) -> Vec<bool> {
    let Some(refs) = kept else { return vec![true; body.paths().len()] };
    let mut shut = vec![false; body.paths().len()];
    for i in refs.iter().flat_map(|f| body.faces(f)) {
        shut[i] = true;
    }
    shut
}

/// The cavity of the body `r` names, whose faces as its other terms make them are `faces`.
fn construction(sk: &Sketch, solid: usize, r: &Recognised, faces: &BodyFaces) -> Result<Shelled, String> {
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
    let cut: Vec<u32> = through.iter().copied().filter(|o| !reads(o)).collect();
    let shut = shut(faces, r.kept.as_deref());
    // a prism less what is cut from it: the cavity a prism; any other body, or a prism that
    // construction refuses (a curved side), by its walls
    let prism = (on.is_empty() && bound.is_empty()
        && matches!(sk.solids[*stock as usize].def, SolidDef::Prism { draft: None, .. }))
        .then(|| -> Result<Shelled, String> {
            let prism = super::faces::cached(sk, *stock as usize)?;
            let (section, length) = cavity(&prism, faces, &shut, t)?;
            let cuts = cut.iter().map(|&c| super::grown::around(sk, c as usize, t, "round")
                .map_err(|why| format!("the cut `{}` grown by the thickness: {why}", sk.solid_name(c as usize))))
                .collect::<Result<Vec<_>, String>>()?;
            Ok(Shelled { built: Built::Prism { section, length, cuts } })
        });
    let refused = match prism {
        Some(Ok(s)) => return Ok(s),
        Some(Err(why)) => Some(why),
        None => None,
    };
    let terms = Terms { stock: *stock, on: on.clone(), cut: cut.clone(), bound: bound.clone() };
    walls(faces, &shut, t, terms).map(|w| Shelled { built: Built::Walls(w) })
        .map_err(|why| match refused { Some(prism) => format!("{prism}; {why}"), None => why })
}

/// The id the body's own boundary is read under where a shell's construction builds from it.
const STRATUM: i64 = -1;

/// A recipe node standing for the body's own boundary as `faces` read it (`STRATUM`).
fn stratum_node() -> Json {
    let identity = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0];
    object([("kind", "placed".into()), ("source", STRATUM.into()),
            ("matrix", Json::Arr(identity.into_iter().map(Json::from).collect()))])
}

/// The walls of the body whose faces are `body` (made by `terms`), leaving open those not `shut`,
/// at thickness `t`: each opening's prism out from it `2t` long, the walls — what lies within `t` of
/// what is outside the body and those prisms, built as a body grown is about the faces turned
/// inside out (`BodyFaces::inverted`, `grown::pieces`) — and the two blocks the cavity is read
/// between, the smaller holding the body and its openings' prisms clear.
fn walls(body: &BodyFaces, shut: &[bool], t: f64, terms: Terms) -> Result<Walls, String> {
    let refused = |why: &str| format!("{why}: read by the field");
    let (b, mm, origin) = body.brep();
    // a wall standing on an opening runs on round its foot below it, which no block outside the
    // body and its openings' prisms says
    for e in 0..b.edges.len() as u32 {
        if let &[fa, fb] = body.faces_of(e) {
            if shut[fa] != shut[fb] && body.convex(e) == Some(false) {
                return Err(refused("a wall standing on a face left open"));
            }
        }
    }
    let mut open = Vec::new();
    for fi in (0..shut.len()).filter(|&i| !shut[i]) {
        let (n, o) = body.plane(fi).ok_or_else(|| refused("a face left open that is no plane"))?;
        let mut section = super::grown::section_of(body, fi, n, o)
            .ok_or_else(|| refused("a face left open bounded by an edge no line or circle carries"))?;
        section.names = section.loops.iter().map(|l| vec!["open".to_string(); l.len()]).collect();
        open.push(section);
    }
    // the body and its openings' prisms, as the recipe builds them, about the body's own origin
    let reach = 2.0 * t;
    let w = Writer::about(mm, origin.map(|x| x * mm));
    let built = std::collections::BTreeMap::from([(STRATUM, b.clone())]);
    let plus = BodyFaces::unnamed(crate::brep::recipe::node(&plus(stratum_node(), &open, reach, &w)?, &built)?, mm, origin);
    let walls = super::grown::pieces(&plus.inverted(), t, "wall").map_err(|why| format!("its walls: {why}"))?;
    let (lo, hi) = plus.bounds();
    let pad = |k: f64| (lo.map(|x| x - k * t), hi.map(|x| x + k * t));
    Ok(Walls { terms, open, reach, walls, boxes: [pad(2.0), pad(4.0)] })
}

/// The cavity's prism within the prism whose faces are `prism`, in the body whose faces are
/// `body` (the prism less what is cut from it), leaving open the body's faces not `shut`, at
/// thickness `t`: its section, in the plane it starts from, and how far it runs.
fn cavity(prism: &BodyFaces, body: &BodyFaces, shut: &[bool], t: f64) -> Result<(Profile, f64), String> {
    let refused = |why: &str| format!("{why}: read by the field");
    let (b, _, _) = prism.brep();
    let named = |name: &str| (0..prism.paths().len()).find(|&i| prism.paths()[i].face == name && prism.paths()[i].route.is_empty());
    let (Some(far), Some(near)) = (named("far"), named("near")) else {
        return Err(refused("a prism whose caps are not named so"));
    };
    let planes: Vec<Option<(V, V)>> = (0..b.faces.len()).map(|i| prism.plane(i)).collect();
    let (Some(far_plane), Some(near_plane)) = (planes[far], planes[near]) else {
        return Err(refused("a prism whose caps are not planes"));
    };
    let (lo, hi) = prism.bounds();
    let tol = 1e-9 * norm(sub(hi, lo)).max(1.0);
    // which of the prism's faces are left open: those the body's open faces lie on
    let mut open = vec![false; b.faces.len()];
    let on = |(n, o): (V, V), (m, q): (V, V)| dot(n, m) > 1.0 - 1e-9 && dot(sub(o, q), m).abs() <= tol;
    for fi in (0..shut.len()).filter(|&i| !shut[i]) {
        let p = body.plane(fi).ok_or_else(|| refused("a face left open that is no plane"))?;
        let s = (0..planes.len()).find(|&j| planes[j].is_some_and(|q| on(p, q)))
            .ok_or_else(|| refused("a face left open that is none of the stock's (a cut's face is never left open)"))?;
        open[s] = true;
    }
    for fi in (0..shut.len()).filter(|&i| shut[i]) {
        if body.plane(fi).is_some_and(|p| (0..planes.len()).any(|j| open[j] && planes[j].is_some_and(|q| on(p, q)))) {
            return Err(refused("a face of the stock left open in part only"));
        }
    }
    // the section, on the far cap, in a frame about the prism's axis
    let axis = scale(far_plane.0, -1.0);
    let frame = Frame::about(far_plane.1, axis);
    let flat = |p: V| { let l = frame.local(p); [l[0], l[1]] };
    let mut loops = Vec::new();
    for l in &b.faces[far].loops {
        let mut edges = Vec::new();
        for c in l {
            let e = c.edge;
            let side = prism.faces_of(e).iter().copied().find(|&g| g != far)
                .ok_or_else(|| refused("a prism's edge bounding one face"))?;
            // each edge as its parameter runs, a circle's counter-clockwise about its own normal
            let (a, z) = (flat(prism.along(e, 0.0)), flat(prism.along(e, 1.0)));
            let curve = if prism.segment(e).is_some() {
                Carried::Line
            } else if let Some((centre, normal, r, _)) = prism.circle(e) {
                let [t0, t1] = prism.span(e);
                Carried::Arc { c: flat(centre), r, ccw: dot(normal, axis) > 0.0, sweep: t1 - t0 }
            } else {
                return Err(refused("a prism whose section has an edge no line or circle carries"));
            };
            let edge = Edge { a, b: z, curve, open: open[side], name: prism.paths()[side].face.clone() };
            edges.push(if c.reversed { edge.reversed() } else { edge });
        }
        loops.push(edges);
    }
    // outer first and turning left, holes turning right, so the material is on each edge's left
    let area = |l: &[Edge]| l.iter().map(|e| plane2::cross(e.a, e.b) / 2.0 + e.bulge()).sum::<f64>();
    loops.sort_by(|x, y| area(y).abs().total_cmp(&area(x).abs()));
    for (k, l) in loops.iter_mut().enumerate() {
        if (area(l) > 0.0) != (k == 0) {
            *l = l.iter().rev().map(Edge::reversed).collect();
        }
    }
    let out = t / 2.0;
    let eroded = loops.iter().map(|l| eroded(l, t, out, tol)).collect::<Result<Vec<_>, _>>().map_err(refused)?;
    simple(&eroded, tol).map_err(refused)?;
    // between the caps, each moved in by the thickness or out through an opening
    let height = dot(sub(near_plane.1, far_plane.1), axis);
    let from = if open[far] { -out } else { t };
    let to = if open[near] { height + out } else { height - t };
    if !(to - from > tol) {
        return Err(refused("a prism no longer than twice the thickness between its caps"));
    }
    let lift = |p: P| frame.at([p[0], p[1], from]);
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
                Piece::Arc { c, r, from, to, name } => {
                    let f = Frame::new(lift(*c), axis, frame.x);
                    let whole = (to - from).abs() >= TAU - 1e-12;
                    edges.push(ProfileEdge::Arc { frame: f, r: *r, span: (!whole).then(|| [from.min(*to), from.max(*to)]) });
                    called.push(name.clone());
                }
            }
        }
        section.push(edges);
        names.push(called);
    }
    Ok((Profile { origin: lift([0.0, 0.0]), normal: axis, loops: section, names }, to - from))
}

/// A loop of the section moved in by `t` from each edge kept and out by `out` from each open one:
/// each edge's stretch of its carrier moved (a line along its normal, a circle's radius less or
/// more), and between two the corner where the carriers meet — where they go on tangent, the point
/// both moved to — or an arc of radius `t` about the corner where the points within `t` of it reach
/// past both: a reflex corner of two kept edges, or one where a kept edge meets an open line at
/// more than a right angle (the arc then run on to the open line).  Refused where a stretch turns
/// round: the walls of a face meet.
fn eroded(l: &[Edge], t: f64, out: f64, tol: f64) -> Result<Vec<Piece>, &'static str> {
    const MEET: &str = "a face narrower than twice the thickness, its walls meeting";
    let m = l.len();
    let by = |k: usize| if l[k].open { -out } else { t };
    // each edge's carrier moved: a line through a point along its direction, or a circle
    let moved = |k: usize| -> Result<plane2::Carrier, &'static str> {
        match l[k].curve {
            Carried::Line => {
                let d = l[k].along(l[k].a);
                Ok(plane2::Carrier::Line { p: plane2::add(l[k].a, plane2::scale(plane2::left(d), by(k))), d })
            }
            Carried::Arc { c, r, ccw, .. } => {
                let r = if ccw { r - by(k) } else { r + by(k) };
                if r > tol { Ok(plane2::Carrier::Circle { c, r }) } else { Err(MEET) }
            }
        }
    };
    let angle = |v: P| v[1].datan2(v[0]);
    // at each corner, between edge `k` and the next: where the first stretch ends, and the next
    // begins, and the arc between them
    let mut corners: Vec<(P, P, Option<Piece>)> = Vec::new();
    for k in 0..m {
        let j = (k + 1) % m;
        let v = l[k].b;
        let (dk, dj) = (l[k].along(v), l[j].along(v));
        let (nk, nj) = (plane2::left(dk), plane2::left(dj));
        let (cr, dt) = (plane2::cross(dk, dj), plane2::dot(dk, dj));
        if cr.abs() <= 1e-12 && dt > 0.0 {
            // the edges go on tangent: both moved to one point, where both are moved alike
            if by(k) != by(j) {
                return Err("a section where an open edge goes on tangent into a kept one");
            }
            let p = plane2::add(v, plane2::scale(nk, by(k)));
            corners.push((p, p, None));
            continue;
        }
        let turn = cr.datan2(dt);
        let interior = PI - turn;
        // the arc turns clockwise about the corner: the cavity lies outside its circle
        let reach = (-out / t).dacos() - turn;
        let line = |k: usize| matches!(l[k].curve, Carried::Line);
        let arc = match (l[k].open, l[j].open) {
            (false, false) if turn < 0.0 => Some((angle(nk), angle(nk) + turn)),
            (false, true) if interior > FRAC_PI_2 + 1e-9 && line(j) => Some((angle(nk), angle(nk) - reach)),
            (true, false) if interior > FRAC_PI_2 + 1e-9 && line(k) => Some((angle(nj) + reach, angle(nj))),
            (false, true) | (true, false) if interior > FRAC_PI_2 + 1e-9 => {
                return Err("a wall leaning in over an opening along a curve");
            }
            _ => None,
        };
        corners.push(match arc {
            Some((from, to)) => (polar(v, t, from), polar(v, t, to),
                                 Some(Piece::Arc { c: v, r: t, from, to, name: "round".to_string() })),
            None => {
                // the meeting of the two carriers nearest the corner moved
                let near = plane2::add(v, plane2::scale(plane2::add(nk, nj), 0.5 * (by(k) + by(j))));
                let p = plane2::meet(moved(k)?, moved(j)?, Touch::Exact).unwrap_or_default().into_iter()
                    .min_by(|a, b| plane2::dist(*a, near).total_cmp(&plane2::dist(*b, near))).ok_or(MEET)?;
                (p, p, None)
            }
        });
    }
    let mut pieces = Vec::new();
    for k in 0..m {
        let (start, end) = (corners[(k + m - 1) % m].1, corners[k].0);
        let name = if l[k].open { "open".to_string() } else { l[k].name.clone() };
        match (l[k].curve, moved(k)?) {
            (Carried::Line, _) => {
                if !(plane2::dot(plane2::sub(end, start), l[k].along(l[k].a)) > tol) {
                    return Err(MEET);
                }
                pieces.push(Piece::Line { a: start, b: end, name });
            }
            (Carried::Arc { ccw, sweep, .. }, plane2::Carrier::Circle { c, r }) => {
                // turned the way the edge turns, about as far: a stretch turned round has met; a
                // whole circle's ends are one point
                let (from, at) = (angle(plane2::sub(start, c)), angle(plane2::sub(end, c)));
                let mut turned = (if ccw { at - from } else { from - at }).rem_euclid(TAU);
                if turned <= tol / r && sweep > PI {
                    turned = TAU;
                }
                if !(turned > tol / r) || (turned - sweep).abs() >= FRAC_PI_2 {
                    return Err(MEET);
                }
                pieces.push(Piece::Arc { c, r, from, to: if ccw { from + turned } else { from - turned }, name });
            }
            _ => unreachable!("a circle moved is a circle"),
        }
        if let Some(arc) = corners[k].2.clone() {
            pieces.push(arc);
        }
    }
    Ok(pieces)
}

/// Whether the loops cross nowhere: no two stretches of them meet (`planar::meets`, exactly) but
/// where one goes on from the next.
fn simple(loops: &[Vec<Piece>], tol: f64) -> Result<(), &'static str> {
    let segs: Vec<Vec<Seg>> = loops.iter().map(|l| l.iter().map(|p| match *p {
        Piece::Line { a, b, .. } => Seg::Line { a, b },
        Piece::Arc { c, r, from, to, .. } => Seg::Arc { c, r, a0: from, sweep: to - from },
    }).collect()).collect();
    for (i, li) in segs.iter().enumerate() {
        for (j, lj) in segs.iter().enumerate().skip(i) {
            for (x, p) in li.iter().enumerate() {
                for (y, q) in lj.iter().enumerate() {
                    let next = i == j && (y == x + 1 || (x == 0 && y + 1 == lj.len()));
                    if (i == j && y <= x) || next {
                        continue;
                    }
                    if !crate::brep::planar::meets(p, q, tol).0.is_empty() {
                        return Err("a section whose walls cross (a face narrower than twice the thickness)");
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
    let mut built = std::collections::BTreeMap::from([(STRATUM, stratum.clone())]);
    let cuts: Vec<u32> = match &s.built {
        Built::Prism { cuts, .. } => cuts.iter().filter_map(|g| g.body()).collect(),
        Built::Walls(_) => Vec::new(),
    };
    for c in cuts {
        // each cut's boundary moved from its own origin to the body's
        let cut = super::faces::cached(sk, c as usize)?;
        let (b, _, at) = cut.brep();
        let shift = crate::brep::geom::Rigid { t: scale(sub(at, origin), mm), ..crate::brep::geom::Rigid::identity() };
        built.insert(c as i64, b.moved(&shift));
    }
    let cavity = crate::brep::recipe::node(&s.cavity(&w, stratum_node())?, &built)?;
    let shelled = crate::brep::boolean::combined(stratum, &cavity, crate::brep::boolean::Op::Cut, 0.0)
        .map_err(|e| format!("`{}`: {}", sk.solid_name(solid), String::from(e)))?;
    let field = super::MaterialField::read(sk, body, super::cad::AXIS_TOLERANCE)?;
    let back = |p: V| faces.back(p);
    super::agreement::gate(&field, &shelled, Some((&back, mm)), "its exact construction disagrees with the field")
}
