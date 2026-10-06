//! **A fillet** (Solvent §6.9, issue #66, rung 1): the rolling ball's material along every edge
//! where a face of one operand meets a face of the other, worked out after the solve from the two
//! operands' exact boundary — which gives the edges, the faces either side and what each face is
//! bounded by.
//!
//! A ball of radius `r` rolled along an edge between two faces touches each at a fixed setback
//! from the edge. Seen in the plane square to the edge (a straight edge between two planes), or in
//! the meridian (a circle between surfaces turned about one axis: a plane square to it, a cylinder
//! or a cone, each a line there), that is a line–line fillet: the [`Wedge`] between the corner,
//! the two points where the ball touches and the arc between them. Its material is a prism of the
//! wedge along a straight edge, or the wedge turned once about the axis (a torus's face) — so every
//! consumer reads an ordinary primitive: the facet term, the CAD recipe, both kernels and the
//! material field, which needs no true distance and no morphological closing.
//!
//! At a concave edge the wedge is empty space the ball fills, and a body takes the fillet with
//! `union`; at a convex edge it is material the ball rolls off, and the body `cut`s it. Anything
//! this cannot round exactly is refused with the reason: faces of other kinds (rung 2), an edge
//! that runs on into another face (rung 3), a ball larger than a face can hold.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::*;
use crate::brep::geom::{around, Curve, Rigid, Surface, V};
use crate::model::FilletSide;
use crate::brep::query::{Located, Place};
use crate::brep::topo::{Brep, EdgeCurve};
use crate::space::{add, cross, dot, norm, scale, sub};

/// A face's trace in a section through the corner: a straight line, or a circle — a cylinder
/// along a straight edge, a sphere or a torus in the meridian of a ring.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Side { Line, Circle { centre: [f64; 2], radius: f64 } }

/// One stroke of a section's loop: a segment, or an arc of a circle from `from` to `to`, `sweep`
/// signed (counter-clockwise positive) from the angle `start`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Stroke {
    Line { from: [f64; 2], to: [f64; 2] },
    Arc { centre: [f64; 2], radius: f64, start: f64, sweep: f64, from: [f64; 2], to: [f64; 2] },
}

impl Stroke {
    /// The arc about `centre` through `radius` from `from` to `to`, turning the way `sense` says
    /// (counter-clockwise positive), never more than a whole turn.
    fn arc(centre: [f64; 2], radius: f64, from: [f64; 2], to: [f64; 2], sense: f64) -> Stroke {
        let (a, b) = (angle(centre, from), angle(centre, to));
        let sweep = if sense > 0.0 { (b - a).rem_euclid(TAU) } else { -(a - b).rem_euclid(TAU) };
        Stroke::Arc { centre, radius, start: a, sweep, from, to }
    }

    /// The least first coordinate it reaches (a meridian's radius).
    fn least_x(&self) -> f64 {
        match *self {
            Stroke::Line { from, to } => from[0].min(to[0]),
            Stroke::Arc { centre, radius, start, sweep, from, to } => {
                let reaches = around(PI, start.min(start + sweep), TAU) <= start.max(start + sweep);
                let ends = from[0].min(to[0]);
                if reaches { ends.min(centre[0] - radius) } else { ends }
            }
        }
    }
}

const TAU: f64 = std::f64::consts::TAU;
const PI: f64 = std::f64::consts::PI;

/// The angle of `p` about `c`.
fn angle(c: [f64; 2], p: [f64; 2]) -> f64 { (p[1] - c[1]).datan2(p[0] - c[0]) }
fn dot2(a: [f64; 2], b: [f64; 2]) -> f64 { a[0] * b[0] + a[1] * b[1] }
fn sub2(a: [f64; 2], b: [f64; 2]) -> [f64; 2] { [a[0] - b[0], a[1] - b[1]] }
fn add2(a: [f64; 2], b: [f64; 2], k: f64) -> [f64; 2] { [a[0] + k * b[0], a[1] + k * b[1]] }
fn len2(a: [f64; 2]) -> f64 { a[0].dhypot(a[1]) }

/// The ball's section at one edge, in its section plane's coordinates: the corner the two faces
/// meet at, each face's trace through it and the direction it leaves the corner along that face,
/// the points where the ball touches the first face and the second, and its centre.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Wedge {
    pub corner: [f64; 2],
    pub sides: [Side; 2],
    pub dirs: [[f64; 2]; 2],
    pub touch: [[f64; 2]; 2],
    pub centre: [f64; 2],
    pub r: f64,
}

/// Why no ball fits a corner.
enum Misfit { Angle, Ball }

impl Wedge {
    /// The ball of radius `r` in the corner at `corner` between the sides, each leaving it along
    /// the unit direction `d` (into its face): its centre on both sides' offsets by `r` into the
    /// corner, touching each ahead of the corner, the nearest such.
    fn new(corner: [f64; 2], sides: [Side; 2], d: [[f64; 2]; 2], r: f64) -> Result<Wedge, Misfit> {
        let theta = dot2(d[0], d[1]).clamp(-1.0, 1.0).dacos();
        if !(theta > MIN_TURN && theta < PI - MIN_TURN) { return Err(Misfit::Angle); }
        // each side's offset into the corner: a line (a point and a direction) or a circle
        enum Offset { Line([f64; 2], [f64; 2]), Circle([f64; 2], f64) }
        let offset = |k: usize| -> Option<Offset> {
            let other = d[1 - k];
            match sides[k] {
                Side::Line => {
                    let n = [-d[k][1], d[k][0]];
                    let n = if dot2(n, other) > 0.0 { n } else { [-n[0], -n[1]] };
                    Some(Offset::Line(add2(corner, n, r), d[k]))
                }
                Side::Circle { centre, radius } => {
                    let radial = sub2(corner, centre);
                    let outside = dot2(other, radial) > 0.0;
                    let big = if outside { radius + r } else { radius - r };
                    (big > 0.0).then_some(Offset::Circle(centre, big))
                }
            }
        };
        let (Some(a), Some(b)) = (offset(0), offset(1)) else { return Err(Misfit::Ball) };
        let line_circle = |p: [f64; 2], u: [f64; 2], c: [f64; 2], rr: f64| -> Vec<[f64; 2]> {
            let foot = add2(p, u, dot2(sub2(c, p), u));
            let h2 = rr * rr - dot2(sub2(foot, c), sub2(foot, c));
            if h2 < 0.0 { return Vec::new(); }
            let h = h2.sqrt();
            vec![add2(foot, u, -h), add2(foot, u, h)]
        };
        let candidates: Vec<[f64; 2]> = match (&a, &b) {
            (Offset::Line(p, u), Offset::Line(q, w)) => {
                let det = u[0] * (-w[1]) - u[1] * (-w[0]);
                if det.abs() <= 1e-15 { Vec::new() } else {
                    let rhs = sub2(*q, *p);
                    let s = (rhs[0] * (-w[1]) - rhs[1] * (-w[0])) / det;
                    vec![add2(*p, *u, s)]
                }
            }
            (Offset::Line(p, u), Offset::Circle(c, rr)) | (Offset::Circle(c, rr), Offset::Line(p, u)) =>
                line_circle(*p, *u, *c, *rr),
            (Offset::Circle(c0, r0), Offset::Circle(c1, r1)) => {
                let between = sub2(*c1, *c0);
                let l = len2(between);
                if l == 0.0 { Vec::new() } else {
                    let u = [between[0] / l, between[1] / l];
                    let x = (l * l + r0 * r0 - r1 * r1) / (2.0 * l);
                    line_circle(add2(*c0, u, x), [-u[1], u[0]], *c0, *r0)
                }
            }
        };
        // where a ball centred at `c` touches side `k`, if ahead of the corner along it
        let touch = |k: usize, c: [f64; 2]| -> Option<[f64; 2]> {
            match sides[k] {
                Side::Line => {
                    let t = add2(corner, d[k], dot2(sub2(c, corner), d[k]));
                    (dot2(sub2(t, corner), d[k]) > 0.0).then_some(t)
                }
                Side::Circle { centre, radius } => {
                    let toward = sub2(c, centre);
                    let t = add2(centre, toward, radius / len2(toward));
                    let radial = sub2(corner, centre);
                    let sense = dot2(d[k], [-radial[1], radial[0]]).signum();
                    let Stroke::Arc { sweep, .. } = Stroke::arc(centre, radius, corner, t, sense) else { unreachable!() };
                    (sweep.abs() > 0.0 && sweep.abs() < PI).then_some(t)
                }
            }
        };
        let best = candidates.into_iter()
            .filter_map(|c| Some((c, [touch(0, c)?, touch(1, c)?])))
            .min_by(|x, y| len2(sub2(x.0, corner)).total_cmp(&len2(sub2(y.0, corner))));
        let (centre, touch) = best.ok_or(Misfit::Ball)?;
        Ok(Wedge { corner, sides, dirs: d, touch, centre, r })
    }

    /// The section's loop: along the first side from the corner to its touch, round the ball to
    /// the second touch the short way (past the corner), back along the second side.
    pub fn strokes(&self) -> [Stroke; 3] {
        let along = |k: usize, from: [f64; 2], to: [f64; 2]| match self.sides[k] {
            Side::Line => Stroke::Line { from, to },
            Side::Circle { centre, radius } => {
                let radial = sub2(self.corner, centre);
                let sense = dot2(self.dirs[k], [-radial[1], radial[0]]).signum();
                // the second side is walked back, from its touch to the corner
                Stroke::arc(centre, radius, from, to, if k == 0 { sense } else { -sense })
            }
        };
        let (a, b) = (angle(self.centre, self.touch[0]), angle(self.centre, self.touch[1]));
        let ccw = (b - a).rem_euclid(TAU);
        let ball = Stroke::arc(self.centre, self.r, self.touch[0], self.touch[1], if ccw <= PI { 1.0 } else { -1.0 });
        [along(0, self.corner, self.touch[0]), ball, along(1, self.touch[1], self.corner)]
    }
}

/// How the section is carried: along the edge's direction (`u × v`) through `length`, or turned
/// through `sweep` (`τ` a whole ring) about the line through the section's origin along its `v`,
/// right-handed — from `u` toward `v × u`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Carry { Prism { length: f64 }, Turn { sweep: f64 } }

/// One edge's fillet, in model units: the section plane `o + a·u + b·v`, the ball in it, and how
/// the section is carried.
#[derive(Clone, Debug, PartialEq)]
pub struct Piece {
    pub section: Basis,
    pub wedge: Wedge,
    pub carry: Carry,
}

impl Piece {
    /// A point of the edge it rounds — what an oracle rounding the same edge selects it by: the
    /// corner, halfway along a straight edge.
    pub fn edge_point(&self) -> V {
        self.at(0.5).lift(self.wedge.corner[0], self.wedge.corner[1])
    }

    /// Its section a fraction `f` of the way along: the prism's carried along its length, the
    /// ring's turned through its sweep (0 where it starts, 1 where it stops).
    pub fn at(&self, f: f64) -> Basis {
        match self.carry {
            Carry::Prism { length } => carried(&self.section, f * length),
            Carry::Turn { sweep } => turned_basis(&self.section, f * sweep),
        }
    }

    /// Whether it is a whole ring, with no ends.
    pub fn whole(&self) -> bool { matches!(self.carry, Carry::Turn { sweep } if whole(sweep)) }

    /// The same piece with every length divided by `per` and its plane stood off by `origin`: a
    /// piece worked out in a boundary's millimetres, in model units about the world's origin.
    fn in_units(&self, per: f64, origin: V) -> Piece {
        let w = &self.wedge;
        let sc = |p: [f64; 2]| [p[0] / per, p[1] / per];
        Piece {
            section: Basis { o: std::array::from_fn(|k| self.section.o[k] / per + origin[k]), ..self.section },
            wedge: Wedge {
                corner: sc(w.corner),
                sides: w.sides.map(|side| match side {
                    Side::Line => Side::Line,
                    Side::Circle { centre, radius } => Side::Circle { centre: sc(centre), radius: radius / per },
                }),
                dirs: w.dirs,
                touch: w.touch.map(sc),
                centre: sc(w.centre),
                r: w.r / per,
            },
            carry: match self.carry { Carry::Prism { length } => Carry::Prism { length: length / per }, turn => turn },
        }
    }
}

/// The names of a piece's section edges, in its loop's order — corner to first touch, the arc,
/// second touch to corner — and so of the faces its sweep makes: the arc's is the fillet's face.
pub const EDGE_NAMES: [&str; 3] = ["a", "round", "b"];

/// A ball rolled along a closed traced loop (`brep::fillet::roll`): the kernel's piece, spine and
/// faces, in the boundary's millimetres about `origin` (model units, `mm` a model unit), and the
/// point of the loop it was rolled from there.
#[derive(Clone, Debug)]
pub struct Roll { pub rolled: std::rc::Rc<crate::brep::fillet::Rolled>, pub mm: f64, pub origin: V, pub at: V }

impl Roll {
    /// The point of the loop it rounds, in model units — what a kernel rolling it again selects
    /// the loop by.
    pub fn edge_point(&self) -> V { self.model(self.at) }
    /// A point of the kernel's boundary in model units.
    pub fn model(&self, p: V) -> V { std::array::from_fn(|k| p[k] / self.mm + self.origin[k]) }
}

/// A fillet's every piece — swept or turned sections, and balls rolled along traced loops — and
/// whether its edges are concave (the ball's material is added). `joins`: the pieces of a chain
/// that end in one another's sections (indices into `pieces`), which share that cap and meet
/// nowhere else.
#[derive(Clone, Debug)]
pub struct Blend { pub pieces: Vec<Piece>, pub joins: Vec<[usize; 2]>, pub rolls: Vec<Roll>, pub concave: bool }

/// Two faces running on into one another, to this (one less the cosine of the angle they turn
/// through, the angle's square over two): no edge, nothing to round.
const SMOOTH: f64 = 1e-12;

/// Faces nearer parallel than this (radians, either way) meet at no edge a ball can round.
const MIN_TURN: f64 = 1.0 * std::f64::consts::PI / 180.0;

/// One use of an edge by a face: (face, (loop, index in the loop)).
type Use = (usize, (usize, usize));

/// Solid `si`'s blend, from the exact boundaries of what it rounds between; `Err` says why it
/// cannot be rounded exactly.
pub(crate) fn derive(sk: &Sketch, si: usize) -> Result<Blend, String> {
    let s = sk.solids.get(si).ok_or("no such solid")?;
    let SolidDef::Fillet { a, b, r } = &s.def else { return Err(format!("`{}` is not a fillet", s.name)) };
    if !(r.value > 0.0) || !r.value.is_finite() {
        return Err("a fillet's radius is a positive length".into());
    }
    let exacts = operands(sk, si).into_iter().map(|i| sk.exact_solid(i as usize).map(|x| (i, x)).map_err(|e| {
        format!("`{}` has no exact boundary to round: {e}", sk.solids[i as usize].name)
    })).collect::<Result<Vec<_>, _>>()?;
    // each side's faces by name, asked before anything is combined
    let side = |x: &FilletSide| -> Result<std::collections::BTreeSet<String>, String> {
        let of = stands_for(sk, si, x.solid);
        let named: std::collections::BTreeSet<String> = exacts.iter().filter(|(i, _)| of.contains(i))
            .flat_map(|(_, e)| faces_of(sk, x, &e.brep)).collect();
        if named.is_empty() {
            return Err(format!("`{}` has no face `{}`", sk.solids[x.solid as usize].name, x.face.join(".")));
        }
        Ok(named)
    };
    let (fa, fb) = (side(a)?, side(b)?);
    // every solid either side stands for, in one boundary about the first's origin
    let (mm, origin) = (exacts[0].1.mm, exacts[0].1.origin);
    let mut brep = exacts[0].1.brep.clone();
    for (x, e) in &exacts[1..] {
        let shift: V = std::array::from_fn(|k| (e.origin[k] - origin[k]) * mm);
        let moved = e.brep.moved(&Rigid { r: Rigid::identity().r, t: shift });
        brep = crate::brep::recipe::combined(&brep, &moved, crate::brep::boolean::Op::Union, 0.0)
            .map_err(|m| format!("`{}` does not combine with what it meets: {m}", sk.solids[*x as usize].name))?;
    }
    let tol = 1e-9 * brep.size().max(1.0);
    let located = Located::new(&brep, tol);
    let table = uses_by_edge(&brep);
    let mut pieces = Vec::new();
    let mut rolls = Vec::new();
    let mut rolled_edges = std::collections::BTreeSet::new();
    let mut ends: Vec<End> = Vec::new();
    // each edge refused, and why: told once the ends are paired
    let mut failed: Vec<(usize, String)> = Vec::new();
    let mut concave: Option<(bool, String)> = None;
    for (ei, uses) in table.iter().enumerate() {
        let [(f0, c0), (f1, c1)] = match uses.as_slice() { [x, y] if x.0 != y.0 => [*x, *y], _ => continue };
        let (n0, n1) = (&brep.faces[f0].name, &brep.faces[f1].name);
        let (first, second) = if fa.contains(n0) && fb.contains(n1) { ((f0, c0), (f1, c1)) }
            else if fa.contains(n1) && fb.contains(n0) { ((f1, c1), (f0, c0)) }
            else { continue };
        let label = format!("`{}` with `{}`", brep.faces[first.0].name, brep.faces[second.0].name);
        // a meeting no line or circle carries: the ball rolled along its whole loop at once
        let traced = match &brep.edges[ei].curve {
            EdgeCurve::Curve(Curve::Line { .. } | Curve::Circle(..)) => false,
            EdgeCurve::Curve(_) => true,
            EdgeCurve::Degenerate => continue,
        };
        let hollow = if traced {
            if rolled_edges.contains(&ei) { continue; }
            let e = &brep.edges[ei];
            let at = e.point(0.5 * (e.t[0] + e.t[1]), &brep.vertices);
            let rolled = crate::brep::fillet::roll(&brep, at, r.value * mm, tol)?;
            rolled_edges.extend(rolled.chain.iter().copied());
            let hollow = rolled.concave;
            rolls.push(Roll { rolled: std::rc::Rc::new(rolled), mm, origin, at });
            hollow
        } else {
            let at = Edge { brep: &brep, located: &located, table: &table, edge: ei, uses: [first, second], tol };
            let rounded = match at.round(r.value * mm) {
                Ok(x) => x,
                Err(why) => { failed.push((ei, why)); continue }
            };
            let Some((piece, hollow, piece_ends)) = rounded else { continue };
            ends.extend(piece_ends.into_iter().map(|e| End { piece: pieces.len(), ..e }));
            pieces.push(piece.in_units(mm, origin));
            hollow
        };
        match &concave {
            Some((was, other)) if *was != hollow => return Err(format!(
                "rounds the concave edge of {} and the convex edge of {}: a `union` adds the one and a \
                 `cut` takes the other away, so write two fillets",
                if hollow { &label } else { other }, if hollow { other } else { &label })),
            Some(_) => {}
            None => concave = Some((hollow, label)),
        }
    }
    // an end that meets nothing turns a corner — unless an edge refused is what it was to run on
    // into (leaving their vertex its way), whose reason says more
    let joins = paired(&ends, tol).map_err(|(end, refusal)| {
        let into = failed.iter().find(|(ei, _)| {
            let e = &brep.edges[*ei];
            let EdgeCurve::Curve(c) = &e.curve else { return false };
            let leaving = [unit(c.tangent(e.t[0])), scale(unit(c.tangent(e.t[1])), -1.0)];
            (0..2).any(|k| e.v[k] == end.vertex && dot(leaving[k], refusal.outward) >= 1.0 - 1e-9)
        });
        into.map_or(&refusal.refusal, |(_, why)| why).clone()
    })?;
    if let Some((_, why)) = failed.into_iter().next() { return Err(why); }
    // a piece ending flush at a corner another end stands at, in a face the other rounds: a vertex blend
    for (i, f) in ends.iter().enumerate().filter(|(_, e)| e.meet.is_none()) {
        if ends.iter().enumerate().any(|(j, g)| j != i && g.vertex == f.vertex) {
            return Err(format!("the fillet of {} meets another of its edges at a corner: fillets meeting at a \
                vertex are rung 3", f.label));
        }
    }
    let Some((concave, _)) = concave else {
        return Err(format!("`{}` and `{}` meet at no edge to round",
            side_name(sk, a), side_name(sk, b)));
    };
    Ok(Blend { pieces, joins, rolls, concave })
}

/// Every solid fillet `f` reads, once each: what either side stands for (`stands_for`).
pub fn operands(sk: &Sketch, f: usize) -> Vec<u32> {
    let SolidDef::Fillet { a, b, .. } = &sk.solids[f].def else { return Vec::new() };
    let mut out = stands_for(sk, f, a.solid);
    for x in stands_for(sk, f, b.solid) { if !out.contains(&x) { out.push(x); } }
    out
}

/// The solids side `x` of fillet `f` is read from: the solid itself — or, where it is a body
/// that takes `f`, its stock and what it adds other than fillets. The edges a fillet rounds are
/// those of the union (§6.9): before what the body cuts, and before any fillet, so
/// `lip := fillet(block.near, block.bc, r: 2mm)` with `lip cut block` reads the block as it was
/// swept.
pub fn stands_for(sk: &Sketch, f: usize, x: u32) -> Vec<u32> {
    match &sk.solids[x as usize].def {
        SolidDef::Body { stock, on, through, bound }
            if std::iter::once(stock).chain(on).chain(through).chain(bound).any(|&o| o as usize == f) =>
        {
            std::iter::once(*stock)
                .chain(on.iter().copied().filter(|&o| !matches!(sk.solids[o as usize].def, SolidDef::Fillet { .. })))
                .collect()
        }
        _ => vec![x],
    }
}

/// How a fillet's side is written: `block.near`, or the solid's name alone.
fn side_name(sk: &Sketch, x: &FilletSide) -> String {
    let n = &sk.solids[x.solid as usize].name;
    if x.face.is_empty() { n.clone() } else { format!("{n}.{}", x.face.join(".")) }
}

/// The names of the faces of `x`'s exact boundary its path names: all of them where it names
/// none, else those whose path from the solid (the operand's route, then the face) is the path or
/// lies under it — `near`, `boss.wall`, or `boss` for every face the operand `boss` made.
fn faces_of(sk: &Sketch, x: &FilletSide, brep: &Brep) -> std::collections::BTreeSet<String> {
    let all = brep.faces.iter().map(|f| f.name.clone());
    if x.face.is_empty() { return all.collect(); }
    let want = x.face.join(".");
    let routes = operand_paths(sk, x.solid as usize);
    all.filter(|name| routes.iter().any(|(prim, route)| {
        let Some(face) = name.strip_prefix(prim.as_str()).and_then(|r| r.strip_prefix('.')) else { return false };
        let path = if route.is_empty() { face.to_string() } else { format!("{route}.{face}") };
        path == want || path.starts_with(&format!("{want}."))
    })).collect()
}

/// For every edge, the faces using it and which use.
fn uses_by_edge(b: &Brep) -> Vec<Vec<Use>> {
    let mut out = vec![Vec::new(); b.edges.len()];
    for (fi, f) in b.faces.iter().enumerate() {
        for (li, l) in f.loops.iter().enumerate() {
            for (k, c) in l.iter().enumerate() { out[c.edge as usize].push((fi, (li, k))); }
        }
    }
    out
}

fn unit(a: V) -> V { scale(a, 1.0 / norm(a)) }

/// `b` turned through `angle` about the line through its origin along its `v`, right-handed.
fn turned_basis(b: &Basis, angle: f64) -> Basis { Basis { u: Rigid::turn(b.o, b.v, angle).vector(b.u), ..*b } }

/// `b` carried `k` along its own normal.
fn carried(b: &Basis, k: f64) -> Basis { Basis { o: add(b.o, scale(b.normal(), k)), ..*b } }

/// Whether a turn is a whole one.
fn whole(sweep: f64) -> bool { sweep >= TAU - 1e-9 }

/// An end of a piece: its piece, the vertex it stands at and whose fillet it is; flush in the
/// faces beyond it, or open (`meet`) for the piece carried on past it.
#[derive(Clone, Debug)]
struct End { piece: usize, vertex: u32, label: String, meet: Option<Meet> }

/// Where an open end's ball stands, for the next piece to meet: `outward` the way the piece
/// leaves its end, `refusal` what is said if nothing meets it.
#[derive(Clone, Debug)]
struct Meet { outward: V, corner: V, centre: V, touch: [V; 2], refusal: String }

/// **A chain of tangent edges is rounded piece by piece**: every open end must meet exactly one
/// other, leaving the vertex the opposite way with its ball where this one's is — the same
/// section, so the two pieces share a cap and their union is exact. The pieces joined so;
/// anything else turns a corner (rung 3): the first end left unmet.
fn paired(ends: &[End], tol: f64) -> Result<Vec<[usize; 2]>, (&End, &Meet)> {
    let near = 8.0 * tol;
    let same = |a: V, b: V| norm(sub(a, b)) <= near;
    let open: Vec<(&End, &Meet)> = ends.iter().filter_map(|e| e.meet.as_ref().map(|m| (e, m))).collect();
    let mut joins = Vec::new();
    for (i, &(e, m)) in open.iter().enumerate() {
        let at: Vec<&(&End, &Meet)> = open.iter().enumerate().filter(|&(j, o)| j != i && o.0.vertex == e.vertex).map(|(_, o)| o).collect();
        let [&(o, n)] = at.as_slice() else { return Err((e, m)) };
        if dot(m.outward, n.outward) > -1.0 + 1e-9 || !same(m.corner, n.corner) || !same(m.centre, n.centre)
            || !(0..2).all(|k| same(m.touch[k], n.touch[k])) {
            return Err((e, m));
        }
        if e.piece < o.piece { joins.push([e.piece, o.piece]); }
    }
    Ok(joins)
}

/// One edge being rounded, in the boundary's millimetres: its two uses, the first the first side's,
/// and every edge's uses beside it.
struct Edge<'a> {
    brep: &'a Brep,
    located: &'a Located<'a>,
    table: &'a [Vec<Use>],
    edge: usize,
    uses: [Use; 2],
    tol: f64,
}

/// How far round a circle a point is from where a band starts, turning the band's way: the
/// coordinate across a band on a cylinder (in the section) or a sphere or torus (in the meridian).
#[derive(Clone, Copy, Debug)]
struct Turned { centre: [f64; 2], start: f64, sense: f64, extent: f64, radius: f64 }

impl Turned {
    /// The stroke from the corner to a touch along a circle side, as a band's turn.
    fn of(stroke: &Stroke) -> Option<Turned> {
        let Stroke::Arc { centre, radius, start, sweep, .. } = *stroke else { return None };
        Some(Turned { centre, start, sense: sweep.signum(), extent: sweep.abs(), radius })
    }
    /// The turn to `p`, in `[0, 2π)`.
    fn at(&self, p: [f64; 2]) -> f64 { around(self.sense * (angle(self.centre, p) - self.start), 0.0, TAU) }
    /// Whether a turn lies within the band, clear of its ends by `tol` (a length).
    fn inside(&self, psi: f64, tol: f64) -> bool {
        let slack = tol / self.radius;
        psi > slack && psi < self.extent - slack
    }
    /// Whether the stretch of turn from `p0` through `pm` to `p1` (the ends and middle of an arc)
    /// overlaps the band, clear of its ends by `tol`.
    fn overlaps(&self, [p0, pm, p1]: [[f64; 2]; 3], tol: f64) -> bool {
        let (a, m, b) = (self.at(p0), self.at(pm), self.at(p1));
        let after = |x: f64, from: f64| around(x - from, 0.0, TAU);
        // the stretch as `[lo, lo + len]`, walked up through the middle
        let (lo, len) = if after(m, a) <= after(b, a) { (a, after(b, a)) } else { (b, after(a, b)) };
        let slack = tol / self.radius;
        len > 0.0 && (lo < self.extent - slack || lo + len > TAU + slack)
    }
}

/// Where the ball rolls on one face, in coordinates its edges are read in.
enum Band {
    /// A plane face beside a straight edge: a rectangle, along the edge from `corner` (`along`,
    /// through `length`) and across it into the face (`d`, through the setback).
    Strip { corner: V, along: V, d: V, length: f64, setback: f64 },
    /// A cylinder along a straight edge: along the edge from the section's origin (`along`, through
    /// `length`) and round the axis from the corner to the touch, read in the section.
    Sleeve { section: Basis, length: f64, turn: Turned },
    /// A face turned about the axis through `o` along `axis` whose meridian is a line: the
    /// stretch of it from the corner along `dm` (radius, height), at every turn of a whole ring or
    /// within a partial ring's `sector`.
    Meridian { o: V, axis: V, corner: [f64; 2], dm: [f64; 2], setback: f64, sector: Option<Sector> },
    /// A sphere or a torus turned about the axis: the arc of its meridian from the corner to the
    /// touch, at every turn of a whole ring or within a partial ring's `sector`.
    Arc { o: V, axis: V, turn: Turned, sector: Option<Sector> },
}

/// The stretch of turn a partial ring sweeps about the axis through `o` along the unit `axis`:
/// from the radial `start`, right-handed through `sweep`.
#[derive(Clone, Copy, Debug)]
struct Sector { o: V, axis: V, start: V, sweep: f64 }

impl Sector {
    /// A point in the plane square to the axis: along `start`, and a quarter turn on.
    fn flat(&self, q: V) -> [f64; 2] {
        let w = sub(q, self.o);
        [dot(w, self.start), dot(w, cross(self.axis, self.start))]
    }
    /// The sector's stretch of turn at radius `rho`, as a band's.
    fn turn(&self, rho: f64) -> Turned { Turned { centre: [0.0, 0.0], start: 0.0, sense: 1.0, extent: self.sweep, radius: rho } }
    /// Whether a point (flat) lies within the sector, clear of its two sides by `tol`.
    fn holds(&self, f: [f64; 2], tol: f64) -> bool {
        let rho = len2(f);
        let turn = self.turn(rho);
        rho > tol && turn.inside(turn.at(f), tol)
    }
    /// Whether a circle about the axis over `t` passes within the sector: its stretch of turn,
    /// walked from one end through its middle to the other, overlaps the sector's.
    fn arc_overlaps(&self, curve: &Curve, t: [f64; 2], tol: f64) -> bool {
        let pts = [t[0], 0.5 * (t[0] + t[1]), t[1]].map(|s| self.flat(curve.point(s)));
        whole(t[1] - t[0]) || self.turn(len2(pts[0]).max(tol)).overlaps(pts, tol)
    }
    /// Whether a stretch cut where both conditions can change has a piece strictly between the
    /// radii `lo` and `hi` and within the sector, asked at each piece's middle.
    fn pieces_in(&self, mut cuts: Vec<f64>, at: impl Fn(f64) -> [f64; 2], lo: f64, hi: f64, tol: f64) -> bool {
        cuts.sort_by(f64::total_cmp);
        cuts.windows(2).any(|w| {
            let m = at(0.5 * (w[0] + w[1]));
            let rho = len2(m);
            w[1] > w[0] && rho > lo && rho < hi && self.holds(m, tol)
        })
    }
    /// Whether the circle `c + r (cos s x + sin s y)` (flat), `s` over `t`, has a stretch strictly
    /// between the radii `lo` and `hi` and within the sector: cut where it crosses either circle
    /// or either side, in closed form (`A cos s + B sin s = K`).
    #[allow(clippy::too_many_arguments)]
    fn arc_in(&self, c: [f64; 2], x: [f64; 2], y: [f64; 2], r: f64, t: [f64; 2], lo: f64, hi: f64, tol: f64) -> bool {
        if !(lo < hi) { return false; }
        let mut cuts = vec![t[0], t[1]];
        let mut cross = |a: f64, b: f64, k: f64| {
            let amp = a.dhypot(b);
            if amp == 0.0 || k.abs() > amp { return; }
            let (base, off) = (b.datan2(a), (k / amp).dacos());
            for s in [base + off, base - off] {
                let s = around(s, t[0], TAU);
                if s <= t[1] { cuts.push(s); }
            }
        };
        for rho in [lo, hi] { cross(dot2(c, x), dot2(c, y), (rho * rho - dot2(c, c) - r * r) / (2.0 * r)); }
        for side in [0.0, self.sweep] {
            let (sn, cs) = side.dsin_cos();
            let n = [-sn, cs];
            cross(dot2(n, x), dot2(n, y), -dot2(n, c) / r);
        }
        self.pieces_in(cuts, |s| { let (sn, cs) = s.dsin_cos(); [c[0] + r * (cs * x[0] + sn * y[0]), c[1] + r * (cs * x[1] + sn * y[1])] },
            lo, hi, tol)
    }
    /// Whether the segment from `a` to `b` (flat) has a stretch strictly between the radii `lo`
    /// and `hi` and within the sector. Where either can change — the segment's crossings of the
    /// two circles and of the sector's two sides — cuts it into stretches on which neither does,
    /// and each stretch is asked at its middle.
    fn segment_in(&self, a: [f64; 2], b: [f64; 2], lo: f64, hi: f64, tol: f64) -> bool {
        if !(lo < hi) { return false; }
        let d = sub2(b, a);
        let mut cuts = vec![0.0, 1.0];
        let (qa, qb) = (dot2(d, d), 2.0 * dot2(a, d));
        for rho in [lo, hi] {
            let qc = dot2(a, a) - rho * rho;
            let disc = qb * qb - 4.0 * qa * qc;
            if qa > 0.0 && disc >= 0.0 {
                cuts.extend([-1.0, 1.0].map(|sg| (-qb + sg * disc.sqrt()) / (2.0 * qa)));
            }
        }
        for side in [0.0, self.sweep] {
            let (sn, cs) = side.dsin_cos();
            let n = [-sn, cs];
            let den = dot2(n, d);
            if den != 0.0 { cuts.push(-dot2(n, a) / den); }
        }
        cuts.retain(|&c| (0.0..=1.0).contains(&c));
        self.pieces_in(cuts, |s| add2(a, d, s), lo, hi, tol)
    }
}

impl Band {
    /// Whether an edge of the face over `t` enters the band; `None` where there is no closed
    /// form for it.
    fn enters(&self, curve: &Curve, t: [f64; 2], tol: f64) -> Option<bool> {
        let mer = |o: V, axis: V, q: V| { let z = dot(sub(q, o), axis); [norm(sub(sub(q, o), scale(axis, z))), z] };
        let coaxial = |o: V, axis: V, frame: &crate::brep::geom::Frame|
            norm(cross(frame.z, axis)) <= 1e-9 && norm(cross(sub(frame.o, o), axis)) <= tol;
        match *self {
            Band::Strip { corner, along, d, length, setback } => {
                let to2 = |q: V| { let w = sub(q, corner); [dot(w, along), dot(w, d)] };
                let (lo, hi) = ([tol, tol], [length - tol, setback - tol]);
                match curve {
                    Curve::Line { .. } => Some(segment_enters(to2(curve.point(t[0])), to2(curve.point(t[1])), lo, hi)),
                    Curve::Circle(frame, rc) => {
                        if dot(frame.z, cross(along, d)).abs() < 1.0 - 1e-9 { return None; }
                        let c2 = to2(frame.o);
                        let x2 = [dot(frame.x, along), dot(frame.x, d)];
                        let y2 = [dot(frame.y, along), dot(frame.y, d)];
                        Some(arc_enters(c2, x2, y2, *rc, t, lo, hi))
                    }
                    _ => None,
                }
            }
            Band::Sleeve { section, length, turn } => {
                let along = section.normal();
                let flat = |q: V| { let w = sub(q, section.o); [dot(w, section.u), dot(w, section.v)] };
                let at = |q: V| dot(sub(q, section.o), along);
                let within = |x: f64| x > tol && x < length - tol;
                match curve {
                    // a generator: one turn, a stretch along
                    Curve::Line { d: dl, .. } if norm(cross(*dl, along)) <= 1e-9 => {
                        let (a, b) = (at(curve.point(t[0])), at(curve.point(t[1])));
                        Some(turn.inside(turn.at(flat(curve.point(t[0]))), tol)
                            && a.max(b) > tol && a.min(b) < length - tol)
                    }
                    // a circle square to the axis: one place along, a stretch of turn
                    Curve::Circle(frame, _) if norm(cross(frame.z, along)) <= 1e-9 => {
                        let pts = [t[0], 0.5 * (t[0] + t[1]), t[1]].map(|s| flat(curve.point(s)));
                        Some(within(at(frame.o)) && turn.overlaps(pts, tol))
                    }
                    _ => None,
                }
            }
            Band::Meridian { o, axis, corner, dm, setback, sector } => {
                let w = |m: [f64; 2]| (m[0] - corner[0]) * dm[0] + (m[1] - corner[1]) * dm[1];
                let (a, b) = match curve {
                    Curve::Circle(frame, rc) if coaxial(o, axis, frame) => {
                        if sector.is_some_and(|s| !s.arc_overlaps(curve, t, tol)) { return Some(false); }
                        let v = w([*rc, dot(sub(frame.o, o), axis)]);
                        (v, v)
                    }
                    // in a plane square to the axis, a plane face's band is radial: a partial
                    // ring's, a stretch of radius within its sector
                    _ if dm[1].abs() <= 1e-12 => {
                        if let Some(s) = sector {
                            let (lo, hi) = if dm[0] > 0.0 { (corner[0] + tol, corner[0] + setback - tol) }
                                else { (corner[0] - setback + tol, corner[0] - tol) };
                            let quarter = cross(s.axis, s.start);
                            return match curve {
                                Curve::Line { .. } => Some(s.segment_in(s.flat(curve.point(t[0])), s.flat(curve.point(t[1])), lo, hi, tol)),
                                Curve::Circle(frame, rc) if norm(cross(frame.z, s.axis)) <= 1e-9 => {
                                    let along = |v: V| [dot(v, s.start), dot(v, quarter)];
                                    Some(s.arc_in(s.flat(frame.o), along(frame.x), along(frame.y), *rc, t, lo, hi, tol))
                                }
                                _ => None,
                            };
                        }
                        let (lo, hi) = radius_range(curve, t, o, axis)?;
                        (w([lo, corner[1]]), w([hi, corner[1]]))
                    }
                    // a line in a meridian: radius and height both affine along it, at one turn
                    Curve::Line { p: q, d: dl } if dot(cross(*dl, axis), sub(*q, o)).abs() <= tol => {
                        let ends = [curve.point(t[0]), curve.point(t[1])];
                        if let Some(s) = sector {
                            let far = if mer(o, axis, ends[0])[0] >= mer(o, axis, ends[1])[0] { ends[0] } else { ends[1] };
                            if !s.holds(s.flat(far), tol) { return Some(false); }
                        }
                        (w(mer(o, axis, ends[0])), w(mer(o, axis, ends[1])))
                    }
                    _ => return None,
                };
                Some(a.max(b) > tol && a.min(b) < setback - tol)
            }
            Band::Arc { o, axis, turn, sector } => match curve {
                // a circle about the axis is one point of the meridian
                Curve::Circle(frame, rc) if coaxial(o, axis, frame) => {
                    if sector.is_some_and(|s| !s.arc_overlaps(curve, t, tol)) { return Some(false); }
                    Some(turn.inside(turn.at([*rc, dot(sub(frame.o, o), axis)]), tol))
                }
                _ => None,
            },
        }
    }
}

impl Edge<'_> {
    fn face_name(&self, k: usize) -> &str { &self.brep.faces[self.uses[k].0].name }

    /// The outward normal of face `fi` at `p`, a point of it.
    fn normal(&self, fi: usize, p: V) -> Result<V, String> {
        let f = &self.brep.faces[fi];
        let n = f.surface.normal(self.located.uv(fi, p))
            .ok_or_else(|| format!("`{}` has no normal where it is rounded", f.name))?;
        Ok(if f.reversed { scale(n, -1.0) } else { n })
    }

    /// Its piece (millimetres, about the boundary's origin), whether it is concave and its ends
    /// that are not flush (for the pieces it continues into to meet); none where the two faces run
    /// on into one another (two operands' coplanar faces), with no edge to round.
    fn round(&self, r: f64) -> Result<Option<(Piece, bool, Vec<End>)>, String> {
        let e = &self.brep.edges[self.edge];
        let EdgeCurve::Curve(curve) = &e.curve else {
            return Err(format!("`{}` meets `{}` at a point", self.face_name(0), self.face_name(1)));
        };
        let mid = 0.5 * (e.t[0] + e.t[1]);
        let p = curve.point(mid);
        let tangent = unit(curve.tangent(mid));
        // walking a loop with its face's normal up, the face lies on the left: `n × τ` points
        // into the face, square to the edge
        let mut n = [[0.0; 3]; 2];
        let mut d = [[0.0; 3]; 2];
        for k in 0..2 {
            let (fi, (li, ci)) = self.uses[k];
            let c = &self.brep.faces[fi].loops[li][ci];
            n[k] = self.normal(fi, p)?;
            let tau = if c.reversed { scale(tangent, -1.0) } else { tangent };
            d[k] = unit(cross(n[k], tau));
        }
        if dot(d[0], d[1]) <= -1.0 + SMOOTH { return Ok(None); }
        let concave = dot(d[1], n[0]) > 0.0;
        let surface = |k: usize| &self.brep.faces[self.uses[k].0].surface;
        let (section, corner, carry, sides) = match curve {
            Curve::Line { d: along, .. } => {
                let (u, start) = (d[0], curve.point(e.t[0]));
                let section = Basis { u, v: cross(*along, u), o: start };
                let (Some(a), Some(b)) = (prism_side(surface(0), &section), prism_side(surface(1), &section))
                    else { return Err(self.unsupported("line")) };
                (section, [0.0, 0.0], Carry::Prism { length: e.t[1] - e.t[0] }, [a, b])
            }
            Curve::Circle(frame, rc) => {
                let axis = frame.z;
                let (Some(a), Some(b)) = (meridian_side(surface(0), frame.o, axis, self.tol),
                    meridian_side(surface(1), frame.o, axis, self.tol))
                    else { return Err(self.unsupported("circle")) };
                // a whole ring, or the stretch of one the edge runs (its parameter turns about `z`)
                let sweep = if e.closed() { TAU } else { e.t[1] - e.t[0] };
                let u = unit(sub(p, add(frame.o, scale(axis, dot(sub(p, frame.o), axis)))));
                (Basis { u, v: axis, o: frame.o }, [*rc, 0.0], Carry::Turn { sweep }, [a, b])
            }
            other => return Err(self.unsupported(other.kind())),
        };
        let flat = |x: V| [dot(x, section.u), dot(x, section.v)];
        // a ring's section stands where its edge starts: the middle's turned back half the sweep;
        // a partial ring's band is the stretch of turn it sweeps from there
        let start = match carry { Carry::Turn { sweep } => turned_basis(&section, -0.5 * sweep), _ => section };
        let sector = |sweep: f64| (!whole(sweep)).then_some(Sector { o: section.o, axis: section.v, start: start.u, sweep });
        if d.iter().any(|x| dot(*x, section.normal()).abs() > 1e-9) {
            return Err(format!("`{}` and `{}` do not meet square to their section",
                self.face_name(0), self.face_name(1)));
        }
        let wedge = Wedge::new(corner, sides, [flat(d[0]), flat(d[1])], r).map_err(|why| match why {
            Misfit::Angle => format!("`{}` and `{}` meet too nearly flat or too sharply to round",
                self.face_name(0), self.face_name(1)),
            Misfit::Ball => format!("no ball of radius {r} fits between `{}` and `{}`",
                self.face_name(0), self.face_name(1)),
        })?;
        // the corner's material says the same as the normals: a ball in the wedge is in the
        // material at a convex edge and out of it at a concave one
        let toward = sub(section.lift(wedge.centre[0], wedge.centre[1]), section.lift(corner[0], corner[1]));
        let place = self.located.solid_place(add(p, scale(toward, 0.01)));
        if place != if concave { Place::Out } else { Place::In } {
            return Err(format!("cannot tell which side of `{}` with `{}` is material",
                self.face_name(0), self.face_name(1)));
        }
        let strokes = wedge.strokes();
        if matches!(carry, Carry::Turn { .. }) && strokes.iter().any(|s| s.least_x() <= self.tol) {
            return Err(format!("the ball between `{}` and `{}` reaches past their axis",
                self.face_name(0), self.face_name(1)));
        }
        for k in 0..2 {
            // the stroke from the corner to this side's touch
            let stroke = strokes[2 * k];
            let setback = len2(sub2(wedge.touch[k], corner));
            let band = match (carry, sides[k]) {
                (Carry::Prism { length }, Side::Line) => Band::Strip {
                    corner: section.lift(corner[0], corner[1]), along: section.normal(), d: d[k], length, setback },
                (Carry::Prism { length }, Side::Circle { .. }) => Band::Sleeve {
                    section, length, turn: turned_from(stroke, k) },
                (Carry::Turn { sweep }, Side::Line) => Band::Meridian {
                    o: section.o, axis: section.v, corner, dm: flat(d[k]), setback, sector: sector(sweep) },
                (Carry::Turn { sweep }, Side::Circle { .. }) =>
                    Band::Arc { o: section.o, axis: section.v, turn: turned_from(stroke, k), sector: sector(sweep) },
            };
            self.holds(k, &band)?;
        }
        let piece = Piece { section: start, wedge, carry };
        // each end flush, or open for the piece it continues into
        let mut ends = Vec::new();
        if !piece.whole() {
            let fd = [flat(d[0]), flat(d[1])];
            let label = format!("`{}` with `{}`", self.face_name(0), self.face_name(1));
            for end in 0..2 {
                let at = piece.at(end as f64);
                let w = &piece.wedge;
                let touch = w.touch.map(|t| at.lift(t[0], t[1]));
                let meet = match self.stops(end, &at, fd, touch) {
                    Ok(()) => None,
                    Err(refusal) => {
                        // the way the piece is carried there: along a prism, round a ring
                        let carried = match carry { Carry::Prism { .. } => at.normal(), Carry::Turn { .. } => cross(at.v, at.u) };
                        Some(Meet {
                            outward: scale(carried, if end == 0 { -1.0 } else { 1.0 }),
                            corner: at.lift(w.corner[0], w.corner[1]),
                            centre: at.lift(w.centre[0], w.centre[1]),
                            touch,
                            refusal,
                        })
                    }
                };
                ends.push(End { piece: 0, vertex: e.v[end], label: label.clone(), meet });
            }
        }
        Ok(Some((piece, concave, ends)))
    }

    fn unsupported(&self, curve: &str) -> String {
        let kind = |k: usize| self.brep.faces[self.uses[k].0].surface.kind();
        format!("`{}` ({}) meets `{}` ({}) on a {curve} no ball rolls along in closed form: a straight \
                 edge between planes and cylinders along it, or a circle between surfaces turned about \
                 its axis, is rounded in closed form, and a closed loop traced (rung 2)",
            self.face_name(0), kind(0), self.face_name(1), kind(1))
    }

    /// **A ball no larger than face `k` can hold**: the band of the face within the setback of the
    /// edge, where the ball rolls, crossed by none of the face's other edges. Read in closed form
    /// for lines and circles; anything else is refused, never sampled.
    fn holds(&self, k: usize, band: &Band) -> Result<(), String> {
        let (fi, (li, ci)) = self.uses[k];
        let f = &self.brep.faces[fi];
        let seams = f.seams();
        for (lj, l) in f.loops.iter().enumerate() {
            for (cj, c) in l.iter().enumerate() {
                if (lj, cj) == (li, ci) || seams.contains(&c.edge) { continue; }
                let e = &self.brep.edges[c.edge as usize];
                let EdgeCurve::Curve(curve) = &e.curve else { continue };
                match band.enters(curve, e.t, self.tol) {
                    Some(false) => {}
                    Some(true) => return Err(format!("the ball of `{}` with `{}` is larger than `{}` can hold",
                        self.face_name(0), self.face_name(1), f.name)),
                    None => return Err(format!(
                        "cannot certify that `{}` holds the ball of `{}` with `{}` (an edge on a {})",
                        f.name, self.face_name(0), self.face_name(1), curve.kind())),
                }
            }
        }
        Ok(())
    }

    /// **A fillet stops flush at end `end`**, its section there `at`: on each face the edge beside
    /// it at that end leaves the corner across the band (along `d`, in the section) and reaches the
    /// touch there (`touch`), lying in a plane that is the section's — so the ball's section
    /// stands in one plane there and the fillet ends in it. A face running on past the end needs
    /// the ball to turn the corner (rung 3), or the next piece of a chain to carry it on.
    fn stops(&self, end: usize, at: &Basis, d: [[f64; 2]; 2], touch: [V; 2]) -> Result<(), String> {
        let vertex = self.brep.edges[self.edge].v[end];
        let along = at.normal();
        let d = d.map(|f| add(scale(at.u, f[0]), scale(at.v, f[1])));
        for k in 0..2 {
            let (fi, (li, ci)) = self.uses[k];
            let l = &self.brep.faces[fi].loops[li];
            let n = l.len();
            let runs_on = || format!(
                "the fillet of `{}` with `{}` runs on past its end into `{}`: a fillet turning a corner is rung 3",
                self.face_name(0), self.face_name(1), self.brep.faces[fi].name);
            // the use before or after ours that shares this vertex
            let next = [(ci + 1) % n, (ci + n - 1) % n].into_iter()
                .find(|&cj| cj != ci && self.brep.ends(&l[cj]).contains(&vertex)).ok_or_else(runs_on)?;
            let c = &l[next];
            let ne = &self.brep.edges[c.edge as usize];
            let EdgeCurve::Curve(curve) = &ne.curve else { return Err(runs_on()) };
            // it leaves the corner into the band...
            let from_start = ne.v[0] == vertex;
            let leaving = scale(unit(curve.tangent(if from_start { ne.t[0] } else { ne.t[1] })),
                if from_start { 1.0 } else { -1.0 });
            if dot(leaving, d[k]) < 1.0 - 1e-9 { return Err(runs_on()); }
            // ...and runs on as far as the touch
            let mut t = curve.inverse(touch[k]);
            if let Some(period) = curve.period() { t = around(t, ne.t[0], period); }
            let near = 8.0 * self.tol.max(1e-9);
            if !(t >= ne.t[0] - 1e-9 && t <= ne.t[1] + 1e-9) || norm(sub(curve.point(t), touch[k])) > near {
                return Err(runs_on());
            }
            // the face across that edge is the plane the fillet ends in
            let across = self.table[c.edge as usize].iter().map(|u| u.0).find(|&g| g != fi).ok_or_else(runs_on)?;
            let Surface::Plane(frame) = &self.brep.faces[across].surface else { return Err(runs_on()) };
            if dot(frame.z, along).abs() < 1.0 - 1e-9 { return Err(runs_on()); }
        }
        Ok(())
    }
}

/// The band's turn along the stroke from the corner to side `k`'s touch — the section's first
/// stroke for the first side, its last walked back for the second.
fn turned_from(stroke: Stroke, k: usize) -> Turned {
    let stroke = match (stroke, k) {
        (Stroke::Arc { centre, radius, start, sweep, from, to }, 1) =>
            Stroke::Arc { centre, radius, start: start + sweep, sweep: -sweep, from: to, to: from },
        (s, _) => s,
    };
    Turned::of(&stroke).expect("a circle side's stroke is an arc")
}

/// A face's trace in the section square to a straight edge: a plane's line, or the circle of a
/// cylinder whose axis runs along the edge; none for any other face.
fn prism_side(s: &Surface, section: &Basis) -> Option<Side> {
    let along = section.normal();
    match s {
        Surface::Plane(_) => Some(Side::Line),
        Surface::Cylinder(f, radius) if norm(cross(f.z, along)) <= 1e-9 => {
            let foot = sub(sub(f.o, section.o), scale(along, dot(sub(f.o, section.o), along)));
            Some(Side::Circle { centre: [dot(foot, section.u), dot(foot, section.v)], radius: *radius })
        }
        _ => None,
    }
}

/// A surface of revolution about the line through `o` along the unit `axis`, read in its meridian
/// `(ρ, z)` from `o`: a line for a plane square to the axis or a cylinder or cone about it, a
/// circle for a sphere centred on it or a torus about it; none for any other.
fn meridian_side(s: &Surface, o: V, axis: V, tol: f64) -> Option<Side> {
    let parallel = |z: V| norm(cross(z, axis)) <= 1e-9;
    let on_axis = |c: V| norm(cross(sub(c, o), axis)) <= tol;
    let height = |c: V| dot(sub(c, o), axis);
    match s {
        Surface::Plane(f) if parallel(f.z) => Some(Side::Line),
        Surface::Cylinder(f, _) | Surface::Cone(f, _, _) if parallel(f.z) && on_axis(f.o) => Some(Side::Line),
        Surface::Sphere(f, radius) if on_axis(f.o) => Some(Side::Circle { centre: [0.0, height(f.o)], radius: *radius }),
        Surface::Torus(f, big, small) if parallel(f.z) && on_axis(f.o) =>
            Some(Side::Circle { centre: [*big, height(f.o)], radius: *small }),
        _ => None,
    }
}


/// The least and greatest distance from the axis through `o` along `axis` over a curve lying in a
/// plane square to the axis: exact for a line and a circle, `None` for any other curve.
fn radius_range(c: &Curve, t: [f64; 2], o: V, axis: V) -> Option<(f64, f64)> {
    let rho = |q: V| { let w = sub(q, o); norm(sub(w, scale(axis, dot(w, axis)))) };
    let (a, b) = (c.point(t[0]), c.point(t[1]));
    let mut lo = rho(a).min(rho(b));
    let mut hi = rho(a).max(rho(b));
    match c {
        // the foot of the axis on the line, where it lies within the stretch
        Curve::Line { p, d } => {
            let foot = add(o, scale(axis, dot(sub(*p, o), axis)));
            let s = dot(sub(foot, *p), *d).clamp(t[0], t[1]);
            lo = lo.min(rho(c.point(s)));
        }
        // nearest and farthest where the circle's radius through the axis meets it
        Curve::Circle(f, _) => {
            let foot = add(o, scale(axis, dot(sub(f.o, o), axis)));
            let to = sub(foot, f.o);
            let base = dot(to, f.y).datan2(dot(to, f.x));
            for s in [base, base + std::f64::consts::PI] {
                let s = around(s, t[0], std::f64::consts::TAU);
                if s <= t[1] { let x = rho(c.point(s)); lo = lo.min(x); hi = hi.max(x); }
            }
        }
        _ => return None,
    }
    Some((lo, hi))
}

/// Whether the segment from `a` to `b` passes into the open box between `lo` and `hi`.
fn segment_enters(a: [f64; 2], b: [f64; 2], lo: [f64; 2], hi: [f64; 2]) -> bool {
    if !(lo[0] < hi[0] && lo[1] < hi[1]) { return false; }
    let (mut t0, mut t1) = (0.0f64, 1.0f64);
    for k in 0..2 {
        let dk = b[k] - a[k];
        if dk == 0.0 {
            if a[k] <= lo[k] || a[k] >= hi[k] { return false; }
            continue;
        }
        let (mut s0, mut s1) = ((lo[k] - a[k]) / dk, (hi[k] - a[k]) / dk);
        if s0 > s1 { std::mem::swap(&mut s0, &mut s1); }
        t0 = t0.max(s0);
        t1 = t1.min(s1);
    }
    t0 < t1
}

/// Whether the arc `c + r (cos s x + sin s y)`, `s` over `t`, passes into the open box between
/// `lo` and `hi` (in the box's coordinates): an end inside, or a crossing of a side within it.
fn arc_enters(c: [f64; 2], x: [f64; 2], y: [f64; 2], r: f64, t: [f64; 2], lo: [f64; 2], hi: [f64; 2]) -> bool {
    if !(lo[0] < hi[0] && lo[1] < hi[1]) { return false; }
    let at = |s: f64| { let (sn, cs) = s.dsin_cos(); [c[0] + r * (cs * x[0] + sn * y[0]), c[1] + r * (cs * x[1] + sn * y[1])] };
    let inside = |p: [f64; 2]| (0..2).all(|k| p[k] > lo[k] && p[k] < hi[k]);
    if inside(at(t[0])) || inside(at(t[1])) || inside(at(0.5 * (t[0] + t[1]))) { return true; }
    let within = |s: f64| around(s, t[0], std::f64::consts::TAU) <= t[1];
    for k in 0..2 {
        // r (cos s x_k + sin s y_k) = side - c_k
        let (a, b) = (r * x[k], r * y[k]);
        let amp = a.dhypot(b);
        if amp == 0.0 { continue; }
        let base = b.datan2(a);
        for side in [lo[k], hi[k]] {
            let q = (side - c[k]) / amp;
            if q.abs() > 1.0 { continue; }
            let off = q.dacos();
            for s in [base + off, base - off] {
                if !within(s) { continue; }
                let p = at(s);
                let o = 1 - k;
                if p[o] > lo[o] && p[o] < hi[o] { return true; }
            }
        }
    }
    false
}

impl Piece {
    /// Its section as the faceted kernel sweeps one (`primitive::prism`, `primitive::revolve`):
    /// the loop in the section's own coordinates, the arc cut to the sheet's flatness at `unit`,
    /// about the evaluation's `origin`.
    pub(super) fn face_poly(&self, origin: [f64; 3], unit: f64) -> Option<FacePoly> {
        // each stroke from its start, an arc as chords within the flatness
        let mut pts = Vec::new();
        let mut of = Vec::new();
        for (k, stroke) in self.wedge.strokes().into_iter().enumerate() {
            match stroke {
                Stroke::Line { from, .. } => {
                    pts.push((from[0], from[1]));
                    of.push((k, false));
                }
                Stroke::Arc { centre, radius, start, sweep, .. } => {
                    let arc = super::profile::tessellate_arc((centre[0], centre[1]), radius, start, sweep, unit);
                    for p in &arc[..arc.len() - 1] {
                        pts.push(*p);
                        of.push((k, true));
                    }
                }
            }
        }
        let basis = Basis { o: std::array::from_fn(|k| self.section.o[k] - origin[k]), ..self.section };
        let poly = FacePoly { pts, of, names: EDGE_NAMES.map(String::from).to_vec(), basis,
            curved: vec![None; EDGE_NAMES.len()] };
        poly.valid().then(|| poly.ccw())
    }
}

/// A rolled piece as one polyhedral primitive of the faceted kernel: its exact boundary meshed
/// within the sheet's flatness at `unit`, about the evaluation's `origin`, each facet on the face
/// it meshes (`round`, `a`, `b`).
pub(super) fn rolled_prim(roll: &Roll, origin: [f64; 3], unit: f64, of: &str) -> Option<Prim> {
    let b = &roll.rolled.piece;
    let bar = crate::curve::flatness(unit) * roll.mm;
    let m = crate::brep::mesh::mesh(b, bar.max(1e-6 * b.size()), TAU / 64.0).ok()?;
    let faces: Vec<String> = b.faces.iter().map(|f| f.name.clone()).collect();
    let at = |p: V| -> V { crate::space::sub(roll.model(p), origin) };
    let facets = m.tris.iter().zip(&m.of).filter_map(|(t, &fi)| {
        let pts: Vec<V> = t.iter().map(|&i| at(m.pts[i as usize])).collect();
        let n = super::primitive::facet_normal(&pts)?;
        let smooth = !matches!(b.faces[fi as usize].surface, Surface::Plane(_));
        Some(Facet { pts, n, face: fi as usize, smooth })
    }).collect();
    Some(super::primitive::finish(facets, faces, of))
}
