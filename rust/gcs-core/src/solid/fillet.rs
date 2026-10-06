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

/// The ball's section at one edge, in its section plane's coordinates: the corner the two faces
/// meet at, the points where the ball touches the first face and the second, and its centre.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Wedge {
    pub corner: [f64; 2],
    pub touch: [[f64; 2]; 2],
    pub centre: [f64; 2],
    pub r: f64,
}

impl Wedge {
    /// The ball of radius `r` in the corner at `corner` between the unit directions `d` (each
    /// along a face, away from the edge): `None` where the two are too near parallel to round.
    fn new(corner: [f64; 2], d: [[f64; 2]; 2], r: f64) -> Option<Wedge> {
        let cos = (d[0][0] * d[1][0] + d[0][1] * d[1][1]).clamp(-1.0, 1.0);
        let theta = cos.dacos();
        if !(theta > MIN_TURN && theta < std::f64::consts::PI - MIN_TURN) { return None; }
        let half = theta / 2.0;
        let setback = r / half.dtan();
        let bis = [d[0][0] + d[1][0], d[0][1] + d[1][1]];
        let bl = bis[0].dhypot(bis[1]);
        let reach = r / half.dsin();
        let at = |u: [f64; 2], k: f64| [corner[0] + k * u[0], corner[1] + k * u[1]];
        Some(Wedge {
            corner,
            touch: [at(d[0], setback), at(d[1], setback)],
            centre: at([bis[0] / bl, bis[1] / bl], reach),
            r,
        })
    }

    /// How far along each face the ball touches it.
    pub fn setback(&self) -> f64 {
        (self.touch[0][0] - self.corner[0]).dhypot(self.touch[0][1] - self.corner[1])
    }

    /// The ball's arc between the touches as a counter-clockwise turn about the centre, the short
    /// way (past the corner): its start angle, its sweep, and its ends in that order — the second
    /// touch first where the turn runs from it.
    pub fn arc(&self) -> (f64, f64, [[f64; 2]; 2]) {
        let angle = |p: [f64; 2]| (p[1] - self.centre[1]).datan2(p[0] - self.centre[0]);
        let [t0, t1] = self.touch;
        let ccw = (angle(t1) - angle(t0)).rem_euclid(std::f64::consts::TAU);
        if ccw <= std::f64::consts::PI { (angle(t0), ccw, [t0, t1]) }
        else { (angle(t1), std::f64::consts::TAU - ccw, [t1, t0]) }
    }
}

/// How the section is carried: along the edge's direction (`u × v`) through `length`, or turned
/// once about the line through the section's origin along its `v`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Carry { Prism { length: f64 }, Turn }

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
        let corner = self.section.lift(self.wedge.corner[0], self.wedge.corner[1]);
        match self.carry {
            Carry::Prism { length } => add(corner, scale(self.section.normal(), length / 2.0)),
            Carry::Turn => corner,
        }
    }

    /// The same piece with every length divided by `per` and its plane stood off by `origin`: a
    /// piece worked out in a boundary's millimetres, in model units about the world's origin.
    fn in_units(&self, per: f64, origin: V) -> Piece {
        let w = &self.wedge;
        let sc = |p: [f64; 2]| [p[0] / per, p[1] / per];
        Piece {
            section: Basis { o: std::array::from_fn(|k| self.section.o[k] / per + origin[k]), ..self.section },
            wedge: Wedge { corner: sc(w.corner), touch: w.touch.map(sc), centre: sc(w.centre), r: w.r / per },
            carry: match self.carry { Carry::Prism { length } => Carry::Prism { length: length / per }, c => c },
        }
    }
}

/// The names of a piece's section edges, in its loop's order — corner to first touch, the arc,
/// second touch to corner — and so of the faces its sweep makes: the arc's is the fillet's face.
pub const EDGE_NAMES: [&str; 3] = ["a", "round", "b"];

/// A fillet's every piece, and whether its edges are concave (the ball's material is added).
#[derive(Clone, Debug, PartialEq)]
pub struct Blend { pub pieces: Vec<Piece>, pub concave: bool }

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
    let mut concave: Option<(bool, String)> = None;
    for (ei, uses) in table.iter().enumerate() {
        let [(f0, c0), (f1, c1)] = match uses.as_slice() { [x, y] if x.0 != y.0 => [*x, *y], _ => continue };
        let (n0, n1) = (&brep.faces[f0].name, &brep.faces[f1].name);
        let (first, second) = if fa.contains(n0) && fb.contains(n1) { ((f0, c0), (f1, c1)) }
            else if fa.contains(n1) && fb.contains(n0) { ((f1, c1), (f0, c0)) }
            else { continue };
        let at = Edge { brep: &brep, located: &located, table: &table, edge: ei, uses: [first, second], tol };
        let Some((piece, hollow)) = at.round(r.value * mm)? else { continue };
        let label = format!("`{}` with `{}`", brep.faces[first.0].name, brep.faces[second.0].name);
        match &concave {
            Some((was, other)) if *was != hollow => return Err(format!(
                "rounds the concave edge of {} and the convex edge of {}: a `union` adds the one and a \
                 `cut` takes the other away, so write two fillets",
                if hollow { &label } else { other }, if hollow { other } else { &label })),
            Some(_) => {}
            None => concave = Some((hollow, label)),
        }
        pieces.push(piece.in_units(mm, origin));
    }
    let Some((concave, _)) = concave else {
        return Err(format!("`{}` and `{}` meet at no edge to round",
            side_name(sk, a), side_name(sk, b)));
    };
    Ok(Blend { pieces, concave })
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

/// Where the ball rolls on one face, in coordinates its edges are read in.
enum Band {
    /// A plane face beside a straight edge: a rectangle, along the edge from `corner` (`along`,
    /// through `length`) and across it into the face (`d`, through the setback).
    Strip { corner: V, along: V, d: V, length: f64 },
    /// A face turned about the axis through `o` along `axis`: the stretch of its meridian line
    /// from the corner along `dm` (radius, height), whatever turn it is at.
    Meridian { o: V, axis: V, corner: [f64; 2], dm: [f64; 2] },
}

impl Band {
    /// The stretch of the band's across-coordinate an edge reaches, or — a strip — whether it
    /// enters the band at all; `None` where there is no closed form for it.
    fn enters(&self, curve: &Curve, t: [f64; 2], setback: f64, tol: f64) -> Option<bool> {
        match *self {
            Band::Strip { corner, along, d, length } => {
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
            Band::Meridian { o, axis, corner, dm } => {
                let w = |rho: f64, z: f64| (rho - corner[0]) * dm[0] + (z - corner[1]) * dm[1];
                let mer = |q: V| { let z = dot(sub(q, o), axis); (norm(sub(sub(q, o), scale(axis, z))), z) };
                let (a, b) = match curve {
                    Curve::Circle(frame, rc)
                        if norm(cross(frame.z, axis)) <= 1e-9 && norm(cross(sub(frame.o, o), axis)) <= tol =>
                    {
                        let v = w(*rc, dot(sub(frame.o, o), axis));
                        (v, v)
                    }
                    // in a plane square to the axis, a plane face's band is radial
                    _ if dm[1].abs() <= 1e-12 => {
                        let (lo, hi) = radius_range(curve, t, o, axis)?;
                        (w(lo, corner[1]), w(hi, corner[1]))
                    }
                    // a line in a meridian: radius and height both affine along it
                    Curve::Line { p: q, d: dl } if dot(cross(*dl, axis), sub(*q, o)).abs() <= tol => {
                        let ((r0, z0), (r1, z1)) = (mer(curve.point(t[0])), mer(curve.point(t[1])));
                        (w(r0, z0), w(r1, z1))
                    }
                    _ => return None,
                };
                Some(a.max(b) > tol && a.min(b) < setback - tol)
            }
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

    /// Its piece (millimetres, about the boundary's origin) and whether it is concave; none where
    /// the two faces run on into one another (two operands' coplanar faces), with no edge to round.
    fn round(&self, r: f64) -> Result<Option<(Piece, bool)>, String> {
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
        let (section, corner, carry) = match curve {
            Curve::Line { d: along, .. }
                if matches!(surface(0), Surface::Plane(_)) && matches!(surface(1), Surface::Plane(_)) =>
            {
                let (u, start) = (d[0], curve.point(e.t[0]));
                (Basis { u, v: cross(*along, u), o: start }, [0.0, 0.0], Carry::Prism { length: e.t[1] - e.t[0] })
            }
            Curve::Circle(frame, rc) => {
                let axis = frame.z;
                if !(0..2).all(|k| coaxial(surface(k), frame.o, axis, self.tol)) {
                    return Err(self.unsupported("circle"));
                }
                if !e.closed() || (e.t[1] - e.t[0] - std::f64::consts::TAU).abs() > 1e-9 {
                    return Err(format!(
                        "`{}` meets `{}` on part of a circle: a ring is rounded only whole (rung 3)",
                        self.face_name(0), self.face_name(1)));
                }
                let u = unit(sub(p, add(frame.o, scale(axis, dot(sub(p, frame.o), axis)))));
                (Basis { u, v: axis, o: frame.o }, [*rc, 0.0], Carry::Turn)
            }
            other => return Err(self.unsupported(other.kind())),
        };
        let flat = |x: V| [dot(x, section.u), dot(x, section.v)];
        if d.iter().any(|x| dot(*x, section.normal()).abs() > 1e-9) {
            return Err(format!("`{}` and `{}` do not meet square to their section",
                self.face_name(0), self.face_name(1)));
        }
        let wedge = Wedge::new(corner, [flat(d[0]), flat(d[1])], r).ok_or_else(|| format!(
            "`{}` and `{}` meet too nearly flat or too sharply to round",
            self.face_name(0), self.face_name(1)))?;
        // the corner's material says the same as the normals: a ball in the wedge is in the
        // material at a convex edge and out of it at a concave one
        let toward = sub(section.lift(wedge.centre[0], wedge.centre[1]), section.lift(corner[0], corner[1]));
        let place = self.located.solid_place(add(p, scale(toward, 0.01)));
        if place != if concave { Place::Out } else { Place::In } {
            return Err(format!("cannot tell which side of `{}` with `{}` is material",
                self.face_name(0), self.face_name(1)));
        }
        if matches!(carry, Carry::Turn) && [wedge.touch[0][0], wedge.touch[1][0]].iter().any(|&x| x <= self.tol) {
            return Err(format!("the ball between `{}` and `{}` reaches past their axis",
                self.face_name(0), self.face_name(1)));
        }
        let setback = wedge.setback();
        for k in 0..2 {
            let band = match carry {
                Carry::Prism { length } => Band::Strip {
                    corner: section.lift(corner[0], corner[1]), along: section.normal(), d: d[k], length },
                Carry::Turn => Band::Meridian { o: section.o, axis: section.v, corner, dm: flat(d[k]) },
            };
            self.holds(k, &band, setback)?;
        }
        if let Carry::Prism { .. } = carry {
            for end in 0..2 { self.stops(end, d, setback, section.normal())?; }
        }
        Ok(Some((Piece { section, wedge, carry }, concave)))
    }

    fn unsupported(&self, curve: &str) -> String {
        let kind = |k: usize| self.brep.faces[self.uses[k].0].surface.kind();
        format!("`{}` ({}) meets `{}` ({}) on a {curve}: only a line between planes, or a circle \
                 between planes, cylinders and cones about its axis, is rounded yet (rung 2)",
            self.face_name(0), kind(0), self.face_name(1), kind(1))
    }

    /// **A ball no larger than face `k` can hold**: the band of the face within the setback of the
    /// edge, where the ball rolls, crossed by none of the face's other edges. Read in closed form
    /// for lines and circles; anything else is refused, never sampled.
    fn holds(&self, k: usize, band: &Band, setback: f64) -> Result<(), String> {
        let (fi, (li, ci)) = self.uses[k];
        let f = &self.brep.faces[fi];
        let seams = f.seams();
        for (lj, l) in f.loops.iter().enumerate() {
            for (cj, c) in l.iter().enumerate() {
                if (lj, cj) == (li, ci) || seams.contains(&c.edge) { continue; }
                let e = &self.brep.edges[c.edge as usize];
                let EdgeCurve::Curve(curve) = &e.curve else { continue };
                match band.enters(curve, e.t, setback, self.tol) {
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

    /// **A straight fillet stops flush at end `end`**: on each face the edge beside it at that end
    /// runs on across the band, a straight edge at least the setback long, into a plane square to
    /// the edge (along `along`) — so the ball's section stands in one plane there and the fillet
    /// ends in it. A face running on past the end needs the ball to turn the corner (rung 3).
    fn stops(&self, end: usize, d: [V; 2], setback: f64, along: V) -> Result<(), String> {
        let vertex = self.brep.edges[self.edge].v[end];
        let at = self.brep.vertices[vertex as usize].p;
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
            let EdgeCurve::Curve(Curve::Line { .. }) = &ne.curve else { return Err(runs_on()) };
            let other = if ne.v[0] == vertex { ne.v[1] } else { ne.v[0] };
            let dir = unit(sub(self.brep.vertices[other as usize].p, at));
            if dot(dir, d[k]) < 1.0 - 1e-9 || ne.t[1] - ne.t[0] < setback - self.tol { return Err(runs_on()); }
            // the face across that edge is the plane the fillet ends in
            let across = self.table[c.edge as usize].iter().map(|u| u.0).find(|&g| g != fi).ok_or_else(runs_on)?;
            let Surface::Plane(frame) = &self.brep.faces[across].surface else { return Err(runs_on()) };
            if dot(frame.z, along).abs() < 1.0 - 1e-9 { return Err(runs_on()); }
        }
        Ok(())
    }
}

/// Whether `s` is a surface of revolution about the line through `o` along the unit `axis` whose
/// meridian is a line: a plane square to it, or a cylinder or cone about it.
fn coaxial(s: &Surface, o: V, axis: V, tol: f64) -> bool {
    let parallel = |z: V| norm(cross(z, axis)) <= 1e-9;
    match s {
        Surface::Plane(f) => parallel(f.z),
        Surface::Cylinder(f, _) | Surface::Cone(f, _, _) => parallel(f.z) && norm(cross(sub(f.o, o), axis)) <= tol,
        _ => false,
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
        let w = &self.wedge;
        let (start, sweep, ends) = w.arc();
        let mut arc = super::profile::tessellate_arc((w.centre[0], w.centre[1]), w.r, start, sweep, unit);
        // walked from the first touch to the second
        if ends[0] != w.touch[0] { arc.reverse(); }
        let mut pts = vec![(w.corner[0], w.corner[1])];
        let mut of = vec![(0, false)];
        for p in &arc[..arc.len() - 1] {
            pts.push(*p);
            of.push((1, true));
        }
        pts.push((w.touch[1][0], w.touch[1][1]));
        of.push((2, false));
        let basis = Basis { o: std::array::from_fn(|k| self.section.o[k] - origin[k]), ..self.section };
        let poly = FacePoly { pts, of, names: EDGE_NAMES.map(String::from).to_vec(), basis,
            curved: vec![None; EDGE_NAMES.len()] };
        poly.valid().then(|| poly.ccw())
    }
}
