//! **A body grown, built exactly** (`std.rounds.Grown`): a region solid read in general position
//! whose one term bounds the distance from a body's material, `{ p | distance(p, within) <= d }`
//! with `within := { c | c inside B }`, is every point within `d` of `B` — `B` and the points
//! nearer than `d` outside it.  Outside, a point's nearest point of `B` is inside a face, inside an
//! edge or a corner, so what it adds is a partition into those points: each face's prism `d` out
//! along its normal (a cylinder's band, its radius grown or a bore's shrunk by `d`), each convex
//! edge's wedge of radius `d` between its two faces' normals (a torus's where the edge is a
//! circle), and each convex corner's sector of the ball where three faces meet.  A concave edge has
//! no wedge: its two faces' prisms overlap there, and each is cut back to the edge's bisector (the
//! plane through it halfway between them), where they meet face to face.  At a corner where one
//! concave edge meets a face square to both its faces (a step's end, a pocket's rim) the ball has
//! no sector, and the two wedges along that face are cut back to the same bisector, meeting in it;
//! where three concave edges meet (a pocket's floor) the three prisms meet in their bisectors.
//! At a foot the bisector is a cone: where a convex edge stands square to a face between two
//! concave ones (a boss's foot) the edge's wedge is turned about it from a section cut back along
//! the cone `z = r` about it, and where a plane meets a cylinder square to it in a concave circle (a
//! post's foot, a bore's floor) the band is cut back along the cone `z = |ρ − R|` about its axis; the
//! face beneath keeps the rest, its prism with the loop moved `d` across it and the pieces between
//! glued to it, each its own sweep (`about_feet`).
//! Each piece shares its faces exactly with the next — a prism its foot with the face, its sides
//! with the wedges or the prism across a bisector, a wedge its ends with the sectors or the wedge
//! across a bisector — so they are glued to `B` there and nothing is intersected
//! (`boolean::glued`): the boundary of `B` grown, planes, cylinders, cones, tori and spheres meeting
//! tangent, the cuts transversal.  Held to the region read point by point before it stands
//! (`gate`), as the other exact constructions are.  A concave edge on a curved face other than a
//! band's foot, a boss's foot whose edge stands not square to the face beneath it, a face beside a
//! concave edge that is no convex polygon or is narrower than the reach (a pocket narrower than
//! twice it), a face no plane or cylinder carries, an edge no line or circle, a convex corner that
//! is no prism's (three faces, one square to the other two), a bore no wider than the reach, or a
//! body whose faces another solid reads leave it the field's, said by name.  The recipe's `grown`
//! node: the body's own node, and its pieces each a `prism` or a `revolve` — one cut back a
//! `region`, in common with a block standing on each bisector; a face's pieces about a foot
//! `glued`, their feet made one face.
use crate::brep::build::{Profile, ProfileEdge};
use crate::brep::geom::{Curve, Frame, Surface};
use crate::brep::recipe::Writer;
use crate::brep::topo::EdgeCurve;
use crate::json::{object, Json};
use crate::model::{FaceRef, RegionShape, RegionTerm, Sketch, SolidDef, Target};
use crate::space::{add, cross, dot, norm, scale, sub, unit};
use crate::vocab::Cmp;
use super::faces::BodyFaces;
use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::TAU;
use std::rc::Rc;

#[allow(unused_imports)]
use crate::fmath::Det;

type V = [f64; 3];

/// A body grown: the body's solid, and what is glued to it (model units).
#[derive(Clone, Debug)]
pub struct Grown {
    body: u32,
    /// Each glued to the body as one: a piece, or a face's pieces about a foot glued to one
    /// another first.
    pieces: Vec<Vec<Piece>>,
    /// What the pieces' faces are called: the term's set.
    name: String,
    /// How far the block a piece is cut back by reaches past its plane, and either way along it:
    /// past every piece.
    reach: f64,
}

/// One piece: a section, how it is swept, and the planes it is cut back to where it meets a
/// concave edge's bisector.
#[derive(Clone, Debug)]
struct Piece {
    section: Profile,
    sweep: Sweep,
    cuts: Vec<Cut>,
}

impl Piece {
    /// A piece cut back nowhere.
    fn new(section: Profile, sweep: Sweep) -> Piece { Piece { section, sweep, cuts: Vec::new() } }
}

/// A plane a piece is cut back to: through `at`, kept on the side the unit `keep` points to.
#[derive(Clone, Copy, Debug)]
struct Cut {
    at: V,
    keep: V,
}

/// The bisector through `at` of a concave edge between the face whose outward unit normal is `own`
/// and the one beyond it, `beyond`, kept on `own`'s side; and how far along the unit `toward`
/// (square to the edge, into `own`'s side of it) it meets the plane `d` out from `own`: `d (1 −
/// cos)` over `beyond` along `toward`.  `None` where `toward` runs along the bisector.
fn bisector(at: V, own: V, beyond: V, toward: V, d: f64) -> Option<(Cut, f64)> {
    let across = dot(beyond, toward);
    (across > 1e-9).then(|| (Cut { at, keep: unit(sub(beyond, own)) }, d * (1.0 - dot(own, beyond)) / across))
}

#[derive(Clone, Copy, Debug)]
enum Sweep {
    /// Along the section's normal, from its plane this far.
    Along(f64),
    /// About the line through `origin` along the unit `axis`, right-handed by `angle`.
    About { origin: V, axis: V, angle: f64 },
}

impl Grown {
    /// The recipe's node, written by `w`: `grown`, the body's node and its `pieces` each a `prism`
    /// or a `revolve`, or one cut back a `region` keeping it and a block on each of its planes, or
    /// a face's pieces about a foot `glued` to one another.
    pub fn node(&self, w: &Writer) -> Result<Json, String> {
        let pieces = self.pieces.iter().map(|group| match &group[..] {
            [piece] => self.piece(piece, w),
            group => {
                let parts = group.iter().map(|p| self.piece(p, w)).collect::<Result<Vec<_>, String>>()?;
                Ok(object([("kind", "glued".into()), ("pieces", Json::Arr(parts))]))
            }
        }).collect::<Result<Vec<_>, String>>()?;
        Ok(object([("kind", "grown".into()), ("body", (self.body as usize).into()), ("pieces", Json::Arr(pieces))]))
    }

    /// One piece's node: its sweep, or that cut back to each of its planes.
    fn piece(&self, piece: &Piece, w: &Writer) -> Result<Json, String> {
        let s = w.scale;
        let names = piece.section.loops.iter().map(|l| vec![self.name.clone(); l.len()]).collect();
        let profile = Profile { names, ..piece.section.clone() }.to_json(w)?;
        let mut swept = match piece.sweep {
            Sweep::Along(to) => object([("kind", "prism".into()), ("profile", profile),
                ("from", 0.0.into()), ("to", (to * s).into())]),
            Sweep::About { origin, axis, angle } => object([("kind", "revolve".into()),
                ("profile", profile), ("origin", w.line_origin(origin.map(|x| x * s), axis)),
                ("axis", Writer::triple(axis)), ("angle", angle.into())]),
        };
        if piece.cuts.is_empty() {
            return Ok(swept);
        }
        // cut back to each bisector: in common with a block standing on its plane
        swept.set("keep", Json::Bool(true));
        let mut kept = vec![swept];
        for cut in &piece.cuts {
            let mut block = super::bounded::half_space(w, cut.at, cut.keep, self.reach, &self.name)?;
            block.set("keep", Json::Bool(true));
            kept.push(block);
        }
        Ok(object([("kind", "region".into()), ("pieces", Json::Arr(kept))]))
    }

    /// The solid it is glued to, whose recipe node its own reads.
    pub fn body(&self) -> u32 { self.body }
}

/// The body region solid `solid` is within reach of, and the reach, where it is that region: one
/// term, the distance from a set whose one term is the inside of a body's faces, at most a stated
/// number with no offset.
fn recognised(sk: &Sketch, solid: usize) -> Option<(&FaceRef, &RegionTerm)> {
    let SolidDef::Region { terms, .. } = &sk.solids.get(solid)?.def else { return None };
    let [t] = &terms[..] else { return None };
    let RegionShape::Measured { of } = &t.shape else { return None };
    let [(Target::Set { terms: inner, .. }, c)] = &of[..] else { return None };
    if *c != 1.0 || t.cmp != Cmp::Le || t.hi.is_some() || t.grow.is_some() {
        return None;
    }
    let [s] = &inner[..] else { return None };
    let RegionShape::FaceSide { face } = &s.shape else { return None };
    let at_nothing = s.lo.free.is_none() && s.lo.c == 0.0;
    (s.cmp == Cmp::Le && s.hi.is_none() && s.grow.is_none() && at_nothing && !face.surface && !face.smooth)
        .then_some((face, t))
}

/// Region solid `solid` built as its body and what lies within reach of each face, edge and corner
/// glued to it, where it is that region (`None` otherwise): the construction, or why it stands not.
/// Remembered against `solid::reads` (`Caches::grown`).
pub fn built(sk: &Sketch, solid: usize) -> Option<Result<Rc<Grown>, String>> {
    let (face, t) = recognised(sk, solid)?;
    sk.caches.grown.get_or(solid, super::MemoKey::of(sk, solid), || {
        Some(construction(sk, face, t).and_then(|(g, body)| {
            gate(sk, solid, &g, &body)?;
            Ok(Rc::new(g))
        }))
    })
}

/// The pieces of the region within `t`'s reach of the body `face` names, and the faces of the body
/// they are glued to.
fn construction(sk: &Sketch, face: &FaceRef, t: &RegionTerm)
    -> Result<(Grown, std::sync::Arc<BodyFaces>), String> {
    let d = t.lo.read(sk).ok_or("its reach reads an unknown nothing determines")?;
    if !(d > 0.0) {
        return Err("a body grown by nothing is the body itself".to_string());
    }
    // the faces its glued pieces meet are the body's own, as its recipe node builds it
    if !super::faces::stratum_is_whole(sk, face.solid as usize) {
        return Err("a body grown whose faces a term of it reads (a round) is read by the field".to_string());
    }
    let mut read = Vec::new();
    let reading = super::faces::reading(sk, face, &mut read)?;
    if reading.faces.len() != reading.body.brep().0.faces.len() {
        return Err("within reach of only some of a body's faces is read by the field".to_string());
    }
    let body = reading.body.clone();
    Ok((pieces(&body, face.solid, d, &t.name)?, body))
}

/// Solid `solid` grown by `d` (model units), its pieces' faces called `name`: the construction a
/// body grown is (`built`), for a solid no region names — a shell's cavity less each cut from the
/// body it hollows, grown by the thickness (`solid::shelled`).  Refused as `built` refuses it.
pub(crate) fn around(sk: &Sketch, solid: usize, d: f64, name: &str) -> Result<Grown, String> {
    if !super::faces::stratum_is_whole(sk, solid) {
        return Err("a body grown whose faces a term of it reads (a round) is read by the field".to_string());
    }
    let body = super::faces::cached(sk, solid)?;
    pieces(&body, solid as u32, d, name)
}

/// What lies within `d` of solid `solid`, whose faces are `body`, glued to it: its pieces.
fn pieces(body: &BodyFaces, solid: u32, d: f64, name: &str) -> Result<Grown, String> {
    let (b, mm, _) = body.brep();
    let refused = |why: &str| format!("{why}: read by the field");
    // the concave edges: two planes meeting along a line, each one's prism cut back to their
    // bisector; or a band's foot, a plane square to a cylinder about its axis, their bisector a cone
    let mut concave = vec![false; b.edges.len()];
    let mut at_vertex = vec![Vec::new(); b.vertices.len()];
    for e in 0..b.edges.len() as u32 {
        let &[fa, fb] = body.faces_of(e) else { continue };
        let EdgeCurve::Curve(curve) = &b.edges[e as usize].curve else { continue };
        for v in body.ends(e) {
            at_vertex[v as usize].push(e);
        }
        if body.convex(e) == Some(false) {
            let planes = body.plane(fa).is_some() && body.plane(fb).is_some();
            if !(planes && matches!(curve, Curve::Line { .. })) && !foot(&body, e) {
                return Err(refused("a concave edge on a curved face, no whole circle where a plane meets a \
                                    cylinder square to it"));
            }
            concave[e as usize] = true;
        }
    }
    let mut pieces: Vec<Vec<Piece>> = Vec::new();
    // each corner's sector of the ball: where three faces meet, one square to the other two (a
    // prism's corner), the quarter disc between the square one and another turned about it.  At
    // a concave corner the ball has no sector: where one concave edge meets the square face, the
    // wedges along that face are cut back to its bisector (`trims`, by edge and end: the corner,
    // what is kept, how far along the edge the cut reaches); where three meet, nothing; where a
    // convex edge stands square to the face two concave ones bound (a boss's foot), its wedge is
    // cut back to the cone about it at that end (`feet`, by edge and end) and the face's prism
    // takes the rest (`about_feet`, its corners `standing`)
    let mut sectors = Vec::new();
    let mut trims = BTreeMap::<u32, [Option<(Cut, f64)>; 2]>::new();
    let mut feet = BTreeMap::<u32, [bool; 2]>::new();
    let mut standing = BTreeSet::<u32>::new();
    for v in 0..b.vertices.len() as u32 {
        let c = body.back(b.vertices[v as usize].p);
        let edges = &at_vertex[v as usize];
        let mut normals: Vec<V> = Vec::new();
        for fi in body.faces_at(v, &[]) {
            let n = body.normal(fi, c);
            if !normals.iter().any(|&m| norm(sub(m, n)) < 1e-9) {
                normals.push(n);
            }
        }
        // no corner: a vertex along an edge, or where a band's seam meets its rims
        if normals.len() < 3 {
            continue;
        }
        let hollow: Vec<u32> = edges.iter().copied().filter(|&e| concave[e as usize]).collect();
        if let Some(&e) = hollow.first() {
            let faces = body.faces_at(v, &[]);
            if faces.len() != 3 || edges.len() != 3 {
                return Err(refused("a concave corner of more than three faces"));
            }
            if hollow.iter().any(|&x| body.segment(x).is_none()) {
                return Err(refused("a concave corner on a curved face"));
            }
            match hollow.len() {
                1 => {
                    let &[f, g] = body.faces_of(e) else { unreachable!("a concave edge between two faces") };
                    let h = faces.iter().copied().find(|&x| x != f && x != g).expect("a third face");
                    let [nf, ng] = [f, g].map(|x| body.plane(x).expect("a concave edge's plane").0);
                    let Some((nh, _)) = body.plane(h) else {
                        return Err(refused("a concave corner on a curved face"));
                    };
                    if dot(nh, nf).abs() > 1e-9 || dot(nh, ng).abs() > 1e-9 {
                        return Err(refused("a corner that is no prism's: three faces, one square to the \
                                            other two"));
                    }
                    for &x in edges.iter().filter(|&&x| x != e) {
                        // the wedge along the face `h` shares with `f` keeps `f`'s side of the bisector
                        let (ns, no) = if body.faces_of(x).contains(&f) { (nf, ng) } else { (ng, nf) };
                        let ends = body.ends(x);
                        let end = usize::from(ends[0] != v);
                        let along = unit(sub(body.back(b.vertices[ends[1 - end] as usize].p), c));
                        let trim = bisector(c, ns, no, along, d)
                            .ok_or_else(|| refused("a concave corner whose wedges run along its bisector"))?;
                        trims.entry(x).or_default()[end] = Some(trim);
                    }
                }
                2 => {
                    // a boss's foot: the convex edge standing square to the face both concave ones
                    // bound, so the walls either side of it square to that face too
                    let x = *edges.iter().find(|&&x| !concave[x as usize]).expect("a third edge");
                    let under = body.faces_of(hollow[0]).iter().copied().find(|g| body.faces_of(hollow[1]).contains(g));
                    let square = under.and_then(|g| body.plane(g)).zip(body.segment(x))
                        .is_some_and(|((n, _), [p, q])| norm(cross(unit(sub(q, p)), n)) <= 1e-9);
                    if body.convex(x) != Some(true) || !square {
                        return Err(refused("a corner where a convex edge stands between two concave ones, not \
                                            square to the face they bound"));
                    }
                    feet.entry(x).or_default()[usize::from(body.ends(x)[0] != v)] = true;
                    standing.insert(v);
                }
                _ => {}
            }
            continue;
        }
        let corner = (normals.len() == 3).then(|| (0..3).find(|&k| {
            (0..3).filter(|&j| j != k).all(|j| dot(normals[k], normals[j]).abs() < 1e-9)
        })).flatten().ok_or_else(|| refused("a corner that is no prism's: three faces, one square to the other two"))?;
        let nc = normals[corner];
        let [na, nb] = [normals[(corner + 1) % 3], normals[(corner + 2) % 3]];
        let section = wedge(c, cross(na, nc), na, nc, d);
        let angle = dot(cross(na, nb), nc).atan2(dot(na, nb));
        sectors.push(Piece::new(section, Sweep::About { origin: c, axis: nc, angle }));
    }
    // each face's prism (a cylinder's band) out along its normal, a face beside a concave edge cut
    // back to its bisector
    for fi in 0..b.faces.len() {
        let f = &b.faces[fi];
        pieces.push(match f.surface {
            Surface::Plane(_) => {
                let (n, o) = body.plane(fi).expect("a plane face");
                let hollow: Vec<u32> = body.edges_of(fi).iter().copied().filter(|&e| concave[e as usize]).collect();
                if hollow.is_empty() {
                    let section = section_of(&body, fi, n, o)
                        .ok_or_else(|| refused("a plane face bounded by an edge no line or circle carries"))?;
                    vec![Piece::new(section, Sweep::Along(d))]
                } else if f.loops.len() == 1 && hollow.iter().all(|&e| body.segment(e).is_some()) {
                    vec![beside_concave(&body, fi, n, &concave, d).map_err(refused)?]
                } else {
                    about_feet(&body, fi, n, o, &concave, &standing, d).map_err(refused)?
                }
            }
            Surface::Cylinder(frame, r) => vec![band(&body, fi, frame, r, mm, &concave, d)
                .ok_or_else(|| refused("a cylinder's face that is no band between two circles about its axis, \
                                        a bore no wider than the reach, or a band shorter than it between its \
                                        feet"))?],
            _ => return Err(refused(&format!("a face on a {}", f.surface.kind()))),
        });
    }
    // each convex edge's wedge between its faces' normals: along a line, or turned about a circle
    for e in 0..b.edges.len() as u32 {
        let &[fa, fb] = body.faces_of(e) else { continue };
        let edge = &b.edges[e as usize];
        let EdgeCurve::Curve(curve) = &edge.curve else { continue };
        // the faces go on tangent, or meet concave: their prisms meet there
        if body.convex(e) != Some(true) {
            continue;
        }
        let [t0, t1] = edge.t;
        let at = body.along(e, 0.0);
        let (na, nb) = (body.normal(fa, at), body.normal(fb, at));
        let tangent = unit(curve.tangent(t0));
        let trim = trims.get(&e).copied().unwrap_or_default();
        let foot = feet.get(&e).copied().unwrap_or_default();
        pieces.push(vec![match curve {
            Curve::Line { .. } => {
                // the faces' normals the same all along it
                let end = body.along(e, 1.0);
                if norm(sub(body.normal(fa, end), na)) > 1e-9 || norm(sub(body.normal(fb, end), nb)) > 1e-9 {
                    return Err(refused("an edge along which a face turns"));
                }
                let length = (t1 - t0) / mm;
                let footing = foot.iter().filter(|&&f| f).count() as f64 * d;
                if trim.iter().flatten().map(|t| t.1).sum::<f64>() + footing >= length * (1.0 - 1e-9) {
                    return Err(refused("an edge shorter than the reach between its concave corners"));
                }
                // cut back at a concave corner: begun past it, so the bisector crosses it clear
                let [before, after] = trim.map(|t| if t.is_some() { d } else { 0.0 });
                let cuts = trim.iter().flatten().map(|t| t.0).collect();
                if foot.iter().any(|&f| f) {
                    // standing at a boss's foot: turned about the edge from one face's normal to
                    // the other's, its section in the half-plane through it toward the first cut
                    // back along the cone about it at each foot (`r = z`, the height over the face)
                    let point = |r: f64, z: f64| add(at, add(scale(na, r), scale(tangent, z)));
                    let mut corners = if foot[0] { vec![point(0.0, 0.0), point(d, d)] }
                        else { vec![point(0.0, -before), point(d, -before)] };
                    corners.extend(if foot[1] { [point(d, length - d), point(0.0, length)] }
                        else { [point(d, length + after), point(0.0, length + after)] });
                    let edges = (0..4).map(|k| ProfileEdge::Line { a: corners[k], b: corners[(k + 1) % 4] }).collect();
                    let angle = dot(cross(na, nb), tangent).atan2(dot(na, nb));
                    Piece { section: Profile { origin: at, normal: cross(tangent, na), loops: vec![edges], names: Vec::new() },
                            sweep: Sweep::About { origin: at, axis: tangent, angle }, cuts }
                } else {
                    Piece { section: wedge(sub(at, scale(tangent, before)), tangent, na, nb, d),
                            sweep: Sweep::Along(length + before + after), cuts }
                }
            }
            Curve::Circle(..) => {
                let (centre, axis, r, _) = body.circle(e).expect("a circle");
                if dot(na, tangent).abs() > 1e-9 || dot(nb, tangent).abs() > 1e-9 {
                    return Err(refused("a circle whose faces turn about no axis of its own"));
                }
                // turned about its axis, the wedge stays clear of it
                if !(r > d) {
                    return Err(refused("a circle no larger than the reach"));
                }
                Piece::new(wedge(at, tangent, na, nb, d),
                           Sweep::About { origin: centre, axis, angle: (t1 - t0).min(TAU) })
            }
            _ => return Err(refused("an edge no line or circle carries")),
        }]);
    }
    pieces.extend(sectors.into_iter().map(|s| vec![s]));
    let (lo, hi) = body.bounds();
    let reach = 2.0 * (norm(sub(hi, lo)) + 4.0 * d);
    Ok(Grown { body: solid, pieces, name: name.to_string(), reach })
}

/// Plane face `fi` (its outward normal `n`) beside a concave edge, where it is a convex polygon:
/// its prism `d` out along `n`, cut back to each concave side's bisector (`bisector`).  The
/// section's concave sides are pushed out past the face so that each bisector crosses the prism
/// clear of its edges, along the face's own side at its foot; at its top, where every side's line
/// moved in by its cut still bounds a polygon of as many sides, the face is no narrower than the
/// reach.
fn beside_concave(body: &BodyFaces, fi: usize, n: V, concave: &[bool], d: f64)
    -> Result<Piece, &'static str> {
    const NO_POLYGON: &str = "a face beside a concave edge that is no convex polygon";
    /// A side of the face: its start, its unit direction, and the normal of the face beyond it
    /// where it is concave.
    struct Side { start: V, along: V, beyond: Option<V> }
    let (b, _, _) = body.brep();
    let [l] = &b.faces[fi].loops[..] else { return Err(NO_POLYGON) };
    let mut sides = Vec::new();
    for c in l {
        let [p, q] = body.segment(c.edge).ok_or(NO_POLYGON)?;
        let (start, end) = if c.reversed { (q, p) } else { (p, q) };
        let beyond = body.faces_of(c.edge).iter().copied().find(|&g| g != fi)
            .filter(|_| concave[c.edge as usize]).map(|g| body.normal(g, start));
        sides.push(Side { start, along: unit(sub(end, start)), beyond });
    }
    let m = sides.len();
    if m < 3 {
        return Err(NO_POLYGON);
    }
    // turning one way at every corner
    let turns: Vec<f64> = (0..m).map(|k| dot(cross(sides[k].along, sides[(k + 1) % m].along), n)).collect();
    let sign = turns[0].signum();
    if turns.iter().any(|&t| !(t * sign > 1e-9)) {
        return Err(NO_POLYGON);
    }
    let inward = |k: usize| scale(cross(n, sides[k].along), sign);
    let size = (0..m).map(|k| norm(sub(sides[(k + 1) % m].start, sides[k].start))).fold(0.0, f64::max);
    // the corners where each side's line moved in by `by(k)` meets the next, where every side
    // still runs its own way
    let corners = |by: &dyn Fn(usize) -> f64| -> Option<Vec<V>> {
        let line = |k: usize| (add(sides[k].start, scale(inward(k), by(k))), sides[k].along);
        let at: Vec<V> = (0..m).map(|k| {
            let ((p, u), (q, w)) = (line((k + m - 1) % m), line(k));
            add(p, scale(u, dot(cross(sub(q, p), w), n) / dot(cross(u, w), n)))
        }).collect();
        (0..m).all(|k| dot(sub(at[(k + 1) % m], at[k]), sides[k].along) > 1e-9 * size).then_some(at)
    };
    // each concave side's bisector, and how far in it meets the prism's top
    let cut = (0..m).map(|k| match sides[k].beyond {
        None => Ok(None),
        Some(beyond) => bisector(sides[k].start, n, beyond, inward(k), d).map(Some).ok_or(NO_POLYGON),
    }).collect::<Result<Vec<_>, _>>()?;
    corners(&|k| cut[k].map_or(0.0, |c| c.1)).ok_or("a face narrower than the reach beside a concave edge")?;
    // pushed out half the reach, or less where a side would not stand
    let foot = std::iter::successors(Some(0.5 * d), |push| Some(0.5 * push)).take_while(|&push| push >= 1e-6 * d)
        .find_map(|push| corners(&|k| if cut[k].is_some() { -push } else { 0.0 })).ok_or(NO_POLYGON)?;
    let edges = (0..m).map(|k| ProfileEdge::Line { a: foot[k], b: foot[(k + 1) % m] }).collect();
    let section = Profile { origin: foot[0], normal: n, loops: vec![edges], names: Vec::new() };
    Ok(Piece { section, sweep: Sweep::Along(d), cuts: cut.into_iter().flatten().map(|c| c.0).collect() })
}

/// The sector of the disc of radius `d` about `o` from the unit `na` to the unit `nb` (both square
/// to the unit `along`, the section's normal, less than a half turn apart).
fn wedge(o: V, along: V, na: V, nb: V, d: f64) -> Profile {
    let along = scale(along, 1.0 / norm(along));
    let f = Frame::new(o, along, na);
    let phi = dot(nb, f.y).atan2(dot(nb, f.x));
    let span = if phi > 0.0 { [0.0, phi] } else { [phi, 0.0] };
    let edges = vec![
        ProfileEdge::Line { a: o, b: add(o, scale(na, d)) },
        ProfileEdge::Arc { frame: f, r: d, span: Some(span) },
        ProfileEdge::Line { a: add(o, scale(nb, d)), b: o },
    ];
    Profile { origin: o, normal: along, loops: vec![edges], names: Vec::new() }
}

/// Plane face `fi`'s loops (lines and circles) as a section in its plane, through `o` (model units)
/// square to its outward normal `n`; `None` where an edge is no line or circle.
fn section_of(body: &BodyFaces, fi: usize, n: V, o: V) -> Option<Profile> {
    let (b, _, _) = body.brep();
    let loops = b.faces[fi].loops.iter().map(|l| loop_section(body, l, n)).collect::<Option<Vec<_>>>()?;
    Some(Profile { origin: o, normal: n, loops, names: Vec::new() })
}

/// A plane face's loop `l` (lines and circles) as a section's edges in its plane, square to its
/// outward normal `n` (model units); `None` where an edge is no line or circle.
fn loop_section(body: &BodyFaces, l: &[crate::brep::topo::Coedge], n: V) -> Option<Vec<ProfileEdge>> {
    let (b, mm, _) = body.brep();
    let back = |q: V| body.back(q);
    let mut out = Vec::new();
    for c in l {
        let edge = &b.edges[c.edge as usize];
        let EdgeCurve::Curve(curve) = &edge.curve else { continue };
        let [t0, t1] = edge.t;
        out.push(match curve {
            Curve::Line { .. } => ProfileEdge::Line { a: back(curve.point(t0)), b: back(curve.point(t1)) },
            Curve::Circle(f, r) => {
                // an arc turning counter-clockwise about the section's normal
                let full = edge.closed() && t1 - t0 >= TAU - 1e-9;
                let (frame, span) = if dot(f.z, n) > 0.0 {
                    (Frame { o: back(f.o), ..*f }, [t0, t1])
                } else {
                    (Frame { o: back(f.o), x: f.x, y: scale(f.y, -1.0), z: scale(f.z, -1.0) }, [-t1, -t0])
                };
                ProfileEdge::Arc { frame, r: r / mm, span: (!full).then_some(span) }
            }
            _ => return None,
        });
    }
    Some(out)
}

/// Whether concave edge `e` is a band's foot: a whole circle where a plane meets a cylinder about
/// its axis, so square to it, their bisector the cone about that axis through the circle at half a
/// right angle.
fn foot(body: &BodyFaces, e: u32) -> bool {
    let &[fa, fb] = body.faces_of(e) else { return false };
    let Some((centre, normal, r, _)) = body.circle(e) else { return false };
    let c = if body.plane(fa).is_some() { fb } else if body.plane(fb).is_some() { fa } else { return false };
    let Some((o, axis, rc)) = body.cylinder(c) else { return false };
    let tol = 1e-9 * (r + norm(centre)).max(1.0);
    body.closed(e) && norm(cross(axis, normal)) <= 1e-9 && norm(cross(sub(centre, o), axis)) <= tol
        && (r - rc).abs() <= tol
}

/// Plane face `fi` (its outward normal `n`, through `o`) about a foot: a hole whose edges are all
/// concave — a band's foot (`about_circle`) or a boss's, a convex polygon each corner of which a
/// convex edge stands on (`about_polygon`) — or a rim that is one concave circle (a bore's floor).
/// Its prism `d` out along `n`, each such loop moved `d` across the face, and glued to it the
/// pieces between the loop and where it moved, each cut back along the plane or the cone the
/// face's prism meets the piece beyond in: as one, the face's own (`glued`), its foot one face.
/// A loop no edge of which is concave stands as it is.  Refused where a loop is both, where the
/// rim is concave about a hole, or where the pieces about one loop reach another's.
fn about_feet(body: &BodyFaces, fi: usize, n: V, o: V, concave: &[bool], standing: &BTreeSet<u32>, d: f64)
    -> Result<Vec<Piece>, &'static str> {
    let (b, _, _) = body.brep();
    let face = &b.faces[fi];
    let mut loops = Vec::new();
    let mut rest = Vec::new();
    // each loop as segments, and how far its pieces reach across the face from it
    let mut zones: Vec<(Vec<[V; 2]>, f64)> = Vec::new();
    for (li, l) in face.loops.iter().enumerate() {
        let hollow = l.iter().filter(|c| concave[c.edge as usize]).count();
        let (edges, reach) = if hollow == 0 {
            (loop_section(body, l, n).ok_or("a plane face bounded by an edge no line or circle carries")?, 0.0)
        } else if hollow < l.len() {
            return Err("a face beside a concave edge that is no convex polygon");
        } else if let [c] = &l[..] {
            let (edge, piece) = about_circle(body, c.edge, n, li > 0, d)?;
            rest.push(piece);
            (vec![edge], d)
        } else if li > 0 {
            let (edges, more) = about_polygon(body, fi, l, n, standing, d)?;
            rest.extend(more);
            (edges, d)
        } else {
            return Err("a face with a hole beside a concave edge round its rim");
        };
        zones.push((polyline(body, l), reach));
        loops.push(edges);
    }
    for i in 0..zones.len() {
        for j in 0..i {
            if apart(&zones[i].0, &zones[j].0) <= (zones[i].1 + zones[j].1) * (1.0 + 1e-9) {
                return Err("a face narrower than the reach beside a concave edge");
            }
        }
    }
    let section = Profile { origin: o, normal: n, loops, names: Vec::new() };
    Ok(std::iter::once(Piece::new(section, Sweep::Along(d))).chain(rest).collect())
}

/// A band's foot `e`, a whole circle of the face square to the unit `n` (a `hole` in it, or its
/// rim): the circle moved `d` across the face, and the ring between them out to the cone the
/// face's prism and the band's meet in (`z = |ρ − r|`, its apex on the axis `r` beyond the face),
/// turned once from where the circle starts, as the band is.
fn about_circle(body: &BodyFaces, e: u32, n: V, hole: bool, d: f64) -> Result<(ProfileEdge, Piece), &'static str> {
    let (centre, _, r, _) = body.circle(e).ok_or("a face beside a concave edge that is no convex polygon")?;
    let x = unit(sub(body.along(e, 0.0), centre));
    let moved = if hole { r + d } else { r - d };
    if !(moved > 1e-9 * r) {
        return Err("a bore no wider than the reach");
    }
    let at = |rho: f64, z: f64| add(centre, add(scale(x, rho), scale(n, z)));
    let corners = [at(r, 0.0), at(moved, 0.0), at(moved, d)];
    let ring = (0..3).map(|k| ProfileEdge::Line { a: corners[k], b: corners[(k + 1) % 3] }).collect();
    let section = Profile { origin: centre, normal: cross(n, x), loops: vec![ring], names: Vec::new() };
    Ok((ProfileEdge::Arc { frame: Frame::new(centre, n, x), r: moved, span: None },
        Piece::new(section, Sweep::About { origin: centre, axis: n, angle: TAU })))
}

/// A boss's foot: hole `l` of face `fi` (its outward normal `n`), a convex polygon of concave
/// edges, each corner one a convex edge stands on square to the face (`standing`) and each wall
/// beyond square to it.  Its sides moved `d` across the face, an arc of radius `d` about each
/// corner between them; and the pieces between: along each side the triangle under its bisector
/// (`z = h`, `h` the distance from the wall) swept along it, at each corner the same triangle
/// turned about the edge standing there from one wall's normal to the next — under the cone `z =
/// r` that edge's wedge is cut back to.
fn about_polygon(body: &BodyFaces, fi: usize, l: &[crate::brep::topo::Coedge], n: V, standing: &BTreeSet<u32>,
    d: f64) -> Result<(Vec<ProfileEdge>, Vec<Piece>), &'static str> {
    const NO_POLYGON: &str = "a face about a boss's foot that is no convex polygon";
    let m = l.len();
    let (mut corner, mut away) = (Vec::new(), Vec::new());
    for c in l {
        let [p, q] = body.segment(c.edge).ok_or(NO_POLYGON)?;
        if !standing.contains(&body.ends(c.edge)[usize::from(c.reversed)]) {
            return Err(NO_POLYGON);
        }
        let start = if c.reversed { q } else { p };
        let wall = body.faces_of(c.edge).iter().copied().find(|&g| g != fi).ok_or(NO_POLYGON)?;
        let out = body.normal(wall, start);
        if dot(out, n).abs() > 1e-9 {
            return Err("a wall standing not square to the face beneath it");
        }
        corner.push(start);
        away.push(out);
    }
    if m < 3 {
        return Err(NO_POLYGON);
    }
    // turning one way at every corner, each wall facing away from the hole
    let along: Vec<V> = (0..m).map(|k| unit(sub(corner[(k + 1) % m], corner[k]))).collect();
    let turns: Vec<f64> = (0..m).map(|k| dot(cross(along[k], along[(k + 1) % m]), n)).collect();
    let middle = scale(corner.iter().fold([0.0; 3], |s, &p| add(s, p)), 1.0 / m as f64);
    let size = (0..m).map(|k| norm(sub(corner[k], middle))).fold(0.0, f64::max);
    if turns.iter().any(|&t| !(t * turns[0] > 1e-9))
        || (0..m).any(|k| !(dot(away[k], sub(corner[k], middle)) > 1e-9 * size)) {
        return Err(NO_POLYGON);
    }
    let (mut edges, mut pieces) = (Vec::new(), Vec::new());
    let triangle = |c: V, out: V| {
        let p = [c, add(c, scale(out, d)), add(add(c, scale(out, d)), scale(n, d))];
        (0..3).map(|k| ProfileEdge::Line { a: p[k], b: p[(k + 1) % 3] }).collect::<Vec<_>>()
    };
    for k in 0..m {
        let (c, before, after) = (corner[k], away[(k + m - 1) % m], away[k]);
        let f = Frame::new(c, n, before);
        let phi = dot(after, f.y).atan2(dot(after, f.x));
        edges.push(ProfileEdge::Arc { frame: f, r: d, span: Some(if phi > 0.0 { [0.0, phi] } else { [phi, 0.0] }) });
        let next = corner[(k + 1) % m];
        edges.push(ProfileEdge::Line { a: add(c, scale(after, d)), b: add(next, scale(after, d)) });
        let corner_section = Profile { origin: c, normal: cross(n, before), loops: vec![triangle(c, before)], names: Vec::new() };
        pieces.push(Piece::new(corner_section, Sweep::About { origin: c, axis: n, angle: phi }));
        let side = Profile { origin: c, normal: along[k], loops: vec![triangle(c, after)], names: Vec::new() };
        pieces.push(Piece::new(side, Sweep::Along(norm(sub(next, c)))));
    }
    Ok((edges, pieces))
}

/// Loop `l` as segments in space (model units), a circle's a polygon fine enough to read how far
/// it lies from another loop.
fn polyline(body: &BodyFaces, l: &[crate::brep::topo::Coedge]) -> Vec<[V; 2]> {
    let mut out = Vec::new();
    for c in l {
        let k = if body.segment(c.edge).is_some() { 1 } else {
            let [t0, t1] = body.span(c.edge);
            ((t1 - t0) / TAU * 256.0).ceil().max(8.0) as usize
        };
        out.extend((0..k).map(|j| [body.along(c.edge, j as f64 / k as f64), body.along(c.edge, (j + 1) as f64 / k as f64)]));
    }
    out
}

/// How far apart two loops of one face are: the least distance between their segments, which
/// cross nowhere.
fn apart(a: &[[V; 2]], b: &[[V; 2]]) -> f64 {
    let to = |p: V, [s, e]: [V; 2]| {
        let u = sub(e, s);
        let t = (dot(sub(p, s), u) / dot(u, u).max(f64::MIN_POSITIVE)).clamp(0.0, 1.0);
        norm(sub(p, add(s, scale(u, t))))
    };
    let mut least = f64::INFINITY;
    for &x in a {
        for &y in b {
            least = least.min(to(x[0], y)).min(to(x[1], y)).min(to(y[0], x)).min(to(y[1], x));
        }
    }
    least
}

/// Cylinder face `fi` on `frame` (radius `r`; both in the boundary's millimetres, `mm` to the
/// model's unit), where it is a band between two circles about its axis, its sides lines along it:
/// the rectangle from its radius to `d` further along its outward normal (out of a shaft, into a
/// bore) and its length along the axis (model units), in the half-plane through its first side,
/// turned through its angle; at a rim that is a foot (`concave`) its far corner moved `d` up the
/// far side, so it is cut back along the cone the plane's prism meets it in.  `None` for any other
/// face, a bore no wider than the reach, or a band no longer than the reach between its feet.
fn band(body: &BodyFaces, fi: usize, frame: Frame, r: f64, mm: f64, concave: &[bool], d: f64) -> Option<Piece> {
    let (b, _, _) = body.brep();
    let tol = 1e-9 * (r + norm(frame.o)).max(1.0);
    let f = &b.faces[fi];
    // each rim's level along the axis, and whether it is a foot
    let (mut levels, mut turns) = (Vec::<(f64, bool)>::new(), Vec::<[f64; 2]>::new());
    let angle = |p: V| { let l = frame.local(p); l[1].atan2(l[0]) };
    let mut on = None;
    for c in f.loops.iter().flatten() {
        let edge = &b.edges[c.edge as usize];
        let EdgeCurve::Curve(curve) = &edge.curve else { return None };
        on.get_or_insert(curve.point(edge.t[0]));
        match curve {
            Curve::Line { d: along, .. } if dot(*along, frame.z).abs() > 1.0 - 1e-9 => {}
            Curve::Circle(cf, _) if dot(cf.z, frame.z).abs() > 1.0 - 1e-9
                && norm(cross(sub(cf.o, frame.o), frame.z)) <= tol => {
                let v = frame.local(cf.o)[2] / mm;
                let foot = concave[c.edge as usize];
                match levels.iter_mut().find(|w| (w.0 - v).abs() * mm <= tol) {
                    Some(w) => w.1 |= foot,
                    None => levels.push((v, foot)),
                }
                let [t0, t1] = edge.t;
                if edge.closed() {
                    // from where the circle starts, as its edge's wedge does
                    let a = angle(curve.point(t0));
                    turns.push([a, a + TAU]);
                } else {
                    let from = if dot(cf.z, frame.z) > 0.0 { curve.point(t0) } else { curve.point(t1) };
                    let a = angle(from);
                    turns.push([a, a + (t1 - t0)]);
                }
            }
            _ => return None,
        }
    }
    let [a, z] = levels[..] else { return None };
    let ((v0, f0), (v1, f1)) = if a.0 <= z.0 { (a, z) } else { (z, a) };
    let &first = turns.first()?;
    let same = |a: [f64; 2]| {
        let shift = (a[0] - first[0]).rem_euclid(TAU);
        let whole = first[1] - first[0] >= TAU - 1e-9;
        ((a[1] - a[0]) - (first[1] - first[0])).abs() <= 1e-9 && (whole || shift <= 1e-9 || TAU - shift <= 1e-9)
    };
    if !turns.iter().all(|&a| same(a)) {
        return None;
    }
    // out of a shaft, or into a bore: whichever way the face's material is
    let at = on?;
    let radial = { let l = frame.local(at); add(scale(frame.x, l[0]), scale(frame.y, l[1])) };
    let out = dot(body.normal(fi, body.back(at)), radial) > 0.0;
    let r = r / mm;
    let far = if out { r + d } else { r - d };
    // at a foot cut back along the cone the plane's prism meets it in: `d` short of it at its far side
    let (s0, s1) = (if f0 { d } else { 0.0 }, if f1 { d } else { 0.0 });
    if !(far > 0.0) || !(v1 - v0 > s0 + s1) {
        return None;
    }
    let [u0, u1] = first;
    let e = add(scale(frame.x, u0.dcos()), scale(frame.y, u0.dsin()));
    let o = body.back(frame.o);
    let point = |rr: f64, v: f64| add(o, add(scale(e, rr), scale(frame.z, v)));
    let corners = [point(r, v0), point(far, v0 + s0), point(far, v1 - s1), point(r, v1)];
    let edges = (0..4).map(|k| ProfileEdge::Line { a: corners[k], b: corners[(k + 1) % 4] }).collect();
    Some(Piece::new(Profile { origin: point(r, v0), normal: cross(frame.z, e), loops: vec![edges], names: Vec::new() },
                    Sweep::About { origin: o, axis: frame.z, angle: u1 - u0 }))
}

/// **The gate**, as the other constructions': the body grown stands only where the region's own
/// field agrees with it — built about the body's boundary, which it is glued to.
fn gate(sk: &Sketch, solid: usize, g: &Grown, body: &BodyFaces) -> Result<(), String> {
    let (b, mm, origin) = body.brep();
    let w = Writer::about(mm, origin.map(|x| x * mm));
    let mut built = std::collections::BTreeMap::new();
    built.insert(g.body as i64, b.clone());
    let brep = crate::brep::recipe::node(&g.node(&w)?, &built)?;
    let field = super::MaterialField::read(sk, solid, super::cad::AXIS_TOLERANCE)?;
    let back = |p: V| body.back(p);
    super::agreement::gate(&field, &brep, Some((&back, mm)), "its exact construction disagrees with the field")
}
