//! **A body grown, built exactly** (`std.Grown`): a region solid read in general position whose one
//! term bounds the distance from a body's material, `{ p | distance(p, within) <= d }` with
//! `within := { c | c inside B }`, is every point within `d` of `B` — `B` and the points nearer
//! than `d` outside it.  Outside, a point's nearest point of `B` is inside a face, inside an edge or
//! a corner, so where every edge of `B` is convex what it adds is a partition into those points:
//! each face's prism `d` out along its normal (a cylinder's band, its radius grown or a bore's
//! shrunk by `d`), each edge's wedge of radius `d` between its two faces' normals (a torus's where
//! the edge is a circle), and each corner's sector of the ball where three faces meet.  Each piece
//! shares its faces exactly with the next — a prism its foot with the face, its sides with the
//! wedges, a wedge its ends with the sectors — so they are glued to `B` there and nothing is
//! intersected (`boolean::glued`): the boundary of `B` grown, planes, cylinders, tori and spheres
//! meeting tangent.  Held to the region read point by point before it stands (`gate`), as the
//! other exact constructions are.  A concave edge, a face no plane or cylinder carries, an edge no
//! line or circle, a corner that is no prism's (three faces, one square to the other two), a bore
//! no wider than the reach, or a body whose faces another solid reads leave it the field's, said by
//! name.  The recipe's `grown` node: the body's own node, and its pieces each a `prism` or a
//! `revolve`.
use crate::brep::build::{Profile, ProfileEdge};
use crate::brep::geom::{Curve, Frame, Surface};
use crate::brep::recipe::Writer;
use crate::brep::topo::EdgeCurve;
use crate::json::{object, Json};
use crate::model::{FaceRef, RegionShape, RegionTerm, Sketch, SolidDef, Target};
use crate::space::{add, cross, dot, norm, scale, sub};
use crate::vocab::Cmp;
use std::f64::consts::TAU;
use std::rc::Rc;

#[allow(unused_imports)]
use crate::fmath::Det;

type V = [f64; 3];

/// A body grown: the body's solid, and what is glued to it (model units).
#[derive(Clone, Debug)]
pub struct Grown {
    body: u32,
    pieces: Vec<Piece>,
    /// What the pieces' faces are called: the term's set.
    name: String,
}

/// One piece: a section, and how it is swept.
#[derive(Clone, Debug)]
struct Piece {
    section: Profile,
    sweep: Sweep,
}

#[derive(Clone, Copy, Debug)]
enum Sweep {
    /// Along the section's normal, from `from` to `to`.
    Along { from: f64, to: f64 },
    /// About the line through `origin` along the unit `axis`, right-handed by `angle`.
    About { origin: V, axis: V, angle: f64 },
}

impl Grown {
    /// The recipe's node, written by `w`: `grown`, the body's node and its `pieces` each a `prism`
    /// or a `revolve`.
    pub fn node(&self, w: &Writer) -> Result<Json, String> {
        let s = w.scale;
        let pieces = self.pieces.iter().map(|piece| {
            let names = piece.section.loops.iter().map(|l| vec![self.name.clone(); l.len()]).collect();
            let profile = Profile { names, ..piece.section.clone() }.to_json(w)?;
            Ok(match piece.sweep {
                Sweep::Along { from, to } => object([("kind", "prism".into()), ("profile", profile),
                    ("from", (from * s).into()), ("to", (to * s).into())]),
                Sweep::About { origin, axis, angle } => object([("kind", "revolve".into()),
                    ("profile", profile), ("origin", w.line_origin(origin.map(|x| x * s), axis)),
                    ("axis", Writer::triple(axis)), ("angle", angle.into())]),
            })
        }).collect::<Result<Vec<_>, String>>()?;
        Ok(object([("kind", "grown".into()), ("body", (self.body as usize).into()), ("pieces", Json::Arr(pieces))]))
    }
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

/// The body region solid `solid` grows, where this construction builds it: the solid its recipe
/// node reads.
pub fn body_of(sk: &Sketch, solid: usize) -> Option<u32> {
    let (face, _) = recognised(sk, solid)?;
    matches!(built(sk, solid), Some(Ok(_))).then_some(face.solid)
}

/// Region solid `solid` built as its body and what lies within reach of each face, edge and corner
/// glued to it, where it is that region (`None` otherwise): the construction, or why it stands not.
/// Remembered against `solid::reads` (`Caches::grown`).
pub fn built(sk: &Sketch, solid: usize) -> Option<Result<Rc<Grown>, String>> {
    recognised(sk, solid)?;
    sk.caches.grown.get_or(solid, super::MemoKey::of(sk, solid), || {
        Some(construction(sk, solid).and_then(|(g, body)| {
            gate(sk, solid, &g, &body)?;
            Ok(Rc::new(g))
        }))
    })
}

/// The pieces of region solid `solid`, and the faces of the body they are glued to.
fn construction(sk: &Sketch, solid: usize) -> Result<(Grown, std::sync::Arc<super::faces::BodyFaces>), String> {
    let (face, t) = recognised(sk, solid).expect("a body grown, asked by `built`");
    let d = t.lo.read(sk).ok_or("its reach reads an unknown nothing determines")?;
    if !(d > 0.0) {
        return Err("a body grown by nothing is the body itself".to_string());
    }
    // the faces its glued pieces meet are the body's own, as its recipe node builds it: none of
    // its terms reads them (a round, which the faces are read without)
    let readers = super::faces::readers_of(sk, face.solid as usize);
    let mut terms = vec![face.solid];
    let mut k = 0;
    while k < terms.len() {
        let more = sk.solids[terms[k] as usize].operands();
        terms.extend(more.into_iter().filter(|o| !terms.contains(o)).collect::<Vec<_>>());
        k += 1;
    }
    if terms.iter().any(|&o| readers.contains(&(o as usize))) {
        return Err("a body grown whose faces a term of it reads (a round) is read by the field".to_string());
    }
    let mut read = Vec::new();
    let reading = super::faces::reading(sk, face, &mut read)?;
    let body = reading.body.clone();
    let (b, mm, _) = body.brep();
    if reading.faces.len() != b.faces.len() {
        return Err("within reach of only some of a body's faces is read by the field".to_string());
    }
    let refused = |why: &str| format!("{why}: read by the field");
    let back = |q: V| body.back(q);
    let mut pieces = Vec::new();
    // each face's prism (a cylinder's band) out along its normal
    for fi in 0..b.faces.len() {
        let f = &b.faces[fi];
        pieces.push(match f.surface {
            Surface::Plane(_) => {
                let (n, o) = body.plane(fi).expect("a plane face");
                let section = section_of(b, fi, n, o, &back, mm).ok_or_else(|| refused("a plane face bounded by an edge no line or circle carries"))?;
                Piece { section, sweep: Sweep::Along { from: 0.0, to: d } }
            }
            Surface::Cylinder(frame, r) => band(&body, fi, frame, r, mm, d)
                .ok_or_else(|| refused("a cylinder's face that is no band between two circles about its axis, \
                                        or a bore no wider than the reach"))?,
            _ => return Err(refused(&format!("a face on a {}", surface_word(&f.surface)))),
        });
    }
    // each convex edge's wedge between its faces' normals: along a line, or turned about a circle
    for e in 0..b.edges.len() as u32 {
        let &[fa, fb] = body.faces_of(e) else { continue };
        let edge = &b.edges[e as usize];
        let EdgeCurve::Curve(curve) = &edge.curve else { continue };
        match body.convex(e) {
            Some(true) => {}
            // the faces go on tangent: their prisms meet there
            None => continue,
            Some(false) => return Err(refused("a concave edge")),
        }
        let [t0, t1] = edge.t;
        let at = curve.point(t0);
        let (na, nb) = (body.outward(fa, at), body.outward(fb, at));
        let wedge = |o: V, along: V| wedge(back(o), along, na, nb, d);
        pieces.push(match curve {
            Curve::Line { d: along, .. } => {
                // the faces' normals the same all along it
                let end = curve.point(t1);
                if norm(sub(body.outward(fa, end), na)) > 1e-9 || norm(sub(body.outward(fb, end), nb)) > 1e-9 {
                    return Err(refused("an edge along which a face turns"));
                }
                Piece { section: wedge(at, *along), sweep: Sweep::Along { from: 0.0, to: (t1 - t0) / mm } }
            }
            Curve::Circle(frame, r) => {
                let tangent = add(scale(frame.x, -t0.dsin()), scale(frame.y, t0.dcos()));
                if dot(na, tangent).abs() > 1e-9 || dot(nb, tangent).abs() > 1e-9 {
                    return Err(refused("a circle whose faces turn about no axis of its own"));
                }
                // turned about its axis, the wedge stays clear of it
                if !(r / mm > d) {
                    return Err(refused("a circle no larger than the reach"));
                }
                Piece { section: wedge(at, tangent),
                        sweep: Sweep::About { origin: back(frame.o), axis: frame.z, angle: (t1 - t0).min(TAU) } }
            }
            _ => return Err(refused("an edge no line or circle carries")),
        });
    }
    // each corner's sector of the ball: where three faces meet, one square to the other two (a
    // prism's corner), the quarter disc between the square one and another turned about it
    for v in 0..b.vertices.len() as u32 {
        let at = b.vertices[v as usize].p;
        let mut normals: Vec<V> = Vec::new();
        for e in 0..b.edges.len() as u32 {
            if !body.ends(e).contains(&v) {
                continue;
            }
            for &fi in body.faces_of(e) {
                let n = body.outward(fi, at);
                if !normals.iter().any(|&m| norm(sub(m, n)) < 1e-9) {
                    normals.push(n);
                }
            }
        }
        if normals.len() < 3 {
            continue;
        }
        let corner = (normals.len() == 3).then(|| (0..3).find(|&k| {
            (0..3).filter(|&j| j != k).all(|j| dot(normals[k], normals[j]).abs() < 1e-9)
        })).flatten().ok_or_else(|| refused("a corner that is no prism's: three faces, one square to the other two"))?;
        let nc = normals[corner];
        let [na, nb] = [normals[(corner + 1) % 3], normals[(corner + 2) % 3]];
        let c = back(at);
        let section = wedge(c, cross(na, nc), na, nc, d);
        let angle = dot(cross(na, nb), nc).atan2(dot(na, nb));
        pieces.push(Piece { section, sweep: Sweep::About { origin: c, axis: nc, angle } });
    }
    Ok((Grown { body: face.solid, pieces, name: t.name.clone() }, body))
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

/// What a surface is called where this construction refuses it.
fn surface_word(s: &Surface) -> &'static str {
    match s {
        Surface::Cone(..) => "cone",
        Surface::Sphere(..) => "sphere",
        Surface::Torus(..) => "torus",
        _ => "surface no line or circle sweeps",
    }
}

/// Plane face `fi`'s loops (lines and circles) as a section in its plane, through `o` (model units)
/// square to its outward normal `n`; `None` where an edge is no line or circle.
fn section_of(b: &crate::brep::topo::Brep, fi: usize, n: V, o: V, back: &dyn Fn(V) -> V, mm: f64) -> Option<Profile> {
    let mut loops = Vec::new();
    for l in &b.faces[fi].loops {
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
        loops.push(out);
    }
    Some(Profile { origin: o, normal: n, loops, names: Vec::new() })
}

/// Cylinder face `fi` on `frame` (radius `r`; both in the boundary's millimetres, `mm` to the
/// model's unit), where it is a band between two circles about its axis, its sides lines along it:
/// the rectangle from its radius to `d` further along its outward normal (out of a shaft, into a
/// bore) and its length along the axis (model units), in the half-plane through its first side,
/// turned through its angle.  `None` for any other face, or a bore no wider than the reach.
fn band(body: &super::faces::BodyFaces, fi: usize, frame: Frame, r: f64, mm: f64, d: f64) -> Option<Piece> {
    let (b, _, _) = body.brep();
    let tol = 1e-9 * (r + norm(frame.o)).max(1.0);
    let f = &b.faces[fi];
    let (mut levels, mut turns) = (Vec::<f64>::new(), Vec::<[f64; 2]>::new());
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
                if !levels.iter().any(|&w| (w - v).abs() * mm <= tol) {
                    levels.push(v);
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
    let [v0, v1] = levels[..] else { return None };
    let (v0, v1) = (v0.min(v1), v0.max(v1));
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
    let out = dot(body.outward(fi, at), radial) > 0.0;
    let r = r / mm;
    let (inner, outer) = if out { (r, r + d) } else { (r - d, r) };
    if !(inner > 0.0) {
        return None;
    }
    let [u0, u1] = first;
    let e = add(scale(frame.x, u0.dcos()), scale(frame.y, u0.dsin()));
    let o = body.back(frame.o);
    let point = |rr: f64, v: f64| add(o, add(scale(e, rr), scale(frame.z, v)));
    let corners = [point(inner, v0), point(outer, v0), point(outer, v1), point(inner, v1)];
    let edges = (0..4).map(|k| ProfileEdge::Line { a: corners[k], b: corners[(k + 1) % 4] }).collect();
    Some(Piece { section: Profile { origin: point(r, v0), normal: cross(frame.z, e), loops: vec![edges], names: Vec::new() },
                 sweep: Sweep::About { origin: o, axis: frame.z, angle: u1 - u0 } })
}

/// **The gate**, as the other constructions': the body grown stands only where the region's own
/// field agrees with it — built about the body's boundary, which it is glued to.
fn gate(sk: &Sketch, solid: usize, g: &Grown, body: &super::faces::BodyFaces) -> Result<(), String> {
    let (b, mm, origin) = body.brep();
    let w = Writer::about(mm, origin.map(|x| x * mm));
    let mut built = std::collections::BTreeMap::new();
    built.insert(g.body as i64, b.clone());
    let brep = crate::brep::recipe::node(&g.node(&w)?, &built)?;
    let field = super::MaterialField::read(sk, solid, super::cad::AXIS_TOLERANCE)?;
    let back = |p: V| body.back(p);
    super::agreement::gate(&field, &brep, Some((&back, mm)), "its exact construction disagrees with the field")
}
