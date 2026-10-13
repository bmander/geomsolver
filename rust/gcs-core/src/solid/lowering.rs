//! **What a region over a body's faces is built as** (#194, #183's L7): the rounds and chamfers of
//! `rust/lib/std.sv` say what they mean as sets, and the field reads them point by point; where a
//! set is one this table recognises, the same points are built exactly — a section swept along the
//! edge it rounds, a ball's patch where three meet — so a body taking it is exported by the kernel
//! and evaluated as a B-rep.
//!
//! **Lowering is a representation, never a meaning** (`lowering.rs`'s rule, here for solids): an
//! entry is recognised from the region's terms — what each bounds, of which faces, by which
//! number — never from the name of the word that wrote them, and what it builds is held to the
//! region's own field before anything reads it (`gate`). A region no entry recognises stays the
//! field's, its exact export refused by name.
//!
//! The entries so far, over plane faces meeting in straight edges (or a plane and a cylinder, or
//! two cylinders, about lines along the edge: in the section lines and circles), planes meeting at
//! any corner, and planes, cylinders, cones and spheres about one axis meeting in a circle; a face
//! named in pieces (a plate's top a rod splits) is a piece per edge. A run is where its nearest point of the faces' meeting is on the edge (the
//! slab square to it over its stretch, a sector of a ring's) and on the side the edge runs on of
//! each face at its ends: a straight run's prism is kept within each plane face standing at a slant
//! there and reaching into it (a run-out), one leaning away leaving the slab's square end; one ending
//! on a cylinder or a sphere is carried on into it and kept on its side of that solid (`Kept`). A face
//! going on tangent from one of the run's is no end (a seam), so a round along an obround's flat
//! side stops at the section the ring round its half circle starts from, and the two are glued
//! there; a ring's stretch is turned through its circle's angle, and on round past an end on a
//! plane not through its axis until its section has crossed it, kept within it (or into a
//! cylinder or a ball it ends on, kept on its side of that solid).
//!
//! | the region's body | built as |
//! |---|---|
//! | `Run` / `Fill`: within (outside) two faces, beyond reach of the ball rolled along their edge | a prism of the corner less the ball's arc (a cylinder face); beside a cylinder, its arc a side of the section; between two, the ball where the two circles grown by its radius meet |
//! | `Flat` / `FlatFill`: within (outside) two faces, short of the flat through the legs | a prism of the triangle (a plane face); beside a cylinder (or a sphere, in a meridian) the flat is a conic's arc, the sum of a distance from a line and one from a circle (`Corner2`, by `brep::corners`' rule: an extrusion, or a revolution, face) |
//! | `Corner` / `CornerFill`: within (outside) three planes meeting at a corner, beyond reach of the ball touching all three | the corner's cell — the trihedron cut by the sections through the ball's centre square to its edges — less the ball (a sphere's patch), and each edge's run beyond it |
//! | the same where a face is a cylinder or a sphere (a D's flat, round and cap) | the ball where Newton's steps put it, `r` from each surface; its cell over the three surfaces (`brep::fillet::corner_among`), and each edge's run or ring stretch cut short at its section there |
//! | `Run` / `Fill` / `Flat` round a circle (or a stretch of one between seams) where a plane, a cylinder, a cone or a sphere meet about one axis | the section turned about the axis through the circle's angle (a torus face); in the meridian a sphere is a circle |
//! | `Run` / `Fill` along an edge no line or coaxial circle carries (two pipes' saddle, a plane across a cylinder) | the ball rolled along the run, traced, against the stratum (`brep::fillet::roll`): its canal fitted, carried past each end and cut off by the plane there |
//! | any of them where the faces meet nowhere, or an edge turns the other way | nothing |
//!
//! A region is its **pieces** (`Piece`: a run along an edge, a ball's cell at a corner), and the
//! regions a body takes in one set (its cuts, its fills) are one union of pieces (`assemble`):
//! each distinct piece once, a run cut short at the section of a corner standing at its end and
//! joined to the corner's cell there (`brep::boolean::glued`, nothing intersected), and what is
//! left unioned (two runs crossing at a corner whose third edge stays sharp: a mitre).
#[allow(unused_imports)]
use crate::fmath::Det;
use crate::json::{object, Json};
use crate::model::{FaceRef, RegionShape, RegionTerm, Sketch, SolidDef, Target};
use crate::space::{add, cross, distance, dot, norm, normalised, scale, sub};
use crate::syntax::Cmp;
use std::rc::Rc;

type V = [f64; 3];

/// A stroke of a lowered region's section, in model units: a line, or an arc of the circle about
/// `centre` turning counter-clockwise about the section's normal from `angles[0]` to `angles[1]`
/// (radians, measured from the unit `x`), or a conic's arc: the rational quadratic Bézier from `a`
/// to `b` over `m`, where their tangents meet, its middle weighed `w`.
#[derive(Clone, Debug, PartialEq)]
pub enum Stroke {
    Line { from: V, to: V },
    Arc { centre: V, radius: f64, x: V, angles: [f64; 2] },
    Conic { a: V, m: V, b: V, w: f64 },
}

/// A section swept along a straight edge (model units): the section in the plane through `origin`
/// (the edge's first end) square to the unit `along`, its loop `strokes` (each named, the name of
/// the face it sweeps), carried over the stretch `span` of the edge (`0` its origin; the edge, or
/// past its ends), and kept within each half-space `n·x <= d` of `within` (`(n, d)`, `n` a unit
/// normal): the faces an end stands at a slant to cut it off; and on its side of each curved face
/// of `kept` it ends on (#218's 2).
#[derive(Clone, Debug, PartialEq)]
pub struct Run {
    pub origin: V,
    pub along: V,
    pub span: [f64; 2],
    pub strokes: Vec<(Stroke, &'static str)>,
    pub within: Vec<(V, f64)>,
    pub kept: Vec<Kept>,
}

/// A face's surface as a corner's cell or a piece's end reads it (model units): a plane through
/// `origin`, its `normal` away from the material; a cylinder of `radius` about the line through
/// `origin` along the unit `axis`; a cone about it, `radius` at `origin` and growing along `axis`
/// by the tangent of `half` (a drafted eye's side, #245); or a sphere.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Surf {
    Plane { origin: V, normal: V },
    Cylinder { origin: V, axis: V, radius: f64 },
    Cone { origin: V, axis: V, radius: f64, half: f64 },
    Sphere { centre: V, radius: f64 },
}

impl Surf {
    /// Face `fi`'s surface, where it is one of these.
    fn of(body: &super::faces::BodyFaces, fi: usize) -> Option<Surf> {
        if let Some((normal, origin)) = body.plane(fi) {
            Some(Surf::Plane { origin, normal })
        } else if let Some((origin, axis, radius)) = body.cylinder(fi) {
            Some(Surf::Cylinder { origin, axis, radius })
        } else if let Some((origin, axis, radius, half)) = body.cone(fi) {
            Some(Surf::Cone { origin, axis, radius, half })
        } else {
            body.sphere(fi).map(|(centre, radius)| Surf::Sphere { centre, radius })
        }
    }

    /// A cylinder or a cone about its axis: a point of the axis, its unit direction, the radius
    /// there and the half-angle (a cylinder's nothing, whose tangent and cosine change no bit).
    fn axial(&self) -> Option<(V, V, f64, f64)> {
        match *self {
            Surf::Cylinder { origin, axis, radius } => Some((origin, axis, radius, 0.0)),
            Surf::Cone { origin, axis, radius, half } => Some((origin, axis, radius, half)),
            Surf::Plane { .. } | Surf::Sphere { .. } => None,
        }
    }

    /// From `x` to the surface along its normal: positive on the side the plane's normal points
    /// to, or outside the cylinder, the cone or the ball.
    fn side(&self, x: V) -> f64 {
        match *self {
            Surf::Plane { origin, normal } => dot(normal, sub(x, origin)),
            Surf::Sphere { centre, radius } => distance(x, centre) - radius,
            _ => {
                let (origin, axis, radius, half) = self.axial().expect("a cylinder or a cone");
                let w = sub(x, origin);
                let h = dot(w, axis);
                (norm(sub(w, scale(axis, h))) - radius - h * half.dtan()) * half.dcos()
            }
        }
    }

    /// Where the line `q + s·along` crosses a cylinder, a cone or a sphere: its two crossings in
    /// order, if it does (a plane is no solid a line goes into and out of; a cone's on its own
    /// nappe, short of its apex).
    fn crossings(&self, q: V, along: V) -> Option<[f64; 2]> {
        // |w⊥ + s a⊥|² = (r + t (h + s a·axis))², the right side's root positive: a sphere's with
        // nothing taken off and no taper
        let (w, a, radius, t, k) = match *self {
            Surf::Plane { .. } => return None,
            Surf::Sphere { centre, radius } => (sub(q, centre), along, radius, 0.0, 0.0),
            _ => {
                let (origin, axis, radius, half) = self.axial().expect("a cylinder or a cone");
                let flat = |v: V| sub(v, scale(axis, dot(v, axis)));
                let (w, t) = (sub(q, origin), half.dtan());
                (flat(w), flat(along), radius + t * dot(w, axis), t, dot(along, axis))
            }
        };
        match crate::roots::quadratic(dot(a, a) - t * t * k * k, 2.0 * (dot(w, a) - t * radius * k),
            dot(w, w) - radius * radius)[..] {
            [s0, s1] if [s0, s1].iter().all(|&s| radius + t * k * s > 0.0) => Some([s0.min(s1), s0.max(s1)]),
            _ => None,
        }
    }

    /// As a recipe reads it (millimetres, `by` of them a model unit; `recipe::surface_of`).
    fn json(&self, by: f64) -> Json {
        use super::cad::vector;
        match *self {
            Surf::Plane { origin, normal } => object([("kind", "plane".into()),
                ("origin", vector(scale(origin, by))), ("normal", vector(normal))]),
            Surf::Cylinder { origin, axis, radius } => object([("kind", "cylinder".into()),
                ("origin", vector(scale(origin, by))), ("axis", vector(axis)), ("radius", (radius * by).into())]),
            Surf::Cone { origin, axis, radius, half } => object([("kind", "cone".into()),
                ("origin", vector(scale(origin, by))), ("axis", vector(axis)), ("radius", (radius * by).into()),
                ("half", half.into())]),
            Surf::Sphere { centre, radius } => object([("kind", "sphere".into()),
                ("origin", vector(scale(centre, by))), ("radius", (radius * by).into())]),
        }
    }
}

/// A curved face a piece ends on, carried on into it and cut off by it: the piece kept within the
/// solid its surface bounds (`inside`) or outside it.
#[derive(Clone, Debug, PartialEq)]
pub struct Kept {
    pub face: Surf,
    pub inside: bool,
}

impl Kept {
    /// How far `x` stands on the side cut off (positive there).
    fn away(&self, x: V) -> f64 { if self.inside { self.face.side(x) } else { -self.face.side(x) } }

    /// Whether its surface passes through `p` (within a billionth of `size`).
    fn through(&self, p: V, size: f64) -> bool { self.face.side(p).abs() <= 1e-9 * size }
}

impl Run {
    /// A round's ball centre in the section at `origin`: its arc's.
    pub fn centre(&self) -> Option<V> {
        self.strokes.iter().find_map(|(s, _)| match *s { Stroke::Arc { centre, .. } => Some(centre), _ => None })
    }

    /// Points at both ends of the prism whose hull holds it (`section_hull`, at either end).
    fn hull(&self) -> Vec<V> {
        let [from, to] = self.span.map(|s| scale(self.along, s));
        self.section_hull().into_iter().flat_map(|p| [add(p, from), add(p, to)]).collect()
    }

    /// Points of the section's plane whose hull holds the section: each line's ends, and each
    /// arc's or conic's with the point its tangents there meet (each lies within those three).
    fn section_hull(&self) -> Vec<V> {
        self.strokes.iter().flat_map(|(s, _)| match *s {
            Stroke::Line { from, to } => vec![from, to],
            Stroke::Arc { centre, radius, x, angles: [a0, a1] } => {
                let frame = crate::brep::geom::Frame::new(centre, self.along, x);
                let at = |a: f64, k: f64| { let (s, c) = a.dsin_cos(); frame.at([k * c, k * s, 0.0]) };
                let apex = radius / (0.5 * (a1 - a0)).dcos().max(1e-3);
                vec![at(a0, radius), at(a1, radius), at(0.5 * (a0 + a1), apex)]
            }
            Stroke::Conic { a, m, b, .. } => vec![a, m, b],
        }).collect()
    }

    /// Where its stretch ends, `k` 0 or 1.
    fn end(&self, k: usize) -> V { add(self.origin, scale(self.along, self.span[k])) }

    /// No longer kept within the faces standing at `vertex`: a corner's cell takes that end over.
    fn release(&mut self, vertex: V, size: f64) {
        self.within.retain(|&(m, d)| (dot(m, vertex) - d).abs() > 1e-9 * size);
        self.kept.retain(|k| !k.through(vertex, size));
    }
}

/// A ball's patch where three faces meet (model units): the corner's cell — the trihedron at
/// `vertex` cut by the three sections through `centre` square to its edges — less the ball of
/// radius `r` about `centre`, over the faces' surfaces (`brep::fillet::corner_among`).
#[derive(Clone, Debug, PartialEq)]
pub struct Corner {
    pub vertex: V,
    pub centre: V,
    pub r: f64,
    pub faces: [Surf; 3],
}

/// A section turned about an axis (model units): a run round a circle, or a stretch of one, where
/// a plane meets a cylinder or a cone about it — the section in the meridian through
/// `section.origin`, square to `section.along` (its `span` nothing), turned `angle` (radians, a
/// whole turn for a whole circle) about the line through `axis_origin` along the unit `axis`.
#[derive(Clone, Debug, PartialEq)]
pub struct Ring {
    pub section: Run,
    pub axis_origin: V,
    pub axis: V,
    pub angle: f64,
}

impl Ring {
    /// Where its section stands at the end of its turn, and the way the turn runs on there.
    fn end(&self) -> (V, V) {
        let turn = crate::brep::geom::Rigid::turn(self.axis_origin, self.axis, self.angle);
        (turn.point(self.section.origin), turn.vector(self.section.along))
    }
}

/// A piece of what a region is built as; a region is the union of its pieces, and one of none
/// holds nowhere (its faces never meet, or an edge they meet in turns the other way from the one
/// its gate asks).
#[derive(Clone, Debug, PartialEq)]
pub enum Piece {
    Run(Run),
    Ring(Ring),
    Corner(Corner),
    Rolled(Rolled),
}

/// A ball rolled along a run of edges no line or coaxial circle carries — a pipe tee's crotch, a
/// bore's rim on a pipe's wall, a boss's root across a sloped face (model units): the ball of
/// radius `r` touching both faces, rolled from where it stands over `at` on the run, its spine
/// traced and its canal fitted (`brep::fillet::roll`) against the body as its other terms make it
/// (the stratum), where the recipe builds it.
#[derive(Clone, Debug, PartialEq)]
pub struct Rolled {
    pub at: V,
    pub r: f64,
}

/// The faces a region rounds or cuts at, and whether it lies within all of them (a convex edge's
/// material) or outside all of them (a concave edge's void).
struct Faces<'a> {
    faces: Vec<&'a FaceRef>,
    within: bool,
}

impl Faces<'_> {
    /// The pairs of faces meeting in an edge: one for two faces, three for three.
    fn pairs(&self) -> &'static [[usize; 2]] {
        if self.faces.len() == 2 { &[[0, 1]] } else { &[[0, 1], [1, 2], [2, 0]] }
    }

    /// The pair `p` is, as its faces' surfaces or patches (`surface`), and whether it names them
    /// the other way round.
    fn pair_of(&self, p: [&FaceRef; 2], surface: bool) -> Option<(usize, bool)> {
        self.pairs().iter().enumerate().find_map(|(k, &[i, j])| {
            let is = |x: usize, y: usize| same(self.faces[x], p[0], surface) && same(self.faces[y], p[1], surface);
            if is(i, j) { Some((k, false)) } else if is(j, i) { Some((k, true)) } else { None }
        })
    }

    /// Whether `set` is every face once, each its surface or its patch.
    fn all(&self, set: &[&FaceRef], surface: bool) -> bool {
        set.len() == self.faces.len()
            && self.faces.iter().all(|f| set.iter().filter(|g| same(f, g, surface)).count() == 1)
    }
}

/// What the region's section is beside its edges.
#[derive(Clone, Copy, Debug)]
enum Section {
    /// Beyond reach of a ball of this radius touching every face.
    Round(f64),
    /// Short of the flat through the point `la` along `a` and the point `lb` along `b`, each from
    /// the other face's surface.
    Flat { la: f64, lb: f64 },
}

impl Section {
    /// The same section, its faces named the other way round.
    fn swapped(self) -> Section {
        match self { Section::Flat { la, lb } => Section::Flat { la: lb, lb: la }, s => s }
    }
}

/// Region solid `solid`'s lowering, its pieces (none where it holds nowhere): `None` where no
/// entry recognises it (it stays the field's), else what it is built as or why the entry that
/// recognised it refuses it. Remembered against its own terms (`document::reads_own`) and the
/// stratum it reads, `faces::cached`'s (`Sketch::lowering_cache`): a body rounded all over has
/// hundreds of regions, each reading one body, and a key over the body's every term would walk
/// them all for each.
pub fn lowered(sk: &Sketch, solid: usize) -> Option<Result<Rc<Vec<Piece>>, String>> {
    let SolidDef::Region { terms, .. } = &sk.solids.get(solid)?.def else { return None };
    let (faces, section) = recognised(sk, terms)?;
    let body = match super::faces::cached(sk, faces.faces[0].solid as usize) {
        Ok(b) => b,
        Err(e) => return Some(Err(e)),
    };
    let key = super::document::reads_own(sk, solid, 0.0);
    if let Some((old, b, l)) = sk.lowering_cache.borrow().get(&solid) {
        if *old == key && std::sync::Arc::ptr_eq(b, &body) {
            return Some(l.clone());
        }
    }
    let l = build(&body, &faces, section).and_then(|pieces| {
        gate(sk, solid, &pieces, &body)?;
        Ok(Rc::new(pieces))
    });
    sk.lowering_cache.borrow_mut().insert(solid, (key, body, l.clone()));
    Some(l)
}

/// A term's number, where it states one and bounds it on one side, with no offset.
fn plain(sk: &Sketch, t: &RegionTerm) -> Option<f64> {
    (t.hi.is_none() && t.grow.is_none()).then(|| t.lo.read(sk)).flatten()
}

/// Whether two face references name the same faces, the second of them its surface or not.
fn same(x: &FaceRef, y: &FaceRef, surface: bool) -> bool {
    x.solid == y.solid && x.path == y.path && !x.surface && y.surface == surface
}

/// A term bounding the distance from one face (or its surface): the face, which way, its number.
fn one_face<'a>(sk: &Sketch, t: &'a RegionTerm) -> Option<(&'a FaceRef, Cmp, f64)> {
    let RegionShape::Measured { of } = &t.shape else { return None };
    let [(Target::Face(f), c)] = &of[..] else { return None };
    (*c == 1.0).then_some((f, t.cmp, plain(sk, t)?))
}

/// The faces a set is the points on (`{ q | q coincident a; q coincident b }`): each term a
/// distance from one face (or its surface) at most nothing.
fn on_all<'a>(sk: &Sketch, terms: &'a [RegionTerm]) -> Option<Vec<&'a FaceRef>> {
    terms.iter().map(|t| match one_face(sk, t)? {
        (f, Cmp::Le, d) if d == 0.0 => Some(f),
        _ => None,
    }).collect()
}

/// The faces a meeting is on the surfaces of (`{ q | q coincident a.surface; q coincident
/// b.surface }`), cut or not where their edge ends (`q inside crease.ends`, std's: a run carried
/// on past its end and cut off there, the same points as one ending square where a face cuts it
/// square — and `gate` holds the build to the field where one does not).
fn meeting_of<'a>(sk: &Sketch, terms: &'a [RegionTerm]) -> Option<Vec<&'a FaceRef>> {
    terms.iter().filter_map(|t| match &t.shape {
        RegionShape::Ends { edge } => match edge.as_ref() {
            Target::Set { terms: inner, .. } if on_all(sk, inner).is_some_and(|f| f.len() == 2)
                && t.cmp == Cmp::Le && plain(sk, t) == Some(0.0) => None,
            _ => Some(None),
        },
        _ => Some(match one_face(sk, t)? {
            (f, Cmp::Le, d) if d == 0.0 => Some(f),
            _ => None,
        }),
    }).collect()
}

/// The face a term bounds the side of at nothing, and which way: `p inside f` or `p outside f`.
fn side_of<'a>(sk: &Sketch, t: &'a RegionTerm) -> Option<(&'a FaceRef, Cmp)> {
    let RegionShape::FaceSide { face } = &t.shape else { return None };
    (!face.surface && t.cmp != Cmp::In && plain(sk, t)? == 0.0).then_some((face, t.cmp))
}

/// The faces and the section a region's terms describe, where they are one of this table's: the
/// faces' sides, the material's angle at each edge between two of them against a half turn, the
/// nearest point of the faces' meeting on their common part, each edge's ends, and one bound
/// saying the section. A term no entry reads is no entry's region.
fn recognised<'a>(sk: &Sketch, terms: &'a [RegionTerm]) -> Option<(Faces<'a>, Section)> {
    let sides: Vec<(&FaceRef, Cmp)> = terms.iter().filter(|t| matches!(t.shape, RegionShape::FaceSide { .. }))
        .map(|t| side_of(sk, t)).collect::<Option<_>>()?;
    let cmp = sides.first()?.1;
    let faces: Vec<&FaceRef> = sides.iter().map(|s| s.0).collect();
    let distinct = faces.iter().enumerate()
        .all(|(i, f)| faces[..i].iter().all(|g| g.path != f.path) && f.solid == faces[0].solid);
    if !(2..=3).contains(&faces.len()) || sides.iter().any(|s| s.1 != cmp) || !distinct {
        return None;
    }
    let faces = Faces { faces, within: cmp == Cmp::Le };
    let n = faces.pairs().len();
    let (mut angles, mut ends, mut slab, mut section) = (vec![0; n], vec![0; n], 0, None);
    for t in terms {
        match &t.shape {
            RegionShape::FaceSide { .. } => {}
            RegionShape::Angle { a, b } => {
                let half = plain(sk, t).is_some_and(|lo| (lo - std::f64::consts::PI).abs() <= 1e-12);
                if !half || t.cmp != cmp {
                    return None;
                }
                angles[faces.pair_of([a, b], false)?.0] += 1;
            }
            RegionShape::Ends { edge } => {
                let Target::Set { terms: inner, .. } = edge.as_ref() else { return None };
                let [x, y] = on_all(sk, inner)?[..] else { return None };
                if t.cmp != Cmp::Le || plain(sk, t)? != 0.0 {
                    return None;
                }
                ends[faces.pair_of([x, y], false)?.0] += 1;
            }
            RegionShape::Measured { of } => match &of[..] {
                // nearer the faces' common part than wherever else their surfaces meet
                [(Target::Set { terms: x, .. }, cx), (Target::Set { terms: y, .. }, cy)] => {
                    let (common, meeting) = if *cx > 0.0 { (x, y) } else { (y, x) };
                    if (cx + cy).abs() > 1e-12 * cx.abs() || t.cmp != Cmp::Le || plain(sk, t)? != 0.0
                        || !faces.all(&on_all(sk, common)?, false) || !faces.all(&meeting_of(sk, meeting)?, true) {
                        return None;
                    }
                    slab += 1;
                }
                // beyond reach of the ball touching every face: where its centre may stand
                [(Target::Set { terms: centres, .. }, c)] => {
                    let r = plain(sk, t)?;
                    if *c != 1.0 || t.cmp != Cmp::Ge || !(r > 0.0) || section.is_some()
                        || !ball_centres(sk, centres, &faces, r) {
                        return None;
                    }
                    section = Some(Section::Round(r));
                }
                // short of a flat: the faces' distances, each over the other's leg, at most one
                [(Target::Face(x), kx), (Target::Face(y), ky)] if faces.faces.len() == 2 => {
                    let l = plain(sk, t)?;
                    if t.cmp != Cmp::Le || !(l > 0.0 && *kx > 0.0 && *ky > 0.0) || section.is_some() {
                        return None;
                    }
                    let flat = Section::Flat { la: l / ky, lb: l / kx };
                    section = Some(if faces.pair_of([x, y], true)?.1 { flat.swapped() } else { flat });
                }
                _ => return None,
            },
            _ => return None,
        }
    }
    (angles.iter().chain(&ends).all(|&k| k == 1) && slab == 1).then_some(())?;
    Some((faces, section?))
}

/// Whether a set is where the centre of a ball of radius `r` touching every face may stand: on
/// the region's side of each face, at least `r` from its surface.
fn ball_centres(sk: &Sketch, terms: &[RegionTerm], faces: &Faces, r: f64) -> bool {
    let (mut sides, mut reach) = (Vec::new(), Vec::new());
    for t in terms {
        match (side_of(sk, t), one_face(sk, t)) {
            (Some((f, c)), _) if (c == Cmp::Le) == faces.within => sides.push(f),
            (_, Some((f, Cmp::Ge, d))) if (d - r).abs() <= 1e-12 * r => reach.push(f),
            _ => return false,
        }
    }
    faces.all(&sides, false) && faces.all(&reach, true)
}

/// Why an entry that recognised a region does not build it.
fn unbuilt<T>(why: &str) -> Result<T, String> {
    Err(format!("{why}: no exact construction is made of it yet"))
}

/// From a face toward a region (its outward normal `n`): into the material at a convex edge, the
/// void at a concave one.
fn toward(n: V, within: bool) -> V { if within { scale(n, -1.0) } else { n } }

/// The region's faces read off its body (the stratum), and the pieces it is (none: it holds
/// nowhere).
fn build(body: &super::faces::BodyFaces, f: &Faces, section: Section) -> Result<Vec<Piece>, String> {
    // each reference's faces: one, or a face another operand split in pieces (a plate's top a rod
    // is sunk along); one a boolean ate, or none of that name, holds nowhere
    let mut groups = Vec::new();
    for r in &f.faces {
        let fs = body.faces(r);
        if fs.is_empty() {
            return Ok(Vec::new());
        }
        groups.push(fs);
    }
    let mut pieces = Vec::new();
    for &[i, j] in f.pairs() {
        let ps = run(body, &groups[i], &groups[j], f.within, section)?;
        if ps.is_empty() {
            return Ok(Vec::new());
        }
        pieces.extend(ps);
    }
    let (Section::Round(r), 3) = (section, groups.len()) else { return Ok(pieces) };
    let one = |g: &Vec<usize>| match g[..] { [fi] => Some(fi), _ => None };
    let Some(ids) = groups.iter().map(one).collect::<Option<Vec<usize>>>() else {
        return unbuilt("a corner of a face in pieces");
    };
    // three faces: the ball touching all three, at each vertex they share (three planes meet at
    // one; a D's flat, its round and a cap at both ends of the flat's edge)
    let vertices = body.vertices_among(&[&[ids[0]], &[ids[1]], &[ids[2]]]);
    if vertices.is_empty() {
        return Ok(Vec::new());
    }
    let corners = vertices.iter().map(|&vertex| corner_at(body, &ids, vertex, f.within, r))
        .collect::<Result<Vec<_>, _>>()?;
    // three planes: every run within each edge's ends, so the region is their union and nothing
    // cuts them
    let planar = |c: &Corner| c.faces.iter().all(|s| matches!(s, Surf::Plane { .. }));
    if let [(corner, toward)] = &corners[..] {
        if !planar(corner) {
            pieces.extend(corners.into_iter().map(|(c, _)| Piece::Corner(c)));
            return Ok(pieces);
        }
        let mut bounds = Vec::new();
        for &[i, j] in f.pairs() {
            let es = body.edges_between(&[ids[i]], &[ids[j]]);
            bounds.extend(ends_at_all(body, &body.end_sides(&es, &[ids[i], ids[j]])));
        }
        let runs: Vec<&Run> = pieces.iter().filter_map(|p| match p { Piece::Run(r) => Some(r), _ => None }).collect();
        let size = runs.iter().map(|r| r.span[1] - r.span[0]).fold(r, f64::max);
        // (a run cut off at a slant by an end is within that end already)
        let within = |p: V, own: &[(V, f64)]| bounds.iter().filter(|b| !own.contains(b))
            .all(|&(n, d)| dot(n, p) <= d + 1e-9 * size);
        let (feet, touches) = crate::brep::fillet::corner_points(corner.vertex, corner.centre, r, *toward);
        let cell = [corner.vertex, corner.centre].into_iter().chain(feet).chain(touches);
        let runs_within = runs.iter().all(|r| r.hull().into_iter().all(|p| within(p, &r.within)));
        if !runs_within || !cell.into_iter().all(|p| within(p, &[])) {
            return unbuilt("a ball larger than the edges it rounds");
        }
    }
    pieces.extend(corners.into_iter().map(|(c, _)| Piece::Corner(c)));
    Ok(pieces)
}

/// The corner at `vertex` where the faces `ids` meet — planes, or a cylinder or a sphere among
/// them (#218's 4) — and the way from each face toward its ball. The ball stands `r` from each
/// surface on the region's side (`within`, the material's): from where it would stand were each
/// face its tangent plane at the vertex (a 3×3 solve: there for three planes), by Newton's steps
/// on the three signed distances. The cell is the ball's normal cone cut by the faces, so the
/// vertex stands in that cone, back from the centre along each face's normal: `c − v = Σ μᵢ tᵢ`,
/// every `μᵢ > 0` (a corner whose shrunk body loses the vertex's face there — a sliver of a face
/// between two nearly alike — is no ball's cell).
fn corner_at(body: &super::faces::BodyFaces, ids: &[usize], vertex: V, within: bool, r: f64)
    -> Result<(Corner, [V; 3]), String> {
    let faces: Vec<Surf> = ids.iter().map(|&fi| Surf::of(body, fi)
        .ok_or_else(|| "a corner where a face neither a plane, a cylinder, a cone nor a sphere meets two others".to_string()))
        .collect::<Result<_, _>>().or_else(|e| unbuilt(&e))?;
    let faces = [faces[0], faces[1], faces[2]];
    let tangents: [V; 3] = std::array::from_fn(|i| toward(body.normal(ids[i], vertex), within));
    if dot(tangents[0], cross(tangents[1], tangents[2])).abs() <= 1e-9 {
        return unbuilt("a corner whose faces share a direction");
    }
    // the signed distance from each surface (positive away from the material) the centre stands at
    let want = if within { -r } else { r };
    let size = r + norm(vertex);
    let start = add(vertex, crate::brep::ssi::solve3(tangents, [r; 3]));
    let centre = crate::roots::newton_onto::<3>(start, r,
        |c| Some(std::array::from_fn(|i| { let (s, g) = body.side_grad(ids[i], c); (g, s - want) })),
        |at| at.iter().all(|(_, m)| m.abs() <= 1e-13 * size))
        .map_or_else(|| unbuilt("a corner where no ball touches all three faces"), Ok)?;
    let toward: [V; 3] = std::array::from_fn(|i| toward(body.side_grad(ids[i], centre).1, within));
    let columns = std::array::from_fn(|i| [toward[0][i], toward[1][i], toward[2][i]]);
    if crate::brep::ssi::solve3(columns, sub(centre, vertex)).iter().any(|&m| !(m > 1e-9)) {
        return unbuilt("a corner whose ball stands beyond one of its faces' edges");
    }
    Ok((Corner { vertex, centre, r, faces }, toward))
}

/// The run along the edges where the faces `ga` (a reference's) meet the faces `gb`: a piece per
/// edge, each where the edge turns the side the region asks (`within`, convex) — none where they
/// meet in none — and all within where the run of them ends, as the field reads it.
fn run(body: &super::faces::BodyFaces, ga: &[usize], gb: &[usize], within: bool, section: Section)
    -> Result<Vec<Piece>, String> {
    let edges = body.edges_between(ga, gb);
    // an edge no line or circle about the faces' common axis carries: the ball rolled along the
    // whole run of them, traced
    let carried = |e: u32| body.segment(e).is_some() || body.circle(e).is_some_and(|(centre, axis, ..)| {
        body.faces_of(e).iter().all(|&f| body.coaxial(f, centre, axis))
    });
    if let Some(&e) = edges.iter().find(|&&e| !carried(e)) {
        let Section::Round(r) = section else { return unbuilt("a flat along a traced edge") };
        return Ok(if body.convex(e) == Some(within) { vec![Piece::Rolled(Rolled { at: body.middle(e), r })] }
            else { Vec::new() });
    }
    let all: Vec<usize> = ga.iter().chain(gb).copied().collect();
    let ends = body.end_sides(&edges, &all);
    let mut pieces = Vec::new();
    for &edge in &edges {
        let &[f0, f1] = body.faces_of(edge) else { return unbuilt("an edge not between two faces") };
        let (fa, fb) = if ga.contains(&f0) { (f0, f1) } else { (f1, f0) };
        pieces.extend(piece(body, edge, fa, fb, within, section, &ends)?);
    }
    Ok(pieces)
}

/// The piece along `edge`, where faces `fa` and `fb` meet — a straight one's section swept along
/// it, a circle's turned about its axis — kept within the half-spaces of the run's `ends` standing
/// at its own ends and reaching into it; `None` where it turns the other way from the side the
/// region asks (`within`, convex).
fn piece(body: &super::faces::BodyFaces, edge: u32, fa: usize, fb: usize, within: bool, section: Section,
    ends: &[(usize, f64)]) -> Result<Option<Piece>, String> {
    // the faces in the body's order, so two regions over one edge build one run
    let (fa, fb, section) = if fb < fa { (fb, fa, section.swapped()) } else { (fa, fb, section) };
    // the gate: an edge turning the other way, or none (faces meeting tangent), holds nowhere
    if body.convex(edge) != Some(within) {
        return Ok(None);
    }
    // round a circle where a plane, a cylinder, a cone or a sphere meet about one axis: the section
    // in the meridian through its start, turned through the circle's angle — a whole turn, or a
    // stretch ending where a face goes on from one of these tangent (an obround's flat side), whose
    // own run goes on from the same section
    // (`run` rolls a circle whose faces are not all about its axis)
    if let Some((centre, axis, radius, x)) = body.circle(edge) {
        let closed = body.closed(edge);
        let [t0, t1] = if closed { [0.0, std::f64::consts::TAU] } else { body.span(edge) };
        let radial_at = |t: f64| crate::brep::geom::Rigid::turn([0.0; 3], axis, t).vector(x);
        let [p0, p1] = [t0, t1].map(|t| add(centre, scale(radial_at(t), radius)));
        // a stretch ends at seams, or at a plane through the axis standing where its turn starts
        // or stops (a groove's rim at the plate's top): that end's own meridian, the sector's
        let meridian = |&(n, d): &(V, f64)| dot(n, axis).abs() <= 1e-9
            && (dot(n, centre) - d).abs() <= 1e-9 * (1.0 + norm(centre))
            && dot(n, body.middle(edge)) < d;
        // in the meridian a plane, a cylinder and a cone are lines, a sphere a circle about its
        // centre (on the axis, so in every meridian)
        let trace = |fi: usize, p: V| match body.sphere(fi) {
            Some((centre, radius)) => Trace::Circle { centre, radius, normal: body.normal(fi, p) },
            None => Trace::Line(body.normal(fi, p)),
        };
        let section_at = |t: f64| {
            let p = add(centre, scale(radial_at(t), radius));
            cut(p, cross(axis, radial_at(t)), [trace(fa, p), trace(fb, p)], within, section)
        };
        // at an end standing on a plane not through the axis (a D's round rim ending at its flat):
        // carried on round past it until the whole section has crossed that plane, and kept within
        // it (#218's 2) — each point of the section's hull turned to its first crossing (`n·x`
        // along a turn is a sinusoid: its roots in closed form); at a cylinder or a ball it runs
        // into, a quarter of the way into what every point spends in that solid (a point turning
        // round a circle may come back out of it, and the other end may run into the same solid
        // from its other side)
        let (mut from, mut to, mut halves, mut kepts) = (t0, t1, Vec::new(), Vec::new());
        if !closed {
            for (k, p, t) in [(0, p0, t0), (1, p1, t1)] {
                let (planes, bent) = ends_at(body, ends, p)?;
                let planes: Vec<(V, f64)> = planes.into_iter().filter(|h| !meridian(h)).collect();
                if planes.is_empty() && bent.is_empty() {
                    continue;
                }
                let way = if k == 0 { -1.0 } else { 1.0 };
                let hull = section_at(t)?.section_hull();
                let turned = |q: V, phi: f64| crate::brep::geom::Rigid::turn(centre, axis, way * phi).point(q);
                let mut reach: f64 = 0.0;
                for &(n, d) in &planes {
                    for &q in &hull {
                        let w = sub(q, centre);
                        let rim = sub(w, scale(axis, dot(w, axis)));
                        let (a, b) = (dot(n, rim), way * dot(n, cross(axis, rim)));
                        let c = dot(n, sub(q, rim)) - d;
                        reach = reach.max(first_over(a, b, c)
                            .ok_or_else(|| unbuilt::<()>("a stretch of a circle its end does not cut off").unwrap_err())?);
                    }
                }
                for kept in &bent {
                    let (mut entry, mut exit) = (0.0_f64, std::f64::consts::TAU);
                    for &q in &hull {
                        let [first, back] = dwell(|phi| kept.away(turned(q, phi)))
                            .ok_or_else(|| unbuilt::<()>("a stretch of a circle its curved end does not cut off").unwrap_err())?;
                        entry = entry.max(first);
                        exit = exit.min(back);
                    }
                    if !(entry < exit) {
                        return unbuilt("a stretch of a circle its curved end does not cut off");
                    }
                    reach = reach.max(entry + 0.25 * (exit - entry));
                }
                if k == 0 { from -= reach } else { to += reach }
                halves.extend(planes);
                kepts.extend(bent);
            }
            if to - from >= std::f64::consts::TAU {
                return unbuilt("a stretch of a circle carried on round its whole turn");
            }
        }
        let mut section = section_at(from)?;
        section.within = halves;
        section.kept = kepts;
        return Ok(Some(Piece::Ring(Ring { section, axis_origin: centre, axis, angle: to - from })));
    }
    let Some([p0, p1]) = body.segment(edge) else { return unbuilt("a curved edge") };
    let length = norm(sub(p1, p0));
    let along = scale(sub(p1, p0), 1.0 / length);
    // each face as the section sees it: a plane its line, a cylinder about a line along the edge
    // its circle
    let trace = |fi: usize| -> Result<Trace, String> {
        if let Some((n, _)) = body.plane(fi) {
            return Ok(Trace::Line(n));
        }
        match body.cylinder(fi) {
            Some((o, d, radius)) if norm(cross(d, along)) <= 1e-9 => Ok(Trace::Circle {
                centre: add(o, scale(d, dot(sub(p0, o), d))), radius, normal: body.normal(fi, p0),
            }),
            _ => unbuilt("an edge beside a curved face not turned about a line along it"),
        }
    };
    let traces = [trace(fa)?, trace(fb)?];
    // carried on past each end until every face there, on the side the edge runs on, cuts it
    // off, however it stands (a run-out): a face square to the edge where the edge ends, one at a
    // slant as far past as the section reaches, kept within its half-space
    let (planes0, bent0) = ends_at(body, ends, p0)?;
    let (planes1, bent1) = ends_at(body, ends, p1)?;
    let mut run = cut(p0, along, traces, within, section)?;
    let outline = run.section_hull();
    run.span = [0.0, length];
    let mut halves = planes0;
    for h in planes1 {
        if !halves.contains(&h) {
            halves.push(h);
        }
    }
    for &(n, d) in &halves {
        let slope = dot(n, along);
        if slope.abs() < 1e-6 {
            return unbuilt("a run carried on along the face it ends on");
        }
        for &q in &outline {
            let s = (d - dot(n, q)) / slope;
            if slope > 0.0 { run.span[1] = run.span[1].max(s) } else { run.span[0] = run.span[0].min(s) }
        }
    }
    // a curved face it ends on (#218's 2), a cylinder or a ball: carried on into it and kept on the
    // side the edge runs on — until every line along the prism through its section has left the
    // side kept. Into a solid kept outside of: as far as the deepest entry, short of every exit,
    // so the prism's end lies wholly within it (the entry over the section is convex, the exit
    // concave: each reads its extreme at the hull's points). Out of one kept within: past every
    // exit and the section's width more (a line leaving a convex solid never comes back). A D's
    // flat ends on its round at both ends.
    let width = outline.iter().map(|&q| distance(q, p0)).fold(0.0, f64::max);
    for (forward, bent) in [(false, bent0), (true, bent1)] {
        // each line's crossings, as far along the way it runs on from the edge's start
        let way = if forward { along } else { scale(along, -1.0) };
        for kept in bent {
            let lines: Vec<Option<[f64; 2]>> = outline.iter().map(|&q| kept.face.crossings(q, way)).collect();
            let to = if kept.inside {
                let from = if forward { length } else { 0.0 };
                lines.iter().flatten().map(|l| l[1]).fold(from, f64::max) + 2.0 * width
            } else {
                let deepest = lines.iter().flatten().map(|l| l[0]).fold(f64::NEG_INFINITY, f64::max);
                if !lines.iter().all(|l| l.is_some_and(|l| deepest <= l[1])) {
                    return unbuilt("a run its curved end does not cut off");
                }
                deepest
            };
            if forward { run.span[1] = run.span[1].max(to) } else { run.span[0] = run.span[0].min(-to) }
            if !run.kept.contains(&kept) {
                run.kept.push(kept);
            }
        }
    }
    // only those reaching into the prism: a square end is the prism's own
    let hull = run.hull();
    run.within = halves.into_iter()
        .filter(|&(n, d)| hull.iter().any(|&p| dot(n, p) > d + 1e-9 * (1.0 + length))).collect();
    Ok(Some(Piece::Run(run)))
}

/// The faces of a run's `ends` (`end_sides`' faces and the sides it runs on) standing at `p`, one
/// end of an edge of it: each plane's half-space `n·x <= d` it is kept within, and each cylinder or
/// ball it is kept within (where that side holds its axis, its centre) or outside of.
fn ends_at(body: &super::faces::BodyFaces, ends: &[(usize, f64)], p: V) -> Result<(Vec<(V, f64)>, Vec<Kept>), String> {
    let (mut planes, mut bent) = (Vec::new(), Vec::new());
    for &(c, sign) in ends.iter().filter(|&&(c, _)| body.side(c, p).abs() <= 1e-7 * (1.0 + norm(p))) {
        match Surf::of(body, c) {
            Some(Surf::Plane { origin, normal }) => planes.push((scale(normal, sign), sign * dot(normal, origin))),
            Some(face) => {
                let (Surf::Cylinder { origin: on, .. } | Surf::Cone { origin: on, .. } | Surf::Sphere { centre: on, .. }
                    | Surf::Plane { origin: on, .. }) = face;
                bent.push(Kept { face, inside: sign * body.side(c, on) < 0.0 });
            }
            None => return unbuilt("an edge cut off by a face neither a plane, a cylinder, a cone nor a sphere"),
        }
    }
    Ok((planes, bent))
}

/// Along a turn from where `over` is nothing or less, where it first goes over nothing and where
/// it next comes back to nothing (a whole turn where it never does): sampled at a 720th of a turn,
/// each crossing refined; none where it never goes over.
fn dwell(over: impl Fn(f64) -> f64) -> Option<[f64; 2]> {
    let step = std::f64::consts::TAU / 720.0;
    let values: Vec<f64> = (0..=720).map(|j| over(j as f64 * step)).collect();
    let root = |j: usize| {
        let (a, b) = ((j - 1) as f64 * step, j as f64 * step);
        crate::roots::bracketed_root(|x| Some(over(x)), a, values[j - 1], b, values[j], 1e-15)
    };
    let j = values.iter().position(|&v| v > 0.0)?;
    let first = if j == 0 { 0.0 } else { root(j) };
    let back = (j + 1..=720).find(|&i| values[i] <= 0.0).map_or(std::f64::consts::TAU, root);
    Some([first, back])
}

/// Of `a cos φ + b sin φ + c` over `φ >= 0`, where it first goes over nothing (at once where it
/// starts over); none where it never does.
fn first_over(a: f64, b: f64, c: f64) -> Option<f64> {
    if a + c > 0.0 {
        return Some(0.0);
    }
    let amplitude = a.dhypot(b);
    if amplitude <= -c {
        return None;
    }
    let (phase, spread) = (b.datan2(a), (-c / amplitude).dacos());
    Some([phase - spread, phase + spread].map(|x| x.rem_euclid(std::f64::consts::TAU)).into_iter().fold(f64::INFINITY, f64::min))
}

/// The half-spaces `n·x <= d` of the planes among a run's `ends` (`end_sides`' faces and the
/// sides it runs on).
fn ends_at_all(body: &super::faces::BodyFaces, ends: &[(usize, f64)]) -> Vec<(V, f64)> {
    ends.iter().filter_map(|&(c, sign)| body.plane(c).map(|(n, o)| (scale(n, sign), sign * dot(n, o)))).collect()
}

/// A face as a section square to the edge sees it at the edge's point: a line, its unit normal
/// away from the material; or a circle about `centre` (in the section), its normal there.
#[derive(Clone, Copy, Debug)]
enum Trace {
    Line(V),
    Circle { centre: V, radius: f64, normal: V },
}

impl Trace {
    fn normal(self) -> V { match self { Trace::Line(n) | Trace::Circle { normal: n, .. } => n } }

    /// From `from` to `to` along the face (both on it): a line, or the shorter arc of the circle.
    fn stroke(self, from: V, to: V, along: V) -> Stroke {
        match self {
            Trace::Line(_) => Stroke::Line { from, to },
            Trace::Circle { centre, radius, .. } => arc(centre, radius, from, to, along),
        }
    }
}

/// The shorter arc about `centre` from `from` to `to`, turning about the unit `along`.
fn arc(centre: V, radius: f64, from: V, to: V, along: V) -> Stroke {
    let x = scale(sub(from, centre), 1.0 / radius);
    let to = sub(to, centre);
    let phi = dot(to, cross(along, x)).datan2(dot(to, x));
    Stroke::Arc { centre, radius, x, angles: if phi > 0.0 { [0.0, phi] } else { [phi, 0.0] } }
}

/// The section's corner as `brep::corners` reads a meridian's: in the plane square to `along` at
/// `p0` (coordinates along `x` and `along × x`, so a turn to the left is one about `along`), face
/// `a`'s trace a step arriving at the corner and `b`'s one leaving it, each `reach` long, and the
/// side the region lies on (+1 the left of travel). A round or a flat there is then what it is in a
/// meridian, by the same rule (`corners::ball`, `corners::sum`): between lines and circles, a
/// circle's flat the conic its sum of distances makes.
struct Corner2 {
    p0: V,
    x: V,
    y: V,
    steps: [crate::brep::planar::Seg; 2],
    side: f64,
}

impl Corner2 {
    fn new(p0: V, along: V, traces: [Trace; 2], [ua, ub]: [V; 2], reach: f64) -> Result<Corner2, String> {
        use crate::brep::planar::Seg;
        let x = normalised(sub(ua, scale(along, dot(ua, along)))).ok_or("a face square to the edge")?;
        let y = cross(along, x);
        let flat = |p: V| [dot(sub(p, p0), x), dot(sub(p, p0), y)];
        // each face's step: from the edge along the face away from the other (toward the region
        // from it), its arc no longer than a quarter turn
        let step = |t: Trace, u: V| -> Result<Seg, String> {
            let n = t.normal();
            let e = normalised(sub(u, scale(n, dot(u, n)))).ok_or("faces lying in one plane")?;
            Ok(match t {
                Trace::Line(_) => Seg::Line { a: [0.0; 2], b: flat(add(p0, scale(e, reach))) },
                Trace::Circle { centre, radius, .. } => {
                    let c = flat(centre);
                    let a0 = (-c[1]).datan2(-c[0]);
                    let way = dot(cross(sub(p0, centre), e), along).signum();
                    Seg::Arc { c, r: radius, a0, sweep: way * (reach / radius).min(0.5 * std::f64::consts::PI) }
                }
            })
        };
        let (sa, sb) = (step(traces[0], ub)?, step(traces[1], ua)?);
        // the region's side, left of travel arriving along `a`: `a`'s way in is the step's way out
        let t = sa.tangent(0.0);
        let toward = flat(add(p0, add(ua, ub)));
        let side = if -t[1] * toward[0] + t[0] * toward[1] < 0.0 { 1.0 } else { -1.0 };
        Ok(Corner2 { p0, x, y, steps: [sa.reversed(), sb], side })
    }

    fn lift(&self, q: [f64; 2]) -> V { add(self.p0, add(scale(self.x, q[0]), scale(self.y, q[1]))) }

    /// The ball of radius `r` in the corner: its centre, and where it touches `a` and `b`.
    fn ball(&self, r: f64) -> Result<(V, V, V), String> {
        let (ta, tb, arc) = crate::brep::corners::ball(&self.steps[0], &self.steps[1], self.side, r)
            .or_else(|e| unbuilt(&e))?;
        let crate::brep::planar::Seg::Arc { c, .. } = arc else { unreachable!("a ball's arc") };
        Ok((self.lift(c), self.lift(self.steps[0].point(ta)), self.lift(self.steps[1].point(tb))))
    }

    /// The flat with legs `la` on `a` and `lb` on `b`, each from the other face's surface: its ends
    /// on `a` and `b` and the stroke between them.
    fn flat(&self, [la, lb]: [f64; 2]) -> Result<(V, V, Stroke), String> {
        use crate::brep::planar::Seg;
        let (ta, tb, cut) = crate::brep::corners::sum(&self.steps[0], &self.steps[1], self.side, [la, lb])
            .or_else(|e| unbuilt(&e))?;
        let (qa, qb) = (self.lift(self.steps[0].point(ta)), self.lift(self.steps[1].point(tb)));
        Ok((qa, qb, match cut {
            Seg::Conic { m, w, .. } => Stroke::Conic { a: qa, m: self.lift(m), b: qb, w },
            _ => Stroke::Line { from: qa, to: qb },
        }))
    }
}

/// The section beside an edge at `p0`, in the plane square to the unit `along` there (its stretch
/// nothing): between faces seen there as `traces` (normals away from the material, each square to
/// `along`), on the region's side of both (`within`: the material's).
fn cut(p0: V, along: V, traces: [Trace; 2], within: bool, section: Section) -> Result<Run, String> {
    let (ua, ub) = (toward(traces[0].normal(), within), toward(traces[1].normal(), within));
    if 1.0 + dot(ua, ub) <= 1e-9 {
        return unbuilt("faces folded back on one another");
    }
    let size = match section { Section::Round(r) => r, Section::Flat { la, lb } => la.max(lb) };
    let corner = Corner2::new(p0, along, traces, [ua, ub], 8.0 * size)?;
    let (ta, middle, tb) = match section {
        // the ball's centre `r` from both faces, and where it touches each
        Section::Round(r) => {
            let (o, ta, tb) = corner.ball(r)?;
            (ta, (arc(o, r, ta, tb, along), "round"), tb)
        }
        // along each face away from the edge, to where it stands its leg from the other face
        Section::Flat { la, lb } => {
            let (qa, qb, flat) = corner.flat([la, lb])?;
            (qa, (flat, "flat"), qb)
        }
    };
    let strokes = vec![(traces[0].stroke(p0, ta, along), "a"), middle, (traces[1].stroke(tb, p0, along), "b")];
    Ok(Run { origin: p0, along, span: [0.0; 2], strokes, within: Vec::new(), kept: Vec::new() })
}

/// What a piece is kept within at its ends, as a recipe reads it (millimetres, `by` of them a model
/// unit): each plane's half-space, `within` (`at` on it, its `normal` out of it), and each curved
/// face's solid, `kept` (its surface, and whether `inside`).
fn ends_json(r: &Run, by: f64) -> [(&'static str, Json); 2] {
    use super::cad::vector;
    let within = r.within.iter().map(|&(n, d)| object([("at", vector(scale(n, d * by))), ("normal", vector(n))])).collect();
    let kept = r.kept.iter().map(|k| { let mut o = k.face.json(by); o.set("inside", k.inside.into()); o }).collect();
    [("within", Json::Arr(within)), ("kept", Json::Arr(kept))]
}

/// A run's section moved by `m` (a turn about a ring's axis): its strokes, its origin and way; the
/// faces it is kept within stand where they stood.
fn turned(r: &Run, m: &crate::brep::geom::Rigid) -> Run {
    let stroke = |s: &Stroke| match *s {
        Stroke::Line { from, to } => Stroke::Line { from: m.point(from), to: m.point(to) },
        Stroke::Arc { centre, radius, x, angles } => Stroke::Arc { centre: m.point(centre), radius, x: m.vector(x), angles },
        Stroke::Conic { a, m: mid, b, w } => Stroke::Conic { a: m.point(a), m: m.point(mid), b: m.point(b), w },
    };
    Run {
        origin: m.point(r.origin), along: m.vector(r.along), span: r.span,
        strokes: r.strokes.iter().map(|(s, name)| (stroke(s), *name)).collect(),
        within: r.within.clone(),
        kept: r.kept.clone(),
    }
}

/// A run's section as a profile `brep::build` reads (model units): a conic's arc the rational
/// quadratic B-spline it is.
fn profile(r: &Run) -> Result<crate::brep::build::Profile, String> {
    use crate::brep::build::ProfileEdge;
    use crate::brep::geom::{BSpline, Frame};
    let (edges, names) = r.strokes.iter().map(|(stroke, name)| Ok((match *stroke {
        Stroke::Line { from, to } => ProfileEdge::Line { a: from, b: to },
        Stroke::Arc { centre, radius, x, angles } => ProfileEdge::Arc {
            frame: Frame::new(centre, r.along, x), r: radius, span: Some(angles),
        },
        Stroke::Conic { a, m, b, w } => ProfileEdge::Spline(std::sync::Arc::new(
            BSpline::rational(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], vec![a, m, b], vec![1.0, w, 1.0])?)),
    }, name.to_string()))).collect::<Result<Vec<_>, String>>()?.into_iter().unzip();
    let (loops, names) = (vec![edges], vec![names]);
    Ok(crate::brep::build::Profile { origin: r.origin, normal: r.along, loops, names })
}

/// Pieces as one construction: each distinct piece once, each run with the stretch of its edge it
/// is built over (cut short at the section of a corner standing at either end), and which run is
/// joined to which corner there (runs counted first, then rings, then corners).
#[derive(Clone, Debug, Default)]
pub struct Assembly {
    pub runs: Vec<Run>,
    pub rings: Vec<Ring>,
    pub corners: Vec<Corner>,
    pub joins: Vec<[usize; 2]>,
    pub rolled: Vec<Rolled>,
}

/// The union of `pieces` as one construction (`Assembly`).
pub fn assemble<'a>(pieces: impl IntoIterator<Item = &'a Piece>) -> Assembly {
    let pieces: Vec<&Piece> = pieces.into_iter().collect();
    let size = pieces.iter().map(|p| match p {
        Piece::Run(r) => norm(r.origin) + r.span[0].abs().max(r.span[1].abs()),
        Piece::Ring(g) => norm(g.section.origin) + norm(g.axis_origin),
        Piece::Corner(c) => norm(c.vertex) + c.r,
        Piece::Rolled(k) => norm(k.at) + k.r,
    }).fold(1.0, f64::max);
    let near = |p: V, q: V| distance(p, q) <= 1e-9 * size;
    let same_stroke = |a: &Stroke, b: &Stroke| match (a, b) {
        (Stroke::Line { from: a0, to: a1 }, Stroke::Line { from: b0, to: b1 }) => near(*a0, *b0) && near(*a1, *b1),
        (Stroke::Arc { centre: c, radius: r, .. }, Stroke::Arc { centre: d, radius: s, .. }) => {
            near(*c, *d) && (r - s).abs() <= 1e-9 * size
        }
        (Stroke::Conic { a: a0, m: m0, b: b0, w: w0 }, Stroke::Conic { a: a1, m: m1, b: b1, w: w1 }) => {
            near(*a0, *a1) && near(*m0, *m1) && near(*b0, *b1) && (w0 - w1).abs() <= 1e-9 * w0.max(1.0)
        }
        _ => false,
    };
    let same_run = |a: &Run, b: &Run| near(a.origin, b.origin) && near(a.along, b.along)
        && a.span.iter().zip(b.span).all(|(s, t)| (s - t).abs() <= 1e-9 * size) && a.strokes.len() == b.strokes.len()
        && a.strokes.iter().zip(&b.strokes).all(|((x, _), (y, _))| same_stroke(x, y))
        && a.within.len() == b.within.len()
        && a.within.iter().zip(&b.within).all(|(x, y)| near(x.0, y.0) && (x.1 - y.1).abs() <= 1e-9 * size)
        && a.kept == b.kept;
    let mut out = Assembly::default();
    for p in pieces {
        match p {
            Piece::Run(r) if !out.runs.iter().any(|q| same_run(q, r)) => out.runs.push(r.clone()),
            Piece::Ring(g) if !out.rings.iter().any(|h| same_run(&h.section, &g.section)
                && (h.angle - g.angle).abs() <= 1e-12) => out.rings.push(g.clone()),
            Piece::Corner(c) if !out.corners.iter().any(|d| near(c.vertex, d.vertex) && near(c.centre, d.centre)) => {
                out.corners.push(c.clone())
            }
            Piece::Rolled(k) if !out.rolled.iter().any(|j| near(j.at, k.at) && (j.r - k.r).abs() <= 1e-9 * size) => {
                out.rolled.push(k.clone());
            }
            _ => {}
        }
    }
    // a round's run, at an end of its edge where a corner stands whose ball its own rolls into,
    // runs from the corner's section on: what lies between is the corner's cell (and a face the
    // run was carried on past and cut off by at a slant there no longer reaches it)
    let n = out.runs.len() + out.rings.len();
    for (i, r) in out.runs.iter_mut().enumerate() {
        let Some(o) = r.centre() else { continue };
        for (k, c) in out.corners.iter().enumerate() {
            let off = sub(c.centre, o);
            let at = dot(sub(c.vertex, r.origin), r.along);
            if norm(cross(off, r.along)) > 1e-9 * size || !near(c.vertex, add(r.origin, scale(r.along, at))) {
                continue;
            }
            // the run's origin is its edge's first end
            let s = dot(off, r.along);
            if at.abs() <= 1e-9 * size {
                r.span[0] = s;
            } else if at > 0.0 && at <= r.span[1] + 1e-9 * size {
                r.span[1] = s;
            } else {
                continue;
            }
            r.release(c.vertex, size);
            out.joins.push([i, n + k]);
        }
    }
    // a ring's stretch, at an end where a corner stands whose ball its own rolls round into, turns
    // only as far as the corner's section: the meridian through its centre (#218's 4)
    let (n, rings) = (out.runs.len(), out.rings.len());
    for (i, g) in out.rings.iter_mut().enumerate() {
        let Some(o) = g.section.centre() else { continue };
        if g.angle >= std::f64::consts::TAU - 1e-12 {
            continue;
        }
        let radial = |p: V| { let w = sub(p, g.axis_origin); sub(w, scale(g.axis, dot(w, g.axis))) };
        let height = |p: V| dot(sub(p, g.axis_origin), g.axis);
        for (k, c) in out.corners.iter().enumerate() {
            // the angle a point stands at round the axis from the stretch's start, the way it turns
            let from = radial(g.section.origin);
            let angle_of = |p: V| {
                let w = radial(p);
                dot(cross(from, w), g.axis).datan2(dot(from, w)).rem_euclid(std::f64::consts::TAU)
            };
            let on_balls = (norm(radial(c.centre)) - norm(radial(o))).abs() <= 1e-9 * size
                && (height(c.centre) - height(o)).abs() <= 1e-9 * size;
            let on_circle = (norm(radial(c.vertex)) - norm(from)).abs() <= 1e-9 * size
                && (height(c.vertex) - height(g.section.origin)).abs() <= 1e-9 * size;
            if !on_balls || !on_circle {
                continue;
            }
            let (at, cut) = (angle_of(c.vertex), angle_of(c.centre));
            if at <= 0.5 * g.angle {
                // at its start: turned on to the corner's section
                let turn = crate::brep::geom::Rigid::turn(g.axis_origin, g.axis, cut);
                g.section = turned(&g.section, &turn);
                g.angle -= cut;
            } else {
                g.angle = cut;
            }
            g.section.release(c.vertex, size);
            out.joins.push([n + i, n + rings + k]);
        }
    }
    // a run and a stretch of a ring going on from one another (a round along an obround's flat
    // side, then round its half circle): one section, where the ring's turn starts or ends
    for (i, r) in out.runs.iter().enumerate() {
        let ends = [r.end(0), r.end(1)];
        for (k, g) in out.rings.iter().enumerate().filter(|(_, g)| g.angle < std::f64::consts::TAU - 1e-12) {
            let (end, way) = g.end();
            let meets = |p: V, d: V| ends.iter().any(|&q| near(p, q)) && norm(cross(d, r.along)) <= 1e-9;
            if meets(g.section.origin, g.section.along) || meets(end, way) {
                out.joins.push([i, out.runs.len() + k]);
            }
        }
    }
    out
}

impl Assembly {
    /// As a recipe node (millimetres, `by` of them a model unit): each run a prism over its
    /// stretch (within the half-spaces it is kept in), each ring a revolution, each corner where it
    /// stands, and the joins.
    pub fn node(&self, by: f64) -> Result<Json, String> {
        use super::cad::vector;
        let mut pieces = self.runs.iter().map(|r| {
            let [from, to] = r.span;
            let mut o = object([
                ("kind", "prism".into()), ("profile", profile(r)?.to_json(by)?),
                ("from", (from * by).into()), ("to", (to * by).into()),
            ]);
            for (k, v) in ends_json(r, by) {
                o.set(k, v);
            }
            Ok(o)
        }).collect::<Result<Vec<_>, String>>()?;
        for g in &self.rings {
            let mut o = object([
                ("kind", "revolve".into()), ("profile", profile(&g.section)?.to_json(by)?),
                ("origin", vector(scale(g.axis_origin, by))), ("axis", vector(g.axis)), ("angle", g.angle.into()),
            ]);
            for (k, v) in ends_json(&g.section, by) {
                o.set(k, v);
            }
            pieces.push(o);
        }
        let corners = self.corners.iter().map(|c| object([
            ("vertex", vector(scale(c.vertex, by))), ("centre", vector(scale(c.centre, by))),
            ("radius", (c.r * by).into()), ("faces", Json::Arr(c.faces.iter().map(|f| f.json(by)).collect())),
        ])).collect();
        let joins = self.joins.iter().map(|j| Json::Arr(j.iter().map(|&k| k.into()).collect())).collect();
        let rolled = self.rolled.iter()
            .map(|k| object([("at", vector(scale(k.at, by))), ("radius", (k.r * by).into())])).collect();
        Ok(object([("kind", "lowered".into()), ("pieces", Json::Arr(pieces)), ("corners", Json::Arr(corners)),
            ("joins", Json::Arr(joins)), ("rolled", Json::Arr(rolled))]))
    }

    /// Built by this kernel (lengths times `by`).
    pub fn brep(&self, by: f64) -> Result<crate::brep::topo::Brep, String> {
        crate::brep::recipe::node_named(&self.node(by)?, &Default::default(), &Default::default(), 0.0)
    }
}

/// **The gate**: a lowering stands only where the region's own field agrees with it — every probe
/// a little inside the built boundary material by the field, every one a little outside not
/// (`agreement`, sampled by area); one holding nowhere, only where the field reads it empty too.
fn gate(sk: &Sketch, solid: usize, pieces: &[Piece], body: &super::faces::BodyFaces) -> Result<(), String> {
    if pieces.is_empty() {
        let empty = super::region::in_general_position(sk, solid)
            .is_some_and(|g| g.is_ok_and(|f| f.is_empty()));
        return if empty { Ok(()) } else {
            Err("the field reads material where the faces it reads say there is none".to_string())
        };
    }
    let field = super::MaterialField::read(sk, solid, super::cad::AXIS_TOLERANCE)?;
    // the pieces' boundary in model units: a rolled one rolled against the stratum (millimetres,
    // about its origin) and brought back
    // (`build` rolls a run alone: none stands beside other pieces)
    let (b, mm) = match pieces {
        [Piece::Rolled(k)] => {
            let (stratum, mm, _) = body.brep();
            let tol = 1e-9 * stratum.size().max(1.0);
            (crate::brep::fillet::roll(stratum, body.q(k.at), k.r * mm, tol)?.piece, Some(mm))
        }
        _ => (assemble(pieces).brep(1.0)?, None),
    };
    // probed `offset` either side of it, so meshed no finer than a twentieth of that
    let size = b.size().max(f64::MIN_POSITIVE);
    let mut offset = 2e-3 * size;
    let mut m = crate::brep::mesh::mesh(&b, offset / 20.0, crate::brep::export::ANGULAR)?;
    if let Some(mm) = mm {
        for p in &mut m.pts {
            *p = body.back(*p);
        }
        offset /= mm;
    }
    let options = super::agreement::Options {
        offset, confirm: offset / 4.0, value_tolerance: offset / 5.0, triangles: 400,
        ..Default::default()
    };
    let report = super::agreement::of_triangles_parallel(&m.pts, &m.tris, &field, 1, &options)?;
    match report.disagreements.first() {
        None => Ok(()),
        Some(d) => Err(format!("its exact construction disagrees with the field at [{:.6}, {:.6}, {:.6}] ({} of {} probes)",
            d.point[0], d.point[1], d.point[2], report.disagreements.len(), report.probes)),
    }
}
