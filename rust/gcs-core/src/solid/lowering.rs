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
//! two cylinders, about lines along the edge: in the section lines and circles), corners of any
//! three faces among planes, cylinders, cones and spheres (a face in pieces taking the piece at
//! each vertex), and planes, cylinders, cones, spheres and tori about one axis meeting in a circle;
//! a face named in pieces (a plate's top a rod splits) is a piece per edge. A run is where its
//! nearest point of the faces' meeting is on the edge (the slab square to it over its stretch, a
//! sector of a ring's) and on the side the edge runs on of each face at its ends: a straight run's
//! prism is kept within each plane face standing at a slant there and reaching into it (a run-out),
//! one leaning away leaving the slab's square end; one ending on a cylinder, a cone or a sphere is
//! carried on into it and kept on its side of that solid (`Kept`). A face going on tangent from
//! one of the run's is no end (a seam), so a round along an obround's flat side stops at the
//! section the ring round its half circle starts from, and the two are glued there; a ring's
//! stretch is turned through its circle's angle, and on round past an end on a plane not through
//! its axis until its section has crossed it, kept within it (or into a cylinder or a ball it ends
//! on, kept on its side of that solid). Along an edge no line or coaxial circle carries, a round's
//! ball is rolled and a chamfer's flat cut (`brep::fillet`), each run of such edges one piece, cut
//! off by every face at its ends (a plane's half-space, a curved face's solid) or, where a corner
//! stands there, cut short at the corner's section and glued to its cell.
//!
//! | the region's body | built as |
//! |---|---|
//! | `Run` / `Fill`: within (outside) two faces, beyond reach of the ball rolled along their edge | a prism of the corner less the ball's arc (a cylinder face); beside a cylinder, its arc a side of the section; between two, the ball where the two circles grown by its radius meet |
//! | `Flat` / `FlatFill`: within (outside) two faces, short of the flat through the legs | a prism of the triangle (a plane face); beside a cylinder (or a sphere, in a meridian) the flat is a conic's arc, the sum of a distance from a line and one from a circle (`Corner2`, by `brep::corners`' rule: an extrusion, or a revolution, face); between two circles with equal legs an ellipse or a hyperbola's arc, their centres its foci (with unequal ones a Cartesian oval, which no rational curve holds: refused) |
//! | `Corner` / `CornerFill`: within (outside) three faces meeting at a corner, beyond reach of the ball touching all three | the ball where Newton's steps put it, `r` from each surface; the corner's cell — the trihedron cut by the planes of the ball's arc between each two faces (its sections) — less the ball (a sphere's patch), over the three surfaces, their meetings in closed form or traced about the corner (`brep::fillet::corner_among`); and each edge's run, ring stretch or rolled run cut short at its section there and glued to it |
//! | `Run` / `Fill` / `Flat` round a circle (or a stretch of one between seams) where a plane, a cylinder, a cone, a sphere or a torus meet about one axis | the section turned about the axis through the circle's angle (a torus face); in the meridian a sphere is a circle, a torus its tube's |
//! | `Run` / `Fill` along an edge no line or coaxial circle carries (two pipes' saddle, a plane across a cylinder, a cone's generator) | the ball rolled along the run, traced, against the stratum (`brep::fillet::roll`): its canal fitted, carried past each end and cut off by the faces there |
//! | `Flat` / `FlatFill` along such an edge | the flat sectioned square to the faces' meeting between its legs and fitted (`brep::fillet::flat_along`), its strips and ends as a rolled round's |
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
use crate::model::{Sketch, SolidDef};
use crate::space::{add, distance, dot, norm, normalised, scale, sub};
use piece::toward;
use recognise::Faces;
use std::rc::Rc;

mod assemble;
mod piece;
mod recognise;
pub use assemble::{assemble, Assembly};

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

    /// A point within the solid a cylinder, a sphere or a cone bounds (its axis's point where the
    /// cone is wider than nothing); a plane's origin.
    fn within(&self) -> V {
        match *self {
            Surf::Plane { origin, .. } | Surf::Cylinder { origin, .. } => origin,
            Surf::Sphere { centre, .. } => centre,
            Surf::Cone { origin, axis, radius, half } => {
                let slope = half.dtan();
                if radius > 0.0 || slope.abs() < 1e-12 { origin }
                else { add(origin, scale(axis, (1.0 - radius) / slope)) }
            }
        }
    }

    /// The gradient of `side` at `x` (a unit vector but at a cone's axis).
    fn gradient(&self, x: V) -> V {
        match *self {
            Surf::Plane { normal, .. } => normal,
            Surf::Sphere { centre, .. } => normalised(sub(x, centre)).unwrap_or([0.0; 3]),
            _ => {
                let (origin, axis, _, half) = self.axial().expect("a cylinder or a cone");
                let w = sub(x, origin);
                let radial = normalised(sub(w, scale(axis, dot(w, axis)))).unwrap_or([0.0; 3]);
                let (sn, cs) = half.dsin_cos();
                sub(scale(radial, cs), scale(axis, sn))
            }
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
    /// A chamfer's flat cut along the run instead (#218's 6): its legs, on the face `on` stands
    /// beside and on the other, each from the other face's surface (`brep::fillet::flat_along`).
    pub flat: Option<([f64; 2], V)>,
    /// Where its run ends (an open one's two vertices).
    pub ends: Vec<V>,
    /// The sections of the corners standing at its ends, each a point of it, the normal toward the
    /// run, and how far the corner's cell reaches from it: what lies on the corner's side within
    /// that reach is cut from the piece, the cell taking it (#218's 7).
    pub cuts: Vec<(V, V, f64)>,
}



/// What the region's section is beside its edges (or, in the meridian, beside its corner:
/// `region::meridian_corner`).
#[derive(Clone, Copy, Debug)]
pub(crate) enum Section {
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
/// recognised it refuses it. Remembered against its own terms (`reads::reads_own`) and the
/// stratum it reads, `faces::cached`'s (`Caches::lowering`): a body rounded all over has
/// hundreds of regions, each reading one body, and a key over the body's every term would walk
/// them all for each.
pub fn lowered(sk: &Sketch, solid: usize) -> Option<Result<Rc<Vec<Piece>>, String>> {
    let SolidDef::Region { terms, .. } = &sk.solids.get(solid)?.def else { return None };
    let (faces, section) = recognise::recognised(sk, terms)?;
    let body = match super::faces::cached(sk, faces.faces[0].solid as usize) {
        Ok(b) => b,
        Err(e) => return Some(Err(e)),
    };
    let key = super::MemoKey::own(sk, solid, vec![body.clone()]);
    Some(sk.caches.lowering.get_or(solid, key, || piece::build(&body, &faces, section).and_then(|pieces| {
        gate(sk, solid, &pieces, &body, || beside_edges(&body, &faces, section))?;
        Ok(Rc::new(pieces))
    })))
}

/// Points just inside the corner along each edge the region rounds or cuts (a quarter, a half and
/// three quarters along it), a tenth of the section's size in along the bisector of its two faces:
/// where the field reads the region there, the pieces must hold it (`gate`).
fn beside_edges(body: &super::faces::BodyFaces, f: &Faces, section: Section) -> Vec<V> {
    let size = match section { Section::Round(r) => r, Section::Flat { la, lb } => la.min(lb) };
    let groups: Vec<Vec<usize>> = f.faces.iter().map(|r| body.faces(r)).collect();
    let mut out = Vec::new();
    for &[i, j] in f.pairs() {
        for e in body.edges_between(&groups[i], &groups[j]) {
            let &[fa, fb] = body.faces_of(e) else { continue };
            if body.convex(e) != Some(f.within) {
                continue;
            }
            for s in [0.25, 0.5, 0.75] {
                let p = body.along(e, s);
                let Some(w) = normalised(add(toward(body.normal(fa, p), f.within), toward(body.normal(fb, p), f.within))) else {
                    continue;
                };
                out.push(add(p, scale(w, 0.1 * size)));
            }
        }
    }
    out
}

/// **The gate**: a lowering stands only where the region's own field agrees with it — every probe
/// a little inside the built boundary material by the field, every one a little outside not
/// (`agreement`, sampled by area), and what the field reads `beside` each edge held; one holding
/// nowhere, only where the field reads it empty too.
fn gate(sk: &Sketch, solid: usize, pieces: &[Piece], body: &super::faces::BodyFaces, beside: impl FnOnce() -> Vec<V>)
    -> Result<(), String> {
    if pieces.is_empty() {
        let empty = matches!(super::region::form(sk, solid), Ok(super::region::Form::General(f)) if f.is_empty());
        return if empty { Ok(()) } else {
            Err("the field reads material where the faces it reads say there is none".to_string())
        };
    }
    let field = super::MaterialField::read(sk, solid, super::cad::AXIS_TOLERANCE)?;
    // the pieces' boundary in model units: a rolled one rolled against the stratum (millimetres,
    // about its origin) and brought back
    let rolled: Vec<&Rolled> = pieces.iter().filter_map(|p| match p { Piece::Rolled(k) => Some(k), _ => None }).collect();
    let (b, mm) = if rolled.is_empty() { (assemble(pieces).brep(1.0)?, None) } else {
        // in the stratum's own frame (millimetres about its origin), as the recipe builds it: the
        // lowered node, rolled against the stratum standing in its built set
        let (stratum, mm, origin) = body.brep();
        let mut node = assemble(pieces).node(&crate::brep::recipe::Writer::about(mm, scale(origin, mm)))?;
        node.set("stratum", object([("kind", "body".into()), ("stock", 0.into()), ("on", Json::Arr(vec![])),
            ("cut", Json::Arr(vec![])), ("bound", Json::Arr(vec![]))]));
        let built = std::collections::BTreeMap::from([(0, stratum.clone())]);
        (crate::brep::recipe::node_named(&node, &built, &Default::default(), 0.0)?, Some(mm))
    };
    // no piece left out: what the field reads of the region just inside each edge, the pieces
    // hold (a probe of the boundary built sees nothing of a piece never built)
    let located = crate::brep::query::Located::new(&b, 1e-9 * b.size().max(f64::MIN_POSITIVE));
    for p in beside() {
        if field.side(p) < -1e-6 * (1.0 + norm(p)) {
            let q = if mm.is_some() { body.q(p) } else { p };
            if located.solid_place(q) == crate::brep::query::Place::Out {
                return Err(format!("its exact construction leaves out what the field reads beside its edge at [{:.6}, {:.6}, {:.6}]",
                    p[0], p[1], p[2]));
            }
        }
    }
    let back = |p: V| body.back(p);
    let back = mm.map(|mm| (&back as &dyn Fn(V) -> V, mm));
    super::agreement::gate(&field, &b, back, "its exact construction disagrees with the field")
}
