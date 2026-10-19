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
mod chain;
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
/// by the tangent of `half` (a drafted eye's side, #245); or a sphere; or a torus about the line
/// through `origin` along the unit `axis`, its tube of radius `tube` about the circle of radius
/// `big` there (a rim's round a section reaches across, #422).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Surf {
    Plane { origin: V, normal: V },
    Cylinder { origin: V, axis: V, radius: f64 },
    Cone { origin: V, axis: V, radius: f64, half: f64 },
    Sphere { centre: V, radius: f64 },
    Torus { origin: V, axis: V, big: f64, tube: f64 },
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
        } else if let Some((centre, radius)) = body.sphere(fi) {
            Some(Surf::Sphere { centre, radius })
        } else {
            body.torus(fi).map(|(origin, axis, big, tube)| Surf::Torus { origin, axis, big, tube })
        }
    }

    /// A point within the solid a cylinder, a sphere or a cone bounds (its axis's point where the
    /// cone is wider than nothing); a plane's origin.
    fn within(&self) -> V {
        match *self {
            Surf::Plane { origin, .. } | Surf::Cylinder { origin, .. } => origin,
            Surf::Sphere { centre, .. } => centre,
            Surf::Torus { origin, axis, big, .. } => add(origin, scale(crate::brep::geom::Frame::about(origin, axis).x, big)),
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
            Surf::Torus { origin, axis, big, .. } => normalised(sub(x, Surf::tube_centre(origin, axis, big, x))).unwrap_or([0.0; 3]),
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
            Surf::Plane { .. } | Surf::Sphere { .. } | Surf::Torus { .. } => None,
        }
    }

    /// The point of a torus's tube's centre circle nearest `x` (about the line through `origin`
    /// along `axis`, of radius `big`).
    fn tube_centre(origin: V, axis: V, big: f64, x: V) -> V {
        let w = sub(x, origin);
        let radial = sub(w, scale(axis, dot(w, axis)));
        let out = normalised(radial).unwrap_or(crate::brep::geom::Frame::about(origin, axis).x);
        add(origin, scale(out, big))
    }

    /// From `x` to the surface along its normal: positive on the side the plane's normal points
    /// to, or outside the cylinder, the cone or the ball.
    fn side(&self, x: V) -> f64 {
        match *self {
            Surf::Plane { origin, normal } => dot(normal, sub(x, origin)),
            Surf::Sphere { centre, radius } => distance(x, centre) - radius,
            Surf::Torus { origin, axis, big, tube } => distance(x, Surf::tube_centre(origin, axis, big, x)) - tube,
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
            Surf::Plane { .. } | Surf::Torus { .. } => return None,
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
        self.strokes.iter().find_map(|(s, n)| match *s { Stroke::Arc { centre, .. } if *n == "round" => Some(centre), _ => None })
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
    Beyond(Beyond),
}

/// A ring's section beyond a seam's normal surface (#422, model units): where the ring's own face
/// goes on tangent across a seam its section reaches, and the seam is no circle about the ring's
/// axis (a boss near a plate's rounded rim), the ring is kept short of the seam's normal surface
/// (its `Kept`) and this takes the rest — the section against the next face, in the half-planes
/// through the ring's axis (`axis_origin`, `axis`), fitted (`brep::spill`) — glued to the ring on
/// that surface.  The ring's edge is the circle `crease` (its centre and radius); `toward` the way
/// from the ring's own face into the region; `chain` the next face's surface, `wall` the other
/// face's, `split` the seam's normal surface, each with the way its signed distance runs toward
/// the piece (`sides`, ±1); `seam` the seam's circle (centre, unit normal, radius); a chamfer's
/// legs, on the chain from the wall and on the wall from the chain, or a round's `radius` (nothing
/// for a chamfer); and `ends`, where the leg on the ring's own face crosses the seam — or where the
/// ball touches it, a round's ring a stretch glued to this at both ends.
#[derive(Clone, Debug, PartialEq)]
pub struct Beyond {
    pub axis_origin: V,
    pub axis: V,
    pub crease: (V, f64),
    pub toward: V,
    pub chain: Surf,
    pub wall: Surf,
    pub split: Surf,
    pub sides: [f64; 3],
    pub seam: (V, V, f64),
    pub legs: [f64; 2],
    pub radius: f64,
    pub ends: [V; 2],
}

impl Beyond {
    /// The point of its edge, the crease, in the half-plane through its axis and `p`.
    pub fn crease_at(&self, p: V) -> V {
        let w = sub(p, self.crease.0);
        let radial = normalised(sub(w, scale(self.axis, dot(w, self.axis)))).unwrap_or([0.0; 3]);
        add(self.crease.0, scale(radial, self.crease.1))
    }
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
    Some(sk.caches.lowering.get_or(solid, key, || {
        let spill = seams_beside(&body, &faces, section);
        piece::build(&body, &faces, section, &spill)
    }.and_then(|pieces| {
        gate(sk, solid, &pieces, &body, || beside_edges(&body, &faces, section))?;
        Ok(Rc::new(pieces))
    })))
}

/// How many points along each edge `seams_beside` stands a section at (and its ends).
const SEAM_SAMPLES: usize = 32;

/// The seams a section reaches across, by the face each goes on from: each seam's edge and the
/// face going on across it.
pub(crate) type Spill = std::collections::BTreeMap<usize, Vec<(u32, usize)>>;

/// **A smooth surface the section reaches across** ([0.149], #422): the round or the flat stands
/// against a face's smooth surface (`F.smooth`), so where the face goes on tangent into another
/// across a seam running beside the edge and the section reaches across it — its footprint on the
/// face (`footprint`: a flat's leg, where a ball touches it), stood at points along the edge, lands
/// past that seam, a chamfer's leg onto a round's face — the section is against the chain of the
/// two (`chain`), and what no construction of the table builds so is refused by name
/// (`piece::spilled`).  A seam beyond the section's reach is none of it: the region's points are
/// over the face between its edge and its footprint, nearest the face's own surface.  A seam the
/// edge runs into at its end (a slot's side going on into its end's half circle) is the table's
/// own rule, a run going on into the next piece, and is none of it either.
fn seams_beside(body: &super::faces::BodyFaces, f: &Faces, section: Section) -> Spill {
    let groups: Vec<Vec<usize>> = f.faces.iter().map(|r| body.faces(r)).collect();
    let mut out = Spill::new();
    for (k, _) in f.faces.iter().enumerate().filter(|(_, r)| r.smooth) {
        let chain = body.smooth(&groups[k]);
        if chain.len() == groups[k].len() {
            continue;
        }
        let beyond: Vec<usize> = chain.into_iter().filter(|fi| !groups[k].contains(fi)).collect();
        let others: Vec<usize> = f.pairs().iter().filter(|p| p.contains(&k))
            .map(|&[i, j]| if i == k { j } else { i }).collect();
        let creases: Vec<u32> = others.iter().flat_map(|&o| body.edges_between(&groups[k], &groups[o])).collect();
        let at: Vec<u32> = creases.iter().flat_map(|&e| body.ends(e)).collect();
        let seams: Vec<u32> = body.edges_between(&groups[k], &beyond).into_iter()
            .filter(|&s| !body.ends(s).iter().any(|v| at.contains(v))).collect();
        if seams.is_empty() {
            continue;
        }
        let mut reached: Vec<(usize, u32)> = Vec::new();
        for &e in &creases {
            let &[x, y] = body.faces_of(e) else { continue };
            // (an edge turning the other way holds nowhere: nothing of it reaches anywhere)
            if body.convex(e) != Some(f.within) {
                continue;
            }
            let (fk, fo) = if groups[k].contains(&x) { (x, y) } else { (y, x) };
            for s in 0..=SEAM_SAMPLES {
                let p = body.along(e, s as f64 / SEAM_SAMPLES as f64);
                let Some(q) = footprint(body, fk, fo, p, f.within, section, k) else { continue };
                if let Some(seam) = body.off_face(fk, q).filter(|s| seams.contains(s)) {
                    if !reached.contains(&(fk, seam)) {
                        reached.push((fk, seam));
                    }
                }
            }
        }
        // with each seam reached, the rest of the curve it lies on (a seam in pieces is one seam
        // to the section)
        let one = |a: u32, b: u32| match (body.segment(a), body.segment(b), body.circle(a), body.circle(b)) {
            (Some([p, q]), Some([u, w]), ..) => {
                let d = normalised(sub(q, p)).unwrap_or([0.0; 3]);
                let off = |x: V| norm(crate::space::cross(sub(x, p), d));
                off(u) <= 1e-9 * (1.0 + norm(u)) && off(w) <= 1e-9 * (1.0 + norm(w))
            }
            (_, _, Some((c, n, r, _)), Some((k, m, s, _))) => distance(c, k) <= 1e-9 * (1.0 + norm(c))
                && norm(crate::space::cross(n, m)) <= 1e-9 && (r - s).abs() <= 1e-9 * (1.0 + r),
            _ => a == b,
        };
        for &(fk, r) in &reached {
            for &s in seams.iter().filter(|&&s| one(s, r) && body.faces_of(s).contains(&fk)) {
                let next = body.faces_of(s).iter().copied().find(|g| !groups[k].contains(g));
                let entry = out.entry(fk).or_default();
                if let Some(g) = next.filter(|_| !entry.iter().any(|&(t, _)| t == s)) {
                    entry.push((s, g));
                }
            }
        }
    }
    out
}

/// Where the section beside the edge of faces `fk` and `fo` at its point `p` reaches furthest
/// along `fk` (the region's `k`th face): a flat's leg, the point of `fk`'s surface its leg from
/// `fo`'s; a round's touch, where the ball `r` from both surfaces touches `fk`'s — on the region's
/// side of both (`within`), in the plane through `p` square to the edge, by Newton's steps (none
/// where they find none).
fn footprint(body: &super::faces::BodyFaces, fk: usize, fo: usize, p: V, within: bool, section: Section, k: usize)
    -> Option<V> {
    let (nk, no) = (body.normal(fk, p), body.normal(fo, p));
    let t = normalised(crate::space::cross(nk, no))?;
    let want = if within { -1.0 } else { 1.0 };
    let (into_k, into_o) = (toward(nk, within), toward(no, within));
    let size = match section { Section::Round(r) => r, Section::Flat { la, lb } => la.max(lb) };
    let settled = |at: &[(V, f64); 3]| at.iter().all(|(_, m)| m.abs() <= 1e-12 * (size + norm(p)));
    let square = (t, dot(t, p));
    match section {
        Section::Flat { la, lb } => {
            let l = if k == 0 { la } else { lb };
            // along `fk` away from the edge, the way `fo`'s side grows toward the region
            let w = normalised(sub(into_o, scale(nk, dot(into_o, nk))))?;
            let start = add(p, scale(w, l / dot(w, into_o).max(0.1)));
            crate::roots::newton_onto::<3>(start, size, |q| {
                let (sk, gk) = body.side_grad(fk, q);
                let (so, go) = body.side_grad(fo, q);
                Some([(gk, sk), (go, so - want * l), (square.0, dot(square.0, q) - square.1)])
            }, settled)
        }
        Section::Round(r) => {
            let start = crate::brep::fillet::across(p, [into_k, into_o], r);
            let c = crate::roots::newton_onto::<3>(start, size, |c| {
                let (sk, gk) = body.side_grad(fk, c);
                let (so, go) = body.side_grad(fo, c);
                Some([(gk, sk - want * r), (go, so - want * r), (square.0, dot(square.0, c) - square.1)])
            }, settled)?;
            let (sk, gk) = body.side_grad(fk, c);
            Some(sub(c, scale(gk, sk)))
        }
    }
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
fn gate(sk: &Sketch, solid: usize, pieces: &[Piece], body: &std::sync::Arc<super::faces::BodyFaces>,
    beside: impl FnOnce() -> Vec<V>) -> Result<(), String> {
    if pieces.is_empty() {
        let empty = matches!(super::region::form(sk, solid), Ok(super::region::Form::General(f)) if f.is_empty());
        return if empty { Ok(()) } else {
            Err("the field reads material where the faces it reads say there is none".to_string())
        };
    }
    let field = super::MaterialField::read(sk, solid, super::cad::AXIS_TOLERANCE)?;
    let built = gated(sk, pieces, body)?;
    let (b, mm) = (&built.brep, built.mm);
    // no piece left out: what the field reads of the region just inside each edge, the pieces
    // hold (a probe of the boundary built sees nothing of a piece never built)
    let located = crate::brep::query::Located::new(b, 1e-9 * b.size().max(f64::MIN_POSITIVE));
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
    super::agreement::gate_meshed(&field, b, &built.mesh, back, "its exact construction disagrees with the field")
}

/// **A gate's construction** (`gated`): the pieces' boundary — in model units, or a rolled one
/// rolled against the stratum in millimetres (`mm` of them a model unit) — and its mesh as the
/// gate probes it, with what it was built from: the node's text up to its order (`canonical`)
/// and the stratum a rolled one rolls against.
#[derive(Debug)]
pub(crate) struct Gated {
    key: String,
    stratum: Option<std::sync::Arc<super::faces::BodyFaces>>,
    brep: crate::brep::topo::Brep,
    mesh: crate::brep::mesh::Mesh,
    mm: Option<f64>,
}

/// A lowered node's text up to the order it lists things in: its pieces, its corners (each's
/// faces sorted), its rolled runs, and its joins, each as the pair of what it joins.
fn canonical(node: &Json) -> String {
    let dump = |j: &Json| j.dump(None);
    let corner = |c: &Json| {
        let mut faces = c.get("faces").map_or(&[][..], Json::arr).to_vec();
        faces.sort_by_cached_key(dump);
        let mut c = c.clone();
        c.set("faces", Json::Arr(faces));
        c.dump(None)
    };
    // in the order joins count them: runs and rings, then corners, then rolled runs, then the
    // pieces beyond a seam's normal surface
    let listed = |key: &str| node.get(key).map_or(&[][..], Json::arr);
    let each: Vec<String> = listed("pieces").iter().map(dump).chain(listed("corners").iter().map(corner))
        .chain(listed("rolled").iter().map(dump)).chain(listed("beyond").iter().map(dump)).collect();
    let mut joins: Vec<[&str; 2]> = listed("joins").iter().map(|j| {
        let at = |k: usize| j.arr().get(k).map_or(usize::MAX, |i| i.as_i64() as usize);
        let mut pair = [0, 1].map(|k| each.get(at(k)).map_or("", String::as_str));
        pair.sort();
        pair
    }).collect();
    joins.sort();
    let mut sorted: Vec<&str> = each.iter().map(String::as_str).collect();
    sorted.sort();
    let mut out = object([
        ("each", Json::Arr(sorted.into_iter().map(|t| Json::Str(t.into())).collect())),
        ("joins", Json::Arr(joins.into_iter().map(|p| Json::Arr(p.map(|t| Json::Str(t.into())).to_vec())).collect())),
    ]);
    if let Some(stratum) = node.get("stratum") {
        out.set("stratum", stratum.clone());
    }
    out.dump(None)
}

/// How many constructions `Caches::gated` keeps, the latest asked: enough for the orders of a
/// corner of nine faces, met among nine distinct constructions.  `std`'s words state each edge
/// and corner once (#415); this is for the words that do not — a document's own written without
/// `symmetric`, a round over faces drawn `in` a plane or riding a frame.
const GATED: usize = 16;

/// The pieces' boundary and its mesh, remembered against the recipe node they are built from up
/// to its order (`canonical`; and the stratum a rolled one rolls against, by identity), the
/// latest few (`Caches::gated`): a round over every edge of a body is a region for each order of
/// the faces it names, so `Corner(a, b, c)` and `Corner(b, a, c)` lower to the same pieces
/// listed in another order — one point set, built and meshed once and held to each region's own
/// field.
fn gated(sk: &Sketch, pieces: &[Piece], body: &std::sync::Arc<super::faces::BodyFaces>) -> Result<Rc<Gated>, String> {
    use crate::brep::recipe::Writer;
    use std::sync::Arc;
    let rolled = pieces.iter().any(|p| matches!(p, Piece::Rolled(_)));
    // the pieces' boundary in model units; a rolled one in the stratum's own frame (millimetres
    // about its origin), as the recipe builds it — the lowered node, rolled against the stratum
    // standing in its built set — and brought back
    let (_, by, origin) = body.brep();
    let (writer, mm) =
        if rolled { (Writer::about(by, scale(origin, by)), Some(by)) } else { (Writer::world(1.0), None) };
    let mut node = assemble(pieces).node(&writer)?;
    if rolled {
        node.set("stratum", object([("kind", "body".into()), ("stock", 0.into()), ("on", Json::Arr(vec![])),
            ("cut", Json::Arr(vec![])), ("bound", Json::Arr(vec![]))]));
    }
    let key = canonical(&node);
    let stratum = rolled.then(|| body.clone());
    let at = |s: &Option<Arc<super::faces::BodyFaces>>| s.as_ref().map(Arc::as_ptr);
    let mut cache = sk.caches.gated.borrow_mut();
    if let Some(k) = cache.iter().position(|g| g.key == key && at(&g.stratum) == at(&stratum)) {
        let found = cache.remove(k);
        cache.push(found.clone());
        return Ok(found);
    }
    drop(cache);
    let built: std::collections::BTreeMap<_, _> = stratum.iter().map(|b| (0, b.brep().0.clone())).collect();
    let brep = crate::brep::recipe::node_named(&node, &built, &Default::default(), 0.0)?;
    let mesh = super::agreement::gate_mesh(&brep)?;
    let built = Rc::new(Gated { key, stratum, brep, mesh, mm });
    let mut cache = sk.caches.gated.borrow_mut();
    if cache.len() >= GATED {
        cache.remove(0);
    }
    cache.push(built.clone());
    Ok(built)
}
