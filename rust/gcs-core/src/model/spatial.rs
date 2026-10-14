//! Spatial entity storage, construction and evaluated-solid cache access.

use super::*;

/// An inner boundary and the source names of its swept sides.
#[derive(Clone, Debug)]
pub struct FaceLoop {
    pub edges: Vec<EntRef>,
    pub edge_names: Vec<String>,
}

/// A face's support is either an inherited profile plane or a named spatial
/// surface/envelope. Spatial support must never default to the page plane.
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum FaceSupport { Plane(Option<u32>), Surface(EntRef) }

/// A region bounded by existing edges. Planar profiles inherit their plane from
/// point membership; spatial faces name their analytic support explicitly.
#[derive(Clone, Debug)]
pub struct FaceE {
    /// The loop, in traversal order: planar lines/arcs/circles, or named spatial edges.
    pub edges: Vec<EntRef>,
    /// What the document calls each edge, in the same order — the name the face swept from it
    /// is reached by (`block.side_l`).  Kept beside the references because a solid's face path
    /// is spelled out of the *source's* words, and an anonymous edge has none to spell.
    pub edge_names: Vec<String>,
    /// Strictly contained, disjoint inner boundaries.
    pub holes: Vec<FaceLoop>,
    /// Inherited profile plane (`None` is the page) or exact named spatial support.
    pub support: FaceSupport,
    /// What the document calls it.  A solid's faces are reached by *path* — `body.bore.wall` —
    /// so unlike every other kind a face and a solid carry their own name: the path is the
    /// naming mechanism, and a report that could not spell one would have nothing to say.
    pub name: String,
    pub class: Classes,
}

impl FaceE {
    pub fn plane(&self) -> Result<Option<u32>,String> {
        match self.support {
            FaceSupport::Plane(p) => Ok(p),
            FaceSupport::Surface(_) => Err("a spatial face is not a planar sweep profile".into()),
        }
    }
    pub fn on(&self) -> Option<EntRef> {
        match self.support { FaceSupport::Surface(s) => Some(s), FaceSupport::Plane(_) => None }
    }
    pub fn boundaries(&self) -> impl Iterator<Item = (&[EntRef], &[String])> {
        std::iter::once((self.edges.as_slice(), self.edge_names.as_slice()))
            .chain(self.holes.iter().map(|h| (h.edges.as_slice(), h.edge_names.as_slice())))
    }
}

/// A finite length in the drawing's user units.
#[derive(Clone, Debug)]
pub struct Length {
    value: f64,
    text: String,
}
impl Length {
    pub fn new(value: f64) -> Result<Self, String> {
        if !value.is_finite() {
            return Err("claim gap must be a finite length".into());
        }
        Ok(Self {
            value,
            text: String::new(),
        })
    }
    pub(crate) fn written(value: f64, text: String) -> Result<Self, String> {
        Ok(Self {
            text,
            ..Self::new(value)?
        })
    }
    pub fn value(&self) -> f64 {
        self.value
    }
    pub fn text(&self) -> &str {
        &self.text
    }
}

/// Complete argument forms. An inside claim cannot carry a gap.
#[derive(Clone, Debug)]
pub enum SolidRequirement {
    Inside,
    Fits { gap: Length },
    Clear { gap: Length },
}
impl SolidRequirement {
    pub fn word(&self) -> crate::constraints::SolidWord {
        use crate::constraints::SolidWord as W;
        match self {
            Self::Inside => W::Inside,
            Self::Fits { .. } => W::Fits,
            Self::Clear { .. } => W::Clear,
        }
    }
    pub fn gap(&self) -> Option<&Length> {
        match self {
            Self::Inside => None,
            Self::Fits { gap } | Self::Clear { gap } => Some(gap),
        }
    }
    /// Checked adapter for callers of the former word/gap API.
    pub(crate) fn from_word(word: crate::constraints::SolidWord, gap: f64) -> Result<Self, String> {
        use crate::constraints::SolidWord as W;
        match word {
            W::Inside if gap == 0.0 => Ok(Self::Inside),
            W::Inside => Err("inside takes no gap".into()),
            W::Fits => Ok(Self::Fits {
                gap: Length::new(gap)?,
            }),
            W::Clear => Ok(Self::Clear {
                gap: Length::new(gap)?,
            }),
        }
    }
}

/// A parsed, validated claim. Solid references and sweep dimensions are checked before creation.
///
/// ```compile_fail
/// fn change_sweep(sw: &mut gcs_core::model::Sweep) {
///     sw.from = f64::NAN;
/// }
/// ```
#[derive(Clone, Debug)]
pub struct SolidClaim {
    pub(crate) requirement: SolidRequirement,
    pub(crate) a: u32,
    pub(crate) b: u32,
    pub(crate) over: Option<Sweep>,
    pub(crate) stmt: u32,
}
impl SolidClaim {
    pub fn requirement(&self) -> &SolidRequirement {
        &self.requirement
    }
    pub fn solids(&self) -> (u32, u32) {
        (self.a, self.b)
    }
    pub fn over(&self) -> Option<&Sweep> {
        self.over.as_ref()
    }
    pub fn stmt(&self) -> u32 {
        self.stmt
    }
}

/// A cap used for placement, checked for surviving material after the drawing solves.
#[derive(Clone, Debug)]
pub struct SolidBearing {
    pub solid: u32,
    pub path: String,
    pub stmt: u32,
    pub span: crate::syntax::Span,
}

/// The interval a claim is swept over: a free variable of the drawing, or a named motion's roll
/// (in degrees), and where it runs.
#[derive(Clone, Debug)]
pub struct Sweep {
    pub(crate) name: String,
    pub(crate) from: f64,
    pub(crate) to: f64,
    pub(crate) dimension: crate::units::Dim,
    /// The motion whose roll is swept, where it is one: every solid placed under it is read at
    /// its `at:` advanced by the roll, and nothing is solved again.
    pub(crate) motion: Option<u32>,
}
impl Sweep {
    /// The motion whose roll the claim runs along, if it runs along one.
    pub fn motion(&self) -> Option<u32> {
        self.motion
    }
    pub fn name(&self) -> &str {
        &self.name
    }
    pub fn from(&self) -> f64 {
        self.from
    }
    pub fn to(&self) -> f64 {
        self.to
    }
    pub fn dimension(&self) -> crate::units::Dim {
        self.dimension
    }
    /// Convex interpolation avoids overflowing the difference of opposite extreme endpoints.
    pub fn sample(&self, index: usize, intervals: usize) -> Option<f64> {
        if intervals == 0 || index > intervals {
            return None;
        }
        if index == 0 {
            return Some(self.from);
        }
        if index == intervals {
            return Some(self.to);
        }
        let t = index as f64 / intervals as f64;
        let value = if self.from.signum() == self.to.signum() {
            self.from + (self.to - self.from) * t
        } else {
            self.from * (1.0 - t) + self.to * t
        };
        Some(value)
    }
}

/// A picture asked of a solid (§6.11): what of, cut where, drawn in which view.
#[derive(Clone, Debug)]
pub struct DerivedE {
    pub solid: u32,
    /// The plane it is drawn in; `None` is the page.
    pub plane: Option<u32>,
    /// A section's cutting plane.  `None` is a plain view.
    pub at: Option<u32>,
    /// A picture that is only its **dimensions** (§6.12): the sheet as a report.
    pub dims: bool,
    pub name: String,
    pub class: Classes,
}

/// A number a solid is swept or stood off by: the text a person wrote and what it came to.
///
/// It is settled by the flattener over the parameters in scope and is **never an unknown**, so
/// a solve moves it no more than it moves a spline's knots.  The text is kept because a drawing
/// reads better for saying `fw + D / 2` than `18`, which is the same reason a dimension carries
/// its expression.
#[derive(Clone, Debug, PartialEq)]
pub struct Extent {
    pub text: String,
    pub value: f64,
}

impl Extent {
    /// An extent nobody wrote: a caller building a solid in Rust, and what a gesture would
    /// splice.  The text is the number, printed as the source printer prints one.
    pub fn at(value: f64) -> Extent {
        Extent { text: crate::syntax::num(value), value }
    }
}

/// **How far a sweep rolls** (§6.9, #271): an interval of its motion's parameter stated (`from:`
/// / `to:`, radians), or `through: body` — the roll about 0 during which the tool meets the body's
/// material, its stock and additions (never its cuts, so a sweep cutting the body it rolls through
/// is no cycle), found after the solve (`solid::roll`).  Every reader asks `solid::roll::interval`,
/// never the extents, so the exact construction, admission and the field read one interval.
#[derive(Clone, Debug)]
pub enum Roll {
    Stated { from: Extent, to: Extent },
    Through { body: u32 },
}

/// Which way a partial revolution turns — a word, never a sign (§9.2).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Sense {
    Ccw,
    Cw,
}

/// **What a solid is** (Solvent §6.9).  Three forms and no fourth: a face swept two ways, and a
/// body over other solids.
#[derive(Clone, Debug)]
pub enum SolidDef {
    /// A rigid instance of another solid at one angle of a named motion family.
    Placed { source: u32, motion: u32, at: Extent },
    /// Continuous material swept under a named motion over its roll (`Roll`): an interval
    /// stated, or the one its tool takes through a body.
    Swept { source: u32, motion: u32, roll: Roll },
    /// A face swept along its plane's normal, between two signed coordinates along it.  `from`
    /// and `to` are ordinates and their signs are arithmetic; `depth: d` is the draughtsman's
    /// spelling of `from: -d, to: 0`, the material behind the face the view shows.
    ///
    /// `draft` (#245, radians) tapers its sides from the face's own plane, the parting plane: the
    /// section at ordinate `h` is the face's offset `|h| tan draft` into its material (an outer
    /// loop shrinks, a hole grows), so the sides lean away from the plane either side of it, as a
    /// forging's come out of its die; a negative draft leans them the other way.  A straight side
    /// stays a plane and a circle's a cone; the faces keep the prism's names, a side crossing the
    /// plane in two pieces of one name.
    Prism { face: u32, from: Extent, to: Extent, draft: Option<Extent> },
    /// Sections at the endpoints of a directed line or circular arc.
    Loft { face: u32, end: Option<u32>, guide: EntRef },
    /// A prism spanning the target's additive material, evaluated after solving.
    Through { face: u32, body: u32 },
    /// A face swept about a line or an axis **in its own plane**, through `sweep` (a full turn
    /// where the document writes none), right-handed about the line's own `p1 → p2` (an axis's
    /// own direction) unless `sense: cw`.  Read through `Sketch::revolve_axis`.
    Revolve { face: u32, axis: EntRef, sweep: Extent, sense: Sense },
    /// **The body rule, and the whole of it**: its stock, plus everything in `union` with it,
    /// minus everything that `cut`s it, within everything it is in `common` with.  All three are
    /// sets, so the statements that fill them may be written anywhere in any order (P2) — which is
    /// what a feature tree, folding a *sequence*, is not.  Difference and intersection commute, so
    /// the last two need no order between them; union comes first.  A design needing the
    /// other order names the intermediate, and then there are two solids because there are
    /// two things.
    Body { stock: u32, on: Vec<u32>, through: Vec<u32>, bound: Vec<u32> },
    /// **A region as a solid** (§6.9, §6.21): `solid(R)`, the points `inside` the set `R` — its
    /// body applied to a hidden `probe` point, each bound it put there one of `terms`.  Its terms
    /// share one axis, so it is a revolution of one meridian region (`solid::region`).
    Region { probe: u32, terms: Vec<RegionTerm> },
}

impl SolidDef {
    /// The set of a body's that `word` fills — `union`'s, `cut`'s or `common`'s — or `None` for a
    /// solid that is not a body.
    pub fn body_set(&mut self, word: crate::syntax::BodyWord) -> Option<&mut Vec<u32>> {
        use crate::syntax::BodyWord;
        match (self, word) {
            (SolidDef::Body { on, .. }, BodyWord::Union) => Some(on),
            (SolidDef::Body { through, .. }, BodyWord::Cut) => Some(through),
            (SolidDef::Body { bound, .. }, BodyWord::Common) => Some(bound),
            _ => None,
        }
    }
}

/// What a region solid's term bounds (§6.21): the measure a bound on its probe reads, recognised
/// by the constraint's kind and never by the set it came from.
#[derive(Clone, Debug, PartialEq)]
pub enum RegionShape {
    /// A distance from a point: a ball about it.
    Ball { center: u32 },
    /// A distance from a line: a cylinder about it.
    Cylinder { line: u32 },
    /// An angle from a line's direction, seen from its start: a cone, apex the line's `p1`.
    Cone { axis: u32 },
    /// An ordinate from a point along a plane's normal or a line's direction (`along`, a plane,
    /// a line or an axis: `RegionShape::normal`): a half-space.  `sign` is -1 where the probe is
    /// the ordinate's first operand, read back from it.
    Plane { along: EntRef, from: u32, sign: f64 },
    /// A sum of measurements of the probe, each times its number (§6.21, #183 F3): `distance(p,
    /// f1) + distance(p, f2) <= 10mm` is `[(f1, 1), (f2, 1)]` against 10.  Each is the distance
    /// from what it names (`Target`).
    Measured { of: Vec<(Target, f64)> },
    /// The signed distance from a face's surface (#183 F1), positive on the side away from the
    /// body's material: `p inside F` at most nothing, `p outside F` at least, `p distance(<= d,
    /// side: outside) F` at most `d`.
    FaceSide { face: FaceRef },
    /// The material's angle at the edge where faces `a` and `b` meet, at the point of it nearest
    /// the probe (#183 F9), in radians: under a half turn where the edge is convex.
    Angle { a: FaceRef, b: FaceRef },
    /// Where an edge ends (#183 F7): on the side the edge runs on of each face it ends on.  The
    /// edge is a set, the points on two faces (`Target::Set`), resolved when the solid is built.
    Ends { edge: Box<Target> },
}

impl RegionShape {
    /// A half-space's unit normal as the drawing stands: its plane's `n`, its line's or its
    /// axis's direction — `None` for another shape, or a line of no length.
    pub fn normal(&self, sk: &Sketch) -> Option<[f64; 3]> {
        let RegionShape::Plane { along, .. } = *self else { return None };
        match along.kind {
            EntKind::Plane => Some(sk.basis(along.i()).normal()),
            EntKind::Line => {
                let l = sk.lines.get(along.i())?;
                let (a, b) = (sk.world_point(l.p1 as usize), sk.world_point(l.p2 as usize));
                crate::space::normalised(crate::space::sub(b, a))
            }
            EntKind::Axis => {
                let d = sk.axes.get(along.i())?.d.map(|q| sk.params[q as usize].value);
                crate::space::normalised(d)
            }
            _ => None,
        }
    }

    /// The entities its measure reads.
    pub fn entities(&self) -> Vec<EntRef> {
        match *self {
            RegionShape::Ball { center } => vec![EntRef::point(center as usize)],
            RegionShape::Cylinder { line } | RegionShape::Cone { axis: line } => {
                vec![EntRef::line(line as usize)]
            }
            RegionShape::Plane { along, from, .. } => vec![along, EntRef::point(from as usize)],
            RegionShape::Measured { ref of } => of.iter().flat_map(|(t, _)| t.entities()).collect(),
            RegionShape::FaceSide { ref face } => vec![EntRef::solid(face.solid as usize)],
            RegionShape::Angle { ref a, ref b } => {
                vec![EntRef::solid(a.solid as usize), EntRef::solid(b.solid as usize)]
            }
            RegionShape::Ends { ref edge } => edge.entities(),
        }
    }

    /// The same measure over the entities `map` renumbers them to, or `None` where one went.
    pub fn remapped(&self, map: &dyn Fn(EntRef) -> Option<EntRef>) -> Option<RegionShape> {
        let i = |e: EntRef| map(e).map(|e| e.i() as u32);
        Some(match *self {
            RegionShape::Ball { center } => {
                RegionShape::Ball { center: i(EntRef::point(center as usize))? }
            }
            RegionShape::Cylinder { line } => {
                RegionShape::Cylinder { line: i(EntRef::line(line as usize))? }
            }
            RegionShape::Cone { axis } => RegionShape::Cone { axis: i(EntRef::line(axis as usize))? },
            RegionShape::Plane { along, from, sign } => RegionShape::Plane {
                along: map(along)?,
                from: i(EntRef::point(from as usize))?,
                sign,
            },
            RegionShape::Measured { ref of } => RegionShape::Measured {
                of: of.iter().map(|(t, c)| t.remapped(map).map(|t| (t, *c))).collect::<Option<_>>()?,
            },
            RegionShape::FaceSide { ref face } => RegionShape::FaceSide { face: face.remapped(map)? },
            RegionShape::Angle { ref a, ref b } => {
                RegionShape::Angle { a: a.remapped(map)?, b: b.remapped(map)? }
            }
            RegionShape::Ends { ref edge } => {
                RegionShape::Ends { edge: Box::new(edge.remapped(map)?) }
            }
        })
    }
}

/// A face of a solid, or the faces, as a set's body names it (#183 F1): `pad.plate.near` is
/// solid `pad`'s faces that its operand `plate` gave it as its near cap; `pad.boss` every face
/// `boss` gave it; `block` every face of `block`.  Read off the solid's exact boundary, as the
/// faces stand in it (a boolean never renames).  `surface` reads the face's whole surface
/// (`F.surface`), not its patch.  `path` is the segments written past the solid, matched
/// against each face's `solid::faces::FacePath` (#188).
#[derive(Clone, Debug, PartialEq)]
pub struct FaceRef {
    pub solid: u32,
    pub path: Vec<String>,
    pub surface: bool,
}

/// What a measurement of a region's probe measures the distance from (#183 F1–F3).
#[derive(Clone, Debug, PartialEq)]
pub enum Target {
    /// A point, a line produced, an axis or a plane.
    Ent(EntRef),
    /// A face's patch, or its surface.
    Face(FaceRef),
    /// A set (#183 F2): the points its body holds, applied to a hidden point of its own whose
    /// bounds are `terms` (filled when the solid is built).
    Set { probe: u32, terms: Vec<RegionTerm> },
}

impl Target {
    pub fn entities(&self) -> Vec<EntRef> {
        match self {
            Target::Ent(e) => vec![*e],
            Target::Face(f) => vec![EntRef::solid(f.solid as usize)],
            Target::Set { terms, .. } => terms.iter().flat_map(|t| t.shape.entities()).collect(),
        }
    }

    pub fn remapped(&self, map: &dyn Fn(EntRef) -> Option<EntRef>) -> Option<Target> {
        Some(match self {
            Target::Ent(e) => Target::Ent(map(*e)?),
            Target::Face(f) => Target::Face(f.remapped(map)?),
            Target::Set { probe, terms } => Target::Set {
                probe: map(EntRef::point(*probe as usize))?.i() as u32,
                terms: terms.iter().map(|t| t.remapped(map)).collect::<Option<_>>()?,
            },
        })
    }
}

impl FaceRef {
    pub fn remapped(&self, map: &dyn Fn(EntRef) -> Option<EntRef>) -> Option<FaceRef> {
        let solid = map(EntRef::solid(self.solid as usize))?.i() as u32;
        Some(FaceRef { solid, ..self.clone() })
    }
}

impl RegionTerm {
    /// The one-sided bound `cmp` on `shape` at `lo`, of no set and no offset.
    pub fn new(shape: RegionShape, cmp: crate::syntax::Cmp, lo: TermNumber) -> RegionTerm {
        RegionTerm { shape, cmp, lo, hi: None, grow: None, name: String::new() }
    }

    /// The same term over the entities `map` renumbers them to, or `None` where one went.
    pub fn remapped(&self, map: &dyn Fn(EntRef) -> Option<EntRef>) -> Option<RegionTerm> {
        Some(RegionTerm { shape: self.shape.remapped(map)?, ..self.clone() })
    }

    /// Its low end and its offset (nothing where it has none) as the drawing stands
    /// (`TermNumber::value`): refused where either reads an unknown nothing determines.
    pub fn value(&self, sk: &Sketch) -> Result<(f64, f64), String> {
        Ok((self.lo.value(sk)?, self.grow_value(sk)?))
    }

    /// `value`, `None` where nothing determines either.
    pub fn read(&self, sk: &Sketch) -> Option<(f64, f64)> {
        Some((self.lo.read(sk)?, self.grow_read(sk)?))
    }

    /// Its offset as the drawing stands, nothing where it has none (`TermNumber::value`).
    pub fn grow_value(&self, sk: &Sketch) -> Result<f64, String> {
        self.grow.as_ref().map_or(Ok(0.0), |g| g.value(sk))
    }

    /// `grow_value`, `None` where nothing determines it.
    pub fn grow_read(&self, sk: &Sketch) -> Option<f64> {
        self.grow.as_ref().map_or(Some(0.0), |g| g.read(sk))
    }
}

/// One bound of a region solid: its measure, which way, and its numbers (a length or an angle in
/// radians) — the low end and an interval's high.  `name` is the set the bound came from, where
/// it came from one: what the faces it makes are called.  `grow` is an offset (§6.21, F4): the
/// bound is on the signed distance from the surface the measure's number makes, positive where
/// the measure exceeds it, compared with `grow` — the set grown by it, or shrunk where it is
/// negative.  Only a cone's is kept as one; a distance's is its number moved by it.
#[derive(Clone, Debug, PartialEq)]
pub struct RegionTerm {
    pub shape: RegionShape,
    pub cmp: crate::syntax::Cmp,
    pub lo: TermNumber,
    pub hi: Option<f64>,
    pub grow: Option<TermNumber>,
    pub name: String,
}

/// A region term's number (§6.21): stated, or affine in one of the drawing's unknowns — `c + m ·
/// x` in the kernels' units, `x` the unknown by its name — read once the drawing is solved, as a
/// sphere's radius is where a point stated on it sets it.
#[derive(Clone, Debug, PartialEq)]
pub struct TermNumber {
    pub c: f64,
    pub free: Option<(String, f64)>,
}

impl TermNumber {
    /// A stated number.
    pub fn stated(c: f64) -> TermNumber {
        TermNumber { c, free: None }
    }

    /// What it reads as the drawing stands: refused where it reads an unknown no row of the
    /// drawing holds, which nothing then determines.
    pub fn value(&self, sk: &Sketch) -> Result<f64, String> {
        self.read(sk).ok_or_else(|| {
            let name = self.free.as_ref().map_or("", |(n, _)| n.as_str());
            format!("`{name}` is an unknown nothing determines: state a point on the set it sizes")
        })
    }

    /// `value`, `None` where nothing determines it.
    pub fn read(&self, sk: &Sketch) -> Option<f64> {
        let Some((name, m)) = &self.free else { return Some(self.c) };
        let x = sk.free_vars.get(name).map(|&p| &sk.params[p as usize]).filter(|p| !p.fixed)?;
        Some(self.c + m * x.value)
    }
}

/// A solid, as the document names it.
#[derive(Clone, Debug)]
pub struct SolidE {
    pub def: SolidDef,
    /// What the document calls it — see `FaceE::name`.
    pub name: String,
    pub class: Classes,
}

/// A generating-profile junction, or an envelope intersected by a boundary surface.
#[derive(Clone, Debug)]
pub struct SeamE {
    pub first: EntRef,
    pub second: EntRef,
    pub name: String,
    pub class: Classes,
}

/// A seam between two corner identities, sliced along a named spatial line.
#[derive(Clone, Debug)]
pub struct EdgeE {
    pub seam: u32,
    pub start: u32,
    pub end: u32,
    pub along: u32,
    pub name: String,
    pub class: Classes,
}

/// A spatial corner at the intersection of two seam identities.
#[derive(Clone, Debug)]
pub struct VertexE {
    pub first: u32,
    pub second: u32,
    pub name: String,
    pub class: Classes,
}

/// A source surface or envelope clipped by closed material regions.
#[derive(Clone, Debug)]
pub struct PatchE {
    pub source: EntRef,
    pub inside: Vec<u32>,
    pub outside: Vec<u32>,
    pub name: String,
    pub class: Classes,
}

/// A named implicit envelope. Roll bounds are radians. Evaluated after solving
/// (`envelope::GeneratedEnvelope`) where its source is a revolution's named side, and held to in
/// the solve (`envelope::held`, #283) where the drawing says what its source is before the
/// relations are stated (`held`).
#[derive(Clone, Debug)]
pub struct EnvelopeE {
    /// The named side of a revolution it is generated by, once the solids are built: `None`
    /// before then, and for a set's envelope, which has no such side.
    pub surface: Option<u32>,
    pub motion: u32,
    pub roll: [f64;2],
    pub name: String,
    pub class: Classes,
    /// What a relation holding something to it reads its source as.
    pub held: Held,
}

/// What the solve reads an envelope's source as (#283): a surface of revolution whose numbers are
/// the drawing's, read before any relation is stated.
#[derive(Clone, Debug, PartialEq)]
pub enum Held {
    /// Nothing the solve can read, and why: a relation naming the envelope is refused saying so.
    Not(String),
    /// A revolution's named side: the line or axis it turns about and the profile edge (a line,
    /// an arc or a circle) it turns, drawn in the profile's view.
    Revolution { axis: EntRef, edge: EntRef },
    /// A set's surface, its body applied to the hidden point `probe` (`{envelope}#probe`, held):
    /// the shape what it states there says, once it is stated.
    Set { probe: u32, shape: Option<SetShape> },
}

/// A set's surface as an envelope reads it, off the one relation its body states at its point.
#[derive(Clone, Debug, PartialEq)]
pub struct SetShape {
    pub kind: SetKind,
    /// The line it is about.
    pub axis: u32,
    /// A cone's half angle or a cylinder's radius, in the kernels' units: stated, or affine in an
    /// unknown of the drawing — which the envelope's contacts read as a column, so it is bound
    /// even where no other row reads it (`expr::evaluate`).
    pub number: TermNumber,
    /// What the number stood at as the set's text was read: where that unknown starts.
    pub stood: f64,
    /// A direction the axis did not run along when the shape was read, which says where the
    /// place's turn round it starts: read once, so a place a contact owns means the same turn
    /// from one solve to the next.
    pub reference: [f64; 3],
}

/// Which surface a set is, as an envelope reads it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SetKind {
    /// The angle `number` from the line `axis`, seen from its start (`std.Cone`,
    /// `std.ConeThrough` about its spine).
    Cone,
    /// The distance `number` from the line `axis` (`std.Cylinder`).
    Cylinder,
}

#[derive(Clone, Debug)]
pub struct SurfaceE {
    pub solid: u32,
    pub edge: EntRef,
    /// Angular limits in radians, measured along the source revolution's declared sense.
    /// None retains the entire source sweep. Parameters keep the original source chart.
    pub span: Option<[f64;2]>,
    pub name: String,
    pub class: Classes,
}

/// One shared angular parameter drives every kind. `advance` is a length (model
/// units) travelled along the axis per full turn of that parameter: zero makes a
/// rotation, nonzero a screw, and a translation moves without turning.
#[derive(Clone, Debug)]
pub enum MotionDef {
    Rotation { axis: u32, ratio: f64, phase: f64, advance: f64 },
    /// A turn in a view about a point of it (`motion(about: c, …)`): a rotation about the line
    /// through `centre` square to its view, which is what it reads as in space; in the plane its
    /// centre is a column of whatever is generated under it (`generate.rs`).
    Turn { centre: u32, ratio: f64, phase: f64 },
    Translation { axis: u32, advance: f64 },
    Relative { source: u32, observer: u32 },
}

#[derive(Clone, Debug)]
pub struct MotionE {
    pub def: MotionDef,
    pub name: String,
    pub class: Classes,
    /// The numbers written as measurements of the drawing (`ratio: length(a) / length(b)`),
    /// worked out whenever the motion is read and standing in for `def`'s own: see
    /// `MotionE::rotation`.  Empty for every motion whose numbers are plain.
    pub measured: Vec<crate::measure::MotionMeasure>,
}

impl MotionE {
    /// A rotation's `(ratio, phase, advance)` — phase in radians — on the drawing as it now
    /// stands: `def`'s numbers, with each one written as a measurement worked out afresh.  The
    /// one reader of a rotation's numbers, so a snapshot and a cache key cannot disagree.
    pub fn rotation(&self, sk: &crate::model::Sketch) -> Result<(f64, f64, f64), String> {
        let (ratio, phase, advance) = match self.def {
            MotionDef::Rotation { ratio, phase, advance, .. } => (ratio, phase, advance),
            MotionDef::Turn { ratio, phase, .. } => (ratio, phase, 0.0),
            _ => return Err(format!("`{}` is not a rotation", self.name)),
        };
        let mut out = (ratio, phase, advance);
        for m in &self.measured {
            let v = m.value.value(sk).map_err(|e| format!("`{}`: {e}", self.name))?;
            match m.slot {
                crate::measure::MotionSlot::Ratio => out.0 = v,
                crate::measure::MotionSlot::Phase => out.1 = v.to_radians(),
                crate::measure::MotionSlot::Advance => out.2 = v,
            }
        }
        Ok(out)
    }

    /// A translation's advance, likewise.
    pub fn advance(&self, sk: &crate::model::Sketch) -> Result<f64, String> {
        let MotionDef::Translation { advance, .. } = self.def else {
            return Err(format!("`{}` is not a translation", self.name));
        };
        match self.measured.iter().find(|m| m.slot == crate::measure::MotionSlot::Advance) {
            Some(m) => m.value.value(sk).map_err(|e| format!("`{}`: {e}", self.name)),
            None => Ok(advance),
        }
    }
}

impl SolidE {
    /// Boolean operands only; a through-extent target is a separate evaluation dependency.
    pub fn operands(&self) -> Vec<u32> {
        match &self.def {
            SolidDef::Placed { source, .. } | SolidDef::Swept { source, .. } => vec![*source],
            SolidDef::Prism { .. } | SolidDef::Revolve { .. } | SolidDef::Through { .. }
                | SolidDef::Loft { .. } | SolidDef::Region { .. } => Vec::new(),
            SolidDef::Body { stock, on, through, bound } => {
                let mut v = vec![*stock];
                v.extend(on.iter().copied());
                v.extend(through.iter().copied());
                v.extend(bound.iter().copied());
                v
            }
        }
    }

    /// The faces a sweep makes besides its section's sides, by their own names (#188, the one
    /// table): a prism's caps, a loft's ends and a partial turn's.
    pub fn caps(&self) -> &'static [&'static str] {
        match &self.def {
            SolidDef::Prism { .. } | SolidDef::Through { .. } => &["near", "far"],
            SolidDef::Loft { .. } => &["start", "end"],
            SolidDef::Revolve { sweep, .. } if sweep.value.abs() < std::f64::consts::TAU - 1e-9 => {
                &["start", "end"]
            }
            _ => &[],
        }
    }

    /// The face it is swept from, if it is swept from one.
    pub fn face(&self) -> Option<u32> {
        match &self.def {
            SolidDef::Prism { face, .. } | SolidDef::Revolve { face, .. } | SolidDef::Through { face, .. } | SolidDef::Loft { face, .. } => Some(*face),
            SolidDef::Body { .. } | SolidDef::Placed { .. } | SolidDef::Swept { .. }
                | SolidDef::Region { .. } => None,
        }
    }
}

impl Sketch {
    // -- faces and solids (§6.8, §6.9) --------------------------------------

    /// A face: a loop of edges the document already drew, on the plane they agree about.
    ///
    /// The plane is *read* rather than given, since a face has no points of its own to put
    /// anywhere: it is the one every point of every edge is a member of, and the elaborator has
    /// already refused a loop whose edges disagree (E080).
    pub fn face(&mut self, edges: Vec<EntRef>, names: Vec<String>, name: &str) -> usize {
        let plane = edges
            .first()
            .and_then(|e| self.children(*e).first().copied().or(Some(*e)))
            .and_then(|p| (p.kind == EntKind::Point).then(|| self.plane_of(p.i())).flatten());
        let plane = match plane {
            Some(p) => Some(p),
            None => edges.iter().find_map(|e| {
                self.children(*e).iter().find_map(|c| {
                    (c.kind == EntKind::Point).then(|| self.plane_of(c.i())).flatten()
                })
            }),
        };
        self.faces.push(FaceE {
            edges,
            edge_names: names,
            holes: Vec::new(),
            support: FaceSupport::Plane(plane.map(|p| p as u32)),
            name: name.to_string(),
            class: Classes::default(),
        });
        self.faces.len() - 1
    }

    /// A solid: a face swept, or a term over other solids.  It allocates no parameter, which is
    /// the whole of what "nothing three-dimensional is ever solved for" comes to in code.
    pub fn solid(&mut self, def: SolidDef, name: &str) -> usize {
        self.solids.push(SolidE { def, name: name.to_string(), class: Classes::default() });
        self.solids.len() - 1
    }

    /// Solid `i` as a body, so the body rule can fill it: a body as it stands, and any other
    /// solid made the body over its own sweep — the name keeps its index, so everything reading
    /// it reads the whole object, and the sweep moves to a stock of the same name past the last,
    /// whose index is returned (`None` where `i` was a body already).
    pub fn as_body(&mut self, i: usize) -> Option<usize> {
        if matches!(self.solids[i].def, SolidDef::Body { .. }) {
            return None;
        }
        let stock = self.solids.len();
        self.solids.push(self.solids[i].clone());
        self.solids[i].def =
            SolidDef::Body { stock: stock as u32, on: Vec::new(), through: Vec::new(), bound: Vec::new() };
        Some(stock)
    }

    /// **`tool word body`, one statement of the body rule** (§6.9): `tool` joins the set of
    /// `body`'s that `word` fills (`body` made a body first, `as_body`).  Refused, with the cause
    /// and leaving the sets as they were, for a solid joined to itself and for a statement that
    /// would close a cycle (E041) — asked once the tool is in, since a `through:` reads only what
    /// its body adds.  The elaborator states each statement by it and `preview_body` each tool.
    pub fn join_body(&mut self, body: usize, word: crate::syntax::BodyWord, tool: usize)
        -> Result<(), String> {
        let name = |sk: &Sketch, i: usize| sk.solids[i].name.clone();
        if tool == body {
            return Err(format!("`{0} {1} {0}` joins a solid to itself", name(self, body), word.as_str()));
        }
        self.as_body(body);
        self.solids[body].def.body_set(word).expect("a body").push(tool as u32);
        if crate::solid::reaches(self, body as u32, body as u32) {
            self.solids[body].def.body_set(word).expect("a body").pop();
            return Err(format!("`{}` would be made of itself: `{}` is made of it",
                               name(self, body), name(self, tool)));
        }
        Ok(())
    }

    /// **What the body rule would make of `target`** with `tools` joined to it by `word` (#162
    /// F4), and nothing written: each tool joined by `join_body` — the elaborator's own step, so
    /// the preview is what the statements make and is refused where they would be — on a copy of
    /// the drawing, and the target evaluated there.  The copy keeps what the caches already know
    /// of every operand.  Refused, with the cause, for an index out of range, no tool, a refused
    /// statement and wherever evaluation refuses — a body with a swept operand among them, whose
    /// surface is meshed elsewhere and never for a preview.
    pub fn preview_body(&self, target: usize, word: crate::syntax::BodyWord, tools: &[usize])
        -> Result<std::rc::Rc<crate::solid::EvaluatedSolid>, String> {
        let n = self.solids.len();
        if target >= n || tools.iter().any(|&t| t >= n) {
            return Err("a Boolean to preview names no solid of this drawing".into());
        }
        if tools.is_empty() {
            return Err(format!("nothing to {} `{}`", word.as_str(), self.solids[target].name));
        }
        let mut copy = self.clone();
        for &t in tools {
            copy.join_body(target, word, t)?;
        }
        copy.evaluated_solid(target, crate::solid::ApproximationPolicy::from_unit(0.0))
    }

    /// What the document calls a solid.
    pub fn solid_name(&self, i: usize) -> String {
        self.solids.get(i).map(|s| s.name.clone()).unwrap_or_default()
    }

    /// Validate and evaluate the current pose with an explicit approximation policy.
    /// Different policies coexist; changed geometry/placement discards all old policies.
    pub fn evaluated_solid(
        &self, i: usize, policy: crate::solid::ApproximationPolicy,
    ) -> Result<std::rc::Rc<crate::solid::EvaluatedSolid>, String> {
        // A swept solid's field mesh does not depend on the view's pixel length: one mesh
        // serves every zoom, rather than a refinement per wheel tick.
        let policy = if crate::solid::has_sweep(self, i) { crate::solid::ApproximationPolicy::Mesh } else { policy };
        let key = crate::solid::reads(self, i, 0.0);
        let slot = (i, policy.cache_key());
        if let Some((old, value)) = self.solid_cache.borrow().get(&slot) {
            if *old == key { return value.clone(); }
        }
        let value = crate::solid::EvaluatedSolid::evaluate(self, i, policy).map(std::rc::Rc::new);
        let mut cache = self.solid_cache.borrow_mut();
        cache.retain(|(index, _), (old, _)| *index != i || *old == key);
        // Bound zoom-history memory without evicting report and mesh entries.
        if cache.keys().filter(|(index, (kind, _))| *index == i && *kind == 2).count() >= 8 {
            cache.retain(|(index, (kind, _)), _| *index != i || *kind != 2);
        }
        cache.insert(slot, (key, value.clone()));
        value
    }

    /// Solid `i`'s exact B-rep (`solid::Exact`), or why this kernel does not build it: built once
    /// for the geometry it reads, whatever approximation asks.
    pub fn exact_solid(&self, i: usize) -> Result<std::rc::Rc<crate::solid::Exact>, String> {
        let key = crate::solid::MemoKey::of(self, i);
        self.exact_cache.get_or(i, key, || crate::solid::build_exact(self, i).map(std::rc::Rc::new))
    }

    /// Give swept solid `i` its exact B-rep, built elsewhere (`brep::export::supply_exact`) against
    /// the drawing as it stands now: its evaluation then reads the B-rep, as a static solid's does
    /// (`EvaluatedSolid::from_brep`), until the drawing changes.
    pub fn supply_exact_solid(&self, i: usize, exact: crate::solid::Exact) {
        self.exact_cache.put(i, crate::solid::MemoKey::of(self, i), Ok(std::rc::Rc::new(exact)));
    }

    /// The exact B-rep supplied for swept solid `i` against the drawing as it stands, if any.
    pub fn supplied_exact_solid(&self, i: usize) -> Option<std::rc::Rc<crate::solid::Exact>> {
        self.exact_cache.get(i, &crate::solid::MemoKey::of(self, i)).and_then(Result::ok)
    }

    /// **What a host meshing swept solids elsewhere has to mesh** (`FieldMeshing::Deferred`): the
    /// objects (`overview::objects`) with a continuous sweep among their operands, each with the
    /// key of the drawing it is a surface of (`FieldJob`). A swept solid that is no object is left
    /// out: nothing shows or exports it, so its surface is never asked for.
    pub fn field_jobs(&self) -> Vec<crate::solid::FieldJob> {
        crate::overview::objects(self).into_iter().filter(|&i| self.is_swept(i))
            .map(|i| crate::solid::FieldJob {
                solid: i, name: self.solids[i].name.clone(), key: crate::solid::field_key(self, i),
            })
            .collect()
    }

    /// Give swept solid `i` a surface meshed elsewhere, against the drawing as it stands now.
    /// Refused, with the reason, for an index out of range, a solid with no sweep (whose surface
    /// is its own) or a triangle naming a vertex that is not there — a surface crosses from
    /// another host, and the core checks what it is handed.
    pub fn supply_field(&self, i: usize, surface: crate::solid::FieldSurface) -> Result<(), String> {
        if i >= self.solids.len() { return Err("a supplied surface names no solid of this drawing".into()); }
        if !self.is_swept(i) {
            return Err(format!("`{}`: a supplied surface is for a swept solid, and this one has no sweep",
                self.solids[i].name));
        }
        let n = surface.vertices.len();
        if surface.triangles.iter().flatten().any(|&v| v as usize >= n) {
            return Err(format!("`{}`: a supplied triangle names a vertex past the {n} given", self.solids[i].name));
        }
        // an exact surface is a finished one, and says the face of every triangle
        if let Some(x) = &surface.exact {
            let name = &self.solids[i].name;
            if surface.provisional { return Err(format!("`{name}`: an exact surface is never provisional")); }
            if x.of.len() != surface.triangles.len() {
                return Err(format!("`{name}`: an exact surface names the faces of {} of its {} triangles", x.of.len(),
                    surface.triangles.len()));
            }
            if x.of.iter().any(|&f| f as usize >= x.smooth.len()) {
                return Err(format!("`{name}`: an exact surface's triangle names a face past the {} given", x.smooth.len()));
            }
            if !(x.volume.is_finite()) { return Err(format!("`{name}`: an exact surface's volume is not a number")); }
        }
        let key = crate::solid::reads(self, i, 0.0);
        self.field_surfaces.borrow_mut().insert(i, (key, std::rc::Rc::new(surface)));
        self.solid_cache.borrow_mut().retain(|(index, _), _| *index != i);
        Ok(())
    }

    /// Whether solid `i`'s surface is still being refined elsewhere: false for a solid with no
    /// sweep, answered without evaluating it, and a swept one's supplied surface's word otherwise.
    pub fn field_provisional(&self, i: usize) -> Result<bool, String> {
        if i >= self.solids.len() { return Err("no such solid in this drawing".into()); }
        if !self.is_swept(i) { return Ok(false); }
        // any policy: a swept solid is always its supplied mesh, whatever the unit asked
        self.evaluated_solid(i, crate::solid::ApproximationPolicy::from_unit(0.)).map(|s| s.provisional())
    }

    /// The surface supplied for swept solid `i`, while the drawing still reads as it did.
    pub fn supplied_field(&self, i: usize) -> Option<std::rc::Rc<crate::solid::FieldSurface>> {
        let supplied = self.field_surfaces.borrow();
        let (key, surface) = supplied.get(&i)?;
        (*key == crate::solid::reads(self, i, 0.0)).then(|| surface.clone())
    }

    /// Whether solid `i` has a continuous sweep among its operands, and so a field's surface.
    pub fn is_swept(&self, i: usize) -> bool {
        crate::solid::has_sweep(self, i)
    }

    /// Compatibility output in world coordinates. New queries use `evaluated_solid` so
    /// invalid geometry remains a diagnostic instead of an empty drawing.
    pub fn solid_boundary(&self, i: usize, unit: f64) -> Vec<crate::csg::Piece> {
        self.evaluated_solid(i, crate::solid::ApproximationPolicy::from_unit(unit))
            .map(|s| s.world_boundary()).unwrap_or_default()
    }

    pub fn solid_mesh(&self, i: usize, unit: f64) -> crate::mesh::Mesh {
        self.evaluated_solid(i, crate::solid::ApproximationPolicy::from_unit(unit))
            .map(|s| s.world_mesh()).unwrap_or_else(|_| crate::mesh::grouped(&[]))
    }

    pub fn solid_edges(&self, i: usize, unit: f64) -> Vec<crate::csg::Edge> {
        self.evaluated_solid(i, crate::solid::ApproximationPolicy::from_unit(unit))
            .map(|s| s.world_edges()).unwrap_or_default()
    }
}

/// Which way an extrusion runs from its face, and how thick it is the language's way to say: behind
/// it (`depth: d`, the material the view looks at), in front of it (`from: 0, to: d`), or both
/// ways (`from: -d/2, to: d/2`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Way {
    Behind,
    Front,
    Both,
}

impl Way {
    /// Where an extrusion `d` thick runs along its face's normal, from and to.
    pub fn extents(self, d: f64) -> (f64, f64) {
        match self {
            Way::Behind => (-d, 0.0),
            Way::Front => (0.0, d),
            Way::Both => (-d / 2.0, d / 2.0),
        }
    }

    /// The way and thickness a prism's extents say, where they say one of the three.
    pub fn of(from: f64, to: f64) -> Option<(Way, f64)> {
        if to == 0.0 && from < 0.0 {
            Some((Way::Behind, -from))
        } else if from == 0.0 && to > 0.0 {
            Some((Way::Front, to))
        } else if from == -to && to > 0.0 {
            Some((Way::Both, 2.0 * to))
        } else {
            None
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Way::Behind => "behind",
            Way::Front => "front",
            Way::Both => "both",
        }
    }

    pub fn parse(w: &str) -> Option<Way> {
        [Way::Behind, Way::Front, Way::Both].into_iter().find(|x| x.as_str() == w)
    }
}

impl Sketch {
    /// Re-extrude a prism where it stands, `d` thick the `way` it runs — the preview an
    /// extrusion being sized shows before the source says it (#162 F2).  Evaluated solids are
    /// memoised against every extent they read, so the next mesh is re-cut, and `shape_epoch`
    /// says so to a binding.  False for a solid that is not a prism.
    pub fn set_prism(&mut self, i: usize, way: Way, d: f64) -> bool {
        let (from, to) = way.extents(d);
        match self.solids.get_mut(i).map(|s| &mut s.def) {
            Some(SolidDef::Prism { from: f, to: t, .. }) => {
                *f = Extent::at(from);
                *t = Extent::at(to);
                self.shape_epoch = self.shape_epoch.wrapping_add(1);
                true
            }
            _ => false,
        }
    }

    /// Re-turn a revolution where it stands through `degrees`, signed (positive right-handed
    /// about its axis, `turn_of`'s reading) — the preview a revolution being sized shows before
    /// the source says it (#162 F3), as `set_prism` is an extrusion's.  False for a solid that is
    /// not one, or no turn.
    pub fn set_turn(&mut self, i: usize, degrees: f64) -> bool {
        let Some((how_far, sense)) = turn_of(degrees) else { return false };
        match self.solids.get_mut(i).map(|s| &mut s.def) {
            Some(SolidDef::Revolve { sweep, sense: s, .. }) => {
                *sweep = match how_far {
                    None => Extent { text: String::new(), value: std::f64::consts::TAU },
                    Some(d) => Extent { text: crate::syntax::num(d), value: d.to_radians() },
                };
                *s = sense;
                self.shape_epoch = self.shape_epoch.wrapping_add(1);
                true
            }
            _ => false,
        }
    }
}

/// A turn said as signed degrees, positive right-handed about its axis, as the language says one:
/// how far in degrees — `None` for a whole turn, at 360 or past it, which a revolution writes as
/// no `sweep:` — and which way.  `None` for no turn at all.  The one reading, shared by the
/// preview (`Sketch::set_turn`) and the edit (`edit::set_turn`).
pub fn turn_of(degrees: f64) -> Option<(Option<f64>, Sense)> {
    if !degrees.is_finite() || degrees == 0.0 {
        return None;
    }
    let sense = if degrees < 0.0 { Sense::Cw } else { Sense::Ccw };
    Some(((degrees.abs() < 360.0).then_some(degrees.abs()), sense))
}

/// How far off its profile's plane a revolution's axis may lie and still be read as in it: a
/// direction's sine, and a place's distance over the drawing's reach.  Coarser than `views`'
/// `ON_PLANE`, which judges held datums: this one judges an axis a solve may have placed.
const AXIS_IN_PLANE: f64 = 1e-7;

impl Sketch {
    /// A revolution's axis in its profile's own plane coordinates, from and toward: a line's two
    /// points as drawn there, or an axis's foot and one unit along it, projected — `None` where
    /// an axis does not lie in the plane (a line is held to it by membership, at elaboration).
    pub fn revolve_axis(&self, face: u32, axis: EntRef) -> Option<((f64, f64), (f64, f64))> {
        match axis.kind {
            EntKind::Line => {
                let l = self.lines.get(axis.i())?;
                Some((self.point_xy(l.p1 as usize), self.point_xy(l.p2 as usize)))
            }
            EntKind::Axis => self.axis_in(self.faces.get(face as usize)?.plane().ok()?, axis.i()),
            _ => None,
        }
    }

    /// Axis `ax` in plane `plane`'s own coordinates (`None` the page), from its foot toward one
    /// unit along it — `None` where it does not lie in the plane.
    pub fn axis_in(&self, plane: Option<u32>, ax: usize) -> Option<((f64, f64), (f64, f64))> {
        use crate::space::{add, dot, normalised, sub};
        let x = self.axes.get(ax)?;
        let value = |q: [u32; 3]| q.map(|q| self.params[q as usize].value);
        let (d, a) = (value(x.d), value(x.a));
        let basis = match plane {
            Some(p) => { self.planes.get(p as usize)?; self.basis(p as usize) }
            None => crate::plane::Basis::page(),
        };
        let n = basis.normal();
        let unit = normalised(d)?;
        if dot(unit, n).abs() > AXIS_IN_PLANE
            || dot(sub(a, basis.o), n).abs() > AXIS_IN_PLANE * self.extent() {
            return None;
        }
        Some((basis.view_coords(a), basis.view_coords(add(a, unit))))
    }
}
