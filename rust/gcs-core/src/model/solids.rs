//! The spatial entities a drawing names: faces and their loops, solids and what they are
//! swept or made of, claims and bearings about them, the pictures asked of them, and the
//! analytic surfaces, envelopes and motions over them.

use super::*;

/// An inner boundary and the source names of its swept sides.
#[derive(Clone, Debug)]
pub struct FaceLoop {
    pub edges: Vec<EntRef>,
    pub edge_names: Vec<String>,
}

/// A region bounded by existing edges, on the plane it inherits from its points' membership.
#[derive(Clone, Debug)]
pub struct FaceE {
    /// The loop, in traversal order: lines, arcs and circles.
    pub edges: Vec<EntRef>,
    /// What the document calls each edge, in the same order — the name the face swept from it
    /// is reached by (`block.side_l`).  Kept beside the references because a solid's face path
    /// is spelled out of the *source's* words, and an anonymous edge has none to spell.
    pub edge_names: Vec<String>,
    /// Strictly contained, disjoint inner boundaries.
    pub holes: Vec<FaceLoop>,
    /// The inherited profile plane: `None` is the page.
    pub plane: Option<u32>,
    /// What the document calls it.  A solid's faces are reached by *path* — `body.bore.wall` —
    /// so unlike every other kind a face and a solid carry their own name: the path is the
    /// naming mechanism, and a report that could not spell one would have nothing to say.
    pub name: String,
    pub class: Classes,
}

impl FaceE {
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

/// The scope a face family's station is drawn under — `#f12.` for the running number `#f12.s`
/// (#243 L1): every name of the station's instance starts with it.
pub fn station_prefix(held: &str) -> String {
    format!("{}.", held.rsplit_once('.').map_or("", |(p, _)| p))
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
    /// **A face family** (#243 L1, `solid(I(…).f over s in (a, b))`): the solid the face `face`
    /// of a station sweeps as the running number `held` (`Sketch::held`) runs from `from` to
    /// `to`.  The station is drawn once, hidden — `station`, every entity its instance made —
    /// at the middle of the interval, and `solid::family` solves it again at every other.  Its
    /// caps are `start` and `end`, its walls the section's edges by their paths in the instance.
    ///
    /// Over a chain (#243 L4) the station is drawn once per link the interval reaches, each a
    /// leg (`legs`, in order; `face` and `held` the first's, `station` every leg's entities):
    /// link `k`'s while the running number is in `(k, k + 1)`, its own running number the place
    /// on that link's run.  Empty for a family over one path.
    Family { face: u32, station: Vec<EntRef>, held: String, from: Extent, to: Extent,
             legs: Vec<FamilyLeg> },
}

/// One leg of a face family over a chain (#243 L4): the face its station sweeps, its running
/// number, the index of its link — where the leg's number starts on the family's — and its
/// station's entities in the order of their names under the leg, so each leg's `i`th is every
/// other's (the stations are one instance drawn once a link): where one link meets the next,
/// the next leg's station starts where this one's ends.
#[derive(Clone, Debug)]
pub struct FamilyLeg {
    pub face: u32,
    pub held: String,
    pub at: f64,
    pub ents: Vec<EntRef>,
}

/// A leg as a family steps it: its face, its running number, where that number starts on the
/// family's (`at`), and the leg's own interval in it.
#[derive(Clone, Debug)]
pub struct Leg<'a> {
    pub face: u32,
    pub held: &'a str,
    pub at: f64,
    pub from: f64,
    pub to: f64,
}

impl SolidDef {
    /// A face family's legs, in order of its running number: over one path the one leg, its
    /// interval the family's; over a chain each link's part of it, in the leg's own number.
    pub fn legs(&self) -> Vec<Leg<'_>> {
        let SolidDef::Family { face, held, from, to, legs, .. } = self else { return Vec::new() };
        let (a, b) = (from.value, to.value);
        if legs.is_empty() {
            return vec![Leg { face: *face, held, at: 0.0, from: a, to: b }];
        }
        legs.iter().map(|l| Leg {
            face: l.face,
            held: &l.held,
            at: l.at,
            from: a.max(l.at) - l.at,
            to: b.min(l.at + 1.0) - l.at,
        }).collect()
    }
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

/// A solid, as the document names it.
#[derive(Clone, Debug)]
pub struct SolidE {
    pub def: SolidDef,
    /// What the document calls it — see `FaceE::name`.
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

/// **The meshing of two flanks** (#283, step 3): `contact := meshing(pinion_flank, gear_flank,
/// under: mesh, from: a, to: b)` — tooth contact analysis, the first flank turning with `mesh`'s
/// observer (the pinion's turn), the second with its source (the gear's).  Held to in the solve by
/// a point on its path of contact (`CKind::PointOnMeshing`), which owns the phase, the lag and a
/// place on each flank, and traced over the phase after it (`envelope::meshing::Meshing`).  Phase
/// bounds are radians.
#[derive(Clone, Debug)]
pub struct MeshingE {
    pub flanks: [Flank; 2],
    pub motion: u32,
    pub roll: [f64;2],
    pub name: String,
    pub class: Classes,
    /// The first flank's ease-off, where it is eased (#345).
    pub ease: Option<Ease>,
}

/// **Ease-off** (#345 S2): `meshing(…, ease: M, along: l, profile: κₚ, lengthwise: κₗ)` relieves
/// the first flank along its normal by `½ κₗ s² + ½ κₚ r²`, `s` how far its point is from `M`
/// along `l` and `r` how far across it (`envelope::meshing::Ease`).
#[derive(Clone, Debug, PartialEq)]
pub struct Ease {
    /// The point it is eased about.
    pub about: u32,
    /// The line whose direction is lengthwise.
    pub along: u32,
    /// The curvatures across the line and along it, per length: stated, or affine in an unknown
    /// of the drawing, which the contacts read as a column.
    pub numbers: [TermNumber; 2],
}

impl Ease {
    /// The unknowns its numbers read, and where each starts: where it was seeded, else zero.
    pub fn unknowns(&self, declared: &BTreeMap<String, Declared>) -> Vec<(String, f64)> {
        self.numbers.iter().filter_map(|n| {
            let (name, _) = n.free.clone().filter(|(_, m)| *m != 0.0)?;
            let seed = declared.get(&name).and_then(|d| d.seed).unwrap_or(0.0);
            Some((name, seed))
        }).collect()
    }
}

/// **A traced body in space** (#326): a point of a component as one or two of its numeric
/// formals run, its body solved in space (`traced::Body`).  Held to by a contact owning a place
/// on it (`CKind::PointOnTraced` and the four at a point already on it), never drawn.
#[derive(Clone, Debug)]
pub struct TracedE {
    pub body: crate::traced::Body,
    /// What the component was given, in its formals' order: points (their lifts), lines (their
    /// ends' lifts) and motions (their axes' ends), the columns of every contact after its place.
    pub args: Vec<EntRef>,
    /// The constant numbers it was given, in the body's order.
    pub values: Vec<f64>,
    /// The interval each formal runs over, as written (degrees for an angle); a curve's second
    /// none.
    pub domain: [[f64; 2]; 2],
    /// Where a march to a place starts, the formals' anchor.
    pub home: [f64; 2],
    pub name: String,
    pub class: Classes,
}

impl TracedE {
    /// How many formals run: one for a curve, two for a surface.
    pub fn formals(&self) -> usize {
        self.body.n_formals()
    }
}

/// A flank a meshing turns: an envelope in space (generated), or a source surface standing in its
/// member (formed) — a cone or a cylinder set, or a revolution's named side — read as an
/// envelope's source is (`Held`).
#[derive(Clone, Debug, PartialEq)]
pub enum Flank {
    Envelope(u32),
    Formed(Held),
    /// A surface traced in space, as its member sees it (#326): the flank stated by its contact
    /// alone.
    Traced(u32),
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
    /// A plane's point, from which its number runs along the axis (#326); `None` for a
    /// revolution.
    pub origin: Option<u32>,
    /// A cone's half angle, a cylinder's radius or a plane's distance along its axis, in the kernels' units: stated, or affine in an
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

impl SetShape {
    /// The unknown its number reads, where it reads one, and where that unknown starts: where it
    /// was seeded (`half: hint(20deg)`), else where the set's text stood.
    pub fn unknown(&self, declared: &BTreeMap<String, Declared>) -> Option<(String, f64)> {
        let (name, m) = self.number.free.clone().filter(|(_, m)| *m != 0.0)?;
        let seed = declared.get(&name).and_then(|d| d.seed);
        Some((name, seed.unwrap_or((self.stood - self.number.c) / m)))
    }
}

/// Which surface a set is, as an envelope reads it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SetKind {
    /// The angle `number` from the line `axis`, seen from its start (`std.Cone`,
    /// `std.ConeThrough` about its spine).
    Cone,
    /// The distance `number` from the line `axis` (`std.Cylinder`).
    Cylinder,
    /// The distance `number` along the line `axis` from the point `origin` (`std.HalfSpace`'s
    /// face, #326): a plane square to the line.
    Plane,
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
    /// The inverse of motion `of`, minted where a chain is lowered (`Sketch::chain`): a joint's
    /// inverse turns and slides back (its ratio, phase and advance negated) and a relative's
    /// swaps its two sides, so every reader reads it as the steps it already knows.
    Inverse { of: u32 },
    /// A frame's joint whose holding relations name no line or centre the kinds above are written
    /// over (#383): turned `ratio` a radian about the axis `axis` (an `AxisE`) as the solve leaves
    /// it, advancing `advance` along it a turn.  Read after the solve (`Family::read`); a reader
    /// that needs a motion's kind before it (a profile in the plane, an envelope the solve holds)
    /// refuses it, naming what would settle it.
    Joint { axis: u32, ratio: f64, advance: f64 },
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

impl crate::model::Sketch {
    /// **A chain** of two motions (`docs/frames-plan.md`, stage 1): `inner` carried by `outer`,
    /// the pose `outer(τ)·inner(τ)` — a planet's turn on its arm's.  Lowered as it is pushed, so
    /// no reader learns a new step: two joints about one axis (one line, or one centre) whose
    /// numbers are stated fold to one joint, their ratios, phases and advances adding — a gear
    /// train on one shaft reads as the single turn `Family::screw` and the generating classes
    /// admit — and any other pair is the relative of `inner` seen from `outer`'s minted inverse,
    /// `inv(outer)⁻¹·inner`.  Returns the chain's index in `motions`.
    pub fn chain(&mut self, outer: usize, inner: usize, name: &str) -> Result<usize, String> {
        let n = self.motions.len();
        if outer >= n || inner >= n {
            return Err("no such motion".into());
        }
        let (a, b) = (&self.motions[outer], &self.motions[inner]);
        let plain = a.measured.is_empty() && b.measured.is_empty();
        let folded = match (&a.def, &b.def) {
            (MotionDef::Rotation { axis: l, ratio: r1, phase: p1, advance: a1 },
             MotionDef::Rotation { axis: k, ratio: r2, phase: p2, advance: a2 }) if plain && l == k =>
                Some(MotionDef::Rotation { axis: *l, ratio: r1 + r2, phase: p1 + p2, advance: a1 + a2 }),
            (MotionDef::Turn { centre: c, ratio: r1, phase: p1 },
             MotionDef::Turn { centre: d, ratio: r2, phase: p2 }) if plain && c == d =>
                Some(MotionDef::Turn { centre: *c, ratio: r1 + r2, phase: p1 + p2 }),
            (MotionDef::Translation { axis: l, advance: a1 },
             MotionDef::Translation { axis: k, advance: a2 }) if plain && l == k =>
                Some(MotionDef::Translation { axis: *l, advance: a1 + a2 }),
            _ => None,
        };
        let class = Classes::default();
        let def = match folded {
            Some(def) => def,
            None => {
                let inverse = self.motions.len();
                self.motions.push(MotionE { def: MotionDef::Inverse { of: outer as u32 },
                    name: format!("{name}⁻"), class: class.clone(), measured: Vec::new() });
                MotionDef::Relative { source: inner as u32, observer: inverse as u32 }
            }
        };
        self.motions.push(MotionE { def, name: name.to_string(), class, measured: Vec::new() });
        Ok(self.motions.len() - 1)
    }
}

impl SolidE {
    /// Boolean operands only; a through-extent target is a separate evaluation dependency.
    pub fn operands(&self) -> Vec<u32> {
        match &self.def {
            SolidDef::Placed { source, .. } | SolidDef::Swept { source, .. } => vec![*source],
            SolidDef::Prism { .. } | SolidDef::Revolve { .. } | SolidDef::Through { .. }
                | SolidDef::Region { .. } | SolidDef::Family { .. } => Vec::new(),
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
    /// table): a prism's caps, a face family's ends and a partial turn's.
    pub fn caps(&self) -> &'static [&'static str] {
        match &self.def {
            SolidDef::Prism { .. } | SolidDef::Through { .. } => &["near", "far"],
            SolidDef::Family { .. } => &["start", "end"],
            SolidDef::Revolve { sweep, .. } if sweep.value.abs() < std::f64::consts::TAU - 1e-9 => {
                &["start", "end"]
            }
            _ => &[],
        }
    }

    /// The face it is swept from, if it is swept from one.
    pub fn face(&self) -> Option<u32> {
        match &self.def {
            SolidDef::Prism { face, .. } | SolidDef::Revolve { face, .. } | SolidDef::Through { face, .. } => Some(*face),
            // a family's face is its station's, at one station of many: no profile to sweep
            SolidDef::Body { .. } | SolidDef::Placed { .. } | SolidDef::Swept { .. }
                | SolidDef::Region { .. } | SolidDef::Family { .. } => None,
        }
    }
}
