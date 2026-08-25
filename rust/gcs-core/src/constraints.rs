//! Constraint types: entities → a local parameter tuple, constants and a kernel.
//!
//! A constraint is `(kind, args)` where `args` are the constructor arguments in `spec` order.
//! `spec` drives everything reflective — JSON I/O, the constraint list, value editing, the
//! toolbar applier, duplicate detection and the witness's dimension jitter — so a new type is
//! covered everywhere as soon as it declares one.
//!
//! Residual forms follow the program: distance uses |p−q|² − d² (no sqrt), parallel is a 2×2
//! determinant, angle a dot/cross combination, tangency a signed distance minus the radius with
//! a chirality flag fixed at construction.

use crate::expr::Free;
use crate::kernels::{self, K};
use crate::model::{EntKind, EntRef, Sketch};

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum CKind {
    Coincident,
    Distance,
    Midpoint,
    DragTarget,
    Horizontal,
    Vertical,
    Parallel,
    Perpendicular,
    Angle,
    ParallelDistance,
    EqualLength,
    PointOnLine,
    PointLineDistance,
    PointOnCircle,
    Radius,
    EqualRadius,
    AnnularDistance,
    TangentLineCircle,
    TangentCircleCircle,
    TangentArcLine,
    TangentLineCircleAt,
    Symmetric,
    PointOnSpline,
    SplineTangentLine,
    SplineCurvature,
    HorizontalPoints,
    VerticalPoints,
    HorizontalDistance,
    VerticalDistance,
    PointOnEllipse,
    EllipseTangentLine,
    EllipseCurvature,
    /// A point on a curve written in the language.  The same shape as `PointOnSpline` — two
    /// residuals against one owned parameter, so the net one equation "a point lies on a curve"
    /// is worth — but the curve is an expression rather than a basis, so the kernel that
    /// evaluates it is chosen per *definition*, not per type.
    PointOnCurve,
}

/// Every concrete constraint type, in the order the registry lists them.
pub const ALL_KINDS: [CKind; 33] = [
    CKind::Coincident,
    CKind::Distance,
    CKind::Midpoint,
    CKind::DragTarget,
    CKind::Horizontal,
    CKind::Vertical,
    CKind::Parallel,
    CKind::Perpendicular,
    CKind::Angle,
    CKind::ParallelDistance,
    CKind::EqualLength,
    CKind::PointOnLine,
    CKind::PointLineDistance,
    CKind::PointOnCircle,
    CKind::Radius,
    CKind::EqualRadius,
    CKind::AnnularDistance,
    CKind::TangentLineCircle,
    CKind::TangentCircleCircle,
    CKind::TangentArcLine,
    CKind::TangentLineCircleAt,
    CKind::Symmetric,
    CKind::PointOnSpline,
    CKind::SplineTangentLine,
    CKind::SplineCurvature,
    CKind::HorizontalPoints,
    CKind::VerticalPoints,
    CKind::HorizontalDistance,
    CKind::VerticalDistance,
    CKind::PointOnEllipse,
    CKind::EllipseTangentLine,
    CKind::EllipseCurvature,
    CKind::PointOnCurve,
];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SpecKind {
    Point,
    Line,
    Circle,
    Arc,
    CircleOrArc,
    Spline,
    Ellipse,
    /// A curve written in the language — see `model::CurveDef`.
    Curve,
    Length,
    Angle,
    Float,
    Int,
    Str,
    Bool,
    /// A hidden unknown the constraint owns — the curve parameter a contact sits at.  It is not
    /// a value a person writes: the slot holds a seed number until `Sketch::add` allocates the
    /// Param and rewrites it to `Arg::Param`, after which the solver moves it like any other.
    Param,
}

impl SpecKind {
    pub fn is_entity(self) -> bool {
        matches!(
            self,
            SpecKind::Point
                | SpecKind::Line
                | SpecKind::Circle
                | SpecKind::Arc
                | SpecKind::CircleOrArc
                | SpecKind::Spline
                | SpecKind::Ellipse
                | SpecKind::Curve
        )
    }

    pub fn is_dimension(self) -> bool {
        matches!(self, SpecKind::Length | SpecKind::Angle)
    }

    /// A slot holding an unknown of the constraint's own.
    pub fn is_param(self) -> bool {
        self == SpecKind::Param
    }

    pub fn as_str(self) -> &'static str {
        match self {
            SpecKind::Point => "point",
            SpecKind::Line => "line",
            SpecKind::Circle => "circle",
            SpecKind::Arc => "arc",
            SpecKind::CircleOrArc => "circle_or_arc",
            SpecKind::Spline => "spline",
            SpecKind::Ellipse => "ellipse",
            SpecKind::Curve => "curve",
            SpecKind::Length => "length",
            SpecKind::Angle => "angle",
            SpecKind::Float => "float",
            SpecKind::Int => "int",
            SpecKind::Str => "str",
            SpecKind::Bool => "bool",
            SpecKind::Param => "param",
        }
    }
}

use SpecKind as S;

type Spec = &'static [(&'static str, SpecKind)];

impl CKind {
    pub fn name(self) -> &'static str {
        match self {
            CKind::Coincident => "Coincident",
            CKind::Distance => "Distance",
            CKind::Midpoint => "Midpoint",
            CKind::DragTarget => "DragTarget",
            CKind::Horizontal => "Horizontal",
            CKind::Vertical => "Vertical",
            CKind::Parallel => "Parallel",
            CKind::Perpendicular => "Perpendicular",
            CKind::Angle => "Angle",
            CKind::ParallelDistance => "ParallelDistance",
            CKind::EqualLength => "EqualLength",
            CKind::PointOnLine => "PointOnLine",
            CKind::PointLineDistance => "PointLineDistance",
            CKind::PointOnCircle => "PointOnCircle",
            CKind::Radius => "Radius",
            CKind::EqualRadius => "EqualRadius",
            CKind::AnnularDistance => "AnnularDistance",
            CKind::TangentLineCircle => "TangentLineCircle",
            CKind::TangentCircleCircle => "TangentCircleCircle",
            CKind::TangentArcLine => "TangentArcLine",
            CKind::TangentLineCircleAt => "TangentLineCircleAt",
            CKind::Symmetric => "Symmetric",
            CKind::PointOnSpline => "PointOnSpline",
            CKind::SplineTangentLine => "SplineTangentLine",
            CKind::SplineCurvature => "SplineCurvature",
            CKind::HorizontalPoints => "HorizontalPoints",
            CKind::VerticalPoints => "VerticalPoints",
            CKind::HorizontalDistance => "HorizontalDistance",
            CKind::VerticalDistance => "VerticalDistance",
            CKind::PointOnEllipse => "PointOnEllipse",
            CKind::EllipseTangentLine => "EllipseTangentLine",
            CKind::EllipseCurvature => "EllipseCurvature",
            CKind::PointOnCurve => "PointOnCurve",
        }
    }

    pub fn from_name(s: &str) -> Option<CKind> {
        ALL_KINDS.iter().copied().find(|k| k.name() == s)
    }

    pub fn spec(self) -> Spec {
        match self {
            CKind::Coincident => &[("p", S::Point), ("q", S::Point)],
            CKind::Distance => &[("p", S::Point), ("q", S::Point), ("d", S::Length)],
            CKind::Midpoint => &[("p", S::Point), ("line", S::Line)],
            CKind::DragTarget => {
                &[("p", S::Point), ("tx", S::Float), ("ty", S::Float), ("weight", S::Float)]
            }
            CKind::Horizontal | CKind::Vertical => &[("line", S::Line)],
            // the same statement about the segment between two points, with no line drawn there
            CKind::HorizontalPoints | CKind::VerticalPoints => {
                &[("p", S::Point), ("q", S::Point)]
            }
            // the run and the rise between two points: what a drawing dimensions when it wants
            // an ordinate rather than a length.  Signed from p to q, so the pair is not
            // commutative — swapping the points negates the number.
            CKind::HorizontalDistance | CKind::VerticalDistance => {
                &[("p", S::Point), ("q", S::Point), ("d", S::Length)]
            }
            CKind::Parallel | CKind::Perpendicular | CKind::EqualLength => {
                &[("l1", S::Line), ("l2", S::Line)]
            }
            CKind::Angle => &[("l1", S::Line), ("l2", S::Line), ("theta", S::Angle)],
            CKind::ParallelDistance => &[("l1", S::Line), ("l2", S::Line), ("d", S::Length)],
            CKind::PointOnLine => &[("p", S::Point), ("line", S::Line)],
            CKind::PointLineDistance => &[("p", S::Point), ("line", S::Line), ("d", S::Length)],
            CKind::PointOnCircle => &[("p", S::Point), ("circle", S::CircleOrArc)],
            CKind::Radius => &[("circle", S::CircleOrArc), ("r", S::Length)],
            CKind::EqualRadius => &[("c1", S::CircleOrArc), ("c2", S::CircleOrArc)],
            CKind::AnnularDistance => {
                &[("c1", S::CircleOrArc), ("c2", S::CircleOrArc), ("d", S::Length)]
            }
            CKind::TangentLineCircle => {
                &[("line", S::Line), ("circle", S::CircleOrArc), ("side", S::Int)]
            }
            CKind::TangentCircleCircle => {
                &[("c1", S::CircleOrArc), ("c2", S::CircleOrArc), ("external", S::Bool)]
            }
            CKind::TangentArcLine => &[("arc", S::Arc), ("line", S::Line), ("at", S::Str)],
            // tangency *at* the line's own endpoint ("p1" or "p2"), for an endpoint the user
            // has put on the circle: the radius is perpendicular to the line there.  The pair
            // (PointOnCircle, TangentLineCircle) says the same thing with a double root — its
            // Jacobian is rank-deficient at every solution, and the contact "swims" along the
            // line to first order — so the app states this instead whenever the tangency's
            // contact is a line end that is already on the circle.
            CKind::TangentLineCircleAt => {
                &[("line", S::Line), ("circle", S::CircleOrArc), ("at", S::Str)]
            }
            CKind::Symmetric => &[("p", S::Point), ("q", S::Point), ("line", S::Line)],
            CKind::PointOnSpline => &[("p", S::Point), ("spline", S::Spline), ("t", S::Param)],
            CKind::PointOnCurve => &[("p", S::Point), ("curve", S::Curve), ("u", S::Param)],
            CKind::SplineTangentLine => {
                &[("spline", S::Spline), ("line", S::Line), ("t", S::Param)]
            }
            CKind::SplineCurvature => {
                &[("spline", S::Spline), ("circle", S::CircleOrArc), ("t", S::Param)]
            }
            CKind::PointOnEllipse => {
                &[("p", S::Point), ("ellipse", S::Ellipse), ("t", S::Param)]
            }
            CKind::EllipseTangentLine => {
                &[("ellipse", S::Ellipse), ("line", S::Line), ("t", S::Param)]
            }
            CKind::EllipseCurvature => {
                &[("ellipse", S::Ellipse), ("circle", S::CircleOrArc), ("t", S::Param)]
            }
        }
    }

    /// The value an omitted argument takes.  One table, read by the JSON path and by both
    /// bindings, so a default can never drift between them.
    pub fn default_arg(self, i: usize) -> Arg {
        match (self, i) {
            (CKind::DragTarget, 3) => Arg::Num(1.0),
            (CKind::TangentCircleCircle, 2) => Arg::Bool(true),
            (CKind::TangentArcLine, 2) => Arg::Str("start".to_string()),
            (CKind::TangentLineCircleAt, 2) => Arg::Str("p1".to_string()),
            (CKind::TangentLineCircle, 2) => Arg::Int(1),
            _ => match self.spec()[i].1 {
                SpecKind::Int => Arg::Int(0),
                SpecKind::Bool => Arg::Bool(false),
                SpecKind::Str => Arg::Str(String::new()),
                _ => Arg::Num(0.0),
            },
        }
    }

    /// Arguments the core reads off the current geometry when the caller leaves them out: which
    /// side of a line a circle is tangent to, and whether two circles touch outside or inside.
    /// The registry publishes a null default for these so a binding cannot substitute a constant
    /// and quietly pick the wrong branch — `default_arg` is the fallback when there is no sketch.
    pub fn infers_arg(self, i: usize) -> bool {
        // a hidden unknown is always read off the geometry: nobody types a curve parameter
        self.spec()[i].1.is_param()
            || matches!((self, i), (CKind::TangentLineCircle, 2) | (CKind::TangentCircleCircle, 2))
    }

    /// The spec slots holding an unknown of this kind's own, as (index, name).  On the kind,
    /// not the constraint: it reads nothing else, and the JSON paths ask before they have one.
    pub fn param_slots(self) -> Vec<(usize, &'static str)> {
        self.spec()
            .iter()
            .enumerate()
            .filter(|(_, (_, k))| k.is_param())
            .map(|(i, (n, _))| (i, *n))
            .collect()
    }

    /// The spec slots a contact on a parametric entity of kind `of` is made of: which argument
    /// names the entity and which holds the parameter along it.  Read off the spec, so a new
    /// kind of contact is covered by declaring one — there is no table of kinds here to forget
    /// to extend.
    fn contact_on(self, of: SpecKind) -> Option<(usize, usize)> {
        let spec = self.spec();
        let e = spec.iter().position(|&(_, k)| k == of)?;
        let t = spec.iter().position(|&(_, k)| k.is_param())?;
        Some((e, t))
    }

    /// A contact on a *spline*.  The two families are asked separately on purpose, and this is
    /// the one that gates the span machinery: a spline contact addresses a span, is clamped to
    /// its knots and carries its span in the topology key, and an ellipse contact must do none
    /// of those — its parameter is periodic and its columns never move at compile time.  A
    /// caller wanting only "does this run along something, and how fast" asks `param_scale`.
    pub fn contact_slots(self) -> Option<(usize, usize)> {
        self.contact_on(SpecKind::Spline)
    }

    /// A contact on an *ellipse* — see `contact_slots` for why the two are not one question.
    pub fn ellipse_contact_slots(self) -> Option<(usize, usize)> {
        self.contact_on(SpecKind::Ellipse)
    }

    /// Carries a dimension — a length or angle the user can edit.  A redundancy among dimensioned
    /// constraints is fragile (the next edit makes it a conflict); one among pure relations is a
    /// theorem that holds on every solution and can never be broken.
    pub fn has_dimension(self) -> bool {
        self.spec().iter().any(|&(_, k)| k.is_dimension())
    }

    /// Holds two things in *contact*.  Where the contact point is also pinned — a line end on
    /// the circle its line is tangent to — the pair is a double root: rank-deficient at every
    /// solution though nothing can move.  That is the one thing the second-order screen looks
    /// for, so a sketch with no tangency in it can skip the screen and its solves entirely.
    ///
    /// Exhaustive on purpose: a new contact type stops the build here and has to say whether
    /// the screen should look at it.
    pub fn is_tangency(self) -> bool {
        match self {
            CKind::TangentLineCircle
            | CKind::TangentCircleCircle
            | CKind::TangentArcLine
            | CKind::TangentLineCircleAt
            | CKind::SplineTangentLine
            | CKind::SplineCurvature
            | CKind::EllipseTangentLine
            | CKind::EllipseCurvature => true,
            CKind::Coincident
            | CKind::Distance
            | CKind::Midpoint
            | CKind::DragTarget
            | CKind::Horizontal
            | CKind::Vertical
            | CKind::Parallel
            | CKind::Perpendicular
            | CKind::Angle
            | CKind::ParallelDistance
            | CKind::EqualLength
            | CKind::PointOnLine
            | CKind::PointLineDistance
            | CKind::PointOnCircle
            | CKind::Radius
            | CKind::EqualRadius
            | CKind::AnnularDistance
            | CKind::Symmetric
            | CKind::PointOnSpline
            // a point on a curve is a contact, not a tangency: it has no double root for the
            // second-order screen to look for
            | CKind::PointOnCurve
            | CKind::PointOnEllipse
            | CKind::HorizontalPoints
            | CKind::VerticalPoints
            | CKind::HorizontalDistance
            | CKind::VerticalDistance => false,
        }
    }

    /// Types that do not have to be satisfied — a drag target compromises, it does not hold.
    pub fn soft_by_default(self) -> bool {
        self == CKind::DragTarget
    }

    /// The first two spec entities may be swapped without changing the relation.
    pub fn commutative(self) -> bool {
        matches!(
            self,
            CKind::Coincident
                | CKind::Distance
                | CKind::Parallel
                | CKind::Perpendicular
                | CKind::EqualLength
                | CKind::EqualRadius
                | CKind::TangentCircleCircle
                | CKind::Symmetric
                | CKind::HorizontalPoints
                | CKind::VerticalPoints
        )
    }

    /// The static kernel a type evaluates through.
    ///
    /// A curve contact has none: the expressions it runs are the *definition's*, so its kernel is
    /// synthesised per definition by `kernels::curve_kernel` and chosen by
    /// `Constraint::kernel_id_in`, which — unlike this — can see the sketch.  Asking here is a
    /// mistake the type system cannot catch, so it says so.
    pub fn kernel(self) -> K {
        match self {
            CKind::PointOnCurve => {
                panic!("a curve contact's kernel belongs to its definition, not its type")
            }
            CKind::Coincident => K::Coincident,
            CKind::Distance => K::Distance,
            CKind::Midpoint => K::Midpoint,
            CKind::DragTarget => K::Drag,
            CKind::Horizontal => K::Horizontal,
            CKind::Vertical => K::Vertical,
            CKind::Parallel => K::Parallel,
            CKind::Perpendicular => K::Perpendicular,
            CKind::Angle => K::Angle,
            CKind::ParallelDistance => K::ParallelDistance,
            CKind::EqualLength => K::EqualLength,
            CKind::PointOnLine => K::PointOnLine,
            CKind::PointLineDistance => K::PointLineDistance,
            CKind::PointOnCircle => K::PointOnCircle,
            CKind::Radius => K::Radius,
            CKind::EqualRadius => K::EqualRadius,
            CKind::AnnularDistance => K::AnnularDistance,
            CKind::TangentLineCircle => K::TangentLineCircle,
            CKind::TangentCircleCircle => K::TangentCircleCircle,
            CKind::TangentArcLine => K::TangentArcLine,
            // the arc kernel unchanged: its columns were always a contact point, a centre and a
            // line, and a circle's contact is the line's own endpoint
            CKind::TangentLineCircleAt => K::TangentArcLine,
            CKind::Symmetric => K::Symmetric,
            CKind::PointOnSpline => K::PointOnSpline,
            CKind::SplineTangentLine => K::SplineTangentLine,
            CKind::SplineCurvature => K::SplineCurvature,
            // the same kernels: their four columns are already two points' coordinates
            CKind::HorizontalPoints => K::Horizontal,
            CKind::VerticalPoints => K::Vertical,
            CKind::HorizontalDistance => K::HorizontalDistance,
            CKind::VerticalDistance => K::VerticalDistance,
            CKind::PointOnEllipse => K::PointOnEllipse,
            CKind::EllipseTangentLine => K::EllipseTangentLine,
            CKind::EllipseCurvature => K::EllipseCurvature,
        }
    }

    /// The kernel for this type when the number it states is not stated but *shared* — written
    /// in terms of a free variable, so the dimension's value is an unknown the solver moves
    /// rather than a constant.  See `expr::Free`.
    ///
    /// The match is exhaustive on purpose: a new type carrying a `Length` or an `Angle` stops
    /// the build here, and `every_dimension_can_be_written_free` checks the arm is the right
    /// one.  Everything that states no number says `None`, and can never be asked.
    pub fn free_kernel(self) -> Option<K> {
        Some(match self {
            CKind::Distance => K::DistanceFree,
            CKind::Angle => K::AngleFree,
            CKind::Radius => K::RadiusFree,
            CKind::ParallelDistance => K::ParallelDistanceFree,
            CKind::PointLineDistance => K::PointLineDistanceFree,
            CKind::AnnularDistance => K::AnnularDistanceFree,
            CKind::HorizontalDistance => K::HorizontalDistanceFree,
            CKind::VerticalDistance => K::VerticalDistanceFree,
            CKind::Coincident
            | CKind::Midpoint
            | CKind::DragTarget
            | CKind::Horizontal
            | CKind::Vertical
            | CKind::Parallel
            | CKind::Perpendicular
            | CKind::EqualLength
            | CKind::PointOnLine
            | CKind::PointOnCircle
            | CKind::EqualRadius
            | CKind::TangentLineCircle
            | CKind::TangentCircleCircle
            | CKind::TangentArcLine
            | CKind::TangentLineCircleAt
            | CKind::Symmetric
            | CKind::PointOnSpline
            | CKind::PointOnCurve
            | CKind::PointOnEllipse
            | CKind::EllipseTangentLine
            | CKind::EllipseCurvature
            | CKind::SplineTangentLine
            | CKind::SplineCurvature
            | CKind::HorizontalPoints
            | CKind::VerticalPoints => return None,
        })
    }
}

/// One constructor argument, in `spec` order.
#[derive(Clone, Debug, PartialEq)]
pub enum Arg {
    Ent(EntRef),
    Num(f64),
    Int(i64),
    Bool(bool),
    Str(String),
    /// A dimension written as text (`w = 1`, `h = w * 2`), carrying the number it evaluates to —
    /// see `expr`.  Only a `Length` or `Angle` slot holds one.
    Expr(crate::expr::Expr),
    /// An index into `Sketch::params`: the unknown this constraint owns, filled in by
    /// `Sketch::add`.  Only a `Param` slot holds one, and only after the constraint has been
    /// added — before that the slot carries an `Arg::Seed` (or a bare `Arg::Num`).
    Param(u32),
    /// What a `Param` slot holds on the way in: the number the unknown starts at, and whether
    /// the caller means it to *stay* there.  A pinned unknown is one somebody has already
    /// worked out — a fit knows where along the curve each of its points sits — so the solver
    /// is not to move it.
    ///
    /// Both halves travel together and are consumed together by `Sketch::add`, which is the one
    /// seam that turns a number into a Param.  That is the point of the variant: a path that
    /// carries the value cannot drop the pin, so a document, a paste, a rebuild and a
    /// constructor are all correct without knowing pins exist.
    Seed { value: f64, pinned: bool },
}

impl Arg {
    pub fn ent(&self) -> EntRef {
        match self {
            Arg::Ent(e) => *e,
            _ => panic!("argument is not an entity"),
        }
    }
    /// The Param index of an allocated unknown.
    pub fn param(&self) -> u32 {
        match self {
            Arg::Param(i) => *i,
            _ => panic!("argument is not an allocated parameter"),
        }
    }
    /// What this argument is worth as a number, resolving an owned unknown through the sketch
    /// that holds it — the one place that lookup lives, for the document writer, `graft`, the
    /// bindings' records and the contact accessors alike.
    pub fn value(&self, sk: &Sketch) -> f64 {
        match self {
            Arg::Param(i) => sk.params[*i as usize].value,
            a => a.num(),
        }
    }
    pub fn num(&self) -> f64 {
        match self {
            Arg::Num(v) => *v,
            Arg::Seed { value, .. } => *value,
            Arg::Int(v) => *v as f64,
            Arg::Bool(b) => {
                if *b {
                    1.0
                } else {
                    0.0
                }
            }
            Arg::Expr(e) => e.value,
            _ => panic!("argument is not a number"),
        }
    }
}

#[derive(Clone, Debug)]
pub struct Constraint {
    /// Document-stable identity, assigned by `Sketch::add` (0 until then).
    pub id: u32,
    pub kind: CKind,
    pub args: Vec<Arg>,
    /// Soft constraints (drag targets) do not count toward convergence.
    pub soft: bool,
    /// Implied by a primitive's definition (an arc's endpoints sit at its radius).
    pub intrinsic: bool,
    /// The unknown this constraint's number is written in terms of, when its dimension names a
    /// free variable — see `expr::Free`.  At most one, which is why it lives here and not on the
    /// argument: one appended column, one `(m, c)` pair, one twin kernel.  Derived state, written
    /// only by `expr::evaluate`, so `Constraint::new` and every rebuild leave it empty.
    pub free: Option<Free>,
}

impl Constraint {
    pub fn new(kind: CKind, args: Vec<Arg>) -> Constraint {
        debug_assert_eq!(args.len(), kind.spec().len(), "{:?} arity", kind);
        Constraint { id: 0, kind, args, soft: false, intrinsic: false, free: None }
    }

    pub fn coincident(p: EntRef, q: EntRef) -> Constraint {
        Constraint::new(CKind::Coincident, vec![Arg::Ent(p), Arg::Ent(q)])
    }

    pub fn distance(p: EntRef, q: EntRef, d: f64) -> Constraint {
        Constraint::new(CKind::Distance, vec![Arg::Ent(p), Arg::Ent(q), Arg::Num(d)])
    }

    pub fn one_line(kind: CKind, line: EntRef) -> Constraint {
        Constraint::new(kind, vec![Arg::Ent(line)])
    }

    pub fn two_line(kind: CKind, l1: EntRef, l2: EntRef) -> Constraint {
        Constraint::new(kind, vec![Arg::Ent(l1), Arg::Ent(l2)])
    }

    pub fn point_on_circle(p: EntRef, circle: EntRef, intrinsic: bool) -> Constraint {
        let mut c = Constraint::new(CKind::PointOnCircle, vec![Arg::Ent(p), Arg::Ent(circle)]);
        c.intrinsic = intrinsic;
        c
    }

    pub fn radius(circle: EntRef, r: f64) -> Constraint {
        Constraint::new(CKind::Radius, vec![Arg::Ent(circle), Arg::Num(r)])
    }

    pub fn drag_target(p: EntRef, tx: f64, ty: f64, weight: f64) -> Constraint {
        let mut c = Constraint::new(
            CKind::DragTarget,
            vec![Arg::Ent(p), Arg::Num(tx), Arg::Num(ty), Arg::Num(weight)],
        );
        c.soft = true;
        c
    }

    /// `TangentLineCircle` with the chirality flag read off the current geometry when `side` is
    /// `None`, so the solver keeps the circle on the side it already is.
    pub fn tangent_line_circle(
        sk: &Sketch,
        line: EntRef,
        circle: EntRef,
        side: Option<i64>,
    ) -> Constraint {
        let s = side.unwrap_or_else(|| {
            let l = &sk.lines[line.i()];
            let (ax, ay) = sk.point_xy(l.p1 as usize);
            let (bx, by) = sk.point_xy(l.p2 as usize);
            let (cx, cy) = sk.point_xy(sk.round_center(circle));
            let (dx, dy) = (bx - ax, by - ay);
            let (wx, wy) = (cx - ax, cy - ay);
            if dx * wy - dy * wx >= 0.0 {
                1
            } else {
                -1
            }
        });
        Constraint::new(
            CKind::TangentLineCircle,
            vec![Arg::Ent(line), Arg::Ent(circle), Arg::Int(s)],
        )
    }

    /// `TangentCircleCircle` with the sense read off the current geometry when `external` is
    /// `None`: whichever of |c1−c2| = r1+r2 (outside) and |c1−c2| = |r1−r2| (inside) the circles
    /// are already nearer to, so the solver keeps the arrangement the user drew.
    pub fn tangent_circle_circle(
        sk: &Sketch,
        c1: EntRef,
        c2: EntRef,
        external: Option<bool>,
    ) -> Constraint {
        let e = external.unwrap_or_else(|| {
            let (ax, ay) = sk.point_xy(sk.round_center(c1));
            let (bx, by) = sk.point_xy(sk.round_center(c2));
            let d = (ax - bx).hypot(ay - by);
            let (r1, r2) = (sk.radius_value(c1).abs(), sk.radius_value(c2).abs());
            (d - (r1 + r2)).abs() <= (d - (r1 - r2).abs()).abs()
        });
        Constraint::new(
            CKind::TangentCircleCircle,
            vec![Arg::Ent(c1), Arg::Ent(c2), Arg::Bool(e)],
        )
    }

    /// A point on a curve, starting at the curve parameter nearest where the point already is.
    pub fn point_on_spline(sk: &Sketch, p: EntRef, spline: EntRef) -> Constraint {
        Constraint::contact(sk, CKind::PointOnSpline, Arg::Ent(p), Arg::Ent(spline))
    }

    /// A line tangent to a curve, starting where the curve already comes nearest that line.
    pub fn spline_tangent_line(sk: &Sketch, spline: EntRef, line: EntRef) -> Constraint {
        Constraint::contact(sk, CKind::SplineTangentLine, Arg::Ent(spline), Arg::Ent(line))
    }

    /// A circle that osculates a curve — the curve's own radius there — starting at the place
    /// the circle's centre is already nearest.
    pub fn spline_curvature(sk: &Sketch, spline: EntRef, circle: EntRef) -> Constraint {
        Constraint::contact(sk, CKind::SplineCurvature, Arg::Ent(spline), Arg::Ent(circle))
    }

    /// A point on an ellipse's rim, starting at the rim parameter nearest where the point
    /// already is.
    pub fn point_on_ellipse(sk: &Sketch, p: EntRef, ellipse: EntRef) -> Constraint {
        Constraint::contact(sk, CKind::PointOnEllipse, Arg::Ent(p), Arg::Ent(ellipse))
    }

    /// A line tangent to an ellipse's rim, starting where the rim already comes nearest that
    /// line.
    pub fn ellipse_tangent_line(sk: &Sketch, ellipse: EntRef, line: EntRef) -> Constraint {
        Constraint::contact(sk, CKind::EllipseTangentLine, Arg::Ent(ellipse), Arg::Ent(line))
    }

    /// A circle that osculates an ellipse's rim — the rim's own radius there — starting at the
    /// place the circle's centre is already nearest.
    pub fn ellipse_curvature(sk: &Sketch, ellipse: EntRef, circle: EntRef) -> Constraint {
        Constraint::contact(sk, CKind::EllipseCurvature, Arg::Ent(ellipse), Arg::Ent(circle))
    }

    /// A two-entity curve contact whose parameter starts where the geometry puts it.
    fn contact(sk: &Sketch, kind: CKind, a: Arg, b: Arg) -> Constraint {
        let mut args = vec![a, b, Arg::Num(0.0)];
        args[2] = Arg::Num(seed_param(sk, kind, &args, 2));
        Constraint::new(kind, args)
    }

    pub fn kernel_id(&self) -> usize {
        self.kernel() as usize
    }

    /// Which kernel evaluates this constraint, when the sketch is at hand.
    ///
    /// The same as `kernel_id` for every type but one.  A curve contact's kernel belongs to the
    /// curve's *definition* — different families read different numbers of coordinates, so they
    /// cannot share a block — and the definition is only reachable through the sketch.  The ids
    /// run on past the static ones, which is what lets `System` hold a table of both.
    pub fn kernel_id_in(&self, sk: &Sketch) -> usize {
        match self.kind {
            CKind::PointOnCurve => {
                let e = self.args[1].ent();
                kernels::N_KERNELS + sk.curves[e.i()].def as usize
            }
            _ => self.kernel_id(),
        }
    }

    /// Which kernel evaluates this constraint: its type's, or the free-variable twin when the
    /// number it states is an unknown rather than a constant.
    fn kernel(&self) -> K {
        match self.free {
            Some(_) => self.kind.free_kernel().expect("a dimension has a free kernel"),
            None => self.kind.kernel(),
        }
    }

    pub fn n_residuals(&self) -> usize {
        match self.kind {
            // two, like every parametric contact: `p - C(u) = 0`.  Not asked of the kernel,
            // because a curve's belongs to its definition and this is a fact about the type.
            CKind::PointOnCurve => 2,
            _ => kernels::kernel(self.kernel()).n_res,
        }
    }

    pub fn spec(&self) -> Spec {
        self.kind.spec()
    }

    pub fn type_name(&self) -> &'static str {
        self.kind.name()
    }

    /// Entities this constraint references directly, in spec order.
    pub fn entities(&self) -> Vec<EntRef> {
        self.kind
            .spec()
            .iter()
            .zip(&self.args)
            .filter(|((_, k), _)| k.is_entity())
            .map(|(_, a)| a.ent())
            .collect()
    }

    /// The Params this constraint owns — empty until `Sketch::add` has allocated them.
    pub fn aux_params(&self) -> Vec<u32> {
        self.kind
            .param_slots()
            .iter()
            .filter_map(|&(i, _)| match self.args[i] {
                Arg::Param(p) => Some(p),
                _ => None,
            })
            .collect()
    }

    /// The (index, name, kind) of this constraint's dimension values.
    pub fn dimensions(&self) -> Vec<(usize, &'static str, SpecKind)> {
        self.kind
            .spec()
            .iter()
            .enumerate()
            .filter(|(_, (_, k))| k.is_dimension())
            .map(|(i, (n, k))| (i, *n, *k))
            .collect()
    }

    pub fn arg_index(&self, name: &str) -> Option<usize> {
        self.kind.spec().iter().position(|(n, _)| *n == name)
    }

    pub fn get_num(&self, name: &str) -> Option<f64> {
        self.arg_index(name).map(|i| self.args[i].num())
    }

    /// Set a numeric-ish argument by name — a dimension, a flag, a count.  `false` if there is no
    /// such argument or it is not one a number can express, rather than overwriting a string
    /// argument with `NaN`.  A dimension written as an expression becomes this plain number:
    /// whoever sets a number means the number, not the formula it replaces.
    ///
    /// This is the write on the constraint alone, and dropping an expression is a change to the
    /// *document*: the name it defined and the free variable it read are other constraints'
    /// business.  `Sketch::set_constraint_num` is the path that settles them, and is what a
    /// caller holding a sketch should use; this one is for a constraint that has no document
    /// behind it yet, or an argument no expression can reach (a soft drag target's own number).
    pub fn set_num(&mut self, name: &str, v: f64) -> bool {
        let Some(i) = self.arg_index(name) else { return false };
        self.args[i] = match self.args[i] {
            Arg::Int(_) => Arg::Int(v as i64),
            Arg::Bool(_) => Arg::Bool(v != 0.0),
            Arg::Num(_) | Arg::Expr(_) => Arg::Num(v),
            _ => return false,
        };
        // no expression left to read a name, so no unknown to be written in terms of: a binding
        // that outlived its text would have this constraint compiled against a column it no
        // longer has anything to say about
        self.free = None;
        true
    }

    /// The expression text behind a dimension argument, if it was written as one.
    pub fn expr_text(&self, name: &str) -> Option<&str> {
        match self.arg_index(name).map(|i| &self.args[i]) {
            Some(Arg::Expr(e)) => Some(&e.text),
            _ => None,
        }
    }

    /// Set a string argument by name (an arc tangency's end).  `false` if there is no such
    /// argument or it is not a string.
    pub fn set_str(&mut self, name: &str, v: &str) -> bool {
        let Some(i) = self.arg_index(name) else { return false };
        if !matches!(self.args[i], Arg::Str(_)) {
            return false;
        }
        self.args[i] = Arg::Str(v.to_string());
        true
    }

    /// Move a `DragTarget`'s target point.  No other kind has one, and several are shorter than
    /// three arguments, so the kind is checked rather than the write being attempted blind.
    pub fn set_target(&mut self, tx: f64, ty: f64) -> bool {
        if self.kind != CKind::DragTarget {
            return false;
        }
        self.args[1] = Arg::Num(tx);
        self.args[2] = Arg::Num(ty);
        true
    }

    /// The per-constraint constants the kernel needs (dimension values, chirality flags, and
    /// the local knot window of the span a curve contact sits on).
    pub fn consts(&self, sk: &Sketch) -> Vec<f64> {
        self.consts_on(sk, None)
    }

    /// The same, for a curve contact read on a *given* span — see `params_on`.
    pub fn consts_on(&self, sk: &Sketch, span: Option<usize>) -> Vec<f64> {
        // a dimension written in terms of a free variable states no number, so what its kernel
        // wants is the map onto the unknown instead: every free twin takes (m, c) and nothing
        // else, which is why this is one branch and not eight
        if let Some(f) = self.free {
            return vec![f.m, f.c];
        }
        if let Some((sp, t)) = self.spline_contact(sk) {
            let span = span.unwrap_or_else(|| crate::curve::span_of(sk, sp, t));
            return crate::curve::local_knots(&sk.splines[sp].knots, span).to_vec();
        }
        // a curve contact carries its family's two tapes and the numbers the instance was given.
        // They are the same for every contact with the same curve, and duplicated per constraint
        // because that is where a block already has room for numbers — which is what keeps the
        // kernel table `fn`-pointered and ignorant of curves.
        if self.kind == CKind::PointOnCurve {
            let cv = &sk.curves[self.args[1].ent().i()];
            let d = &sk.curve_defs[cv.def as usize];
            let mut k = Vec::with_capacity(3 + d.x.flat.len() + d.y.flat.len() + cv.values.len());
            k.push(d.vars.len() as f64);
            k.push(d.x.flat.len() as f64);
            k.push(d.y.flat.len() as f64);
            k.extend_from_slice(&d.x.flat);
            k.extend_from_slice(&d.y.flat);
            k.extend_from_slice(&cv.values);
            return k;
        }
        match self.kind {
            CKind::Distance | CKind::HorizontalDistance | CKind::VerticalDistance => {
                vec![self.args[2].num()]
            }
            CKind::DragTarget => {
                vec![self.args[1].num(), self.args[2].num(), self.args[3].num()]
            }
            CKind::Angle => {
                let t = self.args[2].num();
                vec![t.sin(), t.cos()]
            }
            CKind::ParallelDistance | CKind::PointLineDistance | CKind::AnnularDistance => {
                vec![self.args[2].num()]
            }
            CKind::Radius => vec![self.args[1].num()],
            CKind::TangentLineCircle => vec![self.args[2].num()],
            CKind::TangentCircleCircle => {
                vec![if matches!(self.args[2], Arg::Bool(true)) { 1.0 } else { -1.0 }]
            }
            _ => Vec::new(),
        }
    }

    /// The point a tangency-at-a-contact touches its round entity at: the arc endpoint or the
    /// line endpoint the `at` slot names.  One decode, because `params_on` picks the kernel's
    /// *columns* from it and `cgraph` picks the *cluster element* from it — two readings that
    /// have to name the same point or the plan and the kernel address different geometry.
    pub fn contact_point(&self, sk: &Sketch) -> Option<usize> {
        let at = |i: usize| match &self.args[2] {
            Arg::Str(s) => s.as_str() == ["start", "p1"][i],
            _ => true,
        };
        match self.kind {
            CKind::TangentArcLine => {
                let a = &sk.arcs[self.args[0].ent().i()];
                Some(if at(0) { a.start } else { a.end } as usize)
            }
            CKind::TangentLineCircleAt => {
                let l = &sk.lines[self.args[0].ent().i()];
                Some(if at(1) { l.p1 } else { l.p2 } as usize)
            }
            _ => None,
        }
    }

    /// The spline this constraint touches and the Param holding where on it — `None` for
    /// anything that is not a curve contact, and for one that has not been added yet (its
    /// parameter is still the seed number, not a Param).
    pub fn curve_contact(&self) -> Option<(usize, u32)> {
        let (curve, t) = self.kind.contact_slots()?;
        match self.args[t] {
            Arg::Param(p) => Some((self.args[curve].ent().i(), p)),
            _ => None,
        }
    }

    /// The parametric entity this constraint runs along, of *either* family, and the Param
    /// holding where along it.  `curve_contact` is the spline-only reading, and stays that way
    /// because it gates the span machinery; this is the reading for questions about the
    /// parameter itself — chiefly what one unit of it is worth in world length.
    pub fn parametric_contact(&self) -> Option<(EntRef, u32)> {
        let (e, t) =
            self.kind.contact_slots().or_else(|| self.kind.ellipse_contact_slots())?;
        match self.args[t] {
            Arg::Param(p) => Some((self.args[e].ent(), p)),
            _ => None,
        }
    }

    /// The spline a curve contact touches and the parameter it currently sits at — also before
    /// `Sketch::add`, while the slot still holds the seed number rather than a Param.
    fn spline_contact(&self, sk: &Sketch) -> Option<(usize, f64)> {
        let (curve, t) = self.kind.contact_slots()?;
        Some((self.args[curve].ent().i(), self.args[t].value(sk)))
    }

    /// The ordered Params the kernel's columns refer to.
    pub fn params(&self, sk: &Sketch) -> Vec<u32> {
        self.params_on(sk, None)
    }

    /// The same, for a curve contact read on a *given* span rather than the one its parameter is
    /// in now.  Which control points the columns name and which knots the constants are is one
    /// choice, not two: `System::new` makes it once and passes it here and to `consts_on`, so a
    /// compiled block cannot end up with one span's columns and another's knots.
    pub fn params_on(&self, sk: &Sketch, span: Option<usize>) -> Vec<u32> {
        let mut ps = self.own_params_on(sk, span);
        // the free column always comes last, so appending it is the whole of what a free twin
        // needs from here — see `expr::Free`
        if let Some(f) = self.free {
            ps.push(f.param);
        }
        ps
    }

    fn own_params_on(&self, sk: &Sketch, span: Option<usize>) -> Vec<u32> {
        let e = |i: usize| self.args[i].ent();
        let pt = |i: usize| sk.point_params(e(i).i()).to_vec();
        let ln = |i: usize| sk.line_params(e(i).i()).to_vec();
        let centre = |i: usize| sk.point_params(sk.round_center(e(i))).to_vec();
        let rad = |i: usize| sk.round_radius(e(i)) as u32;
        match self.kind {
            CKind::Coincident
            | CKind::Distance
            | CKind::HorizontalPoints
            | CKind::VerticalPoints
            | CKind::HorizontalDistance
            | CKind::VerticalDistance => [pt(0), pt(1)].concat(),
            CKind::Midpoint | CKind::PointOnLine | CKind::PointLineDistance => {
                [pt(0), ln(1)].concat()
            }
            CKind::DragTarget => pt(0),
            CKind::Horizontal | CKind::Vertical => ln(0),
            CKind::Parallel
            | CKind::Perpendicular
            | CKind::Angle
            | CKind::ParallelDistance
            | CKind::EqualLength => [ln(0), ln(1)].concat(),
            CKind::PointOnCircle => [pt(0), centre(1), vec![rad(1)]].concat(),
            CKind::Radius => vec![rad(0)],
            CKind::EqualRadius | CKind::AnnularDistance => vec![rad(0), rad(1)],
            CKind::TangentLineCircle => [ln(0), centre(1), vec![rad(1)]].concat(),
            CKind::TangentCircleCircle => {
                [centre(0), vec![rad(0)], centre(1), vec![rad(1)]].concat()
            }
            // both say "the radius is perpendicular to the line at the contact", so both are
            // [contact point, centre, line] — the arc names the line second, the circle first
            CKind::TangentArcLine | CKind::TangentLineCircleAt => {
                let at = self.contact_point(sk).unwrap();
                let line = if self.kind == CKind::TangentArcLine { 1 } else { 0 };
                [sk.point_params(at).to_vec(), centre(if line == 1 { 0 } else { 1 }), ln(line)]
                    .concat()
            }
            CKind::Symmetric => [pt(0), pt(1), ln(2)].concat(),
            // the curve columns are one span's control points, which is what keeps the column
            // count fixed however long the spline is
            CKind::PointOnSpline => {
                [pt(0), vec![self.args[2].param()], self.span_params(sk, span)].concat()
            }
            // the point, the parameter it sits at, and every coordinate the curve reads — in
            // `entity_params` order, which is the order the definition's tapes were compiled
            // against, so the gradient that comes back needs no rearranging
            CKind::PointOnCurve => {
                [pt(0), vec![self.args[2].param()], sk.entity_params(e(1))].concat()
            }
            CKind::SplineTangentLine => {
                [vec![self.args[2].param()], self.span_params(sk, span), ln(1)].concat()
            }
            CKind::SplineCurvature => [
                vec![self.args[2].param()],
                self.span_params(sk, span),
                centre(1),
                vec![rad(1)],
            ]
            .concat(),
            // the ellipse contacts: where along the rim, then the five numbers the ellipse is
            // drawn from, then whatever it touches
            CKind::PointOnEllipse => {
                [pt(0), vec![self.args[2].param()], self.ellipse_params(sk, 1)].concat()
            }
            CKind::EllipseTangentLine => {
                [vec![self.args[2].param()], self.ellipse_params(sk, 0), ln(1)].concat()
            }
            CKind::EllipseCurvature => [
                vec![self.args[2].param()],
                self.ellipse_params(sk, 0),
                centre(1),
                vec![rad(1)],
            ]
            .concat(),
        }
    }

    /// The (cx, cy, mx, my, b) columns of the ellipse in spec slot `i` — every ellipse kernel
    /// reads them in this order, after the contact's own parameter.  That order is the model's
    /// canonical one, so it is asked for rather than rebuilt: the kernels and `entity_params`
    /// would otherwise be two statements of the same column layout.
    fn ellipse_params(&self, sk: &Sketch, i: usize) -> Vec<u32> {
        sk.entity_params(self.args[i].ent())
    }

    /// The Params of the span given, or of the one this contact currently sits on.
    fn span_params(&self, sk: &Sketch, span: Option<usize>) -> Vec<u32> {
        let (sp, t) = self.spline_contact(sk).expect("not a curve contact");
        sk.spline_span_params(sp, span.unwrap_or_else(|| crate::curve::span_of(sk, sp, t)))
    }

    pub fn local_values(&self, sk: &Sketch) -> Vec<f64> {
        self.params(sk).iter().map(|&i| sk.params[i as usize].value).collect()
    }

    /// Current residual norm — convenience for reporting and tests.
    pub fn error(&self, sk: &Sketch) -> f64 {
        let v = self.local_values(sk);
        let (r, _) = kernels::eval_one(self.kernel_id(), &v, &self.consts(sk));
        crate::linalg::norm(&r)
    }

    pub fn residual(&self, sk: &Sketch, v: &[f64]) -> Vec<f64> {
        kernels::eval_one(self.kernel_id(), v, &self.consts(sk)).0
    }

    /// n_res x n_par, row-major.
    pub fn jacobian(&self, sk: &Sketch, v: &[f64]) -> Vec<f64> {
        kernels::eval_one(self.kernel_id(), v, &self.consts(sk)).1
    }
}

/// Where a hidden unknown starts when the caller leaves it out — the `Param` counterpart of
/// `infers_arg`: the core reads it off the geometry, so no binding ever has to name a curve
/// parameter.  A document that saved one passes the saved number instead and never comes here.
pub fn seed_param(sk: &Sketch, kind: CKind, args: &[Arg], i: usize) -> f64 {
    match (kind, i) {
        // where the curve already comes nearest the thing it is being tied to: a curve can meet
        // a point or a line in several places, and the nearest is the branch the user drew
        (CKind::PointOnSpline, 2) => {
            let (x, y) = sk.point_xy(args[0].ent().i());
            crate::curve::closest(sk, args[1].ent().i(), x, y).0
        }
        (CKind::SplineTangentLine, 2) => {
            let [ax, ay, bx, by] = sk.line_params(args[1].ent().i());
            let g = |p: u32| sk.params[p as usize].value;
            crate::curve::nearest_to_line(sk, args[0].ent().i(), g(ax), g(ay), g(bx), g(by))
        }
        // an osculating circle sits centred a radius off the curve, so the curve point nearest
        // the centre it already has is the place it is asking about
        (CKind::SplineCurvature, 2) => {
            let (cx, cy) = sk.point_xy(sk.round_center(args[1].ent()));
            crate::curve::closest(sk, args[0].ent().i(), cx, cy).0
        }
        (CKind::PointOnEllipse, 2) => {
            let (x, y) = sk.point_xy(args[0].ent().i());
            crate::ellipse::closest(sk, args[1].ent().i(), x, y).0
        }
        (CKind::EllipseTangentLine, 2) => {
            let [ax, ay, bx, by] = sk.line_params(args[1].ent().i());
            let g = |p: u32| sk.params[p as usize].value;
            crate::ellipse::nearest_to_line(sk, args[0].ent().i(), g(ax), g(ay), g(bx), g(by))
        }
        // an osculating circle sits centred off the rim, so the rim point nearest the centre it
        // already has is the place it is asking about — the same reading as `SplineCurvature`
        (CKind::EllipseCurvature, 2) => {
            let (cx, cy) = sk.point_xy(sk.round_center(args[1].ent()));
            crate::ellipse::closest(sk, args[0].ent().i(), cx, cy).0
        }
        _ => 0.0,
    }
}

/// The world length one unit of a hidden unknown is worth — see `Param::scale`.  Read off the
/// geometry at the moment the constraint is added; it preconditions the step, so an estimate
/// that drifts as the sketch moves costs convergence rate, never correctness.
pub fn param_scale(sk: &Sketch, kind: CKind, args: &[Arg], i: usize) -> f64 {
    // Whichever family the contact runs along, the question is the same one, so it is asked once
    // and answered by the entity the slot actually names.  A hidden unknown that runs along
    // nothing is a length already.
    let slots = kind.contact_slots().or_else(|| kind.ellipse_contact_slots());
    match slots {
        Some((e, t)) if t == i => contact_speed(sk, args[e].ent()),
        _ => 1.0,
    }
}

/// The world length one unit of a contact's parameter is worth, whichever family it runs along
/// — the one answer, so the seed `Sketch::add` records and the scale `System::new` compiles
/// against cannot come from two different rules.
pub fn contact_speed(sk: &Sketch, e: EntRef) -> f64 {
    match e.kind {
        EntKind::Ellipse => crate::ellipse::speed(sk, e.i()),
        _ => crate::curve::speed(sk, e.i()),
    }
}

fn same_args(a: &Constraint, b: &Constraint, swap: bool, want: impl Fn(SpecKind) -> bool)
    -> bool
{
    let spec = a.kind.spec();
    let mut order: Vec<usize> = (0..spec.len()).collect();
    if swap {
        let ents: Vec<usize> =
            spec.iter().enumerate().filter(|(_, (_, k))| k.is_entity()).map(|(i, _)| i).collect();
        if ents.len() < 2 {
            return false;
        }
        order.swap(ents[0], ents[1]);
    }
    // A hidden unknown is never part of what a constraint *says*: two contacts of the same point
    // on the same curve are the same statement however far apart their two seeds started, and a
    // duplicate that slipped through would add rank-free rows the matching cannot see.
    (0..spec.len()).filter(|&i| want(spec[i].1)).all(|i| a.args[i] == b.args[order[i]])
}

/// True when two constraints say exactly the same thing: same type, the same entities in the same
/// roles, the same values.  `commutative` types also match with their first two entities swapped.
///
/// An exact duplicate is worth keeping out of a sketch: it adds equations without adding rank, and
/// a structural matching cannot see that — two identical rows still match two different variables
/// — so it stays invisible until some unrelated edit tips the block into a (spurious)
/// over-constrained report.
pub fn same_constraint(a: &Constraint, b: &Constraint) -> bool {
    matches(a, b, |k| !k.is_param())
}

/// True when two constraints relate the same things in the same way, *whatever numbers they
/// state*: same type, same entities in the same roles, same flags, dimensions ignored.
///
/// `same_constraint` is this plus the values, which is what a duplicate is.  This is what an
/// *edit* would land on: the constraint a second `Distance` on the same pair would be rewriting
/// rather than adding to.  Whether that is what a caller wants is the caller's business — the
/// app states the second one and lets the diagnosis judge the pair.
pub fn same_relation(a: &Constraint, b: &Constraint) -> bool {
    matches(a, b, |k| !k.is_param() && !k.is_dimension())
}

fn matches(a: &Constraint, b: &Constraint, want: impl Fn(SpecKind) -> bool + Copy) -> bool {
    if a.kind != b.kind {
        return false;
    }
    same_args(a, b, false, want) || (a.kind.commutative() && same_args(a, b, true, want))
}

/// Whether an entity can fill a spec slot of this kind.
pub fn kind_matches(spec: SpecKind, ent: EntKind) -> bool {
    match spec {
        SpecKind::Point => ent == EntKind::Point,
        SpecKind::Line => ent == EntKind::Line,
        SpecKind::Circle => ent == EntKind::Circle,
        SpecKind::Arc => ent == EntKind::Arc,
        SpecKind::CircleOrArc => ent == EntKind::Circle || ent == EntKind::Arc,
        SpecKind::Spline => ent == EntKind::Spline,
        SpecKind::Ellipse => ent == EntKind::Ellipse,
        SpecKind::Curve => ent == EntKind::Curve,
        _ => false,
    }
}
