//! Constraint types: entities → a local parameter tuple, constants and a kernel.
//!
//! A constraint is `(kind, args)` where `args` are the constructor arguments in `spec` order.
//! `spec` drives everything reflective — JSON I/O, the constraint list, value editing, the
//! toolbar applier, duplicate detection and the witness's dimension jitter — so a new type is
//! covered everywhere as soon as it declares one.
//!
//! Residual forms follow the program: distance uses |p−q|² − d² (no sqrt), parallel is a 2×2
//! determinant, angle a wrapped atan2 gap (directed, so it needs no chirality), tangency a
//! signed distance minus the radius with a chirality flag fixed at construction.

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
    /// The *directed* angle: the full-turn angle from l1's direction (p1→p2) to l2's, positive
    /// counter-clockwise — exactly the value `model::angle_between` reads and the dimension
    /// dialog offers.  Not a statement mod half a turn: stated that way, every use that meant a
    /// bearing had to drag an orientation predicate behind it to pick the side, where here the
    /// winding is algebraic in the residual itself — the strongest of the three branch
    /// instruments (spec §6.5.1) — and a trace block posing a crank by its angle needs no `ccw`.
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
    /// A line tangent to a curve written in the language — `SplineTangentLine`'s shape over
    /// the curve's own frame: two residuals against one owned parameter, the net one equation a
    /// tangency is worth.  Its kernel is the definition's, beside the contact's.
    CurveTangentLine,
    /// A circle osculating a curve written in the language — `SplineCurvature`'s shape: three
    /// residuals against one owned parameter, the net two an osculating circle costs.  Needs the
    /// curve's second and third derivatives, which a formula has and a trace does not: stated
    /// against a traced curve it is refused (`validate`).
    CurveCurvature,
    /// A frame's rotor held to the unit circle: `c² + s² = 1`.  Intrinsic — `Sketch::frame`
    /// states it and nothing else does — like an arc's endpoints sitting at its radius.
    FrameUnit,
    /// A frame's rotor kept on its chord: `(toward − origin) = r·(c, s)`, with the chord's
    /// length `r` the constraint's own unknown — two residuals, one Param, net one equation,
    /// and directed (with the rotor on the unit circle, `r` stays positive by continuity).
    /// Intrinsic, the other half of what `Sketch::frame` states.
    FrameAlign,
    /// Two points are images of one point in space, each on the plane it is `in`: their
    /// coordinates along the fold line the two planes share agree (`plane::fold_line`).  One
    /// row over both points and both planes' frames.  The plane slots are **inferred** from the
    /// points' memberships at `io::seed_omitted`'s seam — the source and the bindings write two
    /// points — and refused when a point is on no plane, both are on one, or the planes are
    /// parallel.  Not commutative: `same_args` swaps only the first two entity slots, so
    /// `b project a` reads as a second relation, which the diagnosis reports as implied.
    Project,
    /// The **gauges** and the **orientation predicates** (spec §9.2, §9.6; issue #47, item 5):
    /// statements written as every other constraint is — an operator, its operands, a class, a
    /// placement — and settled through the same table, but **applied by the elaborator rather
    /// than held by the model**: `ground` and `fix` mark parameters fixed, `ccw` and `cw` record
    /// a root choice in `Sketch::branches`.  They own no kernel, add no row, and are in no
    /// `Constraint` the sketch holds — so they are **not in `ALL_KINDS`**, the registry never
    /// publishes them, and `CKind::gauge` is how every table that would otherwise reach for a
    /// kernel tells them apart.  A `claim` on one is refused: a claim is judged by rank over
    /// rows, and these have none.
    Ground,
    Fix,
    Ccw,
    Cw,
}

/// Which of a curve definition's kernels a kind runs through: the table holds these three per
/// definition, in this order (`System::kernel_table`), and `Constraint::kernel_id_in` counts
/// them so — the one statement of what "a kind whose kernel is per definition" means.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FamilyKernel {
    Contact,
    Tangent,
    Curvature,
}

impl FamilyKernel {
    pub const ALL: [FamilyKernel; 3] =
        [FamilyKernel::Contact, FamilyKernel::Tangent, FamilyKernel::Curvature];

    /// Rows per constraint — a fact about the kind, not asked of a kernel it may not have yet.
    pub fn n_res(self) -> usize {
        match self {
            FamilyKernel::Contact | FamilyKernel::Tangent => 2,
            FamilyKernel::Curvature => 3,
        }
    }
}

/// Every concrete constraint type, in the order the registry lists them.
pub const ALL_KINDS: [CKind; 38] = [
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
    CKind::CurveTangentLine,
    CKind::CurveCurvature,
    CKind::FrameUnit,
    CKind::FrameAlign,
    CKind::Project,
];

/// What a written operator says, once its operands' kinds are known.
///
/// **The one table that turns a word and a pair of kinds into a constraint.**  It is the inverse
/// of `CKind::operator`, and it is a table rather than a search because several kinds share a
/// word and the operand kinds (and one selector) are what tell them apart: `on` is five kinds,
/// `distance` is six, `tangent` is six.
///
/// Operand order carries meaning, and that is a change worth seeing: `arc tangent line` is
/// `TangentArcLine` and `line tangent circle` is `TangentLineCircle`.  Each named itself before
/// and the order was decoration; as an operator, which side the arc is written on picks the kind.
///
/// `sel` is what stood in the parentheses, by name — `along`, `at` — since two of the choices
/// cannot be made from the kinds alone.  `None` is "this word does not relate those two", which
/// the caller reports with the kinds in it.
pub fn infix_op(word: &str, a: EntKind, b: EntKind, sel: &dyn Fn(&str) -> Option<String>) -> Option<CKind> {
    use EntKind::{Arc, Circle, Curve, Ellipse, Line, Point, Spline};
    let round = |k: EntKind| matches!(k, Circle | Arc);
    Some(match word {
        "on" => match (a, b) {
            (Point, Line) => CKind::PointOnLine,
            (Point, k) if round(k) => CKind::PointOnCircle,
            (Point, Spline) => CKind::PointOnSpline,
            (Point, Ellipse) => CKind::PointOnEllipse,
            (Point, Curve) => CKind::PointOnCurve,
            _ => return None,
        },
        "distance" => match (a, b) {
            // which of the three a pair of points means is `along:`, and the run and the rise
            // are signed from the first point to the second — so they do not commute
            (Point, Point) => match sel("along").as_deref() {
                None => CKind::Distance,
                Some("x") => CKind::HorizontalDistance,
                Some("y") => CKind::VerticalDistance,
                Some(_) => return None,
            },
            (Point, Line) => CKind::PointLineDistance,
            (Line, Line) => CKind::ParallelDistance,
            (x, y) if round(x) && round(y) => CKind::AnnularDistance,
            _ => return None,
        },
        "tangent" => match (a, b) {
            // tangency at a named end of the line is the regular form; the bare pair is
            // rank-deficient at every solution, so `at:` is how a drawing says which
            (Line, k) if round(k) => match sel("at") {
                Some(_) => CKind::TangentLineCircleAt,
                None => CKind::TangentLineCircle,
            },
            (Arc, Line) => CKind::TangentArcLine,
            // two round things meeting at a corner already touch there, so a threaded joint's
            // `at:` has no regular form to pick — refused, never a silently degenerate row
            (x, y) if round(x) && round(y) => match sel("at") {
                None => CKind::TangentCircleCircle,
                Some(_) => return None,
            },
            (Spline, Line) => CKind::SplineTangentLine,
            (Ellipse, Line) => CKind::EllipseTangentLine,
            (Curve, Line) => CKind::CurveTangentLine,
            _ => return None,
        },
        "equal" => match (a, b) {
            (Line, Line) => CKind::EqualLength,
            (x, y) if round(x) && round(y) => CKind::EqualRadius,
            _ => return None,
        },
        "curvature" => match (a, b) {
            (Spline, k) if round(k) => CKind::SplineCurvature,
            (Ellipse, k) if round(k) => CKind::EllipseCurvature,
            (Curve, k) if round(k) => CKind::CurveCurvature,
            _ => return None,
        },
        "horizontal" => match (a, b) {
            (Point, Point) => CKind::HorizontalPoints,
            _ => return None,
        },
        "vertical" => match (a, b) {
            (Point, Point) => CKind::VerticalPoints,
            _ => return None,
        },
        "angle" => match (a, b) {
            (Line, Line) => CKind::Angle,
            _ => return None,
        },
        "coincident" => match (a, b) {
            (Point, Point) => CKind::Coincident,
            _ => return None,
        },
        "midpoint" => match (a, b) {
            (Point, Line) => CKind::Midpoint,
            _ => return None,
        },
        // two images of one point; which planes is read off the points, never written
        "project" => match (a, b) {
            (Point, Point) => CKind::Project,
            _ => return None,
        },
        "parallel" => match (a, b) {
            (Line, Line) => CKind::Parallel,
            _ => return None,
        },
        "perpendicular" => match (a, b) {
            (Line, Line) => CKind::Perpendicular,
            _ => return None,
        },
        "symmetry" => match (a, b) {
            (Point, Point) => CKind::Symmetric,
            _ => return None,
        },
        _ => return None,
    })
}

/// The same for a word standing *before* its one operand.  `distance` on a line is sugar for the
/// distance between its ends, which is why it is here and not in the table above.
pub fn prefix_op(word: &str, on: EntKind) -> Option<CKind> {
    use EntKind::{Arc, Circle, Line};
    Some(match (word, on) {
        ("horizontal", Line) => CKind::Horizontal,
        ("vertical", Line) => CKind::Vertical,
        ("radius", Circle | Arc) => CKind::Radius,
        ("distance", Line) => CKind::Distance,
        _ => return None,
    })
}

/// Every word the language writes a **constraint** with (spec §9.1) — the gauges and the
/// orientation predicates among them, read by the one relation parser and settled by the one
/// table (`gauge_op`), so a class, a placement and the chain's lookahead treat them as any
/// other word.  None of the four is a prefix word a chain can open a link with: `prefix_op`
/// declines them, so `ground point p -> …` stays what it always was, no chain.
pub const OPERATORS: [&str; 19] = [
    "on", "distance", "tangent", "equal", "curvature", "horizontal", "vertical", "angle",
    "radius", "coincident", "midpoint", "parallel", "perpendicular", "symmetry", "project",
    "ground", "fix", "ccw", "cw",
];

pub fn is_operator(w: &str) -> bool {
    OPERATORS.contains(&w)
}

/// The gauges and the orientation predicates, by word: settled **before** the operands' kinds
/// are asked, since `fix c.r` names a number and not an entity and `ccw(a, b, c)` has no
/// operand outside its parentheses.  What each operand must be is checked where the statement
/// is applied (`program::apply_gauge`), in the words the gauges always used.
pub fn gauge_op(word: &str) -> Option<CKind> {
    Some(match word {
        "ground" => CKind::Ground,
        "fix" => CKind::Fix,
        "ccw" => CKind::Ccw,
        "cw" => CKind::Cw,
        _ => return None,
    })
}

/// Whether a word is written as a call — every operand inside the parentheses.
pub fn call_word(w: &str) -> bool {
    matches!(gauge_op(w).map(CKind::operator), Some(Some((_, Fixity::Call))))
}

/// Where an operator stands to its operand(s) — see `CKind::operator`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Fixity {
    /// `radius(25) circle1`, `horizontal line1`, `ground p1`
    Prefix,
    /// `p1 distance(80) p2`, `line1 tangent circle1`
    Infix,
    /// `ccw(a, b, c)` — every operand in the parentheses, since the three are symmetric and an
    /// order written around the word would say something the predicate does not
    Call,
}

impl Fixity {
    pub fn as_str(self) -> &'static str {
        match self {
            Fixity::Prefix => "prefix",
            Fixity::Infix => "infix",
            Fixity::Call => "call",
        }
    }
}

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
    /// The datum: a plane, whose rotor the two intrinsics read and whose basis `Project` does.
    Plane,
    /// One of an entity's own numbers, named by its field — `c.r`, `p.x` — the operand of
    /// `fix`.  Filled from a reference like an entity slot and resolved by the gauge's own
    /// rule, never by `follow`, since a field is not a child.
    Scalar,
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
    /// What a slot's number *is* (`units.rs`).  `SpecKind::Length` and `Angle` already **are**
    /// the dimensions, so the check an expression faces is `Dim(expr).fits(slot.dim())` and
    /// nothing here has to be written per constraint type.
    ///
    /// Exhaustive on purpose, like `own_params` and `free_kernel`: a new slot kind that carries a
    /// number must stop the build here, or it would quietly be dimensionless and accept anything.
    /// `Param` is the one that is *stated* Scalar rather than being one — a slot's hidden unknown
    /// is a curve parameter here, but `FrameAlign`'s is a chord length, and nothing yet asks.
    pub fn dim(self) -> crate::units::Dim {
        use crate::units::Dim;
        match self {
            SpecKind::Length => Dim::LENGTH,
            SpecKind::Angle => Dim::ANGLE,
            SpecKind::Point
            | SpecKind::Line
            | SpecKind::Circle
            | SpecKind::Arc
            | SpecKind::CircleOrArc
            | SpecKind::Spline
            | SpecKind::Ellipse
            | SpecKind::Curve
            | SpecKind::Plane
            | SpecKind::Scalar
            | SpecKind::Float
            | SpecKind::Int
            | SpecKind::Str
            | SpecKind::Bool
            | SpecKind::Param => Dim::SCALAR,
        }
    }

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
                | SpecKind::Plane
        )
    }

    /// A slot a *reference* fills: an entity, or one of an entity's own numbers.
    pub fn takes_ref(self) -> bool {
        self.is_entity() || self == SpecKind::Scalar
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
            SpecKind::Plane => "plane",
            SpecKind::Scalar => "scalar",
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
            CKind::CurveTangentLine => "CurveTangentLine",
            CKind::CurveCurvature => "CurveCurvature",
            CKind::FrameUnit => "FrameUnit",
            CKind::FrameAlign => "FrameAlign",
            CKind::Project => "Project",
            CKind::Ground => "Ground",
            CKind::Fix => "Fix",
            CKind::Ccw => "Ccw",
            CKind::Cw => "Cw",
        }
    }

    /// A statement the elaborator applies rather than a constraint the model holds — see the
    /// variants' note.  Asked wherever a table would otherwise reach for a kernel.
    pub fn gauge(self) -> bool {
        matches!(self, CKind::Ground | CKind::Fix | CKind::Ccw | CKind::Cw)
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
            CKind::PointOnCurve => &[("p", S::Point), ("curve", S::Curve), ("t", S::Param)],
            CKind::CurveTangentLine => &[("curve", S::Curve), ("line", S::Line), ("t", S::Param)],
            CKind::CurveCurvature => {
                &[("curve", S::Curve), ("circle", S::CircleOrArc), ("t", S::Param)]
            }
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
            CKind::FrameUnit => &[("frame", S::Plane)],
            CKind::FrameAlign => &[("frame", S::Plane), ("r", S::Param)],
            // the two planes are real slots — so the drag part, the topology key, the graft
            // and a deletion follow them — and inferred ones, so nobody writes them
            CKind::Project => {
                &[("a", S::Point), ("b", S::Point), ("pa", S::Plane), ("pb", S::Plane)]
            }
            CKind::Ground => &[("p", S::Point)],
            // one of an entity's own numbers, named by its field: `fix c.r`
            CKind::Fix => &[("x", S::Scalar)],
            // the predicate is about the triangle, so all three stand in the parentheses
            CKind::Ccw | CKind::Cw => &[("a", S::Point), ("b", S::Point), ("c", S::Point)],
        }
    }

    /// What this type's `SpecKind::Param` slot *is* (`units.rs`) — `None` where it owns none.
    ///
    /// `SpecKind::dim()` cannot answer this, which is why it is asked here: a hidden unknown is
    /// usually a **place along a curve** and dimensionless, but `FrameAlign`'s is the frame
    /// chord's **length**, and a paste between documents in different units has to convert one
    /// and must not touch the other.  `every_param_slot_states_its_dimension` holds every type
    /// that owns a Param to naming it here, so a new one cannot arrive unstated.
    pub fn param_dim(self) -> Option<crate::units::Dim> {
        use crate::units::Dim;
        match self {
            CKind::FrameAlign => Some(Dim::LENGTH),
            // a place along a curve or an ellipse: a parameter, not a length
            CKind::PointOnSpline
            | CKind::PointOnCurve
            | CKind::CurveTangentLine
            | CKind::CurveCurvature
            | CKind::SplineTangentLine
            | CKind::SplineCurvature
            | CKind::PointOnEllipse
            | CKind::EllipseTangentLine
            | CKind::EllipseCurvature => Some(Dim::SCALAR),
            _ => None,
        }
    }

    /// How a constraint is **written** (Solvent §9.1): the word, and where it stands.
    ///
    /// Every user-facing constraint has one or two entity slots, always first in spec order —
    /// 1 for `Horizontal`, `Vertical` and `Radius`, 2 for twenty-eight others, and 3 for
    /// `Symmetric` alone.  So "two operands, everything else in the parentheses" is not a rule
    /// imposed on the library; it is a description of it, with one exception the parentheses
    /// absorb.  `None` is a constraint nobody writes: `DragTarget` is internal and `soft`,
    /// `FrameUnit`/`FrameAlign` are intrinsic and minted by `Sketch::frame`.
    ///
    /// Several kinds share a word, and that is where the saving is: **`on` is five kinds,
    /// `distance` is six, `tangent` is six**, and `horizontal`/`vertical` are two each with the
    /// *fixity* doing the work — a line prefixed, a pair of points infixed, which is exactly the
    /// distinction `HorizontalPoints` was added to draw.
    ///
    /// The **surface word and the wire name are different things**: `report::registry_json` goes
    /// on publishing the snake_case `name` that both the binding and the JSON export key on, and
    /// this is new information beside it.  Matched exhaustively, so a new kind stops the build —
    /// the pattern `callout::pen` and `free_kernel` already use.
    pub fn operator(self) -> Option<(&'static str, Fixity)> {
        use Fixity::{Call, Infix, Prefix};
        Some(match self {
            // a point on something: five kinds, one word, told apart by the right operand
            CKind::PointOnLine
            | CKind::PointOnCircle
            | CKind::PointOnSpline
            | CKind::PointOnEllipse
            | CKind::PointOnCurve => ("on", Infix),
            CKind::CurveTangentLine => ("tangent", Infix),
            CKind::CurveCurvature => ("curvature", Infix),
            // a measured separation: six kinds, told apart by the pair and by `along:`
            CKind::Distance
            | CKind::HorizontalDistance
            | CKind::VerticalDistance
            | CKind::PointLineDistance
            | CKind::ParallelDistance
            | CKind::AnnularDistance => ("distance", Infix),
            // touching: six kinds, told apart by the pair and by `at:`
            CKind::TangentLineCircle
            | CKind::TangentLineCircleAt
            | CKind::TangentCircleCircle
            | CKind::TangentArcLine
            | CKind::SplineTangentLine
            | CKind::EllipseTangentLine => ("tangent", Infix),
            CKind::EqualLength | CKind::EqualRadius => ("equal", Infix),
            CKind::SplineCurvature | CKind::EllipseCurvature => ("curvature", Infix),
            // the fixity is the distinction: a line prefixed, a pair of points infixed
            CKind::Horizontal => ("horizontal", Prefix),
            CKind::HorizontalPoints => ("horizontal", Infix),
            CKind::Vertical => ("vertical", Prefix),
            CKind::VerticalPoints => ("vertical", Infix),
            // `angle` and `radius` keep their own words rather than folding into `distance`:
            // over two lines a Length means a parallel distance and an Angle means an angle, and
            // nothing but the number's unit could separate them
            CKind::Angle => ("angle", Infix),
            CKind::Radius => ("radius", Prefix),
            CKind::Coincident => ("coincident", Infix),
            CKind::Midpoint => ("midpoint", Infix),
            CKind::Parallel => ("parallel", Infix),
            CKind::Perpendicular => ("perpendicular", Infix),
            // the only kind with three entity slots, and the parentheses absorb the third
            CKind::Symmetric => ("symmetry", Infix),
            // two operands; the plane slots behind them are inferred and never spelled
            CKind::Project => ("project", Infix),
            // the gauges are prefix words like `horizontal`; the orientation predicates keep
            // a call, since `a ccw(c) b` would reorder three points that are symmetric
            CKind::Ground => ("ground", Prefix),
            CKind::Fix => ("fix", Prefix),
            CKind::Ccw => ("ccw", Call),
            CKind::Cw => ("cw", Call),
            CKind::DragTarget | CKind::FrameUnit | CKind::FrameAlign => return None,
        })
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
        // a hidden unknown is always read off the geometry: nobody types a curve parameter;
        // and a projection's planes are read off its points' memberships
        self.spec()[i].1.is_param()
            || matches!(
                (self, i),
                (CKind::TangentLineCircle, 2)
                    | (CKind::TangentCircleCircle, 2)
                    | (CKind::Project, 2 | 3)
            )
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

    /// Whether this kind owns an unknown of its own — a `Param` slot the solver moves, such as a
    /// curve contact's parameter along the curve.  A `claim` (§9.7) compiles to no rows, so such
    /// an unknown would sit in no equation at all: a degree of freedom the drawing does not have,
    /// minted by a statement that promised to add nothing.  Elaboration turns the refusal into an
    /// E040 with a span; the document readers, which take untrusted input, drop the flag instead.
    /// Whether the number this kind states is a *magnitude* — a point-to-point distance, a
    /// radius — as against a signed one (a run, a rise, a point's offset from a line).  A
    /// magnitude's residual squares its sign away or draws its absolute value, so a negative
    /// literal in the source would quietly mean the positive and the drawing and the document
    /// would disagree about what the circle is (#43.12); it is refused where it is written.
    pub fn magnitude(self) -> bool {
        matches!(self, CKind::Distance | CKind::Radius)
    }

    pub fn claimable(self) -> bool {
        !self.gauge() && !self.spec().iter().any(|(_, k)| k.is_param())
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

    /// The per-definition kernel this kind runs through, for the three kinds that have one.
    pub fn family_kernel(self) -> Option<FamilyKernel> {
        Some(match self {
            CKind::PointOnCurve => FamilyKernel::Contact,
            CKind::CurveTangentLine => FamilyKernel::Tangent,
            CKind::CurveCurvature => FamilyKernel::Curvature,
            _ => return None,
        })
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
            | CKind::EllipseCurvature
            | CKind::CurveTangentLine
            | CKind::CurveCurvature => true,
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
            | CKind::VerticalDistance
            // a frame's intrinsics are algebra over its own scalars, not a touch between two
            // figures: there is no contact to double-root
            | CKind::FrameUnit
            | CKind::FrameAlign
            // a projection is a linear tie between two images: no contact, no double root
            | CKind::Project
            | CKind::Ground
            | CKind::Fix
            | CKind::Ccw
            | CKind::Cw => false,
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
            CKind::PointOnCurve | CKind::CurveTangentLine | CKind::CurveCurvature => {
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
            CKind::FrameUnit => K::FrameUnit,
            CKind::FrameAlign => K::FrameAlign,
            CKind::Project => K::Project,
            CKind::Ground | CKind::Fix | CKind::Ccw | CKind::Cw => {
                panic!("{:?} is a gauge: applied by the elaborator, it has no kernel", self)
            }
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
            | CKind::CurveTangentLine
            | CKind::CurveCurvature
            | CKind::PointOnEllipse
            | CKind::EllipseTangentLine
            | CKind::EllipseCurvature
            | CKind::SplineTangentLine
            | CKind::SplineCurvature
            | CKind::HorizontalPoints
            | CKind::VerticalPoints
            | CKind::FrameUnit
            | CKind::FrameAlign
            | CKind::Project
            | CKind::Ground
            | CKind::Fix
            | CKind::Ccw
            | CKind::Cw => return None,
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
    /// A `claim` (Solvent §9.7): stated as *expected to add no rank*.  A claim is no equation —
    /// the solve, the decomposition and the drag walk all leave it out, so it can never move the
    /// geometry or weld two figures — and the diagnosis alone judges it, against the drawing the
    /// rest of the document made: a theorem when it holds and adds no rank, `violated` when it
    /// does not hold, `consuming` when enforcing it would have taken a freedom.  It travels like
    /// any flag: through `graft`, the document and the JSON.
    pub claim: bool,
    /// The unknown this constraint's number is written in terms of, when its dimension names a
    /// free variable — see `expr::Free`.  At most one, which is why it lives here and not on the
    /// argument: one appended column, one `(m, c)` pair, one twin kernel.  Derived state, written
    /// only by `expr::evaluate`, so `Constraint::new` and every rebuild leave it empty.
    pub free: Option<Free>,
    /// The classes the statement carries (§13.2) — a dimension's callout is drawn in them over
    /// `.dimension`, and `display: none` on one leaves the callout out of the layout altogether.
    /// Presentation: nothing that solves, diagnoses or decomposes reads it, and `same_constraint`
    /// does not compare it.
    pub class: crate::style::Classes,
}

impl Constraint {
    /// Whether this constraint *acts* on the drawing — the one predicate behind "everything that
    /// must be satisfied".  A soft one is a transient the solve may miss; a claim is a question
    /// about the drawing rather than part of it, and neither is something a consumer asking for
    /// the constraints that determine the figure wants back.
    pub fn acts(&self) -> bool {
        !self.soft && !self.claim
    }

    pub fn new(kind: CKind, args: Vec<Arg>) -> Constraint {
        debug_assert_eq!(args.len(), kind.spec().len(), "{:?} arity", kind);
        Constraint {
            id: 0,
            kind,
            args,
            soft: false,
            intrinsic: false,
            claim: false,
            free: None,
            class: Default::default(),
        }
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

    /// A frame's rotor held to the unit circle — intrinsic: `Sketch::frame` states it and
    /// nothing else does, the arc's bargain.
    pub fn frame_unit(frame: EntRef) -> Constraint {
        let mut c = Constraint::new(CKind::FrameUnit, vec![Arg::Ent(frame)]);
        c.intrinsic = true;
        c
    }

    /// A frame's rotor kept on its chord, the chord's length its own unknown — intrinsic, the
    /// other half of what `Sketch::frame` states.
    pub fn frame_align(sk: &Sketch, frame: EntRef) -> Constraint {
        let mut args = vec![Arg::Ent(frame), Arg::Num(0.0)];
        args[1] = Arg::Num(seed_param(sk, CKind::FrameAlign, &args, 1));
        let mut c = Constraint::new(CKind::FrameAlign, args);
        c.intrinsic = true;
        c
    }

    /// Two points as images of one point in space — the planes read off their memberships,
    /// through the same seam the document readers use (`io::seed_omitted`), so a Rust caller
    /// and a document are refused by one rule.
    pub fn project(sk: &Sketch, a: EntRef, b: EntRef) -> Result<Constraint, String> {
        let mut args = vec![Arg::Ent(a), Arg::Ent(b), Arg::Num(0.0), Arg::Num(0.0)];
        crate::io::seed_omitted(sk, CKind::Project, &mut args, |i| i >= 2)?;
        Ok(Constraint::new(CKind::Project, args))
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
        match (self.kind.family_kernel(), self.curve_of()) {
            (Some(fk), Some(e)) => {
                let def = sk.curves[e.i()].def as usize;
                kernels::N_KERNELS + FamilyKernel::ALL.len() * def + fk as usize
            }
            _ => self.kernel_id(),
        }
    }

    /// The curve a per-definition kernel is over — the spec's `Curve` slot, wherever it stands.
    pub fn curve_of(&self) -> Option<EntRef> {
        let (e, _) = self.kind.contact_on(SpecKind::Curve)?;
        Some(self.args[e].ent())
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
        // a curve's kernel belongs to its definition, so its row count is a fact about the kind
        match self.kind.family_kernel() {
            Some(fk) => fk.n_res(),
            None => kernels::kernel(self.kernel()).n_res,
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
        // a curve contact carries its family's compiled body — two tapes, or a whole trace
        // block — and the numbers the instance was given.  They are the same for every contact
        // with the same curve, and duplicated per constraint because that is where a block
        // already has room for numbers — which is what keeps the kernel table `fn`-pointered and
        // ignorant of curves.
        if let Some(curve) = self.curve_of() {
            let cv = &sk.curves[curve.i()];
            let d = &sk.curve_defs[cv.def as usize];
            return match &d.body {
                crate::model::CurveBody::Exprs { x, y } => {
                    let mut k =
                        Vec::with_capacity(3 + x.flat.len() + y.flat.len() + cv.values.len());
                    k.push(d.vars.len() as f64);
                    k.push(x.flat.len() as f64);
                    k.push(y.flat.len() as f64);
                    k.extend_from_slice(&x.flat);
                    k.extend_from_slice(&y.flat);
                    k.extend_from_slice(&cv.values);
                    k
                }
                // a trace contact adds the march's home — the parameter its instance is
                // anchored at — and, for a curve of a drawn instance, the pose on the sheet
                // the home solve starts from: instance data the way the values are, read off
                // the sketch here so a refresh carries the pose the drawing has now
                crate::model::CurveBody::Trace(l) => {
                    let ci = curve.i();
                    let n_q = l.n_q();
                    let mut k = Vec::with_capacity(3 + cv.values.len() + l.flat.len() + n_q);
                    k.push(sk.curve_home(ci));
                    k.push(cv.values.len() as f64);
                    k.extend_from_slice(&cv.values);
                    match sk.curve_pose(ci) {
                        Some(pose) => {
                            k.push(1.0);
                            k.extend_from_slice(&l.flat);
                            k.extend(pose);
                        }
                        None => {
                            k.push(0.0);
                            k.extend_from_slice(&l.flat);
                            k.extend(std::iter::repeat(0.0).take(n_q));
                        }
                    }
                    k
                }
            };
        }
        match self.kind {
            CKind::Distance | CKind::HorizontalDistance | CKind::VerticalDistance => {
                vec![self.args[2].num()]
            }
            CKind::DragTarget => {
                vec![self.args[1].num(), self.args[2].num(), self.args[3].num()]
            }
            CKind::Angle
            | CKind::ParallelDistance
            | CKind::PointLineDistance
            | CKind::AnnularDistance => vec![self.args[2].num()],
            CKind::Radius => vec![self.args[1].num()],
            CKind::TangentLineCircle => vec![self.args[2].num()],
            CKind::TangentCircleCircle => {
                vec![if matches!(self.args[2], Arg::Bool(true)) { 1.0 } else { -1.0 }]
            }
            // the fold line in each plane's own coordinates — validated at the add, so a pair
            // with none cannot be here; recomputed with every refresh, being a cross product
            // and four dots, rather than kept in a second skip set beside the curve contacts
            CKind::Project => {
                let basis = |i: usize| &sk.planes[self.args[i].ent().i()].basis;
                let (da, db) = crate::plane::fold_line(basis(2), basis(3))
                    .expect("a projection between parallel planes is refused at the add");
                vec![da[0], da[1], db[0], db[1]]
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

    /// A contact on a curve *family* instance (`PointOnCurve`) and the Param holding where
    /// along it — the third family, asked separately from the other two for the one thing it
    /// shares with a spline and not with an ellipse: a bounded parameter, clamped to the
    /// `over (a, b)` its instance declared (`curve::clamp_contacts`).
    pub fn family_contact(&self) -> Option<(EntRef, u32)> {
        let (e, t) = self.kind.contact_on(SpecKind::Curve)?;
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
            // the parameter, the curve's coordinates, then what it touches — the frame's
            // column order, so the gradients that come back are the row
            CKind::CurveTangentLine => {
                [vec![self.args[2].param()], sk.entity_params(e(0)), ln(1)].concat()
            }
            CKind::CurveCurvature => {
                [vec![self.args[2].param()], sk.entity_params(e(0)), centre(1), vec![rad(1)]].concat()
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
            // the rotor alone; and the chord's length, then the frame's six numbers in
            // `entity_params` order — the kernels' column layouts exactly
            CKind::FrameUnit => sk.own_params(e(0)),
            CKind::FrameAlign => {
                [vec![self.args[1].param()], sk.entity_params(e(0))].concat()
            }
            // the two images, then each plane's origin and rotor — the kernel's twelve columns
            CKind::Project => {
                let datum = |i: usize| {
                    let f = sk.frame_of(e(i));
                    [sk.point_params(f.origin as usize).to_vec(), vec![f.c, f.s]].concat()
                };
                [pt(0), pt(1), datum(2), datum(3)].concat()
            }
            CKind::Ground | CKind::Fix | CKind::Ccw | CKind::Cw => {
                unreachable!("{:?} is a gauge and is never in a sketch", self.kind)
            }
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
        // the language curves' contacts start where their polyline comes nearest — the same
        // readings as a spline's, over the drawn curve rather than a basis
        (CKind::CurveTangentLine, 2) => {
            // nearest the infinite line, since a tangency is about direction
            let [ax, ay, bx, by] = sk.line_params(args[1].ent().i());
            let g = |p: u32| sk.params[p as usize].value;
            let (ax, ay, dx, dy) = (g(ax), g(ay), g(bx) - g(ax), g(by) - g(ay));
            let len = dx.hypot(dy).max(kernels::MIN_LINE_LEN);
            sk.curve_nearest_by(args[0].ent().i(), |px, py| {
                ((px - ax) * dy - (py - ay) * dx).abs() / len
            })
        }
        (CKind::CurveCurvature, 2) => {
            let (cx, cy) = sk.point_xy(sk.round_center(args[1].ent()));
            sk.curve_nearest_by(args[0].ent().i(), |px, py| (px - cx).hypot(py - cy))
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
        // the chord's length as drawn, asked of the one function that also scales the rotor —
        // a degenerate frame must read the same to both, or the preconditioning and the row's
        // seed would disagree about a drawing neither can see
        (CKind::FrameAlign, 1) => {
            let f = sk.frame_of(args[0].ent());
            sk.frame_chord(f.origin as usize, f.toward as usize).1
        }
        _ => 0.0,
    }
}

/// An entity slot the core fills when the caller leaves it out — the entity counterpart of
/// `seed_param`: a projection's planes are its points' memberships, and nobody writes them.
/// `Err` is the reason it cannot, in the words the caller reports.
pub fn infer_entity(sk: &Sketch, kind: CKind, args: &[Arg], i: usize) -> Result<EntRef, String> {
    match (kind, i) {
        (CKind::Project, 2 | 3) => {
            let p = args[i - 2].ent();
            sk.plane_of(p.i()).map(EntRef::plane).ok_or_else(|| {
                format!(
                    "{} is on no plane, so `project` cannot say which view it is in",
                    crate::io::entity_name(p)
                )
            })
        }
        _ => Err(format!("{} leaves nothing for the core to infer in slot {i}", kind.name())),
    }
}

/// What a kind refuses once its arguments are all in — the checks that need the sketch, which
/// the type check on the spec cannot make.  One rule for the elaborator, the document readers,
/// the FFI and the Rust constructors alike.
pub fn validate(sk: &Sketch, kind: CKind, args: &[Arg]) -> Result<(), String> {
    match kind {
        // a curvature reads the curve's second derivative, which a traced curve cannot give:
        // its block's kernels have first derivatives only, and a residual by difference would
        // solve to a slightly wrong circle and call it right
        CKind::CurveCurvature => {
            let cv = &sk.curves[args[0].ent().i()];
            if let crate::model::CurveBody::Trace(_) = sk.curve_defs[cv.def as usize].body {
                return Err(format!(
                    "{} is traced, and a traced curve has no curvature to state a circle \
                     against — only a curve from a computed point does",
                    crate::io::entity_name(args[0].ent())
                ));
            }
            Ok(())
        }
        CKind::Project => {
            let (pa, pb) = (args[2].ent(), args[3].ent());
            if pa == pb {
                return Err(format!(
                    "both points are on {}, and one view relates nothing to itself",
                    crate::io::entity_name(pa)
                ));
            }
            let basis = |e: EntRef| &sk.planes[e.i()].basis;
            if crate::plane::fold_line(basis(pa), basis(pb)).is_none() {
                return Err(format!(
                    "{} and {} are parallel, so no fold line relates their views",
                    crate::io::entity_name(pa),
                    crate::io::entity_name(pb)
                ));
            }
            Ok(())
        }
        _ => Ok(()),
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
        SpecKind::Plane => ent == EntKind::Plane,
        _ => false,
    }
}
