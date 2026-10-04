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

#[allow(unused_imports)]
use crate::fmath::Det;
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
    /// **Two directed angles equal** (`l1 angle(l3, l4) l2`): the angle from `l1` to `l2` is the
    /// angle from `l3` to `l4`, both read as `Angle` reads one — counter-clockwise from the first
    /// line's direction, on the full turn — and `sense: cw` says the second pair turns the other
    /// way, which is what a mirror image does.  The shared free variable it replaces
    /// (`l1 angle(beta) l2` beside `l3 angle(beta) l4`) said the same with an unknown nobody
    /// wanted to name; this states it with no number at all, one row of degree 0.
    EqualAngle,
    /// **An arc's length along itself** (`length(L) a`): `r·θ`, with θ the arc's own sweep
    /// counter-clockwise from its start to its end, in (0, 2π] — what `Sketch::arc_angles`
    /// reads.  A magnitude, degree 1; the length a belt wraps or a circular pitch measures.
    ArcLength,
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
    /// A point on a curve written in the language.  The same shape as `PointOnSpline` — two
    /// residuals against one owned parameter, so the net one equation "a point lies on a curve"
    /// is worth — but the curve is an expression rather than a basis, so the kernel that
    /// evaluates it is chosen per *definition*, not per type.
    PointOnCurve,
    /// A point anywhere in space on the surface a curve of a view stands for (`CurveE::extrusion`:
    /// a prism's side generating under a motion that keeps the view, §6.15, issue #70): the
    /// point's place in the view lies on the curve, `PointOnCurve`'s two rows over the point's
    /// lift — so it is the surface's one equation, and its kernel is the definition's.
    PointOnExtrusion,
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
    /// Signed ordinates of a point relative to a datum: u follows its rotor, v is left of it.
    CoordinateU,
    CoordinateV,
    /// A solved view's quaternion held to the unit sphere: `|q|² = 1`.  Intrinsic — minted with
    /// a view's unknowns (`Sketch::mint_attitude`) and nowhere else, `FrameUnit`'s form one
    /// dimension up.
    QuatUnit,
    /// A hidden point in space held at the lift of the view point it stands for, the view's
    /// attitude read off its unknowns: `X − R(q)·(a + a′, b + b′, d) = 0`.  Three rows over the
    /// hidden point's three Params, so net nothing.  Intrinsic, minted by `Sketch::lift_point`.
    Lift,
    /// The same over a *stated* view, whose basis is constants: `X − (o + a′·u + b′·v) = 0`.
    /// Which twin a lift is follows the view's `att` alone, never its params' fixed flags.
    LiftFixed,
    /// **Relations in space** (`docs/spatial-constraints-plan.md`): statements between points
    /// and lines drawn in *different* views, read over the hidden points the views lift them to
    /// (`model::LiftE`).  Their slots name the drawn entities, as every relation's do;
    /// `Sketch::add` mints the lifts they read, so no caller mints one by hand, and a point on
    /// no view is refused (`validate`).  Each is the word it is across views (`program::reading`
    /// reads which from the operands' views), and none has a callout: a length in space is not
    /// a figure on any one view.
    ///
    /// The same point in space: `X − Y = 0`.
    Coincident3,
    /// The true length between two points: `|X − Y|² = d²`.
    Distance3,
    /// A point's distance from a line's infinite extension in space, a magnitude.
    PointLine3,
    /// The common-perpendicular distance between two lines, a magnitude stated along the side
    /// the seed was on (`sign`, read off the geometry when it is stated and never written by a
    /// person).  Parallel lines have no common perpendicular and are refused.
    LineLine3,
    /// The unsigned angle between two lines' directions, 0 to half a turn, by its cosine.
    Angle3,
    Perpendicular3,
    /// Two rows across the first line's direction — see `kernels::parallel3_res`.
    Parallel3,
    /// A point on a plane in space, over a solved plane's quaternion and offset; the plane is any
    /// view, not only the point's own.
    PointOnPlane,
    /// The same over a *stated* plane, its normal and offset constants — `Lift`'s split again:
    /// which twin a statement is follows the plane's `att`, and `Sketch::add` and
    /// `Sketch::free_attitude` keep it so (`CKind::attitude_twin`).
    PointOnPlaneFixed,
    /// A point on a circle drawn in a view: on the sphere of its radius about the centre's lift,
    /// and on the centre's view — two rows, over that view's solved attitude.
    PointOnCircle3,
    /// The same over a stated view.
    PointOnCircle3Fixed,
    /// **The rest of the words in space**, inferred like the eleven above wherever a
    /// relation's operands are drawn in different views.  A point on a line's infinite extension:
    /// two rows across the line (`kernels::point_on_line3_res`), since the magnitude has no
    /// gradient where it vanishes.
    PointOnLine3,
    /// Two lines of equal true length.
    EqualLength3,
    /// `p distance(d, along: n) P`: a point's signed distance along a plane's normal, over a
    /// solved plane's unknowns — the one relation that names a plane rather than lifting one, so
    /// it is in space whatever views its operands are in.
    PointPlaneDistance,
    /// The same over a stated plane (`attitude_twin`'s pair, as `PointOnPlane`'s is).
    PointPlaneDistanceFixed,
    /// A line on a plane in space: both its ends, over a solved plane's unknowns.
    LineOnPlane,
    /// The same over a stated plane.
    LineOnPlaneFixed,
    /// A point on a sphere: `|X − C| − r` over the point's and the centre's lifts.
    SphereOn,
    /// A sphere's radius, `radius`'s kernel over the sphere's own Param.
    SphereRadius,
    /// A sphere touching a line: the line's distance from the centre, stated as the radius.
    SphereTangentLine,
    /// Two spheres touching, outside or inside as the seed stands (`side`, inferred).
    SphereTangentSphere,
    /// **A view folded from a solved one**: the child's quaternion is its parent's turned
    /// by the fold, `q_c − q_P ⊗ fold_rotor(θ) = 0` — four rows over the child's four quaternion
    /// unknowns, so a folded view adds no freedom and needs no unit row of its own.  `fold` is
    /// the angle written in the plane's brackets: a number, or an expression over the
    /// document's free variable, which is how `fold: beta` is solved for (the free twin,
    /// `hinge_free`).  Stated by the elaborator from the declaration, never written as a
    /// relation, and recorded against the plane's statement.
    Hinge,
    /// A plane stood off a solved one (`from: P, offset: …`): the same attitude, `q_c = q_P`.
    HingeParallel,
    /// **A circle on a sphere**, `c on s`: every point of a circle drawn in a view on a
    /// sphere — the sphere's centre on the circle's axis (two rows, across the view) and
    /// `√(|S − C|² + r²) = R` — over the circle's view solved.  What a gear blank's toe or heel
    /// circle is to its end sphere.  A circle *tangent* to a sphere is refused as ambiguous.
    CircleOnSphere,
    /// The same over a stated view.
    CircleOnSphereFixed,
    /// A point the midpoint of a line drawn in another view, in space.
    Midpoint3,
    /// Two points each the other's image in a line, in space: the half turn about the line — the
    /// mirror in it, which is what `symmetry` means on a page, read one dimension up.
    Symmetric3,
    /// **A point on a cone**, `p on k`: the point's distance from the cone's generator in
    /// its meridian half-plane, `ρ cos α − h sin α` (ρ its distance from the axis, h its height
    /// along it from the apex) — a length, zero on the nappe the axis points into.
    ConeOn,
    /// A point on a cylinder, `p on c`: its distance from the axis, stated as the radius column
    /// (`point_line3`'s free twin at (m, c) = (1, 0), `SphereTangentLine`'s bargain).
    CylinderOn,
    /// `angle(θ) k`: a cone's half-angle, the kind's own Param, degree 0.
    ConeAngle,
    /// `radius(r) c`: a cylinder's radius, `radius`'s kernel over the cylinder's own Param.
    CylinderRadius,
    /// `c tangent l`: a line touching a cylinder — its common perpendicular with the axis is the
    /// radius (`line_line3`'s free twin, the side read off the seed as a skew distance's is).
    CylinderTangentLine,
    /// `k1 tangent(M) k2`: two cones touching at a point with one tangent plane there — the
    /// second's surface normal at M square to both of the first's tangent directions (its
    /// generator and its circle), two rows of degree 0.  That M is on each is said by `M on k`
    /// beside it, as a tangency at a named end leaves the on-circle to its own statement.  What a
    /// hypoid's pitch cones do at the mean point.
    ConeTangentCone,
    /// `fold: along l` — the child contains a line drawn in its parent, folded square to the
    /// parent about it: the hinge over a half-angle rotor `(hc, hs)` of the constraint's own,
    /// held to the unit circle and to the line's bearing (`kernels::hinge_along_res`).  Net
    /// four equations over the child's four quaternion unknowns; where the line crosses is a
    /// `PointOnPlane` stated beside it.
    HingeAlong,
    /// `project` where either view is solved: the projector rule in space over both images'
    /// hidden points and both views' quaternions, `(n_A × n_B)·(X_A − X_B) = 0`
    /// (`kernels::project_free_res`).  The same statement as `Project` — the word, the operands
    /// and the inferred planes — and the twin `Sketch::add` picks when a view it reads is solved.
    ProjectSolved,
    /// **A mate between solved views**: `f against g` where the two faces' planes share an
    /// attitude and the datum's offset is solved — the placed plane's offset unknown held at the
    /// datum's plus the gap between the faces' ordinates, `d_f − d_g − gap = 0`.  Stated by the
    /// elaborator from the `against` statement and never written as a relation, as a hinge is.
    Mate,
    /// The **gauges** and the **orientation predicates** (spec §9.2, §9.6; issue #47, item 5):
    /// statements written as every other constraint is — an operator, its operands, a class, a
    /// placement — and settled through the same table, but **applied by the elaborator rather
    /// than held by the model**: `fix` marks parameters fixed at the numbers it states, `ccw` and
    /// `cw` record a root choice in `Sketch::branches`.  They own no kernel, add no row, and are in
    /// no `Constraint` the sketch holds — so they are **not in `ALL_KINDS`**, the registry never
    /// publishes them, and `CKind::gauge` is how every table that would otherwise reach for a
    /// kernel tells them apart.  A `claim` on one is refused: a claim is judged by rank over
    /// rows, and these have none.
    Fix,
    Ccw,
    Cw,
}

/// Which of a curve definition's kernels a kind runs through: the table holds these four per
/// definition, in this order (`System::kernel_table`), and `Constraint::kernel_id_in` counts
/// them so — the one statement of what "a kind whose kernel is per definition" means.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FamilyKernel {
    Contact,
    Tangent,
    Curvature,
    /// A point in space on the surface the curve stands for (`CKind::PointOnExtrusion`).
    Extrusion,
}

impl FamilyKernel {
    pub const ALL: [FamilyKernel; 4] =
        [FamilyKernel::Contact, FamilyKernel::Tangent, FamilyKernel::Curvature, FamilyKernel::Extrusion];

    /// Rows per constraint — a fact about the kind, not asked of a kernel it may not have yet.
    pub fn n_res(self) -> usize {
        match self {
            FamilyKernel::Contact | FamilyKernel::Tangent | FamilyKernel::Extrusion => 2,
            FamilyKernel::Curvature => 3,
        }
    }
}

/// Every concrete constraint type, in the order the registry lists them.
pub const ALL_KINDS: [CKind; 79] = [
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
    CKind::PointOnCurve,
    CKind::PointOnExtrusion,
    CKind::CurveTangentLine,
    CKind::CurveCurvature,
    CKind::FrameUnit,
    CKind::FrameAlign,
    CKind::Project,
    CKind::CoordinateU,
    CKind::CoordinateV,
    CKind::QuatUnit,
    CKind::Lift,
    CKind::LiftFixed,
    CKind::Coincident3,
    CKind::Distance3,
    CKind::PointLine3,
    CKind::LineLine3,
    CKind::Angle3,
    CKind::Perpendicular3,
    CKind::Parallel3,
    CKind::PointOnPlane,
    CKind::PointOnPlaneFixed,
    CKind::PointOnCircle3,
    CKind::PointOnCircle3Fixed,
    CKind::Hinge,
    CKind::HingeParallel,
    CKind::HingeAlong,
    CKind::ProjectSolved,
    CKind::PointOnLine3,
    CKind::EqualLength3,
    CKind::PointPlaneDistance,
    CKind::PointPlaneDistanceFixed,
    CKind::SphereOn,
    CKind::SphereRadius,
    CKind::SphereTangentLine,
    CKind::SphereTangentSphere,
    CKind::LineOnPlane,
    CKind::LineOnPlaneFixed,
    CKind::CircleOnSphere,
    CKind::CircleOnSphereFixed,
    CKind::Midpoint3,
    CKind::Symmetric3,
    CKind::ConeOn,
    CKind::CylinderOn,
    CKind::ConeAngle,
    CKind::CylinderRadius,
    CKind::CylinderTangentLine,
    CKind::ConeTangentCone,
    CKind::Mate,
    CKind::EqualAngle,
    CKind::ArcLength,
];

/// `along:` says which axis a run or a rise is measured on.  It is the one selector that fills no
/// slot — it *chooses the kind* and is gone — so this table is the only place its words exist,
/// read both by `infix_op` to make the choice and by the elaborator to say what was wrong with a
/// word that is not one of them (issue #48, item 4).  A second list would be a second answer.
pub const ALONG: [(&str, CKind); 9] = [
    ("x", CKind::HorizontalDistance),
    ("y", CKind::VerticalDistance),
    ("u", CKind::CoordinateU),
    ("v", CKind::CoordinateV),
    // a point's signed distance along a plane's normal, in space whatever views it is in
    ("n", CKind::PointPlaneDistance),
    // the same two kinds with the direction named outright, which is the sign said in a word
    ("right", CKind::HorizontalDistance),
    ("left", CKind::HorizontalDistance),
    ("up", CKind::VerticalDistance),
    ("down", CKind::VerticalDistance),
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
    use EntKind::{Arc, Circle, Cone, Curve, Cylinder, Line, Plane, Point, Sphere, Spline};
    let round = |k: EntKind| matches!(k, Circle | Arc);
    if matches!(sel("along").as_deref(), Some("u" | "v" | "n")) && (word != "distance" || (a, b) != (Point, Plane)) {
        return None;
    }
    Some(match word {
        "on" => match (a, b) {
            (Point, Line) => CKind::PointOnLine,
            (Point, k) if round(k) => CKind::PointOnCircle,
            (Point, Spline) => CKind::PointOnSpline,
            (Point, Curve) => CKind::PointOnCurve,
            // in space whatever view the point is in: a plane is a place, not a picture
            (Point, Plane) => CKind::PointOnPlane,
            (Point, Sphere) => CKind::SphereOn,
            (Point, Cone) => CKind::ConeOn,
            (Point, Cylinder) => CKind::CylinderOn,
            (k, Sphere) if round(k) => CKind::CircleOnSphere,
            (Line, Plane) => CKind::LineOnPlane,
            _ => return None,
        },
        "distance" => match (a, b) {
            // which of the three a pair of points means is `along:`, and the run and the rise
            // are signed from the first point to the second — so they do not commute
            (Point, Point) => match sel("along") {
                None => CKind::Distance,
                Some(w) => ALONG.iter().find(|(n, k)| *n == w && matches!(k, CKind::HorizontalDistance | CKind::VerticalDistance)).map(|(_, k)| *k)?,
            },
            (Point, Plane) => ALONG.iter().find(|(n, k)| Some(*n) == sel("along").as_deref() && matches!(k, CKind::CoordinateU | CKind::CoordinateV | CKind::PointPlaneDistance)).map(|(_, k)| *k)?,
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
            (Curve, Line) => CKind::CurveTangentLine,
            // a sphere touches a line or a sphere in space (a circle is `on` one, never tangent)
            (Sphere, Line) => CKind::SphereTangentLine,
            (Sphere, Sphere) => CKind::SphereTangentSphere,
            // a line touching a cylinder, and two cones touching at the point in the parentheses
            (Cylinder, Line) => CKind::CylinderTangentLine,
            (Cone, Cone) => CKind::ConeTangentCone,
            _ => return None,
        },
        "equal" => match (a, b) {
            (Line, Line) => CKind::EqualLength,
            (x, y) if round(x) && round(y) => CKind::EqualRadius,
            _ => return None,
        },
        "curvature" => match (a, b) {
            (Spline, k) if round(k) => CKind::SplineCurvature,
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
        ("radius", EntKind::Sphere) => CKind::SphereRadius,
        ("radius", EntKind::Cylinder) => CKind::CylinderRadius,
        // an arc's length along itself: the other way a round thing is dimensioned
        ("length", Arc) => CKind::ArcLength,
        // a cone's half-angle: the one prefix `angle`, since between two lines it is infix
        ("angle", EntKind::Cone) => CKind::ConeAngle,
        ("distance", Line) => CKind::Distance,
        _ => return None,
    })
}

/// Every word the language writes a **constraint** with (spec §9.1) — the gauges and the
/// orientation predicates among them, read by the one relation parser and settled by the one
/// table (`gauge_op`), so a class, a placement and the chain's lookahead treat them as any
/// other word.  None of the three is a prefix word a chain can open a link with: `prefix_op`
/// declines them, so `fix(x == 0) point p -> …` is no chain.
pub const OPERATORS: [&str; 22] = [
    "on", "distance", "tangent", "equal", "curvature", "horizontal", "vertical", "angle",
    "radius", "length", "coincident", "midpoint", "parallel", "perpendicular", "symmetry", "project",
    "fix", "ccw", "cw",
    // **the words that relate two solids** (§9.8).  They are operators so that a statement
    // reads the way every other statement does; they settle to no `CKind` and compile no row,
    // because a solid is evaluated after the drawing is solved and a claim about one is judged
    // rather than enforced — `solid_claim` is where the word is read.
    "clear", "inside", "fits",
];

/// The words that relate two **solids**, and what each asks (§9.8).  A statement in one of them
/// is a *claim*: judged at the pose, never solved for, and it can no more move geometry than a
/// `project` claim can.
///
/// Kept beside `OPERATORS` rather than in `CKind`, because a `CKind` is a thing with a kernel and
/// these have none.  Asked by `program::constrain` before the spec is read, the way `gauge_op` is.
pub fn solid_word(w: &str) -> Option<SolidWord> {
    Some(match w {
        "clear" => SolidWord::Clear,
        "inside" => SolidWord::Inside,
        "fits" => SolidWord::Fits,
        _ => return None,
    })
}

/// What a claim about two solids asks.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SolidWord {
    /// `a clear(2mm) b` — disjoint, and no point of one nearer than that to the other.
    Clear,
    /// `a inside b` — every point of the left is a point of the right.
    Inside,
    /// `a fits(0.15mm) b` — inside, with that much to spare all round.
    Fits,
}

impl SolidWord {
    pub fn as_str(self) -> &'static str {
        match self {
            SolidWord::Clear => "clear",
            SolidWord::Inside => "inside",
            SolidWord::Fits => "fits",
        }
    }

    /// Whether the word takes a distance in its parentheses.  `inside` asks about containment
    /// and nothing else; the other two ask for room.
    pub fn takes_gap(self) -> bool {
        self != SolidWord::Inside
    }
}

pub fn is_operator(w: &str) -> bool {
    OPERATORS.contains(&w)
}

/// The gauges and the orientation predicates, by word: settled **before** the operands' kinds
/// are asked, since what `fix` may hold depends on the entity (`fix(r == 25) c`, `fix(x == 0) p`)
/// and `ccw(a, b, c)` has no operand outside its parentheses.  What each operand must be is
/// checked where the statement is applied (`program::apply_gauge`).
pub fn gauge_op(word: &str) -> Option<CKind> {
    Some(match word {
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
    /// `radius(25) circle1`, `horizontal line1`, `fix(x == 0, y == 0) p1`
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
    /// A curve written in the language — see `model::CurveDef`.
    Curve,
    /// The datum: a plane, whose rotor the two intrinsics read and whose basis `Project` does.
    Plane,
    /// A sphere: a centre drawn in a view and a radius, on no sheet.
    Sphere,
    /// A cone and a cylinder: an axis drawn in a view and a number each owns.
    Cone,
    Cylinder,
    /// The operand of `fix`: an entity of any kind whose own numbers the statement holds
    /// (`fix(r == 25) c`).  Filled from a reference like an entity slot; which numbers that
    /// kind has is the gauge's own check.
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
            | SpecKind::Curve
            | SpecKind::Plane
            | SpecKind::Sphere
            | SpecKind::Cone
            | SpecKind::Cylinder
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
                | SpecKind::Curve
                | SpecKind::Plane
                | SpecKind::Sphere
                | SpecKind::Cone
                | SpecKind::Cylinder
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
            SpecKind::Curve => "curve",
            SpecKind::Plane => "plane",
            SpecKind::Sphere => "sphere",
            SpecKind::Cone => "cone",
            SpecKind::Cylinder => "cylinder",
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
            CKind::EqualAngle => "EqualAngle",
            CKind::ArcLength => "ArcLength",
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
            CKind::PointOnCurve => "PointOnCurve",
            CKind::PointOnExtrusion => "PointOnExtrusion",
            CKind::CurveTangentLine => "CurveTangentLine",
            CKind::CurveCurvature => "CurveCurvature",
            CKind::FrameUnit => "FrameUnit",
            CKind::FrameAlign => "FrameAlign",
            CKind::Project => "Project",
            CKind::CoordinateU => "CoordinateU",
            CKind::CoordinateV => "CoordinateV",
            CKind::QuatUnit => "QuatUnit",
            CKind::Lift => "Lift",
            CKind::LiftFixed => "LiftFixed",
            CKind::Coincident3 => "Coincident3",
            CKind::Distance3 => "Distance3",
            CKind::PointLine3 => "PointLine3",
            CKind::LineLine3 => "LineLine3",
            CKind::Angle3 => "Angle3",
            CKind::Perpendicular3 => "Perpendicular3",
            CKind::Parallel3 => "Parallel3",
            CKind::PointOnPlane => "PointOnPlane",
            CKind::PointOnPlaneFixed => "PointOnPlaneFixed",
            CKind::PointOnCircle3 => "PointOnCircle3",
            CKind::PointOnCircle3Fixed => "PointOnCircle3Fixed",
            CKind::PointOnLine3 => "PointOnLine3",
            CKind::EqualLength3 => "EqualLength3",
            CKind::PointPlaneDistance => "PointPlaneDistance",
            CKind::PointPlaneDistanceFixed => "PointPlaneDistanceFixed",
            CKind::SphereOn => "SphereOn",
            CKind::CircleOnSphere => "CircleOnSphere",
            CKind::CircleOnSphereFixed => "CircleOnSphereFixed",
            CKind::Midpoint3 => "Midpoint3",
            CKind::Symmetric3 => "Symmetric3",
            CKind::LineOnPlane => "LineOnPlane",
            CKind::LineOnPlaneFixed => "LineOnPlaneFixed",
            CKind::SphereRadius => "SphereRadius",
            CKind::SphereTangentLine => "SphereTangentLine",
            CKind::ConeOn => "ConeOn",
            CKind::CylinderOn => "CylinderOn",
            CKind::ConeAngle => "ConeAngle",
            CKind::CylinderRadius => "CylinderRadius",
            CKind::CylinderTangentLine => "CylinderTangentLine",
            CKind::ConeTangentCone => "ConeTangentCone",
            CKind::SphereTangentSphere => "SphereTangentSphere",
            CKind::Hinge => "Hinge",
            CKind::HingeParallel => "HingeParallel",
            CKind::HingeAlong => "HingeAlong",
            CKind::ProjectSolved => "ProjectSolved",
            CKind::Mate => "Mate",
            CKind::Fix => "Fix",
            CKind::Ccw => "Ccw",
            CKind::Cw => "Cw",
        }
    }

    /// A statement the elaborator applies rather than a constraint the model holds — see the
    /// variants' note.  Asked wherever a table would otherwise reach for a kernel.
    pub fn gauge(self) -> bool {
        matches!(self, CKind::Fix | CKind::Ccw | CKind::Cw)
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
            // the run and the rise: a magnitude, with `along` naming the axis (`x`, `y` — either
            // way along it, the seed choosing) or the direction outright (`right`, `left`, `up`,
            // `down`), which is the same statement the sign used to make (§9.2)
            CKind::HorizontalDistance | CKind::VerticalDistance => {
                &[("p", S::Point), ("q", S::Point), ("d", S::Length), ("along", S::Str)]
            }
            CKind::Parallel | CKind::Perpendicular | CKind::EqualLength => {
                &[("l1", S::Line), ("l2", S::Line)]
            }
            // directed as it always was, counter-clockwise positive from `l1`'s direction; the
            // `sense` word is how a drawing says clockwise without writing a minus (§9.4)
            CKind::Angle => {
                &[("l1", S::Line), ("l2", S::Line), ("theta", S::Angle), ("sense", S::Str)]
            }
            // the second pair stands in the parentheses, as `symmetry`'s line does; `sense: cw`
            // says it turns the other way (§9.4)
            CKind::EqualAngle => &[
                ("l1", S::Line),
                ("l2", S::Line),
                ("l3", S::Line),
                ("l4", S::Line),
                ("sense", S::Str),
            ],
            CKind::ArcLength => &[("arc", S::Arc), ("l", S::Length)],
            // the number is a magnitude, and `side` says which side of `l1` its second line lies
            // on; omitted, both sides are solutions and the seed picks between them (§9.2)
            CKind::ParallelDistance => {
                &[("l1", S::Line), ("l2", S::Line), ("d", S::Length), ("side", S::Str)]
            }
            CKind::PointOnLine => &[("p", S::Point), ("line", S::Line)],
            CKind::PointLineDistance => {
                &[("p", S::Point), ("line", S::Line), ("d", S::Length), ("side", S::Str)]
            }
            CKind::PointOnCircle => &[("p", S::Point), ("circle", S::CircleOrArc)],
            CKind::Radius => &[("circle", S::CircleOrArc), ("r", S::Length)],
            CKind::EqualRadius => &[("c1", S::CircleOrArc), ("c2", S::CircleOrArc)],
            CKind::AnnularDistance => {
                &[("c1", S::CircleOrArc), ("c2", S::CircleOrArc), ("d", S::Length)]
            }
            // which side of the line the circle's centre is: the same two words a distance from
            // a line uses, and for the same reason — `side: -1` was a coin only a rendering could
            // check (§9.2, issue #48 item 4)
            CKind::TangentLineCircle => {
                &[("line", S::Line), ("circle", S::CircleOrArc), ("side", S::Str)]
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
            CKind::PointOnCurve | CKind::PointOnExtrusion => &[("p", S::Point), ("curve", S::Curve), ("t", S::Param)],
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
            CKind::CoordinateU | CKind::CoordinateV => &[("p", S::Point), ("frame", S::Plane), ("d", S::Length)],
            CKind::FrameUnit => &[("frame", S::Plane)],
            CKind::FrameAlign => &[("frame", S::Plane), ("r", S::Param)],
            CKind::QuatUnit => &[("plane", S::Plane)],
            // the view point and its view: the hidden point is the lift's own, found by the
            // point (`Sketch::lift_of`), and the plane is a real slot so a drag part, a
            // deletion and the topology key follow it
            CKind::Lift | CKind::LiftFixed => &[("p", S::Point), ("plane", S::Plane)],
            // the drawn entities, as every relation names them; the hidden points they read are
            // found by the points (`Sketch::lift_of`)
            CKind::Coincident3 => &[("p", S::Point), ("q", S::Point)],
            CKind::Distance3 => &[("p", S::Point), ("q", S::Point), ("d", S::Length)],
            CKind::PointLine3 => &[("p", S::Point), ("line", S::Line), ("d", S::Length)],
            // which side of the first line the second passes: ±1, read off the seed and never
            // written, as a tangency's side is when nobody names one
            CKind::LineLine3 => {
                &[("l1", S::Line), ("l2", S::Line), ("d", S::Length), ("sign", S::Int)]
            }
            CKind::Angle3 => &[("l1", S::Line), ("l2", S::Line), ("theta", S::Angle)],
            CKind::Perpendicular3 | CKind::Parallel3 => &[("l1", S::Line), ("l2", S::Line)],
            CKind::PointOnPlane | CKind::PointOnPlaneFixed => {
                &[("p", S::Point), ("plane", S::Plane)]
            }
            CKind::PointOnCircle3 | CKind::PointOnCircle3Fixed => {
                &[("p", S::Point), ("circle", S::CircleOrArc)]
            }
            CKind::PointOnLine3 => &[("p", S::Point), ("line", S::Line)],
            CKind::EqualLength3 => &[("l1", S::Line), ("l2", S::Line)],
            CKind::PointPlaneDistance | CKind::PointPlaneDistanceFixed => {
                &[("p", S::Point), ("plane", S::Plane), ("d", S::Length)]
            }
            CKind::SphereOn => &[("p", S::Point), ("sphere", S::Sphere)],
            CKind::CircleOnSphere | CKind::CircleOnSphereFixed => {
                &[("circle", S::CircleOrArc), ("sphere", S::Sphere)]
            }
            CKind::Midpoint3 => &[("p", S::Point), ("line", S::Line)],
            CKind::Symmetric3 => &[("p", S::Point), ("q", S::Point), ("line", S::Line)],
            CKind::LineOnPlane | CKind::LineOnPlaneFixed => &[("line", S::Line), ("plane", S::Plane)],
            CKind::SphereRadius => &[("sphere", S::Sphere), ("r", S::Length)],
            CKind::SphereTangentLine => &[("sphere", S::Sphere), ("line", S::Line)],
            CKind::ConeOn => &[("p", S::Point), ("cone", S::Cone)],
            CKind::CylinderOn => &[("p", S::Point), ("cylinder", S::Cylinder)],
            CKind::ConeAngle => &[("cone", S::Cone), ("theta", S::Angle)],
            CKind::CylinderRadius => &[("cylinder", S::Cylinder), ("r", S::Length)],
            // which side of the axis the line passes, read off the seed as a skew distance's is
            CKind::CylinderTangentLine => {
                &[("cylinder", S::Cylinder), ("line", S::Line), ("sign", S::Int)]
            }
            // the contact point stands in the parentheses, as `symmetry`'s line does
            CKind::ConeTangentCone => &[("k1", S::Cone), ("k2", S::Cone), ("at", S::Point)],
            // outside or inside, read off the seed when nobody says, as two circles' is
            CKind::SphereTangentSphere => {
                &[("s1", S::Sphere), ("s2", S::Sphere), ("external", S::Bool)]
            }
            // the child view and its parent, and how the one is turned from the other: an angle,
            // nothing (stood off), or a line of the parent's with the fold's own half-angle rotor
            CKind::Hinge => &[("plane", S::Plane), ("from", S::Plane), ("fold", S::Angle)],
            CKind::HingeParallel => &[("plane", S::Plane), ("from", S::Plane)],
            CKind::HingeAlong => &[
                ("plane", S::Plane),
                ("from", S::Plane),
                ("line", S::Line),
                ("hc", S::Param),
                ("hs", S::Param),
            ],
            // the two planes are real slots — so the drag part, the topology key, the graft
            // and a deletion follow them — and inferred ones, so nobody writes them
            CKind::Project | CKind::ProjectSolved => {
                &[("a", S::Point), ("b", S::Point), ("pa", S::Plane), ("pb", S::Plane)]
            }
            // the plane a mate places, the one it bears on, and the faces' ordinates' difference
            CKind::Mate => &[("plane", S::Plane), ("datum", S::Plane), ("gap", S::Float)],
            // the entity, and the numbers it holds, each pinned under the name of the field it
            // is (`model::EntKind::fields`): `fix(x == 0, y == 0) p`, `fix(r == 25) c`,
            // `fix(half == 30deg) k`.  Every scalar field a kind owns is a slot here, and which
            // of them the entity has is the gauge's own check
            CKind::Fix => &[
                ("of", S::Scalar),
                ("x", S::Param),
                ("y", S::Param),
                ("r", S::Param),
                ("half", S::Param),
            ],
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
            // a place along a curve: a parameter, not a length
            CKind::PointOnSpline
            | CKind::PointOnCurve
            | CKind::PointOnExtrusion
            | CKind::CurveTangentLine
            | CKind::CurveCurvature
            | CKind::SplineTangentLine
            | CKind::SplineCurvature
            => Some(Dim::SCALAR),
            // a fold's half-angle rotor: a pair of cosines, not a length
            CKind::HingeAlong => Some(Dim::SCALAR),
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
            | CKind::PointOnCurve
            | CKind::PointOnExtrusion => ("on", Infix),
            CKind::CurveTangentLine => ("tangent", Infix),
            CKind::CurveCurvature => ("curvature", Infix),
            // a measured separation: six kinds, told apart by the pair and by `along:`
            CKind::Distance
            | CKind::HorizontalDistance
            | CKind::VerticalDistance
            | CKind::PointLineDistance
            | CKind::ParallelDistance
            | CKind::AnnularDistance
            | CKind::CoordinateU
            | CKind::CoordinateV => ("distance", Infix),
            // touching: six kinds, told apart by the pair and by `at:`
            CKind::TangentLineCircle
            | CKind::TangentLineCircleAt
            | CKind::TangentCircleCircle
            | CKind::TangentArcLine
            | CKind::SplineTangentLine
            => ("tangent", Infix),
            CKind::EqualLength | CKind::EqualRadius => ("equal", Infix),
            CKind::SplineCurvature => ("curvature", Infix),
            // the fixity is the distinction: a line prefixed, a pair of points infixed
            CKind::Horizontal => ("horizontal", Prefix),
            CKind::HorizontalPoints => ("horizontal", Infix),
            CKind::Vertical => ("vertical", Prefix),
            CKind::VerticalPoints => ("vertical", Infix),
            // `angle` and `radius` keep their own words rather than folding into `distance`:
            // over two lines a Length means a parallel distance and an Angle means an angle, and
            // nothing but the number's unit could separate them
            CKind::Angle => ("angle", Infix),
            // the same word with a second pair where the number would be: an angle stated as
            // another angle rather than as a number
            CKind::EqualAngle => ("angle", Infix),
            CKind::Radius => ("radius", Prefix),
            CKind::ArcLength => ("length", Prefix),
            CKind::Coincident => ("coincident", Infix),
            CKind::Midpoint => ("midpoint", Infix),
            CKind::Parallel => ("parallel", Infix),
            CKind::Perpendicular => ("perpendicular", Infix),
            // the only kind with three entity slots, and the parentheses absorb the third
            CKind::Symmetric => ("symmetry", Infix),
            // two operands; the plane slots behind them are inferred and never spelled
            CKind::Project | CKind::ProjectSolved => ("project", Infix),
            // the gauges are prefix words like `horizontal`; the orientation predicates keep
            // a call, since `a ccw(c) b` would reorder three points that are symmetric
            CKind::Fix => ("fix", Prefix),
            CKind::Ccw => ("ccw", Call),
            CKind::Cw => ("cw", Call),
            // **the relations in space are the same words**: which kind a word means is the
            // operands' views, read where the statement is settled (`program::reading`), so
            // a lifted or described statement spells the word and the reading comes back
            CKind::Coincident3 => ("coincident", Infix),
            CKind::Distance3 | CKind::PointLine3 | CKind::LineLine3 => ("distance", Infix),
            CKind::PointPlaneDistance | CKind::PointPlaneDistanceFixed => ("distance", Infix),
            CKind::Angle3 => ("angle", Infix),
            CKind::Perpendicular3 => ("perpendicular", Infix),
            CKind::Parallel3 => ("parallel", Infix),
            CKind::EqualLength3 => ("equal", Infix),
            CKind::PointOnPlane | CKind::PointOnPlaneFixed => ("on", Infix),
            CKind::PointOnCircle3 | CKind::PointOnCircle3Fixed => ("on", Infix),
            CKind::PointOnLine3 | CKind::SphereOn => ("on", Infix),
            CKind::CircleOnSphere | CKind::CircleOnSphereFixed => ("on", Infix),
            CKind::Midpoint3 => ("midpoint", Infix),
            CKind::Symmetric3 => ("symmetry", Infix),
            CKind::LineOnPlane | CKind::LineOnPlaneFixed => ("on", Infix),
            CKind::SphereRadius => ("radius", Prefix),
            CKind::SphereTangentLine | CKind::SphereTangentSphere => ("tangent", Infix),
            CKind::ConeOn | CKind::CylinderOn => ("on", Infix),
            CKind::ConeAngle => ("angle", Prefix),
            CKind::CylinderRadius => ("radius", Prefix),
            CKind::CylinderTangentLine | CKind::ConeTangentCone => ("tangent", Infix),
            CKind::DragTarget
            | CKind::FrameUnit
            | CKind::FrameAlign
            | CKind::QuatUnit
            | CKind::Lift
            | CKind::LiftFixed
            // a hinge is a plane's brackets, never a relation anybody writes
            | CKind::Hinge
            | CKind::HingeParallel
            | CKind::HingeAlong
            // nor is a mate's row: the `against` statement is the solids' word
            | CKind::Mate => return None,
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
            (CKind::TangentLineCircle, 2) => Arg::Str("left".to_string()),
            (CKind::LineLine3, 3) => Arg::Int(1),
            (CKind::CylinderTangentLine, 2) => Arg::Int(1),
            (CKind::SphereTangentSphere, 2) => Arg::Bool(true),
            _ => match self.spec()[i].1 {
                SpecKind::Int => Arg::Int(0),
                SpecKind::Bool => Arg::Bool(false),
                SpecKind::Str => Arg::Str(String::new()),
                _ => Arg::Num(0.0),
            },
        }
    }

    /// The words a slot will take, where it takes a word rather than a number.
    ///
    /// `default_arg` above was the only place these ever appeared — as one default each, not as a
    /// vocabulary — so nothing checked them: `tangent(at: banana)` was accepted and silently meant
    /// `end`, because `contact_point` asks `s == "start"` and takes the other end when it is not
    /// (issue #48, item 4).  A word outside the set is refused at the key, and the registry
    /// publishes the set, so a front end offers what the core accepts rather than restating it.
    pub fn words(self, i: usize) -> Option<&'static [&'static str]> {
        Some(match (self, i) {
            (CKind::TangentArcLine, 2) => &["start", "end"][..],
            (CKind::TangentLineCircleAt, 2) => &["p1", "p2"][..],
            // the directions of `side_words`, and — for the run and the rise — the *axis* word
            // beside them, which names no direction: `along: x` is the magnitude form, either way
            // along the page's x, and the seed picks (§9.2).  `every_direction_is_a_word` holds
            // the two tables together.
            (CKind::PointLineDistance, 3) | (CKind::ParallelDistance, 3) => &["left", "right"][..],
            (CKind::TangentLineCircle, 2) => &["left", "right"][..],
            (CKind::HorizontalDistance, 3) => &["x", "right", "left"][..],
            (CKind::VerticalDistance, 3) => &["y", "up", "down"][..],
            (CKind::Angle, 3) => &["ccw", "cw"][..],
            (CKind::EqualAngle, 4) => &["ccw", "cw"][..],
            _ => return None,
        })
    }

    /// **The word that says which way, and the sign it stands for** (§9.2, issue #48 item 4).
    ///
    /// One table for the two questions a reader and a kernel ask of the same word: which words a
    /// slot takes, and what each of them *means*.  `left` is +1 of a line, because the signed
    /// distance is positive to the left of `p1 → p2`; `left` is −1 along the page, because the
    /// run is measured from the first point to the second.  The two are opposite numbers and the
    /// same English, which is exactly why the word is what a document writes and the sign is what
    /// the kernel gets.  A word not in the table — `x`, `y`, or the empty word an omitted
    /// selector holds — names *no* direction: the number is a magnitude, both ways are solutions,
    /// and the seed picks.
    pub fn side_words(self) -> Option<(usize, &'static [(&'static str, f64)])> {
        Some(match self {
            CKind::PointLineDistance | CKind::ParallelDistance => {
                (3, &[("left", 1.0), ("right", -1.0)][..])
            }
            CKind::TangentLineCircle => (2, &[("left", 1.0), ("right", -1.0)][..]),
            CKind::HorizontalDistance => (3, &[("right", 1.0), ("left", -1.0)][..]),
            CKind::VerticalDistance => (3, &[("up", 1.0), ("down", -1.0)][..]),
            CKind::Angle => (3, &[("ccw", 1.0), ("cw", -1.0)][..]),
            // the second pair turns the way the first does, or — `cw` — the other way round
            CKind::EqualAngle => (4, &[("ccw", 1.0), ("cw", -1.0)][..]),
            _ => return None,
        })
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
                    | (CKind::Project | CKind::ProjectSolved, 2 | 3)
                    | (CKind::LineLine3, 3)
                    | (CKind::CylinderTangentLine, 2)
                    | (CKind::SphereTangentSphere, 2)
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
    /// **A magnitude is never negative**, and a negative one is refused where it is written.
    ///
    /// `Distance` and `Radius` square the sign away in the kernel, so a minus said nothing at all.
    /// A distance *from a line* is a magnitude too now (issue #48, item 4): the sign used to say
    /// which side, which is a word (`side: left`) — and left as a number it was a coin a reader
    /// could only check by rendering.  Since a component's formals are substituted before this is
    /// asked, `p distance(-hw) axis` is caught at the call and not silently turned into the other side.
    pub fn magnitude(self) -> bool {
        matches!(
            self,
            CKind::Distance
                | CKind::Radius
                | CKind::PointLineDistance
                | CKind::ParallelDistance
                | CKind::Distance3
                | CKind::PointLine3
                | CKind::LineLine3
                | CKind::SphereRadius
                | CKind::CylinderRadius
                | CKind::ArcLength
        )
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

    /// A contact on a *spline* — the one that gates the span machinery: a spline contact
    /// addresses a span, is clamped to its knots and carries its span in the topology key.  A
    /// caller wanting only "does this run along something, and how fast" asks `param_scale`.
    pub fn contact_slots(self) -> Option<(usize, usize)> {
        self.contact_on(SpecKind::Spline)
    }

    /// The per-definition kernel this kind runs through, for the four kinds that have one.
    pub fn family_kernel(self) -> Option<FamilyKernel> {
        Some(match self {
            CKind::PointOnCurve => FamilyKernel::Contact,
            CKind::PointOnExtrusion => FamilyKernel::Extrusion,
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
            | CKind::EqualAngle
            | CKind::ArcLength
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
            | CKind::PointOnExtrusion
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
            | CKind::CoordinateU
            | CKind::CoordinateV
            // a view's own algebra, and a hidden point tied to the point it lifts: no contact
            | CKind::QuatUnit
            | CKind::Lift
            | CKind::LiftFixed
            // incidence and measure in space: no double root the screen knows how to look for
            | CKind::Coincident3
            | CKind::Distance3
            | CKind::PointLine3
            | CKind::LineLine3
            | CKind::Angle3
            | CKind::Perpendicular3
            | CKind::Parallel3
            | CKind::PointOnPlane
            | CKind::PointOnPlaneFixed
            | CKind::PointOnCircle3
            | CKind::PointOnCircle3Fixed
            | CKind::PointOnLine3
            | CKind::EqualLength3
            | CKind::PointPlaneDistance
            | CKind::PointPlaneDistanceFixed
            | CKind::SphereOn
            | CKind::SphereRadius
            | CKind::LineOnPlane
            | CKind::LineOnPlaneFixed
            | CKind::CircleOnSphere
            | CKind::CircleOnSphereFixed
            | CKind::Midpoint3
            | CKind::Symmetric3
            // touching in space is a magnitude row with a unit gradient, never the double root
            // the screen hunts (its centre is not also held to the line)
            | CKind::SphereTangentLine
            | CKind::SphereTangentSphere
            // and the cones' and cylinders': distances and angles in space, regular at a contact
            | CKind::ConeOn
            | CKind::CylinderOn
            | CKind::ConeAngle
            | CKind::CylinderRadius
            | CKind::CylinderTangentLine
            | CKind::ConeTangentCone
            // a view turned from another, and the projector rule in space: algebra, no contact
            | CKind::Hinge
            | CKind::HingeParallel
            | CKind::HingeAlong
            | CKind::ProjectSolved
            | CKind::Mate
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
                | CKind::Coincident3
                | CKind::Distance3
                | CKind::Angle3
                | CKind::Perpendicular3
                | CKind::EqualLength3
                | CKind::SphereTangentSphere
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
            CKind::PointOnCurve | CKind::PointOnExtrusion | CKind::CurveTangentLine | CKind::CurveCurvature => {
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
            CKind::EqualAngle => K::EqualAngle,
            CKind::ArcLength => K::ArcLength,
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
            CKind::FrameUnit => K::FrameUnit,
            CKind::FrameAlign => K::FrameAlign,
            CKind::Project => K::Project,
            CKind::CoordinateU => K::CoordinateU,
            CKind::CoordinateV => K::CoordinateV,
            CKind::QuatUnit => K::QuatUnit,
            CKind::Lift => K::Lift,
            CKind::LiftFixed => K::LiftFixed,
            CKind::Coincident3 => K::Coincident3,
            CKind::Distance3 => K::Distance3,
            CKind::PointLine3 => K::PointLine3,
            CKind::LineLine3 => K::LineLine3,
            CKind::Angle3 => K::Angle3,
            CKind::Perpendicular3 => K::Perpendicular3,
            CKind::Parallel3 => K::Parallel3,
            CKind::PointOnPlane => K::PointOnPlane,
            CKind::PointOnPlaneFixed => K::PointOnPlaneFixed,
            CKind::PointOnCircle3 => K::PointOnCircle3,
            CKind::PointOnCircle3Fixed => K::PointOnCircle3Fixed,
            CKind::PointOnLine3 => K::PointOnLine3,
            CKind::EqualLength3 => K::EqualLength3,
            CKind::PointPlaneDistance => K::PointPlaneDistance,
            // a stated plane stood off by the number: the incidence kernel, D folded into h
            CKind::PointPlaneDistanceFixed => K::PointOnPlaneFixed,
            CKind::SphereOn => K::SphereOn,
            CKind::CircleOnSphere => K::CircleOnSphere,
            CKind::CircleOnSphereFixed => K::CircleOnSphereFixed,
            CKind::Midpoint3 => K::Midpoint3,
            CKind::Symmetric3 => K::Symmetric3,
            CKind::LineOnPlane => K::LineOnPlane,
            CKind::LineOnPlaneFixed => K::LineOnPlaneFixed,
            CKind::SphereRadius => K::Radius,
            // the line's distance from the centre, with the radius its free column: the free
            // twin of `point_line3` at (m, c) = (1, 0)
            CKind::SphereTangentLine => K::PointLine3Free,
            CKind::SphereTangentSphere => K::SphereSphere,
            CKind::ConeOn => K::ConeOn,
            // the point's distance from the axis, stated as the radius column
            CKind::CylinderOn => K::PointLine3Free,
            CKind::ConeAngle => K::HalfAngle,
            CKind::CylinderRadius => K::Radius,
            // the common perpendicular with the axis, stated as the radius column turned to the
            // seed's side
            CKind::CylinderTangentLine => K::LineLine3Free,
            CKind::ConeTangentCone => K::ConeCone,
            // a stood-off plane is a hinge whose turn is the identity
            CKind::Hinge | CKind::HingeParallel => K::Hinge,
            CKind::HingeAlong => K::HingeAlong,
            CKind::ProjectSolved => K::ProjectFree,
            CKind::Mate => K::Mate,
            CKind::Fix | CKind::Ccw | CKind::Cw => {
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
    /// The kernel for this type when the statement names **no side**: the magnitude form, whose
    /// solution set is both sides and whose branch the seed picks (issue #48, item 4).  `None`
    /// for every type that has no side to name — asked only through `side_slot`, so a type
    /// without one is never asked.
    pub fn magnitude_kernel(self, free: bool) -> Option<K> {
        Some(match (self, free) {
            (CKind::PointLineDistance, false) => K::PointLineMagnitude,
            (CKind::PointLineDistance, true) => K::PointLineMagnitudeFree,
            (CKind::ParallelDistance, false) => K::ParallelMagnitude,
            (CKind::ParallelDistance, true) => K::ParallelMagnitudeFree,
            _ => return None,
        })
    }

    /// The slot that says which side of a line the number is measured to, for the types that
    /// have one — the *word* whose absence makes the number a magnitude.  One table, asked by the
    /// kernel choice, by `consts_on`'s sign and by the elaborator's refusal of a negative.
    pub fn side_slot(self) -> Option<usize> {
        self.side_words().map(|(i, _)| i)
    }

    pub fn free_kernel(self) -> Option<K> {
        Some(match self {
            CKind::Distance => K::DistanceFree,
            CKind::Angle => K::AngleFree,
            CKind::Radius => K::RadiusFree,
            CKind::ArcLength => K::ArcLengthFree,
            CKind::ParallelDistance => K::ParallelDistanceFree,
            CKind::PointLineDistance => K::PointLineDistanceFree,
            CKind::AnnularDistance => K::AnnularDistanceFree,
            CKind::HorizontalDistance => K::HorizontalDistanceFree,
            CKind::VerticalDistance => K::VerticalDistanceFree,
            CKind::CoordinateU => K::CoordinateUFree,
            CKind::CoordinateV => K::CoordinateVFree,
            CKind::Distance3 => K::Distance3Free,
            CKind::PointLine3 => K::PointLine3Free,
            CKind::LineLine3 => K::LineLine3Free,
            CKind::Angle3 => K::Angle3Free,
            CKind::PointPlaneDistance => K::PointPlaneDistanceFree,
            CKind::PointPlaneDistanceFixed => K::PointPlaneDistanceFixedFree,
            CKind::SphereRadius => K::RadiusFree,
            CKind::CylinderRadius => K::RadiusFree,
            CKind::ConeAngle => K::HalfAngleFree,
            // `fold: beta`: the fold is the document's free variable
            CKind::Hinge => K::HingeFree,
            CKind::Coincident
            | CKind::Midpoint
            | CKind::DragTarget
            | CKind::Horizontal
            | CKind::Vertical
            | CKind::Parallel
            | CKind::Perpendicular
            // two angles equal states no number: there is nothing for a free variable to stand in
            | CKind::EqualAngle
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
            | CKind::PointOnExtrusion
            | CKind::CurveTangentLine
            | CKind::CurveCurvature
            | CKind::SplineTangentLine
            | CKind::SplineCurvature
            | CKind::HorizontalPoints
            | CKind::VerticalPoints
            | CKind::FrameUnit
            | CKind::FrameAlign
            | CKind::Project
            | CKind::QuatUnit
            | CKind::Lift
            | CKind::LiftFixed
            | CKind::Coincident3
            | CKind::Perpendicular3
            | CKind::Parallel3
            | CKind::PointOnPlane
            | CKind::PointOnPlaneFixed
            | CKind::PointOnCircle3
            | CKind::PointOnCircle3Fixed
            | CKind::PointOnLine3
            | CKind::EqualLength3
            | CKind::SphereOn
            | CKind::SphereTangentLine
            | CKind::SphereTangentSphere
            | CKind::LineOnPlane
            | CKind::LineOnPlaneFixed
            | CKind::CircleOnSphere
            | CKind::CircleOnSphereFixed
            | CKind::Midpoint3
            | CKind::Symmetric3
            | CKind::ConeOn
            | CKind::CylinderOn
            | CKind::CylinderTangentLine
            | CKind::ConeTangentCone
            | CKind::HingeParallel
            | CKind::HingeAlong
            | CKind::ProjectSolved
            | CKind::Mate
            | CKind::Fix
            | CKind::Ccw
            | CKind::Cw => return None,
        })
    }

    /// A view's **hinge** to the one it is folded from: the plane's own statement, stated by
    /// the elaborator from its brackets and never written as a relation — so it has no word, no
    /// figure and no line of its own in a lifted program.
    pub fn hinge(self) -> bool {
        matches!(self, CKind::Hinge | CKind::HingeParallel | CKind::HingeAlong)
    }

    /// A relation **in space**: one whose kernel reads the hidden points views lift to, so
    /// `Sketch::add` mints them and `validate` refuses an operand on no view.
    pub fn spatial(self) -> bool {
        matches!(
            self,
            CKind::Coincident3
                | CKind::Distance3
                | CKind::PointLine3
                | CKind::LineLine3
                | CKind::Angle3
                | CKind::Perpendicular3
                | CKind::Parallel3
                | CKind::PointOnPlane
                | CKind::PointOnPlaneFixed
                | CKind::PointOnCircle3
                | CKind::PointOnCircle3Fixed
                | CKind::ProjectSolved
                | CKind::PointOnLine3
                | CKind::EqualLength3
                | CKind::PointPlaneDistance
                | CKind::PointPlaneDistanceFixed
                | CKind::SphereOn
                | CKind::SphereTangentLine
                | CKind::SphereTangentSphere
                | CKind::LineOnPlane
                | CKind::LineOnPlaneFixed
                | CKind::CircleOnSphere
                | CKind::CircleOnSphereFixed
                | CKind::Midpoint3
                | CKind::Symmetric3
                | CKind::ConeOn
                | CKind::CylinderOn
                | CKind::CylinderTangentLine
                | CKind::ConeTangentCone
                | CKind::PointOnExtrusion
        )
    }

    /// The form of a statement that reads a view's attitude, for a view that is solved (`att`
    /// is `Some`) or stated: the one table of the pairs, asked by `Sketch::add` when a statement
    /// arrives and by `Sketch::free_attitude` when its view is freed, so a caller names either
    /// and the kernel is always the one the view can feed.  Every other kind is itself.
    pub fn attitude_twin(self, solved: bool) -> CKind {
        match ATTITUDE_TWINS.iter().find(|(s, f)| *s == self || *f == self) {
            Some(&(s, f)) => if solved { s } else { f },
            None => self,
        }
    }
}

/// The pairs `CKind::attitude_twin` reads: each statement over a solved view, and the same
/// statement over a stated one.
const ATTITUDE_TWINS: [(CKind, CKind); 7] = [
    (CKind::Lift, CKind::LiftFixed),
    (CKind::PointOnPlane, CKind::PointOnPlaneFixed),
    (CKind::LineOnPlane, CKind::LineOnPlaneFixed),
    (CKind::PointPlaneDistance, CKind::PointPlaneDistanceFixed),
    (CKind::PointOnCircle3, CKind::PointOnCircle3Fixed),
    (CKind::CircleOnSphere, CKind::CircleOnSphereFixed),
    (CKind::ProjectSolved, CKind::Project),
];

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
    /// Its dimension **as the statement wrote it**, where that is not the text the number is
    /// worked out from: `l1 distance(w) r2` under `w := 100` reaches the sketch as `100`,
    /// since the flattener settles a `param` to its number, and the callout draws `w`.  Set by
    /// the elaborator where the statement is the document's own text — at the root, or in a
    /// component body written in this file, whose formula over its formals is true of every
    /// instance (`program::relations::written`); a module's body and a block's copies draw the
    /// number.  Presentation, like `class`: nothing that solves reads it, and writing a number
    /// (`set_num`, `expr::set_dimension`) drops it.
    pub written: Option<String>,
    /// A dimension **another copy of its block already states**: `repeat n { radius(hole_r)
    /// circle hole … }` is one statement, and six holes are drawn with one callout and not six.
    /// Set by the elaborator (`program::relations::repeated`) on every copy after the first
    /// that draws the same label; the full callout layout leaves it out, and asking for it by id
    /// (editing that one) still draws it.  Presentation, like `class`.
    pub repeated: bool,
}

/// `+1` and `−1` as the words a statement writes them with — the one place the two meet, read by
/// the constructor that infers a tangency's side from the geometry and by the document reader
/// that finds a number where a word now stands (issue #48, item 4).
pub fn side_word(sign: i64) -> &'static str {
    if sign < 0 {
        "right"
    } else {
        "left"
    }
}

impl Constraint {
    /// Whether this constraint *acts* on the drawing — the one predicate behind "everything that
    /// must be satisfied".  A soft one is a transient the solve may miss; a claim is a question
    /// about the drawing rather than part of it, and neither is something a consumer asking for
    /// the constraints that determine the figure wants back.
    pub fn acts(&self) -> bool {
        !self.soft && !self.claim
    }

    /// A constraint of `kind` over the arguments given, **and the defaults for any the caller
    /// stopped short of**.
    ///
    /// One rule, in the one place every builder goes through: a slot appended to a type — a
    /// selector saying which side a magnitude is on (issue #48, item 4) — is a slot no existing
    /// caller knew to fill, and its default is what the type already meant without it.  A caller
    /// that gives *more* than the type has is a mistake, and still one.
    pub fn new(kind: CKind, args: Vec<Arg>) -> Constraint {
        let spec = kind.spec();
        debug_assert!(args.len() <= spec.len(), "{:?} arity", kind);
        let mut args = args;
        for i in args.len()..spec.len() {
            args.push(kind.default_arg(i));
        }
        Constraint {
            id: 0,
            kind,
            args,
            soft: false,
            intrinsic: false,
            claim: false,
            free: None,
            class: Default::default(),
            written: None,
            repeated: false,
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
            vec![Arg::Ent(line), Arg::Ent(circle), Arg::Str(side_word(s).to_string())],
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
            let d = (ax - bx).dhypot(ay - by);
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

    /// A relation in space over drawn entities: the number where the kind states one, and what
    /// the core reads off the geometry (a skew distance's side) filled in and checked through
    /// `io::seed_omitted`, the one rule a document and a Rust caller are refused by.  The hidden
    /// points are minted when the constraint is added.
    pub fn in_space(sk: &Sketch, kind: CKind, ents: &[EntRef], value: Option<f64>)
        -> Result<Constraint, String>
    {
        let spec = kind.spec();
        let mut ents = ents.iter();
        let mut args: Vec<Arg> = Vec::with_capacity(spec.len());
        for (i, (_, k)) in spec.iter().enumerate() {
            args.push(match k {
                k if k.is_entity() => Arg::Ent(*ents.next().ok_or("too few operands")?),
                k if k.is_dimension() => Arg::Num(value.ok_or("this relation states a number")?),
                _ => kind.default_arg(i),
            });
        }
        crate::io::seed_omitted(sk, kind, &mut args, |i| kind.infers_arg(i))?;
        Ok(Constraint::new(kind, args))
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
        let free = self.free.is_some();
        // a side left unsaid is the magnitude form: both sides are solutions, and where the solve
        // lands among them is the seed's business (§9.2)
        if self.side().is_none() {
            if let Some(k) = self.kind.magnitude_kernel(free) {
                return k;
            }
        }
        match free {
            true => self.kind.free_kernel().expect("a dimension has a free kernel"),
            false => self.kind.kernel(),
        }
    }

    /// The number a *cluster* has to place this dimension at: what it states, signed.
    ///
    /// The cluster vocabulary is signed where the language is not: a PL edge places a point at
    /// `n·p − c = v`, so it needs to know which side of the line to put it on.  Where the
    /// statement pins a side, that is the answer; where it does not, both sides are solutions and
    /// the one the drawing is *on* is the answer, because a plan moves a figure that already
    /// satisfies its constraints rather than choosing among their solutions (§9.2).  Every other
    /// type states its own number and reads it straight.
    pub fn signed_gap(&self, sk: &Sketch) -> f64 {
        let d = self.args[2].num();
        let Some(_) = self.kind.side_slot() else { return d };
        if let Some(s) = self.side() {
            return s * d;
        }
        // the same reading each kernel takes: a point against its line, and for a pair of lines
        // the second's first endpoint against the first
        let (p, line) = match self.kind {
            CKind::PointLineDistance => (self.args[0].ent().i(), self.args[1].ent().i()),
            _ => (sk.lines[self.args[1].ent().i()].p1 as usize, self.args[0].ent().i()),
        };
        let (px, py) = sk.point_xy(p);
        let now = crate::model::signed_point_to_line(sk, px, py, line);
        if now < 0.0 {
            -d
        } else {
            d
        }
    }

    /// Which way an angle turns: `sense: cw` is the minus a drawing no longer writes (§9.4), and
    /// an angle with no sense written turns the way the language counts, counter-clockwise.
    pub fn sense(&self) -> f64 {
        self.side().unwrap_or(1.0)
    }

    /// Which side the statement named, as the sign the signed kernel wants: `None` where it named
    /// none, and where the type has no side to name.
    pub fn side(&self) -> Option<f64> {
        let (i, table) = self.kind.side_words()?;
        let Arg::Str(w) = &self.args[i] else { return None };
        table.iter().find(|(n, _)| n == w).map(|(_, s)| *s)
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
        self.written = None;
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
            // a skew distance compares a signed gap against the number turned to the seed's
            // side, and turning (m, c) with it keeps the twin's kernel sign-blind; and a word
            // that says which way — `side:`, `along:`, `sense:` — turns them as it turns a
            // stated number below, or `side: right` over a free `k` meant the left
            let s = match self.kind {
                CKind::LineLine3 => self.skew_sign(),
                _ => self.side().unwrap_or(1.0),
            };
            // a stated plane's distance keeps its normal and offset beside the map
            if self.kind == CKind::PointPlaneDistanceFixed {
                return [stated_plane(sk, self.args[1].ent().i()).to_vec(), vec![f.m, f.c]].concat();
            }
            return vec![s * f.m, s * f.c];
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
            // a point in space reads the curve's view first: its datum on the sheet and its basis
            let view = match self.kind {
                CKind::PointOnExtrusion => sk.extrusion_frame(curve.i()).to_vec(),
                _ => Vec::new(),
            };
            return [view, match &d.body {
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
                // a generated profile carries the roll its root is chosen at, then the
                // numbers its motion was given, then its tool and motion
                crate::model::CurveBody::Envelope(g) => {
                    let mut k = Vec::with_capacity(2 + cv.values.len() + g.flat.len());
                    k.push(sk.curve_home(curve.i()));
                    k.push(cv.values.len() as f64);
                    k.extend_from_slice(&cv.values);
                    k.extend_from_slice(&g.flat);
                    k
                }
            }].concat();
        }
        match self.kind {
            CKind::Distance | CKind::CoordinateU | CKind::CoordinateV => vec![self.args[2].num()],
            // signed from the first point to the second, and which way is the word: `along: left`
            // is the minus a drawing used to write (§9.2)
            CKind::HorizontalDistance | CKind::VerticalDistance => {
                vec![self.side().unwrap_or(1.0) * self.args[2].num()]
            }
            CKind::DragTarget => {
                vec![self.args[1].num(), self.args[2].num(), self.args[3].num()]
            }
            // the number is a magnitude and the word is its sign: `side: right` is the same
            // statement as the negative used to be, said in a word a reader can check (§9.2)
            CKind::ParallelDistance | CKind::PointLineDistance => {
                vec![self.side().unwrap_or(1.0) * self.args[2].num()]
            }
            // an angle is directed, so `sense: cw` turns the number it states rather than the
            // reader having to write the minus (§9.4)
            CKind::Angle => vec![self.sense() * self.args[2].num()],
            // the sign the second pair's angle is read with: `sense: cw` is its mirror image
            CKind::EqualAngle => vec![self.sense()],
            CKind::ArcLength => vec![self.args[1].num()],
            CKind::AnnularDistance => vec![self.args[2].num()],
            CKind::Radius => vec![self.args[1].num()],
            // the word times the radius: the centre stands off the line on the side it names
            CKind::TangentLineCircle => vec![self.side().unwrap_or(1.0)],
            CKind::TangentCircleCircle => {
                vec![if matches!(self.args[2], Arg::Bool(true)) { 1.0 } else { -1.0 }]
            }
            // the fold line in each plane's own coordinates — validated at the add, so a pair
            // with none cannot be here; recomputed with every refresh, being a cross product
            // and four dots, rather than kept in a second skip set beside the curve contacts
            CKind::Project => {
                let basis = |i: usize| sk.basis(self.args[i].ent().i());
                let (da, db) = crate::plane::fold_line(&basis(2), &basis(3))
                    .expect("a projection between parallel planes is refused at the add");
                vec![da[0], da[1], db[0], db[1]]
            }
            // the fold's turn in the parent's axes, or none at all for a plane stood off it
            CKind::Hinge => crate::plane::fold_rotor(self.args[2].num()).to_vec(),
            CKind::HingeParallel => vec![1.0, 0.0, 0.0, 0.0],
            // the in-plane part of the solved view's origin, a constant of its mint
            CKind::Lift => sk.planes[self.args[1].ent().i()].att.as_ref().expect("a solved view")
                .ab.to_vec(),
            // the stated basis: u, v, o
            CKind::LiftFixed => {
                let b = sk.basis(self.args[1].ent().i());
                [b.u, b.v, b.o].concat()
            }
            CKind::Distance3 | CKind::PointLine3 => vec![self.args[2].num()],
            // the magnitude turned to the side the lines stood on when it was stated
            CKind::LineLine3 => vec![self.skew_sign() * self.args[2].num()],
            CKind::Angle3 => vec![self.args[2].num()],
            // two directions across a line as its hidden points stand now — the first line of a
            // parallel, the line a point is on — refreshed with every `refresh_consts`, so they
            // follow the line between solves
            CKind::Parallel3 | CKind::PointOnLine3 => {
                let i = if self.kind == CKind::Parallel3 { 0 } else { 1 };
                let l = &sk.lines[self.args[i].ent().i()];
                let a = crate::space::sub(sk.lifted(l.p2 as usize), sk.lifted(l.p1 as usize));
                let (e1, e2) = across(a);
                [e1, e2].concat()
            }
            CKind::PointOnPlaneFixed | CKind::LineOnPlaneFixed => {
                stated_plane(sk, self.args[1].ent().i()).to_vec()
            }
            CKind::PointPlaneDistance => vec![self.args[2].num()],
            // the stated plane moved along its own normal by the number
            CKind::PointPlaneDistanceFixed => {
                let mut k = stated_plane(sk, self.args[1].ent().i());
                k[3] += self.args[2].num();
                k.to_vec()
            }
            CKind::SphereRadius => vec![self.args[1].num()],
            // `point_line3_free` at (m, c) = (1, 0): the stated number is the radius column
            CKind::SphereTangentLine | CKind::CylinderOn => vec![1.0, 0.0],
            CKind::ConeAngle | CKind::CylinderRadius => vec![self.args[1].num()],
            // the gap between the two faces' ordinates along the planes' shared normal
            CKind::Mate => vec![self.args[2].num()],
            // `line_line3_free` at (m, c) = (±1, 0): the radius, turned to the seed's side
            CKind::CylinderTangentLine => vec![self.skew_sign(), 0.0],
            // outside, or inside with the larger radius positive as the radii stand now
            CKind::SphereTangentSphere => {
                if matches!(self.args[2], Arg::Bool(false)) {
                    let r = |i: usize| sk.radius_value(self.args[i].ent());
                    if r(0) >= r(1) { vec![1.0, -1.0] } else { vec![-1.0, 1.0] }
                } else {
                    vec![1.0, 1.0]
                }
            }
            // the normal of the view the circle is drawn in
            CKind::PointOnCircle3Fixed => {
                let v = self.attitude_read(sk).expect("a circle in space is drawn in a view");
                sk.basis(v).normal().to_vec()
            }
            // the in-plane axes of the view the circle is drawn in
            CKind::CircleOnSphereFixed => {
                let v = self.attitude_read(sk).expect("a circle in space is drawn in a view");
                let b = sk.basis(v);
                [b.u, b.v].concat()
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
        let (e, t) = self.kind.contact_slots()?;
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
        let pt_at = |p: u32| sk.point_params(p as usize).to_vec();
        let ln = |i: usize| sk.line_params(e(i).i()).to_vec();
        let centre = |i: usize| sk.point_params(sk.round_center(e(i))).to_vec();
        let rad = |i: usize| sk.round_radius(e(i)) as u32;
        let att_q = |i: usize| {
            sk.planes[e(i).i()].att.as_ref().expect("a hinge or a solved projection reads a solved view")
                .q.to_vec()
        };
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
            CKind::EqualAngle => [ln(0), ln(1), ln(2), ln(3)].concat(),
            // the centre, the two ends and the radius: the sweep is read off the ends, the
            // length is the radius times it
            CKind::ArcLength => {
                let a = &sk.arcs[e(0).i()];
                [pt_at(a.center), pt_at(a.start), pt_at(a.end), vec![rad(0)]].concat()
            }
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
            // the point's lift, then the parameter and the curve's coordinates as the contact's
            CKind::PointOnExtrusion => {
                [self.lifted_columns(sk), vec![self.args[2].param()], sk.entity_params(e(1))].concat()
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
            // the rotor alone; and the chord's length, then the frame's six numbers in
            // `entity_params` order — the kernels' column layouts exactly
            CKind::CoordinateU | CKind::CoordinateV => {
                let f = sk.frame_of(e(1));
                [pt(0), sk.point_params(f.origin as usize).to_vec(), vec![f.c, f.s]].concat()
            }
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
            CKind::QuatUnit => sk.planes[e(0).i()].att.as_ref().expect("a solved view").q.to_vec(),
            // the child's quaternion and its parent's; along a line, the fold's own rotor, the
            // parent datum's rotor and the line's ends after them
            CKind::Hinge | CKind::HingeParallel => [att_q(0), att_q(1)].concat(),
            CKind::HingeAlong => {
                let f = sk.frame_of(e(1));
                [att_q(0), att_q(1), vec![self.args[3].param(), self.args[4].param(), f.c, f.s],
                 ln(2)].concat()
            }
            // both images' hidden points, then both views' quaternions
            CKind::ProjectSolved => [self.lifted_columns(sk), att_q(2), att_q(3)].concat(),
            // the two planes' offsets along their shared normal
            CKind::Mate => {
                let d = |i: usize| {
                    sk.planes[e(i).i()].att.as_ref().expect("a mate between solved views").d
                };
                vec![d(0), d(1)]
            }
            // the hidden point, the view point, its datum's origin and rotor — and, over a
            // solved view, its quaternion and offset: the kernels' 14 and 9 columns
            CKind::Lift | CKind::LiftFixed => {
                let l = &sk.lifts[sk.lift_of(e(0).i()).expect("a lift's point has one")];
                let f = sk.frame_of(e(1));
                let origin = sk.point_params(f.origin as usize).to_vec();
                let mut ps = [l.x.to_vec(), pt(0), origin, vec![f.c, f.s]].concat();
                if self.kind == CKind::Lift {
                    let a = sk.planes[e(1).i()].att.as_ref().expect("a solved view");
                    ps.extend(a.q);
                    ps.push(a.d);
                }
                ps
            }
            // the hidden points, three columns each, in the order the operands name them
            CKind::Coincident3
            | CKind::Distance3
            | CKind::PointLine3
            | CKind::LineLine3
            | CKind::Angle3
            | CKind::Perpendicular3
            | CKind::Parallel3
            | CKind::PointOnPlaneFixed
            | CKind::PointOnLine3
            | CKind::EqualLength3
            | CKind::PointPlaneDistanceFixed
            | CKind::LineOnPlaneFixed
            | CKind::Midpoint3
            | CKind::Symmetric3 => self.lifted_columns(sk),
            // and a solved plane's quaternion and offset after them
            CKind::PointOnPlane | CKind::PointPlaneDistance | CKind::LineOnPlane => {
                let a = sk.planes[e(1).i()].att.as_ref().expect("a solved view");
                [self.lifted_columns(sk), a.q.to_vec(), vec![a.d]].concat()
            }
            // the hidden points, and then the radii the kernel reads as columns
            CKind::SphereOn => [self.lifted_columns(sk), vec![rad(1)]].concat(),
            // the circle's centre and the sphere's in space, both radii, and the circle's view's
            // quaternion where it is solved
            CKind::CircleOnSphere | CKind::CircleOnSphereFixed => {
                let mut out = [self.lifted_columns(sk), vec![rad(0), rad(1)]].concat();
                if self.kind == CKind::CircleOnSphere {
                    let v = self.attitude_read(sk).expect("a circle in space is drawn in a view");
                    out.extend(sk.planes[v].att.as_ref().expect("a solved view").q);
                }
                out
            }
            CKind::SphereRadius => vec![rad(0)],
            CKind::SphereTangentLine => [self.lifted_columns(sk), vec![rad(0)]].concat(),
            CKind::SphereTangentSphere => [self.lifted_columns(sk), vec![rad(0), rad(1)]].concat(),
            // the hidden points, then the number each cone or cylinder owns: its half-angle or
            // its radius, after its axis's two ends
            CKind::ConeOn | CKind::CylinderOn => {
                [self.lifted_columns(sk), vec![sk.axial(e(1)).param]].concat()
            }
            CKind::ConeAngle | CKind::CylinderRadius => vec![sk.axial(e(0)).param],
            CKind::CylinderTangentLine => {
                [self.lifted_columns(sk), vec![sk.axial(e(0)).param]].concat()
            }
            CKind::ConeTangentCone => {
                let x = self.lifted_columns(sk);
                let (k1, k2) = (sk.axial(e(0)).param, sk.axial(e(1)).param);
                [x[0..9].to_vec(), vec![k1], x[9..15].to_vec(), vec![k2]].concat()
            }
            // the point and the centre in space, the radius, and the view's quaternion
            CKind::PointOnCircle3 | CKind::PointOnCircle3Fixed => {
                let ps = self.lifted_columns(sk);
                let mut out = [ps, vec![rad(1)]].concat();
                if self.kind == CKind::PointOnCircle3 {
                    let v = self.attitude_read(sk).expect("a circle in space is drawn in a view");
                    out.extend(sk.planes[v].att.as_ref().expect("a solved view").q);
                }
                out
            }
            CKind::Fix | CKind::Ccw | CKind::Cw => {
                unreachable!("{:?} is a gauge and is never in a sketch", self.kind)
            }
        }
    }

    /// The drawn points whose hidden points a relation in space reads, in the order its kernel's
    /// columns take them: a point is itself, a line its two ends, a circle its centre.  Empty for
    /// every kind that reads none.  What `Sketch::add` lifts, and what `validate` asks to be on a
    /// view.
    pub fn lifted_points(&self, sk: &Sketch) -> Vec<usize> {
        if !self.kind.spatial() {
            return Vec::new();
        }
        let e = |i: usize| self.args[i].ent();
        let ends = |i: usize| {
            let l = &sk.lines[e(i).i()];
            [l.p1 as usize, l.p2 as usize]
        };
        let axis = |i: usize| {
            let l = &sk.lines[sk.axial(e(i)).axis as usize];
            [l.p1 as usize, l.p2 as usize]
        };
        match self.kind {
            CKind::Coincident3 | CKind::Distance3 | CKind::ProjectSolved => {
                vec![e(0).i(), e(1).i()]
            }
            CKind::PointLine3 | CKind::PointOnLine3 | CKind::Midpoint3 => {
                [vec![e(0).i()], ends(1).to_vec()].concat()
            }
            CKind::PointOnPlane
            | CKind::PointOnPlaneFixed
            | CKind::PointPlaneDistance
            | CKind::PointPlaneDistanceFixed => vec![e(0).i()],
            CKind::LineOnPlane | CKind::LineOnPlaneFixed => ends(0).to_vec(),
            CKind::PointOnCircle3 | CKind::PointOnCircle3Fixed | CKind::SphereOn => {
                vec![e(0).i(), sk.round_center(e(1))]
            }
            CKind::SphereTangentLine => [vec![sk.round_center(e(0))], ends(1).to_vec()].concat(),
            CKind::SphereTangentSphere | CKind::CircleOnSphere | CKind::CircleOnSphereFixed => {
                vec![sk.round_center(e(0)), sk.round_center(e(1))]
            }
            // a cone's or a cylinder's axis, apex first
            CKind::ConeOn | CKind::CylinderOn => [vec![e(0).i()], axis(1).to_vec()].concat(),
            CKind::PointOnExtrusion => vec![e(0).i()],
            CKind::CylinderTangentLine => [axis(0), ends(1)].concat(),
            CKind::ConeTangentCone => [vec![e(2).i()], axis(0).to_vec(), axis(1).to_vec()].concat(),
            CKind::Symmetric3 => [vec![e(0).i(), e(1).i()], ends(2).to_vec()].concat(),
            _ => [ends(0), ends(1)].concat(),
        }
    }

    /// Those hidden points' Params, three a point.
    fn lifted_columns(&self, sk: &Sketch) -> Vec<u32> {
        self.lifted_points(sk)
            .into_iter()
            .flat_map(|p| {
                let k = sk.lift_of(p).expect("a relation in space is lifted at the add");
                sk.lifts[k].x
            })
            .collect()
    }

    /// The plane whose attitude this statement's kernel reads, where it reads one: a lift's view,
    /// the plane a point is put on, the view a circle is drawn in.
    pub fn attitude_read(&self, sk: &Sketch) -> Option<usize> {
        match self.kind {
            CKind::Lift
            | CKind::LiftFixed
            | CKind::PointOnPlane
            | CKind::PointOnPlaneFixed
            | CKind::PointPlaneDistance
            | CKind::PointPlaneDistanceFixed
            | CKind::LineOnPlane
            | CKind::LineOnPlaneFixed => Some(self.args[1].ent().i()),
            CKind::PointOnCircle3 | CKind::PointOnCircle3Fixed => {
                sk.plane_of(sk.round_center(self.args[1].ent()))
            }
            CKind::CircleOnSphere | CKind::CircleOnSphereFixed => {
                sk.plane_of(sk.round_center(self.args[0].ent()))
            }
            _ => None,
        }
    }

    /// Every plane whose attitude decides which twin this statement is: `attitude_read`'s one,
    /// or a projection's two — which is solved when either of them is.
    pub fn attitudes_read(&self, sk: &Sketch) -> Vec<usize> {
        match self.kind {
            CKind::Project | CKind::ProjectSolved => {
                vec![self.args[2].ent().i(), self.args[3].ent().i()]
            }
            _ => self.attitude_read(sk).into_iter().collect(),
        }
    }

    /// The side a skew distance — or a line's tangency to a cylinder, which is one — was stated
    /// on, as the sign its kernel's gap carries there: the kind's `sign` slot.
    fn skew_sign(&self) -> f64 {
        let i = self.kind.spec().iter().position(|(n, _)| *n == "sign");
        match i.map(|i| &self.args[i]) {
            Some(Arg::Int(s)) if *s < 0 => -1.0,
            _ => 1.0,
        }
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
            let len = dx.dhypot(dy).max(kernels::MIN_LINE_LEN);
            sk.curve_nearest_by(args[0].ent().i(), |px, py| {
                ((px - ax) * dy - (py - ay) * dx).abs() / len
            })
        }
        // a point in space starts where the curve comes nearest its place in the curve's view
        (CKind::PointOnExtrusion, 2) => {
            let ci = args[1].ent().i();
            let k = sk.extrusion_frame(ci);
            let d = crate::space::sub(sk.world_point(args[0].ent().i()), [k[10], k[11], k[12]]);
            let (a, b) = (crate::space::dot(d, [k[4], k[5], k[6]]), crate::space::dot(d, [k[7], k[8], k[9]]));
            let (x, y) = crate::plane::on_page(k[2], k[3], (k[0], k[1]), (a, b));
            sk.curve_nearest_by(ci, |px, py| (px - x).dhypot(py - y))
        }
        (CKind::CurveCurvature, 2) => {
            let (cx, cy) = sk.point_xy(sk.round_center(args[1].ent()));
            sk.curve_nearest_by(args[0].ent().i(), |px, py| (px - cx).dhypot(py - cy))
        }
        // an osculating circle sits centred a radius off the curve, so the curve point nearest
        // the centre it already has is the place it is asking about
        (CKind::SplineCurvature, 2) => {
            let (cx, cy) = sk.point_xy(sk.round_center(args[1].ent()));
            crate::curve::closest(sk, args[0].ent().i(), cx, cy).0
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
        (CKind::Project | CKind::ProjectSolved, 2 | 3) => {
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
        // a curvature reads the curve's second derivative, which a trace gives exactly only
        // where every row of its block has a Taylor form (`taylor.rs`): a residual by difference
        // would solve to a slightly wrong circle and call it right
        CKind::CurveCurvature => {
            let cv = &sk.curves[args[0].ent().i()];
            if let crate::model::CurveBody::Trace(l) = &sk.curve_defs[cv.def as usize].body {
                if let Some(kernel) = l.without_form() {
                    return Err(format!(
                        "{} is traced through a `{kernel}`, whose second derivative is not \
                         written, so the curve has no curvature to state a circle against",
                        crate::io::entity_name(args[0].ent())
                    ));
                }
            }
            Ok(())
        }
        CKind::Project | CKind::ProjectSolved => {
            let (pa, pb) = (args[2].ent(), args[3].ent());
            if pa == pb {
                return Err(format!(
                    "both points are on {}, and one view relates nothing to itself",
                    crate::io::entity_name(pa)
                ));
            }
            // two stated views are parallel or not now and for ever; where either is solved,
            // whether they came out parallel is a question for after the solve (E065)
            let solved = |e: EntRef| sk.planes[e.i()].att.is_some();
            let basis = |e: EntRef| sk.basis(e.i());
            if !solved(pa) && !solved(pb)
                && crate::plane::fold_line(&basis(pa), &basis(pb)).is_none()
            {
                return Err(format!(
                    "{} and {} are parallel, so no fold line relates their views",
                    crate::io::entity_name(pa),
                    crate::io::entity_name(pb)
                ));
            }
            Ok(())
        }
        k if k.spatial() => {
            let c = Constraint::new(kind, args.to_vec());
            for p in c.lifted_points(sk) {
                if sk.plane_of(p).is_none() {
                    return Err(format!(
                        "{} is on no view, and a relation in space reads where a view puts it",
                        crate::io::entity_name(EntRef::point(p))
                    ));
                }
            }
            if let Some((i, _, _)) = c.dimensions().first() {
                if k.magnitude() && matches!(args[*i], Arg::Num(v) if v < 0.0) {
                    return Err("a distance in space is a magnitude, and is never negative".into());
                }
            }
            // a line in space needs a direction, and two of them a common perpendicular
            let mut lines: Vec<EntRef> =
                c.entities().into_iter().filter(|e| e.kind == EntKind::Line).collect();
            // a cone's or a cylinder's axis is a line in space too, and needs its direction
            for e in c.entities() {
                if matches!(e.kind, EntKind::Cone | EntKind::Cylinder) {
                    lines.insert(0, EntRef::line(sk.axial(e).axis as usize));
                }
            }
            for &l in &lines {
                let ln = &sk.lines[l.i()];
                let [a, b] = [ln.p1, ln.p2].map(|p| sk.world_point(p as usize));
                if crate::space::norm(crate::space::sub(b, a)) <= kernels::MIN_LINE_LEN {
                    return Err(format!("{} has no length in space", crate::io::entity_name(l)));
                }
            }
            // a view's own points are on it by construction: the row would be identically zero
            let own = |e: EntRef| {
                c.lifted_points(sk).iter().all(|&p| sk.plane_of(p) == Some(e.i()))
            };
            if matches!(k, CKind::PointOnPlane | CKind::PointOnPlaneFixed
                | CKind::PointPlaneDistance | CKind::PointPlaneDistanceFixed
                | CKind::LineOnPlane | CKind::LineOnPlaneFixed)
                && own(args[1].ent())
            {
                return Err(format!(
                    "{} is drawn in {}, so where it stands along that view's normal is not a \
                     question: every point of a view is on it",
                    crate::io::entity_name(args[0].ent()),
                    crate::io::entity_name(args[1].ent())
                ));
            }
            if matches!(k, CKind::LineLine3 | CKind::CylinderTangentLine)
                && skew_seed(sk, lines[0], lines[1]).is_none()
            {
                return Err(format!(
                    "{} and {} are parallel in space, and parallel lines have no common \
                     perpendicular to measure",
                    crate::io::entity_name(lines[0]),
                    crate::io::entity_name(lines[1])
                ));
            }
            Ok(())
        }
        _ => Ok(()),
    }
}

/// The side two lines stand on as drawn, as the sign of their common-perpendicular gap — `None`
/// where they are parallel and have none.  Read off the drawn pose (`Sketch::world_point`), which
/// is where the hidden points are seeded, so it is the same reading before and after the add.
fn skew_seed(sk: &Sketch, l1: EntRef, l2: EntRef) -> Option<f64> {
    use crate::space::{cross, dot, norm, sub};
    let ends = |l: EntRef| {
        let l = &sk.lines[l.i()];
        (sk.world_point(l.p1 as usize), sk.world_point(l.p2 as usize))
    };
    let ((a, b), (c, d)) = (ends(l1), ends(l2));
    let (e1, e2) = (sub(b, a), sub(d, c));
    let m = cross(e1, e2);
    if norm(m) <= crate::plane::PARALLEL_TOL * norm(e1) * norm(e2) {
        return None;
    }
    Some(if dot(m, sub(c, a)) < 0.0 { -1.0 } else { 1.0 })
}

/// A non-entity argument the core reads off the geometry, for the kinds whose omitted value is
/// the geometry's and not a constant: a skew distance's side.  `None` for every other slot, which
/// keeps what the caller or the defaults gave it.
pub fn infer_value(sk: &Sketch, kind: CKind, args: &[Arg], i: usize) -> Option<Arg> {
    match (kind, i) {
        (CKind::LineLine3, 3) => {
            skew_seed(sk, args[0].ent(), args[1].ent()).map(|s| Arg::Int(s as i64))
        }
        (CKind::CylinderTangentLine, 2) => {
            let axis = EntRef::line(sk.axial(args[0].ent()).axis as usize);
            skew_seed(sk, axis, args[1].ent()).map(|s| Arg::Int(s as i64))
        }
        // whichever of touching outside and inside the two spheres already stand nearer, as two
        // circles' is read on the page
        (CKind::SphereTangentSphere, 2) => {
            let c = |i: usize| sk.world_point(sk.round_center(args[i].ent()));
            let d = crate::space::norm(crate::space::sub(c(0), c(1)));
            let (r1, r2) = (sk.radius_value(args[0].ent()).abs(), sk.radius_value(args[1].ent()).abs());
            Some(Arg::Bool((d - (r1 + r2)).abs() <= (d - (r1 - r2).abs()).abs()))
        }
        _ => None,
    }
}

/// A stated plane's normal and its origin along it: the constants of every statement that puts a
/// point on a plane whose attitude is document data.
fn stated_plane(sk: &Sketch, p: usize) -> [f64; 4] {
    let b = sk.basis(p);
    let n = b.normal();
    [n[0], n[1], n[2], b.along_normal()]
}

/// Two unit vectors across a direction: perpendicular to it and to each other, with the second
/// `â × e₁`.  The first is taken against whichever page axis `a` is least along, so the cross
/// product is never small; a direction of no length gets the page's own pair.
fn across(a: [f64; 3]) -> ([f64; 3], [f64; 3]) {
    use crate::space::{cross, normalised};
    let Some(ah) = normalised(a) else { return ([1.0, 0.0, 0.0], [0.0, 1.0, 0.0]) };
    let k = (0..3).min_by(|&x, &y| ah[x].abs().total_cmp(&ah[y].abs())).unwrap_or(0);
    let mut axis = [0.0; 3];
    axis[k] = 1.0;
    let e1 = normalised(cross(ah, axis)).unwrap_or([1.0, 0.0, 0.0]);
    (e1, cross(ah, e1))
}

/// The world length one unit of a hidden unknown is worth — see `Param::scale`.  Read off the
/// geometry at the moment the constraint is added; it preconditions the step, so an estimate
/// that drifts as the sketch moves costs convergence rate, never correctness.
pub fn param_scale(sk: &Sketch, kind: CKind, args: &[Arg], i: usize) -> f64 {
    // Whichever family the contact runs along, the question is the same one, so it is asked once
    // and answered by the entity the slot actually names.  A hidden unknown that runs along
    // nothing is a length already.
    // a fold's half-angle rotor turns the view it folds as far as that view's own quaternion
    // does, so it is worth what one unit of that is (`Sketch::att_scale`)
    if kind == CKind::HingeAlong && (i == 3 || i == 4) {
        return sk.planes[args[0].ent().i()].att.as_ref()
            .map_or(1.0, |a| sk.params[a.q[0] as usize].scale);
    }
    match kind.contact_slots() {
        Some((e, t)) if t == i => contact_speed(sk, args[e].ent()),
        _ => 1.0,
    }
}

/// The world length one unit of a contact's parameter is worth, whichever family it runs along
/// — the one answer, so the seed `Sketch::add` records and the scale `System::new` compiles
/// against cannot come from two different rules.
pub fn contact_speed(sk: &Sketch, e: EntRef) -> f64 {
    crate::curve::speed(sk, e.i())
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
        SpecKind::Curve => ent == EntKind::Curve,
        SpecKind::Plane => ent == EntKind::Plane,
        SpecKind::Sphere => ent == EntKind::Sphere,
        SpecKind::Cone => ent == EntKind::Cone,
        SpecKind::Cylinder => ent == EntKind::Cylinder,
        _ => false,
    }
}
