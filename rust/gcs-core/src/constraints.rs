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
use crate::kernels::{self, Kernel, K};
use crate::model::{EntKind, EntRef, Sketch};
use crate::space::across;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum CKind {
    Coincident,
    Distance,
    Midpoint,
    DragTarget,
    /// **A point in space seen where the pointer is**: the soft target of a drag in the workspace,
    /// two rows over the point's three numbers, read along the eye's picture plane (`az`, `el`,
    /// radians, as `overview::eye` reads them).  Internal and `soft`, as `DragTarget` is.
    DragSeen,
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
    /// **A spline's whole length** (`length(L) s`, #121): `∫ |C'| dt` over its domain, by
    /// Gauss–Legendre per span (`integral.rs`).  Every control point is a column, so the kernel
    /// is built per count (`KernelKey::SplineLength`).  A magnitude, degree 1; a belt's, a
    /// cable's, a hanging rope's length.
    SplineLength,
    /// **A free curve's length** (`length(L) rope`, #144): the curve's own number, `L − d` over
    /// that one column (the radius kernel's), the interval its energy's equation is integrated
    /// over.  A magnitude, degree 1.
    CurveLength,
    /// **An energy's term** (`k minimizes …`, #121, #144): one integral along a free curve with a
    /// coefficient.  The terms over one curve are its energy, compiled into its definition
    /// (`Sketch::settle_variational`), whose shape is the solution of its Euler–Lagrange
    /// equation (`extremal.rs`).  It compiles no row, but the first of a curve's terms carries
    /// one where nothing holds the curve's length: transversality, `H = 0` at the end — the
    /// energy stationary in the length too.  Its kernel is built from the curve.
    Stationary,
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
    /// **An ordinate** (`docs/ordinate-plan.md`): how far `q` stands from `p` along a directed
    /// line, `(q − p)·t̂ − d`.  One statement whatever the direction is — a view's own axis (the
    /// run and the rise), a drawn line, an axis in space, a plane's normal — and which kernel reads
    /// it is the operands' business, held in its `form` slot (`OrdinateForm`), which the core
    /// fills and nobody writes.  Signed from `p` to `q`, along the direction's own sense; the
    /// page's words (`right`, `left`, `up`, `down`) are that sign said as a word.  The plane forms
    /// (`q distance(d, along: u) P`) are this from `P.origin` along `P.u`.
    Ordinate,
    /// **Two points level along a direction**: the ordinate's zero, `(q − p)·t̂ = 0`, so a
    /// relation with no number and no callout (`a level(t) b`).  `a horizontal b` is `a
    /// level(up) b`.  The same slots and kernels as `Ordinate`, at no distance.
    Level,
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
    /// Two points are images of one point in space, each on the plane it is `in`: their
    /// coordinates along the fold line the two planes share agree (`plane::fold_line`).  One
    /// row over both points, with the fold line as constants: both planes fixed (where either
    /// moves in the solve it is `ProjectSolved`).  The plane slots are **inferred** from the
    /// points' memberships at `io::seed_omitted`'s seam — the source and the bindings write two
    /// points — and refused when a point is on no plane, both are on one, or the planes are
    /// parallel.  Not commutative: `same_args` swaps only the first two entity slots, so
    /// `b project a` reads as a second relation, which the diagnosis reports as implied.
    Project,
    /// A hidden point in space held at the lift of the point it stands for, drawn in a plane:
    /// `X − (o + p.x·û + p.y·v̂) = 0` over the plane's axes and origin.  Three rows over the
    /// hidden point's three Params, so net nothing.  Intrinsic, minted by `Sketch::lift_point`.
    Lift,
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
    /// `P coincident p`: a point on a plane in space, over the plane's axes and origin; the plane
    /// is any, not only the point's own.
    PointOnPlane,
    /// A point on a circle drawn in a plane: on the sphere of its radius about the centre's lift,
    /// and on the centre's plane — two rows.
    PointOnCircle3,
    /// **The rest of the words in space**, inferred like the eleven above wherever a
    /// relation's operands are drawn in different views.  A point on a line's infinite extension:
    /// two rows across the line (`kernels::point_on_line3_res`), since the magnitude has no
    /// gradient where it vanishes.
    PointOnLine3,
    /// Two lines of equal true length.
    EqualLength3,
    /// `P coincident l`: a line on a plane in space, both its ends.
    LineOnPlane,
    /// A point the midpoint of a line drawn in another view, in space.
    Midpoint3,
    /// Two points each the other's image in a line, in space: the half turn about the line — the
    /// mirror in it, which is what `symmetry` means on a page, read one dimension up.
    Symmetric3,
    /// `project` where either plane moves in the solve: the projector rule in space over both
    /// images' hidden points and both planes' normals, `(n_A × n_B)·(X_A − X_B) = 0`
    /// (`kernels::project_solved_rows`).  The same statement as `Project` — the word, the
    /// operands and the inferred planes — and the twin `Sketch::add` picks when a plane it reads
    /// is not fixed (`Sketch::plane_fixed`).
    ProjectSolved,
    /// `t coincident P`: an axis lying in a plane — two points on it a drawing's extent apart, each
    /// on the plane (`kernels::axis_on_plane_rows`).  Reads where the axis is, so it places it.
    AxisOnPlane,
    /// Two axes on one line, either way round (`a coincident b`): the second along the first and
    /// its place on it (`kernels::axis_coincident_rows`).  Reads where both axes are.
    AxisCoincident,
    /// `t parallel P`: an axis square to a plane's normal, `d·n̂ = 0`.
    AxisParallelPlane,
    /// `t perpendicular P`: an axis along a plane's normal — two rows across the normal, as
    /// `parallel3` states two lines parallel.
    AxisPerpendicularPlane,
    /// `P parallel Q`: two planes facing alike, either way — their normals parallel, two rows
    /// across the first's, as `parallel3` states two lines parallel.  Says nothing of where
    /// either stands, or of how either is turned within itself.
    PlaneParallel,
    /// `P distance(d) Q`: how far `Q`'s origin stands along `P`'s normal from `P`'s origin —
    /// for parallel planes, the gap between them.  One row; that the two are parallel is said
    /// by their axes.
    PlaneDistance,
    /// **An axis's direction held to the unit sphere**: `|d|² = 1`, dimensionless, degree 0.
    /// Intrinsic, minted by `Sketch::axis` and nowhere else.
    AxisUnit,
    /// **A placed axis's point held at the foot of the perpendicular from the origin**: `a·d = 0`,
    /// so it cannot slide along the axis.  Intrinsic, minted by `Sketch::place_axis` once a
    /// relation reads where the axis is.
    AxisFoot,
    /// A plane's origin on one of its axes: two rows, minted with the plane (`Sketch::push_plane`)
    /// and nowhere else, never serialized — a plane's axes pass through its origin.  The
    /// `point_on_axis` kernel over the plane's `o` in the point's place.
    PlaneAxis,
    /// `p coincident t`: a point on an axis's line in space — `point_on_line3`'s two rows across
    /// the axis's direction, over the point's lift and the axis's place and direction.  What
    /// gives an axis its place.
    PointOnAxis,
    /// `l coincident t`: a line lying on an axis, either way round — both its ends on the axis's
    /// line in space (`kernels::line_on_axis_rows`), so it runs along it too.  Reads where the
    /// axis is, so it places it.
    LineOnAxis,
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
pub const ALL_KINDS: [CKind; 65] = [
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
    CKind::Ordinate,
    CKind::Level,
    CKind::PointOnCurve,
    CKind::PointOnExtrusion,
    CKind::CurveTangentLine,
    CKind::CurveCurvature,
    CKind::Project,
    CKind::Lift,
    CKind::Coincident3,
    CKind::Distance3,
    CKind::PointLine3,
    CKind::LineLine3,
    CKind::Angle3,
    CKind::Perpendicular3,
    CKind::Parallel3,
    CKind::PointOnPlane,
    CKind::PointOnCircle3,
    CKind::ProjectSolved,
    CKind::PointOnLine3,
    CKind::EqualLength3,
    CKind::LineOnPlane,
    CKind::Midpoint3,
    CKind::Symmetric3,
    CKind::EqualAngle,
    CKind::ArcLength,
    CKind::AxisUnit,
    CKind::AxisFoot,
    CKind::PlaneAxis,
    CKind::PointOnAxis,
    CKind::LineOnAxis,
    CKind::AxisOnPlane,
    CKind::AxisCoincident,
    CKind::AxisParallelPlane,
    CKind::AxisPerpendicularPlane,
    CKind::PlaneParallel,
    CKind::PlaneDistance,
    CKind::DragSeen,
    CKind::SplineLength,
    CKind::CurveLength,
    CKind::Stationary,
];

/// **The words a direction may be written as** (`docs/ordinate-plan.md`): `along:` an ordinate
/// is measured on, and what `level` keeps level.  A direction is otherwise a reference — an axis
/// or a drawn line — and these name the ones a drawing has without naming them: the axes of the
/// view both points are drawn in (`x`, `y`, and with the sign said, `right`, `left`, `up`,
/// `down`), or, against a plane, its own `u`, `v` and normal (`n`).  One table, read by the
/// parser (a word here is a word, never a name), by `infix_op`, by the elaborator's refusal and
/// by the inference of the direction itself (`infer_entity`); the signs are `side_words`'.
pub const ALONG: [(&str, Toward); 9] = [
    ("x", Toward::PageU),
    ("y", Toward::PageV),
    ("u", Toward::PlaneU),
    ("v", Toward::PlaneV),
    // a point's signed distance along a plane's normal, in space whatever views it is in
    ("n", Toward::PlaneN),
    // the view's axes with the direction named outright, which is the sign said in a word
    ("right", Toward::PageU),
    ("left", Toward::PageU),
    ("up", Toward::PageV),
    ("down", Toward::PageV),
];

/// The words alone, in `ALONG`'s order: what the registry publishes and `words` offers.
pub const ALONG_WORDS: [&str; ALONG.len()] = {
    let mut w = [""; ALONG.len()];
    let mut i = 0;
    while i < w.len() {
        w[i] = ALONG[i].0;
        i += 1;
    }
    w
};

/// What a direction word names: one of the axes of the view the points are drawn in, or one of
/// the plane's an ordinate is measured against.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Toward {
    PageU,
    PageV,
    PlaneU,
    PlaneV,
    PlaneN,
}

impl Toward {
    /// The word's meaning, or `None` for a word that is not a direction.
    pub fn of(word: &str) -> Option<Toward> {
        ALONG.iter().find(|(w, _)| *w == word).map(|(_, t)| *t)
    }

    /// Whether the word names a plane's own direction, so the second operand is that plane.
    pub fn of_plane(self) -> bool {
        matches!(self, Toward::PlaneU | Toward::PlaneV | Toward::PlaneN)
    }
}

/// **Which kernel reads an ordinate** — the one fact about an `Ordinate` or a `Level` the operands
/// decide rather than the statement: where its points are drawn and what its direction is.  Held
/// in the kind's `form` slot, which `Sketch::add_quiet` sets from `ordinate_form` whatever it was
/// handed, so it is never stale and never written by a person.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OrdinateForm {
    /// Both points in one view, along its `u`: the run, `q.x − p.x`, a constant Jacobian.
    PageU = 0,
    /// Along the view's `v`: the rise.
    PageV = 1,
    /// Both points and a drawn line in one view: along the line, in the view's coordinates.
    InView = 2,
    /// Anything else along an axis or a line: in space, over the points' lifts.
    Space = 3,
    /// A plane's own word, from its origin, for a point not drawn in it: along the plane's frame
    /// in space, its `û` — which is its `u` axis — its `v̂`, square to `û` in the plane whatever
    /// way the `v` axis runs, and its normal `n̂`, over the plane's origin and axes as columns.
    FrameU = 4,
    FrameV = 5,
    FrameN = 6,
    /// From a plane's origin to a point drawn in the plane: the point's own coordinate, `q.x − d`
    /// (`q.y` for `CoordV`) — `radius`'s kernel over the one column, the origin being the
    /// coordinates' zero, so the row reads nothing it does not move.
    CoordU = 7,
    CoordV = 8,
}

impl OrdinateForm {
    pub fn from_int(i: i64) -> OrdinateForm {
        match i {
            0 => OrdinateForm::PageU,
            1 => OrdinateForm::PageV,
            2 => OrdinateForm::InView,
            4 => OrdinateForm::FrameU,
            5 => OrdinateForm::FrameV,
            6 => OrdinateForm::FrameN,
            7 => OrdinateForm::CoordU,
            8 => OrdinateForm::CoordV,
            // 3, and anything a record made up
            _ => OrdinateForm::Space,
        }
    }

    /// Whether the kernel reads the points where they stand in space.
    pub fn in_space(self) -> bool {
        matches!(self, OrdinateForm::Space | OrdinateForm::FrameU | OrdinateForm::FrameV
            | OrdinateForm::FrameN)
    }

    /// Whether it is read over a plane's own frame (`FrameU`, `FrameV`, `FrameN`).
    pub fn of_frame(self) -> bool {
        matches!(self, OrdinateForm::FrameU | OrdinateForm::FrameV | OrdinateForm::FrameN)
    }
}

/// The form an ordinate's operands give it (`OrdinateForm`): `p`, `q`, the direction and the
/// word it was named by.  A view both points are drawn in reads its own `u` as the run and —
/// named so — its up as the rise, and a line drawn there in its coordinates; a plane's own word
/// from its origin reads the plane's frame; every other direction is read in space, along the
/// axis or the line itself.
pub fn ordinate_form(sk: &Sketch, p: usize, q: usize, along: EntRef, word: &str) -> OrdinateForm {
    let view = shared_view(sk, p, q);
    let toward = Toward::of(word);
    // the view's up, its frame's `v̂`: said as a word, or its `v` axis where the plane holds its
    // two axes square — an axis written outright is read along itself, which `v̂` may not be
    let square = |pl: usize| {
        let d = |a: u32| sk.axes[a as usize].d;
        let (du, dv) = (d(sk.planes[pl].u), d(sk.planes[pl].v));
        let held = du.iter().chain(&dv).all(|&k| sk.params[k as usize].fixed);
        let value = |p: [u32; 3]| p.map(|k| sk.params[k as usize].value);
        held && crate::space::dot(value(du), value(dv)).abs() <= 1e-12
    };
    let up = |pl: usize| matches!(toward, Some(Toward::PageV | Toward::PlaneV)) || square(pl);
    let axis = |pl: usize, u: bool| {
        let a = if u { sk.planes[pl].u } else { sk.planes[pl].v };
        along == EntRef::axis(a as usize)
    };
    if along.kind == EntKind::Plane {
        return OrdinateForm::FrameN;
    }
    if let Some(v) = view {
        // from the view's own origin, the point's coordinate in it
        let origin = sk.plane_of_origin(p) == Some(v);
        match along.kind {
            EntKind::Axis if axis(v, true) && origin => return OrdinateForm::CoordU,
            EntKind::Axis if axis(v, false) && up(v) && origin => return OrdinateForm::CoordV,
            EntKind::Axis if axis(v, true) => return OrdinateForm::PageU,
            EntKind::Axis if axis(v, false) && up(v) => return OrdinateForm::PageV,
            EntKind::Line => {
                let l = &sk.lines[along.i()];
                if [l.p1, l.p2].iter().all(|&e| sk.plane_of(e as usize) == Some(v)) {
                    return OrdinateForm::InView;
                }
            }
            _ => {}
        }
    }
    match (sk.plane_of_origin(p), toward) {
        (Some(pl), Some(Toward::PlaneU)) if axis(pl, true) => OrdinateForm::FrameU,
        (Some(pl), Some(Toward::PlaneV)) if axis(pl, false) => OrdinateForm::FrameV,
        _ => OrdinateForm::Space,
    }
}

/// The view points `p` and `q` are both drawn in, if they are.
pub fn shared_view(sk: &Sketch, p: usize, q: usize) -> Option<usize> {
    sk.plane_of(p).filter(|&v| sk.plane_of(q) == Some(v))
}

/// The word an ordinate's direction was named by — `x`, `up`, `u` … — or `""` where it was named
/// by reference, and for every other kind.
pub fn ordinate_word(kind: CKind, args: &[Arg]) -> &str {
    match kind.word_slot().and_then(|w| args.get(w)) {
        Some(Arg::Str(w)) => w,
        _ => "",
    }
}

/// What a written operator says, once its operands' kinds are known.
///
/// **The one table that turns a word and a pair of kinds into a constraint.**  It is the inverse
/// of `CKind::operator`, and it is a table rather than a search because several kinds share a
/// word and the operand kinds (and one selector) are what tell them apart: `coincident` is
/// seventeen kinds, `distance` fifteen, `tangent` ten.
///
/// Operand order carries meaning, and that is a change worth seeing: `arc tangent line` is
/// `TangentArcLine` and `line tangent circle` is `TangentLineCircle`.  Each named itself before
/// and the order was decoration; as an operator, which side the arc is written on picks the kind.
///
/// `sel` is what stood in the parentheses, by name — `along`, `at` — since two of the choices
/// cannot be made from the kinds alone.  `None` is "this word does not relate those two", which
/// the caller reports with the kinds in it.
pub fn infix_op(word: &str, a: EntKind, b: EntKind, sel: &dyn Fn(&str) -> Option<String>) -> Option<CKind> {
    use EntKind::{Arc, Circle, Curve, Line, Plane, Point, Axis, Spline};
    let round = |k: EntKind| matches!(k, Circle | Arc);
    // an axis is in space, so a direction relation naming one is the relation in space
    let axes = |a: EntKind, b: EntKind| matches!((a, b), (Axis, Axis | Line) | (Line, Axis));
    Some(match word {
        // incidence, and two points being one: what it means is the kinds of its operands
        "coincident" => match (a, b) {
            (Point, Point) => CKind::Coincident,
            (Point, Line) => CKind::PointOnLine,
            (Point, k) if round(k) => CKind::PointOnCircle,
            (Point, Spline) => CKind::PointOnSpline,
            (Point, Curve) => CKind::PointOnCurve,
            // in space whatever view the point is in: a plane is a place, not a picture
            (Point, Plane) => CKind::PointOnPlane,
            (Line, Plane) => CKind::LineOnPlane,
            (Point, Axis) => CKind::PointOnAxis,
            // a line lying on an axis
            (Line, Axis) => CKind::LineOnAxis,
            // an axis lying in a plane
            (Axis, Plane) => CKind::AxisOnPlane,
            // two axes on one line
            (Axis, Axis) => CKind::AxisCoincident,
            _ => return None,
        },
        "distance" => match (a, b) {
            // a pair of points apart, or — with `along:` — how far the second stands from the
            // first along a direction: an ordinate, signed, so it does not commute.  Against a
            // plane it is measured from the plane's origin (`along: u`, `v`, `n`); whether the
            // direction suits the operands is the elaborator's to say, in their words
            (Point, Point) => match sel("along") {
                None => CKind::Distance,
                Some(_) => CKind::Ordinate,
            },
            (Point, Plane) => match sel("along") {
                None => return None,
                Some(_) => CKind::Ordinate,
            },
            (Point, Line) => CKind::PointLineDistance,
            (Line, Line) => CKind::ParallelDistance,
            // two parallel planes apart: what a stack is written in
            (Plane, Plane) => CKind::PlaneDistance,
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
        // the ordinate's zero; `horizontal` and `vertical` between points are the standard
        // library's words for `level(up)` and `level(right)` (`std.sv`, §9.9)
        "level" => match (a, b) {
            (Point, Point | Plane) => CKind::Level,
            _ => return None,
        },
        "angle" => match (a, b) {
            (Line, Line) => CKind::Angle,
            (x, y) if axes(x, y) => CKind::Angle3,
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
            (x, y) if axes(x, y) => CKind::Parallel3,
            (Axis, Plane) => CKind::AxisParallelPlane,
            (Plane, Plane) => CKind::PlaneParallel,
            _ => return None,
        },
        "perpendicular" => match (a, b) {
            (Line, Line) => CKind::Perpendicular,
            (x, y) if axes(x, y) => CKind::Perpendicular3,
            // square to a plane: along its normal
            (Axis, Plane) => CKind::AxisPerpendicularPlane,
            _ => return None,
        },
        "symmetry" => match (a, b) {
            (Point, Point) => CKind::Symmetric,
            _ => return None,
        },
        _ => return None,
    })
}

/// **Whether the language itself has a word of this fixity** (§9.9): what a relation word may
/// not be defined as.  Read off the tables that settle a statement — `infix_op` and `prefix_op`
/// over every pair of kinds, the gauges, the calls and the words about solids — so a word the
/// core comes to know is reserved with nothing here to edit.  `horizontal` is a prefix word of
/// the core's (a line's direction) and an infix word of the standard library's (two points'
/// level), which is why the question is asked of a fixity.  Every such word is an operator, so
/// the tables are read once, over `OPERATORS`.
pub fn builtin_word(word: &str, fixity: Fixity) -> bool {
    static BUILTIN: std::sync::OnceLock<Vec<(&'static str, Fixity)>> = std::sync::OnceLock::new();
    let table = BUILTIN.get_or_init(|| {
        let has = |w: &str, f: Fixity| match f {
            Fixity::Call => call_word(w),
            Fixity::Prefix => {
                matches!(gauge_op(w).map(CKind::operator), Some(Some((_, Fixity::Prefix))))
                    || EntKind::ALL.iter().any(|&k| prefix_op(w, k).is_some())
            }
            Fixity::Infix => {
                solid_word(w).is_some()
                    || EntKind::ALL.iter().any(|&a| {
                        EntKind::ALL.iter().any(|&b| infix_op(w, a, b, &|_| None).is_some())
                    })
            }
        };
        let mut out = Vec::new();
        for w in OPERATORS {
            for f in [Fixity::Call, Fixity::Prefix, Fixity::Infix] {
                if has(w, f) {
                    out.push((w, f));
                }
            }
        }
        out
    });
    table.contains(&(word, fixity))
}

/// The same for a word standing *before* its one operand.  `distance` on a line is sugar for the
/// distance between its ends, which is why it is here and not in the table above.
pub fn prefix_op(word: &str, on: EntKind) -> Option<CKind> {
    use EntKind::{Arc, Circle, Line, Spline};
    Some(match (word, on) {
        ("horizontal", Line) => CKind::Horizontal,
        ("vertical", Line) => CKind::Vertical,
        ("radius", Circle | Arc) => CKind::Radius,
        // an arc's length along itself: the other way a round thing is dimensioned
        ("length", Arc) => CKind::ArcLength,
        // and a spline's, along the whole of it, and a free curve's, its own
        ("length", Spline) => CKind::SplineLength,
        ("length", EntKind::Curve) => CKind::CurveLength,
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
    "distance", "tangent", "equal", "curvature", "horizontal", "vertical", "level", "angle",
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
    /// `radius(25) circle1`, `horizontal line1`, `fix((0, 0)) p1`
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
    /// A plane: its origin and its two axes, read by the relations in space and by `Project`.
    Plane,
    /// An axis: a directed line in space, in no view.
    Axis,
    /// A direction in space: a line drawn in some view, or an axis — what `parallel`,
    /// `perpendicular` and `angle` read across views and between axes.
    Direction,
    /// What an ordinate is measured along (`Ordinate`, `Level`): a line or an axis, as a
    /// `Direction` is, or a plane, standing for its normal — never written so, only through the
    /// word `n` against the plane (`program::relations`), since "along a plane" reads as within it.
    Along,
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
    /// `Param` is stated Scalar: a slot's hidden unknown is a place along a curve
    /// (`CKind::param_dim`).
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
            | SpecKind::Axis
            | SpecKind::Direction
            | SpecKind::Along
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
                | SpecKind::Axis
                | SpecKind::Direction
                | SpecKind::Along
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

    /// The slot's kind with its article: `a line`, `an axis`.
    pub fn a(self) -> String {
        crate::model::article(self.as_str())
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
            SpecKind::Axis => "axis",
            SpecKind::Direction => "direction",
            SpecKind::Along => "along",
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
            CKind::DragSeen => "DragSeen",
            CKind::Horizontal => "Horizontal",
            CKind::Vertical => "Vertical",
            CKind::Parallel => "Parallel",
            CKind::Perpendicular => "Perpendicular",
            CKind::Angle => "Angle",
            CKind::EqualAngle => "EqualAngle",
            CKind::ArcLength => "ArcLength",
            CKind::SplineLength => "SplineLength",
            CKind::CurveLength => "CurveLength",
            CKind::Stationary => "Stationary",
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
            CKind::Ordinate => "Ordinate",
            CKind::Level => "Level",
            CKind::PointOnCurve => "PointOnCurve",
            CKind::PointOnExtrusion => "PointOnExtrusion",
            CKind::CurveTangentLine => "CurveTangentLine",
            CKind::CurveCurvature => "CurveCurvature",
            CKind::Project => "Project",
            CKind::AxisOnPlane => "AxisOnPlane",
            CKind::AxisCoincident => "AxisCoincident",
            CKind::AxisParallelPlane => "AxisParallelPlane",
            CKind::AxisPerpendicularPlane => "AxisPerpendicularPlane",
            CKind::PlaneParallel => "PlaneParallel",
            CKind::PlaneDistance => "PlaneDistance",
            CKind::Lift => "Lift",
            CKind::Coincident3 => "Coincident3",
            CKind::Distance3 => "Distance3",
            CKind::PointLine3 => "PointLine3",
            CKind::LineLine3 => "LineLine3",
            CKind::Angle3 => "Angle3",
            CKind::Perpendicular3 => "Perpendicular3",
            CKind::Parallel3 => "Parallel3",
            CKind::PointOnPlane => "PointOnPlane",
            CKind::PointOnCircle3 => "PointOnCircle3",
            CKind::PointOnLine3 => "PointOnLine3",
            CKind::EqualLength3 => "EqualLength3",
            CKind::Midpoint3 => "Midpoint3",
            CKind::Symmetric3 => "Symmetric3",
            CKind::LineOnPlane => "LineOnPlane",
            CKind::ProjectSolved => "ProjectSolved",
            CKind::AxisUnit => "AxisUnit",
            CKind::AxisFoot => "AxisFoot",
            CKind::PlaneAxis => "PlaneAxis",
            CKind::PointOnAxis => "PointOnAxis",
            CKind::LineOnAxis => "LineOnAxis",
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
            CKind::DragSeen => &[
                ("p", S::Point),
                ("tx", S::Float),
                ("ty", S::Float),
                ("weight", S::Float),
                ("az", S::Float),
                ("el", S::Float),
            ],
            CKind::Horizontal | CKind::Vertical => &[("line", S::Line)],
            // how far `q` stands from `p` along `t`, signed from `p` to `q`, so the pair is not
            // commutative.  `t` is written as a reference, or left to the core where `along` names
            // it in a word (`x`, `up`, `u` …), the word kept for its sign and its spelling; `form`
            // is which kernel reads it, the core's alone (`OrdinateForm`)
            CKind::Ordinate => &[
                ("p", S::Point),
                ("q", S::Point),
                ("t", S::Along),
                ("d", S::Length),
                ("along", S::Str),
                ("form", S::Int),
            ],
            // the same at no distance: the two level along `t`
            CKind::Level => &[
                ("p", S::Point),
                ("q", S::Point),
                ("t", S::Along),
                ("along", S::Str),
                ("form", S::Int),
            ],
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
            CKind::SplineLength => &[("spline", S::Spline), ("l", S::Length)],
            CKind::CurveLength => &[("curve", S::Curve), ("l", S::Length)],
            CKind::Stationary => &[
                ("curve", S::Curve),
                ("weight", S::Float),
                ("integrand", S::Str),
                ("degree", S::Int),
                ("maximize", S::Bool),
            ],
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
            // the view point and its view: the hidden point is the lift's own, found by the
            // point (`Sketch::lift_of`), and the plane is a real slot so a drag part, a
            // deletion and the topology key follow it
            CKind::Lift => &[("p", S::Point), ("plane", S::Plane)],
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
            CKind::Angle3 => &[("l1", S::Direction), ("l2", S::Direction), ("theta", S::Angle)],
            CKind::Perpendicular3 | CKind::Parallel3 => &[("l1", S::Direction), ("l2", S::Direction)],
            CKind::PointOnPlane => {
                &[("p", S::Point), ("plane", S::Plane)]
            }
            CKind::PointOnCircle3 => {
                &[("p", S::Point), ("circle", S::CircleOrArc)]
            }
            CKind::PointOnLine3 => &[("p", S::Point), ("line", S::Line)],
            CKind::EqualLength3 => &[("l1", S::Line), ("l2", S::Line)],
            CKind::Midpoint3 => &[("p", S::Point), ("line", S::Line)],
            CKind::Symmetric3 => &[("p", S::Point), ("q", S::Point), ("line", S::Line)],
            CKind::LineOnPlane => &[("line", S::Line), ("plane", S::Plane)],
            // the two planes are real slots — so the drag part, the topology key, the graft
            // and a deletion follow them — and inferred ones, so nobody writes them
            CKind::Project | CKind::ProjectSolved => {
                &[("a", S::Point), ("b", S::Point), ("pa", S::Plane), ("pb", S::Plane)]
            }
            CKind::AxisUnit | CKind::AxisFoot => &[("axis", S::Axis)],
            CKind::PlaneAxis => &[("plane", S::Plane), ("axis", S::Axis)],
            CKind::PointOnAxis => &[("p", S::Point), ("axis", S::Axis)],
            CKind::LineOnAxis => &[("line", S::Line), ("axis", S::Axis)],
            CKind::AxisOnPlane | CKind::AxisParallelPlane | CKind::AxisPerpendicularPlane => {
                &[("axis", S::Axis), ("plane", S::Plane)]
            }
            CKind::AxisCoincident => &[("a", S::Axis), ("b", S::Axis)],
            CKind::PlaneParallel => &[("p1", S::Plane), ("p2", S::Plane)],
            CKind::PlaneDistance => &[("p1", S::Plane), ("p2", S::Plane), ("d", S::Length)],
            // the entity, and the numbers it holds, each pinned under the member it is
            // (`model::EntKind::members`): `fix((0, 0)) p` fills `x` and `y`, `fix(r == 25) c`,
            // `fix(dir == (1, 0, 0)) t` an axis's `dir.x`, `dir.y` and `dir.z`.  Every member
            // a kind owns is a slot here, and which of them the entity has is the gauge's own
            // check
            CKind::Fix => &[
                ("of", S::Scalar),
                ("x", S::Param),
                ("y", S::Param),
                // a point in space's third
                ("z", S::Param),
                ("r", S::Param),
                ("dir.x", S::Param),
                ("dir.y", S::Param),
                ("dir.z", S::Param),
                // where an axis is, the point on it nearest the world's origin; where a plane
                // stands, its own origin
                ("origin.x", S::Param),
                ("origin.y", S::Param),
                ("origin.z", S::Param),
            ],
            // the predicate is about the triangle, so all three stand in the parentheses
            CKind::Ccw | CKind::Cw => &[("a", S::Point), ("b", S::Point), ("c", S::Point)],
        }
    }

    /// What this type's `SpecKind::Param` slot *is* (`units.rs`) — `None` where it owns none.
    ///
    /// `SpecKind::dim()` cannot answer this, which is why it is asked here: every hidden unknown
    /// is a **place along a curve**, dimensionless, which a paste between documents in different
    /// units must not convert.  `every_param_slot_states_its_dimension` holds every type
    /// that owns a Param to naming it here, so a new one cannot arrive unstated.
    pub fn param_dim(self) -> Option<crate::units::Dim> {
        use crate::units::Dim;
        match self {
            // a place along a curve: a parameter, not a length
            CKind::PointOnSpline
            | CKind::PointOnCurve
            | CKind::PointOnExtrusion
            | CKind::CurveTangentLine
            | CKind::CurveCurvature
            | CKind::SplineTangentLine
            | CKind::SplineCurvature
            => Some(Dim::SCALAR),
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
    /// `Lift`, `AxisUnit` and `AxisFoot` are intrinsic (`Sketch::lift_point`, `Sketch::axis`,
    /// `Sketch::place_axis`).
    ///
    /// Several kinds share a word, and that is where the saving is: **`coincident` is nineteen
    /// kinds, `distance` nine, `tangent` ten**, and `horizontal`/`vertical` are a line's direction
    /// prefixed and, between a pair of points infixed, the aliases of `level`.
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
            | CKind::PointOnExtrusion => ("coincident", Infix),
            CKind::CurveTangentLine => ("tangent", Infix),
            CKind::CurveCurvature => ("curvature", Infix),
            // a measured separation: told apart by the pair and by `along:`, which makes it an
            // ordinate
            CKind::Distance
            | CKind::Ordinate
            | CKind::PointLineDistance
            | CKind::ParallelDistance
            | CKind::AnnularDistance => ("distance", Infix),
            // the ordinate's zero; `horizontal` and `vertical` between points are the standard
            // library's words for two of its readings (§9.9)
            CKind::Level => ("level", Infix),
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
            CKind::Vertical => ("vertical", Prefix),
            // `angle` and `radius` keep their own words rather than folding into `distance`:
            // over two lines a Length means a parallel distance and an Angle means an angle, and
            // nothing but the number's unit could separate them
            CKind::Angle => ("angle", Infix),
            // the same word with a second pair where the number would be: an angle stated as
            // another angle rather than as a number
            CKind::EqualAngle => ("angle", Infix),
            CKind::Radius => ("radius", Prefix),
            CKind::ArcLength | CKind::SplineLength | CKind::CurveLength => ("length", Prefix),
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
            CKind::Angle3 => ("angle", Infix),
            CKind::Perpendicular3 => ("perpendicular", Infix),
            CKind::Parallel3 => ("parallel", Infix),
            CKind::EqualLength3 => ("equal", Infix),
            CKind::PointOnPlane => ("coincident", Infix),
            CKind::PointOnCircle3 => ("coincident", Infix),
            CKind::PointOnLine3 => ("coincident", Infix),
            CKind::Midpoint3 => ("midpoint", Infix),
            CKind::Symmetric3 => ("symmetry", Infix),
            CKind::LineOnPlane => ("coincident", Infix),
            CKind::DragTarget
            | CKind::DragSeen
            | CKind::Lift
            // an energy, which is a statement of its own
            | CKind::Stationary
            // an axis's own algebra
            | CKind::AxisUnit
            | CKind::AxisFoot
            | CKind::PlaneAxis => return None,
            CKind::PointOnAxis | CKind::LineOnAxis | CKind::AxisOnPlane | CKind::AxisCoincident => {
                ("coincident", Infix)
            }
            CKind::AxisParallelPlane | CKind::PlaneParallel => ("parallel", Infix),
            CKind::AxisPerpendicularPlane => ("perpendicular", Infix),
            CKind::PlaneDistance => ("distance", Infix),
        })
    }

    /// The value an omitted argument takes.  One table, read by the JSON path and by both
    /// bindings, so a default can never drift between them.
    pub fn default_arg(self, i: usize) -> Arg {
        match (self, i) {
            (CKind::DragTarget | CKind::DragSeen, 3) => Arg::Num(1.0),
            (CKind::TangentCircleCircle, 2) => Arg::Bool(true),
            (CKind::TangentArcLine, 2) => Arg::Str("start".to_string()),
            (CKind::TangentLineCircleAt, 2) => Arg::Str("p1".to_string()),
            (CKind::TangentLineCircle, 2) => Arg::Str("left".to_string()),
            (CKind::LineLine3, 3) => Arg::Int(1),
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
            (CKind::Ordinate, 4) | (CKind::Level, 3) => &ALONG_WORDS[..],
            (CKind::Angle, 3) => &["ccw", "cw"][..],
            (CKind::EqualAngle, 4) => &["ccw", "cw"][..],
            _ => return None,
        })
    }

    /// The slots naming axes whose place in space this reads, not only their direction: an axis
    /// has a place only while something reads it (`Sketch::place_axis`).
    pub fn place_slots(self) -> &'static [usize] {
        match self {
            CKind::PointOnAxis | CKind::LineOnAxis | CKind::PlaneAxis => &[1],
            CKind::AxisOnPlane => &[0],
            CKind::AxisCoincident => &[0, 1],
            _ => &[],
        }
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
            // the page's two axes either way: the run measured `left` is the run turned round.  A
            // level's direction has no sense, so it has no side either
            CKind::Ordinate => {
                (4, &[("right", 1.0), ("left", -1.0), ("up", 1.0), ("down", -1.0)][..])
            }
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
                    // the direction where a word names it, and the form always
                    | (CKind::Ordinate, 2 | 5)
                    | (CKind::Level, 2 | 4)
            )
    }

    /// The slot an ordinate holds its form in (`OrdinateForm`), and the one its direction's word
    /// is in: for `Ordinate` and `Level`, `None` for every other kind.
    pub fn form_slot(self) -> Option<usize> {
        match self {
            CKind::Ordinate => Some(5),
            CKind::Level => Some(4),
            _ => None,
        }
    }

    pub fn word_slot(self) -> Option<usize> {
        self.form_slot().map(|i| i - 1)
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
                | CKind::ArcLength
                | CKind::SplineLength
                | CKind::CurveLength
        )
    }

    pub fn claimable(self) -> bool {
        !self.gauge() && !self.spec().iter().any(|(_, k)| k.is_param())
    }

    /// The spec slots a contact on a parametric entity of kind `of` is made of: which argument
    /// names the entity and which holds the parameter along it.  Read off the spec, so a new
    /// kind of contact is covered by declaring one — there is no table of kinds here to forget
    /// to extend.
    pub(crate) fn contact_on(self, of: SpecKind) -> Option<(usize, usize)> {
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

    /// Whether this kind's kernel is built from the sketch at a width of its operands' (a
    /// spline length's control-point count) rather than registered: `Constraint::kernel_key`
    /// names it, and nothing may ask `kernel()` for it.
    pub fn built(self) -> bool {
        matches!(self, CKind::SplineLength | CKind::Stationary)
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
            | CKind::DragSeen
            | CKind::Horizontal
            | CKind::Vertical
            | CKind::Parallel
            | CKind::Perpendicular
            | CKind::Angle
            | CKind::EqualAngle
            | CKind::ArcLength
            | CKind::SplineLength
            | CKind::CurveLength
            | CKind::Stationary
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
            | CKind::Ordinate
            | CKind::Level
            // a projection is a linear tie between two images: no contact, no double root
            | CKind::Project
            // a hidden point tied to the point it lifts: no contact
            | CKind::Lift
            // incidence and measure in space: no double root the screen knows how to look for
            | CKind::Coincident3
            | CKind::Distance3
            | CKind::PointLine3
            | CKind::LineLine3
            | CKind::Angle3
            | CKind::Perpendicular3
            | CKind::Parallel3
            | CKind::PointOnPlane
            | CKind::PointOnCircle3
            | CKind::PointOnLine3
            | CKind::EqualLength3
            | CKind::LineOnPlane
            | CKind::Midpoint3
            | CKind::Symmetric3
            // the projector rule in space, and an axis's own rows and relations: algebra, no contact
            | CKind::ProjectSolved
            | CKind::AxisUnit
            | CKind::AxisFoot
            | CKind::PlaneAxis
            | CKind::PointOnAxis
            | CKind::LineOnAxis
            | CKind::AxisOnPlane
            | CKind::AxisCoincident
            | CKind::AxisParallelPlane
            | CKind::AxisPerpendicularPlane
            | CKind::PlaneParallel
            | CKind::PlaneDistance
            | CKind::Fix
            | CKind::Ccw
            | CKind::Cw => false,
        }
    }

    /// Types that do not have to be satisfied — a drag target compromises, it does not hold.
    pub fn soft_by_default(self) -> bool {
        matches!(self, CKind::DragTarget | CKind::DragSeen)
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
                // level either way round: the zero is its own negative
                | CKind::Level
                | CKind::Coincident3
                | CKind::Distance3
                | CKind::Angle3
                | CKind::Perpendicular3
                | CKind::EqualLength3
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
            CKind::SplineLength => panic!("a spline length's kernel belongs to its spline's count"),
            CKind::Stationary => panic!("an energy's kernel belongs to its curve"),
            CKind::CurveLength => K::Radius,
            CKind::Coincident => K::Coincident,
            CKind::Distance => K::Distance,
            CKind::Midpoint => K::Midpoint,
            CKind::DragTarget => K::Drag,
            CKind::DragSeen => K::DragSeen,
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
            // an ordinate's kernel is its form's (`Constraint::kernel`); the kind's own is the run,
            // and a level's the line kernel that reads a pair of points' heights alike
            CKind::Ordinate => K::OrdinateU,
            CKind::Level => K::Horizontal,
            CKind::Project => K::Project,
            CKind::Lift => K::Lift,
            CKind::Coincident3 => K::Coincident3,
            CKind::Distance3 => K::Distance3,
            CKind::PointLine3 => K::PointLine3,
            CKind::LineLine3 => K::LineLine3,
            CKind::Angle3 => K::Angle3,
            CKind::Perpendicular3 => K::Perpendicular3,
            CKind::Parallel3 => K::Parallel3,
            CKind::PointOnPlane => K::PointOnPlane,
            CKind::PointOnCircle3 => K::PointOnCircle3,
            CKind::PointOnLine3 => K::PointOnLine3,
            CKind::EqualLength3 => K::EqualLength3,
            CKind::Midpoint3 => K::Midpoint3,
            CKind::Symmetric3 => K::Symmetric3,
            CKind::LineOnPlane => K::LineOnPlane,
            CKind::ProjectSolved => K::ProjectSolved,
            CKind::AxisUnit => K::AxisUnit,
            CKind::AxisFoot => K::AxisFoot,
            CKind::PlaneAxis => K::PointOnAxis,
            CKind::PointOnAxis => K::PointOnAxis,
            CKind::LineOnAxis => K::LineOnAxis,
            CKind::AxisOnPlane => K::AxisOnPlane,
            CKind::AxisCoincident => K::AxisCoincident,
            CKind::AxisParallelPlane => K::AxisParallelPlane,
            CKind::AxisPerpendicularPlane => K::AxisPerpendicularPlane,
            CKind::PlaneParallel => K::PlaneParallel,
            CKind::PlaneDistance => K::PlaneDistance,
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
            // its free twin is built beside it, at the spline's count (`KernelKey::SplineLength`)
            CKind::SplineLength => return None,
            CKind::CurveLength => K::RadiusFree,
            CKind::Stationary => return None,
            CKind::ParallelDistance => K::ParallelDistanceFree,
            CKind::PointLineDistance => K::PointLineDistanceFree,
            CKind::AnnularDistance => K::AnnularDistanceFree,
            CKind::Ordinate => K::OrdinateUFree,
            CKind::PlaneDistance => K::PlaneDistanceFree,
            CKind::Distance3 => K::Distance3Free,
            CKind::PointLine3 => K::PointLine3Free,
            CKind::LineLine3 => K::LineLine3Free,
            CKind::Angle3 => K::Angle3Free,
            CKind::Coincident
            | CKind::Midpoint
            | CKind::DragTarget
            | CKind::DragSeen
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
            | CKind::Level
            | CKind::Project
            | CKind::Lift
            | CKind::Coincident3
            | CKind::Perpendicular3
            | CKind::Parallel3
            | CKind::PointOnPlane
            | CKind::PointOnCircle3
            | CKind::PointOnLine3
            | CKind::EqualLength3
            | CKind::LineOnPlane
            | CKind::Midpoint3
            | CKind::Symmetric3
            | CKind::ProjectSolved
            | CKind::AxisUnit
            | CKind::AxisFoot
            | CKind::PlaneAxis
            | CKind::PointOnAxis
            | CKind::LineOnAxis
            | CKind::AxisOnPlane
            | CKind::AxisCoincident
            | CKind::AxisParallelPlane
            | CKind::AxisPerpendicularPlane
            | CKind::PlaneParallel
            | CKind::Fix
            | CKind::Ccw
            | CKind::Cw => return None,
        })
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
                | CKind::PointOnCircle3
                | CKind::ProjectSolved
                | CKind::PointOnLine3
                | CKind::EqualLength3
                | CKind::LineOnPlane
                | CKind::Midpoint3
                | CKind::Symmetric3
                | CKind::PointOnExtrusion
                | CKind::PointOnAxis
                | CKind::LineOnAxis
                | CKind::AxisOnPlane
                | CKind::AxisCoincident
                | CKind::AxisParallelPlane
                | CKind::AxisPerpendicularPlane
                | CKind::PlaneParallel
                | CKind::PlaneDistance
        )
    }

    /// The form of a statement that reads planes, for planes of which one is solved or both
    /// fixed: the one table of the pairs, asked by `Sketch::add` when a statement arrives, so a
    /// caller names either and the kernel is always the one the planes can feed.  Every other
    /// kind is itself — a kernel over a plane's axes and origin serves a fixed plane as well,
    /// its held columns dropping out — and the projection is the one pair: over two fixed planes
    /// it is two drawn points and a constant fold line, which needs no hidden point.
    pub fn attitude_twin(self, solved: bool) -> CKind {
        match ATTITUDE_TWINS.iter().find(|(s, f)| *s == self || *f == self) {
            Some(&(s, f)) => if solved { s } else { f },
            None => self,
        }
    }
}

/// The pairs `CKind::attitude_twin` reads: a statement over a solved plane, and the same
/// statement over fixed ones.
const ATTITUDE_TWINS: [(CKind, CKind); 1] = [(CKind::ProjectSolved, CKind::Project)];

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
    /// What a `Param` slot holds on the way in when the unknown is **shared**: the name every
    /// contact owning it is written against (`t == s`, issue #70, part 2), and where it starts
    /// if this is the first of them.  `Sketch::add` allocates the unknown for the first and hands
    /// the same index to the rest (`Sketch::shared`), so the slot is an `Arg::Param` after it,
    /// as every owned unknown is.
    Shared { name: String, seed: Option<f64> },
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
            Arg::Shared { seed, .. } => seed.unwrap_or(0.0),
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
    /// The defined word the statement was written with (§9.9), where it was one: `a horizontal
    /// b` reaches the sketch as the `Level` its body states, and is described as written.
    /// Presentation, like `class`; `graft` carries it while its operands survive.
    pub word: Option<WordUse>,
    /// **Stated as its derivative** (§6.21): this row's rate as a tangency's use moves along the
    /// set — the index of its `Sketch::duals` entry, which says what moves each column it reads.
    /// What `l tangent S` makes of each row of a set's body, at the contact on `l`.  `None` for
    /// every constraint stated as itself.  Not presentation: it picks the kernel
    /// (`kernels::dual_kernel`) and appends a tangent column per column and the line's ends to
    /// the columns, as `free` appends its column.
    pub along: Option<usize>,
}

/// A line's two ends lifted, three columns each.
fn lifted_ends(sk: &Sketch, line: usize) -> Vec<u32> {
    let l = &sk.lines[line];
    [l.p1, l.p2]
        .iter()
        .flat_map(|&p| {
            let k = sk.lift_of(p as usize).expect("a relation in space is lifted at the add");
            sk.lifts[k].x
        })
        .collect()
}

/// A defined relation word as a statement wrote it — see `Constraint::word`.
#[derive(Clone, Debug, PartialEq)]
pub struct WordUse {
    pub word: String,
    /// The entities the statement's operands named, in written order: two of an infix word —
    /// but for a set (§6.21), which is no entity and stands in `set`.
    pub ops: Vec<EntRef>,
    /// The parentheses' text as written (`d: 5mm`), or empty.
    pub args: String,
    /// The operands that are sets, each by its place among the operands and its name as written.
    pub sets: Vec<(usize, String)>,
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
            word: None,
            along: None,
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

    /// The soft target of a drag of a point in space, seen by the eye at bearing `az` and
    /// elevation `el` at (`tx`, `ty`) on its picture plane.
    pub fn drag_seen(p: EntRef, tx: f64, ty: f64, weight: f64, az: f64, el: f64) -> Constraint {
        let mut c = Constraint::new(
            CKind::DragSeen,
            vec![Arg::Ent(p), Arg::Num(tx), Arg::Num(ty), Arg::Num(weight), Arg::Num(az), Arg::Num(el)],
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

    /// An ordinate (`docs/ordinate-plan.md`): how far `q` stands from `p` along `t` — an axis,
    /// a line, or a plane standing for its normal.  Its form is read when it is added.
    pub fn ordinate(p: EntRef, q: EntRef, t: EntRef, d: f64) -> Constraint {
        Constraint::new(CKind::Ordinate, vec![Arg::Ent(p), Arg::Ent(q), Arg::Ent(t), Arg::Num(d)])
    }

    /// The two level along `t`: the ordinate's zero.
    pub fn level(p: EntRef, q: EntRef, t: EntRef) -> Constraint {
        Constraint::new(CKind::Level, vec![Arg::Ent(p), Arg::Ent(q), Arg::Ent(t)])
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
    /// The static one (`kernel_id`) for every type but those whose kernel is built from the
    /// sketch.  A curve contact's belongs to the curve's *definition* — different families read
    /// different numbers of coordinates, so they cannot share a block — a spline's length to its
    /// control-point count, and a derivative to its row's kernel.  `System` builds one kernel per
    /// key present (`system::build_kernel`) and orders its blocks by key, so the derived order
    /// keeps every static block ahead of every built one.
    pub fn kernel_key(&self, sk: &Sketch) -> kernels::KernelKey {
        use kernels::KernelKey as Key;
        // a derivative's kernel is its row's kernel's twin (§6.21)
        if self.along.is_some() {
            return Key::Dual(self.kernel_id());
        }
        if self.kind == CKind::SplineLength {
            let n = sk.splines[self.args[0].ent().i()].ctrl.len();
            return Key::SplineLength { n, free: self.free.is_some() };
        }
        if self.kind == CKind::Stationary {
            return Key::Stationary(self.id);
        }
        match (self.kind.family_kernel(), self.curve_of()) {
            (Some(fk), Some(e)) => Key::Family { def: sk.curves[e.i()].def as usize, fk: fk as u8 },
            _ => Key::Static(self.kernel_id()),
        }
    }

    /// The kernel itself (`kernel_key`, built where it must be).
    pub fn kernel_in(&self, sk: &Sketch) -> Kernel {
        crate::system::build_kernel(sk, self.kernel_key(sk))
    }

    /// Rows this constraint compiles to, when the sketch is at hand: `n_residuals` for every kind
    /// whose rows are a fact about it; an energy's — its curve's length stationary, one row on its
    /// first statement where nothing holds the length — and a peg's, none (`variational.rs`).
    pub fn rows_in(&self, sk: &Sketch) -> usize {
        match self.kind {
            CKind::Stationary => crate::variational::rows(sk, self.id),
            // a held point a free curve passes is its problem's, not a row of the drawing's
            CKind::PointOnCurve if sk.is_peg(self.id) => 0,
            _ => self.n_residuals(),
        }
    }

    /// The curve a per-definition kernel is over — the spec's `Curve` slot, wherever it stands.
    pub fn curve_of(&self) -> Option<EntRef> {
        let (e, _) = self.kind.contact_on(SpecKind::Curve)?;
        Some(self.args[e].ent())
    }

    /// Which kernel evaluates this constraint: its type's, or the free-variable twin when the
    /// number it states is an unknown rather than a constant.
    pub(crate) fn kernel(&self) -> K {
        let free = self.free.is_some();
        // an ordinate is read by the kernel its operands give it (`OrdinateForm`), and its zero
        // by the same at no distance — but along a view's own axis by the line kernel that
        // levels a pair of points
        if let Some(form) = self.form() {
            use OrdinateForm as F;
            return match (self.kind, form, free) {
                (CKind::Level, F::PageU, _) => K::Vertical,
                (CKind::Level, F::PageV, _) => K::Horizontal,
                (_, F::CoordU | F::CoordV, false) => K::Radius,
                (_, F::CoordU | F::CoordV, true) => K::RadiusFree,
                (_, F::PageU, false) => K::OrdinateU,
                (_, F::PageU, true) => K::OrdinateUFree,
                (_, F::PageV, false) => K::OrdinateV,
                (_, F::PageV, true) => K::OrdinateVFree,
                (_, F::InView, false) => K::OrdinateLine,
                (_, F::InView, true) => K::OrdinateLineFree,
                (_, F::Space, false) => K::OrdinateSpace,
                (_, F::Space, true) => K::OrdinateSpaceFree,
                (_, F::FrameU, false) => K::OrdinateFrameU,
                (_, F::FrameU, true) => K::OrdinateFrameUFree,
                (_, F::FrameV, false) => K::OrdinateFrameV,
                (_, F::FrameV, true) => K::OrdinateFrameVFree,
                (_, F::FrameN, false) => K::OrdinateFrameN,
                (_, F::FrameN, true) => K::OrdinateFrameNFree,
            };
        }
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
        let d = self.dimensions().first().map_or(0.0, |&(i, _, _)| self.args[i].num());
        // an ordinate is signed already, its word turning it: there is no line to read a side off
        if self.kind == CKind::Ordinate {
            return self.side().unwrap_or(1.0) * d;
        }
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

    /// An ordinate's form (`OrdinateForm`), or `None` for every other kind.
    pub fn form(&self) -> Option<OrdinateForm> {
        let i = self.kind.form_slot()?;
        Some(match self.args.get(i) {
            Some(Arg::Int(f)) => OrdinateForm::from_int(*f),
            _ => OrdinateForm::Space,
        })
    }

    /// Whether this statement reads its points where they stand in space — a relation in space,
    /// or an ordinate whose form is one — so `Sketch::add` mints their lifts.
    pub fn reads_space(&self) -> bool {
        self.kind.spatial() || self.form().is_some_and(|f| f.in_space())
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
            None if self.kind == CKind::SplineLength => 1,
            // its curve's, which only the sketch can say: `rows_in`
            None if self.kind == CKind::Stationary => 0,
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

    /// Whether this constraint owns the Param `p` — `aux_params` asked of one index, building
    /// nothing.
    pub fn owns(&self, p: u32) -> bool {
        let spec = self.kind.spec();
        spec.iter().zip(&self.args).any(|((_, k), a)| k.is_param() && *a == Arg::Param(p))
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
    /// *document*: the unknown it read is retired only when nothing else reads it.
    /// `Sketch::set_constraint_num` is the path that settles that, and is what a caller holding a
    /// sketch should use; this one is for a constraint that has no document behind it yet, or an
    /// argument no expression can reach (a soft drag target's own number).
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

    /// Move a drag target's target point.  No other kind has one, and several are shorter than
    /// three arguments, so the kind is checked rather than the write being attempted blind.
    pub fn set_target(&mut self, tx: f64, ty: f64) -> bool {
        if !matches!(self.kind, CKind::DragTarget | CKind::DragSeen) {
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
        let inner = self.own_consts_on(sk, span);
        let Some(d) = self.along else { return inner };
        // a derivative's: its row's kernel, the row's own constants, then how each of the row's
        // columns moves — held, by a component of the line's direction, or by a tangent column
        // of its own (§6.21)
        let moves = sk.dual_moves(d);
        let mut k = Vec::with_capacity(1 + inner.len() + 8);
        k.push(self.kernel_id() as f64);
        k.extend(inner);
        for p in self.params_on_row(sk, span) {
            k.push(match moves.of(p) {
                crate::model::Move::Held => 0.0,
                crate::model::Move::Dir(c) => (c + 1) as f64,
                crate::model::Move::Tangent => kernels::TANGENT as f64,
            });
        }
        k
    }

    fn own_consts_on(&self, sk: &Sketch, span: Option<usize>) -> Vec<f64> {
        if self.kind == CKind::Stationary {
            return crate::variational::pack(sk, self.id);
        }
        // a spline's length reads the whole curve's knots and weights before what it states
        if self.kind == CKind::SplineLength {
            let s = &sk.splines[self.args[0].ent().i()];
            let stated = match self.free {
                Some(f) => vec![f.m, f.c],
                None => vec![self.args[1].num()],
            };
            return crate::integral::length_consts(&s.knots, s.weights.as_deref(), s.ctrl.len(), &stated);
        }
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
            return vec![s * f.m, s * f.c];
        }
        if let Some((sp, t)) = self.spline_contact(sk) {
            let span = span.unwrap_or_else(|| crate::curve::span_of(sk, sp, t));
            let s = &sk.splines[sp];
            let mut k = crate::curve::local_knots(&s.knots, span).to_vec();
            k.extend(crate::curve::local_weights(s.weights.as_deref(), span));
            return k;
        }
        // a curve contact carries its family's compiled body — two tapes, or a whole trace
        // block — and the numbers the instance was given.  They are the same for every contact
        // with the same curve, and duplicated per constraint because that is where a block
        // already has room for numbers — which is what keeps the kernel table `fn`-pointered and
        // ignorant of curves.
        if let Some(curve) = self.curve_of() {
            let cv = &sk.curves[curve.i()];
            let d = &sk.curve_defs[cv.def as usize];
            let mut k = match &d.body {
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
                crate::model::CurveBody::Envelope(g) => g.contact_consts(sk, curve.i()),
                // a free curve's problem: its Lagrangian and where its pegs are
                crate::model::CurveBody::Extremal(x) => {
                    sk.extremal_consts(curve.i()).unwrap_or_else(|| vec![0.0; x.n_const()])
                }
            };
            // a point in space reads the curve's plane first: its basis in space, `u`, `v`, `o`
            if self.kind == CKind::PointOnExtrusion {
                k.splice(0..0, sk.extrusion_frame(curve.i()));
            }
            return k;
        }
        match self.kind {
            CKind::Distance => vec![self.args[2].num()],
            // signed from the first point to the second, and which way is the word: `along: left`
            // is the minus a drawing used to write (§9.2)
            CKind::Ordinate => vec![self.side().unwrap_or(1.0) * self.args[3].num()],
            // the line kernels a level reads along its view's axes take no number; the others a
            // zero
            CKind::Level => match self.form() {
                Some(OrdinateForm::PageU | OrdinateForm::PageV) => Vec::new(),
                _ => vec![0.0],
            },
            CKind::DragTarget => {
                vec![self.args[1].num(), self.args[2].num(), self.args[3].num()]
            }
            CKind::DragSeen => {
                let (right, up) = crate::overview::eye(self.args[4].num(), self.args[5].num());
                [&[self.args[1].num(), self.args[2].num(), self.args[3].num()][..], &right, &up]
                    .concat()
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
            CKind::ArcLength | CKind::CurveLength => vec![self.args[1].num()],
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
                let (a, b) = (basis(2), basis(3));
                let (da, db) = crate::plane::fold_line(&a, &b)
                    .expect("a projection between parallel planes is refused at the add");
                vec![da[0], da[1], db[0], db[1], crate::plane::fold_offset(&a, &b)]
            }
            CKind::Distance3 | CKind::PointLine3 => vec![self.args[2].num()],
            // the magnitude turned to the side the lines stood on when it was stated
            CKind::LineLine3 => vec![self.skew_sign() * self.args[2].num()],
            CKind::Angle3 => vec![self.args[2].num()],
            // two directions across a line as its hidden points stand now — the first line of a
            // parallel, the line a point is on — refreshed with every `refresh_consts`, so they
            // follow the line between solves
            CKind::Parallel3 => {
                let (e1, e2) = across(self.axis_dir(sk, 0));
                [e1, e2].concat()
            }
            CKind::PointOnLine3 | CKind::PointOnAxis | CKind::LineOnAxis | CKind::PlaneAxis => {
                let (e1, e2) = across(self.axis_dir(sk, 1));
                [e1, e2].concat()
            }
            CKind::PlaneDistance => vec![self.args[2].num()],
            // two points on the axis a drawing's extent apart, so its two rows weigh alike
            CKind::AxisOnPlane => vec![sk.extent().max(1.0)],
            // two directions across the first axis as it stands now, and a drawing's extent, so
            // its turn and its offset weigh alike
            CKind::AxisCoincident => {
                let (e1, e2) = across(self.axis_dir(sk, 0));
                [e1.to_vec(), e2.to_vec(), vec![sk.extent().max(1.0)]].concat()
            }
            // two directions across the plane's normal as it stands now
            CKind::AxisPerpendicularPlane => {
                let (e1, e2) = across(sk.basis(self.args[1].ent().i()).normal());
                [e1, e2].concat()
            }
            CKind::PlaneParallel => {
                let (e1, e2) = across(sk.basis(self.args[0].ent().i()).normal());
                [e1, e2].concat()
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
        let mut ps = self.params_on_row(sk, span);
        // a derivative reads the row's columns, a tangent column for each (the fixed zero where
        // it has none), and the ends of the line it is taken along, in space (§6.21)
        if let Some(d) = self.along {
            let tangents: Vec<u32> = ps.iter().map(|&p| sk.tangent_col(d, p)).collect();
            ps.extend(tangents);
            match sk.duals[d].toward {
                crate::model::Toward::Line(l) => ps.extend(lifted_ends(sk, l)),
                // the axis's direction where a line's ends go, from a held zero
                crate::model::Toward::Axis(a) => {
                    ps.extend([sk.zero_col(); 3]);
                    ps.extend(sk.axes[a].d);
                }
                crate::model::Toward::Chart { .. } => ps.extend([sk.zero_col(); 6]),
            }
        }
        ps
    }

    /// The columns the row reads as itself, before a derivative's — what a use's tangent
    /// columns are minted for (`Sketch::mint_tangents`).
    pub(crate) fn row_params(&self, sk: &Sketch) -> Vec<u32> {
        self.params_on_row(sk, None)
    }

    /// The constants the row reads as itself, before a derivative's.
    pub(crate) fn row_consts(&self, sk: &Sketch) -> Vec<f64> {
        self.own_consts_on(sk, None)
    }

    /// Whether this row may be stated as its derivative under dual `d` (§6.21) — `Ok(false)`
    /// where it reads nothing the use moves, so its derivative is nothing — or why it may not.
    /// The one rule the elaborator and a document reader share.  Asked before the row is added,
    /// so it reads entities, not columns: the dual's point, or geometry of the use's own that
    /// owns a number.
    pub fn differentiable(&self, sk: &Sketch, d: usize) -> Result<bool, String> {
        let dual = &sk.duals[d];
        if let crate::model::Toward::Line(l) = dual.toward {
            if !sk.line_ends(l).iter().all(|&e| sk.has_place(e)) {
                return Err("a line touching a set stands in space, or in a plane: this one is a \
                            2D sketch's"
                    .to_string());
            }
        }
        let what = || crate::model::article(&crate::syntax::snake(self.kind.name()));
        if self.kind.family_kernel().is_some()
            || self.kind.built()
            || !crate::taylor::has_form(self.kernel_id())
        {
            return Err(format!(
                "a set whose body states {} has no derivative yet, so nothing is tangent to it",
                what()
            ));
        }
        // everything the row reads, children and all
        let mut read = self.entities();
        let mut i = 0;
        while i < read.len() {
            let more = sk.children(read[i]);
            read.extend(more.into_iter().filter(|c| c.kind != EntKind::Curve));
            i += 1;
        }
        let point = EntRef::point(dual.point);
        // a point drawn in a view moves in it; a direction solved for in space is read where the
        // point stands, so a row over its place in the view would read it held
        if matches!(dual.toward, crate::model::Toward::Chart { .. })
            && !self.reads_space()
            && read.contains(&point)
            && sk.plane_of(dual.point).is_some()
        {
            return Err(format!(
                "a tangency at a point reads each set where the point stands in space, and {} \
                 reads it in its view",
                what()
            ));
        }
        let moves = |e: &EntRef| {
            *e == point || (dual.owned.contains(e) && !sk.own_params(*e).is_empty())
        };
        Ok(read.iter().any(moves))
    }

    /// The columns the row reads as itself — its own, and a free twin's after them.
    fn params_on_row(&self, sk: &Sketch, span: Option<usize>) -> Vec<u32> {
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
        match self.kind {
            CKind::Coincident | CKind::Distance => [pt(0), pt(1)].concat(),
            // the two points as its form reads them: in their view (or the one coordinate a
            // point's ordinate from its view's origin moves), then a line drawn there; in space,
            // their lifts and the direction as a segment, or one lift and the plane's frame
            CKind::Ordinate | CKind::Level => match self.form().unwrap_or(OrdinateForm::Space) {
                OrdinateForm::PageU | OrdinateForm::PageV => [pt(0), pt(1)].concat(),
                OrdinateForm::CoordU => vec![pt(1)[0]],
                OrdinateForm::CoordV => vec![pt(1)[1]],
                OrdinateForm::InView => [pt(0), pt(1), ln(2)].concat(),
                OrdinateForm::Space => {
                    let x = self.lifted_columns(sk);
                    [x[..6].to_vec(), self.direction_columns(sk, 2)].concat()
                }
                // the second point's lift, then the plane's origin and axes: along its normal the
                // plane is the direction, and along its `û` or `v̂` the one the first point is
                // the origin of
                f => {
                    let pl = match f {
                        OrdinateForm::FrameN => e(2).i(),
                        _ => sk.plane_of_origin(e(0).i()).expect("a frame form is from an origin"),
                    };
                    [self.lifted_columns(sk), plane_columns(sk, pl)].concat()
                }
            },
            CKind::Midpoint | CKind::PointOnLine | CKind::PointLineDistance => {
                [pt(0), ln(1)].concat()
            }
            CKind::DragTarget => pt(0),
            // a point in space: its own three numbers, which are where it stands
            CKind::DragSeen => {
                let p = &sk.points[e(0).i()];
                vec![p.x, p.y, p.z.expect("a point seen in space has its own z")]
            }
            CKind::Horizontal | CKind::Vertical => ln(0),
            CKind::Parallel
            | CKind::Perpendicular
            | CKind::Angle
            | CKind::ParallelDistance
            | CKind::EqualLength => [ln(0), ln(1)].concat(),
            CKind::EqualAngle => [ln(0), ln(1), ln(2), ln(3)].concat(),
            // every control point: the length reads the whole curve
            CKind::SplineLength => sk.entity_params(e(0)),
            // the curve's own number
            CKind::CurveLength => sk.curves[e(0).i()].length.into_iter().collect(),
            CKind::Stationary => crate::variational::columns(sk, self.id),
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
            // the two images, drawn in two fixed planes — the fold line is constants
            CKind::Project => [pt(0), pt(1)].concat(),
            // both images' hidden points, then both planes' axes
            CKind::ProjectSolved => {
                let axes = |i: usize| plane_columns(sk, e(i).i())[3..].to_vec();
                [self.lifted_columns(sk), axes(2), axes(3)].concat()
            }
            CKind::AxisUnit => sk.axes[e(0).i()].d.to_vec(),
            // the axis's place and direction, and the plane's origin and axes
            CKind::AxisOnPlane => {
                let r = &sk.axes[e(0).i()];
                [r.a.to_vec(), r.d.to_vec(), plane_columns(sk, e(1).i())].concat()
            }
            CKind::AxisParallelPlane | CKind::AxisPerpendicularPlane => {
                [sk.axes[e(0).i()].d.to_vec(), plane_columns(sk, e(1).i())[3..].to_vec()].concat()
            }
            // each plane's two axis directions
            CKind::PlaneParallel => {
                [plane_columns(sk, e(0).i())[3..].to_vec(), plane_columns(sk, e(1).i())[3..].to_vec()].concat()
            }
            // each axis's place and direction
            CKind::AxisCoincident => {
                let (a, b) = (&sk.axes[e(0).i()], &sk.axes[e(1).i()]);
                [a.a.to_vec(), a.d.to_vec(), b.a.to_vec(), b.d.to_vec()].concat()
            }
            // the first plane's origin and axes, then the second's origin
            CKind::PlaneDistance => {
                [plane_columns(sk, e(0).i()), sk.planes[e(1).i()].o.to_vec()].concat()
            }
            CKind::AxisFoot => {
                let r = &sk.axes[e(0).i()];
                [r.a, r.d].concat()
            }
            // the point's lift, then the axis's place and direction
            CKind::PointOnAxis => {
                let r = &sk.axes[e(1).i()];
                [self.lifted_columns(sk), r.a.to_vec(), r.d.to_vec()].concat()
            }
            // the line's ends lifted, then the axis's place and direction
            CKind::LineOnAxis => {
                let r = &sk.axes[e(1).i()];
                [self.lifted_columns(sk), r.a.to_vec(), r.d.to_vec()].concat()
            }
            // where the plane stands, in the point's place, and the axis's place and direction
            CKind::PlaneAxis => {
                let r = &sk.axes[e(1).i()];
                [sk.planes[e(0).i()].o.to_vec(), r.a.to_vec(), r.d.to_vec()].concat()
            }
            // the hidden point, the point as drawn, and its plane's origin and axes
            CKind::Lift => {
                let l = &sk.lifts[sk.lift_of(e(0).i()).expect("a lift's point has one")];
                [l.x.to_vec(), pt(0), plane_columns(sk, e(1).i())].concat()
            }
            // the hidden points, three columns each, in the order the operands name them
            CKind::Coincident3
            | CKind::Distance3
            | CKind::PointLine3
            | CKind::LineLine3
            | CKind::PointOnLine3
            | CKind::EqualLength3
            | CKind::Midpoint3
            | CKind::Symmetric3 => self.lifted_columns(sk),
            CKind::Angle3 | CKind::Perpendicular3 | CKind::Parallel3 => self.axis_columns(sk),
            // and the plane's origin and axes after them
            CKind::PointOnPlane | CKind::LineOnPlane => {
                [self.lifted_columns(sk), plane_columns(sk, e(1).i())].concat()
            }
            // the point and the centre in space, the radius, and the circle's plane's axes
            CKind::PointOnCircle3 => {
                let v = self.attitude_read(sk).expect("a circle in space is drawn in a plane");
                [self.lifted_columns(sk), vec![rad(1)], plane_columns(sk, v)[3..].to_vec()].concat()
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
        if !self.reads_space() {
            return Vec::new();
        }
        let e = |i: usize| self.args[i].ent();
        let ends = |i: usize| {
            let l = &sk.lines[e(i).i()];
            [l.p1 as usize, l.p2 as usize]
        };
        match self.kind {
            CKind::Coincident3 | CKind::Distance3 | CKind::ProjectSolved => {
                vec![e(0).i(), e(1).i()]
            }
            CKind::PointLine3 | CKind::PointOnLine3 | CKind::Midpoint3 => {
                [vec![e(0).i()], ends(1).to_vec()].concat()
            }
            CKind::PointOnPlane => vec![e(0).i()],
            // the two points, and a direction that is a drawn line, its ends; over a plane's
            // frame only the second, the first being the frame's origin
            CKind::Ordinate | CKind::Level => {
                if self.form().is_some_and(|f| f.of_frame()) {
                    return vec![e(1).i()];
                }
                let line = (e(2).kind == EntKind::Line).then(|| ends(2).to_vec());
                [vec![e(0).i(), e(1).i()], line.unwrap_or_default()].concat()
            }
            // an axis and a plane, and two planes: in space already
            CKind::AxisOnPlane
            | CKind::AxisCoincident
            | CKind::AxisParallelPlane
            | CKind::AxisPerpendicularPlane
            | CKind::PlaneParallel
            | CKind::PlaneDistance => Vec::new(),
            CKind::LineOnPlane | CKind::LineOnAxis => ends(0).to_vec(),
            CKind::PointOnCircle3 => vec![e(0).i(), sk.round_center(e(1))],
            CKind::PointOnExtrusion | CKind::PointOnAxis => vec![e(0).i()],
            CKind::Symmetric3 => [vec![e(0).i(), e(1).i()], ends(2).to_vec()].concat(),
            // a direction's operands: a line's ends are lifted, and an axis is in space already
            CKind::Angle3 | CKind::Perpendicular3 | CKind::Parallel3 => (0..2)
                .filter(|&i| e(i).kind == EntKind::Line)
                .flat_map(ends)
                .collect(),
            _ => [ends(0), ends(1)].concat(),
        }
    }

    /// The columns of a direction relation's two operands, six each as the kernels read a line
    /// (`B − A`): a line's two hidden points, or an axis's direction as the segment from the
    /// origin, `(0, d)` — so an axis needs no kernels of its own for `parallel`, `perpendicular`
    /// and `angle` (`Sketch::origin_param`).
    fn axis_columns(&self, sk: &Sketch) -> Vec<u32> {
        [self.direction_columns(sk, 0), self.direction_columns(sk, 1)].concat()
    }

    /// Operand `i` as a direction in space, six columns: an axis as the segment from the origin
    /// to its direction, a line as its two ends' hidden points.
    fn direction_columns(&self, sk: &Sketch, i: usize) -> Vec<u32> {
        let e = self.args[i].ent();
        if e.kind == EntKind::Axis {
            let z = sk.zero.expect("an axis mints the origin's Param");
            return [[z, z, z], sk.axes[e.i()].d].concat();
        }
        lifted_ends(sk, e.i())
    }

    /// The axes whose place this reads (`CKind::place_slots`), which `Sketch::add` frees and
    /// `Sketch::remove` holds again once nothing reads them.
    pub fn axes_placed_by(&self) -> impl Iterator<Item = usize> + '_ {
        self.kind.place_slots().iter().filter_map(|&i| match self.args.get(i)? {
            Arg::Ent(r) if r.kind == EntKind::Axis => Some(r.i()),
            _ => None,
        })
    }

    /// Whether every number this reads is held: then it moves nothing.
    pub fn reads_only_held(&self, sk: &Sketch) -> bool {
        self.params(sk).iter().all(|&p| sk.params[p as usize].fixed)
    }

    /// Operand `i`'s direction as it stands now, as a direction relation reads it.
    fn axis_dir(&self, sk: &Sketch, i: usize) -> [f64; 3] {
        let e = self.args[i].ent();
        if e.kind == EntKind::Axis {
            return sk.axes[e.i()].d.map(|p| sk.params[p as usize].value);
        }
        let l = &sk.lines[e.i()];
        crate::space::sub(sk.lifted(l.p2 as usize), sk.lifted(l.p1 as usize))
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
            CKind::Lift | CKind::PointOnPlane | CKind::LineOnPlane => Some(self.args[1].ent().i()),
            CKind::PointOnCircle3 => {
                sk.plane_of(sk.round_center(self.args[1].ent()))
            }
            _ => None,
        }
    }

    /// Every plane whose attitude decides which twin this statement is: `attitude_read`'s one,
    /// or a projection's two — which takes the solved twin when either of them is not fixed.
    pub fn attitudes_read(&self, sk: &Sketch) -> Vec<usize> {
        match self.kind {
            CKind::Project | CKind::ProjectSolved => {
                vec![self.args[2].ent().i(), self.args[3].ent().i()]
            }
            _ => self.attitude_read(sk).into_iter().collect(),
        }
    }

    /// The side a skew distance was stated on, as the sign its kernel's gap carries there: the
    /// kind's `sign` slot.
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
        let (r, _) = self.eval(sk, &v);
        crate::linalg::norm(&r)
    }

    pub fn residual(&self, sk: &Sketch, v: &[f64]) -> Vec<f64> {
        self.eval(sk, v).0
    }

    /// n_res x n_par, row-major.
    pub fn jacobian(&self, sk: &Sketch, v: &[f64]) -> Vec<f64> {
        self.eval(sk, v).1
    }

    /// The residual and Jacobian at `v`, by the kernel this row runs: its own, or its
    /// derivative's (§6.21).
    fn eval(&self, sk: &Sketch, v: &[f64]) -> (Vec<f64>, Vec<f64>) {
        kernels::eval_with(&self.kernel_in(sk), v, &self.consts(sk))
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
            let (x, y) = sk.on_view_sheet(sk.world_point(args[0].ent().i()), sk.curve_view(ci));
            sk.curve_nearest_by(ci, |px, py| (px - x).dhypot(py - y))
        }
        (CKind::CurveCurvature, 2) => {
            let (cx, cy) = sk.point_xy(sk.round_center(args[1].ent()));
            sk.curve_nearest_by(args[0].ent().i(), |px, py| (px - cx).dhypot(py - cy))
        }
        // a point on a curve starts where the curve passes nearest it
        (CKind::PointOnCurve, 2) => {
            let (x, y) = sk.point_xy(args[0].ent().i());
            sk.curve_nearest_by(args[1].ent().i(), |px, py| (px - x).dhypot(py - y))
        }
        // an osculating circle sits centred a radius off the curve, so the curve point nearest
        // the centre it already has is the place it is asking about
        (CKind::SplineCurvature, 2) => {
            let (cx, cy) = sk.point_xy(sk.round_center(args[1].ent()));
            crate::curve::closest(sk, args[0].ent().i(), cx, cy).0
        }
        _ => 0.0,
    }
}

/// An entity slot the core fills when the caller leaves it out — the entity counterpart of
/// `seed_param`: a projection's planes are its points' memberships, and nobody writes them.
/// `Err` is the reason it cannot, in the words the caller reports, naming entities by `name`.
pub fn infer_entity(
    sk: &Sketch,
    kind: CKind,
    args: &[Arg],
    i: usize,
    name: &dyn Fn(EntRef) -> String,
) -> Result<EntRef, String> {
    match (kind, i) {
        (CKind::Project | CKind::ProjectSolved, 2 | 3) => {
            let p = args[i - 2].ent();
            sk.plane_of(p.i()).map(EntRef::plane).ok_or_else(|| {
                format!(
                    "{} is on no plane, so `project` cannot say which view it is in",
                    name(p)
                )
            })
        }
        // an ordinate's direction said in a word: an axis of the view both points are drawn in.
        // The plane's own words never reach here from a document — the elaborator names the
        // plane's axis — and from a record they are a plane's, with no plane to read them off
        (CKind::Ordinate | CKind::Level, 2) => {
            let w = ordinate_word(kind, args);
            let toward = Toward::of(w).ok_or_else(|| {
                format!("an ordinate names its direction: an axis, a line, or one of {}",
                    crate::syntax::one_of(&ALONG_WORDS))
            })?;
            let (p, q) = (args[0].ent(), args[1].ent());
            match (toward, shared_view(sk, p.i(), q.i())) {
                (Toward::PageU, Some(v)) => Ok(EntRef::axis(sk.planes[v].u as usize)),
                (Toward::PageV, Some(v)) => Ok(EntRef::axis(sk.planes[v].v as usize)),
                (Toward::PageU | Toward::PageV, None) => Err(format!(
                    "`{w}` is a direction of the view both points are drawn in, and {} and {} \
                     are not drawn in one: name the axis",
                    name(p),
                    name(q)
                )),
                _ => Err(format!("`{w}` is a direction of a plane, which the ordinate is \
                    measured from: `{} distance(d, along: {w}) P`", name(q))),
            }
        }
        _ => Err(format!("{} leaves nothing for the core to infer in slot {i}", kind.name())),
    }
}

/// What an ordinate (`Ordinate`, `Level`) must be to mean something — `validate`'s arm for the
/// two.  A plane's own word is measured from that plane's origin along its own direction; a
/// level of a point with itself, and an ordinate of two points of one view along that view's
/// normal, are identically nothing; a reading in space needs its points' places, and a drawn
/// line a length.
fn validate_ordinate(
    sk: &Sketch,
    kind: CKind,
    args: &[Arg],
    name: &dyn Fn(EntRef) -> String,
) -> Result<(), String> {
    let (p, q, t) = (args[0].ent(), args[1].ent(), args[2].ent());
    let word = ordinate_word(kind, args);
    if let Some(toward) = Toward::of(word).filter(|t| t.of_plane()) {
        let datum = sk.plane_of_origin(p.i());
        let on = datum.is_some_and(|pl| match toward {
            Toward::PlaneU => t == EntRef::axis(sk.planes[pl].u as usize),
            Toward::PlaneV => t == EntRef::axis(sk.planes[pl].v as usize),
            _ => t == EntRef::plane(pl),
        });
        if !on {
            return Err(format!(
                "`along: {word}` is a plane's own direction, measured from its origin: \
                 `{} distance(d, along: {word}) P`",
                name(q)
            ));
        }
    }
    if p == q {
        return Err(format!("{} is level with itself along every direction", name(p)));
    }
    let form = ordinate_form(sk, p.i(), q.i(), t, word);
    let mut a = args.to_vec();
    if let Some(i) = kind.form_slot() {
        a[i] = Arg::Int(form as i64);
    }
    let c = Constraint::new(kind, a);
    for e in c.lifted_points(sk) {
        if !sk.has_place(e) {
            return Err(format!(
                "{} is a point of a 2D sketch, with no place in space to relate",
                name(EntRef::point(e))
            ));
        }
    }
    if t.kind == EntKind::Line {
        let ln = &sk.lines[t.i()];
        let [a, b] = [ln.p1, ln.p2].map(|e| sk.world_point(e as usize));
        if crate::space::norm(crate::space::sub(b, a)) <= kernels::MIN_LINE_LEN {
            return Err(format!("{} has no length to measure along", name(t)));
        }
    }
    // two points of one view, along its normal: the row is identically zero.  An axis held
    // square to a view held still is the same statement by value, which the count of rows
    // cannot see
    if let Some(v) = shared_view(sk, p.i(), q.i()) {
        let square = match t.kind {
            EntKind::Plane => t.i() == v,
            EntKind::Axis => {
                let d = &sk.axes[t.i()].d;
                let held = d.iter().all(|&k| sk.params[k as usize].fixed) && sk.plane_fixed(v);
                let dir = d.map(|k| sk.params[k as usize].value);
                let n = sk.basis(v).normal();
                held && crate::space::norm(crate::space::cross(dir, n))
                    <= 1e-9 * crate::space::norm(dir).max(1e-300)
            }
            _ => false,
        };
        if square {
            return Err(format!(
                "{} and {} are drawn in {}, so how far apart they stand along its normal is not \
                 a question: every point of a view is on it",
                name(p),
                name(q),
                sk.plane_name(v)
            ));
        }
    }
    Ok(())
}

/// The curve a contact's parameter runs along — a spline or a language curve, whichever the
/// contact names — and `None` for a kind whose own unknown runs along nothing.
pub(crate) fn contact_carrier(kind: CKind, args: &[Arg]) -> Option<EntRef> {
    let (e, _) = kind.contact_on(SpecKind::Spline).or_else(|| kind.contact_on(SpecKind::Curve))?;
    Some(args[e].ent())
}

/// A shared parameter (`t == s`) is **one place along one curve**: every contact owning it
/// stands on the same curve, so the unknown has one interval, one seam and one speed.  Two
/// curves could not agree on any of the three, and a contact's own unknown that runs along no
/// curve has nothing to share.
fn shared_on_one_curve(
    sk: &Sketch,
    kind: CKind,
    args: &[Arg],
    named: &dyn Fn(EntRef) -> String,
) -> Result<(), String> {
    let Some(name) = args.iter().find_map(|a| match a {
        Arg::Shared { name, .. } => Some(name),
        _ => None,
    }) else {
        return Ok(());
    };
    let Some(here) = contact_carrier(kind, args) else {
        return Err(format!("`{name}` names a contact's place along a curve, and this \
            relation's unknown runs along none"));
    };
    if sk.free_vars.get(name).is_some_and(|&p| !sk.params[p as usize].fixed) {
        return Err(format!("`{name}` is a free variable a dimension reads, and a place along a \
            curve is another unknown"));
    }
    match sk.shared.get(name) {
        Some(place) if place.along != here => Err(format!(
            "`{name}` is a place along {}, and a contact on {} cannot share it",
            named(place.along),
            named(here)
        )),
        _ => Ok(()),
    }
}

/// What a kind refuses once its arguments are all in — the checks that need the sketch, which
/// the type check on the spec cannot make.  One rule for the elaborator, the document readers,
/// the FFI and the Rust constructors alike; `name` is what the caller calls an entity.
pub fn validate(
    sk: &Sketch,
    kind: CKind,
    args: &[Arg],
    name: &dyn Fn(EntRef) -> String,
) -> Result<(), String> {
    shared_on_one_curve(sk, kind, args, name)?;
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
                        name(args[0].ent())
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
                    name(pa)
                ));
            }
            // two fixed planes are parallel or not now and for ever; where either is solved,
            // whether they came out parallel is a question for after the solve (E065)
            let solved = |e: EntRef| !sk.plane_fixed(e.i());
            let basis = |e: EntRef| sk.basis(e.i());
            if !solved(pa) && !solved(pb)
                && crate::plane::fold_line(&basis(pa), &basis(pb)).is_none()
            {
                return Err(format!(
                    "{} and {} are parallel, so no fold line relates their views",
                    name(pa),
                    name(pb)
                ));
            }
            Ok(())
        }
        CKind::Ordinate | CKind::Level => validate_ordinate(sk, kind, args, name),
        k if k.spatial() => {
            let c = Constraint::new(kind, args.to_vec());
            for p in c.lifted_points(sk) {
                if !sk.has_place(p) {
                    return Err(format!(
                        "{} is a point of a 2D sketch, with no place in space to relate",
                        name(EntRef::point(p))
                    ));
                }
            }
            if let Some((i, _, _)) = c.dimensions().first() {
                if k.magnitude() && matches!(args[*i], Arg::Num(v) if v < 0.0) {
                    return Err("a distance in space is a magnitude, and is never negative".into());
                }
            }
            // a line in space needs a direction, and two of them a common perpendicular
            let lines: Vec<EntRef> =
                c.entities().into_iter().filter(|e| e.kind == EntKind::Line).collect();
            for &l in &lines {
                let ln = &sk.lines[l.i()];
                let [a, b] = [ln.p1, ln.p2].map(|p| sk.world_point(p as usize));
                if crate::space::norm(crate::space::sub(b, a)) <= kernels::MIN_LINE_LEN {
                    return Err(format!("{} has no length in space", name(l)));
                }
            }
            // a view's own points are on it by construction: the row would be identically zero
            let own = |e: EntRef| {
                c.lifted_points(sk).iter().all(|&p| sk.plane_of(p) == Some(e.i()))
            };
            if matches!(k, CKind::PointOnPlane | CKind::LineOnPlane)
                && own(args[1].ent())
            {
                return Err(format!(
                    "{} is drawn in {}, so where it stands along that view's normal is not a \
                     question: every point of a view is on it",
                    name(args[0].ent()),
                    name(args[1].ent())
                ));
            }
            if k == CKind::LineLine3
                && skew_seed(sk, lines[0], lines[1]).is_none()
            {
                return Err(format!(
                    "{} and {} are parallel in space, and parallel lines have no common \
                     perpendicular to measure",
                    name(lines[0]),
                    name(lines[1])
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
    if kind.form_slot() == Some(i) {
        let ent = |j: usize| match args.get(j) {
            Some(Arg::Ent(e)) => Some(*e),
            _ => None,
        };
        let (p, q, t) = (ent(0)?, ent(1)?, ent(2)?);
        let word = ordinate_word(kind, args);
        return Some(Arg::Int(ordinate_form(sk, p.i(), q.i(), t, word) as i64));
    }
    match (kind, i) {
        (CKind::LineLine3, 3) => {
            skew_seed(sk, args[0].ent(), args[1].ent()).map(|s| Arg::Int(s as i64))
        }
        _ => None,
    }
}

/// A plane's columns as every kernel that reads one takes them: its origin, then its two axes'
/// directions — nine Params, held or not.
fn plane_columns(sk: &Sketch, p: usize) -> Vec<u32> {
    let pl = &sk.planes[p];
    [pl.o, sk.axes[pl.u as usize].d, sk.axes[pl.v as usize].d].concat()
}

/// The world length one unit of a hidden unknown is worth — see `Param::scale`.  Read off the
/// geometry at the moment the constraint is added; it preconditions the step, so an estimate
/// that drifts as the sketch moves costs convergence rate, never correctness.
pub fn param_scale(sk: &Sketch, kind: CKind, args: &[Arg], i: usize) -> f64 {
    // Whichever family the contact runs along, the question is the same one, so it is asked once
    // and answered by the entity the slot actually names.  A hidden unknown that runs along
    // nothing is a length already.
    match kind.contact_slots().or_else(|| kind.contact_on(SpecKind::Curve)) {
        Some((e, t)) if t == i => contact_speed(sk, args[e].ent()),
        _ => 1.0,
    }
}

/// The world length one unit of a contact's parameter is worth, whichever family it runs along
/// — the one answer, so the seed `Sketch::add` records and the scale `System::new` compiles
/// against cannot come from two different rules.
pub fn contact_speed(sk: &Sketch, e: EntRef) -> f64 {
    match e.kind {
        // a free curve runs over [0, 1] at its length's speed; another curve's parameter is its own
        EntKind::Curve => sk.curves[e.i()].length.map_or(1.0, |l| sk.params[l as usize].value.abs().max(1e-9)),
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
    // a row and its derivative are two equations about the same entities
    if a.kind != b.kind || a.along != b.along {
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
        SpecKind::Axis => ent == EntKind::Axis,
        SpecKind::Direction => ent == EntKind::Line || ent == EntKind::Axis,
        SpecKind::Along => matches!(ent, EntKind::Line | EntKind::Axis | EntKind::Plane),
        _ => false,
    }
}
