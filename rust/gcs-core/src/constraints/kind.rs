//! The constraint kinds: `CKind`, the slot kinds of a spec, and the per-kind tables.

use super::*;

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
    /// **A free curve pressed onto a held line where it chooses** (`rope touches floor`, #149):
    /// a corner on the line, the curve two arcs there, its place along the line where no force
    /// runs along it (frictionless).  Part of the curve's problem, like a peg — a slide
    /// (`extremal::Stop`): one more unknown and one more row inside the curve's solve, so it
    /// compiles no row of the drawing's (`Sketch::settle_variational`).
    CurveTouchesLine,
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
    /// **A point at a fraction of a line's run** (`m coincident(t == 0.25) g`, #248): `m = p1 +
    /// t·(p2 − p1)`, the contact owning its place `t` as a spline's does — unpinned, an unknown
    /// seeded where the point stands (two rows against one column, the net one equation a point
    /// on a line is worth); pinned, the place.  Like `coincident` on a line, it runs on past
    /// either end: `t == 1.5` is the point half a run beyond `p2`.  Written only where `t` is:
    /// `m coincident g` stays `PointOnLine` and pays no column.
    PointAlongLine,
    /// **A point at a fraction of an arc's turn** (#248): `t` of the way round from `start` to
    /// `end`, counter-clockwise, by angle — on the arc and nowhere else, so an unpinned `t` is
    /// held to `[0, 1]` as a spline's is to its knots (`curve::clamp_contacts`), and a pinned one
    /// outside it is refused (`validate`).  A circle has no start, so no fraction.
    PointAlongArc,
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
    /// **A point in space on an envelope** (#283, `envelope::held`): `p coincident flank`, the
    /// envelope of a surface of revolution under a motion in space.  Four rows — the point where
    /// the moving source is at its place `[u, v, t]`, and the meshing equation there — against
    /// three owned unknowns, the net one equation a surface is worth; the source's geometry and
    /// the motion's axes are columns, so what makes the source solves from where the point is.
    PointOnEnvelope,
    /// The tangent plane of an envelope at a point a contact holds on it (#283): `flank
    /// tangent(at: p) l`, the line (or axis) running in it.  One row, the line's direction square
    /// to the normal, over **that contact's place** (`Sketch::add`, as `CurveTangentAt` shares a
    /// curve contact's).
    EnvelopeTangentAt,
    /// The normal of an envelope at a point a contact holds on it (#283): `flank perpendicular(at:
    /// p) l`, the line (or axis) along it — the common normal of a mesh.  Two rows over the
    /// contact's place: the direction square to two directions of the tangent plane.
    EnvelopeNormalAt,
    /// A plane touching an envelope at a point a contact holds on it (#283): `flank tangent(at: p)
    /// P`, its normal along the envelope's.  `EnvelopeNormalAt`'s two rows over the plane's axes.
    /// Says nothing of where the plane stands: `p coincident P` does.
    EnvelopePlaneAt,
    /// **A circle osculating an envelope** at a point a contact holds on it (#283): `flank
    /// curvature(at: p) k`, the circle the osculating circle of the envelope's section by its
    /// plane at `p` — `p` in that plane, drawn there or stated.  `CurveCurvature`'s three rows over
    /// the contact's place: the centre the section's centre of curvature (two rows, in the plane,
    /// by Meusnier from the normal curvature, which the envelope's shape operator gives with no
    /// second solve), and the radius its distance.
    EnvelopeCurvatureAt,
    /// **A point on the path of contact of two flanks** (#283 step 3, `envelope::meshing`): `p
    /// coincident contact`, `contact := meshing(pinion_flank, gear_flank, under: mesh, …)`.  It
    /// owns the pinion's phase `t`, the gear's lag `lag` (the transmission error, as a roll of the
    /// mesh) and a place on each flank `[u, v, t]`, eight unknowns against ten rows — the point
    /// where the pinion's flank is at its phase, the two flanks sharing that point and their
    /// normal there, and each flank's own row (a generated flank's meshing equation, a formed
    /// one's roll held at none) — so the point is on a curve in space, net two.  The phase and
    /// the lag are what a pin states (`p coincident(t == 0deg, lag == 0deg) contact`); both
    /// flanks' sources and motions and the mesh's axes are columns, so what makes a flank solves
    /// from where the contact is and how late the gear is there.
    PointOnMeshing,
    /// **The common normal of two flanks** at a point a contact holds on their meshing (#345):
    /// `contact perpendicular(at: q) l`, the line (or axis) along it.  `EnvelopeNormalAt`'s two
    /// rows, over **that contact's** phase, lag and places, so it is read at the contact's phase,
    /// not only at the drawn pose.
    MeshingNormalAt,
    /// **The path of contact** at a point a contact holds on a meshing (#345): `contact
    /// tangent(at: q) l`, the line (or axis) along the contact point's derivative along the phase
    /// — by the implicit function theorem, one solve with the meshing rows' Jacobian in the lag
    /// and the places.  Two rows over that contact's unknowns; at a line contact the path has no
    /// direction and the rows are not finite.
    MeshingTangentAt,
    /// **A line reciprocal to a motion's twist** (#326, `envelope::screw`): `l reciprocal mesh`, a
    /// force along `l` doing no work on the motion — `d · v(p) = 0`, `d` the line's direction and
    /// `v` the twist's velocity at a point of it, the same at every point.  One row; the line's
    /// ends and the motion's axes are columns.  For the mesh of two members it is the meshing law
    /// of a line of action, the members moving as fast along it.
    Reciprocal,
    /// **An axis on a motion's instantaneous screw axis** (#326, `envelope::screw`): `isa screw
    /// mesh`, the axis along the twist's `ω` through the point every other moves past along it,
    /// and its own unknown `pitch` that motion's pitch, a length a radian.  Five rows (the
    /// direction's two and the place's, then the pitch) over the axis, its pitch and the motion's
    /// axes.
    ScrewAxis,
    /// **A point as a moving frame sees it** (#326, R4, `envelope::screw`): `X coincident(under:
    /// m, t == τ) x`, `X` the point `x` carried by the motion's pose at the roll `τ` — `X =
    /// Pose(τ)·x`, three rows over both points' lifts, the roll (the relation's own unknown, in
    /// degrees, pinned or solved) and the motion's axes.  In a traced body, `τ` an expression in
    /// the running formal: a contact seen from a turning member.
    CarriedPoint,
    /// **A generated flank regular within its blank** (#345 S3, `envelope::regular`): `flank
    /// regular(>= m, by: tool) within blank`, the least area factor the flank — a held envelope,
    /// its source a face of `tool` — takes over the part of it the tool cuts within `blank`,
    /// found by search.  One row, the factor less the number (a ratio, nothing at a fold); a bound
    /// as written, an equation where the number is stated outright.  The envelope's columns and
    /// every point, axis and unknown of the two regions' terms are columns.
    RegularWithin,
    /// **A point on a body traced in space** (#326, `traced`): `p coincident k`, three rows `p −
    /// X(u, s)` against the place `[u, s]` it owns (a curve's `s` held at none), so a point on a
    /// curve in space nets two and one on a surface one.  The body's geometry is columns.
    PointOnTraced,
    /// A line along a traced curve's tangent at a point a contact holds on it: `k tangent(at: p)
    /// l`, two rows over that contact's place.
    TracedTangentAt,
    /// A line in a traced surface's tangent plane there: `S tangent(at: p) l`, one row.
    TracedSurfaceTangentAt,
    /// A line along a traced surface's normal there: `S perpendicular(at: p) l`, two rows — the
    /// normal `C_φ × C_s`, exact by the implicit function theorem.
    TracedNormalAt,
    /// A plane touching a traced surface there: `S tangent(at: p) P`, two rows.
    TracedPlaneAt,
    /// A circle osculating a traced surface's section by the circle's plane there: `S
    /// curvature(at: p) k`, three rows, Meusnier from the second derivatives — exact, each a
    /// polarised Taylor series.
    TracedCurvatureAt,
    /// **The tangent at a point already on a spline** (#251), `g tangent(at: m) l`: the line runs
    /// along `C'(t)`, where `t` is the place of the contact holding `m` on the spline — **that
    /// contact's own unknown**, shared rather than a second one (`Sketch::add`, #70's shared
    /// place). One row, `SplineTangentLine`'s direction row alone, regular wherever `C' ≠ 0`:
    /// beside the contact, the two-row tangency restates the point and is a double root. The
    /// point is a slot so the statement names it, a graft keeps it and the contact is found.
    SplineTangentAt,
    /// The same over a curve written in the language: `CurveTangentLine`'s direction row, its
    /// kernel the definition's (`FamilyKernel::TangentAt`).
    CurveTangentAt,
    /// And of a circle or an arc, at any point of it a relation holds there (#251): the radius
    /// to `at` square to the line, `TangentArcLine`'s kernel with the point named rather than
    /// chosen by `start`/`end` or `p1`/`p2`. No place to share: the point is on the round already.
    TangentCircleAt,
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
    /// `PointAlongLine` across views: the point's hidden point at the fraction `t` of the way
    /// between the line's ends' (#248).  Three rows against the one column.
    PointAlongLine3,
    /// `PointAlongArc` across views: the point's hidden point at the fraction `t` of the arc's
    /// turn, read in the plane the arc is drawn in and stood in space there (#248).
    PointAlongArc3,
    /// A line touching a circle or an arc at a point of it, the line drawn out of the round's
    /// plane (#363 S4): its direction square to the radius to `at` and in the round's plane — two
    /// rows over the hidden points, `TangentCircleAt`'s and a chain joint's (`TangentArcLine`) in
    /// space.  What makes a straight between bends in two planes one link of a chain.
    TangentCircleAt3,
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
    /// **A point carried by an affine map** (#357): `q mapped(from -> a, to -> b, third -> c)
    /// p`, `q = a + i·(b − a) + j·(c − a)` with `(i, j)` the place of `p` in the frame `from`,
    /// `to`, `third`.  Each side is read in its own plane, the targets in theirs and the source
    /// in its, so a profile drawn in one view is carried into another; the source points are
    /// columns, so the place is exact wherever the frame stands.  Two rows, degree 1, no number:
    /// not a dimension.
    Mapped,
    /// The same with two pairs (`q mapped(from -> a, to -> b) p`): the similarity taking `from`
    /// to `a` and `to` to `b`, keeping the turn — or, `mirror: true`, reversing it.
    MappedSimilar,
    /// A circle carried by the similarity of two pairs (`k₂ mapped(from -> a, to -> b) k`): its
    /// centre carried as a point is, and its radius scaled by `|b − a| / |to − from|` — what a
    /// face carried whole makes of its circles (#357).  Three rows; under three pairs a circle
    /// is an ellipse, refused where it is written.
    MappedCircle,
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

/// A map's sides as slots (`CKind::map_sides`): each the thing carried (first) and the frame's
/// points, in spec order.
#[derive(Clone, Copy, Debug)]
pub struct MapSides {
    pub source: &'static [usize],
    pub target: &'static [usize],
}

impl MapSides {
    /// The source frame's points, without the thing carried from it.
    pub fn frame(self) -> &'static [usize] {
        &self.source[1..]
    }
}

/// Which of a curve definition's kernels a kind runs through: the table holds these five per
/// definition, in this order (`System::kernel_table`), and `Constraint::kernel_id_in` counts
/// them so — the one statement of what "a kind whose kernel is per definition" means.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FamilyKernel {
    Contact,
    Tangent,
    Curvature,
    /// A point in space on the surface the curve stands for (`CKind::PointOnExtrusion`).
    Extrusion,
    /// The tangent at a point a contact already holds on the curve (`CKind::CurveTangentAt`).
    TangentAt,
}

impl FamilyKernel {
    pub const ALL: [FamilyKernel; 5] = [
        FamilyKernel::Contact,
        FamilyKernel::Tangent,
        FamilyKernel::Curvature,
        FamilyKernel::Extrusion,
        FamilyKernel::TangentAt,
    ];

    /// Rows per constraint — a fact about the kind, not asked of a kernel it may not have yet.
    pub fn n_res(self) -> usize {
        match self {
            FamilyKernel::TangentAt => 1,
            FamilyKernel::Contact | FamilyKernel::Tangent | FamilyKernel::Extrusion => 2,
            FamilyKernel::Curvature => 3,
        }
    }
}

/// Every concrete constraint type, in the order the registry lists them.
pub const ALL_KINDS: [CKind; 95] = [
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
    CKind::PointOnEnvelope,
    CKind::EnvelopeTangentAt,
    CKind::EnvelopeNormalAt,
    CKind::EnvelopePlaneAt,
    CKind::EnvelopeCurvatureAt,
    CKind::PointOnMeshing,
    CKind::MeshingNormalAt,
    CKind::MeshingTangentAt,
    CKind::Reciprocal,
    CKind::ScrewAxis,
    CKind::CarriedPoint,
    CKind::RegularWithin,
    CKind::PointOnTraced,
    CKind::TracedTangentAt,
    CKind::TracedSurfaceTangentAt,
    CKind::TracedNormalAt,
    CKind::TracedPlaneAt,
    CKind::TracedCurvatureAt,
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
    CKind::CurveTouchesLine,
    CKind::SplineTangentAt,
    CKind::CurveTangentAt,
    CKind::TangentCircleAt,
    CKind::PointAlongLine,
    CKind::PointAlongArc,
    CKind::PointAlongLine3,
    CKind::PointAlongArc3,
    CKind::Mapped,
    CKind::MappedSimilar,
    CKind::MappedCircle,
    CKind::TangentCircleAt3,
];

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
    /// An envelope in space a contact holds to (#283): its source and motion read as columns.
    Envelope,
    /// The meshing of two flanks a point is held to (#283): both flanks and the mesh read as
    /// columns.
    Meshing,
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
    /// A motion (#326): a relation reading its twist takes its axes as columns.
    Motion,
    /// A body traced in space (#326).
    Traced,
    /// A solid (#345): a region whose terms a relation reads as columns — a tool, a blank.
    Solid,
    /// A number with no unit, a ratio (#345): an area factor.  A dimension, as a length and an
    /// angle are, of no dimension.
    Ratio,
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
            SpecKind::Ratio => Dim::SCALAR,
            SpecKind::Point
            | SpecKind::Line
            | SpecKind::Circle
            | SpecKind::Arc
            | SpecKind::CircleOrArc
            | SpecKind::Spline
            | SpecKind::Curve
            | SpecKind::Envelope
            | SpecKind::Meshing
            | SpecKind::Motion
            | SpecKind::Traced
            | SpecKind::Solid
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
                | SpecKind::Envelope
                | SpecKind::Meshing
                | SpecKind::Motion
                | SpecKind::Traced
                | SpecKind::Solid
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
        matches!(self, SpecKind::Length | SpecKind::Angle | SpecKind::Ratio)
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
            SpecKind::Envelope => "envelope",
            SpecKind::Meshing => "meshing",
            SpecKind::Motion => "motion",
            SpecKind::Traced => "traced",
            SpecKind::Solid => "solid",
            SpecKind::Ratio => "ratio",
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

pub(super) type Spec = &'static [(&'static str, SpecKind)];

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
            CKind::CurveTouchesLine => "CurveTouchesLine",
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
            CKind::PointAlongLine => "PointAlongLine",
            CKind::PointAlongArc => "PointAlongArc",
            CKind::PointAlongLine3 => "PointAlongLine3",
            CKind::PointAlongArc3 => "PointAlongArc3",
            CKind::TangentCircleAt3 => "TangentCircleAt3",
            CKind::Ordinate => "Ordinate",
            CKind::Level => "Level",
            CKind::PointOnCurve => "PointOnCurve",
            CKind::PointOnExtrusion => "PointOnExtrusion",
            CKind::CurveTangentLine => "CurveTangentLine",
            CKind::SplineTangentAt => "SplineTangentAt",
            CKind::CurveTangentAt => "CurveTangentAt",
            CKind::TangentCircleAt => "TangentCircleAt",
            CKind::CurveCurvature => "CurveCurvature",
            CKind::PointOnEnvelope => "PointOnEnvelope",
            CKind::EnvelopeTangentAt => "EnvelopeTangentAt",
            CKind::EnvelopeNormalAt => "EnvelopeNormalAt",
            CKind::EnvelopePlaneAt => "EnvelopePlaneAt",
            CKind::EnvelopeCurvatureAt => "EnvelopeCurvatureAt",
            CKind::PointOnMeshing => "PointOnMeshing",
            CKind::MeshingNormalAt => "MeshingNormalAt",
            CKind::MeshingTangentAt => "MeshingTangentAt",
            CKind::Reciprocal => "Reciprocal",
            CKind::ScrewAxis => "ScrewAxis",
            CKind::CarriedPoint => "CarriedPoint",
            CKind::RegularWithin => "RegularWithin",
            CKind::PointOnTraced => "PointOnTraced",
            CKind::TracedTangentAt => "TracedTangentAt",
            CKind::TracedSurfaceTangentAt => "TracedSurfaceTangentAt",
            CKind::TracedNormalAt => "TracedNormalAt",
            CKind::TracedPlaneAt => "TracedPlaneAt",
            CKind::TracedCurvatureAt => "TracedCurvatureAt",
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
            CKind::Mapped => "Mapped",
            CKind::MappedSimilar => "MappedSimilar",
            CKind::MappedCircle => "MappedCircle",
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

    /// Whether this kind's constants are read off the pose — directions across a line or a
    /// plane's normal as they stand, a fold line (`Constraint::consts_on`) — and so go stale as
    /// a solve turns what they were read from (`refresh_consts`, #217).
    pub fn reads_pose(self) -> bool {
        matches!(
            self,
            CKind::Project
                | CKind::Parallel3
                | CKind::PointOnLine3
                | CKind::PointOnAxis
                | CKind::LineOnAxis
                | CKind::PlaneAxis
                | CKind::AxisCoincident
                | CKind::AxisPerpendicularPlane
                | CKind::PlaneParallel
        )
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
                ("depth", S::Float),
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
            CKind::CurveTouchesLine => &[("curve", S::Curve), ("line", S::Line)],
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
            // the fraction is the contact's own unknown, as a spline's parameter is
            CKind::PointAlongLine | CKind::PointAlongLine3 => {
                &[("p", S::Point), ("line", S::Line), ("t", S::Param)]
            }
            CKind::PointAlongArc | CKind::PointAlongArc3 => {
                &[("p", S::Point), ("arc", S::Arc), ("t", S::Param)]
            }
            CKind::PointOnCurve | CKind::PointOnExtrusion => &[("p", S::Point), ("curve", S::Curve), ("t", S::Param)],
            CKind::CurveTangentLine => &[("curve", S::Curve), ("line", S::Line), ("t", S::Param)],
            // the point named, then the place: the contact's, which `Sketch::add` hands it
            CKind::SplineTangentAt => {
                &[("spline", S::Spline), ("line", S::Line), ("at", S::Point), ("t", S::Param)]
            }
            CKind::CurveTangentAt => {
                &[("curve", S::Curve), ("line", S::Line), ("at", S::Point), ("t", S::Param)]
            }
            CKind::TangentCircleAt | CKind::TangentCircleAt3 => {
                &[("circle", S::CircleOrArc), ("line", S::Line), ("at", S::Point)]
            }
            CKind::CurveCurvature => {
                &[("curve", S::Curve), ("circle", S::CircleOrArc), ("t", S::Param)]
            }
            // the place on the envelope: along the source's meridian, round its axis, the roll
            CKind::PointOnEnvelope => &[
                ("p", S::Point), ("envelope", S::Envelope), ("u", S::Param), ("v", S::Param), ("t", S::Param),
            ],
            // the point named, then the place: the contact's, which `Sketch::add` hands it
            CKind::EnvelopeTangentAt | CKind::EnvelopeNormalAt => &[
                ("envelope", S::Envelope), ("line", S::Direction), ("at", S::Point),
                ("u", S::Param), ("v", S::Param), ("t", S::Param),
            ],
            CKind::EnvelopePlaneAt => &[
                ("envelope", S::Envelope), ("plane", S::Plane), ("at", S::Point),
                ("u", S::Param), ("v", S::Param), ("t", S::Param),
            ],
            CKind::EnvelopeCurvatureAt => &[
                ("envelope", S::Envelope), ("circle", S::CircleOrArc), ("at", S::Point),
                ("u", S::Param), ("v", S::Param), ("t", S::Param),
            ],
            // the pinion's phase and the gear's lag, then the place on each flank
            CKind::PointOnMeshing => &[
                ("p", S::Point), ("meshing", S::Meshing), ("t", S::Param), ("lag", S::Param),
                ("u1", S::Param), ("v1", S::Param), ("t1", S::Param),
                ("u2", S::Param), ("v2", S::Param), ("t2", S::Param),
            ],
            // the point named, then its contact's unknowns, which `Sketch::add` hands it (#345)
            CKind::MeshingNormalAt | CKind::MeshingTangentAt => &[
                ("meshing", S::Meshing), ("line", S::Direction), ("at", S::Point),
                ("t", S::Param), ("lag", S::Param),
                ("u1", S::Param), ("v1", S::Param), ("t1", S::Param),
                ("u2", S::Param), ("v2", S::Param), ("t2", S::Param),
            ],
            // a line and the motion whose twist it is reciprocal to (#326)
            CKind::Reciprocal => &[("line", S::Line), ("motion", S::Motion)],
            // the axis, the motion, and the pitch: the relation's own unknown
            CKind::ScrewAxis => &[("axis", S::Axis), ("motion", S::Motion), ("pitch", S::Param)],
            // the point carried, the point it is carried from, the motion and the roll
            CKind::CarriedPoint => &[("p", S::Point), ("q", S::Point), ("under", S::Motion), ("t", S::Param)],
            // the flank, the blank it is cut within, the tool whose face its source is, and the
            // least area factor (#345)
            CKind::RegularWithin => &[
                ("flank", S::Envelope), ("blank", S::Solid), ("by", S::Solid), ("margin", S::Ratio),
            ],
            // the place on a traced body: its formals', a curve's second held at none (#326)
            CKind::PointOnTraced => &[("p", S::Point), ("traced", S::Traced), ("u", S::Param), ("s", S::Param)],
            CKind::TracedTangentAt | CKind::TracedSurfaceTangentAt | CKind::TracedNormalAt => &[
                ("traced", S::Traced), ("line", S::Direction), ("at", S::Point), ("u", S::Param), ("s", S::Param),
            ],
            CKind::TracedPlaneAt => &[
                ("traced", S::Traced), ("plane", S::Plane), ("at", S::Point), ("u", S::Param), ("s", S::Param),
            ],
            CKind::TracedCurvatureAt => &[
                ("traced", S::Traced), ("circle", S::CircleOrArc), ("at", S::Point), ("u", S::Param), ("s", S::Param),
            ],
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
            // which side of the first line the second passes: ±1 pins it, 0 — what nothing that
            // writes one says — leaves the distance a magnitude, its side the roots' to choose
            // by a bound, or the seed (#217)
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
            // the point carried and the point it is carried from, then each pair, source and
            // target, in the order the pairs are written (#357)
            CKind::Mapped => &[
                ("q", S::Point), ("p", S::Point),
                ("from", S::Point), ("a", S::Point),
                ("to", S::Point), ("b", S::Point),
                ("third", S::Point), ("c", S::Point),
            ],
            // and two pairs, with whether the map reverses the turn
            CKind::MappedSimilar => &[
                ("q", S::Point), ("p", S::Point),
                ("from", S::Point), ("a", S::Point),
                ("to", S::Point), ("b", S::Point),
                ("mirror", S::Bool),
            ],
            // a circle and the circle it is carried from, then the two pairs
            CKind::MappedCircle => &[
                ("k", S::Circle), ("from_k", S::Circle),
                ("from", S::Point), ("a", S::Point),
                ("to", S::Point), ("b", S::Point),
                ("mirror", S::Bool),
            ],
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
            | CKind::SplineTangentAt
            | CKind::CurveTangentAt
            // a place on an envelope: along a meridian (a fraction of a line, an angle, a length
            // along a generator), round an axis, and a roll — read as numbers
            | CKind::PointOnEnvelope
            | CKind::EnvelopeTangentAt
            | CKind::EnvelopeNormalAt
            | CKind::EnvelopePlaneAt
            | CKind::EnvelopeCurvatureAt
            // a phase and a lag in degrees, and a place on each flank
            | CKind::PointOnMeshing
            | CKind::MeshingNormalAt
            | CKind::MeshingTangentAt
            // a fraction of a run or a turn
            | CKind::PointAlongLine
            | CKind::PointAlongArc
            | CKind::PointAlongLine3
            | CKind::PointAlongArc3
            => Some(Dim::SCALAR),
            // a screw's pitch: how far it slides a radian it turns (#326)
            CKind::ScrewAxis => Some(Dim::LENGTH),
            // a roll in degrees, as a meshing's phase is
            CKind::CarriedPoint => Some(Dim::SCALAR),
            // a place on a traced body: its formals' numbers, as a curve's parameter is
            CKind::PointOnTraced
            | CKind::TracedTangentAt
            | CKind::TracedSurfaceTangentAt
            | CKind::TracedNormalAt
            | CKind::TracedPlaneAt
            | CKind::TracedCurvatureAt => Some(Dim::SCALAR),
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
    /// kinds, `distance` nine, `tangent` thirteen**, and `horizontal`/`vertical` are a line's
    /// direction prefixed and, between a pair of points infixed, the aliases of `level`.
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
            | CKind::PointOnExtrusion
            // and at a fraction of the way along, told apart by its `t`
            | CKind::PointAlongLine
            | CKind::PointAlongArc
            | CKind::PointAlongLine3
            | CKind::PointAlongArc3 => ("coincident", Infix),
            CKind::CurveTangentLine | CKind::CurveTangentAt => ("tangent", Infix),
            CKind::CurveCurvature => ("curvature", Infix),
            // an envelope in space (#283): a point on it, a line or a plane touching it, a line
            // along its normal and a circle osculating it, the last four at a point already on it
            CKind::PointOnEnvelope => ("coincident", Infix),
            // the path of contact of two flanks (#283), and at a point already on it a line along
            // the common normal or along the path (#345)
            CKind::PointOnMeshing => ("coincident", Infix),
            CKind::MeshingNormalAt => ("perpendicular", Infix),
            CKind::MeshingTangentAt => ("tangent", Infix),
            // a motion's twist (#326): a line a force along which does no work on it, and the
            // axis it screws about
            CKind::Reciprocal => ("reciprocal", Infix),
            CKind::ScrewAxis => ("screw", Infix),
            // a point as the moving frame sees it (#326)
            CKind::CarriedPoint => ("coincident", Infix),
            // a flank's regularity within a blank (#345): written `regular(…) within`
            CKind::RegularWithin => ("regular", Infix),
            // a body traced in space (#326): a point on it, and at a point already on it a line
            // or a plane touching it, a line along a surface's normal and a circle osculating it
            CKind::PointOnTraced => ("coincident", Infix),
            CKind::TracedTangentAt | CKind::TracedSurfaceTangentAt | CKind::TracedPlaneAt => ("tangent", Infix),
            CKind::TracedNormalAt => ("perpendicular", Infix),
            CKind::TracedCurvatureAt => ("curvature", Infix),
            CKind::EnvelopeTangentAt | CKind::EnvelopePlaneAt => ("tangent", Infix),
            CKind::EnvelopeNormalAt => ("perpendicular", Infix),
            CKind::EnvelopeCurvatureAt => ("curvature", Infix),
            CKind::CurveTouchesLine => ("touches", Infix),
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
            // touching: seven kinds, told apart by the pair and by `at:`
            CKind::TangentLineCircle
            | CKind::TangentLineCircleAt
            | CKind::TangentCircleCircle
            | CKind::TangentArcLine
            | CKind::SplineTangentLine
            | CKind::SplineTangentAt
            | CKind::TangentCircleAt
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
            // a point carried by a map, its pairs in the parentheses (#357): three an affine map,
            // two a similarity
            CKind::Mapped | CKind::MappedSimilar | CKind::MappedCircle => ("mapped", Infix),
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
            CKind::TangentCircleAt3 => ("tangent", Infix),
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
            // beside them, which says no sense: `along: u` is measured along the view's `u`,
            // signed from the first point to the second (§9.2).  `every_direction_is_a_word`
            // holds the two tables together.
            (CKind::PointLineDistance, 3) | (CKind::ParallelDistance, 3) => &["left", "right"][..],
            (CKind::TangentLineCircle, 2) => &["left", "right"][..],
            (CKind::Ordinate, 4) | (CKind::Level, 3) => &ALONG_WORDS[..],
            (CKind::Angle, 3) => &["ccw", "cw"][..],
            (CKind::EqualAngle, 4) => &["ccw", "cw"][..],
            _ => return None,
        })
    }

    /// **A map's two sides** (#357), as slots: the source frame's points and the thing carried
    /// from it, and the targets and the thing carried — each side read in a plane of its own
    /// (`program::reading`), the source frame what a refusal of a degenerate map asks of
    /// (`validate`).  `None` for every kind that is no map.
    pub fn map_sides(self) -> Option<MapSides> {
        Some(match self {
            CKind::Mapped => MapSides { source: &[1, 2, 4, 6], target: &[0, 3, 5, 7] },
            CKind::MappedSimilar | CKind::MappedCircle => {
                MapSides { source: &[1, 2, 4], target: &[0, 3, 5] }
            }
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
            CKind::ScrewAxis => &[0],
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

    /// Where its number stands, if it states one: a kind has at most one dimension slot.
    pub fn dimension_slot(self) -> Option<usize> {
        self.spec().iter().position(|(_, k)| k.is_dimension())
    }

    /// What its number is — a length or an angle — where it states one (a length where not, the
    /// harmless default for a number's units).
    pub fn dimension_kind(self) -> SpecKind {
        self.dimension_slot().map_or(SpecKind::Length, |i| self.spec()[i].1)
    }

    /// The kinds whose number may be a bound (§9.6): those reading a signed or unsigned measure
    /// a solve can be steered along — an ordinate (and so `inside`/`outside` a plane), a
    /// distance between points, a point's or a parallel line's distance from a line, in a view
    /// or in space, and the unsigned angle in space (a cone's region, §6.21).  Not the angle in
    /// a view, which is signed: bounded, it would be a half-plane and not a wedge.
    pub fn boundable(self) -> bool {
        matches!(
            self,
            CKind::Ordinate
                | CKind::Distance
                | CKind::Distance3
                | CKind::PointLineDistance
                | CKind::PointLine3
                | CKind::ParallelDistance
                | CKind::Angle3
                // a flank's least area factor (#345): at least a margin, read as found
                | CKind::RegularWithin
        )
    }

    /// Whether a bound of this kind chooses a root a solve can be steered to, by carrying a point
    /// it measures across its edge (§9.6) — every boundable kind but a flank's regularity (#345),
    /// a margin read at a point a search finds, which there is no root of the drawing to choose
    /// by: it is a verdict, `BOUND_BROKEN` where it does not hold, and steers nothing.
    pub fn steers(self) -> bool {
        self.boundable() && self != CKind::RegularWithin
    }

    pub fn claimable(self) -> bool {
        // a touch is part of its curve's problem, and holds wherever the problem is solved
        !self.gauge() && !self.spec().iter().any(|(_, k)| k.is_param()) && self != CKind::CurveTouchesLine
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

    /// A contact at a fraction of a line's run or an arc's turn (#248): the slot naming the line
    /// or the arc, and the one holding the fraction.
    pub fn fraction_slots(self) -> Option<(usize, usize)> {
        match self {
            CKind::PointAlongLine | CKind::PointAlongLine3 => self.contact_on(SpecKind::Line),
            CKind::PointAlongArc | CKind::PointAlongArc3 => self.contact_on(SpecKind::Arc),
            _ => None,
        }
    }

    /// The slots of a contact's place along whatever it runs along — a spline, a curve, a line's
    /// or an arc's run — the one reading of what a shared place is on, what one unit of it is
    /// worth, and what a compile scales its column by.
    pub fn place_on(self) -> Option<(usize, usize)> {
        self.contact_slots().or_else(|| self.contact_on(SpecKind::Curve)).or_else(|| self.fraction_slots())
    }

    /// Whether this kind's kernel is built from the sketch at a width of its operands' (a
    /// spline length's control-point count) rather than registered: `Constraint::kernel_key`
    /// names it, and nothing may ask `kernel()` for it.
    pub fn built(self) -> bool {
        matches!(self, CKind::SplineLength | CKind::Stationary | CKind::CurveTouchesLine
                       | CKind::Reciprocal | CKind::ScrewAxis | CKind::CarriedPoint
                       | CKind::RegularWithin)
            || self.envelope_contact().is_some()
            || self.meshing_contact().is_some()
            || self.traced_contact().is_some()
    }

    /// The traced-body contact this kind states (#326): its kernel is built at its operands'
    /// widths (`traced::kernel`).
    pub fn traced_contact(self) -> Option<crate::traced::Contact> {
        use crate::traced::Contact;
        Some(match self {
            CKind::PointOnTraced => Contact::Point,
            CKind::TracedTangentAt => Contact::Tangent,
            CKind::TracedSurfaceTangentAt => Contact::InPlane,
            CKind::TracedNormalAt => Contact::Normal,
            CKind::TracedPlaneAt => Contact::Plane,
            CKind::TracedCurvatureAt => Contact::Curvature,
            _ => return None,
        })
    }

    /// What a relation reading a motion's twist states (#326): its kernel is built at its
    /// operands' widths (`envelope::screw::kernel`).
    pub fn motion_reading(self) -> Option<crate::envelope::screw::Reading> {
        use crate::envelope::screw::Reading;
        Some(match self {
            CKind::Reciprocal => Reading::Reciprocal,
            CKind::ScrewAxis => Reading::Axis,
            CKind::CarriedPoint => Reading::Carried,
            _ => return None,
        })
    }

    /// The envelope contact this kind states (#283), for the five over an envelope in space: its
    /// kernel is built at its operands' widths (`envelope::held::kernel`).
    pub fn envelope_contact(self) -> Option<crate::envelope::held::Contact> {
        use crate::envelope::held::Contact;
        Some(match self {
            CKind::PointOnEnvelope => Contact::Point,
            CKind::EnvelopeTangentAt => Contact::Tangent,
            CKind::EnvelopeNormalAt => Contact::Normal,
            CKind::EnvelopePlaneAt => Contact::Plane,
            CKind::EnvelopeCurvatureAt => Contact::Curvature,
            _ => return None,
        })
    }

    /// The meshing contact this kind states (#283, #345), for the three over a meshing: its kernel
    /// is built at its operands' widths (`envelope::meshing::kernel`).
    pub fn meshing_contact(self) -> Option<crate::envelope::meshing::Contact> {
        use crate::envelope::meshing::Contact;
        Some(match self {
            CKind::PointOnMeshing => Contact::Point,
            CKind::MeshingNormalAt => Contact::Normal,
            CKind::MeshingTangentAt => Contact::Path,
            _ => return None,
        })
    }

    /// The per-definition kernel this kind runs through, for the four kinds that have one.
    pub fn family_kernel(self) -> Option<FamilyKernel> {
        Some(match self {
            CKind::PointOnCurve => FamilyKernel::Contact,
            CKind::PointOnExtrusion => FamilyKernel::Extrusion,
            CKind::CurveTangentLine => FamilyKernel::Tangent,
            CKind::CurveTangentAt => FamilyKernel::TangentAt,
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
            | CKind::CurveCurvature
            | CKind::SplineTangentAt
            | CKind::CurveTangentAt
            | CKind::TangentCircleAt
            | CKind::EnvelopeTangentAt
            | CKind::EnvelopeNormalAt
            | CKind::EnvelopePlaneAt
            | CKind::EnvelopeCurvatureAt
            | CKind::MeshingNormalAt
            | CKind::MeshingTangentAt
            | CKind::TracedTangentAt
            | CKind::TracedSurfaceTangentAt
            | CKind::TracedNormalAt
            | CKind::TracedPlaneAt
            | CKind::TracedCurvatureAt => true,
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
            // a corner, not a tangency: the curve's own problem, no row of the drawing's
            | CKind::CurveTouchesLine
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
            // a point on an envelope is a contact, as one on a curve is
            | CKind::PointOnEnvelope
            | CKind::PointOnMeshing
            // a motion's twist read (#326): no contact
            | CKind::Reciprocal
            | CKind::ScrewAxis
            | CKind::CarriedPoint
            // a flank's regularity (#345): no contact
            | CKind::RegularWithin
            // a point on a traced body is a contact, as one on a curve is
            | CKind::PointOnTraced
            | CKind::PointAlongLine
            | CKind::PointAlongArc
            | CKind::PointAlongLine3
            | CKind::PointAlongArc3
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
            | CKind::TangentCircleAt3
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
            // an affine combination of points: algebra, no contact
            | CKind::Mapped
            | CKind::MappedSimilar
            | CKind::MappedCircle
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
            CKind::PointOnCurve
            | CKind::PointOnExtrusion
            | CKind::CurveTangentLine
            | CKind::CurveTangentAt
            | CKind::CurveCurvature => {
                panic!("a curve contact's kernel belongs to its definition, not its type")
            }
            CKind::PointOnEnvelope
            | CKind::EnvelopeTangentAt
            | CKind::EnvelopeNormalAt
            | CKind::EnvelopePlaneAt
            | CKind::EnvelopeCurvatureAt => {
                panic!("an envelope contact's kernel is built at its operands' widths")
            }
            CKind::PointOnMeshing | CKind::MeshingNormalAt | CKind::MeshingTangentAt => {
                panic!("a meshing contact's kernel is built at its operands' widths")
            }
            CKind::PointOnTraced
            | CKind::TracedTangentAt
            | CKind::TracedSurfaceTangentAt
            | CKind::TracedNormalAt
            | CKind::TracedPlaneAt
            | CKind::TracedCurvatureAt => {
                panic!("a traced body's contact's kernel is built at its operands' widths")
            }
            CKind::RegularWithin => {
                panic!("a flank's regularity is read by a kernel built at its operands' widths")
            }
            CKind::Reciprocal | CKind::ScrewAxis | CKind::CarriedPoint => {
                panic!("a motion's twist is read by a kernel built at its operands' widths")
            }
            CKind::SplineLength => panic!("a spline length's kernel belongs to its spline's count"),
            CKind::Stationary => panic!("an energy's kernel belongs to its curve"),
            CKind::CurveTouchesLine => panic!("a touch is its curve's problem, and has no kernel"),
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
            // and at a point named: the same three things
            CKind::TangentCircleAt => K::TangentArcLine,
            CKind::Symmetric => K::Symmetric,
            CKind::PointOnSpline => K::PointOnSpline,
            CKind::SplineTangentLine => K::SplineTangentLine,
            CKind::SplineTangentAt => K::SplineTangentAt,
            CKind::SplineCurvature => K::SplineCurvature,
            CKind::PointAlongLine => K::PointAlongLine,
            CKind::PointAlongArc => K::PointAlongArc,
            CKind::PointAlongLine3 => K::PointAlongLine3,
            CKind::PointAlongArc3 => K::PointAlongArc3,
            CKind::TangentCircleAt3 => K::TangentCircleAt3,
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
            CKind::Mapped => K::Mapped,
            CKind::MappedSimilar => K::MappedSimilar,
            CKind::MappedCircle => K::MappedCircle,
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
            CKind::Stationary | CKind::CurveTouchesLine => return None,
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
            | CKind::SplineTangentAt
            | CKind::CurveTangentAt
            | CKind::TangentCircleAt
            | CKind::PointOnEnvelope
            | CKind::EnvelopeTangentAt
            | CKind::EnvelopeNormalAt
            | CKind::EnvelopePlaneAt
            | CKind::EnvelopeCurvatureAt
            | CKind::PointOnMeshing
            | CKind::MeshingNormalAt
            | CKind::MeshingTangentAt
            // nor a motion's twist read (#326)
            | CKind::Reciprocal
            | CKind::ScrewAxis
            | CKind::CarriedPoint
            | CKind::RegularWithin
            // nor a contact with a traced body
            | CKind::PointOnTraced
            | CKind::TracedTangentAt
            | CKind::TracedSurfaceTangentAt
            | CKind::TracedNormalAt
            | CKind::TracedPlaneAt
            | CKind::TracedCurvatureAt
            | CKind::PointAlongLine
            | CKind::PointAlongArc
            | CKind::PointAlongLine3
            | CKind::PointAlongArc3
            | CKind::Level
            | CKind::Project
            | CKind::Lift
            | CKind::Coincident3
            | CKind::Perpendicular3
            | CKind::Parallel3
            | CKind::PointOnPlane
            | CKind::PointOnCircle3
            | CKind::TangentCircleAt3
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
            // a map states no number
            | CKind::Mapped
            | CKind::MappedSimilar
            | CKind::MappedCircle
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
                | CKind::TangentCircleAt3
                | CKind::ProjectSolved
                | CKind::PointOnLine3
                | CKind::PointAlongLine3
                | CKind::PointAlongArc3
                | CKind::EqualLength3
                | CKind::LineOnPlane
                | CKind::Midpoint3
                | CKind::Symmetric3
                | CKind::PointOnExtrusion
                | CKind::PointOnEnvelope
                | CKind::EnvelopeTangentAt
                | CKind::EnvelopeNormalAt
                | CKind::EnvelopePlaneAt
                | CKind::EnvelopeCurvatureAt
                | CKind::PointOnMeshing
                | CKind::MeshingNormalAt
                | CKind::MeshingTangentAt
                | CKind::Reciprocal
                | CKind::ScrewAxis
                | CKind::CarriedPoint
                | CKind::RegularWithin
                | CKind::PointOnTraced
                | CKind::TracedTangentAt
                | CKind::TracedSurfaceTangentAt
                | CKind::TracedNormalAt
                | CKind::TracedPlaneAt
                | CKind::TracedCurvatureAt
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
