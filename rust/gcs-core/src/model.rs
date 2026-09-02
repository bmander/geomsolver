//! Parameters, primitives and the Sketch container.
//!
//! Every scalar degree of freedom is a `Param`.  Primitives are thin bundles of Param indices;
//! the Sketch owns the ordered list of Params (its parameter vector) and the ordered list of
//! Constraints.  Ordering is deterministic by construction — insertion order, never hashing — so
//! identical edits give bit-identical solves.
//!
//! Identity is an integer everywhere: a Param is its index, an entity is `(kind, index)`, and a
//! constraint is a monotonic `id`.  The bindings intern their proxies on those, so `is` / `===`
//! keep working across the FFI without any pointer ever leaving the core.

use crate::constraints::{Arg, Constraint};
use crate::rng::Rng;
use crate::style::Classes;
use std::collections::BTreeMap;

pub type Box2 = (f64, f64, f64, f64); // (xmin, ymin, xmax, ymax)

/// Grow a box to take in a point.
pub fn grow(b: &mut Box2, p: (f64, f64)) {
    b.0 = b.0.min(p.0);
    b.1 = b.1.min(p.1);
    b.2 = b.2.max(p.0);
    b.3 = b.3.max(p.1);
}

thread_local! {
    /// Scratch the model-side locus paths run in — a pick, a paint and a bounds query each
    /// evaluate every trace curve they touch, and building a fresh scratch per question would
    /// be an allocation per hover.
    static MODEL_LOCUS: std::cell::RefCell<crate::locus::Scratch> =
        std::cell::RefCell::new(crate::locus::Scratch::new());
}

/// How many steps a user-written curve is sampled at for measuring, picking and bounding.  A
/// B-spline is refined adaptively against its own basis; a curve written in the language has no
/// basis to refine against, so it is sampled evenly and finely enough that a pick test does not
/// lie about what the drawing shows.
pub const CURVE_STEPS: usize = 128;

#[derive(Clone, Debug)]
pub struct Param {
    pub value: f64,
    pub fixed: bool,
    pub name: String,
    /// World length one unit of this parameter is worth.  A coordinate or a radius is a length
    /// already, so 1; a curve parameter is dimensionless, and one unit of it moves a point by
    /// roughly the curve's length.  Everything that measures motion in world units — the
    /// witness perturbation, the warm-start jitter, and the minimum-norm step's column
    /// weighting — divides by it, so a dimensionless unknown is neither shoved across its whole
    /// range by a jitter meant for coordinates nor left immovable by a step that is.
    pub scale: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum EntKind {
    Point,
    Line,
    Circle,
    Arc,
    Spline,
    Ellipse,
    /// An origin and an attitude: a datum other statements measure from.  The attitude is a
    /// unit rotor — two scalars `(c, s)` held to `c² + s² = 1` by an intrinsic constraint, the
    /// 2D form of the quaternion a 3D workplane will want — kept pointed from `origin` at
    /// `toward` by a second intrinsic, so the rotor is a first-class unknown that adds no
    /// freedom beyond the two points it is slaved to.  A trace block reads `f.angle`
    /// (`atan2(s, c)`, degrees — derived in `Tape::compile`, never stored) to state a bearing
    /// relative to the frame instead of the page.
    Frame,
    /// A frame that is also a *view*: the same origin, toward point and rotor, plus a constant
    /// 3D attitude (`plane::Basis`) saying which plane in space the picture drawn in it is of.
    /// A point that says it is `in` the plane is an image of something on that plane, and
    /// `Project` between two such points is the one equation two images of one point share.
    /// Nothing about the attitude is ever solved for — it is document data, like a spline's
    /// knots — which is what keeps the whole engine planar.
    Plane,
    /// A curve written in the language: `C(u)` as an expression over the geometry it is drawn
    /// from.  Unlike every other kind it holds no coordinates of its own — it *is* the two
    /// expressions plus whatever it reads — so it moves when its arguments do and never
    /// otherwise.  See `CurveDef`.
    Curve,
}

impl EntKind {
    pub fn as_str(self) -> &'static str {
        match self {
            EntKind::Point => "point",
            EntKind::Line => "line",
            EntKind::Circle => "circle",
            EntKind::Arc => "arc",
            EntKind::Spline => "spline",
            EntKind::Ellipse => "ellipse",
            EntKind::Frame => "frame",
            EntKind::Plane => "plane",
            EntKind::Curve => "curve",
        }
    }

    pub fn parse(s: &str) -> Option<EntKind> {
        Some(match s {
            "point" => EntKind::Point,
            "line" => EntKind::Line,
            "circle" => EntKind::Circle,
            "arc" => EntKind::Arc,
            "spline" => EntKind::Spline,
            "ellipse" => EntKind::Ellipse,
            "frame" => EntKind::Frame,
            "plane" => EntKind::Plane,
            "curve" => EntKind::Curve,
            _ => return None,
        })
    }

    /// What an entity's declaration names, in order.  A `Child` is a sub-entity and binds by
    /// aliasing; a `Scalar` is a number the entity owns.  `List` is a child field that is a
    /// *list* — a control polygon — and is the reason a spline survives losing one of them.
    ///
    /// One table, so a new entity kind is named the same way wherever one has to be written
    /// down.  The names are the document's own keys (`io::to_json`), which
    /// `tests/io.rs::the_document_uses_the_field_names` holds them to.
    pub fn fields(self) -> &'static [(&'static str, Field)] {
        use Field::{Child as C, List as L, Scalar as S};
        match self {
            EntKind::Point => &[("x", S), ("y", S)],
            EntKind::Line => &[("p1", C), ("p2", C)],
            EntKind::Circle => &[("center", C), ("r", S)],
            EntKind::Arc => &[("center", C), ("start", C), ("end", C), ("r", S)],
            EntKind::Spline => &[("ctrl", L)],
            EntKind::Ellipse => &[("center", C), ("major", C), ("b", S)],
            // a plane's attitude is not a field: a Scalar is a number a solve may write back,
            // and the basis is document data no solve moves
            EntKind::Frame | EntKind::Plane => {
                &[("origin", C), ("toward", C), ("c", S), ("s", S)]
            }
            // as many arguments as its definition takes, and none of them need be points — the
            // first kind for which that is true
            EntKind::Curve => &[("args", L)],
        }
    }

    /// Where an entity of this kind is *entered and left*, as indices into its `fields()` with
    /// the scalars filtered out — which is the indexing `Decl::children` uses.
    ///
    /// A line runs `p1 → p2`; an arc runs CCW `start → end`.  This is what a chain (Solvent
    /// §6.6) threads through: a joint's shared point is one element's exit and the next one's
    /// entry.  `None` is a kind with no boundary — a circle has no ends, which is why its
    /// radius is a Param and not a witness point, and why it cannot sit in a chain.
    ///
    /// It lives here, beside the table it indexes, because two integers derived from
    /// `fields()` and written down somewhere else are two integers that go stale the first time
    /// a field is reordered — silently, and in the direction of a wrong drawing.  Matched
    /// exhaustively for the same reason every other table here is: a new kind with ends must
    /// stop the build and be given an arm.
    pub fn ends(self) -> Option<(usize, usize)> {
        match self {
            EntKind::Line => Some((0, 1)),
            EntKind::Arc => Some((1, 2)),
            EntKind::Point
            | EntKind::Circle
            | EntKind::Spline
            | EntKind::Ellipse
            | EntKind::Frame
            | EntKind::Plane
            | EntKind::Curve => None,
        }
    }

    /// The names a curve written over an entity of this kind reads its coordinates by, in
    /// **`Sketch::entity_params` order** — `c.center.x`, `c.center.y`, `c.r` for a circle.
    ///
    /// That order is the whole contract: it is the order a definition's tapes are compiled
    /// against and the order `params_on` hands the kernel its columns, so a tape's gradient is a
    /// row of the Jacobian with nothing to rearrange.  The two are held together by
    /// `tests/curvedef.rs::the_names_match_the_parameters`.
    ///
    /// `None` for a kind whose parameter count is not fixed — a spline's control polygon is as
    /// long as somebody drew it, so a curve cannot be written over one by name.
    pub fn scalar_names(self, n: &str) -> Option<Vec<String>> {
        let pt = |f: &str| vec![format!("{n}.{f}.x"), format!("{n}.{f}.y")];
        Some(match self {
            EntKind::Point => vec![format!("{n}.x"), format!("{n}.y")],
            EntKind::Line => [pt("p1"), pt("p2")].concat(),
            EntKind::Circle => [pt("center"), vec![format!("{n}.r")]].concat(),
            EntKind::Arc => {
                [pt("center"), pt("start"), pt("end"), vec![format!("{n}.r")]].concat()
            }
            EntKind::Ellipse => [pt("center"), pt("major"), vec![format!("{n}.b")]].concat(),
            EntKind::Frame | EntKind::Plane => {
                [pt("origin"), pt("toward"), vec![format!("{n}.c"), format!("{n}.s")]].concat()
            }
            EntKind::Spline | EntKind::Curve => return None,
        })
    }

    /// Whether an entity of this kind has points of its own to put on a plane — what the `in`
    /// clause and the `in … { }` block ask before stamping one (§6.7).  A datum's two points
    /// are the datum's, and a curve is its expressions; everything else is drawn from points a
    /// membership is about.
    ///
    /// Exhaustive, and asked rather than spelled: written out as a `matches!` at each of the
    /// five sites that ask it — the parser twice, the flattener, the elaborator and the
    /// writeback — a new kind joined the list at whichever of them its author happened to
    /// read, and was silently stamped at the rest.
    pub fn bears_points(self) -> bool {
        match self {
            EntKind::Frame | EntKind::Plane | EntKind::Curve => false,
            EntKind::Point
            | EntKind::Line
            | EntKind::Circle
            | EntKind::Arc
            | EntKind::Spline
            | EntKind::Ellipse => true,
        }
    }

    /// A class a kind carries without being given it — the datum glyph a plane is drawn as is
    /// `.plane`, so the sheet says what a plane looks like the way it says what a dimension
    /// does, and the document's own `style .plane` rule wins over the shipped one.  Never
    /// written to the document: it is a fact about the kind, not about the statement.
    pub fn implicit_class(self) -> Option<&'static str> {
        match self {
            EntKind::Plane => Some("plane"),
            // a point carries no class of its own and is drawn as a handle; `.point` is how a
            // document says the handles are not part of the picture (`display: none`)
            EntKind::Point => Some("point"),
            EntKind::Line
            | EntKind::Circle
            | EntKind::Arc
            | EntKind::Spline
            | EntKind::Ellipse
            | EntKind::Frame
            | EntKind::Curve => None,
        }
    }

    /// How many sub-entities a declaration names — `None` for a kind whose children are a *list*,
    /// which is a control polygon and is as long as somebody drew it.
    pub fn children_arity(self) -> Option<usize> {
        let f = self.fields();
        (!f.iter().any(|(_, k)| *k == Field::List))
            .then(|| f.iter().filter(|(_, k)| *k == Field::Child).count())
    }
}

/// What one field of an entity declaration holds — see `EntKind::fields`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Field {
    /// One sub-entity, bound by aliasing.
    Child,
    /// A list of sub-entities: a control polygon.
    List,
    /// A number the entity owns — a coordinate, a radius, a minor axis.
    Scalar,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct EntRef {
    pub kind: EntKind,
    pub idx: u32,
}

impl EntRef {
    pub fn new(kind: EntKind, idx: usize) -> EntRef {
        EntRef { kind, idx: idx as u32 }
    }
    pub fn point(idx: usize) -> EntRef {
        EntRef::new(EntKind::Point, idx)
    }
    pub fn line(idx: usize) -> EntRef {
        EntRef::new(EntKind::Line, idx)
    }
    pub fn circle(idx: usize) -> EntRef {
        EntRef::new(EntKind::Circle, idx)
    }
    pub fn arc(idx: usize) -> EntRef {
        EntRef::new(EntKind::Arc, idx)
    }
    pub fn spline(idx: usize) -> EntRef {
        EntRef::new(EntKind::Spline, idx)
    }
    pub fn ellipse(idx: usize) -> EntRef {
        EntRef::new(EntKind::Ellipse, idx)
    }
    pub fn frame(idx: usize) -> EntRef {
        EntRef::new(EntKind::Frame, idx)
    }
    pub fn plane(idx: usize) -> EntRef {
        EntRef::new(EntKind::Plane, idx)
    }
    pub fn i(self) -> usize {
        self.idx as usize
    }
}

#[derive(Clone, Debug)]
pub struct PointE {
    pub x: u32,
    pub y: u32,
    /// The plane this point is an image on, if it says (`point a in top`) — what `Project`
    /// reads to know which two views it relates.  A membership, not a constraint: it moves
    /// nothing, and a point with none is simply on the page.
    pub plane: Option<u32>,
}

#[derive(Clone, Debug)]
pub struct LineE {
    pub p1: u32,
    pub p2: u32,
    /// How it is *presented* — see `style.rs`.  Nothing the core computes reads it.
    pub class: Classes,
}

#[derive(Clone, Debug)]
pub struct CircleE {
    pub center: u32,
    pub radius: u32,
    pub class: Classes,
}

/// CCW arc from `start` to `end` about `center`.  The radius is its own Param so Circle and Arc
/// share every radius-based constraint; the two intrinsic constraints |start-center|² = r² and
/// |end-center|² = r² are added by `Sketch::arc`.
#[derive(Clone, Debug)]
pub struct ArcE {
    pub center: u32,
    pub start: u32,
    pub end: u32,
    pub radius: u32,
    pub class: Classes,
}

/// A cubic B-spline over an ordered control polygon.
///
/// The control points are ordinary sketch Points — they drag, snap and take constraints like any
/// others, which is what makes a spline's shape editable with the tools that already exist, the
/// same trick as an arc being a centre and two real points plus its two intrinsic constraints.
/// The knot vector is document data, not unknowns: a repeated interior knot is a corner, and the
/// clamped uniform default runs the curve from the first control point to the last.  Four
/// control points and no interior knot is exactly a cubic Bézier.
#[derive(Clone, Debug)]
pub struct SplineE {
    pub ctrl: Vec<u32>,
    /// `ctrl.len() + curve::DEGREE + 1` non-decreasing values.
    pub knots: Vec<f64>,
    pub class: Classes,
}

/// Centre, one end of the major axis, and a minor radius of its own.  Five numbers — exactly
/// the 5 DOF an ellipse has — so unlike an arc it needs no intrinsic constraint.  The major
/// point is a real rim point at the end of the long axis, so it drags, snaps and constrains
/// like any other point; only the minor radius is the ellipse's own.
#[derive(Clone, Debug)]
pub struct EllipseE {
    pub center: u32,
    pub major: u32,
    pub minor: u32,
    pub class: Classes,
}

/// An origin, a point it is pointed at, and the unit rotor `(c, s)` between them — see
/// `EntKind::Frame`.  `c` and `s` are Param indices; the two intrinsic constraints that slave
/// them to the chord are added by `Sketch::frame` and never serialized, the arc's bargain.
#[derive(Clone, Debug)]
pub struct FrameE {
    pub origin: u32,
    pub toward: u32,
    pub c: u32,
    pub s: u32,
    pub class: Classes,
}

/// A frame with an attitude in space — see `EntKind::Plane`.  The frame half is the page
/// placement (where the view sits and which way it is turned), the basis is which plane of the
/// object it pictures; only the first is ever solved for.
#[derive(Clone, Debug)]
pub struct PlaneE {
    pub frame: FrameE,
    pub basis: crate::plane::Basis,
}

/// A curve, compiled: **a point of a component, as one of the component's numeric formals
/// runs** (Solvent §6.5).  `C(u)` is where the component's own statements put the point, given
/// the formal's value `u` and the geometry the component is written over.
///
/// This is what makes an involute — or a cycloid, or a walking leg's stride — *library code*
/// rather than another entity kind with another pair of kernels: a component is written once,
/// drawn or not, and a curve is one of its points asked over an interval.  A definition names
/// the entities the component is written over, the numbers it takes besides the swept one, and
/// the swept formal; the body is compiled against one variable table, which is the swept formal
/// followed by every scalar the entity formals contribute, in `entity_params` order, then the
/// other numbers.  That order is the kernel's column order, so a tape's gradient *is* the
/// Jacobian row.  One definition serves every instance of the component asked for the same
/// point over the same formal.
#[derive(Clone, Debug)]
pub struct CurveDef {
    /// `CurveDef::key` — never a spelling.
    pub name: String,
    pub component: String,
    /// The point, by its name under the instance: `toe`, `sub.pt`.
    pub port: String,
    /// The entity formals, in order: what an instance must supply, and of what kind.
    pub formals: Vec<(String, EntKind)>,
    /// The numeric formals other than the swept one, in order.
    pub values: Vec<String>,
    /// The swept formal — what the curve runs on.
    pub param: String,
    /// The variable table the body was compiled over: `param` first, then one name per scalar
    /// the formals contribute, then the value parameters.  Kept so a definition can be re-read
    /// and printed.
    pub vars: Vec<String>,
    pub body: CurveBody,
    /// For a trace: each inner unknown's owner — the entity's name under the instance and
    /// which of its own scalars — which is where a **drawn** instance's pose is read off, so
    /// the trace's home is the pose on the sheet rather than the component's seeds.  Empty for
    /// a formula.
    pub pose_of: Vec<(String, usize)>,
}

impl CurveDef {
    /// What one definition is keyed by: the component, the point, the swept formal.
    pub fn key(component: &str, point: &str, swept: &str) -> String {
        format!("{component}.{point}/{swept}")
    }
}

/// How a family says where `C(u)` is: as two expressions, or as the constraints that force it.
/// The second is the Wikipedia sentence — "the curve traced by the end of a taut string as it
/// unwinds" — with the working left to the solver; see `locus`.
#[derive(Clone, Debug)]
pub enum CurveBody {
    Exprs { x: crate::tape::Tape, y: crate::tape::Tape },
    Trace(crate::locus::Locus),
}

/// One curve, drawn: a definition, the entities it is written over, and the numbers it was
/// given.  It holds no parameters of its own — it *is* its expressions — so it moves exactly
/// when its arguments do.
#[derive(Clone, Debug)]
pub struct CurveE {
    pub def: u32,
    pub args: Vec<EntRef>,
    pub values: Vec<f64>,
    /// The interval *this* curve is drawn over — the piece of an involute between two circles
    /// rather than the whole spiral.
    pub domain: (f64, f64),
    /// The parameter value a trace is anchored at — the one place a block's orientation
    /// predicates are read (§6.5).  `Sketch::curve_home` reads it.
    pub home: Home,
    /// Where the anchor pose is read from, for a curve of a **drawn** instance: per inner unknown
    /// of the block, the entity on the sheet and which of its own scalars (`CurveDef::pose_of`,
    /// resolved).  Empty for an instance written in place, whose seeds start the trace.
    pub pose: Vec<(EntRef, usize)>,
    pub class: Classes,
}

/// Where a curve's trace is anchored in the parameter: a number the instance gave the swept
/// formal (or the interval's start, when it gave none), or the drawing's own unknown — a drawn
/// instance that left the formal unbound made it one (`leg.theta`), and the anchor is wherever
/// it stands.  Kept as the unknown's *name*, looked up in `free_vars` when read, since the
/// unknown is allocated after the curve is built and moves with every solve.
#[derive(Clone, Debug, PartialEq)]
pub enum Home {
    At(f64),
    Free(String),
}

/// A pose is whole or it is nothing: a list with a hole in it is not the pose the block's
/// `n_q` unknowns want, and the seeds stand in.  The one statement of the rule `cold_start`
/// relies on (`p.len() == v.n_q`), for the three seams that assemble one.
pub fn whole<T>(v: Vec<T>, n: usize) -> Vec<T> {
    if v.len() == n { v } else { Vec::new() }
}

/// The CCW arc through three points.
#[derive(Clone, Copy, Debug)]
pub struct ThreePointArc {
    pub cx: f64,
    pub cy: f64,
    pub r: f64,
    pub a0: f64,
    pub a1: f64,
    /// True when the sweep runs from the *second* given point to the first.
    pub swapped: bool,
}

/// Arc from (ax, ay) to (bx, by) passing through (cx, cy) — the circumcircle of the three, plus
/// the sweep direction that actually contains the third point.  `None` if they are collinear
/// (the test is on the sine of the angle, so it is scale-free).
pub fn three_point_arc(
    ax: f64,
    ay: f64,
    bx: f64,
    by: f64,
    cx: f64,
    cy: f64,
    tol: f64,
) -> Option<ThreePointArc> {
    let (ux, uy) = (bx - ax, by - ay);
    let (vx, vy) = (cx - ax, cy - ay);
    let cross = ux * vy - uy * vx;
    if cross.abs() <= tol * ux.hypot(uy) * vx.hypot(vy) {
        return None;
    }
    let d = 2.0 * cross;
    let (u2, v2) = (ux * ux + uy * uy, vx * vx + vy * vy);
    let ox = ax + (vy * u2 - uy * v2) / d;
    let oy = ay + (ux * v2 - vx * u2) / d;
    let r = (ax - ox).hypot(ay - oy);
    let ta = (ay - oy).atan2(ax - ox);
    let tb = (by - oy).atan2(bx - ox);
    let tau = 2.0 * std::f64::consts::PI;
    let sweep = |th: f64| ((th - ta) % tau + tau) % tau;
    let to_b = sweep(tb);
    let to_c = sweep((cy - oy).atan2(cx - ox));
    Some(if to_c < to_b {
        ThreePointArc { cx: ox, cy: oy, r, a0: ta, a1: ta + to_b, swapped: false }
    } else {
        ThreePointArc { cx: ox, cy: oy, r, a0: tb, a1: tb + (tau - to_b), swapped: true }
    })
}

#[derive(Default, Clone, Debug)]
pub struct Sketch {
    pub params: Vec<Param>,
    pub points: Vec<PointE>,
    pub lines: Vec<LineE>,
    pub circles: Vec<CircleE>,
    pub arcs: Vec<ArcE>,
    pub splines: Vec<SplineE>,
    pub ellipses: Vec<EllipseE>,
    pub frames: Vec<FrameE>,
    pub planes: Vec<PlaneE>,
    pub curves: Vec<CurveE>,
    /// The curve families this document defines.  Document state like `branches`: a curve
    /// instance names one by index.
    pub curve_defs: Vec<CurveDef>,
    pub constraints: Vec<Constraint>,
    /// Recorded root choices (Stage 5), persisted with the document.
    pub branches: BTreeMap<String, i32>,
    /// Where a dimension's callout has been dragged to, by constraint id, in the frame that
    /// callout hangs off — see `callout::Frame`.  Document state, like `branches`: an entry
    /// only exists for a dimension somebody has moved, and dropping one puts that callout back
    /// where the layout would have placed it.
    pub placements: BTreeMap<u32, (f64, f64)>,
    /// The free variables the document's dimension expressions read, by name, each an index
    /// into `params` — see `expr::Free`.  Derived state, owned by `expr::evaluate`: it allocates
    /// one the first time a name nothing defines is read and retires it when the last reader
    /// stops reading it, so nothing else in the document has to know they exist.
    pub free_vars: BTreeMap<String, u32>,
    /// Each curve's polyline, remembered against everything it was computed from
    /// (`curve_polyline`).  A pick walks every drawn curve on every pointer move, and a traced
    /// curve's polyline is a march of `CURVE_STEPS` block solves — nine milliseconds a move on
    /// the gear, for a drawing that had not changed.  Interior, so `&self` readers share it; a
    /// cache and not state, since a miss recomputes what a hit remembers.
    pub polyline_cache: std::cell::RefCell<BTreeMap<usize, (Vec<f64>, Vec<(f64, f64)>)>>,
    /// The document's style sheet: what each class looks like (`style.rs`).  Presentation, and
    /// nothing the core computes reads it — it is here because it is document state, saved and
    /// grafted with everything else, and because the core resolving it is what keeps two front
    /// ends from disagreeing about how one drawing is drawn.
    pub sheet: crate::style::Sheet,
    /// What the document's numbers are in (`units.rs`).  Storing it costs the solve nothing —
    /// every kernel is homogeneous in length, so scaling a whole sketch moves no residual, no
    /// tolerance and no rank — but it is what makes `80mm` mean something and what lets a paste
    /// out of a document in inches arrive in a document in millimetres as the same drawing.
    pub units: crate::units::Units,
    /// Bumped whenever the sheet or a class changes, so a binding may cache the resolved styles
    /// against it.  Geometry moves every frame and presentation almost never does; the counter
    /// is what lets the second be read at the second's rate.
    pub style_epoch: u32,
    next_cid: u32,
}

impl Sketch {
    pub fn new() -> Sketch {
        Sketch::default()
    }

    // -- presentation (`style.rs`) ------------------------------------------

    /// The classes an entity carries.  Empty for a point, which is drawn as a dot and has no
    /// stroke to style; giving it one is a later question and not this one's.
    pub fn class_of(&self, e: EntRef) -> Classes {
        match e.kind {
            EntKind::Point => Classes::default(),
            EntKind::Line => self.lines[e.i()].class.clone(),
            EntKind::Circle => self.circles[e.i()].class.clone(),
            EntKind::Arc => self.arcs[e.i()].class.clone(),
            EntKind::Spline => self.splines[e.i()].class.clone(),
            EntKind::Ellipse => self.ellipses[e.i()].class.clone(),
            EntKind::Frame => self.frames[e.i()].class.clone(),
            EntKind::Plane => self.planes[e.i()].frame.class.clone(),
            EntKind::Curve => self.curves[e.i()].class.clone(),
        }
    }

    /// Give an entity a class, or take one away.  The one write path, so the epoch a binding
    /// caches against cannot be missed.
    pub fn set_class(&mut self, e: EntRef, name: &str, on: bool) {
        let slot = match e.kind {
            EntKind::Point => return,
            EntKind::Line => self.lines.get_mut(e.i()).map(|x| &mut x.class),
            EntKind::Circle => self.circles.get_mut(e.i()).map(|x| &mut x.class),
            EntKind::Arc => self.arcs.get_mut(e.i()).map(|x| &mut x.class),
            EntKind::Spline => self.splines.get_mut(e.i()).map(|x| &mut x.class),
            EntKind::Ellipse => self.ellipses.get_mut(e.i()).map(|x| &mut x.class),
            EntKind::Frame => self.frames.get_mut(e.i()).map(|x| &mut x.class),
            EntKind::Plane => self.planes.get_mut(e.i()).map(|x| &mut x.frame.class),
            EntKind::Curve => self.curves.get_mut(e.i()).map(|x| &mut x.class),
        };
        if let Some(c) = slot {
            c.set(name, on);
            self.style_epoch = self.style_epoch.wrapping_add(1);
        }
    }

    /// Every length in the sketch, times `k` — what a paste between two documents in different
    /// units does to the figure it carries.
    ///
    /// **Written out by kind, and exhaustively**, because "is this parameter a length?" is not a
    /// question a `Param` can answer: a frame's rotor is a direction and a curve's parameter is
    /// a place along it, and scaling either would take the drawing apart.  So each table that
    /// knows says — `own_length_params` per entity kind, `CKind::param_dim` per constraint that
    /// owns an unknown — and a new kind stops the build in the first rather than being silently
    /// left unconverted.
    ///
    /// **An expression's *text* is not rewritten, and the number it came to is converted.**
    /// `w = 80` is arithmetic the author wrote, and rewriting it would be an edit of what the
    /// document says rather than a change of the units it says it in — the same reason
    /// `commit_seeds` never overwrites `hint(r: Rr)`.  What the formula last came to is not
    /// authored, though: it is a length in the document's units like any other, and it is what a
    /// **free variable** is seeded from, so a figure tied together by `== w` would otherwise
    /// arrive scaled with `w` still at its old size.  Everything that re-evaluates is overwritten
    /// by the `expr::evaluate` at the end of `graft` regardless.
    pub fn rescale(&mut self, k: f64) {
        if k == 1.0 || !k.is_finite() || k <= 0.0 {
            return;
        }
        let mut lengths: Vec<u32> = Vec::new();
        for e in self.primitives() {
            lengths.extend(self.own_length_params(e));
        }
        // a free variable a *length* dimension reads is a length itself, and it is the same
        // unknown however many dimensions read it — so the params are gathered before any is
        // written, and each is scaled once
        for c in &self.constraints {
            let Some(f) = &c.free else { continue };
            if c.dimensions().iter().any(|(_, _, kind)| kind.dim() == crate::units::Dim::LENGTH) {
                lengths.push(f.param);
            }
        }
        lengths.sort_unstable();
        lengths.dedup();
        for i in lengths {
            self.params[i as usize].value *= k;
        }
        for c in self.constraints.iter_mut() {
            let reads_a_length =
                c.dimensions().iter().any(|(_, _, kind)| kind.dim() == crate::units::Dim::LENGTH);
            // the offset of `value = m·a + c` is in the dimension's own units, so it converts
            // with the unknown while the ratio `m` does not
            if reads_a_length {
                if let Some(f) = c.free.as_mut() {
                    f.c *= k;
                }
            }
            for (i, (_, kind)) in c.kind.spec().iter().enumerate() {
                let is_length = *kind == crate::constraints::SpecKind::Length
                    || (kind.is_param()
                        && c.kind.param_dim() == Some(crate::units::Dim::LENGTH));
                if !is_length {
                    continue;
                }
                match c.args.get_mut(i) {
                    Some(Arg::Num(v)) => *v *= k,
                    // a Param slot holds a seed on the way in and an index once added; the
                    // index's value was scaled above, the seed is scaled here
                    Some(Arg::Seed { value, .. }) => *value *= k,
                    // the text stays as written; what it came to converts
                    Some(Arg::Expr(e)) => e.value *= k,
                    _ => {}
                }
            }
        }
        // a callout's placement is two world lengths in a frame that follows the geometry
        // (`callout::Frame`), so it converts with the figure it annotates
        for (t, r) in self.placements.values_mut() {
            *t *= k;
            *r *= k;
        }
    }

    /// Which of an entity's *own* params are lengths.
    ///
    /// Exhaustive for `own_params`' reason, and it is the table `rescale` drives off: a new
    /// entity kind with a number of its own must stop the build here, or a paste between two
    /// documents in different units would leave that number unconverted.
    fn own_length_params(&self, e: EntRef) -> Vec<u32> {
        match e.kind {
            EntKind::Point => self.point_params(e.i()).to_vec(),
            EntKind::Circle => vec![self.circles[e.i()].radius],
            EntKind::Arc => vec![self.arcs[e.i()].radius],
            EntKind::Ellipse => vec![self.ellipses[e.i()].minor],
            // the rotor `(c, s)` is a unit vector — a direction, and scaling it would only
            // break `frame_unit`.  A frame's one length is `frame_align`'s chord, which is a
            // constraint's Param and is converted with the constraints.
            // (and a plane's basis is a direction in space, dimensionless twice over)
            EntKind::Frame | EntKind::Plane => Vec::new(),
            // a line and a spline are their points, and a curve is its expressions: no number
            // of their own to convert
            EntKind::Line | EntKind::Spline | EntKind::Curve => Vec::new(),
        }
    }

    /// The whole sheet, replaced.  Elaboration's write path, and the other half of the epoch.
    pub fn set_sheet(&mut self, sheet: crate::style::Sheet) {
        self.sheet = sheet;
        self.style_epoch = self.style_epoch.wrapping_add(1);
    }

    /// What an entity is drawn with: the base sheet under the document's, cascaded over its
    /// classes.  **The core resolves; a front end strokes what it is handed** — the same seam
    /// `callout.rs` and `curve::tessellate` sit on, so every front end draws one drawing alike.
    pub fn style_of(&self, e: EntRef) -> crate::style::Style {
        // a kind may carry a class it was never given — a plane's datum glyph is `.plane` —
        // under whatever the declaration says, so the document's rule still wins
        let mut classes = self.class_of(e);
        if let Some(c) = e.kind.implicit_class() {
            classes.0.insert(0, c.to_string());
        }
        crate::style::resolve(&self.sheet, &classes)
    }

    /// What a *named* class list comes to, spelled as a declaration spells one: `"dimension"`,
    /// or `"dimension reference"`.  The drawing's chrome asks this — a dimension callout is not
    /// an entity and carries no class of its own, but its ink is shared by every callout in the
    /// document, which is exactly what a class is for.
    ///
    /// A list rather than one name because a claimed dimension *is* a dimension: it takes the
    /// shared rule and then the one that says how it differs, which is how a caller gets a
    /// document's `style .dimension` on a reference dimension too.
    pub fn style_named(&self, classes: &str) -> crate::style::Style {
        let list = Classes(classes.split_whitespace().map(str::to_string).collect());
        crate::style::resolve(&self.sheet, &list)
    }

    // -- construction -------------------------------------------------------

    pub fn param(&mut self, value: f64, fixed: bool, name: &str) -> usize {
        self.param_scaled(value, fixed, name, 1.0)
    }

    /// A parameter that is not a length: `scale` is the world length one unit of it is worth.
    pub fn param_scaled(&mut self, value: f64, fixed: bool, name: &str, scale: f64) -> usize {
        self.params.push(Param { value, fixed, name: name.to_string(), scale });
        self.params.len() - 1
    }

    pub fn point(&mut self, x: f64, y: f64, fixed: bool, name: &str) -> usize {
        let px = self.param(x, fixed, &format!("{name}.x"));
        let py = self.param(y, fixed, &format!("{name}.y"));
        self.points.push(PointE { x: px as u32, y: py as u32, plane: None });
        self.points.len() - 1
    }

    /// Which plane a point is an image on, if it says.
    pub fn plane_of(&self, point: usize) -> Option<usize> {
        self.points[point].plane.map(|p| p as usize)
    }

    /// Put a point on a plane, or take it off (`None`).  A membership and not a constraint:
    /// nothing moves, and only `Project` reads it.
    pub fn set_plane(&mut self, point: usize, plane: Option<usize>) {
        debug_assert!(plane.map_or(true, |p| p < self.planes.len()));
        self.points[point].plane = plane.map(|p| p as u32);
    }

    pub fn line(&mut self, p1: usize, p2: usize) -> usize {
        self.lines.push(LineE { p1: p1 as u32, p2: p2 as u32, class: Classes::default() });
        self.lines.len() - 1
    }

    pub fn line_xy(&mut self, x1: f64, y1: f64, x2: f64, y2: f64, name: &str) -> usize {
        let a = self.point(x1, y1, false, &format!("{name}.p1"));
        let b = self.point(x2, y2, false, &format!("{name}.p2"));
        self.line(a, b)
    }

    pub fn circle(&mut self, center: usize, radius: f64, name: &str) -> usize {
        let r = self.param(radius, false, &format!("{name}.r"));
        self.circles.push(CircleE {
            center: center as u32,
            radius: r as u32,
            class: Classes::default(),
        });
        self.circles.len() - 1
    }

    /// An arc plus its two intrinsic `PointOnCircle` constraints.
    pub fn arc(&mut self, center: usize, start: usize, end: usize, name: &str) -> usize {
        let (cx, cy) = self.point_xy(center);
        let (sx, sy) = self.point_xy(start);
        let r = (sx - cx).hypot(sy - cy);
        let rp = self.param(r, false, &format!("{name}.r"));
        self.arcs.push(ArcE {
            center: center as u32,
            start: start as u32,
            end: end as u32,
            radius: rp as u32,
            class: Classes::default(),
        });
        let ai = self.arcs.len() - 1;
        let aref = EntRef::arc(ai);
        let c1 = Constraint::point_on_circle(EntRef::point(start), aref, true);
        let c2 = Constraint::point_on_circle(EntRef::point(end), aref, true);
        self.add(c1);
        self.add(c2);
        ai
    }

    /// Arc from `start` to `end` bulging through `through` — the three-point construction.
    /// Creates the centre point; `None` if the three are collinear.
    pub fn arc_through(
        &mut self,
        start: usize,
        end: usize,
        through: (f64, f64),
        name: &str,
    ) -> Option<usize> {
        let (ax, ay) = self.point_xy(start);
        let (bx, by) = self.point_xy(end);
        let g = three_point_arc(ax, ay, bx, by, through.0, through.1, 1e-9)?;
        let centre = self.point(g.cx, g.cy, false, &format!("{name}.c"));
        let (a, b) = if g.swapped { (end, start) } else { (start, end) };
        Some(self.arc(centre, a, b, name))
    }

    /// An ellipse about `center` whose major axis ends at `major`, with minor radius `b`.
    pub fn ellipse(&mut self, center: usize, major: usize, b: f64, name: &str) -> usize {
        let bp = self.param(b, false, &format!("{name}.b"));
        self.ellipses.push(EllipseE {
            center: center as u32,
            major: major as u32,
            minor: bp as u32,
            class: Classes::default(),
        });
        self.ellipses.len() - 1
    }

    /// A frame at `origin` pointed at `toward`, plus its two intrinsic constraints: the rotor
    /// `(c, s)` held to the unit circle, and kept on the chord `origin → toward` — so the
    /// attitude is a first-class unknown that adds no freedom beyond the two points.  One rotor
    /// unit is worth the chord's length, which is what `Param::scale` asks of a dimensionless
    /// unknown.
    pub fn frame(&mut self, origin: usize, toward: usize, name: &str) -> usize {
        let f = self.datum(origin, toward, name);
        self.frames.push(f);
        let fi = self.frames.len() - 1;
        self.slave(EntRef::frame(fi));
        fi
    }

    /// A plane: a frame with a stated attitude in space (`plane::Basis`), and the same two
    /// intrinsics.  The basis arrives resolved — the model stores what a plane *is*, and refusing
    /// a degenerate one is the job of whoever read it (`Basis::explicit`).
    pub fn plane(
        &mut self,
        origin: usize,
        toward: usize,
        basis: crate::plane::Basis,
        name: &str,
    ) -> usize {
        let frame = self.datum(origin, toward, name);
        self.planes.push(PlaneE { frame, basis });
        let pi = self.planes.len() - 1;
        self.slave(EntRef::plane(pi));
        pi
    }

    /// The frame half of either datum kind, so every reader of a rotor takes both.
    pub fn frame_of(&self, e: EntRef) -> &FrameE {
        match e.kind {
            EntKind::Frame => &self.frames[e.i()],
            EntKind::Plane => &self.planes[e.i()].frame,
            other => panic!("a {} has no rotor", other.as_str()),
        }
    }

    /// The rotor's two params, seeded from the chord — the half of `frame` a plane shares.
    fn datum(&mut self, origin: usize, toward: usize, name: &str) -> FrameE {
        let ((c, s), scale) = self.frame_chord(origin, toward);
        let cp = self.param_scaled(c, false, &format!("{name}.c"), scale);
        let sp = self.param_scaled(s, false, &format!("{name}.s"), scale);
        FrameE {
            origin: origin as u32,
            toward: toward as u32,
            c: cp as u32,
            s: sp as u32,
            class: Classes::default(),
        }
    }

    /// The two intrinsics that hold a datum's rotor to its chord — minted here and nowhere
    /// else, since intrinsics are never serialized.
    fn slave(&mut self, e: EntRef) {
        let c1 = Constraint::frame_unit(e);
        let c2 = Constraint::frame_align(self, e);
        self.add(c1);
        self.add(c2);
    }

    /// The rotor of the chord `origin → toward`, and the world length one unit of it is worth.
    /// A coincident pair names no direction, so it reads as the identity rotor at unit scale —
    /// the raw delta would normalise to (0, 0) and start life violating `frame_unit`.
    ///
    /// The one answer, so the rotor's `Param::scale` and the seed of the alignment's own unknown
    /// cannot come from two different rules — the same reason `constraints::contact_speed` is
    /// one function.
    pub(crate) fn frame_chord(&self, origin: usize, toward: usize) -> ((f64, f64), f64) {
        let (ox, oy) = self.point_xy(origin);
        let (tx, ty) = self.point_xy(toward);
        let d = (tx - ox).hypot(ty - oy);
        if d > 0.0 { (((tx - ox) / d, (ty - oy) / d), d) } else { ((1.0, 0.0), 1.0) }
    }

    /// A cubic B-spline over `ctrl`, with the clamped uniform knot vector.  `None` if there are
    /// too few control points for a cubic — the curve would have no span to live on.
    pub fn spline(&mut self, ctrl: &[usize]) -> Option<usize> {
        self.spline_with(ctrl, None)
    }

    /// A cubic B-spline with a knot vector of its own — a repeated interior knot is a corner.
    /// `None` if the knots are not ones this control polygon can be drawn with.
    pub fn spline_with(&mut self, ctrl: &[usize], knots: Option<Vec<f64>>) -> Option<usize> {
        if ctrl.iter().any(|&c| c >= self.points.len()) {
            return None;
        }
        let knots = knots.unwrap_or_else(|| crate::curve::clamped_uniform(ctrl.len()));
        if !crate::curve::knots_valid(&knots, ctrl.len()) {
            return None;
        }
        self.splines.push(SplineE {
            ctrl: ctrl.iter().map(|&c| c as u32).collect(),
            knots,
            class: Classes::default(),
        });
        Some(self.splines.len() - 1)
    }

    /// A cubic B-spline that passes through `pts`, in order.  The control points are computed,
    /// not clicked — the same bargain `arc_through` strikes: the third click of a three-point
    /// arc is construction input, not a sketch point.  `None` if there are too few points for a
    /// cubic, or they give no parameterisation.
    pub fn spline_through(&mut self, pts: &[(f64, f64)]) -> Option<usize> {
        self.spline_through_held(pts, &[])
    }

    /// The same, holding the curve to the places that came from a Point rather than from empty
    /// space: each becomes a `PointOnSpline` whose parameter is *pinned* at the value the fit
    /// chose for it.
    ///
    /// The pin is what makes the answer determinate.  A contact whose parameter is free says
    /// only "the curve passes through here somewhere along its length", so a curve through m
    /// points keeps m degrees of freedom — it can slide along itself and still meet every one of
    /// them.  The fit already worked out where along, so that is knowledge and not an unknown,
    /// and a curve fitted to fully constrained points comes out fully constrained.
    pub fn spline_through_held(
        &mut self,
        pts: &[(f64, f64)],
        hold: &[Option<usize>],
    ) -> Option<usize> {
        // a short `hold` holds nothing further: the two lists are one-to-one as far as it goes
        if hold.len() > pts.len() || hold.iter().flatten().any(|&p| p >= self.points.len()) {
            return None;
        }
        let (ctrl, knots, at) = crate::curve::interpolating_ctrl(pts)?;
        let ids: Vec<usize> = ctrl
            .iter()
            .enumerate()
            .map(|(i, &(x, y))| self.point(x, y, false, &format!("k{i}")))
            .collect();
        let s = self.spline_with(&ids, Some(knots))?;
        for (i, held) in hold.iter().enumerate() {
            let Some(p) = *held else { continue };
            // pinned: the fit worked out where along the curve this point sits, so that is
            // knowledge and not something to solve for
            let c = Constraint::new(
                crate::constraints::CKind::PointOnSpline,
                vec![
                    Arg::Ent(EntRef::point(p)),
                    Arg::Ent(EntRef::spline(s)),
                    Arg::Seed { value: at[i], pinned: true },
                ],
            );
            self.add(c);
        }
        Some(s)
    }

    /// Four lines round the corners `a` and (x1, y1), sharing corner points, with three
    /// perpendicular constraints.  Three, not four: the fourth follows, so adding it would make
    /// every rectangle over-constrained by one equation.  What is left is the 5 DOF a rectangle
    /// has — position, rotation, width, height.
    pub fn rectangle(&mut self, a: usize, x1: f64, y1: f64, name: &str) -> Vec<usize> {
        let (x0, y0) = self.point_xy(a);
        let corners = [
            a,
            self.point(x1, y0, false, &format!("{name}.b")),
            self.point(x1, y1, false, &format!("{name}.c")),
            self.point(x0, y1, false, &format!("{name}.d")),
        ];
        let lines: Vec<usize> =
            (0..4).map(|i| self.line(corners[i], corners[(i + 1) % 4])).collect();
        for i in 0..3 {
            let c = Constraint::two_line(
                crate::constraints::CKind::Perpendicular,
                EntRef::line(lines[i]),
                EntRef::line(lines[i + 1]),
            );
            self.add(c);
        }
        lines
    }

    pub fn rectangle_xy(&mut self, x0: f64, y0: f64, x1: f64, y1: f64, name: &str) -> Vec<usize> {
        let a = self.point(x0, y0, false, &format!("{name}.a"));
        self.rectangle(a, x1, y1, name)
    }

    /// Append a constraint, assigning it a fresh document-stable id.
    ///
    /// This is also where a constraint's own hidden unknowns become real Params: a `Param` slot
    /// arrives holding the seed number (from `constraints::seed_param`, from a document, or from
    /// the caller) and leaves holding the index of the Param that now carries it.  Doing it here
    /// and only here means a constraint is a number on the way in — which is what a document
    /// stores and what `graft` copies — and an index everywhere the solver looks at it.
    pub fn add(&mut self, c: Constraint) -> u32 {
        let expr = crate::expr::has_expr(&c.args);
        let id = self.add_quiet(c);
        if expr {
            crate::expr::evaluate(self);   // its text may read names, or define one others read
        }
        id
    }

    /// `add` without the expression pass, for a caller adding a whole document one constraint at
    /// a time and evaluating once at the end — `io::graft`, the rebuild walk behind deletion,
    /// copying, pasting and the part a drag works on.  Evaluating per add would parse every
    /// expression in the document again for each one that carries text, and would make a
    /// dimension whose definition has not been grafted yet briefly a free variable — allocating
    /// an unknown the next pass immediately retires.
    pub(crate) fn add_quiet(&mut self, mut c: Constraint) -> u32 {
        if c.id == 0 {
            self.next_cid += 1;
            c.id = self.next_cid;
        } else {
            self.next_cid = self.next_cid.max(c.id);
        }
        let id = c.id;
        for (i, name) in c.kind.param_slots() {
            if matches!(c.args[i], Arg::Param(_)) {
                continue;   // already allocated (a constraint moved between sketches)
            }
            let (v, pinned) = match c.args[i] {
                Arg::Seed { value, pinned } => (value, pinned),
                ref a => (a.num(), false),
            };
            let scale = crate::constraints::param_scale(self, c.kind, &c.args, i);
            let p = self.param_scaled(v, pinned, &format!("c{id}.{name}"), scale);
            c.args[i] = Arg::Param(p as u32);
        }
        self.constraints.push(c);
        id
    }

    /// Drop a constraint.  Its own unknowns stay in the parameter vector — every index above
    /// them names something — but are retired to `fixed`, since a free parameter no equation
    /// mentions is a degree of freedom the sketch does not actually have, and diagnosis would
    /// report it.  The rebuild walk (`io::without`) is the path that reclaims the slots.
    pub fn remove(&mut self, id: u32) {
        let mut expr = false;
        if let Some(c) = self.constraint(id) {
            expr = crate::expr::has_expr(&c.args);
            for p in c.aux_params() {
                self.params[p as usize].fixed = true;
            }
        }
        self.constraints.retain(|c| c.id != id);
        self.placements.remove(&id);
        if expr {
            // it may have defined a name others read, or been the last reader of a free one
            crate::expr::evaluate(self);
        }
    }

    pub fn constraint(&self, id: u32) -> Option<&Constraint> {
        self.constraints.iter().find(|c| c.id == id)
    }

    /// Set a numeric argument on one constraint — a dimension, a flag, a count — and bring the
    /// document's expressions back into step when the write replaced one.  Whoever sets a number
    /// means the number, so the expression goes; but it may have defined a name others read, or
    /// have been the last reader of a free variable, and neither can be left as it was.
    ///
    /// `false` when there is no such constraint or no such argument, exactly as `set_num`.
    pub fn set_constraint_num(&mut self, id: u32, name: &str, v: f64) -> bool {
        let Some(c) = self.constraint_mut(id) else { return false };
        let was = c.arg_index(name).is_some_and(|i| matches!(c.args[i], Arg::Expr(_)));
        if !c.set_num(name, v) {
            return false;
        }
        if was {
            crate::expr::evaluate(self);
        }
        true
    }

    pub fn constraint_mut(&mut self, id: u32) -> Option<&mut Constraint> {
        self.constraints.iter_mut().find(|c| c.id == id)
    }

    pub fn index_of(&self, id: u32) -> Option<usize> {
        self.constraints.iter().position(|c| c.id == id)
    }

    // -- accessors ----------------------------------------------------------

    pub fn point_xy(&self, i: usize) -> (f64, f64) {
        let p = &self.points[i];
        (self.params[p.x as usize].value, self.params[p.y as usize].value)
    }

    pub fn point_params(&self, i: usize) -> [u32; 2] {
        let p = &self.points[i];
        [p.x, p.y]
    }

    pub fn point_fixed(&self, i: usize) -> bool {
        let p = &self.points[i];
        self.params[p.x as usize].fixed && self.params[p.y as usize].fixed
    }

    pub fn fix_point(&mut self, i: usize, fixed: bool) {
        let (x, y) = (self.points[i].x as usize, self.points[i].y as usize);
        self.params[x].fixed = fixed;
        self.params[y].fixed = fixed;
    }

    pub fn line_params(&self, i: usize) -> [u32; 4] {
        let l = &self.lines[i];
        let (a, b) = (&self.points[l.p1 as usize], &self.points[l.p2 as usize]);
        [a.x, a.y, b.x, b.y]
    }

    pub fn line_dir(&self, i: usize) -> (f64, f64) {
        let l = &self.lines[i];
        let (ax, ay) = self.point_xy(l.p1 as usize);
        let (bx, by) = self.point_xy(l.p2 as usize);
        (bx - ax, by - ay)
    }

    pub fn line_length(&self, i: usize) -> f64 {
        let (dx, dy) = self.line_dir(i);
        dx.hypot(dy)
    }

    /// The Params of the control points one span of a spline reads — `ctrl[span-p ..= span]`,
    /// the only ones whose basis functions are non-zero there, in (x, y) order.  This is what
    /// keeps a contact's column count fixed however long the spline is.
    pub fn spline_span_params(&self, i: usize, span: usize) -> Vec<u32> {
        let s = &self.splines[i];
        let mut v = Vec::with_capacity(2 * crate::curve::SPAN_N);
        for a in 0..crate::curve::SPAN_N {
            let pt = &self.points[s.ctrl[span - crate::curve::DEGREE + a] as usize];
            v.push(pt.x);
            v.push(pt.y);
        }
        v
    }

    /// Centre point index of a circle, arc or ellipse.
    pub fn round_center(&self, e: EntRef) -> usize {
        match e.kind {
            EntKind::Circle => self.circles[e.i()].center as usize,
            EntKind::Arc => self.arcs[e.i()].center as usize,
            EntKind::Ellipse => self.ellipses[e.i()].center as usize,
            _ => panic!("not a round entity"),
        }
    }

    /// Radius Param index of a circle or arc — and an ellipse's minor radius, which is what its
    /// one scalar drag resizes.
    pub fn round_radius(&self, e: EntRef) -> usize {
        match e.kind {
            EntKind::Circle => self.circles[e.i()].radius as usize,
            EntKind::Arc => self.arcs[e.i()].radius as usize,
            EntKind::Ellipse => self.ellipses[e.i()].minor as usize,
            _ => panic!("not a round entity"),
        }
    }

    pub fn radius_value(&self, e: EntRef) -> f64 {
        self.params[self.round_radius(e)].value
    }

    /// Params of any primitive, in the model's canonical order.
    pub fn entity_params(&self, e: EntRef) -> Vec<u32> {
        match e.kind {
            EntKind::Point => self.point_params(e.i()).to_vec(),
            EntKind::Line => self.line_params(e.i()).to_vec(),
            EntKind::Circle => {
                let c = &self.circles[e.i()];
                let p = &self.points[c.center as usize];
                vec![p.x, p.y, c.radius]
            }
            EntKind::Arc => {
                let a = &self.arcs[e.i()];
                let mut v = Vec::with_capacity(7);
                for pi in [a.center, a.start, a.end] {
                    let p = &self.points[pi as usize];
                    v.push(p.x);
                    v.push(p.y);
                }
                v.push(a.radius);
                v
            }
            EntKind::Spline => {
                let s = &self.splines[e.i()];
                let mut v = Vec::with_capacity(2 * s.ctrl.len());
                for &c in &s.ctrl {
                    let p = &self.points[c as usize];
                    v.push(p.x);
                    v.push(p.y);
                }
                v
            }
            EntKind::Ellipse => {
                let el = &self.ellipses[e.i()];
                let mut v = Vec::with_capacity(5);
                for pi in [el.center, el.major] {
                    let p = &self.points[pi as usize];
                    v.push(p.x);
                    v.push(p.y);
                }
                v.push(el.minor);
                v
            }
            EntKind::Frame | EntKind::Plane => {
                let f = self.frame_of(e);
                let mut v = Vec::with_capacity(6);
                for pi in [f.origin, f.toward] {
                    let p = &self.points[pi as usize];
                    v.push(p.x);
                    v.push(p.y);
                }
                v.push(f.c);
                v.push(f.s);
                v
            }
            // whatever its arguments contribute, in argument order — which is the order its
            // tapes were compiled against and so the order of the Jacobian's columns
            EntKind::Curve => {
                let mut v = Vec::new();
                for &a in &self.curves[e.i()].args {
                    v.extend(self.entity_params(a));
                }
                v
            }
        }
    }

    /// The parameters an entity owns *itself*: the ones `entity_params` has that its children do
    /// not.  A point's coordinates, a circle's or an arc's radius, an ellipse's minor — exactly
    /// the `Scalar` fields of `EntKind::fields`, and exactly what a declaration seeds and a solve
    /// may write back into one.
    ///
    /// Exhaustive on purpose, like `min_children`: a new entity kind with a number of its own
    /// must stop the build here, or its number would be a value nothing ever writes down.
    pub fn own_params(&self, e: EntRef) -> Vec<u32> {
        match e.kind {
            EntKind::Point => self.point_params(e.i()).to_vec(),
            EntKind::Circle => vec![self.circles[e.i()].radius],
            EntKind::Arc => vec![self.arcs[e.i()].radius],
            EntKind::Ellipse => vec![self.ellipses[e.i()].minor],
            EntKind::Frame | EntKind::Plane => {
                let f = self.frame_of(e);
                vec![f.c, f.s]
            }
            // a curve holds no number of its own: it is its expressions, and they read
            // the geometry rather than owning any
            EntKind::Line | EntKind::Spline | EntKind::Curve => Vec::new(),
        }
    }

    /// Sub-entities: a line's endpoints, an arc's centre and ends.
    pub fn children(&self, e: EntRef) -> Vec<EntRef> {
        match e.kind {
            EntKind::Point => Vec::new(),
            EntKind::Line => {
                let l = &self.lines[e.i()];
                vec![EntRef::point(l.p1 as usize), EntRef::point(l.p2 as usize)]
            }
            EntKind::Circle => vec![EntRef::point(self.circles[e.i()].center as usize)],
            EntKind::Arc => {
                let a = &self.arcs[e.i()];
                vec![
                    EntRef::point(a.center as usize),
                    EntRef::point(a.start as usize),
                    EntRef::point(a.end as usize),
                ]
            }
            EntKind::Spline => {
                self.splines[e.i()].ctrl.iter().map(|&c| EntRef::point(c as usize)).collect()
            }
            EntKind::Ellipse => {
                let el = &self.ellipses[e.i()];
                vec![EntRef::point(el.center as usize), EntRef::point(el.major as usize)]
            }
            EntKind::Frame | EntKind::Plane => {
                let f = self.frame_of(e);
                vec![EntRef::point(f.origin as usize), EntRef::point(f.toward as usize)]
            }
            // the one kind whose children need not be points
            EntKind::Curve => self.curves[e.i()].args.clone(),
        }
    }

    /// The fewest children an entity can still be rebuilt from.  For everything defined by a
    /// fixed set of points that is all of them — a line without an endpoint is nothing.  A
    /// spline is defined by a *list*, so it survives losing one control point while enough are
    /// left to draw a curve with.
    /// Takes the children rather than fetching them, because the one caller has just built the
    /// list to count the survivors in and would otherwise allocate it twice per entity.
    pub fn min_children(&self, e: EntRef, children: &[EntRef]) -> usize {
        // exhaustive on purpose: a new list-shaped entity must stop the build here, or it would
        // inherit the point-shaped answer and be deleted whole instead of shortened
        match e.kind {
            EntKind::Spline => crate::curve::MIN_CTRL,
            EntKind::Point | EntKind::Line | EntKind::Circle | EntKind::Arc
            | EntKind::Ellipse | EntKind::Frame | EntKind::Plane | EntKind::Curve => {
                children.len()
            }
        }
    }

    /// What a compiled plan or `System` depends on: which entities exist, which constraints (by
    /// id, so swapping one Distance for another shows up — counts and type names alone do not)
    /// and which params are fixed.  A cache over compiled artefacts keys on this.
    pub fn topology_key(&self) -> String {
        use std::fmt::Write;
        let mut s = format!(
            "{}|{}|{}|{}|{}|{}|{}|",
            self.points.len(),
            self.lines.len(),
            self.circles.len(),
            self.arcs.len(),
            self.ellipses.len(),
            self.frames.len(),
            self.planes.len()
        );
        for sp in &self.splines {
            let _ = write!(s, "s{}:", sp.ctrl.len());
            for k in &sp.knots {
                let _ = write!(s, "{k},");
            }
        }
        s.push('|');
        for c in &self.constraints {
            // a claim compiles to no rows, so claiming a relation and stating it are different
            // topologies even though the constraint list reads the same
            let _ = write!(s, "{}:{}{},", c.id, c.type_name(), if c.claim { "?" } else { "" });
            // A constraint whose columns are not fixed by its entities alone writes them out:
            // which span of a spline a contact sits on, and which unknown a dimension written in
            // terms of a free variable is tied to.  Both are compiled into the plan, so both
            // belong in the key — a contact walking past a knot is a recompile, and so is
            // swapping one free name for another, which leaves the parameter vector exactly as
            // it was.  Not the constants (a dimension's `m` and `c`): a compiled system re-reads
            // those without being rebuilt.
            if c.kind.contact_slots().is_some() || c.free.is_some() {
                for p in c.params(self) {
                    let _ = write!(s, "{p}.");
                }
            }
        }
        s.push('|');
        for p in &self.params {
            s.push(if p.fixed { '1' } else { '0' });
        }
        s
    }

    /// The variable vector a curve's tapes are evaluated at: the parameter, then every scalar
    /// its arguments contribute in `entity_params` order, then the numbers it was given.
    ///
    /// That order is the kernel's column order too, which is what lets a tape's gradient *be* a
    /// row of the Jacobian rather than something a kernel has to rearrange.  The instance's own
    /// values come last precisely because they are not columns: they are constants of this
    /// curve, and the gradient in them is computed and ignored.
    pub fn curve_vars(&self, i: usize, u: f64) -> Vec<f64> {
        let mut v = Vec::with_capacity(8);
        v.push(u);
        for p in self.entity_params(EntRef::new(EntKind::Curve, i)) {
            v.push(self.params[p as usize].value);
        }
        v.extend(self.curves[i].values.iter().copied());
        v
    }

    /// Where a curve is at `u`.
    pub fn curve_point(&self, i: usize, u: f64) -> (f64, f64) {
        let d = &self.curve_defs[self.curves[i].def as usize];
        let x = self.curve_vars(i, u);
        match &d.body {
            CurveBody::Exprs { x: tx, y: ty } => {
                let mut s = crate::tape::Scratch::new();
                (tx.eval(&x, &mut s).v, ty.eval(&x, &mut s).v)
            }
            CurveBody::Trace(l) => MODEL_LOCUS.with(|s| {
                let s = &mut *s.borrow_mut();
                let pose = self.curve_pose(i);
                let anchor = crate::locus::Anchor { u: self.curve_home(i), pose: pose.as_deref() };
                let v = crate::locus::eval_flat(&l.flat, &x, anchor, s);
                (v.x, v.y)
            }),
        }
    }

    pub fn curve_domain(&self, i: usize) -> (f64, f64) {
        self.curves[i].domain
    }

    /// The parameter a curve's trace is anchored at — the drawing's unknown where the swept
    /// formal is one, else the number the instance gave.  An unknown no dimension has read yet
    /// is not allocated, and the interval's start stands in until it is.
    pub fn curve_home(&self, i: usize) -> f64 {
        let cv = &self.curves[i];
        match &cv.home {
            Home::At(u) => *u,
            Home::Free(n) => self
                .free_vars
                .get(n)
                .map(|&p| self.params[p as usize].value)
                .unwrap_or(cv.domain.0),
        }
    }

    /// The parameter at which a curve's polyline comes nearest by some measure — where a fresh
    /// contact starts: nearest a point for a contact, nearest a line for a tangency.  Over the
    /// drawn polyline, which is what a person points at, and to its resolution; the solve does
    /// the rest.
    pub fn curve_nearest_by(&self, i: usize, dist: impl Fn(f64, f64) -> f64) -> f64 {
        let (a, b) = self.curve_domain(i);
        let poly = self.curve_polyline(i);
        let n = poly.len().saturating_sub(1).max(1);
        poly.iter()
            .enumerate()
            .map(|(k, &(px, py))| (dist(px, py), k))
            .min_by(|p, q| p.0.total_cmp(&q.0))
            .map(|(_, k)| a + (b - a) * k as f64 / n as f64)
            .unwrap_or(a)
    }

    /// The pose a curve's trace is anchored at, read off the sheet — `None` for a curve whose
    /// instance is not drawn, or whose pose is not whole.
    pub fn curve_pose(&self, i: usize) -> Option<Vec<f64>> {
        let cv = &self.curves[i];
        if cv.pose.is_empty() {
            return None;
        }
        cv.pose
            .iter()
            .map(|&(e, j)| self.own_param(e, j).map(|p| self.params[p as usize].value))
            .collect()
    }

    /// One of an entity's own scalars — the `j`th of `own_params` — without the list.
    pub fn own_param(&self, e: EntRef, j: usize) -> Option<u32> {
        self.own_params(e).get(j).copied()
    }

    /// The curve as a polyline, for measuring and for drawing.  Uniform in the parameter: a
    /// user-written curve has no basis to refine against, so evenly is the only honest default,
    /// and `CURVE_STEPS` is chosen fine enough that a pick test does not lie.  A trace family is
    /// one march across the domain, each sample warm-started from the last — which is also what
    /// carries its branch along the curve.
    pub fn curve_polyline(&self, i: usize) -> Vec<(f64, f64)> {
        let (a, b) = self.curve_domain(i);
        // what the polyline is a function of: the curve's variables at the interval's start
        // (the parameter first, then every coordinate it reads), the interval, the anchor and
        // the anchor pose — the same reading `curve_point` and `sweep` take
        let mut key = self.curve_vars(i, a);
        key.extend([b, self.curve_home(i)]);
        key.extend(self.curve_pose(i).unwrap_or_default());
        if let Some((k, poly)) = self.polyline_cache.borrow().get(&i) {
            if *k == key {
                return poly.clone();
            }
        }
        let poly = self.curve_polyline_uncached(i, a, b);
        self.polyline_cache.borrow_mut().insert(i, (key, poly.clone()));
        poly
    }

    fn curve_polyline_uncached(&self, i: usize, a: f64, b: f64) -> Vec<(f64, f64)> {
        let d = &self.curve_defs[self.curves[i].def as usize];
        match &d.body {
            CurveBody::Exprs { x: tx, y: ty } => {
                let mut s = crate::tape::Scratch::new();
                (0..=CURVE_STEPS)
                    .map(|k| {
                        let u = a + (b - a) * k as f64 / CURVE_STEPS as f64;
                        let x = self.curve_vars(i, u);
                        (tx.eval(&x, &mut s).v, ty.eval(&x, &mut s).v)
                    })
                    .collect()
            }
            CurveBody::Trace(l) => MODEL_LOCUS.with(|s| {
                let pose = self.curve_pose(i);
                let anchor = crate::locus::Anchor { u: self.curve_home(i), pose: pose.as_deref() };
                crate::locus::sweep(&l.flat, &self.curve_vars(i, a), a, b, CURVE_STEPS, anchor,
                                    &mut s.borrow_mut())
            }),
        }
    }

    pub fn count(&self, kind: EntKind) -> usize {
        match kind {
            EntKind::Point => self.points.len(),
            EntKind::Line => self.lines.len(),
            EntKind::Circle => self.circles.len(),
            EntKind::Arc => self.arcs.len(),
            EntKind::Spline => self.splines.len(),
            EntKind::Ellipse => self.ellipses.len(),
            EntKind::Frame => self.frames.len(),
            EntKind::Plane => self.planes.len(),
            EntKind::Curve => self.curves.len(),
        }
    }

    /// Every entity, in creation order per kind.
    pub fn primitives(&self) -> Vec<EntRef> {
        let mut out = Vec::new();
        for kind in [
            EntKind::Point,
            EntKind::Line,
            EntKind::Circle,
            EntKind::Arc,
            EntKind::Spline,
            EntKind::Ellipse,
            EntKind::Frame,
            EntKind::Plane,
        ] {
            for i in 0..self.count(kind) {
                out.push(EntRef::new(kind, i));
            }
        }
        out
    }

    // -- parameter vector ---------------------------------------------------

    pub fn get_x(&self) -> Vec<f64> {
        self.params.iter().map(|p| p.value).collect()
    }

    /// Write the parameter vector.  A vector of the wrong length is not this sketch's — writing
    /// the overlapping prefix would scatter one sketch's coordinates over another's — so it is
    /// refused; `false` says nothing was written.
    /// Write a whole parameter vector back — the one seam every solve and every drag comes
    /// through, which is why the dimensions written in terms of a free variable are brought up
    /// to date here: their number is that unknown's, and a reader of the drawing must not be
    /// shown the one it had before the solve moved it.
    pub fn set_x(&mut self, x: &[f64]) -> bool {
        if x.len() != self.params.len() {
            return false;
        }
        for (i, p) in self.params.iter_mut().enumerate() {
            p.value = x[i];
        }
        crate::expr::sync_free(self);
        true
    }

    pub fn free_indices(&self) -> Vec<i32> {
        self.params
            .iter()
            .enumerate()
            .filter(|(_, p)| !p.fixed)
            .map(|(i, _)| i as i32)
            .collect()
    }

    pub fn n_residuals(&self) -> usize {
        self.constraints.iter().filter(|c| !c.claim).map(|c| c.n_residuals()).sum()
    }

    /// Constraints the user added (excludes intrinsic and soft/transient ones).
    pub fn user_constraints(&self) -> Vec<&Constraint> {
        self.constraints.iter().filter(|c| !(c.intrinsic || c.soft)).collect()
    }

    /// Everything that must be satisfied: excludes soft ones such as drag targets, and claims,
    /// which are judged rather than satisfied.  This is the named half of the rule the solve
    /// seams spell out inline — a caller that only wants the list asks here, so a consumer added
    /// later inherits both exclusions instead of having to remember them.
    pub fn hard_constraints(&self) -> Vec<&Constraint> {
        self.constraints.iter().filter(|c| c.acts()).collect()
    }

    pub fn hard_ids(&self) -> Vec<u32> {
        self.constraints.iter().filter(|c| c.acts()).map(|c| c.id).collect()
    }

    // -- geometry -----------------------------------------------------------

    pub fn arc_angles(&self, i: usize) -> (f64, f64) {
        let a = &self.arcs[i];
        let (cx, cy) = self.point_xy(a.center as usize);
        let (sx, sy) = self.point_xy(a.start as usize);
        let (ex, ey) = self.point_xy(a.end as usize);
        let a0 = (sy - cy).atan2(sx - cx);
        let mut a1 = (ey - cy).atan2(ex - cx);
        if a1 <= a0 {
            a1 += 2.0 * std::f64::consts::PI;
        }
        (a0, a1)
    }

    /// The points that bound the drawn sweep: its two ends, plus every quarter-turn direction the
    /// sweep passes through.
    pub fn arc_extremes(&self, i: usize) -> Vec<(f64, f64)> {
        let a = &self.arcs[i];
        let (cx, cy) = self.point_xy(a.center as usize);
        let r = self.params[a.radius as usize].value.abs();
        let (a0, a1) = self.arc_angles(i);
        let at = |th: f64| (cx + r * th.cos(), cy + r * th.sin());
        let mut out = vec![at(a0), at(a1)];
        let quarter = std::f64::consts::FRAC_PI_2;
        let mut k = (a0 / quarter).ceil();
        while k * quarter < a1 {
            out.push(at(k * quarter));
            k += 1.0;
        }
        out
    }

    pub fn bounds(&self, e: EntRef) -> Box2 {
        match e.kind {
            EntKind::Curve => {
                let mut b = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
                for (x, y) in self.curve_polyline(e.i()) {
                    b = (b.0.min(x), b.1.min(y), b.2.max(x), b.3.max(y));
                }
                b
            }
            EntKind::Point => {
                let (x, y) = self.point_xy(e.i());
                (x, y, x, y)
            }
            EntKind::Line => {
                let l = &self.lines[e.i()];
                let (ax, ay) = self.point_xy(l.p1 as usize);
                let (bx, by) = self.point_xy(l.p2 as usize);
                (ax.min(bx), ay.min(by), ax.max(bx), ay.max(by))
            }
            EntKind::Circle => {
                let c = &self.circles[e.i()];
                let (cx, cy) = self.point_xy(c.center as usize);
                let r = self.params[c.radius as usize].value.abs();
                (cx - r, cy - r, cx + r, cy + r)
            }
            EntKind::Ellipse => crate::ellipse::bounds(self, e.i()),
            EntKind::Frame | EntKind::Plane => {
                let f = self.frame_of(e);
                let (ax, ay) = self.point_xy(f.origin as usize);
                let (bx, by) = self.point_xy(f.toward as usize);
                (ax.min(bx), ay.min(by), ax.max(bx), ay.max(by))
            }
            EntKind::Arc | EntKind::Spline => {
                let pts = if e.kind == EntKind::Arc {
                    self.arc_extremes(e.i())
                } else {
                    crate::curve::sample(self, e.i(), 16)
                };
                let mut b = (f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
                for (x, y) in pts {
                    b.0 = b.0.min(x);
                    b.1 = b.1.min(y);
                    b.2 = b.2.max(x);
                    b.3 = b.3.max(y);
                }
                b
            }
        }
    }

    /// (xmin, ymin, xmax, ymax) over all points.  Points only, deliberately: `extent()` is built
    /// on this, and `extent()` scales the solver's residual tolerances, the violated-constraint
    /// threshold, the witness perturbation and the drag continuation step.
    pub fn bbox(&self) -> Box2 {
        if self.points.is_empty() {
            return (0.0, 0.0, 1.0, 1.0);
        }
        let mut b = (f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
        for i in 0..self.points.len() {
            let (x, y) = self.point_xy(i);
            b.0 = b.0.min(x);
            b.1 = b.1.min(y);
            b.2 = b.2.max(x);
            b.3 = b.3.max(y);
        }
        b
    }

    /// Everything with something to draw: `primitives()` and the curves after it.
    ///
    /// A curve is written *over* the other kinds and so is built and grafted last, which is why
    /// `primitives()` stops short of it — but a consumer that means "draw the drawing" wants
    /// both, and this is where that is said.  Written once because it was already being patched
    /// up locally by everything that needed it.
    pub fn drawn(&self) -> Vec<EntRef> {
        let mut v = self.primitives();
        v.extend((0..self.curves.len()).map(|i| EntRef::new(EntKind::Curve, i)));
        v
    }

    /// Bounds of the drawn primitives — what a "fit the view" wants.
    ///
    /// **Curves are not in it, and that is a cost decision.**  A curve's `bounds` is its
    /// polyline, which for a traced family is a damped-Newton march per point; this runs inside
    /// `callout::layout`, so it is paid on every repaint.  Measured on `gear_trace` (24 traced
    /// curves) that is 11 ms a call against 1 µs — four orders of magnitude, per frame, to
    /// square up a box.  A caller that needs the curves in its box and is already sweeping them
    /// (the SVG export sizes a page from the polylines it is about to draw) grows this by what
    /// it swept; one that is not should not start.
    pub fn drawn_bounds(&self) -> Box2 {
        self.bounds_of(&self.primitives()).unwrap_or_else(|| self.bbox())
    }

    /// The box round a given set of entities, or `None` when the set draws nothing.  The fold
    /// `drawn_bounds` and the SVG export's page both want, so neither writes it out.
    pub fn bounds_of(&self, ents: &[EntRef]) -> Option<Box2> {
        let mut b = (f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
        for &e in ents {
            let x = self.bounds(e);
            b.0 = b.0.min(x.0);
            b.1 = b.1.min(x.1);
            b.2 = b.2.max(x.2);
            b.3 = b.3.max(x.3);
        }
        b.0.is_finite().then_some(b)
    }

    /// Characteristic length of the sketch (tolerances, drag weights).
    pub fn extent(&self) -> f64 {
        let (x0, y0, x1, y1) = self.bbox();
        (x1 - x0).max(y1 - y0).max(1.0)
    }

    /// Seeded Gaussian noise on every free parameter (warm starts, witness construction).
    pub fn perturb(&mut self, sigma: f64, seed: u32) {
        let mut rng = Rng::new(seed);
        for p in self.params.iter_mut() {
            if !p.fixed {
                p.value += rng.normal(0.0, sigma) / p.scale;
            }
        }
    }

    pub fn nearest_point(&self, x: f64, y: f64) -> (Option<usize>, f64) {
        let mut best = None;
        let mut bd = f64::INFINITY;
        for i in 0..self.points.len() {
            let (px, py) = self.point_xy(i);
            let d = (px - x).hypot(py - y);
            if d < bd {
                best = Some(i);
                bd = d;
            }
        }
        (best, bd)
    }
}

/// Signed perpendicular offset from the *infinite* line through `line`, positive to the left of
/// its direction.  A degenerate line has no side; it gives infinity rather than a silent zero.
pub fn signed_point_to_line(sk: &Sketch, px: f64, py: f64, line: usize) -> f64 {
    let l = &sk.lines[line];
    let (ax, ay) = sk.point_xy(l.p1 as usize);
    let (dx, dy) = sk.line_dir(line);
    let length = dx.hypot(dy);
    if length == 0.0 {
        return f64::INFINITY;
    }
    (dx * (py - ay) - dy * (px - ax)) / length
}

/// How far (px, py) is from an entity — the one implementation, so the pair dispatch below and
/// the sweep along a curve cannot drift apart.  A point against a curve is exact: the projection
/// is a Newton solve the core does anyway, and this is the number a reader checks the drawing
/// against.
fn point_to(sk: &Sketch, px: f64, py: f64, e: EntRef) -> f64 {
    match e.kind {
        // a curve has no idealised form a dimension could mean beyond the curve itself, so this
        // measurement and `point_to_drawn`'s are the same one
        EntKind::Curve => polyline_distance(&sk.curve_polyline(e.i()), px, py),
        EntKind::Point => {
            let (x, y) = sk.point_xy(e.i());
            (px - x).hypot(py - y)
        }
        EntKind::Line => point_to_line(sk, px, py, e.i()),
        EntKind::Circle | EntKind::Arc => {
            let (cx, cy) = sk.point_xy(sk.round_center(e));
            ((px - cx).hypot(py - cy) - sk.radius_value(e).abs()).abs()
        }
        EntKind::Spline => crate::curve::distance_to(sk, e.i(), px, py),
        EntKind::Ellipse => crate::ellipse::distance_to(sk, e.i(), px, py),
        // a datum is not a figure: the place it stands at is its origin
        EntKind::Frame | EntKind::Plane => {
            let (x, y) = sk.point_xy(sk.frame_of(e).origin as usize);
            (px - x).hypot(py - y)
        }
    }
}

fn point_to_line(sk: &Sketch, px: f64, py: f64, line: usize) -> f64 {
    let (dx, dy) = sk.line_dir(line);
    if dx == 0.0 && dy == 0.0 {
        let l = &sk.lines[line];
        let (ax, ay) = sk.point_xy(l.p1 as usize);
        return (px - ax).hypot(py - ay);
    }
    signed_point_to_line(sk, px, py, line).abs()
}

/// How far (px, py) is from what is *drawn* of `e`: the segment a line is drawn as, the sweep an
/// arc is drawn as, the curve itself.  `point_to` measures the entity a *dimension* means — a
/// line is infinite, an arc is the whole circle it lies on — which is not what a pointer hits.
pub fn point_to_drawn(sk: &Sketch, px: f64, py: f64, e: EntRef) -> f64 {
    match e.kind {
        EntKind::Curve => polyline_distance(&sk.curve_polyline(e.i()), px, py),
        EntKind::Line => {
            let l = &sk.lines[e.i()];
            let (a, b) = (sk.point_xy(l.p1 as usize), sk.point_xy(l.p2 as usize));
            seg_distance((px, py), a, b)
        }
        EntKind::Arc => {
            let (cx, cy) = sk.point_xy(sk.round_center(e));
            let r = sk.radius_value(e).abs();
            let (a0, a1) = sk.arc_angles(e.i());
            let mut th = (py - cy).atan2(px - cx);
            if th < a0 {
                th += 2.0 * std::f64::consts::PI;     // arc_angles keeps a1 within one turn of a0
            }
            if th <= a1 {
                return ((px - cx).hypot(py - cy) - r).abs();
            }
            // off the ends of the sweep: the nearer end of what was drawn, not the phantom
            // remainder of the circle
            let at = |t: f64| (cx + r * t.cos(), cy + r * t.sin());
            let (sx, sy) = at(a0);
            let (ex, ey) = at(a1);
            (px - sx).hypot(py - sy).min((px - ex).hypot(py - ey))
        }
        // the kinds whose drawn figure *is* the entity: a point, a whole ring or rim, and a
        // curve that `curve::distance_to` already keeps between its own knots
        EntKind::Point | EntKind::Circle | EntKind::Spline | EntKind::Ellipse => {
            point_to(sk, px, py, e)
        }
        // a frame draws nothing of its own — its points are the click targets — so it can
        // never be the nearest drawn thing and a pick can never land on it
        EntKind::Frame => f64::INFINITY,
        // a plane draws its chord as a datum glyph, and that is where it is taken hold of; its
        // points still win a pick within tolerance, as every point does
        EntKind::Plane => {
            let f = sk.frame_of(e);
            let (a, b) = (sk.point_xy(f.origin as usize), sk.point_xy(f.toward as usize));
            seg_distance((px, py), a, b)
        }
    }
}

/// Distance from a point to a segment — the drawn figure of a line, and the flat side of every
/// callout box, so both ask here rather than each keeping the projection formula.
pub fn seg_distance(p: (f64, f64), a: (f64, f64), b: (f64, f64)) -> f64 {
    let (vx, vy) = (b.0 - a.0, b.1 - a.1);
    let l2 = vx * vx + vy * vy;
    if l2 <= 0.0 {
        return (p.0 - a.0).hypot(p.1 - a.1);
    }
    let t = (((p.0 - a.0) * vx + (p.1 - a.1) * vy) / l2).clamp(0.0, 1.0);
    (p.0 - (a.0 + t * vx)).hypot(p.1 - (a.1 + t * vy))
}

/// What a click at (x, y) picks: the nearest entity whose drawn figure comes within `tol`, or
/// nothing.  A point within reach wins outright, however much nearer an edge passes — a point is
/// what most of a sketcher's verbs are about, and it is the smaller target of the two.  The
/// tolerance is a world length, so a front end scales it by what one screen pixel is worth and
/// keeps no geometry of its own.
pub fn pick(sk: &Sketch, x: f64, y: f64, tol: f64) -> Option<EntRef> {
    if let (Some(i), d) = sk.nearest_point(x, y) {
        if d <= tol {
            return Some(EntRef::point(i));
        }
    }
    let mut best: Option<(EntRef, f64)> = None;
    // what is drawn, curves included: a pick measures the figure on the sheet, and a curve
    // written in the language is one (`drawn`, where `primitives` stops short of it)
    for e in sk.drawn() {
        if e.kind == EntKind::Point {
            continue;
        }
        let d = point_to_drawn(sk, x, y, e);
        if d <= tol && best.map_or(true, |(_, bd)| d < bd) {
            best = Some((e, d));
        }
    }
    best.map(|(e, _)| e)
}

fn measure_order(k: EntKind) -> u8 {
    match k {
        EntKind::Point => 0,
        EntKind::Line => 1,
        EntKind::Circle | EntKind::Arc => 2,
        EntKind::Spline => 3,
        EntKind::Ellipse => 4,
        EntKind::Curve => 5,
        // last, so any pair with a datum in it puts the datum second and one arm catches it
        EntKind::Frame | EntKind::Plane => 6,
    }
}

/// The nearest a polyline comes to a point.
fn polyline_distance(pts: &[(f64, f64)], px: f64, py: f64) -> f64 {
    let mut best = f64::MAX;
    for w in pts.windows(2) {
        best = best.min(seg_distance((px, py), w[0], w[1]));
    }
    if pts.len() == 1 {
        best = best.min((px - pts[0].0).hypot(py - pts[0].1));
    }
    best
}

/// Signed CCW angle from line `a` to line `b`, in radians — what an `Angle` constraint's value
/// means, and what a dimension dialog should offer as the current value.
pub fn angle_between(sk: &Sketch, a: EntRef, b: EntRef) -> f64 {
    let (d1x, d1y) = sk.line_dir(a.i());
    let (d2x, d2y) = sk.line_dir(b.i());
    (d1x * d2y - d1y * d2x).atan2(d1x * d2x + d1y * d2y)
}

/// The point at distance `r` from (cx, cy) in the direction of (tx, ty).  The centre–start–end
/// arc construction: the third click gives a direction, and the radius comes from the second.
/// `None` when the target is the centre, which names no direction.
pub fn on_radius(cx: f64, cy: f64, tx: f64, ty: f64, r: f64) -> Option<(f64, f64)> {
    let (dx, dy) = (tx - cx, ty - cy);
    let l = dx.hypot(dy);
    if l <= 1e-12 {
        return None;
    }
    Some((cx + r * dx / l, cy + r * dy / l))
}

/// Points along whatever is drawn of a kind that has no closed form to measure against — a
/// curve's tessellation, an ellipse's rim.  One reader, so the arm in `distance_between` that
/// sweeps is one arm and does not have to know which family it is sweeping.
fn swept(sk: &Sketch, e: EntRef) -> Vec<(f64, f64)> {
    match e.kind {
        EntKind::Spline => crate::curve::sample(sk, e.i(), 64),
        EntKind::Ellipse => crate::ellipse::sample(sk, e.i(), 64),
        // a datum is not a figure: the one place it stands at
        EntKind::Frame | EntKind::Plane => vec![sk.point_xy(sk.frame_of(e).origin as usize)],
        _ => Vec::new(),
    }
}

/// Shortest distance between two entities, as a sketcher measures it.  Lines are treated as
/// infinite; arcs are measured as the whole circle they lie on.
pub fn distance_between(sk: &Sketch, first: EntRef, second: EntRef) -> f64 {
    let (a, b) = if measure_order(first.kind) > measure_order(second.kind) {
        (second, first)
    } else {
        (first, second)
    };
    match a.kind {
        EntKind::Point => {
            let (ax, ay) = sk.point_xy(a.i());
            point_to(sk, ax, ay, b)
        }
        // A curve and an ellipse have no closed form against any of the others, so both are
        // measured by sweeping what is drawn — close enough to measure by, and honestly the best
        // a sampled answer can be.  A frame joins them with a single sample, its origin: not for
        // want of a closed form but because a datum is measured where it stands.  All three sort
        // after everything they could be paired with, so the swept one is always `b`, and the
        // exact point case has already short-circuited above.
        //
        // This sits *above* the arms that reach for a centre and a radius, which is the whole
        // reason it is one arm and not one per family: a sampled kind that fell through to them
        // would ask a curve for a centre it does not have.
        _ if matches!(
            b.kind,
            EntKind::Spline | EntKind::Ellipse | EntKind::Frame | EntKind::Plane
        ) => swept(sk, b)
            .into_iter()
            .map(|(x, y)| point_to(sk, x, y, a))
            .fold(f64::INFINITY, f64::min),
        EntKind::Line => match b.kind {
            EntKind::Line => {
                let d1 = sk.line_dir(a.i());
                let d2 = sk.line_dir(b.i());
                let cross = d1.0 * d2.1 - d1.1 * d2.0;
                if cross.abs() > 1e-9 * d1.0.hypot(d1.1) * d2.0.hypot(d2.1) {
                    return 0.0; // they meet somewhere
                }
                let l = &sk.lines[b.i()];
                let (px, py) = sk.point_xy(l.p1 as usize);
                point_to_line(sk, px, py, a.i())
            }
            _ => {
                let (cx, cy) = sk.point_xy(sk.round_center(b));
                (point_to_line(sk, cx, cy, a.i()) - sk.radius_value(b).abs()).max(0.0)
            }
        },
        _ => {
            // outside each other, or one inside the other; overlapping rings give 0
            let (ax, ay) = sk.point_xy(sk.round_center(a));
            let (bx, by) = sk.point_xy(sk.round_center(b));
            let gap = (ax - bx).hypot(ay - by);
            let (r1, r2) = (sk.radius_value(a).abs(), sk.radius_value(b).abs());
            (gap - r1 - r2).max((r1 - r2).abs() - gap).max(0.0)
        }
    }
}

/// Entities plus their sub-entities.
pub fn expand(sk: &Sketch, ents: &[EntRef]) -> Vec<EntRef> {
    let mut out = Vec::new();
    for &e in ents {
        out.push(e);
        out.extend(sk.children(e));
    }
    out
}

/// Twice the signed area of (a, b, c) — the order-type invariant the drag guards.
pub fn orientation(sk: &Sketch, a: usize, b: usize, c: usize) -> f64 {
    let (ax, ay) = sk.point_xy(a);
    let (bx, by) = sk.point_xy(b);
    let (cx, cy) = sk.point_xy(c);
    orientation_xy(ax, ay, bx, by, cx, cy)
}

/// The same from bare coordinates — one formula, so this reading and a trace predicate's
/// (`locus::holds`) cannot fork on the sign convention.
pub fn orientation_xy(ax: f64, ay: f64, bx: f64, by: f64, cx: f64, cy: f64) -> f64 {
    (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
}

/// Continuation path from (x0, y0) to (x1, y1): waypoints no farther apart than `max_step`, so a
/// solution tracks its branch instead of teleporting across it.  Always at least one point.
pub fn increments(
    x0: f64,
    y0: f64,
    x1: f64,
    y1: f64,
    max_step: f64,
) -> Vec<(f64, f64)> {
    let n = (((x1 - x0).hypot(y1 - y0) / max_step).ceil() as i64).max(1);
    (1..=n)
        .map(|i| {
            let t = i as f64 / n as f64;
            (x0 + (x1 - x0) * t, y0 + (y1 - y0) * t)
        })
        .collect()
}
