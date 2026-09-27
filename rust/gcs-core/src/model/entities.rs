//! Entity identities, declaration metadata and planar storage.

use super::*;

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
    /// **The one datum kind** (issue #47, item 6: `frame` is folded into it).  An origin and an
    /// attitude other statements measure from: the attitude is a unit rotor — two scalars
    /// `(c, s)` held to `c² + s² = 1` by an intrinsic constraint, the 2D form of the quaternion
    /// a 3D workplane will want — kept pointed from `origin` at `toward` by a second intrinsic,
    /// so the rotor is a first-class unknown that adds no freedom beyond the two points it is
    /// slaved to.  A trace block reads `f.angle` (`atan2(s, c)`, degrees — derived in
    /// `Tape::compile`, never stored) to state a bearing relative to the datum instead of the
    /// page.  And it is a *view*: it carries a constant 3D attitude (`plane::Basis`) saying
    /// which plane in space the picture drawn in it is of — the page's, where none is written,
    /// which is what a plain datum is.
    /// A point that says it is `in` the plane is an image of something on that plane, and
    /// `Project` between two such points is the one equation two images of one point share.
    /// A stated attitude is document data, like a spline's knots; only a view asked to be
    /// solved (`Sketch::free_attitude`, which mints `PlaneE::att`) has unknowns in space.
    Plane,
    /// A curve written in the language: `C(u)` as an expression over the geometry it is drawn
    /// from.  Unlike every other kind it holds no coordinates of its own — it *is* the two
    /// expressions plus whatever it reads — so it moves when its arguments do and never
    /// otherwise.  See `CurveDef`.
    Curve,
    /// **A region of a plane**: a closed loop of edges the document already drew (Solvent §6.8).
    /// It owns nothing — no coordinate, no class of its own beyond presentation — and aliases
    /// its edges, so `sec.mouth` and `mouth` are one line.  What a solid is swept from.
    Face,
    /// **A solid** (Solvent §6.9): a face swept along its plane's normal or about a line in its
    /// plane, or a term over other solids — its stock, plus everything `on` it, minus everything
    /// that `cut`s it.
    ///
    /// It is an entity because a document *names* it and reaches its faces by path
    /// (`body.bore.axis`), and it is like `Curve` in owning no coordinates: it is the term plus
    /// whatever the term reads.  **Nothing about a solid is ever solved for** — every extent is
    /// an expression, evaluated after the drawing is solved, which is what keeps the whole
    /// engine planar while the document describes an object.
    Solid,
    /// An exact analytic patch read from a named boundary of an evaluated solid.
    /// It owns no solver coordinates; changing the source sketch changes the patch.
    Surface,
    /// A family of rigid poses over one shared angular parameter, evaluated after solving.
    Motion,
    /// The zero-normal-velocity locus of a surface under a named motion.
    Envelope,
    /// A spatial surface region selected by material-side constraints.
    Patch,
    /// A generating junction or a generated face intersected by a finite boundary.
    Seam,
    /// A spatial corner defined by named seams, evaluated after solving.
    Vertex,
    /// A finite directed portion of a spatial seam between named corners.
    Edge,
    /// **A sphere** (`docs/spatial-constraints-plan.md`): a centre drawn in some view and a
    /// radius it owns, like a circle's — but no picture on any sheet, since a sphere seen in a
    /// view is a circle only square on.  Its relations (`p on s`, `radius`, `tangent`) are in
    /// space and read its centre's lift.  Last in the enum so every kind's id stays what it was.
    Sphere,
    /// **A cone**: an axis line drawn in some view — the apex its start, the axis running
    /// toward its end — and a half-angle it owns, like a sphere's radius.  No picture on any
    /// sheet; its relations (`p on k`, `angle`, `tangent`) are in space and read the axis's lifts.
    Cone,
    /// **A cylinder**: an axis line drawn in some view and a radius it owns.
    Cylinder,
}

impl EntKind {
    pub fn as_str(self) -> &'static str {
        match self {
            EntKind::Point => "point",
            EntKind::Line => "line",
            EntKind::Circle => "circle",
            EntKind::Arc => "arc",
            EntKind::Spline => "spline",
            EntKind::Plane => "plane",
            EntKind::Curve => "curve",
            EntKind::Face => "face",
            EntKind::Solid => "solid",
            EntKind::Surface => "surface",
            EntKind::Motion => "motion",
            EntKind::Envelope => "envelope",
            EntKind::Patch => "patch",
            EntKind::Seam => "seam",
            EntKind::Vertex => "vertex",
            EntKind::Edge => "edge",
            EntKind::Sphere => "sphere",
            EntKind::Cone => "cone",
            EntKind::Cylinder => "cylinder",
        }
    }

    pub fn parse(s: &str) -> Option<EntKind> {
        Some(match s {
            "point" => EntKind::Point,
            "line" => EntKind::Line,
            "circle" => EntKind::Circle,
            "arc" => EntKind::Arc,
            "spline" => EntKind::Spline,
            "plane" => EntKind::Plane,
            "curve" => EntKind::Curve,
            "face" => EntKind::Face,
            "solid" => EntKind::Solid,
            "surface" => EntKind::Surface,
            "motion" => EntKind::Motion,
            "envelope" => EntKind::Envelope,
            "patch" => EntKind::Patch,
            "seam" => EntKind::Seam,
            "vertex" => EntKind::Vertex,
            "edge" => EntKind::Edge,
            "sphere" => EntKind::Sphere,
            "cone" => EntKind::Cone,
            "cylinder" => EntKind::Cylinder,
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
            EntKind::Circle | EntKind::Sphere => &[("center", C), ("r", S)],
            // the axis is a line, the one child that is not a point: the apex is its start
            EntKind::Cone => &[("axis", C), ("half", S)],
            EntKind::Cylinder => &[("axis", C), ("r", S)],
            EntKind::Arc => &[("center", C), ("start", C), ("end", C), ("r", S)],
            EntKind::Spline => &[("ctrl", L)],
            // a plane's attitude is not a field: a Scalar is a number a solve may write back,
            // and the basis is document data no solve moves
            EntKind::Plane => {
                &[("origin", C), ("toward", C), ("c", S), ("s", S)]
            }
            // as many arguments as its definition takes, and none of them need be points — the
            // first kind for which that is true
            EntKind::Curve => &[("args", L)],
            // a loop of edges, as long as the loop is; the plane is read off their memberships
            EntKind::Face => &[("edges", L), ("holes", L), ("on", C)],
            // what it is swept from or made of: a face, or the solids of a term.  Every number
            // a solid carries is an *extent* — an expression, never a Scalar a solve writes back
            EntKind::Solid => &[("of", L)],
            EntKind::Surface => &[("solid", C), ("edge", C)],
            EntKind::Motion => &[("of", L)],
            EntKind::Envelope => &[("surface", C), ("motion", C)],
            EntKind::Patch => &[("source", C), ("inside", L), ("outside", L)],
            EntKind::Seam | EntKind::Vertex => &[("first", C), ("second", C)],
            EntKind::Edge => &[("seam", C), ("from", C), ("to", C), ("along", C)],
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
            | EntKind::Sphere
            | EntKind::Cone
            | EntKind::Cylinder
            | EntKind::Spline
            | EntKind::Plane
            | EntKind::Curve
            | EntKind::Face
            | EntKind::Solid | EntKind::Surface | EntKind::Motion | EntKind::Envelope | EntKind::Patch | EntKind::Seam | EntKind::Vertex | EntKind::Edge => None,
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
            EntKind::Circle | EntKind::Sphere => [pt("center"), vec![format!("{n}.r")]].concat(),
            EntKind::Arc => {
                [pt("center"), pt("start"), pt("end"), vec![format!("{n}.r")]].concat()
            }
            EntKind::Plane => {
                [pt("origin"), pt("toward"), vec![format!("{n}.c"), format!("{n}.s")]].concat()
            }
            // a surface in space is no formal a curve is written over
            EntKind::Cone | EntKind::Cylinder => return None,
            EntKind::Spline | EntKind::Curve | EntKind::Face | EntKind::Solid | EntKind::Surface | EntKind::Motion | EntKind::Envelope | EntKind::Patch | EntKind::Seam | EntKind::Vertex | EntKind::Edge => return None,
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
            // a face bears none of its own: its edges carry the memberships, and the face is on
            // the plane they agree about.  A solid is not on a plane at all.
            EntKind::Plane | EntKind::Curve | EntKind::Face | EntKind::Solid | EntKind::Surface | EntKind::Motion | EntKind::Envelope | EntKind::Patch | EntKind::Seam | EntKind::Vertex | EntKind::Edge => false,
            EntKind::Point
            | EntKind::Line
            | EntKind::Circle
            | EntKind::Arc
            | EntKind::Spline
            // its centre is a point of whatever view the declaration is in
            | EntKind::Sphere
            => true,
            // built over a line already drawn in its view, as a face is over its edges
            EntKind::Cone | EntKind::Cylinder => false,
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
            | EntKind::Curve
            | EntKind::Face
            | EntKind::Sphere
            | EntKind::Cone
            | EntKind::Cylinder
            | EntKind::Solid | EntKind::Surface | EntKind::Motion | EntKind::Envelope | EntKind::Patch | EntKind::Seam | EntKind::Vertex | EntKind::Edge => None,
        }
    }

    /// Whether an entity of this kind is **evaluated after the drawing is solved** rather than
    /// drawn on the sheet — a face, a solid, and what is derived from them (Solvent §6.9).
    ///
    /// The stratification asked as a question, so no consumer writes the list itself: nothing
    /// here owns a parameter, nothing here may be an argument of a 2D constraint, and nothing
    /// here is picked, dragged or dimensioned on the sheet.
    pub fn spatial(self) -> bool {
        match self {
            EntKind::Face | EntKind::Solid | EntKind::Surface | EntKind::Motion | EntKind::Envelope | EntKind::Patch | EntKind::Seam | EntKind::Vertex | EntKind::Edge => true,
            EntKind::Point
            | EntKind::Line
            | EntKind::Circle
            | EntKind::Arc
            | EntKind::Spline
            | EntKind::Plane
            | EntKind::Curve
            // solved, owning its radius — a figure of the drawing stratum, not a reading of one
            | EntKind::Sphere
            | EntKind::Cone
            | EntKind::Cylinder => false,
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
    pub fn plane(idx: usize) -> EntRef {
        EntRef::new(EntKind::Plane, idx)
    }
    pub fn face(idx: usize) -> EntRef {
        EntRef::new(EntKind::Face, idx)
    }
    pub fn solid(idx: usize) -> EntRef {
        EntRef::new(EntKind::Solid, idx)
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

/// A sphere: its centre, a drawn point, and its radius, a Param — a circle's fields with no
/// plane to be drawn in (`EntKind::Sphere`).
#[derive(Clone, Debug)]
pub struct SphereE {
    pub center: u32,
    pub radius: u32,
    pub class: Classes,
}

/// A cone or a cylinder: its axis, a drawn line — a cone's apex is the line's start — and
/// the number it owns, a Param: a cone's half-angle (radians) or a cylinder's radius.
#[derive(Clone, Debug)]
pub struct AxialE {
    pub axis: u32,
    pub param: u32,
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

/// An origin, a point it is pointed at, and the unit rotor `(c, s)` between them — the datum
/// half of a `PlaneE`, see `EntKind::Plane`.  `c` and `s` are Param indices; the two intrinsic
/// constraints that slave them to the chord are added by `Sketch::plane` and never serialized,
/// the arc's bargain.
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
    pub(in crate::model) basis: crate::plane::Basis,
    /// The attitude as **unknowns**, where the view is solved rather than stated (or read by one
    /// that is) — `None` for every other plane a document states, so no parameter is minted and
    /// nothing compiles differently (`Sketch::free_attitude`).
    pub att: Option<Att>,
}

/// A view's attitude and normal offset as solver unknowns (`docs/spatial-constraints-plan.md`).
///
/// `q` is a quaternion (four Params, held to the unit sphere by the intrinsic `quat_unit` row)
/// and `d` the offset along the view's normal (one Param, a length); the view's origin stands at
/// `o = R(q)·(a, b, d)`, where `ab` are **constants** carrying the in-plane part of the origin
/// the plane was minted at — in-plane translation is already the datum's own 2D freedom, so it
/// is not a second unknown here.  Kept out of `entity_params`, `fields` and `scalar_names`: a
/// traced tape's width and a report's table are the same whether or not a view is solved.
#[derive(Clone, Debug)]
pub struct Att {
    pub q: [u32; 4],
    pub d: u32,
    pub ab: [f64; 2],
    /// The values of `(q, d)` the stored basis is exact at.  A quaternion read back from a basis
    /// does not rebuild it to the bit — `cos 45°` squared is not a half — so `Sketch::basis`
    /// answers with the stored basis while the unknowns still hold exactly these numbers, and
    /// freeing a view moves nothing any reader sees until a solve moves the view.
    pub(in crate::model) seat: [f64; 5],
    /// Held by a hinge to the view it is folded from (`CKind::Hinge`) rather than by a
    /// `quat_unit` row of its own: a product of unit quaternions is one, and a second row saying
    /// so would be a redundant equation at every solution.
    pub hinged: bool,
}

/// A **hidden point in space**: the lift of one view point, held to it by an intrinsic `lift`
/// row — three Params and three rows, so it adds no freedom.  What a spatial relation between
/// two views reads, so its kernel sees three coordinates and never a view's attitude.
/// Not an entity: nothing names it, draws it, picks it or saves it; it is minted on request
/// (`Sketch::lift_point`), once per view point, and re-minted rather than stored.
#[derive(Clone, Debug)]
pub struct LiftE {
    pub point: u32,
    pub x: [u32; 3],
}
