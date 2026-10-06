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
    /// **The one datum kind** (issue #47, item 6: `frame` is folded into it): a plane in space
    /// over two axes, `plane(u: r1, v: r2)`, standing at a place of its own (`PlaneE`).  Its
    /// attitude is its axes' directions and where it stands three Params — unknowns of the solve
    /// unless a `fix` holds them, when the plane is **fixed** (`Sketch::plane_fixed`).
    /// A point that says it is `in` the plane is drawn in the plane's own 2D coordinates and
    /// stands in space at its lift (`Sketch::world_point`); `Project` between two such points is
    /// the one equation two images of one point share.
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
    /// plane, or a term over other solids — its stock, plus everything in `union` with it, minus
    /// everything that `cut`s it.
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
    /// **An axis** (`docs/planes-plan.md`): a directed line in space, with no start — a direction
    /// and a place, placed by relations like any other entity and drawn on no sheet.  Its
    /// relations (`parallel`, `perpendicular`, `angle`, `p coincident t`) are in space.
    Axis,
}

impl EntKind {
    /// The kind's name with its article, as a sentence says it: `a line`, `an axis`.
    pub fn a(self) -> String {
        article(self.as_str())
    }

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
            EntKind::Axis => "axis",
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
            "axis" => EntKind::Axis,
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
            // a point in space has a third, which a point drawn in a plane never owns
            EntKind::Point => &[("x", S), ("y", S), ("z", S)],
            EntKind::Line => &[("p1", C), ("p2", C)],
            EntKind::Circle => &[("center", C), ("r", S)],
            // its direction, which a seed and a `fix` name, and then the point on it nearest the
            // origin, which only a relation that reads where the axis is moves
            EntKind::Axis => &[("x", S), ("y", S), ("z", S), ("px", S), ("py", S), ("pz", S)],
            EntKind::Arc => &[("center", C), ("start", C), ("end", C), ("r", S)],
            EntKind::Spline => &[("ctrl", L)],
            // two axes for its attitude, its origin a point drawn in it, and where that stands
            EntKind::Plane => &[("u", C), ("v", C), ("origin", C), ("x", S), ("y", S), ("z", S)],
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
            | EntKind::Axis
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
    /// What the source calls each number the kind owns, in the order `fields` lists its scalars:
    /// the key a `hint(…)` seeds and a `fix(…)` pins.  A point *is* its place, so its numbers are
    /// its own `x`, `y` and `z`; an axis has a direction and an origin, a plane an origin, each a
    /// vector whose components are `dir.x`, `origin.y`, … (`vectors`).
    pub fn members(self) -> &'static [&'static str] {
        match self {
            EntKind::Point => &["x", "y", "z"],
            EntKind::Circle | EntKind::Arc => &["r"],
            EntKind::Axis => &["dir.x", "dir.y", "dir.z", "origin.x", "origin.y", "origin.z"],
            EntKind::Plane => &["origin.x", "origin.y", "origin.z"],
            EntKind::Line | EntKind::Spline | EntKind::Curve | EntKind::Face | EntKind::Solid
            | EntKind::Surface | EntKind::Motion | EntKind::Envelope | EntKind::Patch
            | EntKind::Seam | EntKind::Vertex | EntKind::Edge => &[],
        }
    }

    /// The vectors among `members`, each by its key and its first member: `""` is the entity
    /// itself — `fix((0, 0)) p`, `hint((3, 4))` — and is a point's, as many components as the
    /// point owns (two in a plane, three in space); `dir` and `origin` have three.
    pub fn vectors(self) -> &'static [(&'static str, usize)] {
        match self {
            EntKind::Point => &[("", 0)],
            EntKind::Axis => &[("dir", 0), ("origin", 3)],
            EntKind::Plane => &[("origin", 0)],
            _ => &[],
        }
    }

    pub fn scalar_names(self, n: &str) -> Option<Vec<String>> {
        let pt = |f: &str| vec![format!("{n}.{f}.x"), format!("{n}.{f}.y")];
        Some(match self {
            EntKind::Point => vec![format!("{n}.x"), format!("{n}.y")],
            EntKind::Line => [pt("p1"), pt("p2")].concat(),
            EntKind::Circle => [pt("center"), vec![format!("{n}.r")]].concat(),
            EntKind::Arc => {
                [pt("center"), pt("start"), pt("end"), vec![format!("{n}.r")]].concat()
            }
            // where it stands; its attitude is its axes'
            EntKind::Plane => ["x", "y", "z"].iter().map(|f| format!("{n}.{f}")).collect(),
            EntKind::Axis => ["x", "y", "z", "px", "py", "pz"].iter().map(|f| format!("{n}.{f}")).collect(),
            EntKind::Spline | EntKind::Curve | EntKind::Face | EntKind::Solid | EntKind::Surface | EntKind::Motion | EntKind::Envelope | EntKind::Patch | EntKind::Seam | EntKind::Vertex | EntKind::Edge => return None,
        })
    }

    /// Whether an entity of this kind has points of its own to put on a plane — what the `in`
    /// clause and the `in … { }` block ask before stamping one (§6.7).  A plane's origin is the
    /// plane's, and a curve is its expressions; everything else is drawn from points a
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
            | EntKind::Spline => true,
            // in space, in no view
            EntKind::Axis => false,
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
            | EntKind::Axis
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
            // solved, owning its number — a figure of the drawing stratum, not a reading of one
            | EntKind::Axis => false,
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
    /// The third coordinate of a **point in space** — one a document declares outside every
    /// `in` (`docs/planes-plan.md`), whose three numbers are where it stands.  `None` for a
    /// point drawn in a plane, whose two numbers are that plane's own coordinates.
    pub z: Option<u32>,
    /// The plane this point is drawn in (`a := point in top`): its `x`, `y` are that plane's
    /// coordinates.  A point with neither a plane nor a `z` is a point of a **2D sketch** — one a
    /// program or a JSON document builds with no planes at all — read in space on the front
    /// plane, and never made by a document.
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


/// An axis: a unit direction `d` and the point `a` on it nearest the world origin, six Params.
/// `|d| = 1` is an intrinsic row (`Sketch::axis`); `a·d = 0` is another, minted only once a
/// relation reads where the axis is (`placed`) — an axis read only as a direction leaves `a` fixed
/// and no freedom of it counted.
#[derive(Clone, Debug)]
pub struct AxisE {
    pub d: [u32; 3],
    pub a: [u32; 3],
    pub placed: bool,
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

/// **A plane in space** (`docs/planes-plan.md`): its attitude is two axes' — right along `u`,
/// out along `u × v`, up along `out × u` — and where it stands is `o`, three Params of its own.
/// Its origin is the member point `origin`, drawn in it and held at `(0, 0)`, so it stands at
/// `o`.  A plane owns no intrinsic row: what it is, is read off its axes and `o` (`Sketch::basis`),
/// and it is **fixed** — its basis constants every row may read — when all nine are held.
#[derive(Clone, Debug)]
pub struct PlaneE {
    pub u: u32,
    pub v: u32,
    pub o: [u32; 3],
    pub origin: u32,
    pub class: Classes,
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

/// A word with its indefinite article: `a line`, `an axis`, `an arc`.
pub fn article(word: &str) -> String {
    let an = word.starts_with(|c: char| "aeiou".contains(c));
    format!("{} {word}", if an { "an" } else { "a" })
}
