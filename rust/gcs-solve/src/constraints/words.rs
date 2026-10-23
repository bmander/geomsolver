//! The words a statement is written with: operators, selectors and directions, read by
//! the parser and settled to a `CKind` by the kinds of their operands.

use super::*;

/// **The words a direction may be written as** (`docs/ordinate-plan.md`): `along:` an ordinate
/// is measured on, and what `level` keeps level.  A direction is otherwise a reference — an axis
/// or a drawn line — and these name the ones a drawing has without naming them: **the plane at
/// hand's own `u` and `v`** — the plane operand's, against one, else the view both points are
/// drawn in (a view is a plane, §6.7) — its normal `n` (against a plane only), and the view's
/// two with the sign said in the word, `right`, `left`, `up`, `down`.  One vocabulary for a
/// view's directions (#268): `x` and `y`, which named the view's `u` and `v` by a second set of
/// letters and read as the world's when they were not (in `std.front`, `y` was world z), are
/// retired, and `RETIRED` says so where one is written.  One table, read by the parser (a word
/// here is a word, never a name), by `infix_op`, by the elaborator's refusal and by the
/// inference of the direction itself (`infer_entity`); the signs are `side_words`'.
pub const ALONG: [(&str, Toward); 7] = [
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

/// The direction words `ALONG` no longer has, and the one each is now written as (#268).  A name
/// is free to be a name, so `along: x` reads `x` as a reference where the scope has one; where it
/// names nothing the refusal says what the word became, at the key.
pub const RETIRED: [(&str, &str); 2] = [("x", "u"), ("y", "v")];

/// What a retired direction word is written as now, if `word` is one (`RETIRED`).
pub fn retired_word(word: &str) -> Option<&'static str> {
    RETIRED.iter().find(|(w, _)| *w == word).map(|(_, now)| *now)
}

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
    /// `right`, `left`: the view's `u`, the sign said in the word.
    PageU,
    /// `up`, `down`: the view's `v`.
    PageV,
    /// `u`: the plane operand's own `u` against a plane, else the `u` of the view both points
    /// are drawn in (`in_view`) — one direction, whichever plane is at hand.
    PlaneU,
    /// `v`: likewise.
    PlaneV,
    /// `n`: a plane's normal, against the plane only.
    PlaneN,
}

impl Toward {
    /// The word's meaning, or `None` for a word that is not a direction.
    pub fn of(word: &str) -> Option<Toward> {
        ALONG.iter().find(|(w, _)| *w == word).map(|(_, t)| *t)
    }

    /// Whether the word may name a plane's own direction, so the second operand may be that
    /// plane: `u` and `v` (which with no plane operand are the view's, `in_view`) and `n`.
    pub fn of_plane(self) -> bool {
        matches!(self, Toward::PlaneU | Toward::PlaneV | Toward::PlaneN)
    }

    /// The axis of the view both points are drawn in that the word names there, with no plane
    /// operand (`PageU` or `PageV`): `u`, `right` and `left` its `u`, `v`, `up` and `down` its
    /// `v`.  `None` for `n`, which is a plane's normal and never a view's direction.
    pub fn in_view(self) -> Option<Toward> {
        match self {
            Toward::PageU | Toward::PlaneU => Some(Toward::PageU),
            Toward::PageV | Toward::PlaneV => Some(Toward::PageV),
            Toward::PlaneN => None,
        }
    }
}

/// **Whether an ordinate's word is read against a plane** — `q distance(d, along: u) P`, which
/// `program::relations::ordinate_operands` rewrote from `P.origin` along `P.u` — rather than in
/// the view both of its points are drawn in.  `n` always is; `u` and `v` are where the first
/// point is a plane's origin and the direction that plane's own axis, which the page reading
/// written from a view's own origin also is, and there the two readings are one ordinate.
pub fn against_plane(sk: &Sketch, word: &str, p: EntRef, t: &Arg) -> bool {
    let Some(toward) = Toward::of(word).filter(|t| t.of_plane()) else { return false };
    let Some(pl) = sk.plane_of_origin(p.i()) else { return toward == Toward::PlaneN };
    match (toward, t) {
        (Toward::PlaneU, Arg::Ent(t)) => *t == EntRef::axis(sk.frames[pl].u as usize),
        (Toward::PlaneV, Arg::Ent(t)) => *t == EntRef::axis(sk.frames[pl].v as usize),
        (Toward::PlaneN, _) => true,
        _ => false,
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
        let (du, dv) = (d(sk.frames[pl].u), d(sk.frames[pl].v));
        let held = du.iter().chain(&dv).all(|&k| sk.params[k as usize].fixed);
        let value = |p: [u32; 3]| p.map(|k| sk.params[k as usize].value);
        held && crate::space::dot(value(du), value(dv)).abs() <= 1e-12
    };
    let up = |pl: usize| matches!(toward, Some(Toward::PageV | Toward::PlaneV)) || square(pl);
    let axis = |pl: usize, u: bool| {
        let a = if u { sk.frames[pl].u } else { sk.frames[pl].v };
        along == EntRef::axis(a as usize)
    };
    if along.kind == EntKind::Frame {
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

/// The word an ordinate's direction was named by — `u`, `up`, `n` … — or `""` where it was named
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
/// seventeen kinds, `distance` fifteen, `tangent` thirteen.
///
/// Operand order carries meaning, and that is a change worth seeing: `arc tangent line` is
/// `TangentArcLine` and `line tangent circle` is `TangentLineCircle`.  Each named itself before
/// and the order was decoration; as an operator, which side the arc is written on picks the kind.
///
/// `sel` is what stood in the parentheses, by name — `along`, `at` — since two of the choices
/// cannot be made from the kinds alone.  `None` is "this word does not relate those two", which
/// the caller reports with the kinds in it.
pub fn infix_op(word: &str, a: EntKind, b: EntKind, sel: &dyn Fn(&str) -> Option<String>) -> Option<CKind> {
    use EntKind::{Arc, Circle, Curve, Envelope, Line, Meshing, Frame as Plane, Point, Axis, Spline, Traced};
    let round = |k: EntKind| matches!(k, Circle | Arc);
    // an axis is in space, so a direction relation naming one is the relation in space
    let axes = |a: EntKind, b: EntKind| matches!((a, b), (Axis, Axis | Line) | (Line, Axis));
    let at_point = sel("at").is_some_and(|w| w.is_empty());
    // a turn read about an axis named (#558): two directions, drawn or in space
    let about = |a: EntKind, b: EntKind| {
        sel("about").is_some() && (matches!((a, b), (Line, Line)) || axes(a, b))
    };
    Some(match word {
        // incidence, and two points being one: what it means is the kinds of its operands
        "coincident" => match (a, b) {
            // a point as a moving frame sees it, carried by the motion `under:` names (#326): to
            // the roll stated, `at: τ` (#366), or to one the relation owns, `t == τ`
            (Point, Point) if sel("under").is_some() => CKind::CarriedPoint,
            (Point, Point) => CKind::Coincident,
            // at a fraction of the way along where the place is written (`t == 0.25`, or a
            // seed for it), and only there: no `t`, no column (#248)
            (Point, Line) if sel("t").is_some() => CKind::PointAlongLine,
            (Point, Arc) if sel("t").is_some() => CKind::PointAlongArc,
            (Point, Line) => CKind::PointOnLine,
            (Point, k) if round(k) => CKind::PointOnCircle,
            (Point, Spline) => CKind::PointOnSpline,
            (Point, Curve) => CKind::PointOnCurve,
            // an envelope in space, whatever view the point is drawn in (#283)
            (Point, Envelope) => CKind::PointOnEnvelope,
            // the path of contact of two flanks, in the fixed frame (#283)
            (Point, Meshing) => CKind::PointOnMeshing,
            // a body traced in space (#326)
            (Point, Traced) => CKind::PointOnTraced,
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
            // **the tangent at a point a relation already holds on the curve** (#251): the line
            // runs along the curve there, one row — `at:` naming a point, which `Written::sel`
            // reads as no word.  Of a line, its own direction
            (Line, Line) if at_point => CKind::Parallel,
            (k, Line) if round(k) && at_point => CKind::TangentCircleAt,
            // of a spline, a curve, a meshing and a traced curve, the contact's rate there held
            // along the line (#365): `rate(m over t) parallel l`, written as the tangent it is
            (Spline | Curve, Line) if at_point => CKind::Rate,
            // an envelope in space at a point a contact holds on it (#283): a line or an axis in
            // its tangent plane, a plane touching it there
            (Envelope, Line | Axis) if at_point => CKind::Rate,
            (Envelope, Plane) if at_point => CKind::Rate,
            // the path of contact of a meshing at a point a contact holds on it (#345)
            (Meshing, Line | Axis) if at_point => CKind::Rate,
            // a body traced in space (#326): a curve's tangent, read as a surface's tangent plane
            // where the body is one (`program::relations`)
            (Traced, Line | Axis) if at_point => CKind::Rate,
            (Traced, Plane) if at_point => CKind::Rate,
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
            // a circle osculating an envelope's section by its plane, at a point already on it
            (Envelope, k) if round(k) && at_point => CKind::Rate,
            (Traced, k) if round(k) && at_point => CKind::Rate,
            // a curve's centre of curvature there, a point (#363 S5)
            (Traced, Point) if at_point => CKind::Rate,
            _ => return None,
        },
        // a free curve pressed onto a held line, a corner where it chooses (#149)
        "touches" => match (a, b) {
            (Curve, Line) => CKind::CurveTouchesLine,
            _ => return None,
        },
        // the ordinate's zero; `horizontal` and `vertical` between points are the standard
        // library's words for `level(up)` and `level(right)` (`std.sv`, §9.9)
        "level" => match (a, b) {
            (Point, Point | Plane) => CKind::Level,
            _ => return None,
        },
        "angle" => match (a, b) {
            (x, y) if about(x, y) => CKind::AngleAbout,
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
            (x, y) if about(x, y) => CKind::PerpendicularAbout,
            (Line, Line) => CKind::Perpendicular,
            (x, y) if axes(x, y) => CKind::Perpendicular3,
            // square to a plane: along its normal
            (Axis, Plane) => CKind::AxisPerpendicularPlane,
            (Line, Plane) => CKind::LinePerpendicularPlane,
            // along an envelope's normal at a point already on it (#283): the common normal
            (Envelope, Line | Axis) if at_point => CKind::Rate,
            // and a meshing's, the common normal of its two flanks there (#345)
            (Meshing, Line | Axis) if at_point => CKind::Rate,
            // and a traced surface's (#326)
            (Traced, Line | Axis) if at_point => CKind::Rate,
            _ => return None,
        },
        "symmetry" => match (a, b) {
            (Point, Point) => CKind::Symmetric,
            _ => return None,
        },
        // a point carried by a map (#357): three pairs an affine map, two the similarity — the
        // pairs counted where the statement is settled (`pairs`)
        "mapped" => match (a, b) {
            (Point, Point) if sel("pairs").as_deref() == Some("2") => CKind::MappedSimilar,
            (Point, Point) => CKind::Mapped,
            // a circle is carried by a similarity only: under three pairs it is an ellipse,
            // refused where the pairs are counted (`program::relations::settle`)
            (Circle, Circle) => CKind::MappedCircle,
            _ => return None,
        },
        // a generated flank regular within its blank, cut by a tool (#345)
        "regular" => match (a, b) {
            (Envelope, EntKind::Solid) => CKind::RegularWithin,
            _ => return None,
        },
        _ => return None,
    })
}

/// **What `word` means between two things selected** — of kinds `a` and `b`, in no order: a
/// selection has none, so `infix_op` over them as given, else turned round (no word relates a
/// pair both ways to two different kinds).  What a constraint button reading the selection
/// states; `None` where the word relates them neither way.
pub fn word_between(word: &str, a: EntKind, b: EntKind) -> Option<CKind> {
    let plain = |_: &str| None;
    infix_op(word, a, b, &plain).or_else(|| infix_op(word, b, a, &plain))
}

/// How near parallel two lines may look and still be dimensioned by the gap between them: the
/// sine of the angle between them.  A solved `parallel` sits within a residual scaled to the
/// sketch's extent, which on a short line is a few ten-thousandths of a radian; anything looser
/// is a corner.
pub const DIMENSION_PARALLEL_SIN: f64 = 1e-3;

/// What a number put on two lines measures: the gap between them (`ParallelDistance`) where the
/// sketch makes them parallel, else the angle at their corner (`Angle`) — read off the angle the
/// constraint would be given (`model::angle_between`).
pub fn dimension_between_lines(sk: &Sketch, a: EntRef, b: EntRef) -> CKind {
    use crate::fmath::Det;
    let angle = crate::model::angle_between(sk, a, b);
    if angle.dsin().abs() <= DIMENSION_PARALLEL_SIN { CKind::ParallelDistance } else { CKind::Angle }
}

/// One side of a tangency at a point the drawing already holds (#5): what it is — `None` for a
/// name whose kind elaboration has yet to say — and which of its ends the point is, as
/// `EntKind::ends` indexes them; `None` where the point lies on it but is none of its ends.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Touch {
    pub kind: Option<EntKind>,
    pub end: Option<usize>,
}

/// A tangency stated at its contact: `word` between the two sides — in the order they were
/// asked, or `swap`ped — with `at:` naming an end **of the operand written first**.  The two
/// At-forms name the contact from opposite sides (`arc tangent line at: end` the arc's end,
/// `line tangent circle at: p2` the line's), and that rule is what lets one entry say both.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TangentAt {
    pub word: &'static str,
    pub swap: bool,
    pub at: Option<&'static str>,
}

impl TangentAt {
    /// The constraint it settles to between sides of kinds `a` and `b`, by `infix_op` over the
    /// operands as written — so the entry says a word and a selector, never a second name.
    pub fn kind(self, a: EntKind, b: EntKind) -> Option<CKind> {
        let (x, y) = if self.swap { (b, a) } else { (a, b) };
        let at = self.at;
        infix_op(self.word, x, y, &|k| if k == "at" { at.map(str::to_string) } else { None })
    }
}

/// **Which regular tangency two things touching at a held point mean** (#5).  A tangency whose
/// contact the drawing already holds must be stated *at* that point, or it is a double root —
/// rank-deficient at every solution — and this is the one table saying how.  A chain knows the
/// contact by threading (one link's exit, the next one's entry; `syntax`'s `joint_relation`)
/// and the app by which end it finds already there (`commands.ts::cTangent`, through the
/// registry's `tangencyAt`), so it takes both sides and either may name the point.
///
/// A side names the contact by its own end as it runs there (`EntKind::ends`, matched over
/// every `Run`, so a kind given ends says how it runs and a new way stops the build here): a
/// round end against a straight side first (`TangentArcLine`), then a straight end against a
/// round side (`TangentLineCircleAt`, a circle or an arc's rim); two straight runs meeting are
/// collinear (`parallel`).  Two round things already touch where they meet, so there is no
/// regular form left: `None`, as for every pair it does not name — the caller says why in its
/// own terms.
pub fn tangency_at(a: Touch, b: Touch) -> Option<TangentAt> {
    use crate::model::Run;
    let run = |t: Touch| t.kind.and_then(EntKind::ends).map(|e| e.run);
    // what a side not known yet may turn out to be is elaboration's to refuse
    let straight = |t: Touch| t.kind.is_none() || run(t) == Some(Run::Straight);
    let round = |t: Touch| {
        matches!(t.kind, None | Some(EntKind::Circle)) || run(t) == Some(Run::Round)
    };
    if run(a) == Some(Run::Straight) && run(b) == Some(Run::Straight) {
        return (a.end.is_some() || b.end.is_some())
            .then_some(TangentAt { word: "parallel", swap: false, at: None });
    }
    // `side`'s own end names the contact, ranked: a round end before a straight one
    let named = |side: Touch, other: Touch, swap: bool| {
        let k = side.kind?;
        let ends = k.ends()?;
        let at = k.end_name(side.end?)?;
        let (rank, fits) = match ends.run {
            Run::Round => (0, straight(other)),
            Run::Straight => (1, round(other)),
        };
        fits.then_some((rank, TangentAt { word: "tangent", swap, at: Some(at) }))
    };
    let both = [named(a, b, false), named(b, a, true)];
    both.into_iter().flatten().min_by_key(|(rank, _)| *rank).map(|(_, t)| t)
}

/// **Whether the language itself has a word of this fixity** (§9.9): what a relation word may
/// not be defined as.  Read off the tables that settle a statement — `infix_op` and `prefix_op`
/// over every pair of kinds, the gauges, the calls and the words about solids — so a word the
/// core comes to know is reserved with nothing here to edit.  `horizontal` is a prefix word of
/// the core's (a line's direction) and an infix word of the standard library's (two points'
/// level), which is why the question is asked of a fixity.  Every such word is an operator, so
/// the tables are read once, over `OPERATORS`.
/// The words that say which side of a plane a point is on (§9.6), and the bound each is on the
/// point's ordinate along the plane's normal: `inside` at most zero, behind the normal, and
/// `outside` at least zero.  `inside` between solids is the claim `solid_word` reads instead.
pub fn plane_side_word(w: &str) -> Option<crate::vocab::Cmp> {
    match w {
        "inside" => Some(crate::vocab::Cmp::Le),
        "outside" => Some(crate::vocab::Cmp::Ge),
        _ => None,
    }
}

pub fn builtin_word(word: &str, fixity: Fixity) -> bool {
    static BUILTIN: std::sync::OnceLock<Vec<(&'static str, Fixity)>> = std::sync::OnceLock::new();
    let table = BUILTIN.get_or_init(|| {
        let has = |w: &str, f: Fixity| match f {
            Fixity::Call => call_word(w),
            Fixity::Prefix => {
                matches!(gauge_op(w).map(CKind::operator), Some(Some((_, Fixity::Prefix))))
                    || EntKind::ALL
                        .iter()
                        .any(|&k| prefix_op(w, k).is_some() || retired_prefix(w, k).is_some())
            }
            Fixity::Infix => {
                solid_word(w).is_some()
                    || plane_side_word(w).is_some()
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
    table.contains(&(word, fixity)) || (fixity == Fixity::Prefix && step_word(word))
}

/// **A word the flattener expands where it stands** (`flatten::steps`, #330), rather than one a
/// `CKind` or a file's definition settles: `step`, which says where a line's end stands — from
/// its start (`du`, `dv`) or from a frame's origin (`u`, `v`) — and keeps the ordinate it does
/// not name.  It stands before a line and takes its parameters by label, as a defined word does,
/// and it is no operator: a formal may still be called `step`.
pub fn step_word(w: &str) -> bool {
    w == "step"
}

/// A step's labels along each axis of its frame, in the order it states its relations (`u`, then
/// `v`): the ordinate from the frame's origin, and the one from the line's start.
pub const STEP_AXES: [(&str, &str); 2] = [("u", "du"), ("v", "dv")];

/// The same for a word standing *before* its one operand.  **`length` measures one thing**, of
/// every kind of path (#269): a line's is sugar for the distance between its ends, which is why
/// it is here and not in the table above, and so a word over `g: path` states `length(L) g`
/// whatever `g` is given.
pub fn prefix_op(word: &str, on: EntKind) -> Option<CKind> {
    use EntKind::{Arc, Circle, Line, Spline};
    Some(match (word, on) {
        ("horizontal", Line) => CKind::Horizontal,
        ("vertical", Line) => CKind::Vertical,
        ("radius", Circle | Arc) => CKind::Radius,
        ("length", Line) => CKind::Distance,
        // an arc's length along itself: the other way a round thing is dimensioned
        ("length", Arc) => CKind::ArcLength,
        // and a spline's, along the whole of it, and a free curve's, its own
        ("length", Spline) => CKind::SplineLength,
        ("length", EntKind::Curve) => CKind::CurveLength,
        _ => return None,
    })
}

/// **A prefix spelling the language no longer has**, and the word it became.  `distance relates
/// two things; `length` measures one` (#269), so `distance(d) l` on a line is `length(d) l`.
/// Still a prefix word to the grammar, so a statement or a chain link written the old way
/// parses and is refused where it is settled, naming the new spelling, rather than failing as
/// something else; and still a builtin word, so no definition takes it.
pub fn retired_prefix(word: &str, on: EntKind) -> Option<&'static str> {
    match (word, on) {
        ("distance", EntKind::Line) => Some("length"),
        _ => None,
    }
}

/// Every word the language writes a **constraint** with (spec §9.1) — the gauges and the
/// orientation predicates among them, read by the one relation parser and settled by the one
/// table (`gauge_op`), so a class, a placement and the chain's lookahead treat them as any
/// other word.  None of the three is a prefix word a chain can open a link with: `prefix_op`
/// declines them, so `fix(x == 0) point p -> …` is no chain.
pub const OPERATORS: [&str; 26] = [
    "distance", "tangent", "equal", "curvature", "horizontal", "vertical", "level", "angle",
    "radius", "length", "coincident", "midpoint", "parallel", "perpendicular", "symmetry", "project",
    "touches", "fix", "ccw", "cw",
    // a point carried by a map (#357)
    "mapped",
    // a flank regular within its blank (#345), written `regular(…) within`
    "regular",
    // **the words that relate two solids** (§9.8).  They are operators so that a statement
    // reads the way every other statement does; they settle to no `CKind` and compile no row,
    // because a solid is evaluated after the drawing is solved and a claim about one is judged
    // rather than enforced — `solid_claim` is where the word is read.
    "clear", "inside", "fits",
    // and `outside`, which with `inside` also says which side of a plane a point is on: a bound
    // (§9.6), lowered to the ordinate along the plane's normal (`program::relations`)
    "outside",
];

/// The word an operator is followed by before its second operand, where it takes one: `flank
/// regular(…) within blank` (#345).  Read by the parser after the operator's parentheses, and
/// written back by the printers.
pub fn connective(word: &str) -> Option<&'static str> {
    (word == "regular").then_some("within")
}

/// Whether a relation written with `word` reads a region solid's terms, so is stated once the
/// solids are built (#345).
pub fn reads_regions(word: &str) -> bool {
    word == "regular"
}

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

/// **A joint's value** (#391 S3), the slot `fix(value == τ) f` holds beside its ratio and
/// advance: the instant of the roll the frame stands at, which what is drawn in it is read at
/// (`n := x in f`).  No number of the frame's own and no row: it is read off the statement
/// (`program::motions::values`), and the `fix` holds the rest.
pub const JOINT_VALUE: &str = "value";

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
