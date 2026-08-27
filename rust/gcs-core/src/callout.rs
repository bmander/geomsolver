//! Dimension callouts: what a dimensioned constraint looks like on the drawing.
//!
//! Every constraint carrying a Length or an Angle has a drafting figure — extension lines, a
//! dimension line between two arrowheads, a radial leader, an angular arc — and a number beside
//! it.  All of that is geometry: feet of perpendiculars, intersections of infinite lines, arc
//! sweeps, which side of a feature the dimension line stands off on and how far out it goes when
//! another one is already there.  So it is computed here, once, and a front end only paints the
//! segments it is handed.
//!
//! Where a figure sits is a *placement*: two numbers in a frame that follows the geometry (see
//! `Frame`).  The layout puts every dimension somewhere sensible and out of the way of the ones
//! already there, and anything the user has dragged is remembered on the sketch — so a placement
//! is either theirs or ours, never a mix, and it survives a save, an undo and a re-solve.
//!
//! Sizes are screen-constant.  The caller passes `unit`, the world length of one screen pixel,
//! and every stand-off, arrowhead and character is a fixed number of pixels through it — zooming
//! changes what a callout measures, never how big it looks.  The one thing measured in the
//! sketch's own units is the value itself.

use crate::constraints::{CKind, Constraint};
use crate::io::dimension_text;
use crate::model::{angle_between, seg_distance, signed_point_to_line, EntKind, EntRef, Sketch};
use std::collections::BTreeMap;
use std::f64::consts::{FRAC_PI_4, PI};

/// A world-coordinate point.
pub type P = (f64, f64);

/// Label height, in pixels: the front end draws the text at exactly this size, so the box this
/// module reserves for it and the glyphs that land there are the same size.
pub const FONT_PX: f64 = 12.0;
/// Arrowhead length, in pixels, and the half-width of its barbs as a fraction of that.  The
/// front end draws the head, so it is told the shape rather than left to invent one.
pub const ARROW_PX: f64 = 9.0;
pub const BARB: f64 = 0.3;

/// Average glyph advance as a fraction of the text height.  An estimate — the core cannot measure
/// a font — and it is only ever used to decide whether a number fits between two arrowheads and
/// to reserve the box behind it, both of which tolerate being a few percent out.
const CHAR_EM: f64 = 0.6;
const OFFSET_PX: f64 = 30.0; // a dimension line's stand-off from what it measures
const STACK_PX: f64 = 20.0; // and how much further out the next one in the same lane goes
const GAP_PX: f64 = 5.0; // an extension line's gap from the point it springs from
const OVER_PX: f64 = 6.0; // and its overshoot past the dimension line
const TEXT_GAP_PX: f64 = 3.0; // the label's clearance over the line it belongs to
const LEADER_PX: f64 = 14.0; // a radial leader's run past the rim
const LAND_PX: f64 = 18.0; // the shortest landing a label sits on
const ARC_MIN_PX: f64 = 22.0; // an angular dimension's arc radius, clamped to stay legible
const ARC_MAX_PX: f64 = 64.0;
const SPIN: f64 = 0.4; // radians the next leader about the same centre is rotated by
const LANE_PX: f64 = 10.0; // two dimension lines within this of each other share a lane

/// The kinds with nothing to state: a relation carries no number, so it has no callout.  Both
/// matches over `CKind` in this module are exhaustive on purpose — adding a constraint type
/// stops the build here, which is where its figure has to be decided.
macro_rules! undrawn {
    () => {
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
            | CKind::VerticalPoints
    };
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CalloutKind {
    /// Two arrowheads facing each other along a straight measurement.
    Linear,
    /// A leader out of a centre, an arrowhead at the rim, the label on a landing.
    Radial,
    /// An arc about a corner, swept by exactly the angle the constraint names.
    Angular,
}

#[derive(Clone, Copy, Debug)]
pub struct Seg(pub P, pub P);

#[derive(Clone, Copy, Debug)]
pub struct CArc {
    pub c: P,
    pub r: f64,
    /// Start and end angle; the sweep runs counterclockwise when `a1 > a0` and clockwise
    /// otherwise, which is the sign of the angle the constraint names.
    pub a0: f64,
    pub a1: f64,
}

/// `at` is the tip; `dir` is the direction the tip points, so the head occupies the arrowhead's
/// length back along `-dir`.
#[derive(Clone, Copy, Debug)]
pub struct Arrow {
    pub at: P,
    pub dir: P,
}

/// One dimension, drawn.  `solid` and `arcs` are the dimension line itself, `thin` the extension
/// and witness lines a drawing puts in dashes, `arrows` the heads, and `anchor`/`angle`/`label`
/// the number: the centre of its box, the direction it reads in, and the box's four corners for
/// the front end to clear the background behind it.
#[derive(Clone, Debug)]
pub struct Callout {
    pub id: u32,
    pub kind: CalloutKind,
    pub text: String,
    pub anchor: P,
    pub angle: f64,
    pub label: [P; 4],
    /// The placement this figure was drawn at, in its own `Frame` — whether it was put there by
    /// hand or laid out there.  It is what a drag starts from.
    pub place: (f64, f64),
    pub solid: Vec<Seg>,
    pub thin: Vec<Seg>,
    pub arcs: Vec<CArc>,
    pub arrows: Vec<Arrow>,
}

impl Callout {
    fn new(id: u32, kind: CalloutKind, text: &str) -> Callout {
        Callout {
            id,
            kind,
            text: text.to_string(),
            anchor: (0.0, 0.0),
            angle: 0.0,
            label: [(0.0, 0.0); 4],
            place: (0.0, 0.0),
            solid: Vec::new(),
            thin: Vec::new(),
            arcs: Vec::new(),
            arrows: Vec::new(),
        }
    }

    /// How near `p` comes to this callout, or `None` if it misses.  The drawn line, the arc as a
    /// polyline and the label — solid, since landing in the middle of the number is the most
    /// obvious way to mean that dimension.  A distance to a segment either way, so the curve is
    /// flattened here rather than in two front ends, and only when someone is actually picking:
    /// every frame paints, hardly any frame picks.
    fn near(&self, p: P) -> f64 {
        if inside(&self.label, p) {
            return 0.0; // the number is solid, not an outline
        }
        let edges: [Seg; 4] = std::array::from_fn(|i| Seg(self.label[i], self.label[(i + 1) % 4]));
        let mut best = f64::INFINITY;
        for s in self.solid.iter().chain(&edges) {
            best = best.min(seg_distance(p, s.0, s.1));
        }
        for a in &self.arcs {
            let n = 16;
            let mut prev = along(a.c, ray(a.a0), a.r);
            for i in 1..=n {
                let t = a.a0 + (a.a1 - a.a0) * (i as f64 / n as f64);
                let cur = along(a.c, ray(t), a.r);
                best = best.min(seg_distance(p, prev, cur));
                prev = cur;
            }
        }
        best
    }
}

/// The drafting figure for every dimensioned constraint in the sketch, in world coordinates.
/// `unit` is the world length of one screen pixel.
pub fn layout(sk: &Sketch, unit: f64) -> Vec<Callout> {
    let u = if unit.is_finite() && unit > 0.0 { unit } else { 1.0 };
    let (x0, y0, x1, y1) = sk.drawn_bounds();
    let mut pen = Pen {
        sk,
        u,
        hub: (0.5 * (x0 + x1), 0.5 * (y0 + y1)),
        lanes: BTreeMap::new(),
        spins: BTreeMap::new(),
    };
    let mut out = Vec::with_capacity(sk.constraints.len());
    for c in &sk.constraints {
        // a drag target is a number, not a dimension, and an arc's own definition is not
        // something the drawing states twice
        if c.soft || c.intrinsic {
            continue;
        }
        if let Some(k) = pen.one(c) {
            out.push(k);
        }
    }
    out
}

/* -- vectors ---------------------------------------------------------------- */

fn sub(a: P, b: P) -> P {
    (a.0 - b.0, a.1 - b.1)
}

fn mul(a: P, s: f64) -> P {
    (a.0 * s, a.1 * s)
}

/// `a` moved `s` along `d` — every construction below is one of these.
fn along(a: P, d: P, s: f64) -> P {
    (a.0 + d.0 * s, a.1 + d.1 * s)
}

fn dot(a: P, b: P) -> f64 {
    a.0 * b.0 + a.1 * b.1
}

fn cross(a: P, b: P) -> f64 {
    a.0 * b.1 - a.1 * b.0
}

fn len(a: P) -> f64 {
    a.0.hypot(a.1)
}

/// A quarter turn counterclockwise.
fn perp(a: P) -> P {
    (-a.1, a.0)
}

fn unit(a: P) -> Option<P> {
    let l = len(a);
    if l <= 1e-12 {
        None
    } else {
        Some((a.0 / l, a.1 / l))
    }
}

fn mid(a: P, b: P) -> P {
    (0.5 * (a.0 + b.0), 0.5 * (a.1 + b.1))
}

fn ray(th: f64) -> P {
    (th.cos(), th.sin())
}

/// Text is never upside down: a direction in the left half plane reads the other way round, and
/// since the anchor is the box's centre, turning it by half a turn moves nothing but the glyphs.
fn readable(th: f64) -> f64 {
    let mut t = th.rem_euclid(2.0 * PI);
    if t > PI {
        t -= 2.0 * PI;
    }
    if t > 0.5 * PI {
        t - PI
    } else if t <= -0.5 * PI {
        t + PI
    } else {
        t
    }
}

fn text_em(text: &str) -> f64 {
    text.chars().count() as f64 * CHAR_EM
}

/* -- where a callout hangs --------------------------------------------------- */

/// The frame a callout hangs off: the two numbers a placement is written in, and what they mean
/// on the drawing.  Both follow the geometry, so a dimension dragged into place stays there as
/// the sketch moves under it.
#[derive(Clone, Copy, Debug)]
pub enum Frame {
    /// A straight measurement: `t` runs along it from the middle, `r` across it.  Dragging one
    /// of these slides the number along the dimension and stands the line off to either side.
    Linear { o: P, d: P, n: P },
    /// A leader or an arc about a centre: `t` is the angle from `th0`, `r` the distance out.
    Polar { o: P, th0: f64 },
}

impl Frame {
    /// A world point as this frame's two numbers.
    pub fn of(&self, p: P) -> (f64, f64) {
        match *self {
            Frame::Linear { o, d, n } => (dot(sub(p, o), d), dot(sub(p, o), n)),
            Frame::Polar { o, th0 } => {
                let v = sub(p, o);
                (wrap(v.1.atan2(v.0) - th0), len(v))
            }
        }
    }

    /// A first coordinate brought back into range.  Only a polar frame's is an angle, and only
    /// an angle can pick up a spare lap; keeping the rule here is what stops `grab` and `drag`
    /// each having to remember which frame they are in.
    fn along(&self, t: f64) -> f64 {
        match self {
            Frame::Polar { .. } => wrap(t),
            Frame::Linear { .. } => t,
        }
    }
}

/// Half a turn either way — an angle difference, or a placement angle, brought back to where it
/// can be compared and added without a lap appearing in it.
fn wrap(th: f64) -> f64 {
    let t = th.rem_euclid(2.0 * PI);
    if t > PI {
        t - 2.0 * PI
    } else {
        t
    }
}

/// The frame a dimension's callout hangs off, or `None` for a constraint that has no callout or
/// whose geometry has gone degenerate.
pub fn frame(sk: &Sketch, c: &Constraint) -> Option<Frame> {
    match c.kind {
        CKind::Distance | CKind::PointLineDistance | CKind::ParallelDistance => {
            let (a, b) = ends(sk, c)?;
            let d = unit(sub(b, a))?;
            Some(Frame::Linear { o: mid(a, b), d, n: perp(d) })
        }
        // a run or a rise is written in the page's frame, not the pair's: the number does not
        // turn with the two points, which is the whole of what makes it a different dimension
        CKind::HorizontalDistance | CKind::VerticalDistance => {
            let (a, b) = ends(sk, c)?;
            let d = axis_of(c.kind);
            Some(Frame::Linear { o: mid(a, b), d, n: perp(d) })
        }
        CKind::Radius | CKind::AnnularDistance => {
            let e = c.args[0].ent();
            Some(Frame::Polar { o: sk.point_xy(sk.round_center(e)), th0: sweep_start(sk, e) })
        }
        CKind::Angle => {
            let (corner, d1, _) = corner_of(sk, c)?;
            Some(Frame::Polar { o: corner, th0: d1.1.atan2(d1.0) })
        }
        undrawn!() => None,
    }
}

/// The direction a run or a rise is measured along: the page's own axes.
fn axis_of(k: CKind) -> P {
    if k == CKind::VerticalDistance { (0.0, 1.0) } else { (1.0, 0.0) }
}

/// Which of the three dimensions between two points a callout dropped at `at` states.
///
/// A dimension line stands off *across* what it measures, so where the number is put is what
/// says which measurement was meant: the direction from the middle of the pair out to `at`
/// picks the nearest of the three lines a dimension line could lie along — the pair's own (a
/// length), the page's x (the run between them) or the page's y (the rise).  Nearest, not a
/// threshold: the borders are the bisectors, so every direction belongs to exactly one region
/// and the three meet where they should.  A tie goes to the length, which is the dimension the
/// pair already looked like it wanted.
///
/// This is how a dimension is chosen by putting it somewhere rather than by being asked, so it
/// has to agree with the figure that then gets drawn — which is why it is here, beside the
/// frames, and not in a front end.
pub fn pair_dimension(a: P, b: P, at: P) -> CKind {
    let (Some(d), Some(off)) = (unit(sub(b, a)), unit(sub(at, mid(a, b)))) else {
        return CKind::Distance;
    };
    // |dot| and not dot: a dimension line and its reverse are the same line
    let mut best = (dot(off, perp(d)).abs(), CKind::Distance);
    for (k, across) in [(CKind::HorizontalDistance, (0.0, 1.0)),
                        (CKind::VerticalDistance, (1.0, 0.0))] {
        let s = dot(off, across).abs();
        if s > best.0 + 1e-12 {
            best = (s, k);
        }
    }
    best.1
}

/// The two points a straight dimension's arrowheads land on.  The layout and the frame have to
/// agree about these, so both ask here.
fn ends(sk: &Sketch, c: &Constraint) -> Option<(P, P)> {
    match c.kind {
        CKind::Distance | CKind::HorizontalDistance | CKind::VerticalDistance => {
            Some((sk.point_xy(c.args[0].ent().i()), sk.point_xy(c.args[1].ent().i())))
        }
        CKind::PointLineDistance => {
            let p = sk.point_xy(c.args[0].ent().i());
            let i = c.args[1].ent().i();
            let d = unit(sk.line_dir(i))?; // no direction: nothing to be perpendicular to
            let a0 = sk.point_xy(sk.lines[i].p1 as usize);
            Some((along(a0, d, dot(sub(p, a0), d)), p))
        }
        CKind::ParallelDistance => {
            let (i1, i2) = (c.args[0].ent().i(), c.args[1].ent().i());
            let d = unit(sk.line_dir(i1))?;
            let (p1, q1) = seg(sk, i1);
            let (p2, q2) = seg(sk, i2);
            // cross where the two segments have most in common: the mean of their four ends
            let t = [p1, q1, p2, q2].iter().map(|&e| dot(sub(e, p1), d)).sum::<f64>() / 4.0;
            let a = along(p1, d, t);
            // `signed_point_to_line` measures along this same normal, so the far end lands on l2
            Some((a, along(a, perp(d), signed_point_to_line(sk, p2.0, p2.1, i1))))
        }
        _ => None, // a radial or angular figure is not drawn between two points
    }
}

fn seg(sk: &Sketch, i: usize) -> (P, P) {
    let l = &sk.lines[i];
    (sk.point_xy(l.p1 as usize), sk.point_xy(l.p2 as usize))
}

/// The angle a leader out of a round entity is measured from: an arc's own start, so a placement
/// on one stays where it was put as the arc turns, and plain world angle for a whole circle.
fn sweep_start(sk: &Sketch, e: EntRef) -> f64 {
    match e.kind {
        EntKind::Arc => sk.arc_angles(e.i()).0,
        _ => 0.0,
    }
}

/// Where two lines meet, and the directions the model gives them — `None` when they are parallel
/// and the corner is at infinity.
fn corner_of(sk: &Sketch, c: &Constraint) -> Option<(P, P, P)> {
    let (i1, i2) = (c.args[0].ent().i(), c.args[1].ent().i());
    let d1 = unit(sk.line_dir(i1))?;
    let d2 = unit(sk.line_dir(i2))?;
    let den = cross(d1, d2);
    if den.abs() <= 1e-9 {
        return None;
    }
    let a1 = sk.point_xy(sk.lines[i1].p1 as usize);
    let a2 = sk.point_xy(sk.lines[i2].p1 as usize);
    Some((along(a1, d1, cross(sub(a2, a1), d2) / den), d1, d2))
}

/* -- moving a callout about -------------------------------------------------- */

/// The placement dimension `id` is drawn at now, whether it was put there or laid out there.
pub fn placement(sk: &Sketch, unit: f64, id: u32) -> Option<(f64, f64)> {
    layout(sk, unit).iter().find(|k| k.id == id).map(|k| k.place)
}

/// What to hold on to for the length of a drag: the offset between where the callout is and
/// where the pointer took hold of it, so it moves with the pointer instead of jumping to it.
pub fn grab(sk: &Sketch, unit: f64, id: u32, at: P) -> Option<(f64, f64)> {
    let c = sk.constraint(id)?;
    let f = frame(sk, c)?;
    let (t, r) = f.of(at);
    let (pt, pr) = placement(sk, unit, id)?;
    Some((f.along(pt - t), pr - r))
}

/// Move dimension `id`'s callout so the point it was grabbed at follows the pointer to `at`.
pub fn drag(sk: &mut Sketch, id: u32, at: P, grip: (f64, f64)) -> bool {
    let Some(c) = sk.constraint(id) else { return false };
    let Some(f) = frame(sk, c) else { return false };
    let (t, r) = f.of(at);
    sk.placements.insert(id, (f.along(t + grip.0), r + grip.1));
    true
}

/// Put dimension `id`'s callout back wherever the layout would have put it; true if it had been
/// moved at all, so a caller can tell an edit from a no-op.
pub fn reset(sk: &mut Sketch, id: u32) -> bool {
    sk.placements.remove(&id).is_some()
}

/// The dimension whose callout `at` lands on, within `tol_px` screen pixels of it, or `None`.
/// Nearest wins, so two callouts crossing each other still pick apart.
///
/// A *point* within reach beats the figure's lines, the way it beats an edge in `model::pick`:
/// a radius runs its leader out of the centre it measures from, so without this the one point a
/// circle has could not be clicked once it was dimensioned.  It does not beat the number's own
/// box, which is painted solid over whatever is behind it — what is under the label is the
/// label, and picking something the drawing is covering up would be a lie about the drawing.
pub fn pick(sk: &Sketch, unit: f64, at: P, tol_px: f64) -> Option<u32> {
    let tol = tol_px * unit;
    let mut best: Option<(f64, u32, bool)> = None;
    for k in layout(sk, unit) {
        let d = k.near(at);
        if d <= tol && best.is_none_or(|(bd, _, _)| d < bd) {
            best = Some((d, k.id, inside(&k.label, at)));
        }
    }
    let (_, id, on_label) = best?;
    let (near, d) = sk.nearest_point(at.0, at.1);
    if !on_label && near.is_some() && d <= tol {
        return None;
    }
    Some(id)
}

/// Is `p` within the convex quad `q`?  Its corners run round in order, so every edge has the
/// inside on the same hand.
fn inside(q: &[P; 4], p: P) -> bool {
    let side = |i: usize| cross(sub(q[(i + 1) % 4], q[i]), sub(p, q[i]));
    (0..4).all(|i| side(i) >= 0.0) || (0..4).all(|i| side(i) <= 0.0)
}

/* -- the layout -------------------------------------------------------------- */

struct Pen<'a> {
    sk: &'a Sketch,
    /// World units per screen pixel.
    u: f64,
    /// The middle of the drawing: a dimension line that has not been placed by hand stands off
    /// on the side facing away from it.
    hub: P,
    /// What already stands on each (direction, offset, side) lane, as the stretch of it each
    /// figure occupies and how far out that one went.
    lanes: BTreeMap<(i64, i64, i64), Vec<(f64, f64, f64)>>,
    /// And how many leaders already come out of each centre.
    spins: BTreeMap<(i64, i64), u32>,
}

/// The number as it goes on the drawing.  A **claimed** dimension (§9.7) is drawn in
/// parentheses — the draughtsman's *reference dimension*, which says "this is what it measures,
/// and it is not what controls it".  That is a claim exactly: judged against the figure, never
/// imposed on it, and deleting it leaves the drawing the same.
///
/// It wraps here rather than in `io::dimension_text` because the parentheses go round the whole
/// label: a claimed radius reads `(R50)`, not `R(50)`.  And it wraps before the text is measured,
/// so the lane it is given and the box it is picked by are the size of what is actually drawn.
fn claimed(c: &Constraint, text: String) -> String {
    if c.claim {
        format!("({text})")
    } else {
        text
    }
}

impl Pen<'_> {
    fn px(&self, n: f64) -> f64 {
        n * self.u
    }

    fn center(&self, e: EntRef) -> P {
        self.sk.point_xy(self.sk.round_center(e))
    }

    /// Where this dimension has been dragged to, if anywhere.
    fn placed(&self, c: &Constraint) -> Option<(f64, f64)> {
        self.sk.placements.get(&c.id).copied()
    }

    fn one(&mut self, c: &Constraint) -> Option<Callout> {
        match c.kind {
            CKind::Distance => self.distance(c),
            CKind::HorizontalDistance | CKind::VerticalDistance => self.axis_distance(c),
            CKind::PointLineDistance => self.point_line(c),
            CKind::ParallelDistance => self.parallel(c),
            CKind::Radius => self.radius(c),
            CKind::AnnularDistance => self.annular(c),
            CKind::Angle => self.angle(c),
            undrawn!() => None,
        }
    }

    /// How far past its stand-off the dimension line over `a`–`b` has to go to clear the ones
    /// already there.  Two dimensions share a lane when they stand on the same line, on the same
    /// side of it — quantised in screen terms, so which ones those are does not change with the
    /// zoom — but they only move apart if they also overlap *along* that line.  A chord with a
    /// dimension on every bay is then a row rather than a staircase, while two dimensions over
    /// the same span do stack.  A figure occupies its own span or its number's width, whichever
    /// is wider, since a short span puts the number outside.
    ///
    /// This is a first guess, not an arrangement: it keeps a fresh dimension from landing on top
    /// of one that is already there, and dragging is what actually places them.
    fn lane(&mut self, a: P, b: P, d: P, n: P, text: &str) -> f64 {
        // fold direction and side into the key separately: a line and its reverse are one lane,
        // but its two sides are not
        let dc = if d.1 < 0.0 || (d.1 == 0.0 && d.0 < 0.0) { mul(d, -1.0) } else { d };
        let nc = perp(dc);
        let key = (
            (dc.1.atan2(dc.0) / (PI / 24.0)).round() as i64,
            (dot(nc, a) / self.px(LANE_PX)).round() as i64,
            if dot(n, nc) >= 0.0 { 1 } else { -1 },
        );
        let (ta, tb) = (dot(dc, a), dot(dc, b));
        let half = (0.5 * (tb - ta).abs()).max(0.5 * self.px(FONT_PX * text_em(text)));
        let (t0, t1) = (0.5 * (ta + tb) - half, 0.5 * (ta + tb) + half);
        let step = self.px(STACK_PX);
        let taken = self.lanes.entry(key).or_default();
        let mut off = 0.0;
        while taken.iter().any(|&(u0, u1, o)| (o - off).abs() < 0.5 * step && u0 < t1 && t0 < u1) {
            off += step;
        }
        taken.push((t0, t1, off));
        off
    }

    /// The angle to turn the next leader out of `c` by, so two dimensions on concentric circles
    /// do not come out on top of each other.
    fn spin(&mut self, c: P) -> f64 {
        let key = (
            (c.0 / self.px(LANE_PX)).round() as i64,
            (c.1 / self.px(LANE_PX)).round() as i64,
        );
        let taken = self.spins.entry(key).or_insert(0);
        let n0 = *taken;
        *taken += 1;
        n0 as f64 * SPIN
    }

    /// Where a leader out of a round entity points when nobody has said otherwise, as an angle
    /// from that entity's own start.  A whole circle points away from the middle of the drawing,
    /// which puts the label outside the shape rather than across it; an arc has only its own
    /// sweep to use, so the leader takes the middle of it.
    fn auto_ray(&mut self, e: EntRef) -> f64 {
        let c = self.center(e);
        let spin = self.spin(c);
        match e.kind {
            EntKind::Arc => {
                let (a0, a1) = self.sk.arc_angles(e.i());
                let room = 0.4 * (a1 - a0);
                0.5 * (a1 - a0) + spin.clamp(-room, room)
            }
            _ => {
                let away = unit(sub(c, self.hub)).map_or(FRAC_PI_4, |d| d.1.atan2(d.0));
                away + spin
            }
        }
    }

    /// A leader's direction, given the placement angle it ended up with.  On an arc it is held
    /// inside the drawn sweep: a radius whose arrowhead misses the arc says nothing.
    fn ray_dir(&self, e: EntRef, t: f64) -> P {
        let th0 = sweep_start(self.sk, e);
        match e.kind {
            EntKind::Arc => {
                let (a0, a1) = self.sk.arc_angles(e.i());
                let room = 0.02 * (a1 - a0);
                ray(th0 + t.clamp(room, (a1 - a0) - room))
            }
            _ => ray(th0 + t),
        }
    }

    /* -- the shared figures -------------------------------------------------- */

    /// "The distance from `a` to `b` is `text`", measured along `a`→`b` itself — the figure a
    /// length is drawn as.  Two points in the same place have no direction to measure along, so
    /// the number gets a leader instead.
    fn aligned(&self, id: u32, a: P, b: P, place: (f64, f64), text: &str) -> Callout {
        match unit(sub(b, a)) {
            Some(d) => self.linear(id, a, b, d, place, text),
            None => self.note(id, CalloutKind::Linear, a, text),
        }
    }

    /// The same figure, measured along a direction of its own: the dimension line lies along
    /// `d`, standing `off` to its left of the middle of the two points (negative puts it on the
    /// right), with the number `shift` along it.  The heads land where the two points *fall* on
    /// that line and each extension line runs out to its own point from wherever it is — which
    /// is the whole difference between a length, whose `d` is the pair's own direction and whose
    /// two extension lines are therefore the same length, and a run across the page.  Too short
    /// a span puts the heads outside and the number past the far end, which is what a drawing
    /// does rather than shrink either.
    fn linear(&self, id: u32, a: P, b: P, d: P, place: (f64, f64), text: &str) -> Callout {
        let (shift, off) = place;
        let n = perp(d);
        let s = if off < 0.0 { -1.0 } else { 1.0 };
        let lvl = dot(n, mid(a, b)) + off;                  // the dimension line's own level
        let (ra, rb) = (lvl - dot(n, a), lvl - dot(n, b));  // and how far each point is from it
        let (a1, b1) = (along(a, n, ra), along(b, n, rb));
        let (tw, th) = (self.px(FONT_PX * text_em(text)), self.px(FONT_PX));
        let head = self.px(ARROW_PX);
        let mut k = Callout::new(id, CalloutKind::Linear, text);
        k.solid.push(Seg(a1, b1));
        for (p, r) in [(a, ra), (b, rb)] {
            if r.abs() > 1e-12 {
                let sr = r.signum();
                let g = sr * self.px(GAP_PX).min(r.abs());
                k.thin.push(Seg(along(p, n, g), along(p, n, r + sr * self.px(OVER_PX))));
            }
        }
        // the heads face along the dimension line, which runs backwards when the second point
        // is behind the first across the page — a run of -30 is drawn, and reads, right to left
        let dd = unit(sub(b1, a1)).unwrap_or(d);
        k.angle = readable(dd.1.atan2(dd.0));
        let over = s * (self.px(TEXT_GAP_PX) + 0.5 * th);
        if len(sub(b1, a1)) >= tw + 2.2 * head {
            k.arrows.push(Arrow { at: a1, dir: mul(dd, -1.0) });
            k.arrows.push(Arrow { at: b1, dir: dd });
            k.anchor = along(along(mid(a1, b1), dd, shift), n, over);
        } else {
            // no room between the points: the heads close in from outside and the number sits
            // beyond the far end, on a line stretched out to carry it
            k.arrows.push(Arrow { at: a1, dir: dd });
            k.arrows.push(Arrow { at: b1, dir: mul(dd, -1.0) });
            let stub = 2.5 * head;
            k.solid.push(Seg(along(a1, dd, -stub), a1));
            k.solid.push(Seg(b1, along(b1, dd, stub + tw)));
            k.anchor = along(along(b1, dd, stub + 0.5 * tw + shift), n, over);
        }
        k.place = place;
        self.seal(&mut k);
        k
    }

    /// Nothing to measure — a zero-length distance, two circles already the same size, an angle
    /// on lines that have gone parallel.  The number still has to appear somewhere, so it gets a
    /// stub leader off the feature instead of vanishing.
    fn note(&self, id: u32, kind: CalloutKind, at: P, text: &str) -> Callout {
        let step = self.px(LEADER_PX);
        let mut k = Callout::new(id, kind, text);
        self.land(&mut k, at, (at.0 + step, at.1 + step), text);
        k
    }

    /// Close a callout: reserve the box the label needs.  Called once the anchor and the angle
    /// are final.
    fn seal(&self, k: &mut Callout) {
        let pad = self.px(2.0);
        let hw = 0.5 * self.px(FONT_PX * text_em(&k.text)) + pad;
        let hh = 0.5 * self.px(FONT_PX) + pad;
        let (c, s) = (k.angle.cos(), k.angle.sin());
        let corner = |sx: f64, sy: f64| {
            (
                k.anchor.0 + sx * hw * c - sy * hh * s,
                k.anchor.1 + sx * hw * s + sy * hh * c,
            )
        };
        k.label = [
            corner(-1.0, -1.0),
            corner(1.0, -1.0),
            corner(1.0, 1.0),
            corner(-1.0, 1.0),
        ];
    }

    /* -- one per dimensioned constraint kind --------------------------------- */

    /// Two points.  Left where it fell, the dimension line stands off to the side facing away
    /// from the middle of the drawing, clear of anything already on that lane.
    fn distance(&mut self, c: &Constraint) -> Option<Callout> {
        let (a, b) = ends(self.sk, c)?;
        let text = claimed(c, dimension_text(c)?);
        let Some(d) = unit(sub(b, a)) else {
            return Some(self.note(c.id, CalloutKind::Linear, a, &text));
        };
        let place = self.placed(c).unwrap_or_else(|| {
            let n = perp(d);
            let s = if dot(n, sub(mid(a, b), self.hub)) < 0.0 { -1.0 } else { 1.0 };
            (0.0, s * (self.px(OFFSET_PX) + self.lane(a, b, d, mul(n, s), &text)))
        });
        Some(self.aligned(c.id, a, b, place, &text))
    }

    /// A run or a rise between two points: the dimension line lies along the page's own axis,
    /// the heads where the two points fall on it, and an extension line out to each of them from
    /// wherever it happens to be.  Left where it fell it stands off past the further of the two,
    /// so the figure clears the pair however the pair is turned.
    fn axis_distance(&mut self, c: &Constraint) -> Option<Callout> {
        let (a, b) = ends(self.sk, c)?;
        let text = claimed(c, dimension_text(c)?);
        let d = axis_of(c.kind);
        let n = perp(d);
        let place = self.placed(c).unwrap_or_else(|| {
            let s = if dot(n, sub(mid(a, b), self.hub)) < 0.0 { -1.0 } else { 1.0 };
            let clear = 0.5 * (dot(n, b) - dot(n, a)).abs();
            (0.0, s * (clear + self.px(OFFSET_PX) + self.lane(a, b, d, mul(n, s), &text)))
        });
        Some(self.linear(c.id, a, b, d, place, &text))
    }

    /// A point and a line: measured along the perpendicular, right where it is measured, since
    /// standing that off to one side would say nothing about which line it came from.  The
    /// distance is to the infinite line, so when the foot falls off the end of the segment the
    /// part of the line that is not drawn is shown as a witness.
    fn point_line(&mut self, c: &Constraint) -> Option<Callout> {
        let (foot, p) = ends(self.sk, c)?;
        let i = c.args[1].ent().i();
        let d = unit(self.sk.line_dir(i))?;
        let (a0, a1) = seg(self.sk, i);
        let text = claimed(c, dimension_text(c)?);
        let place = self.placed(c).unwrap_or((0.0, 0.0));
        let mut k = self.aligned(c.id, foot, p, place, &text);
        witness(&mut k, a0, a1, a0, d, dot(sub(p, a0), d), foot);
        // and a witness through the point, parallel to the line, so the measurement reads as an
        // offset from that edge rather than as a stray segment
        let o = self.px(OVER_PX);
        k.thin.push(Seg(along(p, d, -o), along(p, d, o)));
        Some(k)
    }

    /// Two parallel lines: the gap, crossed where the two segments have most in common, with a
    /// witness out to it from either line the crossing has run off the end of.
    fn parallel(&mut self, c: &Constraint) -> Option<Callout> {
        let (a, b) = ends(self.sk, c)?;
        let (i1, i2) = (c.args[0].ent().i(), c.args[1].ent().i());
        let d = unit(self.sk.line_dir(i1))?;
        let (p1, q1) = seg(self.sk, i1);
        let (p2, q2) = seg(self.sk, i2);
        let text = claimed(c, dimension_text(c)?);
        let place = self.placed(c).unwrap_or((0.0, 0.0));
        let mut k = self.aligned(c.id, a, b, place, &text);
        let t = dot(sub(a, p1), d);
        witness(&mut k, p1, q1, p1, d, t, a);
        witness(&mut k, p2, q2, p1, d, t, b);
        Some(k)
    }

    /// A circle or an arc: a leader out of the centre, the head where it crosses the rim, and
    /// the number on a landing clear of the shape.
    fn radius(&mut self, c: &Constraint) -> Option<Callout> {
        let e = c.args[0].ent();
        let text = claimed(c, format!("R{}", dimension_text(c)?));
        let r = self.sk.radius_value(e).abs();
        let place = self
            .placed(c)
            .unwrap_or_else(|| (self.auto_ray(e), r + self.px(LEADER_PX)));
        let d = self.ray_dir(e, place.0);
        let ctr = self.center(e);
        let rim = along(ctr, d, r);
        let mut k = Callout::new(c.id, CalloutKind::Radial, &text);
        k.place = place;
        k.solid.push(Seg(ctr, rim));
        k.arrows.push(Arrow { at: rim, dir: d });
        // the elbow cannot come back inside the rim, or the head would have nothing to sit on
        let elbow = along(ctr, d, place.1.max(r + self.px(2.0)));
        self.land(&mut k, rim, elbow, &text);
        Some(k)
    }

    /// Two circles or arcs: the ring between them, measured out along one ray so the number sits
    /// on the gap it names.  Concentric or not, both rims are taken in the same direction — the
    /// constraint is on the two radii, and that is the pair of points where their difference is
    /// what the eye sees.
    fn annular(&mut self, c: &Constraint) -> Option<Callout> {
        let (e1, e2) = (c.args[0].ent(), c.args[1].ent());
        let (c1, c2) = (self.center(e1), self.center(e2));
        let (r1, r2) = (self.sk.radius_value(e1).abs(), self.sk.radius_value(e2).abs());
        let text = claimed(c, dimension_text(c)?);
        let auto = (self.auto_ray(e1), 0.5 * (r1 + r2));
        let place = self.placed(c).unwrap_or(auto);
        let d = self.ray_dir(e1, place.0);
        let (a, b) = (along(c1, d, r1), along(c2, d, r2));
        // the ray fixes where on the ring it is measured; what is left of the placement slides
        // the number along that ray, out of the middle of the gap
        let shift = place.1 - len(sub(mid(a, b), c1));
        let mut k = self.aligned(c.id, a, b, (shift, 0.0), &text);
        k.place = place;   // the ring is drawn as a straight gap but placed round its centre
        k.thin.push(Seg(c1, a)); // the radius the ring is measured out from
        Some(k)
    }

    /// Two lines: an arc about the corner they make, swept by exactly the angle the constraint
    /// names — counterclockwise from the first line's own direction, which is what the value
    /// means.  Where that leaves the arc off the drawn segments, a witness ray says so.
    fn angle(&mut self, c: &Constraint) -> Option<Callout> {
        let (e1, e2) = (c.args[0].ent(), c.args[1].ent());
        let (i1, i2) = (e1.i(), e2.i());
        let text = claimed(c, dimension_text(c)?);
        let Some((corner, d1, d2)) = corner_of(self.sk, c) else {
            // the lines have gone parallel: their corner is at infinity, and there is no arc to
            // draw around it
            let (a1, b1) = seg(self.sk, i1);
            return Some(self.note(c.id, CalloutKind::Angular, mid(a1, b1), &text));
        };
        let sweep = angle_between(self.sk, e1, e2);
        let th0 = d1.1.atan2(d1.0);
        let (tw, th) = (self.px(FONT_PX * text_em(&text)), self.px(FONT_PX));
        // the label rides outside the arc, clear of it by its own half extent in that direction
        let half = |am: f64| 0.5 * (th * am.sin().abs() + tw * am.cos().abs());
        let place = self.placed(c).unwrap_or_else(|| {
            let reach = |i: usize| {
                let (a, b) = seg(self.sk, i);
                len(sub(a, corner)).max(len(sub(b, corner)))
            };
            let r = (0.45 * reach(i1).min(reach(i2)))
                .clamp(self.px(ARC_MIN_PX), self.px(ARC_MAX_PX));
            (0.5 * sweep, r + self.px(TEXT_GAP_PX) + half(0.5 * sweep))
        });
        let am = th0 + place.0;
        // the arc sits just inside the number, wherever that has been put
        let r = (place.1 - self.px(TEXT_GAP_PX) - half(place.0)).max(self.px(ARC_MIN_PX * 0.4));
        let mut k = Callout::new(c.id, CalloutKind::Angular, &text);
        k.place = place;
        k.arcs.push(CArc { c: corner, r, a0: th0, a1: th0 + sweep });
        // heads tangent to the arc, each facing out of the sweep
        let s = if sweep < 0.0 { -1.0 } else { 1.0 };
        for (t, back) in [(th0, -1.0), (th0 + sweep, 1.0)] {
            k.arrows.push(Arrow {
                at: along(corner, ray(t), r),
                dir: mul(perp(ray(t)), s * back),
            });
        }
        let out = r * 1.15;
        for (i, d) in [(i1, d1), (i2, d2)] {
            let (pa, pb) = seg(self.sk, i);
            let (ta, tb) = (dot(sub(pa, corner), d), dot(sub(pb, corner), d));
            if out < ta.min(tb) || out > ta.max(tb) {
                k.thin.push(Seg(corner, along(corner, d, out)));
            }
        }
        k.anchor = along(corner, ray(am), place.1);
        self.seal(&mut k);
        Some(k)
    }

    /// The tail of a radial callout: out to the elbow, then a horizontal landing long enough to
    /// underline the number, running away from the shape.
    fn land(&self, k: &mut Callout, rim: P, elbow: P, text: &str) {
        let away = if elbow.0 >= rim.0 { 1.0 } else { -1.0 };
        let run = self.px(FONT_PX * text_em(text)).max(self.px(LAND_PX));
        let tail = (elbow.0 + away * run, elbow.1);
        k.solid.push(Seg(rim, elbow));
        k.solid.push(Seg(elbow, tail));
        k.anchor = (
            0.5 * (elbow.0 + tail.0),
            elbow.1 + self.px(TEXT_GAP_PX + 0.5 * FONT_PX),
        );
        self.seal(k);
    }
}

/// A witness line from the near end of the segment `pa`–`pb` out to `at`, when the station `t`
/// (measured from `origin` along `d`) has run off the end of it.
fn witness(k: &mut Callout, pa: P, pb: P, origin: P, d: P, t: f64, at: P) {
    let (ta, tb) = (dot(sub(pa, origin), d), dot(sub(pb, origin), d));
    if t < ta.min(tb) {
        k.thin.push(Seg(if ta < tb { pa } else { pb }, at));
    } else if t > ta.max(tb) {
        k.thin.push(Seg(if ta < tb { pb } else { pa }, at));
    }
}
