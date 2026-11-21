//! **Corner rounds** (#576): the arcs a chain's `A -> round(r: R) B` joint mints, recorded with
//! what they round — the two sides, each a line or an arc (#962), and the virtual corner they
//! were written meeting at — so the construction a round is (where it touches each side, its
//! centre) can be worked out where the drawing stands, and a round that does not fit said in its
//! own terms.
//!
//! **The round is the circle of its radius tangent to both sides, inside the corner.**  Its
//! centre is on each side's offset by the radius toward the side the walk turns: a line moved
//! square to itself, an arc's circle grown by the radius where the round is outside it or shrunk
//! where it is inside (the walk turning toward the arc's centre).  Two lines' offsets cross once;
//! a line's and a circle's, or two circles', cross twice, mirror images — about the square from
//! the circle's centre to the line, or the line of the two centres — and the round is the one on
//! the corner's side of that mirror, the root that comes to the corner as the radius comes to
//! nothing.  Every number of it is a closed form.  A corner where an arc turns toward the round
//! holds it only so large (`Rounded::most`): past that the two offsets no longer meet.

use super::{EntKind, EntRef, Sketch};

/// One corner round: its arc, the side the walk arrives along (`from`, whose exit — a line's
/// `p2`, an arc's `end` as written — is the arc's point there) and the one it leaves along
/// (`to`, whose entry — `p1`, `start` — is), each a line or an arc; the corner they were written
/// meeting at where one was written; and the way the corner turns — `1` left, the arc
/// counter-clockwise from `from` to `to`; `-1` right, the arc drawn from its end (start and end
/// swapped and filed in `Sketch::reversed`, so what reads it reads it as written).  What each is
/// called, for a message.  Set by the elaborator (`program::rounds`).
#[derive(Clone, Debug)]
pub struct Round {
    pub arc: u32,
    pub from: EntRef,
    pub to: EntRef,
    pub corner: Option<u32>,
    pub turn: i8,
    pub names: [String; 3],
}

/// One side of a corner as a round reads it at the corner: a line, by a point it runs toward
/// away from the corner (for the side arrived along, where it came from), or an arc's circle, by
/// its centre and radius and whether the walk runs round it counter-clockwise.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum RoundSide {
    Line((f64, f64)),
    Arc { center: (f64, f64), radius: f64, ccw: bool },
}

/// Where a round of radius `r` touches the two sides at corner `p`, and where its centre is — the
/// round inside the corner, so on the side the walk from the first side through `p` onto the
/// second turns toward.  Between two lines each setback is `r / tan(θ/2)` from `p`, the centre
/// `r / sin(θ/2)` along the bisector, `θ` the angle at `p`.  `None` where the corner is no
/// corner: the sides running straight on there, or folded back on each other, or a side of no
/// length.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Rounded {
    pub turn: i8,
    /// How far each side's point is from the corner along the side: a length along a line, the
    /// length of the arc's stretch between them along an arc.
    pub setback: [f64; 2],
    pub start: (f64, f64),
    pub end: (f64, f64),
    pub center: (f64, f64),
    /// The largest radius the corner holds, where an arc turning toward the round's centre
    /// bounds it — `(ρ + h)/2` beside a line whose turn's side the arc's centre stands `h` off,
    /// `(ρ₁ + ρ₂ − D)/2` between two arcs both turning toward it, `(ρᵢ − ρⱼ + D)/2` where only
    /// the `i`th does, `D` between their centres; `None` where it holds any.  A round asked for
    /// more is placed at this, the nearest the corner comes to holding it.
    pub most: Option<f64>,
    /// Whether the round is inside each side's circle — the walk turning toward an arc's centre
    /// there; never beside a line.
    pub inside: [bool; 2],
}

/// How near a turn may come to none — `|sin θ|` — before a corner is no corner.
const STRAIGHT: f64 = 1e-9;

fn unit(v: (f64, f64)) -> Option<(f64, f64)> {
    let l = v.0.hypot(v.1);
    (l > 0.0).then(|| (v.0 / l, v.1 / l))
}

fn sub(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 - b.0, a.1 - b.1)
}

fn dot(a: (f64, f64), b: (f64, f64)) -> f64 {
    a.0 * b.0 + a.1 * b.1
}

fn cross(a: (f64, f64), b: (f64, f64)) -> f64 {
    a.0 * b.1 - a.1 * b.0
}

/// One side read at the corner: which way the walk runs along it there, and toward the turn's
/// side — a line's unit normal that way, an arc's `σ`, `-1` where its centre is that way (the
/// round inside its circle, the circle shrunk by the radius) and `1` where it is not.
#[derive(Clone, Copy, Debug)]
struct Leg {
    side: RoundSide,
    walk: (f64, f64),
    normal: (f64, f64),
    sigma: f64,
}

/// The corner's two legs and its turn, where it is a corner.
fn legs(p: (f64, f64), a: RoundSide, b: RoundSide) -> Option<(i8, [Leg; 2])> {
    // the direction each side runs away from the corner
    let away = |s: RoundSide, first: bool| match s {
        RoundSide::Line(far) => unit(sub(far, p)),
        RoundSide::Arc { center, ccw, .. } => {
            let r = unit(sub(p, center))?;
            let walk = if ccw { (-r.1, r.0) } else { (r.1, -r.0) };
            Some(if first { (-walk.0, -walk.1) } else { walk })
        }
    };
    let (u, v) = (away(a, true)?, away(b, false)?);
    // the turn from arriving along `−u` onto `v`: (−u) × v
    let sin = u.1 * v.0 - u.0 * v.1;
    if sin.abs() <= STRAIGHT {
        return None;
    }
    let turn: i8 = if sin > 0.0 { 1 } else { -1 };
    let leg = |s: RoundSide, walk: (f64, f64)| {
        let normal = if turn > 0 { (-walk.1, walk.0) } else { (walk.1, -walk.0) };
        let sigma = match s {
            RoundSide::Arc { center, .. } if dot(normal, sub(center, p)) > 0.0 => -1.0,
            _ => 1.0,
        };
        Leg { side: s, walk, normal, sigma }
    };
    Some((turn, [leg(a, (-u.0, -u.1)), leg(b, v)]))
}

/// The largest radius corner `p` of these legs holds (`Rounded::most`).
fn most(p: (f64, f64), legs: &[Leg; 2]) -> Option<f64> {
    match (legs[0].side, legs[1].side) {
        (RoundSide::Line(_), RoundSide::Line(_)) => None,
        (RoundSide::Line(_), RoundSide::Arc { center, radius, .. })
        | (RoundSide::Arc { center, radius, .. }, RoundSide::Line(_)) => {
            let first = matches!(legs[0].side, RoundSide::Line(_));
            let (line, arc) = if first { (0, 1) } else { (1, 0) };
            (legs[arc].sigma < 0.0)
                .then(|| (radius + dot(legs[line].normal, sub(center, p))) / 2.0)
        }
        (RoundSide::Arc { center: c1, radius: r1, .. },
         RoundSide::Arc { center: c2, radius: r2, .. }) => {
            let d = sub(c2, c1).0.hypot(sub(c2, c1).1);
            match (legs[0].sigma < 0.0, legs[1].sigma < 0.0) {
                (true, true) => Some((r1 + r2 - d) / 2.0),
                (true, false) => Some((r1 - r2 + d) / 2.0),
                (false, true) => Some((r2 - r1 + d) / 2.0),
                (false, false) => None,
            }
        }
    }
}

/// The round of radius `r` at corner `p` between sides `a` (arrived along) and `b` (left along)
/// (`Rounded`).
pub fn rounded(p: (f64, f64), a: RoundSide, b: RoundSide, r: f64) -> Option<Rounded> {
    let (turn, legs) = legs(p, a, b)?;
    let most = most(p, &legs);
    let r = most.map_or(r, |m| r.min(m));
    let center = match (a, b) {
        (RoundSide::Line(_), RoundSide::Line(_)) => {
            // half the angle at `p` between the two sides, the centre along its bisector
            let (u, v) = ((-legs[0].walk.0, -legs[0].walk.1), legs[1].walk);
            let theta = cross(v, u).abs().atan2(dot(u, v));
            let reach = r / (theta / 2.0).sin();
            let bis = unit((u.0 + v.0, u.1 + v.1))?;
            (p.0 + reach * bis.0, p.1 + reach * bis.1)
        }
        (RoundSide::Line(_), RoundSide::Arc { center, radius, .. })
        | (RoundSide::Arc { center, radius, .. }, RoundSide::Line(_)) => {
            let (line, arc) = if matches!(a, RoundSide::Line(_)) { (0, 1) } else { (1, 0) };
            let (w, n) = (legs[line].walk, legs[line].normal);
            // along the line's offset `q + t·w`, where it meets the arc's circle grown or shrunk
            // by `r`: `t² + 2bt + c = 0`, the root that is nil at `r = 0`, `c / (−b ∓ √…)`
            let q = (p.0 + r * n.0, p.1 + r * n.1);
            let grown = radius + legs[arc].sigma * r;
            let bb = dot(w, sub(p, center));
            let qc = sub(q, center);
            let c = dot(qc, qc) - grown * grown;
            let disc = (bb * bb - c).max(0.0);
            let big = -bb - bb.signum() * disc.sqrt();
            let t = if big == 0.0 { 0.0 } else { c / big };
            (q.0 + t * w.0, q.1 + t * w.1)
        }
        (RoundSide::Arc { center: c1, radius: r1, .. },
         RoundSide::Arc { center: c2, radius: r2, .. }) => {
            // the two circles grown or shrunk by `r` crossing, on the corner's side of the line
            // through their centres
            let (g1, g2) = (r1 + legs[0].sigma * r, r2 + legs[1].sigma * r);
            let dc = sub(c2, c1);
            let d = dc.0.hypot(dc.1);
            let e = unit(dc)?;
            let along = (g1 * g1 - g2 * g2 + d * d) / (2.0 * d);
            let h = (g1 * g1 - along * along).max(0.0).sqrt();
            let across = (-e.1, e.0);
            let s = cross(e, sub(p, c1)).signum();
            (c1.0 + along * e.0 + s * h * across.0, c1.1 + along * e.1 + s * h * across.1)
        }
    };
    // where it touches each side: the foot of the centre on a line, along the radius to the
    // centre on an arc's circle
    let touch = |leg: &Leg| match leg.side {
        RoundSide::Line(_) => (center.0 - r * leg.normal.0, center.1 - r * leg.normal.1),
        RoundSide::Arc { center: c, radius, .. } => match unit(sub(center, c)) {
            Some(u) => (c.0 + radius * u.0, c.1 + radius * u.1),
            None => p,
        },
    };
    let (start, end) = (touch(&legs[0]), touch(&legs[1]));
    let back = |leg: &Leg, at: (f64, f64)| match leg.side {
        RoundSide::Line(_) => sub(at, p).0.hypot(sub(at, p).1),
        RoundSide::Arc { center: c, radius, .. } => {
            let (x, y) = (sub(p, c), sub(at, c));
            radius * cross(x, y).abs().atan2(dot(x, y))
        }
    };
    Some(Rounded {
        turn,
        setback: [back(&legs[0], start), back(&legs[1], end)],
        start,
        end,
        center,
        most,
        inside: [legs[0].sigma < 0.0, legs[1].sigma < 0.0],
    })
}

/// The radius of the round at corner `p` that touches side `x` (0 the first, 1 the second)
/// exactly at its point `f` — where a round past that side's far end would fit it — tangent to
/// the other side as every round there is: its centre `f + R·m`, `m` the side's normal toward
/// the turn there, set square to the other side's line or on its circle grown or shrunk by `R`.
/// `None` where no round there touches `f`.
pub fn fitting(p: (f64, f64), a: RoundSide, b: RoundSide, x: usize, f: (f64, f64)) -> Option<f64> {
    let (_, legs) = legs(p, a, b)?;
    let (lx, ly) = (&legs[x], &legs[1 - x]);
    let m = match lx.side {
        RoundSide::Line(_) => lx.normal,
        RoundSide::Arc { center, .. } => {
            let u = unit(sub(f, center))?;
            (lx.sigma * u.0, lx.sigma * u.1)
        }
    };
    let r = match ly.side {
        RoundSide::Line(_) => {
            let den = 1.0 - dot(ly.normal, m);
            (den.abs() > STRAIGHT).then(|| dot(ly.normal, sub(f, p)) / den)?
        }
        RoundSide::Arc { center, radius, .. } => {
            let bv = sub(f, center);
            let den = 2.0 * (dot(m, bv) - radius * ly.sigma);
            (den.abs() > STRAIGHT).then(|| (radius * radius - dot(bv, bv)) / den)?
        }
    };
    (r > 0.0).then_some(r)
}

/// Where two lines through `(a, b)` and `(c, d)` cross, where they do.
fn crossing(a: (f64, f64), b: (f64, f64), c: (f64, f64), d: (f64, f64)) -> Option<(f64, f64)> {
    let (r, s) = ((b.0 - a.0, b.1 - a.1), (d.0 - c.0, d.1 - c.1));
    let den = r.0 * s.1 - r.1 * s.0;
    if den.abs() <= STRAIGHT * r.0.hypot(r.1) * s.0.hypot(s.1) {
        return None;
    }
    let t = ((c.0 - a.0) * s.1 - (c.1 - a.1) * s.0) / den;
    Some((a.0 + t * r.0, a.1 + t * r.1))
}

/// A side as a whole figure, for where two cross: a line through two points, or a circle.
#[derive(Clone, Copy)]
enum Whole {
    Line((f64, f64), (f64, f64)),
    Circle((f64, f64), f64),
}

/// Where two sides cross nearest `near`, where they do.
fn crossing_near(x: Whole, y: Whole, near: (f64, f64)) -> Option<(f64, f64)> {
    let pick = |ps: [(f64, f64); 2]| {
        let d = |q: (f64, f64)| sub(q, near).0.hypot(sub(q, near).1);
        if d(ps[0]) <= d(ps[1]) { ps[0] } else { ps[1] }
    };
    match (x, y) {
        (Whole::Line(a, b), Whole::Line(c, d)) => crossing(a, b, c, d),
        (Whole::Line(a, b), Whole::Circle(c, r)) | (Whole::Circle(c, r), Whole::Line(a, b)) => {
            let w = unit(sub(b, a))?;
            let foot = dot(sub(c, a), w);
            let f = (a.0 + foot * w.0, a.1 + foot * w.1);
            let h2 = r * r - dot(sub(f, c), sub(f, c));
            let h = (h2 >= 0.0).then(|| h2.sqrt())?;
            Some(pick([(f.0 + h * w.0, f.1 + h * w.1), (f.0 - h * w.0, f.1 - h * w.1)]))
        }
        (Whole::Circle(c1, r1), Whole::Circle(c2, r2)) => {
            let dc = sub(c2, c1);
            let d = dc.0.hypot(dc.1);
            let e = unit(dc)?;
            let along = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d);
            let h2 = r1 * r1 - along * along;
            let h = (h2 >= 0.0).then(|| h2.sqrt())?;
            let m = (c1.0 + along * e.0, c1.1 + along * e.1);
            Some(pick([(m.0 - h * e.1, m.1 + h * e.0), (m.0 + h * e.1, m.1 - h * e.0)]))
        }
    }
}

impl Sketch {
    /// A round's side's two ends as written, `(start, end)`: a line's `p1` and `p2`, an arc's
    /// `start:` and `end:` — the model's own swapped back where it turns clockwise.
    pub fn side_ends(&self, e: EntRef) -> (u32, u32) {
        match e.kind {
            EntKind::Arc => {
                let a = &self.arcs[e.i()];
                if self.arc_reversed(e.i()) { (a.end, a.start) } else { (a.start, a.end) }
            }
            _ => (self.lines[e.i()].p1, self.lines[e.i()].p2),
        }
    }

    /// Side `e` of a round as it reads at the corner, `first` the side arrived along: a line by
    /// the corner at its far end (`round_far`), an arc by its circle as the drawing stands and
    /// the way it is written to turn.
    pub fn round_side(&self, e: EntRef, first: bool) -> RoundSide {
        match e.kind {
            EntKind::Arc => {
                let a = &self.arcs[e.i()];
                RoundSide::Arc {
                    center: self.point_xy(a.center as usize),
                    radius: self.params[a.radius as usize].value,
                    ccw: !self.arc_reversed(e.i()),
                }
            }
            _ => {
                let (s, t) = self.side_ends(e);
                RoundSide::Line(self.round_far(if first { s } else { t }))
            }
        }
    }

    /// The corner round `k` rounds, where the drawing stands: its point where one was written,
    /// else where its two sides cross — of a line and an arc's circle, or two circles, the
    /// crossing nearest the round's own ends.
    pub fn round_corner(&self, k: &Round) -> Option<(f64, f64)> {
        if let Some(c) = k.corner {
            return Some(self.point_xy(c as usize));
        }
        let whole = |e: EntRef| match e.kind {
            EntKind::Arc => {
                let a = &self.arcs[e.i()];
                Whole::Circle(self.point_xy(a.center as usize),
                              self.params[a.radius as usize].value)
            }
            _ => {
                let l = &self.lines[e.i()];
                Whole::Line(self.point_xy(l.p1 as usize), self.point_xy(l.p2 as usize))
            }
        };
        let arc = &self.arcs[k.arc as usize];
        let (s, t) = (self.point_xy(arc.start as usize), self.point_xy(arc.end as usize));
        crossing_near(whole(k.from), whole(k.to), ((s.0 + t.0) / 2.0, (s.1 + t.1) / 2.0))
    }

    /// Where a side's end at `p` stands for a round further along it: the corner of the round
    /// that point is the end of, where one is, else the point itself — so a side between two
    /// rounds runs corner to corner.
    pub fn round_far(&self, p: u32) -> (f64, f64) {
        let mine = self.rounds.iter().find(|k| {
            self.side_ends(k.from).1 == p || self.side_ends(k.to).0 == p
        });
        mine.and_then(|k| self.round_corner(k)).unwrap_or_else(|| self.point_xy(p as usize))
    }

    /// Round `k` where the drawing stands (`rounded`), its radius the arc's.
    pub fn round_at(&self, k: &Round) -> Option<Rounded> {
        let r = self.params[self.arcs[k.arc as usize].radius as usize].value;
        let (a, b) = (self.round_side(k.from, true), self.round_side(k.to, false));
        rounded(self.round_corner(k)?, a, b, r)
    }
}

/// A length as a message says it, to two places — rounded, or where it is a bound, down.
fn said(sk: &Sketch, v: f64, down: bool) -> String {
    let v = match down {
        true => (v * 100.0 + 1e-6).floor() / 100.0,
        false => (v * 100.0).round() / 100.0,
    };
    crate::units::Dim::LENGTH.number_text(v, sk.units)
}

/// How long side `e` runs between the corners at its two ends (`round_far`): a line's length,
/// an arc's along it, the way it is written to turn — a whole turn where they meet.
fn side_length(sk: &Sketch, e: EntRef) -> f64 {
    let (s, t) = sk.side_ends(e);
    let (a, b) = (sk.round_far(s), sk.round_far(t));
    match sk.round_side(e, true) {
        RoundSide::Line(_) => sub(b, a).0.hypot(sub(b, a).1),
        RoundSide::Arc { center, radius, ccw } => {
            let (x, y) = (sub(a, center), sub(b, center));
            let turned = y.1.atan2(y.0) - x.1.atan2(x.0);
            let sweep = if ccw { turned } else { -turned }.rem_euclid(std::f64::consts::TAU);
            radius * if sweep == 0.0 { std::f64::consts::TAU } else { sweep }
        }
    }
}

/// **The rounds that do not fit where the drawing stands** (#576), each said by name: a round
/// larger than its corner holds where an arc there turns toward it (#962), with the radius it
/// holds; a round whose setback runs past its side's far end, with the largest radius that fits
/// the side as drawn; two rounds whose setbacks on one side add to more than it is long; a round
/// at a corner its sides run straight through.  Read after a solve, beside the solids' own
/// complaints, so a profile refused as self-intersecting is told which round made it so.
pub fn overruns(sk: &Sketch) -> Vec<String> {
    let mut out = Vec::new();
    // the radius each round says — its `radius` row's number, which a solve that could not hold
    // it leaves stated however far the arc's own was pulled from it — else the arc's own
    let radius = |k: &Round| {
        let arc = EntRef::arc(k.arc as usize);
        sk.constraints.iter()
            .find(|c| c.kind == crate::constraints::CKind::Radius && c.args[0].ent() == arc)
            .map(|c| c.args[1].value(sk))
            .unwrap_or(sk.params[sk.arcs[k.arc as usize].radius as usize].value)
    };
    let mut at: Vec<Option<Rounded>> = sk.rounds.iter().map(|k| sk.round_at(k)).collect();
    for (k, r) in sk.rounds.iter().zip(at.iter_mut()) {
        if r.is_none() && sk.round_corner(k).is_some() {
            out.push(format!("`{}` rounds no corner: `{}` and `{}` run straight on there",
                k.names[0], k.names[1], k.names[2]));
        }
        let Some(most) = r.and_then(|r| r.most) else { continue };
        if radius(k) > most * (1.0 + 1e-9) + 1e-9 {
            let arcs: Vec<&str> = [(k.from, &k.names[1]), (k.to, &k.names[2])].into_iter()
                .filter(|(e, _)| e.kind == EntKind::Arc).map(|(_, n)| n.as_str()).collect();
            out.push(format!(
                "`{}` (r {}) is more than the corner of `{}` and `{}` holds: {} turns toward it \
                 there; it fits up to r {}",
                k.names[0], said(sk, radius(k), false), k.names[1], k.names[2],
                arcs.iter().map(|n| format!("`{n}`")).collect::<Vec<_>>().join(" and "),
                said(sk, most, true)));
            // its setbacks are those of the round it fits, not of this one
            *r = None;
        }
    }
    // each side some round sets back along, once: the round at its start and at its end
    let mut sides: Vec<EntRef> = sk.rounds.iter().flat_map(|k| [k.from, k.to]).collect();
    sides.sort_unstable();
    sides.dedup();
    for e in sides {
        let long = side_length(sk, e);
        // the round the side is entered from, and the one it leaves into, each with its setback
        // along it
        let set = |i: Option<usize>, x: usize| {
            i.and_then(|i| Some((&sk.rounds[i], at[i]?, x))).filter(|(_, r, x)| r.setback[*x] > 0.0)
        };
        let entering = set(sk.rounds.iter().position(|k| k.to == e), 1);
        let leaving = set(sk.rounds.iter().position(|k| k.from == e), 0);
        let back = |r: Option<(&Round, Rounded, usize)>| r.map_or(0.0, |(_, r, x)| r.setback[x]);
        let (da, db) = (back(entering), back(leaving));
        if da + db <= long * (1.0 + 1e-9) + 1e-9 {
            continue;
        }
        match (entering, leaving) {
            (Some((i, _, _)), Some((j, _, _))) => out.push(format!(
                "`{}` (r {}) and `{}` (r {}) overlap on `{}`: they set back {} and {} from \
                 corners {} apart",
                i.names[0], said(sk, radius(i), false), j.names[0], said(sk, radius(j), false),
                j.names[1], said(sk, da, false), said(sk, db, false), said(sk, long, false))),
            (Some((k, r, x)), None) | (None, Some((k, r, x))) => {
                let name = if x == 0 { &k.names[1] } else { &k.names[2] };
                // the round touching the side at the corner at its far end
                let (s, t) = sk.side_ends(e);
                let far = sk.round_far(if x == 0 { s } else { t });
                let corner = sk.round_corner(k);
                let (a, b) = (sk.round_side(k.from, true), sk.round_side(k.to, false));
                let fits = corner.and_then(|p| fitting(p, a, b, x, far))
                    .map(|f| r.most.map_or(f, |m| f.min(m)));
                out.push(match fits {
                    Some(f) => format!("`{}` (r {}) runs past `{name}`'s far end; it fits up to \
                        r {}", k.names[0], said(sk, radius(k), false), said(sk, f, true)),
                    None => format!("`{}` (r {}) runs past `{name}`'s far end",
                        k.names[0], said(sk, radius(k), false)),
                });
            }
            (None, None) => {}
        }
    }
    out
}
