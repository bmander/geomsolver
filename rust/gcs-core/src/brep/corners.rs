//! **Rounds and chamfers of a meridian's corners** (#183 X1, the lowering of step 7): a region of
//! lines and arcs — a meridian, as `solid::region::meridian` folds one — rounded or chamfered
//! where a step of one group of faces meets a step of another, with nothing but the planar
//! kernel's own steps.  A round is the ball of radius `r` rolled into that corner: its centre
//! where the two faces' carriers, each offset `r` toward the ball, cross, the corner replaced by
//! the arc between the two feet.  The ball lies inside the material at a convex corner and outside
//! it at a concave one, read off the turn of the boundary, never said.  A chamfer is the segment
//! between a point on each face, each leg measured one of two ways (`Leg`).
//!
//! What a round or a chamfer *is* is said in `rust/lib/std.sv`; this builds exactly what such a
//! set says at a meridian's corner, which `solid::region` checks before it calls it.
use super::planar::{Region, Seg, Step};
use std::f64::consts::{PI, TAU};

type P = [f64; 2];

fn sub(a: P, b: P) -> P { [a[0] - b[0], a[1] - b[1]] }
fn add(a: P, b: P) -> P { [a[0] + b[0], a[1] + b[1]] }
fn scale(a: P, k: f64) -> P { [a[0] * k, a[1] * k] }
fn dot(a: P, b: P) -> f64 { a[0] * b[0] + a[1] * b[1] }
fn cross(a: P, b: P) -> f64 { a[0] * b[1] - a[1] * b[0] }
fn norm(a: P) -> f64 { a[0].hypot(a[1]) }
fn dist(a: P, b: P) -> f64 { norm(sub(a, b)) }
fn unit(a: P) -> P { scale(a, 1.0 / norm(a)) }
fn left(a: P) -> P { [-a[1], a[0]] }

// -- the prototype -----------------------------------------------------------------------------

/// The line or circle a step lies on: what is offset and crossed.
#[derive(Clone, Copy, Debug)]
pub enum Carrier {
    Line { p: P, d: P },
    Circle { c: P, r: f64 },
}

impl Carrier {
    pub fn of(s: &Seg) -> Carrier {
        match *s {
            Seg::Line { a, b } => Carrier::Line { p: a, d: unit(sub(b, a)) },
            Seg::Arc { c, r, .. } => Carrier::Circle { c, r },
        }
    }
    /// The carrier moved `by` to the left of travel `t` at its point `x` (to the right where `by`
    /// is negative): a line shifted, a circle grown or shrunk; none where a circle shrinks away.
    pub fn offset(self, t: P, x: P, by: f64) -> Option<Carrier> {
        let n = unit(left(t));
        match self {
            Carrier::Line { p, d } => Some(Carrier::Line { p: add(p, scale(n, by)), d }),
            Carrier::Circle { c, r } => {
                let r = if dot(n, sub(c, x)) > 0.0 { r - by } else { r + by };
                (r > 0.0).then_some(Carrier::Circle { c, r })
            }
        }
    }
    /// The point of the carrier nearest `x`.
    pub fn foot(self, x: P) -> P {
        match self {
            Carrier::Line { p, d } => add(p, scale(d, dot(sub(x, p), d))),
            Carrier::Circle { c, r } => add(c, scale(unit(sub(x, c)), r)),
        }
    }
    /// Where two carriers cross, in closed form.
    pub fn meet(self, other: Carrier) -> Vec<P> {
        match (self, other) {
            (Carrier::Line { p, d }, Carrier::Line { p: q, d: e }) => {
                let den = cross(d, e);
                if den.abs() < 1e-15 {
                    return Vec::new();
                }
                vec![add(p, scale(d, cross(sub(q, p), e) / den))]
            }
            (Carrier::Line { p, d }, Carrier::Circle { c, r })
            | (Carrier::Circle { c, r }, Carrier::Line { p, d }) => {
                let f = sub(p, c);
                let (b, k) = (dot(f, d), dot(f, f) - r * r);
                let disc = b * b - k;
                if disc < 0.0 {
                    return Vec::new();
                }
                let w = disc.sqrt();
                vec![add(p, scale(d, -b - w)), add(p, scale(d, -b + w))]
            }
            (Carrier::Circle { c, r }, Carrier::Circle { c: c2, r: r2 }) => {
                let between = dist(c, c2);
                if between == 0.0 {
                    return Vec::new();
                }
                let along = (between * between + r * r - r2 * r2) / (2.0 * between);
                let h2 = r * r - along * along;
                if h2 < 0.0 {
                    return Vec::new();
                }
                let u = unit(sub(c2, c));
                let m = add(c, scale(u, along));
                let h = h2.sqrt();
                vec![add(m, scale(left(u), h)), add(m, scale(left(u), -h))]
            }
        }
    }
}

/// Where along a step a point of its carrier is (its fraction; an arc's measured the way it runs,
/// a point just before its start read as before it, as `planar` reads one).
pub fn param(s: &Seg, x: P) -> f64 {
    match *s {
        Seg::Line { a, b } => {
            let d = sub(b, a);
            dot(sub(x, a), d) / dot(d, d)
        }
        Seg::Arc { c, a0, sweep, .. } => {
            let phi = (x[1] - c[1]).atan2(x[0] - c[0]);
            let mut delta = ((phi - a0) * sweep.signum()).rem_euclid(TAU);
            if delta > PI + sweep.abs() / 2.0 {
                delta -= TAU;
            }
            delta / sweep.abs()
        }
    }
}

/// The stretch of a step from `t0` to `t1`.
pub fn part(s: &Seg, t0: f64, t1: f64) -> Seg {
    match *s {
        Seg::Line { .. } => Seg::Line { a: s.point(t0), b: s.point(t1) },
        Seg::Arc { c, r, a0, sweep } => {
            Seg::Arc { c, r, a0: a0 + sweep * t0, sweep: sweep * (t1 - t0) }
        }
    }
}

/// How a chamfer's leg is measured (#183 X0, finding 7).
#[derive(Clone, Copy, Debug)]
pub enum Leg {
    /// The point of the face `d` from the edge: a straight distance, along a line its length.
    FromEdge(f64),
    /// The point of the face `d` from the other face's surface, square to it.
    FromOther(f64),
}

/// What is put in a corner: where the step arriving at it ends and the step leaving it starts
/// (their fractions), and the step between.
pub type Cut = (f64, f64, Seg);

/// The ball of radius `r` rolled into the corner where `s` arrives and `n` leaves, on side `side`
/// (+1 the left: inside the material at a convex corner, outside it at a concave one).
pub fn ball(s: &Seg, n: &Seg, side: f64, r: f64) -> Result<Cut, String> {
    let x = s.end();
    let (ts, tn) = (unit(s.tangent(1.0)), unit(n.tangent(0.0)));
    let off = |c: Carrier, t: P| c.offset(t, x, side * r).ok_or("a face too curved for the ball");
    let (cs, cn) = (off(Carrier::of(s), ts)?, off(Carrier::of(n), tn)?);
    let centre = cs.meet(cn).into_iter().min_by(|a, b| dist(*a, x).total_cmp(&dist(*b, x)))
        .ok_or("the faces' offsets do not meet")?;
    let (fs, fn_) = (Carrier::of(s).foot(centre), Carrier::of(n).foot(centre));
    let a0 = (fs[1] - centre[1]).atan2(fs[0] - centre[0]);
    let a1 = (fn_[1] - centre[1]).atan2(fn_[0] - centre[0]);
    let mut sweep = (a1 - a0).rem_euclid(TAU);
    if side < 0.0 {
        sweep -= TAU;
    }
    Ok((param(s, fs), param(n, fn_), Seg::Arc { c: centre, r, a0, sweep }))
}

/// The flat across the corner where `s` arrives and `n` leaves, its leg on `s` measured by
/// `legs[0]` and on `n` by `legs[1]`.
pub fn flat(s: &Seg, n: &Seg, side: f64, legs: [Leg; 2]) -> Result<Cut, String> {
    let x = s.end();
    let (ts, tn) = (unit(s.tangent(1.0)), unit(n.tangent(0.0)));
    // the leg's end on `on`, by `leg`: of the crossings of `on`'s carrier, the one nearest the
    // corner along `on` (`before`: on the step arriving at the corner)
    let end = |on: &Seg, other: &Seg, t_other: P, leg: Leg, before: bool| -> Result<f64, String> {
        let crossing = match leg {
            Leg::FromEdge(d) => Carrier::Circle { c: x, r: d },
            Leg::FromOther(d) => Carrier::of(other).offset(t_other, x, side * d)
                .ok_or("a face too curved for the leg")?,
        };
        Carrier::of(on).meet(crossing).into_iter().map(|p| param(on, p))
            .filter(|&t| if before { t < 1.0 } else { t > 0.0 })
            .min_by(|a, b| {
                let off = |t: f64| if before { 1.0 - t } else { t };
                off(*a).total_cmp(&off(*b))
            })
            .ok_or_else(|| "the leg does not reach the face".to_string())
    };
    let t_s = end(s, n, tn, legs[0], true)?;
    let t_n = end(n, s, ts, legs[1], false)?;
    Ok((t_s, t_n, Seg::Line { a: s.point(t_s), b: n.point(t_n) }))
}

/// `region` with every corner where a step of `a` meets a step of `b` (either way round) cut by
/// `cut`, the new step tagged `tag`.  A seam (the steps meeting tangent) is no corner.  Refused
/// where the cut does not fit on a face: its foot past the far end of a step, or two cuts meeting
/// within one (a face the rounds would consume, #183 X0's finding 9).
pub fn corners(region: &Region, a: &[u32], b: &[u32], tag: u32,
           cut: &dyn Fn(&Step, &Step, f64) -> Result<Cut, String>) -> Result<Region, String> {
    let mut loops = Vec::new();
    for l in &region.loops {
        let k = l.len();
        // each step's stretch kept, and what follows it
        let mut keep: Vec<(f64, f64)> = vec![(0.0, 1.0); k];
        let mut after: Vec<Option<Seg>> = vec![None; k];
        for i in 0..k {
            let (s, n) = (&l[i], &l[(i + 1) % k]);
            let named = (a.contains(&s.tag) && b.contains(&n.tag))
                || (b.contains(&s.tag) && a.contains(&n.tag));
            if !named || s.chart.is_some() || n.chart.is_some() {
                continue;
            }
            let (ts, tn) = (unit(s.seg.tangent(1.0)), unit(n.seg.tangent(0.0)));
            let turn = cross(ts, tn);
            if turn.abs() <= 1e-12 && dot(ts, tn) > 0.0 {
                continue;
            }
            let (t_s, t_n, seg) = cut(s, n, turn.signum())?;
            if !(0.0..=1.0).contains(&t_s) || !(0.0..=1.0).contains(&t_n) {
                return Err(format!("the cut between faces {} and {} does not fit on them",
                    s.tag, n.tag));
            }
            keep[i].1 = t_s;
            keep[(i + 1) % k].0 = t_n;
            after[i] = Some(seg);
        }
        let mut walk = Vec::new();
        for i in 0..k {
            let (t0, t1) = keep[i];
            if t0 >= t1 {
                return Err(format!("the cuts at both ends of face {} meet within it", l[i].tag));
            }
            walk.push(Step { seg: part(&l[i].seg, t0, t1), ..l[i] });
            if let Some(seg) = after[i] {
                walk.push(Step { seg, chart: None, tag });
            }
        }
        loops.push(walk);
    }
    Ok(Region { loops })
}

/// `A round(r) B`: every corner between faces `a` and `b` rounded by the ball of radius `r`, filled
/// where concave and taken off where convex; a radius of zero the region as it was.
pub fn round(region: &Region, a: &[u32], b: &[u32], r: f64, tag: u32) -> Result<Region, String> {
    if r == 0.0 {
        return Ok(region.clone());
    }
    corners(region, a, b, tag, &|s, n, side| ball(&s.seg, &n.seg, side, r))
}

/// `A chamfer(…) B`: every corner between faces `a` and `b` cut flat, the leg on the face of `a`
/// by `legs[0]` and on the face of `b` by `legs[1]`.
pub fn chamfer(region: &Region, a: &[u32], b: &[u32], legs: [Leg; 2], tag: u32)
    -> Result<Region, String> {
    corners(region, a, b, tag, &|s, n, side| {
        let legs = if a.contains(&s.tag) { legs } else { [legs[1], legs[0]] };
        flat(&s.seg, &n.seg, side, legs)
    })
}

