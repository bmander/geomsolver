//! **Rounds and chamfers of a meridian's corners** (#183 X1, the lowering of step 7): a region of
//! lines and arcs — a meridian, as `solid::region::meridian` folds one — rounded or chamfered
//! where a step of one group of faces meets a step of another, with nothing but the planar
//! kernel's own steps.  A round is the ball of radius `r` rolled into that corner: its centre
//! where the two faces' carriers, each offset `r` toward the ball, cross, the corner replaced by
//! the arc between the two feet.  The ball lies inside the material at a convex corner and outside
//! it at a concave one, read off the turn of the boundary, never said.  A chamfer is the segment
//! between a point on each face, each leg measured one of two ways (`Leg`); `std.Chamfered`'s flat
//! is the curve its sum of distances makes (`sum`): that segment between lines, a conic's arc
//! between a line and a circle.
//!
//! What a round or a chamfer *is* is said in `rust/lib/std.sv`; this builds exactly what such a
//! set says at a meridian's corner, which `solid::region` checks before it calls it.
use super::planar::{Region, Seg, Step};
use crate::fmath::Det;
use std::f64::consts::TAU;

use super::plane2::{self, add, cross, dist, dot, left, norm, scale, sub, unit, Touch, P};

// -- the prototype -----------------------------------------------------------------------------

/// The line or circle a step lies on: what is offset and crossed.
pub use super::plane2::Carrier;

impl Carrier {
    pub fn of(s: &Seg) -> Carrier {
        match *s {
            Seg::Line { a, b } => Carrier::Line { p: a, d: unit(sub(b, a)) },
            Seg::Arc { c, r, .. } => Carrier::Circle { c, r },
            Seg::Conic { .. } => unreachable!("`corners` cuts no conic"),
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
        plane2::meet(self, other, Touch::Exact).unwrap_or_default()
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
    let a0 = (fs[1] - centre[1]).datan2(fs[0] - centre[0]);
    let a1 = (fn_[1] - centre[1]).datan2(fn_[0] - centre[0]);
    let mut sweep = (a1 - a0).rem_euclid(TAU);
    if side < 0.0 {
        sweep -= TAU;
    }
    Ok((s.param(fs), n.param(fn_), Seg::Arc { c: centre, r, a0, sweep }))
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
        Carrier::of(on).meet(crossing).into_iter().map(|p| on.param(p))
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

/// The curve `std.Chamfered` bounds at the corner where `s` arrives and `n`
/// leaves: the points whose distances from the two carriers, each toward side `side`, make
/// `d_s / legs[1] + d_n / legs[0]` one — through the ends `flat` finds with both legs measured from
/// the other face.  Between lines the segment; between a line and a circle about `c` the conic
/// `|q − c| = α + β d_L(q)` (`d_L` affine), focus `c`, eccentricity the legs' ratio, as the
/// rational quadratic Bézier over its ends and where their tangents meet.  Between circles with
/// equal legs a conic too, the centres its foci (`between_circles`); with unequal ones a Cartesian
/// oval, refused.
pub fn sum(s: &Seg, n: &Seg, side: f64, legs: [f64; 2]) -> Result<Cut, String> {
    let (t_s, t_n, chord) = flat(s, n, side, [Leg::FromOther(legs[0]), Leg::FromOther(legs[1])])?;
    let (p0, p2) = (s.point(t_s), n.point(t_n));
    // the line's carrier as `n · q − h`, positive toward `side`; the circle's centre, radius and
    // which way its distance runs toward `side`
    let (line, circle, l_line, l_circle) = match (s, n) {
        (Seg::Line { .. }, Seg::Line { .. }) => return Ok((t_s, t_n, chord)),
        (Seg::Line { a, b }, Seg::Arc { c, r, sweep, .. }) => {
            ((*a, *b), (*c, *r, *sweep), legs[0], legs[1])
        }
        (Seg::Arc { c, r, sweep, .. }, Seg::Line { a, b }) => {
            ((*a, *b), (*c, *r, *sweep), legs[1], legs[0])
        }
        (Seg::Arc { c: cs, r: rs, sweep: ws, .. }, Seg::Arc { c: cn, r: rn, sweep: wn, .. }) => {
            return between_circles(t_s, t_n, p0, p2, [(*cs, *rs, *ws, legs[1]), (*cn, *rn, *wn, legs[0])], side);
        }
        _ => return Err("a flat between a conic and a face takes no cut".into()),
    };
    let nl = scale(left(unit(sub(line.1, line.0))), side);
    let h = dot(nl, line.0);
    let (c, r, sweep) = circle;
    // d_C = k (r − |q − c|) = l_line (1 − d_L / l_circle): `l_line` the leg on the line, measured
    // from the circle, and `l_circle` the leg on the circle
    let k = side * sweep.signum();
    let (alpha, beta) = (r - k * l_line, k * l_line / l_circle);
    let f = |q: P| dist(q, c) - alpha - beta * (dot(nl, q) - h);
    let grad = |q: P| sub(unit(sub(q, c)), scale(nl, beta));
    // the tangents at the ends, and where they meet
    let (g0, g2) = (grad(p0), grad(p2));
    let (d0, d2) = (left(g0), left(g2));
    let den = cross(d0, d2);
    if den.abs() <= 1e-12 * norm(d0) * norm(d2) {
        return Err("the flat's tangents at its ends do not meet".into());
    }
    let m = add(p0, scale(d0, cross(sub(p2, p0), d2) / den));
    // the shoulder: where the conic crosses from the chord's middle toward `m`, a quadratic
    // (nearly linear for a parabola, along its axis)
    let mid = scale(add(p0, p2), 0.5);
    let v = sub(m, mid);
    let (e, rhs0, rhs1) = (sub(mid, c), alpha + beta * (dot(nl, mid) - h), beta * dot(nl, v));
    let roots = crate::roots::quadratic(dot(v, v) - rhs1 * rhs1, 2.0 * (dot(e, v) - rhs0 * rhs1),
        dot(e, e) - rhs0 * rhs0);
    let shoulder = roots.into_iter()
        .filter(|&u| u > 0.0 && u < 1.0 && rhs0 + u * rhs1 >= 0.0)
        .min_by(|a, b| f(add(mid, scale(v, *a))).abs().total_cmp(&f(add(mid, scale(v, *b))).abs()))
        .ok_or("the flat's conic leaves its control triangle")?;
    let w = shoulder / (1.0 - shoulder);
    Ok((t_s, t_n, Seg::Conic { a: p0, m, b: p2, w }))
}

/// `sum` between two circles, each `(centre, radius, sweep, leg)` with the leg its distance is
/// divided by: `Σ kᵢ (rᵢ − |q − cᵢ|) / legᵢ = 1`, `kᵢ` which way each distance runs toward `side`.
/// Where the two are divided alike (equal legs) it is a conic with the centres its foci — an
/// ellipse where the distances add, a hyperbola's branch where they differ — the rational quadratic
/// Bézier over its ends, where their tangents meet, its middle weighed by the point it passes
/// between the chord's middle and there. Else it is a Cartesian oval, a quartic of genus one, which
/// no rational curve parametrises: refused.
fn between_circles(t_s: f64, t_n: f64, p0: P, p2: P, circles: [(P, f64, f64, f64); 2], side: f64) -> Result<Cut, String> {
    let weigh = |&(_, _, sweep, leg): &(P, f64, f64, f64)| -side * sweep.signum() / leg;
    let (a, b) = (weigh(&circles[0]), weigh(&circles[1]));
    if (a.abs() - b.abs()).abs() > 1e-12 * a.abs().max(b.abs()) {
        return Err("a flat between two curved faces with unequal legs is a Cartesian oval, which no rational curve \
            holds".into());
    }
    let f = |q: P| circles.iter().map(|&(c, r, sweep, leg)| side * sweep.signum() * (r - dist(q, c)) / leg).sum::<f64>() - 1.0;
    let grad = |q: P| add(scale(unit(sub(q, circles[0].0)), a), scale(unit(sub(q, circles[1].0)), b));
    let (d0, d2) = (left(grad(p0)), left(grad(p2)));
    let den = cross(d0, d2);
    if den.abs() <= 1e-12 * norm(d0) * norm(d2) {
        return Err("the flat's tangents at its ends do not meet".into());
    }
    let m = add(p0, scale(d0, cross(sub(p2, p0), d2) / den));
    // the shoulder: where the conic crosses the line from the chord's middle toward `m`
    let mid = scale(add(p0, p2), 0.5);
    let v = sub(m, mid);
    let along = |u: f64| f(add(mid, scale(v, u)));
    let (f0, f1) = (along(0.0), along(1.0));
    if !(f0 * f1 < 0.0) {
        return Err("the flat's conic leaves its control triangle".into());
    }
    let u = crate::roots::bracketed_root(|u| Some(along(u)), 0.0, f0, 1.0, f1, 1e-15);
    Ok((t_s, t_n, Seg::Conic { a: p0, m, b: p2, w: u / (1.0 - u) }))
}

/// `region` with every corner where a step of `a` meets a step of `b` (either way round) cut by
/// `cut`, the new step tagged `tag`.  A seam (the steps meeting tangent) is no corner.  Refused
/// at a charted step's corner (a section's, `brep::section`: never a meridian's), and where the
/// cut does not fit on a face: its foot past the far end of a step, or two cuts meeting
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
            if !named {
                continue;
            }
            if s.chart.is_some() || n.chart.is_some() {
                return Err(format!("the corner between faces {} and {} is charted", s.tag, n.tag));
            }
            if matches!(s.seg, Seg::Conic { .. }) || matches!(n.seg, Seg::Conic { .. }) {
                return Err(format!("faces {} and {} meet at a conic's end, which takes no cut",
                    s.tag, n.tag));
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
            walk.push(Step { seg: l[i].seg.part(t0, t1), ..l[i] });
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

/// `std.Chamfered`'s flat at every corner between faces `a` and `b`: the curve its sum of distances
/// bounds (`sum`), the leg on the face of `a` `legs[0]` from the face of `b`, and on `b`'s
/// `legs[1]`.
pub fn summed(region: &Region, a: &[u32], b: &[u32], legs: [f64; 2], tag: u32)
    -> Result<Region, String> {
    corners(region, a, b, tag, &|s, n, side| {
        let legs = if a.contains(&s.tag) { legs } else { [legs[1], legs[0]] };
        sum(&s.seg, &n.seg, side, legs)
    })
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

