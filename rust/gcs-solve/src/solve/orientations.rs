//! **A stated orientation chooses the root** (§9.6, #157): `ccw(a, b, c)` / `cw(a, b, c)` as the
//! document states it (`Sketch::orientations`, apart from the plan's recorded choices in
//! `Sketch::branches`), read on the drawing as a bound is — the triangle's signed height — and,
//! where it does not hold, a point of the triangle carried across the line through the other two:
//! on the seeds (`cross_bounds`) and on the solution (`System::steer`).

#[allow(unused_imports)]
use crate::fmath::Det;
use crate::model::{orientation, EntRef, Sketch};

use super::bounds::{bound_tol, carry};

/// The three points a stated orientation names, ascending as the record keys them; `None` for a
/// key no triple of the sketch's points spells (a document's raw `branch` key, or one naming a
/// point the sketch does not have).
pub fn triangle(sk: &Sketch, key: &str) -> Option<[usize; 3]> {
    let p = crate::decompose::branch_key_points(key)?;
    p.iter().all(|&i| i < sk.points.len()).then_some(p)
}

/// How far apart points `a` and `b` stand, as the orientation reads them (`point_xy`).
fn apart(sk: &Sketch, a: usize, b: usize) -> f64 {
    let ((ax, ay), (bx, by)) = (sk.point_xy(a), sk.point_xy(b));
    (bx - ax).dhypot(by - ay)
}

/// **Whether the stated orientation `key` holds** on the drawing (§9.6): what the steering, the
/// elaborator and the diagnosis all ask.  It holds where the triangle turns the stated way (the
/// record's sign, +1 counter-clockwise in the key's order, −1 clockwise), its least height more
/// than `tol`, the solve's own (`bound_tol`): a triangle flat within it turns neither way, so a
/// stated orientation of three points the drawing puts on one line does not hold.  `None` for a
/// key the record does not state, or one naming no triangle of the sketch.
pub fn holds(sk: &Sketch, key: &str, tol: f64) -> Option<bool> {
    let sign = *sk.orientations.get(key)?;
    let [a, b, c] = triangle(sk, key)?;
    let longest = apart(sk, a, b).max(apart(sk, b, c)).max(apart(sk, c, a));
    // twice the area over the longest side: the least of the triangle's three heights
    let height = if longest > 0.0 { orientation(sk, a, b, c) / longest } else { 0.0 };
    Some(sign.signum() as f64 * height > tol)
}

/// Whether every stated orientation holds on the drawing as it stands (§9.6).
pub fn orientations_hold(sk: &Sketch) -> bool {
    broken(sk).is_empty()
}

/// Every stated orientation the drawing breaks, by key, in the record's order.
pub fn broken(sk: &Sketch) -> Vec<String> {
    let tol = bound_tol(sk);
    sk.orientations.iter()
        .filter(|(k, _)| holds(sk, k, tol) == Some(false))
        .map(|(k, _)| k.clone())
        .collect()
}

/// The height of point `q` of triangle `t` over the line through the other two, signed by the
/// way the triangle turns in the key's order: what carrying `q` across that line reverses.
/// `None` where the other two stand on one another and draw no line.
fn height_of(sk: &Sketch, t: [usize; 3], q: usize) -> Option<f64> {
    let others: Vec<usize> = t.iter().copied().filter(|&p| p != q).collect();
    let base = apart(sk, others[0], others[1]);
    (base > 0.0).then(|| orientation(sk, t[0], t[1], t[2]) / base)
}

/// The point of triangle `t` a solve carries: the last of the three in the key's order with a
/// coordinate it moves (`free`).
fn carried(sk: &Sketch, t: [usize; 3], free: &std::collections::BTreeSet<u32>) -> Option<usize> {
    let own = |p: usize| sk.own_params(EntRef::point(p)).iter().any(|i| free.contains(i));
    t.iter().rev().copied().find(|&p| own(p))
}

/// Where the point of the stated orientation `key` a solve carries is carried to (`across`),
/// nearest first, as the drawing stands: its mirror in the line through the other two, where that
/// is on the stated side by more than the solve's tolerance, then half that line's length onto
/// the stated side, or its height there where that is farther — a point standing on the line, or
/// one of a sliver whose least height is another's, has no mirror that holds.  Empty where no
/// point of it moves, or the other two stand on one another and draw no line.
fn aims(sk: &Sketch, key: &str, free: &std::collections::BTreeSet<u32>) -> Vec<f64> {
    let (Some(t), Some(&sign)) = (triangle(sk, key), sk.orientations.get(key)) else {
        return Vec::new();
    };
    let Some(q) = carried(sk, t, free) else { return Vec::new() };
    let Some(h) = height_of(sk, t, q) else { return Vec::new() };
    let (s, tol) = (sign.signum() as f64, bound_tol(sk));
    let others: Vec<usize> = t.iter().copied().filter(|&p| p != q).collect();
    let far = s * (0.5 * apart(sk, others[0], others[1])).max(h.abs());
    let mut out = Vec::new();
    if -s * h > tol {
        out.push(-h);
    }
    if out.iter().all(|&a| (a - far).abs() > tol) {
        out.push(far);
    }
    out
}

/// Every stated orientation the drawing breaks, by key, with where the point a solve carries
/// aims (`aims`), nearest first: what `System::steer` and `cross_bounds` read.
pub(super) fn broken_aims(
    sk: &Sketch,
    free: &std::collections::BTreeSet<u32>,
) -> Vec<(String, Vec<f64>)> {
    broken(sk).into_iter().map(|k| {
        let a = aims(sk, &k, free);
        (k, a)
    }).collect()
}

/// **A stated orientation chooses the side** (§9.6, #157): carry the point of the triangle `key`
/// names that a solve moves (`carried`) across the line through the other two, to where its
/// height over that line reads `aim`, its twins and lifts with it (`bounds::carry`).  The point
/// moved, or `None` where none can be.
pub(super) fn across(
    sk: &mut Sketch,
    key: &str,
    aim: f64,
    free: &std::collections::BTreeSet<u32>,
) -> Option<usize> {
    let t = triangle(sk, key)?;
    let q = carried(sk, t, free)?;
    carry(sk, &[EntRef::point(q)], &|sk: &Sketch, q| height_of(sk, t, q), false, aim, free)
}
