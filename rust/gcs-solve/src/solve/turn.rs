//! A body turned whole: the selection rotated rigidly, in the plane it is drawn in or in space.
//!
//! A point drag pulls one point toward the pointer softly and polishes; a turn has no one point to
//! pull, and a plane's attitude has no soft row to pull it by.  So the turn is put **exactly**:
//! every free number the body carries is set where the rigid turn of its start puts it, and the
//! hard rows are then polished on their own, as a drag's are — a body nothing else holds turns
//! as it is told, one tied to the rest of the drawing compromises with it.  Each move turns the
//! body on from the start of the drag (never from where the last polish left it), so nothing
//! drifts, and a large turn is taken in increments so the polish tracks its branch.  Like a
//! drag's numeric fallback it works on the part the body can move (`graft::Part`), written back
//! once a move.

use std::collections::BTreeSet;

use crate::brep::geom::Rigid;
use crate::graft::Part;
use crate::model::{EntKind, EntRef, Sketch};
use crate::space::{add, dot, norm, scale, sub};
use crate::system::System;

use super::drag::{NO_REHOME, POLISH_ITER};
use super::{SolveOpts, SolveResult};

/// The largest turn polished in one step (radians): a sweep past it is taken in increments.
const TURN_STEP: f64 = 0.1;

/// A drag's polish, re-homing its own contacts: a turn has one system, and no soft row for a
/// rebuild to pick up.
const POLISH: SolveOpts = SolveOpts { rehome: true, max_iter: POLISH_ITER, ..NO_REHOME };

/// Where a body turns: in the plane its points are drawn in (`None` a 2D sketch's page), or in
/// space — any plane, frame, axis or point in space in it turns it there.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Turning {
    InPlane(Option<usize>),
    InSpace,
}

/// **A body turned whole** (a shift-drag in the page): the points, planes and axes of `body`
/// rotated rigidly about its pivot, then polished.
pub struct TurnDrag {
    part: Part,
    polish: System,
    /// Where it turns, the plane named as the document numbers it.
    pub turning: Turning,
    /// What it turns about: in the plane's coordinates (a third of nought), or in space.
    pub pivot: [f64; 3],
    /// How many free numbers it carries: none, and it can turn nothing.
    pub carries: usize,
    /// The part's points carried: in the plane, drawn in it; in space, points in space.
    points: Vec<usize>,
    /// Where each carried plane or frame of the part stands, turned about the pivot.
    origins: Vec<[u32; 3]>,
    /// Every axis of the part carried: those in the body and each carried frame's own.
    axes: Vec<usize>,
    /// The part's numbers as the drag took hold of them, which each move turns on from.
    start: Vec<f64>,
    /// How far it has turned so far.
    turned: Rigid,
}

/// The middle of `places`, if there are any.
fn middle(places: impl Iterator<Item = [f64; 3]>) -> Option<[f64; 3]> {
    let (mut sum, mut n) = ([0.0; 3], 0.0);
    for p in places {
        sum = add(sum, p);
        n += 1.0;
    }
    (n > 0.0).then(|| scale(sum, 1.0 / n))
}

impl TurnDrag {
    /// Take hold of `body` to turn it.  What it carries is read off the entities: a figure's
    /// points, a plane or frame with its axes and place, an axis.  It turns in space when anything
    /// in it stands there, else in the plane its first entity is drawn in (the one taken hold
    /// of), carrying only the points drawn there.  The pivot is the middle of what is held where
    /// the body holds anything (a line with one end held turns about that end), else the middle
    /// of all it carries — a lone plane's or frame's own origin.
    pub fn new(sk: &Sketch, body: &[EntRef]) -> TurnDrag {
        let part = Part::around_all(sk, body);
        let body: Vec<EntRef> = body.iter().filter_map(|&e| part.entity_in(e)).collect();
        let ps = &part.sketch;
        let first = body.first().and_then(|&e| first_point(ps, e));
        let (mut points, mut frames, mut axes) = (BTreeSet::new(), BTreeSet::new(), BTreeSet::new());
        let mut todo = body;
        let mut seen = BTreeSet::new();
        while let Some(e) = todo.pop() {
            if !seen.insert(e) {
                continue;
            }
            match e.kind {
                EntKind::Point => { points.insert(e.i()); }
                EntKind::Axis => { axes.insert(e.i()); }
                EntKind::Frame if ps.frames[e.i()].chart.is_some() => { frames.insert(e.i()); }
                k if k.bears_points() => todo.extend(ps.children(e)),
                _ => {}
            }
        }
        let in_space = !frames.is_empty() || !axes.is_empty()
            || points.iter().any(|&p| ps.points[p].z.is_some());
        let plane = first.and_then(|p| ps.plane_of(p));
        let turning = if in_space {
            Turning::InSpace
        } else {
            // the plane named as the document numbers it, which the part's need not
            Turning::InPlane(first.and_then(|p| sk.plane_of(part.point_out(p))))
        };
        let points: Vec<usize> = points.into_iter().filter(|&p| {
            let pt = &ps.points[p];
            if in_space {
                // a point drawn in a spatial frame rides on it
                pt.z.is_some() && pt.chart.is_none()
            } else {
                pt.z.is_none() && ps.plane_of(p) == plane
            }
        }).collect();
        // each place the body has, and whether it is held there
        let start = ps.get_x();
        let value = |q: u32| start[q as usize];
        let held = |qs: &[u32]| qs.iter().all(|&q| ps.params[q as usize].fixed);
        let mut places: Vec<([f64; 3], bool)> = points.iter().map(|&p| {
            let pt = &ps.points[p];
            ([value(pt.x), value(pt.y), pt.z.map_or(0.0, value)], ps.point_held(p))
        }).collect();
        let mut origins = Vec::new();
        let mut owned = BTreeSet::new();
        for &f in &frames {
            let c = ps.frames[f].chart.as_ref().expect("charted");
            places.push((c.o.map(value), held(&c.o)));
            origins.push(c.o);
            owned.extend([c.u, c.v].into_iter().chain(c.normal).map(|a| a as usize));
        }
        for &a in axes.difference(&owned) {
            let ax = &ps.axes[a];
            if ax.placed {
                places.push((ax.a.map(value), held(&ax.a)));
            }
        }
        axes.extend(owned);
        let pivot = middle(places.iter().filter(|p| p.1).map(|p| p.0))
            .or_else(|| middle(places.iter().map(|p| p.0)))
            .unwrap_or([0.0; 3]);
        let axes: Vec<usize> = axes.into_iter().collect();
        let free = |q: &u32| !ps.params[*q as usize].fixed;
        let carries = points.iter().flat_map(|&p| ps.point_all_params(p)).filter(free).count()
            + origins.iter().flatten().filter(|q| free(q)).count()
            + axes.iter().flat_map(|&a| ps.axes[a].d.iter().chain(&ps.axes[a].a))
                .filter(|q| free(q)).count();
        let polish = System::new(ps);
        TurnDrag {
            part, polish, turning, pivot, carries, points, origins, axes, start,
            turned: Rigid::identity(),
        }
    }

    /// Turned on in its plane by the angle the pointer sweeps about the pivot from `from` to `to`,
    /// both in the plane's coordinates — the short way round, a sweep being a small step of the
    /// pointer and never most of a turn.
    pub fn sweep(&mut self, sk: &mut Sketch, from: (f64, f64), to: (f64, f64)) -> SolveResult {
        use crate::fmath::Det;
        use std::f64::consts::{PI, TAU};
        let c = self.pivot;
        let a = (from.1 - c[1]).datan2(from.0 - c[0]);
        let b = (to.1 - c[1]).datan2(to.0 - c[0]);
        self.turn_in_plane(sk, (b - a + PI).rem_euclid(TAU) - PI)
    }

    /// Turned on in its plane by `d` radians, counter-clockwise.
    pub fn turn_in_plane(&mut self, sk: &mut Sketch, d: f64) -> SolveResult {
        self.turn_by(sk, [0.0, 0.0, 1.0], d)
    }

    /// Turned on in space as the pointer pushes it, seen by the eye at bearing `az` and elevation
    /// `el` (radians): `across` radians about the eye's up (a drag to the right swings the near
    /// side right) and `down` about its right (a drag down swings the near side down) — as if
    /// the body were a ball held at its pivot.
    pub fn turn_seen(&mut self, sk: &mut Sketch, (az, el): (f64, f64), across: f64, down: f64)
        -> SolveResult {
        let (right, up) = crate::space::eye(az, el);
        let w = add(scale(up, across), scale(right, down));
        let t = norm(w);
        self.turn_by(sk, if t > 0.0 { w } else { up }, t)
    }

    /// Turned on by `angle` about `axis` through the pivot, in steps of at most `TURN_STEP`, and
    /// what the part came to written back.
    fn turn_by(&mut self, sk: &mut Sketch, axis: [f64; 3], angle: f64) -> SolveResult {
        let n = ((angle.abs() / TURN_STEP).ceil() as usize).max(1);
        let step = Rigid::turn(self.pivot, axis, angle / n as f64);
        let res = (0..n).map(|_| {
            self.turned = self.turned.then(&step);
            self.put()
        }).last().expect("one step at least");
        self.part.write_back(sk);
        res
    }

    /// Every free number carried set where the turn puts its start, then the hard rows polished.
    fn put(&mut self) -> SolveResult {
        let (start, r) = (&self.start, &self.turned);
        let ps = &mut self.part.sketch;
        let s = |q: u32| start[q as usize];
        let set = |ps: &mut Sketch, qs: &[u32], v: [f64; 3]| {
            for (&q, v) in qs.iter().zip(v) {
                if !ps.params[q as usize].fixed {
                    ps.params[q as usize].value = v;
                }
            }
        };
        for &p in &self.points {
            let qs = ps.point_all_params(p);
            let at = r.point([s(qs[0]), s(qs[1]), qs.get(2).map_or(0.0, |&z| s(z))]);
            set(ps, &qs, at);
        }
        for o in &self.origins {
            set(ps, o, r.point(o.map(s)));
        }
        for &a in &self.axes {
            let (d0, a0, placed) = { let ax = &ps.axes[a]; (ax.d, ax.a, ax.placed) };
            let d = r.vector(d0.map(s));
            set(ps, &d0, d);
            if placed {
                // the line turned whole: the point on it nearest the world's origin, again
                let on = r.point(a0.map(s));
                set(ps, &a0, sub(on, scale(d, dot(on, d))));
            }
        }
        self.polish.solve(ps, POLISH)
    }
}

/// The first point `e` is made of, in the order it names them, if any.
fn first_point(sk: &Sketch, e: EntRef) -> Option<usize> {
    match e.kind {
        EntKind::Point => Some(e.i()),
        k if k.bears_points() => sk.children(e).into_iter().find_map(|c| first_point(sk, c)),
        _ => None,
    }
}
