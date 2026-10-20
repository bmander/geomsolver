//! A body turned whole: the selection rotated rigidly, in the plane it is drawn in or in space.
//!
//! A point drag pulls one point toward the pointer softly and polishes; a turn has no one point to
//! pull, and a plane's attitude has no soft row to pull it by.  So the turn is put **exactly**:
//! every free number the body carries is set where the rigid turn of its start puts it, and the
//! hard rows are then polished on their own, as a drag's are — a body nothing else holds turns
//! as it is told, one tied to the rest of the drawing compromises with it.  Each move turns the
//! body on from the start of the drag (never from where the last polish left it), so nothing
//! drifts, and a large turn is taken in increments so the polish tracks its branch.

use std::collections::BTreeSet;

use crate::curve;
use crate::fmath::Det;
use crate::model::{EntKind, EntRef, Sketch};
use crate::newton::Method;
use crate::space::{add, dot, scale, sub};
use crate::system::System;

use super::{BlockMode, SolveOpts, SolveResult};

/// The largest turn polished in one step (radians): a sweep past it is taken in increments.
const TURN_STEP: f64 = 0.1;
const POLISH_ITER: i32 = 20;

const POLISH: SolveOpts = SolveOpts {
    method: Method::DogLeg,
    tol: 1e-14,
    acceptance_tol: 1e-6,
    max_nfev: 0,
    writeback: true,
    max_iter: POLISH_ITER,
    dense: None,
    rehome: false,
    retry: false,
    blocks: BlockMode::Rescue,
};

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
    polish: System,
    pub turning: Turning,
    /// The points carried: in the plane, drawn in it; in space, points in space.
    points: Vec<usize>,
    /// The planes and frames carried, each turned about the pivot with its axes.
    frames: Vec<usize>,
    /// Every axis carried: those in the body and each carried frame's own.
    axes: Vec<usize>,
    /// What it turns about: in the plane's coordinates (a third of nought), or in space.
    pub pivot: [f64; 3],
    /// Every number as the drag took hold of it, which each move turns on from.
    start: Vec<f64>,
    /// How far it has turned: the angle in the plane, the rotation in space.
    angle: f64,
    rot: [[f64; 3]; 3],
}

const IDENTITY: [[f64; 3]; 3] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

fn apply(r: &[[f64; 3]; 3], p: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|i| dot(r[i], p))
}

fn compose(a: &[[f64; 3]; 3], b: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
    std::array::from_fn(|i| std::array::from_fn(|j| (0..3).map(|k| a[i][k] * b[k][j]).sum()))
}

/// The turn by `|w|` radians about the unit `w / |w|` (Rodrigues).
fn rotation(w: [f64; 3]) -> [[f64; 3]; 3] {
    let t = dot(w, w).sqrt();
    if t == 0.0 {
        return IDENTITY;
    }
    let k = scale(w, 1.0 / t);
    let (s, c) = t.dsin_cos();
    let kx = [[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]];
    std::array::from_fn(|i| std::array::from_fn(|j| {
        let id = if i == j { 1.0 } else { 0.0 };
        id + s * kx[i][j] + (1.0 - c) * (0..3).map(|m| kx[i][m] * kx[m][j]).sum::<f64>()
    }))
}

impl TurnDrag {
    /// Take hold of `body` to turn it.  What it carries is read off the entities: a figure's
    /// points, a plane or frame with its axes and place, an axis.  It turns in space when anything
    /// in it stands there, else in the plane its first entity is drawn in (the one taken hold
    /// of), carrying only the points drawn there.  The pivot is the middle of what is held where the body holds anything
    /// (a line with one end held turns about that end), else the middle of all it carries — a
    /// lone plane's or frame's own origin.
    pub fn new(sk: &mut Sketch, body: &[EntRef]) -> TurnDrag {
        let (mut points, mut frames, mut axes) = (BTreeSet::new(), BTreeSet::new(), BTreeSet::new());
        // first things first: the first point met is the first entity's, whose plane it turns in
        let mut todo: Vec<EntRef> = body.iter().rev().copied().collect();
        let mut seen = BTreeSet::new();
        let mut first = None;
        while let Some(e) = todo.pop() {
            if !seen.insert(e) {
                continue;
            }
            match e.kind {
                EntKind::Point => {
                    points.insert(e.i());
                    first = first.or(Some(e.i()));
                }
                EntKind::Axis => { axes.insert(e.i()); }
                EntKind::Frame if sk.frames[e.i()].chart.is_some() => { frames.insert(e.i()); }
                EntKind::Line | EntKind::Circle | EntKind::Arc | EntKind::Spline => {
                    todo.extend(sk.children(e).into_iter().rev())
                }
                _ => {}
            }
        }
        let in_space = !frames.is_empty() || !axes.is_empty()
            || points.iter().any(|&p| sk.points[p].z.is_some());
        let turning = if in_space {
            Turning::InSpace
        } else {
            Turning::InPlane(first.and_then(|p: usize| sk.points[p].plane.map(|f| f as usize)))
        };
        let points: Vec<usize> = points.into_iter().filter(|&p| {
            let pt = &sk.points[p];
            match turning {
                // a point drawn in a spatial frame rides on it
                Turning::InSpace => pt.z.is_some() && pt.chart.is_none(),
                Turning::InPlane(pl) => pt.z.is_none() && pt.plane.map(|f| f as usize) == pl,
            }
        }).collect();
        for &f in &frames {
            let c = sk.frames[f].chart.as_ref().expect("charted");
            axes.extend([c.u, c.v].into_iter().chain(c.normal).map(|a| a as usize));
        }
        let frames: Vec<usize> = frames.into_iter().collect();
        let axes: Vec<usize> = axes.into_iter().collect();
        let start = sk.get_x();
        let value = |q: u32| start[q as usize];
        let held = |qs: &[u32]| qs.iter().all(|&q| sk.params[q as usize].fixed);
        // each place the body has, and whether it is held there
        let mut places: Vec<([f64; 3], bool)> = Vec::new();
        for &p in &points {
            let pt = &sk.points[p];
            let qs: Vec<u32> = [pt.x, pt.y].into_iter().chain(pt.z).collect();
            let at = [value(pt.x), value(pt.y), pt.z.map_or(0.0, value)];
            places.push((at, held(&qs)));
        }
        for &f in &frames {
            let o = sk.frames[f].chart.as_ref().expect("charted").o;
            places.push((o.map(value), held(&o)));
        }
        for &a in &axes {
            let ax = &sk.axes[a];
            if ax.placed && !frames.iter().any(|&f| owns_axis(sk, f, a)) {
                places.push((ax.a.map(value), held(&ax.a)));
            }
        }
        let middle = |of: &mut dyn Iterator<Item = [f64; 3]>| {
            let (mut sum, mut n) = ([0.0; 3], 0.0);
            for p in of {
                sum = add(sum, p);
                n += 1.0;
            }
            (n > 0.0).then(|| scale(sum, 1.0 / n))
        };
        let pivot = middle(&mut places.iter().filter(|p| p.1).map(|p| p.0))
            .or_else(|| middle(&mut places.iter().map(|p| p.0)))
            .unwrap_or([0.0; 3]);
        let polish = System::new(sk);
        TurnDrag {
            polish, turning, points, frames, axes, pivot, start, angle: 0.0, rot: IDENTITY,
        }
    }

    /// How many free numbers the turn carries: none, and it can turn nothing.
    pub fn carries(&self, sk: &Sketch) -> usize {
        let free = |q: &u32| !sk.params[*q as usize].fixed;
        let pts: usize = self.points.iter().map(|&p| {
            let pt = &sk.points[p];
            [pt.x, pt.y].iter().chain(pt.z.as_ref()).filter(|q| free(q)).count()
        }).sum();
        let frames: usize = self.frames.iter()
            .map(|&f| sk.frames[f].chart.as_ref().map_or(0, |c| c.o.iter().filter(|q| free(q)).count()))
            .sum();
        let axes: usize = self.axes.iter()
            .map(|&a| sk.axes[a].d.iter().chain(&sk.axes[a].a).filter(|q| free(q)).count())
            .sum();
        pts + frames + axes
    }

    /// Turned on in its plane by the angle the pointer sweeps about the pivot from `from` to `to`,
    /// both in the plane's coordinates.  In space, nothing.
    pub fn sweep(&mut self, sk: &mut Sketch, from: (f64, f64), to: (f64, f64)) -> SolveResult {
        let c = self.pivot;
        let a = (from.1 - c[1]).datan2(from.0 - c[0]);
        let b = (to.1 - c[1]).datan2(to.0 - c[0]);
        let mut d = b - a;
        // the short way round: a sweep is a small step of the pointer, never most of a turn
        if d > std::f64::consts::PI {
            d -= std::f64::consts::TAU;
        } else if d < -std::f64::consts::PI {
            d += std::f64::consts::TAU;
        }
        self.turn_in_plane(sk, d)
    }

    /// Turned on in its plane by `d` radians, counter-clockwise.
    pub fn turn_in_plane(&mut self, sk: &mut Sketch, d: f64) -> SolveResult {
        let n = ((d.abs() / TURN_STEP).ceil() as usize).max(1);
        for _ in 1..n {
            self.angle += d / n as f64;
            self.put(sk);
        }
        self.angle += d / n as f64;
        self.put(sk)
    }

    /// Turned on in space as the pointer pushes it, seen by the eye at bearing `az` and elevation
    /// `el` (radians): `across` radians about the eye's up (a drag to the right swings the near
    /// side right) and `down` about its right (a drag down swings the near side down) — as if
    /// the body were a ball held at its pivot.
    pub fn turn_seen(&mut self, sk: &mut Sketch, (az, el): (f64, f64), across: f64, down: f64)
        -> SolveResult {
        let (right, up) = crate::space::eye(az, el);
        let w = add(scale(up, across), scale(right, down));
        let t = dot(w, w).sqrt();
        let n = ((t / TURN_STEP).ceil() as usize).max(1);
        let step = rotation(scale(w, 1.0 / n as f64));
        for _ in 1..n {
            self.rot = compose(&step, &self.rot);
            self.put(sk);
        }
        self.rot = compose(&step, &self.rot);
        self.put(sk)
    }

    /// Every free number carried set where the turn puts its start, then the hard rows polished.
    fn put(&mut self, sk: &mut Sketch) -> SolveResult {
        let start = &self.start;
        let s = |q: u32| start[q as usize];
        let set = |sk: &mut Sketch, q: u32, v: f64| {
            if !sk.params[q as usize].fixed {
                sk.params[q as usize].value = v;
            }
        };
        let c = self.pivot;
        match self.turning {
            Turning::InPlane(_) => {
                let (sn, cs) = self.angle.dsin_cos();
                for &p in &self.points {
                    let pt = sk.points[p].clone();
                    let (dx, dy) = (s(pt.x) - c[0], s(pt.y) - c[1]);
                    set(sk, pt.x, c[0] + cs * dx - sn * dy);
                    set(sk, pt.y, c[1] + sn * dx + cs * dy);
                }
            }
            Turning::InSpace => {
                let r = self.rot;
                let turned = |p: [f64; 3]| add(c, apply(&r, sub(p, c)));
                for &p in &self.points {
                    let pt = sk.points[p].clone();
                    let Some(z) = pt.z else { continue };
                    let at = turned([s(pt.x), s(pt.y), s(z)]);
                    for (q, v) in [pt.x, pt.y, z].into_iter().zip(at) {
                        set(sk, q, v);
                    }
                }
                for &f in &self.frames {
                    let o = sk.frames[f].chart.as_ref().expect("charted").o;
                    for (q, v) in o.into_iter().zip(turned(o.map(s))) {
                        set(sk, q, v);
                    }
                }
                for &a in &self.axes {
                    let ax = sk.axes[a].clone();
                    let d = apply(&r, ax.d.map(s));
                    for (q, v) in ax.d.into_iter().zip(d) {
                        set(sk, q, v);
                    }
                    if ax.placed {
                        // the line turned whole: the point on it nearest the world's origin, again
                        let on = turned(ax.a.map(s));
                        let foot = sub(on, scale(d, dot(on, d)));
                        for (q, v) in ax.a.into_iter().zip(foot) {
                            set(sk, q, v);
                        }
                    }
                }
            }
        }
        let res = self.polish.solve(sk, POLISH);
        // a contact carried past the end of its span is put back on the curve, and the polish
        // compiled again over the span it is on now
        if !self.polish.spans().is_empty() {
            curve::clamp_contacts(sk);
            if curve::contact_spans(sk) != *self.polish.spans() {
                self.polish = System::new(sk);
            }
        }
        res
    }

    /// The rotation in space the drag has turned through so far (the identity in a plane).
    pub fn rotation(&self) -> [[f64; 3]; 3] {
        self.rot
    }

    /// The angle in its plane the drag has turned through so far (nought in space).
    pub fn angle(&self) -> f64 {
        self.angle
    }
}

/// Whether axis `a` is one of frame `f`'s own: it turns with the frame about the frame's place.
fn owns_axis(sk: &Sketch, f: usize, a: usize) -> bool {
    sk.frames[f].chart.as_ref()
        .is_some_and(|c| [c.u, c.v].into_iter().chain(c.normal).any(|x| x as usize == a))
}
