//! **A frame the drawing carries as it is followed** (#398 F): a frame with no joint, placed by
//! relations on geometry that moves as a declared parameter runs (`c := std.Turned(b, k)` on a
//! four-bar's coupler), stands wherever the march along the parameter (`follow.rs`) puts it, so
//! its pose is a function of the parameter — a motion like any other, read after the solve.
//!
//! **Read off the march** (`Track::read`): the frame's origin and axes at every station, and at
//! enough instants between them (each stood on the branch carried, `Followed::put`) that no two
//! nodes are further apart than `NODES` of the interval; between nodes, a cubic Hermite in the
//! parameter — the origin's coordinates, and the frame's turn as a unit quaternion (its four
//! coordinates fitted and the result normalised, so every pose is rigid) — its rate at each node
//! the derivative there of the quartic through the five nearest nodes.  The fit is fourth order
//! in the node spacing for the pose and third for its rate: on the gate's four-bar the pose stands
//! within 1e-10 mm of the closed form (the march's own solve) and its rate within 1e-7.
//!
//! The motion is `M(u) x = R(u) (x − o_h) + o(u)`, `R(u) = F(u) F(h)ᵀ`, its **home** `h` the
//! instant the drawing stands at (the interval's start, or where it is stood), so what is drawn
//! where it stands is carried from there.  It reads the roll in the kernels' units (radians for an
//! angle), as every motion does, and reaches as far as the march did: past a singular instant it
//! has no pose.  Its enclosures over a stretch (`Track::stretch`, the same as a frame along a
//! path's) are certain of the fitted motion: each cubic's derivative lies in the hull of its
//! Bernstein coefficients, so its speed and its turn's rate are bounded for certain over every
//! segment.
use std::rc::Rc;

use super::guide::Stretch;
use crate::follow::Followed;
use crate::interval::Interval as I;
use crate::model::Sketch;
use crate::space::{norm, sub};

type V = [f64; 3];
type Q = [f64; 4];
/// A frame as three unit columns: `u`, `v` and the normal `u × v`.
type Frame = [V; 3];

/// The widest two nodes stand apart, as a fraction of the interval.
const NODES: f64 = 1.0 / 1024.0;

/// How many samples a stretch is enclosed from.
const SAMPLES: usize = 64;

/// One node: the parameter (in the kernels' units), the frame's origin and its turn, and their
/// rates a unit of the parameter.
#[derive(Clone, Copy, Debug)]
struct Node {
    u: f64,
    o: V,
    q: Q,
    d_o: V,
    d_q: Q,
}

/// **The frame's poses along a march** (`MotionDef::Followed`): its nodes, from the interval's
/// start to as far as the march reached.  Read once per march (`Caches::tracks`).
#[derive(Clone, Debug)]
pub(crate) struct Track {
    name: String,
    over: crate::model::Running,
    units: crate::units::Units,
    nodes: Vec<Node>,
    /// Over each segment, `|o′|` and `|q′|` bounded, and `|q|` bounded below.
    certain: Vec<[f64; 3]>,
    /// Whether no node of the march moves the frame: the world, seen from it.
    still: bool,
}

/// A track and the home it is read from (`Family::read`): the frame where the drawing stands —
/// or, where it carries nothing (no march, or one that leaves the frame still), why: seen from,
/// it is then the world, as a frame with no joint is, and read itself it is refused.
#[derive(Clone, Debug)]
pub(crate) struct Carried {
    read: Result<(std::sync::Arc<Track>, V, Frame), String>,
}

/// Why a relation the solve holds cannot read the frame `f` the drawing carries along `over`: a
/// row reads no march, so the frame is read after the solve, by a reading over the parameter.  The
/// one wording, wherever the row is refused.
pub fn unread_march(f: &str, over: &str) -> String {
    format!("`{f}` is carried by the drawing as it is followed along `{over}`, and a relation the \
             solve holds reads no march (#398): read it over `{over}` after the solve — a sweep, \
             `sweep(tool over {over}, under: {f})`")
}

/// The frame `f`'s axes and origin where `sk` stands, or `None` where it has none.
fn frame_of(sk: &Sketch, f: usize) -> Option<(V, Frame)> {
    sk.frames.get(f)?.chart.as_ref()?;
    let b = sk.basis(f);
    Some((b.o, [b.u, b.v, b.normal()]))
}

/// The turn taking the world's axes to frame `f`'s columns, as a unit quaternion `(w, x, y, z)`
/// (Shepperd's: from the largest of its four squares).
fn quaternion(f: &Frame) -> Q {
    let m = |i: usize, j: usize| f[j][i];
    let tr = m(0, 0) + m(1, 1) + m(2, 2);
    let q = if tr > 0.0 {
        let s = 2.0 * (1.0 + tr).sqrt();
        [0.25 * s, (m(2, 1) - m(1, 2)) / s, (m(0, 2) - m(2, 0)) / s, (m(1, 0) - m(0, 1)) / s]
    } else if m(0, 0) >= m(1, 1) && m(0, 0) >= m(2, 2) {
        let s = 2.0 * (1.0 + m(0, 0) - m(1, 1) - m(2, 2)).sqrt();
        [(m(2, 1) - m(1, 2)) / s, 0.25 * s, (m(0, 1) + m(1, 0)) / s, (m(0, 2) + m(2, 0)) / s]
    } else if m(1, 1) >= m(2, 2) {
        let s = 2.0 * (1.0 + m(1, 1) - m(0, 0) - m(2, 2)).sqrt();
        [(m(0, 2) - m(2, 0)) / s, (m(0, 1) + m(1, 0)) / s, 0.25 * s, (m(1, 2) + m(2, 1)) / s]
    } else {
        let s = 2.0 * (1.0 + m(2, 2) - m(0, 0) - m(1, 1)).sqrt();
        [(m(1, 0) - m(0, 1)) / s, (m(0, 2) + m(2, 0)) / s, (m(1, 2) + m(2, 1)) / s, 0.25 * s]
    };
    let n = q.iter().map(|x| x * x).sum::<f64>().sqrt();
    q.map(|x| x / n)
}

/// The frame a unit quaternion turns the world's axes to.
fn frame(q: Q) -> Frame {
    let [w, x, y, z] = q;
    [[1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y + w * z), 2.0 * (x * z - w * y)],
     [2.0 * (x * y - w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z + w * x)],
     [2.0 * (x * z + w * y), 2.0 * (y * z - w * x), 1.0 - 2.0 * (x * x + y * y)]]
}

/// The cubic Hermite through `(y0, m0)` and `(y1, m1)` over a segment `h` wide, at `s` of it
/// (0 to 1): its value and its rate a unit of the parameter.
fn hermite(y0: f64, m0: f64, y1: f64, m1: f64, h: f64, s: f64) -> (f64, f64) {
    let (s2, s3) = (s * s, s * s * s);
    let v = (2.0 * s3 - 3.0 * s2 + 1.0) * y0 + (s3 - 2.0 * s2 + s) * h * m0
        + (-2.0 * s3 + 3.0 * s2) * y1 + (s3 - s2) * h * m1;
    let d = ((6.0 * s2 - 6.0 * s) * y0 + (3.0 * s2 - 4.0 * s + 1.0) * h * m0
        + (-6.0 * s2 + 6.0 * s) * y1 + (3.0 * s2 - 2.0 * s) * h * m1) / h;
    (v, d)
}

/// The Bernstein coefficients of the hermite's rate over the segment: its rate lies in their
/// hull everywhere along it.
fn rate_hull(y0: f64, m0: f64, y1: f64, m1: f64, h: f64) -> [f64; 3] {
    let (p1, p2) = (y0 + h * m0 / 3.0, y1 - h * m1 / 3.0);
    [3.0 * (p1 - y0) / h, 3.0 * (p2 - p1) / h, 3.0 * (y1 - p2) / h]
}

/// The derivative at `xs[k]` of the polynomial through `(xs, ys)` (Lagrange's).
fn slope(xs: &[f64], ys: &[f64], k: usize) -> f64 {
    let x = xs[k];
    let mut d = 0.0;
    for j in 0..xs.len() {
        let w = if j == k {
            (0..xs.len()).filter(|&m| m != k).map(|m| 1.0 / (x - xs[m])).sum::<f64>()
        } else {
            let num: f64 = (0..xs.len()).filter(|&m| m != j && m != k).map(|m| x - xs[m])
                .product();
            let den: f64 = (0..xs.len()).filter(|&m| m != j).map(|m| xs[j] - xs[m]).product();
            num / den
        };
        d += w * ys[j];
    }
    d
}

impl Track {
    /// The frame `f` (named `name`) along the march `march` of the drawing `sk`: its pose at
    /// every node — or why it is no motion: the frame has no axes.
    pub(crate) fn read(sk: &Sketch, name: &str, f: usize, march: &Followed)
        -> Result<Track, String> {
        let over = &march.over;
        if frame_of(sk, f).is_none() {
            return Err(format!("`{name}` has no axes to be carried by: a frame the drawing \
                                carries is placed by its origin and axes (`std.Turned`)"));
        }
        // the march's stations, and between them the instants no wider apart than `NODES`
        let widest = (over.to - over.from) * NODES;
        let mut at = Vec::new();
        for w in march.stations.windows(2) {
            let n = ((w[1].at - w[0].at) / widest).ceil().max(1.0) as usize;
            at.extend((0..n).map(|j| (w[0].at, w[1].at, j, n)));
        }
        let mut sketch = sk.clone();
        let mut poses: Vec<(f64, V, Frame)> = Vec::new();
        let mut stand = |sketch: &mut Sketch, t: f64| -> Result<(), String> {
            let (o, fr) = frame_of(sketch, f).ok_or("a frame lost its axes")?;
            let u = if over.turns() { t.to_radians() } else { t };
            poses.push((u, o, fr));
            Ok(())
        };
        let first = march.stations.first().ok_or("a march with no station")?;
        sketch.set_x(&first.x);
        stand(&mut sketch, first.at)?;
        for (a, b, j, n) in at {
            let t = if j + 1 == n { b } else { a + (b - a) * (j + 1) as f64 / n as f64 };
            match march.stations.iter().find(|s| s.at == t) {
                Some(s) => {
                    sketch.set_x(&s.x);
                }
                None => march.put(&mut sketch, t)
                    .map_err(|e| format!("`{name}` at {}: {e}", crate::vocab::num(t)))?,
            }
            stand(&mut sketch, t)?;
        }
        if poses.len() < 2 {
            return Err(format!("`{name}` is carried along `{}` over no interval", over.name));
        }
        // the turns, each the sign nearer the one before
        let mut qs: Vec<Q> = Vec::with_capacity(poses.len());
        for (_, _, fr) in &poses {
            let mut q = quaternion(fr);
            if let Some(p) = qs.last() {
                if (0..4).map(|k| p[k] * q[k]).sum::<f64>() < 0.0 {
                    q = q.map(|x| -x);
                }
            }
            qs.push(q);
        }
        let reach = poses.iter().map(|p| norm(p.1)).fold(0.0, f64::max);
        let still = poses.iter().zip(&qs).all(|((_, o, _), q)| {
            norm(sub(*o, poses[0].1)) <= 1e-9 * (1.0 + reach)
                && (0..4).all(|k| (q[k] - qs[0][k]).abs() <= 1e-12)
        });
        // each node's rates, off the quartic through the five nearest
        let xs: Vec<f64> = poses.iter().map(|p| p.0).collect();
        let n = xs.len();
        let nodes: Vec<Node> = (0..n).map(|k| {
            let lo = k.saturating_sub(2).min(n.saturating_sub(5));
            let hi = (lo + 5).min(n);
            let x = &xs[lo..hi];
            let d_o = std::array::from_fn(|c| {
                let ys: Vec<f64> = poses[lo..hi].iter().map(|p| p.1[c]).collect();
                slope(x, &ys, k - lo)
            });
            let d_q = std::array::from_fn(|c| {
                let ys: Vec<f64> = qs[lo..hi].iter().map(|q| q[c]).collect();
                slope(x, &ys, k - lo)
            });
            Node { u: xs[k], o: poses[k].1, q: qs[k], d_o, d_q }
        }).collect();
        // over each segment, the rates' hulls: the speed, the turn's rate, and the least the
        // fitted quaternion's length comes to before it is normalised
        let certain = nodes.windows(2).map(|w| {
            let (a, b) = (w[0], w[1]);
            let h = b.u - a.u;
            let hull = |y0: &[f64], m0: &[f64], y1: &[f64], m1: &[f64]| -> f64 {
                (0..3).map(|j| (0..y0.len()).map(|c| rate_hull(y0[c], m0[c], y1[c], m1[c], h)[j])
                    .map(|x| x * x).sum::<f64>().sqrt()).fold(0.0, f64::max)
            };
            let speed = hull(&a.o, &a.d_o, &b.o, &b.d_o);
            let turning = hull(&a.q, &a.d_q, &b.q, &b.d_q);
            // within `h · |q′|` of the unit `q` it starts at
            [speed, turning, 1.0 - h * turning]
        }).collect();
        Ok(Track { name: name.to_string(), over: over.clone(), units: sk.units, nodes, certain,
                   still })
    }

    /// The interval the track runs over, in the kernels' units.
    fn domain(&self) -> (f64, f64) {
        (self.nodes[0].u, self.nodes[self.nodes.len() - 1].u)
    }

    /// The segment holding `u` and how far along it, or why `u` is past the march.
    fn segment(&self, u: f64) -> Result<(usize, f64), String> {
        let (lo, hi) = self.domain();
        let slack = 1e-12 * (hi - lo).max(1.0);
        if !(u >= lo - slack && u <= hi + slack) {
            // in the units the parameter is read in, degrees for an angle
            let read = |x: f64| crate::follow::reading(&self.over,
                if self.over.turns() { x.to_degrees() } else { x }, self.units);
            let stop = if hi < self.over.radians()[1] { ", where its march stops" } else { "" };
            return Err(format!("`{}` is carried as `{}` runs from {} to {}{stop}, and {} is past \
                                it", self.name, self.over.name, read(lo), read(hi), read(u)));
        }
        let u = u.clamp(lo, hi);
        let k = self.nodes.partition_point(|n| n.u <= u).clamp(1, self.nodes.len() - 1) - 1;
        let (a, b) = (self.nodes[k].u, self.nodes[k + 1].u);
        Ok((k, (u - a) / (b - a)))
    }

    /// The frame's origin and turn at `u`, and their rates: the turn's normalised, its rate the
    /// angular velocity.
    fn at(&self, u: f64) -> Result<(V, V, Frame, V), String> {
        let (k, s) = self.segment(u)?;
        let (a, b) = (self.nodes[k], self.nodes[k + 1]);
        let h = b.u - a.u;
        let mut o = [0.0; 3];
        let mut d_o = [0.0; 3];
        for c in 0..3 {
            (o[c], d_o[c]) = hermite(a.o[c], a.d_o[c], b.o[c], b.d_o[c], h, s);
        }
        let mut q = [0.0; 4];
        let mut d_q = [0.0; 4];
        for c in 0..4 {
            (q[c], d_q[c]) = hermite(a.q[c], a.d_q[c], b.q[c], b.d_q[c], h, s);
        }
        let len = q.iter().map(|x| x * x).sum::<f64>().sqrt();
        let q = q.map(|x| x / len);
        // the rate of the normalised quaternion, and the angular velocity it is: `ω = 2 q̇ q̄`
        let along: f64 = (0..4).map(|c| q[c] * d_q[c]).sum();
        let d = std::array::from_fn::<f64, 4, _>(|c| (d_q[c] - q[c] * along) / len);
        let [w, x, y, z] = q;
        let [dw, dx, dy, dz] = d;
        let omega = [2.0 * (-dw * x + dx * w - dy * z + dz * y),
                     2.0 * (-dw * y + dx * z + dy * w - dz * x),
                     2.0 * (-dw * z - dx * y + dy * x + dz * w)];
        Ok((o, d_o, frame(q), omega))
    }

    /// **What the motion does over a stretch `[a, b]`, enclosed** — the origin's place (a box),
    /// the turn's entries and bounds on the origin's speed and the frame's angular speed — read
    /// from samples a step `w` apart, every pose within `|o′| w / 2` and `|ω| w / 2` of one, the
    /// two bounded for certain over every segment the stretch meets.  `None` where the stretch
    /// leaves the march, or a fitted turn's length could reach zero.
    fn stretch(&self, origin: V, home: &Frame, a: f64, b: f64) -> Option<Stretch> {
        let (lo, hi) = self.domain();
        let slack = 1e-12 * (hi - lo).max(1.0);
        if !(a <= b && a >= lo - slack && b <= hi + slack) {
            return None;
        }
        let (a, b) = (a.max(lo), b.min(hi));
        let (ka, kb) = (self.segment(a).ok()?.0, self.segment(b).ok()?.0);
        let (mut speed, mut spin) = (0.0_f64, 0.0_f64);
        for &[v, turning, least] in &self.certain[ka..=kb] {
            if !(least > 0.0) {
                return None;
            }
            speed = speed.max(v);
            // `|ω| = 2 |q̇ₙ| ≤ 2 |q̇| / |q|`
            spin = spin.max(2.0 * turning / least);
        }
        let w = (b - a) / SAMPLES as f64;
        let mut plo = [f64::INFINITY; 3];
        let mut phi = [f64::NEG_INFINITY; 3];
        let mut rlo = [[f64::INFINITY; 3]; 3];
        let mut rhi = [[f64::NEG_INFINITY; 3]; 3];
        for j in 0..=SAMPLES {
            let u = if j == SAMPLES { b } else { a + w * j as f64 };
            let (o, _, f, _) = self.at(u).ok()?;
            let r = super::guide::turn(&f, home);
            for k in 0..3 {
                plo[k] = plo[k].min(o[k]);
                phi[k] = phi[k].max(o[k]);
                for m in 0..3 {
                    rlo[k][m] = rlo[k][m].min(r[k][m]);
                    rhi[k][m] = rhi[k][m].max(r[k][m]);
                }
            }
        }
        let pad = |x: f64| x.abs() * 1e-12 + 1e-15;
        let around = |lo: f64, hi: f64, by: f64, clamp: f64| {
            I::new((lo - by - pad(lo)).max(-clamp), (hi + by + pad(hi)).min(clamp)).ok()
        };
        let (reach, turnable) = (speed * w / 2.0, spin * w / 2.0);
        let mut point = [I::ZERO; 3];
        let mut turn = [[I::ZERO; 3]; 3];
        for k in 0..3 {
            point[k] = around(plo[k], phi[k], reach, f64::INFINITY)?;
            for m in 0..3 {
                turn[k][m] = around(rlo[k][m], rhi[k][m], turnable, 1.0)?;
            }
        }
        Some(Stretch { point, turn, speed: speed * (1.0 + 1e-12), spin: spin * (1.0 + 1e-12),
                       origin })
    }
}

impl Carried {
    /// The frame `f` (named `name`) carried by the drawing `sk` as it is followed along `over`,
    /// from where it stands: its track read once a march (`Caches::tracks`).
    pub(crate) fn read(sk: &Sketch, name: &str, f: u32, over: &str) -> Carried {
        Carried { read: Carried::track(sk, name, f, over) }
    }

    fn track(sk: &Sketch, name: &str, f: u32, over: &str)
        -> Result<(std::sync::Arc<Track>, V, Frame), String> {
        let march = crate::follow::followed(sk, over);
        let followed = march.as_ref().as_ref()
            .map_err(|e| format!("`{name}` is carried as the drawing is followed along \
                                  `{over}`, and it is not: {e}"))?;
        let key = (f, over.to_string());
        let known = sk.caches.tracks.borrow().get(&key)
            .filter(|(m, _)| Rc::ptr_eq(m, &march)).map(|(_, t)| t.clone());
        let track = match known {
            Some(t) => t,
            None => {
                let t = std::sync::Arc::new(Track::read(sk, name, f as usize, followed)?);
                sk.caches.tracks.borrow_mut().insert(key, (march.clone(), t.clone()));
                t
            }
        };
        if track.still {
            return Err(format!("`{name}` does not move as `{over}` runs: a frame with no joint \
                                moves where the drawing carries it, placed on what moves as it is \
                                followed"));
        }
        // home: the drawing as it stands, read off the track at the instant it stands at
        let at = match &sk.standing {
            Some(s) if s.name == over => s.at,
            _ => followed.over.from,
        };
        let u = if followed.over.turns() { at.to_radians() } else { at };
        let (origin, _, home, _) = track.at(u)?;
        Ok((track, origin, home))
    }

    /// Why the frame carries nothing, where it does not.
    pub(crate) fn still(&self) -> Option<String> {
        self.read.as_ref().err().cloned()
    }

    /// The pose at `u` and its rate: `x ↦ R (x − o_h) + o(u)`, or the world's.
    pub(crate) fn motion(&self, u: f64) -> Result<crate::envelope::Motion, String> {
        use super::guide::{mv, spun, turn};
        let Ok((track, origin, home)) = &self.read else {
            return Ok(crate::envelope::Motion::identity());
        };
        let (o, d_o, f, omega) = track.at(u)?;
        let r = turn(&f, home);
        let dr = spun(omega, &r);
        let p = sub(o, mv(&r, *origin));
        let dp = sub(d_o, mv(&dr, *origin));
        Ok(crate::envelope::Motion::of_parts(r, dr, p, dp))
    }

    /// The pose at `u` alone.
    pub(crate) fn pose(&self, u: f64) -> Result<super::Pose, String> {
        let Ok((track, origin, home)) = &self.read else {
            return Ok(super::Pose { r: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                                    p: [0.0; 3] });
        };
        let (o, _, f, _) = track.at(u)?;
        let r = super::guide::turn(&f, home);
        Ok(super::Pose { r, p: sub(o, super::guide::mv(&r, *origin)) })
    }

    /// The motion over a stretch, enclosed (`Track::stretch`); the world's, which stands still.
    pub(crate) fn stretch(&self, a: f64, b: f64) -> Option<Stretch> {
        let Ok((track, origin, home)) = &self.read else {
            let turn = std::array::from_fn(|i| std::array::from_fn(|j|
                if i == j { I::ONE } else { I::ZERO }));
            return Some(Stretch { point: [I::ZERO; 3], turn, speed: 0.0, spin: 0.0,
                                  origin: [0.0; 3] });
        };
        track.stretch(*origin, home, a, b)
    }
}
