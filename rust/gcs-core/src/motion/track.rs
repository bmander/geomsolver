//! **A frame the drawing moves, read as a motion** (#398 F): a frame with no joint of its own,
//! placed by relations on geometry the drawing moves as a declared parameter runs — a coupler's
//! frame, `coupler := std.Turned(b, c)` on a four-bar's coupler — stands wherever the drawing
//! followed along the parameter (`follow::followed`) puts it.  So it is a motion like any other,
//! which a sweep, a placement and an envelope read over the roll, the roll being the parameter (one
//! clock, as for a drive): the tool is carried from where the frame now stands to where it stands
//! at `t`, `M(t) x = R(t) (x − o) + c(t)`, `R(t) = F(t) F_nowᵀ` and `o` the frame's origin now.
//!
//! **The track** is the frame at every station of the march — its turn from where it stood at the
//! first station, as a unit quaternion (each one's sign the nearer to the one before, so they run
//! on), and its origin — **fitted between stations** by the cubic through the four nodes about the
//! instant asked (Lagrange's, over the stations' own spacing), the quaternion normalised, so the
//! pose and its rate are read off one polynomial and nothing is differenced.  A march's step is
//! chosen to draw, not to fit to this, so the track is **refined** where it is read: the drawing
//! is stood at the middle of each span (`Followed::put`, on the branch the march carried), and
//! where the fit there strays from it by more than `TOL` of the drawing's reach about the frame —
//! its origin, and its turn times the reach — the node is kept and the spans either side are
//! asked again.  So the fit is as near the drawing as the drawing is to itself, and a reading
//! between stations is a few multiplications.
//!
//! **Enclosed over a stretch** (`Ride::stretch`) as a frame along a path is (`guide::Stretch`):
//! the origin within each span's Bernstein hull (`Hull`, worked out once a span when the track is
//! made), and every turn's entry within `|ω| w / 2` of samples a step `w` apart, `|ω|` bounded
//! over each span by the hulls — a bound on the fitted motion, which is the motion every reader
//! reads.  It fails closed past what the march reached.
use std::rc::Rc;
use std::sync::Arc;

use super::guide::{mv, spun, Extremes, Stretch};
use crate::follow::Followed;
use crate::interval::Interval as I;
use crate::model::{Running, Sketch};
use crate::space::{cross, dot, norm, sub};

type V = [f64; 3];
type M = [[f64; 3]; 3];
type Q = [f64; 4];

/// How far the fit may stray from the drawing stood at the middle of a span, as a part of the
/// drawing's reach about the frame.
const TOL: f64 = 1e-10;

/// The most nodes a track keeps, and how many times its spans are halved at most.
const MAX_NODES: usize = 1 << 14;
const DEPTH: usize = 16;

/// How many samples a stretch is enclosed from (`Ride::stretch`).
const SAMPLES: usize = 64;

/// One node of the track: the parameter there (in its declared units), the frame's turn from the
/// first station's, and its origin.
#[derive(Clone, Copy, Debug)]
struct Node {
    at: f64,
    q: Q,
    c: V,
}

/// **The frame a drawing moves, at every node of its march** (`MotionDef::Followed`): the turn
/// from where it stood at the first station and its origin, fitted between nodes.
#[derive(Debug)]
pub struct Track {
    name: String,
    over: Running,
    reached: f64,
    nodes: Vec<Node>,
    /// The nodes' parameters, in order: what a fit's window is found in.
    ats: Vec<f64>,
    /// Each span's fit enclosed whole, once the track is refined.
    spans: Vec<Hull>,
    /// The frame's axes (as columns) at the first station: what each node's turn is from.
    reference: M,
}

fn transpose(a: &M) -> M {
    std::array::from_fn(|i| std::array::from_fn(|j| a[j][i]))
}

fn mm(a: &M, b: &M) -> M {
    std::array::from_fn(|i| std::array::from_fn(|j| (0..3).map(|k| a[i][k] * b[k][j]).sum()))
}

fn dot4(a: Q, b: Q) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]
}

/// The unit quaternion `(w, x, y, z)` of the turn `r` (Shepperd's: from its largest diagonal).
fn quaternion(r: &M) -> Q {
    let tr = r[0][0] + r[1][1] + r[2][2];
    let q = if tr > 0.0 {
        let s = 2.0 * (1.0 + tr).sqrt();
        [s / 4.0, (r[2][1] - r[1][2]) / s, (r[0][2] - r[2][0]) / s, (r[1][0] - r[0][1]) / s]
    } else if r[0][0] > r[1][1] && r[0][0] > r[2][2] {
        let s = 2.0 * (1.0 + r[0][0] - r[1][1] - r[2][2]).sqrt();
        [(r[2][1] - r[1][2]) / s, s / 4.0, (r[0][1] + r[1][0]) / s, (r[0][2] + r[2][0]) / s]
    } else if r[1][1] > r[2][2] {
        let s = 2.0 * (1.0 + r[1][1] - r[0][0] - r[2][2]).sqrt();
        [(r[0][2] - r[2][0]) / s, (r[0][1] + r[1][0]) / s, s / 4.0, (r[1][2] + r[2][1]) / s]
    } else {
        let s = 2.0 * (1.0 + r[2][2] - r[0][0] - r[1][1]).sqrt();
        [(r[1][0] - r[0][1]) / s, (r[0][2] + r[2][0]) / s, (r[1][2] + r[2][1]) / s, s / 4.0]
    };
    let n = dot4(q, q).sqrt();
    q.map(|v| v / n)
}

/// The turn of the unit quaternion `q`.
fn turn_of([w, x, y, z]: Q) -> M {
    [[1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
     [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
     [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)]]
}

/// The angular velocity of a turn whose unit quaternion `q` runs at `dq`: `2 vec(q̇ q̄)`.
fn omega([w, x, y, z]: Q, [dw, dx, dy, dz]: Q) -> V {
    let (v, dv) = ([x, y, z], [dx, dy, dz]);
    let c = cross(v, dv);
    std::array::from_fn(|k| 2.0 * (w * dv[k] - dw * v[k] + c[k]))
}

/// The nodes the fit over span `k` (from node `k` to the next) reads: the four about it, fewer
/// where the track has fewer.
pub(crate) fn window(n: usize, k: usize) -> std::ops::Range<usize> {
    if n <= 4 {
        return 0..n;
    }
    let s = k.saturating_sub(1).min(n - 4);
    s..s + 4
}

/// Lagrange's weights on the nodes `ts` at `t`, and their derivatives in `t`.
fn weights(ts: &[f64], t: f64) -> ([f64; 4], [f64; 4]) {
    let (mut w, mut d) = ([0.0; 4], [0.0; 4]);
    for i in 0..ts.len() {
        let mut value = 1.0;
        let mut rate = 0.0;
        for j in (0..ts.len()).filter(|&j| j != i) {
            let den = ts[i] - ts[j];
            // the product rule over the factors, the value's prefix times the rate's
            rate = rate * (t - ts[j]) / den + value / den;
            value *= (t - ts[j]) / den;
        }
        w[i] = value;
        d[i] = rate;
    }
    (w, d)
}

/// **The fit over a piece of one span, enclosed** by its Bernstein hulls (the cubic's four
/// control points, its derivative's three): the origin's box, the most the origin runs and the
/// quaternion turns a unit of the parameter, and the least the quaternion's length falls to — each
/// a bound on the fit, which is a cubic there, for certain but for rounding.
#[derive(Clone, Copy, Debug)]
struct Hull {
    lo: V,
    hi: V,
    speed: f64,
    rate: f64,
    least: f64,
}

/// A cubic's Bernstein control points over a piece `h` long, from its value and rate at either
/// end.
fn controls<const N: usize>(p: [f64; N], d: [f64; N], q: [f64; N], e: [f64; N], h: f64)
    -> [[f64; N]; 4] {
    [p, std::array::from_fn(|k| p[k] + h / 3.0 * d[k]),
     std::array::from_fn(|k| q[k] - h / 3.0 * e[k]), q]
}

/// The most a quadratic reaches in length, from its value at either end and the middle: the
/// largest of its Bernstein control points (the middle one twice the middle value less half the
/// ends').
fn reach<const N: usize>(a: [f64; N], m: [f64; N], b: [f64; N]) -> f64 {
    let mid: [f64; N] = std::array::from_fn(|k| 2.0 * m[k] - 0.5 * (a[k] + b[k]));
    [a, mid, b].iter().map(|v| v.iter().map(|x| x * x).sum::<f64>().sqrt()).fold(0.0, f64::max)
}

/// The farthest any point of the box `[blo, bhi]` stands from any of `[clo, chi]`.
fn farthest(blo: V, bhi: V, clo: V, chi: V) -> f64 {
    (0..3).map(|k| (bhi[k] - clo[k]).abs().max((chi[k] - blo[k]).abs()).powi(2)).sum::<f64>()
        .sqrt()
}

impl Track {
    /// **The track of frame `frame`** along the declared parameter `over`, read off the drawing's
    /// march and remembered against it (`Caches::tracks`): refused where the drawing is not
    /// followed.  `name` names the motion in a refusal.
    pub(crate) fn cached(sk: &Sketch, name: &str, frame: u32, over: &str)
        -> Result<Arc<Track>, String> {
        let march = crate::follow::followed(sk, over);
        let key = (over.to_string(), frame);
        if let Some((m, t)) = sk.caches.tracks.borrow().get(&key) {
            if Rc::ptr_eq(m, &march) {
                return Ok(t.clone());
            }
        }
        let f = march.as_ref().as_ref().map_err(|e| format!(
            "`{name}` stands where the drawing puts it as `{over}` runs, and the drawing is not \
             followed along `{over}`: {e}"))?;
        let track = Arc::new(Track::of(sk, name, frame as usize, f)?);
        sk.caches.tracks.borrow_mut().insert(key, (march.clone(), track.clone()));
        Ok(track)
    }

    /// The frame `f` at every station of the march `march`, refined.
    fn of(sk: &Sketch, name: &str, f: usize, march: &Followed) -> Result<Track, String> {
        if sk.frames.get(f).and_then(|x| x.chart.as_ref()).is_none() {
            return Err(format!("`{name}` has no axes, so no pose for the drawing to move"));
        }
        let [first, ..] = march.stations.as_slice() else {
            return Err(format!("`{name}`'s drawing is followed nowhere"));
        };
        if march.stations.len() < 2 {
            return Err(format!("`{name}`'s drawing is followed no further than its start"));
        }
        let mut copy = sk.clone();
        crate::follow::home(&mut copy);
        copy.set_x(&first.x);
        let start = super::chart_pose(&copy, f);
        let (reference, origin) = (start.r, start.p);
        let back = transpose(&reference);
        // the drawing's reach about the frame, where it starts: what a turn is weighed by
        let reach = 1.0 + (0..copy.points.len())
            .map(|p| norm(sub(copy.world_point(p), origin))).fold(0.0, f64::max);
        let node = |copy: &Sketch, at: f64| {
            let now = super::chart_pose(copy, f);
            Node { at, q: quaternion(&mm(&now.r, &back)), c: now.p }
        };
        let nodes: Vec<Node> = march.stations.iter().map(|s| {
            copy.set_x(&s.x);
            node(&copy, s.at)
        }).collect();
        let mut track = Track { name: name.to_string(), over: march.over.clone(),
                                reached: march.reached(), nodes: Vec::new(), ats: Vec::new(),
                                spans: Vec::new(), reference };
        track.set(nodes);
        // each span asked at its middle, and halved where the fit strays there
        let mut ask: Vec<usize> = (0..track.nodes.len() - 1).collect();
        let finest = (march.over.to - march.over.from) * crate::follow::FINEST;
        for _ in 0..DEPTH {
            let mut added = Vec::new();
            for &k in &ask {
                let (a, b) = (track.nodes[k].at, track.nodes[k + 1].at);
                let mid = 0.5 * (a + b);
                if b - a <= 2.0 * finest || march.put(&mut copy, mid).is_err() {
                    continue;
                }
                let stood = node(&copy, mid);
                let (p, _, c, _) = track.fit(mid);
                let n = dot4(p, p).sqrt();
                let q = p.map(|v| v / n);
                let s = if dot4(q, stood.q) < 0.0 { -1.0 } else { 1.0 };
                let turned = (0..4).map(|k| (q[k] - s * stood.q[k]).powi(2)).sum::<f64>().sqrt();
                let off = norm(sub(c, stood.c)) + 2.0 * turned * reach;
                if off > TOL * reach {
                    added.push(stood);
                }
            }
            if added.is_empty() || track.nodes.len() + added.len() > MAX_NODES {
                break;
            }
            let ats: Vec<f64> = added.iter().map(|n| n.at).collect();
            let mut all = std::mem::take(&mut track.nodes);
            all.extend(added);
            all.sort_by(|a, b| a.at.total_cmp(&b.at));
            track.set(all);
            // the spans either side of each node added, asked again
            ask = ats.iter().flat_map(|&t| {
                let k = track.nodes.partition_point(|n| n.at < t);
                [k.saturating_sub(1), k]
            }).filter(|&k| k + 1 < track.nodes.len()).collect();
            ask.dedup();
        }
        track.spans = (0..track.nodes.len() - 1)
            .map(|k| track.hull(k, track.ats[k], track.ats[k + 1])).collect();
        Ok(track)
    }

    /// Put `nodes` in, in order, each quaternion's sign the nearer to the one before.
    fn set(&mut self, mut nodes: Vec<Node>) {
        for k in 1..nodes.len() {
            if dot4(nodes[k].q, nodes[k - 1].q) < 0.0 {
                nodes[k].q = nodes[k].q.map(|v| -v);
            }
        }
        self.ats = nodes.iter().map(|n| n.at).collect();
        self.nodes = nodes;
    }

    /// The span `t` lies in.
    fn span(&self, t: f64) -> usize {
        self.ats.partition_point(|&a| a <= t).clamp(1, self.ats.len() - 1) - 1
    }

    /// The fit at `t` (the parameter's own units): the quaternion as fitted (not yet unit) and
    /// its rate, the origin and its rate, each a unit of the parameter.
    fn fit(&self, t: f64) -> (Q, Q, V, V) {
        self.fit_on(self.span(t), t)
    }

    /// The fit over span `k`, read at `t`.
    fn fit_on(&self, k: usize, t: f64) -> (Q, Q, V, V) {
        let r = window(self.nodes.len(), k);
        let (w, d) = weights(&self.ats[r.clone()], t);
        let (mut p, mut dp, mut c, mut dc) = ([0.0; 4], [0.0; 4], [0.0; 3], [0.0; 3]);
        for (i, n) in self.nodes[r].iter().enumerate() {
            for k in 0..4 {
                p[k] += w[i] * n.q[k];
                dp[k] += d[i] * n.q[k];
            }
            for k in 0..3 {
                c[k] += w[i] * n.c[k];
                dc[k] += d[i] * n.c[k];
            }
        }
        (p, dp, c, dc)
    }

    /// **The fit over `[a, b]` within span `k`, enclosed** (`Hull`).
    fn hull(&self, k: usize, a: f64, b: f64) -> Hull {
        let h = b - a;
        let (pa, dpa, ca, dca) = self.fit_on(k, a);
        let (pb, dpb, cb, dcb) = self.fit_on(k, b);
        let (pm, dpm, _, dcm) = self.fit_on(k, 0.5 * (a + b));
        let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
        for v in controls(ca, dca, cb, dcb, h) {
            for k in 0..3 {
                lo[k] = lo[k].min(v[k]);
                hi[k] = hi[k].max(v[k]);
            }
        }
        // the quaternion's length is at least its hull's reach along where it points mid-piece
        let n = dot4(pm, pm).sqrt();
        let least = controls(pa, dpa, pb, dpb, h).iter().map(|&v| dot4(v, pm) / n)
            .fold(f64::INFINITY, f64::min);
        let pad = 1.0 + 1e-9;
        Hull { lo, hi, speed: reach(dca, dcm, dcb) * pad, rate: reach(dpa, dpm, dpb) * pad,
               least: least / pad }
    }

    /// **Each piece of `[a, b]` (the parameter's units) in one span, enclosed**, handed to `each`:
    /// a whole span's hull as the track was made, the ends' worked out — or `None` where the
    /// quaternion's length may fall to nothing, so no turn is bounded.
    fn hulls(&self, a: f64, b: f64, mut each: impl FnMut(&Hull)) -> Option<()> {
        let (first, last) = (self.span(a), self.span(b));
        for k in first..=last.max(first) {
            let (s, e) = (a.max(self.ats[k]), b.min(self.ats[k + 1]));
            if !(s < e || a == b) {
                continue;
            }
            let whole = s == self.ats[k] && e == self.ats[k + 1];
            let h = if whole { self.spans[k] } else { self.hull(k, s, e) };
            if !(h.least > 0.0) {
                return None;
            }
            each(&h);
        }
        Some(())
    }

    /// The parameter a radian of the roll: degrees for an angle.
    fn rate(&self) -> f64 {
        if self.over.dim == crate::units::Dim::ANGLE { 1f64.to_degrees() } else { 1.0 }
    }

    /// The parameter at the roll `angle` (radians for an angle), or why it is not reached.
    fn param(&self, angle: f64) -> Result<f64, String> {
        let t = angle * self.rate();
        let slack = 1e-12 * (self.over.to - self.over.from);
        if !(t >= self.over.from - slack && t <= self.reached + slack) {
            return Err(format!("`{}` is followed along `{}` from {} to {}, and {} is past it",
                               self.name, self.over.name, crate::vocab::num(self.over.from),
                               crate::vocab::num(self.reached), crate::vocab::num(t)));
        }
        Ok(t.clamp(self.over.from, self.reached))
    }
}

/// **A frame the drawing moves, carried from where it now stands** (`Step::Followed`): its track,
/// the turn from the track's reference to the frame now undone (`F_ref F_nowᵀ`), and its origin
/// now.
#[derive(Clone, Debug)]
pub(crate) struct Ride {
    track: Arc<Track>,
    turn: M,
    origin: V,
}

impl Ride {
    /// The frame `frame` along `over`, carried from where it stands in `sk`.
    pub(crate) fn read(sk: &Sketch, name: &str, frame: u32, over: &str) -> Result<Ride, String> {
        let track = Track::cached(sk, name, frame, over)?;
        let now = super::chart_pose(sk, frame as usize);
        let turn = mm(&track.reference, &transpose(&now.r));
        Ok(Ride { track, turn, origin: now.p })
    }

    /// Whether `other` is this ride: the same track, from the frame standing where it stands.
    pub(crate) fn same(&self, other: &Ride) -> bool {
        Arc::ptr_eq(&self.track, &other.track) && self.turn == other.turn
            && self.origin == other.origin
    }

    /// The turn a fitted quaternion `p` (not yet unit) is, carried from the frame now.
    fn turned(&self, p: Q) -> M {
        let n = dot4(p, p).sqrt();
        mm(&turn_of(p.map(|v| v / n)), &self.turn)
    }

    /// The unit quaternion, its rate a radian of the roll, the origin and its rate there.
    fn at(&self, angle: f64) -> Result<(Q, Q, V, V), String> {
        let t = self.track.param(angle)?;
        let k = self.track.rate();
        let (p, dp, c, dc) = self.track.fit(t);
        let n = dot4(p, p).sqrt();
        if !(n > 0.0) {
            return Err(format!("`{}` has no turn at {}", self.track.name, crate::vocab::num(t)));
        }
        let q = p.map(|v| v / n);
        let along = dot4(q, dp);
        let dq = std::array::from_fn(|i| k * (dp[i] - q[i] * along) / n);
        Ok((q, dq, c, dc.map(|v| v * k)))
    }

    /// The pose at the roll `angle` and its rate: `x ↦ R (x − o) + c`.
    pub(crate) fn motion(&self, angle: f64) -> Result<crate::envelope::Motion, String> {
        let (q, dq, c, dc) = self.at(angle)?;
        let r = mm(&turn_of(q), &self.turn);
        let dr = spun(omega(q, dq), &r);
        let p = sub(c, mv(&r, self.origin));
        let dp = sub(dc, mv(&dr, self.origin));
        Ok(crate::envelope::Motion::of_parts(r, dr, p, dp))
    }

    /// The pose at `angle` alone.
    pub(crate) fn pose(&self, angle: f64) -> Result<super::Pose, String> {
        let t = self.track.param(angle)?;
        let (p, _, c, _) = self.track.fit(t);
        let r = self.turned(p);
        Ok(super::Pose { r, p: sub(c, mv(&r, self.origin)) })
    }

    /// **What the motion does over the stretch `[a, b]` of the roll, enclosed** (as a frame along
    /// a path's, `guide::Stretch`), or `None` past what the march reached.
    pub(crate) fn stretch(&self, a: f64, b: f64) -> Option<Stretch> {
        let tr = &self.track;
        if !(a <= b) {
            return None;
        }
        let (lo, hi) = (tr.param(a).ok()?, tr.param(b).ok()?);
        // the origin's box over every piece, and the most it runs and the frame turns, a unit of
        // the parameter: the angular speed `2 |q̇|` of a unit quaternion, `|q̇| ≤ |ṗ| / |p|`
        let mut seen = Extremes::new();
        let (mut speed, mut spin) = (0.0_f64, 0.0_f64);
        tr.hulls(lo, hi, |h| {
            speed = speed.max(h.speed);
            spin = spin.max(2.0 * h.rate / h.least);
            seen.point(h.lo);
            seen.point(h.hi);
        })?;
        // the turn sampled: once at an instant
        let samples = if hi > lo { SAMPLES } else { 0 };
        let w = (hi - lo) / SAMPLES as f64;
        for j in 0..=samples {
            let t = if j == samples { hi } else { lo + w * j as f64 };
            seen.turn(&self.turned(tr.fit(t).0));
        }
        let k = tr.rate();
        seen.enclosed(0.0, spin * w / 2.0, speed * k, spin * k, self.origin)
    }

    /// **The frame in the plane of `basis`** (#398 F), as a profile generated in the plane reads
    /// it (`generate::Op::Table`): `[φ_now, o_x, o_y, n, (t, w, z, c_x, c_y)…]`, the turn from the
    /// track's reference to the frame now and its origin now, and at each node the parameter,
    /// the quaternion's scalar and its part along the plane's normal, and the origin, in the
    /// plane's coordinates.  Refused where the motion leaves the plane: a turn about any other
    /// axis, or the origin off the plane's level.
    pub(crate) fn planar(&self, basis: &crate::plane::Basis) -> Result<Vec<f64>, String> {
        let tr = &self.track;
        let n = basis.normal();
        let reach = 1.0 + tr.nodes.iter().map(|x| norm(sub(x.c, self.origin))).fold(0.0, f64::max);
        let level = dot(sub(self.origin, basis.o), n);
        let leaves = || format!("`{}` leaves the plane as the drawing moves it: a profile in the \
            plane is cut under a motion that keeps it", tr.name);
        let along = |q: Q| -> Result<(f64, f64), String> {
            let v = [q[1], q[2], q[3]];
            let z = dot(v, n);
            if norm(sub(v, n.map(|x| x * z))) > 1e-9 {
                return Err(leaves());
            }
            Ok((q[0], z))
        };
        // the turn now: the frame's from the reference, `F_now F_refᵀ`
        let (w, z) = along(quaternion(&transpose(&self.turn)))?;
        let local = |p: V| { let d = sub(p, basis.o); [dot(d, basis.u), dot(d, basis.v)] };
        let o = local(self.origin);
        let mut out = vec![2.0 * z.atan2(w), o[0], o[1], tr.nodes.len() as f64];
        for x in &tr.nodes {
            let (w, z) = along(x.q)?;
            if (dot(sub(x.c, basis.o), n) - level).abs() > 1e-9 * reach {
                return Err(leaves());
            }
            let c = local(x.c);
            out.extend([x.at, w, z, c[0], c[1]]);
        }
        Ok(out)
    }

    /// **How fast the box `b` is carried, forward or back, over `[lo, hi]` of the roll** — a
    /// radian of it, and `None` past what the march reached: piece by piece, the origin's speed
    /// and the frame's angular speed times the farthest the box stands from the origin there or
    /// now (`x ↦ R (x − o) + c` turns `x − o`; its inverse turns `x − c`), rather than over the
    /// whole stretch at once, where the origin's path is wide and its fastest turn far off.
    pub(crate) fn speed_over(&self, b: [I; 3], lo: f64, hi: f64) -> Option<f64> {
        let tr = &self.track;
        if !(lo <= hi) {
            return None;
        }
        let (lo, hi) = (tr.param(lo).ok()?, tr.param(hi).ok()?);
        let (blo, bhi): (V, V) = (b.map(|x| x.bounds()[0]), b.map(|x| x.bounds()[1]));
        let now = farthest(blo, bhi, self.origin, self.origin);
        let mut out = 0.0_f64;
        tr.hulls(lo, hi, |h| {
            let far = farthest(blo, bhi, h.lo, h.hi).max(now);
            out = out.max(h.speed + 2.0 * h.rate / h.least * far);
        })?;
        Some(out * (1.0 + 1e-9) * tr.rate())
    }
}
