//! **A frame along a path** (#398 C): a jointed frame whose joint is held along a path (`g
//! coincident f.axis`, `std.stations.Along`) stands where the path's parameter puts it, its pose at
//! `u` read off the path after the solve — the frame square to the path's tangent there, turned as
//! the path makes it from where the frame is drawn, its **home**.  Two frames, by how much of the
//! path's torsion the frame turns with (the joint's ratio):
//! - **0, the rotation-minimising frame** (`std.stations.On`'s and `std.stations.Minimal`'s): its
//! axes square to the
//!   tangent turn only as the tangent does, `ω = T × T′`.  Integrated by double reflection (Wang,
//!   Jüttler, Zheng and Liu, 2008) over a grid laid from the home, each node one step from the
//!   last and any other parameter one step from its nearest node: fourth order, its error at the
//!   grid's step a part in 10¹¹ of a turn along a drawing's paths.
//! - **1, the path's own frame** (`std.stations.Frenet`'s): the tangent, the principal normal
//! toward the
//!   centre of curvature and the binormal, read off the derivatives at `u` directly; it turns with
//!   the torsion besides, `ω = T × T′ + τ T` (`τ` a unit of the parameter).
//!
//! The motion is `M(u) x = R(u) (x − o_h) + c(u)`, `R(u) = F(u) F(h)ᵀ` the turn from the home's
//! frame to the one at `u` and `o_h` the path's point at the home, so a frame drawn at its home
//! stands at the path's frame wherever `u` runs, and the choice of its axes at the home (square
//! to the tangent) moves nothing.  A path is read four ways — its point and its first three
//! derivatives in its own parameter (the third only for the path's own frame's rate): a line's
//! fraction of its run, an arc's of its turn, a spline's knot parameter (in a plane, or over
//! points in space), a traced curve's formal (`crate::path`, the one reader, #645).  Its
//! enclosures over a stretch (for `MotionBounds` and the speed bound) are rigorous where its
//! derivatives have a bound — a line, an arc, a polynomial spline's hodographs — and fail closed
//! for any other path, as a law does for a function it carries no enclosure of.
use crate::interval::Interval as I;
use crate::model::{EntRef, Sketch};
use crate::path::Path;
use crate::space::{add, cross, dot, norm, scale, sub};

type V = [f64; 3];
type M = [[f64; 3]; 3];

/// How many steps of the grid the rotation-minimising frame is integrated over, end to end.
const GRID: usize = 4096;

/// How many samples a stretch is enclosed from (`Guide::stretch`).
const SAMPLES: usize = 64;

/// A frame as three unit columns: the tangent and two directions square to it.
type Frame = [V; 3];

/// The path's own frame at a point of it: the tangent, the principal normal toward the centre of
/// curvature, the binormal — `None` where it has no curvature.
fn frenet([_, d1, d2, _]: [V; 4]) -> Option<Frame> {
    let t = crate::space::normalised(d1)?;
    let b = crate::space::normalised(cross(d1, d2))?;
    Some([t, cross(b, t), b])
}

/// `r` reflected in the plane square to `v`.
fn reflect(r: V, v: V, vv: f64) -> V {
    sub(r, scale(v, 2.0 * dot(v, r) / vv))
}

/// **One step of the rotation-minimising frame** (double reflection): the frame `f` at the point
/// `x0` carried to the point `x1` with tangent `t1`.  A step that does not move keeps the frame.
pub(crate) fn reflected(f: &Frame, x0: V, x1: V, t1: V) -> Frame {
    let v1 = sub(x1, x0);
    let c1 = dot(v1, v1);
    if c1 == 0.0 {
        return *f;
    }
    let (rl, tl) = (reflect(f[1], v1, c1), reflect(f[0], v1, c1));
    let v2 = sub(t1, tl);
    let c2 = dot(v2, v2);
    let u = if c2 == 0.0 { rl } else { reflect(rl, v2, c2) };
    // held square to the tangent and unit, the rounding of every step not carried on
    let u = crate::space::normalised(sub(u, scale(t1, dot(u, t1)))).unwrap_or(u);
    [t1, u, cross(t1, u)]
}

/// `a · bᵀ` summed over the columns: the turn taking frame `b` to frame `a`.
fn turn(a: &Frame, b: &Frame) -> M {
    std::array::from_fn(|i| std::array::from_fn(|j| (0..3).map(|k| a[k][i] * b[k][j]).sum()))
}

pub(super) fn mv(m: &M, v: V) -> V {
    std::array::from_fn(|i| dot(m[i], v))
}

/// `[ω]× m`, the rate of a turn `m` turning at `ω`.
pub(super) fn spun(w: V, m: &M) -> M {
    let k = [[0.0, -w[2], w[1]], [w[2], 0.0, -w[0]], [-w[1], w[0], 0.0]];
    std::array::from_fn(|i| std::array::from_fn(|j| (0..3).map(|q| k[i][q] * m[q][j]).sum()))
}

/// **A frame along a path, read** (`MotionDef::Guided`): the path, its home and the frame there,
/// and for the rotation-minimising frame the grid it was integrated over.
#[derive(Clone, Debug)]
pub(crate) struct Guide {
    name: String,
    path: Path,
    frenet: bool,
    domain: (f64, f64),
    home: f64,
    /// The path's point at the home, and the frame there.
    origin: V,
    home_frame: Frame,
    /// The rotation-minimising frame at the grid's nodes, in order of the parameter: each its
    /// parameter, the path's point and the frame.
    nodes: Vec<(f64, V, Frame)>,
    /// `|C′|` and `|C″|` bounded over the whole path, where they are known for certain.
    bounds: Option<[f64; 2]>,
}

impl Guide {
    /// The frame `frame` along the path `guide`, the path's own (`frenet`) or the
    /// rotation-minimising one: its home where the frame's origin stands on the path.
    pub(crate) fn read(sk: &Sketch, name: &str, guide: EntRef, frame: u32, frenet: bool)
        -> Result<Guide, String> {
        let path = Path::read(sk, guide).map_err(|m| format!("`{name}`: {m}"))?;
        let domain = path.domain();
        if !(domain.0 < domain.1) {
            return Err(format!("`{name}`'s path runs over no interval"));
        }
        let chart = sk.frames.get(frame as usize).and_then(|f| f.chart.as_ref())
            .ok_or_else(|| format!("`{name}` has no origin to stand on its path"))?;
        let o: V = chart.o.map(|q| sk.params[q as usize].value);
        let home = path.nearest(o)?;
        let at = path.jet(home, false)?;
        let gap = norm(sub(at[0], o));
        if gap > 1e-6 * (1.0 + norm(o)) {
            return Err(format!("`{name}` is not drawn on its path: its origin stands {} from it \
                                — draw it at its home there, `std.stations.Along(f, g, up: …)`",
                               crate::vocab::num(gap)));
        }
        let home_frame = if frenet {
            frenet_at(name, home, at)?
        } else {
            let t = crate::space::normalised(at[1])
                .ok_or_else(|| format!("`{name}`'s path has no tangent at its home"))?;
            // any directions square to the tangent: the frame the turn is read from moves nothing
            let (u, v) = crate::space::across(t);
            [t, u, v]
        };
        let mut g = Guide { name: name.to_string(), bounds: path.derivative_bounds(), path, frenet,
                            domain, home, origin: at[0], home_frame, nodes: Vec::new() };
        if !frenet {
            g.integrate()?;
        }
        Ok(g)
    }

    /// The rotation-minimising frame at the grid's nodes, out from the home each way.
    fn integrate(&mut self) -> Result<(), String> {
        let (lo, hi) = self.domain;
        let h = (hi - lo) / GRID as f64;
        let run = |to: f64| -> Result<Vec<(f64, V, Frame)>, String> {
            let n = ((to - self.home).abs() / h).ceil() as usize;
            let mut out = vec![(self.home, self.origin, self.home_frame)];
            for k in 1..=n {
                let u = match k == n {
                    true => to,
                    false => self.home + (to - self.home) * k as f64 / n as f64,
                };
                let x = self.path.jet(u, false)?;
                let t = self.tangent(u, &x)?;
                let (_, x0, f0) = out[k - 1];
                out.push((u, x[0], reflected(&f0, x0, x[0], t)));
            }
            Ok(out)
        };
        let (mut down, up) = (run(lo)?, run(hi)?);
        down.reverse();
        down.pop();
        down.extend(up);
        self.nodes = down;
        Ok(())
    }

    /// The unit tangent at `u`, or why the path has none there.
    fn tangent(&self, u: f64, x: &[V; 4]) -> Result<V, String> {
        crate::space::normalised(x[1]).ok_or_else(|| format!(
            "`{}`'s path has no tangent at {}", self.name, crate::vocab::num(u)))
    }

    /// The frame at `u`, and the path's point and derivatives there (the third where `third`
    /// asks for it).
    fn frame(&self, u: f64, third: bool) -> Result<(Frame, [V; 4]), String> {
        let (lo, hi) = self.domain;
        if !(u >= lo - 1e-12 * (hi - lo) && u <= hi + 1e-12 * (hi - lo)) {
            return Err(format!("`{}` runs along its path over ({}, {}), and {} is past it",
                               self.name, crate::vocab::num(lo), crate::vocab::num(hi),
                               crate::vocab::num(u)));
        }
        let x = self.path.jet(u, third)?;
        if self.frenet {
            return Ok((frenet_at(&self.name, u, x)?, x));
        }
        let t = self.tangent(u, &x)?;
        let k = self.nodes.partition_point(|n| n.0 < u);
        let near = match (k.checked_sub(1).map(|j| &self.nodes[j]), self.nodes.get(k)) {
            (Some(a), Some(b)) => if u - a.0 <= b.0 - u { a } else { b },
            (Some(a), None) => a,
            (None, Some(b)) => b,
            (None, None) => return Err(format!("`{}` was integrated nowhere", self.name)),
        };
        Ok((reflected(&near.2, near.1, x[0], t), x))
    }

    /// The frame's angular velocity a unit of the parameter, at a point of the path.
    fn spin(&self, f: &Frame, x: &[V; 4]) -> V {
        let speed = norm(x[1]);
        let t = f[0];
        let bent = scale(sub(x[2], scale(t, dot(x[2], t))), 1.0 / speed);
        let w = cross(t, bent);
        if !self.frenet {
            return w;
        }
        let b = cross(x[1], x[2]);
        add(w, scale(t, dot(b, x[3]) / dot(b, b) * speed))
    }

    /// The pose at `u` and its rate: `x ↦ R (x − o_h) + c(u)`.
    pub(crate) fn motion(&self, u: f64) -> Result<crate::envelope::Motion, String> {
        let (f, x) = self.frame(u, self.frenet)?;
        let r = turn(&f, &self.home_frame);
        let dr = spun(self.spin(&f, &x), &r);
        let p = sub(x[0], mv(&r, self.origin));
        let dp = sub(x[1], mv(&dr, self.origin));
        Ok(crate::envelope::Motion::of_parts(r, dr, p, dp))
    }

    /// The pose at `u` alone.
    pub(crate) fn pose(&self, u: f64) -> Result<super::Pose, String> {
        let (f, x) = self.frame(u, false)?;
        let r = turn(&f, &self.home_frame);
        Ok(super::Pose { r, p: sub(x[0], mv(&r, self.origin)) })
    }

    /// **What the motion does over a stretch `[a, b]` of the parameter, enclosed** — the path's
    /// point (a box), the turn's entries (each an interval) and bounds on the path's speed and the
    /// frame's angular speed — or `None` where the path has no bound on its derivatives, or the
    /// frame's turn none (its speed reaching zero, or the path's own frame), so what reads it
    /// fails closed.  Read from samples a step `w` apart: every point within `w/2` of one, so off
    /// it by no more than `|C′| w / 2`, and every turn by no more than `|ω| w / 2`.
    pub(crate) fn stretch(&self, a: f64, b: f64) -> Option<Stretch> {
        let [d1, d2] = self.bounds?;
        if self.frenet || !(a <= b) {
            return None;
        }
        let (a, b) = (a.max(self.domain.0), b.min(self.domain.1));
        let w = (b - a) / SAMPLES as f64;
        let mut seen = Extremes::new();
        let mut slow = f64::INFINITY;
        for j in 0..=SAMPLES {
            let u = if j == SAMPLES { b } else { a + w * j as f64 };
            let (f, x) = self.frame(u, false).ok()?;
            seen.turn(&turn(&f, &self.home_frame));
            seen.point(x[0]);
            slow = slow.min(norm(x[1]));
        }
        // the slowest the path runs over the stretch, and so the fastest the frame turns: the
        // tangent turns at |T′| ≤ |C″| / |C′|
        let slowest = slow - d2 * w / 2.0;
        if !(slowest > 0.0) && d2 > 0.0 {
            return None;
        }
        let spin = if d2 == 0.0 { 0.0 } else { d2 / slowest };
        seen.enclosed(d1 * w / 2.0, spin * w / 2.0, d1 * (1.0 + 1e-12), spin * (1.0 + 1e-12),
                      self.origin)
    }
}

/// **What a stretch's samples reach**: the box of the points carried and every entry of the turn
/// — enclosed (`enclosed`) by how far the motion can run and turn between two samples.  A frame
/// along a path and a frame the drawing moves (`track::Ride`) are each enclosed so.
#[derive(Clone, Copy)]
pub(super) struct Extremes {
    lo: V,
    hi: V,
    rlo: M,
    rhi: M,
}

impl Extremes {
    pub(super) fn new() -> Extremes {
        Extremes { lo: [f64::INFINITY; 3], hi: [f64::NEG_INFINITY; 3],
                   rlo: [[f64::INFINITY; 3]; 3], rhi: [[f64::NEG_INFINITY; 3]; 3] }
    }

    /// A point carried, taken in.
    pub(super) fn point(&mut self, p: V) {
        for k in 0..3 {
            self.lo[k] = self.lo[k].min(p[k]);
            self.hi[k] = self.hi[k].max(p[k]);
        }
    }

    /// A turn, taken in.
    pub(super) fn turn(&mut self, r: &M) {
        for k in 0..3 {
            for m in 0..3 {
                self.rlo[k][m] = self.rlo[k][m].min(r[k][m]);
                self.rhi[k][m] = self.rhi[k][m].max(r[k][m]);
            }
        }
    }

    /// The stretch: every point within `reach` of the box, every turn's entry within `turnable` of
    /// what the samples reached, and outward by a sample's rounding; its speed, its angular speed
    /// and the origin the turn is about as given.
    pub(super) fn enclosed(&self, reach: f64, turnable: f64, speed: f64, spin: f64, origin: V)
        -> Option<Stretch> {
        let pad = |x: f64| x.abs() * 1e-12 + 1e-15;
        let around = |lo: f64, hi: f64, by: f64, clamp: f64| {
            I::new((lo - by - pad(lo)).max(-clamp), (hi + by + pad(hi)).min(clamp)).ok()
        };
        let mut point = [I::ZERO; 3];
        let mut turn = [[I::ZERO; 3]; 3];
        for k in 0..3 {
            point[k] = around(self.lo[k], self.hi[k], reach, f64::INFINITY)?;
            for m in 0..3 {
                // every entry of a turn lies within ±1
                turn[k][m] = around(self.rlo[k][m], self.rhi[k][m], turnable, 1.0)?;
            }
        }
        Some(Stretch { point, turn, speed, spin, origin })
    }
}

/// A frame along a path over a stretch of its parameter, enclosed (`Guide::stretch`): the
/// path's point, the turn's entries, the path's speed and the frame's angular speed bounded, and
/// the point at the home the turn is about.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Stretch {
    pub(crate) point: [I; 3],
    pub(crate) turn: [[I; 3]; 3],
    pub(crate) speed: f64,
    pub(crate) spin: f64,
    pub(crate) origin: V,
}

/// The path's own frame at `u`, or why it has none.
fn frenet_at(name: &str, u: f64, x: [V; 4]) -> Result<Frame, String> {
    frenet(x).ok_or_else(|| format!("`{name}`'s path has no curvature at {}, so no frame of its \
        own there: a frame along a straight run turns with none of its torsion (ratio 0)",
        crate::vocab::num(u)))
}
