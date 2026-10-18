//! **A frame along a path** (#398 C): a jointed frame whose joint is held along a path (`g
//! coincident f.axis`, `std.Along`) stands where the path's parameter puts it, its pose at `u` read
//! off the path after the solve — the frame square to the path's tangent there, turned as the path
//! makes it from where the frame is drawn, its **home**.  Two frames, by how much of the path's
//! torsion the frame turns with (the joint's ratio):
//! - **0, the rotation-minimising frame** (`std.On`'s and `std.Minimal`'s): its axes square to the
//!   tangent turn only as the tangent does, `ω = T × T′`.  Integrated by double reflection (Wang,
//!   Jüttler, Zheng and Liu, 2008) over a grid laid from the home, each node one step from the
//!   last and any other parameter one step from its nearest node: fourth order, its error at the
//!   grid's step a part in 10¹¹ of a turn along a drawing's paths.
//! - **1, the path's own frame** (`std.Frenet`'s): the tangent, the principal normal toward the
//!   centre of curvature and the binormal, read off the derivatives at `u` directly; it turns with
//!   the torsion besides, `ω = T × T′ + τ T` (`τ` a unit of the parameter).
//!
//! The motion is `M(u) x = R(u) (x − o_h) + c(u)`, `R(u) = F(u) F(h)ᵀ` the turn from the home's
//! frame to the one at `u` and `o_h` the path's point at the home, so a frame drawn at its home
//! stands at the path's frame wherever `u` runs, and the choice of its axes at the home (square
//! to the tangent) moves nothing.  A path is read four ways — its point and its first three
//! derivatives in its own parameter (the third only for the path's own frame's rate): a line's
//! fraction of its run, an arc's of its turn, a spline's knot parameter (in a plane, or over
//! points in space), a traced curve's formal.  Its
//! enclosures over a stretch (for `MotionBounds` and the speed bound) are rigorous where its
//! derivatives have a bound — a line, an arc, a polynomial spline's hodographs — and fail closed
//! for any other path, as a law does for a function it carries no enclosure of.
use crate::interval::Interval as I;
use crate::model::{EntKind, EntRef, Sketch};
use crate::space::{add, cross, dot, norm, scale, sub};

type V = [f64; 3];
type M = [[f64; 3]; 3];

/// How many steps of the grid the rotation-minimising frame is integrated over, end to end.
const GRID: usize = 4096;

/// How many samples a stretch is enclosed from (`Guide::stretch`).
const SAMPLES: usize = 64;

/// A path, as it is read: its point and its first three derivatives in its own parameter.
#[derive(Clone, Debug)]
enum Path {
    /// `a + u·d`.
    Line { a: V, d: V },
    /// About `c` in the plane of `u` and `v` (unit, square), radius `r`, from the bearing `a0` a
    /// `sweep` (radians) as `u` runs from 0 to 1.
    Arc { c: V, u: V, v: V, r: f64, a0: f64, sweep: f64 },
    /// A cubic spline over control points in space (a plane's lifted), rational where weighted.
    Spline { ctrl: Vec<V>, knots: Vec<f64>, weights: Option<Vec<f64>> },
    /// A curve traced in space: its contact's constants and its columns' values.
    Traced { k: Vec<f64>, theta: Vec<f64>, domain: (f64, f64) },
}

impl Path {
    /// The path `g` names, as the drawing stands.
    fn read(sk: &Sketch, g: EntRef) -> Result<Path, String> {
        match g.kind {
            EntKind::Line => {
                let l = sk.lines.get(g.i()).ok_or("no such line")?;
                let (a, b) = (sk.lifted(l.p1 as usize), sk.lifted(l.p2 as usize));
                Ok(Path::Line { a, d: sub(b, a) })
            }
            EntKind::Arc => {
                let arc = sk.arcs.get(g.i()).ok_or("no such arc")?;
                let basis = sk.plane_of(arc.center as usize).map(|i| sk.basis(i))
                    .unwrap_or_else(crate::plane::Basis::page);
                let (a0, a1) = sk.arc_angles(g.i());
                let c = sk.lifted(arc.center as usize);
                let axis = |a: V| crate::space::normalised(a).ok_or("an arc's plane has no axes");
                let (u, v) = (axis(basis.u)?, axis(basis.v)?);
                Ok(Path::Arc { c, u, v, r: sk.radius_value(g).abs(), a0, sweep: a1 - a0 })
            }
            EntKind::Spline => Ok(spline(sk, g.i())),
            EntKind::Traced => {
                // a spline drawn over points in space is a traced body of its own (#376), read
                // as the spline it is
                if let Some((&s, _)) = sk.spline_traced.iter().find(|(_, &t)| t as usize == g.i()) {
                    return Ok(spline(sk, s as usize));
                }
                let t = sk.traced.get(g.i()).ok_or("no such traced curve")?;
                if t.over.len() != 1 {
                    return Err("a surface is no path: a frame runs along a curve".into());
                }
                Ok(Path::Traced { k: crate::traced::consts_of(t),
                                  theta: crate::traced::theta(sk, t), domain: t.over[0].pair() })
            }
            _ => Err(format!("a frame runs along a line, an arc, a spline or a curve traced in \
                              space, and this is {}", g.kind.a())),
        }
    }

    /// The interval its parameter runs over.
    fn domain(&self) -> (f64, f64) {
        match self {
            Path::Line { .. } | Path::Arc { .. } => (0.0, 1.0),
            Path::Spline { ctrl, knots, .. } => (knots[crate::curve::DEGREE], knots[ctrl.len()]),
            Path::Traced { domain, .. } => *domain,
        }
    }

    /// The point and its first three derivatives at `u`, the third only where `third` asks for it
    /// (zero otherwise): a traced curve's is the difference of its second, read exactly either
    /// side, and the path's own frame's rate alone reads it.
    fn jet(&self, u: f64, third: bool) -> Result<[V; 4], String> {
        match self {
            Path::Line { a, d } => Ok([add(*a, scale(*d, u)), *d, [0.0; 3], [0.0; 3]]),
            Path::Arc { c, u: x, v: y, r, a0, sweep } => {
                let (s, k) = (a0 + sweep * u).sin_cos();
                let at = |cs: f64, sn: f64, m: f64| add(scale(*x, m * cs), scale(*y, m * sn));
                Ok([add(*c, at(k, s, *r)), at(-s, k, r * sweep), at(-k, -s, r * sweep * sweep),
                    at(s, -k, r * sweep.powi(3))])
            }
            Path::Spline { ctrl, knots, weights } => {
                use crate::curve::{basis, local_knots, local_weights, span_index, weigh, DEGREE,
                                   SPAN_N};
                let span = span_index(knots, ctrl.len(), u);
                let lk = local_knots(knots, span);
                let (mut b, mut d, mut dd, mut d3) =
                    ([0.0; SPAN_N], [0.0; SPAN_N], [0.0; SPAN_N], [0.0; SPAN_N]);
                basis(u, &lk, &mut b, &mut d, &mut dd, &mut d3);
                weigh(&local_weights(weights.as_deref(), span), &mut b, &mut d, &mut dd, &mut d3);
                let mut out = [[0.0; 3]; 4];
                for a in 0..SPAN_N {
                    let p = ctrl[span - DEGREE + a];
                    for (o, w) in out.iter_mut().zip([b[a], d[a], dd[a], d3[a]]) {
                        *o = add(*o, scale(p, w));
                    }
                }
                Ok(out)
            }
            Path::Traced { k, theta, domain } => {
                let need = crate::traced::Need::Curvature;
                let read = |u: f64| crate::traced::at(k, theta, [u, 0.0], need)
                    .ok_or_else(|| format!("the traced curve has no point at {u}"));
                let x = read(u)?;
                if !third {
                    return Ok([x.position, x.d[0], x.dd[0][0], [0.0; 3]]);
                }
                let h = 1e-4 * (domain.1 - domain.0);
                let (lo, hi) = ((u - h).max(domain.0), (u + h).min(domain.1));
                let (a, b) = (read(lo)?, read(hi)?);
                let d3 = scale(sub(b.dd[0][0], a.dd[0][0]), 1.0 / (hi - lo));
                Ok([x.position, x.d[0], x.dd[0][0], d3])
            }
        }
    }

    /// Bounds on `|C′|` and `|C″|` over the whole path, where they are known for certain — a
    /// line's and an arc's exactly, a polynomial spline's off its hodographs' control points (each
    /// derivative inside their hull) — and `None` for a path with none.
    fn derivative_bounds(&self) -> Option<[f64; 2]> {
        match self {
            Path::Line { d, .. } => Some([norm(*d), 0.0]),
            Path::Arc { r, sweep, .. } => {
                let w = sweep.abs();
                Some([r * w, r * w * w])
            }
            Path::Spline { ctrl, knots, weights: None } => {
                let mut pts = ctrl.clone();
                let mut out = [0.0; 2];
                let p = crate::curve::DEGREE;
                for (order, slot) in out.iter_mut().enumerate() {
                    let deg = p - order;
                    let next: Vec<V> = (0..pts.len().saturating_sub(1)).map(|i| {
                        let dt = knots[i + order + deg + 1] - knots[i + order + 1];
                        if dt > 0.0 { scale(sub(pts[i + 1], pts[i]), deg as f64 / dt) }
                        else { [0.0; 3] }
                    }).collect();
                    *slot = next.iter().map(|&q| norm(q)).fold(0.0, f64::max);
                    pts = next;
                }
                Some(out)
            }
            _ => None,
        }
    }
}

/// A spline of the sketch over its control points in space.
fn spline(sk: &Sketch, i: usize) -> Path {
    let s = &sk.splines[i];
    Path::Spline { ctrl: s.ctrl.iter().map(|&p| sk.lifted(p as usize)).collect(),
                   knots: s.knots.clone(), weights: s.weights.clone() }
}

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
fn reflected(f: &Frame, x0: V, x1: V, t1: V) -> Frame {
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

fn mv(m: &M, v: V) -> V {
    std::array::from_fn(|i| dot(m[i], v))
}

/// `[ω]× m`, the rate of a turn `m` turning at `ω`.
fn spun(w: V, m: &M) -> M {
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
        let home = nearest(&path, domain, o)?;
        let at = path.jet(home, false)?;
        let gap = norm(sub(at[0], o));
        if gap > 1e-6 * (1.0 + norm(o)) {
            return Err(format!("`{name}` is not drawn on its path: its origin stands {} from it \
                                — draw it at its home there, `std.Along(f, g, up: …)`",
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
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        let mut rlo = [[f64::INFINITY; 3]; 3];
        let mut rhi = [[f64::NEG_INFINITY; 3]; 3];
        let mut slow = f64::INFINITY;
        for j in 0..=SAMPLES {
            let u = if j == SAMPLES { b } else { a + w * j as f64 };
            let (f, x) = self.frame(u, false).ok()?;
            let r = turn(&f, &self.home_frame);
            for k in 0..3 {
                lo[k] = lo[k].min(x[0][k]);
                hi[k] = hi[k].max(x[0][k]);
                for m in 0..3 {
                    rlo[k][m] = rlo[k][m].min(r[k][m]);
                    rhi[k][m] = rhi[k][m].max(r[k][m]);
                }
            }
            slow = slow.min(norm(x[1]));
        }
        // the slowest the path runs over the stretch, and so the fastest the frame turns: the
        // tangent turns at |T′| ≤ |C″| / |C′|
        let slowest = slow - d2 * w / 2.0;
        if !(slowest > 0.0) && d2 > 0.0 {
            return None;
        }
        let spin = if d2 == 0.0 { 0.0 } else { d2 / slowest };
        // outward: a sample's rounding, and what lies between the samples
        let pad = |x: f64| x.abs() * 1e-12 + 1e-15;
        let around = |lo: f64, hi: f64, by: f64, clamp: f64| {
            I::new((lo - by - pad(lo)).max(-clamp), (hi + by + pad(hi)).min(clamp)).ok()
        };
        let (reach, turnable) = (d1 * w / 2.0, spin * w / 2.0);
        let mut point = [I::ZERO; 3];
        let mut turn = [[I::ZERO; 3]; 3];
        for k in 0..3 {
            point[k] = around(lo[k], hi[k], reach, f64::INFINITY)?;
            for m in 0..3 {
                // every entry of a turn lies within ±1
                turn[k][m] = around(rlo[k][m], rhi[k][m], turnable, 1.0)?;
            }
        }
        Some(Stretch { point, turn, speed: d1 * (1.0 + 1e-12), spin: spin * (1.0 + 1e-12),
                       origin: self.origin })
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

/// The parameter where the path comes nearest `o`: the nearest of samples along it, then
/// Newton's on `(C − o) · C′ = 0` from there, held within the interval.
fn nearest(path: &Path, (lo, hi): (f64, f64), o: V) -> Result<f64, String> {
    let n = 512;
    let mut best = (f64::INFINITY, lo);
    for k in 0..=n {
        let u = lo + (hi - lo) * k as f64 / n as f64;
        let d = norm(sub(path.jet(u, false)?[0], o));
        if d < best.0 {
            best = (d, u);
        }
    }
    let mut u = best.1;
    for _ in 0..50 {
        let [c, d1, d2, _] = path.jet(u, false)?;
        let e = sub(c, o);
        let (f, df) = (dot(e, d1), dot(d1, d1) + dot(e, d2));
        if df.abs() < 1e-300 {
            break;
        }
        let next = (u - f / df).clamp(lo, hi);
        let done = (next - u).abs() <= 1e-15 * (hi - lo);
        u = next;
        if done {
            break;
        }
    }
    Ok(u)
}
