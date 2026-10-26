//! **One path reader** (#645): a path of the drawing as a curve in space, read four ways — its
//! point and its first three derivatives in its own parameter — wherever a path is followed: a
//! frame along a path (`motion::guide`), a plane touching a path (`kernels`' plane tangency) and
//! the panel a plane touches along two paths (`developable`).
//!
//! - **a line**: the fraction of its run, `a + u·d`;
//! - **an arc**: the fraction of its turn, from its start the way it runs;
//! - **a circle**: closed, the fraction of a turn from its plane's `u` toward its `v`;
//! - **a spline**: its knot parameter, over control points in space (a plane's lifted);
//! - **a curve traced in space**: its formal;
//! - **a curve drawn in a plane by a formula** (`CurveBody::Exprs`, `std.shapes.Ellipse`): its
//! formal,
//!   lifted by its plane's basis — the formula's own derivatives, never differences;
//! - **a chain** of lines and arcs: `s` from 0 to the count of its links, link `k` while `s` is in
//!   `(k, k + 1)`, the place on its run; closed where it comes round.
//!
//! **Kinks.**  Where two links meet at an angle — and where a closed path's seam does — the path
//! has two tangents, one each side: `jet_side` reads either, and `kinks` says where they are.  A
//! path has no kink anywhere else: a plane that touches it there lies along its tangent, and at a
//! kink it *supports* it, lying between the two (`developable`).
use crate::model::{CurveBody, EntKind, EntRef, Sketch};
use crate::space::{add, cross, dot, norm, scale, sub};

pub type V = [f64; 3];

/// Two tangents closer than this, by the sine of the angle between them, are one: no kink.
const KINK: f64 = 1e-9;

/// The side of a place a one-sided derivative is read from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Side {
    /// As the parameter comes up to the place.
    Before,
    /// As it leaves it.
    After,
}

/// A curve drawn in a plane by a formula, as it is read: its two tapes, the variables they are
/// read at (the parameter first, replaced at each place), and its plane's basis in space.
#[derive(Clone, Debug)]
pub struct Formula {
    x: Vec<f64>,
    y: Vec<f64>,
    n_vars: usize,
    vars: Vec<f64>,
    o: V,
    u: V,
    v: V,
    domain: (f64, f64),
    closed: bool,
}

/// A path, as it is read: its point and its first three derivatives in its own parameter.
#[derive(Clone, Debug)]
pub enum Path {
    /// `a + u·d`.
    Line { a: V, d: V },
    /// About `c` in the plane of `u` and `v` (unit, square), radius `r`, from the bearing `a0` a
    /// `sweep` (radians) as `u` runs from 0 to 1.
    Arc { c: V, u: V, v: V, r: f64, a0: f64, sweep: f64 },
    /// About `c` in the plane of `u` and `v` (unit, square), radius `r`, a whole turn from `u`
    /// toward `v` as the parameter runs from 0 to 1.
    Circle { c: V, u: V, v: V, r: f64 },
    /// A cubic spline over control points in space (a plane's lifted), rational where weighted.
    Spline { ctrl: Vec<V>, knots: Vec<f64>, weights: Option<Vec<f64>> },
    /// A curve traced in space: its contact's constants and its columns' values.
    Traced { k: Vec<f64>, theta: Vec<f64>, domain: (f64, f64) },
    /// A curve drawn in a plane by a formula, lifted.
    Formula(Box<Formula>),
    /// Lines and arcs end to end, link `k` while the parameter is in `(k, k + 1)`.
    Chain { links: Vec<Path>, closed: bool },
    /// A path run the other way: at `u` the path's point at `lo + hi − u`.
    Reversed(Box<Path>),
}

impl Path {
    /// The path `g` names, as the drawing stands.
    pub fn read(sk: &Sketch, g: EntRef) -> Result<Path, String> {
        match g.kind {
            EntKind::Line => {
                let l = sk.lines.get(g.i()).ok_or("no such line")?;
                let (a, b) = (sk.lifted(l.p1 as usize), sk.lifted(l.p2 as usize));
                Ok(Path::Line { a, d: sub(b, a) })
            }
            EntKind::Arc => {
                let arc = sk.arcs.get(g.i()).ok_or("no such arc")?;
                let (c, u, v) = round_plane(sk, arc.center as usize)?;
                let (a0, a1) = sk.arc_angles(g.i());
                // one said to turn clockwise is drawn from its end, and runs from its start (#573)
                let (a0, sweep) = if sk.arc_reversed(g.i()) {
                    (a1, a0 - a1)
                } else {
                    (a0, a1 - a0)
                };
                Ok(Path::Arc { c, u, v, r: sk.radius_value(g).abs(), a0, sweep })
            }
            EntKind::Circle => {
                let k = sk.circles.get(g.i()).ok_or("no such circle")?;
                let (c, u, v) = round_plane(sk, k.center as usize)?;
                Ok(Path::Circle { c, u, v, r: sk.radius_value(g).abs() })
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
                    return Err("a surface is no path: a path is a curve".into());
                }
                Ok(Path::Traced { k: crate::traced::consts_of(t),
                                  theta: crate::traced::theta(sk, t), domain: t.over[0].pair() })
            }
            EntKind::Curve => {
                let cv = sk.curves.get(g.i()).ok_or("no such curve")?;
                let d = sk.curve_defs.get(cv.def as usize).ok_or("no such curve")?;
                let CurveBody::Exprs { x, y } = &d.body else {
                    return Err("a curve drawn in a plane is a path where a formula draws it \
                                (`point(x: …, y: …)`, `std.shapes.Ellipse`)".into());
                };
                if sk.curve_turn(g.i()).is_some() {
                    return Err("a ring's turned copy of a curve is no path: name the curve it \
                                copies".into());
                }
                let b = sk.curve_view(g.i()).map(|p| sk.basis(p))
                    .unwrap_or_else(crate::plane::Basis::page);
                let domain = sk.curve_domain(g.i());
                let domain = (domain.0.min(domain.1), domain.0.max(domain.1));
                Ok(Path::Formula(Box::new(Formula {
                    x: x.flat.clone(), y: y.flat.clone(), n_vars: x.n_vars.max(y.n_vars),
                    vars: sk.curve_vars(g.i(), domain.0),
                    o: b.o, u: b.u, v: b.v, domain, closed: sk.curve_closed(g.i()) })))
            }
            _ => Err(format!("a path is a line, an arc, a circle, a spline, a curve or a curve \
                              traced in space, and this is {}", g.kind.a())),
        }
    }

    /// The chain of `links` (lines and arcs, each running from where the last ended), closed
    /// where `closed`.  Each link is read the way the chain runs: one entered by its end is read
    /// backwards.
    pub fn chain(sk: &Sketch, links: &[EntRef], closed: bool) -> Result<Path, String> {
        if links.is_empty() {
            return Err("a chain of no links is no path".into());
        }
        let mut read = Vec::with_capacity(links.len());
        for &l in links {
            if !matches!(l.kind, EntKind::Line | EntKind::Arc) {
                return Err(format!("a chain runs over lines and arcs, and a link of it is {}",
                                   l.kind.a()));
            }
            read.push(Path::read(sk, l)?);
        }
        // each link the way the chain runs: the first so that its end meets the second, every
        // other so that its start meets the one before
        let size = read.iter().map(|p| p.size()).fold(0.0, f64::max).max(1.0);
        let near = |p: V, q: V| norm(sub(p, q)) <= 1e-7 * size;
        for k in 0..read.len() {
            let (s, e) = (read[k].start(), read[k].end());
            let flip = if k == 0 {
                read.len() > 1 && {
                    let (ns, ne) = (read[1].start(), read[1].end());
                    !near(e, ns) && !near(e, ne) && (near(s, ns) || near(s, ne))
                }
            } else {
                let prev = read[k - 1].end();
                !near(s, prev) && near(e, prev)
            };
            if flip {
                read[k] = read[k].reversed();
            }
            if k > 0 && !near(read[k].start(), read[k - 1].end()) {
                return Err(format!("the chain's link {} does not start where link {} ends", k + 1,
                                   k));
            }
        }
        if closed {
            if !near(read[0].start(), read[read.len() - 1].end()) {
                return Err("a closed chain ends where it starts, and this one does not".into());
            }
        }
        Ok(Path::Chain { links: read, closed })
    }

    /// The same path run the other way round: a line's or an arc's its own kind (a chain's link
    /// entered by its end), any other turned.
    pub fn reversed(&self) -> Path {
        match self {
            Path::Line { a, d } => Path::Line { a: add(*a, *d), d: scale(*d, -1.0) },
            Path::Arc { c, u, v, r, a0, sweep } => {
                Path::Arc { c: *c, u: *u, v: *v, r: *r, a0: a0 + sweep, sweep: -sweep }
            }
            Path::Reversed(p) => (**p).clone(),
            p => Path::Reversed(Box::new(p.clone())),
        }
    }

    /// The interval its parameter runs over.
    pub fn domain(&self) -> (f64, f64) {
        match self {
            Path::Line { .. } | Path::Arc { .. } | Path::Circle { .. } => (0.0, 1.0),
            Path::Spline { ctrl, knots, .. } => (knots[crate::curve::DEGREE], knots[ctrl.len()]),
            Path::Traced { domain, .. } => *domain,
            Path::Formula(f) => f.domain,
            Path::Chain { links, .. } => (0.0, links.len() as f64),
            Path::Reversed(p) => p.domain(),
        }
    }

    /// Whether it comes round: a circle, a closed chain, and any other whose two ends are one.
    pub fn closed(&self) -> bool {
        match self {
            Path::Circle { .. } => true,
            Path::Line { .. } | Path::Arc { .. } => false,
            Path::Chain { closed, .. } => *closed,
            Path::Formula(f) => f.closed,
            Path::Reversed(p) => p.closed(),
            _ => norm(sub(self.start(), self.end())) <= 1e-9 * self.size().max(1.0),
        }
    }

    /// Where it starts and where it ends.
    pub fn start(&self) -> V {
        self.point(self.domain().0)
    }

    pub fn end(&self) -> V {
        self.point(self.domain().1)
    }

    /// The point at `u` (NaN where it has none).
    pub fn point(&self, u: f64) -> V {
        self.jet(u, false).map(|j| j[0]).unwrap_or([f64::NAN; 3])
    }

    /// A length its points are of the size of: the largest coordinate of its ends and middle.
    pub fn size(&self) -> f64 {
        let (lo, hi) = self.domain();
        [lo, 0.5 * (lo + hi), hi].iter()
            .map(|&u| self.jet(u, false).map_or(0.0, |j| j[0].iter().fold(0.0f64, |m, c| m.max(c.abs()))))
            .fold(0.0, f64::max)
    }

    /// The link of a chain a place is on, and the place on its run, read from `side`; any other
    /// path is its own one link.
    fn link(&self, u: f64, side: Side) -> (&Path, f64) {
        let Path::Chain { links, closed } = self else { return (self, u) };
        let n = links.len();
        let mut u = u;
        if *closed {
            u = u.rem_euclid(n as f64);
            if side == Side::Before && u == 0.0 {
                u = n as f64;
            }
        }
        let u = u.clamp(0.0, n as f64);
        let mut k = u.floor() as usize;
        if side == Side::Before && u == u.floor() && k > 0 {
            k -= 1;
        }
        let k = k.min(n - 1);
        (&links[k], u - k as f64)
    }

    /// The point and its first three derivatives at `u`, the third only where `third` asks for it
    /// (zero otherwise): a traced curve's is the difference of its second, read exactly either
    /// side, and the path's own frame's rate alone reads it.  At a kink, the side it is left by.
    pub fn jet(&self, u: f64, third: bool) -> Result<[V; 4], String> {
        self.jet_side(u, third, Side::After)
    }

    /// `jet` read from one side of `u`: at a kink the two differ, anywhere else they agree.
    pub fn jet_side(&self, u: f64, third: bool, side: Side) -> Result<[V; 4], String> {
        if let Path::Chain { .. } = self {
            let (l, t) = self.link(u, side);
            return l.jet(t, third);
        }
        if let Path::Reversed(p) = self {
            let (lo, hi) = p.domain();
            let other = match side { Side::Before => Side::After, Side::After => Side::Before };
            return p.jet_side(lo + hi - u, third, other).map(turned);
        }
        match self {
            Path::Line { a, d } => Ok([add(*a, scale(*d, u)), *d, [0.0; 3], [0.0; 3]]),
            Path::Arc { c, u: x, v: y, r, a0, sweep } => Ok(round_jet(*c, *x, *y, *r, *a0, *sweep, u)),
            Path::Circle { c, u: x, v: y, r } => {
                Ok(round_jet(*c, *x, *y, *r, 0.0, std::f64::consts::TAU, u))
            }
            Path::Spline { ctrl, knots, weights } => {
                let mut span = crate::curve::span_index(knots, ctrl.len(), u);
                // read from before a knot, the span that ends there
                if side == Side::Before && span > crate::curve::DEGREE && knots[span] == u {
                    span -= 1;
                }
                Ok(spline_jet(ctrl, knots, weights.as_deref(), span, u))
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
            Path::Formula(f) => {
                let mut vars = f.vars.clone();
                vars[0] = u;
                let mut s = crate::tape::Scratch::new();
                let sx = crate::tape::eval_series_flat(&f.x, f.n_vars, &vars, &mut s);
                let sy = crate::tape::eval_series_flat(&f.y, f.n_vars, &vars, &mut s);
                if !sx.c.iter().chain(&sy.c).all(|c| c.is_finite()) {
                    return Err(format!("the curve has no point at {u}"));
                }
                // `c[k]` is `dᵏC/duᵏ`, the formula's own
                let mut out = [[0.0; 3]; 4];
                for k in 0..4 {
                    let (cx, cy) = (sx.c[k], sy.c[k]);
                    out[k] = add(scale(f.u, cx), scale(f.v, cy));
                    if k == 0 {
                        out[k] = add(out[k], f.o);
                    }
                }
                if !third {
                    out[3] = [0.0; 3];
                }
                Ok(out)
            }
            Path::Chain { .. } | Path::Reversed(_) => unreachable!("read above"),
        }
    }

    /// `jet` of the smooth piece `at` stands on, read at `u` — and past the piece's ends where
    /// `u` is, the piece carried on: a chain's link (its line or its circle), a spline's span (its
    /// cubic).  So a march that steps a hair past a kink reads the piece it is on, never the next.
    pub fn jet_near(&self, u: f64, at: f64, third: bool) -> Result<[V; 4], String> {
        match self {
            Path::Chain { links, .. } => {
                let k = (at.floor().max(0.0) as usize).min(links.len() - 1);
                links[k].jet(u - k as f64, third)
            }
            Path::Spline { ctrl, knots, weights } => {
                // the piece between the kinks `at` stands on — a spline's knots are no kinks
                // unless one repeats to break its tangent — read at `u` on its own span there,
                // and past the piece's ends on the end span carried on
                let d = crate::curve::DEGREE;
                let (mut lo, mut hi) = self.domain();
                let inner = &knots[(d + 1).min(knots.len())..ctrl.len().min(knots.len())];
                if inner.windows(d).any(|w| w[0] == w[d - 1]) {
                    for q in self.kinks() {
                        if q <= at { lo = lo.max(q) } else { hi = hi.min(q) }
                    }
                }
                let x = u.clamp(lo, hi);
                let mut span = crate::curve::span_index(knots, ctrl.len(), x);
                if x >= hi && span > d && knots[span] == x {
                    span -= 1;
                }
                Ok(spline_jet(ctrl, knots, weights.as_deref(), span, u))
            }
            Path::Reversed(p) => {
                let (lo, hi) = p.domain();
                p.jet_near(lo + hi - u, lo + hi - at, third).map(turned)
            }
            _ => self.jet(u, third),
        }
    }

    /// Where it has two tangents, in order: each joint of a chain whose links meet at an angle, and
    /// the seam of a closed path that comes round at one (at the start of its interval).
    pub fn kinks(&self) -> Vec<f64> {
        let (lo, hi) = self.domain();
        // where its pieces meet, a spline's knot a kink only where it repeats to break a tangent
        let mut at = self.joins();
        if self.closed() {
            at.insert(0, lo);
        }
        at.retain(|&u| {
            let (Ok(a), Ok(b)) = (self.jet_side(if u == lo { hi } else { u }, false, Side::Before),
                                  self.jet_side(u, false, Side::After)) else { return false };
            let (ta, tb) = (crate::space::normalised(a[1]), crate::space::normalised(b[1]));
            match (ta, tb) {
                (Some(ta), Some(tb)) => norm(cross(ta, tb)) > KINK || dot(ta, tb) < 0.0,
                _ => false,
            }
        });
        at
    }

    /// Where its pieces meet, in order: a chain's joints and a spline's interior knots — where a
    /// derivative of some order turns a corner, kinked or not.
    pub fn joins(&self) -> Vec<f64> {
        match self {
            Path::Chain { links, .. } => (1..links.len()).map(|k| k as f64).collect(),
            Path::Spline { ctrl, knots, .. } => {
                let mut out = knots[(crate::curve::DEGREE + 1).min(knots.len())
                    ..ctrl.len().min(knots.len())].to_vec();
                out.dedup();
                out
            }
            Path::Reversed(p) => {
                let (lo, hi) = p.domain();
                p.joins().into_iter().rev().map(|q| lo + hi - q).collect()
            }
            _ => Vec::new(),
        }
    }

    /// Its length over `(a, b)`: Gauss–Legendre over each smooth piece (each link of a chain,
    /// each span of a spline), each cut in halves until two readings agree.
    pub fn length(&self, a: f64, b: f64) -> f64 {
        let (lo, hi) = (a.min(b), a.max(b));
        if let Path::Reversed(p) = self {
            let (plo, phi) = p.domain();
            return p.length(plo + phi - hi, plo + phi - lo);
        }
        let mut cuts = vec![lo];
        cuts.extend(self.joins().into_iter().filter(|&u| u > lo && u < hi));
        cuts.push(hi);
        cuts.dedup();
        let speed = |u: f64| self.jet(u, false).map_or(0.0, |j| norm(j[1]));
        let mut total = 0.0;
        for w in cuts.windows(2) {
            total += adaptive(&speed, w[0], w[1], 0);
        }
        total
    }

    /// The parameter where the path comes nearest `o`: the nearest of samples along it, then
    /// Newton's on `(C − o) · C′ = 0` from there, held within the interval.
    pub fn nearest(&self, o: V) -> Result<f64, String> {
        let (lo, hi) = self.domain();
        let n = 512;
        let mut best = (f64::INFINITY, lo);
        for k in 0..=n {
            let u = lo + (hi - lo) * k as f64 / n as f64;
            let d = norm(sub(self.jet(u, false)?[0], o));
            if d < best.0 {
                best = (d, u);
            }
        }
        let mut u = best.1;
        for _ in 0..50 {
            let [c, d1, d2, _] = self.jet(u, false)?;
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

    /// Bounds on `|C′|` and `|C″|` over the whole path, where they are known for certain — a
    /// line's, an arc's and a circle's exactly, a polynomial spline's off its hodographs' control
    /// points (each derivative inside their hull) — and `None` for a path with none.
    pub fn derivative_bounds(&self) -> Option<[f64; 2]> {
        match self {
            Path::Line { d, .. } => Some([norm(*d), 0.0]),
            Path::Arc { r, sweep, .. } => {
                let w = sweep.abs();
                Some([r * w, r * w * w])
            }
            Path::Circle { r, .. } => {
                let w = std::f64::consts::TAU;
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

/// A round's point and derivatives at the fraction `u` of its sweep.
fn round_jet(c: V, x: V, y: V, r: f64, a0: f64, sweep: f64, u: f64) -> [V; 4] {
    let (s, k) = (a0 + sweep * u).sin_cos();
    let at = |cs: f64, sn: f64, m: f64| add(scale(x, m * cs), scale(y, m * sn));
    [add(c, at(k, s, r)), at(-s, k, r * sweep), at(-k, -s, r * sweep * sweep),
     at(s, -k, r * sweep.powi(3))]
}

/// A round's centre in space and its plane's two unit axes.
fn round_plane(sk: &Sketch, center: usize) -> Result<(V, V, V), String> {
    let basis = sk.plane_of(center).map(|i| sk.basis(i)).unwrap_or_else(crate::plane::Basis::page);
    let axis = |a: V| crate::space::normalised(a).ok_or("a round's plane has no axes");
    Ok((sk.lifted(center), axis(basis.u)?, axis(basis.v)?))
}

/// A spline's point and derivatives at `u`, read on the span `span` (carried on past it).
fn spline_jet(ctrl: &[V], knots: &[f64], weights: Option<&[f64]>, span: usize, u: f64) -> [V; 4] {
    use crate::curve::{basis, local_knots, local_weights, weigh, DEGREE, SPAN_N};
    let lk = local_knots(knots, span);
    let (mut b, mut d, mut dd, mut d3) = ([0.0; SPAN_N], [0.0; SPAN_N], [0.0; SPAN_N], [0.0; SPAN_N]);
    basis(u, &lk, &mut b, &mut d, &mut dd, &mut d3);
    weigh(&local_weights(weights, span), &mut b, &mut d, &mut dd, &mut d3);
    let mut out = [[0.0; 3]; 4];
    for a in 0..SPAN_N {
        let p = ctrl[span - DEGREE + a];
        for (o, w) in out.iter_mut().zip([b[a], d[a], dd[a], d3[a]]) {
            *o = add(*o, scale(p, w));
        }
    }
    out
}

/// A spline of the sketch over its control points in space.
fn spline(sk: &Sketch, i: usize) -> Path {
    let s = &sk.splines[i];
    Path::Spline { ctrl: s.ctrl.iter().map(|&p| sk.lifted(p as usize)).collect(),
                   knots: s.knots.clone(), weights: s.weights.clone() }
}

/// Five-point Gauss–Legendre of `f` over `(a, b)`.
pub(crate) fn gauss(f: &dyn Fn(f64) -> f64, a: f64, b: f64) -> f64 {
    gauss_n(&|x| [f(x)], a, b)[0]
}

/// Five-point Gauss–Legendre of each of `f`'s `N` components over `(a, b)`, `f` read once a node.
pub(crate) fn gauss_n<const N: usize>(f: &dyn Fn(f64) -> [f64; N], a: f64, b: f64) -> [f64; N] {
    const X: [f64; 5] = [0.0, 0.538_469_310_105_683_1, -0.538_469_310_105_683_1,
                         0.906_179_845_938_664, -0.906_179_845_938_664];
    const W: [f64; 5] = [0.568_888_888_888_888_9, 0.478_628_670_499_366_5,
                         0.478_628_670_499_366_5, 0.236_926_885_056_189_1,
                         0.236_926_885_056_189_1];
    let (m, h) = (0.5 * (a + b), 0.5 * (b - a));
    let mut out = [0.0; N];
    for (&x, w) in X.iter().zip(W) {
        let v = f(m + h * x);
        for k in 0..N {
            out[k] += h * w * v[k];
        }
    }
    out
}

/// `gauss` cut in halves until the halves agree with the whole to a part in 10¹⁴.
fn adaptive(f: &dyn Fn(f64) -> f64, a: f64, b: f64, depth: usize) -> f64 {
    let whole = gauss(f, a, b);
    let m = 0.5 * (a + b);
    let halves = gauss(f, a, m) + gauss(f, m, b);
    if depth >= 24 || (whole - halves).abs() <= 1e-14 * halves.abs().max(1e-300) {
        return halves;
    }
    adaptive(f, a, m, depth + 1) + adaptive(f, m, b, depth + 1)
}

/// A jet read backwards: the odd derivatives turned.
fn turned(j: [V; 4]) -> [V; 4] {
    [j[0], scale(j[1], -1.0), j[2], scale(j[3], -1.0)]
}

/* -- a path as numbers -------------------------------------------------------------------- */

/// The most numbers one path is read back from: a document is untrusted input, and a kernel's
/// constants are read as they are given.
const MAX_ENCODED: usize = 1 << 20;

impl Path {
    /// The path as numbers, a kernel's constants (`decode` reads it back): a tag, then what it is.
    pub fn encode(&self, out: &mut Vec<f64>) {
        let v3 = |out: &mut Vec<f64>, v: &V| out.extend_from_slice(v);
        match self {
            Path::Line { a, d } => {
                out.push(0.0);
                v3(out, a);
                v3(out, d);
            }
            Path::Arc { c, u, v, r, a0, sweep } => {
                out.push(1.0);
                for x in [c, u, v] {
                    v3(out, x);
                }
                out.extend_from_slice(&[*r, *a0, *sweep]);
            }
            Path::Circle { c, u, v, r } => {
                out.push(2.0);
                for x in [c, u, v] {
                    v3(out, x);
                }
                out.push(*r);
            }
            Path::Spline { ctrl, knots, weights } => {
                out.extend_from_slice(&[3.0, ctrl.len() as f64, knots.len() as f64,
                                        weights.is_some() as u8 as f64]);
                for p in ctrl {
                    v3(out, p);
                }
                out.extend_from_slice(knots);
                if let Some(w) = weights {
                    out.extend_from_slice(w);
                }
            }
            Path::Traced { k, theta, domain } => {
                out.extend_from_slice(&[4.0, k.len() as f64, theta.len() as f64, domain.0,
                                        domain.1]);
                out.extend_from_slice(k);
                out.extend_from_slice(theta);
            }
            Path::Formula(f) => {
                out.extend_from_slice(&[5.0, f.x.len() as f64, f.y.len() as f64, f.n_vars as f64,
                                        f.vars.len() as f64, f.domain.0, f.domain.1,
                                        f.closed as u8 as f64]);
                out.extend_from_slice(&f.x);
                out.extend_from_slice(&f.y);
                out.extend_from_slice(&f.vars);
                for x in [&f.o, &f.u, &f.v] {
                    v3(out, x);
                }
            }
            Path::Chain { links, closed } => {
                out.extend_from_slice(&[6.0, links.len() as f64, *closed as u8 as f64]);
                for l in links {
                    l.encode(out);
                }
            }
            Path::Reversed(p) => {
                out.push(7.0);
                p.encode(out);
            }
        }
    }

    /// A path read back from `encode`'s numbers, and how many it took; `None` where they are not
    /// one (every count and index checked: they reach a kernel from a document).
    pub fn decode(k: &[f64]) -> Option<(Path, usize)> {
        let mut at = 0usize;
        let p = decode_at(k, &mut at, 0)?;
        Some((p, at))
    }
}

fn decode_at(k: &[f64], at: &mut usize, depth: usize) -> Option<Path> {
    if depth > 4 {
        return None;
    }
    let take = |at: &mut usize, n: usize| -> Option<Vec<f64>> {
        let s = k.get(*at..at.checked_add(n)?)?.to_vec();
        *at += n;
        Some(s)
    };
    let one = |at: &mut usize| -> Option<f64> {
        let v = *k.get(*at)?;
        *at += 1;
        v.is_finite().then_some(v)
    };
    let count = |at: &mut usize| -> Option<usize> {
        let v = one(at)?;
        (v >= 0.0 && v <= MAX_ENCODED as f64 && v.fract() == 0.0).then_some(v as usize)
    };
    let v3 = |at: &mut usize| -> Option<V> {
        let s = take(at, 3)?;
        Some([s[0], s[1], s[2]])
    };
    let tag = count(at)?;
    Some(match tag {
        0 => Path::Line { a: v3(at)?, d: v3(at)? },
        1 => {
            let (c, u, v) = (v3(at)?, v3(at)?, v3(at)?);
            Path::Arc { c, u, v, r: one(at)?, a0: one(at)?, sweep: one(at)? }
        }
        2 => {
            let (c, u, v) = (v3(at)?, v3(at)?, v3(at)?);
            Path::Circle { c, u, v, r: one(at)? }
        }
        3 => {
            let (n, m, w) = (count(at)?, count(at)?, count(at)?);
            if n < crate::curve::DEGREE + 1 || m != n + crate::curve::DEGREE + 1 {
                return None;
            }
            let ctrl = (0..n).map(|_| v3(at)).collect::<Option<Vec<V>>>()?;
            let knots = take(at, m)?;
            if knots.windows(2).any(|w| !(w[1] >= w[0])) {
                return None;
            }
            let weights = if w == 1 { Some(take(at, n)?) } else { None };
            Path::Spline { ctrl, knots, weights }
        }
        4 => {
            let (n, m) = (count(at)?, count(at)?);
            let domain = (one(at)?, one(at)?);
            Path::Traced { k: take(at, n)?, theta: take(at, m)?, domain }
        }
        5 => {
            let (nx, ny, n_vars, nv) = (count(at)?, count(at)?, count(at)?, count(at)?);
            let domain = (one(at)?, one(at)?);
            let closed = one(at)? != 0.0;
            let (x, y, vars) = (take(at, nx)?, take(at, ny)?, take(at, nv)?);
            if vars.is_empty() || n_vars > nv {
                return None;
            }
            let (o, u, v) = (v3(at)?, v3(at)?, v3(at)?);
            Path::Formula(Box::new(Formula { x, y, n_vars, vars, o, u, v, domain, closed }))
        }
        6 => {
            let (n, closed) = (count(at)?, one(at)? != 0.0);
            if n == 0 || n > 4096 {
                return None;
            }
            let links = (0..n).map(|_| decode_at(k, at, depth + 1)).collect::<Option<Vec<_>>>()?;
            Path::Chain { links, closed }
        }
        7 => Path::Reversed(Box::new(decode_at(k, at, depth + 1)?)),
        _ => return None,
    })
}
