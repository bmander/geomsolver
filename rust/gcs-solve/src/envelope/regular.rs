//! **A flank regular within its blank** (#345 S3): `flank regular(by: tool) within blank`, the
//! generated flank's worst point over the part of it the tool cuts in the blank, and a number held
//! of how far that point is from a fold.
//!
//! The flank is a held envelope (`envelope::held`): the source `S(u, v)` moved by the motion at
//! the roll `t`, `X(u, v, t)`, where its normal is square to its velocity, `g = n · ∂X/∂t = 0`.
//! On the envelope `∂X/∂t` lies in the source's tangent plane, `α ∂X/∂u + β ∂X/∂v`, and the map
//! from a tool point to the point it generates — `t` solved from `g` — scales the tool's area by
//!
//! ```text
//! A = (g_t − α g_u − β g_v) / g_t,
//! ```
//!
//! the generated surface's **area factor**, oriented: 1 where the two surfaces roll without
//! sliding, nothing where the generated surface folds (the tool point's contact runs back on
//! itself: undercut), and below nothing past the fold.  It is what the sweep's admission samples
//! of the exported flank (`solid::admission`'s E3, `least_area_factor`), read here exactly.
//!
//! **Where it is read.**  A tool point generates the flank only on the tool's face, and only where
//! what it generates lies in the blank: the envelope of the whole source runs past the tool's tip
//! and folds there, inside the blank, whatever the design.  So the region is two regions'
//! terms (`model::RegionTerm`, read in closed form as `solid::field::RegionField` reads them):
//! the tool's, at the source's own point `S(u, v)` as drawn — every term but the one the source
//! lies on, and a round (`std.rounds.Rounded`: within `r` of a core of two terms, one of which the
//! source runs along `r` off) at its tangent line, the core's other term at the source point's foot
//! on the one it runs along — and the blank's, at the generated point `X`, both as drawn; and the
//! envelope's roll.
//!
//! **The worst point** is found by search, as `extremal.rs` finds a curve: a coarse grid over the
//! tool's face and the roll, its contacts in the blank kept; a shrinking grid about the best;
//! then Newton on the conditions a constrained minimum meets (`kkt`), the terms it ends on held.
//! The residual is the least area factor less the number stated; its derivative in the columns is
//! the Lagrangian's at the worst point — the factor's own, plus each held condition's times its
//! multiplier (the envelope theorem) — exact by forward differentiation, the place standing.  The
//! search is remembered by the problem's constants, a nearby question starting from the last
//! answer.  What it certifies is what a search does: the least it found, not a bound proved.
//!
//! **Layout.**  Columns `[source…, motion…, region…]`: the envelope's (`held::Parts`, lead 0),
//! then every point, axis and unknown the regions' terms read, each once.  Constants `[margin,
//! roll₀, roll₁, n, held…, terms, (side, shape, cmp, lo×3, hi, grow×3, a, b, c, sign, 0, 0)…]`
//! (`HEADER`, `TERM`), a number `c + m·x` with `x` the column `col` (−1 none).

use super::held::{self, dot, unit, vscale, vsub, Col, Layout};
use crate::kernels::{Dual, Kernel, Num};
use crate::model::{EntKind, Held, RegionShape, RegionTerm, SetKind, Sketch, SolidDef, Target,
                   TermNumber};
use crate::vocab::Cmp;

/* -- the layout ------------------------------------------------------------------------------ */

/// Constants before the envelope's.
pub const HEADER: usize = 4;
/// Constants per region term.
pub const TERM: usize = 16;

/// Where a term is read.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Side {
    /// Of the generated point, in the blank.
    Member = 0,
    /// Of the source's point, on the tool.
    Source = 1,
    /// A round's part, read only through the round.
    Part = 2,
}

/// What a term measures.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Shape {
    /// The distance from a point, columns `a`.
    Ball = 1,
    /// The distance from the line through `a` and `b`.
    Cylinder = 2,
    /// The angle from the line `a → b`, seen from `a`.
    Cone = 3,
    /// The ordinate from `a` along `b → c` (a line's ends), or along the direction `b` (`c` −1),
    /// times `sign`.
    Plane = 4,
    /// Within `lo` of the core of terms `a` and `b`, the source running along `a` — or, `c` set,
    /// a round of another face.
    Round = 5,
}

/// A number as constants: `c + m·x`, `x` column `col` where `col >= 0`.
pub type Number = (f64, f64, i64);

/// One term as the constants carry it.
#[derive(Clone, Copy, Debug)]
pub struct Term {
    pub side: Side,
    pub shape: Shape,
    pub cmp: Cmp,
    pub lo: Number,
    pub hi: f64,
    pub grow: Number,
    pub at: [i64; 3],
    pub sign: f64,
}

impl Term {
    fn write(&self, out: &mut Vec<f64>) {
        let n = |x: Number| [x.0, x.1, x.2 as f64];
        let cmp = match self.cmp {
            Cmp::Le => 0.0,
            Cmp::Ge => 1.0,
            Cmp::In => 2.0,
        };
        out.extend([self.side as i64 as f64, self.shape as i64 as f64, cmp]);
        out.extend(n(self.lo));
        out.push(self.hi);
        out.extend(n(self.grow));
        out.extend(self.at.map(|a| a as f64));
        out.extend([self.sign, 0.0, 0.0]);
    }

    fn read(k: &[f64]) -> Option<Term> {
        let side = match k[0] as i64 {
            0 => Side::Member,
            1 => Side::Source,
            2 => Side::Part,
            _ => return None,
        };
        let shape = match k[1] as i64 {
            1 => Shape::Ball,
            2 => Shape::Cylinder,
            3 => Shape::Cone,
            4 => Shape::Plane,
            5 => Shape::Round,
            _ => return None,
        };
        let cmp = match k[2] as i64 {
            0 => Cmp::Le,
            1 => Cmp::Ge,
            2 => Cmp::In,
            _ => return None,
        };
        let n = |i: usize| (k[i], k[i + 1], k[i + 2] as i64);
        Some(Term { side, shape, cmp, lo: n(3), hi: k[6], grow: n(7),
                    at: [k[10] as i64, k[11] as i64, k[12] as i64], sign: k[13] })
    }
}

/// A problem read back off its constants (the margin, `k[0]`, is the residual's alone).
#[derive(Clone, Debug)]
pub struct Problem<'a> {
    pub roll: [f64; 2],
    pub layout: Layout<'a>,
    pub terms: Vec<Term>,
}

impl Problem<'_> {
    pub fn read(k: &[f64]) -> Option<Problem<'_>> {
        let n = *k.get(3)? as usize;
        let layout = Layout::read(k.get(HEADER..HEADER + n)?)?;
        if layout.lead != 0 {
            return None;
        }
        let rest = k.get(HEADER + n..)?;
        let count = *rest.first()? as usize;
        let body = rest.get(1..)?;
        if body.len() != count * TERM {
            return None;
        }
        let terms = body.chunks(TERM).map(Term::read).collect::<Option<Vec<_>>>()?;
        Some(Problem { roll: [k[1], k[2]], layout, terms })
    }

    /// What the problem's constants are.
    pub fn write(margin: f64, roll: [f64; 2], held: &[f64], terms: &[Term]) -> Vec<f64> {
        let mut k = vec![margin, roll[0], roll[1], held.len() as f64];
        k.extend_from_slice(held);
        k.push(terms.len() as f64);
        for t in terms {
            t.write(&mut k);
        }
        k
    }

    /// How many conditions a place meets: each term not a round's part, the roll's two ends, and
    /// for a line's side its two ends.
    fn conditions(&self) -> usize {
        self.terms.iter().filter(|t| t.side != Side::Part).count() + 2
            + 2 * (self.layout.source == held::Source::Line) as usize
    }

    /// The conditions on the source's point `s` at `u` along its meridian, into `out`: the tool's
    /// terms, and a line's ends.
    fn on_tool<T: Num>(&self, c: &[T], s: [T; 3], u: T, out: &mut Vec<T>) {
        for t in self.terms.iter().filter(|t| t.side == Side::Source) {
            out.push(value(&self.terms, t, c, s));
        }
        if self.layout.source == held::Source::Line {
            out.push(u - T::cst(1.0));
            out.push(T::cst(0.0) - u);
        }
    }
}

/* -- reading a place ------------------------------------------------------------------------- */

fn number<T: Num>(c: &[T], (k, m, col): Number) -> T {
    if col >= 0 {
        T::cst(k) + T::cst(m) * c[col as usize]
    } else {
        T::cst(k)
    }
}

fn col3<T: Num>(c: &[T], at: i64) -> [T; 3] {
    let i = at as usize;
    [c[i], c[i + 1], c[i + 2]]
}

/// A term's measure at `p` against `v`, as `solid::field::region::Term::from_surface` reads it,
/// and its unit gradient there (what a round's foot moves along); `c` the region's columns.
fn from_surface<T: Num>(t: &Term, c: &[T], p: [T; 3], v: T) -> (T, [T; 3]) {
    let zero = T::cst(0.0);
    match t.shape {
        Shape::Ball => {
            let q = vsub(p, col3(c, t.at[0]));
            (dot(q, q).sqrt() - v, unit(q))
        }
        Shape::Cylinder => {
            let a = col3(c, t.at[0]);
            let d = unit(vsub(col3(c, t.at[1]), a));
            let q = vsub(p, a);
            let r = vsub(q, vscale(d, dot(q, d)));
            (dot(r, r).sqrt() - v, unit(r))
        }
        Shape::Cone => {
            let a = col3(c, t.at[0]);
            let d = unit(vsub(col3(c, t.at[1]), a));
            let q = vsub(p, a);
            let along = dot(q, d);
            let r = vsub(q, vscale(d, along));
            let across = dot(r, r).sqrt();
            let (s, k) = v.sin_cos();
            // the generator nearest, or the apex past a quarter turn either way
            let phi = T::atan2(across, along).val();
            let off = phi - v.val();
            if off.abs() <= std::f64::consts::FRAC_PI_2 {
                let e = unit(r);
                (across * k - along * s, vsub(vscale(e, k), vscale(d, s)))
            } else {
                let rho = dot(q, q).sqrt();
                let g = unit(q);
                if off > 0.0 { (rho, g) } else { (zero - rho, vscale(g, T::cst(-1.0))) }
            }
        }
        Shape::Plane => {
            let o = col3(c, t.at[0]);
            let d = if t.at[2] >= 0 {
                vsub(col3(c, t.at[2]), col3(c, t.at[1]))
            } else {
                col3(c, t.at[1])
            };
            let n = vscale(unit(d), T::cst(t.sign));
            (dot(vsub(p, o), n) - v, n)
        }
        Shape::Round => (T::cst(f64::NAN), [T::cst(f64::NAN); 3]),
    }
}

/// A term at `p`: negative where it holds (`RegionField`'s `Term::value`).
fn value<T: Num>(terms: &[Term], t: &Term, c: &[T], p: [T; 3]) -> T {
    if t.shape == Shape::Round {
        let (run, bound) = (&terms[t.at[0] as usize], &terms[t.at[1] as usize]);
        // another face's round: the core's terms less the radius, which is short of the distance
        // from the core only about that face's own edge
        if t.at[2] >= 0 {
            let (a, b) = (value(terms, run, c, p), value(terms, bound, c, p));
            return if a.val() >= b.val() { a } else { b } - number(c, t.lo);
        }
        // the source's own round, read on it: the core's other term at the point's foot on the
        // surface it runs along, `r` off it
        let (f, n) = from_surface(run, c, p, number(c, run.lo));
        let foot = vsub(p, vscale(n, f - number(c, run.grow)));
        return value(terms, bound, c, foot);
    }
    let grow = number(c, t.grow);
    match t.cmp {
        Cmp::Le => from_surface(t, c, p, number(c, t.lo)).0 - grow,
        Cmp::Ge => grow - from_surface(t, c, p, number(c, t.lo)).0,
        Cmp::In => {
            let hi = from_surface(t, c, p, T::cst(t.hi)).0;
            let lo = T::cst(0.0) - from_surface(t, c, p, number(c, t.lo)).0;
            if hi.val() >= lo.val() { hi } else { lo }
        }
    }
}

/// What a place reads: the area factor, the envelope's equation, and every condition (negative
/// where it holds) — over `cols`, `[u, v, t, columns…]`.
struct Read<T> {
    area: T,
    g: T,
    conditions: Vec<T>,
}

fn read<T: Num>(p: &Problem, cols: &[T]) -> Read<T> {
    let l = &p.layout;
    let f = held::local(l, cols, true);
    let (xu, xv, xt) = (f.dx[0], f.dx[1], f.dx[2]);
    // ∂X/∂t in the tangent plane: α ∂X/∂u + β ∂X/∂v, least squares
    let (e, ff, gg) = (dot(xu, xu), dot(xu, xv), dot(xv, xv));
    let (b1, b2) = (dot(xu, xt), dot(xv, xt));
    let det = e * gg - ff * ff;
    let (alpha, beta) = ((gg * b1 - ff * b2) / det, (e * b2 - ff * b1) / det);
    let area = (f.dg[2] - alpha * f.dg[0] - beta * f.dg[1]) / f.dg[2];
    let c = &cols[3..];
    let mut conditions = Vec::with_capacity(p.conditions());
    p.on_tool(c, held::source_at(l, cols), cols[0], &mut conditions);
    for t in p.terms.iter().filter(|t| t.side == Side::Member) {
        conditions.push(value(&p.terms, t, c, f.x));
    }
    conditions.push(cols[2] - T::cst(p.roll[1]));
    conditions.push(T::cst(p.roll[0]) - cols[2]);
    Read { area, g: f.g, conditions }
}

/* -- the search ------------------------------------------------------------------------------ */

/// The coarse grid: stations along the tool's face, turns round the source's axis, rolls.
const GRID: [usize; 3] = [24, 96, 32];

/// The worst point found: its place, its area factor, the conditions it ends on, and the
/// multipliers of the envelope's equation and of each of those.
#[derive(Clone, Debug)]
pub struct Worst {
    pub place: [f64; 3],
    pub area: f64,
    pub active: Vec<usize>,
    pub lambda: f64,
    pub mu: Vec<f64>,
}

/// One question: the problem and the columns, a place read over plain numbers.
struct Ask<'a> {
    p: &'a Problem<'a>,
    cols: Vec<f64>,
    /// The figure's size: the largest column, or one.
    size: f64,
}

impl Ask<'_> {
    /// How far a condition may stand past nothing and hold: a rounding's share of the figure.
    fn tol(&self) -> f64 {
        1e-9 * self.size
    }

    fn at(&mut self, z: [f64; 3]) -> Option<Read<f64>> {
        self.cols[..3].copy_from_slice(&z);
        let r = read(self.p, &self.cols);
        let finite = r.area.is_finite() && r.g.is_finite()
            && r.conditions.iter().all(|c| c.is_finite());
        finite.then_some(r)
    }

    /// The area factor, the equation and the conditions with their gradients in the place.
    fn grad(&self, z: [f64; 3]) -> Option<Read<Dual<3>>> {
        let cols: Vec<Dual<3>> = self.cols.iter().enumerate().map(|(i, &x)| {
            if i < 3 { Dual::var(z[i], i) } else { Dual::cst(x) }
        }).collect();
        let r = read(self.p, &cols);
        let ok = |d: &Dual<3>| d.v.is_finite() && d.g.iter().all(|x| x.is_finite());
        (ok(&r.area) && ok(&r.g) && r.conditions.iter().all(ok)).then_some(r)
    }

    /// The roll on the envelope at `(u, v)` nearest `t`, by Newton on its equation.
    fn on(&mut self, u: f64, v: f64, t: f64) -> Option<f64> {
        let mut t = t;
        for _ in 0..30 {
            let r = self.grad([u, v, t])?;
            let step = r.g.v / r.g.g[2];
            if !step.is_finite() {
                return None;
            }
            t -= step;
            if step.abs() <= 1e-14 * (1.0 + t.abs()) {
                return Some(t);
            }
        }
        None
    }

    /// The area factor at `(u, v)` on the envelope near `t`, where every condition holds.
    fn feasible(&mut self, u: f64, v: f64, t: f64) -> Option<(f64, f64)> {
        let t = self.on(u, v, t)?;
        let r = self.at([u, v, t])?;
        r.conditions.iter().all(|&c| c <= self.tol()).then_some((r.area, t))
    }

    /// Whether the source's point at `(u, v)` is on the tool's face.
    fn on_tool(&mut self, u: f64, v: f64) -> bool {
        self.cols[0] = u;
        self.cols[1] = v;
        let s = held::source_at(&self.p.layout, &self.cols);
        let mut c = Vec::new();
        self.p.on_tool(&self.cols[3..], s, u, &mut c);
        c.iter().all(|&x| x <= 0.0)
    }

    /// The stretch of the source's meridian on the tool, from a scan along it.
    fn face(&mut self) -> Option<[f64; 2]> {
        if self.p.layout.source == held::Source::Line {
            return self.span(-0.5, 1.5);
        }
        let reach = self.reach();
        self.span(-reach, reach)
    }

    fn span(&mut self, a: f64, b: f64) -> Option<[f64; 2]> {
        let n = 1000;
        let mut found: Option<[f64; 2]> = None;
        for i in 0..=n {
            let u = a + (b - a) * i as f64 / n as f64;
            let on = (0..4).any(|k| {
                let v = k as f64 * std::f64::consts::FRAC_PI_2;
                self.on_tool(u, v)
            });
            if on {
                found = Some(found.map_or([u, u], |[lo, hi]| [lo.min(u), hi.max(u)]));
            }
        }
        let step = (b - a) / n as f64;
        found.map(|[lo, hi]| [lo - step, hi + step])
    }

    /// Twice as far as the tool's terms' points stand from where the source's axis starts: what
    /// its meridian is scanned over, either way.
    fn reach(&self) -> f64 {
        let c = &self.cols[3..];
        let apex = [c[0], c[1], c[2]];
        let tool = |t: &&Term| t.side != Side::Member && t.shape != Shape::Round;
        let points = self.p.terms.iter().filter(tool).flat_map(|t| {
            // an axis's direction is no place
            let places = if t.shape == Shape::Plane && t.at[2] < 0 { 1 } else { 3 };
            t.at[..places].iter().copied().filter(|&a| a >= 0)
        });
        2.0 * points.fold(1.0f64, |far, at| far.max(crate::space::distance(col3(c, at), apex)))
    }

    /// Places on the envelope over a grid of the tool's face and the roll, where every condition
    /// holds, least area factor first.
    fn grid(&mut self, face: [f64; 2]) -> Vec<(f64, [f64; 3])> {
        let l = self.p.layout.clone();
        let (nu, nv, nt) = (GRID[0], GRID[1], GRID[2]);
        let across = |[a, b]: [f64; 2], n: usize, i: usize| a + (b - a) * i as f64 / n as f64;
        let us: Vec<f64> = (0..=nu).map(|i| across(face, nu, i)).collect();
        let vs: Vec<f64> = (0..nv).map(|j| std::f64::consts::TAU * j as f64 / nv as f64).collect();
        let ts: Vec<f64> = (0..=nt).map(|k| across(self.p.roll, nt, k)).collect();
        // the equation at every place of the grid, a roll's pose read once
        let mut g = vec![f64::NAN; ts.len() * us.len() * vs.len()];
        let idx = |k: usize, i: usize, j: usize| (k * us.len() + i) * vs.len() + j;
        for (k, &t) in ts.iter().enumerate() {
            self.cols[2] = t;
            let posed = held::posed(&l, &self.cols, t);
            for (i, &u) in us.iter().enumerate() {
                for (j, &v) in vs.iter().enumerate() {
                    self.cols[0] = u;
                    self.cols[1] = v;
                    g[idx(k, i, j)] = held::local_posed(&l, &self.cols, &posed).g;
                }
            }
        }
        let mut found = Vec::new();
        for (i, &u) in us.iter().enumerate() {
            for (j, &v) in vs.iter().enumerate() {
                for k in 0..ts.len() - 1 {
                    let (a, b) = (g[idx(k, i, j)], g[idx(k + 1, i, j)]);
                    if !(a.is_finite() && b.is_finite()) || a.signum() == b.signum() {
                        continue;
                    }
                    let t = ts[k] + (ts[k + 1] - ts[k]) * a / (a - b);
                    if let Some((area, t)) = self.feasible(u, v, t) {
                        found.push((area, [u, v, t]));
                    }
                }
            }
        }
        found.sort_by(|a, b| a.0.total_cmp(&b.0));
        found
    }

    /// The least area factor near `z`: a grid of `(u, v)` about it shrinking by a third each
    /// round, the roll on the envelope at each.
    fn shrink(&mut self, z: [f64; 3], step: [f64; 2]) -> (f64, [f64; 3]) {
        let Some((mut best, _)) = self.feasible(z[0], z[1], z[2]) else {
            return (f64::INFINITY, z);
        };
        let mut z = z;
        let mut h = step;
        let k = 3i32;
        for _ in 0..40 {
            let mut moved = z;
            for a in -k..=k {
                for b in -k..=k {
                    if a == 0 && b == 0 {
                        continue;
                    }
                    let (fa, fb) = (a as f64 / k as f64, b as f64 / k as f64);
                    let (u, v) = (z[0] + h[0] * fa, z[1] + h[1] * fb);
                    if let Some((area, t)) = self.feasible(u, v, z[2]) {
                        if area < best {
                            best = area;
                            moved = [u, v, t];
                        }
                    }
                }
            }
            if moved == z {
                h = [h[0] / 3.0, h[1] / 3.0];
            }
            z = moved;
            if h[0] <= 1e-13 * (1.0 + z[0].abs()) && h[1] <= 1e-13 {
                break;
            }
        }
        (best, z)
    }

    /// The conditions a constrained minimum at `z` meets, the conditions `active` held: the
    /// Lagrangian's gradient in the place, the envelope's equation and each held condition — by
    /// Newton over the place and the multipliers from `z`.  `None` where it does not converge, a
    /// multiplier comes out on the wrong side, or a condition left free is broken.
    fn kkt(&mut self, z: [f64; 3], active: &[usize]) -> Option<Worst> {
        let m = active.len();
        if m > 2 {
            return None;
        }
        let n = 4 + m;
        // the multipliers at `z`, least squares
        let r = self.grad(z)?;
        let mut w = vec![0.0; n];
        w[..3].copy_from_slice(&z);
        let fit = multipliers(&r, active)?;
        w[3..].copy_from_slice(&fit);
        let residual = |ask: &mut Self, w: &[f64]| -> Option<Vec<f64>> {
            let r = ask.grad([w[0], w[1], w[2]])?;
            let mut f = vec![0.0; n];
            for i in 0..3 {
                let held: f64 = active.iter().enumerate()
                    .map(|(k, &j)| w[4 + k] * r.conditions[j].g[i]).sum();
                f[i] = r.area.g[i] + w[3] * r.g.g[i] + held;
            }
            f[3] = r.g.v;
            for (k, &j) in active.iter().enumerate() {
                f[4 + k] = r.conditions[j].v;
            }
            Some(f)
        };
        let scale = |f: &[f64]| f.iter().map(|x| x * x).sum::<f64>().sqrt();
        let mut f = residual(self, &w)?;
        for _ in 0..30 {
            if scale(&f) <= 1e-13 {
                break;
            }
            // the Jacobian: the place's columns by central differences, the multipliers' exact
            let mut j = crate::linalg::Mat::zeros(n, n);
            for c in 0..3 {
                let h = 1e-7 * (1.0 + w[c].abs());
                let (mut a, mut b) = (w.clone(), w.clone());
                a[c] += h;
                b[c] -= h;
                let (fa, fb) = (residual(self, &a)?, residual(self, &b)?);
                for row in 0..n {
                    j.set(row, c, (fa[row] - fb[row]) / (2.0 * h));
                }
            }
            let r = self.grad([w[0], w[1], w[2]])?;
            for i in 0..3 {
                j.set(i, 3, r.g.g[i]);
                for (k, &jj) in active.iter().enumerate() {
                    j.set(i, 4 + k, r.conditions[jj].g[i]);
                }
            }
            let mut step = f.clone();
            if !crate::linalg::lu_solve(n, &mut j.data, &mut step) {
                return None;
            }
            // halved until the residual falls
            let mut a = 1.0f64;
            let mut next = None;
            for _ in 0..30 {
                let trial: Vec<f64> = w.iter().zip(&step).map(|(x, s)| x - a * s).collect();
                if let Some(ft) = residual(self, &trial) {
                    if scale(&ft) < scale(&f) {
                        next = Some((trial, ft));
                        break;
                    }
                }
                a *= 0.5;
            }
            let (wn, fnext) = next?;
            w = wn;
            f = fnext;
        }
        if scale(&f) > 1e-9 {
            return None;
        }
        let place = [w[0], w[1], w[2]];
        let r = self.at(place)?;
        let mu = w[4..].to_vec();
        let broken = r.conditions.iter().enumerate()
            .any(|(j, &c)| !active.contains(&j) && c > self.tol());
        if broken || mu.iter().any(|&x| x < -1e-9) {
            return None;
        }
        Some(Worst { place, area: r.area, active: active.to_vec(), lambda: w[3], mu })
    }

    /// The worst point from `z`: held at the conditions nearly met there, polished; a condition
    /// whose multiplier comes out on the wrong side let go.
    fn polish(&mut self, z: [f64; 3], area: f64) -> Option<Worst> {
        let near = self.nearly_met(z)?;
        // every subset of the nearly met, the most held first
        let mut tries: Vec<Vec<usize>> = vec![near.clone()];
        if near.len() == 2 {
            tries.push(vec![near[0]]);
            tries.push(vec![near[1]]);
        }
        if !near.is_empty() {
            tries.push(Vec::new());
        }
        let mut best: Option<Worst> = None;
        for active in tries {
            if let Some(w) = self.kkt(z, &active) {
                // a polish wandering off to another stationary place is no polish
                let near = w.area <= area + 1e-6 * (1.0 + area.abs());
                if near && best.as_ref().is_none_or(|b| w.area < b.area) {
                    best = Some(w);
                }
            }
        }
        best
    }

    /// The worst point, from nothing.
    fn cold(&mut self) -> Option<Worst> {
        let face = self.face()?;
        let found = self.grid(face);
        let step = [(face[1] - face[0]) / GRID[0] as f64, std::f64::consts::TAU / GRID[1] as f64];
        let mut best: Option<(f64, [f64; 3])> = None;
        // the grid's best few, each followed down, apart from one another
        let mut starts: Vec<[f64; 3]> = Vec::new();
        for &(_, z) in &found {
            if starts.len() >= 4 {
                break;
            }
            let apart = |s: &[f64; 3]| {
                (s[0] - z[0]).abs() > step[0] || (s[1] - z[1]).abs() > step[1]
            };
            if starts.iter().all(apart) {
                starts.push(z);
            }
        }
        for z in starts {
            let (a, z) = self.shrink(z, step);
            if best.is_none_or(|(b, _)| a < b) {
                best = Some((a, z));
            }
        }
        let (area, z) = best?;
        self.polish(z, area).or_else(|| self.fallback(z))
    }

    /// The worst point from where the last nearby question's was.
    fn warm(&mut self, was: &Worst) -> Option<Worst> {
        let t = self.on(was.place[0], was.place[1], was.place[2])?;
        let z = [was.place[0], was.place[1], t];
        if let Some(w) = self.kkt(z, &was.active) {
            return Some(w);
        }
        let r = self.at(z)?;
        if r.conditions.iter().any(|&c| c > self.tol()) {
            return None;
        }
        let step = [1e-3 * (1.0 + z[0].abs()), 1e-3];
        let (area, z) = self.shrink(z, step);
        self.polish(z, area).or_else(|| self.fallback(z))
    }

    /// A worst point the polish could not finish: where the shrinking grid ended, its
    /// multipliers fitted to the conditions nearly met there.
    fn fallback(&mut self, z: [f64; 3]) -> Option<Worst> {
        let active = self.nearly_met(z)?;
        let g = self.grad(z)?;
        let fit = multipliers(&g, &active)?;
        Some(Worst { place: z, area: g.area.v, active, lambda: fit[0], mu: fit[1..].to_vec() })
    }

    /// The two conditions nearest being broken at `z`, of those within a millionth of the figure
    /// of it: where a shrinking grid stops against them.
    fn nearly_met(&mut self, z: [f64; 3]) -> Option<Vec<usize>> {
        let r = self.at(z)?;
        let mut near: Vec<(f64, usize)> = r.conditions.iter().enumerate()
            .filter(|(_, &c)| c > -1e-6 * self.size).map(|(j, &c)| (c, j)).collect();
        near.sort_by(|a, b| b.0.total_cmp(&a.0));
        Some(near.into_iter().map(|(_, j)| j).take(2).collect())
    }
}

/// The multipliers of the envelope's equation and of the conditions `active` that best make the
/// Lagrangian's gradient in the place nothing, at a place read `r`.
fn multipliers(r: &Read<Dual<3>>, active: &[usize]) -> Option<Vec<f64>> {
    let m = 1 + active.len();
    let mut a = crate::linalg::Mat::zeros(3, m);
    for i in 0..3 {
        a.set(i, 0, r.g.g[i]);
        for (k, &j) in active.iter().enumerate() {
            a.set(i, 1 + k, r.conditions[j].g[i]);
        }
    }
    let b: Vec<f64> = (0..3).map(|i| -r.area.g[i]).collect();
    let (x, _) = crate::linalg::min_norm_solve(&a, &b, 1e-12);
    x.iter().all(|v| v.is_finite()).then_some(x)
}

/* -- the memo -------------------------------------------------------------------------------- */

thread_local! {
    /// The last worst points each problem was asked for, by its constants, with the columns they
    /// were found at: asked again at the same columns, the answer (a residual's Jacobian is asked
    /// where the residual was, and a drawing not yet solved has none); nearby, where the next
    /// search starts.
    static SEEN: std::cell::RefCell<std::collections::BTreeMap<Vec<u64>, Vec<Seen>>> =
        std::cell::RefCell::new(std::collections::BTreeMap::new());
}

/// Columns asked at, and the worst point found there, if any.
type Seen = (Vec<f64>, Option<Worst>);

const SEEN_MAX: usize = 256;
const KEEP: usize = 4;
/// How far the columns may have moved, relative to their size, for the last answer to start
/// the next search.
const NEAR: f64 = 1e-2;

/// The worst point of problem `k` at columns `cols`, `None` where no place of the flank is cut
/// within the blank, or the constants describe no problem.
pub fn worst(k: &[f64], cols: &[f64]) -> Option<Worst> {
    let p = Problem::read(k)?;
    let key: Vec<u64> = k.iter().map(|v| v.to_bits()).collect();
    let size = cols.iter().fold(1.0f64, |m, x| m.max(x.abs()));
    let far = |c: &[f64]| c.iter().zip(cols).map(|(a, b)| (a - b).abs()).fold(0.0, f64::max) / size;
    let prev = SEEN.with(|s| {
        let s = s.borrow();
        let v = s.get(&key)?;
        v.iter().filter(|(c, _)| c.len() == cols.len())
            .min_by(|a, b| far(&a.0).total_cmp(&far(&b.0))).cloned()
    });
    if let Some((c, w)) = &prev {
        if far(c) == 0.0 {
            return w.clone();
        }
    }
    let mut cols3 = vec![0.0; 3];
    cols3.extend_from_slice(cols);
    let mut ask = Ask { p: &p, cols: cols3, size };
    let found = match &prev {
        Some((c, Some(w))) if far(c) <= NEAR => ask.warm(w).or_else(|| ask.cold()),
        _ => ask.cold(),
    };
    SEEN.with(|s| {
        let mut s = s.borrow_mut();
        if s.len() >= SEEN_MAX {
            s.clear();
        }
        let v = s.entry(key).or_default();
        v.insert(0, (cols.to_vec(), found.clone()));
        v.truncate(KEEP);
    });
    found
}

/* -- the kernel ------------------------------------------------------------------------------ */

fn res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    held::block_res(n, v, k, r, 1, |x, kc, out| {
        let w = worst(kc, x)?;
        out[0] = w.area - kc[0];
        Some(())
    });
}

/// The Lagrangian's derivative in the columns at the worst point, the place and the multipliers
/// standing: `∂A + λ ∂g + Σ μ ∂c`.
fn jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    use held::PASS;
    let (n_par, _) = crate::kernels::curve_widths(n, v, k);
    held::block_res(n, v, k, j, n_par, |x, kc, block| {
        let w = worst(kc, x)?;
        let p = Problem::read(kc)?;
        let mut cols = vec![Dual::<PASS>::cst(0.0); 3 + n_par];
        for start in (0..n_par).step_by(PASS) {
            for (q, col) in cols.iter_mut().enumerate() {
                *col = if q < 3 {
                    Dual::cst(w.place[q])
                } else if (start..start + PASS).contains(&(q - 3)) {
                    Dual::var(x[q - 3], q - 3 - start)
                } else {
                    Dual::cst(x[q - 3])
                };
            }
            let r = read(&p, &cols);
            let mut l = r.area + r.g * Dual::cst(w.lambda);
            for (k, &a) in w.active.iter().enumerate() {
                l = l + r.conditions[a] * Dual::cst(w.mu[k]);
            }
            for q in start..(start + PASS).min(n_par) {
                block[q] = l.g[q - start];
            }
        }
        Some(())
    });
}

/// The kernel at a block's widths (`system::build_kernel`): one row, an area factor's.
pub fn kernel(n_par: usize, n_const: usize) -> Kernel {
    Kernel {
        name: "regular_within", n_res: 1, n_par, n_const, degree: 0, res, jac, const_jac: None,
        form: None,
    }
}
/* -- reading the sketch ---------------------------------------------------------------------- */

/// What the kernel reads of an envelope, the tool cutting it and the blank it is cut in, off the
/// sketch: the envelope's parts, the roll, and the regions' terms over columns of their own.
#[derive(Clone, Debug)]
pub struct Parts {
    held: held::Parts,
    roll: [f64; 2],
    terms: Vec<Term>,
    cols: Vec<Col>,
}

impl Parts {
    /// Envelope `e` cut by solid `tool` within solid `blank`, or why the solve cannot read them.
    pub fn of(sk: &Sketch, e: usize, tool: usize, blank: usize) -> Result<Parts, String> {
        let env = sk.envelopes.get(e).ok_or("no such envelope")?;
        let held = held::Parts::of(sk, e)?;
        let Held::Set { shape: Some(shape), .. } = &env.held else {
            return Err(format!("`{}` is no set's envelope: a flank's regularity is read of a \
                                cone's or a cylinder's, the face of the tool that cuts it",
                               env.name));
        };
        let own = match shape.kind {
            SetKind::Cone => RegionShape::Cone { axis: shape.axis },
            SetKind::Cylinder => RegionShape::Cylinder { line: shape.axis },
            SetKind::Plane => {
                return Err(format!("`{}` is a plane's envelope: a flank's regularity is read of a \
                                    cone's or a cylinder's", env.name))
            }
        };
        let base = held.own_width() as i64;
        let mut b = Builder { sk, base, cols: Vec::new(), terms: Vec::new() };
        let region = |s: usize| -> Result<(&str, &Vec<RegionTerm>), String> {
            let solid = sk.solids.get(s).ok_or("no such solid")?;
            match &solid.def {
                SolidDef::Region { terms, .. } => Ok((solid.name.as_str(), terms)),
                _ => Err(format!("`{}` is no region: a tool and a blank are read by their terms, \
                                  `solid(R)` of a set", solid.name)),
            }
        };
        let (tool_name, tool_terms) = region(tool)?;
        // the tool's face is the source itself: its own term is the face, no condition on it
        let mut face = false;
        for t in tool_terms {
            if t.shape == own && same(&t.lo, &shape.number) {
                if grown(sk, t) {
                    return Err(format!("`{tool_name}`'s face `{}` is `{}`'s source grown, not \
                                        the source: the envelope the tool cuts is of the face",
                                       t.name, env.name));
                }
                face = true;
                continue;
            }
            let said = |m: String| format!("`{tool_name}`'s `{}`: {m}", t.name);
            if let RegionShape::Measured { of } = &t.shape {
                b.round(t, of, &own, &shape.number).map_err(said)?;
                continue;
            }
            let term = b.read(t, Side::Source).map_err(said)?;
            b.terms.push(term);
        }
        if !face {
            return Err(format!("`{}`'s source is no face of `{tool_name}`: the tool named is \
                                the one whose face it is", env.name));
        }
        let (blank_name, blank_terms) = region(blank)?;
        for t in blank_terms {
            let said = |m: String| format!("`{blank_name}`'s `{}`: {m}", t.name);
            let term = b.read(t, Side::Member).map_err(said)?;
            b.terms.push(term);
        }
        Ok(Parts { held, roll: env.roll.radians(), terms: b.terms, cols: b.cols })
    }

    /// The constants, the margin stated `margin`.
    pub fn consts(&self, margin: f64) -> Vec<f64> {
        Problem::write(margin, self.roll, &self.held.consts_at(0), &self.terms)
    }

    pub fn n_const(&self) -> usize {
        HEADER + self.held.n_const() + 1 + TERM * self.terms.len()
    }

    /// Every point whose lift the columns read.
    pub fn lifted(&self) -> Vec<usize> {
        [self.held.lifted(), held::lifted(&self.cols)].concat()
    }

    pub fn columns(&self, sk: &Sketch) -> Vec<u32> {
        [self.held.columns(sk), held::columns(sk, &self.cols)].concat()
    }

    pub fn values(&self, sk: &Sketch) -> Vec<f64> {
        [self.held.values(sk), held::values(sk, &self.cols)].concat()
    }

    pub fn width(&self) -> usize {
        self.held.own_width() + held::width(&self.cols)
    }
}

/// Two numbers the same: stated alike, or the same unknown alike.
fn same(a: &TermNumber, b: &TermNumber) -> bool {
    let close = |x: f64, y: f64| (x - y).abs() <= 1e-12 * (1.0 + x.abs().max(y.abs()));
    close(a.c, b.c) && match (&a.free, &b.free) {
        (None, None) => true,
        (Some((n, m)), Some((o, k))) => n == o && close(*m, *k),
        (Some((_, m)), None) | (None, Some((_, m))) => *m == 0.0,
    }
}

/// Whether a term grows its surface by anything.
fn grown(sk: &Sketch, t: &RegionTerm) -> bool {
    t.grow.as_ref().is_some_and(|g| {
        g.read(sk).is_none_or(|v| v != 0.0) || g.free.as_ref().is_some_and(|f| f.1 != 0.0)
    })
}

struct Builder<'a> {
    sk: &'a Sketch,
    /// Where the regions' columns start.
    base: i64,
    cols: Vec<Col>,
    terms: Vec<Term>,
}

impl Builder<'_> {
    /// A column for `c`, shared with any reading it already: its offset among the kernel's.
    fn col(&mut self, c: Col) -> i64 {
        let key = |c: &Col| match *c {
            Col::Lift(p) => (0, p as u64),
            Col::One(p) => (1, p as u64),
            Col::Three(x) => (2, x[0] as u64),
        };
        let mut at = self.base;
        for have in &self.cols {
            if key(have) == key(&c) {
                return at;
            }
            at += held::width(std::slice::from_ref(have)) as i64;
        }
        self.cols.push(c);
        at
    }

    fn number(&mut self, t: &TermNumber) -> Result<Number, String> {
        let ((c, m, _), free) = held::number(self.sk, t)?;
        Ok(match free {
            Some(p) => (c, m, self.col(Col::One(p))),
            None => (c, 0.0, -1),
        })
    }

    fn line(&mut self, l: u32) -> Result<[i64; 2], String> {
        let line = self.sk.lines.get(l as usize).ok_or("no such line")?;
        let (p1, p2) = (line.p1 as usize, line.p2 as usize);
        Ok([self.col(Col::Lift(p1)), self.col(Col::Lift(p2))])
    }

    /// A ball, a cylinder, a cone or a half-space as constants; anything else refused.
    fn read(&mut self, t: &RegionTerm, side: Side) -> Result<Term, String> {
        let lo = self.number(&t.lo)?;
        let grow = match &t.grow {
            Some(g) => self.number(g)?,
            None => (0.0, 0.0, -1),
        };
        let (shape, at, sign) = match t.shape {
            RegionShape::Ball { center } => {
                (Shape::Ball, [self.col(Col::Lift(center as usize)), -1, -1], 1.0)
            }
            RegionShape::Cylinder { line } => {
                let [a, b] = self.line(line)?;
                (Shape::Cylinder, [a, b, -1], 1.0)
            }
            RegionShape::Cone { axis } => {
                let [a, b] = self.line(axis)?;
                (Shape::Cone, [a, b, -1], 1.0)
            }
            RegionShape::Plane { along, from, sign } => {
                let o = self.col(Col::Lift(from as usize));
                match along.kind {
                    EntKind::Line => {
                        let [a, b] = self.line(along.idx)?;
                        (Shape::Plane, [o, a, b], sign)
                    }
                    EntKind::Axis => {
                        let x = self.sk.axes.get(along.i()).ok_or("no such axis")?;
                        (Shape::Plane, [o, self.col(Col::Three(x.d)), -1], sign)
                    }
                    _ => return Err("a half-space is read along a line or an axis".into()),
                }
            }
            _ => return Err("a region is read here of balls, cylinders, cones, half-spaces and a \
                             tool's rounds".into()),
        };
        Ok(Term { side, shape, cmp: t.cmp, lo, hi: t.hi.unwrap_or(f64::NAN), grow, at, sign })
    }

    /// A round of the tool's face: within `r` of a core of two terms, the source running along
    /// one of them `r` off it — read on the source as the other at the foot on that one.
    fn round(&mut self, t: &RegionTerm, of: &[(Target, f64)], own: &RegionShape,
             number: &TermNumber) -> Result<(), String> {
        let [(Target::Set { terms, .. }, w)] = of else {
            return Err("a tool's term read as a distance is a round's, from one set".into());
        };
        let ([a, b], Cmp::Le, true) = (terms.as_slice(), t.cmp, *w == 1.0) else {
            return Err("a tool's round is within a distance of a core of two terms".into());
        };
        // the round of the source's own face, or of another one
        let (run, bound, other) = if a.shape == *own && same(&a.lo, number) {
            (a, b, -1)
        } else if b.shape == *own && same(&b.lo, number) {
            (b, a, -1)
        } else {
            (a, b, 1)
        };
        let at = self.terms.len() as i64;
        let run = self.read(run, Side::Part)?;
        let bound = self.read(bound, Side::Part)?;
        self.terms.push(run);
        self.terms.push(bound);
        let lo = self.number(&t.lo)?;
        let none = (0.0, 0.0, -1);
        self.terms.push(Term { side: Side::Source, shape: Shape::Round, cmp: Cmp::Le, lo,
                               hi: f64::NAN, grow: none, at: [at, at + 1, other], sign: 1.0 });
        Ok(())
    }
}
