//! Spike #453: can a small component be *verified* unique over a box of its inputs (tier DT3 of
//! `docs/solve-boundaries-plan.md`), by interval arithmetic rather than by an argument?
//!
//! This is prototype code, held here as the spike's verified boxes (§12's DT3 row), not an API.
//! A component is its rows `F(y, x) = 0` in its own unknowns `y`, its root choices as sign
//! conditions `R(y, x) > 0`, a seed, and an *a priori region* `S(x)` that every admissible root
//! lies in.  `verify` proves, for every `x` in an input box `X`, that exactly one `y ∈ S(x)`
//! satisfies `F = 0` and `R > 0`, and that `F_y` is nonsingular there.  It does so in two
//! halves, both in outward-rounded arithmetic (`gcs_core::interval`):
//!
//! * **the tube.**  Newton at the box's middle gives `ỹ`.  The parametric Krawczyk operator
//!   `K(Y, X) = ỹ − C·F(ỹ, X) + (I − C·F_y(Y, X))·(Y − ỹ)`, with `C ≈ F_y(ỹ, mid X)⁻¹`, is
//!   applied to boxes `Y` about `ỹ` inflated until `K ⊂ int Y`.  Then for each `x ∈ X` exactly one
//!   root lies in `Y`, and every matrix in `F_y(Y, X)` is nonsingular (so `det F_y` keeps its
//!   sign: no fold inside, DD3).  `R` is checked over the whole of `Y × X`.
//! * **the exclusion.**  `S \ Y` is cut into boxes until each is shown to hold no admissible
//!   root for any `x ∈ X`: a row bounded away from zero, a sign condition `≤ 0` throughout, or a
//!   Krawczyk image disjoint from the box.  A box that will not go is either a second root,
//!   which is then proved by a tube of its own (a **refutation**), or overestimation, which
//!   splits `X` and tries again.
//!
//! Failure is reported honestly: `NotVerified` names what would not go, and the classifier
//! falls back to DT2 or DT1.  Each row below is the residual of the kernel the solver compiles
//! for that statement (`kernels.rs`, `taylor.rs`), written once over interval duals; the common
//! perpendicular's alone is written over the lines' parameters.

use gcs_core::interval::{Error, Interval as I};
use gcs_core::rng::Rng;
use std::array::from_fn;
use std::ops::{Add, Div, Mul, Sub};
use std::time::Instant;

/// An enclosure, or nothing: every operation fails closed, and a failed enclosure proves
/// nothing.
#[derive(Clone, Copy, Debug)]
struct E(Option<I>);

impl E {
    fn c(x: f64) -> E {
        E(I::point(x).ok())
    }
    fn of(x: I) -> E {
        E(Some(x))
    }
    fn lift(self, o: E, f: fn(I, I) -> Result<I, Error>) -> E {
        E(match (self.0, o.0) { (Some(a), Some(b)) => f(a, b).ok(), _ => None })
    }
    fn sq(self) -> E {
        E(self.0.and_then(|a| a.square().ok()))
    }
    fn sqrt(self) -> E {
        E(self.0.and_then(|a| a.sqrt().ok()))
    }
    /// `max(self, m)`.
    fn floor(self, m: f64) -> E {
        E(self.0.and_then(|a| I::new(lo(a).max(m), hi(a).max(m)).ok()))
    }
    /// The hull with zero: the derivative of a `max` across the point where it switches.
    fn with_zero(self) -> E {
        E(self.0.and_then(|a| I::new(lo(a).min(0.), hi(a).max(0.)).ok()))
    }
}

impl Add for E {
    type Output = E;
    fn add(self, o: E) -> E {
        self.lift(o, I::add)
    }
}
impl Sub for E {
    type Output = E;
    fn sub(self, o: E) -> E {
        self.lift(o, I::sub)
    }
}
impl Mul for E {
    type Output = E;
    fn mul(self, o: E) -> E {
        self.lift(o, I::mul)
    }
}
impl Div for E {
    type Output = E;
    fn div(self, o: E) -> E {
        self.lift(o, I::div)
    }
}

/// A value and its gradient in the `N` unknowns, each an enclosure over the box evaluated:
/// forward differentiation, so one pass gives `F` and `F_y` together, as a kernel's `Jet` form
/// gives its Taylor coefficients.
#[derive(Clone, Copy, Debug)]
struct D<const N: usize> {
    v: E,
    d: [E; N],
}

impl<const N: usize> D<N> {
    fn cst(v: I) -> Self {
        D { v: E::of(v), d: [E::c(0.); N] }
    }
    fn num(v: f64) -> Self {
        D { v: E::c(v), d: [E::c(0.); N] }
    }
    fn var(v: I, i: usize) -> Self {
        D { v: E::of(v), d: from_fn(|k| E::c(if k == i { 1. } else { 0. })) }
    }
    fn sq(self) -> Self {
        D { v: self.v.sq(), d: self.d.map(|dk| dk * self.v * E::c(2.)) }
    }
    /// `kernels::line_len`: `|(dx, dy)|` floored at `MIN_LINE_LEN`.
    fn line_len(dx: Self, dy: Self) -> Self {
        let q = dx.sq() + dy.sq();
        let s = q.v.sqrt();
        let floor = gcs_core::kernels::MIN_LINE_LEN;
        let v = s.floor(floor);
        let switches = s.0.map_or(true, |s| lo(s) < floor);
        let d = q.d.map(|dk| {
            let g = dk / (v * E::c(2.));
            if switches { g.with_zero() } else { g }
        });
        D { v, d }
    }
}

impl<const N: usize> Add for D<N> {
    type Output = Self;
    fn add(self, o: Self) -> Self {
        D { v: self.v + o.v, d: from_fn(|k| self.d[k] + o.d[k]) }
    }
}
impl<const N: usize> Sub for D<N> {
    type Output = Self;
    fn sub(self, o: Self) -> Self {
        D { v: self.v - o.v, d: from_fn(|k| self.d[k] - o.d[k]) }
    }
}
impl<const N: usize> Mul for D<N> {
    type Output = Self;
    fn mul(self, o: Self) -> Self {
        D { v: self.v * o.v, d: from_fn(|k| self.d[k] * o.v + o.d[k] * self.v) }
    }
}
impl<const N: usize> Div for D<N> {
    type Output = Self;
    fn div(self, o: Self) -> Self {
        let q = self.v / o.v;
        D { v: q, d: from_fn(|k| (self.d[k] - q * o.d[k]) / o.v) }
    }
}

fn lo(a: I) -> f64 {
    a.bounds()[0]
}
fn hi(a: I) -> f64 {
    a.bounds()[1]
}
fn width(a: I) -> f64 {
    hi(a) - lo(a)
}
fn iv(a: f64, b: f64) -> I {
    I::new(a, b).unwrap()
}
fn pt(x: f64) -> I {
    I::point(x).unwrap()
}
fn around(x: f64, r: f64) -> I {
    iv(x - r, x + r)
}
/// Inputs known exactly, as intervals.
fn exact(x: &[f64]) -> Vec<I> {
    x.iter().map(|&v| pt(v)).collect()
}
fn points<const N: usize>(y: &[f64; N]) -> [I; N] {
    from_fn(|i| pt(y[i]))
}
fn hull(xs: &[I]) -> I {
    iv(xs.iter().map(|&x| lo(x)).fold(f64::INFINITY, f64::min),
        xs.iter().map(|&x| hi(x)).fold(f64::NEG_INFINITY, f64::max))
}
/// The widest side of a box.
fn extent(b: &[I]) -> f64 {
    b.iter().fold(0f64, |m, &v| m.max(width(v)))
}
/// `1 + max |yᵢ|`: what a tolerance on `y` is relative to.
fn size(y: &[f64]) -> f64 {
    1. + y.iter().fold(0f64, |m, v| m.max(v.abs()))
}
fn widest(b: &[I]) -> usize {
    (0..b.len()).max_by(|&i, &k| width(b[i]).total_cmp(&width(b[k]))).unwrap()
}
/// `b` cut across side `i` at `at`.
fn halves<B: Clone + AsMut<[I]> + AsRef<[I]>>(b: &B, i: usize, at: f64) -> [B; 2] {
    let s = b.as_ref()[i];
    let (mut p, mut q) = (b.clone(), b.clone());
    p.as_mut()[i] = iv(lo(s), at);
    q.as_mut()[i] = iv(at, hi(s));
    [p, q]
}
/// `k` strictly inside `y`: Krawczyk's test.
fn interior(k: &[I], y: &[I]) -> bool {
    k.iter().zip(y).all(|(&k, &y)| lo(y) < lo(k) && hi(k) < hi(y))
}
fn apart(a: &[I], b: &[I]) -> bool {
    a.iter().zip(b).any(|(&a, &b)| hi(a) < lo(b) || hi(b) < lo(a))
}

/// `(b − a) × (p − a)`: positive when `p` is left of `a → b` (`ccw(a, b, p)`).
fn ccw<const N: usize>(a: [D<N>; 2], b: [D<N>; 2], p: [D<N>; 2]) -> D<N> {
    (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
}

/// `point_on_circle`'s residual: `|p − c|² − r²`.
fn on_circle<const N: usize>(p: [D<N>; 2], c: [D<N>; 2], r: D<N>) -> D<N> {
    (p[0] - c[0]).sq() + (p[1] - c[1]).sq() - r.sq()
}

/// `tangent_line_circle`'s residual over the line `p1 → p2`: the signed distance of the
/// centre, positive on the left, less `k·r`.
fn tangent<const N: usize>(p1: [D<N>; 2], p2: [D<N>; 2], c: [D<N>; 2], r: D<N>, k: f64) -> D<N> {
    let (dx, dy) = (p2[0] - p1[0], p2[1] - p1[1]);
    let (wx, wy) = (c[0] - p1[0], c[1] - p1[1]);
    (dx * wy - dy * wx) / D::line_len(dx, dy) - r * D::num(k)
}

type Rows<const N: usize> = fn(&[D<N>; N], &[D<N>]) -> [D<N>; N];
type Signs<const N: usize> = fn(&[D<N>; N], &[D<N>]) -> Vec<D<N>>;

struct Component<const N: usize> {
    rows: Rows<N>,
    /// Each must be strictly positive at an admissible root.
    signs: Signs<N>,
    seed: fn(&[f64]) -> [f64; N],
    /// A box holding every admissible root for every input in the box given.
    region: fn(&[I]) -> [I; N],
}

#[derive(Debug, Default)]
struct Cost {
    /// Interval evaluations of the rows (value and gradient in one pass).
    evals: usize,
    /// Input boxes the verification was attempted on.
    input_boxes: usize,
    /// Boxes of unknowns examined by the exclusion.
    unknown_boxes: usize,
    /// Input boxes split because no tube contracted over them, or one did but its root choices
    /// were not decided over the whole of it.
    no_tube: usize,
    /// Input boxes split because a box of unknowns would not go and was no second root.
    undecided: usize,
}

#[derive(Debug)]
enum Outcome<const N: usize> {
    Verified,
    NotVerified(String),
    /// Two admissible roots at one input, each proved by a tube of its own.
    Refuted { x: Vec<f64>, roots: [[f64; N]; 2] },
}

fn eval<const N: usize>(c: &Component<N>, y: &[I; N], x: &[I], cost: &mut Cost) -> [D<N>; N] {
    cost.evals += 1;
    let yv: [D<N>; N] = from_fn(|i| D::var(y[i], i));
    let xv: Vec<D<N>> = x.iter().map(|&v| D::cst(v)).collect();
    (c.rows)(&yv, &xv)
}

fn signs<const N: usize>(c: &Component<N>, y: &[I; N], x: &[I]) -> Vec<E> {
    let yv: [D<N>; N] = from_fn(|i| D::cst(y[i]));
    let xv: Vec<D<N>> = x.iter().map(|&v| D::cst(v)).collect();
    (c.signs)(&yv, &xv).into_iter().map(|s| s.v).collect()
}

/// Every sign condition strictly positive over the box.
fn admitted<const N: usize>(c: &Component<N>, y: &[I; N], x: &[I]) -> bool {
    signs(c, y, x).iter().all(|s| s.0.is_some_and(|s| lo(s) > 0.))
}

fn mid_matrix<const N: usize>(r: &[D<N>; N]) -> Option<[[f64; N]; N]> {
    let mut m = [[0.; N]; N];
    for i in 0..N {
        for k in 0..N {
            m[i][k] = r[i].d[k].0?.mid();
        }
    }
    Some(m)
}

/// Gauss–Jordan with partial pivoting: a floating-point *preconditioner*, which the Krawczyk
/// test does not need to be exact.
fn inverse<const N: usize>(mut a: [[f64; N]; N]) -> Option<[[f64; N]; N]> {
    let mut inv: [[f64; N]; N] = from_fn(|i| from_fn(|k| if i == k { 1. } else { 0. }));
    for col in 0..N {
        let p = (col..N).max_by(|&i, &k| a[i][col].abs().total_cmp(&a[k][col].abs()))?;
        if !(a[p][col].abs() > 1e-300) {
            return None;
        }
        a.swap(col, p);
        inv.swap(col, p);
        let d = a[col][col];
        for k in 0..N {
            a[col][k] /= d;
            inv[col][k] /= d;
        }
        for i in 0..N {
            if i != col {
                let f = a[i][col];
                for k in 0..N {
                    a[i][k] -= f * a[col][k];
                    inv[i][k] -= f * inv[col][k];
                }
            }
        }
    }
    inv.iter().flatten().all(|v| v.is_finite()).then_some(inv)
}

fn newton<const N: usize>(c: &Component<N>, x: &[f64], seed: [f64; N], cost: &mut Cost)
    -> Option<[f64; N]> {
    let xi = exact(x);
    // Only an approximation is wanted: the tube is what proves anything.
    let mut y = seed;
    let mut last = f64::INFINITY;
    for _ in 0..80 {
        let r = eval(c, &points(&y), &xi, cost);
        let ci = inverse(mid_matrix(&r)?)?;
        let f: Vec<f64> = r.iter().map(|ri| ri.v.0.map(|v| v.mid())).collect::<Option<_>>()?;
        let step: [f64; N] = from_fn(|i| (0..N).map(|k| ci[i][k] * f[k]).sum());
        for i in 0..N {
            y[i] -= step[i];
        }
        last = step.iter().fold(0f64, |m, s| m.max(s.abs())) / size(&y);
        if last <= 1e-14 {
            break;
        }
    }
    (last <= 1e-10).then_some(y)
}

/// `F(ỹ, X)`: the one evaluation of the Krawczyk operator that does not depend on `Y`.
fn at_centre<const N: usize>(c: &Component<N>, yt: &[f64; N], x: &[I], cost: &mut Cost)
    -> [E; N] {
    eval(c, &points(yt), x, cost).map(|r| r.v)
}

fn krawczyk<const N: usize>(
    c: &Component<N>, y: &[I; N], yt: &[f64; N], f0: &[E; N], cm: &[[f64; N]; N], x: &[I],
    cost: &mut Cost,
) -> Option<[I; N]> {
    let r = eval(c, y, x, cost);
    let mut k = [I::ZERO; N];
    for i in 0..N {
        let mut s = E::c(yt[i]);
        for l in 0..N {
            s = s - E::c(cm[i][l]) * f0[l];
        }
        for m in 0..N {
            let mut a = E::c(if i == m { 1. } else { 0. });
            for l in 0..N {
                a = a - E::c(cm[i][l]) * r[l].d[m];
            }
            s = s + a * (E::of(y[m]) - E::c(yt[m]));
        }
        k[i] = s.0?;
    }
    Some(k)
}

/// A box `Y` about `ỹ` with `K(Y, X) ⊂ int Y`, and what proved it.
struct Tube<const N: usize> {
    y: [I; N],
    yt: [f64; N],
    f0: [E; N],
    cm: [[f64; N]; N],
}

/// A tube about `ỹ` by ε-inflation; `None` if none is found.
fn tube<const N: usize>(c: &Component<N>, x: &[I], yt: [f64; N], cost: &mut Cost)
    -> Option<Tube<N>> {
    let xm: Vec<I> = x.iter().map(|v| pt(v.mid())).collect();
    let cm = inverse(mid_matrix(&eval(c, &points(&yt), &xm, cost))?)?;
    let f0 = at_centre(c, &yt, x, cost);
    let pad = 1e-12 * size(&yt);
    let mut y = [I::ZERO; N];
    for i in 0..N {
        let mut z = E::c(0.);
        for l in 0..N {
            z = z + E::c(cm[i][l]) * f0[l];
        }
        let r = z.0.map(|z| lo(z).abs().max(hi(z).abs()))?;
        y[i] = I::new(yt[i] - 2. * r - pad, yt[i] + 2. * r + pad).ok()?;
    }
    for _ in 0..16 {
        let k = krawczyk(c, &y, &yt, &f0, &cm, x, cost)?;
        if interior(&k, &y) {
            return Some(Tube { y, yt, f0, cm });
        }
        for i in 0..N {
            let (a, b) = (lo(k[i]).min(lo(y[i])), hi(k[i]).max(hi(y[i])));
            let r = 0.5 * (b - a) + pad;
            y[i] = I::new(a - r, b + r).ok()?;
        }
    }
    None
}

/// The largest box `ỹ + 2ᵏ(Y − ỹ)` within the region that still contracts: the root is unique
/// in all of it, so the exclusion starts where the rows are bounded away from zero, not at the
/// tube's face, where an input box's width keeps every row's enclosure about zero.
fn grow<const N: usize>(c: &Component<N>, x: &[I], t: &Tube<N>, region: &[I; N], cost: &mut Cost)
    -> [I; N] {
    let mut best = t.y;
    'doubling: for _ in 0..30 {
        let mut next = best;
        for i in 0..N {
            let r = 2. * (hi(best[i]) - t.yt[i]).max(t.yt[i] - lo(best[i]));
            let (a, b) = ((t.yt[i] - r).max(lo(region[i])), (t.yt[i] + r).min(hi(region[i])));
            let Ok(side) = I::new(a, b) else { break 'doubling };
            next[i] = side;
        }
        if next == best {
            break;
        }
        match krawczyk(c, &next, &t.yt, &t.f0, &t.cm, x, cost) {
            Some(k) if interior(&k, &next) => best = next,
            _ => break,
        }
    }
    best
}

enum Exclusion<const N: usize> {
    Clear,
    Undecided([I; N]),
}

/// Show that no box of `region` outside `y` holds an admissible root for any input in `x`.
fn exclude<const N: usize>(
    c: &Component<N>, x: &[I], y: &[I; N], region: [I; N], cost: &mut Cost, budget: usize,
) -> Exclusion<N> {
    let scale = extent(&region);
    let mut stack = vec![region];
    'boxes: while let Some(b) = stack.pop() {
        cost.unknown_boxes += 1;
        if cost.evals > budget {
            return Exclusion::Undecided(b);
        }
        if (0..N).all(|i| lo(y[i]) <= lo(b[i]) && hi(b[i]) <= hi(y[i])) {
            continue; // inside the tube: its one root is the verified one
        }
        // Cut at a face of the tube, so each piece lies inside it or outside its interior.
        for i in 0..N {
            for face in [lo(y[i]), hi(y[i])] {
                if lo(b[i]) < face && face < hi(b[i]) {
                    stack.extend(halves(&b, i, face));
                    continue 'boxes;
                }
            }
        }
        let r = eval(c, &b, x, cost);
        if r.iter().any(|ri| ri.v.0.is_some_and(|v| lo(v) > 0. || hi(v) < 0.)) {
            continue;
        }
        if signs(c, &b, x).iter().any(|s| s.0.is_some_and(|s| hi(s) <= 0.)) {
            continue;
        }
        if let Some(cm) = mid_matrix(&r).and_then(inverse) {
            let ym: [f64; N] = from_fn(|i| b[i].mid());
            let f0 = at_centre(c, &ym, x, cost);
            if krawczyk(c, &b, &ym, &f0, &cm, x, cost).is_some_and(|k| apart(&k, &b)) {
                continue;
            }
        }
        let i = widest(&b);
        if width(b[i]) < 1e-7 * scale {
            return Exclusion::Undecided(b);
        }
        stack.extend(halves(&b, i, b[i].mid()));
    }
    Exclusion::Clear
}

/// A second admissible root at the input's middle, near `b` and outside the tube `y`, proved by
/// a tube of its own: what turns "would not go" into "two roots".
fn second_root<const N: usize>(
    c: &Component<N>, x: &[f64], b: &[I; N], y: &[I; N], cost: &mut Cost,
) -> Option<[f64; N]> {
    let xi = exact(x);
    let root = newton(c, x, from_fn(|i| b[i].mid()), cost)?;
    let t = tube(c, &xi, root, cost)?;
    (apart(&t.y, y) && admitted(c, &t.y, &xi)).then_some(root)
}

struct Report<const N: usize> {
    outcome: Outcome<N>,
    cost: Cost,
    seconds: f64,
}

fn verify<const N: usize>(c: &Component<N>, x: Vec<I>, budget: usize) -> Report<N> {
    let start = Instant::now();
    let mut cost = Cost::default();
    let outcome = run(c, x, budget, &mut cost);
    Report { outcome, cost, seconds: start.elapsed().as_secs_f64() }
}

fn run<const N: usize>(c: &Component<N>, x: Vec<I>, budget: usize, cost: &mut Cost)
    -> Outcome<N> {
    const DEPTH: usize = 60;
    let mut stack = vec![(x, 0)];
    while let Some((xb, depth)) = stack.pop() {
        if cost.evals > budget {
            return Outcome::NotVerified(format!("budget of {budget} evaluations spent"));
        }
        cost.input_boxes += 1;
        let xm: Vec<f64> = xb.iter().map(|v| v.mid()).collect();
        let split = |stack: &mut Vec<(Vec<I>, usize)>, why: &str| {
            if depth == DEPTH {
                return Err(format!("{why} at {xm:?}, the input box split {DEPTH} times"));
            }
            let i = widest(&xb);
            stack.extend(halves(&xb, i, xb[i].mid()).map(|h| (h, depth + 1)));
            Ok(())
        };
        let Some(yt) = newton(c, &xm, (c.seed)(&xm), cost) else {
            return Outcome::NotVerified(format!("no root from the seed at {xm:?}"));
        };
        if !admitted(c, &points(&yt), &exact(&xm)) {
            return Outcome::NotVerified(format!("the seed's root breaks a root choice at {xm:?}"));
        }
        let region = (c.region)(&xb);
        let y = match tube(c, &xb, yt, cost) {
            Some(t) if admitted(c, &t.y, &xb) => grow(c, &xb, &t, &region, cost),
            _ => {
                cost.no_tube += 1;
                match split(&mut stack, "no tube") {
                    Ok(()) => continue,
                    Err(why) => return Outcome::NotVerified(why),
                }
            }
        };
        match exclude(c, &xb, &y, region, cost, budget) {
            Exclusion::Clear => {}
            Exclusion::Undecided(b) => {
                cost.undecided += 1;
                if let Some(other) = second_root(c, &xm, &b, &y, cost) {
                    return Outcome::Refuted { x: xm, roots: [yt, other] };
                }
                if let Err(why) = split(&mut stack, "a box would not go") {
                    return Outcome::NotVerified(why);
                }
            }
        }
    }
    Outcome::Verified
}

/// `c ± r` over the input box: where a point on that circle can stand.
fn disc(c: I, r: I) -> I {
    (E::of(c) + E::of(r) * E::of(iv(-1., 1.))).0.unwrap()
}

// -- the components ---------------------------------------------------------------------------

/// A point at distance `ra` from `a` and `rb` from `b`, left of `a → b`: §13's first case.
/// Inputs `(ax, ay, bx, by, ra, rb)`, unknowns `p`.  Two roots, mirrored in the line of
/// centres; `ccw(a, b, p)` admits one.  Ω: `|ra − rb| < |a − b| < ra + rb`.
fn apex(ccw_chosen: bool) -> Component<2> {
    fn rows(y: &[D<2>; 2], x: &[D<2>]) -> [D<2>; 2] {
        let p = [y[0], y[1]];
        [on_circle(p, [x[0], x[1]], x[4]), on_circle(p, [x[2], x[3]], x[5])]
    }
    fn chosen(y: &[D<2>; 2], x: &[D<2>]) -> Vec<D<2>> {
        vec![ccw([x[0], x[1]], [x[2], x[3]], [y[0], y[1]])]
    }
    fn free(_: &[D<2>; 2], _: &[D<2>]) -> Vec<D<2>> {
        vec![]
    }
    Component {
        rows,
        signs: if ccw_chosen { chosen } else { free },
        // hint(at: a, toward: b, by: ra, turn: 60deg)
        seed: |x| {
            let (dx, dy) = (x[2] - x[0], x[3] - x[1]);
            let l = dx.hypot(dy);
            let (c, s) = (0.5, 0.75f64.sqrt());
            [x[0] + x[4] * (c * dx - s * dy) / l, x[1] + x[4] * (s * dx + c * dy) / l]
        },
        region: |x| [disc(x[0], x[4]), disc(x[1], x[4])],
    }
}

/// The circle through three points, its radius bounded positive (`r > 0`).  Inputs `(p1, p2,
/// p3)`, unknowns `(cx, cy, r)`; three `point_on_circle` rows, which hold at `±r`.  Its
/// region is stated, not read off its rows: the centre within ten times the points' spread of
/// them, the radius within twenty.  Ω: the points not collinear.
fn circumcircle(bounded: bool) -> Component<3> {
    fn rows(y: &[D<3>; 3], x: &[D<3>]) -> [D<3>; 3] {
        let c = [y[0], y[1]];
        from_fn(|i| on_circle([x[2 * i], x[2 * i + 1]], c, y[2]))
    }
    fn positive(y: &[D<3>; 3], _: &[D<3>]) -> Vec<D<3>> {
        vec![y[2]]
    }
    fn free(_: &[D<3>; 3], _: &[D<3>]) -> Vec<D<3>> {
        vec![]
    }
    Component {
        rows,
        signs: if bounded { positive } else { free },
        seed: |x| {
            let (cx, cy) = ((x[0] + x[2] + x[4]) / 3., (x[1] + x[3] + x[5]) / 3.);
            [cx, cy, (x[0] - cx).hypot(x[1] - cy)]
        },
        region: |x| {
            let (hx, hy) = (hull(&[x[0], x[2], x[4]]), hull(&[x[1], x[3], x[5]]));
            let l = width(hx).max(width(hy));
            let wide = |h: I| iv(lo(h) - 10. * l, hi(h) + 10. * l);
            [wide(hx), wide(hy), iv(-20. * l, 20. * l)]
        },
    }
}

/// The left external tangent from circle `a` to circle `b`, both on its right: the paper's
/// `CommonTangent` (§6.3).  Inputs `(ax, ay, ra, bx, by, rb)`, unknowns `(s, e)`, `s` on `a` and
/// `e` on `b`.  Three ways to say that the line `s → e` touches each circle there:
///
/// * `AtBothEnds`: the radius perpendicular to the line at each end, `TangentLineCircleAt`
///   (`tangent_arc_line`'s residual), the regular form the solver compiles a tangency at a
///   point to (solver.md).  All four tangents satisfy it; `ccw(a.center, b.center, s)` and
///   `… e)` choose one.
/// * `ParallelRadii`: the radius perpendicular to the line at `s`, and the two radii parallel
///   (`parallel`'s determinant); the same `ccw` choice.
/// * `OnAndTangent`: the centre at signed distance `−r` from the line (`TangentLineCircle`,
///   `k = −1`), which chooses the side in the row itself, beside `s` and `e` on their circles.
///   solver.md calls this pair a double root at every solution.
#[derive(Clone, Copy)]
enum Touch {
    AtBothEnds,
    ParallelRadii,
    OnAndTangent,
}

fn common_tangent(touch: Touch) -> Component<4> {
    fn on(y: &[D<4>; 4], x: &[D<4>]) -> ([D<4>; 2], [D<4>; 2], [D<4>; 2], [D<4>; 2]) {
        ([y[0], y[1]], [y[2], y[3]], [x[0], x[1]], [x[3], x[4]])
    }
    fn at_both_ends(y: &[D<4>; 4], x: &[D<4>]) -> [D<4>; 4] {
        let (s, e, a, b) = on(y, x);
        let (dx, dy) = (e[0] - s[0], e[1] - s[1]);
        [
            on_circle(s, a, x[2]),
            on_circle(e, b, x[5]),
            (s[0] - a[0]) * dx + (s[1] - a[1]) * dy,
            (e[0] - b[0]) * dx + (e[1] - b[1]) * dy,
        ]
    }
    fn parallel_radii(y: &[D<4>; 4], x: &[D<4>]) -> [D<4>; 4] {
        let (s, e, a, b) = on(y, x);
        let (dx, dy) = (e[0] - s[0], e[1] - s[1]);
        [
            on_circle(s, a, x[2]),
            on_circle(e, b, x[5]),
            (s[0] - a[0]) * dx + (s[1] - a[1]) * dy,
            (s[0] - a[0]) * (e[1] - b[1]) - (s[1] - a[1]) * (e[0] - b[0]),
        ]
    }
    fn on_and_tangent(y: &[D<4>; 4], x: &[D<4>]) -> [D<4>; 4] {
        let (s, e, a, b) = on(y, x);
        [
            on_circle(s, a, x[2]),
            on_circle(e, b, x[5]),
            tangent(s, e, a, x[2], -1.),
            tangent(s, e, b, x[5], -1.),
        ]
    }
    fn both_left(y: &[D<4>; 4], x: &[D<4>]) -> Vec<D<4>> {
        let (s, e, a, b) = on(y, x);
        vec![ccw(a, b, s), ccw(a, b, e)]
    }
    fn none(_: &[D<4>; 4], _: &[D<4>]) -> Vec<D<4>> {
        vec![]
    }
    let (rows, signs): (Rows<4>, Signs<4>) = match touch {
        Touch::AtBothEnds => (at_both_ends, both_left),
        Touch::ParallelRadii => (parallel_radii, both_left),
        Touch::OnAndTangent => (on_and_tangent, none),
    };
    Component {
        rows,
        signs,
        // hint(at: a.center, toward: b.center, by: a.r, turn: 90deg), and the same at b
        seed: |x| {
            let (dx, dy) = (x[3] - x[0], x[4] - x[1]);
            let l = dx.hypot(dy);
            let (nx, ny) = (-dy / l, dx / l);
            [x[0] + x[2] * nx, x[1] + x[2] * ny, x[3] + x[5] * nx, x[4] + x[5] * ny]
        },
        region: |x| [disc(x[0], x[2]), disc(x[1], x[2]), disc(x[3], x[5]), disc(x[4], x[5])],
    }
}

/// The common perpendicular of two lines in space, `p = a₁ + s·u₁` and `q = a₂ + t·u₂` with
/// `(q − p)·u₁ = (q − p)·u₂ = 0`.  Inputs `(a₁, b₁, a₂, b₂)` (twelve), unknowns `(s, t)`.  The rows
/// are affine in the unknowns, so the tube's test is a test of the interval Gram matrix alone,
/// and holds for a tube of any size.  Ω: the lines not parallel.
fn common_perpendicular() -> Component<2> {
    fn dot(a: [D<2>; 3], b: [D<2>; 3]) -> D<2> {
        a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
    }
    Component {
        rows: |y, x| {
            let u1: [D<2>; 3] = from_fn(|i| x[3 + i] - x[i]);
            let u2: [D<2>; 3] = from_fn(|i| x[9 + i] - x[6 + i]);
            let gap = from_fn(|i| x[6 + i] + y[1] * u2[i] - x[i] - y[0] * u1[i]);
            [dot(gap, u1), dot(gap, u2)]
        },
        signs: |_, _| vec![],
        seed: |_| [0.5, 0.5],
        region: |_| [iv(-1e3, 1e3), iv(-1e3, 1e3)],
    }
}

// -- the spike's cases ------------------------------------------------------------------------
//
// Each box is over the component's *shape*, not its placement: the first operand at the origin
// and the second on the x axis, the rest free (EQ1: a proper rigid motion carries the rows, the
// root choices and the region, so a verification in the canonical placement holds wherever the
// operands stand).  `the_placement_is_what_costs` shows why: the same box in all six inputs.

const BUDGET: usize = 2_000_000;

/// `x`, each input listed in `free` widened by `w` either side, the rest exact.
fn shape(x: &[f64], w: f64, free: &[usize]) -> Vec<I> {
    x.iter().enumerate().map(|(i, &v)| around(v, if free.contains(&i) { w } else { 0. })).collect()
}

/// Two operands `a`, `b` at `x[0..2]`, `x[3..5]` with one number after each (`apex`'s `ra`,
/// `rb` come last instead): the free part is the centre distance and the two numbers.
const APEX: [usize; 3] = [2, 4, 5];
const PAIR: [usize; 3] = [2, 3, 5];
/// `p1` at the origin, `p2` on the x axis: `|p1 p2|` and `p3` free.
const TRIANGLE: [usize; 3] = [2, 4, 5];

fn report<const N: usize>(name: &str, r: &Report<N>) {
    eprintln!(
        "{name:56} {:>12} {:>6} input boxes ({} no tube, {} undecided) {:>8} unknown boxes \
         {:>9} evals {:>9.2} ms",
        match &r.outcome {
            Outcome::Verified => "verified",
            Outcome::NotVerified(_) => "not verified",
            Outcome::Refuted { .. } => "refuted",
        },
        r.cost.input_boxes,
        r.cost.no_tube,
        r.cost.undecided,
        r.cost.unknown_boxes,
        r.cost.evals,
        1e3 * r.seconds
    );
    if let Outcome::NotVerified(why) = &r.outcome {
        eprintln!("    {why}");
    }
}

#[test]
fn the_apex_chosen_by_ccw_is_verified_unique_over_a_box() {
    // §13: `x distance(5) a`, `x distance(5) b`, `a`, `b` six apart; the distance and both
    // lengths free by 0.5.
    let r = verify(&apex(true), shape(&[0., 0., 6., 0., 5., 5.], 0.5, &APEX), BUDGET);
    report("apex, ccw, ±0.5 about |a − b| = 6, ra = rb = 5", &r);
    assert!(matches!(r.outcome, Outcome::Verified), "{:?}", r.outcome);
}

#[test]
fn the_apex_without_its_choice_is_refuted_with_both_roots() {
    let r = verify(&apex(false), shape(&[0., 0., 6., 0., 5., 5.], 0.5, &APEX), BUDGET);
    report("apex, no choice", &r);
    let Outcome::Refuted { x, roots } = r.outcome else { panic!("{:?}", r.outcome) };
    // one each side of the line of centres, each at its two distances
    let side = |p: [f64; 2]| (x[2] - x[0]) * (p[1] - x[1]) - (x[3] - x[1]) * (p[0] - x[0]);
    assert!(side(roots[0]) * side(roots[1]) < 0., "{roots:?}");
    for p in roots {
        assert!(((p[0] - x[0]).hypot(p[1] - x[1]) - x[4]).abs() < 1e-9, "{p:?}");
        assert!(((p[0] - x[2]).hypot(p[1] - x[3]) - x[5]).abs() < 1e-9, "{p:?}");
    }
}

#[test]
fn the_apex_outside_its_domain_is_not_verified() {
    // |a − b| = 12 > 10: no root at all.  Not verified, and nothing returned.
    let r = verify(&apex(true), shape(&[0., 0., 12., 0., 5., 5.], 0.1, &APEX), BUDGET);
    assert!(matches!(r.outcome, Outcome::NotVerified(_)), "{:?}", r.outcome);
}

#[test]
fn the_cost_grows_toward_the_fold() {
    // `b` slides along x over [6, 10 − g]: at 10 the two roots meet on the line of centres.
    // Every box short of the fold verifies; the work grows as the box nears it.
    let mut last = 0;
    for g in [1., 0.1, 0.01, 0.001, 1e-6] {
        let mut x = shape(&[0., 0., 0., 0., 5., 5.], 0., &[]);
        x[2] = iv(6., 10. - g);
        let r = verify(&apex(true), x, BUDGET);
        report(&format!("apex, ccw, |a − b| over [6, 10 − {g}]"), &r);
        assert!(matches!(r.outcome, Outcome::Verified), "{g}: {:?}", r.outcome);
        assert!(r.cost.input_boxes >= last, "{g}");
        last = r.cost.input_boxes;
    }
    // and a box reaching the fold is never verified
    let mut x = shape(&[0., 0., 0., 0., 5., 5.], 0., &[]);
    x[2] = iv(6., 10.);
    let r = verify(&apex(true), x, 200_000);
    report("apex, ccw, |a − b| over [6, 10]", &r);
    assert!(matches!(r.outcome, Outcome::NotVerified(_)), "{:?}", r.outcome);
}

#[test]
fn the_placement_is_what_costs() {
    // The same ±w in the shape's three numbers, and in all six inputs: the same shapes, placed
    // every way.  Covering a box by boxes small enough for the tube costs a power of its
    // dimension, so the three placement coordinates multiply the work many times over.
    let x = [0., 0., 6., 0., 5., 5.];
    for w in [0.2, 0.3] {
        let r = verify(&apex(true), shape(&x, w, &APEX), BUDGET);
        report(&format!("apex, ccw, ±{w} in the shape"), &r);
        assert!(matches!(r.outcome, Outcome::Verified), "{:?}", r.outcome);
        let placed = verify(&apex(true), shape(&x, w, &[0, 1, 2, 3, 4, 5]), 300_000);
        report(&format!("apex, ccw, ±{w} in every input"), &placed);
        assert!(matches!(placed.outcome, Outcome::Verified), "{:?}", placed.outcome);
        assert!(placed.cost.evals > 50 * r.cost.evals, "{w}");
    }
}

#[test]
fn the_circle_through_three_points_is_verified_with_its_radius_bounded() {
    let x = [0., 0., 10., 0., 3., 7.];
    let r = verify(&circumcircle(true), shape(&x, 0.25, &TRIANGLE), BUDGET);
    report("circumcircle, r > 0, ±0.25", &r);
    assert!(matches!(r.outcome, Outcome::Verified), "{:?}", r.outcome);
    let r = verify(&circumcircle(false), shape(&x, 0.25, &TRIANGLE), BUDGET);
    report("circumcircle, r free", &r);
    let Outcome::Refuted { roots: [a, b], .. } = r.outcome else { panic!("{:?}", r.outcome) };
    assert!((a[2] + b[2]).abs() < 1e-9 && (a[0] - b[0]).abs() < 1e-9, "{a:?} {b:?}");
}

#[test]
fn the_circle_through_nearly_collinear_points_is_not_verified_within_a_budget() {
    // the third point `h` off the line of the other two, each of the shape's numbers free by
    // ±0.05: as `h` falls, `F_y` loses conditioning, the tube needs ever smaller input boxes
    // and the exclusion ever smaller boxes along the valleys where two rows vanish.  At h = 3
    // one box does; at 0.5 the verifier gives up, as it should, rather than guess.
    let x = [0., 0., 10., 0., 5., 3.];
    let r = verify(&circumcircle(true), shape(&x, 0.05, &TRIANGLE), BUDGET);
    report("circumcircle, r > 0, h = 3, ±0.05", &r);
    assert!(matches!(r.outcome, Outcome::Verified), "{:?}", r.outcome);
    let x = [0., 0., 10., 0., 5., 0.5];
    let r = verify(&circumcircle(true), shape(&x, 0.05, &TRIANGLE), 100_000);
    report("circumcircle, r > 0, h = 0.5, ±0.05", &r);
    assert!(matches!(r.outcome, Outcome::NotVerified(_)), "{:?}", r.outcome);
}

#[test]
fn the_common_tangent_at_both_ends_is_verified_where_the_circles_clear() {
    let x = [0., 0., 3., 10., 0., 2.];
    let r = verify(&common_tangent(Touch::AtBothEnds), shape(&x, 0.25, &PAIR), BUDGET);
    report("tangent, at both ends + ccw, clear (d = 10), ±0.25", &r);
    assert!(matches!(r.outcome, Outcome::Verified), "{:?}", r.outcome);
}

#[test]
fn the_common_tangent_at_both_ends_is_refuted_where_the_circles_cross() {
    // d = 4: inside the paper's Ω (|ra − rb| = 1 < d) but the circles cross, and `s = e` at the
    // crossing left of the centres satisfies every row and both `ccw`: a second, regular root.
    let x = [0., 0., 3., 4., 0., 2.];
    let r = verify(&common_tangent(Touch::AtBothEnds), shape(&x, 0.05, &PAIR), BUDGET);
    report("tangent, at both ends + ccw, crossing (d = 4)", &r);
    let Outcome::Refuted { x, roots: [_, o] } = r.outcome else { panic!("{:?}", r.outcome) };
    let ((ax, ay, ra), (bx, by, rb)) = ((x[0], x[1], x[2]), (x[3], x[4], x[5]));
    assert!((o[0] - o[2]).abs() < 1e-9 && (o[1] - o[3]).abs() < 1e-9, "{o:?}");
    assert!(((o[0] - ax).hypot(o[1] - ay) - ra).abs() < 1e-9, "{o:?}");
    assert!(((o[0] - bx).hypot(o[1] - by) - rb).abs() < 1e-9, "{o:?}");
}

#[test]
fn the_common_tangent_by_parallel_radii_is_verified_where_the_circles_cross() {
    for (d, w) in [(10., 0.25), (4., 0.05), (1.5, 0.02)] {
        let x = [0., 0., 3., d, 0., 2.];
        let r = verify(&common_tangent(Touch::ParallelRadii), shape(&x, w, &PAIR), BUDGET);
        report(&format!("tangent, parallel radii + ccw, d = {d}, ±{w}"), &r);
        assert!(matches!(r.outcome, Outcome::Verified), "{d}: {:?}", r.outcome);
    }
}

#[test]
fn the_common_tangent_on_and_tangent_is_never_verified() {
    // the double root solver.md names: `s` slides along the line to first order, so `F_y` is
    // singular at the root and no tube contracts.
    let x = [0., 0., 3., 10., 0., 2.];
    let r = verify(&common_tangent(Touch::OnAndTangent), shape(&x, 0.01, &PAIR), 200_000);
    report("tangent, on + signed tangent (a double root)", &r);
    assert!(matches!(r.outcome, Outcome::NotVerified(_)), "{:?}", r.outcome);
}

#[test]
fn the_common_perpendicular_is_verified_by_its_gram_matrix() {
    // affine in its unknowns: one tube over the whole region, so no exclusion is left, and the
    // box is over every input, placement and all
    let x = [0., 0., 0., 10., 0., 0., 3., -5., 4., 3., 5., 2.];
    let r = verify(&common_perpendicular(), shape(&x, 0.25, &(0..12).collect::<Vec<_>>()), BUDGET);
    report("common perpendicular, ±0.25 in all twelve", &r);
    assert!(matches!(r.outcome, Outcome::Verified), "{:?}", r.outcome);
    assert_eq!(r.cost.input_boxes, 1);
}

/// The admissible roots found at one input from `starts` scattered starts in the region: the
/// DT2 refutation of the contract, which finds a second root or finds none, and proves nothing.
fn scattered<const N: usize>(c: &Component<N>, x: &[f64], starts: usize, rng: &mut Rng)
    -> Vec<[f64; N]> {
    let xi = exact(x);
    let region = (c.region)(&xi);
    let scale = extent(&region);
    let mut cost = Cost::default();
    let mut found: Vec<[f64; N]> = vec![];
    for _ in 0..starts {
        let seed = from_fn(|i| rng.uniform(lo(region[i]), hi(region[i])));
        let Some(y) = newton(c, x, seed, &mut cost) else { continue };
        let inside = (0..N).all(|i| region[i].contains(y[i]));
        if inside && admitted(c, &points(&y), &xi)
            && !found.iter().any(|f| (0..N).all(|i| (f[i] - y[i]).abs() < 1e-7 * scale)) {
            found.push(y);
        }
    }
    found
}

#[test]
fn sampling_agrees_with_every_verdict() {
    // The verdicts above, checked by the contract's sampled refutation (DT3's "held by"): in a
    // verified box no sample finds a second admissible root, and in a refuted one some does.
    let mut rng = Rng::new(453);
    let draw = |x: &[I], rng: &mut Rng| -> Vec<f64> {
        x.iter().map(|v| rng.uniform(lo(*v), hi(*v))).collect()
    };
    let clear = shape(&[0., 0., 3., 10., 0., 2.], 0.25, &PAIR);
    let crossing = shape(&[0., 0., 3., 4., 0., 2.], 0.05, &PAIR);
    for _ in 0..20 {
        for t in [Touch::AtBothEnds, Touch::ParallelRadii] {
            let x = draw(&clear, &mut rng);
            assert_eq!(scattered(&common_tangent(t), &x, 40, &mut rng).len(), 1, "{x:?}");
        }
        let x = draw(&crossing, &mut rng);
        assert_eq!(scattered(&common_tangent(Touch::ParallelRadii), &x, 40, &mut rng).len(), 1);
        let x = draw(&shape(&[0., 0., 6., 0., 5., 5.], 0.5, &APEX), &mut rng);
        assert_eq!(scattered(&apex(true), &x, 40, &mut rng).len(), 1, "{x:?}");
        assert_eq!(scattered(&apex(false), &x, 40, &mut rng).len(), 2, "{x:?}");
    }
    let refuted = (0..20).any(|_| {
        let x = draw(&crossing, &mut rng);
        scattered(&common_tangent(Touch::AtBothEnds), &x, 40, &mut rng).len() == 2
    });
    assert!(refuted, "no sample found the crossing's second root");
}
