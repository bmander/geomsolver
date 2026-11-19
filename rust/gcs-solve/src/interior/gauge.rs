//! **What an integral row reads of its structure** (§6.25, #953): each integral of its equation
//! (`Part`), a body over a point of the structure's solid elements or of facets of its boundary
//! — a tape over the point's three numbers, the density there and what the body reads of the
//! drawing besides (`Reading`'s numbers) — and the equation's two sides over the integrals and the
//! unknowns the row reads (`Gauge::lhs`, `Gauge::rhs`).  A row with no part reads one integral, the
//! structure's whole (`Structure::measure`): a beam's, a prism's, a body's volume or mass.
//!
//! Each part is taken by the structure's own quadrature: a quadratic tetrahedron's degree-five
//! rule, a facet's collapsed Gauss rule of eight points a side (exact to degree fourteen in its
//! parameters).  A body polynomial in the point of degree two or less — a volume, a first moment,
//! a second — is so read exactly on a conforming mesh of curved elements, through the ten moments
//! of where it is taken (`Moments`, in each run's own frame); any other body is read to the rule's
//! order, and the error a claim is told is the change from the
//! mesh twice as coarse.  A body's nodes inside stand where they were meshed
//! (`Moved::boundary`'s rule): an integral of a body reads its boundary.

use super::solid::P;
use super::Structure;
use crate::kernels::{Dual, Num};
use crate::model::POINT_VARS as POINT;
use crate::tape::{eval_flat, Scratch, Value};

/// **Where an integral is taken**: over solid elements, each run `(first, count)` of the
/// structure's (a body's, or each of an instance's bodies'), or over facets of the boundary, each
/// its six nodes (`[a, b, c, ab, bc, ca]`, a quadratic triangle).
#[derive(Clone, Debug, PartialEq)]
pub enum Domain {
    Tets(Vec<(usize, usize)>),
    Facets(Vec<[usize; 6]>),
}

/// **One integral of a row**: where, its body as a tape's flat instructions over `[x, y, z, ρ,
/// r…]` (`model::integrand_var`), and whether that body is a polynomial of degree two or less in
/// its point (`quadratic`) — read then through the ten moments of where it is taken, exactly
/// (`Moments`), its coefficients its tape's at ten points (`coefficients`).
#[derive(Clone, Debug, PartialEq)]
pub struct Part {
    pub domain: Domain,
    pub integrand: Vec<f64>,
    pub quadratic: bool,
}

/// The ten moments' order: `∫ 1`, `∫ x`, `∫ y`, `∫ z`, then `∫ x x`, `y y`, `z z`, `x y`, `x z`,
/// `y z`.
pub const MOMENTS: usize = 10;

/// The ten monomials at a point `x`, `dv` its measure there, added into `out`.
fn moments_at<T: Num>(x: P<T>, dv: T, out: &mut [T; MOMENTS]) {
    let f = [T::cst(1.0), x[0], x[1], x[2], x[0] * x[0], x[1] * x[1], x[2] * x[2], x[0] * x[1],
             x[0] * x[2], x[1] * x[2]];
    for (o, f) in out.iter_mut().zip(f) {
        *o = *o + f * dv;
    }
}

/// **Where a run's moments are taken about**: its centre `c` and its size `h`, the monomials read
/// in `t = (x − c) / h` — so a body's coefficients are drawn from its values a run's size apart,
/// and its moments are of numbers near one, each well conditioned however far from the origin
/// the run stands.  Any centre and size give the same integral; these are the run's own.
#[derive(Clone, Copy, Debug)]
struct Frame {
    c: [f64; 3],
    h: f64,
}

impl Frame {
    /// `x` in the frame's numbers.
    fn local<T: Num>(&self, x: P<T>) -> P<T> {
        std::array::from_fn(|k| (x[k] - T::cst(self.c[k])) * T::cst(1.0 / self.h))
    }

    /// The frame of points `ps`: their mean, and their farthest from it along an axis.
    fn of(ps: impl Iterator<Item = [f64; 3]> + Clone) -> Frame {
        let (mut c, mut n) = ([0.0; 3], 0.0);
        for p in ps.clone() {
            for k in 0..3 {
                c[k] += p[k];
            }
            n += 1.0;
        }
        let c = if n > 0.0 { c.map(|v| v / n) } else { c };
        let h = ps.flat_map(|p| (0..3).map(move |k| (p[k] - c[k]).abs())).fold(0.0, f64::max);
        Frame { c, h: if h > 0.0 && h.is_finite() { h } else { 1.0 } }
    }
}

/// **A body quadratic in its point, its ten coefficients in a frame's numbers** at density
/// `rho`, reading `r`: what multiplies each monomial of `t` (`moments_at`'s order), each with its
/// rate in the body's variables — from its tape at the frame's centre, a step of its size along
/// each axis and back, and each pair of steps, exact for a quadratic.
fn coefficients(flat: &[f64], n: usize, rho: f64, r: &[f64], frame: &Frame, s: &mut Scratch)
    -> [Value; MOMENTS]
{
    let mut f = |t: [f64; 3]| {
        body(flat, n, std::array::from_fn(|k| frame.c[k] + frame.h * t[k]), rho, r, s)
    };
    let e = |c: usize, k: f64| {
        let mut p = [0.0; 3];
        p[c] = k;
        p
    };
    let f0 = f([0.0; 3]);
    let plus: [Value; 3] = std::array::from_fn(|c| f(e(c, 1.0)));
    let minus: [Value; 3] = std::array::from_fn(|c| f(e(c, -1.0)));
    let pairs = [(0, 1), (0, 2), (1, 2)];
    let both: [Value; 3] = pairs.map(|(c, d)| {
        let mut p = e(c, 1.0);
        p[d] = 1.0;
        f(p)
    });
    // each coefficient a combination of the values, its rate the same of their rates
    let comb = |terms: &[(f64, &Value)]| {
        let mut out = Value::default();
        for (k, v) in terms {
            out.v += k * v.v;
            for (o, d) in out.d.iter_mut().zip(&v.d) {
                *o += k * d;
            }
        }
        out
    };
    let linear: [Value; 3] = std::array::from_fn(|c| comb(&[(0.5, &plus[c]), (-0.5, &minus[c])]));
    let square: [Value; 3] =
        std::array::from_fn(|c| comb(&[(0.5, &plus[c]), (0.5, &minus[c]), (-1.0, &f0)]));
    let cross: [Value; 3] = std::array::from_fn(|k| {
        let (c, d) = pairs[k];
        comb(&[(1.0, &both[k]), (-1.0, &f0), (-1.0, &linear[c]), (-1.0, &linear[d]),
               (-1.0, &square[c]), (-1.0, &square[d])])
    });
    [f0, linear[0], linear[1], linear[2], square[0], square[1], square[2], cross[0], cross[1],
     cross[2]]
}

/// **What an integral row reads**: its integrals (none: the structure's whole), its sides as
/// tapes over `[I…, u…]` — the integrals, then the `unknowns` its last columns are — and how many
/// numbers its parts' bodies read of the drawing (`reads`, the hull's readings').
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Gauge {
    pub parts: Vec<Part>,
    pub lhs: Vec<f64>,
    pub rhs: Vec<f64>,
    pub unknowns: usize,
    pub reads: usize,
}


impl Gauge {
    /// How many integrals its sides read: its parts', or the whole's one.
    pub fn integrals(&self) -> usize {
        self.parts.len().max(1)
    }

    /// Whether it reads the structure's whole alone.
    pub fn whole(&self) -> bool {
        self.parts.is_empty()
    }

    /// How many variables a part's body reads.
    pub fn integrand_vars(&self) -> usize {
        POINT + self.reads
    }

    /// **Its two sides at integrals `ints` and unknowns `u`**, each with its rate in them.
    pub fn sides(&self, ints: &[f64], u: &[f64]) -> (Value, Value) {
        eval_sides(&self.lhs, &self.rhs, ints, u)
    }
}

/// **Two sides' tapes at integrals `ints` and unknowns `u`**, each with its rate in them: what a
/// row reads, and a claim as its row would.
pub fn eval_sides(lhs: &[f64], rhs: &[f64], ints: &[f64], u: &[f64]) -> (Value, Value) {
    let mut vars = [0.0; crate::tape::MAX_VARS];
    let n = (ints.len() + u.len()).min(vars.len());
    for (v, x) in vars.iter_mut().zip(ints.iter().chain(u)) {
        *v = *x;
    }
    let mut s = Scratch::new();
    (eval_flat(lhs, n, &vars[..n], &mut s), eval_flat(rhs, n, &vars[..n], &mut s))
}

/// The body of `flat` over `n` variables at a point `p` of density `rho`, reading `r`.
fn body(flat: &[f64], n: usize, p: [f64; 3], rho: f64, r: &[f64], s: &mut Scratch) -> Value {
    let mut vars = [0.0; crate::tape::MAX_VARS];
    vars[..3].copy_from_slice(&p);
    vars[3] = rho;
    vars[POINT..POINT + r.len()].copy_from_slice(r);
    eval_flat(flat, n, &vars[..n], s)
}

/// **A facet's points**, at its six nodes' places: each point of the collapsed Gauss rule, its
/// place and its measure there, `|x_u × x_v|` times its weight.
fn each_facet_point<T: Num>(x: &[P<T>; 6], mut f: impl FnMut(P<T>, T)) {
    for &(s, t, w) in triangle_rule() {
        let n = super::plane::shape(s, t);
        let d = super::plane::shape_d(s, t);
        let mut p = [T::cst(0.0); 3];
        let mut ts = [[T::cst(0.0); 3]; 2];
        for k in 0..6 {
            for c in 0..3 {
                p[c] = p[c] + T::cst(n[k]) * x[k][c];
                for (r, tr) in ts.iter_mut().enumerate() {
                    tr[c] = tr[c] + T::cst(d[r][k]) * x[k][c];
                }
            }
        }
        let [a, b] = ts;
        let nda = [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
        f(p, (nda[0] * nda[0] + nda[1] * nda[1] + nda[2] * nda[2]).sqrt() * T::cst(w));
    }
}

/// The collapsed (Duffy) product of Gauss–Legendre's eight points on `[0, 1]` a side
/// (`integral::GAUSS8`) over the triangle `(0,0), (1,0), (0,1)`: `s = u`, `t = v (1 − u)`, weighed
/// by `1 − u` — exact to degree fourteen; its weights sum to the triangle's area, ½.
fn triangle_rule() -> &'static [(f64, f64, f64)] {
    static RULE: std::sync::OnceLock<Vec<(f64, f64, f64)>> = std::sync::OnceLock::new();
    RULE.get_or_init(|| {
        let unit: Vec<(f64, f64)> =
            crate::integral::GAUSS8.iter().map(|&(x, w)| ((1.0 + x) / 2.0, w / 2.0)).collect();
        let mut out = Vec::with_capacity(unit.len() * unit.len());
        for &(u, wu) in &unit {
            for &(v, wv) in &unit {
                out.push((u, v * (1.0 - u), wu * wv * (1.0 - u)));
            }
        }
        out
    })
}

/// **A run's moments**: its density, the frame they are taken in, and the ten.
#[derive(Clone, Copy, Debug)]
pub(super) struct Moments {
    rho: f64,
    frame: Frame,
    m: [f64; MOMENTS],
}

/// **Where a quadratic part's moments are taken, one density at a time**: a run of solid
/// elements of one body, or facets (which weigh nothing).
enum Run<'a> {
    Tets(usize, usize),
    Facets(&'a [[usize; 6]]),
}

/// **The moments of each domain a quadratic part is over**, each domain once: the first part over
/// it, and its runs' moments — taken once where a row stands, read by its value and its rate.
pub(super) type Taken = Vec<(usize, Vec<Moments>)>;

/// A facet's rate widened to an element's thirty places, its six nodes the first eighteen.
fn wide(d: Dual<18>) -> Dual<30> {
    let mut g = [0.0; 30];
    g[..18].copy_from_slice(&d.g);
    Dual { v: d.v, g }
}

impl Structure {
    /// A domain's runs, each of one density.
    fn domain_runs<'a>(&self, d: &'a Domain) -> Vec<Run<'a>> {
        match d {
            Domain::Tets(runs) => runs.iter().map(|&(a, k)| Run::Tets(a, k)).collect(),
            Domain::Facets(f) => vec![Run::Facets(f)],
        }
    }

    /// **The moments of each domain a quadratic part is over**, each domain once, at `x`.
    fn quadratic_moments(&self, x: &[f64]) -> Taken {
        let parts = &self.gauge.parts;
        let mut out: Taken = Vec::new();
        for (j, part) in parts.iter().enumerate().filter(|(_, p)| p.quadratic) {
            if !out.iter().any(|(k, _)| parts[*k].domain == part.domain) {
                out.push((j, self.domain_moments(&part.domain, x)));
            }
        }
        out
    }

    /// The moments `taken` holds of part `part`'s domain.
    fn moments_of<'t>(&self, taken: &'t Taken, part: &Part) -> &'t [Moments] {
        let parts = &self.gauge.parts;
        &taken.iter().find(|(k, _)| parts[*k].domain == part.domain).expect("its moments").1
    }

    /// A run's frame at `x` (`Frame::of`): over its elements' or its facets' corners.
    fn frame(&self, run: &Run, x: &[f64]) -> Frame {
        let at = |n: usize| [x[3 * n], x[3 * n + 1], x[3 * n + 2]];
        match run {
            Run::Tets(a, k) => Frame::of(self.tets[*a..a + k].iter()
                .flat_map(|t| t.n[..4].iter().copied()).map(at)),
            Run::Facets(f) => Frame::of(f.iter().flat_map(|f| f[..3].iter().copied()).map(at)),
        }
    }

    /// **Each run's density, frame and ten moments at `x`** (`MOMENTS`, in the frame's numbers):
    /// a body's elements' by the degree-five rule, exact for its volume and first moments and its
    /// second within the rule's order; a face's facets' by the collapsed Gauss rule, exact for a
    /// plane facet's second.
    fn domain_moments(&self, d: &Domain, x: &[f64]) -> Vec<Moments> {
        self.domain_runs(d).into_iter().map(|run| {
            let frame = self.frame(&run, x);
            let mut m = [Dual::<0>::cst(0.0); MOMENTS];
            let rho = match &run {
                Run::Tets(a, k) => {
                    for i in *a..a + k {
                        let places = self.tet_places_d::<0>(i, x);
                        super::solid::each_point(&places, |p, w, det| {
                            moments_at(frame.local(p), det * Dual::cst(w), &mut m)
                        });
                    }
                    self.tets.get(*a).map_or(0.0, |t| t.rho)
                }
                Run::Facets(f) => {
                    for f in *f {
                        each_facet_point(&self.facet_places::<0>(f, x),
                                         |p, da| moments_at(frame.local(p), da, &mut m));
                    }
                    0.0
                }
            };
            Moments { rho, frame, m: m.map(|v| v.v) }
        }).collect()
    }

    /// **`Σ_k w_k ∂m_k/∂x` of a run's moments in `frame` at `x`, added into `g`**: each facet's
    /// or element's weighed moments, one number in `Dual` over its nodes' places, the frame held.
    fn moments_gradient(&self, run: &Run, frame: &Frame, x: &[f64], w: &[f64; MOMENTS],
                        g: &mut [f64]) {
        // the monomials weighed at a point, times its measure
        fn weighed<T: Num>(x: P<T>, dv: T, w: &[f64; MOMENTS]) -> T {
            let k = |j: usize| T::cst(w[j]);
            let lin = k(0) + k(1) * x[0] + k(2) * x[1] + k(3) * x[2];
            let quad = x[0] * (k(4) * x[0] + k(7) * x[1] + k(8) * x[2])
                + x[1] * (k(5) * x[1] + k(9) * x[2]) + k(6) * x[2] * x[2];
            (lin + quad) * dv
        }
        let add = |m: &Dual<30>, nodes: &[usize], g: &mut [f64]| {
            for (k, &node) in nodes.iter().enumerate() {
                for c in 0..3 {
                    g[3 * node + c] += m.g[3 * k + c];
                }
            }
        };
        match run {
            Run::Tets(a, k) => {
                for i in *a..a + k {
                    let mut m = Dual::<30>::cst(0.0);
                    super::solid::each_point(&self.tet_places_d::<30>(i, x), |p, wq, det| {
                        m = m + weighed(frame.local(p), det * Dual::cst(wq), w)
                    });
                    add(&m, &self.tets[i].n, g);
                }
            }
            Run::Facets(f) => {
                for f in *f {
                    let mut m = Dual::<18>::cst(0.0);
                    each_facet_point(&self.facet_places::<18>(f, x),
                                     |p, da| m = m + weighed(frame.local(p), da, w));
                    add(&wide(m), f, g);
                }
            }
        }
    }

    /// A facet's six nodes' places at `x`, over a number that may carry their rates (`N` = 18).
    fn facet_places<const N: usize>(&self, f: &[usize; 6], x: &[f64]) -> [P<Dual<N>>; 6] {
        std::array::from_fn(|k| {
            let at = [x[3 * f[k]], x[3 * f[k] + 1], x[3 * f[k] + 2]];
            if N == 0 { at.map(Dual::cst) } else { super::solid::var(at, 3 * k) }
        })
    }

    /// **Each part's integral at `x`**, its body reading `r`: a quadratic body's through its
    /// domain's moments, any other's point by point.
    pub fn integrals(&self, x: &[f64], r: &[f64]) -> Vec<f64> {
        self.integrals_taken(x, r).0
    }

    /// `integrals`, and the moments taken for them, which the rate reads again (`Taken`).
    pub(super) fn integrals_taken(&self, x: &[f64], r: &[f64]) -> (Vec<f64>, Taken) {
        let n = self.gauge.integrand_vars();
        let mut s = Scratch::new();
        let taken = self.quadratic_moments(x);
        let ints = self.gauge.parts.iter().map(|part| {
            if !part.quadratic {
                return self.pointwise(part, x, r, |f| f);
            }
            self.moments_of(&taken, part).iter().map(|run| {
                let a = coefficients(&part.integrand, n, run.rho, r, &run.frame, &mut s);
                a.iter().zip(&run.m).map(|(a, m)| a.v * m).sum::<f64>()
            }).sum()
        }).collect();
        (ints, taken)
    }

    /// **Each part's body's size integrated**, `∫ |f|`, at `x` reading `r`, point by point —
    /// what a claim's rounding is measured against, a polynomial's too (its moments may cancel
    /// where its values do not).
    pub fn integral_sizes(&self, x: &[f64], r: &[f64]) -> Vec<f64> {
        self.gauge.parts.iter().map(|part| self.pointwise(part, x, r, f64::abs)).collect()
    }

    /// `∫ g(f)` of part `part`'s body `f` at `x`, reading `r`, by its domain's quadrature.
    fn pointwise(&self, part: &Part, x: &[f64], r: &[f64], g: impl Fn(f64) -> f64) -> f64 {
        let n = self.gauge.integrand_vars();
        let mut s = Scratch::new();
        let mut v = 0.0;
        let mut add = |p: P<Dual<0>>, rho: f64, dv: f64| {
            v += g(body(&part.integrand, n, p.map(|c| c.v), rho, r, &mut s).v) * dv;
        };
        match &part.domain {
            Domain::Tets(runs) => {
                for i in runs.iter().flat_map(|&(a, k)| a..a + k) {
                    let rho = self.tets[i].rho;
                    super::solid::each_point(&self.tet_places_d::<0>(i, x),
                                             |p, w, det| add(p, rho, w * det.v));
                }
            }
            Domain::Facets(facets) => {
                for f in facets {
                    each_facet_point(&self.facet_places::<0>(f, x), |p, da| add(p, 0.0, da.v));
                }
            }
        }
        v
    }

    /// **`Σ_j w_j ∂I_j` at `x`, reading `r`**, the quadratic parts' moments `taken` there: its
    /// rate in the nodes' places added into `g`, and in what the bodies read into `gr` — each
    /// element's or facet's points in `Dual` over its nodes' places, each body's rate in its point
    /// and its readings off its tape.
    pub(super) fn integrals_gradient(&self, x: &[f64], r: &[f64], w: &[f64], taken: &Taken,
                                     g: &mut [f64], gr: &mut [f64]) {
        let n = self.gauge.integrand_vars();
        let mut s = Scratch::new();
        // the quadratic parts over one domain weigh its moments' rates once: each run's weights
        // summed over the parts, then one pass over its elements
        for (first, runs) in taken {
            let domain = &self.gauge.parts[*first].domain;
            let mut weights = vec![[0.0; MOMENTS]; runs.len()];
            for (part, &wj) in self.gauge.parts.iter().zip(w) {
                if !part.quadratic || part.domain != *domain || wj == 0.0 {
                    continue;
                }
                for (k, run) in runs.iter().enumerate() {
                    let a = coefficients(&part.integrand, n, run.rho, r, &run.frame, &mut s);
                    for (wk, ak) in weights[k].iter_mut().zip(&a) {
                        *wk += wj * ak.v;
                    }
                    for (q, gq) in gr.iter_mut().enumerate() {
                        *gq += wj * a.iter().zip(&run.m).map(|(a, m)| a.d[POINT + q] * m)
                            .sum::<f64>();
                    }
                }
            }
            for ((run, moments), weights) in self.domain_runs(domain).into_iter().zip(runs)
                .zip(&weights)
            {
                if weights.iter().any(|&w| w != 0.0) {
                    self.moments_gradient(&run, &moments.frame, x, weights, g);
                }
            }
        }
        for (part, &wj) in self.gauge.parts.iter().zip(w) {
            if wj == 0.0 || part.quadratic {
                continue;
            }
            // one point's share: its body's rate carried through its place and its measure
            let mut point = |p: &P<Dual<30>>, rho: f64, dv: &Dual<30>, nodes: &[usize],
                             g: &mut [f64], gr: &mut [f64]| {
                let f = body(&part.integrand, n, p.map(|c| c.v), rho, r, &mut s);
                for (k, &node) in nodes.iter().enumerate() {
                    for c in 0..3 {
                        let j = 3 * k + c;
                        let along = f.d[0] * p[0].g[j] + f.d[1] * p[1].g[j] + f.d[2] * p[2].g[j];
                        g[3 * node + c] += wj * (along * dv.v + f.v * dv.g[j]);
                    }
                }
                for (k, gk) in gr.iter_mut().enumerate() {
                    *gk += wj * f.d[POINT + k] * dv.v;
                }
            };
            match &part.domain {
                Domain::Tets(runs) => {
                    for i in runs.iter().flat_map(|&(a, k)| a..a + k) {
                        let t = &self.tets[i];
                        super::solid::each_point(&self.tet_places_d::<30>(i, x), |p, wq, det| {
                            point(&p, t.rho, &(det * Dual::cst(wq)), &t.n, g, gr)
                        });
                    }
                }
                Domain::Facets(facets) => {
                    for f in facets {
                        let places = self.facet_places::<18>(f, x);
                        each_facet_point(&places, |p, da| {
                            point(&p.map(wide), 0.0, &wide(da), f, g, gr)
                        });
                    }
                }
            }
        }
    }
}
