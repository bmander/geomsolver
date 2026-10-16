//! **A traced body in space** (#326, R3 and R5): a point of a component as one or two of its
//! numeric formals run, where the component states relations in space.
//!
//! `k := Helix(ax, r: 5, lead: 3).p over u in (0, 720)` is a **curve in space** and
//! `S := Element(P, L, …).X over (phi in (a, b), s in (c, d))` a **surface**: §6.5's curve, the
//! component's body solved in space rather than in one plane, over one formal or two.  The body is
//! lowered once (`program::traced`) to **rows** — static kernels, each with a map from its columns
//! into one variable table, as a planar trace's are (`locus.rs`) — over `[formals, θ, values] ++ q
//! ++ w`: the running formals, the coordinates of the geometry the component is written over
//! (each point's lift, three numbers), the constants it was given, then the body's own points
//! `q` and the numbers it reads over the formals `w` (a tape each, read by its row's free twin).
//! A point a motion carries (`X coincident(under: m, t == −phi) x`, R4) is a row of its own, the
//! motion's steps its constants and its axes columns of the variable table.
//!
//! **Its point is a small Newton solve**, started at the body's seeds (tapes over the formals) at
//! the asked place and, where that fails, marched there from the body's home: no pose is
//! remembered, so a reading is a function of its place alone.  **Its derivatives are exact.**
//! Along any direction `d` of the outer variables the point is a Taylor series — the implicit
//! function's, one order a solve with the factorisation the root paid for (Wagner, Walther and
//! Schaefer, as `locus::higher_orders` reads one along `u`), every row read over `taylor::Jet`s
//! (each kernel's form) and every tape over them too (`tape::eval_num`).  A derivative across
//! directions is read by **polarisation**: `D²X[a, b] = (S(a + b) − S(a) − S(b))/2`, `S(v)` the
//! second order along `v`, and the third order likewise over seven series — so a surface's normal
//! `C_φ × C_s` and its curvature are exact, and their Jacobians in every column too.
//!
//! **Contacts** with a traced body own a place `[u, s]` (a curve's `s` held at none): a point on
//! it (three rows), a line along a curve's tangent (two), a line in a surface's tangent plane
//! (one), along its normal or a plane touching it (two each), and a circle osculating a surface's
//! section by the circle's plane (three, Meusnier, as an envelope's, `envelope::held`).  Each is
//! written once over `kernels::Num` from a `Frame` — the point and its derivatives in the formals —
//! read exact in every column, so a kernel's Jacobian is exact by forward differentiation.
//!
//! What this certifies is local: a root near where it was seeded, on the branch the seeds pick;
//! not that the body is regular over its whole domain (`regularity` says where it is not).

use crate::envelope::held::{cross, dot, unit};
use crate::envelope::{held, screw};
use crate::kernels::{Dual, Kernel, KernelFn, Num, KERNELS};
use crate::taylor::{self, Jet};

/// Bounds on what a document may ask for: a document is untrusted input, and the encoding is
/// range-checked as it is read.
pub const MAX_Q: usize = 48;
pub const MAX_ROWS: usize = 64;
pub const MAX_W: usize = 16;
/// A traced body runs over one formal (a curve) or two (a surface).
pub const MAX_FORMALS: usize = 2;
/// The place a contact owns: two numbers whatever the body, a curve's second held at none.
pub const PLACE: usize = 2;

const NEWTON_MAX: usize = 40;
const MARCH: usize = 32;

/* -- the compiled form -------------------------------------------------------------------- */

/// What a row of the body states.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RowKind {
    /// A static kernel of `KERNELS`, by its id.
    Kernel(usize),
    /// A point a motion carries (`screw::Reading::Carried`, its constants the motion's steps).
    Carried,
    /// A point on a spline over points in space (#363 S3): `X − Σ Rᵢ(u)·Pᵢ`, its columns
    /// `[u, P₀ … Pₙ₋₁, X]`, its constants the knot vector and the weights (`spline_res`,
    /// `spline_jets`).
    Spline,
    /// **A number the body places** (#378), held to an expression over the formals, the geometry
    /// and the body's own coordinates: `v − f(x…)`, its columns `[v, x…]`, its constants `f`'s
    /// tape over `x` (`number_res`, `number_jets`).  What a dimension that reads the body's own
    /// points is lowered to: the row it stands in reads `v` where it would read a tape over the
    /// formals alone.
    Number,
    /// A point in space on a curve drawn in a plane (#376): `X − (o + Cₓ(t)·u + C_y(t)·v)`, its
    /// columns `[t, θ…, X]` (`θ` the curve's own, `Sketch::entity_params`), its constants the
    /// curve's body (`kernels::FORMULA`, `TRACE`, `ENVELOPE`, `EXTREMAL`), its plane's basis `u`,
    /// `v`, `o`, then a contact's constants on it (`curve_res`, `curve_jac`).  It has no Taylor
    /// form: Cauchy data of a marched body alone, which its home reads to the first order.
    Curve,
    /// A point on a curve traced in space (#376): the contact `point_on_traced`, its columns
    /// `[X, t, s, θ…]` (`s` held at none), its constants the curve's (`consts_of`).  Cauchy data
    /// alone, as `Curve` is.
    Traced,
}

impl RowKind {
    /// Its kernel at a row's widths.
    fn kernel(self, n_par: usize, n_const: usize) -> Option<Kernel> {
        match self {
            RowKind::Kernel(k) => KERNELS.get(k).copied(),
            kind => Some(kind.at_widths(n_par, n_const)),
        }
    }

    /// Its kernel at a row's widths, a static kernel's by its id (which must be one).
    pub(crate) fn at_widths(self, n_par: usize, n_const: usize) -> Kernel {
        match self {
            RowKind::Kernel(k) => KERNELS[k],
            RowKind::Carried => screw::kernel(screw::Reading::Carried, n_par, n_const),
            RowKind::Spline => Kernel { name: "spline_point3", n_res: 3, n_par, n_const,
                degree: 1, res: spline_res, jac: spline_jac, const_jac: None, form: None },
            RowKind::Number => number_kernel(n_par, n_const),
            RowKind::Curve => Kernel { name: "point_on_drawn", n_res: 3, n_par, n_const,
                degree: 1, res: curve_res, jac: curve_jac, const_jac: None, form: None },
            RowKind::Traced => kernel(Contact::Point, n_par, n_const),
        }
    }

    /// Its rows over jets (`v` its columns), into `r`: a static kernel's Taylor form, a carried
    /// point's, a spline's and a placed number's rows read over jets as they are written, and a
    /// point on a curve with no form to the first order (#376, `first_order`).  `false` where a
    /// row has no form or its constants describe nothing.
    pub(crate) fn jets(self, v: &[Jet], k: &[f64], r: &mut [Jet], jrow: &mut Vec<f64>) -> bool {
        match self {
            RowKind::Kernel(id) => taylor::residual(id, v, k, r, jrow),
            RowKind::Carried => screw::rows(screw::Reading::Carried, v, k, r).is_some(),
            RowKind::Spline => spline_jets(v, k, r).is_some(),
            RowKind::Number => number_jets(v, k, r).is_some(),
            RowKind::Curve | RowKind::Traced => {
                first_order(&self.at_widths(v.len(), k.len()), v, k, r)
            }
        }
    }

    /// Whether it is a point on a curve the drawing has, which a marched body's `initially`
    /// alone may read (#376).
    pub fn drawn(self) -> bool {
        matches!(self, RowKind::Spline | RowKind::Curve | RowKind::Traced)
    }

    /// Its kernel's id in `KERNELS`, where it is a static kernel's.
    pub fn id(self) -> Option<usize> {
        match self {
            RowKind::Kernel(k) => Some(k),
            RowKind::Carried | RowKind::Spline | RowKind::Number | RowKind::Curve
            | RowKind::Traced => None,
        }
    }

    /// How it is written in the encoding: a kernel's id, `-1` a carried point, `-2` a point on a
    /// spline, `-3` a placed number, `-4` a point on a curve drawn in a plane, `-5` on a curve
    /// traced in space.
    fn code(self) -> f64 {
        match self {
            RowKind::Kernel(k) => k as f64,
            RowKind::Carried => -1.0,
            RowKind::Spline => -2.0,
            RowKind::Number => -3.0,
            RowKind::Curve => -4.0,
            RowKind::Traced => -5.0,
        }
    }
}

/// One row of the body: what it states (`kind`), the variable-table slot each of its columns
/// reads, and its constants.
#[derive(Clone, Debug)]
pub struct Row {
    pub kind: RowKind,
    pub cols: Vec<u32>,
    pub consts: Vec<f64>,
}

impl Row {
    /// How many residuals it makes: its kernel's, three for a point a motion carries or a point
    /// on a spline or a curve, one for a placed number; none for an id no kernel has, which the
    /// encoding refuses.
    pub fn n_res(&self) -> usize {
        match self.kind {
            RowKind::Kernel(k) => KERNELS.get(k).map_or(0, |kn| kn.n_res),
            RowKind::Carried | RowKind::Spline | RowKind::Curve | RowKind::Traced => 3,
            RowKind::Number => 1,
        }
    }
}

/* -- a spline over points in space -------------------------------------------------------- */

/// A spline row's widths read back: its control points' count, or `None` where the columns and
/// constants describe no spline (`[u, P×n, X]` and `knots(n + DEGREE + 1) ++ weights(n)`).
fn spline_shape(n_par: usize, n_const: usize, k: &[f64]) -> Option<usize> {
    use crate::curve::{knots_valid, weights_valid, DEGREE};
    let n = n_par.checked_sub(4)? / 3;
    if 3 * n + 4 != n_par || n_const != 2 * n + DEGREE + 1 {
        return None;
    }
    let (knots, weights) = k.split_at(n + DEGREE + 1);
    (knots_valid(knots, n) && weights_valid(weights, n)).then_some(n)
}

/// The rational basis at `u`, on the span `u` falls in: the first control point it reads and
/// each one's value and first three derivatives (`curve::basis`, `weigh`).
fn spline_basis(n: usize, k: &[f64], u: f64) -> (usize, [[f64; 4]; crate::curve::SPAN_N]) {
    use crate::curve::{basis, local_knots, local_weights, span_index, weigh, DEGREE, SPAN_N};
    let (knots, weights) = k.split_at(n + DEGREE + 1);
    let span = span_index(knots, n, u);
    let lk = local_knots(knots, span);
    let (mut b, mut d) = ([0.0; SPAN_N], [0.0; SPAN_N]);
    let (mut dd, mut d3) = ([0.0; SPAN_N], [0.0; SPAN_N]);
    basis(u, &lk, &mut b, &mut d, &mut dd, &mut d3);
    weigh(&local_weights(Some(weights), span), &mut b, &mut d, &mut dd, &mut d3);
    (span - DEGREE, std::array::from_fn(|a| [b[a], d[a], dd[a], d3[a]]))
}

fn spline_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    let (n_par, n_const) = crate::kernels::curve_widths(n, v, k);
    for i in 0..n {
        let (v, k, r) = (&v[i * n_par..(i + 1) * n_par], &k[i * n_const..(i + 1) * n_const],
                         &mut r[3 * i..3 * i + 3]);
        let Some(m) = spline_shape(n_par, n_const, k) else { r.fill(f64::NAN); continue };
        let (first, rb) = spline_basis(m, k, v[0]);
        for t in 0..3 {
            let on: f64 = rb.iter().enumerate().map(|(a, w)| w[0] * v[1 + 3 * (first + a) + t]).sum();
            r[t] = v[1 + 3 * m + t] - on;
        }
    }
}

fn spline_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    let (n_par, n_const) = crate::kernels::curve_widths(n, v, k);
    for i in 0..n {
        let (v, k) = (&v[i * n_par..(i + 1) * n_par], &k[i * n_const..(i + 1) * n_const]);
        let j = &mut j[3 * n_par * i..3 * n_par * (i + 1)];
        j.fill(0.0);
        let Some(m) = spline_shape(n_par, n_const, k) else { j.fill(f64::NAN); continue };
        let (first, rb) = spline_basis(m, k, v[0]);
        for t in 0..3 {
            let row = &mut j[t * n_par..(t + 1) * n_par];
            for (a, w) in rb.iter().enumerate() {
                row[0] -= w[1] * v[1 + 3 * (first + a) + t];
                row[1 + 3 * (first + a) + t] = -w[0];
            }
            row[1 + 3 * m + t] = 1.0;
        }
    }
}

/// The spline row over jets: each basis function the cubic `Σ R⁽ᵏ⁾(u₀)/k!·(u − u₀)ᵏ` of the
/// jet's offset — exact to the third order the basis carries, all the orders a polynomial span
/// has.
fn spline_jets(v: &[Jet], k: &[f64], r: &mut [Jet]) -> Option<()> {
    let m = spline_shape(v.len(), k.len(), k)?;
    let u0 = v[0].0[0];
    let (first, rb) = spline_basis(m, k, u0);
    let du = v[0] - Jet::constant(u0);
    for (t, out) in r.iter_mut().enumerate().take(3) {
        let mut acc = v[1 + 3 * m + t];
        for (a, w) in rb.iter().enumerate() {
            let basis = ((Jet::constant(w[3] / 6.0) * du + Jet::constant(w[2] / 2.0)) * du
                + Jet::constant(w[1])) * du + Jet::constant(w[0]);
            acc = acc - basis * v[1 + 3 * (first + a) + t];
        }
        *out = acc;
    }
    Some(())
}

/* -- a curve drawn in a plane, read in space (#376) --------------------------------------- */

/// What a point on a drawn curve's constants open with: the body's code and the plane's basis.
const CURVE_LEAD: usize = 1 + crate::kernels::EXTRUSION_FRAME;

/// The contact kernel on a drawn curve's definition a row's widths and constants name, and how
/// many columns the curve reads: `None` where they describe none.
fn curve_inner(n_par: usize, k: &[f64]) -> Option<(Kernel, usize)> {
    use crate::kernels::{curve_kernel, point_on_body_kernel, ENVELOPE, EXTREMAL, FORMULA, TRACE};
    let n_theta = n_par.checked_sub(4)?;
    let n_const = k.len().checked_sub(CURVE_LEAD)?;
    if n_const < 2 || !k[..CURVE_LEAD].iter().all(|x| x.is_finite()) {
        return None;
    }
    let inner = match k[0] {
        b if b == FORMULA as f64 => curve_kernel(n_theta, n_const),
        b if [TRACE, ENVELOPE, EXTREMAL].iter().any(|&c| b == c as f64) => {
            point_on_body_kernel(n_theta, n_const, k[0] as u8)
        }
        _ => return None,
    };
    Some((inner, n_theta))
}

/// The drawn curve's contact at `[0, 0, t, θ…]`: `−C(t)` and, where asked, its Jacobian (two rows
/// over `[px, py, t, θ…]`).
fn curve_at(v: &[f64], k: &[f64], jac: bool) -> Option<([f64; 2], Vec<f64>)> {
    let (inner, n_theta) = curve_inner(v.len(), k)?;
    let mut w = vec![0.0; 3 + n_theta];
    w[2..].copy_from_slice(&v[..1 + n_theta]);
    let kc = &k[CURVE_LEAD..];
    let mut r = [0.0; 2];
    (inner.res)(1, &w, kc, &mut r);
    let mut j = Vec::new();
    if jac {
        j.resize(2 * inner.n_par, 0.0);
        inner.jac_into(&w, kc, &mut j);
    }
    Some((r, j))
}

fn curve_res(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    let (n_par, n_const) = crate::kernels::curve_widths(n, v, k);
    for i in 0..n {
        let (v, k, r) = (&v[i * n_par..(i + 1) * n_par], &k[i * n_const..(i + 1) * n_const],
                         &mut r[3 * i..3 * i + 3]);
        let Some((m, _)) = curve_at(v, k, false) else { r.fill(f64::NAN); continue };
        let (u, w, o) = (&k[1..4], &k[4..7], &k[7..10]);
        let x = &v[n_par - 3..];
        // `m` is `−C`: the point less where the curve stands in its plane
        for t in 0..3 {
            r[t] = x[t] - o[t] + m[0] * u[t] + m[1] * w[t];
        }
    }
}

fn curve_jac(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    let (n_par, n_const) = crate::kernels::curve_widths(n, v, k);
    for i in 0..n {
        let (v, k) = (&v[i * n_par..(i + 1) * n_par], &k[i * n_const..(i + 1) * n_const]);
        let j = &mut j[3 * n_par * i..3 * n_par * (i + 1)];
        j.fill(0.0);
        let Some((_, jm)) = curve_at(v, k, true) else { j.fill(f64::NAN); continue };
        let (u, w) = (&k[1..4], &k[4..7]);
        // the contact's columns are ours less the point, past its own two
        let row1 = n_par - 1;
        for t in 0..3 {
            let row = &mut j[t * n_par..(t + 1) * n_par];
            for c in 0..n_par - 3 {
                row[c] = jm[2 + c] * u[t] + jm[row1 + 2 + c] * w[t];
            }
            row[n_par - 3 + t] = 1.0;
        }
    }
}

/// A row with no Taylor form over jets to the first order: its residual at the jets' values, and
/// its Jacobian along their first order; every higher order NaN, so nothing that reads one can
/// take it for a number.  All a marched body's home reads of a point on a curve (#376).
fn first_order(kn: &Kernel, v: &[Jet], k: &[f64], out: &mut [Jet]) -> bool {
    let x: Vec<f64> = v.iter().map(|j| j.0[0]).collect();
    let (mut r, mut j) = (vec![0.0; kn.n_res], vec![0.0; kn.n_res * kn.n_par]);
    (kn.res)(1, &x, k, &mut r);
    kn.jac_into(&x, k, &mut j);
    for (t, o) in out.iter_mut().enumerate() {
        let d: f64 = (0..kn.n_par).map(|c| j[t * kn.n_par + c] * v[c].0[1]).sum();
        let mut a = [f64::NAN; taylor::ORDER];
        a[0] = r[t];
        a[1] = d;
        *o = Jet(a);
    }
    out.iter().all(|o| o.0[0].is_finite() && o.0[1].is_finite())
}

/// A compiled body, held only as its encoding — what rides in a contact's constants.  The
/// variable table its rows index is `[formals, θ, values] ++ q ++ w`.
#[derive(Clone, Debug, PartialEq)]
pub struct Body {
    pub flat: Vec<f64>,
}

const HEADER: usize = 16;

/// What a body that states a rate adds (#339 D2, D4): the rows that read a rate, those that hold
/// at its home alone (`initially { … }`), the interval its curve runs over, which its march
/// reaches, and over two formals the sampled formal's interval, what a rate along each is per
/// (`lines::Body::per`) and the pieces it is sampled on.  Empty for every other body.
#[derive(Clone, Debug, Default)]
pub struct Rates {
    pub rows: Vec<Row>,
    pub init: Vec<Row>,
    pub domain: [f64; 2],
    pub second: [f64; 2],
    pub per: [f64; 2],
    pub pieces: usize,
}

impl Body {
    /// Validate and encode.  An `Err` is a diagnostic for the body, not a panic.  `rates` is what
    /// a body that states a rate adds (#339 D2).
    #[allow(clippy::too_many_arguments)]
    pub fn new(n_formals: usize, n_outer: usize, n_theta: usize, n_q: usize, traced: usize,
               w: Vec<crate::tape::Tape>, seeds: Vec<crate::tape::Tape>, rows: Vec<Row>,
               rates: Rates) -> Result<Body, String> {
        let stated = !rates.rows.is_empty();
        if !stated && !rates.init.is_empty() {
            return Err("`initially { … }` says what holds at a home, and only a body that \
                states a rate has one: its rates are integrated from it".into());
        }
        // a point on a curve the drawing has is Cauchy data (#376): read at a marched body's home
        // to the first order, and nowhere else — a spline over points in space aside, which is a
        // body of its own (#363 S3)
        let misplaced = |r: &Row| match r.kind {
            RowKind::Spline => stated,
            k => k.drawn(),
        };
        let marched = stated && n_formals == 2;
        if rows.iter().chain(&rates.rows).any(misplaced)
            || (!marched && rates.init.iter().any(|r| r.kind.drawn())) {
            return Err("a point on a curve the drawing has, `p coincident(t == u) k`, is Cauchy \
                data: it is read in `initially { … }` of a body marched over two formals".into());
        }
        if !(1..=MAX_FORMALS).contains(&n_formals) || n_formals + n_theta > n_outer {
            return Err("a traced body runs over one formal or two".into());
        }
        if n_q == 0 || n_q > MAX_Q {
            return Err(format!("a traced body in space places between 1 and {MAX_Q} numbers"));
        }
        if rows.len() > MAX_ROWS || w.len() > MAX_W {
            return Err(format!("a traced body may state at most {MAX_ROWS} relations and \
                                {MAX_W} expressions"));
        }
        if seeds.len() != n_q || traced + 3 > n_q {
            return Err("traced body shape mismatch".into());
        }
        let mut n_res = 0;
        for r in &rows {
            let kn = r.kind.kernel(r.cols.len(), r.consts.len()).ok_or("no such kernel")?;
            if let RowKind::Kernel(k) = r.kind {
                if !taylor::has_form(k) {
                    return Err(format!("a body traced in space reads its rows' derivatives \
                                        exactly, and `{}` has no Taylor form", KERNELS[k].name));
                }
            }
            n_res += kn.n_res;
        }
        if !stated && n_res != n_q {
            return Err(format!("a traced body must determine its points: {n_q} unknowns against \
                                {n_res} equations"));
        }
        let per = if stated { rates.per } else { [1.0; 2] };
        let mut f = vec![n_formals as f64, n_outer as f64, n_theta as f64, n_q as f64,
                         w.len() as f64, rows.len() as f64, traced as f64,
                         rates.rows.len() as f64, rates.init.len() as f64,
                         rates.domain[0], rates.domain[1], rates.second[0], rates.second[1],
                         per[0], per[1], rates.pieces as f64];
        for t in &w {
            f.push(t.flat.len() as f64);
            f.extend_from_slice(&t.flat);
        }
        for r in rows.iter().chain(&rates.rows).chain(&rates.init) {
            f.push(r.kind.code());
            f.push(r.cols.len() as f64);
            f.push(r.consts.len() as f64);
            f.extend(r.cols.iter().map(|&c| c as f64));
            f.extend_from_slice(&r.consts);
        }
        for t in &seeds {
            f.push(t.flat.len() as f64);
            f.extend_from_slice(&t.flat);
        }
        // every row's shape and column is checked once, as any encoding is read
        let v = view(&f).ok_or("traced body shape mismatch")?;
        match (stated, n_formals) {
            (true, 1) => v.rate_body().check()?,
            (true, _) => {
                v.lines_body().plan()?;
            }
            _ => {}
        }
        Ok(Body { flat: f })
    }

    /// Whether the body states a rate (#339 D2).
    pub fn states_rate(&self) -> bool {
        view(&self.flat).is_some_and(|v| !v.rates.is_empty())
    }

    /// How many formals it runs over.
    pub fn n_formals(&self) -> usize {
        view(&self.flat).map_or(0, |v| v.nf)
    }

    /// How many coordinates of geometry it is written over (its columns past the place).
    pub fn n_theta(&self) -> usize {
        view(&self.flat).map_or(0, |v| v.n_theta)
    }
}

/// A row as the encoding is read back: its kernel, what it states, its columns and constants.
type Decoded<'a> = (Kernel, RowKind, Vec<usize>, &'a [f64]);

/// The encoding, decoded to borrowed slices, every length and column checked on the way in.
struct View<'a> {
    nf: usize,
    n_outer: usize,
    n_theta: usize,
    n_q: usize,
    traced: usize,
    w: Vec<&'a [f64]>,
    rows: Vec<Decoded<'a>>,
    seeds: Vec<&'a [f64]>,
    /// A body that states a rate (#339 D2, D4): its rate rows, its home's, its intervals, what a
    /// rate along each formal is per and the pieces the second is sampled on.
    rates: Vec<Decoded<'a>>,
    init: Vec<Decoded<'a>>,
    domain: [f64; 2],
    second: [f64; 2],
    per: [f64; 2],
    pieces: usize,
}

/// Rows as an integrator reads them: a static kernel's, a point a motion carries (#377), a
/// placed number (#378), or a point on a curve the drawing has, which a marched body's home reads
/// (#376).
fn rate_rows<'a>(rs: &[Decoded<'a>]) -> Vec<crate::rates::Row<'a>> {
    rs.iter().map(|(_, kind, cols, consts)| crate::rates::Row {
        kind: *kind,
        cols: cols.clone(),
        consts,
    }).collect()
}

impl<'a> View<'a> {
    /// The body over two formals as the method of lines reads it (`lines.rs`, #339 D4).
    fn lines_body(&self) -> crate::lines::Body<'a> {
        crate::lines::Body {
            n_outer: self.n_outer,
            n_theta: self.n_theta,
            n_q: self.n_q,
            traced: self.traced,
            w: self.w.clone(),
            seeds: self.seeds.clone(),
            alg: rate_rows(&self.rows),
            rate: rate_rows(&self.rates),
            init: rate_rows(&self.init),
            domain: [self.domain, self.second],
            per: self.per,
            pieces: self.pieces,
        }
    }

    /// The body as the integrator reads it (`rates.rs`): over one formal, in space.
    fn rate_body(&self) -> crate::rates::Body<'a> {
        crate::rates::Body {
            n_outer: self.n_outer,
            n_theta: self.n_theta,
            n_q: self.n_q,
            pose: false,
            traced: self.traced,
            dim: 3,
            w: self.w.clone(),
            seeds: self.seeds.clone(),
            alg: rate_rows(&self.rows),
            rate: rate_rows(&self.rates),
            init: rate_rows(&self.init),
        }
    }
}

fn view(flat: &[f64]) -> Option<View<'_>> {
    let g = |i: usize| flat.get(i).copied()
        .filter(|v| v.is_finite() && *v >= 0.0 && v.fract() == 0.0).map(|v| v as usize);
    let (nf, n_outer, n_theta, n_q, n_w, n_rows, traced) =
        (g(0)?, g(1)?, g(2)?, g(3)?, g(4)?, g(5)?, g(6)?);
    let (n_rates, n_init) = (g(7)?, g(8)?);
    let domain = [*flat.get(9)?, *flat.get(10)?];
    let second = [*flat.get(11)?, *flat.get(12)?];
    let per = [*flat.get(13)?, *flat.get(14)?];
    let pieces = g(15)?;
    if !(1..=MAX_FORMALS).contains(&nf) || n_q > MAX_Q || n_w > MAX_W || n_rows > MAX_ROWS
        || n_rates > MAX_ROWS || n_init > MAX_ROWS
        || traced + 3 > n_q || nf + n_theta > n_outer || n_outer > 4096 {
        return None;
    }
    // a body that states a rate has its rates as columns past `w` (#339 D2), over two formals
    // the rates along the second past those (D4)
    let width = n_outer + n_q + n_w + if n_rates > 0 { n_q * nf } else { 0 };
    let mut at = HEADER;
    let take = |n: usize, at: &mut usize| -> Option<&[f64]> {
        let s = flat.get(*at..*at + n)?;
        *at += n;
        Some(s)
    };
    let mut w = Vec::with_capacity(n_w);
    for _ in 0..n_w {
        let n = g(at)?;
        at += 1;
        w.push(take(n, &mut at)?);
    }
    let mut all = Vec::with_capacity(n_rows + n_rates + n_init);
    for _ in 0..n_rows + n_rates + n_init {
        let kind = match flat.get(at).copied()? {
            -1.0 => RowKind::Carried,
            -2.0 => RowKind::Spline,
            -3.0 => RowKind::Number,
            -4.0 => RowKind::Curve,
            -5.0 => RowKind::Traced,
            _ => RowKind::Kernel(g(at)?),
        };
        let (n_cols, n_consts) = (g(at + 1)?, g(at + 2)?);
        at += 3;
        let cols: Vec<usize> = take(n_cols, &mut at)?.iter().map(|&c| c as usize).collect();
        if cols.iter().any(|&c| c >= width) {
            return None;
        }
        let consts = take(n_consts, &mut at)?;
        let kn = kind.kernel(n_cols, n_consts)?;
        match kind {
            RowKind::Kernel(_) => {
                if kn.n_par != n_cols || kn.n_const != n_consts {
                    return None;
                }
            }
            RowKind::Carried => {
                let steps = held::read_steps(consts)?;
                if n_cols != 7 + held::steps_width(&steps) {
                    return None;
                }
            }
            RowKind::Spline => {
                spline_shape(n_cols, n_consts, consts)?;
            }
            RowKind::Number => {
                if !number_shape(n_cols, consts) {
                    return None;
                }
            }
            RowKind::Curve => {
                curve_inner(n_cols, consts)?;
            }
            RowKind::Traced => {
                let (_, _, inner) = parts(consts)?;
                let iv = view(inner)?;
                if iv.nf != 1 || n_cols != Contact::Point.lead() + PLACE + iv.n_theta {
                    return None;
                }
            }
        }
        all.push((kn, kind, cols, consts));
    }
    let init = all.split_off(n_rows + n_rates);
    let rates = all.split_off(n_rows);
    let rows = all;
    let mut seeds = Vec::with_capacity(n_q);
    for _ in 0..n_q {
        let n = g(at)?;
        at += 1;
        seeds.push(take(n, &mut at)?);
    }
    if rates.is_empty() && rows.iter().map(|r| r.0.n_res).sum::<usize>() != n_q {
        return None;
    }
    Some(View { nf, n_outer, n_theta, n_q, traced, w, rows, seeds, rates, init, domain, second,
                per, pieces })
}

/* -- a number the body places ------------------------------------------------------------- */

/// The most columns a placed number's row reads, the number first.
pub const NUMBER_COLS: usize = 16;

/// A placed number's kernel at its widths: `v − f(x…)`, one row.
pub(crate) fn number_kernel(n_par: usize, n_const: usize) -> Kernel {
    Kernel { name: "placed_number", n_res: 1, n_par, n_const, degree: 1, res: number_res,
             jac: number_jac, const_jac: None, form: None }
}

/// Whether a placed number's columns and constants describe one: the number and what its tape
/// reads, within `NUMBER_COLS`, the tape whole instructions reading no variable past its columns.
pub(crate) fn number_shape(n_par: usize, k: &[f64]) -> bool {
    (1..=NUMBER_COLS).contains(&n_par) && !k.is_empty()
        && crate::tape::reads_within(k, n_par - 1)
}

fn number_res(_: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    r[0] = v[0] - crate::tape::eval_num::<f64>(k, &v[1..]).unwrap_or(f64::NAN);
}

fn number_jac(_: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    let x: [Dual<NUMBER_COLS>; NUMBER_COLS] =
        std::array::from_fn(|i| Dual::var(v.get(i + 1).copied().unwrap_or(0.0), i));
    let f = crate::tape::eval_num::<Dual<NUMBER_COLS>>(k, &x[..v.len() - 1]);
    j[0] = 1.0;
    for (i, j) in j[1..v.len()].iter_mut().enumerate() {
        *j = f.map_or(f64::NAN, |f| -f.g[i]);
    }
}

/// A placed number's row over jets: exact to the third order, as every tape over jets is
/// (`tape::eval_num`).
pub(crate) fn number_jets(v: &[Jet], k: &[f64], r: &mut [Jet]) -> Option<()> {
    r[0] = v[0] - crate::tape::eval_num::<Jet>(k, &v[1..])?;
    Some(())
}

/* -- solving the body at a place ---------------------------------------------------------- */

/// The body solved at one place: the variable table there and the factorised `∂r/∂q`.
struct Solved {
    z: Vec<f64>,
    lu: Vec<f64>,
    piv: Vec<usize>,
}

impl View<'_> {
    /// The derived values over the outer variables.
    fn derived(&self, outer: &[f64]) -> Option<Vec<f64>> {
        self.w.iter().map(|t| crate::tape::eval_num::<f64>(t, outer)).collect()
    }

    /// The body's residuals over the whole variable table `z`, into `r`.
    fn residual(&self, z: &[f64], r: &mut Vec<f64>) {
        r.resize(self.n_q, 0.0);
        let mut at = 0;
        for (kn, _, cols, consts) in &self.rows {
            let x: Vec<f64> = cols.iter().map(|&c| z[c]).collect();
            (kn.res)(1, &x, consts, &mut r[at..at + kn.n_res]);
            at += kn.n_res;
        }
    }

    /// `∂r/∂q`, row-major `n_q × n_q`.
    fn jacobian(&self, z: &[f64]) -> Vec<f64> {
        let n = self.n_q;
        let mut j = vec![0.0; n * n];
        let mut row = 0;
        for (kn, _, cols, consts) in &self.rows {
            let x: Vec<f64> = cols.iter().map(|&c| z[c]).collect();
            let mut jr = vec![0.0; kn.n_res * kn.n_par];
            kn.jac_into(&x, consts, &mut jr);
            for t in 0..kn.n_res {
                for (c, &col) in cols.iter().enumerate() {
                    if (self.n_outer..self.n_outer + n).contains(&col) {
                        j[(row + t) * n + col - self.n_outer] += jr[t * kn.n_par + c];
                    }
                }
            }
            row += kn.n_res;
        }
        j
    }

    /// The seeds' places at the outer variables.
    fn seeded(&self, outer: &[f64]) -> Option<Vec<f64>> {
        self.seeds.iter().map(|t| crate::tape::eval_num::<f64>(t, outer)).collect()
    }

    /// Damped Newton on `q` at `outer`, from `q`.  The root, or `None`.
    fn newton(&self, outer: &[f64], q: &[f64]) -> Option<Solved> {
        let w = self.derived(outer)?;
        let mut z: Vec<f64> = outer.iter().chain(q).chain(&w).copied().collect();
        let scale = z.iter().fold(1.0f64, |a, x| a.max(x.abs()));
        let tol = 1e-12 * scale;
        let mut r = Vec::with_capacity(self.n_q);
        self.residual(&z, &mut r);
        let norm = |r: &[f64]| r.iter().fold(0.0f64, |a, x| a.max(x.abs()));
        let mut now = norm(&r);
        for _ in 0..NEWTON_MAX {
            if !now.is_finite() {
                return None;
            }
            let mut lu = self.jacobian(&z);
            let mut piv = Vec::new();
            let regular = crate::linalg::lu_factor(self.n_q, &mut lu, &mut piv);
            if now <= tol {
                return regular.then_some(Solved { z, lu, piv });
            }
            if !regular {
                return None;
            }
            let mut step: Vec<f64> = r.iter().map(|x| -x).collect();
            crate::linalg::lu_apply(self.n_q, &lu, &piv, &mut step);
            let mut lambda = 1.0;
            let at = self.n_outer;
            loop {
                let mut trial = z.clone();
                for (k, s) in step.iter().enumerate() {
                    trial[at + k] += lambda * s;
                }
                self.residual(&trial, &mut r);
                let next = norm(&r);
                if next.is_finite() && (next < now || lambda < 1e-4) {
                    z = trial;
                    now = next;
                    break;
                }
                lambda *= 0.5;
                if lambda < 1e-6 {
                    return None;
                }
            }
        }
        None
    }

    /// The body at `outer`: from its seeds there, else marched from `home` (the formals' anchor,
    /// the rest of the outer variables as asked) in steps, each from the last.
    fn solve(&self, outer: &[f64], home: &[f64]) -> Option<Solved> {
        if let Some(s) = self.seeded(outer).and_then(|q| self.newton(outer, &q)) {
            return Some(s);
        }
        let mut at = outer.to_vec();
        at[..self.nf].copy_from_slice(&home[..self.nf]);
        let mut s = self.seeded(&at).and_then(|q| self.newton(&at, &q))?;
        for k in 1..=MARCH {
            let f = k as f64 / MARCH as f64;
            for i in 0..self.nf {
                at[i] = home[i] + f * (outer[i] - home[i]);
            }
            let q = s.z[self.n_outer..self.n_outer + self.n_q].to_vec();
            s = self.newton(&at, &q)?;
        }
        Some(s)
    }

    /// The traced point's Taylor series along `dir` (one entry per outer variable), to `order`:
    /// `order + 1` coefficients of each coordinate.  `None` where a row or a tape cannot be read
    /// over jets.
    fn series(&self, s: &Solved, dir: &[f64], order: usize) -> Option<[Jet; 3]> {
        let (no, nq) = (self.n_outer, self.n_q);
        let outer: Vec<Jet> = (0..no).map(|i| Jet::from(&[s.z[i], dir[i]])).collect();
        let w: Vec<Jet> = self.w.iter().map(|t| crate::tape::eval_num::<Jet>(t, &outer))
            .collect::<Option<_>>()?;
        let mut q: Vec<Jet> = (0..nq).map(|i| Jet::constant(s.z[no + i])).collect();
        let mut jrow = Vec::new();
        // the tapes over jets are exact to the third order (`tape::eval_num`)
        for k in 1..=order.min(taylor::ORDER - 2) {
            let z: Vec<Jet> = outer.iter().chain(&q).chain(&w).copied().collect();
            let mut rk = Vec::with_capacity(nq);
            for (kn, kind, cols, consts) in &self.rows {
                let v: Vec<Jet> = cols.iter().map(|&c| z[c]).collect();
                let mut r = vec![Jet::default(); kn.n_res];
                if !kind.jets(&v, consts, &mut r, &mut jrow) {
                    return None;
                }
                rk.extend(r.iter().map(|j| -j.0[k]));
            }
            crate::linalg::lu_apply(nq, &s.lu, &s.piv, &mut rk);
            for (i, x) in rk.into_iter().enumerate() {
                q[i].0[k] = x;
            }
        }
        let t = self.traced;
        Some([q[t], q[t + 1], q[t + 2]])
    }
}

/* -- the frame a contact reads ------------------------------------------------------------- */

/// The traced point and its derivatives in the formals, at a place: `x`, `d[a] = ∂x/∂a` and
/// `dd[a][b] = ∂²x/∂a∂b` (zero past the body's formals, and where not asked for).
#[derive(Clone, Copy, Debug)]
pub struct Frame<T> {
    pub x: [T; 3],
    pub d: [[T; 3]; MAX_FORMALS],
    pub dd: [[[T; 3]; MAX_FORMALS]; MAX_FORMALS],
}

/// How much of the frame to read.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Need {
    /// The point.
    Point,
    /// And its first derivatives.
    Tangent,
    /// And its second.
    Curvature,
}

/// A frame read exactly, and — where asked — its Jacobian in the body's columns: the place (two)
/// then `θ`, each a `[f64; 3]` per entry, so `grad[c]` is the derivative along column `c`.
pub struct Read {
    pub frame: Frame<f64>,
    /// `x`, then `d[a]`, then `dd[a][b]` (in the frame's order), each against every column.
    grad: Option<Vec<[[f64; 3]; SLOTS]>>,
}

/// A frame's entries: `x`, `d[a]`, `dd[a][b]`.
const SLOTS: usize = 1 + MAX_FORMALS + MAX_FORMALS * MAX_FORMALS;

/// One coordinate set's `k`-th derivative from a series: the coefficient times `k!`.
fn nth(s: &[Jet; 3], k: usize) -> [f64; 3] {
    s.map(|j| j.nth(k))
}

/// A memo of series along directions, keyed by the direction's coefficients — a frame reads many
/// directions more than once.
struct Series<'a, 'b> {
    v: &'a View<'b>,
    s: &'a Solved,
    order: usize,
    memo: Vec<(Vec<f64>, [Jet; 3])>,
}

impl Series<'_, '_> {
    fn along(&mut self, dir: Vec<f64>) -> Option<[Jet; 3]> {
        if let Some((_, j)) = self.memo.iter().find(|(d, _)| *d == dir) {
            return Some(*j);
        }
        let j = self.v.series(self.s, &dir, self.order)?;
        self.memo.push((dir, j));
        Some(j)
    }

    /// The unit direction of outer variable `i`, and sums of them.
    fn dir(&self, terms: &[usize]) -> Vec<f64> {
        let mut d = vec![0.0; self.v.n_outer];
        for &i in terms {
            d[i] += 1.0;
        }
        d
    }

    /// `D^k X` along the sum of the unit directions `terms`.
    fn power(&mut self, terms: &[usize], k: usize) -> Option<[f64; 3]> {
        let d = self.dir(terms);
        Some(nth(&self.along(d)?, k))
    }

    /// The symmetric `k`-linear derivative `D^k X[e_terms…]` by polarisation over the series along
    /// the sums of every subset of `terms` (`k` of them, two or three).
    fn mixed(&mut self, terms: &[usize]) -> Option<[f64; 3]> {
        let k = terms.len();
        let mut acc = [0.0; 3];
        // Σ over subsets S of (−1)^(k − |S|) D^k X[Σ_S e]^k / k!
        for mask in 1u32..(1 << k) {
            let sub: Vec<usize> =
                (0..k).filter(|i| mask & (1 << i) != 0).map(|i| terms[i]).collect();
            let sign = if (k - sub.len()) % 2 == 0 { 1.0 } else { -1.0 };
            let p = self.power(&sub, k)?;
            for c in 0..3 {
                acc[c] += sign * p[c];
            }
        }
        let fact: f64 = (1..=k).map(|x| x as f64).product();
        Some(acc.map(|x| x / fact))
    }
}

/// The body over its constants at the outer variables `outer` (`[place, θ, values]`): the frame
/// `need` asks for, and with `grad` its derivative in every column of the place and `θ`.  `home`
/// is where a march starts.  `None` where the body cannot be solved there.
pub fn read(flat: &[f64], outer: &[f64], home: &[f64], need: Need, grad: bool) -> Option<Read> {
    let v = view(flat)?;
    if outer.len() != v.n_outer || home.len() < v.nf {
        return None;
    }
    if !v.rates.is_empty() && v.nf == 2 {
        return lines_read(flat, &v, outer, home[0], need, grad);
    }
    if !v.rates.is_empty() {
        return rate_read(flat, &v, outer, home[0], need, grad);
    }
    let s = v.solve(outer, home)?;
    let order = match need {
        Need::Point => 1,
        Need::Tangent => 2,
        Need::Curvature => 3,
    } - usize::from(!grad);
    let nf = v.nf;
    let mut ser = Series { v: &v, s: &s, order: order.max(1), memo: Vec::new() };
    let t = v.traced;
    let x = [s.z[v.n_outer + t], s.z[v.n_outer + t + 1], s.z[v.n_outer + t + 2]];
    let zero = [0.0; 3];
    let mut frame = Frame { x, d: [zero; MAX_FORMALS], dd: [[zero; MAX_FORMALS]; MAX_FORMALS] };
    if need >= Need::Tangent {
        for a in 0..nf {
            frame.d[a] = ser.power(&[a], 1)?;
        }
    }
    if need >= Need::Curvature {
        for a in 0..nf {
            for b in a..nf {
                let m = if a == b { ser.power(&[a], 2)? } else { ser.mixed(&[a, b])? };
                frame.dd[a][b] = m;
                frame.dd[b][a] = m;
            }
        }
    }
    let grad = if grad {
        // every column of the place and the geometry: the formals are the first outer
        // variables, θ the next; a curve's second place column moves nothing
        let n_c = PLACE + v.n_theta;
        let mut g = vec![[zero; SLOTS]; n_c];
        for (c, gc) in g.iter_mut().enumerate() {
            let k = match c {
                c if c < PLACE && c >= nf => continue,
                c if c < PLACE => c,
                c => nf + c - PLACE,
            };
            gc[0] = ser.power(&[k], 1)?;
            if need >= Need::Tangent {
                for a in 0..nf {
                    gc[1 + a] = ser.mixed(&[a, k])?;
                }
            }
            if need >= Need::Curvature {
                for a in 0..nf {
                    for b in 0..nf {
                        gc[1 + MAX_FORMALS + a * MAX_FORMALS + b] = ser.mixed(&[a, b, k])?;
                    }
                }
            }
        }
        Some(g)
    } else {
        None
    };
    Some(Read { frame, grad })
}

/// A body that states a rate, read at `outer[0]` (#339 D2): the flow from its home, its frame
/// and, with `grad`, the frame's derivative in its place and every `θ` column — along the formal
/// the flow's own higher orders, along a column its sensitivities.
fn rate_read(flat: &[f64], v: &View, outer: &[f64], home: f64, need: Need, grad: bool)
    -> Option<Read> {
    let body = v.rate_body();
    let u = outer[0];
    let (lo, hi) = (v.domain[0], v.domain[1]);
    let m = crate::rates::remembered(&body, flat, outer, home,
                                     crate::rates::reach(home, u, Some((lo, hi)))).ok()?;
    // a Jacobian reads one order past the frame
    let order = match need {
        Need::Point => 1,
        Need::Tangent => 2,
        Need::Curvature => 3,
    } + u8::from(grad);
    let r = body.read(&m, u, order.min(3), grad, grad)?;
    let zero = [0.0; 3];
    let mut frame =
        Frame { x: r.d[0], d: [zero; MAX_FORMALS], dd: [[zero; MAX_FORMALS]; MAX_FORMALS] };
    if need >= Need::Tangent {
        frame.d[0] = r.d[1];
    }
    if need >= Need::Curvature {
        frame.dd[0][0] = r.d[2];
    }
    let grad = grad.then(|| {
        let mut g = vec![[zero; SLOTS]; PLACE + v.n_theta];
        g[0][0] = r.d[1];
        g[0][1] = r.d[2];
        if r.orders >= 3 {
            g[0][1 + MAX_FORMALS] = r.d[3];
        }
        for (c, gc) in g.iter_mut().enumerate().skip(PLACE) {
            let j = c - PLACE;
            gc[0] = r.dc[j];
            if let Some(d) = r.dd1.get(j) {
                gc[1] = *d;
            }
            if let Some(d) = r.dd2.get(j) {
                gc[1 + MAX_FORMALS] = *d;
            }
        }
        g
    });
    Some(Read { frame, grad })
}

/// A body over two formals that states a rate, read at `outer[..2]` (#339 D4): the march along
/// the first from its home, the surface the method of lines makes of it there, and with `grad`
/// the frame's derivative in its place and every `θ` column.  A Jacobian of a curvature (a third
/// order) is not read.
fn lines_read(flat: &[f64], v: &View, outer: &[f64], home: f64, need: Need, grad: bool)
    -> Option<Read> {
    let body = v.lines_body();
    let plan = body.plan().ok()?;
    let u = outer[0];
    let to = crate::rates::reach(home, u, Some((v.domain[0], v.domain[1])));
    let m = crate::lines::remembered(&body, flat, outer, home, to, body.pieces, grad).ok()?;
    let r = body.read(&plan, &m, u, outer[1], need as u8, grad)?;
    let frame = Frame { x: r.x, d: r.d, dd: r.dd };
    let grad = r.grad.map(|g| g.into_iter().map(|[x, d0, d1]| {
        let mut slot = [[0.0; 3]; SLOTS];
        slot[0] = x;
        slot[1] = d0;
        slot[2] = d1;
        slot
    }).collect());
    Some(Read { frame, grad })
}

impl Read {
    /// The frame over numbers carrying derivatives, linearised: `x + Σ δ_c ∂x/∂c` over the body's
    /// columns `c` (place then `θ`), `δ_c` the caller's motion of column `c` (`None` where it has
    /// none) — exact to first order, all a Jacobian reads.
    fn linear<T: Num>(&self, delta: impl Fn(usize) -> Option<T>) -> Frame<T> {
        let n = self.grad.as_ref().map_or(0, |g| g.len());
        let deltas: Vec<Option<T>> = (0..n).map(&delta).collect();
        let lift = |v: [f64; 3], slot: usize| -> [T; 3] {
            std::array::from_fn(|i| {
                let mut acc = T::cst(v[i]);
                for (g, d) in self.grad.iter().flatten().zip(&deltas) {
                    if let (Some(d), true) = (d, g[slot][i] != 0.0) {
                        acc = acc + *d * T::cst(g[slot][i]);
                    }
                }
                acc
            })
        };
        let f = &self.frame;
        Frame {
            x: lift(f.x, 0),
            d: std::array::from_fn(|a| lift(f.d[a], 1 + a)),
            dd: std::array::from_fn(|a| std::array::from_fn(|b| {
                lift(f.dd[a][b], 1 + MAX_FORMALS + a * MAX_FORMALS + b)
            })),
        }
    }
}

/* -- the contacts ---------------------------------------------------------------------------- */

/// Which statement a contact with a traced body makes: a point on it.  What is read at a point
/// a contact holds — a curve's tangent, a surface's normal, a plane or a line touching it, a
/// section's osculating circle — is the contact's frame, read by a rate of it (`contact_rate`,
/// #365; `frame` here).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Contact {
    /// `p coincident k`: `p − X`.  Lead: the point's lift.
    Point = 0,
}

impl Contact {
    pub const ALL: [Contact; 1] = [Contact::Point];

    pub fn rows(self) -> usize {
        3
    }

    pub fn lead(self) -> usize {
        3
    }

    fn name(self) -> &'static str {
        "point_on_traced"
    }

    /// A point's rows are lengths.
    fn degree(self) -> u32 {
        1
    }
}

/// A contact's constants: `[n_values, values…, home…(2), body…]`.
pub fn consts(values: &[f64], home: [f64; 2], body: &Body) -> Vec<f64> {
    let mut k = vec![values.len() as f64];
    k.extend_from_slice(values);
    k.extend(home);
    k.extend_from_slice(&body.flat);
    k
}

/// The constants read back: the values, the home and the body's encoding.
fn parts(k: &[f64]) -> Option<(&[f64], [f64; 2], &[f64])> {
    let n = k.first().copied().filter(|v| v.is_finite() && *v >= 0.0 && *v <= 4096.0)? as usize;
    let values = k.get(1..1 + n)?;
    let home = [*k.get(1 + n)?, *k.get(2 + n)?];
    Some((values, home, k.get(3 + n..)?))
}

/// A contact's rows from its lead columns and the frame, into `out`.
fn rows<T: Num>(c: Contact, lead: &[T], f: &Frame<T>, out: &mut [T]) {
    match c {
        Contact::Point => {
            for i in 0..3 {
                out[i] = lead[i] - f.x[i];
            }
        }
    }
}

/// **The body's frame at a place** (#365, `contact_rate`), over columns `[u, s, θ…]` and a
/// contact's constants: the rate along formal `slot`, a direction a normal is read across (the
/// first formal's), and of a surface its unit normal and — where `second` — its two rates and its
/// second fundamental form along them; where `grad`, its derivative in each column, exact.
pub(crate) fn frame(cols: &[f64], k: &[f64], slot: usize, second: bool, grad: bool)
    -> Option<crate::contact_rate::Read> {
    use crate::contact_rate::Read as Rated;
    let (values, home, flat) = parts(k)?;
    let v = view(flat)?;
    if cols.len() != PLACE + v.n_theta || slot >= v.nf {
        return None;
    }
    let mut outer = cols[..v.nf].to_vec();
    outer.extend_from_slice(&cols[PLACE..]);
    outer.extend_from_slice(values);
    let need = if second { Need::Curvature } else { Need::Tangent };
    let r = read(flat, &outer, &home, need, grad)?;
    let surface = v.nf == 2;
    let derive = |f: Frame<f64>| derived(&f, slot, surface, second);
    if !grad {
        return Some(Rated::value(derive(r.frame)));
    }
    Rated::by_passes(cols, |c, _| {
        let f = r.linear(|q| c.get(q).map(|&x| x - Dual::<{ held::PASS }>::cst(x.val())));
        Some(derived(&f, slot, surface, second))
    })
}

/// The contact frame of a body's frame: its rate along formal `slot`, its second rate there where
/// `second`, and of a surface its normal and second order.
fn derived<T: Num>(f: &Frame<T>, slot: usize, surface: bool, second: bool)
    -> crate::contact_rate::Frame<T> {
    let mut out = crate::contact_rate::Frame::zero();
    out.x = f.x;
    out.rate = f.d[slot];
    out.across = f.d[0];
    if second {
        out.accel = f.dd[slot][slot];
    }
    if surface {
        let n = unit(cross(f.d[0], f.d[1]));
        out.n = n;
        if second {
            out.tan = [f.d[0], f.d[1]];
            out.second = [dot(f.dd[0][0], n), dot(f.dd[0][1], n), dot(f.dd[1][1], n)];
        }
    }
    out
}

/// The body's outer variables at a contact's columns: its place, its `θ` and the values.
fn outer_of(c: Contact, cols: &[f64], values: &[f64], nf: usize) -> Vec<f64> {
    let at = c.lead();
    let mut o = cols[at..at + nf].to_vec();
    o.extend_from_slice(&cols[at + PLACE..]);
    o.extend_from_slice(values);
    o
}

/// A contact's residual over its columns and constants, into `out`; `None` where they do not
/// describe one or the body cannot be solved there.
fn residual(c: Contact, cols: &[f64], k: &[f64], out: &mut [f64]) -> Option<()> {
    let (values, home, flat) = parts(k)?;
    let v = view(flat)?;
    if cols.len() != c.lead() + PLACE + v.n_theta || out.len() != c.rows()
        || c != Contact::Point {
        return None;
    }
    let outer = outer_of(c, cols, values, v.nf);
    let r = read(flat, &outer, &home, Need::Point, false)?;
    rows(c, &cols[..c.lead()], &r.frame, out);
    out.iter().all(|x| x.is_finite()).then_some(())
}

/// The same, its Jacobian row-major into `j`: the lead by `Dual` directly, the place and `θ`
/// through the frame's exact derivatives.
fn jacobian(c: Contact, cols: &[f64], k: &[f64], j: &mut [f64]) -> Option<()> {
    const P: usize = held::PASS;
    let (values, home, flat) = parts(k)?;
    let v = view(flat)?;
    let w = cols.len();
    if w != c.lead() + PLACE + v.n_theta || j.len() != c.rows() * w
        || c != Contact::Point {
        return None;
    }
    let outer = outer_of(c, cols, values, v.nf);
    let r = read(flat, &outer, &home, Need::Point, true)?;
    let mut lead = vec![Dual::<P>::cst(0.0); c.lead()];
    let mut out = vec![Dual::<P>::cst(0.0); c.rows()];
    for start in (0..w).step_by(P) {
        for (q, x) in lead.iter_mut().enumerate() {
            *x = if (start..start + P).contains(&q) {
                Dual::var(cols[q], q - start)
            } else {
                Dual::cst(cols[q])
            };
        }
        let from = c.lead();
        let f = r.linear(|q| (start..start + P).contains(&(from + q))
            .then(|| Dual::<P>::var(0.0, from + q - start)));
        rows(c, &lead, &f, &mut out);
        held::scatter_pass(&out, j, start);
    }
    Some(())
}

fn res<const C: u8>(n: usize, v: &[f64], k: &[f64], r: &mut [f64]) {
    let c = Contact::ALL[C as usize];
    held::block_res(n, v, k, r, c.rows(), |x, kc, out| residual(c, x, kc, out));
}

fn jac<const C: u8>(n: usize, v: &[f64], k: &[f64], j: &mut [f64]) {
    let c = Contact::ALL[C as usize];
    let (n_par, _) = crate::kernels::curve_widths(n, v, k);
    held::block_res(n, v, k, j, c.rows() * n_par, |x, kc, out| jacobian(c, x, kc, out));
}

/// The kernel of contact `c` at a block's widths (`system::build_kernel`).
pub fn kernel(c: Contact, n_par: usize, n_const: usize) -> Kernel {
    let (res, jac): (KernelFn, KernelFn) = match c {
        Contact::Point => (res::<0>, jac::<0>),
    };
    Kernel { name: c.name(), n_res: c.rows(), n_par, n_const, degree: c.degree(), res, jac,
             const_jac: None, form: None }
}

/* -- reading a body apart from the solve --------------------------------------------------- */

/// The traced point at a place `[u, s]` over the body's columns' values `theta` and constants:
/// its position, its derivatives in the formals and, on a surface, its unit normal.  What a
/// seed, a report and a check read.
#[derive(Clone, Copy, Debug)]
pub struct Reading {
    pub position: [f64; 3],
    pub d: [[f64; 3]; MAX_FORMALS],
    pub dd: [[[f64; 3]; MAX_FORMALS]; MAX_FORMALS],
    pub normal: Option<[f64; 3]>,
}

/// The body over a contact's constants `k` (`consts`), at `place` and over `theta`.
pub fn at(k: &[f64], theta: &[f64], place: [f64; 2], need: Need) -> Option<Reading> {
    let (values, home, flat) = parts(k)?;
    let v = view(flat)?;
    if theta.len() != v.n_theta {
        return None;
    }
    let mut outer = place[..v.nf].to_vec();
    outer.extend_from_slice(theta);
    outer.extend_from_slice(values);
    let r = read(flat, &outer, &home, need, false)?;
    let f = r.frame;
    let normal = (v.nf == 2 && need >= Need::Tangent).then(|| {
        let n = crate::space::cross(f.d[0], f.d[1]);
        let l = crate::space::norm(n);
        if l > 0.0 { n.map(|x| x / l) } else { n }
    });
    Some(Reading { position: f.x, d: f.d, dd: f.dd, normal })
}

/// Where a point `p` sits nearest on the body over a domain: a coarse grid of places, then
/// Gauss–Newton on `|X − p|²` — a seed, not a projection.  A curve's second place is 0.
pub fn nearest(k: &[f64], theta: &[f64], p: [f64; 3], domain: [[f64; 2]; 2]) -> Option<[f64; 2]> {
    // asked again for the same thing — each place slot of a contact seeds itself alike — the
    // answer is the last one: a body read by a march pays for every place it reads
    let key: Vec<u64> = k.iter().chain(theta).chain(&p).chain(domain.iter().flatten())
        .map(|x| x.to_bits()).collect();
    if let Some(found) = NEAREST.with(|c| c.borrow().as_ref().filter(|(k, _)| *k == key)
        .map(|(_, f)| *f)) {
        return found;
    }
    let found = nearest_of(k, theta, p, domain);
    NEAREST.with(|c| *c.borrow_mut() = Some((key, found)));
    found
}

thread_local! {
    /// The last `nearest` asked, and its answer.
    static NEAREST: std::cell::RefCell<Option<(Vec<u64>, Option<[f64; 2]>)>> =
        const { std::cell::RefCell::new(None) };
}

fn nearest_of(k: &[f64], theta: &[f64], p: [f64; 3], domain: [[f64; 2]; 2]) -> Option<[f64; 2]> {
    let (_, _, flat) = parts(k)?;
    let nf = view(flat)?.nf;
    let grid = |i: usize, n: usize| -> Vec<f64> {
        if i >= nf {
            return vec![0.0];
        }
        let [a, b] = domain[i];
        (0..=n).map(|j| a + (b - a) * j as f64 / n as f64).collect()
    };
    let mut best: Option<([f64; 2], f64)> = None;
    for &u in &grid(0, 24) {
        for &s in &grid(1, 12) {
            let Some(r) = at(k, theta, [u, s], Need::Point) else { continue };
            let d = crate::space::distance(r.position, p);
            if best.is_none_or(|(_, b)| d < b) {
                best = Some(([u, s], d));
            }
        }
    }
    let (mut place, _) = best?;
    for _ in 0..30 {
        let r = at(k, theta, place, Need::Tangent)?;
        let e = crate::space::sub(r.position, p);
        // the normal equations in the formals
        let g: Vec<f64> = (0..nf).map(|a| crate::space::dot(r.d[a], e)).collect();
        let h = |a: usize, b: usize| crate::space::dot(r.d[a], r.d[b]);
        let step = if nf == 1 {
            let d = h(0, 0);
            if d <= 0.0 { break }
            [-g[0] / d, 0.0]
        } else {
            let det = h(0, 0) * h(1, 1) - h(0, 1) * h(0, 1);
            if det.abs() <= 1e-300 { break }
            [-(h(1, 1) * g[0] - h(0, 1) * g[1]) / det, -(h(0, 0) * g[1] - h(0, 1) * g[0]) / det]
        };
        place = [place[0] + step[0], place[1] + step[1]];
        if step[0].abs() + step[1].abs() < 1e-13 {
            break;
        }
    }
    place.iter().all(|x| x.is_finite()).then_some(place)
}

/// Whether the body is **regular** at a place: its derivatives in the formals independent
/// (`rank [C_φ, C_s] = 2` on a surface, `C_u ≠ 0` on a curve), read as the sine between them —
/// the undercut check of a flank seen from its member (#326 R5).  `None` where it cannot be read.
pub fn regularity(k: &[f64], theta: &[f64], place: [f64; 2]) -> Option<f64> {
    let r = at(k, theta, place, Need::Tangent)?;
    match r.normal {
        Some(_) => {
            let (a, b) = (r.d[0], r.d[1]);
            let (na, nb) = (crate::space::norm(a), crate::space::norm(b));
            Some(crate::space::norm(crate::space::cross(a, b)) / (na * nb).max(1e-300))
        }
        None => Some(crate::space::norm(r.d[0])),
    }
}

/* -- reading the sketch ---------------------------------------------------------------------- */

use crate::model::{EntKind, Sketch, TracedE};

/// Frame `f`'s motion, its joint's: `None` at rest, the world, which reads no column (#383).
fn moving(sk: &Sketch, f: usize) -> Option<held::Moving> {
    let m = sk.frames.get(f)?.joint.as_ref()?.motion?;
    held::Moving::of_any(sk, m as usize).ok()
}

/// The points whose lifts a traced body's columns read: its points, its lines' ends, and every
/// point its motions' axes read.
pub fn lifted(sk: &Sketch, t: &TracedE) -> Vec<usize> {
    t.args.iter().flat_map(|e| match e.kind {
        EntKind::Point => vec![e.i()],
        EntKind::Line => {
            let l = &sk.lines[e.i()];
            vec![l.p1 as usize, l.p2 as usize]
        }
        EntKind::Frame => moving(sk, e.i()).map(|m| m.lifted()).unwrap_or_default(),
        // a spline's control points, and what a curve traced in space reads (#376)
        EntKind::Spline => sk.splines[e.i()].ctrl.iter().map(|&p| p as usize).collect(),
        EntKind::Traced => sk.traced.get(e.i()).map(|t| lifted(sk, t)).unwrap_or_default(),
        _ => Vec::new(),
    }).collect()
}

/// A traced body's columns past a contact's place, every point's lift minted: `θ` in the
/// variable table's order.
pub fn columns(sk: &Sketch, t: &TracedE) -> Vec<u32> {
    let lift = |p: usize| -> Vec<u32> {
        let k = sk.lift_of(p).expect("a traced body's points are lifted at the add");
        sk.lifts[k].x.to_vec()
    };
    t.args.iter().flat_map(|e| match e.kind {
        EntKind::Point => lift(e.i()),
        EntKind::Line => {
            let l = &sk.lines[e.i()];
            [lift(l.p1 as usize), lift(l.p2 as usize)].concat()
        }
        EntKind::Frame => moving(sk, e.i()).map(|m| m.columns(sk)).unwrap_or_default(),
        // a spline's control points' lifts, a drawn curve's own columns, and a curve traced in
        // space's (#376)
        EntKind::Spline => sk.splines[e.i()].ctrl.iter().flat_map(|&p| lift(p as usize)).collect(),
        EntKind::Curve => sk.entity_params(*e),
        EntKind::Traced => sk.traced.get(e.i()).map(|t| columns(sk, t)).unwrap_or_default(),
        _ => Vec::new(),
    }).collect()
}

/// The columns' values as the drawing stands, a point not lifted yet read where it is.
pub fn theta(sk: &Sketch, t: &TracedE) -> Vec<f64> {
    t.args.iter().flat_map(|e| match e.kind {
        EntKind::Point => sk.lifted(e.i()).to_vec(),
        EntKind::Line => {
            let l = &sk.lines[e.i()];
            [sk.lifted(l.p1 as usize), sk.lifted(l.p2 as usize)].concat()
        }
        EntKind::Frame => moving(sk, e.i()).map(|m| m.values(sk)).unwrap_or_default(),
        EntKind::Spline => {
            sk.splines[e.i()].ctrl.iter().flat_map(|&p| sk.lifted(p as usize)).collect()
        }
        EntKind::Curve => {
            sk.entity_params(*e).into_iter().map(|q| sk.params[q as usize].value).collect()
        }
        EntKind::Traced => sk.traced.get(e.i()).map(|t| theta(sk, t)).unwrap_or_default(),
        _ => Vec::new(),
    }).collect()
}

/// A contact's constants over traced body `t`.
pub fn consts_of(t: &TracedE) -> Vec<f64> {
    consts(&t.values, t.home, &t.body)
}

/// Traced body `i` at a place, as the drawing stands.
pub fn reading(sk: &Sketch, i: usize, place: [f64; 2], need: Need) -> Option<Reading> {
    let t = sk.traced.get(i)?;
    at(&consts_of(t), &theta(sk, t), place, need)
}

/// How far traced body `i`, where it states a rate (#339 D2), is integrated across its interval
/// from its home: `None` for any other body, else each way's refusal (`locus::rate_reach`).
pub fn rate_reach(sk: &Sketch, i: usize) -> Option<[crate::rates::Reached; 2]> {
    let t = sk.traced.get(i)?;
    let v = view(&t.body.flat).filter(|v| !v.rates.is_empty() && v.nf == 1)?;
    let k = consts_of(t);
    let (values, home, flat) = parts(&k)?;
    let mut outer = vec![home[0]];
    outer.extend(theta(sk, t));
    outer.extend_from_slice(values);
    Some(crate::rates::reaches(&v.rate_body(), flat, &outer, home[0], v.domain))
}

/// How far traced body `i`, where it runs over two formals and states a rate (#339 D4), is
/// marched along its first from its home: `None` for any other body, else each way's refusal and
/// why.
pub fn lines_reach(sk: &Sketch, i: usize) -> Option<[crate::lines::Reached; 2]> {
    let (flat, outer, home) = lines_of(sk, i)?;
    let v = view(&flat)?;
    Some(crate::lines::reaches(&v.lines_body(), &flat, &outer, home, v.domain))
}

/// Traced body `i`, where it runs over two formals and states a rate, as the method of lines
/// reads it (#339 D4): its body and encoding, and the outer variables at its home.
fn lines_of(sk: &Sketch, i: usize) -> Option<(Vec<f64>, Vec<f64>, f64)> {
    let t = sk.traced.get(i)?;
    view(&t.body.flat).filter(|v| !v.rates.is_empty() && v.nf == 2)?;
    let k = consts_of(t);
    let (values, home, flat) = parts(&k)?;
    let mut outer = vec![home[0], home[1]];
    outer.extend(theta(sk, t));
    outer.extend_from_slice(values);
    Some((flat.to_vec(), outer, home[0]))
}

/// Traced body `i` marched on `pieces` pieces toward `to` along its first formal, read at `place`
/// to `need` — how the gate measures the method of lines' convergence in its nodes.  `None` for
/// any other body, or where it is not read there.
pub fn lines_at(sk: &Sketch, i: usize, pieces: usize, to: f64, place: [f64; 2], need: Need)
    -> Option<crate::lines::Reading> {
    let (flat, outer, home) = lines_of(sk, i)?;
    let body = view(&flat)?.lines_body();
    let plan = body.plan().ok()?;
    let m = crate::lines::remembered(&body, &flat, &outer, home, to, pieces, false).ok()?;
    body.read(&plan, &m, place[0], place[1], need as u8, false)
}

/// How far traced body `i`, marched on `pieces` pieces toward `to`, is from the same march on
/// twice the pieces at `places` along its first formal (`lines::check`).
pub fn lines_check(sk: &Sketch, i: usize, pieces: usize, to: f64, places: &[f64]) -> Option<f64> {
    let (flat, outer, home) = lines_of(sk, i)?;
    let body = view(&flat)?.lines_body();
    crate::lines::check(&body, &flat, &outer, home, to, pieces, places)
}

/// How far traced body `i`, marched on `pieces` pieces, reaches toward `to`: there, or refused
/// and why.
pub fn lines_reached(sk: &Sketch, i: usize, pieces: usize, to: f64)
    -> Option<crate::lines::Reached> {
    let (flat, outer, home) = lines_of(sk, i)?;
    let body = view(&flat)?.lines_body();
    Some(crate::lines::remembered(&body, &flat, &outer, home, to, pieces, false)
        .and_then(|m| m.refused.map_or(Ok(()), |r| Err(Some(r)))))
}

/// Where point `p` (in space) sits nearest on traced body `i`, as the drawing stands.
pub fn place_of(sk: &Sketch, i: usize, p: [f64; 3]) -> Option<[f64; 2]> {
    let t = sk.traced.get(i)?;
    nearest(&consts_of(t), &theta(sk, t), p, t.domain)
}

/* -- a traced surface as a meshing's flank --------------------------------------------------- */

/// What marks a meshing flank's constants as a traced surface's (`envelope::meshing`): where a
/// held flank's carry its source's code.
pub const FLANK: f64 = 100.0;

/// A meshing flank's constants over traced surface `t`: `[0, FLANK, domain×4, consts…]`.
pub fn flank_consts(t: &TracedE) -> Vec<f64> {
    let d = t.domain;
    [vec![0.0, FLANK, d[0][0], d[0][1], d[1][0], d[1][1]], consts_of(t)].concat()
}

/// A traced flank's constants read back: its domain and a contact's constants.
pub(crate) fn flank_parts(k: &[f64]) -> Option<([[f64; 2]; 2], &[f64])> {
    if k.len() < 6 || k[0] != 0.0 || k[1] != FLANK {
        return None;
    }
    let rest = &k[6..];
    let (_, _, flat) = parts(rest)?;
    (view(flat)?.nf == 2).then_some(([[k[2], k[3]], [k[4], k[5]]], rest))
}

/// How many columns a traced flank reads past its place.
pub(crate) fn flank_width(k: &[f64]) -> usize {
    flank_parts(k).and_then(|(_, c)| parts(c)).and_then(|(_, _, f)| view(f))
        .map_or(0, |v| v.n_theta)
}

/// What a meshing reads of a traced flank over its columns `c` — `[u, s, t, θ…]`, the roll `t`
/// held at none — as numbers of any kind: its point, its unit normal `C_u × C_s` and `C_u`, each
/// exact in every column where the numbers carry derivatives (the frame's, linearised: a `Dual`'s
/// first order is all a meshing's Jacobian reads).  `None` where the body cannot be solved there.
pub(crate) fn seen<T: Num>(k: &[f64], c: &[T]) -> Option<([T; 3], [T; 3], [T; 3])> {
    let (values, home, flat) = parts(k)?;
    let v = view(flat)?;
    if v.nf != 2 || c.len() != 3 + v.n_theta {
        return None;
    }
    let vals: Vec<f64> = c.iter().map(|x| x.val()).collect();
    let mut outer = vec![vals[0], vals[1]];
    outer.extend_from_slice(&vals[3..]);
    outer.extend_from_slice(values);
    let r = read(flat, &outer, &home, Need::Tangent, c.iter().any(|x| x.moves()))?;
    // the place's two columns, then the body's, past the held roll
    let f = r.linear(|q| {
        let at = if q < PLACE { q } else { q + 1 };
        Some(c[at] - T::cst(vals[at]))
    });
    Some((f.x, unit(cross(f.d[0], f.d[1])), f.d[0]))
}
