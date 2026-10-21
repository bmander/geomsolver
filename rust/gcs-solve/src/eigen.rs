//! The lowest eigenpairs of a sparse symmetric pencil (#510, P2): a structure's modes,
//! `K φ = ω² M φ`, and its linear buckling, `(K + λ K_G) φ = 0` — the analysis a drawing's
//! requirements will be judged by.  Dense Jacobi (`linalg::sym_eigen`) is for the small matrices
//! diagnosis reads; a mesh's pencil is sparse and only a few of its pairs are wanted.
//!
//! **One pencil, `L x = λ R x`, one operator.**  `T = (L − σR)⁻¹ R` is self-adjoint in the inner
//! product `⟨x, y⟩ = xᵀ B y` of whichever side is positive definite, and its eigenvalues
//! `θ = 1/(λ − σ)` are largest for the λ just above σ — and a Krylov method finds the largest
//! first.  The modes are `L = K`, `R = B = M`, σ found below the spectrum, so those are the
//! lowest.  Buckling is `L = B = K`, `R = −K_G`, σ = 0: a supported structure's K is positive
//! definite, and the largest θ are the lowest positive load factors.  The factor of `L − σR` is
//! made once (`sparse::Pencil`); each step is one solve and one product.  The basis is kept
//! B-orthonormal by full reorthogonalization, and the iteration restarts thickly (Wu & Simon):
//! the wanted Ritz vectors are kept, the rest dropped, and the residual the kept ones share is
//! the next vector.
//!
//! **No pair is missed.**  A Krylov space holds one direction of each eigenspace, so a repeated
//! eigenvalue (a ring's modes come in pairs) may be found once.  After the iteration converges,
//! Sylvester's count says how many eigenvalues the pencil has below a μ just above the highest
//! found: `L − μR`'s negative pivots, less `L − σR`'s (none).  When it has more than were found,
//! the iteration runs again B-orthogonal to every pair found (deflated), and finds the copies the
//! first run did not.  The count, not the iteration, is what answers.
//!
//! Every draw is the seeded `rng::Rng`'s, so a pencil has one answer.

use crate::linalg::{dot, norm, sym_eigen, Mat};
use crate::rng::Rng;
use crate::sparse::{Ldl, Pencil, Sym};

/// What a solve found: the eigenvalues ascending, each with its vector and its residual.  A mode
/// is normalized in the mass (`φᵀ M φ = 1`), a buckling shape in the stiffness (`φᵀ K φ = 1`): the
/// inner product the iteration is orthogonal in, so the vectors are B-orthonormal.  A residual is
/// the pair's backward error, `‖L φ − λ R φ‖ / ((‖L‖₁ + |λ| ‖R‖₁) ‖φ‖)`, computed from the matrices
/// and not the iteration's estimate: as meaningful for a rigid-body mode at zero as for any.
#[derive(Clone, Debug, Default)]
pub struct Pairs {
    pub values: Vec<f64>,
    pub vectors: Vec<Vec<f64>>,
    pub residuals: Vec<f64>,
}

impl Pairs {
    /// **Its runs of one value**: the indices of consecutive values within `GAP` of the first of
    /// each run (relative to it, or to `BREAKDOWN` of the largest for a run near zero), the gap
    /// Sylvester's count reads as one value.  A value found once is a run of one.
    pub fn runs(&self) -> Vec<std::ops::Range<usize>> {
        let scale = self.values.iter().fold(0.0f64, |m, v| m.max(v.abs()));
        let mut out = Vec::new();
        let mut i = 0;
        while i < self.values.len() {
            let v = self.values[i];
            let mut j = i + 1;
            while j < self.values.len()
                && self.values[j] - v <= GAP * v.abs().max(BREAKDOWN * scale) {
                j += 1;
            }
            out.push(i..j);
            i = j;
        }
        out
    }
}

/// Why a solve has no answer.
#[derive(Clone, Debug, PartialEq)]
pub enum Refusal {
    /// The matrix the inner product is drawn from is not positive definite: a mass with a
    /// massless freedom, or for buckling a stiffness a rigid-body motion is free in.
    NotDefinite,
    /// No shift below the spectrum factored, or no value just above the highest found to take
    /// Sylvester's count at.
    NoShift,
    /// The iteration ran out of restarts with this many of the wanted pairs converged.
    NoConvergence { converged: usize },
}

/// The iteration's budget.  A Ritz pair is converged when its residual in the B-norm is below
/// `tol` times its θ, or times `FLOOR` of the largest θ for a pair nearer zero.
#[derive(Clone, Debug)]
pub struct Opts {
    pub tol: f64,
    pub restarts: usize,
    pub seed: u32,
}

impl Default for Opts {
    fn default() -> Opts {
        Opts { tol: 1e-10, restarts: 300, seed: 1 }
    }
}

/// Below this fraction of the operator's scale a coupling, or a θ, is zero: the Krylov space has
/// closed on an invariant subspace, or the θ stands for no eigenvalue (a load factor beyond the
/// load's reach).
const BREAKDOWN: f64 = 1e-12;
/// A θ below this fraction of the largest is judged at it, so a pair near zero converges to
/// rounding instead of never.
const FLOOR: f64 = 1e-4;
/// Sylvester's count is taken this far above the highest value found, relative to its distance
/// from the shift: values closer than that are one repeated value.
const GAP: f64 = 1e-6;
/// The shift's first distance below zero, relative to the least diagonal ratio `L_ii / R_ii`.
const SHIFT: f64 = 1e-6;

/// A vector and B times it: every vector of the iteration is carried with its product, so an
/// inner product is one dot and a combination of vectors needs no further product.
type Pair = (Vec<f64>, Vec<f64>);

fn scaled((x, bx): &Pair, s: f64) -> Pair {
    (x.iter().map(|v| v * s).collect(), bx.iter().map(|v| v * s).collect())
}

fn axpy(a: f64, x: &[f64], y: &mut [f64]) {
    for (yi, xi) in y.iter_mut().zip(x) {
        *yi += a * xi;
    }
}

/// The spectral transformation: `op(x, Bx)` is `T x`, self-adjoint in `⟨x, y⟩ = xᵀ B y`.
struct Operator<'a> {
    n: usize,
    op: &'a dyn Fn(&[f64], &[f64]) -> Vec<f64>,
    b: &'a Sym,
}

/// `w <- w − Σ ⟨v_i, w⟩ v_i` over `basis`, twice ("twice is enough", Kahan–Parlett), and the
/// coefficients of both passes summed: the column of the projected operator.
fn orthogonalize(w: &mut [f64], basis: &[Pair]) -> Vec<f64> {
    let mut h = vec![0.0; basis.len()];
    for _ in 0..2 {
        for (i, (v, bv)) in basis.iter().enumerate() {
            let c = dot(bv, w);
            h[i] += c;
            axpy(-c, v, w);
        }
    }
    h
}

/// A random unit vector B-orthogonal to `locked` and `basis`: the start, and the next vector when
/// the Krylov space has closed on an invariant subspace.
fn fresh(t: &Operator, rng: &mut Rng, locked: &[Pair], basis: &[Pair]) -> Result<Pair, Refusal> {
    for _ in 0..4 {
        let mut w: Vec<f64> = (0..t.n).map(|_| rng.normal(0.0, 1.0)).collect();
        let before = norm(&w);
        orthogonalize(&mut w, locked);
        orthogonalize(&mut w, basis);
        let bw = t.b.mul_vec(&w);
        let nn = dot(&w, &bw);
        if norm(&w) > 1e-8 * before && nn > 0.0 {
            return Ok(scaled(&(w, bw), 1.0 / nn.sqrt()));
        }
    }
    Err(Refusal::NoConvergence { converged: 0 })
}

/// `Σ_j basis_j · y[j][col]`, vectors and products both.
fn combine(basis: &[Pair], y: &Mat, col: usize) -> Pair {
    let n = basis[0].0.len();
    let (mut x, mut bx) = (vec![0.0; n], vec![0.0; n]);
    for (j, (v, bv)) in basis.iter().enumerate() {
        let c = y.at(j, col);
        axpy(c, v, &mut x);
        axpy(c, bv, &mut bx);
    }
    (x, bx)
}

/// The `count` eigenpairs of T with the algebraically largest θ, B-orthogonal to `locked`
/// (eigenvectors already found), largest first, and the largest |θ| of the last projection, the
/// scale a θ's accuracy is judged against.  Thick-restart Lanczos with full reorthogonalization:
/// the projected operator is formed column by column from the orthogonalization's coefficients,
/// so after a restart its arrow (the kept Ritz values on the diagonal, the coupling to the
/// residual in the next column) needs no bookkeeping of its own.
fn lanczos(
    t: &Operator,
    count: usize,
    locked: &[Pair],
    opts: &Opts,
    rng: &mut Rng,
) -> Result<(Vec<(f64, Pair)>, f64), Refusal> {
    let room = t.n.saturating_sub(locked.len());
    let count = count.min(room);
    if count == 0 {
        return Ok((Vec::new(), 0.0));
    }
    let dim = (2 * count).max(count + 20).min(room);
    // the Ritz pairs by θ, largest first: `sym_eigen` ascends
    let wanted = |k: usize| dim - 1 - k;
    let mut basis: Vec<Pair> = Vec::with_capacity(dim);
    let mut h = Mat::zeros(dim, dim);
    let mut hmax = 0.0f64;
    let mut next = fresh(t, rng, locked, &basis)?;
    for restart in 0..=opts.restarts {
        // the residual past the last column, and its B-norm
        let mut last: Option<(Pair, f64)> = None;
        while basis.len() < dim {
            let j = basis.len();
            basis.push(std::mem::take(&mut next));
            let mut w = (t.op)(&basis[j].0, &basis[j].1);
            orthogonalize(&mut w, locked);
            let c = orthogonalize(&mut w, &basis);
            for i in 0..=j {
                h.set(i, j, c[i]);
                h.set(j, i, c[i]);
                hmax = hmax.max(c[i].abs());
            }
            let bw = t.b.mul_vec(&w);
            let beta = dot(&w, &bw).max(0.0).sqrt();
            if j + 1 == dim {
                last = Some(((w, bw), beta));
            } else if beta > BREAKDOWN * hmax {
                next = scaled(&(w, bw), 1.0 / beta);
            } else {
                // an invariant subspace: the coupling is zero, and the space goes on from a new
                // direction
                next = fresh(t, rng, locked, &basis)?;
            }
        }
        // where the basis spans everything left, the projection is the operator
        let (f, f_norm) = match last {
            Some((f, b)) if dim < room => (f, b),
            _ => ((Vec::new(), Vec::new()), 0.0),
        };
        let (theta, y) = sym_eigen(&h);
        let scale = theta.iter().fold(0.0f64, |m, t| m.max(t.abs()));
        let done = |i: usize| {
            f_norm * y.at(dim - 1, i).abs() <= opts.tol * theta[i].abs().max(FLOOR * scale)
        };
        let converged = (0..count).take_while(|&k| done(wanted(k))).count();
        if converged == count {
            let out = (0..count).map(|k| (theta[wanted(k)], combine(&basis, &y, wanted(k))));
            return Ok((out.collect(), scale));
        }
        if restart == opts.restarts {
            return Err(Refusal::NoConvergence { converged });
        }
        // keep the wanted pairs and half of the rest; the residual they share comes next
        let keep = count + (dim - count) / 2;
        basis = (0..keep).map(|k| combine(&basis, &y, wanted(k))).collect();
        h = Mat::zeros(dim, dim);
        for k in 0..keep {
            h.set(k, k, theta[wanted(k)]);
        }
        next = if f_norm > BREAKDOWN * hmax {
            scaled(&f, 1.0 / f_norm)
        } else {
            fresh(t, rng, locked, &basis)?
        };
    }
    unreachable!("the last restart returns")
}

/// Whether `m` is positive definite: it factors, and no pivot is negative.
fn definite(m: Option<Ldl>) -> Option<Ldl> {
    m.filter(|f| f.negative_pivots() == 0)
}

/// The `count` lowest eigenpairs of `L x = λ R x` above the shift: the iteration, then Sylvester's
/// count, until no pair below the highest wanted is missing.  `inner_l` draws the inner product
/// from L (buckling, σ = 0) rather than R (the modes, σ found).  Each round finds at least one
/// pair or stops, and a search the count says is short of a pair is `NoConvergence` (`NoShift`
/// where no μ near the top factors to count at), never an answer.
fn solve(l: &Sym, r: &Sym, inner_l: bool, count: usize, opts: &Opts) -> Result<Pairs, Refusal> {
    let n = l.n;
    if n == 0 || count == 0 {
        return Ok(Pairs::default());
    }
    let pencil = Pencil::new(l, r);
    let (b, sigma, factor) = if inner_l {
        (l, 0.0, definite(pencil.factor(0.0)).ok_or(Refusal::NotDefinite)?)
    } else {
        definite(Ldl::new(r)).ok_or(Refusal::NotDefinite)?;
        // a Rayleigh quotient, so no eigenvalue is above the least ratio; moved down until
        // Sylvester's count says nothing is below
        let (dl, dr) = (l.diag(), r.diag());
        let ratio: Vec<f64> = (0..n).map(|i| dl[i] / dr[i]).collect();
        let rmin = ratio.iter().copied().fold(f64::INFINITY, f64::min);
        let top = ratio.iter().fold(0.0f64, |m, q| m.max(q.abs()));
        let mut step = SHIFT * if rmin > 0.0 { rmin } else if top > 0.0 { top } else { 1.0 };
        let mut sigma = rmin.min(0.0) - step;
        let mut factor = None;
        for _ in 0..40 {
            factor = definite(pencil.factor(sigma));
            if factor.is_some() {
                break;
            }
            step *= 10.0;
            sigma -= step;
        }
        (r, sigma, factor.ok_or(Refusal::NoShift)?)
    };
    let op = |x: &[f64], bx: &[f64]| {
        let mut w = if inner_l { r.mul_vec(x) } else { bx.to_vec() };
        factor.solve(&mut w);
        w
    };
    let t = Operator { n, op: &op, b };

    let want = count.min(n);
    let mut rng = Rng::new(opts.seed);
    let (mut values, mut locked): (Vec<f64>, Vec<Pair>) = (Vec::new(), Vec::new());
    let mut need = want;
    for round in 0..=n {
        let (more, scale) = lanczos(&t, need, &locked, opts, &mut rng)?;
        let before = values.len();
        for (theta, pair) in more {
            // a θ of zero, or a negative one, is no eigenvalue above the shift
            if theta > BREAKDOWN * scale {
                values.push(sigma + 1.0 / theta);
                locked.push(pair);
            }
        }
        let any = values.len() > before;
        if !any && round > 0 {
            // the count said a pair was missing, and the deflated iteration found none
            return Err(Refusal::NoConvergence { converged: values.len() });
        }
        if !any || values.len() >= n {
            break;
        }
        let mut sorted = values.clone();
        sorted.sort_by(|a, b| a.total_cmp(b));
        let top = sorted[sorted.len().min(want) - 1];
        let eta = GAP * top.abs().max(top - sigma);
        let (c, mu) = (1..=4)
            .map(|k| top + k as f64 * eta)
            .find_map(|mu| pencil.factor(mu).map(|f| (f.negative_pivots(), mu)))
            .ok_or(Refusal::NoShift)?;
        let have = values.iter().filter(|&&v| v < mu).count();
        if c <= have {
            break;
        }
        need = c - have;
        if values.len() + need > n {
            return Err(Refusal::NoConvergence { converged: have });
        }
    }
    let mut order: Vec<usize> = (0..values.len()).collect();
    order.sort_by(|&i, &j| values[i].total_cmp(&values[j]));
    order.truncate(want);
    let (nl, nr) = (l.norm1(), r.norm1());
    let mut out = Pairs::default();
    for i in order {
        let (v, x) = (values[i], &locked[i].0);
        let (lx, rx) = (l.mul_vec(x), r.mul_vec(x));
        let d: Vec<f64> = lx.iter().zip(&rx).map(|(p, q)| p - v * q).collect();
        let den = (nl + v.abs() * nr) * norm(x);
        out.residuals.push(if den > 0.0 { norm(&d) / den } else { 0.0 });
        out.values.push(v);
        out.vectors.push(x.clone());
    }
    Ok(out)
}

/// The `count` lowest eigenpairs of `A x = λ B x`, A symmetric and B symmetric positive definite:
/// a structure's modes from its stiffness and mass (λ = ω²), a rigid-body motion among them as a
/// value of zero.  The shift is found, not given: `SHIFT` of the smallest diagonal ratio
/// `A_ii / B_ii` below zero (that ratio is a Rayleigh quotient, so no eigenvalue is above it),
/// moved further down until Sylvester's count says nothing is below it.
pub fn lowest(a: &Sym, b: &Sym, count: usize, opts: &Opts) -> Result<Pairs, Refusal> {
    solve(a, b, false, count, opts)
}

/// The `count` lowest positive load factors of `(K + λ K_G) φ = 0`: K the stiffness of a
/// supported structure (positive definite, else `NotDefinite` — a rigid-body motion buckles at no
/// load), `K_G` the geometric stiffness of the reference load, and λ the multiple of that load
/// the structure buckles at.  A load that buckles nothing (all tension) has none, and fewer than
/// `count` are returned when the load reaches fewer.  The pencil is `K φ = λ (−K_G) φ`.
pub fn buckling(k: &Sym, kg: &Sym, count: usize, opts: &Opts) -> Result<Pairs, Refusal> {
    solve(k, &kg.scaled(-1.0), true, count, opts)
}
