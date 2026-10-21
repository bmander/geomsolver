//! The lowest eigenpairs of a sparse symmetric pencil (#510, P2): a structure's modes,
//! `K φ = ω² M φ`, and its linear buckling, `(K + λ K_G) φ = 0` — the analysis a drawing's
//! requirements will be judged by.  Dense Jacobi (`linalg::sym_eigen`) is for the small matrices
//! diagnosis reads; a mesh's pencil is sparse and only a few of its pairs are wanted.
//!
//! **Shift-invert Lanczos.**  `A x = λ B x` with B positive definite is turned into the operator
//! `T = (A − σB)⁻¹ B`, self-adjoint in the inner product `⟨x, y⟩ = xᵀ B y`, whose eigenvalues
//! `θ = 1/(λ − σ)` are largest for the λ nearest σ — and a Krylov method finds the largest first.
//! With σ below the spectrum (Sylvester's count of `A − σB`'s negative pivots zero) those are the
//! lowest.  The factor is `sparse::Ldl`'s, made once; each step is one solve and one product with
//! B.  The basis is kept B-orthonormal by full reorthogonalization, and the iteration restarts
//! thickly (Wu & Simon): the wanted Ritz vectors are kept, the rest dropped, and the residual the
//! kept ones share is the next vector.
//!
//! **Buckling** is the same iteration on `T = K⁻¹(−K_G)`, B-orthogonal in K's inner product, so
//! a load that leaves K positive definite (a supported structure) needs no shift: `θ = 1/λ`, and
//! the largest θ are the lowest positive load factors.
//!
//! **No pair is missed.**  A Krylov space holds one direction of each eigenspace, so a repeated
//! eigenvalue (a ring's modes come in pairs) is found once.  After the iteration converges,
//! Sylvester's count at just above the highest value found says how many the pencil has below
//! it: `A − μB`'s negative pivots for the modes, `K + μ K_G`'s for buckling.  When it has more
//! than were found, the iteration runs again B-orthogonal to every pair found (deflated), and
//! finds the copies the first run could not.  The count, not the iteration, is what answers.
//!
//! Every draw is the seeded `rng::Rng`'s, so a pencil has one answer.

use crate::linalg::{dot, norm, sym_eigen, Mat};
use crate::rng::Rng;
use crate::sparse::{Ldl, Sym};

/// What a solve found: the eigenvalues ascending, each with its vector and its residual.  A mode
/// is normalized in the mass (`φᵀ M φ = 1`), a buckling shape in the stiffness (`φᵀ K φ = 1`): the
/// inner product the iteration is orthogonal in, so the vectors are B-orthonormal.  A residual is
/// the pair's backward error, `‖A φ − λ B φ‖ / ((‖A‖₁ + |λ| ‖B‖₁) ‖φ‖)`, computed from the matrices
/// and not the iteration's estimate: as meaningful for a rigid-body mode at zero as for any.
#[derive(Clone, Debug)]
pub struct Pairs {
    pub values: Vec<f64>,
    pub vectors: Vec<Vec<f64>>,
    pub residuals: Vec<f64>,
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
/// `tol` times its θ — or times a ten-thousandth of the largest θ, for a pair near zero (a load
/// factor out of the load's reach), which would otherwise wait for rounding to agree.
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

/// A vector and B times it: every vector of the iteration is carried with its product, so an
/// inner product is one dot and a combination of vectors needs no further product.
type Pair = (Vec<f64>, Vec<f64>);

/// A spectral transformation: `op(x, Bx)` is `T x`, self-adjoint in `⟨x, y⟩ = xᵀ B y`.
struct Pencil<'a> {
    n: usize,
    op: &'a dyn Fn(&[f64], &[f64]) -> Vec<f64>,
    b: &'a dyn Fn(&[f64]) -> Vec<f64>,
}

fn axpy(a: f64, x: &[f64], y: &mut [f64]) {
    for (yi, xi) in y.iter_mut().zip(x) {
        *yi += a * xi;
    }
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
fn fresh(p: &Pencil, rng: &mut Rng, locked: &[Pair], basis: &[Pair]) -> Result<Pair, Refusal> {
    for _ in 0..4 {
        let mut w: Vec<f64> = (0..p.n).map(|_| rng.normal(0.0, 1.0)).collect();
        let before = norm(&w);
        orthogonalize(&mut w, locked);
        orthogonalize(&mut w, basis);
        let bw = (p.b)(&w);
        let nn = dot(&w, &bw);
        if nn < 0.0 {
            return Err(Refusal::NotDefinite);
        }
        if norm(&w) > 1e-8 * before && nn > 0.0 {
            let s = 1.0 / nn.sqrt();
            return Ok((w.iter().map(|x| x * s).collect(), bw.iter().map(|x| x * s).collect()));
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
/// (eigenvectors already found): `(θ, x, Bx)`, largest first, and the largest |θ| of the last
/// projection, the scale a θ's accuracy is judged against.  Thick-restart Lanczos with full
/// reorthogonalization: the projected operator is formed column by column from the
/// orthogonalization's coefficients, so after a restart its arrow (the kept Ritz values on the
/// diagonal, the coupling to the residual in the next column) needs no bookkeeping of its own.
fn lanczos(
    p: &Pencil,
    count: usize,
    locked: &[Pair],
    opts: &Opts,
    rng: &mut Rng,
) -> Result<(Vec<(f64, Vec<f64>, Vec<f64>)>, f64), Refusal> {
    let room = p.n.saturating_sub(locked.len());
    let count = count.min(room);
    if count == 0 {
        return Ok((Vec::new(), 0.0));
    }
    let dim = (2 * count).max(count + 20).min(room);
    let mut basis: Vec<Pair> = Vec::with_capacity(dim);
    let mut h = Mat::zeros(dim, dim);
    let mut hmax = 0.0f64;
    let mut next = fresh(p, rng, locked, &basis)?;
    for restart in 0..=opts.restarts {
        let mut f: Pair = (vec![0.0; p.n], vec![0.0; p.n]);
        let mut f_norm = 0.0;
        while basis.len() < dim {
            let j = basis.len();
            basis.push(next.clone());
            let mut w = (p.op)(&basis[j].0, &basis[j].1);
            orthogonalize(&mut w, locked);
            let c = orthogonalize(&mut w, &basis);
            for i in 0..=j {
                h.set(i, j, c[i]);
                h.set(j, i, c[i]);
                hmax = hmax.max(c[i].abs());
            }
            let bw = (p.b)(&w);
            let beta = dot(&w, &bw).max(0.0).sqrt();
            if j + 1 == dim {
                f = (w, bw);
                f_norm = beta;
            } else if beta > 1e-12 * hmax {
                let unit = |u: &[f64]| u.iter().map(|x| x / beta).collect();
                next = (unit(&w), unit(&bw));
            } else {
                // an invariant subspace: the coupling is zero, and the space goes on from a new
                // direction
                next = fresh(p, rng, locked, &basis)?;
            }
        }
        if dim == room {
            // the basis spans everything left: the projection is the operator
            f_norm = 0.0;
        }
        let (theta, y) = sym_eigen(&h);
        let order: Vec<usize> = (0..dim).rev().collect();
        let scale = theta.iter().fold(0.0f64, |m, t| m.max(t.abs()));
        let resid = |i: usize| f_norm * y.at(dim - 1, i).abs();
        let done = |i: usize| resid(i) <= opts.tol * theta[i].abs().max(1e-4 * scale);
        let converged = order[..count].iter().take_while(|&&i| done(i)).count();
        if converged == count {
            let out = order[..count]
                .iter()
                .map(|&i| {
                    let (x, bx) = combine(&basis, &y, i);
                    (theta[i], x, bx)
                })
                .collect();
            return Ok((out, scale));
        }
        if restart == opts.restarts {
            return Err(Refusal::NoConvergence { converged });
        }
        // keep the wanted pairs and half of the rest; the residual they share comes next
        let keep = count + (dim - count) / 2;
        let kept: Vec<Pair> = order[..keep].iter().map(|&i| combine(&basis, &y, i)).collect();
        basis = kept;
        h = Mat::zeros(dim, dim);
        for (k, &i) in order[..keep].iter().enumerate() {
            h.set(k, k, theta[i]);
        }
        next = if f_norm > 1e-12 * hmax {
            (f.0.iter().map(|x| x / f_norm).collect(), f.1.iter().map(|x| x / f_norm).collect())
        } else {
            fresh(p, rng, locked, &basis)?
        };
    }
    unreachable!("the last restart returns")
}

/// The iteration, then Sylvester's count, until no pair below the highest wanted is missing.
/// `value(θ)` is the pencil's eigenvalue a θ stands for (`None`: not one wanted; a θ within the
/// iteration's accuracy of zero, `1e-12` of the largest, stands for none), `below(μ)` the count
/// of eigenvalues the pencil has below μ (`None`: μ did not factor), `floor` the bottom of the
/// range the count is taken over (the shift).  Each round finds at least one pair or stops, and
/// a search the count says is short of a pair is `NoConvergence` (`NoShift` where no μ near the
/// top factors to count at), never an answer.
fn search(
    p: &Pencil,
    count: usize,
    value: &dyn Fn(f64) -> Option<f64>,
    below: &dyn Fn(f64) -> Option<usize>,
    floor: f64,
    opts: &Opts,
) -> Result<Vec<(f64, Pair)>, Refusal> {
    let want = count.min(p.n);
    let mut rng = Rng::new(opts.seed);
    let mut found: Vec<(f64, Pair)> = Vec::new();
    let mut need = want;
    for round in 0..=p.n {
        let locked: Vec<Pair> = found.iter().map(|f| f.1.clone()).collect();
        let (more, scale) = lanczos(p, need, &locked, opts, &mut rng)?;
        let mut any = false;
        for (theta, x, bx) in more {
            if theta.abs() <= 1e-12 * scale {
                continue;
            }
            if let Some(v) = value(theta) {
                found.push((v, (x, bx)));
                any = true;
            }
        }
        found.sort_by(|a, b| a.0.total_cmp(&b.0));
        if !any && round > 0 {
            // the count said a pair was missing, and the deflated iteration found none
            return Err(Refusal::NoConvergence { converged: found.len() });
        }
        if !any || found.len() >= p.n {
            break;
        }
        let top = found[found.len().min(want) - 1].0;
        let eta = 1e-6 * top.abs().max(top - floor);
        let mut counted = None;
        for k in 1..=4 {
            if let Some(c) = below(top + k as f64 * eta) {
                counted = Some((c, top + k as f64 * eta));
                break;
            }
        }
        let Some((c, mu)) = counted else { return Err(Refusal::NoShift) };
        let have = found.iter().filter(|f| f.0 < mu).count();
        if c <= have {
            break;
        }
        need = c - have;
        if found.len() + need > p.n {
            return Err(Refusal::NoConvergence { converged: have });
        }
    }
    found.truncate(want);
    Ok(found)
}

/// The pairs found, with each residual `‖l − λ r‖ / ((‖L‖ + |λ| ‖R‖) ‖x‖)` for the two sides
/// `sides(x, Bx) = (l, r)` of the pencil's equation, `L x = λ R x`, and the norms of L and R.
fn pairs(
    found: Vec<(f64, Pair)>,
    sides: &dyn Fn(&[f64], &[f64]) -> Pair,
    (nl, nr): (f64, f64),
) -> Pairs {
    let mut out = Pairs { values: Vec::new(), vectors: Vec::new(), residuals: Vec::new() };
    for (v, (x, bx)) in found {
        let (l, r) = sides(&x, &bx);
        let d: Vec<f64> = l.iter().zip(&r).map(|(p, q)| p - v * q).collect();
        let den = (nl + v.abs() * nr) * norm(&x);
        out.residuals.push(if den > 0.0 { norm(&d) / den } else { 0.0 });
        out.values.push(v);
        out.vectors.push(x);
    }
    out
}

/// Whether `m` is positive definite: it factors, and no pivot is negative.
fn definite(m: &Sym) -> Option<Ldl> {
    Ldl::new(m).filter(|f| f.negative_pivots() == 0)
}

/// The `count` lowest eigenpairs of `A x = λ B x`, A symmetric and B symmetric positive definite:
/// a structure's modes from its stiffness and mass (λ = ω²), a rigid-body motion among them as a
/// value of zero.  The shift is found, not given: a millionth of the smallest diagonal ratio
/// `A_ii / B_ii` below zero (that ratio is a Rayleigh quotient, so no eigenvalue is above it),
/// moved further down until Sylvester's count says nothing is below it.
pub fn lowest(a: &Sym, b: &Sym, count: usize, opts: &Opts) -> Result<Pairs, Refusal> {
    assert_eq!(a.n, b.n, "a pencil of matrices of different sizes");
    let n = a.n;
    if n == 0 || count == 0 {
        return Ok(pairs(Vec::new(), &|_, _| (Vec::new(), Vec::new()), (0.0, 0.0)));
    }
    if definite(b).is_none() {
        return Err(Refusal::NotDefinite);
    }
    let (da, db) = (a.diag(), b.diag());
    let ratio: Vec<f64> = (0..n).map(|i| da[i] / db[i]).collect();
    let rmin = ratio.iter().copied().fold(f64::INFINITY, f64::min);
    let scale = ratio.iter().fold(0.0f64, |m, r| m.max(r.abs()));
    let mut step = 1e-6 * if rmin > 0.0 { rmin } else if scale > 0.0 { scale } else { 1.0 };
    let mut sigma = rmin.min(0.0) - step;
    let mut factor = None;
    for _ in 0..40 {
        if let Some(f) = definite(&a.combine(1.0, b, -sigma)) {
            factor = Some(f);
            break;
        }
        step *= 10.0;
        sigma -= step;
    }
    let Some(factor) = factor else { return Err(Refusal::NoShift) };
    let op = |_: &[f64], bx: &[f64]| {
        let mut w = bx.to_vec();
        factor.solve(&mut w);
        w
    };
    let bmul = |x: &[f64]| b.mul_vec(x);
    let pencil = Pencil { n, op: &op, b: &bmul };
    let value = |theta: f64| (theta > 0.0).then(|| sigma + 1.0 / theta);
    let below = |mu: f64| Ldl::new(&a.combine(1.0, b, -mu)).map(|f| f.negative_pivots());
    let found = search(&pencil, count, &value, &below, sigma, opts)?;
    Ok(pairs(found, &|x, bx| (a.mul_vec(x), bx.to_vec()), (a.norm1(), b.norm1())))
}

/// The `count` lowest positive load factors of `(K + λ K_G) φ = 0`: K the stiffness of a
/// supported structure (positive definite, else `NotDefinite` — a rigid-body motion buckles at no
/// load), `K_G` the geometric stiffness of the reference load, and λ the multiple of that load
/// the structure buckles at.  A load that buckles nothing (all tension) has none, and fewer than
/// `count` are returned when the load reaches fewer.  The value stated is λ; the residual is the
/// pencil's, `K φ = λ (−K_G) φ`.
pub fn buckling(k: &Sym, kg: &Sym, count: usize, opts: &Opts) -> Result<Pairs, Refusal> {
    assert_eq!(k.n, kg.n, "a pencil of matrices of different sizes");
    let n = k.n;
    let minus_kg = |x: &[f64]| kg.mul_vec(x).iter().map(|v| -v).collect::<Vec<f64>>();
    if n == 0 || count == 0 {
        return Ok(pairs(Vec::new(), &|_, _| (Vec::new(), Vec::new()), (0.0, 0.0)));
    }
    let Some(factor) = definite(k) else { return Err(Refusal::NotDefinite) };
    let op = |x: &[f64], _: &[f64]| {
        let mut w = minus_kg(x);
        factor.solve(&mut w);
        w
    };
    let bmul = |x: &[f64]| k.mul_vec(x);
    let pencil = Pencil { n, op: &op, b: &bmul };
    let value = |theta: f64| (theta > 0.0).then(|| 1.0 / theta);
    let below = |mu: f64| Ldl::new(&k.combine(1.0, kg, mu)).map(|f| f.negative_pivots());
    let found = search(&pencil, count, &value, &below, 0.0, opts)?;
    Ok(pairs(found, &|x, kx| (kx.to_vec(), minus_kg(x)), (k.norm1(), kg.norm1())))
}
