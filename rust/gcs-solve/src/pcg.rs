//! **A mesh's stiffness solved by conjugate gradients** (#510), preconditioned by its corners:
//! what a plane or a solid structure too large to factor is solved by (`interior::ITERATIVE`).
//!
//! A quadratic element's field restricted to its corners is the linear element's, and a linear
//! field read at a side's middle node is the mean of the side's ends — in the element's own
//! coordinates, so a side curved onto the boundary too.  So the corners' space is a subspace of
//! the mesh's, exactly, and `P` (each corner's moves its own, each middle node's the mean of its
//! side's ends') carries it in.  The preconditioner is one two-level cycle on that pair:
//!
//! 1. a Gauss–Seidel sweep forward from zero, `(L + D) z = r`;
//! 2. the corners' correction, `z += P (PᵀKP)⁻¹ Pᵀ (r − K z)`, on a Cholesky factor of the
//!    Galerkin matrix `PᵀKP` (`cholesky::Cholesky`, minimum degree) — a twentieth of the mesh's
//!    unknowns and a fraction of its fill;
//! 3. a Gauss–Seidel sweep backward, `(U + D) z = r − L z`.
//!
//! The cycle is symmetric and positive definite, so it preconditions CG.  Whatever is held by
//! no support among the corners' moves the Galerkin matrix holds too, so a structure the
//! supports leave free fails its factor, as the mesh's would.  Everything is read off the
//! stiffness's lower triangle by column (`cholesky::Symmetric`): the forward sweep and `L z` by
//! scattering down a column, `U z` and the backward sweep by gathering it, `K x` both — one copy
//! of the matrix, which a factor fills twentyfold on a mesh in three dimensions.

use crate::cholesky::{Cholesky, NotPositive, Ordering, Symmetric};

/// Where CG stops: the residual this fraction of the right-hand side's (the moves then good to
/// about twenty times this, on the library's solids).
pub const TOLERANCE: f64 = 1e-11;

/// The most iterations CG makes before it is said not to converge.  A mesh fit for analysis
/// takes 30 to 60 (the library's solids, the conrod); one taking hundreds holds an element
/// nearly flat, where a residual this small need not mean moves as good, and is better factored.
pub const MOST: usize = 300;

/// **A stiffness held for CG, and its corners' factor.**
#[derive(Clone, Debug)]
pub struct Pcg {
    n: usize,
    /// The lower triangle by column (`Symmetric`'s), rows as `u32`.
    colptr: Vec<usize>,
    rows: Vec<u32>,
    values: Vec<f64>,
    /// Each diagonal's inverse.
    inv: Vec<f64>,
    /// Per equation, the corners' equations it is carried from and their weights (`P`'s row).
    up: Vec<[(u32, f64); 2]>,
    len: Vec<u8>,
    coarse: Cholesky,
    nc: usize,
}

/// CG did not reach `TOLERANCE` in `MOST` iterations: the residual it reached, relative.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Unconverged(pub f64);

impl Pcg {
    /// `k` (its lower triangle) preconditioned by the corners' space: `up[e]` equation `e`'s row
    /// of `P` (at most two corners' equations, each with its weight), `coarse` the Galerkin matrix
    /// `PᵀKP` over the `coarse.n` corners' equations.  `NotPositive` where the corners' factor
    /// fails (a column of the corners' space; a structure the supports leave free) or a diagonal
    /// of `k` is not positive.
    pub fn new(k: Symmetric, up: Vec<Vec<(usize, f64)>>, coarse: &Symmetric)
        -> Result<Pcg, NotPositive>
    {
        let n = k.n;
        assert!(up.len() == n, "Pcg::new: a row of P per equation");
        let mut inv = vec![0.0; n];
        for j in 0..n {
            let q = k.colptr[j];
            let d = if q < k.colptr[j + 1] && k.rowind[q] == j { k.values[q] } else { 0.0 };
            if !(d > 0.0) {
                return Err(NotPositive { column: j, pivot: d });
            }
            inv[j] = 1.0 / d;
        }
        let mut coarse_factor = Cholesky::analyse(coarse, Ordering::Amd);
        coarse_factor.factor(coarse)?;
        let len = up.iter().map(|r| r.len() as u8).collect();
        let up = up.iter().map(|r| {
            let mut row = [(0u32, 0.0); 2];
            for (t, &(c, w)) in r.iter().take(2).enumerate() {
                row[t] = (c as u32, w);
            }
            row
        }).collect();
        let rows = k.rowind.iter().map(|&i| i as u32).collect();
        Ok(Pcg { n, colptr: k.colptr, rows, values: k.values, inv, up, len, coarse: coarse_factor,
                 nc: coarse.n })
    }

    /// How many equations.
    pub fn n(&self) -> usize {
        self.n
    }

    /// `K x`.
    pub fn mul(&self, x: &[f64], y: &mut [f64]) {
        y.fill(0.0);
        for j in 0..self.n {
            let mut s = 0.0;
            for q in self.colptr[j]..self.colptr[j + 1] {
                let (i, v) = (self.rows[q] as usize, self.values[q]);
                if i == j {
                    s += v * x[j];
                } else {
                    y[i] += v * x[j];
                    s += v * x[i];
                }
            }
            y[j] += s;
        }
    }

    /// The two-level cycle on `r`: `z` its preconditioned residual, `w` a scratch the size of
    /// both.
    fn cycle(&self, r: &[f64], z: &mut [f64], w: &mut [f64], wc: &mut [f64]) {
        let n = self.n;
        // forward from zero, by columns: z_j = w_j / K_jj, then w_i −= K_ij z_j below it
        w.copy_from_slice(r);
        for j in 0..n {
            let zj = w[j] * self.inv[j];
            z[j] = zj;
            for q in self.colptr[j] + 1..self.colptr[j + 1] {
                w[self.rows[q] as usize] -= self.values[q] * zj;
            }
        }
        // its residual r − (L + D + U) z = −U z: row j of U is column j of L
        for j in 0..n {
            let mut s = 0.0;
            for q in self.colptr[j] + 1..self.colptr[j + 1] {
                s += self.values[q] * z[self.rows[q] as usize];
            }
            w[j] = -s;
        }
        // the corners' correction
        wc.fill(0.0);
        for e in 0..n {
            for t in 0..self.len[e] as usize {
                let (c, f) = self.up[e][t];
                wc[c as usize] += f * w[e];
            }
        }
        self.coarse.solve(wc);
        for e in 0..n {
            for t in 0..self.len[e] as usize {
                let (c, f) = self.up[e][t];
                z[e] += f * wc[c as usize];
            }
        }
        // backward: w = r − L z, then z_j = (w_j − Σ_{i>j} K_ij z_i) / K_jj, last first
        w.copy_from_slice(r);
        for j in 0..n {
            let zj = z[j];
            for q in self.colptr[j] + 1..self.colptr[j + 1] {
                w[self.rows[q] as usize] -= self.values[q] * zj;
            }
        }
        for j in (0..n).rev() {
            let mut s = w[j];
            for q in self.colptr[j] + 1..self.colptr[j + 1] {
                s -= self.values[q] * z[self.rows[q] as usize];
            }
            z[j] = s * self.inv[j];
        }
    }

    /// **`b` overwritten by `K⁻¹ b`**, to `TOLERANCE` of its residual: how many iterations it
    /// took, or how near it came.
    pub fn solve(&self, b: &mut [f64]) -> Result<usize, Unconverged> {
        let n = self.n;
        let norm = |v: &[f64]| v.iter().map(|x| x * x).sum::<f64>().sqrt();
        let dot = |a: &[f64], c: &[f64]| a.iter().zip(c).map(|(x, y)| x * y).sum::<f64>();
        let bn = norm(b);
        if bn == 0.0 {
            return Ok(0);
        }
        let mut r = b.to_vec();
        let mut x = vec![0.0; n];
        let mut z = vec![0.0; n];
        let mut w = vec![0.0; n];
        let mut wc = vec![0.0; self.nc];
        let mut kp = vec![0.0; n];
        self.cycle(&r, &mut z, &mut w, &mut wc);
        let mut p = z.clone();
        let mut rz = dot(&r, &z);
        let mut rel = 1.0;
        for it in 1..=MOST {
            self.mul(&p, &mut kp);
            let pkp = dot(&p, &kp);
            if !(pkp > 0.0) {
                return Err(Unconverged(rel));
            }
            let alpha = rz / pkp;
            for i in 0..n {
                x[i] += alpha * p[i];
                r[i] -= alpha * kp[i];
            }
            rel = norm(&r) / bn;
            if !rel.is_finite() {
                return Err(Unconverged(rel));
            }
            if rel <= TOLERANCE {
                b.copy_from_slice(&x);
                return Ok(it);
            }
            self.cycle(&r, &mut z, &mut w, &mut wc);
            let next = dot(&r, &z);
            let beta = next / rz;
            rz = next;
            for i in 0..n {
                p[i] = z[i] + beta * p[i];
            }
        }
        Err(Unconverged(rel))
    }
}
