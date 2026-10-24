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
//! stiffness's lower triangle by column (`Lower`, assembled in place): the forward sweep and
//! `L z` by scattering down a column, `U z` and the backward sweep by gathering it, `K x` both —
//! one copy of the matrix, which a factor fills twentyfold on a mesh in three dimensions.

use crate::cholesky::{Cholesky, Ordering, Symmetric};

/// Where CG stops: the residual this fraction of the right-hand side's (the moves then good to
/// about twenty times this, on the library's solids).
pub const TOLERANCE: f64 = 1e-11;

/// The most iterations CG makes before it is said not to converge.  A mesh fit for analysis
/// takes 30 to 60 (the library's solids, the conrod); one taking hundreds holds an element
/// nearly flat, where a residual this small need not mean moves as good, and is better factored.
pub const MOST: usize = 300;

/// **A symmetric matrix's lower triangle by column, its pattern a mesh's** (`pattern`): column
/// `j`'s rows `rows[colptr[j]..colptr[j + 1]]`, increasing, each `>= j`, with their `values`.
/// Assembled in place (`add`) — never as triplets, which for a solid of 385,368 equations held
/// 18 million of them, copied once to sort and doubled as they grew: three gigabytes at its peak,
/// where the matrix itself is 175 MB.
#[derive(Clone, Debug)]
pub struct Lower {
    pub n: usize,
    pub colptr: Vec<usize>,
    pub rows: Vec<u32>,
    pub values: Vec<f64>,
}

impl Lower {
    /// **The pattern of a mesh's equations**, its values zero: an entry wherever two equations'
    /// nodes share an element.  `elements` are the elements' nodes, node `m`'s degrees of freedom
    /// are `per·m..per·m + per`, and `eq` numbers them — `None` where a support holds one —
    /// increasing with the degree of freedom, so a column's rows come out in order.
    pub fn pattern<'a>(nodes: usize, per: usize, elements: impl IntoIterator<Item = &'a [usize]>,
                       eq: &[Option<usize>], n: usize) -> Lower
    {
        // every element's nodes, and the elements each node is in
        let (mut flat, mut starts) = (Vec::new(), vec![0usize]);
        for ns in elements {
            flat.extend(ns.iter().map(|&v| v as u32));
            starts.push(flat.len());
        }
        let mut inc = vec![0usize; nodes + 1];
        for &v in &flat {
            inc[v as usize + 1] += 1;
        }
        for v in 0..nodes {
            inc[v + 1] += inc[v];
        }
        let mut of = vec![0u32; flat.len()];
        let mut at = inc.clone();
        for e in 0..starts.len() - 1 {
            for &v in &flat[starts[e]..starts[e + 1]] {
                of[at[v as usize]] = e as u32;
                at[v as usize] += 1;
            }
        }
        // each node's neighbours at or after it, in order
        let (mut adj, mut adjp): (Vec<u32>, _) = (Vec::new(), vec![0usize]);
        let mut near: Vec<u32> = Vec::new();
        for v in 0..nodes {
            near.clear();
            for &e in &of[inc[v]..inc[v + 1]] {
                let e = e as usize;
                near.extend(flat[starts[e]..starts[e + 1]].iter().filter(|&&m| m as usize >= v));
            }
            near.sort_unstable();
            near.dedup();
            adj.extend_from_slice(&near);
            adjp.push(adj.len());
        }
        drop((flat, starts, of, inc));
        // each equation's column: its node's neighbours' equations from its own on, counted
        // first so the rows are allocated once
        let column = |d: usize, e: usize, out: &mut dyn FnMut(usize)| {
            let v = d / per;
            for &m in &adj[adjp[v]..adjp[v + 1]] {
                for k in 0..per {
                    if let Some(r) = eq[m as usize * per + k].filter(|&r| r >= e) {
                        out(r);
                    }
                }
            }
        };
        let mut colptr = vec![0usize; n + 1];
        for (d, e) in eq.iter().enumerate() {
            let Some(e) = *e else { continue };
            let mut count = 0;
            column(d, e, &mut |_| count += 1);
            colptr[e + 1] = count;
        }
        for e in 0..n {
            colptr[e + 1] += colptr[e];
        }
        let mut rows = Vec::with_capacity(colptr[n]);
        for (d, e) in eq.iter().enumerate() {
            let Some(e) = *e else { continue };
            column(d, e, &mut |r| rows.push(r as u32));
        }
        let values = vec![0.0; rows.len()];
        Lower { n, colptr, rows, values }
    }

    /// Add element matrix `m` at the equations `eqs` (`None` where held: left out), its lower
    /// triangle into the pattern.
    pub fn add<const M: usize>(&mut self, m: &[[f64; M]; M], eqs: &[Option<usize>; M]) {
        for a in 0..M {
            let Some(ea) = eqs[a] else { continue };
            for b in 0..M {
                let Some(eb) = eqs[b] else { continue };
                if ea < eb || m[a][b] == 0.0 {
                    continue;
                }
                let (lo, hi) = (self.colptr[eb], self.colptr[eb + 1]);
                let at = self.rows[lo..hi].binary_search(&(ea as u32))
                    .expect("Lower::add: an entry outside the pattern");
                self.values[lo + at] += m[a][b];
            }
        }
    }

    /// As `cholesky::Symmetric`, for a factor.
    pub fn symmetric(self) -> Symmetric {
        Symmetric { n: self.n, colptr: self.colptr,
                    rowind: self.rows.into_iter().map(|r| r as usize).collect(),
                    values: self.values }
    }
}

/// **A stiffness held for CG, and its corners' factor.**
#[derive(Clone, Debug)]
pub struct Pcg {
    k: Lower,
    /// Each diagonal's inverse.
    inv: Vec<f64>,
    /// Per equation, the corners' equations it is carried from and their weights (`P`'s row: an
    /// unused place is `(0, 0.0)`, which carries nothing).
    up: Vec<[(u32, f64); 2]>,
    coarse: Cholesky,
    nc: usize,
}

impl Pcg {
    /// `k` (its lower triangle) preconditioned by the corners' space: `up[e]` equation `e`'s row
    /// of `P` (at most two corners' equations, each with its weight), `coarse` the Galerkin
    /// matrix `PᵀKP` over the `coarse.n` corners' equations — or `k` back where the corners'
    /// factor fails (a column of the corners' space; a structure the supports leave free) or a
    /// diagonal of `k` is not positive, for a factor to say why.
    pub fn new(k: Lower, up: Vec<[(u32, f64); 2]>, coarse: &Symmetric) -> Result<Pcg, Lower> {
        let n = k.n;
        assert!(up.len() == n, "Pcg::new: a row of P per equation");
        let mut inv = vec![0.0; n];
        for j in 0..n {
            let q = k.colptr[j];
            let d = if q < k.colptr[j + 1] && k.rows[q] as usize == j { k.values[q] } else { 0.0 };
            if !(d > 0.0) {
                return Err(k);
            }
            inv[j] = 1.0 / d;
        }
        // its pattern's analysis remembered between evaluations (`cholesky::analysed`): a row
        // over a mesh held through a solve assembles the same pattern at every step (#567)
        let mut factor = crate::cholesky::analysed(coarse, Ordering::Amd);
        if factor.factor(coarse).is_err() {
            return Err(k);
        }
        Ok(Pcg { k, inv, up, coarse: factor, nc: coarse.n })
    }

    /// The stiffness it holds, for a factor where CG does not settle.
    pub fn into_lower(self) -> Lower {
        self.k
    }

    /// `K x`.
    pub fn mul(&self, x: &[f64], y: &mut [f64]) {
        let k = &self.k;
        y.fill(0.0);
        for j in 0..k.n {
            let mut s = 0.0;
            for q in k.colptr[j]..k.colptr[j + 1] {
                let (i, v) = (k.rows[q] as usize, k.values[q]);
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
        let k = &self.k;
        // column j below its diagonal, as (row, entry)
        let below = |j: usize| {
            (k.colptr[j] + 1..k.colptr[j + 1]).map(|q| (k.rows[q] as usize, k.values[q]))
        };
        // forward from zero, by columns: z_j = w_j / K_jj, then w_i −= K_ij z_j below it
        w.copy_from_slice(r);
        for j in 0..k.n {
            let zj = w[j] * self.inv[j];
            z[j] = zj;
            for (i, v) in below(j) {
                w[i] -= v * zj;
            }
        }
        // its residual r − (L + D + U) z = −U z: row j of U is column j of L
        for j in 0..k.n {
            w[j] = -below(j).map(|(i, v)| v * z[i]).sum::<f64>();
        }
        // the corners' correction
        wc.fill(0.0);
        for (e, row) in self.up.iter().enumerate() {
            for &(c, f) in row {
                wc[c as usize] += f * w[e];
            }
        }
        self.coarse.solve(wc);
        for (e, row) in self.up.iter().enumerate() {
            for &(c, f) in row {
                z[e] += f * wc[c as usize];
            }
        }
        // backward: w = r − L z, then z_j = (w_j − Σ_{i>j} K_ij z_i) / K_jj, last first
        w.copy_from_slice(r);
        for j in 0..k.n {
            let zj = z[j];
            for (i, v) in below(j) {
                w[i] -= v * zj;
            }
        }
        for j in (0..k.n).rev() {
            z[j] = (w[j] - below(j).map(|(i, v)| v * z[i]).sum::<f64>()) * self.inv[j];
        }
    }

    /// **`b` overwritten by `K⁻¹ b`**, to `TOLERANCE` of its residual: how many iterations it
    /// took, or `None` where it did not settle in `MOST` (`b` then as it was).
    pub fn solve(&self, b: &mut [f64]) -> Option<usize> {
        use crate::linalg::{dot, norm};
        let n = self.k.n;
        let bn = norm(b);
        if bn == 0.0 {
            return Some(0);
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
        for it in 1..=MOST {
            self.mul(&p, &mut kp);
            let pkp = dot(&p, &kp);
            if !(pkp > 0.0) {
                return None;
            }
            let alpha = rz / pkp;
            for i in 0..n {
                x[i] += alpha * p[i];
                r[i] -= alpha * kp[i];
            }
            let rel = norm(&r) / bn;
            if !rel.is_finite() {
                return None;
            }
            if rel <= TOLERANCE {
                b.copy_from_slice(&x);
                return Some(it);
            }
            self.cycle(&r, &mut z, &mut w, &mut wc);
            let next = dot(&r, &z);
            let beta = next / rz;
            rz = next;
            for i in 0..n {
                p[i] = z[i] + beta * p[i];
            }
        }
        None
    }
}
