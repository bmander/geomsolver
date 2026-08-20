//! Compile a sketch to a flat evaluation plan; evaluate r(z), J(z); solve.
//!
//! `System` groups the sketch's constraints by kernel type into *blocks* — pure arrays of (kernel
//! id, global parameter indices, constants) — and owns the residual/Jacobian loop, the sparsity
//! structure and the solve iteration.  The Jacobian's structure (CSR indices, duplicate-summing
//! scatter map) is computed once at compile time; each evaluation only refills `data`.
//!
//! This compile-once / evaluate-many seam is the architectural boundary the program's Stage 1
//! calls for: the object model stays out of the hot loop.

use crate::kernels::{self, Kernel};
use crate::linalg::Mat;
use crate::model::Sketch;
use crate::sparse::Ata;
use std::collections::BTreeMap;

/// Free params up to which J is dense (exact minimum-norm step + rank); sparse normal equations
/// above.
pub const DENSE_MAX: usize = 120;

pub struct Block {
    pub kid: usize,
    pub count: usize,
    pub row0: usize,
    /// (count * n_par) global parameter index per local column.
    pub gidx: Vec<i32>,
    /// (count * n_const)
    pub consts: Vec<f64>,
    pub cids: Vec<u32>,
    jac_off: usize,
}

pub struct System {
    pub n_params: usize,
    pub free: Vec<i32>,
    pub n_free: usize,
    pub col_of: Vec<i32>,
    pub n_res: usize,
    pub extent: f64,
    /// Residual units for squared distances: `max(1, extent)²`.
    pub scale: f64,
    /// Residual units per row: `max(1, extent)^degree` for the row's kernel.  Kernels are not
    /// all written to the same power of length, so one system-wide scale judges half of them
    /// against a tolerance meant for the other half.
    pub row_scale: Vec<f64>,
    /// The smallest `row_scale` over hard rows — the strictest tolerance in the system, which is
    /// what the inner solver has to iterate to for every row to come in under its own.
    pub min_hard_scale: f64,
    /// One flag per residual row: rows that must be satisfied.
    pub hard: Vec<bool>,
    pub blocks: Vec<Block>,
    /// Constraint ids in block order (the order `constraint_errors` reports in).
    pub cids: Vec<u32>,
    pub csr_indptr: Vec<i32>,
    pub csr_indices: Vec<i32>,
    pub nnz: usize,
    x: Vec<f64>,
    jdata: Vec<f64>,
    ent_src: Vec<i32>,
    ent_slot: Vec<i32>,
    csr_data: Vec<f64>,
    slot_of: BTreeMap<u32, (usize, usize)>,
    ata: Option<Ata>,
}

fn k(kid: usize) -> &'static Kernel {
    kernels::kernel_by_id(kid)
}

impl System {
    pub fn new(sk: &Sketch) -> System {
        let n = sk.params.len();
        let free = sk.free_indices();
        let n_free = free.len();
        let mut col_of = vec![-1i32; n];
        for (i, &p) in free.iter().enumerate() {
            col_of[p as usize] = i as i32;
        }
        let extent = sk.extent();
        let scale = extent.max(1.0).powi(2);

        // group by kernel id, then sketch order — deterministic
        let mut by_kernel: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
        for (i, c) in sk.constraints.iter().enumerate() {
            by_kernel.entry(c.kernel_id()).or_default().push(i);
        }

        let mut blocks: Vec<Block> = Vec::new();
        let mut slot_of = BTreeMap::new();
        let mut cids: Vec<u32> = Vec::new();
        let mut hard: Vec<bool> = Vec::new();
        let mut row0 = 0usize;
        let mut joff = 0usize;
        for (&kid, idxs) in by_kernel.iter() {
            let kn = k(kid);
            let nb = idxs.len();
            let mut gidx = Vec::with_capacity(nb * kn.n_par);
            let mut consts = Vec::with_capacity(nb * kn.n_const);
            let mut bcids = Vec::with_capacity(nb);
            for (i, &ci) in idxs.iter().enumerate() {
                let c = &sk.constraints[ci];
                let ps = c.params(sk);
                debug_assert_eq!(ps.len(), kn.n_par, "{:?} params", c.kind);
                for p in ps {
                    gidx.push(p as i32);
                }
                if kn.n_const > 0 {
                    consts.extend(c.consts());
                }
                bcids.push(c.id);
                slot_of.insert(c.id, (blocks.len(), i));
                cids.push(c.id);
                for _ in 0..kn.n_res {
                    hard.push(!c.soft);
                }
            }
            blocks.push(Block { kid, count: nb, row0, gidx, consts, cids: bcids, jac_off: joff });
            row0 += nb * kn.n_res;
            joff += nb * kn.n_res * kn.n_par;
        }
        let n_res = row0;

        let mut jdata = vec![0.0; joff.max(1)];
        // constant Jacobians are filled once and never recomputed
        for b in &blocks {
            let kn = k(b.kid);
            if let Some(cj) = kn.const_jac {
                let sz = kn.n_res * kn.n_par;
                for i in 0..b.count {
                    jdata[b.jac_off + i * sz..b.jac_off + (i + 1) * sz].copy_from_slice(cj);
                }
            }
        }

        // Jacobian structure: entry (block, i, res, par) -> (row, col), duplicates merged
        let ncols = n_free.max(1) as i64;
        let mut es: Vec<(i64, i32)> = Vec::with_capacity(joff);
        for b in &blocks {
            let kn = k(b.kid);
            for i in 0..b.count {
                for t in 0..kn.n_res {
                    for c in 0..kn.n_par {
                        let col = col_of[b.gidx[i * kn.n_par + c] as usize];
                        if col < 0 {
                            continue;
                        }
                        let row = (b.row0 + i * kn.n_res + t) as i64;
                        let src = (b.jac_off + (i * kn.n_res + t) * kn.n_par + c) as i32;
                        es.push((row * ncols + col as i64, src));
                    }
                }
            }
        }
        es.sort_by_key(|e| e.0);
        let ne = es.len();
        let mut ent_src = vec![0i32; ne];
        let mut ent_slot = vec![0i32; ne];
        let mut csr_indices: Vec<i32> = Vec::with_capacity(ne);
        let mut csr_indptr = vec![0i32; n_res + 1];
        let mut nnz = 0usize;
        for e in 0..ne {
            if e == 0 || es[e].0 != es[e - 1].0 {
                csr_indices.push((es[e].0 % ncols) as i32);
                csr_indptr[(es[e].0 / ncols) as usize + 1] = nnz as i32 + 1;
                nnz += 1;
            }
            ent_src[e] = es[e].1;
            ent_slot[e] = nnz as i32 - 1;
        }
        for i in 1..=n_res {
            if csr_indptr[i] < csr_indptr[i - 1] {
                csr_indptr[i] = csr_indptr[i - 1];
            }
        }

        let mut row_scale = vec![1.0; n_res];
        for b in &blocks {
            let kn = k(b.kid);
            let sc = extent.max(1.0).powi(kn.degree as i32);
            for r in b.row0..b.row0 + b.count * kn.n_res {
                row_scale[r] = sc;
            }
        }
        let mut min_hard_scale = f64::INFINITY;
        for r in 0..n_res {
            if hard[r] {
                min_hard_scale = min_hard_scale.min(row_scale[r]);
            }
        }
        if !min_hard_scale.is_finite() {
            min_hard_scale = extent.max(1.0);
        }

        System {
            n_params: n,
            free,
            n_free,
            col_of,
            n_res,
            extent,
            scale,
            row_scale,
            min_hard_scale,
            hard,
            blocks,
            cids,
            csr_indptr,
            csr_indices,
            nnz,
            x: sk.get_x(),
            jdata,
            ent_src,
            ent_slot,
            csr_data: vec![0.0; nnz.max(1)],
            slot_of,
            ata: None,
        }
    }

    // -- constants -----------------------------------------------------------

    /// Push a constraint's (mutated) constants into the compiled plan — a moving drag target or
    /// an edited dimension.  Topology is unchanged, so no recompile.
    pub fn update_consts(&mut self, sk: &Sketch, cid: u32) {
        let Some(&(b, i)) = self.slot_of.get(&cid) else { return };
        let kn = k(self.blocks[b].kid);
        if kn.n_const == 0 {
            return;
        }
        let Some(c) = sk.constraint(cid) else { return };
        let vals = c.consts();
        self.blocks[b].consts[i * kn.n_const..(i + 1) * kn.n_const].copy_from_slice(&vals);
    }

    /// Re-read every constraint's constants (after arbitrary dimension edits).
    pub fn refresh_consts(&mut self, sk: &Sketch) {
        for b in self.blocks.iter_mut() {
            let kn = k(b.kid);
            if kn.n_const == 0 {
                continue;
            }
            for (i, &cid) in b.cids.iter().enumerate() {
                if let Some(c) = sk.constraint(cid) {
                    b.consts[i * kn.n_const..(i + 1) * kn.n_const].copy_from_slice(&c.consts());
                }
            }
        }
    }

    /// First residual row of a constraint — `None` for one this plan was not compiled from.
    pub fn row_of(&self, cid: u32) -> Option<usize> {
        let &(b, i) = self.slot_of.get(&cid)?;
        Some(self.blocks[b].row0 + i * k(self.blocks[b].kid).n_res)
    }

    // -- evaluation ----------------------------------------------------------

    /// Free values of the current sketch geometry (also refreshes our copy of x).
    pub fn z0(&mut self, sk: &Sketch) -> Vec<f64> {
        self.x = sk.get_x();
        self.free.iter().map(|&i| self.x[i as usize]).collect()
    }

    pub fn full_x(&self, z: &[f64]) -> Vec<f64> {
        let mut x = self.x.clone();
        for (i, &p) in self.free.iter().enumerate() {
            x[p as usize] = z[i];
        }
        x
    }

    fn apply_z(&mut self, z: &[f64]) {
        for (i, &p) in self.free.iter().enumerate() {
            self.x[p as usize] = z[i];
        }
    }

    pub fn residuals_into(&mut self, z: &[f64], r: &mut [f64]) {
        self.apply_z(z);
        let mut v: Vec<f64> = Vec::new();
        for b in &self.blocks {
            let kn = k(b.kid);
            let len = b.count * kn.n_par;
            v.clear();
            v.reserve(len);
            for t in 0..len {
                v.push(self.x[b.gidx[t] as usize]);
            }
            let rows = b.count * kn.n_res;
            (kn.res)(b.count, &v, &b.consts, &mut r[b.row0..b.row0 + rows]);
        }
    }

    pub fn residuals(&mut self, z: &[f64]) -> Vec<f64> {
        let mut r = vec![0.0; self.n_res];
        self.residuals_into(z, &mut r);
        r
    }

    fn jac_blocks(&mut self, z: &[f64]) {
        self.apply_z(z);
        let mut v: Vec<f64> = Vec::new();
        for b in &self.blocks {
            let kn = k(b.kid);
            if kn.const_jac.is_some() {
                continue;
            }
            let len = b.count * kn.n_par;
            v.clear();
            v.reserve(len);
            for t in 0..len {
                v.push(self.x[b.gidx[t] as usize]);
            }
            let sz = b.count * kn.n_res * kn.n_par;
            (kn.jac)(b.count, &v, &b.consts, &mut self.jdata[b.jac_off..b.jac_off + sz]);
        }
    }

    /// Refill the Jacobian's CSR values at z (the structure never changes).
    pub fn compute_csr(&mut self, z: &[f64]) -> &[f64] {
        self.jac_blocks(z);
        for v in self.csr_data.iter_mut() {
            *v = 0.0;
        }
        for e in 0..self.ent_src.len() {
            self.csr_data[self.ent_slot[e] as usize] += self.jdata[self.ent_src[e] as usize];
        }
        &self.csr_data
    }

    pub fn jacobian_dense(&mut self, z: &[f64]) -> Mat {
        let mut j = Mat::zeros(self.n_res, self.n_free);
        if self.n_free == 0 {
            return j;
        }
        self.compute_csr(z);
        for r in 0..self.n_res {
            for p in self.csr_indptr[r]..self.csr_indptr[r + 1] {
                j.data[r * self.n_free + self.csr_indices[p as usize] as usize] =
                    self.csr_data[p as usize];
            }
        }
        j
    }

    /// max |r| over hard rows at z — what "solved" means.
    pub fn max_hard_residual(&mut self, z: &[f64]) -> f64 {
        let r = self.residuals(z);
        let mut mx = 0.0f64;
        for i in 0..self.n_res {
            if self.hard[i] {
                if r[i].is_nan() {
                    return f64::NAN; // NaN is not "no error": it must not read as converged
                }
                let a = r[i].abs();
                if a > mx {
                    mx = a;
                }
            }
        }
        mx
    }

    /// max |residual| / (that row's units) over the hard rows — dimensionless, so one threshold
    /// judges every kernel.  This, not `max_hard_residual`, is what "solved" means.
    pub fn max_relative_residual(&mut self, z: &[f64]) -> f64 {
        let r = self.residuals(z);
        let mut mx = 0.0f64;
        for i in 0..self.n_res {
            if self.hard[i] {
                if r[i].is_nan() {
                    return f64::NAN;
                }
                let a = r[i].abs() / self.row_scale[i];
                if a > mx {
                    mx = a;
                }
            }
        }
        mx
    }

    /// The units of a constraint's residual, for judging `constraint_errors` against.
    pub fn constraint_scale(&self, cid: u32) -> f64 {
        match self.slot_of.get(&cid) {
            Some(&(b, _)) => self.extent.max(1.0).powi(k(self.blocks[b].kid).degree as i32),
            None => self.scale,
        }
    }

    /// max |residual| per constraint, in block order (`self.cids`).
    /// How many constraints this plan was compiled from — the length `constraint_errors`
    /// reports, which is the sketch's count only until the sketch is edited.
    pub fn n_constraints(&self) -> usize {
        self.cids.len()
    }

    pub fn constraint_errors(&mut self, z: &[f64]) -> Vec<f64> {
        let r = self.residuals(z);
        let mut out = Vec::with_capacity(self.cids.len());
        for b in &self.blocks {
            let kn = k(b.kid);
            for i in 0..b.count {
                let mut mx = 0.0f64;
                for t in 0..kn.n_res {
                    let v = r[b.row0 + i * kn.n_res + t];
                    if v.is_nan() {
                        mx = f64::NAN;
                        break;
                    }
                    if v.abs() > mx {
                        mx = v.abs();
                    }
                }
                out.push(mx);
            }
        }
        out
    }

    /// Numerical rank of the Jacobian at z — the workhorse of Stage 2/4 diagnosis.
    pub fn rank(&mut self, z: &[f64], rcond: f64, hard_only: bool) -> usize {
        if self.n_free == 0 || self.n_res == 0 {
            return 0;
        }
        let j = self.jacobian_dense(z);
        let m = if hard_only {
            let keep: Vec<usize> = (0..self.n_res).filter(|&i| self.hard[i]).collect();
            j.select_rows(&keep)
        } else {
            j
        };
        crate::linalg::rank_rrqr(&m, rcond)
    }

    /// Structural Jacobian as a bipartite graph: `adj[row]` = sorted free columns with a
    /// structural nonzero, plus row → owning constraint id.  The public surface for diagnosis and
    /// decomposition, derived from the compiled blocks so it stays in step with what the solver
    /// actually evaluates.  Soft rows (drag targets) are never part of it.
    pub fn structure(&self) -> (Vec<Vec<usize>>, Vec<u32>) {
        let mut adj = Vec::new();
        let mut row_c = Vec::new();
        for b in &self.blocks {
            let kn = k(b.kid);
            for i in 0..b.count {
                let cid = b.cids[i];
                // a soft constraint has no hard rows; `hard` is per row, so consult row0
                if !self.hard[b.row0 + i * kn.n_res] {
                    continue;
                }
                let mut cols: Vec<usize> = Vec::with_capacity(kn.n_par);
                for t in 0..kn.n_par {
                    let col = self.col_of[b.gidx[i * kn.n_par + t] as usize];
                    if col >= 0 {
                        cols.push(col as usize);
                    }
                }
                cols.sort_unstable();
                cols.dedup();
                for _ in 0..kn.n_res {
                    adj.push(cols.clone());
                    row_c.push(cid);
                }
            }
        }
        (adj, row_c)
    }

    /// Rows of the full residual vector that are hard, in order.
    pub fn hard_rows(&self) -> Vec<usize> {
        (0..self.n_res).filter(|&i| self.hard[i]).collect()
    }

    // -- linear algebra plumbing for the solvers -----------------------------

    pub(crate) fn ata_mut(&mut self) -> &mut Ata {
        if self.ata.is_none() {
            self.ata = Some(Ata::new(self.n_res, self.n_free, &self.csr_indptr, &self.csr_indices));
        }
        self.ata.as_mut().unwrap()
    }

    pub(crate) fn csr_values(&self) -> &[f64] {
        &self.csr_data
    }

    /// out (n) <- Jᵀ v (m), from the CSR values last computed.
    pub(crate) fn jt_mul_sparse(&self, v: &[f64], out: &mut [f64]) {
        for x in out.iter_mut() {
            *x = 0.0;
        }
        for i in 0..self.n_res {
            let vi = v[i];
            if vi == 0.0 {
                continue;
            }
            for p in self.csr_indptr[i]..self.csr_indptr[i + 1] {
                out[self.csr_indices[p as usize] as usize] += self.csr_data[p as usize] * vi;
            }
        }
    }

    /// out (m) <- J v (n), from the CSR values last computed.
    pub(crate) fn j_mul_sparse(&self, v: &[f64], out: &mut [f64]) {
        for i in 0..self.n_res {
            let mut s = 0.0;
            for p in self.csr_indptr[i]..self.csr_indptr[i + 1] {
                s += self.csr_data[p as usize] * v[self.csr_indices[p as usize] as usize];
            }
            out[i] = s;
        }
    }
}
