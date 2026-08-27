//! Compile a sketch to a flat evaluation plan; evaluate r(z), J(z); solve.
//!
//! `System` groups the sketch's constraints by kernel type into *blocks* — pure arrays of (kernel
//! id, global parameter indices, constants) — and owns the residual/Jacobian loop, the sparsity
//! structure and the solve iteration.  The Jacobian's structure (CSR indices, duplicate-summing
//! scatter map) is computed once at compile time; each evaluation only refills `data`.
//!
//! This compile-once / evaluate-many seam is the architectural boundary the program's Stage 1
//! calls for: the object model stays out of the hot loop.

use crate::constraints::Constraint;
use crate::kernels::{self, Kernel};
use crate::linalg::{rank_and_nullspace_with, rrqr_with, Mat, RankNull, Tol};
use crate::model::{EntRef, Sketch};
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

/// The tolerance a rank is judged at: a singular value of a `Conditioned` Jacobian below this
/// is zero.  Dimensionless and absolute — "a motion the size of the drawing changes this
/// residual by less than `RANK_TOL` of its own units" — so it is the same statement in every
/// sketch at every size.  The one number the diagnosis, the witness and `System::rank` share.
pub const RANK_TOL: f64 = 1e-9;

/// Rows of the Jacobian at z with the units divided out: row r over
/// `max(1, extent)^(degree - 1)`, columns already in world length (`z = x * col_scale`).  Every
/// entry is dimensionless and O(1) for a well-posed row, so a singular value is an absolute
/// statement and one tolerance judges every sketch at every size.
///
/// This is the only matrix a rank or a null space is ever asked of, and it is why: a raw row
/// is in its residual's units, a squared distance's gradient is `2d` next to a unit normal's
/// `1`, and a threshold relative to the largest singular value then belongs to whichever row
/// is largest — which may be a dimension in another figure entirely.  A relative tolerance is
/// not on offer here; the methods take an absolute one.  Only `System` builds one.
pub struct Conditioned {
    m: Mat,
}

impl Conditioned {
    pub fn rows(&self) -> usize {
        self.m.rows
    }

    pub fn cols(&self) -> usize {
        self.m.cols
    }

    /// The given rows, in the given order.
    pub fn select_rows(&self, rows: &[usize]) -> Conditioned {
        Conditioned { m: self.m.select_rows(rows) }
    }

    /// Rank and right null space (the motions) from one SVD.
    pub fn rank_and_nullspace(&self, tol: f64) -> RankNull {
        rank_and_nullspace_with(&self.m, Tol::Abs(tol))
    }

    /// Rank and left null space (the dependencies among rows) from one SVD.
    pub fn left_nullspace(&self, tol: f64) -> RankNull {
        rank_and_nullspace_with(&self.m.transpose(), Tol::Abs(tol))
    }

    /// Rank by pivoted QR of the transpose: `(rank, pivots)`, the first `rank` pivots indexing
    /// a maximal independent set of rows.
    pub fn independent_rows(&self, tol: f64) -> (usize, Vec<i32>) {
        rrqr_with(&self.m.transpose(), Tol::Abs(tol))
    }

    pub fn rank_rrqr(&self, tol: f64) -> usize {
        rrqr_with(&self.m, Tol::Abs(tol)).0
    }

    /// The numbers, for handing across the ABI and for the witness's dependency coefficients.
    /// Not for a rank: that is the methods above, with the tolerance they insist on.
    #[doc(hidden)]
    pub fn as_mat(&self) -> &Mat {
        &self.m
    }
}

pub struct System {
    pub n_params: usize,
    pub free: Vec<i32>,
    pub n_free: usize,
    pub col_of: Vec<i32>,
    pub n_res: usize,
    /// World length one unit of each free column is worth — `Param::scale`, gathered.  The
    /// solver's variables are `z = x * col_scale`, so a step of a given size means the same
    /// amount of motion whichever column it is in.  Without it a dimensionless unknown (a curve
    /// parameter, whose one unit is a whole span of curve) and a coordinate share one trust
    /// region and one minimum-norm objective, and the conditioning that follows is bad enough
    /// to stall a tangency that solves perfectly at a tenth the size.
    pub col_scale: Vec<f64>,
    /// False when every scale is 1 — the ordinary sketch, which then pays nothing for any of it.
    scaled: bool,
    pub extent: f64,
    /// Residual units for squared distances: `max(1, extent)²`.
    pub scale: f64,
    /// Residual units per row: `max(1, extent)^degree` for the row's kernel.  Kernels are not
    /// all written to the same power of length, so one system-wide scale judges half of them
    /// against a tolerance meant for the other half.
    pub row_scale: Vec<f64>,
    /// Units of a row of the Jacobian: `max(1, extent)^(degree - 1)`, since the derivative of a
    /// degree-`d` residual with respect to a world length carries one power of length fewer.
    /// What `conditioned` divides each row by.
    jac_scale: Vec<f64>,
    /// The smallest `row_scale` over hard rows — the strictest tolerance in the system, which is
    /// what the inner solver has to iterate to for every row to come in under its own.
    pub min_hard_scale: f64,
    /// One flag per residual row: rows that must be satisfied.
    pub hard: Vec<bool>,
    pub blocks: Vec<Block>,
    /// Constraint ids in block order (the order `constraint_errors` reports in).
    pub cids: Vec<u32>,
    /// The span of a spline each curve contact was compiled on — which control points its
    /// columns name.  Its constants are that span's knots, so a refresh reads them from here
    /// and not from a parameter that may since have moved to another span.  Empty for a sketch
    /// with no curves in it, which is the check every curve path is behind.
    spans: BTreeMap<u32, usize>,
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
    /// The static kernels plus one per curve definition — see `kernel_table`.
    kernels: Vec<Kernel>,
}

/// Every kernel this system may evaluate: the static table, then one per curve definition the
/// document holds.
///
/// A curve family's kernel is not in `KERNELS` because there is no fixed number of them, and its
/// width is the family's rather than the type's.  Building the table here — once, at compile
/// time, like everything else about a block — is what lets two different curves have different
/// column counts while each block keeps a fixed one.
fn kernel_table(sk: &Sketch) -> Vec<Kernel> {
    let mut t: Vec<Kernel> = kernels::KERNELS.to_vec();
    for d in &sk.curve_defs {
        let n_theta = d.vars.len().saturating_sub(1 + d.values.len());
        t.push(match &d.body {
            crate::model::CurveBody::Exprs { x, y } => {
                kernels::curve_kernel(n_theta, 3 + x.flat.len() + y.flat.len() + d.values.len())
            }
            crate::model::CurveBody::Trace(l) => {
                kernels::trace_kernel(n_theta, 2 + d.values.len() + l.flat.len())
            }
        });
    }
    t
}

impl System {
    pub fn new(sk: &Sketch) -> System {
        // A remembered pose is addressed by where its contact's constants live, and this is the
        // one moment those move: the blocks about to be built may take the memory a dropped
        // system's did, and a pose read back through a reused address would be another curve's.
        // Forgetting here is exact — nothing earlier is worth carrying past a recompile anyway.
        crate::locus::forget();
        let table = kernel_table(sk);
        let n = sk.params.len();
        let free = sk.free_indices();
        let n_free = free.len();
        let mut col_of = vec![-1i32; n];
        for (i, &p) in free.iter().enumerate() {
            col_of[p as usize] = i as i32;
        }
        // A contact parameter's scale is read off the thing it runs along here rather than off
        // the Param, so it is a fact about this compile and cannot be stale — and once per
        // entity, not once per contact, since the arc-length walk is the expensive part.  Either
        // family: an ellipse resized since its contact was added would otherwise keep the scale
        // the seed recorded, which is the stall the scaling exists to prevent.
        let mut speed: BTreeMap<EntRef, f64> = BTreeMap::new();
        let mut scale_of: BTreeMap<u32, f64> = BTreeMap::new();
        for c in &sk.constraints {
            if let Some((e, t)) = c.parametric_contact() {
                let v = *speed
                    .entry(e)
                    .or_insert_with(|| crate::constraints::contact_speed(sk, e));
                scale_of.insert(t, v);
            }
        }
        let col_scale: Vec<f64> = free
            .iter()
            .map(|&p| {
                let s = scale_of.get(&(p as u32)).copied().unwrap_or(sk.params[p as usize].scale);
                if s.is_finite() && s > 0.0 {
                    s
                } else {
                    1.0
                }
            })
            .collect();
        let scaled = col_scale.iter().any(|&s| s != 1.0);
        let extent = sk.extent();
        let scale = extent.max(1.0).powi(2);

        // group by kernel id, then sketch order — deterministic.  A claim is no equation and no
        // system carries one, which is the whole of what keeps a claim from moving the geometry:
        // the diagnosis judges it by stacking its rows onto a compiled system (`conditioned_with`)
        // rather than by compiling a system that has them.
        let mut by_kernel: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
        for (i, c) in sk.constraints.iter().enumerate() {
            if c.claim {
                continue;
            }
            by_kernel.entry(c.kernel_id_in(sk)).or_default().push(i);
        }

        let mut blocks: Vec<Block> = Vec::new();
        let mut slot_of = BTreeMap::new();
        let mut cids: Vec<u32> = Vec::new();
        let spans = crate::curve::contact_spans(sk);
        let mut hard: Vec<bool> = Vec::new();
        let mut row0 = 0usize;
        let mut joff = 0usize;
        for (&kid, idxs) in by_kernel.iter() {
            let kn = table[kid];
            let nb = idxs.len();
            let mut gidx = Vec::with_capacity(nb * kn.n_par);
            let mut consts = Vec::with_capacity(nb * kn.n_const);
            let mut bcids = Vec::with_capacity(nb);
            for (i, &ci) in idxs.iter().enumerate() {
                let c = &sk.constraints[ci];
                // one span for the block: the columns it names and the knots it carries
                let span = spans.get(&c.id).copied();
                let ps = c.params_on(sk, span);
                debug_assert_eq!(ps.len(), kn.n_par, "{:?} params", c.kind);
                for p in ps {
                    gidx.push(p as i32);
                }
                if kn.n_const > 0 {
                    consts.extend(c.consts_on(sk, span));
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
            let kn = table[b.kid];
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
            let kn = table[b.kid];
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
        let mut jac_scale = vec![1.0; n_res];
        for b in &blocks {
            let kn = table[b.kid];
            let sc = extent.max(1.0).powi(kn.degree as i32);
            let jsc = extent.max(1.0).powi(kn.degree as i32 - 1);
            for r in b.row0..b.row0 + b.count * kn.n_res {
                row_scale[r] = sc;
                jac_scale[r] = jsc;
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
            col_scale,
            scaled,
            extent,
            scale,
            row_scale,
            jac_scale,
            min_hard_scale,
            hard,
            blocks,
            cids,
            spans,
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
            kernels: table,
        }
    }

    /// The span of a spline each curve contact was compiled on — which control points its
    /// columns name.  Empty for a sketch with no curves in it, which is the check every curve
    /// path is behind.
    pub fn spans(&self) -> &BTreeMap<u32, usize> {
        &self.spans
    }

    // -- constants -----------------------------------------------------------

    /// Push a constraint's (mutated) constants into the compiled plan — a moving drag target or
    /// an edited dimension.  Topology is unchanged, so no recompile.
    pub fn update_consts(&mut self, sk: &Sketch, cid: u32) {
        let Some(&(b, i)) = self.slot_of.get(&cid) else { return };
        let kn = self.kernels[self.blocks[b].kid];
        if kn.n_const == 0 {
            return;
        }
        if self.spans.contains_key(&cid) {
            return; // a curve contact's constants are invariant for this system — see below
        }
        let Some(c) = sk.constraint(cid) else { return };
        let vals = c.consts_on(sk, None);
        self.blocks[b].consts[i * kn.n_const..(i + 1) * kn.n_const].copy_from_slice(&vals);
    }

    /// Re-read every constraint's constants (after arbitrary dimension edits).  Curve contacts
    /// are skipped: see below.
    ///
    /// One pass over the sketch's constraints, not a `Sketch::constraint` lookup per slot: that
    /// is a linear scan, so looking each one up would make refreshing quadratic in the
    /// constraint count — and this runs on every plan solve and at every drag start.
    pub fn refresh_consts(&mut self, sk: &Sketch) {
        let by_id: BTreeMap<u32, &Constraint> = sk.constraints.iter().map(|c| (c.id, c)).collect();
        let spans = &self.spans;
        for b in self.blocks.iter_mut() {
            let kn = self.kernels[b.kid];
            if kn.n_const == 0 {
                continue;
            }
            for (i, &cid) in b.cids.iter().enumerate() {
                if let Some(c) = by_id.get(&cid) {
                    // a curve contact's constants are its compiled span's knots — document data
                    // no solve moves, and the span is pinned for this system's life, so there is
                    // nothing here that could have changed
                    if spans.contains_key(&cid) {
                        continue;
                    }
                    let v = c.consts_on(sk, None);
                    b.consts[i * kn.n_const..(i + 1) * kn.n_const].copy_from_slice(&v);
                }
            }
        }
    }

    /// First residual row of a constraint — `None` for one this plan was not compiled from.
    pub fn row_of(&self, cid: u32) -> Option<usize> {
        let &(b, i) = self.slot_of.get(&cid)?;
        Some(self.blocks[b].row0 + i * self.kernels[self.blocks[b].kid].n_res)
    }

    // -- evaluation ----------------------------------------------------------

    /// Free values of the current sketch geometry, in the solver's scaled units (also refreshes
    /// our copy of x).
    pub fn z0(&mut self, sk: &Sketch) -> Vec<f64> {
        self.x = sk.get_x();
        if !self.scaled {
            return self.free.iter().map(|&i| self.x[i as usize]).collect();
        }
        self.free
            .iter()
            .enumerate()
            .map(|(i, &p)| self.x[p as usize] * self.col_scale[i])
            .collect()
    }

    pub fn full_x(&self, z: &[f64]) -> Vec<f64> {
        let mut x = self.x.clone();
        for (i, &p) in self.free.iter().enumerate() {
            x[p as usize] = if self.scaled { z[i] / self.col_scale[i] } else { z[i] };
        }
        x
    }

    fn apply_z(&mut self, z: &[f64]) {
        for (i, &p) in self.free.iter().enumerate() {
            self.x[p as usize] = if self.scaled { z[i] / self.col_scale[i] } else { z[i] };
        }
    }

    pub fn residuals_into(&mut self, z: &[f64], r: &mut [f64]) {
        self.apply_z(z);
        let mut v: Vec<f64> = Vec::new();
        for b in &self.blocks {
            let kn = self.kernels[b.kid];
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
            let kn = self.kernels[b.kid];
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
        // dr/dz = (dr/dx) / col_scale: the same chain rule that turned x into z above
        if self.scaled {
            for r in 0..self.n_res {
                for p in self.csr_indptr[r]..self.csr_indptr[r + 1] {
                    let p = p as usize;
                    self.csr_data[p] /= self.col_scale[self.csr_indices[p] as usize];
                }
            }
        }
        &self.csr_data
    }

    /// The raw `dr/dz` — what the solvers step on and what a finite-difference check has to see.
    /// Its rows are in the residuals' own units and not comparable with each other: a rank or
    /// a null space is asked of `conditioned`, never of this.
    pub fn jacobian_dense(&mut self, z: &[f64]) -> Mat {
        let rows: Vec<usize> = (0..self.n_res).collect();
        self.scatter(z, &rows, false)
    }

    /// The chosen rows of the CSR Jacobian, filled into a dense matrix — optionally with each
    /// row divided by its units (`jac_scale`), which is the whole of what `Conditioned` is.
    fn scatter(&mut self, z: &[f64], rows: &[usize], condition: bool) -> Mat {
        let mut m = Mat::zeros(rows.len(), self.n_free);
        if self.n_free == 0 || rows.is_empty() {
            return m;
        }
        self.compute_csr(z);
        for (i, &r) in rows.iter().enumerate() {
            let inv = if condition { 1.0 / self.jac_scale[r] } else { 1.0 };
            for p in self.csr_indptr[r]..self.csr_indptr[r + 1] {
                m.data[i * self.n_free + self.csr_indices[p as usize] as usize] =
                    self.csr_data[p as usize] * inv;
            }
        }
        m
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
    /// `locus::assemble` states the same rule in miniature for a trace block's inner rows; an
    /// edit to what counts toward a row's units has a twin there.
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
            Some(&(b, _)) => self.extent.max(1.0).powi(self.kernels[self.blocks[b].kid].degree as i32),
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
            let kn = self.kernels[b.kid];
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

    /// Numerical rank of the Jacobian at z — the workhorse of Stage 2/4 diagnosis.  `tol` is
    /// absolute and dimensionless (`RANK_TOL` is the one the diagnosis uses).
    pub fn rank(&mut self, z: &[f64], tol: f64, hard_only: bool) -> usize {
        if self.n_free == 0 || self.n_res == 0 {
            return 0;
        }
        let rows: Vec<usize> = if hard_only { self.hard_rows() } else { (0..self.n_res).collect() };
        self.condition(z, &rows).rank_rrqr(tol)
    }

    /// The hard rows of the Jacobian at z with their units divided out — see `Conditioned`.
    /// Rows are in `structure()`'s order, so its `row_c[i]` names row `i` here.
    pub fn conditioned(&mut self, z: &[f64]) -> Conditioned {
        let rows = self.hard_rows();
        self.condition(z, &rows)
    }

    /// `conditioned`, with the rows of constraints this system was *not* compiled from stacked
    /// underneath, and the owning constraint id per row.  Its one caller is the diagnosis judging
    /// a `claim` (§9.7): a claim has no rows precisely because it is never solved for, so asking
    /// whether it adds rank means asking about this matrix plus its rows.
    ///
    /// It is asked *here* rather than by compiling a second `System` over the claims, for two
    /// reasons the compile would get wrong.  The row's units and the column mapping are written
    /// down once, in `scatter` and `col_of`, and a caller assembling rows itself would be a
    /// second copy of both.  And a compile calls `locus::forget`, which is what makes an
    /// address-keyed pose sound — so a second system built beside a live one throws that one's
    /// remembered trace poses away and every contact re-walks its march from the home.  On
    /// `peaucellier`, a traced document that ends on a claim, that cost 834 µs a diagnosis
    /// against 45 µs for the whole of the rest of it.
    ///
    /// `extra` may own no `Param` and bind no free variable — which is exactly what a claim may
    /// not do either (`CKind::claimable`, `expr::write_value`), so its columns are its entities'
    /// and `kind.kernel()` is safe to ask.
    pub(crate) fn conditioned_with(
        &mut self,
        sk: &Sketch,
        z: &[f64],
        extra: &[&Constraint],
    ) -> (Conditioned, Vec<u32>) {
        let base = self.conditioned(z);
        let (_, mut row_c) = self.structure();
        if extra.is_empty() || self.n_free == 0 {
            return (base, row_c);
        }
        let n_extra: usize = extra.iter().map(|c| c.n_residuals()).sum();
        let mut m = Mat::zeros(base.rows() + n_extra, self.n_free);
        m.data[..base.as_mat().data.len()].copy_from_slice(&base.as_mat().data);
        let mut r = base.rows();
        for c in extra {
            let ps = c.params(sk);
            let v = c.local_values(sk);
            let j = c.jacobian(sk, &v);
            let kn = crate::kernels::kernel(c.kind.kernel());
            let inv = 1.0 / self.extent.max(1.0).powi(kn.degree as i32 - 1);
            for t in 0..kn.n_res {
                for (k, &p) in ps.iter().enumerate() {
                    let col = self.col_of[p as usize];
                    if col >= 0 {
                        m.data[r * self.n_free + col as usize] += j[t * kn.n_par + k] * inv;
                    }
                }
                row_c.push(c.id);
                r += 1;
            }
        }
        (Conditioned { m }, row_c)
    }

    fn condition(&mut self, z: &[f64], rows: &[usize]) -> Conditioned {
        Conditioned { m: self.scatter(z, rows, true) }
    }

    /// Structural Jacobian as a bipartite graph: `adj[row]` = sorted free columns with a
    /// structural nonzero, plus row → owning constraint id.  The public surface for diagnosis and
    /// decomposition, derived from the compiled blocks so it stays in step with what the solver
    /// actually evaluates.  Soft rows (drag targets) are never part of it.
    pub fn structure(&self) -> (Vec<Vec<usize>>, Vec<u32>) {
        let mut adj = Vec::new();
        let mut row_c = Vec::new();
        for b in &self.blocks {
            let kn = self.kernels[b.kid];
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
