//! Compile a sketch to a flat evaluation plan; evaluate r(z), J(z); solve.
//!
//! `System` groups the sketch's constraints by kernel type into *blocks* — pure arrays of (kernel
//! id, global parameter indices, constants) — and owns the residual/Jacobian loop, the sparsity
//! structure and the solve iteration.  The Jacobian's structure (CSR indices, duplicate-summing
//! scatter map) is computed once at compile time; each evaluation only refills `data`.
//!
//! This compile-once / evaluate-many seam is the architectural boundary the program's Stage 1
//! calls for: the object model stays out of the hot loop.

#[allow(unused_imports)]
use crate::fmath::Det;
use crate::constraints::Constraint;
use crate::kernels::{self, Kernel, KernelKey};
use crate::linalg::{rank_and_nullspace_with, rrqr_with, Mat, RankNull, Tol};
use crate::model::{EntRef, Sketch};
use crate::sparse::Ata;
use std::collections::BTreeMap;
use std::sync::Arc;

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

/// `‖Jᵀr‖∞ / (‖J‖∞ ‖r‖∞)` below which a pose counts as stationary (`System::stationary`).  At
/// a minimum the solvers actually reach, the ratio is rounding — 1e-12 and below; a solve
/// that stopped short of one is off by orders of magnitude, so the line between them is wide.
pub const STATIONARY_TOL: f64 = 1e-6;

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
    /// Each parameter's column: `>= 0` a free one, `-1` none (held), and `-2 - k` the `k`th
    /// derived parameter, whose Jacobian is folded into its bases' columns (`dcols`).
    pub col_of: Vec<i32>,
    /// A ring's turned copies' parameters (`Sketch::derived`): worked out from their bases
    /// wherever the free vector is written (`apply_z`, `full_x`).
    derived: Vec<crate::model::Derivation>,
    /// Per derived parameter, its free bases' columns with their weights: where one kernel output
    /// lands by the chain rule.
    dcols: Vec<Vec<(i64, f64)>>,
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
    ///
    /// **Every residual this system hands out is already divided by it**, and so is every row
    /// of the Jacobian — `residuals_into` and `compute_csr` are the two places a row is
    /// produced (`subset_residuals` and `subset_csr` for some of the rows), and each scales it
    /// there.  So the vector the solvers minimise is
    /// dimensionless: a degree-0 `angle` row (a bearing gap in radians, O(1)) sits beside a
    /// degree-2 `distance` row (a squared length, O(L²)) at equal weight.  Left raw, the
    /// dogleg's merit function, its ratio test and its Cauchy step all belonged to the length
    /// rows — a four-bar linkage with one angle stated turned its crank about a degree an
    /// iteration and ran out at a hundred, and the same figure at ten times the size did not
    /// move at all (issue #43).  The relative rule "solved" already used for the *test* is
    /// now the rule the *iteration* sees, which is the only way the two can agree.
    pub row_scale: Vec<f64>,
    /// False when every `row_scale` is 1 — a drawing within a unit of the origin — which then
    /// pays nothing for the division.
    row_scaled: bool,
    /// Units of a row of the Jacobian: `max(1, extent)^(degree - 1)`, since the derivative of a
    /// degree-`d` residual with respect to a world length carries one power of length fewer.
    /// What `conditioned` divides a *raw* row by; over a row already over `row_scale`, that is
    /// a multiplication by `row_scale / jac_scale`, which is `max(1, extent)` for every row.
    jac_scale: Vec<f64>,
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
    ent_w: Vec<f64>,
    csr_data: Vec<f64>,
    slot_of: BTreeMap<u32, (usize, usize)>,
    ata: Option<Ata>,
    /// The kernel of each block, by `Block::kid` — see `build_kernel`.
    kernels: Vec<Kernel>,
    /// The block-triangular order of the hard rows, worked out the first time it is asked for
    /// (`block_order`) and kept for the life of the compile, whose topology it is a fact about.
    order: Option<Arc<BlockOrder>>,
}

/// One strongly connected block of a system's equations: rows that read one another round a
/// cycle and so are solved together, with the free columns matched to them.  Every other column
/// a row reads belongs to an earlier block (or to the over-determined part) and is held.
#[derive(Clone, Debug)]
pub struct SolveBlock {
    /// Residual rows of the full vector, ascending — every row of each instance below.
    pub rows: Vec<usize>,
    /// The free columns the rows are matched to, ascending: as many as there are rows.
    pub cols: Vec<usize>,
    /// The constraint instances producing the rows: (kernel block, instance in it), ascending.
    pub instances: Vec<(usize, usize)>,
}

/// The block-triangular order of a system's hard rows (`System::block_order`): the matched
/// square part of the Dulmage–Mendelsohn decomposition as strongly connected blocks in solve
/// order (`graph::blocks`), and the over- and under-determined parts beside them, which are in
/// no block — redundant rows have no columns of their own and free columns no rows, so neither
/// is solved block by block.  Rows are of the full residual vector; columns are free columns.
#[derive(Clone, Debug, Default)]
pub struct BlockOrder {
    pub blocks: Vec<SolveBlock>,
    /// How deep each block sits: 1 for one that reads no other (`graph::Blt::level`).
    pub level: Vec<usize>,
    pub over_rows: Vec<usize>,
    pub over_cols: Vec<usize>,
    pub under_rows: Vec<usize>,
    pub under_cols: Vec<usize>,
}

impl BlockOrder {
    /// The longest chain of blocks, each reading the one before it.
    pub fn depth(&self) -> usize {
        self.level.iter().copied().max().unwrap_or(0)
    }
}

/// The conditioned Jacobian as sparse rows (`System::conditioned_sparse`).
pub struct SparseConditioned {
    pub n_cols: usize,
    indptr: Vec<i32>,
    indices: Vec<i32>,
    data: Vec<f64>,
}

impl SparseConditioned {
    /// How many rows: the full residual vector's.
    pub fn n_rows(&self) -> usize {
        self.indptr.len() - 1
    }

    /// Row `r`'s entries, `(column, value)`.
    pub fn row(&self, r: usize) -> impl Iterator<Item = (usize, f64)> + '_ {
        let (a, b) = (self.indptr[r] as usize, self.indptr[r + 1] as usize);
        self.indices[a..b].iter().zip(&self.data[a..b]).map(|(&c, &v)| (c as usize, v))
    }

    /// Row `r` against a whole column vector.
    pub fn row_dot(&self, r: usize, x: &[f64]) -> f64 {
        self.row(r).map(|(c, v)| v * x[c]).sum()
    }

    /// The given rows against the given columns, dense, in the given orders.
    pub fn dense(&self, rows: &[usize], cols: &[usize]) -> Mat {
        let mut local = BTreeMap::new();
        for (k, &c) in cols.iter().enumerate() {
            local.insert(c, k);
        }
        let mut m = Mat::zeros(rows.len(), cols.len());
        for (i, &r) in rows.iter().enumerate() {
            for (c, v) in self.row(r) {
                if let Some(&k) = local.get(&c) {
                    m.data[i * cols.len() + k] = v;
                }
            }
        }
        m
    }
}

/// Some of a system's constraint instances, evaluated on their own over some of its columns
/// (`System::subset`): their residual rows, and a CSR Jacobian of those rows against the chosen
/// columns only, every other column read from `z` and held.  Each number is the one the whole
/// system's evaluation makes for that row and column, to the bit — the same kernels on the same
/// slices, the same summation order for a column a constraint names twice, and the same
/// `row_scale` and `col_scale` — so a block solved through it is solved in the whole system's
/// units, and a traced contact reads its constants at the address its memory is keyed by.
pub struct Subset {
    /// (kernel block, first instance, count, first local row, first local kernel Jacobian output).
    runs: Vec<(usize, usize, usize, usize, usize)>,
    /// Residual rows of the full vector, in the order they are evaluated (ascending).
    pub rows: Vec<usize>,
    /// The free columns differentiated against, in local order.
    pub cols: Vec<usize>,
    pub indptr: Vec<i32>,
    pub indices: Vec<i32>,
    /// The Jacobian's values over `indptr`/`indices`, as `System::subset_csr` last filled them.
    pub data: Vec<f64>,
    ent_src: Vec<i32>,
    ent_slot: Vec<i32>,
    ent_w: Vec<f64>,
    jdata: Vec<f64>,
    v: Vec<f64>,
}

impl Subset {
    /// out (rows) <- J v (columns), from the values `System::subset_csr` last filled.
    pub(crate) fn j_mul(&self, v: &[f64], out: &mut [f64]) {
        csr_mul(&self.indptr, &self.indices, &self.data, v, out);
    }

    /// out (columns) <- Jᵀ v (rows), likewise.
    pub(crate) fn jt_mul(&self, v: &[f64], out: &mut [f64]) {
        out.iter_mut().for_each(|x| *x = 0.0);
        for r in 0..self.rows.len() {
            for p in self.indptr[r]..self.indptr[r + 1] {
                out[self.indices[p as usize] as usize] += self.data[p as usize] * v[r];
            }
        }
    }
}

/* -- evaluation, shared by the whole system and a subset of it -------------------------------- */
// Each is one step both paths take, written once so a subset's numbers cannot drift from the
// whole system's: the same values gathered, the same entries in the same order, the same sums.

/// `v` <- the parameter values `gidx` names, in order: a kernel call's input.
fn gather(v: &mut Vec<f64>, x: &[f64], gidx: &[i32]) {
    v.clear();
    v.extend(gidx.iter().map(|&g| x[g as usize]));
}

/// Fill `count` instances' Jacobian outputs, from the start of `out`, with the kernel's constant
/// Jacobian where it has one — once, since nothing recomputes it.
fn fill_const_jac(kn: Kernel, count: usize, out: &mut [f64]) {
    if let Some(cj) = kn.const_jac {
        let sz = kn.n_res * kn.n_par;
        for i in 0..count {
            out[i * sz..(i + 1) * sz].copy_from_slice(cj);
        }
    }
}

/// Push the Jacobian entries of `count` instances of `kn` over the parameters `gidx`, their first
/// row `row0` and their first kernel output `src0`, as `(row * ncols + column, output, weight)`
/// in (instance, row, parameter) order; `col` pushes a parameter's columns with their weights —
/// none where it has none, one of weight 1 where it is free, a derived one's bases by the chain
/// rule.
fn jac_entries(
    es: &mut Vec<(i64, i32, f64)>,
    kn: Kernel,
    gidx: &[i32],
    count: usize,
    row0: usize,
    src0: usize,
    ncols: i64,
    col: impl Fn(i32, &mut Vec<(i64, f64)>),
) {
    let mut cs = Vec::new();
    for i in 0..count {
        for t in 0..kn.n_res {
            for c in 0..kn.n_par {
                cs.clear();
                col(gidx[i * kn.n_par + c], &mut cs);
                let row = (row0 + i * kn.n_res + t) as i64;
                let src = (src0 + (i * kn.n_res + t) * kn.n_par + c) as i32;
                for &(col, w) in &cs {
                    es.push((row * ncols + col, src, w));
                }
            }
        }
    }
}

/// A CSR Jacobian's structure, and per entry the kernel output it reads and the value it adds to.
struct Csr {
    indptr: Vec<i32>,
    indices: Vec<i32>,
    ent_src: Vec<i32>,
    ent_slot: Vec<i32>,
    ent_w: Vec<f64>,
}

/// The structure of `n_rows` rows from `jac_entries`' list: sorted stably by position, so a
/// column a row names twice is one value, summed in the order its entries were listed.
fn csr_structure(mut es: Vec<(i64, i32, f64)>, n_rows: usize, ncols: i64) -> Csr {
    es.sort_by_key(|e| e.0);
    let mut indptr = vec![0i32; n_rows + 1];
    let mut indices: Vec<i32> = Vec::with_capacity(es.len());
    let (mut ent_src, mut ent_slot) = (Vec::with_capacity(es.len()), Vec::with_capacity(es.len()));
    let mut ent_w = Vec::with_capacity(es.len());
    for e in 0..es.len() {
        if e == 0 || es[e].0 != es[e - 1].0 {
            indices.push((es[e].0 % ncols) as i32);
            indptr[(es[e].0 / ncols) as usize + 1] = indices.len() as i32;
        }
        ent_src.push(es[e].1);
        ent_slot.push(indices.len() as i32 - 1);
        ent_w.push(es[e].2);
    }
    for i in 1..indptr.len() {
        if indptr[i] < indptr[i - 1] {
            indptr[i] = indptr[i - 1];
        }
    }
    Csr { indptr, indices, ent_src, ent_slot, ent_w }
}

/// out <- J v, J the CSR over `indptr`, `indices` and `data`.
pub(crate) fn csr_mul(indptr: &[i32], indices: &[i32], data: &[f64], v: &[f64], out: &mut [f64]) {
    for i in 0..indptr.len() - 1 {
        let mut s = 0.0;
        for p in indptr[i]..indptr[i + 1] {
            s += data[p as usize] * v[indices[p as usize] as usize];
        }
        out[i] = s;
    }
}

/// `data` <- every entry's kernel output, times its weight, summed into its value.  A weight is
/// 1 but where a derived parameter's output is folded into a base, and multiplying by 1 is exact,
/// so a sketch with no ring keeps every bit.
fn assemble(data: &mut [f64], jdata: &[f64], ent_src: &[i32], ent_slot: &[i32], ent_w: &[f64]) {
    for v in data.iter_mut() {
        *v = 0.0;
    }
    for e in 0..ent_src.len() {
        data[ent_slot[e] as usize] += ent_w[e] * jdata[ent_src[e] as usize];
    }
}

/// Push parameter `p`'s columns with their weights: its own where it is free, a derived one's
/// bases', none where it is held.
fn fold(col_of: &[i32], dcols: &[Vec<(i64, f64)>], p: i32, cs: &mut Vec<(i64, f64)>) {
    match col_of[p as usize] {
        c if c >= 0 => cs.push((c as i64, 1.0)),
        -1 => {}
        c => cs.extend_from_slice(&dcols[(-2 - c) as usize]),
    }
}

/// `x` <- every derived parameter worked out from its bases.
fn settle(x: &mut [f64], derived: &[crate::model::Derivation]) {
    for d in derived {
        x[d.param as usize] = d.value(x);
    }
}

/// Each CSR value over its row's units and its column's scale — `row_scale` and `col_scale` of
/// a row and a column of the CSR's own.
fn to_units(
    data: &mut [f64],
    indptr: &[i32],
    indices: &[i32],
    row_scale: impl Fn(usize) -> f64,
    col_scale: impl Fn(usize) -> f64,
) {
    for r in 0..indptr.len() - 1 {
        let inv = 1.0 / row_scale(r);
        for p in indptr[r]..indptr[r + 1] {
            let p = p as usize;
            data[p] *= inv / col_scale(indices[p] as usize);
        }
    }
}

/// The kernel a key names (`Constraint::kernel_key`): a static one, or one built from the
/// sketch.
///
/// A curve family's kernel is not in `KERNELS` because there is no fixed number of them, and its
/// width is the family's rather than the type's — as a spline length's is its control-point
/// count's.  Building them here — once per key, at compile time, like everything else about a
/// block — is what lets two different curves have different column counts while each block
/// keeps a fixed one.
pub fn build_kernel(sk: &Sketch, key: KernelKey) -> Kernel {
    use crate::constraints::FamilyKernel;
    match key {
        KernelKey::Static(id) => kernels::KERNELS[id],
        KernelKey::Family { def, fk } => {
            let d = &sk.curve_defs[def];
            let n_theta = d.vars.len().saturating_sub(1 + d.values.len());
            let (n_const, body, formed) = match &d.body {
                crate::model::CurveBody::Exprs { x, y } => {
                    (3 + x.flat.len() + y.flat.len() + d.values.len(), kernels::FORMULA, true)
                }
                crate::model::CurveBody::Trace(l) => {
                    (3 + d.values.len() + l.flat.len() + l.n_q(), kernels::TRACE, l.without_form().is_none())
                }
                crate::model::CurveBody::Envelope(g) => {
                    (2 + d.values.len() + g.flat.len(), kernels::ENVELOPE, true)
                }
            };
            match (FamilyKernel::ALL[fk as usize], body) {
                (FamilyKernel::Contact, kernels::TRACE) => kernels::trace_kernel(n_theta, n_const),
                (FamilyKernel::Contact, kernels::ENVELOPE) => kernels::envelope_kernel(n_theta, n_const),
                (FamilyKernel::Contact, _) => kernels::curve_kernel(n_theta, n_const),
                (FamilyKernel::Tangent, _) => kernels::curve_tangent_kernel(n_theta, n_const, body),
                (FamilyKernel::Curvature, _) => {
                    kernels::curve_curvature_kernel(n_theta, n_const, body, formed)
                }
                (FamilyKernel::Extrusion, _) => kernels::extrusion_kernel(n_theta, n_const, body),
            }
        }
        KernelKey::Dual(inner) => kernels::dual_kernel(inner),
        KernelKey::SplineLength { n, free } => kernels::spline_length_kernel(n, free),
        KernelKey::Stationary(cid) => crate::variational::kernel(sk, cid),
    }
}

impl System {
    pub fn new(sk: &Sketch) -> System {
        // A remembered pose is addressed by where its contact's constants live, and this is the
        // one moment those move: the blocks about to be built may take the memory a dropped
        // system's did, and a pose read back through a reused address would be another curve's.
        // Forgetting here is exact — nothing earlier is worth carrying past a recompile anyway.
        crate::locus::forget();
        crate::generate::forget();
        let n = sk.params.len();
        let free = sk.free_indices();
        let n_free = free.len();
        let mut col_of = vec![-1i32; n];
        for (i, &p) in free.iter().enumerate() {
            col_of[p as usize] = i as i32;
        }
        let derived = sk.derived();
        let mut dcols = Vec::with_capacity(derived.len());
        for (k, d) in derived.iter().enumerate() {
            col_of[d.param as usize] = -2 - k as i32;
            dcols.push(
                d.terms.iter()
                    .filter(|&&(b, _)| col_of[b as usize] >= 0)
                    .map(|&(b, w)| (col_of[b as usize] as i64, w))
                    .collect::<Vec<_>>(),
            );
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
        let scale = extent.max(1.0).dpowi(2);

        // group by kernel id, then sketch order — deterministic.  A claim is no equation and no
        // system carries one, which is the whole of what keeps a claim from moving the geometry:
        // the diagnosis judges it by stacking its rows onto a compiled system (`conditioned_with`)
        // rather than by compiling a system that has them.
        let spans = crate::curve::contact_spans(sk);
        let mut by_kernel: BTreeMap<KernelKey, Vec<usize>> = BTreeMap::new();
        for (i, c) in sk.constraints.iter().enumerate() {
            // a constraint with no rows here compiles none: an energy's are carried once, by the
            // constraint leading its group (#121)
            if c.claim || c.rows_in(sk) == 0 {
                continue;
            }
            by_kernel.entry(c.kernel_key(sk)).or_default().push(i);
        }
        let table: Vec<Kernel> = by_kernel.keys().map(|&k| build_kernel(sk, k)).collect();

        let mut blocks: Vec<Block> = Vec::new();
        let mut slot_of = BTreeMap::new();
        let mut cids: Vec<u32> = Vec::new();
        let mut hard: Vec<bool> = Vec::new();
        let mut row0 = 0usize;
        let mut joff = 0usize;
        for (kid, idxs) in by_kernel.values().enumerate() {
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
                // an intrinsic row over held unknowns only — the unit row of an axis whose
                // direction a `fix` holds outright (`std.x`) — is a fact about the numbers held,
                // true by construction and no equation the drawing answers for.  Counted, it
                // would stand in the ledger as an equation over the rank.  Not hard, so no count
                // or rank sees it.  A statement over held numbers stays one: it may be wrong.
                let held = c.intrinsic && ps.iter().all(|&p| col_of[p as usize] < 0);
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
                    hard.push(!c.soft && !held);
                }
            }
            blocks.push(Block { kid, count: nb, row0, gidx, consts, cids: bcids, jac_off: joff });
            row0 += nb * kn.n_res;
            joff += nb * kn.n_res * kn.n_par;
        }
        let n_res = row0;

        let mut jdata = vec![0.0; joff.max(1)];
        for b in &blocks {
            fill_const_jac(table[b.kid], b.count, &mut jdata[b.jac_off..]);
        }

        // Jacobian structure: entry (block, i, res, par) -> (row, col), duplicates merged
        let ncols = n_free.max(1) as i64;
        let mut es: Vec<(i64, i32, f64)> = Vec::with_capacity(joff);
        for b in &blocks {
            let col = |p: i32, cs: &mut Vec<(i64, f64)>| fold(&col_of, &dcols, p, cs);
            jac_entries(&mut es, table[b.kid], &b.gidx, b.count, b.row0, b.jac_off, ncols, col);
        }
        let Csr { indptr: csr_indptr, indices: csr_indices, ent_src, ent_slot, ent_w } =
            csr_structure(es, n_res, ncols);
        let nnz = csr_indices.len();

        let mut row_scale = vec![1.0; n_res];
        let mut jac_scale = vec![1.0; n_res];
        for b in &blocks {
            let kn = table[b.kid];
            let sc = extent.max(1.0).dpowi(kn.degree as i32);
            let jsc = extent.max(1.0).dpowi(kn.degree as i32 - 1);
            for r in b.row0..b.row0 + b.count * kn.n_res {
                row_scale[r] = sc;
                jac_scale[r] = jsc;
            }
        }
        let row_scaled = row_scale.iter().any(|&s| s != 1.0);

        System {
            n_params: n,
            free,
            n_free,
            col_of,
            derived,
            dcols,
            n_res,
            col_scale,
            scaled,
            extent,
            scale,
            row_scale,
            row_scaled,
            jac_scale,
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
            ent_w,
            csr_data: vec![0.0; nnz.max(1)],
            slot_of,
            ata: None,
            kernels: table,
            order: None,
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
        settle(&mut x, &self.derived);
        x
    }

    fn apply_z(&mut self, z: &[f64]) {
        for (i, &p) in self.free.iter().enumerate() {
            self.x[p as usize] = if self.scaled { z[i] / self.col_scale[i] } else { z[i] };
        }
        settle(&mut self.x, &self.derived);
    }

    pub fn residuals_into(&mut self, z: &[f64], r: &mut [f64]) {
        self.apply_z(z);
        let mut v: Vec<f64> = Vec::new();
        for b in &self.blocks {
            let kn = self.kernels[b.kid];
            gather(&mut v, &self.x, &b.gidx[..b.count * kn.n_par]);
            let rows = b.count * kn.n_res;
            (kn.res)(b.count, &v, &b.consts, &mut r[b.row0..b.row0 + rows]);
        }
        // each row in its own units — see `row_scale`
        if self.row_scaled {
            for i in 0..self.n_res {
                r[i] /= self.row_scale[i];
            }
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
            gather(&mut v, &self.x, &b.gidx[..b.count * kn.n_par]);
            let sz = b.count * kn.n_res * kn.n_par;
            (kn.jac)(b.count, &v, &b.consts, &mut self.jdata[b.jac_off..b.jac_off + sz]);
        }
    }

    /// Refill the Jacobian's CSR values at z (the structure never changes).
    pub fn compute_csr(&mut self, z: &[f64]) -> &[f64] {
        self.jac_blocks(z);
        assemble(&mut self.csr_data, &self.jdata, &self.ent_src, &self.ent_slot, &self.ent_w);
        // dr/dz = (dr/dx) / col_scale: the same chain rule that turned x into z above — and the
        // row over its own units, as `residuals_into` hands the residual out
        if self.scaled || self.row_scaled {
            let (rs, cs) = (&self.row_scale, &self.col_scale);
            to_units(&mut self.csr_data, &self.csr_indptr, &self.csr_indices, |r| rs[r], |c| cs[c]);
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
            // the CSR row is already over `row_scale`; conditioning wants it over `jac_scale`
            let inv = if condition { self.row_scale[r] / self.jac_scale[r] } else { 1.0 };
            for p in self.csr_indptr[r]..self.csr_indptr[r + 1] {
                m.data[i * self.n_free + self.csr_indices[p as usize] as usize] =
                    self.csr_data[p as usize] * inv;
            }
        }
        m
    }

    /// max |r| over hard rows at z.  A residual is handed out in its row's own units, so this
    /// is `max_relative_residual` under its older name; both are kept because the ABI
    /// publishes both.
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

    /// Whether `z` is a stationary point of ½‖r‖² over the hard rows: `‖Jᵀr‖∞` within
    /// `STATIONARY_TOL` of `‖J‖∞ · ‖r‖∞`, which is the largest one entry of the gradient could
    /// be, so the ratio is dimensionless and the same statement at every size.
    ///
    /// A pose that is not a solution means one of two things, and only this question tells
    /// them apart.  At a stationary point the solver could go no further: the residual left
    /// is what the constraints *cannot* agree on, and the diagnosis may call it a conflict and
    /// look for the minimal set behind it.  Anywhere else the solver simply stopped — out of
    /// iterations, a collapsed trust region — and the unsatisfied rows are a fact about the
    /// solve, not about the geometry: a diagnosis that read them as a conflict named three
    /// innocent statements on a consistent four-bar linkage and invited the user to delete one
    /// (issue #43).  A NaN anywhere is not stationary either; nothing is known there.
    pub fn stationary(&mut self, z: &[f64]) -> bool {
        let r = self.residuals(z);
        self.compute_csr(z);
        let mut g = vec![0.0; self.n_free];
        let (mut jmax, mut rmax) = (0.0f64, 0.0f64);
        for row in 0..self.n_res {
            if !self.hard[row] {
                continue;
            }
            if r[row].is_nan() {
                return false;
            }
            rmax = rmax.max(r[row].abs());
            for p in self.csr_indptr[row]..self.csr_indptr[row + 1] {
                let p = p as usize;
                let v = self.csr_data[p];
                if v.is_nan() {
                    return false;
                }
                jmax = jmax.max(v.abs());
                g[self.csr_indices[p] as usize] += v * r[row];
            }
        }
        let gmax = g.iter().fold(0.0f64, |m, v| m.max(v.abs()));
        gmax <= STATIONARY_TOL * jmax * rmax
    }

    /// max |residual| / (that row's units) over the hard rows — dimensionless, so one threshold
    /// judges every kernel.  This is what "solved" means.  The division happened where the
    /// residual was produced (`row_scale`), so there is nothing left to do here but the max.
    /// `locus::assemble` states the same rule in miniature for a trace block's inner rows; an
    /// edit to what counts toward a row's units has a twin there.
    pub fn max_relative_residual(&mut self, z: &[f64]) -> f64 {
        self.max_hard_residual(z)
    }

    /// max |residual| per constraint, in block order (`self.cids`), each in its own units.
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
        let n_extra: usize = extra.iter().map(|c| c.rows_in(sk)).sum();
        let mut m = Mat::zeros(base.rows() + n_extra, self.n_free);
        m.data[..base.as_mat().data.len()].copy_from_slice(&base.as_mat().data);
        let mut r = base.rows();
        for c in extra {
            let ps = c.params(sk);
            let v = c.local_values(sk);
            let j = c.jacobian(sk, &v);
            let kn = c.kernel_in(sk);
            let inv = 1.0 / self.extent.max(1.0).dpowi(kn.degree as i32 - 1);
            let mut cs = Vec::new();
            for t in 0..kn.n_res {
                for (k, &p) in ps.iter().enumerate() {
                    cs.clear();
                    fold(&self.col_of, &self.dcols, p as i32, &mut cs);
                    for &(col, w) in &cs {
                        m.data[r * self.n_free + col as usize] += w * j[t * kn.n_par + k] * inv;
                    }
                }
                row_c.push(c.id);
                r += 1;
            }
        }
        (Conditioned { m }, row_c)
    }

    /// `conditioned`'s numbers as sparse rows, never made dense: what the diagnosis reads a
    /// system past the dense limit by, part by part (`diagnose::parts`, #88).  Rows are of the
    /// full residual vector, columns free columns.
    pub fn conditioned_sparse(&mut self, z: &[f64]) -> SparseConditioned {
        let mut j = SparseConditioned {
            n_cols: self.n_free,
            indptr: vec![0; self.n_res + 1],
            indices: Vec::new(),
            data: Vec::new(),
        };
        if self.n_free == 0 || self.n_res == 0 {
            return j;
        }
        self.compute_csr(z);
        j.indptr = self.csr_indptr.clone();
        j.indices = self.csr_indices.clone();
        j.data = self.csr_data.clone();
        for r in 0..self.n_res {
            // the CSR row is already over `row_scale`; conditioning wants it over `jac_scale`
            let inv = self.row_scale[r] / self.jac_scale[r];
            for p in j.indptr[r]..j.indptr[r + 1] {
                j.data[p as usize] *= inv;
            }
        }
        j
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
                let mut cs = Vec::new();
                for t in 0..kn.n_par {
                    cs.clear();
                    fold(&self.col_of, &self.dcols, b.gidx[i * kn.n_par + t], &mut cs);
                    cols.extend(cs.iter().map(|&(c, _)| c as usize));
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

    /// The constraint instance a residual row belongs to: (kernel block, instance in it).
    pub fn instance_of(&self, row: usize) -> (usize, usize) {
        let b = self.blocks.partition_point(|b| b.row0 <= row) - 1;
        (b, (row - self.blocks[b].row0) / self.kernels[self.blocks[b].kid].n_res)
    }

    /// The block-triangular order of the hard rows — see `BlockOrder`.  Worked out once per
    /// compile: `structure()`, `graph::dulmage_mendelsohn`, then `graph::blocks` over the
    /// matched square part.  A constraint's rows name the same columns, so they always fall in
    /// one block (each reads the columns the others are matched to).
    pub fn block_order(&mut self) -> Arc<BlockOrder> {
        if let Some(o) = &self.order {
            return o.clone();
        }
        let (adj, _) = self.structure();
        let hard = self.hard_rows();
        let dm = crate::graph::dulmage_mendelsohn(&adj, self.n_free);
        let blt = crate::graph::blocks(&adj, &dm);
        let mut order = BlockOrder { level: blt.level, ..BlockOrder::default() };
        for rows in &blt.rows {
            let mut cols: Vec<usize> = rows.iter().map(|&r| dm.mate_row[r] as usize).collect();
            cols.sort_unstable();
            let mut instances: Vec<(usize, usize)> =
                rows.iter().map(|&r| self.instance_of(hard[r])).collect();
            instances.dedup();
            let full = self.rows_of(&instances);
            debug_assert_eq!(full, rows.iter().map(|&r| hard[r]).collect::<Vec<_>>());
            order.blocks.push(SolveBlock { rows: full, cols, instances });
        }
        order.over_rows = dm.over_rows.iter().map(|&r| hard[r]).collect();
        order.under_rows = dm.under_rows.iter().map(|&r| hard[r]).collect();
        order.over_cols = dm.over_cols;
        order.under_cols = dm.under_cols;
        let order = Arc::new(order);
        self.order = Some(order.clone());
        order
    }

    /// Every residual row of the given instances, in order.
    fn rows_of(&self, instances: &[(usize, usize)]) -> Vec<usize> {
        let mut rows = Vec::new();
        for &(b, i) in instances {
            let n_res = self.kernels[self.blocks[b].kid].n_res;
            let r0 = self.blocks[b].row0 + i * n_res;
            rows.extend(r0..r0 + n_res);
        }
        rows
    }

    /// The seam a block is solved through: `instances` (kernel block, instance) evaluated on
    /// their own, differentiated against `cols` (free columns) only — see `Subset`.
    pub fn subset(&self, instances: &[(usize, usize)], cols: &[usize]) -> Subset {
        let mut inst = instances.to_vec();
        inst.sort_unstable();
        inst.dedup();
        let mut local = vec![-1i32; self.n_free];
        for (k, &c) in cols.iter().enumerate() {
            local[c] = k as i32;
        }
        // contiguous instances of one kernel block are one call, as the whole system's are
        let mut runs: Vec<(usize, usize, usize, usize, usize)> = Vec::new();
        let (mut out, mut joff) = (0usize, 0usize);
        for &(b, i) in &inst {
            let kn = self.kernels[self.blocks[b].kid];
            match runs.last_mut() {
                Some(run) if run.0 == b && run.1 + run.2 == i => run.2 += 1,
                _ => runs.push((b, i, 1, out, joff)),
            }
            out += kn.n_res;
            joff += kn.n_res * kn.n_par;
        }
        let rows = self.rows_of(&inst);
        // entries in the whole system's order — (instance, row, parameter) — so a column named
        // twice sums in the same order, merged as `new` merges them
        let ncols = cols.len().max(1) as i64;
        let mut es: Vec<(i64, i32, f64)> = Vec::with_capacity(joff);
        let mut jdata = vec![0.0; joff];
        for &(b, i0, count, out0, joff0) in &runs {
            let blk = &self.blocks[b];
            let kn = self.kernels[blk.kid];
            let gidx = &blk.gidx[i0 * kn.n_par..(i0 + count) * kn.n_par];
            let col = |p: i32, cs: &mut Vec<(i64, f64)>| {
                fold(&self.col_of, &self.dcols, p, cs);
                cs.retain_mut(|(c, _)| {
                    *c = local[*c as usize] as i64;
                    *c >= 0
                });
            };
            jac_entries(&mut es, kn, gidx, count, out0, joff0, ncols, col);
            fill_const_jac(kn, count, &mut jdata[joff0..]);
        }
        let Csr { indptr, indices, ent_src, ent_slot, ent_w } = csr_structure(es, rows.len(), ncols);
        let data = vec![0.0; indices.len()];
        Subset { runs, rows, cols: cols.to_vec(), indptr, indices, data, ent_src, ent_slot, ent_w,
            jdata, v: Vec::new() }
    }

    /// The subset's residuals at `z` (the whole free vector), one per `s.rows`, each over its
    /// row's units exactly as `residuals_into` hands it out.
    pub fn subset_residuals(&mut self, s: &mut Subset, z: &[f64], out: &mut [f64]) {
        self.apply_z(z);
        for &(b, i0, count, out0, _) in &s.runs {
            let blk = &self.blocks[b];
            let kn = self.kernels[blk.kid];
            gather(&mut s.v, &self.x, &blk.gidx[i0 * kn.n_par..(i0 + count) * kn.n_par]);
            let k = &blk.consts[i0 * kn.n_const..(i0 + count) * kn.n_const];
            (kn.res)(count, &s.v, k, &mut out[out0..out0 + count * kn.n_res]);
        }
        if self.row_scaled {
            for (k, &r) in s.rows.iter().enumerate() {
                out[k] /= self.row_scale[r];
            }
        }
    }

    /// Fill `s.data`: the subset's Jacobian at `z` against its columns, each value the whole
    /// system's `compute_csr` makes for that row and column.
    pub fn subset_csr(&mut self, s: &mut Subset, z: &[f64]) {
        self.apply_z(z);
        for &(b, i0, count, _, joff0) in &s.runs {
            let blk = &self.blocks[b];
            let kn = self.kernels[blk.kid];
            if kn.const_jac.is_some() {
                continue;
            }
            gather(&mut s.v, &self.x, &blk.gidx[i0 * kn.n_par..(i0 + count) * kn.n_par]);
            let k = &blk.consts[i0 * kn.n_const..(i0 + count) * kn.n_const];
            let sz = count * kn.n_res * kn.n_par;
            (kn.jac)(count, &s.v, k, &mut s.jdata[joff0..joff0 + sz]);
        }
        assemble(&mut s.data, &s.jdata, &s.ent_src, &s.ent_slot, &s.ent_w);
        if self.scaled || self.row_scaled {
            let (rs, cs, rows, cols) = (&self.row_scale, &self.col_scale, &s.rows, &s.cols);
            to_units(&mut s.data, &s.indptr, &s.indices, |k| rs[rows[k]], |j| cs[cols[j]]);
        }
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
        csr_mul(&self.csr_indptr, &self.csr_indices, &self.csr_data, v, out);
    }
}
