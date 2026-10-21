//! **A parent solved through its instances** (#454): the prototype of a reusable solve boundary,
//! against the contract in `docs/solve-boundaries-plan.md`.
//!
//! An instance marked as a boundary is taken out of the parent's system.  Its own unknowns `y`
//! are solved by its own rows `F(y, x) = 0`, a square block, with its inputs `x` held where the
//! parent has them; the parent's unknowns are everything else, and its rows every other row, read
//! at `(z, y(z))`.  The parent's DogLeg sees each instance as a smooth function of its inputs, and
//! the derivative it steps on is the implicit function theorem on the block's own factorisation:
//! `dy/dx = −F_y⁻¹ F_x`, one factorisation for every input column at once (DD1).  That is the
//! locus's machinery (`locus.rs`) taken from a traced point over a running formal to a component
//! instance: the same square Newton (`newton::square`), the same derivative, and the same resume,
//! each evaluation started from the last one accepted by its tangent's prediction and checked
//! against it (`continues`, DT5).
//!
//! Nothing is compiled twice.  The instance's block is a `Subset` of the one `System` the flat
//! document compiles to, so its rows are the flat solve's rows in the flat solve's units, and the
//! parent's are the rest.  That also means each row is judged at the *document's* scale, not the
//! boundary's own: CL1's own scale is #455's to set.
//!
//! **No syntax.**  A boundary is named from Rust (`Boundary`, built for a component instance by
//! `boundary::instance`), so the experiment commits the language to nothing.
//!
//! What a boundary must be, and what an evaluation reports:
//!
//! * **Square** (FR1, FR3): as many rows as unknowns of its own, so `F_y` is square and its
//!   derivative never a minimum-norm choice.  A boundary that is not is refused.
//! * **A value or a failure** (FA1): an inner solve that does not settle, a singular `F_y`, or a
//!   step that does not continue the branch it resumed from — the correction against the move the
//!   last derivative predicted, or the sign of `det F_y` changed (DT5, DD3) — is a failure, and
//!   the parent reads NaN rows for it, which its DogLeg rejects as it rejects any NaN residual
//!   (FA2).  Never a nearby value.
//! * **Outputs flow one way** (IO4): one boundary may read another's unknowns as inputs, and they
//!   are evaluated in that order, the derivative chained through.  Boundaries that read each
//!   other round a cycle are the parent's coupled system: they are solved in the parent, flat,
//!   and said to be (`Boundaries::inlined`).
//!
//! What it does not do yet, each its issue's: a condition estimate and its threshold (DD2), a
//! domain (DD4, DD5) and the path-independence check (DT6) are #455's; a fresh evaluation starts
//! from the pose the document gave it, not from seeds stated relative to its inputs, and nothing
//! is cached across instances (DT4, #456); claims and bounds inside a boundary are not judged
//! through it (CM1).

use super::{SolveOpts, SolveResult};
use crate::linalg::{absmax, norm, Mat};
use crate::model::Sketch;
use crate::newton::{self, Square, SquareTol, SquareWork, TrustRegion};
use crate::system::{Subset, System, RANK_TOL};
use std::collections::BTreeSet;
use std::fmt;

/// One instance a parent solves through: what it made and what it states.
#[derive(Clone, Debug, Default)]
pub struct Boundary {
    /// Its name, as its members' names start: `upper`, `drive.upper`.
    pub name: String,
    /// Its own unknowns: the parameters of what it made.  Those the document holds are held.
    pub own: BTreeSet<u32>,
    /// Its acting statements, by constraint id: the rows it is solved by.  A statement that
    /// compiles no row (a claim, a bound) is passed over.
    pub rows: BTreeSet<u32>,
}

/// Why an evaluation failed (FA1).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Reason {
    /// The inner solve did not settle.
    Unsettled,
    /// `F_y` has no inverse at the value.
    Singular,
    /// A resumed evaluation did not continue the branch it resumed from: its correction was more
    /// than the move predicted, or the sign of `det F_y` changed.
    BranchChanged,
}

impl fmt::Display for Reason {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        f.write_str(match self {
            Reason::Unsettled => "unsettled",
            Reason::Singular => "singular",
            Reason::BranchChanged => "branch changed",
        })
    }
}

/// A failed evaluation: which boundary, why, and its inputs where it failed (parameter units, in
/// `Boundaries::inputs` order).
#[derive(Clone, Debug, PartialEq)]
pub struct Failure {
    pub boundary: String,
    pub reason: Reason,
    pub inputs: Vec<f64>,
}

impl fmt::Display for Failure {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "`{}` failed: {}", self.boundary, self.reason)
    }
}

/// An accepted evaluation, in parameter units.
#[derive(Clone, Debug)]
pub struct Evaluation {
    /// Its unknowns, in `Boundaries::unknowns` order.
    pub y: Vec<f64>,
    /// `dy/dx`: a row per unknown, a column per input (`Boundaries::inputs`).
    pub dydx: Mat,
}

/// The last value an instance accepted, in the solver's units: where the next evaluation
/// resumes from.
#[derive(Clone, Debug)]
struct Seen {
    x: Vec<f64>,
    y: Vec<f64>,
    /// `dy/dx`, `y × x`.
    d: Mat,
    sign: i8,
}

/// One instance, compiled: its columns, its block, and where it last got to.
struct Inner {
    name: String,
    /// Its own unknowns and its inputs, as free columns.
    ycols: Vec<usize>,
    xcols: Vec<usize>,
    /// Its rows, differentiated against `ycols` then `xcols`.
    sub: Subset,
    /// The unknowns the document started them at: a fresh evaluation's start.
    start: Vec<f64>,
    last: Option<Seen>,
    /// Inner solves made, the Newton iterations they took (each one assembly and LU of `F_y`),
    /// and evaluations asked again at the last one's inputs and answered from it.
    evaluations: usize,
    iterations: usize,
    again: usize,
    r: Vec<f64>,
    j: Vec<f64>,
}

/// Where a free column is: the parent's `k`th unknown, or row `k` of the boundaries' unknowns
/// stacked in the order they are evaluated (`Boundaries::dydz`).
#[derive(Clone, Copy)]
enum Place {
    Parent(usize),
    Own(usize),
}

/// The instances a parent is solved through, over the one `System` the document compiles to.
pub struct Boundaries {
    sys: System,
    /// The whole free vector: the parent's columns and every instance's.
    z: Vec<f64>,
    /// The parent's own unknowns, as free columns.
    zcols: Vec<usize>,
    /// The instances, in the order they are evaluated: each after those it reads.
    inner: Vec<Inner>,
    /// The boundaries that read each other round a cycle, solved in the parent instead (IO4).
    inlined: Vec<String>,
    /// The parent's rows, against `zcols` then every instance's `ycols` in order.
    parent: Subset,
    /// Each free column's place.
    place: Vec<Place>,
    work: SquareWork,
}

impl Boundaries {
    /// Compile `sk` and take each of `bounds` out of it.  An `Err` says which boundary cannot be
    /// one, and why.
    pub fn new(sk: &Sketch, bounds: &[Boundary]) -> Result<Boundaries, String> {
        let mut sys = System::new(sk);
        let z = sys.z0(sk);
        let (adj, row_c) = sys.structure();
        let mut taken_rows: BTreeSet<u32> = BTreeSet::new();
        let mut taken_cols: BTreeSet<usize> = BTreeSet::new();
        let mut all: Vec<(Inner, Vec<(usize, usize)>)> = Vec::new();
        for b in bounds {
            let mut ycols = Vec::new();
            for &p in &b.own {
                match sys.col_of.get(p as usize).copied() {
                    Some(c) if c >= 0 => ycols.push(c as usize),
                    Some(c) if c <= -2 => {
                        return Err(format!("`{}` owns a ring's turned copy, which is worked out \
                                            from its bases and solved by none", b.name));
                    }
                    _ => {}
                }
            }
            if ycols.is_empty() {
                return Err(format!("`{}` has no unknown of its own to solve", b.name));
            }
            if ycols.iter().any(|&c| !taken_cols.insert(c)) {
                return Err(format!("`{}` shares an unknown with another boundary", b.name));
            }
            // its rows, as kernel instances
            let mut instances = Vec::new();
            for &cid in &b.rows {
                let Some(&slot) = sys.slot_of.get(&cid) else { continue };
                if !taken_rows.insert(cid) {
                    return Err(format!("`{}` shares a statement with another boundary", b.name));
                }
                instances.push(slot);
            }
            instances.sort_unstable();
            let n_rows = sys.rows_of(&instances).len();
            if n_rows != ycols.len() {
                return Err(format!(
                    "`{}` is not square: {} unknowns of its own against {} equations (FR1)",
                    b.name, ycols.len(), n_rows
                ));
            }
            // its inputs: every other column its rows read
            let mut x: BTreeSet<usize> = BTreeSet::new();
            for (row, cols) in adj.iter().enumerate() {
                if b.rows.contains(&row_c[row]) {
                    x.extend(cols.iter().copied());
                }
            }
            let xcols: Vec<usize> = x.into_iter().filter(|c| !ycols.contains(c)).collect();
            let cols: Vec<usize> = ycols.iter().chain(&xcols).copied().collect();
            let inner = Inner {
                name: b.name.clone(),
                start: ycols.iter().map(|&c| z[c]).collect(),
                sub: sys.subset(&instances, &cols),
                ycols,
                xcols,
                last: None,
                evaluations: 0,
                iterations: 0,
                again: 0,
                r: Vec::new(),
                j: Vec::new(),
            };
            all.push((inner, instances));
        }
        // each after the boundaries it reads; those that read each other round a cycle are the
        // parent's (IO4), whose rows and unknowns it takes back
        let reads = |j: usize, k: usize| {
            j != k && all[k].0.ycols.iter().any(|c| all[j].0.xcols.contains(c))
        };
        let deps: Vec<Vec<usize>> =
            (0..all.len()).map(|j| (0..all.len()).filter(|&k| reads(j, k)).collect()).collect();
        let mut slots: Vec<Option<(Inner, Vec<(usize, usize)>)>> =
            all.into_iter().map(Some).collect();
        let (mut inner, mut inlined) = (Vec::new(), Vec::new());
        let mut bound_rows: BTreeSet<(usize, usize)> = BTreeSet::new();
        let mut place: Vec<Option<Place>> = vec![None; sys.n_free];
        let mut stacked = 0;
        for comp in crate::graph::tarjan(&deps) {
            if comp.len() > 1 {
                let mut names: Vec<usize> = comp;
                names.sort_unstable();
                inlined.extend(names.iter().map(|&j| slots[j].as_ref().unwrap().0.name.clone()));
                continue;
            }
            let (b, instances) = slots[comp[0]].take().unwrap();
            for &c in &b.ycols {
                place[c] = Some(Place::Own(stacked));
                stacked += 1;
            }
            bound_rows.extend(instances);
            inner.push(b);
        }
        let zcols: Vec<usize> = (0..sys.n_free).filter(|&c| place[c].is_none()).collect();
        for (k, &c) in zcols.iter().enumerate() {
            place[c] = Some(Place::Parent(k));
        }
        let rest: Vec<(usize, usize)> = sys.blocks.iter().enumerate()
            .flat_map(|(b, blk)| (0..blk.count).map(move |i| (b, i)))
            .filter(|slot| !bound_rows.contains(slot))
            .collect();
        let cols: Vec<usize> =
            zcols.iter().chain(inner.iter().flat_map(|b| &b.ycols)).copied().collect();
        let parent = sys.subset(&rest, &cols);
        let place = place.into_iter().map(|p| p.expect("every column placed")).collect();
        Ok(Boundaries { sys, z, zcols, inner, inlined, parent, place, work: SquareWork::default() })
    }

    /// The boundaries that read each other round a cycle, and so are solved in the parent.
    pub fn inlined(&self) -> &[String] {
        &self.inlined
    }

    fn find(&self, name: &str) -> Option<usize> {
        self.inner.iter().position(|b| b.name == name)
    }

    /// A boundary's own unknowns the solve moves, as parameters, in the order its values are.
    pub fn unknowns(&self, name: &str) -> Option<Vec<u32>> {
        let b = &self.inner[self.find(name)?];
        Some(b.ycols.iter().map(|&c| self.sys.free[c] as u32).collect())
    }

    /// A boundary's inputs the solve moves, as parameters: every other column its rows read.
    /// What the document holds is a constant of it, and no input.
    pub fn inputs(&self, name: &str) -> Option<Vec<u32>> {
        let b = &self.inner[self.find(name)?];
        Some(b.xcols.iter().map(|&c| self.sys.free[c] as u32).collect())
    }

    /// Evaluate one boundary at `x` (parameter units, `inputs` order), resumed from the last value
    /// it accepted.  Panics where `name` is no boundary here or `x` is not its inputs' width.
    pub fn evaluate(&mut self, name: &str, x: &[f64]) -> Result<Evaluation, Failure> {
        let k = self.find(name).expect("no such boundary");
        let b = &self.inner[k];
        assert_eq!(x.len(), b.xcols.len(), "`{name}` has {} inputs", b.xcols.len());
        for (&c, &v) in b.xcols.iter().zip(x) {
            self.z[c] = v * self.sys.col_scale[c];
        }
        self.evaluate_one(k)?;
        let (b, cs) = (&self.inner[k], &self.sys.col_scale);
        let s = b.last.as_ref().expect("an accepted evaluation is remembered");
        let mut dydx = Mat::zeros(b.ycols.len(), b.xcols.len());
        for (i, &yc) in b.ycols.iter().enumerate() {
            for (j, &xc) in b.xcols.iter().enumerate() {
                dydx.set(i, j, s.d.at(i, j) * cs[xc] / cs[yc]);
            }
        }
        let y = b.ycols.iter().zip(&s.y).map(|(&c, &v)| v / cs[c]).collect();
        Ok(Evaluation { y, dydx })
    }

    /// Evaluate the `k`th boundary at the inputs `z` holds, and leave its value there.
    fn evaluate_one(&mut self, k: usize) -> Result<(), Failure> {
        let b = &mut self.inner[k];
        evaluate(&mut self.sys, &mut self.z, b, &mut self.work).map_err(|reason| Failure {
            boundary: b.name.clone(),
            reason,
            inputs: b.xcols.iter().map(|&c| self.z[c] / self.sys.col_scale[c]).collect(),
        })
    }

    /// Every boundary at the parent's unknowns `zp`, in order.
    fn evaluate_all(&mut self, zp: &[f64]) -> Result<(), Failure> {
        for (k, &c) in self.zcols.iter().enumerate() {
            self.z[c] = zp[k];
        }
        (0..self.inner.len()).try_for_each(|k| self.evaluate_one(k))
    }

    /// `dy/dz`: every boundary's unknowns stacked in the order they are evaluated (`Place::Own`),
    /// against the parent's unknowns, each boundary's chained through those it reads.
    fn dydz(&self) -> Mat {
        let nz = self.zcols.len();
        let mut g = Mat::zeros(self.inner.iter().map(|b| b.ycols.len()).sum(), nz);
        let mut at = 0;
        for b in &self.inner {
            let s = b.last.as_ref().expect("evaluated");
            // the rows read are earlier boundaries', all above this one's
            let (above, here) = g.data.split_at_mut(at * nz);
            for (j, &c) in b.xcols.iter().enumerate() {
                for i in 0..b.ycols.len() {
                    let d = s.d.at(i, j);
                    if d == 0.0 {
                        continue;
                    }
                    let row = &mut here[i * nz..(i + 1) * nz];
                    match self.place[c] {
                        Place::Parent(k) => row[k] += d,
                        Place::Own(t) => {
                            let from = &above[t * nz..(t + 1) * nz];
                            row.iter_mut().zip(from).for_each(|(r, v)| *r += d * v);
                        }
                    }
                }
            }
            at += b.ycols.len();
        }
        g
    }

    /// The parent's Jacobian against its own unknowns, through the boundaries: `P_z + P_y dy/dz`,
    /// at the pose `z` holds (every boundary evaluated there).
    fn reduced(&mut self) -> Mat {
        let g = self.dydz();
        let (m, nz) = (self.parent.rows.len(), self.zcols.len());
        self.sys.subset_csr(&mut self.parent, &self.z);
        let mut j = Mat::zeros(m, nz);
        for r in 0..m {
            let row = &mut j.data[r * nz..(r + 1) * nz];
            for p in self.parent.indptr[r]..self.parent.indptr[r + 1] {
                let p = p as usize;
                let (c, v) = (self.parent.indices[p] as usize, self.parent.data[p]);
                // the parent's columns, then the boundaries' unknowns in `g`'s order
                if c < nz {
                    row[c] += v;
                } else {
                    let from = &g.data[(c - nz) * nz..(c - nz + 1) * nz];
                    row.iter_mut().zip(from).for_each(|(r, u)| *r += v * u);
                }
            }
        }
        j
    }
}

/// One instance's block over the whole free vector, as `newton::square` reads it: its unknowns
/// are its own columns of `z`.
struct Block<'a> {
    sys: &'a mut System,
    sub: &'a mut Subset,
    z: &'a mut [f64],
    cols: &'a [usize],
    r: &'a mut Vec<f64>,
    j: &'a mut Vec<f64>,
    iterations: &'a mut usize,
}

impl Square for Block<'_> {
    fn n(&self) -> usize {
        self.cols.len()
    }
    fn get(&self, q: &mut Vec<f64>) {
        q.clear();
        q.extend(self.cols.iter().map(|&c| self.z[c]));
    }
    fn put(&mut self, q: &[f64]) {
        for (&c, &v) in self.cols.iter().zip(q) {
            self.z[c] = v;
        }
    }
    fn assemble(&mut self, jac: bool) -> f64 {
        let n = self.cols.len();
        self.r.resize(self.sub.rows.len(), 0.0);
        self.sys.subset_residuals(self.sub, self.z, self.r);
        // `absmax` is NaN where a row is, and a NaN is no convergence
        let worst = absmax(self.r);
        if jac && worst.is_finite() {
            *self.iterations += 1;
            self.sys.subset_csr(self.sub, self.z);
            self.j.resize(n * n, 0.0);
            self.sub.dense_into(0, n, self.j);
        }
        worst
    }
    fn residual(&self) -> &[f64] {
        self.r
    }
    fn jacobian(&self) -> &[f64] {
        self.j
    }
}

/// Evaluate one instance at the inputs `z` holds: resumed from its last value by the tangent it
/// predicts, else from the pose the document started it at; then the implicit function theorem
/// at the value.  On success its value is in `z` and remembered in `b.last`.
fn evaluate(sys: &mut System, z: &mut [f64], b: &mut Inner, w: &mut SquareWork)
    -> Result<(), Reason> {
    let unmoved = |s: &Seen| b.xcols.iter().zip(&s.x).all(|(&c, &v)| z[c] == v);
    if let Some(s) = b.last.as_ref().filter(|s| unmoved(s)) {
        // asked again where it last was: the Jacobian at the point the residual was just read at
        for (&c, &v) in b.ycols.iter().zip(&s.y) {
            z[c] = v;
        }
        b.again += 1;
        return Ok(());
    }
    b.evaluations += 1;
    let x: Vec<f64> = b.xcols.iter().map(|&c| z[c]).collect();
    // where the last value's tangent says the new one is
    let predicted: Option<Vec<f64>> = b.last.as_ref().map(|s| {
        let dx: Vec<f64> = x.iter().zip(&s.x).map(|(a, c)| a - c).collect();
        s.y.iter().zip(s.d.mul_vec(&dx)).map(|(y, d)| y + d).collect()
    });
    for (&c, &v) in b.ycols.iter().zip(predicted.as_ref().unwrap_or(&b.start)) {
        z[c] = v;
    }
    let mut block = Block {
        sys,
        sub: &mut b.sub,
        z,
        cols: &b.ycols,
        r: &mut b.r,
        j: &mut b.j,
        iterations: &mut b.iterations,
    };
    if !newton::square(&mut block, w, SquareTol::BLOCK) {
        return Err(Reason::Unsettled);
    }
    let y: Vec<f64> = b.ycols.iter().map(|&c| block.z[c]).collect();
    if let (Some(s), Some(p)) = (&b.last, &predicted) {
        let apart = |a: &[f64], c: &[f64]| {
            norm(&a.iter().zip(c).map(|(u, v)| u - v).collect::<Vec<_>>())
        };
        if !newton::continues(apart(&y, p), apart(p, &s.y), 1.0 + absmax(&s.y)) {
            return Err(Reason::BranchChanged);
        }
    }
    // the implicit function theorem: one factorisation of F_y for every input column
    let (n, nx) = (b.ycols.len(), b.xcols.len());
    block.sys.subset_csr(block.sub, block.z);
    let mut fy = vec![0.0; n * n];
    let mut fx = Mat::zeros(n, nx);
    b.sub.dense_into(0, n, &mut fy);
    b.sub.dense_into(n, n + nx, &mut fx.data);
    let mut piv = Vec::with_capacity(n);
    if !crate::linalg::lu_factor(n, &mut fy, &mut piv) {
        return Err(Reason::Singular);
    }
    let sign = crate::linalg::lu_det_sign(n, &fy, &piv);
    if b.last.as_ref().is_some_and(|s| s.sign != sign) {
        return Err(Reason::BranchChanged);
    }
    let mut d = Mat::zeros(n, nx);
    let mut rhs = vec![0.0; n];
    for j in 0..nx {
        for (i, r) in rhs.iter_mut().enumerate() {
            *r = -fx.at(i, j);
        }
        crate::linalg::lu_apply(n, &fy, &piv, &mut rhs);
        for (i, &v) in rhs.iter().enumerate() {
            d.set(i, j, v);
        }
    }
    b.last = Some(Seen { x, y, d, sign });
    Ok(())
}

/// The parent as a `TrustRegion`: its rows over its own unknowns, every boundary evaluated at
/// them.  A failed evaluation is NaN rows, which the DogLeg rejects and shrinks for.
struct Parent<'a> {
    b: &'a mut Boundaries,
    j: Mat,
    rank: i32,
    /// The last evaluation that failed, and how many did.
    failure: Option<Failure>,
    failures: usize,
}

impl TrustRegion for Parent<'_> {
    fn n(&self) -> usize {
        self.b.zcols.len()
    }
    fn m(&self) -> usize {
        self.b.parent.rows.len()
    }
    fn residuals_into(&mut self, zp: &[f64], out: &mut [f64]) {
        match self.b.evaluate_all(zp) {
            Ok(()) => self.b.sys.subset_residuals(&mut self.b.parent, &self.b.z, out),
            Err(f) => {
                out.fill(f64::NAN);
                self.failure = Some(f);
                self.failures += 1;
            }
        }
    }
    fn jacobian_at(&mut self, zp: &[f64]) {
        self.j = match self.b.evaluate_all(zp) {
            Ok(()) => self.b.reduced(),
            Err(_) => Mat::from_vec(self.m(), self.n(), vec![f64::NAN; self.m() * self.n()]),
        };
    }
    fn jt_mul(&mut self, v: &[f64], out: &mut [f64]) {
        out.copy_from_slice(&self.j.mul_t_vec(v));
    }
    fn j_mul(&mut self, v: &[f64], out: &mut [f64]) {
        out.copy_from_slice(&self.j.mul_vec(v));
    }
    fn gn_step(&mut self, r: &[f64], _g: &[f64], p: &mut [f64]) {
        if !self.j.data.iter().all(|x| x.is_finite()) {
            p.fill(0.0);
            self.rank = -1;
            return;
        }
        self.rank = newton::min_norm_step(&self.j, r, p);
    }
    fn rank(&self) -> i32 {
        self.rank
    }
}

/// What a solve through boundaries came to.
#[derive(Clone, Debug)]
pub struct Through {
    /// Judged on the flat document's rows at the pose it ended on: what "solved" means is the
    /// same with and without boundaries (FL1).
    pub result: SolveResult,
    /// The evaluation the solve ended on, where it failed: the parent's failure (FA3).  Where it
    /// ended on values and still did not solve, the last evaluation that failed along the way:
    /// the boundary that would not go where the parent's step asked (a parent at a domain's
    /// edge stops short of it, DD5).
    pub failure: Option<Failure>,
    /// How many evaluations failed along the way, each a step the parent rejected (FA2).
    pub rejected: usize,
    /// Each boundary solved through and how many inner solves it made, in evaluation order.
    pub evaluations: Vec<(String, usize)>,
    /// The Newton iterations of every inner solve, each one assembly and LU of a block's `F_y`
    /// (the derivative's own LU, one per accepted solve, not counted).
    pub inner_iterations: usize,
    /// Evaluations asked again at the inputs of a boundary's last one and answered from it, no
    /// solve made: the parent's Jacobian read where its residual just was, and every boundary
    /// whose inputs a step left where they were.
    pub answered_again: usize,
    /// The boundaries that read each other round a cycle and were solved in the parent (IO4).
    pub inlined: Vec<String>,
    /// The parent's own unknowns, and every boundary's.
    pub parent_unknowns: usize,
    pub boundary_unknowns: usize,
    /// The parent's rows: its Jacobian through the boundaries is a dense `parent_rows ×
    /// parent_unknowns`, factored at every step (`Boundaries::reduced`).
    pub parent_rows: usize,
    /// The most unknowns one boundary has: the largest `F_y` an inner solve factors.
    pub largest_boundary: usize,
    /// The rank of the parent's rows over its own unknowns through the boundaries' derivatives,
    /// conditioned as the diagnosis conditions a system, at the pose it ended on (FL3).
    pub reduced_rank: usize,
}

/// Solve `sk` through `bounds`: the parent's DogLeg over its own unknowns, every boundary solved
/// at each of its steps.  The pose it ends on is written back.  An `Err` is a boundary that
/// cannot be one (`Boundaries::new`).
pub fn solve_through(sk: &mut Sketch, bounds: &[Boundary], opts: SolveOpts)
    -> Result<Through, String> {
    let mut b = Boundaries::new(sk, bounds)?;
    let mut zp: Vec<f64> = b.zcols.iter().map(|&c| b.z[c]).collect();
    let mut t = Parent { b: &mut b, j: Mat::zeros(0, 0), rank: -1, failure: None, failures: 0 };
    let (m, n) = (t.m(), t.n());
    let mut r = vec![0.0; m];
    t.residuals_into(&zp, &mut r);
    let info = if m > 0 && n > 0 && r.iter().all(|v| v.is_finite()) {
        let tol = t.b.sys.tolerances(&opts);
        let max_nfev = if opts.max_nfev <= 0 { 4 * opts.max_iter } else { opts.max_nfev };
        newton::dogleg(&mut t, &mut zp, &mut r, tol, opts.max_iter, max_nfev)
    } else {
        newton::Info { status: if r.iter().all(|v| v.is_finite()) { 0 } else { -1 }, nfev: 1,
                       ..newton::Info::default() }
    };
    let (rejected, last) = (t.failures, t.failure.take());
    // the boundaries at the pose it ended on: a rejected trial last wrote their columns
    let ended = b.evaluate_all(&zp).err();
    let reduced_rank = match ended {
        None if n > 0 && m > 0 => {
            let j = b.reduced();
            b.sys.conditioned_of(&b.parent.rows, j).rank_rrqr(RANK_TOL)
        }
        _ => 0,
    };
    let z = b.z.clone();
    let max = if ended.is_some() { f64::NAN } else { b.sys.max_relative_residual(&z) };
    let all = b.sys.residuals(&z);
    let residual_norm = norm(&all);
    if opts.writeback {
        sk.set_x(&b.sys.full_x(&z));
    }
    let success = ended.is_none() && info.status >= 0 && max < opts.acceptance_tol;
    // a solve that stopped short of a solution was held back by the evaluations it could not make
    let failure = ended.or(if success { None } else { last });
    let mut message = newton::status_message(info.status).to_string();
    if let Some(f) = &failure {
        message = format!("{message}; {f}");
    }
    let result = SolveResult {
        success,
        status: info.status,
        message,
        residual_norm,
        max_residual: max,
        nfev: info.nfev,
        njev: info.njev,
        time_s: 0.0,
        method: "dogleg through boundaries".to_string(),
        iterations: info.iterations,
        rank: (info.rank >= 0).then_some(info.rank),
    };
    Ok(Through {
        result,
        failure,
        rejected,
        evaluations: b.inner.iter().map(|i| (i.name.clone(), i.evaluations)).collect(),
        inner_iterations: b.inner.iter().map(|i| i.iterations).sum(),
        answered_again: b.inner.iter().map(|i| i.again).sum(),
        inlined: b.inlined.clone(),
        parent_unknowns: b.zcols.len(),
        boundary_unknowns: b.inner.iter().map(|i| i.ycols.len()).sum(),
        parent_rows: b.parent.rows.len(),
        largest_boundary: b.inner.iter().map(|i| i.ycols.len()).max().unwrap_or(0),
        reduced_rank,
    })
}
