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
use crate::clock::Instant;
use crate::linalg::{absmax, norm, Mat};
use crate::model::Sketch;
use crate::newton::{self, Square, SquareTol, SquareWork, TrustRegion};
use crate::sparse::Ata;
use crate::system::{csr_mul, Conditioned, Subset, System, DENSE_MAX, RANK_TOL};
use std::collections::{BTreeMap, BTreeSet};
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

/// Where one of an instance's inputs comes from, for its derivative against the parent's
/// unknowns (`Boundaries::chain`).
enum Input {
    /// A parent unknown, at this place of the instance's `dep`.
    Parent(usize),
    /// Row `row` of the earlier instance `from` (in evaluation order), whose `dep` sits at `at`
    /// of this one's.
    Own { from: usize, row: usize, at: Vec<usize> },
}

/// One instance, compiled: its columns, its block, and where it last got to.
struct Inner {
    name: String,
    /// Its own unknowns and its inputs, as free columns.
    ycols: Vec<usize>,
    xcols: Vec<usize>,
    /// The parent unknowns its value moves with, ascending: those it reads, and those every
    /// instance it reads moves with.  Its `dy/dz` is over these alone, so a parent of a thousand
    /// parts holds a thousand small derivatives, not one dense one (#459).
    dep: Vec<usize>,
    /// Each input's source, in `xcols` order.
    inputs: Vec<Input>,
    /// `dy/dz` over `dep`, `ycols × dep`, as `chain` last made it.
    g: Mat,
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
/// stacked in the order they are evaluated.
#[derive(Clone, Copy)]
enum Place {
    Parent(usize),
    Own(usize),
}

/// The parent's Jacobian through the boundaries, `P_z + P_y·dy/dz`, as compressed rows over the
/// parent's unknowns.  Its structure is fixed when the boundaries are taken out: a parent row
/// reads the parent unknowns it reads itself and every one the boundaries it reads move with.
struct Reduced {
    indptr: Vec<i32>,
    indices: Vec<i32>,
    data: Vec<f64>,
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
    /// Each stacked unknown of the boundaries (`Place::Own`): its instance and its row there.
    own: Vec<(usize, usize)>,
    /// The parent's Jacobian through the boundaries, and a row of it as it is summed.
    reduced: Reduced,
    acc: Vec<f64>,
    work: SquareWork,
    /// Seconds spent compiling the document (`System::new`), and evaluating the boundaries
    /// (`evaluate_all`): inner solves, their derivatives, and the evaluations answered again.
    compile_s: f64,
    inner_s: f64,
}

impl Boundaries {
    /// Compile `sk` and take each of `bounds` out of it.  An `Err` says which boundary cannot be
    /// one, and why.
    pub fn new(sk: &Sketch, bounds: &[Boundary]) -> Result<Boundaries, String> {
        let clock = Instant::now();
        let mut sys = System::new(sk);
        let compile_s = clock.elapsed().as_secs_f64();
        let z = sys.z0(sk);
        let (adj, row_c) = sys.structure();
        let mut rows_of: BTreeMap<u32, Vec<usize>> = BTreeMap::new();
        for (row, &cid) in row_c.iter().enumerate() {
            rows_of.entry(cid).or_default().push(row);
        }
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
            for row in b.rows.iter().filter_map(|cid| rows_of.get(cid)).flatten() {
                x.extend(adj[*row].iter().copied());
            }
            let xcols: Vec<usize> = x.into_iter().filter(|c| !ycols.contains(c)).collect();
            let cols: Vec<usize> = ycols.iter().chain(&xcols).copied().collect();
            let inner = Inner {
                name: b.name.clone(),
                start: ycols.iter().map(|&c| z[c]).collect(),
                sub: sys.subset(&instances, &cols),
                ycols,
                xcols,
                dep: Vec::new(),
                inputs: Vec::new(),
                g: Mat::zeros(0, 0),
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
        let mut owner = vec![usize::MAX; sys.n_free];
        for (k, (b, _)) in all.iter().enumerate() {
            b.ycols.iter().for_each(|&c| owner[c] = k);
        }
        let deps: Vec<Vec<usize>> = all.iter().enumerate().map(|(j, (b, _))| {
            let mut d: Vec<usize> = b.xcols.iter().map(|&c| owner[c])
                .filter(|&k| k != usize::MAX && k != j).collect();
            d.sort_unstable();
            d.dedup();
            d
        }).collect();
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
        let place: Vec<Place> =
            place.into_iter().map(|p| p.expect("every column placed")).collect();
        // what each instance's value moves with, each after those it reads
        let mut own: Vec<(usize, usize)> = Vec::new();
        for k in 0..inner.len() {
            let (done, rest) = inner.split_at_mut(k);
            let b = &mut rest[0];
            // an input another boundary makes is one of the unknowns stacked before this one's
            let from_of = |t: usize| own[t];
            let mut dep: BTreeSet<usize> = BTreeSet::new();
            for &c in &b.xcols {
                match place[c] {
                    Place::Parent(p) => {
                        dep.insert(p);
                    }
                    Place::Own(t) => dep.extend(&done[from_of(t).0].dep),
                }
            }
            b.dep = dep.into_iter().collect();
            let at = |p: usize| b.dep.binary_search(&p).expect("in what it moves with");
            b.inputs = b.xcols.iter().map(|&c| match place[c] {
                Place::Parent(p) => Input::Parent(at(p)),
                Place::Own(t) => {
                    let (from, row) = from_of(t);
                    Input::Own { from, row, at: done[from].dep.iter().map(|&p| at(p)).collect() }
                }
            }).collect();
            b.g = Mat::zeros(b.ycols.len(), b.dep.len());
            own.extend((0..b.ycols.len()).map(|i| (k, i)));
        }
        // the reduced Jacobian's structure: each parent row's own parent unknowns, and what
        // every boundary unknown it reads moves with
        let nz = zcols.len();
        let (mut indptr, mut indices) = (vec![0i32], Vec::new());
        let mut row: BTreeSet<usize> = BTreeSet::new();
        for r in 0..parent.rows.len() {
            row.clear();
            for p in parent.indptr[r]..parent.indptr[r + 1] {
                match parent.indices[p as usize] as usize {
                    c if c < nz => {
                        row.insert(c);
                    }
                    c => row.extend(&inner[own[c - nz].0].dep),
                }
            }
            indices.extend(row.iter().map(|&c| c as i32));
            indptr.push(indices.len() as i32);
        }
        let reduced = Reduced { data: vec![0.0; indices.len()], indptr, indices };
        Ok(Boundaries { sys, z, zcols, inner, inlined, parent, own, reduced,
                        acc: vec![0.0; nz], work: SquareWork::default(), compile_s,
                        inner_s: 0.0 })
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
        let clock = Instant::now();
        for (k, &c) in self.zcols.iter().enumerate() {
            self.z[c] = zp[k];
        }
        let done = (0..self.inner.len()).try_for_each(|k| self.evaluate_one(k));
        self.inner_s += clock.elapsed().as_secs_f64();
        done
    }

    /// Each boundary's `dy/dz` over what it moves with (`Inner::g`), chained through the
    /// boundaries it reads, in the order they are evaluated.
    fn chain(&mut self) {
        for k in 0..self.inner.len() {
            let (done, rest) = self.inner.split_at_mut(k);
            let b = &mut rest[0];
            let s = b.last.as_ref().expect("evaluated");
            let w = b.dep.len();
            b.g.data.fill(0.0);
            for (j, input) in b.inputs.iter().enumerate() {
                for i in 0..b.ycols.len() {
                    let d = s.d.at(i, j);
                    if d == 0.0 {
                        continue;
                    }
                    let row = &mut b.g.data[i * w..(i + 1) * w];
                    match input {
                        Input::Parent(at) => row[*at] += d,
                        Input::Own { from, row: t, at } => {
                            let f = &done[*from];
                            let from = &f.g.data[t * f.dep.len()..(t + 1) * f.dep.len()];
                            at.iter().zip(from).for_each(|(&a, v)| row[a] += d * v);
                        }
                    }
                }
            }
        }
    }

    /// The parent's Jacobian against its own unknowns, through the boundaries: `P_z + P_y dy/dz`,
    /// at the pose `z` holds (every boundary evaluated there), into `self.reduced`.
    fn reduce(&mut self) {
        self.chain();
        let nz = self.zcols.len();
        self.sys.subset_csr(&mut self.parent, &self.z);
        let (p, red, acc) = (&self.parent, &mut self.reduced, &mut self.acc);
        for r in 0..p.rows.len() {
            for q in p.indptr[r]..p.indptr[r + 1] {
                let (c, v) = (p.indices[q as usize] as usize, p.data[q as usize]);
                // the parent's columns, then the boundaries' unknowns in the order evaluated
                if c < nz {
                    acc[c] += v;
                } else {
                    let (k, i) = self.own[c - nz];
                    let b = &self.inner[k];
                    let g = &b.g.data[i * b.dep.len()..(i + 1) * b.dep.len()];
                    b.dep.iter().zip(g).for_each(|(&a, u)| acc[a] += v * u);
                }
            }
            for q in red.indptr[r] as usize..red.indptr[r + 1] as usize {
                let a = red.indices[q] as usize;
                red.data[q] = acc[a];
                acc[a] = 0.0;
            }
        }
    }

    /// The rows `rows` of the reduced Jacobian over the columns `cols` (ascending), dense.
    fn reduced_dense(&self, rows: &[usize], cols: &[usize]) -> Mat {
        let red = &self.reduced;
        let mut j = Mat::zeros(rows.len(), cols.len());
        for (k, &r) in rows.iter().enumerate() {
            for q in red.indptr[r] as usize..red.indptr[r + 1] as usize {
                if let Ok(c) = cols.binary_search(&(red.indices[q] as usize)) {
                    j.set(k, c, red.data[q]);
                }
            }
        }
        j
    }

    /// The reduced Jacobian's parts over its unknowns but the `held` ones: `(rows, columns)`
    /// that share no column, ascending.  A row that reads only held columns is a part with no
    /// column of its own; one that reads none is in no part.
    fn parts(&self, held: &[bool]) -> Vec<(Vec<usize>, Vec<usize>)> {
        let (m, nz) = (self.parent.rows.len(), self.zcols.len());
        let red = &self.reduced;
        let row = |r: usize| {
            red.indices[red.indptr[r] as usize..red.indptr[r + 1] as usize].iter()
                .map(|&c| c as usize).filter(|&c| !held[c])
        };
        let mut up: Vec<usize> = (0..nz).collect();
        fn root(up: &mut [usize], mut a: usize) -> usize {
            while up[a] != a {
                up[a] = up[up[a]];
                a = up[a];
            }
            a
        }
        for r in 0..m {
            let mut cs = row(r);
            if let Some(first) = cs.next() {
                for c in cs {
                    let (a, b) = (root(&mut up, first), root(&mut up, c));
                    up[a.max(b)] = a.min(b);
                }
            }
        }
        let mut parts: BTreeMap<usize, (Vec<usize>, Vec<usize>)> = BTreeMap::new();
        for c in (0..nz).filter(|&c| !held[c]) {
            parts.entry(root(&mut up, c)).or_default().1.push(c);
        }
        for r in 0..m {
            match row(r).next() {
                Some(c) => parts.entry(root(&mut up, c)).or_default().0.push(r),
                None if red.indptr[r] < red.indptr[r + 1] => {
                    parts.entry(usize::MAX).or_default().0.push(r)
                }
                None => {}
            }
        }
        parts.into_values().filter(|(rows, _)| !rows.is_empty()).collect()
    }

    /// The rank of the reduced Jacobian, conditioned as the diagnosis conditions a system,
    /// without factoring it whole.  Parts that share no column are blocks of a block-diagonal
    /// matrix, whose rank is theirs summed, so N drives that stand apart are N small
    /// factorisations, not one of `8N` columns.  An unknown every part reads (a shared centre
    /// distance) joins them all into one, so the columns read by the most rows are held out,
    /// one at a time, until no part is over `DENSE_MAX` columns (or `COUPLED` are held): the rank
    /// is each part's, plus the rank of what of the held columns no part's columns span (a
    /// block QR: `Conditioned::rank_and_rest`).
    fn reduced_rank(&self) -> usize {
        const COUPLED: usize = 32;
        let nz = self.zcols.len();
        let red = &self.reduced;
        let mut reads = vec![0usize; nz];
        red.indices.iter().for_each(|&c| reads[c as usize] += 1);
        let mut held = vec![false; nz];
        let mut n_held = 0;
        let parts = loop {
            let parts = self.parts(&held);
            match parts.iter().map(|p| &p.1).max_by_key(|cols| cols.len()) {
                Some(cols) if cols.len() > DENSE_MAX && n_held < COUPLED => {
                    let c = *cols.iter().max_by_key(|&&c| (reads[c], std::cmp::Reverse(c)))
                        .expect("a part over DENSE_MAX columns has one");
                    held[c] = true;
                    n_held += 1;
                }
                _ => break parts,
            }
        };
        let held: Vec<usize> = (0..nz).filter(|&c| held[c]).collect();
        let mut rank = 0;
        let mut rest = Vec::new();
        for (rows, cols) in &parts {
            let full: Vec<usize> = rows.iter().map(|&r| self.parent.rows[r]).collect();
            let own = self.sys.conditioned_of(&full, self.reduced_dense(rows, cols));
            if held.is_empty() {
                rank += own.rank_rrqr(RANK_TOL);
                continue;
            }
            let with = self.sys.conditioned_of(&full, self.reduced_dense(rows, &held));
            let (r, left) = own.rank_and_rest(&with, RANK_TOL);
            rank += r;
            rest.push(left);
        }
        rank + Conditioned::stack(&rest, held.len()).rank_rrqr(RANK_TOL)
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
/// them.  A failed evaluation is NaN rows, which the DogLeg rejects and shrinks for.  Its step
/// is the flat solve's (`newton::JacCtx`): the minimum-norm step of the dense Jacobian to
/// `DENSE_MAX` unknowns, the normal equations of the sparse one above, refined for a document.
struct Parent<'a> {
    b: &'a mut Boundaries,
    dense: bool,
    refine: bool,
    /// The reduced Jacobian dense, where the step is.
    j: Mat,
    /// Its normal equations, factored over the reduced structure, made at the first sparse step.
    ata: Option<Ata>,
    finite: bool,
    rank: i32,
    /// The last evaluation that failed, and how many did.
    failure: Option<Failure>,
    failures: usize,
    times: Times,
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
            Ok(()) => {
                let clock = Instant::now();
                self.b.sys.subset_residuals(&mut self.b.parent, &self.b.z, out);
                self.times.rows += clock.elapsed().as_secs_f64();
            }
            Err(f) => {
                out.fill(f64::NAN);
                self.failure = Some(f);
                self.failures += 1;
            }
        }
    }
    fn jacobian_at(&mut self, zp: &[f64]) {
        let ok = self.b.evaluate_all(zp).is_ok();
        let clock = Instant::now();
        match ok {
            true => self.b.reduce(),
            false => self.b.reduced.data.fill(f64::NAN),
        }
        self.finite = self.b.reduced.data.iter().all(|x| x.is_finite());
        if self.dense {
            let rows: Vec<usize> = (0..self.m()).collect();
            let cols: Vec<usize> = (0..self.n()).collect();
            self.j = self.b.reduced_dense(&rows, &cols);
        }
        self.times.jacobian += clock.elapsed().as_secs_f64();
    }
    fn jt_mul(&mut self, v: &[f64], out: &mut [f64]) {
        let clock = Instant::now();
        let red = &self.b.reduced;
        out.fill(0.0);
        for (r, &vr) in v.iter().enumerate() {
            for q in red.indptr[r] as usize..red.indptr[r + 1] as usize {
                out[red.indices[q] as usize] += red.data[q] * vr;
            }
        }
        self.times.step += clock.elapsed().as_secs_f64();
    }
    fn j_mul(&mut self, v: &[f64], out: &mut [f64]) {
        let clock = Instant::now();
        let red = &self.b.reduced;
        csr_mul(&red.indptr, &red.indices, &red.data, v, out);
        self.times.step += clock.elapsed().as_secs_f64();
    }
    fn gn_step(&mut self, r: &[f64], g: &[f64], p: &mut [f64]) {
        let clock = Instant::now();
        self.step(r, g, p);
        self.times.step += clock.elapsed().as_secs_f64();
    }
    fn rank(&self) -> i32 {
        self.rank
    }
}

impl Parent<'_> {
    fn step(&mut self, r: &[f64], g: &[f64], p: &mut [f64]) {
        if !self.finite {
            p.fill(0.0);
            self.rank = -1;
            return;
        }
        if self.dense {
            self.rank = newton::min_norm_step(&self.j, r, p);
            return;
        }
        let red = &self.b.reduced;
        let (m, n) = (r.len(), p.len());
        let ata = self.ata.get_or_insert_with(|| Ata::new(m, n, &red.indptr, &red.indices));
        self.rank = -1;
        let eps = newton::normal_step(ata, &red.data, g, p);
        if self.refine {
            newton::refine(ata, eps, g, p);
        }
    }
}

/// Where a solve through boundaries spent its time, seconds (zero where the target has no clock,
/// `clock::Instant`).  The phases do not overlap; what `total` holds beyond them (`other`) is the
/// DogLeg's own vector work, the flat rows judged at the end and the write back.
#[derive(Clone, Copy, Debug, Default)]
pub struct Times {
    /// Compiling the document (`System::new`, as a flat solve does first), and taking the
    /// boundaries out of it: their blocks, the order they are evaluated in, the parent's rows and
    /// the structure of its Jacobian (the rest of `Boundaries::new`).
    pub compile: f64,
    pub setup: f64,
    /// Evaluating the boundaries: every inner solve, its derivative by the implicit function
    /// theorem, and the evaluations answered again at the last inputs.  What a cache of
    /// evaluations (#456) could save at most.
    pub inner: f64,
    /// The parent's rows at each trial pose.
    pub rows: f64,
    /// The parent's Jacobian through the boundaries: the chain of `dy/dz`, the parent's own
    /// entries, and their sum (`Boundaries::reduce`).
    pub jacobian: f64,
    /// The step's linear algebra: the Gauss–Newton step (dense or the normal equations) and the
    /// products the DogLeg asks of the Jacobian.
    pub step: f64,
    /// The rank at the pose it ended on (FL3, `Boundaries::reduced_rank`).
    pub rank: f64,
    pub total: f64,
}

impl Times {
    /// What `total` holds beyond the phases.
    pub fn other(&self) -> f64 {
        self.total - self.compile - self.setup - self.inner - self.rows - self.jacobian - self.step
            - self.rank
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
    /// The parent's rows, and the entries of its Jacobian through the boundaries, `parent_rows ×
    /// parent_unknowns` (`Boundaries::reduce`): dense to `DENSE_MAX` unknowns, as the flat
    /// solve's is, and the normal equations of these entries above.
    pub parent_rows: usize,
    pub parent_entries: usize,
    /// The most unknowns one boundary has: the largest `F_y` an inner solve factors.
    pub largest_boundary: usize,
    /// The rank of the parent's rows over its own unknowns through the boundaries' derivatives,
    /// conditioned as the diagnosis conditions a system, at the pose it ended on (FL3).
    pub reduced_rank: usize,
    /// Where its time went (#459).
    pub times: Times,
}

/// Solve `sk` through `bounds`: the parent's DogLeg over its own unknowns, every boundary solved
/// at each of its steps.  The pose it ends on is written back.  An `Err` is a boundary that
/// cannot be one (`Boundaries::new`).
pub fn solve_through(sk: &mut Sketch, bounds: &[Boundary], opts: SolveOpts)
    -> Result<Through, String> {
    let clock = Instant::now();
    let mut b = Boundaries::new(sk, bounds)?;
    let setup = clock.elapsed().as_secs_f64();
    let mut zp: Vec<f64> = b.zcols.iter().map(|&c| b.z[c]).collect();
    let n = b.zcols.len();
    let mut t = Parent {
        b: &mut b,
        dense: opts.dense.unwrap_or(n <= DENSE_MAX),
        refine: opts.document(),
        j: Mat::zeros(0, 0),
        ata: None,
        finite: false,
        rank: -1,
        failure: None,
        failures: 0,
        times: Times { setup, ..Times::default() },
    };
    let m = t.m();
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
    let (rejected, last, mut times) = (t.failures, t.failure.take(), t.times);
    // the boundaries at the pose it ended on: a rejected trial last wrote their columns
    let ended = b.evaluate_all(&zp).err();
    let ranked = Instant::now();
    let reduced_rank = match ended {
        None if n > 0 && m > 0 => {
            b.reduce();
            b.reduced_rank()
        }
        _ => 0,
    };
    times.rank = ranked.elapsed().as_secs_f64();
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
        time_s: clock.elapsed().as_secs_f64(),
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
        parent_entries: b.reduced.indices.len(),
        largest_boundary: b.inner.iter().map(|i| i.ycols.len()).max().unwrap_or(0),
        reduced_rank,
        times: Times { compile: b.compile_s, setup: times.setup - b.compile_s, inner: b.inner_s,
                       total: clock.elapsed().as_secs_f64(), ..times },
    })
}
