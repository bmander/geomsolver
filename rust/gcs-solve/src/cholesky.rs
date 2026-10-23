//! Sparse Cholesky `PAPᵀ = LLᵀ` of a symmetric positive definite matrix, for the systems a mesh
//! makes (#510, P1): a stiffness `K u = f` over a tetrahedral mesh, factored once per pattern
//! and again per value.
//!
//! `sparse::Ata` is the normal equations of a sketch — trusses, chains, rings — ordered by
//! reverse Cuthill–McKee and factored a column at a time; on a 3D mesh a band order fills
//! `n^{5/3}` and a scalar column loop runs at a fraction of what the arithmetic allows.  Here
//! the order is approximate minimum degree (`amd`), postordered on its elimination tree (an
//! order of the same fill whose *supernodes* — runs of columns sharing one row structure below
//! their diagonal — are consecutive), and the numeric factorisation is supernodal and
//! left-looking: each supernode is gathered as one dense panel, updated by the descendants
//! whose rows reach it, and factored densely.  Three unknowns per node of a mesh make every
//! supernode at least three columns wide.
//!
//! [`Cholesky::analyse`] does what depends on the pattern only — order, elimination tree,
//! column counts, supernodes and their rows — and [`Cholesky::factor`] what depends on the
//! values, so a mesh moved with the parameters (its topology fixed within one solve) pays the
//! analysis once.  A pivot that is not positive to working precision ([`NotPositive`]) is an
//! error naming its column.
//!
//! The same analysis factors `PAPᵀ = LDLᵀ` without pivoting ([`Cholesky::factor_ldl`], #510 S3):
//! a pencil `K − μM` shifted into its spectrum is indefinite, and its pivots' signs are its
//! inertia (Sylvester), what an eigensolver counts its pairs by (`sparse::Pencil`).  Each
//! descendant's update is formed four columns at a time, each of its columns read once for the
//! four.

use crate::amd;

/// A symmetric matrix held by its lower triangle, compressed by column: column `j`'s rows are
/// `rowind[colptr[j]..colptr[j + 1]]`, each `>= j`, increasing, with their `values`.
#[derive(Clone, Debug)]
pub struct Symmetric {
    pub n: usize,
    pub colptr: Vec<usize>,
    pub rowind: Vec<usize>,
    pub values: Vec<f64>,
}

impl Symmetric {
    /// Assemble from `(row, col, value)` triplets, duplicates summed.  Only the lower triangle is
    /// read: an entry above the diagonal is ignored, so an assembly of whole element matrices
    /// may be passed as it stands.
    pub fn from_triplets(n: usize, t: &[(usize, usize, f64)]) -> Symmetric {
        let low = lower(t);
        let sum = Summed::of(n, &low);
        let values = sum.values(&low);
        Symmetric { values, ..sum.pattern }
    }

    /// `A x`, both triangles.
    pub fn mul_vec(&self, x: &[f64]) -> Vec<f64> {
        let mut y = vec![0.0; self.n];
        for j in 0..self.n {
            for q in self.colptr[j]..self.colptr[j + 1] {
                let (i, v) = (self.rowind[q], self.values[q]);
                y[i] += v * x[j];
                if i != j {
                    y[j] += v * x[i];
                }
            }
        }
        y
    }

    /// The pattern with both triangles, by column: what an ordering reads.
    fn full_pattern(&self) -> (Vec<usize>, Vec<usize>) {
        bucket(self, self.n, |i, j, _| [Some((j, i)), (i != j).then_some((i, j))])
    }
}

/// **How triplets are summed into a pattern** (`Symmetric::from_triplets`): the pattern (no
/// values), and the lower triangle's triplets in the order they are summed — `order[s]` the
/// `s`th summed, by its place among them, landing on entry `place[s]` of the pattern.  The order
/// is the sort's, which reads the triplets' places alone, so every assembly whose places come in
/// the same sequence is summed in it, bit for bit.
#[derive(Clone, Debug)]
struct Summed {
    pattern: Symmetric,
    order: Vec<u32>,
    place: Vec<u32>,
}

impl Summed {
    /// How the lower triangle's triplets `low` (`lower`) are summed.
    fn of(n: usize, low: &[(usize, usize, f64)]) -> Summed {
        // each triplet's place among them carried where its value was, so the sort is the one
        // a matrix of values makes: the same type, the same places
        let mut low: Vec<(usize, usize, f64)> = low.iter().enumerate()
            .map(|(q, &(i, j, _))| (i, j, f64::from_bits(q as u64))).collect();
        assert!(low.len() < u32::MAX as usize, "Symmetric::from_triplets: {} entries", low.len());
        for &(i, _, _) in &low {
            assert!(i < n, "Symmetric::from_triplets: row {i} of {n}");
        }
        low.sort_unstable_by_key(|&(i, j, _)| (j, i));
        let mut colptr = vec![0usize; n + 1];
        let mut rowind = Vec::with_capacity(low.len());
        let (mut order, mut place) = (Vec::with_capacity(low.len()), Vec::with_capacity(low.len()));
        for (q, &(i, j, at)) in low.iter().enumerate() {
            if q == 0 || low[q - 1].0 != i || low[q - 1].1 != j {
                rowind.push(i);
                colptr[j + 1] += 1;
            }
            order.push(at.to_bits() as u32);
            place.push((rowind.len() - 1) as u32);
        }
        for j in 0..n {
            colptr[j + 1] += colptr[j];
        }
        Summed { pattern: Symmetric { n, colptr, rowind, values: Vec::new() }, order, place }
    }

    /// The values of the lower triangle's triplets `low`, summed into the pattern in order: the
    /// first onto an entry put there, the rest added to it.
    fn values(&self, low: &[(usize, usize, f64)]) -> Vec<f64> {
        let mut values = vec![0.0; self.pattern.rowind.len()];
        for (s, (&q, &p)) in self.order.iter().zip(&self.place).enumerate() {
            let v = low[q as usize].2;
            if s > 0 && self.place[s - 1] == p {
                values[p as usize] += v;
            } else {
                values[p as usize] = v;
            }
        }
        values
    }

    /// Whether `low`'s places come in the sequence this was summed from: as many, and each
    /// where it is summed to.
    fn fits(&self, n: usize, low: &[(usize, usize, f64)]) -> bool {
        let a = &self.pattern;
        a.n == n && low.len() == self.order.len()
            && self.order.iter().zip(&self.place).all(|(&q, &p)| {
                let (i, j, _) = low[q as usize];
                let p = p as usize;
                j < n && (a.colptr[j]..a.colptr[j + 1]).contains(&p) && a.rowind[p] == i
            })
    }
}

/// The triplets of the lower triangle, the only ones a `Symmetric` reads.
fn lower(t: &[(usize, usize, f64)]) -> Vec<(usize, usize, f64)> {
    t.iter().copied().filter(|&(i, j, _)| i >= j).collect()
}

/// How many patterns a thread remembers the analysis of (`assembled`): a moved region's two —
/// its nodes' harmonic map and its stiffness — for each of a couple of rows.
const KNOWN: usize = 4;

thread_local! {
    /// The patterns analysed last on this thread, the latest first: each its order, how its
    /// triplets are summed, and its analysis (no panels).
    static PATTERNS: std::cell::RefCell<Vec<(Ordering, Summed, Cholesky)>> =
        const { std::cell::RefCell::new(Vec::new()) };
}

/// **A pattern analysed once** (#510): `t` assembled (`Symmetric::from_triplets`) with the
/// analysis of its pattern under `ordering`, nothing factored.  A mesh moved with the drawing's
/// numbers is assembled again at every evaluation of its row — a kernel keeps nothing between
/// calls — its topology fixed, and sorting its triplets and analysing its pattern are most of
/// what an evaluation costs; so both are remembered on this thread for the last `KNOWN`
/// patterns, for the next assembly whose triplets come at the same places in the same sequence.
/// Its values are summed in the order `from_triplets` sums them, so what is remembered changes
/// no bit of an answer.
pub fn assembled(n: usize, t: &[(usize, usize, f64)], ordering: Ordering) -> (Symmetric, Cholesky) {
    let low = lower(t);
    PATTERNS.with(|known| {
        let mut known = known.borrow_mut();
        let at = known.iter().position(|(o, sum, _)| *o == ordering && sum.fits(n, &low));
        let (o, sum, kept) = match at {
            Some(at) => known.remove(at),
            None => {
                let sum = Summed::of(n, &low);
                let mut kept = Cholesky::analyse(&sum.pattern, ordering);
                kept.lx = Vec::new();
                (ordering, sum, kept)
            }
        };
        let a = Symmetric { values: sum.values(&low), ..sum.pattern.clone() };
        let mut c = kept.clone();
        c.lx = vec![0.0; c.xp[c.supernodes()]];
        known.insert(0, (o, sum, kept));
        known.truncate(KNOWN);
        (a, c)
    })
}

/// The fill-reducing order the analysis starts from, before its postorder.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Ordering {
    /// Approximate minimum degree: the order for a mesh.
    Amd,
    /// Reverse Cuthill–McKee, the band order `sparse::Ata` takes on a pattern with no dense
    /// column, kept to measure AMD against.
    Rcm,
    /// The matrix's own order.
    Natural,
}

/// A pivot not positive to working precision: the factor stopped at `column` (of A, in its own
/// numbering), whose pivot `pivot` is within the rounding of its own computation.
///
/// Column j's pivot is `a_jj − Σ L_jk²`, a sum of `r_j` terms (row j's count in L) whose
/// magnitudes add up to `a_jj` itself; rounding perturbs it by up to about `r_j · ε · a_jj`
/// (the backward error of Cholesky, `|ΔA| ≤ γ |L||Lᵀ|`).  A pivot no larger than that has lost
/// every digit to cancellation and is zero for all the arithmetic can tell: the matrix is
/// singular there, not positive definite.  That is all it promises.  A free stiffness's
/// rigid-body modes fall under it in AMD's order in every block measured, but in a band order
/// they can round to just above it, and a slender supported body's pivots fall to 1e-9 of their
/// diagonal honestly, so no ratio tells the two apart: naming a freedom is the diagnosis's
/// (#510, P7), from the supports, not the factor's.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NotPositive {
    pub column: usize,
    pub pivot: f64,
}

const NONE: usize = usize::MAX;

/// How many columns of a descendant's update are formed together, each of its columns read once
/// for them all.
const BLOCK: usize = 4;

/// The analysis of one pattern and the factor of the last values given it.
#[derive(Clone, Debug)]
pub struct Cholesky {
    n: usize,
    /// `perm[k]` is the column of A eliminated k-th.
    perm: Vec<usize>,
    /// A's entries gathered into the panels, supernode by supernode: those of supernode `s` are
    /// `gather[gp[s]..gp[s + 1]]`, each `(q, at)` putting `values[q]` of A at `lx[at]`.
    gp: Vec<usize>,
    gather: Vec<(usize, usize)>,
    /// Supernode `s` is the columns `first[s]..first[s + 1]`; `snode[j]` the one holding `j`.
    first: Vec<usize>,
    snode: Vec<usize>,
    /// Its rows, `rows[rp[s]..rp[s + 1]]`: its own columns, then those below in increasing order.
    rp: Vec<usize>,
    rows: Vec<usize>,
    /// Its panel, column-major, `rows` long, at `lx[xp[s]..xp[s + 1]]`.
    xp: Vec<usize>,
    lx: Vec<f64>,
    /// Each row's count in L, its diagonal counted, in the numbering of PAPᵀ: the terms its
    /// pivot sums, so the rounding it carries.
    rcount: Vec<usize>,
    /// Entries of L, its diagonal counted.
    nnz_l: usize,
    factored: bool,
    /// Whether the last factor is `LDLᵀ`: each panel's diagonal D's, below it L's, its own
    /// diagonal a unit not stored.
    ldl: bool,
}

/// A pivot of `LDLᵀ` that is zero, or not a number: the factor stopped at `column` (of A).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ZeroPivot {
    pub column: usize,
}

impl Cholesky {
    /// Order, elimination tree, column counts and supernodes of `a`'s pattern; nothing of its
    /// values is read.
    pub fn analyse(a: &Symmetric, ordering: Ordering) -> Cholesky {
        let n = a.n;
        let (fp, fi) = a.full_pattern();
        let order = match ordering {
            Ordering::Amd => amd::order(n, &fp, &fi),
            Ordering::Rcm => {
                let ap: Vec<i32> = fp.iter().map(|&p| p as i32).collect();
                let ai: Vec<i32> = fi.iter().map(|&i| i as i32).collect();
                crate::sparse::rcm(n, &ap, &ai).0.iter().map(|&i| i as usize).collect()
            }
            Ordering::Natural => (0..n).collect(),
        };
        // the order's elimination tree, postordered: the same fill and the same tree relabelled,
        // its supernodes consecutive
        let (rp0, ri0) = lower_rows(a, &inverse(&order));
        let parent0 = etree(n, &rp0, &ri0);
        let post = postorder(&parent0);
        let ipost = inverse(&post);
        let perm: Vec<usize> = post.iter().map(|&k| order[k]).collect();
        let relabel = |k: usize| if parent0[k] == NONE { NONE } else { ipost[parent0[k]] };
        let parent: Vec<usize> = post.iter().map(|&k| relabel(k)).collect();
        let iperm = inverse(&perm);
        let (rowp, rowi) = lower_rows(a, &iperm);

        // column counts, by each row's subtree: row k of L reaches from the columns of row k of
        // C up the tree to k
        let mut count = vec![1usize; n];
        let mut rcount = vec![1usize; n];
        let mut flag = vec![NONE; n];
        row_subtrees(&rowp, &rowi, &parent, &mut flag, |j, k| {
            count[j] += 1;
            rcount[k] += 1;
        });

        // supernodes: j + 1 joins j's when it is j's parent and holds j's rows but j
        let mut first = Vec::new();
        let mut snode = vec![0usize; n];
        for j in 0..n {
            if j == 0 || !(parent[j - 1] == j && count[j - 1] == count[j] + 1) {
                first.push(j);
            }
            snode[j] = first.len() - 1;
        }
        first.push(n);
        let ns = first.len() - 1;

        // each supernode's rows: its columns, then those of its first column below them
        let mut rp = vec![0usize; ns + 1];
        for s in 0..ns {
            rp[s + 1] = rp[s] + count[first[s]];
        }
        let mut rows = vec![0usize; rp[ns]];
        let mut at = vec![0usize; ns];
        for s in 0..ns {
            for (c, j) in (first[s]..first[s + 1]).enumerate() {
                rows[rp[s] + c] = j;
            }
            at[s] = rp[s] + first[s + 1] - first[s];
        }
        let mut sflag = vec![NONE; ns];
        flag.fill(NONE);
        row_subtrees(&rowp, &rowi, &parent, &mut flag, |j, k| {
            let s = snode[j];
            if k >= first[s + 1] && sflag[s] != k {
                sflag[s] = k;
                rows[at[s]] = k;
                at[s] += 1;
            }
        });
        debug_assert!((0..ns).all(|s| at[s] == rp[s + 1]));

        let mut xp = vec![0usize; ns + 1];
        for s in 0..ns {
            xp[s + 1] = xp[s] + (rp[s + 1] - rp[s]) * (first[s + 1] - first[s]);
        }

        // where each entry of A lands: the lower triangle of PAPᵀ by column, so by supernode
        let (cp, ce) = bucket(a, n, |i, j, q| {
            let (pi, pj) = (iperm[i], iperm[j]);
            [Some((pi.min(pj), (pi.max(pj), q))), None]
        });
        let mut map = vec![0usize; n];
        let mut gp = vec![0usize; ns + 1];
        let mut gather = Vec::with_capacity(ce.len());
        for s in 0..ns {
            let (f, nrow) = (first[s], rp[s + 1] - rp[s]);
            for (p, &r) in rows[rp[s]..rp[s + 1]].iter().enumerate() {
                map[r] = p;
            }
            for j in f..first[s + 1] {
                for &(r, q) in &ce[cp[j]..cp[j + 1]] {
                    gather.push((q, xp[s] + (j - f) * nrow + map[r]));
                }
            }
            gp[s + 1] = gather.len();
        }

        Cholesky {
            n,
            perm,
            gp,
            gather,
            first,
            snode,
            rp,
            rows,
            lx: vec![0.0; xp[ns]],
            xp,
            rcount,
            nnz_l: count.iter().sum(),
            factored: false,
            ldl: false,
        }
    }

    /// Entries of L, its diagonal counted: the fill of the order chosen.
    pub fn factor_nnz(&self) -> usize {
        self.nnz_l
    }

    pub fn supernodes(&self) -> usize {
        self.first.len() - 1
    }

    /// `perm[k]` is the column of A that is column k of PAPᵀ.
    pub fn perm(&self) -> &[usize] {
        &self.perm
    }

    fn panel(&self, s: usize) -> (usize, usize, &[usize], usize) {
        self.shape().panel(s)
    }

    fn shape(&self) -> Shape<'_> {
        Shape { first: &self.first, rp: &self.rp, rows: &self.rows, xp: &self.xp }
    }

    /// Factor `a`, which must have the pattern analysed (its order of entries too: it is
    /// gathered by position).  On a pivot not positive the factor is left unusable and the
    /// column named.
    pub fn factor(&mut self, a: &Symmetric) -> Result<(), NotPositive> {
        self.numeric(a, false).map(|_| ()).map_err(|(column, pivot)| NotPositive { column, pivot })
    }

    /// **Factor `a` as `LDLᵀ`, without pivoting** (#510 S3): `a` symmetric, positive definite or
    /// not, on the pattern analysed.  Its count of negative pivots — the number of `a`'s
    /// eigenvalues below zero, Sylvester's inertia — or the column whose pivot is zero.
    pub fn factor_ldl(&mut self, a: &Symmetric) -> Result<usize, ZeroPivot> {
        self.numeric(a, true).map_err(|(column, _)| ZeroPivot { column })
    }

    /// The numeric factor, `LLᵀ` or, where `ldl`, `LDLᵀ`: the count of negative pivots, or the
    /// column (of A) and pivot it stopped at.
    fn numeric(&mut self, a: &Symmetric, ldl: bool) -> Result<usize, (usize, f64)> {
        assert!(
            a.n == self.n && a.values.len() == self.gather.len(),
            "Cholesky::factor: not the pattern analysed"
        );
        self.factored = false;
        self.ldl = ldl;
        let ns = self.supernodes();
        let shape = Shape { first: &self.first, rp: &self.rp, rows: &self.rows, xp: &self.xp };
        let lx = &mut self.lx;
        let mut map = vec![0usize; self.n];
        let mut rel: Vec<usize> = Vec::new();
        let mut head = vec![NONE; ns];
        let mut next = vec![NONE; ns];
        // the position in each supernode's rows of the first not yet applied to an ancestor
        let mut ptr = vec![0usize; ns];
        let mut acc: Vec<[f64; BLOCK]> = Vec::new();
        let mut diag: Vec<f64> = Vec::new();
        let mut negative = 0;
        for s in 0..ns {
            let (f, ncol, rows, base) = shape.panel(s);
            let (l, nrow) = (f + ncol, rows.len());
            for (p, &r) in rows.iter().enumerate() {
                map[r] = p;
            }
            // gather A's columns, the panel zeroed as it is reached
            lx[base..base + ncol * nrow].fill(0.0);
            for &(q, at) in &self.gather[self.gp[s]..self.gp[s + 1]] {
                lx[at] = a.values[q];
            }
            diag.clear();
            diag.extend((0..ncol).map(|c| lx[base + c * nrow + c]));
            // the descendants whose rows reach s, each `L_d[p0.., :] D_d L_d[p0..p0 + m1, :]ᵀ`
            // (D a unit in `LLᵀ`), `BLOCK` columns of the update at a time
            let mut d = head[s];
            head[s] = NONE;
            while d != NONE {
                let dn = next[d];
                let (_, dncol, drows, dbase) = shape.panel(d);
                let dnrow = drows.len();
                let p0 = ptr[d];
                let m2 = dnrow - p0;
                let m1 = drows[p0..].iter().take_while(|&&r| r < l).count();
                rel.clear();
                rel.extend(drows[p0..].iter().map(|&r| map[r]));
                let mut j = 0;
                while j < m1 {
                    let w = (m1 - j).min(BLOCK);
                    acc.clear();
                    acc.resize(m2 - j, [0.0; BLOCK]);
                    for k in 0..dncol {
                        let colk = &lx[dbase + k * dnrow + p0 + j..dbase + (k + 1) * dnrow];
                        let dk = if ldl { lx[dbase + k * dnrow + k] } else { 1.0 };
                        let mut lk = [0.0; BLOCK];
                        for t in 0..w {
                            lk[t] = colk[t] * dk;
                        }
                        if lk.iter().all(|&v| v == 0.0) {
                            continue;
                        }
                        for (a, &v) in acc.iter_mut().zip(colk) {
                            for t in 0..BLOCK {
                                a[t] += v * lk[t];
                            }
                        }
                    }
                    for t in 0..w {
                        let col = base + (drows[p0 + j + t] - f) * nrow;
                        for (a, &r) in acc[t..].iter().zip(&rel[j + t..]) {
                            lx[col + r] -= a[t];
                        }
                    }
                    j += w;
                }
                ptr[d] = p0 + m1;
                if ptr[d] < dnrow {
                    let t = self.snode[drows[ptr[d]]];
                    next[d] = head[t];
                    head[t] = d;
                }
                d = dn;
            }
            // the dense panel
            let panel = &mut lx[base..base + ncol * nrow];
            for c in 0..ncol {
                let (done, rest) = panel.split_at_mut(c * nrow);
                let colc = &mut rest[c..nrow];
                for k in 0..c {
                    let colk = &done[k * nrow + c..(k + 1) * nrow];
                    let lck = if ldl { colk[0] * done[k * nrow + k] } else { colk[0] };
                    if lck != 0.0 {
                        for (x, &v) in colc.iter_mut().zip(colk) {
                            *x -= v * lck;
                        }
                    }
                }
                let pivot = colc[0];
                if ldl {
                    if pivot == 0.0 || !pivot.is_finite() {
                        return Err((self.perm[f + c], pivot));
                    }
                    negative += (pivot < 0.0) as usize;
                    for x in &mut colc[1..] {
                        *x /= pivot;
                    }
                    continue;
                }
                let rounding = self.rcount[f + c] as f64 * f64::EPSILON * diag[c];
                if !(diag[c] > 0.0 && pivot > rounding) || !pivot.is_finite() {
                    return Err((self.perm[f + c], pivot));
                }
                let root = pivot.sqrt();
                colc[0] = root;
                for x in &mut colc[1..] {
                    *x /= root;
                }
            }
            if nrow > ncol {
                ptr[s] = ncol;
                let t = self.snode[rows[ncol]];
                next[s] = head[t];
                head[t] = s;
            }
        }
        self.factored = true;
        Ok(negative)
    }

    /// Solve `A x = b` in place with the last factor, `LLᵀ` or `LDLᵀ`.
    pub fn solve(&self, b: &mut [f64]) {
        assert!(self.factored, "Cholesky::solve: no factor");
        let mut y: Vec<f64> = self.perm.iter().map(|&i| b[i]).collect();
        // L's diagonal: its own in LLᵀ, a unit in LDLᵀ (where the panel's diagonal holds D)
        let unit = |d: f64| if self.ldl { 1.0 } else { d };
        for s in 0..self.supernodes() {
            let (f, ncol, rows, base) = self.panel(s);
            let nrow = rows.len();
            for c in 0..ncol {
                let col = &self.lx[base + c * nrow..base + (c + 1) * nrow];
                y[f + c] /= unit(col[c]);
                let yj = y[f + c];
                for r in c + 1..nrow {
                    y[rows[r]] -= col[r] * yj;
                }
            }
        }
        if self.ldl {
            for s in 0..self.supernodes() {
                let (f, ncol, rows, base) = self.panel(s);
                for c in 0..ncol {
                    y[f + c] /= self.lx[base + c * rows.len() + c];
                }
            }
        }
        for s in (0..self.supernodes()).rev() {
            let (f, ncol, rows, base) = self.panel(s);
            let nrow = rows.len();
            for c in (0..ncol).rev() {
                let col = &self.lx[base + c * nrow..base + (c + 1) * nrow];
                let mut sum = y[f + c];
                for r in c + 1..nrow {
                    sum -= col[r] * y[rows[r]];
                }
                y[f + c] = sum / unit(col[c]);
            }
        }
        for (k, &i) in self.perm.iter().enumerate() {
            b[i] = y[k];
        }
    }

    /// The entries of L as `(row, col, value)`, `row >= col`, in the numbering of PAPᵀ.
    pub fn l(&self) -> Vec<(usize, usize, f64)> {
        assert!(self.factored, "Cholesky::l: no factor");
        let mut out = Vec::with_capacity(self.nnz_l);
        for s in 0..self.supernodes() {
            let (f, ncol, rows, base) = self.panel(s);
            let nrow = rows.len();
            for c in 0..ncol {
                for r in c..nrow {
                    out.push((rows[r], f + c, self.lx[base + c * nrow + r]));
                }
            }
        }
        out
    }
}

/// The supernodes' layout, borrowed apart from the panels' values so `factor` can write them.
struct Shape<'a> {
    first: &'a [usize],
    rp: &'a [usize],
    rows: &'a [usize],
    xp: &'a [usize],
}

impl<'a> Shape<'a> {
    /// Supernode `s` as (its first column, its width, its rows, where its panel starts in `lx`).
    fn panel(&self, s: usize) -> (usize, usize, &'a [usize], usize) {
        let f = self.first[s];
        (f, self.first[s + 1] - f, &self.rows[self.rp[s]..self.rp[s + 1]], self.xp[s])
    }
}

/// One counting sort over A's lower triangle: each entry `q` (row `i`, column `j`) emits up to
/// two `(bucket, payload)`s, and the buckets come out in order, `out[ptr[b]..ptr[b + 1]]`, each
/// in A's order.  The orderings' pattern, the rows of PAPᵀ and the gather are all one.
fn bucket<T: Copy + Default>(
    a: &Symmetric,
    nb: usize,
    emit: impl Fn(usize, usize, usize) -> [Option<(usize, T)>; 2],
) -> (Vec<usize>, Vec<T>) {
    let mut ptr = vec![0usize; nb + 1];
    for j in 0..a.n {
        for q in a.colptr[j]..a.colptr[j + 1] {
            for (b, _) in emit(a.rowind[q], j, q).into_iter().flatten() {
                ptr[b + 1] += 1;
            }
        }
    }
    for b in 0..nb {
        ptr[b + 1] += ptr[b];
    }
    let mut at = ptr.clone();
    let mut out = vec![T::default(); ptr[nb]];
    for j in 0..a.n {
        for q in a.colptr[j]..a.colptr[j + 1] {
            for (b, t) in emit(a.rowind[q], j, q).into_iter().flatten() {
                out[at[b]] = t;
                at[b] += 1;
            }
        }
    }
    (ptr, out)
}

fn inverse(perm: &[usize]) -> Vec<usize> {
    let mut inv = vec![0usize; perm.len()];
    for (k, &i) in perm.iter().enumerate() {
        inv[i] = k;
    }
    inv
}

/// The rows of the lower triangle of PAPᵀ (`iperm` the inverse order), each as the columns
/// before its diagonal: row k's are `rowi[rowp[k]..rowp[k + 1]]`.
fn lower_rows(a: &Symmetric, iperm: &[usize]) -> (Vec<usize>, Vec<usize>) {
    bucket(a, a.n, |i, j, _| {
        let (pi, pj) = (iperm[i], iperm[j]);
        [(pi != pj).then_some((pi.max(pj), pi.min(pj))), None]
    })
}

/// Each row's subtree of the elimination tree: row k of L holds column j for every `j` on the
/// paths up from the columns of row k of the matrix to k.  `visit(j, k)` once for each.  `flag`
/// is n long and holds no row's mark on entry.
fn row_subtrees(
    rowp: &[usize],
    rowi: &[usize],
    parent: &[usize],
    flag: &mut [usize],
    mut visit: impl FnMut(usize, usize),
) {
    for k in 0..parent.len() {
        flag[k] = k;
        for &j0 in &rowi[rowp[k]..rowp[k + 1]] {
            let mut j = j0;
            while flag[j] != k {
                visit(j, k);
                flag[j] = k;
                j = parent[j];
            }
        }
    }
}

/// The elimination tree from the rows below the diagonal (Liu, with path compression):
/// `parent[j]` is the first row below j of L's column j, `NONE` at a root.
fn etree(n: usize, rowp: &[usize], rowi: &[usize]) -> Vec<usize> {
    let mut parent = vec![NONE; n];
    let mut ancestor = vec![NONE; n];
    for k in 0..n {
        for &j0 in &rowi[rowp[k]..rowp[k + 1]] {
            let mut j = j0;
            while j != NONE && j < k {
                let up = ancestor[j];
                ancestor[j] = k;
                if up == NONE {
                    parent[j] = k;
                }
                j = up;
            }
        }
    }
    parent
}

/// A postorder of the forest `parent`: `post[k]` is the node numbered k, children before their
/// parent and each subtree contiguous.
fn postorder(parent: &[usize]) -> Vec<usize> {
    let n = parent.len();
    let mut head = vec![NONE; n];
    let mut next = vec![NONE; n];
    for j in (0..n).rev() {
        if parent[j] != NONE {
            next[j] = head[parent[j]];
            head[parent[j]] = j;
        }
    }
    let mut post = Vec::with_capacity(n);
    let mut stack = Vec::new();
    for root in 0..n {
        if parent[root] != NONE {
            continue;
        }
        stack.push(root);
        while let Some(&p) = stack.last() {
            let i = head[p];
            if i == NONE {
                stack.pop();
                post.push(p);
            } else {
                head[p] = next[i];
                stack.push(i);
            }
        }
    }
    post
}
