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
        let mut low: Vec<(usize, usize, f64)> =
            t.iter().copied().filter(|&(i, j, _)| i >= j).collect();
        for &(i, _, _) in &low {
            assert!(i < n, "Symmetric::from_triplets: row {i} of {n}");
        }
        low.sort_by_key(|&(i, j, _)| (j, i));
        let mut colptr = vec![0usize; n + 1];
        let mut rowind = Vec::with_capacity(low.len());
        let mut values: Vec<f64> = Vec::with_capacity(low.len());
        for (q, &(i, j, v)) in low.iter().enumerate() {
            if q > 0 && low[q - 1].0 == i && low[q - 1].1 == j {
                *values.last_mut().unwrap() += v;
            } else {
                rowind.push(i);
                values.push(v);
                colptr[j + 1] += 1;
            }
        }
        for j in 0..n {
            colptr[j + 1] += colptr[j];
        }
        Symmetric { n, colptr, rowind, values }
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
        let n = self.n;
        let mut cnt = vec![0usize; n + 1];
        for j in 0..n {
            for q in self.colptr[j]..self.colptr[j + 1] {
                let i = self.rowind[q];
                cnt[j + 1] += 1;
                if i != j {
                    cnt[i + 1] += 1;
                }
            }
        }
        for j in 0..n {
            cnt[j + 1] += cnt[j];
        }
        let mut at = cnt.clone();
        let mut ai = vec![0usize; cnt[n]];
        for j in 0..n {
            for q in self.colptr[j]..self.colptr[j + 1] {
                let i = self.rowind[q];
                ai[at[j]] = i;
                at[j] += 1;
                if i != j {
                    ai[at[i]] = j;
                    at[i] += 1;
                }
            }
        }
        (cnt, ai)
    }
}

/// The fill-reducing order the analysis starts from, before its postorder.
#[derive(Clone, Debug)]
pub enum Ordering {
    /// Approximate minimum degree: the order for a mesh.
    Amd,
    /// Reverse Cuthill–McKee, `sparse::Ata`'s order, kept to measure AMD against.
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

/// The analysis of one pattern and the factor of the last values given it.
#[derive(Clone, Debug)]
pub struct Cholesky {
    n: usize,
    /// `perm[k]` is the column of A eliminated k-th; `iperm` its inverse.
    perm: Vec<usize>,
    iperm: Vec<usize>,
    /// The pattern analysed, to hold `factor` to it.
    colptr: Vec<usize>,
    rowind: Vec<usize>,
    /// The lower triangle of PAPᵀ by column: rows `ci`, each taken from `values[cs]` of A.
    cp: Vec<usize>,
    ci: Vec<usize>,
    cs: Vec<usize>,
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
    /// Entries of L, its diagonal counted, and the multiply-adds of its factor, `Σ c_j²` over
    /// the column counts.
    nnz_l: usize,
    flops: f64,
    factored: bool,
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
        // the order's elimination tree, postordered: the same fill, its supernodes consecutive
        let (rp0, ri0) = upper_rows(a, &inverse(&order));
        let parent0 = etree(n, &rp0, &ri0);
        let post = postorder(&parent0);
        let perm: Vec<usize> = post.iter().map(|&k| order[k]).collect();
        let iperm = inverse(&perm);
        let (rowp, rowi) = upper_rows(a, &iperm);
        let parent = etree(n, &rowp, &rowi);

        // column counts, by each row's subtree: row k of L reaches from the columns of row k of
        // C up the tree to k
        let mut count = vec![1usize; n];
        let mut rcount = vec![1usize; n];
        let mut flag = vec![NONE; n];
        for k in 0..n {
            flag[k] = k;
            for &j0 in &rowi[rowp[k]..rowp[k + 1]] {
                let mut j = j0;
                while flag[j] != k {
                    count[j] += 1;
                    rcount[k] += 1;
                    flag[j] = k;
                    j = parent[j];
                }
            }
        }

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
        flag.iter_mut().for_each(|f| *f = NONE);
        for k in 0..n {
            flag[k] = k;
            for &j0 in &rowi[rowp[k]..rowp[k + 1]] {
                let mut j = j0;
                while flag[j] != k {
                    let s = snode[j];
                    if k >= first[s + 1] && sflag[s] != k {
                        sflag[s] = k;
                        rows[at[s]] = k;
                        at[s] += 1;
                    }
                    flag[j] = k;
                    j = parent[j];
                }
            }
        }
        debug_assert!((0..ns).all(|s| at[s] == rp[s + 1]));

        let mut xp = vec![0usize; ns + 1];
        for s in 0..ns {
            xp[s + 1] = xp[s] + (rp[s + 1] - rp[s]) * (first[s + 1] - first[s]);
        }

        // the lower triangle of PAPᵀ, each entry pointing at its value in A
        let mut cp = vec![0usize; n + 1];
        for j in 0..n {
            for q in a.colptr[j]..a.colptr[j + 1] {
                let (pi, pj) = (iperm[a.rowind[q]], iperm[j]);
                cp[pi.min(pj) + 1] += 1;
            }
        }
        for j in 0..n {
            cp[j + 1] += cp[j];
        }
        let mut cat = cp.clone();
        let mut entries = vec![(0usize, 0usize); cp[n]];
        for j in 0..n {
            for q in a.colptr[j]..a.colptr[j + 1] {
                let (pi, pj) = (iperm[a.rowind[q]], iperm[j]);
                let col = pi.min(pj);
                entries[cat[col]] = (pi.max(pj), q);
                cat[col] += 1;
            }
        }
        for j in 0..n {
            entries[cp[j]..cp[j + 1]].sort_unstable();
        }
        let (ci, cs) = entries.into_iter().unzip();

        Cholesky {
            n,
            perm,
            iperm,
            colptr: a.colptr.clone(),
            rowind: a.rowind.clone(),
            cp,
            ci,
            cs,
            first,
            snode,
            rp,
            rows,
            lx: vec![0.0; xp[ns]],
            xp,
            rcount,
            nnz_l: count.iter().sum(),
            flops: count.iter().map(|&c| (c * c) as f64).sum(),
            factored: false,
        }
    }

    pub fn n(&self) -> usize {
        self.n
    }

    /// Entries of L, its diagonal counted: the fill of the order chosen.
    pub fn factor_nnz(&self) -> usize {
        self.nnz_l
    }

    /// The multiply-adds the factor costs, `Σ c_j²` over its column counts: what an order buys.
    pub fn flops(&self) -> f64 {
        self.flops
    }

    pub fn supernodes(&self) -> usize {
        self.first.len() - 1
    }

    /// `perm[k]` is the column of A that is column k of PAPᵀ.
    pub fn perm(&self) -> &[usize] {
        &self.perm
    }

    /// Factor `a`, which must have the pattern analysed.  On a pivot not positive the factor is
    /// left unusable and the column named.
    pub fn factor(&mut self, a: &Symmetric) -> Result<(), NotPositive> {
        assert!(
            a.n == self.n && a.colptr == self.colptr && a.rowind == self.rowind,
            "Cholesky::factor: not the pattern analysed"
        );
        self.factored = false;
        let ns = self.first.len() - 1;
        let mut map = vec![0usize; self.n];
        let mut head = vec![NONE; ns];
        let mut next = vec![NONE; ns];
        // the position in each supernode's rows of the first not yet applied to an ancestor
        let mut ptr = vec![0usize; ns];
        let mut tmp: Vec<f64> = Vec::new();
        let mut diag: Vec<f64> = Vec::new();
        self.lx.iter_mut().for_each(|v| *v = 0.0);
        for s in 0..ns {
            let (f, l) = (self.first[s], self.first[s + 1]);
            let ncol = l - f;
            let rows = &self.rows[self.rp[s]..self.rp[s + 1]];
            let nrow = rows.len();
            let base = self.xp[s];
            for (p, &r) in rows.iter().enumerate() {
                map[r] = p;
            }
            // gather A's columns
            diag.clear();
            diag.resize(ncol, 0.0);
            for c in 0..ncol {
                let col = base + c * nrow;
                for q in self.cp[f + c]..self.cp[f + c + 1] {
                    let v = a.values[self.cs[q]];
                    self.lx[col + map[self.ci[q]]] = v;
                    if self.ci[q] == f + c {
                        diag[c] = v;
                    }
                }
            }
            // the descendants whose rows reach s: `L_d[p0.., :] · L_d[p0..p0 + m1, :]ᵀ` each
            let mut d = head[s];
            head[s] = NONE;
            while d != NONE {
                let dn = next[d];
                let drows = &self.rows[self.rp[d]..self.rp[d + 1]];
                let dnrow = drows.len();
                let dncol = self.first[d + 1] - self.first[d];
                let p0 = ptr[d];
                let m2 = dnrow - p0;
                let m1 = drows[p0..].iter().take_while(|&&r| r < l).count();
                tmp.clear();
                tmp.resize(m1 * m2, 0.0);
                let dbase = self.xp[d];
                for k in 0..dncol {
                    let colk = &self.lx[dbase + k * dnrow + p0..dbase + (k + 1) * dnrow];
                    for j in 0..m1 {
                        let ljk = colk[j];
                        if ljk == 0.0 {
                            continue;
                        }
                        let t = &mut tmp[j * m2..(j + 1) * m2];
                        for i in j..m2 {
                            t[i] += colk[i] * ljk;
                        }
                    }
                }
                for j in 0..m1 {
                    let col = base + (drows[p0 + j] - f) * nrow;
                    for i in j..m2 {
                        self.lx[col + map[drows[p0 + i]]] -= tmp[j * m2 + i];
                    }
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
            let panel = &mut self.lx[base..base + ncol * nrow];
            for c in 0..ncol {
                let (done, rest) = panel.split_at_mut(c * nrow);
                let colc = &mut rest[..nrow];
                for k in 0..c {
                    let colk = &done[k * nrow..(k + 1) * nrow];
                    let lck = colk[c];
                    if lck == 0.0 {
                        continue;
                    }
                    for r in c..nrow {
                        colc[r] -= colk[r] * lck;
                    }
                }
                let pivot = colc[c];
                let rounding = self.rcount[f + c] as f64 * f64::EPSILON * diag[c];
                if !(diag[c] > 0.0 && pivot > rounding) || !pivot.is_finite() {
                    return Err(NotPositive { column: self.perm[f + c], pivot });
                }
                let root = pivot.sqrt();
                colc[c] = root;
                for r in c + 1..nrow {
                    colc[r] /= root;
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
        Ok(())
    }

    /// Solve `A x = b` in place with the last factor.
    pub fn solve(&self, b: &mut [f64]) {
        assert!(self.factored, "Cholesky::solve: no factor");
        let n = self.n;
        let mut y: Vec<f64> = (0..n).map(|k| b[self.perm[k]]).collect();
        let ns = self.first.len() - 1;
        for s in 0..ns {
            let (f, ncol) = (self.first[s], self.first[s + 1] - self.first[s]);
            let rows = &self.rows[self.rp[s]..self.rp[s + 1]];
            let nrow = rows.len();
            for c in 0..ncol {
                let col = &self.lx[self.xp[s] + c * nrow..self.xp[s] + (c + 1) * nrow];
                y[f + c] /= col[c];
                let yj = y[f + c];
                for r in c + 1..nrow {
                    y[rows[r]] -= col[r] * yj;
                }
            }
        }
        for s in (0..ns).rev() {
            let (f, ncol) = (self.first[s], self.first[s + 1] - self.first[s]);
            let rows = &self.rows[self.rp[s]..self.rp[s + 1]];
            let nrow = rows.len();
            for c in (0..ncol).rev() {
                let col = &self.lx[self.xp[s] + c * nrow..self.xp[s] + (c + 1) * nrow];
                let mut sum = y[f + c];
                for r in c + 1..nrow {
                    sum -= col[r] * y[rows[r]];
                }
                y[f + c] = sum / col[c];
            }
        }
        for k in 0..n {
            b[self.perm[k]] = y[k];
        }
    }

    /// The entries of L as `(row, col, value)`, `row >= col`, in the numbering of PAPᵀ.
    pub fn l(&self) -> Vec<(usize, usize, f64)> {
        assert!(self.factored, "Cholesky::l: no factor");
        let mut out = Vec::with_capacity(self.nnz_l);
        for s in 0..self.first.len() - 1 {
            let f = self.first[s];
            let rows = &self.rows[self.rp[s]..self.rp[s + 1]];
            let nrow = rows.len();
            for c in 0..self.first[s + 1] - f {
                for r in c..nrow {
                    out.push((rows[r], f + c, self.lx[self.xp[s] + c * nrow + r]));
                }
            }
        }
        out
    }

    /// The row of A that is row `k` of PAPᵀ, inverse to `perm`.
    pub fn iperm(&self) -> &[usize] {
        &self.iperm
    }
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
fn upper_rows(a: &Symmetric, iperm: &[usize]) -> (Vec<usize>, Vec<usize>) {
    let n = a.n;
    let mut rowp = vec![0usize; n + 1];
    for j in 0..n {
        for q in a.colptr[j]..a.colptr[j + 1] {
            let (pi, pj) = (iperm[a.rowind[q]], iperm[j]);
            if pi != pj {
                rowp[pi.max(pj) + 1] += 1;
            }
        }
    }
    for k in 0..n {
        rowp[k + 1] += rowp[k];
    }
    let mut at = rowp.clone();
    let mut rowi = vec![0usize; rowp[n]];
    for j in 0..n {
        for q in a.colptr[j]..a.colptr[j + 1] {
            let (pi, pj) = (iperm[a.rowind[q]], iperm[j]);
            if pi != pj {
                let k = pi.max(pj);
                rowi[at[k]] = pi.min(pj);
                at[k] += 1;
            }
        }
    }
    (rowp, rowi)
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
