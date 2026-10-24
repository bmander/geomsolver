//! Sparse normal equations for the large-sketch path: JᵀJ assembled from the fixed CSR structure
//! of the Jacobian, ordered by reverse Cuthill–McKee with any dense column last, and factored by
//! an up-looking LDLᵀ (Davis).  Regularizing the diagonal keeps rank-deficient
//! (under-constrained) systems solvable, which is the normal case while editing.
//!
//! A mesh's symmetric matrix (`cholesky::Symmetric`, its lower triangle) — a stiffness, a mass —
//! is factored by P1's supernodal LDLᵀ under minimum degree (`Ldl`, #510 S3), and so is a pencil
//! `L − μR` analysed once and factored at each μ the eigensolver asks for (`Pencil`, `eigen.rs`,
//! #510 P2), whose pivots' signs count the eigenvalues below μ.

use std::borrow::Cow;

use crate::cholesky::{analysed, Cholesky, Ordering, Symmetric};

/// Reverse Cuthill–McKee on the pattern of A: short, and the sketch graphs this solver sees
/// (trusses, chains, rings) are close to banded once relabelled this way.
pub(crate) fn rcm(n: usize, ap: &[i32], ai: &[i32]) -> (Vec<i32>, Vec<i32>) {
    let mut deg = vec![0i32; n];
    let mut seen = vec![false; n];
    let mut queue: Vec<i32> = Vec::with_capacity(n);
    for i in 0..n {
        deg[i] = ap[i + 1] - ap[i];
    }
    while queue.len() < n {
        let mut start = usize::MAX;
        for i in 0..n {
            if !seen[i] && (start == usize::MAX || deg[i] < deg[start]) {
                start = i;
            }
        }
        let mut head = queue.len();
        seen[start] = true;
        queue.push(start as i32);
        while head < queue.len() {
            let v = queue[head] as usize;
            head += 1;
            let lo = queue.len();
            for p in ap[v]..ap[v + 1] {
                let w = ai[p as usize] as usize;
                if !seen[w] {
                    seen[w] = true;
                    queue.push(w as i32);
                }
            }
            // neighbours in increasing degree
            let hi = queue.len();
            for i in lo..hi {
                for j in i + 1..hi {
                    if deg[queue[j] as usize] < deg[queue[i] as usize] {
                        queue.swap(i, j);
                    }
                }
            }
        }
    }
    let mut perm = vec![0i32; n];
    let mut iperm = vec![0i32; n];
    for i in 0..n {
        perm[i] = queue[n - 1 - i];
    }
    for i in 0..n {
        iperm[perm[i] as usize] = i as i32;
    }
    (perm, iperm)
}

/// A column of n is dense when it touches more than `max(16, 10·√n)` others — AMD's rule for
/// dense rows (Amestoy, Davis & Duff), and `amd::order`'s.  One unknown every instance of a
/// component reads (a shared margin, a pitch) is such a column, and nothing of an ordinary truss,
/// chain or ring is.
pub(crate) fn dense_limit(n: usize) -> f64 {
    (10.0 * (n as f64).sqrt()).max(16.0)
}

fn dense_columns(n: usize, ap: &[i32]) -> Vec<bool> {
    let limit = dense_limit(n);
    // A holds its diagonal, which is not a neighbour
    (0..n).map(|i| (ap[i + 1] - ap[i] - 1) as f64 > limit).collect()
}

/// The fill-reducing order: RCM on the columns that are not `dense`, then the dense ones last, in
/// index order (#440).  A breadth-first search through a dense column puts every column it
/// touches in one level, interleaving what was separate, and the factor of that order is close
/// to dense; eliminated last, a dense column costs one row of L.  With no dense column this is
/// `rcm` on A as it stands.
fn order(n: usize, ap: &[i32], ai: &[i32], dense: &[bool]) -> (Vec<i32>, Vec<i32>) {
    if !dense.contains(&true) {
        return rcm(n, ap, ai);
    }
    // the pattern among the columns that are not dense, renumbered in index order
    let mut local = vec![-1i32; n];
    let mut kept: Vec<i32> = Vec::new();
    for i in 0..n {
        if !dense[i] {
            local[i] = kept.len() as i32;
            kept.push(i as i32);
        }
    }
    let mut sp = vec![0i32; kept.len() + 1];
    let mut si: Vec<i32> = Vec::new();
    for (k, &i) in kept.iter().enumerate() {
        for p in ap[i as usize]..ap[i as usize + 1] {
            let j = local[ai[p as usize] as usize];
            if j >= 0 {
                si.push(j);
            }
        }
        sp[k + 1] = si.len() as i32;
    }
    let (sub, _) = rcm(kept.len(), &sp, &si);
    let mut perm: Vec<i32> = sub.iter().map(|&k| kept[k as usize]).collect();
    perm.extend((0..n).filter(|&i| dense[i]).map(|i| i as i32));
    let mut iperm = vec![0i32; n];
    for i in 0..n {
        iperm[perm[i] as usize] = i as i32;
    }
    (perm, iperm)
}

/// Davis's up-looking LDLᵀ: symbolic elimination tree and column counts.
fn ldl_symbolic(
    n: usize,
    ap: &[i32],
    ai: &[i32],
    perm: &[i32],
    iperm: &[i32],
) -> (Vec<i32>, Vec<i32>, Vec<i32>) {
    let mut parent = vec![-1i32; n];
    let mut lnz = vec![0i32; n];
    let mut flag = vec![0i32; n];
    let mut lp = vec![0i32; n + 1];
    for k in 0..n {
        parent[k] = -1;
        flag[k] = k as i32;
        lnz[k] = 0;
        let kk = perm[k] as usize;
        for p in ap[kk]..ap[kk + 1] {
            let mut i = iperm[ai[p as usize] as usize] as usize;
            if i >= k {
                continue;
            }
            while flag[i] != k as i32 {
                if parent[i] == -1 {
                    parent[i] = k as i32;
                }
                lnz[i] += 1;
                flag[i] = k as i32;
                i = parent[i] as usize;
            }
        }
    }
    lp[0] = 0;
    for k in 0..n {
        lp[k + 1] = lp[k] + lnz[k];
    }
    (lp, parent, lnz)
}

/// The LDLᵀ of a symmetric pattern: its order, its elimination tree and room for the factor's
/// numbers, refilled by `numeric` for each new set of values on the same pattern.  `Ata` is its
/// user.
struct Factor {
    /// The columns ordered last as dense (`dense_columns`).
    dense: usize,
    perm: Vec<i32>,
    iperm: Vec<i32>,
    parent: Vec<i32>,
    lnz: Vec<i32>,
    lp: Vec<i32>,
    li: Vec<i32>,
    lx: Vec<f64>,
    flag: Vec<i32>,
    pattern: Vec<i32>,
    d: Vec<f64>,
    y: Vec<f64>,
    /// The least pivot of the last factorisation as a fraction of its diagonal (`least_pivot`).
    least: f64,
}

impl Factor {
    /// The order and the symbolic factor of `a`'s pattern.
    fn new(a: &Sym) -> Factor {
        let n = a.n;
        let dense = dense_columns(n, &a.ap);
        let (perm, iperm) = order(n, &a.ap, &a.ai, &dense);
        let (lp, parent, lnz) = ldl_symbolic(n, &a.ap, &a.ai, &perm, &iperm);
        let lnz_total = lp[n] as usize;
        Factor {
            dense: dense.iter().filter(|&&d| d).count(),
            perm,
            iperm,
            parent,
            lnz,
            lp,
            li: vec![0i32; lnz_total],
            lx: vec![0.0; lnz_total],
            flag: vec![0i32; n],
            pattern: vec![0i32; n],
            d: vec![0.0; n],
            y: vec![0.0; n],
            least: 1.0,
        }
    }

    fn nnz(&self) -> usize {
        self.lp[self.perm.len()] as usize
    }

    /// Factor `A + diag(damp)`, A's values on the pattern the factor was made for.  `false` on a
    /// zero pivot.
    fn numeric(&mut self, a: &Sym, damp: &[f64]) -> bool {
        let (ap, ai, ax) = (&a.ap, &a.ai, &a.ax);
        let n = self.perm.len();
        self.least = 1.0;
        for k in 0..n {
            self.y[k] = 0.0;
            let mut top = n;
            self.flag[k] = k as i32;
            self.lnz[k] = 0;
            let kk = self.perm[k] as usize;
            for p in ap[kk]..ap[kk + 1] {
                let mut i = self.iperm[ai[p as usize] as usize] as usize;
                if i > k {
                    continue;
                }
                self.y[i] += ax[p as usize];
                let mut len = 0usize;
                while self.flag[i] != k as i32 {
                    self.pattern[len] = i as i32;
                    len += 1;
                    self.flag[i] = k as i32;
                    i = self.parent[i] as usize;
                }
                while len > 0 {
                    top -= 1;
                    len -= 1;
                    self.pattern[top] = self.pattern[len];
                }
            }
            // the diagonal entry of A + diag(damp), what elimination takes the pivot from
            let diag = self.y[k] + damp[kk];
            self.d[k] = diag;
            self.y[k] = 0.0;
            while top < n {
                let i = self.pattern[top] as usize;
                top += 1;
                let yi = self.y[i];
                self.y[i] = 0.0;
                let p2 = (self.lp[i] + self.lnz[i]) as usize;
                for p in self.lp[i] as usize..p2 {
                    let idx = self.li[p] as usize;
                    self.y[idx] -= self.lx[p] * yi;
                }
                let lki = yi / self.d[i];
                self.d[k] -= lki * yi;
                self.li[p2] = k as i32;
                self.lx[p2] = lki;
                self.lnz[i] += 1;
            }
            self.least = self.least.min(if diag > 0.0 { self.d[k].max(0.0) / diag } else { 0.0 });
            if self.d[k] == 0.0 {
                return false;
            }
        }
        true
    }

    /// `b <- (L D Lᵀ)⁻¹ b` with the factor the last `numeric` that succeeded left.
    fn resolve(&self, b: &mut [f64]) {
        let n = self.perm.len();
        let mut x = vec![0.0; n];
        for k in 0..n {
            x[k] = b[self.perm[k] as usize];
        }
        for j in 0..n {
            let (lo, hi) = (self.lp[j] as usize, (self.lp[j] + self.lnz[j]) as usize);
            for p in lo..hi {
                let i = self.li[p] as usize;
                x[i] -= self.lx[p] * x[j];
            }
        }
        for k in 0..n {
            x[k] /= self.d[k];
        }
        for j in (0..n).rev() {
            let (lo, hi) = (self.lp[j] as usize, (self.lp[j] + self.lnz[j]) as usize);
            for p in lo..hi {
                let i = self.li[p] as usize;
                x[j] -= self.lx[p] * x[i];
            }
        }
        for k in 0..n {
            b[self.perm[k] as usize] = x[k];
        }
    }
}

pub struct Ata {
    pub n: usize,
    pub nnz: usize,
    /// The columns ordered last as dense (`dense_columns`): none, for an ordinary sketch.
    pub dense: usize,
    /// JᵀJ, refilled by `fill`.
    a: Sym,
    /// A[ts[t]] += Jdata[ta[t]] * Jdata[tb[t]]
    ta: Vec<i32>,
    tb: Vec<i32>,
    ts: Vec<i32>,
    factor: Factor,
}

impl Ata {
    pub fn new(n_rows: usize, n_cols: usize, indptr: &[i32], indices: &[i32]) -> Ata {
        let n = n_cols;
        let mut tr: Vec<(i32, i32, i32, i32)> = Vec::new();
        for r in 0..n_rows {
            let (lo, hi) = (indptr[r] as usize, indptr[r + 1] as usize);
            for p in lo..hi {
                for q in lo..hi {
                    tr.push((indices[p], indices[q], p as i32, q as i32));
                }
            }
        }
        tr.sort_by_key(|t| (t.0, t.1));
        let nt = tr.len();
        let mut ta = vec![0i32; nt];
        let mut tb = vec![0i32; nt];
        let mut ts = vec![0i32; nt];
        let mut ap = vec![0i32; n + 1];
        let mut ai: Vec<i32> = Vec::with_capacity(nt);
        let mut nnz = 0usize;
        for t in 0..nt {
            if t == 0 || tr[t].0 != tr[t - 1].0 || tr[t].1 != tr[t - 1].1 {
                ai.push(tr[t].1);
                ap[tr[t].0 as usize + 1] = nnz as i32 + 1;
                nnz += 1;
            }
            ta[t] = tr[t].2;
            tb[t] = tr[t].3;
            ts[t] = nnz as i32 - 1;
        }
        for i in 1..=n {
            if ap[i] < ap[i - 1] {
                ap[i] = ap[i - 1];
            }
        }
        let a = Sym { n, ap, ai, ax: vec![0.0; nnz] };
        let factor = Factor::new(&a);
        Ata { n, nnz, dense: factor.dense, a, ta, tb, ts, factor }
    }

    /// The entries of the factor L below its diagonal: the fill of the order chosen.
    pub fn factor_nnz(&self) -> usize {
        self.factor.nnz()
    }

    /// A <- JᵀJ from the Jacobian's CSR values.
    pub fn fill(&mut self, jdata: &[f64]) {
        for v in self.a.ax.iter_mut() {
            *v = 0.0;
        }
        for t in 0..self.ta.len() {
            self.a.ax[self.ts[t] as usize] +=
                jdata[self.ta[t] as usize] * jdata[self.tb[t] as usize];
        }
    }

    /// The diagonal of A, in original (unpermuted) order.
    pub fn diag(&self, out: &mut [f64]) {
        self.a.diag_into(out);
    }

    /// Solve `(A + diag(damp)) x = b` in place.  `false` on a zero pivot.
    pub fn solve(&mut self, damp: &[f64], b: &mut [f64]) -> bool {
        if !self.factor.numeric(&self.a, damp) {
            return false;
        }
        self.resolve(b);
        true
    }

    /// The least pivot of the last factorisation, as a fraction of its column's diagonal in
    /// `A + diag(damp)`, recorded as it was made: in (0, 1] for a positive definite matrix, at
    /// rounding's size where it is singular in exact arithmetic but no pivot came out exactly
    /// zero, and 0 where one did.
    pub fn least_pivot(&self) -> f64 {
        self.factor.least
    }

    /// Solve again, in place, with the factorisation the last `solve` that succeeded left: the
    /// same `(A + diag(damp))`, another right-hand side.
    pub fn resolve(&self, b: &mut [f64]) {
        self.factor.resolve(b);
    }
}

/// JᵀJ for `Ata`: a sparse symmetric matrix, both triangles stored, in compressed columns (each
/// column's rows ascending), refilled in place.  A mesh's matrices are `cholesky::Symmetric`, the
/// lower triangle, assembled in place on the mesh's pattern (#632).
#[derive(Clone, Debug)]
struct Sym {
    n: usize,
    ap: Vec<i32>,
    ai: Vec<i32>,
    ax: Vec<f64>,
}

impl Sym {
    fn diag_into(&self, out: &mut [f64]) {
        for j in 0..self.n {
            let (lo, hi) = (self.ap[j] as usize, self.ap[j + 1] as usize);
            out[j] = match self.ai[lo..hi].binary_search(&(j as i32)) {
                Ok(at) => self.ax[lo + at],
                Err(_) => 0.0,
            };
        }
    }
}

/// The LDLᵀ of a `cholesky::Symmetric`, without pivoting: P1's supernodal factor under
/// approximate minimum degree (`cholesky::Cholesky::factor_ldl`, #510 S3), where a mesh's pencils
/// are factored.  An SPD matrix factors stably; an indefinite one (a pencil shifted into its
/// spectrum) factors when no pivot is zero, and its pivots then carry its inertia (Sylvester): as
/// many negative as the matrix has negative eigenvalues.
pub struct Ldl {
    factor: Cholesky,
    negative: usize,
}

impl Ldl {
    /// The factor of `a`, or `None` on a zero pivot: its pattern analysed through
    /// `cholesky::analysed`, so one analysed already is not ordered again.
    pub fn new(a: &Symmetric) -> Option<Ldl> {
        Ldl::of(analysed(a, Ordering::Amd), &a.values)
    }

    /// `factor`'s analysis given the values `values` on its pattern, or `None` on a zero pivot.
    fn of(mut factor: Cholesky, values: &[f64]) -> Option<Ldl> {
        let negative = factor.factor_ldl_values(values).ok()?;
        Some(Ldl { factor, negative })
    }

    /// `b <- A⁻¹ b`.
    pub fn solve(&self, b: &mut [f64]) {
        self.factor.solve(b);
    }

    /// How many of the pivots are negative: the number of A's eigenvalues below zero, so for
    /// `A − σB` with B positive definite the number of the pencil's below σ.
    pub fn negative_pivots(&self) -> usize {
        self.negative
    }
}

/// **The pencil `L − μR` of two symmetric matrices**, on the union of their patterns, analysed
/// once under minimum degree (#510 S3): a shift, or a count of the eigenvalues below one, is a
/// numeric factor and no more.  A mesh's stiffness, mass and geometric stiffness are all
/// assembled on the mesh's one pattern (`cholesky::Symmetric::pattern`, #632), so their union is
/// that pattern, held by reference, not copied; and it is analysed through `cholesky::analysed`,
/// so the order the static solve found for it is not found again.  Two matrices of different
/// patterns (a frame's, assembled from triplets) are merged column by column.
pub struct Pencil<'a> {
    /// L on the pattern L ∪ R.
    l: Cow<'a, Symmetric>,
    /// R's values on it.
    r: Cow<'a, [f64]>,
    /// The pattern's analysis.
    symbolic: Cholesky,
}

impl<'a> Pencil<'a> {
    pub fn new(l: &'a Symmetric, r: &'a Symmetric) -> Pencil<'a> {
        assert_eq!(l.n, r.n, "a pencil of matrices of different sizes");
        let (l, r) = if l.same_pattern(r) {
            (Cow::Borrowed(l), Cow::Borrowed(&r.values[..]))
        } else {
            let u = union(l, r);
            let (lv, rv) = (scatter(&u, l), scatter(&u, r));
            (Cow::Owned(Symmetric { values: lv, ..u }), Cow::Owned(rv))
        };
        let symbolic = analysed(&l, Ordering::Amd);
        Pencil { l, r, symbolic }
    }

    /// The factor of `L − μR`, or `None` on a zero pivot.
    pub fn factor(&self, mu: f64) -> Option<Ldl> {
        let values: Vec<f64> =
            self.l.values.iter().zip(&*self.r).map(|(l, r)| l - mu * r).collect();
        Ldl::of(self.symbolic.clone(), &values)
    }

    /// The factor of R alone, on the pencil's analysis, or `None` on a zero pivot: whether R is
    /// definite, read without ordering its pattern again.
    pub fn right(&self) -> Option<Ldl> {
        Ldl::of(self.symbolic.clone(), &self.r)
    }
}

/// The pattern `a ∪ b`, no values: each column's rows merged, increasing.
fn union(a: &Symmetric, b: &Symmetric) -> Symmetric {
    let next = |m: &Symmetric, q: usize, end: usize| (q < end).then(|| m.rowind[q]);
    let mut colptr = vec![0usize; a.n + 1];
    let mut rowind = Vec::with_capacity(a.rowind.len().max(b.rowind.len()));
    for j in 0..a.n {
        let (mut p, mut q, pe, qe) = (a.colptr[j], b.colptr[j], a.colptr[j + 1], b.colptr[j + 1]);
        loop {
            let r = match (next(a, p, pe), next(b, q, qe)) {
                (Some(x), Some(y)) => x.min(y),
                (Some(x), None) | (None, Some(x)) => x,
                (None, None) => break,
            };
            p += (next(a, p, pe) == Some(r)) as usize;
            q += (next(b, q, qe) == Some(r)) as usize;
            rowind.push(r);
        }
        colptr[j + 1] = rowind.len();
    }
    Symmetric { n: a.n, colptr, rowind, values: Vec::new() }
}

/// `m`'s values on the pattern `u`, which holds every entry of `m`'s.
fn scatter(u: &Symmetric, m: &Symmetric) -> Vec<f64> {
    let mut out = vec![0.0; u.rowind.len()];
    for j in 0..m.n {
        let (lo, hi) = (u.colptr[j], u.colptr[j + 1]);
        for q in m.colptr[j]..m.colptr[j + 1] {
            let at = u.rowind[lo..hi].binary_search(&m.rowind[q]).expect("an entry of the union");
            out[lo + at] = m.values[q];
        }
    }
    out
}
