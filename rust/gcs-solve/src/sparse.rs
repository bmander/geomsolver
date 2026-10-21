//! Sparse normal equations for the large-sketch path: JᵀJ assembled from the fixed CSR structure
//! of the Jacobian, ordered by reverse Cuthill–McKee with any dense column last, and factored by
//! an up-looking LDLᵀ (Davis).  Regularizing the diagonal keeps rank-deficient
//! (under-constrained) systems solvable, which is the normal case while editing.
//!
//! The same factor serves a symmetric matrix given by its entries (`Sym`, `Ldl`): a stiffness or
//! a mass, or a pencil `A − σB` shifted for the eigensolver (`eigen.rs`, #510), whose pivots'
//! signs count the eigenvalues below the shift.

/// Reverse Cuthill–McKee on the pattern of A: short, and the sketch graphs this solver sees
/// (trusses, chains, rings) are close to banded once relabelled this way.
fn rcm(n: usize, ap: &[i32], ai: &[i32]) -> (Vec<i32>, Vec<i32>) {
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

/// A column of A is dense when it touches more than `max(16, 10·√n)` others — AMD's rule for
/// dense rows (Amestoy, Davis & Duff).  One unknown every instance of a component reads (a shared
/// margin, a pitch) is such a column, and nothing of an ordinary truss, chain or ring is.
fn dense_columns(n: usize, ap: &[i32]) -> Vec<bool> {
    let limit = (10.0 * (n as f64).sqrt()).max(16.0);
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
/// numbers, refilled by `numeric` for each new set of values on the same pattern.  `Ata` and
/// `Ldl` are its two users.
struct Factor {
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
}

impl Factor {
    /// The order and the symbolic factor of the pattern `ap`/`ai` (both triangles).
    fn new(n: usize, ap: &[i32], ai: &[i32], dense: &[bool]) -> Factor {
        let (perm, iperm, lp, parent, lnz) = if n > 0 {
            let (perm, iperm) = order(n, ap, ai, dense);
            let (lp, parent, lnz) = ldl_symbolic(n, ap, ai, &perm, &iperm);
            (perm, iperm, lp, parent, lnz)
        } else {
            (Vec::new(), Vec::new(), vec![0i32], Vec::new(), Vec::new())
        };
        let lnz_total = if n > 0 { lp[n] as usize } else { 0 };
        Factor {
            perm,
            iperm,
            parent,
            lnz,
            lp,
            li: vec![0i32; lnz_total.max(1)],
            lx: vec![0.0; lnz_total.max(1)],
            flag: vec![0i32; n.max(1)],
            pattern: vec![0i32; n.max(1)],
            d: vec![0.0; n.max(1)],
            y: vec![0.0; n.max(1)],
        }
    }

    fn nnz(&self) -> usize {
        self.lp.last().map_or(0, |&k| k as usize)
    }

    /// Factor `A + diag(damp)`, A's values `ax` on the pattern the factor was made for.  `false`
    /// on a zero pivot.
    fn numeric(&mut self, ap: &[i32], ai: &[i32], ax: &[f64], damp: &[f64]) -> bool {
        let n = self.perm.len();
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
            self.d[k] = self.y[k] + damp[kk];
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
    ap: Vec<i32>,
    ai: Vec<i32>,
    ax: Vec<f64>,
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
        let mut ai_tmp: Vec<i32> = Vec::with_capacity(nt);
        let mut nnz = 0usize;
        for t in 0..nt {
            if t == 0 || tr[t].0 != tr[t - 1].0 || tr[t].1 != tr[t - 1].1 {
                ai_tmp.push(tr[t].1);
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
        let ai = ai_tmp;
        let ax = vec![0.0; nnz.max(1)];

        let dense = dense_columns(n, &ap);
        let factor = Factor::new(n, &ap, &ai, &dense);
        Ata {
            n,
            nnz,
            dense: dense.iter().filter(|&&d| d).count(),
            ap,
            ai,
            ax,
            ta,
            tb,
            ts,
            factor,
        }
    }

    /// The entries of the factor L below its diagonal: the fill of the order chosen.
    pub fn factor_nnz(&self) -> usize {
        self.factor.nnz()
    }

    /// A <- JᵀJ from the Jacobian's CSR values.
    pub fn fill(&mut self, jdata: &[f64]) {
        for v in self.ax.iter_mut() {
            *v = 0.0;
        }
        for t in 0..self.ta.len() {
            self.ax[self.ts[t] as usize] += jdata[self.ta[t] as usize] * jdata[self.tb[t] as usize];
        }
    }

    /// The diagonal of A, in original (unpermuted) order.
    pub fn diag(&self, out: &mut [f64]) {
        for i in 0..self.n {
            out[i] = 0.0;
            for q in self.ap[i]..self.ap[i + 1] {
                if self.ai[q as usize] == i as i32 {
                    out[i] = self.ax[q as usize];
                }
            }
        }
    }

    /// Solve `(A + diag(damp)) x = b` in place.  `false` on a zero pivot.
    pub fn solve(&mut self, damp: &[f64], b: &mut [f64]) -> bool {
        let n = self.n;
        if n == 0 {
            return true;
        }
        if !self.factor.numeric(&self.ap, &self.ai, &self.ax, damp) {
            return false;
        }
        self.resolve(b);
        true
    }

    /// Solve again, in place, with the factorisation the last `solve` that succeeded left: the
    /// same `(A + diag(damp))`, another right-hand side.
    pub fn resolve(&self, b: &mut [f64]) {
        if self.n > 0 {
            self.factor.resolve(b);
        }
    }
}

/// A sparse symmetric matrix, both triangles stored, in compressed columns (each column's rows
/// ascending).  What an assembly makes: `from_triplets` sums repeated entries, the way element
/// matrices are added into a global one.
#[derive(Clone, Debug)]
pub struct Sym {
    pub n: usize,
    ap: Vec<i32>,
    ai: Vec<i32>,
    ax: Vec<f64>,
}

impl Sym {
    /// The n×n matrix whose `(i, j)` entry is the sum of the triplets naming it.  Every entry
    /// of the matrix is given, below the diagonal as well as above; one given on a side only
    /// is not mirrored, and a matrix that is not symmetric is not one this factors.
    pub fn from_triplets(n: usize, entries: &[(usize, usize, f64)]) -> Sym {
        let mut t: Vec<(usize, usize, f64)> = entries.to_vec();
        t.sort_by_key(|e| (e.1, e.0));
        let mut ap = vec![0i32; n + 1];
        let mut ai: Vec<i32> = Vec::with_capacity(t.len());
        let mut ax: Vec<f64> = Vec::with_capacity(t.len());
        let mut last: Option<(usize, usize)> = None;
        for &(i, j, v) in &t {
            assert!(i < n && j < n, "entry ({i}, {j}) outside a {n}×{n} matrix");
            if last == Some((i, j)) {
                *ax.last_mut().unwrap() += v;
                continue;
            }
            last = Some((i, j));
            ai.push(i as i32);
            ax.push(v);
            ap[j + 1] += 1;
        }
        for j in 0..n {
            ap[j + 1] += ap[j];
        }
        Sym { n, ap, ai, ax }
    }

    /// The stored entries, `(row, column, value)`, column by column.
    pub fn entries(&self) -> impl Iterator<Item = (usize, usize, f64)> + '_ {
        (0..self.n).flat_map(move |j| {
            (self.ap[j] as usize..self.ap[j + 1] as usize)
                .map(move |p| (self.ai[p] as usize, j, self.ax[p]))
        })
    }

    pub fn nnz(&self) -> usize {
        self.ax.len()
    }

    /// `y = A x`.
    pub fn mul_vec(&self, x: &[f64]) -> Vec<f64> {
        let mut y = vec![0.0; self.n];
        for j in 0..self.n {
            let xj = x[j];
            for p in self.ap[j] as usize..self.ap[j + 1] as usize {
                y[self.ai[p] as usize] += self.ax[p] * xj;
            }
        }
        y
    }

    /// The diagonal, zero where nothing is stored.
    pub fn diag(&self) -> Vec<f64> {
        let mut d = vec![0.0; self.n];
        for (i, j, v) in self.entries() {
            if i == j {
                d[i] = v;
            }
        }
        d
    }

    /// The largest column sum of |A|: a bound on its 2-norm, the scale a residual is read at.
    pub fn norm1(&self) -> f64 {
        (0..self.n)
            .map(|j| (self.ap[j] as usize..self.ap[j + 1] as usize).map(|p| self.ax[p].abs()).sum())
            .fold(0.0f64, f64::max)
    }

    /// `a·self + b·other`, on the union of the two patterns.
    pub fn combine(&self, a: f64, other: &Sym, b: f64) -> Sym {
        assert_eq!(self.n, other.n, "combining matrices of different sizes");
        let mut t: Vec<(usize, usize, f64)> =
            self.entries().map(|(i, j, v)| (i, j, a * v)).collect();
        t.extend(other.entries().map(|(i, j, v)| (i, j, b * v)));
        Sym::from_triplets(self.n, &t)
    }
}

/// The LDLᵀ of a `Sym`, without pivoting: the order `Ata`'s is (RCM, dense columns last), the
/// same up-looking factor.  An SPD matrix factors stably; an indefinite one (a pencil shifted
/// into its spectrum) factors when no pivot is zero, and its pivots then carry its inertia
/// (Sylvester): as many negative as the matrix has negative eigenvalues.
pub struct Ldl {
    factor: Factor,
}

impl Ldl {
    /// The factor of `a`, or `None` on a zero pivot.
    pub fn new(a: &Sym) -> Option<Ldl> {
        let n = a.n;
        let dense = dense_columns(n, &a.ap);
        let mut factor = Factor::new(n, &a.ap, &a.ai, &dense);
        if n > 0 && !factor.numeric(&a.ap, &a.ai, &a.ax, &vec![0.0; n]) {
            return None;
        }
        Some(Ldl { factor })
    }

    /// `b <- A⁻¹ b`.
    pub fn solve(&self, b: &mut [f64]) {
        if !self.factor.perm.is_empty() {
            self.factor.resolve(b);
        }
    }

    /// How many of the pivots are negative: the number of A's eigenvalues below zero, so for
    /// `A − σB` with B positive definite the number of the pencil's below σ.
    pub fn negative_pivots(&self) -> usize {
        let n = self.factor.perm.len();
        self.factor.d[..n].iter().filter(|&&d| d < 0.0).count()
    }

    /// The fill: the entries of L below its diagonal.
    pub fn factor_nnz(&self) -> usize {
        self.factor.nnz()
    }
}
