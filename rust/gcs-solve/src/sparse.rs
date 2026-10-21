//! Sparse normal equations for the large-sketch path: JᵀJ assembled from the fixed CSR structure
//! of the Jacobian, ordered by reverse Cuthill–McKee with any dense column last, and factored by
//! an up-looking LDLᵀ (Davis).  Regularizing the diagonal keeps rank-deficient
//! (under-constrained) systems solvable, which is the normal case while editing.
//!
//! The same factor serves a symmetric matrix given by its entries (`Sym`, `Ldl`): a stiffness or
//! a mass, and a pencil `L − μR` analysed once and factored at each μ the eigensolver asks for
//! (`Pencil`, `eigen.rs`, #510), whose pivots' signs count the eigenvalues below μ.

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
/// numbers, refilled by `numeric` for each new set of values on the same pattern.  `Ata`, `Ldl`
/// and `Pencil` are its users.
#[derive(Clone)]
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
    fn numeric(&mut self, a: &Sym, damp: Option<&[f64]>) -> bool {
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
            let diag = self.y[k] + damp.map_or(0.0, |d| d[kk]);
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
        if !self.factor.numeric(&self.a, Some(damp)) {
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

    /// `A x`.  Both triangles are stored, so the columns read as rows.
    pub fn mul_vec(&self, x: &[f64]) -> Vec<f64> {
        let mut y = vec![0.0; self.n];
        crate::system::csr_mul(&self.ap, &self.ai, &self.ax, x, &mut y);
        y
    }

    /// The diagonal, zero where nothing is stored.
    pub fn diag(&self) -> Vec<f64> {
        let mut d = vec![0.0; self.n];
        self.diag_into(&mut d);
        d
    }

    fn diag_into(&self, out: &mut [f64]) {
        for j in 0..self.n {
            let (lo, hi) = (self.ap[j] as usize, self.ap[j + 1] as usize);
            out[j] = match self.ai[lo..hi].binary_search(&(j as i32)) {
                Ok(at) => self.ax[lo + at],
                Err(_) => 0.0,
            };
        }
    }

    /// The largest column sum of |A|: a bound on its 2-norm, the scale a residual is read at.
    pub fn norm1(&self) -> f64 {
        let mut sum = vec![0.0; self.n];
        for (_, j, v) in self.entries() {
            sum[j] += v.abs();
        }
        sum.into_iter().fold(0.0f64, f64::max)
    }

    /// `c A`.
    pub fn scaled(&self, c: f64) -> Sym {
        Sym { ax: self.ax.iter().map(|v| c * v).collect(), ..self.clone() }
    }

    /// `m`'s values on this matrix's pattern, which holds every entry of `m`'s.
    fn scatter(&self, m: &Sym) -> Vec<f64> {
        let mut out = vec![0.0; self.ax.len()];
        for (i, j, v) in m.entries() {
            let (lo, hi) = (self.ap[j] as usize, self.ap[j + 1] as usize);
            let at = self.ai[lo..hi].binary_search(&(i as i32)).expect("an entry of the union");
            out[lo + at] += v;
        }
        out
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
        let mut factor = Factor::new(a);
        factor.numeric(a, None).then_some(Ldl { factor })
    }

    /// `b <- A⁻¹ b`.
    pub fn solve(&self, b: &mut [f64]) {
        self.factor.resolve(b);
    }

    /// How many of the pivots are negative: the number of A's eigenvalues below zero, so for
    /// `A − σB` with B positive definite the number of the pencil's below σ.
    pub fn negative_pivots(&self) -> usize {
        self.factor.d.iter().filter(|&&d| d < 0.0).count()
    }
}

/// The pencil `L − μR` of two `Sym`s, on the union of their patterns, ordered and analysed
/// once: a shift, or a count of the eigenvalues below one, is a numeric factor and no more.
pub struct Pencil {
    /// L's values on the pattern L ∪ R.
    l: Sym,
    /// R's values on the same pattern.
    r: Vec<f64>,
    symbolic: Factor,
}

impl Pencil {
    pub fn new(l: &Sym, r: &Sym) -> Pencil {
        assert_eq!(l.n, r.n, "a pencil of matrices of different sizes");
        let mut t: Vec<(usize, usize, f64)> = l.entries().collect();
        t.extend(r.entries().map(|(i, j, _)| (i, j, 0.0)));
        let mut u = Sym::from_triplets(l.n, &t);
        let r_values = u.scatter(r);
        u.ax = u.scatter(l);
        let symbolic = Factor::new(&u);
        Pencil { l: u, r: r_values, symbolic }
    }

    /// `L − μR`.
    pub fn at(&self, mu: f64) -> Sym {
        let ax = self.l.ax.iter().zip(&self.r).map(|(l, r)| l - mu * r).collect();
        Sym { ax, ..self.l.clone() }
    }

    /// The factor of `L − μR`, or `None` on a zero pivot.
    pub fn factor(&self, mu: f64) -> Option<Ldl> {
        let mut factor = self.symbolic.clone();
        factor.numeric(&self.at(mu), None).then_some(Ldl { factor })
    }
}
