//! Sparse normal equations for the large-sketch path: JᵀJ assembled from the fixed CSR structure
//! of the Jacobian, ordered by reverse Cuthill–McKee and factored by an up-looking LDLᵀ (Davis).
//! Regularizing the diagonal keeps rank-deficient (under-constrained) systems solvable, which is
//! the normal case while editing.

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

pub struct Ata {
    pub n: usize,
    pub nnz: usize,
    ap: Vec<i32>,
    ai: Vec<i32>,
    ax: Vec<f64>,
    /// A[ts[t]] += Jdata[ta[t]] * Jdata[tb[t]]
    ta: Vec<i32>,
    tb: Vec<i32>,
    ts: Vec<i32>,
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

        let (perm, iperm, lp, parent, lnz) = if n > 0 {
            let (perm, iperm) = rcm(n, &ap, &ai);
            let (lp, parent, lnz) = ldl_symbolic(n, &ap, &ai, &perm, &iperm);
            (perm, iperm, lp, parent, lnz)
        } else {
            (Vec::new(), Vec::new(), vec![0i32], Vec::new(), Vec::new())
        };
        let lnz_total = if n > 0 { lp[n] as usize } else { 0 };
        Ata {
            n,
            nnz,
            ap,
            ai,
            ax,
            ta,
            tb,
            ts,
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

    fn numeric(&mut self, damp: &[f64]) -> bool {
        let n = self.n;
        for k in 0..n {
            self.y[k] = 0.0;
            let mut top = n;
            self.flag[k] = k as i32;
            self.lnz[k] = 0;
            let kk = self.perm[k] as usize;
            for p in self.ap[kk]..self.ap[kk + 1] {
                let mut i = self.iperm[self.ai[p as usize] as usize] as usize;
                if i > k {
                    continue;
                }
                self.y[i] += self.ax[p as usize];
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

    /// Solve `(A + diag(damp)) x = b` in place.  `false` on a zero pivot.
    pub fn solve(&mut self, damp: &[f64], b: &mut [f64]) -> bool {
        let n = self.n;
        if n == 0 {
            return true;
        }
        if !self.numeric(damp) {
            return false;
        }
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
        true
    }
}
