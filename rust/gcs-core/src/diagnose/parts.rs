//! **The numeric rank, part by part** (#88).  Past `NUMERIC_MAX` the whole Jacobian is too large
//! to factor after every edit, but the block-triangular order (`System::block_order`) cuts it
//! into parts that are each small, ordered so a part's rows read only its own columns and earlier
//! parts': the over-determined part (its rows read nothing else), the square blocks in the order
//! the matching solves them, and the under-determined part last.  Each part is factored on its
//! own, and what one part leaves free is carried into the parts after it.
//!
//! **The reading is exact, not a bound.**  A vector the whole Jacobian sends to zero is zero on
//! the first part's rows, so its piece there is in that part's null space; on each later part's
//! rows, the part's own piece makes up what the earlier ones put there.  So every such vector is
//! a combination of *candidates* — a part's null vector, its effect on each later part made up by
//! that part's minimum-norm solve — and only where a later part cannot make it up (rows it is
//! short of) does a combination fail.  Those few rows against the few candidates are one small
//! matrix; its null space is the whole's.  Summing the parts' ranks instead reads a later part
//! making up an earlier one's deficiency (two tangencies of one belt) as a freedom it is not.
//! Dependencies among rows are the same reading of the transpose, the parts taken backwards.

use crate::linalg::{rrqr_with, svd, Mat, Tol};
use crate::system::{SparseConditioned, RANK_TOL};

/// One part: its rows (of the full residual vector) and its free columns.
pub(super) struct Part {
    pub rows: Vec<usize>,
    pub cols: Vec<usize>,
}

/// A part, factored: the singular vectors a minimum-norm solve and the null space are read off.
struct Factored {
    u: Mat,
    s: Vec<f64>,
    vt: Mat,
    rank: usize,
    /// The null space where a QR found it, the SVD's `vt` then unread.
    null: Option<Vec<Vec<f64>>>,
}

impl Factored {
    /// `a` factored; `solves` says whether a minimum-norm solve will be asked of it, which needs
    /// the left singular vectors (most parts are only asked for their null space).
    fn of(a: &Mat, solves: bool) -> Factored {
        // a part asked only for its null space, and of full column rank by a pivoted QR (a few
        // times cheaper than the SVD), has none: the one large block of a big drawing is
        // usually this
        let quick = |rank: usize, null: Vec<Vec<f64>>| Factored {
            u: Mat::zeros(0, 0),
            s: Vec::new(),
            vt: Mat::zeros(0, 0),
            rank,
            null: Some(null),
        };
        if !solves && a.cols <= a.rows && a.cols > 0 {
            let (rank, _) = rrqr_with(a, Tol::Abs(RANK_TOL));
            if rank == a.cols {
                return quick(rank, Vec::new());
            }
        }
        // and a wide one of full row rank — a drawing's under part — has its null space in a QR
        if !solves && a.rows < a.cols && a.rows > 0 {
            if let Some(n) = crate::linalg::null_full_row_rank(a, Tol::Abs(RANK_TOL)) {
                let null = (0..n.cols).map(|k| n.col(k)).collect();
                return quick(a.rows, null);
            }
        }
        let d = svd(a, solves);
        let rank = d.s.iter().filter(|&&x| x > RANK_TOL).count();
        Factored { u: d.u, s: d.s, vt: d.vt, rank, null: None }
    }

    /// `A⁺ b`, over the singular values past the tolerance: what makes up `b` as well as the part
    /// can, and nothing along its null space.
    fn solve(&self, b: &[f64]) -> Vec<f64> {
        let n = self.vt.cols;
        let mut x = vec![0.0; n];
        for k in 0..self.rank {
            let coef: f64 = (0..b.len()).map(|i| self.u.at(i, k) * b[i]).sum::<f64>() / self.s[k];
            for (j, xj) in x.iter_mut().enumerate() {
                *xj += coef * self.vt.at(k, j);
            }
        }
        x
    }

    /// The null space, one vector per column of the part.
    fn null(&self) -> Vec<Vec<f64>> {
        match &self.null {
            Some(n) => n.clone(),
            None => (self.rank..self.vt.rows).map(|k| self.vt.row(k).to_vec()).collect(),
        }
    }
}

fn normalized(mut v: Vec<f64>) -> Vec<f64> {
    let n = v.iter().map(|x| x * x).sum::<f64>().sqrt();
    if n > 0.0 {
        v.iter_mut().for_each(|x| *x /= n);
    }
    v
}

/// The combinations of `cands` the operator `op` sends to zero on every entry in `check`.
fn kernel_of(
    cands: Vec<Vec<f64>>,
    check: &[usize],
    op: impl Fn(&[f64]) -> Vec<f64>,
) -> Vec<Vec<f64>> {
    if cands.is_empty() || check.is_empty() {
        return cands;
    }
    let mut m = Mat::zeros(check.len(), cands.len());
    for (k, v) in cands.iter().enumerate() {
        let out = op(v);
        for (i, &r) in check.iter().enumerate() {
            m.data[i * cands.len() + k] = out[r];
        }
    }
    let f = Factored::of(&m, false);
    f.null().iter().map(|c| {
        let mut v = vec![0.0; cands[0].len()];
        for (k, ck) in c.iter().enumerate() {
            for (x, y) in v.iter_mut().zip(&cands[k]) {
                *x += ck * y;
            }
        }
        normalized(v)
    }).collect()
}

/// The whole Jacobian's null space over its free columns, part by part: each vector `n_cols` long.
pub(super) fn right_null(j: &SparseConditioned, parts: &[Part]) -> Vec<Vec<f64>> {
    let n = j.n_cols;
    let mut cands: Vec<Vec<f64>> = Vec::new();
    // rows a part could not make up exactly: where a combination of candidates may still fail
    let mut short: Vec<usize> = Vec::new();
    for p in parts {
        if p.cols.is_empty() {
            continue;
        }
        let f = Factored::of(&j.dense(&p.rows, &p.cols), !cands.is_empty() && !p.rows.is_empty());
        if !p.rows.is_empty() {
            // what each earlier candidate puts on this part's rows, made up here
            for v in cands.iter_mut() {
                let b: Vec<f64> = p.rows.iter().map(|&r| -j.row_dot(r, v)).collect();
                if b.iter().all(|x| *x == 0.0) {
                    continue;
                }
                for (&c, y) in p.cols.iter().zip(f.solve(&b)) {
                    v[c] = y;
                }
            }
        }
        if f.rank < p.rows.len() {
            short.extend(&p.rows);
        }
        for nv in f.null() {
            let mut v = vec![0.0; n];
            for (&c, y) in p.cols.iter().zip(nv) {
                v[c] = y;
            }
            cands.push(v);
        }
    }
    let cands = cands.into_iter().map(normalized).collect();
    let rows = j.n_rows();
    kernel_of(cands, &short, |v| (0..rows).map(|r| j.row_dot(r, v)).collect())
}

/// The whole Jacobian's left null space over `rows` (the rows a reading counts), part by part,
/// backwards: each vector indexed by the full residual vector's rows.
pub(super) fn left_null(j: &SparseConditioned, parts: &[Part], n_rows: usize) -> Vec<Vec<f64>> {
    // a row vector against every column, `wᵀJ`
    let jt = |w: &[f64]| -> Vec<f64> {
        let mut v = vec![0.0; j.n_cols];
        for (r, &wr) in w.iter().enumerate().filter(|(_, wr)| **wr != 0.0) {
            for (c, x) in j.row(r) {
                v[c] += wr * x;
            }
        }
        v
    };
    let mut cands: Vec<Vec<f64>> = Vec::new();
    let mut short: Vec<usize> = Vec::new();
    for p in parts.iter().rev() {
        if p.rows.is_empty() {
            continue;
        }
        let f = Factored::of(&j.dense(&p.rows, &p.cols).transpose(), !cands.is_empty());
        if !p.cols.is_empty() {
            for w in cands.iter_mut() {
                let wj = jt(w);
                let b: Vec<f64> = p.cols.iter().map(|&c| -wj[c]).collect();
                if b.iter().all(|x| *x == 0.0) {
                    continue;
                }
                for (&r, y) in p.rows.iter().zip(f.solve(&b)) {
                    w[r] = y;
                }
            }
        }
        if f.rank < p.cols.len() {
            short.extend(&p.cols);
        }
        for nv in f.null() {
            let mut w = vec![0.0; n_rows];
            for (&r, y) in p.rows.iter().zip(nv) {
                w[r] = y;
            }
            cands.push(w);
        }
    }
    let cands = cands.into_iter().map(normalized).collect();
    kernel_of(cands, &short, jt)
}
