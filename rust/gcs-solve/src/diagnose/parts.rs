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

use crate::linalg::{null_full_row_rank, rrqr_with, svd, Mat, Svd, Tol};
use crate::system::{SparseConditioned, RANK_TOL};

/// One part: its rows (of the full residual vector) and its free columns.
pub(super) struct Part {
    pub rows: Vec<usize>,
    pub cols: Vec<usize>,
}

/// A part, factored: its rank, its null space (a vector per column of the part), and the SVD a
/// minimum-norm solve reads, where one is asked of it.
struct Factored {
    rank: usize,
    null: Vec<Vec<f64>>,
    svd: Option<Svd>,
}

impl Factored {
    /// `a` factored; `solves` says whether a minimum-norm solve will be asked of it, which needs
    /// the SVD with its left singular vectors.  A part asked only for its null space is read by
    /// a pivoted QR where one says enough — of full column rank, it has none; wide and of full
    /// row rank (a drawing's under part), its null space is in a QR of its transpose — a few
    /// times cheaper than the SVD.
    fn of(a: &Mat, solves: bool) -> Factored {
        // columns no row reads are free, each one (the SVD of no rows says nothing useful)
        if a.rows == 0 {
            let null = (0..a.cols).map(|k| (0..a.cols).map(|i| f64::from(i == k)).collect());
            return Factored { rank: 0, null: null.collect(), svd: None };
        }
        if !solves && a.cols <= a.rows && a.cols > 0 {
            let (rank, _) = rrqr_with(a, Tol::Abs(RANK_TOL));
            if rank == a.cols {
                return Factored { rank, null: Vec::new(), svd: None };
            }
        }
        if !solves && a.rows < a.cols && a.rows > 0 {
            if let Some(n) = null_full_row_rank(a, Tol::Abs(RANK_TOL)) {
                let null = (0..n.cols).map(|k| n.col(k)).collect();
                return Factored { rank: a.rows, null, svd: None };
            }
        }
        let d = svd(a, solves);
        // an SVD that did not converge says nothing: the matching's rank stands for the part
        if !d.converged {
            return Factored { rank: a.rows.min(a.cols), null: Vec::new(), svd: None };
        }
        let rank = d.s.iter().filter(|&&x| x > RANK_TOL).count();
        let null = (rank..d.vt.rows).map(|k| d.vt.row(k).to_vec()).collect();
        Factored { rank, null, svd: solves.then_some(d) }
    }

    /// `A⁺ b` for each `b`, over the singular values past the tolerance: what makes up `b` as
    /// well as the part can, and nothing along its null space.  `None` where the part was not
    /// factored to solve (its SVD did not converge).
    fn solve(&self, bs: &[Vec<f64>]) -> Option<Vec<Vec<f64>>> {
        let d = self.svd.as_ref()?;
        let n = d.vt.cols;
        Some(bs.iter().map(|b| {
            let mut x = vec![0.0; n];
            for k in 0..self.rank {
                let coef = (0..b.len()).map(|i| d.u.at(i, k) * b[i]).sum::<f64>() / d.s[k];
                for (xj, vj) in x.iter_mut().zip(d.vt.row(k)) {
                    *xj += coef * vj;
                }
            }
            x
        }).collect())
    }
}

fn normalized(mut v: Vec<f64>) -> Vec<f64> {
    let n = crate::linalg::norm(&v);
    if n > 0.0 {
        v.iter_mut().for_each(|x| *x /= n);
    }
    v
}

/// The null space of `j`, over the parts in an order where each part's rows read only its own
/// columns and earlier parts': each vector `j.n_cols` long.
fn null_by_parts<'p>(
    j: &SparseConditioned,
    parts: impl Iterator<Item = (&'p [usize], &'p [usize])>,
) -> Vec<Vec<f64>> {
    let n = j.n_cols;
    let mut cands: Vec<Vec<f64>> = Vec::new();
    // rows a part could not make up exactly: where a combination of candidates may still fail
    let mut short: Vec<usize> = Vec::new();
    for (rows, cols) in parts {
        if cols.is_empty() {
            continue;
        }
        // what each earlier candidate puts on this part's rows, to be made up here
        let bs: Vec<Vec<f64>> =
            cands.iter().map(|v| rows.iter().map(|&r| -j.row_dot(r, v)).collect()).collect();
        let solves = bs.iter().any(|b| b.iter().any(|x| *x != 0.0));
        let f = Factored::of(&j.dense(rows, cols), solves);
        if solves {
            match f.solve(&bs) {
                Some(xs) => {
                    for (v, x) in cands.iter_mut().zip(xs) {
                        for (&c, y) in cols.iter().zip(x) {
                            v[c] = y;
                        }
                    }
                }
                None => short.extend(rows),
            }
        }
        if f.rank < rows.len() {
            short.extend(rows);
        }
        for nv in f.null {
            let mut v = vec![0.0; n];
            for (&c, y) in cols.iter().zip(nv) {
                v[c] = y;
            }
            cands.push(normalized(v));
        }
    }
    if cands.is_empty() || short.is_empty() {
        return cands;
    }
    // the combinations every short row sends to zero
    let mut m = Mat::zeros(short.len(), cands.len());
    for (k, v) in cands.iter().enumerate() {
        for (i, &r) in short.iter().enumerate() {
            m.data[i * cands.len() + k] = j.row_dot(r, v);
        }
    }
    let kernel = Factored::of(&m, false).null;
    let combos = Mat::from_vec(cands.len(), n, cands.concat());
    kernel.iter().map(|c| normalized(combos.mul_t_vec(c))).collect()
}

/// The whole Jacobian's null space over its free columns, part by part: each vector `n_cols` long.
pub(super) fn right_null(j: &SparseConditioned, parts: &[Part]) -> Vec<Vec<f64>> {
    null_by_parts(j, parts.iter().map(|p| (&p.rows[..], &p.cols[..])))
}

/// The whole Jacobian's left null space over the parts' rows: the null space of its transpose,
/// the parts taken backwards with rows and columns exchanged — each vector indexed by the full
/// residual vector's rows.
pub(super) fn left_null(j: &SparseConditioned, parts: &[Part]) -> Vec<Vec<f64>> {
    null_by_parts(&j.transpose(), parts.iter().rev().map(|p| (&p.cols[..], &p.rows[..])))
}
