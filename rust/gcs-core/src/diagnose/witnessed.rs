//! **A claim with witnesses of its own** (§9.7, #460): `claim { x := point; x coincident upper;
//! x coincident lower; x coincident centres }` claims that a point lying on all three exists.
//! The drawing held every witness while it solved (`program::witnessed`), so the block's
//! geometry restricted nothing; here, after it, the block's witnesses are solved by the
//! block's rows with the drawing held where it landed, and the block is judged as one claim.
//!
//! - **violated**: no witness satisfies the block at this solution — the runs of a belt over
//!   equal pulleys are parallel and meet nowhere.  The drawing is untouched either way.
//! - **theorem**: a witness does, and does for every nearby solution of the drawing: each
//!   motion `dp` the drawing allows (`J_m dp = 0`) is followed by some motion `dw` of the
//!   witnesses (`G_p dp + G_w dw = 0`).  That is `rank [J_m 0; G_p G_w] = rank J_m + rank G_w`
//!   — and with no witness it is §9.7's own test, the claim's rows adding no rank.
//! - **consuming**: a witness does, but only at this pose of the drawing.

use super::*;

/// What one block came to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Verdict {
    Theorem,
    Violated,
    Consuming,
}

/// The `claim { … }` blocks the sketch states (`Constraint::witness`), each its rows' ids in
/// sketch order.
pub(super) fn blocks(sk: &Sketch) -> BTreeMap<u32, Vec<u32>> {
    let mut out: BTreeMap<u32, Vec<u32>> = BTreeMap::new();
    for c in sk.constraints.iter().filter(|c| c.claim) {
        if let Some(w) = c.witness {
            out.entry(w).or_default().push(c.id);
        }
    }
    out
}

/// Solve one block's witnesses and judge it, writing the witnesses back where the block holds.
/// `tol` is the diagnosis's residual tolerance, `numeric` whether ranks are read at all (past
/// the numeric limit a block that holds is reported a theorem, as a lone claim is).
///
/// The witness solve is a compile of its own — the witnesses' unknowns are no column of `sys`
/// — over a copy holding everything else, so it pays `locus::forget` once per block: paid only
/// by a document that writes a block, and nothing beside it is a contact's march to forget.
pub(super) fn judge(
    sk: &mut Sketch,
    sys: &mut System,
    rows: &[u32],
    tol: f64,
    numeric: bool,
) -> Verdict {
    // the witnesses' numbers the block reads: each a number the drawing holds
    let owned: BTreeSet<u32> = sk.roles.iter()
        .filter(|(_, r)| r.witness)
        .flat_map(|(&e, _)| sk.own_params(e))
        .collect();
    let block: Vec<&Constraint> = rows.iter().filter_map(|&id| sk.constraint(id)).collect();
    let mut unknowns: Vec<u32> = block.iter()
        .flat_map(|c| c.params(sk))
        .filter(|p| owned.contains(p))
        .collect();
    unknowns.sort_unstable();
    unknowns.dedup();
    // "does it hold?" in the units the drawing measures every row in, the witnesses where they
    // were before their solve: a witness run off to infinity cannot loosen its own test
    let extent = sk.extent().max(1.0);
    let holds = |sk: &Sketch, c: &Constraint| {
        let e = c.error(sk);
        !e.is_nan() && e <= tol * extent.dpowi(c.kernel_in(sk).degree as i32)
    };
    if !unknowns.is_empty() {
        let mut scratch = sk.clone();
        for (i, p) in scratch.params.iter_mut().enumerate() {
            p.fixed = unknowns.binary_search(&(i as u32)).is_err();
        }
        // the block's rows stated, and the witnesses' own intrinsic rows with them; nothing
        // else reads an unknown, so nothing else is kept
        scratch.constraints.retain(|c| {
            rows.contains(&c.id)
                || (c.intrinsic && c.params(sk).iter().any(|p| unknowns.binary_search(p).is_ok()))
        });
        for c in scratch.constraints.iter_mut() {
            c.claim = false;
            c.witness = None;
        }
        let mut wsys = System::new(&scratch);
        wsys.solve(&mut scratch, SolveOpts::default());
        let all_hold = scratch.constraints.iter().all(|c| holds(&scratch, c));
        if !all_hold {
            return Verdict::Violated;
        }
        for &p in &unknowns {
            sk.params[p as usize].value = scratch.params[p as usize].value;
        }
    } else if !block.iter().all(|c| holds(sk, c)) {
        return Verdict::Violated;
    }
    if !numeric {
        return Verdict::Theorem;
    }
    let z = sys.z0(sk);
    let block: Vec<&Constraint> = rows.iter().filter_map(|&id| sk.constraint(id)).collect();
    let (jc, row_c) = sys.conditioned_over(sk, &z, &block, &unknowns);
    let base: Vec<usize> = (0..jc.rows()).filter(|&r| !rows.contains(&row_c[r])).collect();
    let mine: Vec<usize> = (0..jc.rows()).filter(|&r| rows.contains(&row_c[r])).collect();
    let rank_m = jc.select_rows(&base).rank_rrqr(RANK_TOL);
    let rank_w = jc.select_tail(&mine, sys.n_free).rank_rrqr(RANK_TOL);
    if jc.rank_rrqr(RANK_TOL) > rank_m + rank_w {
        Verdict::Consuming
    } else {
        Verdict::Theorem
    }
}
