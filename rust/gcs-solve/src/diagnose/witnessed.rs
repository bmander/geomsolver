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

/// Solve block `w`'s witnesses and judge it, writing them back where the block holds.  `tol` is
/// the diagnosis's residual tolerance, `extent` the drawing's before any witness moved — each row
/// is judged at its own size (`Sizes`, #492) but never larger than that, so a witness run off to
/// infinity cannot loosen its own test — and `rank` the drawing's own — `None` where ranks are
/// not read (past the numeric limit a block that holds is reported a theorem, as a lone claim
/// is).
///
/// The witness solve is a compile of its own, since the witnesses' unknowns are no column of
/// `sys`: a copy holding everything else, with only the block's rows.  A compile calls
/// `locus::forget`, so a document that traces as well pays its contacts' marches again once
/// per block — paid only by a document that writes one.
pub(super) fn judge(
    sk: &mut Sketch,
    sys: &mut System,
    w: u32,
    tol: f64,
    extent: f64,
    rank: Option<usize>,
) -> Verdict {
    // the block's witnesses' numbers that its rows read: each a number the drawing holds
    let owned: BTreeSet<u32> = sk.roles.iter()
        .filter(|(_, r)| r.witness == Some(w))
        .flat_map(|(&e, _)| sk.own_params(e))
        .collect();
    let mut unknowns: Vec<u32> = sk.constraints.iter()
        .filter(|c| c.witness == Some(w))
        .flat_map(|c| c.params(sk))
        .filter(|p| owned.contains(p))
        .collect();
    unknowns.sort_unstable();
    unknowns.dedup();
    // what each number is to a size: the scratch copy below has the drawing's own
    let sizes = Sizes::of(sk);
    if unknowns.is_empty() {
        let mine = sk.constraints.iter().filter(|c| c.witness == Some(w));
        let x = sk.get_x();
        if !mine.clone().all(|c| claim_holds(sk, &x, c, tol, extent, &sizes)) {
            return Verdict::Violated;
        }
    } else {
        // the copy is made without the drawing's rows, which it would only throw away: the
        // block's rows stated, and the witnesses' own intrinsic rows with them
        let all = std::mem::take(&mut sk.constraints);
        let mut scratch = sk.clone();
        sk.constraints = all;
        let reads = |c: &Constraint| c.params(sk).iter().any(|p| unknowns.binary_search(p).is_ok());
        scratch.constraints = sk.constraints.iter()
            .filter(|c| c.witness == Some(w) || (c.intrinsic && reads(c)))
            .map(|c| Constraint { claim: false, witness: None, ..c.clone() })
            .collect();
        for (i, p) in scratch.params.iter_mut().enumerate() {
            p.fixed = unknowns.binary_search(&(i as u32)).is_err();
        }
        System::new(&scratch).solve(&mut scratch, SolveOpts::default());
        let x = scratch.get_x();
        if !scratch.constraints.iter().all(|c| claim_holds(&scratch, &x, c, tol, extent, &sizes)) {
            return Verdict::Violated;
        }
        for &p in &unknowns {
            sk.params[p as usize].value = scratch.params[p as usize].value;
        }
    }
    let Some(rank_m) = rank else { return Verdict::Theorem };
    let z = sys.z0(sk);
    let block: Vec<&Constraint> = sk.constraints.iter().filter(|c| c.witness == Some(w)).collect();
    let (jc, _) = sys.conditioned_over(sk, &z, &block, &unknowns);
    // the block's rows stand after the drawing's, its witnesses' columns after the free ones
    let n = jc.rows() - block.iter().map(|c| c.rows_in(sk)).sum::<usize>();
    let mine: Vec<usize> = (n..jc.rows()).collect();
    let cols: Vec<usize> = (sys.n_free..jc.cols()).collect();
    let rank_w = jc.select(&mine, &cols).rank_rrqr(RANK_TOL);
    if jc.rank_rrqr(RANK_TOL) > rank_m + rank_w {
        Verdict::Consuming
    } else {
        Verdict::Theorem
    }
}
