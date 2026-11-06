//! **Claims judged (Solvent §9.7)**, against the drawing the rest of the document made.
//!
//! A claim is no equation, so nothing else the diagnosis reads has seen one: the system, the
//! matching, the ranks and every set are exactly what they would be with the claims deleted —
//! which is the contract, since a claim states only that deleting it changes nothing.  Judging one
//! takes a second compile that *does* carry its rows (`System::with_claims`), read three ways: its
//! residual (does it hold?), and the rank of the other rows with and without it (does the drawing
//! already say it, or would enforcing it have taken a freedom?).  All of it is behind the
//! emptiness test, so a document with no claim in it pays nothing.
//!
//! It is asked of `sys` — the system already compiled, already warm — through
//! `conditioned_with`, and never of a second `System` built over the claims.  A compile is what
//! invalidates the remembered trace poses (it calls `locus::forget`, since only a recompile can
//! put another contact at a given address), so a second system beside a live one throws that
//! one's poses away and every contact re-walks its march from the home: on `peaucellier`, a
//! traced document that ends on a claim, 834 µs a diagnosis against 45 µs for the whole of the
//! rest of it.  The residual is `Constraint::error`, which needs no system at all.

use super::*;

/// Every claim's verdict, plain claims first and then each `claim { … }` block's rows filed
/// under its one verdict.
#[derive(Default)]
pub(super) struct Claims {
    pub theorem: Vec<u32>,
    pub violated: Vec<u32>,
    pub consuming: Vec<u32>,
}

/// Whether a claim's row holds (§9.7): the rule `violated_constraints` states against a compiled
/// system, in the units the row carries — the residual over its own size to its kernel's power
/// (`Sizes`, #492), no larger than `extent`, never a NaN.  `x` is the drawing's numbers, read
/// once by the caller for every claim it asks of.
pub(super) fn claim_holds(
    sk: &Sketch,
    x: &[f64],
    c: &Constraint,
    tol: f64,
    extent: f64,
    sizes: &Sizes,
) -> bool {
    let e = c.error(sk);
    let size = sizes.size(x, c.params(sk).into_iter().map(|p| p as usize), extent);
    !e.is_nan() && e <= tol * size.dpowi(c.kernel_in(sk).degree as i32)
}

/// Judge every claim of `sk` at the pose `sys` was compiled for.  `numeric` is whether ranks are
/// read at all (the diagnosis's own gate): past it a claim that holds is reported a theorem.
/// None is judged on a pose no solve settled at (`settled` false), which may not even satisfy
/// the claim's premises (#449).
pub(super) fn judge(
    sk: &mut Sketch,
    sys: &mut System,
    tol: f64,
    numeric: bool,
    settled: bool,
) -> Claims {
    let mut out = Claims::default();
    if !settled {
        return out;
    }
    plain(sk, sys, tol, numeric, &mut out);
    blocks(sk, sys, tol, numeric, &mut out);
    out
}

/// The claims stated one at a time.  A claim with witnesses of its own is judged by its block
/// (`blocks`).
fn plain(sk: &Sketch, sys: &mut System, tol: f64, numeric: bool, out: &mut Claims) {
    let claims: Vec<&Constraint> =
        sk.constraints.iter().filter(|c| c.claim && c.witness.is_none()).collect();
    if claims.is_empty() {
        return;
    }
    // "does it hold?" is the same rule `violated_constraints` states against a compiled
    // system, in the units the row carries: the residual over its own size^degree
    let (extent, sizes, x) = (sk.extent(), Sizes::of(sk), sk.get_x());
    let held: BTreeSet<u32> = claims
        .iter()
        .filter(|c| claim_holds(sk, &x, c, tol, extent, &sizes))
        .map(|c| c.id)
        .collect();
    // The rank question is the numeric cross-check's kind of question, and is gated the same
    // way; past the limit a claim that holds is reported a theorem, under the same
    // `numeric_skipped` flag the rest of the numeric reading carries.  Judged against the
    // claim-free base rather than through `removable_constraints`, which measures each row
    // against every other row there is: §9.7 asks what the *rest of the document* implies,
    // so a claim two other claims imply is still consuming.
    // no `n_res` test: a base with no rows at all is a real case (a lone grounded point), and
    // a claim over it is exactly the one that adds rank
    let judged = (numeric && sys.n_free > 0).then(|| {
        let z = sys.z0(sk);
        let (jc, row_c) = sys.conditioned_with(sk, &z, &claims);
        let claimed: BTreeSet<u32> = claims.iter().map(|c| c.id).collect();
        let base: Vec<usize> =
            (0..jc.rows()).filter(|&r| !claimed.contains(&row_c[r])).collect();
        let rank = jc.select_rows(&base).rank_rrqr(RANK_TOL);
        (jc, row_c, base, rank)
    });
    // One rank over every row answers the whole question when the answer is "none of them" —
    // which is the case a claim is written for.  Only when the claims together do add rank
    // does it cost a factorisation each to say which.
    let any_adds = judged
        .as_ref()
        .is_some_and(|(jc, _, _, rank)| jc.rank_rrqr(RANK_TOL) > *rank);
    let mut rows: Vec<usize> = Vec::new();
    for c in &claims {
        if !held.contains(&c.id) {
            out.violated.push(c.id);
            continue;
        }
        let consuming = any_adds
            && judged.as_ref().is_some_and(|(jc, row_c, base, rank)| {
                rows.clear();
                rows.extend_from_slice(base);
                rows.extend((0..jc.rows()).filter(|&r| row_c[r] == c.id));
                jc.select_rows(&rows).rank_rrqr(RANK_TOL) > *rank
            });
        if consuming {
            out.consuming.push(c.id);
        } else {
            out.theorem.push(c.id);
        }
    }
}

/// A `claim { … }` block (#460): its witnesses solved after the drawing, the drawing held, and
/// the block judged as one claim — every statement of it filed under the one verdict.
fn blocks(sk: &mut Sketch, sys: &mut System, tol: f64, numeric: bool, out: &mut Claims) {
    let blocks = witnessed::blocks(sk);
    if blocks.is_empty() {
        return;
    }
    // read once for every block: the drawing's extent before any witness moves, and its own
    // rank, which no block's solve changes (a witness is no column of `sys`)
    let extent = sk.extent();
    let rank = numeric.then(|| match sys.n_free {
        0 => 0,
        _ => {
            let z = sys.z0(sk);
            sys.conditioned(&z).rank_rrqr(RANK_TOL)
        }
    });
    for (w, rows) in blocks {
        let into = match witnessed::judge(sk, sys, w, tol, extent, rank) {
            witnessed::Verdict::Theorem => &mut out.theorem,
            witnessed::Verdict::Violated => &mut out.violated,
            witnessed::Verdict::Consuming => &mut out.consuming,
        };
        into.extend(rows);
    }
}
