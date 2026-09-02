//! Stage 2 — structural constraint diagnosis.
//!
//! Turns "solver failed" into "these constraints conflict / this entity has 2 DOF":
//!
//! * bipartite equations-vs-free-parameters graph (`System::structure`), maximum matching
//!   (Hopcroft–Karp) → structural rank; Dulmage–Mendelsohn → over-determined (structurally
//!   redundant equations), under-determined (structurally free parameters) and well-determined
//!   parts; per connected component DOF bookkeeping;
//! * (2,3) pebble game on the point-distance subgraph → rigid clusters and redundant distances;
//! * minimal conflict set (deletion filter) when the solve is infeasible;
//! * structural vs numeric rank cross-check: everything above is structural and cannot see
//!   theorem-induced dependencies; when the Jacobian rank is lower than the matching we log it —
//!   that residue is Stage 4's motivation.

use crate::cgraph::coincident_classes;
use crate::constraints::{same_constraint, CKind, Constraint};
use crate::graph;
use crate::linalg::{rank_and_nullspace, Mat};
use crate::system::RANK_TOL;
use crate::model::{EntRef, Sketch};
use crate::newton::Method;
use crate::solve::SolveOpts;
use crate::system::System;
use crate::witness::{analyze_with, movable_columns, screen, shaky_warning, WitnessReport};
use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum State {
    Well,
    Under,
    Over,
    Conflict,
    /// Constraints are unsatisfied at a pose that is not stationary: the solve stopped short,
    /// and nothing is known about the geometry there — not a conflict, which is a verdict.  A
    /// *status* only: an entity keeps the determination the rank gives it.
    Unsolved,
}

impl State {
    pub fn as_str(self) -> &'static str {
        match self {
            State::Well => "well",
            State::Under => "under",
            State::Over => "over",
            State::Conflict => "conflict",
            State::Unsolved => "unsolved",
        }
    }
}

/// Free parameters up to which the automatic numeric cross-check (a dense SVD) runs.
pub const NUMERIC_MAX: usize = 300;

/// A null-space entry below this is zero: the parameter does not move, the row takes no part.
const RTOL: f64 = 1e-8;

/// A connected component of the constraint graph with its own DOF accounting.
#[derive(Clone, Debug)]
pub struct Component {
    pub params: Vec<u32>,
    pub constraints: Vec<u32>,
    pub structural_rank: usize,
    pub dof: i64,
}

#[derive(Clone, Debug)]
pub struct Diagnosis {
    /// Free parameters.
    pub n_params: usize,
    /// Hard residual rows.
    pub n_equations: usize,
    /// Maximum matching size.
    pub structural_rank: usize,
    /// Jacobian rank at the current configuration.
    /// Jacobian rank at the current configuration, second order included — `shaky` is already
    /// counted in it, so no consumer adds anything back.
    pub numeric_rank: Option<usize>,
    /// The numeric cross-check was skipped because the system is past the dense limit.
    pub numeric_skipped: bool,
    /// How many dependencies only the numbers can see (0 when the check did not run).
    pub geometric_dependency: usize,
    /// How many of `numeric_rank` the settle test contributed: first-order motions blocked at
    /// second order — a tangency evaluated at its own contact point (a double root: the contact
    /// "swims" along the line to first order and is pulled back at second).  Reported so the
    /// correction is visible; never added to anything, since the rank already carries it.  The
    /// term of art is a *shaky* framework — infinitesimally flexible, rigid.
    pub shaky: usize,
    /// "Remove one of these."  Where the numeric cross-check saw the dependency, the constraints
    /// wholly implied by one that involves a dimension.  The structural over-block stands in
    /// only where no numeric rank was computed: at DOF 0 that block is every row touching the
    /// cluster, and would name a determined figure's one dimension for a closure theorem.
    pub over: Vec<u32>,
    /// Constraints wholly implied by a dependency among pure relations — a theorem (the altitudes
    /// concur) rather than a surplus.  Consistent on every solution, nothing to fix; each could be
    /// deleted without changing the sketch, but none has to be.
    pub implied: Vec<u32>,
    /// The `claim` statements, judged.  A claim is no equation — it joins none of the counts or
    /// sets above, and can never make the sketch Over or Conflict — so its whole report is which
    /// of these three lists it landed in.  *Theorem*: it holds, and its rows add no rank — the
    /// drawing already says it.  *Violated*: it does not hold at this solution.  *Consuming*: it
    /// holds here, but enforcing it would have taken a freedom — satisfied by the pose, not by
    /// the document, which is a claim that claims too much.
    pub claims_theorem: Vec<u32>,
    pub claims_violated: Vec<u32>,
    pub claims_consuming: Vec<u32>,
    /// Parameters that can move *at the configuration diagnosed*.
    pub under_params: Vec<u32>,
    pub structural_under_params: Vec<u32>,
    pub components: Vec<Component>,
    pub entity_state: BTreeMap<EntRef, State>,
    /// From the pebble game on the distance graph.
    pub rigid_clusters: Vec<Vec<usize>>,
    pub redundant_distances: Vec<u32>,
    pub violated: Vec<u32>,
    /// Minimal conflict set.
    pub conflicts: Option<Vec<u32>>,
    pub warnings: Vec<String>,
    /// Stage 4 analysis, on demand.
    pub witness: Option<WitnessReport>,
    /// Degrees of freedom left at the current configuration — what can still be dragged.
    pub dof: i64,
    /// DOF the matching alone sees.
    pub structural_dof: i64,
    pub n_redundant: i64,
    pub structural_n_redundant: i64,
    pub status: State,
}

pub fn summary(d: &Diagnosis) -> String {
    let mut parts = vec![
        format!(
            "{} params, {} equations, structural rank {}",
            d.n_params, d.n_equations, d.structural_rank
        ),
        format!("DOF {}", d.dof),
    ];
    if d.dof != d.structural_dof {
        parts.push(format!(
            "the matching alone would say DOF {} — {} equation(s) carry no information",
            d.structural_dof, d.geometric_dependency
        ));
    }
    if d.n_redundant != 0 && !d.over.is_empty() {
        parts.push(format!(
            "{} redundant equation(s) among {} constraint(s)",
            d.n_redundant,
            d.over.len()
        ));
    }
    if d.geometric_dependency > 0 {
        parts.push(format!(
            "numeric rank {} < structural {}: {} geometric (theorem-type) dependency",
            d.numeric_rank.unwrap_or(0),
            d.structural_rank,
            d.geometric_dependency
        ));
    }
    if d.shaky > 0 {
        parts.push(format!(
            "{} motion(s) blocked at second order (a tangency at its contact) — not DOF",
            d.shaky
        ));
    }
    if !d.implied.is_empty() {
        parts.push(format!(
            "{} constraint(s) implied by the others (a relation-only dependency: consistent, \
             nothing to fix)",
            d.implied.len()
        ));
    }
    if d.status == State::Unsolved {
        parts.push(format!(
            "UNSOLVED — {} constraint(s) unsatisfied where the solve stopped short of a \
             stationary point; no verdict on them",
            d.violated.len()
        ));
    } else if d.conflicts.as_ref().map(|c| !c.is_empty()).unwrap_or(false) {
        parts.push("CONFLICT — remove one of the listed constraints".to_string());
    } else if !d.violated.is_empty() {
        parts.push(format!("{} constraint(s) violated", d.violated.len()));
    }
    if d.components.len() > 1 {
        // paged like every other list here: `repeat 100000` is a document somebody can write,
        // and a hundred thousand DOFs on one line is not a summary (#43.18)
        const SHOW: usize = 12;
        let dofs: Vec<String> =
            d.components.iter().take(SHOW).map(|c| c.dof.to_string()).collect();
        let more = d.components.len().saturating_sub(SHOW);
        parts.push(format!(
            "{} components: DOF {}{}",
            d.components.len(),
            dofs.join(", "),
            if more > 0 { format!(", … and {more} more") } else { String::new() }
        ));
    }
    if !d.rigid_clusters.is_empty() {
        parts.push(format!("{} rigid cluster(s) in the distance graph", d.rigid_clusters.len()));
    }
    parts.join("; ")
}

/// Constraints that could be deleted without losing any information, given `w`, a basis of the
/// left null space of the Jacobian (one column per dependency).
///
/// With W orthonormal, dropping a set of rows R gives
/// `rank(J minus R) = rank(J) - |R| + rank(W[R])`, so a constraint is free to delete exactly when
/// its own rows are *independent* in W.  That distinction is the whole point: an arc whose
/// endpoints are mirrored about a line through its centre makes one of `Symmetric`'s two residuals
/// implied by the arc's radius equations, but `Symmetric` still carries the perpendicularity — its
/// two rows are dependent in W, it is doing real work, and telling the user to remove it would be
/// wrong.  Only a wholly implied constraint is worth naming.
///
/// Intrinsic constraints are skipped: they come with the primitive and cannot be deleted.
pub fn removable_constraints(sk: &Sketch, w: &Mat, row_c: &[u32], rtol: f64) -> Vec<u32> {
    if w.rows == 0 || w.cols == 0 {
        return Vec::new();
    }
    let mut rows: Vec<(u32, Vec<usize>)> = Vec::new();
    let mut at: BTreeMap<u32, usize> = BTreeMap::new(); // constraint -> its slot in `rows`
    for (r, &c) in row_c.iter().enumerate() {
        match at.get(&c) {
            Some(&i) => rows[i].1.push(r),
            None => {
                at.insert(c, rows.len());
                rows.push((c, vec![r]));
            }
        }
    }
    let intrinsic: BTreeSet<u32> =
        sk.constraints.iter().filter(|c| c.intrinsic).map(|c| c.id).collect();
    let mut out = Vec::new();
    for (cid, rs) in rows {
        if intrinsic.contains(&cid) {
            continue;
        }
        // W is orthonormal, so both tests here are absolute and fair to every row alike
        let sub = w.select_rows(&rs);
        let peak = crate::linalg::absmax(&sub.data);
        if peak <= rtol {
            continue;
        }
        if rank_and_nullspace(&sub, rtol).rank == rs.len() {
            out.push(cid);
        }
    }
    out
}

/// Hard constraints whose residual is not (numerically) zero at the current configuration.
/// Each is judged in its own kernel's units — a radius error is a length, a distance error a
/// length squared, and one absolute threshold for both calls half of them satisfied — which is
/// how `constraint_errors` already reports them (`System::row_scale`).
pub fn violated_constraints(sk: &Sketch, sys: &mut System, tol: f64) -> Vec<u32> {
    let z = sys.z0(sk);
    let err = sys.constraint_errors(&z);
    // one pass to collect the ids to leave out, rather than a linear scan of the constraint
    // list per constraint — this runs after every edit.  A soft row is a drag's, and an
    // intrinsic row is an entity's own definition: neither is a statement the document made,
    // so neither is named as one it could remove.
    let soft: BTreeSet<u32> =
        sk.constraints.iter().filter(|c| c.soft || c.intrinsic).map(|c| c.id).collect();
    let mut out = Vec::new();
    for (i, &cid) in sys.cids.iter().enumerate() {
        if !soft.contains(&cid) && (err[i].is_nan() || err[i] > tol) {
            out.push(cid);
        }
    }
    out
}

#[derive(Clone, Copy, Debug)]
pub struct DiagnoseOptions {
    /// `None`: run the numeric cross-check only below `numeric_max`.  `Some`: force it.
    pub numeric: Option<bool>,
    /// `None`: compute the minimal conflict set only when some constraint is violated.
    pub conflicts: Option<bool>,
    pub witness: bool,
    pub tol: f64,
    pub numeric_max: usize,
}

impl Default for DiagnoseOptions {
    fn default() -> DiagnoseOptions {
        DiagnoseOptions {
            numeric: None,
            conflicts: None,
            witness: false,
            tol: 1e-6,
            numeric_max: NUMERIC_MAX,
        }
    }
}

/// Structural (and optionally numeric) diagnosis of a sketch at its current configuration.
pub fn diagnose(sk: &mut Sketch, opts: DiagnoseOptions) -> Diagnosis {
    let mut sys = System::new(sk);
    diagnose_with(sk, &mut sys, opts)
}

pub fn diagnose_with(sk: &mut Sketch, sys: &mut System, opts: DiagnoseOptions) -> Diagnosis {
    // the caller hands us the System it solved with; dimensions may have been edited since, and
    // the residuals we are about to judge are read from the compiled constants
    sys.refresh_consts(sk);
    let (adj, row_c) = sys.structure();
    let n_cols = sys.n_free;
    let dm = graph::dulmage_mendelsohn(&adj, n_cols);
    let free_params: Vec<u32> = sys.free.iter().map(|&i| i as u32).collect();

    let mut over_set: BTreeSet<u32> = BTreeSet::new();
    let mut over: Vec<u32> = Vec::new();
    let mut implied: Vec<u32> = Vec::new();
    // an entity's own intrinsic rows — the two that make an arc's ends its ends — are not in
    // the document and cannot be removed, so "remove one of these" never names one (#43.17);
    // the numeric path (`removable_constraints`) already leaves them out
    let intrinsic: BTreeSet<u32> =
        sk.constraints.iter().filter(|c| c.intrinsic).map(|c| c.id).collect();
    for &r in &dm.over_rows {
        let c = row_c[r];
        if !intrinsic.contains(&c) && over_set.insert(c) {
            over.push(c);
        }
    }
    // the matching's reading, kept: it is where a redundancy *must* live, so the conflict
    // search below seeds its candidates from it whatever W says about who is removable
    let structural_over = over.clone();
    let structural_under: Vec<u32> = dm.under_cols.iter().map(|&j| free_params[j]).collect();
    let mut under_params = structural_under.clone();

    // -- components --
    let comps = graph::bipartite_components(&adj, n_cols);
    let n_comp = comps.count;
    let mut comp_params: Vec<Vec<u32>> = vec![Vec::new(); n_comp];
    let mut comp_cs: Vec<Vec<u32>> = vec![Vec::new(); n_comp];
    let mut comp_rank = vec![0usize; n_comp];
    for j in 0..n_cols {
        comp_params[comps.comp_col[j]].push(free_params[j]);
        if dm.mate_col[j] >= 0 {
            comp_rank[comps.comp_col[j]] += 1;
        }
    }
    let mut comp_seen: Vec<BTreeSet<u32>> = vec![BTreeSet::new(); n_comp];
    for r in 0..adj.len() {
        let (c, k) = (row_c[r], comps.comp_row[r]);
        if comp_seen[k].insert(c) {
            comp_cs[k].push(c); // in row order, deduplicated in constant time
        }
    }
    let mut components: Vec<Component> = (0..n_comp)
        .map(|i| Component {
            dof: comp_params[i].len() as i64 - comp_rank[i] as i64,
            params: std::mem::take(&mut comp_params[i]),
            constraints: std::mem::take(&mut comp_cs[i]),
            structural_rank: comp_rank[i],
        })
        .collect();
    components.sort_by_key(|c| std::cmp::Reverse(c.params.len()));

    // -- witness analysis (Stage 4), on demand --
    let mut warnings: Vec<String> = Vec::new();
    let mut wit: Option<WitnessReport> = None;
    if opts.witness && n_cols > 0 && sys.n_res > 0 {
        wit = Some(analyze_with(sk, sys, None, &over_set, RANK_TOL, 0));
    }

    // -- numeric cross-check: rank and the parameters that can actually move --
    let mut numeric_rank: Option<usize> = None;
    let mut shaky = 0usize;
    let want_numeric = opts.numeric.unwrap_or(n_cols <= opts.numeric_max);
    let numeric_skipped = opts.numeric.is_none() && !want_numeric;
    if numeric_skipped {
        warnings.push(format!(
            "numeric cross-check skipped: {n_cols} free parameters is above the dense limit \
             ({}) — the diagnosis below is structural only",
            opts.numeric_max
        ));
    }
    if want_numeric && n_cols > 0 && sys.n_res > 0 {
        let movable: Vec<usize>;
        // the conditioned Jacobian at z, kept for the over/implied block below rather than
        // built a second time at the same pose — the screen leaves both sketch and system as
        // it found them, so it is still the right matrix afterwards
        let mut cond: Option<crate::system::Conditioned> = None;
        match &wit {
            Some(w) if w.used_current => {
                numeric_rank = Some(w.numeric_rank); // same J at the same x, screened already
                movable = w.movable.clone();
                shaky = w.blocked;
            }
            _ => {
                let z = sys.z0(sk);
                let c = sys.conditioned(&z);
                let rn = c.rank_and_nullspace(RANK_TOL);
                cond = Some(c);
                if rn.converged {
                    // where the Jacobian claims motions the matching cannot account for, settle-
                    // test them: a tangency at its own contact is a double root whose "motion"
                    // walks back.  `screen` owns both guards — only at a solution, and never
                    // more than the discrepancy, so a motion the matching also sees is safe.
                    let deficit = dm.rank.saturating_sub(rn.rank);
                    let (null, blocked) = screen(sk, sys, &z, rn.null(), deficit);
                    shaky = blocked;
                    numeric_rank = Some(rn.rank + blocked);
                    if blocked > 0 {
                        warnings.push(shaky_warning(blocked));
                    }
                    movable = movable_columns(&null, RTOL);
                } else {
                    // no rank at all, rather than the zero a failed SVD leaves behind — which
                    // reads as "every constraint is redundant"
                    warnings.push(
                        "numeric rank unavailable: the SVD did not converge (a degenerate or \
                         non-finite Jacobian) — the structural analysis stands alone"
                            .to_string(),
                    );
                    movable = Vec::new();
                }
            }
        }
        // Which parameters can actually move: rows of the null space that are nonzero.  Sharper
        // than the DM under-block (which counts a parameter as free if it could be in some generic
        // assignment); evaluated at the current configuration.
        if numeric_rank.is_some() {
            under_params = movable.iter().map(|&j| free_params[j]).collect();
        }
        if numeric_rank.is_some_and(|r| r < adj.len()) {
            // ...and name the constraints worth removing, or the report would say
            // "over-constrained" with nothing to point at.  One extra SVD, only where something
            // is redundant at all — whether the matching saw it or only the Jacobian does.  Run
            // only on the second case, a determined square with a stated side had every one of
            // its ten constraints `over`, the side length first: the matching's over-block at
            // DOF 0 is every row touching the cluster, where W's reading is the relation-only
            // closure it always was, which is `implied` and nothing to fix (#45.5).
            let j = cond.unwrap_or_else(|| {
                let z = sys.z0(sk);
                sys.conditioned(&z)
            });
            let w = j.left_nullspace(RANK_TOL).null();
            // `implied` is what the relations alone already say: the same test on the left null
            // space of the relation-only rows (a dependency there embeds in W, so this is a subset
            // of the removable set).  Whatever is removable only with a dimension's help is `over`.
            // The static "carries a dimension" test stands in for the witness criterion (the
            // deficiency survives jittered dimensions) because this runs after every edit.  An
            // exact duplicate is never a theorem: two Horizontals on one line match two variables,
            // so the graph passes them, but a copy is a surplus whatever it is made of.
            let rel_rows: Vec<usize> = (0..j.rows())
                .filter(|&r| sk.constraint(row_c[r]).is_some_and(|c| !c.kind.has_dimension()))
                .collect();
            let w_rel = if rel_rows.len() == j.rows() {
                w.clone()
            } else {
                j.select_rows(&rel_rows).left_nullspace(RANK_TOL).null()
            };
            let row_c_rel: Vec<u32> = rel_rows.iter().map(|&r| row_c[r]).collect();
            // `same_constraint` compares what is said, not whether it is said: a claim restating
            // a relation matches it exactly.  Counted as a duplicate it would move that relation
            // out of `implied` and into `over` — a claim making the sketch over-constrained,
            // which is the one thing §9.7 promises cannot happen.  Only what acts can duplicate.
            let duplicated = |c: u32| {
                sk.constraint(c).is_some_and(|m| {
                    sk.constraints.iter().any(|o| o.acts() && o.id != c && same_constraint(o, m))
                })
            };
            implied = removable_constraints(sk, &w_rel, &row_c_rel, RTOL)
                .into_iter()
                .filter(|&c| !duplicated(c))
                .collect();
            // W is every dependency at this configuration, so it outranks the matching here
            // exactly as the numeric rank does below: the structural over-block is generic, and
            // where a theorem tips the count it blames a whole block — a rectangle with three
            // surplus perpendiculars indicts its two side lengths.  Rebuild `over` from W's
            // reading alone; the structural seeds stand only where no W was computed.
            over.clear();
            over_set.clear();
            for c in removable_constraints(sk, &w, &row_c, RTOL) {
                if !implied.contains(&c) && over_set.insert(c) {
                    over.push(c);
                }
            }
            if numeric_rank.is_some_and(|r| r < dm.rank) {
                warnings.push(format!(
                    "structural rank {} but numeric rank {}: a dependency the graph cannot see \
                     (theorem-induced or degenerate configuration) — Stage 4",
                    dm.rank,
                    numeric_rank.unwrap()
                ));
            }
        }
    }

    // -- violated --
    let violated = violated_constraints(sk, sys, opts.tol);
    // A violated row is evidence about the *geometry* only at a stationary point — where the
    // solver could go no further, what is left is what the constraints cannot agree on.
    // Anywhere else it is evidence about the solve, and the diagnosis finds out which it is for
    // itself rather than guessing: a scratch copy is solved.  If that solves, the constraints
    // are consistent and the caller's pose is merely *unsolved* — no conflict, no culprits.
    // If it does not, the search for the minimal conflict set is the last word: it proves a
    // conflict by finding the subset it cannot add to a feasible pose, and an empty answer
    // from a pose that was never stationary means it found every constraint feasible after
    // all.  The search runs from the caller's pose, not the scratch one — a failed solve of
    // the whole system has pulled everything toward the impossible row, and the trials
    // warm-start better from where the drawing was.  Read as a conflict directly, a solve that ran out on a consistent four-bar named
    // three innocent statements to delete (issue #43).  The extra solve is paid only on this
    // path, which the app is rarely on (it solves before it diagnoses).
    let mut stalled = false;
    let mut unsettled = false;
    if !violated.is_empty() {
        let z = sys.z0(sk);
        if !sys.stationary(&z) {
            let mut scratch = sk.clone();
            let mut ssys = System::new(&scratch);
            if ssys.solve(&mut scratch, SolveOpts::default()).success {
                stalled = true;
            } else {
                unsettled = true;
            }
        }
    }

    // -- claims (Solvent §9.7), judged against the drawing the rest of the document made --
    //
    // A claim is no equation, so nothing above has seen one: the system, the matching, the ranks
    // and every set are exactly what they would be with the claims deleted — which is the
    // contract, since a claim states only that deleting it changes nothing.  Judging one takes a
    // second compile that *does* carry its rows (`System::with_claims`), read three ways: its
    // residual (does it hold?), and the rank of the other rows with and without it (does the
    // drawing already say it, or would enforcing it have taken a freedom?).  All of it is behind
    // the emptiness test, so a document with no claim in it pays nothing.
    //
    // It is asked of `sys` — the system already compiled, already warm — through
    // `conditioned_with`, and never of a second `System` built over the claims.  A compile is
    // what invalidates the remembered trace poses (it calls `locus::forget`, since only a
    // recompile can put another contact at a given address), so a second system beside a live one
    // throws that one's poses away and every contact re-walks its march from the home: on
    // `peaucellier`, a traced document that ends on a claim, 834 µs a diagnosis against 45 µs for
    // the whole of the rest of it.  The residual is `Constraint::error`, which needs no system at
    // all.
    let mut claims_theorem: Vec<u32> = Vec::new();
    let mut claims_violated: Vec<u32> = Vec::new();
    let mut claims_consuming: Vec<u32> = Vec::new();
    let claims: Vec<&Constraint> = sk.constraints.iter().filter(|c| c.claim).collect();
    if !claims.is_empty() {
        // "does it hold?" is the same rule `violated_constraints` states against a compiled
        // system, in the units the row carries: the residual over `extent^degree`
        let held: BTreeSet<u32> = claims
            .iter()
            .filter(|c| {
                let e = c.error(sk);
                let deg = crate::kernels::kernel(c.kind.kernel()).degree as i32;
                !e.is_nan() && e <= opts.tol * sk.extent().max(1.0).powi(deg)
            })
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
        let judged = (want_numeric && sys.n_free > 0).then(|| {
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
                claims_violated.push(c.id);
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
                claims_consuming.push(c.id);
            } else {
                claims_theorem.push(c.id);
            }
        }
    }

    // -- conflicts --
    let mut conflict_set: Option<Vec<u32>> = None;
    if !stalled && opts.conflicts.unwrap_or(!violated.is_empty()) {
        // Candidates = the structurally over-determined block (where a redundancy must live)
        // *together with* whatever is actually violated.  Not just the over-block: a consistent
        // duplicate (two Horizontals on one line) makes an over-block with nothing to do with an
        // infeasibility elsewhere, and confining the search to it pins the blame on the harmless
        // pair — the real conflict is then in the constraints held fixed, so no candidate can be
        // satisfied and the first one tried wins.  Everything else stays fixed, so the result is
        // minimal "among the suspects", and the filter costs |candidates| solves, not |all|.
        let mut cands = structural_over;
        for &c in over.iter().chain(&violated) {
            if !cands.contains(&c) {
                cands.push(c);
            }
        }
        let set = minimal_conflict_set(sk, Some(&cands), opts.tol, Method::DogLeg, 60);
        // the search found every constraint feasible from a pose that was never stationary:
        // consistent, and merely unsolved
        stalled = set.is_empty() && unsettled;
        // an intrinsic row may be *in* the set the search proves — the radius an arc's ends fix
        // is what a stray dimension on it contradicts — but it is nobody's statement and
        // nothing anyone can remove, so the culprits named are the document's own (#45.6)
        conflict_set = Some(set.into_iter().filter(|c| !intrinsic.contains(c)).collect());
    }
    // a dependency whose rows do not hold is no theorem: two tangencies of one pair that
    // cannot both be true are dependent in W and carry no dimension, so W's reading files them
    // as `implied` — "consistent, nothing to fix" — beside the conflict that names them.  What
    // is violated, or found in the conflict set, is a surplus and is said so.
    let contested = |c: &u32| {
        violated.contains(c) || conflict_set.as_ref().is_some_and(|s| s.contains(c))
    };
    if implied.iter().any(contested) {
        let (moved, kept): (Vec<u32>, Vec<u32>) = implied.iter().copied().partition(|c| contested(c));
        implied = kept;
        for c in moved {
            if over_set.insert(c) {
                over.push(c);
            }
        }
    }

    // -- pebble game on the point-distance graph --
    let (clusters, redundant) = distance_rigidity(sk);

    // -- entity states --
    let under_set: BTreeSet<u32> = under_params.iter().copied().collect();
    let conflict_ids: BTreeSet<u32> = match &conflict_set {
        Some(c) => c.iter().copied().collect(),
        // stalled, the violated rows are no verdict on anything they touch: an entity keeps
        // the determination the rank gives it, which holds at a non-solution as well
        None if stalled => BTreeSet::new(),
        None => violated.iter().copied().collect(),
    };
    let mut touched: BTreeMap<EntRef, Vec<u32>> = BTreeMap::new();
    for c in sk.hard_constraints() {
        for e in c.entities() {
            touched.entry(e).or_default().push(c.id);
            for ch in sk.children(e) {
                touched.entry(ch).or_default().push(c.id);
            }
        }
    }
    let ents = sk.primitives();
    let mut state: BTreeMap<EntRef, State> = BTreeMap::new();
    for &e in &ents {
        let empty = Vec::new();
        let cs = touched.get(&e).unwrap_or(&empty);
        let st = if cs.iter().any(|c| conflict_ids.contains(c)) {
            State::Conflict
        } else if cs.iter().any(|c| over_set.contains(c)) {
            State::Over
        } else if sk.entity_params(e).iter().any(|p| under_set.contains(p)) {
            State::Under
        } else {
            State::Well
        };
        state.insert(e, st);
    }
    for &e in &ents {
        for ch in sk.children(e) {
            let cs = state[&ch];
            if cs > state[&e] {
                state.insert(e, cs);
            }
        }
    }
    if let Some(w) = &wit {
        warnings.extend(w.warnings.iter().cloned());
    }

    // the structural matching is a *generic* upper bound on the rank; where the numeric
    // cross-check ran it is the truth at this configuration, and it is what decides what moves
    let effective_rank = numeric_rank.unwrap_or(dm.rank);
    let dof = n_cols as i64 - effective_rank as i64;
    let structural_dof = n_cols as i64 - dm.rank as i64;
    let n_redundant = adj.len() as i64 - effective_rank as i64;
    let structural_n_redundant = adj.len() as i64 - dm.rank as i64;
    // `over` only when something is actually worth removing: a rank deficiency alone is not
    // enough, since a dependency can be shared between a user constraint that is still doing work
    // and a primitive's own definition, leaving nothing to delete — and a relation-only theorem
    // leaves `implied` constraints that *could* go but need not, which is not "over" either
    let status = if stalled {
        State::Unsolved
    } else if conflict_set.as_ref().map(|c| !c.is_empty()).unwrap_or(false)
        || !violated.is_empty()
    {
        State::Conflict
    } else if !over.is_empty() {
        State::Over
    } else if dof > 0 {
        State::Under
    } else {
        State::Well
    };
    Diagnosis {
        n_params: n_cols,
        n_equations: adj.len(),
        structural_rank: dm.rank,
        numeric_rank,
        numeric_skipped,
        geometric_dependency: numeric_rank
            .map(|nr| dm.rank.saturating_sub(nr))
            .unwrap_or(0),
        shaky,
        over,
        implied,
        claims_theorem,
        claims_violated,
        claims_consuming,
        under_params,
        structural_under_params: structural_under,
        components,
        entity_state: state,
        rigid_clusters: clusters,
        redundant_distances: redundant,
        violated,
        conflicts: conflict_set,
        warnings,
        witness: wit,
        dof,
        structural_dof,
        n_redundant,
        structural_n_redundant,
        status,
    }
}

/// Minimal infeasible subset among `candidates` (default: all hard constraints); the rest stay in
/// the system throughout.  "Remove one of these."
///
/// Grow-then-shrink: add candidates one at a time, each solve warm-started from the previous
/// feasible configuration, until one breaks feasibility (it is in the conflict); then delete the
/// earlier ones one at a time, keeping a deletion whenever the rest is still infeasible.
/// Warm-starting from feasible states is what makes the trials reliable.
pub fn minimal_conflict_set(
    sk: &mut Sketch,
    candidates: Option<&[u32]>,
    tol: f64,
    method: Method,
    max_iter: i32,
) -> Vec<u32> {
    let x0 = sk.get_x();
    let hard: Vec<u32> = sk.hard_ids();
    let hard_set: BTreeSet<u32> = hard.iter().copied().collect();
    let cands: Vec<u32> = match candidates {
        Some(c) => c.iter().copied().filter(|id| hard_set.contains(id)).collect(),
        None => hard.clone(),
    };
    let cand_set: BTreeSet<u32> = cands.iter().copied().collect();
    let others: Vec<u32> = hard.iter().copied().filter(|c| !cand_set.contains(c)).collect();
    let saved = sk.constraints.clone();

    let solve_with = |sk: &mut Sketch, ids: &[u32], x_start: &[f64]| -> (bool, Vec<f64>) {
        sk.set_x(x_start);
        let keep: BTreeSet<u32> = ids.iter().copied().collect();
        sk.constraints.retain(|c| keep.contains(&c.id));
        let mut sys = System::new(sk);
        sys.solve(sk, SolveOpts { method, max_iter, ..SolveOpts::default() });
        let z = sys.z0(sk);
        let ok = sys.max_relative_residual(&z) <= tol;
        let x = sk.get_x();
        (ok, x)
    };

    let restore = |sk: &mut Sketch, saved: &Vec<crate::constraints::Constraint>| {
        sk.constraints = saved.clone();
    };

    // a state satisfying the non-candidates
    let (ok, xb) = solve_with(sk, &others, &x0);
    restore(sk, &saved);
    if !ok && candidates.is_some() {
        // the restriction does not hold: the conflict is not confined to the candidates, so every
        // trial below would fail on whatever happened to be added first.  Search the whole system.
        sk.set_x(&x0);
        return minimal_conflict_set(sk, None, tol, method, max_iter);
    }
    let x_base = if ok { xb } else { x0.clone() };
    let mut accepted: Vec<u32> = Vec::new();
    let mut x_feas = x_base;
    let mut culprit: Option<u32> = None;
    for &c in &cands {
        let mut trial = others.clone();
        trial.extend(accepted.iter().copied());
        trial.push(c);
        let (good, x) = solve_with(sk, &trial, &x_feas);
        restore(sk, &saved);
        if good {
            accepted.push(c);
            x_feas = x;
        } else {
            culprit = Some(c);
            break;
        }
    }
    let Some(culprit) = culprit else {
        sk.set_x(&x0);
        return Vec::new();
    };
    let mut keep = accepted.clone();
    for &c in &accepted {
        let trial_keep: Vec<u32> = keep.iter().copied().filter(|&k| k != c).collect();
        let mut trial = others.clone();
        trial.extend(trial_keep.iter().copied());
        trial.push(culprit);
        let (good, _) = solve_with(sk, &trial, &x_feas);
        restore(sk, &saved);
        if !good {
            keep = trial_keep;
        }
    }
    sk.set_x(&x0);
    keep.push(culprit);
    keep
}

/// (2,3) pebble game on the point-distance graph: vertices are points with Coincident points
/// merged; edges are Distance constraints.  Returns the rigid clusters (as sets of point indices)
/// and the redundant Distance constraints.
pub fn distance_rigidity(sk: &Sketch) -> (Vec<Vec<usize>>, Vec<u32>) {
    let (of, members) = coincident_classes(sk);
    let edge_c: Vec<&crate::constraints::Constraint> =
        sk.constraints.iter().filter(|c| c.kind == CKind::Distance).collect();
    if edge_c.is_empty() {
        return (Vec::new(), Vec::new());
    }
    let edges: Vec<(usize, usize)> = edge_c
        .iter()
        .map(|c| (of[c.args[0].ent().i()], of[c.args[1].ent().i()]))
        .collect();
    let res = graph::pebble_game(members.len(), &edges);
    let clusters = res
        .components
        .iter()
        .map(|comp| comp.iter().flat_map(|&v| members[v].iter().copied()).collect())
        .collect();
    let redundant = res.redundant.iter().map(|&i| edge_c[i].id).collect();
    (clusters, redundant)
}
