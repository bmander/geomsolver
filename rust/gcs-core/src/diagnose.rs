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
use crate::constraints::CKind;
use crate::graph;
use crate::linalg::{rank_and_nullspace, Mat};
use crate::model::{EntRef, Sketch};
use crate::newton::Method;
use crate::solve::SolveOpts;
use crate::system::System;
use crate::witness::{analyze_with, movable_columns, WitnessReport};
use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum State {
    Well,
    Under,
    Over,
    Conflict,
}

impl State {
    pub fn as_str(self) -> &'static str {
        match self {
            State::Well => "well",
            State::Under => "under",
            State::Over => "over",
            State::Conflict => "conflict",
        }
    }
}

/// Free parameters up to which the automatic numeric cross-check (a dense SVD) runs.
pub const NUMERIC_MAX: usize = 300;

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
    pub numeric_rank: Option<usize>,
    /// The numeric cross-check was skipped because the system is past the dense limit.
    pub numeric_skipped: bool,
    /// How many dependencies only the numbers can see (0 when the check did not run).
    pub geometric_dependency: usize,
    /// Constraints in the over-determined block.
    pub over: Vec<u32>,
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
    if d.n_redundant != 0 {
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
    if d.conflicts.as_ref().map(|c| !c.is_empty()).unwrap_or(false) {
        parts.push("CONFLICT — remove one of the listed constraints".to_string());
    } else if !d.violated.is_empty() {
        parts.push(format!("{} constraint(s) violated", d.violated.len()));
    }
    if d.components.len() > 1 {
        parts.push(format!(
            "{} components: DOF {}",
            d.components.len(),
            d.components.iter().map(|c| c.dof.to_string()).collect::<Vec<_>>().join(", ")
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
    for (r, &c) in row_c.iter().enumerate() {
        match rows.iter_mut().find(|(k, _)| *k == c) {
            Some((_, v)) => v.push(r),
            None => rows.push((c, vec![r])),
        }
    }
    let mut out = Vec::new();
    for (cid, rs) in rows {
        if sk.constraint(cid).map(|c| c.intrinsic).unwrap_or(false) {
            continue;
        }
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
/// Each is judged against its own kernel's units — a radius error is a length, a distance error a
/// length squared, and one absolute threshold for both calls half of them satisfied.
pub fn violated_constraints(sk: &Sketch, sys: &mut System, tol: f64) -> Vec<u32> {
    let z = sys.z0(sk);
    let err = sys.constraint_errors(&z);
    let mut out = Vec::new();
    for (i, &cid) in sys.cids.iter().enumerate() {
        let soft = sk.constraint(cid).map(|c| c.soft).unwrap_or(false);
        if !soft && !(err[i] <= tol * sys.constraint_scale(cid)) {
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
    for &r in &dm.over_rows {
        let c = row_c[r];
        if over_set.insert(c) {
            over.push(c);
        }
    }
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
    for r in 0..adj.len() {
        let c = row_c[r];
        let bucket = &mut comp_cs[comps.comp_row[r]];
        if !bucket.contains(&c) {
            bucket.push(c);
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
        wit = Some(analyze_with(sk, sys, None, &over_set, 1e-9, 0));
    }

    // -- numeric cross-check: rank and the parameters that can actually move --
    let mut numeric_rank: Option<usize> = None;
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
        match &wit {
            Some(w) if w.used_current => {
                numeric_rank = Some(w.numeric_rank); // same J at the same x
                movable = w.movable.clone();
            }
            _ => {
                let z = sys.z0(sk);
                let dense = sys.jacobian_dense(&z);
                let hard_rows = sys.hard_rows();
                let rn = rank_and_nullspace(&dense.select_rows(&hard_rows), 1e-10);
                numeric_rank = Some(n_cols - rn.n.cols);
                movable = movable_columns(&rn.n, 1e-8);
            }
        }
        // Which parameters can actually move: rows of the null space that are nonzero.  Sharper
        // than the DM under-block (which counts a parameter as free if it could be in some generic
        // assignment); evaluated at the current configuration.
        under_params = movable.iter().map(|&j| free_params[j]).collect();
        if numeric_rank.unwrap() < dm.rank {
            // ...and name the constraints worth removing, or the report would say
            // "over-constrained" with nothing to point at.  One extra SVD, only on this path.
            let z = sys.z0(sk);
            let dense = sys.jacobian_dense(&z);
            let hard_rows = sys.hard_rows();
            let w = rank_and_nullspace(&dense.select_rows(&hard_rows).transpose(), 1e-10).n;
            for c in removable_constraints(sk, &w, &row_c, 1e-8) {
                if over_set.insert(c) {
                    over.push(c);
                }
            }
            warnings.push(format!(
                "structural rank {} but numeric rank {}: a dependency the graph cannot see \
                 (theorem-induced or degenerate configuration) — Stage 4",
                dm.rank,
                numeric_rank.unwrap()
            ));
        }
    }

    // -- violated / conflicts --
    let violated = violated_constraints(sk, sys, opts.tol);
    let mut conflict_set: Option<Vec<u32>> = None;
    if opts.conflicts.unwrap_or(!violated.is_empty()) {
        // Candidates = the structurally over-determined block (where a redundancy must live); if
        // the graph sees nothing wrong (e.g. the triangle inequality) fall back to the violated
        // constraints.  Everything else stays fixed, so the result is minimal "among the suspects"
        // — and the filter costs |candidates| solves, not |all|.
        let cands = if !over.is_empty() { over.clone() } else { violated.clone() };
        conflict_set = Some(minimal_conflict_set(sk, Some(&cands), opts.tol, Method::DogLeg, 60));
    }

    // -- pebble game on the point-distance graph --
    let (clusters, redundant) = distance_rigidity(sk);

    // -- entity states --
    let under_set: BTreeSet<u32> = under_params.iter().copied().collect();
    let conflict_ids: BTreeSet<u32> = match &conflict_set {
        Some(c) => c.iter().copied().collect(),
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
    // and a primitive's own definition, leaving nothing to delete
    let status = if conflict_set.as_ref().map(|c| !c.is_empty()).unwrap_or(false)
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
        over,
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
    let cands: Vec<u32> = match candidates {
        Some(c) => c
            .iter()
            .copied()
            .filter(|id| sk.constraint(*id).map(|k| !k.soft).unwrap_or(false))
            .collect(),
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
