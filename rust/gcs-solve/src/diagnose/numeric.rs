//! **The numeric cross-check**: the Jacobian's rank where the drawing stands, the parameters that
//! can actually move, and the dependencies among its rows — everything the matching reads is
//! structural and cannot see a theorem-induced dependency, and this is what does.  The whole
//! Jacobian is factored dense up to `NUMERIC_MAX` free parameters, and past it part by part over
//! the block-triangular order (#88, `parts`).

use super::*;
use crate::constraints::same_constraint;
use crate::linalg::{rank_and_nullspace, Mat};
use crate::system::SparseConditioned;
use crate::witness::{movable_columns, screen, shaky_warning};

/// A null-space entry below this is zero: the parameter does not move, the row takes no part.
const RTOL: f64 = 1e-8;

/// How far the numeric cross-check reaches.
pub(super) struct Gate {
    /// Ranks are read at all: below `numeric_max`, or asked for.  The claims and the parameters'
    /// ledgers read theirs under the same gate.
    pub numeric: bool,
    /// The whole Jacobian is factored: ranks are read and there is a matrix to read them of.
    whole: bool,
    /// Past the dense limit the whole matrix is not factored, but its parts are (#88).
    pub by_parts: bool,
    /// Neither reading is taken.
    pub skipped: bool,
}

impl Gate {
    pub fn of(opts: &DiagnoseOptions, sys: &System) -> Gate {
        let n_cols = sys.n_free;
        let some = n_cols > 0 && sys.n_res > 0;
        let numeric = opts.numeric.unwrap_or(n_cols <= opts.numeric_max);
        let by_parts = opts.numeric.is_none() && !numeric && some;
        let skipped = opts.numeric.is_none() && !numeric && !by_parts;
        Gate { numeric, whole: numeric && some, by_parts, skipped }
    }
}

/// The numeric cross-check's answer at this configuration.
#[derive(Default)]
pub(super) struct Numeric {
    /// The Jacobian's rank, screened; `None` where it was not read.
    pub rank: Option<usize>,
    /// The motions the screen blocked (`Diagnosis::shaky`).
    pub shaky: usize,
    /// The parameters that can actually move, wherever the rank was read: rows of the null space
    /// that are nonzero.  Sharper than the DM under-block (which counts a parameter as free if it
    /// could be in some generic assignment); evaluated at the current configuration.
    pub movable: Option<Vec<u32>>,
    /// The dependencies among the rows, where the rank says there are any.
    pub dependencies: Option<Dependencies>,
}

/// The constraints a numeric reading finds wholly implied (`removable_constraints`).
pub(super) struct Dependencies {
    /// By any rows at all.
    pub removable: Vec<u32>,
    /// By the relations alone, an exact duplicate left out: what the relations already say.  The
    /// static "carries a dimension" test (`states_no_number`) stands in for the witness criterion
    /// (the deficiency survives jittered dimensions) because this runs after every edit.  An exact
    /// duplicate is never a theorem: two Horizontals on one line match two variables, so the graph
    /// passes them, but a copy is a surplus whatever it is made of.
    pub implied: Vec<u32>,
}

impl Dependencies {
    fn of(sk: &Sketch, removable: Vec<u32>, removable_rel: Vec<u32>) -> Dependencies {
        let implied = removable_rel.into_iter().filter(|&c| !duplicated(sk, c)).collect();
        Dependencies { removable, implied }
    }
}

/// The numeric cross-check, the whole Jacobian or its parts as `gate` says.
pub(super) fn read(
    sk: &mut Sketch,
    sys: &mut System,
    m: &Matched,
    gate: &Gate,
    wit: Option<&WitnessReport>,
    warnings: &mut Vec<String>,
) -> Numeric {
    let out = if gate.whole {
        whole(sk, sys, m, wit, warnings)
    } else if gate.by_parts {
        by_parts(sk, sys, m, warnings)
    } else {
        return Numeric::default();
    };
    if let Some(rank) = out.rank.filter(|&r| r < m.dm.rank) {
        let structural = m.dm.rank;
        warnings.push(format!(
            "structural rank {structural} but numeric rank {rank}: a dependency the graph cannot \
             see (theorem-induced or degenerate configuration) — Stage 4"
        ));
    }
    out
}

/// The numeric cross-check on the whole Jacobian, factored dense.
fn whole(
    sk: &mut Sketch,
    sys: &mut System,
    m: &Matched,
    wit: Option<&WitnessReport>,
    warnings: &mut Vec<String>,
) -> Numeric {
    // the conditioned Jacobian at z, kept for the dependencies below rather than built a second
    // time at the same pose — the screen leaves both sketch and system as it found them, so it
    // is still the right matrix afterwards
    let mut cond: Option<crate::system::Conditioned> = None;
    let (rank, shaky, movable) = match wit {
        // same J at the same x, screened already
        Some(w) if w.used_current => (w.numeric_rank, w.blocked, m.params(&w.movable)),
        _ => {
            let z = sys.z0(sk);
            let c = sys.conditioned(&z);
            let rn = c.rank_and_nullspace(RANK_TOL);
            cond = Some(c);
            if !rn.converged {
                // no rank at all, rather than the zero a failed SVD leaves behind — which
                // reads as "every constraint is redundant"
                warnings.push(
                    "numeric rank unavailable: the SVD did not converge (a degenerate or \
                     non-finite Jacobian) — the structural analysis stands alone"
                        .to_string(),
                );
                return Numeric::default();
            }
            let (rank, null, blocked) =
                screened(sk, sys, &z, rn.null(), rn.rank, m.dm.rank, warnings);
            (rank, blocked, m.params(&movable_columns(&null, RTOL)))
        }
    };
    // ...and name the constraints worth removing, or the report would say "over-constrained"
    // with nothing to point at.  One extra SVD, only where something is redundant at all —
    // whether the matching saw it or only the Jacobian does.  Run only on the second case, a
    // determined square with a stated side had every one of its ten constraints `over`, the side
    // length first: the matching's over-block at DOF 0 is every row touching the cluster, where
    // W's reading is the relation-only closure it always was, which is `implied` and nothing to
    // fix (#45.5).
    let dependencies = (rank < m.n_rows()).then(|| {
        let j = cond.unwrap_or_else(|| {
            let z = sys.z0(sk);
            sys.conditioned(&z)
        });
        let w = j.left_nullspace(RANK_TOL).null();
        // `implied` is what the relations alone already say: the same test on the left null
        // space of the relation-only rows (a dependency there embeds in W, so this is a subset
        // of the removable set).  Whatever is removable only with a dimension's help is `over`.
        let rel_rows: Vec<usize> = (0..j.rows())
            .filter(|&r| sk.constraint(m.row_c[r]).is_some_and(states_no_number))
            .collect();
        let w_rel = if rel_rows.len() == j.rows() {
            w.clone()
        } else {
            j.select_rows(&rel_rows).left_nullspace(RANK_TOL).null()
        };
        let row_c_rel: Vec<u32> = rel_rows.iter().map(|&r| m.row_c[r]).collect();
        let removable = removable_constraints(sk, &w, &m.row_c, RTOL);
        let removable_rel = removable_constraints(sk, &w_rel, &row_c_rel, RTOL);
        Dependencies::of(sk, removable, removable_rel)
    });
    Numeric { rank: Some(rank), shaky, movable: Some(movable), dependencies }
}

/// The numeric cross-check past the dense limit: the same reading part by part (#88).
fn by_parts(
    sk: &mut Sketch,
    sys: &mut System,
    m: &Matched,
    warnings: &mut Vec<String>,
) -> Numeric {
    let z = sys.z0(sk);
    let p = numeric_by_parts(sk, sys, &m.row_c, &z);
    let (rank, null, shaky) = screened(sk, sys, &z, p.null, p.rank, m.dm.rank, warnings);
    let dependencies =
        (rank < m.n_rows()).then(|| Dependencies::of(sk, p.removable, p.removable_rel));
    Numeric {
        rank: Some(rank),
        shaky,
        movable: Some(m.params(&movable_columns(&null, RTOL))),
        dependencies,
    }
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

/// Whether constraint `c` says exactly what another that acts says.  `same_constraint` compares
/// what is said, not whether it is said: a claim restating a relation matches it exactly, and
/// counted as a duplicate it would move that relation out of `implied` and into `over` — a claim
/// making the sketch over-constrained, which is the one thing §9.7 promises cannot happen.  Only
/// what acts can duplicate.
fn duplicated(sk: &Sketch, c: u32) -> bool {
    sk.constraint(c).is_some_and(|m| {
        sk.constraints.iter().any(|o| o.acts() && o.id != c && same_constraint(o, m))
    })
}

/// The second-order screen on a numeric reading's null space, as both readings take it: where
/// the Jacobian claims motions the matching cannot account for, settle-test them — a tangency at
/// its own contact is a double root whose "motion" walks back.  `screen` owns both guards (only
/// at a solution, never more than the discrepancy, so a motion the matching also sees is safe).
/// The rank screened, the motions left, and how many it blocked.
fn screened(
    sk: &mut Sketch,
    sys: &mut System,
    z: &[f64],
    null: Mat,
    rank: usize,
    structural: usize,
    warnings: &mut Vec<String>,
) -> (usize, Mat, usize) {
    let (null, blocked) = screen(sk, sys, z, null, structural.saturating_sub(rank));
    if blocked > 0 {
        warnings.push(shaky_warning(blocked));
    }
    (rank + blocked, null, blocked)
}

/// A row that states no number anyone edits — a relation, or a derivative row whatever its row
/// states (#148) — so a dependency through it is no surplus: the `implied` side of the line.
fn states_no_number(c: &Constraint) -> bool {
    !c.kind.has_dimension() || c.along.is_some()
}

/// What the parts say together (`parts`, #88): the whole Jacobian's rank and null space, and the
/// constraints wholly implied — by any rows, and by relations alone.
struct ByParts {
    rank: usize,
    /// One column per motion, a row per free column.
    null: Mat,
    removable: Vec<u32>,
    removable_rel: Vec<u32>,
}

/// The block-triangular order's parts (#88): over first (its rows read nothing else), the square
/// blocks as they are solved, under last — and each residual row's constraint.
fn parts_in_order(sys: &mut System, row_c: &[u32]) -> (Vec<parts::Part>, Vec<u32>) {
    use parts::Part;
    let order = sys.block_order();
    let mut of_row = vec![u32::MAX; sys.n_res];
    for (i, r) in sys.hard_rows().into_iter().enumerate() {
        of_row[r] = row_c[i];
    }
    let mut all = vec![Part { rows: order.over_rows.clone(), cols: order.over_cols.clone() }];
    all.extend(order.blocks.iter().map(|b| Part { rows: b.rows.clone(), cols: b.cols.clone() }));
    all.push(Part { rows: order.under_rows.clone(), cols: order.under_cols.clone() });
    (all, of_row)
}

/// **What the parts read, for a caller to hold it to the Jacobian itself** (#88, #780): the
/// conditioned Jacobian where `sk` stands, how many of its rows the parts read (the hard ones),
/// the motions the parts read (each a value per free column) and the dependencies among those
/// rows (each a value per residual row) — what `diagnose` reads past `NUMERIC_MAX`, without the
/// whole SVD it is otherwise held to.
#[doc(hidden)]
pub fn parts_null_spaces(sk: &Sketch) -> (SparseConditioned, usize, Vec<Vec<f64>>, Vec<Vec<f64>>) {
    let mut sys = System::new(sk);
    let (_, row_c) = sys.structure();
    let (all, _) = parts_in_order(&mut sys, &row_c);
    let rows = all.iter().map(|p| p.rows.len()).sum();
    let z = sys.z0(sk);
    let j = sys.conditioned_sparse(&z);
    let (null, left) = (parts::right_null(&j, &all), parts::left_null(&j, &all));
    (j, rows, null, left)
}

fn numeric_by_parts(sk: &Sketch, sys: &mut System, row_c: &[u32], z: &[f64]) -> ByParts {
    use parts::Part;
    let (all, of_row) = parts_in_order(sys, row_c);
    let relation = |r: &usize| sk.constraint(of_row[*r]).is_some_and(states_no_number);
    let rel: Vec<Part> = all.iter().map(|p| Part {
        rows: p.rows.iter().copied().filter(relation).collect(),
        cols: p.cols.clone(),
    }).collect();
    let j = sys.conditioned_sparse(z);
    let null = parts::right_null(&j, &all);
    // vectors as the columns of a matrix
    let as_cols = |vs: &[Vec<f64>], n: usize| Mat::from_vec(vs.len(), n, vs.concat()).transpose();
    // the rows a left null space reaches, and the constraints it may name
    let removable = |w: &[Vec<f64>]| {
        let rows: Vec<usize> = (0..sys.n_res).filter(|&r| w.iter().any(|v| v[r] != 0.0)).collect();
        let m = as_cols(w, sys.n_res).select_rows(&rows);
        let ids: Vec<u32> = rows.iter().map(|&r| of_row[r]).collect();
        removable_constraints(sk, &m, &ids, RTOL)
    };
    // a dependency among rows only where there are more rows than the rank; among relations
    // only where there is one among all the rows
    let rank = sys.n_free - null.len();
    let rows: usize = all.iter().map(|p| p.rows.len()).sum();
    let all_rows = if rank < rows { parts::left_null(&j, &all) } else { Vec::new() };
    let rel_rows = match all_rows.is_empty() {
        true => Vec::new(),
        false => parts::left_null(&j, &rel),
    };
    ByParts {
        rank,
        null: as_cols(&null, sys.n_free),
        removable: removable(&all_rows),
        removable_rel: removable(&rel_rows),
    }
}
