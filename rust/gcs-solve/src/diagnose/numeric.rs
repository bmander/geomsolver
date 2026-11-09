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
    /// The dependent rows are read for the constraints worth removing: all but `movable`.
    dependencies: bool,
}

impl Gate {
    pub fn of(opts: &DiagnoseOptions, sys: &System) -> Gate {
        let n_cols = sys.n_free;
        let some = n_cols > 0 && sys.n_res > 0;
        let numeric = opts.numeric.unwrap_or(n_cols <= opts.numeric_max);
        let by_parts = opts.numeric.is_none() && !numeric && some;
        let skipped = opts.numeric.is_none() && !numeric && !by_parts;
        Gate { numeric, whole: numeric && some, by_parts, skipped, dependencies: true }
    }

    /// The same reading of the rank and what moves, naming no constraint (`diagnose::movable`).
    pub(super) fn movable(self) -> Gate {
        Gate { dependencies: false, ..self }
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
    /// The constraints the dependencies no removable one takes are among (`said_twice`), and
    /// how many such dependencies there are.
    pub twice: (Vec<u32>, usize),
}

impl Dependencies {
    fn of(sk: &Sketch, removable: Vec<u32>, removable_rel: Vec<u32>) -> Dependencies {
        let implied = removable_rel.into_iter().filter(|&c| !duplicated(sk, c)).collect();
        Dependencies { removable, implied, twice: (Vec::new(), 0) }
    }
}

/// **The constraints a row said twice is among, where no one of them can go** (#850): the
/// dependencies among the rows (`w`, an orthonormal basis of the left null space, a column each,
/// a row per residual row of `row_c`) that leaving out every constraint `removable` names would
/// not take — those of W that vanish on its rows, `W · null(W[R])` — and the constraints whose
/// rows they reach, intrinsic ones too, in row order, with how many such dependencies there are.
/// A point held on two axes that meet, and the axes held through a point there: four statements
/// of two rows over the point's three numbers, each needed, and one row past them — no statement
/// is implied whole, so `removable` names none, and the count said "1700 equations, structural
/// rank 1699" and nothing more.
fn said_twice(w: &Mat, row_c: &[u32], removable: &[u32], rtol: f64) -> (Vec<u32>, usize) {
    let removable: BTreeSet<u32> = removable.iter().copied().collect();
    let named: Vec<usize> = (0..w.rows).filter(|&r| removable.contains(&row_c[r])).collect();
    // every row reached is a removable constraint's: their leaving out takes every dependency
    if w.cols == 0 || named.len() == w.rows {
        return (Vec::new(), 0);
    }
    // each row of `W · N`, where N spans the dependencies that vanish on the rows named
    let n = (!named.is_empty()).then(|| {
        let tol = crate::linalg::Tol::Abs(TWICE);
        crate::linalg::rank_and_nullspace_with(&w.select_rows(&named), tol).null()
    });
    let left = n.as_ref().map_or(w.cols, |n| n.cols);
    if left == 0 {
        return (Vec::new(), 0);
    }
    let mut ids: Vec<u32> = Vec::new();
    let mut seen = BTreeSet::new();
    for r in 0..w.rows {
        let share = match &n {
            Some(n) => n.mul_t_vec(w.row(r)),
            None => w.row(r).to_vec(),
        };
        if share.iter().any(|v| v.abs() > rtol) && seen.insert(row_c[r]) {
            ids.push(row_c[r]);
        }
    }
    (ids, left)
}

/// A dependency that touches the rows of the constraints named removable less than this (W being
/// orthonormal, an absolute measure) is one their leaving out does not take (`said_twice`).
const TWICE: f64 = 1e-6;

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
        whole(sk, sys, m, gate, wit, warnings)
    } else if gate.by_parts {
        by_parts(sk, sys, m, gate, warnings)
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
    gate: &Gate,
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
    let dependencies = (gate.dependencies && rank < m.n_rows()).then(|| {
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
        let twice = said_twice(&w, &m.row_c, &removable, RTOL);
        Dependencies { twice, ..Dependencies::of(sk, removable, removable_rel) }
    });
    Numeric { rank: Some(rank), shaky, movable: Some(movable), dependencies }
}

/// The numeric cross-check past the dense limit: the same reading part by part (#88).
fn by_parts(
    sk: &mut Sketch,
    sys: &mut System,
    m: &Matched,
    gate: &Gate,
    warnings: &mut Vec<String>,
) -> Numeric {
    let z = sys.z0(sk);
    let p = numeric_by_parts(sk, sys, &m.row_c, &z, gate.dependencies);
    let (rank, null, shaky) = screened(sk, sys, &z, p.null, p.rank, m.dm.rank, warnings);
    let dependencies = p.dependent.filter(|_| rank < m.n_rows()).map(|d| Dependencies {
        twice: d.twice,
        ..Dependencies::of(sk, d.all, d.rel)
    });
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
        sk.constraints.iter().filter(|c| c.is_intrinsic()).map(|c| c.id).collect();
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
    /// What the dependent rows name, where they were read.
    dependent: Option<Removable>,
}

/// The constraints the parts' dependent rows name: removable among all of them, among the
/// relations', and the constraints a row said twice is among (`said_twice`).
struct Removable {
    all: Vec<u32>,
    rel: Vec<u32>,
    twice: (Vec<u32>, usize),
}

/// **What the parts read, for a caller to hold it to the Jacobian itself** (#88, #780): the
/// conditioned Jacobian where `sk` stands, how many of its rows the parts read (the hard ones),
/// the motions the parts read (each a value per free column) and the dependencies among those
/// rows (each a value per residual row) — what `diagnose` reads past `NUMERIC_MAX`, without the
/// whole SVD it is otherwise held to.
#[doc(hidden)]
pub fn parts_null_spaces(sk: &Sketch) -> (SparseConditioned, usize, Vec<Vec<f64>>, Vec<Vec<f64>>) {
    let mut sys = System::new(sk);
    let z = sys.z0(sk);
    let j = sys.conditioned_sparse(&z);
    let hard = sys.hard_rows();
    let all = parts::order(&j, &hard);
    let (null, left) = (parts::right_null(&j, &all), parts::left_null(&j, &all));
    (j, hard.len(), null, left)
}

/// `dependencies` false reads the rank and the null space alone: no left null space.
fn numeric_by_parts(
    sk: &Sketch, sys: &mut System, row_c: &[u32], z: &[f64], dependencies: bool,
) -> ByParts {
    use parts::Part;
    let j = sys.conditioned_sparse(z);
    let hard = sys.hard_rows();
    let all = parts::order(&j, &hard);
    // each hard row's constraint, by the full residual vector's rows
    let mut of_row = vec![u32::MAX; sys.n_res];
    for (&r, &c) in hard.iter().zip(row_c) {
        of_row[r] = c;
    }
    let relation = |r: &usize| sk.constraint(of_row[*r]).is_some_and(states_no_number);
    let rel: Vec<Part> = all.iter().map(|p| Part {
        rows: p.rows.iter().copied().filter(relation).collect(),
        cols: p.cols.clone(),
    }).collect();
    let null = parts::right_null(&j, &all);
    // vectors as the columns of a matrix
    let as_cols = |vs: &[Vec<f64>], n: usize| Mat::from_vec(vs.len(), n, vs.concat()).transpose();
    // the rows a left null space reaches, and the constraints it may name
    let reached = |w: &[Vec<f64>]| {
        let rows: Vec<usize> = (0..sys.n_res).filter(|&r| w.iter().any(|v| v[r] != 0.0)).collect();
        let m = as_cols(w, sys.n_res).select_rows(&rows);
        let ids: Vec<u32> = rows.iter().map(|&r| of_row[r]).collect();
        (m, ids)
    };
    let removable = |w: &[Vec<f64>]| {
        let (m, ids) = reached(w);
        removable_constraints(sk, &m, &ids, RTOL)
    };
    // a dependency among rows only where there are more rows than the rank; among relations
    // only where there is one among all the rows
    let rank = sys.n_free - null.len();
    let null_cols = as_cols(&null, sys.n_free);
    if !dependencies {
        return ByParts { rank, null: null_cols, dependent: None };
    }
    let rows = hard.len();
    let all_rows = if rank < rows { parts::left_null(&j, &all) } else { Vec::new() };
    let rel_rows = match all_rows.is_empty() {
        true => Vec::new(),
        false => parts::left_null(&j, &rel),
    };
    let (m, ids) = reached(&all_rows);
    let removable_all = removable_constraints(sk, &m, &ids, RTOL);
    // the parts' reading is a basis and not an orthonormal one, which `said_twice` reads
    // (spanning what they do, so reaching the same rows, in the same order)
    let unit: Vec<Vec<f64>> = all_rows.iter().map(|v| (v, crate::linalg::norm(v)))
        .filter(|&(_, n)| n > 0.0)
        .map(|(v, n)| v.iter().map(|x| x / n).collect())
        .collect();
    let (w, _) = reached(&crate::linalg::orthonormalize(&unit, 1e-10));
    let twice = said_twice(&w, &ids, &removable_all, RTOL);
    let dependent = Removable { all: removable_all, rel: removable(&rel_rows), twice };
    ByParts { rank, null: null_cols, dependent: Some(dependent) }
}
