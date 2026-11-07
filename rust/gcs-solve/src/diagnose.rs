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

#[allow(unused_imports)]
use crate::fmath::Det;
use crate::cgraph::coincident_classes;
use crate::constraints::{CKind, Constraint};
use crate::graph;
use crate::model::{EntRef, Sketch};
use crate::newton::Method;
use crate::solve::SolveOpts;
use crate::system::{Sizes, Stop, System};
use crate::system::RANK_TOL;
use crate::witness::{analyze_with, WitnessReport};
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

/// A connected component of the constraint graph with its own DOF accounting.
#[derive(Clone, Debug)]
pub struct Component {
    pub params: Vec<u32>,
    pub constraints: Vec<u32>,
    pub structural_rank: usize,
    pub dof: i64,
}

pub mod analysis;
mod parts;
mod claims;
mod numeric;
mod solid_claims;
mod witnessed;
use numeric::{Gate, Numeric};
pub use numeric::{parts_null_spaces, removable_constraints};
pub use parts::rank_by_parts;
pub use solid_claims::{
    describe_solid_claim, judge_solids, ClaimJudge, ClaimProgress, SolidOutcome, SolidPose,
    SolidVerdict, SWEEP_STEPS,
};

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
    /// Past the dense limit, the numeric rank was read part by part (#88): `numeric_rank` and
    /// what follows from it are the parts' — a lower bound on the whole's.
    pub by_parts: bool,
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
    /// A plane's tangency to a set made dependent by what the drawing already says along the
    /// plane (#148): the body's derivative along both of the plane's axes, where one is what the
    /// other says — a cone's, its apex on the plane, stationary along the generator.  How the
    /// statement is made, not something the author wrote twice: said nowhere, never painted.
    pub expected: Vec<u32>,
    /// The `claim` statements, judged.  A claim is no equation — it joins none of the counts or
    /// sets above, and can never make the sketch Over or Conflict — so its whole report is which
    /// of these three lists it landed in.  *Theorem*: it holds, and its rows add no rank — the
    /// drawing already says it.  *Violated*: it does not hold at this solution.  *Consuming*: it
    /// holds here, but enforcing it would have taken a freedom — satisfied by the pose, not by
    /// the document, which is a claim that claims too much.
    pub claims_theorem: Vec<u32>,
    pub claims_violated: Vec<u32>,
    pub claims_consuming: Vec<u32>,
    /// The bounds (§9.6), read on the solution: a bound is no equation either, and chooses which
    /// root is meant.  *Violated*: it does not hold, which a solve reports as a failure.
    /// *Active*: it holds with the solution on its edge — told, not painted.  One that holds
    /// with room is in neither.
    pub bounds_violated: Vec<u32>,
    pub bounds_active: Vec<u32>,
    /// The orientations the document states (`ccw`, `cw`, §9.6, #157) that the solution breaks,
    /// by their records' keys (`Sketch::orientations`): which a solve reports as a failure.
    pub orientations_violated: Vec<String>,
    /// The tangencies at a contact, `parallel`s and `perpendicular`s with a line of no length
    /// (#479, `solve::collapsed`): each holds whatever that line's direction, so the figure is
    /// degenerate, which a solve reports as a failure.
    pub collapsed: Vec<u32>,
    /// Each `minimizes`'s and `maximizes`'s verdict (#121), by its terms' constraints: minimum,
    /// maximum, saddle or degenerate, read off the reduced Hessian at the pose diagnosed — told,
    /// never enforced: a stationary point the solve reached is the drawing's, and the verdict says
    /// which kind it is.
    pub extrema: Vec<(u32, crate::variational::Extremum)>,
    /// The claims about **solids**, judged (§9.8).  Kept apart from the three lists above
    /// because the trichotomy does not apply: a solid claim compiles no row, so there is no rank
    /// for it to consume and *consuming* has no meaning.  It holds, it is refuted, or the
    /// faceting cannot tell — which is the honest third answer and not a failure.
    pub solid_claims: Vec<SolidVerdict>,
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
    /// Each requirement out of reach of what drives it where the solve stopped (#510 P7), with
    /// why in the core's words (`Reach::text`): the conflict set, where there is one, and no
    /// search was made for another (`System::stop_at`).
    pub out_of_reach: Vec<(u32, String)>,
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
    /// **The ledger per parameter** (#391 S4): each declared parameter the drawing's relations
    /// read, what reads it and the freedom it drives.  Empty where the drawing declares none.
    pub parameters: Vec<Driven>,
}

/// What a declared parameter drives (`Diagnosis::parameters`, #391 S4): the relations whose
/// numbers read it (`follow::readers`), and the degrees of freedom they take up — the rank their
/// rows add to every other row's, which is the freedom of the drawing the parameter moves: a
/// four-bar's one, its crank's angle, `1 DOF over t`.  `None` where no numeric rank was read (past
/// the dense limit).  A freedom left beside it (`Diagnosis::dof`) is a free *function* of the
/// parameter, which nothing says how to move as it runs.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Driven {
    pub name: String,
    pub readers: Vec<u32>,
    pub dof: Option<usize>,
}

/// **Which freedoms of a plane are left**, by the name the ledger gives them (§16.3):
/// `Q.attitude` for any of its axes' direction unknowns the diagnosis finds movable, `Q.origin`
/// for any of the three numbers saying where it stands — a plane's turn is carried by its axes'
/// six numbers and their rows, so naming the numbers would name nothing a reader wrote.  Empty
/// where every plane is held, so what a report says of a drawing with no free plane is what it
/// said before.
pub fn view_freedoms(sk: &Sketch, d: &Diagnosis) -> Vec<String> {
    let under: BTreeSet<u32> = d.under_params.iter().copied().collect();
    let mut out = Vec::new();
    let mut places = BTreeSet::new();
    for (i, p) in sk.frames.iter().enumerate().filter_map(|(i, f)| Some((i, f.chart.as_ref()?))) {
        // one standing where an earlier one does is that frame read another way (a frame's own
        // datums, `Sketch::plane_at`): its freedoms are the earlier one's, said there
        if !places.insert(p.o) {
            continue;
        }
        let axes = [p.u, p.v].map(|r| sk.axes[r as usize].d);
        if axes.iter().flatten().any(|q| under.contains(q)) {
            out.push(format!("{}.attitude", sk.plane_name(i)));
        }
        if p.o.iter().any(|q| under.contains(q)) {
            out.push(format!("{}.origin", sk.plane_name(i)));
        }
    }
    out
}

pub fn summary(d: &Diagnosis) -> String {
    let mut parts = vec![
        format!(
            "{} params, {} equations, structural rank {}",
            d.n_params, d.n_equations, d.structural_rank
        ),
        format!("DOF {}", d.dof),
    ];
    // a drawing over a parameter: what each drives, and what is left a free function of them
    let read: Vec<&Driven> = d.parameters.iter().filter(|p| !p.readers.is_empty()).collect();
    for p in &read {
        if let Some(n) = p.dof {
            parts.push(format!("{n} DOF over {}", p.name));
        }
    }
    if d.dof > 0 && !read.is_empty() {
        let names: Vec<&str> = read.iter().map(|p| p.name.as_str()).collect();
        parts.push(format!("each DOF left a free function of {}", names.join(", ")));
    }
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
    if d.status == State::Unsolved && d.violated.is_empty() {
        parts.push(
            "UNSOLVED — the solve stopped short of a solution, though every constraint reads \
             as holding where it stopped; no verdict on it"
                .to_string(),
        );
    } else if d.status == State::Unsolved {
        parts.push(format!(
            "UNSOLVED — {} constraint(s) unsatisfied where the solve stopped short of a \
             stationary point; no verdict on them",
            d.violated.len()
        ));
    } else if !d.out_of_reach.is_empty() {
        parts.push(
            "CONFLICT — a requirement out of reach of what drives it: change what it states, or \
             what drives it"
                .to_string(),
        );
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

/// **What a failed drawing's driven deflections cannot meet**, each with why in words (#510 P7,
/// S2; `Reach::text`): every failing one out of reach of what drives it, else each at the edge of
/// the mesh it is read on, as `stop` says.  Empty where neither is the failure.
fn unmet(sk: &Sketch, stop: Stop) -> Vec<(u32, String)> {
    let spell = |v| crate::beams::value_text(sk, crate::model::Output::Deflection, v);
    match stop {
        Stop::OutOfReach(rows) | Stop::Folded(rows) => {
            rows.into_iter().filter_map(|(c, v)| v.text(&spell).map(|t| (c, t))).collect()
        }
        Stop::Other => Vec::new(),
    }
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
        sk.constraints.iter().filter(|c| c.soft || c.is_intrinsic()).map(|c| c.id).collect();
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
    /// Whether the claims about solids are judged here.  A page leaves them to a worker
    /// (`ClaimJudge`): they read the solved pose and enter nothing else the diagnosis says, and
    /// judging one is seconds of solids evaluated.  Not judged, `solid_claims` is empty.
    pub solid_claims: bool,
    /// Whether the pose diagnosed is a solve's answer (`SolveResult::settled`), as the caller
    /// that solved it knows (#449).  Where it is not — the solve failed, or stopped on its
    /// iteration limit under the acceptance — nothing is judged *on* it: no claim lands in any
    /// of the three lists, and a drawing whose rows all read as holding there is `Unsolved`, not
    /// well.  The rows can read as holding where they do not: a point run off to infinity widens
    /// the extent every row is measured against.  True where the caller does not say.
    pub settled: bool,
}

impl Default for DiagnoseOptions {
    fn default() -> DiagnoseOptions {
        DiagnoseOptions {
            numeric: None,
            conflicts: None,
            witness: false,
            tol: 1e-6,
            numeric_max: NUMERIC_MAX,
            solid_claims: true,
            settled: true,
        }
    }
}

/// Structural (and optionally numeric) diagnosis of a sketch at its current configuration.
pub fn diagnose(sk: &mut Sketch, opts: DiagnoseOptions) -> Diagnosis {
    let mut sys = System::new(sk);
    diagnose_with(sk, &mut sys, opts)
}

/// The diagnosis of `sk` against `sys`, the system the caller solved it with.  Each reading is a
/// phase of its own below, taken in this order — the matching, the numeric cross-check, what is
/// violated and whether a solve could do better, bounds, claims, the conflict search, rigidity,
/// each entity's state and the status — since the later ones read the earlier ones' answers, and
/// several share `sys`, whose remembered poses each compile throws away.
pub fn diagnose_with(sk: &mut Sketch, sys: &mut System, opts: DiagnoseOptions) -> Diagnosis {
    // the caller hands us the System it solved with; dimensions may have been edited since, and
    // the residuals we are about to judge are read from the compiled constants
    sys.refresh_consts(sk);
    let m = Matched::of(sys);
    let gate = Gate::of(&opts, sys);
    // an entity's own intrinsic rows — the two that make an arc's ends its ends — are not in
    // the document and cannot be removed, so "remove one of these" never names one (#43.17);
    // the numeric path (`removable_constraints`) already leaves them out
    let intrinsic: BTreeSet<u32> =
        sk.constraints.iter().filter(|c| c.is_intrinsic()).map(|c| c.id).collect();
    let mut red = Redundancy::matched(&m, &intrinsic);
    // the matching's reading, kept: it is where a redundancy *must* live, so the conflict
    // search below seeds its candidates from it whatever W says about who is removable
    let structural_over = red.over.clone();
    let structural_under = m.params(&m.dm.under_cols);
    let components = components(&m);

    let mut warnings = standing_warnings(sk);
    // the witness analysis (Stage 4), on demand
    let wit = (opts.witness && m.n_cols > 0 && sys.n_res > 0)
        .then(|| analyze_with(sk, sys, None, &red.in_over, RANK_TOL, 0));
    let numeric: Numeric = numeric::read(sk, sys, &m, &gate, wit.as_ref(), &mut warnings);
    if let Some(d) = numeric.dependencies {
        red.reread(d);
    }
    // where the numeric cross-check read a rank, what it says moves is the truth here
    let under_params = numeric.movable.unwrap_or_else(|| structural_under.clone());

    let violated = violated_constraints(sk, sys, opts.tol);
    let reach = reach(sk, sys, &violated, opts.tol);
    let (bounds_violated, bounds_active) = bounds(sk);
    let claims = claims::judge(sk, sys, opts.tol, gate.numeric, opts.settled);
    let (conflicts, stalled) =
        conflicts(sk, &opts, structural_over, &red.over, &violated, &intrinsic, &reach);

    let contested = |c: &u32| {
        violated.contains(c) || conflicts.as_ref().is_some_and(|s| s.contains(c))
    };
    let expected = red.file(contested, |c| touches_axis(sk, c));

    // -- pebble game on the point-distance graph --
    let (clusters, redundant) = distance_rigidity(sk);

    let under_set: BTreeSet<u32> = under_params.iter().copied().collect();
    let blamed: BTreeSet<u32> = match &conflicts {
        Some(c) => c.iter().copied().collect(),
        // stalled, the violated rows are no verdict on anything they touch: an entity keeps
        // the determination the rank gives it, which holds at a non-solution as well
        None if stalled => BTreeSet::new(),
        None => violated.iter().copied().collect(),
    };
    let entity_state = entity_states(sk, &blamed, &red.in_over, &under_set);
    if let Some(w) = &wit {
        warnings.extend(w.warnings.iter().cloned());
    }

    let parameters = driven(sk, sys, &m.row_c, gate.numeric);
    // the structural matching is a *generic* upper bound on the rank; where the numeric
    // cross-check ran it is the truth at this configuration, and it is what decides what moves
    let effective_rank = numeric.rank.unwrap_or(m.dm.rank);
    let dof = m.n_cols as i64 - effective_rank as i64;
    let status = status(opts.settled, stalled, &violated, conflicts.as_deref(), &red.over, dof);
    Diagnosis {
        n_params: m.n_cols,
        n_equations: m.n_rows(),
        structural_rank: m.dm.rank,
        numeric_rank: numeric.rank,
        numeric_skipped: gate.skipped,
        by_parts: gate.by_parts,
        geometric_dependency: numeric.rank.map(|nr| m.dm.rank.saturating_sub(nr)).unwrap_or(0),
        shaky: numeric.shaky,
        over: red.over,
        implied: red.implied,
        expected,
        claims_theorem: claims.theorem,
        claims_violated: claims.violated,
        claims_consuming: claims.consuming,
        bounds_violated,
        bounds_active,
        orientations_violated: crate::solve::orientations_broken(sk),
        collapsed: crate::solve::collapsed(sk),
        extrema: crate::variational::extrema(sk),
        // **judged, at the very end, and never compiled.**  A solid claim reads the solved pose
        // and the terms over it; it enters no system, so nothing above this line can see it and
        // nothing it says can move a number.
        solid_claims: if opts.solid_claims { judge_solids(sk) } else { Vec::new() },
        under_params,
        structural_under_params: structural_under,
        components,
        entity_state,
        rigid_clusters: clusters,
        redundant_distances: redundant,
        violated,
        conflicts,
        out_of_reach: reach.out_of_reach,
        warnings,
        witness: wit,
        dof,
        structural_dof: m.n_cols as i64 - m.dm.rank as i64,
        n_redundant: m.n_rows() as i64 - effective_rank as i64,
        structural_n_redundant: m.n_rows() as i64 - m.dm.rank as i64,
        status,
        parameters,
    }
}

/// The matching's reading of the compiled system: its rows' graph over the free columns, each
/// row's constraint, and the Dulmage–Mendelsohn decomposition every later reading starts from.
struct Matched {
    adj: Vec<Vec<usize>>,
    row_c: Vec<u32>,
    n_cols: usize,
    dm: graph::Dm,
    /// Each free column's parameter.
    free_params: Vec<u32>,
}

impl Matched {
    fn of(sys: &System) -> Matched {
        let (adj, row_c) = sys.structure();
        let n_cols = sys.n_free;
        let dm = graph::dulmage_mendelsohn(&adj, n_cols);
        let free_params = sys.free.iter().map(|&i| i as u32).collect();
        Matched { adj, row_c, n_cols, dm, free_params }
    }

    /// The residual rows: the equations.
    fn n_rows(&self) -> usize {
        self.adj.len()
    }

    /// The parameters of free columns `cols`.
    fn params(&self, cols: &[usize]) -> Vec<u32> {
        cols.iter().map(|&j| self.free_params[j]).collect()
    }
}

/// What the diagnosis finds surplus: the constraints worth removing (`over`, with `in_over` its
/// set) and those the relations alone already say (`implied`: consistent, nothing to fix).
#[derive(Default)]
struct Redundancy {
    over: Vec<u32>,
    in_over: BTreeSet<u32>,
    implied: Vec<u32>,
}

impl Redundancy {
    /// The matching's reading: each constraint with a row in the over-determined block, in row
    /// order, but none of the `intrinsic` rows nobody can remove.
    fn matched(m: &Matched, intrinsic: &BTreeSet<u32>) -> Redundancy {
        let mut out = Redundancy::default();
        for &r in &m.dm.over_rows {
            let c = m.row_c[r];
            if !intrinsic.contains(&c) {
                out.add_over(c);
            }
        }
        out
    }

    fn add_over(&mut self, c: u32) {
        if self.in_over.insert(c) {
            self.over.push(c);
        }
    }

    /// `implied` and `over` from a numeric reading's dependencies, by whichever reading found
    /// them.  W is every dependency at this configuration, so it outranks the matching here
    /// exactly as the numeric rank does: the structural over-block is generic, and where a
    /// theorem tips the count it blames a whole block — a rectangle with three surplus
    /// perpendiculars indicts its two side lengths — so `over` is rebuilt from W's reading alone.
    fn reread(&mut self, d: numeric::Dependencies) {
        self.implied = d.implied;
        self.over.clear();
        self.in_over.clear();
        for c in d.removable {
            if !self.implied.contains(&c) {
                self.add_over(c);
            }
        }
    }

    /// File what the readings after the numeric one found, and give back what is `expected`.
    ///
    /// A dependency whose rows do not hold is no theorem: two tangencies of one pair that cannot
    /// both be true are dependent in W and carry no dimension, so W's reading files them as
    /// `implied` — "consistent, nothing to fix" — beside the conflict that names them.  What is
    /// `contested` (violated, or found in the conflict set) is a surplus and is said so.
    ///
    /// What a plane's tangency to a set makes dependent of its own rows is how it is `stated`,
    /// and said nowhere (#148): taken out of both lists and returned, in their order — but a row
    /// that does not hold is a conflict's, said as one, and stays.
    fn file(&mut self, contested: impl Fn(&u32) -> bool, stated: impl Fn(u32) -> bool) -> Vec<u32> {
        if self.implied.iter().any(&contested) {
            let (moved, kept): (Vec<u32>, Vec<u32>) =
                self.implied.iter().copied().partition(|c| contested(c));
            self.implied = kept;
            for c in moved {
                self.add_over(c);
            }
        }
        let expected: Vec<u32> = self.implied.iter().chain(&self.over).copied()
            .filter(|c| !contested(c) && stated(*c))
            .collect();
        if !expected.is_empty() {
            let set: BTreeSet<u32> = expected.iter().copied().collect();
            self.implied.retain(|c| !set.contains(c));
            self.over.retain(|c| !set.contains(c));
            self.in_over.retain(|c| !set.contains(c));
        }
        expected
    }
}

/// Whether constraint `c` is a plane's tangency to a set, stated toward an axis (#148).
fn touches_axis(sk: &Sketch, c: u32) -> bool {
    sk.constraint(c)
        .and_then(|c| c.along)
        .is_some_and(|d| matches!(sk.duals[d].toward, crate::model::Toward::Axis(_)))
}

/// The constraint graph's connected components, each with its own DOF accounting, largest first.
fn components(m: &Matched) -> Vec<Component> {
    let comps = graph::bipartite_components(&m.adj, m.n_cols);
    let n_comp = comps.count;
    let mut comp_params: Vec<Vec<u32>> = vec![Vec::new(); n_comp];
    let mut comp_cs: Vec<Vec<u32>> = vec![Vec::new(); n_comp];
    let mut comp_rank = vec![0usize; n_comp];
    for j in 0..m.n_cols {
        comp_params[comps.comp_col[j]].push(m.free_params[j]);
        if m.dm.mate_col[j] >= 0 {
            comp_rank[comps.comp_col[j]] += 1;
        }
    }
    let mut comp_seen: Vec<BTreeSet<u32>> = vec![BTreeSet::new(); n_comp];
    for r in 0..m.n_rows() {
        let (c, k) = (m.row_c[r], comps.comp_row[r]);
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
    components
}

/// What is said whatever the rank: a corner round that does not fit, said by name before the
/// profile it spoils (#576), a solid that is refused, and a load no solid bears.
fn standing_warnings(sk: &Sketch) -> Vec<String> {
    let mut warnings: Vec<String> = crate::model::overruns(sk);
    for (i, s) in sk.solids.iter().enumerate() {
        if !matches!(s.def, crate::model::SolidDef::Body { .. }) {
            if let Err(message) = crate::solid::validate(sk, i) { warnings.push(message); }
        }
    }
    warnings.extend(crate::solid::bearing_errors(sk).into_iter().map(|(_, message)| message));
    warnings
}

/// Where a drawing with violated rows stands: what of it no solve can meet, or whether it is
/// consistent and merely unsolved.
#[derive(Default)]
struct Reach {
    /// The driven deflections out of reach, each with why (`unmet`).
    out_of_reach: Vec<(u32, String)>,
    /// A scratch solve settled: the constraints are consistent and the pose is merely
    /// *unsolved* — no conflict, no culprits.
    stalled: bool,
    /// A scratch solve failed too, with nothing out of reach: a conflict search that then finds
    /// nothing means `stalled`.
    unsettled: bool,
}

/// A violated row is evidence about the *geometry* only at a stationary point — where the
/// solver could go no further, what is left is what the constraints cannot agree on.  Anywhere
/// else it is evidence about the solve, and the diagnosis finds out which it is for itself
/// rather than guessing: a scratch copy is solved.  If that solves, the constraints are
/// consistent and the caller's pose is merely *unsolved* — no conflict, no culprits.  If it does
/// not, the search for the minimal conflict set is the last word (`conflicts`): it proves a
/// conflict by finding the subset it cannot add to a feasible pose, and an empty answer from a
/// pose that was never stationary means it found every constraint feasible after all.  The
/// search runs from the caller's pose, not the scratch one — a failed solve of the whole system
/// has pulled everything toward the impossible row, and the trials warm-start better from where
/// the drawing was.  Read as a conflict directly, a solve that ran out on a consistent four-bar
/// named three innocent statements to delete (issue #43).  The extra solve is paid only on this
/// path, which the app is rarely on (it solves before it diagnoses).
///
/// A driven deflection the solve left at the extreme of what drives it, or at the edge of the
/// mesh it is read on, short of what it states, is the verdict before either (#510 P7, S2;
/// `unmet`): no solve from here meets it, and every trial of the search would factor its
/// analysis at every step to find the set it already is — seconds of a page's thread, an edit at
/// a time.  Read where the drawing stands, or, where that is no stationary point (a page puts a
/// failed solve's pose back), where the scratch solve stopped.
fn reach(sk: &Sketch, sys: &mut System, violated: &[u32], tol: f64) -> Reach {
    let mut out = Reach::default();
    if violated.is_empty() {
        return out;
    }
    let z = sys.z0(sk);
    // where the solve that failed on `sys` began, it said why already
    let stop = if sys.stopped_from.as_deref() == Some(&sk.get_x()[..]) {
        sys.stop.clone()
    } else {
        sys.stop_at(sk, &z, tol)
    };
    out.out_of_reach = unmet(sk, stop);
    if out.out_of_reach.is_empty() && !sys.stationary(&z) {
        let mut scratch = sk.clone();
        let mut ssys = System::new(&scratch);
        if ssys.solve(&mut scratch, SolveOpts::default()).settled() {
            out.stalled = true;
        } else {
            out.out_of_reach = unmet(sk, ssys.stop.clone());
            out.unsettled = out.out_of_reach.is_empty();
        }
    }
    out
}

/// The bounds that do not hold, and those that hold at their edge.
fn bounds(sk: &Sketch) -> (Vec<u32>, Vec<u32>) {
    let (mut violated, mut active) = (Vec::new(), Vec::new());
    let tol = crate::solve::bound_tol(sk);
    for c in sk.constraints.iter().filter(|c| c.bound.is_some()) {
        match crate::solve::bound_reading(sk, c, tol) {
            Some(r) if !r.holds => violated.push(c.id),
            Some(r) if r.edge => active.push(c.id),
            _ => {}
        }
    }
    (violated, active)
}

/// The conflict set, if there is one to name — what is out of `reach`, else the minimal conflict
/// set the search proves among the suspects — and whether the drawing is consistent and merely
/// unsolved: as `reach` found, or where the search finds no conflict from a pose a scratch solve
/// could not settle either.
fn conflicts(
    sk: &mut Sketch,
    opts: &DiagnoseOptions,
    structural_over: Vec<u32>,
    over: &[u32],
    violated: &[u32],
    intrinsic: &BTreeSet<u32>,
    reach: &Reach,
) -> (Option<Vec<u32>>, bool) {
    if !reach.out_of_reach.is_empty() {
        return (Some(reach.out_of_reach.iter().map(|(c, _)| *c).collect()), reach.stalled);
    }
    // a pose no solve settled at is searched too where every row reads as holding there: what
    // reads as holding at that pose's extent may not hold at the drawing's (#449)
    let search = !violated.is_empty() || !opts.settled;
    if reach.stalled || !opts.conflicts.unwrap_or(search) {
        return (None, reach.stalled);
    }
    // Candidates = the structurally over-determined block (where a redundancy must live)
    // *together with* whatever is actually violated.  Not just the over-block: a consistent
    // duplicate (two Horizontals on one line) makes an over-block with nothing to do with an
    // infeasibility elsewhere, and confining the search to it pins the blame on the harmless
    // pair — the real conflict is then in the constraints held fixed, so no candidate can be
    // satisfied and the first one tried wins.  Everything else stays fixed, so the result is
    // minimal "among the suspects", and the filter costs |candidates| solves, not |all|.
    let mut cands = structural_over;
    for &c in over.iter().chain(violated) {
        if !cands.contains(&c) {
            cands.push(c);
        }
    }
    let set = minimal_conflict_set(sk, Some(&cands), opts.tol, Method::DogLeg, 60);
    // the search found every constraint feasible from a pose that was never stationary:
    // consistent, and merely unsolved
    let stalled = set.is_empty() && reach.unsettled;
    // an intrinsic row may be *in* the set the search proves — the radius an arc's ends fix
    // is what a stray dimension on it contradicts — but it is nobody's statement and
    // nothing anyone can remove, so the culprits named are the document's own (#45.6)
    (Some(set.into_iter().filter(|c| !intrinsic.contains(c)).collect()), stalled)
}

/// Each primitive's state, by the worst any constraint touching it (or a child of it) says:
/// `blamed` for a conflict, `in_over`, or a parameter in `under` — and no parent better than its
/// worst child.
fn entity_states(
    sk: &Sketch,
    blamed: &BTreeSet<u32>,
    in_over: &BTreeSet<u32>,
    under: &BTreeSet<u32>,
) -> BTreeMap<EntRef, State> {
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
    // a ring's turned copy is as free as what it is worked out from
    let derived: BTreeMap<u32, Vec<u32>> =
        sk.derived().into_iter().map(|d| (d.param, d.terms.iter().map(|t| t.0).collect())).collect();
    let free_of = |p: u32| -> Vec<u32> { derived.get(&p).cloned().unwrap_or_else(|| vec![p]) };
    for &e in &ents {
        let empty = Vec::new();
        let cs = touched.get(&e).unwrap_or(&empty);
        let st = if cs.iter().any(|c| blamed.contains(c)) {
            State::Conflict
        } else if cs.iter().any(|c| in_over.contains(c)) {
            State::Over
        } else if sk.entity_params(e).iter().flat_map(|&p| free_of(p)).any(|p| under.contains(&p)) {
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
    state
}

/// The drawing's one word.  `over` only when something is actually worth removing: a rank
/// deficiency alone is not enough, since a dependency can be shared between a user constraint
/// that is still doing work and a primitive's own definition, leaving nothing to delete — and a
/// relation-only theorem leaves `implied` constraints that *could* go but need not, which is not
/// "over" either.
fn status(
    settled: bool,
    stalled: bool,
    violated: &[u32],
    conflicts: Option<&[u32]>,
    over: &[u32],
    dof: i64,
) -> State {
    // a pose no solve settled at, every row reading as holding and no conflict found: unsolved,
    // and no more is known (#449)
    let unsolved =
        !settled && violated.is_empty() && conflicts.map(|c| c.is_empty()).unwrap_or(true);
    if stalled || unsolved {
        State::Unsolved
    } else if conflicts.is_some_and(|c| !c.is_empty()) || !violated.is_empty() {
        State::Conflict
    } else if !over.is_empty() {
        State::Over
    } else if dof > 0 {
        State::Under
    } else {
        State::Well
    }
}

/// Each declared parameter's ledger (`Driven`): its readers, and — where the drawing is small
/// enough to factor (`numeric`) — the rank their rows add to the rest's, read on the conditioned
/// Jacobian where the drawing stands.
fn driven(sk: &Sketch, sys: &mut System, row_c: &[u32], numeric: bool) -> Vec<Driven> {
    // the Jacobian and its rank, read once for every parameter that is read
    let mut whole: Option<(crate::system::Conditioned, usize)> = None;
    sk.parameters.keys().map(|name| {
        let readers = crate::follow::readers(sk, name);
        let dof = (numeric && !readers.is_empty() && sys.n_free > 0).then(|| {
            let (j, full) = whole.get_or_insert_with(|| {
                let z = sys.z0(sk);
                let j = sys.conditioned(&z);
                let full = j.rank_rrqr(RANK_TOL);
                (j, full)
            });
            let rest: Vec<usize> = (0..j.rows()).filter(|&r| !readers.contains(&row_c[r]))
                .collect();
            *full - j.select_rows(&rest).rank_rrqr(RANK_TOL)
        });
        Driven { name: name.clone(), readers, dof }
    }).collect()
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
