//! What the drawing is: diagnosis, the graph algorithms, the witness, decomposition and homotopy.

use super::*;

/* -- diagnosis ------------------------------------------------------------- */

fn diagnose_options(v: &Json) -> DiagnoseOptions {
    let mut o = DiagnoseOptions::default();
    if let Some(x) = v.get("numeric") {
        if !matches!(x, Json::Null) {
            o.numeric = Some(x.as_bool());
        }
    }
    if let Some(x) = v.get("conflicts") {
        if !matches!(x, Json::Null) {
            o.conflicts = Some(x.as_bool());
        }
    }
    if let Some(x) = v.get("witness") {
        o.witness = x.as_bool();
    }
    if let Some(x) = v.get("tol") {
        o.tol = x.as_f64();
    }
    if let Some(x) = v.get("numericMax") {
        o.numeric_max = x.as_i64() as usize;
    }
    if let Some(x) = v.get("solidClaims") {
        o.solid_claims = x.as_bool();
    }
    if let Some(x) = v.get("settled") {
        o.settled = x.as_bool();
    }
    o
}

/* -- claims about solids, judged a pose at a time --------------------------- */

/// A drawing's claims about solids being judged (`diagnose::ClaimJudge`), with the drawing it
/// judges: a copy, so the judge outlives nothing it reads.
pub struct ClaimJudge {
    sketch: Sketch,
    judge: diagnose::ClaimJudge,
}

/// The drawing's claims about solids, judged or not: `[{ "stmt", "text" }]`, each claim's statement
/// and its words as the report says them (`claim a clear(1mm) b`) — what a page lists as waiting.
#[no_mangle]
pub unsafe extern "C" fn gcs_sketch_solid_claims_json(h: *mut Sketch) -> *mut u8 {
    json_out(move || {
        let s = sk(h);
        Json::Arr(s.solid_claims.iter().map(|c| json::object([
            ("stmt", Json::Int(c.stmt() as i64)),
            ("text", Json::Str(gcs_core::diagnose::describe_solid_claim(s, c))),
        ])).collect())
    })
}

/// Start judging the claims about solids of this sketch, at the pose it stands in.
#[no_mangle]
pub unsafe extern "C" fn gcs_claim_judge_new(h: *mut Sketch) -> *mut ClaimJudge {
    guard(std::ptr::null_mut(), move || {
        let sketch = sk(h).clone();
        let judge = diagnose::ClaimJudge::new(&sketch);
        Box::into_raw(Box::new(ClaimJudge { sketch, judge }))
    })
}

/// Judge the next pose: 1 once every claim is judged, 0 while some are not, −1 on a panic.
#[no_mangle]
pub unsafe extern "C" fn gcs_claim_judge_step(j: *mut ClaimJudge) -> i32 {
    guard(-1, move || {
        let j = &mut *j;
        j.judge.step(&j.sketch) as i32
    })
}

/// Where the judging stands: `{ "doing", "fraction", "claim", "claims", "pose", "poses", "stmt" }`
/// — the words a host shows (`ClaimProgress::doing`), and the claim in hand's statement or null.
#[no_mangle]
pub unsafe extern "C" fn gcs_claim_judge_progress(j: *mut ClaimJudge) -> *mut u8 {
    json_out(move || {
        let j = &*j;
        let p = j.judge.progress(&j.sketch);
        json::object([
            ("doing", Json::Str(p.doing())),
            ("fraction", Json::Num(p.fraction())),
            ("claim", Json::Int(p.claim as i64)),
            ("claims", Json::Int(p.claims as i64)),
            ("pose", Json::Int(p.pose as i64)),
            ("poses", Json::Int(p.poses as i64)),
            ("stmt", p.stmt.map_or(Json::Null, |s| Json::Int(s as i64))),
        ])
    })
}

/// The verdicts so far, each `report::solid_claim_json` with `"stmt"` (its statement) and
/// `"line"` (the report's own words, `report::solid_claim_text`) beside it.
#[no_mangle]
pub unsafe extern "C" fn gcs_claim_judge_verdicts_json(j: *mut ClaimJudge) -> *mut u8 {
    json_out(move || {
        let j = &*j;
        Json::Arr(j.judge.verdicts().iter().map(|v| {
            let mut o = gcs_core::report::solid_claim_json(v);
            o.set("stmt", Json::Int(v.stmt() as i64));
            o.set("line", Json::Str(gcs_core::report::solid_claim_text(v)));
            o
        }).collect())
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_claim_judge_free(j: *mut ClaimJudge) {
    guard((), move || {
        if !j.is_null() {
            drop(Box::from_raw(j));
        }
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_diagnose_json(
    h: *mut Sketch,
    opts: *const u8,
    opts_len: usize,
) -> *mut u8 {
    json_out(move || {
        let s = sk(h);
        let o = diagnose_options(&as_json(opts, opts_len));
        let d = diagnose::diagnose(s, o);
        report::diagnosis_json(s, &d)
    })
}

/// Diagnose reusing an already-compiled system (what the app does after a solve).
#[no_mangle]
pub unsafe extern "C" fn gcs_diagnose_with_json(
    h: *mut Sketch,
    sys: *mut System,
    opts: *const u8,
    opts_len: usize,
) -> *mut u8 {
    json_out(move || {
        let s = sk(h);
        let o = diagnose_options(&as_json(opts, opts_len));
        let d = diagnose::diagnose_with(s, &mut *sys, o);
        report::diagnosis_json(s, &d)
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_minimal_conflict_set_json(
    h: *mut Sketch,
    cands: *const u8,
    cands_len: usize,
    tol: f64,
) -> *mut u8 {
    json_out(move || {
        let s = sk(h);
        let cv = as_json(cands, cands_len);
        let list: Vec<u32> = cv.arr().iter().map(|v| v.as_i64() as u32).collect();
        let out = diagnose::minimal_conflict_set(
            s,
            if matches!(cv, Json::Null) { None } else { Some(&list) },
            tol,
            Method::DogLeg,
            60,
        );
        Json::Arr(out.iter().map(|&c| Json::Int(c as i64)).collect())
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_violated_json(h: *mut Sketch, tol: f64) -> *mut u8 {
    json_out(move || {
        let s = sk(h);
        let mut sys = System::new(s);
        let v = diagnose::violated_constraints(s, &mut sys, tol);
        Json::Arr(v.iter().map(|&c| Json::Int(c as i64)).collect())
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_distance_rigidity_json(h: *mut Sketch) -> *mut u8 {
    json_out(move || {
        let (clusters, redundant) = diagnose::distance_rigidity(sk(h));
        json::object([
            (
                "clusters",
                Json::Arr(
                    clusters
                        .iter()
                        .map(|c| Json::Arr(c.iter().map(|&p| Json::Int(p as i64)).collect()))
                        .collect(),
                ),
            ),
            ("redundant", Json::Arr(redundant.iter().map(|&c| Json::Int(c as i64)).collect())),
        ])
    })
}

/* -- load scenarios --------------------------------------------------------- */

/// What each load scenario of the document says of the drawing as it stands
/// (`report::scenarios_json`): its name, its words, and each requirement's answer — its value,
/// whether it holds, and the sentence `solventc` prints (`` scenario `shelf`: safety(bracket) is
/// 1.22695: claimed >= 1, kept ``).  `[]` for a document with none.
#[no_mangle]
pub unsafe extern "C" fn gcs_elab_scenarios_json(h: *mut Elaborated, s: *mut Sketch) -> *mut u8 {
    json_out(move || gcs_core::report::scenarios_json(sk(s), &(*h).scenarios))
}

/* -- pure graph algorithms -------------------------------------------------- */

fn adj_from(v: &Json) -> Vec<Vec<usize>> {
    v.arr()
        .iter()
        .map(|r| r.arr().iter().map(|c| c.as_i64() as usize).collect())
        .collect()
}

fn i_arr(v: &[i32]) -> Json {
    Json::Arr(v.iter().map(|&x| Json::Int(x as i64)).collect())
}

fn u_arr(v: &[usize]) -> Json {
    Json::Arr(v.iter().map(|&x| Json::Int(x as i64)).collect())
}

/// Maximum bipartite matching: `{mateL, mateR}` with -1 for unmatched.
#[no_mangle]
pub unsafe extern "C" fn gcs_hopcroft_karp_json(
    adj: *const u8,
    adj_len: usize,
    n_right: i32,
) -> *mut u8 {
    json_out(move || {
        let a = adj_from(&as_json(adj, adj_len));
        let (l, r) = gcs_core::graph::hopcroft_karp(&a, n_right as usize);
        json::object([("mateL", i_arr(&l)), ("mateR", i_arr(&r))])
    })
}

/// The coarse Dulmage–Mendelsohn decomposition.
#[no_mangle]
pub unsafe extern "C" fn gcs_dulmage_mendelsohn_json(
    adj: *const u8,
    adj_len: usize,
    n_cols: i32,
) -> *mut u8 {
    json_out(move || {
        let a = adj_from(&as_json(adj, adj_len));
        let d = gcs_core::graph::dulmage_mendelsohn(&a, n_cols as usize);
        json::object([
            ("mateRow", i_arr(&d.mate_row)),
            ("mateCol", i_arr(&d.mate_col)),
            ("overRows", u_arr(&d.over_rows)),
            ("overCols", u_arr(&d.over_cols)),
            ("underRows", u_arr(&d.under_rows)),
            ("underCols", u_arr(&d.under_cols)),
            ("wellRows", u_arr(&d.well_rows)),
            ("wellCols", u_arr(&d.well_cols)),
            ("rank", Json::Int(d.rank as i64)),
            ("nRedundant", Json::Int(d.n_redundant)),
            ("nFree", Json::Int(d.n_free)),
        ])
    })
}

/// The (2,3) pebble game on `n` vertices with `[[u, v], ...]` edges.
#[no_mangle]
pub unsafe extern "C" fn gcs_pebble_game_json(
    n: i32,
    edges: *const u8,
    edges_len: usize,
) -> *mut u8 {
    json_out(move || {
        let e: Vec<(usize, usize)> = as_json(edges, edges_len)
            .arr()
            .iter()
            .map(|p| {
                let a = p.arr();
                (a[0].as_i64() as usize, a[1].as_i64() as usize)
            })
            .collect();
        let r = gcs_core::graph::pebble_game(n as usize, &e);
        json::object([
            ("independent", u_arr(&r.independent)),
            ("redundant", u_arr(&r.redundant)),
            ("components", Json::Arr(r.components.iter().map(|c| u_arr(c)).collect())),
            ("dof", Json::Int(r.dof as i64)),
            ("isRigid", Json::Bool(r.is_rigid())),
        ])
    })
}

/// Connected components of a bipartite graph.
#[no_mangle]
pub unsafe extern "C" fn gcs_bipartite_components_json(
    adj: *const u8,
    adj_len: usize,
    n_cols: i32,
) -> *mut u8 {
    json_out(move || {
        let a = adj_from(&as_json(adj, adj_len));
        let c = gcs_core::graph::bipartite_components(&a, n_cols as usize);
        json::object([
            ("compRow", u_arr(&c.comp_row)),
            ("compCol", u_arr(&c.comp_col)),
            ("count", Json::Int(c.count as i64)),
        ])
    })
}

/// A random Laman graph's edges by Henneberg construction — the property-test generator.
#[no_mangle]
pub extern "C" fn gcs_henneberg_edges_json(n: i32, seed: u32) -> *mut u8 {
    json_out(move || {
        let mut rng = gcs_core::rng::Rng::new(seed);
        let e = gcs_core::fixtures::henneberg_edges(n.max(2) as usize, &mut rng);
        Json::Arr(
            e.iter()
                .map(|&(a, b)| Json::Arr(vec![Json::Int(a as i64), Json::Int(b as i64)]))
                .collect(),
        )
    })
}

/* -- witness --------------------------------------------------------------- */

#[no_mangle]
pub unsafe extern "C" fn gcs_witness_json(h: *mut Sketch, seed: u32) -> *mut u8 {
    json_out(move || {
        let s = sk(h);
        let w = witness::analyze(s, None, seed);
        report::witness_json(s, &w)
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_make_witness(h: *mut Sketch, seed: u32, out: *mut f64) {
    guard((), move || {
        let x = witness::make_witness(sk(h), seed, 0.05, 1e-8);
        write(out, &x);
    })
}

/* -- decomposition --------------------------------------------------------- */

#[no_mangle]
pub unsafe extern "C" fn gcs_graph_json(h: *mut Sketch) -> *mut u8 {
    json_out(move || {
        report::graph_json(&cgraph::build(sk(h)))
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_plan_solver_new(h: *mut Sketch, sticky: i32) -> *mut PlanSolver {
    guard(std::ptr::null_mut(), move || {
        Box::into_raw(Box::new(PlanSolver::new(sk(h), sticky != 0)))
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_plan_solver_free(p: *mut PlanSolver) {
    guard((), move || {
        if !p.is_null() {
            drop(Box::from_raw(p));
        }
    })
}

/// The verification System the plan solver owns — borrowed, never freed by the caller.
#[no_mangle]
pub unsafe extern "C" fn gcs_plan_solver_system(p: *mut PlanSolver) -> *mut System {
    guard(std::ptr::null_mut(), move || {
        &mut (*p).system as *mut System
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_plan_solver_plan_json(p: *mut PlanSolver) -> *mut u8 {
    json_out(move || {
        report::plan_json(&(*p).plan)
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_plan_solver_graph_json(p: *mut PlanSolver) -> *mut u8 {
    json_out(move || {
        report::graph_json(&(*p).plan.graph)
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_plan_solver_solve(
    p: *mut PlanSolver,
    h: *mut Sketch,
    tol: f64,
    fallback: i32,
    method: i32,
) -> *mut u8 {
    json_out(move || {
        let m = if method == 1 { Method::Lm } else { Method::DogLeg };
        let r = (*p).solve(sk(h), tol, fallback != 0, m);
        report::plan_result_json(&r)
    })
}

/// Flip the closed-form constructions that place a point element; returns how many.
#[no_mangle]
pub unsafe extern "C" fn gcs_plan_solver_flip(
    p: *mut PlanSolver,
    h: *mut Sketch,
    point: i32,
) -> i32 {
    guard(-1, move || {
        let s = sk(h);
        let ps = &mut *p;
        let cls = ps.plan.graph.point_of[point as usize];
        ps.flip(s, El::p(cls)) as i32
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_plan_solver_sticky(p: *mut PlanSolver, sticky: i32) {
    guard((), move || {
        (*p).plan.sticky_branches = sticky != 0;
    })
}

/// Replay the plan on the current geometry (no solving, no fallback).
#[no_mangle]
pub unsafe extern "C" fn gcs_plan_solver_execute(p: *mut PlanSolver, h: *mut Sketch) {
    guard((), move || {
        decompose::execute(&mut (*p).plan, sk(h), None);
    })
}

/// The point element index of a sketch point (its coincidence class), or -1.
#[no_mangle]
pub unsafe extern "C" fn gcs_plan_solver_point_element(p: *mut PlanSolver, point: i32) -> i32 {
    guard(-1, move || {
        let ps = &*p;
        ps.plan.graph.point_of.get(point as usize).map(|&c| c as i32).unwrap_or(-1)
    })
}

/// The merges that place a point: closed-form ones where it is the constructed apex, else the
/// numeric merges that share it.  Writes step indices; returns how many.
#[no_mangle]
pub unsafe extern "C" fn gcs_plan_steps_placing(
    p: *mut PlanSolver,
    point: i32,
    out: *mut i32,
) -> i32 {
    guard(-1, move || {
        let ps = &*p;
        let Some(&cls) = ps.plan.graph.point_of.get(point as usize) else { return 0 };
        let idxs = ps.plan.steps_placing(El::p(cls));
        write_map(out, &idxs, |&s| s as i32);
        idxs.len() as i32
    })
}

/* -- homotopy -------------------------------------------------------------- */

#[no_mangle]
pub unsafe extern "C" fn gcs_enumerate_step_json(
    p: *mut PlanSolver,
    h: *mut Sketch,
    step: i32,
    locate_point: i32,
    seed: u32,
    max_paths: i32,
) -> *mut u8 {
    json_out(move || {
        let s = sk(h);
        let ps = &mut *p;
        let locate = if locate_point < 0 {
            None
        } else {
            ps.plan.graph.point_of.get(locate_point as usize).map(|&c| El::p(c))
        };
        let alts = homotopy::enumerate_step(
            &mut ps.plan,
            s,
            step as usize,
            EnumerateOptions {
                locate,
                seed,
                max_paths: max_paths.max(1) as usize,
                ..Default::default()
            },
        );
        report::alternatives_json(&alts)
    })
}

#[no_mangle]
pub unsafe extern "C" fn gcs_apply_alternative(
    p: *mut PlanSolver,
    h: *mut Sketch,
    step: i32,
    alt: *const u8,
    alt_len: usize,
) {
    guard((), move || {
        let a = report::alternative_from_json(&as_json(alt, alt_len));
        homotopy::apply_alternative(&mut (*p).plan, sk(h), step as usize, &a);
    })
}
