//! JSON views of the analysis results.
//!
//! Diagnosis, witness reports, plans and constraint graphs are rich, ragged structures.  Encoding
//! them here — once — is what lets the Python and TypeScript packages stay thin bindings: they
//! parse one document instead of reimplementing a dozen accessors, and both see exactly the same
//! field names.  Hot-path numbers (residuals, Jacobians, drag frames) never go through here.

use crate::cgraph::{ConstraintGraph, El, ElKind};
use crate::constraints::{Arg, CKind, Constraint, ALL_KINDS};
use crate::decompose::{Plan, PlanResult};
use crate::diagnose::{summary as diag_summary, Diagnosis};
use crate::homotopy::Alternative;
use crate::io::describe;
use crate::json::{object, Json};
use crate::kernels::KERNELS;
use crate::model::{EntRef, Sketch};
use crate::solve::SolveResult;
use crate::witness::WitnessReport;

fn ids(v: &[u32]) -> Json {
    Json::Arr(v.iter().map(|&x| Json::Int(x as i64)).collect())
}

fn idx(v: &[usize]) -> Json {
    Json::Arr(v.iter().map(|&x| Json::Int(x as i64)).collect())
}

fn floats(v: &[f64]) -> Json {
    Json::Arr(v.iter().map(|&x| Json::Num(x)).collect())
}

pub fn ent_json(e: EntRef) -> Json {
    Json::Arr(vec![Json::Str(e.kind.as_str().to_string()), Json::Int(e.idx as i64)])
}

pub fn el_json(e: El) -> Json {
    let k = match e.kind {
        ElKind::P => "P",
        ElKind::L => "L",
        ElKind::V => "V",
    };
    Json::Arr(vec![Json::Str(k.to_string()), Json::Int(e.idx as i64)])
}

pub fn el_from_json(v: &Json) -> Option<El> {
    let a = v.arr();
    if a.len() != 2 {
        return None;
    }
    let kind = match a[0].as_str() {
        "P" => ElKind::P,
        "L" => ElKind::L,
        "V" => ElKind::V,
        _ => return None,
    };
    Some(El::new(kind, a[1].as_i64() as i32))
}

pub fn solve_result_json(r: &SolveResult) -> Json {
    object([
        ("success", r.success.into()),
        ("status", (r.status as i64).into()),
        ("message", r.message.clone().into()),
        ("residualNorm", r.residual_norm.into()),
        ("maxResidual", r.max_residual.into()),
        ("nfev", (r.nfev as i64).into()),
        ("njev", (r.njev as i64).into()),
        ("timeS", r.time_s.into()),
        ("method", r.method.clone().into()),
        ("iterations", (r.iterations as i64).into()),
        ("rank", r.rank.map(|v| v as i64).into()),
    ])
}

pub fn plan_result_json(r: &PlanResult) -> Json {
    object([
        ("success", r.success.into()),
        ("maxResidual", r.max_residual.into()),
        ("fellBack", r.fell_back.into()),
        ("nSteps", (r.n_steps as i64).into()),
        ("numeric", r.numeric.as_ref().map(solve_result_json).unwrap_or(Json::Null)),
    ])
}

pub fn witness_json(sk: &Sketch, w: &WitnessReport) -> Json {
    let deps: Vec<Json> = w
        .dependencies
        .iter()
        .map(|d| {
            object([
                ("constraint", (d.constraint as i64).into()),
                ("impliedBy", ids(&d.implied_by)),
                ("theorem", d.theorem.into()),
                (
                    "describe",
                    sk.constraint(d.constraint).map(describe).unwrap_or_default().into(),
                ),
            ])
        })
        .collect();
    let motions: Vec<Json> = w
        .motions
        .iter()
        .map(|m| {
            object([
                ("velocity", floats(&m.velocity)),
                ("rigid", m.rigid.into()),
                ("movingParams", ids(&w.moving_params(m, 1e-3))),
            ])
        })
        .collect();
    object([
        ("xWitness", floats(&w.x_witness)),
        ("usedCurrent", w.used_current.into()),
        ("numericRank", (w.numeric_rank as i64).into()),
        ("dependencies", Json::Arr(deps)),
        ("motions", Json::Arr(motions)),
        ("movable", idx(&w.movable)),
        ("params", ids(&w.params)),
        ("warnings", Json::Arr(w.warnings.iter().map(|s| Json::Str(s.clone())).collect())),
        ("nDof", (w.n_dof() as i64).into()),
        ("nInternalDof", (w.n_internal_dof() as i64).into()),
        ("summary", w.summary().into()),
    ])
}

pub fn diagnosis_json(sk: &Sketch, d: &Diagnosis) -> Json {
    let components: Vec<Json> = d
        .components
        .iter()
        .map(|c| {
            object([
                ("params", ids(&c.params)),
                ("constraints", ids(&c.constraints)),
                ("structuralRank", (c.structural_rank as i64).into()),
                ("dof", c.dof.into()),
            ])
        })
        .collect();
    let entity_state: Vec<Json> = d
        .entity_state
        .iter()
        .map(|(&e, &s)| {
            Json::Arr(vec![
                Json::Str(e.kind.as_str().to_string()),
                Json::Int(e.idx as i64),
                Json::Str(s.as_str().to_string()),
            ])
        })
        .collect();
    object([
        ("nParams", (d.n_params as i64).into()),
        ("nEquations", (d.n_equations as i64).into()),
        ("structuralRank", (d.structural_rank as i64).into()),
        ("numericRank", d.numeric_rank.map(|v| v as i64).into()),
        ("numericSkipped", d.numeric_skipped.into()),
        ("geometricDependency", (d.geometric_dependency as i64).into()),
        ("over", ids(&d.over)),
        ("underParams", ids(&d.under_params)),
        ("structuralUnderParams", ids(&d.structural_under_params)),
        ("components", Json::Arr(components)),
        ("entityState", Json::Arr(entity_state)),
        (
            "rigidClusters",
            Json::Arr(d.rigid_clusters.iter().map(|c| idx(c)).collect()),
        ),
        ("redundantDistances", ids(&d.redundant_distances)),
        ("violated", ids(&d.violated)),
        ("conflicts", d.conflicts.as_ref().map(|c| ids(c)).unwrap_or(Json::Null)),
        ("warnings", Json::Arr(d.warnings.iter().map(|s| Json::Str(s.clone())).collect())),
        ("witness", d.witness.as_ref().map(|w| witness_json(sk, w)).unwrap_or(Json::Null)),
        ("dof", d.dof.into()),
        ("structuralDof", d.structural_dof.into()),
        ("nRedundant", d.n_redundant.into()),
        ("structuralNRedundant", d.structural_n_redundant.into()),
        ("status", d.status.as_str().into()),
        ("summary", diag_summary(d).into()),
    ])
}

pub fn graph_json(g: &ConstraintGraph) -> Json {
    let edges: Vec<Json> = g
        .edges
        .iter()
        .map(|e| {
            object([
                ("kind", if e.kind == crate::cgraph::EdgeKind::Pp { "PP" } else { "PL" }.into()),
                ("a", el_json(e.a)),
                ("b", el_json(e.b)),
                ("source", e.source.map(|c| c as i64).into()),
            ])
        })
        .collect();
    let dirs: Vec<Json> = g
        .dirs
        .iter()
        .map(|d| {
            object([
                ("a", el_json(d.a)),
                ("b", el_json(d.b)),
                ("phi", d.phi.into()),
                ("source", (d.source as i64).into()),
            ])
        })
        .collect();
    object([
        ("nPoints", (g.n_points() as i64).into()),
        ("members", Json::Arr(g.members.iter().map(|m| idx(m)).collect())),
        ("lines", idx(&g.lines)),
        ("virtuals", Json::Arr(g.virtuals.iter().map(|&(a, b)| Json::Arr(vec![el_json(a), el_json(b)])).collect())),
        ("edges", Json::Arr(edges)),
        ("dirs", Json::Arr(dirs)),
        ("unsupported", ids(&g.unsupported)),
        (
            "knownRadius",
            Json::Obj(
                g.known_radius.iter().map(|(&p, &v)| (p.to_string(), Json::Num(v))).collect(),
            ),
        ),
        ("groundPoints", idx(&g.ground_points)),
        ("passive", idx(&g.passive)),
        ("summary", g.summary().into()),
    ])
}

pub fn plan_json(p: &Plan) -> Json {
    let steps: Vec<Json> = p
        .steps
        .iter()
        .map(|st| {
            object([
                ("ids", idx(&st.ids)),
                (
                    "ppp",
                    st.ppp
                        .map(|(a, b, c)| Json::Arr(vec![el_json(a), el_json(b), el_json(c)]))
                        .unwrap_or(Json::Null),
                ),
                ("branch", st.branch.map(|b| b as i64).into()),
                ("key", st.key(&p.graph).map(Json::Str).unwrap_or(Json::Null)),
                ("nPairs", (st.pairs.len() as i64).into()),
                ("nDpairs", (st.dpairs.len() as i64).into()),
            ])
        })
        .collect();
    object([
        ("leaves", (p.leaves.len() as i64).into()),
        ("steps", Json::Arr(steps)),
        ("roots", idx(&p.roots)),
        ("fullyDecomposed", p.fully_decomposed().into()),
        ("stickyBranches", p.sticky_branches.into()),
        ("summary", p.summary().into()),
        (
            "pppTriangles",
            Json::Arr(
                crate::decompose::ppp_triangles(p)
                    .iter()
                    .map(|&(a, b, c)| idx(&[a, b, c]))
                    .collect(),
            ),
        ),
    ])
}

pub fn alternatives_json(alts: &[Alternative]) -> Json {
    Json::Arr(
        alts.iter()
            .map(|a| {
                object([
                    ("u", floats(&a.u)),
                    ("distance", a.distance.into()),
                    (
                        "location",
                        a.location.map(|(x, y)| floats(&[x, y])).unwrap_or(Json::Null),
                    ),
                    ("isCurrent", a.is_current().into()),
                ])
            })
            .collect(),
    )
}

pub fn alternative_from_json(v: &Json) -> Alternative {
    Alternative {
        u: v.get("u").map(|a| a.arr().iter().map(|x| x.as_f64()).collect()).unwrap_or_default(),
        distance: v.get("distance").map(|x| x.as_f64()).unwrap_or(0.0),
        location: None,
    }
}

fn arg_json_value(a: &Arg) -> Json {
    match a {
        Arg::Ent(e) => ent_json(*e),
        Arg::Num(v) => Json::Num(*v),
        Arg::Int(v) => Json::Int(*v),
        Arg::Bool(b) => Json::Bool(*b),
        Arg::Str(s) => Json::Str(s.clone()),
    }
}

/// One constraint as the bindings see it: identity, type, spec-ordered arguments and flags.
pub fn constraint_json(sk: &Sketch, c: &Constraint) -> Json {
    let args: Vec<Json> = c
        .args
        .iter()
        .map(|a| match a {
            Arg::Ent(e) => ent_json(*e),
            Arg::Num(v) => Json::Num(*v),
            Arg::Int(v) => Json::Int(*v),
            Arg::Bool(b) => Json::Bool(*b),
            Arg::Str(s) => Json::Str(s.clone()),
        })
        .collect();
    object([
        ("id", (c.id as i64).into()),
        ("type", c.type_name().into()),
        ("args", Json::Arr(args)),
        ("soft", c.soft.into()),
        ("intrinsic", c.intrinsic.into()),
        ("nResiduals", (c.n_residuals() as i64).into()),
        ("describe", describe(c).into()),
        ("error", c.error(sk).into()),
    ])
}

pub fn constraints_json(sk: &Sketch) -> Json {
    Json::Arr(sk.constraints.iter().map(|c| constraint_json(sk, c)).collect())
}

/// The constraint-type registry: what a front end needs to build a toolbar, a constraint list and
/// a value editor without knowing any type by name.
pub fn registry_json() -> Json {
    let types: Vec<Json> = ALL_KINDS
        .iter()
        .map(|&k| {
            let spec: Vec<Json> = k
                .spec()
                .iter()
                .map(|(n, sk)| {
                    Json::Arr(vec![Json::Str(n.to_string()), Json::Str(sk.as_str().to_string())])
                })
                .collect();
            // null for an entity, and for an argument the core reads off the geometry — a
            // binding that substituted a constant for one of those would pick the branch itself
            let defaults: Vec<Json> = (0..k.spec().len())
                .map(|i| match k.spec()[i].1 {
                    s if s.is_entity() => Json::Null,
                    _ if k.infers_arg(i) => Json::Null,
                    _ => arg_json_value(&k.default_arg(i)),
                })
                .collect();
            object([
                ("name", k.name().into()),
                ("spec", Json::Arr(spec)),
                ("defaults", Json::Arr(defaults)),
                ("soft", k.soft_by_default().into()),
                ("commutative", k.commutative().into()),
                ("kernel", (k.kernel() as i64).into()),
            ])
        })
        .collect();
    let kernels: Vec<Json> = KERNELS
        .iter()
        .map(|k| {
            object([
                ("name", k.name.into()),
                ("nRes", (k.n_res as i64).into()),
                ("nPar", (k.n_par as i64).into()),
                ("nConst", (k.n_const as i64).into()),
            ])
        })
        .collect();
    object([("types", Json::Arr(types)), ("kernels", Json::Arr(kernels))])
}

/// Build a constraint from `{"type": ..., "args": [...], "soft": ?, "intrinsic": ?}`.
pub fn constraint_from_json(sk: &Sketch, v: &Json) -> Result<Constraint, String> {
    let name = v.get("type").map(|t| t.as_str().to_string()).unwrap_or_default();
    let kind =
        CKind::from_name(&name).ok_or_else(|| format!("unknown constraint type: {name}"))?;
    let spec = kind.spec();
    let empty = Json::Arr(Vec::new());
    let raw = v.get("args").unwrap_or(&empty).arr();
    let mut args = Vec::with_capacity(spec.len());
    for (i, (_, k)) in spec.iter().enumerate() {
        let a = raw.get(i).unwrap_or(&Json::Null);
        args.push(match (k, a) {
            (_, Json::Null) => kind.default_arg(i),
            (k, _) if k.is_entity() => {
                let arr = a.arr();
                let ek = crate::model::EntKind::parse(arr.first().map(|x| x.as_str()).unwrap_or(""))
                    .ok_or_else(|| "bad entity reference".to_string())?;
                Arg::Ent(EntRef::new(ek, arr.get(1).map(|x| x.as_i64()).unwrap_or(0) as usize))
            }
            (crate::constraints::SpecKind::Int, _) => Arg::Int(a.as_i64()),
            (crate::constraints::SpecKind::Bool, _) => Arg::Bool(a.as_bool()),
            (crate::constraints::SpecKind::Str, _) => Arg::Str(a.as_str().to_string()),
            _ => Arg::Num(a.as_f64()),
        });
    }
    // the tangencies read their branch off the sketch when none was supplied
    let omitted = raw.get(2).map(|x| matches!(x, Json::Null)).unwrap_or(true);
    if omitted && kind == CKind::TangentLineCircle {
        return Ok(Constraint::tangent_line_circle(sk, args[0].ent(), args[1].ent(), None));
    }
    if omitted && kind == CKind::TangentCircleCircle {
        return Ok(Constraint::tangent_circle_circle(sk, args[0].ent(), args[1].ent(), None));
    }
    let mut c = Constraint::new(kind, args);
    c.soft = v.get("soft").map(|x| x.as_bool()).unwrap_or(false) || kind.soft_by_default();
    c.intrinsic = v.get("intrinsic").map(|x| x.as_bool()).unwrap_or(false);
    Ok(c)
}
