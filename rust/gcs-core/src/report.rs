//! JSON views of the analysis results.
//!
//! Diagnosis, witness reports, plans and constraint graphs are rich, ragged structures.  Encoding
//! them here — once — is what lets the TypeScript package stay a thin binding: it parses one
//! document instead of reimplementing a dozen accessors, and a second binding would see exactly
//! the same field names.  Hot-path numbers (residuals, Jacobians, drag frames) never go through
//! here.

use crate::callout::{self, Callout, Seg};
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
        ("blocked", (w.blocked as i64).into()),
        ("dependencies", Json::Arr(deps)),
        ("motions", Json::Arr(motions)),
        ("movable", idx(&w.movable)),
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
        ("shaky", (d.shaky as i64).into()),
        ("over", ids(&d.over)),
        ("implied", ids(&d.implied)),
        ("claimsTheorem", ids(&d.claims_theorem)),
        ("claimsViolated", ids(&d.claims_violated)),
        ("claimsConsuming", ids(&d.claims_consuming)),
        // the claims about solids, each with what was measured and how far the faceting could be
        // wrong — there is no `consuming` here, since a solid claim compiles no row to consume
        // rank with (§9.8)
        (
            "solidClaims",
            Json::Arr(
                d.solid_claims
                    .iter()
                    .map(|v| {
                        let mut o = object([
                            ("statement", Json::Str(v.text.clone())),
                            ("measured", if v.measured.is_finite() { Json::Num(v.measured) } else { Json::Null }),
                            ("tolerance", Json::Num(v.tolerance)),
                            (
                                "verdict",
                                Json::Str(
                                    match v.holds {
                                        Some(true) => "holds",
                                        Some(false) => "refuted",
                                        None => "undecided",
                                    }
                                    .to_string(),
                                ),
                            ),
                        ]);
                        if v.samples > 0 {
                            o.set("method", Json::Str("sampling".into()));
                            o.set("samples", Json::Int(v.samples as i64));
                            o.set(
                                "failedSamples",
                                Json::Arr(v.failed_samples.iter().map(|x| Json::Num(*x)).collect()),
                            );
                        }
                        if let Some(w) = v.worst {
                            o.set("worst", Json::Num(w));
                        }
                        o
                    })
                    .collect(),
            ),
        ),
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

/// **Where everything the source names has landed** (issue #48, item 3).
///
/// A report that says how many freedoms a drawing has and which statements fight, and never
/// where anything *is*, leaves the one question a reader without a picture asks most — and
/// answers it, if at all, by writing a claim and reading whether it is refuted.  The numbers are
/// already here: this is a **serialisation**, not a reading.  `EntKind::scalar_names` spells an
/// entity's scalars in `entity_params` order and `Sketch::entity_params` gives that same list as
/// parameters, so the two zip and nothing here learns what a circle is made of; a kind that
/// spells no scalars (a spline, a curve) contributes none, its control points being named points
/// of their own.
///
/// Keyed by **every** name the source calls the thing (`SourceMap::names`) — a declaration, under
/// the prefixes of the instance and the block copy it was elaborated in, which is the name a
/// diagnostic already uses and a reader already writes.  A formal is not a second key: aliasing
/// makes one entity of two names, so the entity answers under the name it was *declared* with,
/// wherever the caller wrote it.  A plane's `.angle` is the one number here the sketch does not
/// hold: it is derived from the rotor beside it exactly as `Tape::compile` derives it, because
/// `f.angle` is what a reader asks for and `f.c` is not.
///
/// Sorted and deduplicated by name, so a report is stable and a reader can find a name in it.
pub fn positions(sk: &Sketch, map: &crate::program::SourceMap) -> Vec<(String, f64)> {
    let mut out: std::collections::BTreeMap<String, f64> = Default::default();
    for (&e, names) in &map.names {
        let params = sk.entity_params(e);
        for n in names {
            let Some(scalars) = e.kind.scalar_names(n) else { continue };
            if scalars.len() != params.len() {
                continue;
            }
            for (name, &p) in scalars.iter().zip(params.iter()) {
                out.insert(name.clone(), sk.params[p as usize].value);
            }
            if e.kind == crate::model::EntKind::Plane {
                let v = |i: usize| sk.params[params[i] as usize].value;
                out.insert(format!("{n}.angle"), v(5).atan2(v(4)).to_degrees());
            }
        }
    }
    // **and what a solid came to.**  The report is the reader's only picture of an object no
    // view of the sheet shows whole, so it carries what a person would measure off it: how much
    // material there is, the box it stands in, and where each of its faces is and how much of
    // it survived — a bore that ate a cap leaves a name the document still writes and no area
    // behind it, which is a fact and not an error.
    //
    // At `REPORT_UNIT`, never at the screen's: a volume is a property of the document, and one
    // that changed with the zoom would be a number nobody could quote.
    for (&e, names) in &map.names {
        if e.kind != crate::model::EntKind::Solid {
            continue;
        }
        if crate::solid::validate(sk, e.i()).is_err() { continue; }
        let pieces = sk.solid_boundary(e.i(), crate::solid::REPORT_UNIT);
        let b = crate::mesh::bounds(&pieces);
        // Boundary pieces retain the primitive's absolute name. Recover the route through
        // this body's operands instead of dropping the first segment of every primitive.
        let paths = crate::solid::operand_paths(sk, e.i());
        for n in names {
            out.insert(format!("{n}.volume"), crate::mesh::volume(&pieces));
            out.insert(format!("{n}.area"), crate::mesh::area(&pieces));
            if !b.is_empty() {
                for (k, axis) in ["x", "y", "z"].iter().enumerate() {
                    out.insert(format!("{n}.bounds.{axis}0"), b.lo[k]);
                    out.insert(format!("{n}.bounds.{axis}1"), b.hi[k]);
                }
            }
            let mut faces: std::collections::BTreeMap<String, f64> = Default::default();
            for p in &pieces {
                if let Some((primitive, relative)) = paths.iter()
                    .filter(|(primitive, _)| p.path.starts_with(&format!("{primitive}.")))
                    .max_by_key(|(primitive, _)| primitive.len())
                {
                    let face = &p.path[primitive.len() + 1..];
                    let tail = if relative.is_empty() { face.to_string() } else { format!("{relative}.{face}") };
                    *faces.entry(tail).or_default() += p.area();
                }
            }
            for (tail, a) in faces {
                out.insert(format!("{n}.{tail}.area"), a);
            }
        }
    }
    out.into_iter().collect()
}

pub fn positions_json(sk: &Sketch, map: &crate::program::SourceMap) -> Json {
    Json::Obj(positions(sk, map).into_iter().map(|(n, v)| (n, Json::Num(v))).collect())
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
        Arg::Expr(e) => Json::Num(e.value),
        Arg::Seed { value, .. } => Json::Num(*value),
        // only a sketch can say what an owned unknown currently holds; `constraint_json` does
        Arg::Param(_) => Json::Null,
    }
}

/// One constraint as the bindings see it: identity, type, spec-ordered arguments and flags.
/// An argument written as an expression is its number here, like any other dimension, with the
/// text beside it under `exprs` (attribute → text) — a proxy's `c.d` stays a number and the
/// formula is there for whoever asks.
pub fn constraint_json(sk: &Sketch, c: &Constraint) -> Json {
    let args: Vec<Json> = c
        .args
        .iter()
        .map(|a| match a {
            // a hidden unknown reads as the number it holds, like any other argument: a proxy's
            // `c.t` is the parameter's current value, and the solver moves it between edits
            Arg::Param(_) => Json::Num(a.value(sk)),
            a => arg_json_value(a),
        })
        .collect();
    let exprs: Vec<(String, Json)> = c
        .spec()
        .iter()
        .zip(&c.args)
        .filter_map(|((n, _), a)| match a {
            Arg::Expr(e) => Some((n.to_string(), Json::Str(e.text.clone()))),
            _ => None,
        })
        .collect();
    // Identity and arguments only.  This is the record both bindings rebuild their whole
    // constraint list from after every edit; a `describe` string and an `error` that evaluates
    // the kernel are work per constraint per edit that nothing above reads — `gcs_describe` and
    // `gcs_constraint_error` are there for the one constraint someone is actually looking at.
    let mut v = object([
        ("id", (c.id as i64).into()),
        ("type", c.type_name().into()),
        ("args", Json::Arr(args)),
        ("soft", c.soft.into()),
        ("intrinsic", c.intrinsic.into()),
        ("claim", c.claim.into()),
    ]);
    if !exprs.is_empty() {
        v.set("exprs", Json::Obj(exprs));
    }
    v
}

/// The document's expressions, in evaluation order — see `expr::evaluate`.
pub fn exprs_json(sk: &mut Sketch) -> Json {
    Json::Arr(
        crate::expr::evaluate(sk)
            .into_iter()
            .map(|it| {
                object([
                    ("id", (it.id as i64).into()),
                    ("attr", it.attr.into()),
                    ("text", it.text.as_str().into()),
                    ("name", it.name.map(Json::Str).unwrap_or(Json::Null)),
                    ("value", it.value.into()),
                    ("deps", Json::Arr(it.deps.iter().map(|d| Json::Str(d.clone())).collect())),
                    ("free", Json::Arr(it.free.iter().map(|d| Json::Str(d.clone())).collect())),
                    ("error", it.error.map(|e| Json::Str(e.message)).unwrap_or(Json::Null)),
                ])
            })
            .collect(),
    )
}

fn pt(p: (f64, f64)) -> Json {
    floats(&[p.0, p.1])
}

fn segs(v: &[Seg]) -> Json {
    Json::Arr(v.iter().map(|s| Json::Arr(vec![pt(s.0), pt(s.1)])).collect())
}

/// One dimension callout, ready to paint.  Everything is in world coordinates; the front end
/// maps them to the screen with the same transform it draws the geometry through.  What a click
/// on it can land on is deliberately not here: `callout::pick` answers that question in the
/// core, which keeps this payload to what is actually drawn — it is rebuilt every frame.
fn callout_json(k: &Callout) -> Json {
    object([
        ("id", (k.id as i64).into()),
        ("text", k.text.as_str().into()),
        ("anchor", pt(k.anchor)),
        ("angle", Json::Num(k.angle)),
        ("label", Json::Arr(k.label.iter().map(|&p| pt(p)).collect())),
        ("solid", segs(&k.solid)),
        ("thin", segs(&k.thin)),
        (
            "arcs",
            Json::Arr(
                k.arcs
                    .iter()
                    .map(|a| {
                        object([
                            ("c", pt(a.c)),
                            ("r", Json::Num(a.r)),
                            ("a0", Json::Num(a.a0)),
                            ("a1", Json::Num(a.a1)),
                        ])
                    })
                    .collect(),
            ),
        ),
        (
            "arrows",
            Json::Arr(
                k.arrows
                    .iter()
                    .map(|a| object([("at", pt(a.at)), ("dir", pt(a.dir))]))
                    .collect(),
            ),
        ),
    ])
}

/// Every dimensioned constraint as a drafting figure.  `unit` is the world length of one screen
/// pixel, which is what makes the stand-offs, arrowheads and characters come out the same size at
/// any zoom; `font`, `arrow` and `barb` come back so the front end draws the text and the heads
/// at exactly the size and shape the layout reserved for them.
pub fn callouts_json(sk: &Sketch, unit: f64) -> Json {
    let items: Vec<Json> = callout::layout(sk, unit).iter().map(callout_json).collect();
    object([
        ("font", Json::Num(callout::FONT_PX)),
        ("arrow", Json::Num(callout::ARROW_PX)),
        ("barb", Json::Num(callout::BARB)),
        ("items", Json::Arr(items)),
    ])
}

/// The 3D overview, projected to 2D world coordinates (`overview.rs`): the glass box the views
/// were unfolded from, and the object they are of.  `unit` sizes what is screen-constant, `az`
/// and `el` are the orbit in radians.
pub fn overview_json(sk: &Sketch, unit: f64, az: f64, el: f64, shaded: bool) -> Json {
    let s = crate::overview::scene_with(sk, unit, az, el, shaded);
    let items: Vec<Json> = s
        .items
        .iter()
        .map(|it| {
            let pts: Vec<Json> = it.pts.iter().map(|&p| pt(p)).collect();
            let mut o = object([
                ("part", Json::Str(it.what.as_str().to_string())),
                ("pts", Json::Arr(pts)),
            ]);
            // what it is drawn from, so the front end resolves style and selection through the
            // entity exactly as it does on the sheet
            if let Some(e) = it.of {
                o.set("kind", Json::Str(e.kind.as_str().to_string()));
                o.set("index", Json::Int(e.idx as i64));
            }
            // the view it belongs to, so a front end can go to that plane without working out
            // a second time which one an entity is drawn in
            if let Some(p) = it.in_plane {
                o.set("plane", Json::Int(p.idx as i64));
            }
            // how squarely a surface faces the light — a number, since which tone that is is the
            // front end's chrome.  Written only for a surface, the way a class is written only
            // when it is set
            if let Some(sh) = it.shade {
                o.set("shade", Json::Num(sh));
            }
            o
        })
        .collect();
    object([
        ("items", Json::Arr(items)),
        ("bounds", floats(&[s.bounds.0, s.bounds.1, s.bounds.2, s.bounds.3])),
    ])
}

/// **The pictures the document asked for** (§6.11), laid out and inked.
///
/// `overview_json`'s shape, and `callouts_json`'s bargain: heterogeneous records rather than a
/// fixed-width block, one call for the whole sketch, and the *ink already resolved* — so the
/// canvas strokes what it is handed and a document's own `style .hidden` rule reaches a derived
/// view with nothing added on the far side of the ABI.
pub fn derived_json(sk: &Sketch, unit: f64) -> Json {
    let items: Vec<Json> = crate::hidden::layout(sk, unit)
        .iter()
        .map(|d| {
            let mut o = object([
                ("pts", Json::Arr(d.pts.iter().map(|&p| pt(p)).collect())),
                ("of", Json::Int(d.of as i64)),
                ("solid", Json::Str(d.solid.clone())),
                ("path", Json::Str(d.path.clone())),
                ("stroke", style_json(&d.style)),
            ]);
            // written only when set, the way a class is: a reader that does not care about
            // hidden lines never has to know the word
            if d.hidden {
                o.set("hidden", Json::Bool(true));
            }
            if d.silhouette {
                o.set("silhouette", Json::Bool(true));
            }
            o
        })
        .collect();
    Json::Arr(items)
}

fn style_json(s: &crate::style::Style) -> Json {
    let mut o = object([]);
    if let Some(c) = &s.color {
        o.set("color", Json::Str(c.clone()));
    }
    if let Some(w) = s.width {
        o.set("width", Json::Num(w));
    }
    if let Some(d) = s.dash.as_ref().filter(|d| !d.is_empty()) {
        o.set("dash", floats(d));
    }
    o
}

/// **The scene in space**, for a front end with a camera of its own — `overview_json`'s shape,
/// with three numbers a point instead of two and no orbit applied, because the orbit is the
/// renderer's now.
pub fn overview3d_json(sk: &Sketch, unit: f64) -> Json {
    let items: Vec<Json> = crate::overview::scene3d(sk, unit)
        .iter()
        .map(|it| {
            let pts: Vec<Json> = it
                .pts
                .iter()
                .map(|p| Json::Arr(p.iter().map(|&v| Json::Num(v)).collect()))
                .collect();
            let mut o = object([
                ("part", Json::Str(it.what.as_str().to_string())),
                ("pts", Json::Arr(pts)),
            ]);
            if let Some(e) = it.of {
                o.set("kind", Json::Str(e.kind.as_str().to_string()));
                o.set("index", Json::Int(e.idx as i64));
            }
            if let Some(p) = it.in_plane {
                o.set("plane", Json::Int(p.idx as i64));
            }
            o
        })
        .collect();
    Json::Arr(items)
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
            // the words a slot will take, where it takes words: published so a front end offers
            // what the core accepts rather than keeping its own list of them (issue #48, item 4)
            let words: Vec<Json> = (0..k.spec().len())
                .map(|i| match k.words(i) {
                    Some(ws) => Json::Arr(ws.iter().map(|w| Json::Str(w.to_string())).collect()),
                    None => Json::Null,
                })
                .collect();
            // **the surface word and the wire name are different things** (spec §9.1): `name`
            // is the snake_case identifier the binding keys on and the JSON export writes, and
            // is unchanged; the operator is new information beside it, published once so that
            // no binding learns the grammar.
            let (word, fixity) = match k.operator() {
                Some((w, f)) => (Json::Str(w.to_string()), Json::Str(f.as_str().to_string())),
                None => (Json::Null, Json::Null),
            };
            object([
                ("name", k.name().into()),
                ("operator", word),
                ("fixity", fixity),
                // how many of the leading spec slots are the operator's *operands*; the rest is
                // what goes in its parentheses
                ("operands", Json::Int(k.spec().iter().take_while(|(_, s)| s.is_entity()).count().min(2) as i64)),
                ("spec", Json::Arr(spec)),
                ("defaults", Json::Arr(defaults)),
                ("words", Json::Arr(words)),
                ("soft", k.soft_by_default().into()),
                ("commutative", k.commutative().into()),
                // -1 for a curve contact: its kernel is the curve *definition's*, so there is no
                // one id to publish and a binding has nothing to do with the number anyway
                (
                    "kernel",
                    match k {
                        k if k.family_kernel().is_some() => Json::Int(-1),
                        _ => (k.kernel() as i64).into(),
                    },
                ),
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
    object([
        ("types", Json::Arr(types)),
        ("kernels", Json::Arr(kernels)),
        // what a front end needs to know about the curves without knowing the degree: how many
        // control points make one, so its tool and its messages cannot drift from `spline_with`
        ("curve", object([
            ("degree", (crate::curve::DEGREE as i64).into()),
            ("minCtrl", (crate::curve::MIN_CTRL as i64).into()),
        ])),
    ])
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
                // the kind the slot takes and an index the sketch has: a binding's record is
                // untrusted the way a document is, and every reader past this one indexes by
                // the spec's kind — a mismatch or an overrun is a panic there, an abort in wasm
                if !crate::constraints::kind_matches(*k, ek) {
                    return Err(format!("a {} slot does not take a {}", k.as_str(), ek.as_str()));
                }
                let i = arr.get(1).map(|x| x.as_i64()).unwrap_or(0);
                if i < 0 || i as usize >= sk.count(ek) {
                    return Err(format!("{} index {i} out of range", ek.as_str()));
                }
                Arg::Ent(EntRef::new(ek, i as usize))
            }
            // a hidden unknown arrives as its number, or `{"value", "pinned"}` when whoever
            // computed it means it to stay — the same form the document uses
            (crate::constraints::SpecKind::Param, _) => Arg::Seed {
                value: a.get("value").map(|x| x.as_f64()).unwrap_or_else(|| a.as_f64()),
                pinned: a.get("pinned").map(|x| x.as_bool()).unwrap_or(false),
            },
            (crate::constraints::SpecKind::Int, _) => Arg::Int(a.as_i64()),
            (crate::constraints::SpecKind::Bool, _) => Arg::Bool(a.as_bool()),
            (crate::constraints::SpecKind::Str, _) => Arg::Str(a.as_str().to_string()),
            // a dimension may arrive as text, under `set_dimension`'s rule: a bare number is a
            // constant in the units a person writes (degrees for an angle), anything else an
            // expression — `"w = 1"` — evaluated once it is added
            (k, Json::Str(text)) if k.is_dimension() => match crate::expr::literal(text) {
                Some(v) => Arg::Num(crate::expr::to_arg_units(*k, v)),
                None => {
                    crate::expr::parse(text)?;
                    Arg::Expr(crate::expr::Expr::new(text.trim(), 0.0))
                }
            },
            _ => Arg::Num(a.as_f64()),
        });
    }
    // a hidden unknown nobody supplied starts where the geometry puts it
    crate::io::seed_omitted(sk, kind, &mut args, |i| crate::io::omitted(raw.get(i)))?;
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
    // as in `io::from_json`: a claim that would own an unknown is not one this kind can carry
    c.claim = v.get("claim").map(|x| x.as_bool()).unwrap_or(false) && kind.claimable();
    c.intrinsic = v.get("intrinsic").map(|x| x.as_bool()).unwrap_or(false);
    Ok(c)
}
