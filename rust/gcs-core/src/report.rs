//! JSON views of the analysis results.
//!
//! Diagnosis, witness reports, plans and constraint graphs are rich, ragged structures.  Encoding
//! them here — once — is what lets the TypeScript package stay a thin binding: it parses one
//! document instead of reimplementing a dozen accessors, and a second binding would see exactly
//! the same field names.  Hot-path numbers (residuals, Jacobians, drag frames) never go through
//! here.

#[allow(unused_imports)]
use crate::fmath::Det;
use crate::callout::{self, Callout, Seg};
use crate::cgraph::{ConstraintGraph, El, ElKind};
use crate::constraints::{tangency_at, Arg, CKind, Constraint, SpecKind, Touch, ALL_KINDS};
use crate::decompose::{Plan, PlanResult};
use crate::diagnose::{summary as diag_summary, Diagnosis};
use crate::homotopy::Alternative;
use crate::io::describe;
use crate::json::{object, Json};
use crate::kernels::KERNELS;
use crate::model::{EntKind, EntRef, Sketch};
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
        ("settled", r.settled().into()),
        ("outcome", r.outcome().into()),
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
        ("byParts", d.by_parts.into()),
        ("geometricDependency", (d.geometric_dependency as i64).into()),
        ("shaky", (d.shaky as i64).into()),
        ("over", ids(&d.over)),
        ("implied", ids(&d.implied)),
        ("expected", ids(&d.expected)),
        ("claimsTheorem", ids(&d.claims_theorem)),
        ("claimsViolated", ids(&d.claims_violated)),
        ("claimsConsuming", ids(&d.claims_consuming)),
        ("boundsViolated", ids(&d.bounds_violated)),
        ("boundsActive", ids(&d.bounds_active)),
        // the orientations the document states that the solution breaks, by record key (#157)
        (
            "orientationsViolated",
            Json::Arr(d.orientations_violated.iter().map(|k| Json::Str(k.clone())).collect()),
        ),
        ("collapsed", ids(&d.collapsed)),
        // each energy's verdict, by its terms' constraints (#121): `[id, "minimum", asked]`, the
        // last whether it is what the statement asked for — a minimum of a `minimizes`
        (
            "extrema",
            Json::Arr(
                d.extrema
                    .iter()
                    .map(|&(id, v)| {
                        let asked = crate::variational::asked(sk, id) == v;
                        Json::Arr(vec![Json::Int(id as i64), Json::Str(v.name().into()), asked.into()])
                    })
                    .collect(),
            ),
        ),
        // the claims about solids, each with what was measured and how far the faceting could be
        // wrong — there is no `consuming` here, since a solid claim compiles no row to consume
        // rank with (§9.8)
        (
            "solidClaims",
            Json::Arr(
                d.solid_claims
                    .iter()
                    .map(solid_claim_json)
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
        // each requirement out of reach of what drives it, and why (#510 P7): `[id, words]`
        (
            "outOfReach",
            Json::Arr(d.out_of_reach.iter().map(|(id, text)| {
                Json::Arr(vec![Json::Int(*id as i64), Json::Str(text.clone())])
            }).collect()),
        ),
        ("warnings", Json::Arr(d.warnings.iter().map(|s| Json::Str(s.clone())).collect())),
        ("witness", d.witness.as_ref().map(|w| witness_json(sk, w)).unwrap_or(Json::Null)),
        ("dof", d.dof.into()),
        ("structuralDof", d.structural_dof.into()),
        ("nRedundant", d.n_redundant.into()),
        ("structuralNRedundant", d.structural_n_redundant.into()),
        ("status", d.status.as_str().into()),
        ("summary", diag_summary(d).into()),
        // what each declared parameter drives (#391 S4); how far the drawing is followed along
        // it is a march, asked of `parameter_text` and not of every diagnosis
        ("parameters", Json::Arr(d.parameters.iter().map(|p| object([
            ("name", p.name.as_str().into()),
            ("readers", ids(&p.readers)),
            ("dof", p.dof.map(|n| n as i64).into()),
        ])).collect())),
    ])
}

/// **An orientation the document states, as a statement** (§9.6, #157): the record `key` of
/// `Sketch::orientations` written back as the call it is, `ccw(a, b, c)` or `cw(a, b, c)`, its
/// points in the record's order and named by `name` where it names them.  `None` for a key that
/// is no stated orientation of three points.
pub fn orientation_text(
    sk: &Sketch,
    key: &str,
    name: &dyn Fn(EntRef) -> Option<String>,
) -> Option<String> {
    let t = crate::solve::orientation_triangle(sk, key)?;
    let word = if *sk.orientations.get(key)? >= 0 { "ccw" } else { "cw" };
    let names: Vec<String> = t.iter()
        .map(|&p| name(EntRef::point(p)).unwrap_or_else(|| crate::model::entity_name(EntRef::point(p))))
        .collect();
    Some(format!("{word}({})", names.join(", ")))
}

/// **How a drawing runs over its declared parameter `name`** (#391 S4), in a reader's words: how
/// far it is followed (`follow::followed`) and, where the march stopped short, the singular
/// instant it stopped at — or why it is not followed at all.
pub fn parameter_text(sk: &Sketch, name: &str) -> String {
    let followed = crate::follow::followed(sk, name);
    match followed.as_ref() {
        Err(why) => format!("{name}: not followed — {why}"),
        Ok(f) => {
            let at = |v: f64| crate::follow::reading(&f.over, v, sk.units);
            let reach = format!("{name}: followed from {} to {}", at(f.over.from), at(f.reached()));
            match &f.singular {
                Some(s) => format!("{reach}, {}", s.text(&f.over, sk.units)),
                None => reach,
            }
        }
    }
}

/// **The materials a drawing states** (#510 P3): each as the document JSON holds it
/// (`io::material_json`, its properties in SI), its `text` in the core's words, and `solids`, the
/// indices of the solids made of it.
pub fn materials_json(sk: &Sketch) -> Json {
    Json::Arr(sk.materials.values().map(|m| {
        let mut o = crate::io::material_json(m);
        o.set("text", Json::Str(material_text(sk, &m.name)));
        let of = made_of(sk, &m.name).map(|i| Json::Int(i as i64));
        o.set("solids", Json::Arr(of.collect()));
        o
    }).collect())
}

/// The solids made of material `name`, by index.
fn made_of<'a>(sk: &'a Sketch, name: &'a str) -> impl Iterator<Item = usize> + 'a {
    (0..sk.solids.len()).filter(move |&i| sk.solids[i].material.as_deref() == Some(name))
}

/// A material as a reader reads it, and what is made of it — its solids, then its beams (#510
/// S0): `steel: E 200GPa, nu 0.3, of arm`.
pub fn material_text(sk: &Sketch, name: &str) -> String {
    let Some(m) = sk.materials.get(name) else { return format!("{name}: no material") };
    let beams = sk.beams.iter().filter(|b| b.material.as_deref() == Some(name));
    let of: Vec<&str> = made_of(sk, name).map(|i| sk.solids[i].name.as_str())
        .chain(beams.map(|b| b.name.as_str())).collect();
    if of.is_empty() {
        format!("{}, of no solid", m.text())
    } else {
        format!("{}, of {}", m.text(), of.join(", "))
    }
}

/// **What each load scenario says** (#510 P7, S0) at the solved drawing `sk`: per scenario, its
/// name, its words (`diagnose::analysis::scenario_read`) — the motions its supports leave free,
/// each re-entrant edge where a `safety` it states is unbounded, what its frames cannot be read
/// under — and each requirement a frame answers (`scenario_answers`); none where all is well.
pub fn scenarios_json(sk: &Sketch, scenarios: &[crate::program::Stated]) -> Json {
    Json::Arr(scenarios.iter().map(|s| {
        let (words, read) = crate::diagnose::analysis::scenario_read(sk, &s.scenario);
        scenario_json(sk, s, words, &read)
    }).collect())
}

/// One scenario's record in `scenarios_json`: its name, its words and its answers, each with
/// `index`, the requirement's place among the scenario's.
fn scenario_json(sk: &Sketch, s: &crate::program::Stated, words: Vec<String>,
                 read: &crate::beams::Read) -> Json {
    let answers = scenario_answers(sk, s, read).into_iter().map(|a| object([
        ("index", Json::Int(a.index as i64)),
        ("requirement", a.head.as_str().into()),
        ("grade", a.grade.into()),
        ("value", a.value.map_or(Json::Null, Json::from)),
        ("error", a.error.map_or(Json::Null, Json::from)),
        ("holds", a.holds.map_or(Json::Null, Json::from)),
        ("text", a.text.as_str().into()),
    ]));
    object([
        ("name", s.scenario.name.as_str().into()),
        ("words", Json::Arr(words.into_iter().map(Json::Str).collect())),
        ("answers", Json::Arr(answers.collect())),
    ])
}

/// **What every load scenario says, and scenario `drawn`'s picture** (#510): `scenarios_json`,
/// and beside it the picture of the one asked to be drawn (`deformed::Picture`, `picture_json`)
/// — its answers and its picture one reading, each body analysed once for both — or `null`
/// where `drawn` is none of them; and what the drawing's masses and volumes come to
/// (`measures_json`), read with them since a solid's may be meshed for seconds too.
pub fn scenarios_pictured_json(sk: &Sketch, scenarios: &[crate::program::Stated],
                               measures: &[crate::program::MeasureStated], drawn: Option<usize>)
    -> Json
{
    let mut picture = Json::Null;
    let all = scenarios.iter().enumerate().map(|(k, s)| {
        let (words, read) = if Some(k) == drawn {
            let (words, read, p) = crate::diagnose::analysis::scenario_pictured(sk, &s.scenario);
            picture = picture_json(sk, &p);
            (words, read)
        } else {
            crate::diagnose::analysis::scenario_read(sk, &s.scenario)
        };
        scenario_json(sk, s, words, &read)
    }).collect();
    object([("scenarios", Json::Arr(all)), ("picture", picture),
            ("measures", measures_json(sk, measures))])
}

/// **The sizes a page reads its scenarios at, coarsest first** (#510): each element four times
/// the readers' own size, then twice, then their own.  Each level's error is estimated against
/// the level before it, so the last is `scenarios_pictured_json`'s reading — its own size against
/// twice as coarse — and the first, a fraction of the cost, comes in seconds.
pub const LEVELS: [f64; 3] = [4.0, 2.0, 1.0];

/// **Every scenario read progressively** (#510, `LEVELS`): a level a `next`, coarsest first,
/// each holding what it read for the next to estimate its error against.  What a page's worker
/// steps, posting each level as it comes and stopping when the drawing changes.  The drawing's
/// masses and volumes answer no level (`measures_json`): read with the first, they stand beside
/// every one; and a drawing with no scenario is read once, its last.
#[derive(Clone, Debug, Default)]
pub struct Levels {
    at: usize,
    last: Vec<Option<crate::beams::Read>>,
    measures: Option<Json>,
}

impl Levels {
    pub fn new() -> Levels {
        Levels::default()
    }

    /// Whether every level has been read.
    pub fn done(&self) -> bool {
        self.at >= LEVELS.len()
    }

    /// **The next level**: `scenarios_pictured_json`'s `{ "scenarios", "picture", "measures" }`
    /// read at it — its answers' errors against the level before, none at the first — with
    /// `"level"`, how many times the readers' size its elements are, `"final"`, whether it is the
    /// last, and `"levelText"`, what it is in the core's words (empty at the last).  `None` once
    /// done.
    pub fn next(&mut self, sk: &Sketch, scenarios: &[crate::program::Stated],
                measures: &[crate::program::MeasureStated], drawn: Option<usize>)
        -> Option<Json>
    {
        if scenarios.is_empty() {
            self.at = self.at.max(LEVELS.len() - 1);
        }
        let coarser = *LEVELS.get(self.at)?;
        let measures = self.measures.get_or_insert_with(|| measures_json(sk, measures)).clone();
        self.last.resize(scenarios.len(), None);
        let mut picture = Json::Null;
        let all = scenarios.iter().enumerate().map(|(k, s)| {
            let (words, read, p) = crate::diagnose::analysis::scenario_read_at(
                sk, &s.scenario, coarser, self.last[k].as_ref(), Some(k) == drawn);
            if Some(k) == drawn {
                picture = picture_json(sk, &p);
            }
            let out = scenario_json(sk, s, words, &read);
            self.last[k] = Some(read);
            out
        }).collect();
        self.at += 1;
        let last = self.done();
        Some(object([
            ("scenarios", Json::Arr(all)),
            ("picture", picture),
            ("measures", measures),
            ("level", coarser.into()),
            ("final", last.into()),
            ("levelText", level_text(self.at - 1).into()),
        ]))
    }
}

/// What reading `LEVELS[k]` is, as a page says it while the next is read.
fn level_text(k: usize) -> String {
    match k {
        0 => "a first reading, its elements four times their size: finer ones follow".into(),
        1 => "a second reading, its elements twice their size, its errors against the first: \
              the reading at their own size follows".into(),
        _ => String::new(),
    }
}

/// **A scenario's picture** (`deformed::Picture`) as a host draws it: per body its kind, name and
/// solid, and flat arrays — its vertices at rest and their moves (three numbers each, to seven
/// figures), each vertex's move as a fraction of the largest, its triangles and its mesh's edges
/// (vertex indices); then why any body it falls on is not drawn, the largest move and where it
/// stands, as a reader reads it, the extent drawn, and the factor to draw at (`×200`).
pub fn picture_json(sk: &Sketch, p: &crate::deformed::Picture) -> Json {
    let fig = |v: f64| Json::Num(format!("{v:.6e}").parse().unwrap_or(v));
    let flat = |vs: &[[f64; 3]]| Json::Arr(vs.iter().flatten().map(|&v| fig(v)).collect());
    let ints = |vs: Vec<u32>| Json::Arr(vs.into_iter().map(|v| Json::Int(v as i64)).collect());
    let bodies = p.bodies.iter().enumerate().map(|(k, b)| object([
        ("kind", b.kind.into()),
        ("name", b.name.as_str().into()),
        ("solid", b.solid.map_or(Json::Null, |s| Json::Int(s as i64))),
        ("rest", flat(&b.rest)),
        ("moved", flat(&b.moved)),
        ("size", Json::Arr(p.sizes(k).into_iter().map(|v| fig(v)).collect())),
        ("triangles", ints(b.triangles.iter().flatten().copied().collect())),
        ("edges", ints(b.edges.iter().flatten().copied().collect())),
    ]));
    object([
        ("bodies", Json::Arr(bodies.collect())),
        ("words", Json::Arr(p.words.iter().map(|w| Json::Str(w.clone())).collect())),
        ("largest", p.largest.into()),
        ("largestText", p.largest_text(sk).into()),
        ("largestAt", Json::Arr(p.largest_at.map(Json::from).to_vec())),
        ("extent", p.extent.into()),
        ("scale", p.scale.into()),
    ])
}

/// **The document's load scenarios by name** (#510), in the order written: each `name` and the
/// scenario it `extends` (null for none) — what a chooser lists, read with no analysis.
pub fn scenario_names_json(scenarios: &[crate::program::Stated]) -> Json {
    Json::Arr(scenarios.iter().map(|s| object([
        ("name", s.scenario.name.as_str().into()),
        ("extends", s.scenario.extends.and_then(|b| scenarios.get(b))
            .map_or(Json::Null, |b| b.scenario.name.as_str().into())),
    ])).collect())
}

/// **Where scenario `k` of the document falls on the drawing** (`overview::scenario::marks`):
/// each mark's kind and index, its anchors, its directions and normals there, the triangles it
/// tints (nine numbers each), its size among its kind, its words and where they are drawn,
/// whether it is a requirement read at a place (`probe`), and `span`, the bytes of the line it
/// is written on in the document (an extended scenario's base's lines where its base wrote them)
/// — `null` for a line a module wrote.
pub fn scenario_marks_json(e: &crate::program::Elaborated, sk: &Sketch, k: usize) -> Json {
    use crate::overview::scenario::Kind;
    let Some(s) = e.scenarios.get(k) else { return Json::Arr(Vec::new()) };
    let span = |line: Option<&crate::program::Line>| -> Json {
        let at = line.and_then(|l| {
            let st = e.program.document_stmt(l.stmt)?;
            let crate::syntax::StmtKind::Scenario(sc) = &st.kind else { return None };
            sc.items.get(l.item).map(|i| i.span)
        });
        at.map_or(Json::Null, |sp| Json::Arr(vec![Json::Int(sp.lo as i64),
                                                  Json::Int(sp.hi as i64)]))
    };
    let v3 = |v: &[f64; 3]| Json::Arr(v.map(Json::from).to_vec());
    let vs = |vs: &[[f64; 3]]| Json::Arr(vs.iter().map(v3).collect());
    Json::Arr(crate::overview::scenario::marks(sk, s).into_iter().map(|m| {
        let line = match m.kind {
            Kind::Fixed | Kind::Pinned => s.supports.get(m.index),
            Kind::Load | Kind::Pressure | Kind::Accelerate => s.loads.get(m.index),
            Kind::Requirement => s.requirements.get(m.index),
            Kind::Plane => s.readings.get(m.index),
        };
        object([
            ("kind", m.kind.word().into()),
            ("index", Json::Int(m.index as i64)),
            ("at", vs(&m.at)),
            ("dir", vs(&m.dir)),
            ("normal", vs(&m.normal)),
            ("tint", Json::Arr(m.tint.iter().flatten().flatten().map(|&x| Json::from(x))
                .collect())),
            ("size", m.size.into()),
            ("text", m.text.as_str().into()),
            ("label", v3(&m.label)),
            ("probe", m.probe.into()),
            ("span", span(line)),
        ])
    }).collect())
}

/// **A requirement a frame answers** (#510 S0): what it is of as written, its grade, its value
/// in the document's units (`None` where there is none), the error estimated beside it (#510
/// S3, `beams::Read::errors`), whether it keeps what it states, and the sentence a report says
/// — `` scenario `lift`: deflection(tip, along: down) is 0.4mm ± 0.0012mm: claimed <= 0.5mm,
/// kept ``.
#[derive(Clone, Debug, PartialEq)]
pub struct ScenarioAnswer {
    /// Its requirement's place among the scenario's.
    pub index: usize,
    pub head: String,
    pub grade: &'static str,
    pub value: Option<f64>,
    pub error: Option<f64>,
    pub holds: Option<bool>,
    pub text: String,
}

/// Every requirement of scenario `s` its frames or its regions answer (`beams::read`,
/// `prisms::read`, in `read`), in the order written.  A claim is kept or refuted; a driving
/// deflection the drawing is solved for (#510 S1, `Stated::driven`) is said to be held or not
/// reached, its value its own row's — a region's on the mesh the row was compiled on and moved
/// with the drawing (#510 S2), which a fresh mesh of the solved drawing reads to its
/// discretisation, not to rounding; any other driving row or bound is said to hold or not, and
/// that nothing drives it yet (S3–S4).  A driven requirement is said whenever its row has a value,
/// whatever its reading answers.  What a body is made of is no scenario's: the drawing's masses and
/// volumes are said by `measure_answers`.
pub fn scenario_answers(sk: &Sketch, s: &crate::program::Stated, read: &crate::beams::Read)
    -> Vec<ScenarioAnswer>
{
    use crate::model::Grade;
    let mut out = Vec::new();
    for (k, r) in s.scenario.requirements.iter().enumerate() {
        let head = s.heads[k].clone();
        let grade = r.grade().word();
        let stated = format!("{} {}", r.compare.text(),
                             crate::beams::value_text(sk, r.output, r.value));
        // a driven requirement's value is its row's at the solved drawing
        let row = s.driven.get(k).copied().flatten().and_then(|id| sk.constraint(id)).map(|c| {
            c.residual(sk, &c.local_values(sk))[0] + r.value
        });
        let got = match (&read.answers[k], row) {
            (_, Some(v)) if v.is_finite() => Ok(v),
            (Some(got), _) => got.clone(),
            (None, _) => continue,
        };
        let error = read.errors.get(k).copied().flatten();
        let (value, holds, said) = match &got {
            Ok(v) => {
                let keeps = crate::beams::keeps(r, *v);
                let driven = s.driven.get(k).is_some_and(Option::is_some);
                let verdict = match (r.grade(), keeps) {
                    (Grade::Claim, true) => format!("claimed {stated}, kept"),
                    (Grade::Claim, false) => format!("claimed {stated}, refuted"),
                    (Grade::Driving, true) if driven => format!("driven {stated}, held"),
                    (Grade::Driving, false) if driven => format!("driven {stated}, not reached"),
                    // a filled body's drive is the native core's (#567, `DRIVES_FILLED`)
                    (Grade::Driving, holds) if natively(sk, &s.scenario, r) => {
                        let holds = if holds { "holds" } else { "does not hold" };
                        format!("stated {stated}, which {holds}; `solventc` drives it, the page \
                                 does not: a filled body's drive solves its mesh at every step")
                    }
                    (_, true) => format!("stated {stated}, which holds; nothing drives it yet"),
                    (_, false) => format!("stated {stated}, which does not hold; nothing drives \
                                           it yet"),
                };
                let mut v_text = crate::beams::value_text(sk, r.output, *v);
                if let Some(e) = error {
                    v_text = format!("{v_text} {}", crate::beams::error_text(sk, r.output, e));
                }
                (Some(*v), Some(keeps), format!("{head} is {v_text}: {verdict}"))
            }
            Err(why) => (None, None, format!("{head} has no answer: {why}")),
        };
        let text = format!("scenario `{}`: {said}", s.scenario.name);
        let error = value.and(error);
        out.push(ScenarioAnswer { index: k, head, grade, value, error, holds, text });
    }
    out
}

/// Whether requirement `r` of scenario `sc` is a deflection of a filled body that the native core
/// drives and this one does not (#567, `bodies::DRIVES_FILLED`): the page's.
fn natively(sk: &Sketch, sc: &crate::model::Scenario, r: &crate::model::Requirement) -> bool {
    !crate::bodies::DRIVES_FILLED && r.output == crate::model::Output::Deflection
        && r.along.is_some() && crate::bodies::of_filled(sk, sc, &r.of)
}

/// **What a mass or a volume the drawing states comes to** (#510, `program::MeasureStated`),
/// read on the solved drawing (`measured::read`) — its value, the error beside it, whether it holds
/// and the sentence: `` mass(arm) is 10kg: driven == 10kg, held ``, `` mass(plank) is 0.85kg ±
/// 0.0001kg: claimed <= 1kg, kept ``.  A stated one's value is its row's (its residual plus what it
/// states), what the solve held; a claim's is the reading's.  No scenario's: what a body is made of
/// answers no load.
pub fn measure_answers(sk: &Sketch, measures: &[crate::program::MeasureStated])
    -> Vec<ScenarioAnswer>
{
    measures.iter().enumerate().filter_map(|(k, ms)| {
        let m = sk.measured.get(ms.index)?;
        let q = m.quantity;
        let stated =
            format!("{} {}", m.compare.text(), crate::measured::value_text(sk, q, m.value));
        let read = crate::measured::read(sk, m);
        let row = ms.driven.and_then(|id| sk.constraint(id))
            .map(|c| c.residual(sk, &c.local_values(sk))[0] + m.value)
            .filter(|v| v.is_finite());
        let got = match (row, read) {
            (Some(v), read) => Ok((v, read.ok().and_then(|r| r.1))),
            (None, read) => read,
        };
        let head = ms.head.clone();
        let grade = if m.claim { "claim" } else { "driving" };
        let (value, error, holds, text) = match got {
            Ok((v, error)) => {
                let keeps = crate::measured::keeps(m, v);
                let verdict = match (m.claim, keeps) {
                    (true, true) => format!("claimed {stated}, kept"),
                    (true, false) => format!("claimed {stated}, refuted"),
                    (false, true) => format!("driven {stated}, held"),
                    (false, false) => format!("driven {stated}, not reached"),
                };
                let mut said = crate::measured::value_text(sk, q, v);
                if let Some(e) = error {
                    said = format!("{said} {}", crate::measured::error_text(sk, q, e));
                }
                (Some(v), error, Some(keeps), format!("{head} is {said}: {verdict}"))
            }
            Err(why) => (None, None, None, format!("{head} has no answer: {why}")),
        };
        Some(ScenarioAnswer { index: k, head, grade, value, error, holds, text })
    }).collect()
}

/// **`measure_answers` as JSON**: per mass or volume stated, in the order written, its
/// `requirement` (what it is of), `grade` (`driving` or `claim`), `value`, `error`, `holds` and
/// `text`.
pub fn measures_json(sk: &Sketch, measures: &[crate::program::MeasureStated]) -> Json {
    Json::Arr(measure_answers(sk, measures).into_iter().map(|a| object([
        ("index", Json::Int(a.index as i64)),
        ("requirement", a.head.as_str().into()),
        ("grade", a.grade.into()),
        ("value", a.value.map_or(Json::Null, Json::from)),
        ("error", a.error.map_or(Json::Null, Json::from)),
        ("holds", a.holds.map_or(Json::Null, Json::from)),
        ("text", a.text.as_str().into()),
    ])).collect())
}

/// **A number the document names, as a reader is told it** (#596): `torque = 0.0864 N·m`, to
/// `READING_SIG` figures in the unit it reads in (`Dim::reading`).
pub fn value_text(units: crate::units::Units, name: &str, v: &crate::flatten::Valued) -> String {
    format!("{name} = {}", v.dim.reading(v.value, units, crate::expr::READING_SIG))
}

/// **A claim over values, judged** (#596): `claim torque >= 2Nm — refuted: torque = 0.0864 N·m`,
/// what it says, whether it is a theorem or refuted (a value is worked out before the solve, so
/// it is never consuming), and each value it reads.
pub fn value_claim_text(units: crate::units::Units, c: &crate::flatten::ValueClaimed) -> String {
    let verdict = if c.holds { "theorem" } else { "refuted" };
    let reads: Vec<String> = c.reads.iter().map(|(n, v)| value_text(units, n, v)).collect();
    if reads.is_empty() {
        format!("claim {} — {verdict}", c.text)
    } else {
        format!("claim {} — {verdict}: {}", c.text, reads.join(", "))
    }
}

/// **`value_text` as JSON** (#596): each name to its `value` in the unit it reads in, that
/// `unit` (empty for a plain number), its `dim` and its `text`.
pub fn values_json<'v>(
    units: crate::units::Units,
    values: impl IntoIterator<Item = (&'v String, &'v crate::flatten::Valued)>,
) -> Json {
    Json::Obj(values.into_iter().map(|(name, v)| {
        let (value, unit) = v.dim.read_in(v.value, units);
        (name.clone(), object([
            ("value", value.into()),
            ("unit", unit.as_str().into()),
            ("dim", v.dim.name().into()),
            ("text", value_text(units, name, v).into()),
        ]))
    }).collect())
}

/// **`value_claim_text` as JSON** (#596): per claim over values, in the order walked, its
/// `claim` as written, `holds`, the `values` it reads (`values_json`) and its `text`.
pub fn value_claims_json(units: crate::units::Units, claims: &[crate::flatten::ValueClaimed])
    -> Json
{
    Json::Arr(claims.iter().map(|c| object([
        ("claim", c.text.as_str().into()),
        ("holds", c.holds.into()),
        ("values", values_json(units, c.reads.iter().map(|(n, v)| (n, v)))),
        ("text", value_claim_text(units, c).into()),
    ])).collect())
}

/// **The parameters a drawing declares** (#398 L), for a timeline: each by name, what it is and
/// the interval it runs over, its ends in the units a person reads (degrees for an angle) and as
/// read.  No march: that is `parameter_json`'s, asked of one parameter.
pub fn parameters_json(sk: &Sketch) -> Json {
    Json::Arr(sk.parameters.values().map(|p| {
        let at = |v: f64| Json::from(crate::follow::reading(p, v, sk.units));
        object([
            ("name", p.name.as_str().into()),
            ("dim", p.dim.name().into()),
            ("from", p.from.into()),
            ("to", p.to.into()),
            ("fromText", at(p.from)),
            ("toText", at(p.to)),
        ])
    }).collect())
}

/// **How a drawing runs over its declared parameter `name`** (#398 L), for a timeline: followed
/// (`follow::followed`, remembered), from where and how far it reached and, where the march
/// stopped short, the singular instant and what it is — every reading in the core's words,
/// `parameter_text` the whole of it — or why it is not followed.
pub fn parameter_json(sk: &Sketch, name: &str) -> Json {
    let followed = crate::follow::followed(sk, name);
    let text = parameter_text(sk, name);
    match followed.as_ref() {
        Err(why) => object([
            ("name", name.into()),
            ("followed", false.into()),
            ("why", why.as_str().into()),
            ("text", text.into()),
        ]),
        Ok(f) => {
            let at = |v: f64| Json::from(crate::follow::reading(&f.over, v, sk.units));
            let singular = f.singular.map_or(Json::Null, |s| object([
                ("at", s.at.into()),
                ("kind", match s.kind {
                    crate::follow::Kind::Limit => "limit",
                    crate::follow::Kind::Branch => "branch",
                }.into()),
                ("text", s.text(&f.over, sk.units).into()),
            ]));
            object([
                ("name", name.into()),
                ("followed", true.into()),
                ("from", f.over.from.into()),
                ("reached", f.reached().into()),
                ("reachedText", at(f.reached())),
                ("singular", singular),
                ("text", text.into()),
            ])
        }
    }
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
/// wherever the caller wrote it.  A plane's axes and normal (`f.u.x` … `f.n.z`) are the numbers
/// here the sketch does not hold as Params: read off its axes by `Sketch::basis`, because how a
/// plane turns is what a reader asks for, and its axes' Params are another entity's.
///
/// Sorted and deduplicated by name, so a report is stable and a reader can find a name in it.
pub fn positions(sk: &Sketch, map: &crate::program::SourceMap) -> Vec<(String, f64)> {
    positions_where(sk, map, &|_| true)
}

/// `positions`, a solid read only where `asked` takes one of its names: a solid is evaluated to
/// report its volume, and one nobody asks about (a construction sweep, meshed from its field) need
/// not be.
pub fn positions_where(sk: &Sketch, map: &crate::program::SourceMap, asked: &dyn Fn(&str) -> bool) -> Vec<(String, f64)> {
    let mut out: std::collections::BTreeMap<String, f64> = Default::default();
    // a mirrored body's entity is published as its image, which goes by its name (#578)
    let images = sk.images();
    for (&e, names) in &map.names {
        let e = images.of.get(&e).copied().unwrap_or(e);
        let params = sk.entity_params(e);
        for n in names {
            let Some(scalars) = sk.scalar_names(e, n) else { continue };
            if scalars.len() != params.len() {
                continue;
            }
            for (name, &p) in scalars.iter().zip(params.iter()) {
                out.insert(name.clone(), sk.params[p as usize].value);
            }
            // **and how a plane turns in space**: its axes and its normal, as the solve left them
            // (where it stands is its own `x`, `y`, `z`, above)
            if e.kind == crate::model::EntKind::Frame {
                let b = sk.basis(e.i());
                for (key, w) in [("u", b.u), ("v", b.v), ("n", b.normal())] {
                    for (axis, x) in ["x", "y", "z"].iter().zip(w) {
                        out.insert(format!("{n}.{key}.{axis}"), x);
                    }
                }
            }
        }
    }
    // **and the unknowns the source names that no entity owns**: a free variable (`w` in
    // `distance(w)`, a formal a drawn instance left unbound, `leg.h`) and a place two contacts
    // share (`t == s`) are answers the solve gives by name, in the units the source writes them
    let shared = sk.shared.iter().map(|(n, s)| (n, &s.param));
    for (name, &p) in sk.free_vars.iter().chain(shared) {
        if !sk.params[p as usize].fixed {
            out.insert(name.clone(), sk.params[p as usize].value);
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
        if e.kind != crate::model::EntKind::Solid || !names.iter().any(|n| asked(n)) {
            continue;
        }
        let Ok(solid) = sk.evaluated_solid(e.i(), crate::solid::ApproximationPolicy::Report) else { continue };
        let pieces = solid.boundary();
        let b = solid.world_bounds();
        // Boundary pieces retain the primitive's absolute name. Recover the route through
        // this body's operands instead of dropping the first segment of every primitive.
        for n in names {
            out.insert(format!("{n}.volume"), solid.volume());
            out.insert(format!("{n}.area"), solid.area());
            if !b.is_empty() {
                for (k, axis) in ["x", "y", "z"].iter().enumerate() {
                    out.insert(format!("{n}.bounds.{axis}0"), b.lo[k]);
                    out.insert(format!("{n}.bounds.{axis}1"), b.hi[k]);
                }
            }
            let mut faces: std::collections::BTreeMap<String, f64> = Default::default();
            for p in pieces {
                if let Some(tail) = solid.face_route(&p.path) {
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
        Arg::Shared { seed, .. } => seed.map_or(Json::Null, Json::Num),
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
    // the claim block it was written in (#460), only where there is one
    if let Some(w) = c.witness {
        v.set("witness", w.into());
    }
    if !exprs.is_empty() {
        v.set("exprs", Json::Obj(exprs));
    }
    // the dimension as the statement wrote it, where that is not its text here (`w` for a
    // `param` worked out to 100): what an editor opened on it shows, since it is what is drawn
    if let Some(w) = &c.written {
        v.set("written", Json::Str(w.clone()));
    }
    v
}

/// The document's expressions, in document order — see `expr::evaluate`.
pub fn exprs_json(sk: &mut Sketch) -> Json {
    Json::Arr(
        crate::expr::evaluate(sk)
            .into_iter()
            .map(|it| {
                object([
                    ("id", (it.id as i64).into()),
                    ("attr", it.attr.into()),
                    ("text", it.text.as_str().into()),
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
    selected_callouts_json(sk, unit, None)
}

pub fn selected_callouts_json(sk: &Sketch, unit: f64, ids: Option<&[u32]>) -> Json {
    // the view each figure is laid out in (`workspace::Views::constraint_view`): -1 the page, a
    // plane's index, -2 where its points stand in views that are not one place in space
    let views = crate::overview::workspace::Views::new(sk);
    let items: Vec<Json> = callout::layout_selected(sk, unit, ids)
        .iter()
        .map(|k| {
            let mut o = callout_json(k);
            let view = sk.constraint(k.id).map_or(Err(()), |c| views.constraint_view(sk, c));
            o.set("view", Json::Int(view_code(view)));
            o
        })
        .collect();
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
    let items: Vec<Json> = crate::renderer::layout(sk, unit)
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
/// A view as the workspace's wire says it: -1 the page, a plane's index, -2 none.
fn view_code(v: Result<Option<usize>, ()>) -> i64 {
    match v {
        Ok(None) => -1,
        Ok(Some(i)) => i as i64,
        Err(()) => -2,
    }
}

/// **The workspace** (`overview::workspace`), as far as it does not depend on the eye: for every
/// view, the page first and then each plane, its place (the first view on the same plane in space)
/// and the eye square on to it; and the view each point and each drawn entity stands in.  The eye's
/// maps of each view are asked apart (`workspace_maps`), being per frame where these are per edit.
pub fn workspace_json(sk: &Sketch) -> Json {
    use crate::model::EntKind;
    let views = crate::overview::workspace::Views::new(sk);
    let all: Vec<Option<usize>> = std::iter::once(None).chain((0..sk.frames.len()).map(Some)).collect();
    let places = all.iter().map(|&v| Json::Int(view_code(Ok(views.place(v))))).collect();
    let looks = all
        .iter()
        .map(|&v| {
            let (a, e) = views.look(sk, v);
            floats(&[a, e])
        })
        .collect();
    let of = |kind: EntKind, n: usize| -> Json {
        Json::Arr((0..n).map(|i| Json::Int(view_code(views.entity_view(sk, EntRef::new(kind, i))))).collect())
    };
    object([
        ("places", Json::Arr(places)),
        ("looks", Json::Arr(looks)),
        (
            "views",
            object([
                ("point", of(EntKind::Point, sk.points.len())),
                ("line", of(EntKind::Line, sk.lines.len())),
                ("circle", of(EntKind::Circle, sk.circles.len())),
                ("arc", of(EntKind::Arc, sk.arcs.len())),
                ("spline", of(EntKind::Spline, sk.splines.len())),
                ("curve", of(EntKind::Curve, sk.curves.len())),
            ]),
        ),
    ])
}

/// The entities a rubber band from `lo` to `hi` on the eye's picture plane touches, as
/// `[kind, index]` pairs.
pub fn workspace_overlapping_json(sk: &Sketch, unit: f64, az: f64, el: f64, lo: (f64, f64), hi: (f64, f64)) -> Json {
    let proj = crate::overview::workspace::Projection::new(sk, az, el);
    Json::Arr(crate::overview::workspace::overlapping(sk, &proj, lo, hi, unit).into_iter().map(ent_json).collect())
}

pub fn overview3d_json(sk: &Sketch, unit: f64, layer: crate::overview::Layer) -> Json {
    let items: Vec<Json> = crate::overview::scene3d_of(sk, unit, layer)
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

/// **The model outline** (#435) over `sk`, the drawing `e` made: `[{label, kind, component?,
/// path, span?: [lo, hi], ents: [[kind, index]…], solid?, operand?, children}…]`, the span in the
/// document's bytes (`overview::outline`).
pub fn outline_json(e: &crate::program::Elaborated, sk: &Sketch) -> Json {
    fn node(n: &crate::overview::outline::Node) -> Json {
        let ents = n.ents.iter().map(|&e| ent_json(e)).collect();
        let mut o = object([
            ("label", Json::Str(n.label.clone())),
            ("kind", Json::Str(n.kind.as_str().to_string())),
            ("path", Json::Str(n.path.clone())),
            ("ents", Json::Arr(ents)),
        ]);
        if let Some(c) = &n.component {
            o.set("component", Json::Str(c.clone()));
        }
        if let Some(sp) = n.span {
            o.set("span", Json::Arr(vec![Json::Int(sp.lo as i64), Json::Int(sp.hi as i64)]));
        }
        if let Some(i) = n.solid {
            o.set("solid", Json::Int(i as i64));
        }
        if n.operand {
            o.set("operand", Json::Bool(true));
        }
        o.set("children", Json::Arr(n.children.iter().map(node).collect()));
        o
    }
    Json::Arr(crate::overview::outline::outline_with(e, sk).iter().map(node).collect())
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
                // one id to publish and a binding has nothing to do with the number anyway — and
                // for a kind whose kernel is built at its operands' width (a spline's length)
                (
                    "kernel",
                    match k {
                        k if k.family_kernel().is_some() || k.built() => Json::Int(-1),
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
    // which regular tangency two things touching at a held point mean (#5): every entry of
    // `constraints::tangency_at` between kinds, each side's end by its word (`null` its rim), so
    // a front end that finds a contact asks the core's table instead of keeping a copy of it
    let mut forms = Vec::new();
    let sides = |k: EntKind| {
        let mut ends = vec![None];
        if let Some(e) = k.ends() {
            ends.extend([Some(e.entry), Some(e.exit)]);
        }
        ends.into_iter().map(move |end| (k, Touch { kind: Some(k), end }))
    };
    for (ka, a) in EntKind::ALL.iter().flat_map(|&k| sides(k)) {
        for (kb, b) in EntKind::ALL.iter().flat_map(|&k| sides(k)) {
            let Some(t) = tangency_at(a, b) else { continue };
            let Some(kind) = t.kind(ka, kb) else { continue };
            forms.push(object([
                ("a", ka.as_str().into()),
                ("aEnd", a.end.and_then(|e| ka.end_name(e)).into()),
                ("b", kb.as_str().into()),
                ("bEnd", b.end.and_then(|e| kb.end_name(e)).into()),
                ("type", kind.name().into()),
                ("swap", t.swap.into()),
                ("at", t.at.into()),
            ]));
        }
    }
    object([
        ("types", Json::Arr(types)),
        ("kernels", Json::Arr(kernels)),
        ("tangencyAt", Json::Arr(forms)),
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
                    return Err(format!("{} slot does not take {}", k.a(), ek.a()));
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
    crate::constraints::seed_omitted(sk, kind, &mut args, |i| crate::io::omitted(raw.get(i)))?;
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
    c.witness = v.get("witness").filter(|_| c.claim)
        .map(|x| x.as_i64()).and_then(|w| u32::try_from(w).ok());
    c.intrinsic = v.get("intrinsic").map(|x| x.as_bool()).unwrap_or(false);
    Ok(c)
}

/// How many intervals a meshing is traced over for a report (#283).
pub const MESHING_STEPS: usize = 24;

/// What meshing `i` comes to after the solve (#283), in a line: whether its flanks touch at a
/// point or along a line, the transmission error's range over its phase, and how long the path of
/// contact is — or why it could not be traced.
pub fn meshing_text(sk: &Sketch, i: usize) -> String {
    let name = &sk.meshings[i].name;
    let t = match meshing_trace(sk, i) {
        Ok(t) => t,
        Err(why) => return format!("{name}: not traced — {why}"),
    };
    let Some([lo, hi]) = t.error_range() else {
        return format!("{name}: not traced — no contact where its point holds it");
    };
    let n = t.found().count();
    let lines = t.found().filter(|s| s.line).count();
    let contact = match lines {
        0 => "point contact",
        l if l == n => "line contact",
        _ => "point and line contact",
    };
    let deg = |v: f64| format!("{}°", crate::expr::reading(SpecKind::Scalar, v));
    let ratio = t.found().any(|s| s.gear_error.is_some());
    let what = if ratio { "the gear's angle" } else { "the gear's lag" };
    let (first, last) = (t.phases[0], *t.phases.last().expect("a trace has a phase"));
    let path = crate::expr::reading(SpecKind::Length, t.path_length());
    let mut text = format!("{name}: {contact}; transmission error ({what}) {} … {} over the phase \
                            {} … {}, the path of contact {path} long", deg(lo), deg(hi), deg(first),
                           deg(last));
    if n < t.phases.len() {
        text += &format!(" ({} of {} phases found)", n, t.phases.len());
    }
    text
}

/// What the drawing says of each body traced in space (#326): a curve or a surface, and whether
/// it is **regular** over its domain — its derivatives in its formals independent at every place
/// of a grid across it (`traced::regularity`), else where it is least so: the undercut check of a
/// flank seen from its member.
pub fn traced_text(sk: &Sketch, i: usize) -> String {
    let t = &sk.traced[i];
    let domain = t.domain();
    // a body that states a rate is integrated from its home, and says how far (#339 D2)
    if let Some(reach) = crate::traced::rate_reach(sk, i) {
        let said = reach_text(reach, t.home[0], (domain[0][0], domain[0][1]));
        return format!("{}: a curve traced in space, {said}", t.name);
    }
    // over two formals, marched along the first by the method of lines (#339 D4)
    if let Some(reach) = crate::traced::lines_reach(sk, i) {
        return format!("{}: a surface traced in space, {}", t.name,
                       lines_text(reach, t.home[0], (domain[0][0], domain[0][1])));
    }
    let what = if t.formals() == 2 { "a surface" } else { "a curve" };
    let k = crate::traced::consts_of(t);
    let theta = crate::traced::theta(sk, t);
    let n = 12;
    let grid = |d: [f64; 2], j: usize, n: usize| d[0] + (d[1] - d[0]) * j as f64 / n as f64;
    let mut least: Option<(f64, [f64; 2])> = None;
    let mut missed = 0;
    for a in 0..=n {
        for b in 0..=if t.formals() == 2 { n } else { 0 } {
            let place = [grid(domain[0], a, n), grid(domain[1], b, n)];
            match crate::traced::regularity(&k, &theta, place) {
                Some(r) if least.is_none_or(|(l, _)| r < l) => least = Some((r, place)),
                Some(_) => {}
                None => missed += 1,
            }
        }
    }
    let name = &t.name;
    let deg = |v: f64| crate::expr::reading(SpecKind::Scalar, v);
    let mut text = match least {
        None => format!("{name}: {what} traced in space, not read anywhere over its domain"),
        Some((r, _)) if r > 1e-6 => format!("{name}: {what} traced in space, regular over its domain"),
        Some((_, [u, s])) if t.formals() == 2 => {
            format!("{name}: {what} traced in space, folding near ({}, {})", deg(u), deg(s))
        }
        Some((_, [u, _])) => format!("{name}: {what} traced in space, stopping near {}", deg(u)),
    };
    if missed > 0 && least.is_some() {
        text += &format!(" ({missed} places not solved)");
    }
    text
}

/// What the drawing says of a curve whose body states a rate (#339 D2): integrated from its home
/// across its interval, or refused where its rates lose rank — the place, as a station that
/// loses rank is said today — or where the march ran past its bound of steps.  `None` for any
/// other curve.
pub fn rates_text(sk: &Sketch, i: usize) -> Option<String> {
    let cv = sk.curves.get(i)?;
    let crate::model::CurveBody::Trace(l) = &sk.curve_defs[cv.def as usize].body else {
        return None;
    };
    let home = sk.curve_home(i);
    let reach = crate::locus::rate_reach(&l.flat, &sk.curve_vars(i, home), home, cv.over.pair())?;
    Some(reach_text(reach, home, cv.over.pair()))
}

/// How far a surface's march reached each way from `home` across `domain` (#339 D4), in the
/// CLI's words: where it is refused, why — its rates lost rank at a node, or the surface folds
/// there — and at which node along the sampled formal.
fn lines_text(reach: [crate::lines::Reached; 2], home: f64, domain: (f64, f64)) -> String {
    use crate::lines::Why;
    let num = |v: f64| crate::expr::reading(SpecKind::Scalar, v);
    let mut out = format!("marched from its home at {}", num(home));
    for r in reach {
        match r {
            Ok(()) => {}
            Err(None) => return format!("not marched: its home at {} is not to be had, or is \
                not regular", num(home)),
            Err(Some((refusal, why))) => {
                let at = num(refusal.at());
                out += &match why {
                    Why::Fold { sigma } => format!(", refused at {at}: it folds there, near {} \
                        along its second formal", num(sigma)),
                    Why::Rank { sigma } if sigma.is_finite() => format!(", refused at {at}: its \
                        rates lose rank there, near {} along its second formal", num(sigma)),
                    Why::Rank { .. } => format!(", refused at {at}: its rates lose rank there"),
                    Why::Steps => format!(", refused at {at}: the march ran past its bound of \
                        steps (stiff, or near where it folds)"),
                };
            }
        }
    }
    let (a, b) = domain;
    if !out.contains("refused") {
        out += &format!(" across ({}, {})", num(a), num(b));
    }
    out
}

/// How far a march reached each way from `home` across `domain`, in the CLI's words.
fn reach_text(reach: [crate::rates::Reached; 2], home: f64, domain: (f64, f64)) -> String {
    let num = |v: f64| crate::expr::reading(SpecKind::Scalar, v);
    let mut out = format!("integrated from its home at {}", num(home));
    for r in reach {
        match r {
            Ok(()) => {}
            Err(None) => return format!("not integrated: its home at {} is not to be had",
                num(home)),
            Err(Some(crate::ode::Refusal::Undefined { at })) => {
                out += &format!(", refused at {}: its rates lose rank there", num(at))
            }
            Err(Some(crate::ode::Refusal::Steps { at })) => {
                out += &format!(", refused at {}: the march ran past its bound of steps (stiff, \
                    or near where its rates lose rank)", num(at))
            }
        }
    }
    let (a, b) = domain;
    if !out.contains("refused") {
        out += &format!(" across ({}, {})", num(a), num(b));
    }
    out
}

/// Meshing `i` traced over its phase for a report, from the first point held to it.
fn meshing_trace(sk: &Sketch, i: usize) -> Result<crate::envelope::meshing::Trace, String> {
    crate::envelope::meshing::Meshing::named(sk, i).and_then(|m| m.traced(MESHING_STEPS))
}

/// Every meshing's trace (#283): by name, each phase in degrees with the lag, the gear's angle
/// error, the contact in the fixed frame, and whether it is a line contact — or why it could not
/// be traced.
pub fn meshings_json(sk: &Sketch) -> Json {
    let one = |i: usize| -> Json {
        let name = Json::Str(sk.meshings[i].name.clone());
        match meshing_trace(sk, i) {
            Err(why) => object([("name", name), ("error", Json::Str(why))]),
            Ok(t) => {
                let rows = t.phases.iter().zip(&t.samples).map(|(&phase, s)| match s {
                    None => object([("phase", Json::Num(phase))]),
                    Some(s) => object([
                        ("phase", Json::Num(phase)),
                        ("lag", Json::Num(s.place.lag)),
                        ("gear_error", s.gear_error.map_or(Json::Null, Json::Num)),
                        ("position", floats(&s.position)),
                        ("line", Json::Bool(s.line)),
                    ]),
                }).collect();
                object([("name", name), ("trace", Json::Arr(rows))])
            }
        }
    };
    Json::Arr((0..sk.meshings.len()).map(one).collect())
}

/// The same immutable claim result supplies terminal and JSON reports.
pub fn solid_claim_text(v: &crate::diagnose::SolidVerdict) -> String {
    use crate::diagnose::SolidOutcome;
    let outcome = match v.outcome() {
        SolidOutcome::SampledSuccess => "sampled success (continuous interval unproven)",
        SolidOutcome::Holds => "holds",
        SolidOutcome::Refuted => "refuted",
        SolidOutcome::Indeterminate => "undecided",
    };
    let mut text = format!("{} — {outcome}", v.text());
    // a claim at the drawing's pose that could not be measured says why, which names the solid
    // (#615); poses that did not solve are a sweep's
    let failed_here = match v.poses() {
        [p] if v.sweep().is_none() => p.evaluation().err(),
        _ => None,
    };
    if let Some(reason) = failed_here {
        text += &format!(": {reason}");
    } else if v.valid_samples().next().is_none() {
        text += ": no solved valid poses";
    }
    if let Some(i) = v
        .measurement()
        .and_then(crate::clear::Measurement::interval)
    {
        text += &format!(
            ", measured {} (interval [{}, {}])",
            crate::syntax::num(i.midpoint()),
            crate::syntax::num(i.lower()),
            crate::syntax::num(i.upper())
        );
    }
    if let Some(sw) = v.sweep() {
        let unit = if sw.dimension() == crate::units::Dim::ANGLE {
            "deg"
        } else {
            ""
        };
        text += &format!(
            ", sampling {} poses ({} failed), {} in [{}{}, {}{}], {} valid",
            v.samples(),
            v.failures().count(),
            sw.name(),
            crate::syntax::num(sw.from()),
            unit,
            crate::syntax::num(sw.to()),
            unit,
            v.valid_samples().count()
        );
        if let Some(p) = v.worst() {
            text += &format!(
                ", {} at {}{}",
                if v.counterexample().is_some() {
                    "counterexample"
                } else {
                    "worst"
                },
                crate::syntax::num(p),
                unit
            );
        }
    }
    // Every unresolved pose is disclosed, including failed solves and uncertain geometry; a lone
    // claim's failure is said once, above.
    for p in v.poses().iter().filter(|p| failed_here.is_none() && p.holds().is_none()) {
        let reason = match p.evaluation() {
            Err(reason) => reason.to_string(),
            Ok(e) => match e.predicate() {
                crate::clear::Predicate::Unresolved(reason) => reason.clone(),
                _ => "requested spacing is within geometric uncertainty".into(),
            },
        };
        let at = p
            .parameter()
            .map(|t| format!(" at {}", crate::syntax::num(t)))
            .unwrap_or_default();
        text += &format!("\n    unresolved{at}: {reason}");
    }
    text
}

fn solid_measurement_json(m: Option<&crate::clear::Measurement>) -> Json {
    use crate::clear::Measurement;
    match m {
        Some(Measurement::Bounded(i)) => object([
            ("kind", "bounded".into()),
            ("lower", Json::Num(i.lower())),
            ("upper", Json::Num(i.upper())),
        ]),
        Some(Measurement::Unbounded) => object([("kind", "unbounded".into())]),
        None => object([("kind", "absent".into())]),
    }
}

fn solid_pose_json(p: &crate::diagnose::SolidPose) -> Json {
    let mut o = object([(
        "parameter",
        p.parameter().map(Json::Num).unwrap_or(Json::Null),
    )]);
    match p.evaluation() {
        Err(reason) => {
            o.set("status", "failed".into());
            o.set("reason", reason.into());
        }
        Ok(e) => {
            let predicate = e.predicate();
            o.set("status", "valid".into());
            o.set("measurement", solid_measurement_json(Some(e.measurement())));
            o.set(
                "verdict",
                match e.holds() {
                    Some(true) => "holds",
                    Some(false) => "refuted",
                    None => "undecided",
                }
                .into(),
            );
            let mut required = object([
                ("kind", e.predicate_name().into()),
                (
                    "status",
                    match predicate {
                        crate::clear::Predicate::Satisfied => "satisfied",
                        crate::clear::Predicate::Refuted => "refuted",
                        crate::clear::Predicate::Unresolved(_) => "unresolved",
                    }
                    .into(),
                ),
            ]);
            if let crate::clear::Predicate::Unresolved(reason) = predicate {
                required.set("reason", reason.clone().into());
            }
            o.set("requiredPredicate", required);
        }
    }
    o
}

pub fn solid_claim_json(v: &crate::diagnose::SolidVerdict) -> Json {
    let mut o = object([
        ("statement", v.text().into()),
        ("text", solid_claim_text(v).into()),
        (
            "measured",
            v.measured().map(Json::Num).unwrap_or(Json::Null),
        ),
        (
            "tolerance",
            v.tolerance().map(Json::Num).unwrap_or(Json::Null),
        ),
        ("measurement", solid_measurement_json(v.measurement())),
        ("verdict", v.outcome().as_str().into()),
        (
            "method",
            if v.sweep().is_some() {
                "sampling"
            } else {
                "single-pose"
            }
            .into(),
        ),
        ("continuousProof", false.into()),
        ("validSamples", Json::Int(v.valid_samples().count() as i64)),
        (
            "poses",
            Json::Arr(v.poses().iter().map(solid_pose_json).collect()),
        ),
        (
            "counterexample",
            v.counterexample()
                .map(solid_pose_json)
                .unwrap_or(Json::Null),
        ),
    ]);
    if let Some(sw) = v.sweep() {
        o.set("samples", Json::Int(v.samples() as i64));
        o.set(
            "failedSamples",
            Json::Arr(v.failed_samples().into_iter().map(Json::Num).collect()),
        );
        o.set(
            "coverage",
            object([
                ("parameter", sw.name().into()),
                ("from", Json::Num(sw.from())),
                ("to", Json::Num(sw.to())),
                ("dimension", sw.dimension().name().into()),
                (
                    "units",
                    if sw.dimension() == crate::units::Dim::ANGLE {
                        "degrees"
                    } else {
                        "user"
                    }
                    .into(),
                ),
                ("method", "uniform-inclusive".into()),
                ("attempted", Json::Int(v.samples() as i64)),
            ]),
        );
    }
    if let Some(w) = v.worst() {
        o.set("worst", Json::Num(w));
    }
    o
}
