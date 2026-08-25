//! Does the order of a document's constraint list mean anything?
//!
//! Solvent (`solvent-spec.md`, P2) says a component body is an *unordered set*: reordering the
//! statements of a body must not change the meaning of a program.  Before building a language on
//! that promise, find out what in this core disagrees with it — because the failure mode is
//! quiet.  Reorder two statements and a callout jumps to a different dimension, or a recorded
//! root choice stops applying, and the drawing changes with no error anywhere.
//!
//! The experiment: shuffle each example's constraint list with a seeded permutation, **carrying
//! every piece of document state with the constraint it belongs to** — a placement with its own
//! dimension, a branch with its own points — reload, and ask whether the rest of the system
//! noticed.  The carrying is done correctly here on purpose: what is under test is not whether a
//! shuffle can be done, but whether anything downstream depends on the order once it has been.
//!
//! What each test finds is recorded in its own doc comment.  Nothing here is a proposal; this
//! file is a measurement.

use gcs_core::constraints::Constraint;
use gcs_core::decompose::{self, PlanSolver};
use gcs_core::diagnose::{diagnose, DiagnoseOptions};
use gcs_core::examples;
use gcs_core::io;
use gcs_core::json::{parse, Json};
use gcs_core::model::Sketch;
use gcs_core::newton::Method;
use gcs_core::rng::Rng;

/// The cases worth shuffling: every reference sketch, plus the ones that carry the document state
/// this is about — `pythagoras` for dimension expressions and `belt_tangency` / `rect_fillets`
/// for the closed-form constructions that record a root choice.
fn cases() -> Vec<(&'static str, Sketch)> {
    let mut v: Vec<(&'static str, Sketch)> = examples::EXAMPLES
        .iter()
        .map(|&n| (n, examples::example(n).expect(n)))
        .collect();
    for n in ["pythagoras", "belt_tangency", "altitudes", "parallels", "k33"] {
        v.push((n, examples::case(n).expect(n)));
    }
    v
}

/// Give a sketch the document state a shuffle is supposed to carry: a callout placement on every
/// dimension (deterministic, and distinct per constraint so a swap is visible), and whatever root
/// choices the decomposition records for it.
fn furnish(sk: &mut Sketch) {
    let dims: Vec<u32> = sk
        .user_constraints()
        .iter()
        .filter(|c| !c.dimensions().is_empty())
        .map(|c| c.id)
        .collect();
    for (i, id) in dims.iter().enumerate() {
        sk.placements.insert(*id, (1.0 + i as f64, -2.0 - i as f64));
    }
    let mut ps = PlanSolver::new(sk, true);
    ps.solve(sk, 1e-9, true, Method::DogLeg);
}

/// A document with its constraint list permuted, and every placement key moved with the
/// constraint it names.  Placements are stored *by position in that list* (`io::to_json`), so
/// carrying them is the shuffle's own job — and the fact that it is is the first finding.
fn shuffle(doc: &Json, seed: u32) -> Json {
    let cs = doc.get("constraints").map(|c| c.arr()).unwrap_or(&[]);
    let n = cs.len();
    let mut perm: Vec<usize> = (0..n).collect();
    let mut rng = Rng::new(seed);
    for i in (1..n).rev() {
        perm.swap(i, rng.int(i + 1));
    }
    let moved: Vec<Json> = perm.iter().map(|&i| cs[i].clone()).collect();
    // `perm[new] = old`, and a placement is keyed by the old position
    let mut places: Vec<(String, Json)> = Vec::new();
    if let Some(Json::Obj(p)) = doc.get("placements") {
        for (k, v) in p {
            let old: usize = k.parse().expect("a placement key is a position");
            let new = perm.iter().position(|&i| i == old).expect("every position is in the perm");
            places.push((new.to_string(), v.clone()));
        }
    }
    places.sort_by(|a, b| a.0.cmp(&b.0));
    let Json::Obj(fields) = doc else { panic!("a document is an object") };
    Json::Obj(
        fields
            .iter()
            .map(|(k, v)| match k.as_str() {
                "constraints" => (k.clone(), Json::Arr(moved.clone())),
                "placements" => (k.clone(), Json::Obj(places.clone())),
                _ => (k.clone(), v.clone()),
            })
            .collect(),
    )
}

/// A document with its **point declarations** permuted, and every reference to a point rewritten
/// to follow.  This is the sharper half of the question: a constraint's order is only its own,
/// but a point's order is its *index*, and an index is what `decompose::branch_key` names a
/// recorded root choice by and what `io::describe` calls the point.
///
/// Only points move.  Lines, circles, arcs, splines and ellipses keep their own order, so any
/// difference is attributable to the renumbering alone.
fn shuffle_points(doc: &Json, seed: u32) -> Json {
    let pts = doc.get("points").map(|p| p.arr()).unwrap_or(&[]);
    let n = pts.len();
    let mut perm: Vec<usize> = (0..n).collect(); // perm[new] = old
    let mut rng = Rng::new(seed);
    for i in (1..n).rev() {
        perm.swap(i, rng.int(i + 1));
    }
    let mut to_new = vec![0usize; n];
    for (new, &old) in perm.iter().enumerate() {
        to_new[old] = new;
    }
    let moved: Vec<Json> = perm.iter().map(|&i| pts[i].clone()).collect();
    let pt = |v: &Json| Json::Int(to_new[v.as_i64() as usize] as i64);
    // a point reference is a field on an entity, an element of a spline's `ctrl`, or an
    // `["point", i]` argument of a constraint
    let ent = |e: &Json, fields: &[&str]| -> Json {
        let Json::Obj(o) = e else { return e.clone() };
        Json::Obj(
            o.iter()
                .map(|(k, v)| {
                    if fields.contains(&k.as_str()) {
                        (k.clone(), pt(v))
                    } else if k == "ctrl" {
                        (k.clone(), Json::Arr(v.arr().iter().map(pt).collect()))
                    } else {
                        (k.clone(), v.clone())
                    }
                })
                .collect(),
        )
    };
    let arg = |a: &Json| -> Json {
        match a {
            Json::Arr(v) if v.len() == 2 && v[0].as_str() == "point" => {
                Json::Arr(vec![v[0].clone(), pt(&v[1])])
            }
            _ => a.clone(),
        }
    };
    let con = |c: &Json| -> Json {
        let Json::Obj(o) = c else { return c.clone() };
        Json::Obj(
            o.iter()
                .map(|(k, v)| {
                    if k == "args" {
                        (k.clone(), Json::Arr(v.arr().iter().map(arg).collect()))
                    } else {
                        (k.clone(), v.clone())
                    }
                })
                .collect(),
        )
    };
    // a recorded root choice is keyed by a triple of point indices, and `io::graft` already has
    // to renumber them by hand when points move — so a shuffle must too, or it is testing the
    // renumbering rather than the order
    let branches = |b: &Json| -> Json {
        let Json::Obj(o) = b else { return b.clone() };
        Json::Obj(
            o.iter()
                .map(|(k, v)| {
                    let key = decompose::branch_key_points(k)
                        .map(|t| decompose::branch_key([to_new[t[0]], to_new[t[1]], to_new[t[2]]]))
                        .unwrap_or_else(|| k.clone());
                    (key, v.clone())
                })
                .collect(),
        )
    };
    let Json::Obj(fields) = doc else { panic!("a document is an object") };
    Json::Obj(
        fields
            .iter()
            .map(|(k, v)| {
                let out = match k.as_str() {
                    "points" => Json::Arr(moved.clone()),
                    "lines" => Json::Arr(v.arr().iter().map(|e| ent(e, &["p1", "p2"])).collect()),
                    "circles" => Json::Arr(v.arr().iter().map(|e| ent(e, &["center"])).collect()),
                    "arcs" => Json::Arr(
                        v.arr().iter().map(|e| ent(e, &["center", "start", "end"])).collect(),
                    ),
                    "splines" => Json::Arr(v.arr().iter().map(|e| ent(e, &[])).collect()),
                    "ellipses" => {
                        Json::Arr(v.arr().iter().map(|e| ent(e, &["center", "major"])).collect())
                    }
                    "constraints" => Json::Arr(v.arr().iter().map(con).collect()),
                    "branches" => branches(v),
                    _ => v.clone(),
                };
                (k.clone(), out)
            })
            .collect(),
    )
}

/// The drawing as a *set* of places, so it can be compared across a renumbering of the points.
/// Sorted rather than indexed, because the whole point is that the indices moved.
fn places(sk: &Sketch) -> Vec<(f64, f64)> {
    let mut v: Vec<(f64, f64)> = (0..sk.points.len()).map(|i| sk.point_xy(i)).collect();
    v.sort_by(|a, b| a.partial_cmp(b).expect("a solved coordinate is a number"));
    v
}

/// Two drawings are the same drawing if every place matches to `tol`.
///
/// Compared as numbers and never as text: renumbering the points changes the column order the
/// factorisation pivots on, so a coordinate that lands on 0 in one may land on -3e-15 in the
/// other.  That is the same place; only a formatter would ever say otherwise, and one did.
fn same_places(a: &[(f64, f64)], b: &[(f64, f64)], tol: f64) -> bool {
    a.len() == b.len()
        && a.iter().zip(b).all(|(p, q)| (p.0 - q.0).abs() <= tol && (p.1 - q.1).abs() <= tol)
}

/// How many of a document's recorded root choices the plan built from it can actually use.  A
/// choice whose key names no construction is inert: the document still carries it, and it decides
/// nothing.
fn branches_that_apply(sk: &Sketch) -> usize {
    let mut ps = PlanSolver::new(sk, true);
    ps.plan.apply_branches(&sk.branches)
}

/// What a constraint says, with no reference to any list position: its own text, and the
/// placement (if any) that hangs off it.  Two documents saying the same thing have the same
/// multiset of these however their lists are ordered.
fn statements(sk: &Sketch) -> Vec<String> {
    let mut v: Vec<String> = sk
        .user_constraints()
        .iter()
        .map(|c| {
            let place = sk
                .placements
                .get(&c.id)
                .map(|&(t, r)| format!(" at ({t}, {r})"))
                .unwrap_or_default();
            format!("{}{place}", io::describe(c))
        })
        .collect();
    v.sort();
    v
}

/// The drawing, read off the entities rather than the parameter vector.
///
/// `Sketch::get_x` is in parameter order, and parameter order is exactly what a reload permutes:
/// `add_quiet` allocates a constraint's own unknowns as it goes, interleaved with the entities',
/// while `io::from_json` builds every entity first.  Comparing two `get_x` across a load compares
/// different parameters and reports a drift that is only the permutation.  Entity indices *are*
/// stable — nothing here reorders entities — so this is what "the drawing moved" has to mean.
fn geometry(sk: &Sketch) -> Vec<f64> {
    let mut v = Vec::new();
    for i in 0..sk.points.len() {
        let (x, y) = sk.point_xy(i);
        v.push(x);
        v.push(y);
    }
    for c in &sk.circles {
        v.push(sk.params[c.radius as usize].value);
    }
    for a in &sk.arcs {
        v.push(sk.params[a.radius as usize].value);
    }
    for e in &sk.ellipses {
        v.push(sk.params[e.minor as usize].value);
    }
    v
}

fn close(a: &[f64], b: &[f64], tol: f64) -> bool {
    a.len() == b.len() && a.iter().zip(b).all(|(x, y)| (x - y).abs() <= tol)
}

/// A parameter named by what owns it, for the same reason: an index is not comparable across a
/// load.  An entity's parameters are named by its own stable `(kind, index)`; one a constraint
/// owns is named by what that constraint *says*, which is the only handle on it that a
/// reordering leaves alone.
fn param_names(sk: &Sketch) -> Vec<String> {
    let mut names = vec![String::new(); sk.params.len()];
    for i in 0..sk.points.len() {
        let [x, y] = sk.point_params(i);
        names[x as usize] = format!("p{i}.x");
        names[y as usize] = format!("p{i}.y");
    }
    for (i, c) in sk.circles.iter().enumerate() {
        names[c.radius as usize] = format!("c{i}.r");
    }
    for (i, a) in sk.arcs.iter().enumerate() {
        names[a.radius as usize] = format!("a{i}.r");
    }
    for (i, e) in sk.ellipses.iter().enumerate() {
        names[e.minor as usize] = format!("e{i}.b");
    }
    for c in &sk.constraints {
        for (n, p) in c.aux_params().iter().enumerate() {
            names[*p as usize] = format!("{}#{n}", io::describe(c));
        }
    }
    for (n, p) in &sk.free_vars {
        names[*p as usize] = format!("${n}");
    }
    names
}

/// Which parameters can move, said by owner and sorted — a set, not a numbering.
fn movable(sk: &Sketch, ps: &[u32]) -> Vec<String> {
    let names = param_names(sk);
    let mut v: Vec<String> = ps.iter().map(|&p| names[p as usize].clone()).collect();
    v.sort();
    v
}

const SEEDS: [u32; 4] = [1, 7, 42, 1337];

/// **The document survives.**  Every constraint, every argument and every placement comes back
/// attached to what it was attached to — provided the shuffle remaps the placement keys, which is
/// the whole of the finding: a placement is keyed by *position in the constraint list*
/// (`io::to_json`), so nothing but the shuffle itself can keep them together.  A language whose
/// statements are a set has no positions to key on, which is why a placement must ride on its own
/// statement.
#[test]
fn a_shuffled_document_says_the_same_thing() {
    for (name, mut sk) in cases() {
        furnish(&mut sk);
        let want = statements(&sk);
        let doc = io::to_json(&sk);
        for seed in SEEDS {
            let sk2 = io::from_json(&shuffle(&doc, seed)).expect(name);
            assert_eq!(statements(&sk2), want, "{name} seed {seed}");
        }
    }
}

/// **A placement keyed by position is a trap, and this is the proof.**  Shuffle the list *without*
/// remapping the keys — which is exactly what a naive reordering of statements would do — and
/// callouts land on other dimensions.  This test asserts the damage, so that when placements move
/// onto their own statements it becomes the test that they no longer can.
#[test]
fn a_placement_keyed_by_position_follows_the_position() {
    let mut hurt = 0;
    for (name, mut sk) in cases() {
        furnish(&mut sk);
        if sk.placements.is_empty() {
            continue;
        }
        let want = statements(&sk);
        let doc = io::to_json(&sk);
        for seed in SEEDS {
            // the shuffle a language with no positions would perform: move the statements, leave
            // the position-keyed table alone
            let mut naive = shuffle(&doc, seed);
            if let Json::Obj(fields) = &mut naive {
                for (k, v) in fields.iter_mut() {
                    if k == "placements" {
                        *v = doc.get("placements").cloned().unwrap_or(Json::obj());
                    }
                }
            }
            let sk2 = io::from_json(&naive).expect(name);
            if statements(&sk2) != want {
                hurt += 1;
            }
        }
    }
    assert!(hurt > 0, "if this ever stops failing, placements no longer travel by position");
}

/// **The solve does not care.**  Same constraints in another order, same answer — so the
/// parameter-vector permutation a reordering causes is not enough to move a converged solution.
#[test]
fn a_shuffled_document_solves_to_the_same_place() {
    for (name, mut sk) in cases() {
        furnish(&mut sk);
        let want = geometry(&sk);
        let doc = io::to_json(&sk);
        for seed in SEEDS {
            let mut sk2 = io::from_json(&shuffle(&doc, seed)).expect(name);
            let mut ps = PlanSolver::new(&sk2, true);
            ps.solve(&mut sk2, 1e-9, true, Method::DogLeg);
            assert!(
                close(&geometry(&sk2), &want, 1e-6),
                "{name} seed {seed}: the drawing moved",
            );
        }
    }
}

/// **The diagnosis does not care either** — about its verdict.  Which constraint it *names* as the
/// surplus of a dependency may well move, since "one of these is redundant" has no canonical
/// member; that is checked as a count, not as a set.
#[test]
fn a_shuffled_document_diagnoses_the_same() {
    for (name, mut sk) in cases() {
        furnish(&mut sk);
        let d = diagnose(&mut sk, DiagnoseOptions::default());
        let doc = io::to_json(&sk);
        for seed in SEEDS {
            let mut sk2 = io::from_json(&shuffle(&doc, seed)).expect(name);
            let d2 = diagnose(&mut sk2, DiagnoseOptions::default());
            let at = format!("{name} seed {seed}");
            assert_eq!(d2.n_params, d.n_params, "{at}: free parameters");
            assert_eq!(d2.n_equations, d.n_equations, "{at}: equations");
            assert_eq!(d2.structural_rank, d.structural_rank, "{at}: structural rank");
            assert_eq!(d2.numeric_rank, d.numeric_rank, "{at}: numeric rank");
            assert_eq!(d2.over.len(), d.over.len(), "{at}: how many are over");
            assert_eq!(d2.implied.len(), d.implied.len(), "{at}: how many are implied");
            assert_eq!(
                movable(&sk2, &d2.under_params),
                movable(&sk, &d.under_params),
                "{at}: which parameters can move",
            );
        }
    }
}

/// **The recorded root choices are where the order shows.**  `decompose::branch_key` names a
/// triangle by the *indices* of its three points and the plan is built greedily in constraint
/// order, so a reordering can produce a different plan, whose triangles have different keys, and
/// the choices the document recorded then apply to nothing.  Entities are not reordered here — only
/// constraints — so any difference is the plan's alone.
#[test]
fn a_shuffled_document_keeps_its_root_choices() {
    let mut lost = Vec::new();
    for (name, mut sk) in cases() {
        furnish(&mut sk);
        let want = sk.branches.clone();
        if want.is_empty() {
            continue;
        }
        let doc = io::to_json(&sk);
        for seed in SEEDS {
            let mut sk2 = io::from_json(&shuffle(&doc, seed)).expect(name);
            let mut ps = PlanSolver::new(&sk2, true);
            ps.solve(&mut sk2, 1e-9, true, Method::DogLeg);
            let missing: Vec<&String> = want.keys().filter(|k| !sk2.branches.contains_key(*k)).collect();
            if !missing.is_empty() {
                lost.push(format!("{name} seed {seed}: {missing:?}"));
            }
        }
    }
    assert!(lost.is_empty(), "root choices a reordering threw away:\n  {}", lost.join("\n  "));
}

/// **Renumbering the points changes nothing either** — the drawing lands in the same places and
/// the recorded root choices still apply, *provided the branch keys are renumbered with the
/// points*.  That proviso is the finding: `decompose::branch_key` names a construction by a
/// **triple of point indices**, which is why `io::graft` already has to rewrite them by hand
/// whenever points move, and why it drops one whose point did not come along.
///
/// A declaration's position is not something a reader would take to mean anything, so in a
/// language a root choice has to name its points rather than their positions.
#[test]
fn renumbering_the_points_keeps_the_drawing_and_its_root_choices() {
    for (name, mut sk) in cases() {
        furnish(&mut sk);
        let want = places(&sk);
        let applied = branches_that_apply(&sk);
        let doc = io::to_json(&sk);
        for seed in SEEDS {
            let mut sk2 = io::from_json(&shuffle_points(&doc, seed)).expect(name);
            let at = format!("{name} seed {seed}");
            assert_eq!(branches_that_apply(&sk2), applied, "{at}: the root choices still apply");
            let mut ps = PlanSolver::new(&sk2, true);
            ps.solve(&mut sk2, 1e-9, true, Method::DogLeg);
            assert!(same_places(&places(&sk2), &want, 1e-6), "{at}: the drawing moved");
        }
    }
}

/// **A root choice keyed by point index follows the index.**  Renumber the points *without*
/// renumbering the keys — what reordering declarations would do to a language that stored them
/// positionally — and the recorded choices go inert: the document still carries them, `from_json`
/// still loads them, and they name constructions that no longer exist, so they decide nothing.
///
/// The damage is silent, which is the point of measuring it.  The counterpart of the placement
/// test: it asserts the damage, so that keying by name becomes the test that it can no longer
/// happen.
#[test]
fn a_root_choice_keyed_by_index_goes_inert() {
    let (mut inert, mut had) = (0, 0);
    for (name, mut sk) in cases() {
        furnish(&mut sk);
        let applied = branches_that_apply(&sk);
        if applied == 0 {
            continue;
        }
        had += 1;
        let doc = io::to_json(&sk);
        for seed in SEEDS {
            let mut naive = shuffle_points(&doc, seed);
            if let Json::Obj(fields) = &mut naive {
                for (k, v) in fields.iter_mut() {
                    if k == "branches" {
                        *v = doc.get("branches").cloned().unwrap_or(Json::obj());
                    }
                }
            }
            let sk2 = io::from_json(&naive).expect(name);
            assert_eq!(sk2.branches.len(), sk.branches.len(), "{name}: they are still carried");
            if branches_that_apply(&sk2) < applied {
                inert += 1;
            }
        }
    }
    assert!(had > 0, "no case records a root choice that applies — measuring nothing");
    assert!(inert > 0, "if this ever stops failing, root choices no longer travel by point index");
}

/// **A JSON round trip does not preserve `topology_key`**, and this is not about shuffling at all.
/// `Sketch::add_quiet` allocates a constraint's own unknowns the moment it is added, interleaved
/// with entity parameters; `io::from_json` builds every entity first and every constraint after.
/// So the parameter vector permutes on a save and a load, the compiled-plan cache misses, and the
/// column order that pivoting sees is not the one it saw before.
///
/// This is why the acceptance bar for a text format is *document-state preservation* and not
/// `Sketch` identity: the format we already ship does not meet the stricter one.
#[test]
fn a_round_trip_does_not_preserve_the_topology_key() {
    let mut moved = Vec::new();
    for (name, sk) in cases() {
        let s = io::dumps(&sk, Some(1));
        let sk2 = io::loads(&s).expect(name);
        assert_eq!(io::dumps(&sk2, Some(1)), s, "{name}: the document itself is a fixed point");
        if sk2.topology_key() != sk.topology_key() {
            moved.push(name);
        }
    }
    assert!(
        !moved.is_empty(),
        "if this ever stops failing, the parameter vector now survives a load and the note above \
         is stale",
    );
}

/// The shuffle is a permutation and nothing else: same length, same multiset of statements.
#[test]
fn the_shuffle_itself_loses_nothing() {
    let mut sk = examples::example("rect_fillets").unwrap();
    furnish(&mut sk);
    let doc = io::to_json(&sk);
    let n = doc.get("constraints").map(|c| c.arr().len()).unwrap_or(0);
    assert!(n > 4, "a case worth shuffling");
    for seed in SEEDS {
        let s = shuffle(&doc, seed);
        assert_eq!(s.get("constraints").map(|c| c.arr().len()), Some(n));
        assert_eq!(
            s.get("placements").map(|p| match p {
                Json::Obj(o) => o.len(),
                _ => 0,
            }),
            doc.get("placements").map(|p| match p {
                Json::Obj(o) => o.len(),
                _ => 0,
            }),
        );
        assert_ne!(parse(&s.dump(None)).unwrap().dump(None), "", "it is still a document");
    }
}

/// A constraint's own text is what a language would print, so it must not depend on where in the
/// list the constraint sits.  `io::describe` names entities by index, which the shuffle does not
/// touch — a guard on that assumption, so the tests above are measuring what they claim to.
#[test]
fn describe_does_not_depend_on_list_position() {
    let sk = examples::example("slotted_link").unwrap();
    let cs: Vec<&Constraint> = sk.user_constraints();
    let said: Vec<String> = cs.iter().map(|c| io::describe(c)).collect();
    let sk2 = io::from_json(&shuffle(&io::to_json(&sk), 3)).unwrap();
    for text in &said {
        assert!(
            sk2.user_constraints().iter().any(|c| &io::describe(c) == text),
            "{text}: the same statement, wherever it landed",
        );
    }
}
