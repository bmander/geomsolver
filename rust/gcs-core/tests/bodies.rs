//! **A solid read in three dimensions** (#510 S3): a solid a requirement names, and no plane
//! reading reads in its plane, is meshed in quadratic tetrahedra (`bodies::Mesh`) of the
//! interior's solid element (`interior::Tet10`), and its requirements are answered and judged
//! (`bodies::read`).  The corpus's solid cases answer their references, and what has no answer
//! says why.  The rules are `docs/contract/analysis.md`'s.
use gcs_core::diagnose::analysis::scenario_read;
use gcs_core::model::Output;
use gcs_core::program::Elaborated;

use crate::analysis_corpus::{read as read_case, Reading, CASES};

/// Each requirement's answer under each scenario of `e`, as the report reads it, with the
/// scenario's words: `(scenario, output, count, answer)`.
#[allow(clippy::type_complexity)]
fn answers(e: &Elaborated)
    -> (Vec<(String, Output, Option<u32>, Result<f64, String>)>, Vec<String>)
{
    let (mut out, mut words) = (Vec::new(), Vec::new());
    for s in &e.scenarios {
        let (said, read) = scenario_read(&e.sketch, &s.scenario);
        words.extend(said);
        for (r, a) in s.scenario.requirements.iter().zip(read.answers) {
            let a = a.unwrap_or_else(|| Err("about nothing read".to_string()));
            out.push((s.scenario.name.clone(), r.output, r.count, a));
        }
    }
    (out, words)
}

/// **Every solid case of the corpus answers its reference** (#510 S3), read in three
/// dimensions: the cantilever's tip sag and first mode, Euler's column's buckling factor, Lamé's
/// bore and its stress, the plate's six modes — each within 1% (stress is read at nodes, the
/// elements' averaged there), and every claim kept, with nothing said against it.  Each answer
/// has its error estimated beside it, and where the reference is exact in three dimensions
/// (Lamé's, the plate's Rayleigh–Lamb roots) the estimate is no less than the error, or both are
/// within 1e-4 of the value (Lamé's bore, within 3e-5, where the arcs' pieces need not converge
/// monotonically).
#[test]
fn every_solid_case_answers_its_reference() {
    for case in CASES.iter().filter(|c| c.reading == Reading::Solid) {
        let e = read_case(case);
        let want = (case.answers)(case, &e);
        let exact = matches!(case.name, "lame" | "plate_modes");
        for s in &e.scenarios {
            let (words, read) = scenario_read(&e.sketch, &s.scenario);
            assert!(words.is_empty(), "{}: {words:?}", case.name);
            for (k, r) in s.scenario.requirements.iter().enumerate() {
                let v = read.answers[k].clone().expect("an answer").unwrap_or_else(|m| {
                    panic!("{}: `{}` {:?} has no answer: {m}", case.name, s.scenario.name,
                           r.output)
                });
                assert!(gcs_core::beams::keeps(r, v), "{}: {r:?} is {v}", case.name);
                let a = want.iter().find(|a| (a.scenario, a.output, a.count)
                    == (s.scenario.name.as_str(), r.output, r.count)).expect("a reference");
                assert!((v - a.reference).abs() <= 1e-2 * a.reference.abs(),
                        "{}: {:?} is {v}, its reference {}", case.name, r.output, a.reference);
                let error = read.errors[k].expect("an estimate");
                eprintln!("{}: {:?} {:?} is {v} ± {error:.3e}, its reference {} ({:+.3}%)",
                          case.name, r.output, r.count, a.reference,
                          100.0 * (v / a.reference - 1.0));
                if exact {
                    let off = (v - a.reference).abs();
                    assert!(off <= error.max(1e-4 * a.reference.abs()),
                            "{}: {:?}'s estimate {error} is under its error {off}", case.name,
                            r.output);
                }
            }
        }
    }
}

/// A bar `w` by `h` by `t` in the front, swept across y about it: its left face held along x,
/// its corner `o` along y and z and its corner `d` above it along y — just what takes out the
/// rigid motions, so its narrowing is free — and pulled by `−σ` on its right face; its far
/// corner `k` and a point `q` drawn inside it, loaded by nothing.
fn bar(extra: &str) -> String {
    format!("\
unit mm
use std
w := 80mm
h := 30mm
t := 10mm
sigma := 40MPa
steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3, yield: 250MPa)
in std.front {{
  o := point
  b := point
  k := point
  d := point
  q := point
  fix((0mm, 0mm)) o
  fix((w, 0mm)) b
  fix((w, h)) k
  fix((0mm, h)) d
  fix((50mm, 10mm)) q
  outline := (low := line(o, b)) -> (right := line(b, k)) -> (top := line(k, d)) ->
             (left := line(d, o)) -> close
}}
tilted := axis
fix(dir == (1, 0, 1), origin == (0, 0, 0)) tilted
plate := solid(outline, from: -t / 2, to: t / 2) of steel
scenario pull {{
  pinned plate.left along: std.x
  pinned o along: std.z
  pinned o along: std.y
  pinned d along: std.y
  pressure plate.right -sigma
{extra}}}
")
}

/// **A bar pulled answers to rounding** (#510 S3): its stress is uniform, which the quadratic
/// tetrahedron holds exactly, so its far corner moves `σ w / E` along it and `−ν σ h / E` across,
/// the work the pull does is `σ² w h t / E`, its factor of safety is the yield over `σ`, its mass
/// `ρ w h t`, and a point drawn inside it is a node of its mesh, moving as the field says — each
/// with nothing for its error, the coarser mesh reading the same.
#[test]
fn a_bar_pulled_answers_to_rounding() {
    let claims = "  claim deflection(k, along: std.x) >= 0mm\n  \
                  claim deflection(k, along: std.z) <= 0mm\n  claim compliance(plate) >= 0J\n  \
                  claim safety(plate) >= 1\n  claim mass(plate) >= 1g\n  \
                  claim deflection(q, along: std.x) >= 0mm\n";
    let (w, h, t, s, young, nu, rho, yields) =
        (80.0, 30.0, 10.0, 40e3, 200e6, 0.3, 7850.0 * 1e-9, 250e3);
    let e = crate::common::text::solved(&bar(claims));
    let (words, read) = scenario_read(&e.sketch, &e.scenarios[0].scenario);
    assert!(words.is_empty(), "{words:?}");
    let v: Vec<f64> = read.answers.iter().map(|a| a.clone().unwrap().unwrap()).collect();
    let near = |got: f64, want: f64, what: &str| {
        assert!((got - want).abs() <= 1e-9 * want.abs(), "{what} is {got}, not {want}")
    };
    near(v[0], s * w / young, "the corner's move");
    near(v[1], -nu * s * h / young, "the corner's narrowing");
    near(v[2], s * s * w * h * t / young, "the pull's work");
    near(v[3], yields / s, "the factor of safety");
    near(v[4], rho * w * h * t, "the mass");
    near(v[5], s * 50.0 / young, "the inner point's move");
    for (k, error) in read.errors.iter().enumerate() {
        let error = error.expect("an estimate");
        assert!(error <= 1e-9 * v[k].abs(), "answer {k}'s estimate is {error}");
    }
}

/// **What has no answer says why** (#510 S3): a bar its supports leave free to slide (said once,
/// in the solver's words, as the scenario's word), a material that states no `nu`, a support
/// along a direction off the world's axes, a buckling load under loads that press nothing, a
/// factor of safety where an L's re-entrant edge leaves the stress unbounded — whose deflection
/// still answers — and a solid that is no prism.
#[test]
fn what_has_no_answer_in_three_dimensions_says_why() {
    let why = |src: &str| -> Vec<String> {
        let e = crate::common::text::solved(src);
        let (got, words) = answers(&e);
        got.into_iter().map(|g| g.3.err().unwrap_or_default()).chain(words).collect()
    };
    let has = |said: &[String], words: &str| {
        assert!(said.iter().any(|m| m.contains(words)), "{words}: {said:?}");
    };
    let loose = bar("  claim deflection(k) >= 0mm\n").replace("  pinned o along: std.z\n", "");
    let said = why(&loose);
    has(&said, "scenario `pull` leaves `plate` free to slide along z");
    has(&said, "`plate` is free to slide along z");
    let no_nu = bar("  claim deflection(k) >= 0mm\n").replace("nu: 0.3, ", "");
    has(&why(&no_nu), "states no `nu`: a reading in three dimensions reads Poisson's ratio");
    let tilted = bar("  claim deflection(k) >= 0mm\n")
        .replace("pinned d along: std.y", "pinned d along: tilted");
    has(&why(&tilted), "along a direction off the world's axes");
    has(&why(&bar("  claim buckling(plate) >= 1\n")), "the loads press nothing");
    // an L: its inner edge is three quarters of a turn of material
    let l = "\
unit mm
use std
steel := material(E: 200GPa, nu: 0.3, yield: 250MPa)
in std.front {
  a := point
  b := point
  c := point
  d := point
  f := point
  g := point
  fix((0mm, 0mm)) a
  fix((40mm, 0mm)) b
  fix((40mm, 10mm)) c
  fix((10mm, 10mm)) d
  fix((10mm, 40mm)) f
  fix((0mm, 40mm)) g
  outline := (foot := line(a, b)) -> (toe := line(b, c)) -> (step := line(c, d)) ->
             (riser := line(d, f)) -> (head := line(f, g)) -> (back := line(g, a)) -> close
}
ell := solid(outline, from: -5mm, to: 5mm) of steel
scenario hang {
  fixed ell.foot
  load g 100N along: std.x
  claim safety(ell) >= 1
  claim deflection(g, along: std.x) >= 0mm
}
";
    let e = crate::common::text::solved(l);
    let (got, words) = answers(&e);
    assert!(got[0].3.as_ref().unwrap_err().contains("round it"), "{got:?}");
    assert!(got[1].3.as_ref().is_ok_and(|&v| v > 0.0), "{got:?}");
    assert!(words.is_empty(), "the answer says it, not a word: {words:?}");
}
