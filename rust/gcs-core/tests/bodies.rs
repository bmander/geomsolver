//! **A solid read in three dimensions** (#510 S3): a solid a requirement names, and no plane
//! reading reads in its plane, is meshed in quadratic tetrahedra (`bodies::Mesh`) of the
//! interior's solid element (`interior::Tet10`), and its requirements are answered and judged
//! (`bodies::read`).  The corpus's solid cases answer their references, and what has no answer
//! says why.  The rules are `docs/contract/analysis.md`'s.
use gcs_core::diagnose::analysis::scenario_read;
use gcs_core::model::Output;

use crate::analysis_corpus::{read as read_case, Reading, CASES};
use crate::prisms::{answers, measured, value};

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
                  claim safety(plate) >= 1\n  claim deflection(q, along: std.x) >= 0mm\n";
    let (w, h, t, s, young, nu, rho, yields) =
        (80.0, 30.0, 10.0, 40e3, 200e6, 0.3, 7850.0 * 1e-9, 250e3);
    // its mass is the drawing's, claimed of it: no scenario's
    let e = crate::common::text::solved(&(bar(claims) + "claim mass(>= 1g) plate\n"));
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
    near(v[4], s * 50.0 / young, "the inner point's move");
    for (k, error) in read.errors.iter().enumerate() {
        let error = error.expect("an estimate");
        assert!(error <= 1e-9 * v[k].abs(), "answer {k}'s estimate is {error}");
    }
    let [(mass, Some(error), true)] = measured(&e)[..] else { panic!("one mass, kept") };
    near(mass, rho * w * h * t, "the mass");
    assert!(error <= 1e-9 * mass, "the mass's estimate is {error}");
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

/// **A requirement sizes a plank** (#510 S3): the library's plank (`sized_plank.sv`), drawn in
/// plan, built in at its root and loaded at its end across its plane, its width `w` an unknown,
/// driven so the end's corner sinks 0.9mm — a solid's driving row, its face's mesh moved with
/// the drawing and stacked through its thickness.  It solves to no freedom, its row holds what
/// it states, and the width found lies between Euler–Bernoulli's `4 P L³ / (E t³ sag)` (30mm, a
/// beam free to curl across its width) and a plate's, `(1 − ν²)` of it (27.3mm, held from
/// curling): three times as wide as it is thick, and held at its root, the plank is a little
/// stiffer than the beam.  Read afresh at the solved drawing, the claims' reading agrees with
/// the row to its discretisation, within its error estimated.
#[test]
fn a_requirement_sizes_a_plank() {
    let src = gcs_core::examples::source("sized_plank").unwrap();
    let mut e = crate::common::text::solved(src);
    assert_eq!(crate::common::dof(&mut e.sketch), 0);
    let w = e.sketch.params[*e.sketch.free_vars.get("w").expect("w") as usize].value;
    let beam = 4.0 * 50e3 * 300f64.powi(3) / (200e6 * 1000.0 * 0.9);
    let plate = (1.0 - 0.3 * 0.3) * beam;
    assert!(w > plate && w < beam, "{w} between {plate} and {beam}");
    let s = &e.scenarios[0];
    let (words, read) = scenario_read(&e.sketch, &s.scenario);
    assert!(words.is_empty(), "{words:?}");
    let said = gcs_core::report::scenario_answers(&e.sketch, s, &read);
    assert!(said[0].text.contains("driven == 0.9mm, held"), "{}", said[0].text);
    // its mass, claimed of the drawing, at the width found
    assert!(measured(&e).iter().all(|m| m.2), "its mass is kept");
    let fresh = read.answers[0].clone().unwrap().unwrap();
    let error = read.errors[0].expect("an estimate");
    assert!((fresh - 0.9).abs() <= error.max(1e-4), "{fresh} ± {error} against 0.9mm");
    eprintln!("w = {w}; afresh {fresh} ± {error}");
}


/// A bar `w` by `h` by `t` drawn in the front and swept from it, made of two blocks united — so
/// no prism, and filled by refining its exact boundary — held and pulled as `bar`'s is, its
/// far block's face named from the body (`bar.tail.right`).
const UNITED: &str = "unit mm
use std
w := 80mm
h := 30mm
t := 10mm
sigma := 40MPa
steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3, yield: 250MPa)
in std.front {
  o := point
  m := point
  b := point
  k := point
  n := point
  d := point
  fix((0mm, 0mm)) o
  fix((40mm, 0mm)) m
  fix((w, 0mm)) b
  fix((w, h)) k
  fix((40mm, h)) n
  fix((0mm, h)) d
  near := (low := line(o, m)) -> (seam := line(m, n)) -> (top := line(n, d)) ->
          (left := line(d, o)) -> close
  far := (low2 := line(m, b)) -> (right := line(b, k)) -> (top2 := line(k, n)) ->
         (seam2 := line(n, m)) -> close
}
bar := solid(near, from: 0mm, to: t) of steel
tail := solid(far, from: 0mm, to: t)
tail union bar
scenario pull {
  pinned bar.left along: std.x
  pinned o along: std.z
  pinned o along: std.y
  pinned d along: std.y
  pressure bar.tail.right -sigma
  claim deflection(k, along: std.x) >= 0mm
  claim deflection(k, along: std.z) <= 0mm
  claim compliance(bar) >= 0J
  claim safety(bar) >= 1
}
claim mass(>= 1g) bar
";

/// **A body that is no prism is filled and answers** (#510 S3, `bodies::Mesh::filled`): the bar
/// of two blocks united is meshed by refining its exact boundary (P6), its corners vertices of
/// the refinement, so the points held at them are nodes and its uniform stress is held exactly:
/// its far corner moves `σ w / E` along it and `−ν σ h / E` across, the pull does `σ² w h t /
/// E`, its factor of safety is the yield over `σ` and its mass `ρ w h t` — to 1e-6, the nodes'
/// places on its flat faces read to the refinement's bisection — and the reading twice as
/// coarse says the same.  A line naming the block united into it rather than the body is told
/// to name it from the body.
#[test]
fn a_body_of_two_blocks_answers_to_rounding() {
    let (w, h, t, s, young, nu, rho, yields) =
        (80.0, 30.0, 10.0, 40e3, 200e6, 0.3, 7850.0 * 1e-9, 250e3);
    let e = crate::common::text::solved(UNITED);
    let (words, read) = scenario_read(&e.sketch, &e.scenarios[0].scenario);
    assert!(words.is_empty(), "{words:?}");
    let v: Vec<f64> = read.answers.iter().map(|a| a.clone().unwrap().unwrap()).collect();
    let want = [s * w / young, -nu * s * h / young, s * s * w * h * t / young, yields / s];
    for (k, (&got, want)) in v.iter().zip(want).enumerate() {
        assert!((got - want).abs() <= 1e-6 * want.abs(), "answer {k} is {got}, not {want}");
        let error = read.errors[k].expect("an estimate");
        assert!(error <= 1e-6 * want.abs(), "answer {k}'s estimate is {error}");
    }
    // its mass, the drawing's, over its own tetrahedra
    let [(mass, Some(error), true)] = measured(&e)[..] else { panic!("one mass, kept") };
    let want = rho * w * h * t;
    assert!((mass - want).abs() <= 1e-6 * want && error <= 1e-6 * want, "{mass} ± {error}");
    let e = crate::common::text::solved(&UNITED.replace("bar.tail.right", "tail.right"));
    let (_, read) = scenario_read(&e.sketch, &e.scenarios[0].scenario);
    let said = read.answers[0].clone().unwrap().unwrap_err();
    assert!(said.contains("`tail` is a part of `bar`: a line about it names it from the body, \
                           `bar.tail.…`"), "{said}");
}

/// A rod of radius `r` and length `L`, turned about z from its half section in the front, its
/// base held along z and its centre's nearest node across, pulled by `−σ` on its head.
const ROD: &str = "unit mm
use std
r := 10mm
L := 100mm
sigma := 40MPa
steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3, yield: 250MPa)
in std.front {
  o := point
  a := point
  b := point
  c := point
  fix((0mm, 0mm)) o
  fix((r, 0mm)) a
  fix((r, L)) b
  fix((0mm, L)) c
  profile := (base := line(o, a)) -> (side := line(a, b)) -> (head := line(b, c)) ->
             (spine := line(c, o)) -> close
}
rod := solid(profile, about: std.z) of steel
scenario pull {
  pinned rod.base along: std.z
  pinned o along: std.x
  pinned o along: std.y
  pinned a along: std.y
  pressure rod.head -sigma
  claim deflection(c, along: std.z) >= 0mm
  claim safety(rod) >= 1
}
claim mass(>= 1g) rod
";

/// **A turned rod answers its closed forms** (#510 S3): a solid of revolution is no prism, so
/// its exact boundary is refined and filled, its cylinder's nodes on it.  Pulled along its axis,
/// its head moves `σ L / E` within 1e-3 — a point names the node nearest it, and the points
/// holding its base across are near the axis, not on it — and its mass is `ρ π r² L` within the
/// error estimated beside it.
#[test]
fn a_turned_rod_answers_its_closed_forms() {
    let (r, l, s, young, rho) = (10.0f64, 100.0, 40e3, 200e6, 7850.0 * 1e-9);
    let e = crate::common::text::solved(ROD);
    let (words, read) = scenario_read(&e.sketch, &e.scenarios[0].scenario);
    assert!(words.is_empty(), "{words:?}");
    let moved = read.answers[0].clone().unwrap().unwrap();
    assert!((moved - s * l / young).abs() <= 2e-3 * s * l / young, "{moved}");
    let [(mass, Some(error), true)] = measured(&e)[..] else { panic!("one mass, kept") };
    let want = rho * std::f64::consts::PI * r * r * l;
    assert!((mass - want).abs() <= error && error <= 1e-2 * want, "{mass} ± {error}: {want}");
}

/// The library's solids read in three dimensions (#510 S3), each a capability: a driving row
/// stacked through a plank, a load across a panel's plane, a solid's buckling, a plate's modes out
/// of its plane, a solid of revolution filled from its boundary with a round making its stress
/// answerable, and a body cut by another solid, its load named on the cutter's face from it.
const LIBRARY: [&str; 6] =
    ["sized_plank", "pressed_panel", "flat_bar_strut", "cantilever_plate", "turned_shaft",
     "pin_lug"];

/// **Every solid case of the library says what it claims** (#510 S3): solved, it leaves no
/// freedom; every requirement has an answer with its error estimated beside it, and keeps what it
/// states, a driven one held by its row; nothing is said against it.  The panel's sag and the bar's
/// buckling factor are their closed forms within 1% (Timoshenko's clamped plate, Euler's strut),
/// and the shaft's shoulder chamfered instead of rounded says its stress is unbounded and to round
/// it, while its mass still answers.
#[test]
fn every_solid_case_of_the_library_says_what_it_claims() {
    for key in LIBRARY {
        let src = gcs_core::examples::source(key).unwrap();
        let e = crate::common::read(src);
        let mut sk = crate::common::solved(&e);
        assert_eq!(crate::common::dof(&mut sk), 0, "{key}");
        for s in &e.scenarios {
            let (words, read) = scenario_read(&sk, &s.scenario);
            assert!(words.is_empty(), "{key}: {words:?}");
            let said = gcs_core::report::scenario_answers(&sk, s, &read);
            assert_eq!(said.len(), s.scenario.requirements.len(), "{key}: {said:?}");
            for (k, a) in said.iter().enumerate() {
                let end = if s.driven[k].is_some() { "held" } else { "kept" };
                assert!(a.text.ends_with(end), "{key}: {}", a.text);
                assert!(a.error.is_some(), "{key}: {}", a.text);
            }
        }
    }
    let value = |key: &str, output: Output| {
        value(&crate::common::text::solved(gcs_core::examples::source(key).unwrap()), output)
    };
    let d = 200e6 * 1000.0 / (12.0 * (1.0 - 0.09));
    let sag = 0.00126 * 0.1e3 * 400f64.powi(4) / d;
    let got = value("pressed_panel", Output::Deflection);
    assert!((got - sag).abs() <= 1e-2 * sag, "the panel sinks {got}, Timoshenko's {sag}");
    let euler = std::f64::consts::PI.powi(2) * 200e6 * 40.0 * 216.0 / 12.0 / (4.0 * 360000.0);
    let factor = euler / (2e3 * 40.0 * 6.0);
    let got = value("flat_bar_strut", Output::Buckling);
    assert!((got - factor).abs() <= 1e-2 * factor, "the bar buckles at {got}, Euler's {factor}");
    let chamfered = gcs_core::examples::source("turned_shaft").unwrap()
        .replace("  round := arc(center: c, start: g, end: f) hint(r: rho)\n",
                 "  round := line(f, g)\n");
    let e = crate::common::text::solved(&chamfered);
    let (got, _) = answers(&e);
    let safety = got.iter().find(|g| g.1 == Output::Safety).unwrap();
    assert!(safety.3.as_ref().unwrap_err().contains("round it"), "{safety:?}");
    assert!(measured(&e).iter().all(|m| m.2), "its mass, claimed of the drawing, is kept");
}

/// **A sliver lying on the boundary is peeled** (#510; `refine::volume`'s `peel`): the engine's
/// connecting rod filled at four times its reading's size held a cell of no thickness flat on a
/// planar face — two of its faces on the boundary, two corners protecting balls' centres, its
/// least dihedral angle 0°, its stiffness nearly singular (CG took 552 iterations and missed the
/// factor's moves by 2e-4).  Peeled, every element keeps a least dihedral angle of 3° or more, and
/// the quadratic elements fill the rod's exact volume within 2e-3, as at that size they did before
/// (the six cells peeled held 6e-5 of it).
#[test]
fn a_sliver_on_the_boundary_is_peeled() {
    use gcs_core::delaunay::tetra::{least_dihedral, Tet10};
    let e = crate::engine::open("engine/conrod.sv");
    let solid = fixtures::solid(&e, "rod.body");
    let mesh = gcs_core::bodies::mesh(&e.sketch, solid, &[], 4.0).expect("meshed");
    let least = mesh.tets.iter().map(|t| least_dihedral([0, 1, 2, 3].map(|k| mesh.nodes[t[k]])))
        .fold(180.0, f64::min);
    assert!(least >= 3.0, "an element's least dihedral angle is {least}°");
    let filled: f64 = mesh.tets.iter()
        .map(|t| Tet10 { nodes: t.map(|n| mesh.nodes[n]) }.volume()).sum();
    let x = e.sketch.exact_solid(solid).unwrap();
    let exact = gcs_core::brep::props::volume(&x.brep) / x.mm.powi(3);
    assert!((filled - exact).abs() <= 2e-3 * exact, "filled {filled}, exactly {exact}");
    eprintln!("{} elements, least dihedral {least:.2}°, volume {filled} of {exact}",
              mesh.tets.len());
}

/// **Stress is read where no sharp edge reaches** (#510; `analysis::reentrant_touching`): an L's
/// inner edge leaves its stress unbounded, so its factor of safety as a whole has no answer, nor
/// over a face the edge bounds (its riser) — but over a face clear of it (its head, its back,
/// the load spread over the head) the greatest stress converges, and is read; the same refusal
/// says which edge, and to round it.
#[test]
fn stress_is_read_where_no_sharp_edge_reaches() {
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
  load ell.head 100N along: std.x
  claim safety(ell) >= 1
  claim safety(ell.riser) >= 1
  claim safety(ell.head) >= 1
  claim safety(ell.back) >= 1
}
";
    let e = crate::common::text::solved(l);
    let (got, words) = answers(&e);
    assert!(words.is_empty(), "{words:?}");
    for k in [0, 1] {
        let why = got[k].3.as_ref().unwrap_err();
        assert!(why.contains("round it") && why.contains("riser") && why.contains("step"),
                "{why}");
    }
    let head = *got[2].3.as_ref().expect("the head's safety");
    let back = *got[3].3.as_ref().expect("the back's safety");
    assert!(head > 1.0 && back > 1.0 && head.is_finite() && back.is_finite(),
            "head {head}, back {back}");
}

/// **The connecting rod, stressed, says what it claims** (#510, the slow tier:
/// `conrod_stress.sv`, the engine's rod imported and loaded): it solves to no freedom, and every
/// requirement — its sinking under the firing load, and its factor of safety over three faces no
/// sharp edge reaches — has an answer with its error beside it, and keeps what it states.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, a minute: the rod read in three dimensions")]
fn the_connecting_rod_stressed_says_what_it_claims() {
    let e = crate::common::read(gcs_core::examples::source("conrod_stress").unwrap());
    let mut sk = crate::common::solved(&e);
    assert_eq!(crate::common::dof(&mut sk), 0);
    let s = &e.scenarios[0];
    let (words, read) = scenario_read(&sk, &s.scenario);
    assert!(words.is_empty(), "{words:?}");
    let said = gcs_core::report::scenario_answers(&sk, s, &read);
    assert_eq!(said.len(), 4, "{said:?}");
    for a in &said {
        assert!(a.text.ends_with("kept") && a.error.is_some(), "{}", a.text);
        eprintln!("{}", a.text);
    }
}
