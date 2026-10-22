//! **A prism read in its plane** (#510 S2): `plane stress plate` (or `plane strain`) in a
//! scenario reads the solid over the face it is swept from, meshed in quadratic triangles
//! (`delaunay::plane`) of the interior's plane element (`interior::Tri6`), and its requirements
//! are answered and judged (`prisms::read`).  The corpus's plane cases answer their references;
//! Lamé's cylinder, exact in plane stress with its ends open, answers in either reading; a sheet
//! pulled answers to rounding; and what has no answer says why.  The rules are
//! `docs/contract/analysis.md`'s.
use gcs_core::diagnose::analysis::scenario_read;
use gcs_core::model::{InPlane, Output};
use gcs_core::program::Elaborated;

use crate::analysis_corpus::{read as read_case, Reading, CASES};

/// Each requirement's answer under each scenario of `e`, as the report reads it (frames and
/// regions merged), with the scenario's words: `(scenario, output, count, answer)`.
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

/// The one value `e` answers of `output`.
fn value(e: &Elaborated, output: Output) -> f64 {
    let (got, words) = answers(e);
    let found: Vec<f64> = got.iter().filter(|g| g.1 == output).map(|g| {
        g.3.clone().unwrap_or_else(|m| panic!("{output:?} has no answer: {m}; {words:?}"))
    }).collect();
    found[0]
}

/// **Every plane case of the corpus answers its reference** (#510 S2), read in the plane its
/// document says: Kirsch's stress concentration and LE1's stress at D within 1% (stress is read
/// at nodes, the elements' averaged there), FV32's six frequencies within 0.1% — and keeps every
/// claim, with nothing said against it.
#[test]
fn every_plane_case_answers_its_reference() {
    for case in CASES.iter().filter(|c| c.reading == Reading::Plane) {
        let e = read_case(case);
        let want = (case.answers)(case, &e);
        let (got, words) = answers(&e);
        assert!(words.is_empty(), "{}: {words:?}", case.name);
        for (scenario, output, count, answer) in &got {
            let v = answer.clone().unwrap_or_else(|m| {
                panic!("{}: `{scenario}` {output:?} has no answer: {m}", case.name)
            });
            let a = want.iter().find(|a| (a.scenario, a.output, a.count)
                == (scenario.as_str(), *output, *count)).expect("a reference");
            let within = if *output == Output::Mode { 1e-3 } else { 1e-2 };
            assert!((v - a.reference).abs() <= within * a.reference.abs(),
                    "{}: `{scenario}` {output:?} is {v}, its reference {}", case.name, a.reference);
        }
        for s in &e.scenarios {
            let (_, read) = scenario_read(&e.sketch, &s.scenario);
            for (r, a) in s.scenario.requirements.iter().zip(&read.answers) {
                let v = *a.as_ref().unwrap().as_ref().unwrap();
                assert!(gcs_core::beams::keeps(r, v), "{}: {r:?} is {v}", case.name);
            }
        }
    }
}

/// The corpus's Lamé cylinder, its scenario reading it in the plane `plane`.
fn lame(plane: &str) -> Elaborated {
    let case = CASES.iter().find(|c| c.name == "lame").unwrap();
    let src = case.src.replace("scenario inflate {\n",
                               &format!("scenario inflate {{\n  plane {plane} tube\n"));
    crate::common::text::solved(&src)
}

/// **Lamé's cylinder in either reading** (#510 S2).  With its ends open the axial stress is
/// nothing and the solution is exact in plane stress: the corpus's document read so answers its
/// own claims, the bore's move within 1e-4 and its von Mises stress within 0.5%.  Read in plane
/// strain — its length held — the bore moves `(1 + ν) A a / E ((1 − 2ν) + c²/a²)` and the stress
/// reads `σz = ν(σr + σθ) = 2νA`.  Curved sides, a pressure on one: the arcs' nodes are on them.
#[test]
fn lames_cylinder_answers_in_either_reading() {
    let (a, c, p, young, nu, yields) = (50.0f64, 100.0f64, 50e3, 200e6, 0.3, 250e3);
    let big_a = p * a * a / (c * c - a * a);
    let stress = lame("stress");
    let moved = big_a * a / young * ((1.0 - nu) + (1.0 + nu) * c * c / (a * a));
    let mises = big_a * (1.0 + 3.0 * c.powi(4) / a.powi(4)).sqrt();
    let got = value(&stress, Output::Deflection);
    assert!((got - moved).abs() <= 1e-4 * moved, "plane stress: {got} against {moved}");
    let got = value(&stress, Output::Safety);
    assert!((got - yields / mises).abs() <= 5e-3 * yields / mises, "plane stress: {got}");

    let strain = lame("strain");
    let moved = (1.0 + nu) * big_a * a / young * ((1.0 - 2.0 * nu) + c * c / (a * a));
    let (r, t, z) = (-p, big_a * (1.0 + c * c / (a * a)), 2.0 * nu * big_a);
    let mises = (0.5 * ((r - t).powi(2) + (t - z).powi(2) + (z - r).powi(2))).sqrt();
    let got = value(&strain, Output::Deflection);
    assert!((got - moved).abs() <= 1e-4 * moved, "plane strain: {got} against {moved}");
    let got = value(&strain, Output::Safety);
    assert!((got - yields / mises).abs() <= 5e-3 * yields / mises, "plane strain: {got}");
}

/// A sheet `w` by `h` by `t` in the front, its left edge held along x and one corner along z,
/// pulled by `−σ` on its right edge, read in `plane`; its far corner `k` and a point `q` drawn
/// inside it, loaded by nothing.
fn sheet(plane: &str, extra: &str) -> String {
    format!("\
unit mm
use std
w := 80mm
h := 30mm
t := 2mm
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
plate := solid(outline, from: -t / 2, to: t / 2) of steel
scenario pull {{
  plane {plane} plate
  pinned plate.left along: std.x
  pinned o along: std.z
  pressure plate.right -sigma
{extra}}}
")
}

/// **A sheet pulled answers to rounding** (#510 S2): its stress is uniform, which the quadratic
/// element holds exactly, so its far corner moves `σ w / E'` (`E' = E` in plane stress, `E/(1 −
/// ν²)` in plane strain), the work the pull does is `σ² w h t / E'`, its factor of safety is the
/// yield over `σ` (`σ √(1 − ν + ν²)` in plane strain, `σz = νσ`), its mass `ρ w h t`, and a point
/// drawn inside it is a node of its mesh, moving as the field says.
#[test]
fn a_sheet_pulled_answers_to_rounding() {
    let claims = "  claim deflection(k, along: std.x) >= 0mm\n  claim compliance(plate) >= 0J\n  \
                  claim safety(plate) >= 1\n  claim mass(plate) >= 1g\n  \
                  claim deflection(q, along: std.x) >= 0mm\n";
    let (w, h, t, s, young, nu, rho, yields) =
        (80.0, 30.0, 2.0, 40e3, 200e6, 0.3, 7850.0 * 1e-9, 250e3);
    for plane in ["stress", "strain"] {
        let e = crate::common::text::solved(&sheet(plane, claims));
        let strain = plane == "strain";
        let e1 = if strain { young / (1.0 - nu * nu) } else { young };
        let (got, words) = answers(&e);
        assert!(words.is_empty(), "{words:?}");
        let v: Vec<f64> = got.iter().map(|g| g.3.clone().unwrap()).collect();
        let near = |got: f64, want: f64, what: &str| {
            assert!((got - want).abs() <= 1e-9 * want.abs(), "{plane}: {what} is {got}, not {want}")
        };
        near(v[0], s * w / e1, "the corner's move");
        near(v[1], s * s * w * h * t / e1, "the pull's work");
        let mises = if strain { s * (1.0 - nu + nu * nu).sqrt() } else { s };
        near(v[2], yields / mises, "the factor of safety");
        near(v[3], rho * w * h * t, "the mass");
        near(v[4], s * 50.0 / e1, "the inner point's move");
    }
}

/// **What has no answer says why** (#510 S2): a sheet its supports leave free to slide (said
/// once, in the solver's words, as the scenario's word), a material that states no `nu`, a
/// buckling load (a plate buckles out of its plane), a load across the plane, and a factor of
/// safety where an L's re-entrant corner leaves the stress unbounded — whose deflection still
/// answers.
#[test]
fn what_has_no_answer_in_the_plane_says_why() {
    let why = |src: &str| -> Vec<String> {
        let e = crate::common::text::solved(src);
        let (got, words) = answers(&e);
        got.into_iter().map(|g| g.3.err().unwrap_or_default()).chain(words).collect()
    };
    let has = |said: &[String], words: &str| {
        assert!(said.iter().any(|m| m.contains(words)), "{words}: {said:?}");
    };
    let loose = sheet("stress", "  claim deflection(k) >= 0mm\n")
        .replace("  pinned o along: std.z\n", "");
    let said = why(&loose);
    has(&said, "scenario `pull` leaves `plate` free to slide along z");
    has(&said, "`plate` is free to slide along z");
    let no_nu = sheet("stress", "  claim deflection(k) >= 0mm\n").replace("nu: 0.3, ", "");
    has(&why(&no_nu), "states no `nu`: a reading in the plane reads Poisson's ratio");
    has(&why(&sheet("stress", "  claim buckling(plate) >= 1\n")), "buckles out of its plane");
    has(&why(&sheet("stress", "  load q (0N, 5N, 0N)\n  claim deflection(k) >= 0mm\n")),
        "across the plane");
    // an L: its inner corner is three quarters of a turn of material
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
ell := solid(outline, from: -1mm, to: 1mm) of steel
scenario hang {
  plane stress ell
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

/// **The reading is a line of the scenario** (#510 S2): `plane stress B` and `plane strain B`
/// print as written, are tinted where they stand, carry into a scenario extending this one, and
/// a scenario's own replaces its base's.  Refused where written: a target that is no solid or no
/// prism swept square to its face (**E040**), a solid read twice in one scenario (**E001**), and
/// a word other than `stress` or `strain` (a syntax error).
#[test]
fn the_reading_is_a_line_of_the_scenario() {
    use gcs_core::syntax::{highlight, parse, Tint};
    let body = "scenario a {\n  plane stress plate\n  fixed plate.left\n}\n";
    let (p, errs) = parse(body);
    assert!(errs.is_empty());
    let mut out = String::new();
    gcs_core::syntax::write_stmt_to(&mut out, &p.root().body[0].kind).unwrap();
    assert_eq!(format!("{out}\n"), body);
    let tint = |word: &str| {
        let lo = body.find(word).expect("written") as u32;
        highlight(body).into_iter().find(|(_, s)| s.lo == lo).map(|(t, _)| t)
    };
    assert_eq!(tint("plane"), Some(Tint::Word));
    assert_eq!(tint("stress"), Some(Tint::Word));

    let base = sheet("stress", "  claim deflection(k) >= 0mm\n");
    let e = crate::common::text::solved(&format!(
        "{base}scenario more: pull {{\n  claim deflection(k) >= 0mm\n}}\n\
         scenario other: pull {{\n  plane strain plate\n  claim deflection(k) >= 0mm\n}}\n"));
    let plane = |name: &str| e.scenarios.iter().find(|s| s.scenario.name == name).unwrap()
        .scenario.readings.iter().map(|r| r.plane).collect::<Vec<_>>();
    assert_eq!(plane("more"), [InPlane::Stress]);
    assert_eq!(plane("other"), [InPlane::Strain]);

    let errors = |src: &str| {
        let (p, errs, _) = gcs_core::library::parse_linked(src);
        let mut out: Vec<(String, String)> =
            errs.into_iter().map(|e| ("syntax".to_string(), e.message)).collect();
        let e = gcs_core::program::elaborate(&p);
        out.extend(e.errors().map(|d| (d.code.as_str().to_string(), d.message.clone())));
        out
    };
    let has = |got: &[(String, String)], code: &str, words: &str| {
        assert!(got.iter().any(|(c, m)| c == code && m.contains(words)), "{code} {words}: {got:?}");
    };
    let with = |line: &str| errors(&base.replace("  plane stress plate\n", line));
    has(&with("  plane stress q\n"), "E040", "is no solid");
    has(&with("  plane stress plate\n  plane strain plate\n"), "E001", "read in the plane already");
    has(&with("  plane stretch plate\n"), "syntax", "`plane stress` or `plane strain`");
    let turned = base.replace("plate := solid(outline, from: -t / 2, to: t / 2) of steel",
                              "plate := solid(outline, about: low, sweep: 90deg) of steel");
    has(&errors(&turned), "E040", "is a prism");
}

/// **A requirement sizes a plate** (#510 S2): the library's plate (`sized_plate.sv`), built in
/// along its root and loaded along its end, read in plane stress, finds the depth at which its
/// end's lower corner sinks `sag` — within 0.5% of Timoshenko's beam with its shear counted,
/// `P L³/3EI + P L/(κ G A) = sag` (κ = 5/6), for each sag asked.  It leaves no freedom: the depth
/// is the row's.  The row holds what it drives, and the report says so; a fresh mesh of the solved
/// plate — the claims' reading, where the row reads the mesh it was compiled on, moved — reads
/// the same sag to its discretisation, 1e-3; and the claim beside it is kept.
#[test]
fn a_requirement_sizes_a_plate() {
    let src = gcs_core::examples::source("sized_plate").unwrap();
    let (p, l, young, t, nu): (f64, f64, f64, f64, f64) = (4e6, 300.0, 200e6, 10.0, 0.3);
    let g = young / (2.0 * (1.0 + nu));
    for sag in [0.5, 0.35, 0.8] {
        let text = src.replace("sag := 0.5mm", &format!("sag := {sag}mm"));
        let e = crate::common::read(&text);
        let mut sk = crate::common::solved(&e);
        assert_eq!(crate::common::dof(&mut sk), 0, "sag {sag}");
        let h = sk.params[sk.free_vars["h"] as usize].value;
        let sinks = |h: f64| {
            p * l.powi(3) / (3.0 * young * t * h.powi(3) / 12.0) + p * l / (5.0 / 6.0 * g * t * h)
        };
        let (mut lo, mut hi) = (10.0, 300.0);
        for _ in 0..200 {
            let m = 0.5 * (lo + hi);
            if sinks(m) > sag { lo = m } else { hi = m }
        }
        assert!((h - lo).abs() <= 5e-3 * lo, "sag {sag}: depth {h}, Timoshenko's {lo}");
        let s = &e.scenarios[0];
        let (_, read) = scenario_read(&sk, &s.scenario);
        let said = gcs_core::report::scenario_answers(&sk, s, &read);
        assert!(said[0].text.ends_with("held"), "sag {sag}: {}", said[0].text);
        assert_eq!(said[0].value.map(|v| (v - sag).abs() <= 1e-9 * sag), Some(true), "{}",
                   said[0].text);
        let fresh = *read.answers[0].as_ref().unwrap().as_ref().unwrap();
        assert!((fresh - sag).abs() <= 1e-3 * sag, "sag {sag}: a fresh mesh reads {fresh}");
        assert!(said[1].text.ends_with("kept"), "{}", said[1].text);
    }
}

/// **A deflection over a region that cannot drive is refused where it is written** (#510 S2,
/// **E040**): a region its supports leave free, and one bounded by a spline, which the row does not
/// move yet.  A deflection driven of a point of no region read in the plane compiles nothing,
/// and the report says nothing drives it.
#[test]
fn a_regions_driving_row_is_refused_where_it_cannot_be_read() {
    let errors = |src: &str| {
        let (p, errs, _) = gcs_core::library::parse_linked(src);
        assert!(errs.is_empty(), "{errs:?}");
        let e = gcs_core::program::elaborate(&p);
        e.errors().map(|d| (d.code.as_str().to_string(), d.message.clone())).collect::<Vec<_>>()
    };
    let has = |got: &[(String, String)], words: &str| {
        assert!(got.iter().any(|(c, m)| c == "E040" && m.contains(words)), "{words}: {got:?}");
    };
    let plate = gcs_core::examples::source("sized_plate").unwrap();
    has(&errors(&plate.replace("  fixed plate.root\n", "  pinned plate.root along: std.z\n")),
        "free");
    let case = CASES.iter().find(|c| c.name == "nafems_le1").unwrap();
    let driven = case.src.replace("  claim safety(membrane) >= 0.97 * le1.yield / target\n",
                                  "  deflection(a, along: std.z) == 0.1mm\n");
    has(&errors(&driven), "spline or a curve");
    // read as a solid, the plate's deflection drives nothing
    let solid = plate.replace("  plane stress plate\n", "");
    let e = crate::common::text::solved(&solid.replace("param h: Length hint(78mm)", "h := 78mm"));
    use gcs_core::constraints::CKind;
    assert!(e.sketch.constraints.iter().all(|c| c.kind != CKind::Deflection));
}
