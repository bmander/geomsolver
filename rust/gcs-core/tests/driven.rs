//! **Frames, driving** (#510 S1): a scenario's `deflection(p, along: d) == v` at a frame's joint
//! is a row of the drawing (`CKind::Deflection`, `beams::Row`) — the interior's kernel over the
//! frame, its value the frame's solve and its rate the adjoint's — so the solve sizes a section
//! read over an unknown, or places a node, to make it hold.  Each is held to its closed form; and
//! what cannot drive says why.  The rules are `docs/contract/analysis.md`'s; the row's
//! finite-difference check is `jacobians.rs`'s (`a_frames_deflection_agrees_with_finite_differences`).
use gcs_core::beams;
use gcs_core::constraints::CKind;
use gcs_core::model::Sketch;
use gcs_core::program::{elaborate, Elaborated};

const E: f64 = 200e9;
const RHO: f64 = 7850.0;

/// A cantilever along x in the front, built in at `foot`, its section's numbers `section`, read
/// under `scenario`'s lines; `extra` declared before it.
fn cantilever(extra: &str, end: &str, section: &str, scenario: &str) -> String {
    format!("\
unit mm
use std (horizontal)
steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3, yield: 250MPa)
{extra}
in std.front {{
  foot := point
  end := point
  fix((0mm, 0mm)) foot
  {end}
  span := line(foot, end)
}}
down := axis
fix(dir == (0, 0, -1), origin == (0, 0, 0)) down
arm := beam(span, {section}) of steel
scenario s {{
{scenario}
}}
")
}

/// The section of a `b`-wide rectangle `h` deep, either a name or a number.
fn rectangle(b: &str, h: &str) -> String {
    format!("A: {b} * {h}, Iy: {h} * {b}^3 / 12, Iz: {b} * {h}^3 / 12, J: 0.229 * {h} * {b}^3, \
             Sy: {h} * {b}^2 / 6, Sz: {b} * {h}^2 / 6")
}

/// `src` elaborated with no error and solved: the drawing, and what an unknown came to.
fn sized(src: &str) -> (Elaborated, Sketch) {
    let e = crate::common::read(src);
    let sk = crate::common::solved(&e);
    (e, sk)
}

/// What the unknown `name` came to.
fn unknown(sk: &Sketch, name: &str) -> f64 {
    sk.params[sk.free_vars[name] as usize].value
}

/// **A requirement sizes a section**: the library's cantilever (`sized_cantilever.sv`) finds the
/// depth at which its tip sinks `sag`, Euler–Bernoulli's `(4 P L³ / (E b sag))^(1/3)`, to the
/// solver's precision, for each sag asked; its tip's deflection read after the solve is what it
/// states; the row is one `Deflection`, and the report says it is driven and held.  It leaves no
/// freedom: the depth is the row's.
#[test]
fn a_requirement_sizes_a_section() {
    let src = gcs_core::examples::source("sized_cantilever").unwrap();
    for sag in [4.0, 1.5, 9.0] {
        let text = src.replace("sag := 4mm", &format!("sag := {sag}mm"));
        let (e, mut sk) = sized(&text);
        let rows: Vec<_> = sk.constraints.iter().filter(|c| c.kind == CKind::Deflection).collect();
        assert_eq!(rows.len(), 1);
        let (p, l, b) = (1e3, 1.0, 0.025);
        let want = (4.0 * p * l * l * l / (E * b * sag * 1e-3)).cbrt() * 1e3;
        let h = unknown(&sk, "h");
        assert!((h - want).abs() <= 1e-9 * want, "sag {sag}: {h} against {want}");
        let s = &e.scenarios[0];
        assert_eq!(s.driven.iter().filter(|d| d.is_some()).count(), 1);
        let read = beams::read(&sk, &s.scenario);
        let got = *read.answers[0].as_ref().unwrap().as_ref().unwrap();
        assert!((got - sag).abs() <= 1e-9 * sag, "{got}");
        let said = gcs_core::report::scenario_answers(&sk, s, &read);
        assert!(said[0].text.contains("driven == ") && said[0].text.ends_with("held"),
                "{}", said[0].text);
        // the claim beside it is judged at the depth found: the root's stress P L / (b h² / 6)
        let safety = 250e6 / (p * l / (b * (h * 1e-3).powi(2) / 6.0));
        let kept = *read.answers[1].as_ref().unwrap().as_ref().unwrap();
        assert!((kept - safety).abs() <= 1e-6 * safety, "{kept} against {safety}");
        assert_eq!(crate::common::dof(&mut sk), 0);
    }
}

/// **A requirement places a node**: the cantilever's length an unknown its line is dimensioned
/// by, the tip's deflection driving it — `L = (3 E I sag / P)^(1/3)`, the lifts of the joints the
/// row's columns.
#[test]
fn a_requirement_places_a_node() {
    let src = cantilever("param len: Length hint(800mm)", "end := point hint((800, 0))\n  \
                          foot horizontal end\n  foot distance(len) end",
                         &rectangle("25mm", "50mm"),
                         "  fixed foot\n  load end 1kN along: down\n  \
                          deflection(end, along: down) == 4mm");
    let (_, sk) = sized(&src.replace("  end := point\n", ""));
    let i = 0.025 * 0.05f64.powi(3) / 12.0;
    let want = (3.0 * E * i * 4e-3 / 1e3).cbrt() * 1e3;
    let len = unknown(&sk, "len");
    assert!((len - want).abs() <= 1e-9 * want, "{len} against {want}");
}

/// **A spread load moves with what it is spread along** (`interior::Spread`, its `∂f/∂x`): a
/// cantilever under its own weight sinks `ρ A g L⁴ / 8EI`, so the depth it sinks `δ` at is
/// `√(1.5 ρ g L⁴ / (E δ))`, its load growing with the section it sizes; under a force spread
/// evenly along it, `P L³ / 8EI`, the depth `(1.5 P L³ / (E b δ))^(1/3)`, and the length it
/// sinks `δ` at, the load a length shrinking as it grows, `(8 E I δ / P)^(1/3)`.
#[test]
fn a_spread_load_moves_with_what_it_is_spread_along() {
    let (b, l) = (0.025f64, 1.0f64);
    let at = "fix((1000mm, 0mm)) end";
    let weight = cantilever("param h: Length hint(60mm)", at, &rectangle("25mm", "h"),
                            "  fixed foot\n  accelerate arm (0, 0, -9.81 * 1m / 1s^2)\n  \
                             deflection(end, along: down) == 0.1mm");
    let (_, sk) = sized(&weight);
    let want = (1.5 * RHO * 9.81 * l.powi(4) / (E * 1e-4)).sqrt() * 1e3;
    let h = unknown(&sk, "h");
    assert!((h - want).abs() <= 1e-9 * want, "its weight: {h} against {want}");

    let spread = cantilever("param h: Length hint(40mm)", at, &rectangle("25mm", "h"),
                            "  fixed foot\n  load arm 1kN along: down\n  \
                             deflection(end, along: down) == 4mm");
    let (_, sk) = sized(&spread);
    let want = (1.5 * 1e3 * l.powi(3) / (E * b * 4e-3)).cbrt() * 1e3;
    let h = unknown(&sk, "h");
    assert!((h - want).abs() <= 1e-9 * want, "spread: {h} against {want}");

    let long = cantilever("param len: Length hint(1500mm)", "end := point hint((1500, 0))\n  \
                           foot horizontal end\n  foot distance(len) end",
                          &rectangle("25mm", "50mm"),
                          "  fixed foot\n  load arm 1kN along: down\n  \
                           deflection(end, along: down) == 4mm");
    let (_, sk) = sized(&long.replace("  end := point\n", ""));
    let i = 0.025 * 0.05f64.powi(3) / 12.0;
    let want = (8.0 * E * i * 4e-3 / 1e3).cbrt() * 1e3;
    let len = unknown(&sk, "len");
    assert!((len - want).abs() <= 1e-9 * want, "its length: {len} against {want}");
}

/// **A frame of two members, driven**: an L, a post built in at its foot and an arm off its
/// head, both one section `h` deep, its arm's end sinking `P L³ / 3EI + P L² H / EI` (the
/// post's turn swings it) and `P H / EA` (the post shortens under it), `I = b h³ / 12` and `A = b
/// h`: `h` the root of that, found here by Newton on the closed form.
#[test]
fn a_frame_of_two_members_is_driven() {
    let src = "\
unit mm
use std
H := 600mm
L := 400mm
steel := material(E: 200GPa, nu: 0.3)
param h: Length hint(30mm)
in std.front {
  foot := point
  corner := point
  end := point
  fix((0mm, 0mm)) foot
  fix((0mm, H)) corner
  fix((L, H)) end
  post := line(foot, corner)
  arm := line(corner, end)
}
down := axis
fix(dir == (0, 0, -1), origin == (0, 0, 0)) down
frame := beam(post, arm, A: 20mm * h, Iy: h * (20mm)^3 / 12, Iz: 20mm * h^3 / 12,
              J: 0.2 * h * (20mm)^3) of steel
scenario s {
  fixed foot
  load end 2kN along: down
  deflection(end, along: down) == 3mm
}
";
    let (_, sk) = sized(src);
    let (p, post, l, b) = (2e3, 0.6, 0.4, 0.02);
    let sink = |h: f64| p * (l * l * l / 3.0 + l * l * post) / (E * b * h.powi(3) / 12.0)
        + p * post / (E * b * h) - 3e-3;
    let mut want = 0.06f64;
    for _ in 0..60 {
        let d = (sink(want * (1.0 + 1e-7)) - sink(want)) / (want * 1e-7);
        want -= sink(want) / d;
    }
    let want = want * 1e3;
    let h = unknown(&sk, "h");
    assert!((h - want).abs() <= 1e-9 * want, "{h} against {want}");
}

/// **The row is the source's alone**: the document's JSON holds no `Deflection` and the drawing
/// lifted to a flat program states none, since neither holds the beams and the scenario it reads.
#[test]
fn the_row_is_the_sources_alone() {
    let e = crate::common::read(gcs_core::examples::source("sized_cantilever").unwrap());
    assert!(e.sketch.constraints.iter().any(|c| c.kind == CKind::Deflection));
    assert!(!gcs_core::io::dumps(&e.sketch, None).contains("Deflection"));
    let lifted = gcs_core::program::to_program(&e.sketch);
    assert!(!lifted.text().contains("deflection"), "{}", lifted.text());
}

/// **What cannot drive says why**, where it is written: a section over an unknown nothing sizes,
/// over an angle, over one with no seed; a driving deflection on a frame its supports leave free,
/// or along a direction the solve moves.  A driving requirement a frame does not drive yet (a
/// mode) compiles nothing and is judged: nothing drives it.
#[test]
fn what_cannot_drive_says_why() {
    let errors = |src: &str| -> Vec<(String, String)> {
        let (p, errs, _) = gcs_core::library::parse_linked(src);
        assert!(errs.is_empty(), "{errs:?}");
        let e = elaborate(&p);
        e.errors().map(|d| (d.code.as_str().to_string(), d.message.clone())).collect()
    };
    let has = |src: String, code: &str, words: &str| {
        let got = errors(&src);
        assert!(got.iter().any(|(c, m)| c == code && m.contains(words)), "{code} {words}: {got:?}");
    };
    let at = "fix((1000mm, 0mm)) end";
    let sizes = rectangle("25mm", "h");
    let drives = "  fixed foot\n  load end 1kN along: down\n  deflection(end, along: down) == 4mm";
    has(cantilever("param h: Length hint(40mm)", at, &sizes,
                   "  fixed foot\n  claim deflection(end, along: down) <= 4mm"),
        "E040", "`h` is an unknown nothing determines");
    has(cantilever("param h: Angle hint(40deg)", at, &sizes, drives), "E040",
        "read over lengths and plain numbers");
    has(cantilever("param h: Length", at, &sizes, drives), "E040", "has no seed");
    has(cantilever("param h: Length hint(40mm)", at, &sizes,
                   "  pinned foot\n  load end 1kN along: down\n  \
                    deflection(end, along: down) == 4mm"),
        "E040", "free to turn");
    has(cantilever("param h: Length hint(40mm)\nin std.front {\n  q := point hint((0, 5))\n  \
                    r := point hint((3, 1))\n  slope := line(q, r)\n}", at, &sizes,
                   "  fixed foot\n  load end 1kN along: down\n  \
                    deflection(end, along: slope) == 4mm"),
        "E040", "moves in the solve");
    // a mode driven is judged, not solved for
    let mode = cantilever("", at, &rectangle("25mm", "50mm"), "  fixed foot\n  mode(arm, 1) == 30Hz");
    let e = crate::common::text::solved(&mode);
    assert!(e.sketch.constraints.iter().all(|c| c.kind != CKind::Deflection));
    let read = beams::read(&e.sketch, &e.scenarios[0].scenario);
    let said = gcs_core::report::scenario_answers(&e.sketch, &e.scenarios[0], &read);
    assert!(said[0].text.contains("nothing drives it yet"), "{}", said[0].text);
}
