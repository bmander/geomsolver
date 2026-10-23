//! **What a body is made of, driving** (#510): a scenario's `mass(B) == m` or `volume(B) == v`
//! is a row of the drawing (`CKind::Mass`, `CKind::Volume`) — the interior's kernel reading the
//! structure's measure (`interior::Reads`) rather than its moves, so it solves nothing — over a
//! beam's or a member's lines (`beams::Row::measured`) or a prism's face meshed once and moved
//! with the drawing (`prisms::Row::measured`).  Each is held to its closed form; what cannot
//! drive says why.  The rules are `docs/contract/analysis.md`'s; the rows' finite-difference
//! checks are `jacobians.rs`'s (`a_measure_agrees_with_finite_differences`,
//! `a_measure_agrees_with_finite_differences_as_the_drawing_states_it`).
use gcs_core::constraints::CKind;
use gcs_core::model::Sketch;
use gcs_core::program::{elaborate, Elaborated};

const RHO: f64 = 7850.0;

/// A beam along x in the front from `foot` to `end`, its section's numbers `section`, read under
/// `scenario`'s lines; `extra` declared before it.
fn beam(extra: &str, end: &str, section: &str, scenario: &str) -> String {
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

/// The section of a `b`-wide rectangle `h` deep.
fn rectangle(b: &str, h: &str) -> String {
    format!("A: {b} * {h}, Iy: {h} * {b}^3 / 12, Iz: {b} * {h}^3 / 12, J: 0.229 * {h} * {b}^3")
}

/// A plate `L` by `h` in the front, `t` thick, with a hole of radius `r` (an unknown) at its
/// middle, read under `scenario`'s lines.
fn holed(scenario: &str) -> String {
    format!("\
unit mm
use std
alloy := material(E: 70GPa, nu: 0.33, rho: 2700kg / 1m^3)
param r: Length hint(18mm)
in std.front {{
  a := point
  b := point
  c := point
  d := point
  o := point
  fix((0mm, 0mm)) a
  fix((160mm, 0mm)) b
  fix((160mm, 60mm)) c
  fix((0mm, 60mm)) d
  fix((80mm, 30mm)) o
  low := line(a, b)
  end := line(b, c)
  top := line(c, d)
  root := line(d, a)
  hole := radius(r) circle(center: o) hint(r: 18)
  plate := face(low, end, top, root, holes: hole)
}}
web := solid(plate, from: -2.5mm, to: 2.5mm) of alloy
scenario s {{
{scenario}
}}
")
}

/// `src` elaborated with no error and solved.
fn sized(src: &str) -> (Elaborated, Sketch) {
    let e = crate::common::read(src);
    let sk = crate::common::solved(&e);
    (e, sk)
}

/// What the unknown `name` came to.
fn unknown(sk: &Sketch, name: &str) -> f64 {
    sk.params[sk.free_vars[name] as usize].value
}

/// What the report says of scenario `s`'s requirements, each its sentence.
fn said(e: &Elaborated, sk: &Sketch) -> Vec<String> {
    let s = &e.scenarios[0];
    let (_, read) = gcs_core::diagnose::analysis::scenario_read(sk, &s.scenario);
    gcs_core::report::scenario_answers(sk, s, &read).into_iter().map(|a| a.text).collect()
}

/// **A mass sizes a section**: a beam's depth an unknown its area reads, its mass driven, is
/// `m / (ρ b L)` to the solver's precision, for each mass asked — under no support and no load,
/// its frame free, so no solve is read and nothing is said against it.  The row is one `Mass`; it
/// leaves no freedom; the report says it is driven and held, and judges a claim of the volume
/// beside it at the depth found.
#[test]
fn a_mass_sizes_a_section() {
    for m in [10.0, 2.5, 40.0] {
        let src = beam("param h: Length hint(40mm)", "fix((1000mm, 0mm)) end",
                       &rectangle("25mm", "h"),
                       &format!("  mass(arm) == {m}kg\n  claim volume(arm) >= 1 * 1cm^3"));
        let (e, mut sk) = sized(&src);
        let rows: Vec<_> = sk.constraints.iter().filter(|c| c.kind == CKind::Mass).collect();
        assert_eq!(rows.len(), 1);
        let want = m / (RHO * 0.025 * 1.0) * 1e3;
        let h = unknown(&sk, "h");
        assert!((h - want).abs() <= 1e-9 * want, "mass {m}: {h} against {want}");
        assert_eq!(crate::common::dof(&mut sk), 0);
        let said = said(&e, &sk);
        assert_eq!(said.len(), 2, "{said:?}");
        assert!(said[0].contains("driven == ") && said[0].ends_with("held"), "{}", said[0]);
        let volume = 25.0 * want * 1000.0;
        assert!(said[1].contains(&format!("is {}mm³", gcs_core::json::fmt_g(volume, 6)))
                && said[1].ends_with("kept"), "{}", said[1]);
        let (words, _) = gcs_core::diagnose::analysis::scenario_read(&sk, &e.scenarios[0].scenario);
        assert!(words.is_empty(), "{words:?}");
    }
}

/// **The library's weighed cantilever holds what it drives** (`weighed_cantilever.sv`): its width
/// and depth found together by its sag and its mass, each its closed form to 1e-9; no freedom
/// left; both rows held, the deflection the claims' reading of the solved frame reads to
/// rounding, and the claim beside them kept.
#[test]
fn the_librarys_weighed_cantilever_holds_what_it_drives() {
    let (e, mut sk) = sized(gcs_core::examples::source("weighed_cantilever").unwrap());
    let area = 10.0 / (RHO * 1.0);
    let h = (4.0 * 1e3 / (200e9 * 4e-3) / area).sqrt();
    assert!((unknown(&sk, "h") * 1e-3 - h).abs() <= 1e-9 * h, "{}", unknown(&sk, "h"));
    assert!((unknown(&sk, "b") * 1e-3 - area / h).abs() <= 1e-9 * area / h);
    assert_eq!(crate::common::dof(&mut sk), 0);
    let s = &e.scenarios[0];
    assert_eq!(s.driven.iter().flatten().count(), 2);
    let read = gcs_core::beams::read(&sk, &s.scenario);
    let sag = *read.answers[0].as_ref().unwrap().as_ref().unwrap();
    assert!((sag - 4.0).abs() <= 1e-9 * 4.0, "{sag}");
    let said = said(&e, &sk);
    assert!(said[0].ends_with("held") && said[1].ends_with("held"), "{said:?}");
    assert!(said[2].ends_with("kept"), "{}", said[2]);
}

/// **A volume places a node**: a beam's length an unknown its line is dimensioned by, its volume
/// driven, is `V / A` — its end's lift a column of the row.
#[test]
fn a_volume_places_a_node() {
    let src = beam("param len: Length hint(800mm)",
                   "end := point hint((800, 0))\n  foot horizontal end\n  foot distance(len) end",
                   &rectangle("25mm", "40mm"), "  volume(arm) == 1200 * 1cm^3");
    let (_, mut sk) = sized(&src.replace("  end := point\n", ""));
    let len = unknown(&sk, "len");
    assert!((len - 1200.0).abs() <= 1e-9 * 1200.0, "{len}");
    assert_eq!(crate::common::dof(&mut sk), 0);
}

/// **A mass and a sag size a section together**: a cantilever's width and depth both unknowns,
/// its tip's deflection and its mass driven — `b h = m / (ρ L)` and `b h³ = 4 P L³ / (E δ)`, so
/// `h = √((4 P L³ / (E δ)) / (b h))` — each to 1e-9.  Two rows, two unknowns, no freedom.
#[test]
fn a_mass_and_a_sag_size_a_section_together() {
    let src = beam("param b: Length hint(20mm)\nparam h: Length hint(40mm)",
                   "fix((1000mm, 0mm)) end", &rectangle("b", "h"),
                   "  fixed foot\n  load end 1kN along: down\n  \
                    deflection(end, along: down) == 4mm\n  mass(arm) == 10kg");
    let (e, mut sk) = sized(&src);
    let area = 10.0 / (RHO * 1.0);
    let bh3 = 4.0 * 1e3 / (200e9 * 4e-3);
    let h = (bh3 / area).sqrt();
    let (got_b, got_h) = (unknown(&sk, "b") * 1e-3, unknown(&sk, "h") * 1e-3);
    assert!((got_h - h).abs() <= 1e-9 * h, "{got_h} against {h}");
    assert!((got_b - area / h).abs() <= 1e-9 * area / h, "{got_b} against {}", area / h);
    assert_eq!(crate::common::dof(&mut sk), 0);
    assert_eq!(e.scenarios[0].driven.iter().filter(|d| d.is_some()).count(), 2);
}

/// **A volume sizes a hole**: a plate's hole an unknown circle, its volume driven over the
/// plate's face meshed once and moved with the drawing, its radius `√((L h − V / t) / π)` within
/// what the mesh's quadratic sides leave of the circle — read in its plane, and read in three
/// dimensions (no reading line), one row either way; the row holds what it states, and a claim of
/// the mass beside it is judged.
#[test]
fn a_volume_sizes_a_hole() {
    let want = ((160.0 * 60.0 - 42000.0 / 5.0) / std::f64::consts::PI).sqrt();
    for reading in ["  plane stress web\n", ""] {
        let src = holed(&format!("{reading}  volume(web) == 42 * 1cm^3\n  \
                                  claim mass(web) <= 1kg"));
        let (e, mut sk) = sized(&src);
        let rows: Vec<_> = sk.constraints.iter().filter(|c| c.kind == CKind::Volume).collect();
        assert_eq!(rows.len(), 1);
        let r = unknown(&sk, "r");
        assert!((r - want).abs() <= 1e-5 * want, "{reading:?}: {r} against {want}");
        assert_eq!(crate::common::dof(&mut sk), 0);
        let said = said(&e, &sk);
        assert!(said[0].contains("is 42000mm³") && said[0].ends_with("held"), "{}", said[0]);
        assert!(said[1].ends_with("kept"), "{}", said[1]);
    }
}

/// **A mass reads its material's density**: a beam made of a material that states no `rho` has
/// no mass to drive (**E040** where it is written), and a volume of it does; a solid that is no
/// prism — turned — compiles nothing, and the report says nothing drives it yet.
#[test]
fn what_cannot_be_measured_says_why() {
    let src = beam("param h: Length hint(40mm)", "fix((1000mm, 0mm)) end",
                   &rectangle("25mm", "h"), "  mass(arm) == 10kg")
        .replace(", rho: 7850kg / 1m^3", "");
    let (p, errs, _) = gcs_core::library::parse_linked(&src);
    assert!(errs.is_empty(), "{errs:?}");
    let errors: Vec<(String, String)> = elaborate(&p).errors()
        .map(|d| (d.code.as_str().to_string(), d.message.clone())).collect();
    assert!(errors.iter().any(|(c, m)| c == "E040" && m.contains("`mass(arm)` cannot drive")
                                      && m.contains("states no `rho`")), "{errors:?}");
    let src = src.replace("mass(arm) == 10kg", "volume(arm) == 1000 * 1cm^3");
    let (_, sk) = sized(&src);
    assert!((unknown(&sk, "h") - 40.0).abs() <= 1e-9 * 40.0);
    let turned = "\
unit mm
use std
steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3)
in std.front {
  a := point
  b := point
  c := point
  d := point
  fix((0mm, 0mm)) a
  fix((10mm, 0mm)) b
  fix((10mm, 40mm)) c
  fix((0mm, 40mm)) d
  low := line(a, b)
  side := line(b, c)
  top := line(c, d)
  spine := line(d, a)
  sec := face(low, side, top, spine)
}
rod := solid(sec, about: spine) of steel
scenario s {
  mass(rod) == 1kg
}
";
    let e = crate::common::read(turned);
    assert!(e.sketch.constraints.iter().all(|c| !c.kind.interior()));
    let sk = crate::common::solved(&e);
    let said = said(&e, &sk);
    assert!(said.iter().any(|s| s.contains("nothing drives it yet")), "{said:?}");
}

/// **The row is the source's alone**: the document's JSON holds no `Mass` and no `Volume`.
#[test]
fn a_measures_row_is_the_sources_alone() {
    let src = beam("param h: Length hint(40mm)", "fix((1000mm, 0mm)) end",
                   &rectangle("25mm", "h"), "  mass(arm) == 10kg");
    let e = crate::common::read(&src);
    assert!(e.sketch.constraints.iter().any(|c| c.kind == CKind::Mass));
    let json = gcs_core::io::dumps(&e.sketch, None);
    assert!(!json.contains("\"Mass\"") && !json.contains("\"Volume\""));
    let lifted = gcs_core::program::to_program(&e.sketch);
    assert!(!lifted.text().contains("mass"), "{}", lifted.text());
}
