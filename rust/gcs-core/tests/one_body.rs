//! **Both faces are one body's** (#961, [0.223]): an edge, a corner and the material's angle are
//! between two faces of one body, so a round or a chamfer between a face of one solid and a face
//! of another it is not part of is refused by name — E103 at the document's statement, naming
//! the body both are faces of where one is made of both — and never reads one solid's face list
//! by the other's face number: that panicked where the number was past the list, and rounded some
//! other face where it was in range.  Below the language, the field refuses faces of two solids
//! read as one body's (`solid::faces::one_body`), so a model handed to the solver alone does too.
use gcs_core::model::{FaceRef, RegionShape, SolidDef};
use gcs_core::program::{self, Elaborated};

use crate::common::lenient::read;

/// `src` read, whatever it says, and solved: a refused round leaves the solids it names drawn.
fn solved(src: &str) -> Elaborated {
    let (mut e, _) = read(src);
    let r = gcs_core::solve::solve(&mut e.sketch, gcs_core::solve::SolveOpts::default());
    assert!(r.success, "{}\n{src}", r.message);
    e
}

/// A plate 60 × 60 × 5 (`plate`, six faces) and a bar's outline 40 × 20 over it, each side named.
const PLATE: &str = "unit mm\nuse std\nuse std.solids (union)\nuse std.rounds (round, chamfer)\n\
    in std.top {\n\
      a := point\nb := point\nc := point\nd := point\n\
      fix((-30, -30)) a\nfix((30, -30)) b\nfix((30, 30)) c\nfix((-30, 30)) d\n\
      (ab := line(a, b)) -> (bc := line(b, c)) -> (cd := line(c, d)) -> (da := line(d, a)) -> close\n\
      sq := face(ab, bc, cd, da)\n\
      e := point\nf := point\ng := point\nh := point\n\
      fix((-20, -10)) e\nfix((20, -10)) f\nfix((20, 10)) g\nfix((-20, 10)) h\n\
      (ef := line(e, f)) -> (fg := line(f, g)) -> (gh := line(g, h)) -> (he := line(h, e)) -> close\n\
      bar := face(ef, fg, gh, he)\n\
    }\n\
    plate := solid(std.solids.Prism(sq, from: 0mm, to: 5mm))\n";

/// The issue's document: an octagonal plate (ten faces) and a bar standing clear of it (six), no
/// `union` joining them.
const ISSUE: &str = "unit mm\nuse std\nuse std.solids\nuse std.rounds (round)\nin std.top {\n\
    p0 := point\nfix((50.0, 0.0)) p0\np1 := point\nfix((35.355339, 35.355339)) p1\n\
    p2 := point\nfix((0.0, 50.0)) p2\np3 := point\nfix((-35.355339, 35.355339)) p3\n\
    p4 := point\nfix((-50.0, 0.0)) p4\np5 := point\nfix((-35.355339, -35.355339)) p5\n\
    p6 := point\nfix((-0.0, -50.0)) p6\np7 := point\nfix((35.355339, -35.355339)) p7\n\
    oct := face(p0, p1, p2, p3, p4, p5, p6, p7, -> close)\n\
    a := point\nb := point\nc := point\nd := point\n\
    fix((-20, -10)) a\nfix((20, -10)) b\nfix((20, 10)) c\nfix((-20, 10)) d\n\
    bar := face(a, b, c, d, -> close)\n}\n\
    plate := solid(std.solids.Prism(oct, from: 0mm, to: 5mm))\n\
    wall := solid(std.solids.Prism(bar, from: 20mm, to: 25mm))\n";

/// Every diagnostic `src` reads, and the elaboration.
fn said(src: &str) -> (Elaborated, Vec<String>) {
    read(src)
}

/// The E103s refusing faces of two solids, each with the line it is said on.
fn refused(src: &str) -> Vec<(usize, String)> {
    let (e, d) = said(src);
    let mut out = Vec::new();
    for diag in &e.diags {
        if diag.message.contains("are faces of two solids") {
            assert_eq!(diag.code.as_str(), "E103", "{d:?}");
            let line = 1 + src[..diag.span.lo as usize].matches('\n').count();
            out.push((line, diag.message.clone()));
        }
    }
    out
}

/// Each solid's volume as the report gives it (built where the drawing solves).
fn volume(src: &str, solid: &str) -> f64 {
    let e = solved(src);
    let key = format!("{solid}.volume");
    gcs_core::report::positions(&e.sketch, &e.map).into_iter()
        .find(|(k, _)| *k == key).map(|(_, v)| v)
        .unwrap_or_else(|| panic!("no {key}"))
}

/// The fill a ball of radius `r` leaves in a square concave edge `l` long.
fn fill(r: f64, l: f64) -> f64 {
    r * r * (1.0 - std::f64::consts::PI / 4.0) * l
}

/// The issue's document, either way round: refused once, at the round, naming both solids in the
/// order written and saying no body is made of both; nothing of the round is made, so every solid
/// builds (as `solventc --solid` asks either) as drawn, and nothing panics.
#[test]
fn a_round_between_faces_of_two_solids_is_refused_by_name() {
    let ways = [("wall.far round(r: 2mm) plate.near", "wall.far", "plate.near"),
                ("plate.near round(r: 2mm) wall.far", "plate.near", "wall.far")];
    for (round, first, second) in ways {
        let src = format!("{ISSUE}{round}\n");
        let line = src.lines().count();
        let got = refused(&src);
        assert_eq!(got.len(), 1, "{round}: {got:?}");
        let (at, m) = &got[0];
        assert_eq!(*at, line, "{round}: said at the round: {m}");
        let (a, b) = (first.split('.').next().unwrap(), second.split('.').next().unwrap());
        assert!(m.starts_with(&format!("`{first}` and `{second}` are faces of two solids, and \
                                        `{a}` and `{b}` are not one body")), "{m}");
        assert!(m.contains("no body is made of both"), "{m}");
        let e = solved(&src);
        assert!(!e.sketch.solids.iter().any(|s| matches!(s.def, SolidDef::Region { .. })),
                "{round}: the round makes nothing");
        // what `solventc` builds: the solid diagnostics read each solid, and neither refuses
        let built = program::solid_diagnostics(&e.sketch, &e.map, &e.program);
        assert!(built.is_empty(), "{round}: {built:?}");
        for s in ["plate", "wall"] {
            let i = e.sketch.solid_called(s).unwrap();
            gcs_core::solid::refusal(&e.sketch, i).unwrap_or_else(|m| panic!("{s}: {m}"));
        }
    }
}

/// The silent case: two solids of six faces each, so either one's face numbers are in range of
/// the other's list — through 0.222 the round read the bar's faces off the plate without a word.
/// Refused alike, and both solids are as drawn.
#[test]
fn faces_of_two_solids_whose_numbers_are_in_range_are_refused_too() {
    let src = format!("{PLATE}wall := solid(std.solids.Prism(bar, from: 20mm, to: 25mm))\n\
                       wall.far round(r: 2mm) plate.near\n");
    let got = refused(&src);
    assert_eq!(got.len(), 1, "{got:?}");
    assert!(got[0].1.contains("`wall` and `plate` are not one body"), "{}", got[0].1);
    assert!((volume(&src, "wall") - 40.0 * 20.0 * 5.0).abs() < 1e-6);
    assert!((volume(&src, "plate") - 60.0 * 60.0 * 5.0).abs() < 1e-6);
}

/// The fixture's case: the wall unioned into `part`, whose stock is the plate, and the round
/// naming the wall's and the plate's own faces.  The refusal names the part and the faces through
/// it, and that spelling rounds the edge where the wall stands on the plate: the fill of a
/// concave edge 40 long.
#[test]
fn the_refusal_names_the_body_both_are_faces_of() {
    let parts = format!("{PLATE}wall := solid(std.solids.Prism(bar, from: 5mm, to: 25mm))\n\
                         part := solid(plate)\nwall union part\n");
    let got = refused(&format!("{parts}wall.ef round(r: 2mm) plate.near\n"));
    assert_eq!(got.len(), 1, "{got:?}");
    let m = &got[0].1;
    assert!(m.contains("`wall` and `plate` are not one body"), "{m}");
    assert!(m.ends_with("both are faces of `part`: name them through it, `part.wall.ef` and \
                         `part.plate.near`"), "{m}");
    // a face of the part and one of an operand it holds: the part names both
    let ways = [("part.wall.ef round(r: 2mm) plate.near", "`part.wall.ef` and `part.plate.near`"),
                ("plate.near round(r: 2mm) part.wall.ef", "`part.plate.near` and `part.wall.ef`")];
    for (round, names) in ways {
        let got = refused(&format!("{parts}{round}\n"));
        assert_eq!(got.len(), 1, "{round}: {got:?}");
        assert!(got[0].1.ends_with(names), "{round}: {}", got[0].1);
    }
    // the spelling it gives builds the round
    let meant = format!("{parts}part.wall.ef round(r: 2mm) part.plate.near\n");
    assert!(refused(&meant).is_empty());
    let want = 60.0 * 60.0 * 5.0 + 40.0 * 20.0 * 20.0 + fill(2.0, 40.0);
    let got = volume(&meant, "part");
    assert!((got - want).abs() < 1e-6 * want, "{got} against {want}");
}

/// Whole solids, every pair of their faces: a round and a chamfer, each said once at its own
/// statement, however many pairs it asks of.
#[test]
fn a_word_over_two_whole_solids_is_said_once() {
    let src = format!("{PLATE}wall := solid(std.solids.Prism(bar, from: 20mm, to: 25mm))\n\
                       wall round(r: 1mm) plate\nwall chamfer(d: 1mm) plate\n");
    let n = src.lines().count();
    let got = refused(&src);
    let lines: Vec<usize> = got.iter().map(|(l, _)| *l).collect();
    assert_eq!(lines, [n - 1, n], "{got:?}");
}

/// `angle(a, b, at: p)` stated over faces of two solids: E040 where it is written.
#[test]
fn the_material_angle_between_faces_of_two_solids_is_refused() {
    let src = format!("{PLATE}wall := solid(std.solids.Prism(bar, from: 5mm, to: 25mm))\n\
                       s := solid({{ p | p in wall.ef.side; p in plate.near.side\n\
                       angle(wall.ef, plate.near, at: p) <= 180deg }})\n");
    let (_, d) = said(&src);
    let angle: Vec<&String> = d.iter().filter(|m| m.contains("material's angle")).collect();
    assert_eq!(angle.len(), 1, "{d:?}");
    assert!(angle[0].starts_with("E040: `wall.ef` and `plate.near` are faces of two solids"),
            "{}", angle[0]);
}

/// Below the language: a round of the part's own faces, its regions' faces then moved onto two
/// solids as a model handed to the solver might carry them — the angle's faces each of a solid of
/// its own, in both orders, one face number past the other's list and both in range.  The field
/// refuses each by name; none panics, none is built.
#[test]
fn the_field_refuses_an_angle_between_faces_of_two_solids() {
    let src = format!("{PLATE}wall := solid(std.solids.Prism(bar, from: 5mm, to: 25mm))\n\
                       part := solid(plate)\nwall union part\n\
                       part.wall.ef round(r: 2mm) part.plate.near\n");
    let e = solved(&src);
    let called = |n: &str| e.sketch.solid_called(n).unwrap() as u32;
    let (part, wall, plate) = (called("part"), called("wall"), called("plate"));
    let face = |solid: u32, path: &[&str]| FaceRef {
        solid, path: path.iter().map(|s| s.to_string()).collect(), surface: false, smooth: false,
    };
    let cases = [
        // the wall's six faces read with the part's numbers (eleven): past its list
        (face(wall, &["ef"]), face(part, &["plate", "near"])),
        (face(part, &["plate", "near"]), face(wall, &["ef"])),
        // six and six: every number in range
        (face(wall, &["ef"]), face(plate, &["near"])),
        (face(plate, &["near"]), face(wall, &["ef"])),
    ];
    let angled: Vec<usize> = (0..e.sketch.solids.len()).filter(|&i| {
        matches!(&e.sketch.solids[i].def, SolidDef::Region { terms, .. }
                 if terms.iter().any(|t| matches!(t.shape, RegionShape::Angle { .. })))
    }).collect();
    assert!(!angled.is_empty(), "the round's regions read the material's angle");
    for (fa, fb) in cases {
        let mut sk = e.sketch.clone();
        for &i in &angled {
            let SolidDef::Region { terms, .. } = &mut sk.solids[i].def else { unreachable!() };
            for t in terms.iter_mut() {
                if let RegionShape::Angle { a, b } = &mut t.shape {
                    (*a, *b) = (fa.clone(), fb.clone());
                }
            }
        }
        for &i in &angled {
            let m = gcs_core::solid::refusal(&sk, i).expect_err("faces of two solids");
            assert!(m.contains("are not one body"), "{fa:?} {fb:?}: {m}");
        }
    }
}
