//! **Frames, as claims** (#510 S0): `arm := beam(span, A: …, Iy: …, Iz: …, J: …) of steel`
//! makes lines of the drawing members of a frame, and a scenario's requirements on it are
//! answered by the interior's beam (`interior::Beam`, `beams::read`) and judged — never solved
//! for.  The corpus's frame cases answer their references, exact where a cubic is; a frame
//! joins where its members meet; and what has no answer says why.  The rules are
//! `docs/contract/analysis.md`'s.
use gcs_core::beams;
use gcs_core::model::{Applied, Output, SectionProp};
use gcs_core::program::{elaborate, Elaborated};
use gcs_core::syntax::{parse, StmtKind};

use crate::analysis_corpus::{read as read_case, Reading, CASES};

/// Each requirement's answer under each scenario of `e`, in the order written:
/// `(scenario, output, count, answer)`.
fn answers(e: &Elaborated) -> Vec<(String, Output, Option<u32>, Option<Result<f64, String>>)> {
    let mut out = Vec::new();
    for s in &e.scenarios {
        let read = beams::read(&e.sketch, &s.scenario);
        for (r, a) in s.scenario.requirements.iter().zip(read.answers) {
            out.push((s.scenario.name.clone(), r.output, r.count, a));
        }
    }
    out
}

/// **Every frame case of the corpus answers its reference**, read off its solved drawing as the
/// corpus reads it (`Case::answers`), and keeps every claim it states: to rounding where the
/// cubic is the beam's exact deflection — a load at a joint, a load spread evenly, the stress
/// they give, a mass — and within 1e-5 where it approximates, a mode and a buckling load, the
/// member cut into `beams::PIECES` between its joints.
#[test]
fn every_frame_case_answers_its_reference() {
    for case in CASES.iter().filter(|c| c.reading == Reading::Frame) {
        let e = read_case(case);
        let want = (case.answers)(case, &e);
        let got = answers(&e);
        for (scenario, output, count, answer) in &got {
            let v = answer.clone().expect("about a frame").unwrap_or_else(|m| {
                panic!("{}: `{scenario}` {output:?} has no answer: {m}", case.name)
            });
            let a = want.iter().find(|a| (a.scenario, a.output, a.count)
                == (scenario.as_str(), *output, *count)).expect("a reference");
            let cubic = matches!(output, Output::Mode | Output::Buckling);
            let within = if cubic { 1e-5 } else { 1e-9 };
            assert!((v - a.reference).abs() <= within * a.reference.abs(),
                    "{}: `{scenario}` {output:?} is {v}, its reference {}", case.name, a.reference);
        }
        for s in &e.scenarios {
            let read = beams::read(&e.sketch, &s.scenario);
            for (r, a) in s.scenario.requirements.iter().zip(&read.answers) {
                let v = *a.as_ref().unwrap().as_ref().unwrap();
                assert!(beams::keeps(r, v), "{}: {r:?} is {v}", case.name);
            }
            assert!(read.words.is_empty(), "{}: {:?}", case.name, read.words);
        }
    }
}

/// An L of two beams, a post built in at its foot and an arm off its head, loaded at the arm's
/// end: the arm bends as a cantilever, `P L³ / 3EI`; the post bends under the moment `P L` and
/// turns its head by `P L H / EI`, which swings the arm's end down `P L² H / EI`; and the post
/// shortens `P H / EA`.  To rounding: the frame joins at the corner, rigid, and a cubic is exact.
/// A tee joins too: a prop whose end stands on a span's middle holds the span there.
#[test]
fn a_frame_joins_where_its_members_meet() {
    let src = "\
unit mm
use std
H := 600mm
L := 400mm
P := 2kN
steel := material(E: 210GPa, nu: 0.3)
in std.front {
  foot := point
  corner := point
  end := point
  fix((0mm, 0mm)) foot
  fix((0mm, H)) corner
  fix((L, H)) end
  post := line(foot, corner)
  reach := line(corner, end)
}
down := axis
fix(dir == (0, 0, -1), origin == (0, 0, 0)) down
frame := beam(post, reach, A: 400 * 1mm^2, Iy: 13333 * 1mm^4, Iz: 13333 * 1mm^4,
              J: 22500 * 1mm^4) of steel
scenario hung {
  fixed foot
  load end P along: down
  claim deflection(end, along: down) >= 0mm
}
";
    let e = crate::common::text::solved(src);
    let [(_, _, _, Some(Ok(v)))] = &answers(&e)[..] else { panic!("{:?}", answers(&e)) };
    let (h, l, p): (f64, f64, f64) = (600.0, 400.0, 2e6);
    let (young, i, a) = (210e3 * 1e3, 13333.0, 400.0);
    let want =
        p * l.powi(3) / (3.0 * young * i) + p * l * l * h / (young * i) + p * h / (young * a);
    assert!((v - want).abs() <= 1e-9 * want, "{v} against {want}");

    // the tee: a span free at both ends, propped at its middle by a post built in at its foot;
    // loaded evenly about the prop, it is two cantilevers off the prop's head
    let tee = "\
unit mm
use std
steel := material(E: 200GPa, nu: 0.3)
in std.front {
  a := point
  c := point
  m := point
  base := point
  fix((0mm, 100mm)) a
  fix((200mm, 100mm)) c
  fix((100mm, 100mm)) m
  fix((100mm, 0mm)) base
  span := line(a, c)
  prop := line(base, m)
}
down := axis
fix(dir == (0, 0, -1), origin == (0, 0, 0)) down
tee := beam(span, prop, A: 100 * 1mm^2, Iy: 833 * 1mm^4, Iz: 833 * 1mm^4,
            J: 1400 * 1mm^4) of steel
scenario held {
  fixed base
  load a 10N along: down
  load c 10N along: down
  claim deflection(a, along: down) >= 0mm
  claim deflection(c, along: down) >= 0mm
}
";
    let e = crate::common::text::solved(tee);
    let got = answers(&e);
    let ends: Vec<f64> = got.iter().map(|(_, _, _, a)| *a.as_ref().unwrap().as_ref().unwrap())
        .collect();
    // each end a cantilever of 100 off a head the symmetric loads neither turn nor sway
    let (young, i, a, p) = (200e3 * 1e3, 833.0, 100.0, 10e3);
    let want = p * 100f64.powi(3) / (3.0 * young * i) + 2.0 * p * 100.0 / (young * a);
    for v in ends {
        assert!((v - want).abs() <= 1e-9 * want, "{v} against {want}");
    }
}

/// A cantilever loaded at a point drawn on its span, not at a joint: the scenario cuts the
/// member there, and its end sinks `P a² (3L − a) / 6EI`, the point `P a³ / 3EI`.
#[test]
fn a_point_on_a_member_cuts_it() {
    let src = "\
unit mm
use std
steel := material(E: 200GPa, nu: 0.3)
in std.front {
  foot := point
  end := point
  at := point
  fix((0mm, 0mm)) foot
  fix((300mm, 0mm)) end
  fix((200mm, 0mm)) at
  span := line(foot, end)
}
down := axis
fix(dir == (0, 0, -1), origin == (0, 0, 0)) down
arm := beam(span, A: 100 * 1mm^2, Iy: 833 * 1mm^4, Iz: 833 * 1mm^4,
            J: 1400 * 1mm^4) of steel
scenario s {
  fixed foot
  load at 10N along: down
  claim deflection(end, along: down) >= 0mm
  claim deflection(at, along: down) >= 0mm
  claim deflection(arm, along: down) >= 0mm
}
";
    let e = crate::common::text::solved(src);
    let got: Vec<f64> =
        answers(&e).iter().map(|(_, _, _, a)| *a.as_ref().unwrap().as_ref().unwrap()).collect();
    let (ei, p, a, l): (f64, f64, f64, f64) = (200e6 * 833.0, 10e3, 200.0, 300.0);
    let end = p * a * a * (3.0 * l - a) / (6.0 * ei);
    let at = p * a.powi(3) / (3.0 * ei);
    for (v, want) in got.iter().zip([end, at, end]) {
        assert!((v - want).abs() <= 1e-9 * want, "{got:?} against {end}, {at}");
    }
}

/// **What has no answer says why**: a frame its supports leave free is named, in the solver's
/// words, once; a material that does not state what is read of it, a section without its
/// moduli for a factor of safety, a load that presses nothing for a buckling load.  A
/// requirement on a solid is no frame's, and has no answer of this kind.
#[test]
fn what_has_no_answer_says_why() {
    let doc = |material: &str, section: &str, scenario: &str| format!("\
unit mm
use std
steel := material({material})
in std.front {{
  foot := point
  end := point
  fix((0mm, 0mm)) foot
  fix((300mm, 0mm)) end
  span := line(foot, end)
}}
down := axis
fix(dir == (0, 0, -1), origin == (0, 0, 0)) down
arm := beam(span, A: 100 * 1mm^2, Iy: 833 * 1mm^4, Iz: 833 * 1mm^4,
            J: 1400 * 1mm^4{section}) of steel
scenario s {{
{scenario}
}}
");
    let said = |src: String| {
        let e = crate::common::text::solved(&src);
        let read = beams::read(&e.sketch, &e.scenarios[0].scenario);
        let why: Vec<String> = read.answers.into_iter().flatten().filter_map(|a| a.err()).collect();
        (read.words, why)
    };
    let full = "E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3, yield: 250MPa";
    let (words, why) = said(doc(full, "", "  pinned foot\n  load end 1N along: down\n  \
                                           claim deflection(end) <= 1mm"));
    assert_eq!(words, ["scenario `s` leaves `arm` free to turn about x through (0, 0, 0), turn \
                        about y through (0, 0, 0) and turn about z through (0, 0, 0)"]);
    assert!(why[0].starts_with("`arm` is free to turn about x"), "{why:?}");
    let (_, why) = said(doc("E: 200GPa", "", "  fixed foot\n  claim deflection(end) <= 1mm"));
    assert!(why[0].contains("`steel`, which states no `nu`"), "{why:?}");
    let (_, why) = said(doc("E: 200GPa, nu: 0.3", "", "  fixed foot\n  claim mode(arm, 1) >= 1Hz"));
    assert!(why[0].contains("states no `rho`"), "{why:?}");
    let (_, why) = said(doc(full, "", "  fixed foot\n  load end 1N along: down\n  \
                                       claim safety(arm) >= 1"));
    assert!(why[0].contains("`Sy` and `Sz`, and it states neither"), "{why:?}");
    let (_, why) = said(doc(full, ", Sy: 1mm^3, Sz: 1mm^3",
                            "  fixed foot\n  load end 1N along: down\n  claim buckling(arm) >= 1"));
    assert!(why[0].contains("press no member"), "{why:?}");
    // pulled, it does not buckle; pressed, it does
    let (_, why) = said(doc(full, "", "  fixed foot\n  load end (-1N, 0N, 0N)\n  \
                                       claim buckling(arm) >= 1"));
    assert!(why.is_empty(), "{why:?}");
}

/// A beam is its lines, its section and its material — model values in the document's units —
/// and adds nothing to the drawing: the sketch is the one the document elaborates to without it.
#[test]
fn a_beam_is_its_lines_section_and_material() {
    let base = "\
unit cm
use std
steel := material(E: 200GPa, nu: 0.3)
w := 2cm
in std.front {
  a := point
  b := point
  fix((0cm, 0cm)) a
  fix((10cm, 0cm)) b
  span := line(a, b)
}
";
    let with = format!("{base}arm := beam(span, A: w^2, Iy: 1cm^4, Iz: 2 * 1cm^4, \
                        J: 3 * 1cm^4, Sz: 5 * 1cm^3) of steel\n");
    let e = crate::common::read(&with);
    assert!(e.ok(), "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    let [b] = &e.sketch.beams[..] else { panic!() };
    assert_eq!((b.name.as_str(), b.lines.as_slice(), b.material.as_deref()),
               ("arm", &[crate::common::ent(&e, "span").i()][..], Some("steel")));
    let got: Vec<(SectionProp, f64)> = b.section.props.iter().map(|(p, v)| (*p, *v)).collect();
    use SectionProp::*;
    assert_eq!(got, [(A, 4.0), (Iy, 1.0), (Iz, 2.0), (J, 3.0), (Sz, 5.0)]);
    let without = crate::common::read(base);
    assert_eq!(gcs_core::io::dumps(&e.sketch, None), gcs_core::io::dumps(&without.sketch, None));
    // a scenario names it as the beam it is
    let e = crate::common::read(&format!("{with}scenario s {{\n  fixed a\n  \
                                         claim mode(arm, 1) >= 1Hz\n}}\n"));
    assert_eq!(e.scenarios[0].scenario.requirements[0].of, Applied::Beam(0));
    assert_eq!(e.scenarios[0].heads, ["mode(arm, 1)"]);
}

#[test]
fn a_beam_prints_as_it_is_written() {
    let src = "arm := beam(span, post, A: b * h, Iy: 1mm^4, Iz: 2 * 1mm^4, J: 3 * 1mm^4) of steel\n\
               rod := beam(l, A: 1mm^2, Iy: 1mm^4, Iz: 1mm^4, J: 2 * 1mm^4)\n";
    let (p, errs) = parse(src);
    assert!(errs.is_empty(), "{errs:?}");
    let mut out = String::new();
    for st in &p.root().body {
        assert!(matches!(st.kind, StmtKind::Beam(_)));
        gcs_core::syntax::write_stmt_to(&mut out, &st.kind).expect("printable");
        out.push('\n');
    }
    assert_eq!(out, src);
}

/// What a beam refuses, each where it is written; and a scenario's lines about one.
#[test]
fn what_a_beam_refuses() {
    let base = "\
unit mm
use std
steel := material(E: 200GPa, nu: 0.3)
param u: Length hint(5mm)
in std.front {
  a := point
  b := point
  fix((0mm, 0mm)) a
  fix((10mm, 0mm)) b
  span := line(a, b)
}
far := point hint((0, 0, 10))
up := line(a, far)
";
    let errors = |text: &str| {
        let (p, errs, _) = gcs_core::library::parse_linked(&format!("{base}{text}\n"));
        let mut out: Vec<(String, String)> =
            errs.into_iter().map(|e| ("syntax".to_string(), e.message)).collect();
        let e = elaborate(&p);
        out.extend(e.errors().map(|d| (d.code.as_str().to_string(), d.message.clone())));
        out
    };
    let ok = "A: 1mm^2, Iy: 1mm^4, Iz: 1mm^4, J: 1mm^4";
    let has = |text: String, code: &str, words: &str| {
        let got = errors(&text);
        assert!(got.iter().any(|(c, m)| c == code && m.contains(words)), "{code} {words}: {got:?}");
    };
    has(format!("x := beam(a, {ok}) of steel"), "E040", "drawn on lines, and `a` is none");
    has(format!("x := beam(up, {ok}) of steel"), "E040", "stands in space");
    has(format!("x := beam(nowhere, {ok}) of steel"), "E101", "nowhere");
    has(format!("x := beam(span, {ok}, K: 1mm^4) of steel"), "E103", "no number of a beam's");
    has(format!("x := beam(span, {ok}, A: 2 * 1mm^2) of steel"), "E001", "given twice");
    has("x := beam(span, A: 1mm^2, Iy: 1mm^4, Iz: 1mm^4) of steel".into(), "E103",
        "leaves out `J`");
    has("x := beam(span, A: 1mm, Iy: 1mm^4, Iz: 1mm^4, J: 1mm^4) of steel".into(), "E103",
        "`A` is Length^2");
    has("x := beam(span, A: -1mm^2, Iy: 1mm^4, Iz: 1mm^4, J: 1mm^4) of steel".into(), "E103",
        "positive");
    has("x := beam(span, A: u * 1mm, Iy: 1mm^4, Iz: 1mm^4, J: 1mm^4) of steel".into(), "E040",
        "reads an unknown");
    has(format!("x := beam(span, {ok}) of span"), "E103", "`span` is no material");
    has(format!("x := beam({ok}) of steel"), "syntax", "drawn on lines");
    has(format!("x := beam(span, {ok}, b) of steel"), "syntax", "drawn on lines, then");
    // a scenario about a beam
    let beam = format!("x := beam(span, {ok}) of steel\n");
    has(format!("{beam}scenario s {{\n  fixed x\n}}"), "E040", "hold its ends");
    has(format!("{beam}scenario s {{\n  pressure x 1MPa\n}}"), "E040", "is a beam");
}
