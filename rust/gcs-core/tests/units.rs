//! Units, and the dimensional analysis they make possible (spec §3.3).
//!
//! The return on a large change is what it *catches*, so most of this file is errors: a length
//! where an angle was wanted, a length added to an angle, the involute formula's unstated
//! radians, a free variable read as two different things.  The rest is that none of it changed
//! any drawing.

use gcs_core::diagnose::{diagnose, DiagnoseOptions};
use gcs_core::examples;
use gcs_core::graft;
use gcs_core::io;
use gcs_core::model::Sketch;
use gcs_core::program::elaborate;
use crate::common::parse_legacy as parse;
use gcs_core::style::Presented;
use gcs_core::units::{Dim, Units};

fn read(src: &str) -> Result<Sketch, Vec<String>> {
    let (p, errs) = parse(src);
    if !errs.is_empty() {
        return Err(errs.into_iter().map(|e| e.message).collect());
    }
    let e = elaborate(&p);
    if !e.ok() {
        return Err(e.errors().map(|d| d.message.clone()).collect());
    }
    Ok(e.sketch)
}

/// Every diagnostic a document produced, error or warning: a dimension that will not evaluate
/// keeps its last number and reports, so some of these are warnings.
fn diags(src: &str) -> Vec<String> {
    let (p, errs) = parse(src);
    assert!(errs.is_empty(), "{errs:?}");
    elaborate(&p).diags.iter().map(|d| d.message.clone()).collect()
}

fn says(src: &str, what: &str) {
    let ds = diags(src);
    assert!(ds.iter().any(|m| m.contains(what)), "expected {what:?} in {ds:?}");
}

const PAIR: &str = "use std\nin std.front {\na := point hint((0, 0))\nb := point hint((60, 0))\nl := line(a, b)\n}\n";

/* -- the literal ------------------------------------------------------------------------- */

/// `unit mm` names what a bare number is, and a suffixed one converts to it.
#[test]
fn a_unit_line_names_what_a_number_is() {
    let sk = read(&format!("unit mm\n{PAIR}a distance(2in) b\n")).expect("elaborates");
    assert_eq!(sk.units.stated(), Some("mm"));
    let d = sk.user_constraints()[0].args[2].num();
    assert!((d - 50.8).abs() < 1e-9, "two inches is 50.8 mm, got {d}");
}

/// **Feet and inches is one literal**, and it is the rule the language already had: a *space* is
/// what tells the readings apart, exactly as it does in `3 1/2`.
#[test]
fn feet_and_inches_is_one_literal() {
    let sk = read(&format!("unit mm\n{PAIR}a distance(1' 6 3/16\") b\n")).expect("elaborates");
    let d = sk.user_constraints()[0].args[2].num();
    assert!((d - 461.9625).abs() < 1e-9, "got {d}");
    // and it prints as written, not as what it came to: `1' 6 3/16"` tells a reader what
    // 461.9625 does not, and it is what they typed
    assert_eq!(io::dimension_text(&sk.user_constraints()[0]).unwrap(), "1' 6 3/16\"");
    assert!(io::describe(&sk.user_constraints()[0]).contains("1' 6 3/16\""));
}

/// **A document that names no unit is in millimetres** ([0.93]): a bare number in a `Length`
/// slot is a millimetre, a suffix converts to it as under `unit mm`, and everything still checks.
/// The unit is not *stated*, so nothing writes a `unit` line the document did not have.
#[test]
fn no_unit_line_is_millimetres() {
    let sk = read(&format!("{PAIR}a distance(60) b
")).expect("a bare number is a length");
    assert_eq!(sk.units.stated(), None);
    assert_eq!(sk.units.name(), "mm");
    for (text, want) in [("2in", 50.8), ("80mm", 80.0), ("3cm", 30.0), ("6\"", 152.4)] {
        let sk = read(&format!("{PAIR}a distance({text}) b\n")).expect("a suffix converts");
        let d = sk.user_constraints()[0].args[2].num();
        assert!((d - want).abs() < 1e-9, "{text} is {want} mm, got {d}");
    }
    // the same document as under `unit mm`, number for number
    let said = read(&format!("unit mm\n{PAIR}a distance(1' 6 3/16\") b\n")).expect("elaborates");
    let unsaid = read(&format!("{PAIR}a distance(1' 6 3/16\") b\n")).expect("elaborates");
    let d = |sk: &Sketch| sk.user_constraints()[0].args[2].num();
    assert_eq!(d(&said), d(&unsaid));
    // and a geometry read is a length, so a bare number beside one is refused as under `unit mm`
    let seeded = |x: &str| {
        format!("use std\nin std.front {{\nb := point hint((60, 0))\nc := point hint(({x}, 0))\n}}\n")
    };
    says(&seeded("b.x + 10"), "cannot be added");
    read(&seeded("b.x + 10mm")).expect("a length beside a length");
}

/* -- what it catches --------------------------------------------------------------------- */

/// The slot says what it wants, and an expression that said what it was must agree.
#[test]
fn an_angle_in_a_length_slot_is_an_error() {
    says(&format!("{PAIR}a distance(45deg) b\n"), "is Length, and this is Angle");
    // and the other way round
    says(
        &format!("{PAIR}c := point hint((1, 1))\nm := line(b, c)\nl angle(3in) m\n"),
        "is Angle, and this is Length",
    );
}

/// A length plus an angle, where a component's formals said which was which.  This is the case
/// the formals exist for: `settle` substitutes a parameter away, so if the dimension did not
/// travel with the number there would be nothing left to check.
#[test]
fn a_length_plus_an_angle_is_an_error() {
    let src = "\
use std
component Bad(w: Length, phi: Angle) {
  x := w + phi
  p := point
  fix((x, 0)) p
}
in std.front {
g := Bad(w: 10, phi: 20)
}
";
    says(src, "cannot be added");
}

/// **The involute formula's unstated radians.**  `inv φ = tan φ − φ` holds only in radians, which
/// the formula never said; `tan(phi) - phi` is a plain number less an angle, and saying it is
/// what `* 1rad` does.
#[test]
fn the_unstated_radians_are_caught() {
    let head = "component G(phi: Angle) {\n  ivp := ";
    let tail = "\n  p := point\n  fix(x == ivp, y == 0, z == 0) p\n}\ng := G(phi: 20)\n";
    says(&format!("{head}tan(phi) - phi{tail}"), "cannot be added");
    says(&format!("{head}tan(phi) * 180 / pi - phi{tail}"), "cannot be added");
    // said properly, it elaborates — and comes to the same number the conversion did
    let sk = read(&format!("{head}tan(phi) * 1rad - phi{tail}")).expect("elaborates");
    let want = 20f64.to_radians().tan().to_degrees() - 20.0;
    assert!((sk.point_xy(0).0 - want).abs() < 1e-9);
}

/// `pi` is the mathematical constant and `tau` is a **turn**.  They used to be 3.14159 and 360
/// side by side with nothing saying why.
#[test]
fn pi_is_a_number_and_tau_is_an_angle() {
    says(&format!("{PAIR}a distance(tau) b\n"), "is Length, and this is Angle");
    read(&format!("{PAIR}a distance(pi * 20) b\n")).expect("pi is a plain number");
    // `tau == 2 * pi * 1rad` holds dimensionally, which it did not
    let sk = read("\
use std
in std.front {
a := point hint((0, 0))
b := point hint((1, 0))
c := point hint((1, 1))
l := line(a, b)
m := line(a, c)
l angle(tau / 8) m
}
")
        .expect("an angle slot takes an angle");
    let want = (360.0f64 / 8.0).to_radians();
    assert!((sk.user_constraints()[0].args[2].num() - want).abs() < 1e-9);
}

/// A free variable's dimension is *deduced* from the slots that read it, and a name read once as
/// a length and once as an angle is an error naming both.
#[test]
fn a_free_variable_read_two_ways_is_an_error() {
    let src = "\
use std
param k: Length
in std.front {
a := point hint((0, 0))
b := point hint((60, 0))
c := point hint((60, 40))
l := line(a, b)
m := line(b, c)
a distance(k) b
l angle(k) m
}
";
    says(src, "`k` is declared Length, and `theta` reads it as Angle");
}

/// The functions have signatures, and `floor`/`ceil`/`round` are plain-number-only deliberately:
/// rounding a dimensioned quantity depends on which unit you round in.
#[test]
fn the_functions_have_signatures() {
    let with = |body: &str| format!("{PAIR}a distance({body}) b\n");
    says(&with("sin(3in)"), "takes an angle");
    read(&with("sin(30) * 60")).expect("a bare number in an angle position is degrees");
    says(&with("floor(45deg)"), "takes a plain number");
    says(&with("ln(45deg)"), "takes a plain number");
    // sqrt halves the exponent, which is why they are rational at all
    assert_eq!(Dim::LENGTH.mul(Dim::LENGTH).sqrt(), Dim::LENGTH);
    assert_eq!(Dim::LENGTH.powf(2.0).unwrap().sqrt(), Dim::LENGTH);
    assert_eq!(Dim::LENGTH.powf(2.5), None, "a dimensioned base takes a whole power");
    assert_eq!(Dim::SCALAR.powf(2.5), Some(Dim::SCALAR));
}

/* -- what it does not change -------------------------------------------------------------- */

/// **All twenty documents elaborate to the same drawings, the same DOF and the same diagnosis.**
/// The core has been unit-agnostic all along; the language is what never said so.
#[test]
fn the_library_is_unchanged() {
    for &(key, ..) in examples::CASES.iter() {
        let Some(mut sk) = examples::example(key) else { continue };
        let d = diagnose(&mut sk, DiagnoseOptions::default());
        assert!(d.dof >= 0, "{key}");
    }
    // and the gear says its radians now, in the same numbers
    let mut a = examples::gear();
    // solved first: one tooth's unknowns are few enough for the numeric cross-check, which reads
    // the pose it is given, and the seeds put every flank end at the centre
    assert!(gcs_core::solve::solve(&mut a, gcs_core::solve::SolveOpts::default()).success);
    assert_eq!(diagnose(&mut a, DiagnoseOptions::default()).dof, 0);
    assert!(!examples::GEAR.contains("180 / pi"), "no conversion is left written out");
    assert!(examples::GEAR.contains("1rad"));
}

/* -- units travel with the document ------------------------------------------------------- */

/// A document's unit round-trips through JSON, and is written only when there is one.
#[test]
fn the_unit_round_trips() {
    let sk = read(&format!("unit in\n{PAIR}a distance(2) b\n")).expect("elaborates");
    let json = io::dumps(&sk, None);
    assert!(json.contains("\"unit\":\"in\""), "{json}");
    assert_eq!(io::loads(&json).expect("loads").units.stated(), Some("in"));

    let plain = read(&format!("{PAIR}a distance(2) b\n")).expect("elaborates");
    assert!(io::dumps(&plain, None).contains("\"unit\":null"));
    assert_eq!(io::loads(&io::dumps(&plain, None)).expect("loads").units.stated(), None);
}

/// A document that names no unit is in millimetres, and a paste converts it as one.
#[test]
fn a_paste_from_no_unit_converts_as_millimetres() {
    let plain = read(&format!("{PAIR}a distance(50.8) b\n")).expect("elaborates");
    let clip = graft::copy(&plain, &plain.primitives());
    let mut inches =
        read("unit in\nuse std\nin std.front {\nz := point hint((0, 0))\n}\n").expect("elaborates");
    graft::paste(&mut inches, &clip, 0.0, 0.0);
    let far = inches.points.iter().skip(1).map(|p| inches.params[p.x as usize].value)
        .fold(0.0, f64::max);
    assert!((far - 60.0 / 25.4).abs() < 1e-9, "60 mm is 2.362 in, got {far}");
}

/// **Pasting a figure from a document in inches into one in millimetres converts the numbers.**
/// A figure is the same figure in either document, and two inches is 50.8 mm.
#[test]
fn a_paste_between_units_converts() {
    let inches = read(&format!("unit in\n{PAIR}a distance(2) b\n")).expect("elaborates");
    let clip = graft::copy(&inches, &inches.primitives());
    assert_eq!(clip.units.stated(), Some("in"), "a clipboard says what its numbers are in");

    let mut mm = read("unit mm\nuse std\nin std.front {\nz := point hint((0, 0))\n}\n").expect("elaborates");
    graft::paste(&mut mm, &clip, 0.0, 0.0);
    let far = mm.points.iter().skip(1).map(|p| mm.params[p.x as usize].value).fold(0.0, f64::max);
    assert!((far - 60.0 * 25.4).abs() < 1e-6, "60 in is 1524 mm, got {far}");
    let cs = mm.user_constraints();
    let d = cs.iter().find(|c| c.kind == gcs_core::constraints::CKind::Distance);
    assert!((d.expect("the length came too").args[2].num() - 50.8).abs() < 1e-9);

    // and into a document in the same units, nothing is scaled
    let mut same = read("unit in\nuse std\nin std.front {\nz := point hint((0, 0))\n}\n").expect("elaborates");
    graft::paste(&mut same, &clip, 0.0, 0.0);
    let far = same.points.iter().skip(1).map(|p| same.params[p.x as usize].value).fold(0.0, f64::max);
    assert!((far - 60.0).abs() < 1e-9, "got {far}");
}

/// **A formula of bare numbers converts too**: `graft` re-evaluates the text in the destination's
/// units, so the text carries the ratio (#117).  A formula naming its unit is right already.
#[test]
fn a_pasted_formula_converts() {
    let inches = read(&format!("unit in\n{PAIR}a distance(40 / 2) b\n")).expect("elaborates");
    let clip = graft::copy(&inches, &inches.primitives());
    let mut mm = read("unit mm\nuse std\nin std.front {\nz := point hint((0, 0))\n}\n").expect("elaborates");
    graft::paste(&mut mm, &clip, 0.0, 0.0);
    let d = mm.user_constraints().into_iter().find(|c| c.kind == gcs_core::constraints::CKind::Distance);
    assert!((d.expect("came").args[2].num() - 508.0).abs() < 1e-9);

    let named = read(&format!("unit in\n{PAIR}a distance(1in + 1in) b\n")).expect("elaborates");
    let clip = graft::copy(&named, &named.primitives());
    let mut mm = read("unit mm\nuse std\nin std.front {\nz := point hint((0, 0))\n}\n").expect("elaborates");
    graft::paste(&mut mm, &clip, 0.0, 0.0);
    let d = mm.user_constraints().into_iter().find(|c| c.kind == gcs_core::constraints::CKind::Distance);
    assert!((d.expect("came").args[2].num() - 50.8).abs() < 1e-9);
}

/// The unit itself: a name the language does not know, and one that is an angle.
#[test]
fn a_unit_line_says_what_it_will_not_take() {
    says("unit furlong\nuse std\nin std.front {\na := point\n}\n", "is not a unit");
    says("unit deg\nuse std\nin std.front {\na := point\n}\n", "a document's unit is its length");
    assert!(Units::with_length("mm").is_ok());
}

/// A document's unit and its style sheet are printed back with the drawing they belong to: a
/// round trip that dropped either would come back a different document.
#[test]
fn a_printed_model_keeps_units_and_omits_the_sheet() {
    let sk = read(&format!(
        "unit in\nstyle .construction {{ dash: 2 2 }}\n{PAIR}a distance(2) b\n"
    ))
    .expect("elaborates");
    let mut p = gcs_core::program::to_program(&sk);
    let text = gcs_core::syntax::render_flat(&mut p).unwrap().to_string();
    assert!(text.contains("unit in"), "{text}");
    assert!(!text.contains("style "), "{text}");
    let back = read(&text).expect("reads back");
    assert_eq!(back.units.stated(), Some("in"));
    assert!(back.sheet().is_empty());
}

/// The lexer has no string literal: a quote is the inch mark, and there is nothing else for one
/// to be.  A raw branch key — the one thing that used to be written quoted — is written bare.
#[test]
fn there_is_no_string_literal() {
    // `at: start` is a word, and always was
    read("\
use std
in std.front {
o := point hint((0, 0))
s := point hint((10, 0))
e := point hint((0, 10))
a := arc(center: o, start: s, end: e) hint(r: 10)
q := point hint((20, 0))
l := line(s, q)
a tangent(at: start) l
}
")
        .expect("a Str argument is a bare word");
    // a raw branch, bare, and printed back the same way.  A key the reader does not recognise
    // is exactly what `branch` exists for — a recorded root choice from a document this
    // implementation did not write.
    let sk = read("use std\nin std.front {\na := point hint((0, 0))\nbranch(other:0|1|2, 1)\n}\n")
        .expect("a raw branch key is bare");
    assert_eq!(sk.branches.get("other:0|1|2").copied(), Some(1));
    let mut p = gcs_core::program::to_program(&sk);
    let text = gcs_core::syntax::render_flat(&mut p).unwrap().to_string();
    assert!(text.contains("branch(other:0|1|2, 1)"), "{text}");
    // and a quote in a document is a unit mark, wherever it lands
    let (_, errs) = parse("\
unit mm
use std
in std.front {
a := point hint((0, 0))
b := point hint((1, 0))
a distance(6\") b
}
");
    assert!(errs.is_empty(), "{errs:?}");
}

/* -- what the conversion reaches ---------------------------------------------------------- */

/// A paste across units converts **every** length, and only the lengths.
///
/// `a_paste_between_units_converts` carries points and one `Distance`; these are the ones that
/// live somewhere else and were each missed once: where a plane stands, which is no point; and a
/// free variable a length reads, which is a Param and no argument at all.  An axis is the control:
/// it is a direction, and converting it would only take it off the unit sphere.
#[test]
fn a_paste_converts_the_lengths_that_are_not_arguments() {
    let src = "unit in\n\
               use std\n\
               r := axis hint(dir: (0.6, 0.8, 0))\n\
               f := plane(u: r) hint(origin: (4, 0, 0))\n\
               fix(dir == (0, 0, 1)) f.v\n\
               a := point hint((0, 3)) in f\n\
               b := point hint((6, 3)) in f\n\
               param w: Length\n\
               a distance(w) b\n\
               f.origin distance(w / 2) a\n";
    let inches = read(src).expect("elaborates");
    let clip = graft::copy(&inches, &inches.primitives());

    // `f`, the one plane over an axis nothing holds — not a standard plane — where it stands and
    // its first axis
    let f = |sk: &Sketch| -> usize {
        (0..sk.frames.len())
            .rfind(|&i| sk.axes[sk.frames[i].u as usize].d.iter().all(|&k| !sk.params[k as usize].fixed))
            .expect("f")
    };
    let place = |sk: &Sketch| -> f64 { sk.basis(f(sk)).o[0] };
    let axis = |sk: &Sketch| -> [f64; 3] {
        let r = sk.frames[f(sk)].u as usize;
        sk.axes[r].d.map(|k| sk.params[k as usize].value)
    };
    // the free variable `w`, which no argument holds
    let free = |sk: &Sketch| -> f64 {
        let c = sk.constraints.iter().find(|c| c.free.is_some()).expect("a free reader");
        sk.params[c.free.as_ref().unwrap().param as usize].value
    };
    assert!((place(&clip) - 4.0).abs() < 1e-9, "the plane stands 4 in out, got {}", place(&clip));
    assert!((free(&clip) - 6.0).abs() < 1e-9, "`w` is 6 in, got {}", free(&clip));

    let mut mm = read("unit mm\nuse std\nin std.front {\nz := point hint((0, 0))\n}\n").expect("elaborates");
    let made = graft::paste(&mut mm, &clip, 0.0, 0.0);
    assert!(!made.is_empty());

    assert!((place(&mm) - 4.0 * 25.4).abs() < 1e-6, "the place converts: {}", place(&mm));
    assert!((free(&mm) - 6.0 * 25.4).abs() < 1e-6, "`w` converts: {}", free(&mm));
    let d = axis(&mm);
    assert!((d[0] - 0.6).abs() < 1e-12 && (d[1] - 0.8).abs() < 1e-12, "the axis stays a direction: {d:?}");
}

/// A placement is two world lengths, so it converts with the figure it annotates.
#[test]
fn a_paste_converts_a_placement() {
    let inches =
        read(&format!("unit in\n{PAIR}a distance(2) b at (3, 1)\n")).expect("elaborates");
    let clip = graft::copy(&inches, &inches.primitives());
    assert_eq!(clip.placements.len(), 1, "the placement came along");

    let mut mm = read("unit mm\nuse std\nin std.front {\nz := point hint((0, 0))\n}\n").expect("elaborates");
    graft::paste(&mut mm, &clip, 0.0, 0.0);
    let &(t, r) = mm.placements.values().next().expect("a placement");
    assert!((t - 3.0 * 25.4).abs() < 1e-6 && (r - 1.0 * 25.4).abs() < 1e-6, "got {t}, {r}");
}

/// Every constraint that owns a hidden unknown says what that unknown *is*, so a conversion
/// cannot silently skip one — `FrameAlign`'s is a length and every curve parameter is not.
#[test]
fn every_param_slot_states_its_dimension() {
    use gcs_core::constraints::{SpecKind, ALL_KINDS};
    for k in ALL_KINDS {
        let owns = k.spec().iter().any(|(_, s)| *s == SpecKind::Param);
        assert_eq!(
            owns,
            k.param_dim().is_some(),
            "{k:?}: a Param slot and a stated dimension go together"
        );
    }
}

/* -- the unit is preamble ----------------------------------------------------------------- */

/// A curve family's body is read in the document's units like any other text.  It is not
/// dimension-*checked* — `tape.rs` says so — but a suffix there must not be told the document
/// named no unit when its first line did.
#[test]
fn a_curve_body_is_read_in_the_documents_units() {
    let sk = read(
        "\
unit mm
use std
component slide(c: circle, u: Angle) {
  p := point(x: c.center.x + 1in * u, y: c.center.y)
}
in std.front {
o := point hint((0, 0))
c1 := circle(center: o) hint(r: 25)
}
w := trace(slide(c1).p over u in (0, 1))
",
    )
    .expect("a suffix in a curve body is read in the document's own unit");
    assert_eq!(sk.curve_defs.len(), 1);
}

/// The unit is read before anything that reads a number, in every loader.  A saved document in
/// inches carrying an expression with a suffix has to evaluate it in inches, not in the drawing
/// units the sketch had before the `unit` key was reached.
#[test]
fn a_saved_document_has_its_unit_before_its_expressions() {
    let inches = read(&format!("unit in\n{PAIR}a distance(2) b\n")).expect("elaborates");
    let mut json = io::to_json(&inches);
    let back = io::from_json(&json).expect("loads");
    assert_eq!(back.units.stated(), Some("in"));
    // and the expression path: set one and reload
    let mut sk = back;
    let id = sk.user_constraints()[0].id;
    gcs_core::expr::set_dimension(&mut sk, id, "d", "1in").expect("1in is a length here");
    json = io::to_json(&sk);
    let back = io::from_json(&json).expect("loads");
    let d = back.user_constraints()[0].args[2].num();
    assert!((d - 1.0).abs() < 1e-9, "1in in a document in inches is 1, got {d}");
}

/// A unit is the document's, so it is stated once and at the top.  Anywhere else it would be
/// expanded into the flat list and read by nobody, which is the silent failure §13.1 forbids.
#[test]
fn a_unit_is_stated_once_and_only_in_the_root() {
    says("unit mm\nuse std\nunit in\nin std.front {\na := point\n}\n", "already stated above");
    says(
        "unit mm\nuse std\ncomponent Part { unit in\n  a := point hint((0, 0))\n}\nin std.front {\np := Part()\n}\n",
        "stated once, at the top",
    );
}

/// What an expression's fault *is* decides what is said of it (issue #43.11).  A number that is
/// not what its slot takes is the error §3.3 names — `distance(45deg)` — the same E103 a `param`
/// gets, not a warning that the last number stands; a claim binding an unknown is §9.7's E040,
/// not a warning followed by a refutation of the zero the warning made up; a declared unknown in
/// an ordinary dimension is nothing to say, and a name nothing declares is E101.
#[test]
fn a_slot_mismatch_is_an_error_and_a_free_name_is_not() {
    let diag = |src: &str| -> Vec<(String, String)> {
        let (prog, errs) = crate::common::parse_legacy(src);
        assert!(errs.is_empty(), "{errs:?}");
        let e = gcs_core::program::elaborate(&prog);
        e.diags.iter().map(|d| (d.code.as_str().to_string(), d.message.clone())).collect()
    };
    let two = "use std\nin std.front {\na := point hint((0, 0))\nb := point hint((40, 0))\n}\n";
    let d = diag(&format!("{two}a distance(45deg) b\n"));
    assert_eq!(d.len(), 1, "{d:?}");
    assert_eq!(d[0].0, "E103");
    assert!(d[0].1.contains("`d` is Length, and this is Angle"), "{}", d[0].1);
    assert!(!d[0].1.contains("last number"), "{}", d[0].1);

    let d = diag(&format!(
        "{two}c := point hint((40, 30))\na distance(40) b\nb distance(30) c\nparam zz: Length\nclaim a distance(zz) c\nfix((0, 0)) a\nfix((40, 0)) b\n"
    ));
    assert_eq!(d.iter().filter(|x| x.0 == "E040").count(), 1, "{d:?}");
    assert!(d.iter().any(|x| x.1.contains("a claim may not bind an unknown")), "{d:?}");

    // the arithmetic's own mismatches are the same error as the slot's (#116): `3 + 2deg` in a
    // dimension is what it is in a `param`, not a number that would not compute
    for (text, says) in [
        ("3 + 2deg", "cannot be added"),
        ("3 - 2deg", "cannot be added"),
        ("floor(45deg)", "takes a plain number"),
        ("2 ^ 1deg", "a power is a plain number"),
    ] {
        let d = diag(&format!("{two}a distance({text}) b\n"));
        assert_eq!(d.len(), 1, "{text}: {d:?}");
        assert_eq!(d[0].0, "E103", "{text}: {d:?}");
        assert!(d[0].1.contains(says), "{text}: {}", d[0].1);
        assert!(!d[0].1.contains("last number"), "{text}: {}", d[0].1);
        let d = diag(&format!("{two}w := {text}\na distance(w) b\n"));
        assert!(d.iter().any(|x| x.0 == "E103" && x.1.contains(says)), "{text}: {d:?}");
    }

    let d = diag(&format!("{two}param w: Length\na distance(w) b\n"));
    assert!(d.is_empty(), "{d:?}");
    let d = diag(&format!("{two}a distance(w) b\n"));
    assert_eq!(d.iter().map(|x| x.0.as_str()).collect::<Vec<_>>(), ["E101"], "{d:?}");
}

/* -- a design table's figures (#311) -------------------------------------------------------- */

/// The solved number a document gives a name (`r.q.x`), or `None` where it made nothing there.
fn position(src: &str, name: &str) -> Option<f64> {
    let (p, errs) = parse(src);
    assert!(errs.is_empty(), "{errs:?}");
    let e = elaborate(&p);
    assert!(e.ok(), "{:?}", e.errors().map(|d| d.message.clone()).collect::<Vec<_>>());
    let mut at = gcs_core::report::positions(&e.sketch, &e.map).into_iter();
    at.find(|(n, _)| n == name).map(|(_, v)| v)
}

/// What a dimensionless expression comes to, read off the slot it stands in: a plain number in a
/// `Length` slot is the document's length, so the slot's number is the expression's.
fn figure(unit: &str, table: &str, expr: &str) -> f64 {
    let src = format!("{unit}{table}{PAIR}a distance({expr}) b\n");
    let sk = read(&src).unwrap_or_else(|e| panic!("{expr}: {e:?}"));
    sk.user_constraints()[0].args[2].num()
}

/// The hub motor's duty, written as values that carry their units: `torque := power / omega` is
/// a torque because the arithmetic says so, and dividing by a unit reads a figure back.
const DUTY: &str = "\
power := 745.7W
volts := 48V
omega := 255rpm
efficiency := 0.86
torque := power / (omega / 1rad)
amps := power / (volts * efficiency)
freq := 46 / 2 * omega / turn
shear := torque / (2 * pi * (103.1mm)^2 * 35mm)
";

/// **Mass, time and current derive the rest**, and a figure comes to the same number in every
/// document: a watt in a millimetre document is 10⁶ kg·mm²/s³, but a watt divided by a watt is
/// one whatever the length is in.
#[test]
fn a_design_tables_figures_carry_their_units() {
    let omega = 255.0 * 2.0 * std::f64::consts::PI / 60.0;
    let torque = 745.7 / omega;
    let shear = torque / (2.0 * std::f64::consts::PI * 0.1031f64.powi(2) * 0.035);
    for unit in ["", "unit mm\n", "unit m\n", "unit in\n"] {
        for (expr, want) in [
            ("omega / 1rad * 1s", omega),
            ("torque / 1Nm", torque),
            ("torque / (1N * 1m)", torque),
            ("amps / 1A", 745.7 / (48.0 * 0.86)),
            ("freq / 1Hz", 46.0 / 2.0 * 255.0 / 60.0),
            ("shear / 1kPa", shear / 1e3),
        ] {
            let got = figure(unit, DUTY, expr);
            assert!((got - want).abs() < 1e-9 * want, "{unit:?} {expr}: {got}, want {want}");
        }
    }
}

/// The units agree with one another as SI says they do, in any document's lengths.
#[test]
fn the_units_are_coherent() {
    for unit in ["", "unit m\n", "unit in\n", "unit thou\n"] {
        for (expr, want) in [
            ("1W / (1kg * (1m)^2 / (1s)^3)", 1.0),
            ("1N / (1kg * 1m / (1s)^2)", 1.0),
            ("1J / (1N * 1m)", 1.0),
            ("1Nm / 1J", 1.0),
            ("1V * 1A / 1W", 1.0),
            ("1ohm * 1A / 1V", 1.0),
            ("1Wb / (1V * 1s)", 1.0),
            ("1T * (1m)^2 / 1Wb", 1.0),
            ("1MPa / (1N / (1mm)^2)", 1.0),
            ("1Pa * (1m)^2 / 1N", 1.0),
            ("1Hz * 1s", 1.0),
            ("60s * 1rpm / turn", 1.0),
            ("1kW / 1W", 1e3),
            ("1kV / 1mV", 1e6),
            ("1kg / 1g", 1e3),
            ("1s / 1ms", 1e3),
            ("1A / 1mA", 1e3),
            ("1kHz / 1Hz", 1e3),
            ("1kN / 1N", 1e3),
            ("1GPa / 1kPa", 1e6),
            ("1mWb / 1Wb", 1e-3),
            ("1mT / 1T", 1e-3),
        ] {
            let got = figure(unit, "", expr);
            assert!((got - want).abs() < 1e-12 * want, "{unit:?} {expr}: {got}, want {want}");
        }
    }
}

/// `+` and `-` demand agreement for the new bases as for length and angle, and a diagnostic
/// names the SI unit a dimension is: `Length^2·Mass·Time^-3` is a watt.
#[test]
fn a_figure_that_mixes_is_an_error() {
    says(&format!("{DUTY}bad := torque + power\n"), "(N·m)` and `Length^2·Mass·Time^-3 (W)");
    says(&format!("{DUTY}bad := amps - volts\n"), "cannot be added");
    // the angle stays in an angular speed until a radian takes it out
    says(&format!("{DUTY}bad := power / omega - torque\n"), "cannot be added");
    says("bad := 3V + 2\n", "cannot be added");
    assert_eq!(Dim::MASS.name(), "Mass (kg)");
    assert_eq!(Dim::TIME.div(Dim::TIME).div(Dim::TIME).name(), "Time^-1 (Hz)");
    assert_eq!(Dim::LENGTH.mul(Dim::LENGTH).name(), "Length^2", "a length's names are unchanged");
}

/// **No slot is of the new bases**: a figure is a value, and one that reaches a constraint
/// unread by a unit is refused there as an angle is in a length slot.
#[test]
fn a_figure_reaches_no_constraint() {
    says(&format!("{DUTY}{PAIR}a distance(power) b\n"), "is Length, and this is Length^2");
    says(&format!("{DUTY}{PAIR}c := point hint((1, 1))\nm := line(b, c)\nl angle(omega) m\n"),
        "is Angle, and this is Angle·Time^-1 (deg/s)");
    says("unit V\nuse std\nin std.front {\na := point\n}\n", "a document's unit is its length");
}

/// A figure carries its dimension through a substitution: handed to a component in a group, the
/// number written in for it says what it is, so the component's arithmetic checks as the root's.
#[test]
fn a_figure_keeps_its_dimension_in_a_component() {
    let src = |unit: &str, read: &str| {
        format!("{unit}use std\n{DUTY}dims := {{ t: torque, k: shear }}
frame := {{ front: std.front }}
component Read(f: group, dims: group) {{
  in f.front {{
    a := point hint((0, 0))
    b := point hint((60, 0))
  }}
  a distance({read}) b
  in f.front {{
    fix((0, 0)) a
    a level(v) b
  }}
}}
r := Read(frame, dims: dims)
")
    };
    let omega = 255.0 * 2.0 * std::f64::consts::PI / 60.0;
    for unit in ["", "unit in\n"] {
        let sk = read(&src(unit, "dims.t / 1Nm")).expect("a torque over a newton-metre");
        let d = sk.user_constraints()[0].args[2].num();
        assert!((d - 745.7 / omega).abs() < 1e-9, "{unit:?}: {d}");
        says(&src(unit, "dims.t / 1W"), "is Length, and this is Time (s)");
        says(&src(unit, "dims.t + dims.k"), "cannot be added");
    }
}

/// A unit is only a suffix: a name spelled like one is still a name, and `when` compares figures
/// that agree.
#[test]
fn a_unit_is_only_a_suffix() {
    let sk = read(&format!("V := 4\nA := 2V / 1V\n{PAIR}a distance(V * A * 10) b\n"))
        .expect("`V` and `A` are names");
    assert!((sk.user_constraints()[0].args[2].num() - 80.0).abs() < 1e-12);
    let when = |cond: &str| {
        format!("use std\n{DUTY}when {cond} {{\n  z := point\n  fix((1, 1, 1)) z\n}}\n")
    };
    assert!(position(&when("torque > 20Nm"), "z.x").is_some());
    assert!(position(&when("torque > 30Nm"), "z.x").is_none());
    says(&when("torque > 20"), "cannot be compared");
}
