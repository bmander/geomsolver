//! Units, and the dimensional analysis they make possible (spec §3.3).
//!
//! The return on a large change is what it *catches*, so most of this file is errors: a length
//! where an angle was wanted, a length added to an angle, the involute formula's unstated
//! radians, a free variable read as two different things.  The rest is that none of it changed
//! any drawing.

use gcs_core::diagnose::{diagnose, DiagnoseOptions};
use gcs_core::examples;
use gcs_core::io;
use gcs_core::model::Sketch;
use gcs_core::program::elaborate;
use gcs_core::syntax::parse;
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

const PAIR: &str = "point a hint(x: 0, y: 0)\npoint b hint(x: 60, y: 0)\nline l(a, b)\n";

/* -- the literal ------------------------------------------------------------------------- */

/// `unit mm` names what a bare number is, and a suffixed one converts to it.
#[test]
fn a_unit_line_names_what_a_number_is() {
    let sk = read(&format!("unit mm\n{PAIR}a distance(2in) b\n")).expect("elaborates");
    assert_eq!(sk.units.name(), Some("mm"));
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

/// A document that names no unit is in **drawing units**: everything still checks, and a suffix
/// is refused rather than guessed at.
#[test]
fn drawing_units_still_check_and_refuse_a_suffix() {
    let sk = read(&format!("{PAIR}a distance(60) b\n")).expect("a bare number is a length");
    assert_eq!(sk.units.name(), None);
    says(&format!("{PAIR}a distance(6\") b\n"), "names no unit");
    says(&format!("{PAIR}a distance(80mm) b\n"), "names no unit");
}

/* -- what it catches --------------------------------------------------------------------- */

/// The slot says what it wants, and an expression that said what it was must agree.
#[test]
fn an_angle_in_a_length_slot_is_an_error() {
    says(&format!("{PAIR}a distance(45deg) b\n"), "is Length, and this is Angle");
    // and the other way round
    says(
        &format!("{PAIR}point c hint(x: 1, y: 1)\nline m(b, c)\nl angle(3in) m\n"),
        "names no unit",
    );
}

/// A length plus an angle, where a component's formals said which was which.  This is the case
/// the formals exist for: `settle` substitutes a parameter away, so if the dimension did not
/// travel with the number there would be nothing left to check.
#[test]
fn a_length_plus_an_angle_is_an_error() {
    let src = "\
component Bad(w: Length, phi: Angle) {
  param x = w + phi
  point p hint(x: x, y: 0)
  ground p
}
g: Bad(w: 10, phi: 20)
";
    says(src, "cannot be added");
}

/// **The involute formula's unstated radians.**  `inv φ = tan φ − φ` holds only in radians, which
/// the formula never said; `tan(phi) - phi` is a plain number less an angle, and saying it is
/// what `* 1rad` does.
#[test]
fn the_unstated_radians_are_caught() {
    let head = "component G(phi: Angle) {\n  param ivp = ";
    let tail = "\n  point p hint(x: ivp, y: 0)\n  ground p\n}\ng: G(phi: 20)\n";
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
    let sk = read("point a hint(x: 0, y: 0)\npoint b hint(x: 1, y: 0)\npoint c hint(x: 1, y: 1)\n\
                   line l(a, b)\nline m(a, c)\nl angle(tau / 8) m\n")
        .expect("an angle slot takes an angle");
    let want = (360.0f64 / 8.0).to_radians();
    assert!((sk.user_constraints()[0].args[2].num() - want).abs() < 1e-9);
}

/// A free variable's dimension is *deduced* from the slots that read it, and a name read once as
/// a length and once as an angle is an error naming both.
#[test]
fn a_free_variable_read_two_ways_is_an_error() {
    let src = "\
point a hint(x: 0, y: 0)
point b hint(x: 60, y: 0)
point c hint(x: 60, y: 40)
line l(a, b)
line m(b, c)
a distance(k) b
l angle(k) m
";
    says(src, "one free name, one dimension");
}

/// The functions have signatures, and `floor`/`ceil`/`round` are plain-number-only deliberately:
/// rounding a dimensioned quantity depends on which unit you round in.
#[test]
fn the_functions_have_signatures() {
    let with = |body: &str| format!("{PAIR}a distance({body}) b\n");
    says(&with("sin(3in)"), "names no unit");
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
    assert_eq!(io::loads(&json).expect("loads").units.name(), Some("in"));

    let plain = read(&format!("{PAIR}a distance(2) b\n")).expect("elaborates");
    assert!(io::dumps(&plain, None).contains("\"unit\":null"));
    assert_eq!(io::loads(&io::dumps(&plain, None)).expect("loads").units.name(), None);
}

/// **Pasting a figure from a document in inches into one in millimetres converts the numbers.**
/// A figure is the same figure in either document, and two inches is 50.8 mm.
#[test]
fn a_paste_between_units_converts() {
    let inches = read(&format!("unit in\n{PAIR}a distance(2) b\n")).expect("elaborates");
    let clip = io::copy(&inches, &inches.primitives());
    assert_eq!(clip.units.name(), Some("in"), "a clipboard says what its numbers are in");

    let mut mm = read("unit mm\npoint z hint(x: 0, y: 0)\n").expect("elaborates");
    io::paste(&mut mm, &clip, 0.0, 0.0);
    let far = mm.points.iter().skip(1).map(|p| mm.params[p.x as usize].value).fold(0.0, f64::max);
    assert!((far - 60.0 * 25.4).abs() < 1e-6, "60 in is 1524 mm, got {far}");
    let cs = mm.user_constraints();
    let d = cs.iter().find(|c| c.kind == gcs_core::constraints::CKind::Distance);
    assert!((d.expect("the length came too").args[2].num() - 50.8).abs() < 1e-9);

    // and into a document in the same units, nothing is scaled
    let mut same = read("unit in\npoint z hint(x: 0, y: 0)\n").expect("elaborates");
    io::paste(&mut same, &clip, 0.0, 0.0);
    let far = same.points.iter().skip(1).map(|p| same.params[p.x as usize].value).fold(0.0, f64::max);
    assert!((far - 60.0).abs() < 1e-9, "got {far}");
}

/// The unit itself: a name the language does not know, and one that is an angle.
#[test]
fn a_unit_line_says_what_it_will_not_take() {
    says("unit furlong\npoint a\n", "is not a unit");
    says("unit deg\npoint a\n", "a document's unit is its length");
    assert!(Units::with_length("mm").is_ok());
}

/// A document's unit and its style sheet are printed back with the drawing they belong to: a
/// round trip that dropped either would come back a different document.
#[test]
fn a_printed_program_keeps_the_unit_and_the_sheet() {
    let sk = read(&format!(
        "unit in\nstyle .construction {{ dash: 2 2 }}\n{PAIR}a distance(2) b\n"
    ))
    .expect("elaborates");
    let mut p = gcs_core::program::to_program(&sk);
    let text = gcs_core::syntax::render(&mut p).to_string();
    assert!(text.contains("unit in"), "{text}");
    assert!(text.contains("style .construction { dash: 2 2 }"), "{text}");
    let back = read(&text).expect("reads back");
    assert_eq!(back.units.name(), Some("in"));
    assert_eq!(back.style_named("construction").dash, Some(vec![2.0, 2.0]));
}

/// The lexer has no string literal: a quote is the inch mark, and there is nothing else for one
/// to be.  A raw branch key — the one thing that used to be written quoted — is written bare.
#[test]
fn there_is_no_string_literal() {
    // `at: start` is a word, and always was
    read("point o hint(x: 0, y: 0)\npoint s hint(x: 10, y: 0)\npoint e hint(x: 0, y: 10)\n\
          arc a(center: o, start: s, end: e) hint(r: 10)\npoint q hint(x: 20, y: 0)\n\
          line l(s, q)\na tangent(at: start) l\n")
        .expect("a Str argument is a bare word");
    // a raw branch, bare, and printed back the same way.  A key the reader does not recognise
    // is exactly what `branch` exists for — a recorded root choice from a document this
    // implementation did not write.
    let sk = read("point a hint(x: 0, y: 0)\nbranch(other:0|1|2, 1)\n")
        .expect("a raw branch key is bare");
    assert_eq!(sk.branches.get("other:0|1|2").copied(), Some(1));
    let mut p = gcs_core::program::to_program(&sk);
    let text = gcs_core::syntax::render(&mut p).to_string();
    assert!(text.contains("branch(other:0|1|2, 1)"), "{text}");
    // and a quote in a document is a unit mark, wherever it lands
    let (_, errs) = parse("unit mm\npoint a hint(x: 0, y: 0)\npoint b hint(x: 1, y: 0)\n\
                           a distance(6\") b\n");
    assert!(errs.is_empty(), "{errs:?}");
}

/* -- what the conversion reaches ---------------------------------------------------------- */

/// A paste across units converts **every** length, and only the lengths.
///
/// `a_paste_between_units_converts` carries points and one `Distance`; these are the three that
/// live somewhere else and were each missed once: a frame's chord, which is a constraint's own
/// unknown; a free variable a length reads, which is a Param and no argument at all; and a
/// callout's placement, which is two world lengths on the statement.  A rotor is the control:
/// it is a direction, and converting it would only take the frame apart.
#[test]
fn a_paste_converts_the_lengths_that_are_not_arguments() {
    use gcs_core::constraints::CKind;

    let src = "unit in\n\
               point o hint(x: 0, y: 0)\n\
               point q hint(x: 4, y: 0)\n\
               frame f(origin: o, toward: q)\n\
               point a hint(x: 0, y: 3)\n\
               point b hint(x: 6, y: 3)\n\
               a distance(w) b\n\
               o distance(w / 2) a\n";
    let inches = read(src).expect("elaborates");
    let clip = io::copy(&inches, &inches.primitives());

    // the frame's chord, `frame_align`'s own unknown, and the rotor beside it
    let chord = |sk: &Sketch| -> f64 {
        let c = sk.constraints.iter().find(|c| c.kind == CKind::FrameAlign).expect("a frame");
        c.args[1].value(sk)
    };
    let rotor = |sk: &Sketch| -> (f64, f64) {
        let f = &sk.frames[0];
        (sk.params[f.c as usize].value, sk.params[f.s as usize].value)
    };
    // the free variable `w`, which no argument holds
    let free = |sk: &Sketch| -> f64 {
        let c = sk.constraints.iter().find(|c| c.free.is_some()).expect("a free reader");
        sk.params[c.free.as_ref().unwrap().param as usize].value
    };
    assert!((chord(&clip) - 4.0).abs() < 1e-9, "the chord is 4 in, got {}", chord(&clip));
    assert!((free(&clip) - 6.0).abs() < 1e-9, "`w` is 6 in, got {}", free(&clip));

    let mut mm = read("unit mm\npoint z hint(x: 0, y: 0)\n").expect("elaborates");
    let made = io::paste(&mut mm, &clip, 0.0, 0.0);
    assert!(!made.is_empty());

    assert!((chord(&mm) - 4.0 * 25.4).abs() < 1e-6, "the chord converts: {}", chord(&mm));
    assert!((free(&mm) - 6.0 * 25.4).abs() < 1e-6, "`w` converts: {}", free(&mm));
    let (c, s) = rotor(&mm);
    assert!((c * c + s * s - 1.0).abs() < 1e-9, "the rotor stays on the unit circle: {c}, {s}");
}

/// A placement is two world lengths, so it converts with the figure it annotates.
#[test]
fn a_paste_converts_a_placement() {
    let inches =
        read(&format!("unit in\n{PAIR}a distance(2) b at (3, 1)\n")).expect("elaborates");
    let clip = io::copy(&inches, &inches.primitives());
    assert_eq!(clip.placements.len(), 1, "the placement came along");

    let mut mm = read("unit mm\npoint z hint(x: 0, y: 0)\n").expect("elaborates");
    io::paste(&mut mm, &clip, 0.0, 0.0);
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
        "unit mm\n\
         curve ray(c: circle)(u) = ( c.center.x + 1in * u, c.center.y )\n\
         point o hint(x: 0, y: 0)\n\
         circle c1(center: o) hint(r: 25)\n",
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
    assert_eq!(back.units.name(), Some("in"));
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
    says("unit mm\nunit in\npoint a\n", "already stated above");
    says(
        "unit mm\ncomponent Part { unit in\n  point a hint(x: 0, y: 0)\n}\np: Part()\n",
        "stated once, at the top",
    );
}

/// What an expression's fault *is* decides what is said of it (issue #43.11).  A number that is
/// not what its slot takes is the error §3.3 names — `distance(45deg)` — the same E103 a `param`
/// gets, not a warning that the last number stands; a claim binding a free name is §9.7's E040,
/// not a warning followed by a refutation of the zero the warning made up; and a free name in
/// an ordinary dimension is still the W111 it always was.
#[test]
fn a_slot_mismatch_is_an_error_and_a_free_name_is_not() {
    let diag = |src: &str| -> Vec<(String, String)> {
        let (prog, errs) = gcs_core::syntax::parse(src);
        assert!(errs.is_empty(), "{errs:?}");
        let e = gcs_core::program::elaborate(&prog);
        e.diags.iter().map(|d| (d.code.as_str().to_string(), d.message.clone())).collect()
    };
    let two = "point a hint(x: 0, y: 0)\npoint b hint(x: 40, y: 0)\n";
    let d = diag(&format!("{two}a distance(45deg) b\n"));
    assert_eq!(d.len(), 1, "{d:?}");
    assert_eq!(d[0].0, "E103");
    assert!(d[0].1.contains("`d` is Length, and this is Angle"), "{}", d[0].1);
    assert!(!d[0].1.contains("last number"), "{}", d[0].1);

    let d = diag(&format!(
        "{two}point c hint(x: 40, y: 30)\na distance(40) b\nb distance(30) c\nclaim a distance(zz) c\nground a\nground b\n"
    ));
    assert_eq!(d.iter().filter(|x| x.0 == "E040").count(), 1, "{d:?}");
    assert!(d.iter().any(|x| x.1.contains("a claim may not bind an unknown")), "{d:?}");

    let d = diag(&format!("{two}a distance(w) b\n"));
    assert_eq!(d.iter().map(|x| x.0.as_str()).collect::<Vec<_>>(), ["W111"], "{d:?}");
}
