//! A generated flank regular within its blank (#345 S3): `flank regular(by: tool) within blank`,
//! the least area factor the flank takes over the part of it the tool cuts in the blank — found
//! by search — held at or above a margin (a bound) or to it (an equation).  The pair is
//! `spiral_bevel/formate.sv`'s, conjugate: the pinion the envelope of the gear tooth's inner cone
//! under the mesh (the gear's turn seen from the pinion's), cut within its blank.  Each number is
//! held to the sweep's admission (`solid::admission`), which samples the generated surface's area
//! factor by central differences over the tool's faces, its contacts in the blank, apart from
//! `envelope::regular` — the fold check of the flank as it is exported.
use gcs_core::constraints::CKind;
use gcs_core::program::Elaborated;
use gcs_core::solid::admission::{self, Condition, Error, Options};

/// The pinion's flank a held envelope of the gear tooth's inner cone under the mesh.
const FLANK: &str =
    "pinion_flank := envelope(mill.inner_flank over t in (-60deg, 60deg), under: gear_turn, seen_from: pinion_turn)";

/// No fold of the pinion's flank within its blank, where the gear's tooth cuts it.
pub(crate) const REGULAR: &str =
    "pinion_flank regular(by: gear_tooth.crown) within pinion_blank.material";

/// `REGULAR` with its number `margin` (`>= m` a bound, `m` an equation).
pub(crate) fn regular(margin: &str) -> String {
    format!("pinion_flank regular({margin}, by: gear_tooth.crown) within pinion_blank.material")
}

/// The Formate pair, conjugate, at the pressure shift `shift` (degrees), the pinion's flank held
/// and `stmt` stated of it — the shift left to the solve, seeded at 10°, where `unknown`.
pub(crate) fn pair(shift: f64, stmt: &str, unknown: bool) -> String {
    let text = fixtures::gear::design(&fixtures::gear::formate_source(), 25.0, shift, 25.0);
    let anchor = "  std.Turn(gear_turn, about: gear_cone.ax, ratio: -pinion_teeth / gear_teeth)\n";
    assert!(text.contains(anchor), "the pair's gear turn is where it was");
    let text = text.replace(anchor, &format!("{anchor}  {FLANK}\n  {stmt}\n"));
    if !unknown {
        return text;
    }
    let call = "  pressure_shift: pressure_shift, spiral_angle";
    assert!(text.contains(call), "the pair is called with its pressure shift");
    text.replace(call, "  pressure_shift: hint(10deg), spiral_angle")
}

/// The flank's least area factor as the solve reads it.
pub(crate) fn reading(e: &Elaborated) -> f64 {
    let sk = &e.sketch;
    let c = sk.constraints.iter().find(|c| c.kind == CKind::RegularWithin).expect("a regularity");
    c.reading(sk).expect("a regularity reads its least area factor")
}

/// The pressure shift as solved, in degrees.
fn shift(e: &Elaborated) -> f64 {
    let p = e.sketch.free_vars["pair.pressure_shift"];
    e.sketch.params[p as usize].value
}

/// The sweep's admission of the pinion: its least area factor, or the condition it is refused at.
pub(crate) fn admitted(e: &Elaborated) -> Result<f64, Condition> {
    let pinion = fixtures::solid(e, "pair.pinion.body");
    match admission::admit_body(&e.sketch, pinion, &Options::default()) {
        Ok(a) => Ok(a.sweeps()[0].least_area_factor),
        Err(Error::Refused(r)) => Err(r.condition),
        Err(e) => panic!("{e}"),
    }
}

/// The least area factor found is the one the admission samples of the exported flank, from
/// below by what its grid misses: at the configured shift, and nearer the fold.  Each is where the
/// inner cone's face meets its round, the tool's edge that cuts deepest.
#[test]
fn the_least_area_factor_is_the_one_the_admission_samples() {
    for shift in [12.5, 8.0] {
        let e = fixtures::gear::read(&pair(shift, REGULAR, false));
        let ours = reading(&e);
        let theirs = admitted(&e).expect("the pinion is admitted");
        eprintln!("{shift}°: least area factor {ours:.6}, sampled {theirs:.6}");
        assert!(ours > 0.0, "{shift}°: {ours}");
        assert!(theirs - ours >= -1e-9 && theirs - ours < 5e-3, "{shift}°: {ours} {theirs}");
    }
}

/// The bound becomes active exactly where the exported flank first folds: the shift at which the
/// least area factor is nothing, solved; a tenth of a degree above it the bound holds and the
/// admission admits the pinion; a tenth below, the solve says the bound is broken and the
/// admission refuses the pinion for a fold (E3).
#[test]
fn the_bound_breaks_where_the_exported_flank_first_folds() {
    let at = shift(&fixtures::gear::read(&pair(12.5, &regular("0"), true)));
    eprintln!("the flank folds below {at:.6}°");
    assert!(at > 7.0 && at < 8.0, "{at}");
    let above = fixtures::gear::read(&pair(at + 0.1, &regular(">= -1"), false));
    assert!(reading(&above) > 0.0);
    assert!(admitted(&above).is_ok_and(|least| least > 0.0));
    let below = fixtures::gear::read(&pair(at - 0.1, &regular(">= -1"), false));
    assert!(reading(&below) < 0.0);
    assert_eq!(admitted(&below), Err(Condition::Fold));
    // stated with no number, the flank regular: held above the fold, broken below it
    let mut e = fixtures::gear::unsolved(&pair(at - 0.1, REGULAR, false));
    let r = gcs_core::solve::solve(&mut e.sketch, fixtures::accurate());
    assert!(!r.success && r.status == gcs_core::solve::BOUND_BROKEN, "{r:?}");
    let mut e = fixtures::gear::unsolved(&pair(at + 0.1, REGULAR, false));
    assert!(gcs_core::solve::solve(&mut e.sketch, fixtures::accurate()).success);
}

/// A margin stated solves the setting it binds at: the pressure shift at which the pinion's least
/// area factor within its blank is 0.3, recorded; stated at that shift, the pair reads it back,
/// and the admission samples it.
#[test]
fn a_stated_margin_solves_the_pressure_shift() {
    let solved = shift(&fixtures::gear::read(&pair(12.5, &regular("0.3"), true)));
    eprintln!("a least area factor of 0.3 at {solved:.6}°");
    assert!((solved - 8.782426).abs() < 1e-4, "{solved}");
    let e = fixtures::gear::read(&pair(solved, REGULAR, false));
    assert!((reading(&e) - 0.3).abs() < 1e-9, "{}", reading(&e));
    let sampled = admitted(&e).expect("the pinion is admitted");
    assert!(sampled - 0.3 >= -1e-9 && sampled - 0.3 < 5e-3, "{sampled}");
}

/// The pair elaborated, its diagnostics returned rather than asserted.
fn elaborated(text: &str) -> Elaborated {
    let (mut p, errs) = gcs_core::syntax::parse(text);
    assert!(errs.is_empty(), "{errs:?}");
    let project = fixtures::gear::project();
    let errs = gcs_core::modules::link(&mut p, &mut fixtures::beside(&project, &mut |_, t| t));
    assert!(errs.is_empty(), "{errs:?}");
    gcs_core::program::elaborate(&p)
}

/// It prints as written — the number first, the tool by its key, `within` before the blank — and
/// reads back the same; one with no `within`, of a source that is no face of the tool, or within a
/// solid that is no region, is refused saying why.
#[test]
fn regular_prints_back_and_refuses_what_it_cannot_read() {
    let said = regular(">= 0.25");
    let (prog, errs) = gcs_core::syntax::parse(&pair(12.5, &said, false));
    assert!(errs.is_empty(), "{errs:?}");
    let stmt = prog.components.iter().flat_map(|c| &c.body)
        .find(|st| matches!(&st.kind, gcs_core::syntax::StmtKind::Relation(r)
            if r.form.written().is_some_and(|w| w.word.text == "regular")))
        .expect("the statement is in the pair");
    let mut printed = String::new();
    gcs_core::syntax::write_stmt_to(&mut printed, &stmt.kind).unwrap();
    assert_eq!(printed.trim(), said);
    let e = elaborated(&pair(12.5, &said, false));
    assert!(e.ok(), "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    let c = e.sketch.constraints.iter().find(|c| c.kind == CKind::RegularWithin).unwrap();
    let described = gcs_core::io::describe(c);
    let said_so = described.contains("regular(>= 0.25, by: ") && described.contains(") within ");
    assert!(said_so, "{described}");

    let (_, errs) = gcs_core::syntax::parse(&pair(12.5, &said.replace(" within ", " "), false));
    let want = "reads its second operand after `within`";
    assert!(errs.iter().any(|e| format!("{e:?}").contains(want)), "{errs:?}");
    let refused = |stmt: &str, want: &str| {
        let e = elaborated(&pair(12.5, stmt, false));
        assert!(e.errors().any(|d| d.message.contains(want)),
                "{want}: {:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    };
    refused(&said.replace("by: gear_tooth.crown", "by: pinion_blank.material"),
            "is no face of `pair.pinion_blank.material`");
    refused(&said.replace("within pinion_blank.material", "within pinion.body"), "is no region");
}
