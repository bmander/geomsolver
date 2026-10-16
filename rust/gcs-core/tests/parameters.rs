//! **One parameter axis** (#391 stage 1): `param t: Angle over (a, b)`, a number that runs over
//! an interval, and the one model type every number that runs is (`model::Running`) — a face
//! family's running number, a traced body's formals (in space and in the plane) and the
//! intervals of the shared roll a reading states (an envelope's in the plane and in space, a
//! swept solid's, a claim's).  No new behaviour: the gate is that each use reads the one type,
//! with the very numbers it read before (the corpus's own tests hold every solved value), and that
//! a declared parameter is represented once.  Stage 2's two operations on what moves: `x at τ`
//! makes what `x` is — a point held, a solid placed, a frame carried — and a reading over the
//! roll may name a declared parameter for its interval (`over t`); the spellings they replace are
//! refused naming them.

use gcs_core::model::{EntKind, Roll, Running, SolidDef};
use gcs_core::program::{elaborate, Elaborated};
use gcs_core::units::Dim;

use crate::common::parse;

const REDUCER: &str = include_str!("../../../docs/lofts/l0/reducer.sv");
const PITCH: &str = include_str!("../../examples/rates/pitch.sv");
const SWEPT_TORUS: &str = include_str!("../../examples/swept_torus.sv");
const CLAIM: &str = include_str!("fixtures/solid_issue51/sweep_possible.sv");

/// An envelope in space over a stated roll (`tests/generated_envelopes.rs`'s).
const ENVELOPE: &str = "\
unit mm
use std
in std.front {
o := point
q := point
x := point
c := point
fix((0, 0)) o
fix((0, 1)) q
fix((1, 0)) x
fix((3, 0)) c
ax := line(o,q)
spin_axis := line(o,x)
meridian := circle(center: c)
radius(1mm) meridian
}
ring := solid(face(meridian),about: ax)
wall := surface(ring,meridian)
roll := frame
std.Turn(roll,about: spin_axis,ratio: 1)
generated := envelope(wall over t in (-20deg, 20deg), under: roll)
";

/// `src` linked against the library and elaborated: what was made, and the errors' codes and
/// words.
fn read(src: &str) -> (Elaborated, Vec<(String, String)>) {
    let (mut prog, errs) = parse(src);
    assert!(errs.is_empty(), "{errs:?}");
    let linked = gcs_core::modules::link(&mut prog, &mut gcs_core::library::resolve);
    assert!(linked.is_empty(), "{:?}", linked.iter().map(|d| &d.message).collect::<Vec<_>>());
    let e = elaborate(&prog);
    let said = e.errors().map(|d| (d.code.as_str().to_string(), d.message.clone())).collect();
    (e, said)
}

/// `src` elaborated, or the test fails saying why.
fn built(src: &str) -> Elaborated {
    let (e, said) = read(src);
    assert!(said.is_empty(), "{said:?}");
    e
}

/// A point, so a document is a drawing.
const POINT: &str = "use std\nin std.front {\n  o := point\n  fix((0, 0)) o\n}\n";

#[test]
fn a_declared_parameter_is_one_running_number() {
    let e = built(&format!("unit mm\nturns := 3\nparam t: Angle over (0deg, 360deg)\n\
        param s: Scalar over (0, turns)\nparam h: Length over (-1in, 2 * 5mm)\n{POINT}"));
    let p = &e.sketch.parameters;
    assert_eq!(p.len(), 3, "{p:?}");
    assert_eq!(p["t"], Running::new("t", Dim::ANGLE, 0.0, 360.0));
    assert!(p["t"].turns());
    assert_eq!(p["t"].radians(), [0.0, 360f64.to_radians()]);
    // an end may read any number the document defines, and is in the document's units
    assert_eq!(p["s"], Running::new("s", Dim::SCALAR, 0.0, 3.0));
    assert_eq!(p["h"], Running::new("h", Dim::LENGTH, -25.4, 10.0));
    // it is no unknown: the solve has no column for it, and nothing is declared for it to seed
    assert!(e.sketch.declared.is_empty() && e.sketch.free_vars.is_empty());
}

#[test]
fn a_declared_parameter_prints_as_written_and_lifts() {
    let src = format!("param t: Angle over (0deg, 360deg)\n{POINT}");
    let (mut prog, errs) = parse(&src);
    assert!(errs.is_empty(), "{errs:?}");
    let printed = gcs_core::syntax::render_flat(&mut prog).unwrap().to_string();
    assert!(printed.contains("param t: Angle over (0deg, 360deg)"), "{printed}");
    assert_eq!(built(&printed).sketch.parameters, built(&src).sketch.parameters);
    // a sketch lifted to a program declares it again, its ends in the document's units
    let lifted = gcs_core::program::dumps(&built(&src).sketch);
    assert!(lifted.contains("param t: Angle over (0, 360)"), "{lifted}");
    assert_eq!(built(&lifted).sketch.parameters, built(&src).sketch.parameters);
}

#[test]
fn a_parameter_refuses_what_it_cannot_mean() {
    let syntax = |src: &str, want: &str| {
        let (_, errs) = parse(&format!("{src}\n{POINT}"));
        assert!(errs.iter().any(|e| e.message.contains(want)), "{src}: {errs:?}");
    };
    syntax("param t: Angle over (0deg, 90deg) := 30deg", "has no value");
    syntax("param t: Angle over (0deg, 90deg) hint(30deg)", "a seed is for an unknown");
    syntax("param t over (0deg, 90deg)", "says what it is: `param t: Angle over (a, b)`");
    syntax("param t: Int over (0, 3)", "a `Length`, an `Angle` or a `Scalar`");
    syntax("component C(a: Length) {\n  param t: Angle over (0deg, 90deg)\n}", "formals");

    let refused = |src: &str, code: &str, want: &str| {
        let (e, said) = read(&format!("unit mm\n{src}\n{POINT}"));
        assert!(said.iter().any(|(c, m)| c == code && m.contains(want)), "{src}: {said:?}");
        e
    };
    refused("param t: Angle over (90deg, 0deg)", "E103", "runs over an increasing interval");
    refused("param t: Angle over (0deg, 0deg)", "E103", "runs over an increasing interval");
    refused("param t: Angle over (0mm, 1mm)", "E103", "`0mm`");
    refused("param w: Length\nparam t: Length over (0mm, w)", "E103", "reads an unknown");
    // read as a number it is refused, and the relation with it: no unknown is made of the name
    let e = refused("param t: Angle over (0deg, 90deg)\nin std.front {\n  a := point\n  \
        b := point\n  fix((1, 0)) a\n  a distance(t * 1mm / 1deg) b\n}",
        "E103", "`t` runs over an interval");
    assert!(!e.sketch.free_vars.contains_key("t"), "{:?}", e.sketch.free_vars);
    // a name is declared once, whatever it is
    refused("param t: Angle over (0deg, 90deg)\nt := 3", "E001", "declared twice");
}

#[test]
fn a_face_familys_running_number_is_a_running_number() {
    let e = built(REDUCER);
    let body = e.map.ent_named("body").expect("the reducer's body").i();
    let SolidDef::Family { over, .. } = &e.sketch.solids[body].def else {
        panic!("not a family: {:?}", e.sketch.solids[body].def);
    };
    assert!(over.name.ends_with(".s"), "{over:?}");
    assert_eq!((over.dim, over.span()), (Dim::SCALAR, [0.0, 1.0]));
    // the one name it is held under, at the middle of its interval
    let held = &e.sketch.held[&over.name];
    assert_eq!(held.dim, over.dim);
    assert_eq!(e.sketch.params[held.param as usize].value, over.mid());
}

#[test]
fn a_traced_bodys_formals_are_running_numbers() {
    let e = built(PITCH);
    let ent = |name: &str| e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`"));
    // over two formals, in space: `over (d in (0, rr), u in (10deg, 170deg))`
    let pitch = ent("pitch");
    assert_eq!(pitch.kind, EntKind::Traced);
    let t = &e.sketch.traced[pitch.i()];
    assert_eq!(t.over, vec![Running::new("d", Dim::LENGTH, 0.0, 6.0),
                            Running::new("u", Dim::ANGLE, 10.0, 170.0)]);
    assert_eq!(t.domain(), [[0.0, 6.0], [10.0, 170.0]]);
    // over one formal, in the plane: `Lobed(o).p over u in (0deg, 120deg)`
    let lobed = ent("lobed");
    assert_eq!(lobed.kind, EntKind::Curve);
    assert_eq!(e.sketch.curves[lobed.i()].over, Running::new("u", Dim::ANGLE, 0.0, 120.0));
    assert_eq!(e.sketch.curve_domain(lobed.i()), (0.0, 120.0));
}

#[test]
fn the_rolls_a_reading_states_are_intervals_of_the_shared_roll() {
    // an envelope in the plane runs over the shared roll as written, in degrees, named by the
    // parameter its `over` names (#391 S2)
    let e = built(PITCH);
    let rise = e.map.ent_named("pitch_rise").expect("the pitch curve");
    assert_eq!(rise.kind, EntKind::Curve);
    let over = &e.sketch.curves[rise.i()].over;
    assert_eq!(*over, Running::new("t", Dim::ANGLE, 0.0, 180.0));

    // an envelope in space: the same interval, read by its kernels in radians to the bit
    let e = built(ENVELOPE);
    let roll = &e.sketch.envelopes[0].roll;
    assert_eq!(*roll, Running::new("t", Dim::ANGLE, -20.0, 20.0));
    assert_eq!(roll.radians().map(f64::to_bits),
               [(-20f64).to_radians(), 20f64.to_radians()].map(f64::to_bits));

    // a swept solid's stated roll, and the interval every reader asks for
    let e = built(SWEPT_TORUS);
    let removal = e.map.ent_named("removal").expect("the sweep").i();
    let SolidDef::Swept { roll: Roll::Stated(r), .. } = &e.sketch.solids[removal].def else {
        panic!("not a stated sweep: {:?}", e.sketch.solids[removal].def);
    };
    assert_eq!(*r, Running::new("t", Dim::ANGLE, -75.0, 75.0));
    let interval = gcs_core::solid::roll::interval(&e.sketch, removal).unwrap();
    assert_eq!(interval.map(f64::to_bits),
               [(-75f64).to_radians(), 75f64.to_radians()].map(f64::to_bits));

    // a claim swept over a number of the drawing
    let e = built(CLAIM);
    let sweep = e.sketch.solid_claims[0].over().expect("a swept claim");
    assert_eq!(*sweep.over(), Running::new("reach", Dim::LENGTH, 0.1, 0.9));
    assert_eq!((sweep.name(), sweep.from(), sweep.to()), ("reach", 0.1, 0.9));
}

/// A screw about a line in space and a point at rest, for the readings at an instant (#391 S2).
const SCREW: &str = "\
unit mm
use std
a0 := point hint((1, 2, 3))
a1 := point hint((4, 3, 11))
fix((1, 2, 3)) a0
fix((4, 3, 11)) a1
ax := line(a0, a1)
screw := frame
std.Turn(screw, about: ax, ratio: 1.5, advance: 7mm)
x := point hint((9, -4, 6))
fix((9, -4, 6)) x
";

/// `src` elaborated and solved, or the test fails saying why.
fn solved(src: &str) -> Elaborated {
    let mut e = built(src);
    let r = gcs_core::solve::solve(&mut e.sketch, Default::default());
    assert!(r.success, "{r:?}");
    e
}

/// **`x at τ` makes what `x` is** (#391 S2): a point read at an instant is a point of its own held
/// where the frame carries `x` — the place `X coincident x at τ` holds a point drawn elsewhere at,
/// one `CarriedPoint` each at a stated roll — a solid read so is a solid placed at that roll, and
/// a frame read so the frame carried.
#[test]
fn a_point_read_at_an_instant_is_held_where_its_frame_carries_it() {
    let e = solved(&format!("{SCREW}X := x at(under: screw) 40deg\nP := point\n\
        P coincident x at(under: screw) 40deg\nn := std.front at(under: screw) 40deg\n"));
    let at = |n: &str| e.sketch.world_point(e.map.ent_named(n).unwrap().i());
    let (x, p) = (at("X"), at("P"));
    assert!((0..3).all(|k| (x[k] - p[k]).abs() < 1e-9), "{x:?} {p:?}");
    assert!((0..3).any(|k| (x[k] - at("x")[k]).abs() > 1.0), "carried, not left at rest");
    let carried = e.sketch.constraints.iter().filter(|c| c.kind.reads_motion());
    assert!(carried.clone().all(|c| c.stated_roll()));
    // the point and the frame's three are each held so: one row group apiece, and no other way
    // of writing the reading out survives it
    assert_eq!(carried.count(), 5);
    assert_eq!(e.map.ent_named("X").unwrap().kind, EntKind::Point);
    assert_eq!(e.map.ent_named("n").unwrap().kind, EntKind::Frame);
    assert!(e.sketch.solids.is_empty());

    // a solid read at an instant is placed there
    let e = built(&SWEPT_TORUS.replace(
        "construction removal := sweep(tool over t in (-75deg, 75deg), under: spin, seen_from: \
         observer)",
        "construction removal := tool at(under: spin, seen_from: observer) 30deg"));
    let removal = e.map.ent_named("removal").expect("the placed tool");
    assert_eq!(removal.kind, EntKind::Solid);
    let SolidDef::Placed { at, .. } = &e.sketch.solids[removal.i()].def else {
        panic!("not placed: {:?}", e.sketch.solids[removal.i()].def);
    };
    assert_eq!(at.value.to_bits(), 30f64.to_radians().to_bits());
}

/// **A reading over the roll reads a declared parameter** (#391 S2): `over t` takes the interval
/// `param t: Angle over (a, b)` declares, the very `Running` `over t in (a, b)` states — and is
/// printed as written; a parameter of another type, or a name nothing declares, is E103.
#[test]
fn a_reading_over_a_declared_parameter_reads_its_interval() {
    let inline = "construction removal := sweep(tool over t in (-75deg, 75deg), under: spin, \
                  seen_from: observer)";
    assert!(SWEPT_TORUS.contains(inline));
    let declared = SWEPT_TORUS.replace(inline, "param t: Angle over (-75deg, 75deg)\n\
        construction removal := sweep(tool over t, under: spin, seen_from: observer)");
    let roll = |e: &Elaborated| {
        let i = e.map.ent_named("removal").unwrap().i();
        let SolidDef::Swept { roll: Roll::Stated(r), .. } = &e.sketch.solids[i].def else {
            panic!("not a stated sweep: {:?}", e.sketch.solids[i].def);
        };
        r.clone()
    };
    let (a, b) = (built(SWEPT_TORUS), built(&declared));
    assert_eq!(roll(&a), roll(&b));
    assert_eq!(roll(&b), b.sketch.parameters["t"]);

    // an envelope in space over a declared parameter, the same — and printed as written, the
    // joint written out as `std.Turn` writes it, since the flat form prints no instance
    let env = ENVELOPE.replace("envelope(wall over t in (-20deg, 20deg), under: roll)",
                               "envelope(wall over r, under: roll)");
    let flat = format!("param r: Angle over (-20deg, 20deg)\n{}", env.replace(
        "std.Turn(roll,about: spin_axis,ratio: 1)",
        "spin_axis coincident roll.axis\nfix(ratio == 1, advance == 0mm) roll"));
    let e = built(&flat);
    assert_eq!(e.sketch.envelopes[0].roll, Running::new("r", Dim::ANGLE, -20.0, 20.0));
    let (mut prog, errs) = parse(&flat);
    assert!(errs.is_empty(), "{errs:?}");
    let printed = gcs_core::syntax::render_flat(&mut prog).unwrap().to_string();
    assert!(printed.contains("generated := envelope(surface: wall over r, motion: roll)"),
            "{printed}");
    assert_eq!(built(&printed).sketch.envelopes[0].roll, e.sketch.envelopes[0].roll);

    let refused = |src: &str, want: &str| {
        let (_, said) = read(src);
        assert!(said.iter().any(|(c, m)| c == "E103" && m.contains(want)), "{said:?}");
    };
    refused(&format!("param r: Length over (0mm, 1mm)\n{env}"), "`r` is no angle");
    refused(&env, "`r` names no parameter the document declares");
}

/// **The spellings `at` and `over` replace are refused, each naming its replacement** (#391 S2,
/// decision 7): a reading at a roll in a `solid`'s, a `carried`'s or a `coincident`'s brackets, an
/// envelope's or a meshing's `from:`/`to:`, a face family written `solid(…)` and a curve written
/// bare.
#[test]
fn the_retired_reading_spellings_name_their_replacements() {
    let syntax = |src: &str, want: &str| {
        let (_, errs) = parse(&format!("{SCREW}{src}\n"));
        assert!(errs.iter().any(|e| e.message.contains(want)), "{src}: {errs:?}");
    };
    syntax("P := point\nP coincident(under: screw, at: 40deg) x", "`X coincident x at τ`");
    syntax("n := std.front carried(under: screw, at: 40deg)", "`n := base at τ`");
    syntax("s := solid(tool, under: screw, at: 40deg)", "`s := tool at θ`");
    syntax("s := solid(tool, under: screw, from: 0deg, to: 40deg)",
           "`sweep(tool over t in (a, b), under: m, seen_from: g)`");
    syntax("s := solid(tool, under: screw, through: body)", "`sweep(tool, through: body");
    syntax("e := envelope(tool, under: screw, from: 0deg, to: 40deg)",
           "`envelope(tool over t in (a, b), …)`");
    syntax("m := meshing(f1, f2, under: screw, from: 0deg, to: 40deg)",
           "`meshing(f1, f2 over t in (a, b), …)`");
    syntax("d := solid(Station(g).f over s in (0, 1))", "`sweep(Component(…).face over s in");
    syntax("k := leg.toe over u in (0, 360)", "`trace(leg.toe over theta in (a, b))`");
    // and the frames of an instant stand on the `at`, not on the `coincident`
    syntax("P := point\nP coincident(under: screw) x at 40deg", "names its frames on the `at`");
}
