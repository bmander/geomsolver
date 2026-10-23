//! **One parameter axis** (#391 stage 1): `param t: Angle over (a, b)`, a number that runs over
//! an interval, and the one model type every number that runs is (`model::Running`) — a face
//! family's running number, a traced body's formals (in space and in the plane) and the
//! intervals of the shared roll a reading states (an envelope's in the plane and in space, a
//! swept solid's, a claim's).  No new behaviour: the gate is that each use reads the one type,
//! with the very numbers it read before (the corpus's own tests hold every solved value), and that
//! a declared parameter is represented once.  Stage 2's two operations on what moves: `x at τ`
//! makes what `x` is — a point held, a solid placed, a frame carried — and a reading over the
//! roll may name a declared parameter for its interval (`over t`); the spellings they replace are
//! refused naming them.  Stage 3's joint's value: `fix(value == τ) f`, the instant a jointed frame
//! stands at, and `n := x in f`, `x` drawn where that value puts it.

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
use std (spins, turns)
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
roll turns(1) spin_axis
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
const POINT: &str = "use std (spins, turns)\nin std.front {\n  o := point\n  fix((0, 0)) o\n}\n";

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
    // a value reading it is refused: a value is worked out once (a relation reads it, #391 S4)
    let e = refused("param t: Angle over (0deg, 90deg)\nw := 2 * t\nin std.front {\n  \
        a := point\n  b := point\n  fix((1, 0)) a\n  a distance(w * 1mm / 1deg) b\n}",
        "E103", "`t` runs over an interval, and a value is worked out once");
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
use std (spins, turns)
a0 := point hint((1, 2, 3))
a1 := point hint((4, 3, 11))
fix((1, 2, 3)) a0
fix((4, 3, 11)) a1
ax := line(a0, a1)
screw := frame
ax coincident(sense: same) screw.axis; fix(ratio == 1.5, advance == 7mm) screw
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
        "roll turns(1) spin_axis",
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
    syntax("d := solid(Station(g).f over s)", "`sweep(Component(…).face over s)`");
    syntax("d := sweep(Station(g).f over s in (0, 1))", "`over s in (a, b)` is retired");
    syntax("k := leg.toe over u in (0, 360)", "`trace(leg.toe over theta in (a, b))`");
    // and the frames of an instant stand on the `at`, not on the `coincident`
    syntax("P := point\nP coincident(under: screw) x at 40deg", "names its frames on the `at`");
}

/// How near a frame's value puts what is drawn in it to the motion's closed form: the solve's own
/// tolerance, as `tests/carried_frame.rs` holds a reading at an instant.
const EXACT: f64 = 1e-10;

/// The joint of the frame `name`, as the motion's closed form reads it.
fn family(e: &Elaborated, name: &str) -> gcs_core::motion::Family {
    let i = e.sketch.motions.iter().position(|m| m.name == name)
        .unwrap_or_else(|| panic!("no motion `{name}`"));
    gcs_core::motion::Family::read(&e.sketch, i).unwrap()
}

/// Plane `name`'s frame: its origin, `u`, `v` and normal.
fn chart(e: &Elaborated, name: &str) -> [[f64; 3]; 4] {
    let b = e.sketch.basis(e.map.ent_named(name).unwrap().i());
    [b.o, b.u, b.v, b.normal()]
}

fn near(got: [f64; 3], want: [f64; 3], what: &str) {
    let off = (0..3).map(|k| (got[k] - want[k]).abs()).fold(0.0, f64::max);
    assert!(off < EXACT, "{what}: {got:?} against {want:?}");
}

/// **A joint's value puts what is drawn in its frame** (#391 S3): `fix(value == 40deg) screw` is
/// the instant the screw stands at, so `n := std.front in screw` is the front plane where the
/// screw's joint carries it at 40° — turned `1.5 · 40°` about its axis and risen `7mm · 40/360`
/// along it, the value linear in the instant with the ratio and the advance its slopes, as the
/// motion's closed form gives it — and `X := x in screw` the point.  Each is the reading at that
/// instant, `x at(under: screw) 40deg`, row for row: no unknown of its own.  A frame stating no
/// value stands at rest, and what is drawn in it where it is drawn.
#[test]
fn what_is_drawn_in_a_jointed_frame_stands_where_its_value_puts_it() {
    let e = solved(&format!("{SCREW}fix(value == 40deg) screw\nn := std.front in screw\n\
        X := x in screw\nm := std.front at(under: screw) 40deg\nY := x at(under: screw) 40deg\n\
        still := frame\nstill turns(2) ax\nr := std.front in still\n"));
    let screw = family(&e, "screw");
    let pose = screw.at(40f64.to_radians()).unwrap();
    let front = chart(&e, "std.front");
    let [o, u, v, normal] = chart(&e, "n");
    near(o, pose.point(front[0]), "the origin");
    let turned = |d: [f64; 3]| {
        let (a, b) = (pose.point(d), pose.point([0.0; 3]));
        [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
    };
    near(u, turned(front[1]), "u");
    near(v, turned(front[2]), "v");
    near(normal, turned(front[3]), "the normal");
    let point = |n: &str| e.sketch.world_point(e.map.ent_named(n).unwrap().i());
    near(point("X"), pose.point(point("x")), "a point");
    // the reading at the value's instant, row for row
    for k in 0..4 {
        near(chart(&e, "n")[k], chart(&e, "m")[k], "the frame read at the instant");
    }
    near(point("X"), point("Y"), "the point read at the instant");
    let carried: Vec<_> = e.sketch.constraints.iter().filter(|c| c.kind.reads_motion()).collect();
    assert!(carried.iter().all(|c| c.stated_roll() && c.aux_params().is_empty()));
    assert_eq!(carried.len(), 3 + 1 + 3 + 1 + 3);
    // at rest: no value, the joint's at 0
    for k in 0..4 {
        near(chart(&e, "r")[k], front[k], "a frame at rest");
    }

    // a solid drawn in a frame is placed at its value
    let placed = |src: &str| {
        let e = built(src);
        let i = e.map.ent_named("removal").expect("the placed tool").i();
        let SolidDef::Placed { at, motion, .. } = &e.sketch.solids[i].def else {
            panic!("not placed: {:?}", e.sketch.solids[i].def);
        };
        (at.value.to_bits(), e.sketch.motions[*motion as usize].name.clone())
    };
    let sweep = "construction removal := sweep(tool over t in (-75deg, 75deg), under: spin, \
                 seen_from: observer)";
    assert!(SWEPT_TORUS.contains(sweep));
    let drawn = placed(&SWEPT_TORUS.replace(sweep, "fix(value == 30deg) spin\n\
        construction removal := tool in spin"));
    assert_eq!(drawn, placed(&SWEPT_TORUS.replace(sweep,
        "construction removal := tool at(under: spin) 30deg")));
    assert_eq!(drawn, (30f64.to_radians().to_bits(), "spin".to_string()));
}

/// **A value may be an unknown the drawing solves for** (#391 S3): the value names a free
/// variable, read as a stated roll naming one is (its twin's last column), and the turn that puts
/// the point drawn in the frame five above the world's `x` axis is the solve's — 30°, from a
/// seed of 10°.  A value worked out over a face family's running number is the station words'
/// (`tests/carried_frame.rs`, `tests/carried_family.rs`).
#[test]
fn a_joints_value_may_be_an_unknown_the_drawing_solves_for() {
    let e = solved("\
unit mm
use std (spins, turns)
param theta: Angle hint(10deg)
x := point hint((10, 0, 0))
fix((10, 0, 0)) x
spin := frame
spin spins(1) std.top.origin
fix(value == theta) spin
X := x in spin
std.origin distance(5mm, along: std.y) X
");
    let c = e.sketch.constraints.iter().find(|c| c.kind.reads_motion()).unwrap();
    let free = c.free.expect("the value is the unknown's");
    let theta = free.m * e.sketch.params[free.param as usize].value + free.c;
    assert!((theta.to_degrees() - 30.0).abs() < 1e-9, "{}", theta.to_degrees());
    let pose = family(&e, "spin").at(theta).unwrap();
    near(e.sketch.world_point(e.map.ent_named("X").unwrap().i()), pose.point([10.0, 0.0, 0.0]),
         "at the value solved");
}

/// **A value and what is drawn in its frame print as written** and read back the same; a value
/// is a jointed frame's, stated once, an angle, never a declared parameter's (a relation holding
/// for every value of one is stage 4's); and a frame with a value rides on none and carries
/// nothing drawn in its block, which would stand where it is drawn and not where the value puts it.
#[test]
fn a_joints_value_prints_back_and_refuses_what_it_cannot_mean() {
    // the flat form prints no instance: the joint written out as `std.Turn` writes it, its value
    // beside its other numbers
    let flat = SCREW.replace("ax coincident(sense: same) screw.axis; fix(ratio == 1.5, advance == 7mm) screw",
        "ax coincident screw.axis\nfix(ratio == 1.5, advance == 7mm, value == 40deg) screw");
    let src = format!("{flat}n := std.front in screw\nX := x in screw\n");
    let (mut prog, errs) = parse(&src);
    assert!(errs.is_empty(), "{errs:?}");
    let printed = gcs_core::syntax::render_flat(&mut prog).unwrap().to_string();
    for line in ["fix(ratio == 1.5, advance == 7mm, value == 40deg) screw",
                 "n := std.front in screw", "X := x in screw"] {
        assert!(printed.contains(line), "{line}: {printed}");
    }
    let (a, b) = (solved(&src), solved(&printed));
    let stated = solved(&format!("{SCREW}fix(value == 40deg) screw\nn := std.front in screw\n"));
    for k in 0..4 {
        near(chart(&a, "n")[k], chart(&b, "n")[k], "read back");
        near(chart(&a, "n")[k], chart(&stated, "n")[k], "the value beside the joint's numbers");
    }

    let refused = |src: &str, code: &str, want: &str| {
        let (_, said) = read(&format!("{SCREW}{src}\n"));
        assert!(said.iter().any(|(c, m)| c == code && m.contains(want)), "{src}: {said:?}");
    };
    refused("fix(value == 10deg) x", "E080", "a value is a jointed frame's");
    refused("fix(value == 10deg) screw\nfix(value == 20deg) screw", "E040", "given twice");
    refused("fix(value == 10mm) screw", "E103", "`value` is Angle, and this is Length");
    refused("fix(value == 10deg) screw\nin screw {\n  p := point\n}", "E080",
            "rides on `screw`, which has a value");
    refused("in (arm := frame) {\n  planet := frame\n}\narm turns(1) ax\n\
             planet turns(2) ax\nfix(value == 10deg) planet", "E080",
            "a frame with a value rides on none");
    // and a frame drawn in one that does not move is drawn in nothing that carries it
    refused("still := frame\nn := std.front in still", "E040", "does not move");
}

/// **A joint's value over a declared parameter** (#391 S4): `fix(value == t) screw` holds for
/// every `t`, so what is drawn in the screw is a trajectory — followed along `t`, the point drawn
/// in it stands at each station where the motion's closed form carries it at that instant, and
/// the drawing stands at the interval's start.
#[test]
fn a_joints_value_may_run_over_a_declared_parameter() {
    let e = solved(&format!("{SCREW}param t: Angle over (-30deg, 400deg)\n\
        fix(value == t) screw\nX := x in screw\n"));
    let screw = family(&e, "screw");
    let point = |sk: &gcs_core::model::Sketch, n: &str| {
        sk.world_point(e.map.ent_named(n).unwrap().i())
    };
    let x = point(&e.sketch, "x");
    near(point(&e.sketch, "X"), screw.at((-30f64).to_radians()).unwrap().point(x), "at rest");
    let f = gcs_core::follow::follow(&e.sketch, "t").expect("followed");
    assert!(f.singular.is_none() && f.reached() == 400.0, "{:?}", f.singular);
    let mut sk = e.sketch.clone();
    for s in &f.stations {
        sk.set_x(&s.x);
        near(point(&sk, "X"), screw.at(s.at.to_radians()).unwrap().point(x), "followed");
    }
}

/// **A value is no hold a gesture's reconcile can lose** (#391 S3): the sketch holds no number
/// for it, so a reconcile after a solve — what the page does after every edit — reads
/// `fix(value == τ) f` as no gauge and writes nothing, the value alone or beside the joint's
/// numbers.  It once took the statement out, and the cam on the timeline stopped turning.
#[test]
fn a_joints_value_survives_a_reconcile() {
    let flat = SCREW.replace("ax coincident(sense: same) screw.axis; fix(ratio == 1.5, advance == 7mm) screw",
        "ax coincident screw.axis\nfix(ratio == 1.5, advance == 7mm, value == t) screw");
    for src in [format!("{SCREW}param t: Angle over (-30deg, 400deg)\nfix(value == t) screw\n\
                         X := x in screw\n"),
                format!("param t: Angle over (-30deg, 400deg)\n{flat}X := x in screw\n")] {
        let mut e = solved(&src);
        let sk = e.sketch.clone();
        let out = gcs_core::edit::reconcile(&mut e, &sk);
        assert!(out.refused.is_none(), "{:?}", out.refused);
        assert!(out.text.contains("value == t) screw"), "reconcile wrote {}", out.text);
        assert_ne!(out.kind, gcs_core::edit::Kind::Structural, "reconcile wrote {}", out.text);
    }
}
