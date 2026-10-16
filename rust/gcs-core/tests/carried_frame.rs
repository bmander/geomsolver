//! **A frame another frame's motion carries** (#366 S1): `n := base carried(under: m, at: τ)` is
//! the plane `base` carried by frame `m`'s pose at the roll `τ`, and `X coincident(under: m, at:
//! τ) x` the point.  The roll is a number the statement states (`CKind::CarriedPoint`'s roll slot
//! holding text, not an unknown: `Constraint::stated_roll`), so a station
//! may be carried to one written over its running number.  Each is held here to the motion's own
//! closed form, `motion::Family::at`, to `EXACT`: a frame's origin and axes, a point drawn in it, a
//! frame carried twice and by a frame seen from another, a point drawn in a view held there in two
//! rows, a screw up a plane's normal, a roll the drawing solves for, and the stations of
//! `std.Helix`, `std.OnHelix` and `std.Twisted` at places of their running numbers, against their
//! closed forms (#366 S2).
use crate::common::{build, ent};
use gcs_core::constraints::CKind;
use gcs_core::motion::Family;
use gcs_core::program::Elaborated;
use gcs_core::solve;
use gcs_core::space::{norm, sub};
use std::f64::consts::TAU;

/// How near the solve puts a carried frame to the motion's closed form: the solve's own
/// tolerance, not the export's.
const EXACT: f64 = 1e-10;

/// A frame screwing about a skew line and one turning about another (the first is read seen from
/// the second as `under: screw, seen_from: turn`).
const MOTIONS: &str = "\
unit mm
use std
a0 := point hint((1, 2, 3))
a1 := point hint((4, 3, 11))
fix((1, 2, 3)) a0
fix((4, 3, 11)) a1
ax := line(a0, a1)
b0 := point hint((-5, 0, 2))
b1 := point hint((-5, 9, 0))
fix((-5, 0, 2)) b0
fix((-5, 9, 0)) b1
bx := line(b0, b1)
screw := frame
std.Turn(screw, about: ax, ratio: 1.5, advance: 7mm)
turn := frame
std.Turn(turn, about: bx, ratio: -0.4)
";

fn solved(src: &str) -> Elaborated {
    let mut e = build(src);
    let r = solve::solve(&mut e.sketch, Default::default());
    assert!(r.success, "{r:?}");
    let d = gcs_core::diagnose::diagnose(&mut e.sketch, Default::default());
    assert_eq!(d.dof, 0, "{:?}", d.status);
    e
}

/// The motion of the frame `name`.
fn family(e: &Elaborated, name: &str) -> Family {
    let i = e.sketch.motions.iter().position(|m| m.name == name)
        .unwrap_or_else(|| panic!("no motion `{name}`"));
    Family::read(&e.sketch, i).unwrap()
}

/// The one motion seen from another the document reads (`under: m, seen_from: g`).
fn seen(e: &Elaborated) -> Family {
    let i = e.sketch.motions.iter()
        .position(|m| matches!(m.def, gcs_core::model::MotionDef::Relative { .. }))
        .expect("a motion seen from another");
    Family::read(&e.sketch, i).unwrap()
}

/// `p` where `f` carries it at `deg` degrees of its roll.
fn at(f: &Family, deg: f64, p: [f64; 3]) -> [f64; 3] {
    f.at(deg.to_radians()).unwrap().point(p)
}

/// A direction `d` turned as `f` turns it at `deg`.
fn turned(f: &Family, deg: f64, d: [f64; 3]) -> [f64; 3] {
    sub(at(f, deg, d), at(f, deg, [0.0; 3]))
}

fn same(got: [f64; 3], want: [f64; 3], what: &str) {
    assert!(norm(sub(got, want)) < EXACT, "{what}: {got:?} against {want:?}");
}

/// Plane `name`'s frame: its origin, `u`, `v` and normal.
fn frame(e: &Elaborated, name: &str) -> [[f64; 3]; 4] {
    let b = e.sketch.basis(ent(e, name).i());
    [b.o, b.u, b.v, b.normal()]
}

/// **A plane carried by a screw is the screw's pose of it**, past a half turn and either way:
/// its origin where the pose carries the source's, each of its axes the source's turned, and a
/// point drawn in it where the pose carries the source's point at the same place.
#[test]
fn a_plane_carried_by_a_screw_is_the_screws_pose_of_it() {
    let rolls = [-200.0, -30.0, 0.0, 75.0, 140.0, 300.0, 725.0];
    let mut src = MOTIONS.to_string();
    for (k, t) in rolls.iter().enumerate() {
        src += &format!("n{k} := std.front carried(under: screw, at: {t}deg)\n\
                         in n{k} {{\n  p{k} := point\n}}\nfix((3, 4)) p{k}\n");
    }
    let e = solved(&src);
    let f = family(&e, "screw");
    let front = frame(&e, "std.front");
    let p = e.sketch.basis(ent(&e, "std.front").i()).lift(3.0, 4.0);
    for (k, &t) in rolls.iter().enumerate() {
        let [o, u, v, n] = frame(&e, &format!("n{k}"));
        same(o, at(&f, t, front[0]), &format!("τ {t}: the origin"));
        same(u, turned(&f, t, front[1]), &format!("τ {t}: u"));
        same(v, turned(&f, t, front[2]), &format!("τ {t}: v"));
        same(n, turned(&f, t, front[3]), &format!("τ {t}: the normal"));
        same(e.sketch.world_point(ent(&e, &format!("p{k}")).i()), at(&f, t, p),
             &format!("τ {t}: a point drawn in it"));
    }
}

/// **A frame carried twice is the poses composed**, and one carried by a frame seen from another
/// is that relative's pose of it.
#[test]
fn a_frame_carried_twice_is_the_poses_composed() {
    let src = format!("{MOTIONS}\
n := std.top carried(under: screw, seen_from: turn, at: -65deg)
m := n carried(under: screw, at: 230deg)
");
    let e = solved(&src);
    let (screw, seen) = (family(&e, "screw"), seen(&e));
    let top = frame(&e, "std.top");
    let [o, u, v, _] = frame(&e, "m");
    same(o, at(&screw, 230.0, at(&seen, -65.0, top[0])), "the origin");
    same(u, turned(&screw, 230.0, turned(&seen, -65.0, top[1])), "u");
    same(v, turned(&screw, 230.0, turned(&seen, -65.0, top[2])), "v");
}

/// **A point carried to a stated roll**, in space under a screw and seen from a turn; drawn in a
/// view under a turn in that view, held there by two rows (it has two numbers); and under a screw
/// about a line drawn up a plane's normal, rising along it.
#[test]
fn a_point_carried_to_a_stated_roll_is_the_pose_of_it() {
    let src = format!("{MOTIONS}\
x := point hint((9, -4, 6))
fix((9, -4, 6)) x
P := point
P coincident(under: screw, at: 40deg) x
Q := point
Q coincident(under: screw, seen_from: turn, at: -125deg) x
in std.top {{
  c := point
  fix((2, 3)) c
  y := point
  fix((7, 3)) y
  T := point
}}
twirl := frame
std.Spin(twirl, about: c, on: std.top, ratio: 0.7)
T coincident(under: twirl, at: 100deg) y
up := point hint((0, 0, 10))
fix((0, 0, 10)) up
spindle := line(std.origin, up)
lift := frame
std.Turn(lift, about: spindle, ratio: 1, advance: 4mm)
S := point
S coincident(under: lift, at: 290deg) x
");
    let e = solved(&src);
    let point = |n: &str| e.sketch.world_point(ent(&e, n).i());
    same(point("P"), at(&family(&e, "screw"), 40.0, point("x")), "under the screw");
    same(point("Q"), at(&seen(&e), -125.0, point("x")), "seen from the turn");
    same(point("T"), at(&family(&e, "twirl"), 100.0, point("y")), "in its view");
    let rise = point("S")[2] - point("x")[2];
    same(point("S"), at(&family(&e, "lift"), 290.0, point("x")), "up a plane's normal");
    assert!((rise - 4.0 * 290.0 / 360.0).abs() < EXACT, "risen {rise}");
    // the point in the view is held by as many rows as it has numbers; one in space by three
    let rows = |k: CKind| e.sketch.constraints.iter().filter(|c| c.kind == k && c.stated_roll())
        .map(|c| c.rows_in(&e.sketch)).collect::<Vec<_>>();
    assert_eq!(rows(CKind::CarriedPointIn), vec![2]);
    assert_eq!(rows(CKind::CarriedPoint), vec![3; 3]);
}

/// **A frame carried reads what its source rides on** (#366, #383): `base` drawn over points in
/// the block of a jointed frame rides on it, so `n := base carried(at: τ)` is `base` carried by
/// that frame's joint, as a point riding is (`X coincident(at: τ) x`) — the pose `Family::at`
/// gives, to `EXACT`.  Naming the frame `under:` for a source that rides is refused, as on any
/// reading, and a source riding on nothing must name one.
#[test]
fn a_frame_carried_reads_what_its_source_rides_on() {
    let src = "\
unit mm
use std
a0 := point hint((1, 2, 3))
a1 := point hint((4, 3, 11))
fix((1, 2, 3)) a0
fix((4, 3, 11)) a1
ax := line(a0, a1)
in (screw := frame) {
  o := point hint((9, -4, 6))
  p := point hint((19, -4, 6))
  q := point hint((9, 6, 6))
  fix((9, -4, 6)) o
  fix((19, -4, 6)) p
  fix((9, 6, 6)) q
  construction lu := line(o, p)
  construction lv := line(o, q)
  base := plane(u: lu, v: lv)
}
std.Turn(screw, about: ax, ratio: 1.5, advance: 7mm)
n := base carried(at: 40deg)
X := point
X coincident(at: 40deg) q
";
    let e = solved(src);
    let screw = family(&e, "screw");
    let [o, u, v, _] = frame(&e, "n");
    let [bo, bu, bv, _] = frame(&e, "base");
    same(o, at(&screw, 40.0, bo), "the origin carried");
    same(u, turned(&screw, 40.0, bu), "u carried");
    same(v, turned(&screw, 40.0, bv), "v carried");
    same(e.sketch.world_point(ent(&e, "X").i()), at(&screw, 40.0, [9.0, 6.0, 6.0]),
         "a point riding, carried alike");
    // `under:` on a source that rides is the reading's to infer, and refused
    let (prog, errs) = crate::common::parse(&src.replace("base carried(at:",
                                                         "base carried(under: screw, at:"));
    assert!(errs.is_empty(), "{errs:?}");
    let e = gcs_core::program::elaborate(&prog);
    assert!(!e.ok(), "a riding source names no frame `under:`");
    // and a source riding on nothing must name the frame it is carried by
    let (prog, errs) = crate::common::parse(&format!("{src}m := std.front carried(at: 10deg)\n"));
    assert!(errs.is_empty(), "{errs:?}");
    let e = gcs_core::program::elaborate(&prog);
    assert!(e.errors().any(|d| d.message.contains("rides on none: name the frame")),
            "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
}

/// **A roll owned and a roll stated are one reading** (#366): `t == 40` (the relation's own
/// unknown, pinned) and `at: 40deg` (the angle stated) are the one kind, `CarriedPoint`, at one
/// place to `EXACT` — the first an unknown of its own, the second none — and saying both is
/// refused, as any slot said twice is.
#[test]
fn a_roll_owned_and_a_roll_stated_are_one_reading() {
    let src = "\
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
P := point hint((9, -4, 6))
P coincident(under: screw, t == 40) x
Q := point hint((9, -4, 6))
Q coincident(under: screw, at: 40deg) x
";
    let e = solved(src);
    let carried: Vec<_> = e.sketch.constraints.iter().filter(|c| c.kind.reads_motion()).collect();
    assert_eq!(carried.len(), 2);
    assert!(carried.iter().all(|c| c.kind == CKind::CarriedPoint));
    assert_eq!(carried.iter().filter(|c| c.stated_roll()).count(), 1);
    assert_eq!(carried.iter().map(|c| c.aux_params().len()).sum::<usize>(), 1,
               "the owned roll is an unknown, the stated one none");
    let point = |n: &str| e.sketch.world_point(ent(&e, n).i());
    same(point("P"), point("Q"), "one place");
    same(point("Q"), at(&family(&e, "screw"), 40.0, point("x")), "the pose at 40°");
    let (prog, errs) = crate::common::parse(&src.replace("at: 40deg", "t == 40, at: 40deg"));
    assert!(errs.is_empty(), "{errs:?}");
    let e = gcs_core::program::elaborate(&prog);
    assert!(e.errors().any(|d| d.message.contains("is given twice: the roll")),
            "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
}

/// **A roll may be an unknown the drawing solves for**: the stated roll names a free variable
/// (its twin is the same kernel, the unknown its last column), and the turn that puts the point
/// five above the world's `x` axis is the solve's — 30°, from a seed of 10°.
#[test]
fn a_roll_may_be_an_unknown_the_drawing_solves_for() {
    let src = "\
unit mm
use std
param theta: Angle hint(10deg)
x := point hint((10, 0, 0))
fix((10, 0, 0)) x
spin := frame
std.Spin(spin, about: std.top.origin, on: std.top, ratio: 1)
X := point
X coincident(under: spin, at: theta) x
std.origin distance(5mm, along: std.y) X
";
    let e = solved(src);
    let c = e.sketch.constraints.iter().find(|c| c.kind == CKind::CarriedPoint).unwrap();
    assert!(c.stated_roll());
    let free = c.free.expect("the roll is the unknown's");
    let theta = free.m * e.sketch.params[free.param as usize].value + free.c;
    assert!((theta.to_degrees() - 30.0).abs() < 1e-9, "{}", theta.to_degrees());
    same(e.sketch.world_point(ent(&e, "X").i()),
         at(&family(&e, "spin"), 30.0, [10.0, 0.0, 0.0]), "at the roll solved");
}

/// **`std.Helix`'s station is `base` carried by its screw**, read at three places of its running
/// number: its `n` the screw's pose of `base` at `360deg · turns · s`, and its `axial` `n` turned a
/// quarter about its `u`, so its `v` runs up the axis.
#[test]
fn a_helix_station_is_its_base_carried_by_its_screw() {
    for s in [0.0, 0.3, 0.85] {
        let src = format!("unit mm\nuse std\nst := std.Helix(std.top, pitch: 4mm, turns: 3, \
                           s: {s})\n");
        let e = solved(&src);
        let screw = family(&e, "st.screw");
        let roll = 360.0 * 3.0 * s;
        let top = frame(&e, "std.top");
        let [o, u, v, n] = frame(&e, "st.n");
        same(o, at(&screw, roll, top[0]), &format!("s {s}: the origin"));
        same(u, turned(&screw, roll, top[1]), &format!("s {s}: u"));
        same(v, turned(&screw, roll, top[2]), &format!("s {s}: v"));
        same(o, [0.0, 0.0, 4.0 * 3.0 * s], &format!("s {s}: risen a pitch a turn"));
        let [ao, au, av, _] = frame(&e, "st.axial");
        same(ao, o, &format!("s {s}: the axial plane's origin"));
        same(au, u, &format!("s {s}: the axial plane's u"));
        same(av, n, &format!("s {s}: the axial plane's v, up the axis"));
    }
}

/// **`std.OnHelix`'s station is square to its helix**, as its closed form says, at three places
/// of its running number: its origin on the helix of radius `r` at `θ = 360° · turns · s`, its `u`
/// out from the axis there and its `v` up the axis leaned back by the lead angle — `(p/L · sin θ,
/// −p/L · cos θ, 2πr/L)`, `L` a turn's length — its start `base` tilted about its `u` and slid out
/// by one screw, the helix's screw carrying it from there.
#[test]
fn an_on_helix_station_is_square_to_its_helix() {
    let (r, p, turns) = (10.0, 5.0, 4.0);
    let lead = (TAU * r).hypot(p);
    for s in [0.0, 0.13, 0.6] {
        let src = format!("unit mm\nuse std\nst := std.OnHelix(std.top, r: 10mm, pitch: 5mm, \
                           turns: 4, s: {s})\n");
        let e = solved(&src);
        let t = TAU * turns * s;
        let [o, u, v, _] = frame(&e, "st.n");
        same(o, [r * t.cos(), r * t.sin(), p * turns * s], &format!("s {s}: on the helix"));
        same(u, [t.cos(), t.sin(), 0.0], &format!("s {s}: out from the axis"));
        same(v, [p / lead * t.sin(), -p / lead * t.cos(), TAU * r / lead],
             &format!("s {s}: leaned back by the lead angle"));
    }
}

/// **`std.Twisted`'s point is its plane's `u` turned within it**: `reach` out along `n.u` turned
/// `twist · s` about `n`'s origin, past a half turn as written.
#[test]
fn a_twisted_point_turns_within_its_plane() {
    for (twist, s) in [(250.0, 0.4), (1260.0, 0.75), (-90.0, 0.5)] {
        let src = format!("unit mm\nuse std\ntw := std.Twisted(std.top, twist: {twist}deg, \
                           reach: 7mm, s: {s})\n");
        let e = solved(&src);
        let a = f64::to_radians(twist * s);
        same(e.sketch.world_point(ent(&e, "tw.t").i()), [7.0 * a.cos(), 7.0 * a.sin(), 0.0],
             &format!("{twist}° at {s}"));
    }
}

/// The statements print as they are written and read back the same; a frame names its roll, a
/// roll is an angle, and a frame is carried from a plane.
#[test]
fn a_carried_frame_prints_back_and_is_refused_where_it_says_too_little() {
    // the flat form prints no component instance: the joints written out as `std.Turn` writes them
    let flat = MOTIONS
        .replace("std.Turn(screw, about: ax, ratio: 1.5, advance: 7mm)",
                 "ax coincident screw.axis\nfix(ratio == 1.5, advance == 7mm) screw")
        .replace("std.Turn(turn, about: bx, ratio: -0.4)",
                 "bx coincident turn.axis\nfix(ratio == -0.4, advance == 0mm) turn");
    let src = format!("{flat}\
n := std.front carried(under: screw, at: 140deg)
x := point hint((9, -4, 6))
fix((9, -4, 6)) x
P := point
P coincident(under: screw, at: 40deg) x
");
    let (mut prog, errs) = crate::common::parse(&src);
    assert!(errs.is_empty(), "{errs:?}");
    let printed = gcs_core::syntax::render_flat(&mut prog).unwrap().to_string();
    assert!(printed.contains("n := std.front carried(under: screw, at: 140deg)"), "{printed}");
    assert!(printed.contains("P coincident(under: screw, at: 40deg) x"), "{printed}");
    let (a, b) = (solved(&src), solved(&printed));
    for k in 0..4 {
        same(frame(&a, "n")[k], frame(&b, "n")[k], "read back");
    }
    // a frame with no roll says nothing of where it is carried
    let (_, errs) =
        crate::common::parse(&format!("{MOTIONS}n := std.front carried(under: screw)\n"));
    assert!(errs.iter().any(|e| e.message.contains("carried to a roll by the frames it rides")),
            "{errs:?}");
    // a roll is an angle
    let (prog, errs) = crate::common::parse(&format!("{MOTIONS}x := point\nP := point\n\
        P coincident(under: screw, at: 10mm) x\n"));
    assert!(errs.is_empty(), "{errs:?}");
    let e = gcs_core::program::elaborate(&prog);
    assert!(e.errors().any(|d| d.code.as_str() == "E103"),
            "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    // and a frame is carried from a plane
    let (prog, errs) =
        crate::common::parse(&format!("{MOTIONS}n := ax carried(under: screw, at: 10deg)\n"));
    assert!(errs.is_empty(), "{errs:?}");
    let e = gcs_core::program::elaborate(&prog);
    assert!(!e.ok(), "a line is no frame");
}
