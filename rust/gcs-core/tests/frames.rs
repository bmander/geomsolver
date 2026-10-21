//! **Frames** (#383, `docs/frames-plan.md`): one declaration for planes and motions.  Stage 1 is
//! a chain of motions in the core with no syntax: `Sketch::chain(outer, inner)` is the pose
//! `outer(τ)·inner(τ)`, lowered as it is pushed to the relative of `inner` seen from `outer`'s
//! minted inverse — or, over one axis, folded to one joint — so every reader of a motion reads a
//! chain on the first day.  Each is held here to a closed form worked out apart from the core: a
//! planet's point on its arm, three turns multiplied by hand, a wheel rolling along a slide (the
//! cycloid), and two screws on one shaft read back as one screw.
use crate::common::build;
use gcs_core::constraints::{Arg, CKind, Constraint};
use gcs_core::model::{EntKind, EntRef, MotionDef, Sketch};
use gcs_core::motion::Family;
use gcs_core::solve;
use gcs_core::space::{norm, sub};

/// An arm turning about `o` a radian a radian, and a planet on it turning about `c` (drawn at
/// rest at (30, 0)) three times as fast the other way; `x` is a point of the planet.
const PLANET: &str = "\
unit mm
use std
in std.top {
  o := point
  fix((0, 0)) o
  c := point
  fix((30, 0)) c
  x := point
  fix((40, 0)) x
  X := point hint((10, 17))
}
arm := frame
std.Spin(arm, about: o, on: std.top, ratio: 1)
planet := frame
std.Spin(planet, about: c, on: std.top, ratio: -3)
";

fn index(sk: &Sketch, name: &str) -> usize {
    sk.motions.iter().position(|m| m.name == name).unwrap_or_else(|| panic!("no motion `{name}`"))
}

/// A turn of the plane by `a` radians about `c`, applied to `p`.
fn turn(c: [f64; 2], a: f64, p: [f64; 2]) -> [f64; 2] {
    let (s, k) = a.sin_cos();
    let d = [p[0] - c[0], p[1] - c[1]];
    [c[0] + k * d[0] - s * d[1], c[1] + s * d[0] + k * d[1]]
}

/// The planet's point at the roll `τ`, by hand: `R(τ)·(c + R(−3τ)(x − c))`.
fn planet_at(tau: f64) -> [f64; 3] {
    let p = turn([0.0, 0.0], tau, turn([30.0, 0.0], -3.0 * tau, [40.0, 0.0]));
    [p[0], p[1], 0.0]
}

/// **A planet's point carried by its chain is the closed form**, read two ways: the chain's pose
/// sampled after the solve (`Family::at`), and a carried point (`CarriedPoint`, the rows the
/// solve holds) pinned at a roll — at 60° the plan's (10, 17.3205).
#[test]
fn a_planets_point_on_its_arm_is_the_product_of_the_two_turns() {
    let mut e = build(PLANET);
    let sk = &mut e.sketch;
    let (arm, planet) = (index(sk, "arm"), index(sk, "planet"));
    let chain = sk.chain(arm, planet, "carried").unwrap();
    assert!(matches!(sk.motions[chain].def, MotionDef::Relative { .. }), "two centres do not fold");
    let x = e.map.ent_named("x").unwrap().i();
    let family = Family::read(sk, chain).unwrap();
    for deg in [-170.0, -60.0, 0.0, 25.0, 60.0, 135.0] {
        let tau = f64::to_radians(deg);
        let got = family.at(tau).unwrap().point(sk.world_point(x));
        assert!(norm(sub(got, planet_at(tau))) < 1e-12, "τ = {deg}: {got:?} against {:?}", planet_at(tau));
    }
    let want = planet_at(f64::to_radians(60.0));
    assert!((want[0] - 10.0).abs() < 1e-9 && (want[1] - 17.320508075688775).abs() < 1e-9, "{want:?}");
    // held by the solve: X carried from x at 60°
    let big = e.map.ent_named("X").unwrap();
    sk.add(Constraint::new(CKind::CarriedPoint, vec![Arg::Ent(big), Arg::Ent(EntRef::point(x)),
        Arg::Ent(EntRef::new(EntKind::Motion, chain)), Arg::Seed { value: 60.0, pinned: true }]));
    assert!(solve::solve(sk, Default::default()).success);
    let got = sk.world_point(big.i());
    assert!(norm(sub(got, want)) < 1e-9, "{got:?} against {want:?}");
}

/// **Three chained turns are their product**: `A·(B·C)` and `(A·B)·C` both, against the three
/// turns multiplied by hand, so a chain of a chain reads through the inverse of a relative.
#[test]
fn three_chained_turns_are_the_hand_product() {
    let src = "\
unit mm
use std
in std.top {
  a := point
  fix((0, 0)) a
  b := point
  fix((20, 5)) b
  c := point
  fix((35, -10)) c
  x := point
  fix((42, -3)) x
}
first := frame
std.Spin(first, about: a, on: std.top, ratio: 0.5)
second := frame
std.Spin(second, about: b, on: std.top, ratio: 2)
third := frame
std.Spin(third, about: c, on: std.top, ratio: -1.5)
";
    let mut e = build(src);
    let sk = &mut e.sketch;
    let (a, b, c) = (index(sk, "first"), index(sk, "second"), index(sk, "third"));
    let bc = sk.chain(b, c, "bc").unwrap();
    let right = sk.chain(a, bc, "a(bc)").unwrap();
    let ab = sk.chain(a, b, "ab").unwrap();
    let left = sk.chain(ab, c, "(ab)c").unwrap();
    let x = sk.world_point(e.map.ent_named("x").unwrap().i());
    let hand = |tau: f64| {
        let p = turn([35.0, -10.0], -1.5 * tau, [x[0], x[1]]);
        let p = turn([20.0, 5.0], 2.0 * tau, p);
        let p = turn([0.0, 0.0], 0.5 * tau, p);
        [p[0], p[1], 0.0]
    };
    for m in [right, left] {
        let family = Family::read(sk, m).unwrap();
        for deg in [-90.0, -20.0, 0.0, 33.0, 180.0] {
            let tau = f64::to_radians(deg);
            let moving = family.at(tau).unwrap();
            let got = moving.point(x);
            assert!(norm(sub(got, hand(tau))) < 1e-11, "{}: τ = {deg}: {got:?} against {:?}",
                    sk.motions[m].name, hand(tau));
            // the pose alone agrees with the pose and its rate
            let pose = family.pose_at(tau).unwrap().point(x);
            assert!(norm(sub(pose, got)) < 1e-11);
            // and the rate is the hand product's, by central differences
            let h = 1e-6;
            let fd: [f64; 3] = std::array::from_fn(|k| (hand(tau + h)[k] - hand(tau - h)[k]) / (2.0 * h));
            let rate = moving.velocity(x);
            assert!(norm(sub(rate, fd)) < 1e-6, "τ = {deg}: rate {rate:?} against {fd:?}");
        }
    }
}

/// **A wheel turning on a slide traces the cycloid**: a slide along `std.x` by `2πr` a turn
/// carrying a wheel of radius `r` turning back about its centre, so it rolls without slipping, and
/// its point at the ground is `(r(τ − sin τ), r(1 − cos τ))`.
#[test]
fn a_turn_on_a_slide_is_the_cycloid() {
    let src = "\
unit mm
use std
in std.top {
  g0 := point
  fix((0, 0)) g0
  g1 := point
  fix((10, 0)) g1
  ground := line(g0, g1)
  hub := point
  fix((0, 8)) hub
  rim := point
  fix((0, 0)) rim
}
slide := frame
std.Slide(slide, along: ground, advance: 2 * pi * 8)
roll := frame
std.Spin(roll, about: hub, on: std.top, ratio: -1)
";
    let mut e = build(src);
    let sk = &mut e.sketch;
    let (slide, roll) = (index(sk, "slide"), index(sk, "roll"));
    let chain = sk.chain(slide, roll, "wheel").unwrap();
    let rim = sk.world_point(e.map.ent_named("rim").unwrap().i());
    let family = Family::read(sk, chain).unwrap();
    let r = 8.0;
    for deg in [-200.0, -45.0, 0.0, 90.0, 180.0, 300.0] {
        let tau = f64::to_radians(deg);
        let got = family.at(tau).unwrap().point(rim);
        let want = [r * (tau - tau.sin()), r * (1.0 - tau.cos()), 0.0];
        assert!(norm(sub(got, want)) < 1e-11, "τ = {deg}: {got:?} against {want:?}");
    }
}

/// **Two joints about one axis fold to one**: a screw chained on a screw about the same line reads
/// as a single screw (`Family::screw`) whose ratio and advance are the two added — a gear train on
/// one shaft is the turn the generating classes admit — and two turns about one centre fold too.
/// A chain over two lines is a relative, and no screw.
#[test]
fn coaxial_joints_fold_to_one_screw() {
    let src = "\
unit mm
use std
o := point hint((1, 2, 3))
z := point hint((2, 4, 13))
fix((1, 2, 3)) o
fix((2, 4, 13)) z
shaft := line(o, z)
w := point hint((5, 0, 0))
fix((5, 0, 0)) w
other := line(o, w)
in std.top {
  c := point
  fix((3, 4)) c
}
lead := frame
std.Turn(lead, about: shaft, ratio: 2, advance: 3mm)
follow := frame
std.Turn(follow, about: shaft, ratio: -0.5, advance: 1mm)
skew := frame
std.Turn(skew, about: other, ratio: 1, advance: 1mm)
t1 := frame
std.Spin(t1, about: c, on: std.top, ratio: 1)
t2 := frame
std.Spin(t2, about: c, on: std.top, ratio: 2)
";
    let mut e = build(src);
    let sk = &mut e.sketch;
    let (lead, follow, skew) = (index(sk, "lead"), index(sk, "follow"), index(sk, "skew"));
    let n = sk.motions.len();
    let folded = sk.chain(lead, follow, "train").unwrap();
    assert_eq!(sk.motions.len(), n + 1, "a fold mints no inverse");
    let screw = Family::read(sk, folded).unwrap().screw().expect("one screw");
    assert!((screw.ratio - 1.5).abs() < 1e-15 && (screw.advance - 4.0).abs() < 1e-15, "{screw:?}");
    // the fold is the chain it stands for: the same pose as the relative over the inverse
    let separate = {
        let inverse = sk.motions.len();
        sk.motions.push(gcs_core::model::MotionE { def: MotionDef::Inverse { of: lead as u32 },
            name: "lead⁻".into(), class: Default::default(), numbers: None });
        sk.motions.push(gcs_core::model::MotionE {
            def: MotionDef::Relative { source: follow as u32, observer: inverse as u32 },
            name: "unfolded".into(), class: Default::default(), numbers: None });
        sk.motions.len() - 1
    };
    let (a, b) = (Family::read(sk, folded).unwrap(), Family::read(sk, separate).unwrap());
    assert!(b.screw().is_none(), "a relative is no single screw");
    let p = [7.0, -3.0, 5.0];
    for deg in [-120.0, 0.0, 47.0, 400.0] {
        let tau = f64::to_radians(deg);
        let (x, y) = (a.at(tau).unwrap().point(p), b.at(tau).unwrap().point(p));
        assert!(norm(sub(x, y)) < 1e-11, "τ = {deg}: {x:?} against {y:?}");
    }
    let crossed = sk.chain(lead, skew, "crossed").unwrap();
    assert!(Family::read(sk, crossed).unwrap().screw().is_none());
    let (t1, t2) = (index(sk, "t1"), index(sk, "t2"));
    let turns = sk.chain(t1, t2, "turns").unwrap();
    match sk.motions[turns].def {
        MotionDef::Turn { ratio, .. } => {
            assert!((ratio - 3.0).abs() < 1e-15);
        }
        ref d => panic!("two turns about one centre fold to a turn: {d:?}"),
    }
}

/// **The height check counts the minted nodes**: a chain of a chain … 70 deep is refused by the
/// reader, naming the limit, while one 30 deep reads.
#[test]
fn a_chain_past_sixty_four_levels_is_refused() {
    let mut e = build(PLANET);
    let sk = &mut e.sketch;
    let (arm, planet) = (index(sk, "arm"), index(sk, "planet"));
    let mut top = planet;
    let mut at_thirty = None;
    for k in 0..70 {
        top = sk.chain(arm, top, &format!("c{k}")).unwrap();
        if k == 30 {
            at_thirty = Some(top);
        }
    }
    assert!(Family::read(sk, at_thirty.unwrap()).is_ok());
    let err = Family::read(sk, top).unwrap_err();
    assert!(err.contains("64 levels"), "{err}");
}

// -- stage 2: one kind, and the bare frame --------------------------------------------------------

/// A frame stood off the world by `fix`: `u` along `y`, `v` along `z` — so up is `z` and out is
/// `x` — at (10, 20, 30).
const STOOD: &str = "\
unit mm
use std
f := frame
fix(dir == (0, 1, 0)) f.u
fix(dir == (0, 0, 1)) f.v
fix(origin == (10, 20, 30)) f
";

/// **A point in a spatial frame has three numbers, read along its axes**: `x` along `f.u`, `y`
/// up and `z` out, standing at `o + x·u + y·up + z·out` — and a relation in space between it and a
/// point in space reads its lift there.
#[test]
fn a_point_in_a_spatial_frame_lifts_through_its_chart() {
    let src = format!("{STOOD}\
in f {{
  p := point
  fix((1, 2, 3)) p
}}
q := point hint((0, 0, 0))
q coincident p
in f {{
  r := point hint(at: f, (4, 5, 6))
}}
s := point hint(at: f, (4, 5, 6))
");
    let e = crate::common::text::solved(&src);
    let p = crate::common::at(&e, "p");
    assert!(norm(sub(p, [13.0, 21.0, 32.0])) < 1e-9, "{p:?}");
    let q = crate::common::at(&e, "q");
    assert!(norm(sub(q, p)) < 1e-9, "{q:?}");
    // its own numbers are the frame's coordinates, three of them
    let pt = &e.sketch.points[e.map.ent_named("p").unwrap().i()];
    assert!(pt.chart.is_some() && pt.plane.is_none() && pt.z.is_some());
    let v = |k: u32| e.sketch.params[k as usize].value;
    assert_eq!([v(pt.x), v(pt.y), v(pt.z.unwrap())], [1.0, 2.0, 3.0]);
    // a place in its coordinates seeds a point drawn in it by its own three numbers, and one in
    // space where the place stands
    let r = &e.sketch.points[e.map.ent_named("r").unwrap().i()];
    assert_eq!([v(r.x), v(r.y), v(r.z.unwrap())], [4.0, 5.0, 6.0]);
    let s = crate::common::at(&e, "s");
    assert!(norm(sub(s, [16.0, 24.0, 35.0])) < 1e-9, "{s:?}");
    // and the frame's origin is the point at (0, 0, 0) along them
    let o = crate::common::at(&e, "f.origin");
    assert!(norm(sub(o, [10.0, 20.0, 30.0])) < 1e-9, "{o:?}");
}

/// **A bare frame's axes are placed by relations**: named, `f.u` and `f.v` are axes of its own and
/// `f.origin` its origin, seven numbers like a plane's, held here by ordinary relations — its `u`
/// along a drawn line, its `v` along the world's `z`, its origin on a point — and a point drawn in
/// it stands where the relations put the frame: along `u`, up (`out × u`) and out (`u × v`).
#[test]
fn a_bare_frames_axes_are_placed_by_relations() {
    let src = "\
unit mm
use std
a := point
fix((5, 0, 0)) a
b := point
fix((8, 3, 4)) b
rail := line(a, b)
f := frame
f.origin coincident a
f.u parallel rail
f.v parallel std.z
in f {
  p := point
  fix((10, 1, 2)) p
}
";
    let mut e = crate::common::text::solved(src);
    let unit = |v: [f64; 3]| { let n = norm(v); v.map(|x| x / n) };
    let cross = gcs_core::space::cross;
    // the parallels leave each axis its sense: read the one the solve kept
    let pick = |name: &str, along: [f64; 3]| {
        let r = &e.sketch.axes[e.map.ent_named(name).unwrap().i()];
        let d: [f64; 3] = r.d.map(|q| e.sketch.params[q as usize].value);
        if gcs_core::space::dot(d, along) < 0.0 { along.map(|x| -x) } else { along }
    };
    let u = pick("f.u", unit([3.0, 3.0, 4.0]));
    let v = pick("f.v", [0.0, 0.0, 1.0]);
    let out = unit(cross(u, v));
    let up = cross(out, u);
    let want: [f64; 3] = std::array::from_fn(|k| [5.0, 0.0, 0.0][k] + 10.0 * u[k] + up[k] + 2.0 * out[k]);
    let p = crate::common::at(&e, "p");
    assert!(norm(sub(p, want)) < 1e-9, "{p:?} against {want:?}");
    assert_eq!(crate::common::dof(&mut e.sketch), 0, "the frame's seven numbers are held");
}

/// **A frame nothing reads owns nothing**: declared bare and never named, it mints no axis, no
/// place and no origin, so the document's unknowns are what they were without it; a point drawn in
/// it is a point in space, drawn in its parent's coordinates — the world's.
#[test]
fn an_unnamed_frame_owns_no_unknowns() {
    let with = "\
unit mm
f := frame
in f {
  p := point hint((1, 2, 3))
}
";
    let without = "\
unit mm
p := point hint((1, 2, 3))
";
    let (a, b) = (crate::common::text::solved(with), crate::common::text::solved(without));
    assert_eq!(a.sketch.params.len(), b.sketch.params.len());
    assert_eq!(a.sketch.axes.len(), b.sketch.axes.len());
    assert_eq!(a.sketch.constraints.len(), b.sketch.constraints.len());
    let f = a.map.ent_named("f").expect("the frame is named");
    assert_eq!(f.kind, EntKind::Frame);
    assert!(a.sketch.frames[f.i()].chart.is_none());
    assert_eq!(crate::common::at(&a, "p"), [1.0, 2.0, 3.0]);
}

/// **The words**: `frame` declares, `in (f := frame) { … }` declares in the header, a frame takes
/// no brackets and no seed, and a relation over a plane refuses a spatial frame, which is no set.
#[test]
fn a_frame_is_declared_bare_and_in_a_header() {
    let e = crate::common::text::solved("\
unit mm
in (g := frame) {
  p := point hint((1, 2, 3))
}
");
    assert_eq!(e.map.ent_named("g").unwrap().kind, EntKind::Frame);
    assert_eq!(crate::common::at(&e, "p"), [1.0, 2.0, 3.0]);
    for (src, says) in [
        ("f := frame(u: a)\n", "declared bare"),
        ("f := frame hint((1, 2, 3))\n", "no seed"),
    ] {
        let (_, errs) = crate::common::parse(&format!("unit mm\n{src}"));
        assert!(errs.iter().any(|e| e.message.contains(says)), "{src}: {:?}",
                errs.iter().map(|e| &e.message).collect::<Vec<_>>());
    }
}

// -- stage 3: joints as members, stated --------------------------------------------------------

/// The motion node frame `name`'s joint was lowered to.
fn joint_of(e: &gcs_core::program::Elaborated, name: &str) -> usize {
    let f = e.map.ent_named(name).unwrap_or_else(|| panic!("no frame `{name}`"));
    assert_eq!(f.kind, EntKind::Frame);
    let j = e.sketch.frames[f.i()].joint.as_ref()
        .unwrap_or_else(|| panic!("`{name}` has no joint"));
    j.motion.unwrap_or_else(|| panic!("`{name}`'s joint was not lowered")) as usize
}

/// **A joint's kind is read off the relations holding it**: `std.Turn` holds the axis on a line (a
/// turn about it, its numbers stated: a screw where it advances), `std.Spin` through a point of a
/// view square to it (a turn in that view about the point), `std.Slide` on a line with no ratio (a
/// slide) — the motions the readers built with the drawing take; an axis held by anything else is a
/// joint read after the solve, the axis where the solve leaves it.
#[test]
fn a_joints_kind_is_read_off_what_holds_it() {
    let src = "\
unit mm
use std
in std.top {
  o := point
  fix((0, 0)) o
  q := point
  fix((10, 0)) q
  rail := line(o, q)
}
a := point
fix((1, 2, 3)) a
b := point
fix((1, 2, 13)) b
shaft := line(a, b)
turned := frame
std.Turn(turned, about: shaft, ratio: 2)
screwed := frame
std.Turn(screwed, about: shaft, ratio: 1, advance: 4mm)
spun := frame
std.Spin(spun, about: o, on: std.top, ratio: -1)
slid := frame
std.Slide(slid, along: rail, advance: 6mm)
free := frame
fix(dir == (0, 1, 0), origin == (5, 0, 0)) free.axis
fix(ratio == 3, advance == 0mm) free
";
    let mut e = build(src);
    assert!(solve::solve(&mut e.sketch, Default::default()).success);
    let sk = &e.sketch;
    let def = |n: &str| sk.motions[joint_of(&e, n)].def.clone();
    assert!(matches!(def("turned"), MotionDef::Rotation { ratio, advance, .. }
        if ratio == 2.0 && advance == 0.0));
    assert!(matches!(def("screwed"), MotionDef::Rotation { ratio, advance, .. }
        if ratio == 1.0 && advance == 4.0));
    assert!(matches!(def("spun"), MotionDef::Turn { ratio, .. } if ratio == -1.0));
    assert!(matches!(def("slid"), MotionDef::Translation { advance, .. } if advance == 6.0));
    assert!(matches!(def("free"), MotionDef::Joint { ratio, .. } if ratio == 3.0));
    // the screw is a screw (`Family::screw`, the generating classes'), and the free joint turns
    // about its axis where it stands: (5, 0, 1) a quarter turn on over three radians a radian
    assert!(Family::read(sk, joint_of(&e, "screwed")).unwrap().screw().is_some());
    let free = Family::read(sk, joint_of(&e, "free")).unwrap();
    let got = free.at(f64::to_radians(30.0)).unwrap().point([5.0, 0.0, 1.0]);
    assert!(norm(sub(got, [6.0, 0.0, 0.0])) < 1e-12, "{got:?}");
    // the joint's members are held: no freedom of theirs
    let mut sk2 = e.sketch.clone();
    assert_eq!(gcs_core::diagnose::diagnose(&mut sk2, Default::default()).dof, 0);
}

/// **`seen_from:` is the relative**: a point carried by one frame seen from another stands where
/// the observer's inverse takes the carried point — `motion(a, relative_to: b)` said where it is
/// read — and a frame at rest observes as the world does.
#[test]
fn a_reading_seen_from_a_frame_is_the_relative() {
    let src = "\
unit mm
use std
in std.top {
  o := point
  fix((0, 0)) o
  c := point
  fix((20, 0)) c
  x := point
  fix((30, 0)) x
  X := point hint((25, 5))
  Y := point hint((25, 5))
  Z := point hint((25, 5))
}
ring := frame
std.Spin(ring, about: o, on: std.top, ratio: 1)
gear := frame
std.Spin(gear, about: c, on: std.top, ratio: -2)
rest := frame
X coincident(under: gear, seen_from: ring, t == 40) x
Y coincident(under: gear, seen_from: rest, t == 40) x
Z coincident(under: gear, t == 40) x
";
    let mut e = build(src);
    assert!(solve::solve(&mut e.sketch, Default::default()).success);
    let tau = f64::to_radians(40.0);
    // R_o(−τ)·R_c(−2τ)·x
    let p = turn([0.0, 0.0], -tau, turn([20.0, 0.0], -2.0 * tau, [30.0, 0.0]));
    let x = crate::common::at(&e, "X");
    assert!(norm(sub(x, [p[0], p[1], 0.0])) < 1e-9, "{x:?} against {p:?}");
    let (y, z) = (crate::common::at(&e, "Y"), crate::common::at(&e, "Z"));
    let q = turn([20.0, 0.0], -2.0 * tau, [30.0, 0.0]);
    assert!(norm(sub(y, [q[0], q[1], 0.0])) < 1e-9 && norm(sub(z, y)) < 1e-12, "{y:?} {z:?}");
}

/// **A component's numbers may default, and a frame formal left out is the world**: `std.Turn`
/// with no `advance:` turns and screws nothing, and `std.Screw` with no `seen_from:` reads the
/// frame's own motion.
#[test]
fn a_left_out_advance_is_none_and_a_left_out_observer_is_the_world() {
    let src = "\
unit mm
use std
a := point
fix((0, 0, 0)) a
b := point
fix((0, 0, 10)) b
shaft := line(a, b)
spin := frame
std.Turn(spin, about: shaft, ratio: 2, advance: 3mm)
isa := axis hint(dir: (0.1, 0.1, 1))
s := std.Screw(isa, spin)
";
    let mut e = build(src);
    assert!(solve::solve(&mut e.sketch, Default::default()).success);
    // the pitch: 3 mm a turn at two radians a radian, `3 / 2π` a radian of the roll
    let d: [f64; 3] = e.sketch.axes[e.map.ent_named("isa").unwrap().i()].d
        .map(|q| e.sketch.params[q as usize].value);
    assert!(norm(gcs_core::space::cross(d, [0.0, 0.0, 1.0])) < 1e-9, "{d:?}");
    let plain = build("unit mm\nuse std\na := point\nfix((0, 0, 0)) a\nb := point\n\
                       fix((0, 0, 10)) b\nshaft := line(a, b)\nturn := frame\n\
                       std.Turn(turn, about: shaft, ratio: 1)\n");
    let def = plain.sketch.motions[joint_of(&plain, "turn")].def.clone();
    assert!(matches!(def, MotionDef::Rotation { advance, .. } if advance == 0.0), "{def:?}");
}

/// **The retired spellings are refused saying what each is now**, and a frame read as a motion
/// with no joint says how to turn it.
#[test]
fn the_motion_spellings_are_refused_with_what_they_became() {
    for (src, says) in [
        ("o := point\nm := motion(about: o, ratio: 2)\n", "f turns(r) l"),
        ("l := line\nm := motion(along: l, advance: 2mm)\n", "f slides(a) l"),
        ("m := motion(a, relative_to: b)\n", "seen_from:"),
        ("o := point\nm := motion(about: o, phase: 10deg)\n", "roll"),
        ("component C(m: motion) {\n  p := point\n}\n", "`m: frame`"),
    ] {
        let (_, errs) = crate::common::parse(&format!("unit mm\n{src}"));
        assert!(errs.iter().any(|e| e.message.contains(says)), "{src}: {:?}",
                errs.iter().map(|e| &e.message).collect::<Vec<_>>());
    }
    let (prog, errs) = crate::common::parse(
        "unit mm\nuse std\nin std.top {\n  x := point\n  X := point\n}\n\
         still := frame\nX coincident(under: still, t == 10) x\n");
    assert!(errs.is_empty(), "{errs:?}");
    let e = gcs_core::program::elaborate(&prog);
    assert!(e.errors().any(|d| d.message.contains("does not move")),
            "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
    // a joint's ratio pinned to a measurement of the drawing: none is read before the solve
    let (prog, _) = crate::common::parse(
        "unit mm\nuse std\na := point\nb := point\nl := line(a, b)\nf := frame\n\
         std.Turn(f, about: l, ratio: length(l) / 1mm)\n");
    let e = gcs_core::program::elaborate(&prog);
    assert!(e.errors().any(|d| d.code.as_str() == "E107"),
            "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
}

/* -- stage 4: riding -------------------------------------------------------------------------- */

/// **A planet rides on its arm** (#383 stage 4): drawn in nested `in` blocks, `x` rides on the
/// planet and the planet, declared in the arm's block, on the arm — so `X coincident(t == τ) x`,
/// `X` riding on nothing, is `x` carried by the chain `arm·planet` with no `under:` written, the
/// closed form `R(τ)·(c + R(−3τ)(x − c))`; and `under: planet` of a point at rest is the same
/// chain, since the planet's motion is its joint after its arm's.
#[test]
fn a_planet_drawn_in_its_arm_rides_on_both() {
    let src = "\
unit mm
use std
in std.top {
  o := point
  fix((0, 0)) o
  X := point hint((10, 17))
  Y := point hint((10, 17))
  still := point
  fix((40, 0)) still
}
in (arm := frame) {
  in std.top {
    c := point
    fix((30, 0)) c
  }
  in (planet := frame) {
    in std.top {
      x := point
      fix((40, 0)) x
    }
  }
}
std.Spin(arm, about: o, on: std.top, ratio: 1)
std.Spin(planet, about: c, on: std.top, ratio: -3)
X coincident(t == 60deg) x
Y coincident(under: planet, t == 60deg) still
";
    let mut e = build(src);
    assert!(e.diags.iter().all(|d| d.code.as_str() != "W115"), "one chart on each path");
    assert!(solve::solve(&mut e.sketch, Default::default()).success);
    let want = planet_at(f64::to_radians(60.0));
    for name in ["X", "Y"] {
        let got = crate::common::at(&e, name);
        assert!(norm(sub(got, want)) < 1e-9, "{name}: {got:?} against {want:?}");
    }
    // at rest the riding point is drawn where it stands, as every relation naming no roll reads
    assert!(norm(sub(crate::common::at(&e, "x"), [40.0, 0.0, 0.0])) < 1e-12);
    let planet = e.map.ent_named("planet").unwrap();
    let arm = e.map.ent_named("arm").unwrap();
    assert_eq!(e.sketch.frames[planet.i()].rides, vec![arm.i() as u32]);
    assert_eq!(e.sketch.carriers(e.map.ent_named("x").unwrap()).unwrap(),
               vec![arm.i() as u32, planet.i() as u32]);
}

/// The primer's rack (§2.11) with the rack's flank drawn in the rack: the profile it cuts, read
/// `seen_from: blank` with `under:` left out, is the one `under: rack` cuts from the same flank
/// drawn at rest — the slope `g coincident cut` decides the same.
#[test]
fn a_rack_riding_its_frame_cuts_what_the_reading_under_it_cuts() {
    let views = "\
unit mm
use std
in std.front {
  o := point
  fix((0, 0)) o
  rk0 := point
  rk1 := point hint((30, 10))
  fix((30, 0)) rk0
  fix((30, 10)) rk1
  slide := line(rk0, rk1)
  g := point
  fix((31.5, -3)) g
}
rack := frame
blank := frame
std.Spin(blank, about: o, on: std.front, ratio: 1)
std.Slide(rack, along: slide, advance: 2 * pi * 30)
";
    let flank = "\
    f0 := point hint((30, 5))
    f1 := point hint((39.4, 8.4))
    flank := line(f0, f1)
    fix((30, 5)) f0
    f0 distance(10) f1
";
    let riding = format!("{}in rack {{\n  in std.front {{\n{flank}  }}\n}}\n\
        cut := envelope(flank over t in (-25deg, 25deg), seen_from: blank)\n\
        g coincident cut hint(t: 5)\n", views);
    let under = format!("{}in std.front {{\n{flank}}}\n\
        cut := envelope(flank over t in (-25deg, 25deg), under: rack, seen_from: blank)\n\
        g coincident cut hint(t: 5)\n", views);
    let (mut a, mut b) = (build(&riding), build(&under));
    assert!(solve::solve(&mut a.sketch, Default::default()).success);
    assert!(solve::solve(&mut b.sketch, Default::default()).success);
    let (fa, fb) = (crate::common::at(&a, "f1"), crate::common::at(&b, "f1"));
    assert!(norm(sub(fa, fb)) < 1e-9, "{fa:?} against {fb:?}");
    // the profile rides on its observer
    let cut = a.map.ent_named("cut").unwrap();
    let blank = a.map.ent_named("blank").unwrap();
    assert_eq!(a.sketch.carriers(cut).unwrap(), vec![blank.i() as u32]);
}

/// **A solid riding on a frame is moved by it with no `under:`**: placed at a roll and swept
/// over an interval, it is the solid the same tool drawn at rest makes `under:` the frame.
#[test]
fn a_tool_riding_its_frame_is_placed_and_swept_as_under_it() {
    let head = "\
unit mm
use std
in std.front {
  o := point
  z := point
  fix((0, 0)) o
  fix((0, 1)) z
  ax := line(o, z)
}
turn := frame
std.Turn(turn, about: ax, ratio: 1)
";
    let tool = "\
    a := point
    b := point
    c := point
    d := point
    fix((2, 0)) a
    fix((4, 0)) b
    fix((4, 2)) c
    fix((2, 2)) d
    ab := line(a, b)
    bc := line(b, c)
    cd := line(c, d)
    da := line(d, a)
    profile := face(ab, bc, cd, da)
";
    let riding = format!("{head}in turn {{\n  in std.front {{\n{tool}  }}\n}}\n\
        stock := solid(profile, from: 0mm, to: 2mm)\n\
        placed := stock at 30deg\n\
        swept := sweep(stock over t in (-20deg, 20deg))\n");
    let under = format!("{head}in std.front {{\n{tool}}}\n\
        stock := solid(profile, from: 0mm, to: 2mm)\n\
        placed := stock at(under: turn) 30deg\n\
        swept := sweep(stock over t in (-20deg, 20deg), under: turn)\n");
    let (mut a, mut b) = (build(&riding), build(&under));
    assert!(solve::solve(&mut a.sketch, Default::default()).success);
    assert!(solve::solve(&mut b.sketch, Default::default()).success);
    let volume = |e: &gcs_core::program::Elaborated, name: &str| {
        let i = e.sketch.solids.iter().position(|s| s.name == name).unwrap();
        e.sketch.evaluated_solid(i, gcs_core::solid::ApproximationPolicy::Report).unwrap().volume()
    };
    for name in ["placed", "swept"] {
        let (va, vb) = (volume(&a, name), volume(&b, name));
        assert!((va - vb).abs() <= 1e-9 * vb, "{name}: {va} against {vb}");
    }
    assert!((volume(&a, "placed") - 8.0).abs() < 1e-9);
}

/// **What riding refuses**, each saying why: `under:` naming the frame of a point that rides
/// already, a reading of a line whose ends ride on different frames, and two frames with axes
/// nested — the inner draws, which is warned.
#[test]
fn riding_refuses_what_it_cannot_mean() {
    let head = "\
unit mm
use std
in std.top {
  o := point
  fix((0, 0)) o
  c := point
  fix((10, 0)) c
  X := point hint((1, 1))
}
arm := frame
std.Spin(arm, about: o, on: std.top, ratio: 1)
other := frame
std.Spin(other, about: c, on: std.top, ratio: 2)
";
    let errors = |src: &str| -> Vec<String> {
        let (prog, errs) = crate::common::parse(src);
        assert!(errs.is_empty(), "{errs:?}");
        gcs_core::program::elaborate(&prog).errors().map(|d| d.message.clone()).collect()
    };
    let under = format!("{head}in arm {{\n  in std.top {{\n    x := point\n    \
        fix((5, 0)) x\n  }}\n}}\n\
        X coincident(under: arm, t == 10deg) x\n");
    let errs = errors(&under);
    assert!(errs.iter().any(|m| m.contains("rides on `arm` already")), "{errs:?}");
    let mixed = format!("{head}in arm {{\n  in std.top {{\n    p := point\n    \
        fix((5, 0)) p\n  }}\n}}\n\
        in other {{\n  in std.top {{\n    q := point\n    fix((5, 3)) q\n  }}\n}}\n\
        in std.top {{\n  l := line(p, q)\n}}\n\
        e := envelope(l over t in (-10deg, 10deg))\n");
    let errs = errors(&mixed);
    assert!(errs.iter().any(|m| m.contains("rides on `arm`") && m.contains("on `other`")),
            "{errs:?}");
    let (prog, _) = crate::common::parse("unit mm\nuse std\nin std.front {\n  in std.top {\n    \
        p := point\n  }\n}\n");
    let e = gcs_core::program::elaborate(&prog);
    assert!(e.ok());
    assert!(e.diags.iter().any(|d| d.code.as_str() == "W115" && d.message.contains("`std.top`")),
            "{:?}", e.diags.iter().map(|d| &d.message).collect::<Vec<_>>());
    assert_eq!(e.sketch.plane_of(e.map.ent_named("p").unwrap().i()),
               Some(e.map.ent_named("std.top").unwrap().i()));
}

/* -- stage 5: joints in the solve ------------------------------------------------------------- */

/// A frame's joint numbers as the solve leaves them.
fn numbers(e: &gcs_core::program::Elaborated, name: &str) -> (f64, f64) {
    let f = e.map.ent_named(name).unwrap_or_else(|| panic!("no frame `{name}`"));
    let j = e.sketch.frames[f.i()].joint.as_ref().unwrap();
    (e.sketch.params[j.ratio as usize].value, e.sketch.params[j.advance as usize].value)
}

/// Two pitch circles about `o` and `c`, the wheel's radius `R` and the pinion's `r`, touching at
/// the pitch point `P` on the line of centres; they roll along the view's `v`, square to it.
fn gears(c: &str, pinion: &str) -> String {
    format!("\
unit mm
use std
in std.top {{
  o := point
  fix((0, 0)) o
  c := point {c}
  P := point
  fix((30, 0)) P
}}
wheel := frame
std.Spin(wheel, about: o, on: std.top, ratio: 1)
pinion := frame
{pinion}
std.Rolls(P, pinion, wheel, along: std.top.v)
")
}

/// **A joint's ratio left to the solve is found by what holds it** (#383 stage 5): two pitch
/// circles rolling at their pitch point without slipping (`std.Rolls`) turn at `−R/r` — the wheel
/// 30 out at a radian a radian, the pinion 10 out three the other way.
#[test]
fn a_gear_pairs_ratio_is_solved_from_rolling() {
    let mut e = build(&gears("hint((40, 0))\n  fix((40, 0)) c",
                             "std.Spin(pinion, about: c, on: std.top)"));
    let r = solve::solve(&mut e.sketch, Default::default());
    assert!(r.success, "{r:?}");
    let (ratio, advance) = numbers(&e, "pinion");
    assert!((ratio + 3.0).abs() < 1e-9 && advance == 0.0, "{ratio}, {advance}");
    // and a placement under the pinion reads the ratio solved
    let m = e.sketch.frames[e.map.ent_named("pinion").unwrap().i()].joint.as_ref().unwrap().motion;
    let (r, _) = e.sketch.motions[m.unwrap() as usize].rotation(&e.sketch).unwrap();
    assert!((r + 3.0).abs() < 1e-9, "{r}");
}

/// **And a ratio stated places what it is stated of**: the pinion turning three times as fast the
/// other way, its centre left free on the line of centres, solves 10 out past the pitch point.
#[test]
fn a_stated_ratio_solves_the_centre_distance() {
    let mut e = build(&gears("hint((36, 0))\n  centres := line(o, P)\n  c coincident centres",
                             "std.Spin(pinion, about: c, on: std.top, ratio: -3)"));
    let r = solve::solve(&mut e.sketch, Default::default());
    assert!(r.success, "{r:?}");
    let c = crate::common::at(&e, "c");
    assert!(norm(sub(c, [40.0, 0.0, 0.0])) < 1e-9, "{c:?}");
}

/// **A joint left free is reported free, with its numbers** (stage 5): a frame whose axis only a
/// claim reads, place and direction, has the axis's four freedoms and its ratio and advance — six
/// — and the diagnosis names each.  Its numbers stated, `std.Turn` with no ratio leaves the ratio
/// alone free.
#[test]
fn a_free_joint_is_six_numbers_the_diagnosis_reports() {
    use gcs_core::diagnose::{diagnose, DiagnoseOptions};
    let head = "unit mm\nuse std\na := point\nb := point\nfix((0, 0, 0)) a\nfix((0, 0, 10)) b\n\
                l := line(a, b)\nf := frame\n";
    let names = |e: &mut gcs_core::program::Elaborated| {
        let d = diagnose(&mut e.sketch, DiagnoseOptions::default());
        let mut v: Vec<String> = d.under_params.iter()
            .map(|&p| e.sketch.params[p as usize].name.clone()).collect();
        v.sort();
        (d.dof, v)
    };
    let mut e = build(&format!("{head}claim l coincident f.axis\n"));
    let (dof, free) = names(&mut e);
    assert_eq!(dof, 6, "{free:?}");
    assert!(["f.ratio", "f.advance"].iter().all(|n| free.iter().any(|m| m == n)), "{free:?}");
    assert!(free.iter().filter(|n| n.starts_with("f.axis")).count() >= 4, "{free:?}");
    let mut e = build(&format!("{head}std.Turn(f, about: l)\n"));
    let (dof, free) = names(&mut e);
    assert_eq!((dof, free), (1, vec!["f.ratio".to_string()]));
}
