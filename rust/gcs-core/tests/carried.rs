//! **A point as a moving frame sees it** (#326, R4): `X coincident(under: m, t == τ) x` holds `X`
//! where the frame's pose at the roll `τ` carries `x`, `X = Pose(τ)·x`.  Under a frame seen from a
//! turning one (`under: m, seen_from: g`), at `τ = −φ`, it is a point of the fixed frame as a
//! member turned `φ` sees it.  Each
//! is held here to closed forms and to what the solve already knows of a motion apart from it: an
//! epicycloid is a rolling circle's point seen by the fixed one, which is also the planar envelope
//! of that point under the roll (§6.15.1), and a point carried by a screw about a skew line is on
//! the helix about that line.
use crate::common::build;
use gcs_core::constraints::CKind;
use gcs_core::solve;
use gcs_core::space::{add, cross, dot, norm, scale, sub, unit};

/// A circle of radius 20 rolling round the outside of one of radius 40, drawn in the top view: the
/// fixed circle turns about `o` with the roll, the rolling one about `c` twice as fast the other
/// way, and the rolling circle seen from the fixed one is `under: rolling_turn, seen_from:
/// fixed_turn`.  `x` is the rolling circle's
/// point where the two touch at the start; `path`, its envelope under the roll, is its path there.
const ROLLING: &str = "\
unit mm
use std
in std.top {
  o := point
  fix((0, 0)) o
  c := point
  fix((60, 0)) c
  x := point
  fix((40, 0)) x
}
fixed_turn := frame
std.Spin(fixed_turn, about: o, on: std.top, ratio: 1)
rolling_turn := frame
std.Spin(rolling_turn, about: c, on: std.top, ratio: -2)
path := envelope(x over t in (-180deg, 180deg), under: rolling_turn, seen_from: fixed_turn)
";

/// The epicycloid of a circle of radius `b` rolling round one of radius `a`, at `θ`: the
/// textbook's.
fn epicycloid(a: f64, b: f64, theta: f64) -> [f64; 2] {
    let k = (a + b) / b;
    [(a + b) * theta.cos() - b * (k * theta).cos(), (a + b) * theta.sin() - b * (k * theta).sin()]
}

fn solved(src: &str) -> gcs_core::program::Elaborated {
    let mut e = build(src);
    let r = solve::solve(&mut e.sketch, Default::default());
    assert!(r.success, "{r:?}");
    e
}

fn at(e: &gcs_core::program::Elaborated, name: &str) -> [f64; 3] {
    e.sketch.world_point(e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`")).i())
}

/// **A rolling circle's point, seen by the fixed one, is on the epicycloid.**  Carried by the roll
/// at a stated `τ`, the point is the textbook's epicycloid at `θ = −τ` — the fixed circle turns
/// with the roll, so the carrier turns back against it — and a point carried at a roll left to
/// the solve, held to the planar envelope of `x` under the same motion and 50 from the fixed
/// circle's centre, is where the epicycloid is that far out: the two readings of one motion agree.
#[test]
fn a_rolling_circles_point_seen_by_the_fixed_one_is_on_the_epicycloid() {
    let rolls = [-150.0, -60.0, -15.0, 0.0, 30.0, 90.0, 170.0];
    // wherever Y is hinted below the line of centres, its roll starts where the motion carries x
    // nearest it (`infer::carried_roll`), not at 0, halfway between the two places 50 out
    for hint in ["(49, -7)", "(50, -6)", "(45, -12)", "(30, -38)"] {
        let mut src = ROLLING.to_string();
        for (k, t) in rolls.iter().enumerate() {
            src += &format!("in std.top {{\n  X{k} := point hint((50, 0))\n}}\n\
                             X{k} coincident(under: rolling_turn, seen_from: fixed_turn, \
                             t == {t}) x\n");
        }
        src += &format!("in std.top {{\n  Y := point hint({hint})\n}}\n\
                         Y coincident(under: rolling_turn, seen_from: fixed_turn) x\n\
                         Y distance(50mm) o\nY coincident path\n");
        let y = at(&solved(&src), "Y");
        assert!(norm(sub(y, [49.5763, -6.49519, 0.0])) < 1e-3, "hint {hint}: {y:?}");
    }
    let mut src = ROLLING.to_string();
    for (k, t) in rolls.iter().enumerate() {
        src += &format!("in std.top {{\n  X{k} := point hint((50, 0))\n}}\n\
                         X{k} coincident(under: rolling_turn, seen_from: fixed_turn, \
                         t == {t}) x\n");
    }
    src += "in std.top {\n  Y := point hint((49, -7))\n}\n\
            Y coincident(under: rolling_turn, seen_from: fixed_turn) x\n\
            Y distance(50mm) o\nY coincident path\n";
    let e = solved(&src);
    for (k, t) in rolls.iter().enumerate() {
        let x = at(&e, &format!("X{k}"));
        let want = epicycloid(40.0, 20.0, -f64::to_radians(*t));
        assert!(norm(sub(x, [want[0], want[1], 0.0])) < 1e-9, "τ = {t}: {x:?} against {want:?}");
    }
    let y = at(&e, "Y");
    assert!((norm(y) - 50.0).abs() < 1e-9, "{y:?}");
    // the roll the carried point solved to is where the epicycloid is that point
    let c = e.sketch.constraints.iter().filter(|c| c.kind == CKind::CarriedPoint).last().unwrap();
    let tau = e.sketch.params[c.args[3].param() as usize].value;
    let want = epicycloid(40.0, 20.0, -tau.to_radians());
    assert!(norm(sub(y, [want[0], want[1], 0.0])) < 1e-9, "τ {tau}: {y:?} against {want:?}");
}

/// `x` turned `angle` about the line through `o` along the unit `d`.
fn turned(x: [f64; 3], o: [f64; 3], d: [f64; 3], angle: f64) -> [f64; 3] {
    let w = sub(x, o);
    let (s, c) = angle.sin_cos();
    add(o, add(add(scale(w, c), scale(cross(d, w), s)), scale(d, dot(d, w) * (1.0 - c))))
}

/// **A point carried by a screw about a skew line is on its helix**, in space: turned `τ` about
/// the line and slid `τ/360°` of the lead along it — and, seen from a turn about another skew
/// line, carried by that turn back as well.  Both read the poses alone.
#[test]
fn a_point_carried_by_a_screw_is_on_its_helix() {
    let src = "\
unit mm
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
use std
screw := frame
std.Turn(screw, about: ax, ratio: 1.5, advance: 7mm)
turn := frame
std.Turn(turn, about: bx, ratio: -0.4)
x := point hint((9, -4, 6))
fix((9, -4, 6)) x
P := point hint((9, -4, 6))
P coincident(under: screw, t == 40) x
Q := point hint((9, -4, 6))
Q coincident(under: screw, seen_from: turn, t == -25) x
";
    let e = solved(src);
    let (a0, d) = ([1.0, 2.0, 3.0], unit([3.0, 1.0, 8.0]));
    let (b0, e_) = ([-5.0, 0.0, 2.0], unit([0.0, 9.0, -2.0]));
    let x = [9.0, -4.0, 6.0];
    let screw = |x: [f64; 3], t: f64| {
        add(turned(x, a0, d, 1.5 * t.to_radians()), scale(d, 7.0 * t / 360.0))
    };
    let p = screw(x, 40.0);
    assert!(norm(sub(at(&e, "P"), p)) < 1e-9, "{:?} against {p:?}", at(&e, "P"));
    // on the helix: as far from the axis as `x`, and as far along it as the lead says
    let off = |y: [f64; 3]| { let w = sub(y, a0); norm(sub(w, scale(d, dot(w, d)))) };
    assert!((off(p) - off(x)).abs() < 1e-9);
    assert!((dot(sub(p, x), d) - 7.0 * 40.0 / 360.0).abs() < 1e-9);
    let q = turned(screw(x, -25.0), b0, e_, 0.4 * -25f64.to_radians());
    assert!(norm(sub(at(&e, "Q"), q)) < 1e-9, "{:?} against {q:?}", at(&e, "Q"));
}

/// The relation prints as it is written and reads back the same; `under:` names a frame, and
/// anything else there is refused.
#[test]
fn a_carried_point_prints_back_and_names_a_motion() {
    // the flat form prints no component instance: the joints are written out as `std.Spin`
    // writes them
    let spin = |f: &str, c: &str, r: i32| format!("{c} coincident {f}.axis\n\
        {f}.axis parallel std.top.normal\nfix(ratio == {r}, advance == 0mm) {f}\n");
    let src = format!("{ROLLING}in std.top {{\n  X := point hint((50, 0))\n}}\n\
                       X coincident(under: rolling_turn, seen_from: fixed_turn, t == 30) x\n")
        .replace("std.Spin(fixed_turn, about: o, on: std.top, ratio: 1)\n",
                 &spin("fixed_turn", "o", 1))
        .replace("std.Spin(rolling_turn, about: c, on: std.top, ratio: -2)\n",
                 &spin("rolling_turn", "c", -2))
        .replace("path := envelope(x over t in (-180deg, 180deg), under: rolling_turn, seen_from: fixed_turn)\n", "");
    let (mut prog, errs) = crate::common::parse(&src);
    assert!(errs.is_empty(), "{errs:?}");
    let first = gcs_core::program::elaborate(&prog);
    assert!(first.ok(), "{:?}", first.errors().map(|d| &d.message).collect::<Vec<_>>());
    let printed = gcs_core::syntax::render_flat(&mut prog).unwrap().to_string();
    let written = "X coincident(under: rolling_turn, seen_from: fixed_turn, t == 30) x";
    assert!(printed.contains(written), "{printed}");
    let again = solved(&printed);
    let carried = again.sketch.constraints.iter().filter(|c| c.kind == CKind::CarriedPoint);
    assert_eq!(carried.count(), 1);
    let (prog, errs) = crate::common::parse(&src.replace("under: rolling_turn", "under: o"));
    assert!(errs.is_empty(), "{errs:?}");
    let e = gcs_core::program::elaborate(&prog);
    assert!(e.errors().any(|d| d.message.contains("a motion is read off a frame")),
            "{:?}", e.errors().map(|d| &d.message).collect::<Vec<_>>());
}
