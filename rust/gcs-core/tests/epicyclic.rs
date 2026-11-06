//! **A planet riding its valued carrier** (#823): an epicyclic train drawn as it is built, each
//! planet in its carrier's block, spinning on its own pin, both frames given a value over the one
//! scrubbed parameter (`fix(value == t)`).  A frame declared in a valued frame's block rides on it
//! as the rest of the block does — drawn where the value puts it — so the planet's pin, drawn in
//! the carrier's block, stands where the carrier carries it, the planet's joint turns about it,
//! and what the planet's block draws is read through the carrier's value and then the planet's.
//!
//! Gated by the closed form of the train (a point of the planet turned on the pin at rest, then
//! carried round, to `EXACT` at every station of the march along `t`), by the rest pose spun and
//! then carried — a placed copy placed again, the pattern drawn before a planet could ride
//! (bmander/radiatorbot) — standing corner for corner where the riding planet does, and by the
//! diagnostics the issue's own source drew: none, and no hidden name in what is said of the rest.
use gcs_core::program::Elaborated;
use gcs_core::space::{norm, sub};

use crate::common::parsed::{build, determined};

/// How near the solve puts a point to the train's closed form.
const EXACT: f64 = 1e-9;

/// **A train**: a carrier spinning about `o`, a planet in its block spinning on the pin the
/// carrier's block draws, three times as fast the other way, both stood at `t`; and beside it the
/// same planet drawn at rest, `part`, spun on its pin and the copy carried round — the pattern
/// drawn while a planet could not ride.
const TRAIN: &str = "unit mm
use std
use std.solids
use std.place (coords)
use std.motion (spins)

param t: Angle over (0deg, 360deg)
in std.top {
  o := point
  o coords(du: 0mm, dv: 0mm) std.top
  pin0 := point hint((20, 0))
  pin0 coords(du: 20mm, dv: 0mm) std.top
  rim0 := circle(center: pin0) hint(r: 5)
  radius(5mm) rim0
}
in (carrier := frame) {
  in std.top {
    pin := point hint((20, 0))
    pin coords(du: 20mm, dv: 0mm) std.top
  }
  in (planet := frame) {
    in std.top {
      c := point hint((20, 0))
      c coords(du: 20mm, dv: 0mm) std.top
      x := point hint((25, 0))
      x coords(du: 25mm, dv: 0mm) std.top
      rim := circle(center: c) hint(r: 5)
      radius(5mm) rim
    }
    body := solid(std.solids.Prism(face(rim), from: 0mm, to: 3mm))
  }
}
carrier spins(1) o
planet spins(-3) pin
fix(value == t) carrier
fix(value == t) planet

part := solid(std.solids.Prism(face(rim0), from: 0mm, to: 3mm))
spin := frame
spin spins(-3) pin0
fix(value == t) spin
orbit := frame
orbit spins(1) o
fix(value == t) orbit
spun := part at spin
shown := spun at orbit
";

/// The issue's own source: the planet in its carrier's block with no value of its own, a solid
/// in its block, and the carrier's value over `t`.
const ASKED: &str = "unit mm
use std
use std.solids
use std.place (coords)
use std.motion (spins)

param t: Angle over (0deg, 360deg)
in std.top {
  o := point
  o coords(du: 0mm, dv: 0mm) std.top
}
in (carrier := frame) {
  in std.top {
    pin := point hint((20, 0))
    pin coords(du: 20mm, dv: 0mm) std.top
  }
  in (planet := frame) {
    in std.top {
      rim := circle(center: pin) hint(r: 5)
      radius(5mm) rim
    }
    planet_body := solid(std.solids.Prism(face(rim), from: 0mm, to: 3mm))
  }
}
carrier spins(1) o
planet spins(-3) pin
fix(value == t) carrier
";

fn index(e: &Elaborated, name: &str) -> usize {
    e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`")).i()
}

/// `p` turned `deg` degrees about the world's `z` through `c`.
fn turned(p: [f64; 3], c: [f64; 3], deg: f64) -> [f64; 3] {
    let (s, k) = deg.to_radians().sin_cos();
    let (x, y) = (p[0] - c[0], p[1] - c[1]);
    [c[0] + x * k - y * s, c[1] + x * s + y * k, p[2]]
}

/// Where the train puts a point of the planet drawn at `p` at `t`: turned `-3t` on the pin at
/// rest, `(20, 0, 0)`, then `t` about the carrier's axis through the origin.
fn train(p: [f64; 3], t: f64) -> [f64; 3] {
    turned(turned(p, [20.0, 0.0, 0.0], -3.0 * t), [0.0; 3], t)
}

/// The words every diagnostic of `e` says, with its code.
fn said(e: &Elaborated) -> Vec<String> {
    e.diags.iter().map(|d| format!("{} {}", d.code.as_str(), d.message)).collect()
}

/// **The planet rides its valued carrier**: followed along `t`, the planet's point and its pin's
/// image stand where the carrier carries the planet turned on its pin at rest — the carrier's
/// value and then the planet's, each at its own joint — and the pin the carrier draws stays on
/// the planet's centre.  Nothing is said of the drawing: no refusal, no warning.
#[test]
fn a_planet_rides_its_valued_carrier() {
    let e = determined(TRAIN);
    assert!(e.diags.is_empty(), "{:?}", said(&e));
    let f = gcs_core::follow::follow(&e.sketch, "t").expect("the train is followed along `t`");
    assert!(f.singular.is_none() && f.reached() == 360.0, "{:?}", f.singular);
    let rest = |n: &str| match n {
        "x" => [25.0, 0.0, 0.0],
        _ => [20.0, 0.0, 0.0],
    };
    for t in [0.0, 30.0, 90.0, 137.5, 250.0, 360.0] {
        let mut sk = e.sketch.clone();
        f.put(&mut sk, t).unwrap();
        for n in ["x", "c", "pin"] {
            let (got, want) = (sk.world_point(index(&e, n)), train(rest(n), t));
            assert!(norm(sub(got, want)) < EXACT, "{n} at t = {t}: {got:?} against {want:?}");
        }
    }
}

/// **A placed copy may be placed again** — the rest pose spun on its pin, then carried round —
/// and it stands where the planet riding its carrier does: their exact B-reps corner for corner,
/// at every station, the same 75π mm³.
#[test]
fn the_rest_pose_spun_then_carried_is_the_planet_riding() {
    let e = determined(TRAIN);
    let f = gcs_core::follow::follow(&e.sketch, "t").expect("followed");
    let (body, shown) = (index(&e, "body"), index(&e, "shown"));
    for t in [0.0, 45.0, 200.0] {
        let mut sk = e.sketch.clone();
        f.put(&mut sk, t).unwrap();
        let brep = |i: usize| sk.exact_solid(i).unwrap_or_else(|m| panic!("{i}: {m}")).brep.clone();
        let (a, b) = (brep(body), brep(shown));
        let v = gcs_core::brep::props::volume(&a);
        assert!((v - 75.0 * std::f64::consts::PI).abs() < 1e-9 * v, "{v} against 75π");
        assert!((gcs_core::brep::props::volume(&b) - v).abs() < 1e-9 * v);
        assert_eq!(a.vertices.len(), b.vertices.len());
        for p in &a.vertices {
            assert!(b.vertices.iter().any(|q| norm(sub(p.p, q.p)) < EXACT),
                    "t = {t}: {:?} is no corner of the copy carried round", p.p);
        }
        // and where the train puts the planet: every corner on its rim, 5 about the carried pin
        let pin = train([20.0, 0.0, 0.0], t);
        for p in &a.vertices {
            let r = (p.p[0] - pin[0]).hypot(p.p[1] - pin[1]);
            assert!((r - 5.0).abs() < EXACT, "t = {t}: {:?} is {r} from the pin", p.p);
        }
    }
}

/// **The issue's source says nothing it should not**: it elaborates and solves, with no E105 and
/// no hidden twin's key (`#in…`) — and where a plane a valued frame's block draws in is named, it
/// is named as the source reads it, `std.top in carrier`.
#[test]
fn a_valued_carriers_planes_are_named_as_the_source_reads_them() {
    let e = build(ASKED);
    assert!(e.ok(), "{:?}", said(&e));
    assert!(said(&e).iter().all(|m| !m.contains("E105") && !m.contains('#')), "{:?}", said(&e));
    // two views nested in the carrier's block: the inner one draws, said by its name
    let nested = ASKED.replace("  in std.top {\n    pin := point",
                               "  in std.front {\n  in std.top {\n    pin := point")
        .replace("    pin coords(du: 20mm, dv: 0mm) std.top\n  }",
                 "    pin coords(du: 20mm, dv: 0mm) std.top\n  }\n  }");
    let s = said(&build(&nested));
    assert!(s.iter().any(|m| m.starts_with("W115") && m.contains("`std.top in carrier`")),
            "{s:?}");
    assert!(s.iter().all(|m| !m.contains("#in")), "{s:?}");
    // a point drawn in the carrier's plane and again in the valued planet's
    let twice = format!("{ASKED}fix(value == t) planet\n");
    let s = said(&build(&twice));
    assert!(s.iter().any(|m| m.starts_with("E060") && m.contains("`std.top in carrier`")),
            "{s:?}");
    assert!(s.iter().all(|m| !m.contains("#in")), "{s:?}");
}

/// **A frame with a value carries what is declared in its block**, and only that: drawn in two
/// valued frames, the inner one declared outside the outer one's block, a point is refused —
/// that frame turns about an axis the outer one does not carry — and so is one drawn in the
/// planet's block outside the carrier's, which the carrier's value would not carry.
#[test]
fn two_valued_frames_nest_only_as_declared() {
    let outside = format!("{ASKED}in planet {{\n  in std.top {{\n    q := point\n  }}\n}}\n");
    let s = said(&build(&outside));
    assert!(s.iter().any(|m| m.starts_with("E080") && m.contains("`q` is drawn in `planet`, \
        which is declared in `carrier`'s block, and outside it")), "{s:?}");
    let src = ASKED.replace("  in (planet := frame) {", "  in planet {")
        .replace("in (carrier := frame) {", "planet := frame\nin (carrier := frame) {");
    let src = format!("{src}fix(value == t) planet\n");
    let s = said(&build(&src));
    assert!(s.iter().any(|m| m.starts_with("E080") && m.contains("declared outside `carrier`'s \
                                                                 block")), "{s:?}");
}
