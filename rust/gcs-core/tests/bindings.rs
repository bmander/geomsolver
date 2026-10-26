//! **One figure, many placements** (#566): a closed instance's solids are a function of its
//! binding — its definition and its arguments — evaluated once and placed by the frame each
//! instance stands in, and a `ring` of instances is one binding under N placements.  Held here
//! by count (one build, one evaluation, said to a listener) and by the property: each placed copy
//! is what a fresh evaluation of its own solid makes, within the solve's own tolerance.

use std::rc::Rc;

use crate::common::{counted, read_solved as solved, refused, solid_named as solid};
use gcs_core::model::Sketch;
use gcs_core::progress::Kind;
use gcs_core::solid::{ApproximationPolicy as Policy, EvaluatedSolid};

/// A plate drawn about its frame's origin, off it by a millimetre, so a turn of the frame turns it
/// and a move moves it.
const PLATE: &str = "
component Plate(w: Length, d: Length, h: Length) {
  in std.front {
    a := point hint((1, 1))
    b := point hint((w, 1))
    c := point hint((w, d))
    e := point hint((1, d))
    a offset(dx: 1mm, dy: 1mm) std.origin
    b offset(dx: w, dy: 1mm) std.origin
    c offset(dx: w, dy: d) std.origin
    e offset(dx: 1mm, dy: d) std.origin
    ab := line(a, b)
    bc := line(b, c)
    ce := line(c, e)
    ea := line(e, a)
    body := solid(face(ab, bc, ce, ea), from: 0mm, to: h)
  }
}
";

/// Two frames standing apart, turned unalike, and an instance in each.
fn two(first: &str, second: &str) -> String {
    format!("
unit mm
use std
use std.place (offset)
{PLATE}
in (f := frame) {{ one := {first} }}
fix(dir == (0, 1, 0)) f.u
fix(dir == (0, 0, 1)) f.v
fix(origin == (10, 20, 30)) f
in (g := frame) {{ two := {second} }}
fix(dir == (0, 0, 1)) g.u
fix(dir == (1, 0, 0)) g.v
fix(origin == (-40, 5, 12)) g
")
}

/// The drawing with every binding forgotten, and nothing remembered: each solid its own.
fn unbound(sk: &Sketch) -> Sketch {
    let mut own = sk.clone();
    own.caches.clear();
    for s in &mut own.solids {
        s.binding = None;
    }
    own
}

/// Solid `i` evaluated as its own, its binding forgotten.
fn fresh(sk: &Sketch, i: usize) -> Rc<EvaluatedSolid> {
    unbound(sk).evaluated_solid(i, Policy::Report).unwrap()
}

/// How near a placed copy's every number is to a fresh evaluation's: a thousand times the solve's
/// step tolerance (`solve::XTOL`) of drawings a few hundred millimetres across, what two instances
/// of one shape solved apart, and a pose worked through the world, may differ by.
const NEAR: f64 = 1e3 * gcs_core::solve::XTOL * 1e3;

/// `placed` is `own`: the same volume, the same faces by name, and every corner of its mesh one
/// of the other's.
fn same(placed: &EvaluatedSolid, own: &EvaluatedSolid) {
    assert!((placed.volume() - own.volume()).abs() <= 1e-9 * own.volume(),
            "{} against {}", placed.volume(), own.volume());
    let (a, b) = (placed.world_mesh(), own.world_mesh());
    let faces = |m: &gcs_core::mesh::Mesh| {
        let mut f: Vec<String> = m.groups.iter().map(|g| g.path.clone()).collect();
        f.sort();
        f
    };
    assert_eq!(faces(&a), faces(&b));
    for p in a.positions.chunks(3) {
        let d = b.positions.chunks(3)
            .map(|q| (0..3).map(|k| (p[k] - q[k]).powi(2)).sum::<f64>().sqrt())
            .fold(f64::INFINITY, f64::min);
        assert!(d <= NEAR, "{p:?} is {d} from the fresh evaluation's nearest corner");
    }
    let (x, y) = (placed.world_bounds(), own.world_bounds());
    for k in 0..3 {
        assert!((x.lo[k] - y.lo[k]).abs() <= NEAR && (x.hi[k] - y.hi[k]).abs() <= NEAR);
    }
}

#[test]
fn two_closed_instances_with_equal_arguments_in_two_frames_are_built_once() {
    let src = two("Plate(w: 30mm, d: 20mm, h: 5mm)", "Plate(w: 30mm, d: 20mm, h: 5mm)");
    let (e, sk) = solved(&src);
    let (one, two) = (solid(&e, "one.body"), solid(&e, "two.body"));
    let (a, b) = (sk.solids[one].binding.clone().unwrap(), sk.solids[two].binding.clone().unwrap());
    assert_eq!(a.key, b.key);
    assert_ne!(a.frame, b.frame);
    let ((x, y), builds) = counted(|| {
        let x = sk.evaluated_solid(one, Policy::Report).unwrap();
        let y = sk.evaluated_solid(two, Policy::Report).unwrap();
        // and asked again, the copy is the one placed
        let z = sk.evaluated_solid(two, Policy::Report).unwrap();
        assert!(Rc::ptr_eq(&y, &z));
        (x, y)
    });
    assert_eq!(builds, vec![(Kind::Evaluate, one), (Kind::Build, one)], "{builds:?}");
    // the copy is the other instance's: its faces are its own instance's names
    assert!(y.mesh().groups.iter().all(|g| g.path.starts_with("two.")), "{:?}",
            y.mesh().groups.iter().map(|g| &g.path).collect::<Vec<_>>());
    same(&x, &fresh(&sk, one));
    same(&y, &fresh(&sk, two));
    // the exact B-rep likewise: built once, the copy's moved and renamed
    let (exact, builds) = counted(|| sk.exact_solid(two).unwrap());
    assert!(builds.is_empty(), "{builds:?}");
    assert!(exact.brep.faces.iter().all(|f| f.name.starts_with("two.")));
    let theirs = unbound(&sk).exact_solid(two).unwrap();
    let corners = |x: &gcs_core::solid::Exact| {
        let mut c: Vec<[i64; 3]> = x.brep.vertices.iter()
            .map(|v| std::array::from_fn(|k| ((x.origin[k] + v.p[k] / x.mm) * 1e6).round() as i64))
            .collect();
        c.sort();
        c
    };
    assert_eq!(corners(&exact), corners(&theirs));
}

/// A copy whose frame moves is the representative's evaluation placed again: nothing built or
/// renamed again (its shape keeps its serial, which a host keeps its geometry by), and still what
/// a fresh evaluation of it makes.
#[test]
fn a_copy_whose_frame_moves_is_placed_again_and_not_built() {
    let src = two("Plate(w: 30mm, d: 20mm, h: 5mm)", "Plate(w: 30mm, d: 20mm, h: 5mm)");
    let (e, mut sk) = solved(&src);
    let two = solid(&e, "two.body");
    let before = sk.evaluated_solid(two, Policy::Report).unwrap();
    let g = sk.solids[two].binding.as_ref().unwrap().frame.unwrap() as usize;
    sk.set_plane_origin(g, [5.0, -7.0, 3.0]);
    let (after, builds) = counted(|| sk.evaluated_solid(two, Policy::Report).unwrap());
    assert!(builds.is_empty(), "{builds:?}");
    assert_eq!(after.shape_serial(), before.shape_serial());
    assert!(after.world_bounds().lo != before.world_bounds().lo);
    same(&after, &fresh(&sk, two));
}

#[test]
fn unequal_arguments_are_two_builds_and_an_open_instance_is_built_as_its_own() {
    let src = two("Plate(w: 30mm, d: 20mm, h: 5mm)", "Plate(w: 30mm, d: 20mm, h: 6mm)");
    let (e, sk) = solved(&src);
    let (one, two) = (solid(&e, "one.body"), solid(&e, "two.body"));
    assert_ne!(sk.solids[one].binding.as_ref().unwrap().key,
               sk.solids[two].binding.as_ref().unwrap().key);
    let (_, builds) = counted(|| {
        sk.evaluated_solid(one, Policy::Report).unwrap();
        sk.evaluated_solid(two, Policy::Report).unwrap();
    });
    assert_eq!(builds.iter().filter(|(k, _)| *k == Kind::Build).count(), 2, "{builds:?}");

    // given an entity, an instance reads beside its frame: no binding, evaluated as today
    let (e, sk) = solved("
unit mm
use std
use std.place (offset)
component Lug(o: point, h: Length) {
  in std.front {
    a := point hint((1, 1))
    b := point hint((9, 1))
    c := point hint((9, 4))
    a offset(dx: 1mm, dy: 1mm) o
    b offset(dx: 9mm, dy: 1mm) o
    c offset(dx: 9mm, dy: 4mm) o
    ab := line(a, b)
    bc := line(b, c)
    ca := line(c, a)
    body := solid(face(ab, bc, ca), from: 0mm, to: h)
  }
}
in std.front {
  p := point
  fix((0, 0)) p
  q := point
  fix((40, 0)) q
}
one := Lug(p, h: 2mm)
two := Lug(q, h: 2mm)
");
    let (one, two) = (solid(&e, "one.body"), solid(&e, "two.body"));
    assert!(sk.solids[one].binding.is_none() && sk.solids[two].binding.is_none());
    let (_, builds) = counted(|| {
        sk.evaluated_solid(one, Policy::Report).unwrap();
        sk.evaluated_solid(two, Policy::Report).unwrap();
    });
    assert_eq!(builds.iter().filter(|(k, _)| *k == Kind::Build).count(), 2, "{builds:?}");
}

/// A peg drawn about its own origin, read by its relations alone, in a ring of six about a point
/// beside it.
const RING: &str = "
unit mm
use std
use std.place (offset, right_of)
component Peg(r: Length) {
  in std.front {
    c := point hint((20, 0))
    c right_of(d: 20mm) std.origin
    t := point hint((24, 1))
    t offset(dx: 24mm, dy: 1mm) std.origin
    k := point hint((20, 3))
    k offset(dx: 20mm, dy: 3mm) std.origin
    rim := circle(center: c) hint(r: r)
    radius(r) rim
    body := solid(face(rim), from: 0mm, to: 5mm)
    ct := line(c, t)
    tk := line(t, k)
    kc := line(k, c)
    tab := solid(face(ct, tk, kc), from: 0mm, to: 1mm)
  }
}
in std.front {
  o := point hint((50, 0))
  fix((50, 0)) o
  ring 6 about o {
    p := Peg(r: 3mm)
  }
}
";

#[test]
fn a_ring_of_instances_is_built_once_and_each_copy_stands_turned() {
    let (e, sk) = solved(RING);
    let bodies: Vec<usize> = (0..6).map(|k| solid(&e, &format!("p[{k}].body"))).collect();
    for (k, &i) in bodies.iter().enumerate() {
        let b = sk.solids[i].binding.as_ref().unwrap();
        assert_eq!(b.key, sk.solids[bodies[0]].binding.as_ref().unwrap().key);
        assert_eq!(b.turn.map(|(_, k, n)| (k, n)), (k > 0).then_some((k as u32, 6)));
    }
    let (placed, builds) = counted(|| {
        bodies.iter().map(|&i| sk.evaluated_solid(i, Policy::Report).unwrap()).collect::<Vec<_>>()
    });
    assert_eq!(builds, vec![(Kind::Evaluate, bodies[0]), (Kind::Build, bodies[0])], "{builds:?}");
    // copy k is the representative turned k sixths about `o`, and what a fresh evaluation of its
    // own solid makes
    let rep = placed[0].world_bounds();
    for (k, s) in placed.iter().enumerate() {
        let th = std::f64::consts::TAU * k as f64 / 6.0;
        let centre = |b: &gcs_core::solid::Box3| [0.5 * (b.lo[0] + b.hi[0]), 0.5 * (b.lo[2] + b.hi[2])];
        let [x, z] = centre(&rep);
        let turned = [50.0 + (x - 50.0) * th.cos() - z * th.sin(), (x - 50.0) * th.sin() + z * th.cos()];
        let at = centre(&s.world_bounds());
        assert!((at[0] - turned[0]).hypot(at[1] - turned[1]) < 1e-6, "p[{k}]: {at:?} {turned:?}");
        same(s, &fresh(&sk, bodies[k]));
    }
}

/// **A solid made of a binding's solids reads each as the one placed**: the union of the ring's
/// pegs, written outside the ring, builds one peg's B-rep, and is the union built peg by peg.
#[test]
fn a_union_of_a_rings_instances_builds_one_of_them() {
    let src = RING.replace("    p := Peg(r: 3mm)\n  }\n", "    p := Peg(r: 3mm)\n  }\n  pegs := solid(p[0].body)\n  \
        repeat 5 as i {\n    p[i + 1].body union pegs\n  }\n");
    let (e, sk) = solved(&src);
    let (pegs, rep) = (solid(&e, "pegs"), solid(&e, "p[0].body"));
    let (union, builds) = counted(|| sk.evaluated_solid(pegs, Policy::Report).unwrap());
    let built: Vec<usize> =
        builds.iter().filter(|(k, _)| *k == Kind::Build).map(|&(_, i)| i).collect();
    assert_eq!(built, vec![pegs, rep], "{builds:?}");
    same(&union, &fresh(&sk, pegs));
}

#[test]
fn reaching_outside_the_ring_is_still_refused() {
    // a declaration built on the frame's origin is the same origin in every copy, unturned
    let src = RING.replace("    rim := circle(center: c) hint(r: r)\n",
                           "    rim := circle(center: c) hint(r: r)\n    s := line(std.origin, c)\n");
    refused(&src, "E021", "`std.origin` is outside the `ring`", "std.origin");
    // and an argument read from outside it
    refused("
unit mm
use std
use std.place (offset)
component Lug(o: point) {
  in std.front {
    a := point hint((1, 1))
    a offset(dx: 1mm, dy: 1mm) o
  }
}
in std.front {
  o := point hint((50, 0))
  fix((50, 0)) o
  q := point
  fix((0, 0)) q
  ring 6 about o {
    p := Lug(q)
  }
}
", "E021", "`q` is outside the `ring`", "q");
    // and the ring's own body reading a datum, outside any instance
    refused("
unit mm
use std
use std.place (right_of)
in std.front {
  o := point hint((50, 0))
  fix((50, 0)) o
  ring 6 about o {
    c := point hint((20, 0))
    c right_of(d: 20mm) std.origin
  }
}
", "E021", "`std.origin` is outside the `ring`", "std.origin");
}
