//! **What makes an exact swept cut quick, and reads as the long way did**: a spline's foot found
//! from its spans' Bézier forms, a field's side of a level read without the operands it does not
//! need, a cone's step read where the pair meets, a prism of whole circles turned as its section,
//! and a closed instance's swept gear built once for every copy of it.
use crate::common::build;
use gcs_core::brep::geom::{Curve, Frame, Surface};
use gcs_core::brep::nurbs::{arc, interpolate, BSpline};
use gcs_core::rng::Rng;
use gcs_core::space::{distance as dist, dot, sub};
use std::f64::consts::{FRAC_PI_4, TAU};
use std::sync::Arc;

type V = [f64; 3];

/// The curve's nearest point to `p` by a walk over a thousand samples a span refined by Newton:
/// what the foot is held to, written apart from it.
fn walked(b: &BSpline, p: V) -> f64 {
    let [a, z] = b.domain();
    let mut best = (f64::INFINITY, a);
    for j in 0..=20_000 {
        let t = a + (z - a) * j as f64 / 20_000.;
        let d = dist(b.point(t), p);
        if d < best.0 { best = (d, t); }
    }
    let mut t = best.1;
    for _ in 0..50 {
        let (x, d, dd) = b.d2(t);
        let e = sub(x, p);
        let (g, h) = (dot(e, d), dot(d, d) + dot(e, dd));
        if !(h > 0.) { break }
        t = (t - g / h).clamp(a, z);
    }
    dist(b.point(t), p)
}

/// **A spline's foot is the nearest point of it** (`BSpline::nearest`): its spans' samples read
/// off their Bézier forms find the point a dense walk finds, on a wavy cubic and on a rational
/// circle, from points near the curve, on it and far off it; asked again, a thread's held foot is
/// the same answer to the bit.
#[test]
fn a_splines_foot_is_the_nearest_point_of_it() {
    let pts: Vec<V> = (0..12).map(|i| {
        let x = i as f64;
        [x, (0.9 * x).sin() * 3., 0.2 * x * x - x]
    }).collect();
    let ts: Vec<f64> = (0..12).map(|i| i as f64 / 11.).collect();
    let wavy = interpolate(&pts, &ts, 3).unwrap();
    let circle = arc(&Frame::new([1., -2., 0.5], [0., 0., 1.], [1., 0., 0.]), 4., [0., TAU]);
    let mut rng = Rng::new(0x5eed);
    for b in [wavy, circle] {
        let curve = Curve::BSpline(Arc::new(b.clone()));
        let [a, z] = b.domain();
        for k in 0..60 {
            let on = b.point(a + (z - a) * rng.uniform(0., 1.));
            let off = match k % 3 { 0 => 0., 1 => 0.3, _ => 6. };
            let p = [on[0] + off * rng.uniform(-1., 1.), on[1] + off * rng.uniform(-1., 1.), on[2] + off * rng.uniform(-1., 1.)];
            let t = curve.inverse(p);
            let found = dist(b.point(t), p);
            let want = walked(&b, p);
            assert!(found <= want + 1e-9 * (1. + want), "the foot of {p:?} is {found} away, a walk finds {want}");
            assert_eq!(curve.inverse(p).to_bits(), t.to_bits(), "asked again");
        }
    }
}

/// **A cone's step is read where the pair meets** (`Surface::feature_within`): a cone framed at
/// its apex says nothing of its curvature by its radius there, and a box far from its axis reads
/// the least radius of curvature in it, `ρ / cos α`, never more than a point of the box has.
#[test]
fn a_cones_feature_is_read_in_the_box_it_is_asked_over() {
    let cone = Surface::Cone(Frame::about([0.; 3], [0., 0., 1.]), 0., FRAC_PI_4);
    assert!(cone.feature() < 1e-300);
    let (lo, hi) = ([9., -1., 4.], [11., 1., 6.]);
    let f = cone.feature_within(lo, hi);
    let half = dist(lo, hi) / 2.;
    assert!((f - (10. - half) / FRAC_PI_4.cos()).abs() < 1e-12, "{f}");
    // no point of the box is nearer the axis than the bound read
    let mut rng = Rng::new(7);
    for _ in 0..200 {
        let p: V = std::array::from_fn(|k| rng.uniform(lo[k], hi[k]));
        assert!(p[0].hypot(p[1]) / FRAC_PI_4.cos() >= f);
    }
    // a box about the axis reads the cone as its frame does
    assert_eq!(cone.feature_within([-1., -1., 0.], [1., 1., 1.]), cone.feature());
}

const PLATE: &str = "\
unit mm
use std
in std.front {
a := point
b := point hint((60, 0))
c := point hint((60, 30))
d := point hint((40, 30))
e := point hint((20, 30))
f := point hint((0, 30))
m := point
fix((0, 0)) a
fix((30, 30)) m
a distance(60) b
b distance(30) c
a distance(30) f
ab := line(a, b)
bc := line(b, c)
cd := line(c, d)
notch := arc(center: m, start: e, end: d) hint(r: 10)
ef := line(e, f)
fa := line(f, a)
horizontal ab
vertical bc
horizontal cd
horizontal ef
vertical fa
radius(10) notch
h := point
fix((45, 12)) h
hole := circle(center: h) hint(r: 5)
radius(5) hole
o := point
z := point
fix((0, 0)) o
fix((0, 10)) z
ax := line(o, z)
}
plate := solid(face(ab, bc, cd, notch, ef, fa), depth: 8mm)
bore := solid(face(hole), from: -10mm, to: 2mm)
bore cut plate
turn := frame
ax coincident(sense: same) turn.axis; fix(ratio == 1, advance == 0mm) turn
moved := plate at(under: turn) 40deg
both := solid(plate)
moved union both
";

/// **A field's side of a level is its value's** (`SpatialField::below`): a notched, bored plate
/// and its turned copy united — arcs, holes, a placement and every Boolean — read at points
/// all about them, at levels either side of zero, says what comparing the value says.
#[test]
fn a_fields_side_of_a_level_is_what_its_value_says() {
    let mut e = build(PLATE);
    assert!(gcs_core::solve::solve(&mut e.sketch, Default::default()).success);
    let mut rng = Rng::new(0xf1e1d);
    for name in ["plate", "both"] {
        let i = e.map.ent_named(name).unwrap().i();
        let f = gcs_core::solid::SpatialField::read(&e.sketch, i, 1e-10).unwrap();
        let mut sides = [0; 2];
        for _ in 0..4000 {
            let p: V = [rng.uniform(-5., 65.), rng.uniform(-12., 12.), rng.uniform(-5., 35.)];
            let t = [-3., -1e-3, 0., 1e-3, 2.][rng.uniform(0., 5.) as usize % 5];
            let below = f.below(p, t);
            assert_eq!(below, f.value(p) < t, "`{name}` at {p:?} against {t}: {}", f.value(p));
            sides[usize::from(below)] += 1;
        }
        assert!(sides.iter().all(|&n| n > 100), "{sides:?}");
    }
}

/// **A prism of whole circles about the line is turned as its section** (`Recipe::meridian`): a
/// bored disc chamfered at both faces, as a gear's blank is, is built from its meridian, with the
/// volume and the faces' names its Booleans' build has.
#[test]
fn a_prism_of_whole_circles_is_the_turn_of_its_section() {
    let src = "\
unit mm
use std
use std (vertical, on_v, raised)
in std.front {
  construction spindle := vertical line(std.origin, hint((0, 10)))
  length(10mm) spindle
}
in std.top {
  tip := circle(center: std.top.origin) hint(r: 20)
  radius(20mm) tip
  hole := circle(center: std.top.origin) hint(r: 3)
  radius(3mm) hole
  top_at := point hint((0, 19))
  top_at on_v(d: 19mm) std.top
  bottom_at := point hint((0, 25))
  bottom_at on_v(d: 25mm) std.top
}
disc := solid(face(tip, holes: hole), from: 0mm, to: 6mm)
top_q := point
top_q raised(d: 6mm, along: std.z) top_at
bottom_q := point
bottom_q raised(d: 6mm, along: std.z) bottom_at
top_cone := std.ConeThrough(spindle, top_q, lean: -45deg)
bottom_cone := std.ConeThrough(spindle, bottom_q, lean: 45deg)
bicone := { p | p inside top_cone; p inside bottom_cone }
construction chamfers := solid(bicone)
chamfers common disc
";
    let mut e = build(src);
    assert!(gcs_core::solve::solve(&mut e.sketch, Default::default()).success);
    let disc = e.map.ent_named("disc").unwrap().i();
    let recipe = gcs_core::solid::cad::recipe_static(&e.sketch, disc).unwrap();
    let read = gcs_core::brep::recipe::Recipe::read(&recipe.recipe).unwrap();
    let (turned, _, _) = read.meridian([1., 0., 0.]).unwrap_or_else(|e| panic!("{e:?}"));
    let built = read.build().unwrap();
    let volume = gcs_core::brep::props::volume;
    assert!((volume(&turned) - volume(&built)).abs() < 1e-9 * volume(&built), "{} against {}", volume(&turned), volume(&built));
    let names = |b: &gcs_core::brep::topo::Brep| b.faces.iter().map(|f| f.name.clone()).collect::<std::collections::BTreeSet<_>>();
    assert_eq!(names(&turned), names(&built));
}

/// A closed rack-cut spur, twenty teeth of module 1.5, its tooth spaces the rack's sweep turned a
/// pitch at a time, drawn about its own origin to be seated where it is wanted.
const SPURS: &str = "\
unit mm
use std
use std (vertical, turns, slides, coords, on_v, raised, seats)
component Spur(N: Int, m: Length, z1: Length) {
  R := m * N / 2
  hw := pi * m / 4
  reach := sqrt((R + m) ^ 2 - (R - 1.25 * m) ^ 2) + hw + 1.5 * m * tan(20deg)
  window := reach / R * 1rad
  in std.front {
    construction spindle := vertical line(std.origin, hint((0, 10)))
    length(10mm) spindle
  }
  in std.top {
    construction rack_line := line(hint((10, R)), hint((0, R)))
    rack_line.p1 coords(du: 10mm, dv: R) std.top
    rack_line.p2 on_v(d: R) std.top
    a := point hint((-hw - 1.5 * m * tan(20deg), R + 1.5 * m))
    b := point hint((-hw + 1.25 * m * tan(20deg), R - 1.25 * m))
    c := point hint((hw - 1.25 * m * tan(20deg), R - 1.25 * m))
    d := point hint((hw + 1.5 * m * tan(20deg), R + 1.5 * m))
    a coords(du: -hw - 1.5 * m * tan(20deg), dv: R + 1.5 * m) std.top
    b coords(du: -hw + 1.25 * m * tan(20deg), dv: R - 1.25 * m) std.top
    c coords(du: hw - 1.25 * m * tan(20deg), dv: R - 1.25 * m) std.top
    d coords(du: hw + 1.5 * m * tan(20deg), dv: R + 1.5 * m) std.top
    rack_tooth := face(a, b, c, d, -> close)
    tip := circle(center: std.top.origin) hint(r: R + m)
    radius(R + m) tip
    hole := circle(center: std.top.origin) hint(r: 3)
    radius(3mm) hole
  }
  private construction tooth := solid(rack_tooth, from: -1mm, to: z1 + 1mm)
  blank_turn := frame
  blank_turn turns(1) spindle
  rack := frame
  rack slides(2 * pi * R) rack_line
  index := frame
  index turns(1) spindle
  construction gear := solid(face(tip, holes: hole), from: 0mm, to: z1)
  private construction space := sweep(tooth over t in (-window, window), under: rack, seen_from: blank_turn)
  space cut gear
  repeat N - 1 as j {
    turned := space at(under: index) ((j + 1) * 360deg / N)
    turned cut gear
  }
}
a := Spur(N: 20, m: 1.5mm, z1: 6mm)
b := Spur(N: 20, m: 1.5mm, z1: 6mm)
a_at := point
a_at raised(d: -40mm, along: std.y) std.origin
a seats(turn: 0deg, up: std.z, east: std.x) a_at
b_at := point
b_at raised(d: 40mm, along: std.x) std.origin
b seats(turn: 15deg, up: std.z, east: std.x) b_at
";

/// **Two closed instances of one swept gear are one build** (`Sketch::bound_where`, #566): the
/// second's exact B-rep is the first's moved where it stands — said so — its faces named in its
/// own instance, with the first's volume, and its mesh agrees with its own material field.
#[test]
fn two_closed_instances_of_a_swept_gear_are_built_once_and_placed() {
    use gcs_core::brep::{export, sweep::Say};
    let mut e = build(SPURS);
    assert!(gcs_core::solve::solve(&mut e.sketch, Default::default()).success);
    let (a, b) = (e.map.ent_named("a.gear").unwrap().i(), e.map.ent_named("b.gear").unwrap().i());
    let said = std::sync::Mutex::new(Vec::<String>::new());
    let say = Say { stage: &|l: &str| said.lock().unwrap().push(l.to_string()), mark: &|_| {} };
    assert_eq!(e.sketch.bound_where(b, true).map(|bound| bound.rep), Some(a));
    let first = export::exact(&e.sketch, a, None, None, &say).unwrap_or_else(|r| panic!("refused at {:?}: {}", r.stage, r.message));
    said.lock().unwrap().clear();
    let second = export::exact(&e.sketch, b, None, None, &say).unwrap_or_else(|r| panic!("refused at {:?}: {}", r.stage, r.message));
    assert!(said.lock().unwrap().iter().any(|l| l.contains("is `a.gear` placed")), "{:?}", said.lock().unwrap());
    assert!(second.solid.faces.iter().all(|f| !f.name.starts_with("a.")), "a face of the copy keeps the first's name");
    let volume = gcs_core::brep::props::volume;
    assert!((volume(&first.solid) - volume(&second.solid)).abs() < 1e-9 * volume(&first.solid));
    export::stl(&e.sketch, b, &second, None, &say).unwrap_or_else(|r| panic!("the copy's STL refused at {:?}: {}", r.stage, r.message));
}
