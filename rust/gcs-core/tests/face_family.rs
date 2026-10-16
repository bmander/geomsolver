//! **The face family** (#243 L1): `solid(I(…).f over s in (a, b))`, the solid a face of an
//! instance sweeps as one of its numbers runs.  The station is drawn once, hidden, at the middle
//! of its interval and solved with the drawing; the family solves it again at every other station,
//! each warm from its neighbour, and facets what the stations sweep.  Its walls are exact (L2):
//! each the surface its edge sweeps as the station is solved at the nodes of each piece of the
//! interval, a polynomial in the running number between them (`brep::family`), lowered to a plane,
//! a cylinder, a cone or a torus where its numbers say it is one, and to the surface its first rim
//! sweeps where the whole section moves rigidly (L6).
//!
//! The fixtures are the L0 target programs (`docs/lofts/l0/`), over the station words they were
//! written with (`std.OnLine` and its kin since L3, drafted in L0's `lofts.sv`), and the gates are
//! their closed forms: a reduced section is its prismatoid, a twisted prism A·L at any twist and
//! under any law (Cavalieri), a scaled one a frustum and a pyramid, a bend and a thread Pappus, a
//! wire square to its helix A × the helix's length, a smooth loft π∫r² by a quadrature exact for
//! its rail's degree — each read off the exact B-rep (`props::volume`, its boundary checked by
//! `Brep::check`) to a part in 10⁹.


use gcs_core::model::{EntKind, SolidDef};
use gcs_core::program::{elaborate, Elaborated};
use gcs_core::solid::ApproximationPolicy::Report;
use gcs_core::solve::{solve, SolveOpts};
use std::f64::consts::PI;

use crate::common::parse;

const REDUCER: &str = include_str!("../../../docs/lofts/l0/reducer.sv");
const ELBOW: &str = include_str!("../../../docs/lofts/l0/elbow.sv");
const TWISTED: &str = include_str!("../../../docs/lofts/l0/twisted_duct.sv");
const PYRAMID: &str = include_str!("../../../docs/lofts/l0/pyramid.sv");
const SQUARE_TO_CIRCLE: &str = include_str!("../../../docs/lofts/l0/square_to_circle.sv");
const SPRING: &str = include_str!("../../../docs/lofts/l0/spring.sv");
const BOTTLE: &str = include_str!("../../../docs/lofts/l0/bottle.sv");
const BLADE: &str = include_str!("../../../docs/lofts/l0/blade.sv");

/// `src` linked as `solventc` links an L0 program (the library), elaborated: the diagnostics'
/// words, and what was made.
fn read(src: &str) -> (Elaborated, Vec<String>) {
    let (mut prog, errs) = parse(src);
    assert!(errs.is_empty(), "{errs:?}");
    let linked = gcs_core::modules::link(&mut prog, &mut gcs_core::library::resolve);
    assert!(linked.is_empty(), "{:?}", linked.iter().map(|d| &d.message).collect::<Vec<_>>());
    let e = elaborate(&prog);
    let said = e.errors().map(|d| d.message.clone()).collect();
    (e, said)
}

/// `src` elaborated and solved, or the test fails saying why.
fn family(src: &str) -> Elaborated {
    let (mut e, said) = read(src);
    assert!(said.is_empty(), "{said:?}");
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}", r.message);
    e
}

fn index(e: &Elaborated, name: &str) -> usize {
    e.map.ent_named(name).unwrap_or_else(|| panic!("no solid `{name}`")).i()
}

/// The volume of the solid `name` read off its exact B-rep (`props::volume`, in model units), its
/// boundary checked first (`Brep::check`) — or why it is refused.
fn volume(e: &Elaborated, name: &str) -> Result<f64, String> {
    let x = e.sketch.exact_solid(index(e, name))?;
    x.brep.check(1e-9 * x.brep.size()).map_err(|k| k.msg)?;
    Ok(gcs_core::brep::props::volume(&x.brep) / x.mm.powi(3))
}

/// The kinds of the surfaces of the exact solid `name`'s faces, by face.
fn kinds(e: &Elaborated, name: &str) -> std::collections::BTreeMap<String, &'static str> {
    let x = e.sketch.exact_solid(index(e, name)).unwrap();
    x.brep.faces.iter().map(|f| (f.name.clone(), f.surface.kind())).collect()
}

/// `got` within `rel` of `want`, said with both.
fn near(got: f64, want: f64, rel: f64, what: &str) {
    assert!((got - want).abs() <= rel * want.abs(), "{what}: {got} against {want} (±{rel})");
}

/// The names of the faces of the family `name` that survive.
fn faces_of(e: &Elaborated, name: &str) -> std::collections::BTreeSet<String> {
    let s = e.sketch.evaluated_solid(index(e, name), Report).unwrap();
    s.surviving_faces().iter().map(|f| f.to_string()).collect()
}

/// Why the family `name` is refused, its words.
fn refused(e: &Elaborated, name: &str) -> String {
    match gcs_core::solid::validate(&e.sketch, index(e, name)) {
        Ok(()) => panic!("`{name}` was not refused"),
        Err(m) => m,
    }
}

/// **A reducer is its prismatoid**: a hollow square whose size runs linearly from 24 to 12 over
/// 40 mm has an area linear in `s`, so V = 40 · (8 · 18 − 16) = 5120 mm³ — and every wall is flat,
/// so the faceting holds it exactly.
#[test]
fn a_reducer_is_its_prismatoid() {
    let e = family(REDUCER);
    near(volume(&e, "body").unwrap(), 5120.0, 1e-9, "reducer");
}

/// **A bend is Pappus**: an 18 mm hollow square (area 128) carried a quarter turn round a bend of
/// radius 30, its centroid on the arc: V = 128 · 30 · π/2 = 1920π mm³.
#[test]
fn a_bend_is_pappus() {
    let e = family(ELBOW);
    near(volume(&e, "body").unwrap(), 1920.0 * PI, 1e-9, "elbow");
}

/// **A twisted prism is A·L at any twist** (Cavalieri): every section is the 20 mm square, so the
/// duct turned a quarter and the auger turned three and a half turns — which no pairing of two
/// faces can say — are both 20² · 40 = 16000 mm³, their walls twisted sheets (`Surface::Family`).
#[test]
fn a_twisted_prism_is_cavalieri_at_any_twist() {
    let e = family(TWISTED);
    near(volume(&e, "duct").unwrap(), 16000.0, 1e-9, "a quarter turn");
    near(volume(&e, "auger").unwrap(), 16000.0, 1e-9, "three and a half turns");
}

/// **And under any law**: the turn eased in and out, `twist · (3s² − 2s³)` — `std.Twisted` given
/// that fraction for `s` — is an expression no unknown of the drawing could be (one free name,
/// affine), and a station holds `s` at a number, so the law is its body's to say.  The sections
/// are the same squares: 16000 again.
#[test]
fn an_eased_twist_is_cavalieri_too() {
    let law = "std.Twisted(st.n, twist: twist, reach: 10mm, s: 3 * s^2 - 2 * s^3)";
    let eased = TWISTED.replace("std.Twisted(st.n, twist: twist, reach: 10mm, s: s)", law);
    assert_ne!(eased, TWISTED, "the law is written in");
    let e = family(&eased);
    near(volume(&e, "duct").unwrap(), 16000.0, 1e-9, "an eased quarter turn");
}

/// **A scaled prism is a frustum, and one scaled to nothing a pyramid**: the square shrinking from
/// 20 to its centre over 30 mm is 20² · 30 / 3 = 4000 mm³; its end is an apex, where the station
/// loses rank, so it is read as the limit of the stations closing in and never solved (finding
/// 9), and it has no `end` cap.  Stopped halfway, it is the frustum 15/3 · (400 + 100 + 200).
#[test]
fn a_scaled_prism_is_a_frustum_and_a_pyramid_its_limit() {
    let e = family(PYRAMID);
    near(volume(&e, "pyramid").unwrap(), 4000.0, 1e-9, "the pyramid");
    let faces = faces_of(&e, "pyramid");
    assert!(faces.contains("pyramid.start") && !faces.contains("pyramid.end"), "{faces:?}");
    let e = family(&PYRAMID.replace("over s in (0, 1)", "over s in (0, 0.5)"));
    near(volume(&e, "pyramid").unwrap(), 3500.0, 1e-9, "the frustum");
}

/// **A square lofted to a circle** as one section: a rounded square whose corner radius runs from
/// 0 to 10 while its flats run from 20 to nothing.  At `s = 0` the corners are arcs of radius 0
/// and the station solves; at `s = 1` the flats are points.  area(s) = 400(1 − s)² + 800s(1 − s)
/// + 100πs², so V = 8000 + 1000π mm³.  Its corners' walls start from an edge of no length.
#[test]
fn a_square_lofts_to_a_circle() {
    let e = family(SQUARE_TO_CIRCLE);
    near(volume(&e, "transition").unwrap(), 8000.0 + 1000.0 * PI, 1e-9, "square to circle");
}

/// **A spring and a thread**: a wire of radius 1.5 square to a helix of radius 10, pitch 5, four
/// turns, its centroid on the helix, is A × the helix's length, π · 1.5² · 4√((20π)² + 5²); a 2 mm
/// axial square under a screw of pitch 4, three turns, is Pappus, 4 · 2π · 10 · 3 = 240π.
#[test]
fn a_spring_and_a_thread() {
    let e = family(SPRING);
    let helix = 4.0 * ((20.0 * PI).powi(2) + 25.0).sqrt();
    near(volume(&e, "spring").unwrap(), PI * 1.5 * 1.5 * helix, 1e-9, "the spring");
    near(volume(&e, "thread").unwrap(), 240.0 * PI, 1e-9, "the thread");
}

/// **Two rails hold the width**: the blade's edges reach rails drawn in plan, so its width runs
/// linearly from 20 to 8 over 50 mm and V = 3 · 50 · 14 = 2100 mm³, every wall flat.
#[test]
fn two_rails_hold_a_blade() {
    let e = family(BLADE);
    near(volume(&e, "blade").unwrap(), 2100.0, 1e-9, "the blade");
}

/// **A smooth loft is its quadrature**: the bottle's circles reach the spline through its three
/// rims (L7), so V = π∫ x² dy over the rail — on each of its two spans a polynomial of degree 8 in
/// the rail's parameter, which five Gauss–Legendre points integrate exactly.  The rail is a
/// clamped cubic of five control points, read off the solve.
#[test]
fn a_smooth_loft_is_its_quadrature() {
    let e = family(BOTTLE);
    let sk = &e.sketch;
    let rail = e.map.ent_named("rail").unwrap();
    assert_eq!(rail.kind, EntKind::Spline);
    assert_eq!(sk.splines[rail.i()].ctrl.len(), 5, "a cubic of five control points");
    assert_eq!(gcs_core::curve::domain(sk, rail.i()), (0.0, 2.0), "a span a pair of sections");
    // five-point Gauss–Legendre on (−1, 1)
    let (inner, outer) = ((5.0 - 2.0 * (10.0f64 / 7.0).sqrt()).sqrt() / 3.0,
                          (5.0 + 2.0 * (10.0f64 / 7.0).sqrt()).sqrt() / 3.0);
    let (w1, w2) = ((322.0 + 13.0 * 70f64.sqrt()) / 900.0, (322.0 - 13.0 * 70f64.sqrt()) / 900.0);
    let gauss = [(0.0, 128.0 / 225.0), (-inner, w1), (inner, w1), (-outer, w2), (outer, w2)];
    let want: f64 = (0..2).map(|span| gauss.iter().map(|&(x, w)| {
        let f = gcs_core::curve::eval(sk, rail.i(), span as f64 + 0.5 * (x + 1.0));
        0.5 * w * PI * f.p.0.powi(2) * f.d1.1
    }).sum::<f64>()).sum();
    near(volume(&e, "bottle").unwrap(), want, 1e-9, "the bottle");
}

/// **Walls are named by the section's edges** under its instance (finding 12): the reducer's
/// outer loop's sides as the face names them, its hole's by the loop they came from, all under
/// the section's path `sec`; the caps are `start` and `end`.  A name never depends on position.
#[test]
fn walls_are_named_by_the_sections_edges() {
    let e = family(REDUCER);
    let faces = faces_of(&e, "body");
    let want = ["body.start", "body.end", "body.sec.ab", "body.sec.bc", "body.sec.cd",
                "body.sec.da", "body.sec.inner.loop.ab", "body.sec.inner.loop.bc",
                "body.sec.inner.loop.cd", "body.sec.inner.loop.da"];
    assert_eq!(faces.len(), want.len(), "{faces:?}");
    for w in want {
        assert!(faces.contains(w), "no `{w}` in {faces:?}");
    }
    let e = family(SQUARE_TO_CIRCLE);
    let faces = faces_of(&e, "transition");
    for w in ["transition.k0", "transition.top", "transition.right", "transition.end"] {
        assert!(faces.contains(w), "no `{w}` in {faces:?}");
    }
}

/// **The station is written in place and never drawn**: solved with the drawing (the document is
/// DOF 0 with it), but no stroke, no pane and no callout of it reaches the page.
#[test]
fn the_station_is_never_drawn() {
    let mut e = family(REDUCER);
    let SolidDef::Family { station, .. } = &e.sketch.solids[index(&e, "body")].def else {
        panic!("a face family");
    };
    let station = station.clone();
    assert!(station.iter().any(|s| s.kind == EntKind::Frame), "the station's plane is its");
    let drawn = e.sketch.drawn();
    assert!(!drawn.iter().any(|d| station.contains(d)), "a station's entity is drawn");
    assert!(drawn.iter().any(|d| d.kind == EntKind::Line), "the guide is drawn");
    let dimensioned: Vec<_> = gcs_core::callout::layout(&e.sketch, 1.0).into_iter()
        .filter_map(|k| e.sketch.constraints.iter().find(|c| c.id == k.id))
        .filter(|c| c.entities().iter().any(|x| station.contains(x)))
        .map(|c| c.kind)
        .collect();
    assert!(dimensioned.is_empty(), "a station's dimension is drawn: {dimensioned:?}");
    use gcs_core::diagnose::{diagnose, DiagnoseOptions};
    let d = diagnose(&mut e.sketch, DiagnoseOptions::default());
    assert_eq!(d.dof, 0, "the drawn station is held");
}

/// **Only an end may lose rank**: the pyramid run past its apex, `s` over (0, 1.5), meets the
/// section shrunk to nothing inside the interval, which is refused where it happens — as the rank
/// lost there where a station stands at it, or as the section turned over between the stations
/// either side of it where none does (past its apex the square is drawn inside out).
#[test]
fn a_station_that_loses_rank_inside_is_refused_there() {
    let e = family(&PYRAMID.replace("over s in (0, 1)", "over s in (0, 1.5)"));
    let m = refused(&e, "pyramid");
    let number = |t: &str| t.split(|c: char| !(c.is_ascii_digit() || c == '.')).next()
        .and_then(|n| n.parse::<f64>().ok());
    let there = |w: &str| m.split(w).nth(1).and_then(number);
    match (m.contains("not held"), m.contains("turns over")) {
        (true, _) => {
            assert!(there("the station at ").is_some_and(|s| (s - 1.0).abs() < 1e-3), "{m}")
        }
        (_, true) => {
            let (a, b) = (there("between the stations at ").unwrap(), there(" and ").unwrap());
            assert!(a < 1.0 && 1.0 < b && b - a < 0.2, "{m}");
        }
        _ => panic!("{m}"),
    }
}

/// **The swept volume may not cross itself**: the elbow's 18 mm section bent on a radius of 8
/// reaches past the bend's axis, which a bend's section may not.
#[test]
fn a_section_reaching_past_its_bend_is_refused() {
    let e = family(&ELBOW.replace("bend_radius := 30mm", "bend_radius := 8mm"));
    let m = refused(&e, "body");
    assert!(m.contains("crosses itself"), "{m}");
}

/// **What a family is written over, refused where it is written**: its formal bound by the call,
/// a formal the component has not, an interval run backwards, a drawn station (L10's).
#[test]
fn a_family_is_written_over_an_instance_in_place() {
    let bound = REDUCER.replace("wall: 2mm).sec", "wall: 2mm, s: 0.5).sec");
    let (_, said) = read(&bound);
    assert!(said.iter().any(|m| m.contains("runs over the family's interval")), "{said:?}");
    let (_, said) = read(&REDUCER.replace("over s in (0, 1)", "over t in (0, 1)"));
    assert!(said.iter().any(|m| m.contains("has no formal `t`")), "{said:?}");
    let (_, said) = read(&REDUCER.replace("over s in (0, 1)", "over s in (1, 0)"));
    assert!(said.iter().any(|m| m.contains("runs nowhere or backwards")), "{said:?}");
    let drawn = REDUCER.replace("body := solid(Reducer(", "st := Reducer(")
        .replace(").sec.profile over s in (0, 1))",
                 ")\nbody := solid(st.sec.profile over s in (0, 1))");
    let (_, errs) = parse(&drawn);
    assert!(errs.iter().any(|e| e.message.contains("written in place")), "{errs:?}");
}

/// **A family inside a component** is the component's like any other solid: its running number
/// is named under the instance's prefix, and the reducer drawn as part of a larger part sweeps
/// the same 5120 mm³.
#[test]
fn a_family_inside_a_component() {
    let written = "body := solid(Reducer(guide, std.z, inlet: 24mm, outlet: 12mm, wall: 2mm)\
                   .sec.profile over s in (0, 1))";
    assert!(REDUCER.contains(written));
    let inside = written.replace("std.z", "up");
    let wrapped = REDUCER.replace(written, "part := Part(guide, std.z)")
        .replace("in std.top {", &format!("component Part(guide: line, up: axis) {{\n  \
                                           {inside}\n}}\nin std.top {{"));
    let e = family(&wrapped);
    near(volume(&e, "part.body").unwrap(), 5120.0, 1e-9, "the reducer in a part");
    assert!(e.sketch.held.keys().all(|n| n.starts_with("part.#f")), "{:?}", e.sketch.held.keys());
}

/// A disc of the radius of a circle drawn beside the guide, shrinking to half of it as it runs
/// 30 mm: a station reading the geometry it was given.
const CONE: &str = "\
unit mm
use std

component Disc(g: line, up: axis, rim: circle, s: Scalar) {
  st := std.OnLine(g, up: up, s: s)
  in st.n {
    k := circle(center: st.n.origin)
    radius(rim.r * (1 - s / 2)) k
  }
  f := face(k)
}

in std.top {
  o := point
  fix((0mm, 0mm)) o
  e := point
  fix((0mm, 30mm)) e
  guide := line(o, e)
  c := point
  fix((50mm, 0mm)) c
  ring := circle(center: c)
  radius(10mm) ring
}
cone := solid(Disc(guide, std.z, ring).f over s in (0, 1))
";

/// **A station reads the geometry it was given** as a traced body does (finding 1): the disc's
/// radius is the ring's, read where the solve put it (the ring's seed is not 10), shrinking to
/// half — a frustum, π · 30/3 · (10² + 10 · 5 + 5²) = 1750π, its wall a cone.
/// Moving the ring moves every station; reading the station's own geometry is refused.
#[test]
fn a_station_reads_the_geometry_it_was_given() {
    let e = family(CONE);
    near(volume(&e, "cone").unwrap(), 1750.0 * PI, 1e-9, "the frustum");
    let e = family(&CONE.replace("radius(10mm) ring", "radius(8mm) ring"));
    near(volume(&e, "cone").unwrap(), 1750.0 * PI * 0.64, 1e-9, "the ring made smaller");
    let own = CONE.replace("radius(rim.r * (1 - s / 2)) k", "radius(rim.r * (1 - s / 2)) k\n    \
                           q := point\n    q distance(k.r) st.n.origin");
    let (_, said) = read(&own);
    assert!(said.iter().any(|m| m.contains("station's own geometry")), "{said:?}");
}


/// **The fast path is what the numbers say** (#243 L2): a wall lies on a plane, a cylinder, a cone
/// or a torus where its edge's numbers keep to one — never by what wrote them — on the surface its
/// first rim sweeps where the section is carried rigidly (L6), and on the sheet the station sweeps
/// (`Surface::Family`) everywhere else.  The reducer and the pyramid are all planes; the elbow's
/// flat walls are planes and its curved ones cylinders about the bend; the disc shrinking is a
/// cone; the twisted duct, screwed rigidly along its guide, is blends; the square running to a
/// circle keeps sheets.  Each lowered wall was held to its sheet (its points, and its area) before
/// it was kept.
#[test]
fn walls_lie_on_the_surfaces_their_numbers_say() {
    let all = |e: &Elaborated, name: &str, want: &[&str]| {
        let k = kinds(e, name);
        let cap = |n: &str| n.ends_with(".start") || n.ends_with(".end");
        let walls: Vec<&str> = k.iter().filter(|(n, _)| !cap(n)).map(|(_, &kind)| kind).collect();
        assert!(!walls.is_empty() && walls.iter().all(|k| want.contains(k)), "{name}: {k:?}");
        k
    };
    all(&family(REDUCER), "body", &["plane"]);
    all(&family(PYRAMID), "pyramid", &["plane"]);
    let k = all(&family(ELBOW), "body", &["plane", "cylinder"]);
    assert_eq!(k.values().filter(|&&v| v == "cylinder").count(), 4, "{k:?}");
    all(&family(CONE), "cone", &["cone"]);
    all(&family(TWISTED), "duct", &["blend"]);
    let k = kinds(&family(SQUARE_TO_CIRCLE), "transition");
    assert!(k.values().any(|&v| v == "family") && k.values().any(|&v| v == "plane"), "{k:?}");
}

/// **A family is a solid like any other** (#243 L2): the 60 × 20 × 60 slab cut by the twisted
/// duct running through it loses the duct's 20² · 20 there — 72000 − 8000 = 64000 mm³ — the
/// slab's faces cut along curves traced where they meet the duct's twisted walls (blends since
/// L6: the duct is its section screwed).
#[test]
fn a_family_cuts_like_any_solid() {
    // the duct alone: the auger, written last, left off
    let auger = TWISTED.find("// Three and a half turns").expect("the auger");
    let src = TWISTED[..auger].to_string() + "in std.top {\n  m := point\n  fix((0mm, 20mm)) m\n}\n\
        r := std.CenteredRectangle(m, w: 60mm, h: 20mm) in std.top\nslab := face(r.loop)\n\
        block := solid(slab, from: -30mm, to: 30mm)\nduct cut block\n";
    let e = family(&src);
    near(volume(&e, "block").unwrap(), 64000.0, 1e-9, "the slab less the duct");
    let k = kinds(&e, "block");
    assert!(k.iter().any(|(n, &v)| n.starts_with("duct.") && v == "blend"), "{k:?}");
    assert!(!k.contains_key("duct.start") && !k.contains_key("duct.end"), "{k:?}");
}

/// **And it is written as STEP**: each sheet as the net fitted within the file's tolerance, its
/// rails and rims likewise, and the file read back describes the solid (`step_check::verify`).
#[test]
fn a_family_is_written_as_step() {
    for (src, name) in [(TWISTED, "duct"), (ELBOW, "body"), (SPRING, "spring")] {
        let e = family(src);
        let x = e.sketch.exact_solid(index(&e, name)).unwrap();
        let text = gcs_core::brep::step::write(&x.brep, name, 1e-3).unwrap();
        let solid = gcs_core::brep::step_check::Solid::of(&x.brep);
        gcs_core::brep::step_check::verify(&text, &solid).unwrap_or_else(|m| panic!("{name}: {m}"));
    }
}

/// **The solid the page shows is the exact one**: the evaluated elbow is its B-rep meshed, its
/// volume the B-rep's own, and its walls named by the section's edges.
#[test]
fn the_evaluated_family_is_its_exact_brep() {
    let e = family(ELBOW);
    let s = e.sketch.evaluated_solid(index(&e, "body"), Report).unwrap();
    assert_eq!(s.volume(), volume(&e, "body").unwrap());
    assert!(s.surviving_faces().iter().any(|f| f.starts_with("body.") && !f.ends_with("start")));
}
