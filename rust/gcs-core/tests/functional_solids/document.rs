//! Ordinary solved source drives the same continuous material evaluator as the workbench.
use super::*;
use gcs_core::{program,solve,solid::{RevolvedRegion,SweptField},interval::minimum::{Options,Status}};

fn read(source: &str) -> program::Elaborated {
    let (mut p,errors) = crate::common::parse(source);
    assert!(errors.is_empty(),"{errors:?}");
    // the spiral bevel's modules, where a document uses one
    assert!(gcs_core::modules::link(&mut p,&mut fixtures::gear::module).is_empty());
    let mut e = program::elaborate(&p);
    assert!(e.ok(),"{:?}",e.diags);
    assert!(solve::solve(&mut e.sketch,Default::default()).success);
    e
}
fn field(e: &program::Elaborated,name: &str) -> SpatialField {
    SpatialField::read(&e.sketch,e.map.ent_named(name).unwrap().i(),1e-10).unwrap()
}
const SPHERE: &str = "\
unit mm
use std
in std.front {
o := point
a := point
b := point
fix(x == 3, y == 0) o
fix(x == 3, y == -1) a
fix(x == 3, y == 1) b
rim := arc(center: o,start: a,end: b)
radius(1) rim
ax := line(b,a)
ball := solid(face(rim,ax),about: ax)
z := point
q := point
fix(x == 0, y == 0) z
fix(x == 0, y == 1) q
spindle := line(z,q)
}
indexing := motion(about: spindle)
";

#[test]
fn source_sphere_sweep_matches_a_torus_and_keeps_snapshot_geometry() {
    let mut e = read(SPHERE);
    let ball = e.map.ent_named("ball").unwrap().i();
    let motion = e.map.ent_named("indexing").unwrap().i();
    let sweep = SweptField::read(&e.sketch,ball,motion,I::new(-4.,4.).unwrap(),1e-10).unwrap();
    let mut query = sweep.evaluator(128);
    let options = Options {value_tolerance:1e-4,max_evaluations:50000};
    for p in [[0_f64,0.,0.],[3.,0.,0.],[0.,3.,0.],[4.2,0.,0.3],[2.,1.,2.]] {
        let got = query.bounds(point(p),options).unwrap();
        assert_eq!(got.status,Status::Converged);
        super::sweeps::includes(got.value,(p[0].hypot(p[1])-3.).hypot(p[2])-1.);
    }
    let old = field(&e,"ball");
    let radius = e.sketch.arcs[0].radius as usize;
    e.sketch.params[radius].value = 0.5;
    for (name,y) in [("a",-0.5),("b",0.5)] {
        let i = e.map.ent_named(name).unwrap().i();
        let parameter = e.sketch.points[i].y as usize;
        e.sketch.params[parameter].value = y;
    }
    let changed = field(&e,"ball");
    assert!(old.bounds(point([3.75,0.,0.])).unwrap().bounds()[1] < -0.24);
    assert!(changed.bounds(point([3.75,0.,0.])).unwrap().bounds()[0] > 0.24);
}

#[test]
fn source_bodies_holes_and_nested_placements_keep_material_and_finite_support() {
    let e = read(&format!("{SPHERE}\nfirst := solid(ball,under: indexing,at: 45deg)\n\
        moved := solid(first,under: indexing,at: 45deg)\nbody := solid(ball)\nmoved union body\n"));
    let body = field(&e,"body");
    for p in [[3.,0.,0.],[0.,3.,0.]] { assert!(body.bounds(point(p)).unwrap().bounds()[1] < -0.99); }
    assert!(body.bounds(point([0.;3])).unwrap().bounds()[0] > 1.99);
    assert!(body.support_bounds().unwrap().is_some());
    let e = read("\
unit mm
use std
in std.front {
o := point
z := point
c := point
fix(x == 0, y == 0) o
fix(x == 0, y == 1) z
fix(x == 3, y == 0) c
ax := line(o,z)
outer := circle(center: c)
radius(1) outer
inner := circle(center: c)
radius(0.4) inner
}
body := solid(face(outer,holes: inner),about: ax)
");
    let body = field(&e,"body");
    for x in 0..=40 {
        let x = x as f64/8.; let d = (x-3.).abs();
        close(body.bounds(point([x,0.,0.])).unwrap(),(d-1.).max(0.4-d));
    }
}

#[test]
fn rounded_crown_source_matches_independent_analytic_material_queries() {
    let e = read(include_str!("../../../examples/spiral_bevel/crown/section.sv"));
    let crown = e.map.ent_named("crown").unwrap().i();
    let source = RevolvedRegion::read(&e.sketch,crown,1e-10).unwrap();
    let field = field(&e,"crown");
    let mut checked = 0;
    for r in 360..=480 { for z in -35..=35 {
        let p = [r as f64/10.,0.,z as f64/10.];
        let reference = source.classify(p,0.).unwrap().signed_distance;
        let got = field.bounds(point(p)).unwrap().bounds();
        if reference.abs() > 1e-7 {
            assert!(if reference < 0. { got[1] < 0. } else { got[0] > 0. },"{p:?}: {reference}, {got:?}");
            checked += 1;
        }
    } }
    assert!(checked > 8000);
}

#[test]
fn major_arcs_and_reversed_edges_keep_their_finite_sector() {
    let source = "\
unit mm
use std
in std.front {
o := point
z := point
c := point
a := point
b := point
fix(x == 0, y == 0) o
fix(x == 0, y == 1) z
fix(x == 3, y == 0) c
fix(x == 4, y == 0) a
fix(x == 3, y == -1) b
ax := line(o,z)
round := arc(center: c,start: a,end: b)
radius(1) round
chord := line(a,b)
}
profile := face(round,chord)
body := solid(profile,about: ax)
";
    for source in [source.to_string(),source.replace("profile := face(round,chord)","profile := face(chord,round)"),
        source.replace("ax := line(o,z)","ax := line(z,o)")] {
        let e = read(&source);
        let field = field(&e,"body");
        for ir in 10..=50 { for iz in -20..=20 {
            let r = ir as f64/10.; let z = iz as f64/10.;
            let expected = (r-3.).hypot(z)-1.;
            let expected = expected.max((r-4.-z)/2_f64.sqrt());
            let got = field.bounds(point([r,0.,z])).unwrap().bounds();
            if expected.abs() > 1e-8 {
                assert!(if expected < 0. { got[1] < 0. } else { got[0] > 0. },
                    "{r},{z}: {expected}, {got:?}");
            }
        } }
    }
}

/// A member's blank as its own step draws it (`spiral_bevel/blank/member.sv`): the pitch
/// generator 50 along the page's x from the apex, the face width 10 about its mean point.
#[test]
fn declarative_bevel_blank_reads_without_gear_names_or_coordinate_adapters() {
    let e = read(include_str!("../../../examples/spiral_bevel/blank/member.sv"));
    let blank = field(&e,"body");
    assert!(blank.support_bounds().unwrap().is_some());
    let rm = 50.;
    for radius in [0.8*rm,rm,1.2*rm] {
        let p = [radius,0.,0.];
        let got = blank.bounds(point(p)).unwrap().bounds();
        assert!(if radius == rm { got[1] < 0. } else { got[0] > 0. },"{p:?}: {got:?}");
    }
}

#[test]
fn partial_sweeps_fail_explicitly_while_prisms_and_concave_profiles_read() {
    let e = read(&SPHERE.replace("about: ax)","about: ax,sweep: 90deg)"));
    assert!(SpatialField::read(&e.sketch,0,1e-10).unwrap_err().contains("full revolution"));
    assert!(SpatialField::read(&e.sketch,999,1e-10).is_err());
    // The same half disc extruded 2 behind the page is a prism on x >= 3, y in [0, 2].
    let e = read(&SPHERE.replace("about: ax)","depth: 2)"));
    let half = SpatialField::read(&e.sketch,0,1e-10).unwrap();
    for (p,inside) in [([3.5,1.,0.],true),([2.5,1.,0.],false),([3.5,3.,0.],false),([3.9,0.5,0.],true)] {
        let got = half.bounds(point(p)).unwrap().bounds();
        assert!(if inside { got[1] < 0. } else { got[0] > 0. },"{p:?}: {got:?}");
    }
    // A reflex corner is no longer refused: the loop reads as its signed boundary distance.
    let e = read("\
unit mm
use std
in std.front {
o := point
z := point
fix(x == 0, y == 0) o
fix(x == 0, y == 1) z
ax := line(o,z)
a := point
b := point
c := point
d := point
e := point
fix(x == 1, y == 0) a
fix(x == 4, y == 0) b
fix(x == 4, y == 3) c
fix(x == 2, y == 1) d
fix(x == 1, y == 3) e
}
profile := face(a,b,c,d,e,-> close)
body := solid(profile,about: ax)
");
    let body = SpatialField::read(&e.sketch,0,1e-10).unwrap();
    for (p,inside) in [([3.,0.,0.5],true),([0.,1.4,2.],true),([2.2,0.,2.],false),([0.5,0.,1.],false)] {
        let got = body.bounds(point(p)).unwrap().bounds();
        assert!(if inside { got[1] < 0. } else { got[0] > 0. },"{p:?}: {got:?}");
    }
    // In the notch the nearest wall is the slanted edge c-d, an exact distance.
    let got = body.bounds(point([2.5,0.,2.])).unwrap().bounds();
    assert!(got[0] <= 0.125_f64.sqrt() && got[1] >= 0.125_f64.sqrt() && got[1]-got[0] < 1e-9,"{got:?}");
    assert!(body.support_bounds().unwrap().is_some());
}

#[test]
fn profile_connectivity_uses_shared_source_vertices_at_rounded_junctions() {
    let mut e = read(include_str!("../../../examples/spiral_bevel/crown/section.sv"));
    let crown = e.map.ent_named("crown").unwrap().i();
    // A tiny radius residual must not change the topology of shared endpoints.
    // The analytic field still uses the current radius, not a snapped curve.
    for arc in &e.sketch.arcs {
        e.sketch.params[arc.radius as usize].value += 2e-11;
    }
    let field = SpatialField::read(&e.sketch,crown,1e-10).unwrap();
    assert!(field.bounds(point([42.,0.,0.])).unwrap().bounds()[1] < 0.);
}
