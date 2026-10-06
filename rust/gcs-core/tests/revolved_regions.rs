use gcs_core::{envelope::Error,model::SolidDef,program,
    solid::{RegionLocation,RevolvedRegion},solve};

fn solved(source: &str) -> program::Elaborated {
    let (p,errors) = crate::common::parse(source);
    assert!(errors.is_empty(),"{errors:?}");
    let mut e = program::elaborate(&p);
    assert!(e.ok(),"{:?}",e.diags);
    assert!(solve::solve(&mut e.sketch,Default::default()).success);
    e
}

const RING: &str = "\
unit mm
use std
in std.front {
o := point
q := point
c := point
fix((0, 0)) o
fix((0, 1)) q
fix((3, 0)) c
ax := line(o,q)
rim := circle(center: c)
radius(1mm) rim
}
body := solid(face(rim), about: ax)
";

const SPHERE: &str = "\
unit mm
use std
in std.front {
o := point
a := point
b := point
fix((0, 0)) o
fix((0, -2)) a
fix((0, 2)) b
rim := arc(center: o, start: a, end: b)
radius(2mm) rim
ax := line(b,a)
}
body := solid(face(rim,ax), about: ax)
";

fn check(region: &RevolvedRegion,p: [f64;3],expected: f64) {
    let s = region.classify(p,1e-10).unwrap();
    let want = if expected.abs() <= 1e-10 { RegionLocation::Boundary }
        else if expected < 0. { RegionLocation::Inside } else { RegionLocation::Outside };
    assert_eq!(s.location,want,"at {p:?}: {s:?}, expected {expected}");
    assert!((s.signed_distance.abs()-expected.abs()).abs() < 2e-12,
        "at {p:?}: {s:?}, expected {expected}");
    if want != RegionLocation::Boundary {
        assert!((s.signed_distance-expected).abs() < 2e-12);
    }
}

#[test]
fn torus_membership_and_distance_follow_analytic_circles_including_ray_tangencies() {
    let e = solved(RING);
    let region = RevolvedRegion::read(&e.sketch,0,1e-12).unwrap();
    for r in 0..=40 {
        for z in -20..=20 {
            let r = f64::from(r)/8.; let z = f64::from(z)/8.;
            for angle in [0_f64,0.7,2.8] {
                check(&region,[r*angle.cos(),r*angle.sin(),z],(r-3.).hypot(z)-1.);
            }
        }
    }
}

#[test]
fn holes_are_voids_and_source_axis_direction_does_not_change_material() {
    let source = RING.replace("body := solid(face(rim)",
        "hole := circle(center: c)\nradius(0.4mm) hole\nbody := solid(face(rim, holes: hole)");
    for source in [source.clone(),source.replace("ax := line(o,q)","ax := line(q,o)")] {
        let e = solved(&source);
        let region = RevolvedRegion::read(&e.sketch,0,1e-12).unwrap();
        for r in 0..=40 {
            for z in -10..=10 {
                let r = f64::from(r)/8.; let z = f64::from(z)/8.;
                let d = (r-3.).hypot(z);
                let expected = if d < 0.4 { 0.4-d } else if d > 1. { d-1. }
                    else { -(d-0.4).min(1.-d) };
                check(&region,[r,0.,z],expected);
            }
        }
    }
}

#[test]
fn a_spheres_diameter_disappears_and_its_center_is_interior_material() {
    for source in [SPHERE.to_string(),SPHERE.replace("ax := line(b,a)","ax := line(a,b)"),
        SPHERE.replace("fix((0, -2)) a","fix((-2, 0)) a")
            .replace("fix((0, 2)) b","fix((2, 0)) b")] {
        let e = solved(&source);
        let region = RevolvedRegion::read(&e.sketch,0,1e-12).unwrap();
        for x in -20..=20 {
            for z in -20..=20 {
                let x = f64::from(x)/8.; let z = f64::from(z)/8.;
                check(&region,[x,0.,z],x.hypot(z)-2.);
            }
        }
        check(&region,[0.,0.,0.],-2.);
        check(&region,[0.,0.,2.],0.);
    }
}

#[test]
fn finite_conical_material_includes_caps_and_refuses_support_continuations() {
    let e = solved("\
unit mm
use std
in std.front {
a := point
b := point
c := point
d := point
fix((0, 1)) a
fix((1, 1)) b
fix((3, 3)) c
fix((0, 3)) d
ax := line(a,d)
profile := (ab := line(a,b)) -> (bc := line(b,c)) -> (cd := line(c,d)) -> (da := line(d,a)) -> close
}
body := solid(profile,about: ax)
");
    let region = RevolvedRegion::read(&e.sketch,0,1e-12).unwrap();
    for (p,d) in [([0.,0.,2.],-1.),([1.,0.,2.],-std::f64::consts::FRAC_1_SQRT_2),
        ([0.,0.,4.],1.),([0.,0.,0.5],0.5),([2.,0.,3.],0.),([0.5,0.,1.],0.)] {
        check(&region,p,d);
    }
}

#[test]
fn material_queries_are_snapshots_and_refuse_unsupported_or_nonfinite_geometry() {
    let mut e = solved(RING);
    let region = RevolvedRegion::read(&e.sketch,0,1e-12).unwrap();
    let r = e.sketch.circles[0].radius as usize;
    e.sketch.params[r].value = 2.;
    let changed = RevolvedRegion::read(&e.sketch,0,1e-12).unwrap();
    check(&region,[4.5,0.,0.],0.5);
    check(&changed,[4.5,0.,0.],-0.5);
    let SolidDef::Revolve {sweep,..} = &mut e.sketch.solids[0].def else { panic!() };
    sweep.value = std::f64::consts::TAU-1e-8;
    assert!(RevolvedRegion::read(&e.sketch,0,1e-12).unwrap_err().contains("full revolution"));
    assert!(RevolvedRegion::read(&e.sketch,999,1e-12).is_err());
    assert!(RevolvedRegion::read(&e.sketch,0,-1.).is_err());
    assert!(RevolvedRegion::read(&e.sketch,0,f64::NAN).is_err());
    assert_eq!(region.classify([f64::NAN,0.,0.],0.).unwrap_err(),Error::NonFinite);
    assert!(region.classify([0.;3],-1.).is_err());
    assert!(region.classify([0.;3],f64::INFINITY).is_err());
    let e = solved(&RING.replace("about: ax","depth: 2mm"));
    assert!(RevolvedRegion::read(&e.sketch,0,1e-12).unwrap_err().contains("unmodified revolution"));
}
