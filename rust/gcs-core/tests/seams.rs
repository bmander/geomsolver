//! Seams, corners, edges and face boundaries made of their parts (#364): the post-solve B-rep
//! vocabulary is Rust API, built over the envelopes and surfaces a drawing names, as the spiral
//! bevel's checks build it (`envelope/paired.rs`).
use gcs_core::{envelope::{Error,GeneratedEnvelope},patch::TrimmedPatch,program,
    seam::{EnvelopeSeam,SeamIntersectionOptions,SeamTolerance},
    solid::{RevolvedRegion,RevolvedSurface,SurfaceProjector}};
use crate::common::parsed::determined as solved;

mod boundary;
mod surfaces;
mod vertices;

const MODEL: &str = "\
unit mm
use std
use std.place (vertical)
use std.motion (turns)
in std.front {
o := point
q := point
x := point
a := point
b := point
m := point
c := point
d := point
fix((0, 0)) o
fix((0, 2)) q
fix((1, 0)) x
fix((2, 0)) a
fix((3, 0)) b
fix((3, 1)) m
fix((3, 2)) c
fix((2, 2)) d
ax := line(o,q)
spin_axis := line(o,x)
bottom := line(a,b)
low := line(b,m)
high := line(m,c)
top := line(c,d)
inner := line(d,a)
}
profile := face(bottom,low,high,top,inner)
body := solid(profile,about: ax)
first_surface := surface(body,low,from: 0deg,to: 90deg)
second_surface := surface(body,high,from: 0deg,to: 90deg)
roll := frame
roll turns(1) spin_axis
first_envelope := envelope(first_surface over t in (-20deg, 20deg), under: roll)
second_envelope := envelope(second_surface over t in (-20deg, 20deg), under: roll)
";

fn index(e: &program::Elaborated,name: &str) -> usize {
    e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`")).i()
}
fn envelope(e: &program::Elaborated,name: &str) -> GeneratedEnvelope {
    GeneratedEnvelope::named(&e.sketch,index(e,name)).unwrap()
}
fn surface(e: &program::Elaborated,name: &str) -> RevolvedSurface {
    RevolvedSurface::named(&e.sketch,index(e,name)).unwrap()
}
fn projector(e: &program::Elaborated,name: &str) -> SurfaceProjector {
    SurfaceProjector::named(&e.sketch,index(e,name)).unwrap()
}
/// The material of the solid the surface `name` is a side of.
fn region(e: &program::Elaborated,name: &str) -> RevolvedRegion {
    RevolvedRegion::read(&e.sketch,e.sketch.surfaces[index(e,name)].solid as usize,1e-10).unwrap()
}
/// The envelope `name`, kept within the material of each surface's solid in `inside`.
fn face(e: &program::Elaborated,name: &str,inside: &[&str]) -> TrimmedPatch {
    TrimmedPatch::of_envelope(name.into(),envelope(e,name),
        inside.iter().map(|s| region(e,s)).collect(),vec![])
}
fn tolerance() -> SeamTolerance { SeamTolerance {position:1e-9,normal:1e-9,axis:1e-10} }
/// The two envelopes' shared characteristic: `low` ends at `m` (`u` 1) where `high` starts (0).
fn shared_with(e: &program::Elaborated,t: SeamTolerance) -> Result<EnvelopeSeam,String> {
    EnvelopeSeam::new("shared".into(),[&face(e,"first_envelope",&[]),&face(e,"second_envelope",&[])],
        [1.,0.],t)
}
fn shared(e: &program::Elaborated) -> EnvelopeSeam { shared_with(e,tolerance()).unwrap() }
fn options(seam: &EnvelopeSeam) -> SeamIntersectionOptions {
    let [_,v,roll] = seam.domain();
    SeamIntersectionOptions {bounds:[v,roll],parameter_scale:[1.;2],
        normal_tolerance:1e-10,section_tolerance:1e-10,max_iterations:100}
}

#[test]
fn a_shared_characteristic_checks_both_envelopes_at_the_ends_it_is_given() {
    let e = solved(MODEL);
    let seam = shared(&e);
    assert_eq!(seam.endpoint_parameters(),[1.,0.]);
    assert_eq!(seam.domain()[0],[1.,1.]);
    let c = seam.evaluate([1.,0.,0.1],1e-10,1e-10).unwrap();
    let expected = [3.,-0.1f64.sin(),0.1f64.cos()];
    for i in 0..3 { assert!((c.position[i]-expected[i]).abs() < 1e-10); }
    assert_eq!(seam.evaluate([1.,0.1,0.1],1e-10,1e-10).unwrap_err(),Error::OutsideDomain);
    assert_eq!(seam.evaluate([0.,0.,0.1],1e-10,1e-10).unwrap_err(),Error::OutsideDomain);
    let p = seam.intersect(|c| c.position[1]+0.1f64.sin(),[0.05,0.],options(&seam),1e-9).unwrap();
    for i in 0..3 { assert!((p.contact.position[i]-expected[i]).abs() < 1e-9); }
    assert!((p.parameters[2]-0.1).abs() < 1e-9);
}

#[test]
fn a_seam_is_a_snapshot_of_its_parts() {
    let mut e = solved(MODEL);
    let original = shared(&e);
    let m = e.map.ent_named("m").unwrap();
    let y = e.sketch.points[m.i()].y as usize;
    e.sketch.params[y].value = 1.25;
    let changed = shared(&e);
    assert!((changed.evaluate([1.,0.,0.],1e-9,1e-9).unwrap().position[2]-1.25).abs() < 1e-10);
    assert!((original.evaluate([1.,0.,0.],1e-9,1e-9).unwrap().position[2]-1.).abs() < 1e-10);
    let copied = original.clone();
    assert_eq!(copied.evaluate([1.,0.,0.1],1e-9,1e-9).unwrap().position,
        original.evaluate([1.,0.,0.1],1e-9,1e-9).unwrap().position);
}

#[test]
fn a_seam_refuses_ends_that_meet_nowhere_nontangent_junctions_and_disjoint_domains() {
    let e = solved(MODEL);
    let faces = [face(&e,"first_envelope",&[]),face(&e,"second_envelope",&[])];
    // The ends are the caller's: a pair of ends that is no shared vertex is refused, never
    // repaired by looking for the nearest one.
    let error = EnvelopeSeam::new("gap".into(),[&faces[0],&faces[1]],[0.,0.],tolerance()).unwrap_err();
    assert!(error.contains("position error"),"{error}");
    // `high` and `top` meet at `c`, but at a right angle.
    let e = solved(&MODEL.replace("first_surface := surface(body,low,","first_surface := surface(body,high,")
        .replace("second_surface := surface(body,high,","second_surface := surface(body,top,"));
    assert!(shared_with(&e,tolerance()).unwrap_err().contains("tangent"));
    let e = solved(&MODEL.replace("second_surface := surface(body,high,from: 0deg,to: 90deg)",
        "second_surface := surface(body,high,from: 180deg,to: 270deg)"));
    assert!(shared_with(&e,tolerance()).unwrap_err().contains("overlapping"));
    // A seam is of two generating faces: a trimmed surface is not one.
    let e = solved(MODEL);
    let wall = TrimmedPatch::of_surface("wall".into(),surface(&e,"first_surface"),vec![],vec![]);
    assert!(EnvelopeSeam::new("bad".into(),[&wall,&faces[1]],[1.,0.],tolerance()).is_err());
}

#[test]
fn a_retained_seam_must_satisfy_material_trims_on_both_faces() {
    let source = format!("{MODEL}
component Sphere(o: point,size: Length) {{
  private bottom := point
  private top := point hint((0, size))
  fix((0, -size)) bottom
  fix(x == 0) top
  private rim := arc(center: o,start: bottom,end: top)
  private diameter := line(top,bottom)
  ball := solid(face(rim,diameter),about: diameter)
  wall := surface(ball,rim)
}}
large := Sphere(o,size: 4mm) in std.front
small := Sphere(o,size: 3.1mm) in std.front
");
    for (source,retained) in [(source.clone(),false),(source.replace("size: 3.1mm","size: 3.5mm"),true)] {
        let e = solved(&source);
        let seam = EnvelopeSeam::new("clipped".into(),[&face(&e,"first_envelope",&["large.wall"]),
            &face(&e,"second_envelope",&["small.wall"])],[1.,0.],tolerance()).unwrap();
        let evaluated = seam.evaluate([1.,0.,0.1],1e-9,1e-9);
        let intersected = seam.intersect(|c| c.position[1]+0.1f64.sin(),
            [0.05,0.],options(&seam),1e-9);
        assert_eq!(evaluated.is_ok(),retained,"{evaluated:?}");
        assert_eq!(intersected.is_ok(),retained,"{intersected:?}");
        if !retained { assert_eq!(intersected.unwrap_err(),Error::OutsideDomain); }
    }
}

#[test]
fn an_unresolved_arc_radius_cannot_hide_a_gap_at_a_shared_vertex() {
    let source = MODEL.replace("fix((2, 0)) a","fix((1, 0)) a")
        .replace("fix((3, 2)) c","fix((2, 2)) c")
        .replace("fix((2, 2)) d","fix((1, 2)) d")
        .replace("high := line(m,c)","arc_center := point\nfix((2, 1)) arc_center\n\
            high := arc(center: arc_center,start: m,end: c)\nradius(1mm) high");
    let mut e = solved(&source);
    assert!(shared_with(&e,tolerance()).is_ok());
    let arc = e.map.ent_named("high").unwrap();
    let radius = e.sketch.arcs[arc.i()].radius as usize;
    e.sketch.params[radius].value += 1e-4;
    let error = shared_with(&e,tolerance()).unwrap_err();
    assert!(error.contains("position error"),"{error}");
}

#[test]
fn a_stationary_family_does_not_produce_a_falsely_unique_seam_intersection() {
    let e = solved(&MODEL.replace("roll turns(1) spin_axis",
        "roll turns(0) spin_axis"));
    let seam = shared(&e);
    let result = seam.intersect(|c| c.position[1]-1.,[0.05,0.],options(&seam),1e-9);
    assert_eq!(result.unwrap_err(),Error::SingularIntersection);
}

#[test]
fn seam_evaluation_rejects_invalid_numerical_controls() {
    let e = solved(MODEL);
    let mut bad = tolerance(); bad.normal = 1.;
    assert!(shared_with(&e,bad).is_err());
    bad = tolerance(); bad.axis = f64::NAN;
    assert!(shared_with(&e,bad).is_err());
    let seam = shared(&e);
    assert!(seam.evaluate([1.,0.,0.],-1.,1e-9).is_err());
    assert!(seam.evaluate([1.,0.,0.],1e-9,f64::NAN).is_err());
    assert!(seam.evaluate([1.,0.,f64::NAN],1e-9,1e-9).is_err());
    let mut bad = options(&seam); bad.bounds[0] = [0.,1.];
    assert_eq!(seam.intersect(|_| 0.,[0.,0.],bad,1e-9).unwrap_err(),Error::OutsideDomain);
    for tolerance in [-1.,f64::NAN,f64::INFINITY] {
        assert_eq!(seam.intersect(|_| panic!("invalid controls must precede callbacks"),
            [0.,0.],options(&seam),tolerance).unwrap_err(),Error::InvalidOptions);
    }
}
