//! Trimmed patches made of their parts (#364): a source surface or envelope the drawing names,
//! kept within some solids' material and outside others'.
use gcs_core::{envelope::{Error,GeneratedEnvelope,IntersectionOptions},patch::TrimmedPatch,program,
    solid::{RevolvedRegion,RevolvedSurface}};
use crate::common::parsed::determined as solved;

const SOURCE: &str = "\
unit mm
use std
use std.motion (turns)
in std.front {
o := point
q := point
x := point
c := point
fix((0, 0)) o
fix((0, 1)) q
fix((1, 0)) x
fix((3, 0)) c
ax := line(o,q)
spin_axis := line(o,x)
meridian := circle(center: c)
radius(1mm) meridian
}
ring := solid(face(meridian),about: ax)
wall := surface(ring,meridian)
roll := frame
roll turns(1) spin_axis
generated := envelope(wall over t in (-20deg, 20deg), under: roll)
component Sphere(o: point, size: Length) {
  private bottom := point
  private top := point hint((0, size))
  fix((0, -size)) bottom
  fix(x == 0) top
  private rim := arc(center: o,start: bottom,end: top)
  private ax := line(top,bottom)
  body := solid(face(rim,ax),about: ax)
}
in std.front {
inner := Sphere(o, size: 2.5mm)
outer := Sphere(o, size: 3.5mm)
}
";

fn index(e: &program::Elaborated,name: &str) -> usize { e.map.ent_named(name).unwrap().i() }
/// Within `outer.body`, outside `inner.body`.
fn trims(e: &program::Elaborated) -> (Vec<RevolvedRegion>,Vec<RevolvedRegion>) {
    let read = |name: &str| RevolvedRegion::read(&e.sketch,index(e,name),1e-10).unwrap();
    (vec![read("outer.body")],vec![read("inner.body")])
}
/// The torus `wall`, trimmed to the shell between the spheres.
fn bounded(e: &program::Elaborated) -> TrimmedPatch {
    let (inside,outside) = trims(e);
    TrimmedPatch::of_surface("bounded".into(),RevolvedSurface::named(&e.sketch,index(e,"wall")).unwrap(),inside,outside)
}
/// The envelope `generated`, trimmed to the same shell.
fn tooth(e: &program::Elaborated) -> TrimmedPatch {
    let (inside,outside) = trims(e);
    TrimmedPatch::of_envelope("tooth".into(),GeneratedEnvelope::named(&e.sketch,index(e,"generated")).unwrap(),inside,outside)
}
fn options() -> IntersectionOptions {
    IntersectionOptions {bounds:[[0.,0.4],[0.,0.2],[-0.2,0.2]],
        parameter_scale:[1.;3],residual_tolerance:[1e-10;3],max_iterations:100}
}

#[test]
fn patches_select_material_sides_and_enforce_the_source_locus() {
    let e = solved(SOURCE);
    let face = bounded(&e);
    let tooth = tooth(&e);
    assert!(face.surface_at(0.25,0.73,1e-10).is_ok());
    assert_eq!(face.surface_at(0.,0.,1e-10).unwrap_err(),Error::OutsideDomain);
    assert_eq!(face.surface_at(0.5,0.,1e-10).unwrap_err(),Error::OutsideDomain);
    assert!(tooth.envelope_at([0.25,0.,0.1],1e-10,1e-10).is_ok());
    // This trial satisfies the spherical trims, but not the envelope equation.
    assert_eq!(tooth.envelope_at([0.25,0.1,0.1],1e-10,1e-10).unwrap_err(),Error::OutsideDomain);
    assert!(tooth.envelope_at([0.25,0.,0.1],f64::NAN,1e-10).is_err());
    assert!(face.surface_at(0.25,0.,-1.).is_err());
    assert!(face.envelope_at([0.;3],1e-10,1e-10).is_err());
    assert!(tooth.surface_at(0.25,0.,1e-10).is_err());
}

#[test]
fn intersection_trials_may_cross_trims_but_a_converged_excluded_point_is_refused() {
    let e = solved(SOURCE);
    let tooth = tooth(&e);
    let result = tooth.intersect(|p,_| [p[0]-0.25,p[2]-0.1],
        [0.01,0.01,0.1],options(),1e-9).unwrap();
    assert!((result.parameters[0]-0.25).abs() < 1e-9);
    assert_eq!(tooth.intersect(|p,_| [p[0]-0.01,p[2]-0.1],
        [0.25,0.01,0.1],options(),1e-9).unwrap_err(),Error::OutsideDomain);
}

#[test]
fn invalid_trim_controls_are_rejected_before_a_section_is_evaluated() {
    let e = solved(SOURCE);
    let tooth = tooth(&e);
    for tolerance in [-1.,f64::NAN,f64::INFINITY] {
        assert_eq!(tooth.intersect(|_,_| panic!("invalid controls must precede callbacks"),
            [0.25,0.01,0.1],options(),tolerance).unwrap_err(),Error::InvalidOptions);
    }
}

#[test]
fn a_patch_is_a_snapshot_of_its_parts() {
    let mut e = solved(SOURCE);
    let original = tooth(&e);
    let copied = original.clone();
    let before = original.envelope_at([0.25,0.,0.1],1e-9,1e-9).unwrap().position;
    assert_eq!(copied.envelope_at([0.25,0.,0.1],1e-9,1e-9).unwrap().position,before);
    // the meridian moves out after the patch is made: the patch stays where it was made
    let c = e.map.ent_named("c").unwrap();
    let x = e.sketch.points[c.i()].x as usize;
    e.sketch.params[x].value = 3.2;
    let moved = tooth(&e).envelope_at([0.25,0.,0.1],1e-9,1e-9).unwrap().position;
    assert!(gcs_core::space::distance(moved,before) > 0.1,"{moved:?} {before:?}");
    assert_eq!(original.envelope_at([0.25,0.,0.1],1e-9,1e-9).unwrap().position,before);
}

#[test]
fn the_post_solve_vocabulary_is_not_solvent() {
    // `patch`, `seam`, `vertex`, `edge(…, along:)` and `face(…, on:)` were certification
    // objects read after the solve: they are Rust API now (#364), and no word of the language.
    let corners = "\
first := envelope(wall over t in (-20deg, 20deg), under: roll)
s := seam(generated,first)
v := vertex(s,s)
";
    for tail in [
        "p := patch(wall,inside: ring)",
        "p := patch(generated,inside: outer.body,outside: inner.body)",
        "s := seam(generated,wall)",
        corners,
        "k := edge(s,from: v,to: v,along: ax)",
        "f := face(meridian,on: generated)",
    ] {
        let src = format!("{SOURCE}{tail}\n");
        let (p,errors) = crate::common::parse(&src);
        assert!(!errors.is_empty() || !program::elaborate(&p).ok(),"accepted {tail}");
    }
}
