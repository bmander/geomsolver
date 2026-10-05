use gcs_core::{diagnose,envelope::{Error,IntersectionOptions},io,model::{EntKind,EntRef},
    patch::TrimmedPatch,program,solve,syntax};

const SOURCE: &str = "\
unit mm
use std
in std.front {
o := point
q := point
x := point
c := point
fix(x == 0, y == 0) o
fix(x == 0, y == 1) q
fix(x == 1, y == 0) x
fix(x == 3, y == 0) c
ax := line(o,q)
spin_axis := line(o,x)
meridian := circle(center: c)
radius(1mm) meridian
}
ring := solid(face(meridian),about: ax)
wall := surface(ring,meridian)
roll := motion(about: spin_axis)
generated := envelope(wall,under: roll,from: -20deg,to: 20deg)
component Sphere(o: point, size: Length) {
  private bottom := point
  private top := point
  fix(x == 0, y == -size) bottom
  fix(x == 0, y == size) top
  private rim := arc(center: o,start: bottom,end: top)
  radius(size) rim
  private ax := line(top,bottom)
  body := solid(face(rim,ax),about: ax)
}
in std.front {
inner := Sphere(o, size: 2.5mm)
outer := Sphere(o, size: 3.5mm)
}
bounded := patch(wall,inside: outer.body,outside: inner.body)
tooth := patch(generated,inside: outer.body,outside: inner.body)
";

fn build(src: &str) -> program::Elaborated {
    let (p,errors) = crate::common::parse(src);
    assert!(errors.is_empty(),"{errors:?}");
    program::elaborate(&p)
}

fn solved(src: &str) -> program::Elaborated {
    let mut e = build(src);
    assert!(e.ok(),"{:?}",e.diags);
    assert!(solve::solve(&mut e.sketch,Default::default()).success);
    assert_eq!(diagnose::diagnose(&mut e.sketch,Default::default()).dof,0);
    e
}

#[test]
fn patches_select_material_sides_and_enforce_the_source_locus() {
    let e = solved(SOURCE);
    let face = TrimmedPatch::named(&e.sketch,0,1e-10).unwrap();
    let tooth = TrimmedPatch::named(&e.sketch,1,1e-10).unwrap();
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
    assert!(TrimmedPatch::named(&e.sketch,99,1e-10).is_err());
    assert!(e.sketch.entity_params(e.map.ent_named("bounded").unwrap()).is_empty());
}

#[test]
fn intersection_trials_may_cross_trims_but_a_converged_excluded_point_is_refused() {
    let e = solved(SOURCE);
    let tooth = TrimmedPatch::named(&e.sketch,1,1e-10).unwrap();
    let options = IntersectionOptions {bounds:[[0.,0.4],[0.,0.2],[-0.2,0.2]],
        parameter_scale:[1.;3],residual_tolerance:[1e-10;3],max_iterations:100};
    let result = tooth.intersect(|p,_| [p[0]-0.25,p[2]-0.1],
        [0.01,0.01,0.1],options,1e-9).unwrap();
    assert!((result.parameters[0]-0.25).abs() < 1e-9);
    assert_eq!(tooth.intersect(|p,_| [p[0]-0.01,p[2]-0.1],
        [0.25,0.01,0.1],options,1e-9).unwrap_err(),Error::OutsideDomain);
}

#[test]
fn invalid_trim_controls_are_rejected_before_a_section_is_evaluated() {
    let e = solved(SOURCE);
    let tooth = TrimmedPatch::named(&e.sketch,1,1e-10).unwrap();
    let options = IntersectionOptions {bounds:[[0.,0.4],[0.,0.2],[-0.2,0.2]],
        parameter_scale:[1.;3],residual_tolerance:[1e-10;3],max_iterations:100};
    for tolerance in [-1.,f64::NAN,f64::INFINITY] {
        assert_eq!(tooth.intersect(|_,_| panic!("invalid controls must precede callbacks"),
            [0.25,0.01,0.1],options,tolerance).unwrap_err(),Error::InvalidOptions);
    }
}

#[test]
fn patch_source_references_roundtrip_and_copy_with_all_clipping_dependencies() {
    let e = solved(SOURCE);
    let flat = format!("{}bounded := patch(wall, inside: ring, inside: ring, outside: ring)\n",
        SOURCE.split("component Sphere").next().unwrap());
    let mut p = solved(&flat).program;
    let text = syntax::render_flat(&mut p).unwrap().to_string();
    let again = solved(&text);
    assert_eq!(again.sketch.patches.len(),1);
    assert_eq!(again.sketch.patches[0].inside.len(),2);
    assert_eq!(again.sketch.patches[0].outside.len(),1);
    let tooth = e.map.ent_named("tooth").unwrap();
    let clip = io::copy(&e.sketch,&[tooth]);
    assert_eq!(clip.patches.len(),1);
    assert_eq!(clip.solids.len(),3);
    assert_eq!(clip.envelopes.len(),1);
    let mut dst = e.sketch.clone();
    io::paste(&mut dst,&clip,0.,0.);
    assert_eq!(dst.patches.len(),3);
    let a = TrimmedPatch::named(&clip,0,1e-10).unwrap()
        .envelope_at([0.25,0.,0.1],1e-9,1e-9).unwrap();
    let b = TrimmedPatch::named(&dst,2,1e-10).unwrap()
        .envelope_at([0.25,0.,0.1],1e-9,1e-9).unwrap();
    assert_eq!(a.position,b.position);
    assert_eq!(io::without(&e.sketch,&[tooth],&[]).patches.len(),1);
    assert!(io::without(&e.sketch,&[e.map.ent_named("outer.body").unwrap()],&[]).patches.is_empty());
    assert!(io::copy(&e.sketch,&[EntRef::point(0)]).patches.is_empty());
    assert_eq!(tooth.kind,EntKind::Patch);
}

#[test]
fn patch_formals_follow_forward_source_paths_and_private_regions_stay_private() {
    let forward = format!("component Copy(p: patch,b: solid) {{ out := patch(p.source,inside: b) }}\n\
        copy := Copy(tooth,outer.body)\n{SOURCE}");
    let e = solved(&forward);
    let copy = e.map.ent_named("copy.out").unwrap();
    assert_eq!(e.sketch.patches[copy.i()].source,e.map.ent_named("generated").unwrap());
    let src = format!("{SOURCE}\ncomponent Part(s: surface,b: solid) {{\n\
        private construction region := patch(s,inside: b)\n}}\npart := Part(wall,outer.body)\n");
    let e = solved(&src);
    assert!(e.map.entity_path(&e.sketch,"part.region").is_none());
    let e = build(&format!("{src}leak := patch(part.region.source,inside: outer.body)\n"));
    assert!(e.errors().any(|d| d.message.contains("private member")),"{:?}",e.diags);
}

#[test]
fn patch_diagnostics_refuse_missing_or_wrong_operands_and_ambiguous_labels() {
    for (tail,want) in [
        ("bad := patch(wall)","at least one"),
        ("bad := patch(wall,inside: ax)","trim names a solid"),
        ("bad := patch(ax,inside: ring)","source must be"),
        ("bad := patch(wall,inside: missing)","no such entity"),
        ("bad := patch(source: wall,source: wall,inside: ring)","exactly one"),
        ("fix(x == 0, y == 0) tooth","a patch has no number of its own to fix"),
    ] {
        let e = build(&format!("{SOURCE}{tail}\n"));
        assert!(e.errors().any(|d| d.message.contains(want)),"{:?}",e.diags);
    }
    for tail in ["bad := patch(wall,ring)","bad := patch(wall,insdie: ring)"] {
        let (_,errors) = crate::common::parse(&format!("{SOURCE}{tail}\n"));
        assert!(!errors.is_empty());
    }
}
