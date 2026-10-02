use super::*;
use gcs_core::{envelope::IntersectionOptions,seam::{BoundarySeam,BoundarySeamTolerance}};

pub(super) const SPHERE: &str = "
component Sphere(origin: point, size: Length) {
  private bottom := point hint(x: 0,y: -size)
  private top := point hint(x: 0,y: size)
  ground bottom
  ground top
  private rim := arc(center: origin,start: bottom,end: top)
  radius(size) rim
  private diameter := line(top,bottom)
  private carrier := solid(face(rim,diameter),about: diameter)
  wall := surface(carrier,rim)
}
cut := Sphere(o,size: sqrt(9.25) * 1mm)
boundary_edge := seam(first_envelope,cut.wall)
";

fn model() -> program::Elaborated { solved(&format!("{MODEL}{SPHERE}")) }
fn read(e: &program::Elaborated) -> BoundarySeam {
    BoundarySeam::named(&e.sketch,e.map.ent_named("boundary_edge").unwrap().i(),1e-10).unwrap()
}
fn tolerance() -> BoundarySeamTolerance {
    BoundarySeamTolerance {normal_velocity:1e-10,incidence:1e-10,trim:1e-10}
}
fn options(seam: &BoundarySeam) -> IntersectionOptions {
    IntersectionOptions {bounds:seam.domain(),parameter_scale:[1.;3],
        residual_tolerance:[1e-10;3],max_iterations:100}
}
fn section(c: &gcs_core::envelope::Contact) -> f64 { c.position[1]+0.5*0.1f64.sin() }

#[test]
fn a_boundary_seam_solves_the_envelope_sphere_and_section_independently() {
    let e = model();
    let seam = read(&e);
    let p = seam.intersect(section,[0.4,0.01,0.],options(&seam),1e-10).unwrap();
    for (a,b) in p.parameters.into_iter().zip([0.5,0.,0.1]) {
        assert!((a-b).abs() < 1e-8,"{p:?}");
    }
    for (a,b) in p.contact.position.into_iter().zip([3.,-0.5*0.1f64.sin(),0.5*0.1f64.cos()]) {
        assert!((a-b).abs() < 1e-9,"{p:?}");
    }
    assert!(seam.evaluate([0.5,0.,0.1],tolerance()).is_ok());
    assert_eq!(seam.evaluate([0.6,0.,0.1],tolerance()).unwrap_err(),Error::OutsideDomain);
    assert_eq!(seam.evaluate([0.5,0.1,0.1],tolerance()).unwrap_err(),Error::OutsideDomain);
    let e = e.map.ent_named("boundary_edge").unwrap();
    assert_eq!(e.kind,EntKind::Seam);
}

#[test]
fn boundary_seams_refuse_support_continuations_and_excluded_material() {
    let mut e = model();
    let original = read(&e);
    // The support sphere is unchanged, but the actual finite surface is far from
    // this intersection. A root of the support equation is not a root of the seam.
    let wall = e.map.ent_named("cut.wall").unwrap().i();
    e.sketch.surfaces[wall].span = Some([90f64.to_radians(),180f64.to_radians()]);
    let limited = read(&e);
    assert_eq!(limited.intersect(section,[0.4,0.01,0.],options(&limited),1e-10)
        .unwrap_err(),Error::OutsideDomain);
    assert!(original.evaluate([0.5,0.,0.1],tolerance()).is_ok());
    for radius in [3.01,3.1] {
        let src = format!("{MODEL}{}\nsmall := Sphere(o,size: {radius}mm)\n\
            clipped := patch(first_envelope,inside: small.wall.solid)\n",
            SPHERE.replace("boundary_edge := seam(first_envelope,cut.wall)","boundary_edge := seam(clipped,cut.wall)"));
        let e = solved(&src);
        let seam = read(&e);
        let result = seam.intersect(section,[0.4,0.01,0.],options(&seam),1e-10);
        assert_eq!(result.is_ok(),radius > 3.05,"{result:?}");
    }
}

#[test]
fn boundary_seams_roundtrip_copy_delete_and_preserve_private_dependencies() {
    let e = model();
    // The flat printer deliberately refuses component definitions. Exercise its
    // seam spelling with a flat sphere, and component privacy separately below.
    let mut flat = solved(&format!("{MODEL}\n\
        south := point hint(x: 0,y: -sqrt(9.25))\n\
        north := point hint(x: 0,y: sqrt(9.25))\n\
        ground south\nground north\n\
        rim := arc(center: o,start: south,end: north)\n\
        radius(sqrt(9.25) * 1mm) rim\ndiameter := line(north,south)\n\
        ball := solid(face(rim,diameter),about: diameter)\n\
        wall := surface(ball,rim)\nboundary_edge := seam(first_envelope,wall)\n")).program;
    let text = syntax::render_flat(&mut flat).unwrap().to_string();
    assert!(read(&solved(&text)).evaluate([0.5,0.,0.1],tolerance()).is_ok());
    let edge = e.map.ent_named("boundary_edge").unwrap();
    let clip = io::copy(&e.sketch,&[edge]);
    assert_eq!(clip.seams.len(),1);
    assert_eq!(clip.envelopes.len(),1);
    assert_eq!(clip.surfaces.len(),2);
    assert!(BoundarySeam::named(&clip,0,1e-10).unwrap().evaluate([0.5,0.,0.1],tolerance()).is_ok());
    let mut pasted = e.sketch.clone();
    io::paste(&mut pasted,&clip,0.,0.);
    assert!(BoundarySeam::named(&pasted,pasted.seams.len()-1,1e-10).unwrap()
        .evaluate([0.5,0.,0.1],tolerance()).is_ok());
    let removed = io::without(&e.sketch,&[e.map.ent_named("cut.wall").unwrap()],&[]);
    assert_eq!(removed.seams.len(),1); // The unrelated generating junction survives.
    let src = format!("component Edge(e: envelope,b: surface) {{\n\
        private hidden := seam(e,b)\n}}\npart := Edge(first_envelope,cut.wall)\n{MODEL}{SPHERE}");
    assert!(solved(&src).map.ent_named("part.hidden").is_some());
    let bad = build(&format!("{src}bad := seam(part.hidden.first,cut.wall)\n"));
    assert!(bad.errors().any(|d| d.message.contains("private member")),"{:?}",bad.diags);
}

#[test]
fn boundary_seams_refuse_invalid_controls_and_nonisolated_sections() {
    let e = model();
    let seam = read(&e);
    for t in [-1.,f64::NAN,f64::INFINITY] {
        assert_eq!(seam.intersect(|_| panic!("invalid controls must precede callbacks"),
            [0.4,0.01,0.],options(&seam),t).unwrap_err(),Error::InvalidOptions);
        assert_eq!(seam.evaluate([0.5,0.,0.1],BoundarySeamTolerance {incidence:t,..tolerance()})
            .unwrap_err(),Error::InvalidOptions);
    }
    assert!(BoundarySeam::named(&e.sketch,99,1e-10).is_err());
    assert!(BoundarySeam::named(&e.sketch,0,1e-10).is_err()); // Generating seam is a different kind.
    // A duplicate sphere section leaves roll free, even at an exact initial root.
    assert_eq!(seam.intersect(|c| c.position[0].hypot(c.position[1]).hypot(c.position[2])-9.25f64.sqrt(),
        [0.5,0.,0.1],options(&seam),1e-10).unwrap_err(),Error::SingularIntersection);
    for tail in ["bad := seam(cut.wall,first_envelope)","bad := seam(cut.wall,cut.wall)"] {
        assert!(!build(&format!("{MODEL}{SPHERE}{tail}\n")).ok());
    }
}
