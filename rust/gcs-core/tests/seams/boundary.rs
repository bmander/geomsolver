use super::*;
use gcs_core::{envelope::IntersectionOptions,seam::{BoundarySeam,BoundarySeamTolerance}};

pub(super) const SPHERE: &str = "\
component Sphere(origin: point, size: Length) {
  private bottom := point
  private top := point
  fix((0, -size)) bottom
  fix((0, size)) top
  private rim := arc(center: origin,start: bottom,end: top)
  radius(size) rim
  private diameter := line(top,bottom)
  private carrier := solid(face(rim,diameter),about: diameter)
  wall := surface(carrier,rim)
}
in std.front {
cut := Sphere(o,size: sqrt(9.25) * 1mm)
}
";

fn model() -> program::Elaborated { solved(&format!("{MODEL}{SPHERE}")) }
/// The first envelope, kept within each of `inside`, meeting the sphere `cut.wall`.
fn read(e: &program::Elaborated,inside: &[&str]) -> BoundarySeam {
    BoundarySeam::new("boundary_edge".into(),&face(e,"first_envelope",inside),projector(e,"cut.wall")).unwrap()
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
    let seam = read(&e,&[]);
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
}

#[test]
fn boundary_seams_refuse_support_continuations_and_excluded_material() {
    let mut e = model();
    let original = read(&e,&[]);
    // The support sphere is unchanged, but the actual finite surface is far from
    // this intersection. A root of the support equation is not a root of the seam.
    let wall = index(&e,"cut.wall");
    e.sketch.surfaces[wall].span = Some([90f64.to_radians(),180f64.to_radians()]);
    let limited = read(&e,&[]);
    assert_eq!(limited.intersect(section,[0.4,0.01,0.],options(&limited),1e-10)
        .unwrap_err(),Error::OutsideDomain);
    // A snapshot keeps the boundary it was made with.
    assert!(original.evaluate([0.5,0.,0.1],tolerance()).is_ok());
    for radius in [3.01,3.1] {
        let e = solved(&format!("{MODEL}{SPHERE}\nsmall := Sphere(o,size: {radius}mm) in std.front\n"));
        let seam = read(&e,&["small.wall"]);
        let result = seam.intersect(section,[0.4,0.01,0.],options(&seam),1e-10);
        assert_eq!(result.is_ok(),radius > 3.05,"{result:?}");
    }
}

#[test]
fn boundary_seams_refuse_invalid_controls_surface_faces_and_nonisolated_sections() {
    let e = model();
    let seam = read(&e,&[]);
    for t in [-1.,f64::NAN,f64::INFINITY] {
        assert_eq!(seam.intersect(|_| panic!("invalid controls must precede callbacks"),
            [0.4,0.01,0.],options(&seam),t).unwrap_err(),Error::InvalidOptions);
        assert_eq!(seam.evaluate([0.5,0.,0.1],BoundarySeamTolerance {incidence:t,..tolerance()})
            .unwrap_err(),Error::InvalidOptions);
    }
    // A boundary seam's face is generated: a surface meeting a surface is a `SurfaceSeam`.
    let wall = TrimmedPatch::of_surface("wall".into(),surface(&e,"first_surface"),vec![],vec![]);
    assert!(BoundarySeam::new("bad".into(),&wall,projector(&e,"cut.wall")).is_err());
    // A duplicate sphere section leaves roll free, even at an exact initial root.
    assert_eq!(seam.intersect(|c| c.position[0].hypot(c.position[1]).hypot(c.position[2])-9.25f64.sqrt(),
        [0.5,0.,0.1],options(&seam),1e-10).unwrap_err(),Error::SingularIntersection);
}
