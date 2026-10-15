use super::*;
use gcs_core::{envelope::IntersectionOptions,seam::BoundarySeamTolerance,
    vertex::{BoundaryVertex,JunctionVertex}};

mod edges;
mod faces;

const CORNERS: &str = "\
component Sphere(origin: point,size: Length) {
  private south := point hint((origin.x, origin.y-size))
  private north := point hint((origin.x, origin.y+size))
  south vertical origin
  north vertical origin
  private rim := arc(center: origin,start: south,end: north)
  radius(size) rim
  private diameter := line(north,south)
  private carrier := solid(face(rim,diameter),about: diameter)
  wall := surface(carrier,rim)
}
in std.front {
shifted := point
fix((0, 1)) shifted
globe := Sphere(o,size: sqrt(9.25)*1mm)
offset := Sphere(shifted,size: sqrt(10.25-cos(0.1rad))*1mm)
join_cut := Sphere(shifted,size: sqrt(11-2*cos(0.1rad))*1mm)
}
";

fn model() -> program::Elaborated { solved(&format!("{MODEL}{CORNERS}")) }
/// The first envelope, kept within each of `inside`, meeting two spheres.
fn corner(e: &program::Elaborated,inside: &[&str],walls: [&str;2]) -> BoundaryVertex {
    BoundaryVertex::new("corner".into(),&face(e,"first_envelope",inside),walls.map(|w| projector(e,w))).unwrap()
}
/// The shared characteristic meeting the sphere `wall` on its second face.
fn junction_with(e: &program::Elaborated,wall: &str,t: SeamTolerance) -> JunctionVertex {
    JunctionVertex::new("junction".into(),shared_with(e,t).unwrap(),projector(e,wall),1).unwrap()
}
fn junction(e: &program::Elaborated) -> JunctionVertex { junction_with(e,"join_cut.wall",tolerance()) }
fn checked() -> BoundarySeamTolerance {
    BoundarySeamTolerance {normal_velocity:1e-10,incidence:1e-10,trim:1e-10}
}
fn bounds(v: &BoundaryVertex) -> IntersectionOptions {
    // The two spheres also meet at negative roll. Select the positive local root.
    let mut bounds = v.domain(); bounds[2][0] = 0.02;
    IntersectionOptions {bounds,parameter_scale:[1.;3],residual_tolerance:[1e-10;3],max_iterations:100}
}
fn junction_options(v: &JunctionVertex) -> SeamIntersectionOptions {
    let [_,a,mut b] = v.domain(); b[0] = 0.02;
    SeamIntersectionOptions {bounds:[a,b],parameter_scale:[1.;2],normal_tolerance:1e-10,
        section_tolerance:1e-10,max_iterations:100}
}
fn near(a: [f64;3],b: [f64;3]) {
    for i in 0..3 { assert!((a[i]-b[i]).abs() < 1e-8,"{a:?} != {b:?}"); }
}

#[test]
fn corners_of_both_kinds_solve_without_planar_coordinates() {
    let e = model();
    let corner = corner(&e,&[],["globe.wall","offset.wall"]);
    let junction = junction(&e);
    assert_eq!(corner.domain()[0],[0.,1.]);
    assert_eq!(junction.domain()[0],[1.,1.]);
    let p = corner.solve([0.45,0.01,0.08],bounds(&corner),1e-10).unwrap();
    let q = junction.solve([0.01,0.08],junction_options(&junction),1e-10).unwrap();
    for (p,u) in [(p,0.5),(q,1.)] {
        near(p.parameters,[u,0.,0.1]);
        near(p.position,[3.,-u*0.1f64.sin(),u*0.1f64.cos()]);
    }
    near(corner.position(p.parameters,checked()).unwrap(),p.position);
    near(junction.position(q.parameters,checked()).unwrap(),q.position);
    assert_eq!(corner.position([0.6,0.,0.1],checked()).unwrap_err(),Error::OutsideDomain);
    assert_eq!(junction.position([0.,0.,0.1],checked()).unwrap_err(),Error::OutsideDomain);
    assert_eq!(junction.position([1.,0.,0.2],checked()).unwrap_err(),Error::OutsideDomain);
}

#[test]
fn corners_refuse_surface_faces_and_a_third_junction_face() {
    let e = model();
    let wall = TrimmedPatch::of_surface("wall".into(),surface(&e,"first_surface"),vec![],vec![]);
    assert!(BoundaryVertex::new("bad".into(),&wall,
        [projector(&e,"globe.wall"),projector(&e,"offset.wall")]).is_err());
    assert!(JunctionVertex::new("bad".into(),shared(&e),projector(&e,"join_cut.wall"),2).is_err());
}

#[test]
fn corner_snapshots_enforce_finite_boundaries_material_and_numerical_controls() {
    let mut e = model();
    let original = corner(&e,&[],["globe.wall","offset.wall"]);
    let joined = junction(&e);
    let wall = index(&e,"offset.wall");
    e.sketch.surfaces[wall].span = Some([90f64.to_radians(),180f64.to_radians()]);
    let limited = corner(&e,&[],["globe.wall","offset.wall"]);
    assert_eq!(limited.solve([0.45,0.01,0.08],bounds(&limited),1e-10).unwrap_err(),Error::OutsideDomain);
    assert!(original.position([0.5,0.,0.1],checked()).is_ok());
    let wall = index(&e,"join_cut.wall");
    e.sketch.surfaces[wall].span = Some([90f64.to_radians(),180f64.to_radians()]);
    let junction = junction(&e);
    assert_eq!(junction.solve([0.01,0.08],junction_options(&junction),1e-10).unwrap_err(),Error::OutsideDomain);
    assert!(joined.position([1.,0.,0.1],checked()).is_ok());
    for t in [-1.,f64::NAN,f64::INFINITY] {
        assert_eq!(original.solve([0.5,0.,0.1],bounds(&original),t).unwrap_err(),Error::InvalidOptions);
        assert_eq!(junction.solve([0.,0.1],junction_options(&junction),t).unwrap_err(),Error::InvalidOptions);
        for field in 0..3 {
            let mut tol = checked();
            match field { 0 => tol.normal_velocity = t, 1 => tol.incidence = t, _ => tol.trim = t }
            assert_eq!(original.position([0.5,0.,0.1],tol).unwrap_err(),Error::InvalidOptions);
            assert_eq!(junction.position([1.,0.,0.1],tol).unwrap_err(),Error::InvalidOptions);
        }
    }
    for radius in [3.01,3.5] {
        let e = solved(&format!("{MODEL}{CORNERS}\nclip := Sphere(o,size: {radius}mm) in std.front\n"));
        let v = corner(&e,&["clip.wall"],["globe.wall","offset.wall"]);
        assert_eq!(v.solve([0.45,0.01,0.08],bounds(&v),1e-10).is_ok(),radius > 3.05);
    }
}

#[test]
fn a_corner_does_not_transfer_fine_incidence_across_a_coarse_seam_tolerance() {
    let src = MODEL.replace("fix((2, 0)) a","fix((1, 0)) a")
        .replace("fix((3, 2)) c","fix((2, 2)) c")
        .replace("fix((2, 2)) d","fix((1, 2)) d")
        .replace("high := line(m,c)","arc_center := point\nfix((2, 1)) arc_center\n\
            high := arc(center: arc_center,start: m,end: c)\nradius(1mm) high");
    let mut e = solved(&format!("{src}{CORNERS}"));
    let radius = e.sketch.arcs[e.map.ent_named("high").unwrap().i()].radius as usize;
    e.sketch.params[radius].value += 1e-5;
    let v = junction_with(&e,"join_cut.wall",SeamTolerance {position:1e-4,..tolerance()});
    // The canonical first face still hits the boundary exactly. The second face,
    // which actually meets the sphere, misses it despite passing seam coincidence.
    assert_eq!(v.position([1.,0.,0.1],checked()).unwrap_err(),Error::OutsideDomain);
}

#[test]
fn distinct_boundaries_do_not_make_a_nonisolated_corner_unique() {
    let src = format!("{MODEL}{}",CORNERS.replace(
        "offset := Sphere(shifted,size: sqrt(10.25-cos(0.1rad))*1mm)",
        "offset := Sphere(o,size: sqrt(9.25)*1mm)"));
    let e = solved(&src);
    let v = corner(&e,&[],["globe.wall","offset.wall"]);
    assert_eq!(v.solve([0.5,0.,0.1],bounds(&v),1e-10).unwrap_err(),Error::SingularIntersection);
    let src = format!("{MODEL}{}",CORNERS.replace(
        "join_cut := Sphere(shifted,size: sqrt(11-2*cos(0.1rad))*1mm)",
        "join_cut := Sphere(o,size: sqrt(10)*1mm)"));
    let e = solved(&src);
    let v = junction(&e);
    assert_eq!(v.solve([0.,0.1],junction_options(&v),1e-10).unwrap_err(),Error::SingularIntersection);
}
