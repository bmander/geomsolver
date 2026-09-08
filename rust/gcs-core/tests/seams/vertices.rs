use super::*;
use gcs_core::{envelope::IntersectionOptions,seam::BoundarySeamTolerance,
    vertex::{self,BoundaryVertex,JunctionVertex,VertexKind}};

mod edges;
mod faces;

const CORNERS: &str = "
component Sphere(origin: point,size: Length) {
  private point south hint(x: origin.x,y: origin.y-size)
  private point north hint(x: origin.x,y: origin.y+size)
  ground south
  ground north
  private arc rim(center: origin,start: south,end: north)
  radius(size) rim
  private line diameter(north,south)
  private solid carrier(face(rim,diameter),about: diameter)
  surface wall(carrier,rim)
}
point shifted hint(x: 0,y: 1)
ground shifted
sphere: Sphere(o,size: sqrt(9.25)*1mm)
offset: Sphere(shifted,size: sqrt(10.25-cos(0.1rad))*1mm)
join_cut: Sphere(shifted,size: sqrt(11-2*cos(0.1rad))*1mm)
seam radial(first_envelope,sphere.wall)
seam offset_edge(first_envelope,offset.wall)
seam join_edge(second_envelope,join_cut.wall)
vertex corner(first: radial,second: offset_edge)
vertex junction(shared,join_edge)
";

fn model() -> program::Elaborated { solved(&format!("{MODEL}{CORNERS}")) }
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
fn named_vertices_solve_both_corner_kinds_without_planar_coordinates() {
    let e = model();
    let corner = BoundaryVertex::named(&e.sketch,0,1e-10).unwrap();
    let junction = JunctionVertex::named(&e.sketch,1,tolerance()).unwrap();
    assert_eq!(vertex::kind(&e.sketch,0).unwrap(),VertexKind::Boundaries);
    assert_eq!(vertex::kind(&e.sketch,1).unwrap(),VertexKind::Junction);
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
    for name in ["corner","junction"] {
        let entity = e.map.ent_named(name).unwrap();
        assert_eq!(entity.kind,EntKind::Vertex);
        assert!(e.sketch.entity_params(entity).is_empty());
        assert!(e.sketch.children(entity).iter().all(|e| e.kind == EntKind::Seam));
    }
    assert!(BoundaryVertex::named(&e.sketch,1,1e-10).is_err());
    assert!(JunctionVertex::named(&e.sketch,0,tolerance()).is_err());
    assert!(vertex::kind(&e.sketch,99).is_err());
    assert_eq!(corner.position([0.6,0.,0.1],checked()).unwrap_err(),Error::OutsideDomain);
    assert_eq!(junction.position([0.,0.,0.1],checked()).unwrap_err(),Error::OutsideDomain);
    assert_eq!(junction.position([1.,0.,0.2],checked()).unwrap_err(),Error::OutsideDomain);
}

#[test]
fn vertex_identity_formals_privacy_and_dependency_copy_survive() {
    let src = format!("component Forward(v: vertex) {{ construction vertex out(v.second,v.first) }}\n\
        copy: Forward(corner)\ncopy_join: Forward(junction)\n{MODEL}{CORNERS}");
    let e = solved(&src);
    let out = e.map.ent_named("copy.out").unwrap();
    let original = e.map.ent_named("corner").unwrap();
    assert_ne!(out,original); // Equal positions do not merge declaration identity.
    let v = BoundaryVertex::named(&e.sketch,out.i(),1e-10).unwrap();
    near(v.position([0.5,0.,0.1],checked()).unwrap(),[3.,-0.5*0.1f64.sin(),0.5*0.1f64.cos()]);
    let join = e.map.ent_named("copy_join.out").unwrap();
    assert!(JunctionVertex::named(&e.sketch,join.i(),tolerance()).unwrap()
        .position([1.,0.,0.1],checked()).is_ok());
    let clip = io::copy(&e.sketch,&[out]);
    assert_eq!(clip.vertices.len(),1);
    assert_eq!(clip.seams.len(),2);
    assert_eq!(clip.envelopes.len(),1);
    assert!(clip.roles_of(gcs_core::model::EntRef::new(EntKind::Vertex,0)).construction);
    assert!(BoundaryVertex::named(&clip,0,1e-10).unwrap().position([0.5,0.,0.1],checked()).is_ok());
    let mut pasted = e.sketch.clone();
    io::paste(&mut pasted,&clip,0.,0.);
    assert!(BoundaryVertex::named(&pasted,pasted.vertices.len()-1,1e-10).unwrap()
        .position([0.5,0.,0.1],checked()).is_ok());
    let deleted = io::without(&e.sketch,&[e.map.ent_named("offset.wall").unwrap()],&[]);
    assert_eq!(deleted.vertices.len(),2); // Both junction declarations survive.
    let src = format!("{MODEL}{CORNERS}\ncomponent Private(a: seam,b: seam) {{\n\
        private vertex hidden(a,b)\n}}\npart: Private(radial,offset_edge)\n");
    let e = solved(&src);
    assert!(e.map.entity_path(&e.sketch,"part.hidden").is_none());
    let e = build(&format!("{src}vertex leak(part.hidden.first,offset_edge)\n"));
    assert!(e.errors().any(|d| d.message.contains("private member")),"{:?}",e.diags);
}

#[test]
fn vertex_spelling_roundtrips_in_a_flat_program() {
    let source = format!("{MODEL}\npoint south hint(x: 0,y: -sqrt(10))\n\
        point north hint(x: 0,y: sqrt(10))\nground south\nground north\n\
        arc rim(center: o,start: south,end: north)\nradius(sqrt(10)*1mm) rim\n\
        line diameter(north,south)\nsolid ball(face(rim,diameter),about: diameter)\n\
        surface wall(ball,rim)\nseam boundary(first_envelope,wall)\n\
        construction vertex corner(shared,boundary)\nvertex other(shared,boundary)\n\
        edge extent(shared,from: corner,to: other,along: axis)\n");
    let mut p = solved(&source).program;
    let text = syntax::render_flat(&mut p).unwrap();
    let e = solved(text);
    assert_eq!(e.sketch.vertices.len(),2);
    assert!(JunctionVertex::named(&e.sketch,0,tolerance()).unwrap()
        .position([1.,0.,0.1],checked()).is_ok());
    let edge = gcs_core::edge::SpatialEdge::named(&e.sketch,0,[[1.,0.,0.1],[1.,0.,0.2]],
        gcs_core::edge::EdgeTolerance {junction:tolerance(),point:checked()}).unwrap();
    assert!(edge.sample(0.5,100).is_ok());
}

#[test]
fn vertices_reject_unrelated_faces_duplicate_boundaries_and_wrong_operands() {
    for tail in [
        "vertex bad(radial,radial)",
        "seam duplicate(first_envelope,sphere.wall)\nvertex bad(radial,duplicate)",
        "seam different(second_envelope,offset.wall)\nvertex bad(radial,different)",
        "vertex bad(shared,shared)",
        "seam other_join(second_envelope,first_envelope)\nvertex bad(shared,other_join)",
        "vertex bad(first_envelope,radial)",
        "vertex bad(radial)",
        "vertex bad(radial,offset_edge,join_edge)",
        "ground corner",
    ] {
        let (p,errors) = syntax::parse(&format!("{MODEL}{CORNERS}{tail}\n"));
        assert!(!errors.is_empty() || !program::elaborate(&p).ok(),"accepted {tail}");
    }
    let src = format!("{MODEL}{CORNERS}\nenvelope lookalike(first_surface,under: roll,from: -20deg,to: 20deg)\n\
        seam unrelated(lookalike,join_cut.wall)\nvertex bad(shared,unrelated)\n");
    assert!(build(&src).errors().any(|d| d.message.contains("face of the generating junction")));
}

#[test]
fn corner_snapshots_enforce_finite_boundaries_material_and_numerical_controls() {
    let mut e = model();
    let original = BoundaryVertex::named(&e.sketch,0,1e-10).unwrap();
    let wall = e.map.ent_named("offset.wall").unwrap().i();
    e.sketch.surfaces[wall].span = Some([90f64.to_radians(),180f64.to_radians()]);
    let limited = BoundaryVertex::named(&e.sketch,0,1e-10).unwrap();
    assert_eq!(limited.solve([0.45,0.01,0.08],bounds(&limited),1e-10).unwrap_err(),Error::OutsideDomain);
    assert!(original.position([0.5,0.,0.1],checked()).is_ok());
    let wall = e.map.ent_named("join_cut.wall").unwrap().i();
    e.sketch.surfaces[wall].span = Some([90f64.to_radians(),180f64.to_radians()]);
    let junction = JunctionVertex::named(&e.sketch,1,tolerance()).unwrap();
    assert_eq!(junction.solve([0.01,0.08],junction_options(&junction),1e-10).unwrap_err(),Error::OutsideDomain);
    for t in [-1.,f64::NAN,f64::INFINITY] {
        assert!(BoundaryVertex::named(&e.sketch,0,t).is_err());
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
        let src = format!("{MODEL}{}\nclip: Sphere(o,size: {radius}mm)\n\
            patch clipped(first_envelope,inside: clip.wall.solid)\n",
            CORNERS.replace("seam radial(first_envelope,","seam radial(clipped,")
                .replace("seam offset_edge(first_envelope,","seam offset_edge(clipped,"));
        let e = solved(&src);
        let v = BoundaryVertex::named(&e.sketch,0,1e-10).unwrap();
        assert_eq!(v.solve([0.45,0.01,0.08],bounds(&v),1e-10).is_ok(),radius > 3.05);
    }
}

#[test]
fn a_corner_does_not_transfer_fine_incidence_across_a_coarse_seam_tolerance() {
    let src = MODEL.replace("point a hint(x: 2,y: 0)","point a hint(x: 1,y: 0)")
        .replace("point c hint(x: 3,y: 2)","point c hint(x: 2,y: 2)")
        .replace("point d hint(x: 2,y: 2)","point d hint(x: 1,y: 2)")
        .replace("line high(m,c)","point arc_center hint(x: 2,y: 1)\nground arc_center\n\
            arc high(center: arc_center,start: m,end: c)\nradius(1mm) high");
    let mut e = solved(&format!("{src}{CORNERS}"));
    let radius = e.sketch.arcs[e.map.ent_named("high").unwrap().i()].radius as usize;
    e.sketch.params[radius].value += 1e-5;
    let v = JunctionVertex::named(&e.sketch,1,SeamTolerance {position:1e-4,..tolerance()}).unwrap();
    // The canonical first face still hits the boundary exactly. The second face,
    // which actually owns join_edge, misses it despite passing seam coincidence.
    assert_eq!(v.position([1.,0.,0.1],checked()).unwrap_err(),Error::OutsideDomain);
}

#[test]
fn distinct_names_do_not_make_a_nonisolated_corner_unique() {
    let src = format!("{MODEL}{}",CORNERS.replace(
        "offset: Sphere(shifted,size: sqrt(10.25-cos(0.1rad))*1mm)",
        "offset: Sphere(o,size: sqrt(9.25)*1mm)"));
    let e = solved(&src);
    let v = BoundaryVertex::named(&e.sketch,0,1e-10).unwrap();
    assert_eq!(v.solve([0.5,0.,0.1],bounds(&v),1e-10).unwrap_err(),Error::SingularIntersection);
    let src = format!("{MODEL}{}",CORNERS.replace(
        "join_cut: Sphere(shifted,size: sqrt(11-2*cos(0.1rad))*1mm)",
        "join_cut: Sphere(o,size: sqrt(10)*1mm)"));
    let e = solved(&src);
    let v = JunctionVertex::named(&e.sketch,1,tolerance()).unwrap();
    assert_eq!(v.solve([0.,0.1],junction_options(&v),1e-10).unwrap_err(),Error::SingularIntersection);
}
