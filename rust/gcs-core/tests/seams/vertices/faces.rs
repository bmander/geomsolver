use super::*;
use gcs_core::{edge::EdgeTolerance,model::{EntRef,FaceSupport},spatial_face::SpatialFaceBoundary,
    topology::Direction};

const SOURCE: &str = include_str!("../../fixtures/spatial_faces.sv");
const FACES: &str = "
construction face lower_face(left_low,middle_edge,right_low,bottom_edge,on: first_envelope)
face upper_face(left_high,top_edge,right_high,middle_edge,on: second_envelope)
";

fn source() -> String { SOURCE.to_string() }
fn without_faces() -> &'static str { SOURCE.split("// Spatial face declarations").next().unwrap() }
fn controls() -> EdgeTolerance { EdgeTolerance {junction:tolerance(),point:checked()} }
fn parameters(sk: &gcs_core::model::Sketch) -> Vec<[f64;3]> {
    sk.vertices.iter().map(|v| {
        let (h,t,second): (f64,f64,bool) = match v.name.as_str() {
            "bl" => (0.98,0.1,false), "br" => (0.98,0.2,false),
            "ml" => (1.,0.1,false), "mr" => (1.,0.2,false),
            "tl" => (1.02,0.1,true), "tr" => (1.02,0.2,true),
            _ => return [f64::NAN;3], // Unused witnesses are deliberately not read.
        };
        [if second { h-1. } else { h },0.,((h*h-1.+2.*t.cos())/(2.*h)).acos()]
    }).collect()
}
fn face(e: &program::Elaborated,name: &str) -> SpatialFaceBoundary {
    SpatialFaceBoundary::named(&e.sketch,e.map.ent_named(name).unwrap().i(),
        &parameters(&e.sketch),controls()).unwrap()
}

#[test]
fn spatial_faces_reuse_shared_edges_and_convert_their_support_charts() {
    let e = solved(&source());
    let lower = face(&e,"lower_face"); let upper = face(&e,"upper_face");
    let shared = e.map.ent_named("middle_edge").unwrap().i();
    assert_eq!(lower.edge_uses()[1].edge,shared);
    assert_eq!(upper.edge_uses()[3].edge,shared);
    assert_eq!(lower.edge_uses()[1].direction,Direction::Forward);
    assert_eq!(upper.edge_uses()[3].direction,Direction::Reverse);
    for f in [0.,0.25,0.5,0.75,1.] {
        let a = lower.sample_boundary(1,f,100).unwrap();
        let b = upper.sample_boundary(3,1.-f,100).unwrap();
        assert_eq!(a.position,b.position);
        assert_eq!(a.parameters[0],1.);
        assert_eq!(b.parameters[0],0.);
    }
    for face in [&lower,&upper] {
        for (edge,u) in face.edge_uses().iter().enumerate() {
            let name = &e.sketch.edges[u.edge].name;
            let a = face.sample_boundary(edge,0.,100).unwrap();
            let b = face.sample_boundary(edge,1.,100).unwrap();
            for f in [0.,0.25,0.5,0.75,1.] {
                let p = face.sample_boundary(edge,f,100).unwrap();
                let z = (1.-f)*a.position[2]+f*b.position[2];
                let (radius2,center) = match name.as_str() {
                    "bottom_edge" => (0.98f64.powi(2),0.),
                    "middle_edge" => (1.,0.),
                    "top_edge" => (1.02f64.powi(2),0.),
                    "left_low" | "left_high" => (2.-2.*0.1f64.cos(),1.),
                    "right_low" | "right_high" => (2.-2.*0.2f64.cos(),1.),
                    _ => unreachable!(),
                };
                near(p.position,[3.,-(radius2-(z-center).powi(2)).sqrt(),z]);
                assert!(p.incidence_error <= controls().point.incidence);
            }
        }
    }
    let reversed = solved(&source().replace("left_low,middle_edge,right_low,bottom_edge",
        "bottom_edge,right_low,middle_edge,left_low"));
    let reversed = face(&reversed,"lower_face");
    for i in 0..4 {
        near(lower.sample_boundary(i,0.25,100).unwrap().position,
            reversed.sample_boundary(3-i,0.75,100).unwrap().position);
    }
}

#[test]
fn spatial_face_dependencies_roundtrip_copy_delete_and_retain_owned_snapshots() {
    // Forward face declarations must not consume an index in the planar-profile phase.
    let mut e = solved(&format!("{FACES}{}",without_faces()));
    let target = e.map.ent_named("upper_face").unwrap();
    assert_eq!(e.sketch.faces[target.i()].on(),e.map.ent_named("second_envelope"));
    assert!(e.sketch.faces[target.i()].plane().is_err());
    assert!(e.sketch.entity_params(target).is_empty());
    let original = face(&e,"upper_face");
    let clip = io::copy(&e.sketch,&[target]);
    let copied = clip.faces.iter().position(|f| f.on().is_some()).unwrap();
    assert_eq!(clip.edges.len(),4);
    assert!(SpatialFaceBoundary::named(&clip,copied,&parameters(&clip),controls()).is_ok());
    let mut pasted = e.sketch.clone();
    io::paste(&mut pasted,&clip,0.,0.);
    near(SpatialFaceBoundary::named(&pasted,pasted.faces.len()-1,&parameters(&pasted),controls())
        .unwrap().sample_boundary(1,0.5,100).unwrap().position,
        original.sample_boundary(1,0.5,100).unwrap().position);
    let removed = io::without(&e.sketch,&[e.map.ent_named("top_edge").unwrap()],&[]);
    assert_eq!(removed.faces.iter().filter(|f| f.on().is_some()).count(),1);
    let removed = io::without(&e.sketch,&[e.map.ent_named("first_envelope").unwrap()],&[]);
    assert!(removed.faces.iter().all(|f| f.on().is_none()));
    let (mut program,errors) = syntax::parse(FACES);
    assert!(errors.is_empty());
    let printed = syntax::render_flat(&mut program).unwrap();
    let again = solved(&format!("{}{printed}",without_faces()));
    assert_eq!(again.sketch.faces.iter().filter(|f| f.on().is_some()).count(),2);
    let lower = again.map.ent_named("lower_face").unwrap();
    assert!(again.sketch.roles_of(lower).construction);
    // Snapshot domains survive subsequent source changes.
    let surface = e.map.ent_named("second_surface").unwrap().i();
    e.sketch.surfaces[surface].span = Some([0.2,0.4]);
    assert!(SpatialFaceBoundary::named(&e.sketch,target.i(),&parameters(&e.sketch),controls()).is_err());
    assert!(original.sample_boundary(1,0.5,100).is_ok());
}

#[test]
fn spatial_faces_reject_wrong_support_identity_open_loops_and_planar_sweep_use() {
    for declaration in [
        "face bad(left_low,middle_edge,right_low,bottom_edge,on: second_envelope)",
        "face bad(left_low,middle_edge,right_high,bottom_edge,on: first_envelope)",
        "face bad(left_low,middle_edge,right_low,on: first_envelope)",
        "face bad(left_low,middle_edge,right_low,left_low,on: first_envelope)",
        "face bad(left_low,middle_edge,right_low,bottom_edge,on: axis)",
        "face bad(left_low,middle_edge,right_low,bottom_edge,on: first_envelope,on: first_envelope)",
        "face bad(left_low,middle_edge,right_low,bottom_edge,on: first_envelope,-> close)",
        "face bad(left_low,middle_edge,right_low,bottom_edge,on: first_envelope,holes: sphere)",
        "solid bad(lower_face,depth: 1mm)",
        "solid bad(lower_face,about: axis)",
        "solid bad(lower_face,along: axis)",
        "solid bad(face(left_low,middle_edge,right_low,bottom_edge,on: first_envelope),depth: 1mm)",
    ] {
        let (p,errors) = syntax::parse(&format!("{}{declaration}\n",source()));
        assert!(!errors.is_empty() || !program::elaborate(&p).ok(),"accepted {declaration}");
    }
    let e = solved(&source());
    let mut sk = e.sketch.clone();
    let index = e.map.ent_named("lower_face").unwrap().i();
    let body = e.map.ent_named("body").unwrap().i();
    if let gcs_core::model::SolidDef::Revolve {face,..} = &mut sk.solids[body].def { *face = index as u32; }
    assert!(gcs_core::solid::validate(&sk,body).is_err());
    assert!(gcs_core::solid::face_poly(&sk,index,1.).is_none());
    // A separately declared coincident support still has different identity.
    let src = format!("{}\nenvelope lookalike(first_surface,under: roll,from: -20deg,to: 20deg)\n\
        face bad(left_low,middle_edge,right_low,bottom_edge,on: lookalike)\n",source());
    assert!(build(&src).errors().any(|d| d.message.contains("exact named support")));
    assert!(SpatialFaceBoundary::named(&e.sketch,index,&[],controls()).is_err());
    let mut invalid = controls(); invalid.point.incidence = f64::NAN;
    assert!(SpatialFaceBoundary::named(&e.sketch,index,&parameters(&e.sketch),invalid).is_err());
    let face = face(&e,"lower_face");
    assert_eq!(face.sample_boundary(99,0.5,100).unwrap_err(),Error::InvalidOptions);
    assert_eq!(face.sample_boundary(2,-f64::from_bits(1),100).unwrap_err(),Error::OutsideDomain);
    assert_eq!(face.sample_boundary(2,f64::NAN,100).unwrap_err(),Error::NonFinite);
    assert_eq!(face.sample_boundary(2,0.,0).unwrap_err(),Error::InvalidOptions);
}

#[test]
fn spatial_face_support_paths_formals_and_privacy_use_ordinary_component_rules() {
    let src = format!("component F(a: edge,b: edge,c: edge,d: edge,s: envelope) {{\n\
        private face hidden(a,b,c,d,on: s)\nface public(a,b,c,d,on: hidden.on)\n}}\n\
        instance: F(left_low,middle_edge,right_low,bottom_edge,first_envelope)\n{}",source());
    let e = solved(&src);
    let face = e.map.ent_named("instance.public").unwrap();
    assert_eq!(e.sketch.faces[face.i()].support,FaceSupport::Surface(e.map.ent_named("first_envelope").unwrap()));
    assert!(e.map.entity_path(&e.sketch,"instance.hidden").is_none());
    assert!(build(&format!("{src}\nface leak(left_low,middle_edge,right_low,bottom_edge,on: instance.hidden.on)\n"))
        .errors().any(|d| d.message.contains("private member")));
    let clip = io::copy(&e.sketch,&[EntRef::face(face.i())]);
    assert_eq!(clip.faces.iter().filter(|f| f.on().is_some()).count(),1);
}

#[test]
fn surface_supported_boundaries_check_incidence_without_asserting_a_disk_interior() {
    // Two distinct edges trace the same arc in opposite directions. Their loop
    // has valid declared incidence but no disk interior: this reader must not be
    // confused with the still-separate geometric face/solid validity check.
    let e = solved(&format!("{}\nedge other_bottom(low_cut,from: bl,to: br,along: axis)\n\
        face loop_only(bottom_edge,other_bottom,on: lower.wall)\n",source()));
    let face = face(&e,"loop_only");
    let support = gcs_core::solid::RevolvedSurface::named(&e.sketch,face.support().i()).unwrap();
    for fraction in [0.,0.25,0.5,0.75,1.] {
        let a = face.sample_boundary(0,fraction,100).unwrap();
        let b = face.sample_boundary(1,1.-fraction,100).unwrap();
        assert_eq!(a.position,b.position);
        assert_eq!(a.parameters[2],0.);
        near(support.at(a.parameters[0],a.parameters[1]).unwrap().position,a.position);
        assert!(a.incidence_error <= controls().point.incidence);
    }
}
